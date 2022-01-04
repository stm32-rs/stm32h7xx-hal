//! Real-Time Clock
//!
//! # Examples
//!
//! - [RTC example](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/rtc.rs)

use cast::{f32, i32, u16, u32, u8};
use chrono::prelude::*;

use crate::exti::{Event as ExtiEvent, ExtiExt};
use crate::rcc::backup;
use crate::rcc::rec::ResetEnable;
use crate::rcc::CoreClocks;
use crate::stm32::{EXTI, RCC, RTC};
use crate::time::Hertz;

pub enum Event {
    AlarmA,
    AlarmB,
    Wakeup,
    Timestamp,
    LseCss,
}

#[derive(Copy, Clone, PartialEq)]
pub enum DstState {
    /// Standard Time
    Standard = 0,
    /// Daylight Savings Time
    ///
    /// One hour ahead of Standard Time
    Dst = 1,
}

#[derive(Copy, Clone, PartialEq)]
pub enum RtcClock {
    /// LSE (Low-Speed External)
    ///
    /// This is in the Backup power domain, and so it can
    /// remain operational as long as VBat is present.
    Lse {
        freq: Hertz,
        bypass: bool,
        /// Enable the Clock Security Subsystem for the LSE
        ///
        /// Note that errata 2.2.19 for STM32H74x prevents the LSE from initializing without resetting after VDD
        /// power-up.
        css: bool,
    },
    /// LSI (Low-Speed Internal)
    ///
    /// This clock remains functional in Stop or Standby mode,
    /// but requires VDD to remain powered. LSI is an RC
    /// oscillator and has poor accuracy.
    Lsi,
    /// HSE (High-Speed External) divided by 2..=63
    ///
    /// The resulting clock must be lower than 1MHz. This clock is
    /// automatically disabled by hardware when the CPU enters Stop or
    /// standby mode.
    Hse { divider: u8 },
}

#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// An error preventing the RTC from initializing
pub enum InitError {
    RtcNotRunning,
    ClockNotRunning,
    ConfigMismatch,
}

#[derive(Debug, Copy, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DstError {
    ClockNotInitialized,
    AlreadyDst,
    AlreadyStandardTime,
    CannotSubtract,
}

/// Real-Time Clock
pub struct Rtc {
    reg: RTC,
    prec: backup::Rtc,
}

impl Rtc {
    /// Opens the RTC if it is running and its configuration matches, otherwise resets and inits the RTC
    pub fn open_or_init(
        rtc: RTC,
        prec: backup::Rtc,
        clock_source: RtcClock,
        clocks: &CoreClocks,
    ) -> Self {
        match Rtc::try_open(rtc, prec, clock_source, clocks) {
            Ok(rtc) => rtc,
            Err((rtc, prec, _err)) => {
                Rtc::init(rtc, prec, clock_source, clocks)
            }
        }
    }

    /// Opens the RTC if it is running and its configuration matches
    pub fn try_open(
        rtc: RTC,
        prec: backup::Rtc,
        clock_source: RtcClock,
        clocks: &CoreClocks,
    ) -> Result<Self, (RTC, backup::Rtc, InitError)> {
        if !prec.is_enabled() {
            return Err((rtc, prec, InitError::RtcNotRunning));
        }

        let rcc = unsafe { &*RCC::ptr() };
        let bdcr = rcc.bdcr.read();

        let clock_source_matches =
            match (clock_source, prec.get_kernel_clk_mux()) {
                (RtcClock::Lsi, backup::RtcClkSel::LSI) => true,
                (RtcClock::Hse { divider }, backup::RtcClkSel::HSE) => {
                    rcc.cfgr.read().rtcpre().bits() == divider
                }
                (RtcClock::Lse { bypass, css, .. }, backup::RtcClkSel::LSE) => {
                    bdcr.lseon().is_on()
                        && bypass == bdcr.lsebyp().is_bypassed()
                        && css == bdcr.lsecsson().is_security_on()
                }
                _ => false,
            };
        if !clock_source_matches {
            return Err((rtc, prec, InitError::ConfigMismatch));
        }

        let clock_source_running = match clock_source {
            RtcClock::Lsi => clocks.lsi_ck().is_some(),
            RtcClock::Hse { .. } => clocks.hse_ck().is_some(),
            RtcClock::Lse { .. } => bdcr.lserdy().is_ready(),
        };
        if !clock_source_running {
            return Err((rtc, prec, InitError::ClockNotRunning));
        }

        Ok(Rtc { reg: rtc, prec })
    }

    /// Resets the RTC, including the backup registers, then initializes it.
    pub fn init(
        rtc: RTC,
        prec: backup::Rtc,
        clock_source: RtcClock,
        clocks: &CoreClocks,
    ) -> Self {
        let mut prec = prec.reset().enable();

        let rcc = unsafe { &*RCC::ptr() };

        // Check and configure clock source
        let ker_ck = match clock_source {
            RtcClock::Lse { bypass, freq, .. } => {
                // Set LSEBYP before enabling
                rcc.bdcr.modify(|_, w| w.lsebyp().bit(bypass));
                // Ensure LSE is on and stable
                rcc.bdcr.modify(|_, w| w.lseon().on());
                while rcc.bdcr.read().lserdy().is_not_ready() {}

                Some(freq)
            }
            RtcClock::Hse { divider } => {
                // Set HSE divider
                assert!(divider < 64, "HSE Divider larger than 63");
                rcc.cfgr.modify(|_, w| w.rtcpre().bits(divider));

                clocks.hse_ck().map(|x| Hertz(x.0 / u32(divider)))
            }
            RtcClock::Lsi => clocks.lsi_ck(),
        }
        .expect("rtc_ker_ck not running")
        .0;

        assert!(ker_ck <= 1 << 22, "rtc_ker_ck too fast for prescaler");

        // Select RTC kernel clock
        prec.kernel_clk_mux(match clock_source {
            RtcClock::Hse { .. } => backup::RtcClkSel::HSE,
            RtcClock::Lsi => backup::RtcClkSel::LSI,
            RtcClock::Lse { .. } => backup::RtcClkSel::LSE,
        });

        // Now we can enable CSS, if required
        if let RtcClock::Lse { css: true, .. } = clock_source {
            rcc.bdcr.modify(|_, w| w.lsecsson().security_on());
        }

        // Disable RTC register write protection
        rtc.wpr.write(|w| unsafe { w.bits(0xCA) });
        rtc.wpr.write(|w| unsafe { w.bits(0x53) });

        // Enter initialization mode
        rtc.isr.modify(|_, w| w.init().set_bit());
        while rtc.isr.read().initf().bit_is_clear() {}

        // Enable Shadow Register Bypass
        rtc.cr.modify(|_, w| w.bypshad().set_bit());

        // Configure prescaler for 1Hz clock
        // Want to maximize a_pre_max for power reasons, though it reduces the
        // subsecond precision.
        let total_div = ker_ck;
        let a_pre_max = 1 << 7;
        let s_pre_max = 1 << 15;

        let (a_pre, s_pre) = if total_div <= a_pre_max {
            (total_div, 1)
        } else if total_div % a_pre_max == 0 {
            (a_pre_max, total_div / a_pre_max)
        } else {
            let mut a_pre = a_pre_max;
            while a_pre > 1 {
                if total_div % a_pre == 0 {
                    break;
                }
                a_pre -= 1;
            }
            let s_pre = total_div / a_pre;
            (a_pre, s_pre)
        };
        assert!(
            a_pre <= a_pre_max && s_pre <= s_pre_max,
            "Invalid RTC prescaler value"
        );

        rtc.prer.write(|w| {
            w.prediv_s()
                .bits(u16(s_pre - 1).unwrap())
                .prediv_a()
                .bits(u8(a_pre - 1).unwrap())
        });

        // Exit initialization mode
        rtc.isr.modify(|_, w| w.init().clear_bit());

        Rtc { reg: rtc, prec }
    }

    /// Reads the value of a 32-bit backup register
    ///
    /// # Panics
    ///
    /// Panics if `reg` is greater than 31.
    pub fn read_backup_reg(&self, reg: u8) -> u32 {
        self.reg.bkpr[reg as usize].read().bkp().bits()
    }

    /// Writes `value` to a 32-bit backup register
    ///
    /// # Panics
    ///
    /// Panics if `reg` is greater than 31.
    pub fn write_backup_reg(&mut self, reg: u8, value: u32) {
        self.reg.bkpr[reg as usize].write(|w| w.bkp().bits(value));
    }

    /// Sets the date and time of the RTC
    ///
    /// # Panics
    ///
    /// Panics if `date_time` is before 01-01-2001 or after 12-31-2099
    /// when debug assertions are enabled.
    pub fn set_date_time(&mut self, date_time: NaiveDateTime) {
        // Enter initialization mode
        self.reg.isr.modify(|_, w| w.init().set_bit());
        while self.reg.isr.read().initf().bit_is_clear() {}

        let hour = date_time.hour() as u8;
        let ht = hour / 10;
        let hu = hour % 10;

        let minute = date_time.minute() as u8;
        let mnt = minute / 10;
        let mnu = minute % 10;

        let second = date_time.second() as u8;
        let st = second / 10;
        let su = second % 10;

        self.reg.tr.write(|w| {
            w.pm()
                .clear_bit()
                .ht()
                .bits(ht)
                .hu()
                .bits(hu)
                .mnt()
                .bits(mnt)
                .mnu()
                .bits(mnu)
                .st()
                .bits(st)
                .su()
                .bits(su)
        });

        let year = date_time.year();
        debug_assert!(year > 2000 && year < 2100);
        let yt = ((year - 2000) / 10) as u8;
        let yu = ((year - 2000) % 10) as u8;

        let wdu = date_time.weekday().number_from_monday() as u8;

        let month = date_time.month() as u8;
        let mt = month > 9;
        let mu = month % 10;

        let day = date_time.day() as u8;
        let dt = day / 10;
        let du = day % 10;

        self.reg.dr.write(|w| unsafe {
            w.yt()
                .bits(yt)
                .yu()
                .bits(yu)
                .wdu()
                .bits(wdu)
                .mt()
                .bit(mt)
                .mu()
                .bits(mu)
                .dt()
                .bits(dt)
                .du()
                .bits(du)
        });

        // Exit initialization mode
        self.reg.isr.modify(|_, w| w.init().clear_bit());
    }

    /// De-initializes the calendar and clock
    ///
    /// For when you lose confidince in the stored time e.g. if the LSE clock fails.
    pub fn clear_date_time(&mut self) {
        // Enter initialization mode
        self.reg.isr.modify(|_, w| w.init().set_bit());
        while self.reg.isr.read().initf().bit_is_clear() {}

        self.reg.tr.reset();
        self.reg.dr.reset();

        // Exit initialization mode
        self.reg.isr.modify(|_, w| w.init().clear_bit());
    }

    /// Returns `None` if the calendar is uninitialized
    fn calendar_initialized(&self) -> Option<()> {
        match self.reg.isr.read().inits().bit() {
            true => Some(()),
            false => None,
        }
    }

    /// Calendar Date
    ///
    /// Returns `None` if the calendar has not been initialized
    pub fn date(&self) -> Option<NaiveDate> {
        self.calendar_initialized()?;
        let data = self.reg.dr.read();
        let year = 2000 + i32(data.yt().bits()) * 10 + i32(data.yu().bits());
        let month = data.mt().bits() as u8 * 10 + data.mu().bits();
        let day = data.dt().bits() * 10 + data.du().bits();
        NaiveDate::from_ymd_opt(year, u32(month), u32(day))
    }

    /// Current Time
    ///
    /// Returns `None` if the calendar has not been initialized
    pub fn time(&self) -> Option<NaiveTime> {
        loop {
            self.calendar_initialized()?;
            let ss = self.reg.ssr.read().ss().bits();
            let data = self.reg.tr.read();

            // If an RTCCLK edge occurs during read we may see inconsistent values
            // so read ssr again and see if it has changed. (see RM0433 Rev 7 46.3.9)
            let ss_after = self.reg.ssr.read().ss().bits();
            if ss == ss_after {
                let mut hour = data.ht().bits() * 10 + data.hu().bits();
                if data.pm().bit_is_set() {
                    hour += 12;
                }
                let minute = data.mnt().bits() * 10 + data.mnu().bits();
                let second = data.st().bits() * 10 + data.su().bits();
                let micro = self.ss_to_us(ss);

                return NaiveTime::from_hms_micro_opt(
                    u32(hour),
                    u32(minute),
                    u32(second),
                    micro,
                );
            }
        }
    }

    /// Calendar Date and Time
    ///
    /// Returns `None` if the calendar has not been initialized
    pub fn date_time(&self) -> Option<NaiveDateTime> {
        loop {
            self.calendar_initialized()?;
            let ss = self.reg.ssr.read().ss().bits();
            let dr = self.reg.dr.read();
            let tr = self.reg.tr.read();

            // If an RTCCLK edge occurs during read we may see inconsistent values
            // so read ssr again and see if it has changed. (see RM0433 Rev 7 46.3.9)
            let ss_after = self.reg.ssr.read().ss().bits();
            if ss == ss_after {
                let year =
                    2000 + i32(dr.yt().bits()) * 10 + i32(dr.yu().bits());
                let month = dr.mt().bits() as u8 * 10 + dr.mu().bits();
                let day = dr.dt().bits() * 10 + dr.du().bits();

                let date = NaiveDate::from_ymd_opt(year, u32(month), u32(day))?;

                let mut hour = tr.ht().bits() * 10 + tr.hu().bits();
                if tr.pm().bit_is_set() {
                    hour += 12;
                }
                let minute = tr.mnt().bits() * 10 + tr.mnu().bits();
                let second = tr.st().bits() * 10 + tr.su().bits();
                let micro = self.ss_to_us(ss);

                let time = NaiveTime::from_hms_micro_opt(
                    u32(hour),
                    u32(minute),
                    u32(second),
                    micro,
                )?;

                return Some(date.and_time(time));
            }
        }
    }

    /// Returns the fraction of seconds that have occurred since the last second tick
    ///
    /// The precision of this value depends on the value of the synchronous prescale divider
    /// (prediv_s) which depends on the frequency of the RTC clock. E.g. with a 32,768 Hz
    /// crystal this value has a resolution of 1/256 of a second.
    pub fn subseconds(&self) -> Option<f32> {
        self.calendar_initialized()?;
        let ss = f32(self.reg.ssr.read().ss().bits());
        let prediv_s = f32(self.reg.prer.read().prediv_s().bits());
        Some((prediv_s - ss) / (prediv_s + 1.0))
    }

    fn ss_to_us(&self, ss: u16) -> u32 {
        let ss = u32(ss);
        let prediv_s = u32(self.reg.prer.read().prediv_s().bits());
        assert!(ss <= prediv_s); // See RM0433 Rev 7 46.6.10, shift operations not supported

        // Multiplying (prediv_s - ss) by 1,000,000 could overflow a u32 if prediv_s is large enough.
        // u64 division would call `__aeabi_uldivmod` which is large and slow.
        // Our maximum resolution is 1/32768 seconds or 32.5 us, so we can get away with rounding to
        // the nearest 10 to avoid overflow.
        (((prediv_s - ss) * 100_000) / (prediv_s + 1)) * 10
    }

    /// Returns the fraction of seconds that have occurred since the last second tick
    /// as a number of milliseconds rounded to the nearest whole number.
    pub fn subsec_millis(&self) -> Option<u16> {
        self.calendar_initialized()?;
        let ss = u32(self.reg.ssr.read().ss().bits());
        let prediv_s = u32(self.reg.prer.read().prediv_s().bits());
        Some(u16(((prediv_s - ss) * 1_000) / (prediv_s + 1)).unwrap())
    }

    /// Returns the fraction of seconds that have occurred since the last second tick
    /// as a number of microseconds rounded to the nearest whole number.
    pub fn subsec_micros(&self) -> Option<u32> {
        self.calendar_initialized()?;
        let ss = self.reg.ssr.read().ss().bits();
        Some(self.ss_to_us(ss))
    }

    /// Returns the raw value of the synchronous subsecond counter
    ///
    /// This counts up to `self.subsec_res()` then resets to zero once per second.
    pub fn subsec_raw(&self) -> Option<u16> {
        self.calendar_initialized()?;
        Some(self.reg.ssr.read().ss().bits())
    }

    /// Returns the resolution of subsecond values
    ///
    /// The RTC counter increments this number of times per second.
    pub fn subsec_res(&self) -> Option<u16> {
        self.calendar_initialized()?;
        Some(self.reg.prer.read().prediv_s().bits())
    }

    /// Returns the stored Daylight Savings Time status
    pub fn dst(&self) -> DstState {
        if self.reg.cr.read().bkp().bit_is_set() {
            DstState::Dst
        } else {
            DstState::Standard
        }
    }

    /// Sets the stored Daylight Savings Time status without adjusting the clock
    pub fn set_dst(&mut self, dst: DstState) {
        self.reg.cr.modify(|_, w| w.bkp().bit(dst == DstState::Dst));
    }

    /// Begin Daylight Savings Time
    ///
    /// Adds an hour to the stored time and sets the stored DST status.
    /// Returns an error if the clock has not been initialized, or if the stored DST status
    /// indicates it has already begun.
    pub fn begin_dst(&mut self) -> Result<(), DstError> {
        self.calendar_initialized()
            .ok_or(DstError::ClockNotInitialized)?;
        if self.reg.cr.read().bkp().bit_is_set() {
            return Err(DstError::AlreadyDst);
        }
        self.reg
            .cr
            .modify(|_, w| w.add1h().set_bit().bkp().set_bit());
        Ok(())
    }

    /// End Daylight Savings Time
    ///
    /// Subtracts an hour from the stored time and sets the stored DST status.
    /// Returns an error if the clock has not been initialized, if the stored DST status
    /// indicates it is already standard time, or if we cannot subtract an hour because
    /// it is not at least one hour past midnight.
    pub fn end_dst(&mut self) -> Result<(), DstError> {
        self.calendar_initialized()
            .ok_or(DstError::ClockNotInitialized)?;
        if self.reg.cr.read().bkp().bit_is_clear() {
            return Err(DstError::AlreadyStandardTime);
        }
        let time = self.reg.tr.read();
        if time.ht().bits() == 0 && time.hu().bits() == 0 {
            return Err(DstError::CannotSubtract);
        }
        self.reg
            .cr
            .modify(|_, w| w.sub1h().set_bit().bkp().clear_bit());
        Ok(())
    }

    /// Configures the wakeup timer to trigger periodically every `interval` seconds
    ///
    /// # Panics
    ///
    /// Panics if interval is greater than 2¹⁷-1.
    pub fn enable_wakeup(&mut self, interval: u32) {
        self.reg.cr.modify(|_, w| w.wute().clear_bit());
        self.reg.isr.modify(|_, w| w.wutf().clear_bit());
        while self.reg.isr.read().wutwf().bit_is_clear() {}

        if interval > 1 << 16 {
            self.reg
                .cr
                .modify(|_, w| unsafe { w.wucksel().bits(0b110) });
            let interval = u16(interval - (1 << 16) - 1)
                .expect("Interval was too large for wakeup timer");
            self.reg.wutr.write(|w| w.wut().bits(interval));
        } else {
            self.reg
                .cr
                .modify(|_, w| unsafe { w.wucksel().bits(0b100) });
            let interval = u16(interval - 1)
                .expect("Interval was too large for wakeup timer");
            self.reg.wutr.write(|w| w.wut().bits(interval));
        }

        self.reg.cr.modify(|_, w| w.wute().set_bit());
    }

    /// Disables the wakeup timer
    pub fn disable_wakeup(&mut self) {
        self.reg.cr.modify(|_, w| w.wute().clear_bit());
        self.reg.isr.modify(|_, w| w.wutf().clear_bit());
    }

    /// Configures the timestamp to be captured when the RTC switches to Vbat power
    pub fn enable_vbat_timestamp(&mut self) {
        self.reg.cr.modify(|_, w| w.tse().clear_bit());
        self.reg.isr.modify(|_, w| w.tsf().clear_bit());
        self.reg.cr.modify(|_, w| w.itse().set_bit());
        self.reg.cr.modify(|_, w| w.tse().set_bit());
    }

    /// Disables the timestamp
    pub fn disable_timestamp(&mut self) {
        self.reg.cr.modify(|_, w| w.tse().clear_bit());
        self.reg.isr.modify(|_, w| w.tsf().clear_bit());
    }

    /// Reads the stored value of the timestamp if present
    ///
    /// Clears the timestamp interrupt flags.
    pub fn read_timestamp(&self) -> Option<NaiveDateTime> {
        if !self.reg.isr.read().tsf().bit_is_clear() {
            return None;
        }

        // Timestamp doesn't include year, get it from the main calendar
        let data = self.reg.dr.read();
        let year = 2000 + i32(data.yt().bits()) * 10 + i32(data.yu().bits());

        let data = self.reg.tsdr.read();
        let month = data.mt().bits() as u8 * 10 + data.mu().bits();
        let day = data.dt().bits() * 10 + data.du().bits();
        let date = NaiveDate::from_ymd_opt(year, u32(month), u32(day))?;

        let data = self.reg.tstr.read();
        let mut hour = data.ht().bits() * 10 + data.hu().bits();
        if data.pm().bit_is_set() {
            hour += 12; // Shouldn't be configured this way, but handle it anyway
        }
        let minute = data.mnt().bits() * 10 + data.mnu().bits();
        let second = data.st().bits() * 10 + data.su().bits();
        let micro = self.ss_to_us(self.reg.tsssr.read().ss().bits());
        let time = NaiveTime::from_hms_micro_opt(
            u32(hour),
            u32(minute),
            u32(second),
            u32(micro),
        )?;

        // Clear timestamp interrupt and internal timestamp interrupt (VBat transition)
        // TODO: Timestamp overflow flag
        self.reg
            .isr
            .modify(|_, w| w.tsf().clear_bit().itsf().clear_bit());

        Some(date.and_time(time))
    }

    // TODO: Alarms

    /// Start listening for `event`
    pub fn listen(&mut self, exti: &mut EXTI, event: Event) {
        // RTC interrupts aren't connected directly to the NVIC but are routed through the EXTI,
        // I suppose so that they can use the EXTI's ability to wakeup the CPU or other things.
        // We need to also enable the interrupt in the EXTI.
        //
        // Input Mapping (RM0433 Rev 7 Section 20.4):
        // EXTI 17 = RTC Alarms
        // EXTI 18 = RTC Tamper, RTC Timestamp, RTC LSECSS
        // EXTI 19 = RTC Wakeup Timer

        // unsafe: Only we can use these bits for anything
        let rcc = unsafe { &*RCC::ptr() };

        match event {
            Event::LseCss => {
                exti.listen(ExtiEvent::RTC_OTHER);
                exti.rtsr1.modify(|_, w| w.tr18().enabled());
                rcc.cier.modify(|_, w| w.lsecssie().enabled());
            }
            Event::AlarmA => {
                exti.listen(ExtiEvent::RTC_ALARM);
                exti.rtsr1.modify(|_, w| w.tr17().enabled());
                self.reg.cr.modify(|_, w| w.alraie().set_bit());
            }
            Event::AlarmB => {
                exti.listen(ExtiEvent::RTC_ALARM);
                exti.rtsr1.modify(|_, w| w.tr17().enabled());
                self.reg.cr.modify(|_, w| w.alrbie().set_bit());
            }
            Event::Wakeup => {
                exti.listen(ExtiEvent::RTC_WAKEUP);
                exti.rtsr1.modify(|_, w| w.tr19().enabled());
                self.reg.cr.modify(|_, w| w.wutie().set_bit());
            }
            Event::Timestamp => {
                exti.listen(ExtiEvent::RTC_OTHER);
                exti.rtsr1.modify(|_, w| w.tr18().enabled());
                self.reg.cr.modify(|_, w| w.tsie().set_bit());
            }
        }
    }

    /// Stop listening for `event`
    pub fn unlisten(&mut self, exti: &mut EXTI, event: Event) {
        // See the note in listen() about EXTI
        // unsafe: Only we can use these bits for anything
        let rcc = unsafe { &*RCC::ptr() };

        match event {
            Event::LseCss => {
                rcc.cier.modify(|_, w| w.lsecssie().disabled());
                exti.unlisten(ExtiEvent::RTC_OTHER);
                exti.rtsr1.modify(|_, w| w.tr18().disabled());
            }
            Event::AlarmA => {
                self.reg.cr.modify(|_, w| w.alraie().clear_bit());
                exti.unlisten(ExtiEvent::RTC_ALARM);
                exti.rtsr1.modify(|_, w| w.tr17().disabled());
            }
            Event::AlarmB => {
                self.reg.cr.modify(|_, w| w.alrbie().clear_bit());
                exti.unlisten(ExtiEvent::RTC_ALARM);
                exti.rtsr1.modify(|_, w| w.tr17().disabled());
            }
            Event::Wakeup => {
                self.reg.cr.modify(|_, w| w.wutie().clear_bit());
                exti.unlisten(ExtiEvent::RTC_WAKEUP);
                exti.rtsr1.modify(|_, w| w.tr19().disabled());
            }
            Event::Timestamp => {
                self.reg.cr.modify(|_, w| w.tsie().clear_bit());
                exti.unlisten(ExtiEvent::RTC_OTHER);
                exti.rtsr1.modify(|_, w| w.tr18().disabled());
            }
        }
    }

    /// Returns `true` if `event` is pending
    pub fn is_pending(&self, event: Event) -> bool {
        // unsafe: Only we can do anything with this bit
        let rcc = unsafe { &*RCC::ptr() };

        match event {
            Event::LseCss => rcc.cifr.read().lsecssf().bit_is_set(),
            Event::AlarmA => self.reg.isr.read().alraf().bit_is_set(),
            Event::AlarmB => self.reg.isr.read().alrbf().bit_is_set(),
            Event::Wakeup => self.reg.isr.read().wutf().bit_is_set(),
            Event::Timestamp => self.reg.isr.read().tsf().bit_is_set(),
        }
    }

    /// Clears the interrupt flag for `event`
    pub fn unpend(&mut self, exti: &mut EXTI, event: Event) {
        // See the note in listen() about EXTI
        // unsafe: Only we can do anything with these bits
        let rcc = unsafe { &*RCC::ptr() };

        match event {
            Event::LseCss => {
                rcc.cicr.write(|w| w.lsecssc().clear());
                exti.unpend(ExtiEvent::RTC_OTHER);
            }
            Event::AlarmA => {
                self.reg.isr.modify(|_, w| w.alraf().clear_bit());
                exti.unpend(ExtiEvent::RTC_ALARM);
            }
            Event::AlarmB => {
                self.reg.isr.modify(|_, w| w.alrbf().clear_bit());
                exti.unpend(ExtiEvent::RTC_ALARM);
            }
            Event::Wakeup => {
                self.reg.isr.modify(|_, w| w.wutf().clear_bit());
                exti.unpend(ExtiEvent::RTC_WAKEUP);
            }
            Event::Timestamp => {
                self.reg.isr.modify(|_, w| w.tsf().clear_bit());
                exti.unpend(ExtiEvent::RTC_OTHER);
            }
        }
    }

    /// Handle a Clock Security Subsystem failure for the LSE clock
    ///
    /// Disables the LSE, disables the LSE CSS, and changes the RTC to use the LSI clock.
    /// You may want to call `clear_date_time()`. You still need to unpend the LSECSS interrupt.
    pub fn handle_lse_css(&mut self) {
        if !self.is_pending(Event::LseCss) {
            return;
        }

        // unsafe: Only we can use these bits
        let rcc = unsafe { &*RCC::ptr() };
        rcc.bdcr
            .modify(|_, w| w.lsecsson().security_off().lseon().off());

        // We're allowed to change this once after the LSE fails
        self.prec.kernel_clk_mux(backup::RtcClkSel::LSI);
    }

    /// Returns a reference to the inner peripheral
    pub fn inner(&self) -> &RTC {
        &self.reg
    }

    /// Returns a mutable reference to the inner peripheral
    pub fn inner_mut(&mut self) -> &mut RTC {
        &mut self.reg
    }
}
