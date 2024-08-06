//! Delay providers
//!
//! There are currently two delay providers. In general you should prefer to use
//! [Delay], however if you do not have access to `SYST` you can use
//! [DelayFromCountDownTimer] with any timer that implements the [CountDown]
//! trait. This can be useful if you're using [RTIC](https://rtic.rs)'s schedule
//! API, which occupies the `SYST` peripheral.
//!
//! # Examples
//!
//! ## Delay
//!
//! ```no_run
//! let mut delay = Delay::new(core.SYST, device.clocks);
//!
//! delay.delay_ms(500);
//!
//! // Release SYST from the delay
//! let syst = delay.free();
//! ```
//!
//! ## DelayFromCountDownTimer
//!
//! ```no_run
//! let timer2 = device
//!     .TIM2
//!     .timer(1.kHz(), device.peripheral.TIM2, &mut device.clocks);
//! let mut delay = DelayFromCountDownTimer::new(timer2);
//!
//! delay.delay_ms(500);
//!
//! // Release the timer from the delay
//! let timer2 = delay.free();
//! ```
//!
//! # Examples
//!
//! - [Blinky](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/blinky.rs)

use cast::u32;
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m::peripheral::SYST;
use void::Void;

use crate::nb::block;
use crate::rcc::CoreClocks;
use crate::time::Hertz;
use fugit::RateExtU32;

pub trait DelayExt {
    fn delay(self, clocks: CoreClocks) -> Delay;
}

impl DelayExt for SYST {
    fn delay(self, clocks: CoreClocks) -> Delay {
        Delay::new(self, clocks)
    }
}

/// System timer (SysTick) as a delay provider
pub struct Delay {
    clocks: CoreClocks,
    syst: SYST,
}

/// Implements [CountDown] for the System timer (SysTick).
pub struct Countdown<'a> {
    clocks: CoreClocks,
    syst: &'a mut SYST,
    total_rvr: u64,
    finished: bool,
}

impl<'a> Countdown<'a> {
    /// Create a new [CountDown] measured in microseconds.
    pub fn new(syst: &'a mut SYST, clocks: CoreClocks) -> Self {
        Self {
            syst,
            clocks,
            total_rvr: 0,
            finished: true,
        }
    }

    /// start a wait cycle and sets finished to true if [CountdownUs] is done waiting.
    fn start_wait(&mut self) {
        // The SysTick Reload Value register supports values between 1 and 0x00FFFFFF.
        const MAX_RVR: u32 = 0x00FF_FFFF;

        if self.total_rvr != 0 {
            self.finished = false;
            let current_rvr = if self.total_rvr <= MAX_RVR.into() {
                self.total_rvr as u32
            } else {
                MAX_RVR
            };

            self.syst.set_reload(current_rvr);
            self.syst.clear_current();
            self.syst.enable_counter();

            self.total_rvr -= current_rvr as u64;
        } else {
            self.finished = true;
        }
    }
}

impl<'a> embedded_hal_02::timer::CountDown for Countdown<'a> {
    type Time = fugit::MicrosDurationU32;

    fn start<T>(&mut self, count: T)
    where
        T: Into<Self::Time>,
    {
        let us = count.into().ticks();

        // With c_ck up to 480e6, we need u64 for delays > 8.9s

        self.total_rvr = if cfg!(not(feature = "revision_v")) {
            // See errata ES0392 §2.2.3. Revision Y does not have the /8 divider
            u64::from(us) * u64::from(self.clocks.c_ck().raw() / 1_000_000)
        } else if cfg!(feature = "cm4") {
            // CM4 dervived from HCLK
            u64::from(us) * u64::from(self.clocks.hclk().raw() / 8_000_000)
        } else {
            // Normally divide by 8
            u64::from(us) * u64::from(self.clocks.c_ck().raw() / 8_000_000)
        };

        self.start_wait();
    }

    fn wait(&mut self) -> nb::Result<(), Void> {
        if self.finished {
            return Ok(());
        }

        if self.syst.has_wrapped() {
            self.syst.disable_counter();
            self.start_wait();
        }

        Err(nb::Error::WouldBlock)
    }
}

impl Delay {
    /// Configures the system timer (SysTick) as a delay provider
    pub fn new(mut syst: SYST, clocks: CoreClocks) -> Self {
        syst.set_clock_source(SystClkSource::External);

        Delay { clocks, syst }
    }

    /// Releases the system timer (SysTick) resource
    pub fn free(self) -> SYST {
        self.syst
    }
}

impl embedded_hal_02::blocking::delay::DelayMs<u32> for Delay {
    fn delay_ms(&mut self, ms: u32) {
        use embedded_hal_02::blocking::delay::DelayUs;
        self.delay_us(ms * 1_000);
    }
}

impl embedded_hal_02::blocking::delay::DelayMs<u16> for Delay {
    fn delay_ms(&mut self, ms: u16) {
        self.delay_ms(u32(ms));
    }
}

impl embedded_hal_02::blocking::delay::DelayMs<u8> for Delay {
    fn delay_ms(&mut self, ms: u8) {
        self.delay_ms(u32(ms));
    }
}

impl embedded_hal_02::blocking::delay::DelayUs<u32> for Delay {
    fn delay_us(&mut self, us: u32) {
        // The SysTick Reload Value register supports values between 1 and 0x00FFFFFF.
        const MAX_RVR: u32 = 0x00FF_FFFF;

        // With c_ck up to 480e6, we need u64 for delays > 8.9s

        let mut total_rvr = if cfg!(not(feature = "revision_v")) {
            // See errata ES0392 §2.2.3. Revision Y does not have the /8 divider
            u64::from(us) * u64::from(self.clocks.c_ck().raw() / 1_000_000)
        } else if cfg!(feature = "cm4") {
            // CM4 derived from HCLK
            u64::from(us) * u64::from(self.clocks.hclk().raw() / 8_000_000)
        } else {
            // Normally divide by 8
            u64::from(us) * u64::from(self.clocks.c_ck().raw() / 8_000_000)
        };

        while total_rvr != 0 {
            let current_rvr = if total_rvr <= MAX_RVR.into() {
                total_rvr as u32
            } else {
                MAX_RVR
            };

            self.syst.set_reload(current_rvr);
            self.syst.clear_current();
            self.syst.enable_counter();

            // Update the tracking variable while we are waiting...
            total_rvr -= u64::from(current_rvr);

            while !self.syst.has_wrapped() {}

            self.syst.disable_counter();
        }
    }
}

impl embedded_hal_02::blocking::delay::DelayUs<u16> for Delay {
    fn delay_us(&mut self, us: u16) {
        self.delay_us(u32(us))
    }
}

impl embedded_hal_02::blocking::delay::DelayUs<u8> for Delay {
    fn delay_us(&mut self, us: u8) {
        self.delay_us(u32(us))
    }
}

/// CountDown Timer as a delay provider
pub struct DelayFromCountDownTimer<T>(T);

impl<T> DelayFromCountDownTimer<T> {
    /// Creates delay provider from a CountDown timer
    pub fn new(timer: T) -> Self {
        Self(timer)
    }

    /// Releases the Timer
    pub fn free(self) -> T {
        self.0
    }
}

macro_rules! impl_delay_from_count_down_timer  {
    ($(($Delay:ident, $delay:ident, $num:expr)),+) => {
        $(

            impl<T> embedded_hal_02::blocking::delay::$Delay<u32> for DelayFromCountDownTimer<T>
            where
                T: embedded_hal_02::timer::CountDown<Time = Hertz>,
            {
                fn $delay(&mut self, t: u32) {
                    let mut time_left = t;

                    // Due to the LpTimer having only a 3 bit scaler, it is
                    // possible that the max timeout we can set is
                    // (128 * 65536) / clk_hz milliseconds.
                    // Assuming the fastest clk_hz = 480Mhz this is roughly ~17ms,
                    // or a frequency of ~57.2Hz. We use a 60Hz frequency for each
                    // loop step here to ensure that we stay within these bounds.
                    let looping_delay = $num / 60;
                    let looping_delay_hz = Hertz::from_raw($num / looping_delay);

                    self.0.start(looping_delay_hz);
                    while time_left > looping_delay {
                        block!(self.0.wait()).ok();
                        time_left -= looping_delay;
                    }

                    if time_left > 0 {
                        self.0.start(($num / time_left).Hz());
                        block!(self.0.wait()).ok();
                    }
                }
            }

            impl<T> embedded_hal_02::blocking::delay::$Delay<u16> for DelayFromCountDownTimer<T>
            where
                T: embedded_hal_02::timer::CountDown<Time = Hertz>,
            {
                fn $delay(&mut self, t: u16) {
                    self.$delay(t as u32);
                }
            }

            impl<T> embedded_hal_02::blocking::delay::$Delay<u8> for DelayFromCountDownTimer<T>
            where
                T: embedded_hal_02::timer::CountDown<Time = Hertz>,
            {
                fn $delay(&mut self, t: u8) {
                    self.$delay(t as u32);
                }
            }
        )+
    }
}

impl_delay_from_count_down_timer! {
    (DelayMs, delay_ms, 1_000),
    (DelayUs, delay_us, 1_000_000)
}
