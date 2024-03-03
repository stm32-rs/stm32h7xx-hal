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

use cortex_m::peripheral::syst::SystClkSource;
use cortex_m::peripheral::SYST;
use void::Void;

use crate::block;
use crate::rcc::CoreClocks;
use crate::stm32::{LPTIM1, LPTIM2, LPTIM3};
#[cfg(not(feature = "rm0455"))]
use crate::stm32::{LPTIM4, LPTIM5};
use crate::stm32::{
    TIM1, TIM12, TIM13, TIM14, TIM15, TIM16, TIM17, TIM2, TIM3, TIM4, TIM5,
    TIM6, TIM7, TIM8,
};
use crate::time::Hertz;
use crate::timer::{Enabled, LpTimer, Timer};

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
    /// Create a new [Countdown] measured in microseconds.
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

impl<'a> Countdown<'a> {
    /// Starts a new count down
    pub fn start<T>(&mut self, count: T)
    where
        T: Into<fugit::MicrosDurationU32>,
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

    /// Non-blockingly “waits” until the count down finishes
    pub fn wait(&mut self) -> nb::Result<(), Void> {
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

impl Delay {
    /// Internal method to delay cycles with systick
    fn systick_delay_cycles(&mut self, mut total_rvr: u64) {
        // The SysTick Reload Value register supports values between 1 and 0x00FFFFFF.
        const MAX_RVR: u32 = 0x00FF_FFFF;

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
    /// Internal method that returns the clock frequency of systick
    fn systick_clock(&self) -> u64 {
        if cfg!(not(feature = "revision_v")) {
            // See errata ES0392 §2.2.3. Revision Y does not have the /8 divider
            u64::from(self.clocks.c_ck().raw())
        } else if cfg!(feature = "cm4") {
            // CM4 derived from HCLK
            u64::from(self.clocks.hclk().raw() / 8)
        } else {
            // Normally divide by 8
            u64::from(self.clocks.c_ck().raw() / 8)
        }
    }
}

impl embedded_hal_1::delay::DelayNs for Delay {
    fn delay_ns(&mut self, ns: u32) {
        // With c_ck up to 480e6, 1 cycle is always > 2ns

        self.systick_delay_cycles(u64::from(ns + 1) / 2);
    }
    fn delay_us(&mut self, us: u32) {
        // With c_ck up to 480e6, we need u64 for delays > 8.9s

        let total_rvr =
            u64::from(us) * ((self.systick_clock() + 999_999) / 1_000_000);
        self.systick_delay_cycles(total_rvr);
    }
}

// embedded_hal_02 implementations
//

impl embedded_hal_02::blocking::delay::DelayMs<u32> for Delay {
    fn delay_ms(&mut self, ms: u32) {
        // With c_ck up to 480e6, we need u64 for delays > 8.9s
        let total_rvr = u64::from(ms) * ((self.systick_clock() + 999) / 1_000);
        self.systick_delay_cycles(total_rvr);
    }
}

impl embedded_hal_02::blocking::delay::DelayMs<u16> for Delay {
    fn delay_ms(&mut self, ms: u16) {
        self.delay_ms(cast::u32(ms));
    }
}

impl embedded_hal_02::blocking::delay::DelayMs<u8> for Delay {
    fn delay_ms(&mut self, ms: u8) {
        self.delay_ms(cast::u32(ms));
    }
}

impl embedded_hal_02::blocking::delay::DelayUs<u32> for Delay {
    fn delay_us(&mut self, us: u32) {
        // With c_ck up to 480e6, we need u64 for delays > 8.9s
        let total_rvr =
            u64::from(us) * ((self.systick_clock() + 999_999) / 1_000_000);
        self.systick_delay_cycles(total_rvr);
    }
}

impl embedded_hal_02::blocking::delay::DelayUs<u16> for Delay {
    fn delay_us(&mut self, us: u16) {
        self.delay_us(cast::u32(us))
    }
}

impl embedded_hal_02::blocking::delay::DelayUs<u8> for Delay {
    fn delay_us(&mut self, us: u8) {
        self.delay_us(cast::u32(us))
    }
}

/// Delay that implements [embedded_hal::blocking::delay] traits
pub struct DelayFromCountDownTimer<TIM>(TIM);

impl<TIM> DelayFromCountDownTimer<TIM> {
    /// Creates a delay from a timer resource
    pub fn new(timer: TIM) -> Self {
        Self(timer)
    }
    /// Free the timer resource for other uses
    pub fn free(self) -> TIM {
        self.0
    }
}

macro_rules! impl_delay_from_count_down_timer  {
    ($($TIMX:ty),+) => {
        $(
            impl DelayFromCountDownTimer<$TIMX>
            {
                fn delay_us_internal(&mut self, t: u32) {
                    let mut time_left = t;

                    // Due to the LpTimer having only a 3 bit scaler, it is
                    // possible that the max timeout we can set is
                    // (128 * 65536) / clk_hz milliseconds.
                    // Assuming the fastest clk_hz = 480Mhz this is roughly ~17ms,
                    // or a frequency of ~57.2Hz. We use a 60Hz frequency for each
                    // loop step here to ensure that we stay within these bounds.
                    let looping_delay = 1_000_000 / 60;
                    let looping_delay_hz = Hertz::from_raw(1_000_000 / looping_delay);

                    let _ = self.0.start(looping_delay_hz); // This method is infallible
                    while time_left > looping_delay {
                        block!(self.0.wait()).ok();
                        time_left -= looping_delay;
                    }

                    if time_left > 0 {
                        // This method is infallible
                        let _ = self.0.start(Hertz::from_raw(1_000_000 / time_left));
                        block!(self.0.wait()).ok();
                    }
                }
            }

            impl embedded_hal_1::delay::DelayNs for DelayFromCountDownTimer<$TIMX> {
                fn delay_ns(&mut self, ns: u32) {
                    // TODO(): This delay is 1000x longer than the intended duration!
                    self.delay_us_internal(ns);
                }
                fn delay_us(&mut self, us: u32) {
                    self.delay_us_internal(us);
                }
            }
            // embedded_hal_02 implementations
            //

            impl embedded_hal_02::blocking::delay::DelayMs<u32>
                for DelayFromCountDownTimer<$TIMX>
            {
                fn delay_ms(&mut self, ms: u32) {
                    for _ in 0..1000 {
                        self.delay_us_internal(ms);
                    }
                }
            }
            impl embedded_hal_02::blocking::delay::DelayMs<u16>
                for DelayFromCountDownTimer<$TIMX>
            {
                fn delay_ms(&mut self, ms: u16) {
                    self.delay_ms(cast::u32(ms));
                }
            }
            impl embedded_hal_02::blocking::delay::DelayMs<u8>
                for DelayFromCountDownTimer<$TIMX>
            {
                fn delay_ms(&mut self, ms: u8) {
                    self.delay_ms(cast::u32(ms));
                }
            }
            impl embedded_hal_02::blocking::delay::DelayUs<u32>
                for DelayFromCountDownTimer<$TIMX>
            {
                fn delay_us(&mut self, us: u32) {
                    self.delay_us_internal(us);
                }
            }
            impl embedded_hal_02::blocking::delay::DelayUs<u16>
                for DelayFromCountDownTimer<$TIMX>
            {
                fn delay_us(&mut self, us: u16) {
                    self.delay_us(cast::u32(us))
                }
            }
            impl embedded_hal_02::blocking::delay::DelayUs<u8>
                for DelayFromCountDownTimer<$TIMX>
            {
                fn delay_us(&mut self, us: u8) {
                    self.delay_us(cast::u32(us))
                }
            }
        )+
    }
}

impl_delay_from_count_down_timer! {
    Timer<TIM1>, Timer<TIM8>,
    Timer<TIM2>, Timer<TIM3>, Timer<TIM4>,
    Timer<TIM5>, Timer<TIM6>, Timer<TIM7>,
    Timer<TIM12>, Timer<TIM13>, Timer<TIM14>, Timer<TIM15>, Timer<TIM16>, Timer<TIM17>
}
impl_delay_from_count_down_timer! {
    LpTimer<LPTIM1, Enabled>, LpTimer<LPTIM2, Enabled>, LpTimer<LPTIM3, Enabled>
}
#[cfg(not(feature = "rm0455"))]
impl_delay_from_count_down_timer! {
    LpTimer<LPTIM4, Enabled>, LpTimer<LPTIM5, Enabled>
}
