//! Implementation of [IndependentWatchdog]

use crate::{prelude::*, time::MilliSeconds};

#[cfg(any(feature = "rm0433", feature = "rm0455"))]
use crate::stm32::iwdg::pr::PR_A;
#[cfg(any(feature = "rm0433", feature = "rm0455"))]
use crate::stm32::IWDG;

#[cfg(all(feature = "rm0399", feature = "cm7"))]
use crate::stm32::iwdg1::pr::PR_A;
#[cfg(all(feature = "rm0399", feature = "cm7"))]
use crate::stm32::IWDG1 as IWDG;

#[cfg(all(feature = "rm0399", feature = "cm4"))]
use crate::stm32::iwdg2::pr::PR_A;
#[cfg(all(feature = "rm0399", feature = "cm4"))]
use crate::stm32::IWDG2 as IWDG;

/// The implementation of the hardware IWDG
pub struct IndependentWatchdog {
    iwdg: IWDG,
}

impl IndependentWatchdog {
    const CLOCK_SPEED: u32 = 32000;
    const MAX_COUNTER_VALUE: u32 = 0x00000FFF;
    const MAX_MILLIS_FOR_PRESCALER: [(PR_A, u32); 8] = [
        (
            PR_A::DIVIDEBY4,
            (Self::MAX_COUNTER_VALUE * 1000) / (Self::CLOCK_SPEED / 4),
        ),
        (
            PR_A::DIVIDEBY8,
            (Self::MAX_COUNTER_VALUE * 1000) / (Self::CLOCK_SPEED / 8),
        ),
        (
            PR_A::DIVIDEBY16,
            (Self::MAX_COUNTER_VALUE * 1000) / (Self::CLOCK_SPEED / 16),
        ),
        (
            PR_A::DIVIDEBY32,
            (Self::MAX_COUNTER_VALUE * 1000) / (Self::CLOCK_SPEED / 32),
        ),
        (
            PR_A::DIVIDEBY64,
            (Self::MAX_COUNTER_VALUE * 1000) / (Self::CLOCK_SPEED / 64),
        ),
        (
            PR_A::DIVIDEBY128,
            (Self::MAX_COUNTER_VALUE * 1000) / (Self::CLOCK_SPEED / 128),
        ),
        (
            PR_A::DIVIDEBY256,
            (Self::MAX_COUNTER_VALUE * 1000) / (Self::CLOCK_SPEED / 256),
        ),
        (
            PR_A::DIVIDEBY256BIS,
            (Self::MAX_COUNTER_VALUE * 1000) / (Self::CLOCK_SPEED / 256),
        ),
    ];

    /// Create a new instance
    pub fn new(iwdg: IWDG) -> Self {
        Self { iwdg }
    }

    /// Feed the watchdog, resetting the timer to 0
    pub fn feed(&mut self) {
        self.iwdg.kr.write(|w| w.key().reset());
    }

    /// Start the watchdog where it must be fed before the max time is over and
    /// not before the min time has passed
    pub fn start_windowed<T: Into<MilliSeconds>>(
        &mut self,
        min_window_time: T,
        max_window_time: T,
    ) {
        let min_window_time: MilliSeconds = min_window_time.into();
        let max_window_time: MilliSeconds = max_window_time.into();

        // Start the watchdog
        self.iwdg.kr.write(|w| w.key().start());
        // Enable register access
        self.iwdg.kr.write(|w| w.key().enable());

        // Set the prescaler
        let (prescaler, _) = Self::MAX_MILLIS_FOR_PRESCALER
            .iter()
            .find(|(_, max_millis)| *max_millis >= max_window_time.to_millis())
            .expect("IWDG max time is greater than is possible");
        while self.iwdg.sr.read().pvu().bit_is_set() {
            cortex_m::asm::nop();
        }
        self.iwdg.pr.write(|w| w.pr().variant(*prescaler));

        // Reset the window value
        while self.iwdg.sr.read().wvu().bit_is_set() {
            cortex_m::asm::nop();
        }
        self.iwdg
            .winr
            .write(|w| w.win().bits(Self::MAX_COUNTER_VALUE as u16));

        // Calculate the counter values
        let reload_value = max_window_time.to_millis()
            * (Self::CLOCK_SPEED / 1000)
            / Self::get_prescaler_divider(prescaler);
        let window_value = min_window_time.to_millis()
            * (Self::CLOCK_SPEED / 1000)
            / Self::get_prescaler_divider(prescaler);

        // Set the reload value
        while self.iwdg.sr.read().rvu().bit_is_set() {
            cortex_m::asm::nop();
        }
        self.iwdg.rlr.write(|w| w.rl().bits(reload_value as u16));

        self.feed();
        // Enable register access
        self.iwdg.kr.write(|w| w.key().enable());

        // Set the window value
        while self.iwdg.sr.read().wvu().bit_is_set() {
            cortex_m::asm::nop();
        }
        self.iwdg
            .winr
            .write(|w| w.win().bits((reload_value - window_value) as u16));

        // Wait until everything is set
        while self.iwdg.sr.read().bits() != 0 {
            cortex_m::asm::nop();
        }

        self.feed();
    }

    /// Start the watchdog with the given max time and no minimal time
    pub fn start<T: Into<MilliSeconds>>(&mut self, max_time: T) {
        self.start_windowed(0_u32.millis(), max_time.into());
    }

    fn get_prescaler_divider(prescaler: &PR_A) -> u32 {
        match prescaler {
            PR_A::DIVIDEBY4 => 4,
            PR_A::DIVIDEBY8 => 8,
            PR_A::DIVIDEBY16 => 16,
            PR_A::DIVIDEBY32 => 32,
            PR_A::DIVIDEBY64 => 64,
            PR_A::DIVIDEBY128 => 128,
            PR_A::DIVIDEBY256 => 256,
            PR_A::DIVIDEBY256BIS => 256,
        }
    }
}
