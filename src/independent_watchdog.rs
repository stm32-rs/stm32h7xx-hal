//! Independent Watchdog
//!
//! This module implements the embedded-hal
//! [Watchdog](https://docs.rs/embedded-hal/latest/embedded_hal/watchdog/trait.Watchdog.html)
//! trait for the Independent Watchdog peripheral.
//!
//! The Independent Watchdog peripheral triggers a system reset when its internal counter expires.
//!
//! # Peripheral Naming
//!
//! The naming of the Independent Watchdog peripheral varies between parts
//!
//! | Parts | IWDG Peripheral | Second IWDG Peripheral |
//! | --- | --- | --- |
//! | stm32H742/743/750/753/7a3/7b0/7b3 | IWDG | - |
//! | stm32h745/747/755/757 | IWDG1 | IWDG2 |
//! | stm32h723/725/730/733/735 | IWDG1 | - |
//!
//! # Examples
//!
//! - [IWDG Example](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/independent_watchdog.rs)
use crate::{prelude::*, time::MilliSeconds};

#[cfg(any(feature = "rm0433", feature = "rm0455"))]
use crate::stm32::iwdg::pr::PR;
#[cfg(any(feature = "rm0433", feature = "rm0455"))]
use crate::stm32::IWDG;

#[cfg(any(all(feature = "rm0399", feature = "cm7"), feature = "rm0468"))]
use crate::stm32::iwdg1::pr::PR;
#[cfg(any(all(feature = "rm0399", feature = "cm7"), feature = "rm0468"))]
use crate::stm32::IWDG1 as IWDG;

#[cfg(all(feature = "rm0399", feature = "cm4"))]
use crate::stm32::iwdg2::pr::PR;
#[cfg(all(feature = "rm0399", feature = "cm4"))]
use crate::stm32::IWDG2 as IWDG;

/// The implementation of the hardware IWDG
pub struct IndependentWatchdog {
    iwdg: IWDG,
}

impl IndependentWatchdog {
    const CLOCK_SPEED: u32 = 32000;
    const MAX_COUNTER_VALUE: u32 = 0x00000FFF;
    const MAX_MILLIS_FOR_PRESCALER: [(PR, u32); 8] = [
        (
            PR::DivideBy4,
            (Self::MAX_COUNTER_VALUE * 1000) / (Self::CLOCK_SPEED / 4),
        ),
        (
            PR::DivideBy8,
            (Self::MAX_COUNTER_VALUE * 1000) / (Self::CLOCK_SPEED / 8),
        ),
        (
            PR::DivideBy16,
            (Self::MAX_COUNTER_VALUE * 1000) / (Self::CLOCK_SPEED / 16),
        ),
        (
            PR::DivideBy32,
            (Self::MAX_COUNTER_VALUE * 1000) / (Self::CLOCK_SPEED / 32),
        ),
        (
            PR::DivideBy64,
            (Self::MAX_COUNTER_VALUE * 1000) / (Self::CLOCK_SPEED / 64),
        ),
        (
            PR::DivideBy128,
            (Self::MAX_COUNTER_VALUE * 1000) / (Self::CLOCK_SPEED / 128),
        ),
        (
            PR::DivideBy256,
            (Self::MAX_COUNTER_VALUE * 1000) / (Self::CLOCK_SPEED / 256),
        ),
        (
            PR::DivideBy256,
            (Self::MAX_COUNTER_VALUE * 1000) / (Self::CLOCK_SPEED / 256), // TODO: Can we remove this element now?
        ),
    ];

    /// Create a new instance
    pub fn new(iwdg: IWDG) -> Self {
        Self { iwdg }
    }

    /// Feed the watchdog, resetting the timer to 0
    pub fn feed(&mut self) {
        self.iwdg.kr().write(|w| w.key().feed());
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
        self.iwdg.kr().write(|w| w.key().start());
        // Enable register access
        self.iwdg.kr().write(|w| w.key().unlock());

        // Set the prescaler
        let (prescaler, _) = Self::MAX_MILLIS_FOR_PRESCALER
            .iter()
            .find(|(_, max_millis)| *max_millis >= max_window_time.to_millis())
            .expect("IWDG max time is greater than is possible");
        while self.iwdg.sr().read().pvu().bit_is_set() {
            cortex_m::asm::nop();
        }
        self.iwdg.pr().write(|w| w.pr().variant(*prescaler));

        // Reset the window value
        while self.iwdg.sr().read().wvu().bit_is_set() {
            cortex_m::asm::nop();
        }

        //NOTE(unsafe) 0xFFF is a valid bit patterns for w.win()
        unsafe {
            self.iwdg
                .winr()
                .write(|w| w.win().bits(Self::MAX_COUNTER_VALUE as u16));
        }

        // Calculate the counter values
        let reload_value = max_window_time.to_millis()
            * (Self::CLOCK_SPEED / 1000)
            / Self::get_prescaler_divider(prescaler);
        let window_value = min_window_time.to_millis()
            * (Self::CLOCK_SPEED / 1000)
            / Self::get_prescaler_divider(prescaler);

        // Set the reload value
        while self.iwdg.sr().read().rvu().bit_is_set() {
            cortex_m::asm::nop();
        }
        //NOTE(unsafe) Only valid bit patterns written, values are checked above for maximum value
        unsafe {
            self.iwdg.rlr().write(|w| w.rl().bits(reload_value as u16));
        }

        self.feed();
        // Enable register access
        self.iwdg.kr().write(|w| w.key().unlock());

        // Set the window value
        while self.iwdg.sr().read().wvu().bit_is_set() {
            cortex_m::asm::nop();
        }

        // TODO: There is nothing preventing this from underflowing right?
        unsafe {
            self.iwdg
                .winr()
                .write(|w| w.win().bits((reload_value - window_value) as u16));
        }

        // Wait until everything is set
        while self.iwdg.sr().read().bits() != 0 {
            cortex_m::asm::nop();
        }

        self.feed();
    }

    /// Start the watchdog with the given max time and no minimal time
    pub fn start<T: Into<MilliSeconds>>(&mut self, max_time: T) {
        self.start_windowed(0_u32.millis(), max_time.into());
    }

    fn get_prescaler_divider(prescaler: &PR) -> u32 {
        match prescaler {
            PR::DivideBy4 => 4,
            PR::DivideBy8 => 8,
            PR::DivideBy16 => 16,
            PR::DivideBy32 => 32,
            PR::DivideBy64 => 64,
            PR::DivideBy128 => 128,
            PR::DivideBy256 => 256,
        }
    }
}
