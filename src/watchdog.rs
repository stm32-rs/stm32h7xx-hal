//! System Window Watchdog
//!
//! # Examples
//!
//! - [Example application](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/watchdog.rs)

use crate::hal::watchdog::{Watchdog, WatchdogEnable};
use crate::rcc::Ccdr;
use crate::time::{Hertz, MilliSeconds};
use cast::u8;

/// Select Window Watchdog hardware based on core
#[cfg(any(feature = "rm0433", feature = "rm0455"))]
use crate::stm32::WWDG;
#[cfg(all(feature = "rm0399", feature = "cm7"))]
use crate::stm32::WWDG1 as WWDG;
#[cfg(feature = "rm0468")]
use crate::stm32::WWDG1 as WWDG;
#[cfg(all(feature = "rm0399", feature = "cm4"))]
use crate::stm32::WWDG2 as WWDG;

/// Event enum for [SystemWindowWatchdog]
pub enum Event {
    /// Early wakeup interrupt. This will generate an interrupt when the watchdog would otherwise reset.
    /// This interrupt can then be used for data logging or even recovery.
    /// If not handled, the watchdog will still reset the device after a period that is at least double the period that
    /// was set with the [SystemWindowWatchdog::start] function.
    /// When the watchdog is fed, it will resume as normal again.
    EarlyWakeup,
}

/// Implements the System Window Watchdog
pub struct SystemWindowWatchdog {
    wwdg: WWDG,
    down_counter: u8,
    pclk3_frequency: Hertz,
}

impl SystemWindowWatchdog {
    /// Returns a System Window Watchdog object with down_counter intialized to zero
    /// to indicate the clock has not been used yet
    pub fn new(wwdg: WWDG, ccdr: &Ccdr) -> Self {
        // enable the peripheral inside the APB3
        #[cfg(not(feature = "rm0455"))]
        ccdr.rb.apb3enr.modify(|_, w| w.wwdg1en().set_bit());
        #[cfg(feature = "rm0455")]
        ccdr.rb.apb3enr.modify(|_, w| w.wwdgen().set_bit());

        SystemWindowWatchdog {
            wwdg,
            down_counter: 0,
            pclk3_frequency: ccdr.clocks.pclk3(),
        }
    }

    /// Start listening for `event`
    pub fn listen(&mut self, event: Event) {
        match event {
            Event::EarlyWakeup => {
                // If this value is 0 it is assumed that the watchdog has not yet been started.
                // It needs to have started because the starting procedure already makes the early wakeup pending,
                // which would immediately call the interrupt.
                assert!(self.down_counter != 0);
                // Set the ewi bit
                self.wwdg.cfr.modify(|_, w| w.ewi().enable());
            }
        }
    }

    /// Stop listening for `event`
    pub fn unlisten(&mut self, event: Event) {
        match event {
            Event::EarlyWakeup => panic!("Early wakeup of the SystemWindowWatchdog can only be cleared by hardware after a reset"),
        }
    }

    /// Returns `true` if `event` is pending
    pub fn is_pending(&self, event: Event) -> bool {
        match event {
            Event::EarlyWakeup => self.wwdg.sr.read().ewif().is_pending(),
        }
    }

    /// Clears the interrupt flag for `event`
    pub fn unpend(&mut self, event: Event) {
        match event {
            Event::EarlyWakeup => {
                self.wwdg.sr.write(|w| w.ewif().finished());
            }
        }
    }

    /// Returns a reference to the inner peripheral
    pub fn inner(&self) -> &WWDG {
        &self.wwdg
    }

    /// Returns a mutable reference to the inner peripheral
    pub fn inner_mut(&mut self) -> &mut WWDG {
        &mut self.wwdg
    }
}

impl Watchdog for SystemWindowWatchdog {
    /// Feeds the watchdog in order to avoid a reset, only executes properly if the watchdog
    /// has already been started aka. the down_counter is not 0 anymore
    fn feed(&mut self) {
        // if this value is 0 it is assumed that the watchdog has not yet been started
        assert!(self.down_counter != 0);
        self.wwdg.cr.modify(|_, w| w.t().bits(self.down_counter));
    }
}

impl WatchdogEnable for SystemWindowWatchdog {
    type Time = MilliSeconds;
    /// Starts the watchdog with a given timeout period, if this period is out of bounds the function
    /// is going to panic
    fn start<T>(&mut self, period: T)
    where
        T: Into<Self::Time>,
    {
        let period_ms = period.into().0;
        let maximum =
            (4096 * 2u32.pow(7) * 64) / (self.pclk3_frequency.0 / 1000);
        assert!(period_ms <= maximum);

        // timeout = pclk * 4096 * 2^WDGTB[2:0] * (t[5:0] +1)
        let ratio = period_ms * (self.pclk3_frequency.0 / 1000) / 4096;

        // Prescaler
        let (tb_div, wdgtb) = match ratio / 64 {
            0 => (1, 0),
            1 => (2, 1),
            2..=3 => (4, 2),
            4..=7 => (8, 3),
            8..=15 => (16, 4),
            16..=31 => (32, 5),
            32..=63 => (64, 6),
            64..=127 => (128, 7),
            _ => (128, 7),
        };

        let t = ratio / tb_div;
        assert!(t < 64);

        self.down_counter = u8(t).unwrap() | (1 << 6);

        // write the config values, matching the set timeout the most
        // TODO: stm32h7 0.14.0 WDGTB is 3 bits (currently it's 2 and that's wrong), so let's set it directly
        const WDGTB_MASK: u32 = 0b111 << 11;
        self.wwdg.cfr.modify(|r, w| unsafe {
            w.bits((r.bits() & !WDGTB_MASK) | (wdgtb << 11))
        });
        self.wwdg.cfr.modify(|_, w| w.w().bits(self.down_counter));
        self.wwdg.cr.modify(|_, w| w.t().bits(self.down_counter));
        // For some reason, setting the t value makes the early wakeup pending.
        // That's bad behaviour, so lets turn it off again.
        self.unpend(Event::EarlyWakeup);
        // enable the watchdog
        self.wwdg.cr.modify(|_, w| w.wdga().set_bit());
    }
}
