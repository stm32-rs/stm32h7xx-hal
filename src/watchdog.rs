//! System Window Watchdog

use crate::hal::watchdog::{Watchdog, WatchdogEnable};
use crate::rcc::Ccdr;
use crate::time::{Hertz, Milliseconds};
use cast::u8;

/// Select Window Watchdog hardware based on core
#[cfg(any(feature = "singlecore"))]
use crate::stm32::WWDG;
#[cfg(all(feature = "dualcore", feature = "cm7"))]
use crate::stm32::WWDG1 as WWDG;
#[cfg(all(feature = "dualcore", feature = "cm4"))]
use crate::stm32::WWDG2 as WWDG;

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
        ccdr.rb.apb3enr.modify(|_, w| w.wwdg1en().set_bit());
        SystemWindowWatchdog {
            wwdg,
            down_counter: 0,
            pclk3_frequency: ccdr.clocks.pclk3(),
        }
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
    type Time = Milliseconds;
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
        self.wwdg.cfr.modify(|_, w| w.wdgtb().bits(wdgtb));
        self.wwdg.cfr.modify(|_, w| w.w().bits(self.down_counter));
        self.wwdg.cr.modify(|_, w| w.t().bits(self.down_counter));
        // enable the watchdog
        self.wwdg.cr.modify(|_, w| w.wdga().set_bit());
    }
}
