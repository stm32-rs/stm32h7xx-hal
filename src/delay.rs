//! Delays

use cast::u32;
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m::peripheral::SYST;

use crate::rcc::CoreClocks;
use embedded_hal::blocking::delay::{DelayMs, DelayUs};

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

impl Delay {
    /// Configures the system timer (SysTick) as a delay provider
    pub fn new(mut syst: SYST, clocks: CoreClocks) -> Self {
        syst.set_clock_source(SystClkSource::External);

        Delay { syst, clocks }
    }

    /// Releases the system timer (SysTick) resource
    pub fn free(self) -> SYST {
        self.syst
    }
}

impl DelayMs<u32> for Delay {
    fn delay_ms(&mut self, ms: u32) {
        self.delay_us(ms * 1_000);
    }
}

impl DelayMs<u16> for Delay {
    fn delay_ms(&mut self, ms: u16) {
        self.delay_ms(u32(ms));
    }
}

impl DelayMs<u8> for Delay {
    fn delay_ms(&mut self, ms: u8) {
        self.delay_ms(u32(ms));
    }
}

impl DelayUs<u32> for Delay {
    fn delay_us(&mut self, us: u32) {
        // The SysTick Reload Value register supports values between 1 and 0x00FFFFFF.
        const MAX_RVR: u32 = 0x00FF_FFFF;

        // With c_ck up to 480e6, we need u64 for delays > 8.9s

        let mut total_rvr = if cfg!(not(feature = "revision_v")) {
            // See errata ES0392 §2.2.3. Revision Y does not have the /8 divider
            u64::from(us) * u64::from(self.clocks.c_ck().0 / 1_000_000)
        } else if cfg!(feature = "cm4") {
            // CM4 dervived from HCLK
            u64::from(us) * u64::from(self.clocks.hclk().0 / 8_000_000)
        } else {
            // Normally divide by 8
            u64::from(us) * u64::from(self.clocks.c_ck().0 / 8_000_000)
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

impl DelayUs<u16> for Delay {
    fn delay_us(&mut self, us: u16) {
        self.delay_us(u32(us))
    }
}

impl DelayUs<u8> for Delay {
    fn delay_us(&mut self, us: u8) {
        self.delay_us(u32(us))
    }
}
