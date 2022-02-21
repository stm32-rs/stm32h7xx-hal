//! Time units

pub use fugit::{
    HertzU32 as Hertz, KilohertzU32 as KiloHertz, MegahertzU32 as MegaHertz,
    MicrosDurationU32 as MicroSeconds, MillisDurationU32 as MilliSeconds,
    NanosDurationU32 as NanoSeconds,
};

//use core::time::Duration;
use cortex_m::peripheral::DWT;

/// Bits per second
pub type Bps = Hertz;

/// Extension trait that adds convenience methods to the `u32` type
pub trait U32Ext {
    /// Wrap in `Bps`
    fn bps(self) -> Bps;
}

impl U32Ext for u32 {
    fn bps(self) -> Bps {
        Bps::from_raw(self)
    }
}

/*
// Implement conversion from time periods into core::time::Duration
impl From<MilliSeconds> for Duration {
    fn from(ms: MilliSeconds) -> Self {
        Self::from_millis(ms.0 as u64)
    }
}

impl From<MicroSeconds> for Duration {
    fn from(us: MicroSeconds) -> Self {
        Self::from_micros(us.0 as u64)
    }
}

impl From<NanoSeconds> for Duration {
    fn from(ns: NanoSeconds) -> Self {
        Self::from_nanos(ns.0 as u64)
    }
}
*/

// /// A monotonic nondecreasing timer
// #[derive(Clone, Copy)]
// pub struct MonoTimer {
//     frequency: Hertz,
// }

// impl MonoTimer {
//     /// Creates a new `Monotonic` timer
//     pub fn new(mut dwt: DWT, clocks: Clocks) -> Self {
//         dwt.enable_cycle_counter();

//         // now the CYCCNT counter can't be stopped or resetted
//         drop(dwt);

//         MonoTimer {
//             frequency: clocks.sysclk(),
//         }
//     }

//     /// Returns the frequency at which the monotonic timer is operating at
//     pub fn frequency(&self) -> Hertz {
//         self.frequency
//     }

//     /// Returns an `Instant` corresponding to "now"
//     pub fn now(&self) -> Instant {
//         Instant {
//             now: DWT::cycle_count(),
//         }
//     }
// }

/// A measurement of a monotonically nondecreasing clock
#[derive(Clone, Copy)]
pub struct Instant {
    now: u32,
}

impl Instant {
    /// Ticks elapsed since the `Instant` was created
    pub fn elapsed(&self) -> u32 {
        DWT::cycle_count().wrapping_sub(self.now)
    }
}
