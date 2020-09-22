//! Time units

use core::fmt;
use core::time::Duration;
use cortex_m::peripheral::DWT;

/// Bits per second
#[derive(Clone, Copy, Debug)]
pub struct Bps(pub u32);

/// Hertz
#[derive(Clone, Copy, Debug)]
pub struct Hertz(pub u32);

/// KiloHertz
#[derive(Clone, Copy, Debug)]
pub struct KiloHertz(pub u32);

/// MegaHertz
#[derive(Clone, Copy, Debug)]
pub struct MegaHertz(pub u32);

/// MilliSeconds
#[derive(PartialEq, PartialOrd, Clone, Copy, Debug)]
pub struct MilliSeconds(pub u32);

/// MicroSeconds
#[derive(PartialEq, PartialOrd, Clone, Copy, Debug)]
pub struct MicroSeconds(pub u32);

/// NanoSeconds
#[derive(PartialEq, PartialOrd, Clone, Copy, Debug)]
pub struct NanoSeconds(pub u32);

impl fmt::Display for Bps {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{} bits per second", self.0)
    }
}
impl fmt::Display for Hertz {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{} Hz", self.0)
    }
}
impl fmt::Display for KiloHertz {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{} kHz", self.0)
    }
}
impl fmt::Display for MegaHertz {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{} MHz", self.0)
    }
}
impl fmt::Display for MilliSeconds {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{} ms", self.0)
    }
}
impl fmt::Display for MicroSeconds {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{} us", self.0)
    }
}
impl fmt::Display for NanoSeconds {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{} ns", self.0)
    }
}

/// Extension trait that adds convenience methods to the `u32` type
pub trait U32Ext {
    /// Wrap in `Bps`
    fn bps(self) -> Bps;

    /// Wrap in `Hertz`
    fn hz(self) -> Hertz;

    /// Wrap in `KiloHertz`
    fn khz(self) -> KiloHertz;

    /// Wrap in `MegaHertz`
    fn mhz(self) -> MegaHertz;

    /// Wrap in "MilliSeconds"
    fn ms(self) -> MilliSeconds;

    /// Wrap in "MicroSeconds"
    fn us(self) -> MicroSeconds;

    /// Wrap in "NanoSeconds"
    fn ns(self) -> NanoSeconds;
}

impl U32Ext for u32 {
    fn bps(self) -> Bps {
        Bps(self)
    }

    fn hz(self) -> Hertz {
        Hertz(self)
    }

    fn khz(self) -> KiloHertz {
        KiloHertz(self)
    }

    fn mhz(self) -> MegaHertz {
        MegaHertz(self)
    }

    fn ms(self) -> MilliSeconds {
        MilliSeconds(self)
    }

    fn us(self) -> MicroSeconds {
        MicroSeconds(self)
    }

    fn ns(self) -> NanoSeconds {
        NanoSeconds(self)
    }
}

// Unit conversions
impl Into<Hertz> for Bps {
    fn into(self) -> Hertz {
        Hertz(self.0)
    }
}

impl Into<Hertz> for KiloHertz {
    fn into(self) -> Hertz {
        Hertz(self.0 * 1_000)
    }
}

impl Into<Hertz> for MegaHertz {
    fn into(self) -> Hertz {
        Hertz(self.0 * 1_000_000)
    }
}

impl Into<KiloHertz> for MegaHertz {
    fn into(self) -> KiloHertz {
        KiloHertz(self.0 * 1_000)
    }
}

impl Into<NanoSeconds> for MicroSeconds {
    fn into(self) -> NanoSeconds {
        NanoSeconds(self.0 * 1_000)
    }
}

impl Into<NanoSeconds> for MilliSeconds {
    fn into(self) -> NanoSeconds {
        NanoSeconds(self.0 * 1_000_000)
    }
}

impl Into<MicroSeconds> for MilliSeconds {
    fn into(self) -> MicroSeconds {
        MicroSeconds(self.0 * 1_000)
    }
}

// MilliSeconds <-> Hertz
impl Into<MilliSeconds> for Hertz {
    fn into(self) -> MilliSeconds {
        let freq = self.0;
        assert!(freq != 0 && freq <= 1_000);
        MilliSeconds(1_000 / freq)
    }
}
impl Into<Hertz> for MilliSeconds {
    fn into(self) -> Hertz {
        let period = self.0;
        assert!(period != 0 && period <= 1_000);
        Hertz(1_000 / period)
    }
}

// MicroSeconds <-> Hertz
impl Into<MicroSeconds> for Hertz {
    fn into(self) -> MicroSeconds {
        let freq = self.0;
        assert!(freq != 0 && freq <= 1_000_000);
        MicroSeconds(1_000_000 / freq)
    }
}
impl Into<Hertz> for MicroSeconds {
    fn into(self) -> Hertz {
        let period = self.0;
        assert!(period != 0 && period <= 1_000_000);
        Hertz(1_000_000 / period)
    }
}

// NanoSeconds <-> Hertz
impl Into<NanoSeconds> for Hertz {
    fn into(self) -> NanoSeconds {
        let freq = self.0;
        assert!(freq != 0 && freq <= 1_000_000_000);
        NanoSeconds(1_000_000_000 / freq)
    }
}
impl Into<Hertz> for NanoSeconds {
    fn into(self) -> Hertz {
        let period = self.0;
        assert!(period != 0 && period <= 1_000_000_000);
        Hertz(1_000_000_000 / period)
    }
}

// Into core::time::Duration
impl Into<Duration> for MilliSeconds {
    fn into(self) -> Duration {
        Duration::from_millis(self.0 as u64)
    }
}

// Into core::time::Duration
impl Into<Duration> for MicroSeconds {
    fn into(self) -> Duration {
        Duration::from_micros(self.0 as u64)
    }
}

// Into core::time::Duration
impl Into<Duration> for NanoSeconds {
    fn into(self) -> Duration {
        Duration::from_nanos(self.0 as u64)
    }
}

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
//             now: DWT::get_cycle_count(),
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
        DWT::get_cycle_count().wrapping_sub(self.now)
    }
}
