//! Time units

use core::fmt;
use core::time::Duration;
use cortex_m::peripheral::DWT;

/// Bits per second
#[derive(PartialEq, PartialOrd, Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Bps(pub u32);

/// Hertz
#[derive(PartialEq, PartialOrd, Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Hertz(pub u32);

/// KiloHertz
#[derive(PartialEq, PartialOrd, Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct KiloHertz(pub u32);

/// MegaHertz
#[derive(PartialEq, PartialOrd, Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MegaHertz(pub u32);

/// MilliSeconds
#[derive(PartialEq, PartialOrd, Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MilliSeconds(pub u32);

/// MicroSeconds
#[derive(PartialEq, PartialOrd, Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MicroSeconds(pub u32);

/// NanoSeconds
#[derive(PartialEq, PartialOrd, Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
impl From<Bps> for Hertz {
    fn from(bps: Bps) -> Self {
        Self(bps.0)
    }
}

impl From<KiloHertz> for Hertz {
    fn from(khz: KiloHertz) -> Self {
        Self(khz.0 * 1_000)
    }
}

impl From<MegaHertz> for Hertz {
    fn from(mhz: MegaHertz) -> Self {
        Self(mhz.0 * 1_000_000)
    }
}

impl From<MegaHertz> for KiloHertz {
    fn from(mhz: MegaHertz) -> Self {
        Self(mhz.0 * 1_000)
    }
}

impl From<MicroSeconds> for NanoSeconds {
    fn from(us: MicroSeconds) -> Self {
        Self(us.0 * 1_000)
    }
}

impl From<MilliSeconds> for NanoSeconds {
    fn from(ms: MilliSeconds) -> Self {
        Self(ms.0 * 1_000_000)
    }
}

impl From<MilliSeconds> for MicroSeconds {
    fn from(ms: MilliSeconds) -> Self {
        Self(ms.0 * 1_000)
    }
}

// MilliSeconds <-> Hertz
impl From<Hertz> for MilliSeconds {
    fn from(hz: Hertz) -> Self {
        let freq = hz.0;
        assert!(freq != 0 && freq <= 1_000);
        Self(1_000 / freq)
    }
}
impl From<MilliSeconds> for Hertz {
    fn from(ms: MilliSeconds) -> Self {
        let period = ms.0;
        assert!(period != 0 && period <= 1_000);
        Self(1_000 / period)
    }
}

// MicroSeconds <-> Hertz
impl From<Hertz> for MicroSeconds {
    fn from(hz: Hertz) -> Self {
        let freq = hz.0;
        assert!(freq != 0 && freq <= 1_000_000);
        Self(1_000_000 / freq)
    }
}
impl From<MicroSeconds> for Hertz {
    fn from(us: MicroSeconds) -> Self {
        let period = us.0;
        assert!(period != 0 && period <= 1_000_000);
        Self(1_000_000 / period)
    }
}

// NanoSeconds <-> Hertz
impl From<Hertz> for NanoSeconds {
    fn from(hz: Hertz) -> Self {
        let freq = hz.0;
        assert!(freq != 0 && freq <= 1_000_000_000);
        Self(1_000_000_000 / freq)
    }
}
impl From<NanoSeconds> for Hertz {
    fn from(ns: NanoSeconds) -> Self {
        let period = ns.0;
        assert!(period != 0 && period <= 1_000_000_000);
        Self(1_000_000_000 / period)
    }
}

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
