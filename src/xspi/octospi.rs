//! Octo SPI (OCTOSPI) bus
//!
//! See the parent module for documentation

use core::fmt;

use crate::{
    gpio, pac,
    rcc::{rec, CoreClocks, ResetEnable},
    time::{Hertz, MicroSeconds},
};

use super::{Config, Octospi, SamplingEdge};

/// A structure for specifying a Hyperbus configuration.
///
/// This structure uses builder semantics to generate the configuration. The
/// default values used are documented [`here`](#method.new), but in most cases
/// you will need to set at least some configuration parameters to those needed
/// for your memory device.
///
/// ```
/// let config = HyperbusConfig::new(50.MHz())
///     .device_size_bytes(24) // 16 Mbyte
///     .refresh_interval(4.us())
///     .read_write_recovery(4);
/// ```
#[derive(Copy, Clone)]
pub struct HyperbusConfig {
    pub(super) frequency: Hertz,
    size_order: u8,
    refresh_interval: MicroSeconds,
    chip_select_high: u8,
    read_write_recovery: u8,
    access_initial_latency: u8,
}

impl HyperbusConfig {
    /// Create a default Hyperbus configuration.
    ///
    /// # Arguments
    ///
    /// `frequency` - Bus clock frequency for the hyperbus interface.
    ///
    /// # Defaults
    ///
    /// * Device Size = 23 (8 MByte)
    /// * Refresh Interval = 4µs
    /// * Chip select high between transactions = 4
    /// * Read-write recovery = 4
    /// * Access initial latency = 6
    pub fn new(frequency: Hertz) -> Self {
        HyperbusConfig {
            frequency,
            size_order: 23, // 8 MByte
            refresh_interval: MicroSeconds::from_ticks(4),
            chip_select_high: 4,       // 40ns @ 100MHz
            read_write_recovery: 4,    // 40ns @ 100MHz
            access_initial_latency: 6, // 60ns @ 100MHz
        }
    }

    /// Number of bytes in the device, expressed as a power of 2.
    ///
    /// | Value | Memory Size (Bytes) | Memory Size (bits)
    /// |---|---|---
    /// | 23 | 8 MByte | 64 Mbit
    /// | 24 | 16 MByte | 128 Mbit
    /// | ... | |
    ///
    /// ```
    /// let hyperbusconfig = hyperbusconfig.device_size(23);
    /// ```
    pub fn device_size_bytes(mut self, size_order: u8) -> Self {
        debug_assert!(size_order > 4, "Memory size must be at least 32 bytes");
        debug_assert!(
            size_order <= 28,
            "Maximum memory size that can be mapped is 256 MBytes"
        );

        self.size_order = size_order;
        self
    }

    /// The refresh interval sets an upper limit on the length of read and write
    /// transactions, so that the distributed refresh mechanism in the memory
    /// can operate.
    ///
    /// Typically calculated by dividing the array refresh interval by the
    /// number of rows in the array, with some margin. Called t_CSM in the
    /// memory datasheet.
    ///
    /// Set to zero to disable the upper limit on the length of read and write
    /// transactions. In this case you become reponsible for issuing the reads
    /// needed to cover the required refreshes.
    ///
    /// ```
    /// let hyperbusconfig = hyperbusconfig.refresh_interval(4.us());
    /// ```
    pub fn refresh_interval(mut self, refresh_interval: MicroSeconds) -> Self {
        self.refresh_interval = refresh_interval;
        self
    }

    /// Chip select high time t_CSHI between transactions in clock cycles
    ///
    /// The chip select high time between transactions is specified in the
    /// memory datasheet. This should be converted to clock cycles based on the
    /// maximum bus frequency. The minimum is one cycle.
    ///
    /// ```
    /// let hyperbusconfig = hyperbusconfig.chip_select_high(4);
    /// ```
    pub fn chip_select_high(mut self, chip_select_high: u8) -> Self {
        debug_assert!(
            chip_select_high > 0,
            "There is a minimum of one clock cycle between transcations"
        );
        #[cfg(feature = "rm0468")]
        debug_assert!(
            chip_select_high <= 64,
            "Maximum 64 cycles between transcations"
        );
        #[cfg(not(feature = "rm0468"))]
        debug_assert!(
            chip_select_high <= 8,
            "Maximum 8 cycles between transcations"
        );

        self.chip_select_high = chip_select_high;
        self
    }

    /// Read-write recovery time t_RWR in clock cycles
    ///
    /// The read-write recovery time is specified in the memory datasheet. This
    /// should be converted to clock cycles based on the maximum bus frequency.
    ///
    /// ```
    /// let hyperbusconfig = hyperbusconfig.read_write_recovery(4);
    /// ```
    pub fn read_write_recovery(mut self, read_write_recovery: u8) -> Self {
        self.read_write_recovery = read_write_recovery;
        self
    }

    /// Initial access time t_ACC in clock cycles
    ///
    /// This parameter is given as a time period in the memory timing
    /// characteristics. However there is typically a default number of clock
    /// cycles used by the memory which can only be changed through a
    /// re-configuration. This parameter must be set equal to the currently
    /// configured number of cycles.
    ///
    /// ```
    /// let hyperbusconfig = hyperbusconfig.access_initial_latency(6);
    /// ```
    pub fn access_initial_latency(
        mut self,
        access_initial_latency: u8,
    ) -> Self {
        self.access_initial_latency = access_initial_latency;
        self
    }
}

/// Type for a Hyperbus interface
pub struct Hyperbus<OSPI> {
    rb: OSPI,

    /// Achieved bus frequency
    frequency: Hertz,

    /// Configured refresh interval in clock cycles
    refresh_cycles: u32,
}

impl<OSPI> fmt::Display for Hyperbus<OSPI> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Hyperbus with Clock {}; ", self.frequency)?;
        write!(f, "Refresh interval: {} cycles", self.refresh_cycles)
    }
}

// impl<OSPI, CLK, NCS, IO0, IO1, IO2, IO3> PinsQuad<OSPI>
//     for (CLK, NCS, IO0, IO1, IO2, IO3)
// where
//     OSPI: Instance,
//     CLK: Into<OSPI::Clk>,
//     NCS: Into<OSPI::Ncs>,
//     IO0: Into<OSPI::Io0>,
//     IO0: Into<OSPI::Io1>,
//     IO0: Into<OSPI::Io2>,
//     IO0: Into<OSPI::Io3>,
// {
// }

pub trait Instance:
    crate::Sealed
    + core::ops::Deref<Target = pac::octospi1::RegisterBlock>
    + super::common::GetClk
{
    const MEMADDR: u32;

    /// The `ResetEnable` singleton for this peripheral
    type Rec: ResetEnable;

    type Clk;
    type Nclk;
    type Dqs;
    type Ncs;
    type Io0;
    type Io1;
    type Io2;
    type Io3;
    type Io4;
    type Io5;
    type Io6;
    type Io7;
}

pub trait OctospiExt: Sized + Instance {
    /// Create and enable the Octospi peripheral
    fn octospi_unchecked(
        self,
        config: impl Into<Config>,
        clocks: &CoreClocks,
        prec: Self::Rec,
    ) -> Octospi<Self>;

    /// Create and enable the Octospi peripheral as a memory-mapped hyperbus
    fn octospi_hyperbus_unchecked(
        self,
        config: impl Into<HyperbusConfig>,
        clocks: &CoreClocks,
        prec: Self::Rec,
    ) -> Hyperbus<Self>;
}

impl<OCTO: Instance> Octospi<OCTO> {
    pub fn unchecked(
        regs: OCTO,
        config: impl Into<Config>,
        clocks: &CoreClocks,
        prec: OCTO::Rec,
    ) -> Self {
        prec.enable();

        // Disable OCTOSPI before configuring it.
        regs.cr.write(|w| w.en().clear_bit());

        let spi_kernel_ck = OCTO::kernel_clk_unwrap(clocks).raw();

        while regs.sr.read().busy().bit_is_set() {}

        let config: Config = config.into();

        // Clear all pending flags.
        regs.fcr.write(|w| {
            w.ctof()
                .set_bit()
                .csmf()
                .set_bit()
                .ctcf()
                .set_bit()
                .ctef()
                .set_bit()
        });

        // Configure the communication method for OCTOSPI.
        regs.cr.write(|w| unsafe {
            w.fmode()
                .bits(0) // indirect mode
                .fthres()
                .bits(config.fifo_threshold - 1)
        });

        regs.dcr1.write(|w| unsafe {
            w.mtyp()
                .bits(2) // standard mode
                // Configure the FSIZE to maximum. It appears that even when addressing
                // is not used, the flash size violation may still trigger.
                .devsize()
                .bits(0x1F)
        });

        // Communications configuration register
        regs.ccr.write(|w| unsafe {
            w.dmode()
                .bits(config.mode.reg_value())
                .admode()
                .bits(config.mode.reg_value())
                .adsize()
                .bits(0) // Eight-bit address
                .imode()
                .bits(0) // No instruction phase
        });

        // Prescaler
        let spi_frequency = config.frequency.raw();
        let divisor = match (spi_kernel_ck + spi_frequency - 1) / spi_frequency
        {
            divisor @ 1..=256 => divisor - 1,
            _ => panic!("Invalid OCTOSPI frequency requested"),
        };
        regs.dcr2
            .write(|w| unsafe { w.prescaler().bits(divisor as u8) });

        // Note that we default to setting SSHIFT (sampling on the falling
        // edge). This is because it appears that the QSPI may have signal
        // contention issues when reading with zero dummy cycles. Setting SSHIFT
        // forces the read to occur on the falling edge instead of the rising
        // edge. Refer to https://github.com/quartiq/stabilizer/issues/101 for
        // more information
        //
        // SSHIFT must not be set in DDR mode.
        regs.tcr.write(|w| unsafe {
            w.sshift()
                .bit(config.sampling_edge == SamplingEdge::Falling)
                .dcyc()
                .bits(config.dummy_cycles)
        });

        // Enable the peripheral
        regs.cr.modify(|_, w| w.en().set_bit());

        Octospi {
            rb: regs,
            mode: config.mode,
        }
    }

    pub fn hyperbus_unchecked(
        regs: OCTO,
        hyperbus: impl Into<HyperbusConfig>,
        clocks: &CoreClocks,
        prec: OCTO::Rec,
    ) -> Hyperbus<OCTO> {
        prec.enable().reset();

        // Disable OCTOSPI before configuring it.
        regs.cr.write(|w| w.en().clear_bit());

        let spi_kernel_ck = OCTO::kernel_clk_unwrap(clocks).raw();

        // Configure clock
        let hyperbus: HyperbusConfig = hyperbus.into();
        let spi_frequency = hyperbus.frequency.raw();
        let divisor = match (spi_kernel_ck + spi_frequency - 1) / spi_frequency
        {
            divisor @ 1..=256 => divisor as u8,
            _ => panic!("Invalid OCTOSPI frequency requested"),
        };
        let frequency = Hertz::from_raw(spi_kernel_ck / divisor as u32);

        // Calculate the achieved clock period in ns
        let period_ns = 1e9 * (divisor as f32) / (spi_kernel_ck as f32);
        let period_ns = period_ns as u32; // floor

        while regs.sr.read().busy().bit_is_set() {}

        // Clear all pending flags.
        regs.fcr.write(|w| {
            w.ctof().set_bit();
            w.csmf().set_bit();
            w.ctcf().set_bit();
            w.ctef().set_bit()
        });

        // Configure the communication method for OCTOSPI
        regs.cr.write(|w| unsafe {
            w.fmode().bits(3); // Memory-mapped
            w.fthres().bits(4 - 1) // TODO?
        });

        regs.dcr1.write(|w| unsafe {
            w.mtyp().bits(4); // Hyperbus memory mode
            w.devsize().bits(hyperbus.size_order);
            w.csht().bits(hyperbus.chip_select_high - 1)
        });

        // Prescaler
        regs.dcr2
            .write(|w| unsafe { w.prescaler().bits(divisor - 1) });
        // CS boundary. We actually use this feature to ensure the
        // transcation is re-started when crossing between each half of
        // the memory. These are separate dies on some parts (thus
        // separate transactions may be required) and has a very minimal
        // performance penalty if not.
        regs.dcr3
            .write(|w| unsafe { w.csbound().bits(hyperbus.size_order - 1) });
        // Release nCS for refresh
        let refresh_cycles = {
            let interval_ns = 1000 * hyperbus.refresh_interval.ticks();
            (interval_ns + period_ns - 1) / period_ns
        };
        regs.dcr4
            .write(|w| unsafe { w.refresh().bits(refresh_cycles) });

        // 8-wide, DDR
        regs.ccr.write(|w| unsafe {
            w.dqse().set_bit(); // DQS enable
            w.ddtr().set_bit(); // DDR mode enabled
            w.dmode().bits(4); // 8-wide
            w.adsize().bits(3); // 32-bit
            w.addtr().set_bit(); // DDR mode enabled
            w.admode().bits(4) // 8-wide
        });
        // 8-wide, DDR
        regs.wccr.write(|w| unsafe {
            w.dqse().set_bit(); // DQS enable
            w.ddtr().set_bit(); // DDR mode enabled
            w.dmode().bits(4); // 8-wide
            w.adsize().bits(3); // 32-bit
            w.addtr().set_bit(); // DDR mode enabled
            w.admode().bits(4) // 8-wide
        });
        // TCR
        regs.tcr.write(|w| {
            w.dhqc().set_bit() // Delay hold quarter cycle
        });

        // Hyperbus
        regs.hlcr.modify(|_, w| unsafe {
            w.trwr().bits(hyperbus.read_write_recovery);
            w.tacc().bits(hyperbus.access_initial_latency);
            w.wzl().clear_bit(); // latency on write accesses
            w.lm().set_bit() // fixed latency mode
        });

        Hyperbus {
            rb: regs,
            frequency,
            refresh_cycles,
        }
    }
}

impl<OCTO: Instance> OctospiExt for OCTO {
    fn octospi_unchecked(
        self,
        config: impl Into<Config>,
        clocks: &CoreClocks,
        prec: Self::Rec,
    ) -> Octospi<Self> {
        Octospi::unchecked(self, config, clocks, prec)
    }
    fn octospi_hyperbus_unchecked(
        self,
        config: impl Into<HyperbusConfig>,
        clocks: &CoreClocks,
        prec: Self::Rec,
    ) -> Hyperbus<Self> {
        Octospi::hyperbus_unchecked(self, config, clocks, prec)
    }
}

impl<OCTO: Instance> Hyperbus<OCTO> {
    /// Initialise a memory-mapped Hyperbus peripheral and return a raw
    /// pointer to the memory
    pub fn init(self) -> *mut u32 {
        // Enable the peripheral
        self.rb.cr.modify(|_, w| w.en().set_bit());

        // Wait for the peripheral to indicate it is no longer busy
        while self.rb.sr.read().busy().bit_is_set() {}

        // Transition to memory-mapped mode
        self.rb.cr.modify(|_, w| unsafe {
            w.fmode().bits(3) // Memory mapped
        });

        // Wait for the peripheral to indicate it is no longer busy
        while self.rb.sr.read().busy().bit_is_set() {}

        // Mapped to memory
        OCTO::MEMADDR as *mut u32
    }
}

macro_rules! octospi_impl {
    ($peripheral:ty, $octo:ident, $rec:ty, $memaddr:literal) => {
        impl crate::Sealed for $peripheral {}
        impl Instance for $peripheral {
            const MEMADDR: u32 = $memaddr;

            /// The `ResetEnable` singleton for this peripheral
            type Rec = $rec;

            type Clk = gpio::alt::$octo::Clk;
            type Nclk = gpio::alt::$octo::Nclk;
            type Dqs = gpio::alt::$octo::Dqs;
            type Ncs = gpio::alt::$octo::Ncs;
            type Io0 = gpio::alt::$octo::Io0;
            type Io1 = gpio::alt::$octo::Io1;
            type Io2 = gpio::alt::$octo::Io2;
            type Io3 = gpio::alt::$octo::Io3;
            type Io4 = gpio::alt::$octo::Io4;
            type Io5 = gpio::alt::$octo::Io5;
            type Io6 = gpio::alt::$octo::Io6;
            type Io7 = gpio::alt::$octo::Io7;
        }
    };
}

octospi_impl! {
    pac::OCTOSPI1, octospi1, rec::Octospi1, 0x9000_0000
}

octospi_impl! {
    pac::OCTOSPI2, octospi2, rec::Octospi2, 0x7000_0000
}
