//! Quad SPI (QSPI) bus
//!
//! The QSPI peripheral supports a SPI interface operating over 1, 2, or 4 IO lines.
//!
//! # Usage
//!
//! This driver supports using the QSPI peripheral in indirect mode. This allows the peripheral to
//! be used to read and write from an address over a quad-SPI interface.
//!
//! The SPI can be configured to operate on either of the two available banks on the board. In the
//! simplest case, this can be accomplished with just the peripheral and the GPIO pins.
//!
//! ```
//! use stm32h7xx_hal::qspi;
//!
//! // Get the device peripherals and instantiate IO pins.
//! let dp = ...;
//! let (sck, io0, io1, io2, io3) = ...;
//!
//! let mut qspi = dp.QUADSPI.bank1((sck, io0, io1, io2, io3), 3.mhz(), &ccdr.clocks,
//!                                 ccdr.peripheral.QSPI);
//!
//! // Configure QSPI to operate in 4-bit mode.
//! qspi.configure_mode(qspi::QspiMode::FourBit).unwrap();
//!
//! // Write data to address 0x00 on the QSPI interface.
//! qspi.write(0x00, &[0xAB, 0xCD]).unwrap();
//! ```
//!
//! # Limitations
//!
//! This driver currently only supports indirect operation mode of the QSPI
//! interface. Automatic polling or memory-mapped modes are not supported.  This
//! driver support either bank 1 or bank 2 as well as a dual flash bank (in
//! which all 8 IOs are used for the interface).
use crate::{
    gpio::{
        gpioa::PA1,
        gpiob::PB2,
        gpioc::{PC10, PC9},
        gpiod::{PD11, PD12, PD13},
        gpioe::{PE10, PE2, PE7, PE8, PE9},
        gpiof::{PF10, PF6, PF7, PF8, PF9},
        gpiog::{PG14, PG9},
        gpioh::{PH2, PH3},
        Alternate, AF10, AF9,
    },
    rcc::{rec, CoreClocks, ResetEnable},
    stm32,
    time::Hertz,
};

use core::{marker::PhantomData, ptr};

/// Represents operation modes of the QSPI interface.
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum QspiMode {
    /// Only a single IO line (IO0) is used for transmit and a separate line (IO1) is used for receive.
    OneBit,

    /// Two IO lines (IO0 and IO1) are used for transmit/receive.
    TwoBit,

    /// All four IO lines are used for transmit/receive.
    FourBit,
}
impl QspiMode {
    pub(self) fn reg_value(&self) -> u8 {
        match self {
            QspiMode::OneBit => 1,
            QspiMode::TwoBit => 2,
            QspiMode::FourBit => 3,
        }
    }
}

/// Address sizes used by the QSPI interface
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum AddressSize {
    EightBit,
    SixteenBit,
    TwentyFourBit,
    ThirtyTwoBit,
}

/// Sampling mode for the QSPI interface
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum SamplingEdge {
    Falling,
    Rising,
}

/// Indicates an error with the QSPI peripheral.
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum QspiError {
    Busy,
    Underflow,
}

/// Indicates a specific QSPI bank to use.
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum Bank {
    One,
    Two,
    Dual,
}

/// A structure for specifying the QSPI configuration.
///
/// This structure uses builder semantics to generate the configuration.
///
/// ```
/// let config = Config::new().dummy_cycles(1);
/// ```
#[derive(Copy, Clone)]
pub struct Config {
    mode: QspiMode,
    frequency: Hertz,
    address_size: AddressSize,
    dummy_cycles: u8,
    sampling_edge: SamplingEdge,
}

impl Config {
    /// Create a default configuration for the QSPI interface.
    ///
    /// * Bus in 1-bit Mode
    /// * 8-bit Address
    /// * No dummy cycle
    /// * Sample on falling edge
    pub fn new<T: Into<Hertz>>(freq: T) -> Self {
        Config {
            mode: QspiMode::OneBit,
            frequency: freq.into(),
            address_size: AddressSize::EightBit,
            dummy_cycles: 0,
            sampling_edge: SamplingEdge::Falling,
        }
    }

    /// Specify the operating mode of the QSPI bus. Can be a 1-bit, 2-bit or
    /// 4-bit width.
    ///
    /// The operating mode can also be changed using the
    /// [`configure_mode`](Qspi#method.configure_mode) method
    pub fn mode(mut self, mode: QspiMode) -> Self {
        self.mode = mode;
        self
    }

    /// Specify the size of the address phase
    pub fn address_size(mut self, address_size: AddressSize) -> Self {
        self.address_size = address_size;
        self
    }

    /// Specify the number of dummy cycles in between the address and data
    /// phases.
    ///
    /// Hardware supports 0-31 dummy cycles.
    ///
    /// # Note
    ///
    /// With zero dummy cycles, the QSPI peripheral will erroneously drive the
    /// output pins for an extra half clock cycle before IO is swapped from
    /// output to input. Refer to
    /// https://github.com/quartiq/stabilizer/issues/101 for more information.
    pub fn dummy_cycles(mut self, cycles: u8) -> Self {
        debug_assert!(cycles < 32, "Hardware only supports 0-31 dummy cycles");

        self.dummy_cycles = cycles;
        self
    }

    /// Specify the sampling edge for the QSPI receiver.
    ///
    /// # Note
    ///
    /// If zero dummy cycles are used, during read operations the QSPI
    /// peripheral will erroneously drive the output pins for an extra half
    /// clock cycle before IO is swapped from output to input. Refer to
    /// https://github.com/quartiq/stabilizer/issues/101 for more information.
    ///
    /// In this case it is recommended to sample on the falling edge. Although
    /// this doesn't stop the possible bus contention, delaying the sampling
    /// point by an extra half cycle results in a sampling point after the bus
    /// contention.
    pub fn sampling_edge(mut self, sampling_edge: SamplingEdge) -> Self {
        self.sampling_edge = sampling_edge;
        self
    }
}

impl<T: Into<Hertz>> From<T> for Config {
    fn from(frequency: T) -> Self {
        Self::new(frequency)
    }
}

/// Used to indicate that an IO pin is not used by the QSPI interface.
pub struct NoIo {}

/// Indicates a set of pins can be used for the QSPI interface on bank 1.
pub trait PinsBank1 {}
pub trait PinIo0Bank1 {}
pub trait PinIo1Bank1 {}
pub trait PinIo2Bank1 {}
pub trait PinIo3Bank1 {}

/// Indicates a set of pins can be used for the QSPI interface on bank 2.
pub trait PinsBank2 {}
pub trait PinSckBank2 {}
pub trait PinIo0Bank2 {}
pub trait PinIo1Bank2 {}
pub trait PinIo2Bank2 {}
pub trait PinIo3Bank2 {}

pub trait PinSck {}

impl<SCK, IO0, IO1, IO2, IO3> PinsBank1 for (SCK, IO0, IO1, IO2, IO3)
where
    SCK: PinSck,
    IO0: PinIo0Bank1,
    IO1: PinIo1Bank1,
    IO2: PinIo2Bank1,
    IO3: PinIo3Bank1,
{
}

impl<SCK, IO0, IO1, IO2, IO3> PinsBank2 for (SCK, IO0, IO1, IO2, IO3)
where
    SCK: PinSck,
    IO0: PinIo0Bank2,
    IO1: PinIo1Bank2,
    IO2: PinIo2Bank2,
    IO3: PinIo3Bank2,
{
}

macro_rules! pins {
    (Bank1: [IO0: [$($IO0:ty),*] IO1: [$($IO1:ty),*] IO2: [$($IO2:ty),*] IO3: [$($IO3:ty),*]]) => {
        $(
            impl PinIo0Bank1 for $IO0 {}
        )*
        $(
            impl PinIo1Bank1 for $IO1 {}
        )*
        $(
            impl PinIo2Bank1 for $IO2 {}
        )*
        $(
            impl PinIo3Bank1 for $IO3 {}
        )*
    };

    (Bank2: [IO0: [$($IO0:ty),*] IO1: [$($IO1:ty),*] IO2: [$($IO2:ty),*] IO3: [$($IO3:ty),*]]) => {
        $(
            impl PinIo0Bank2 for $IO0 {}
        )*
        $(
            impl PinIo1Bank2 for $IO1 {}
        )*
        $(
            impl PinIo2Bank2 for $IO2 {}
        )*
        $(
            impl PinIo3Bank2 for $IO3 {}
        )*
    };

    (SCK: [$($SCK:ty),*], Bank1: $bank1:tt, Bank2: $bank2:tt) => {
        $(
            impl PinSck for $SCK {}
        )*
        pins!(Bank1: $bank1);
        pins!(Bank2: $bank2);
    };
}

pins! {
    SCK: [
        PB2<Alternate<AF9>>,
        PF10<Alternate<AF9>>
    ],
    Bank1: [
        IO0: [
            PC9<Alternate<AF9>>,
            PD11<Alternate<AF9>>,
            PF8<Alternate<AF10>>
        ]
        IO1: [
            PC10<Alternate<AF9>>,
            PD12<Alternate<AF9>>,
            PF9<Alternate<AF10>>,
            NoIo
        ]
        IO2: [
            PE2<Alternate<AF9>>,
            PF7<Alternate<AF9>>,
            NoIo
        ]
        IO3: [
            PA1<Alternate<AF9>>,
            PD13<Alternate<AF9>>,
            PF6<Alternate<AF9>>,
            NoIo
        ]
    ],
    Bank2: [
        IO0: [
            PE7<Alternate<AF10>>,
            PF8<Alternate<AF10>>,
            PH2<Alternate<AF9>>
        ]
        IO1: [
            PE8<Alternate<AF10>>,
            PF9<Alternate<AF10>>,
            PH3<Alternate<AF9>>,
            NoIo
        ]
        IO2: [
            PE9<Alternate<AF10>>,
            PG9<Alternate<AF9>>,
            NoIo
        ]
        IO3: [
            PE10<Alternate<AF10>>,
            PG14<Alternate<AF9>>,
            NoIo
        ]
    ]
}

pub trait QspiExt {
    fn bank1<CONFIG, PINS>(
        self,
        _pins: PINS,
        config: CONFIG,
        clocks: &CoreClocks,
        prec: rec::Qspi,
    ) -> Qspi
    where
        CONFIG: Into<Config>,
        PINS: PinsBank1;

    fn bank2<CONFIG, PINS>(
        self,
        _pins: PINS,
        config: CONFIG,
        clocks: &CoreClocks,
        prec: rec::Qspi,
    ) -> Qspi
    where
        CONFIG: Into<Config>,
        PINS: PinsBank2;

    fn qspi_unchecked<CONFIG>(
        self,
        config: CONFIG,
        bank: Bank,
        clocks: &CoreClocks,
        prec: rec::Qspi,
    ) -> Qspi
    where
        CONFIG: Into<Config>;
}

/// Interrupt events
#[derive(Copy, Clone, PartialEq)]
pub enum Event {
    /// FIFO Threashold
    FIFOThreashold,
    /// Transfer complete
    Complete,
    /// Tranfer error
    Error,
}

pub struct Qspi {
    rb: stm32::QUADSPI,
}

impl Qspi {
    pub fn bank1<CONFIG, PINS>(
        regs: stm32::QUADSPI,
        _pins: PINS,
        config: CONFIG,
        clocks: &CoreClocks,
        prec: rec::Qspi,
    ) -> Self
    where
        CONFIG: Into<Config>,
        PINS: PinsBank1,
    {
        Self::qspi_unchecked(regs, config, Bank::One, clocks, prec)
    }

    pub fn bank2<CONFIG, PINS>(
        regs: stm32::QUADSPI,
        _pins: PINS,
        config: CONFIG,
        clocks: &CoreClocks,
        prec: rec::Qspi,
    ) -> Self
    where
        CONFIG: Into<Config>,
        PINS: PinsBank2,
    {
        Self::qspi_unchecked(regs, config, Bank::Two, clocks, prec)
    }

    pub fn qspi_unchecked<CONFIG>(
        regs: stm32::QUADSPI,
        config: CONFIG,
        bank: Bank,
        clocks: &CoreClocks,
        prec: rec::Qspi,
    ) -> Self
    where
        CONFIG: Into<Config>,
    {
        prec.enable();

        // Disable QUADSPI before configuring it.
        regs.cr.write(|w| w.en().clear_bit());

        let spi_kernel_ck = match Self::get_clock(clocks) {
            Some(freq_hz) => freq_hz.0,
            _ => panic!("QSPI kernel clock not running!"),
        };

        while regs.sr.read().busy().bit_is_set() {}

        let config: Config = config.into();

        // Configure the FSIZE to maximum. It appears that even when addressing is not used, the
        // flash size violation may still trigger.
        regs.dcr.write(|w| unsafe { w.fsize().bits(0x1F) });

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

        // Configure the communication method for QSPI.
        regs.ccr.write(|w| unsafe {
            w.fmode()
                .bits(0) // indirect mode
                .dmode()
                .bits(config.mode.reg_value())
                .admode()
                .bits(config.mode.reg_value())
                .adsize()
                .bits(config.address_size as u8)
                .imode()
                .bits(0) // No instruction phase
                .dcyc()
                .bits(config.dummy_cycles)
        });

        let spi_frequency = config.frequency.0;
        let divisor = match (spi_kernel_ck + spi_frequency - 1) / spi_frequency
        {
            divisor @ 1..=256 => divisor - 1,
            _ => panic!("Invalid QSPI frequency requested"),
        };

        // Write the prescaler and the SSHIFT bit.
        //
        // Note that we default to setting SSHIFT (sampling on the falling
        // edge). This is because it appears that the QSPI may have signal
        // contention issues when reading with zero dummy cycles. Setting SSHIFT
        // forces the read to occur on the falling edge instead of the rising
        // edge. Refer to https://github.com/quartiq/stabilizer/issues/101 for
        // more information
        //
        // SSHIFT must not be set in DDR mode.
        regs.cr.write(|w| unsafe {
            w.prescaler()
                .bits(divisor as u8)
                .sshift()
                .bit(config.sampling_edge == SamplingEdge::Falling)
        });

        match bank {
            Bank::One => regs.cr.modify(|_, w| w.fsel().clear_bit()),
            Bank::Two => regs.cr.modify(|_, w| w.fsel().set_bit()),
            Bank::Dual => regs.cr.modify(|_, w| w.dfm().set_bit()),
        }

        // Enable ther peripheral
        regs.cr.modify(|_, w| w.en().set_bit());

        Qspi { rb: regs }
    }

    /// Deconstructs the QSPI HAL and returns the component parts
    pub fn free(self) -> (stm32::QUADSPI, rec::Qspi) {
        (
            self.rb,
            rec::Qspi {
                _marker: PhantomData,
            },
        )
    }

    /// Returns a reference to the inner peripheral
    pub fn inner(&self) -> &stm32::QUADSPI {
        &self.rb
    }

    /// Returns a mutable reference to the inner peripheral
    pub fn inner_mut(&mut self) -> &mut stm32::QUADSPI {
        &mut self.rb
    }

    /// Check if the QSPI peripheral is currently busy with a transaction
    pub fn is_busy(&self) -> bool {
        self.rb.sr.read().busy().bit_is_set()
    }

    /// Enable interrupts for the given `event`
    pub fn listen(&mut self, event: Event) {
        self.rb.cr.modify(|_, w| match event {
            Event::FIFOThreashold => w.ftie().set_bit(),
            Event::Complete => w.tcie().set_bit(),
            Event::Error => w.teie().set_bit(),
        });
    }

    /// Disable interrupts for the given `event`
    pub fn unlisten(&mut self, event: Event) {
        self.rb.cr.modify(|_, w| match event {
            Event::FIFOThreashold => w.ftie().clear_bit(),
            Event::Complete => w.tcie().clear_bit(),
            Event::Error => w.teie().clear_bit(),
        });
    }

    fn get_clock(clocks: &CoreClocks) -> Option<Hertz> {
        let d1ccipr = unsafe { (*stm32::RCC::ptr()).d1ccipr.read() };

        match d1ccipr.qspisel().variant() {
            stm32::rcc::d1ccipr::QSPISEL_A::RCC_HCLK3 => Some(clocks.hclk()),
            stm32::rcc::d1ccipr::QSPISEL_A::PLL1_Q => clocks.pll1_q_ck(),
            stm32::rcc::d1ccipr::QSPISEL_A::PLL2_R => clocks.pll2_r_ck(),
            stm32::rcc::d1ccipr::QSPISEL_A::PER => clocks.per_ck(),
        }
    }

    /// Configure the operational mode of the QSPI interface.
    ///
    /// # Args
    /// * `mode` - The newly desired mode of the interface.
    ///
    /// # Errors
    /// Returns QspiError::Busy if an operation is ongoing
    pub fn configure_mode(&mut self, mode: QspiMode) -> Result<(), QspiError> {
        if self.is_busy() {
            return Err(QspiError::Busy);
        }

        self.rb.ccr.modify(|_, w| unsafe {
            w.admode()
                .bits(mode.reg_value())
                .dmode()
                .bits(mode.reg_value())
        });

        Ok(())
    }

    /// Begin a write over the QSPI interface. This is mostly useful for use with
    /// DMA or if you are managing the read yourself. If you want to complete a
    /// whole transaction, see the [`write`](#method.write) method.
    ///
    /// # Args
    /// * `addr` - The address to write data to. If the address size is less
    ///            than 32-bit, then unused bits are discarded.
    pub fn begin_write(
        &mut self,
        addr: u32,
        length: usize,
    ) -> Result<(), QspiError> {
        if self.is_busy() {
            return Err(QspiError::Busy);
        }

        // Clear the transfer complete flag.
        self.rb.fcr.modify(|_, w| w.ctcf().set_bit());

        // Write the length
        self.rb
            .dlr
            .write(|w| unsafe { w.dl().bits(length as u32 - 1) });

        // Configure the mode to indirect write.
        self.rb.ccr.modify(|_, w| unsafe { w.fmode().bits(0b00) });

        self.rb.ar.write(|w| unsafe { w.address().bits(addr) });

        Ok(())
    }

    /// Write data over the QSPI interface.
    ///
    /// # Args
    /// * `addr` - The address to write data to. If the address size is less
    ///            than 32-bit, then unused bits are discarded.
    /// * `data` - An array of data to transfer over the QSPI interface.
    ///
    /// # Panics
    ///
    /// Panics if the length of `data` is greater than the size of the QSPI
    /// hardware FIFO (32 bytes).
    pub fn write(&mut self, addr: u32, data: &[u8]) -> Result<(), QspiError> {
        assert!(
            data.len() <= 32,
            "Transactions larger than the QSPI FIFO are currently unsupported"
        );

        self.begin_write(addr, data.len())?;

        // Write data to the FIFO in a byte-wise manner.
        unsafe {
            for byte in data {
                ptr::write_volatile(&self.rb.dr as *const _ as *mut u8, *byte);
            }
        }

        // Wait for the transaction to complete
        while self.rb.sr.read().tcf().bit_is_clear() {}

        // Wait for the peripheral to indicate it is no longer busy.
        while self.is_busy() {}

        Ok(())
    }

    /// Begin a read over the QSPI interface. This is mostly useful for use with
    /// DMA or if you are managing the read yourself. If you want to complete a
    /// whole transaction, see the [`read`](#method.read) method.
    ///
    /// # Args
    /// * `addr` - The address to read data from. If the address size is less
    ///            than 32-bit, then unused bits are discarded.
    pub fn begin_read(
        &mut self,
        addr: u32,
        length: usize,
    ) -> Result<(), QspiError> {
        if self.is_busy() {
            return Err(QspiError::Busy);
        }

        // Clear the transfer complete flag.
        self.rb.fcr.modify(|_, w| w.ctcf().set_bit());

        // Write the length that should be read.
        self.rb
            .dlr
            .write(|w| unsafe { w.dl().bits(length as u32 - 1) });

        // Configure the mode to indirect read.
        self.rb.ccr.modify(|_, w| unsafe { w.fmode().bits(0b01) });

        // Write the address to force the read to start.
        self.rb.ar.write(|w| unsafe { w.address().bits(addr) });

        Ok(())
    }

    /// Read data over the QSPI interface.
    ///
    /// # Args
    /// * `addr` - The address to read data from. If the address size is less
    ///            than 32-bit, then unused bits are discarded.
    /// * `dest` - An array to store the result of the read into.
    ///
    /// # Panics
    ///
    /// Panics if the length of `data` is greater than the size of the QSPI
    /// hardware FIFO (32 bytes).
    pub fn read(
        &mut self,
        addr: u32,
        dest: &mut [u8],
    ) -> Result<(), QspiError> {
        assert!(
            dest.len() <= 32,
            "Transactions larger than the QSPI FIFO are currently unsupported"
        );

        // Begin the read operation
        self.begin_read(addr, dest.len())?;

        // Wait for the transaction to complete
        while self.rb.sr.read().tcf().bit_is_clear() {}

        // Check for underflow on the FIFO.
        if (self.rb.sr.read().flevel().bits() as usize) < dest.len() {
            return Err(QspiError::Underflow);
        }

        // Read data from the FIFO in a byte-wise manner.
        unsafe {
            for location in dest {
                *location =
                    ptr::read_volatile(&self.rb.dr as *const _ as *const u8);
            }
        }

        // Wait for the peripheral to indicate it is no longer busy.
        while self.is_busy() {}

        Ok(())
    }
}

impl QspiExt for stm32::QUADSPI {
    fn bank1<CONFIG, PINS>(
        self,
        _pins: PINS,
        config: CONFIG,
        clocks: &CoreClocks,
        prec: rec::Qspi,
    ) -> Qspi
    where
        CONFIG: Into<Config>,
        PINS: PinsBank1,
    {
        Qspi::qspi_unchecked(self, config, Bank::One, clocks, prec)
    }

    fn bank2<CONFIG, PINS>(
        self,
        _pins: PINS,
        config: CONFIG,
        clocks: &CoreClocks,
        prec: rec::Qspi,
    ) -> Qspi
    where
        CONFIG: Into<Config>,
        PINS: PinsBank2,
    {
        Qspi::qspi_unchecked(self, config, Bank::Two, clocks, prec)
    }

    fn qspi_unchecked<CONFIG>(
        self,
        config: CONFIG,
        bank: Bank,
        clocks: &CoreClocks,
        prec: rec::Qspi,
    ) -> Qspi
    where
        CONFIG: Into<Config>,
    {
        Qspi::qspi_unchecked(self, config, bank, clocks, prec)
    }
}
