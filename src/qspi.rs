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
//! This driver currently only supports indirect operation mode of the QSPI interface. It
//! implements an 8-bit address followed by an arbitrary transaction length. It supports using
//! either bank 1 or bank 2 as well as a dual flash bank (in which all 8 IOs are used for the
//! interface).
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

use core::ptr;

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
    fn bank1<T, PINS>(
        self,
        _pins: PINS,
        frequency: T,
        clocks: &CoreClocks,
        prec: rec::Qspi,
    ) -> Qspi
    where
        T: Into<Hertz>,
        PINS: PinsBank1;

    fn bank2<T, PINS>(
        self,
        _pins: PINS,
        frequency: T,
        clocks: &CoreClocks,
        prec: rec::Qspi,
    ) -> Qspi
    where
        T: Into<Hertz>,
        PINS: PinsBank2;

    fn qspi_unchecked<T>(
        self,
        frequency: T,
        bank: Bank,
        clocks: &CoreClocks,
        prec: rec::Qspi,
    ) -> Qspi
    where
        T: Into<Hertz>;
}

pub struct Qspi {
    rb: stm32::QUADSPI,
}

impl Qspi {
    pub fn bank1<T, PINS>(
        regs: stm32::QUADSPI,
        _pins: PINS,
        frequency: T,
        clocks: &CoreClocks,
        prec: rec::Qspi,
    ) -> Self
    where
        T: Into<Hertz>,
        PINS: PinsBank1,
    {
        Self::qspi_unchecked(regs, frequency, Bank::One, clocks, prec)
    }

    pub fn bank2<T, PINS>(
        regs: stm32::QUADSPI,
        _pins: PINS,
        frequency: T,
        clocks: &CoreClocks,
        prec: rec::Qspi,
    ) -> Self
    where
        T: Into<Hertz>,
        PINS: PinsBank2,
    {
        Self::qspi_unchecked(regs, frequency, Bank::Two, clocks, prec)
    }

    pub fn qspi_unchecked<T>(
        regs: stm32::QUADSPI,
        frequency: T,
        bank: Bank,
        clocks: &CoreClocks,
        prec: rec::Qspi,
    ) -> Self
    where
        T: Into<Hertz>,
    {
        prec.enable();

        // Disable QUADSPI before configuring it.
        regs.cr.write(|w| w.en().clear_bit());

        let spi_kernel_ck = match Self::get_clock(clocks) {
            Some(freq_hz) => freq_hz.0,
            _ => panic!("QSPI kernel clock not running!"),
        };

        while regs.sr.read().busy().bit_is_set() {}

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
                .bits(0)
                .dmode()
                .bits(0b01)
                .admode()
                .bits(0b01)
                .adsize()
                .bits(0)
                .imode()
                .bits(0)
                .dcyc()
                .bits(0)
        });

        let spi_frequency = frequency.into().0;
        let divisor = match (spi_kernel_ck + spi_frequency - 1) / spi_frequency
        {
            divisor @ 1..=256 => divisor - 1,
            _ => panic!("Invalid QSPI frequency requested"),
        };

        // Write the prescaler and the SSHIFT bit. Note that SSHIFT is required because it appears
        // that the QSPI may have signal contention issues when reading. SSHIFT forces the read to
        // occur on the falling edge instead of the rising edge. Refer to
        // https://github.com/quartiq/stabilizer/issues/101 for more information
        //
        // This is also noted in the docstring for the read() method.
        //
        // SSHIFT must not be set in DDR mode.
        regs.cr.write(|w| unsafe {
            w.prescaler().bits(divisor as u8).sshift().set_bit()
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

    /// Check if the QSPI peripheral is currently busy with a transaction.
    pub fn is_busy(&self) -> bool {
        self.rb.sr.read().busy().bit_is_set()
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
    pub fn configure_mode(&mut self, mode: QspiMode) -> Result<(), QspiError> {
        if self.is_busy() {
            return Err(QspiError::Busy);
        }

        match mode {
            QspiMode::OneBit => {
                self.rb.ccr.modify(|_, w| unsafe {
                    w.admode().bits(0b01).dmode().bits(0b01)
                });
            }
            QspiMode::TwoBit => {
                self.rb.ccr.modify(|_, w| unsafe {
                    w.admode().bits(0b10).dmode().bits(0b10)
                });
            }
            QspiMode::FourBit => {
                self.rb.ccr.modify(|_, w| unsafe {
                    w.admode().bits(0b11).dmode().bits(0b11)
                });
            }
        }

        Ok(())
    }

    /// Write data over the QSPI interface.
    ///
    /// # Args
    /// * `addr` - The address to write data to.
    /// * `data` - An array of data to transfer over the QSPI interface.
    pub fn write(&mut self, addr: u8, data: &[u8]) -> Result<(), QspiError> {
        if self.is_busy() {
            return Err(QspiError::Busy);
        }

        // Clear the transfer complete flag.
        self.rb.fcr.modify(|_, w| w.ctcf().set_bit());

        // Write the length
        self.rb
            .dlr
            .write(|w| unsafe { w.dl().bits(data.len() as u32 - 1) });

        // Configure the mode to indirect write.
        self.rb.ccr.modify(|_, w| unsafe { w.fmode().bits(0b00) });

        self.rb
            .ar
            .write(|w| unsafe { w.address().bits(addr as u32) });

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

    /// Read data over the QSPI interface.
    ///
    /// # Note
    ///
    /// Without any dummy cycles, the QSPI peripheral will erroneously drive the
    /// output pins for an extra half clock cycle before IO is swapped from
    /// output to input. Refer to
    /// https://github.com/quartiq/stabilizer/issues/101 for more information.
    ///
    /// Although it doesn't stop the possible bus contention, this HAL sets the
    /// SSHIFT bit in the CR register. With this bit set, the QSPI receiver
    /// sampling point is delayed by an extra half cycle. Then the receiver
    /// sampling point is after the bus contention.
    ///
    /// # Args
    /// * `addr` - The address to read data from.
    /// * `dest` - An array to store the result of the read into.
    pub fn read(&mut self, addr: u8, dest: &mut [u8]) -> Result<(), QspiError> {
        if self.is_busy() {
            return Err(QspiError::Busy);
        }

        // Clear the transfer complete flag.
        self.rb.fcr.modify(|_, w| w.ctcf().set_bit());

        // Write the length that should be read.
        self.rb
            .dlr
            .write(|w| unsafe { w.dl().bits(dest.len() as u32 - 1) });

        // Configure the mode to indirect read.
        self.rb.ccr.modify(|_, w| unsafe { w.fmode().bits(0b01) });

        // Write the address to force the read to start.
        self.rb
            .ar
            .write(|w| unsafe { w.address().bits(addr as u32) });

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
    fn bank1<T, PINS>(
        self,
        _pins: PINS,
        frequency: T,
        clocks: &CoreClocks,
        prec: rec::Qspi,
    ) -> Qspi
    where
        T: Into<Hertz>,
        PINS: PinsBank1,
    {
        Qspi::qspi_unchecked(self, frequency, Bank::One, clocks, prec)
    }

    fn bank2<T, PINS>(
        self,
        _pins: PINS,
        frequency: T,
        clocks: &CoreClocks,
        prec: rec::Qspi,
    ) -> Qspi
    where
        T: Into<Hertz>,
        PINS: PinsBank2,
    {
        Qspi::qspi_unchecked(self, frequency, Bank::Two, clocks, prec)
    }

    fn qspi_unchecked<T>(
        self,
        frequency: T,
        bank: Bank,
        clocks: &CoreClocks,
        prec: rec::Qspi,
    ) -> Qspi
    where
        T: Into<Hertz>,
    {
        Qspi::qspi_unchecked(self, frequency, bank, clocks, prec)
    }
}
