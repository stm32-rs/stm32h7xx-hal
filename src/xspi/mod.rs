//! Quad or Octo SPI bus
//!
//! STM32H7 parts support either Quad or Octo SPI using dedicated peripheral(s).
//!
//! | Interface | Parts | # IO lines |
//! | --- | --- | --- |
//! | Quad Spi | H742/743/750/753/747/757 | 1-bit, 2-bit or 4-bit |
//! | Octo Spi | H725/735/7a3/7b0/7b3 | 1-bit, 2-bit, 4-bit or 8-bit |
//!
//! # Usage
//!
//! This driver supports using the xSPI peripheral in indirect mode. This allows
//! the peripheral to be used to read and write from an address over a
//! SPI/QUADSPI/OCTOSPI interface.
//!
//! The QUADSPI interface can be configured to operate on either of the two
//! available banks.
//!
//! ```
//! use stm32h7xx_hal::xspi;
//!
//! // Get the device peripherals and instantiate IO pins.
//! let dp = ...;
//! let (sck, io0, io1, io2, io3) = ...;
//!
//! let mut qspi = dp.QUADSPI.bank1((sck, io0, io1, io2, io3), 3.mhz(), &ccdr.clocks,
//!                                 ccdr.peripheral.QSPI);
//!
//! // Configure QSPI to operate in 4-bit mode.
//! qspi.configure_mode(xspi::QspiMode::FourBit).unwrap();
//!
//! // Write data to address 0x00 on the QSPI interface.
//! qspi.write(0x00, &[0xAB, 0xCD]).unwrap();
//! ```
//!
//! # Limitations
//!
//! This driver currently only supports indirect operation mode of the xSPI
//! interface. Automatic polling or memory-mapped modes are not supported.

// Parts of the Quad and Octo SPI support are shared (this file), but they are
// different enough to require different initialisation routines and pin
// allocations (for example). The Octospi peripheral appears to be 'loosly'
// based upon the (older?) Quadspi peripheral

// Quadspi
#[cfg(any(feature = "rm0433", feature = "rm0399"))]
mod qspi;
#[cfg(any(feature = "rm0433", feature = "rm0399"))]
pub use common::{
    Bank, Xspi as Qspi, XspiError as QspiError, XspiMode as QspiMode,
};
#[cfg(any(feature = "rm0433", feature = "rm0399"))]
pub use qspi::QspiExt as XspiExt;

// Octospi
#[cfg(any(feature = "rm0455", feature = "rm0468"))]
mod octospi;
#[cfg(any(feature = "rm0455", feature = "rm0468"))]
pub use common::{
    Xspi as Octospi, XspiError as OctospiError, XspiMode as OctospiMode,
};
#[cfg(any(feature = "rm0455", feature = "rm0468"))]
pub use octospi::OctospiExt as XspiExt;

// Both
pub use common::{AddressSize, Config, Event, SamplingEdge};

/// This modulate contains functionality common to both Quad and Octo SPI
mod common {
    use crate::{
        rcc::{rec, CoreClocks},
        stm32,
        time::Hertz,
    };
    use core::{marker::PhantomData, ptr};

    /// Represents operation modes of the XSPI interface.
    #[derive(Debug, Copy, Clone, PartialEq)]
    pub enum XspiMode {
        /// Only a single IO line (IO0) is used for transmit and a separate line
        /// (IO1) is used for receive.
        OneBit,

        /// Two IO lines (IO0 and IO1) are used for transmit/receive.
        TwoBit,

        /// Four IO lines are used for transmit/receive.
        FourBit,

        #[cfg(any(feature = "rm0455", feature = "rm0468"))]
        /// Eight IO lines are used for transmit/receive.
        EightBit,
    }
    impl XspiMode {
        pub(super) fn reg_value(&self) -> u8 {
            match self {
                XspiMode::OneBit => 1,
                XspiMode::TwoBit => 2,
                XspiMode::FourBit => 3,
                #[cfg(any(feature = "rm0455", feature = "rm0468"))]
                XspiMode::EightBit => 4,
            }
        }
    }
    /// Indicates an error with the XSPI peripheral.
    #[derive(Debug, Copy, Clone, PartialEq)]
    pub enum XspiError {
        Busy,
        Underflow,
    }

    /// Address sizes used by the XSPI interface
    #[derive(Debug, Copy, Clone, PartialEq)]
    pub enum AddressSize {
        EightBit,
        SixteenBit,
        TwentyFourBit,
        ThirtyTwoBit,
    }

    /// Sampling mode for the XSPI interface
    #[derive(Debug, Copy, Clone, PartialEq)]
    pub enum SamplingEdge {
        Falling,
        Rising,
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

    /// Indicates a specific QUADSPI bank to use
    #[derive(Debug, Copy, Clone, PartialEq)]
    pub enum Bank {
        One,
        Two,
        Dual,
    }
    // Banks are not supported by the Octospi peripheral (there's two Octospi
    // peripherals instead)

    /// A structure for specifying the XSPI configuration.
    ///
    /// This structure uses builder semantics to generate the configuration.
    ///
    /// ```
    /// let config = Config::new().dummy_cycles(1);
    /// ```
    #[derive(Copy, Clone)]
    pub struct Config {
        pub(super) mode: XspiMode,
        pub(super) frequency: Hertz,
        pub(super) address_size: AddressSize,
        pub(super) dummy_cycles: u8,
        pub(super) sampling_edge: SamplingEdge,
        pub(super) fifo_threshold: u8,
    }

    impl Config {
        /// Create a default configuration for the XSPI interface.
        ///
        /// * Bus in 1-bit Mode
        /// * 8-bit Address
        /// * No dummy cycle
        /// * Sample on falling edge
        pub fn new<T: Into<Hertz>>(freq: T) -> Self {
            Config {
                mode: XspiMode::OneBit,
                frequency: freq.into(),
                address_size: AddressSize::EightBit,
                dummy_cycles: 0,
                sampling_edge: SamplingEdge::Falling,
                fifo_threshold: 1,
            }
        }

        /// Specify the operating mode of the XSPI bus. Can be 1-bit, 2-bit or
        /// 4-bit for Quadspi; 1-bit, 2-bit, 4-bit or 8-bit for Octospi.
        ///
        /// The operating mode can also be changed using the
        /// [`configure_mode`](Xspi#method.configure_mode) method
        pub fn mode(mut self, mode: XspiMode) -> Self {
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
        /// With zero dummy cycles, the QUADSPI peripheral will erroneously drive the
        /// output pins for an extra half clock cycle before IO is swapped from
        /// output to input. Refer to
        /// https://github.com/quartiq/stabilizer/issues/101 for more information.
        pub fn dummy_cycles(mut self, cycles: u8) -> Self {
            debug_assert!(
                cycles < 32,
                "Hardware only supports 0-31 dummy cycles"
            );

            self.dummy_cycles = cycles;
            self
        }

        /// Specify the sampling edge for the XSPI receiver.
        ///
        /// # Note
        ///
        /// If zero dummy cycles are used, during read operations the QUADSPI
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

        /// Specify the number of bytes in the FIFO that will set the FIFO threshold
        /// flag. Must be in the range 1-32 inclusive.
        ///
        /// In indirect write mode, this is the number of free bytes that will raise
        /// the FIFO threshold flag.
        ///
        /// In indirect read mode, this is the number of valid pending bytes that
        /// will raise the FIFO threshold flag.
        pub fn fifo_threshold(mut self, threshold: u8) -> Self {
            debug_assert!(threshold > 0 && threshold <= 32);

            self.fifo_threshold = threshold;
            self
        }
    }

    impl<T: Into<Hertz>> From<T> for Config {
        fn from(frequency: T) -> Self {
            Self::new(frequency)
        }
    }

    /// Generic type for Quad or Octo SPI
    pub struct Xspi<XSPI> {
        pub(super) rb: XSPI,
    }

    #[cfg(any(feature = "rm0433", feature = "rm0399"))]
    macro_rules! fmode_reg {
        ($e:expr) => {
            $e.rb.ccr
        };
    }
    #[cfg(any(feature = "rm0455", feature = "rm0468"))]
    macro_rules! fmode_reg {
        ($e:expr) => {
            $e.rb.cr
        };
    }

    // This macro uses the paste::item! macro to create identifiers.
    //
    // https://crates.io/crates/paste
    macro_rules! xspi_impl {
        ($peripheral:expr, $rec:expr, $ccip:ident) => {
    paste::item! {
        impl Xspi<$peripheral> {
            /// Deconstructs the XSPI HAL and returns the component parts
            pub fn free(self) -> ($peripheral, $rec) {
                (
                    self.rb,
                    $rec {
                        _marker: PhantomData,
                    },
                )
            }

            /// Returns a reference to the inner peripheral
            pub fn inner(&self) -> &$peripheral {
                &self.rb
            }

            /// Returns a mutable reference to the inner peripheral
            pub fn inner_mut(&mut self) -> &mut $peripheral {
                &mut self.rb
            }

            /// Check if the XSPI peripheral is currently busy with a
            /// transaction
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
                let _ = self.rb.cr.read();
                let _ = self.rb.cr.read(); // Delay 2 peripheral clocks
            }

            pub(super) fn get_clock(clocks: &CoreClocks) -> Option<Hertz> {
                let d1ccipr = unsafe { (*stm32::RCC::ptr()).d1ccipr.read() };

                match d1ccipr.$ccip().variant() {
                    stm32::rcc::d1ccipr::[< $ccip:upper _A >]::RCC_HCLK3 => Some(clocks.hclk()),
                    stm32::rcc::d1ccipr::[< $ccip:upper _A >]::PLL1_Q => clocks.pll1_q_ck(),
                    stm32::rcc::d1ccipr::[< $ccip:upper _A >]::PLL2_R => clocks.pll2_r_ck(),
                    stm32::rcc::d1ccipr::[< $ccip:upper _A >]::PER => clocks.per_ck(),
                }
            }

            /// Configure the operational mode of the XSPI interface.
            ///
            /// # Args
            /// * `mode` - The newly desired mode of the interface.
            ///
            /// # Errors
            /// Returns XspiError::Busy if an operation is ongoing
            pub fn configure_mode(&mut self, mode: XspiMode) -> Result<(), XspiError> {
                if self.is_busy() {
                    return Err(XspiError::Busy);
                }

                self.rb.ccr.modify(|_, w| unsafe {
                    w.admode()
                        .bits(mode.reg_value())
                        .dmode()
                        .bits(mode.reg_value())
                });

                // if mode == XspiMode::EightBit {
                //     // TODO
                //     self.rb.ccr.modify(|_, w| unsafe {
                //         w.admode()
                //             .bits(mode.reg_value())
                //             .adsize()
                //             .bits(3) // 32-bit
                //             .isize()
                //             .bits(1) // 16-bit
                //             .imode()
                //             .bits(mode.reg_value())
                //             .dmode()
                //             .bits(mode.reg_value())
                //     });
                //     self.rb.tcr.write(|w| unsafe { w.dcyc().bits(4) });
                // }

                Ok(())
            }

            /// Begin a write over the XSPI interface. This is mostly useful for use with
            /// DMA or if you are managing the read yourself. If you want to complete a
            /// whole transaction, see the [`write`](#method.write) method.
            ///
            /// # Args
            /// * `addr` - The address to write data to. If the address size is less
            ///            than 32-bit, then unused bits are discarded.
            /// * `length` - The length of the write operation in bytes
            pub fn begin_write(
                &mut self,
                addr: u32,
                length: usize,
            ) -> Result<(), XspiError> {
                if self.is_busy() {
                    return Err(XspiError::Busy);
                }

                // Clear the transfer complete flag.
                self.rb.fcr.write(|w| w.ctcf().set_bit());

                // Write the length
                self.rb
                    .dlr
                    .write(|w| unsafe { w.dl().bits(length as u32 - 1) });

                // Configure the mode to indirect write.
                fmode_reg!(self).modify(|_, w| unsafe { w.fmode().bits(0b00) });

                // Write the address to force the write to start
                self.rb.ar.write(|w| unsafe { w.address().bits(addr) });

                Ok(())
            }

            /// Write data over the XSPI interface.
            ///
            /// # Args
            /// * `addr` - The address to write data to. If the address size is less
            ///            than 32-bit, then unused bits are discarded.
            /// * `data` - An array of data to transfer over the XSPI interface.
            ///
            /// # Panics
            ///
            /// Panics if the length of `data` is greater than the size of the XSPI
            /// hardware FIFO (32 bytes).
            pub fn write(&mut self, addr: u32, data: &[u8]) -> Result<(), XspiError> {
                assert!(
                    data.len() <= 32,
                    "Transactions larger than the XSPI FIFO are currently unsupported"
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

            /// Begin a read over the XSPI interface. This is mostly useful for use with
            /// DMA or if you are managing the read yourself. If you want to complete a
            /// whole transaction, see the [`read`](#method.read) method.
            ///
            /// # Args
            /// * `addr` - The address to read data from. If the address size is less
            ///            than 32-bit, then unused bits are discarded.
            /// * `length` - The length of the read operation in bytes
            pub fn begin_read(
                &mut self,
                addr: u32,
                length: usize,
            ) -> Result<(), XspiError> {
                if self.is_busy() {
                    return Err(XspiError::Busy);
                }

                // Clear the transfer complete flag.
                self.rb.fcr.write(|w| w.ctcf().set_bit());

                // Write the length that should be read.
                self.rb
                    .dlr
                    .write(|w| unsafe { w.dl().bits(length as u32 - 1) });

                // Configure the mode to indirect read.
                fmode_reg!(self).modify(|_, w| unsafe { w.fmode().bits(0b01) });

                // Write the address to force the read to start.
                self.rb.ar.write(|w| unsafe { w.address().bits(addr) });

                Ok(())
            }

            /// Read data over the XSPI interface.
            ///
            /// # Args
            /// * `addr` - The address to read data from. If the address size is less
            ///            than 32-bit, then unused bits are discarded.
            /// * `dest` - An array to store the result of the read into.
            ///
            /// # Panics
            ///
            /// Panics if the length of `data` is greater than the size of the XSPI
            /// hardware FIFO (32 bytes).
            pub fn read(
                &mut self,
                addr: u32,
                dest: &mut [u8],
            ) -> Result<(), XspiError> {
                assert!(
                    dest.len() <= 32,
                    "Transactions larger than the XSPI FIFO are currently unsupported"
                );

                // Begin the read operation
                self.begin_read(addr, dest.len())?;

                // Wait for the transaction to complete
                while self.rb.sr.read().tcf().bit_is_clear() {}

                // Check for underflow on the FIFO.
                if (self.rb.sr.read().flevel().bits() as usize) < dest.len() {
                    return Err(XspiError::Underflow);
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
        }}}
    }

    #[cfg(any(feature = "rm0433", feature = "rm0399"))]
    xspi_impl! { stm32::QUADSPI, rec::Qspi, qspisel }

    #[cfg(any(feature = "rm0455", feature = "rm0468"))]
    xspi_impl! { stm32::OCTOSPI1, rec::Octospi1, octospisel }
    #[cfg(any(feature = "rm0455", feature = "rm0468"))]
    xspi_impl! { stm32::OCTOSPI2, rec::Octospi2, octospisel }
}
