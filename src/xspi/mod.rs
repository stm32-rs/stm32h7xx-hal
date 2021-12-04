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
//! For OCTOSPI there are two peripherals, which can be initialised separately.
//!
//! ```
//! use stm32h7xx_hal::{xspi, xspi::OctospiWord as XW};
//!
//! // Get the device peripherals and instantiate IO pins.
//! let dp = ...;
//! let _ = ...;
//!
//! let mut octospi = dp.OCTOSPI1.octospi_unchecked(12.mhz(), &ccdr.clocks,
//!                                 ccdr.peripheral.OCTOSPI1);
//!
//! // Configure OCTOSPI to operate in 8-bit mode.
//! octospi.configure_mode(xspi::OctospiMode::EightBit).unwrap();
//!
//! // Example RDID Read Indentification
//! let mut read: [u8; 3] = [0; 3];
//! octospi
//!     .read_extended(XW::U16(0x9F60), XW::U32(0), XW::None, 4, &mut read)
//!     .unwrap();
//! ```
//!
//! # Configuration
//!
//! A [`Config`](#struct.Config) struct is used to configure the xSPI.
//!
//! ```
//! use stm32h7xx_hal::xspi;
//! let config = xspi::Config::new(12.mhz()).fifo_threshold(16);
//!
//! # Hyperbus
//!
//! This driver supports a memory-mapped Hyperbus mode for the OCTOSPI
//! peripheral.
//!
//! ```
//! let config = HyperbusConfig::new(80.mhz())
//!     .device_size_bytes(24) // 16 Mbyte
//!     .refresh_interval(4.us())
//!     .read_write_recovery(4) // 50ns
//!     .access_initial_latency(6);
//!
//! let hyperram = dp.OCTOSPI1.octospi_hyperbus_unchecked(
//!     config,
//!     &ccdr.clocks,
//!     ccdr.peripheral.OCTOSPI1,
//! );
//!
//! // Initialise and convert raw pointer to slice
//! let ram_ptr: *mut u32 = hyperram.init();
//! let ram = unsafe { slice::from_raw_parts_mut(ram_ptr, size_u32) };
//! ```
//!
//! # Examples
//!
//! - [Simple QSPI example](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/qspi.rs)
//! - [QSPI memory usage](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/qspi_flash_memory.rs)
//! - [QSPI using MDMA](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/qspi_mdma.rs)
//! - [OCTOSPI memory usage](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/octospi.rs)
//!
//! # Limitations
//!
//! This driver currently only supports indirect operation mode of the xSPI
//! interface. Automatic polling or memory-mapped modes are not supported,
//! except for the OCTOSPI Hyperbus mode.
//!
//! Using different operational modes (1-bit/2-bit/4-bit etc.) for different
//! phases of a single transaction is not supported. It is possible to change
//! operational mode between transactions by calling
//! [`configure_mode`](#method.configure_mode).

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
    XspiWord as QspiWord,
};
#[cfg(any(feature = "rm0433", feature = "rm0399"))]
pub use qspi::QspiExt as XspiExt;

// Octospi
#[cfg(any(feature = "rm0455", feature = "rm0468"))]
mod octospi;
#[cfg(any(feature = "rm0455", feature = "rm0468"))]
pub use common::{
    Xspi as Octospi, XspiError as OctospiError, XspiMode as OctospiMode,
    XspiWord as OctospiWord,
};
#[cfg(any(feature = "rm0455", feature = "rm0468"))]
pub use octospi::{Hyperbus, HyperbusConfig, OctospiExt as XspiExt};

// Both
pub use common::{Config, Event, SamplingEdge};

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
        #[inline(always)]
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

        #[cfg(any(feature = "rm0433", feature = "rm0399"))]
        /// The specified XspiWord does not fit in the available register
        WordTooLarge,
    }

    /// Instruction, Address or Alternate Byte word used by the XSPI interface
    #[derive(Debug, Copy, Clone, PartialEq)]
    pub enum XspiWord {
        None,
        U8(u8),
        U16(u16),
        U24(u32),
        U32(u32),
    }
    impl XspiWord {
        #[inline(always)]
        fn size(&self) -> u8 {
            match self {
                XspiWord::U16(_) => 1,
                XspiWord::U24(_) => 2,
                XspiWord::U32(_) => 3,
                _ => 0, // 8-bit
            }
        }
        #[inline(always)]
        fn bits(self) -> u32 {
            match self {
                XspiWord::None => 0,
                XspiWord::U8(x) => x as u32,
                XspiWord::U16(x) => x as u32,
                XspiWord::U24(x) | XspiWord::U32(x) => x,
            }
        }
        #[inline(always)]
        #[cfg(any(feature = "rm0433", feature = "rm0399"))]
        fn bits_u8(self) -> Result<u8, XspiError> {
            match self {
                XspiWord::None => Ok(0),
                XspiWord::U8(x) => Ok(x),
                _ => Err(XspiError::WordTooLarge),
            }
        }
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
    #[cfg(any(feature = "rm0433", feature = "rm0399"))]
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
        pub(super) dummy_cycles: u8,
        pub(super) sampling_edge: SamplingEdge,
        pub(super) fifo_threshold: u8,
    }

    impl Config {
        /// Create a default configuration for the XSPI interface.
        ///
        /// * Bus in 1-bit Mode
        /// * No dummy cycle
        /// * Sample on falling edge
        pub fn new<T: Into<Hertz>>(freq: T) -> Self {
            Config {
                mode: XspiMode::OneBit,
                frequency: freq.into(),
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
        /// <https://github.com/quartiq/stabilizer/issues/101> for more information.
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
        /// <https://github.com/quartiq/stabilizer/issues/101> for more information.
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

        /// We store the current mode here because for extended transactions
        /// various phases may be removed. Therefore we need to restore them
        /// after each transaction.
        pub(super) mode: XspiMode,
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
            pub fn is_busy(&self) -> Result<(), XspiError> {
                if self.rb.sr.read().busy().bit_is_set() {
                    Err(XspiError::Busy)
                } else {
                    Ok(())
                }
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

            pub fn kernel_clk_unwrap(clocks: &CoreClocks) -> Hertz {
                #[cfg(not(feature = "rm0455"))]
                use stm32::rcc::d1ccipr as ccipr;
                #[cfg(feature = "rm0455")]
                use stm32::rcc::cdccipr as ccipr;

                #[cfg(not(feature = "rm0455"))]
                let ccipr = unsafe { (*stm32::RCC::ptr()).d1ccipr.read() };
                #[cfg(feature = "rm0455")]
                let ccipr = unsafe { (*stm32::RCC::ptr()).cdccipr.read() };

                match ccipr.$ccip().variant() {
                    ccipr::[< $ccip:upper _A >]::RCC_HCLK3 => clocks.hclk(),
                    ccipr::[< $ccip:upper _A >]::PLL1_Q => {
                        clocks.pll1_q_ck().expect("$peripheral: PLL1_Q must be enabled")
                    }
                    ccipr::[< $ccip:upper _A >]::PLL2_R => {
                        clocks.pll2_r_ck().expect("$peripheral: PLL2_R must be enabled")
                    }
                    ccipr::[< $ccip:upper _A >]::PER => {
                        clocks.per_ck().expect("$peripheral: PER clock must be enabled")
                    }
                }
            }

            /// Configure the operational mode (number of bits) of the XSPI
            /// interface.
            ///
            /// # Args
            /// * `mode` - The newly desired mode of the interface.
            ///
            /// # Errors
            /// Returns XspiError::Busy if an operation is ongoing
            pub fn configure_mode(&mut self, mode: XspiMode) -> Result<(), XspiError> {
                self.is_busy()?;
                self.mode = mode;
                self.set_mode_address_data_only();

                Ok(())
            }

            /// Sets the interface to 8-bit address and data only using the
            /// current operational mode (number of bits).
            fn set_mode_address_data_only(&mut self) {
                self.rb.ccr.modify(|_, w| unsafe {
                    w.imode()     // NO instruction phase
                        .bits(0)
                        .admode() // address phase
                        .bits(self.mode.reg_value())
                        .adsize()
                        .bits(0)  // 8-bit address
                        .abmode() // NO alternate-bytes phase
                        .bits(0)
                        .dmode()  // data phase
                        .bits(self.mode.reg_value())
                });
            }

            /// Sets the interface in extended mode
            ///
            /// # Args
            /// * `instruction` - The word to be used for the instruction phase.
            /// * `address` - The word to be used for the address phase.
            /// * `alternate_bytes` - The word to be used for the alternate-bytes phase.
            /// * `dummy_cycles` - The number of dummy cycles between the alternate-bytes
            ///                    and the data phase.
            /// * `data` - true is there is a data phase, false for no data phase.
            fn setup_extended(&mut self, instruction: XspiWord, address: XspiWord,
                              alternate_bytes: XspiWord, dummy_cycles: u8, data: bool, read: bool) {

                let fmode = if read { 0b01 } else { 0b00 };
                let mode = self.mode.reg_value();
                let imode = if instruction != XspiWord::None { mode } else { 0 };
                let admode = if address != XspiWord::None { mode } else { 0 };
                let abmode = if alternate_bytes != XspiWord::None { mode } else { 0 };
                let dmode = if data { mode } else { 0 };

                //writing to ccr will trigger the start of a transaction if there is no address or
                //data rm0433 pg 894, so we do it all in one go
                self.rb.ccr.modify(|_, w| unsafe {
                    #[cfg(any(feature = "rm0433", feature = "rm0399"))]
                    let w = {
                        let ir = instruction.bits_u8().unwrap();
                        w.dcyc().bits(dummy_cycles).instruction().bits(ir).fmode().bits(fmode)
                    };

                    #[cfg(any(feature = "rm0455", feature = "rm0468"))]
                    let w = w.isize().bits(instruction.size());

                    w.imode()
                        .bits(imode)
                        .admode()
                        .bits(admode)
                        .adsize()
                        .bits(address.size())
                        .abmode()
                        .bits(abmode)
                        .absize()
                        .bits(alternate_bytes.size())
                        .dmode()
                        .bits(dmode)
                });

                #[cfg(any(feature = "rm0455", feature = "rm0468"))]
                {
                    self.rb.tcr.write(|w| unsafe { w.dcyc().bits(dummy_cycles) });
                    self.rb.cr.modify(|_, w| unsafe { w.fmode().bits(fmode) });
                }

                // Write alternate-bytes
                self.rb.abr.write(|w| unsafe {
                    w.alternate().bits(alternate_bytes.bits())
                });

                #[cfg(any(feature = "rm0455", feature = "rm0468"))]
                if instruction != XspiWord::None {
                    self.rb.ir.write(|w| unsafe {
                        w.instruction().bits(instruction.bits())
                    });
                }

                if address != XspiWord::None {
                    // Write the address. The transaction starts on the next write
                    // to DATA, unless there is no DATA phase configured, in which
                    // case it starts here.
                    self.rb.ar.write(|w| unsafe { w.address().bits(address.bits()) });
                }
            }

            /// Begin a write over the XSPI interface. This is mostly useful for use with
            /// DMA or if you are managing the read yourself. If you want to complete a
            /// whole transaction, see the [`write`](#method.write) method.
            ///
            /// # Args
            /// * `addr` - The address to write data to. If the address size is less
            ///            than 32-bit, then unused bits are discarded.
            /// * `length` - The length of the write operation in bytes. Must be greater
            ///             than zero
            pub fn begin_write(
                &mut self,
                addr: u32,
                length: usize,
            ) -> Result<(), XspiError> {
                self.is_busy()?;

                // Exit extended mode
                self.set_mode_address_data_only();

                // Clear the transfer complete flag.
                self.rb.fcr.write(|w| w.ctcf().set_bit());

                // Write the length
                self.rb
                    .dlr
                    .write(|w| unsafe { w.dl().bits(length as u32 - 1) });

                // Configure the mode to indirect write.
                fmode_reg!(self).modify(|_, w| unsafe { w.fmode().bits(0b00) });

                // Write the address. The transaction starts on the next write
                // to DATA, unless there is no DATA phase configured, in which
                // case it starts here.
                self.rb.ar.write(|w| unsafe { w.address().bits(addr) });

                Ok(())
            }

            /// Write data over the XSPI interface.
            ///
            /// # Args
            /// * `addr` - The address to write data to
            /// * `data` - An array of data to transfer over the XSPI interface.
            ///
            /// # Panics
            ///
            /// Panics if the length of `data` is greater than the size of the XSPI
            /// hardware FIFO (32 bytes).
            pub fn write(&mut self, addr: u8, data: &[u8]) -> Result<(), XspiError> {
                assert!(
                    data.len() <= 32,
                    "Transactions larger than the XSPI FIFO are currently unsupported"
                );
                assert!(
                    !data.is_empty(),
                    "Must have a non-zero number of data cycles"
                );

                self.begin_write(addr as u32, data.len())?;

                // Write data to the FIFO in a byte-wise manner.
                unsafe {
                    for byte in data {
                        ptr::write_volatile(&self.rb.dr as *const _ as *mut u8, *byte);
                    }
                }

                // Wait for the transaction to complete
                while self.rb.sr.read().tcf().bit_is_clear() {}

                // Wait for the peripheral to indicate it is no longer busy.
                while self.is_busy().is_err() {}

                Ok(())
            }

            /// Write data over the XSPI interface, using an extended
            /// transaction that may contain instruction, address, alternate-bytes
            /// and data phases with various sizes.
            ///
            /// # Args
            /// * `instruction` - The word to be used for the instruction phase.
            /// * `address` - The word to be used for the address phase.
            /// * `alternate_bytes` - The word to be used for the alternate-bytes phase.
            /// * `data` - An array of data to transfer over the XSPI interface. Use
            ///            and empty slice to remove the data phase entirely.
            ///
            /// # Panics
            ///
            /// Panics if the length of `data` is greater than the size of the XSPI
            /// hardware FIFO (32 bytes).
            pub fn write_extended(&mut self,
                                  instruction: XspiWord,
                                  address: XspiWord,
                                  alternate_bytes: XspiWord,
                                  data: &[u8]) -> Result<(), XspiError> {
                assert!(
                    data.len() <= 32,
                    "Transactions larger than the XSPI FIFO are currently unsupported"
                );

                self.is_busy()?;

                // Clear the transfer complete flag.
                self.rb.fcr.write(|w| w.ctcf().set_bit());

                // Data length
                if !data.is_empty() {
                    self.rb
                        .dlr
                        .write(|w| unsafe { w.dl().bits(data.len() as u32 - 1) });
                }

                // Setup extended mode. Typically no dummy cycles in write mode
                self.setup_extended(instruction, address, alternate_bytes, 0, !data.is_empty(), false);

                // Write data to the FIFO in a byte-wise manner.
                // Transaction starts here
                unsafe {
                    for byte in data {
                        ptr::write_volatile(&self.rb.dr as *const _ as *mut u8, *byte);
                    }
                }

                // Wait for the transaction to complete
                while self.rb.sr.read().tcf().bit_is_clear() {}

                // Wait for the peripheral to indicate it is no longer busy.
                while self.is_busy().is_err() {}

                Ok(())
            }

            /// Begin a read over the XSPI interface. This is mostly useful for use with
            /// DMA or if you are managing the read yourself. If you want to complete a
            /// whole transaction, see the [`read`](#method.read) method.
            ///
            /// # Args
            /// * `addr` - The address to read data from. If the address size is less
            ///            than 32-bit, then unused bits are discarded.
            /// * `length` - The length of the read operation in bytes. Must be greater
            ///              than zero
            pub fn begin_read(
                &mut self,
                addr: u32,
                length: usize,
            ) -> Result<(), XspiError> {
                self.is_busy()?;

                // Exit extended mode
                self.set_mode_address_data_only();

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
            /// * `addr` - The address to read data from
            /// * `dest` - An array to store the result of the read into.
            ///
            /// # Panics
            ///
            /// Panics if the length of `data` is greater than the size of the XSPI
            /// hardware FIFO (32 bytes).
            pub fn read(
                &mut self,
                addr: u8,
                dest: &mut [u8],
            ) -> Result<(), XspiError> {
                assert!(
                    dest.len() <= 32,
                    "Transactions larger than the XSPI FIFO are currently unsupported"
                );

                // Begin the read operation
                self.begin_read(addr as u32, dest.len())?;

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
                while self.is_busy().is_err() {}

                Ok(())
            }

            /// Read data over the XSPI interface, using an extended transaction
            /// that may contain instruction, address, alternate-bytes and data
            /// phases with various sizes.
            ///
            /// # Args
            /// * `instruction` - The word to be used for the instruction phase.
            /// * `address` - The word to be used for the address phase.
            /// * `alternate_bytes` - The word to be used for the alternate-bytes phase.
            /// * `dummy_cycles` - 0 to 31 clock cycles between the alternate-bytes
            ///                    and the data phases.
            /// * `data` - An array of data to transfer over the XSPI interface
            ///
            /// # Panics
            ///
            /// Panics if the length of `dest` is greater than the size of the
            /// XSPI hardware FIFO (32 bytes). Panics if the length of `dest` is
            /// zero. Panics if the number of dummy cycles is not 0 - 31 inclusive.
            pub fn read_extended(&mut self,
                                 instruction: XspiWord,
                                 address: XspiWord,
                                 alternate_bytes: XspiWord,
                                 dummy_cycles: u8,
                                 dest: &mut [u8]) -> Result<(), XspiError> {
                assert!(
                    dest.len() <= 32,
                    "Transactions larger than the XSPI FIFO are currently unsupported"
                );
                assert!(
                    !dest.is_empty(),
                    "Must have a non-zero number of data cycles (otherwise use a write operation!)"
                );
                assert!(
                    dummy_cycles < 32,
                    "Hardware only supports 0-31 dummy cycles"
                );

                self.is_busy()?;

                // Clear the transfer complete flag.
                self.rb.fcr.write(|w| w.ctcf().set_bit());

                // Write the length that should be read.
                self.rb
                    .dlr
                    .write(|w| unsafe { w.dl().bits(dest.len() as u32 - 1) });

                // Setup extended mode. Read operations always have a data phase.
                // Transaction starts here
                self.setup_extended(instruction, address, alternate_bytes,
                                    dummy_cycles, true, true);

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
                while self.is_busy().is_err() {}

                Ok(())
            }
        }}}
    }

    #[cfg(any(feature = "rm0433", feature = "rm0399"))]
    xspi_impl! { stm32::QUADSPI, rec::Qspi, qspisel }

    #[cfg(any(feature = "rm0468"))] // TODO feature = "rm0455"
    xspi_impl! { stm32::OCTOSPI1, rec::Octospi1, octospisel }
    #[cfg(any(feature = "rm0455", feature = "rm0468"))]
    xspi_impl! { stm32::OCTOSPI2, rec::Octospi2, octospisel }
}
