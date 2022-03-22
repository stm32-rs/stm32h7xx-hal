//! Serial Peripheral Interface (SPI)
//!
//! This module implements the [embedded-hal](embedded-hal) traits for
//! master mode SPI.
//!
//! # Usage
//!
//! In the simplest case, SPI can be initialised from the device
//! peripheral and the GPIO pins.
//!
//! ```
//! use stm32h7xx_hal::spi;
//!
//! let dp = ...;                   // Device peripherals
//! let (sck, miso, mosi) = ...;    // GPIO pins
//!
//! let spi = dp.SPI1.spi((sck, miso, mosi), spi::MODE_0, 1.mhz(), ccdr.peripheral.SPI1, &ccdr.clocks);
//! ```
//!
//! The GPIO pins should be supplied as a
//! tuple in the following order:
//!
//! - Serial Clock (SCK)
//! - Master In Slave Out (MISO)
//! - Master Out Slave In (MOSI)
//!
//! If one of the pins is not required, explicitly pass one of the
//! filler types instead:
//!
//! ```
//! let spi = dp.SPI1.spi((sck, spi::NoMiso, mosi), spi::MODE_0, 1.mhz(), ccdr.peripheral.SPI1, &ccdr.clocks);
//! ```
//!
//! ## Word Sizes
//!
//! The word size used by the SPI controller must be indicated to the
//! compiler. This can be done either using an explicit type
//! annotation, or with a type hint. The possible word sizes are 8
//! bits (`u8`) or 16 bits (`u16`).
//!
//! For example, an explict type annotation:
//! ```
//! let _: spi:Spi<_, _, u8> = dp.SPI1.spi((sck, spi::NoMiso, mosi), spi::MODE_0, 1.mhz(), ccdr.peripheral.SPI1, &ccdr.clocks);
//! ```
//!
//! ## Clocks
//!
//! The bitrate calculation is based upon the clock currently assigned
//! in the RCC CCIP register. The default assignments are:
//!
//! - SPI1, SPI2, SPI3: __PLL1 Q CK__
//! - SPI4, SPI5: __APB__
//! - SPI6: __PCLK4__
//!
//! # Examples
//!
//! - [Blocking SPI](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/spi.rs)
//! - [SPI with DMA](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/spi-dma.rs).
//!
//! [embedded_hal]: https://docs.rs/embedded-hal/0.2.3/embedded_hal/spi/index.html

use core::convert::From;
use core::marker::PhantomData;
use core::ptr;

use crate::gpio::{self, Alternate};
use crate::hal;
use crate::hal::spi::FullDuplex;
pub use crate::hal::spi::{
    Mode, Phase, Polarity, MODE_0, MODE_1, MODE_2, MODE_3,
};
use crate::rcc::{rec, CoreClocks, ResetEnable};
use crate::stm32;
#[cfg(feature = "rm0455")]
use crate::stm32::rcc::{cdccip1r as ccip1r, srdccipr};
#[cfg(not(feature = "rm0455"))]
use crate::stm32::rcc::{d2ccip1r as ccip1r, d3ccipr as srdccipr};
use crate::stm32::spi1::{
    cfg1::MBR_A as MBR, cfg2::COMM_A as COMM, cfg2::SSIOP_A as SSIOP,
};
use crate::stm32::{SPI1, SPI2, SPI3, SPI4, SPI5, SPI6};
use crate::time::Hertz;

/// SPI error
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// Overrun occurred
    Overrun,
    /// Mode fault occurred
    ModeFault,
    /// CRC error
    Crc,
    /// Calling this method is not valid in this state
    InvalidCall,
    /// Can't start a transaction because one is already started
    TransactionAlreadyStarted,
    /// A buffer is too big to be processed
    BufferTooBig { max_size: usize },
    /// Duplex operation failed. This occours when a word was sent, but no
    /// corresponding word was received. May be caused by hardware issues where
    /// the SPI master fails to receive its own clock on the CLK pin
    DuplexFailed,
}

/// Enabled SPI peripheral (type state)
pub struct Enabled;

/// Disabled SPI peripheral (type state)
pub struct Disabled;

pub trait Pins<SPI> {
    /// States whether or not the Hardware Chip Select is present in this set of pins
    const HCS_PRESENT: bool;
}

pub trait PinSck<SPI> {}

pub trait PinMiso<SPI> {}

pub trait PinMosi<SPI> {}

pub trait PinHCS<SPI> {}

impl<SPI, SCK, MISO, MOSI> Pins<SPI> for (SCK, MISO, MOSI)
where
    SCK: PinSck<SPI>,
    MISO: PinMiso<SPI>,
    MOSI: PinMosi<SPI>,
{
    const HCS_PRESENT: bool = false;
}

impl<SPI, SCK, MISO, MOSI, HCS> Pins<SPI> for (SCK, MISO, MOSI, HCS)
where
    SCK: PinSck<SPI>,
    MISO: PinMiso<SPI>,
    MOSI: PinMosi<SPI>,
    HCS: PinHCS<SPI>,
{
    const HCS_PRESENT: bool = true;
}

/// Specifies the communication mode of the SPI interface.
#[derive(Copy, Clone)]
pub enum CommunicationMode {
    /// Both RX and TX are used.
    FullDuplex,

    /// Only the SPI TX functionality is used.
    Transmitter,

    /// Only the SPI RX functionality is used.
    Receiver,
}

/// A structure for specifying SPI configuration.
///
/// This structure uses builder semantics to generate the configuration.
///
/// `Example`
/// ```
/// use embedded_hal::spi::Mode;
///
/// let config = Config::new(Mode::MODE_0)
///     .manage_cs()
/// ```
#[derive(Copy, Clone)]
pub struct Config {
    mode: Mode,
    swap_miso_mosi: bool,
    hardware_cs: HardwareCS,
    inter_word_delay: f32,
    communication_mode: CommunicationMode,
}

impl Config {
    /// Create a default configuration for the SPI interface.
    ///
    /// Arguments:
    /// * `mode` - The SPI mode to configure.
    pub fn new(mode: Mode) -> Self {
        Config {
            mode,
            swap_miso_mosi: false,
            hardware_cs: HardwareCS {
                mode: HardwareCSMode::Disabled,
                assertion_delay: 0.0,
                polarity: Polarity::IdleHigh,
            },
            inter_word_delay: 0.0,
            communication_mode: CommunicationMode::FullDuplex,
        }
    }

    /// Specify that the SPI MISO/MOSI lines are swapped.
    ///
    /// Note:
    /// * This function updates the HAL peripheral to treat the pin provided in the MISO parameter
    /// as the MOSI pin and the pin provided in the MOSI parameter as the MISO pin.
    pub fn swap_mosi_miso(mut self) -> Self {
        self.swap_miso_mosi = true;
        self
    }

    /// Specify the behaviour of the hardware chip select.
    ///
    /// This also affects the way data is sent using [HardwareCSMode].
    /// By default the hardware cs is disabled.
    pub fn hardware_cs(mut self, hardware_cs: HardwareCS) -> Self {
        self.hardware_cs = hardware_cs;
        self
    }

    /// Specify the time in seconds that should be idled between every data word being sent.
    ///
    /// Note:
    /// * This value is converted to a number of spi peripheral clock ticks and at most 15 of those.
    pub fn inter_word_delay(mut self, inter_word_delay: f32) -> Self {
        self.inter_word_delay = inter_word_delay;
        self
    }

    /// Select the communication mode of the SPI bus.
    pub fn communication_mode(mut self, mode: CommunicationMode) -> Self {
        self.communication_mode = mode;
        self
    }
}

impl From<Mode> for Config {
    fn from(mode: Mode) -> Self {
        Self::new(mode)
    }
}

/// Object containing the settings for the hardware chip select pin
#[derive(Clone, Copy)]
pub struct HardwareCS {
    /// The value that determines the behaviour of the hardware chip select pin.
    pub mode: HardwareCSMode,
    /// The delay between CS assertion and the beginning of the SPI transaction in seconds.
    ///
    /// Note:
    /// * This value introduces a delay on SCK from the initiation of the transaction. The delay
    /// is specified as a number of SCK cycles, so the actual delay may vary.
    pub assertion_delay: f32,
    /// The polarity of the CS pin.
    pub polarity: Polarity,
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum HardwareCSMode {
    /// Handling the CS is left for the user to do in software
    Disabled,
    /// The CS will assert when the first data is sent and will not de-assert,
    /// unless manually done in software using [Spi::end_transaction].
    EndlessTransaction,
    /// The CS will assert and de-assert for each word being sent
    WordTransaction,
    /// The CS will assert and only de-assert after the whole frame is sent.
    ///
    /// When this mode is active, the blocking embedded hal interface automatically
    /// sets up the frames so it will be one frame per call.
    ///
    /// Note:
    /// * This mode does require some maintenance. Before sending, you must setup
    /// the frame with [Spi::setup_transaction]. After everything has been sent,
    /// you must also clean it up with [Spi::end_transaction].
    FrameTransaction,
}

impl HardwareCS {
    fn assertion_delay(&self) -> f32 {
        self.assertion_delay
    }

    fn polarity(&self) -> Polarity {
        self.polarity
    }

    fn enabled(&self) -> bool {
        !matches!(self.mode, HardwareCSMode::Disabled)
    }

    fn interleaved_cs(&self) -> bool {
        matches!(self.mode, HardwareCSMode::WordTransaction { .. })
    }
}

/// A filler type for when the SCK pin is unnecessary
pub struct NoSck;

/// A filler type for when the Miso pin is unnecessary
pub struct NoMiso;

/// A filler type for when the Mosi pin is unnecessary
pub struct NoMosi;

macro_rules! pins {
    ($($SPIX:ty:
       SCK: [$($( #[ $pmeta1:meta ] )* $SCK:ty),*]
       MISO: [$($( #[ $pmeta2:meta ] )* $MISO:ty),*]
       MOSI: [$($( #[ $pmeta3:meta ] )* $MOSI:ty),*]
       HCS: [$($( #[ $pmeta4:meta ] )* $HCS:ty),*]
    )+) => {
        $(
            $(
                $( #[ $pmeta1 ] )*
                impl PinSck<$SPIX> for $SCK {}
            )*
            $(
                $( #[ $pmeta2 ] )*
                impl PinMiso<$SPIX> for $MISO {}
            )*
            $(
                $( #[ $pmeta3 ] )*
                impl PinMosi<$SPIX> for $MOSI {}
            )*
            $(
                $( #[ $pmeta4 ] )*
                impl PinHCS<$SPIX> for $HCS {}
            )*
        )+
    }
}

pins! {
    SPI1:
        SCK: [
            NoSck,
            gpio::PA5<Alternate<5>>,
            gpio::PB3<Alternate<5>>,
            gpio::PG11<Alternate<5>>
        ]
        MISO: [
            NoMiso,
            gpio::PA6<Alternate<5>>,
            gpio::PB4<Alternate<5>>,
            gpio::PG9<Alternate<5>>
        ]
        MOSI: [
            NoMosi,
            gpio::PA7<Alternate<5>>,
            gpio::PB5<Alternate<5>>,
            gpio::PD7<Alternate<5>>
        ]
        HCS: [
            gpio::PA4<Alternate<5>>,
            gpio::PA15<Alternate<5>>,
            gpio::PG10<Alternate<5>>
        ]
    SPI2:
        SCK: [
            NoSck,
            gpio::PA9<Alternate<5>>,
            gpio::PA12<Alternate<5>>,
            gpio::PB10<Alternate<5>>,
            gpio::PB13<Alternate<5>>,
            gpio::PD3<Alternate<5>>,
            #[cfg(not(feature = "rm0468"))]
            gpio::PI1<Alternate<5>>
        ]
        MISO: [
            NoMiso,
            gpio::PB14<Alternate<5>>,
            gpio::PC2<Alternate<5>>,
            #[cfg(not(feature = "rm0468"))]
            gpio::PI2<Alternate<5>>
        ]
        MOSI: [
            NoMosi,
            gpio::PB15<Alternate<5>>,
            gpio::PC1<Alternate<5>>,
            gpio::PC3<Alternate<5>>,
            #[cfg(not(feature = "rm0468"))]
            gpio::PI3<Alternate<5>>
        ]
        HCS: [
            gpio::PA11<Alternate<5>>,
            gpio::PB4<Alternate<7>>,
            gpio::PB9<Alternate<5>>,
            gpio::PB12<Alternate<5>>,
            #[cfg(not(feature = "rm0468"))]
            gpio::PI0<Alternate<5>>
        ]
    SPI3:
        SCK: [
            NoSck,
            gpio::PB3<Alternate<6>>,
            gpio::PC10<Alternate<6>>
        ]
        MISO: [
            NoMiso,
            gpio::PB4<Alternate<6>>,
            gpio::PC11<Alternate<6>>
        ]
        MOSI: [
            NoMosi,
            gpio::PB2<Alternate<7>>,
            gpio::PB5<Alternate<7>>,
            gpio::PC12<Alternate<6>>,
            gpio::PD6<Alternate<5>>
        ]
        HCS: [
            gpio::PA4<Alternate<6>>,
            gpio::PA15<Alternate<6>>
        ]
    SPI4:
        SCK: [
            NoSck,
            gpio::PE2<Alternate<5>>,
            gpio::PE12<Alternate<5>>
        ]
        MISO: [
            NoMiso,
            gpio::PE5<Alternate<5>>,
            gpio::PE13<Alternate<5>>
        ]
        MOSI: [
            NoMosi,
            gpio::PE6<Alternate<5>>,
            gpio::PE14<Alternate<5>>
        ]
        HCS: [
            gpio::PE4<Alternate<5>>,
            gpio::PE11<Alternate<5>>
        ]
    SPI5:
        SCK: [
            NoSck,
            gpio::PF7<Alternate<5>>,
            gpio::PH6<Alternate<5>>,
            #[cfg(not(feature = "stm32h7b0"))]
            gpio::PK0<Alternate<5>>
        ]
        MISO: [
            NoMiso,
            gpio::PF8<Alternate<5>>,
            gpio::PH7<Alternate<5>>,
            #[cfg(not(feature = "stm32h7b0"))]
            gpio::PJ11<Alternate<5>>
        ]
        MOSI: [
            NoMosi,
            gpio::PF9<Alternate<5>>,
            gpio::PF11<Alternate<5>>,
            #[cfg(not(feature = "stm32h7b0"))]
            gpio::PJ10<Alternate<5>>
        ]
        HCS: [
            gpio::PF6<Alternate<5>>,
            gpio::PH5<Alternate<5>>,
            #[cfg(not(feature = "stm32h7b0"))]
            gpio::PK1<Alternate<5>>
        ]
    SPI6:
        SCK: [
            NoSck,
            gpio::PA5<Alternate<8>>,
            gpio::PB3<Alternate<8>>,
            #[cfg(feature = "rm0455")]
            gpio::PC12<Alternate<5>>,
            gpio::PG13<Alternate<5>>
        ]
        MISO: [
            NoMiso,
            gpio::PA6<Alternate<8>>,
            gpio::PB4<Alternate<8>>,
            gpio::PG12<Alternate<5>>
        ]
        MOSI: [
            NoMosi,
            gpio::PA7<Alternate<8>>,
            gpio::PB5<Alternate<8>>,
            gpio::PG14<Alternate<5>>
        ]
        HCS: [
            gpio::PA4<Alternate<8>>,
            gpio::PA15<Alternate<7>>,
            gpio::PG8<Alternate<5>>
        ]
}

macro_rules! check_status_error {
    ($spi:expr; $(  {$flag:ident, $variant:ident, $blk:block}  ),*) => {{
        let sr = $spi.sr.read();

        return Err(if sr.ovr().is_overrun() {
            nb::Error::Other(Error::Overrun)
        } else if sr.modf().is_fault() {
            nb::Error::Other(Error::ModeFault)
        } else if sr.crce().is_error() {
            nb::Error::Other(Error::Crc)
        }
            $(
                else if sr.$flag().$variant() { $blk }
            )*
        else {
            nb::Error::WouldBlock
        })
    }}
}

/// Interrupt events
#[derive(Copy, Clone, PartialEq)]
pub enum Event {
    /// New data has been received
    Rxp,
    /// Data can be sent
    Txp,
    /// An error occurred
    Error,
}

#[derive(Debug)]
pub struct Spi<SPI, ED, WORD = u8> {
    spi: SPI,
    hardware_cs_mode: HardwareCSMode,
    _word: PhantomData<WORD>,
    _ed: PhantomData<ED>,
}

pub trait SpiExt<SPI, WORD>: Sized {
    type Rec: ResetEnable;

    fn spi<PINS, T, CONFIG>(
        self,
        _pins: PINS,
        config: CONFIG,
        freq: T,
        prec: Self::Rec,
        clocks: &CoreClocks,
    ) -> Spi<SPI, Enabled, WORD>
    where
        PINS: Pins<SPI>,
        T: Into<Hertz>,
        CONFIG: Into<Config>;

    fn spi_unchecked<T, CONFIG>(
        self,
        config: CONFIG,
        freq: T,
        prec: Self::Rec,
        clocks: &CoreClocks,
    ) -> Spi<SPI, Enabled, WORD>
    where
        T: Into<Hertz>,
        CONFIG: Into<Config>;
}

pub trait HalEnabledSpi:
    HalSpi + FullDuplex<Self::Word, Error = Error>
{
    type Disabled: HalDisabledSpi<
        Spi = Self::Spi,
        Word = Self::Word,
        Enabled = Self,
    >;

    /// Disables the SPI peripheral. Any SPI operation is
    /// stopped and disabled, the internal state machine is
    /// reset, all the FIFOs content is flushed, the MODF
    /// flag is cleared, the SSI flag is cleared, and the
    /// CRC calculation is re-initialized. Clocks are not
    /// disabled.
    fn disable(self) -> Self::Disabled;

    /// Resets the SPI peripheral.
    fn reset(&mut self);

    /// Sets up a frame transaction with the given amount of data words.
    ///
    /// If this is called when the hardware CS mode is not [HardwareCSMode::FrameTransaction],
    /// then an error is returned with [Error::InvalidCall].
    ///
    /// If this is called when a transaction has already started,
    /// then an error is returned with [Error::TransactionAlreadyStarted].
    fn setup_transaction(
        &mut self,
        words: core::num::NonZeroU16,
    ) -> Result<(), Error>;

    /// Ends the current transaction, both for endless and frame transactions.
    ///
    /// This method must always be called for frame transaction,
    /// even if the full size has been sent. If this is not done,
    /// no new data can be sent even when it looks like it should.
    ///
    /// If it's not either a frame or endless transaction,
    /// an error is returned with [Error::InvalidCall].
    fn end_transaction(&mut self) -> Result<(), Error>;
}

pub trait HalDisabledSpi: HalSpi {
    type Rec;
    type Enabled: HalEnabledSpi<Spi = Self::Spi, Word = Self::Word>;

    /// Enables the SPI peripheral.
    /// Clears the MODF flag, the SSI flag, and sets the SPE bit.
    fn enable(self) -> Self::Enabled;

    /// Enables the Rx DMA stream. If the DMA Rx is used, the
    /// reference manual recommends that this is enabled before
    /// enabling the DMA
    fn enable_dma_rx(&mut self);

    fn disable_dma_rx(&mut self);

    /// Enables the Tx DMA stream. If the DMA Tx is used, the
    /// reference manual recommends that this is enabled after
    /// enabling the DMA
    fn enable_dma_tx(&mut self);

    fn disable_dma_tx(&mut self);

    /// Deconstructs the SPI peripheral and returns the component parts.
    fn free(self) -> (Self::Spi, Self::Rec);
}

pub trait HalSpi: Sized {
    type Spi;
    type Word;

    /// Returns a mutable reference to the inner peripheral
    fn inner(&self) -> &Self::Spi;

    /// Returns a mutable reference to the inner peripheral
    fn inner_mut(&mut self) -> &mut Self::Spi;

    /// Enable interrupts for the given `event`:
    ///  - Received data ready to be read (RXP)
    ///  - Transmit data register empty (TXP)
    ///  - Error
    fn listen(&mut self, event: Event);

    /// Disable interrupts for the given `event`:
    ///  - Received data ready to be read (RXP)
    ///  - Transmit data register empty (TXP)
    ///  - Error
    fn unlisten(&mut self, event: Event);

    /// Return `true` if the TXP flag is set, i.e. new
    /// data to transmit can be written to the SPI.
    fn is_txp(&self) -> bool;

    /// Return `true` if the RXP flag is set, i.e. new
    /// data has been received and can be read from the
    /// SPI.
    fn is_rxp(&self) -> bool;

    /// Return `true` if the MODF flag is set, i.e. the
    /// SPI has experienced a mode fault
    fn is_modf(&self) -> bool;

    /// Return `true` if the OVR flag is set, i.e. new
    /// data has been received while the receive data
    /// register was already filled.
    fn is_ovr(&self) -> bool;

    /// Clears the MODF flag, which indicates that a
    /// mode fault has occurred.
    fn clear_modf(&mut self);
}

macro_rules! spi {
    (DSIZE, $spi:ident,  u8) => {
        $spi.cfg1.modify(|_, w| {
            w.dsize()
                .bits(8 - 1) // 8 bit words
        });
    };
    (DSIZE, $spi:ident, u16) => {
        $spi.cfg1.modify(|_, w| {
            w.dsize()
                .bits(16 - 1) // 16 bit words
        });
    };
    (DSIZE, $spi:ident, u32) => {
        $spi.cfg1.modify(|_, w| {
            w.dsize()
                .bits(32 - 1) // 32 bit words
        });
    };
	($($SPIX:ident: ($spiX:ident, $Rec:ident, $pclkX:ident)
       => ($($TY:ident),+),)+) => {
	    $(
            // For each $TY
            $(

                impl Spi<$SPIX, Enabled, $TY> {
                    fn $spiX<T, CONFIG>(
                        spi: $SPIX,
                        config: CONFIG,
                        freq: T,
                        prec: rec::$Rec,
                        clocks: &CoreClocks,
                    ) -> Self
                    where
                        T: Into<Hertz>,
                        CONFIG: Into<Config>,
                    {
                        // Enable clock for SPI
                        prec.enable();

                        // Disable SS output
                        spi.cfg2.write(|w| w.ssoe().disabled());

                        let config: Config = config.into();

                        let spi_freq = freq.into().0;
	                    let spi_ker_ck = Self::kernel_clk_unwrap(clocks).0;
                        let mbr = match (spi_ker_ck + spi_freq - 1) / spi_freq {
                            1..=2 => MBR::DIV2,
                            3..=4 => MBR::DIV4,
                            5..=8 => MBR::DIV8,
                            9..=16 => MBR::DIV16,
                            17..=32 => MBR::DIV32,
                            33..=64 => MBR::DIV64,
                            65..=128 => MBR::DIV128,
                            _ => MBR::DIV256,
                        };
                        spi.cfg1.modify(|_, w| {
                            w.mbr()
                                .variant(mbr) // master baud rate
                        });
                        spi!(DSIZE, spi, $TY); // modify CFG1 for DSIZE

                        // ssi: select slave = master mode
                        spi.cr1.write(|w| w.ssi().slave_not_selected());

                        // Calculate the CS->transaction cycle delay bits.
                        let (assertion_delay, inter_word_delay) = {
                            let mut assertion_delay: u32 = (config.hardware_cs.assertion_delay() * spi_freq as f32) as u32;
                            let mut inter_word_delay: u32 = (config.inter_word_delay * spi_freq as f32) as u32;

                            // If a delay is specified as non-zero, add 1 to the delay cycles
                            // before truncation to an integer to ensure that we have at least as
                            // many cycles as required.
                            if config.hardware_cs.assertion_delay() > 0.0_f32 {
                                assertion_delay += 1;
                            }
                            if config.inter_word_delay > 0.0_f32 {
                                inter_word_delay += 1;
                            }

                            // If CS suspends while data is inactive, we also require an
                            // "inter-data" delay.
                            if matches!(config.hardware_cs.mode, HardwareCSMode::WordTransaction) {
                                inter_word_delay = inter_word_delay.max(1);
                            }

                            (assertion_delay.min(0xF) as u8, inter_word_delay.min(0xF) as u8)
                        };

                        // The calculated cycle delay may not be more than 4 bits wide for the
                        // configuration register.
                        let communication_mode = match config.communication_mode {
                            CommunicationMode::Transmitter => COMM::TRANSMITTER,
                            CommunicationMode::Receiver => COMM::RECEIVER,
                            CommunicationMode::FullDuplex => COMM::FULLDUPLEX,
                        };

                        let cs_polarity = match config.hardware_cs.polarity() {
                            Polarity::IdleHigh => SSIOP::ACTIVELOW,
                            Polarity::IdleLow => SSIOP::ACTIVEHIGH,
                        };

                        // mstr: master configuration
                        // lsbfrst: MSB first
                        // comm: full-duplex
                        spi.cfg2.write(|w| {
                            w.cpha()
                                .bit(config.mode.phase ==
                                     Phase::CaptureOnSecondTransition)
                                .cpol()
                                .bit(config.mode.polarity == Polarity::IdleHigh)
                                .master()
                                .master()
                                .lsbfrst()
                                .msbfirst()
                                .ssom()
                                .bit(config.hardware_cs.interleaved_cs())
                                .ssm()
                                .bit(config.hardware_cs.enabled() == false)
                                .ssoe()
                                .bit(config.hardware_cs.enabled() == true)
                                .mssi()
                                .bits(assertion_delay)
                                .midi()
                                .bits(inter_word_delay)
                                .ioswp()
                                .bit(config.swap_miso_mosi == true)
                                .comm()
                                .variant(communication_mode)
                                .ssiop()
                                .variant(cs_polarity)
                        });

                        // Reset to default (might have been set if previously used by a frame transaction)
                        // So that is 1 when it's a frame transaction and 0 when in another mode
                        spi.cr2.write(|w| w.tsize().bits(matches!(config.hardware_cs.mode, HardwareCSMode::FrameTransaction) as u16));

                        // spe: enable the SPI bus
                        spi.cr1.write(|w| w.ssi().slave_not_selected().spe().enabled());

                        Spi { spi, hardware_cs_mode: config.hardware_cs.mode, _word: PhantomData, _ed: PhantomData }
                    }
                }

                impl <Ed> Spi<$SPIX, Ed, $TY> {
                    /// internally disable the SPI without changing its type-state
                    fn internal_disable(&mut self) {
                        self.spi.cr1.modify(|_, w| w.csusp().requested());
                        while self.spi.sr.read().eot().is_completed() {}
                        self.spi.cr1.write(|w| w.ssi().slave_not_selected().spe().disabled());
                    }

                    /// internally enable the SPI without changing its type-state
                    fn internal_enable(&mut self) {
                        self.clear_modf(); // SPE cannot be set when MODF is set
                        self.spi.cr1.write(|w| w.ssi().slave_not_selected().spe().enabled());
                    }
                }

                impl HalEnabledSpi for Spi<$SPIX, Enabled, $TY> {
                    type Disabled = Spi<Self::Spi, Disabled, Self::Word>;

                    fn reset(&mut self) {
                        self.internal_disable();
                        self.internal_enable();
                    }

                    fn disable(mut self) -> Spi<$SPIX, Disabled, $TY> {
                        // Master communication must be suspended before the peripheral is disabled
                        self.internal_disable();

                        Spi {
                            spi: self.spi,
                            hardware_cs_mode: self.hardware_cs_mode,
                            _word: PhantomData,
                            _ed: PhantomData,
                        }
                    }

                    fn setup_transaction(&mut self, words: core::num::NonZeroU16) -> Result<(), Error> {
                        if !matches!(self.hardware_cs_mode, HardwareCSMode::FrameTransaction) {
                            return Err(Error::InvalidCall);
                        }

                        if self.spi.cr1.read().cstart().is_started() {
                            return Err(Error::TransactionAlreadyStarted);
                        }

                        // We can only set tsize when spi is disabled
                        self.spi.cr1.modify(|_, w| w.csusp().requested());
                        while self.spi.sr.read().eot().is_completed() {}
                        self.spi.cr1.write(|w| w.ssi().slave_not_selected().spe().disabled());

                        // Set the frame size
                        self.spi.cr2.write(|w| w.tsize().bits(words.get()));

                        // Re-enable
                        self.clear_modf(); // SPE cannot be set when MODF is set
                        self.spi.cr1.write(|w| w.ssi().slave_not_selected().spe().enabled());

                        Ok(())
                    }

                    fn end_transaction(&mut self) -> Result<(), Error> {
                        if !matches!(self.hardware_cs_mode, HardwareCSMode::FrameTransaction | HardwareCSMode::EndlessTransaction) {
                            return Err(Error::InvalidCall);
                        }

                        self.spi.cr1.modify(|_, w| w.csusp().requested());
                        while(self.spi.cr1.read().cstart().is_started()) {}

                        self.spi.ifcr.write(|w| w.txtfc().clear().eotc().clear());

                        Ok(())
                    }
                }

                impl HalDisabledSpi for Spi<$SPIX, Disabled, $TY> {
                    type Rec = rec::$Rec;
                    type Enabled = Spi<Self::Spi, Enabled, Self::Word>;

                    fn enable(mut self) -> Self::Enabled {
                        self.internal_enable();
                        Spi {
                            spi: self.spi,
                            hardware_cs_mode: self.hardware_cs_mode,
                            _word: PhantomData,
                            _ed: PhantomData,
                        }
                    }

                    fn enable_dma_rx(&mut self) {
                        self.spi.cfg1.modify(|_,w| w.rxdmaen().enabled());
                    }

                    fn disable_dma_rx(&mut self) {
                        self.spi.cfg1.modify(|_,w| w.rxdmaen().disabled());
                    }

                    fn enable_dma_tx(&mut self) {
                        self.spi.cfg1.modify(|_,w| w.txdmaen().enabled());
                    }

                    fn disable_dma_tx(&mut self) {
                        self.spi.cfg1.modify(|_,w| w.txdmaen().disabled());
                    }

                    fn free(self) -> ($SPIX, rec::$Rec) {
                        (self.spi, rec::$Rec { _marker: PhantomData })
                    }
                }

                impl<EN> HalSpi for Spi<$SPIX, EN, $TY>
                {
                    type Word = $TY;
                    type Spi = $SPIX;

                    /// Returns a mutable reference to the inner peripheral
                    fn inner(&self) -> &Self::Spi {
                        &self.spi
                    }

                    /// Returns a mutable reference to the inner peripheral
                    fn inner_mut(&mut self) -> &mut Self::Spi {
                        &mut self.spi
                    }

                    /// Enable interrupts for the given `event`:
                    ///  - Received data ready to be read (RXP)
                    ///  - Transmit data register empty (TXP)
                    ///  - Error
                    fn listen(&mut self, event: Event) {
                        match event {
                            Event::Rxp => self.spi.ier.modify(|_, w|
                                                              w.rxpie().not_masked()),
                            Event::Txp => self.spi.ier.modify(|_, w|
                                                              w.txpie().not_masked()),
                            Event::Error => self.spi.ier.modify(|_, w| {
                                w.udrie() // Underrun
                                    .not_masked()
                                    .ovrie() // Overrun
                                    .not_masked()
                                    .crceie() // CRC error
                                    .not_masked()
                                    .modfie() // Mode fault
                                    .not_masked()
                            }),
                        }
                    }

                    /// Disable interrupts for the given `event`:
                    ///  - Received data ready to be read (RXP)
                    ///  - Transmit data register empty (TXP)
                    ///  - Error
                    fn unlisten(&mut self, event: Event) {
                        match event {
                            Event::Rxp => {
                                self.spi.ier.modify(|_, w| w.rxpie().masked());
                            }
                            Event::Txp => {
                                self.spi.ier.modify(|_, w| w.txpie().masked());
                            }
                            Event::Error => {
                                self.spi.ier.modify(|_, w| {
                                    w.udrie() // Underrun
                                        .masked()
                                        .ovrie() // Overrun
                                        .masked()
                                        .crceie() // CRC error
                                        .masked()
                                        .modfie() // Mode fault
                                        .masked()
                                })
                            }
                        }
                        let _ = self.spi.ier.read();
                        let _ = self.spi.ier.read(); // Delay 2 peripheral clocks
                    }

                    /// Return `true` if the TXP flag is set, i.e. new
                    /// data to transmit can be written to the SPI.
                    fn is_txp(&self) -> bool {
                        self.spi.sr.read().txp().is_not_full()
                    }

                    /// Return `true` if the RXP flag is set, i.e. new
                    /// data has been received and can be read from the
                    /// SPI.
                    fn is_rxp(&self) -> bool {
                        self.spi.sr.read().rxp().is_not_empty()
                    }

                    /// Return `true` if the MODF flag is set, i.e. the
                    /// SPI has experienced a mode fault
                    fn is_modf(&self) -> bool {
                        self.spi.sr.read().modf().is_fault()
                    }

                    /// Return `true` if the OVR flag is set, i.e. new
                    /// data has been received while the receive data
                    /// register was already filled.
                    fn is_ovr(&self) -> bool {
                        self.spi.sr.read().ovr().is_overrun()
                    }

                    /// Clears the MODF flag, which indicates that a
                    /// mode fault has occurred.
                    fn clear_modf(&mut self) {
                        self.spi.ifcr.write(|w| w.modfc().clear());
                        let _ = self.spi.sr.read();
                        let _ = self.spi.sr.read(); // Delay 2 peripheral clocks
                    }
                }

                impl SpiExt<$SPIX, $TY> for $SPIX {
                    type Rec = rec::$Rec;

	                fn spi<PINS, T, CONFIG>(self,
                                    _pins: PINS,
                                    config: CONFIG,
                                    freq: T,
                                    prec: rec::$Rec,
                                    clocks: &CoreClocks) -> Spi<$SPIX, Enabled, $TY>
	                where
	                    PINS: Pins<$SPIX>,
	                    T: Into<Hertz>,
                        CONFIG: Into<Config>,
	                {
                        let config = config.into();
                        assert_eq!(
                            config.hardware_cs.enabled(),
                            PINS::HCS_PRESENT,
                            "If the hardware cs is enabled in the config, an HCS pin must be present in the given pins"
                        );
	                    Spi::<$SPIX, Enabled, $TY>::$spiX(self, config, freq, prec, clocks)
	                }

	                fn spi_unchecked<T, CONFIG>(self,
                                        config: CONFIG,
                                        freq: T,
                                        prec: rec::$Rec,
                                        clocks: &CoreClocks) -> Spi<$SPIX, Enabled, $TY>
	                where
	                    T: Into<Hertz>,
                        CONFIG: Into<Config>,
	                {
	                    Spi::<$SPIX, Enabled, $TY>::$spiX(self, config, freq, prec, clocks)
	                }
	            }

                impl hal::spi::FullDuplex<$TY> for Spi<$SPIX, Enabled, $TY> {
                    type Error = Error;

                    fn read(&mut self) -> nb::Result<$TY, Error> {
                        check_status_error!(self.spi;
                        {    // } else if sr.rxp().is_not_empty() {
                            rxp, is_not_empty,
                            {
                                // NOTE(read_volatile) read only 1 word
                                return Ok(unsafe {
                                    ptr::read_volatile(
                                        &self.spi.rxdr as *const _ as *const $TY,
                                    )
                                });
                            }
                        })
                    }

                    fn send(&mut self, word: $TY) -> nb::Result<(), Error> {
                        check_status_error!(self.spi;
                        {    // } else if sr.txp().is_not_full() {
                            txp, is_not_full,
                            {
                                // NOTE(write_volatile) see note above
                                unsafe {
                                    ptr::write_volatile(
                                        &self.spi.txdr as *const _ as *mut $TY,
                                        word,
                                    )
                                }
                                // write CSTART to start a transaction in
                                // master mode
                                self.spi.cr1.modify(|_, w| w.cstart().started());

                                return Ok(());
                            }
                        })
                    }
                }

                impl Spi<$SPIX, Enabled, $TY>
                {
                    /// Internal implementation for exchanging a word
                    ///
                    /// * Assumes the transaction has started (CSTART handled externally)
                    /// * Assumes at least one word has already been written to the Tx FIFO
                    #[inline(always)]
                    fn exchange_duplex_internal(&mut self, word: $TY) -> nb::Result<$TY, Error> {
                        check_status_error!(self.spi;
                        {    // else if sr.dxp().is_available() {
                            dxp, is_available,
                            {
                                // NOTE(write_volatile/read_volatile) write/read only 1 word
                                unsafe {
                                    ptr::write_volatile(
                                        &self.spi.txdr as *const _ as *mut $TY,
                                        word,
                                    );
                                    return Ok(ptr::read_volatile(
                                        &self.spi.rxdr as *const _ as *const $TY,
                                    ));
                                }
                            }
                        }, { // else if sr.txc().is_completed() {
                            txc, is_completed,
                            {
                                let sr = self.spi.sr.read(); // Read SR again on a subsequent PCLK cycle

                                if sr.txc().is_completed() && !sr.rxp().is_not_empty() {
                                    // The Tx FIFO completed, but no words were
                                    // available in the Rx FIFO. This is a duplex failure
                                    nb::Error::Other(Error::DuplexFailed)
                                } else {
                                    nb::Error::WouldBlock
                                }
                            }
                        })
                    }
                    /// Internal implementation for reading a word
                    ///
                    /// * Assumes the transaction has started (CSTART handled externally)
                    /// * Assumes at least one word has already been written to the Tx FIFO
                    #[inline(always)]
                    fn read_duplex_internal(&mut self) -> nb::Result<$TY, Error> {
                        check_status_error!(self.spi;
                        {    // else if sr.rxp().is_not_empty()
                            rxp, is_not_empty,
                            {
                                // NOTE(read_volatile) read only 1 word
                                return Ok(unsafe {
                                    ptr::read_volatile(
                                        &self.spi.rxdr as *const _ as *const $TY,
                                    )
                                });
                            }
                        }, { // else if sr.txc().is_completed()
                            txc, is_completed,
                            {
                                let sr = self.spi.sr.read(); // Read SR again on a subsequent PCLK cycle

                                if sr.txc().is_completed() && !sr.rxp().is_not_empty() {
                                    // The Tx FIFO completed, but no words were
                                    // available in the Rx FIFO. This is a duplex failure
                                    nb::Error::Other(Error::DuplexFailed)
                                } else {
                                    nb::Error::WouldBlock
                                }
                            }
                        })
                    }

                    /// Internal implementation for blocking::spi::Transfer and
                    /// blocking::spi::Write
                    fn transfer_internal<'w>(&mut self,
                                             write_words: &'w [$TY],
                                             read_words: Option<&'w mut [$TY]>
                    ) -> Result<(), Error> {
                        use hal::spi::FullDuplex;

                        // both buffers are the same length
                        if let Some(ref read) = read_words {
                            debug_assert!(write_words.len() == read.len());
                        }
                        if write_words.is_empty() {
                            return Ok(());
                        }

                        // Are we in frame mode?
                        if matches!(self.hardware_cs_mode, HardwareCSMode::FrameTransaction) {
                            const MAX_WORDS: usize = 0xFFFF;

                            // Can we send
                            if write_words.len() > MAX_WORDS {
                                return Err(Error::BufferTooBig { max_size: MAX_WORDS });
                            }

                            // Setup that we're going to send this amount of bits
                            // SAFETY: We already checked that `write_words` is not empty
                            self.setup_transaction(unsafe {
                                core::num::NonZeroU16::new_unchecked(write_words.len() as u16)
                            })?;
                        }

                        // Depth of FIFO to use. All current SPI implementations
                        // have a FIFO depth of at least 8 (see RM0433 Rev 7
                        // Tabel 409.) but pick 4 as a conservative value.
                        const FIFO_WORDS: usize = 4;

                        // Fill the first half of the write FIFO
                        let len = write_words.len();
                        let mut write = write_words.iter();
                        for _ in 0..core::cmp::min(FIFO_WORDS, len) {
                            nb::block!(self.send(*write.next().unwrap()))?;
                        }

                        if let Some(read) = read_words {
                            let mut read = read.iter_mut();

                            // Continue filling write FIFO and emptying read FIFO
                            for word in write {
                                *read.next().unwrap() = nb::block!(
                                    self.exchange_duplex_internal(*word)
                                )?;
                            }

                            // Finish emptying the read FIFO
                            for word in read {
                                *word = nb::block!(self.read_duplex_internal())?;
                            }
                        } else {
                            // Continue filling write FIFO and emptying read FIFO
                            for word in write {
                                let _ = nb::block!(
                                    self.exchange_duplex_internal(*word)
                                )?;
                            }

                            // Dummy read from the read FIFO
                            for _ in 0..core::cmp::min(FIFO_WORDS, len) {
                                let _ = nb::block!(self.read_duplex_internal())?;
                            }
                        }

                        // Are we in frame mode?
                        if matches!(self.hardware_cs_mode, HardwareCSMode::FrameTransaction) {
                            // Clean up
                            self.end_transaction()?;
                        }

                        Ok(())
                    }
                }
                impl hal::blocking::spi::Transfer<$TY> for Spi<$SPIX, Enabled, $TY> {
                    type Error = Error;

                    fn transfer<'w>(&mut self, words: &'w mut [$TY]) -> Result<&'w [$TY], Self::Error> {
                        // SAFETY: transfer_internal always writes out words
                        // before modifying them
                        let write = unsafe {
                            core::slice::from_raw_parts(words.as_ptr(), words.len())
                        };
                        self.transfer_internal(write, Some(words))?;

                        Ok(words)
                    }
                }
                impl hal::blocking::spi::Write<$TY> for Spi<$SPIX, Enabled, $TY> {
                    type Error = Error;

                    fn write(&mut self, words: &[$TY]) -> Result<(), Self::Error> {
                        self.transfer_internal(words, None)
                    }
                }
            )+
        )+
	}
}

macro_rules! spi123sel {
	($($SPIX:ident,)+) => {
	    $(
            impl<WORD> Spi<$SPIX, Enabled, WORD> {
                /// Returns the frequency of the current kernel clock
                /// for SPI1, SPI2, SPI3
                pub fn kernel_clk(clocks: &CoreClocks) -> Option<Hertz> {
                    #[cfg(not(feature = "rm0455"))]
                    let ccip1r = unsafe { (*stm32::RCC::ptr()).d2ccip1r.read() };
                    #[cfg(feature = "rm0455")]
                    let ccip1r = unsafe { (*stm32::RCC::ptr()).cdccip1r.read() };

                    match ccip1r.spi123sel().variant() {
                        Some(ccip1r::SPI123SEL_A::PLL1_Q) => clocks.pll1_q_ck(),
                        Some(ccip1r::SPI123SEL_A::PLL2_P) => clocks.pll2_p_ck(),
                        Some(ccip1r::SPI123SEL_A::PLL3_P) => clocks.pll3_p_ck(),
                        // Need a method of specifying pin clock
                        Some(ccip1r::SPI123SEL_A::I2S_CKIN) => unimplemented!(),
                        Some(ccip1r::SPI123SEL_A::PER) => clocks.per_ck(),
                        _ => unreachable!(),
                    }
                }
                /// Returns the frequency of the current kernel clock
                /// for SPI1, SPI2, SPI3
                ///
                /// # Panics
                ///
                /// Panics if the kernel clock is not running
                pub fn kernel_clk_unwrap(clocks: &CoreClocks) -> Hertz {
                    #[cfg(not(feature = "rm0455"))]
                    let ccip1r = unsafe { (*stm32::RCC::ptr()).d2ccip1r.read() };
                    #[cfg(feature = "rm0455")]
                    let ccip1r = unsafe { (*stm32::RCC::ptr()).cdccip1r.read() };

                    match ccip1r.spi123sel().variant() {
                        Some(ccip1r::SPI123SEL_A::PLL1_Q) => {
                            clocks.pll1_q_ck().expect("SPI123: PLL1_Q must be enabled")
                        }
                        Some(ccip1r::SPI123SEL_A::PLL2_P) => {
                            clocks.pll2_p_ck().expect("SPI123: PLL2_P must be enabled")
                        }
                        Some(ccip1r::SPI123SEL_A::PLL3_P) => {
                            clocks.pll3_p_ck().expect("SPI123: PLL3_P must be enabled")
                        }
                        // Need a method of specifying pin clock
                        Some(ccip1r::SPI123SEL_A::I2S_CKIN) => unimplemented!(),
                        Some(ccip1r::SPI123SEL_A::PER) => {
                            clocks.per_ck().expect("SPI123: PER clock must be enabled")
                        }
                        _ => unreachable!(),
                    }
                }
            }
        )+
    }
}
macro_rules! spi45sel {
	($($SPIX:ident,)+) => {
	    $(
            impl<WORD> Spi<$SPIX, Enabled, WORD> {
                /// Returns the frequency of the current kernel clock
                /// for SPI4, SPI5
                pub fn kernel_clk(clocks: &CoreClocks) -> Option<Hertz> {
                    #[cfg(not(feature = "rm0455"))]
                    let ccip1r = unsafe { (*stm32::RCC::ptr()).d2ccip1r.read() };
                    #[cfg(feature = "rm0455")]
                    let ccip1r = unsafe { (*stm32::RCC::ptr()).cdccip1r.read() };

                    match ccip1r.spi45sel().variant() {
                        Some(ccip1r::SPI45SEL_A::APB) => Some(clocks.pclk2()),
                        Some(ccip1r::SPI45SEL_A::PLL2_Q) => clocks.pll2_q_ck(),
                        Some(ccip1r::SPI45SEL_A::PLL3_Q) => clocks.pll3_q_ck(),
                        Some(ccip1r::SPI45SEL_A::HSI_KER) => clocks.hsi_ck(),
                        Some(ccip1r::SPI45SEL_A::CSI_KER) => clocks.csi_ck(),
                        Some(ccip1r::SPI45SEL_A::HSE) => clocks.hse_ck(),
                        _ => unreachable!(),
                    }
                }
                /// Returns the frequency of the current kernel clock
                /// for SPI4, SPI5
                ///
                /// # Panics
                ///
                /// Panics if the kernel clock is not running
                pub fn kernel_clk_unwrap(clocks: &CoreClocks) -> Hertz {
                    #[cfg(not(feature = "rm0455"))]
                    let ccip1r = unsafe { (*stm32::RCC::ptr()).d2ccip1r.read() };
                    #[cfg(feature = "rm0455")]
                    let ccip1r = unsafe { (*stm32::RCC::ptr()).cdccip1r.read() };

                    match ccip1r.spi45sel().variant() {
                        Some(ccip1r::SPI45SEL_A::APB) => clocks.pclk2(),
                        Some(ccip1r::SPI45SEL_A::PLL2_Q) => {
                            clocks.pll2_q_ck().expect("SPI45: PLL2_Q must be enabled")
                        }
                        Some(ccip1r::SPI45SEL_A::PLL3_Q) => {
                            clocks.pll3_q_ck().expect("SPI45: PLL3_Q must be enabled")
                        }
                        Some(ccip1r::SPI45SEL_A::HSI_KER) => {
                            clocks.hsi_ck().expect("SPI45: HSI clock must be enabled")
                        }
                        Some(ccip1r::SPI45SEL_A::CSI_KER) => {
                            clocks.csi_ck().expect("SPI45: CSI clock must be enabled")
                        }
                        Some(ccip1r::SPI45SEL_A::HSE) => {
                            clocks.hse_ck().expect("SPI45: HSE clock must be enabled")
                        }
                        _ => unreachable!(),
                    }
                }
            }
        )+
    }
}
macro_rules! spi6sel {
	($($SPIX:ident,)+) => {
	    $(
            impl<WORD> Spi<$SPIX, Enabled, WORD> {
                /// Returns the frequency of the current kernel clock
                /// for SPI6
                pub fn kernel_clk(clocks: &CoreClocks) -> Option<Hertz> {
                    #[cfg(not(feature = "rm0455"))]
                    let srdccipr = unsafe { (*stm32::RCC::ptr()).d3ccipr.read() };
                    #[cfg(feature = "rm0455")]
                    let srdccipr = unsafe { (*stm32::RCC::ptr()).srdccipr.read() };

                    match srdccipr.spi6sel().variant() {
                        Some(srdccipr::SPI6SEL_A::RCC_PCLK4) => Some(clocks.pclk4()),
                        Some(srdccipr::SPI6SEL_A::PLL2_Q) => clocks.pll2_q_ck(),
                        Some(srdccipr::SPI6SEL_A::PLL3_Q) => clocks.pll3_q_ck(),
                        Some(srdccipr::SPI6SEL_A::HSI_KER) => clocks.hsi_ck(),
                        Some(srdccipr::SPI6SEL_A::CSI_KER) => clocks.csi_ck(),
                        Some(srdccipr::SPI6SEL_A::HSE) => clocks.hse_ck(),
                        _ => unreachable!(),
                    }
                }
                /// Returns the frequency of the current kernel clock
                /// for SPI6
                pub fn kernel_clk_unwrap(clocks: &CoreClocks) -> Hertz {
                    #[cfg(not(feature = "rm0455"))]
                    let srdccipr = unsafe { (*stm32::RCC::ptr()).d3ccipr.read() };
                    #[cfg(feature = "rm0455")]
                    let srdccipr = unsafe { (*stm32::RCC::ptr()).srdccipr.read() };

                    match srdccipr.spi6sel().variant() {
                        Some(srdccipr::SPI6SEL_A::RCC_PCLK4) => clocks.pclk4(),
                        Some(srdccipr::SPI6SEL_A::PLL2_Q) => {
                            clocks.pll2_q_ck().expect("SPI6: PLL2_Q must be enabled")
                        }
                        Some(srdccipr::SPI6SEL_A::PLL3_Q) => {
                            clocks.pll3_q_ck().expect("SPI6: PLL3_Q must be enabled")
                        }
                        Some(srdccipr::SPI6SEL_A::HSI_KER) => {
                            clocks.hsi_ck().expect("SPI6: HSI clock must be enabled")
                        }
                        Some(srdccipr::SPI6SEL_A::CSI_KER) => {
                            clocks.csi_ck().expect("SPI6: CSI clock must be enabled")
                        }
                        Some(srdccipr::SPI6SEL_A::HSE) => {
                            clocks.hse_ck().expect("SPI6: HSE clock must be enabled")
                        }
                        _ => unreachable!(),
                    }
                }
            }
        )+
    }
}

spi! {
    SPI1: (spi1, Spi1, pclk2) => (u8, u16, u32),
    SPI2: (spi2, Spi2, pclk1) => (u8, u16, u32),
    SPI3: (spi3, Spi3, pclk1) => (u8, u16, u32),
    SPI4: (spi4, Spi4, pclk2) => (u8, u16, u32),
    SPI5: (spi5, Spi5, pclk2) => (u8, u16, u32),
    SPI6: (spi6, Spi6, pclk2) => (u8, u16, u32),
}

spi123sel! {
    SPI1, SPI2, SPI3,
}
spi45sel! {
    SPI4, SPI5,
}
spi6sel! {
    SPI6,
}
