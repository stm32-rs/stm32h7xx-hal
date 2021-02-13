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
//! [embedded_hal]: https://docs.rs/embedded-hal/0.2.3/embedded_hal/spi/index.html

use crate::hal;
pub use crate::hal::spi::{
    Mode, Phase, Polarity, MODE_0, MODE_1, MODE_2, MODE_3,
};

#[cfg(feature = "rm0455")]
use crate::stm32::rcc::{cdccip1r as ccip1r, srdccipr};
#[cfg(not(feature = "rm0455"))]
use crate::stm32::rcc::{d2ccip1r as ccip1r, d3ccipr as srdccipr};

use crate::stm32;
use crate::stm32::spi1::{cfg1::MBR_A as MBR, cfg2::COMM_A as COMM};
use core::convert::From;
use core::marker::PhantomData;
use core::ptr;
use nb;
use stm32h7::Variant::Val;

use crate::stm32::{SPI1, SPI2, SPI3, SPI4, SPI5, SPI6};

use crate::gpio::gpioa::{PA12, PA5, PA6, PA7, PA9};
use crate::gpio::gpiob::{PB10, PB13, PB14, PB15, PB2, PB3, PB4, PB5};
use crate::gpio::gpioc::{PC1, PC10, PC11, PC12, PC2, PC3};
use crate::gpio::gpiod::{PD3, PD6, PD7};
use crate::gpio::gpioe::{PE12, PE13, PE14, PE2, PE5, PE6};
use crate::gpio::gpiof::{PF11, PF7, PF8, PF9};
use crate::gpio::gpiog::{PG11, PG12, PG13, PG14, PG9};
use crate::gpio::gpioh::{PH6, PH7};
use crate::gpio::gpioi::{PI1, PI2, PI3};
#[cfg(not(feature = "stm32h7b0"))]
use crate::gpio::gpioj::{PJ10, PJ11};
#[cfg(not(feature = "stm32h7b0"))]
use crate::gpio::gpiok::PK0;

use crate::gpio::{Alternate, AF5, AF6, AF7, AF8};

use crate::rcc::{rec, CoreClocks, ResetEnable};
use crate::time::Hertz;

/// SPI error
#[derive(Debug)]
pub enum Error {
    /// Overrun occurred
    Overrun,
    /// Mode fault occurred
    ModeFault,
    /// CRC error
    Crc,
    #[doc(hidden)]
    _Extensible,
}

/// Enabled SPI peripheral (type state)
pub struct Enabled;
/// Disabled SPI peripheral (type state)
pub struct Disabled;

pub trait Pins<SPI> {}
pub trait PinSck<SPI> {}
pub trait PinMiso<SPI> {}
pub trait PinMosi<SPI> {}

impl<SPI, SCK, MISO, MOSI> Pins<SPI> for (SCK, MISO, MOSI)
where
    SCK: PinSck<SPI>,
    MISO: PinMiso<SPI>,
    MOSI: PinMosi<SPI>,
{
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
    cs_delay: f32,
    managed_cs: bool,
    suspend_when_inactive: bool,
    communication_mode: CommunicationMode,
}

impl Config {
    /// Create a default configuration for the SPI interface.
    ///
    /// Arguments:
    /// * `mode` - The SPI mode to configure.
    pub fn new(mode: Mode) -> Self {
        Config {
            mode: mode,
            swap_miso_mosi: false,
            cs_delay: 0.0,
            managed_cs: false,
            suspend_when_inactive: false,
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

    /// Specify a delay between CS assertion and the beginning of the SPI transaction.
    ///
    /// Note:
    /// * This function introduces a delay on SCK from the initiation of the transaction. The delay
    /// is specified as a number of SCK cycles, so the actual delay may vary.
    ///
    /// Arguments:
    /// * `delay` - The delay between CS assertion and the start of the transaction in seconds.
    /// register for the output pin.
    pub fn cs_delay(mut self, delay: f32) -> Self {
        self.cs_delay = delay;
        self
    }

    /// CS pin is automatically managed by the SPI peripheral.
    ///
    /// # Note
    /// SPI is configured in "endless transaction" mode, which means that the SPI CSn pin will
    /// assert when the first data is sent and will not de-assert.
    ///
    /// If CSn should be de-asserted between each data transfer, use `suspend_when_inactive()` as
    /// well.
    pub fn manage_cs(mut self) -> Self {
        self.managed_cs = true;
        self
    }

    /// Suspend a transaction automatically if data is not available in the FIFO.
    ///
    /// # Note
    /// This will de-assert CSn when no data is available for transmission and hardware is managing
    /// the CSn pin.
    pub fn suspend_when_inactive(mut self) -> Self {
        self.suspend_when_inactive = true;
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
       MOSI: [$($( #[ $pmeta3:meta ] )* $MOSI:ty),*])+) => {
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
        )+
    }
}

pins! {
    SPI1:
        SCK: [
            NoSck,
            PA5<Alternate<AF5>>,
            PB3<Alternate<AF5>>,
            PG11<Alternate<AF5>>
        ]
        MISO: [
            NoMiso,
            PA6<Alternate<AF5>>,
            PB4<Alternate<AF5>>,
            PG9<Alternate<AF5>>
        ]
        MOSI: [
            NoMosi,
            PA7<Alternate<AF5>>,
            PB5<Alternate<AF5>>,
            PD7<Alternate<AF5>>
        ]
    SPI2:
        SCK: [
            NoSck,
            PA9<Alternate<AF5>>,
            PA12<Alternate<AF5>>,
            PB10<Alternate<AF5>>,
            PB13<Alternate<AF5>>,
            PD3<Alternate<AF5>>,
            PI1<Alternate<AF5>>
        ]
        MISO: [
            NoMiso,
            PB14<Alternate<AF5>>,
            PC2<Alternate<AF5>>,
            PI2<Alternate<AF5>>
        ]
        MOSI: [
            NoMosi,
            PB15<Alternate<AF5>>,
            PC1<Alternate<AF5>>,
            PC3<Alternate<AF5>>,
            PI3<Alternate<AF5>>
        ]
    SPI3:
        SCK: [
            NoSck,
            PB3<Alternate<AF6>>,
            PC10<Alternate<AF6>>
        ]
        MISO: [
            NoMiso,
            PB4<Alternate<AF6>>,
            PC11<Alternate<AF6>>
        ]
        MOSI: [
            NoMosi,
            PB2<Alternate<AF7>>,
            PB5<Alternate<AF7>>,
            PC12<Alternate<AF6>>,
            PD6<Alternate<AF5>>
        ]
    SPI4:
        SCK: [
            NoSck,
            PE2<Alternate<AF5>>,
            PE12<Alternate<AF5>>
        ]
        MISO: [
            NoMiso,
            PE5<Alternate<AF5>>,
            PE13<Alternate<AF5>>
        ]
        MOSI: [
            NoMosi,
            PE6<Alternate<AF5>>,
            PE14<Alternate<AF5>>
        ]
    SPI5:
        SCK: [
            NoSck,
            PF7<Alternate<AF5>>,
            PH6<Alternate<AF5>>,
            #[cfg(not(feature = "stm32h7b0"))]
            PK0<Alternate<AF5>>
        ]
        MISO: [
            NoMiso,
            PF8<Alternate<AF5>>,
            PH7<Alternate<AF5>>,
            #[cfg(not(feature = "stm32h7b0"))]
            PJ11<Alternate<AF5>>
        ]
        MOSI: [
            NoMosi,
            PF9<Alternate<AF5>>,
            PF11<Alternate<AF5>>,
            #[cfg(not(feature = "stm32h7b0"))]
            PJ10<Alternate<AF5>>
        ]
    SPI6:
        SCK: [
            NoSck,
            PA5<Alternate<AF8>>,
            PB3<Alternate<AF8>>,
            #[cfg(feature = "rm0455")]
            PC12<Alternate<AF5>>,
            PG13<Alternate<AF5>>
        ]
        MISO: [
            NoMiso,
            PA6<Alternate<AF8>>,
            PB4<Alternate<AF8>>,
            PG12<Alternate<AF5>>
        ]
        MOSI: [
            NoMosi,
            PA7<Alternate<AF8>>,
            PB5<Alternate<AF8>>,
            PG14<Alternate<AF5>>
        ]
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

macro_rules! spi {
    (DSIZE, $spi:ident,  u8) => {
        $spi.cfg1.modify(|_, w| {
            w.dsize()
                .bits(8 - 1) // 8 bit frames
        });
    };
    (DSIZE, $spi:ident, u16) => {
        $spi.cfg1.modify(|_, w| {
            w.dsize()
                .bits(16 - 1) // 16 bit frames
        });
    };
	($($SPIX:ident: ($spiX:ident, $Rec:ident, $pclkX:ident)
       => ($($TY:ident),+),)+) => {
	    $(
            // For each $TY
            $(
                impl Spi<$SPIX, Enabled, $TY> {
                    pub fn $spiX<T, CONFIG>(
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
	                    let spi_ker_ck = match Self::kernel_clk(clocks) {
                            Some(ker_hz) => ker_hz.0,
                            _ => panic!("$SPIX kernel clock not running!")
                        };
                        let mbr = match spi_ker_ck / spi_freq {
                            0 => unreachable!(),
                            1..=2 => MBR::DIV2,
                            3..=5 => MBR::DIV4,
                            6..=11 => MBR::DIV8,
                            12..=23 => MBR::DIV16,
                            24..=47 => MBR::DIV32,
                            48..=95 => MBR::DIV64,
                            96..=191 => MBR::DIV128,
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
                        let (start_cycle_delay, interdata_cycle_delay) = {
                            let mut delay: u32 = (config.cs_delay * spi_freq as f32) as u32;

                            // If the cs-delay is specified as non-zero, add 1 to the delay cycles
                            // before truncation to an integer to ensure that we have at least as
                            // many cycles as required.
                            if config.cs_delay > 0.0_f32 {
                                delay = delay + 1;
                            }

                            if delay > 0xF {
                                delay = 0xF;
                            }

                            // If CS suspends while data is inactive, we also require an
                            // "inter-data" delay.
                            if config.suspend_when_inactive {
                                (delay as u8, delay as u8)
                            } else {
                                (delay as u8, 0_u8)
                            }
                        };

                        // The calculated cycle delay may not be more than 4 bits wide for the
                        // configuration register.
                        let communication_mode = match config.communication_mode {
                            CommunicationMode::Transmitter => COMM::TRANSMITTER,
                            CommunicationMode::Receiver => COMM::RECEIVER,
                            CommunicationMode::FullDuplex => COMM::FULLDUPLEX,
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
                                .bit(config.suspend_when_inactive)
                                .ssm()
                                .bit(config.managed_cs == false)
                                .ssoe()
                                .bit(config.managed_cs == true)
                                .mssi()
                                .bits(start_cycle_delay)
                                .midi()
                                .bits(interdata_cycle_delay)
                                .ioswp()
                                .bit(config.swap_miso_mosi == true)
                                .comm()
                                .variant(communication_mode)
                        });

                        // spe: enable the SPI bus
                        spi.cr1.write(|w| w.ssi().slave_not_selected().spe().enabled());

                        Spi { spi, _word: PhantomData, _ed: PhantomData }
                    }

                    /// Disables the SPI peripheral. Any SPI operation is
                    /// stopped and disabled, the internal state machine is
                    /// reset, all the FIFOs content is flushed, the MODF
                    /// flag is cleared, the SSI flag is cleared, and the
                    /// CRC calculation is re-initialized. Clocks are not
                    /// disabled.
                    pub fn disable(self) -> Spi<$SPIX, Disabled, $TY> {
                        // Master communication must be suspended before the peripheral is disabled
                        self.spi.cr1.modify(|_, w| w.csusp().requested());
                        while self.spi.sr.read().eot().is_completed() {}
                        self.spi.cr1.write(|w| w.ssi().slave_not_selected().spe().disabled());
                        Spi {
                            spi: self.spi,
                            _word: PhantomData,
                            _ed: PhantomData,
                        }
                    }
                }

                impl Spi<$SPIX, Disabled, $TY> {
                    /// Enables the SPI peripheral.
                    /// Clears the MODF flag, the SSI flag, and sets the SPE bit.
                    pub fn enable(mut self) -> Spi<$SPIX, Enabled, $TY> {
                        self.clear_modf(); // SPE cannot be set when MODF is set
                        self.spi.cr1.write(|w| w.ssi().slave_not_selected().spe().enabled());
                        Spi {
                            spi: self.spi,
                            _word: PhantomData,
                            _ed: PhantomData,
                        }
                    }

                    /// Enables the Rx DMA stream. If the DMA Rx is used, the
                    /// reference manual recommends that this is enabled before
                    /// enabling the DMA
                    pub fn enable_dma_rx(&mut self) {
                        self.spi.cfg1.modify(|_,w| w.rxdmaen().enabled());
                    }

                    /// Enables the Tx DMA stream. If the DMA Tx is used, the
                    /// reference manual recommends that this is enabled after
                    /// enablign the DMA
                    pub fn enable_dma_tx(&mut self) {
                        self.spi.cfg1.modify(|_,w| w.txdmaen().enabled());
                    }

                    /// Deconstructs the SPI peripheral and returns the component parts.
                    pub fn free(self) -> ($SPIX, rec::$Rec) {
                        (self.spi, rec::$Rec { _marker: PhantomData })
                    }
                }

                impl<EN> Spi<$SPIX, EN, $TY>
                {
                    /// Returns a mutable reference to the inner peripheral
                    pub fn inner(&self) -> &$SPIX {
                        &self.spi
                    }

                    /// Returns a mutable reference to the inner peripheral
                    pub fn inner_mut(&mut self) -> &mut $SPIX {
                        &mut self.spi
                    }

                    /// Enable interrupts for the given `event`:
                    ///  - Received data ready to be read (RXP)
                    ///  - Transmit data register empty (TXP)
                    ///  - Error
                    pub fn listen(&mut self, event: Event) {
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
                    pub fn unlisten(&mut self, event: Event) {
                        match event {
                            Event::Rxp => self.spi.ier.modify(|_, w|
                                                              w.rxpie().masked()),
                            Event::Txp => self.spi.ier.modify(|_, w|
                                                              w.txpie().masked()),
                            Event::Error => self.spi.ier.modify(|_, w| {
                                w.udrie() // Underrun
                                    .masked()
                                    .ovrie() // Overrun
                                    .masked()
                                    .crceie() // CRC error
                                    .masked()
                                    .modfie() // Mode fault
                                    .masked()
                            }),
                        }
                    }

                    /// Return `true` if the TXP flag is set, i.e. new
                    /// data to transmit can be written to the SPI.
                    pub fn is_txp(&self) -> bool {
                        self.spi.sr.read().txp().is_not_full()
                    }

                    /// Return `true` if the RXP flag is set, i.e. new
                    /// data has been received and can be read from the
                    /// SPI.
                    pub fn is_rxp(&self) -> bool {
                        self.spi.sr.read().rxp().is_not_empty()
                    }

                    /// Return `true` if the MODF flag is set, i.e. the
                    /// SPI has experienced a mode fault
                    pub fn is_modf(&self) -> bool {
                        self.spi.sr.read().modf().is_fault()
                    }

                    /// Return `true` if the OVR flag is set, i.e. new
                    /// data has been received while the receive data
                    /// register was already filled.
                    pub fn is_ovr(&self) -> bool {
                        self.spi.sr.read().ovr().is_overrun()
                    }

                    /// Clears the MODF flag, which indicates that a
                    /// mode fault has occurred.
                    pub fn clear_modf(&mut self) {
                        self.spi.ifcr.write(|w| w.modfc().clear());
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
                        let sr = self.spi.sr.read();

                        Err(if sr.ovr().is_overrun() {
                            nb::Error::Other(Error::Overrun)
                        } else if sr.modf().is_fault() {
                            nb::Error::Other(Error::ModeFault)
                        } else if sr.crce().is_error() {
                            nb::Error::Other(Error::Crc)
                        } else if sr.rxp().is_not_empty() {
                            // NOTE(read_volatile) read only 1 byte (the
                            // svd2rust API only allows reading a
                            // half-word)
                            return Ok(unsafe {
                                ptr::read_volatile(
                                    &self.spi.rxdr as *const _ as *const $TY,
                                )
                            });
                        } else {
                            nb::Error::WouldBlock
                        })
                    }

                    fn send(&mut self, byte: $TY) -> nb::Result<(), Error> {
                        let sr = self.spi.sr.read();

                        Err(if sr.ovr().is_overrun() {
                            nb::Error::Other(Error::Overrun)
                        } else if sr.modf().is_fault() {
                            nb::Error::Other(Error::ModeFault)
                        } else if sr.crce().is_error() {
                            nb::Error::Other(Error::Crc)
                        } else if sr.txp().is_not_full() {
                            // NOTE(write_volatile) see note above
                            unsafe {
                                ptr::write_volatile(
                                    &self.spi.txdr as *const _ as *mut $TY,
                                    byte,
                                )
                            }
                            // write CSTART to start a transaction in
                            // master mode
                            self.spi.cr1.modify(|_, w| w.cstart().started());

                            return Ok(());
                        } else {
                            nb::Error::WouldBlock
                        })
                    }
                }

                impl hal::blocking::spi::transfer::Default<$TY>
                    for Spi<$SPIX, Enabled, $TY> {}

                impl hal::blocking::spi::write::Default<$TY>
                    for Spi<$SPIX, Enabled, $TY> {}
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
                fn kernel_clk(clocks: &CoreClocks) -> Option<Hertz> {
                    #[cfg(not(feature = "rm0455"))]
                    let ccip1r = unsafe { (*stm32::RCC::ptr()).d2ccip1r.read() };
                    #[cfg(feature = "rm0455")]
                    let ccip1r = unsafe { (*stm32::RCC::ptr()).cdccip1r.read() };

                    match ccip1r.spi123sel().variant() {
                        Val(ccip1r::SPI123SEL_A::PLL1_Q) => clocks.pll1_q_ck(),
                        Val(ccip1r::SPI123SEL_A::PLL2_P) => clocks.pll2_p_ck(),
                        Val(ccip1r::SPI123SEL_A::PLL3_P) => clocks.pll3_p_ck(),
                        // Need a method of specifying pin clock
                        Val(ccip1r::SPI123SEL_A::I2S_CKIN) => unimplemented!(),
                        Val(ccip1r::SPI123SEL_A::PER) => clocks.per_ck(),
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
                fn kernel_clk(clocks: &CoreClocks) -> Option<Hertz> {
                    #[cfg(not(feature = "rm0455"))]
                    let ccip1r = unsafe { (*stm32::RCC::ptr()).d2ccip1r.read() };
                    #[cfg(feature = "rm0455")]
                    let ccip1r = unsafe { (*stm32::RCC::ptr()).cdccip1r.read() };

                    match ccip1r.spi45sel().variant() {
                        Val(ccip1r::SPI45SEL_A::APB) => Some(clocks.pclk2()),
                        Val(ccip1r::SPI45SEL_A::PLL2_Q) => clocks.pll2_q_ck(),
                        Val(ccip1r::SPI45SEL_A::PLL3_Q) => clocks.pll3_q_ck(),
                        Val(ccip1r::SPI45SEL_A::HSI_KER) => clocks.hsi_ck(),
                        Val(ccip1r::SPI45SEL_A::CSI_KER) => clocks.csi_ck(),
                        Val(ccip1r::SPI45SEL_A::HSE) => clocks.hse_ck(),
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
                fn kernel_clk(clocks: &CoreClocks) -> Option<Hertz> {
                    #[cfg(not(feature = "rm0455"))]
                    let srdccipr = unsafe { (*stm32::RCC::ptr()).d3ccipr.read() };
                    #[cfg(feature = "rm0455")]
                    let srdccipr = unsafe { (*stm32::RCC::ptr()).srdccipr.read() };

                    match srdccipr.spi6sel().variant() {
                        Val(srdccipr::SPI6SEL_A::RCC_PCLK4) => Some(clocks.pclk4()),
                        Val(srdccipr::SPI6SEL_A::PLL2_Q) => clocks.pll2_q_ck(),
                        Val(srdccipr::SPI6SEL_A::PLL3_Q) => clocks.pll3_q_ck(),
                        Val(srdccipr::SPI6SEL_A::HSI_KER) => clocks.hsi_ck(),
                        Val(srdccipr::SPI6SEL_A::CSI_KER) => clocks.csi_ck(),
                        Val(srdccipr::SPI6SEL_A::HSE) => clocks.hse_ck(),
                        _ => unreachable!(),
                    }
                }
            }
        )+
    }
}

spi! {
    SPI1: (spi1, Spi1, pclk2) => (u8, u16),
    SPI2: (spi2, Spi2, pclk1) => (u8, u16),
    SPI3: (spi3, Spi3, pclk1) => (u8, u16),
    SPI4: (spi4, Spi4, pclk2) => (u8, u16),
    SPI5: (spi5, Spi5, pclk2) => (u8, u16),
    SPI6: (spi6, Spi6, pclk2) => (u8, u16),
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
