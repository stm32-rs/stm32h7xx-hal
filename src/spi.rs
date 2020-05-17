//! Serial Peripheral Interface (SPI) bus
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
use crate::stm32;
use crate::stm32::rcc::{d2ccip1r, d3ccipr};
use crate::stm32::spi1::cfg1::MBR_A as MBR;
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
use crate::gpio::gpioj::{PJ10, PJ11};
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

/// A filler type for when the SCK pin is unnecessary
pub struct NoSck;
/// A filler type for when the Miso pin is unnecessary
pub struct NoMiso;
/// A filler type for when the Mosi pin is unnecessary
pub struct NoMosi;

macro_rules! pins {
    ($($SPIX:ty: SCK: [$($SCK:ty),*] MISO: [$($MISO:ty),*] MOSI: [$($MOSI:ty),*])+) => {
        $(
            $(
                impl PinSck<$SPIX> for $SCK {}
            )*
            $(
                impl PinMiso<$SPIX> for $MISO {}
            )*
            $(
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
            PK0<Alternate<AF5>>
        ]
        MISO: [
            NoMiso,
            PF8<Alternate<AF5>>,
            PH7<Alternate<AF5>>,
            PJ11<Alternate<AF5>>
        ]
        MOSI: [
            NoMosi,
            PF9<Alternate<AF5>>,
            PF11<Alternate<AF5>>,
            PJ10<Alternate<AF5>>
        ]
    SPI6:
        SCK: [
            NoSck,
            PA5<Alternate<AF8>>,
            PB3<Alternate<AF8>>,
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
pub enum Event {
    /// New data has been received
    Rxp,
    /// Data can be sent
    Txp,
    /// An error occurred
    Error,
}

#[derive(Debug)]
pub struct Spi<SPI, WORD = u8> {
    spi: SPI,
    _word: PhantomData<WORD>,
}

pub trait SpiExt<SPI, WORD>: Sized {
    type Rec: ResetEnable;

    fn spi<PINS, T>(
        self,
        _pins: PINS,
        mode: Mode,
        freq: T,
        prec: Self::Rec,
        clocks: &CoreClocks,
    ) -> Spi<SPI, WORD>
    where
        PINS: Pins<SPI>,
        T: Into<Hertz>;

    fn spi_unchecked<T>(
        self,
        mode: Mode,
        freq: T,
        prec: Self::Rec,
        clocks: &CoreClocks,
    ) -> Spi<SPI, WORD>
    where
        T: Into<Hertz>;
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
                impl Spi<$SPIX, $TY> {
                    pub fn $spiX<T>(
                        spi: $SPIX,
                        mode: Mode,
                        freq: T,
                        prec: rec::$Rec,
                        clocks: &CoreClocks,
                    ) -> Self
                    where
                        T: Into<Hertz>,
                    {
                        // Enable clock for SPI
                        prec.enable();

                        // Disable SS output
                        spi.cfg2.write(|w| w.ssoe().disabled());

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

                        // mstr: master configuration
                        // lsbfrst: MSB first
                        // ssm: enable software slave management (NSS pin
                        // free for other uses)
                        // comm: full-duplex
                        spi.cfg2.write(|w| {
                            w.cpha()
                                .bit(mode.phase ==
                                     Phase::CaptureOnSecondTransition)
                                .cpol()
                                .bit(mode.polarity == Polarity::IdleHigh)
                                .master()
                                .master()
                                .lsbfrst()
                                .msbfirst()
                                .ssm()
                                .enabled()
                                .comm()
                                .full_duplex()
                        });

                        // spe: enable the SPI bus
                        spi.cr1.write(|w| w.ssi().slave_not_selected().spe().enabled());

                        Spi { spi, _word: PhantomData }
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

                    pub fn free(self) -> ($SPIX, rec::$Rec) {
                        (self.spi, rec::$Rec { _marker: PhantomData })
                    }
                }

                impl SpiExt<$SPIX, $TY> for $SPIX {
                    type Rec = rec::$Rec;

	                fn spi<PINS, T>(self,
                                    _pins: PINS,
                                    mode: Mode,
                                    freq: T,
                                    prec: rec::$Rec,
                                    clocks: &CoreClocks) -> Spi<$SPIX, $TY>
	                where
	                    PINS: Pins<$SPIX>,
	                    T: Into<Hertz>
	                {
	                    Spi::<$SPIX, $TY>::$spiX(self, mode, freq, prec, clocks)
	                }

	                fn spi_unchecked<T>(self,
                                        mode: Mode,
                                        freq: T,
                                        prec: rec::$Rec,
                                        clocks: &CoreClocks) -> Spi<$SPIX, $TY>
	                where
	                    T: Into<Hertz>
	                {
	                    Spi::<$SPIX, $TY>::$spiX(self, mode, freq, prec, clocks)
	                }
	            }

                impl hal::spi::FullDuplex<$TY> for Spi<$SPIX, $TY> {
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
                    for Spi<$SPIX, $TY> {}

                impl hal::blocking::spi::write::Default<$TY>
                    for Spi<$SPIX, $TY> {}
            )+
        )+
	}
}

macro_rules! spi123sel {
	($($SPIX:ident,)+) => {
	    $(
            impl<WORD> Spi<$SPIX, WORD> {
                /// Returns the frequency of the current kernel clock
                /// for SPI1, SPI2, SPI3
                fn kernel_clk(clocks: &CoreClocks) -> Option<Hertz> {
                    let d2ccip1r = unsafe { (*stm32::RCC::ptr()).d2ccip1r.read() };

                    match d2ccip1r.spi123sel().variant() {
                        Val(d2ccip1r::SPI123SEL_A::PLL1_Q) => clocks.pll1_q_ck(),
                        Val(d2ccip1r::SPI123SEL_A::PLL2_P) => clocks.pll2_p_ck(),
                        Val(d2ccip1r::SPI123SEL_A::PLL3_P) => clocks.pll3_p_ck(),
                        // Need a method of specifying pin clock
                        Val(d2ccip1r::SPI123SEL_A::I2S_CKIN) => unimplemented!(),
                        Val(d2ccip1r::SPI123SEL_A::PER) => clocks.per_ck(),
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
            impl<WORD> Spi<$SPIX, WORD> {
                /// Returns the frequency of the current kernel clock
                /// for SPI4, SPI5
                fn kernel_clk(clocks: &CoreClocks) -> Option<Hertz> {
                    let d2ccip1r = unsafe { (*stm32::RCC::ptr()).d2ccip1r.read() };

                    match d2ccip1r.spi45sel().variant() {
                        Val(d2ccip1r::SPI45SEL_A::APB) => Some(clocks.pclk2()),
                        Val(d2ccip1r::SPI45SEL_A::PLL2_Q) => clocks.pll2_q_ck(),
                        Val(d2ccip1r::SPI45SEL_A::PLL3_Q) => clocks.pll3_q_ck(),
                        Val(d2ccip1r::SPI45SEL_A::HSI_KER) => clocks.hsi_ck(),
                        Val(d2ccip1r::SPI45SEL_A::CSI_KER) => clocks.csi_ck(),
                        Val(d2ccip1r::SPI45SEL_A::HSE) => clocks.hse_ck(),
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
            impl<WORD> Spi<$SPIX, WORD> {
                /// Returns the frequency of the current kernel clock
                /// for SPI6
                fn kernel_clk(clocks: &CoreClocks) -> Option<Hertz> {
                    let d3ccipr = unsafe { (*stm32::RCC::ptr()).d3ccipr.read() };

                    match d3ccipr.spi6sel().variant() {
                        Val(d3ccipr::SPI6SEL_A::RCC_PCLK4) => Some(clocks.pclk4()),
                        Val(d3ccipr::SPI6SEL_A::PLL2_Q) => clocks.pll2_q_ck(),
                        Val(d3ccipr::SPI6SEL_A::PLL3_Q) => clocks.pll3_q_ck(),
                        Val(d3ccipr::SPI6SEL_A::HSI_KER) => clocks.hsi_ck(),
                        Val(d3ccipr::SPI6SEL_A::CSI_KER) => clocks.csi_ck(),
                        Val(d3ccipr::SPI6SEL_A::HSE) => clocks.hse_ck(),
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
