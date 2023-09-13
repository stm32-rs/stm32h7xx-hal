//! Serial
//!
//! # Examples
//!
//! - [Simple Blocking Example](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/serial.rs)
//! - [Serial Transfer using DMA](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/serial-dma.rs)
//! - [Advanced USART Functions](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/serial-advanced.rs)
//! - [Inverted Signal Levels](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/serial-inverted-loopback.rs)

use core::fmt;
use core::marker::PhantomData;
use core::ptr;

use embedded_hal::blocking::serial as serial_block;
use embedded_hal::prelude::*;
use embedded_hal::serial;
use nb::block;

use pac::usart1::cr2::{
    CLKEN_A, CPHA_A, CPOL_A, LBCL_A, MSBFIRST_A, RXINV_A, TXINV_A,
};
use pac::usart1::cr3::HDSEL_A;

use crate::gpio;
use crate::gpio::PushPull;
use crate::pac;
use crate::rcc::{rec, CoreClocks, ResetEnable};
#[cfg(feature = "rm0455")]
use crate::stm32::rcc::cdccip2r::{USART16910SEL_A, USART234578SEL_A};
#[cfg(feature = "rm0468")]
use crate::stm32::rcc::d2ccip2r::{USART16910SEL_A, USART234578SEL_A};
#[cfg(any(feature = "rm0433", feature = "rm0399"))]
use crate::stm32::rcc::d2ccip2r::{USART16SEL_A, USART234578SEL_A};

use crate::pac::usart1::cr1::{M0_A as M0, PCE_A as PCE, PS_A as PS};
use crate::time::Hertz;

/// Serial error
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// Framing error
    Framing,
    /// Noise error
    Noise,
    /// RX buffer overrun
    Overrun,
    /// Parity check error
    Parity,
}

/// Interrupt event
#[derive(Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Event {
    /// New data has been received
    Rxne,
    /// New data can be sent
    Txe,
    /// Idle line state detected
    Idle,

    ///Tx threshlold interrupt enable
    Txftie,

    ///Rx threshlold interrupt enable
    Rxftie,
}

pub mod config {
    use crate::time::Hertz;

    #[derive(Copy, Clone, PartialEq, Eq)]
    pub enum FifoThreshold {
        Eighth,
        Quarter,
        Half,
        ThreeQuarter,
        SevenEighth,
        Full,
    }

    /// The parity bits appended to each serial data word
    ///
    /// When enabled parity bits will be automatically added by hardware on transmit, and automatically checked by
    /// hardware on receive. For example, `read()` would return [`Error::Parity`](super::Error::Parity).
    ///
    /// Note that parity bits are included in the serial word length, so if parity is used word length will be set to 9.
    #[derive(Copy, Clone, PartialEq, Eq)]
    pub enum Parity {
        ParityNone,
        ParityEven,
        ParityOdd,
    }
    #[derive(Copy, Clone, PartialEq, Eq)]
    pub enum StopBits {
        #[doc = "1 stop bit"]
        Stop1,
        #[doc = "0.5 stop bits"]
        Stop0p5,
        #[doc = "2 stop bits"]
        Stop2,
        #[doc = "1.5 stop bits"]
        Stop1p5,
    }
    #[derive(Copy, Clone, PartialEq, Eq)]
    pub enum BitOrder {
        LsbFirst,
        MsbFirst,
    }
    #[derive(Copy, Clone, PartialEq, Eq)]
    pub enum ClockPhase {
        First,
        Second,
    }
    #[derive(Copy, Clone, PartialEq, Eq)]
    pub enum ClockPolarity {
        IdleHigh,
        IdleLow,
    }

    /// A structure for specifying the USART or UART configuration. Fields
    /// relating to synchronous mode are ignored for UART peripherals.
    ///
    /// This structure uses builder semantics to generate the configuration.
    ///
    /// ```
    /// let config = Config::new().partity_odd();
    /// ```
    #[derive(Copy, Clone)]
    pub struct Config {
        pub baudrate: Hertz,
        pub parity: Parity,
        pub stopbits: StopBits,
        pub bitorder: BitOrder,
        pub clockphase: ClockPhase,
        pub clockpolarity: ClockPolarity,
        pub lastbitclockpulse: bool,
        pub swaptxrx: bool,
        pub invertrx: bool,
        pub inverttx: bool,
        pub rxfifothreshold: FifoThreshold,
        pub txfifothreshold: FifoThreshold,
        pub halfduplex: bool,
    }

    impl Config {
        /// Create a default configuration for the USART or UART interface
        ///
        /// * 8 bits, 1 stop bit, no parity (8N1)
        /// * LSB first
        pub fn new(frequency: Hertz) -> Self {
            Config {
                baudrate: frequency,
                parity: Parity::ParityNone,
                stopbits: StopBits::Stop1,
                bitorder: BitOrder::LsbFirst,
                clockphase: ClockPhase::First,
                clockpolarity: ClockPolarity::IdleLow,
                lastbitclockpulse: false,
                swaptxrx: false,
                invertrx: false,
                inverttx: false,
                rxfifothreshold: FifoThreshold::Eighth,
                txfifothreshold: FifoThreshold::Eighth,
                halfduplex: false,
            }
        }

        pub fn baudrate(mut self, baudrate: Hertz) -> Self {
            self.baudrate = baudrate;
            self
        }

        pub fn parity_none(mut self) -> Self {
            self.parity = Parity::ParityNone;
            self
        }

        /// Enables Even Parity
        ///
        /// Note that parity bits are included in the serial word length, so if parity is used word length will be set
        /// to 9.
        pub fn parity_even(mut self) -> Self {
            self.parity = Parity::ParityEven;
            self
        }

        /// Enables Odd Parity
        ///
        /// Note that parity bits are included in the serial word length, so if parity is used word length will be set
        /// to 9.
        pub fn parity_odd(mut self) -> Self {
            self.parity = Parity::ParityOdd;
            self
        }

        /// Specify the number of stop bits
        pub fn stopbits(mut self, stopbits: StopBits) -> Self {
            self.stopbits = stopbits;
            self
        }
        /// Specify the bit order
        pub fn bitorder(mut self, bitorder: BitOrder) -> Self {
            self.bitorder = bitorder;
            self
        }
        /// Specify the clock phase. Only applies to USART peripherals
        pub fn clockphase(mut self, clockphase: ClockPhase) -> Self {
            self.clockphase = clockphase;
            self
        }
        /// Specify the clock polarity. Only applies to USART peripherals
        pub fn clockpolarity(mut self, clockpolarity: ClockPolarity) -> Self {
            self.clockpolarity = clockpolarity;
            self
        }
        /// Specify if the last bit transmitted in each word has a corresponding
        /// clock pulse in the SCLK pin. Only applies to USART peripherals
        pub fn lastbitclockpulse(mut self, lastbitclockpulse: bool) -> Self {
            self.lastbitclockpulse = lastbitclockpulse;
            self
        }

        /// If `true`, swap the Tx and Rx pins
        pub fn swaptxrx(mut self, swaptxrx: bool) -> Self {
            self.swaptxrx = swaptxrx;
            self
        }

        /// If `true`, RX pin signal levels are inverted
        pub fn invertrx(mut self, invertrx: bool) -> Self {
            self.invertrx = invertrx;
            self
        }

        /// If `true`, TX pin signal levels are inverted
        pub fn inverttx(mut self, inverttx: bool) -> Self {
            self.inverttx = inverttx;
            self
        }

        pub fn rxfifothreshold(
            mut self,
            rxfifothreshold: FifoThreshold,
        ) -> Self {
            self.rxfifothreshold = rxfifothreshold;
            self
        }

        pub fn txfifothreshold(
            mut self,
            txfifothreshold: FifoThreshold,
        ) -> Self {
            self.txfifothreshold = txfifothreshold;
            self
        }

        /// If `true`, sets to half-duplex mode
        pub fn halfduplex(mut self, halfduplex: bool) -> Self {
            self.halfduplex = halfduplex;
            self
        }
    }

    #[derive(Debug)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct InvalidConfig;

    impl Default for Config {
        fn default() -> Config {
            Self::new(Hertz::from_raw(19_200)) // 19k2 baud
        }
    }

    impl From<Hertz> for Config {
        fn from(frequency: Hertz) -> Config {
            Self::new(frequency)
        }
    }
}

pub trait Pins<USART> {
    const SYNCHRONOUS: bool = false;
    type AltPins;
    fn convert(self) -> Self::AltPins;
}

impl<USART, TX, RX> Pins<USART> for (TX, RX)
where
    USART: SerialAsync,
    TX: Into<USART::Tx<PushPull>>,
    RX: Into<USART::Rx<PushPull>>,
{
    type AltPins = (USART::Tx<PushPull>, USART::Rx<PushPull>);
    fn convert(self) -> Self::AltPins {
        (self.0.into(), self.1.into())
    }
}

impl<USART, TX, RX, CK> Pins<USART> for (TX, RX, CK)
where
    USART: SerialAsync + SerialSync,
    TX: Into<USART::Tx<PushPull>>,
    RX: Into<USART::Rx<PushPull>>,
    CK: Into<USART::Ck>,
{
    const SYNCHRONOUS: bool = true;
    type AltPins = (USART::Tx<PushPull>, USART::Rx<PushPull>, USART::Ck);
    fn convert(self) -> Self::AltPins {
        (self.0.into(), self.1.into(), self.2.into())
    }
}

/// A filler type for when the Tx pin is unnecessary
pub use gpio::NoPin as NoTx;
/// A filler type for when the Rx pin is unnecessary
pub use gpio::NoPin as NoRx;
/// A filler type for when the Ck pin is unnecessary
pub use gpio::NoPin as NoCk;

use gpio::alt::{SerialAsync, SerialSync};

pub trait UartInstance:
    crate::Sealed
    + core::ops::Deref<Target = pac::uart4::RegisterBlock>
    + SerialAsync
{
    /// Returns the frequency of the current kernel clock
    fn kernel_clk(clocks: &CoreClocks) -> Option<Hertz>;
    /// Returns the frequency of the current kernel clock
    ///
    /// # Panics
    ///
    /// Panics if the kernel clock is not running
    fn kernel_clk_unwrap(clocks: &CoreClocks) -> Hertz {
        Self::kernel_clk(clocks).unwrap()
    }
}

pub trait UsartInstance:
    crate::Sealed
    + core::ops::Deref<Target = pac::usart1::RegisterBlock>
    + SerialAsync
    + SerialSync
{
    /// Returns the frequency of the current kernel clock
    fn kernel_clk(clocks: &CoreClocks) -> Option<Hertz>;
    /// Returns the frequency of the current kernel clock
    ///
    /// # Panics
    ///
    /// Panics if the kernel clock is not running
    fn kernel_clk_unwrap(clocks: &CoreClocks) -> Hertz {
        Self::kernel_clk(clocks).unwrap()
    }
}

/// Serial abstraction
pub struct Serial<USART> {
    pub(crate) usart: USART,
    ker_ck: Hertz,
}

/// Serial receiver
pub struct Rx<USART> {
    _usart: PhantomData<USART>,
    ker_ck: Hertz,
}

/// Serial transmitter
pub struct Tx<USART> {
    _usart: PhantomData<USART>,
}

pub trait SerialExt: Sized {
    type Rec: ResetEnable;

    fn serial<P: Pins<Self>>(
        self,
        pins: P,
        config: impl Into<config::Config>,
        prec: Self::Rec,
        clocks: &CoreClocks,
    ) -> Result<Serial<Self>, config::InvalidConfig>;

    fn serial_unchecked(
        self,
        config: impl Into<config::Config>,
        prec: Self::Rec,
        clocks: &CoreClocks,
        synchronous: bool,
    ) -> Result<Serial<Self>, config::InvalidConfig>;

    #[deprecated(since = "0.7.0", note = "Deprecated in favour of .serial(..)")]
    fn usart(
        self,
        pins: impl Pins<Self>,
        config: impl Into<config::Config>,
        prec: Self::Rec,
        clocks: &CoreClocks,
    ) -> Result<Serial<Self>, config::InvalidConfig> {
        self.serial(pins, config, prec, clocks)
    }

    #[deprecated(
        since = "0.7.0",
        note = "Deprecated in favour of .serial_unchecked(..)"
    )]
    fn usart_unchecked(
        self,
        config: impl Into<config::Config>,
        prec: Self::Rec,
        clocks: &CoreClocks,
        synchronous: bool,
    ) -> Result<Serial<Self>, config::InvalidConfig> {
        self.serial_unchecked(config, prec, clocks, synchronous)
    }
}

macro_rules! replace_expr {
    ($_t:tt $sub:expr) => {
        $sub
    };
}
macro_rules! usart {
    ($(
        $USARTX:ty: ($usartX:ident, $Rec:ident, $pclkX:ident $(, $synchronous:ident)?),
    )+) => {
        $(
            /// Configures a USART peripheral to provide serial
            /// communication
            impl Serial<$USARTX> {
                pub fn $usartX(
                    usart: $USARTX,
                    config: impl Into<config::Config>,
                    prec: rec::$Rec,
                    clocks: &CoreClocks
                    $(, $synchronous: bool)?
                ) -> Result<Self, config::InvalidConfig>
                {
                    // Enable clock for USART and reset
                    prec.enable().reset();

                    let ker_ck = <$USARTX>::kernel_clk_unwrap(clocks);
                    let mut serial = Serial { usart, ker_ck };
                    let config = config.into();
                    serial.usart.cr1.reset();

                    // If synchronous mode is supported, check that it is not
                    // enabled alongside half duplex mode
                    $(
                        if config.halfduplex & $synchronous {
                            return Err(config::InvalidConfig);
                        }
                    )?
                    serial.configure(&config $(, $synchronous )?);

                    Ok(serial)
                }

                /// Runs the serial port configuration process
                ///
                /// The serial port must be disabled when called.
                fn configure(&mut self, config: &config::Config $(, $synchronous: bool)?) {
                    use crate::stm32::usart1::cr2::STOP_A as STOP;
                    use self::config::*;

                    // Prescaler not used for now
                    let usart_ker_ck_presc = self.ker_ck;
                    self.usart.presc.reset();

                    // Calculate baudrate divisor
                    let usartdiv = usart_ker_ck_presc / config.baudrate;
                    assert!(usartdiv <= 65_536);

                    // 16 times oversampling, OVER8 = 0
                    let brr = usartdiv as u16;
                    self.usart.brr.write(|w| { w.brr().bits(brr) });

                    // Reset registers to disable advanced USART features
                    self.usart.cr2.reset();
                    self.usart.cr3.reset();

                    // RXFIFO threshold
                    let fifo_threshold_bits = match config.rxfifothreshold {
                        FifoThreshold::Eighth => 0,
                        FifoThreshold::Quarter => 1,
                        FifoThreshold::Half => 2,
                        FifoThreshold::ThreeQuarter => 3,
                        FifoThreshold::SevenEighth => 4,
                        FifoThreshold::Full => 5,
                    };
                    unsafe {
                        self.usart.cr3.modify(|_, w| w.rxftcfg().bits(fifo_threshold_bits));
                    }

                    // TXFIFO threashold
                    let fifo_threshold_bits = match config.txfifothreshold {
                        FifoThreshold::Eighth => 0,
                        FifoThreshold::Quarter => 1,
                        FifoThreshold::Half => 2,
                        FifoThreshold::ThreeQuarter => 3,
                        FifoThreshold::SevenEighth => 4,
                        FifoThreshold::Full => 5,
                    };
                    unsafe {
                        self.usart.cr3.modify(|_, w| w.txftcfg().bits(fifo_threshold_bits));
                    }

                    // Configure half-duplex mode
                    self.usart.cr3.modify(|_, w| {
                        w.hdsel().variant(if config.halfduplex {
                            HDSEL_A::Selected
                        } else {
                            HDSEL_A::NotSelected
                        })
                    });

                    // Configure serial mode
                    self.usart.cr2.write(|w| {
                        w.stop().variant(match config.stopbits {
                            StopBits::Stop0p5 => STOP::Stop0p5,
                            StopBits::Stop1 => STOP::Stop1,
                            StopBits::Stop1p5 => STOP::Stop1p5,
                            StopBits::Stop2 => STOP::Stop2,
                        });

                        w.msbfirst().variant(match config.bitorder {
                            BitOrder::LsbFirst => MSBFIRST_A::Lsb,
                            BitOrder::MsbFirst => MSBFIRST_A::Msb,
                        });

                        w.swap().bit(config.swaptxrx);

                        w.rxinv().variant(if config.invertrx {
                            RXINV_A::Inverted
                        } else {
                            RXINV_A::Standard
                        });

                        w.txinv().variant(if config.inverttx {
                            TXINV_A::Inverted
                        } else {
                            TXINV_A::Standard
                        });

                        // If synchronous mode is not supported, these bits are
                        // reserved and must be kept at reset value
                        $(
                            w.lbcl().variant(if config.lastbitclockpulse {
                                LBCL_A::Output
                            } else {
                                LBCL_A::NotOutput
                            });

                            w.clken().variant(if $synchronous {
                                CLKEN_A::Enabled
                            } else {
                                CLKEN_A::Disabled
                            });

                            w.cpol().variant(match config.clockpolarity {
                                ClockPolarity::IdleHigh =>CPOL_A::High,
                                ClockPolarity::IdleLow =>CPOL_A::Low
                            });

                            w.cpha().variant(match config.clockphase {
                                ClockPhase::First => CPHA_A::First,
                                ClockPhase::Second => CPHA_A::Second
                            });
                        )?

                        w
                    });

                    // Enable transmission and receiving and configure frame
                    // Retain enabled events
                    self.usart.cr1.modify(|_, w| {
                        w.fifoen()
                            .set_bit() // FIFO mode enabled
                            .over8()
                            .oversampling16() // Oversampling by 16
                            .ue()
                            .enabled()
                            .te()
                            .enabled()
                            .re()
                            .enabled()
                            .m1()
                            .clear_bit()
                            .m0()
                            .variant(match config.parity {
                                Parity::ParityNone => M0::Bit8,
                                _ => M0::Bit9,
                            }).pce()
                            .variant(match config.parity {
                                Parity::ParityNone => PCE::Disabled,
                                _ => PCE::Enabled,
                            }).ps()
                            .variant(match config.parity {
                                Parity::ParityOdd => PS::Odd,
                                _ => PS::Even,
                            })
                    });
                }

                /// Applies the configuration to the serial port.
                ///
                /// Ensure that the serial port is not transmitting data when calling this method.
                ///
                /// # Panics
                ///
                /// Panics if DMA Rx or Tx are enabled.
                pub fn reconfigure(&mut self, config: impl Into<config::Config> $(, $synchronous: bool)?) {
                    if self.dma_rx_enabled() || self.dma_tx_enabled() {
                        panic!("Cannot reconfigure serial while DMA enabled");
                    }

                    self.usart.cr1.modify(|_, w| w.ue().disabled());

                    let config = config.into();
                    self.configure(&config $(, $synchronous )?);
                }

                /// Enables the Rx DMA stream.
                pub fn enable_dma_rx(&mut self) {
                    self.usart.cr3.modify(|_, w| w.dmar().set_bit());
                }

                /// Disables the Rx DMA stream.
                pub fn disable_dma_rx(&mut self) {
                    self.usart.cr3.modify(|_, w| w.dmar().clear_bit());
                }

                /// Returns `true` if the Rx DMA stream is enabled.
                pub fn dma_rx_enabled(&self) -> bool {
                    self.usart.cr3.read().dmar().bit_is_set()
                }

                /// Enables the Tx DMA stream.
                pub fn enable_dma_tx(&mut self) {
                    self.usart.cr3.modify(|_, w| w.dmat().set_bit());
                }

                /// Disables the Tx DMA stream.
                pub fn disable_dma_tx(&mut self) {
                    self.usart.cr3.modify(|_, w| w.dmat().clear_bit());
                }

                /// Returns `true` if the Tx DMA stream is enabled.
                pub fn dma_tx_enabled(&self) -> bool {
                    self.usart.cr3.read().dmat().bit_is_set()
                }

                /// Starts listening for an interrupt event
                pub fn listen(&mut self, event: Event) {
                    match event {
                        Event::Rxne => {
                            self.usart.cr1.modify(|_, w| w.rxneie().enabled())
                        },
                        Event::Txe => {
                            self.usart.cr1.modify(|_, w| w.txeie().enabled())
                        },
                        Event::Idle => {
                            self.usart.cr1.modify(|_, w| w.idleie().enabled())
                        },
                        Event::Txftie => {
                            self.usart.cr3.modify(|_, w| w.txftie().set_bit())
                        },
                        Event::Rxftie => {
                            self.usart.cr3.modify(|_, w| w.rxftie().set_bit())
                        },
                    }
                }

                /// Stop listening for an interrupt event
                pub fn unlisten(&mut self, event: Event) {
                    match event {
                        Event::Rxne => {
                            self.usart.cr1.modify(|_, w| w.rxneie().disabled())
                        },
                        Event::Txe => {
                            self.usart.cr1.modify(|_, w| w.txeie().disabled())
                        },
                        Event::Idle => {
                            self.usart.cr1.modify(|_, w| w.idleie().disabled())
                        },
                        Event::Txftie => {
                            self.usart.cr3.modify(|_, w| w.txftie().clear_bit())
                        },
                        Event::Rxftie => {
                            self.usart.cr3.modify(|_, w| w.rxftie().clear_bit())
                        },
                    }
                    let _ = self.usart.cr1.read();
                    let _ = self.usart.cr1.read(); // Delay 2 peripheral clocks
                }

                /// Return true if the line idle status is set
                ///
                /// The line idle status bit is set when the peripheral detects the receive line is idle.
                /// The bit is cleared by software, by calling `clear_idle()`.
                pub fn is_idle(&self) -> bool {
                    unsafe { (*<$USARTX>::ptr()).isr.read().idle().bit_is_set() }
                }

                /// Clear the line idle status bit
                pub fn clear_idle(&mut self) {
                    unsafe { (*<$USARTX>::ptr()).icr.write(|w| w.idlecf().set_bit()) }
                    let _ = self.usart.isr.read();
                    let _ = self.usart.isr.read(); // Delay 2 peripheral clocks
                }

                /// Return true if the line busy status is set
                ///
                /// The busy status bit is set when there is communication active on the receive line,
                /// and reset at the end of reception.
                pub fn is_busy(&self) -> bool {
                    unsafe { (*<$USARTX>::ptr()).isr.read().busy().bit_is_set() }
                }

                /// Return true if the tx register is empty (and can accept data)
                pub fn is_txe(&self) -> bool {
                    unsafe { (*<$USARTX>::ptr()).isr.read().txe().bit_is_set() }
                }

                /// Return true if the rx register is not empty (and can be read)
                pub fn is_rxne(&self) -> bool {
                    unsafe { (*<$USARTX>::ptr()).isr.read().rxne().bit_is_set() }
                }

                /// Splits the [`Serial`] struct into transmit ([`Tx`]) and receive ([`Rx`]) parts which can be used
                /// separately.
                pub fn split(self) -> (Tx<$USARTX>, Rx<$USARTX>) {
                    (
                        Tx {
                            _usart: PhantomData,
                        },
                        Rx {
                            _usart: PhantomData,
                            ker_ck: self.ker_ck,
                        },
                    )
                }

                /// Combines the [`Tx`] and [`Rx`] structs from [`Serial::split()`] into a [`Serial`]
                #[allow(unused_variables)]
                pub fn join(tx: Tx<$USARTX>, rx: Rx<$USARTX>) -> Self {
                    assert_eq!(core::mem::size_of::<$USARTX>(), 0);
                    Self {
                        usart: unsafe { core::mem::zeroed::<$USARTX>() },
                        ker_ck: rx.ker_ck,
                    }
                }

                /// Releases the USART peripheral
                pub fn release(self) -> $USARTX {
                    // Wait until both TXFIFO and shift register are empty
                    while self.usart.isr.read().tc().bit_is_clear() {}

                    self.usart
                }

                /// Returns a reference to the inner peripheral
                pub fn inner(&self) -> &$USARTX {
                    &self.usart
                }

                /// Returns a mutable reference to the inner peripheral
                pub fn inner_mut(&mut self) -> &mut $USARTX {
                    &mut self.usart
                }
            }

            impl SerialExt for $USARTX {
                type Rec = rec::$Rec;

                fn serial<P: Pins<$USARTX>>(self,
                         pins: P,
                         config: impl Into<config::Config>,
                         prec: rec::$Rec,
                         clocks: &CoreClocks
                ) -> Result<Serial<Self>, config::InvalidConfig>
                {
                    let _pins = pins.convert();
                    Serial::$usartX(
                        self, config, prec, clocks
                            $(, replace_expr!($synchronous P::SYNCHRONOUS))?
                    )
                }

                fn serial_unchecked(self,
                                   config: impl Into<config::Config>,
                                   prec: rec::$Rec,
                                   clocks: &CoreClocks,
                                   #[allow(unused)]
                                   synchronous: bool,
                ) -> Result<Serial<$USARTX>, config::InvalidConfig>
                {
                    Serial::$usartX(
                        self, config, prec, clocks
                            $(, replace_expr!($synchronous synchronous))?
                    )
                }
            }

            impl serial::Read<u8> for Serial<$USARTX> {
                type Error = Error;

                fn read(&mut self) -> nb::Result<u8, Error> {
                    let mut rx: Rx<$USARTX> = Rx {
                        _usart: PhantomData,
                        ker_ck: self.ker_ck,
                    };
                    rx.read()
                }
            }

            impl serial::Read<u8> for Rx<$USARTX> {
                type Error = Error;

                fn read(&mut self) -> nb::Result<u8, Error> {
                    // NOTE(unsafe) atomic read with no side effects
                    let isr = unsafe { (*<$USARTX>::ptr()).isr.read() };

                    Err(if isr.pe().bit_is_set() {
                        unsafe { (*<$USARTX>::ptr()).icr.write(|w| w.pecf().clear() );};
                        nb::Error::Other(Error::Parity)
                    } else if isr.fe().bit_is_set() {
                        unsafe { (*<$USARTX>::ptr()).icr.write(|w| w.fecf().clear() );};
                        nb::Error::Other(Error::Framing)
                    } else if isr.nf().bit_is_set() {
                        unsafe { (*<$USARTX>::ptr()).icr.write(|w| w.ncf().clear() );};
                        nb::Error::Other(Error::Noise)
                    } else if isr.ore().bit_is_set() {
                        unsafe { (*<$USARTX>::ptr()).icr.write(|w| w.orecf().clear() );};
                        nb::Error::Other(Error::Overrun)
                    } else if isr.rxne().bit_is_set() {
                        // NOTE(read_volatile) see `write_volatile` below
                        return Ok(unsafe {
                            ptr::read_volatile(&(*<$USARTX>::ptr()).rdr as *const _ as *const _)
                        });
                    } else {
                        nb::Error::WouldBlock
                    })
                }
            }

            impl Rx<$USARTX> {
                /// Start listening for `Rxne` event
                pub fn listen(&mut self) {
                    // unsafe: rxneie bit accessed by Rx part only
                    unsafe { &*<$USARTX>::ptr() }.cr1.modify(|_, w| w.rxneie().enabled());
                }

                /// Stop listening for `Rxne` event
                pub fn unlisten(&mut self) {
                    // unsafe: rxneie bit accessed by Rx part only
                    let cr1 = &unsafe { &*<$USARTX>::ptr() }.cr1;
                    cr1.modify(|_, w| w.rxneie().disabled());
                    let _ = cr1.read();
                    let _ = cr1.read(); // Delay 2 peripheral clocks
                }

                /// Enables the Rx DMA stream.
                pub fn enable_dma_rx(&mut self) {
                    // unsafe: dmar bit accessed by Rx part only
                    unsafe { &*<$USARTX>::ptr() }.cr3.modify(|_, w| w.dmar().set_bit());
                }

                /// Disables the Rx DMA stream.
                pub fn disable_dma_rx(&mut self) {
                    // unsafe: dmar bit accessed by Rx part only
                    unsafe { &*<$USARTX>::ptr() }.cr3.modify(|_, w| w.dmar().clear_bit());
                }

                /// Return true if the line idle status is set
                ///
                /// The line idle status bit is set when the peripheral detects the receive line is idle.
                /// The bit is cleared by software, by calling `clear_idle()`.
                pub fn is_idle(&self) -> bool {
                    unsafe { (*<$USARTX>::ptr()).isr.read().idle().bit_is_set() }
                }

                /// Clear the line idle status bit
                pub fn clear_idle(&mut self) {
                    let usart = unsafe { &*<$USARTX>::ptr() };
                    usart.icr.write(|w| w.idlecf().set_bit());
                    let _ = usart.isr.read();
                    let _ = usart.isr.read(); // Delay 2 peripheral clocks
                }

                /// Return true if the line busy status is set
                ///
                /// The busy status bit is set when there is communication active on the receive line,
                /// and reset at the end of reception.
                pub fn is_busy(&self) -> bool {
                    unsafe { (*<$USARTX>::ptr()).isr.read().busy().bit_is_set() }
                }

                /// Return true if the rx register is not empty (and can be read)
                pub fn is_rxne(&self) -> bool {
                    unsafe { (*<$USARTX>::ptr()).isr.read().rxne().bit_is_set() }
                }
            }

            impl serial::Write<u8> for Serial<$USARTX> {
                type Error = core::convert::Infallible;

                fn flush(&mut self) -> nb::Result<(), Self::Error> {
                    let mut tx: Tx<$USARTX> = Tx {
                        _usart: PhantomData,
                    };
                    tx.flush()
                }

                fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
                    let mut tx: Tx<$USARTX> = Tx {
                        _usart: PhantomData,
                    };
                    tx.write(byte)
                }
            }

            impl serial_block::write::Default<u8> for Serial<$USARTX> {
                //implement marker trait to opt-in to default blocking write implementation
            }

            impl serial::Write<u8> for Tx<$USARTX> {
                // NOTE(Void) See section "29.7 USART interrupts"; the
                // only possible errors during transmission are: clear
                // to send (which is disabled in this case) errors and
                // framing errors (which only occur in SmartCard
                // mode); neither of these apply to our hardware
                // configuration
                type Error = core::convert::Infallible;

                fn flush(&mut self) -> nb::Result<(), Self::Error> {
                    // NOTE(unsafe) atomic read with no side effects
                    let isr = unsafe { (*<$USARTX>::ptr()).isr.read() };

                    if isr.tc().bit_is_set() {
                        Ok(())
                    } else {
                        Err(nb::Error::WouldBlock)
                    }
                }

                fn write(&mut self, byte: u8) -> nb::Result<(), Self::Error> {
                    // NOTE(unsafe) atomic read with no side effects
                    let isr = unsafe { (*<$USARTX>::ptr()).isr.read() };

                    if isr.txe().bit_is_set() {
                        // NOTE(unsafe) atomic write to stateless register
                        // NOTE(write_volatile) 8-bit write that's not
                        // possible through the svd2rust API
                        unsafe {
                            ptr::write_volatile(
                                &(*<$USARTX>::ptr()).tdr as *const _ as *mut _, byte)
                        }
                        Ok(())
                    } else {
                        Err(nb::Error::WouldBlock)
                    }
                }
            }

            impl Tx<$USARTX> {
                /// Start listening for `Txe` event
                pub fn listen(&mut self) {
                    // unsafe: txeie bit accessed by Tx part only
                    unsafe { &*<$USARTX>::ptr() }.cr1.modify(|_, w| w.txeie().enabled());
                }

                /// Stop listening for `Txe` event
                pub fn unlisten(&mut self) {
                    // unsafe: txeie bit accessed by Tx part only
                    let cr1 = &unsafe { &*<$USARTX>::ptr() }.cr1;
                    cr1.modify(|_, w| w.txeie().disabled());
                    let _ = cr1.read();
                    let _ = cr1.read(); // Delay 2 peripheral clocks
                }

                /// Enables the Tx DMA stream.
                pub fn enable_dma_tx(&mut self) {
                    // unsafe: dmat bit accessed by Tx part only
                    unsafe { &*<$USARTX>::ptr() }.cr3.modify(|_, w| w.dmat().set_bit());
                }

                /// Disables the Tx DMA stream.
                pub fn disable_dma_tx(&mut self) {
                    // unsafe: dmat bit accessed by Tx part only
                    unsafe { &*<$USARTX>::ptr() }.cr3.modify(|_, w| w.dmat().clear_bit());
                }

                /// Return true if the tx register is empty (and can accept data)
                pub fn is_txe(& self) -> bool {
                    unsafe { (*<$USARTX>::ptr()).isr.read().txe().bit_is_set() }
                }
            }
        )+
    }
}

macro_rules! usart_sel {
	($ccip:ident, $SEL:ident, $sel:ident, $PCLK:ident, $pclk:ident;
     $($USARTX:ty: $Instance:ident, $usartX:ident, $doc:expr $(, $Ck:ident)?;)+) => {
	    $(
            impl crate::Sealed for $USARTX {}
            impl $Instance for $USARTX {
                /// Returns the frequency of the current kernel clock for
                #[doc=$doc]
                fn kernel_clk(clocks: &CoreClocks) -> Option<Hertz> {
                    // unsafe: read only
                    let ccip = unsafe { (*pac::RCC::ptr()).$ccip.read() };

                    match ccip.$sel().variant() {
                        Some($SEL::$PCLK) => Some(clocks.$pclk()),
                        Some($SEL::Pll2Q) => clocks.pll2_q_ck(),
                        Some($SEL::Pll3Q) => clocks.pll3_q_ck(),
                        Some($SEL::HsiKer) => clocks.hsi_ck(),
                        Some($SEL::CsiKer) => clocks.csi_ck(),
                        Some($SEL::Lse) => unimplemented!(),
                        _ => unreachable!(),
                    }
                }
                /// Returns the frequency of the current kernel clock for
                #[doc=$doc]
                ///
                /// # Panics
                ///
                /// Panics if the kernel clock is not running
                fn kernel_clk_unwrap(clocks: &CoreClocks) -> Hertz {
                    // unsafe: read only
                    let ccip = unsafe { (*pac::RCC::ptr()).$ccip.read() };

                    match ccip.$sel().variant() {
                        Some($SEL::$PCLK) => clocks.$pclk(),
                        Some($SEL::Pll2Q) => {
                            clocks.pll2_q_ck().expect(
                                concat!(stringify!($USARTX), ": PLL2_Q must be enabled")
                            )
                        }
                        Some($SEL::Pll3Q) => {
                            clocks.pll3_q_ck().expect(
                                concat!(stringify!($USARTX), ": PLL3_Q must be enabled")
                            )
                        }
                        Some($SEL::HsiKer) => {
                            clocks.hsi_ck().expect(
                                concat!(stringify!($USARTX), ": HSI clock must be enabled")
                            )
                        }
                        Some($SEL::CsiKer) => {
                            clocks.csi_ck().expect(
                                concat!(stringify!($USARTX), ": CSI clock must be enabled")
                            )
                        }
                        Some($SEL::Lse) => unimplemented!(),
                        _ => unreachable!(),
                    }
                }
            }
        )+
    }
}

usart! {
    pac::USART1: (usart1, Usart1, pclk2, synchronous),
    pac::USART2: (usart2, Usart2, pclk1, synchronous),
    pac::USART3: (usart3, Usart3, pclk1, synchronous),
    pac::USART6: (usart6, Usart6, pclk2, synchronous),

    pac::UART4: (uart4, Uart4, pclk1),
    pac::UART5: (uart5, Uart5, pclk1),
    pac::UART7: (uart7, Uart7, pclk1),
    pac::UART8: (uart8, Uart8, pclk1),
}
#[cfg(any(feature = "rm0455", feature = "rm0468"))]
usart! {
    pac::UART9: (uart9, Uart9, pclk2),
    pac::USART10: (usart10, Usart10, pclk2, synchronous),
}

#[cfg(any(feature = "rm0433", feature = "rm0399"))]
usart_sel! {
    d2ccip2r, USART16SEL_A, usart16sel, RccPclk2, pclk2;

    pac::USART1: UsartInstance, usart1, "USART1", Ck;
    pac::USART6: UsartInstance, usart6, "USART6", Ck;
}
#[cfg(feature = "rm0455")]
usart_sel! {
    cdccip2r, USART16910SEL_A, usart16910sel, RccPclk2, pclk2;

    pac::USART1: UsartInstance, usart1, "USART1", Ck;
    pac::USART6: UsartInstance, usart6, "USART6", Ck;
    pac::USART10: UsartInstance, usart10, "USART10", Ck;

    pac::UART9: UartInstance, uart9, "UART9";
}
#[cfg(feature = "rm0468")]
usart_sel! {
    d2ccip2r, USART16910SEL_A, usart16910sel, RccPclk2, pclk2;

    pac::USART1: UsartInstance, usart1, "USART1", Ck;
    pac::USART6: UsartInstance, usart6, "USART6", Ck;
    pac::USART10: UsartInstance, usart10, "USART10", Ck;

    pac::UART9: UartInstance, uart9, "UART9";
}

#[cfg(not(feature = "rm0455"))]
usart_sel! {
    d2ccip2r, USART234578SEL_A, usart234578sel, RccPclk1, pclk1;

    pac::USART2: UsartInstance, usart2, "USART2", Ck;
    pac::USART3: UsartInstance, usart3, "USART3", Ck;

    pac::UART4: UartInstance, uart4, "UART4";
    pac::UART5: UartInstance, uart5, "UART5";
    pac::UART8: UartInstance, uart8, "UART8";
    pac::UART7: UartInstance, uart7, "UART7";
}
#[cfg(feature = "rm0455")]
usart_sel! {
    cdccip2r, USART234578SEL_A, usart234578sel, RccPclk1, pclk1;

    pac::USART2: UsartInstance, usart2, "USART2", Ck;
    pac::USART3: UsartInstance, usart3, "USART3", Ck;

    pac::UART4: UartInstance, uart4, "UART4";
    pac::UART5: UartInstance, uart5, "UART5";
    pac::UART8: UartInstance, uart8, "UART8";
    pac::UART7: UartInstance, uart7, "UART7";
}

impl<USART> fmt::Write for Tx<USART>
where
    Tx<USART>: serial::Write<u8>,
{
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let _ = s.as_bytes().iter().map(|c| block!(self.write(*c))).last();
        Ok(())
    }
}

impl<USART> fmt::Write for Serial<USART>
where
    Serial<USART>: serial::Write<u8>,
{
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let _ = s.as_bytes().iter().map(|c| block!(self.write(*c))).last();
        Ok(())
    }
}
