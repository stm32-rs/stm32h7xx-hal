//! Serial

use core::fmt;
use core::marker::PhantomData;
use core::ptr;

use embedded_hal::blocking::serial as serial_block;
use embedded_hal::prelude::*;
use embedded_hal::serial;
use nb::block;

use crate::stm32;
use crate::stm32::usart1::cr1::{M0_A as M0, PCE_A as PCE, PS_A as PS};
use stm32h7::Variant::Val;

#[cfg(feature = "rm0455")]
use crate::stm32::rcc::cdccip2r::{USART16910SEL_A, USART234578SEL_A};
#[cfg(not(feature = "rm0455"))]
use crate::stm32::rcc::d2ccip2r::{USART16SEL_A, USART234578SEL_A};

use crate::stm32::{UART4, UART5, UART7, UART8};
use crate::stm32::{USART1, USART2, USART3, USART6};

use crate::gpio::gpioa::{
    PA0, PA1, PA10, PA11, PA12, PA15, PA2, PA3, PA4, PA8, PA9,
};
use crate::gpio::gpiob::{
    PB10, PB11, PB12, PB13, PB14, PB15, PB3, PB4, PB5, PB6, PB7, PB8, PB9,
};
use crate::gpio::gpioc::{PC10, PC11, PC12, PC6, PC7, PC8};
use crate::gpio::gpiod::{PD0, PD1, PD10, PD2, PD5, PD6, PD7, PD8, PD9};
use crate::gpio::gpioe::{PE0, PE1, PE7, PE8};
use crate::gpio::gpiof::{PF6, PF7};
use crate::gpio::gpiog::{PG14, PG7, PG9};
use crate::gpio::gpioh::{PH13, PH14};
use crate::gpio::gpioi::PI9;
#[cfg(not(feature = "stm32h7b0"))]
use crate::gpio::gpioj::{PJ8, PJ9};

use crate::gpio::{Alternate, AF11, AF14, AF4, AF6, AF7, AF8};
use crate::rcc::{rec, CoreClocks, ResetEnable};
use crate::time::Hertz;

use crate::Never;

/// Serial error
#[derive(Debug)]
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
#[derive(Copy, Clone, PartialEq)]
pub enum Event {
    /// New data has been received
    Rxne,
    /// New data can be sent
    Txe,
    /// Idle line state detected
    Idle,
}

pub mod config {
    use crate::time::Hertz;

    #[derive(Copy, Clone, PartialEq)]
    pub enum WordLength {
        DataBits8,
        DataBits9,
    }

    #[derive(Copy, Clone, PartialEq)]
    pub enum Parity {
        ParityNone,
        ParityEven,
        ParityOdd,
    }

    #[derive(Copy, Clone, PartialEq)]
    pub enum StopBits {
        #[doc = "1 stop bit"]
        STOP1,
        #[doc = "0.5 stop bits"]
        STOP0P5,
        #[doc = "2 stop bits"]
        STOP2,
        #[doc = "1.5 stop bits"]
        STOP1P5,
    }

    pub struct Config {
        pub baudrate: Hertz,
        pub wordlength: WordLength,
        pub parity: Parity,
        pub stopbits: StopBits,
    }

    impl Config {
        pub fn baudrate(mut self, baudrate: impl Into<Hertz>) -> Self {
            self.baudrate = baudrate.into();
            self
        }

        pub fn parity_none(mut self) -> Self {
            self.parity = Parity::ParityNone;
            self
        }

        pub fn parity_even(mut self) -> Self {
            self.parity = Parity::ParityEven;
            self
        }

        pub fn parity_odd(mut self) -> Self {
            self.parity = Parity::ParityOdd;
            self
        }

        pub fn wordlength_8(mut self) -> Self {
            self.wordlength = WordLength::DataBits8;
            self
        }

        pub fn wordlength_9(mut self) -> Self {
            self.wordlength = WordLength::DataBits9;
            self
        }

        pub fn stopbits(mut self, stopbits: StopBits) -> Self {
            self.stopbits = stopbits;
            self
        }
    }

    #[derive(Debug)]
    pub struct InvalidConfig;

    impl Default for Config {
        fn default() -> Config {
            Config {
                baudrate: Hertz(19_200), // 19k2 baud
                wordlength: WordLength::DataBits8,
                parity: Parity::ParityNone,
                stopbits: StopBits::STOP1,
            }
        }
    }

    impl<T: Into<Hertz>> From<T> for Config {
        fn from(f: T) -> Config {
            Config {
                baudrate: f.into(),
                ..Default::default()
            }
        }
    }
}

pub trait Pins<USART> {}
pub trait PinTx<USART> {}
pub trait PinRx<USART> {}
pub trait PinCk<USART> {}

impl<USART, TX, RX> Pins<USART> for (TX, RX)
where
    TX: PinTx<USART>,
    RX: PinRx<USART>,
{
}

/// A filler type for when the Tx pin is unnecessary
pub struct NoTx;
/// A filler type for when the Rx pin is unnecessary
pub struct NoRx;
/// A filler type for when the Ck pin is unnecessary
pub struct NoCk;

macro_rules! usart_pins {
    ($($USARTX:ty: TX: [$($TX:ty),*] RX: [$($RX:ty),*] CK: [$($CK:ty),*])+) => {
        $(
            $(
                impl PinTx<$USARTX> for $TX {}
            )*
            $(
                impl PinRx<$USARTX> for $RX {}
            )*
            $(
                impl PinCk<$USARTX> for $CK {}
            )*
        )+
    }
}
macro_rules! uart_pins {
    ($($UARTX:ty:
       TX: [$($( #[ $pmeta1:meta ] )* $TX:ty),*]
       RX: [$($( #[ $pmeta2:meta ] )* $RX:ty),*]
    )+) => {
        $(
            $(
                $( #[ $pmeta1 ] )*
                impl PinTx<$UARTX> for $TX {}
            )*
            $(
                $( #[ $pmeta2 ] )*
                impl PinRx<$UARTX> for $RX {}
            )*
        )+
    }
}

usart_pins! {
    USART1:
        TX: [
            NoTx,
            PA9<Alternate<AF7>>,
            PB6<Alternate<AF7>>,
            PB14<Alternate<AF4>>
        ]
        RX: [
            NoRx,
            PA10<Alternate<AF7>>,
            PB7<Alternate<AF7>>,
            PB15<Alternate<AF4>>
        ]
        CK: [
            NoCk,
            PA8<Alternate<AF7>>
        ]
    USART2:
        TX: [
            NoTx,
            PA2<Alternate<AF7>>,
            PD5<Alternate<AF7>>
        ]
        RX: [
            NoRx,
            PA3<Alternate<AF7>>,
            PD6<Alternate<AF7>>
        ]
        CK: [
            NoCk,
            PA4<Alternate<AF7>>,
            PD7<Alternate<AF7>>
        ]
    USART3:
        TX: [
            NoTx,
            PB10<Alternate<AF7>>,
            PC10<Alternate<AF7>>,
            PD8<Alternate<AF7>>
        ]
        RX: [
            NoRx,
            PB11<Alternate<AF7>>,
            PC11<Alternate<AF7>>,
            PD9<Alternate<AF7>>
        ]
        CK: [
            NoCk,
            PB12<Alternate<AF7>>,
            PC12<Alternate<AF7>>,
            PD10<Alternate<AF7>>
        ]
    USART6:
        TX: [
            NoTx,
            PC6<Alternate<AF7>>,
            PG14<Alternate<AF7>>
        ]
        RX: [
            NoRx,
            PC7<Alternate<AF7>>,
            PG9<Alternate<AF7>>
        ]
        CK: [
            NoCk,
            PC8<Alternate<AF7>>,
            PG7<Alternate<AF7>>
        ]
}
uart_pins! {
    UART4:
        TX: [
            NoTx,
            PA0<Alternate<AF8>>,
            PA12<Alternate<AF6>>,
            PB9<Alternate<AF8>>,
            PC10<Alternate<AF8>>,
            PD1<Alternate<AF8>>,
            PH13<Alternate<AF8>>
        ]
        RX: [
            NoRx,
            PA1<Alternate<AF8>>,
            PA11<Alternate<AF6>>,
            PB8<Alternate<AF8>>,
            PC11<Alternate<AF8>>,
            PD0<Alternate<AF8>>,
            PH14<Alternate<AF8>>,
            PI9<Alternate<AF8>>
        ]
    UART5:
        TX: [
            NoTx,
            PB6<Alternate<AF14>>,
            PB13<Alternate<AF14>>,
            PC12<Alternate<AF8>>
        ]
        RX: [
            NoRx,
            PB5<Alternate<AF14>>,
            PB12<Alternate<AF14>>,
            PD2<Alternate<AF8>>
        ]
    UART7:
        TX: [
            NoTx,
            PA15<Alternate<AF11>>,
            PB4<Alternate<AF11>>,
            PE8<Alternate<AF7>>,
            PF7<Alternate<AF7>>
        ]
        RX: [
            NoRx,
            PA8<Alternate<AF11>>,
            PB3<Alternate<AF11>>,
            PE7<Alternate<AF7>>,
            PF6<Alternate<AF7>>
        ]
    UART8:
        TX: [
            NoTx,
            PE1<Alternate<AF8>>,
            #[cfg(not(feature = "stm32h7b0"))]
            PJ8<Alternate<AF8>>
        ]
        RX: [
            NoRx,
            PE0<Alternate<AF8>>,
            #[cfg(not(feature = "stm32h7b0"))]
            PJ9<Alternate<AF8>>
        ]
}

/// Serial abstraction
pub struct Serial<USART> {
    pub(crate) usart: USART,
}

/// Serial receiver
pub struct Rx<USART> {
    _usart: PhantomData<USART>,
}

/// Serial transmitter
pub struct Tx<USART> {
    _usart: PhantomData<USART>,
}

pub trait SerialExt<USART>: Sized {
    type Rec: ResetEnable;

    fn serial(
        self,
        _pins: impl Pins<USART>,
        config: impl Into<config::Config>,
        prec: Self::Rec,
        clocks: &CoreClocks,
    ) -> Result<Serial<USART>, config::InvalidConfig>;

    fn serial_unchecked(
        self,
        config: impl Into<config::Config>,
        prec: Self::Rec,
        clocks: &CoreClocks,
    ) -> Result<Serial<USART>, config::InvalidConfig>;

    #[deprecated(since = "0.7.0", note = "Deprecated in favour of .serial(..)")]
    fn usart(
        self,
        pins: impl Pins<USART>,
        config: impl Into<config::Config>,
        prec: Self::Rec,
        clocks: &CoreClocks,
    ) -> Result<Serial<USART>, config::InvalidConfig> {
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
    ) -> Result<Serial<USART>, config::InvalidConfig> {
        self.serial_unchecked(config, prec, clocks)
    }
}

macro_rules! usart {
    ($(
        $USARTX:ident: ($usartX:ident, $Rec:ident, $pclkX:ident),
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
                ) -> Result<Self, config::InvalidConfig>
                {
                    use crate::stm32::usart1::cr2::STOP_A as STOP;
                    use self::config::*;

                    let config = config.into();

                    // Enable clock for USART and reset
                    prec.enable().reset();

                    // Get kernel clock
	                let usart_ker_ck = match Self::kernel_clk(clocks) {
                        Some(ker_hz) => ker_hz.0,
                        _ => panic!("$USARTX kernel clock not running!")
                    };

                    // Prescaler not used for now
                    let usart_ker_ck_presc = usart_ker_ck;
                    usart.presc.reset();

                    // Calculate baudrate divisor
                    let usartdiv = usart_ker_ck_presc / config.baudrate.0;
                    assert!(usartdiv <= 65_536);

                    // 16 times oversampling, OVER8 = 0
                    let brr = usartdiv as u16;
                    usart.brr.write(|w| { w.brr().bits(brr) });

                    // disable hardware flow control
                    // TODO enable DMA
                    // usart.cr3.write(|w| w.rtse().clear_bit().ctse().clear_bit());

                    // Reset registers to disable advanced USART features
                    usart.cr2.reset();
                    usart.cr3.reset();

                    // Set stop bits
                    usart.cr2.write(|w| {
                        w.stop().variant(match config.stopbits {
                            StopBits::STOP0P5 => STOP::STOP0P5,
                            StopBits::STOP1 => STOP::STOP1,
                            StopBits::STOP1P5 => STOP::STOP1P5,
                            StopBits::STOP2 => STOP::STOP2,
                        })
                    });

                    // Enable transmission and receiving
                    // and configure frame
                    usart.cr1.write(|w| {
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
                            .variant(match config.wordlength {
                                WordLength::DataBits8 => M0::BIT8,
                                WordLength::DataBits9 => M0::BIT9,
                            }).pce()
                            .variant(match config.parity {
                                Parity::ParityNone => PCE::DISABLED,
                                _ => PCE::ENABLED,
                            }).ps()
                            .variant(match config.parity {
                                Parity::ParityOdd => PS::EVEN,
                                _ => PS::ODD,
                            })
                    });

                    Ok(Serial { usart })
                }

                /// Enables the Rx DMA stream.
                pub fn enable_dma_rx(&mut self) {
                    self.usart.cr3.modify(|_, w| w.dmar().set_bit());
                }

                /// Disables the Rx DMA stream.
                pub fn disable_dma_rx(&mut self) {
                    self.usart.cr3.modify(|_, w| w.dmar().clear_bit());
                }

                /// Enables the Tx DMA stream.
                pub fn enable_dma_tx(&mut self) {
                    self.usart.cr3.modify(|_, w| w.dmat().set_bit());
                }

                /// Disables the Tx DMA stream.
                pub fn disable_dma_tx(&mut self) {
                    self.usart.cr3.modify(|_, w| w.dmat().clear_bit());
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
                    }
                    let _ = self.usart.cr1.read();
                    let _ = self.usart.cr1.read(); // Delay 2 peripheral clocks
                }

                /// Return true if the line idle status is set
                ///
                /// The line idle status bit is set when the peripheral detects the receive line is idle.
                /// The bit is cleared by software, by calling `clear_idle()`.
                pub fn is_idle(&self) -> bool {
                    unsafe { (*$USARTX::ptr()).isr.read().idle().bit_is_set() }
                }

                /// Clear the line idle status bit
                pub fn clear_idle(&mut self) {
                    unsafe { (*$USARTX::ptr()).icr.write(|w| w.idlecf().set_bit()) }
                    let _ = self.usart.isr.read();
                    let _ = self.usart.isr.read(); // Delay 2 peripheral clocks
                }

                /// Return true if the line busy status is set
                ///
                /// The busy status bit is set when there is communication active on the receive line,
                /// and reset at the end of reception.
                pub fn is_busy(&self) -> bool {
                    unsafe { (*$USARTX::ptr()).isr.read().busy().bit_is_set() }
                }

                /// Return true if the tx register is empty (and can accept data)
                pub fn is_txe(&self) -> bool {
                    unsafe { (*$USARTX::ptr()).isr.read().txe().bit_is_set() }
                }

                /// Return true if the rx register is not empty (and can be read)
                pub fn is_rxne(&self) -> bool {
                    unsafe { (*$USARTX::ptr()).isr.read().rxne().bit_is_set() }
                }

                pub fn split(self) -> (Tx<$USARTX>, Rx<$USARTX>) {
                    (
                        Tx {
                            _usart: PhantomData,
                        },
                        Rx {
                            _usart: PhantomData,
                        },
                    )
                }
                /// Releases the USART peripheral
                pub fn release(self) -> $USARTX {
                    // Wait until both TXFIFO and shift register are empty
                    while self.usart.isr.read().tc().bit_is_clear() {}

                    self.usart
                }
            }

            impl SerialExt<$USARTX> for $USARTX {
                type Rec = rec::$Rec;

                fn serial(self,
                         _pins: impl Pins<$USARTX>,
                         config: impl Into<config::Config>,
                         prec: rec::$Rec,
                         clocks: &CoreClocks
                ) -> Result<Serial<$USARTX>, config::InvalidConfig>
                {
                    Serial::$usartX(self, config, prec, clocks)
                }

                fn serial_unchecked(self,
                                   config: impl Into<config::Config>,
                                   prec: rec::$Rec,
                                   clocks: &CoreClocks
                ) -> Result<Serial<$USARTX>, config::InvalidConfig>
                {
                    Serial::$usartX(self, config, prec, clocks)
                }
            }

            impl serial::Read<u8> for Serial<$USARTX> {
                type Error = Error;

                fn read(&mut self) -> nb::Result<u8, Error> {
                    let mut rx: Rx<$USARTX> = Rx {
                        _usart: PhantomData,
                    };
                    rx.read()
                }
            }

            impl serial::Read<u8> for Rx<$USARTX> {
                type Error = Error;

                fn read(&mut self) -> nb::Result<u8, Error> {
                    // NOTE(unsafe) atomic read with no side effects
                    let isr = unsafe { (*$USARTX::ptr()).isr.read() };

                    Err(if isr.pe().bit_is_set() {
                        unsafe { (*$USARTX::ptr()).icr.write(|w| w.pecf().clear() );};
                        nb::Error::Other(Error::Parity)
                    } else if isr.fe().bit_is_set() {
                        unsafe { (*$USARTX::ptr()).icr.write(|w| w.fecf().clear() );};
                        nb::Error::Other(Error::Framing)
                    } else if isr.nf().bit_is_set() {
                        unsafe { (*$USARTX::ptr()).icr.write(|w| w.ncf().clear() );};
                        nb::Error::Other(Error::Noise)
                    } else if isr.ore().bit_is_set() {
                        unsafe { (*$USARTX::ptr()).icr.write(|w| w.orecf().clear() );};
                        nb::Error::Other(Error::Overrun)
                    } else if isr.rxne().bit_is_set() {
                        // NOTE(read_volatile) see `write_volatile` below
                        return Ok(unsafe {
                            ptr::read_volatile(&(*$USARTX::ptr()).rdr as *const _ as *const _)
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
                    unsafe { &*$USARTX::ptr() }.cr1.modify(|_, w| w.rxneie().enabled());
                }

                /// Stop listening for `Rxne` event
                pub fn unlisten(&mut self) {
                    // unsafe: rxneie bit accessed by Rx part only
                    let cr1 = &unsafe { &*$USARTX::ptr() }.cr1;
                    cr1.modify(|_, w| w.rxneie().disabled());
                    let _ = cr1.read();
                    let _ = cr1.read(); // Delay 2 peripheral clocks
                }

                /// Enables the Rx DMA stream.
                pub fn enable_dma_rx(&mut self) {
                    // unsafe: dmar bit accessed by Rx part only
                    unsafe { &*$USARTX::ptr() }.cr3.modify(|_, w| w.dmar().set_bit());
                }

                /// Disables the Rx DMA stream.
                pub fn disable_dma_rx(&mut self) {
                    // unsafe: dmar bit accessed by Rx part only
                    unsafe { &*$USARTX::ptr() }.cr3.modify(|_, w| w.dmar().clear_bit());
                }

                /// Return true if the line idle status is set
                ///
                /// The line idle status bit is set when the peripheral detects the receive line is idle.
                /// The bit is cleared by software, by calling `clear_idle()`.
                pub fn is_idle(&self) -> bool {
                    unsafe { (*$USARTX::ptr()).isr.read().idle().bit_is_set() }
                }

                /// Clear the line idle status bit
                pub fn clear_idle(&mut self) {
                    let usart = unsafe { &*$USARTX::ptr() };
                    usart.icr.write(|w| w.idlecf().set_bit());
                    let _ = usart.isr.read();
                    let _ = usart.isr.read(); // Delay 2 peripheral clocks
                }

                /// Return true if the line busy status is set
                ///
                /// The busy status bit is set when there is communication active on the receive line,
                /// and reset at the end of reception.
                pub fn is_busy(&self) -> bool {
                    unsafe { (*$USARTX::ptr()).isr.read().busy().bit_is_set() }
                }

                /// Return true if the rx register is not empty (and can be read)
                pub fn is_rxne(&self) -> bool {
                    unsafe { (*$USARTX::ptr()).isr.read().rxne().bit_is_set() }
                }
            }

            impl serial::Write<u8> for Serial<$USARTX> {
                type Error = Never;

                fn flush(&mut self) -> nb::Result<(), Never> {
                    let mut tx: Tx<$USARTX> = Tx {
                        _usart: PhantomData,
                    };
                    tx.flush()
                }

                fn write(&mut self, byte: u8) -> nb::Result<(), Never> {
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
                type Error = Never;

                fn flush(&mut self) -> nb::Result<(), Never> {
                    // NOTE(unsafe) atomic read with no side effects
                    let isr = unsafe { (*$USARTX::ptr()).isr.read() };

                    if isr.tc().bit_is_set() {
                        Ok(())
                    } else {
                        Err(nb::Error::WouldBlock)
                    }
                }

                fn write(&mut self, byte: u8) -> nb::Result<(), Never> {
                    // NOTE(unsafe) atomic read with no side effects
                    let isr = unsafe { (*$USARTX::ptr()).isr.read() };

                    if isr.txe().bit_is_set() {
                        // NOTE(unsafe) atomic write to stateless register
                        // NOTE(write_volatile) 8-bit write that's not
                        // possible through the svd2rust API
                        unsafe {
                            ptr::write_volatile(
                                &(*$USARTX::ptr()).tdr as *const _ as *mut _, byte)
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
                    unsafe { &*$USARTX::ptr() }.cr1.modify(|_, w| w.txeie().enabled());
                }

                /// Stop listening for `Txe` event
                pub fn unlisten(&mut self) {
                    // unsafe: txeie bit accessed by Tx part only
                    let cr1 = &unsafe { &*$USARTX::ptr() }.cr1;
                    cr1.modify(|_, w| w.txeie().disabled());
                    let _ = cr1.read();
                    let _ = cr1.read(); // Delay 2 peripheral clocks
                }

                /// Enables the Tx DMA stream.
                pub fn enable_dma_tx(&mut self) {
                    // unsafe: dmat bit accessed by Tx part only
                    unsafe { &*$USARTX::ptr() }.cr3.modify(|_, w| w.dmat().set_bit());
                }

                /// Disables the Tx DMA stream.
                pub fn disable_dma_tx(&mut self) {
                    // unsafe: dmat bit accessed by Tx part only
                    unsafe { &*$USARTX::ptr() }.cr3.modify(|_, w| w.dmat().clear_bit());
                }

                /// Return true if the tx register is empty (and can accept data)
                pub fn is_txe(& self) -> bool {
                    unsafe { (*$USARTX::ptr()).isr.read().txe().bit_is_set() }
                }
            }
        )+
    }
}

macro_rules! usart_sel {
	($ccip:ident, $SEL:ident, $sel:ident, $PCLK:ident, $pclk:ident;
     $($USARTX:ident: $doc:expr,)+) => {
	    $(
            impl Serial<$USARTX> {
                /// Returns the frequency of the current kernel clock for
                #[doc=$doc]
                fn kernel_clk(clocks: &CoreClocks) -> Option<Hertz> {
                    // unsafe: read only
                    let ccip = unsafe { (*stm32::RCC::ptr()).$ccip.read() };

                    match ccip.$sel().variant() {
                        Val($SEL::$PCLK) => Some(clocks.$pclk()),
                        Val($SEL::PLL2_Q) => clocks.pll2_q_ck(),
                        Val($SEL::PLL3_Q) => clocks.pll3_q_ck(),
                        Val($SEL::HSI_KER) => clocks.hsi_ck(),
                        Val($SEL::CSI_KER) => clocks.csi_ck(),
                        Val($SEL::LSE) => unimplemented!(),
                        _ => unreachable!(),
                    }
                }
            }
        )+
    }
}

usart! {
    USART1: (usart1, Usart1, pclk2),
    USART2: (usart2, Usart2, pclk1),
    USART3: (usart3, Usart3, pclk1),
    USART6: (usart6, Usart6, pclk2),

    UART4: (uart4, Uart4, pclk1),
    UART5: (uart5, Uart5, pclk1),
    UART7: (uart7, Uart7, pclk1),
    UART8: (uart8, Uart8, pclk1),
}

#[cfg(not(feature = "rm0455"))]
usart_sel! {
    d2ccip2r, USART16SEL_A, usart16sel, RCC_PCLK2, pclk2;

    USART1: "USART1",
    USART6: "USART6",
}
#[cfg(feature = "rm0455")]
usart_sel! {
    cdccip2r, USART16910SEL_A, usart16910sel, RCC_PCLK2, pclk2;

    USART1: "USART1",
    USART6: "USART6",
}

#[cfg(not(feature = "rm0455"))]
usart_sel! {
    d2ccip2r, USART234578SEL_A, usart234578sel, RCC_PCLK1, pclk1;

    USART2: "USART2",
    USART3: "USART3",

    UART4: "UART4",
    UART5: "UART5",
    UART8: "UART8",
    UART7: "UART7",
}
#[cfg(feature = "rm0455")]
usart_sel! {
    cdccip2r, USART234578SEL_A, usart234578sel, RCC_PCLK1, pclk1;

    USART2: "USART2",
    USART3: "USART3",

    UART4: "UART4",
    UART5: "UART5",
    UART8: "UART8",
    UART7: "UART7",
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
