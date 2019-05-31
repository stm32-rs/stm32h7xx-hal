//! Serial

use core::fmt;
use core::marker::PhantomData;
use core::ptr;

use embedded_hal::prelude::*;
use embedded_hal::serial;
use nb::block;

use crate::stm32::rcc::d2ccip2r;
use crate::stm32::usart1::cr1::{M0W, PCEW, PSW};

#[cfg(any(
    feature = "stm32h742",
    feature = "stm32h743",
    feature = "stm32h753",
    feature = "stm32h750"
))]
use crate::stm32::{USART1, USART2, USART3, USART6};

#[cfg(any(
    feature = "stm32h742",
    feature = "stm32h743",
    feature = "stm32h753",
    feature = "stm32h750"
))]
use crate::stm32::{UART4, UART5, UART7, UART8};

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
use crate::gpio::gpioj::{PJ8, PJ9};

use crate::gpio::{Alternate, AF11, AF14, AF4, AF6, AF7, AF8};
use crate::rcc::Ccdr;
use crate::time::Hertz;

use crate::Never;

/// Serial error
#[derive(Debug)]
pub enum Error {
    /// Framing error
    Framing,
    /// Noise error
    Noise,
    /// RX buffer overrun
    Overrun,
    /// Parity check error
    Parity,
    #[doc(hidden)]
    _Extensible,
}

/// Interrupt event
pub enum Event {
    /// New data has been received
    Rxne,
    /// New data can be sent
    Txe,
    /// Idle line state detected
    Idle,
}

pub mod config {
    use crate::time::Bps;
    use crate::time::U32Ext;

    pub enum WordLength {
        DataBits8,
        DataBits9,
    }

    pub enum Parity {
        ParityNone,
        ParityEven,
        ParityOdd,
    }

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
        pub baudrate: Bps,
        pub wordlength: WordLength,
        pub parity: Parity,
        pub stopbits: StopBits,
    }

    impl Config {
        pub fn baudrate(mut self, baudrate: Bps) -> Self {
            self.baudrate = baudrate;
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
            let baudrate = 19_200_u32.bps();
            Config {
                baudrate,
                wordlength: WordLength::DataBits8,
                parity: Parity::ParityNone,
                stopbits: StopBits::STOP1,
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
    ($($UARTX:ty: TX: [$($TX:ty),*] RX: [$($RX:ty),*])+) => {
        $(
            $(
                impl PinTx<$UARTX> for $TX {}
            )*
            $(
                impl PinRx<$UARTX> for $RX {}
            )*
        )+
    }
}

#[cfg(any(
    feature = "stm32h742",
    feature = "stm32h743",
    feature = "stm32h753",
    feature = "stm32h750"
))]
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
#[cfg(any(
    feature = "stm32h742",
    feature = "stm32h743",
    feature = "stm32h753",
    feature = "stm32h750"
))]
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
            PJ8<Alternate<AF8>>
        ]
        RX: [
            NoRx,
            PE0<Alternate<AF8>>,
            PJ9<Alternate<AF8>>
        ]
}

/// Serial abstraction
pub struct Serial<USART, PINS> {
    usart: USART,
    pins: PINS,
}

/// Serial receiver
pub struct Rx<USART> {
    _usart: PhantomData<USART>,
}

/// Serial transmitter
pub struct Tx<USART> {
    _usart: PhantomData<USART>,
}

pub trait SerialExt<USART> {
    fn usart<PINS>(
        self,
        pins: PINS,
        config: config::Config,
        ccdr: &mut Ccdr,
    ) -> Result<Serial<USART, PINS>, config::InvalidConfig>
    where
        PINS: Pins<USART>;
}

macro_rules! usart {
    ($(
        $USARTX:ident: ($usartX:ident, $apb:ident, $usartXen:ident, $usartXrst:ident,
                        $pclkX:ident, $enr:ident, $rstr:ident),
    )+) => {
        $(
            /// Configures a USART peripheral to provide serial
            /// communication
            impl<PINS> Serial<$USARTX, PINS> {
                pub fn $usartX(
                    usart: $USARTX,
                    pins: PINS,
                    config: config::Config,
                    ccdr: &mut Ccdr,
                ) -> Result<Self, config::InvalidConfig>
                where
                    PINS: Pins<$USARTX>,
                {
                    use crate::stm32::usart1::cr2::STOPW;
                    use self::config::*;

                    // Enable clock for USART and reset
                    ccdr.$apb.$enr().modify(|_, w| w.$usartXen().enabled());
                    ccdr.$apb.$rstr().modify(|_, w| w.$usartXrst().set_bit());
                    ccdr.$apb.$rstr().modify(|_, w| w.$usartXrst().clear_bit());

                    // Get kernel clock
	                let usart_ker_ck = match Self::kernel_clk(ccdr) {
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
                            StopBits::STOP0P5 => STOPW::STOP0P5,
                            StopBits::STOP1 => STOPW::STOP1,
                            StopBits::STOP1P5 => STOPW::STOP1P5,
                            StopBits::STOP2 => STOPW::STOP2,
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
                                WordLength::DataBits8 => M0W::BIT8,
                                WordLength::DataBits9 => M0W::BIT9,
                            }).pce()
                            .variant(match config.parity {
                                Parity::ParityNone => PCEW::DISABLED,
                                _ => PCEW::ENABLED,
                            }).ps()
                            .variant(match config.parity {
                                Parity::ParityOdd => PSW::EVEN,
                                _ => PSW::ODD,
                            })
                    });

                    Ok(Serial { usart, pins })
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
                }

                /// Return true if the line idle status is set
                pub fn is_idle(& self) -> bool {
                    unsafe { (*$USARTX::ptr()).isr.read().idle().bit_is_set() }
                }

                /// Return true if the tx register is empty (and can accept data)
                pub fn is_txe(& self) -> bool {
                    unsafe { (*$USARTX::ptr()).isr.read().txe().bit_is_set() }
                }

                /// Return true if the rx register is not empty (and can be read)
                pub fn is_rxne(& self) -> bool {
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
                /// Releases the USART peripheral and associated pins
                pub fn release(self) -> ($USARTX, PINS) {
                    // Wait until both TXFIFO and shift register are empty
                    while self.usart.isr.read().tc().bit_is_clear() {}

                    (self.usart, self.pins)
                }
            }

            impl SerialExt<$USARTX> for $USARTX {
                fn usart<PINS>(self,
                               pins: PINS,
                               config: config::Config,
                               ccdr: &mut Ccdr) -> Result<Serial<$USARTX, PINS>, config::InvalidConfig>
                where
                    PINS: Pins<$USARTX>
                {
                    Serial::$usartX(self, pins, config, ccdr)
                }
            }

            impl<PINS> serial::Read<u8> for Serial<$USARTX, PINS> {
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

                    // TODO clear errors in ICR

                    Err(if isr.pe().bit_is_set() {
                        nb::Error::Other(Error::Parity)
                    } else if isr.fe().bit_is_set() {
                        nb::Error::Other(Error::Framing)
                    } else if isr.nf().bit_is_set() {
                        nb::Error::Other(Error::Noise)
                    } else if isr.ore().bit_is_set() {
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

            impl<PINS> serial::Write<u8> for Serial<$USARTX, PINS> {
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
        )+
    }
}

macro_rules! usart16sel {
	($($USARTX:ident,)+) => {
	    $(
            impl<PINS> Serial<$USARTX, PINS> {
                /// Returns the frequency of the current kernel clock
                /// for USART1 and 6
                fn kernel_clk(ccdr: &Ccdr) -> Option<Hertz> {
                    match ccdr.rb.d2ccip2r.read().usart16sel() {
                        d2ccip2r::USART16SELR::RCC_PCLK2 => Some(ccdr.clocks.pclk2()),
                        d2ccip2r::USART16SELR::PLL2_Q => ccdr.clocks.pll2_q_ck(),
                        d2ccip2r::USART16SELR::PLL3_Q => ccdr.clocks.pll3_q_ck(),
                        d2ccip2r::USART16SELR::HSI_KER => ccdr.clocks.hsi_ck(),
                        d2ccip2r::USART16SELR::CSI_KER => ccdr.clocks.csi_ck(),
                        d2ccip2r::USART16SELR::LSE => unimplemented!(),
                        _ => unreachable!(),
                    }
                }
            }
        )+
    }
}
macro_rules! usart234578sel {
	($($USARTX:ident,)+) => {
	    $(
            impl<PINS> Serial<$USARTX, PINS> {
                /// Returns the frequency of the current kernel clock
                /// for USART2/3, UART4/5/7/8
                fn kernel_clk(ccdr: &Ccdr) -> Option<Hertz> {
                    match ccdr.rb.d2ccip2r.read().usart234578sel() {
                        d2ccip2r::USART234578SELR::RCC_PCLK1 => Some(ccdr.clocks.pclk1()),
                        d2ccip2r::USART234578SELR::PLL2_Q => ccdr.clocks.pll2_q_ck(),
                        d2ccip2r::USART234578SELR::PLL3_Q => ccdr.clocks.pll3_q_ck(),
                        d2ccip2r::USART234578SELR::HSI_KER => ccdr.clocks.hsi_ck(),
                        d2ccip2r::USART234578SELR::CSI_KER => ccdr.clocks.csi_ck(),
                        d2ccip2r::USART234578SELR::LSE => unimplemented!(),
                        _ => unreachable!(),
                    }
                }
            }
        )+
    }
}

usart! {
    USART1: (usart1, apb2, usart1en, usart1rst, pclk2, enr, rstr),
    USART2: (usart2, apb1, usart2en, usart2rst, pclk1, lenr, lrstr),
    USART3: (usart3, apb1, usart3en, usart3rst, pclk1, lenr, lrstr),
    USART6: (usart6, apb2, usart6en, usart6rst, pclk2, enr, rstr),

    UART4: (uart4, apb1, uart4en, uart4rst, pclk1, lenr, lrstr),
    UART5: (uart5, apb1, uart5en, uart5rst, pclk1, lenr, lrstr),
    UART7: (uart7, apb1, uart7en, uart7rst, pclk1, lenr, lrstr),
    UART8: (uart8, apb1, uart8en, uart8rst, pclk1, lenr, lrstr),
}

#[cfg(any(
    feature = "stm32h742",
    feature = "stm32h743",
    feature = "stm32h753",
    feature = "stm32h750"
))]
usart16sel! {
    USART1, USART6,
}
#[cfg(any(
    feature = "stm32h742",
    feature = "stm32h743",
    feature = "stm32h753",
    feature = "stm32h750"
))]
usart234578sel! {
    USART2, USART3, UART4, UART5, UART7, UART8,
}

impl<USART> fmt::Write for Tx<USART>
where
    Tx<USART>: serial::Write<u8>,
{
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let _ =
            s.as_bytes().iter().map(|c| block!(self.write(*c))).last();
        Ok(())
    }
}
