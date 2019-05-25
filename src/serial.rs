//! Serial

use core::marker::PhantomData;
use core::ptr;

use crate::hal::serial;
use crate::stm32::{USART1, USART2, USART3};
use nb;
use void::Void;

use crate::gpio::gpioa::{PA10, PA2, PA3, PA9};
use crate::gpio::gpiob::{PB10, PB6, PB7};
use crate::gpio::gpioc::{PC10, PC11, PC4, PC5};
use crate::gpio::gpiod::{PD8, PD9};

use crate::gpio::AF7;
use crate::rcc::{Clocks, APB1};
use crate::time::Bps;

/// Interrupt event
pub enum Event {
    /// New data has been received
    Rxne,
    /// New data can be sent
    Txe,
}

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

// FIXME these should be "closed" traits
/// TX pin - DO NOT IMPLEMENT THIS TRAIT
pub unsafe trait TxPin<USART> {}

/// RX pin - DO NOT IMPLEMENT THIS TRAIT
pub unsafe trait RxPin<USART> {}

unsafe impl TxPin<USART1> for PA9<AF7> {}
unsafe impl TxPin<USART1> for PB6<AF7> {}
unsafe impl TxPin<USART1> for PC4<AF7> {}

unsafe impl RxPin<USART1> for PA10<AF7> {}
unsafe impl RxPin<USART1> for PB7<AF7> {}
unsafe impl RxPin<USART1> for PC5<AF7> {}

unsafe impl TxPin<USART2> for PA2<AF7> {}

unsafe impl RxPin<USART2> for PA3<AF7> {}
unsafe impl TxPin<USART3> for PB10<AF7> {}
unsafe impl TxPin<USART3> for PC10<AF7> {}

unsafe impl RxPin<USART3> for PC11<AF7> {}

unsafe impl TxPin<USART3> for PD8<AF7> {}
unsafe impl RxPin<USART3> for PD9<AF7> {}

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

macro_rules! hal {
    ($(
        $USARTX:ident: ($usartX:ident, $APB:ident, $usartXen:ident, $usartXrst:ident, $pclkX:ident, $enr:ident, $rstr:ident),
    )+) => {
        $(
            impl<TX, RX> Serial<$USARTX, (TX, RX)> {
                /// Configures a USART peripheral to provide serial communication
                pub fn $usartX(
                    usart: $USARTX,
                    pins: (TX, RX),
                    baud_rate: Bps,
                    clocks: Clocks,
                    apb: &mut $APB,
                ) -> Self
                where
                    TX: TxPin<$USARTX>,
                    RX: RxPin<$USARTX>,
                {
                    // enable or reset $USARTX
                    apb.$enr().modify(|_, w| w.$usartXen().set_bit());
                    apb.$rstr().modify(|_, w| w.$usartXrst().set_bit());
                    apb.$rstr().modify(|_, w| w.$usartXrst().clear_bit());

                    // disable hardware flow control
                    // TODO enable DMA
                    // usart.cr3.write(|w| w.rtse().clear_bit().ctse().clear_bit());

                    let brr = clocks.$pclkX().0 / baud_rate.0;
                    assert!(brr >= 16, "impossible baud rate");
                    usart.brr.write(|w| unsafe { w.bits(brr) });

                    // UE: enable USART
                    // RE: enable receiver
                    // TE: enable transceiver
                    usart
                        .cr1
                        .write(|w| w.ue().set_bit().re().set_bit().te().set_bit());

                    Serial { usart, pins }
                }

                /// Starts listening for an interrupt event
                pub fn listen(&mut self, event: Event) {
                    match event {
                        Event::Rxne => {
                            self.usart.cr1.modify(|_, w| w.rxneie().set_bit())
                        },
                        Event::Txe => {
                            self.usart.cr1.modify(|_, w| w.txeie().set_bit())
                        },
                    }
                }

                /// Starts listening for an interrupt event
                pub fn unlisten(&mut self, event: Event) {
                    match event {
                        Event::Rxne => {
                            self.usart.cr1.modify(|_, w| w.rxneie().clear_bit())
                        },
                        Event::Txe => {
                            self.usart.cr1.modify(|_, w| w.txeie().clear_bit())
                        },
                    }
                }

                /// Splits the `Serial` abstraction into a transmitter and a receiver half
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
                pub fn free(self) -> ($USARTX, (TX, RX)) {
                    (self.usart, self.pins)
                }
            }

            impl serial::Read<u8> for Rx<$USARTX> {
                type Error = Error;

                fn read(&mut self) -> nb::Result<u8, Error> {
                    // NOTE(unsafe) atomic read with no side effects
                    let isr = unsafe { (*$USARTX::ptr()).isr.read() };

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

            impl serial::Write<u8> for Tx<$USARTX> {
                // NOTE(Void) See section "29.7 USART interrupts"; the only possible errors during
                // transmission are: clear to send (which is disabled in this case) errors and
                // framing errors (which only occur in SmartCard mode); neither of these apply to
                // our hardware configuration
                type Error = Void;

                fn flush(&mut self) -> nb::Result<(), Void> {
                    // NOTE(unsafe) atomic read with no side effects
                    let isr = unsafe { (*$USARTX::ptr()).isr.read() };

                    if isr.tc().bit_is_set() {
                        Ok(())
                    } else {
                        Err(nb::Error::WouldBlock)
                    }
                }

                fn write(&mut self, byte: u8) -> nb::Result<(), Void> {
                    // NOTE(unsafe) atomic read with no side effects
                    let isr = unsafe { (*$USARTX::ptr()).isr.read() };

                    if isr.txe().bit_is_set() {
                        // NOTE(unsafe) atomic write to stateless register
                        // NOTE(write_volatile) 8-bit write that's not possible through the svd2rust API
                        unsafe {
                            ptr::write_volatile(&(*$USARTX::ptr()).tdr as *const _ as *mut _, byte)
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

#[cfg(any(
    feature = "stm32h7x3",
))]
hal! {
    USART2: (usart2, APB1, usart2en, usart2rst, pclk1, lenr, lrstr),
    USART3: (usart3, APB1, usart3en, usart3rst, pclk1, lenr, lrstr),
}
