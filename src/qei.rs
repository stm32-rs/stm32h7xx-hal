//! # Quadrature Encoder Interface
use crate::hal::{self, Direction};
use crate::rcc::Ccdr;

#[cfg(any(
    feature = "stm32h742",
    feature = "stm32h743",
    feature = "stm32h753",
    feature = "stm32h750"
))]
use crate::gpio::gpioa::{PA0, PA1, PA15, PA5, PA6, PA7, PA8, PA9};
use crate::gpio::gpiob::{PB0, PB13, PB14, PB3, PB4, PB5, PB6, PB7};
use crate::gpio::gpioc::{PC6, PC7};
use crate::gpio::gpiod::{PD12, PD13};
use crate::gpio::gpioe::{PE10, PE11, PE8, PE9};
use crate::gpio::gpioh::{PH10, PH11, PH13, PH14};
use crate::gpio::gpioi::{PI5, PI6};
use crate::gpio::gpioj::{PJ10, PJ11, PJ6, PJ7, PJ8, PJ9};
use crate::gpio::gpiok::{PK0, PK1};

use crate::gpio::Alternate;

#[cfg(any(
    feature = "stm32h742",
    feature = "stm32h743",
    feature = "stm32h753",
    feature = "stm32h750"
))]
use crate::gpio::AF1;
use crate::gpio::AF2;
use crate::gpio::AF3;

#[cfg(any(
    feature = "stm32h742",
    feature = "stm32h743",
    feature = "stm32h753",
    feature = "stm32h750"
))]
use crate::stm32::{TIM1, TIM8};

#[cfg(any(
    feature = "stm32h742",
    feature = "stm32h743",
    feature = "stm32h753",
    feature = "stm32h750"
))]
use crate::stm32::{TIM2, TIM3, TIM4, TIM5};

pub trait Pins<TIM> {}
pub trait PinCh1<TIM> {}
pub trait PinCh2<TIM> {}

impl<TIM, PCH1, PCH2> Pins<TIM> for (PCH1, PCH2)
where
    PCH1: PinCh1<TIM>,
    PCH2: PinCh2<TIM>,
{
}

macro_rules! pins {
    ($($TIMX:ty: CH1: [$($CH1:ty),*] CH2: [$($CH2:ty),*])+) => {
        $(
            $(
                impl PinCh1<$TIMX> for $CH1 {}
            )*
            $(
                impl PinCh2<$TIMX> for $CH2 {}
            )*
        )+
    }
}

#[cfg(any(
    feature = "stm32h742",
    feature = "stm32h743",
    feature = "stm32h753",
    feature = "stm32h750",
))]
pins! {
    TIM1:
        CH1: [
            PA7<Alternate<AF1>>,
            PA8<Alternate<AF1>>,
            PB13<Alternate<AF1>>,
            PE8<Alternate<AF1>>,
            PE9<Alternate<AF1>>,
            PK0<Alternate<AF1>>,
            PK1<Alternate<AF1>>
        ]
        CH2: [
            PA9<Alternate<AF1>>,
            PB0<Alternate<AF1>>,
            PB14<Alternate<AF1>>,
            PE10<Alternate<AF1>>,
            PE11<Alternate<AF1>>,
            PJ10<Alternate<AF1>>,
            PJ11<Alternate<AF1>>
        ]

    TIM2:
        CH1: [
            PA0<Alternate<AF1>>,
            PA5<Alternate<AF1>>,
            PA15<Alternate<AF1>>
        ]
        CH2: [
            PA1<Alternate<AF1>>,
            PB3<Alternate<AF1>>
        ]

    TIM3:
        CH1: [
            PA6<Alternate<AF2>>,
            PB4<Alternate<AF2>>,
            PC6<Alternate<AF2>>
        ]
        CH2: [
            PA7<Alternate<AF2>>,
            PB5<Alternate<AF2>>,
            PC7<Alternate<AF2>>
        ]

    TIM4:
        CH1: [
            PB6<Alternate<AF2>>,
            PD12<Alternate<AF2>>
        ]
        CH2: [
            PB7<Alternate<AF2>>,
            PD13<Alternate<AF2>>
        ]

    TIM5:
        CH1: [
            PA0<Alternate<AF2>>,
            PH10<Alternate<AF2>>
        ]
        CH2: [
            PA1<Alternate<AF2>>,
            PH11<Alternate<AF2>>
        ]

    TIM8:
        CH1: [
            PA5<Alternate<AF3>>,
            PA7<Alternate<AF3>>,
            PC6<Alternate<AF3>>,
            PH13<Alternate<AF3>>,
            PI5<Alternate<AF3>>,
            PJ8<Alternate<AF3>>,
            PJ9<Alternate<AF3>>
        ]
        CH2: [
            PB0<Alternate<AF3>>,
            PB14<Alternate<AF3>>,
            PC7<Alternate<AF3>>,
            PH14<Alternate<AF3>>,
            PI6<Alternate<AF3>>,
            PJ6<Alternate<AF3>>,
            PJ7<Alternate<AF3>>,
            PJ10<Alternate<AF3>>,
            PJ11<Alternate<AF3>>
        ]

}

/// Hardware quadrature encoder interface peripheral
pub struct Qei<TIM, PINS> {
    tim: TIM,
    pins: PINS,
}

macro_rules! tim_hal {
    ($($TIM:ident: ($tim:ident, $apb:ident, $timXen:ident, $timXrst:ident, $bits:ident),)+) => {
        $(
            impl<PINS> Qei<$TIM, PINS> {
                /// Configures a TIM peripheral as a quadrature
                /// encoder interface input
                pub fn $tim(tim: $TIM,
                            pins: PINS,
                            ccdr: &mut Ccdr,
                ) -> Self
                where
                    PINS: Pins<$TIM>
                {
                    // enable and reset peripheral to a clean slate
                    // state
                    ccdr.$apb.enr().modify(|_, w| w.$timXen().set_bit());
                    ccdr.$apb.rstr().modify(|_, w| w.$timXrst().set_bit());
                    ccdr.$apb.rstr().modify(|_, w| w.$timXrst().clear_bit());


                    // Configure TxC1 and TxC2 as captures
                    tim.ccmr1_output.write(|w| unsafe {
                        w.cc1s()
                            .bits(0b01)
                            .cc2s()
                            .bits(0b01)
                    });

                    // enable and configure to capture on rising edge
                    tim.ccer.write(|w| {
                        w.cc1e()
                            .set_bit()
                            .cc1p()
                            .clear_bit()
                            .cc2e()
                            .set_bit()
                            .cc2p()
                            .clear_bit()
                    });

                    // configure as quadrature encoder
                    tim.smcr.write(|w| { w.sms().bits(3) });

                    tim.arr.write(|w| unsafe { w.bits(core::u32::MAX) });
                    tim.cr1.write(|w| w.cen().set_bit());

                    Qei { tim, pins }
                }

                /// Releases the TIM peripheral and QEI pins
                pub fn release(self) -> ($TIM, PINS) {
                    (self.tim, self.pins)
                }
            }

            impl<PINS> hal::Qei for Qei<$TIM, PINS> {
                type Count = $bits;

                fn count(&self) -> $bits {
                    self.tim.cnt.read().bits() as $bits
                }

                fn direction(&self) -> Direction {
                    if self.tim.cr1.read().dir().bit_is_clear() {
                        hal::Direction::Upcounting
                    } else {
                        hal::Direction::Downcounting
                    }
                }
            }

        )+
    }
}

#[cfg(any(
    feature = "stm32h742",
    feature = "stm32h743",
    feature = "stm32h753",
    feature = "stm32h750"
))]
tim_hal! {
    TIM1: (tim1, apb2, tim1en, tim1rst, u16),
    TIM8: (tim8, apb2, tim8en, tim8rst, u16),
    TIM2: (tim2, apb1l, tim2en, tim2rst, u32),
    TIM3: (tim3, apb1l, tim3en, tim3rst, u16),
    TIM4: (tim4, apb1l, tim4en, tim4rst, u16),
    TIM5: (tim5, apb1l, tim5en, tim5rst, u32),
}
