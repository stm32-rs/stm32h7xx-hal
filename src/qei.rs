//! # Quadrature Encoder Interface
use crate::hal::{self, Direction};
use crate::rcc::{rec, ResetEnable};

use crate::gpio::{self, Alternate};

use crate::stm32::{TIM1, TIM8};

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
    ($($TIMX:ty:
       CH1: [$($( #[ $pmeta1:meta ] )* $CH1:ty),*]
       CH2: [$($( #[ $pmeta2:meta ] )* $CH2:ty),*])+) => {
        $(
            $(
                $( #[ $pmeta1 ] )*
                impl PinCh1<$TIMX> for $CH1 {}
            )*
            $(
                $( #[ $pmeta2 ] )*
                impl PinCh2<$TIMX> for $CH2 {}
            )*
        )+
    }
}

pins! {
    TIM1:
        CH1: [
            gpio::PA7<Alternate<1>>,
            gpio::PA8<Alternate<1>>,
            gpio::PB13<Alternate<1>>,
            gpio::PE8<Alternate<1>>,
            gpio::PE9<Alternate<1>>,
            #[cfg(not(feature = "stm32h7b0"))]
            gpio::PK0<Alternate<1>>,
            #[cfg(not(feature = "stm32h7b0"))]
            gpio::PK1<Alternate<1>>
        ]
        CH2: [
            gpio::PA9<Alternate<1>>,
            gpio::PB0<Alternate<1>>,
            gpio::PB14<Alternate<1>>,
            gpio::PE10<Alternate<1>>,
            gpio::PE11<Alternate<1>>,
            #[cfg(not(feature = "stm32h7b0"))]
            gpio::PJ10<Alternate<1>>,
            #[cfg(not(feature = "stm32h7b0"))]
            gpio::PJ11<Alternate<1>>
        ]

    TIM2:
        CH1: [
            gpio::PA0<Alternate<1>>,
            gpio::PA5<Alternate<1>>,
            gpio::PA15<Alternate<1>>
        ]
        CH2: [
            gpio::PA1<Alternate<1>>,
            gpio::PB3<Alternate<1>>
        ]

    TIM3:
        CH1: [
            gpio::PA6<Alternate<2>>,
            gpio::PB4<Alternate<2>>,
            gpio::PC6<Alternate<2>>
        ]
        CH2: [
            gpio::PA7<Alternate<2>>,
            gpio::PB5<Alternate<2>>,
            gpio::PC7<Alternate<2>>
        ]

    TIM4:
        CH1: [
            gpio::PB6<Alternate<2>>,
            gpio::PD12<Alternate<2>>
        ]
        CH2: [
            gpio::PB7<Alternate<2>>,
            gpio::PD13<Alternate<2>>
        ]

    TIM5:
        CH1: [
            gpio::PA0<Alternate<2>>,
            gpio::PH10<Alternate<2>>
        ]
        CH2: [
            gpio::PA1<Alternate<2>>,
            gpio::PH11<Alternate<2>>
        ]

    TIM8:
        CH1: [
            gpio::PA5<Alternate<3>>,
            gpio::PA7<Alternate<3>>,
            gpio::PC6<Alternate<3>>,
            gpio::PH13<Alternate<3>>,
            #[cfg(not(feature = "rm0468"))]
            gpio::PI5<Alternate<3>>,
            #[cfg(not(feature = "stm32h7b0"))]
            gpio::PJ8<Alternate<3>>,
            #[cfg(not(feature = "stm32h7b0"))]
            gpio::PJ9<Alternate<3>>
        ]
        CH2: [
            gpio::PB0<Alternate<3>>,
            gpio::PB14<Alternate<3>>,
            gpio::PC7<Alternate<3>>,
            gpio::PH14<Alternate<3>>,
            #[cfg(not(feature = "rm0468"))]
            gpio::PI6<Alternate<3>>,
            #[cfg(not(any(feature = "stm32h7b0", feature = "rm0468")))]
            gpio::PJ6<Alternate<3>>,
            #[cfg(not(feature = "stm32h7b0"))]
            gpio::PJ7<Alternate<3>>,
            #[cfg(not(feature = "stm32h7b0"))]
            gpio::PJ10<Alternate<3>>,
            #[cfg(not(feature = "stm32h7b0"))]
            gpio::PJ11<Alternate<3>>
        ]

}

/// Hardware quadrature encoder interface peripheral
pub struct Qei<TIM> {
    tim: TIM,
}

pub trait QeiExt<TIM> {
    type Rec: ResetEnable;

    fn qei<PINS>(self, _pins: PINS, prec: Self::Rec) -> Qei<TIM>
    where
        PINS: Pins<TIM>;

    fn qei_unchecked(self, prec: Self::Rec) -> Qei<TIM>;
}

macro_rules! tim_hal {
    ($($TIM:ident: ($tim:ident, $Rec:ident, $bits:ident),)+) => {
        $(
            impl Qei<$TIM> {
                /// Configures a TIM peripheral as a quadrature
                /// encoder interface input
                pub fn $tim(tim: $TIM, prec: rec::$Rec) -> Self
                {
                    // enable and reset peripheral to a clean slate
                    prec.enable().reset();

                    // Configure TxC1 and TxC2 as captures
                    tim.ccmr1_output().write(|w| unsafe {
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

                    Qei { tim }
                }

                /// Releases the TIM peripheral
                pub fn release(self) -> $TIM {
                    self.tim
                }
            }

            impl QeiExt<$TIM> for $TIM {
                type Rec = rec::$Rec;

                fn qei<PINS>(self, _pins: PINS, prec: Self::Rec) -> Qei<$TIM> {
                    Qei::$tim(self, prec)
                }

                fn qei_unchecked(self, prec: Self::Rec) -> Qei<$TIM> {
                    Qei::$tim(self, prec)
                }
            }

            impl hal::Qei for Qei<$TIM> {
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

tim_hal! {
    TIM1: (tim1, Tim1, u16),
    TIM8: (tim8, Tim8, u16),
    TIM2: (tim2, Tim2, u32),
    TIM3: (tim3, Tim3, u16),
    TIM4: (tim4, Tim4, u16),
    TIM5: (tim5, Tim5, u32),
}
