//! # Quadrature Encoder Interface
use crate::hal::{self, Direction};
use crate::pac;
use crate::pwm::{PinCh, C1, C2};
use crate::rcc::{rec, ResetEnable};

/// Hardware quadrature encoder interface peripheral
pub struct Qei<TIM> {
    tim: TIM,
}

pub trait QeiExt: Sized + PinCh<C1> + PinCh<C2> {
    type Rec: ResetEnable;

    fn qei(
        self,
        pins: (
            impl Into<<Self as PinCh<C1>>::Ch>,
            impl Into<<Self as PinCh<C2>>::Ch>,
        ),
        prec: Self::Rec,
    ) -> Qei<Self> {
        let _pins: (<Self as PinCh<C1>>::Ch, <Self as PinCh<C2>>::Ch) =
            (pins.0.into(), pins.1.into());
        Self::qei_unchecked(self, prec)
    }

    fn qei_unchecked(self, prec: Self::Rec) -> Qei<Self>;
}

macro_rules! tim_hal {
    ($($TIM:ty: ($tim:ident, $Rec:ident, $bits:ident),)+) => {
        $(
            impl Qei<$TIM> {
                /// Configures a TIM peripheral as a quadrature
                /// encoder interface input
                pub fn $tim(tim: $TIM, prec: rec::$Rec) -> Self
                {
                    // enable and reset peripheral to a clean slate
                    let _ = prec.enable().reset(); // drop

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

                    #[allow(unused_unsafe)] // method is safe for some timers
                    tim.arr.write(|w| unsafe { w.bits(core::u32::MAX) });
                    tim.cr1.write(|w| w.cen().set_bit());

                    Qei { tim }
                }

                /// Releases the TIM peripheral
                pub fn release(self) -> ($TIM, rec::$Rec) {
                    (self.tim, rec::$Rec { _marker: core::marker::PhantomData })
                }
            }

            impl QeiExt for $TIM {
                type Rec = rec::$Rec;

                fn qei_unchecked(self, prec: Self::Rec) -> Qei<Self> {
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
    pac::TIM1: (tim1, Tim1, u16),
    pac::TIM8: (tim8, Tim8, u16),
    pac::TIM2: (tim2, Tim2, u32),
    pac::TIM3: (tim3, Tim3, u16),
    pac::TIM4: (tim4, Tim4, u16),
    pac::TIM5: (tim5, Tim5, u32),
}
