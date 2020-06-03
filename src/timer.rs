//! Timers

// TODO: on the h7x3 at least, only TIM2, TIM3, TIM4, TIM5 can support 32 bits.
// TIM1 is 16 bit.

use core::marker::PhantomData;

use crate::hal::timer::{CountDown, Periodic};
use crate::stm32::{LPTIM1, LPTIM2, LPTIM3, LPTIM4, LPTIM5};
use crate::stm32::{
    TIM1, TIM12, TIM13, TIM14, TIM15, TIM16, TIM17, TIM2, TIM3, TIM4, TIM5,
    TIM6, TIM7, TIM8,
};

use cast::{u16, u32};
use nb;
use void::Void;

use crate::rcc::{rec, CoreClocks, ResetEnable};
use crate::stm32;
use crate::stm32::rcc::{d2ccip2r, d3ccipr};
use crate::time::Hertz;
use stm32h7::Variant::Val;

/// Associate clocks with timers
pub trait GetClk {
    fn get_clk(clocks: &CoreClocks) -> Option<Hertz>;
}

/// Timers with CK_INT derived from rcc_tim[xy]_ker_ck
macro_rules! impl_tim_ker_ck {
    ($($ckX:ident: $($TIMX:ident),+)+) => {
        $(
            $(
                impl GetClk for $TIMX {
                    fn get_clk(clocks: &CoreClocks) -> Option<Hertz> {
                        Some(clocks.$ckX())
                    }
                }
            )+
        )+
    }
}
impl_tim_ker_ck! {
    timx_ker_ck: TIM2, TIM3, TIM4, TIM5, TIM6, TIM7, TIM12, TIM13, TIM14
    timy_ker_ck: TIM1, TIM8, TIM15, TIM16, TIM17
}

/// LPTIM1 Kernel Clock
impl GetClk for LPTIM1 {
    /// Current kernel clock
    fn get_clk(clocks: &CoreClocks) -> Option<Hertz> {
        // unsafe: read only
        let d2ccip2r = &unsafe { &*stm32::RCC::ptr() }.d2ccip2r;

        match d2ccip2r.read().lptim1sel().variant() {
            Val(d2ccip2r::LPTIM1SEL_A::RCC_PCLK1) => Some(clocks.pclk1()),
            Val(d2ccip2r::LPTIM1SEL_A::PLL2_P) => clocks.pll2_p_ck(),
            Val(d2ccip2r::LPTIM1SEL_A::PLL3_R) => clocks.pll3_r_ck(),
            Val(d2ccip2r::LPTIM1SEL_A::LSE) => unimplemented!(),
            Val(d2ccip2r::LPTIM1SEL_A::LSI) => unimplemented!(),
            Val(d2ccip2r::LPTIM1SEL_A::PER) => clocks.per_ck(),
            _ => unreachable!(),
        }
    }
}
/// LPTIM2 Kernel Clock
impl GetClk for LPTIM2 {
    /// Current kernel clock
    fn get_clk(clocks: &CoreClocks) -> Option<Hertz> {
        // unsafe: read only
        let d3ccipr = &unsafe { &*stm32::RCC::ptr() }.d3ccipr;

        match d3ccipr.read().lptim2sel().variant() {
            Val(d3ccipr::LPTIM2SEL_A::RCC_PCLK4) => Some(clocks.pclk4()),
            Val(d3ccipr::LPTIM2SEL_A::PLL2_P) => clocks.pll2_p_ck(),
            Val(d3ccipr::LPTIM2SEL_A::PLL3_R) => clocks.pll3_r_ck(),
            Val(d3ccipr::LPTIM2SEL_A::LSE) => unimplemented!(),
            Val(d3ccipr::LPTIM2SEL_A::LSI) => unimplemented!(),
            Val(d3ccipr::LPTIM2SEL_A::PER) => clocks.per_ck(),
            _ => unreachable!(),
        }
    }
}
/// LPTIM345 Kernel Clock
macro_rules! impl_clk_lptim345 {
	($($TIMX:ident),+) => {
	    $(
            impl GetClk for $TIMX {
                /// Current kernel clock
                fn get_clk(clocks: &CoreClocks) -> Option<Hertz> {
                    // unsafe: read only
                    let d3ccipr = &unsafe { &*stm32::RCC::ptr() }.d3ccipr;

                    match d3ccipr.read().lptim345sel().variant() {
                        Val(d3ccipr::LPTIM345SEL_A::RCC_PCLK4) => Some(clocks.pclk4()),
                        Val(d3ccipr::LPTIM345SEL_A::PLL2_P) => clocks.pll2_p_ck(),
                        Val(d3ccipr::LPTIM345SEL_A::PLL3_R) => clocks.pll3_r_ck(),
                        Val(d3ccipr::LPTIM345SEL_A::LSE) => unimplemented!(),
                        Val(d3ccipr::LPTIM345SEL_A::LSI) => unimplemented!(),
                        Val(d3ccipr::LPTIM345SEL_A::PER) => clocks.per_ck(),
                        _ => unreachable!(),
                    }
                }
            }
        )+
    }
}
impl_clk_lptim345! { LPTIM3, LPTIM4, LPTIM5 }

/// External trait for hardware timers
pub trait TimerExt<TIM> {
    type Rec: ResetEnable;

    fn timer<T>(
        self,
        timeout: T,
        prec: Self::Rec,
        clocks: &CoreClocks,
    ) -> Timer<TIM>
    where
        T: Into<Hertz>;
}

/// Hardware timers
pub struct Timer<TIM> {
    clk: u32,
    tim: TIM,
    timeout: Hertz,
}

/// Timer Events
///
/// Each event is a possible interrupt source, if enabled
pub enum Event {
    /// Timer timed out / count down ended
    TimeOut,
}

macro_rules! hal {
    ($($TIMX:ident: ($timX:ident, $Rec:ident),)+) => {
        $(
            impl Periodic for Timer<$TIMX> {}

            impl CountDown for Timer<$TIMX> {
                type Time = Hertz;

                fn start<T>(&mut self, timeout: T)
                where
                    T: Into<Hertz>,
                {
                    // Pause
                    self.pause();

                    // Reset counter
                    self.tim.cnt.reset();

                    // UEV event occours on next overflow
                    self.tim.cr1.modify(|_, w| w.urs().counter_only());
                    self.clear_uif_bit();

                    // Set PSC and ARR
                    self.set_freq(timeout);

                    // Start counter
                    self.resume()
                }

                fn wait(&mut self) -> nb::Result<(), Void> {
                    if self.tim.sr.read().uif().bit_is_clear() {
                        Err(nb::Error::WouldBlock)
                    } else {
                        self.clear_uif_bit();
                        Ok(())
                    }
                }
            }

            impl TimerExt<$TIMX> for $TIMX {
                type Rec = rec::$Rec;

                fn timer<T>(self, timeout: T,
                            prec: Self::Rec, clocks: &CoreClocks
                ) -> Timer<$TIMX>
                    where
                        T: Into<Hertz>,
                {
                    Timer::$timX(self, timeout, prec, clocks)
                }
            }

            impl Timer<$TIMX> {
                /// Configures a TIM peripheral as a periodic count down timer
                pub fn $timX<T>(tim: $TIMX, timeout: T,
                                prec: rec::$Rec, clocks: &CoreClocks
                ) -> Self
                where
                    T: Into<Hertz>,
                {
                    // enable and reset peripheral to a clean slate state
                    prec.enable().reset();

                    let clk = $TIMX::get_clk(clocks)
                        .expect("Timer input clock not running!").0;

                    let mut timer = Timer {
                        clk,
                        tim,
                        timeout: Hertz(0),
                    };
                    timer.start(timeout);

                    timer
                }

                pub fn set_freq<T>(&mut self, timeout: T)
                where
                    T: Into<Hertz>,
                {
                    self.timeout = timeout.into();

                    let clk = self.clk;
                    let frequency = self.timeout.0;
                    let ticks = clk / frequency;

                    let psc = u16((ticks - 1) / (1 << 16)).unwrap();
                    self.tim.psc.write(|w| { w.psc().bits(psc) });

                    let arr = u16(ticks / u32(psc + 1)).unwrap();
                    self.tim.arr.write(|w| unsafe { w.bits(u32(arr)) });
                }

                /// Clear uif bit
                pub fn clear_uif_bit(&mut self) {
                    self.tim.sr.modify(|_, w| w.uif().clear_bit());
                }

                /// Pauses the TIM peripheral
                pub fn pause(&mut self) {
                    self.tim.cr1.modify(|_, w| w.cen().clear_bit());
                }

                /// Resume (unpause) the TIM peripheral
                pub fn resume(&mut self) {
                    self.tim.cr1.modify(|_, w| w.cen().set_bit());
                }

                /// Reset the counter of the TIM peripheral
                pub fn reset_counter(&mut self) {
                    self.tim.cnt.reset();
                }

                /// Read the counter of the TIM peripheral
                pub fn counter(&self) -> u32 {
                    self.tim.cnt.read().bits()
                }

                /// Start listening for `event`
                pub fn listen(&mut self, event: Event) {
                    match event {
                        Event::TimeOut => {
                            // Enable update event interrupt
                            self.tim.dier.write(|w| w.uie().set_bit());
                        }
                    }
                }

                /// Stop listening for `event`
                pub fn unlisten(&mut self, event: Event) {
                    match event {
                        Event::TimeOut => {
                            // Enable update event interrupt
                            self.tim.dier.write(|w| w.uie().clear_bit());
                        }
                    }
                }

                /// Clears interrupt flag
                pub fn clear_irq(&mut self) {
                    self.tim.sr.modify(|_, w| {
                        // Clears timeout event
                        w.uif().clear_bit()
                    });
                }

                /// Releases the TIM peripheral
                pub fn free(mut self) -> ($TIMX, rec::$Rec) {
                    // pause counter
                    self.pause();

                    (self.tim, rec::$Rec { _marker: PhantomData })
                }
            }
        )+
    }
}

hal! {
    // Advanced-control
    TIM1: (tim1, Tim1),
    TIM8: (tim8, Tim8),

    // General-purpose
    TIM2: (tim2, Tim2),
    TIM3: (tim3, Tim3),
    TIM4: (tim4, Tim4),
    TIM5: (tim5, Tim5),

    // Basic
    TIM6: (tim6, Tim6),
    TIM7: (tim7, Tim7),

    // General-purpose
    TIM12: (tim12, Tim12),
    TIM13: (tim13, Tim13),
    TIM14: (tim14, Tim14),

    // General-purpose
    TIM15: (tim15, Tim15),
    TIM16: (tim16, Tim16),
    TIM17: (tim17, Tim17),
}
