//! Timers

// TODO: on the h7x3 at least, only TIM2, TIM3, TIM4, TIM5 can support 32 bits.
// TIM1 is 16 bit.

use crate::hal::timer::{CountDown, Periodic};
use crate::stm32::{LPTIM1, LPTIM2, LPTIM3, LPTIM4, LPTIM5};
use crate::stm32::{
    TIM1, TIM12, TIM13, TIM14, TIM15, TIM16, TIM17, TIM2, TIM3, TIM4, TIM5,
    TIM6, TIM7, TIM8,
};

use cast::{u16, u32};
use nb;
use void::Void;

use crate::rcc::Ccdr;
use crate::stm32::rcc::{d2ccip2r, d3ccipr};
use crate::time::Hertz;
use stm32h7::Variant::Val;

/// Associate clocks with timers
pub trait GetClk {
    fn get_clk(ccdr: &Ccdr) -> Option<Hertz>;
}

/// Timers with CK_INT derived from rcc_tim[xy]_ker_ck
macro_rules! impl_tim_ker_ck {
    ($($ckX:ident: $($TIMX:ident),+)+) => {
        $(
            $(
                impl GetClk for $TIMX {
                    fn get_clk(ccdr: &Ccdr) -> Option<Hertz> {
                        Some(ccdr.clocks.$ckX())
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
    fn get_clk(ccdr: &Ccdr) -> Option<Hertz> {
        match ccdr.rb.d2ccip2r.read().lptim1sel().variant() {
            Val(d2ccip2r::LPTIM1SEL_A::RCC_PCLK1) => Some(ccdr.clocks.pclk1()),
            Val(d2ccip2r::LPTIM1SEL_A::PLL2_P) => ccdr.clocks.pll2_p_ck(),
            Val(d2ccip2r::LPTIM1SEL_A::PLL3_R) => ccdr.clocks.pll3_r_ck(),
            Val(d2ccip2r::LPTIM1SEL_A::LSE) => unimplemented!(),
            Val(d2ccip2r::LPTIM1SEL_A::LSI) => unimplemented!(),
            Val(d2ccip2r::LPTIM1SEL_A::PER) => ccdr.clocks.per_ck(),
            _ => unreachable!(),
        }
    }
}
/// LPTIM2 Kernel Clock
impl GetClk for LPTIM2 {
    /// Current kernel clock
    fn get_clk(ccdr: &Ccdr) -> Option<Hertz> {
        match ccdr.rb.d3ccipr.read().lptim2sel().variant() {
            Val(d3ccipr::LPTIM2SEL_A::RCC_PCLK4) => Some(ccdr.clocks.pclk4()),
            Val(d3ccipr::LPTIM2SEL_A::PLL2_P) => ccdr.clocks.pll2_p_ck(),
            Val(d3ccipr::LPTIM2SEL_A::PLL3_R) => ccdr.clocks.pll3_r_ck(),
            Val(d3ccipr::LPTIM2SEL_A::LSE) => unimplemented!(),
            Val(d3ccipr::LPTIM2SEL_A::LSI) => unimplemented!(),
            Val(d3ccipr::LPTIM2SEL_A::PER) => ccdr.clocks.per_ck(),
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
                fn get_clk(ccdr: &Ccdr) -> Option<Hertz> {
                    match ccdr.rb.d3ccipr.read().lptim345sel().variant() {
                        Val(d3ccipr::LPTIM345SEL_A::RCC_PCLK4) => Some(ccdr.clocks.pclk4()),
                        Val(d3ccipr::LPTIM345SEL_A::PLL2_P) => ccdr.clocks.pll2_p_ck(),
                        Val(d3ccipr::LPTIM345SEL_A::PLL3_R) => ccdr.clocks.pll3_r_ck(),
                        Val(d3ccipr::LPTIM345SEL_A::LSE) => unimplemented!(),
                        Val(d3ccipr::LPTIM345SEL_A::LSI) => unimplemented!(),
                        Val(d3ccipr::LPTIM345SEL_A::PER) => ccdr.clocks.per_ck(),
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
    fn timer<T>(self, timeout: T, ccdr: &mut Ccdr) -> Timer<TIM>
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

    /// Timer requests a DMA transaction when timed out.
    DmaRequest,
}

macro_rules! hal {
    ($($TIMX:ident: ($timX:ident, $apb:ident, $timXen:ident, $timXrst:ident),)+) => {
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
                fn timer<T>(self, timeout: T, ccdr: &mut Ccdr) -> Timer<$TIMX>
                    where
                        T: Into<Hertz>,
                {
                    Timer::$timX(self, timeout, ccdr)
                }
            }

            impl Timer<$TIMX> {
                /// Configures a TIM peripheral as a periodic count down timer
                pub fn $timX<T>(tim: $TIMX, timeout: T, ccdr: &mut Ccdr) -> Self
                where
                    T: Into<Hertz>,
                {
                    // enable and reset peripheral to a clean slate state
                    ccdr.$apb.enr().modify(|_, w| w.$timXen().set_bit());
                    ccdr.$apb.rstr().modify(|_, w| w.$timXrst().set_bit());
                    ccdr.$apb.rstr().modify(|_, w| w.$timXrst().clear_bit());

                    let clk = $TIMX::get_clk(&ccdr)
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
                            self.tim.dier.modify(|_, w| w.uie().set_bit());
                        },
                        Event::DmaRequest => {
                            self.tim.dier.modify(|_, w| w.ude().set_bit());
                        }
                    }
                }

                /// Stop listening for `event`
                pub fn unlisten(&mut self, event: Event) {
                    match event {
                        Event::TimeOut => {
                            // Enable update event interrupt
                            self.tim.dier.modify(|_, w| w.uie().clear_bit());
                        },
                        Event::DmaRequest => {
                            self.tim.dier.modify(|_, w| w.ude().clear_bit());
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
                pub fn free(mut self) -> $TIMX {
                    // pause counter
                    self.pause();
                    self.tim
                }
            }
        )+
    }
}

hal! {
    // Advanced-control
    TIM1: (tim1, apb2, tim1en, tim1rst),
    TIM8: (tim8, apb2, tim8en, tim8rst),

    // General-purpose
    TIM2: (tim2, apb1l, tim2en, tim2rst),
    TIM3: (tim3, apb1l, tim3en, tim3rst),
    TIM4: (tim4, apb1l, tim4en, tim4rst),
    TIM5: (tim5, apb1l, tim5en, tim5rst),

    // Basic
    TIM6: (tim6, apb1l, tim6en, tim6rst),
    TIM7: (tim7, apb1l, tim7en, tim7rst),

    // General-purpose
    TIM12: (tim12, apb1l, tim12en, tim12rst),
    TIM13: (tim13, apb1l, tim13en, tim13rst),
    TIM14: (tim14, apb1l, tim14en, tim14rst),

    // General-purpose
    TIM15: (tim15, apb2, tim15en, tim15rst),
    TIM16: (tim16, apb2, tim16en, tim16rst),
    TIM17: (tim17, apb2, tim17en, tim17rst),
}
