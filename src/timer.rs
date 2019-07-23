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
use crate::rcc::{APB1L, APB2};
use crate::stm32::rcc::{d2ccip2r, d3ccipr};
use crate::time::Hertz;

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
        match ccdr.rb.d2ccip2r.read().lptim1sel() {
            d2ccip2r::LPTIM1SELR::RCC_PCLK1 => Some(ccdr.clocks.pclk1()),
            d2ccip2r::LPTIM1SELR::PLL2_P => ccdr.clocks.pll2_p_ck(),
            d2ccip2r::LPTIM1SELR::PLL3_R => ccdr.clocks.pll3_r_ck(),
            d2ccip2r::LPTIM1SELR::LSE => unimplemented!(),
            d2ccip2r::LPTIM1SELR::LSI => unimplemented!(),
            d2ccip2r::LPTIM1SELR::PER => ccdr.clocks.per_ck(),
            _ => unreachable!(),
        }
    }
}
/// LPTIM2 Kernel Clock
impl GetClk for LPTIM2 {
    /// Current kernel clock
    fn get_clk(ccdr: &Ccdr) -> Option<Hertz> {
        match ccdr.rb.d3ccipr.read().lptim2sel() {
            d3ccipr::LPTIM2SELR::RCC_PCLK4 => Some(ccdr.clocks.pclk4()),
            d3ccipr::LPTIM2SELR::PLL2_P => ccdr.clocks.pll2_p_ck(),
            d3ccipr::LPTIM2SELR::PLL3_R => ccdr.clocks.pll3_r_ck(),
            d3ccipr::LPTIM2SELR::LSE => unimplemented!(),
            d3ccipr::LPTIM2SELR::LSI => unimplemented!(),
            d3ccipr::LPTIM2SELR::PER => ccdr.clocks.per_ck(),
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
                    match ccdr.rb.d3ccipr.read().lptim345sel() {
                        d3ccipr::LPTIM345SELR::RCC_PCLK4 => Some(ccdr.clocks.pclk4()),
                        d3ccipr::LPTIM345SELR::PLL2_P => ccdr.clocks.pll2_p_ck(),
                        d3ccipr::LPTIM345SELR::PLL3_R => ccdr.clocks.pll3_r_ck(),
                        d3ccipr::LPTIM345SELR::LSE => unimplemented!(),
                        d3ccipr::LPTIM345SELR::LSI => unimplemented!(),
                        d3ccipr::LPTIM345SELR::PER => ccdr.clocks.per_ck(),
                        _ => unreachable!(),
                    }
                }
            }
        )+
    }
}
impl_clk_lptim345! { LPTIM3, LPTIM4, LPTIM5 }

/// Hardware timers
pub struct Timer<TIM> {
    clk: u32,
    tim: TIM,
    timeout: Hertz,
}

/// Interrupt events
pub enum Event {
    /// Timer timed out / count down ended
    TimeOut,
}

macro_rules! hal {
    ($($TIMX:ident: ($timX:ident, $APB:ident, $timXen:ident, $timXrst:ident),)+) => {
        $(
            impl Periodic for Timer<$TIMX> {}

            impl CountDown for Timer<$TIMX> {
                type Time = Hertz;

                #[allow(unused_unsafe)]
                fn start<T>(&mut self, timeout: T)
                where
                    T: Into<Hertz>,
                {
                    // pause
                    self.pause();
                    // restart counter
                    self.tim.cnt.reset();

                    self.set_freq(timeout);

                    // start counter
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

            impl Timer<$TIMX> {
                // XXX(why not name this `new`?) bummer: constructors need to have different names
                // even if the `$TIMX` are non overlapping (compare to the `free` function below
                // which just works)
                /// Configures a TIM peripheral as a periodic count down timer
                pub fn $timX<T>(tim: $TIMX, timeout: T, ccdr: &Ccdr, apb: &mut $APB) -> Self
                where
                    T: Into<Hertz>,
                {
                    // enable and reset peripheral to a clean slate state
                    apb.enr().modify(|_, w| w.$timXen().set_bit());
                    apb.rstr().modify(|_, w| w.$timXrst().set_bit());
                    apb.rstr().modify(|_, w| w.$timXrst().clear_bit());

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

                /// Starts listening for an `event`
                pub fn listen(&mut self, event: Event) {
                    match event {
                        Event::TimeOut => {
                            // Enable update event interrupt
                            self.tim.dier.write(|w| w.uie().set_bit());
                        }
                    }
                }

                /// Stops listening for an `event`
                pub fn unlisten(&mut self, event: Event) {
                    match event {
                        Event::TimeOut => {
                            // Enable update event interrupt
                            self.tim.dier.write(|w| w.uie().clear_bit());
                        }
                    }
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
    TIM1: (tim1, APB2, tim1en, tim1rst),
    TIM2: (tim2, APB1L, tim2en, tim2rst),
    // TIM3: (tim3, APB1, tim3en, tim3rst),
    // TIM4: (tim4, APB1, tim4en, tim4rst),
    // TIM5: (tim5, APB1, tim7en, tim7rst),
}
