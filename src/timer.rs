//! Timers

// TODO: on the h7x3 at least, only TIM2, TIM3, TIM4, TIM5 can support 32 bits.
// TIM1 is 16 bit.

use core::convert::TryInto;
use core::fmt::Debug;
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
use crate::time::{self, Duration, Hertz, Nanoseconds, Rate};
use stm32h7::Variant::Val;

/// Used to configure periodic timer timeouts using either a duration or a frequency
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum Timeout {
    Duration(Nanoseconds<u64>),
    Frequency(Hertz),
}

impl Into<Hertz> for Timeout {
    fn into(self) -> Hertz {
        match self {
            Timeout::Frequency(hertz) => hertz,
            Timeout::Duration(ns) => ns.to_rate().unwrap(),
        }
    }
}

impl Into<Nanoseconds<u64>> for Timeout {
    fn into(self) -> Nanoseconds<u64> {
        match self {
            Timeout::Frequency(hertz) => hertz.to_duration().unwrap(),
            Timeout::Duration(ns) => ns,
        }
    }
}

impl From<core::time::Duration> for Timeout {
    fn from(dur: core::time::Duration) -> Self {
        Timeout::Duration(dur.try_into().unwrap())
    }
}

macro_rules! impl_timeout_duration {
    ($ty:ty) => {
        impl From<$ty> for Timeout {
            fn from(dur: $ty) -> Self {
                Timeout::Duration(dur.try_into().unwrap())
            }
        }
    };
}

impl_timeout_duration!(Nanoseconds<u64>);
impl_timeout_duration!(time::Milliseconds);
impl_timeout_duration!(time::Seconds);
impl_timeout_duration!(Nanoseconds);
impl_timeout_duration!(time::Hours);
impl_timeout_duration!(time::Minutes);

macro_rules! impl_timeout_rate {
    ($ty:ty) => {
        impl From<$ty> for Timeout {
            fn from(rate: $ty) -> Self {
                Timeout::Frequency(rate.try_into().unwrap())
            }
        }
    };
}

impl_timeout_rate!(time::Hertz);
impl_timeout_rate!(time::Kibihertz);
impl_timeout_rate!(time::Kilohertz);
impl_timeout_rate!(time::Mebihertz);
impl_timeout_rate!(time::Megahertz);

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

    /// Configures a periodic timer
    ///
    /// Generates an overflow event at the `timeout` frequency.
    fn timer<T>(
        self,
        timeout: T,
        prec: Self::Rec,
        clocks: &CoreClocks,
    ) -> Timer<TIM>
    where
        T: Into<Timeout>;

    /// Configures the timer to count up at the given frequency
    ///
    /// Counts from 0 to the counter's maximum value, then repeats.
    /// Because this only uses the timer prescaler, the frequency
    /// is rounded to a multiple of the timer's kernel clock.
    fn tick_timer<F>(
        self,
        frequency: F,
        prec: Self::Rec,
        clocks: &CoreClocks,
    ) -> Timer<TIM>
    where
        F: Into<Timeout>;
}

/// Hardware timers
pub struct Timer<TIM> {
    clk: u32,
    tim: TIM,
}

/// Timer Events
///
/// Each event is a possible interrupt source, if enabled
pub enum Event {
    /// Timer timed out / count down ended
    TimeOut,
}

macro_rules! hal {
    ($($TIMX:ident: ($timX:ident, $Rec:ident, $cntType:ty),)+) => {
        $(
            impl Periodic for Timer<$TIMX> {}

            impl CountDown for Timer<$TIMX> {
                type Time = Timeout;

                fn start<F>(&mut self, timeout: F)
                where
                    F: Into<Timeout>,
                {
                    // Pause
                    self.pause();

                    // Reset counter
                    self.tim.cnt.reset();

                    // UEV event occours on next overflow
                    self.tim.cr1.modify(|_, w| w.urs().counter_only());
                    self.clear_uif_bit();

                    // Set PSC and ARR
                    let timeout = timeout.into();
                    self.set_timeout(timeout);

                    // Generate an update event to force an update of the ARR register. This ensures
                    // the first timer cycle is of the specified duration.
                    self.tim.egr.write(|w| w.ug().set_bit());

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
                    T: Into<Timeout>,
                {
                    let mut timer = Timer::$timX(self, prec, clocks);
                    timer.start(timeout);
                    timer
                }

                fn tick_timer<F>(self, frequency: F,
                                 prec: Self::Rec, clocks: &CoreClocks
                ) -> Timer<$TIMX>
                where
                    F: Into<Timeout>,
                {
                    let mut timer = Timer::$timX(self, prec, clocks);

                    timer.pause();

                    // UEV event occours on next overflow
                    timer.tim.cr1.modify(|_, w| w.urs().counter_only());
                    timer.clear_uif_bit();

                    // Set PSC and ARR
                    timer.set_tick_freq(frequency);

                    // Generate an update event to force an update of the ARR register. This ensures
                    // the first timer cycle is of the specified duration.
                    timer.tim.egr.write(|w| w.ug().set_bit());

                    // Start counter
                    timer.resume();

                    timer
                }
            }

            impl Timer<$TIMX> {
                /// Configures a TIM peripheral as a periodic count down timer, without starting it
                pub fn $timX(tim: $TIMX, prec: rec::$Rec, clocks: &CoreClocks) -> Self
                {
                    // enable and reset peripheral to a clean slate state
                    prec.enable().reset();

                    let clk = $TIMX::get_clk(clocks)
                        .expect("Timer input clock not running!").0;

                    Timer {
                        clk,
                        tim,
                    }
                }

                /// Configures the timer's frequency and counter reload value
                /// so that it underflows at the timeout's frequency
                #[deprecated(since = "0.8.0", note = "use set_timeout(), it works with frequencies now")]
                pub fn set_freq<T>(&mut self, frequency: T)
                where
                    T: TryInto<Hertz>,
                    T::Error: Debug,
                {
                    self.set_timeout(frequency.try_into().unwrap())
                }

                /// Sets the timer's timeout period from a duration or frequency
                ///
                /// ```
                /// // Set timeout to 100ms
                /// timer.set_timeout(100u32.milliseconds());
                ///
                /// // core::time::Duration is also supported
                /// let duration = core::time::Duration::from_nanos(2_500);
                /// // Set timeout to 2.5µs
                /// timer.set_timeout(duration);
                ///
                /// // Set timeout to 100kHz
                /// timer.set_timeout(100u32.kHz());
                /// ```
                pub fn set_timeout<T>(&mut self, timeout: T)
                where
                    T: Into<Timeout>,
                {
                    const NANOS_PER_SECOND: u64 = 1_000_000_000;

                    let timeout: Nanoseconds<u64> = timeout.into().into();
                    let clk = self.clk as u64;
                    let ticks = clk * timeout.0 / NANOS_PER_SECOND;
                    let ticks = ticks.try_into().unwrap_or(u32::MAX).max(1);

                    let psc = u16((ticks - 1) / (1 << 16)).unwrap();
                    self.tim.psc.write(|w| w.psc().bits(psc));

                    let arr = u16(ticks / u32(psc + 1)).unwrap();
                    self.tim.arr.write(|w| unsafe { w.bits(u32(arr)) });
                }

                /// Configures the timer to count up at the given frequency
                ///
                /// Counts from 0 to the counter's maximum value, then repeats.
                /// Because this only uses the timer prescaler, the frequency
                /// is rounded to a multiple of the timer's kernel clock.
                pub fn set_tick_freq<F>(&mut self, frequency: F)
                where
                    F: Into<Timeout>,
                {
                    let frequency: Hertz = frequency.into().into();
                    let div = self.clk / frequency.0;

                    let psc = u16(div - 1).unwrap();
                    self.tim.psc.write(|w| w.psc().bits(psc));

                    let counter_max = u32(<$cntType>::MAX);
                    self.tim.arr.write(|w| unsafe { w.bits(counter_max) });
                }

                /// Applies frequency/timeout changes immediately
                ///
                /// The timer will normally update its prescaler and auto-reload
                /// value when its counter overflows. This function causes
                /// those changes to happen immediately. Also clears the counter.
                pub fn apply_freq(&mut self) {
                    self.tim.egr.write(|w| w.ug().set_bit());
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
    TIM1: (tim1, Tim1, u16),
    TIM8: (tim8, Tim8, u16),

    // General-purpose
    TIM2: (tim2, Tim2, u32),
    TIM3: (tim3, Tim3, u16),
    TIM4: (tim4, Tim4, u16),
    TIM5: (tim5, Tim5, u32),

    // Basic
    TIM6: (tim6, Tim6, u16),
    TIM7: (tim7, Tim7, u16),

    // General-purpose
    TIM12: (tim12, Tim12, u16),
    TIM13: (tim13, Tim13, u16),
    TIM14: (tim14, Tim14, u16),

    // General-purpose
    TIM15: (tim15, Tim15, u16),
    TIM16: (tim16, Tim16, u16),
    TIM17: (tim17, Tim17, u16),
}
