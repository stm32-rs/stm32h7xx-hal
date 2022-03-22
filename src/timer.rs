//! Timers
//!
//! # Examples
//!
//! - [Blinky using a Timer](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/blinky_timer.rs)
//! - [64 bit microsecond timer](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/tick_timer.rs)

// TODO: on the h7x3 at least, only TIM2, TIM3, TIM4, TIM5 can support 32 bits.
// TIM1 is 16 bit.

use core::convert::TryFrom;
use core::marker::PhantomData;

use crate::hal::timer::{CountDown, Periodic};

use crate::stm32::{lptim1, lptim3};
use crate::stm32::{LPTIM1, LPTIM2, LPTIM3};
#[cfg(not(feature = "rm0455"))]
use crate::stm32::{LPTIM4, LPTIM5};

use crate::stm32::{
    TIM1, TIM12, TIM13, TIM14, TIM15, TIM16, TIM17, TIM2, TIM3, TIM4, TIM5,
    TIM6, TIM7, TIM8,
};

use cast::{u16, u32};
use void::Void;

#[cfg(feature = "rm0455")]
use crate::stm32::rcc::{cdccip2r as ccip2r, srdccipr};
#[cfg(not(feature = "rm0455"))]
use crate::stm32::rcc::{d2ccip2r as ccip2r, d3ccipr as srdccipr};

use crate::rcc::{rec, CoreClocks, ResetEnable};
use crate::stm32;
use crate::time::Hertz;

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
        #[cfg(not(feature = "rm0455"))]
        let ccip2r = &unsafe { &*stm32::RCC::ptr() }.d2ccip2r;
        #[cfg(feature = "rm0455")]
        let ccip2r = &unsafe { &*stm32::RCC::ptr() }.cdccip2r;

        match ccip2r.read().lptim1sel().variant() {
            Some(ccip2r::LPTIM1SEL_A::RCC_PCLK1) => Some(clocks.pclk1()),
            Some(ccip2r::LPTIM1SEL_A::PLL2_P) => clocks.pll2_p_ck(),
            Some(ccip2r::LPTIM1SEL_A::PLL3_R) => clocks.pll3_r_ck(),
            Some(ccip2r::LPTIM1SEL_A::LSE) => unimplemented!(),
            Some(ccip2r::LPTIM1SEL_A::LSI) => unimplemented!(),
            Some(ccip2r::LPTIM1SEL_A::PER) => clocks.per_ck(),
            _ => unreachable!(),
        }
    }
}
/// LPTIM2 Kernel Clock
impl GetClk for LPTIM2 {
    /// Current kernel clock
    fn get_clk(clocks: &CoreClocks) -> Option<Hertz> {
        // unsafe: read only
        #[cfg(not(feature = "rm0455"))]
        let srdccipr = &unsafe { &*stm32::RCC::ptr() }.d3ccipr;
        #[cfg(feature = "rm0455")]
        let srdccipr = &unsafe { &*stm32::RCC::ptr() }.srdccipr;

        match srdccipr.read().lptim2sel().variant() {
            Some(srdccipr::LPTIM2SEL_A::RCC_PCLK4) => Some(clocks.pclk4()),
            Some(srdccipr::LPTIM2SEL_A::PLL2_P) => clocks.pll2_p_ck(),
            Some(srdccipr::LPTIM2SEL_A::PLL3_R) => clocks.pll3_r_ck(),
            Some(srdccipr::LPTIM2SEL_A::LSE) => unimplemented!(),
            Some(srdccipr::LPTIM2SEL_A::LSI) => unimplemented!(),
            Some(srdccipr::LPTIM2SEL_A::PER) => clocks.per_ck(),
            _ => unreachable!(),
        }
    }
}

#[cfg(feature = "rm0455")]
/// LPTIM3 Kernel Clock
impl GetClk for LPTIM3 {
    /// Current kernel clock
    fn get_clk(clocks: &CoreClocks) -> Option<Hertz> {
        // unsafe: read only
        let srdccipr = &unsafe { &*stm32::RCC::ptr() }.srdccipr;

        match srdccipr.read().lptim3sel().bits() {
            0 => Some(clocks.pclk4()),
            1 => clocks.pll2_p_ck(),
            2 => clocks.pll3_r_ck(),
            3 => unimplemented!(),
            4 => unimplemented!(),
            5 => clocks.per_ck(),
            _ => unreachable!(),
        }
    }
}

#[cfg(not(feature = "rm0455"))]
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
                        Some(srdccipr::LPTIM345SEL_A::RCC_PCLK4) => Some(clocks.pclk4()),
                        Some(srdccipr::LPTIM345SEL_A::PLL2_P) => clocks.pll2_p_ck(),
                        Some(srdccipr::LPTIM345SEL_A::PLL3_R) => clocks.pll3_r_ck(),
                        Some(srdccipr::LPTIM345SEL_A::LSE) => unimplemented!(),
                        Some(srdccipr::LPTIM345SEL_A::LSI) => unimplemented!(),
                        Some(srdccipr::LPTIM345SEL_A::PER) => clocks.per_ck(),
                        _ => unreachable!(),
                    }
                }
            }
        )+
    }
}
#[cfg(not(feature = "rm0455"))]
impl_clk_lptim345! { LPTIM3, LPTIM4, LPTIM5 }

/// Enabled LPTIM (type state)
pub struct Enabled;
/// Disabled LPTIM (type state)
pub struct Disabled;

/// External trait for hardware timers
pub trait TimerExt<TIM> {
    type Rec: ResetEnable;

    /// Configures a periodic timer
    ///
    /// Generates an overflow event at the `timeout` frequency.
    fn timer<T>(self, timeout: T, prec: Self::Rec, clocks: &CoreClocks) -> TIM
    where
        T: Into<Hertz>;

    /// Configures the timer to count up at the given frequency
    ///
    /// Counts from 0 to the counter's maximum value, then repeats.
    /// Because this only uses the timer prescaler, the frequency
    /// is rounded to a multiple of the timer's kernel clock.
    ///
    /// For example, calling `.tick_timer(1.MHz(), ..)` for a 16-bit timer will
    /// result in a timers that increments every microsecond and overflows every
    /// ~65 milliseconds
    fn tick_timer<T>(
        self,
        frequency: T,
        prec: Self::Rec,
        clocks: &CoreClocks,
    ) -> TIM
    where
        T: Into<Hertz>;
}

/// Hardware timers
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Timer<TIM> {
    clk: u32,
    tim: TIM,
}

/// Low power hardware timers
#[derive(Debug)]
pub struct LpTimer<TIM, ED> {
    clk: u32,
    tim: TIM,
    timeout: Hertz,
    _enabled: PhantomData<ED>,
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
                type Time = Hertz;

                fn start<T>(&mut self, timeout: T)
                where
                    T: Into<Hertz>,
                {
                    // Pause
                    self.pause();

                    // Reset counter
                    self.reset_counter();

                    // UEV event occours on next overflow
                    self.urs_counter_only();
                    self.clear_irq();

                    // Set PSC and ARR
                    self.set_freq(timeout);

                    // Generate an update event to force an update of the ARR register. This ensures
                    // the first timer cycle is of the specified duration.
                    self.apply_freq();

                    // Start counter
                    self.resume()
                }

                fn wait(&mut self) -> nb::Result<(), Void> {
                    if self.is_irq_clear() {
                        Err(nb::Error::WouldBlock)
                    } else {
                        self.clear_irq();
                        Ok(())
                    }
                }
            }

            impl TimerExt<Timer<$TIMX>> for $TIMX {
                type Rec = rec::$Rec;

                fn timer<T>(self, timeout: T,
                            prec: Self::Rec, clocks: &CoreClocks
                ) -> Timer<$TIMX>
                where
                    T: Into<Hertz>,
                {
                    let mut timer = Timer::$timX(self, prec, clocks);
                    timer.start(timeout);
                    timer
                }

                fn tick_timer<T>(self, frequency: T,
                                 prec: Self::Rec, clocks: &CoreClocks
                ) -> Timer<$TIMX>
                where
                    T: Into<Hertz>,
                {
                    let mut timer = Timer::$timX(self, prec, clocks);

                    timer.pause();

                    // UEV event occours on next overflow
                    timer.urs_counter_only();
                    timer.clear_irq();

                    // Set PSC and ARR
                    timer.set_tick_freq(frequency);

                    // Generate an update event to force an update of the ARR
                    // register. This ensures the first timer cycle is of the
                    // specified duration.
                    timer.apply_freq();

                    // Start counter
                    timer.resume();

                    timer
                }
            }

            impl Timer<$TIMX> {
                /// Configures a TIM peripheral as a periodic count down timer,
                /// without starting it
                pub fn $timX(tim: $TIMX, prec: rec::$Rec, clocks: &CoreClocks) -> Self
                {
                    // enable and reset peripheral to a clean state
                    prec.enable().reset();

                    let clk = $TIMX::get_clk(clocks)
                        .expect(concat!(stringify!($TIMX), ": Input clock not running!")).0;

                    Timer {
                        clk,
                        tim,
                    }
                }

                /// Configures the timer's frequency and counter reload value
                /// so that it underflows at the timeout's frequency
                pub fn set_freq<T>(&mut self, timeout: T)
                where
                    T: Into<Hertz>,
                {
                    let timeout = timeout.into();
                    let ticks = self.clk / timeout.0;

                    self.set_timeout_ticks(ticks);
                }

                /// Sets the timer period from a time duration
                ///
                /// ```
                /// // Set timeout to 100ms
                /// timer.set_timeout(100.ms());
                /// ```
                ///
                /// Alternatively, the duration can be set using the
                /// core::time::Duration type
                ///
                /// ```
                /// let duration = core::time::Duration::from_nanos(2_500);
                ///
                /// // Set timeout to 2.5µs
                /// timer.set_timeout(duration);
                /// ```
                pub fn set_timeout<T>(&mut self, timeout: T)
                where
                    T: Into<core::time::Duration>
                {
                    const NANOS_PER_SECOND: u64 = 1_000_000_000;
                    let timeout = timeout.into();

                    let clk = self.clk as u64;
                    let ticks = u32::try_from(
                        clk * timeout.as_secs() +
                        clk * u64::from(timeout.subsec_nanos()) / NANOS_PER_SECOND,
                    )
                    .unwrap_or(u32::max_value());

                    self.set_timeout_ticks(ticks.max(1));
                }

                /// Sets the timer's prescaler and auto reload register so that the timer will reach
                /// the ARR after `ticks` amount of timer clock ticks.
                ///
                /// ```
                /// // Set auto reload register to 50000 and prescaler to divide by 2.
                /// timer.set_timeout_ticks(100000);
                /// ```
                ///
                /// This function will round down if the prescaler is used to extend the range:
                /// ```
                /// // Set auto reload register to 50000 and prescaler to divide by 2.
                /// timer.set_timeout_ticks(100001);
                /// ```
                fn set_timeout_ticks(&mut self, ticks: u32) {
                    let (psc, arr) = calculate_timeout_ticks_register_values(ticks);
                    self.tim.psc.write(|w| w.psc().bits(psc));
                    self.tim.arr.write(|w| unsafe { w.bits(u32(arr)) });
                }

                /// Configures the timer to count up at the given frequency
                ///
                /// Counts from 0 to the counter's maximum value, then repeats.
                /// Because this only uses the timer prescaler, the frequency
                /// is rounded to a multiple of the timer's kernel clock.
                pub fn set_tick_freq<T>(&mut self, frequency: T)
                where
                    T: Into<Hertz>,
                {
                    let frequency = frequency.into();
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

                /// Pauses the TIM peripheral
                pub fn pause(&mut self) {
                    self.tim.cr1.modify(|_, w| w.cen().clear_bit());
                }

                /// Resume (unpause) the TIM peripheral
                pub fn resume(&mut self) {
                    self.tim.cr1.modify(|_, w| w.cen().set_bit());
                }

                /// Set Update Request Source to counter overflow/underflow only
                pub fn urs_counter_only(&mut self) {
                    self.tim.cr1.modify(|_, w| w.urs().counter_only());
                }

                /// Reset the counter of the TIM peripheral
                pub fn reset_counter(&mut self) {
                    self.tim.cnt.reset();
                }

                /// Read the counter of the TIM peripheral
                pub fn counter(&self) -> u32 {
                    self.tim.cnt.read().cnt().bits().into()
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
                            // Disable update event interrupt
                            self.tim.dier.write(|w| w.uie().clear_bit());
                            let _ = self.tim.dier.read();
                            let _ = self.tim.dier.read(); // Delay 2 peripheral clocks
                        }
                    }
                }

                /// Check if Update Interrupt flag is cleared
                pub fn is_irq_clear(&mut self) -> bool {
                    self.tim.sr.read().uif().bit_is_clear()
                }

                /// Clears interrupt flag
                pub fn clear_irq(&mut self) {
                    self.tim.sr.modify(|_, w| {
                        // Clears timeout event
                        w.uif().clear_bit()
                    });
                    let _ = self.tim.sr.read();
                    let _ = self.tim.sr.read(); // Delay 2 peripheral clocks
                }

                /// Releases the TIM peripheral
                pub fn free(mut self) -> ($TIMX, rec::$Rec) {
                    // pause counter
                    self.pause();

                    (self.tim, rec::$Rec { _marker: PhantomData })
                }

                /// Returns a reference to the inner peripheral
                pub fn inner(&self) -> &$TIMX {
                    &self.tim
                }

                /// Returns a mutable reference to the inner peripheral
                pub fn inner_mut(&mut self) -> &mut $TIMX {
                    &mut self.tim
                }
            }
        )+
    }
}

/// We want to have `ticks` amount of timer ticks before it reloads.
/// But `ticks` may have a higher value than what the timer can hold directly.
/// So we'll use the prescaler to extend the range.
///
/// To know how many times we would overflow with a prescaler of 1, we divide `ticks` by 2^16 (the max amount of ticks per overflow).
/// If the result is e.g. 3, then we need to increase our range by 4 times to fit all the ticks.
/// We can increase the range enough by setting the prescaler to 3 (which will divide the clock freq by 4).
/// Because every tick is now 4x as long, we need to divide `ticks` by 4 to keep the same timeout.
///
/// This function returns the prescaler register value and auto reload register value.
fn calculate_timeout_ticks_register_values(ticks: u32) -> (u16, u16) {
    // Note (unwrap): Never panics because 32-bit value is shifted right by 16 bits,
    // resulting in a value that always fits in 16 bits.
    let psc = u16(ticks / (1 << 16)).unwrap();
    // Note (unwrap): Never panics because the divisor is always such that the result fits in 16 bits.
    let arr = u16(ticks / (u32(psc) + 1)).unwrap();
    (psc, arr)
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

macro_rules! lptim_hal {
    ($($TIMX:ident: ($timx:ident, $Rec:ident, $timXpac:ident),)+) => {
        $(
            impl Periodic for LpTimer<$TIMX, Enabled> {}

            impl CountDown for LpTimer<$TIMX, Enabled> {
                type Time = Hertz;

                fn start<T>(&mut self, timeout: T)
                where
                    T: Into<Hertz>,
                {
                    // Reset: counter must be running
                    self.reset_counter();

                    // Disable counter
                    self.tim.cr.write(|w| w.enable().disabled());

                    // Set prescale and ARR
                    self.priv_set_freq(timeout); // side effect: enables counter

                    // Clear IRQ
                    self.clear_irq();

                    // Start counter
                    self.tim.cr.write(|w| w.cntstrt().set_bit().enable().enabled());
                }

                fn wait(&mut self) -> nb::Result<(), Void> {
                    if self.tim.isr.read().arrm().bit_is_clear() {
                        Err(nb::Error::WouldBlock)
                    } else {
                        self.clear_irq();
                        Ok(())
                    }
                }
            }

            impl TimerExt<LpTimer<$TIMX, Enabled>> for $TIMX {
                type Rec = rec::$Rec;

                fn timer<T>(self, timeout: T,
                            prec: Self::Rec, clocks: &CoreClocks
                ) -> LpTimer<$TIMX, Enabled>
                    where
                        T: Into<Hertz>,
                {
                    LpTimer::$timx(self, timeout, prec, clocks)
                }

                fn tick_timer<T>(self, frequency: T,
                                 prec: Self::Rec, clocks: &CoreClocks
                ) -> LpTimer<$TIMX, Enabled>
                where
                    T: Into<Hertz>,
                {
                    // enable and reset peripheral to a clean state
                    prec.enable().reset();

                    let clk = $TIMX::get_clk(clocks)
                        .expect(concat!(stringify!($TIMX), ": Input clock not running!")).0;

                    let mut timer = LpTimer {
                        clk,
                        tim: self,
                        timeout: Hertz(0),
                        _enabled: PhantomData,
                    };

                    // Configures the timer to count up at the given frequency

                    // Reset: counter must be running
                    timer.reset_counter();

                    // Disable counter
                    timer.tim.cr.write(|w| w.enable().disabled());

                    // Set tick frequency
                    timer.priv_set_tick_freq(frequency);

                    // Clear IRQ
                    timer.clear_irq();

                    // Start counter
                    timer.tim.cr.write(|w| w.cntstrt().set_bit().enable().enabled());

                    timer
                }
            }

            impl LpTimer<$TIMX, Enabled> {
                /// Configures a LPTIM peripheral as a periodic count down timer
                pub fn $timx<T>(tim: $TIMX, timeout: T,
                                prec: rec::$Rec, clocks: &CoreClocks
                ) -> Self
                where
                    T: Into<Hertz>,
                {
                    // enable and reset peripheral to a clean state
                    prec.enable().reset();

                    let clk = $TIMX::get_clk(clocks)
                        .expect(concat!(stringify!($TIMX), ": Input clock not running!")).0;

                    let mut timer = LpTimer {
                        clk,
                        tim,
                        timeout: Hertz(0),
                        _enabled: PhantomData,
                    };
                    timer.start(timeout);

                    timer
                }

                /// Perform a synchronous reset of the LPTIM counter. The timer
                /// must be running.
                pub fn reset_counter(&mut self) {
                    // Counter Reset
                    self.tim.cr.write(|w| w.countrst().set_bit().enable().enabled());
                    let _ = self.tim.cr.read();
                    let _ = self.tim.cr.read(); // Delay 2 peripheral clocks
                }

                /// Disables the LPTIM peripheral
                pub fn pause(self) -> LpTimer<$TIMX, Disabled> {
                    // Disable the entire timer
                    self.tim.cr.write(|w| w.enable().disabled());

                    LpTimer {
                        clk: self.clk,
                        tim: self.tim,
                        timeout: self.timeout,
                        _enabled: PhantomData,
                    }
                }
            }

            impl LpTimer<$TIMX, Disabled> {
                /// Sets the frequency of the LPTIM counter
                ///
                /// The counter must be disabled
                pub fn set_freq<T>(&mut self, timeout: T)
                where
                    T: Into<Hertz>,
                {
                    self.priv_set_freq(timeout); // side effect: enables counter

                    // Disable timer
                    self.tim.cr.write(|w| w.enable().disabled());
                }

                /// Configures the timer to count up at the given frequency
                ///
                /// The counter must be disabled
                pub fn set_tick_freq<T>(&mut self, frequency: T)
                where
                    T: Into<Hertz>,
                {
                    self.priv_set_tick_freq(frequency); // side effect: enables counter

                    // Disable timer
                    self.tim.cr.write(|w| w.enable().disabled());
                }

                /// Start listening for `event`
                ///
                /// The timer must be disabled, see RM0433 Rev 7. 43.4.13
                pub fn listen(&mut self, event: Event) {
                    match event {
                        Event::TimeOut => {
                            // Enable autoreload match interrupt
                            self.tim.ier.modify(|_, w| w.arrmie().set_bit());
                        }
                    }
                }

                /// Stop listening for `event`
                ///
                /// The timer must be disabled, see RM0433 Rev 7. 43.4.13
                pub fn unlisten(&mut self, event: Event) {
                    match event {
                        Event::TimeOut => {
                            // Disable autoreload match interrupt
                            self.tim.ier.modify(|_, w| w.arrmie().clear_bit());
                            let _ = self.tim.ier.read();
                            let _ = self.tim.ier.read(); // Delay 2 peripheral clocks
                        }
                    }
                }

                /// Enables the LPTIM, and starts counting
                pub fn resume(self) -> LpTimer<$TIMX, Enabled> {
                    // Enable and start counting
                    self.tim.cr.write(|w| w.enable().enabled());
                    self.tim.cr.write(|w| w.cntstrt().set_bit().enable().enabled());

                    LpTimer {
                        clk: self.clk,
                        tim: self.tim,
                        timeout: self.timeout,
                        _enabled: PhantomData,
                    }
                }
            }

            impl<ED> LpTimer<$TIMX, ED> {
                /// Private method to set the frequency of the LPTIM
                /// counter. The counter must be disabled, but it will be
                /// enabled at the end of this method.
                fn priv_set_freq<T>(&mut self, timeout: T)
                where
                    T: Into<Hertz>,
                {
                    self.timeout = timeout.into();

                    let clk = self.clk;
                    let frequency = self.timeout.0;
                    let ticks = clk / frequency;
                    assert!(ticks < 128 * (1 << 16));

                    // Calculate prescaler
                    let (prescale, prescale_div) = match ticks / (1 << 16) {
                        0 => ($timXpac::cfgr::PRESC_A::DIV1, 1),
                        1 => ($timXpac::cfgr::PRESC_A::DIV2, 2),
                        2..=3 => ($timXpac::cfgr::PRESC_A::DIV4, 4),
                        4..=7 => ($timXpac::cfgr::PRESC_A::DIV8, 8),
                        8..=15 => ($timXpac::cfgr::PRESC_A::DIV16, 16),
                        16..=31 => ($timXpac::cfgr::PRESC_A::DIV32, 32),
                        32..=63 => ($timXpac::cfgr::PRESC_A::DIV64, 64),
                        _ => ($timXpac::cfgr::PRESC_A::DIV128, 128),
                    };

                    // Calcuate reload
                    let arr = ticks / prescale_div;
                    assert!(arr <= 0xFFFF);
                    assert!(arr > 0);

                    // Write CFGR: LPTIM must be disabled
                    self.tim.cfgr.modify(|_, w| w.presc().variant(prescale));

                    // Enable
                    self.tim.cr.write(|w| w.enable().enabled());

                    // Write ARR: LPTIM must be enabled
                    self.tim.arr.write(|w| w.arr().bits(arr as u16));
                    while self.tim.isr.read().arrok().bit_is_clear() {}
                    self.tim.icr.write(|w| w.arrokcf().clear());
                }

                /// Private method to configure the timer to count up at the
                /// given frequency
                ///
                /// The counter must be disabled, but it will be enabled at the
                /// end of this method
                fn priv_set_tick_freq<T>(&mut self, frequency: T)
                where
                    T: Into<Hertz>,
                {
                    // Calculate prescaler
                    let frequency = frequency.into().0;
                    let ticks = (self.clk + frequency - 1) / frequency;
                    assert!(ticks <= 128,
                            "LPTIM input clock is too slow to achieve this frequency");

                    let (prescale, _prescale_div) = match ticks {
                        0..=1 => ($timXpac::cfgr::PRESC_A::DIV1, 1),
                        2 => ($timXpac::cfgr::PRESC_A::DIV2, 2),
                        3..=4 => ($timXpac::cfgr::PRESC_A::DIV4, 4),
                        5..=8 => ($timXpac::cfgr::PRESC_A::DIV8, 8),
                        9..=16 => ($timXpac::cfgr::PRESC_A::DIV16, 16),
                        17..=32 => ($timXpac::cfgr::PRESC_A::DIV32, 32),
                        33..=64 => ($timXpac::cfgr::PRESC_A::DIV64, 64),
                        _ => ($timXpac::cfgr::PRESC_A::DIV128, 128),
                    };

                    // Write CFGR: LPTIM must be disabled
                    self.tim.cfgr.modify(|_, w| w.presc().variant(prescale));

                    // Enable
                    self.tim.cr.write(|w| w.enable().enabled());

                    // Set ARR = max

                    // Write ARR: LPTIM must be enabled
                    self.tim.arr.write(|w| w.arr().bits(0xFFFF as u16));
                    while self.tim.isr.read().arrok().bit_is_clear() {}
                    self.tim.icr.write(|w| w.arrokcf().clear());
                }

                /// Read the counter of the LPTIM peripheral
                pub fn counter(&self) -> u16 {
                    loop {
                        // Read once
                        let count1 = self.tim.cnt.read().cnt().bits();

                        // Read twice - see RM0433 Rev 7. 43.4.14
                        let count2 = self.tim.cnt.read().cnt().bits();

                        if count1 == count2 { return count2; }
                    }
                }

                /// Clears interrupt flag
                pub fn clear_irq(&mut self) {
                    // Clear autoreload match event
                    self.tim.icr.write(|w| w.arrmcf().set_bit());
                    while self.tim.isr.read().arrm().bit_is_set() {}
                }

                /// Releases the LPTIM peripheral
                pub fn free(self) -> ($TIMX, rec::$Rec) {
                    // Disable timer
                    self.tim.cr.write(|w| w.enable().disabled());

                    (self.tim, rec::$Rec { _marker: PhantomData })
                }
            }
        )+
    }
}

lptim_hal! {
    LPTIM1: (lptim1, Lptim1, lptim1),
    LPTIM2: (lptim2, Lptim2, lptim1),
    LPTIM3: (lptim3, Lptim3, lptim3),
}
#[cfg(not(feature = "rm0455"))]
lptim_hal! {
    LPTIM4: (lptim4, Lptim4, lptim3),
    LPTIM5: (lptim5, Lptim5, lptim3),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn timeout_ticks_register_values() {
        assert_eq!(calculate_timeout_ticks_register_values(0), (0, 0));
        assert_eq!(calculate_timeout_ticks_register_values(50000), (0, 50000));
        assert_eq!(calculate_timeout_ticks_register_values(100000), (1, 50000));
        assert_eq!(calculate_timeout_ticks_register_values(65535), (0, 65535));
        assert_eq!(calculate_timeout_ticks_register_values(65536), (1, 32768));
        assert_eq!(
            calculate_timeout_ticks_register_values(1000000),
            (15, 62500)
        );
        assert_eq!(
            calculate_timeout_ticks_register_values(u32::MAX),
            (u16::MAX, u16::MAX)
        );
    }
}
