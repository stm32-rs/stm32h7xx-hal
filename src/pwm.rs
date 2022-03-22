//! Pulse Width Modulation (PWM)
//!
//! PWM output is avaliable for the advanced control timers (`TIM1`, `TIM8`),
//! the general purpose timers (`TIM[2-5]`, `TIM[12-17]`) and the Low-power
//! timers (`LPTIM[1-5]`).
//!
//! Timers support up to 4 simultaneous PWM output channels
//!
//! ## Examples
//!
//! - [Simple example](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/pwm.rs)
//! - [Advanced example](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/pwm_advanced.rs)
//!
//! ## Usage
//!
//! ```rust
//! let gpioa = ..; // Set up and split GPIOA
//! let pins = (
//!     gpioa.pa8.into_alternate_af1(),
//!     gpioa.pa9.into_alternate_af1(),
//!     gpioa.pa10.into_alternate_af1(),
//!     gpioa.pa11.into_alternate_af1(),
//! );
//! ```
//!
//! To see which pins can be used with which timers, see your device datasheet or see which pins implement the [Pins](trait.Pins.html) trait.
//!
//! Then call the `pwm` function on the corresponding timer:
//!
//! ```
//!   let device: pac::Peripherals = ..;
//!
//!   // Put the timer in PWM mode using the specified pins
//!   // with a frequency of 100 hz.
//!   let (c0, c1, c2, c3) = device.TIM1.pwm(
//!       pins,
//!       100.hz(),
//!       prec,
//!       &clocks
//!   );
//!
//!   // Set the duty cycle of channel 0 to 50%
//!   c0.set_duty(c0.get_max_duty() / 2);
//!   // PWM outputs are disabled by default
//!   c0.enable()
//! ```
//!
//! ## Advanced features
//!
//! Some timers support various advanced features. These features are
//! accessed by calling TIMx.[pwm_advanced](trait.PwmAdvExt.html#tymethod.pwm_advanced) to get a [PwmBuilder](struct.PwmBuilder.html)
//! and calling appropriate methods of the PwmBuilder before calling [PwmBuilder::finalize](struct.PwmBuilder.html#method.finalize)
//! to create the PWM channels and a [PwmControl](struct.PwmControl.html) struct exposing things like [FaultMonitor](trait.FaultMonitor.html) functionality.
//!
//! ```rust
//! let gpioa = ..; // Set up and split GPIOA
//! let pins = (
//!     gpioa.pa8.into_alternate_af1(),
//!     gpioa.pa9.into_alternate_af1(),
//!     gpioa.pa10.into_alternate_af1(),
//!     gpioa.pa11.into_alternate_af1(),
//! );
//! ```
//!
//! Then call the `pwm_advanced` function on the corresponding timer:
//!
//! ```
//!   let device: pac::Peripherals = ..;
//!
//!   // Put the timer in PWM mode using the specified pins
//!   // with a frequency of 100 hz, 2us deadtime between complementary edges,
//!   // center-aligned PWM, and an active-low fault input
//!   let (mut control, (c1, c2, c3, c4)) = device.TIM1
//!       .pwm_advanced(
//!           pins,
//!           prec,
//!           &clocks
//!       )
//!       .frequency(100.hz())
//!       .center_aligned()
//!       .with_break_pin(gpioe.pe15.into_alternate_af1(), Polarity::ActiveLow)
//!       .finalize();
//! ```
//!
//! Then change some PWM channels to active-low (reversing pin polarity relative to logic polarity)
//! or enable a complementary PWM output (both only on some timer channels).
//!
//! ```
//!   // Set channel 1 to complementary with both the regular and complementary output active low
//!   let mut c1 = c1
//!       .into_complementary(gpioe.pe8.into_alternate_af1())
//!       .into_active_low()
//!       .into_comp_active_low();
//!
//!   // Set the duty cycle of channel 1 to 50%
//!   c1.set_duty(c1.get_max_duty() / 2);
//!
//!   // PWM outputs are disabled by default
//!   c1.enable()
//!
//!   // If a PWM fault happened, you can clear it through the control structure
//!   if control.is_fault_active() {
//!       control.clear_fault();
//!   }
//! ```
//!
//! ## Fault (Break) inputs
//!
//! The [PwmBuilder::with_break_pin](struct.PwmBuilder.html#method.with_break_pin) method emables break/fault functionality as described in the reference manual.
//!
//! The [PwmControl](struct.PwmControl.html) will then implement [FaultMonitor](trait.FaultMonitor.html) which can be used to monitor and control the fault status.
//!
//! If the break input becomes active, all PWM will be stopped.
//!
//! The BKIN hardware respects deadtimes when going into the fault state while the BKIN2 hardware acts immediately.
//!
//! The fault state puts all PWM pins into high-impedance mode, so pull-ups or pull-downs should be used to set the pins to a safe state.
//!
//! Currently only one break input (BKIN or BKIN2) can be enabled, this could be changed to allow two break inputs at the same time.
//!
//! ## Complementary outputs
//!
//! Once a PWM channel has been created through TIMx.pwm(...) or TIMx.pwm_advanced(...).finalize(), it can be put into complementary mode or have its polarity changed.
//!
//! [Pwm::into_complementary](struct.Pwm.html#method.into_complementary) takes a pin that can be used as a complementary output
//!
//! For allowed pins, see the device datasheet or see which pins implement the [NPins](trait.NPins.html) trait.
//!
//! ## PWM alignment
//!
//! A timer with multiple PWM channels can have different alignments between PWM channels.
//!
//! All PWM-capable timers support left aligned PWM. In left aligned PWM, all channels go active at the start of a PWM cycle then go inactive when their duty cycles expire.
//!
//! Some timers also support right aligned and center aligned PWM. In right aligned PWM, all channels go inactive at the end of a PWM cycle and go active when their duty cycle remains until the end of the PWM cycle.
//!
//! In center aligned PWM, all channels start inactive, then go active when half their duty cycle remains until the center of the PWM period, then go inactive again once their duty cycle expires and remain inactive for the rest of the PWM cycle.
//! This produces a symmetrical PWM waveform, with increasing duty cycle moving both the inactive and active edge equally.
//! When a component is placed across multiple PWM channels with different duty cycles in center aligned mode, the component will see twice the ripple frequency as the PWM switching frequency.
//!
//! ## PWM channel polarity
//!
//! A PWM channel is active or inactive based on the duty cycle, alignment, etc. However, the actual GPIO signal level that represents active vs inactive is configurable.
//!
//! The [into_active_low](struct.Pwm.html#method.into_active_low) and [into_active_high](struct.Pwm.html#method.into_active_high) methods set the active signal level to low (VSS) or high (VDD).
//!
//! The complementary output is active when the regular output is inactive. The active signal level of the complementary output is set by the [into_comp_active_low](struct.Pwm.html#method.into_comp_active_low), and [into_comp_active_high](struct.Pwm.html#method.into_comp_active_high) methods.
//!
//! ## Deadtime
//!
//! All channels on a given timer share the same deadtime setting as set by [PwmBuilder::with_deadtime](struct.PwmBuilder.html#method.with_deadtime)
//!
//! PWM channels with complementary outputs can have deadtime added to the signal. Dead time is used to prevent cross-conduction in some power electronics topologies.
//!
//! With complementary outputs and dead time enabled on a PWM channel, when the regular output goes inactive (high or low based on into_active_high/into_active_low), the complementary output remains inactive until the deadtime passes.
//! Similarily, when the complementary output goes inactive, the regular output waits until the deadtime passes before it goes active.
//!
//! Deadtime is applied based on the logical active/inactive levels. Depending on the PWM polarity and complementary polarity, both pins can be high or low during deadtime; they will both be in the inactive state.
//!
//! The deadtime must be 4032 counts of the timer clock or less or the builder will assert/panic. For a 200MHz timer this is 20 microseconds; slower timers can have even longer deadtimes.
//!
//! ## Disabled or faulted state
//!
//! At initialization, when a PWM channel is disabled, or while a fault is active, the PWM outputs will be in a high impedance state.
//!
//! If needed, pull-up or pull-down resistors should be used to ensure that all power electronics are in a safe state while the GPIO pins are high impedance.
//!
//! Although the timers allow quite a bit of configuration here, that would require configuring the PWM pins before configuring other parts of the timer, which would be a challenge with how type states and traits are used for timer configuration.
//!
//! Additionally, the GPIO will always be high-impedance during power-up or in reset, so pull-ups or pull-downs to ensure safe state are always a good idea.

use core::marker::PhantomData;
use core::mem::MaybeUninit;

use crate::hal;
use crate::stm32::{lptim1, lptim3};
use crate::stm32::{LPTIM1, LPTIM2, LPTIM3};
#[cfg(not(feature = "rm0455"))]
use crate::stm32::{LPTIM4, LPTIM5};
use crate::stm32::{
    TIM1, TIM12, TIM13, TIM14, TIM15, TIM16, TIM17, TIM2, TIM3, TIM4, TIM5,
    TIM8,
};

use crate::rcc::{rec, CoreClocks, ResetEnable};
use crate::time::{Hertz, NanoSeconds, U32Ext};
use crate::timer::GetClk;

use crate::gpio::{self, Alternate};

// This trait marks that a GPIO pin can be used with a specific timer channel
// TIM is the timer being used
// CHANNEL is a marker struct for the channel (or multi channels for tuples)
// Example: impl Pins<TIM1, C1> for PA8<Alternate<1>> { type Channel = Pwm<TIM1, C1>; }
/// Pins is a trait that marks which GPIO pins may be used as PWM channels; it should not be directly used.
/// See the device datasheet 'Pin descriptions' chapter for which pins can be used with which timer PWM channels (or look at Implementors)
pub trait Pins<TIM, CHANNEL, COMP> {
    type Channel;
}

/// NPins is a trait that marks which GPIO pins may be used as complementary PWM channels; it should not be directly used.
/// See the device datasheet 'Pin descriptions' chapter for which pins can be used with which timer PWM channels (or look at Implementors)
pub trait NPins<TIM, CHANNEL> {}

/// FaultPins is a trait that marks which GPIO pins may be used as PWM fault inputs; it should not be directly used.
/// See the device datasheet 'Pin descriptions' chapter for which pins can be used with which timer PWM channels (or look at Implementors)
pub trait FaultPins<TIM> {
    const INPUT: BreakInput;
}

/// Marker struct for PWM channel 1 on Pins trait and Pwm struct
pub struct C1;
/// Marker struct for PWM channel 2 on Pins trait and Pwm struct
pub struct C2;
/// Marker struct for PWM channel 3 on Pins trait and Pwm struct
pub struct C3;
/// Marker struct for PWM channel 4 on Pins trait and Pwm struct
pub struct C4;

/// Marker struct for pins and PWM channels that do not support complementary output
pub struct ComplementaryImpossible;
/// Marker struct for pins and PWM channels that support complementary output but are not using it
pub struct ComplementaryDisabled;
/// Marker struct for PWM channels that have complementary outputs enabled
pub struct ComplementaryEnabled;

/// Enum for IO polarity
pub enum Polarity {
    ActiveHigh,
    ActiveLow,
}

/// Configuration enum to keep track of which break input corresponds with which FaultPins
#[derive(PartialEq)]
pub enum BreakInput {
    BreakIn,
    BreakIn2,
}

/// Internal enum that keeps track of the count settings before PWM is finalized
enum CountSettings<WIDTH> {
    Frequency(Hertz),
    Explicit { period: WIDTH, prescaler: u16 },
}

/// Marker struct for active high IO polarity
pub struct ActiveHigh;
/// Marker struct for active low IO polarity
pub struct ActiveLow;

/// Whether a PWM signal is left-aligned, right-aligned, or center-aligned
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Alignment {
    Left,
    Right,
    Center,
}

/// Pwm represents one PWM channel; it is created by calling TIM?.pwm(...) and lets you control the channel through the PwmPin trait
pub struct Pwm<TIM, CHANNEL, COMP, POL, NPOL> {
    _channel: PhantomData<CHANNEL>,
    _tim: PhantomData<TIM>,
    _complementary: PhantomData<COMP>,
    _polarity: PhantomData<POL>,
    _npolarity: PhantomData<NPOL>,
}

/// PwmBuilder is used to configure advanced PWM features
pub struct PwmBuilder<TIM, PINS, CHANNEL, FAULT, COMP, WIDTH> {
    _tim: PhantomData<TIM>,
    _pins: PhantomData<PINS>,
    _channel: PhantomData<CHANNEL>,
    _fault: PhantomData<FAULT>,
    _comp: PhantomData<COMP>,
    alignment: Alignment,
    base_freq: Hertz,
    count: CountSettings<WIDTH>,
    bkin_enabled: bool, // If the FAULT type parameter is FaultEnabled, either bkin or bkin2 must be enabled
    bkin2_enabled: bool,
    fault_polarity: Polarity,
    deadtime: NanoSeconds,
}

/// Allows a PwmControl to monitor and control faults (break inputs) of a timer's PWM channels
pub trait FaultMonitor {
    /// Returns true if a fault is preventing PWM output
    fn is_fault_active(&self) -> bool;

    /// Enables PWM output, clearing fault state and immediately resuming PWM; if the break pin is still active, this can't clear the fault.
    fn clear_fault(&mut self);

    /// Disables PWM output, setting fault state; this can be used to stop all PWM from a timer in software detected faults
    fn set_fault(&mut self);
}

/// Exposes timer wide advanced features, such as [FaultMonitor](trait.FaultMonitor.html)
/// or future features like trigger outputs for synchronization with ADCs and other peripherals
pub struct PwmControl<TIM, FAULT> {
    _tim: PhantomData<TIM>,
    _fault: PhantomData<FAULT>,
}

/// Marker struct indicating that a PwmControl is in charge of fault monitoring
pub struct FaultEnabled;
/// Marker struct indicating that a PwmControl does not handle fault monitoring
pub struct FaultDisabled;

// automatically implement Pins trait for tuples of individual pins
macro_rules! pins_tuples {
    // Tuple of two pins
    ($(($CHA:ty, $CHB:ty)),*) => {
        $(
            impl<TIM, CHA, CHB, TA, TB> Pins<TIM, ($CHA, $CHB), (TA, TB)> for (CHA, CHB)
            where
                CHA: Pins<TIM, $CHA, TA>,
                CHB: Pins<TIM, $CHB, TB>,
            {
                type Channel = (Pwm<TIM, $CHA, TA, ActiveHigh, ActiveHigh>, Pwm<TIM, $CHB, TB, ActiveHigh, ActiveHigh>);
            }
        )*
    };
    // Tuple of three pins
    ($(($CHA:ty, $CHB:ty, $CHC:ty)),*) => {
        $(
            pins_tuples! {
                PERM3: ($CHA, $CHB, $CHC),
                PERM3: ($CHA, $CHC, $CHB),
                PERM3: ($CHB, $CHA, $CHC),
                PERM3: ($CHB, $CHC, $CHA),
                PERM3: ($CHC, $CHA, $CHB),
                PERM3: ($CHC, $CHB, $CHA)
            }
        )*
    };
    // Permutate tuple of three pins
    ($(PERM3: ($CHA:ty, $CHB:ty, $CHC:ty)),*) => {
        $(
            impl<TIM, CHA, CHB, CHC, TA, TB, TC> Pins<TIM, ($CHA, $CHB, $CHC), (TA, TB, TC)> for (CHA, CHB, CHC)
            where
                CHA: Pins<TIM, $CHA, TA>,
                CHB: Pins<TIM, $CHB, TB>,
                CHC: Pins<TIM, $CHC, TC>,
            {
                type Channel = (Pwm<TIM, $CHA, TA, ActiveHigh, ActiveHigh>, Pwm<TIM, $CHB, TB, ActiveHigh, ActiveHigh>, Pwm<TIM, $CHC, TC, ActiveHigh, ActiveHigh>);
            }
        )*
    };
    // Tuple of four pins (permutates the last 3, leaves 4th in place)
    ($(($CHD:ty, $CHA:ty, $CHB:ty, $CHC:ty)),*) => {
        $(
            pins_tuples! {
                PERM4: ($CHD, $CHA, $CHB, $CHC),
                PERM4: ($CHD, $CHA, $CHC, $CHB),
                PERM4: ($CHD, $CHB, $CHA, $CHC),
                PERM4: ($CHD, $CHB, $CHC, $CHA),
                PERM4: ($CHD, $CHC, $CHA, $CHB),
                PERM4: ($CHD, $CHC, $CHB, $CHA)
            }
        )*
    };
    // Tuple of four pins (permutates the last 3, leaves 1st in place)
    ($(PERM4: ($CHA:ty, $CHB:ty, $CHC:ty, $CHD:ty)),*) => {
        $(
            impl<TIM, CHA, CHB, CHC, CHD, TA, TB, TC, TD> Pins<TIM, ($CHA, $CHB, $CHC, $CHD), (TA, TB, TC, TD)> for (CHA, CHB, CHC, CHD)
            where
                CHA: Pins<TIM, $CHA, TA>,
                CHB: Pins<TIM, $CHB, TB>,
                CHC: Pins<TIM, $CHC, TC>,
                CHD: Pins<TIM, $CHD, TD>,
            {
                type Channel = (Pwm<TIM, $CHA, TA, ActiveHigh, ActiveHigh>, Pwm<TIM, $CHB, TB, ActiveHigh, ActiveHigh>, Pwm<TIM, $CHC, TC, ActiveHigh, ActiveHigh>, Pwm<TIM, $CHD, TD, ActiveHigh, ActiveHigh>);
            }
        )*
    }
}

pins_tuples! {
    (C1, C2),
    (C2, C1),
    (C1, C3),
    (C3, C1),
    (C1, C4),
    (C4, C1),
    (C2, C3),
    (C3, C2),
    (C2, C4),
    (C4, C2),
    (C3, C4),
    (C4, C3)
}

pins_tuples! {
    (C1, C2, C3),
    (C1, C2, C4),
    (C1, C3, C4),
    (C2, C3, C4)
}

pins_tuples! {
    (C1, C2, C3, C4),
    (C2, C1, C3, C4),
    (C3, C1, C2, C4),
    (C4, C1, C2, C3)
}

// Pin definitions, mark which pins can be used with which timers and channels
macro_rules! pins {
    // Single channel timer
    ($($TIMX:ty: OUT: [$($OUT:ty),*])+) => {
        $(
            $(
                impl Pins<$TIMX, C1, ComplementaryImpossible> for $OUT {
                    type Channel = Pwm<$TIMX, C1, ComplementaryImpossible, ActiveHigh, ActiveHigh>;
                }
            )*
        )+
    };
    // Dual channel timer $pm
    ($($TIMX:ty:
        CH1($COMP1:ty): [$($( #[ $pmeta1:meta ] )* $CH1:ty),*] CH2($COMP2:ty): [$($( #[ $pmeta2:meta ] )* $CH2:ty),*]
        CH1N: [$($( #[ $pmeta3:meta ] )* $CH1N:ty),*] CH2N: [$($( #[ $pmeta4:meta ] )* $CH2N:ty),*] BRK: [$($( #[ $pmeta5:meta ] )* $BRK:ty),*] BRK2: [$($( #[ $pmeta6:meta ] )* $BRK2:ty),*])+) => {
        $(
            $(
                $( #[ $pmeta1 ] )*
                impl Pins<$TIMX, C1, $COMP1> for $CH1 {
                    type Channel = Pwm<$TIMX, C1, $COMP1, ActiveHigh, ActiveHigh>;
                }
            )*
            $(
                $( #[ $pmeta2 ] )*
                impl Pins<$TIMX, C2, $COMP2> for $CH2 {
                    type Channel = Pwm<$TIMX, C2, $COMP2, ActiveHigh, ActiveHigh>;
                }
            )*
            $(
                $( #[ $pmeta3 ] )*
                impl NPins<$TIMX, C1> for $CH1N {}
            )*
            $(
                $( #[ $pmeta4 ] )*
                impl NPins<$TIMX, C2> for $CH2N {}
            )*
            $(
                $( #[ $pmeta5 ] )*
                impl FaultPins<$TIMX,> for $BRK {
                    const INPUT: BreakInput = BreakInput::BreakIn;
                }
            )*
            $(
                $( #[ $pmeta6 ] )*
                impl FaultPins<$TIMX> for $BRK2 {
                    const INPUT: BreakInput = BreakInput::BreakIn2;
                }
            )*
        )+
    };
    // Quad channel timers
    ($($TIMX:ty:
       CH1($COMP1:ty): [$($( #[ $pmeta1:meta ] )* $CH1:ty),*] CH2($COMP2:ty): [$($( #[ $pmeta2:meta ] )* $CH2:ty),*]
       CH3($COMP3:ty): [$($( #[ $pmeta3:meta ] )* $CH3:ty),*] CH4($COMP4:ty): [$($( #[ $pmeta4:meta ] )* $CH4:ty),*]
       CH1N: [$($( #[ $pmeta5:meta ] )* $CH1N:ty),*] CH2N: [$($( #[ $pmeta6:meta ] )* $CH2N:ty),*]
       CH3N: [$($( #[ $pmeta7:meta ] )* $CH3N:ty),*] CH4N: [$($( #[ $pmeta8:meta ] )* $CH4N:ty),*]
       BRK: [$($( #[ $pmeta9:meta ] )* $BRK:ty),*]
       BRK2: [$($( #[ $pmeta10:meta ] )* $BRK2:ty),*])+) => {
        $(
            $(
                $( #[ $pmeta1 ] )*
                impl Pins<$TIMX, C1, $COMP1> for $CH1 {
                    type Channel = Pwm<$TIMX, C1, $COMP1, ActiveHigh, ActiveHigh>;
                }
            )*
            $(
                $( #[ $pmeta2 ] )*
                impl Pins<$TIMX, C2, $COMP2> for $CH2 {
                    type Channel = Pwm<$TIMX, C2, $COMP2, ActiveHigh, ActiveHigh>;
                }
            )*
            $(
                $( #[ $pmeta3 ] )*
                impl Pins<$TIMX, C3, $COMP3> for $CH3 {
                    type Channel = Pwm<$TIMX, C3, $COMP3, ActiveHigh, ActiveHigh>;
                }
            )*
            $(
                $( #[ $pmeta4 ] )*
                impl Pins<$TIMX, C4, $COMP4> for $CH4 {
                    type Channel = Pwm<$TIMX, C4, $COMP4, ActiveHigh, ActiveHigh>;
                }
            )*
            $(
                $( #[ $pmeta5 ] )*
                impl NPins<$TIMX, C1> for $CH1N {}
            )*
            $(
                $( #[ $pmeta6 ] )*
                impl NPins<$TIMX, C2> for $CH2N {}
            )*
            $(
                $( #[ $pmeta7 ] )*
                impl NPins<$TIMX, C3> for $CH3N {}
            )*
            $(
                $( #[ $pmeta8 ] )*
                impl NPins<$TIMX, C4> for $CH4N {}
            )*
            $(
                $( #[ $pmeta9 ] )*
                impl FaultPins<$TIMX> for $BRK {
                    const INPUT: BreakInput = BreakInput::BreakIn;
                }
            )*
            $(
                $( #[ $pmeta10 ] )*
                impl FaultPins<$TIMX> for $BRK2 {
                    const INPUT: BreakInput = BreakInput::BreakIn2;
                }
            )*
        )+
    }
}
// Single channel timers
pins! {
    LPTIM1:
        OUT: [
            gpio::PD13<Alternate<1>>,
            gpio::PG13<Alternate<1>>
        ]
    LPTIM2:
        OUT: [
            gpio::PB13<Alternate<3>>
        ]
    LPTIM3:
        OUT: [
            gpio::PA1<Alternate<3>>
        ]
}
#[cfg(not(feature = "rm0455"))]
pins! {
    LPTIM4:
        OUT: [
            gpio::PA2<Alternate<3>>
        ]
    LPTIM5:
        OUT: [
            gpio::PA3<Alternate<3>>
        ]
}
// Dual channel timers
pins! {
    TIM12:
        CH1(ComplementaryImpossible): [
            gpio::PB14<Alternate<2>>,
            gpio::PH6<Alternate<2>>
        ]
        CH2(ComplementaryImpossible): [
            gpio::PB15<Alternate<2>>,
            gpio::PH9<Alternate<2>>
        ]
        CH1N: []
        CH2N: []
        BRK: []
        BRK2: []
    TIM13:
        CH1(ComplementaryImpossible): [
            gpio::PA6<Alternate<9>>,
            gpio::PF8<Alternate<9>>
        ]
        CH2(ComplementaryImpossible): []
        CH1N: []
        CH2N: []
        BRK: []
        BRK2: []
    TIM14:
        CH1(ComplementaryImpossible): [
            gpio::PA7<Alternate<9>>,
            gpio::PF9<Alternate<9>>
        ]
        CH2(ComplementaryImpossible): []
        CH1N: []
        CH2N: []
        BRK: []
        BRK2: []
    TIM15:
        CH1(ComplementaryDisabled): [
            gpio::PA2<Alternate<4>>,
            gpio::PE5<Alternate<4>>,
            #[cfg(any(feature = "rm0455", feature = "rm0468"))]
            gpio::PC12<Alternate<2>>
        ]
        CH2(ComplementaryImpossible): [
            gpio::PA3<Alternate<4>>,
            gpio::PE6<Alternate<4>>
        ]
        CH1N: [
            gpio::PA1<Alternate<4>>,
            gpio::PE4<Alternate<4>>
        ]
        CH2N: []
        BRK: [
            gpio::PA0<Alternate<4>>,
            #[cfg(any(feature = "rm0455", feature = "rm0468"))]
            gpio::PD2<Alternate<4>>,
            gpio::PE3<Alternate<4>>
        ]
        BRK2: []
    TIM16:
        CH1(ComplementaryDisabled): [
            gpio::PB8<Alternate<1>>,
            gpio::PF6<Alternate<1>>
        ]
        CH2(ComplementaryImpossible): []
        CH1N: [
            gpio::PB6<Alternate<1>>,
            gpio::PF8<Alternate<1>>
        ]
        CH2N: []
        BRK: [
            gpio::PB4<Alternate<1>>,
            gpio::PF10<Alternate<1>>
        ]
        BRK2: []
    TIM17:
        CH1(ComplementaryDisabled): [
            gpio::PB9<Alternate<1>>,
            gpio::PF7<Alternate<1>>
        ]
        CH2(ComplementaryImpossible): []
        CH1N: [
            gpio::PB7<Alternate<1>>,
            gpio::PF9<Alternate<1>>
        ]
        CH2N: []
        BRK: [
            gpio::PB5<Alternate<1>>,
            gpio::PG6<Alternate<1>>
        ]
        BRK2: []
}
// Quad channel timers
pins! {
    TIM1:
        CH1(ComplementaryDisabled): [
            gpio::PA8<Alternate<1>>,
            gpio::PE9<Alternate<1>>,
            #[cfg(not(feature = "stm32h7b0"))]
            gpio::PK1<Alternate<1>>
        ]
        CH2(ComplementaryDisabled): [
            gpio::PA9<Alternate<1>>,
            gpio::PE11<Alternate<1>>,
            #[cfg(not(feature = "stm32h7b0"))]
            gpio::PJ11<Alternate<1>>
        ]
        CH3(ComplementaryDisabled): [
            gpio::PA10<Alternate<1>>,
            gpio::PE13<Alternate<1>>,
            #[cfg(not(feature = "stm32h7b0"))]
            gpio::PJ9<Alternate<1>>
        ]
        CH4(ComplementaryImpossible): [
            gpio::PA11<Alternate<1>>,
            gpio::PE14<Alternate<1>>
        ]
        CH1N: [
            gpio::PA7<Alternate<1>>,
            gpio::PB13<Alternate<1>>,
            gpio::PE8<Alternate<1>>,
            #[cfg(not(feature = "stm32h7b0"))]
            gpio::PK0<Alternate<1>>
        ]
        CH2N: [
            gpio::PB0<Alternate<1>>,
            gpio::PB14<Alternate<1>>,
            gpio::PE10<Alternate<1>>,
            #[cfg(not(feature = "stm32h7b0"))]
            gpio::PJ10<Alternate<1>>
        ]
        CH3N: [
            gpio::PB1<Alternate<1>>,
            gpio::PB15<Alternate<1>>,
            gpio::PE12<Alternate<1>>,
            #[cfg(not(feature = "stm32h7b0"))]
            gpio::PJ8<Alternate<1>>
        ]
        CH4N: []
        BRK: [
            gpio::PA6<Alternate<1>>,
            gpio::PB12<Alternate<1>>,
            gpio::PE15<Alternate<1>>,
            #[cfg(not(feature = "stm32h7b0"))]
            gpio::PK2<Alternate<1>>
        ]
        BRK2: [
            gpio::PE6<Alternate<1>>,
            gpio::PG4<Alternate<1>>
        ]
    TIM2:
        CH1(ComplementaryImpossible): [
            gpio::PA0<Alternate<1>>,
            gpio::PA5<Alternate<1>>,
            gpio::PA15<Alternate<1>>
        ]
        CH2(ComplementaryImpossible): [
            gpio::PA1<Alternate<1>>,
            gpio::PB3<Alternate<1>>
        ]
        CH3(ComplementaryImpossible): [
            gpio::PA2<Alternate<1>>,
            gpio::PB10<Alternate<1>>
        ]
        CH4(ComplementaryImpossible): [
            gpio::PA3<Alternate<1>>,
            gpio::PB11<Alternate<1>>
        ]
        CH1N: []
        CH2N: []
        CH3N: []
        CH4N: []
        BRK: []
        BRK2: []
    TIM3:
        CH1(ComplementaryImpossible): [
            gpio::PA6<Alternate<2>>,
            gpio::PB4<Alternate<2>>,
            gpio::PC6<Alternate<2>>
        ]
        CH2(ComplementaryImpossible): [
            gpio::PA7<Alternate<2>>,
            gpio::PB5<Alternate<2>>,
            gpio::PC7<Alternate<2>>
        ]
        CH3(ComplementaryImpossible): [
            gpio::PB0<Alternate<2>>,
            gpio::PC8<Alternate<2>>
        ]
        CH4(ComplementaryImpossible): [
            gpio::PB1<Alternate<2>>,
            gpio::PC9<Alternate<2>>
        ]
        CH1N: []
        CH2N: []
        CH3N: []
        CH4N: []
        BRK: []
        BRK2: []
    TIM4:
        CH1(ComplementaryImpossible): [
            gpio::PB6<Alternate<2>>,
            gpio::PD12<Alternate<2>>
        ]
        CH2(ComplementaryImpossible): [
            gpio::PB7<Alternate<2>>,
            gpio::PD13<Alternate<2>>
        ]
        CH3(ComplementaryImpossible): [
            gpio::PB8<Alternate<2>>,
            gpio::PD14<Alternate<2>>
        ]
        CH4(ComplementaryImpossible): [
            gpio::PB9<Alternate<2>>,
            gpio::PD15<Alternate<2>>
        ]
        CH1N: []
        CH2N: []
        CH3N: []
        CH4N: []
        BRK: []
        BRK2: []
    TIM5:
        CH1(ComplementaryImpossible): [
            gpio::PA0<Alternate<2>>,
            gpio::PH10<Alternate<2>>
        ]
        CH2(ComplementaryImpossible): [
            gpio::PA1<Alternate<2>>,
            gpio::PH11<Alternate<2>>
        ]
        CH3(ComplementaryImpossible): [
            gpio::PA2<Alternate<2>>,
            gpio::PH12<Alternate<2>>
        ]
        CH4(ComplementaryImpossible): [
            gpio::PA3<Alternate<2>>,
            #[cfg(not(feature = "rm0468"))]
            gpio::PI0<Alternate<2>>
        ]
        CH1N: []
        CH2N: []
        CH3N: []
        CH4N: []
        BRK: []
        BRK2: []
    TIM8:
        CH1(ComplementaryDisabled): [
            gpio::PC6<Alternate<3>>,
            #[cfg(not(feature = "rm0468"))]
            gpio::PI5<Alternate<3>>,
            #[cfg(not(feature = "stm32h7b0"))]
            gpio::PJ8<Alternate<3>>
        ]
        CH2(ComplementaryDisabled): [
            gpio::PC7<Alternate<3>>,
            #[cfg(not(feature = "rm0468"))]
            gpio::PI6<Alternate<3>>,
            #[cfg(not(any(feature = "stm32h7b0", feature = "rm0468")))]
            gpio::PJ6<Alternate<3>>,
            #[cfg(not(feature = "stm32h7b0"))]
            gpio::PJ10<Alternate<3>>
        ]
        CH3(ComplementaryDisabled): [
            gpio::PC8<Alternate<3>>,
            #[cfg(not(feature = "rm0468"))]
            gpio::PI7<Alternate<3>>,
            #[cfg(not(feature = "stm32h7b0"))]
            gpio::PK0<Alternate<3>>
        ]
        CH4(ComplementaryImpossible): [
            gpio::PC9<Alternate<3>>,
            #[cfg(not(feature = "rm0468"))]
            gpio::PI2<Alternate<3>>
        ]
        CH1N: [
            gpio::PA5<Alternate<3>>,
            gpio::PA7<Alternate<3>>,
            gpio::PH13<Alternate<3>>,
            #[cfg(not(feature = "stm32h7b0"))]
            gpio::PJ9<Alternate<3>>
        ]
        CH2N: [
            gpio::PB0<Alternate<3>>,
            gpio::PB14<Alternate<3>>,
            gpio::PH14<Alternate<3>>,
            #[cfg(not(feature = "stm32h7b0"))]
            gpio::PJ7<Alternate<3>>,
            #[cfg(not(feature = "stm32h7b0"))]
            gpio::PJ11<Alternate<3>>
        ]
        CH3N: [
            gpio::PB1<Alternate<3>>,
            gpio::PB15<Alternate<3>>,
            gpio::PH15<Alternate<3>>,
            #[cfg(not(feature = "stm32h7b0"))]
            gpio::PK1<Alternate<3>>
        ]
        CH4N: []
        BRK: [
            gpio::PA6<Alternate<3>>,
            gpio::PG2<Alternate<3>>,
            #[cfg(not(feature = "rm0468"))]
            gpio::PI4<Alternate<3>>,
            #[cfg(not(feature = "stm32h7b0"))]
            gpio::PK2<Alternate<3>>
        ]
        BRK2: [
            gpio::PA8<Alternate<3>>,
            gpio::PG3<Alternate<3>>,
            #[cfg(not(feature = "rm0468"))]
            gpio::PI1<Alternate<3>>
        ]
}

// Period and prescaler calculator for 32-bit timers
// Returns (arr, psc)
fn calculate_frequency_32bit(
    base_freq: Hertz,
    freq: Hertz,
    alignment: Alignment,
) -> (u32, u16) {
    let divisor = if let Alignment::Center = alignment {
        freq.0 * 2
    } else {
        freq.0
    };

    // Round to the nearest period
    let arr = (base_freq.0 + (divisor >> 1)) / divisor - 1;

    (arr, 0)
}

// Period and prescaler calculator for 16-bit timers
// Returns (arr, psc)
// Returns as (u32, u16) to be compatible but arr will always be a valid u16
fn calculate_frequency_16bit(
    base_freq: Hertz,
    freq: Hertz,
    alignment: Alignment,
) -> (u32, u16) {
    let ideal_period =
        calculate_frequency_32bit(base_freq, freq, alignment).0 + 1;

    // Division factor is (PSC + 1)
    let prescale = (ideal_period - 1) / (1 << 16);

    // This will always fit in a 16-bit value because u32::MAX / (1 << 16) fits in a 16 bit

    // Round to the nearest period
    let period = (ideal_period + (prescale >> 1)) / (prescale + 1) - 1;

    // It should be impossible to fail these asserts
    assert!(period <= 0xFFFF);
    assert!(prescale <= 0xFFFF);

    (period, prescale as u16)
}

// Deadtime calculator helper function
// Returns (BDTR.DTG, CR1.CKD)
fn calculate_deadtime(base_freq: Hertz, deadtime: NanoSeconds) -> (u8, u8) {
    // tDTS is based on tCK_INT which is before the prescaler
    // It uses its own separate prescaler CR1.CKD

    // ticks = ns * GHz = ns * Hz / 1e9
    // Cortex-M7 has 32x32->64 multiply but no 64-bit divide
    // Divide by 100000 then 10000 by multiplying and shifting
    // This can't overflow because both values being multiplied are u32
    let deadtime_ticks = deadtime.0 as u64 * base_freq.0 as u64;
    // Make sure we won't overflow when multiplying; DTG is max 1008 ticks and CKD is max prescaler of 4
    // so deadtimes over 4032 ticks are impossible (4032*10^9 before dividing)
    assert!(deadtime_ticks <= 4_032_000_000_000u64);
    let deadtime_ticks = deadtime_ticks * 42950;
    let deadtime_ticks = (deadtime_ticks >> 32) as u32;
    let deadtime_ticks = deadtime_ticks as u64 * 429497;
    let deadtime_ticks = (deadtime_ticks >> 32) as u32;

    let deadtime_ticks = deadtime_ticks as u32;

    // Choose CR1 CKD divider of 1, 2, or 4 to determine tDTS
    let (deadtime_ticks, ckd) = match deadtime_ticks {
        t if t <= 1008 => (deadtime_ticks, 1),
        t if t <= 2016 => (deadtime_ticks / 2, 2),
        t if t <= 4032 => (deadtime_ticks / 4, 4),
        _ => {
            panic!("Deadtime must be less than 4032 ticks of timer base clock.")
        }
    };

    // Choose BDTR DTG bits to match deadtime_ticks
    // We want the smallest value of DTG that gives a deadtime >= the requested deadtime
    for dtg in 0..=255 {
        let actual_deadtime: u32 = match dtg {
            d if d < 128 => d,
            d if d < 192 => 2 * (64 + (d & 0x3F)),
            d if d < 224 => 8 * (32 + (d & 0x1F)),
            _ => 16 * (32 + (dtg & 0x1F)),
        };

        if actual_deadtime >= deadtime_ticks {
            return (dtg as u8, ckd);
        }
    }

    panic!("This should be unreachable.");
}

// PwmExt trait
/// Allows the pwm() method to be added to the peripheral register structs from the device crate
pub trait PwmExt: Sized {
    type Rec: ResetEnable;

    /// The requested frequency will be rounded to the nearest achievable frequency; the actual frequency may be higher or lower than requested.
    fn pwm<PINS, T, U, V>(
        self,
        _pins: PINS,
        frequency: T,
        prec: Self::Rec,
        clocks: &CoreClocks,
    ) -> PINS::Channel
    where
        PINS: Pins<Self, U, V>,
        T: Into<Hertz>;
}

pub trait PwmAdvExt<WIDTH>: Sized {
    type Rec: ResetEnable;

    fn pwm_advanced<PINS, CHANNEL, COMP>(
        self,
        _pins: PINS,
        prec: Self::Rec,
        clocks: &CoreClocks,
    ) -> PwmBuilder<Self, PINS, CHANNEL, FaultDisabled, COMP, WIDTH>
    where
        PINS: Pins<Self, CHANNEL, COMP>;
}

// Implement PwmExt trait for timer
macro_rules! pwm_ext_hal {
    ($TIMX:ident: $timX:ident, $Rec:ident) => {
        impl PwmExt for $TIMX {
            type Rec = rec::$Rec;

            fn pwm<PINS, T, U, V>(
                self,
                pins: PINS,
                frequency: T,
                prec: rec::$Rec,
                clocks: &CoreClocks,
            ) -> PINS::Channel
            where
                PINS: Pins<Self, U, V>,
                T: Into<Hertz>,
            {
                $timX(self, pins, frequency.into(), prec, clocks)
            }
        }
    };
}

// Implement PWM configuration for timer
macro_rules! tim_hal {
    ($($TIMX:ident: ($timX:ident, $Rec:ident,
                     $typ:ty, $bits:expr $(, DIR: $cms:ident)* $(, BDTR: $bdtr:ident, $moe_set:ident, $af1:ident, $bkinp_setting:ident $(, $bk2inp_setting:ident)*)*),)+) => {
        $(
            pwm_ext_hal!($TIMX: $timX, $Rec);

            /// Configures PWM
            fn $timX<PINS, T, U>(
                tim: $TIMX,
                _pins: PINS,
                freq: Hertz,
                prec: rec::$Rec,
                clocks: &CoreClocks,
            ) -> PINS::Channel
            where
                PINS: Pins<$TIMX, T, U>,
            {
                prec.enable().reset();

                let clk = $TIMX::get_clk(clocks)
                    .expect(concat!(stringify!($TIMX), ": Input clock not running!"));

                let (period, prescale) = match $bits {
                    16 => calculate_frequency_16bit(clk, freq, Alignment::Left),
                    _ => calculate_frequency_32bit(clk, freq, Alignment::Left),
                };

                // Write prescale
                tim.psc.write(|w| { w.psc().bits(prescale as u16) });

                // Write period
                tim.arr.write(|w| { w.arr().bits(period as $typ) });

                // BDTR: Advanced-control timers
                $(
                    // Set CCxP = OCxREF / CCxNP = !OCxREF
                    // Refer to RM0433 Rev 6 - Table 324.
                    tim.$bdtr.write(|w|
                                   w.moe().$moe_set()
                    );
                )*

                tim.cr1.write(|w|
                          w.cen().enabled()
                );

                unsafe { MaybeUninit::<PINS::Channel>::uninit().assume_init() }
            }

            impl PwmAdvExt<$typ> for $TIMX {
                type Rec = rec::$Rec;

                fn pwm_advanced<PINS, CHANNEL, COMP>(
                    self,
                    _pins: PINS,
                    prec: Self::Rec,
                    clocks: &CoreClocks,
                ) -> PwmBuilder<Self, PINS, CHANNEL, FaultDisabled, COMP, $typ>
                where
                    PINS: Pins<Self, CHANNEL, COMP>
                {
                    prec.enable().reset();

                    let clk = $TIMX::get_clk(clocks)
                        .expect(concat!(stringify!($TIMX), ": Input clock not running!"))
                        .0;

                    PwmBuilder {
                        _tim: PhantomData,
                        _pins: PhantomData,
                        _channel: PhantomData,
                        _fault: PhantomData,
                        _comp: PhantomData,
                        alignment: Alignment::Left,
                        base_freq: clk.hz(),
                        count: CountSettings::Explicit { period: 65535, prescaler: 0, },
                        bkin_enabled: false,
                        bkin2_enabled: false,
                        fault_polarity: Polarity::ActiveLow,
                        deadtime: 0.ns(),
                    }
                }
            }

            impl<PINS, CHANNEL, FAULT, COMP>
                PwmBuilder<$TIMX, PINS, CHANNEL, FAULT, COMP, $typ>
            where
                PINS: Pins<$TIMX, CHANNEL, COMP>,
            {
                pub fn finalize(self) -> (PwmControl<$TIMX, FAULT>, PINS::Channel) {
                    let tim = unsafe { &*$TIMX::ptr() };

                    let (period, prescaler) = match self.count {
                        CountSettings::Explicit { period, prescaler } => (period as u32, prescaler),
                        CountSettings::Frequency( freq ) => {
                            match $bits {
                                16 => calculate_frequency_16bit(self.base_freq, freq, self.alignment),
                                _ => calculate_frequency_32bit(self.base_freq, freq, self.alignment),
                            }
                        },
                    };

                    // Write prescaler
                    tim.psc.write(|w| w.psc().bits(prescaler as u16));

                    // Write period
                    tim.arr.write(|w| w.arr().bits(period as $typ));

                    $(
                        let (dtg, ckd) = calculate_deadtime(self.base_freq, self.deadtime);

                        match ckd {
                            1 => tim.cr1.modify(|_, w| w.ckd().div1()),
                            2 => tim.cr1.modify(|_, w| w.ckd().div2()),
                            4 => tim.cr1.modify(|_, w| w.ckd().div4()),
                            _ => panic!("Should be unreachable, invalid deadtime prescaler"),
                        }

                        let bkp = match self.fault_polarity {
                            Polarity::ActiveLow => false,
                            Polarity::ActiveHigh => true,
                        };

                        if self.bkin_enabled {
                            // BDTR:
                            //  BKF = 1 -> break pin filtering of 2 cycles of CK_INT (peripheral source clock)
                            //  AOE = 0 -> after a fault, master output enable MOE can only be set by software, not automatically
                            //  BKE = 1 -> break is enabled
                            //  BKP = 0 for active low, 1 for active high
                            // Safety: bkf is set to a constant value (1) that is a valid value for the field per the reference manual
                            unsafe { tim.$bdtr.write(|w| w.dtg().bits(dtg).bkf().bits(1).aoe().clear_bit().bke().set_bit().bkp().bit(bkp).moe().$moe_set()); }

                            // AF1:
                            //  BKINE = 1 -> break input enabled
                            //  BKINP should make input active high (BDTR BKP will set polarity), bit value varies timer to timer
                            tim.$af1.write(|w| w.bkine().set_bit().bkinp().$bkinp_setting());
                        }
                        $(
                            // Not all timers that have break inputs have break2 inputs
                            else if self.bkin2_enabled {
                                // BDTR:
                                //  BK2F = 1 -> break pin filtering of 2 cycles of CK_INT (peripheral source clock)
                                //  AOE = 0 -> after a fault, master output enable MOE can only be set by software, not automatically
                                //  BK2E = 1 -> break is enabled
                                //  BK2P = 0 for active low, 1 for active high
                                // Safety: bkf is set to a constant value (1) that is a valid value for the field per the reference manual
                                unsafe { tim.$bdtr.write(|w| w.dtg().bits(dtg).bk2f().bits(1).aoe().clear_bit().bk2e().set_bit().bk2p().bit(bkp).moe().$moe_set()); }

                                // AF1:
                                //  BKINE = 1 -> break input enabled
                                //  BKINP should make input active high (BDTR BKP will set polarity), bit value varies timer to timer
                                tim.af2.write(|w| w.bk2ine().set_bit().bk2inp().$bk2inp_setting());
                            }
                        )*
                        else {
                            // Safety: the DTG field of BDTR allows any 8-bit deadtime value and the dtg variable is u8
                            unsafe {
                                tim.$bdtr.write(|w| w.dtg().bits(dtg).aoe().clear_bit().moe().$moe_set());
                            }
                        }

                        // BDTR: Advanced-control timers
                        // Set CCxP = OCxREF / CCxNP = !OCxREF
                        // Refer to RM0433 Rev 6 - Table 324.
                        tim.$bdtr.modify(|_, w| w.moe().$moe_set());
                    )*


                    $(
                        match self.alignment {
                            Alignment::Left => { },
                            Alignment::Right => { tim.cr1.modify(|_, w| w.dir().down()); },
                            Alignment::Center => { tim.cr1.modify(|_, w| w.$cms().center_aligned3()); }
                        }
                    )*

                    tim.cr1.modify(|_, w| w.cen().enabled());

                    unsafe {
                        MaybeUninit::<(PwmControl<$TIMX, FAULT>, PINS::Channel)>::uninit()
                            .assume_init()
                    }
                }

                /// Set the PWM frequency; will overwrite the previous prescaler and period
                /// The requested frequency will be rounded to the nearest achievable frequency; the actual frequency may be higher or lower than requested.
                pub fn frequency<T: Into<Hertz>>(mut self, freq: T) -> Self {
                    self.count = CountSettings::Frequency( freq.into() );

                    self
                }

                /// Set the prescaler; PWM count runs at base_frequency/(prescaler+1)
                pub fn prescaler(mut self, prescaler: u16) -> Self {
                    let period = match self.count {
                        CountSettings::Frequency(_) => 65535,
                        CountSettings::Explicit { period, prescaler: _ } => period,
                    };

                    self.count = CountSettings::Explicit { period, prescaler };

                    self
                }

                /// Set the period; PWM count runs from 0 to period, repeating every (period+1) counts
                pub fn period(mut self, period: $typ) -> Self {
                    let prescaler = match self.count {
                        CountSettings::Frequency(_) => 0,
                        CountSettings::Explicit { period: _, prescaler } => prescaler,
                    };

                    self.count = CountSettings::Explicit { period, prescaler };

                    self
                }


                // Timers with complementary and deadtime and faults
                $(
                    /// Set the deadtime for complementary PWM channels of this timer
                    pub fn with_deadtime<T: Into<NanoSeconds>>(mut self, deadtime: T) -> Self {
                        // $bdtr is an Ident that only exists for timers with deadtime, so we can use it as a variable name to
                        // only implement this method for timers that support deadtime.
                        let $bdtr = deadtime.into();

                        self.deadtime = $bdtr;

                        self
                    }
                )*

                pub fn left_aligned( mut self ) -> Self {
                    self.alignment = Alignment::Left;

                    self
                }

                // Timers with advanced counting options, including center aligned and right aligned PWM
                $(
                    pub fn center_aligned( mut self ) -> Self {
                        // $cms is an Ident that only exists for timers with center/right aligned PWM, so we can use it as a variable name to
                        // only implement this method for timers that support center/right aligned PWM.
                        let $cms = Alignment::Center;

                        self.alignment = $cms;

                        self
                    }

                    pub fn right_aligned( mut self ) -> Self {
                        self.alignment = Alignment::Right;

                        self
                    }
                )*
            }

            // Timers with break/fault, dead time, and complimentary capabilities
            $(
                impl<PINS, CHANNEL, COMP> PwmBuilder<$TIMX, PINS, CHANNEL, FaultDisabled, COMP, $typ> {
                    /// Configure a break pin that will disable PWM when activated (active level based on polarity argument)
                    /// Note: not all timers have fault inputs; FaultPins<TIM> is only implemented for valid pins/timers.
                    pub fn with_break_pin<P: FaultPins<$TIMX>>(self, _pin: P, polarity: Polarity) -> PwmBuilder<$TIMX, PINS, CHANNEL, FaultEnabled, COMP, $typ> {
                        PwmBuilder {
                            _tim: PhantomData,
                            _pins: PhantomData,
                            _channel: PhantomData,
                            _fault: PhantomData,
                            _comp: PhantomData,
                            alignment: self.alignment,
                            base_freq: self.base_freq,
                            count: self.count,
                            bkin_enabled: self.bkin_enabled || P::INPUT == BreakInput::BreakIn,
                            bkin2_enabled: self.bkin2_enabled || P::INPUT == BreakInput::BreakIn2,
                            fault_polarity: polarity,
                            deadtime: self.deadtime,
                        }
                    }
                }

                impl FaultMonitor for PwmControl<$TIMX, FaultEnabled> {
                    fn is_fault_active(&self) -> bool {
                        let tim = unsafe { &*$TIMX::ptr() };

                        !tim.$bdtr.read().moe().bit()
                    }

                    fn clear_fault(&mut self) {
                        let tim = unsafe { &*$TIMX::ptr() };

                        tim.$bdtr.modify(|_, w| w.moe().set_bit());
                    }

                    fn set_fault(&mut self) {
                        let tim = unsafe { &*$TIMX::ptr() };

                        tim.$bdtr.modify(|_, w| w.moe().clear_bit());
                    }
                }
            )*
        )+
    }
}

tim_hal! {
    TIM1: (tim1, Tim1, u16, 16, DIR: cms, BDTR: bdtr, enabled, af1, clear_bit, clear_bit),
    TIM2: (tim2, Tim2, u32, 32, DIR: cms),
    TIM3: (tim3, Tim3, u16, 16, DIR: cms),
    TIM4: (tim4, Tim4, u16, 16, DIR: cms),
    TIM5: (tim5, Tim5, u32, 32, DIR: cms),
    TIM8: (tim8, Tim8, u16, 16, DIR: cms, BDTR: bdtr, enabled, af1, clear_bit, clear_bit),
}
tim_hal! {
    TIM12: (tim12, Tim12, u16, 16),
    TIM13: (tim13, Tim13, u16, 16),
    TIM14: (tim14, Tim14, u16, 16),
}
tim_hal! {
    TIM15: (tim15, Tim15, u16, 16, BDTR: bdtr, set_bit, af1, set_bit),
    TIM16: (tim16, Tim16, u16, 16, BDTR: bdtr, set_bit, tim16_af1, set_bit),
    TIM17: (tim17, Tim17, u16, 16, BDTR: bdtr, set_bit, tim17_af1, set_bit),
}

pub trait PwmPinEnable {
    fn ccer_enable(&mut self);
    fn ccer_disable(&mut self);
}

// Implement PwmPin for timer channels
macro_rules! tim_pin_hal {
    // Standard pins (no complementary functionality)
    ($($TIMX:ident:
       ($CH:ty, $ccxe:ident, $ccxp:ident, $ccmrx_output:ident, $ocxpe:ident, $ocxm:ident,
        $ccrx:ident, $typ:ident $(,$ccxne:ident, $ccxnp:ident)*),)+
    ) => {
        $(
            impl<COMP, POL, NPOL> hal::PwmPin for Pwm<$TIMX, $CH, COMP, POL, NPOL>
                where Pwm<$TIMX, $CH, COMP, POL, NPOL>: PwmPinEnable {
                type Duty = $typ;

                // You may not access self in the following methods!
                // See unsafe above

                fn disable(&mut self) {
                    self.ccer_disable();
                }

                fn enable(&mut self) {
                    let tim = unsafe { &*$TIMX::ptr() };

                    tim.$ccmrx_output().modify(|_, w|
                        w.$ocxpe()
                            .enabled() // Enable preload
                            .$ocxm()
                            .pwm_mode1() // PWM Mode
                    );

                    self.ccer_enable();
                }

                fn get_duty(&self) -> Self::Duty {
                    let tim = unsafe { &*$TIMX::ptr() };

                    tim.$ccrx.read().ccr().bits()
                }

                fn get_max_duty(&self) -> Self::Duty {
                    let tim = unsafe { &*$TIMX::ptr() };

                    let arr = tim.arr.read().arr().bits();

                    // One PWM cycle is ARR+1 counts long
                    // Valid PWM duty cycles are 0 to ARR+1
                    // However, if ARR is 65535 on a 16-bit timer, we can't add 1
                    // In that case, 100% duty cycle is not possible, only 65535/65536
                    if arr == Self::Duty::MAX {
                        arr
                    }
                    else {
                        arr + 1
                    }
                }

                fn set_duty(&mut self, duty: Self::Duty) {
                    let tim = unsafe { &*$TIMX::ptr() };

                    tim.$ccrx.write(|w| w.ccr().bits(duty));
                }
            }

            // Enable implementation for ComplementaryImpossible
            impl<POL, NPOL> PwmPinEnable for Pwm<$TIMX, $CH, ComplementaryImpossible, POL, NPOL> {
                fn ccer_enable(&mut self) {
                    let tim = unsafe { &*$TIMX::ptr() };

                    tim.ccer.modify(|_, w| w.$ccxe().set_bit());
                }
                fn ccer_disable(&mut self) {
                    let tim = unsafe { &*$TIMX::ptr() };

                    tim.ccer.modify(|_, w| w.$ccxe().clear_bit());
                }
            }

            impl<COMP, NPOL> Pwm<$TIMX, $CH, COMP, ActiveHigh, NPOL> {
                pub fn into_active_low(self) -> Pwm<$TIMX, $CH, COMP, ActiveLow, NPOL> {
                    let tim = unsafe { &*$TIMX::ptr() };

                    tim.ccer.modify(|_, w| w.$ccxp().set_bit());

                    Pwm {
                        _channel: PhantomData,
                        _tim: PhantomData,
                        _complementary: PhantomData,
                        _polarity: PhantomData,
                        _npolarity: PhantomData,
                    }
                }
            }

            impl<COMP, NPOL> Pwm<$TIMX, $CH, COMP, ActiveLow, NPOL> {
                pub fn into_active_high(self) -> Pwm<$TIMX, $CH, COMP, ActiveHigh, NPOL> {
                    let tim = unsafe { &*$TIMX::ptr() };

                    tim.ccer.modify(|_, w| w.$ccxp().clear_bit());

                    Pwm {
                        _channel: PhantomData,
                        _tim: PhantomData,
                        _complementary: PhantomData,
                        _polarity: PhantomData,
                        _npolarity: PhantomData,
                    }
                }
            }

            // Complementary channels
            $(
                // Enable implementation for ComplementaryDisabled
                impl<POL, NPOL> PwmPinEnable for Pwm<$TIMX, $CH, ComplementaryDisabled, POL, NPOL> {
                    fn ccer_enable(&mut self) {
                        let tim = unsafe { &*$TIMX::ptr() };

                        tim.ccer.modify(|_, w| w.$ccxe().set_bit());
                    }
                    fn ccer_disable(&mut self) {
                        let tim = unsafe { &*$TIMX::ptr() };

                        tim.ccer.modify(|_, w| w.$ccxe().clear_bit());
                    }
                }

                // Enable implementation for ComplementaryEnabled
                impl<POL, NPOL> PwmPinEnable for Pwm<$TIMX, $CH, ComplementaryEnabled, POL, NPOL> {
                    fn ccer_enable(&mut self) {
                        let tim = unsafe { &*$TIMX::ptr() };

                        tim.ccer.modify(|_, w| w.$ccxe().set_bit().$ccxne().set_bit());
                    }
                    fn ccer_disable(&mut self) {
                        let tim = unsafe { &*$TIMX::ptr() };

                        tim.ccer.modify(|_, w| w.$ccxe().clear_bit().$ccxne().clear_bit());
                    }
                }

                impl<POL, NPOL> Pwm<$TIMX, $CH, ComplementaryDisabled, POL, NPOL> {
                    pub fn into_complementary<NPIN>(self, _npin: NPIN) -> Pwm<$TIMX, $CH, ComplementaryEnabled, POL, NPOL>
                        where NPIN: NPins<$TIMX, $CH> {
                        // Make sure we aren't switching to complementary after we enable the channel
                        let tim = unsafe { &*$TIMX::ptr() };

                        let enabled = tim.ccer.read().$ccxe().bit();

                        assert!(!enabled);

                        Pwm {
                            _channel: PhantomData,
                            _tim: PhantomData,
                            _complementary: PhantomData,
                            _polarity: PhantomData,
                            _npolarity: PhantomData,
                        }
                    }
                }

                impl<POL> Pwm<$TIMX, $CH, ComplementaryEnabled, POL, ActiveHigh> {
                    pub fn into_comp_active_low(self) -> Pwm<$TIMX, $CH, ComplementaryEnabled, POL, ActiveLow> {
                        let tim = unsafe { &*$TIMX::ptr() };

                        tim.ccer.modify(|_, w| w.$ccxnp().set_bit());

                        Pwm {
                            _channel: PhantomData,
                            _tim: PhantomData,
                            _complementary: PhantomData,
                            _polarity: PhantomData,
                            _npolarity: PhantomData,
                        }
                    }
                }

                impl<POL> Pwm<$TIMX, $CH, ComplementaryEnabled, POL, ActiveLow> {
                    pub fn into_comp_active_high(self) -> Pwm<$TIMX, $CH, ComplementaryEnabled, POL, ActiveHigh> {
                        let tim = unsafe { &*$TIMX::ptr() };

                        tim.ccer.modify(|_, w| w.$ccxnp().clear_bit());

                        Pwm {
                            _channel: PhantomData,
                            _tim: PhantomData,
                            _complementary: PhantomData,
                            _polarity: PhantomData,
                            _npolarity: PhantomData,
                        }
                    }
                }
            )*
        )+
    };
}

// Dual channel timers
tim_pin_hal! {
    TIM12: (C1, cc1e, cc1p, ccmr1_output, oc1pe, oc1m, ccr1, u16),
    TIM12: (C2, cc2e, cc2p, ccmr1_output, oc2pe, oc2m, ccr2, u16),
}
tim_pin_hal! {
    TIM15: (C1, cc1e, cc1p, ccmr1_output, oc1pe, oc1m, ccr1, u16, cc1ne, cc1np),
}
// Channel 1 is complementary, channel 2 isn't
tim_pin_hal! {
    TIM15: (C2, cc2e, cc2p, ccmr1_output, oc2pe, oc2m, ccr2, u16),
}

// Single channel timers
tim_pin_hal! {
    TIM13: (C1, cc1e, cc1p, ccmr1_output, oc1pe, oc1m, ccr1, u16),
}
tim_pin_hal! {
    TIM14: (C1, cc1e, cc1p, ccmr1_output, oc1pe, oc1m, ccr1, u16),
}
tim_pin_hal! {
    TIM16: (C1, cc1e, cc1p, ccmr1_output, oc1pe, oc1m, ccr1, u16, cc1ne, cc1np),
}
tim_pin_hal! {
    TIM17: (C1, cc1e, cc1p, ccmr1_output, oc1pe, oc1m, ccr1, u16, cc1ne, cc1np),
}

// Quad channel timers
tim_pin_hal! {
    TIM1: (C1, cc1e, cc1p, ccmr1_output, oc1pe, oc1m, ccr1, u16, cc1ne, cc1np),
    TIM1: (C2, cc2e, cc2p, ccmr1_output, oc2pe, oc2m, ccr2, u16, cc2ne, cc2np),
    TIM1: (C3, cc3e, cc3p, ccmr2_output, oc3pe, oc3m, ccr3, u16, cc3ne, cc3np),
}
// Channels 1-3 are complementary, channel 4 isn't
tim_pin_hal! {
    TIM1: (C4, cc4e, cc4p, ccmr2_output, oc4pe, oc4m, ccr4, u16),
}
tim_pin_hal! {
    TIM2: (C1, cc1e, cc1p, ccmr1_output, oc1pe, oc1m, ccr1, u32),
    TIM2: (C2, cc2e, cc2p, ccmr1_output, oc2pe, oc2m, ccr2, u32),
    TIM2: (C3, cc3e, cc3p, ccmr2_output, oc3pe, oc3m, ccr3, u32),
    TIM2: (C4, cc4e, cc4p, ccmr2_output, oc4pe, oc4m, ccr4, u32),
}
tim_pin_hal! {
    TIM3: (C1, cc1e, cc1p, ccmr1_output, oc1pe, oc1m, ccr1, u16),
    TIM3: (C2, cc2e, cc2p, ccmr1_output, oc2pe, oc2m, ccr2, u16),
    TIM3: (C3, cc3e, cc3p, ccmr2_output, oc3pe, oc3m, ccr3, u16),
    TIM3: (C4, cc4e, cc4p, ccmr2_output, oc4pe, oc4m, ccr4, u16),
}
tim_pin_hal! {
    TIM4: (C1, cc1e, cc1p, ccmr1_output, oc1pe, oc1m, ccr1, u16),
    TIM4: (C2, cc2e, cc2p, ccmr1_output, oc2pe, oc2m, ccr2, u16),
    TIM4: (C3, cc3e, cc3p, ccmr2_output, oc3pe, oc3m, ccr3, u16),
    TIM4: (C4, cc4e, cc4p, ccmr2_output, oc4pe, oc4m, ccr4, u16),
}
tim_pin_hal! {
    TIM5: (C1, cc1e, cc1p, ccmr1_output, oc1pe, oc1m, ccr1, u32),
    TIM5: (C2, cc2e, cc2p, ccmr1_output, oc2pe, oc2m, ccr2, u32),
    TIM5: (C3, cc3e, cc3p, ccmr2_output, oc3pe, oc3m, ccr3, u32),
    TIM5: (C4, cc4e, cc4p, ccmr2_output, oc4pe, oc4m, ccr4, u32),
}
// Quad channel timers
tim_pin_hal! {
    TIM8: (C1, cc1e, cc1p, ccmr1_output, oc1pe, oc1m, ccr1, u16, cc1ne, cc1np),
    TIM8: (C2, cc2e, cc2p, ccmr1_output, oc2pe, oc2m, ccr2, u16, cc2ne, cc2np),
    TIM8: (C3, cc3e, cc3p, ccmr2_output, oc3pe, oc3m, ccr3, u16, cc3ne, cc3np),
}
// Channels 1-3 are complementary, channel 4 isn't
tim_pin_hal! {
    TIM8: (C4, cc4e, cc4p, ccmr2_output, oc4pe, oc4m, ccr4, u16),
}

// Low-power timers
macro_rules! lptim_hal {
    ($($TIMX:ident: ($timX:ident, $Rec:ident, $timXpac:ident),)+) => {
        $(
            pwm_ext_hal!($TIMX: $timX, $Rec);

            /// Configures PWM signal on the LPTIM OUT pin.
            fn $timX<PINS, T, U>(
                tim: $TIMX,
                _pins: PINS,
                freq: Hertz,
                prec: rec::$Rec,
                clocks: &CoreClocks,
            ) -> PINS::Channel
            where
                PINS: Pins<$TIMX, T, U>,
            {
                prec.enable().reset();

                let clk = $TIMX::get_clk(clocks)
                    .expect(concat!(stringify!($TIMX), ": Input clock not running!"))
                    .0;
                let freq = freq.0;
                let reload = clk / freq;
                assert!(reload < 128 * (1 << 16));

                // Calculate prescaler
                let (prescale, prescale_div) = match reload / (1 << 16) {
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
                let arr = reload / prescale_div;
                assert!(arr <= 0xFFFF);
                assert!(arr > 0);

                // CFGR
                tim.cfgr.modify(|_, w| w.presc().variant(prescale));

                // Enable
                tim.cr.modify(|_, w| w.enable().enabled());

                // Write ARR: LPTIM must be enabled
                tim.arr.write(|w| w.arr().bits(arr as u16));
                while !tim.isr.read().arrok().is_set() {}
                tim.icr.write(|w| w.arrokcf().clear());

                // PWM output is disabled by default, disable the
                // entire timer
                tim.cr.modify(|_, w| w.enable().disabled());

                unsafe { MaybeUninit::<PINS::Channel>::uninit().assume_init() }
            }

            impl hal::PwmPin for Pwm<$TIMX, C1, ComplementaryImpossible, ActiveHigh, ActiveHigh> {
                type Duty = u16;

                // You may not access self in the following methods!
                // See unsafe above

                fn disable(&mut self) {
                    let tim = unsafe { &*$TIMX::ptr() };

                    // LPTIM only has one output, so we disable the
                    // entire timer
                    tim.cr.modify(|_, w| w.enable().disabled());
                }

                fn enable(&mut self) {
                    let tim = unsafe { &*$TIMX::ptr() };

                    tim.cr.modify(|_, w| w.cntstrt().start().enable().enabled());
                }

                fn get_duty(&self) -> u16 {
                    let tim = unsafe { &*$TIMX::ptr() };

                    tim.cmp.read().cmp().bits()
                }

                fn get_max_duty(&self) -> u16 {
                    let tim = unsafe { &*$TIMX::ptr() };

                    tim.arr.read().arr().bits()
                }

                fn set_duty(&mut self, duty: u16) {
                    let tim = unsafe { &*$TIMX::ptr() };

                    tim.cmp.write(|w| w.cmp().bits(duty));
                    while !tim.isr.read().cmpok().is_set() {}
                    tim.icr.write(|w| w.cmpokcf().clear());
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
