//! Pulse width modulation
/// # Pulse width modulation
///
/// PWM output is avaliable for the advanced control timers (`TIM1`,
/// `TIM8`), the general purpose timers (`TIM[2-5]`, `TIM[12-17]`) and
/// the Low-power timers (`LPTIM[1-5]`).
///
/// Timers support up to 4 simultaneous PWM output channels
///
/// ## Usage
///
/// ```rust
/// let gpioa = ..; // Set up and split GPIOA
/// let pins = (
///     gpioa.pa8.into_alternate_af1(),
///     gpioa.pa9.into_alternate_af1(),
///     gpioa.pa10.into_alternate_af1(),
///     gpioa.pa11.into_alternate_af1(),
/// );
/// ```
///
/// Then call the `pwm` function on the corresponding timer:
///
/// ```
///   let device: pac::Peripherals = ..;
///
///   // Put the timer in PWM mode using the specified pins
///   // with a frequency of 100 hz.
///   let (c0, c1, c2, c3) = device.TIM1.pwm(
///       pins,
///       100.hz(),
///       &mut ccdr
///   );
///
///   // Set the duty cycle of channel 0 to 50%
///   c0.set_duty(c0.get_max_duty() / 2);
///   // PWM outputs are disabled by default
///   c0.enable()
/// ```
///
use core::marker::PhantomData;
use core::mem::MaybeUninit;

use crate::hal;
use crate::stm32::{lptim1, lptim3};
use crate::stm32::{LPTIM1, LPTIM2, LPTIM3, LPTIM4, LPTIM5};
use crate::stm32::{
    TIM1, TIM12, TIM13, TIM14, TIM15, TIM16, TIM17, TIM2, TIM3, TIM4, TIM5,
    TIM8,
};

use crate::rcc::{rec, CoreClocks, ResetEnable};
use crate::time::Hertz;
use crate::timer::GetClk;

use crate::gpio::gpioa::{
    PA0, PA1, PA10, PA11, PA15, PA2, PA3, PA5, PA6, PA7, PA8, PA9,
};
use crate::gpio::gpiob::{
    PB0, PB1, PB10, PB11, PB13, PB14, PB15, PB3, PB4, PB5, PB6, PB7, PB8, PB9,
};
use crate::gpio::gpioc::{PC6, PC7, PC8, PC9};
use crate::gpio::gpiod::{PD13, PD14, PD15};
use crate::gpio::gpioe::{PE11, PE13, PE14, PE5, PE6, PE9};
use crate::gpio::gpiof::{PF6, PF7, PF8, PF9};
use crate::gpio::gpiog::PG13;
use crate::gpio::gpioh::{PH10, PH11, PH12, PH6, PH9};
use crate::gpio::gpioi::{PI0, PI2, PI5, PI6, PI7};
use crate::gpio::gpioj::{PJ10, PJ11, PJ6, PJ8, PJ9};
use crate::gpio::gpiok::{PK0, PK1};

use crate::gpio::{Alternate, AF1, AF2, AF3, AF4, AF9};

pub trait Pins<TIM> {
    type Channel;
}

pub struct C1;
pub struct C2;
pub struct C3;
pub struct C4;

pub struct Pwm<TIM, CHANNEL> {
    _channel: PhantomData<CHANNEL>,
    _tim: PhantomData<TIM>,
}

impl<TIM, CH1, CH2> Pins<TIM> for (CH1, CH2)
where
    CH1: Pins<TIM>,
    CH2: Pins<TIM>,
{
    type Channel = (Pwm<TIM, C1>, Pwm<TIM, C2>);
}
impl<TIM, CH1, CH2, CH3> Pins<TIM> for (CH1, CH2, CH3)
where
    CH1: Pins<TIM>,
    CH2: Pins<TIM>,
    CH3: Pins<TIM>,
{
    type Channel = (Pwm<TIM, C1>, Pwm<TIM, C2>, Pwm<TIM, C3>);
}
impl<TIM, CH1, CH2, CH3, CH4> Pins<TIM> for (CH1, CH2, CH3, CH4)
where
    CH1: Pins<TIM>,
    CH2: Pins<TIM>,
    CH3: Pins<TIM>,
    CH4: Pins<TIM>,
{
    type Channel = (Pwm<TIM, C1>, Pwm<TIM, C2>, Pwm<TIM, C3>, Pwm<TIM, C4>);
}

// Pin definitions
macro_rules! pins {
    // Single channel timer
    ($($TIMX:ty: OUT: [$($OUT:ty),*])+) => {
        $(
            $(
                impl Pins<$TIMX> for $OUT {
                    type Channel = Pwm<$TIMX, C1>;
                }
            )*
        )+
    };
    // Dual channel timer
    ($($TIMX:ty: CH1: [$($CH1:ty),*] CH2: [$($CH2:ty),*]
       CH1N: [$($CH1N:ty),*] CH2N: [$($CH2N:ty),*])+) => {
        $(
            $(
                impl Pins<$TIMX> for $CH1 {
                    type Channel = Pwm<$TIMX, C1>;
                }
            )*
            $(
                impl Pins<$TIMX> for $CH2 {
                    type Channel = Pwm<$TIMX, C2>;
                }
            )*
        )+
    };
    // Quad channel timers
    ($($TIMX:ty: CH1: [$($CH1:ty),*] CH2: [$($CH2:ty),*] CH3: [$($CH3:ty),*] CH4: [$($CH4:ty),*]
       CH1N: [$($CH1N:ty),*] CH2N: [$($CH2N:ty),*] CH3N: [$($CH3N:ty),*] CH4N: [$($CH4N:ty),*])+) => {
        $(
            $(
                impl Pins<$TIMX> for $CH1 {
                    type Channel = Pwm<$TIMX, C1>;
                }
            )*
            $(
                impl Pins<$TIMX> for $CH2 {
                    type Channel = Pwm<$TIMX, C2>;
                }
            )*
            $(
                impl Pins<$TIMX> for $CH3 {
                    type Channel = Pwm<$TIMX, C3>;
                }
            )*
            $(
                impl Pins<$TIMX> for $CH4 {
                    type Channel = Pwm<$TIMX, C4>;
                }
            )*
        )+
    }
}
// Single channel timers
pins! {
    LPTIM1:
        OUT: [
            PD13<Alternate<AF1>>,
            PG13<Alternate<AF1>>
        ]
    LPTIM2:
        OUT: [
            PB13<Alternate<AF3>>
        ]
    LPTIM3:
        OUT: [
            PA1<Alternate<AF3>>
        ]
    LPTIM4:
        OUT: [
            PA2<Alternate<AF3>>
        ]
    LPTIM5:
        OUT: [
            PA3<Alternate<AF3>>
        ]
}
// Dual channel timers
pins! {
    TIM12:
        CH1: [
            PB14<Alternate<AF2>>,
            PH6<Alternate<AF2>>
        ]
        CH2: [
            PB15<Alternate<AF2>>,
            PH9<Alternate<AF2>>
        ]
        CH1N: []
        CH2N: []
    TIM13:
        CH1: [
            PA6<Alternate<AF9>>,
            PF8<Alternate<AF9>>
        ]
        CH2: []
        CH1N: []
        CH2N: []
    TIM14:
        CH1: [
            PA7<Alternate<AF9>>,
            PF9<Alternate<AF9>>
        ]
        CH2: []
        CH1N: []
        CH2N: []
    TIM15:
        CH1: [
            PA2<Alternate<AF4>>,
            PE5<Alternate<AF4>>
        ]
        CH2: [
            PA3<Alternate<AF4>>,
            PE6<Alternate<AF4>>
        ]
        CH1N: [
            PA1<Alternate<AF4>>,
            PE4<Alternate<AF4>>
        ]
        CH2N: []
}
// More single channel timers
pins! {
    TIM16:
        CH1: [
            PB8<Alternate<AF1>>,
            PF6<Alternate<AF1>>
        ]
        CH2: []
        CH1N: [
            PB6<Alternate<AF1>>,
            PF8<Alternate<AF1>>
        ]
        CH2N: []
    TIM17:
        CH1: [
            PB9<Alternate<AF1>>,
            PF7<Alternate<AF1>>
        ]
        CH2: []
        CH1N: [
            PB7<Alternate<AF1>>,
            PF9<Alternate<AF1>>
        ]
        CH2N: []
}
// Quad channel timers
pins! {
    TIM1:
        CH1: [
            PA8<Alternate<AF1>>,
            PE9<Alternate<AF1>>,
            PK1<Alternate<AF1>>
        ]
        CH2: [
            PA9<Alternate<AF1>>,
            PE11<Alternate<AF1>>,
            PJ11<Alternate<AF1>>
        ]
        CH3: [
            PA10<Alternate<AF1>>,
            PE13<Alternate<AF1>>,
            PJ9<Alternate<AF1>>
        ]
        CH4: [
            PA11<Alternate<AF1>>,
            PE14<Alternate<AF1>>
        ]
        CH1N: [
            PA7<Alternate<AF1>>,
            PB13<Alternate<AF1>>,
            PE8<Alternate<AF1>>,
            PK0<Alternate<AF1>>
        ]
        CH2N: [
            PB0<Alternate<AF1>>,
            PB14<Alternate<AF1>>,
            PE10<Alternate<AF1>>,
            PJ10<Alternate<AF1>>
        ]
        CH3N: [
            PB1<Alternate<AF1>>,
            PB15<Alternate<AF1>>,
            PE12<Alternate<AF1>>,
            PJ8<Alternate<AF1>>
        ]
        CH4N: []
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
        CH3: [
            PA2<Alternate<AF1>>,
            PB10<Alternate<AF1>>
        ]
        CH4: [
            PA3<Alternate<AF1>>,
            PB11<Alternate<AF1>>
        ]
        CH1N: []
        CH2N: []
        CH3N: []
        CH4N: []
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
        CH3: [
            PB0<Alternate<AF2>>,
            PC8<Alternate<AF2>>
        ]
        CH4: [
            PB1<Alternate<AF2>>,
            PC9<Alternate<AF2>>
        ]
        CH1N: []
        CH2N: []
        CH3N: []
        CH4N: []
    TIM4:
        CH1: [
            PB6<Alternate<AF2>>
        ]
        CH2: [
            PB7<Alternate<AF2>>,
            PD13<Alternate<AF2>>
        ]
        CH3: [
            PB8<Alternate<AF2>>,
            PD14<Alternate<AF2>>
        ]
        CH4: [
            PB9<Alternate<AF2>>,
            PD15<Alternate<AF2>>
        ]
        CH1N: []
        CH2N: []
        CH3N: []
        CH4N: []
    TIM5:
        CH1: [
            PA0<Alternate<AF2>>,
            PH10<Alternate<AF2>>
        ]
        CH2: [
            PA1<Alternate<AF2>>,
            PH11<Alternate<AF2>>
        ]
        CH3: [
            PA2<Alternate<AF2>>,
            PH12<Alternate<AF2>>
        ]
        CH4: [
            PA3<Alternate<AF2>>,
            PI0<Alternate<AF2>>
        ]
        CH1N: []
        CH2N: []
        CH3N: []
        CH4N: []
    TIM8:
        CH1: [
            PC6<Alternate<AF3>>,
            PI5<Alternate<AF3>>,
            PJ8<Alternate<AF3>>
        ]
        CH2: [
            PC7<Alternate<AF3>>,
            PI6<Alternate<AF3>>,
            PJ6<Alternate<AF3>>,
            PJ10<Alternate<AF3>>
        ]
        CH3: [
            PC8<Alternate<AF3>>,
            PI7<Alternate<AF3>>,
            PK0<Alternate<AF3>>
        ]
        CH4: [
            PC9<Alternate<AF3>>,
            PI2<Alternate<AF3>>
        ]
        CH1N: [
            PA5<Alternate<AF3>>,
            PA7<Alternate<AF3>>,
            PH13<Alternate<AF3>>,
            PJ9<Alternate<AF3>>
        ]
        CH2N: [
            PB0<Alternate<AF3>>,
            PB14<Alternate<AF3>>,
            PH14<Alternate<AF3>>,
            PJ7<Alternate<AF3>>,
            PJ11<Alternate<AF3>>
        ]
        CH3N: [
            PB1<Alternate<AF3>>,
            PB15<Alternate<AF3>>,
            PH15<Alternate<AF3>>,
            PK1<Alternate<AF3>>
        ]
        CH4N: []
}

// PwmExt trait
pub trait PwmExt: Sized {
    type Rec: ResetEnable;

    fn pwm<PINS, T>(
        self,
        _pins: PINS,
        frequency: T,
        prec: Self::Rec,
        clocks: &CoreClocks,
    ) -> PINS::Channel
    where
        PINS: Pins<Self>,
        T: Into<Hertz>;
}

// Implement PwmExt trait for timer
macro_rules! pwm_ext_hal {
    ($TIMX:ident: $timX:ident, $Rec:ident) => {
        impl PwmExt for $TIMX {
            type Rec = rec::$Rec;

            fn pwm<PINS, T>(
                self,
                pins: PINS,
                frequency: T,
                prec: rec::$Rec,
                clocks: &CoreClocks,
            ) -> PINS::Channel
            where
                PINS: Pins<Self>,
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
                     $typ:ty, $bits:expr $(,$bdtr:ident)*),)+) => {
        $(
            pwm_ext_hal!($TIMX: $timX, $Rec);

            /// Configures PWM
            fn $timX<PINS>(
                tim: $TIMX,
                _pins: PINS,
                freq: Hertz,
                prec: rec::$Rec,
                clocks: &CoreClocks,
            ) -> PINS::Channel
            where
                PINS: Pins<$TIMX>,
            {
                prec.enable().reset();

                let clk = $TIMX::get_clk(clocks)
                    .expect("Timer input clock not running!").0;
                let freq = freq.0;
                let reload : u32 = clk / freq; // u32

                let prescale = match $bits {
                    16 => {
                        // Division factor is (PSC + 1)
                        let prescale = (reload - 1) / (1 << 16);
                        assert!(prescale <= 0xFFFF);
                        prescale
                    },
                    _ => 0      // No prescale required for 32-bit timer
                };

                // Write prescale
                tim.psc.write(|w| { w.psc().bits(prescale as u16) });

                // Set TOP value
                let top = reload / (prescale + 1);
                tim.arr.write(|w| { w.arr().bits(top as $typ) });

                // BDTR: Advanced-control timers
                $(
                    // Set CCxP = OCxREF / CCxNP = !OCxREF
                    // Refer to RM0433 Rev 6 - Table 324.
                    tim.$bdtr.write(|w|
                                   w.moe().enabled()
                    );
                )*

                tim.cr1.write(|w|
                          w.cen().enabled()
                );

                unsafe { MaybeUninit::<PINS::Channel>::uninit().assume_init() }
            }
        )+
    }
}
tim_hal! {
    TIM1: (tim1, Tim1, u16, 16, bdtr),
    TIM2: (tim2, Tim2, u32, 32),
    TIM3: (tim3, Tim3, u16, 16),
    TIM4: (tim4, Tim4, u16, 16),
    TIM5: (tim5, Tim5, u32, 32),
    TIM8: (tim8, Tim8, u16, 16, bdtr),
}
tim_hal! {
    TIM12: (tim12, Tim12, u16, 16),
    TIM13: (tim13, Tim13, u16, 16),
    TIM14: (tim14, Tim14, u16, 16),
}
tim_hal! {
    TIM15: (tim15, Tim15, u16, 16),
    TIM16: (tim16, Tim16, u16, 16),
    TIM17: (tim17, Tim17, u16, 16),
}

// Implement PwmPin for timer
macro_rules! tim_pin_hal {
    ($($TIMX:ident:
       ($CH:ty, $ccxe:ident, $ccmrx_output:ident, $ocxpe:ident, $ocxm:ident,
        $ccrx:ident, $typ:ident),)+
    ) => {
        $(
            impl hal::PwmPin for Pwm<$TIMX, $CH> {
                type Duty = $typ;

                // You may not access self in the following methods!
                // See unsafe above

                fn disable(&mut self) {
                    let tim = unsafe { &*$TIMX::ptr() };

                    tim.ccer.modify(|_, w| w.$ccxe().clear_bit());
                }

                fn enable(&mut self) {
                    let tim = unsafe { &*$TIMX::ptr() };

                    tim.$ccmrx_output().modify(|_, w|
                        w.$ocxpe()
                            .enabled() // Enable preload
                            .$ocxm()
                            .pwm_mode1() // PWM Mode
                    );
                    tim.ccer.modify(|_, w| w.$ccxe().set_bit());
                }

                fn get_duty(&self) -> Self::Duty {
                    let tim = unsafe { &*$TIMX::ptr() };

                    tim.$ccrx.read().ccr().bits()
                }

                fn get_max_duty(&self) -> Self::Duty {
                    let tim = unsafe { &*$TIMX::ptr() };

                    tim.arr.read().arr().bits()
                }

                fn set_duty(&mut self, duty: Self::Duty) {
                    let tim = unsafe { &*$TIMX::ptr() };

                    tim.$ccrx.write(|w| w.ccr().bits(duty));
                }
            }
        )+
    };
}
tim_pin_hal! {
    TIM1: (C1, cc1e, ccmr1_output, oc1pe, oc1m, ccr1, u16),
    TIM1: (C2, cc2e, ccmr1_output, oc2pe, oc2m, ccr2, u16),
    TIM1: (C3, cc3e, ccmr2_output, oc3pe, oc3m, ccr3, u16),
    TIM1: (C4, cc4e, ccmr2_output, oc4pe, oc4m, ccr4, u16),
}
tim_pin_hal! {
    TIM2: (C1, cc1e, ccmr1_output, oc1pe, oc1m, ccr1, u32),
    TIM2: (C2, cc2e, ccmr1_output, oc2pe, oc2m, ccr2, u32),
    TIM2: (C3, cc3e, ccmr2_output, oc3pe, oc3m, ccr3, u32),
    TIM2: (C4, cc4e, ccmr2_output, oc4pe, oc4m, ccr4, u32),
}
tim_pin_hal! {
    TIM3: (C1, cc1e, ccmr1_output, oc1pe, oc1m, ccr1, u16),
    TIM3: (C2, cc2e, ccmr1_output, oc2pe, oc2m, ccr2, u16),
    TIM3: (C3, cc3e, ccmr2_output, oc3pe, oc3m, ccr3, u16),
    TIM3: (C4, cc4e, ccmr2_output, oc4pe, oc4m, ccr4, u16),
}
tim_pin_hal! {
    TIM4: (C1, cc1e, ccmr1_output, oc1pe, oc1m, ccr1, u16),
    TIM4: (C2, cc2e, ccmr1_output, oc2pe, oc2m, ccr2, u16),
    TIM4: (C3, cc3e, ccmr2_output, oc3pe, oc3m, ccr3, u16),
    TIM4: (C4, cc4e, ccmr2_output, oc4pe, oc4m, ccr4, u16),
}
tim_pin_hal! {
    TIM5: (C1, cc1e, ccmr1_output, oc1pe, oc1m, ccr1, u32),
    TIM5: (C2, cc2e, ccmr1_output, oc2pe, oc2m, ccr2, u32),
    TIM5: (C3, cc3e, ccmr2_output, oc3pe, oc3m, ccr3, u32),
    TIM5: (C4, cc4e, ccmr2_output, oc4pe, oc4m, ccr4, u32),
}
tim_pin_hal! {
    TIM8: (C1, cc1e, ccmr1_output, oc1pe, oc1m, ccr1, u16),
    TIM8: (C2, cc2e, ccmr1_output, oc2pe, oc2m, ccr2, u16),
    TIM8: (C3, cc3e, ccmr2_output, oc3pe, oc3m, ccr3, u16),
    TIM8: (C4, cc4e, ccmr2_output, oc4pe, oc4m, ccr4, u16),
}

// Low-power timers
macro_rules! lptim_hal {
    ($($TIMX:ident: ($timX:ident, $Rec:ident, $timXpac:ident),)+) => {
        $(
            pwm_ext_hal!($TIMX: $timX, $Rec);

            /// Configures PWM signal on the LPTIM OUT pin.
            fn $timX<PINS>(
                tim: $TIMX,
                _pins: PINS,
                freq: Hertz,
                prec: rec::$Rec,
                clocks: &CoreClocks,
            ) -> PINS::Channel
            where
                PINS: Pins<$TIMX>,
            {
                prec.enable().reset();

                let clk = $TIMX::get_clk(clocks).unwrap().0;
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

            impl hal::PwmPin for Pwm<$TIMX, C1> {
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
    LPTIM4: (lptim4, Lptim4, lptim3),
    LPTIM5: (lptim5, Lptim5, lptim3),
}
