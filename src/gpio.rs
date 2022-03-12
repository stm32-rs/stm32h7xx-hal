//! General Purpose Input / Output
//!
//! The GPIO pins are organised into groups of 16 pins which can be accessed through the
//! `gpioa`, `gpiob`... modules. To get access to the pins, you first need to convert them into a
//! HAL designed struct from the `pac` struct using the [split](trait.GpioExt.html#tymethod.split) function.
//! ```rust
//! // Acquire the GPIOC peripheral
//! // NOTE: `dp` is the device peripherals from the `PAC` crate
//! let mut gpioa = dp.GPIOA.split();
//! ```
//!
//! This gives you a struct containing all the pins `px0..px15`.
//! By default pins are in floating input mode. You can change their modes.
//! For example, to set `pa5` high, you would call
//!
//! ```rust
//! let output = gpioa.pa5.into_push_pull_output();
//! output.set_high();
//! ```
//!
//! ## Modes
//!
//! Each GPIO pin can be set to various modes:
//!
//! - **Alternate**: Pin mode required when the pin is driven by other peripherals
//! - **Analog**: Analog input to be used with ADC.
//! - **Dynamic**: Pin mode is selected at runtime. See changing configurations for more details
//! - Input
//!     - **PullUp**: Input connected to high with a weak pull up resistor. Will be high when nothing
//!     is connected
//!     - **PullDown**: Input connected to high with a weak pull up resistor. Will be low when nothing
//!     is connected
//!     - **Floating**: Input not pulled to high or low. Will be undefined when nothing is connected
//! - Output
//!     - **PushPull**: Output which either drives the pin high or low
//!     - **OpenDrain**: Output which leaves the gate floating, or pulls it do ground in drain
//!     mode. Can be used as an input in the `open` configuration
//!
//! ## Changing modes
//! The simplest way to change the pin mode is to use the `into_<mode>` functions. These return a
//! new struct with the correct mode that you can use the input or output functions on.
//!
//! If you need a more temporary mode change, and can not use the `into_<mode>` functions for
//! ownership reasons, you can use the closure based `with_<mode>` functions to temporarily change the pin type, do
//! some output or input, and then have it change back once done.
//!
//! ### Dynamic Mode Change
//! The above mode change methods guarantee that you can only call input functions when the pin is
//! in input mode, and output when in output modes, but can lead to some issues. Therefore, there
//! is also a mode where the state is kept track of at runtime, allowing you to change the mode
//! often, and without problems with ownership, or references, at the cost of some performance and
//! the risk of runtime errors.
//!
//! To make a pin dynamic, use the `into_dynamic` function, and then use the `make_<mode>` functions to
//! change the mode
//!
//! # Examples
//!
//! - [Simple Blinky](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/blinky.rs)
//! - [Digital Read](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/digital_read.rs)
//! - [External Interrupt via Button](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/exti_interrupt.rs)

use core::marker::PhantomData;

use crate::pac::{EXTI, SYSCFG};
use crate::rcc::ResetEnable;

mod convert;
mod partially_erased;
pub use partially_erased::{PEPin, PartiallyErasedPin};
mod erased;
pub use erased::{EPin, ErasedPin};
mod dynamic;
pub use dynamic::{Dynamic, DynamicPin};
mod hal_02;

pub use embedded_hal::digital::v2::PinState;

use core::fmt;

/// A filler pin type
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct NoPin;

/// Extension trait to split a GPIO peripheral into independent pins and
/// registers
pub trait GpioExt {
    /// The parts to split the GPIO into
    type Parts;

    /// The Reset and Enable control block for this GPIO block
    type Rec: ResetEnable;

    /// Takes the GPIO peripheral and splits it into Zero-Sized Types
    /// (ZSTs) representing individual pins. These are public
    /// members of the return type.
    ///
    /// ```
    /// let device_peripherals = stm32::Peripherals.take().unwrap();
    /// let ccdr = ...; // From RCC
    ///
    /// let gpioa = device_peripherals.GPIOA.split(ccdr.peripheral.GPIOA);
    ///
    /// let pa0 = gpioa.pa0; // Pin 0
    /// ```
    fn split(self, prec: Self::Rec) -> Self::Parts;

    /// As [split](GpioExt#tymethod.split), but does not reset the GPIO
    /// peripheral in the RCC_AHB4RSTR register. However it still enables the
    /// peripheral in RCC_AHB4ENR, so our accesses to the peripheral memory will
    /// always be valid.
    ///
    /// This is useful for situations where some GPIO functionality
    /// was already activated outside the HAL in early startup code
    /// or a bootloader. That might be needed for watchdogs, clock
    /// circuits, or executing from an external memory. In this
    /// case, `split_without_reset` allows this GPIO HAL to be used
    /// without generating unwanted edges on already initialised
    /// pins.
    ///
    /// However, the user takes responsibility that the GPIO
    /// peripheral is in a valid state already. Note that the
    /// registers accessed and written by this HAL may change in any
    /// patch revision.
    fn split_without_reset(self, prec: Self::Rec) -> Self::Parts;
}

pub trait PinExt {
    type Mode;
    /// Return pin number
    fn pin_id(&self) -> u8;
    /// Return port number
    fn port_id(&self) -> u8;
}

/// Some alternate mode (type state)
pub struct Alternate<const A: u8, Otype = PushPull>(PhantomData<Otype>);

/// Input mode (type state)
pub struct Input<MODE = Floating> {
    _mode: PhantomData<MODE>,
}

/// Floating input (type state)
pub struct Floating;

/// Pulled down input (type state)
pub struct PullDown;

/// Pulled up input (type state)
pub struct PullUp;

/// Open drain input or output (type state)
pub struct OpenDrain;

/// Output mode (type state)
pub struct Output<MODE = PushPull> {
    _mode: PhantomData<MODE>,
}

/// Push pull output (type state)
pub struct PushPull;

/// Analog mode (type state)
pub struct Analog;

pub type Debugger = Alternate<0, PushPull>;

/// GPIO Pin speed selection
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum Speed {
    Low = 0,
    Medium = 1,
    High = 2,
    VeryHigh = 3,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum Edge {
    Rising,
    Falling,
    RisingFalling,
}

macro_rules! af {
    ($($AF:ident: $i:literal;)+) => {
        $(
            #[doc="Alternate function "]
            #[doc=stringify!($i)]
            #[deprecated(since = "0.12.0")]
            pub const $AF: u8 = $i;
        )+
    }
}

af! {
    AF0: 0; AF1: 1; AF2: 2; AF3: 3; AF4: 4; AF5: 5;
    AF6: 6; AF7: 7; AF8: 8; AF9: 9; AF10: 10;
    AF11: 11; AF12: 12; AF13: 13; AF14: 14; AF15: 15;
}

mod sealed {
    /// Marker trait that show if `ExtiPin` can be implemented
    pub trait Interruptable {}
}

use sealed::Interruptable;
impl<MODE> Interruptable for Output<MODE> {}
impl<MODE> Interruptable for Input<MODE> {}

/// External Interrupt Pin
pub trait ExtiPin {
    fn make_interrupt_source(&mut self, syscfg: &mut SYSCFG);
    fn trigger_on_edge(&mut self, exti: &mut EXTI, level: Edge);
    fn enable_interrupt(&mut self, exti: &mut EXTI);
    fn disable_interrupt(&mut self, exti: &mut EXTI);
    fn clear_interrupt_pending_bit(&mut self);
    fn check_interrupt(&self) -> bool;
}

impl<PIN> ExtiPin for PIN
where
    PIN: PinExt,
    PIN::Mode: Interruptable,
{
    /// Make corresponding EXTI line sensitive to this pin
    #[inline(always)]
    fn make_interrupt_source(&mut self, syscfg: &mut SYSCFG) {
        let i = self.pin_id();
        let port = self.port_id() as u32;
        let offset = 4 * (i % 4);
        match i {
            0..=3 => {
                syscfg.exticr1.modify(|r, w| unsafe {
                    w.bits((r.bits() & !(0xf << offset)) | (port << offset))
                });
            }
            4..=7 => {
                syscfg.exticr2.modify(|r, w| unsafe {
                    w.bits((r.bits() & !(0xf << offset)) | (port << offset))
                });
            }
            8..=11 => {
                syscfg.exticr3.modify(|r, w| unsafe {
                    w.bits((r.bits() & !(0xf << offset)) | (port << offset))
                });
            }
            12..=15 => {
                syscfg.exticr4.modify(|r, w| unsafe {
                    w.bits((r.bits() & !(0xf << offset)) | (port << offset))
                });
            }
            _ => unreachable!(),
        }
    }

    /// Generate interrupt on rising edge, falling edge or both
    #[inline(always)]
    fn trigger_on_edge(&mut self, exti: &mut EXTI, edge: Edge) {
        let i = self.pin_id();
        match edge {
            Edge::Rising => {
                exti.rtsr1
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << i)) });
                exti.ftsr1
                    .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << i)) });
            }
            Edge::Falling => {
                exti.ftsr1
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << i)) });
                exti.rtsr1
                    .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << i)) });
            }
            Edge::RisingFalling => {
                exti.rtsr1
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << i)) });
                exti.ftsr1
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << i)) });
            }
        }
    }

    /// Enable external interrupts from this pin.
    #[inline(always)]
    fn enable_interrupt(&mut self, exti: &mut EXTI) {
        #[cfg(not(feature = "rm0399"))]
        let imr1 = &exti.cpuimr1;
        #[cfg(all(feature = "rm0399", feature = "cm7"))]
        let imr1 = &exti.c1imr1;
        #[cfg(all(feature = "rm0399", feature = "cm4"))]
        let imr1 = &exti.c2imr1;

        imr1.modify(|r, w| unsafe { w.bits(r.bits() | (1 << self.pin_id())) });
    }

    /// Disable external interrupts from this pin
    #[inline(always)]
    fn disable_interrupt(&mut self, exti: &mut EXTI) {
        #[cfg(not(feature = "rm0399"))]
        let imr1 = &exti.cpuimr1;
        #[cfg(all(feature = "rm0399", feature = "cm7"))]
        let imr1 = &exti.c1imr1;
        #[cfg(all(feature = "rm0399", feature = "cm4"))]
        let imr1 = &exti.c2imr1;

        imr1.modify(|r, w| unsafe { w.bits(r.bits() & !(1 << self.pin_id())) });
    }

    /// Clear the interrupt pending bit for this pin
    #[inline(always)]
    fn clear_interrupt_pending_bit(&mut self) {
        unsafe {
            #[cfg(not(feature = "rm0399"))]
            let pr1 = &(*EXTI::ptr()).cpupr1;
            #[cfg(all(feature = "rm0399", feature = "cm7"))]
            let pr1 = &(*EXTI::ptr()).c1pr1;
            #[cfg(all(feature = "rm0399", feature = "cm4"))]
            let pr1 = &(*EXTI::ptr()).c2pr1;

            pr1.write(|w| w.bits(1 << self.pin_id()));
            let _ = pr1.read();
            let _ = pr1.read(); // Delay 2 peripheral clocks
        }
    }

    /// Reads the interrupt pending bit for this pin
    #[inline(always)]
    fn check_interrupt(&self) -> bool {
        unsafe {
            #[cfg(not(feature = "rm0399"))]
            let pr1 = &(*EXTI::ptr()).cpupr1;
            #[cfg(all(feature = "rm0399", feature = "cm7"))]
            let pr1 = &(*EXTI::ptr()).c1pr1;
            #[cfg(all(feature = "rm0399", feature = "cm4"))]
            let pr1 = &(*EXTI::ptr()).c2pr1;

            (pr1.read().bits() & (1 << self.pin_id())) != 0
        }
    }
}

/// Generic pin type
///
/// - `MODE` is one of the pin modes (see [Modes](crate::gpio#modes) section).
/// - `P` is port name: `A` for GPIOA, `B` for GPIOB, etc.
/// - `N` is pin number: from `0` to `15`.
pub struct Pin<const P: char, const N: u8, MODE = Analog> {
    _mode: PhantomData<MODE>,
}
impl<const P: char, const N: u8, MODE> Pin<P, N, MODE> {
    const fn new() -> Self {
        Self { _mode: PhantomData }
    }
}

impl<const P: char, const N: u8, MODE> fmt::Debug for Pin<P, N, MODE> {
    fn fmt(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
        formatter.write_fmt(format_args!(
            "P{}{}<{}>",
            P,
            N,
            crate::stripped_type_name::<MODE>()
        ))
    }
}

#[cfg(feature = "defmt")]
impl<const P: char, const N: u8, MODE> defmt::Format for Pin<P, N, MODE> {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "P{}{}<{}>",
            P,
            N,
            crate::stripped_type_name::<MODE>()
        );
    }
}

impl<const P: char, const N: u8, MODE> PinExt for Pin<P, N, MODE> {
    type Mode = MODE;

    #[inline(always)]
    fn pin_id(&self) -> u8 {
        N
    }
    #[inline(always)]
    fn port_id(&self) -> u8 {
        P as u8 - b'A'
    }
}

impl<const P: char, const N: u8, MODE> Pin<P, N, Output<MODE>> {
    /// Set pin speed
    pub fn set_speed(self, speed: Speed) -> Self {
        let offset = 2 * { N };

        unsafe {
            (*Gpio::<P>::ptr()).ospeedr.modify(|r, w| {
                w.bits(
                    (r.bits() & !(0b11 << offset)) | ((speed as u32) << offset),
                )
            })
        };

        self
    }
}

impl<const P: char, const N: u8> Pin<P, N, Output<OpenDrain>> {
    /// Enables / disables the internal pull up
    pub fn internal_pull_up(self, on: bool) -> Self {
        let offset = 2 * { N };
        let value = if on { 0b01 } else { 0b00 };
        unsafe {
            (*Gpio::<P>::ptr()).pupdr.modify(|r, w| {
                w.bits((r.bits() & !(0b11 << offset)) | (value << offset))
            })
        };

        self
    }

    /// Enables / disables the internal pull down
    pub fn internal_pull_down(self, on: bool) -> Self {
        let offset = 2 * { N };
        let value = if on { 0b10 } else { 0b00 };
        unsafe {
            (*Gpio::<P>::ptr()).pupdr.modify(|r, w| {
                w.bits((r.bits() & !(0b11 << offset)) | (value << offset))
            })
        };

        self
    }
}

impl<const P: char, const N: u8, const A: u8>
    Pin<P, N, Alternate<A, PushPull>>
{
    /// Set pin speed
    pub fn set_speed(self, speed: Speed) -> Self {
        let offset = 2 * { N };

        unsafe {
            (*Gpio::<P>::ptr()).ospeedr.modify(|r, w| {
                w.bits(
                    (r.bits() & !(0b11 << offset)) | ((speed as u32) << offset),
                )
            })
        };

        self
    }

    /// Enables / disables the internal pull up
    pub fn internal_pull_up(self, on: bool) -> Self {
        let offset = 2 * { N };
        let value = if on { 0b01 } else { 0b00 };
        unsafe {
            (*Gpio::<P>::ptr()).pupdr.modify(|r, w| {
                w.bits((r.bits() & !(0b11 << offset)) | (value << offset))
            })
        };

        self
    }

    /// Enables / disables the internal pull down
    pub fn internal_pull_down(self, on: bool) -> Self {
        let offset = 2 * { N };
        let value = if on { 0b10 } else { 0b00 };
        unsafe {
            (*Gpio::<P>::ptr()).pupdr.modify(|r, w| {
                w.bits((r.bits() & !(0b11 << offset)) | (value << offset))
            })
        };

        self
    }
}

impl<const P: char, const N: u8, const A: u8>
    Pin<P, N, Alternate<A, PushPull>>
{
    /// Turns pin alternate configuration pin into open drain
    pub fn set_open_drain(self) -> Pin<P, N, Alternate<A, OpenDrain>> {
        let offset = { N };
        unsafe {
            (*Gpio::<P>::ptr())
                .otyper
                .modify(|r, w| w.bits(r.bits() | (1 << offset)))
        };

        Pin::new()
    }
}

impl<const P: char, const N: u8, MODE> Pin<P, N, MODE> {
    /// Erases the pin number from the type
    ///
    /// This is useful when you want to collect the pins into an array where you
    /// need all the elements to have the same type
    pub fn erase_number(self) -> PEPin<P, MODE> {
        PEPin::new(N)
    }

    /// Erases the pin number and the port from the type
    ///
    /// This is useful when you want to collect the pins into an array where you
    /// need all the elements to have the same type
    pub fn erase(self) -> EPin<MODE> {
        EPin::new(P as u8 - b'A', N)
    }
}

impl<const P: char, const N: u8, MODE> Pin<P, N, MODE> {
    /// Set the output of the pin regardless of its mode.
    /// Primarily used to set the output value of the pin
    /// before changing its mode to an output to avoid
    /// a short spike of an incorrect value
    #[inline(always)]
    fn _set_state(&mut self, state: PinState) {
        match state {
            PinState::High => self._set_high(),
            PinState::Low => self._set_low(),
        }
    }
    #[inline(always)]
    fn _set_high(&mut self) {
        // NOTE(unsafe) atomic write to a stateless register
        unsafe { (*Gpio::<P>::ptr()).bsrr.write(|w| w.bits(1 << N)) }
    }
    #[inline(always)]
    fn _set_low(&mut self) {
        // NOTE(unsafe) atomic write to a stateless register
        unsafe { (*Gpio::<P>::ptr()).bsrr.write(|w| w.bits(1 << (16 + N))) }
    }
    #[inline(always)]
    fn _is_set_low(&self) -> bool {
        // NOTE(unsafe) atomic read with no side effects
        unsafe { (*Gpio::<P>::ptr()).odr.read().bits() & (1 << N) == 0 }
    }
    #[inline(always)]
    fn _is_low(&self) -> bool {
        // NOTE(unsafe) atomic read with no side effects
        unsafe { (*Gpio::<P>::ptr()).idr.read().bits() & (1 << N) == 0 }
    }
}

impl<const P: char, const N: u8, MODE> Pin<P, N, Output<MODE>> {
    #[inline(always)]
    pub fn set_high(&mut self) {
        self._set_high()
    }

    #[inline(always)]
    pub fn set_low(&mut self) {
        self._set_low()
    }

    #[inline(always)]
    pub fn get_state(&self) -> PinState {
        if self.is_set_low() {
            PinState::Low
        } else {
            PinState::High
        }
    }

    #[inline(always)]
    pub fn set_state(&mut self, state: PinState) {
        match state {
            PinState::Low => self.set_low(),
            PinState::High => self.set_high(),
        }
    }

    #[inline(always)]
    pub fn is_set_high(&self) -> bool {
        !self.is_set_low()
    }

    #[inline(always)]
    pub fn is_set_low(&self) -> bool {
        self._is_set_low()
    }

    #[inline(always)]
    pub fn toggle(&mut self) {
        if self.is_set_low() {
            self.set_high()
        } else {
            self.set_low()
        }
    }
}

impl<const P: char, const N: u8> Pin<P, N, Output<OpenDrain>> {
    #[inline(always)]
    pub fn is_high(&self) -> bool {
        !self.is_low()
    }

    #[inline(always)]
    pub fn is_low(&self) -> bool {
        self._is_low()
    }
}

impl<const P: char, const N: u8, MODE> Pin<P, N, Input<MODE>> {
    #[inline(always)]
    pub fn is_high(&self) -> bool {
        !self.is_low()
    }

    #[inline(always)]
    pub fn is_low(&self) -> bool {
        self._is_low()
    }
}

macro_rules! gpio {
    ($GPIOX:ident, $gpiox:ident, $gpio_doc:expr, $Rec:ident, $PEPin:ident, $port_id:expr, $PXn:ident, [
        $($PXi:ident: ($pxi:ident, $i:expr $(, $MODE:ty)?),)+
    ]) => {
        #[doc=$gpio_doc]
        pub mod $gpiox {
            use crate::pac::$GPIOX;
            use crate::rcc::{rec, ResetEnable};
            use super::Analog;

            /// GPIO parts
            pub struct Parts {
                $(
                    /// Pin
                    pub $pxi: $PXi $(<$MODE>)?,
                )+
            }

            impl super::GpioExt for $GPIOX {
                type Parts = Parts;
                type Rec = rec::$Rec;

                fn split(self, prec: rec::$Rec) -> Parts {
                    prec.enable().reset();

                    Parts {
                        $(
                            $pxi: $PXi::new(),
                        )+
                    }
                }

                fn split_without_reset(self, prec: rec::$Rec) -> Parts {
                    prec.enable();

                    Parts {
                        $(
                            $pxi: $PXi::new(),
                        )+
                    }
                }
            }


            pub type $PXn<MODE> = super::PEPin<$port_id, MODE>;

            $(
                pub type $PXi<MODE = Analog> = super::Pin<$port_id, $i, MODE>;
            )+

        }

        pub use $gpiox::{ $($PXi,)+ };
    }
}

gpio!(GPIOA, gpioa, "Port A", Gpioa, PA, 'A', PAn, [
    PA0: (pa0, 0),
    PA1: (pa1, 1),
    PA2: (pa2, 2),
    PA3: (pa3, 3),
    PA4: (pa4, 4),
    PA5: (pa5, 5),
    PA6: (pa6, 6),
    PA7: (pa7, 7),
    PA8: (pa8, 8),
    PA9: (pa9, 9),
    PA10: (pa10, 10),
    PA11: (pa11, 11),
    PA12: (pa12, 12),
    PA13: (pa13, 13, super::Debugger), // SWDIO, PullUp VeryHigh speed
    PA14: (pa14, 14, super::Debugger), // SWCLK, PullDown
    PA15: (pa15, 15, super::Debugger), // JTDI, PullUp
]);

gpio!(GPIOB, gpiob, "Port B", Gpiob, PB, 'B', PBn, [
    PB0: (pb0, 0),
    PB1: (pb1, 1),
    PB2: (pb2, 2),
    PB3: (pb3, 3, super::Debugger), // SWO, VeryHigh speed
    PB4: (pb4, 4, super::Debugger), // JTRST, PullUp
    PB5: (pb5, 5),
    PB6: (pb6, 6),
    PB7: (pb7, 7),
    PB8: (pb8, 8),
    PB9: (pb9, 9),
    PB10: (pb10, 10),
    PB11: (pb11, 11),
    PB12: (pb12, 12),
    PB13: (pb13, 13),
    PB14: (pb14, 14),
    PB15: (pb15, 15),
]);

gpio!(GPIOC, gpioc, "Port C", Gpioc, PC, 'C', PCn, [
    PC0: (pc0, 0),
    PC1: (pc1, 1),
    PC2: (pc2, 2),
    PC3: (pc3, 3),
    PC4: (pc4, 4),
    PC5: (pc5, 5),
    PC6: (pc6, 6),
    PC7: (pc7, 7),
    PC8: (pc8, 8),
    PC9: (pc9, 9),
    PC10: (pc10, 10),
    PC11: (pc11, 11),
    PC12: (pc12, 12),
    PC13: (pc13, 13),
    PC14: (pc14, 14),
    PC15: (pc15, 15),
]);

gpio!(GPIOD, gpiod, "Port D", Gpiod, PD, 'D', PDn, [
    PD0: (pd0, 0),
    PD1: (pd1, 1),
    PD2: (pd2, 2),
    PD3: (pd3, 3),
    PD4: (pd4, 4),
    PD5: (pd5, 5),
    PD6: (pd6, 6),
    PD7: (pd7, 7),
    PD8: (pd8, 8),
    PD9: (pd9, 9),
    PD10: (pd10, 10),
    PD11: (pd11, 11),
    PD12: (pd12, 12),
    PD13: (pd13, 13),
    PD14: (pd14, 14),
    PD15: (pd15, 15),
]);

gpio!(GPIOE, gpioe, "Port E", Gpioe, PE, 'E', PEn, [
    PE0: (pe0, 0),
    PE1: (pe1, 1),
    PE2: (pe2, 2),
    PE3: (pe3, 3),
    PE4: (pe4, 4),
    PE5: (pe5, 5),
    PE6: (pe6, 6),
    PE7: (pe7, 7),
    PE8: (pe8, 8),
    PE9: (pe9, 9),
    PE10: (pe10, 10),
    PE11: (pe11, 11),
    PE12: (pe12, 12),
    PE13: (pe13, 13),
    PE14: (pe14, 14),
    PE15: (pe15, 15),
]);

gpio!(GPIOF, gpiof, "Port F", Gpiof, PF, 'F', PFn, [
    PF0: (pf0, 0),
    PF1: (pf1, 1),
    PF2: (pf2, 2),
    PF3: (pf3, 3),
    PF4: (pf4, 4),
    PF5: (pf5, 5),
    PF6: (pf6, 6),
    PF7: (pf7, 7),
    PF8: (pf8, 8),
    PF9: (pf9, 9),
    PF10: (pf10, 10),
    PF11: (pf11, 11),
    PF12: (pf12, 12),
    PF13: (pf13, 13),
    PF14: (pf14, 14),
    PF15: (pf15, 15),
]);

gpio!(GPIOG, gpiog, "Port G", Gpiog, PG, 'G', PGn, [
    PG0: (pg0, 0),
    PG1: (pg1, 1),
    PG2: (pg2, 2),
    PG3: (pg3, 3),
    PG4: (pg4, 4),
    PG5: (pg5, 5),
    PG6: (pg6, 6),
    PG7: (pg7, 7),
    PG8: (pg8, 8),
    PG9: (pg9, 9),
    PG10: (pg10, 10),
    PG11: (pg11, 11),
    PG12: (pg12, 12),
    PG13: (pg13, 13),
    PG14: (pg14, 14),
    PG15: (pg15, 15),
]);

gpio!(GPIOH, gpioh, "Port H", Gpioh, PH, 'H', PHn, [
    PH0: (ph0, 0),
    PH1: (ph1, 1),
    PH2: (ph2, 2),
    PH3: (ph3, 3),
    PH4: (ph4, 4),
    PH5: (ph5, 5),
    PH6: (ph6, 6),
    PH7: (ph7, 7),
    PH8: (ph8, 8),
    PH9: (ph9, 9),
    PH10: (ph10, 10),
    PH11: (ph11, 11),
    PH12: (ph12, 12),
    PH13: (ph13, 13),
    PH14: (ph14, 14),
    PH15: (ph15, 15),
]);

#[cfg(not(feature = "rm0468"))]
gpio!(GPIOI, gpioi, "Port I", Gpioi, PI, 'I', PIn, [
    PI0: (pi0, 0),
    PI1: (pi1, 1),
    PI2: (pi2, 2),
    PI3: (pi3, 3),
    PI4: (pi4, 4),
    PI5: (pi5, 5),
    PI6: (pi6, 6),
    PI7: (pi7, 7),
    PI8: (pi8, 8),
    PI9: (pi9, 9),
    PI10: (pi10, 10),
    PI11: (pi11, 11),
    PI12: (pi12, 12),
    PI13: (pi13, 13),
    PI14: (pi14, 14),
    PI15: (pi15, 15),
]);

gpio!(GPIOJ, gpioj, "Port J", Gpioj, PJ, 'J', PJn, [
    PJ0: (pj0, 0),
    PJ1: (pj1, 1),
    PJ2: (pj2, 2),
    PJ3: (pj3, 3),
    PJ4: (pj4, 4),
    PJ5: (pj5, 5),
    PJ6: (pj6, 6),
    PJ7: (pj7, 7),
    PJ8: (pj8, 8),
    PJ9: (pj9, 9),
    PJ10: (pj10, 10),
    PJ11: (pj11, 11),
    PJ12: (pj12, 12),
    PJ13: (pj13, 13),
    PJ14: (pj14, 14),
    PJ15: (pj15, 15),
]);

gpio!(GPIOK, gpiok, "Port K", Gpiok, PK, 'K', PKn, [
    PK0: (pk0, 0),
    PK1: (pk1, 1),
    PK2: (pk2, 2),
    PK3: (pk3, 3),
    PK4: (pk4, 4),
    PK5: (pk5, 5),
    PK6: (pk6, 6),
    PK7: (pk7, 7),
    PK8: (pk8, 8),
    PK9: (pk9, 9),
    PK10: (pk10, 10),
    PK11: (pk11, 11),
    PK12: (pk12, 12),
    PK13: (pk13, 13),
    PK14: (pk14, 14),
    PK15: (pk15, 15),
]);

struct Gpio<const P: char>;
impl<const P: char> Gpio<P> {
    const fn ptr() -> *const crate::pac::gpioa::RegisterBlock {
        match P {
            'A' => crate::pac::GPIOA::ptr(),
            'B' => crate::pac::GPIOB::ptr() as _,
            'C' => crate::pac::GPIOC::ptr() as _,
            'D' => crate::pac::GPIOD::ptr() as _,
            'E' => crate::pac::GPIOE::ptr() as _,
            'F' => crate::pac::GPIOF::ptr() as _,
            'G' => crate::pac::GPIOG::ptr() as _,
            'H' => crate::pac::GPIOH::ptr() as _,
            #[cfg(not(feature = "rm0468"))]
            'I' => crate::pac::GPIOI::ptr() as _,
            'J' => crate::pac::GPIOJ::ptr() as _,
            'K' => crate::pac::GPIOK::ptr() as _,
            _ => crate::pac::GPIOA::ptr(),
        }
    }
}
