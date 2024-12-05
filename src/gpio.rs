//! General Purpose Input / Output
//!
//! The GPIO pins are organised into groups of 16 pins which can be accessed through the
//! `gpioa`, `gpiob`... modules. To get access to the pins, you first need to convert them into a
//! HAL designed struct from the `pac` struct using the [split](trait.GpioExt.html#tymethod.split) function.
//! ```rust
//! // Acquire the GPIOA peripheral
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
//!       is connected
//!     - **PullDown**: Input connected to high with a weak pull up resistor. Will be low when nothing
//!       is connected
//!     - **Floating**: Input not pulled to high or low. Will be undefined when nothing is connected
//! - Output
//!     - **PushPull**: Output which either drives the pin high or low
//!     - **OpenDrain**: Output which leaves the gate floating, or pulls it do ground in drain
//!       mode. Can be used as an input in the `open` configuration
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
//! - [Usage of `with_*` methods](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/gpio_with_input.rs)

use core::marker::PhantomData;

use crate::rcc::ResetEnable;

mod convert;
pub use convert::PinMode;
mod partially_erased;
pub use partially_erased::{PEPin, PartiallyErasedPin};
mod erased;
pub use erased::{EPin, ErasedPin};
mod exti;
pub use exti::ExtiPin;
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

/// Id, port and mode for any pin
pub trait PinExt {
    /// Current pin mode
    type Mode;
    /// Pin number
    fn pin_id(&self) -> u8;
    /// Port number starting from 0
    fn port_id(&self) -> u8;
}

/// Some alternate mode (type state)
pub struct Alternate<const A: u8, Otype = PushPull>(PhantomData<Otype>);

/// Input mode (type state)
pub struct Input;

/// Pull setting for an input.
#[derive(Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Pull {
    /// Floating
    None = 0,
    /// Pulled up
    Up = 1,
    /// Pulled down
    Down = 2,
}

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

/// JTAG/SWD mote (type state)
pub type Debugger = Alternate<0, PushPull>;

mod marker {
    /// Marker trait that show if `ExtiPin` can be implemented
    pub trait Interruptable {}
    /// Marker trait for readable pin modes
    pub trait Readable {}
    /// Marker trait for slew rate configurable pin modes
    pub trait OutputSpeed {}
    /// Marker trait for active pin modes
    pub trait Active {}
    /// Marker trait for all pin modes except alternate
    pub trait NotAlt {}
    /// Marker trait for pins with alternate function `A` mapping
    pub trait IntoAf<const A: u8> {}
}

impl<MODE> marker::Interruptable for Output<MODE> {}
impl marker::Interruptable for Input {}
impl marker::Readable for Input {}
impl<const A: u8, MODE> marker::Readable for Alternate<A, MODE> {}
impl marker::Readable for Output<OpenDrain> {}
impl marker::Active for Input {}
impl<Otype> marker::OutputSpeed for Output<Otype> {}
impl<const A: u8, Otype> marker::OutputSpeed for Alternate<A, Otype> {}
impl<Otype> marker::Active for Output<Otype> {}
impl<const A: u8, Otype> marker::Active for Alternate<A, Otype> {}
impl marker::NotAlt for Input {}
impl<Otype> marker::NotAlt for Output<Otype> {}
impl marker::NotAlt for Analog {}

/// GPIO Pin speed selection
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum Speed {
    /// Low speed
    Low = 0,
    /// Medium speed
    Medium = 1,
    /// High speed
    High = 2,
    /// Very high speed
    VeryHigh = 3,
}

/// GPIO interrupt trigger edge selection
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum Edge {
    /// Rising edge of voltage
    Rising,
    /// Falling edge of voltage
    Falling,
    /// Rising and falling edge of voltage
    RisingFalling,
}

macro_rules! af {
    ($($i:literal: $AFi:ident),+) => {
        $(
            #[doc = concat!("Alternate function ", $i, " (type state)" )]
            pub type $AFi<Otype = PushPull> = Alternate<$i, Otype>;
        )+
    };
}

af!(
    0: AF0,
    1: AF1,
    2: AF2,
    3: AF3,
    4: AF4,
    5: AF5,
    6: AF6,
    7: AF7,
    8: AF8,
    9: AF9,
    10: AF10,
    11: AF11,
    12: AF12,
    13: AF13,
    14: AF14,
    15: AF15
);

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

impl<const P: char, const N: u8, MODE> Pin<P, N, MODE>
where
    MODE: marker::OutputSpeed,
{
    /// Set pin speed
    pub fn set_speed(&mut self, speed: Speed) {
        let offset = 2 * { N };

        unsafe {
            (*Gpio::<P>::ptr()).ospeedr().modify(|r, w| {
                w.bits(
                    (r.bits() & !(0b11 << offset)) | ((speed as u32) << offset),
                )
            });
        }
    }

    /// Set pin speed
    pub fn speed(mut self, speed: Speed) -> Self {
        self.set_speed(speed);
        self
    }
}

impl<const P: char, const N: u8, MODE> Pin<P, N, MODE>
where
    MODE: marker::Active,
{
    /// Set the internal pull-up and pull-down resistor
    pub fn set_internal_resistor(&mut self, resistor: Pull) {
        let offset = 2 * { N };
        let value = resistor as u32;
        unsafe {
            (*Gpio::<P>::ptr()).pupdr().modify(|r, w| {
                w.bits((r.bits() & !(0b11 << offset)) | (value << offset))
            });
        }
    }

    /// Set the internal pull-up and pull-down resistor
    pub fn internal_resistor(mut self, resistor: Pull) -> Self {
        self.set_internal_resistor(resistor);
        self
    }

    /// Enables / disables the internal pull up
    pub fn internal_pull_up(self, on: bool) -> Self {
        if on {
            self.internal_resistor(Pull::Up)
        } else {
            self.internal_resistor(Pull::None)
        }
    }

    /// Enables / disables the internal pull down
    pub fn internal_pull_down(self, on: bool) -> Self {
        if on {
            self.internal_resistor(Pull::Down)
        } else {
            self.internal_resistor(Pull::None)
        }
    }
}

impl<const P: char, const N: u8, MODE> Pin<P, N, MODE> {
    /// Erases the pin number from the type
    ///
    /// This is useful when you want to collect the pins into an array where you
    /// need all the elements to have the same type
    pub fn erase_number(self) -> PartiallyErasedPin<P, MODE> {
        PartiallyErasedPin::new(N)
    }

    /// Erases the pin number and the port from the type
    ///
    /// This is useful when you want to collect the pins into an array where you
    /// need all the elements to have the same type
    pub fn erase(self) -> ErasedPin<MODE> {
        ErasedPin::new(P as u8 - b'A', N)
    }
}

impl<const P: char, const N: u8, MODE> From<Pin<P, N, MODE>>
    for PartiallyErasedPin<P, MODE>
{
    /// Pin-to-partially erased pin conversion using the [`From`] trait.
    ///
    /// Note that [`From`] is the reciprocal of [`Into`].
    fn from(p: Pin<P, N, MODE>) -> Self {
        p.erase_number()
    }
}

impl<const P: char, const N: u8, MODE> From<Pin<P, N, MODE>>
    for ErasedPin<MODE>
{
    /// Pin-to-erased pin conversion using the [`From`] trait.
    ///
    /// Note that [`From`] is the reciprocal of [`Into`].
    fn from(p: Pin<P, N, MODE>) -> Self {
        p.erase()
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
        unsafe {
            (*Gpio::<P>::ptr()).bsrr().write(|w| w.bits(1 << N));
        }
    }
    #[inline(always)]
    fn _set_low(&mut self) {
        // NOTE(unsafe) atomic write to a stateless register
        unsafe {
            (*Gpio::<P>::ptr()).bsrr().write(|w| w.bits(1 << (16 + N)));
        }
    }
    #[inline(always)]
    fn _is_set_low(&self) -> bool {
        // NOTE(unsafe) atomic read with no side effects
        unsafe { (*Gpio::<P>::ptr()).odr().read().bits() & (1 << N) == 0 }
    }
    #[inline(always)]
    fn _is_low(&self) -> bool {
        // NOTE(unsafe) atomic read with no side effects
        unsafe { (*Gpio::<P>::ptr()).idr().read().bits() & (1 << N) == 0 }
    }
}

impl<const P: char, const N: u8, MODE> Pin<P, N, Output<MODE>> {
    /// Drives the pin high
    #[inline(always)]
    pub fn set_high(&mut self) {
        self._set_high()
    }

    /// Drives the pin low
    #[inline(always)]
    pub fn set_low(&mut self) {
        self._set_low()
    }

    /// Is the pin in drive high or low mode?
    #[inline(always)]
    pub fn get_state(&self) -> PinState {
        if self.is_set_low() {
            PinState::Low
        } else {
            PinState::High
        }
    }

    /// Drives the pin high or low depending on the provided value
    #[inline(always)]
    pub fn set_state(&mut self, state: PinState) {
        match state {
            PinState::Low => self.set_low(),
            PinState::High => self.set_high(),
        }
    }

    /// Is the pin in drive high mode?
    #[inline(always)]
    pub fn is_set_high(&self) -> bool {
        !self.is_set_low()
    }

    /// Is the pin in drive low mode?
    #[inline(always)]
    pub fn is_set_low(&self) -> bool {
        self._is_set_low()
    }

    /// Toggle pin output
    #[inline(always)]
    pub fn toggle(&mut self) {
        if self.is_set_low() {
            self.set_high()
        } else {
            self.set_low()
        }
    }
}

impl<const P: char, const N: u8, MODE> Pin<P, N, MODE>
where
    MODE: marker::Readable,
{
    /// Is the input pin high?
    #[inline(always)]
    pub fn is_high(&self) -> bool {
        !self.is_low()
    }

    /// Is the input pin low?
    #[inline(always)]
    pub fn is_low(&self) -> bool {
        self._is_low()
    }
}

macro_rules! gpio {
    ($GPIOX:ident, $gpiox:ident, $Rec:ident, $PEPin:ident, $port_id:expr, $PXn:ident, [
        $($PXi:ident: ($pxi:ident, $i:expr, [$($A:literal),*] $(, $MODE:ty)?),)+
    ]) => {
        #[doc=concat!("Port ", $port_id)]
        pub mod $gpiox {
            use crate::pac::$GPIOX;
            use crate::rcc::{rec, ResetEnable};

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

            #[doc=concat!("Common type for GPIO", $port_id, " related pins")]
            pub type $PXn<MODE> = super::PartiallyErasedPin<$port_id, MODE>;

            $(
                #[doc=concat!("P", $port_id, $i, " pin")]
                pub type $PXi<MODE = super::Analog> = super::Pin<$port_id, $i, MODE>;

                $(
                    impl<MODE> super::marker::IntoAf<$A> for $PXi<MODE> { }
                )*
            )+

        }

        pub use $gpiox::{ $($PXi,)+ };
    }
}

#[cfg(feature = "gpio-h72")]
gpio!(GPIOA, gpioa, Gpioa, PA, 'A', PAn, [
    PA0: (pa0, 0, [1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12, 15]),
    PA1: (pa1, 1, [1, 2, 3, 4, 7, 8, 9, 10, 11, 12, 14, 15]),
    PA2: (pa2, 2, [1, 2, 3, 4, 6, 7, 8, 11, 12, 14, 15]),
    PA3: (pa3, 3, [1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12, 14, 15]),
    PA4: (pa4, 4, [2, 5, 6, 7, 8, 12, 13, 14, 15]),
    PA5: (pa5, 5, [1, 3, 5, 8, 10, 12, 13, 14, 15]),
    PA6: (pa6, 6, [1, 2, 3, 5, 6, 8, 9, 10, 11, 12, 13, 14, 15]),
    PA7: (pa7, 7, [1, 2, 3, 5, 8, 9, 10, 11, 12, 14, 15]),
    PA8: (pa8, 8, [0, 1, 3, 4, 6, 7, 10, 11, 12, 13, 14, 15]),
    PA9: (pa9, 9, [1, 3, 4, 5, 6, 7, 11, 13, 14, 15]),
    PA10: (pa10, 10, [1, 3, 7, 10, 11, 12, 13, 14, 15]),
    PA11: (pa11, 11, [1, 3, 5, 6, 7, 9, 14, 15]),
    PA12: (pa12, 12, [1, 3, 5, 6, 7, 8, 9, 12, 14, 15]),
    PA13: (pa13, 13, [0, 15], super::Debugger), // SWDIO, PullUp VeryHigh speed
    PA14: (pa14, 14, [0, 15], super::Debugger), // SWCLK, PullDown
    PA15: (pa15, 15, [0, 1, 4, 5, 6, 7, 8, 9, 11, 14, 15], super::Debugger), // JTDI, PullUp
]);

#[cfg(feature = "gpio-h72")]
gpio!(GPIOB, gpiob, Gpiob, PB, 'B', PBn, [
    PB0: (pb0, 0, [1, 2, 3, 4, 6, 8, 9, 10, 11, 14, 15]),
    PB1: (pb1, 1, [1, 2, 3, 4, 6, 9, 10, 11, 14, 15]),
    PB2: (pb2, 2, [0, 1, 2, 4, 6, 7, 8, 9, 10, 11, 13, 15]),
    PB3: (pb3, 3, [0, 1, 5, 6, 8, 9, 10, 11, 14, 15], super::Debugger), // SWO, VeryHigh speed
    PB4: (pb4, 4, [0, 1, 2, 5, 6, 7, 8, 9, 11, 15], super::Debugger), // JTRST, PullUp
    PB5: (pb5, 5, [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]),
    PB6: (pb6, 6, [1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]),
    PB7: (pb7, 7, [1, 2, 4, 6, 7, 8, 11, 12, 13, 15]),
    PB8: (pb8, 8, [1, 2, 3, 4, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]),
    PB9: (pb9, 9, [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]),
    PB10: (pb10, 10, [1, 3, 4, 5, 6, 7, 9, 10, 11, 14, 15]),
    PB11: (pb11, 11, [1, 3, 4, 6, 7, 10, 11, 14, 15]),
    PB12: (pb12, 12, [1, 3, 4, 5, 6, 7, 9, 10, 11, 12, 13, 14, 15]),
    PB13: (pb13, 13, [1, 3, 4, 5, 6, 7, 9, 10, 11, 12, 13, 14, 15]),
    PB14: (pb14, 14, [1, 2, 3, 4, 5, 6, 7, 8, 9, 12, 14, 15]),
    PB15: (pb15, 15, [0, 1, 2, 3, 4, 5, 6, 8, 9, 12, 14, 15]),
]);

#[cfg(feature = "gpio-h72")]
gpio!(GPIOC, gpioc, Gpioc, PC, 'C', PCn, [
    PC0: (pc0, 0, [1, 3, 6, 8, 9, 10, 11, 12, 14, 15]),
    PC1: (pc1, 1, [0, 1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 14, 15]),
    PC2: (pc2, 2, [0, 3, 4, 5, 6, 9, 10, 11, 12, 15]),
    PC3: (pc3, 3, [0, 3, 4, 5, 9, 10, 11, 12, 15]),
    PC4: (pc4, 4, [1, 3, 5, 9, 10, 11, 12, 14, 15]),
    PC5: (pc5, 5, [1, 2, 3, 4, 9, 10, 11, 12, 13, 14, 15]),
    PC6: (pc6, 6, [2, 3, 4, 5, 7, 8, 9, 10, 12, 13, 14, 15]),
    PC7: (pc7, 7, [0, 2, 3, 4, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]),
    PC8: (pc8, 8, [0, 2, 3, 7, 8, 9, 10, 11, 12, 13, 15]),
    PC9: (pc9, 9, [0, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13, 14, 15]),
    PC10: (pc10, 10, [3, 4, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]),
    PC11: (pc11, 11, [3, 4, 6, 7, 8, 9, 12, 13, 14, 15]),
    PC12: (pc12, 12, [0, 1, 2, 4, 5, 6, 7, 8, 12, 13, 14, 15]),
    PC13: (pc13, 13, [15]),
    PC14: (pc14, 14, [15]),
    PC15: (pc15, 15, [15]),
]);

#[cfg(feature = "gpio-h72")]
gpio!(GPIOD, gpiod, Gpiod, PD, 'D', PDn, [
    PD0: (pd0, 0, [3, 8, 9, 11, 12, 14, 15]),
    PD1: (pd1, 1, [3, 8, 9, 12, 15]),
    PD2: (pd2, 2, [0, 1, 2, 4, 8, 9, 12, 13, 14, 15]),
    PD3: (pd3, 3, [3, 5, 7, 12, 13, 14, 15]),
    PD4: (pd4, 4, [7, 10, 12, 15]),
    PD5: (pd5, 5, [7, 10, 12, 15]),
    PD6: (pd6, 6, [1, 2, 3, 4, 5, 6, 7, 8, 10, 11, 12, 13, 14, 15]),
    PD7: (pd7, 7, [3, 5, 6, 7, 9, 10, 11, 12, 15]),
    PD8: (pd8, 8, [3, 7, 9, 12, 15]),
    PD9: (pd9, 9, [3, 7, 12, 15]),
    PD10: (pd10, 10, [3, 7, 12, 14, 15]),
    PD11: (pd11, 11, [3, 4, 7, 9, 10, 12, 15]),
    PD12: (pd12, 12, [1, 2, 3, 4, 5, 7, 9, 10, 12, 13, 15]),
    PD13: (pd13, 13, [1, 2, 4, 5, 9, 10, 11, 12, 13, 15]),
    PD14: (pd14, 14, [2, 8, 11, 12, 15]),
    PD15: (pd15, 15, [2, 8, 11, 12, 15]),
]);

#[cfg(feature = "gpio-h72")]
gpio!(GPIOE, gpioe, Gpioe, PE, 'E', PEn, [
    PE0: (pe0, 0, [1, 2, 4, 8, 10, 12, 13, 14, 15]),
    PE1: (pe1, 1, [1, 8, 12, 13, 14, 15]),
    PE2: (pe2, 2, [0, 2, 4, 5, 6, 8, 9, 10, 11, 12, 15]),
    PE3: (pe3, 3, [0, 4, 6, 8, 11, 12, 15]),
    PE4: (pe4, 4, [0, 2, 3, 4, 5, 6, 8, 10, 12, 13, 14, 15]),
    PE5: (pe5, 5, [0, 2, 3, 4, 5, 6, 8, 10, 12, 13, 14, 15]),
    PE6: (pe6, 6, [0, 1, 2, 4, 5, 6, 8, 9, 10, 11, 12, 13, 14, 15]),
    PE7: (pe7, 7, [1, 3, 7, 10, 12, 15]),
    PE8: (pe8, 8, [1, 3, 7, 10, 12, 13, 15]),
    PE9: (pe9, 9, [1, 3, 7, 10, 12, 15]),
    PE10: (pe10, 10, [1, 3, 7, 10, 12, 15]),
    PE11: (pe11, 11, [1, 3, 5, 10, 11, 12, 14, 15]),
    PE12: (pe12, 12, [1, 3, 5, 10, 12, 13, 14, 15]),
    PE13: (pe13, 13, [1, 3, 5, 10, 12, 13, 14, 15]),
    PE14: (pe14, 14, [1, 5, 10, 12, 14, 15]),
    PE15: (pe15, 15, [1, 11, 12, 13, 14, 15]),
]);

#[cfg(feature = "gpio-h72")]
gpio!(GPIOF, gpiof, Gpiof, PF, 'F', PFn, [
    PF0: (pf0, 0, [4, 6, 9, 12, 13, 15]),
    PF1: (pf1, 1, [4, 6, 9, 12, 13, 15]),
    PF2: (pf2, 2, [4, 6, 9, 12, 13, 15]),
    PF3: (pf3, 3, [9, 12, 13, 15]),
    PF4: (pf4, 4, [9, 12, 15]),
    PF5: (pf5, 5, [9, 12, 15]),
    PF6: (pf6, 6, [1, 2, 5, 6, 7, 8, 10, 13, 15]),
    PF7: (pf7, 7, [1, 2, 5, 6, 7, 8, 10, 13, 15]),
    PF8: (pf8, 8, [1, 5, 6, 7, 8, 9, 10, 13, 15]),
    PF9: (pf9, 9, [1, 5, 6, 7, 8, 9, 10, 13, 15]),
    PF10: (pf10, 10, [1, 2, 4, 9, 10, 13, 14, 15]),
    PF11: (pf11, 11, [5, 9, 10, 12, 13, 14, 15]),
    PF12: (pf12, 12, [9, 12, 14, 15]),
    PF13: (pf13, 13, [3, 4, 12, 14, 15]),
    PF14: (pf14, 14, [3, 4, 12, 14, 15]),
    PF15: (pf15, 15, [4, 12, 15]),
]);

#[cfg(feature = "gpio-h72")]
gpio!(GPIOG, gpiog, Gpiog, PG, 'G', PGn, [
    PG0: (pg0, 0, [9, 11, 12, 15]),
    PG1: (pg1, 1, [9, 11, 12, 15]),
    PG2: (pg2, 2, [3, 11, 12, 14, 15]),
    PG3: (pg3, 3, [3, 11, 12, 13, 15]),
    PG4: (pg4, 4, [1, 11, 12, 15]),
    PG5: (pg5, 5, [1, 12, 15]),
    PG6: (pg6, 6, [1, 10, 12, 13, 14, 15]),
    PG7: (pg7, 7, [6, 7, 9, 12, 13, 14, 15]),
    PG8: (pg8, 8, [3, 5, 7, 8, 11, 12, 14, 15]),
    PG9: (pg9, 9, [2, 5, 7, 8, 9, 10, 11, 12, 13, 15]),
    PG10: (pg10, 10, [2, 3, 5, 9, 10, 11, 12, 13, 14, 15]),
    PG11: (pg11, 11, [1, 4, 5, 8, 9, 10, 11, 13, 14, 15]),
    PG12: (pg12, 12, [1, 3, 4, 5, 7, 8, 9, 10, 11, 12, 13, 14, 15]),
    PG13: (pg13, 13, [0, 1, 4, 5, 7, 10, 11, 12, 13, 14, 15]),
    PG14: (pg14, 14, [0, 1, 4, 5, 7, 9, 10, 11, 12, 13, 14, 15]),
    PG15: (pg15, 15, [7, 9, 11, 12, 13, 15]),
]);

#[cfg(feature = "gpio-h72")]
gpio!(GPIOH, gpioh, Gpioh, PH, 'H', PHn, [
    PH0: (ph0, 0, [15]),
    PH1: (ph1, 1, [15]),
    PH2: (ph2, 2, [1, 9, 10, 11, 12, 14, 15]),
    PH3: (ph3, 3, [9, 10, 11, 12, 14, 15]),
    PH4: (ph4, 4, [4, 9, 10, 13, 14, 15]),
    PH5: (ph5, 5, [4, 5, 12, 15]),
    PH6: (ph6, 6, [2, 4, 5, 11, 12, 13, 15]),
    PH7: (ph7, 7, [4, 5, 11, 12, 13, 15]),
    PH8: (ph8, 8, [2, 4, 12, 13, 14, 15]),
    PH9: (ph9, 9, [2, 4, 12, 13, 14, 15]),
    PH10: (ph10, 10, [2, 4, 12, 13, 14, 15]),
    PH11: (ph11, 11, [2, 4, 12, 13, 14, 15]),
    PH12: (ph12, 12, [2, 4, 12, 13, 14, 15]),
    PH13: (ph13, 13, [3, 8, 9, 12, 14, 15]),
    PH14: (ph14, 14, [3, 8, 9, 12, 13, 14, 15]),
    PH15: (ph15, 15, [3, 12, 13, 14, 15]),
]);

#[cfg(feature = "gpio-h72")]
gpio!(GPIOJ, gpioj, Gpioj, PJ, 'J', PJn, [
    PJ8: (pj8, 8, [1, 3, 8, 14, 15]),
    PJ9: (pj9, 9, [1, 3, 8, 14, 15]),
    PJ10: (pj10, 10, [1, 3, 5, 14, 15]),
    PJ11: (pj11, 11, [1, 3, 5, 14, 15]),
]);

#[cfg(feature = "gpio-h72")]
gpio!(GPIOK, gpiok, Gpiok, PK, 'K', PKn, [
    PK0: (pk0, 0, [1, 3, 5, 14, 15]),
    PK1: (pk1, 1, [1, 3, 5, 14, 15]),
    PK2: (pk2, 2, [1, 3, 10, 11, 14, 15]),
]);

#[cfg(feature = "gpio-h747")]
gpio!(GPIOA, gpioa, Gpioa, PA, 'A', PAn, [
    PA0: (pa0, 0, [1, 2, 3, 4, 7, 8, 9, 10, 11, 15]),
    PA1: (pa1, 1, [1, 2, 3, 4, 7, 8, 9, 10, 11, 14, 15]),
    PA2: (pa2, 2, [1, 2, 3, 4, 7, 8, 11, 12, 14, 15]),
    PA3: (pa3, 3, [1, 2, 3, 4, 7, 9, 10, 11, 14, 15]),
    PA4: (pa4, 4, [2, 5, 6, 7, 8, 12, 13, 14, 15]),
    PA5: (pa5, 5, [1, 3, 5, 8, 10, 14, 15]),
    PA6: (pa6, 6, [1, 2, 3, 5, 8, 9, 10, 11, 12, 13, 14, 15]),
    PA7: (pa7, 7, [1, 2, 3, 5, 8, 9, 11, 12, 15]),
    PA8: (pa8, 8, [0, 1, 2, 3, 4, 7, 10, 11, 12, 13, 14, 15]),
    PA9: (pa9, 9, [1, 2, 3, 4, 5, 7, 13, 14, 15]),
    PA10: (pa10, 10, [1, 2, 3, 7, 10, 11, 12, 13, 14, 15]),
    PA11: (pa11, 11, [1, 2, 3, 5, 6, 7, 9, 10, 14, 15]),
    PA12: (pa12, 12, [1, 2, 3, 5, 6, 7, 8, 9, 10, 14, 15]),
    PA13: (pa13, 13, [0, 15], super::Debugger),
    PA14: (pa14, 14, [0, 15], super::Debugger),
    PA15: (pa15, 15, [0, 1, 2, 4, 5, 6, 7, 8, 11, 13, 15], super::Debugger),
]);

#[cfg(feature = "gpio-h747")]
gpio!(GPIOB, gpiob, Gpiob, PB, 'B', PBn, [
    PB0: (pb0, 0, [1, 2, 3, 6, 8, 9, 10, 11, 14, 15]),
    PB1: (pb1, 1, [1, 2, 3, 6, 9, 10, 11, 14, 15]),
    PB2: (pb2, 2, [0, 2, 4, 6, 7, 8, 9, 10, 15]),
    PB3: (pb3, 3, [0, 1, 2, 5, 6, 8, 9, 10, 11, 15], super::Debugger),
    PB4: (pb4, 4, [0, 1, 2, 3, 5, 6, 7, 8, 9, 11, 15], super::Debugger),
    PB5: (pb5, 5, [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]),
    PB6: (pb6, 6, [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]),
    PB7: (pb7, 7, [1, 2, 3, 4, 6, 7, 8, 11, 12, 13, 15]),
    PB8: (pb8, 8, [1, 2, 3, 4, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]),
    PB9: (pb9, 9, [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]),
    PB10: (pb10, 10, [1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 14, 15]),
    PB11: (pb11, 11, [1, 2, 3, 4, 6, 7, 10, 11, 13, 14, 15]),
    PB12: (pb12, 12, [1, 4, 5, 6, 7, 9, 10, 11, 12, 13, 14, 15]),
    PB13: (pb13, 13, [1, 3, 5, 6, 7, 9, 10, 11, 14, 15]),
    PB14: (pb14, 14, [1, 2, 3, 4, 5, 6, 7, 8, 9, 12, 15]),
    PB15: (pb15, 15, [0, 1, 2, 3, 4, 5, 6, 8, 9, 12, 15]),
]);

#[cfg(feature = "gpio-h747")]
gpio!(GPIOC, gpioc, Gpioc, PC, 'C', PCn, [
    PC0: (pc0, 0, [3, 6, 8, 10, 12, 14, 15]),
    PC1: (pc1, 1, [0, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 15]),
    PC2: (pc2, 2, [3, 5, 6, 10, 11, 12, 15]),
    PC3: (pc3, 3, [3, 5, 10, 11, 12, 15]),
    PC4: (pc4, 4, [3, 5, 9, 11, 12, 15]),
    PC5: (pc5, 5, [2, 3, 9, 10, 11, 12, 13, 15]),
    PC6: (pc6, 6, [1, 2, 3, 4, 5, 7, 8, 9, 10, 12, 13, 14, 15]),
    PC7: (pc7, 7, [0, 1, 2, 3, 4, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]),
    PC8: (pc8, 8, [0, 1, 2, 3, 7, 8, 9, 11, 12, 13, 15]),
    PC9: (pc9, 9, [0, 2, 3, 4, 5, 8, 9, 10, 11, 12, 13, 14, 15]),
    PC10: (pc10, 10, [2, 3, 6, 7, 8, 9, 12, 13, 14, 15]),
    PC11: (pc11, 11, [2, 3, 6, 7, 8, 9, 12, 13, 15]),
    PC12: (pc12, 12, [0, 2, 6, 7, 8, 12, 13, 15]),
    PC13: (pc13, 13, [15]),
    PC14: (pc14, 14, [15]),
    PC15: (pc15, 15, [15]),
]);

#[cfg(feature = "gpio-h747")]
gpio!(GPIOD, gpiod, Gpiod, PD, 'D', PDn, [
    PD0: (pd0, 0, [3, 6, 8, 9, 12, 15]),
    PD1: (pd1, 1, [3, 6, 8, 9, 12, 15]),
    PD2: (pd2, 2, [0, 2, 8, 12, 13, 15]),
    PD3: (pd3, 3, [3, 5, 7, 12, 13, 14, 15]),
    PD4: (pd4, 4, [2, 6, 7, 12, 15]),
    PD5: (pd5, 5, [2, 7, 12, 15]),
    PD6: (pd6, 6, [2, 3, 4, 5, 6, 7, 8, 10, 11, 12, 13, 14, 15]),
    PD7: (pd7, 7, [3, 5, 6, 7, 9, 11, 12, 15]),
    PD8: (pd8, 8, [3, 6, 7, 9, 12, 15]),
    PD9: (pd9, 9, [3, 6, 7, 12, 15]),
    PD10: (pd10, 10, [3, 6, 7, 12, 14, 15]),
    PD11: (pd11, 11, [3, 4, 7, 9, 10, 12, 15]),
    PD12: (pd12, 12, [1, 2, 3, 4, 7, 9, 10, 12, 15]),
    PD13: (pd13, 13, [1, 2, 4, 9, 10, 12, 15]),
    PD14: (pd14, 14, [2, 6, 8, 12, 15]),
    PD15: (pd15, 15, [2, 6, 8, 12, 15]),
]);

#[cfg(feature = "gpio-h747")]
gpio!(GPIOE, gpioe, Gpioe, PE, 'E', PEn, [
    PE0: (pe0, 0, [1, 2, 3, 4, 8, 10, 12, 13, 15]),
    PE1: (pe1, 1, [1, 3, 8, 12, 13, 15]),
    PE2: (pe2, 2, [0, 2, 5, 6, 8, 9, 10, 11, 12, 15]),
    PE3: (pe3, 3, [0, 4, 6, 8, 12, 15]),
    PE4: (pe4, 4, [0, 2, 3, 4, 5, 6, 8, 10, 12, 13, 14, 15]),
    PE5: (pe5, 5, [0, 2, 3, 4, 5, 6, 8, 10, 12, 13, 14, 15]),
    PE6: (pe6, 6, [0, 1, 2, 4, 5, 6, 8, 9, 10, 11, 12, 13, 14, 15]),
    PE7: (pe7, 7, [1, 3, 7, 10, 12, 15]),
    PE8: (pe8, 8, [1, 3, 7, 10, 12, 13, 15]),
    PE9: (pe9, 9, [1, 3, 7, 10, 12, 15]),
    PE10: (pe10, 10, [1, 3, 7, 10, 12, 15]),
    PE11: (pe11, 11, [1, 3, 5, 10, 12, 14, 15]),
    PE12: (pe12, 12, [1, 3, 5, 10, 12, 13, 14, 15]),
    PE13: (pe13, 13, [1, 3, 5, 10, 12, 13, 14, 15]),
    PE14: (pe14, 14, [1, 5, 10, 12, 14, 15]),
    PE15: (pe15, 15, [1, 12, 13, 14, 15]),
]);

#[cfg(feature = "gpio-h747")]
gpio!(GPIOF, gpiof, Gpiof, PF, 'F', PFn, [
    PF0: (pf0, 0, [4, 12, 15]),
    PF1: (pf1, 1, [4, 12, 15]),
    PF2: (pf2, 2, [4, 12, 15]),
    PF3: (pf3, 3, [12, 15]),
    PF4: (pf4, 4, [12, 15]),
    PF5: (pf5, 5, [12, 15]),
    PF6: (pf6, 6, [1, 5, 6, 7, 8, 9, 15]),
    PF7: (pf7, 7, [1, 5, 6, 7, 8, 9, 15]),
    PF8: (pf8, 8, [1, 5, 6, 7, 8, 9, 10, 15]),
    PF9: (pf9, 9, [1, 5, 6, 7, 8, 9, 10, 15]),
    PF10: (pf10, 10, [1, 2, 9, 10, 13, 14, 15]),
    PF11: (pf11, 11, [5, 10, 12, 13, 15]),
    PF12: (pf12, 12, [12, 15]),
    PF13: (pf13, 13, [3, 4, 12, 15]),
    PF14: (pf14, 14, [3, 4, 12, 15]),
    PF15: (pf15, 15, [4, 12, 15]),
]);

#[cfg(feature = "gpio-h747")]
gpio!(GPIOG, gpiog, Gpiog, PG, 'G', PGn, [
    PG0: (pg0, 0, [12, 15]),
    PG1: (pg1, 1, [12, 15]),
    PG2: (pg2, 2, [3, 11, 12, 15]),
    PG3: (pg3, 3, [3, 11, 12, 15]),
    PG4: (pg4, 4, [1, 11, 12, 15]),
    PG5: (pg5, 5, [1, 12, 15]),
    PG6: (pg6, 6, [1, 2, 10, 12, 13, 14, 15]),
    PG7: (pg7, 7, [2, 6, 7, 12, 13, 14, 15]),
    PG8: (pg8, 8, [3, 5, 7, 8, 11, 12, 14, 15]),
    PG9: (pg9, 9, [5, 7, 8, 9, 10, 12, 13, 15]),
    PG10: (pg10, 10, [2, 5, 9, 10, 12, 13, 14, 15]),
    PG11: (pg11, 11, [1, 2, 5, 8, 10, 11, 13, 14, 15]),
    PG12: (pg12, 12, [1, 2, 5, 7, 8, 9, 11, 12, 14, 15]),
    PG13: (pg13, 13, [0, 1, 2, 5, 7, 11, 12, 14, 15]),
    PG14: (pg14, 14, [0, 1, 5, 7, 9, 11, 12, 14, 15]),
    PG15: (pg15, 15, [7, 12, 13, 15]),
]);

#[cfg(feature = "gpio-h747")]
gpio!(GPIOH, gpioh, Gpioh, PH, 'H', PHn, [
    PH0: (ph0, 0, [15]),
    PH1: (ph1, 1, [15]),
    PH2: (ph2, 2, [1, 9, 10, 11, 12, 14, 15]),
    PH3: (ph3, 3, [9, 10, 11, 12, 14, 15]),
    PH4: (ph4, 4, [4, 9, 10, 14, 15]),
    PH5: (ph5, 5, [4, 5, 12, 15]),
    PH6: (ph6, 6, [2, 4, 5, 11, 12, 13, 15]),
    PH7: (ph7, 7, [4, 5, 11, 12, 13, 15]),
    PH8: (ph8, 8, [2, 4, 12, 13, 14, 15]),
    PH9: (ph9, 9, [2, 4, 12, 13, 14, 15]),
    PH10: (ph10, 10, [2, 4, 12, 13, 14, 15]),
    PH11: (ph11, 11, [2, 4, 12, 13, 14, 15]),
    PH12: (ph12, 12, [2, 4, 12, 13, 14, 15]),
    PH13: (ph13, 13, [3, 8, 9, 12, 14, 15]),
    PH14: (ph14, 14, [3, 8, 9, 12, 13, 14, 15]),
    PH15: (ph15, 15, [3, 12, 13, 14, 15]),
]);

#[cfg(feature = "gpio-h747")]
gpio!(GPIOI, gpioi, Gpioi, PI, 'I', PIn, [
    PI0: (pi0, 0, [2, 5, 12, 13, 14, 15]),
    PI1: (pi1, 1, [3, 5, 11, 12, 13, 14, 15]),
    PI2: (pi2, 2, [3, 5, 12, 13, 14, 15]),
    PI3: (pi3, 3, [3, 5, 12, 13, 15]),
    PI4: (pi4, 4, [3, 10, 11, 12, 13, 14, 15]),
    PI5: (pi5, 5, [3, 10, 12, 13, 14, 15]),
    PI6: (pi6, 6, [3, 10, 12, 13, 14, 15]),
    PI7: (pi7, 7, [3, 10, 12, 13, 14, 15]),
    PI8: (pi8, 8, [15]),
    PI9: (pi9, 9, [8, 9, 12, 14, 15]),
    PI10: (pi10, 10, [11, 12, 14, 15]),
    PI11: (pi11, 11, [9, 10, 15]),
    PI12: (pi12, 12, [14, 15]),
    PI13: (pi13, 13, [14, 15]),
    PI14: (pi14, 14, [14, 15]),
    PI15: (pi15, 15, [9, 14, 15]),
]);

#[cfg(feature = "gpio-h747")]
gpio!(GPIOJ, gpioj, Gpioj, PJ, 'J', PJn, [
    PJ0: (pj0, 0, [9, 14, 15]),
    PJ1: (pj1, 1, [14, 15]),
    PJ2: (pj2, 2, [13, 14, 15]),
    PJ3: (pj3, 3, [14, 15]),
    PJ4: (pj4, 4, [14, 15]),
    PJ5: (pj5, 5, [14, 15]),
    PJ6: (pj6, 6, [3, 14, 15]),
    PJ7: (pj7, 7, [0, 3, 14, 15]),
    PJ8: (pj8, 8, [1, 3, 8, 14, 15]),
    PJ9: (pj9, 9, [1, 3, 8, 14, 15]),
    PJ10: (pj10, 10, [1, 3, 5, 14, 15]),
    PJ11: (pj11, 11, [1, 3, 5, 14, 15]),
    PJ12: (pj12, 12, [0, 9, 14, 15]),
    PJ13: (pj13, 13, [9, 14, 15]),
    PJ14: (pj14, 14, [14, 15]),
    PJ15: (pj15, 15, [14, 15]),
]);

#[cfg(feature = "gpio-h747")]
gpio!(GPIOK, gpiok, Gpiok, PK, 'K', PKn, [
    PK0: (pk0, 0, [1, 3, 5, 14, 15]),
    PK1: (pk1, 1, [1, 3, 5, 14, 15]),
    PK2: (pk2, 2, [1, 3, 10, 11, 14, 15]),
    PK3: (pk3, 3, [14, 15]),
    PK4: (pk4, 4, [14, 15]),
    PK5: (pk5, 5, [14, 15]),
    PK6: (pk6, 6, [14, 15]),
    PK7: (pk7, 7, [14, 15]),
]);

#[cfg(feature = "gpio-h7a2")]
gpio!(GPIOA, gpioa, Gpioa, PA, 'A', PAn, [
    PA0: (pa0, 0, [1, 2, 3, 4, 5, 7, 8, 9, 10, 15]),
    PA1: (pa1, 1, [1, 2, 3, 4, 7, 8, 9, 10, 11, 14, 15]),
    PA2: (pa2, 2, [1, 2, 4, 6, 7, 8, 12, 14, 15]),
    PA3: (pa3, 3, [1, 2, 3, 4, 5, 7, 9, 10, 14, 15]),
    PA4: (pa4, 4, [2, 5, 6, 7, 8, 13, 14, 15]),
    PA5: (pa5, 5, [0, 1, 3, 5, 8, 10, 13, 14, 15]),
    PA6: (pa6, 6, [1, 2, 3, 5, 6, 8, 9, 10, 11, 12, 13, 14, 15]),
    PA7: (pa7, 7, [1, 2, 3, 4, 5, 8, 9, 10, 12, 14, 15]),
    PA8: (pa8, 8, [0, 1, 3, 4, 7, 10, 11, 12, 13, 14, 15]),
    PA9: (pa9, 9, [1, 3, 4, 5, 7, 13, 14, 15]),
    PA10: (pa10, 10, [1, 3, 7, 10, 11, 12, 13, 14, 15]),
    PA11: (pa11, 11, [1, 3, 5, 6, 7, 9, 14, 15]),
    PA12: (pa12, 12, [1, 3, 5, 6, 7, 8, 9, 14, 15]),
    PA13: (pa13, 13, [0, 15], super::Debugger),
    PA14: (pa14, 14, [0, 15], super::Debugger),
    PA15: (pa15, 15, [0, 1, 4, 5, 6, 7, 8, 9, 11, 14, 15], super::Debugger),
]);

#[cfg(feature = "gpio-h7a2")]
gpio!(GPIOB, gpiob, Gpiob, PB, 'B', PBn, [
    PB0: (pb0, 0, [1, 2, 3, 4, 6, 8, 9, 10, 11, 14, 15]),
    PB1: (pb1, 1, [1, 2, 3, 6, 9, 10, 11, 14, 15]),
    PB2: (pb2, 2, [0, 2, 4, 6, 7, 9, 10, 15]),
    PB3: (pb3, 3, [0, 1, 5, 6, 8, 9, 10, 11, 15], super::Debugger),
    PB4: (pb4, 4, [1, 2, 5, 6, 7, 8, 9, 11, 15], super::Debugger),
    PB5: (pb5, 5, [1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]),
    PB6: (pb6, 6, [1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]),
    PB7: (pb7, 7, [1, 2, 4, 6, 7, 8, 11, 12, 13, 15]),
    PB8: (pb8, 8, [1, 2, 3, 4, 6, 7, 8, 9, 10, 12, 13, 14, 15]),
    PB9: (pb9, 9, [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]),
    PB10: (pb10, 10, [1, 3, 4, 5, 6, 7, 9, 10, 14, 15]),
    PB11: (pb11, 11, [1, 3, 4, 6, 7, 10, 14, 15]),
    PB12: (pb12, 12, [1, 3, 4, 5, 6, 7, 9, 10, 11, 13, 14, 15]),
    PB13: (pb13, 13, [1, 3, 4, 5, 6, 7, 9, 10, 12, 13, 14, 15]),
    PB14: (pb14, 14, [1, 2, 3, 4, 5, 6, 7, 8, 9, 14, 15]),
    PB15: (pb15, 15, [0, 1, 2, 3, 4, 5, 6, 8, 9, 14, 15]),
]);

#[cfg(feature = "gpio-h7a2")]
gpio!(GPIOC, gpioc, Gpioc, PC, 'C', PCn, [
    PC0: (pc0, 0, [3, 6, 8, 9, 10, 11, 12, 14, 15]),
    PC1: (pc1, 1, [0, 2, 3, 4, 5, 6, 9, 10, 12, 14, 15]),
    PC2: (pc2, 2, [0, 3, 5, 6, 9, 10, 11, 12, 15]),
    PC3: (pc3, 3, [0, 3, 5, 9, 10, 11, 12, 15]),
    PC4: (pc4, 4, [3, 5, 9, 12, 14, 15]),
    PC5: (pc5, 5, [2, 3, 4, 9, 10, 12, 13, 14, 15]),
    PC6: (pc6, 6, [2, 3, 4, 5, 7, 8, 9, 10, 12, 13, 14, 15]),
    PC7: (pc7, 7, [0, 2, 3, 4, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]),
    PC8: (pc8, 8, [0, 2, 3, 7, 8, 9, 10, 11, 12, 13, 15]),
    PC9: (pc9, 9, [0, 2, 3, 4, 5, 8, 9, 10, 11, 12, 13, 14, 15]),
    PC10: (pc10, 10, [3, 4, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]),
    PC11: (pc11, 11, [3, 4, 6, 7, 8, 9, 12, 13, 14, 15]),
    PC12: (pc12, 12, [0, 2, 4, 5, 6, 7, 8, 12, 13, 14, 15]),
    PC13: (pc13, 13, [15]),
    PC14: (pc14, 14, [15]),
    PC15: (pc15, 15, [15]),
]);

#[cfg(feature = "gpio-h7a2")]
gpio!(GPIOD, gpiod, Gpiod, PD, 'D', PDn, [
    PD0: (pd0, 0, [3, 8, 9, 11, 12, 14, 15]),
    PD1: (pd1, 1, [3, 8, 9, 12, 15]),
    PD2: (pd2, 2, [0, 2, 4, 8, 9, 12, 13, 14, 15]),
    PD3: (pd3, 3, [3, 5, 7, 12, 13, 14, 15]),
    PD4: (pd4, 4, [7, 10, 12, 15]),
    PD5: (pd5, 5, [7, 10, 12, 15]),
    PD6: (pd6, 6, [2, 3, 4, 5, 6, 7, 10, 11, 12, 13, 14, 15]),
    PD7: (pd7, 7, [3, 5, 6, 7, 9, 10, 11, 12, 15]),
    PD8: (pd8, 8, [3, 7, 9, 12, 15]),
    PD9: (pd9, 9, [3, 7, 12, 15]),
    PD10: (pd10, 10, [3, 4, 7, 12, 14, 15]),
    PD11: (pd11, 11, [3, 4, 7, 9, 10, 12, 15]),
    PD12: (pd12, 12, [1, 2, 3, 4, 7, 9, 10, 12, 13, 15]),
    PD13: (pd13, 13, [1, 2, 4, 9, 10, 11, 12, 13, 15]),
    PD14: (pd14, 14, [2, 8, 11, 12, 15]),
    PD15: (pd15, 15, [2, 8, 11, 12, 15]),
]);

#[cfg(feature = "gpio-h7a2")]
gpio!(GPIOE, gpioe, Gpioe, PE, 'E', PEn, [
    PE0: (pe0, 0, [1, 2, 4, 8, 10, 12, 13, 14, 15]),
    PE1: (pe1, 1, [1, 8, 12, 13, 14, 15]),
    PE2: (pe2, 2, [0, 2, 5, 6, 9, 11, 12, 15]),
    PE3: (pe3, 3, [0, 4, 6, 11, 12, 15]),
    PE4: (pe4, 4, [0, 2, 3, 4, 5, 6, 12, 13, 14, 15]),
    PE5: (pe5, 5, [0, 2, 3, 4, 5, 6, 12, 13, 14, 15]),
    PE6: (pe6, 6, [0, 1, 2, 4, 5, 6, 10, 11, 12, 13, 14, 15]),
    PE7: (pe7, 7, [1, 3, 7, 10, 12, 15]),
    PE8: (pe8, 8, [1, 3, 7, 10, 12, 13, 15]),
    PE9: (pe9, 9, [1, 3, 7, 10, 12, 15]),
    PE10: (pe10, 10, [1, 3, 7, 10, 12, 15]),
    PE11: (pe11, 11, [1, 3, 5, 10, 11, 12, 14, 15]),
    PE12: (pe12, 12, [1, 3, 5, 10, 12, 13, 14, 15]),
    PE13: (pe13, 13, [1, 3, 5, 10, 12, 13, 14, 15]),
    PE14: (pe14, 14, [1, 5, 10, 12, 14, 15]),
    PE15: (pe15, 15, [1, 11, 12, 13, 14, 15]),
]);

#[cfg(feature = "gpio-h7a2")]
gpio!(GPIOF, gpiof, Gpiof, PF, 'F', PFn, [
    PF0: (pf0, 0, [4, 9, 12, 15]),
    PF1: (pf1, 1, [4, 9, 12, 15]),
    PF2: (pf2, 2, [4, 9, 12, 15]),
    PF3: (pf3, 3, [9, 12, 15]),
    PF4: (pf4, 4, [9, 12, 15]),
    PF5: (pf5, 5, [9, 12, 15]),
    PF6: (pf6, 6, [1, 5, 6, 7, 10, 15]),
    PF7: (pf7, 7, [1, 5, 6, 7, 10, 15]),
    PF8: (pf8, 8, [1, 5, 6, 7, 9, 10, 15]),
    PF9: (pf9, 9, [1, 5, 6, 7, 9, 10, 15]),
    PF10: (pf10, 10, [1, 2, 4, 9, 13, 14, 15]),
    PF11: (pf11, 11, [5, 9, 10, 12, 13, 15]),
    PF12: (pf12, 12, [9, 12, 15]),
    PF13: (pf13, 13, [3, 4, 12, 15]),
    PF14: (pf14, 14, [3, 4, 12, 15]),
    PF15: (pf15, 15, [4, 12, 15]),
]);

#[cfg(feature = "gpio-h7a2")]
gpio!(GPIOG, gpiog, Gpiog, PG, 'G', PGn, [
    PG0: (pg0, 0, [9, 11, 12, 15]),
    PG1: (pg1, 1, [9, 11, 12, 15]),
    PG2: (pg2, 2, [3, 11, 12, 15]),
    PG3: (pg3, 3, [3, 11, 12, 15]),
    PG4: (pg4, 4, [1, 11, 12, 15]),
    PG5: (pg5, 5, [1, 12, 15]),
    PG6: (pg6, 6, [1, 10, 12, 13, 14, 15]),
    PG7: (pg7, 7, [6, 7, 9, 12, 13, 14, 15]),
    PG8: (pg8, 8, [3, 5, 7, 8, 12, 14, 15]),
    PG9: (pg9, 9, [5, 7, 8, 9, 10, 11, 12, 13, 15]),
    PG10: (pg10, 10, [3, 5, 9, 10, 11, 12, 13, 14, 15]),
    PG11: (pg11, 11, [1, 5, 8, 9, 10, 11, 13, 14, 15]),
    PG12: (pg12, 12, [1, 3, 5, 7, 8, 9, 10, 11, 12, 14, 15]),
    PG13: (pg13, 13, [0, 1, 5, 7, 10, 11, 12, 14, 15]),
    PG14: (pg14, 14, [0, 1, 5, 7, 9, 10, 11, 12, 14, 15]),
    PG15: (pg15, 15, [7, 9, 11, 12, 13, 15]),
]);

#[cfg(feature = "gpio-h7a2")]
gpio!(GPIOH, gpioh, Gpioh, PH, 'H', PHn, [
    PH0: (ph0, 0, [15]),
    PH1: (ph1, 1, [15]),
    PH2: (ph2, 2, [1, 9, 10, 12, 14, 15]),
    PH3: (ph3, 3, [9, 10, 12, 14, 15]),
    PH4: (ph4, 4, [4, 9, 10, 13, 14, 15]),
    PH5: (ph5, 5, [4, 5, 12, 15]),
    PH6: (ph6, 6, [2, 4, 5, 12, 13, 15]),
    PH7: (ph7, 7, [4, 5, 12, 13, 15]),
    PH8: (ph8, 8, [2, 4, 12, 13, 14, 15]),
    PH9: (ph9, 9, [2, 4, 12, 13, 14, 15]),
    PH10: (ph10, 10, [2, 4, 12, 13, 14, 15]),
    PH11: (ph11, 11, [2, 4, 12, 13, 14, 15]),
    PH12: (ph12, 12, [2, 4, 12, 13, 14, 15]),
    PH13: (ph13, 13, [3, 8, 9, 12, 14, 15]),
    PH14: (ph14, 14, [3, 8, 9, 12, 13, 14, 15]),
    PH15: (ph15, 15, [3, 12, 13, 14, 15]),
]);

#[cfg(feature = "gpio-h7a2")]
gpio!(GPIOI, gpioi, Gpioi, PI, 'I', PIn, [
    PI0: (pi0, 0, [2, 5, 12, 13, 14, 15]),
    PI1: (pi1, 1, [3, 5, 11, 12, 13, 14, 15]),
    PI2: (pi2, 2, [3, 5, 12, 13, 14, 15]),
    PI3: (pi3, 3, [3, 5, 12, 13, 15]),
    PI4: (pi4, 4, [3, 10, 11, 12, 13, 14, 15]),
    PI5: (pi5, 5, [3, 10, 12, 13, 14, 15]),
    PI6: (pi6, 6, [3, 10, 12, 13, 14, 15]),
    PI7: (pi7, 7, [3, 10, 12, 13, 14, 15]),
    PI8: (pi8, 8, [15]),
    PI9: (pi9, 9, [3, 8, 9, 12, 14, 15]),
    PI10: (pi10, 10, [3, 12, 13, 14, 15]),
    PI11: (pi11, 11, [3, 9, 10, 13, 15]),
    PI12: (pi12, 12, [3, 14, 15]),
    PI13: (pi13, 13, [3, 14, 15]),
    PI14: (pi14, 14, [3, 14, 15]),
    PI15: (pi15, 15, [9, 14, 15]),
]);

#[cfg(feature = "gpio-h7a2")]
gpio!(GPIOJ, gpioj, Gpioj, PJ, 'J', PJn, [
    PJ0: (pj0, 0, [9, 14, 15]),
    PJ1: (pj1, 1, [3, 14, 15]),
    PJ2: (pj2, 2, [3, 14, 15]),
    PJ3: (pj3, 3, [11, 14, 15]),
    PJ4: (pj4, 4, [11, 14, 15]),
    PJ5: (pj5, 5, [14, 15]),
    PJ6: (pj6, 6, [3, 14, 15]),
    PJ7: (pj7, 7, [0, 3, 14, 15]),
    PJ8: (pj8, 8, [1, 3, 8, 14, 15]),
    PJ9: (pj9, 9, [1, 3, 8, 14, 15]),
    PJ10: (pj10, 10, [1, 3, 5, 14, 15]),
    PJ11: (pj11, 11, [1, 3, 5, 14, 15]),
    PJ12: (pj12, 12, [0, 9, 14, 15]),
    PJ13: (pj13, 13, [9, 14, 15]),
    PJ14: (pj14, 14, [14, 15]),
    PJ15: (pj15, 15, [14, 15]),
]);

#[cfg(feature = "gpio-h7a2")]
gpio!(GPIOK, gpiok, Gpiok, PK, 'K', PKn, [
    PK0: (pk0, 0, [1, 3, 5, 14, 15]),
    PK1: (pk1, 1, [1, 3, 5, 14, 15]),
    PK2: (pk2, 2, [1, 3, 10, 11, 14, 15]),
    PK3: (pk3, 3, [3, 14, 15]),
    PK4: (pk4, 4, [3, 14, 15]),
    PK5: (pk5, 5, [3, 14, 15]),
    PK6: (pk6, 6, [3, 14, 15]),
    PK7: (pk7, 7, [14, 15]),
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
            #[cfg(not(feature = "gpio-h72"))]
            'I' => crate::pac::GPIOI::ptr() as _,
            'J' => crate::pac::GPIOJ::ptr() as _,
            'K' => crate::pac::GPIOK::ptr() as _,
            _ => panic!("Unknown GPIO port"),
        }
    }
}
