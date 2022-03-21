//! General Purpose Input / Output
//!
//! # Examples
//!
//! - [Simple Blinky](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/blinky.rs)
//! - [Digital Read](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/digital_read.rs)
//! - [External Interrupt via Button](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/exti_interrupt.rs)

use core::marker::PhantomData;

use crate::rcc::ResetEnable;
use crate::stm32::{EXTI, SYSCFG};

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

pub struct Alternate<MODE> {
    _mode: PhantomData<MODE>,
}

/// Input mode (type state)
pub struct Input<MODE> {
    _mode: PhantomData<MODE>,
}

/// Floating input (type state)
pub struct Floating;
/// Pulled down input (type state)
pub struct PullDown;
/// Pulled up input (type state)
pub struct PullUp;

/// Output mode (type state)
pub struct Output<MODE> {
    _mode: PhantomData<MODE>,
}

/// Push pull output (type state)
pub struct PushPull;
/// Open drain output (type state)
pub struct OpenDrain;

/// Analog mode (type state)
pub struct Analog;

/// GPIO Pin speed selection
#[derive(Copy, Clone, PartialEq)]
pub enum Speed {
    Low = 0,
    Medium = 1,
    High = 2,
    VeryHigh = 3,
}

/// GPIO Edge selection
#[derive(Copy, Clone, PartialEq)]
pub enum Edge {
    Rising,
    Falling,
    RisingFalling,
}

/// Alternate function 0 (type state)
pub struct AF0;
/// Alternate function 1 (type state)
pub struct AF1;
/// Alternate function 2 (type state)
pub struct AF2;
/// Alternate function 3 (type state)
pub struct AF3;
/// Alternate function 4 (type state)
pub struct AF4;
/// Alternate function 5 (type state)
pub struct AF5;
/// Alternate function 6 (type state)
pub struct AF6;
/// Alternate function 7 (type state)
pub struct AF7;
/// Alternate function 8 (type state)
pub struct AF8;
/// Alternate function 9 (type state)
pub struct AF9;
/// Alternate function 10 (type state)
pub struct AF10;
/// Alternate function 11 (type state)
pub struct AF11;
/// Alternate function 12 (type state)
pub struct AF12;
/// Alternate function 13 (type state)
pub struct AF13;
/// Alternate function 14 (type state)
pub struct AF14;
/// Alternate function 15 (type state)
pub struct AF15;

/// External Interrupt Pin
pub trait ExtiPin {
    fn make_interrupt_source(&mut self, syscfg: &mut SYSCFG);
    fn trigger_on_edge(&mut self, exti: &mut EXTI, level: Edge);
    fn enable_interrupt(&mut self, exti: &mut EXTI);
    fn disable_interrupt(&mut self, exti: &mut EXTI);
    fn clear_interrupt_pending_bit(&mut self);
}

macro_rules! gpio {
    ($GPIOX:ident, $gpiox:ident, $gpio_doc:expr,
     $Rec:ident, $PXx:ident, $extigpionr:expr, [
         $($PXi:ident: ($pxi:ident, $i:expr, $MODE:ty, $exticri:ident),)+
     ]) => {
        #[doc=$gpio_doc]
        pub mod $gpiox {
            use core::marker::PhantomData;

            use embedded_hal::digital::v2::{InputPin, OutputPin, IoPin, PinState,
                                            StatefulOutputPin, toggleable};

            use crate::rcc::{rec, ResetEnable};
            use crate::stm32::$GPIOX;
            use crate::stm32::{EXTI, SYSCFG};
            use super::{
                Alternate, Floating, GpioExt, Input, OpenDrain,
                Output, Speed, PullDown, PullUp, PushPull, AF0, AF1,
                AF2, AF3, AF4, AF5, AF6, AF7, AF8, AF9, AF10, AF11,
                AF12, AF13, AF14, AF15, Analog, Edge, ExtiPin, erased::ErasedPin, };

            use crate::Never;

            /// GPIO parts
            pub struct Parts {
                $(
                    /// Pin
                    pub $pxi: $PXi<$MODE>,
                )+
            }

            impl GpioExt for $GPIOX {
                type Parts = Parts;
                type Rec = rec::$Rec;

                fn split(self, prec: rec::$Rec) -> Parts {
                    prec.enable().reset();

                    Parts {
                        $(
                            $pxi: $PXi { _mode: PhantomData },
                        )+
                    }
                }

                fn split_without_reset(self, prec: rec::$Rec) -> Parts {
                    prec.enable();

                    Parts {
                        $(
                            $pxi: $PXi { _mode: PhantomData },
                        )+
                    }
                }
            }

            /// Partially erased pin
            pub struct $PXx<MODE> {
                i: u8,
                _mode: PhantomData<MODE>,
            }

            impl<MODE> $PXx<MODE> {
                pub fn get_id(&self) -> u8 {
                    self.i
                }
            }

            impl<MODE> OutputPin for $PXx<Output<MODE>> {
                type Error = Never;

                fn set_high(&mut self) -> Result<(), Never> {
                    // NOTE(unsafe) atomic write to a stateless
                    // register
                    unsafe { (*$GPIOX::ptr()).bsrr
                               .write(|w| w.bits(1 << self.i)) }

                    Ok(())
                }

                fn set_low(&mut self) -> Result<(), Never> {
                    // NOTE(unsafe) atomic write to a stateless register
                    unsafe { (*$GPIOX::ptr()).bsrr
                               .write(|w| w.bits(1 << (self.i + 16))) }

                    Ok(())
                }
            }

            impl<MODE> StatefulOutputPin for $PXx<Output<MODE>> {
                fn is_set_high(&self) -> Result<bool, Never> {
                    self.is_set_low().map(|v| !v)
                }

                fn is_set_low(&self) -> Result<bool, Never> {
                    // NOTE(unsafe) atomic read with no side effects
                    Ok(unsafe { (*$GPIOX::ptr()).odr
                                  .read().bits() & (1 << self.i) } == 0)
                }
            }

            impl<MODE> toggleable::Default for $PXx<Output<MODE>> {}

            impl<MODE> InputPin for $PXx<Output<MODE>> {
                type Error = Never;

                fn is_high(&self) -> Result<bool, Never> {
                    self.is_low().map(|v| !v)
                }

                fn is_low(&self) -> Result<bool, Never> {
                    // NOTE(unsafe) atomic read with no side effects
                    Ok(unsafe { (*$GPIOX::ptr()).idr
                                  .read().bits() & (1 << self.i) } == 0)
                }
            }

            impl<MODE> InputPin for $PXx<Input<MODE>> {
                type Error = Never;

                fn is_high(&self) -> Result<bool, Never> {
                    self.is_low().map(|v| !v)
                }

                fn is_low(&self) -> Result<bool, Never> {
                    // NOTE(unsafe) atomic read with no side effects
                    Ok(unsafe { (*$GPIOX::ptr()).idr
                                  .read().bits() & (1 << self.i) } == 0)
                }
            }

            impl<MODE> InputPin for $PXx<Alternate<MODE>> {
                type Error = Never;

                fn is_high(&self) -> Result<bool, Never> {
                    self.is_low().map(|v| !v)
                }

                fn is_low(&self) -> Result<bool, Never> {
                    // NOTE(unsafe) atomic read with no side effects
                    Ok(unsafe { (*$GPIOX::ptr()).idr
                                  .read().bits() & (1 << self.i) } == 0)
                }
            }

            impl<MODE> ExtiPin for $PXx<Input<MODE>> {
                /// Make corresponding EXTI line sensitive to this pin
                fn make_interrupt_source(&mut self, syscfg: &mut SYSCFG) {
                    let offset = 4 * (self.i % 4);
                    match self.i {
                        0..=3 => {
                            syscfg.exticr1.modify(|r, w| unsafe {
                                w.bits((r.bits() & !(0xf << offset)) | ($extigpionr << offset))
                            });
                        },
                        4..=7 => {
                            syscfg.exticr2.modify(|r, w| unsafe {
                                w.bits((r.bits() & !(0xf << offset)) | ($extigpionr << offset))
                            });
                        },
                        8..=11 => {
                            syscfg.exticr3.modify(|r, w| unsafe {
                                w.bits((r.bits() & !(0xf << offset)) | ($extigpionr << offset))
                            });
                        },
                        12..=15 => {
                            syscfg.exticr4.modify(|r, w| unsafe {
                                w.bits((r.bits() & !(0xf << offset)) | ($extigpionr << offset))
                            });
                        },
                        _ => {}
                    }
                }

                /// Generate interrupt on rising edge, falling edge or both
                fn trigger_on_edge(&mut self, exti: &mut EXTI, edge: Edge) {
                    match edge {
                        Edge::Rising => {
                            exti.rtsr1.modify(|r, w| unsafe { w.bits(r.bits() | (1 << self.i)) });
                            exti.ftsr1.modify(|r, w| unsafe { w.bits(r.bits() & !(1 << self.i)) });
                        },
                        Edge::Falling => {
                            exti.ftsr1.modify(|r, w| unsafe { w.bits(r.bits() | (1 << self.i)) });
                            exti.rtsr1.modify(|r, w| unsafe { w.bits(r.bits() & !(1 << self.i)) });
                        },
                        Edge::RisingFalling => {
                            exti.rtsr1.modify(|r, w| unsafe { w.bits(r.bits() | (1 << self.i)) });
                            exti.ftsr1.modify(|r, w| unsafe { w.bits(r.bits() | (1 << self.i)) });
                        }
                    }
                }

                /// Enable external interrupts from this pin.
                fn enable_interrupt(&mut self, exti: &mut EXTI) {
                    #[cfg(not(feature = "rm0399"))]
                    let imr1 = &exti.cpuimr1;
                    #[cfg(all(feature = "rm0399", feature = "cm7"))]
                    let imr1 = &exti.c1imr1;
                    #[cfg(all(feature = "rm0399", feature = "cm4"))]
                    let imr1 = &exti.c2imr1;

                    imr1.modify(|r, w| unsafe { w.bits(r.bits() | (1 << self.i)) });
                }

                /// Disable external interrupts from this pin
                fn disable_interrupt(&mut self, exti: &mut EXTI) {
                    #[cfg(not(feature = "rm0399"))]
                    let imr1 = &exti.cpuimr1;
                    #[cfg(all(feature = "rm0399", feature = "cm7"))]
                    let imr1 = &exti.c1imr1;
                    #[cfg(all(feature = "rm0399", feature = "cm4"))]
                    let imr1 = &exti.c2imr1;

                    imr1.modify(|r, w| unsafe { w.bits(r.bits() & !(1 << self.i)) });
                }

                /// Clear the interrupt pending bit for this pin
                fn clear_interrupt_pending_bit(&mut self) {
                    unsafe {
                        #[cfg(not(feature = "rm0399"))]
                        let pr1 = &(*EXTI::ptr()).cpupr1;
                        #[cfg(all(feature = "rm0399", feature = "cm7"))]
                        let pr1 = &(*EXTI::ptr()).c1pr1;
                        #[cfg(all(feature = "rm0399", feature = "cm4"))]
                        let pr1 = &(*EXTI::ptr()).c2pr1;

                        pr1.write(|w| w.bits(1 << self.i));
                        let _ = pr1.read();
                        let _ = pr1.read(); // Delay 2 peripheral clocks
                    }
                }
            }

            fn _set_alternate_mode (index: usize, mode: u32)
            {
                let offset = 2 * index;
                let offset2 = 4 * index;
                unsafe {
                    if offset2 < 32 {
                        (*$GPIOX::ptr()).afrl.modify(|r, w| {
                            w.bits((r.bits() & !(0b1111 << offset2)) | (mode << offset2))
                        });
                    } else {
                        let offset2 = offset2 - 32;
                        (*$GPIOX::ptr()).afrh.modify(|r, w| {
                            w.bits((r.bits() & !(0b1111 << offset2)) | (mode << offset2))
                        });
                    }
                    (*$GPIOX::ptr()).moder.modify(|r, w| {
                        w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset))
                    });
                }
            }

            $(
                /// Pin
                pub struct $PXi<MODE> {
                    _mode: PhantomData<MODE>,
                }

                impl<MODE> $PXi<MODE> {
                    /// Configures the pin to operate in AF0 mode
                    pub fn into_alternate_af0(self) -> $PXi<Alternate<AF0>> {
                        _set_alternate_mode($i, 0);
                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate in AF1 mode
                    pub fn into_alternate_af1(self) -> $PXi<Alternate<AF1>> {
                        _set_alternate_mode($i, 1);
                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate in AF2 mode
                    pub fn into_alternate_af2(self) -> $PXi<Alternate<AF2>> {
                        _set_alternate_mode($i, 2);
                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate in AF3 mode
                    pub fn into_alternate_af3(self) -> $PXi<Alternate<AF3>> {
                        _set_alternate_mode($i, 3);
                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate in AF4 mode
                    pub fn into_alternate_af4(self) -> $PXi<Alternate<AF4>> {
                        _set_alternate_mode($i, 4);
                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate in AF5 mode
                    pub fn into_alternate_af5(self) -> $PXi<Alternate<AF5>> {
                        _set_alternate_mode($i, 5);
                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate in AF6 mode
                    pub fn into_alternate_af6(self) -> $PXi<Alternate<AF6>> {
                        _set_alternate_mode($i, 6);
                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate in AF7 mode
                    pub fn into_alternate_af7(self) -> $PXi<Alternate<AF7>> {
                        _set_alternate_mode($i, 7);
                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate in AF8 mode
                    pub fn into_alternate_af8(self) -> $PXi<Alternate<AF8>> {
                        _set_alternate_mode($i, 8);
                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate in AF9 mode
                    pub fn into_alternate_af9(self) -> $PXi<Alternate<AF9>> {
                        _set_alternate_mode($i, 9);
                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate in AF10 mode
                    pub fn into_alternate_af10(self) -> $PXi<Alternate<AF10>> {
                        _set_alternate_mode($i, 10);
                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate in AF11 mode
                    pub fn into_alternate_af11(self) -> $PXi<Alternate<AF11>> {
                        _set_alternate_mode($i, 11);
                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate in AF12 mode
                    pub fn into_alternate_af12(self) -> $PXi<Alternate<AF12>> {
                        _set_alternate_mode($i, 12);
                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate in AF13 mode
                    pub fn into_alternate_af13(self) -> $PXi<Alternate<AF13>> {
                        _set_alternate_mode($i, 13);
                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate in AF14 mode
                    pub fn into_alternate_af14(self) -> $PXi<Alternate<AF14>> {
                        _set_alternate_mode($i, 14);
                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate in AF15 mode
                    pub fn into_alternate_af15(self) -> $PXi<Alternate<AF15>> {
                        _set_alternate_mode($i, 15);
                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate as a floating
                    /// input pin
                    pub fn into_floating_input(self) -> $PXi<Input<Floating>> {
                        let offset = 2 * $i;
                        unsafe {
                            (*$GPIOX::ptr()).pupdr.modify(|r, w| {
                                w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset))
                            });
                            (*$GPIOX::ptr()).moder.modify(|r, w| {
                                w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset))
                            })
                        };

                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate as a pulled down
                    /// input pin
                    pub fn into_pull_down_input(self) -> $PXi<Input<PullDown>> {
                        let offset = 2 * $i;
                        unsafe {
                            (*$GPIOX::ptr()).pupdr.modify(|r, w| {
                                w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset))
                            });
                            (*$GPIOX::ptr()).moder.modify(|r, w| {
                                w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset))
                            })
                        };

                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate as a pulled up
                    /// input pin
                    pub fn into_pull_up_input(self) -> $PXi<Input<PullUp>> {
                        let offset = 2 * $i;
                        unsafe {
                            (*$GPIOX::ptr()).pupdr.modify(|r, w| {
                                w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset))
                            });
                            (*$GPIOX::ptr()).moder.modify(|r, w| {
                                w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset))
                            }
                            )};

                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate as an open drain
                    /// output pin
                    pub fn into_open_drain_output(self) -> $PXi<Output<OpenDrain>> {
                        let offset = 2 * $i;
                        unsafe {
                            (*$GPIOX::ptr()).pupdr.modify(|r, w| {
                                w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset))
                            });
                            (*$GPIOX::ptr()).otyper.modify(|r, w| {
                                w.bits(r.bits() | (0b1 << $i))
                            });
                            (*$GPIOX::ptr()).moder.modify(|r, w| {
                                w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset))
                            })
                        };

                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate as an push pull
                    /// output pin
                    pub fn into_push_pull_output(self) -> $PXi<Output<PushPull>> {
                        let offset = 2 * $i;

                        unsafe {
                            (*$GPIOX::ptr()).pupdr.modify(|r, w| {
                                w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset))
                            });
                            (*$GPIOX::ptr()).otyper.modify(|r, w| {
                                w.bits(r.bits() & !(0b1 << $i))
                            });
                            (*$GPIOX::ptr()).moder.modify(|r, w| {
                                w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset))
                            })
                        };

                        $PXi { _mode: PhantomData }
                    }

                    /// Configures the pin to operate as an analog
                    /// input pin
                    pub fn into_analog(self) -> $PXi<Analog> {
                        let offset = 2 * $i;

                        unsafe {
                            (*$GPIOX::ptr()).pupdr.modify(|r, w| {
                                w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset))
                            });
                            (*$GPIOX::ptr()).moder.modify(|r, w| {
                                w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset))
                            }
                            )};

                        $PXi { _mode: PhantomData }
                    }
                }

                impl<MODE> $PXi<Output<MODE>> {
                    /// Set pin speed
                    pub fn set_speed(self, speed: Speed) -> Self {
                        let offset = 2 * $i;

                        unsafe {
                            (*$GPIOX::ptr()).ospeedr.modify(|r, w| {
                                w.bits((r.bits() & !(0b11 << offset)) | ((speed as u32) << offset))
                            })
                        };

                        self
                    }
                }

                impl $PXi<Output<OpenDrain>> {
                    /// Enables / disables the internal pull up
                    pub fn internal_pull_up(&mut self, on: bool) {
                        let offset = 2 * $i;
                        let value = if on { 0b01 } else { 0b00 };
                        unsafe {
                            (*$GPIOX::ptr()).pupdr.modify(|r, w| {
                                w.bits((r.bits() & !(0b11 << offset)) | (value << offset))
                            })
                        };
                    }
                }

                impl<MODE> $PXi<Alternate<MODE>> {
                    /// Set pin speed
                    pub fn set_speed(self, speed: Speed) -> Self {
                        let offset = 2 * $i;

                        unsafe {
                            (*$GPIOX::ptr()).ospeedr.modify(|r, w| {
                                w.bits((r.bits() & !(0b11 << offset)) | ((speed as u32) << offset))
                            })
                        };

                        self
                    }

                    /// Enables / disables the internal pull up
                    pub fn internal_pull_up(self, on: bool) -> Self {
                        let offset = 2 * $i;
                        let value = if on { 0b01 } else { 0b00 };
                        unsafe {
                            (*$GPIOX::ptr()).pupdr.modify(|r, w| {
                                w.bits((r.bits() & !(0b11 << offset)) | (value << offset))
                            })
                        };

                        self
                    }
                }

                impl<MODE> $PXi<Alternate<MODE>> {
                    /// Turns pin alternate configuration pin into
                    /// open drain
                    pub fn set_open_drain(self) -> Self {
                        let offset = $i;
                        unsafe {
                            (*$GPIOX::ptr()).otyper.modify(|r, w| {
                                w.bits(r.bits() | (1 << offset))
                            })
                        };

                        self
                    }
                }

                impl<MODE> $PXi<MODE> {
                    /// Erases both the pin number and port from the type
                    ///
                    /// This is useful when you want to collect the
                    /// pins into an array where you need all the
                    /// elements to have the same type
                    pub fn erase(&self) -> ErasedPin<MODE> {
                        ErasedPin::new($extigpionr, $i)
                        // ErasedPin {
                        //      pin_port: $extigpionr << 4 | $i,
                        //      _mode: self._mode,
                        // }
                    }

                    /// Erases the pin number from the type
                    ///
                    /// This is useful when you want to collect the
                    /// pins into an array where you need all the
                    /// elements to have the same type
                    pub fn downgrade(self) -> $PXx<MODE> {
                        $PXx {
                            i: $i,
                            _mode: self._mode,
                        }
                    }
                }

                impl<MODE> OutputPin for $PXi<Output<MODE>> {
                    type Error = Never;

                    fn set_high(&mut self) -> Result<(), Never> {
                        // NOTE(unsafe) atomic write to a stateless
                        // register
                        unsafe { (*$GPIOX::ptr()).bsrr
                                   .write(|w| w.bits(1 << $i)) }

                        Ok(())
                    }

                    fn set_low(&mut self) -> Result<(), Never> {
                        // NOTE(unsafe) atomic write to a stateless
                        // register
                        unsafe { (*$GPIOX::ptr()).bsrr
                                   .write(|w| w.bits(1 << ($i + 16))) }

                        Ok(())
                    }
                }

                impl<MODE> StatefulOutputPin for $PXi<Output<MODE>> {
                    fn is_set_high(&self) -> Result<bool, Never> {
                        self.is_set_low().map(|v| !v)
                    }

                    fn is_set_low(&self) -> Result<bool, Never> {
                        // NOTE(unsafe) atomic read with no side effects
                        Ok(unsafe { (*$GPIOX::ptr()).odr
                                      .read().bits() & (1 << $i) } == 0)
                    }
                }

                impl<MODE> toggleable::Default for $PXi<Output<MODE>> {}

                impl<MODE> InputPin for $PXi<Output<MODE>> {
                    type Error = Never;

                    fn is_high(&self) -> Result<bool, Never> {
                        self.is_low().map(|v| !v)
                    }

                    fn is_low(&self) -> Result<bool, Never> {
                        // NOTE(unsafe) atomic read with no side effects
                        Ok(unsafe { (*$GPIOX::ptr()).idr
                                      .read().bits() & (1 << $i) } == 0)
                    }
                }

                impl<MODE> InputPin for $PXi<Input<MODE>> {
                    type Error = Never;

                    fn is_high(&self) -> Result<bool, Never> {
                        self.is_low().map(|v| !v)
                    }

                    fn is_low(&self) -> Result<bool, Never> {
                        // NOTE(unsafe) atomic read with no side effects
                        Ok(unsafe { (*$GPIOX::ptr()).idr
                                      .read().bits() & (1 << $i) } == 0)
                    }
                }

                impl<MODE> InputPin for $PXi<Alternate<MODE>> {
                    type Error = Never;

                    fn is_high(&self) -> Result<bool, Never> {
                        self.is_low().map(|v| !v)
                    }

                    fn is_low(&self) -> Result<bool, Never> {
                        // NOTE(unsafe) atomic read with no side effects
                        Ok(unsafe { (*$GPIOX::ptr()).idr
                                      .read().bits() & (1 << $i) } == 0)
                    }
                }

                impl IoPin<Self, Self>
                    for $PXi<Output<OpenDrain>>
                {
                    type Error = Never;
                    fn into_input_pin(self) -> Result<Self, Never> {
                        Ok(self)
                    }
                    fn into_output_pin(mut self, state: PinState) -> Result<Self, Never> {
                        self.set_state(state).unwrap(); // Infallible
                        Ok(self)
                    }
                }

                impl IoPin<$PXi<Input<Floating>>, Self>
                    for $PXi<Output<PushPull>>
                {
                    type Error = Never;
                    fn into_input_pin(self) -> Result<$PXi<Input<Floating>>, Never> {
                        Ok(self.into_floating_input())
                    }
                    fn into_output_pin(mut self, state: PinState) -> Result<Self, Never> {
                        self.set_state(state).unwrap(); // Infallible
                        Ok(self)
                    }
                }

                impl IoPin<Self, $PXi<Output<PushPull>>>
                    for $PXi<Input<Floating>>
                {
                    type Error = Never;
                    fn into_input_pin(self) -> Result<Self, Never> {
                        Ok(self)
                    }
                    fn into_output_pin(self, state: PinState) -> Result<$PXi<Output<PushPull>>, Never> {
                        let mut pin = self.into_push_pull_output();
                        pin.set_state(state).unwrap(); // Infallible
                        Ok(pin)
                    }
                }

                impl<MODE> ExtiPin for $PXi<Input<MODE>> {
                    /// Configure EXTI Line $i to trigger from this pin.
                    fn make_interrupt_source(&mut self, syscfg: &mut SYSCFG) {
                        let offset = 4 * ($i % 4);
                        syscfg.$exticri.modify(|r, w| unsafe {
                            let mut exticr = r.bits();
                            exticr = (exticr & !(0xf << offset)) | ($extigpionr << offset);
                            w.bits(exticr)
                        });
                    }

                    /// Generate interrupt on rising edge, falling edge or both
                    fn trigger_on_edge(&mut self, exti: &mut EXTI, edge: Edge) {
                        match edge {
                            Edge::Rising => {
                                exti.rtsr1.modify(|r, w| unsafe { w.bits(r.bits() | (1 << $i)) });
                                exti.ftsr1.modify(|r, w| unsafe { w.bits(r.bits() & !(1 << $i)) });
                            },
                            Edge::Falling => {
                                exti.ftsr1.modify(|r, w| unsafe { w.bits(r.bits() | (1 << $i)) });
                                exti.rtsr1.modify(|r, w| unsafe { w.bits(r.bits() & !(1 << $i)) });
                            },
                            Edge::RisingFalling => {
                                exti.rtsr1.modify(|r, w| unsafe { w.bits(r.bits() | (1 << $i)) });
                                exti.ftsr1.modify(|r, w| unsafe { w.bits(r.bits() | (1 << $i)) });
                            }
                        }
                    }

                    /// Enable external interrupts from this pin.
                    fn enable_interrupt(&mut self, exti: &mut EXTI) {
                        #[cfg(not(feature = "rm0399"))]
                        let imr1 = &exti.cpuimr1;
                        #[cfg(all(feature = "rm0399", feature = "cm7"))]
                        let imr1 = &exti.c1imr1;
                        #[cfg(all(feature = "rm0399", feature = "cm4"))]
                        let imr1 = &exti.c2imr1;

                        imr1.modify(|r, w| unsafe { w.bits(r.bits() | (1 << $i)) });
                    }

                    /// Disable external interrupts from this pin
                    fn disable_interrupt(&mut self, exti: &mut EXTI) {
                        #[cfg(not(feature = "rm0399"))]
                        let imr1 = &exti.cpuimr1;
                        #[cfg(all(feature = "rm0399", feature = "cm7"))]
                        let imr1 = &exti.c1imr1;
                        #[cfg(all(feature = "rm0399", feature = "cm4"))]
                        let imr1 = &exti.c2imr1;

                        imr1.modify(|r, w| unsafe { w.bits(r.bits() & !(1 << $i)) });
                    }

                    /// Clear the interrupt pending bit for this pin
                    fn clear_interrupt_pending_bit(&mut self) {
                        unsafe {
                            #[cfg(not(feature = "rm0399"))]
                            let pr1 = &(*(EXTI::ptr())).cpupr1;
                            #[cfg(all(feature = "rm0399", feature = "cm7"))]
                            let pr1 = &(*(EXTI::ptr())).c1pr1;
                            #[cfg(all(feature = "rm0399", feature = "cm4"))]
                            let pr1 = &(*(EXTI::ptr())).c2pr1;

                            pr1.write(|w| w.bits(1 << $i));
                            let _ = pr1.read();
                            let _ = pr1.read(); // Delay 2 peripheral clocks
                        }
                    }
                }
            )+
        }
    }
}

gpio!(GPIOA, gpioa, "Port A", Gpioa, PA, 0, [
    PA0: (pa0, 0, Analog, exticr1),
    PA1: (pa1, 1, Analog, exticr1),
    PA2: (pa2, 2, Analog, exticr1),
    PA3: (pa3, 3, Analog, exticr1),
    PA4: (pa4, 4, Analog, exticr2),
    PA5: (pa5, 5, Analog, exticr2),
    PA6: (pa6, 6, Analog, exticr2),
    PA7: (pa7, 7, Analog, exticr2),
    PA8: (pa8, 8, Analog, exticr3),
    PA9: (pa9, 9, Analog, exticr3),
    PA10: (pa10, 10, Analog, exticr3),
    PA11: (pa11, 11, Analog, exticr3),
    PA12: (pa12, 12, Analog, exticr4),
    PA13: (pa13, 13, Alternate<AF0>, exticr4), // JTMS/SWDIO
    PA14: (pa14, 14, Alternate<AF0>, exticr4), // JTCK/SWCLK
    PA15: (pa15, 15, Alternate<AF0>, exticr4), // JTDI
]);

gpio!(GPIOB, gpiob, "Port B", Gpiob, PB, 1, [
    PB0: (pb0, 0, Analog, exticr1),
    PB1: (pb1, 1, Analog, exticr1),
    PB2: (pb2, 2, Analog, exticr1),
    PB3: (pb3, 3, Alternate<AF0>, exticr1), // JTDO/TRACESWO
    PB4: (pb4, 4, Alternate<AF0>, exticr2), // !JTRST
    PB5: (pb5, 5, Analog, exticr2),
    PB6: (pb6, 6, Analog, exticr2),
    PB7: (pb7, 7, Analog, exticr2),
    PB8: (pb8, 8, Analog, exticr3),
    PB9: (pb9, 9, Analog, exticr3),
    PB10: (pb10, 10, Analog, exticr3),
    PB11: (pb11, 11, Analog, exticr3),
    PB12: (pb12, 12, Analog, exticr4),
    PB13: (pb13, 13, Analog, exticr4),
    PB14: (pb14, 14, Analog, exticr4),
    PB15: (pb15, 15, Analog, exticr4),
]);

gpio!(GPIOC, gpioc, "Port C", Gpioc, PC, 2, [
    PC0: (pc0, 0, Analog, exticr1),
    PC1: (pc1, 1, Analog, exticr1),
    PC2: (pc2, 2, Analog, exticr1),
    PC3: (pc3, 3, Analog, exticr1),
    PC4: (pc4, 4, Analog, exticr2),
    PC5: (pc5, 5, Analog, exticr2),
    PC6: (pc6, 6, Analog, exticr2),
    PC7: (pc7, 7, Analog, exticr2),
    PC8: (pc8, 8, Analog, exticr3),
    PC9: (pc9, 9, Analog, exticr3),
    PC10: (pc10, 10, Analog, exticr3),
    PC11: (pc11, 11, Analog, exticr3),
    PC12: (pc12, 12, Analog, exticr4),
    PC13: (pc13, 13, Analog, exticr4),
    PC14: (pc14, 14, Analog, exticr4),
    PC15: (pc15, 15, Analog, exticr4),
]);

gpio!(GPIOD, gpiod, "Port D", Gpiod, PD, 3, [
    PD0: (pd0, 0, Analog, exticr1),
    PD1: (pd1, 1, Analog, exticr1),
    PD2: (pd2, 2, Analog, exticr1),
    PD3: (pd3, 3, Analog, exticr1),
    PD4: (pd4, 4, Analog, exticr2),
    PD5: (pd5, 5, Analog, exticr2),
    PD6: (pd6, 6, Analog, exticr2),
    PD7: (pd7, 7, Analog, exticr2),
    PD8: (pd8, 8, Analog, exticr3),
    PD9: (pd9, 9, Analog, exticr3),
    PD10: (pd10, 10, Analog, exticr3),
    PD11: (pd11, 11, Analog, exticr3),
    PD12: (pd12, 12, Analog, exticr4),
    PD13: (pd13, 13, Analog, exticr4),
    PD14: (pd14, 14, Analog, exticr4),
    PD15: (pd15, 15, Analog, exticr4),
]);

gpio!(GPIOE, gpioe, "Port E", Gpioe, PE, 4, [
    PE0: (pe0, 0, Analog, exticr1),
    PE1: (pe1, 1, Analog, exticr1),
    PE2: (pe2, 2, Analog, exticr1),
    PE3: (pe3, 3, Analog, exticr1),
    PE4: (pe4, 4, Analog, exticr2),
    PE5: (pe5, 5, Analog, exticr2),
    PE6: (pe6, 6, Analog, exticr2),
    PE7: (pe7, 7, Analog, exticr2),
    PE8: (pe8, 8, Analog, exticr3),
    PE9: (pe9, 9, Analog, exticr3),
    PE10: (pe10, 10, Analog, exticr3),
    PE11: (pe11, 11, Analog, exticr3),
    PE12: (pe12, 12, Analog, exticr4),
    PE13: (pe13, 13, Analog, exticr4),
    PE14: (pe14, 14, Analog, exticr4),
    PE15: (pe15, 15, Analog, exticr4),
]);

gpio!(GPIOF, gpiof, "Port F", Gpiof, PF, 5, [
    PF0: (pf0, 0, Analog, exticr1),
    PF1: (pf1, 1, Analog, exticr1),
    PF2: (pf2, 2, Analog, exticr1),
    PF3: (pf3, 3, Analog, exticr1),
    PF4: (pf4, 4, Analog, exticr2),
    PF5: (pf5, 5, Analog, exticr2),
    PF6: (pf6, 6, Analog, exticr2),
    PF7: (pf7, 7, Analog, exticr2),
    PF8: (pf8, 8, Analog, exticr3),
    PF9: (pf9, 9, Analog, exticr3),
    PF10: (pf10, 10, Analog, exticr3),
    PF11: (pf11, 11, Analog, exticr3),
    PF12: (pf12, 12, Analog, exticr4),
    PF13: (pf13, 13, Analog, exticr4),
    PF14: (pf14, 14, Analog, exticr4),
    PF15: (pf15, 15, Analog, exticr4),
]);

gpio!(GPIOG, gpiog, "Port G", Gpiog, PG, 6, [
    PG0: (pg0, 0, Analog, exticr1),
    PG1: (pg1, 1, Analog, exticr1),
    PG2: (pg2, 2, Analog, exticr1),
    PG3: (pg3, 3, Analog, exticr1),
    PG4: (pg4, 4, Analog, exticr2),
    PG5: (pg5, 5, Analog, exticr2),
    PG6: (pg6, 6, Analog, exticr2),
    PG7: (pg7, 7, Analog, exticr2),
    PG8: (pg8, 8, Analog, exticr3),
    PG9: (pg9, 9, Analog, exticr3),
    PG10: (pg10, 10, Analog, exticr3),
    PG11: (pg11, 11, Analog, exticr3),
    PG12: (pg12, 12, Analog, exticr4),
    PG13: (pg13, 13, Analog, exticr4),
    PG14: (pg14, 14, Analog, exticr4),
    PG15: (pg15, 15, Analog, exticr4),
]);

gpio!(GPIOH, gpioh, "Port H", Gpioh, PH, 7, [
    PH0: (ph0, 0, Analog, exticr1),
    PH1: (ph1, 1, Analog, exticr1),
    PH2: (ph2, 2, Analog, exticr1),
    PH3: (ph3, 3, Analog, exticr1),
    PH4: (ph4, 4, Analog, exticr2),
    PH5: (ph5, 5, Analog, exticr2),
    PH6: (ph6, 6, Analog, exticr2),
    PH7: (ph7, 7, Analog, exticr2),
    PH8: (ph8, 8, Analog, exticr3),
    PH9: (ph9, 9, Analog, exticr3),
    PH10: (ph10, 10, Analog, exticr3),
    PH11: (ph11, 11, Analog, exticr3),
    PH12: (ph12, 12, Analog, exticr4),
    PH13: (ph13, 13, Analog, exticr4),
    PH14: (ph14, 14, Analog, exticr4),
    PH15: (ph15, 15, Analog, exticr4),
]);

#[cfg(not(feature = "rm0468"))]
gpio!(GPIOI, gpioi, "Port I", Gpioi, PI, 8, [
    PI0: (pi0, 0, Analog, exticr1),
    PI1: (pi1, 1, Analog, exticr1),
    PI2: (pi2, 2, Analog, exticr1),
    PI3: (pi3, 3, Analog, exticr1),
    PI4: (pi4, 4, Analog, exticr2),
    PI5: (pi5, 5, Analog, exticr2),
    PI6: (pi6, 6, Analog, exticr2),
    PI7: (pi7, 7, Analog, exticr2),
    PI8: (pi8, 8, Analog, exticr3),
    PI9: (pi9, 9, Analog, exticr3),
    PI10: (pi10, 10, Analog, exticr3),
    PI11: (pi11, 11, Analog, exticr3),
    PI12: (pi12, 12, Analog, exticr4),
    PI13: (pi13, 13, Analog, exticr4),
    PI14: (pi14, 14, Analog, exticr4),
    PI15: (pi15, 15, Analog, exticr4),
]);

gpio!(GPIOJ, gpioj, "Port J", Gpioj, PJ, 9, [
    PJ0: (pj0, 0, Analog, exticr1),
    PJ1: (pj1, 1, Analog, exticr1),
    PJ2: (pj2, 2, Analog, exticr1),
    PJ3: (pj3, 3, Analog, exticr1),
    PJ4: (pj4, 4, Analog, exticr2),
    PJ5: (pj5, 5, Analog, exticr2),
    PJ6: (pj6, 6, Analog, exticr2),
    PJ7: (pj7, 7, Analog, exticr2),
    PJ8: (pj8, 8, Analog, exticr3),
    PJ9: (pj9, 9, Analog, exticr3),
    PJ10: (pj10, 10, Analog, exticr3),
    PJ11: (pj11, 11, Analog, exticr3),
    PJ12: (pj12, 12, Analog, exticr4),
    PJ13: (pj13, 13, Analog, exticr4),
    PJ14: (pj14, 14, Analog, exticr4),
    PJ15: (pj15, 15, Analog, exticr4),
]);

gpio!(GPIOK, gpiok, "Port K", Gpiok, PK, 10, [
    PK0: (pk0, 0, Analog, exticr1),
    PK1: (pk1, 1, Analog, exticr1),
    PK2: (pk2, 2, Analog, exticr1),
    PK3: (pk3, 3, Analog, exticr1),
    PK4: (pk4, 4, Analog, exticr2),
    PK5: (pk5, 5, Analog, exticr2),
    PK6: (pk6, 6, Analog, exticr2),
    PK7: (pk7, 7, Analog, exticr2),
    PK8: (pk8, 8, Analog, exticr3),
    PK9: (pk9, 9, Analog, exticr3),
    PK10: (pk10, 10, Analog, exticr3),
    PK11: (pk11, 11, Analog, exticr3),
    PK12: (pk12, 12, Analog, exticr4),
    PK13: (pk13, 13, Analog, exticr4),
    PK14: (pk14, 14, Analog, exticr4),
    PK15: (pk15, 15, Analog, exticr4),
]);

pub mod erased {
    use super::{
        Alternate, Analog, Floating, Input, OpenDrain, Output, PullDown,
        PullUp, PushPull,
    };
    use crate::Never;
    use core::fmt;
    use core::marker::PhantomData;
    use embedded_hal::digital::v2::{
        toggleable, InputPin, OutputPin, StatefulOutputPin,
    };
    pub type EPin<MODE> = ErasedPin<MODE>;

    /// Fully erased pin
    ///
    /// `MODE` is one of the pin modes (see [Modes](crate::gpio#modes) section).
    pub struct ErasedPin<MODE> {
        // Bits 0-3: Pin, Bits 4-7: Port
        pin_port: u8,
        _mode: PhantomData<MODE>,
    }

    impl<MODE> ErasedPin<MODE> {
        pub(crate) fn new(port: u8, pin: u8) -> Self {
            Self {
                pin_port: port << 4 | pin,
                _mode: PhantomData,
            }
        }

        #[inline(always)]
        fn pin_id(&self) -> u8 {
            self.pin_port & 0x0f
        }
        #[inline(always)]
        fn port_id(&self) -> u8 {
            self.pin_port >> 4
        }

        #[inline]
        fn block(&self) -> &crate::pac::gpioa::RegisterBlock {
            // This function uses pointer arithmetic instead of branching to be more efficient

            // The logic relies on the following assumptions:
            // - GPIOA register is available on all chips
            // - all gpio register blocks have the same layout
            // - consecutive gpio register blocks have the same offset between them, namely 0x0400
            // - ErasedPin::new was called with a valid port

            // FIXME could be calculated after const_raw_ptr_to_usize_cast stabilization #51910
            const GPIO_REGISTER_OFFSET: usize = 0x0400;

            let offset = GPIO_REGISTER_OFFSET * self.port_id() as usize;
            unsafe { &*crate::pac::GPIOA::ptr().add(offset) }
        }

        /// Configures the pin to operate as a floating
        /// input pin
        pub fn into_floating_input(self) -> EPin<Input<Floating>> {
            let offset = 2 * self.pin_id();
            let block = self.block();

            unsafe {
                block.pupdr.modify(|r, w| {
                    w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset))
                });
                block.moder.modify(|r, w| {
                    w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset))
                })
            };

            EPin {
                pin_port: self.pin_port,
                _mode: PhantomData,
            }
        }

        /// Configures the pin to operate as a pulled down
        /// input pin
        pub fn into_pull_down_input(self) -> EPin<Input<PullDown>> {
            let offset = 2 * self.pin_id();
            let block = self.block();

            unsafe {
                block.pupdr.modify(|r, w| {
                    w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset))
                });
                block.moder.modify(|r, w| {
                    w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset))
                })
            };

            EPin {
                pin_port: self.pin_port,
                _mode: PhantomData,
            }
        }

        /// Configures the pin to operate as a pulled up
        /// input pin
        pub fn into_pull_up_input(self) -> EPin<Input<PullUp>> {
            let offset = 2 * self.pin_id();
            let block = self.block();

            unsafe {
                block.pupdr.modify(|r, w| {
                    w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset))
                });
                block.moder.modify(|r, w| {
                    w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset))
                })
            };

            EPin {
                pin_port: self.pin_port,
                _mode: PhantomData,
            }
        }

        /// Configures the pin to operate as an open drain
        /// output pin
        pub fn into_open_drain_output(self) -> EPin<Output<OpenDrain>> {
            let offset = 2 * self.pin_id();
            let block = self.block();

            unsafe {
                block.pupdr.modify(|r, w| {
                    w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset))
                });
                block
                    .otyper
                    .modify(|r, w| w.bits(r.bits() | (0b1 << self.pin_id())));
                block.moder.modify(|r, w| {
                    w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset))
                })
            };

            EPin {
                pin_port: self.pin_port,
                _mode: PhantomData,
            }
        }

        /// Configures the pin to operate as an push pull
        /// output pin
        pub fn into_push_pull_output(self) -> EPin<Output<PushPull>> {
            let offset = 2 * self.pin_id();
            let block = self.block();

            unsafe {
                block.pupdr.modify(|r, w| {
                    w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset))
                });
                block
                    .otyper
                    .modify(|r, w| w.bits(r.bits() & !(0b1 << self.pin_id())));
                block.moder.modify(|r, w| {
                    w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset))
                })
            };

            EPin {
                pin_port: self.pin_port,
                _mode: PhantomData,
            }
        }

        /// Configures the pin to operate as an analog
        /// input pin
        pub fn into_analog(self) -> EPin<Analog> {
            let offset = 2 * self.pin_id();
            let block = self.block();

            unsafe {
                block.pupdr.modify(|r, w| {
                    w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset))
                });
                block.moder.modify(|r, w| {
                    w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset))
                })
            };

            EPin {
                pin_port: self.pin_port,
                _mode: PhantomData,
            }
        }
    }

    impl<MODE> OutputPin for ErasedPin<Output<MODE>> {
        type Error = Never;

        #[inline(always)]
        fn set_high(&mut self) -> Result<(), Never> {
            // NOTE(unsafe) atomic write to a stateless register
            unsafe { self.block().bsrr.write(|w| w.bits(1 << self.pin_id())) };
            Ok(())
        }

        #[inline(always)]
        fn set_low(&mut self) -> Result<(), Never> {
            // NOTE(unsafe) atomic write to a stateless register
            unsafe {
                self.block()
                    .bsrr
                    .write(|w| w.bits(1 << (self.pin_id() + 16)))
            };
            Ok(())
        }
    }

    impl<MODE> StatefulOutputPin for ErasedPin<Output<MODE>> {
        #[inline(always)]
        fn is_set_high(&self) -> Result<bool, Never> {
            self.is_set_low().map(|x| !x)
        }

        #[inline(always)]
        fn is_set_low(&self) -> Result<bool, Never> {
            Ok(self.block().odr.read().bits() & (1 << self.pin_id()) == 0)
        }
    }

    impl<MODE> toggleable::Default for ErasedPin<Output<MODE>> {}
    impl<MODE> InputPin for ErasedPin<Output<MODE>> {
        type Error = Never;

        #[inline(always)]
        fn is_high(&self) -> Result<bool, Never> {
            self.is_low().map(|x| !x)
        }

        #[inline(always)]
        fn is_low(&self) -> Result<bool, Never> {
            Ok(self.block().idr.read().bits() & (1 << self.pin_id()) == 0)
        }
    }

    impl<MODE> InputPin for ErasedPin<Input<MODE>> {
        type Error = Never;

        #[inline(always)]
        fn is_high(&self) -> Result<bool, Never> {
            self.is_low().map(|x| !x)
        }

        #[inline(always)]
        fn is_low(&self) -> Result<bool, Never> {
            Ok(self.block().idr.read().bits() & (1 << self.pin_id()) == 0)
        }
    }

    impl<MODE> InputPin for ErasedPin<Alternate<MODE>> {
        type Error = Never;

        #[inline(always)]
        fn is_high(&self) -> Result<bool, Never> {
            self.is_low().map(|x| !x)
        }

        #[inline(always)]
        fn is_low(&self) -> Result<bool, Never> {
            Ok(self.block().idr.read().bits() & (1 << self.pin_id()) == 0)
        }
    }

    impl<MODE> fmt::Debug for ErasedPin<MODE> {
        fn fmt(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
            formatter.write_fmt(format_args!(
                "P({}{})",
                char::from(self.port_id() + b'A'),
                self.pin_id()
            ))
        }
    }
}
