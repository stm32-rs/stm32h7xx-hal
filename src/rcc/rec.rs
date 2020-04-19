//! Peripheral Reset and Enable Control
#![deny(missing_docs)]

use core::marker::PhantomData;

use super::Rcc;
use crate::stm32::{rcc, RCC};
use cortex_m::interrupt;

/// A trait for Resetting, Enabling and Disabling a single peripheral
pub trait ResetEnable {
    /// Enable this periperhal
    fn enable(self) -> Self;
    /// Disable this periperhal
    fn disable(self) -> Self;
    /// Reset this periperhal
    fn reset(self) -> Self;
}

// NOTE `no_mangle` is used here to prevent linking different minor
// versions of this crate as that would let you `take` more than once (one
// per minor version)
#[no_mangle]
static mut RCC_PERIPHERALS_REC: bool = false;
impl Rcc {
    /// Returns all the peripherals resets / enables *once*
    #[inline]
    pub fn take_peripherals(&self) -> Option<PeripheralsREC> {
        interrupt::free(|_| {
            if unsafe { RCC_PERIPHERALS_REC } {
                None
            } else {
                Some(unsafe { self.steal_periperhals() })
            }
        })
    }
}

macro_rules! peripheral_reset_and_enable_control {
    ($($AXBn:ident, $axb_doc:expr => [
        $( $p:ident $(kernel $ccip:ident $clk_doc:expr)* ),*
    ];)+) => {
        paste::item! {
            /// Peripheral Reset and Enable Control
            #[allow(non_snake_case)]
            pub struct PeripheralsREC {
                $(
                    $(
                        #[allow(missing_docs)]
                        pub [< $p:upper >]: $p,
                    )*
                )+

                // Private field making `Peripherals` non-exhaustive. We
                // don't use `#[non_exhaustive]` so we can support older
                // Rust versions.
                _priv: (),
            }
            impl Rcc {
                /// Unchecked version of `PeripheralsREC::take`
                #[inline]
                pub unsafe fn steal_periperhals(&self) -> PeripheralsREC {
                    RCC_PERIPHERALS_REC = true;

                    PeripheralsREC {
                        $(
                            $(
                                [< $p:upper >]: $p {
                                    _marker: PhantomData,
                                },
                            )*
                        )+
                            _priv: (),
                    }
                }
            }
            $(
                $(
                    /// Owned ability to Reset, Enable and Disable peripheral
                    pub struct $p {
                        _marker: PhantomData<*const ()>,
                    }
                    unsafe impl Send for $p {}
                    impl ResetEnable for $p {
                        #[inline(always)]
                        fn enable(self) -> Self {
                            // unsafe: Owned exclusive access to this bitfield
                            interrupt::free(|_| {
                                let enr = unsafe {
                                    &(*RCC::ptr()).[< $AXBn:lower enr >]
                                };
                                enr.modify(|_, w| w.
                                           [< $p:lower en >]().set_bit());
                            });
                            self
                        }
                        #[inline(always)]
                        fn disable(self) -> Self {
                            // unsafe: Owned exclusive access to this bitfield
                            interrupt::free(|_| {
                                let enr = unsafe {
                                    &(*RCC::ptr()).[< $AXBn:lower enr >]
                                };
                                enr.modify(|_, w| w.
                                           [< $p:lower en >]().clear_bit());
                            });
                            self
                        }
                        #[inline(always)]
                        fn reset(self) -> Self {
                            // unsafe: Owned exclusive access to this bitfield
                            interrupt::free(|_| {
                                let rstr = unsafe {
                                    &(*RCC::ptr()).[< $AXBn:lower rstr >]
                                };
                                rstr.modify(|_, w| w.
                                            [< $p:lower rst >]().set_bit());
                                rstr.modify(|_, w| w.
                                            [< $p:lower rst >]().clear_bit());
                            });
                            self
                        }
                    }
                    impl $p {
                        $(
                            #[inline(always)]
                            #[allow(unused)]
                            /// Modify a kernel clock for this
                            /// peripheral. See RM0433 Section 8.5.8.
                            ///
                            /// It is possible to switch this clock
                            /// dynamically without generating spurs or
                            /// timing violations. However, the user must
                            /// ensure that both clocks are running. See
                            /// RM0433 Section 8.5.10
                            pub fn kernel_clk_mux(self, sel: [< $p ClkSel >]) -> Self {
                                // unsafe: Owned exclusive access to this bitfield
                                interrupt::free(|_| {
                                    let ccip = unsafe {
                                        &(*RCC::ptr()).[< $ccip r >]
                                    };
                                    ccip.modify(|_, w| w.
                                                [< $p:lower sel >]().variant(sel));
                                });
                                self
                            }
                        )*
                    }

                    $(
                        #[doc=$clk_doc]
                        pub type [< $p ClkSel >] =
                            rcc::[< $ccip r >]::[< $p:upper SEL_A >];
                    )*
                )*
            )+
        }
    }
}

// Must only contain periperhals that are not used anywhere in this HAL
peripheral_reset_and_enable_control! {
    AHB1, "AMBA High-performance Bus (AHB1) peripherals" => [
        Eth1Mac, Art, Dma2, Dma1
    ];
    AHB2, "AMBA High-performance Bus (AHB2) peripherals" => [
        Sdmmc2, Hash, Crypt
    ];
    AHB3, "AMBA High-performance Bus (AHB3) peripherals" => [
        Sdmmc1,
        Qspi kernel d1ccip "QUADSPI kernel clock source selection",
        Fmc kernel d1ccip "FMC kernel clock source selection",
        Jpgdec, Dma2d, Mdma
    ];
    AHB4, "AMBA High-performance Bus (AHB4) peripherals" => [
        Hsem, Bdma, Crc
    ];
    APB1L, "Advanced Peripheral Bus 1L (APB1L) peripherals" => [
        Dac12, HdmiCec
    ];
    APB1H, "Advanced Peripheral Bus 1H (APB1H) peripherals" => [
        Fdcan kernel d2ccip1 "FDCAN kernel clock source selection",
        Swp kernel d2ccip1 "SWPMI kernel clock source selection",
        Crs, Mdios, Opamp
    ];
    APB2, "Advanced Peripheral Bus 2 (APB2) peripherals" => [
        Hrtim,
        Dfsdm1 kernel d2ccip1 "DFSDM1 kernel Clk source selection"
    ];
    APB3, "Advanced Peripheral Bus 3 (APB3) peripherals" => [
        Ltdc
    ];
    APB4, "Advanced Peripheral Bus 4 (APB4) peripherals" => [
        Vref, Comp12
    ];
}
