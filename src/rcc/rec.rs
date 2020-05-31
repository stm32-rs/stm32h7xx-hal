//! Peripheral Reset and Enable Control (REC)
//!
//! This module contains safe accessors to the RCC functionality for each
//! periperal.
//!
//! At a minimum each peripheral implements
//! [ResetEnable](trait.ResetEnable.html). Additionally those peripherals
//! that have a multiplexer in the PKSU may also have methods
//! `kernel_clk_mux` and `get_kernel_clk_mux`. These set and get the state
//! of the kernel clock multiplexer respectively.
//!
//! # Example
//!
//! ```
//! // Constrain and Freeze power
//! ...
//! let rcc = dp.RCC.constrain();
//! let ccdr = rcc.sys_ck(100.mhz()).freeze(vos, &dp.SYSCFG);
//!
//! // Enable the clock to a peripheral and reset it
//! ccdr.peripheral.FDCAN.enable().reset();
//! ```
#![deny(missing_docs)]

use core::marker::PhantomData;

use super::Rcc;
use crate::stm32::{rcc, RCC};
use cortex_m::interrupt;

/// A trait for Resetting, Enabling and Disabling a single peripheral
pub trait ResetEnable {
    /// Enable this peripheral
    fn enable(self) -> Self;
    /// Disable this peripheral
    fn disable(self) -> Self;
    /// Reset this peripheral
    fn reset(self) -> Self;
}

impl Rcc {
    /// Returns all the peripherals resets / enables / kernel clocks.
    ///
    /// # Use case
    ///
    /// Allows peripherals to be reset / enabled before the calling
    /// freeze. For example, the internal watchdog could be enabled to
    /// issue a reset if the call the freeze hangs waiting for an external
    /// clock that is stopped.
    ///
    /// # Safety
    ///
    /// If this method is called multiple times, or is called before the
    /// [freeze](struct.Rcc.html#freeze), then multiple accesses to the
    /// same memory exist.
    #[inline]
    pub unsafe fn steal_peripheral_rec(&self) -> PeripheralREC {
        PeripheralREC::new_singleton()
    }
}

macro_rules! peripheral_reset_and_enable_control {
    ($($AXBn:ident, $axb_doc:expr => [
        $(
            $( #[ $pmeta:meta ] )*
                $p:ident
            $( kernel_clk: $(($Variant:ident))* $ccip:ident $clk_doc:expr )*
        ),*
    ];)+) => {
        paste::item! {
            /// Peripheral Reset and Enable Control
            #[allow(non_snake_case)]
            #[non_exhaustive]
            pub struct PeripheralREC {
                $(
                    $(
                        #[allow(missing_docs)]
                        $( #[ $pmeta ] )*
                        pub [< $p:upper >]: $p,
                    )*
                )+
            }
            impl PeripheralREC {
                /// Return a new instance of the peripheral resets /
                /// enables / kernel clocks
                ///
                /// # Safety
                ///
                /// If this method is called multiple times, then multiple
                /// accesses to the same memory exist.
                pub(super) unsafe fn new_singleton() -> PeripheralREC {
                    PeripheralREC {
                        $(
                            $(
                                $( #[ $pmeta ] )*
                                [< $p:upper >]: $p {
                                    _marker: PhantomData,
                                },
                            )*
                        )+
                    }
                }
            }
            $(
                $(
                    /// Owned ability to Reset, Enable and Disable peripheral
                    $( #[ $pmeta ] )*
                    pub struct $p {
                        _marker: PhantomData<*const ()>,
                    }
                    $( #[ $pmeta ] )*
                    unsafe impl Send for $p {}
                    $( #[ $pmeta ] )*
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
                    $( #[ $pmeta ] )*
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

                            #[inline(always)]
                            #[allow(unused)]
                            /// Return the current kernel clock selection
                            pub fn get_kernel_clk_mux(&self) ->
                                variant_return_type!([< $p ClkSel >] $(, $Variant)*)
                            {
                                // unsafe: We only read from this bitfield
                                let ccip = unsafe {
                                    &(*RCC::ptr()).[< $ccip r >]
                                };
                                ccip.read().[< $p:lower sel >]().variant()
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

// If the PAC does not fully specify a CCIP field (perhaps because one or
// more values are reserved), then we use a different return type
macro_rules! variant_return_type {
    ($t:ty) => { $t };
    ($t:ty, $Variant: ident) => {
        stm32h7::Variant<u8, $t>
    };
}

// Must only contain peripherals that are not used anywhere in this HAL
peripheral_reset_and_enable_control! {
    AHB1, "AMBA High-performance Bus (AHB1) peripherals" => [
        Eth1Mac, Dma2, Dma1,
        #[cfg(any(feature = "dualcore"))] Art
    ];
    AHB2, "AMBA High-performance Bus (AHB2) peripherals" => [
        Sdmmc2, Hash, Crypt
    ];
    AHB3, "AMBA High-performance Bus (AHB3) peripherals" => [
        Sdmmc1,
        Qspi kernel_clk: d1ccip "QUADSPI kernel clock source selection",
        Fmc kernel_clk: d1ccip "FMC kernel clock source selection",
        Jpgdec, Dma2d, Mdma
    ];
    AHB4, "AMBA High-performance Bus (AHB4) peripherals" => [
        Hsem, Bdma, Crc
    ];
    APB1L, "Advanced Peripheral Bus 1L (APB1L) peripherals" => [
        Dac12,
        Cec
    ];
    APB1H, "Advanced Peripheral Bus 1H (APB1H) peripherals" => [
        Fdcan kernel_clk: (Variant) d2ccip1 "FDCAN kernel clock source selection",
        Swp kernel_clk: d2ccip1 "SWPMI kernel clock source selection",
        Crs, Mdios, Opamp
    ];
    APB2, "Advanced Peripheral Bus 2 (APB2) peripherals" => [
        Hrtim,
        Dfsdm1 kernel_clk: d2ccip1 "DFSDM1 kernel Clk source selection"
    ];
    APB3, "Advanced Peripheral Bus 3 (APB3) peripherals" => [
        Ltdc,
        #[cfg(any(feature = "dsi"))] Dsi
    ];
    APB4, "Advanced Peripheral Bus 4 (APB4) peripherals" => [
        Vref, Comp12
    ];
}
