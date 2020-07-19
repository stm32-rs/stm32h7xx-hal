//! Peripheral Reset and Enable Control (REC)
//!
//! This module contains safe accessors to the RCC functionality for each
//! periperal.
//!
//! At a minimum each peripheral implements
//! [ResetEnable](trait.ResetEnable.html). Peripherals that have an
//! individual clock multiplexer in the PKSU also have methods
//! `kernel_clk_mux` and `get_kernel_clk_mux`. These set and get the state
//! of the kernel clock multiplexer respectively.
//!
//! Peripherals that share a clock multiplexer in the PKSU with other
//! peripherals implement a trait with a `get_kernel_clk_mux` method that
//! returns the current kernel clock state. There is currently no safe API
//! for setting these shared multiplexers.
//!
//! # Reset/Enable Example
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
//!
//! # Kernel Clock Example
//! ```
//! let ccdr = ...; // Returned by `freeze()`, see example above
//!
//! let cec_mux_state = ccdr.peripheral.CEC.kernel_clk_mux(CecClkSel::LSI).get_kernel_clk_mux();
//!
//! assert_eq!(cec_mux_state, CecClkSel::LSI);
//!
//! // Can't set this mux because it would also affect I2C2 and I2C3
//! let i2c_mux_state = ccdr.peripheral.I2C1.get_kernel_clk_mux();
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
                $([ kernel $clk:ident: $pk:ident $(($Variant:ident))* $ccip:ident $clk_doc:expr ])*
                $([ group clk: $pk_g:ident $( $(($Variant_g:ident))* $ccip_g:ident $clk_doc_g:expr )* ])*
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
                        pub(crate) _marker: PhantomData<*const ()>,
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
                        $(      // Individual kernel clocks
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
                            pub fn [< kernel_ $clk _mux >](self, sel: [< $pk ClkSel >]) -> Self {
                                // unsafe: Owned exclusive access to this bitfield
                                interrupt::free(|_| {
                                    let ccip = unsafe {
                                        &(*RCC::ptr()).[< $ccip r >]
                                    };
                                    ccip.modify(|_, w| w.
                                                [< $pk:lower sel >]().variant(sel));
                                });
                                self
                            }

                            #[inline(always)]
                            #[allow(unused)]
                            /// Return the current kernel clock selection
                            pub fn [< get_kernel_ $clk _mux>](&self) ->
                                variant_return_type!([< $pk ClkSel >] $(, $Variant)*)
                            {
                                // unsafe: We only read from this bitfield
                                let ccip = unsafe {
                                    &(*RCC::ptr()).[< $ccip r >]
                                };
                                ccip.read().[< $pk:lower sel >]().variant()
                            }
                        )*
                    }
                    $(      // Group kernel clocks
                        impl [< $pk_g ClkSelGetter >] for $p {}
                    )*

                    $(          // Individual kernel clocks
                        #[doc=$clk_doc]
                        /// kernel clock source selection
                        pub type [< $pk ClkSel >] =
                            rcc::[< $ccip r >]::[< $pk:upper SEL_A >];
                    )*
                    $(          // Group kernel clocks
                        $(
                            #[doc=$clk_doc_g]
                            /// kernel clock source selection
                            pub type [< $pk_g ClkSel >] =
                                rcc::[< $ccip_g r >]::[< $pk_g:upper SEL_A >];

                            /// Can return
                            #[doc=$clk_doc_g]
                            /// kernel clock source selection
                            pub trait [< $pk_g ClkSelGetter >] {
                                #[inline(always)]
                                #[allow(unused)]
                                /// Return the
                                #[doc=$clk_doc_g]
                                /// kernel clock selection
                                fn get_kernel_clk_mux(&self) ->
                                    variant_return_type!([< $pk_g ClkSel >] $(, $Variant_g)*)
                                {
                                    // unsafe: We only read from this bitfield
                                    let ccip = unsafe {
                                        &(*RCC::ptr()).[< $ccip_g r >]
                                    };
                                    ccip.read().[< $pk_g:lower sel >]().variant()
                                }
                            }
                        )*
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

// Enumerate all peripherals and optional clock multiplexers
peripheral_reset_and_enable_control! {
    AHB1, "AMBA High-performance Bus (AHB1) peripherals" => [
        Eth1Mac, Dma2, Dma1,
        #[cfg(any(feature = "dualcore"))] Art,
        Adc12
    ];

    AHB2, "AMBA High-performance Bus (AHB2) peripherals" => [
        Hash, Crypt,
        Rng [kernel clk: Rng d2ccip2 "RNG"],
        Sdmmc2 [group clk: Sdmmc]
    ];

    AHB3, "AMBA High-performance Bus (AHB3) peripherals" => [
        Sdmmc1 [group clk: Sdmmc d1ccip "SDMMC"],
        Qspi [kernel clk: Qspi d1ccip "QUADSPI"],
        Fmc [kernel clk: Fmc d1ccip "FMC"],
        Jpgdec, Dma2d, Mdma
    ];

    AHB4, "AMBA High-performance Bus (AHB4) peripherals" => [
        Hsem, Bdma, Crc, Adc3,
        Gpioa, Gpiob, Gpioc, Gpiod, Gpioe, Gpiof, Gpiog, Gpioh, Gpioi, Gpioj, Gpiok
    ];

    APB1L, "Advanced Peripheral Bus 1L (APB1L) peripherals" => [
        Dac12,
        I2c1 [group clk: I2c123 d2ccip2 "I2C1/2/3"],
        I2c2 [group clk: I2c123],
        I2c3 [group clk: I2c123],

        Cec [kernel clk: Cec(Variant) d2ccip2 "CEC"],
        Lptim1 [kernel clk: Lptim1(Variant) d2ccip2 "LPTIM1"],

        Spi2 [group clk: Spi123],
        Spi3 [group clk: Spi123],

        Tim2, Tim3, Tim4, Tim5, Tim6, Tim7, Tim12, Tim13, Tim14,

        Usart2 [group clk: Usart234578(Variant) d2ccip2 "USART2/3/4/5/7/8"],
        Usart3 [group clk: Usart234578],
        Uart4 [group clk: Usart234578],
        Uart5 [group clk: Usart234578],
        Uart7 [group clk: Usart234578],
        Uart8 [group clk: Usart234578]
    ];

    APB1H, "Advanced Peripheral Bus 1H (APB1H) peripherals" => [
        Fdcan [kernel clk: Fdcan(Variant) d2ccip1 "FDCAN"],
        Swp [kernel clk: Swp d2ccip1 "SWPMI"],
        Crs, Mdios, Opamp
    ];

    APB2, "Advanced Peripheral Bus 2 (APB2) peripherals" => [
        Hrtim,
        Dfsdm1 [kernel clk: Dfsdm1 d2ccip1 "DFSDM1"],

        Sai1 [kernel clk: Sai1(Variant) d2ccip1 "SAI1"],
        Sai2 [group clk: Sai23(Variant) d2ccip1 "SAI2/3"],
        Sai3 [group clk: Sai23],

        Spi1 [group clk: Spi123(Variant) d2ccip1 "SPI1/2/3"],
        Spi4 [group clk: Spi45(Variant) d2ccip1 "SPI4/5"],
        Spi5 [group clk: Spi45],

        Tim1, Tim8, Tim15, Tim16, Tim17,

        Usart1 [group clk: Usart16(Variant) d2ccip2 "USART1/6"],
        Usart6 [group clk: Usart16]
    ];

    APB3, "Advanced Peripheral Bus 3 (APB3) peripherals" => [
        Ltdc,
        #[cfg(any(feature = "dsi"))] Dsi
    ];

    APB4, "Advanced Peripheral Bus 4 (APB4) peripherals" => [
        Vref, Comp12,

        Lptim2 [kernel clk: Lptim2(Variant) d3ccip "LPTIM2"],
        Lptim3 [group clk: Lptim345(Variant) d3ccip "LPTIM3/4/5"],
        Lptim4 [group clk: Lptim345],
        Lptim5 [group clk: Lptim345],
        I2c4 [kernel clk: I2c4 d3ccip "I2C4"],
        Spi6 [kernel clk: Spi6(Variant) d3ccip "SPI6"],
        Sai4 [kernel clk_a: Sai4A(Variant) d3ccip
            "Sub-Block A of SAI4"]
            [kernel clk_b: Sai4B(Variant) d3ccip
            "Sub-Block B of SAI4"]
    ];
}
