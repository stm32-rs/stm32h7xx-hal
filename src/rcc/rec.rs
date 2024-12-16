//! Peripheral Reset and Enable Control (REC)
//!
//! This module contains safe accessors to the RCC functionality for each
//! peripheral.
//!
//! At a minimum each peripheral implements
//! [ResetEnable](trait.ResetEnable.html). Peripherals that have an
//! individual clock multiplexer in the PKSU also have methods
//! `kernel_clk_mux` and `get_kernel_clk_mux`. These set and get the state
//! of the kernel clock multiplexer respectively.
//!
//! Peripherals that share a clock multiplexer in the PKSU with other
//! peripherals implement a trait with a `get_kernel_clk_mux` method that
//! returns the current kernel clock state. Because the kernel_clk_mux is shared
//! between multiple peripherals, it cannot be set by any individual one of
//! them. Instead it can only be set by methods on the
//! [`PeripheralRec`](struct.PeripheralREC.html) itself. These methods are named
//! `kernel_xxxx_clk_mux()`.
//!
//! # Reset/Enable Example
//!
//! ```
//! // Constrain and Freeze power
//! ...
//! let rcc = dp.RCC.constrain();
//! let ccdr = rcc.sys_ck(100.MHz()).freeze(pwrcfg, &dp.SYSCFG);
//!
//! // Enable the clock to a peripheral and reset it
//! ccdr.peripheral.FDCAN.enable().reset();
//! ```
//!
//! # Individual Kernel Clock Example
//! ```
//! let ccdr = ...; // Returned by `freeze()`, see example above
//!
//! // Set individual kernel clock
//! let cec_prec = ccdr.peripheral.CEC.kernel_clk_mux(CecClkSel::LSI);
//!
//! assert_eq!(cec_prec.get_kernel_clk_mux(), CecClkSel::LSI);
//! ```
//!
//! # Group Kernel Clock Example
//! ```
//! let mut ccdr = ...; // Returned by `freeze()`, see example above
//!
//! // Set group kernel clock mux
//! ccdr.peripheral.kernel_i2c123_clk_mux(I2c123ClkSel::PLL3_R);
//!
//! // Enable and reset peripheral
//! let i2c3_prec = ccdr.peripheral.I2C3.enable().reset();
//!
//! assert_eq!(i2c3_prec.get_kernel_clk_mux(), I2c123ClkSel::PLL3_R);
//!
//! // Some method that consumes the i2c3 prec
//! init_i2c3(..., i2c3_prec);
//!
//! // Can't set group kernel clock (it would also affect I2C3)
//! // ccdr.peripheral.kernel_i2c123_clk_mux(I2c123ClkSel::HSI_KER);
//! ```
//!
//! # REC object
//!
//! There is a REC object for each peripheral. For example:
//!
//! ```
//! let rec_object = ccdr.peripheral.FDCAN;
//! ```
//!
//! If REC object is dropped by user code, then the Reset or Enable state of
//! this peripheral cannot be modified for the lifetime of the program.
#![deny(missing_docs)]

use core::marker::PhantomData;

use super::Rcc;
use crate::stm32::{rcc, RCC};
use cortex_m::interrupt;

//const X: stm32h7::stm32h743v::rcc::d1ccipr::FMCSEL = ();

/// A trait for Resetting, Enabling and Disabling a single peripheral
pub trait ResetEnable {
    /// Enable this peripheral
    #[allow(clippy::return_self_not_must_use)]
    fn enable(self) -> Self;
    /// Disable this peripheral
    #[allow(clippy::return_self_not_must_use)]
    fn disable(self) -> Self;
    /// Reset this peripheral
    #[allow(clippy::return_self_not_must_use)]
    fn reset(self) -> Self;
}

/// The clock gating state of a peripheral in low-power mode
///
/// See RM0433 rev 7. Section 8.5.11
#[derive(Default, Copy, Clone, PartialEq, Eq)]
pub enum LowPowerMode {
    /// Kernel and bus interface clocks are not provided in low-power modes.
    Off,
    /// Kernel and bus interface clocks are provided in CSleep mode.
    #[default]
    Enabled,
    /// Kernel and bus interface clocks are provided in both CSleep and CStop
    /// modes. Only applies to peripherals in the D3 / SRD. If the peripheral is
    /// not in the D3 / SRD then this has the same effect as `Enabled`.
    Autonomous,
}

impl Rcc {
    /// Returns all the peripherals resets / enables / kernel clocks.
    ///
    /// # Use case
    ///
    /// Allows peripherals to be reset / enabled before the calling
    /// freeze. For example, the internal watchdog could be enabled to
    /// issue a reset if the call to freeze hangs waiting for an external
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

// This macro uses the paste::item! macro to create identifiers.
//
// https://crates.io/crates/paste
macro_rules! peripheral_reset_and_enable_control {
    ($( #[ $tmeta:meta ] $AXBn:ident, $axb_doc:expr => [
        $(
            $( #[ $pmeta:meta ] )*
                $(($Auto:ident))* $p:ident
                $([ kernel $clk:ident: $pk:ident $(($Variant:ident))* $pk_alias:ident $ccip:ident $clk_doc:expr ])*
                $([ group clk: $pk_g:ident $( $(($Variant_g:ident))* $pk_g_alias:ident $ccip_g:ident $clk_doc_g:expr )* ])*
                $([ fixed clk: $clk_doc_f:expr ])*
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
                        #[ $tmeta ]
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
                                #[ $tmeta ]
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
                    #[ $tmeta ]
                    peripheral_reset_and_enable_control_generator! (
                        $AXBn, $(($Auto))* $p, [< $p:upper >], [< $p:lower >],
                        $( $pmeta )*
                        $(
                            [kernel $clk: $pk $(($Variant))* $pk_alias $ccip $clk_doc]
                        )*
                        $(
                            [group clk: $pk_g [< $pk_g:lower >] $( $(($Variant_g))* $pk_g_alias $ccip_g $clk_doc_g )* ]
                        )*
                        $(
                            [fixed clk: $clk_doc_f]
                        )*
                    );
                )*
            )+
        }
    }
}

// This macro uses the paste::item! macro to create identifiers.
//
// https://crates.io/crates/paste
//
// The macro is intended only to be called from within the
// peripheral_reset_and_enable_control macro
macro_rules! peripheral_reset_and_enable_control_generator {
    (
        $AXBn:ident,
        $(($Auto:ident))* $p:ident,
        $p_upper:ident,         // Lower and upper case $p available for use in
        $p_lower:ident,         // comments, equivalent to with the paste macro.

        $( $pmeta:meta )*
        $([ kernel $clk:ident: $pk:ident $(($Variant:ident))* $pk_alias:ident $ccip:ident $clk_doc:expr ])*
        $([ group clk: $pk_g:ident $pk_g_lower:ident $( $(($Variant_g:ident))* $pk_g_alias:ident $ccip_g:ident $clk_doc_g:expr )* ])*
        $([ fixed clk: $clk_doc_f:expr ])*
    ) => {
        paste::item! {
            #[doc = " Reset, Enable and Clock functionality for " $p]
            ///
            /// # Reset/Enable Example
            ///
            /// ```
            /// let ccdr = ...; // From RCC
            ///
            /// // Enable the clock to the peripheral and reset it
            #[doc = "ccdr.peripheral." $p_upper ".enable().reset();"]
            /// ```
            ///
            $(                  // Individual kernel clocks
                /// # Individual Kernel Clock
                ///
                /// This peripheral has its own dedicated kernel clock.
                #[doc = "See [" $pk "ClkSel](crate::rcc::rec::" $pk "ClkSel) "
                  "for possible clock sources."]
                ///
                /// ```
                /// let ccdr = ...; // From RCC
                ///
                /// // Set individual kernel clock
                #[doc = "let " $p_lower "_prec = ccdr.peripheral." $p_upper
                  ".kernel_clk_mux(" $pk "ClkSel::XX_clock_soruce_XX);"]
                ///
                #[doc = "assert_eq!(" $p_lower "_prec.get_kernel_clk_mux(), "
                  $pk "ClkSel::XX_clock_source_XX);"]
                /// ```
            )*
            $(                  // Group kernel clocks
                /// # Group Kernel Clock
                ///
                /// This peripheral has a kernel clock that is shared with other
                /// peripherals.
                ///
                #[doc = "Since it is shared, it must be set using the "
                  "[kernel_" $pk_g_lower "_clk_mux](crate::rcc::rec::PeripheralREC#method"
                  ".kernel_" $pk_g_lower "_clk_mux) method."]
                ///
                /// ```
                /// let mut ccdr = ...; // From RCC
                ///
                /// // Set group kernel clock mux
                #[doc = " ccdr.peripheral."
                  "kernel_" $pk_g_lower "_clk_mux("
                  $pk_g "ClkSel::XX_clock_source_XX);"]
                ///
                #[doc = " assert_eq!(ccdr.peripheral." $p_upper
                  ".get_kernel_clk_mux(), " $pk_g "ClkSel::XX_clock_source_XX);"]
            )*
            $(                  // Fixed kernel clocks
                /// # Fixed Kernel Clock
                ///
                /// This peripheral has a kernel clock that is always equal to
                #[doc= $clk_doc_f "."]
            )*
            $( #[ $pmeta ] )*
            pub struct $p {
                pub(crate) _marker: PhantomData<*const ()>,
            }
            $( #[ $pmeta ] )*
            impl $p {
                /// Set Low Power Mode for peripheral
                #[allow(clippy::return_self_not_must_use)]
                pub fn low_power(self, lpm: LowPowerMode) -> Self {
                    // unsafe: Owned exclusive access to this bitfield
                    interrupt::free(|_| {
                        // LPEN
                        let lpenr = unsafe {
                            &(*RCC::ptr()).[< $AXBn:lower lpenr >]()
                        };
                        lpenr.modify(|_, w| w.[< $p:lower lpen >]()
                                     .bit(lpm != LowPowerMode::Off));
                        // AMEN
                        $(
                            let amr = unsafe { autonomous!($Auto) };
                            amr.modify(|_, w| w.[< $p:lower amen >]()
                                       .bit(lpm == LowPowerMode::Autonomous));
                        )*
                    });
                    self
                }
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
                            &(*RCC::ptr()).[< $AXBn:lower enr >]()
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
                            &(*RCC::ptr()).[< $AXBn:lower enr >]()
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
                            &(*RCC::ptr()).[< $AXBn:lower rstr >]()
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
                    #[allow(clippy::return_self_not_must_use)]
                    /// Modify the kernel clock for
                    #[doc=$clk_doc "."]
                    /// See RM0433 Rev 7 Section 8.5.8.
                    ///
                    /// It is possible to switch this clock dynamically without
                    /// generating spurs or timing violations. However, the user
                    /// must ensure that both clocks are running. See RM0433 Rev
                    /// 7 Section 8.5.10.
                    pub fn [< kernel_ $clk _mux >](self, sel: [< $pk ClkSel >]) -> Self {
                        // unsafe: Owned exclusive access to this bitfield
                        interrupt::free(|_| {
                            let ccip = unsafe {
                                &(*RCC::ptr()).[< $ccip r >]()
                            };
                            ccip.modify(|_, w| w.
                                        [< $pk:lower sel >]().variant(sel));
                        });
                        self
                    }

                    #[inline(always)]
                    /// Return the current kernel clock selection
                    pub fn [< get_kernel_ $clk _mux>](&self) ->
                        variant_return_type!([< $pk ClkSel >] $(, $Variant)*)
                    {
                        // unsafe: We only read from this bitfield
                        let ccip = unsafe {
                            &(*RCC::ptr()).[< $ccip r >]()
                        };
                        ccip.read().[< $pk:lower sel >]().variant()
                    }
                )*
            }
            $(          // Individual kernel clocks
                #[doc=$clk_doc]
                /// kernel clock source selection
                pub type [< $pk ClkSel >] =
                    rcc::[< $ccip r >]::[< $pk_alias SEL >];
            )*
            $(          // Group kernel clocks
                impl [< $pk_g ClkSelGetter >] for $p {}
            )*
            $(          // Group kernel clocks
                $(
                    #[doc=$clk_doc_g]
                    /// kernel clock source selection.
                    pub type [< $pk_g ClkSel >] =
                        rcc::[< $ccip_g r >]::[< $pk_g_alias SEL >];

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
                                &(*RCC::ptr()).[< $ccip_g r >]()
                            };
                            ccip.read().[< $pk_g:lower sel >]().variant()
                        }
                    }
                )*
            )*
            impl PeripheralREC {
                $(          // Group kernel clocks
                    $(
                        /// Modify the kernel clock for
                        #[doc=$clk_doc_g "."]
                        /// See RM0433 Rev 7 Section 8.5.8.
                        ///
                        /// It is possible to switch this clock dynamically
                        /// without generating spurs or timing
                        /// violations. However, the user must ensure that both
                        /// clocks are running. See RM0433 Rev 7 Section 8.5.10.
                        pub fn [< kernel_ $pk_g:lower _clk_mux >](&mut self, sel: [< $pk_g ClkSel >]) -> &mut Self {
                            // unsafe: Owned exclusive access to this bitfield
                            interrupt::free(|_| {
                                let ccip = unsafe {
                                    &(*RCC::ptr()).[< $ccip_g r >]()
                                };
                                ccip.modify(|_, w| w.
                                            [< $pk_g:lower sel >]().variant(sel));
                            });
                            self
                        }
                    )*
                )*
            }
        }
    }
}

// If the PAC does not fully specify a CCIP field (perhaps because one or
// more values are reserved), then we use a different return type
macro_rules! variant_return_type {
    ($t:ty) => { $t };
    ($t:ty, $Variant: ident) => {
        Option<$t>
    };
}

// Register for autonomous mode enable bits
#[cfg(not(feature = "rm0455"))]
macro_rules! autonomous {
    ($Auto:ident) => {
        &(*RCC::ptr()).d3amr()
    };
}
#[cfg(feature = "rm0455")]
macro_rules! autonomous {
    ($Auto:ident) => {
        &(*RCC::ptr()).srdamr()
    };
}

// Enumerate all peripherals and optional clock multiplexers
//
// Peripherals are grouped by bus for convenience. Each bus is specified like:
// #[attribute] name, "description" => [..];
//
// The attribute is mandatory for the bus grouping, but can just be
// #[cfg(all())]. The description is not used. Each bus grouping can be repeated
// multiple times if needed.
//
// As well as busses, peripherals can optionally be preceeded by a conditional
// compilation attribute. However, this only works for peripherals without
// kernel clock multiplexers.
//
// Peripherals with an individual kernel clock must be marked "kernel clk". If a
// kernel clock multiplexer is shared between multiple peripherals, all those
// peripherals must instead be marked with a common "group clk".
peripheral_reset_and_enable_control! {
    #[cfg(all())]
    AHB1, "AMBA High-performance Bus (AHB1) peripherals" => [
        Dma2, Dma1
    ];
    #[cfg(not(feature = "rm0455"))]
    AHB1, "" => [
        Eth1Mac,
        #[cfg(any(feature = "rm0399"))] Art,
        Adc12 [group clk: Adc(Variant) ADC d3ccip "ADC"],
        Usb1Otg [group clk: Usb USB d2ccip2 "USB"]
    ];
    #[cfg(any(feature = "rm0433", feature = "rm0399"))]
    AHB1, "" => [
        Usb2Otg [group clk: Usb]
    ];
    #[cfg(feature = "rm0455")]
    AHB1, "" => [
        Crc,
        Usb1Otg [group clk: Usb USB cdccip2 "USB"],
        Adc12 [group clk: Adc(Variant) ADC srdccip "ADC"]
    ];


    #[cfg(all())]
    AHB2, "AMBA High-performance Bus (AHB2) peripherals" => [
        Hash, Crypt,
        Sdmmc2 [group clk: Sdmmc]
    ];
    #[cfg(not(feature = "rm0455"))]
    AHB2, "" => [
        Rng [kernel clk: Rng RNG d2ccip2 "RNG"]
    ];
    #[cfg(feature = "rm0455")]
    AHB2, "" => [
        Rng [kernel clk: Rng RNG cdccip2 "RNG"]
    ];
    #[cfg(feature = "rm0468")]
    AHB2, "" => [
        Cordic, Fmac
    ];


    #[cfg(all())]
    AHB3, "AMBA High-performance Bus (AHB3) peripherals" => [
        Dma2d, Mdma
    ];
    #[cfg(not(feature = "rm0455"))]
    AHB3, "" => [
        Sdmmc1 [group clk: Sdmmc SDMMC d1ccip "SDMMC"],
        Fmc [kernel clk: Fmc FMC d1ccip "FMC"]
    ];
    #[cfg(any(feature = "rm0433", feature = "rm0399"))]
    AHB3, "" => [
        Jpgdec,
        Qspi [kernel clk: Qspi FMC d1ccip "QUADSPI"]
    ];
    #[cfg(feature = "rm0455")]
    AHB3, "" => [
        Jpgdec,
        Sdmmc1 [group clk: Sdmmc SDMMC cdccip "SDMMC"],
        Fmc [kernel clk: Fmc FMC cdccip "FMC"],
        Octospi1 [group clk: Octospi FMC cdccip "OCTOSPI"],
        Octospi2 [group clk: Octospi]
    ];
    #[cfg(feature = "rm0468")]
    AHB3, "" => [
        Octospi1 [group clk: Octospi FMC d1ccip "OCTOSPI"],
        Octospi2 [group clk: Octospi]
    ];


    #[cfg(all())]
    AHB4, "AMBA High-performance Bus (AHB4) peripherals" => [
        Gpioa, Gpiob, Gpioc, Gpiod, Gpioe, Gpiof, Gpiog, Gpioh, Gpioi, Gpioj, Gpiok
    ];
    #[cfg(not(feature = "rm0455"))]
    AHB4, "" => [
        (Auto) Crc,
        (Auto) Bdma,
        (Auto) Adc3 [group clk: Adc]
    ];
    #[cfg(feature = "rm0455")]
    AHB4, "" => [
        (Auto) Bdma2
    ];


    #[cfg(all())]
    APB1L, "Advanced Peripheral Bus 1L (APB1L) peripherals" => [
        Spi2 [group clk: Spi123],
        Spi3 [group clk: Spi123],

        Tim2, Tim3, Tim4, Tim5, Tim6, Tim7, Tim12, Tim13, Tim14,

        Usart3 [group clk: Usart234578],
        Uart4 [group clk: Usart234578],
        Uart5 [group clk: Usart234578],
        Uart7 [group clk: Usart234578],
        Uart8 [group clk: Usart234578]
    ];
    #[cfg(not(feature = "rm0455"))]
    APB1L, "" => [
        Dac12,

        Cec [kernel clk: Cec(Variant) CEC d2ccip2 "CEC"],
        Lptim1 [kernel clk: Lptim1(Variant) LPTIM1 d2ccip2 "LPTIM1"],
        Usart2 [group clk: Usart234578(Variant) USART234578 d2ccip2 "USART2/3/4/5/7/8"]
    ];
    #[cfg(any(feature = "rm0433", feature = "rm0399"))]
    APB1L, "" => [
        I2c1 [group clk: I2c123 I2C123 d2ccip2 "I2C1/2/3"],
        I2c2 [group clk: I2c123],
        I2c3 [group clk: I2c123]
    ];
    #[cfg(feature = "rm0455")]
    APB1L, "" => [
        Dac1,

        I2c1 [group clk: I2c123 I2C123 cdccip2 "I2C1/2/3"],
        I2c2 [group clk: I2c123],
        I2c3 [group clk: I2c123],
        Cec [kernel clk: Cec(Variant) CEC cdccip2 "CEC"],
        Lptim1 [kernel clk: Lptim1(Variant) LPTIM1 cdccip2 "LPTIM1"],
        Usart2 [group clk: Usart234578(Variant) USART234578 cdccip2 "USART2/3/4/5/7/8"]
    ];
    #[cfg(feature = "rm0468")]
    APB1L, "" => [
        I2c1 [group clk: I2c1235 I2C1235 d2ccip2 "I2C1/2/3/5"],
        I2c2 [group clk: I2c1235],
        I2c3 [group clk: I2c1235],
        I2c5 [group clk: I2c1235]
    ];


    #[cfg(all())]
    APB1H, "Advanced Peripheral Bus 1H (APB1H) peripherals" => [
        Crs, Mdios, Opamp
    ];
    #[cfg(not(feature = "rm0455"))]
    APB1H, "" => [
        Fdcan [kernel clk: Fdcan(Variant) FDCAN d2ccip1 "FDCAN"]
    ];
    #[cfg(any(feature = "rm0433", feature = "rm0399"))]
    APB1H, "" => [
        Swp [kernel clk: Swp SWP d2ccip1 "SWPMI"]
    ];
    #[cfg(feature = "rm0455")]
    APB1H, "" => [
        Fdcan [kernel clk: Fdcan(Variant) FDCAN cdccip1 "FDCAN"],
        Swpmi [kernel clk: Swpmi SWPMI cdccip1 "SWPMI"]
    ];
    #[cfg(feature = "rm0468")]
    APB1H, "" => [
        Swpmi [kernel clk: Swpmi SWPMI d2ccip1 "SWPMI"],

        Tim23, Tim24
    ];


    #[cfg(all())]
    APB2, "Advanced Peripheral Bus 2 (APB2) peripherals" => [
        Tim1, Tim8, Tim15, Tim16, Tim17
    ];
    #[cfg(not(feature = "rm0455"))]
    APB2, "" => [
        Dfsdm1 [kernel clk: Dfsdm1 DFSDM1 d2ccip1 "DFSDM1"],

        Sai1 [kernel clk: Sai1(Variant) SAI1 d2ccip1 "SAI1"],

        Spi1 [group clk: Spi123(Variant) SAI1 d2ccip1 "SPI1/2/3"],
        Spi4 [group clk: Spi45(Variant) SPI45 d2ccip1 "SPI4/5"],
        Spi5 [group clk: Spi45]
    ];
    #[cfg(any(feature = "rm0433", feature = "rm0399"))]
    APB2, "" => [
        Sai2 [group clk: Sai23(Variant) SAI1 d2ccip1 "SAI2/3"],
        Sai3 [group clk: Sai23],

        Usart1 [group clk: Usart16(Variant) USART16 d2ccip2 "USART1/6"],
        Usart6 [group clk: Usart16],
        Hrtim [kernel clk: Hrtim HRTIM cfg "HRTIM"]
    ];
    #[cfg(feature = "rm0455")]
    APB2, "" => [
        Dfsdm1 [kernel clk: Dfsdm1 DFSDM1 cdccip1 "DFSDM1"],

        Sai1 [kernel clk: Sai1(Variant) SAI1 cdccip1 "SAI1"],
        Sai2 [kernel clk_a: Sai2A(Variant) SAI2A cdccip1
            "Sub-Block A of SAI2"]
            [kernel clk_b: Sai2B(Variant) SAI2A cdccip1
            "Sub-Block B of SAI2"],

        Spi1 [group clk: Spi123(Variant) SAI1 cdccip1 "SPI1/2/3"],
        Spi4 [group clk: Spi45(Variant) SPI45 cdccip1 "SPI4/5"],
        Spi5 [group clk: Spi45],

        Usart1 [group clk: Usart16910(Variant) USART16910 cdccip2 "USART1/6/9/10"],
        Usart6 [group clk: Usart16910],
        Uart9 [group clk: Usart16910],
        Usart10 [group clk: Usart16910]
    ];
    #[cfg(feature = "rm0468")]
    APB2, "" => [
        Usart1 [group clk: Usart16910(Variant) USART16910 d2ccip2 "USART1/6/9/10"],
        Usart6 [group clk: Usart16910],
        Uart9 [group clk: Usart16910],
        Usart10 [group clk: Usart16910]
    ];

    #[cfg(all())]
    APB3, "Advanced Peripheral Bus 3 (APB3) peripherals" => [
        Ltdc [fixed clk: "pll3_r_ck"],
        #[cfg(any(feature = "rm0399"))] Dsi
    ];

    #[cfg(all())]
    APB4, "Advanced Peripheral Bus 4 (APB4) peripherals" => [
        (Auto) Vref,
        (Auto) Comp12
    ];
    #[cfg(not(feature = "rm0455"))]
    APB4, "" => [
        (Auto) Lptim2 [kernel clk: Lptim2(Variant) LPTIM2 d3ccip "LPTIM2"],
        (Auto) Lptim3 [group clk: Lptim345(Variant) LPTIM2 d3ccip "LPTIM3/4/5"],
        (Auto) Lptim4 [group clk: Lptim345],
        (Auto) Lptim5 [group clk: Lptim345],

        (Auto) I2c4 [kernel clk: I2c4 I2C4 d3ccip "I2C4"],
        (Auto) Spi6 [kernel clk: Spi6(Variant) SPI6 d3ccip "SPI6"],
        (Auto) Sai4 [kernel clk_a: Sai4A(Variant) SAI4A d3ccip
            "Sub-Block A of SAI4"]
            [kernel clk_b: Sai4B(Variant) SAI4A d3ccip
            "Sub-Block B of SAI4"]
    ];
    #[cfg(feature = "rm0455")]
    APB4, "" => [
        (Auto) Dac2,

        (Auto) Lptim2 [kernel clk: Lptim2(Variant) LPTIM2 srdccip "LPTIM2"],
        (Auto) Lptim3,// TODO [group clk: Lptim3(Variant) srdccip "LPTIM3"],

        (Auto) I2c4 [kernel clk: I2c4 I2C4 srdccip "I2C4"],
        (Auto) Spi6 [kernel clk: Spi6(Variant) SPI6 srdccip "SPI6"]
    ];
}
