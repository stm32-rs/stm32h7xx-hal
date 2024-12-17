//! Reset and Clock Control
//!
//! This module configures the RCC unit to provide set frequencies for
//! the input to the SCGU `sys_ck`, the AMBA High-performance Busses
//! and Advanced eXtensible Interface Bus `hclk`, the AMBA Peripheral
//! Busses `pclkN` and the peripheral clock `per_ck`.
//!
//! See Figure 49 "Core and bus clock generation" in Reference Manual
//! RM0433 Rev 7 for more information (p 348).
//!
//! HSI is 64 MHz.
//! CSI is 4 MHz.
//! HSI48 is 48MHz.
//!
//! # Usage
//!
//! This peripheral must be used alongside the
//! [`PWR`](../pwr/index.html) peripheral to freeze voltage scaling of the
//! device.
//!
//! A builder pattern is used to specify the state and frequency of
//! possible clocks. The `freeze` method configures the RCC peripheral
//! in a best-effort attempt to generate these clocks. The actual
//! clocks configured are returned in `ccdr.clocks`.
//!
//! No clock specification overrides another. However supplying some
//! clock specifications may influence multiple resulting clocks,
//! including those corresponding to other clock specifications. This
//! is particularly the case for PLL clocks, where the frequencies of
//! adjacent 'P', 'Q, and 'R' clock outputs must have a simple integer
//! fraction relationship.
//!
//! Some clock specifications imply other clock specifications, as follows:
//!
//! * `use_hse(a)` implies `sys_ck(a)`
//!
//! * `sys_ck(b)` implies `pll1_p_ck(b)` unless `b` equals HSI or
//!   `use_hse(b)` was specified
//!
//! * `pll1_p_ck(c)` implies `pll1_r_ck(c/2)`, including when
//!   `pll1_p_ck` was implied by `sys_ck(c)` or `mco2_from_pll1_p_ck(c)`.
//!
//! Implied clock specifications can always be overridden by explicitly
//! specifying that clock. If this results in a configuration that cannot
//! be achieved by hardware, `freeze` will panic.
//!
//! # Examples
//!
//! - [Simple RCC example](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/rcc.rs).
//! - [Fractional PLL configuration](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/fractional-pll.rs)
//! - [MCO example](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/mco.rs)
//!
//! Simple example:
//!
//! ```rust
//!     let dp = pac::Peripherals::take().unwrap();
//!
//!     let pwr = dp.PWR.constrain();
//!     let pwrcfg = pwr.freeze();
//!
//!     let rcc = dp.RCC.constrain();
//!     let ccdr = rcc
//!         .sys_ck(96.MHz())
//!         .pclk1(48.MHz())
//!         .freeze(pwrcfg, &dp.SYSCFG);
//! ```
//!
//! A more complex example, involving the PLL:
//!
//! ```rust
//!     let dp = pac::Peripherals::take().unwrap();
//!
//!     let pwr = dp.PWR.constrain();
//!     let pwrcfg = pwr.freeze();
//!
//!     let rcc = dp.RCC.constrain();
//!     let ccdr = rcc
//!         .sys_ck(200.MHz()) // Implies pll1_p_ck
//!         // For non-integer values, round up. `freeze` will never
//!         // configure a clock faster than that specified.
//!         .pll1_q_ck(33_333_334.hz())
//!         .freeze(pwrcfg, &dp.SYSCFG);
//! ```
//!
//! A much more complex example, indicative of real usage with a
//! significant fraction of the STM32H7's capabilities.
//!
//! ```rust
//!     let dp = pac::Peripherals::take().unwrap();
//!
//!     let pwr = dp.PWR.constrain();
//!     let pwrcfg = pwr.freeze();
//!
//!     let rcc = dp.RCC.constrain();
//!     let ccdr = rcc
//!         .use_hse(25.MHz()) // XTAL X1
//!         .sys_ck(400.MHz())
//!         .pll1_r_ck(100.MHz()) // for TRACECK
//!         .pll1_q_ck(200.MHz())
//!         .hclk(200.MHz())
//!         .pll3_strategy(PllConfigStrategy::Iterative)
//!         .pll3_p_ck(240.MHz()) // for LTDC
//!         .pll3_q_ck(48.MHz()) // for LTDC
//!         .pll3_r_ck(26_666_667.Hz()) // Pixel clock for LTDC
//!         .freeze(pwrcfg, &dp.SYSCFG);
//!```
//!
//! # Peripherals
//!
//! The `freeze()` method returns a [Core Clocks Distribution and Reset
//! (CCDR)](Ccdr) object. This singleton tells you how the core clocks were
//! actually configured (in [CoreClocks]) and allows you to configure the
//! remaining peripherals (see [PeripheralREC]).
//!
//!```rust
//! let ccdr = ...; // Returned by `freeze()`, see examples above
//!
//! // Runtime confirmation that hclk really is 200MHz
//! assert_eq!(ccdr.clocks.hclk().raw(), 200_000_000);
//!
//! // Panics if pll1_q_ck is not running
//! let _ = ccdr.clocks.pll1_q_ck().unwrap();
//!
//! // Enable the clock to a peripheral and reset it
//! ccdr.peripheral.FDCAN.enable().reset();
//!```
//!
//! The [PeripheralREC] members implement move
//! semantics, so once you have passed them to a constructor they cannot be
//! modified again in safe Rust.
//!
//!```rust
//! // Constructor for custom FDCAN driver
//! my_fdcan(dp.FDCAN,
//!          &ccdr.clocks,         // Immutable reference to core clock state
//!          ccdr.peripheral.FDCAN // Ownership of reset + enable control
//! );
//!
//! // Compile error, value was moved ^^
//! ccdr.peripheral.FDCAN.disable();
//!```
//!
#![deny(missing_docs)]

use crate::pwr::PowerConfiguration;
use crate::pwr::VoltageScale as Voltage;
use crate::stm32::rcc::cfgr::SW;
use crate::stm32::rcc::cfgr::TIMPRE;
use crate::stm32::rcc::pllckselr::PLLSRC;
use crate::stm32::{RCC, SYSCFG};
use crate::time::Hertz;

#[cfg(feature = "rm0455")]
use crate::stm32::rcc::cdcfgr1::HPRE;
#[cfg(not(feature = "rm0455"))]
use crate::stm32::rcc::d1cfgr::HPRE;
#[cfg(feature = "log")]
use log::debug;

#[cfg(feature = "rm0455")]
use crate::stm32::rcc::cdccipr::CKPERSEL;
#[cfg(not(feature = "rm0455"))]
use crate::stm32::rcc::d1ccipr::CKPERSEL;

pub mod backup;
mod core_clocks;
mod pll;
pub mod rec;
mod reset_reason;

pub use core_clocks::CoreClocks;
pub use pll::{PllConfig, PllConfigStrategy};
pub use rec::{LowPowerMode, PeripheralREC, ResetEnable};
pub use reset_reason::ResetReason;

mod mco;
use mco::{MCO1Config, MCO2Config, MCO1, MCO2};

/// Configuration of the core clocks
pub struct Config {
    hse: Option<u32>,
    bypass_hse: bool,
    sys_ck: Option<u32>,
    per_ck: Option<u32>,
    rcc_hclk: Option<u32>,
    rcc_pclk1: Option<u32>,
    rcc_pclk2: Option<u32>,
    rcc_pclk3: Option<u32>,
    rcc_pclk4: Option<u32>,
    mco1: MCO1Config,
    mco2: MCO2Config,
    pll1: PllConfig,
    pll2: PllConfig,
    pll3: PllConfig,
}

/// Extension trait that constrains the `RCC` peripheral
pub trait RccExt {
    /// Constrains the `RCC` peripheral so it plays nicely with the
    /// other abstractions
    fn constrain(self) -> Rcc;
}

impl RccExt for RCC {
    fn constrain(self) -> Rcc {
        Rcc {
            config: Config {
                hse: None,
                bypass_hse: false,
                sys_ck: None,
                per_ck: None,
                rcc_hclk: None,
                rcc_pclk1: None,
                rcc_pclk2: None,
                rcc_pclk3: None,
                rcc_pclk4: None,
                mco1: MCO1Config::default(),
                mco2: MCO2Config::default(),
                pll1: PllConfig::default(),
                pll2: PllConfig::default(),
                pll3: PllConfig::default(),
            },
            rb: self,
        }
    }
}

/// Constrained RCC peripheral
///
/// Generated by calling `constrain` on the PAC's RCC peripheral.
///
/// ```rust
/// let dp = stm32::Peripherals::take().unwrap();
/// let rcc = dp.RCC.constrain();
/// ```
pub struct Rcc {
    config: Config,
    pub(crate) rb: RCC,
}

impl Rcc {
    /// Gets and clears the reason of why the mcu was reset
    pub fn get_reset_reason(&mut self) -> ResetReason {
        reset_reason::get_reset_reason(&mut self.rb)
    }
}

/// Core Clock Distribution and Reset (CCDR)
///
/// Generated when the RCC is frozen. The configuration of the Sys_Ck
/// `sys_ck`, CPU Clock `c_ck`, AXI peripheral clock `aclk`, AHB
/// clocks `hclk`, APB clocks `pclkN` and PLL outputs `pllN_X_ck` are
/// frozen. However the distribution of some clocks may still be
/// modified and peripherals enabled / reset by passing this object
/// to other implementations in this stack.
pub struct Ccdr {
    /// A record of the frozen core clock frequencies
    pub clocks: CoreClocks,

    /// Peripheral reset / enable / kernel clock control
    pub peripheral: PeripheralREC,

    // Yes, it lives (locally)! We retain the right to switch most
    // PKSUs on the fly, to fine-tune PLL frequencies, and to enable /
    // reset peripherals.
    //
    // TODO: Remove this once all permitted RCC register accesses
    // after freeze are enumerated in this struct
    pub(crate) rb: RCC,
}

const HSI: u32 = 64_000_000; // Hz
const CSI: u32 = 4_000_000; // Hz
const HSI48: u32 = 48_000_000; // Hz
const LSI: u32 = 32_000; // Hz

/// Setter defintion for pclk 1 - 4
macro_rules! pclk_setter {
    ($($name:ident: $pclk:ident,)+) => {
        $(
            /// Set the peripheral clock frequency for APB
            /// peripherals.
            #[must_use]
            pub fn $name(mut self, freq: Hertz) -> Self {
                self.config.$pclk = Some(freq.raw());
                self
            }
        )+
    };
}

/// Setter definition for pll 1 - 3 p, q, r
macro_rules! pll_setter {
    ($($pll:ident: [ $($name:ident: $ck:ident,)+ ],)+) => {
        $(
            $(
                /// Set the target clock frequency for PLL output
                #[must_use]
                pub fn $name(mut self, freq: Hertz) -> Self {
                    self.config.$pll.$ck = Some(freq.raw());
                    self
                }
            )+
        )+
    };
}

/// Setter definition for pll 1 - 3 strategy
macro_rules! pll_strategy_setter {
    ($($pll:ident: $name:ident,)+) => {
        $(
            /// Set the PLL divider strategy to be used when the PLL
            /// is configured
            #[must_use]
            pub fn $name(mut self, strategy: PllConfigStrategy) -> Self
            {
                self.config.$pll.strategy = strategy;
                self
            }
        )+
    }
}

impl Rcc {
    /// Uses HSE (external oscillator) instead of HSI (internal RC
    /// oscillator) as the clock source. Will result in a hang if an
    /// external oscillator is not connected or it fails to start.
    #[must_use]
    pub fn use_hse(mut self, freq: Hertz) -> Self {
        self.config.hse = Some(freq.raw());
        self
    }

    /// Use an external clock signal rather than a crystal oscillator,
    /// bypassing the XTAL driver.
    #[must_use]
    pub fn bypass_hse(mut self) -> Self {
        self.config.bypass_hse = true;
        self
    }

    /// Set input frequency to the SCGU
    #[must_use]
    pub fn sys_ck(mut self, freq: Hertz) -> Self {
        self.config.sys_ck = Some(freq.raw());
        self
    }

    /// Set input frequency to the SCGU - ALIAS
    #[must_use]
    pub fn sysclk(mut self, freq: Hertz) -> Self {
        self.config.sys_ck = Some(freq.raw());
        self
    }

    /// Set peripheral clock frequency
    #[must_use]
    pub fn per_ck(mut self, freq: Hertz) -> Self {
        self.config.per_ck = Some(freq.raw());
        self
    }

    /// Set the peripheral clock frequency for AHB and AXI peripherals. There
    /// are several gated versions `rcc_hclk[1-4]` for different power domains,
    /// and the AXI bus clock is called `rcc_aclk`. However they are all the
    /// same frequency.
    #[must_use]
    pub fn hclk(mut self, freq: Hertz) -> Self {
        self.config.rcc_hclk = Some(freq.raw());
        self
    }

    pclk_setter! {
        pclk1: rcc_pclk1,
        pclk2: rcc_pclk2,
        pclk3: rcc_pclk3,
        pclk4: rcc_pclk4,
    }

    pll_setter! {
        pll1: [
            pll1_p_ck: p_ck,
            pll1_q_ck: q_ck,
            pll1_r_ck: r_ck,
        ],
        pll2: [
            pll2_p_ck: p_ck,
            pll2_q_ck: q_ck,
            pll2_r_ck: r_ck,
        ],
        pll3: [
            pll3_p_ck: p_ck,
            pll3_q_ck: q_ck,
            pll3_r_ck: r_ck,
        ],
    }

    pll_strategy_setter! {
        pll1: pll1_strategy,
        pll2: pll2_strategy,
        pll3: pll3_strategy,
    }
}

/// Divider calculator for pclk 1 - 4
///
/// Also calulate tim[xy]_ker_clk if there are timers on this bus
macro_rules! ppre_calculate {
    ($(($ppre:ident, $bits:ident): ($self: ident, $hclk: ident,
                                    $pclk: ident, $max: ident
                                    $(,$rcc_tim_ker_clk:ident, $timpre:ident)*),)+) => {
        $(
            // Get intended rcc_pclkN frequency
            let $pclk: u32 = $self.config
                .$pclk
                .unwrap_or_else(|| core::cmp::min($max, $hclk / 2));

            // Calculate suitable divider
            let ($bits, $ppre) = match $hclk.div_ceil($pclk)
            {
                0 => unreachable!(),
                1 => (0b000, 1 as u8),
                2 => (0b100, 2),
                3..=5 => (0b101, 4),
                6..=11 => (0b110, 8),
                _ => (0b111, 16),
            };

            // Calculate real APBn clock
            let $pclk = $hclk / u32::from($ppre);

            // Check in range
            assert!($pclk <= $max);

            $(
                let $rcc_tim_ker_clk = match ($bits, &$timpre)
                {
                    (0b101, TIMPRE::DefaultX2) => $hclk / 2,
                    (0b110, TIMPRE::DefaultX4) => $hclk / 2,
                    (0b110, TIMPRE::DefaultX2) => $hclk / 4,
                    (0b111, TIMPRE::DefaultX4) => $hclk / 4,
                    (0b111, TIMPRE::DefaultX2) => $hclk / 8,
                    _ => $hclk,
                };
            )*
        )+
    };
}

impl Rcc {
    fn flash_setup(rcc_aclk: u32, vos: Voltage) {
        use crate::stm32::FLASH;
        // ACLK in MHz, round down and subtract 1 from integers. eg.
        // 61_999_999 -> 61MHz
        // 62_000_000 -> 61MHz
        // 62_000_001 -> 62MHz
        let rcc_aclk_mhz = (rcc_aclk - 1) / 1_000_000;

        // See RM0433 Rev 7 Table 17. FLASH recommended number of wait
        // states and programming delay
        #[cfg(any(feature = "rm0433", feature = "rm0399"))]
        let (wait_states, progr_delay) = match vos {
            // VOS 0 range VCORE 1.26V - 1.40V
            Voltage::Scale0 => match rcc_aclk_mhz {
                0..=69 => (0, 0),
                70..=139 => (1, 1),
                140..=184 => (2, 1),
                185..=209 => (2, 2),
                210..=224 => (3, 2),
                225..=239 => (4, 2),
                _ => (7, 3),
            },
            // VOS 1 range VCORE 1.15V - 1.26V
            Voltage::Scale1 => match rcc_aclk_mhz {
                0..=69 => (0, 0),
                70..=139 => (1, 1),
                140..=184 => (2, 1),
                185..=209 => (2, 2),
                210..=224 => (3, 2),
                _ => (7, 3),
            },
            // VOS 2 range VCORE 1.05V - 1.15V
            Voltage::Scale2 => match rcc_aclk_mhz {
                0..=54 => (0, 0),
                55..=109 => (1, 1),
                110..=164 => (2, 1),
                165..=224 => (3, 2),
                _ => (7, 3),
            },
            // VOS 3 range VCORE 0.95V - 1.05V
            Voltage::Scale3 => match rcc_aclk_mhz {
                0..=44 => (0, 0),
                45..=89 => (1, 1),
                90..=134 => (2, 1),
                135..=179 => (3, 2),
                180..=224 => (4, 2),
                _ => (7, 3),
            },
        };

        // See RM0455 Rev 3 Table 15
        #[cfg(feature = "rm0455")]
        let (wait_states, progr_delay) = match vos {
            // VOS 0 range VCORE 1.25V - 1.35V
            Voltage::Scale0 => match rcc_aclk_mhz {
                0..=41 => (1, 0), // Errata? 7A3 Rev Z did not run at 0 wait-states
                42..=83 => (1, 0),
                84..=125 => (2, 1),
                126..=167 => (3, 1),
                168..=209 => (4, 2),
                210..=251 => (5, 2),
                252..=280 => (6, 3),
                _ => (7, 3),
            },
            // VOS 1 range VCORE 1.15V - 1.25V
            Voltage::Scale1 => match rcc_aclk_mhz {
                0..=37 => (1, 0), // Errata? 7A3 Rev Z did not run at 0 wait-states
                38..=75 => (1, 0),
                76..=113 => (2, 1),
                114..=151 => (3, 1),
                152..=189 => (4, 2),
                190..=224 => (5, 2),
                _ => (7, 3),
            },
            // VOS 2 range VCORE 1.05V - 1.15V
            Voltage::Scale2 => match rcc_aclk_mhz {
                0..=33 => (1, 0), // Errata? 7A3 Rev Z did not run at 0 wait-states
                34..=67 => (1, 0),
                68..=101 => (2, 1),
                102..=135 => (3, 1),
                136..=159 => (4, 2),
                _ => (7, 3),
            },
            // VOS 3 range VCORE 0.95V - 1.05V
            Voltage::Scale3 => match rcc_aclk_mhz {
                0..=21 => (1, 0), // Errata? 7A3 Rev Z did not run at 0 wait-states
                22..=43 => (1, 0),
                44..=65 => (2, 1),
                66..=87 => (3, 1),
                _ => (7, 3),
            },
        };

        // See RM0468 Rev 2 Table 16
        #[cfg(feature = "rm0468")]
        let (wait_states, progr_delay) = match vos {
            // VOS 0 range VCORE 1.25V - 1.35V
            Voltage::Scale0 => match rcc_aclk_mhz {
                0..=69 => (0, 0),
                70..=139 => (1, 1),
                140..=209 => (2, 2),
                _ => (3, 3),
            },
            // VOS 1 range VCORE 1.15V - 1.25V
            Voltage::Scale1 => match rcc_aclk_mhz {
                0..=66 => (0, 0),
                67..=132 => (1, 1),
                133..=199 => (2, 2),
                _ => (3, 3),
            },
            // VOS 2 range VCORE 1.05V - 1.15V
            Voltage::Scale2 => match rcc_aclk_mhz {
                0..=49 => (0, 0),
                50..=99 => (1, 1),
                100..=149 => (2, 2),
                _ => (3, 3),
            },
            // VOS 3 range VCORE 0.95V - 1.05V
            Voltage::Scale3 => match rcc_aclk_mhz {
                0..=34 => (0, 0),
                35..=69 => (1, 1),
                70..=84 => (2, 2),
                _ => (3, 3),
            },
        };

        let flash = unsafe { &(*FLASH::ptr()) };
        // Adjust flash wait states
        flash.acr().write(|w| unsafe {
            w.wrhighfreq().bits(progr_delay).latency().bits(wait_states)
        });
        while flash.acr().read().latency().bits() != wait_states {}
    }

    /// Setup sys_ck
    /// Returns sys_ck frequency, and a pll1_p_ck
    fn sys_ck_setup(&mut self) -> (Hertz, bool) {
        // Compare available with wanted clocks
        let srcclk = self.config.hse.unwrap_or(HSI); // Available clocks
        let sys_ck = self.config.sys_ck.unwrap_or(srcclk);

        if sys_ck != srcclk {
            // The requested system clock is not the immediately available
            // HSE/HSI clock. Perhaps there are other ways of obtaining
            // the requested system clock (such as `HSIDIV`) but we will
            // ignore those for now.
            //
            // Therefore we must use pll1_p_ck
            let pll1_p_ck = match self.config.pll1.p_ck {
                Some(p_ck) => {
                    assert!(p_ck == sys_ck,
                            "Error: Cannot set pll1_p_ck independently as it must be used to generate sys_ck");
                    Some(p_ck)
                }
                None => Some(sys_ck),
            };
            self.config.pll1.p_ck = pll1_p_ck;

            (Hertz::from_raw(sys_ck), true)
        } else {
            // sys_ck is derived directly from a source clock
            // (HSE/HSI). pll1_p_ck can be as requested
            (Hertz::from_raw(sys_ck), false)
        }
    }

    /// Setup traceclk
    /// Returns a pll1_r_ck
    fn traceclk_setup(&mut self, sys_use_pll1_p: bool) {
        let pll1_r_ck = match (sys_use_pll1_p, self.config.pll1.r_ck) {
            // pll1_p_ck selected as system clock but pll1_r_ck not
            // set. The traceclk mux is synchronous with the system
            // clock mux, but has pll1_r_ck as an input. In order to
            // keep traceclk running, we force a pll1_r_ck.
            #[cfg(not(feature = "rm0455"))]
            (true, None) => Some(self.config.pll1.p_ck.unwrap() / 2),
            #[cfg(feature = "rm0455")]
            (true, None) => Some(self.config.pll1.p_ck.unwrap() / 8),
            // Either pll1 not selected as system clock, free choice
            // of pll1_r_ck. Or pll1 is selected, assume user has set
            // a suitable pll1_r_ck frequency.
            _ => self.config.pll1.r_ck,
        };
        self.config.pll1.r_ck = pll1_r_ck;
    }

    /// Freeze the core clocks, returning a Core Clocks Distribution
    /// and Reset (CCDR) structure. The actual frequency of the clocks
    /// configured is returned in the `clocks` member of the CCDR
    /// structure.
    ///
    /// Note that `freeze` will never result in a clock _faster_ than
    /// that specified. It may result in a clock that is a factor of [1,
    /// 2) slower.
    ///
    /// `syscfg` is required to enable the I/O compensation cell.
    ///
    /// # Panics
    ///
    /// If a clock specification cannot be achieved within the
    /// hardware specification then this function will panic. This
    /// function may also panic if a clock specification can be
    /// achieved, but the mechanism for doing so is not yet
    /// implemented here.
    pub fn freeze(
        mut self,
        pwrcfg: PowerConfiguration,
        syscfg: &SYSCFG,
    ) -> Ccdr {
        // We do not reset RCC here. This routine must assert when
        // the previous state of the RCC peripheral is unacceptable.

        // config modifications ----------------------------------------
        // (required for self-consistency and usability)

        // if needed for mco, set sys_ck / pll1_p / pll1_q / pll2_p
        self.mco1_setup();
        self.mco2_setup();

        // sys_ck from PLL if needed, else HSE or HSI
        let (sys_ck, sys_use_pll1_p) = self.sys_ck_setup();

        // Configure traceclk from PLL if needed
        self.traceclk_setup(sys_use_pll1_p);

        // self is now immutable ----------------------------------------
        let rcc = &self.rb;

        // Configure PLL1
        let (pll1_p_ck, pll1_q_ck, pll1_r_ck) =
            self.pll1_setup(rcc, &self.config.pll1);
        // Configure PLL2
        let (pll2_p_ck, pll2_q_ck, pll2_r_ck) =
            self.pll2_setup(rcc, &self.config.pll2);
        // Configure PLL3
        let (pll3_p_ck, pll3_q_ck, pll3_r_ck) =
            self.pll3_setup(rcc, &self.config.pll3);

        let sys_ck = if sys_use_pll1_p {
            pll1_p_ck.unwrap() // Must have been set by sys_ck_setup
        } else {
            sys_ck
        };

        // hsi_ck = HSI. This routine does not support HSIDIV != 1. To
        // do so it would need to ensure all PLLxON bits are clear
        // before changing the value of HSIDIV
        let hsi = HSI;
        assert!(
            rcc.cr().read().hsion().is_on(),
            "HSI oscillator must be on!"
        );
        assert!(rcc.cr().read().hsidiv().is_div1());

        let csi = CSI;
        let hsi48 = HSI48;

        // Enable LSI for RTC, IWDG, AWU, or MCO2
        let lsi = LSI;
        rcc.csr().modify(|_, w| w.lsion().on());
        while rcc.csr().read().lsirdy().is_not_ready() {}

        // per_ck from HSI by default
        let (per_ck, ckpersel) =
            match (self.config.per_ck == self.config.hse, self.config.per_ck) {
                (true, Some(hse)) => (hse, CKPERSEL::Hse), // HSE
                (_, Some(CSI)) => (csi, CKPERSEL::Csi),    // CSI
                _ => (hsi, CKPERSEL::Hsi),                 // HSI
            };

        // D1 Core Prescaler
        // Set to 1
        let d1cpre_bits = 0;
        let d1cpre_div = 1;
        let sys_d1cpre_ck = sys_ck.raw() / d1cpre_div;

        // Timer prescaler selection
        let timpre = TIMPRE::DefaultX2;

        // Refer to part datasheet "General operating conditions"
        // table for (rev V). We do not assert checks for earlier
        // revisions which may have lower limits.
        #[cfg(any(feature = "rm0433", feature = "rm0399"))]
        let (sys_d1cpre_ck_max, rcc_hclk_max, pclk_max) = match pwrcfg.vos {
            Voltage::Scale0 => (480_000_000, 240_000_000, 120_000_000),
            Voltage::Scale1 => (400_000_000, 200_000_000, 100_000_000),
            Voltage::Scale2 => (300_000_000, 150_000_000, 75_000_000),
            _ => (200_000_000, 100_000_000, 50_000_000),
        };

        #[cfg(feature = "rm0455")] // 7B3 / 7A3 / 7B0
        let (sys_d1cpre_ck_max, rcc_hclk_max, pclk_max) = match pwrcfg.vos {
            Voltage::Scale0 => (280_000_000, 280_000_000, 140_000_000),
            Voltage::Scale1 => (225_000_000, 225_000_000, 112_500_000),
            Voltage::Scale2 => (160_000_000, 160_000_000, 80_000_000),
            _ => (88_000_000, 88_000_000, 44_000_000),
        };

        #[cfg(feature = "rm0468")] // 725 / 735 / 730
        let (sys_d1cpre_ck_max, rcc_hclk_max, pclk_max) = match pwrcfg.vos {
            Voltage::Scale0 => (520_000_000, 275_000_000, 137_500_000),
            Voltage::Scale1 => (400_000_000, 200_000_000, 100_000_000),
            Voltage::Scale2 => (300_000_000, 150_000_000, 75_000_000),
            _ => (170_000_000, 85_000_000, 42_500_000),
        };

        // Check resulting sys_d1cpre_ck
        assert!(sys_d1cpre_ck <= sys_d1cpre_ck_max);

        // Get AHB clock or sensible default
        #[cfg(not(feature = "rm0455"))]
        let rcc_hclk = self.config.rcc_hclk.unwrap_or(sys_d1cpre_ck / 2);
        #[cfg(feature = "rm0455")]
        let rcc_hclk = self.config.rcc_hclk.unwrap_or(sys_d1cpre_ck);

        assert!(rcc_hclk <= rcc_hclk_max);

        // Estimate divisor
        let (hpre_bits, hpre_div) = match sys_d1cpre_ck.div_ceil(rcc_hclk) {
            0 => unreachable!(),
            1 => (HPRE::Div1, 1),
            2 => (HPRE::Div2, 2),
            3..=5 => (HPRE::Div4, 4),
            6..=11 => (HPRE::Div8, 8),
            12..=39 => (HPRE::Div16, 16),
            40..=95 => (HPRE::Div64, 64),
            96..=191 => (HPRE::Div128, 128),
            192..=383 => (HPRE::Div256, 256),
            _ => (HPRE::Div512, 512),
        };

        // Calculate real AXI and AHB clock
        let rcc_hclk = sys_d1cpre_ck / hpre_div;
        assert!(rcc_hclk <= rcc_hclk_max);
        let rcc_aclk = rcc_hclk; // AXI clock is always equal to AHB clock on H7

        // Calculate ppreN dividers and real rcc_pclkN frequencies
        ppre_calculate! {
            (ppre1, ppre1_bits):
                (self, rcc_hclk, rcc_pclk1, pclk_max, rcc_timx_ker_ck, timpre),
            (ppre2, ppre2_bits):
                (self, rcc_hclk, rcc_pclk2, pclk_max, rcc_timy_ker_ck, timpre),
            (ppre3, ppre3_bits): (self, rcc_hclk, rcc_pclk3, pclk_max),
            (ppre4, ppre4_bits): (self, rcc_hclk, rcc_pclk4, pclk_max),
        }

        // Calculate MCO dividers and real MCO frequencies
        let mco1_in = match self.config.mco1.source {
            // We set the required clock earlier, so can unwrap() here.
            MCO1::Hsi => HSI,
            MCO1::Lse => unimplemented!(),
            MCO1::Hse => self.config.hse.unwrap(),
            MCO1::Pll1Q => pll1_q_ck.unwrap().raw(),
            MCO1::Hsi48 => HSI48,
        };
        let (mco_1_pre, mco1_ck) =
            self.config.mco1.calculate_prescaler(mco1_in);

        let mco2_in = match self.config.mco2.source {
            // We set the required clock earlier, so can unwrap() here.
            MCO2::Sysclk => sys_ck.raw(),
            MCO2::Pll2P => pll2_p_ck.unwrap().raw(),
            MCO2::Hse => self.config.hse.unwrap(),
            MCO2::Pll1P => pll1_p_ck.unwrap().raw(),
            MCO2::Csi => CSI,
            MCO2::Lsi => LSI,
        };
        let (mco_2_pre, mco2_ck) =
            self.config.mco2.calculate_prescaler(mco2_in);

        // Start switching clocks here! ----------------------------------------

        // Flash setup
        Self::flash_setup(rcc_aclk, pwrcfg.vos);

        // Ensure CSI is on and stable
        rcc.cr().modify(|_, w| w.csion().on());
        while rcc.cr().read().csirdy().is_not_ready() {}

        // Ensure HSI48 is on and stable
        rcc.cr().modify(|_, w| w.hsi48on().on());
        while rcc.cr().read().hsi48rdy().is_not_ready() {}

        // Set the MCO outputs.
        //
        // It is highly recommended to configure these bits only after
        // reset, before enabling the external oscillators and the PLLs.
        rcc.cfgr().modify(|_, w| {
            w.mco1()
                .variant(self.config.mco1.source)
                .mco1pre()
                .set(mco_1_pre)
                .mco2()
                .variant(self.config.mco2.source)
                .mco2pre()
                .set(mco_2_pre)
        });

        // HSE
        let hse_ck = match self.config.hse {
            Some(hse) => {
                // Ensure HSE is on and stable
                rcc.cr().modify(|_, w| {
                    w.hseon().on().hsebyp().bit(self.config.bypass_hse)
                });
                while rcc.cr().read().hserdy().is_not_ready() {}

                Some(Hertz::from_raw(hse))
            }
            None => None,
        };

        // PLL
        let pllsrc = if self.config.hse.is_some() {
            PLLSRC::Hse
        } else {
            PLLSRC::Hsi
        };
        rcc.pllckselr().modify(|_, w| w.pllsrc().variant(pllsrc));

        // PLL1
        if pll1_p_ck.is_some() {
            // Enable PLL and wait for it to stabilise
            rcc.cr().modify(|_, w| w.pll1on().on());
            while rcc.cr().read().pll1rdy().is_not_ready() {}
        }

        // PLL2
        if pll2_p_ck.is_some() {
            // Enable PLL and wait for it to stabilise
            rcc.cr().modify(|_, w| w.pll2on().on());
            while rcc.cr().read().pll2rdy().is_not_ready() {}
        }

        // PLL3
        if pll3_p_ck.is_some() {
            // Enable PLL and wait for it to stabilise
            rcc.cr().modify(|_, w| w.pll3on().on());
            while rcc.cr().read().pll3rdy().is_not_ready() {}
        }

        // Core Prescaler / AHB Prescaler / APB3 Prescaler
        #[cfg(not(feature = "rm0455"))]
        rcc.d1cfgr().modify(|_, w| {
            w.d1cpre()
                .set(d1cpre_bits)
                .d1ppre() // D1 contains APB3
                .set(ppre3_bits)
                .hpre()
                .variant(hpre_bits)
        });
        #[cfg(feature = "rm0455")]
        rcc.cdcfgr1().modify(|_, w| {
            w.cdcpre()
                .set(d1cpre_bits)
                .cdppre() // D1/CD contains APB3
                .set(ppre3_bits)
                .hpre()
                .variant(hpre_bits)
        });
        // Ensure core prescaler value is valid before future lower
        // core voltage
        #[cfg(not(feature = "rm0455"))]
        while rcc.d1cfgr().read().d1cpre().bits() != d1cpre_bits {}
        #[cfg(feature = "rm0455")]
        while rcc.cdcfgr1().read().cdcpre().bits() != d1cpre_bits {}

        // APB1 / APB2 Prescaler
        #[cfg(not(feature = "rm0455"))]
        rcc.d2cfgr().modify(|_, w| {
            w.d2ppre1() // D2 contains APB1
                .set(ppre1_bits)
                .d2ppre2() // D2 also contains APB2
                .set(ppre2_bits)
        });
        #[cfg(feature = "rm0455")]
        rcc.cdcfgr2().modify(|_, w| {
            w.cdppre1() // D2/CD contains APB1
                .set(ppre1_bits)
                .cdppre2() // D2/CD also contains APB2
                .set(ppre2_bits)
        });

        // APB4 Prescaler
        #[cfg(not(feature = "rm0455"))]
        rcc.d3cfgr().modify(|_, w| {
            w.d3ppre() // D3 contains APB4
                .set(ppre4_bits)
        });
        #[cfg(feature = "rm0455")]
        rcc.srdcfgr().modify(|_, w| unsafe {
            w.srdppre() // D3 contains APB4
                .bits(ppre4_bits)
        });

        // Peripheral Clock (per_ck)
        #[cfg(not(feature = "rm0455"))]
        rcc.d1ccipr().modify(|_, w| w.ckpersel().variant(ckpersel));
        #[cfg(feature = "rm0455")]
        rcc.cdccipr().modify(|_, w| w.ckpersel().variant(ckpersel));

        // Set timer clocks prescaler setting
        rcc.cfgr().modify(|_, w| w.timpre().variant(timpre));

        // Select system clock source
        let swbits = match (sys_use_pll1_p, self.config.hse.is_some()) {
            (true, _) => SW::Pll1 as u8,
            (false, true) => SW::Hse as u8,
            _ => SW::Hsi as u8,
        };
        rcc.cfgr().modify(|_, w| unsafe { w.sw().bits(swbits) });
        while rcc.cfgr().read().sws().bits() != swbits {}

        // IO compensation cell - Requires CSI clock and SYSCFG
        assert!(rcc.cr().read().csirdy().is_ready());
        rcc.apb4enr().modify(|_, w| w.syscfgen().enabled());

        // Enable the compensation cell, using back-bias voltage code
        // provide by the cell.
        syscfg.cccsr().modify(|_, w| {
            w.en().set_bit().cs().clear_bit().hslv().clear_bit()
        });
        while syscfg.cccsr().read().ready().bit_is_clear() {}

        // This section prints the final register configuration for the main RCC registers:
        // - System Clock and PLL Source MUX
        // - PLL configuration
        // - System Prescalers
        // Does not include peripheral/MCO/RTC clock MUXes
        #[cfg(feature = "log")]
        {
            debug!("--- RCC register settings");

            let cfgr = rcc.cfgr().read();
            debug!(
                "CFGR register: SWS (System Clock Mux)={:?}",
                cfgr.sws().variant().unwrap()
            );

            #[cfg(not(feature = "rm0455"))]
            {
                let d1cfgr = rcc.d1cfgr().read();
                debug!(
                    "D1CFGR register: D1CPRE={:?} HPRE={:?} D1PPRE={:?}",
                    d1cfgr.d1cpre().variant(),
                    d1cfgr.hpre().variant(),
                    d1cfgr.d1ppre().variant(),
                );

                let d2cfgr = rcc.d2cfgr().read();
                debug!(
                    "D2CFGR register: D2PPRE1={:?} D2PPRE1={:?}",
                    d2cfgr.d2ppre1().variant(),
                    d2cfgr.d2ppre2().variant(),
                );

                let d3cfgr = rcc.d3cfgr().read();
                debug!(
                    "D3CFGR register: D3PPRE={:?}",
                    d3cfgr.d3ppre().variant(),
                );
            }
            #[cfg(feature = "rm0455")]
            {
                let cdcfgr1 = rcc.cdcfgr1().read();
                debug!(
                    "CDCFGR1 register: CDCPRE={:?} HPRE={:?} CDPPRE={:?}",
                    cdcfgr1.cdcpre().variant(),
                    cdcfgr1.hpre().variant(),
                    cdcfgr1.cdppre().variant(),
                );

                let cdcfgr2 = rcc.cdcfgr2().read();
                debug!(
                    "CDCFGR2 register: CDPPRE1={:?} CDPPRE1={:?}",
                    cdcfgr2.cdppre1().variant(),
                    cdcfgr2.cdppre2().variant(),
                );

                let srdcfgr = rcc.srdcfgr().read();
                debug!(
                    "SRDCFGR register: SRDPPRE={:?}",
                    srdcfgr.srdppre().bits(),
                );
            }

            let pllckselr = rcc.pllckselr().read();
            debug!(
                "PLLCKSELR register: PLLSRC={:?} DIVM1={:#x} DIVM2={:#x} DIVM3={:#x}",
                pllckselr.pllsrc().variant(),
                pllckselr.divm1().bits(),
                pllckselr.divm2().bits(),
                pllckselr.divm3().bits(),
            );

            let pllcfgr = rcc.pllcfgr().read();
            debug!(
                "PLLCKSELR register (PLL1): PLL1FRACEN={:?} PLL1VCOSEL={:?} PLL1RGE={:?} DIVP1EN={:?} DIVQ1EN={:?} DIVR1EN={:?}",
                pllcfgr.pll1fracen().variant(),
                pllcfgr.pll1vcosel().variant(),
                pllcfgr.pll1rge().variant(),
                pllcfgr.divp1en().variant(),
                pllcfgr.divq1en().variant(),
                pllcfgr.divr1en().variant(),
            );
            debug!(
                "PLLCKSELR register (PLL2): PLL2FRACEN={:?} PLL2VCOSEL={:?} PLL2RGE={:?} DIVP2EN={:?} DIVQ2EN={:?} DIVR2EN={:?}",
                pllcfgr.pll2fracen().variant(),
                pllcfgr.pll2vcosel().variant(),
                pllcfgr.pll2rge().variant(),
                pllcfgr.divp2en().variant(),
                pllcfgr.divq2en().variant(),
                pllcfgr.divr2en().variant(),
            );
            debug!(
                "PLLCKSELR register (PLL3): PLL3FRACEN={:?} PLL3VCOSEL={:?} PLL3RGE={:?} DIVP3EN={:?} DIVQ3EN={:?} DIVR3EN={:?}",
                pllcfgr.pll3fracen().variant(),
                pllcfgr.pll3vcosel().variant(),
                pllcfgr.pll3rge().variant(),
                pllcfgr.divp3en().variant(),
                pllcfgr.divq3en().variant(),
                pllcfgr.divr3en().variant(),
            );

            let pll1divr = rcc.pll1divr().read();
            debug!(
                "PLL1DIVR register: DIVN1={:#x} DIVP1={:#x} DIVQ1={:#x} DIVR1={:#x}",
                pll1divr.divn1().bits(),
                pll1divr.divp1().bits(),
                pll1divr.divq1().bits(),
                pll1divr.divr1().bits(),
            );

            let pll1fracr = rcc.pll1fracr().read();
            debug!(
                "PLL1FRACR register: FRACN1={:#x}",
                pll1fracr.fracn1().bits(),
            );

            let pll2divr = rcc.pll2divr().read();
            debug!(
                "PLL2DIVR register: DIVN2={:#x} DIVP2={:#x} DIVQ2={:#x} DIVR2={:#x}",
                pll2divr.divn2().bits(),
                pll2divr.divp2().bits(),
                pll2divr.divq2().bits(),
                pll2divr.divr2().bits(),
            );

            let pll2fracr = rcc.pll2fracr().read();
            debug!(
                "PLL2FRACR register: FRACN2={:#x}",
                pll2fracr.fracn2().bits(),
            );

            let pll3divr = rcc.pll3divr().read();
            debug!(
                "PLL3DIVR register: DIVN3={:#x} DIVP3={:#x} DIVQ3={:#x} DIVR3={:#x}",
                pll3divr.divn3().bits(),
                pll3divr.divp3().bits(),
                pll3divr.divq3().bits(),
                pll3divr.divr3().bits(),
            );

            let pll3fracr = rcc.pll3fracr().read();
            debug!(
                "PLL3FRACR register: FRACN3={:#x}",
                pll3fracr.fracn3().bits(),
            );
        }

        // Return frozen clock configuration
        Ccdr {
            clocks: CoreClocks {
                hclk: Hertz::from_raw(rcc_hclk),
                pclk1: Hertz::from_raw(rcc_pclk1),
                pclk2: Hertz::from_raw(rcc_pclk2),
                pclk3: Hertz::from_raw(rcc_pclk3),
                pclk4: Hertz::from_raw(rcc_pclk4),
                ppre1,
                ppre2,
                ppre3,
                ppre4,
                csi_ck: Some(Hertz::from_raw(csi)),
                hsi_ck: Some(Hertz::from_raw(hsi)),
                hsi48_ck: Some(Hertz::from_raw(hsi48)),
                lsi_ck: Some(Hertz::from_raw(lsi)),
                per_ck: Some(Hertz::from_raw(per_ck)),
                hse_ck,
                mco1_ck,
                mco2_ck,
                pll1_p_ck,
                pll1_q_ck,
                pll1_r_ck,
                pll2_p_ck,
                pll2_q_ck,
                pll2_r_ck,
                pll3_p_ck,
                pll3_q_ck,
                pll3_r_ck,
                timx_ker_ck: Hertz::from_raw(rcc_timx_ker_ck),
                timy_ker_ck: Hertz::from_raw(rcc_timy_ker_ck),
                sys_ck,
                c_ck: Hertz::from_raw(sys_d1cpre_ck),
            },
            peripheral: unsafe {
                // unsafe: we consume self which was a singleton, hence
                // we can safely create a singleton here
                PeripheralREC::new_singleton()
            },
            rb: self.rb,
        }
    }
}
