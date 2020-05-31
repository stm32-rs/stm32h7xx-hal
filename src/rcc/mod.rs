//! Reset and Clock Control
//!
//! This module configures the RCC unit to provide set frequencies for
//! the input to the SCGU `sys_ck`, the AMBA High-performance Busses
//! and Advanced eXtensible Interface bus `hclk`, the AMBA Peripheral
//! Busses `pclkN` and the peripheral clock `per_ck`.
//!
//! See Fig 46 "Core and bus clock generation" in Reference Manual
//! RM0433 for information (p 336).
//!
//! HSI is 64 MHz.
//! CSI is 4 MHz.
//! HSI48 is 48MHz.
//!
//! # Usage
//!
//! This peripheral is must be used alongside the
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
//! `use_hse(b)` was specified
//!
//! * `pll1_p_ck(c)` implies `pll1_r_ck(c/2)`, including when
//! `pll1_p_ck` was implied by `sys_ck(c)`.
//!
//! Implied clock specifications can always be overridden by explicitly
//! specifying that clock. If this results in a configuration that cannot
//! be achieved by hardware, `freeze` will panic.
//!
//! # Configuration Example
//!
//! A simple example:
//!
//! ```rust
//!     let dp = pac::Peripherals::take().unwrap();
//!
//!     let pwr = dp.PWR.constrain();
//!     let vos = pwr.freeze();
//!
//!     let rcc = dp.RCC.constrain();
//!     let ccdr = rcc
//!         .sys_ck(96.mhz())
//!         .pclk1(48.mhz())
//!         .freeze(vos, &dp.SYSCFG);
//! ```
//!
//! A more complex example, involving the PLL:
//!
//! ```rust
//!     let dp = pac::Peripherals::take().unwrap();
//!
//!     let pwr = dp.PWR.constrain();
//!     let vos = pwr.freeze();
//!
//!     let rcc = dp.RCC.constrain();
//!     let ccdr = rcc
//!         .sys_ck(200.mhz()) // Implies pll1_p_ck
//!         // For non-integer values, round up. `freeze` will never
//!         // configure a clock faster than that specified.
//!         .pll1_q_ck(33_333_334.hz())
//!         .freeze(vos, &dp.SYSCFG);
//! ```
//!
//! A much more complex example, indicative of real usage with a
//! significant fraction of the STM32H7's capabilities.
//!
//! ```rust
//!     let dp = pac::Peripherals::take().unwrap();
//!
//!     let pwr = dp.PWR.constrain();
//!     let vos = pwr.freeze();
//!
//!     let rcc = dp.RCC.constrain();
//!     let ccdr = rcc
//!         .use_hse(25.mhz()) // XTAL X1
//!         .sys_ck(400.mhz())
//!         .pll1_r_ck(100.mhz()) // for TRACECK
//!         .pll1_q_ck(200.mhz())
//!         .hclk(200.mhz())
//!         .pll3_strategy(PllConfigStrategy::Iterative)
//!         .pll3_p_ck(240.mhz()) // for LTDC
//!         .pll3_q_ck(48.mhz()) // for LTDC
//!         .pll3_r_ck(26_666_667.hz()) // Pixel clock for LTDC
//!         .freeze(vos, &dp.SYSCFG);
//!```
//!
//! # Peripherals
//!
//! The `freeze()` method returns a [Core Clocks Distribution and Reset
//! (CCDR)](struct.Ccdr.html) object. This singleton tells you how the core
//! clocks were actually configured (in
//! [CoreClocks](struct.CoreClocks.html)) and allows you to configure the
//! remaining peripherals (see [PeripheralREC](struct.PeripheralREC.html)).
//!
//!```rust
//! let ccdr = ...; // Returned by `freeze()`, see examples above
//!
//! // Runtime confirmation that hclk really is 200MHz
//! assert_eq!(ccdr.clocks.hclk().0, 200_000_000);
//!
//! // Panics if pll1_q_ck is not running
//! let _ = ccdr.clocks.pll1_q_ck().unwrap();
//!
//! // Enable the clock to a peripheral and reset it
//! ccdr.peripheral.FDCAN.enable().reset();
//!```
//!
//! The [PeripheralREC](struct.PeripheralREC.html) members implement move
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

use crate::pwr::VoltageScale as Voltage;
use crate::stm32::rcc::cfgr::SW_A as SW;
use crate::stm32::rcc::cfgr::TIMPRE_A as TIMPRE;
use crate::stm32::rcc::d1ccipr::CKPERSEL_A as CKPERSEL;
use crate::stm32::rcc::d1cfgr::HPRE_A as HPRE;
use crate::stm32::rcc::pllckselr::PLLSRC_A as PLLSRC;
use crate::stm32::{RCC, SYSCFG};
use crate::time::Hertz;

mod core_clocks;
mod pll;
pub mod rec;

pub use core_clocks::CoreClocks;
pub use pll::{PllConfig, PllConfigStrategy};
pub use rec::{PeripheralREC, ResetEnable};

/// Configuration of the core clocks
pub struct Config {
    hse: Option<u32>,
    sys_ck: Option<u32>,
    per_ck: Option<u32>,
    rcc_hclk: Option<u32>,
    rcc_pclk1: Option<u32>,
    rcc_pclk2: Option<u32>,
    rcc_pclk3: Option<u32>,
    rcc_pclk4: Option<u32>,
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
                sys_ck: None,
                per_ck: None,
                rcc_hclk: None,
                rcc_pclk1: None,
                rcc_pclk2: None,
                rcc_pclk3: None,
                rcc_pclk4: None,
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

/// Setter defintion for pclk 1 - 4
macro_rules! pclk_setter {
    ($($name:ident: $pclk:ident,)+) => {
        $(
            /// Set the peripheral clock frequency for APB
            /// peripherals.
            pub fn $name<F>(mut self, freq: F) -> Self
            where
                F: Into<Hertz>,
            {
                self.config.$pclk = Some(freq.into().0);
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
                pub fn $name<F>(mut self, freq: F) -> Self
                where
                    F: Into<Hertz>,
                {
                    self.config.$pll.$ck = Some(freq.into().0);
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
    pub fn use_hse<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.config.hse = Some(freq.into().0);
        self
    }

    /// Set input frequency to the SCGU
    pub fn sys_ck<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.config.sys_ck = Some(freq.into().0);
        self
    }

    /// Set input frequency to the SCGU - ALIAS
    pub fn sysclk<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.config.sys_ck = Some(freq.into().0);
        self
    }

    /// Set peripheral clock frequency
    pub fn per_ck<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.config.per_ck = Some(freq.into().0);
        self
    }

    /// Set the peripheral clock frequency for AHB and AXI
    /// peripherals. There are several gated versions `rcc_hclk[1-4]`
    /// for different power domains, but they are all the same frequency
    pub fn hclk<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.config.rcc_hclk = Some(freq.into().0);
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
            let ($bits, $ppre) = match ($hclk + $pclk - 1) / $pclk
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
                    (0b101, TIMPRE::DEFAULTX2) => $hclk / 2,
                    (0b110, TIMPRE::DEFAULTX4) => $hclk / 2,
                    (0b110, TIMPRE::DEFAULTX2) => $hclk / 4,
                    (0b111, TIMPRE::DEFAULTX4) => $hclk / 4,
                    (0b111, TIMPRE::DEFAULTX2) => $hclk / 8,
                    _ => $hclk,
                };
            )*
        )+
    };
}

impl Rcc {
    fn flash_setup(rcc_aclk: u32, vos: Voltage) {
        use crate::stm32::FLASH;
        let rcc_aclk_mhz = rcc_aclk / 1_000_000;

        // See RM0433 Table 13. FLASH recommended number of wait
        // states and programming delay
        let (wait_states, progr_delay) = match vos {
            // VOS 1 range VCORE 1.15V - 1.26V
            Voltage::Scale0 | Voltage::Scale1 => match rcc_aclk_mhz {
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
                225 => (4, 2),
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

        let flash = unsafe { &(*FLASH::ptr()) };
        // Adjust flash wait states
        flash.acr.write(|w| unsafe {
            w.wrhighfreq().bits(progr_delay).latency().bits(wait_states)
        });
        while flash.acr.read().latency().bits() != wait_states {}
    }

    /// Setup sys_ck
    /// Returns sys_ck frequency, and a pll1_p_ck
    fn sys_ck_setup(&self) -> (Hertz, Option<u32>, bool) {
        // Compare available with wanted clocks
        let srcclk = self.config.hse.unwrap_or(HSI); // Available clocks
        let sys_ck = self.config.sys_ck.unwrap_or(srcclk);

        // The requested system clock is not the immediately available
        // HSE/HSI clock. Perhaps there are other ways of obtaining
        // the requested system clock (such as `HSIDIV`) but we will
        // ignore those for now.
        if sys_ck != srcclk {
            // Therefore we must use pll1_p_ck
            let pll1_p_ck = match self.config.pll1.p_ck {
                Some(p_ck) => {
                    assert!(p_ck == sys_ck,
                            "Error: Cannot set pll1_p_ck independently as it must be used to generate sys_ck");
                    Some(p_ck)
                }
                None => Some(sys_ck),
            };

            (Hertz(sys_ck), pll1_p_ck, true)
        } else {
            // sys_ck is derived directly from a source clock
            // (HSE/HSI). pll1_p_ck can be as requested
            (Hertz(sys_ck), self.config.pll1.p_ck, false)
        }
    }

    /// Setup traceclk
    /// Returns a pll1_r_ck
    fn traceclk_setup(
        &self,
        sys_use_pll1_p: bool,
        pll1_p_ck: Option<u32>,
    ) -> Option<u32> {
        match (sys_use_pll1_p, self.config.pll1.r_ck) {
            // pll1_p_ck selected as system clock but pll1_r_ck not
            // set. The traceclk mux is synchronous with the system
            // clock mux, but has pll1_r_ck as an input. In order to
            // keep traceclk running, we force a pll1_r_ck.
            (true, None) => Some(pll1_p_ck.unwrap() / 2),
            // Either pll1 not selected as system clock, free choice
            // of pll1_r_ck. Or pll1 is selected, assume user has set
            // a suitable pll1_r_ck frequency.
            _ => self.config.pll1.r_ck,
        }
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
    pub fn freeze(self, vos: Voltage, syscfg: &SYSCFG) -> Ccdr {
        let rcc = &self.rb;

        // We do not reset RCC here. This routine must assert when
        // the previous state of the RCC peripheral is unacceptable.

        // sys_ck from PLL if needed, else HSE or HSI
        let (sys_ck, pll1_p_ck, sys_use_pll1_p) = self.sys_ck_setup();

        // Configure traceclk from PLL if needed
        let pll1_r_ck = self.traceclk_setup(sys_use_pll1_p, pll1_p_ck);

        // Configure PLL1
        let pll1_config = PllConfig {
            strategy: self.config.pll1.strategy,
            p_ck: pll1_p_ck,
            q_ck: self.config.pll1.q_ck,
            r_ck: pll1_r_ck,
        };
        let (pll1_p_ck, pll1_q_ck, pll1_r_ck) =
            self.pll1_setup(rcc, &pll1_config);
        // Configure PLL2
        let (pll2_p_ck, pll2_q_ck, pll2_r_ck) =
            self.pll2_setup(rcc, &self.config.pll2);
        // Configure PLL3
        let (pll3_p_ck, pll3_q_ck, pll3_r_ck) =
            self.pll3_setup(rcc, &self.config.pll3);

        // hsi_ck = HSI. This routine does not support HSIDIV != 1. To
        // do so it would need to ensure all PLLxON bits are clear
        // before changing the value of HSIDIV
        let hsi = HSI;
        assert!(rcc.cr.read().hsion().is_on(), "HSI oscillator must be on!");
        assert!(rcc.cr.read().hsidiv().is_div1());

        let csi = CSI;
        let hsi48 = HSI48;

        // per_ck from HSI by default
        let (per_ck, ckpersel) =
            match (self.config.per_ck == self.config.hse, self.config.per_ck) {
                (true, Some(hse)) => (hse, CKPERSEL::HSE), // HSE
                (_, Some(CSI)) => (csi, CKPERSEL::CSI),    // CSI
                _ => (hsi, CKPERSEL::HSI),                 // HSI
            };

        // D1 Core Prescaler
        // Set to 1
        let d1cpre_bits = 0;
        let d1cpre_div = 1;
        let sys_d1cpre_ck = sys_ck.0 / d1cpre_div;

        // Timer prescaler selection
        let timpre = TIMPRE::DEFAULTX2;

        // Refer to part datasheet "General operating conditions"
        // table for (rev V). We do not assert checks for earlier
        // revisions which may have lower limits.
        let (sys_d1cpre_ck_max, rcc_hclk_max, pclk_max) = match vos {
            Voltage::Scale0 => (480_000_000, 240_000_000, 120_000_000),
            Voltage::Scale1 => (400_000_000, 200_000_000, 100_000_000),
            Voltage::Scale2 => (300_000_000, 150_000_000, 75_000_000),
            _ => (200_000_000, 100_000_000, 50_000_000),
        };

        // Check resulting sys_d1cpre_ck
        assert!(sys_d1cpre_ck <= sys_d1cpre_ck_max);

        // Get ideal AHB clock
        let rcc_hclk = self.config.rcc_hclk.unwrap_or(sys_d1cpre_ck / 2);
        assert!(rcc_hclk <= rcc_hclk_max);

        // Estimate divisor
        let (hpre_bits, hpre_div) =
            match (sys_d1cpre_ck + rcc_hclk - 1) / rcc_hclk {
                0 => unreachable!(),
                1 => (HPRE::DIV1, 1),
                2 => (HPRE::DIV2, 2),
                3..=5 => (HPRE::DIV4, 4),
                6..=11 => (HPRE::DIV8, 8),
                12..=39 => (HPRE::DIV16, 16),
                40..=95 => (HPRE::DIV64, 64),
                96..=191 => (HPRE::DIV128, 128),
                192..=383 => (HPRE::DIV256, 256),
                _ => (HPRE::DIV512, 512),
            };

        // Calculate real AXI and AHB clock
        let rcc_hclk = sys_d1cpre_ck / hpre_div;
        assert!(rcc_hclk <= rcc_hclk_max);

        // Calculate ppreN dividers and real rcc_pclkN frequencies
        ppre_calculate! {
            (ppre1, ppre1_bits):
                (self, rcc_hclk, rcc_pclk1, pclk_max, rcc_timx_ker_ck, timpre),
            (ppre2, ppre2_bits):
                (self, rcc_hclk, rcc_pclk2, pclk_max, rcc_timy_ker_ck, timpre),
            (ppre3, ppre3_bits): (self, rcc_hclk, rcc_pclk3, pclk_max),
            (ppre4, ppre4_bits): (self, rcc_hclk, rcc_pclk4, pclk_max),
        }

        // Start switching clocks here! ----------------------------------------

        // Flash setup
        Self::flash_setup(sys_d1cpre_ck, vos);

        // Ensure CSI is on and stable
        rcc.cr.modify(|_, w| w.csion().on());
        while rcc.cr.read().csirdy().is_not_ready() {}

        // Ensure HSI48 is on and stable
        rcc.cr.modify(|_, w| w.hsi48on().on());
        while rcc.cr.read().hsi48rdy().is_not_ready() {}

        // HSE
        let hse_ck = match self.config.hse {
            Some(hse) => {
                // Ensure HSE is on and stable
                rcc.cr.modify(|_, w| w.hseon().on().hsebyp().not_bypassed());
                while rcc.cr.read().hserdy().is_not_ready() {}

                Some(Hertz(hse))
            }
            None => None,
        };

        // PLL
        let pllsrc = if self.config.hse.is_some() {
            PLLSRC::HSE
        } else {
            PLLSRC::HSI
        };
        rcc.pllckselr.modify(|_, w| w.pllsrc().variant(pllsrc));

        // PLL1
        if pll1_p_ck.is_some() {
            // Enable PLL and wait for it to stabilise
            rcc.cr.modify(|_, w| w.pll1on().on());
            while rcc.cr.read().pll1rdy().is_not_ready() {}
        }

        // PLL2
        if pll2_p_ck.is_some() {
            // Enable PLL and wait for it to stabilise
            rcc.cr.modify(|_, w| w.pll2on().on());
            while rcc.cr.read().pll2rdy().is_not_ready() {}
        }

        // PLL3
        if pll3_p_ck.is_some() {
            // Enable PLL and wait for it to stabilise
            rcc.cr.modify(|_, w| w.pll3on().on());
            while rcc.cr.read().pll3rdy().is_not_ready() {}
        }

        // Core Prescaler / AHB Prescaler / APB3 Prescaler
        rcc.d1cfgr.modify(|_, w| unsafe {
            w.d1cpre()
                .bits(d1cpre_bits)
                .d1ppre() // D1 contains APB3
                .bits(ppre3_bits)
                .hpre()
                .variant(hpre_bits)
        });
        // Ensure core prescaler value is valid before future lower
        // core voltage
        while rcc.d1cfgr.read().d1cpre().bits() != d1cpre_bits {}

        // APB1 / APB2 Prescaler
        rcc.d2cfgr.modify(|_, w| unsafe {
            w.d2ppre1() // D2 contains APB1
                .bits(ppre1_bits)
                .d2ppre2() // D2 also contains APB2
                .bits(ppre2_bits)
        });

        // APB4 Prescaler
        rcc.d3cfgr.modify(|_, w| unsafe {
            w.d3ppre() // D3 contains APB4
                .bits(ppre4_bits)
        });

        // Peripheral Clock (per_ck)
        rcc.d1ccipr.modify(|_, w| w.ckpersel().variant(ckpersel));

        // Set timer clocks prescaler setting
        rcc.cfgr.modify(|_, w| w.timpre().variant(timpre));

        // Select system clock source
        let swbits = match (sys_use_pll1_p, self.config.hse.is_some()) {
            (true, _) => SW::PLL1 as u8,
            (false, true) => SW::HSE as u8,
            _ => SW::HSI as u8,
        };
        rcc.cfgr.modify(|_, w| unsafe { w.sw().bits(swbits) });
        while rcc.cfgr.read().sws().bits() != swbits {}

        // IO compensation cell - Requires CSI clock and SYSCFG
        assert!(rcc.cr.read().csirdy().is_ready());
        rcc.apb4enr.modify(|_, w| w.syscfgen().enabled());

        // Enable the compensation cell, using back-bias voltage code
        // provide by the cell.
        syscfg.cccsr.modify(|_, w| {
            w.en().set_bit().cs().clear_bit().hslv().clear_bit()
        });
        while syscfg.cccsr.read().ready().bit_is_clear() {}

        // Return frozen clock configuration
        Ccdr {
            clocks: CoreClocks {
                hclk: Hertz(rcc_hclk),
                pclk1: Hertz(rcc_pclk1),
                pclk2: Hertz(rcc_pclk2),
                pclk3: Hertz(rcc_pclk3),
                pclk4: Hertz(rcc_pclk4),
                ppre1,
                ppre2,
                ppre3,
                ppre4,
                csi_ck: Some(Hertz(csi)),
                hsi_ck: Some(Hertz(hsi)),
                hsi48_ck: Some(Hertz(hsi48)),
                per_ck: Some(Hertz(per_ck)),
                hse_ck,
                pll1_p_ck,
                pll1_q_ck,
                pll1_r_ck,
                pll2_p_ck,
                pll2_q_ck,
                pll2_r_ck,
                pll3_p_ck,
                pll3_q_ck,
                pll3_r_ck,
                timx_ker_ck: Hertz(rcc_timx_ker_ck),
                timy_ker_ck: Hertz(rcc_timy_ker_ck),
                sys_ck,
                c_ck: Hertz(sys_d1cpre_ck),
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
