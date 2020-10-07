//! Structure to represent frozen core clock frequencies

use crate::time::rate::Hertz;

/// Frozen core clock frequencies
///
/// The existence of this value indicates that the core clock
/// configuration can no longer be changed
#[derive(Clone, Copy)]
pub struct CoreClocks {
    pub(super) hclk: Hertz,
    pub(super) pclk1: Hertz,
    pub(super) pclk2: Hertz,
    pub(super) pclk3: Hertz,
    pub(super) pclk4: Hertz,
    pub(super) ppre1: u8,
    pub(super) ppre2: u8,
    pub(super) ppre3: u8,
    pub(super) ppre4: u8,
    pub(super) csi_ck: Option<Hertz>,
    pub(super) hsi_ck: Option<Hertz>,
    pub(super) hsi48_ck: Option<Hertz>,
    pub(super) lsi_ck: Option<Hertz>,
    pub(super) per_ck: Option<Hertz>,
    pub(super) hse_ck: Option<Hertz>,
    pub(super) mco1_ck: Option<Hertz>,
    pub(super) mco2_ck: Option<Hertz>,
    pub(super) pll1_p_ck: Option<Hertz>,
    pub(super) pll1_q_ck: Option<Hertz>,
    pub(super) pll1_r_ck: Option<Hertz>,
    pub(super) pll2_p_ck: Option<Hertz>,
    pub(super) pll2_q_ck: Option<Hertz>,
    pub(super) pll2_r_ck: Option<Hertz>,
    pub(super) pll3_p_ck: Option<Hertz>,
    pub(super) pll3_q_ck: Option<Hertz>,
    pub(super) pll3_r_ck: Option<Hertz>,
    pub(super) timx_ker_ck: Hertz,
    pub(super) timy_ker_ck: Hertz,
    pub(super) sys_ck: Hertz,
    pub(super) c_ck: Hertz,
}

/// Getters for pclk and ppre
macro_rules! pclk_ppre_getter {
    ($(($pclk:ident, $ppre:ident),)+) => {
        $(
            /// Returns the frequency of the APBn
            pub fn $pclk(&self) -> Hertz {
                self.$pclk
            }
            /// Returns the prescaler of the APBn
            pub fn $ppre(&self) -> u8 {
                self.$ppre
            }
        )+
    };
}

/// Getters for optional clocks
macro_rules! optional_ck_getter {
    ($($opt_ck:ident: $doc:expr,)+) => {
        $(
            /// Returns `Some(frequency)` if
            #[doc=$doc]
            /// is running, otherwise `None`
            pub fn $opt_ck(&self) -> Option<Hertz> {
                self.$opt_ck
            }
        )+
    };
}

/// Getters for pll clocks
macro_rules! pll_getter {
    ($($pll_ck:ident,)+) => {
        $(
            /// Returns `Some(frequency)` if the PLLx output is running,
            /// otherwise `None`
            pub fn $pll_ck(&self) -> Option<Hertz> {
                self.$pll_ck
            }
        )+
    };
}

impl CoreClocks {
    /// Returns the frequency of AHB1,2,3 busses
    pub fn hclk(&self) -> Hertz {
        self.hclk
    }

    /// Returns the frequency of the AXI bus
    pub fn aclk(&self) -> Hertz {
        self.hclk // Same as HCLK
    }

    pclk_ppre_getter! {
        (pclk1, ppre1),
        (pclk2, ppre2),
        (pclk3, ppre3),
        (pclk4, ppre4),
    }

    optional_ck_getter! {
        csi_ck: "csi_ck",
        hsi_ck: "hsi_ck",
        hsi48_ck: "hsi48_ck",
        per_ck: "per_ck",
        hse_ck: "hse_ck",
        lsi_ck: "lsi_ck",
    }

    /// Returns `Some(frequency)` if the MCO1 output is running, otherwise
    /// `None`
    pub fn mco1_ck(&self) -> Option<Hertz> {
        self.mco1_ck
    }

    /// Returns `Some(frequency)` if the MCO2 output is running, otherwise
    /// `None`
    pub fn mco2_ck(&self) -> Option<Hertz> {
        self.mco2_ck
    }

    pll_getter! {
        pll1_p_ck,
        pll1_q_ck,
        pll1_r_ck,
        pll2_p_ck,
        pll2_q_ck,
        pll2_r_ck,
        pll3_p_ck,
        pll3_q_ck,
        pll3_r_ck,
    }

    /// Returns the input frequency to the SCGU
    pub fn sys_ck(&self) -> Hertz {
        self.sys_ck
    }

    /// Returns the input frequency to the SCGU - ALIAS
    pub fn sysclk(&self) -> Hertz {
        self.sys_ck
    }

    /// Returns the CK_INT frequency for timers on APB1
    pub fn timx_ker_ck(&self) -> Hertz {
        self.timx_ker_ck
    }

    /// Returns the CK_INT frequency for timers on APB2
    pub fn timy_ker_ck(&self) -> Hertz {
        self.timy_ker_ck
    }

    /// Returns the core frequency
    pub fn c_ck(&self) -> Hertz {
        self.c_ck
    }
}
