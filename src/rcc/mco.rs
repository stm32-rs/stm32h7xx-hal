//! Micro-Controller Out (MCO) pins

use super::Rcc;
use crate::time::Hertz;

pub use crate::stm32::rcc::cfgr::MCO1_A as MCO1;
pub use crate::stm32::rcc::cfgr::MCO2_A as MCO2;

/// Clock settings for Micro-Controller Out 1 (MCO1)
pub struct MCO1Config {
    pub(super) source: MCO1,
    pub(super) frequency: Option<u32>,
}
impl Default for MCO1Config {
    fn default() -> MCO1Config {
        Self {
            source: MCO1::Hsi,
            frequency: None,
        }
    }
}

/// Clock settings for Micro-Controller Out 2 (MCO2)
pub struct MCO2Config {
    pub(super) source: MCO2,
    frequency: Option<u32>,
}
impl Default for MCO2Config {
    fn default() -> MCO2Config {
        Self {
            source: MCO2::Sysclk,
            frequency: None,
        }
    }
}

macro_rules! calculate_prescaler {
    () => {
        /// Calculates the prescaler and the resulting clock frequency
        pub(super) fn calculate_prescaler(
            &self,
            in_ck: u32,
        ) -> (u8, Option<Hertz>) {
            // Running?
            if let Some(freq) = self.frequency {
                // Calculate prescaler
                let prescaler = match (in_ck + freq - 1) / freq {
                    0 => unreachable!(),
                    x @ 1..=15 => x,
                    _ => {
                        panic!("Clock is too fast to achieve {} Hz MCO!", freq)
                    }
                };

                (prescaler as u8, Some(Hertz::from_raw(in_ck / prescaler)))
            } else {
                // Disabled
                (0, None)
            }
        }
    };
}
impl MCO1Config {
    calculate_prescaler!();
}
impl MCO2Config {
    calculate_prescaler!();
}

impl Rcc {
    /// Checks the MCO1 setup and sets further requirements in `config` if they
    /// are currently set to `None`
    ///
    /// # Panics
    ///
    /// Panics if the MCO1 setup is invalid, or if it is inconsistent with the
    /// rest of the `config`
    pub(super) fn mco1_setup(&mut self) {
        // HSI always runs

        // LSE unimplemented

        // HSE must be explicitly stated
        if self.config.mco1.source == MCO1::Hse {
            assert!(
                self.config.hse.is_some(),
                "HSE is required for MCO1. Explicitly state its frequency with `use_hse`"
            );
        }

        // Set pll1_q_ck based on requirement
        if self.config.mco1.source == MCO1::Pll1Q
            && self.config.pll1.q_ck.is_none()
        {
            self.config.pll1.q_ck = self.config.mco1.frequency;
        }

        // HSI48 always runs
    }

    /// Checks the MCO2 setup and sets further requirements in `config` if they
    /// are currently set to `None`
    ///
    /// # Panics
    ///
    /// Panics if the MCO2 setup is invalid, or if it is inconsistent with the
    /// rest of the `config`
    pub(super) fn mco2_setup(&mut self) {
        // Set sysclk based on requirement
        if self.config.mco2.source == MCO2::Sysclk
            && self.config.sys_ck.is_none()
        {
            self.config.sys_ck = self.config.mco2.frequency;
        }

        // Set pll2_p_ck based on requirement
        if self.config.mco2.source == MCO2::Pll2P
            && self.config.pll2.p_ck.is_none()
        {
            self.config.pll2.p_ck = self.config.mco2.frequency;
        }

        // HSE must be explicitly stated
        if self.config.mco2.source == MCO2::Hse {
            assert!(
                self.config.hse.is_some(),
                "HSE is required for MCO2. Explicitly state its frequency with `use_hse`"
            );
        }

        // Set pll1_p_ck based on requirement
        if self.config.mco2.source == MCO2::Pll1P
            && self.config.pll1.p_ck.is_none()
        {
            self.config.pll1.p_ck = self.config.mco2.frequency;
        }

        // CSI always runs

        // LSI unimplemented
    }
}

macro_rules! mco1_setters {
    ($($mco_setter:ident: $source:ident $doc:expr),+) => {
        /// Setters for Micro-Controller Out 1 (MCO1)
        impl Rcc {
            $(
                /// Set the MCO1 output frequency. The clock is sourced from
                #[doc=$doc]
                ///
                /// This only enables the signal within the RCC block, it does
                /// not enable the MCO1 output pin itself (use the GPIO for
                /// that).
                #[must_use]
                pub fn $mco_setter(mut self, freq: Hertz) -> Self {
                    self.config.mco1.source = MCO1::$source;
                    self.config.mco1.frequency = Some(freq.raw());
                    self
                }
            )+
        }
    }
}
mco1_setters! {
    mco1_from_hsi: Hsi "the HSI",
    //mco1_from_lse: Lse "the LSE",    UNIMPLEMENTED
    mco1_from_hse: Hse "the HSE",
    mco1_from_pll1_q_ck: Pll1Q "pll1_q_ck",
    mco1_from_hsi48: Hsi48 "HSI48"
}

macro_rules! mco2_setters {
    ($($mco_setter:ident: $source:ident $doc:expr),+) => {
        /// Setters for Micro-Controller Out 2 (MCO2)
        impl Rcc {
            $(
                /// Set the MCO2 output frequency. The clock is sourced from
                #[doc=$doc]
                ///
                /// This only enables the signal within the RCC block, it does
                /// not enable the MCO2 output pin itself (use the GPIO for
                /// that).
                #[must_use]
                pub fn $mco_setter(mut self, freq: Hertz) -> Self {
                    self.config.mco2.source = MCO2::$source;
                    self.config.mco2.frequency = Some(freq.raw());
                    self
                }
            )+
        }
    }
}
mco2_setters! {
    mco2_from_sys_ck: Sysclk "sys_ck",
    mco2_from_pll2_p_ck: Pll2P "pll2_p_ck",
    mco2_from_hse: Hse "the HSE",
    mco2_from_pll1_p_ck: Pll1P "pll1_p_ck",
    mco2_from_csi: Csi "CSI"
    //mco2_from_lsi: Lsi "the LSI",    UNIMPLEMENTED
}
