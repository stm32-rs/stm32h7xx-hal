//! Phase Locked Loop Configuration

use super::{Rcc, HSI};
use crate::stm32::RCC;
use crate::time::Hertz;

/// Strategies for configuring a Phase Locked Loop (PLL)
#[derive(Copy, Clone)]
pub enum PllConfigStrategy {
    /// VCOL, highest PFD frequency, highest VCO frequency
    Normal,
    /// VCOH, choose PFD frequency for accuracy, highest VCO frequency
    Iterative,
}

/// Configuration of a Phase Locked Loop (PLL)
pub struct PllConfig {
    pub(super) strategy: PllConfigStrategy,
    pub(super) p_ck: Option<u32>,
    pub(super) q_ck: Option<u32>,
    pub(super) r_ck: Option<u32>,
}
impl Default for PllConfig {
    fn default() -> PllConfig {
        PllConfig {
            strategy: PllConfigStrategy::Normal,
            p_ck: None,
            q_ck: None,
            r_ck: None,
        }
    }
}

/// Calculate VCO output divider (p-divider). Choose the highest VCO
/// frequency to give specified output.
///
/// Returns *target* VCO frequency
///
macro_rules! vco_output_divider_setup {
    ($output: ident, $vco_min: ident, $vco_max: ident $(,$pll1_p:ident)*) => {{
        // Macro-based selection
        #[allow(clippy::match_bool)]
        let pll_x_p = match true {
            $(
                // Specific to PLL1
                true => {
                    let $pll1_p = if $output > $vco_max / 2 {
                        1
                    } else {
                        (($vco_max / $output) | 1) - 1 // Must be even or unity
                    };
                    $pll1_p
                },
            )*
                // Specific to PLL2/3
                _ => if $output > $vco_max / 2 {
                    1
                } else {
                    $vco_max / $output
                }
        };

        // Calcuate VCO output
        let vco_ck = $output * pll_x_p;

        assert!(pll_x_p <= 128);
        assert!(vco_ck >= $vco_min);
        assert!(vco_ck <= $vco_max);

        (vco_ck, pll_x_p)
    }};
}

/// Setup PFD input frequency and VCO output frequency
///
macro_rules! vco_setup {
    // Normal: VCOL, highest PFD frequency, highest VCO frequency
    (NORMAL: $pllsrc:ident, $output:ident,
     $rcc:ident, $pllXvcosel:ident, $pllXrge:ident $(,$pll1_p:ident)*) => {{
         // VCO output frequency. Choose the highest VCO frequency
         let vco_min = 150_000_000;
         let vco_max = 420_000_000;
         let (vco_ck_target, pll_x_p) = {
             vco_output_divider_setup! { $output, vco_min, vco_max $(, $pll1_p)* }
         };

         // Input divisor, resulting in a reference clock in the range
         // 1 to 2 MHz. Choose the highest reference clock (lowest m)
         let pll_x_m = ($pllsrc + 1_999_999) / 2_000_000;

         assert!(pll_x_m < 64);

         // Calculate resulting reference clock
         let ref_x_ck = $pllsrc / pll_x_m;
         assert!(ref_x_ck >= 1_000_000 && ref_x_ck <= 2_000_000);

         // Configure VCO
         $rcc.pllcfgr.modify(|_, w| {
             w.$pllXvcosel()
                 .medium_vco() // 150 - 420MHz Medium VCO
                 .$pllXrge()
                 .range1() // ref_x_ck is 1 - 2 MHz
         });

         (ref_x_ck, pll_x_m, pll_x_p, vco_ck_target)
     }};
    // Iterative: VCOH, choose PFD frequency for accuracy, highest VCO frequency
    (ITERATIVE: $pllsrc:ident, $output:ident,
     $rcc:ident, $pllXvcosel:ident, $pllXrge:ident $(,$pll1_p:ident)*) => {{
         // VCO output frequency limits
         let vco_min = 192_000_000;
         #[cfg(not(feature = "revision_v"))]
         let vco_max = 836_000_000;
         #[cfg(feature = "revision_v")]
         let vco_max = 960_000_000;

         // VCO output frequency. Choose the highest VCO frequency
         let (vco_ck_target, pll_x_p) = {
             vco_output_divider_setup! { $output, vco_min, vco_max $(, $pll1_p)* }
         };

         // Input divisor, resulting in a reference clock in the
         // range 2 to 16 MHz.
         let pll_x_m_min = ($pllsrc + 15_999_999) / 16_000_000;
         let pll_x_m_max = match $pllsrc {
             0 ..= 127_999_999 => $pllsrc / 2_000_000,
             _ => 63            // pllm < 64
         };

         // Iterative search for the lowest m value that minimizes
         // the difference between requested and actual VCO frequency
         let pll_x_m = (pll_x_m_min..=pll_x_m_max).min_by_key(|pll_x_m| {
             let ref_x_ck = $pllsrc / pll_x_m;

             // Feedback divider. Integer only
             let pll_x_n = vco_ck_target / ref_x_ck;

             vco_ck_target as i32 - (ref_x_ck * pll_x_n) as i32
         }).unwrap();

         assert!(pll_x_m < 64);

         // Calculate resulting reference clock
         let ref_x_ck = $pllsrc / pll_x_m;
         assert!(ref_x_ck >= 2_000_000 && ref_x_ck <= 16_000_000);

         // Configure VCO
         $rcc.pllcfgr.modify(|_, w| {
             w.$pllXvcosel()
                 .wide_vco() // 192 - 836MHz Medium VCO
         });
         $rcc.pllcfgr.modify(|_, w| {
             match ref_x_ck {
                 2_000_000 ..= 3_999_999 => // ref_x_ck is 2 - 4 MHz
                     w.$pllXrge().range2(),
                 4_000_000 ..= 7_999_999 => // ref_x_ck is 4 - 8 MHz
                     w.$pllXrge().range4(),
                 _ =>           // ref_x_ck is 8 - 16 MHz
                     w.$pllXrge().range8(),
             }
         });

         (ref_x_ck, pll_x_m, pll_x_p, vco_ck_target)
     }};
}

macro_rules! pll_setup {
    ($pll_setup:ident: ($pllXvcosel:ident, $pllXrge:ident, $pllXfracen:ident,
                   $pllXdivr:ident, $divnX:ident, $divmX:ident,
                   OUTPUTS: [ $($CK:ident:
                                ($div:ident, $diven:ident, $DD:tt $(,$unsafe:ident)*)),+ ]
                   $(,$pll1_p:ident)*
    )) => {
        /// PLL Setup
        /// Returns (Option(pllX_p_ck), Option(pllX_q_ck), Option(pllX_r_ck))
        pub(super) fn $pll_setup(
            &self,
            rcc: &RCC,
            pll: &PllConfig,
        ) -> (Option<Hertz>, Option<Hertz>, Option<Hertz>) {
            // PLL sourced from either HSE or HSI
            let pllsrc = self.config.hse.unwrap_or(HSI);
            assert!(pllsrc > 0);

            // PLL output
            match pll.p_ck {
                Some(output) => {
                    // Set VCO parameters based on VCO strategy
                    let (ref_x_ck, pll_x_m, pll_x_p, vco_ck) =
                        match pll.strategy {
                            PllConfigStrategy::Iterative => {
                                vco_setup! { ITERATIVE: pllsrc, output,
                                             rcc, $pllXvcosel,
                                             $pllXrge $(, $pll1_p)* }
                            },
                            _ => {
                                vco_setup! { NORMAL: pllsrc, output,
                                             rcc, $pllXvcosel,
                                             $pllXrge $(, $pll1_p)* }
                            }

                        };

                    // Feedback divider. Integer only
                    let pll_x_n = vco_ck / ref_x_ck;

                    // Write dividers
                    rcc.pllckselr.modify(|_, w| {
                        w.$divmX().bits(pll_x_m as u8) // ref prescaler
                    });
                    // unsafe as not all values are permitted: see RM0433
                    assert!(pll_x_n >= 4);
                    assert!(pll_x_n <= 512);
                    rcc.$pllXdivr
                        .modify(|_, w| unsafe { w.$divnX().bits((pll_x_n - 1) as u16) });

                    // Configure PLL
                    rcc.pllcfgr.modify(|_, w| {
                        w.$pllXfracen().reset() // No FRACN
                    });

                    // Calulate additional output dividers
                    let pll_x_q = match pll.q_ck {
                        Some(ck) => (vco_ck + ck - 1) / ck,
                        None => 0
                    };
                    let pll_x_r = match pll.r_ck {
                        Some(ck) => (vco_ck + ck - 1) / ck,
                        None => 0
                    };
                    let dividers = (pll_x_p, pll_x_q, pll_x_r);

                    // Setup and return output clocks
                    ($(
                        // Enable based on config
                        match pll.$CK {
                            Some(_) => {
                                // Setup divider
                                rcc.$pllXdivr
                                    .modify(|_, w| $($unsafe)* {
                                        w.$div().bits((dividers.$DD - 1) as u8)
                                    });

                                rcc.pllcfgr.modify(|_, w| w.$diven().enabled());
                                Some(Hertz(ref_x_ck * pll_x_n / dividers.$DD))
                            }
                            None => {
                                rcc.pllcfgr.modify(|_, w| w.$diven().disabled());
                                None
                            }
                        },
                    )+)
                },
                None => {
                    assert!(pll.q_ck.is_none(), "Must set PLL P clock for Q clock to take effect!");
                    assert!(pll.r_ck.is_none(), "Must set PLL P clock for R clock to take effect!");
                    (None, None, None)
                }
            }
        }
    };
}

impl Rcc {
    pll_setup! {
    pll1_setup: (pll1vcosel, pll1rge, pll1fracen, pll1divr, divn1, divm1,
                 OUTPUTS: [
                      // unsafe as not all values are permitted: see RM0433
                     p_ck: (divp1, divp1en, 0, unsafe),
                     q_ck: (divq1, divq1en, 1),
                     r_ck: (divr1, divr1en, 2) ],
                 pll1_p)
    }
    pll_setup! {
    pll2_setup: (pll2vcosel, pll2rge, pll2fracen, pll2divr, divn2, divm2,
                 OUTPUTS: [
                     p_ck: (divp2, divp2en, 0),
                     q_ck: (divq2, divq2en, 1),
                     r_ck: (divr2, divr2en, 2)])
    }
    pll_setup! {
    pll3_setup: (pll3vcosel, pll3rge, pll3fracen, pll3divr, divn3, divm3,
                 OUTPUTS: [
                     p_ck: (divp3, divp3en, 0),
                     q_ck: (divq3, divq3en, 1),
                     r_ck: (divr3, divr3en, 2)])
    }
}

#[cfg(test)]
mod tests {
    macro_rules! dummy_method {
        ($($name:ident),+) => (
            $(
                fn $name(self) -> Self {
                    self
                }
            )+
        )
    }

    // Mock PLL CFGR
    struct WPllCfgr {}
    impl WPllCfgr {
        dummy_method! { vcosel, medium_vco, wide_vco }
        dummy_method! { pllrge, range1, range2, range4, range8 }
    }
    struct MockPllCfgr {}
    impl MockPllCfgr {
        // Modify mock registers
        fn modify<F>(&self, func: F)
        where
            F: FnOnce((), WPllCfgr) -> WPllCfgr,
        {
            func((), WPllCfgr {});
        }
    }

    // Mock RCC
    struct MockRcc {
        pub pllcfgr: MockPllCfgr,
    }
    impl MockRcc {
        pub fn new() -> Self {
            MockRcc {
                pllcfgr: MockPllCfgr {},
            }
        }
    }

    #[test]
    /// Test PFD input frequency PLL and VCO output frequency
    fn vco_setup_normal() {
        let rcc = MockRcc::new();

        let pllsrc = 25_000_000; // PLL source frequency eg. 25MHz crystal
        let output = 240_000_000; // PLL output frequency (P_CK)
        println!(
            "PLL2/3 {} MHz -> {} MHz",
            pllsrc as f32 / 1e6,
            output as f32 / 1e6
        );

        // ----------------------------------------

        // VCO Setup
        println!("NORMAL");
        let (ref_x_ck, pll_x_m, pll_x_p, vco_ck_target) = vco_setup! {
            NORMAL: pllsrc, output, rcc, vcosel, pllrge
        };
        // Feedback divider. Integer only
        let pll_x_n = vco_ck_target / ref_x_ck;

        // ----------------------------------------

        // Input
        println!("M Divider {}", pll_x_m);
        let input = pllsrc as f32 / pll_x_m as f32;
        println!("==> Input {} MHz", input / 1e6);
        println!();
        assert!((input > 1e6) && (input < 2e6));

        println!("VCO CK Target {} MHz", vco_ck_target as f32 / 1e6);
        println!("VCO CK Achieved {} MHz", pll_x_n as f32 * input / 1e6);

        // Output
        println!("P Divider {}", pll_x_p);
        let output = pll_x_n as f32 * input / pll_x_p as f32;
        println!("==> Output {} MHz", output / 1e6);
        println!();

        let error = output - 240e6;
        assert!(f32::abs(error) < 2.4e6); // < ±1% error
    }

    #[test]
    /// Test PFD input frequency PLL and VCO output frequency
    fn vco_setup_iterative() {
        let rcc = MockRcc::new();

        let pllsrc = 25_000_000; // PLL source frequency eg. 25MHz crystal
        let output = 240_000_000; // PLL output frequency (P_CK)
        println!(
            "PLL2/3 {} MHz -> {} MHz",
            pllsrc as f32 / 1e6,
            output as f32 / 1e6
        );

        // ----------------------------------------

        // VCO Setup
        println!("ITERATIVE");
        let (ref_x_ck, pll_x_m, pll_x_p, vco_ck_target) = vco_setup! {
            ITERATIVE: pllsrc, output, rcc, vcosel, pllrge
        };
        // Feedback divider. Integer only
        let pll_x_n = vco_ck_target / ref_x_ck;

        // ----------------------------------------

        // Input
        println!("M Divider {}", pll_x_m);
        let input = pllsrc as f32 / pll_x_m as f32;
        println!("==> Input {} MHz", input / 1e6);
        println!();
        assert_eq!(input, 5e6);

        println!("VCO CK Target {} MHz", vco_ck_target as f32 / 1e6);
        println!("VCO CK Achieved {} MHz", pll_x_n as f32 * input / 1e6);

        // Output
        println!("P Divider {}", pll_x_p);
        let output = pll_x_n as f32 * input / pll_x_p as f32;
        println!("==> Output {} MHz", output / 1e6);
        println!();
        assert_eq!(output, 240e6);
    }
}
