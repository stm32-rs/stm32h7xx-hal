//! HAL for Flexible memory controller (FMC)
//!
//! FMC support is implemented via the
//! [stm32-fmc](https://github.com/stm32-rs/stm32-fmc) crate.
//!
//! ## SDRAM
//!
//! An external SDRAM can be instantiated by calling the [sdram](FmcExt::sdram)
//! extension method. To avoid the pin checks, you can use
//! [sdram_unchecked](FmcExt::sdram_unchecked) instead.
//!
//! ```
//! use stm32h7xx_hal::prelude::*;
//!
//! let sdram_pins = ...; // Tuple, see stm32-fmc docs for pin ordering
//! let sdram_chip = ...; // See stm32-fmc docs
//!
//! let mut sdram = dp.FMC.sdram(
//!     sdram_pins,
//!     sdram_chip,
//!     ccdr.peripheral.FMC,
//!     &ccdr.clocks,
//! );
//! ```
//!
//! `sdram` usage is described
//! [here](https://github.com/stm32-rs/stm32-fmc#usage).
//!
//! # Examples
//!
//! - [FMC example using the IS42S32800G SDRAM](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/fmc.rs)

// From stm32_fmc
use stm32_fmc::FmcPeripheral;
use stm32_fmc::{
    AddressPinSet, PinsSdram, Sdram, SdramChip, SdramPinSet, SdramTargetBank,
};

use crate::rcc::{rec, rec::ResetEnable, CoreClocks};
use crate::stm32;
use crate::time::Hertz;

use crate::gpio::{self, Alternate};

/// Storage type for Flexible Memory Controller and its clocks
///
/// AHB access to the FMC peripheral must be enabled
pub struct FMC {
    fmc: stm32::FMC,
    fmc_ker_ck: Hertz,
}

/// Extension trait for FMC controller
pub trait FmcExt: Sized {
    fn fmc(self, prec: rec::Fmc, clocks: &CoreClocks) -> FMC;

    /// A new SDRAM memory via the Flexible Memory Controller
    fn sdram<
        BANK: SdramPinSet,
        ADDR: AddressPinSet,
        PINS: PinsSdram<BANK, ADDR>,
        CHIP: SdramChip,
    >(
        self,
        pins: PINS,
        chip: CHIP,
        prec: rec::Fmc,
        clocks: &CoreClocks,
    ) -> Sdram<FMC, CHIP> {
        let fmc = self.fmc(prec, clocks);
        Sdram::new(fmc, pins, chip)
    }

    /// A new SDRAM memory via the Flexible Memory Controller
    fn sdram_unchecked<CHIP: SdramChip, BANK: Into<SdramTargetBank>>(
        self,
        bank: BANK,
        chip: CHIP,
        prec: rec::Fmc,
        clocks: &CoreClocks,
    ) -> Sdram<FMC, CHIP> {
        let fmc = self.fmc(prec, clocks);
        Sdram::new_unchecked(fmc, bank, chip)
    }
}

impl FmcExt for stm32::FMC {
    /// New FMC instance
    fn fmc(self, prec: rec::Fmc, clocks: &CoreClocks) -> FMC {
        let clk_sel = prec.get_kernel_clk_mux();

        // Calculate kernel clock
        let fmc_ker_ck = match clk_sel {
            rec::FmcClkSel::RCC_HCLK3 => {
                Some(clocks.hclk()).expect("FMC: HCLK must be enabled")
            }
            rec::FmcClkSel::PLL1_Q => {
                clocks.pll1_q_ck().expect("FMC: PLL1_Q must be enabled")
            }
            rec::FmcClkSel::PLL2_R => {
                clocks.pll2_r_ck().expect("FMC: PLL2_R must be enabled")
            }
            rec::FmcClkSel::PER => {
                clocks.per_ck().expect("FMC: PER clock must be enabled")
            }
        };

        // Enable AHB access and reset peripheral
        prec.enable().reset();

        FMC {
            fmc: self,
            fmc_ker_ck,
        }
    }
}

impl FMC {
    /// Returns a reference to the inner peripheral
    pub fn inner(&self) -> &stm32::FMC {
        &self.fmc
    }

    /// Returns a mutable reference to the inner peripheral
    pub fn inner_mut(&mut self) -> &mut stm32::FMC {
        &mut self.fmc
    }
}

unsafe impl FmcPeripheral for FMC {
    const REGISTERS: *const () = stm32::FMC::ptr() as *const ();

    fn enable(&mut self) {
        // Already enabled as part of the contract for creating FMC
    }

    fn memory_controller_enable(&mut self) {
        // The FMCEN bit of the FMC_BCR2..4 registers is don’t
        // care. It is only enabled through the FMC_BCR1 register.
        self.fmc.bcr1.modify(|_, w| w.fmcen().set_bit());
    }

    fn source_clock_hz(&self) -> u32 {
        self.fmc_ker_ck.0
    }
}

macro_rules! pins {
    (FMC: $($pin:ident: [$( $( #[ $pmeta:meta ] )* $inst:ty),*])+) => {
        $(
            $(
                $( #[ $pmeta ] )*
                impl stm32_fmc::$pin for $inst {}
            )*
        )+
    }
}

pins! {
    FMC:
        A0: [ gpio::PF0<Alternate<12>> ]
        A1: [ gpio::PF1<Alternate<12>> ]
        A2: [ gpio::PF2<Alternate<12>> ]
        A3: [ gpio::PF3<Alternate<12>> ]
        A4: [ gpio::PF4<Alternate<12>> ]
        A5: [ gpio::PF5<Alternate<12>> ]
        A6: [ gpio::PF12<Alternate<12>> ]
        A7: [ gpio::PF13<Alternate<12>> ]
        A8: [ gpio::PF14<Alternate<12>> ]
        A9: [ gpio::PF15<Alternate<12>> ]
        A10: [ gpio::PG0<Alternate<12>> ]
        A11: [ gpio::PG1<Alternate<12>> ]
        A12: [ gpio::PG2<Alternate<12>> ]
        A13: [ gpio::PG3<Alternate<12>> ]
        A14: [ gpio::PG4<Alternate<12>> ]
        A15: [ gpio::PG5<Alternate<12>> ]
        A16: [ gpio::PD11<Alternate<12>> ]
        A17: [ gpio::PD12<Alternate<12>> ]
        A18: [ gpio::PD13<Alternate<12>> ]
        A19: [ gpio::PE3<Alternate<12>> ]
        A20: [ gpio::PE4<Alternate<12>> ]
        A21: [ gpio::PE5<Alternate<12>> ]
        A22: [ gpio::PE6<Alternate<12>> ]
        A23: [ gpio::PE2<Alternate<12>> ]
        A24: [ gpio::PG13<Alternate<12>> ]
        A25: [ gpio::PG14<Alternate<12>> ]

        BA0: [ gpio::PG4<Alternate<12>> ]
        BA1: [ gpio::PG5<Alternate<12>> ]

        CLK: [ gpio::PD3<Alternate<12>> ]

        D0: [ gpio::PD14<Alternate<12>> ]
        D1: [ gpio::PD15<Alternate<12>> ]
        D2: [ gpio::PD0<Alternate<12>> ]
        D3: [ gpio::PD1<Alternate<12>> ]
        D4: [ gpio::PE7<Alternate<12>> ]
        D5: [ gpio::PE8<Alternate<12>> ]
        D6: [ gpio::PE9<Alternate<12>> ]
        D7: [ gpio::PE10<Alternate<12>> ]
        D8: [ gpio::PE11<Alternate<12>> ]
        D9: [ gpio::PE12<Alternate<12>> ]
        D10: [ gpio::PE13<Alternate<12>> ]
        D11: [ gpio::PE14<Alternate<12>> ]
        D12: [ gpio::PE15<Alternate<12>> ]
        D13: [ gpio::PD8<Alternate<12>> ]
        D14: [ gpio::PD9<Alternate<12>> ]
        D15: [ gpio::PD10<Alternate<12>> ]
        D16: [ gpio::PH8<Alternate<12>> ]
        D17: [ gpio::PH9<Alternate<12>> ]
        D18: [ gpio::PH10<Alternate<12>> ]
        D19: [ gpio::PH11<Alternate<12>> ]
        D20: [ gpio::PH12<Alternate<12>> ]
        D21: [ gpio::PH13<Alternate<12>> ]
        D22: [ gpio::PH14<Alternate<12>> ]
        D23: [ gpio::PH15<Alternate<12>> ]
        D24: [ #[cfg(not(feature = "rm0468"))] gpio::PI0<Alternate<12>> ]
        D25: [ #[cfg(not(feature = "rm0468"))] gpio::PI1<Alternate<12>> ]
        D26: [ #[cfg(not(feature = "rm0468"))] gpio::PI2<Alternate<12>> ]
        D27: [ #[cfg(not(feature = "rm0468"))] gpio::PI3<Alternate<12>> ]
        D28: [ #[cfg(not(feature = "rm0468"))] gpio::PI6<Alternate<12>> ]
        D29: [ #[cfg(not(feature = "rm0468"))] gpio::PI7<Alternate<12>> ]
        D30: [ #[cfg(not(feature = "rm0468"))] gpio::PI9<Alternate<12>> ]
        D31: [ #[cfg(not(feature = "rm0468"))] gpio::PI10<Alternate<12>> ]

        DA0: [ gpio::PD14<Alternate<12>> ]
        DA1: [ gpio::PD15<Alternate<12>> ]
        DA2: [ gpio::PD0<Alternate<12>> ]
        DA3: [ gpio::PD1<Alternate<12>> ]
        DA4: [ gpio::PE7<Alternate<12>> ]
        DA5: [ gpio::PE8<Alternate<12>> ]
        DA6: [ gpio::PE9<Alternate<12>> ]
        DA7: [ gpio::PE10<Alternate<12>> ]
        DA8: [ gpio::PE11<Alternate<12>> ]
        DA9: [ gpio::PE12<Alternate<12>> ]
        DA10: [ gpio::PE13<Alternate<12>> ]
        DA11: [ gpio::PE14<Alternate<12>> ]
        DA12: [ gpio::PE15<Alternate<12>> ]
        DA13: [ gpio::PD8<Alternate<12>> ]
        DA14: [ gpio::PD9<Alternate<12>> ]
        DA15: [ gpio::PD10<Alternate<12>> ]

        INT: [ gpio::PG7<Alternate<12>> ]

        NBL0: [ gpio::PE0<Alternate<12>> ]
        NBL1: [ gpio::PE1<Alternate<12>> ]
        NBL2: [ #[cfg(not(feature = "rm0468"))] gpio::PI4<Alternate<12>> ]
        NBL3: [ #[cfg(not(feature = "rm0468"))] gpio::PI5<Alternate<12>> ]

        // NAND
        NCE: [
            gpio::PC8<Alternate<9>>,
            gpio::PG9<Alternate<12>>
        ]
        NE1: [
            gpio::PC7<Alternate<9>>,
            gpio::PD7<Alternate<12>>
        ]
        NE2: [
            gpio::PC8<Alternate<9>>,
            gpio::PG9<Alternate<12>>
        ]
        NE3: [
            gpio::PG6<Alternate<12>>,
            gpio::PG10<Alternate<12>>
        ]
        NE4: [
            gpio::PG12<Alternate<12>>
        ]
        NL: [ gpio::PB7<Alternate<12>> ]
        NOE: [ gpio::PD4<Alternate<12>> ]
        NWAIT: [
            gpio::PC6<Alternate<9>>,
            gpio::PD6<Alternate<12>>
        ]
        NWE: [ gpio::PD5<Alternate<12>> ]

        // SDRAM
        SDCKE0: [
            gpio::PC3<Alternate<12>>,
            gpio::PC5<Alternate<12>>,
            gpio::PH2<Alternate<12>>
        ]
        SDCKE1: [
            gpio::PB5<Alternate<12>>,
            gpio::PH7<Alternate<12>>
        ]
        SDCLK: [ gpio::PG8<Alternate<12>> ]
        SDNCAS: [ gpio::PG15<Alternate<12>> ]
        SDNE0: [
            gpio::PC2<Alternate<12>>,
            gpio::PC4<Alternate<12>>,
            gpio::PH3<Alternate<12>>
        ]
        SDNE1: [
            gpio::PB6<Alternate<12>>,
            gpio::PH6<Alternate<12>>
        ]
        SDNRAS: [ gpio::PF11<Alternate<12>> ]
        SDNWE: [
            gpio::PA7<Alternate<12>>,
            gpio::PC0<Alternate<12>>,
            gpio::PH5<Alternate<12>>
        ]
}
