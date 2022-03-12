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

use crate::gpio::gpioa::PA7;
use crate::gpio::gpiob::{PB5, PB6, PB7};
use crate::gpio::gpioc::{PC0, PC2, PC3, PC4, PC5, PC6, PC7, PC8};
use crate::gpio::gpiod::{
    PD0, PD1, PD10, PD11, PD12, PD13, PD14, PD15, PD3, PD4, PD5, PD6, PD7, PD8,
    PD9,
};
use crate::gpio::gpioe::{
    PE0, PE1, PE10, PE11, PE12, PE13, PE14, PE15, PE2, PE3, PE4, PE5, PE6, PE7,
    PE8, PE9,
};
use crate::gpio::gpiof::{
    PF0, PF1, PF11, PF12, PF13, PF14, PF15, PF2, PF3, PF4, PF5,
};
use crate::gpio::gpiog::{
    PG0, PG1, PG10, PG12, PG13, PG14, PG15, PG2, PG3, PG4, PG5, PG6, PG7, PG8,
    PG9,
};
use crate::gpio::gpioh::{
    PH10, PH11, PH12, PH13, PH14, PH15, PH2, PH3, PH5, PH6, PH7, PH8, PH9,
};
#[cfg(not(feature = "rm0468"))]
use crate::gpio::gpioi::{PI0, PI1, PI10, PI2, PI3, PI4, PI5, PI6, PI7, PI9};
use crate::gpio::Alternate;

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
        A0: [ PF0<Alternate<12>> ]
        A1: [ PF1<Alternate<12>> ]
        A2: [ PF2<Alternate<12>> ]
        A3: [ PF3<Alternate<12>> ]
        A4: [ PF4<Alternate<12>> ]
        A5: [ PF5<Alternate<12>> ]
        A6: [ PF12<Alternate<12>> ]
        A7: [ PF13<Alternate<12>> ]
        A8: [ PF14<Alternate<12>> ]
        A9: [ PF15<Alternate<12>> ]
        A10: [ PG0<Alternate<12>> ]
        A11: [ PG1<Alternate<12>> ]
        A12: [ PG2<Alternate<12>> ]
        A13: [ PG3<Alternate<12>> ]
        A14: [ PG4<Alternate<12>> ]
        A15: [ PG5<Alternate<12>> ]
        A16: [ PD11<Alternate<12>> ]
        A17: [ PD12<Alternate<12>> ]
        A18: [ PD13<Alternate<12>> ]
        A19: [ PE3<Alternate<12>> ]
        A20: [ PE4<Alternate<12>> ]
        A21: [ PE5<Alternate<12>> ]
        A22: [ PE6<Alternate<12>> ]
        A23: [ PE2<Alternate<12>> ]
        A24: [ PG13<Alternate<12>> ]
        A25: [ PG14<Alternate<12>> ]

        BA0: [ PG4<Alternate<12>> ]
        BA1: [ PG5<Alternate<12>> ]

        CLK: [ PD3<Alternate<12>> ]

        D0: [ PD14<Alternate<12>> ]
        D1: [ PD15<Alternate<12>> ]
        D2: [ PD0<Alternate<12>> ]
        D3: [ PD1<Alternate<12>> ]
        D4: [ PE7<Alternate<12>> ]
        D5: [ PE8<Alternate<12>> ]
        D6: [ PE9<Alternate<12>> ]
        D7: [ PE10<Alternate<12>> ]
        D8: [ PE11<Alternate<12>> ]
        D9: [ PE12<Alternate<12>> ]
        D10: [ PE13<Alternate<12>> ]
        D11: [ PE14<Alternate<12>> ]
        D12: [ PE15<Alternate<12>> ]
        D13: [ PD8<Alternate<12>> ]
        D14: [ PD9<Alternate<12>> ]
        D15: [ PD10<Alternate<12>> ]
        D16: [ PH8<Alternate<12>> ]
        D17: [ PH9<Alternate<12>> ]
        D18: [ PH10<Alternate<12>> ]
        D19: [ PH11<Alternate<12>> ]
        D20: [ PH12<Alternate<12>> ]
        D21: [ PH13<Alternate<12>> ]
        D22: [ PH14<Alternate<12>> ]
        D23: [ PH15<Alternate<12>> ]
        D24: [ #[cfg(not(feature = "rm0468"))] PI0<Alternate<12>> ]
        D25: [ #[cfg(not(feature = "rm0468"))] PI1<Alternate<12>> ]
        D26: [ #[cfg(not(feature = "rm0468"))] PI2<Alternate<12>> ]
        D27: [ #[cfg(not(feature = "rm0468"))] PI3<Alternate<12>> ]
        D28: [ #[cfg(not(feature = "rm0468"))] PI6<Alternate<12>> ]
        D29: [ #[cfg(not(feature = "rm0468"))] PI7<Alternate<12>> ]
        D30: [ #[cfg(not(feature = "rm0468"))] PI9<Alternate<12>> ]
        D31: [ #[cfg(not(feature = "rm0468"))] PI10<Alternate<12>> ]

        DA0: [ PD14<Alternate<12>> ]
        DA1: [ PD15<Alternate<12>> ]
        DA2: [ PD0<Alternate<12>> ]
        DA3: [ PD1<Alternate<12>> ]
        DA4: [ PE7<Alternate<12>> ]
        DA5: [ PE8<Alternate<12>> ]
        DA6: [ PE9<Alternate<12>> ]
        DA7: [ PE10<Alternate<12>> ]
        DA8: [ PE11<Alternate<12>> ]
        DA9: [ PE12<Alternate<12>> ]
        DA10: [ PE13<Alternate<12>> ]
        DA11: [ PE14<Alternate<12>> ]
        DA12: [ PE15<Alternate<12>> ]
        DA13: [ PD8<Alternate<12>> ]
        DA14: [ PD9<Alternate<12>> ]
        DA15: [ PD10<Alternate<12>> ]

        INT: [ PG7<Alternate<12>> ]

        NBL0: [ PE0<Alternate<12>> ]
        NBL1: [ PE1<Alternate<12>> ]
        NBL2: [ #[cfg(not(feature = "rm0468"))] PI4<Alternate<12>> ]
        NBL3: [ #[cfg(not(feature = "rm0468"))] PI5<Alternate<12>> ]

        // NAND
        NCE: [
            PC8<Alternate<9>>,
            PG9<Alternate<12>>
        ]
        NE1: [
            PC7<Alternate<9>>,
            PD7<Alternate<12>>
        ]
        NE2: [
            PC8<Alternate<9>>,
            PG9<Alternate<12>>
        ]
        NE3: [
            PG6<Alternate<12>>,
            PG10<Alternate<12>>
        ]
        NE4: [
            PG12<Alternate<12>>
        ]
        NL: [ PB7<Alternate<12>> ]
        NOE: [ PD4<Alternate<12>> ]
        NWAIT: [
            PC6<Alternate<9>>,
            PD6<Alternate<12>>
        ]
        NWE: [ PD5<Alternate<12>> ]

        // SDRAM
        SDCKE0: [
            PC3<Alternate<12>>,
            PC5<Alternate<12>>,
            PH2<Alternate<12>>
        ]
        SDCKE1: [
            PB5<Alternate<12>>,
            PH7<Alternate<12>>
        ]
        SDCLK: [ PG8<Alternate<12>> ]
        SDNCAS: [ PG15<Alternate<12>> ]
        SDNE0: [
            PC2<Alternate<12>>,
            PC4<Alternate<12>>,
            PH3<Alternate<12>>
        ]
        SDNE1: [
            PB6<Alternate<12>>,
            PH6<Alternate<12>>
        ]
        SDNRAS: [ PF11<Alternate<12>> ]
        SDNWE: [
            PA7<Alternate<12>>,
            PC0<Alternate<12>>,
            PH5<Alternate<12>>
        ]
}
