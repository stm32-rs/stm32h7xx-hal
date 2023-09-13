//! Flexible Memory Controller
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
    AddressPinSet, Nand, NandChip, PinsNand, PinsSdram, Sdram, SdramChip,
    SdramPinSet, SdramTargetBank,
};

use crate::rcc::{rec, rec::ResetEnable, CoreClocks};
use crate::stm32;
use crate::time::Hertz;

use crate::gpio::alt::fmc as alt;

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

    /// A new NAND flash memory via the Flexible Memory Controller
    fn nand<PINS: PinsNand, CHIP: NandChip>(
        self,
        pins: PINS,
        chip: CHIP,
        prec: rec::Fmc,
        clocks: &CoreClocks,
    ) -> Nand<FMC, CHIP> {
        let fmc = self.fmc(prec, clocks);
        Nand::new(fmc, pins, chip)
    }

    /// A new NAND flash memory via the Flexible Memory Controller
    ///
    /// # Safety
    ///
    /// This method does not ensure that IO pins are configured
    /// correctly. Misconfiguration may result in a bus lockup or stall when
    /// attempting to initialise the NAND device.
    ///
    /// The pins are not checked against the requirements for the NAND
    /// chip. Using this method it is possible to initialise a NAND device
    /// without sufficient pins to access the whole memory
    unsafe fn nand_unchecked<CHIP: NandChip>(
        self,
        chip: CHIP,
        prec: rec::Fmc,
        clocks: &CoreClocks,
    ) -> Nand<FMC, CHIP> {
        let fmc = self.fmc(prec, clocks);
        Nand::new_unchecked(fmc, chip)
    }
}

impl FmcExt for stm32::FMC {
    /// New FMC instance
    fn fmc(self, prec: rec::Fmc, clocks: &CoreClocks) -> FMC {
        let clk_sel = prec.get_kernel_clk_mux();

        // Calculate kernel clock
        let fmc_ker_ck = match clk_sel {
            rec::FmcClkSel::RccHclk3 => {
                Some(clocks.hclk()).expect("FMC: HCLK must be enabled")
            }
            rec::FmcClkSel::Pll1Q => {
                clocks.pll1_q_ck().expect("FMC: PLL1_Q must be enabled")
            }
            rec::FmcClkSel::Pll2R => {
                clocks.pll2_r_ck().expect("FMC: PLL2_R must be enabled")
            }
            rec::FmcClkSel::Per => {
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
        self.fmc_ker_ck.raw()
    }
}

impl stm32_fmc::A0 for alt::A0 {}
impl stm32_fmc::A1 for alt::A1 {}
impl stm32_fmc::A2 for alt::A2 {}
impl stm32_fmc::A3 for alt::A3 {}
impl stm32_fmc::A4 for alt::A4 {}
impl stm32_fmc::A5 for alt::A5 {}
impl stm32_fmc::A6 for alt::A6 {}
impl stm32_fmc::A7 for alt::A7 {}
impl stm32_fmc::A8 for alt::A8 {}
impl stm32_fmc::A9 for alt::A9 {}
impl stm32_fmc::A10 for alt::A10 {}
impl stm32_fmc::A11 for alt::A11 {}
impl stm32_fmc::A12 for alt::A12 {}
impl stm32_fmc::A13 for alt::A13 {}
impl stm32_fmc::A14 for alt::A14 {}
impl stm32_fmc::A15 for alt::A15 {}
impl stm32_fmc::A16 for alt::A16 {}
impl stm32_fmc::A17 for alt::A17 {}
impl stm32_fmc::A18 for alt::A18 {}
impl stm32_fmc::A19 for alt::A19 {}
impl stm32_fmc::A20 for alt::A20 {}
impl stm32_fmc::A21 for alt::A21 {}
impl stm32_fmc::A22 for alt::A22 {}
impl stm32_fmc::A23 for alt::A23 {}
impl stm32_fmc::A24 for alt::A24 {}
impl stm32_fmc::BA0 for alt::Ba0 {}
impl stm32_fmc::BA1 for alt::Ba1 {}
impl stm32_fmc::CLK for alt::Clk {}
impl stm32_fmc::D0 for alt::D0 {}
impl stm32_fmc::D1 for alt::D1 {}
impl stm32_fmc::D2 for alt::D2 {}
impl stm32_fmc::D3 for alt::D3 {}
impl stm32_fmc::D4 for alt::D4 {}
impl stm32_fmc::D5 for alt::D5 {}
impl stm32_fmc::D6 for alt::D6 {}
impl stm32_fmc::D7 for alt::D7 {}
impl stm32_fmc::D8 for alt::D8 {}
impl stm32_fmc::D9 for alt::D9 {}
impl stm32_fmc::D10 for alt::D10 {}
impl stm32_fmc::D11 for alt::D11 {}
impl stm32_fmc::D12 for alt::D12 {}
impl stm32_fmc::D13 for alt::D13 {}
impl stm32_fmc::D14 for alt::D14 {}
impl stm32_fmc::D15 for alt::D15 {}
impl stm32_fmc::D16 for alt::D16 {}
impl stm32_fmc::D17 for alt::D17 {}
impl stm32_fmc::D18 for alt::D18 {}
impl stm32_fmc::D19 for alt::D19 {}
impl stm32_fmc::D20 for alt::D20 {}
impl stm32_fmc::D21 for alt::D21 {}
impl stm32_fmc::D22 for alt::D22 {}
impl stm32_fmc::D23 for alt::D23 {}
impl stm32_fmc::DA0 for alt::Da0 {}
impl stm32_fmc::DA1 for alt::Da1 {}
impl stm32_fmc::DA2 for alt::Da2 {}
impl stm32_fmc::DA3 for alt::Da3 {}
impl stm32_fmc::DA4 for alt::Da4 {}
impl stm32_fmc::DA5 for alt::Da5 {}
impl stm32_fmc::DA6 for alt::Da6 {}
impl stm32_fmc::DA7 for alt::Da7 {}
impl stm32_fmc::DA8 for alt::Da8 {}
impl stm32_fmc::DA9 for alt::Da9 {}
impl stm32_fmc::DA10 for alt::Da10 {}
impl stm32_fmc::DA11 for alt::Da11 {}
impl stm32_fmc::DA12 for alt::Da12 {}
impl stm32_fmc::DA13 for alt::Da13 {}
impl stm32_fmc::DA14 for alt::Da14 {}
impl stm32_fmc::DA15 for alt::Da15 {}
impl stm32_fmc::INT for alt::Int {}
impl stm32_fmc::NBL0 for alt::Nbl0 {}
impl stm32_fmc::NBL1 for alt::Nbl1 {}
impl stm32_fmc::NCE for alt::Nce {}
impl stm32_fmc::NE1 for alt::Ne1 {}
impl stm32_fmc::NE2 for alt::Ne2 {}
impl stm32_fmc::NE3 for alt::Ne3 {}
impl stm32_fmc::NE4 for alt::Ne4 {}
impl stm32_fmc::NL for alt::Nl {}
impl stm32_fmc::NOE for alt::Noe {}
impl stm32_fmc::NWAIT for alt::Nwait {}
impl stm32_fmc::NWE for alt::Nwe {}
impl stm32_fmc::SDCKE0 for alt::Sdcke0 {}
impl stm32_fmc::SDCKE1 for alt::Sdcke1 {}
impl stm32_fmc::SDCLK for alt::Sdclk {}
impl stm32_fmc::SDNCAS for alt::Sdncas {}
impl stm32_fmc::SDNE0 for alt::Sdne0 {}
impl stm32_fmc::SDNE1 for alt::Sdne1 {}
impl stm32_fmc::SDNRAS for alt::Sdnras {}
impl stm32_fmc::SDNWE for alt::Sdnwe {}

#[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
mod advanced {
    use crate::gpio::alt::fmc as alt;
    impl stm32_fmc::D24 for alt::D24 {}
    impl stm32_fmc::D25 for alt::D25 {}
    impl stm32_fmc::D26 for alt::D26 {}
    impl stm32_fmc::D27 for alt::D27 {}
    impl stm32_fmc::D28 for alt::D28 {}
    impl stm32_fmc::D29 for alt::D29 {}
    impl stm32_fmc::D30 for alt::D30 {}
    impl stm32_fmc::D31 for alt::D31 {}
    impl stm32_fmc::NBL2 for alt::Nbl2 {}
    impl stm32_fmc::NBL3 for alt::Nbl3 {}
}
