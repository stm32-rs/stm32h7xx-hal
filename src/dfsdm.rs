//! # Digital filter for sigma delta modulators (DFSDM)
//!
//!

use crate::rcc::{rec, CoreClocks, ResetEnable};
use crate::stm32::DFSDM;

use crate::gpio::gpiob::{
    PB1, PB10, PB11, PB12, PB13, PB14, PB15, PB2, PB6, PB7, PB8, PB9,
};
use crate::gpio::gpioc::{PC0, PC1, PC10, PC11, PC2, PC3, PC4, PC5, PC6, PC7};
use crate::gpio::gpiod::{PD0, PD1, PD6, PD7, PD8, PD9};
use crate::gpio::gpioe::{PE10, PE11, PE12, PE13, PE4, PE5, PE7, PE8};
use crate::gpio::gpiof::{PF13, PF14};

macro_rules! pins {
    (DFSDM1: $($pin_ty:ident: [$($P:ty),*])+) => {};
}

pins! {
    DFSDM1:
        DATIN0: [
            PC1<Alternate<AF3>>
        ]
        DATIN1: [
            PB1<Alternate<AF6>>,
            PB12<Alternate<AF6>>,
            PC3<Alternate<AF3>>,
            PD6<Alternate<AF4>>
        ]
        DATIN2: [
            PB14<Alternate<AF6>>,
            PC5<Alternate<AF3>>,
            PE7<Alternate<AF3>>
        ]
        DATIN3: [
            PC7<Alternate<AF4>>,
            PD9<Alternate<AF3>>,
            PE4<Alternate<AF3>>
        ]
        DATIN4: [
            PC0<Alternate<AF6>>,
            PD7<Alternate<AF3>>,
            PE10<Alternate<AF3>>
        ]
        DATIN5: [
            PB6<Alternate<AF11>>,
            PC11<Alternate<AF3>>,
            PE12<Alternate<AF3>>
        ]
        DATIN6: [
            PD1<Alternate<AF3>>,
            PF13<Alternate<AF3>>
        ]
        DATIN7: [
            PB9<Alternate<AF3>>,
            PB10<Alternate<AF6>>
        ]
        CKIN0: [
            PC0<Alternate<AF3>>
        ]
        CKIN1: [
            PB2<Alternate<AF4>>,
            PB13<Alternate<AF6>>,
            PC2<Alternate<AF3>>,
            PD7<Alternate<AF6>>
        ]
        CKIN2: [
            PB15<Alternate<AF6>>,
            PC4<Alternate<AF3>>,
            PE8<Alternate<AF3>>
        ]
        CKIN3: [
            PC6<Alternate<AF4>>,
            PD8<Alternate<AF3>>,
            PE5<Alternate<AF3>>
        ]
        CKIN4: [
            PC1<Alternate<AF4>>,
            PD6<Alternate<AF3>>,
            PE11<Alternate<AF3>>
        ]
        CKIN5: [
            PB7<Alternate<AF11>>,
            PC10<Alternate<AF3>>,
            PE13<Alternate<AF3>>
        ]
        CKIN6: [
            PD0<Alternate<AF3>>,
            PF14<Alternate<AF3>>
        ]
        CKIN7: [
            PB8<Alternate<AF3>>,
            PB11<Alternate<AF6>>
        ]
}

pub struct Dfsdm {
    dfsdm: DFSDM,
}

/// Extension trait for DFSDM peripheral
pub trait DfsdmExt: Sized {
    type Rec: ResetEnable;

    fn dfsdm(self, prec: Self::Rec) -> Dfsdm;
}
impl DfsdmExt for DFSDM {
    type Rec = rec::Dfsdm1;

    fn dfsdm(self, prec: rec::Dfsdm1) -> Dfsdm {
        Dfsdm::dfsdm(self, prec)
    }
}

impl Dfsdm {
    pub fn dfsdm(dfsdm: DFSDM, prec: rec::Dfsdm1) -> Self {
        // Enable and reset peripheral
        prec.enable().reset();

        unsafe {
            // Set Channel Control Register 2
            dfsdm.dfsdm_chcfg0r2.modify(|_, w| w.offset().bits(2));

            // Set Channel Control Register 1
            dfsdm.dfsdm_chcfg0r1.modify(|_, w| {
                w.datmpx()
                    .bits(2) // Data input from register writes
                    .chen()
                    .set_bit()
            });

            // Set Filter Control Register
            dfsdm.dfsdm0_fcr.modify(|_, w| {
                w.ford().bits(3).fosr().bits(64 - 1).iosr().bits(0)
            });

            // Set Filter Control Register 1
            dfsdm.dfsdm0_cr1.modify(|_, w| {
                w.rch()
                    .bits(0) // Channel 0
                    .dfen()
                    .set_bit()
            });
        }

        // Global enable
        dfsdm.dfsdm_chcfg0r1.modify(|_, w| w.dfsdmen().set_bit());

        Self { dfsdm }
    }

    /// Listen for end of conversion interrupt
    pub fn listen(&mut self) {
        // Set REOC IE
        self.dfsdm.dfsdm0_cr2.modify(|_, w| w.reocie().set_bit());
    }

    /// Write input data
    pub fn write(&mut self, data: i16) {
        unsafe {
            self.dfsdm
                .dfsdm_chdatin0r
                .write(|w| w.indat0().bits(data as u16));

            self.dfsdm.dfsdm0_cr1.modify(|_, w| w.rswstart().set_bit());
        }
    }

    /// Return data
    pub fn read(&mut self) -> Result<i16, ()> {
        let isr = self.dfsdm.dfsdm0_isr.read();

        if isr.reocf().bit_is_set() {
            Ok(self.dfsdm.dfsdm0_rdatar.read().rdata().bits() as i16)
        } else {
            Err(())
        }
    }
}
