//! DMA Request Generator

use super::super::utils::UniqueRef;
use crate::private;
use crate::stm32::dmamux1::{RGCFR, RGCR, RGSR};
use core::convert::TryInto;
use core::marker::PhantomData;

pub struct RequestGenerator<GXX, ED>
where
    GXX: GenId,
    ED: IED,
{
    /// This field *must not* be mutated using shared references
    rb: UniqueRef<'static, RGCR>,
    _phantom_data: PhantomData<(GXX, ED)>,
}

impl<GXX> RequestGenerator<GXX, Disabled>
where
    GXX: GenId,
{
    pub(in super::super) fn after_reset(rb: UniqueRef<'static, RGCR>) -> Self {
        RequestGenerator {
            rb,
            _phantom_data: PhantomData,
        }
    }
}

impl<GXX, ED> RequestGenerator<GXX, ED>
where
    GXX: GenId,
    ED: IED,
{
    pub fn id(&self) -> usize {
        GXX::ID
    }

    pub fn sig_id(&self) -> SigId {
        self.rb.read().sig_id().bits().try_into().unwrap()
    }

    pub fn set_sig_id(&mut self, sig_id: SigId) {
        unsafe {
            self.rb.modify(|_, w| w.sig_id().bits(sig_id.into()));
        }
    }

    pub fn overrun_interrupt(&self) -> TriggerOverrunInterrupt {
        self.rb.read().oie().bit().into()
    }

    pub fn set_trigger_overrun_interrupt(
        &mut self,
        overrun_intrpt: TriggerOverrunInterrupt,
    ) {
        self.rb.modify(|_, w| w.oie().bit(overrun_intrpt.into()));
    }

    pub fn gpol(&self) -> GPol {
        self.rb.read().gpol().bits().try_into().unwrap()
    }

    pub fn set_gpol(&mut self, gpol: GPol) {
        self.rb.modify(|_, w| w.gpol().bits(gpol.into()));
    }

    pub fn gnbreq(&self) -> GNbReq {
        self.rb.read().gnbreq().bits().try_into().unwrap()
    }

    fn transmute<NewED>(self) -> RequestGenerator<GXX, NewED>
    where
        NewED: IED,
    {
        RequestGenerator {
            rb: self.rb,
            _phantom_data: PhantomData,
        }
    }
}

impl<GXX> RequestGenerator<GXX, Disabled>
where
    GXX: GenId,
{
    pub fn set_gnbreq(&mut self, gnbreq: GNbReq) {
        self.rb.modify(|_, w| w.gnbreq().bits(gnbreq.into()));
    }

    pub fn enable(self) -> RequestGenerator<GXX, Enabled> {
        self.rb.modify(|_, w| w.ge().set_bit());

        self.transmute()
    }
}

impl<GXX> RequestGenerator<GXX, Enabled>
where
    GXX: GenId,
{
    pub fn disable(self) -> RequestGenerator<GXX, Disabled> {
        self.rb.modify(|_, w| w.ge().clear_bit());

        self.transmute()
    }
}

impl<GXX, ED> RequestGenerator<GXX, ED>
where
    GXX: GenId,
    ED: IED,
{
    pub fn check_isr(
        &self,
        isr: &RequestGenIsr,
    ) -> Result<(), TriggerOverrunError> {
        if self.trigger_overrun_flag(isr) {
            Err(TriggerOverrunError)
        } else {
            Ok(())
        }
    }
    pub fn trigger_overrun_flag(&self, isr: &RequestGenIsr) -> bool {
        match self.id() {
            0 => isr.rgsr.read().of0().bit_is_set(),
            1 => isr.rgsr.read().of1().bit_is_set(),
            2 => isr.rgsr.read().of2().bit_is_set(),
            3 => isr.rgsr.read().of3().bit_is_set(),
            4 => isr.rgsr.read().of4().bit_is_set(),
            5 => isr.rgsr.read().of5().bit_is_set(),
            6 => isr.rgsr.read().of6().bit_is_set(),
            7 => isr.rgsr.read().of7().bit_is_set(),
            _ => unreachable!(),
        }
    }

    pub fn clear_isr(&self, isr: &mut RequestGenIsr) {
        match self.id() {
            0 => isr.rgcfr.write(|w| w.cof0().set_bit()),
            1 => isr.rgcfr.write(|w| w.cof1().set_bit()),
            2 => isr.rgcfr.write(|w| w.cof2().set_bit()),
            3 => isr.rgcfr.write(|w| w.cof3().set_bit()),
            4 => isr.rgcfr.write(|w| w.cof4().set_bit()),
            5 => isr.rgcfr.write(|w| w.cof5().set_bit()),
            6 => isr.rgcfr.write(|w| w.cof6().set_bit()),
            7 => isr.rgcfr.write(|w| w.cof7().set_bit()),
            _ => unreachable!(),
        }
    }
}

unsafe impl<GXX, ED> Sync for RequestGenerator<GXX, ED>
where
    GXX: GenId,
    ED: IED,
{
}

type_state! {
    IED, Disabled, Enabled
}

pub trait GenId: Send + private::Sealed {
    const ID: usize;
}

macro_rules! gen_ids {
    ($($name:ident => $id:tt),*) => {
        $(
            pub struct $name {
                _private: (),
            }

            impl private::Sealed for $name {}
            impl GenId for $name {
                const ID:usize = $id;
            }
        )*
    };
}

gen_ids! {
    G0 => 0,
    G1 => 1,
    G2 => 2,
    G3 => 3,
    G4 => 4,
    G5 => 5,
    G6 => 6,
    G7 => 7
}

int_enum! {
    SigId <=> u8,
    "Signal Identification",
    DmaMux1Evt0 <=> 0b000 (D),
    DmaMux1Evt1 <=> 0b001,
    DmaMux1Evt2 <=> 0b010,
    Lptim1Out <=> 0b011,
    Lptim2Out <=> 0b100,
    Lptim3Out <=> 0b101,
    Extit0 <=> 0b110,
    Tim12Trgo <=> 0b111
}

bool_enum! {
    TriggerOverrunInterrupt, "Overrun Interrupt", Disabled (D), Enabled
}

int_enum! {
    GPol <=> u8,
    "Request Generator Trigger Polarity",
    NoEvent <=> 0b00 (D),
    RisingEdge <=> 0b01,
    FallingEdge <=> 0b10,
    RisingFallingEdge <=> 0b11
}

int_struct! {
    GNbReq, u8, 5, "Number of DMA Requests to be generated (minus 1)", 0
}

#[derive(Debug, Clone, Copy)]
pub struct TriggerOverrunError;

pub struct RequestGenIsr {
    rgsr: &'static RGSR,
    /// This field *must not* be mutated using shared references
    rgcfr: UniqueRef<'static, RGCFR>,
}

impl RequestGenIsr {
    pub(in super::super) fn new(
        rgsr: &'static RGSR,
        rgcfr: UniqueRef<'static, RGCFR>,
    ) -> Self {
        RequestGenIsr { rgsr, rgcfr }
    }
}

unsafe impl Send for RequestGenIsr {}
unsafe impl Sync for RequestGenIsr {}
