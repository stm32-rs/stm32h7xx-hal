//! DMA Request Generator

use super::super::utils::UniqueRef;
use crate::private;
use crate::stm32::dmamux1::{RGCFR, RGCR, RGSR};
use core::convert::TryInto;
use core::marker::PhantomData;

#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Debug, Hash)]
pub enum RequestGenId {
    G_0,
    G_1,
    G_2,
    G_3,
    G_4,
    G_5,
    G_6,
    G_7,
}

impl From<RequestGenId> for usize {
    fn from(x: RequestGenId) -> usize {
        match x {
            RequestGenId::G_0 => 0,
            RequestGenId::G_1 => 1,
            RequestGenId::G_2 => 2,
            RequestGenId::G_3 => 3,
            RequestGenId::G_4 => 4,
            RequestGenId::G_5 => 5,
            RequestGenId::G_6 => 6,
            RequestGenId::G_7 => 7,
        }
    }
}

pub struct RequestGenerator<GXX, ED>
where
    GXX: GenId,
    ED: IED,
{
    /// This field *must not* be mutated using shared references
    rb: &'static RGCR,
    _phantom_data: PhantomData<(GXX, ED)>,
}

impl<GXX> RequestGenerator<GXX, Disabled>
where
    GXX: GenId,
{
    pub(in super::super) fn after_reset(rb: UniqueRef<'static, RGCR>) -> Self {
        RequestGenerator {
            rb: rb.into_inner(),
            _phantom_data: PhantomData,
        }
    }
}

impl<GXX, ED> RequestGenerator<GXX, ED>
where
    GXX: GenId,
    ED: IED,
{
    pub fn id(&self) -> RequestGenId {
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
        isr.trigger_overrun_flag(self.id())
    }

    pub fn clear_isr(&self, isr: &mut RequestGenIsr) {
        isr.clear_isr(self.id());
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
    const ID: RequestGenId;
}

macro_rules! gen_ids {
    ($($name:ident => $id:tt),*) => {
        $(
            pub struct $name {
                _private: (),
            }

            impl private::Sealed for $name {}
            impl GenId for $name {
                const ID: RequestGenId = RequestGenId::$id;
            }
        )*
    };
}

gen_ids! {
    G0 => G_0,
    G1 => G_1,
    G2 => G_2,
    G3 => G_3,
    G4 => G_4,
    G5 => G_5,
    G6 => G_6,
    G7 => G_7
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

#[derive(Copy, Clone, Debug)]
pub struct TriggerOverrunError;

pub struct RequestGenIsr {
    rgsr: &'static RGSR,
    /// This field *must not* be mutated using shared references
    rgcfr: &'static RGCFR,
}

impl RequestGenIsr {
    pub(in super::super) fn new(
        rgsr: &'static RGSR,
        rgcfr: UniqueRef<'static, RGCFR>,
    ) -> Self {
        RequestGenIsr {
            rgsr,
            rgcfr: rgcfr.into_inner(),
        }
    }

    pub fn trigger_overrun_flag(&self, id: RequestGenId) -> bool {
        match id {
            RequestGenId::G_0 => self.rgsr.read().of0().bit_is_set(),
            RequestGenId::G_1 => self.rgsr.read().of1().bit_is_set(),
            RequestGenId::G_2 => self.rgsr.read().of2().bit_is_set(),
            RequestGenId::G_3 => self.rgsr.read().of3().bit_is_set(),
            RequestGenId::G_4 => self.rgsr.read().of4().bit_is_set(),
            RequestGenId::G_5 => self.rgsr.read().of5().bit_is_set(),
            RequestGenId::G_6 => self.rgsr.read().of6().bit_is_set(),
            RequestGenId::G_7 => self.rgsr.read().of7().bit_is_set(),
        }
    }

    pub fn clear_isr(&mut self, id: RequestGenId) {
        match id {
            RequestGenId::G_0 => self.rgcfr.write(|w| w.cof0().set_bit()),
            RequestGenId::G_1 => self.rgcfr.write(|w| w.cof1().set_bit()),
            RequestGenId::G_2 => self.rgcfr.write(|w| w.cof2().set_bit()),
            RequestGenId::G_3 => self.rgcfr.write(|w| w.cof3().set_bit()),
            RequestGenId::G_4 => self.rgcfr.write(|w| w.cof4().set_bit()),
            RequestGenId::G_5 => self.rgcfr.write(|w| w.cof5().set_bit()),
            RequestGenId::G_6 => self.rgcfr.write(|w| w.cof6().set_bit()),
            RequestGenId::G_7 => self.rgcfr.write(|w| w.cof7().set_bit()),
        }
    }
}

unsafe impl Send for RequestGenIsr {}
unsafe impl Sync for RequestGenIsr {}
