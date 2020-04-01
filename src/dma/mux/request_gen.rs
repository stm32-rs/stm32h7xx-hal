//! DMA Request Generator

use crate::private;
use crate::stm32::dmamux1::{RGCFR, RGSR};

type_state! {
    ED, Disabled, Enabled
}

pub trait GenId: Send + private::Sealed {
    const ID: usize;
}

macro_rules! gen_ids {
    ($($name:ident => $id:tt),*) => {
        $(
            pub struct $name;

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
    pub(super) rgsr: &'static RGSR,
    /// This field *must not* be mutated using shared references
    pub(super) rgcfr: &'static RGCFR,
}

impl RequestGenIsr {
    pub(in super::super) fn new(
        rgsr: &'static RGSR,
        rgcfr: &'static RGCFR,
    ) -> Self {
        RequestGenIsr { rgsr, rgcfr }
    }
}

unsafe impl Send for RequestGenIsr {}
unsafe impl Sync for RequestGenIsr {}
