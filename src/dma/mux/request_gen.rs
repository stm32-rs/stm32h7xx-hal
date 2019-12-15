use core::fmt::Debug;

type_state! {
    ED, Disabled, Enabled
}

pub unsafe trait GenId: Debug {
    const ID: usize;
}

macro_rules! gen_ids {
    ($($name:ident => $id:tt),*) => {
        $(
            #[derive(Debug)]
            pub struct $name;

            unsafe impl GenId for $name {
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
    DmaMux1Evt0 <=> 0b000,
    DmaMux1Evt1 <=> 0b001,
    DmaMux1Evt2 <=> 0b010,
    Lptim1Out <=> 0b011,
    Lptim2Out <=> 0b100,
    Lptim3Out <=> 0b101,
    Extit0 <=> 0b110,
    Tim12Trgo <=> 0b111
}

bool_enum! {
    TriggerOverrunInterrupt, "Overrun Interrupt", Disabled, Enabled
}

int_enum! {
    GPol <=> u8,
    "Request Generator Trigger Polarity",
    NoEvent <=> 0b00,
    RisingEdge <=> 0b01,
    FallingEdge <=> 0b10,
    RisingFallingEdge <=> 0b11
}

int_struct! {
    GNbReq, u8, 5, "Number of DMA Requests to be generated (minus 1)"
}

#[derive(Debug, Clone, Copy)]
pub struct TriggerOverrunError;
