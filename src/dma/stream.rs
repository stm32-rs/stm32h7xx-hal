use super::stm32::dma1::{HIFCR, HISR, LIFCR, LISR};
use super::DMATrait;
use core::marker::PhantomData;

type_state! {
    ED, Disabled, Disabling, Enabled
}

pub unsafe trait NotDisabled: ED {}
unsafe impl NotDisabled for Disabling {}
unsafe impl NotDisabled for Enabled {}

type_state! {
    IsrState, IsrCleared, IsrUncleared
}

bool_enum! {
    TransferCompleteInterrupt, "Transfer Complete Interrupt", Disabled, Enabled
}

bool_enum! {
    HalfTransferInterrupt, "Half Transfer Interrupt", Disabled, Enabled
}

bool_enum! {
    TransferErrorInterrupt, "Transfer Error Interrupt", Disabled, Enabled
}

bool_enum! {
    DirectModeErrorInterrupt, "Direct Mode Error Interrupt", Disabled, Enabled
}

bool_enum! {
    FifoErrorInterrupt, "Fifo Error Interrupt", Disabled, Enabled
}

bool_enum! {
    FlowController, "Flow Controller", Dma, Peripheral
}

int_enum! {
    TransferDirection <=> u8,
    "Transfer Direction",
    P2M <=> 0b00,
    M2P <=> 0b01,
    M2M <=> 0b10
}

bool_enum! {
    CircularMode, "Circular Mode", Disabled, Enabled
}

bool_enum! {
    Pinc, "Peripheral Increment Mode", Fixed, Incremented
}

bool_enum! {
    Minc, "Memory Increment Mode", Fixed, Incremented
}

int_enum! {
    PSize <=> u8,
    "Peripheral Data Size",
    Byte <=> 0b00,
    HalfWord <=> 0b01,
    Word <=> 0b10
}

int_enum! {
    MSize <=> u8,
    "Memory Data Size",
    Byte <=> 0b00,
    HalfWord <=> 0b01,
    Word <=> 0b10
}

bool_enum! {
    Pincos, "Peripheral Increment Offset Size", PSize, Word
}

int_enum! {
    PriorityLevel <=> u8,
    "Priority Level",
    Low <=> 0b00,
    Medium <=> 0b01,
    High <=> 0b10,
    VeryHigh <=> 0b11
}

bool_enum! {
    BufferMode, "Buffer Mode", Regular, DoubleBuffer
}

bool_enum! {
    CurrentTarget, "CurrentTarget", M0a, M1a
}

int_enum! {
    PBurst <=> u8,
    "Peripheral Burst",
    Single <=> 0b00,
    Incr4 <=> 0b01,
    Incr8 <=> 0b10,
    Incr16 <=> 0b11
}

int_enum! {
    MBurst <=> u8,
    "Memory Burst",
    Single <=> 0b00,
    Incr4 <=> 0b01,
    Incr8 <=> 0b10,
    Incr16 <=> 0b11
}

int_struct! {
    Ndt, u16, 0, "Number of Data Items to transfer"
}

int_struct! {
    Pa, u32, 0, "Peripheral Address"
}

int_struct! {
    M0a, u32, 0, "Memory 0 Address"
}

int_struct! {
    M1a, u32, 0, "Memory 1 Address"
}

bool_enum! {
    TransferMode, "Transfer Mode", Direct, Fifo
}

int_enum! {
    FifoThreshold <=> u8,
    "Fifo Threshold",
    F1_4 <=> 0b00,
    F1_2 <=> 0b01,
    F3_4 <=> 0b10,
    Full <=> 0b11
}

pub struct StreamIsr<DMA>
where
    DMA: DMATrait,
{
    pub(super) lisr: &'static LISR,
    pub(super) hisr: &'static HISR,
    pub(super) lifcr: &'static LIFCR,
    pub(super) hifcr: &'static HIFCR,
    _phantom_data: PhantomData<DMA>,
}

unsafe impl<DMA> Sync for StreamIsr<DMA> where DMA: DMATrait {}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum Event {
    HalfTransfer,
    TransferComplete,
}

#[derive(Debug, Clone, Copy)]
pub struct Error {
    pub transfer_error: bool,
    pub direct_mode_error: bool,
    pub fifo_error: bool,
    pub event: Option<Event>,
    pub crashed: bool,
}
