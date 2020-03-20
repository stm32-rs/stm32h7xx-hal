//! DMA Stream

use super::DMATrait;
use crate::stm32::dma1::{HIFCR, HISR, LIFCR, LISR};
use core::marker::PhantomData;
use core::fmt;
use super::utils::DefaultTraits;
use crate::private;

type_state! {
    ED, Disabled, Enabled
}

type_state! {
    IsrState, IsrCleared, IsrUncleared
}

pub trait IntoNum {
    fn into_num(self) -> usize;
}

bool_enum! {
    TransferCompleteInterrupt, "Transfer Complete Interrupt", Disabled (D), Enabled
}

bool_enum! {
    HalfTransferInterrupt, "Half Transfer Interrupt", Disabled (D), Enabled
}

bool_enum! {
    TransferErrorInterrupt, "Transfer Error Interrupt", Disabled (D), Enabled
}

bool_enum! {
    DirectModeErrorInterrupt, "Direct Mode Error Interrupt", Disabled (D), Enabled
}

bool_enum! {
    FifoErrorInterrupt, "Fifo Error Interrupt", Disabled (D), Enabled
}

bool_enum! {
    FlowController, "Flow Controller", Dma (D), Peripheral
}

int_enum! {
    TransferDirection <=> u8,
    "Transfer Direction",
    P2M <=> 0b00 (D),
    M2P <=> 0b01,
    M2M <=> 0b10
}

bool_enum! {
    CircularMode, "Circular Mode", Disabled (D), Enabled
}

bool_enum! {
    Pinc, "Peripheral Increment Mode", Fixed (D), Incremented
}

bool_enum! {
    Minc, "Memory Increment Mode", Fixed (D), Incremented
}

int_enum! {
    PSize <=> u8,
    "Peripheral Data Size",
    Byte <=> 0b00 (D),
    HalfWord <=> 0b01,
    Word <=> 0b10
}

impl IntoNum for PSize {
    fn into_num(self) -> usize {
        match self {
            PSize::Byte => 1,
            PSize::HalfWord => 2,
            PSize::Word => 4,
        }
    }
}

int_enum! {
    MSize <=> u8,
    "Memory Data Size",
    Byte <=> 0b00 (D),
    HalfWord <=> 0b01,
    Word <=> 0b10
}

impl IntoNum for MSize {
    fn into_num(self) -> usize {
        match self {
            MSize::Byte => 1,
            MSize::HalfWord => 2,
            MSize::Word => 4,
        }
    }
}

bool_enum! {
    Pincos, "Peripheral Increment Offset Size", PSize (D), Word
}

int_enum! {
    PriorityLevel <=> u8,
    "Priority Level",
    Low <=> 0b00 (D),
    Medium <=> 0b01,
    High <=> 0b10,
    VeryHigh <=> 0b11
}

bool_enum! {
    BufferMode, "Buffer Mode", Regular (D), DoubleBuffer
}

bool_enum! {
    CurrentTarget, "CurrentTarget", M0a (D), M1a
}

int_enum! {
    PBurst <=> u8,
    "Peripheral Burst",
    Single <=> 0b00 (D),
    Incr4 <=> 0b01,
    Incr8 <=> 0b10,
    Incr16 <=> 0b11
}

impl IntoNum for PBurst {
    fn into_num(self) -> usize {
        match self {
            PBurst::Single => 1,
            PBurst::Incr4 => 4,
            PBurst::Incr8 => 8,
            PBurst::Incr16 => 16,
        }
    }
}

int_enum! {
    MBurst <=> u8,
    "Memory Burst",
    Single <=> 0b00 (D),
    Incr4 <=> 0b01,
    Incr8 <=> 0b10,
    Incr16 <=> 0b11
}

impl IntoNum for MBurst {
    fn into_num(self) -> usize {
        match self {
            MBurst::Single => 1,
            MBurst::Incr4 => 4,
            MBurst::Incr8 => 8,
            MBurst::Incr16 => 16,
        }
    }
}

int_struct! {
    Ndt, u16, 0, "Number of Data Items to transfer", 0
}

int_struct! {
    Pa, u32, 0, "Peripheral Address", 0
}

int_struct! {
    M0a, u32, 0, "Memory 0 Address", 0
}

int_struct! {
    M1a, u32, 0, "Memory 1 Address", 0
}

bool_enum! {
    TransferMode, "Transfer Mode", Direct (D), Fifo
}

int_enum! {
    FifoThreshold <=> u8,
    "Fifo Threshold",
    F1_4 <=> 0b00 (D),
    F1_2 <=> 0b01,
    F3_4 <=> 0b10,
    Full <=> 0b11
}

impl IntoNum for FifoThreshold {
    /// Fifo threshold in words
    fn into_num(self) -> usize {
        match self {
            FifoThreshold::F1_4 => 1,
            FifoThreshold::F1_2 => 2,
            FifoThreshold::F3_4 => 3,
            FifoThreshold::Full => 4,
        }
    }
}

pub struct StreamIsr<DMA>
where
    DMA: DMATrait,
{
    pub(super) lisr: &'static LISR,
    pub(super) hisr: &'static HISR,
    /// This field *must not* be mutated using shared references
    pub(super) lifcr: &'static LIFCR,
    /// This field *must not* be mutated using shared references
    pub(super) hifcr: &'static HIFCR,
    _phantom_data: PhantomData<DMA>,
}

impl<DMA> StreamIsr<DMA>
where
    DMA: DMATrait,
{
    pub(super) fn new(
        lisr: &'static LISR,
        hisr: &'static HISR,
        lifcr: &'static LIFCR,
        hifcr: &'static HIFCR,
    ) -> Self {
        StreamIsr {
            lisr,
            hisr,
            lifcr,
            hifcr,
            _phantom_data: PhantomData,
        }
    }
}

unsafe impl<DMA> Send for StreamIsr<DMA> where DMA: DMATrait {}
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

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct ConfigBuilder<TransferDir>
where
    TransferDir: TransferDirectionTrait,
{
    transfer_dir: TransferDir,
}

pub trait TransferDirectionTrait: DefaultTraits + private::Sealed {
    fn transfer_direction() -> TransferDirection;
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct P2M {
    conf: NotM2MConf,
}
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct M2P {
    conf: NotM2MConf,
}
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct M2M;

impl private::Sealed for P2M {}
impl private::Sealed for M2P {}
impl private::Sealed for M2M {}

impl TransferDirectionTrait for P2M {
    fn transfer_direction() -> TransferDirection {
        TransferDirection::P2M
    }
}

impl TransferDirectionTrait for M2P {
    fn transfer_direction() -> TransferDirection {
        TransferDirection::M2P
    }
}

impl TransferDirectionTrait for M2M {
    fn transfer_direction() -> TransferDirection {
        TransferDirection::M2M
    }
}

pub trait NotM2M: DefaultTraits + private::Sealed {
    fn not_m2m_conf(self) -> NotM2MConf;
}

impl NotM2M for P2M {
    fn not_m2m_conf(self) -> NotM2MConf {
        self.conf
    }
}
impl NotM2M for M2P {
    fn not_m2m_conf(self) -> NotM2MConf {
        self.conf
    }
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct NotM2MConf {

}

impl private::Sealed for NotM2MConf {}

pub trait TransferModeTrait: private::Sealed {
    fn transfer_mode() -> TransferMode;
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct NotConfigured;

macro_rules! panic_not_configured {
    ($item:tt) => {
        panic!("{} is not configured.", $item);
    }
}

impl private::Sealed for NotConfigured {}

impl TransferDirectionTrait for NotConfigured {
    fn transfer_direction() -> TransferDirection {
        panic_not_configured!("Transfer Direction");
    }
}
