//! DMA Stream

use super::utils::DefaultTraits;
use super::DMATrait;
use crate::private;
use crate::stm32::dma1::{HIFCR, HISR, LIFCR, LISR};
use core::fmt;
use core::marker::PhantomData;

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

/////////////////////////////////////////////////////////////////////////
// CONFIG BUILDER
/////////////////////////////////////////////////////////////////////////

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct ConfigBuilder<
    C_TransferDir,
    C_TransferMode,
    C_FlowController,
    C_CircularMode,
    C_BufferMode,
> where
    C_TransferDir: TransferDirectionTrait,
    C_TransferMode: TransferModeTrait,
    C_FlowController: FlowControllerTrait,
    C_CircularMode: CircularModeTrait,
    C_BufferMode: BufferModeTrait,
{
    transfer_mode: C_TransferMode,
    buffer_mode: C_BufferMode,
    _phantom: PhantomData<(C_TransferDir, C_FlowController, C_CircularMode)>,
}

// Generic Impl
impl<
        C_TransferDir,
        C_TransferMode,
        C_FlowController,
        C_CircularMode,
        C_BufferMode,
    >
    ConfigBuilder<
        C_TransferDir,
        C_TransferMode,
        C_FlowController,
        C_CircularMode,
        C_BufferMode,
    >
where
    C_TransferDir: TransferDirectionTrait,
    C_TransferMode: TransferModeTrait,
    C_FlowController: FlowControllerTrait,
    C_CircularMode: CircularModeTrait,
    C_BufferMode: BufferModeTrait,
{
    // IMPL
}

/////////////////////////////////////////////////////////////////////////
// # TRANSFER DIRECTION
/////////////////////////////////////////////////////////////////////////
pub trait TransferDirectionTrait: DefaultTraits + private::Sealed {
    const TRANSFER_DIRECTION: Option<TransferDirection>;
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct P2M {}
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct M2P {}
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct M2M;

impl private::Sealed for P2M {}
impl private::Sealed for M2P {}
impl private::Sealed for M2M {}

impl TransferDirectionTrait for P2M {
    const TRANSFER_DIRECTION: Option<TransferDirection> =
        Some(TransferDirection::P2M);
}

impl TransferDirectionTrait for M2P {
    const TRANSFER_DIRECTION: Option<TransferDirection> =
        Some(TransferDirection::M2P);
}

impl TransferDirectionTrait for M2M {
    const TRANSFER_DIRECTION: Option<TransferDirection> =
        Some(TransferDirection::M2M);
}

pub trait NotM2M: DefaultTraits + private::Sealed {}

impl NotM2M for P2M {}
impl NotM2M for M2P {}

/////////////////////////////////////////////////////////////////////////
// # TRANSFER MODE
/////////////////////////////////////////////////////////////////////////

pub trait TransferModeTrait: DefaultTraits + private::Sealed {
    const TRANSFER_MODE: Option<TransferMode>;
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct Direct;

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct Fifo {
    fifo_threshold: Option<FifoThreshold>,
    p_burst: Option<PBurst>,
    m_burst: Option<MBurst>,
    m_size: Option<MSize>,
}

impl private::Sealed for Direct {}
impl private::Sealed for Fifo {}

impl TransferModeTrait for Direct {
    const TRANSFER_MODE: Option<TransferMode> = Some(TransferMode::Direct);
}
impl TransferModeTrait for Fifo {
    const TRANSFER_MODE: Option<TransferMode> = Some(TransferMode::Fifo);
}

/////////////////////////////////////////////////////////////////////////
// # FLOW CONTROLLER
/////////////////////////////////////////////////////////////////////////

pub trait FlowControllerTrait: DefaultTraits + private::Sealed {
    const FLOW_CONTROLLER: Option<FlowController>;
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct Dma;
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct Peripheral;

impl private::Sealed for Dma {}
impl private::Sealed for Peripheral {}

impl FlowControllerTrait for Dma {
    const FLOW_CONTROLLER: Option<FlowController> = Some(FlowController::Dma);
}
impl FlowControllerTrait for Peripheral {
    const FLOW_CONTROLLER: Option<FlowController> =
        Some(FlowController::Peripheral);
}

/////////////////////////////////////////////////////////////////////////
// # CIRCULAR MODE
/////////////////////////////////////////////////////////////////////////

pub trait CircularModeTrait: DefaultTraits + private::Sealed {
    const CIRCULAR_MODE: Option<CircularMode>;
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct Circular;
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct NotCircular;

impl private::Sealed for Circular {}
impl private::Sealed for NotCircular {}

impl CircularModeTrait for Circular {
    const CIRCULAR_MODE: Option<CircularMode> = Some(CircularMode::Enabled);
}
impl CircularModeTrait for NotCircular {
    const CIRCULAR_MODE: Option<CircularMode> = Some(CircularMode::Disabled);
}

/////////////////////////////////////////////////////////////////////////
// # BUFFER MODE
/////////////////////////////////////////////////////////////////////////

pub trait BufferModeTrait: DefaultTraits + private::Sealed {
    const BUFFER_MODE: Option<BufferMode>;
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct RegularBuffer;
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct DoubleBuffer {
    current_target: Option<CurrentTarget>,
    m1a: Option<M1a>,
}

impl private::Sealed for RegularBuffer {}
impl private::Sealed for DoubleBuffer {}

impl BufferModeTrait for RegularBuffer {
    const BUFFER_MODE: Option<BufferMode> = Some(BufferMode::Regular);
}
impl BufferModeTrait for DoubleBuffer {
    const BUFFER_MODE: Option<BufferMode> = Some(BufferMode::DoubleBuffer);
}

/////////////////////////////////////////////////////////////////////////
// # NOT CONFIGURED
/////////////////////////////////////////////////////////////////////////

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct NotConfigured;

impl private::Sealed for NotConfigured {}

impl TransferDirectionTrait for NotConfigured {
    const TRANSFER_DIRECTION: Option<TransferDirection> = None;
}

impl TransferModeTrait for NotConfigured {
    const TRANSFER_MODE: Option<TransferMode> = None;
}

impl FlowControllerTrait for NotConfigured {
    const FLOW_CONTROLLER: Option<FlowController> = None;
}

impl CircularModeTrait for NotConfigured {
    const CIRCULAR_MODE: Option<CircularMode> = None;
}

impl BufferModeTrait for NotConfigured {
    const BUFFER_MODE: Option<BufferMode> = None;
}
