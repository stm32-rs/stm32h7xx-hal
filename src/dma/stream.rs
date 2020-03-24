//! DMA Stream

use super::utils::DefaultTraits;
use super::DmaPeripheral;
use crate::private;
use crate::stm32::dma1::{HIFCR, HISR, LIFCR, LISR};
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
    DMA: DmaPeripheral,
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
    DMA: DmaPeripheral,
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

unsafe impl<DMA> Send for StreamIsr<DMA> where DMA: DmaPeripheral {}
unsafe impl<DMA> Sync for StreamIsr<DMA> where DMA: DmaPeripheral {}

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
// CONFIG
/////////////////////////////////////////////////////////////////////////

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct Config {
    pub transfer_complete_interrupt: TransferCompleteInterrupt,
    pub half_transfer_interrupt: HalfTransferInterrupt,
    pub transfer_error_interrupt: TransferErrorInterrupt,
    pub direct_mode_error_interrupt: DirectModeErrorInterrupt,
    pub fifo_error_interrupt: FifoErrorInterrupt,
    pub pinc: Pinc,
    pub minc: Minc,
    pub priority_level: PriorityLevel,
    pub p_size: PSize,
    pub ndt: Ndt,
    pub pa: Pa,
    pub m0a: M0a,
}

/////////////////////////////////////////////////////////////////////////
// # TRANSFER DIRECTION CONFIG
/////////////////////////////////////////////////////////////////////////

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum TransferDirectionConf {
    M2M(FifoConf),
    NotM2M(NotM2MConf),
}

/////////////////////////////////////////////////////////////////////////
// # NOT-M2M CONFIG
/////////////////////////////////////////////////////////////////////////

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct NotM2MConf {
    pub transfer_dir: TransferDirectionNotM2M,
    pub transfer_mode: TransferModeConf,
    pub flow: FlowControllerConf,
}

/////////////////////////////////////////////////////////////////////////
// # TRANSFER DIRECTION NOT M2M
/////////////////////////////////////////////////////////////////////////

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum TransferDirectionNotM2M {
    P2M,
    M2P,
}

/////////////////////////////////////////////////////////////////////////
// # TRANSFER MODE CONFIG
/////////////////////////////////////////////////////////////////////////

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum TransferModeConf {
    Direct,
    Fifo(FifoConf),
}

/////////////////////////////////////////////////////////////////////////
// # FLOW CONTROLLER CONFIG
/////////////////////////////////////////////////////////////////////////

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum FlowControllerConf {
    Dma(CircularModeConf),
    Peripheral,
}

/////////////////////////////////////////////////////////////////////////
// # FIFO CONFIG
/////////////////////////////////////////////////////////////////////////

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct FifoConf {
    pub fifo_threshold: FifoThreshold,
    pub p_burst: PBurstConf,
    pub m_burst: MBurst,
    pub m_size: MSize,
}

/////////////////////////////////////////////////////////////////////////
// # PBURST CONFIG
/////////////////////////////////////////////////////////////////////////

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum PBurstConf {
    Single(Pincos),
    Incr4,
    Incr8,
    Incr16,
}

/////////////////////////////////////////////////////////////////////////
// # CIRCULAR CONFIG
/////////////////////////////////////////////////////////////////////////

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum CircularModeConf {
    Disabled,
    Enabled(BufferModeConf),
}

/////////////////////////////////////////////////////////////////////////
// # BUFFER MODE CONFIG
/////////////////////////////////////////////////////////////////////////

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum BufferModeConf {
    Regular,
    DoubleBuffer(DoubleBufferConf),
}

/////////////////////////////////////////////////////////////////////////
// # DOUBLE BUFFER CONFIG
/////////////////////////////////////////////////////////////////////////

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct DoubleBufferConf {
    pub current_target: CurrentTarget,
    pub m1a: M1a,
}

/////////////////////////////////////////////////////////////////////////
// CONFIG BUILDER
/////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////
// # CONFIG BUILDER
/////////////////////////////////////////////////////////////////////////

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct ConfigBuilder<
    C_TransferDir,
    C_TransferMode,
    C_FlowController,
    C_CircularMode,
    C_BufferMode,
> where
    C_TransferDir: ITransferDirection,
    C_TransferMode: ITransferMode,
    C_FlowController: IFlowController,
    C_CircularMode: ICircularMode,
    C_BufferMode: IBufferMode,
{
    tc_intrpt: Option<TransferCompleteInterrupt>,
    ht_intrpt: Option<HalfTransferInterrupt>,
    te_intrpt: Option<TransferErrorInterrupt>,
    dme_intrpt: Option<DirectModeErrorInterrupt>,
    fe_intrpt: Option<FifoErrorInterrupt>,
    pinc: Option<Pinc>,
    minc: Option<Minc>,
    priority: Option<PriorityLevel>,
    p_size: Option<PSize>,
    ndt: Option<Ndt>,
    pa: Option<Pa>,
    m0a: Option<M0a>,
    transfer_mode: C_TransferMode,
    buffer_mode: C_BufferMode,
    _phantom: PhantomData<(C_TransferDir, C_FlowController, C_CircularMode)>,
}

/////////////////////////////////////////////////////////////////////////
// ## GENERIC IMPLEMENTATION
/////////////////////////////////////////////////////////////////////////

impl
    ConfigBuilder<
        NotConfigured,
        NotConfigured,
        NotConfigured,
        NotConfigured,
        NotConfigured,
    >
{
    pub fn new() -> Self {
        Self {
            tc_intrpt: None,
            ht_intrpt: None,
            te_intrpt: None,
            dme_intrpt: None,
            fe_intrpt: None,
            pinc: None,
            minc: None,
            priority: None,
            p_size: None,
            ndt: None,
            pa: None,
            m0a: None,
            transfer_mode: NotConfigured,
            buffer_mode: NotConfigured,
            _phantom: PhantomData,
        }
    }
}

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
    C_TransferDir: ITransferDirection,
    C_TransferMode: ITransferMode,
    C_FlowController: IFlowController,
    C_CircularMode: ICircularMode,
    C_BufferMode: IBufferMode,
{
    pub fn transfer_complete_interrupt(
        mut self,
        tc_intrpt: TransferCompleteInterrupt,
    ) -> Self {
        self.tc_intrpt = Some(tc_intrpt);

        self
    }

    pub fn half_transfer_interrupt(
        mut self,
        ht_intrpt: HalfTransferInterrupt,
    ) -> Self {
        self.ht_intrpt = Some(ht_intrpt);

        self
    }

    pub fn transfer_error_interrupt(
        mut self,
        te_intrpt: TransferErrorInterrupt,
    ) -> Self {
        self.te_intrpt = Some(te_intrpt);

        self
    }

    pub fn direct_mode_error_interrupt(
        mut self,
        dme_intrpt: DirectModeErrorInterrupt,
    ) -> Self {
        self.dme_intrpt = Some(dme_intrpt);

        self
    }

    pub fn fifo_error_interrupt(
        mut self,
        fe_intrpt: FifoErrorInterrupt,
    ) -> Self {
        self.fe_intrpt = Some(fe_intrpt);

        self
    }

    pub fn pinc(mut self, pinc: Pinc) -> Self {
        self.pinc = Some(pinc);

        self
    }

    pub fn minc(mut self, minc: Minc) -> Self {
        self.minc = Some(minc);

        self
    }

    pub fn priority_level(mut self, priority: PriorityLevel) -> Self {
        self.priority = Some(priority);

        self
    }

    pub fn p_size(mut self, p_size: PSize) -> Self {
        self.p_size = Some(p_size);

        self
    }

    pub fn ndt(mut self, ndt: Ndt) -> Self {
        self.ndt = Some(ndt);

        self
    }

    pub fn pa(mut self, pa: Pa) -> Self {
        self.pa = Some(pa);

        self
    }

    pub fn m0a(mut self, m0a: M0a) -> Self {
        self.m0a = Some(m0a);

        self
    }

    pub fn transfer_dir_p2m(
        self,
    ) -> ConfigBuilder<
        P2M,
        C_TransferMode,
        C_FlowController,
        C_CircularMode,
        C_BufferMode,
    > {
        self.transmute()
    }

    pub fn transfer_dir_m2p(
        self,
    ) -> ConfigBuilder<
        M2P,
        C_TransferMode,
        C_FlowController,
        C_CircularMode,
        C_BufferMode,
    > {
        self.transmute()
    }

    pub fn transfer_mode_fifo(
        self,
    ) -> ConfigBuilder<
        C_TransferDir,
        Fifo,
        C_FlowController,
        C_CircularMode,
        C_BufferMode,
    > {
        self.transmute_transfer_mode(Fifo::default())
    }

    pub fn flow_controller_dma(
        self,
    ) -> ConfigBuilder<
        C_TransferDir,
        C_TransferMode,
        Dma,
        C_CircularMode,
        C_BufferMode,
    > {
        self.transmute()
    }

    pub fn buffer_mode_regular(
        self,
    ) -> ConfigBuilder<
        C_TransferDir,
        C_TransferMode,
        C_FlowController,
        C_CircularMode,
        RegularBuffer,
    > {
        self.transmute_buffer_mode(RegularBuffer)
    }

    fn transmute<NewC_TransferDir, NewC_FlowController, NewC_CircularMode>(
        self,
    ) -> ConfigBuilder<
        NewC_TransferDir,
        C_TransferMode,
        NewC_FlowController,
        NewC_CircularMode,
        C_BufferMode,
    >
    where
        NewC_TransferDir: ITransferDirection,
        NewC_FlowController: IFlowController,
        NewC_CircularMode: ICircularMode,
    {
        ConfigBuilder {
            tc_intrpt: self.tc_intrpt,
            ht_intrpt: self.ht_intrpt,
            te_intrpt: self.te_intrpt,
            dme_intrpt: self.dme_intrpt,
            fe_intrpt: self.fe_intrpt,
            pinc: self.pinc,
            minc: self.minc,
            priority: self.priority,
            p_size: self.p_size,
            ndt: self.ndt,
            pa: self.pa,
            m0a: self.m0a,
            transfer_mode: self.transfer_mode,
            buffer_mode: self.buffer_mode,
            _phantom: PhantomData,
        }
    }

    fn transmute_transfer_mode<NewC_TransferMode>(
        self,
        transfer_mode: NewC_TransferMode,
    ) -> ConfigBuilder<
        C_TransferDir,
        NewC_TransferMode,
        C_FlowController,
        C_CircularMode,
        C_BufferMode,
    >
    where
        NewC_TransferMode: ITransferMode,
    {
        ConfigBuilder {
            tc_intrpt: self.tc_intrpt,
            ht_intrpt: self.ht_intrpt,
            te_intrpt: self.te_intrpt,
            dme_intrpt: self.dme_intrpt,
            fe_intrpt: self.fe_intrpt,
            pinc: self.pinc,
            minc: self.minc,
            priority: self.priority,
            p_size: self.p_size,
            ndt: self.ndt,
            pa: self.pa,
            m0a: self.m0a,
            buffer_mode: self.buffer_mode,
            _phantom: PhantomData,
            transfer_mode,
        }
    }

    fn transmute_buffer_mode<NewC_BufferMode>(
        self,
        buffer_mode: NewC_BufferMode,
    ) -> ConfigBuilder<
        C_TransferDir,
        C_TransferMode,
        C_FlowController,
        C_CircularMode,
        NewC_BufferMode,
    >
    where
        NewC_BufferMode: IBufferMode,
    {
        ConfigBuilder {
            tc_intrpt: self.tc_intrpt,
            ht_intrpt: self.ht_intrpt,
            te_intrpt: self.te_intrpt,
            dme_intrpt: self.dme_intrpt,
            fe_intrpt: self.fe_intrpt,
            pinc: self.pinc,
            minc: self.minc,
            priority: self.priority,
            p_size: self.p_size,
            ndt: self.ndt,
            pa: self.pa,
            m0a: self.m0a,
            transfer_mode: self.transfer_mode,
            _phantom: PhantomData,
            buffer_mode,
        }
    }
}

impl<C_TransferDir, C_Fifo, C_Dma, C_NotCircular, C_RegularBuffer>
    ConfigBuilder<C_TransferDir, C_Fifo, C_Dma, C_NotCircular, C_RegularBuffer>
where
    C_TransferDir: ITransferDirection,
    C_Fifo: ITransferMode + Into<Fifo>,
    C_Dma: IFlowController + Into<Dma>,
    C_NotCircular: ICircularMode + Into<NotCircular>,
    C_RegularBuffer: IBufferMode + Into<RegularBuffer>,
{
    pub fn transfer_dir_m2m(
        self,
    ) -> ConfigBuilder<M2M, Fifo, Dma, NotCircular, RegularBuffer> {
        self.transmute()
            .transmute_transfer_mode(Fifo::default())
            .transmute_buffer_mode(RegularBuffer)
    }
}

impl<
        C_NotM2M,
        C_TransferMode,
        C_FlowController,
        C_CircularMode,
        C_BufferMode,
    >
    ConfigBuilder<
        C_NotM2M,
        C_TransferMode,
        C_FlowController,
        C_CircularMode,
        C_BufferMode,
    >
where
    C_NotM2M: NotM2M,
    C_TransferMode: ITransferMode,
    C_FlowController: IFlowController,
    C_CircularMode: ICircularMode,
    C_BufferMode: IBufferMode,
{
    pub fn transfer_mode_direct(
        self,
    ) -> ConfigBuilder<
        C_NotM2M,
        Direct,
        C_FlowController,
        C_CircularMode,
        C_BufferMode,
    > {
        self.transmute_transfer_mode(Direct)
    }

    pub fn flow_controller_peripheral(
        self,
    ) -> ConfigBuilder<
        C_NotM2M,
        C_TransferMode,
        Peripheral,
        C_CircularMode,
        C_BufferMode,
    > {
        self.transmute()
    }
}

impl<
        C_TransferDir,
        C_TransferMode,
        C_FlowController,
        C_CircularMode,
        C_RegularBuffer,
    >
    ConfigBuilder<
        C_TransferDir,
        C_TransferMode,
        C_FlowController,
        C_CircularMode,
        C_RegularBuffer,
    >
where
    C_TransferDir: ITransferDirection,
    C_TransferMode: ITransferMode,
    C_FlowController: IFlowController,
    C_CircularMode: ICircularMode,
    C_RegularBuffer: IBufferMode + Into<RegularBuffer>,
{
    pub fn circular_mode_disabled(
        self,
    ) -> ConfigBuilder<
        C_TransferDir,
        C_TransferMode,
        C_FlowController,
        NotCircular,
        RegularBuffer,
    > {
        self.transmute().transmute_buffer_mode(RegularBuffer)
    }
}

impl<C_NotM2M, C_TransferMode, C_Dma, C_CircularMode, C_BufferMode>
    ConfigBuilder<C_NotM2M, C_TransferMode, C_Dma, C_CircularMode, C_BufferMode>
where
    C_NotM2M: NotM2M,
    C_TransferMode: ITransferMode,
    C_Dma: IFlowController + Into<Dma>,
    C_CircularMode: ICircularMode,
    C_BufferMode: IBufferMode,
{
    pub fn circular_mode_enabled(
        self,
    ) -> ConfigBuilder<C_NotM2M, C_TransferMode, C_Dma, Circular, C_BufferMode>
    {
        self.transmute()
    }
}

impl<C_NotM2M, C_TransferMode, C_Dma, C_Circular, C_BufferMode>
    ConfigBuilder<C_NotM2M, C_TransferMode, C_Dma, C_Circular, C_BufferMode>
where
    C_NotM2M: NotM2M,
    C_TransferMode: ITransferMode,
    C_Dma: IFlowController + Into<Dma>,
    C_Circular: ICircularMode + Into<Circular>,
    C_BufferMode: IBufferMode,
{
    pub fn buffer_mode_double_buffer(
        self,
    ) -> ConfigBuilder<C_NotM2M, C_TransferMode, C_Dma, C_Circular, DoubleBuffer>
    {
        self.transmute_buffer_mode(DoubleBuffer::default())
    }
}

impl Default
    for ConfigBuilder<
        NotConfigured,
        NotConfigured,
        NotConfigured,
        NotConfigured,
        NotConfigured,
    >
{
    fn default() -> Self {
        Self::new()
    }
}

impl<C_TransferDir, C_FlowController, C_CircularMode, C_BufferMode>
    ConfigBuilder<
        C_TransferDir,
        Fifo,
        C_FlowController,
        C_CircularMode,
        C_BufferMode,
    >
where
    C_TransferDir: ITransferDirection,
    C_FlowController: IFlowController,
    C_CircularMode: ICircularMode,
    C_BufferMode: IBufferMode,
{
    pub fn fifo_threshold(mut self, fifo_threshold: FifoThreshold) -> Self {
        self.transfer_mode.fifo_threshold = Some(fifo_threshold);

        self
    }

    pub fn p_burst(mut self, p_burst: PBurst) -> Self {
        self.transfer_mode.p_burst = Some(p_burst);

        self
    }

    pub fn m_burst(mut self, m_burst: MBurst) -> Self {
        self.transfer_mode.m_burst = Some(m_burst);

        self
    }

    pub fn m_size(mut self, m_size: MSize) -> Self {
        self.transfer_mode.m_size = Some(m_size);

        self
    }
}

impl<C_TransferDir, C_TransferMode, C_FlowController, C_CircularMode>
    ConfigBuilder<
        C_TransferDir,
        C_TransferMode,
        C_FlowController,
        C_CircularMode,
        DoubleBuffer,
    >
where
    C_TransferDir: ITransferDirection,
    C_TransferMode: ITransferMode,
    C_FlowController: IFlowController,
    C_CircularMode: ICircularMode,
{
    pub fn current_target(mut self, current_target: CurrentTarget) -> Self {
        self.buffer_mode.current_target = Some(current_target);

        self
    }

    pub fn m1a(mut self, m1a: M1a) -> Self {
        self.buffer_mode.m1a = Some(m1a);

        self
    }
}

/////////////////////////////////////////////////////////////////////////
// # TRANSFER DIRECTION
/////////////////////////////////////////////////////////////////////////

pub trait ITransferDirection: DefaultTraits + private::Sealed {
    const TRANSFER_DIRECTION: Option<TransferDirection>;
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct P2M;
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct M2P;
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct M2M;

impl private::Sealed for P2M {}
impl private::Sealed for M2P {}
impl private::Sealed for M2M {}

impl ITransferDirection for P2M {
    const TRANSFER_DIRECTION: Option<TransferDirection> =
        Some(TransferDirection::P2M);
}

impl ITransferDirection for M2P {
    const TRANSFER_DIRECTION: Option<TransferDirection> =
        Some(TransferDirection::M2P);
}

impl ITransferDirection for M2M {
    const TRANSFER_DIRECTION: Option<TransferDirection> =
        Some(TransferDirection::M2M);
}

pub trait NotM2M: ITransferDirection {}

impl NotM2M for P2M {}
impl NotM2M for M2P {}

/////////////////////////////////////////////////////////////////////////
// # TRANSFER MODE
/////////////////////////////////////////////////////////////////////////

pub trait ITransferMode: DefaultTraits + private::Sealed {
    const TRANSFER_MODE: Option<TransferMode>;
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct Direct;

#[derive(Default, Debug, PartialEq, Eq, Clone, Copy)]
pub struct Fifo {
    fifo_threshold: Option<FifoThreshold>,
    p_burst: Option<PBurst>,
    m_burst: Option<MBurst>,
    m_size: Option<MSize>,
}

impl private::Sealed for Direct {}
impl private::Sealed for Fifo {}

impl ITransferMode for Direct {
    const TRANSFER_MODE: Option<TransferMode> = Some(TransferMode::Direct);
}
impl ITransferMode for Fifo {
    const TRANSFER_MODE: Option<TransferMode> = Some(TransferMode::Fifo);
}

/////////////////////////////////////////////////////////////////////////
// # FLOW CONTROLLER
/////////////////////////////////////////////////////////////////////////

pub trait IFlowController: DefaultTraits + private::Sealed {
    const FLOW_CONTROLLER: Option<FlowController>;
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct Dma;
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct Peripheral;

impl private::Sealed for Dma {}
impl private::Sealed for Peripheral {}

impl IFlowController for Dma {
    const FLOW_CONTROLLER: Option<FlowController> = Some(FlowController::Dma);
}
impl IFlowController for Peripheral {
    const FLOW_CONTROLLER: Option<FlowController> =
        Some(FlowController::Peripheral);
}

/////////////////////////////////////////////////////////////////////////
// # CIRCULAR MODE
/////////////////////////////////////////////////////////////////////////

pub trait ICircularMode: DefaultTraits + private::Sealed {
    const CIRCULAR_MODE: Option<CircularMode>;
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct Circular;
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct NotCircular;

impl private::Sealed for Circular {}
impl private::Sealed for NotCircular {}

impl ICircularMode for Circular {
    const CIRCULAR_MODE: Option<CircularMode> = Some(CircularMode::Enabled);
}
impl ICircularMode for NotCircular {
    const CIRCULAR_MODE: Option<CircularMode> = Some(CircularMode::Disabled);
}

/////////////////////////////////////////////////////////////////////////
// # BUFFER MODE
/////////////////////////////////////////////////////////////////////////

pub trait IBufferMode: DefaultTraits + private::Sealed {
    const BUFFER_MODE: Option<BufferMode>;
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct RegularBuffer;
#[derive(Default, Debug, PartialEq, Eq, Clone, Copy)]
pub struct DoubleBuffer {
    current_target: Option<CurrentTarget>,
    m1a: Option<M1a>,
}

impl private::Sealed for RegularBuffer {}
impl private::Sealed for DoubleBuffer {}

impl IBufferMode for RegularBuffer {
    const BUFFER_MODE: Option<BufferMode> = Some(BufferMode::Regular);
}
impl IBufferMode for DoubleBuffer {
    const BUFFER_MODE: Option<BufferMode> = Some(BufferMode::DoubleBuffer);
}

/////////////////////////////////////////////////////////////////////////
// # NOT CONFIGURED
/////////////////////////////////////////////////////////////////////////

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct NotConfigured;

impl private::Sealed for NotConfigured {}

impl ITransferDirection for NotConfigured {
    const TRANSFER_DIRECTION: Option<TransferDirection> = None;
}

impl ITransferMode for NotConfigured {
    const TRANSFER_MODE: Option<TransferMode> = None;
}

impl IFlowController for NotConfigured {
    const FLOW_CONTROLLER: Option<FlowController> = None;
}

impl ICircularMode for NotConfigured {
    const CIRCULAR_MODE: Option<CircularMode> = None;
}

impl IBufferMode for NotConfigured {
    const BUFFER_MODE: Option<BufferMode> = None;
}

impl NotM2M for NotConfigured {}

impl From<NotConfigured> for Fifo {
    fn from(_: NotConfigured) -> Self {
        Self::default()
    }
}

impl From<NotConfigured> for Dma {
    fn from(_: NotConfigured) -> Self {
        Self
    }
}

impl From<NotConfigured> for NotCircular {
    fn from(_: NotConfigured) -> Self {
        Self
    }
}

impl From<NotConfigured> for Circular {
    fn from(_: NotConfigured) -> Self {
        Self
    }
}

impl From<NotConfigured> for RegularBuffer {
    fn from(_: NotConfigured) -> Self {
        Self
    }
}
