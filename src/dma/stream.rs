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
    pub transfer_direction: TransferDirectionConf,
}

impl Config {
    pub fn transfer_direction(self) -> TransferDirection {
        self.transfer_direction.into()
    }

    pub fn transfer_mode(self) -> TransferMode {
        match self.transfer_direction {
            TransferDirectionConf::NotM2M(NotM2MConf {
                transfer_mode, ..
            }) => transfer_mode.into(),
            TransferDirectionConf::M2M(_) => TransferMode::Fifo,
        }
    }

    pub fn flow_controller(self) -> FlowController {
        match self.transfer_direction {
            TransferDirectionConf::NotM2M(NotM2MConf { flow, .. }) => {
                flow.into()
            }
            TransferDirectionConf::M2M(_) => FlowController::Dma,
        }
    }

    pub fn circular_mode(self) -> CircularMode {
        match self.transfer_direction {
            TransferDirectionConf::NotM2M(NotM2MConf {
                flow: FlowControllerConf::Dma(circular_mode),
                ..
            }) => circular_mode.into(),
            _ => CircularMode::Disabled,
        }
    }

    pub fn buffer_mode(self) -> BufferMode {
        match self.transfer_direction {
            TransferDirectionConf::NotM2M(NotM2MConf {
                flow:
                    FlowControllerConf::Dma(CircularModeConf::Enabled(buffer_mode)),
                ..
            }) => buffer_mode.into(),
            _ => BufferMode::Regular,
        }
    }

    pub fn fifo_threshold(self) -> Option<FifoThreshold> {
        match self.transfer_direction {
            TransferDirectionConf::NotM2M(NotM2MConf {
                transfer_mode:
                    TransferModeConf::Fifo(FifoConf { fifo_threshold, .. }),
                ..
            }) => Some(fifo_threshold),
            TransferDirectionConf::M2M(FifoConf { fifo_threshold, .. }) => {
                Some(fifo_threshold)
            }
            _ => None,
        }
    }

    pub fn p_burst(self) -> PBurst {
        match self.transfer_direction {
            TransferDirectionConf::NotM2M(NotM2MConf {
                transfer_mode: TransferModeConf::Fifo(FifoConf { p_burst, .. }),
                ..
            }) => p_burst.into(),
            TransferDirectionConf::M2M(FifoConf { p_burst, .. }) => {
                p_burst.into()
            }
            _ => PBurst::Single,
        }
    }

    pub fn m_burst(self) -> MBurst {
        match self.transfer_direction {
            TransferDirectionConf::NotM2M(NotM2MConf {
                transfer_mode: TransferModeConf::Fifo(FifoConf { m_burst, .. }),
                ..
            }) => m_burst,
            TransferDirectionConf::M2M(FifoConf { m_burst, .. }) => m_burst,
            _ => MBurst::Single,
        }
    }

    pub fn m_size(self) -> MSize {
        match self.transfer_direction {
            TransferDirectionConf::NotM2M(NotM2MConf {
                transfer_mode: TransferModeConf::Fifo(FifoConf { m_size, .. }),
                ..
            }) => m_size,
            TransferDirectionConf::M2M(FifoConf { m_size, .. }) => m_size,
            _ => match self.p_size {
                PSize::Byte => MSize::Byte,
                PSize::HalfWord => MSize::HalfWord,
                PSize::Word => MSize::Word,
            },
        }
    }

    pub fn pincos(self) -> Pincos {
        match self.transfer_direction {
            TransferDirectionConf::NotM2M(NotM2MConf {
                transfer_mode:
                    TransferModeConf::Fifo(FifoConf {
                        p_burst: PBurstConf::Single(pincos),
                        ..
                    }),
                ..
            }) => pincos,
            TransferDirectionConf::M2M(FifoConf {
                p_burst: PBurstConf::Single(pincos),
                ..
            }) => pincos,
            _ => Pincos::PSize,
        }
    }

    pub fn current_target(self) -> CurrentTarget {
        match self.transfer_direction {
            TransferDirectionConf::NotM2M(NotM2MConf {
                flow:
                    FlowControllerConf::Dma(CircularModeConf::Enabled(
                        BufferModeConf::DoubleBuffer(DoubleBufferConf {
                            current_target,
                            ..
                        }),
                    )),
                ..
            }) => current_target,
            _ => CurrentTarget::M0a,
        }
    }

    pub fn m1a(self) -> Option<M1a> {
        match self.transfer_direction {
            TransferDirectionConf::NotM2M(NotM2MConf {
                flow:
                    FlowControllerConf::Dma(CircularModeConf::Enabled(
                        BufferModeConf::DoubleBuffer(DoubleBufferConf {
                            m1a,
                            ..
                        }),
                    )),
                ..
            }) => Some(m1a),
            _ => None,
        }
    }
}

/////////////////////////////////////////////////////////////////////////
// # TRANSFER DIRECTION CONFIG
/////////////////////////////////////////////////////////////////////////

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum TransferDirectionConf {
    M2M(FifoConf),
    NotM2M(NotM2MConf),
}

impl From<TransferDirectionConf> for TransferDirection {
    fn from(conf: TransferDirectionConf) -> TransferDirection {
        match conf {
            TransferDirectionConf::NotM2M(NotM2MConf {
                transfer_dir, ..
            }) => transfer_dir.into(),
            TransferDirectionConf::M2M(_) => TransferDirection::M2M,
        }
    }
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

impl From<TransferDirectionNotM2M> for TransferDirection {
    fn from(dir: TransferDirectionNotM2M) -> Self {
        match dir {
            TransferDirectionNotM2M::P2M => Self::P2M,
            TransferDirectionNotM2M::M2P => Self::M2P,
        }
    }
}

/////////////////////////////////////////////////////////////////////////
// # TRANSFER MODE CONFIG
/////////////////////////////////////////////////////////////////////////

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum TransferModeConf {
    Direct,
    Fifo(FifoConf),
}

impl From<TransferModeConf> for TransferMode {
    fn from(conf: TransferModeConf) -> Self {
        match conf {
            TransferModeConf::Fifo(_) => Self::Fifo,
            TransferModeConf::Direct => Self::Direct,
        }
    }
}

/////////////////////////////////////////////////////////////////////////
// # FLOW CONTROLLER CONFIG
/////////////////////////////////////////////////////////////////////////

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum FlowControllerConf {
    Dma(CircularModeConf),
    Peripheral,
}

impl From<FlowControllerConf> for FlowController {
    fn from(conf: FlowControllerConf) -> Self {
        match conf {
            FlowControllerConf::Dma(_) => Self::Dma,
            FlowControllerConf::Peripheral => Self::Peripheral,
        }
    }
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

impl From<PBurstConf> for PBurst {
    fn from(conf: PBurstConf) -> Self {
        match conf {
            PBurstConf::Single(_) => Self::Single,
            PBurstConf::Incr4 => Self::Incr4,
            PBurstConf::Incr8 => Self::Incr8,
            PBurstConf::Incr16 => Self::Incr16,
        }
    }
}

/////////////////////////////////////////////////////////////////////////
// # CIRCULAR CONFIG
/////////////////////////////////////////////////////////////////////////

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum CircularModeConf {
    Disabled,
    Enabled(BufferModeConf),
}

impl From<CircularModeConf> for CircularMode {
    fn from(conf: CircularModeConf) -> Self {
        match conf {
            CircularModeConf::Disabled => Self::Disabled,
            CircularModeConf::Enabled(_) => Self::Enabled,
        }
    }
}

/////////////////////////////////////////////////////////////////////////
// # BUFFER MODE CONFIG
/////////////////////////////////////////////////////////////////////////

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum BufferModeConf {
    Regular,
    DoubleBuffer(DoubleBufferConf),
}

impl From<BufferModeConf> for BufferMode {
    fn from(conf: BufferModeConf) -> Self {
        match conf {
            BufferModeConf::Regular => Self::Regular,
            BufferModeConf::DoubleBuffer(_) => Self::DoubleBuffer,
        }
    }
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
    C_PBurst,
> where
    C_TransferDir: MaybeNotConfigured<S_TransferDir>,
    C_TransferMode: MaybeNotConfigured<S_TransferMode>,
    C_FlowController: MaybeNotConfigured<S_FlowController>,
    C_CircularMode: MaybeNotConfigured<S_CircularMode>,
    C_BufferMode: MaybeNotConfigured<S_BufferMode>,
    C_PBurst: MaybeNotConfigured<S_PBurst>,
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
    p_burst: C_PBurst,
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
            p_burst: NotConfigured,
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
        C_PBurst,
    >
    ConfigBuilder<
        C_TransferDir,
        C_TransferMode,
        C_FlowController,
        C_CircularMode,
        C_BufferMode,
        C_PBurst,
    >
where
    C_TransferDir: MaybeNotConfigured<S_TransferDir>,
    C_TransferMode: MaybeNotConfigured<S_TransferMode>,
    C_FlowController: MaybeNotConfigured<S_FlowController>,
    C_CircularMode: MaybeNotConfigured<S_CircularMode>,
    C_BufferMode: MaybeNotConfigured<S_BufferMode>,
    C_PBurst: MaybeNotConfigured<S_PBurst>,
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

    fn transmute<NewC_TransferDir, NewC_FlowController, NewC_CircularMode>(
        self,
    ) -> ConfigBuilder<
        NewC_TransferDir,
        C_TransferMode,
        NewC_FlowController,
        NewC_CircularMode,
        C_BufferMode,
        C_PBurst,
    >
    where
        NewC_TransferDir: MaybeNotConfigured<S_TransferDir>,
        NewC_FlowController: MaybeNotConfigured<S_FlowController>,
        NewC_CircularMode: MaybeNotConfigured<S_CircularMode>,
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
            p_burst: self.p_burst,
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
        C_PBurst,
    >
    where
        NewC_TransferMode: MaybeNotConfigured<S_TransferMode>,
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
            p_burst: self.p_burst,
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
        C_PBurst,
    >
    where
        NewC_BufferMode: MaybeNotConfigured<S_BufferMode>,
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
            p_burst: self.p_burst,
            _phantom: PhantomData,
            buffer_mode,
        }
    }

    fn transmute_p_burst<NewC_PBurst>(
        self,
        p_burst: NewC_PBurst,
    ) -> ConfigBuilder<
        C_TransferDir,
        C_TransferMode,
        C_FlowController,
        C_CircularMode,
        C_BufferMode,
        NewC_PBurst,
    >
    where
        NewC_PBurst: MaybeNotConfigured<S_PBurst>,
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
            p_burst,
        }
    }
}

impl<
        C_TransferMode,
        C_FlowController,
        C_CircularMode,
        C_BufferMode,
        C_PBurst,
    >
    ConfigBuilder<
        NotConfigured,
        C_TransferMode,
        C_FlowController,
        C_CircularMode,
        C_BufferMode,
        C_PBurst,
    >
where
    C_TransferMode: MaybeNotConfigured<S_TransferMode>,
    C_FlowController: MaybeNotConfigured<S_FlowController>,
    C_CircularMode: MaybeNotConfigured<S_CircularMode>,
    C_BufferMode: MaybeNotConfigured<S_BufferMode>,
    C_PBurst: MaybeNotConfigured<S_PBurst>,
{
    pub fn transfer_dir_p2m(
        self,
    ) -> ConfigBuilder<
        P2M,
        C_TransferMode,
        C_FlowController,
        C_CircularMode,
        C_BufferMode,
        C_PBurst,
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
        C_PBurst,
    > {
        self.transmute()
    }

    pub fn transfer_dir_m2m(
        self,
    ) -> ConfigBuilder<M2M, Fifo, Dma, NotCircular, RegularBuffer, C_PBurst>
    {
        self.transmute()
            .transmute_transfer_mode(Fifo::default())
            .transmute_buffer_mode(RegularBuffer)
    }
}

impl<
        C_TransferDir,
        C_FlowController,
        C_CircularMode,
        C_BufferMode,
        C_PBurst,
    >
    ConfigBuilder<
        C_TransferDir,
        NotConfigured,
        C_FlowController,
        C_CircularMode,
        C_BufferMode,
        C_PBurst,
    >
where
    C_TransferDir: ITransferDirection,
    C_FlowController: MaybeNotConfigured<S_FlowController>,
    C_CircularMode: MaybeNotConfigured<S_CircularMode>,
    C_BufferMode: MaybeNotConfigured<S_BufferMode>,
    C_PBurst: MaybeNotConfigured<S_PBurst>,
{
    pub fn transfer_mode_fifo(
        self,
    ) -> ConfigBuilder<
        C_TransferDir,
        Fifo,
        C_FlowController,
        C_CircularMode,
        C_BufferMode,
        C_PBurst,
    > {
        self.transmute_transfer_mode(Fifo::default())
    }
}

impl<C_NotM2M, C_FlowController, C_CircularMode, C_BufferMode, C_PBurst>
    ConfigBuilder<
        C_NotM2M,
        NotConfigured,
        C_FlowController,
        C_CircularMode,
        C_BufferMode,
        C_PBurst,
    >
where
    C_NotM2M: NotM2M,
    C_FlowController: MaybeNotConfigured<S_FlowController>,
    C_CircularMode: MaybeNotConfigured<S_CircularMode>,
    C_BufferMode: MaybeNotConfigured<S_BufferMode>,
    C_PBurst: MaybeNotConfigured<S_PBurst>,
{
    pub fn transfer_mode_direct(
        self,
    ) -> ConfigBuilder<
        C_NotM2M,
        Direct,
        C_FlowController,
        C_CircularMode,
        C_BufferMode,
        Single,
    > {
        self.transmute_transfer_mode(Direct)
            .transmute_p_burst(Single::default())
    }
}

impl<C_TransferDir, C_TransferMode, C_CircularMode, C_BufferMode, C_PBurst>
    ConfigBuilder<
        C_TransferDir,
        C_TransferMode,
        NotConfigured,
        C_CircularMode,
        C_BufferMode,
        C_PBurst,
    >
where
    C_TransferDir: ITransferDirection,
    C_TransferMode: MaybeNotConfigured<S_TransferMode>,
    C_CircularMode: MaybeNotConfigured<S_CircularMode>,
    C_BufferMode: MaybeNotConfigured<S_BufferMode>,
    C_PBurst: MaybeNotConfigured<S_PBurst>,
{
    pub fn flow_controller_dma(
        self,
    ) -> ConfigBuilder<
        C_TransferDir,
        C_TransferMode,
        Dma,
        C_CircularMode,
        C_BufferMode,
        C_PBurst,
    > {
        self.transmute()
    }
}

impl<C_NotM2M, C_TransferMode, C_CircularMode, C_BufferMode, C_PBurst>
    ConfigBuilder<
        C_NotM2M,
        C_TransferMode,
        NotConfigured,
        C_CircularMode,
        C_BufferMode,
        C_PBurst,
    >
where
    C_NotM2M: NotM2M,
    C_TransferMode: MaybeNotConfigured<S_TransferMode>,
    C_CircularMode: MaybeNotConfigured<S_CircularMode>,
    C_BufferMode: MaybeNotConfigured<S_BufferMode>,
    C_PBurst: MaybeNotConfigured<S_PBurst>,
{
    pub fn flow_controller_peripheral(
        self,
    ) -> ConfigBuilder<
        C_NotM2M,
        C_TransferMode,
        Peripheral,
        NotCircular,
        RegularBuffer,
        C_PBurst,
    > {
        self.transmute().transmute_buffer_mode(RegularBuffer)
    }
}

impl<
        C_TransferDir,
        C_TransferMode,
        C_FlowController,
        C_BufferMode,
        C_PBurst,
    >
    ConfigBuilder<
        C_TransferDir,
        C_TransferMode,
        C_FlowController,
        NotConfigured,
        C_BufferMode,
        C_PBurst,
    >
where
    C_TransferDir: MaybeNotConfigured<S_TransferDir>,
    C_TransferMode: MaybeNotConfigured<S_TransferMode>,
    C_FlowController: IFlowController,
    C_BufferMode: MaybeNotConfigured<S_BufferMode>,
    C_PBurst: MaybeNotConfigured<S_PBurst>,
{
    pub fn circular_mode_disabled(
        self,
    ) -> ConfigBuilder<
        C_TransferDir,
        C_TransferMode,
        C_FlowController,
        NotCircular,
        C_BufferMode,
        C_PBurst,
    > {
        self.transmute()
    }
}

impl<C_NotM2M, C_TransferMode, C_BufferMode, C_PBurst>
    ConfigBuilder<
        C_NotM2M,
        C_TransferMode,
        Dma,
        NotConfigured,
        C_BufferMode,
        C_PBurst,
    >
where
    C_NotM2M: NotM2M,
    C_TransferMode: MaybeNotConfigured<S_TransferMode>,
    C_BufferMode: MaybeNotConfigured<S_BufferMode>,
    C_PBurst: MaybeNotConfigured<S_PBurst>,
{
    pub fn circular_mode_enabled(
        self,
    ) -> ConfigBuilder<
        C_NotM2M,
        C_TransferMode,
        Dma,
        Circular,
        C_BufferMode,
        C_PBurst,
    > {
        self.transmute()
    }
}

impl<
        C_TransferDir,
        C_TransferMode,
        C_FlowController,
        C_CircularMode,
        C_PBurst,
    >
    ConfigBuilder<
        C_TransferDir,
        C_TransferMode,
        C_FlowController,
        C_CircularMode,
        NotConfigured,
        C_PBurst,
    >
where
    C_TransferDir: MaybeNotConfigured<S_TransferDir>,
    C_TransferMode: MaybeNotConfigured<S_TransferMode>,
    C_FlowController: MaybeNotConfigured<S_FlowController>,
    C_CircularMode: ICircularMode,
    C_PBurst: MaybeNotConfigured<S_PBurst>,
{
    pub fn buffer_mode_regular(
        self,
    ) -> ConfigBuilder<
        C_TransferDir,
        C_TransferMode,
        C_FlowController,
        C_CircularMode,
        RegularBuffer,
        C_PBurst,
    > {
        self.transmute_buffer_mode(RegularBuffer)
    }
}

impl<C_TransferDir, C_TransferMode, C_FlowController, C_PBurst>
    ConfigBuilder<
        C_TransferDir,
        C_TransferMode,
        C_FlowController,
        Circular,
        NotConfigured,
        C_PBurst,
    >
where
    C_TransferDir: MaybeNotConfigured<S_TransferDir>,
    C_TransferMode: MaybeNotConfigured<S_TransferMode>,
    C_FlowController: MaybeNotConfigured<S_FlowController>,
    C_PBurst: MaybeNotConfigured<S_PBurst>,
{
    pub fn buffer_mode_double_buffer(
        self,
    ) -> ConfigBuilder<
        C_TransferDir,
        C_TransferMode,
        C_FlowController,
        Circular,
        DoubleBuffer,
        C_PBurst,
    > {
        self.transmute_buffer_mode(DoubleBuffer::default())
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
        NotConfigured,
    >
where
    C_TransferDir: MaybeNotConfigured<S_TransferDir>,
    C_TransferMode: ITransferMode,
    C_FlowController: MaybeNotConfigured<S_FlowController>,
    C_CircularMode: MaybeNotConfigured<S_CircularMode>,
    C_BufferMode: MaybeNotConfigured<S_BufferMode>,
{
}

impl<C_TransferDir, C_FlowController, C_CircularMode, C_BufferMode>
    ConfigBuilder<
        C_TransferDir,
        Fifo,
        C_FlowController,
        C_CircularMode,
        C_BufferMode,
        NotConfigured,
    >
where
    C_TransferDir: MaybeNotConfigured<S_TransferDir>,
    C_FlowController: MaybeNotConfigured<S_FlowController>,
    C_CircularMode: MaybeNotConfigured<S_CircularMode>,
    C_BufferMode: MaybeNotConfigured<S_BufferMode>,
{
    pub fn p_burst_single(
        self,
    ) -> ConfigBuilder<
        C_TransferDir,
        Fifo,
        C_FlowController,
        C_CircularMode,
        C_BufferMode,
        Single,
    > {
        self.transmute_p_burst(Single::default())
    }

    pub fn p_burst_incr4(
        self,
    ) -> ConfigBuilder<
        C_TransferDir,
        Fifo,
        C_FlowController,
        C_CircularMode,
        C_BufferMode,
        Incr4,
    > {
        self.transmute_p_burst(Incr4)
    }

    pub fn p_burst_incr8(
        self,
    ) -> ConfigBuilder<
        C_TransferDir,
        Fifo,
        C_FlowController,
        C_CircularMode,
        C_BufferMode,
        Incr8,
    > {
        self.transmute_p_burst(Incr8)
    }

    pub fn p_burst_incr16(
        self,
    ) -> ConfigBuilder<
        C_TransferDir,
        Fifo,
        C_FlowController,
        C_CircularMode,
        C_BufferMode,
        Incr16,
    > {
        self.transmute_p_burst(Incr16)
    }
}

impl Default
    for ConfigBuilder<
        NotConfigured,
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

impl<
        C_TransferDir,
        C_FlowController,
        C_CircularMode,
        C_BufferMode,
        C_PBurst,
    >
    ConfigBuilder<
        C_TransferDir,
        Fifo,
        C_FlowController,
        C_CircularMode,
        C_BufferMode,
        C_PBurst,
    >
where
    C_TransferDir: MaybeNotConfigured<S_TransferDir>,
    C_FlowController: MaybeNotConfigured<S_FlowController>,
    C_CircularMode: MaybeNotConfigured<S_CircularMode>,
    C_BufferMode: MaybeNotConfigured<S_BufferMode>,
    C_PBurst: MaybeNotConfigured<S_PBurst>,
{
    pub fn fifo_threshold(mut self, fifo_threshold: FifoThreshold) -> Self {
        self.transfer_mode.fifo_threshold = Some(fifo_threshold);

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

impl<
        C_TransferDir,
        C_TransferMode,
        C_FlowController,
        C_CircularMode,
        C_PBurst,
    >
    ConfigBuilder<
        C_TransferDir,
        C_TransferMode,
        C_FlowController,
        C_CircularMode,
        DoubleBuffer,
        C_PBurst,
    >
where
    C_TransferDir: MaybeNotConfigured<S_TransferDir>,
    C_TransferMode: MaybeNotConfigured<S_TransferMode>,
    C_FlowController: MaybeNotConfigured<S_FlowController>,
    C_CircularMode: MaybeNotConfigured<S_CircularMode>,
    C_PBurst: MaybeNotConfigured<S_PBurst>,
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
        Single,
    >
where
    C_TransferDir: MaybeNotConfigured<S_TransferDir>,
    C_TransferMode: MaybeNotConfigured<S_TransferMode>,
    C_FlowController: MaybeNotConfigured<S_FlowController>,
    C_CircularMode: MaybeNotConfigured<S_CircularMode>,
    C_BufferMode: MaybeNotConfigured<S_BufferMode>,
{
    pub fn pincos(mut self, pincos: Pincos) -> Self {
        self.p_burst.pincos = Some(pincos);

        self
    }
}

impl<
        C_TransferDir,
        C_TransferMode,
        C_FlowController,
        C_CircularMode,
        C_BufferMode,
        C_PBurst,
    >
    ConfigBuilder<
        C_TransferDir,
        C_TransferMode,
        C_FlowController,
        C_CircularMode,
        C_BufferMode,
        C_PBurst,
    >
where
    C_TransferDir: ITransferDirection,
    C_TransferMode: ITransferMode,
    C_FlowController: IFlowController,
    C_CircularMode: ICircularMode,
    C_BufferMode: IBufferMode,
    C_PBurst: IPBurst,
{
    pub fn build(self) -> Config {
        let transfer_direction_conf =
            C_TransferDir::build::<_, C_FlowController, C_CircularMode, _, _>(
                self.transfer_mode,
                self.buffer_mode,
                self.p_burst,
            );

        Config {
            transfer_complete_interrupt: self.tc_intrpt.unwrap(),
            half_transfer_interrupt: self.ht_intrpt.unwrap(),
            transfer_error_interrupt: self.te_intrpt.unwrap(),
            direct_mode_error_interrupt: self.dme_intrpt.unwrap(),
            fifo_error_interrupt: self.fe_intrpt.unwrap(),
            pinc: self.pinc.unwrap(),
            minc: self.minc.unwrap(),
            priority_level: self.priority.unwrap(),
            p_size: self.p_size.unwrap(),
            ndt: self.ndt.unwrap(),
            pa: self.pa.unwrap(),
            m0a: self.m0a.unwrap(),
            transfer_direction: transfer_direction_conf,
        }
    }
}

/////////////////////////////////////////////////////////////////////////
// # TRANSFER DIRECTION
/////////////////////////////////////////////////////////////////////////

pub trait ITransferDirection: DefaultTraits + private::Sealed {
    const TRANSFER_DIRECTION: TransferDirection;

    #[doc(hidden)]
    fn build<
        C_TransferMode,
        C_FlowController,
        C_CircularMode,
        C_BufferMode,
        C_PBurst,
    >(
        transfer_mode: C_TransferMode,
        buffer_mode: C_BufferMode,
        p_burst: C_PBurst,
    ) -> TransferDirectionConf
    where
        C_TransferMode: ITransferMode,
        C_FlowController: IFlowController,
        C_CircularMode: ICircularMode,
        C_BufferMode: IBufferMode,
        C_PBurst: IPBurst;
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
    const TRANSFER_DIRECTION: TransferDirection = TransferDirection::P2M;

    #[doc(hidden)]
    fn build<
        C_TransferMode,
        C_FlowController,
        C_CircularMode,
        C_BufferMode,
        C_PBurst,
    >(
        transfer_mode: C_TransferMode,
        buffer_mode: C_BufferMode,
        p_burst: C_PBurst,
    ) -> TransferDirectionConf
    where
        C_TransferMode: ITransferMode,
        C_FlowController: IFlowController,
        C_CircularMode: ICircularMode,
        C_BufferMode: IBufferMode,
        C_PBurst: IPBurst,
    {
        TransferDirectionConf::NotM2M(NotM2MConf {
            transfer_dir: TransferDirectionNotM2M::P2M,
            transfer_mode: transfer_mode.build(p_burst),
            flow: C_FlowController::build::<C_CircularMode, _>(buffer_mode),
        })
    }
}

impl ITransferDirection for M2P {
    const TRANSFER_DIRECTION: TransferDirection = TransferDirection::M2P;

    #[doc(hidden)]
    fn build<
        C_TransferMode,
        C_FlowController,
        C_CircularMode,
        C_BufferMode,
        C_PBurst,
    >(
        transfer_mode: C_TransferMode,
        buffer_mode: C_BufferMode,
        p_burst: C_PBurst,
    ) -> TransferDirectionConf
    where
        C_TransferMode: ITransferMode,
        C_FlowController: IFlowController,
        C_CircularMode: ICircularMode,
        C_BufferMode: IBufferMode,
        C_PBurst: IPBurst,
    {
        TransferDirectionConf::NotM2M(NotM2MConf {
            transfer_dir: TransferDirectionNotM2M::M2P,
            transfer_mode: transfer_mode.build(p_burst),
            flow: C_FlowController::build::<C_CircularMode, _>(buffer_mode),
        })
    }
}

impl ITransferDirection for M2M {
    const TRANSFER_DIRECTION: TransferDirection = TransferDirection::M2M;

    #[doc(hidden)]
    fn build<
        C_TransferMode,
        C_FlowController,
        C_CircularMode,
        C_BufferMode,
        C_PBurst,
    >(
        transfer_mode: C_TransferMode,
        _: C_BufferMode,
        p_burst: C_PBurst,
    ) -> TransferDirectionConf
    where
        C_TransferMode: ITransferMode,
        C_FlowController: IFlowController,
        C_CircularMode: ICircularMode,
        C_BufferMode: IBufferMode,
        C_PBurst: IPBurst,
    {
        TransferDirectionConf::M2M(match transfer_mode.build(p_burst) {
            TransferModeConf::Fifo(fifo_conf) => fifo_conf,
            _ => unreachable!(),
        })
    }
}

pub trait NotM2M: ITransferDirection {}

impl NotM2M for P2M {}
impl NotM2M for M2P {}

/////////////////////////////////////////////////////////////////////////
// # TRANSFER MODE
/////////////////////////////////////////////////////////////////////////

pub trait ITransferMode: DefaultTraits + private::Sealed {
    const TRANSFER_MODE: TransferMode;

    #[doc(hidden)]
    fn build<C_PBurst>(self, p_burst: C_PBurst) -> TransferModeConf
    where
        C_PBurst: IPBurst;
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct Direct;

#[derive(Default, Debug, PartialEq, Eq, Clone, Copy)]
pub struct Fifo {
    fifo_threshold: Option<FifoThreshold>,
    m_burst: Option<MBurst>,
    m_size: Option<MSize>,
}

impl private::Sealed for Direct {}
impl private::Sealed for Fifo {}

impl ITransferMode for Direct {
    const TRANSFER_MODE: TransferMode = TransferMode::Direct;

    #[doc(hidden)]
    fn build<C_PBurst>(self, _: C_PBurst) -> TransferModeConf
    where
        C_PBurst: IPBurst,
    {
        TransferModeConf::Direct
    }
}
impl ITransferMode for Fifo {
    const TRANSFER_MODE: TransferMode = TransferMode::Fifo;

    fn build<C_PBurst>(self, p_burst: C_PBurst) -> TransferModeConf
    where
        C_PBurst: IPBurst,
    {
        let conf = FifoConf {
            fifo_threshold: self.fifo_threshold.unwrap(),
            p_burst: p_burst.build(),
            m_burst: self.m_burst.unwrap(),
            m_size: self.m_size.unwrap(),
        };

        TransferModeConf::Fifo(conf)
    }
}

/////////////////////////////////////////////////////////////////////////
// # FLOW CONTROLLER
/////////////////////////////////////////////////////////////////////////

pub trait IFlowController: DefaultTraits + private::Sealed {
    const FLOW_CONTROLLER: FlowController;

    #[doc(hidden)]
    fn build<C_CircularMode, C_BufferMode>(
        buffer_mode: C_BufferMode,
    ) -> FlowControllerConf
    where
        C_CircularMode: ICircularMode,
        C_BufferMode: IBufferMode;
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct Dma;
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct Peripheral;

impl private::Sealed for Dma {}
impl private::Sealed for Peripheral {}

impl IFlowController for Dma {
    const FLOW_CONTROLLER: FlowController = FlowController::Dma;

    #[doc(hidden)]
    fn build<C_CircularMode, C_BufferMode>(
        buffer_mode: C_BufferMode,
    ) -> FlowControllerConf
    where
        C_CircularMode: ICircularMode,
        C_BufferMode: IBufferMode,
    {
        FlowControllerConf::Dma(C_CircularMode::build(buffer_mode))
    }
}
impl IFlowController for Peripheral {
    const FLOW_CONTROLLER: FlowController = FlowController::Peripheral;

    fn build<C_CircularMode, C_BufferMode>(
        _: C_BufferMode,
    ) -> FlowControllerConf
    where
        C_CircularMode: ICircularMode,
        C_BufferMode: IBufferMode,
    {
        FlowControllerConf::Peripheral
    }
}

/////////////////////////////////////////////////////////////////////////
// # CIRCULAR MODE
/////////////////////////////////////////////////////////////////////////

pub trait ICircularMode: DefaultTraits + private::Sealed {
    const CIRCULAR_MODE: CircularMode;

    #[doc(hidden)]
    fn build<C_BufferMode>(buffer_mode: C_BufferMode) -> CircularModeConf
    where
        C_BufferMode: IBufferMode;
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct Circular;
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct NotCircular;

impl private::Sealed for Circular {}
impl private::Sealed for NotCircular {}

impl ICircularMode for Circular {
    const CIRCULAR_MODE: CircularMode = CircularMode::Enabled;

    #[doc(hidden)]
    fn build<C_BufferMode>(buffer_mode: C_BufferMode) -> CircularModeConf
    where
        C_BufferMode: IBufferMode,
    {
        CircularModeConf::Enabled(buffer_mode.build())
    }
}
impl ICircularMode for NotCircular {
    const CIRCULAR_MODE: CircularMode = CircularMode::Disabled;

    fn build<C_BufferMode>(_: C_BufferMode) -> CircularModeConf
    where
        C_BufferMode: IBufferMode,
    {
        CircularModeConf::Disabled
    }
}

/////////////////////////////////////////////////////////////////////////
// # BUFFER MODE
/////////////////////////////////////////////////////////////////////////

pub trait IBufferMode: DefaultTraits + private::Sealed {
    const BUFFER_MODE: BufferMode;

    #[doc(hidden)]
    fn build(self) -> BufferModeConf;
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
    const BUFFER_MODE: BufferMode = BufferMode::Regular;

    #[doc(hidden)]
    fn build(self) -> BufferModeConf {
        BufferModeConf::Regular
    }
}
impl IBufferMode for DoubleBuffer {
    const BUFFER_MODE: BufferMode = BufferMode::DoubleBuffer;

    #[doc(hidden)]
    fn build(self) -> BufferModeConf {
        let conf = DoubleBufferConf {
            current_target: self.current_target.unwrap(),
            m1a: self.m1a.unwrap(),
        };

        BufferModeConf::DoubleBuffer(conf)
    }
}

/////////////////////////////////////////////////////////////////////////
// # PBURST
/////////////////////////////////////////////////////////////////////////

pub trait IPBurst: DefaultTraits + private::Sealed {
    const P_BURST: PBurst;

    #[doc(hidden)]
    fn build(self) -> PBurstConf;
}

#[derive(Default, Debug, PartialEq, Eq, Clone, Copy)]
pub struct Single {
    pincos: Option<Pincos>,
}
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct Incr4;
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct Incr8;
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct Incr16;

impl private::Sealed for Single {}
impl private::Sealed for Incr4 {}
impl private::Sealed for Incr8 {}
impl private::Sealed for Incr16 {}

impl IPBurst for Single {
    const P_BURST: PBurst = PBurst::Single;

    #[doc(hidden)]
    fn build(self) -> PBurstConf {
        PBurstConf::Single(self.pincos.unwrap())
    }
}

impl IPBurst for Incr4 {
    const P_BURST: PBurst = PBurst::Incr4;

    #[doc(hidden)]
    fn build(self) -> PBurstConf {
        PBurstConf::Incr4
    }
}

impl IPBurst for Incr8 {
    const P_BURST: PBurst = PBurst::Incr8;

    #[doc(hidden)]
    fn build(self) -> PBurstConf {
        PBurstConf::Incr8
    }
}

impl IPBurst for Incr16 {
    const P_BURST: PBurst = PBurst::Incr16;

    #[doc(hidden)]
    fn build(self) -> PBurstConf {
        PBurstConf::Incr16
    }
}

/////////////////////////////////////////////////////////////////////////
// # NOT CONFIGURED
/////////////////////////////////////////////////////////////////////////

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct NotConfigured;

impl private::Sealed for NotConfigured {}

pub trait MaybeNotConfigured<C>: DefaultTraits + private::Sealed {}

pub struct S_TransferDir;
pub struct S_TransferMode;
pub struct S_FlowController;
pub struct S_CircularMode;
pub struct S_BufferMode;
pub struct S_PBurst;

impl MaybeNotConfigured<S_TransferDir> for NotConfigured {}
impl MaybeNotConfigured<S_TransferMode> for NotConfigured {}
impl MaybeNotConfigured<S_FlowController> for NotConfigured {}
impl MaybeNotConfigured<S_CircularMode> for NotConfigured {}
impl MaybeNotConfigured<S_BufferMode> for NotConfigured {}
impl MaybeNotConfigured<S_PBurst> for NotConfigured {}

impl<T> MaybeNotConfigured<S_TransferDir> for T where T: ITransferDirection {}
impl<T> MaybeNotConfigured<S_TransferMode> for T where T: ITransferMode {}
impl<T> MaybeNotConfigured<S_FlowController> for T where T: IFlowController {}
impl<T> MaybeNotConfigured<S_CircularMode> for T where T: ICircularMode {}
impl<T> MaybeNotConfigured<S_BufferMode> for T where T: IBufferMode {}
impl<T> MaybeNotConfigured<S_PBurst> for T where T: IPBurst {}
