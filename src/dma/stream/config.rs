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
        self.transfer_direction.transfer_mode()
    }

    pub fn flow_controller(self) -> FlowController {
        self.transfer_direction.flow_controller()
    }

    pub fn circular_mode(self) -> CircularMode {
        self.transfer_direction.circular_mode()
    }

    pub fn buffer_mode(self) -> BufferMode {
        self.transfer_direction.buffer_mode()
    }

    pub fn fifo_threshold(self) -> Option<FifoThreshold> {
        self.transfer_direction.fifo_threshold()
    }

    pub fn p_burst(self) -> PBurst {
        self.transfer_direction.p_burst()
    }

    pub fn m_burst(self) -> MBurst {
        self.transfer_direction.m_burst()
    }

    pub fn m_size(self) -> MSize {
        self.transfer_direction.m_size(Some(self.p_size))
    }

    pub fn pinc(self) -> Pinc {
        self.transfer_direction.pinc()
    }

    pub fn pincos(self) -> Option<Pincos> {
        self.transfer_direction.pincos()
    }

    pub fn current_target(self) -> CurrentTarget {
        self.transfer_direction.current_target()
    }

    pub fn m1a(self) -> Option<M1a> {
        self.transfer_direction.m1a()
    }
}

/////////////////////////////////////////////////////////////////////////
// # TRANSFER DIRECTION CONFIG
/////////////////////////////////////////////////////////////////////////

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum TransferDirectionConf {
    P2M(NotM2MConf),
    M2P(NotM2MConf),
    M2M(FifoConf),
}

impl TransferDirectionConf {
    pub fn transfer_mode(self) -> TransferMode {
        match self {
            Self::P2M(conf) | Self::M2P(conf) => conf.transfer_mode(),
            Self::M2M(_) => TransferMode::Fifo,
        }
    }

    pub fn flow_controller(self) -> FlowController {
        match self {
            Self::P2M(conf) | Self::M2P(conf) => conf.flow_controller(),
            Self::M2M(_) => FlowController::Dma,
        }
    }

    pub fn pinc(self) -> Pinc {
        match self {
            Self::P2M(conf) | Self::M2P(conf) => conf.pinc(),
            Self::M2M(conf) => conf.pinc(),
        }
    }

    pub fn pincos(self) -> Option<Pincos> {
        match self {
            Self::P2M(conf) | Self::M2P(conf) => conf.pincos(),
            Self::M2M(conf) => conf.pincos(),
        }
    }

    pub fn fifo_threshold(self) -> Option<FifoThreshold> {
        match self {
            Self::P2M(conf) | Self::M2P(conf) => conf.fifo_threshold(),
            Self::M2M(conf) => Some(conf.fifo_threshold),
        }
    }

    pub fn p_burst(self) -> PBurst {
        match self {
            Self::P2M(conf) | Self::M2P(conf) => conf.p_burst(),
            Self::M2M(conf) => conf.p_burst(),
        }
    }

    pub fn m_burst(self) -> MBurst {
        match self {
            Self::P2M(conf) | Self::M2P(conf) => conf.m_burst(),
            Self::M2M(conf) => conf.m_burst,
        }
    }

    pub fn m_size(self, p_size: Option<PSize>) -> MSize {
        match self {
            Self::P2M(conf) | Self::M2P(conf) => conf.m_size(p_size),
            Self::M2M(conf) => conf.m_size,
        }
    }

    pub fn circular_mode(self) -> CircularMode {
        match self {
            Self::P2M(conf) | Self::M2P(conf) => conf.circular_mode(),
            Self::M2M(_) => CircularMode::Disabled,
        }
    }

    pub fn buffer_mode(self) -> BufferMode {
        match self {
            Self::P2M(conf) | Self::M2P(conf) => conf.buffer_mode(),
            Self::M2M(_) => CircularModeConf::Disabled.buffer_mode(),
        }
    }

    pub fn current_target(self) -> CurrentTarget {
        match self {
            Self::P2M(conf) | Self::M2P(conf) => conf.current_target(),
            Self::M2M(_) => CircularModeConf::Disabled.current_target(),
        }
    }

    pub fn m1a(self) -> Option<M1a> {
        match self {
            Self::P2M(conf) | Self::M2P(conf) => conf.m1a(),
            Self::M2M(_) => CircularModeConf::Disabled.m1a(),
        }
    }
}

impl From<TransferDirectionConf> for TransferDirection {
    fn from(conf: TransferDirectionConf) -> TransferDirection {
        match conf {
            TransferDirectionConf::P2M(_) => TransferDirection::P2M,
            TransferDirectionConf::M2P(_) => TransferDirection::M2P,
            TransferDirectionConf::M2M(_) => TransferDirection::M2M,
        }
    }
}

/////////////////////////////////////////////////////////////////////////
// # NOT-M2M CONFIG
/////////////////////////////////////////////////////////////////////////

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct NotM2MConf {
    pub transfer_mode: TransferModeConf,
    pub flow_controller: FlowControllerConf,
}

impl NotM2MConf {
    pub fn transfer_mode(self) -> TransferMode {
        self.transfer_mode.into()
    }

    pub fn flow_controller(self) -> FlowController {
        self.flow_controller.into()
    }

    pub fn pinc(self) -> Pinc {
        self.transfer_mode.pinc()
    }

    pub fn pincos(self) -> Option<Pincos> {
        self.transfer_mode.pincos()
    }

    pub fn fifo_threshold(self) -> Option<FifoThreshold> {
        self.transfer_mode.fifo_threshold()
    }

    pub fn p_burst(self) -> PBurst {
        self.transfer_mode.p_burst()
    }

    pub fn m_burst(self) -> MBurst {
        self.transfer_mode.m_burst()
    }

    pub fn m_size(self, p_size: Option<PSize>) -> MSize {
        self.transfer_mode.m_size(p_size)
    }

    pub fn circular_mode(self) -> CircularMode {
        self.flow_controller.circular_mode()
    }

    pub fn buffer_mode(self) -> BufferMode {
        self.flow_controller.buffer_mode()
    }

    pub fn current_target(self) -> CurrentTarget {
        self.flow_controller.current_target()
    }

    pub fn m1a(self) -> Option<M1a> {
        self.flow_controller.m1a()
    }
}

/////////////////////////////////////////////////////////////////////////
// # TRANSFER MODE CONFIG
/////////////////////////////////////////////////////////////////////////

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum TransferModeConf {
    Direct(DirectConf),
    Fifo(FifoConf),
}

impl TransferModeConf {
    pub fn pinc(self) -> Pinc {
        match self {
            Self::Fifo(conf) => conf.pinc(),
            Self::Direct(conf) => conf.pinc,
        }
    }

    pub fn pincos(self) -> Option<Pincos> {
        match self {
            TransferModeConf::Fifo(conf) => conf.pincos(),
            TransferModeConf::Direct(conf) => match conf.pinc {
                Pinc::Fixed => PincConf::Fixed.pincos(),
                Pinc::Incremented => Some(Pincos::PSize),
            },
        }
    }

    pub fn fifo_threshold(self) -> Option<FifoThreshold> {
        match self {
            Self::Direct(_) => None,
            Self::Fifo(conf) => Some(conf.fifo_threshold),
        }
    }

    pub fn p_burst(self) -> PBurst {
        match self {
            Self::Direct(_) => PBurst::Single,
            Self::Fifo(conf) => conf.p_burst(),
        }
    }

    pub fn m_burst(self) -> MBurst {
        match self {
            Self::Direct(_) => MBurst::Single,
            Self::Fifo(conf) => conf.m_burst,
        }
    }

    pub fn m_size(self, p_size: Option<PSize>) -> MSize {
        match self {
            Self::Direct(_) => match p_size.unwrap() {
                PSize::Byte => MSize::Byte,
                PSize::HalfWord => MSize::HalfWord,
                PSize::Word => MSize::Word,
            },
            Self::Fifo(conf) => conf.m_size,
        }
    }
}

impl From<TransferModeConf> for TransferMode {
    fn from(conf: TransferModeConf) -> Self {
        match conf {
            TransferModeConf::Fifo(_) => Self::Fifo,
            TransferModeConf::Direct(_) => Self::Direct,
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

impl FlowControllerConf {
    pub fn circular_mode(self) -> CircularMode {
        match self {
            Self::Dma(circ) => circ.into(),
            Self::Peripheral => CircularMode::Disabled,
        }
    }

    pub fn buffer_mode(self) -> BufferMode {
        match self {
            Self::Dma(circ) => circ.buffer_mode(),
            Self::Peripheral => CircularModeConf::Disabled.buffer_mode(),
        }
    }

    pub fn current_target(self) -> CurrentTarget {
        match self {
            Self::Dma(circ) => circ.current_target(),
            Self::Peripheral => CircularModeConf::Disabled.current_target(),
        }
    }

    pub fn m1a(self) -> Option<M1a> {
        match self {
            Self::Dma(circ) => circ.m1a(),
            Self::Peripheral => CircularModeConf::Disabled.m1a(),
        }
    }
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
// # PINC CONFIG
/////////////////////////////////////////////////////////////////////////

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum PincConf {
    Fixed,
    Incremented(Pincos),
}

impl PincConf {
    pub fn pincos(self) -> Option<Pincos> {
        match self {
            Self::Fixed => None,
            Self::Incremented(pincos) => Some(pincos),
        }
    }
}

impl From<PincConf> for Pinc {
    fn from(x: PincConf) -> Self {
        match x {
            PincConf::Fixed => Self::Fixed,
            PincConf::Incremented(_) => Self::Incremented,
        }
    }
}

/////////////////////////////////////////////////////////////////////////
// # DIRECT CONFIG
/////////////////////////////////////////////////////////////////////////

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct DirectConf {
    pub pinc: Pinc,
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

impl FifoConf {
    pub fn pinc(self) -> Pinc {
        self.p_burst.pinc()
    }

    pub fn pincos(self) -> Option<Pincos> {
        self.p_burst.pincos()
    }

    pub fn p_burst(self) -> PBurst {
        self.p_burst.into()
    }
}

/////////////////////////////////////////////////////////////////////////
// # PBURST CONFIG
/////////////////////////////////////////////////////////////////////////

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum PBurstConf {
    Single(PincConf),
    Incr4(Pinc),
    Incr8(Pinc),
    Incr16(Pinc),
}

impl PBurstConf {
    pub fn pinc(self) -> Pinc {
        match self {
            Self::Single(pinc_conf) => pinc_conf.into(),
            Self::Incr4(pinc) | Self::Incr8(pinc) | Self::Incr16(pinc) => pinc,
        }
    }

    pub fn pincos(self) -> Option<Pincos> {
        match self {
            Self::Single(pinc_conf) => pinc_conf.pincos(),
            Self::Incr4(pinc) | Self::Incr8(pinc) | Self::Incr16(pinc) => {
                match pinc {
                    Pinc::Fixed => PincConf::Fixed.pincos(),
                    Pinc::Incremented => Some(Pincos::PSize),
                }
            }
        }
    }
}

impl From<PBurstConf> for PBurst {
    fn from(conf: PBurstConf) -> Self {
        match conf {
            PBurstConf::Single(_) => Self::Single,
            PBurstConf::Incr4(_) => Self::Incr4,
            PBurstConf::Incr8(_) => Self::Incr8,
            PBurstConf::Incr16(_) => Self::Incr16,
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

impl CircularModeConf {
    pub fn buffer_mode(self) -> BufferMode {
        match self {
            Self::Disabled => BufferMode::Regular,
            Self::Enabled(conf) => conf.into(),
        }
    }

    pub fn current_target(self) -> CurrentTarget {
        match self {
            Self::Disabled => BufferModeConf::Regular.current_target(),
            Self::Enabled(conf) => conf.current_target(),
        }
    }

    pub fn m1a(self) -> Option<M1a> {
        match self {
            Self::Disabled => BufferModeConf::Regular.m1a(),
            Self::Enabled(conf) => conf.m1a(),
        }
    }
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

impl BufferModeConf {
    pub fn current_target(self) -> CurrentTarget {
        match self {
            Self::Regular => CurrentTarget::M0a,
            Self::DoubleBuffer(conf) => conf.current_target,
        }
    }

    pub fn m1a(self) -> Option<M1a> {
        match self {
            Self::Regular => None,
            Self::DoubleBuffer(conf) => Some(conf.m1a),
        }
    }
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
