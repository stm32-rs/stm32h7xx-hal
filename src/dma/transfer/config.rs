use super::super::stream::config::{
    BufferModeConf, CircularModeConf, Config as StreamConfig, CurrentTarget,
    DirectConf, DoubleBufferConf, FifoConf, FifoThreshold, FlowControllerConf,
    M0a, M1a, MBurst, MSize, Minc, Ndt, NotM2MConf, PBurstConf, Pa, Pinc,
    PincConf, Pincos, TransferDirectionConf, TransferModeConf,
};
use super::buffer::{IncrementedBuffer, MemoryBufferType};
use super::{
    Buffer, IPayloadSize, Payload, PayloadSize, TransferBuffers,
    TransferDirection,
};
use core::convert::TryFrom;

#[derive(Debug)]
pub struct Config<'wo, Peripheral, Memory>
where
    Peripheral: Payload,
    Memory: Payload,
{
    transfer_direction: TransferDirection<'wo, Peripheral, Memory>,
    additional_config: AdditionalConfig,
}

impl<'wo, Peripheral, Memory> Config<'wo, Peripheral, Memory>
where
    Peripheral: Payload,
    Memory: Payload,
{
    pub fn new(
        transfer_direction: TransferDirection<'wo, Peripheral, Memory>,
        additional_config: AdditionalConfig,
    ) -> Self {
        let s = Self {
            transfer_direction,
            additional_config,
        };

        s.check_self();

        s
    }

    pub fn additional_config(&self) -> AdditionalConfig {
        self.additional_config
    }

    pub fn transfer_direction(
        &self,
    ) -> &TransferDirection<'wo, Peripheral, Memory> {
        &self.transfer_direction
    }

    pub fn transfer_direction_mut<F>(&mut self, op: F)
    where
        for<'a> F: FnOnce(&'a mut TransferDirection<'wo, Peripheral, Memory>),
    {
        op(&mut self.transfer_direction);

        self.check_self();
    }

    pub unsafe fn transfer_direction_mut_unchecked(
        &mut self,
    ) -> &mut TransferDirection<'wo, Peripheral, Memory> {
        &mut self.transfer_direction
    }

    pub fn transfer_direction_conf(&self) -> TransferDirectionConf {
        match self.transfer_direction() {
            TransferDirection::P2M(_) => {
                TransferDirectionConf::P2M(self.not_m2m_conf())
            }
            TransferDirection::M2P(_) => {
                TransferDirectionConf::M2P(self.not_m2m_conf())
            }
            TransferDirection::M2M(_) => {
                TransferDirectionConf::M2M(self.fifo_conf().unwrap())
            }
        }
    }

    fn not_m2m_conf(&self) -> NotM2MConf {
        NotM2MConf {
            transfer_mode: self.transfer_mode_conf(),
            flow_controller: self.flow_controller_conf(),
        }
    }

    fn transfer_mode_conf(&self) -> TransferModeConf {
        self.additional_config.transfer_mode.into_stream_conf(
            Some(self.m_size()),
            self.pinc(),
            self.pincos(),
        )
    }

    fn flow_controller_conf(&self) -> FlowControllerConf {
        self.additional_config
            .flow_controller
            .into_stream_conf(self.m1a())
    }

    fn pinc(&self) -> Pinc {
        let buffers = self.buffers().get();

        if buffers.peripheral_buffer.is_fixed() {
            Pinc::Fixed
        } else {
            Pinc::Incremented
        }
    }

    fn pincos(&self) -> Option<Pincos> {
        let buffers = self.buffers().get();

        match &buffers.peripheral_buffer {
            Buffer::Fixed(_) => None,
            Buffer::Incremented(buffer) => match buffer {
                IncrementedBuffer::RegularOffset(_) => Some(Pincos::PSize),
                IncrementedBuffer::WordOffset(_) => Some(Pincos::Word),
            },
        }
    }

    fn minc(&self) -> Minc {
        let buffers = self.buffers().get();

        match &buffers.memory_buffer.m0a().get() {
            Buffer::Fixed(_) => Minc::Fixed,
            Buffer::Incremented(_) => Minc::Incremented,
        }
    }

    fn m_size(&self) -> MSize {
        PayloadSize::from_payload::<Memory>().into()
    }

    fn fifo_conf(&self) -> Option<FifoConf> {
        if let TransferModeTransferConf::Fifo(conf) =
            self.additional_config.transfer_mode
        {
            Some(conf.into_stream_conf(
                self.m_size(),
                self.pinc(),
                self.pincos(),
            ))
        } else {
            None
        }
    }

    pub fn pa(&self) -> Pa {
        Pa(self.buffers().get().peripheral_buffer.as_ptr(Some(0)) as u32)
    }

    pub fn m0a(&self) -> M0a {
        match &self.buffers().get().memory_buffer {
            MemoryBufferType::SingleBuffer(memory) => {
                M0a(memory.get().as_ptr(Some(0)) as u32)
            }
            MemoryBufferType::DoubleBuffer(double) => {
                M0a(double.memories()[0].get().as_ptr(Some(0)) as u32)
            }
        }
    }

    pub fn m1a(&self) -> Option<M1a> {
        if let MemoryBufferType::DoubleBuffer(buffer) =
            &self.buffers().get().memory_buffer
        {
            Some(M1a(buffer.memories()[1].get().as_ptr(Some(0)) as u32))
        } else {
            None
        }
    }

    pub fn stream_config(&self, conf: &mut StreamConfig) {
        conf.transfer_direction = self.transfer_direction_conf();

        let buffers = self.buffers().get();

        // Configure ndt

        match &buffers.peripheral_buffer {
            Buffer::Fixed(_) => {
                match buffers.memory_buffer.m0a().get() {
                    Buffer::Fixed(_) => {
                        // NDT must be configured in advance
                    }
                    Buffer::Incremented(buffer) => {
                        let p_size: usize =
                            PayloadSize::from_payload::<Peripheral>().into();
                        let m_size: usize =
                            PayloadSize::from_payload::<Memory>().into();

                        let memory_bytes = buffer.len() * m_size;

                        if memory_bytes % p_size != 0 {
                            panic!("Last transfer may be incomplete.");
                        }

                        let ndt = u16::try_from(memory_bytes / p_size).unwrap();
                        conf.ndt = Ndt(ndt);
                    }
                }
            }
            Buffer::Incremented(buffer) => {
                let ndt = u16::try_from(buffer.len()).unwrap();
                conf.ndt = Ndt(ndt);
            }
        }

        // Configure addresses
        // Note: M1a is already configured in `self.transfer_direction_conf()`

        conf.pa = self.pa();
        conf.m0a = self.m0a();

        // Configure Incrementing
        // Note: Pinc is already configured in `self.transfer_direction_conf()`

        conf.minc = self.minc();
    }

    pub fn buffers(&self) -> &TransferBuffers<'wo, Peripheral, Memory> {
        self.transfer_direction.buffers()
    }

    pub fn buffers_mut<F>(&mut self, op: F)
    where
        for<'a> F: FnOnce(&'a mut TransferBuffers<'wo, Peripheral, Memory>),
    {
        self.transfer_direction.buffers_mut(op);

        self.check_self();
    }

    pub unsafe fn buffers_mut_unchecked(
        &mut self,
    ) -> &mut TransferBuffers<'wo, Peripheral, Memory> {
        self.transfer_direction.buffers_mut_unchecked()
    }

    pub fn free(self) -> TransferDirection<'wo, Peripheral, Memory> {
        self.transfer_direction
    }

    fn check_self(&self) {
        let buffers = self.buffers().get();
        // Check Buffer Mode

        if matches!(
            self.additional_config.flow_controller,
            FlowControllerTransferConf::Dma(
                CircularModeTransferConf::Enabled(
                    BufferModeTransferConf::DoubleBuffer(_),
                ),
            )
        ) {
            assert!(buffers.memory_buffer.is_double_buffer());
        } else {
            assert!(buffers.memory_buffer.is_single_buffer());
        }

        // Check Transfer Direction

        if matches!(self.transfer_direction(), TransferDirection::M2M(_)) {
            assert!(matches!(
                self.additional_config.transfer_mode,
                TransferModeTransferConf::Fifo(_)
            ));
            assert!(matches!(
                self.additional_config.flow_controller,
                FlowControllerTransferConf::Dma(
                    CircularModeTransferConf::Disabled
                )
            ));
        }

        // Check Transfer Mode

        if matches!(
            self.additional_config.transfer_mode,
            TransferModeTransferConf::Direct
        ) {
            assert_eq!(Peripheral::Size::SIZE, Memory::Size::SIZE);
        }

        // Check Incrementing
    }
}

#[derive(Debug, Clone, Copy)]
pub struct AdditionalConfig {
    pub transfer_mode: TransferModeTransferConf,
    pub flow_controller: FlowControllerTransferConf,
}

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum TransferModeTransferConf {
    Direct,
    Fifo(FifoTransferConf),
}

impl TransferModeTransferConf {
    pub fn into_stream_conf(
        self,
        m_size: Option<MSize>,
        pinc: Pinc,
        pincos: Option<Pincos>,
    ) -> TransferModeConf {
        match self {
            Self::Direct => TransferModeConf::Direct(DirectConf { pinc }),
            Self::Fifo(fifo) => TransferModeConf::Fifo(fifo.into_stream_conf(
                m_size.unwrap(),
                pinc,
                pincos,
            )),
        }
    }
}

impl From<TransferModeConf> for TransferModeTransferConf {
    fn from(x: TransferModeConf) -> Self {
        match x {
            TransferModeConf::Direct(_) => Self::Direct,
            TransferModeConf::Fifo(fifo) => Self::Fifo(fifo.into()),
        }
    }
}

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct FifoTransferConf {
    pub fifo_threshold: FifoThreshold,
    pub p_burst: PBurstTransferConf,
    pub m_burst: MBurst,
}

impl FifoTransferConf {
    pub fn into_stream_conf(
        self,
        m_size: MSize,
        pinc: Pinc,
        pincos: Option<Pincos>,
    ) -> FifoConf {
        FifoConf {
            fifo_threshold: self.fifo_threshold,
            p_burst: self.p_burst.into_stream_conf(pinc, pincos),
            m_burst: self.m_burst,
            m_size,
        }
    }
}

impl From<FifoConf> for FifoTransferConf {
    fn from(x: FifoConf) -> Self {
        Self {
            fifo_threshold: x.fifo_threshold,
            p_burst: x.p_burst.into(),
            m_burst: x.m_burst,
        }
    }
}

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum PBurstTransferConf {
    Single,
    Incr4,
    Incr8,
    Incr16,
}

impl PBurstTransferConf {
    pub fn into_stream_conf(
        self,
        pinc: Pinc,
        pincos: Option<Pincos>,
    ) -> PBurstConf {
        match self {
            Self::Single => {
                let pinc_conf = match pinc {
                    Pinc::Fixed => PincConf::Fixed,
                    Pinc::Incremented => PincConf::Incremented(pincos.unwrap()),
                };

                PBurstConf::Single(pinc_conf)
            }
            Self::Incr4 => PBurstConf::Incr4(pinc),
            Self::Incr8 => PBurstConf::Incr8(pinc),
            Self::Incr16 => PBurstConf::Incr16(pinc),
        }
    }
}

impl From<PBurstConf> for PBurstTransferConf {
    fn from(x: PBurstConf) -> Self {
        match x {
            PBurstConf::Single(_) => Self::Single,
            PBurstConf::Incr4(_) => Self::Incr4,
            PBurstConf::Incr8(_) => Self::Incr8,
            PBurstConf::Incr16(_) => Self::Incr16,
        }
    }
}

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum FlowControllerTransferConf {
    Dma(CircularModeTransferConf),
    Peripheral,
}

impl FlowControllerTransferConf {
    pub fn into_stream_conf(self, m1a: Option<M1a>) -> FlowControllerConf {
        match self {
            Self::Dma(circular) => {
                FlowControllerConf::Dma(circular.into_stream_conf(m1a))
            }
            Self::Peripheral => FlowControllerConf::Peripheral,
        }
    }
}

impl From<FlowControllerConf> for FlowControllerTransferConf {
    fn from(x: FlowControllerConf) -> Self {
        match x {
            FlowControllerConf::Dma(circ) => Self::Dma(circ.into()),
            FlowControllerConf::Peripheral => Self::Peripheral,
        }
    }
}

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum CircularModeTransferConf {
    Disabled,
    Enabled(BufferModeTransferConf),
}

impl CircularModeTransferConf {
    pub fn into_stream_conf(self, m1a: Option<M1a>) -> CircularModeConf {
        match self {
            Self::Disabled => CircularModeConf::Disabled,
            Self::Enabled(buffer_mode) => {
                CircularModeConf::Enabled(buffer_mode.into_stream_conf(m1a))
            }
        }
    }
}

impl From<CircularModeConf> for CircularModeTransferConf {
    fn from(x: CircularModeConf) -> Self {
        match x {
            CircularModeConf::Disabled => Self::Disabled,
            CircularModeConf::Enabled(buffer_mode) => {
                Self::Enabled(buffer_mode.into())
            }
        }
    }
}

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum BufferModeTransferConf {
    Regular,
    DoubleBuffer(DoubleBufferTransferConf),
}

impl BufferModeTransferConf {
    pub fn into_stream_conf(self, m1a: Option<M1a>) -> BufferModeConf {
        match self {
            Self::Regular => BufferModeConf::Regular,
            Self::DoubleBuffer(conf) => BufferModeConf::DoubleBuffer(
                conf.into_stream_conf(m1a.unwrap()),
            ),
        }
    }
}

impl From<BufferModeConf> for BufferModeTransferConf {
    fn from(x: BufferModeConf) -> Self {
        match x {
            BufferModeConf::Regular => Self::Regular,
            BufferModeConf::DoubleBuffer(conf) => {
                Self::DoubleBuffer(DoubleBufferTransferConf {
                    current_target: conf.current_target,
                })
            }
        }
    }
}

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub struct DoubleBufferTransferConf {
    pub current_target: CurrentTarget,
}

impl DoubleBufferTransferConf {
    pub fn into_stream_conf(self, m1a: M1a) -> DoubleBufferConf {
        DoubleBufferConf {
            current_target: self.current_target,
            m1a,
        }
    }
}

impl From<DoubleBufferConf> for DoubleBufferTransferConf {
    fn from(x: DoubleBufferConf) -> Self {
        Self {
            current_target: x.current_target,
        }
    }
}
