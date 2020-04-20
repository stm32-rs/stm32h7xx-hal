//! Safe DMA Transfers

use super::stream::{
    BufferModeConf, CircularModeConf, Config as StreamConfig, CurrentTarget,
    DirectConf, DoubleBufferConf, Enabled, FifoConf, FifoThreshold,
    FlowControllerConf, IsrUncleared, M0a, M1a, MBurst, MSize, Minc, Ndt,
    NotM2MConf, PBurstConf, PSize, Pa, Pinc, PincConf, Pincos,
    TransferDirectionConf, TransferModeConf,
};
use super::{ChannelId, Stream};
use crate::private;
use core::convert::TryFrom;
use core::fmt::Debug;
use core::{mem, ptr};
use enum_as_inner::EnumAsInner;

pub trait TransferState<'wo>: Send + Sync + private::Sealed {
    type Peripheral: Payload;
    type Memory: Payload;

    fn buffers(&self) -> &TransferBuffers<'wo, Self::Peripheral, Self::Memory>;

    fn buffers_mut<F>(&mut self, op: F)
    where
        for<'a> F: FnOnce(
            &'a mut TransferBuffers<'wo, Self::Peripheral, Self::Memory>,
        );

    unsafe fn buffers_mut_unchecked(
        &mut self,
    ) -> &mut TransferBuffers<'wo, Self::Peripheral, Self::Memory>;
}

pub struct Start<'wo, Peripheral, Memory>
where
    Peripheral: Payload,
    Memory: Payload,
{
    pub(super) conf: Config<'wo, Peripheral, Memory>,
}

impl<Peripheral, Memory> private::Sealed for Start<'_, Peripheral, Memory>
where
    Peripheral: Payload,
    Memory: Payload,
{
}

impl<'wo, Peripheral, Memory> TransferState<'wo>
    for Start<'wo, Peripheral, Memory>
where
    Peripheral: Payload,
    Memory: Payload,
{
    type Peripheral = Peripheral;
    type Memory = Memory;

    fn buffers(&self) -> &TransferBuffers<'wo, Peripheral, Memory> {
        self.conf.transfer_direction.buffers()
    }

    fn buffers_mut<F>(&mut self, op: F)
    where
        for<'a> F: FnOnce(&'a mut TransferBuffers<'wo, Peripheral, Memory>),
    {
        self.conf.transfer_direction.buffers_mut(|b| op(b));
    }

    unsafe fn buffers_mut_unchecked(
        &mut self,
    ) -> &mut TransferBuffers<'wo, Peripheral, Memory> {
        self.conf.transfer_direction.buffers_mut_unchecked()
    }
}

pub struct Ongoing<'wo, Peripheral, Memory, CXX>
where
    Peripheral: Payload,
    Memory: Payload,
    CXX: ChannelId,
{
    pub(super) stream: Stream<CXX, Enabled, IsrUncleared>,
    pub(super) buffers: TransferBuffers<'wo, Peripheral, Memory>,
}

impl<Peripheral, Memory, CXX> private::Sealed
    for Ongoing<'_, Peripheral, Memory, CXX>
where
    Peripheral: Payload,
    Memory: Payload,
    CXX: ChannelId,
{
}

impl<'wo, Peripheral, Memory, CXX> TransferState<'wo>
    for Ongoing<'wo, Peripheral, Memory, CXX>
where
    Peripheral: Payload,
    Memory: Payload,
    CXX: ChannelId,
{
    type Peripheral = Peripheral;
    type Memory = Memory;

    fn buffers(&self) -> &TransferBuffers<'wo, Peripheral, Memory> {
        &self.buffers
    }

    fn buffers_mut<F>(&mut self, op: F)
    where
        for<'a> F: FnOnce(&'a mut TransferBuffers<'wo, Peripheral, Memory>),
    {
        op(&mut self.buffers)
    }

    unsafe fn buffers_mut_unchecked(
        &mut self,
    ) -> &mut TransferBuffers<'wo, Peripheral, Memory> {
        &mut self.buffers
    }
}

/// # Safety
///
/// * `Self` must be valid for any bit representation
/// * `Self::Size` must be equal to actual size
pub unsafe trait Payload:
    Sized + Clone + Copy + Send + Sync + 'static
{
    type Size: IPayloadSize;
}

// Maps Payload size to number of bytes
int_enum! {
    PayloadSize <=> usize,
    "Payload Size",
    Byte <=> 1,
    HalfWord <=> 2,
    Word <=> 4
}

impl From<PayloadSize> for MSize {
    fn from(val: PayloadSize) -> Self {
        match val {
            PayloadSize::Byte => MSize::Byte,
            PayloadSize::HalfWord => MSize::HalfWord,
            PayloadSize::Word => MSize::Word,
        }
    }
}

impl From<MSize> for PayloadSize {
    fn from(val: MSize) -> Self {
        match val {
            MSize::Byte => PayloadSize::Byte,
            MSize::HalfWord => PayloadSize::HalfWord,
            MSize::Word => PayloadSize::Word,
        }
    }
}

impl From<PayloadSize> for PSize {
    fn from(val: PayloadSize) -> Self {
        match val {
            PayloadSize::Byte => PSize::Byte,
            PayloadSize::HalfWord => PSize::HalfWord,
            PayloadSize::Word => PSize::Word,
        }
    }
}

impl From<PSize> for PayloadSize {
    fn from(val: PSize) -> Self {
        match val {
            PSize::Byte => PayloadSize::Byte,
            PSize::HalfWord => PayloadSize::HalfWord,
            PSize::Word => PayloadSize::Word,
        }
    }
}

impl PayloadSize {
    pub fn from_payload<P: Payload>() -> Self {
        let size = P::Size::SIZE;

        debug_assert_eq!(mem::size_of::<P>(), size.into());

        size
    }
}

pub trait IPayloadSize {
    const SIZE: PayloadSize;
}

pub struct Byte;
pub struct HalfWord;
pub struct Word;

impl IPayloadSize for Byte {
    const SIZE: PayloadSize = PayloadSize::Byte;
}
impl IPayloadSize for HalfWord {
    const SIZE: PayloadSize = PayloadSize::HalfWord;
}
impl IPayloadSize for Word {
    const SIZE: PayloadSize = PayloadSize::Word;
}

unsafe impl Payload for u8 {
    type Size = Byte;
}

unsafe impl Payload for i8 {
    type Size = Byte;
}

unsafe impl Payload for u16 {
    type Size = HalfWord;
}

unsafe impl Payload for i16 {
    type Size = HalfWord;
}

unsafe impl Payload for u32 {
    type Size = Word;
}

unsafe impl Payload for i32 {
    type Size = Word;
}

unsafe impl Payload for f32 {
    type Size = Word;
}

#[derive(Debug)]
pub struct MemoryBuffer<Memory>
where
    Memory: Payload,
{
    buffer: Buffer<'static, Memory>,
}

impl<Memory> MemoryBuffer<Memory>
where
    Memory: Payload,
{
    pub fn new(buffer: Buffer<'static, Memory>) -> Self {
        let s = Self { buffer };

        s.check_self();

        s
    }

    pub fn get(&self) -> &Buffer<'static, Memory> {
        &self.buffer
    }

    pub fn get_mut<F>(&mut self, op: F)
    where
        for<'a> F: FnOnce(&'a mut Buffer<'static, Memory>),
    {
        op(&mut self.buffer);

        self.check_self();
    }

    pub unsafe fn get_mut_unchecked(&mut self) -> &mut Buffer<'static, Memory> {
        &mut self.buffer
    }

    pub fn free(self) -> Buffer<'static, Memory> {
        self.buffer
    }

    fn check_self(&self) {
        if let Buffer::Incremented(IncrementedBuffer::WordOffset(_)) =
            &self.buffer
        {
            panic!("Memory Buffer can't have word offset.");
        }
    }
}

#[derive(Debug, EnumAsInner)]
pub enum MemoryBufferType<Memory>
where
    Memory: Payload,
{
    SingleBuffer(MemoryBuffer<Memory>),
    DoubleBuffer(DoubleBuffer<Memory>),
}

impl<Memory> MemoryBufferType<Memory>
where
    Memory: Payload,
{
    pub fn is_single_buffer(&self) -> bool {
        self.as_single_buffer().is_some()
    }

    pub fn is_double_buffer(&self) -> bool {
        self.as_double_buffer().is_some()
    }

    pub fn is_read(&self) -> bool {
        match self {
            MemoryBufferType::SingleBuffer(buffer) => buffer.get().is_read(),
            MemoryBufferType::DoubleBuffer(buffer) => {
                buffer.memories[0].get().is_read()
            }
        }
    }

    pub fn is_write(&self) -> bool {
        !self.is_read()
    }

    pub fn m0a(&self) -> &MemoryBuffer<Memory> {
        match self {
            Self::SingleBuffer(buffer) => &buffer,
            Self::DoubleBuffer(buffer) => &buffer.memories()[0],
        }
    }
}

#[derive(Debug)]
pub struct DoubleBuffer<Memory>
where
    Memory: Payload,
{
    memories: [MemoryBuffer<Memory>; 2],
}

impl<Memory> DoubleBuffer<Memory>
where
    Memory: Payload,
{
    pub fn new(memories: [MemoryBuffer<Memory>; 2]) -> Self {
        let s = Self { memories };

        s.check_self();

        s
    }

    pub fn memories(&self) -> &[MemoryBuffer<Memory>; 2] {
        &self.memories
    }

    /// Exposes a mutable reference to the double buffer inside a closure.
    ///
    /// At the end, the double buffer will be checked.
    ///
    /// If the closure is too limiting, consider using the unchecked version.
    pub fn memories_mut<F>(&mut self, op: F)
    where
        for<'a> F: FnOnce(&'a mut [MemoryBuffer<Memory>; 2]),
    {
        op(&mut self.memories);

        self.check_self();
    }

    /// Returns mutable reference to the double buffer.
    ///
    /// # Safety
    ///
    /// This is unsafe because - by `mem::replace`ing the buffers - the api contract of both buffers
    /// having the same characteristics (fixed/incremented, read/write) can be violated.
    /// This may result in breaking the aliasing rules and subsequently undefined behaviour.
    ///
    /// This is safe as long as you don't `mem::replace` the buffers, or at least ensure that
    /// both buffers have the same characteristics afterwards.
    pub unsafe fn memories_mut_unchecked(
        &mut self,
    ) -> &mut [MemoryBuffer<Memory>; 2] {
        &mut self.memories
    }

    pub fn free(self) -> [MemoryBuffer<Memory>; 2] {
        self.memories
    }

    fn check_self(&self) {
        assert_eq!(
            self.memories[0].get().is_read(),
            self.memories[1].get().is_read()
        );
        assert_eq!(
            self.memories[0].get().is_fixed(),
            self.memories[1].get().is_fixed()
        );

        if self.memories[0].get().is_incremented() {
            assert_eq!(
                self.memories[0].get().as_incremented().unwrap().len(),
                self.memories[1].get().as_incremented().unwrap().len()
            );
        }
    }
}

#[derive(Debug, EnumAsInner)]
pub enum TransferDirection<'wo, Peripheral, Memory>
where
    Peripheral: Payload,
    Memory: Payload,
{
    P2M(PeripheralToMemory<'wo, Peripheral, Memory>),
    M2P(MemoryToPeripheral<'wo, Peripheral, Memory>),
    M2M(MemoryToMemory<'wo, Peripheral, Memory>),
}

impl<'wo, Peripheral, Memory> TransferDirection<'wo, Peripheral, Memory>
where
    Peripheral: Payload,
    Memory: Payload,
{
    pub fn buffers(&self) -> &TransferBuffers<'wo, Peripheral, Memory> {
        match self {
            TransferDirection::P2M(p2m) => p2m.buffers(),
            TransferDirection::M2P(m2p) => m2p.buffers(),
            TransferDirection::M2M(m2m) => m2m.buffers(),
        }
    }

    pub fn buffers_mut<F>(&mut self, op: F)
    where
        for<'a> F: FnOnce(&'a mut TransferBuffers<'wo, Peripheral, Memory>),
    {
        match self {
            TransferDirection::P2M(p2m) => p2m.buffers_mut(move |b| op(b)),
            TransferDirection::M2P(m2p) => m2p.buffers_mut(move |b| op(b)),
            TransferDirection::M2M(m2m) => m2m.buffers_mut(move |b| op(b)),
        }
    }

    pub unsafe fn buffers_mut_unchecked(
        &mut self,
    ) -> &mut TransferBuffers<'wo, Peripheral, Memory> {
        match self {
            TransferDirection::P2M(p2m) => p2m.buffers_mut_unchecked(),
            TransferDirection::M2P(m2p) => m2p.buffers_mut_unchecked(),
            TransferDirection::M2M(m2m) => m2m.buffers_mut_unchecked(),
        }
    }

    pub fn free(self) -> TransferBuffers<'wo, Peripheral, Memory> {
        match self {
            TransferDirection::P2M(p2m) => p2m.free(),
            TransferDirection::M2P(m2p) => m2p.free(),
            TransferDirection::M2M(m2m) => m2m.free(),
        }
    }
}

#[derive(Debug)]
pub struct PeripheralToMemory<'wo, Peripheral, Memory>
where
    Peripheral: Payload,
    Memory: Payload,
{
    buffers: TransferBuffers<'wo, Peripheral, Memory>,
}

impl<'wo, Peripheral, Memory> PeripheralToMemory<'wo, Peripheral, Memory>
where
    Peripheral: Payload,
    Memory: Payload,
{
    pub fn new(buffers: TransferBuffers<'wo, Peripheral, Memory>) -> Self {
        let s = Self { buffers };

        s.check_self();

        s
    }

    pub fn buffers(&self) -> &TransferBuffers<'wo, Peripheral, Memory> {
        &self.buffers
    }

    pub fn buffers_mut<F>(&mut self, op: F)
    where
        for<'a> F: FnOnce(&'a mut TransferBuffers<'wo, Peripheral, Memory>),
    {
        op(&mut self.buffers);

        self.check_self();
    }

    pub unsafe fn buffers_mut_unchecked(
        &mut self,
    ) -> &mut TransferBuffers<'wo, Peripheral, Memory> {
        &mut self.buffers
    }

    pub fn free(self) -> TransferBuffers<'wo, Peripheral, Memory> {
        self.buffers
    }

    fn check_self(&self) {
        assert!(self.buffers.get().peripheral_buffer.is_read());
        assert!(self.buffers.get().memory_buffer.is_read());
    }
}

#[derive(Debug)]
pub struct MemoryToPeripheral<'wo, Peripheral, Memory>
where
    Peripheral: Payload,
    Memory: Payload,
{
    buffers: TransferBuffers<'wo, Peripheral, Memory>,
}

impl<'wo, Peripheral, Memory> MemoryToPeripheral<'wo, Peripheral, Memory>
where
    Peripheral: Payload,
    Memory: Payload,
{
    pub fn new(buffers: TransferBuffers<'wo, Peripheral, Memory>) -> Self {
        let s = Self { buffers };

        s.check_self();

        s
    }

    pub fn buffers(&self) -> &TransferBuffers<'wo, Peripheral, Memory> {
        &self.buffers
    }

    pub fn buffers_mut<F>(&mut self, op: F)
    where
        for<'a> F: FnOnce(&'a mut TransferBuffers<'wo, Peripheral, Memory>),
    {
        op(&mut self.buffers);

        self.check_self();
    }

    pub unsafe fn buffers_mut_unchecked(
        &mut self,
    ) -> &mut TransferBuffers<'wo, Peripheral, Memory> {
        &mut self.buffers
    }

    pub fn free(self) -> TransferBuffers<'wo, Peripheral, Memory> {
        self.buffers
    }

    fn check_self(&self) {
        assert!(self.buffers.get().peripheral_buffer.is_write());
        assert!(self.buffers.get().memory_buffer.is_read());
    }
}

#[derive(Debug)]
pub struct MemoryToMemory<'wo, Source, Dest>
where
    Source: Payload,
    Dest: Payload,
{
    buffers: TransferBuffers<'wo, Source, Dest>,
}

impl<'wo, Source, Dest> MemoryToMemory<'wo, Source, Dest>
where
    Source: Payload,
    Dest: Payload,
{
    pub fn new(buffers: TransferBuffers<'wo, Source, Dest>) -> Self {
        let s = Self { buffers };

        s.check_self();

        s
    }

    pub fn buffers(&self) -> &TransferBuffers<'wo, Source, Dest> {
        &self.buffers
    }

    pub fn buffers_mut<F>(&mut self, op: F)
    where
        for<'a> F: FnOnce(&'a mut TransferBuffers<'wo, Source, Dest>),
    {
        op(&mut self.buffers);

        self.check_self();
    }

    pub unsafe fn buffers_mut_unchecked(
        &mut self,
    ) -> &mut TransferBuffers<'wo, Source, Dest> {
        &mut self.buffers
    }

    pub fn source_buffer(&self) -> &Buffer<'wo, Source> {
        &self.buffers.get().peripheral_buffer
    }

    pub fn dest_buffer(&self) -> &MemoryBuffer<Dest> {
        &self.buffers.get().memory_buffer.as_single_buffer().unwrap()
    }

    pub fn dest_buffer_mut<F>(&mut self, op: F)
    where
        for<'a> F: FnOnce(&'a mut MemoryBuffer<Dest>),
    {
        self.buffers.get_mut(move |b| {
            op(&mut b.memory_buffer.as_single_buffer_mut().unwrap())
        });
    }

    pub unsafe fn dest_buffer_mut_unchecked(
        &mut self,
    ) -> &mut MemoryBuffer<Dest> {
        self.buffers
            .get_mut_unchecked()
            .memory_buffer
            .as_single_buffer_mut()
            .unwrap()
    }

    pub fn free(self) -> TransferBuffers<'wo, Source, Dest> {
        self.buffers
    }

    fn check_self(&self) {
        assert!(self.buffers.get().peripheral_buffer.is_read());
        assert!(self.buffers.get().memory_buffer.is_write());

        assert!(self.buffers.get().memory_buffer.is_single_buffer());
    }
}

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
        self.transfer_direction.buffers_mut(|b| op(b));

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

#[derive(Debug)]
pub struct Buffers<'wo, Peripheral, Memory>
where
    Peripheral: Payload,
    Memory: Payload,
{
    pub peripheral_buffer: Buffer<'wo, Peripheral>,
    pub memory_buffer: MemoryBufferType<Memory>,
}

#[derive(Debug)]
pub struct TransferBuffers<'wo, Peripheral, Memory>
where
    Peripheral: Payload,
    Memory: Payload,
{
    buffers: Buffers<'wo, Peripheral, Memory>,
}

impl<'wo, Peripheral, Memory> TransferBuffers<'wo, Peripheral, Memory>
where
    Peripheral: Payload,
    Memory: Payload,
{
    /// # Args
    ///
    /// * `peripheral_buffer`: Usually `Buffer::Peripheral`, except for `M2M`-transfers (Memory to Memory), where the source buffer is `Buffer::Memory`.
    /// * `memory_buffer`: The `MemoryBuffer` of the transfer.
    pub fn new(buffers: Buffers<'wo, Peripheral, Memory>) -> Self {
        let s = Self { buffers };

        s.check_self();

        s
    }

    pub fn get(&self) -> &Buffers<'wo, Peripheral, Memory> {
        &self.buffers
    }

    pub fn get_mut<F>(&mut self, op: F)
    where
        for<'a> F: FnOnce(&'a mut Buffers<'wo, Peripheral, Memory>),
    {
        op(&mut self.buffers);

        self.check_self();
    }

    pub unsafe fn get_mut_unchecked(
        &mut self,
    ) -> &mut Buffers<'wo, Peripheral, Memory> {
        &mut self.buffers
    }

    pub fn free(self) -> Buffers<'wo, Peripheral, Memory> {
        self.buffers
    }

    fn check_self(&self) {
        assert_ne!(
            self.buffers.peripheral_buffer.is_read(),
            self.buffers.memory_buffer.is_read()
        );
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

#[derive(Copy, Clone, Debug, EnumAsInner)]
pub enum PayloadPort<Peripheral, Memory>
where
    Peripheral: Payload,
    Memory: Payload,
{
    Peripheral(Peripheral),
    Memory(Memory),
}

#[derive(Debug, EnumAsInner)]
pub enum PointerPort<Peripheral, Memory>
where
    Peripheral: Payload,
    Memory: Payload,
{
    Peripheral(*mut Peripheral),
    Memory(*mut Memory),
}

#[derive(Debug, EnumAsInner)]
pub enum Buffer<'wo, P>
where
    P: Payload,
{
    Fixed(FixedBuffer<P>),
    Incremented(IncrementedBuffer<'wo, P>),
}

impl<'wo, P> Buffer<'wo, P>
where
    P: Payload,
{
    pub unsafe fn get(&self, index: Option<usize>) -> P {
        match self {
            Buffer::Fixed(buffer) => buffer.get(),
            Buffer::Incremented(buffer) => buffer.get(index.unwrap()),
        }
    }

    pub fn as_ptr(&self, index: Option<usize>) -> *const P {
        match self {
            Buffer::Fixed(buffer) => buffer.as_ptr(),
            Buffer::Incremented(buffer) => buffer.as_ptr(index.unwrap()),
        }
    }

    pub fn is_read(&self) -> bool {
        match self {
            Buffer::Fixed(buffer) => buffer.is_read(),
            Buffer::Incremented(buffer) => buffer.is_read(),
        }
    }

    pub fn is_write(&self) -> bool {
        match self {
            Buffer::Fixed(buffer) => buffer.is_write(),
            Buffer::Incremented(buffer) => buffer.is_write(),
        }
    }

    pub fn is_fixed(&self) -> bool {
        if let Self::Fixed(_) = self {
            true
        } else {
            false
        }
    }

    pub fn is_incremented(&self) -> bool {
        if let Self::Incremented(_) = self {
            true
        } else {
            false
        }
    }
}

#[derive(Debug, EnumAsInner)]
pub enum IncrementedBuffer<'wo, P>
where
    P: Payload,
{
    RegularOffset(RegularOffsetBuffer<P>),
    WordOffset(WordOffsetBuffer<'wo, P>),
}

impl<'wo, P> IncrementedBuffer<'wo, P>
where
    P: Payload,
{
    pub fn is_regular_offset(&self) -> bool {
        if let IncrementedBuffer::RegularOffset(_) = self {
            true
        } else {
            false
        }
    }

    pub fn is_word_offset(&self) -> bool {
        if let IncrementedBuffer::WordOffset(_) = self {
            true
        } else {
            false
        }
    }

    pub fn len(&self) -> usize {
        match self {
            IncrementedBuffer::RegularOffset(buffer) => buffer.len(),
            IncrementedBuffer::WordOffset(buffer) => buffer.len(),
        }
    }

    pub unsafe fn get(&self, index: usize) -> P {
        match self {
            IncrementedBuffer::RegularOffset(buffer) => buffer.get(index),
            IncrementedBuffer::WordOffset(buffer) => buffer.get(index),
        }
    }

    pub fn as_ptr(&self, index: usize) -> *const P {
        match self {
            IncrementedBuffer::RegularOffset(buffer) => buffer.as_ptr(index),
            IncrementedBuffer::WordOffset(buffer) => buffer.as_ptr(index),
        }
    }

    pub fn is_read(&self) -> bool {
        match self {
            IncrementedBuffer::RegularOffset(buffer) => buffer.is_read(),
            IncrementedBuffer::WordOffset(buffer) => buffer.is_read(),
        }
    }

    pub fn is_write(&self) -> bool {
        match self {
            IncrementedBuffer::RegularOffset(buffer) => buffer.is_write(),
            IncrementedBuffer::WordOffset(buffer) => buffer.is_write(),
        }
    }
}

#[derive(Debug, EnumAsInner)]
pub enum FixedBuffer<P>
where
    P: Payload,
{
    Read(FixedBufferR<P>),
    Write(FixedBufferW<P>),
}

impl<P> FixedBuffer<P>
where
    P: Payload,
{
    pub fn is_read(&self) -> bool {
        match self {
            FixedBuffer::Read(_) => true,
            FixedBuffer::Write(_) => false,
        }
    }

    pub fn is_write(&self) -> bool {
        match self {
            FixedBuffer::Write(_) => true,
            FixedBuffer::Read(_) => false,
        }
    }

    pub unsafe fn get(&self) -> P {
        match self {
            FixedBuffer::Read(buffer) => buffer.get(),
            FixedBuffer::Write(buffer) => buffer.get(),
        }
    }

    pub fn as_ptr(&self) -> *const P {
        match self {
            FixedBuffer::Read(buffer) => buffer.as_ptr(),
            FixedBuffer::Write(buffer) => buffer.as_ptr(),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct FixedBufferR<P>(*const P)
where
    P: Payload;

impl<P> FixedBufferR<P>
where
    P: Payload,
{
    pub fn new(buffer: &'static P) -> Self {
        FixedBufferR(buffer)
    }

    pub fn get(self) -> P {
        unsafe { self.0.read_volatile() }
    }

    pub fn as_ptr(self) -> *const P {
        self.0
    }
}

unsafe impl<P> Send for FixedBufferR<P> where P: Payload {}

unsafe impl<P> Sync for FixedBufferR<P> where P: Payload {}

#[derive(Debug)]
pub struct FixedBufferW<P>(*mut P)
where
    P: Payload;

impl<P> FixedBufferW<P>
where
    P: Payload,
{
    pub fn new(buffer: &'static mut P) -> Self {
        FixedBufferW(buffer)
    }

    /// # Safety
    ///
    /// - The caller must ensure, that the DMA is currently not writing this address.
    pub unsafe fn get(&self) -> P {
        ptr::read_volatile(self.0)
    }

    /// # Safety
    ///
    /// - The caller must ensure, that the DMA is currently not writing this address.
    pub unsafe fn set(&mut self, buf: P) {
        ptr::write_volatile(self.0, buf);
    }

    pub fn as_ptr(&self) -> *const P {
        self.0
    }

    pub fn as_mut_ptr(&mut self) -> *mut P {
        self.0
    }
}

unsafe impl<P> Send for FixedBufferW<P> where P: Payload {}

unsafe impl<P> Sync for FixedBufferW<P> where P: Payload {}

#[derive(Debug, EnumAsInner)]
pub enum RegularOffsetBuffer<P>
where
    P: Payload,
{
    Read(RegularOffsetBufferR<P>),
    Write(RegularOffsetBufferW<P>),
}

impl<P> RegularOffsetBuffer<P>
where
    P: Payload,
{
    pub fn is_read(&self) -> bool {
        match self {
            RegularOffsetBuffer::Read(_) => true,
            RegularOffsetBuffer::Write(_) => false,
        }
    }

    pub fn is_write(&self) -> bool {
        match self {
            RegularOffsetBuffer::Write(_) => true,
            RegularOffsetBuffer::Read(_) => false,
        }
    }

    // Methods both variants implement

    pub unsafe fn get(&self, index: usize) -> P {
        match self {
            RegularOffsetBuffer::Read(buffer) => buffer.get(index),
            RegularOffsetBuffer::Write(buffer) => buffer.get(index),
        }
    }

    pub fn as_ptr(&self, index: usize) -> *const P {
        match self {
            RegularOffsetBuffer::Read(buffer) => buffer.as_ptr(index),
            RegularOffsetBuffer::Write(buffer) => buffer.as_ptr(index),
        }
    }

    pub fn len(&self) -> usize {
        match self {
            RegularOffsetBuffer::Read(buffer) => buffer.len(),
            RegularOffsetBuffer::Write(buffer) => buffer.len(),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct RegularOffsetBufferR<P>(*const [P])
where
    P: Payload;

#[allow(clippy::len_without_is_empty)]
impl<P> RegularOffsetBufferR<P>
where
    P: Payload,
{
    pub fn new(buffer: &'static [P]) -> Self {
        check_buffer_not_empty(buffer);

        RegularOffsetBufferR(buffer)
    }

    pub fn get(self, index: usize) -> P {
        unsafe { read_volatile_slice_buffer(self.0, index) }
    }

    pub fn as_ptr(self, index: usize) -> *const P {
        unsafe {
            let slice = &*self.0;
            &slice[index] as *const _
        }
    }

    pub fn len(self) -> usize {
        unsafe {
            let slice = &*self.0;
            slice.len()
        }
    }
}

unsafe impl<P> Send for RegularOffsetBufferR<P> where P: Payload {}

unsafe impl<P> Sync for RegularOffsetBufferR<P> where P: Payload {}

#[derive(Debug)]
pub struct RegularOffsetBufferW<P>(*mut [P])
where
    P: Payload;

#[allow(clippy::len_without_is_empty)]
impl<P> RegularOffsetBufferW<P>
where
    P: Payload,
{
    pub fn new(buffer: &'static mut [P]) -> Self {
        check_buffer_not_empty(buffer);

        RegularOffsetBufferW(buffer)
    }

    /// # Safety
    ///
    /// - The caller must ensure, that the DMA is currently not modifying this address.
    pub unsafe fn get(&self, index: usize) -> P {
        read_volatile_slice_buffer(self.0, index)
    }

    /// # Safety
    ///
    /// - The caller must ensure, that the DMA is currently not modifying this address.
    pub unsafe fn set(&mut self, index: usize, item: P) {
        let slice = &mut *self.0;
        ptr::write_volatile(&mut slice[index] as *mut _, item);
    }

    pub fn as_ptr(&self, index: usize) -> *const P {
        unsafe {
            let slice = &*self.0;
            &slice[index] as *const _
        }
    }

    pub fn as_mut_ptr(&mut self, index: usize) -> *mut P {
        unsafe {
            let slice = &mut *self.0;
            &mut slice[index] as *mut _
        }
    }

    pub fn len(&self) -> usize {
        unsafe {
            let slice = &*self.0;
            slice.len()
        }
    }
}

unsafe impl<P> Send for RegularOffsetBufferW<P> where P: Payload {}

unsafe impl<P> Sync for RegularOffsetBufferW<P> where P: Payload {}

unsafe fn read_volatile_slice_buffer<P>(
    slice_ptr: *const [P],
    index: usize,
) -> P
where
    P: Payload,
{
    let slice = &*slice_ptr;
    ptr::read_volatile(&slice[index] as *const _)
}

#[derive(Debug, EnumAsInner)]
pub enum WordOffsetBuffer<'wo, P>
where
    P: Payload,
{
    Read(WordOffsetBufferR<'wo, P>),
    Write(WordOffsetBufferW<'wo, P>),
}

impl<'wo, P> WordOffsetBuffer<'wo, P>
where
    P: Payload,
{
    pub fn is_read(&self) -> bool {
        match self {
            WordOffsetBuffer::Read(_) => true,
            WordOffsetBuffer::Write(_) => false,
        }
    }

    pub fn is_write(&self) -> bool {
        match self {
            WordOffsetBuffer::Write(_) => true,
            WordOffsetBuffer::Read(_) => false,
        }
    }

    pub unsafe fn get(&self, index: usize) -> P {
        match self {
            WordOffsetBuffer::Read(buffer) => buffer.get(index),
            WordOffsetBuffer::Write(buffer) => buffer.get(index),
        }
    }

    pub fn as_ptr(&self, index: usize) -> *const P {
        match self {
            WordOffsetBuffer::Read(buffer) => buffer.as_ptr(index),
            WordOffsetBuffer::Write(buffer) => buffer.as_ptr(index),
        }
    }

    pub fn len(&self) -> usize {
        match self {
            WordOffsetBuffer::Read(buffer) => buffer.len(),
            WordOffsetBuffer::Write(buffer) => buffer.len(),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct WordOffsetBufferR<'wo, P>(&'wo [*const P])
where
    P: Payload;

#[allow(clippy::len_without_is_empty)]
impl<'wo, P> WordOffsetBufferR<'wo, P>
where
    P: Payload,
{
    pub fn new(buffer: &'wo [&'static P]) -> Self {
        check_buffer_not_empty(buffer);

        let buffer = unsafe { &*(buffer as *const _ as *const _) };

        check_word_offset(buffer);

        WordOffsetBufferR(buffer)
    }

    pub fn get(self, index: usize) -> P {
        unsafe { ptr::read_volatile(self.0[index]) }
    }

    pub fn as_ptr(self, index: usize) -> *const P {
        self.0[index]
    }

    pub fn len(self) -> usize {
        self.0.len()
    }
}

unsafe impl<P> Send for WordOffsetBufferR<'_, P> where P: Payload {}

unsafe impl<P> Sync for WordOffsetBufferR<'_, P> where P: Payload {}

#[derive(Debug)]
pub struct WordOffsetBufferW<'wo, P>(&'wo mut [*mut P])
where
    P: Payload;

#[allow(clippy::len_without_is_empty)]
impl<'wo, P> WordOffsetBufferW<'wo, P>
where
    P: Payload,
{
    pub fn new(buffer: &'wo mut [&'static mut P]) -> Self {
        check_buffer_not_empty(buffer);

        unsafe {
            check_word_offset::<P>(&*(buffer as *const _ as *const _));

            WordOffsetBufferW(&mut *(buffer as *mut _ as *mut _))
        }
    }

    /// # Safety
    ///
    /// The caller must ensure, that the DMA is currently not modifying this address.
    pub unsafe fn get(&self, index: usize) -> P {
        ptr::read_volatile(self.0[index])
    }

    /// # Safety
    ///
    /// The caller must ensure, that the DMA is currently not modifying this address.
    pub unsafe fn set(&mut self, index: usize, item: P) {
        ptr::write_volatile(self.0[index], item);
    }

    pub fn as_ptr(&self, index: usize) -> *const P {
        self.0[index]
    }

    pub fn as_mut_ptr(&mut self, index: usize) -> *mut P {
        self.0[index]
    }

    pub fn len(&self) -> usize {
        self.0.len()
    }
}

unsafe impl<P> Send for WordOffsetBufferW<'_, P> where P: Payload {}

unsafe impl<P> Sync for WordOffsetBufferW<'_, P> where P: Payload {}

fn check_buffer_not_empty<P>(buffer: &[P]) {
    if buffer.is_empty() {
        panic!("The buffer must not be empty.");
    }
}

//
// Safe Transfer implementations
//

fn check_word_offset<P>(buffer: &[*const P])
where
    P: Payload,
{
    if buffer.is_empty() {
        return;
    }

    let mut last_pointer = buffer[0] as *const _ as *const u32;

    for &current_pointer in buffer.iter().skip(1) {
        // Size of u32 is one word / 4 bytes
        let current_pointer = current_pointer as *const u32;
        let expected_pointer = unsafe { last_pointer.add(1) };
        if current_pointer != expected_pointer {
            panic!("The offset must be one word (4 bytes).");
        }

        last_pointer = current_pointer;
    }
}

#[derive(Debug, Clone, Copy)]
pub enum WhichBuffer {
    First,
    Second,
}

impl WhichBuffer {
    pub fn index(self) -> usize {
        match self {
            WhichBuffer::First => 0,
            WhichBuffer::Second => 1,
        }
    }
}
