//! Safe DMA Transfers

use super::stream::{
    BufferModeConf, CircularModeConf, Enabled, FlowControllerConf,
    IsrUncleared, M0a, M1a, MSize, NotM2MConf, PSize, Pa, TransferDirection,
    TransferDirectionConf, TransferModeConf,
};
use super::{ChannelId, Stream};
use crate::private;
use core::convert::TryInto;
use core::fmt::Debug;
use core::{mem, ptr};

pub trait TransferState<'wo>: Send + Sync + private::Sealed {
    type Peripheral: Payload;
    type Memory: Payload;

    fn buffers(&self) -> &Buffers<'wo, Self::Peripheral, Self::Memory>;
    unsafe fn buffers_mut(
        &mut self,
    ) -> &mut Buffers<'wo, Self::Peripheral, Self::Memory>;
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

    fn buffers(&self) -> &Buffers<'wo, Peripheral, Memory> {
        &self.conf.buffers
    }

    unsafe fn buffers_mut(&mut self) -> &mut Buffers<'wo, Peripheral, Memory> {
        self.conf.buffers_mut_unchecked()
    }
}

pub struct Ongoing<'wo, Peripheral, Memory, CXX>
where
    Peripheral: Payload,
    Memory: Payload,
    CXX: ChannelId,
{
    pub(super) stream: Stream<CXX, Enabled, IsrUncleared>,
    pub(super) buffers: Buffers<'wo, Peripheral, Memory>,
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

    fn buffers(&self) -> &Buffers<'wo, Peripheral, Memory> {
        &self.buffers
    }

    unsafe fn buffers_mut(&mut self) -> &mut Buffers<'wo, Peripheral, Memory> {
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

pub enum MemoryBufferType<Memory: Payload> {
    SingleBuffer(SingleBuffer<Memory>),
    DoubleBuffer(DoubleBuffer<Memory>),
}

impl<Memory: Payload> MemoryBufferType<Memory> {
    pub fn is_single_buffer(&self) -> bool {
        self.as_single_buffer().is_some()
    }

    pub fn is_double_buffer(&self) -> bool {
        self.as_double_buffer().is_some()
    }

    pub fn into_single_buffer(self) -> Option<SingleBuffer<Memory>> {
        if let Self::SingleBuffer(buffer) = self {
            Some(buffer)
        } else {
            None
        }
    }

    pub fn as_single_buffer(&self) -> Option<&SingleBuffer<Memory>> {
        if let Self::SingleBuffer(buffer) = self {
            Some(buffer)
        } else {
            None
        }
    }

    pub fn as_single_buffer_mut(
        &mut self,
    ) -> Option<&mut SingleBuffer<Memory>> {
        if let Self::SingleBuffer(buffer) = self {
            Some(buffer)
        } else {
            None
        }
    }

    pub fn into_double_buffer(self) -> Option<DoubleBuffer<Memory>> {
        if let Self::DoubleBuffer(buffer) = self {
            Some(buffer)
        } else {
            None
        }
    }

    pub fn as_double_buffer(&self) -> Option<&DoubleBuffer<Memory>> {
        if let Self::DoubleBuffer(buffer) = self {
            Some(buffer)
        } else {
            None
        }
    }

    pub fn as_double_buffer_mut(
        &mut self,
    ) -> Option<&mut DoubleBuffer<Memory>> {
        if let Self::DoubleBuffer(buffer) = self {
            Some(buffer)
        } else {
            None
        }
    }

    pub fn is_read(&self) -> bool {
        match self {
            MemoryBufferType::SingleBuffer(buffer) => buffer.memory.is_read(),
            MemoryBufferType::DoubleBuffer(buffer) => {
                buffer.memories[0].is_read()
            }
        }
    }

    pub fn is_write(&self) -> bool {
        !self.is_read()
    }

    pub fn m0a(&self) -> &MemoryBuffer<Memory> {
        match self {
            Self::SingleBuffer(buffer) => &buffer.memory,
            Self::DoubleBuffer(buffer) => &buffer.memories()[0],
        }
    }
}

pub struct SingleBuffer<Memory>
where
    Memory: Payload,
{
    pub memory: MemoryBuffer<Memory>,
}

pub struct DoubleBuffer<Memory>
where
    Memory: Payload,
{
    memories: [MemoryBuffer<Memory>; 2],
}

impl<Memory: Payload> DoubleBuffer<Memory> {
    pub fn new(memories: [MemoryBuffer<Memory>; 2]) -> Self {
        let s = Self { memories };

        s.check_self();

        s
    }

    pub fn memories(&self) -> &[MemoryBuffer<Memory>; 2] {
        &self.memories
    }

    /// Exposes a mutable reference to the double buffer inside a closure.
    /// The **same** reference must be returned in the closure, or this method panics.
    /// This prevents the caller to move the reference out of the closure.
    ///
    /// At the end, the double buffer will be checked.
    ///
    /// If the closure is too limiting, consider using the unchecked version.
    pub fn memories_mut<F>(&mut self, op: F)
    where
        for<'a> F: FnOnce(
            &'a mut [MemoryBuffer<Memory>; 2],
        ) -> &'a mut [MemoryBuffer<Memory>; 2],
    {
        let ptr_before = &mut self.memories as *mut _;
        let ptr_after = op(&mut self.memories) as *mut _;

        assert_eq!(ptr_before, ptr_after);

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
        assert_eq!(self.memories[0].is_read(), self.memories[1].is_read());
        assert_eq!(self.memories[0].is_fixed(), self.memories[1].is_fixed());

        if self.memories[0].is_incremented() {
            assert_eq!(
                self.memories[0].as_incremented().unwrap().len(),
                self.memories[1].as_incremented().unwrap().len()
            );
        }
    }
}

pub struct Config<'wo, Peripheral, Memory>
where
    Peripheral: Payload,
    Memory: Payload,
{
    buffers: Buffers<'wo, Peripheral, Memory>,
    stream_config: StreamConfig,
}

impl<'wo, Peripheral, Memory> Config<'wo, Peripheral, Memory>
where
    Peripheral: Payload,
    Memory: Payload,
{
    pub fn new(
        buffers: Buffers<'wo, Peripheral, Memory>,
        stream_config: StreamConfig,
    ) -> Self {
        let s = Self {
            buffers,
            stream_config,
        };

        s.check_self();

        s
    }

    pub fn stream_config(&self) -> StreamConfig {
        self.stream_config
    }

    pub fn transfer_direction(&self) -> TransferDirection {
        if self.buffers.get().peripheral_buffer.is_peripheral() {
            if self.buffers.get().peripheral_buffer.is_read() {
                TransferDirection::P2M
            } else {
                TransferDirection::M2P
            }
        } else {
            TransferDirection::M2M
        }
    }

    pub fn transfer_direction_conf(&self) -> TransferDirectionConf {
        match self.transfer_direction() {
            dir @ TransferDirection::P2M | dir @ TransferDirection::M2P => {
                TransferDirectionConf::NotM2M(NotM2MConf {
                    transfer_dir: dir.try_into().unwrap(),
                    transfer_mode: self.stream_config.transfer_mode,
                    flow: self.stream_config.flow_controller,
                })
            }
            TransferDirection::M2M => {
                if let TransferModeConf::Fifo(fifo_conf) =
                    self.stream_config.transfer_mode
                {
                    TransferDirectionConf::M2M(fifo_conf)
                } else {
                    unreachable!()
                }
            }
        }
    }

    pub fn pa(&self) -> Pa {
        Pa(self.buffers.get().peripheral_buffer.as_ptr(Some(0)) as u32)
    }

    pub fn m0a(&self) -> M0a {
        match &self.buffers.get().memory_buffer {
            MemoryBufferType::SingleBuffer(single) => {
                M0a(single.memory.as_ptr(Some(0)) as u32)
            }
            MemoryBufferType::DoubleBuffer(double) => {
                M0a(double.memories()[0].as_ptr(Some(0)) as u32)
            }
        }
    }

    pub fn m1a(&self) -> Option<M1a> {
        if let MemoryBufferType::DoubleBuffer(buffer) =
            &self.buffers.get().memory_buffer
        {
            Some(M1a(buffer.memories()[1].as_ptr(Some(0)) as u32))
        } else {
            None
        }
    }

    pub fn buffers(&self) -> &Buffers<'wo, Peripheral, Memory> {
        &self.buffers
    }

    pub fn buffers_mut<F>(&mut self, op: F)
    where
        for<'a> F: FnOnce(
            &'a mut Buffers<'wo, Peripheral, Memory>,
        ) -> &'a mut Buffers<'wo, Peripheral, Memory>,
    {
        let ptr_before = &self.buffers as *const _;
        let ptr_after = op(&mut self.buffers) as *const _;

        assert_eq!(ptr_before, ptr_after);

        self.check_self();
    }

    pub unsafe fn buffers_mut_unchecked(
        &mut self,
    ) -> &mut Buffers<'wo, Peripheral, Memory> {
        &mut self.buffers
    }

    pub fn free(self) -> Buffers<'wo, Peripheral, Memory> {
        self.buffers
    }

    fn check_self(&self) {
        if matches!(
            self.stream_config.flow_controller,
            FlowControllerConf::Dma(
                CircularModeConf::Enabled(BufferModeConf::DoubleBuffer(_)),
            )
        ) {
            assert!(self.buffers.get().memory_buffer.is_double_buffer());
        } else {
            assert!(self.buffers.get().memory_buffer.is_single_buffer());
        }

        if self.transfer_direction() == TransferDirection::M2M {
            assert!(matches!(
                self.stream_config.transfer_mode,
                TransferModeConf::Fifo(_)
            ));
            assert!(matches!(
                self.stream_config.flow_controller,
                FlowControllerConf::Dma(CircularModeConf::Disabled)
            ));
        }

        if matches!(self.stream_config.transfer_mode, TransferModeConf::Direct)
        {
            assert_eq!(Peripheral::Size::SIZE, Memory::Size::SIZE);
        }
    }
}

pub struct Buffers<'wo, Peripheral, Memory>
where
    Peripheral: Payload,
    Memory: Payload,
{
    peripheral_buffer: Buffer<'wo, Peripheral>,
    memory_buffer: MemoryBufferType<Memory>,
}

pub struct BuffersView<'a, 'wo, Peripheral, Memory>
where
    Peripheral: Payload,
    Memory: Payload,
{
    pub peripheral_buffer: &'a Buffer<'wo, Peripheral>,
    pub memory_buffer: &'a MemoryBufferType<Memory>,
}

pub struct BuffersViewMut<'a, 'wo, Peripheral, Memory>
where
    Peripheral: Payload,
    Memory: Payload,
{
    pub peripheral_buffer: &'a mut Buffer<'wo, Peripheral>,
    pub memory_buffer: &'a mut MemoryBufferType<Memory>,
}

impl<'wo, Peripheral, Memory> Buffers<'wo, Peripheral, Memory>
where
    Peripheral: Payload,
    Memory: Payload,
{
    /// # Args
    ///
    /// * `peripheral_buffer`: Usually `Buffer::Peripheral`, except for `M2M`-transfers (Memory to Memory), where the source buffer is `Buffer::Memory`.
    /// * `memory_buffer`: The `MemoryBuffer` of the transfer.
    pub fn new(
        peripheral_buffer: Buffer<'wo, Peripheral>,
        memory_buffer: MemoryBufferType<Memory>,
    ) -> Self {
        let s = Self {
            peripheral_buffer,
            memory_buffer,
        };

        s.check_self();

        s
    }

    pub fn get(&self) -> BuffersView<Peripheral, Memory> {
        BuffersView {
            peripheral_buffer: &self.peripheral_buffer,
            memory_buffer: &self.memory_buffer,
        }
    }

    pub fn get_mut<F>(&mut self, op: F)
    where
        for<'a> F: FnOnce(
            BuffersViewMut<'a, 'wo, Peripheral, Memory>,
        )
            -> BuffersViewMut<'a, 'wo, Peripheral, Memory>,
    {
        let peripheral_before = &self.peripheral_buffer as *const _;
        let memory_before = &self.memory_buffer as *const _;

        let view = op(BuffersViewMut {
            peripheral_buffer: &mut self.peripheral_buffer,
            memory_buffer: &mut self.memory_buffer,
        });

        let peripheral_after = view.peripheral_buffer as *const _;
        let memory_after = view.memory_buffer as *const _;

        assert_eq!(peripheral_before, peripheral_after);
        assert_eq!(memory_before, memory_after);

        self.check_self();
    }

    pub unsafe fn get_mut_unchecked(
        &mut self,
    ) -> BuffersViewMut<'_, 'wo, Peripheral, Memory> {
        BuffersViewMut {
            peripheral_buffer: &mut self.peripheral_buffer,
            memory_buffer: &mut self.memory_buffer,
        }
    }

    pub fn free(self) -> (Buffer<'wo, Peripheral>, MemoryBufferType<Memory>) {
        (self.peripheral_buffer, self.memory_buffer)
    }

    fn check_self(&self) {
        assert_ne!(
            self.peripheral_buffer.is_read(),
            self.memory_buffer.is_read()
        );

        if self.peripheral_buffer.is_memory() {
            assert!(self.peripheral_buffer.is_read());
        }
    }
}

#[derive(Clone, Copy)]
pub struct StreamConfig {
    pub transfer_mode: TransferModeConf,
    pub flow_controller: FlowControllerConf,
}

#[derive(Clone, Copy)]
pub enum PayloadPort<Peripheral, Memory>
where
    Peripheral: Payload,
    Memory: Payload,
{
    Peripheral(Peripheral),
    Memory(Memory),
}

impl<Peripheral, Memory> PayloadPort<Peripheral, Memory>
where
    Peripheral: Payload,
    Memory: Payload,
{
    pub fn peripheral(self) -> Peripheral {
        if let PayloadPort::Peripheral(p) = self {
            p
        } else {
            panic!("Tried to unwrap memory port as peripheral port.");
        }
    }

    pub fn memory(self) -> Memory {
        if let PayloadPort::Memory(m) = self {
            m
        } else {
            panic!("Tried to unwrap peripheral port as memory port.");
        }
    }
}

pub enum PointerPort<Peripheral, Memory>
where
    Peripheral: Payload,
    Memory: Payload,
{
    Peripheral(*mut Peripheral),
    Memory(*mut Memory),
}

pub enum Buffer<'wo, P>
where
    P: Payload,
{
    Peripheral(PeripheralBuffer<'wo, P>),
    Memory(MemoryBuffer<P>),
}

impl<'wo, P> Buffer<'wo, P>
where
    P: Payload,
{
    pub fn is_peripheral(&self) -> bool {
        if let Buffer::Peripheral(_) = self {
            true
        } else {
            false
        }
    }

    pub fn is_memory(&self) -> bool {
        if let Buffer::Memory(_) = self {
            true
        } else {
            false
        }
    }

    pub fn into_peripheral(self) -> Option<PeripheralBuffer<'wo, P>> {
        if let Self::Peripheral(buffer) = self {
            Some(buffer)
        } else {
            None
        }
    }

    pub fn as_peripheral(&self) -> Option<&PeripheralBuffer<'wo, P>> {
        if let Self::Peripheral(buffer) = self {
            Some(buffer)
        } else {
            None
        }
    }

    pub fn as_peripheral_mut(
        &mut self,
    ) -> Option<&mut PeripheralBuffer<'wo, P>> {
        if let Self::Peripheral(buffer) = self {
            Some(buffer)
        } else {
            None
        }
    }

    pub fn into_memory(self) -> Option<MemoryBuffer<P>> {
        if let Self::Memory(buffer) = self {
            Some(buffer)
        } else {
            None
        }
    }

    pub fn as_memory(&self) -> Option<&MemoryBuffer<P>> {
        if let Self::Memory(buffer) = self {
            Some(buffer)
        } else {
            None
        }
    }

    pub fn as_memory_mut(&mut self) -> Option<&mut MemoryBuffer<P>> {
        if let Self::Memory(buffer) = self {
            Some(buffer)
        } else {
            None
        }
    }

    pub unsafe fn get(&self, index: Option<usize>) -> P {
        match self {
            Buffer::Peripheral(buffer) => buffer.get(index),
            Buffer::Memory(buffer) => buffer.get(index),
        }
    }

    pub fn as_ptr(&self, index: Option<usize>) -> *const P {
        match self {
            Buffer::Peripheral(buffer) => buffer.as_ptr(index),
            Buffer::Memory(buffer) => buffer.as_ptr(index),
        }
    }

    pub fn is_read(&self) -> bool {
        match self {
            Buffer::Peripheral(buffer) => buffer.is_read(),
            Buffer::Memory(buffer) => buffer.is_read(),
        }
    }

    pub fn is_write(&self) -> bool {
        match self {
            Buffer::Peripheral(buffer) => buffer.is_write(),
            Buffer::Memory(buffer) => buffer.is_write(),
        }
    }

    pub fn is_fixed(&self) -> bool {
        match self {
            Buffer::Peripheral(buffer) => buffer.is_fixed(),
            Buffer::Memory(buffer) => buffer.is_fixed(),
        }
    }

    pub fn is_incremented(&self) -> bool {
        match self {
            Buffer::Peripheral(buffer) => buffer.is_incremented(),
            Buffer::Memory(buffer) => buffer.is_incremented(),
        }
    }
}

pub enum MemoryBuffer<P>
where
    P: Payload,
{
    Fixed(FixedBuffer<P>),
    Incremented(RegularOffsetBuffer<P>),
}

impl<P> MemoryBuffer<P>
where
    P: Payload,
{
    pub fn is_fixed(&self) -> bool {
        if let MemoryBuffer::Fixed(_) = self {
            true
        } else {
            false
        }
    }

    pub fn is_incremented(&self) -> bool {
        if let MemoryBuffer::Incremented(_) = self {
            true
        } else {
            false
        }
    }

    pub fn into_fixed(self) -> Option<FixedBuffer<P>> {
        if let Self::Fixed(buffer) = self {
            Some(buffer)
        } else {
            None
        }
    }

    pub fn as_fixed(&self) -> Option<&FixedBuffer<P>> {
        if let Self::Fixed(buffer) = self {
            Some(buffer)
        } else {
            None
        }
    }

    pub fn as_fixed_mut(&mut self) -> Option<&mut FixedBuffer<P>> {
        if let Self::Fixed(buffer) = self {
            Some(buffer)
        } else {
            None
        }
    }

    pub fn into_incremented(self) -> Option<RegularOffsetBuffer<P>> {
        if let Self::Incremented(buffer) = self {
            Some(buffer)
        } else {
            None
        }
    }

    pub fn as_incremented(&self) -> Option<&RegularOffsetBuffer<P>> {
        if let Self::Incremented(buffer) = self {
            Some(buffer)
        } else {
            None
        }
    }

    pub fn as_incremented_mut(
        &mut self,
    ) -> Option<&mut RegularOffsetBuffer<P>> {
        if let Self::Incremented(buffer) = self {
            Some(buffer)
        } else {
            None
        }
    }

    pub unsafe fn get(&self, index: Option<usize>) -> P {
        match self {
            MemoryBuffer::Fixed(buffer) => buffer.get(),
            MemoryBuffer::Incremented(buffer) => buffer.get(index.unwrap()),
        }
    }

    pub fn as_ptr(&self, index: Option<usize>) -> *const P {
        match self {
            MemoryBuffer::Fixed(buffer) => buffer.as_ptr(),
            MemoryBuffer::Incremented(buffer) => buffer.as_ptr(index.unwrap()),
        }
    }

    pub fn is_read(&self) -> bool {
        match self {
            MemoryBuffer::Fixed(buffer) => buffer.is_read(),
            MemoryBuffer::Incremented(buffer) => buffer.is_read(),
        }
    }

    pub fn is_write(&self) -> bool {
        match self {
            MemoryBuffer::Fixed(buffer) => buffer.is_write(),
            MemoryBuffer::Incremented(buffer) => buffer.is_write(),
        }
    }
}

pub enum PeripheralBuffer<'wo, P>
where
    P: Payload,
{
    Fixed(FixedBuffer<P>),
    Incremented(IncrementedBuffer<'wo, P>),
}

impl<'wo, P> PeripheralBuffer<'wo, P>
where
    P: Payload,
{
    pub fn is_fixed(&self) -> bool {
        if let PeripheralBuffer::Fixed(_) = self {
            true
        } else {
            false
        }
    }

    pub fn is_incremented(&self) -> bool {
        if let PeripheralBuffer::Incremented(_) = self {
            true
        } else {
            false
        }
    }

    pub fn into_fixed(self) -> Option<FixedBuffer<P>> {
        if let Self::Fixed(buffer) = self {
            Some(buffer)
        } else {
            None
        }
    }

    pub fn as_fixed(&self) -> Option<&FixedBuffer<P>> {
        if let Self::Fixed(buffer) = self {
            Some(buffer)
        } else {
            None
        }
    }

    pub fn as_fixed_mut(&mut self) -> Option<&mut FixedBuffer<P>> {
        if let Self::Fixed(buffer) = self {
            Some(buffer)
        } else {
            None
        }
    }

    pub fn into_incremented(self) -> Option<IncrementedBuffer<'wo, P>> {
        if let Self::Incremented(buffer) = self {
            Some(buffer)
        } else {
            None
        }
    }

    pub fn as_incremented(&self) -> Option<&IncrementedBuffer<'wo, P>> {
        if let Self::Incremented(buffer) = self {
            Some(buffer)
        } else {
            None
        }
    }

    pub fn as_incremented_mut(
        &mut self,
    ) -> Option<&mut IncrementedBuffer<'wo, P>> {
        if let Self::Incremented(buffer) = self {
            Some(buffer)
        } else {
            None
        }
    }

    pub unsafe fn get(&self, index: Option<usize>) -> P {
        match self {
            PeripheralBuffer::Fixed(buffer) => buffer.get(),
            PeripheralBuffer::Incremented(buffer) => buffer.get(index.unwrap()),
        }
    }

    pub fn as_ptr(&self, index: Option<usize>) -> *const P {
        match self {
            PeripheralBuffer::Fixed(buffer) => buffer.as_ptr(),
            PeripheralBuffer::Incremented(buffer) => {
                buffer.as_ptr(index.unwrap())
            }
        }
    }

    pub fn is_read(&self) -> bool {
        match self {
            PeripheralBuffer::Fixed(buffer) => buffer.is_read(),
            PeripheralBuffer::Incremented(buffer) => buffer.is_read(),
        }
    }

    pub fn is_write(&self) -> bool {
        match self {
            PeripheralBuffer::Fixed(buffer) => buffer.is_write(),
            PeripheralBuffer::Incremented(buffer) => buffer.is_write(),
        }
    }
}

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

    pub fn into_regular_offset(self) -> Option<RegularOffsetBuffer<P>> {
        if let Self::RegularOffset(buffer) = self {
            Some(buffer)
        } else {
            None
        }
    }

    pub fn as_regular_offset(&self) -> Option<&RegularOffsetBuffer<P>> {
        if let Self::RegularOffset(buffer) = self {
            Some(buffer)
        } else {
            None
        }
    }

    pub fn as_regular_offset_mut(
        &mut self,
    ) -> Option<&mut RegularOffsetBuffer<P>> {
        if let Self::RegularOffset(buffer) = self {
            Some(buffer)
        } else {
            None
        }
    }

    pub fn into_word_offset(self) -> Option<WordOffsetBuffer<'wo, P>> {
        if let Self::WordOffset(buffer) = self {
            Some(buffer)
        } else {
            None
        }
    }

    pub fn as_word_offset(&self) -> Option<&WordOffsetBuffer<'wo, P>> {
        if let Self::WordOffset(buffer) = self {
            Some(buffer)
        } else {
            None
        }
    }

    pub fn as_word_offset_mut(
        &mut self,
    ) -> Option<&mut WordOffsetBuffer<'wo, P>> {
        if let Self::WordOffset(buffer) = self {
            Some(buffer)
        } else {
            None
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

    pub fn into_read(self) -> Option<FixedBufferR<P>> {
        if let Self::Read(buffer) = self {
            Some(buffer)
        } else {
            None
        }
    }

    pub fn as_read(&self) -> Option<&FixedBufferR<P>> {
        if let Self::Read(buffer) = self {
            Some(buffer)
        } else {
            None
        }
    }

    pub fn as_read_mut(&mut self) -> Option<&mut FixedBufferR<P>> {
        if let Self::Read(buffer) = self {
            Some(buffer)
        } else {
            None
        }
    }

    pub fn into_write(self) -> Option<FixedBufferW<P>> {
        if let Self::Write(buffer) = self {
            Some(buffer)
        } else {
            None
        }
    }

    pub fn as_write(&self) -> Option<&FixedBufferW<P>> {
        if let Self::Write(buffer) = self {
            Some(buffer)
        } else {
            None
        }
    }

    pub fn as_write_mut(&mut self) -> Option<&mut FixedBufferW<P>> {
        if let Self::Write(buffer) = self {
            Some(buffer)
        } else {
            None
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

#[derive(Clone, Copy)]
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

    pub fn into_read(self) -> Option<RegularOffsetBufferR<P>> {
        if let RegularOffsetBuffer::Read(buffer) = self {
            Some(buffer)
        } else {
            None
        }
    }

    pub fn as_read(&self) -> Option<&RegularOffsetBufferR<P>> {
        if let RegularOffsetBuffer::Read(buffer) = self {
            Some(buffer)
        } else {
            None
        }
    }

    pub fn as_read_mut(&mut self) -> Option<&mut RegularOffsetBufferR<P>> {
        if let RegularOffsetBuffer::Read(buffer) = self {
            Some(buffer)
        } else {
            None
        }
    }

    pub fn into_write(self) -> Option<RegularOffsetBufferW<P>> {
        if let RegularOffsetBuffer::Write(buffer) = self {
            Some(buffer)
        } else {
            None
        }
    }

    pub fn as_write(&self) -> Option<&RegularOffsetBufferW<P>> {
        if let RegularOffsetBuffer::Write(buffer) = self {
            Some(buffer)
        } else {
            None
        }
    }

    pub fn as_write_mut(&mut self) -> Option<&mut RegularOffsetBufferW<P>> {
        if let RegularOffsetBuffer::Write(buffer) = self {
            Some(buffer)
        } else {
            None
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

#[derive(Clone, Copy)]
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

    pub fn as_read(&self) -> WordOffsetBufferR<'wo, P> {
        if let &WordOffsetBuffer::Read(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a write buffer.");
        }
    }

    pub fn into_write(self) -> WordOffsetBufferW<'wo, P> {
        if let WordOffsetBuffer::Write(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a read buffer.");
        }
    }

    pub fn as_write(&self) -> &WordOffsetBufferW<'wo, P> {
        if let WordOffsetBuffer::Write(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a read buffer.");
        }
    }

    pub fn as_mut_write(&mut self) -> &mut WordOffsetBufferW<'wo, P> {
        if let WordOffsetBuffer::Write(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a read buffer.");
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

#[derive(Clone, Copy)]
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

#[derive(Clone, Copy)]
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
