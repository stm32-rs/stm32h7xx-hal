//! Safe DMA Transfers

use super::stream::{
    BufferMode, CircularMode, Disabled, Enabled, IsrCleared, IsrUncleared, M0a,
    MSize, Minc, Ndt, PSize, Pa, Pinc, Pincos, TransferDirection, TransferMode,
};
use super::{ChannelId, Stream};
use crate::private;
use core::convert::TryFrom;
use core::fmt::Debug;
use core::marker::PhantomData;
use core::{mem, ptr};

pub trait TransferState: Send + Sync + private::Sealed {}

pub struct Start;
impl private::Sealed for Start {}
impl TransferState for Start {}

pub struct Ongoing<CXX>
where
    CXX: ChannelId,
{
    pub(super) stream: Stream<CXX, Enabled, IsrUncleared>,
}

impl<CXX> private::Sealed for Ongoing<CXX> where CXX: ChannelId {}

impl<CXX> TransferState for Ongoing<CXX> where CXX: ChannelId {}

/// # Safety
///
/// * `Self` must be valid for any bit representation
/// * `Self::Size` must be
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
    pub fn from_payload<P>() -> Self
    where
        P: Payload,
    {
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

pub trait IMemoryBuffer {
    type Memory: Payload;

    const BUFFER_MODE: BufferMode;

    fn first(&self) -> &MemoryBuffer<Self::Memory>;
}

pub struct SingleBuffer<'buf, Memory>
where
    Memory: Payload,
{
    pub memory: MemoryBuffer<'buf, Memory>,
}

impl<Memory> IMemoryBuffer for SingleBuffer<'_, Memory>
where
    Memory: Payload,
{
    type Memory = Memory;

    const BUFFER_MODE: BufferMode = BufferMode::Regular;

    fn first(&self) -> &MemoryBuffer<Memory> {
        &self.memory
    }
}

pub struct DoubleBuffer<'buf0, 'buf1, Memory>
where
    Memory: Payload,
{
    pub memory_0: MemoryBuffer<'buf0, Memory>,
    pub memory_1: MemoryBuffer<'buf1, Memory>,
}

impl<Memory> IMemoryBuffer for DoubleBuffer<'_, '_, Memory>
where
    Memory: Payload,
{
    type Memory = Memory;

    const BUFFER_MODE: BufferMode = BufferMode::DoubleBuffer;

    fn first(&self) -> &MemoryBuffer<Memory> {
        &self.memory_0
    }
}

pub trait IBuffers<'p_buf, 'wo> {
    type Peripheral: Payload;
    type MemoryBuffer: IMemoryBuffer;

    const TRANSFER_MODE: TransferMode;

    fn buffers(
        self,
    ) -> Buffers<'p_buf, 'wo, Self::Peripheral, Self::MemoryBuffer>;

    fn buffers_ref(
        &self,
    ) -> BuffersRef<'_, 'p_buf, 'wo, Self::Peripheral, Self::MemoryBuffer>;
}

pub struct Buffers<'p_buf, 'wo, Peripheral, MemoryBuffer>
where
    Peripheral: Payload,
    MemoryBuffer: IMemoryBuffer,
{
    pub peripheral_buffer: PeripheralBuffer<'p_buf, 'wo, Peripheral>,
    pub memory_buffer: MemoryBuffer,
}

pub struct BuffersRef<'r, 'p_buf, 'wo, Peripheral, MemoryBuffer>
where
    Peripheral: Payload,
    MemoryBuffer: IMemoryBuffer,
{
    pub peripheral_buffer: &'r PeripheralBuffer<'p_buf, 'wo, Peripheral>,
    pub memory_buffer: &'r MemoryBuffer,
}
pub struct FifoBuffers<'p_buf, 'wo, Peripheral, MemoryBuffer>
where
    Peripheral: Payload,
    MemoryBuffer: IMemoryBuffer,
{
    peripheral_buffer: PeripheralBuffer<'p_buf, 'wo, Peripheral>,
    memory_buffer: MemoryBuffer,
}

impl<'p_buf, 'wo, Peripheral, MemoryBuffer>
    FifoBuffers<'p_buf, 'wo, Peripheral, MemoryBuffer>
where
    Peripheral: Payload,
    MemoryBuffer: IMemoryBuffer,
{
    pub fn new(
        peripheral: PeripheralBuffer<'p_buf, 'wo, Peripheral>,
        memory_buffer: MemoryBuffer,
    ) -> Self {
        assert_ne!(peripheral.is_read(), memory_buffer.first().is_read());

        Self {
            peripheral_buffer: peripheral,
            memory_buffer,
        }
    }
}

impl<'p_buf, 'wo, Peripheral, MemoryBuffer> IBuffers<'p_buf, 'wo>
    for FifoBuffers<'p_buf, 'wo, Peripheral, MemoryBuffer>
where
    Peripheral: Payload,
    MemoryBuffer: IMemoryBuffer,
{
    type Peripheral = Peripheral;
    type MemoryBuffer = MemoryBuffer;

    const TRANSFER_MODE: TransferMode = TransferMode::Fifo;

    fn buffers(self) -> Buffers<'p_buf, 'wo, Peripheral, MemoryBuffer> {
        Buffers {
            peripheral_buffer: self.peripheral_buffer,
            memory_buffer: self.memory_buffer,
        }
    }

    fn buffers_ref(
        &self,
    ) -> BuffersRef<'_, 'p_buf, 'wo, Peripheral, MemoryBuffer> {
        BuffersRef {
            peripheral_buffer: &self.peripheral_buffer,
            memory_buffer: &self.memory_buffer,
        }
    }
}

pub struct DirectBuffers<'p_buf, 'wo, Peripheral, MemoryBuffer>
where
    Peripheral: Payload<Size = <MemoryBuffer::Memory as Payload>::Size>,
    MemoryBuffer: IMemoryBuffer,
{
    peripheral_buffer: PeripheralBuffer<'p_buf, 'wo, Peripheral>,
    memory_buffer: MemoryBuffer,
}

impl<'p_buf, 'wo, Peripheral, Memory, MemoryBuffer>
    DirectBuffers<'p_buf, 'wo, Peripheral, MemoryBuffer>
where
    Peripheral: Payload,
    Memory: Payload<Size = Peripheral::Size>,
    MemoryBuffer: IMemoryBuffer<Memory = Memory>,
{
    pub fn new(
        peripheral: PeripheralBuffer<'p_buf, 'wo, Peripheral>,
        memory_buffer: MemoryBuffer,
    ) -> Self {
        assert_ne!(peripheral.is_read(), memory_buffer.first().is_read());

        Self {
            peripheral_buffer: peripheral,
            memory_buffer,
        }
    }
}

impl<'p_buf, 'wo, Peripheral, Memory, MemoryBuffer> IBuffers<'p_buf, 'wo>
    for DirectBuffers<'p_buf, 'wo, Peripheral, MemoryBuffer>
where
    Peripheral: Payload,
    Memory: Payload<Size = Peripheral::Size>,
    MemoryBuffer: IMemoryBuffer<Memory = Memory>,
{
    type Peripheral = Peripheral;
    type MemoryBuffer = MemoryBuffer;

    const TRANSFER_MODE: TransferMode = TransferMode::Direct;

    fn buffers(self) -> Buffers<'p_buf, 'wo, Peripheral, MemoryBuffer> {
        Buffers {
            peripheral_buffer: self.peripheral_buffer,
            memory_buffer: self.memory_buffer,
        }
    }

    fn buffers_ref(
        &self,
    ) -> BuffersRef<'_, 'p_buf, 'wo, Peripheral, MemoryBuffer> {
        BuffersRef {
            peripheral_buffer: &self.peripheral_buffer,
            memory_buffer: &self.memory_buffer,
        }
    }
}

pub trait ITransferDirection<'p_buf, 'wo> {
    type Buffers: IBuffers<'p_buf, 'wo>;

    const TRANSFER_DIRECTION: TransferDirection;

    fn buffers(self) -> Self::Buffers;
}

pub struct PeripheralToMemory<BUFFERS>
where
    for<'p_buf, 'wo> BUFFERS: IBuffers<'p_buf, 'wo>,
{
    buffers: BUFFERS,
}

impl<BUFFERS> PeripheralToMemory<BUFFERS>
where
    for<'p_buf, 'wo> BUFFERS: IBuffers<'p_buf, 'wo>,
{
    pub fn new(buffers: BUFFERS) -> Self {
        assert!(buffers.buffers_ref().peripheral_buffer.is_read());
        assert!(buffers.buffers_ref().memory_buffer.first().is_write());

        Self { buffers }
    }
}

impl<'p_buf, 'wo, BUFFERS> ITransferDirection<'p_buf, 'wo>
    for PeripheralToMemory<BUFFERS>
where
    for<'p_buf1, 'wo1> BUFFERS: IBuffers<'p_buf1, 'wo1>,
{
    type Buffers = BUFFERS;

    const TRANSFER_DIRECTION: TransferDirection = TransferDirection::P2M;

    fn buffers(self) -> BUFFERS {
        self.buffers
    }
}

pub struct MemoryToPeripheral<BUFFERS>
where
    for<'p_buf, 'wo> BUFFERS: IBuffers<'p_buf, 'wo>,
{
    buffers: BUFFERS,
}

impl<BUFFERS> MemoryToPeripheral<BUFFERS>
where
    for<'p_buf, 'wo> BUFFERS: IBuffers<'p_buf, 'wo>,
{
    pub fn new(buffers: BUFFERS) -> Self {
        assert!(buffers.buffers_ref().memory_buffer.first().is_read());
        assert!(buffers.buffers_ref().peripheral_buffer.is_write());

        Self { buffers }
    }
}

impl<'p_buf, 'wo, BUFFERS> ITransferDirection<'p_buf, 'wo>
    for MemoryToPeripheral<BUFFERS>
where
    for<'p_buf1, 'wo1> BUFFERS: IBuffers<'p_buf1, 'wo1>,
{
    type Buffers = BUFFERS;

    const TRANSFER_DIRECTION: TransferDirection = TransferDirection::M2P;

    fn buffers(self) -> BUFFERS {
        self.buffers
    }
}

pub struct MemoryToMemory<'m_buf, BUFFERS, Memory>
where
    for<'p_buf, 'wo> BUFFERS:
        IBuffers<'p_buf, 'wo, MemoryBuffer = SingleBuffer<'m_buf, Memory>>,
    Memory: Payload,
{
    buffers: BUFFERS,
    _phantom: PhantomData<&'m_buf Memory>,
}

impl<'m_buf, BUFFERS, Memory> MemoryToMemory<'m_buf, BUFFERS, Memory>
where
    for<'p_buf, 'wo> BUFFERS:
        IBuffers<'p_buf, 'wo, MemoryBuffer = SingleBuffer<'m_buf, Memory>>,
    Memory: Payload,
{
    pub fn new(buffers: BUFFERS) -> Self {
        assert!(buffers.buffers_ref().peripheral_buffer.is_read());
        assert!(buffers.buffers_ref().memory_buffer.memory.is_write());

        Self {
            buffers,
            _phantom: PhantomData,
        }
    }
}

impl<'p_buf, 'm_buf, 'wo, BUFFERS, Memory> ITransferDirection<'p_buf, 'wo>
    for MemoryToMemory<'m_buf, BUFFERS, Memory>
where
    for<'p_buf1, 'wo1> BUFFERS:
        IBuffers<'p_buf1, 'wo1, MemoryBuffer = SingleBuffer<'m_buf, Memory>>,
    Memory: Payload,
{
    type Buffers = BUFFERS;

    const TRANSFER_DIRECTION: TransferDirection = TransferDirection::M2M;

    fn buffers(self) -> BUFFERS {
        self.buffers
    }
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

pub enum Buffer<'buf, 'wo, P>
where
    P: Payload,
{
    Peripheral(PeripheralBuffer<'buf, 'wo, P>),
    Memory(MemoryBuffer<'buf, P>),
}

impl<'buf, 'wo, P> Buffer<'buf, 'wo, P>
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

    pub fn into_peripheral(self) -> PeripheralBuffer<'buf, 'wo, P> {
        if let Buffer::Peripheral(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a memory buffer.");
        }
    }

    pub fn as_peripheral(&self) -> &PeripheralBuffer<'buf, 'wo, P> {
        if let Buffer::Peripheral(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a memory buffer.");
        }
    }

    pub fn as_mut_peripheral(&mut self) -> &mut PeripheralBuffer<'buf, 'wo, P> {
        if let Buffer::Peripheral(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a memory buffer.");
        }
    }

    pub fn into_memory(self) -> MemoryBuffer<'buf, P> {
        if let Buffer::Memory(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a peripheral puffer.");
        }
    }

    pub fn as_memory(&self) -> &MemoryBuffer<'buf, P> {
        if let Buffer::Memory(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a peripheral puffer.");
        }
    }

    pub fn as_mut_memory(&mut self) -> &mut MemoryBuffer<'buf, P> {
        if let Buffer::Memory(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a peripheral puffer.");
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

pub type BufferStatic<'wo, P> = Buffer<'static, 'wo, P>;

pub enum MemoryBuffer<'buf, P>
where
    P: Payload,
{
    Fixed(FixedBuffer<'buf, P>),
    Incremented(RegularOffsetBuffer<'buf, P>),
}

impl<'buf, P> MemoryBuffer<'buf, P>
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

    pub fn into_fixed(self) -> FixedBuffer<'buf, P> {
        if let MemoryBuffer::Fixed(buffer) = self {
            buffer
        } else {
            panic!("The buffer is incremented.");
        }
    }

    pub fn as_fixed(&self) -> &FixedBuffer<'buf, P> {
        if let MemoryBuffer::Fixed(buffer) = self {
            buffer
        } else {
            panic!("The buffer is incremented.");
        }
    }

    pub fn as_mut_fixed(&mut self) -> &mut FixedBuffer<'buf, P> {
        if let MemoryBuffer::Fixed(buffer) = self {
            buffer
        } else {
            panic!("The buffer is incremented.");
        }
    }

    pub fn into_incremented(self) -> RegularOffsetBuffer<'buf, P> {
        if let MemoryBuffer::Incremented(buffer) = self {
            buffer
        } else {
            panic!("The buffer is fixed.");
        }
    }

    pub fn as_incremented(&self) -> &RegularOffsetBuffer<'buf, P> {
        if let MemoryBuffer::Incremented(buffer) = self {
            buffer
        } else {
            panic!("The buffer is fixed.");
        }
    }

    pub fn as_mut_incremented(&mut self) -> &mut RegularOffsetBuffer<'buf, P> {
        if let MemoryBuffer::Incremented(buffer) = self {
            buffer
        } else {
            panic!("The buffer is fixed.");
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

pub type MemoryBufferStatic<P> = MemoryBuffer<'static, P>;

pub enum PeripheralBuffer<'buf, 'wo, P>
where
    P: Payload,
{
    Fixed(FixedBuffer<'buf, P>),
    Incremented(IncrementedBuffer<'buf, 'wo, P>),
}

impl<'buf, 'wo, P> PeripheralBuffer<'buf, 'wo, P>
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

    pub fn into_fixed(self) -> FixedBuffer<'buf, P> {
        if let PeripheralBuffer::Fixed(buffer) = self {
            buffer
        } else {
            panic!("The buffer is incremented.");
        }
    }

    pub fn as_fixed(&self) -> &FixedBuffer<'buf, P> {
        if let PeripheralBuffer::Fixed(buffer) = self {
            buffer
        } else {
            panic!("The buffer is incremented.");
        }
    }

    pub fn as_mut_fixed(&mut self) -> &mut FixedBuffer<'buf, P> {
        if let PeripheralBuffer::Fixed(buffer) = self {
            buffer
        } else {
            panic!("The buffer is incremented.");
        }
    }

    pub fn into_incremented(self) -> IncrementedBuffer<'buf, 'wo, P> {
        if let PeripheralBuffer::Incremented(buffer) = self {
            buffer
        } else {
            panic!("The buffer is fixed.");
        }
    }

    pub fn as_incremented(&self) -> &IncrementedBuffer<'buf, 'wo, P> {
        if let PeripheralBuffer::Incremented(buffer) = self {
            buffer
        } else {
            panic!("The buffer is fixed.");
        }
    }

    pub fn as_mut_incremented(
        &mut self,
    ) -> &mut IncrementedBuffer<'buf, 'wo, P> {
        if let PeripheralBuffer::Incremented(buffer) = self {
            buffer
        } else {
            panic!("The buffer is fixed.");
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

pub type PeripheralBufferStatic<'wo, P> = PeripheralBuffer<'static, 'wo, P>;

pub enum IncrementedBuffer<'buf, 'wo, P>
where
    P: Payload,
{
    RegularOffset(RegularOffsetBuffer<'buf, P>),
    WordOffset(WordOffsetBuffer<'buf, 'wo, P>),
}

impl<'buf, 'wo, P> IncrementedBuffer<'buf, 'wo, P>
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

    pub fn into_regular_offset(self) -> RegularOffsetBuffer<'buf, P> {
        if let IncrementedBuffer::RegularOffset(buffer) = self {
            buffer
        } else {
            panic!("The buffer has word offset.");
        }
    }

    pub fn as_regular_offset(&self) -> &RegularOffsetBuffer<'buf, P> {
        if let IncrementedBuffer::RegularOffset(buffer) = self {
            buffer
        } else {
            panic!("The buffer has word offset.");
        }
    }

    pub fn as_mut_regular_offset(
        &mut self,
    ) -> &mut RegularOffsetBuffer<'buf, P> {
        if let IncrementedBuffer::RegularOffset(buffer) = self {
            buffer
        } else {
            panic!("The buffer has word offset.");
        }
    }

    pub fn into_word_offset(self) -> WordOffsetBuffer<'buf, 'wo, P> {
        if let IncrementedBuffer::WordOffset(buffer) = self {
            buffer
        } else {
            panic!("The buffer has regular offset.");
        }
    }

    pub fn as_word_offset(&self) -> &WordOffsetBuffer<'buf, 'wo, P> {
        if let IncrementedBuffer::WordOffset(buffer) = self {
            buffer
        } else {
            panic!("The buffer has regular offset.");
        }
    }

    pub fn as_mut_word_offset(
        &mut self,
    ) -> &mut WordOffsetBuffer<'buf, 'wo, P> {
        if let IncrementedBuffer::WordOffset(buffer) = self {
            buffer
        } else {
            panic!("The buffer has regular offset.");
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

pub type IncrementedBufferStatic<'wo, P> = IncrementedBuffer<'static, 'wo, P>;

pub enum FixedBuffer<'buf, P>
where
    P: Payload,
{
    Read(FixedBufferR<'buf, P>),
    Write(FixedBufferW<'buf, P>),
}

impl<'buf, P> FixedBuffer<'buf, P>
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

    pub fn as_read(&self) -> FixedBufferR<'buf, P> {
        if let &FixedBuffer::Read(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a write buffer.");
        }
    }

    pub fn into_write(self) -> FixedBufferW<'buf, P> {
        if let FixedBuffer::Write(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a read buffer.");
        }
    }

    pub fn as_write(&self) -> &FixedBufferW<'buf, P> {
        if let FixedBuffer::Write(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a read buffer.");
        }
    }

    pub fn as_mut_write(&mut self) -> &mut FixedBufferW<'buf, P> {
        if let FixedBuffer::Write(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a read buffer.");
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

pub type FixedBufferStatic<P> = FixedBuffer<'static, P>;

#[derive(Clone, Copy)]
pub struct FixedBufferR<'buf, P>(*const P, PhantomData<&'buf P>)
where
    P: Payload;

impl<'buf, P> FixedBufferR<'buf, P>
where
    P: Payload,
{
    pub fn new(buffer: &'buf P) -> Self {
        FixedBufferR(buffer, PhantomData)
    }

    pub fn get(self) -> P {
        unsafe { self.0.read_volatile() }
    }

    pub fn as_ptr(self) -> *const P {
        self.0
    }
}

unsafe impl<P> Send for FixedBufferR<'_, P> where P: Payload {}

unsafe impl<P> Sync for FixedBufferR<'_, P> where P: Payload {}

pub type FixedBufferRStatic<P> = FixedBufferR<'static, P>;

pub struct FixedBufferW<'buf, P>(*mut P, PhantomData<&'buf mut P>)
where
    P: Payload;

impl<'buf, P> FixedBufferW<'buf, P>
where
    P: Payload,
{
    pub fn new(buffer: &'buf mut P) -> Self {
        FixedBufferW(buffer, PhantomData)
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

unsafe impl<P> Send for FixedBufferW<'_, P> where P: Payload {}

unsafe impl<P> Sync for FixedBufferW<'_, P> where P: Payload {}

pub type FixedBufferWStatic<P> = FixedBufferW<'static, P>;

pub enum RegularOffsetBuffer<'buf, P>
where
    P: Payload,
{
    Read(RegularOffsetBufferR<'buf, P>),
    Write(RegularOffsetBufferW<'buf, P>),
}

impl<'buf, P> RegularOffsetBuffer<'buf, P>
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

    pub fn as_read(&self) -> RegularOffsetBufferR<'buf, P> {
        if let &RegularOffsetBuffer::Read(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a write buffer.");
        }
    }

    pub fn into_write(self) -> RegularOffsetBufferW<'buf, P> {
        if let RegularOffsetBuffer::Write(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a read buffer.");
        }
    }

    pub fn as_write(&self) -> &RegularOffsetBufferW<'buf, P> {
        if let RegularOffsetBuffer::Write(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a read buffer.");
        }
    }

    pub fn as_mut_write(&mut self) -> &mut RegularOffsetBufferW<'buf, P> {
        if let RegularOffsetBuffer::Write(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a read buffer.");
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

pub type RegularOffsetBufferStatic<P> = RegularOffsetBuffer<'static, P>;

#[derive(Clone, Copy)]
pub struct RegularOffsetBufferR<'buf, P>(*const [P], PhantomData<&'buf P>)
where
    P: Payload;

#[allow(clippy::len_without_is_empty)]
impl<'buf, P> RegularOffsetBufferR<'buf, P>
where
    P: Payload,
{
    pub fn new(buffer: &'buf [P]) -> Self {
        check_buffer_not_empty(buffer);

        RegularOffsetBufferR(buffer, PhantomData)
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

unsafe impl<P> Send for RegularOffsetBufferR<'_, P> where P: Payload {}

unsafe impl<P> Sync for RegularOffsetBufferR<'_, P> where P: Payload {}

pub type RegularOffsetBufferRStatic<P> = RegularOffsetBufferR<'static, P>;

pub struct RegularOffsetBufferW<'buf, P>(*mut [P], PhantomData<&'buf mut P>)
where
    P: Payload;

#[allow(clippy::len_without_is_empty)]
impl<'buf, P> RegularOffsetBufferW<'buf, P>
where
    P: Payload,
{
    pub fn new(buffer: &'buf mut [P]) -> Self {
        check_buffer_not_empty(buffer);

        RegularOffsetBufferW(buffer, PhantomData)
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

unsafe impl<P> Send for RegularOffsetBufferW<'_, P> where P: Payload {}

unsafe impl<P> Sync for RegularOffsetBufferW<'_, P> where P: Payload {}

pub type RegularOffsetBufferWStatic<P> = RegularOffsetBufferW<'static, P>;

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

pub enum WordOffsetBuffer<'buf, 'wo, P>
where
    P: Payload,
{
    Read(WordOffsetBufferR<'buf, 'wo, P>),
    Write(WordOffsetBufferW<'buf, 'wo, P>),
}

impl<'buf, 'wo, P> WordOffsetBuffer<'buf, 'wo, P>
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

    pub fn as_read(&self) -> WordOffsetBufferR<'buf, 'wo, P> {
        if let &WordOffsetBuffer::Read(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a write buffer.");
        }
    }

    pub fn into_write(self) -> WordOffsetBufferW<'buf, 'wo, P> {
        if let WordOffsetBuffer::Write(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a read buffer.");
        }
    }

    pub fn as_write(&self) -> &WordOffsetBufferW<'buf, 'wo, P> {
        if let WordOffsetBuffer::Write(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a read buffer.");
        }
    }

    pub fn as_mut_write(&mut self) -> &mut WordOffsetBufferW<'buf, 'wo, P> {
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

pub type WordOffsetBufferStatic<'wo, P> = WordOffsetBuffer<'static, 'wo, P>;

#[derive(Clone, Copy)]
pub struct WordOffsetBufferR<'buf, 'wo, P>(
    &'wo [*const P],
    PhantomData<&'buf P>,
)
where
    P: Payload;

#[allow(clippy::len_without_is_empty)]
impl<'buf, 'wo, P> WordOffsetBufferR<'buf, 'wo, P>
where
    P: Payload,
{
    pub fn new(buffer: &'wo [&'buf P]) -> Self {
        check_buffer_not_empty(buffer);

        let buffer = unsafe { &*(buffer as *const _ as *const _) };

        check_word_offset(buffer);

        WordOffsetBufferR(buffer, PhantomData)
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

unsafe impl<'buf, 'wo, P> Send for WordOffsetBufferR<'buf, 'wo, P> where
    P: Payload
{
}

unsafe impl<'buf, 'wo, P> Sync for WordOffsetBufferR<'buf, 'wo, P> where
    P: Payload
{
}

pub type WordOffsetBufferRStatic<'wo, P> = WordOffsetBufferR<'static, 'wo, P>;

pub struct WordOffsetBufferW<'buf, 'wo, P>(
    &'wo mut [*mut P],
    PhantomData<&'buf mut P>,
)
where
    P: Payload;

#[allow(clippy::len_without_is_empty)]
impl<'buf, 'wo, P> WordOffsetBufferW<'buf, 'wo, P>
where
    P: Payload,
{
    pub fn new(buffer: &'wo mut [&'buf mut P]) -> Self {
        check_buffer_not_empty(buffer);

        unsafe {
            check_word_offset::<P>(&*(buffer as *const _ as *const _));

            WordOffsetBufferW(&mut *(buffer as *mut _ as *mut _), PhantomData)
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

unsafe impl<P> Send for WordOffsetBufferW<'_, '_, P> where P: Payload {}

unsafe impl<P> Sync for WordOffsetBufferW<'_, '_, P> where P: Payload {}

pub type WordOffsetBufferWStatic<'wo, P> = WordOffsetBufferW<'static, 'wo, P>;

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

pub(super) fn configure_safe_transfer<CXX, Peripheral, Memory>(
    stream: &mut Stream<CXX, Disabled, IsrCleared>,
    peripheral: &PeripheralBuffer<Peripheral>,
    memory: &MemoryBuffer<Memory>,
) where
    CXX: ChannelId,
    Peripheral: Payload,
    Memory: Payload,
{
    let p_size = PayloadSize::from_payload::<Peripheral>();
    let m_size = PayloadSize::from_payload::<Memory>();

    if stream.transfer_mode() == TransferMode::Direct && p_size != m_size {
        panic!("The buffer sizes must match if the stream is configured in direct mode.");
    }

    stream.set_p_size(p_size.into());
    stream.set_m_size(m_size.into());

    match peripheral {
        PeripheralBuffer::Fixed(buffer) => {
            stream.set_pa(Pa(buffer.as_ptr() as u32));
            stream.set_pinc(Pinc::Fixed);
        }
        PeripheralBuffer::Incremented(buffer) => match buffer {
            IncrementedBuffer::RegularOffset(buffer) => {
                stream.set_pa(Pa(buffer.as_ptr(0) as u32));
                stream.set_pinc(Pinc::Incremented);
                stream.set_pincos(Pincos::PSize);
            }
            IncrementedBuffer::WordOffset(buffer) => {
                stream.set_pa(Pa(buffer.as_ptr(0) as u32));
                stream.set_pinc(Pinc::Incremented);
                stream.set_pincos(Pincos::Word);
            }
        },
    }

    match memory {
        MemoryBuffer::Fixed(buffer) => {
            stream.set_m0a(M0a(buffer.as_ptr() as u32));
            stream.set_minc(Minc::Fixed);
        }
        MemoryBuffer::Incremented(buffer) => {
            stream.set_m0a(M0a(buffer.as_ptr(0) as u32));
            stream.set_minc(Minc::Incremented);
        }
    }

    if stream.circular_mode() == CircularMode::Enabled {
        if peripheral.is_write() {
            stream.set_transfer_direction(TransferDirection::M2P);
        } else {
            stream.set_transfer_direction(TransferDirection::P2M);
        }
    } else if peripheral.is_write() {
        stream.set_transfer_direction(TransferDirection::M2P);
    } else if stream.transfer_direction() == TransferDirection::M2P {
        panic!("If memory is the destination, the stream transfer direction must be configured in advance to either `P2M` or `M2M`.");
    }

    configure_ndt(stream, peripheral, memory);
}

fn configure_ndt<CXX, Peripheral, Memory>(
    stream: &mut Stream<CXX, Disabled, IsrCleared>,
    peripheral: &PeripheralBuffer<Peripheral>,
    memory: &MemoryBuffer<Memory>,
) where
    CXX: ChannelId,
    Peripheral: Payload,
    Memory: Payload,
{
    match peripheral {
        PeripheralBuffer::Fixed(_) => {
            match memory {
                MemoryBuffer::Fixed(_) => {
                    // NDT must be configured in advance
                }
                MemoryBuffer::Incremented(buffer) => {
                    let p_size: usize =
                        PayloadSize::from_payload::<Peripheral>().into();
                    let m_size: usize =
                        PayloadSize::from_payload::<Memory>().into();

                    let memory_bytes = buffer.len() * m_size;

                    if memory_bytes % p_size != 0 {
                        panic!("Last transfer may be incomplete.");
                    }

                    let ndt = u16::try_from(memory_bytes / p_size).unwrap();
                    stream.set_ndt(ndt.into());
                }
            }
        }
        PeripheralBuffer::Incremented(buffer) => {
            let ndt = u16::try_from(buffer.len()).unwrap();
            stream.set_ndt(Ndt(ndt));
        }
    }
}

pub(super) fn check_buffer<Peripheral, Memory>(
    peripheral: &PeripheralBuffer<Peripheral>,
    memory: &MemoryBuffer<Memory>,
) where
    Peripheral: Payload,
    Memory: Payload,
{
    if peripheral.is_write() == memory.is_write() {
        panic!("One buffer mut be read, one must be write.");
    }
}

pub(super) fn check_double_buffer<Memory>(
    double_buffer: &[MemoryBuffer<Memory>; 2],
) where
    Memory: Payload,
{
    if double_buffer[0].is_fixed() {
        if double_buffer[1].is_incremented() {
            panic!("Invalid double buffer config: First buffer `Fixed`, second buffer `Incremented`.");
        }
    } else {
        if double_buffer[1].is_fixed() {
            panic!("Invalid double buffer config: First buffer `Incremented`, second buffer `Fixed`.");
        }

        let len_0 = double_buffer[0].as_incremented().len();
        let len_1 = double_buffer[1].as_incremented().len();

        if len_0 != len_1 {
            panic!(
                "Invalid double buffer config: len_0 ({}) != len_1 ({})",
                len_0, len_1
            );
        }
    }

    if double_buffer[0].is_write() != double_buffer[1].is_write() {
        panic!("Both buffers must be either both read or write.");
    }
}

pub(super) unsafe fn set_peripheral_impl<Peripheral>(
    peripheral: &mut PeripheralBuffer<Peripheral>,
    index: Option<usize>,
    payload: Peripheral,
) where
    Peripheral: Payload,
{
    match peripheral {
        PeripheralBuffer::Fixed(buffer) => buffer.as_mut_write().set(payload),
        PeripheralBuffer::Incremented(buffer) => match buffer {
            IncrementedBuffer::RegularOffset(buffer) => {
                buffer.as_mut_write().set(index.unwrap(), payload)
            }
            IncrementedBuffer::WordOffset(buffer) => {
                buffer.as_mut_write().set(index.unwrap(), payload)
            }
        },
    }
}

pub(super) unsafe fn set_memory_impl<Memory>(
    memory: &mut MemoryBuffer<Memory>,
    index: Option<usize>,
    payload: Memory,
) where
    Memory: Payload,
{
    match memory {
        MemoryBuffer::Fixed(buffer) => buffer.as_mut_write().set(payload),
        MemoryBuffer::Incremented(buffer) => {
            buffer.as_mut_write().set(index.unwrap(), payload)
        }
    }
}

pub(super) fn mut_ptr_peripheral<Peripheral>(
    peripheral: &mut PeripheralBuffer<Peripheral>,
    index: Option<usize>,
) -> *mut Peripheral
where
    Peripheral: Payload,
{
    match peripheral {
        PeripheralBuffer::Fixed(buffer) => buffer.as_mut_write().as_mut_ptr(),
        PeripheralBuffer::Incremented(buffer) => match buffer {
            IncrementedBuffer::RegularOffset(buffer) => {
                buffer.as_mut_write().as_mut_ptr(index.unwrap())
            }
            IncrementedBuffer::WordOffset(buffer) => {
                buffer.as_mut_write().as_mut_ptr(index.unwrap())
            }
        },
    }
}

pub(super) fn mut_ptr_memory<Memory>(
    memory: &mut MemoryBuffer<Memory>,
    index: Option<usize>,
) -> *mut Memory
where
    Memory: Payload,
{
    match memory {
        MemoryBuffer::Fixed(buffer) => buffer.as_mut_write().as_mut_ptr(),
        MemoryBuffer::Incremented(buffer) => {
            buffer.as_mut_write().as_mut_ptr(index.unwrap())
        }
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
