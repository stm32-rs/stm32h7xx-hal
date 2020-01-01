use super::channel::ChannelId;
use super::stream::{
    BufferMode, Disabled, Enabled, FlowController, IsrCleared, IsrUncleared,
    M0a, MSize, Minc, Ndt, PSize, Pa, Pinc, Pincos, TransferDirection,
    TransferMode,
};
use super::{DMATrait, Stream};
use core::convert::{TryFrom, TryInto};
use core::fmt::Debug;
use core::marker::PhantomData;
use core::{mem, ptr};

pub unsafe trait TransferState: Send + Sync {}

pub struct Start;
unsafe impl TransferState for Start {}

pub struct Ongoing<CXX, DMA>
where
    CXX: ChannelId<DMA = DMA>,
    DMA: DMATrait,
{
    pub(super) stream: Stream<CXX, DMA, Enabled, IsrUncleared>,
}

unsafe impl<CXX, DMA> TransferState for Ongoing<CXX, DMA>
where
    CXX: ChannelId<DMA = DMA>,
    DMA: DMATrait,
{
}

/// # Safety
///
/// * `Self` must be valid for any bit representation
pub unsafe trait Payload:
    Sized + Clone + Copy + Send + Sync + 'static
{
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
        let size_bytes: usize = mem::size_of::<P>();

        size_bytes.try_into().unwrap_or_else(|_| {
            panic!("The size of the buffer type must be either 1, 2 or 4 bytes")
        })
    }
}

unsafe impl Payload for u8 {}

unsafe impl Payload for i8 {}

unsafe impl Payload for u16 {}

unsafe impl Payload for i16 {}

unsafe impl Payload for u32 {}

unsafe impl Payload for i32 {}

unsafe impl Payload for f32 {}

pub trait AsImmutable<'s> {
    type Target: 's;

    /// # Safety
    ///
    /// All unsafe methods of `Self` remain unsafe.
    ///
    /// If `Self` was already immutable, this is safe.
    unsafe fn as_immutable(&'s self) -> Self::Target;
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
}

#[derive(Clone, Copy)]
pub enum BufferR<'buf, 'wo, P>
where
    P: Payload,
{
    Memory(MemoryBufferR<'buf, P>),
    Peripheral(PeripheralBufferR<'buf, 'wo, P>),
}

impl<'buf, 'wo, P> BufferR<'buf, 'wo, P>
where
    P: Payload,
{
    pub fn memory(self) -> MemoryBufferR<'buf, P> {
        if let BufferR::Memory(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a peripheral buffer.");
        }
    }

    pub fn peripheral(self) -> PeripheralBufferR<'buf, 'wo, P> {
        if let BufferR::Peripheral(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a memory buffer.");
        }
    }
}

impl<'s, P> AsImmutable<'s> for BufferR<'_, '_, P>
where
    P: Payload,
{
    type Target = BufferR<'s, 's, P>;

    unsafe fn as_immutable(&self) -> BufferR<P> {
        *self
    }
}

impl<'buf, 'wo, P> From<BufferR<'buf, 'wo, P>> for Buffer<'buf, 'wo, P>
where
    P: Payload,
{
    fn from(buffer: BufferR<'buf, 'wo, P>) -> Self {
        match buffer {
            BufferR::Peripheral(buffer) => Buffer::Peripheral(buffer.into()),
            BufferR::Memory(buffer) => Buffer::Memory(buffer.into()),
        }
    }
}

pub type BufferRStatic<'wo, P> = BufferR<'static, 'wo, P>;

pub enum BufferW<'buf, 'wo, P>
where
    P: Payload,
{
    Memory(MemoryBufferW<'buf, P>),
    Peripheral(PeripheralBufferW<'buf, 'wo, P>),
}

impl<'buf, 'wo, P> BufferW<'buf, 'wo, P>
where
    P: Payload,
{
    pub fn into_memory(self) -> MemoryBufferW<'buf, P> {
        if let BufferW::Memory(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a peripheral buffer.");
        }
    }

    pub fn as_memory(&self) -> &MemoryBufferW<'buf, P> {
        if let BufferW::Memory(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a peripheral buffer.");
        }
    }

    pub fn as_mut_memory(&mut self) -> &mut MemoryBufferW<'buf, P> {
        if let BufferW::Memory(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a peripheral buffer.");
        }
    }

    pub fn into_peripheral(self) -> PeripheralBufferW<'buf, 'wo, P> {
        if let BufferW::Peripheral(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a memory buffer.");
        }
    }

    pub fn as_peripheral(&self) -> &PeripheralBufferW<'buf, 'wo, P> {
        if let BufferW::Peripheral(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a memory buffer.");
        }
    }

    pub fn as_mut_peripheral(
        &mut self,
    ) -> &mut PeripheralBufferW<'buf, 'wo, P> {
        if let BufferW::Peripheral(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a memory buffer.");
        }
    }
}

impl<'s, P> AsImmutable<'s> for BufferW<'_, '_, P>
where
    P: Payload,
{
    type Target = BufferR<'s, 's, P>;

    unsafe fn as_immutable(&'s self) -> BufferR<P> {
        match self {
            BufferW::Memory(buffer) => BufferR::Memory(buffer.as_immutable()),
            BufferW::Peripheral(buffer) => {
                BufferR::Peripheral(buffer.as_immutable())
            }
        }
    }
}

impl<'buf, 'wo, P> From<BufferW<'buf, 'wo, P>> for Buffer<'buf, 'wo, P>
where
    P: Payload,
{
    fn from(buffer: BufferW<'buf, 'wo, P>) -> Self {
        match buffer {
            BufferW::Peripheral(buffer) => Buffer::Peripheral(buffer.into()),
            BufferW::Memory(buffer) => Buffer::Memory(buffer.into()),
        }
    }
}

pub type BufferWStatic<'wo, P> = BufferW<'static, 'wo, P>;

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
}

#[derive(Clone, Copy)]
pub enum MemoryBufferR<'buf, P>
where
    P: Payload,
{
    Fixed(FixedBufferR<'buf, P>),
    Incremented(RegularOffsetBufferR<'buf, P>),
}

impl<'buf, P> MemoryBufferR<'buf, P>
where
    P: Payload,
{
    pub fn fixed(self) -> FixedBufferR<'buf, P> {
        if let MemoryBufferR::Fixed(buffer) = self {
            buffer
        } else {
            panic!("The buffer is incremented.");
        }
    }

    pub fn incremented(self) -> RegularOffsetBufferR<'buf, P> {
        if let MemoryBufferR::Incremented(buffer) = self {
            buffer
        } else {
            panic!("The buffer is fixed.");
        }
    }
}

impl<'s, P> AsImmutable<'s> for MemoryBufferR<'_, P>
where
    P: Payload,
{
    type Target = MemoryBufferR<'s, P>;

    unsafe fn as_immutable(&self) -> MemoryBufferR<P> {
        *self
    }
}

impl<'buf, P> From<MemoryBufferR<'buf, P>> for MemoryBuffer<'buf, P>
where
    P: Payload,
{
    fn from(buffer: MemoryBufferR<'buf, P>) -> Self {
        match buffer {
            MemoryBufferR::Fixed(buffer) => MemoryBuffer::Fixed(buffer.into()),
            MemoryBufferR::Incremented(buffer) => {
                MemoryBuffer::Incremented(buffer.into())
            }
        }
    }
}

pub type MemoryBufferRStatic<P> = MemoryBufferR<'static, P>;

pub enum MemoryBufferW<'buf, P>
where
    P: Payload,
{
    Fixed(FixedBufferW<'buf, P>),
    Incremented(RegularOffsetBufferW<'buf, P>),
}

impl<'buf, P> MemoryBufferW<'buf, P>
where
    P: Payload,
{
    pub fn into_fixed(self) -> FixedBufferW<'buf, P> {
        if let MemoryBufferW::Fixed(buffer) = self {
            buffer
        } else {
            panic!("The buffer is incremented.");
        }
    }

    pub fn as_fixed(&self) -> &FixedBufferW<'buf, P> {
        if let MemoryBufferW::Fixed(buffer) = self {
            buffer
        } else {
            panic!("The buffer is incremented.");
        }
    }

    pub fn as_mut_fixed(&mut self) -> &mut FixedBufferW<'buf, P> {
        if let MemoryBufferW::Fixed(buffer) = self {
            buffer
        } else {
            panic!("The buffer is incremented.");
        }
    }

    pub fn into_incremented(self) -> RegularOffsetBufferW<'buf, P> {
        if let MemoryBufferW::Incremented(buffer) = self {
            buffer
        } else {
            panic!("The buffer is fixed.");
        }
    }

    pub fn as_incremented(&self) -> &RegularOffsetBufferW<'buf, P> {
        if let MemoryBufferW::Incremented(buffer) = self {
            buffer
        } else {
            panic!("The buffer is fixed.");
        }
    }

    pub fn as_mut_incremented(&mut self) -> &mut RegularOffsetBufferW<'buf, P> {
        if let MemoryBufferW::Incremented(buffer) = self {
            buffer
        } else {
            panic!("The buffer is fixed.");
        }
    }
}

impl<'s, P> AsImmutable<'s> for MemoryBufferW<'_, P>
where
    P: Payload,
{
    type Target = MemoryBufferR<'s, P>;

    unsafe fn as_immutable(&self) -> MemoryBufferR<P> {
        match self {
            MemoryBufferW::Fixed(buffer) => {
                MemoryBufferR::Fixed(buffer.as_immutable())
            }
            MemoryBufferW::Incremented(buffer) => {
                MemoryBufferR::Incremented(buffer.as_immutable())
            }
        }
    }
}

impl<'buf, P> From<MemoryBufferW<'buf, P>> for MemoryBuffer<'buf, P>
where
    P: Payload,
{
    fn from(buffer: MemoryBufferW<'buf, P>) -> Self {
        match buffer {
            MemoryBufferW::Fixed(buffer) => MemoryBuffer::Fixed(buffer.into()),
            MemoryBufferW::Incremented(buffer) => {
                MemoryBuffer::Incremented(buffer.into())
            }
        }
    }
}

pub type MemoryBufferWStatic<P> = MemoryBufferW<'static, P>;

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
}

#[derive(Clone, Copy)]
pub enum PeripheralBufferR<'buf, 'wo, P>
where
    P: Payload,
{
    Fixed(FixedBufferR<'buf, P>),
    Incremented(IncrementedBufferR<'buf, 'wo, P>),
}

impl<'buf, 'wo, P> PeripheralBufferR<'buf, 'wo, P>
where
    P: Payload,
{
    pub fn fixed(self) -> FixedBufferR<'buf, P> {
        if let PeripheralBufferR::Fixed(buffer) = self {
            buffer
        } else {
            panic!("The buffer is incremented.");
        }
    }

    pub fn incremented(self) -> IncrementedBufferR<'buf, 'wo, P> {
        if let PeripheralBufferR::Incremented(buffer) = self {
            buffer
        } else {
            panic!("The buffer is fixed.");
        }
    }
}

impl<'s, P> AsImmutable<'s> for PeripheralBufferR<'_, '_, P>
where
    P: Payload,
{
    type Target = PeripheralBufferR<'s, 's, P>;

    unsafe fn as_immutable(&self) -> PeripheralBufferR<P> {
        *self
    }
}

impl<'buf, 'wo, P> From<PeripheralBufferR<'buf, 'wo, P>>
    for PeripheralBuffer<'buf, 'wo, P>
where
    P: Payload,
{
    fn from(buffer: PeripheralBufferR<'buf, 'wo, P>) -> Self {
        match buffer {
            PeripheralBufferR::Fixed(buffer) => {
                PeripheralBuffer::Fixed(buffer.into())
            }
            PeripheralBufferR::Incremented(buffer) => {
                PeripheralBuffer::Incremented(buffer.into())
            }
        }
    }
}

pub type PeripheralBufferRStatic<'wo, P> = PeripheralBufferR<'static, 'wo, P>;

pub enum PeripheralBufferW<'buf, 'wo, P>
where
    P: Payload,
{
    Fixed(FixedBufferW<'buf, P>),
    Incremented(IncrementedBufferW<'buf, 'wo, P>),
}

impl<'buf, 'wo, P> PeripheralBufferW<'buf, 'wo, P>
where
    P: Payload,
{
    pub fn into_fixed(self) -> FixedBufferW<'buf, P> {
        if let PeripheralBufferW::Fixed(buffer) = self {
            buffer
        } else {
            panic!("The buffer is incremented.");
        }
    }

    pub fn as_fixed(&self) -> &FixedBufferW<'buf, P> {
        if let PeripheralBufferW::Fixed(buffer) = self {
            buffer
        } else {
            panic!("The buffer is incremented.");
        }
    }

    pub fn as_mut_fixed(&mut self) -> &mut FixedBufferW<'buf, P> {
        if let PeripheralBufferW::Fixed(buffer) = self {
            buffer
        } else {
            panic!("The buffer is incremented.");
        }
    }

    pub fn into_incremented(self) -> IncrementedBufferW<'buf, 'wo, P> {
        if let PeripheralBufferW::Incremented(buffer) = self {
            buffer
        } else {
            panic!("The buffer is fixed.");
        }
    }

    pub fn as_incremented(&self) -> &IncrementedBufferW<'buf, 'wo, P> {
        if let PeripheralBufferW::Incremented(buffer) = self {
            buffer
        } else {
            panic!("The buffer is fixed.");
        }
    }

    pub fn as_mut_incremented(
        &mut self,
    ) -> &mut IncrementedBufferW<'buf, 'wo, P> {
        if let PeripheralBufferW::Incremented(buffer) = self {
            buffer
        } else {
            panic!("The buffer is fixed.");
        }
    }
}

impl<'s, P> AsImmutable<'s> for PeripheralBufferW<'_, '_, P>
where
    P: Payload,
{
    type Target = PeripheralBufferR<'s, 's, P>;

    unsafe fn as_immutable(&self) -> PeripheralBufferR<P> {
        match self {
            PeripheralBufferW::Fixed(buffer) => {
                PeripheralBufferR::Fixed(buffer.as_immutable())
            }
            PeripheralBufferW::Incremented(buffer) => {
                PeripheralBufferR::Incremented(buffer.as_immutable())
            }
        }
    }
}

impl<'buf, 'wo, P> From<PeripheralBufferW<'buf, 'wo, P>>
    for PeripheralBuffer<'buf, 'wo, P>
where
    P: Payload,
{
    fn from(buffer: PeripheralBufferW<'buf, 'wo, P>) -> Self {
        match buffer {
            PeripheralBufferW::Fixed(buffer) => {
                PeripheralBuffer::Fixed(buffer.into())
            }
            PeripheralBufferW::Incremented(buffer) => {
                PeripheralBuffer::Incremented(buffer.into())
            }
        }
    }
}

pub type PeripheralBufferWStatic<'wo, P> = PeripheralBufferW<'static, 'wo, P>;

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

    pub fn len(self) -> usize {
        match self {
            IncrementedBuffer::RegularOffset(buffer) => buffer.len(),
            IncrementedBuffer::WordOffset(buffer) => buffer.len(),
        }
    }

    pub unsafe fn get(self, index: usize) -> P {
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
}

#[derive(Clone, Copy)]
pub enum IncrementedBufferR<'buf, 'wo, P>
where
    P: Payload,
{
    RegularOffset(RegularOffsetBufferR<'buf, P>),
    WordOffset(WordOffsetBufferR<'buf, 'wo, P>),
}

#[allow(clippy::len_without_is_empty)]
impl<'buf, 'wo, P> IncrementedBufferR<'buf, 'wo, P>
where
    P: Payload,
{
    pub fn regular_offset(self) -> RegularOffsetBufferR<'buf, P> {
        if let IncrementedBufferR::RegularOffset(buffer) = self {
            buffer
        } else {
            panic!("The buffer has word offset.");
        }
    }

    pub fn word_offset(self) -> WordOffsetBufferR<'buf, 'wo, P> {
        if let IncrementedBufferR::WordOffset(buffer) = self {
            buffer
        } else {
            panic!("The buffer has regular offset.");
        }
    }

    pub fn len(self) -> usize {
        match self {
            IncrementedBufferR::RegularOffset(buffer) => buffer.len(),
            IncrementedBufferR::WordOffset(buffer) => buffer.len(),
        }
    }

    pub fn get(self, index: usize) -> P {
        match self {
            IncrementedBufferR::RegularOffset(buffer) => buffer.get(index),
            IncrementedBufferR::WordOffset(buffer) => buffer.get(index),
        }
    }

    pub fn as_ptr(&self, index: usize) -> *const P {
        match self {
            IncrementedBufferR::RegularOffset(buffer) => buffer.as_ptr(index),
            IncrementedBufferR::WordOffset(buffer) => buffer.as_ptr(index),
        }
    }
}

impl<'s, P> AsImmutable<'s> for IncrementedBufferR<'_, '_, P>
where
    P: Payload,
{
    type Target = IncrementedBufferR<'s, 's, P>;

    unsafe fn as_immutable(&self) -> IncrementedBufferR<P> {
        *self
    }
}

impl<'buf, 'wo, P> From<IncrementedBufferR<'buf, 'wo, P>>
    for IncrementedBuffer<'buf, 'wo, P>
where
    P: Payload,
{
    fn from(buffer: IncrementedBufferR<'buf, 'wo, P>) -> Self {
        match buffer {
            IncrementedBufferR::RegularOffset(buffer) => {
                IncrementedBuffer::RegularOffset(buffer.into())
            }
            IncrementedBufferR::WordOffset(buffer) => {
                IncrementedBuffer::WordOffset(buffer.into())
            }
        }
    }
}

pub type IncrementedBufferRStatic<'wo, P> = IncrementedBufferR<'static, 'wo, P>;

pub enum IncrementedBufferW<'buf, 'wo, P>
where
    P: Payload,
{
    RegularOffset(RegularOffsetBufferW<'buf, P>),
    WordOffset(WordOffsetBufferW<'buf, 'wo, P>),
}

#[allow(clippy::len_without_is_empty)]
impl<'buf, 'wo, P> IncrementedBufferW<'buf, 'wo, P>
where
    P: Payload,
{
    pub fn into_regular_offset(self) -> RegularOffsetBufferW<'buf, P> {
        if let IncrementedBufferW::RegularOffset(buffer) = self {
            buffer
        } else {
            panic!("The buffer has word offset.");
        }
    }

    pub fn as_regular_offset(&self) -> &RegularOffsetBufferW<'buf, P> {
        if let IncrementedBufferW::RegularOffset(buffer) = self {
            buffer
        } else {
            panic!("The buffer has word offset.");
        }
    }

    pub fn as_mut_regular_offset(
        &mut self,
    ) -> &mut RegularOffsetBufferW<'buf, P> {
        if let IncrementedBufferW::RegularOffset(buffer) = self {
            buffer
        } else {
            panic!("The buffer has word offset.");
        }
    }

    pub fn into_word_offset(self) -> WordOffsetBufferW<'buf, 'wo, P> {
        if let IncrementedBufferW::WordOffset(buffer) = self {
            buffer
        } else {
            panic!("The buffer has regular offset.");
        }
    }

    pub fn as_word_offset(&self) -> &WordOffsetBufferW<'buf, 'wo, P> {
        if let IncrementedBufferW::WordOffset(buffer) = self {
            buffer
        } else {
            panic!("The buffer has regular offset.");
        }
    }

    pub fn as_mut_word_offset(
        &mut self,
    ) -> &mut WordOffsetBufferW<'buf, 'wo, P> {
        if let IncrementedBufferW::WordOffset(buffer) = self {
            buffer
        } else {
            panic!("The buffer has regular offset.");
        }
    }

    pub fn len(&self) -> usize {
        match self {
            IncrementedBufferW::RegularOffset(buffer) => buffer.len(),
            IncrementedBufferW::WordOffset(buffer) => buffer.len(),
        }
    }

    /// # Safety
    ///
    /// The caller must ensure, that the DMA is currently not modifying this address.
    pub unsafe fn get(&self, index: usize) -> P {
        match self {
            IncrementedBufferW::RegularOffset(buffer) => buffer.get(index),
            IncrementedBufferW::WordOffset(buffer) => buffer.get(index),
        }
    }

    /// # Safety
    ///
    /// The caller must ensure, that the DMA is currently not modifying this address.
    pub unsafe fn set(&mut self, index: usize, payload: P) {
        match self {
            IncrementedBufferW::RegularOffset(buffer) => {
                buffer.set(index, payload)
            }
            IncrementedBufferW::WordOffset(buffer) => {
                buffer.set(index, payload)
            }
        }
    }

    pub fn as_ptr(&self, index: usize) -> *const P {
        match self {
            IncrementedBufferW::RegularOffset(buffer) => buffer.as_ptr(index),
            IncrementedBufferW::WordOffset(buffer) => buffer.as_ptr(index),
        }
    }

    pub fn as_mut_ptr(&mut self, index: usize) -> *mut P {
        match self {
            IncrementedBufferW::RegularOffset(buffer) => {
                buffer.as_mut_ptr(index)
            }
            IncrementedBufferW::WordOffset(buffer) => buffer.as_mut_ptr(index),
        }
    }
}

impl<'s, P> AsImmutable<'s> for IncrementedBufferW<'_, '_, P>
where
    P: Payload,
{
    type Target = IncrementedBufferR<'s, 's, P>;

    /// # Safety
    ///
    /// `IncrementedBuffer` assumes that the DMA is only reading the buffer.
    /// Therefore the getters of the immutable version are as unsafe as the getters of this struct.
    unsafe fn as_immutable(&self) -> IncrementedBufferR<P> {
        match self {
            IncrementedBufferW::RegularOffset(buffer) => {
                IncrementedBufferR::RegularOffset(buffer.as_immutable())
            }
            IncrementedBufferW::WordOffset(buffer) => {
                IncrementedBufferR::WordOffset(buffer.as_immutable())
            }
        }
    }
}

impl<'buf, 'wo, P> From<IncrementedBufferW<'buf, 'wo, P>>
    for IncrementedBuffer<'buf, 'wo, P>
where
    P: Payload,
{
    fn from(buffer: IncrementedBufferW<'buf, 'wo, P>) -> Self {
        match buffer {
            IncrementedBufferW::RegularOffset(buffer) => {
                IncrementedBuffer::RegularOffset(buffer.into())
            }
            IncrementedBufferW::WordOffset(buffer) => {
                IncrementedBuffer::WordOffset(buffer.into())
            }
        }
    }
}

pub type IncrementedBufferWStatic<'wo, P> = IncrementedBufferW<'static, 'wo, P>;

pub enum FixedBuffer<'buf, P>
where
    P: Payload,
{
    Read(FixedBufferR<'buf, P>),
    Write(FixedBufferW<'buf, P>),
}

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

impl<'s, P> AsImmutable<'s> for FixedBufferR<'_, P>
where
    P: Payload,
{
    type Target = FixedBufferR<'s, P>;

    unsafe fn as_immutable(&self) -> FixedBufferR<P> {
        *self
    }
}

impl<'buf, P> From<FixedBufferR<'buf, P>> for FixedBuffer<'buf, P>
where
    P: Payload,
{
    fn from(buffer: FixedBufferR<'buf, P>) -> Self {
        FixedBuffer::Read(buffer)
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

impl<'s, P> AsImmutable<'s> for FixedBufferW<'_, P>
where
    P: Payload,
{
    type Target = FixedBufferR<'s, P>;

    unsafe fn as_immutable(&self) -> FixedBufferR<P> {
        FixedBufferR(self.0, PhantomData)
    }
}

impl<'buf, P> From<FixedBufferW<'buf, P>> for FixedBuffer<'buf, P>
where
    P: Payload,
{
    fn from(buffer: FixedBufferW<'buf, P>) -> Self {
        FixedBuffer::Write(buffer)
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

impl<'s, P> AsImmutable<'s> for RegularOffsetBufferR<'_, P>
where
    P: Payload,
{
    type Target = RegularOffsetBufferR<'s, P>;

    unsafe fn as_immutable(&self) -> RegularOffsetBufferR<P> {
        *self
    }
}

impl<'buf, P> From<RegularOffsetBufferR<'buf, P>>
    for RegularOffsetBuffer<'buf, P>
where
    P: Payload,
{
    fn from(buffer: RegularOffsetBufferR<'buf, P>) -> Self {
        RegularOffsetBuffer::Read(buffer)
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

impl<'s, P> AsImmutable<'s> for RegularOffsetBufferW<'_, P>
where
    P: Payload,
{
    type Target = RegularOffsetBufferR<'s, P>;

    unsafe fn as_immutable(&self) -> RegularOffsetBufferR<P> {
        RegularOffsetBufferR(self.0, PhantomData)
    }
}

impl<'buf, P> From<RegularOffsetBufferW<'buf, P>>
    for RegularOffsetBuffer<'buf, P>
where
    P: Payload,
{
    fn from(buffer: RegularOffsetBufferW<'buf, P>) -> Self {
        RegularOffsetBuffer::Write(buffer)
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

    // Methods both variants implement

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

impl<'s, P> AsImmutable<'s> for WordOffsetBufferR<'_, '_, P>
where
    P: Payload,
{
    type Target = WordOffsetBufferR<'s, 's, P>;

    unsafe fn as_immutable(&self) -> WordOffsetBufferR<P> {
        *self
    }
}

impl<'buf, 'wo, P> From<WordOffsetBufferR<'buf, 'wo, P>>
    for WordOffsetBuffer<'buf, 'wo, P>
where
    P: Payload,
{
    fn from(buffer: WordOffsetBufferR<'buf, 'wo, P>) -> Self {
        WordOffsetBuffer::Read(buffer)
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

impl<'s, P> AsImmutable<'s> for WordOffsetBufferW<'_, '_, P>
where
    P: Payload,
{
    type Target = WordOffsetBufferR<'s, 's, P>;

    unsafe fn as_immutable(&self) -> WordOffsetBufferR<P> {
        WordOffsetBufferR(&*(self.0 as *const _ as *const _), PhantomData)
    }
}

impl<'buf, 'wo, P> From<WordOffsetBufferW<'buf, 'wo, P>>
    for WordOffsetBuffer<'buf, 'wo, P>
where
    P: Payload,
{
    fn from(buffer: WordOffsetBufferW<'buf, 'wo, P>) -> Self {
        WordOffsetBuffer::Write(buffer)
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
// Secure Transfer implementations
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

/// Configures the buffers of the transfer.
///
/// **Note**: Configures the following values:
///
/// - `PSize`, `Msize`
/// - `Pa`, `M0a`
/// - `Pinc`, `Minc`
/// - `Pincos`
/// - `Ndt`
pub(super) fn configure_safe_transfer<CXX, DMA, Source, Dest>(
    stream: &mut Stream<CXX, DMA, Disabled, IsrCleared>,
    source: BufferR<Source>,
    dest: &BufferW<Dest>,
) where
    CXX: ChannelId<DMA = DMA>,
    DMA: DMATrait,
    Source: Payload,
    Dest: Payload,
{
    // Note(safety): Safe as the transfer has not started yet.
    let dest = unsafe { dest.as_immutable() };
    match stream.transfer_direction() {
        TransferDirection::P2M => {
            configure_buffers(stream, source.peripheral(), dest.memory());
        }
        TransferDirection::M2P => {
            configure_buffers(stream, dest.peripheral(), source.memory());
        }
        TransferDirection::M2M => {
            configure_buffers(stream, source.peripheral(), dest.memory());
        }
    }
}

fn configure_buffers<CXX, DMA, Peripheral, Memory>(
    stream: &mut Stream<CXX, DMA, Disabled, IsrCleared>,
    peripheral: PeripheralBufferR<Peripheral>,
    memory: MemoryBufferR<Memory>,
) where
    CXX: ChannelId<DMA = DMA>,
    DMA: DMATrait,
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
        PeripheralBufferR::Fixed(buffer) => {
            stream.set_pa(Pa(buffer.as_ptr() as u32));
            stream.set_pinc(Pinc::Fixed);
        }
        PeripheralBufferR::Incremented(buffer) => match buffer {
            IncrementedBufferR::RegularOffset(buffer) => {
                stream.set_pa(Pa(buffer.as_ptr(0) as u32));
                stream.set_pinc(Pinc::Incremented);
                stream.set_pincos(Pincos::PSize);
            }
            IncrementedBufferR::WordOffset(buffer) => {
                stream.set_pa(Pa(buffer.as_ptr(0) as u32));
                stream.set_pinc(Pinc::Incremented);
                stream.set_pincos(Pincos::Word);
            }
        },
    }

    match memory {
        MemoryBufferR::Fixed(buffer) => {
            stream.set_m0a(M0a(buffer.as_ptr() as u32));
            stream.set_minc(Minc::Fixed);
        }
        MemoryBufferR::Incremented(buffer) => {
            stream.set_m0a(M0a(buffer.as_ptr(0) as u32));
            stream.set_minc(Minc::Incremented);
        }
    }

    configure_ndt(stream, peripheral, memory);
}

fn configure_ndt<CXX, DMA, Peripheral, Memory>(
    stream: &mut Stream<CXX, DMA, Disabled, IsrCleared>,
    peripheral: PeripheralBufferR<Peripheral>,
    memory: MemoryBufferR<Memory>,
) where
    CXX: ChannelId<DMA = DMA>,
    DMA: DMATrait,
    Peripheral: Payload,
    Memory: Payload,
{
    match peripheral {
        PeripheralBufferR::Fixed(_) => {
            match memory {
                MemoryBufferR::Fixed(_) => {
                    // NDT must be configured in advance
                }
                MemoryBufferR::Incremented(buffer) => {
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
        PeripheralBufferR::Incremented(buffer) => {
            let ndt = u16::try_from(buffer.len()).unwrap();
            stream.set_ndt(Ndt(ndt));
        }
    }
}

pub(super) fn check_double_buffer_stream_config<CXX, DMA>(
    stream: &Stream<CXX, DMA, Disabled, IsrCleared>,
) where
    CXX: ChannelId<DMA = DMA>,
    DMA: DMATrait,
{
    if stream.transfer_direction() == TransferDirection::M2M {
        panic!("The stream direction must not be `M2M` when configuring double buffer streams.");
    }

    if stream.effective_flow_controller() == FlowController::Peripheral {
        panic!("The flow controller must not be `Peripheral` when configuring double buffer streams.");
    }

    debug_assert_eq!(stream.effective_buffer_mode(), BufferMode::DoubleBuffer);
}

pub(super) fn check_double_buffer<'s, MemBuf, P>(double_buffer: &'s [MemBuf; 2])
where
    MemBuf: AsImmutable<'s, Target = MemoryBufferR<'s, P>>,
    P: Payload,
{
    let double_buffer = unsafe {
        [
            double_buffer[0].as_immutable(),
            double_buffer[1].as_immutable(),
        ]
    };
    match double_buffer[0] {
        MemoryBufferR::Fixed(_) => {
            if let MemoryBufferR::Incremented(_) = double_buffer[1] {
                panic!("Invalid double buffer config: First buffer `Fixed`, second buffer `Incremented`.");
            }
        }
        MemoryBufferR::Incremented(buffer_0) => {
            if let MemoryBufferR::Fixed(_) = double_buffer[1] {
                panic!("Invalid double buffer config: First buffer `Incremented`, second buffer `Fixed`.");
            }

            let len_0 = buffer_0.len();
            let len_1 = double_buffer[1].incremented().len();

            if len_0 != len_1 {
                panic!(
                    "Invalid double buffer config: len_0 ({}) != len_1({})",
                    len_0, len_1
                );
            }
        }
    }
}

pub(super) fn first_ptr_from_buffer<'s, ImmutBuf, P>(
    buffer: &'s ImmutBuf,
) -> *const P
where
    ImmutBuf: AsImmutable<'s, Target = BufferR<'s, 's, P>>,
    P: Payload,
{
    let buffer = unsafe { buffer.as_immutable() };

    match buffer {
        BufferR::Peripheral(buffer) => match buffer {
            PeripheralBufferR::Fixed(buffer) => buffer.as_ptr(),
            PeripheralBufferR::Incremented(buffer) => buffer.as_ptr(0),
        },
        BufferR::Memory(buffer) => match buffer {
            MemoryBufferR::Fixed(buffer) => buffer.as_ptr(),
            MemoryBufferR::Incremented(buffer) => buffer.as_ptr(0),
        },
    }
}
