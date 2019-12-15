use super::stream::{MSize, PSize};
use core::convert::TryInto;
use core::fmt::Debug;
use core::mem;
use core::ops::{Deref, DerefMut};

/// # Safety
///
/// * `Self` must be valid for any bit representation
pub unsafe trait BufferType:
    Sized + Clone + Copy + Sync + 'static
{
}

// Maps BufferTypeSize to number of bytes
int_enum! {
    BufferTypeSize <=> usize,
    "Buffer Size",
    Byte <=> 1,
    HalfWord <=> 2,
    Word <=> 4
}

impl From<BufferTypeSize> for MSize {
    fn from(val: BufferTypeSize) -> Self {
        match val {
            BufferTypeSize::Byte => MSize::Byte,
            BufferTypeSize::HalfWord => MSize::HalfWord,
            BufferTypeSize::Word => MSize::Word,
        }
    }
}

impl From<MSize> for BufferTypeSize {
    fn from(val: MSize) -> Self {
        match val {
            MSize::Byte => BufferTypeSize::Byte,
            MSize::HalfWord => BufferTypeSize::HalfWord,
            MSize::Word => BufferTypeSize::Word,
        }
    }
}

impl From<BufferTypeSize> for PSize {
    fn from(val: BufferTypeSize) -> Self {
        match val {
            BufferTypeSize::Byte => PSize::Byte,
            BufferTypeSize::HalfWord => PSize::HalfWord,
            BufferTypeSize::Word => PSize::Word,
        }
    }
}

impl From<PSize> for BufferTypeSize {
    fn from(val: PSize) -> Self {
        match val {
            PSize::Byte => BufferTypeSize::Byte,
            PSize::HalfWord => BufferTypeSize::HalfWord,
            PSize::Word => BufferTypeSize::Word,
        }
    }
}

impl BufferTypeSize {
    pub fn from_buffer_type<BUF>() -> Self
    where
        BUF: BufferType,
    {
        let size_bytes = mem::size_of::<BUF>();

        size_bytes.try_into().unwrap_or_else(|_| {
            panic!("The Size of the Buffer type must be either 1, 2 or 4 bytes")
        })
    }
}

unsafe impl BufferType for u8 {}

unsafe impl BufferType for i8 {}

unsafe impl BufferType for u16 {}

unsafe impl BufferType for i16 {}

unsafe impl BufferType for u32 {}

unsafe impl BufferType for i32 {}

unsafe impl BufferType for f32 {}

pub enum ImmutableBuffer<'buf, BUF>
where
    BUF: BufferType,
{
    Peripheral(PeripheralBuffer<'buf, BUF>),
    Memory(MemoryBuffer<BUF>),
}

impl<'buf, BUF> ImmutableBuffer<'buf, BUF>
where
    BUF: BufferType,
{
    pub fn peripheral(self) -> PeripheralBuffer<'buf, BUF> {
        if let ImmutableBuffer::Peripheral(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a memory buffer.");
        }
    }

    pub fn memory(self) -> MemoryBuffer<BUF> {
        if let ImmutableBuffer::Memory(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a peripheral buffer.");
        }
    }
}

pub enum MutableBuffer<'buf, BUF>
where
    BUF: BufferType,
{
    Peripheral(PeripheralBufferMut<'buf, BUF>),
    Memory(MemoryBufferMut<BUF>),
}

impl<'buf, BUF> MutableBuffer<'buf, BUF>
where
    BUF: BufferType,
{
    pub fn peripheral(self) -> PeripheralBufferMut<'buf, BUF> {
        if let MutableBuffer::Peripheral(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a memory buffer.");
        }
    }

    pub fn memory(self) -> MemoryBufferMut<BUF> {
        if let MutableBuffer::Memory(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a peripheral buffer.");
        }
    }

    pub fn as_immutable(&self) -> ImmutableBuffer<BUF> {
        match self {
            MutableBuffer::Memory(buffer) => {
                ImmutableBuffer::Memory(buffer.as_immutable())
            }
            MutableBuffer::Peripheral(buffer) => {
                ImmutableBuffer::Peripheral(buffer.as_immutable())
            }
        }
    }
}

pub enum PeripheralBuffer<'buf, BUF>
where
    BUF: BufferType,
{
    Fixed(&'static BUF),
    Incremented(IncrementedBuffer<'buf, BUF>),
}

impl<'buf, BUF> PeripheralBuffer<'buf, BUF>
where
    BUF: BufferType,
{
    pub fn fixed(self) -> &'static BUF {
        if let PeripheralBuffer::Fixed(buffer) = self {
            buffer
        } else {
            panic!("The buffer is incremented.");
        }
    }

    pub fn incremented(self) -> IncrementedBuffer<'buf, BUF> {
        if let PeripheralBuffer::Incremented(buffer) = self {
            buffer
        } else {
            panic!("The buffer is fixed.");
        }
    }
}

pub enum PeripheralBufferMut<'buf, BUF>
where
    BUF: BufferType,
{
    Fixed(&'static mut BUF),
    Incremented(IncrementedBufferMut<'buf, BUF>),
}

impl<'buf, BUF> PeripheralBufferMut<'buf, BUF>
where
    BUF: BufferType,
{
    pub fn as_immutable(&self) -> PeripheralBuffer<BUF> {
        match self {
            PeripheralBufferMut::Fixed(buffer) => unsafe {
                PeripheralBuffer::Fixed(&*(*buffer as *const _))
            },
            PeripheralBufferMut::Incremented(buffer) => {
                PeripheralBuffer::Incremented(buffer.as_immutable())
            }
        }
    }
}

pub enum MemoryBuffer<BUF>
where
    BUF: BufferType,
{
    Fixed(&'static BUF),
    Incremented(&'static [BUF]),
}

pub enum MemoryBufferMut<BUF>
where
    BUF: BufferType,
{
    Fixed(&'static mut BUF),
    Incremented(&'static mut [BUF]),
}

impl<BUF> MemoryBufferMut<BUF>
where
    BUF: BufferType,
{
    pub fn as_immutable(&self) -> MemoryBuffer<BUF> {
        match self {
            MemoryBufferMut::Fixed(buffer) => unsafe {
                MemoryBuffer::Fixed(&*(*buffer as *const _))
            },
            MemoryBufferMut::Incremented(buffer) => unsafe {
                MemoryBuffer::Incremented(&*(*buffer as *const _))
            },
        }
    }
}

pub enum IncrementedBuffer<'buf, BUF>
where
    BUF: BufferType,
{
    RegularOffset(&'static [BUF]),
    WordOffset(WordOffsetBuffer<'buf, BUF>),
}

pub enum IncrementedBufferMut<'buf, BUF>
where
    BUF: BufferType,
{
    RegularOffset(&'static mut [BUF]),
    WordOffset(WordOffsetBufferMut<'buf, BUF>),
}

impl<'buf, BUF> IncrementedBufferMut<'buf, BUF>
where
    BUF: BufferType,
{
    pub fn as_immutable(&self) -> IncrementedBuffer<BUF> {
        match self {
            IncrementedBufferMut::RegularOffset(buffer) => unsafe {
                IncrementedBuffer::RegularOffset(&*(*buffer as *const _))
            },
            IncrementedBufferMut::WordOffset(buffer) => {
                IncrementedBuffer::WordOffset(buffer.as_immutable())
            }
        }
    }
}

pub struct FixedBuffer<BUF>(pub &'static BUF)
where
    BUF: BufferType;

impl<BUF> Deref for FixedBuffer<BUF>
where
    BUF: BufferType,
{
    type Target = BUF;

    fn deref(&self) -> &BUF {
        &*self.0
    }
}

pub struct FixedBufferMut<BUF>(pub &'static mut BUF)
where
    BUF: BufferType;

impl<BUF> Deref for FixedBufferMut<BUF>
where
    BUF: BufferType,
{
    type Target = BUF;

    fn deref(&self) -> &BUF {
        &*self.0
    }
}

impl<BUF> DerefMut for FixedBufferMut<BUF>
where
    BUF: BufferType,
{
    fn deref_mut(&mut self) -> &mut BUF {
        &mut *self.0
    }
}

pub struct RegularOffsetBuffer<BUF>(&'static [BUF])
where
    BUF: BufferType;

impl<BUF> Deref for RegularOffsetBuffer<BUF>
where
    BUF: BufferType,
{
    type Target = [BUF];

    fn deref(&self) -> &[BUF] {
        &*self.0
    }
}

pub struct RegularOffsetBufferMut<BUF>(&'static mut [BUF])
where
    BUF: BufferType;

impl<BUF> Deref for RegularOffsetBufferMut<BUF>
where
    BUF: BufferType,
{
    type Target = [BUF];

    fn deref(&self) -> &[BUF] {
        &*self.0
    }
}

impl<BUF> DerefMut for RegularOffsetBufferMut<BUF>
where
    BUF: BufferType,
{
    fn deref_mut(&mut self) -> &mut [BUF] {
        &mut *self.0
    }
}

pub struct WordOffsetBuffer<'buf, BUF>(&'buf [&'static BUF])
where
    BUF: BufferType;

impl<'buf, BUF> Deref for WordOffsetBuffer<'buf, BUF>
where
    BUF: BufferType,
{
    type Target = [&'static BUF];

    fn deref(&self) -> &[&'static BUF] {
        &*self.0
    }
}

pub struct WordOffsetBufferMut<'buf, BUF>(&'buf [&'static mut BUF])
where
    BUF: BufferType;

impl<'buf, BUF> WordOffsetBufferMut<'buf, BUF>
where
    BUF: BufferType,
{
    pub fn as_immutable(&self) -> WordOffsetBuffer<BUF> {
        let buffer = unsafe { &*(self.0 as *const _ as *const _) };

        WordOffsetBuffer(buffer)
    }
}

impl<'buf, BUF> Deref for WordOffsetBufferMut<'buf, BUF>
where
    BUF: BufferType,
{
    type Target = [&'static mut BUF];

    fn deref(&self) -> &[&'static mut BUF] {
        &*self.0
    }
}
