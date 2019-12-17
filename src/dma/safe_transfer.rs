use super::stream::{MSize, PSize};
use core::convert::TryInto;
use core::fmt::Debug;
use core::marker::PhantomData;
use core::{mem, ptr};
use vcell::VolatileCell;

/// # Safety
///
/// * `Self` must be valid for any bit representation
pub unsafe trait BufferType: Sized + Clone + Copy + Sync {}

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

pub enum ImmutableBuffer<'buf, 'wo, BUF>
where
    BUF: BufferType,
{
    Memory(MemoryBuffer<'buf, BUF>),
    Peripheral(PeripheralBuffer<'buf, 'wo, BUF>),
}

impl<'buf, 'wo, BUF> ImmutableBuffer<'buf, 'wo, BUF>
where
    BUF: BufferType,
{
    pub fn memory(self) -> MemoryBuffer<'buf, BUF> {
        if let ImmutableBuffer::Memory(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a peripheral buffer.");
        }
    }

    pub fn peripheral(self) -> PeripheralBuffer<'buf, 'wo, BUF> {
        if let ImmutableBuffer::Peripheral(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a memory buffer.");
        }
    }
}

pub type ImmutableBufferStatic<'wo, BUF> = ImmutableBuffer<'static, 'wo, BUF>;

pub enum MutableBuffer<'buf, 'wo, BUF>
where
    BUF: BufferType,
{
    Memory(MemoryBufferMut<'buf, BUF>),
    Peripheral(PeripheralBufferMut<'buf, 'wo, BUF>),
}

impl<'buf, 'wo, BUF> MutableBuffer<'buf, 'wo, BUF>
where
    BUF: BufferType,
{
    pub fn into_memory(self) -> MemoryBufferMut<'buf, BUF> {
        if let MutableBuffer::Memory(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a peripheral buffer.");
        }
    }

    pub fn as_memory(&self) -> &MemoryBufferMut<'buf, BUF> {
        if let MutableBuffer::Memory(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a peripheral buffer.");
        }
    }

    pub fn as_mut_memory(&mut self) -> &mut MemoryBufferMut<'buf, BUF> {
        if let MutableBuffer::Memory(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a peripheral buffer.");
        }
    }

    pub fn into_peripheral(self) -> PeripheralBufferMut<'buf, 'wo, BUF> {
        if let MutableBuffer::Peripheral(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a memory buffer.");
        }
    }

    pub fn as_peripheral(&self) -> &PeripheralBufferMut<'buf, 'wo, BUF> {
        if let MutableBuffer::Peripheral(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a memory buffer.");
        }
    }

    pub fn as_mut_peripheral(
        &mut self,
    ) -> &mut PeripheralBufferMut<'buf, 'wo, BUF> {
        if let MutableBuffer::Peripheral(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a memory buffer.");
        }
    }
}

pub type MutableBufferStatic<'wo, BUF> = MutableBuffer<'static, 'wo, BUF>;

pub enum MemoryBuffer<'buf, BUF>
where
    BUF: BufferType,
{
    Fixed(FixedBuffer<'buf, BUF>),
    Incremented(RegularOffsetBuffer<'buf, BUF>),
}

impl<'buf, BUF> MemoryBuffer<'buf, BUF>
where
    BUF: BufferType,
{
    pub fn fixed(self) -> FixedBuffer<'buf, BUF> {
        if let MemoryBuffer::Fixed(buffer) = self {
            buffer
        } else {
            panic!("The buffer is incremented.");
        }
    }

    pub fn incremented(self) -> RegularOffsetBuffer<'buf, BUF> {
        if let MemoryBuffer::Incremented(buffer) = self {
            buffer
        } else {
            panic!("The buffer is fixed.");
        }
    }
}

pub type MemoryBufferStatic<BUF> = MemoryBuffer<'static, BUF>;

pub enum MemoryBufferMut<'buf, BUF>
where
    BUF: BufferType,
{
    Fixed(FixedBufferMut<'buf, BUF>),
    Incremented(RegularOffsetBufferMut<'buf, BUF>),
}

impl<'buf, BUF> MemoryBufferMut<'buf, BUF>
where
    BUF: BufferType,
{
    pub fn into_fixed(self) -> FixedBufferMut<'buf, BUF> {
        if let MemoryBufferMut::Fixed(buffer) = self {
            buffer
        } else {
            panic!("The buffer is incremented.");
        }
    }

    pub fn as_fixed(&self) -> &FixedBufferMut<'buf, BUF> {
        if let MemoryBufferMut::Fixed(buffer) = self {
            buffer
        } else {
            panic!("The buffer is incremented.");
        }
    }

    pub fn as_mut_fixed(&mut self) -> &mut FixedBufferMut<'buf, BUF> {
        if let MemoryBufferMut::Fixed(buffer) = self {
            buffer
        } else {
            panic!("The buffer is incremented.");
        }
    }

    pub fn into_incremented(self) -> RegularOffsetBufferMut<'buf, BUF> {
        if let MemoryBufferMut::Incremented(buffer) = self {
            buffer
        } else {
            panic!("The buffer is fixed.");
        }
    }

    pub fn as_incremented(&self) -> &RegularOffsetBufferMut<'buf, BUF> {
        if let MemoryBufferMut::Incremented(buffer) = self {
            buffer
        } else {
            panic!("The buffer is fixed.");
        }
    }

    pub fn as_mut_incremented(
        &mut self,
    ) -> &mut RegularOffsetBufferMut<'buf, BUF> {
        if let MemoryBufferMut::Incremented(buffer) = self {
            buffer
        } else {
            panic!("The buffer is fixed.");
        }
    }
}

pub type MemoryBufferMutStatic<BUF> = MemoryBufferMut<'static, BUF>;

pub enum PeripheralBuffer<'buf, 'wo, BUF>
where
    BUF: BufferType,
{
    Fixed(FixedBuffer<'buf, BUF>),
    Incremented(IncrementedBuffer<'buf, 'wo, BUF>),
}

impl<'buf, 'wo, BUF> PeripheralBuffer<'buf, 'wo, BUF>
where
    BUF: BufferType,
{
    pub fn fixed(self) -> FixedBuffer<'buf, BUF> {
        if let PeripheralBuffer::Fixed(buffer) = self {
            buffer
        } else {
            panic!("The buffer is incremented.");
        }
    }

    pub fn incremented(self) -> IncrementedBuffer<'buf, 'wo, BUF> {
        if let PeripheralBuffer::Incremented(buffer) = self {
            buffer
        } else {
            panic!("The buffer is fixed.");
        }
    }
}

pub type PeripheralBufferStatic<'wo, BUF> = PeripheralBuffer<'static, 'wo, BUF>;

pub enum PeripheralBufferMut<'buf, 'wo, BUF>
where
    BUF: BufferType,
{
    Fixed(FixedBufferMut<'buf, BUF>),
    Incremented(IncrementedBufferMut<'buf, 'wo, BUF>),
}

impl<'buf, 'wo, BUF> PeripheralBufferMut<'buf, 'wo, BUF>
where
    BUF: BufferType,
{
    pub fn into_fixed(self) -> FixedBufferMut<'buf, BUF> {
        if let PeripheralBufferMut::Fixed(buffer) = self {
            buffer
        } else {
            panic!("The buffer is incremented.");
        }
    }

    pub fn as_fixed(&self) -> &FixedBufferMut<'buf, BUF> {
        if let PeripheralBufferMut::Fixed(buffer) = self {
            buffer
        } else {
            panic!("The buffer is incremented.");
        }
    }

    pub fn as_mut_fixed(&mut self) -> &mut FixedBufferMut<'buf, BUF> {
        if let PeripheralBufferMut::Fixed(buffer) = self {
            buffer
        } else {
            panic!("The buffer is incremented.");
        }
    }

    pub fn into_incremented(self) -> IncrementedBufferMut<'buf, 'wo, BUF> {
        if let PeripheralBufferMut::Incremented(buffer) = self {
            buffer
        } else {
            panic!("The buffer is fixed.");
        }
    }

    pub fn as_incremented(&self) -> &IncrementedBufferMut<'buf, 'wo, BUF> {
        if let PeripheralBufferMut::Incremented(buffer) = self {
            buffer
        } else {
            panic!("The buffer is fixed.");
        }
    }

    pub fn as_mut_incremented(
        &mut self,
    ) -> &mut IncrementedBufferMut<'buf, 'wo, BUF> {
        if let PeripheralBufferMut::Incremented(buffer) = self {
            buffer
        } else {
            panic!("The buffer is fixed.");
        }
    }
}

pub type PeripheralBufferMutStatic<'wo, BUF> =
    PeripheralBufferMut<'static, 'wo, BUF>;

pub enum IncrementedBuffer<'buf, 'wo, BUF>
where
    BUF: BufferType,
{
    RegularOffset(RegularOffsetBuffer<'buf, BUF>),
    WordOffset(WordOffsetBuffer<'buf, 'wo, BUF>),
}

impl<'buf, 'wo, BUF> IncrementedBuffer<'buf, 'wo, BUF>
where
    BUF: BufferType,
{
    pub fn regular_offset(self) -> RegularOffsetBuffer<'buf, BUF> {
        if let IncrementedBuffer::RegularOffset(buffer) = self {
            buffer
        } else {
            panic!("The buffer has word offset.");
        }
    }

    pub fn word_offset(self) -> WordOffsetBuffer<'buf, 'wo, BUF> {
        if let IncrementedBuffer::WordOffset(buffer) = self {
            buffer
        } else {
            panic!("The buffer has regular offset.");
        }
    }
}

pub type IncrementedBufferStatic<'wo, BUF> =
    IncrementedBuffer<'static, 'wo, BUF>;

pub enum IncrementedBufferMut<'buf, 'wo, BUF>
where
    BUF: BufferType,
{
    RegularOffset(RegularOffsetBufferMut<'buf, BUF>),
    WordOffset(WordOffsetBufferMut<'buf, 'wo, BUF>),
}

impl<'buf, 'wo, BUF> IncrementedBufferMut<'buf, 'wo, BUF>
where
    BUF: BufferType,
{
    pub fn into_regular_offset(self) -> RegularOffsetBufferMut<'buf, BUF> {
        if let IncrementedBufferMut::RegularOffset(buffer) = self {
            buffer
        } else {
            panic!("The buffer has word offset.");
        }
    }

    pub fn as_regular_offset(&self) -> &RegularOffsetBufferMut<'buf, BUF> {
        if let IncrementedBufferMut::RegularOffset(buffer) = self {
            buffer
        } else {
            panic!("The buffer has word offset.");
        }
    }

    pub fn as_mut_regular_offset(
        &mut self,
    ) -> &mut RegularOffsetBufferMut<'buf, BUF> {
        if let IncrementedBufferMut::RegularOffset(buffer) = self {
            buffer
        } else {
            panic!("The buffer has word offset.");
        }
    }

    pub fn into_word_offset(self) -> WordOffsetBufferMut<'buf, 'wo, BUF> {
        if let IncrementedBufferMut::WordOffset(buffer) = self {
            buffer
        } else {
            panic!("The buffer has regular offset.");
        }
    }

    pub fn as_word_offset(&self) -> &WordOffsetBufferMut<'buf, 'wo, BUF> {
        if let IncrementedBufferMut::WordOffset(buffer) = self {
            buffer
        } else {
            panic!("The buffer has regular offset.");
        }
    }

    pub fn as_mut_word_offset(
        &mut self,
    ) -> &mut WordOffsetBufferMut<'buf, 'wo, BUF> {
        if let IncrementedBufferMut::WordOffset(buffer) = self {
            buffer
        } else {
            panic!("The buffer has regular offset.");
        }
    }
}

pub type IncrementedBufferMutStatic<'wo, BUF> =
    IncrementedBufferMut<'static, 'wo, BUF>;

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct FixedBuffer<'buf, BUF>(*const BUF, PhantomData<&'buf BUF>)
where
    BUF: BufferType;

impl<'buf, BUF> FixedBuffer<'buf, BUF>
where
    BUF: BufferType,
{
    pub fn new(buffer: &'buf BUF) -> Self {
        FixedBuffer(buffer, PhantomData)
    }

    pub fn get(self) -> BUF {
        unsafe { self.0.read_volatile() }
    }

    pub fn as_ptr(self) -> *const BUF {
        self.0
    }
}

unsafe impl<'buf, BUF> Send for FixedBuffer<'buf, BUF> where BUF: BufferType {}

unsafe impl<'buf, BUF> Sync for FixedBuffer<'buf, BUF> where BUF: BufferType {}

pub type FixedBufferStatic<BUF> = FixedBuffer<'static, BUF>;

#[derive(Debug, PartialEq, Eq)]
pub struct FixedBufferMut<'buf, BUF>(*mut BUF, PhantomData<&'buf mut BUF>)
where
    BUF: BufferType;

impl<'buf, BUF> FixedBufferMut<'buf, BUF>
where
    BUF: BufferType,
{
    pub fn new(buffer: &'buf mut BUF) -> Self {
        FixedBufferMut(buffer, PhantomData)
    }

    /// # Safety
    ///
    /// - The caller must ensure, that the DMA is currently not writing this address.
    pub unsafe fn get(&self) -> BUF {
        ptr::read_volatile(self.0)
    }

    /// # Safety
    ///
    /// - The caller must ensure, that the DMA is currently not writing this address.
    pub unsafe fn set(&mut self, buf: BUF) {
        ptr::write_volatile(self.0, buf);
    }

    pub fn as_ptr(&self) -> *const BUF {
        self.0
    }

    pub fn as_mut_ptr(&mut self) -> *mut BUF {
        self.0
    }

    pub fn as_immutable(&self) -> FixedBuffer<BUF> {
        FixedBuffer(self.0, PhantomData)
    }
}

unsafe impl<'buf, BUF> Send for FixedBufferMut<'buf, BUF> where BUF: BufferType {}

unsafe impl<'buf, BUF> Sync for FixedBufferMut<'buf, BUF> where BUF: BufferType {}

pub type FixedBufferMutStatic<BUF> = FixedBufferMut<'static, BUF>;

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct RegularOffsetBuffer<'buf, BUF>(*const [BUF], PhantomData<&'buf BUF>)
where
    BUF: BufferType;

impl<'buf, BUF> RegularOffsetBuffer<'buf, BUF>
where
    BUF: BufferType,
{
    pub fn new(buffer: &'buf [BUF]) -> Self {
        RegularOffsetBuffer(buffer, PhantomData)
    }

    pub fn get(self, index: usize) -> BUF {
        unsafe { read_volatile_slice_buffer(self.0, index) }
    }

    pub fn as_ptr(self, index: usize) -> *const BUF {
        unsafe {
            let slice = &*self.0;
            &slice[index] as *const _
        }
    }
}

unsafe impl<'buf, BUF> Send for RegularOffsetBuffer<'buf, BUF> where
    BUF: BufferType
{
}

unsafe impl<'buf, BUF> Sync for RegularOffsetBuffer<'buf, BUF> where
    BUF: BufferType
{
}

pub type RegularOffsetBufferStatic<BUF> = RegularOffsetBuffer<'static, BUF>;

#[derive(Debug, PartialEq, Eq)]
pub struct RegularOffsetBufferMut<'buf, BUF>(
    *mut [BUF],
    PhantomData<&'buf mut BUF>,
)
where
    BUF: BufferType;

impl<'buf, BUF> RegularOffsetBufferMut<'buf, BUF>
where
    BUF: BufferType,
{
    pub fn new(buffer: &'buf mut [BUF]) -> Self {
        RegularOffsetBufferMut(buffer, PhantomData)
    }

    /// # Safety
    ///
    /// - The caller must ensure, that the DMA is currently not modifying this address.
    pub unsafe fn get(&self, index: usize) -> BUF {
        read_volatile_slice_buffer(self.0, index)
    }

    /// # Safety
    ///
    /// - The caller must ensure, that the DMA is currently not modifying this address.
    pub unsafe fn set(&mut self, index: usize, item: BUF) {
        let slice = &mut *self.0;
        ptr::write_volatile(&mut slice[index] as *mut _, item);
    }

    pub fn as_ptr(&self, index: usize) -> *const BUF {
        unsafe {
            let slice = &*self.0;
            &slice[index] as *const _
        }
    }

    pub fn as_mut_ptr(&mut self, index: usize) -> *mut BUF {
        unsafe {
            let slice = &mut *self.0;
            &mut slice[index] as *mut _
        }
    }

    pub fn as_immutable(&self) -> RegularOffsetBuffer<BUF> {
        RegularOffsetBuffer(self.0, PhantomData)
    }
}

unsafe impl<'buf, BUF> Send for RegularOffsetBufferMut<'buf, BUF> where
    BUF: BufferType
{
}

unsafe impl<'buf, BUF> Sync for RegularOffsetBufferMut<'buf, BUF> where
    BUF: BufferType
{
}

pub type RegularOffsetBufferMutStatic<BUF> =
    RegularOffsetBufferMut<'static, BUF>;

unsafe fn read_volatile_slice_buffer<BUF>(
    slice_ptr: *const [BUF],
    index: usize,
) -> BUF
where
    BUF: BufferType,
{
    let slice = &*slice_ptr;
    ptr::read_volatile(&slice[index] as *const _)
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct WordOffsetBuffer<'buf, 'wo, BUF>(
    &'wo [*const BUF],
    PhantomData<&'buf BUF>,
)
where
    BUF: BufferType;

impl<'buf, 'wo, BUF> WordOffsetBuffer<'buf, 'wo, BUF>
where
    BUF: BufferType,
{
    pub fn new(buffer: &'wo [&'buf BUF]) -> Self {
        let buffer = unsafe { &*(buffer as *const _ as *const _) };

        check_word_offset(buffer);

        WordOffsetBuffer(buffer, PhantomData)
    }

    pub fn get(self, index: usize) -> BUF {
        unsafe { ptr::read_volatile(self.0[index]) }
    }

    pub fn as_ptr(self, index: usize) -> *const BUF {
        self.0[index]
    }
}

unsafe impl<'buf, 'wo, BUF> Send for WordOffsetBuffer<'buf, 'wo, BUF> where
    BUF: BufferType
{
}

unsafe impl<'buf, 'wo, BUF> Sync for WordOffsetBuffer<'buf, 'wo, BUF> where
    BUF: BufferType
{
}

pub type WordOffsetBufferStatic<'wo, BUF> = WordOffsetBuffer<'static, 'wo, BUF>;

pub struct WordOffsetBufferMut<'buf, 'wo, BUF>(
    &'wo [&'buf mut VolatileCell<BUF>],
)
where
    BUF: BufferType;

impl<'buf, 'wo, BUF> WordOffsetBufferMut<'buf, 'wo, BUF>
where
    BUF: BufferType,
{
    pub fn new(buffer: &'wo [&'buf mut BUF]) -> Self {
        unsafe {
            check_word_offset::<BUF>(&*(buffer as *const _ as *const _));

            WordOffsetBufferMut(&*(buffer as *const _ as *const _))
        }
    }

    /// # Safety
    ///
    /// - The caller must ensure, that the DMA is currently not modifying this address.
    pub unsafe fn get(&self, index: usize) -> BUF {
        self.0[index].get()
    }

    /// # Safety
    ///
    /// - The caller must ensure, that the DMA is currently not modifying this address.
    pub unsafe fn set(&mut self, index: usize, item: BUF) {
        self.0[index].set(item);
    }

    pub fn as_ptr(&self, index: usize) -> *const BUF {
        self.0[index].as_ptr()
    }

    pub fn as_mut_ptr(&mut self, index: usize) -> *mut BUF {
        self.0[index].as_ptr()
    }

    pub fn as_immutable(&self) -> WordOffsetBuffer<'_, 'wo, BUF> {
        unsafe {
            WordOffsetBuffer(&*(self.0 as *const _ as *const _), PhantomData)
        }
    }
}

unsafe impl<'buf, 'wo, BUF> Sync for WordOffsetBufferMut<'buf, 'wo, BUF> where
    BUF: BufferType
{
}

pub type WordOffsetBufferMutStatic<'wo, BUF> =
    WordOffsetBufferMut<'static, 'wo, BUF>;

fn check_word_offset<BUF>(buffer: &[*const BUF])
where
    BUF: BufferType,
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
