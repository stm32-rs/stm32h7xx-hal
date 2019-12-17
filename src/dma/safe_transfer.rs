use super::stream::{MSize, PSize};
use core::convert::TryInto;
use core::fmt::Debug;
use core::marker::PhantomData;
use core::{mem, ptr};
use vcell::VolatileCell;

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

pub enum ImmutableBuffer<'buf, 'wo, BUF>
where
    BUF: BufferType,
{
    Memory(MemoryBuffer<'buf, BUF>),
    Peripheral(PeripheralBuffer<'buf, 'wo, BUF>),
}

pub enum MutableBuffer<'buf, 'wo, BUF>
where
    BUF: BufferType,
{
    Memory(MemoryBufferMut<'buf, BUF>),
    Peripheral(PeripheralBufferMut<'buf, 'wo, BUF>),
}

pub enum MemoryBuffer<'buf, BUF>
where
    BUF: BufferType,
{
    Fixed(FixedBuffer<'buf, BUF>),
    Incremented(RegularOffsetBuffer<'buf, BUF>),
}

pub enum MemoryBufferMut<'buf, BUF>
where
    BUF: BufferType,
{
    Fixed(FixedBufferMut<'buf, BUF>),
    Incremented(RegularOffsetBufferMut<'buf, BUF>),
}

pub enum PeripheralBuffer<'buf, 'wo, BUF>
where
    BUF: BufferType,
{
    Fixed(FixedBuffer<'buf, BUF>),
    Incremented(IncrementedBuffer<'buf, 'wo, BUF>),
}

pub enum PeripheralBufferMut<'buf, 'wo, BUF>
where
    BUF: BufferType,
{
    Fixed(FixedBufferMut<'buf, BUF>),
    Incremented(IncrementedBufferMut<'buf, 'wo, BUF>),
}

pub enum IncrementedBuffer<'buf, 'wo, BUF>
where
    BUF: BufferType,
{
    RegularOffset(RegularOffsetBuffer<'buf, BUF>),
    WordOffset(WordOffsetBuffer<'buf, 'wo, BUF>),
}

pub enum IncrementedBufferMut<'buf, 'wo, BUF>
where
    BUF: BufferType,
{
    RegularOffset(RegularOffsetBufferMut<'buf, BUF>),
    WordOffset(WordOffsetBufferMut<'buf, 'wo, BUF>),
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct FixedBuffer<'buf, BUF>(*const BUF, PhantomData<&'buf BUF>)
where
    BUF: BufferType;

impl<'buf, BUF> FixedBuffer<'buf, BUF>
where
    BUF: BufferType,
{
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

impl<BUF> FixedBufferStatic<BUF>
where
    BUF: BufferType,
{
    pub fn new(buffer: &'static BUF) -> Self {
        FixedBuffer(buffer, PhantomData)
    }
}

#[derive(Debug, PartialEq, Eq)]
pub struct FixedBufferMut<'buf, BUF>(*mut BUF, PhantomData<&'buf mut BUF>)
where
    BUF: BufferType;

impl<BUF> FixedBufferMut<'_, BUF>
where
    BUF: BufferType,
{
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

impl<BUF> FixedBufferMutStatic<BUF>
where
    BUF: BufferType,
{
    pub fn new(buffer: &'static mut BUF) -> Self {
        FixedBufferMut(buffer, PhantomData)
    }
}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct RegularOffsetBuffer<'buf, BUF>(*const [BUF], PhantomData<&'buf BUF>)
where
    BUF: BufferType;

impl<'buf, BUF> RegularOffsetBuffer<'buf, BUF>
where
    BUF: BufferType,
{
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

impl<BUF> RegularOffsetBufferStatic<BUF>
where
    BUF: BufferType,
{
    pub fn new(buffer: &'static [BUF]) -> Self {
        RegularOffsetBuffer(buffer, PhantomData)
    }
}

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

impl<BUF> RegularOffsetBufferMutStatic<BUF>
where
    BUF: BufferType,
{
    pub fn new(buffer: &'static mut [BUF]) -> Self {
        RegularOffsetBufferMut(buffer, PhantomData)
    }
}

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

impl<'wo, BUF> WordOffsetBufferStatic<'wo, BUF>
where
    BUF: BufferType,
{
    pub fn new(buffer: &'wo [&'static BUF]) -> Self {
        let buffer = unsafe { &*(buffer as *const _ as *const _) };

        check_word_offset(buffer);

        WordOffsetBuffer(buffer, PhantomData)
    }
}

pub struct WordOffsetBufferMut<'buf, 'wo, BUF>(
    &'wo [VolatileCell<BUF>],
    PhantomData<&'buf mut BUF>,
)
where
    BUF: BufferType;

impl<'buf, 'wo, BUF> WordOffsetBufferMut<'buf, 'wo, BUF>
where
    BUF: BufferType,
{
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

impl<'wo, BUF> WordOffsetBufferMutStatic<'wo, BUF>
where
    BUF: BufferType,
{
    pub fn new(buffer: &'wo [&'static mut BUF]) -> Self {
        unsafe {
            check_word_offset::<BUF>(&*(buffer as *const _ as *const _));

            WordOffsetBufferMut(&*(buffer as *const _ as *const _), PhantomData)
        }
    }
}

fn check_word_offset<BUF>(buffer: &[*const BUF])
where
    BUF: BufferType,
{
    if buffer.len() > 0 {
        let mut last_pointer = buffer[0] as *const _ as *const u32;

        for i in 1..buffer.len() {
            let current_pointer = buffer[i] as *const _ as *const u32;
            let current_address = current_pointer as usize;
            let expected_address = unsafe { last_pointer.add(1) } as usize;
            if current_address != expected_address {
                panic!("The offset must be one word (4 bytes).");
            }

            last_pointer = current_pointer;
        }
    }
}
