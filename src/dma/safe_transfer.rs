use super::stream::{MSize, PSize};
use core::convert::TryInto;
use core::fmt::Debug;
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

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct FixedBuffer<BUF>(*const BUF)
where
    BUF: BufferType;

impl<BUF> FixedBuffer<BUF>
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

unsafe impl<BUF> Send for FixedBuffer<BUF> where BUF: BufferType {}

unsafe impl<BUF> Sync for FixedBuffer<BUF> where BUF: BufferType {}

#[derive(Debug, PartialEq, Eq)]
pub struct FixedBufferMut<BUF>(*mut BUF)
where
    BUF: BufferType;

impl<BUF> FixedBufferMut<BUF>
where
    BUF: BufferType,
{
    pub fn get(&self) -> BUF {
        unsafe { ptr::read_volatile(self.0) }
    }

    pub fn set(&mut self, buf: BUF) {
        unsafe {
            ptr::write_volatile(self.0, buf);
        }
    }

    pub fn as_ptr(&self) -> *const BUF {
        self.0
    }

    pub fn as_mut_ptr(&mut self) -> *mut BUF {
        self.0
    }
}

unsafe impl<BUF> Send for FixedBufferMut<BUF> where BUF: BufferType {}

unsafe impl<BUF> Sync for FixedBufferMut<BUF> where BUF: BufferType {}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub struct RegularOffsetBuffer<BUF>(*const [BUF])
where
    BUF: BufferType;

impl<BUF> RegularOffsetBuffer<BUF>
where
    BUF: BufferType,
{
    pub fn get(self, index: usize) -> BUF {
        unsafe { volatile_read_buffer_slice(self.0, index) }
    }

    pub fn as_ptr(self, index: usize) -> *const BUF {
        unsafe {
            let slice = &*self.0;
            &slice[index] as *const _
        }
    }
}

unsafe impl<BUF> Send for RegularOffsetBuffer<BUF> where BUF: BufferType {}

unsafe impl<BUF> Sync for RegularOffsetBuffer<BUF> where BUF: BufferType {}

#[derive(Debug, PartialEq, Eq)]
pub struct RegularOffsetBufferMut<BUF>(*mut [BUF])
where
    BUF: BufferType;

impl<BUF> RegularOffsetBufferMut<BUF>
where
    BUF: BufferType,
{
    pub fn get(&self, index: usize) -> BUF {
        unsafe { volatile_read_buffer_slice(self.0, index) }
    }

    pub fn set(&mut self, index: usize, item: BUF) {
        unsafe {
            let slice = &mut *self.0;
            ptr::write_volatile(&mut slice[index] as *mut _, item);
        }
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
}

unsafe impl<BUF> Send for RegularOffsetBufferMut<BUF> where BUF: BufferType {}

unsafe impl<BUF> Sync for RegularOffsetBufferMut<BUF> where BUF: BufferType {}

unsafe fn volatile_read_buffer_slice<BUF>(
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
pub struct WordOffsetBuffer<'buf, BUF>(&'buf [*const BUF])
where
    BUF: BufferType;

impl<'buf, BUF> WordOffsetBuffer<'buf, BUF>
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

unsafe impl<'buf, BUF> Send for WordOffsetBuffer<'buf, BUF> where BUF: BufferType
{}

unsafe impl<'buf, BUF> Sync for WordOffsetBuffer<'buf, BUF> where BUF: BufferType
{}

pub struct WordOffsetBufferMut<'buf, BUF>(&'buf [VolatileCell<BUF>])
where
    BUF: BufferType;

impl<'buf, BUF> WordOffsetBufferMut<'buf, BUF>
where
    BUF: BufferType,
{
    pub fn get(&self, index: usize) -> BUF {
        self.0[index].get()
    }

    pub fn set(&mut self, index: usize, item: BUF) {
        self.0[index].set(item);
    }

    pub fn as_ptr(&self, index: usize) -> *const BUF {
        self.0[index].as_ptr()
    }

    pub fn as_mut_ptr(&mut self, index: usize) -> *mut BUF {
        self.0[index].as_ptr()
    }
}

unsafe impl<'buf, BUF> Sync for WordOffsetBufferMut<'buf, BUF> where
    BUF: BufferType
{
}
