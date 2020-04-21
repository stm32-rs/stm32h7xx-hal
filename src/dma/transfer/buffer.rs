use super::Payload;
use core::ptr;
use enum_as_inner::EnumAsInner;

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

#[allow(clippy::len_without_is_empty)]
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

#[allow(clippy::len_without_is_empty)]
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

#[allow(clippy::len_without_is_empty)]
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
