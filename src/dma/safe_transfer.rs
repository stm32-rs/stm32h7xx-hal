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

#[derive(Clone, Copy)]
pub enum ImmutableBuffer<'buf, 'wo, P>
where
    P: Payload,
{
    Memory(MemoryBuffer<'buf, P>),
    Peripheral(PeripheralBuffer<'buf, 'wo, P>),
}

impl<'buf, 'wo, P> ImmutableBuffer<'buf, 'wo, P>
where
    P: Payload,
{
    pub fn memory(self) -> MemoryBuffer<'buf, P> {
        if let ImmutableBuffer::Memory(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a peripheral buffer.");
        }
    }

    pub fn peripheral(self) -> PeripheralBuffer<'buf, 'wo, P> {
        if let ImmutableBuffer::Peripheral(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a memory buffer.");
        }
    }
}

impl<'s, P> AsImmutable<'s> for ImmutableBuffer<'_, '_, P>
where
    P: Payload,
{
    type Target = ImmutableBuffer<'s, 's, P>;

    unsafe fn as_immutable(&self) -> ImmutableBuffer<P> {
        *self
    }
}

pub type ImmutableBufferStatic<'wo, P> = ImmutableBuffer<'static, 'wo, P>;

pub enum MutableBuffer<'buf, 'wo, P>
where
    P: Payload,
{
    Memory(MemoryBufferMut<'buf, P>),
    Peripheral(PeripheralBufferMut<'buf, 'wo, P>),
}

impl<'buf, 'wo, P> MutableBuffer<'buf, 'wo, P>
where
    P: Payload,
{
    pub fn into_memory(self) -> MemoryBufferMut<'buf, P> {
        if let MutableBuffer::Memory(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a peripheral buffer.");
        }
    }

    pub fn as_memory(&self) -> &MemoryBufferMut<'buf, P> {
        if let MutableBuffer::Memory(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a peripheral buffer.");
        }
    }

    pub fn as_mut_memory(&mut self) -> &mut MemoryBufferMut<'buf, P> {
        if let MutableBuffer::Memory(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a peripheral buffer.");
        }
    }

    pub fn into_peripheral(self) -> PeripheralBufferMut<'buf, 'wo, P> {
        if let MutableBuffer::Peripheral(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a memory buffer.");
        }
    }

    pub fn as_peripheral(&self) -> &PeripheralBufferMut<'buf, 'wo, P> {
        if let MutableBuffer::Peripheral(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a memory buffer.");
        }
    }

    pub fn as_mut_peripheral(
        &mut self,
    ) -> &mut PeripheralBufferMut<'buf, 'wo, P> {
        if let MutableBuffer::Peripheral(buffer) = self {
            buffer
        } else {
            panic!("The buffer is a memory buffer.");
        }
    }
}

impl<'s, P> AsImmutable<'s> for MutableBuffer<'_, '_, P>
where
    P: Payload,
{
    type Target = ImmutableBuffer<'s, 's, P>;

    unsafe fn as_immutable(&'s self) -> ImmutableBuffer<P> {
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

pub type MutableBufferStatic<'wo, P> = MutableBuffer<'static, 'wo, P>;

#[derive(Clone, Copy)]
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
    pub fn fixed(self) -> FixedBuffer<'buf, P> {
        if let MemoryBuffer::Fixed(buffer) = self {
            buffer
        } else {
            panic!("The buffer is incremented.");
        }
    }

    pub fn incremented(self) -> RegularOffsetBuffer<'buf, P> {
        if let MemoryBuffer::Incremented(buffer) = self {
            buffer
        } else {
            panic!("The buffer is fixed.");
        }
    }
}

impl<'s, P> AsImmutable<'s> for MemoryBuffer<'_, P>
where
    P: Payload,
{
    type Target = MemoryBuffer<'s, P>;

    unsafe fn as_immutable(&self) -> MemoryBuffer<P> {
        *self
    }
}

pub type MemoryBufferStatic<P> = MemoryBuffer<'static, P>;

pub enum MemoryBufferMut<'buf, P>
where
    P: Payload,
{
    Fixed(FixedBufferMut<'buf, P>),
    Incremented(RegularOffsetBufferMut<'buf, P>),
}

impl<'buf, P> MemoryBufferMut<'buf, P>
where
    P: Payload,
{
    pub fn into_fixed(self) -> FixedBufferMut<'buf, P> {
        if let MemoryBufferMut::Fixed(buffer) = self {
            buffer
        } else {
            panic!("The buffer is incremented.");
        }
    }

    pub fn as_fixed(&self) -> &FixedBufferMut<'buf, P> {
        if let MemoryBufferMut::Fixed(buffer) = self {
            buffer
        } else {
            panic!("The buffer is incremented.");
        }
    }

    pub fn as_mut_fixed(&mut self) -> &mut FixedBufferMut<'buf, P> {
        if let MemoryBufferMut::Fixed(buffer) = self {
            buffer
        } else {
            panic!("The buffer is incremented.");
        }
    }

    pub fn into_incremented(self) -> RegularOffsetBufferMut<'buf, P> {
        if let MemoryBufferMut::Incremented(buffer) = self {
            buffer
        } else {
            panic!("The buffer is fixed.");
        }
    }

    pub fn as_incremented(&self) -> &RegularOffsetBufferMut<'buf, P> {
        if let MemoryBufferMut::Incremented(buffer) = self {
            buffer
        } else {
            panic!("The buffer is fixed.");
        }
    }

    pub fn as_mut_incremented(
        &mut self,
    ) -> &mut RegularOffsetBufferMut<'buf, P> {
        if let MemoryBufferMut::Incremented(buffer) = self {
            buffer
        } else {
            panic!("The buffer is fixed.");
        }
    }
}

impl<'s, P> AsImmutable<'s> for MemoryBufferMut<'_, P>
where
    P: Payload,
{
    type Target = MemoryBuffer<'s, P>;

    unsafe fn as_immutable(&self) -> MemoryBuffer<P> {
        match self {
            MemoryBufferMut::Fixed(buffer) => {
                MemoryBuffer::Fixed(buffer.as_immutable())
            }
            MemoryBufferMut::Incremented(buffer) => {
                MemoryBuffer::Incremented(buffer.as_immutable())
            }
        }
    }
}

pub type MemoryBufferMutStatic<P> = MemoryBufferMut<'static, P>;

#[derive(Clone, Copy)]
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
    pub fn fixed(self) -> FixedBuffer<'buf, P> {
        if let PeripheralBuffer::Fixed(buffer) = self {
            buffer
        } else {
            panic!("The buffer is incremented.");
        }
    }

    pub fn incremented(self) -> IncrementedBuffer<'buf, 'wo, P> {
        if let PeripheralBuffer::Incremented(buffer) = self {
            buffer
        } else {
            panic!("The buffer is fixed.");
        }
    }
}

impl<'s, P> AsImmutable<'s> for PeripheralBuffer<'_, '_, P>
where
    P: Payload,
{
    type Target = PeripheralBuffer<'s, 's, P>;

    unsafe fn as_immutable(&self) -> PeripheralBuffer<P> {
        *self
    }
}

pub type PeripheralBufferStatic<'wo, P> = PeripheralBuffer<'static, 'wo, P>;

pub enum PeripheralBufferMut<'buf, 'wo, P>
where
    P: Payload,
{
    Fixed(FixedBufferMut<'buf, P>),
    Incremented(IncrementedBufferMut<'buf, 'wo, P>),
}

impl<'buf, 'wo, P> PeripheralBufferMut<'buf, 'wo, P>
where
    P: Payload,
{
    pub fn into_fixed(self) -> FixedBufferMut<'buf, P> {
        if let PeripheralBufferMut::Fixed(buffer) = self {
            buffer
        } else {
            panic!("The buffer is incremented.");
        }
    }

    pub fn as_fixed(&self) -> &FixedBufferMut<'buf, P> {
        if let PeripheralBufferMut::Fixed(buffer) = self {
            buffer
        } else {
            panic!("The buffer is incremented.");
        }
    }

    pub fn as_mut_fixed(&mut self) -> &mut FixedBufferMut<'buf, P> {
        if let PeripheralBufferMut::Fixed(buffer) = self {
            buffer
        } else {
            panic!("The buffer is incremented.");
        }
    }

    pub fn into_incremented(self) -> IncrementedBufferMut<'buf, 'wo, P> {
        if let PeripheralBufferMut::Incremented(buffer) = self {
            buffer
        } else {
            panic!("The buffer is fixed.");
        }
    }

    pub fn as_incremented(&self) -> &IncrementedBufferMut<'buf, 'wo, P> {
        if let PeripheralBufferMut::Incremented(buffer) = self {
            buffer
        } else {
            panic!("The buffer is fixed.");
        }
    }

    pub fn as_mut_incremented(
        &mut self,
    ) -> &mut IncrementedBufferMut<'buf, 'wo, P> {
        if let PeripheralBufferMut::Incremented(buffer) = self {
            buffer
        } else {
            panic!("The buffer is fixed.");
        }
    }
}

impl<'s, P> AsImmutable<'s> for PeripheralBufferMut<'_, '_, P>
where
    P: Payload,
{
    type Target = PeripheralBuffer<'s, 's, P>;

    unsafe fn as_immutable(&self) -> PeripheralBuffer<P> {
        match self {
            PeripheralBufferMut::Fixed(buffer) => {
                PeripheralBuffer::Fixed(buffer.as_immutable())
            }
            PeripheralBufferMut::Incremented(buffer) => {
                PeripheralBuffer::Incremented(buffer.as_immutable())
            }
        }
    }
}

pub type PeripheralBufferMutStatic<'wo, P> =
    PeripheralBufferMut<'static, 'wo, P>;

#[derive(Clone, Copy)]
pub enum IncrementedBuffer<'buf, 'wo, P>
where
    P: Payload,
{
    RegularOffset(RegularOffsetBuffer<'buf, P>),
    WordOffset(WordOffsetBuffer<'buf, 'wo, P>),
}

#[allow(clippy::len_without_is_empty)]
impl<'buf, 'wo, P> IncrementedBuffer<'buf, 'wo, P>
where
    P: Payload,
{
    pub fn regular_offset(self) -> RegularOffsetBuffer<'buf, P> {
        if let IncrementedBuffer::RegularOffset(buffer) = self {
            buffer
        } else {
            panic!("The buffer has word offset.");
        }
    }

    pub fn word_offset(self) -> WordOffsetBuffer<'buf, 'wo, P> {
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

    pub fn get(self, index: usize) -> P {
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

impl<'s, P> AsImmutable<'s> for IncrementedBuffer<'_, '_, P>
where
    P: Payload,
{
    type Target = IncrementedBuffer<'s, 's, P>;

    unsafe fn as_immutable(&self) -> IncrementedBuffer<P> {
        *self
    }
}

pub type IncrementedBufferStatic<'wo, P> = IncrementedBuffer<'static, 'wo, P>;

pub enum IncrementedBufferMut<'buf, 'wo, P>
where
    P: Payload,
{
    RegularOffset(RegularOffsetBufferMut<'buf, P>),
    WordOffset(WordOffsetBufferMut<'buf, 'wo, P>),
}

#[allow(clippy::len_without_is_empty)]
impl<'buf, 'wo, P> IncrementedBufferMut<'buf, 'wo, P>
where
    P: Payload,
{
    pub fn into_regular_offset(self) -> RegularOffsetBufferMut<'buf, P> {
        if let IncrementedBufferMut::RegularOffset(buffer) = self {
            buffer
        } else {
            panic!("The buffer has word offset.");
        }
    }

    pub fn as_regular_offset(&self) -> &RegularOffsetBufferMut<'buf, P> {
        if let IncrementedBufferMut::RegularOffset(buffer) = self {
            buffer
        } else {
            panic!("The buffer has word offset.");
        }
    }

    pub fn as_mut_regular_offset(
        &mut self,
    ) -> &mut RegularOffsetBufferMut<'buf, P> {
        if let IncrementedBufferMut::RegularOffset(buffer) = self {
            buffer
        } else {
            panic!("The buffer has word offset.");
        }
    }

    pub fn into_word_offset(self) -> WordOffsetBufferMut<'buf, 'wo, P> {
        if let IncrementedBufferMut::WordOffset(buffer) = self {
            buffer
        } else {
            panic!("The buffer has regular offset.");
        }
    }

    pub fn as_word_offset(&self) -> &WordOffsetBufferMut<'buf, 'wo, P> {
        if let IncrementedBufferMut::WordOffset(buffer) = self {
            buffer
        } else {
            panic!("The buffer has regular offset.");
        }
    }

    pub fn as_mut_word_offset(
        &mut self,
    ) -> &mut WordOffsetBufferMut<'buf, 'wo, P> {
        if let IncrementedBufferMut::WordOffset(buffer) = self {
            buffer
        } else {
            panic!("The buffer has regular offset.");
        }
    }

    pub fn len(&self) -> usize {
        match self {
            IncrementedBufferMut::RegularOffset(buffer) => buffer.len(),
            IncrementedBufferMut::WordOffset(buffer) => buffer.len(),
        }
    }

    /// # Safety
    ///
    /// The caller must ensure, that the DMA is currently not modifying this address.
    pub unsafe fn get(&self, index: usize) -> P {
        match self {
            IncrementedBufferMut::RegularOffset(buffer) => buffer.get(index),
            IncrementedBufferMut::WordOffset(buffer) => buffer.get(index),
        }
    }

    /// # Safety
    ///
    /// The caller must ensure, that the DMA is currently not modifying this address.
    pub unsafe fn set(&mut self, index: usize, payload: P) {
        match self {
            IncrementedBufferMut::RegularOffset(buffer) => {
                buffer.set(index, payload)
            }
            IncrementedBufferMut::WordOffset(buffer) => {
                buffer.set(index, payload)
            }
        }
    }

    pub fn as_ptr(&self, index: usize) -> *const P {
        match self {
            IncrementedBufferMut::RegularOffset(buffer) => buffer.as_ptr(index),
            IncrementedBufferMut::WordOffset(buffer) => buffer.as_ptr(index),
        }
    }

    pub fn as_mut_ptr(&mut self, index: usize) -> *mut P {
        match self {
            IncrementedBufferMut::RegularOffset(buffer) => {
                buffer.as_mut_ptr(index)
            }
            IncrementedBufferMut::WordOffset(buffer) => {
                buffer.as_mut_ptr(index)
            }
        }
    }
}

impl<'s, P> AsImmutable<'s> for IncrementedBufferMut<'_, '_, P>
where
    P: Payload,
{
    type Target = IncrementedBuffer<'s, 's, P>;

    /// # Safety
    ///
    /// `IncrementedBuffer` assumes that the DMA is only reading the buffer.
    /// Therefore the getters of the immutable version are as unsafe as the getters of this struct.
    unsafe fn as_immutable(&self) -> IncrementedBuffer<P> {
        match self {
            IncrementedBufferMut::RegularOffset(buffer) => {
                IncrementedBuffer::RegularOffset(buffer.as_immutable())
            }
            IncrementedBufferMut::WordOffset(buffer) => {
                IncrementedBuffer::WordOffset(buffer.as_immutable())
            }
        }
    }
}

pub type IncrementedBufferMutStatic<'wo, P> =
    IncrementedBufferMut<'static, 'wo, P>;

#[derive(Clone, Copy)]
pub struct FixedBuffer<'buf, P>(*const P, PhantomData<&'buf P>)
where
    P: Payload;

impl<'buf, P> FixedBuffer<'buf, P>
where
    P: Payload,
{
    pub fn new(buffer: &'buf P) -> Self {
        FixedBuffer(buffer, PhantomData)
    }

    pub fn get(self) -> P {
        unsafe { self.0.read_volatile() }
    }

    pub fn as_ptr(self) -> *const P {
        self.0
    }
}

impl<'s, P> AsImmutable<'s> for FixedBuffer<'_, P>
where
    P: Payload,
{
    type Target = FixedBuffer<'s, P>;

    unsafe fn as_immutable(&self) -> FixedBuffer<P> {
        *self
    }
}

unsafe impl<P> Send for FixedBuffer<'_, P> where P: Payload {}

unsafe impl<P> Sync for FixedBuffer<'_, P> where P: Payload {}

pub type FixedBufferStatic<P> = FixedBuffer<'static, P>;

pub struct FixedBufferMut<'buf, P>(*mut P, PhantomData<&'buf mut P>)
where
    P: Payload;

impl<'buf, P> FixedBufferMut<'buf, P>
where
    P: Payload,
{
    pub fn new(buffer: &'buf mut P) -> Self {
        FixedBufferMut(buffer, PhantomData)
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

impl<'s, P> AsImmutable<'s> for FixedBufferMut<'_, P>
where
    P: Payload,
{
    type Target = FixedBuffer<'s, P>;

    unsafe fn as_immutable(&self) -> FixedBuffer<P> {
        FixedBuffer(self.0, PhantomData)
    }
}

unsafe impl<P> Send for FixedBufferMut<'_, P> where P: Payload {}

unsafe impl<P> Sync for FixedBufferMut<'_, P> where P: Payload {}

pub type FixedBufferMutStatic<P> = FixedBufferMut<'static, P>;

#[derive(Clone, Copy)]
pub struct RegularOffsetBuffer<'buf, P>(*const [P], PhantomData<&'buf P>)
where
    P: Payload;

#[allow(clippy::len_without_is_empty)]
impl<'buf, P> RegularOffsetBuffer<'buf, P>
where
    P: Payload,
{
    pub fn new(buffer: &'buf [P]) -> Self {
        check_buffer_not_empty(buffer);

        RegularOffsetBuffer(buffer, PhantomData)
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

impl<'s, P> AsImmutable<'s> for RegularOffsetBuffer<'_, P>
where
    P: Payload,
{
    type Target = RegularOffsetBuffer<'s, P>;

    unsafe fn as_immutable(&self) -> RegularOffsetBuffer<P> {
        *self
    }
}

unsafe impl<P> Send for RegularOffsetBuffer<'_, P> where P: Payload {}

unsafe impl<P> Sync for RegularOffsetBuffer<'_, P> where P: Payload {}

pub type RegularOffsetBufferStatic<P> = RegularOffsetBuffer<'static, P>;

pub struct RegularOffsetBufferMut<'buf, P>(*mut [P], PhantomData<&'buf mut P>)
where
    P: Payload;

#[allow(clippy::len_without_is_empty)]
impl<'buf, P> RegularOffsetBufferMut<'buf, P>
where
    P: Payload,
{
    pub fn new(buffer: &'buf mut [P]) -> Self {
        check_buffer_not_empty(buffer);

        RegularOffsetBufferMut(buffer, PhantomData)
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

impl<'s, P> AsImmutable<'s> for RegularOffsetBufferMut<'_, P>
where
    P: Payload,
{
    type Target = RegularOffsetBuffer<'s, P>;

    unsafe fn as_immutable(&self) -> RegularOffsetBuffer<P> {
        RegularOffsetBuffer(self.0, PhantomData)
    }
}

unsafe impl<P> Send for RegularOffsetBufferMut<'_, P> where P: Payload {}

unsafe impl<P> Sync for RegularOffsetBufferMut<'_, P> where P: Payload {}

pub type RegularOffsetBufferMutStatic<P> = RegularOffsetBufferMut<'static, P>;

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

#[derive(Clone, Copy)]
pub struct WordOffsetBuffer<'buf, 'wo, P>(
    &'wo [*const P],
    PhantomData<&'buf P>,
)
where
    P: Payload;

#[allow(clippy::len_without_is_empty)]
impl<'buf, 'wo, P> WordOffsetBuffer<'buf, 'wo, P>
where
    P: Payload,
{
    pub fn new(buffer: &'wo [&'buf P]) -> Self {
        check_buffer_not_empty(buffer);

        let buffer = unsafe { &*(buffer as *const _ as *const _) };

        check_word_offset(buffer);

        WordOffsetBuffer(buffer, PhantomData)
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

impl<'s, P> AsImmutable<'s> for WordOffsetBuffer<'_, '_, P>
where
    P: Payload,
{
    type Target = WordOffsetBuffer<'s, 's, P>;

    unsafe fn as_immutable(&self) -> WordOffsetBuffer<P> {
        *self
    }
}

unsafe impl<'buf, 'wo, P> Send for WordOffsetBuffer<'buf, 'wo, P> where
    P: Payload
{
}

unsafe impl<'buf, 'wo, P> Sync for WordOffsetBuffer<'buf, 'wo, P> where
    P: Payload
{
}

pub type WordOffsetBufferStatic<'wo, P> = WordOffsetBuffer<'static, 'wo, P>;

pub struct WordOffsetBufferMut<'buf, 'wo, P>(
    &'wo mut [*mut P],
    PhantomData<&'buf mut P>,
)
where
    P: Payload;

#[allow(clippy::len_without_is_empty)]
impl<'buf, 'wo, P> WordOffsetBufferMut<'buf, 'wo, P>
where
    P: Payload,
{
    pub fn new(buffer: &'wo mut [&'buf mut P]) -> Self {
        check_buffer_not_empty(buffer);

        unsafe {
            check_word_offset::<P>(&*(buffer as *const _ as *const _));

            WordOffsetBufferMut(&mut *(buffer as *mut _ as *mut _), PhantomData)
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

impl<'s, P> AsImmutable<'s> for WordOffsetBufferMut<'_, '_, P>
where
    P: Payload,
{
    type Target = WordOffsetBuffer<'s, 's, P>;

    unsafe fn as_immutable(&self) -> WordOffsetBuffer<P> {
        WordOffsetBuffer(&*(self.0 as *const _ as *const _), PhantomData)
    }
}

unsafe impl<P> Send for WordOffsetBufferMut<'_, '_, P> where P: Payload {}

unsafe impl<P> Sync for WordOffsetBufferMut<'_, '_, P> where P: Payload {}

pub type WordOffsetBufferMutStatic<'wo, P> =
    WordOffsetBufferMut<'static, 'wo, P>;

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
    source: ImmutableBuffer<Source>,
    dest: &MutableBuffer<Dest>,
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
    peripheral: PeripheralBuffer<Peripheral>,
    memory: MemoryBuffer<Memory>,
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

    configure_ndt(stream, peripheral, memory);
}

fn configure_ndt<CXX, DMA, Peripheral, Memory>(
    stream: &mut Stream<CXX, DMA, Disabled, IsrCleared>,
    peripheral: PeripheralBuffer<Peripheral>,
    memory: MemoryBuffer<Memory>,
) where
    CXX: ChannelId<DMA = DMA>,
    DMA: DMATrait,
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
    MemBuf: AsImmutable<'s, Target = MemoryBuffer<'s, P>>,
    P: Payload,
{
    let double_buffer = unsafe {
        [
            double_buffer[0].as_immutable(),
            double_buffer[1].as_immutable(),
        ]
    };
    match double_buffer[0] {
        MemoryBuffer::Fixed(_) => {
            if let MemoryBuffer::Incremented(_) = double_buffer[1] {
                panic!("Invalid double buffer config: First buffer `Fixed`, second buffer `Incremented`.");
            }
        }
        MemoryBuffer::Incremented(buffer_0) => {
            if let MemoryBuffer::Fixed(_) = double_buffer[1] {
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
    ImmutBuf: AsImmutable<'s, Target = ImmutableBuffer<'s, 's, P>>,
    P: Payload,
{
    let buffer = unsafe { buffer.as_immutable() };

    match buffer {
        ImmutableBuffer::Peripheral(buffer) => match buffer {
            PeripheralBuffer::Fixed(buffer) => buffer.as_ptr(),
            PeripheralBuffer::Incremented(buffer) => buffer.as_ptr(0),
        },
        ImmutableBuffer::Memory(buffer) => match buffer {
            MemoryBuffer::Fixed(buffer) => buffer.as_ptr(),
            MemoryBuffer::Incremented(buffer) => buffer.as_ptr(0),
        },
    }
}
