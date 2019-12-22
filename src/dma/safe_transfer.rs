use super::channel::ChannelId;
use super::stream::{
    Disabled, Enabled, FlowController, IsrCleared, IsrUncleared, M0a, MSize,
    Minc, Ndt, PSize, Pa, Pinc, Pincos, TransferDirection, TransferMode,
};
use super::{DMATrait, Stream};
use core::convert::TryFrom;
use core::convert::TryInto;
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
    pub fn from_payload<BUF>() -> Self
    where
        BUF: Payload,
    {
        let size_bytes: usize = mem::size_of::<BUF>();

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

#[derive(Clone, Copy)]
pub enum ImmutableBuffer<'buf, 'wo, BUF>
where
    BUF: Payload,
{
    Memory(MemoryBuffer<'buf, BUF>),
    Peripheral(PeripheralBuffer<'buf, 'wo, BUF>),
}

impl<'buf, 'wo, BUF> ImmutableBuffer<'buf, 'wo, BUF>
where
    BUF: Payload,
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
    BUF: Payload,
{
    Memory(MemoryBufferMut<'buf, BUF>),
    Peripheral(PeripheralBufferMut<'buf, 'wo, BUF>),
}

impl<'buf, 'wo, BUF> MutableBuffer<'buf, 'wo, BUF>
where
    BUF: Payload,
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

    /// # Safety
    ///
    /// `ImmutableBuffer` assumes that the DMA is only reading the buffer.
    /// Therefore the getters of the immutable version are as unsafe as the getters of this struct.
    pub unsafe fn as_immutable(&self) -> ImmutableBuffer<BUF> {
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

pub type MutableBufferStatic<'wo, BUF> = MutableBuffer<'static, 'wo, BUF>;

#[derive(Clone, Copy)]
pub enum MemoryBuffer<'buf, BUF>
where
    BUF: Payload,
{
    Fixed(FixedBuffer<'buf, BUF>),
    Incremented(RegularOffsetBuffer<'buf, BUF>),
}

impl<'buf, BUF> MemoryBuffer<'buf, BUF>
where
    BUF: Payload,
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
    BUF: Payload,
{
    Fixed(FixedBufferMut<'buf, BUF>),
    Incremented(RegularOffsetBufferMut<'buf, BUF>),
}

impl<'buf, BUF> MemoryBufferMut<'buf, BUF>
where
    BUF: Payload,
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

    /// # Safety
    ///
    /// `MemoryBuffer` assumes that the DMA is only reading the buffer.
    /// Therefore the getters of the immutable version are as unsafe as the getters of this struct.
    pub unsafe fn as_immutable(&self) -> MemoryBuffer<BUF> {
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

pub type MemoryBufferMutStatic<BUF> = MemoryBufferMut<'static, BUF>;

#[derive(Clone, Copy)]
pub enum PeripheralBuffer<'buf, 'wo, BUF>
where
    BUF: Payload,
{
    Fixed(FixedBuffer<'buf, BUF>),
    Incremented(IncrementedBuffer<'buf, 'wo, BUF>),
}

impl<'buf, 'wo, BUF> PeripheralBuffer<'buf, 'wo, BUF>
where
    BUF: Payload,
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
    BUF: Payload,
{
    Fixed(FixedBufferMut<'buf, BUF>),
    Incremented(IncrementedBufferMut<'buf, 'wo, BUF>),
}

impl<'buf, 'wo, BUF> PeripheralBufferMut<'buf, 'wo, BUF>
where
    BUF: Payload,
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

    /// # Safety
    ///
    /// `PeripheralBuffer` assumes that the DMA is only reading the buffer.
    /// Therefore the getters of the immutable version are as unsafe as the getters of this struct.
    pub unsafe fn as_immutable(&self) -> PeripheralBuffer<BUF> {
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

pub type PeripheralBufferMutStatic<'wo, BUF> =
    PeripheralBufferMut<'static, 'wo, BUF>;

#[derive(Clone, Copy)]
pub enum IncrementedBuffer<'buf, 'wo, BUF>
where
    BUF: Payload,
{
    RegularOffset(RegularOffsetBuffer<'buf, BUF>),
    WordOffset(WordOffsetBuffer<'buf, 'wo, BUF>),
}

#[allow(clippy::len_without_is_empty)]
impl<'buf, 'wo, BUF> IncrementedBuffer<'buf, 'wo, BUF>
where
    BUF: Payload,
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

    pub fn len(self) -> usize {
        match self {
            IncrementedBuffer::RegularOffset(buffer) => buffer.len(),
            IncrementedBuffer::WordOffset(buffer) => buffer.len(),
        }
    }

    pub fn get(self, index: usize) -> BUF {
        match self {
            IncrementedBuffer::RegularOffset(buffer) => buffer.get(index),
            IncrementedBuffer::WordOffset(buffer) => buffer.get(index),
        }
    }

    pub fn as_ptr(&self, index: usize) -> *const BUF {
        match self {
            IncrementedBuffer::RegularOffset(buffer) => buffer.as_ptr(index),
            IncrementedBuffer::WordOffset(buffer) => buffer.as_ptr(index),
        }
    }
}

pub type IncrementedBufferStatic<'wo, BUF> =
    IncrementedBuffer<'static, 'wo, BUF>;

pub enum IncrementedBufferMut<'buf, 'wo, BUF>
where
    BUF: Payload,
{
    RegularOffset(RegularOffsetBufferMut<'buf, BUF>),
    WordOffset(WordOffsetBufferMut<'buf, 'wo, BUF>),
}

#[allow(clippy::len_without_is_empty)]
impl<'buf, 'wo, BUF> IncrementedBufferMut<'buf, 'wo, BUF>
where
    BUF: Payload,
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

    pub fn len(&self) -> usize {
        match self {
            IncrementedBufferMut::RegularOffset(buffer) => buffer.len(),
            IncrementedBufferMut::WordOffset(buffer) => buffer.len(),
        }
    }

    /// # Safety
    ///
    /// The caller must ensure, that the DMA is currently not modifying this address.
    pub unsafe fn get(&self, index: usize) -> BUF {
        match self {
            IncrementedBufferMut::RegularOffset(buffer) => buffer.get(index),
            IncrementedBufferMut::WordOffset(buffer) => buffer.get(index),
        }
    }

    /// # Safety
    ///
    /// The caller must ensure, that the DMA is currently not modifying this address.
    pub unsafe fn set(&mut self, index: usize, payload: BUF) {
        match self {
            IncrementedBufferMut::RegularOffset(buffer) => {
                buffer.set(index, payload)
            }
            IncrementedBufferMut::WordOffset(buffer) => {
                buffer.set(index, payload)
            }
        }
    }

    pub fn as_ptr(&self, index: usize) -> *const BUF {
        match self {
            IncrementedBufferMut::RegularOffset(buffer) => buffer.as_ptr(index),
            IncrementedBufferMut::WordOffset(buffer) => buffer.as_ptr(index),
        }
    }

    pub fn as_mut_ptr(&mut self, index: usize) -> *mut BUF {
        match self {
            IncrementedBufferMut::RegularOffset(buffer) => {
                buffer.as_mut_ptr(index)
            }
            IncrementedBufferMut::WordOffset(buffer) => {
                buffer.as_mut_ptr(index)
            }
        }
    }

    /// # Safety
    ///
    /// `IncrementedBuffer` assumes that the DMA is only reading the buffer.
    /// Therefore the getters of the immutable version are as unsafe as the getters of this struct.
    pub unsafe fn as_immutable(&self) -> IncrementedBuffer<BUF> {
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

pub type IncrementedBufferMutStatic<'wo, BUF> =
    IncrementedBufferMut<'static, 'wo, BUF>;

#[derive(Clone, Copy)]
pub struct FixedBuffer<'buf, BUF>(*const BUF, PhantomData<&'buf BUF>)
where
    BUF: Payload;

impl<'buf, BUF> FixedBuffer<'buf, BUF>
where
    BUF: Payload,
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

unsafe impl<BUF> Send for FixedBuffer<'_, BUF> where BUF: Payload {}

unsafe impl<BUF> Sync for FixedBuffer<'_, BUF> where BUF: Payload {}

pub type FixedBufferStatic<BUF> = FixedBuffer<'static, BUF>;

pub struct FixedBufferMut<'buf, BUF>(*mut BUF, PhantomData<&'buf mut BUF>)
where
    BUF: Payload;

impl<'buf, BUF> FixedBufferMut<'buf, BUF>
where
    BUF: Payload,
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

    /// # Safety
    ///
    /// `FixedBuffer` assumes that the DMA is only reading the buffer.
    /// Therefore the getters of the immutable version are as unsafe as the getters of this struct.
    pub unsafe fn as_immutable(&self) -> FixedBuffer<BUF> {
        FixedBuffer(self.0, PhantomData)
    }
}

unsafe impl<BUF> Send for FixedBufferMut<'_, BUF> where BUF: Payload {}

unsafe impl<BUF> Sync for FixedBufferMut<'_, BUF> where BUF: Payload {}

pub type FixedBufferMutStatic<BUF> = FixedBufferMut<'static, BUF>;

#[derive(Clone, Copy)]
pub struct RegularOffsetBuffer<'buf, BUF>(*const [BUF], PhantomData<&'buf BUF>)
where
    BUF: Payload;

#[allow(clippy::len_without_is_empty)]
impl<'buf, BUF> RegularOffsetBuffer<'buf, BUF>
where
    BUF: Payload,
{
    pub fn new(buffer: &'buf [BUF]) -> Self {
        check_buffer_not_empty(buffer);

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

    pub fn len(self) -> usize {
        unsafe {
            let slice = &*self.0;
            slice.len()
        }
    }
}

unsafe impl<BUF> Send for RegularOffsetBuffer<'_, BUF> where BUF: Payload {}

unsafe impl<BUF> Sync for RegularOffsetBuffer<'_, BUF> where BUF: Payload {}

pub type RegularOffsetBufferStatic<BUF> = RegularOffsetBuffer<'static, BUF>;

pub struct RegularOffsetBufferMut<'buf, BUF>(
    *mut [BUF],
    PhantomData<&'buf mut BUF>,
)
where
    BUF: Payload;

#[allow(clippy::len_without_is_empty)]
impl<'buf, BUF> RegularOffsetBufferMut<'buf, BUF>
where
    BUF: Payload,
{
    pub fn new(buffer: &'buf mut [BUF]) -> Self {
        check_buffer_not_empty(buffer);

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

    pub fn len(&self) -> usize {
        unsafe {
            let slice = &*self.0;
            slice.len()
        }
    }

    /// # Safety
    ///
    /// `RegularOffsetBuffer` assumes that the DMA is only reading the buffer.
    /// Therefore the getters of the immutable version are as unsafe as the getters of this struct.
    pub unsafe fn as_immutable(&self) -> RegularOffsetBuffer<BUF> {
        RegularOffsetBuffer(self.0, PhantomData)
    }
}

unsafe impl<BUF> Send for RegularOffsetBufferMut<'_, BUF> where BUF: Payload {}

unsafe impl<BUF> Sync for RegularOffsetBufferMut<'_, BUF> where BUF: Payload {}

pub type RegularOffsetBufferMutStatic<BUF> =
    RegularOffsetBufferMut<'static, BUF>;

unsafe fn read_volatile_slice_buffer<BUF>(
    slice_ptr: *const [BUF],
    index: usize,
) -> BUF
where
    BUF: Payload,
{
    let slice = &*slice_ptr;
    ptr::read_volatile(&slice[index] as *const _)
}

#[derive(Clone, Copy)]
pub struct WordOffsetBuffer<'buf, 'wo, BUF>(
    &'wo [*const BUF],
    PhantomData<&'buf BUF>,
)
where
    BUF: Payload;

#[allow(clippy::len_without_is_empty)]
impl<'buf, 'wo, BUF> WordOffsetBuffer<'buf, 'wo, BUF>
where
    BUF: Payload,
{
    pub fn new(buffer: &'wo [&'buf BUF]) -> Self {
        check_buffer_not_empty(buffer);

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

    pub fn len(self) -> usize {
        self.0.len()
    }
}

unsafe impl<'buf, 'wo, BUF> Send for WordOffsetBuffer<'buf, 'wo, BUF> where
    BUF: Payload
{
}

unsafe impl<'buf, 'wo, BUF> Sync for WordOffsetBuffer<'buf, 'wo, BUF> where
    BUF: Payload
{
}

pub type WordOffsetBufferStatic<'wo, BUF> = WordOffsetBuffer<'static, 'wo, BUF>;

pub struct WordOffsetBufferMut<'buf, 'wo, BUF>(
    &'wo mut [*mut BUF],
    PhantomData<&'buf mut BUF>,
)
where
    BUF: Payload;

#[allow(clippy::len_without_is_empty)]
impl<'buf, 'wo, BUF> WordOffsetBufferMut<'buf, 'wo, BUF>
where
    BUF: Payload,
{
    pub fn new(buffer: &'wo mut [&'buf mut BUF]) -> Self {
        check_buffer_not_empty(buffer);

        unsafe {
            check_word_offset::<BUF>(&*(buffer as *const _ as *const _));

            WordOffsetBufferMut(&mut *(buffer as *mut _ as *mut _), PhantomData)
        }
    }

    /// # Safety
    ///
    /// The caller must ensure, that the DMA is currently not modifying this address.
    pub unsafe fn get(&self, index: usize) -> BUF {
        ptr::read_volatile(self.0[index])
    }

    /// # Safety
    ///
    /// The caller must ensure, that the DMA is currently not modifying this address.
    pub unsafe fn set(&mut self, index: usize, item: BUF) {
        ptr::write_volatile(self.0[index], item);
    }

    pub fn as_ptr(&self, index: usize) -> *const BUF {
        self.0[index]
    }

    pub fn as_mut_ptr(&mut self, index: usize) -> *mut BUF {
        self.0[index]
    }

    pub fn len(&self) -> usize {
        self.0.len()
    }

    /// # Safety
    ///
    /// `WordOffsetBuffer` assumes that the DMA is only reading the buffer.
    /// Therefore the getters of the immutable version are as unsafe as the getters of this struct.
    pub unsafe fn as_immutable(&self) -> WordOffsetBuffer<BUF> {
        WordOffsetBuffer(&*(self.0 as *const _ as *const _), PhantomData)
    }
}

unsafe impl<BUF> Send for WordOffsetBufferMut<'_, '_, BUF> where BUF: Payload {}

unsafe impl<BUF> Sync for WordOffsetBufferMut<'_, '_, BUF> where BUF: Payload {}

pub type WordOffsetBufferMutStatic<'wo, BUF> =
    WordOffsetBufferMut<'static, 'wo, BUF>;

fn check_buffer_not_empty<T>(buffer: &[T]) {
    if buffer.is_empty() {
        panic!("The buffer must not be empty.");
    }
}

//
// Secure Transfer implementations
//

fn check_word_offset<BUF>(buffer: &[*const BUF])
where
    BUF: Payload,
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
}

pub(super) fn check_double_buffer_mut<BUF>(
    double_buffer: &[MemoryBufferMut<BUF>; 2],
) where
    BUF: Payload,
{
    unsafe {
        let [buffer_0, buffer_1] = double_buffer;
        let double_buffer_immutable =
            [buffer_0.as_immutable(), buffer_1.as_immutable()];

        check_double_buffer(double_buffer_immutable);
    }
}

pub(super) fn check_double_buffer<BUF>(double_buffer: [MemoryBuffer<BUF>; 2])
where
    BUF: Payload,
{
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

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum DoubleBuffer {
    First,
    Second,
}

pub(super) fn double_buffer_idx(buffer: DoubleBuffer) -> usize {
    match buffer {
        DoubleBuffer::First => 0,
        DoubleBuffer::Second => 1,
    }
}
