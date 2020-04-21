//! Safe DMA Transfers

pub mod buffer;
pub mod config;

use self::buffer::MemoryBufferType;
use super::stream::config::{MSize, PSize};
use super::stream::{Disabled, Enabled, IsrCleared, IsrUncleared};
use super::{ChannelId, Stream};
use crate::private;
use core::fmt::Debug;
use core::marker::PhantomData;
use core::mem;
use enum_as_inner::EnumAsInner;

pub use self::buffer::Buffer;
pub use self::config::Config;

pub struct Transfer<'wo, State: TransferState<'wo>> {
    state: State,
    _phantom: PhantomData<&'wo ()>,
}

impl<'wo, Peripheral, Memory> Transfer<'wo, Start<'wo, Peripheral, Memory>>
where
    Peripheral: Payload,
    Memory: Payload,
{
    pub fn new(conf: Config<'wo, Peripheral, Memory>) -> Self {
        Self {
            state: Start { conf },
            _phantom: PhantomData,
        }
    }

    pub fn start<CXX: ChannelId>(
        self,
        mut stream: Stream<CXX, Disabled, IsrCleared>,
    ) -> Transfer<'wo, Ongoing<'wo, Peripheral, Memory, CXX>> {
        self.configure_stream(&mut stream);

        Transfer {
            state: Ongoing {
                stream: unsafe { stream.enable() },
                buffers: self.state.conf.free().free(),
            },
            _phantom: PhantomData,
        }
    }

    fn configure_stream<CXX: ChannelId>(
        &self,
        stream: &mut Stream<CXX, Disabled, IsrCleared>,
    ) {
        let mut conf = stream.config();

        self.state.conf.stream_config(&mut conf);

        stream.apply_config(conf);
    }
}

pub trait TransferState<'wo>: Send + Sync + private::Sealed {
    type Peripheral: Payload;
    type Memory: Payload;

    fn buffers(&self) -> &TransferBuffers<'wo, Self::Peripheral, Self::Memory>;

    fn buffers_mut<F>(&mut self, op: F)
    where
        for<'a> F: FnOnce(
            &'a mut TransferBuffers<'wo, Self::Peripheral, Self::Memory>,
        );

    unsafe fn buffers_mut_unchecked(
        &mut self,
    ) -> &mut TransferBuffers<'wo, Self::Peripheral, Self::Memory>;
}

pub struct Start<'wo, Peripheral, Memory>
where
    Peripheral: Payload,
    Memory: Payload,
{
    conf: Config<'wo, Peripheral, Memory>,
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

    fn buffers(&self) -> &TransferBuffers<'wo, Peripheral, Memory> {
        self.conf.transfer_direction().buffers()
    }

    fn buffers_mut<F>(&mut self, op: F)
    where
        for<'a> F: FnOnce(&'a mut TransferBuffers<'wo, Peripheral, Memory>),
    {
        self.conf.transfer_direction_mut(|t| t.buffers_mut(op));
    }

    unsafe fn buffers_mut_unchecked(
        &mut self,
    ) -> &mut TransferBuffers<'wo, Peripheral, Memory> {
        self.conf
            .transfer_direction_mut_unchecked()
            .buffers_mut_unchecked()
    }
}

pub struct Ongoing<'wo, Peripheral, Memory, CXX>
where
    Peripheral: Payload,
    Memory: Payload,
    CXX: ChannelId,
{
    stream: Stream<CXX, Enabled, IsrUncleared>,
    buffers: TransferBuffers<'wo, Peripheral, Memory>,
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

    fn buffers(&self) -> &TransferBuffers<'wo, Peripheral, Memory> {
        &self.buffers
    }

    fn buffers_mut<F>(&mut self, op: F)
    where
        for<'a> F: FnOnce(&'a mut TransferBuffers<'wo, Peripheral, Memory>),
    {
        op(&mut self.buffers)
    }

    unsafe fn buffers_mut_unchecked(
        &mut self,
    ) -> &mut TransferBuffers<'wo, Peripheral, Memory> {
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

#[derive(Debug)]
pub struct Buffers<'wo, Peripheral, Memory>
where
    Peripheral: Payload,
    Memory: Payload,
{
    pub peripheral_buffer: Buffer<'wo, Peripheral>,
    pub memory_buffer: MemoryBufferType<Memory>,
}

#[derive(Debug)]
pub struct TransferBuffers<'wo, Peripheral, Memory>
where
    Peripheral: Payload,
    Memory: Payload,
{
    buffers: Buffers<'wo, Peripheral, Memory>,
}

impl<'wo, Peripheral, Memory> TransferBuffers<'wo, Peripheral, Memory>
where
    Peripheral: Payload,
    Memory: Payload,
{
    /// # Args
    ///
    /// * `peripheral_buffer`: Usually `Buffer::Peripheral`, except for `M2M`-transfers (Memory to Memory), where the source buffer is `Buffer::Memory`.
    /// * `memory_buffer`: The `MemoryBuffer` of the transfer.
    pub fn new(buffers: Buffers<'wo, Peripheral, Memory>) -> Self {
        let s = Self { buffers };

        s.check_self();

        s
    }

    pub fn get(&self) -> &Buffers<'wo, Peripheral, Memory> {
        &self.buffers
    }

    pub fn get_mut<F>(&mut self, op: F)
    where
        for<'a> F: FnOnce(&'a mut Buffers<'wo, Peripheral, Memory>),
    {
        op(&mut self.buffers);

        self.check_self();
    }

    pub unsafe fn get_mut_unchecked(
        &mut self,
    ) -> &mut Buffers<'wo, Peripheral, Memory> {
        &mut self.buffers
    }

    pub fn free(self) -> Buffers<'wo, Peripheral, Memory> {
        self.buffers
    }

    fn check_self(&self) {
        assert_ne!(
            self.buffers.peripheral_buffer.is_read(),
            self.buffers.memory_buffer.is_read()
        );
    }
}

#[derive(Copy, Clone, Debug, EnumAsInner)]
pub enum PayloadPort<Peripheral, Memory>
where
    Peripheral: Payload,
    Memory: Payload,
{
    Peripheral(Peripheral),
    Memory(Memory),
}

#[derive(Debug, EnumAsInner)]
pub enum PointerPort<Peripheral, Memory>
where
    Peripheral: Payload,
    Memory: Payload,
{
    Peripheral(*mut Peripheral),
    Memory(*mut Memory),
}
