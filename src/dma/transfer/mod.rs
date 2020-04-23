//! Safe DMA Transfers

pub mod buffer;
pub mod config;

use self::buffer::MemoryBufferType;
use super::stream::config::{MSize, PSize};
use super::stream::{
    Disabled, Enabled, Error as StreamError, Event, IsrCleared, IsrUncleared,
    StreamIsr,
};
use super::{ChannelId, Stream};
use crate::{nb, private};
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

    pub fn free(self) -> Config<'wo, Peripheral, Memory> {
        self.state.conf
    }
}

impl<'wo, Peripheral, Memory, CXX>
    Transfer<'wo, Ongoing<'wo, Peripheral, Memory, CXX>>
where
    Peripheral: Payload,
    Memory: Payload,
    CXX: ChannelId,
{
    pub fn stream(&self) -> &Stream<CXX, Enabled, IsrUncleared> {
        &self.state.stream
    }

    pub fn stream_mut(&mut self) -> &mut Stream<CXX, Enabled, IsrUncleared> {
        &mut self.state.stream
    }

    pub fn check_isr(
        &self,
        isr: &StreamIsr<CXX::DMA>,
    ) -> Result<Option<Event>, StreamError> {
        self.state.stream.check_isr(isr)
    }

    pub fn wait_until_completed(
        &self,
        isr: &StreamIsr<CXX::DMA>,
    ) -> nb::Result<(), StreamError> {
        self.state.stream.wait_until_completed(isr)
    }

    pub fn wait_until_completed_clear(
        &self,
        isr: &mut StreamIsr<CXX::DMA>,
    ) -> nb::Result<(), StreamError> {
        self.state.stream.wait_until_completed_clear(isr)
    }

    pub fn wait_until_half_transfer(
        &self,
        isr: &StreamIsr<CXX::DMA>,
    ) -> nb::Result<(), StreamError> {
        self.state.stream.wait_until_half_transfer(isr)
    }

    pub fn wait_until_half_transfer_clear(
        &self,
        isr: &mut StreamIsr<CXX::DMA>,
    ) -> nb::Result<(), StreamError> {
        self.state.stream.wait_until_half_transfer_clear(isr)
    }

    pub fn wait_until_next_half(
        &self,
        isr: &StreamIsr<CXX::DMA>,
    ) -> nb::Result<(), StreamError> {
        self.state.stream.wait_until_next_half(isr)
    }

    pub fn wait_until_next_half_clear(
        &self,
        isr: &mut StreamIsr<CXX::DMA>,
    ) -> nb::Result<(), StreamError> {
        self.state.stream.wait_until_next_half_clear(isr)
    }

    pub fn current_peripheral_index_(&self) -> Option<usize> {
        match &self.state.buffers.peripheral_buffer {
            Buffer::Fixed(_) => None,
            Buffer::Incremented(buffer) => {
                let ndt = u16::from(self.state.stream.ndt()) as usize;

                if ndt == 0 {
                    None
                } else {
                    Some(buffer.len() - ndt)
                }
            }
        }
    }

    pub fn current_memory_index(&self) -> Option<usize> {
        match &self.state.buffers.memory_buffer.m0a().get() {
            Buffer::Fixed(_) => None,
            Buffer::Incremented(buffer) => {
                let ndt = u16::from(self.state.stream.ndt()) as usize;
                let p_size: usize =
                    PayloadSize::from_payload::<Peripheral>().into();
                let m_size: usize =
                    PayloadSize::from_payload::<Memory>().into();

                let ndt_bytes = ndt * p_size;

                let remaining_memory_items;

                if ndt_bytes % m_size == 0 {
                    remaining_memory_items = ndt_bytes / m_size;
                } else {
                    remaining_memory_items = ndt_bytes / m_size + 1;
                }

                if ndt == 0 {
                    None
                } else {
                    Some(buffer.len() - remaining_memory_items)
                }
            }
        }
    }

    pub fn stop(
        self,
    ) -> (
        Transfer<'wo, Start<'wo, Peripheral, Memory>>,
        Stream<CXX, Disabled, IsrUncleared>,
    ) {
        let stream = self.state.stream.disable();

        let conf =
            Config::from_stream_config(stream.config(), self.state.buffers);
        let transfer = Transfer {
            state: Start { conf },
            _phantom: PhantomData,
        };

        (transfer, stream)
    }
}

impl<'wo, State> Transfer<'wo, State>
where
    State: TransferState<'wo>,
{
    pub fn buffers(&self) -> &Buffers<'wo, State::Peripheral, State::Memory> {
        self.state.buffers()
    }

    pub fn buffers_mut<F>(&mut self, op: F)
    where
        for<'a> F:
            FnOnce(&'a mut Buffers<'wo, State::Peripheral, State::Memory>),
    {
        self.state.buffers_mut(op);
    }

    pub unsafe fn buffers_mut_unchecked(
        &mut self,
    ) -> &mut Buffers<'wo, State::Peripheral, State::Memory> {
        self.state.buffers_mut_unchecked()
    }
}

pub trait TransferState<'wo>: Send + Sync + private::Sealed {
    type Peripheral: Payload;
    type Memory: Payload;

    fn buffers(&self) -> &Buffers<'wo, Self::Peripheral, Self::Memory>;

    fn buffers_mut<F>(&mut self, op: F)
    where
        for<'a> F: FnOnce(&'a mut Buffers<'wo, Self::Peripheral, Self::Memory>);

    unsafe fn buffers_mut_unchecked(
        &mut self,
    ) -> &mut Buffers<'wo, Self::Peripheral, Self::Memory>;
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

    fn buffers(&self) -> &Buffers<'wo, Peripheral, Memory> {
        self.conf.transfer_direction().buffers()
    }

    fn buffers_mut<F>(&mut self, op: F)
    where
        for<'a> F: FnOnce(&'a mut Buffers<'wo, Peripheral, Memory>),
    {
        self.conf.transfer_direction_mut(|t| t.buffers_mut(op));
    }

    unsafe fn buffers_mut_unchecked(
        &mut self,
    ) -> &mut Buffers<'wo, Peripheral, Memory> {
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
    buffers: Buffers<'wo, Peripheral, Memory>,
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

    fn buffers(&self) -> &Buffers<'wo, Peripheral, Memory> {
        &self.buffers
    }

    fn buffers_mut<F>(&mut self, op: F)
    where
        for<'a> F: FnOnce(&'a mut Buffers<'wo, Peripheral, Memory>),
    {
        op(&mut self.buffers)
    }

    unsafe fn buffers_mut_unchecked(
        &mut self,
    ) -> &mut Buffers<'wo, Peripheral, Memory> {
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
