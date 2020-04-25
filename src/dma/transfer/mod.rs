//! Safe DMA Transfers

pub mod buffer;
pub mod config;

use self::buffer::{MemoryBuffer, MemoryBufferType};
use super::stream::config::{
    DirectModeErrorInterrupt, FifoErrorInterrupt, HalfTransferInterrupt, M0a,
    M1a, MSize, PSize, TransferCompleteInterrupt, TransferErrorInterrupt,
};
use super::stream::{
    Disabled, Enabled, Error as StreamError, Event, IsrCleared, IsrUncleared,
    StreamIsr,
};
use super::{ChannelId, Stream};
use crate::{nb, private};
use core::fmt::Debug;
use core::hint;
use core::marker::PhantomData;
use core::mem;

use nb::block;

pub use self::buffer::Buffer;
pub use self::config::Config;

pub struct Transfer<'wo, Peripheral, Memory, State>
where
    Peripheral: Payload,
    Memory: Payload,
    State: TransferState,
{
    conf: Config<'wo, Peripheral, Memory>,
    state: State,
    _phantom: PhantomData<&'wo ()>,
}

impl<'wo, Peripheral, Memory> Transfer<'wo, Peripheral, Memory, Start>
where
    Peripheral: Payload,
    Memory: Payload,
{
    pub fn new(conf: Config<'wo, Peripheral, Memory>) -> Self {
        Self {
            conf,
            state: Start { _private: () },
            _phantom: PhantomData,
        }
    }

    pub fn start<CXX: ChannelId>(
        self,
        mut stream: Stream<CXX, Disabled, IsrCleared>,
    ) -> Transfer<'wo, Peripheral, Memory, Ongoing<CXX>> {
        self.configure_stream(&mut stream);

        Transfer {
            conf: self.conf,
            state: Ongoing {
                stream: unsafe { stream.enable() },
            },
            _phantom: PhantomData,
        }
    }

    fn configure_stream<CXX: ChannelId>(
        &self,
        stream: &mut Stream<CXX, Disabled, IsrCleared>,
    ) {
        let mut conf = stream.config().config();

        self.conf.stream_config(&mut conf);

        stream.apply_config(conf.check());
    }

    pub fn config_mut(&mut self) -> &mut Config<'wo, Peripheral, Memory> {
        &mut self.conf
    }

    pub fn buffers_mut<F>(&mut self, op: F)
    where
        for<'a> F: FnOnce(&'a mut Buffers<'wo, Peripheral, Memory>),
    {
        self.conf.buffers_mut(op);
    }

    pub fn free(self) -> Config<'wo, Peripheral, Memory> {
        self.conf
    }
}

impl<'wo, Peripheral, Memory, CXX>
    Transfer<'wo, Peripheral, Memory, Ongoing<CXX>>
where
    Peripheral: Payload,
    Memory: Payload,
    CXX: ChannelId,
{
    pub fn stream(&self) -> &Stream<CXX, Enabled, IsrUncleared> {
        &self.state.stream
    }

    pub fn set_transfer_complete_interrupt(
        &mut self,
        tc: TransferCompleteInterrupt,
    ) {
        self.state.stream.set_transfer_complete_interrupt(tc);
    }

    pub fn set_half_transfer_interrupt(&mut self, ht: HalfTransferInterrupt) {
        self.state.stream.set_half_transfer_interrupt(ht);
    }

    pub fn set_transfer_error_interrupt(&mut self, te: TransferErrorInterrupt) {
        self.state.stream.set_transfer_error_interrupt(te);
    }

    pub fn set_direct_mode_error_interrupt(
        &mut self,
        dme: DirectModeErrorInterrupt,
    ) {
        self.state.stream.set_direct_mode_error_interrupt(dme);
    }

    pub fn set_fifo_error_interrupt(&mut self, fe: FifoErrorInterrupt) {
        self.state.stream.set_fifo_error_interrupt(fe);
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

    pub fn current_peripheral_index(&self) -> Option<usize> {
        match &self.conf.buffers().peripheral_buffer {
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
        match self.conf.buffers().memory_buffer.m0a().get() {
            Buffer::Fixed(_) => None,
            Buffer::Incremented(buffer) => {
                let ndt = u16::from(self.state.stream.ndt()) as usize;

                if ndt == 0 {
                    return None;
                }

                let p_size: usize =
                    PayloadSize::from_payload::<Peripheral>().into();
                let m_size: usize =
                    PayloadSize::from_payload::<Memory>().into();

                let ndt_bytes = ndt * p_size;

                let remaining_memory_items = if ndt_bytes % m_size == 0 {
                    ndt_bytes / m_size
                } else {
                    ndt_bytes / m_size + 1
                };

                Some(buffer.len() - remaining_memory_items)
            }
        }
    }

    pub fn replace_m0a(
        &mut self,
        memory_buffer: MemoryBuffer<Memory>,
    ) -> MemoryBuffer<Memory> {
        unsafe {
            block!(self
                .state
                .stream
                .set_m0a(M0a(memory_buffer.get().as_ptr(Some(0)) as u32)))
        }
        .unwrap();

        let mut x = None;
        self.conf.buffers_mut(|b| match &mut b.memory_buffer {
            MemoryBufferType::SingleBuffer(_) => panic!(
                "Cannot replace memory buffer on the fly for single buffer."
            ),
            MemoryBufferType::DoubleBuffer(buffer) => {
                buffer.memories_mut(|b| {
                    x = Some(mem::replace(&mut b[0], memory_buffer))
                })
            }
        });

        x.unwrap_or_else(|| unsafe { hint::unreachable_unchecked() })
    }

    pub fn replace_m1a(
        &mut self,
        memory_buffer: MemoryBuffer<Memory>,
    ) -> MemoryBuffer<Memory> {
        unsafe {
            block!(self
                .state
                .stream
                .set_m1a(M1a(memory_buffer.get().as_ptr(Some(0)) as u32)))
        }
        .unwrap();

        let mut x = None;
        self.conf.buffers_mut(|b| match &mut b.memory_buffer {
            MemoryBufferType::SingleBuffer(_) => panic!(
                "Cannot replace memory buffer on the fly for single buffer."
            ),
            MemoryBufferType::DoubleBuffer(buffer) => {
                buffer.memories_mut(|b| {
                    x = Some(mem::replace(&mut b[1], memory_buffer))
                })
            }
        });

        x.unwrap_or_else(|| unsafe { hint::unreachable_unchecked() })
    }

    pub fn halt(self) -> Transfer<'wo, Peripheral, Memory, Halted<CXX>> {
        let (stream, _) = self.state.stream.halt();

        Transfer {
            conf: self.conf,
            state: Halted { stream },
            _phantom: PhantomData,
        }
    }

    pub fn stop(
        self,
    ) -> (
        Transfer<'wo, Peripheral, Memory, Start>,
        Stream<CXX, Disabled, IsrUncleared>,
    ) {
        let stream = self.state.stream.disable();

        let transfer = Transfer {
            conf: self.conf,
            state: Start { _private: () },
            _phantom: PhantomData,
        };

        (transfer, stream)
    }
}

impl<'wo, Peripheral, Memory, State> Transfer<'wo, Peripheral, Memory, State>
where
    Peripheral: Payload,
    Memory: Payload,
    State: TransferState,
{
    pub fn config(&self) -> &Config<'wo, Peripheral, Memory> {
        &self.conf
    }

    pub unsafe fn config_mut_unchecked(
        &mut self,
    ) -> &mut Config<'wo, Peripheral, Memory> {
        &mut self.conf
    }

    pub fn buffers(&self) -> &Buffers<'wo, Peripheral, Memory> {
        self.conf.buffers()
    }

    pub unsafe fn buffers_mut_unchecked(
        &mut self,
    ) -> &mut Buffers<'wo, Peripheral, Memory> {
        self.conf.buffers_mut_unchecked()
    }
}

impl<'wo, Peripheral, Memory, CXX>
    Transfer<'wo, Peripheral, Memory, Halted<CXX>>
where
    Peripheral: Payload,
    Memory: Payload,
    CXX: ChannelId,
{
    pub fn resume(
        self,
        isr: &mut StreamIsr<CXX::DMA>,
    ) -> Transfer<'wo, Peripheral, Memory, Ongoing<CXX>> {
        let stream = unsafe { self.state.stream.clear_isr(isr).enable() };

        Transfer {
            conf: self.conf,
            state: Ongoing { stream },
            _phantom: PhantomData,
        }
    }

    pub fn stop(
        self,
    ) -> (
        Transfer<'wo, Peripheral, Memory, Start>,
        Stream<CXX, Disabled, IsrUncleared>,
    ) {
        let transfer = Transfer {
            conf: self.conf,
            state: Start { _private: () },
            _phantom: PhantomData,
        };

        (transfer, self.state.stream)
    }
}

pub trait TransferState: Send + Sync + private::Sealed {}

pub struct Start {
    _private: (),
}

impl private::Sealed for Start {}

impl TransferState for Start {}

pub struct Ongoing<CXX: ChannelId> {
    stream: Stream<CXX, Enabled, IsrUncleared>,
}

impl<CXX: ChannelId> private::Sealed for Ongoing<CXX> {}

impl<CXX: ChannelId> TransferState for Ongoing<CXX> {}

pub struct Halted<CXX: ChannelId> {
    stream: Stream<CXX, Disabled, IsrUncleared>,
}

impl<CXX: ChannelId> private::Sealed for Halted<CXX> {}

impl<CXX: ChannelId> TransferState for Halted<CXX> {}

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

pub struct Byte {
    _private: (),
}
pub struct HalfWord {
    _private: (),
}
pub struct Word {
    _private: (),
}

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
