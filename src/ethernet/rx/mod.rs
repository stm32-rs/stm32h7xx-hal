pub use descriptor::RxDescriptor;

use super::{raw_descriptor::DescriptorRing, PacketId};
use crate::stm32::ETHERNET_DMA;

#[path = "./h_descriptor.rs"]
mod descriptor;

#[cfg(feature = "ptp")]
use crate::{ethernet::eth::PacketIdNotFound, ptp::Timestamp};

#[cfg(feature = "async-await")]
use core::task::Poll;

/// Errors that can occur during RX
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, PartialEq)]
pub(crate) enum RxDescriptorError {
    /// The received packet was truncated
    Truncated,
    /// An error occured with the DMA
    DmaError,
}

/// Errors that can occur during RX
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, PartialEq)]
pub enum RxError {
    /// The received packet was truncated
    Truncated,
    /// An error occured with the DMA
    DmaError,
    /// Receiving would block
    WouldBlock,
}

impl From<RxDescriptorError> for RxError {
    fn from(value: RxDescriptorError) -> Self {
        match value {
            RxDescriptorError::Truncated => Self::Truncated,
            RxDescriptorError::DmaError => Self::DmaError,
        }
    }
}

/// An RX descriptor ring.
pub type RxDescriptorRing<'rx> = DescriptorRing<'rx, RxDescriptor>;

/// Rx DMA state
pub struct RxRing<'a> {
    ring: RxDescriptorRing<'a>,
    next_entry: usize,
}

impl<'a> RxRing<'a> {
    /// Allocate
    pub(crate) fn new(ring: RxDescriptorRing<'a>) -> Self {
        RxRing {
            ring,
            next_entry: 0,
        }
    }

    /// Setup the DMA engine (**required**)
    pub(crate) fn start(&mut self, eth_dma: &ETHERNET_DMA) {
        // Setup ring
        let ring_len = self.ring.len();
        for (idx, (entry, buffer)) in
            self.ring.descriptors_and_buffers().enumerate()
        {
            entry.setup(idx == ring_len - 1, buffer);
        }

        self.next_entry = 0;
        let ring_ptr = self.ring.descriptors_start_address();

        {
            let rx_ring_descriptors = self.ring.descriptors().count();
            assert!(rx_ring_descriptors >= 4);

            // Assert that the descriptors are properly aligned.
            //
            // FIXME: these require different alignment if the data is stored
            // in AXI SRAM
            assert!(ring_ptr as u32 % 4 == 0);
            assert!(
                self.ring.last_descriptor_mut() as *const _ as u32 % 4 == 0
            );

            // Set the start pointer.
            eth_dma
                .dmacrx_dlar
                .write(|w| unsafe { w.bits(ring_ptr as u32) });

            // Set the Receive Descriptor Ring Length
            eth_dma.dmacrx_rlr.write(|w| {
                w.rdrl()
                    .variant((self.ring.descriptors().count() - 1) as u16)
            });

            // Set the tail pointer
            eth_dma.dmacrx_dtpr.write(|w| unsafe {
                w.bits(self.ring.last_descriptor() as *const _ as u32)
            });

            // Set receive buffer size
            let receive_buffer_size = self.ring.last_buffer().len() as u16;
            assert!(receive_buffer_size % 4 == 0);

            eth_dma.dmacrx_cr.modify(|_, w| unsafe {
                w
                    // Start receive
                    .sr()
                    .set_bit()
                    // Set receive buffer size
                    .rbsz()
                    .bits(receive_buffer_size >> 1)
                    // AUtomatically flush on bus error
                    .rpf()
                    .set_bit()
            });
        }

        Self::demand_poll();
    }

    /// Stop the RX DMA
    pub(crate) fn stop(&self, eth_dma: &ETHERNET_DMA) {
        let start_reg = &eth_dma.dmacrx_cr;

        start_reg.modify(|_, w| w.sr().clear_bit());
        // DMA accesses do not stop before the running state
        // of the DMA has changed to something other than
        // running.
        while Self::running_state().is_running() {}
    }

    /// Demand that the DMA engine polls the current `RxDescriptor`
    /// (when in [`RunningState::Stopped`].)
    fn demand_poll() {
        // # SAFETY
        //
        // On F7, we only perform an atomic write to `damrpdr`.
        //
        // On H7, we only perform a Read-Write to `dmacrx_dtpr`,
        // always with the same value. Running `demand_poll` concurrently
        // with the other location in which this register is written ([`RxRing::start`])
        // is impossible, which is guaranteed the state transition from NotRunning to
        // Running.
        let eth_dma = unsafe { &*ETHERNET_DMA::ptr() };

        // On H7, we poll by re-writing the tail pointer register.
        eth_dma
            .dmacrx_dtpr
            .modify(|r, w| unsafe { w.bits(r.bits()) });
    }

    /// Get current state of the RxDMA
    pub fn running_state() -> RunningState {
        // SAFETY: we only perform an atomic read of `dmasr`.
        let eth_dma = unsafe { &*ETHERNET_DMA::ptr() };

        if eth_dma.dmacsr.read().fbe().bit_is_set() {
            super::EthernetDMA::panic_fbe();
        }

        let rps = eth_dma.dmadsr.read().rps0().bits();

        match rps {
            //  Reset or Stop Receive Command issued
            0b000 => RunningState::Stopped,
            //  Fetching receive transfer descriptor
            0b001 => RunningState::Running,
            //  Waiting for receive packet
            0b011 => RunningState::Running,
            //  Receive descriptor unavailable
            0b100 => RunningState::Stopped,
            //  Closing receive descriptor
            0b101 => RunningState::Running,
            //  Transferring the receive packet data from receive buffer to host memory
            0b111 => RunningState::Running,
            // Timestamp write state
            0b110 => RunningState::Running,
            _ => RunningState::Unknown,
        }
    }

    /// Check if we can receive a new packet
    pub fn next_entry_available(&self) -> bool {
        self.ring.descriptor(self.next_entry).is_available()
    }

    /// Obtain the index of the packet to receive (if any is ready).
    ///
    /// This function returns a tuple of `Ok(entry_index)` on
    /// success. Whoever receives the `Ok` must ensure that `set_owned`
    /// is eventually called on the entry with that index.
    ///
    /// Actually obtaining the relevant RxPacket is done using
    /// [`RxRing::recv_and_timestamp`].
    fn recv_next_impl(
        &mut self,
        // NOTE(allow): packet_id is unused if ptp is disabled.
        #[allow(unused_variables)] packet_id: Option<PacketId>,
    ) -> Result<usize, RxError> {
        if !Self::running_state().is_running() {
            Self::demand_poll();
        }

        if self.next_entry_available() {
            let entries_len = self.ring.len();
            let (desc, buffer) = self.ring.get_mut(self.next_entry);

            desc.recv(packet_id, buffer)?;

            let entry_num = self.next_entry;
            self.next_entry = (self.next_entry + 1) % entries_len;

            Ok(entry_num)
        } else {
            Err(RxError::WouldBlock)
        }
    }

    fn recv_and_timestamp(&mut self, entry: usize) -> RxPacket {
        let entries_len = self.ring.len();

        let (desc, buffer) = {
            let (desc, buffer, next_desc, next_buffer) =
                self.ring.get_mut_and_next(entry);

            // Read the timestamp from the next context descriptor, if it's available.
            if next_desc.is_context() {
                #[cfg(feature = "ptp")]
                if desc.has_timestamp() {
                    let timestamp = next_desc.read_timestamp();
                    desc.attach_timestamp(timestamp);
                }
                next_desc.set_owned(next_buffer);
                self.next_entry = (self.next_entry + 1) % entries_len;
            }

            (desc, buffer)
        };

        let length = desc.frame_len();

        RxPacket {
            entry: desc,
            buffer,
            length,
        }
    }

    /// Receive the next packet (if any is ready), or return [`Err`]
    /// immediately.
    pub fn recv_next(
        &mut self,
        packet_id: Option<PacketId>,
    ) -> Result<RxPacket, RxError> {
        let entry = self.recv_next_impl(packet_id)?;

        Ok(self.recv_and_timestamp(entry))
    }

    /// Receive the next packet.
    ///
    /// The returned [`RxPacket`] can be used as a slice, and
    /// will contain the ethernet data.
    #[cfg(feature = "async-await")]
    pub async fn recv(&mut self, packet_id: Option<PacketId>) -> RxPacket {
        let entry = core::future::poll_fn(|ctx| {
            let res = self.recv_next_impl(packet_id.clone());

            match res {
                Ok(value) => Poll::Ready(value),
                Err(_) => {
                    crate::dma::EthernetDMA::rx_waker().register(ctx.waker());
                    Poll::Pending
                }
            }
        })
        .await;

        self.recv_and_timestamp(entry)
    }
}

#[cfg(feature = "ptp")]
impl<'a> RxRing<'a> {
    /// Get the timestamp for a specific ID
    pub fn timestamp(
        &self,
        id: &PacketId,
    ) -> Result<Option<Timestamp>, PacketIdNotFound> {
        let entry = self.ring.descriptors().find(|e| e.has_packet_id(id));

        let entry = entry.ok_or(PacketIdNotFound)?;

        Ok(entry.timestamp())
    }
}

/// Running state of the `RxRing`
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(PartialEq, Eq, Debug)]
pub enum RunningState {
    /// Running state is unknown.
    Unknown,
    /// The RX DMA is stopped.
    Stopped,
    /// The RX DMA is running.
    Running,
}

impl RunningState {
    /// whether self equals to `RunningState::Running`
    pub fn is_running(&self) -> bool {
        *self == RunningState::Running
    }
}

/// A received packet.
///
/// This packet implements [Deref<\[u8\]>](core::ops::Deref) and should be used
/// as a slice.
pub struct RxPacket<'a, 'buf> {
    entry: &'a mut RxDescriptor,
    buffer: &'buf mut [u8],
    length: usize,
}

impl core::ops::Deref for RxPacket<'_, '_> {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        &self.buffer[..self.length]
    }
}

impl<'a> core::ops::DerefMut for RxPacket<'_, '_> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.buffer[..self.length]
    }
}

impl Drop for RxPacket<'_, '_> {
    fn drop(&mut self) {
        self.entry.set_owned(self.buffer);
    }
}

impl RxPacket<'_, '_> {
    /// Pass the received packet back to the DMA engine.
    pub fn free(self) {
        drop(self)
    }

    /// Get the timestamp associated with this packet
    #[cfg(feature = "ptp")]
    pub fn timestamp(&self) -> Option<Timestamp> {
        self.entry.timestamp()
    }
}
