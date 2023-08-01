use super::{raw_descriptor::DescriptorRing, PacketId};
use crate::stm32::ETHERNET_DMA;

#[cfg(feature = "ptp")]
use super::eth::PacketIdNotFound;
use crate::ptp::Timestamp;

#[path = "./h_descriptor.rs"]
mod descriptor;

pub use descriptor::TxDescriptor;

#[cfg(any(feature = "ptp", feature = "async-await"))]
use core::task::Poll;

/// A TX descriptor ring.
pub type TxDescriptorRing<'rx> = DescriptorRing<'rx, TxDescriptor>;

/// Errors that can occur during Ethernet TX
#[derive(Debug, PartialEq)]
pub enum TxError {
    /// Ring buffer is full
    WouldBlock,
}

/// Tx DMA state
pub struct TxRing<'a> {
    ring: TxDescriptorRing<'a>,
    next_entry: usize,
}

impl<'ring> TxRing<'ring> {
    /// Allocate
    ///
    /// `start()` will be needed before `send()`
    pub(crate) fn new(ring: TxDescriptorRing<'ring>) -> Self {
        TxRing {
            ring,
            next_entry: 0,
        }
    }

    /// Start the Tx DMA engine
    pub(crate) fn start(&mut self, eth_dma: &ETHERNET_DMA) {
        for descriptor in self.ring.descriptors_mut() {
            descriptor.setup();
        }

        let ring_ptr = self.ring.descriptors_start_address();

        {
            let tx_descriptor_count = self.ring.descriptors().count();
            assert!(tx_descriptor_count >= 4);

            // Assert that the descriptors are properly aligned.
            //
            // FIXME: these require different alignment if the data is stored
            // in AXI SRAM
            assert!(ring_ptr as u32 % 4 == 0);
            assert!(self.ring.last_descriptor() as *const _ as u32 % 4 == 0);

            // Set the start pointer.
            eth_dma
                .dmactx_dlar
                .write(|w| unsafe { w.bits(ring_ptr as u32) });

            // Set the Transmit Descriptor Ring Length
            eth_dma.dmactx_rlr.write(|w| {
                w.tdrl()
                    .variant((self.ring.descriptors().count() - 1) as u16)
            });

            // Set the tail pointer
            eth_dma.dmactx_dtpr.write(|w| unsafe {
                w.bits(self.ring.last_descriptor_mut() as *const _ as u32)
            });
        }

        // "Preceding reads and writes cannot be moved past subsequent writes."
        #[cfg(feature = "fence")]
        core::sync::atomic::fence(core::sync::atomic::Ordering::Release);

        // We don't need a compiler fence here because all interactions with `Descriptor` are
        // volatiles

        let start_reg = &eth_dma.dmactx_cr;

        // Start transmission
        start_reg.modify(|_, w| w.st().set_bit());
    }

    /// Stop the TX DMA
    pub(crate) fn stop(&self, eth_dma: &ETHERNET_DMA) {
        let start_reg = &eth_dma.dmactx_cr;

        start_reg.modify(|_, w| w.st().clear_bit());

        // DMA accesses do not stop before the running state
        // of the DMA has changed to something other than
        // running.
        while Self::is_running() {}
    }

    fn entry_available(&self, index: usize) -> bool {
        self.ring.descriptor(index).is_available()
    }

    /// If this returns `true`, the next `send` will succeed.
    pub fn next_entry_available(&self) -> bool {
        self.entry_available(self.next_entry % self.ring.len())
    }

    /// Check if we can send the next TX entry.
    ///
    /// If [`Ok(res)`] is returned, the caller of must ensure
    /// that [`self.entries[res].send()`](TxRingEntry::send) is called
    /// before a new invocation of `send_next_impl`.
    fn send_next_impl(&mut self) -> Result<usize, TxError> {
        if self.next_entry_available() {
            let entry_num = self.next_entry;

            self.next_entry = (self.next_entry + 1) % self.ring.len();
            Ok(entry_num)
        } else {
            Err(TxError::WouldBlock)
        }
    }

    /// Prepare a packet for sending.
    ///
    /// Write the data that you wish to send to the buffer
    /// represented by the returned [`TxPacket`] by using it
    /// as a slice.
    ///
    /// When all data is copied into the TX buffer, use [`TxPacket::send()`]
    /// to transmit it.
    pub fn send_next(
        &mut self,
        length: usize,
        packet_id: Option<PacketId>,
    ) -> Result<TxPacket, TxError> {
        let entry = self.send_next_impl()?;

        let (desc, tx_buffer) = self.ring.get_mut(entry);

        assert!(length <= tx_buffer.len(), "Not enough space in TX buffer");

        Ok(TxPacket {
            desc,
            buffer: tx_buffer,
            length,
            packet_id,
        })
    }

    /// Prepare a packet for sending.
    ///
    /// Write the data that you wish to send to the buffer
    /// represented by the returned [`TxPacket`] by using it
    /// as a slice.
    ///
    /// When all data is copied into the TX buffer, use [`TxPacket::send()`]
    /// to transmit it.
    #[cfg(feature = "async-await")]
    pub async fn prepare_packet<'tx>(
        &'tx mut self,
        length: usize,
        packet_id: Option<PacketId>,
    ) -> TxPacket {
        let entry = core::future::poll_fn(|ctx| match self.send_next_impl() {
            Ok(packet) => Poll::Ready(packet),
            Err(_) => {
                crate::dma::EthernetDMA::tx_waker().register(ctx.waker());
                Poll::Pending
            }
        })
        .await;

        let (desc, tx_buffer) = self.ring.get_mut(entry);

        assert!(length <= tx_buffer.len(), "Not enough space in TX buffer");

        TxPacket {
            desc,
            buffer: tx_buffer,
            length,
            packet_id,
        }
    }

    /// Demand that the DMA engine polls the current `TxDescriptor`
    /// (when we just transferred ownership to the hardware).
    pub(crate) fn demand_poll() {
        // # SAFETY
        //
        // On F-series, we only perform an atomic write to `damrpdr`.
        //
        // On H7, we only perform a Read-Write to `dmacrx_dtpr`,
        // always with the same value. Running `demand_poll` concurrently
        // with the other location in which this register is written ([`TxRing::start`])
        // is impossible, which is guaranteed the state transition from NotRunning to
        // Running.
        let eth_dma = unsafe { &*ETHERNET_DMA::ptr() };

        // To issue a poll demand, write a value to
        // the tail pointer. We just re-write the
        // current value.
        eth_dma
            .dmactx_dtpr
            .modify(|r, w| unsafe { w.bits(r.bits()) });
    }

    /// Is the Tx DMA engine running?
    pub fn is_running() -> bool {
        Self::running_state().is_running()
    }

    /// Get the current state of the TxDMA
    pub fn running_state() -> RunningState {
        // SAFETY: we only perform an atomic read of `dmasr` or
        // `dmadsr`.
        let eth_dma = unsafe { &*ETHERNET_DMA::ptr() };

        if eth_dma.dmacsr.read().fbe().bit_is_set() {
            super::EthernetDMA::panic_fbe();
        }

        let tx_status = eth_dma.dmadsr.read().tps0().bits();

        match tx_status {
            // Reset or Stop Transmit Command issued
            0b000 => RunningState::Stopped,
            // Fetching transmit transfer descriptor
            0b001 => RunningState::Running,
            // Waiting for status
            0b010 => RunningState::Running,
            // Reading Data from host memory buffer and queuing it to transmit buffer
            0b011 => RunningState::Running,

            0b101 => RunningState::Reserved,

            // Transmit descriptor unavailable
            0b110 => RunningState::Suspended,
            // Timestamp write
            0b100 => RunningState::Running,
            // Closing Tx descriptor
            0b111 => RunningState::Running,

            _ => RunningState::Unknown,
        }
    }
}

#[cfg(feature = "ptp")]
impl TxRing<'_> {
    fn entry_for_id(&self, id: &PacketId) -> Option<usize> {
        self.ring.descriptors().enumerate().find_map(|(idx, e)| {
            if e.has_packet_id(id) {
                defmt::info!("Entry: {} for packet id: {}", id.0, idx);
                Some(idx)
            } else {
                // defmt::info!("No entry for packet id: {}", id.0);
                None
            }
        })
    }

    fn entry_timestamp(&self, index: usize) -> Option<Timestamp> {
        self.ring.descriptor(index).timestamp()
    }

    /// Blockingly wait untill the timestamp for the
    /// given ID is available.
    pub fn wait_for_timestamp(
        &self,
        packet_id: &PacketId,
    ) -> Result<Option<Timestamp>, PacketIdNotFound> {
        loop {
            if let Poll::Ready(res) = self.poll_timestamp(packet_id) {
                return res;
            }
        }
    }

    /// Poll to check if the timestamp for the given ID is already
    /// available.
    pub fn poll_timestamp(
        &self,
        packet_id: &PacketId,
    ) -> Poll<Result<Option<Timestamp>, PacketIdNotFound>> {
        let entry = if let Some(entry) = self.entry_for_id(packet_id) {
            entry
        } else {
            return Poll::Ready(Err(PacketIdNotFound));
        };

        if self.entry_available(entry) {
            Poll::Ready(Ok(self.entry_timestamp(entry)))
        } else {
            Poll::Pending
        }
    }

    /// Wait until the timestamp for the given ID is available.
    #[cfg(feature = "async-await")]
    pub async fn timestamp(
        &mut self,
        packet_id: &PacketId,
    ) -> Result<Option<Timestamp>, PacketIdNotFound> {
        core::future::poll_fn(move |ctx| {
            let res = self.poll_timestamp(packet_id);
            if res.is_pending() {
                crate::dma::EthernetDMA::tx_waker().register(ctx.waker());
            }
            res
        })
        .await
    }
}

#[derive(Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// The run state of the TX DMA.
pub enum RunningState {
    /// Reset or Stop Transmit Command issued
    Stopped,
    /// Fetching transmit transfer descriptor;
    /// Waiting for status;
    /// Reading Data from host memory buffer and queuing it to transmit buffer
    Running,
    /// Reserved for future use
    Reserved,
    /// Transmit descriptor unavailable
    Suspended,
    /// Invalid value
    Unknown,
}

impl RunningState {
    /// Check whether this state represents that the
    /// TX DMA is running
    pub fn is_running(&self) -> bool {
        *self == RunningState::Running
    }
}

/// A struct that represents a soon-to-be-sent packet.
///
/// Implements [`Deref`] and [`DerefMut`] with `[u8]` as a target
/// so it can be used as a slice.
///
/// [`Deref`]: core::ops::Deref
/// [`DerefMut`]: core::ops::DerefMut
pub struct TxPacket<'borrow> {
    desc: &'borrow mut TxDescriptor,
    buffer: &'borrow mut [u8],
    length: usize,
    packet_id: Option<PacketId>,
}

impl core::ops::Deref for TxPacket<'_> {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        &self.buffer[..self.length]
    }
}

impl core::ops::DerefMut for TxPacket<'_> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.buffer[..self.length]
    }
}

impl TxPacket<'_> {
    /// Send this packet!
    pub fn send(self) {
        drop(self);
    }
}

impl Drop for TxPacket<'_> {
    fn drop(&mut self) {
        self.desc
            .send(self.packet_id.clone(), &self.buffer[..self.length]);
        TxRing::demand_poll();
    }
}
