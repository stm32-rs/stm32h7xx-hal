//! This implementation is derived from 0BSD-relicensed work done by
//! Johannes Draaijer <jcdra1@gmail.com> for the
//! [`stm32-eth`](https://github.com/stm32-rs/stm32-eth) project

use core::sync::atomic::{self, Ordering};

use crate::ethernet::{raw_descriptor::RawDescriptor, Cache, PacketId};

#[cfg(feature = "ptp")]
use crate::ptp::Timestamp;

mod consts {
    #![allow(unused)]

    /// Owned by DMA
    pub const RXDESC_3_OWN: u32 = 1 << 31;

    // Read format bits
    /// Interrupt On Completion
    pub const RXDESC_3_IOC: u32 = 1 << 30;
    /// Buffer 2 Address Valid
    pub const RXDESC_3_BUF2V: u32 = 1 << 25;
    /// Buffer 1 Address valid
    pub const RXDESC_3_BUF1V: u32 = 1 << 24;

    // Write-back bits
    /// Timestamp Dropped
    pub const RXDESC_1_TD: u32 = 1 << 16;
    /// Timestamp Avaialble
    pub const RXDESC_1_TSA: u32 = 1 << 14;
    /// Context Descriptor
    pub const RXDESC_3_CTXT: u32 = 1 << 30;
    /// First Descriptor
    pub const RXDESC_3_FD: u32 = 1 << 29;
    /// Last Descriptor
    pub const RXDESC_3_LD: u32 = 1 << 28;
    /// Receive Status RDES2 valid
    pub const RXDESC_3_RS2V: u32 = 1 << 27;
    /// Receive status RDES1 valid
    pub const RXDESC_3_RS1V: u32 = 1 << 26;
    /// Receive status RDES0 valid
    pub const RXDESC_3_RS0V: u32 = 1 << 26;
    /// CRC error
    pub const RXDESC_3_CE: u32 = 1 << 24;
    /// Giant Packet
    pub const RXDESC_3_GP: u32 = 1 << 23;
    /// Receive Watchdog Timeout
    pub const RXDESC_3_RWT: u32 = 1 << 22;
    /// Overflow Error
    pub const RXDESC_3_OE: u32 = 1 << 21;
    /// Receive Error
    pub const RXDESC_3_RE: u32 = 1 << 20;
    /// Dribble Bit Error
    pub const RXDESC_3_DE: u32 = 1 << 19;

    /// Length/Type Field shift
    pub const RXDESC_3_LT_SHIFT: u32 = 16;
    /// Length/Type Field mask
    pub const RXDESC_3_LT_MASK: u32 = 0b111 << RXDESC_3_LT_SHIFT;
    /// Length/Type Field
    #[allow(non_camel_case_types)]
    #[repr(u32)]
    pub enum RXDESC_3_LT {
        Length = 0b000 << RXDESC_3_LT_SHIFT,
        Type = 0b001 << RXDESC_3_LT_SHIFT,
        Reserved = 0b010 << RXDESC_3_LT_SHIFT,
        ArpRequest = 0b011 << RXDESC_3_LT_SHIFT,
        TypeWithVlan = 0b100 << RXDESC_3_LT_SHIFT,
        TypeWIthDoubleVlan = 0b101 << RXDESC_3_LT_SHIFT,
        MacControl = 0b110 << RXDESC_3_LT_SHIFT,
        Oam = 0b111 << RXDESC_3_LT_SHIFT,
    }

    /// Error Summary
    pub const RXDESC_3_ES: u32 = 1 << 15;

    /// Packet Length shift
    pub const RXDESC_3_PL_SHIFT: u32 = 0;
    /// Packet Length mask
    pub const RXDESC_3_PL_MASK: u32 = 0x3FFF;
}
pub use consts::*;

use super::RxDescriptorError;

#[repr(C)]
#[repr(align(4))]
#[derive(Clone, Copy)]
/// An RX DMA Descriptor.
pub struct RxDescriptor {
    inner_raw: RawDescriptor,
    cache: Cache,
}

impl Default for RxDescriptor {
    fn default() -> Self {
        Self::new()
    }
}

impl RxDescriptor {
    /// Creates a new [`RxDescriptor`].
    pub const fn new() -> Self {
        Self {
            inner_raw: RawDescriptor::new(),
            cache: Cache::new(),
        }
    }

    /// Is owned by the DMA engine?
    pub(super) fn is_owned(&self) -> bool {
        (self.inner_raw.read(3) & RXDESC_3_OWN) == RXDESC_3_OWN
    }

    pub(super) fn is_available(&self) -> bool {
        !self.is_owned()
    }

    fn has_error(&self) -> bool {
        self.inner_raw.read(3) & RXDESC_3_ES == RXDESC_3_ES
    }

    fn is_first(&self) -> bool {
        self.inner_raw.read(3) & RXDESC_3_FD == RXDESC_3_FD
    }

    fn is_last(&self) -> bool {
        self.inner_raw.read(3) & RXDESC_3_LD == RXDESC_3_LD
    }

    pub(super) fn is_context(&self) -> bool {
        self.inner_raw.read(3) & RXDESC_3_CTXT == RXDESC_3_CTXT
            && self.is_available()
    }

    pub(super) fn frame_len(&self) -> usize {
        if self.is_owned() {
            0
        } else {
            ((self.inner_raw.read(3) & RXDESC_3_PL_MASK) >> RXDESC_3_PL_SHIFT)
                as usize
        }
    }

    // We ignore the end_of_ring bool, but need it for compatibility with the F-series
    // descriptor.
    pub(super) fn setup(&mut self, _end_of_ring: bool, buffer: &[u8]) {
        self.set_owned(buffer);
    }

    /// Pass ownership to the DMA engine
    pub(super) fn set_owned(&mut self, buffer: &[u8]) {
        let buffer = buffer.as_ptr();
        self.set_buffer(buffer);

        // "Preceding reads and writes cannot be moved past subsequent writes."
        #[cfg(feature = "fence")]
        atomic::fence(Ordering::Release);
        atomic::compiler_fence(Ordering::Release);

        unsafe {
            self.inner_raw
                .modify(3, |w| w | RXDESC_3_OWN | RXDESC_3_IOC);
        }

        // Used to flush the store buffer as fast as possible to make the buffer available for the
        // DMA.
        #[cfg(feature = "fence")]
        atomic::fence(Ordering::SeqCst);
    }

    /// Configure the buffer and its length.
    fn set_buffer(&mut self, buffer_ptr: *const u8) {
        unsafe {
            // Set buffer 1 address.
            self.inner_raw.modify(0, |_| buffer_ptr as u32);

            // RXDESC does not contain buffer length, it is set
            // in register INSERT_HERE instead. The size of all
            // buffers is verified by [`RxRing`](super::RxRing)

            self.inner_raw.write(3, RXDESC_3_BUF1V);
        }
    }

    pub(super) fn recv(
        &mut self,
        packet_id: Option<PacketId>,
        buffer: &mut [u8],
    ) -> Result<(), RxDescriptorError> {
        // Only single-frame descriptors and non-context descriptors are supported
        // for now.
        if self.has_error() {
            Err(RxDescriptorError::DmaError)
        } else if self.is_first()
            && self.is_last()
            && !self.has_error()
            && !self.is_context()
        {
            // "Subsequent reads and writes cannot be moved ahead of preceding reads."
            atomic::compiler_fence(Ordering::Acquire);

            self.cache.set_id_and_clear_ts(packet_id);

            Ok(())
        } else {
            self.set_owned(buffer);
            Err(RxDescriptorError::Truncated)
        }
    }
}

#[cfg(feature = "ptp")]
impl RxDescriptor {
    pub(super) fn has_timestamp(&self) -> bool {
        (self.inner_raw.read(1) & RXDESC_1_TSA) == RXDESC_1_TSA
            && self.is_last()
    }

    /// Get PTP timestamps if available
    pub(super) fn read_timestamp(&self) -> Option<Timestamp> {
        if self.is_context() && !self.is_owned() {
            let (high, low) = (self.inner_raw.read(1), self.inner_raw.read(0));
            Some(Timestamp::from_parts(high, low))
        } else {
            None
        }
    }

    pub(super) fn has_packet_id(&self, packet_id: &PacketId) -> bool {
        self.cache.id().as_ref() == Some(packet_id)
    }

    pub(super) fn attach_timestamp(&mut self, timestamp: Option<Timestamp>) {
        self.cache.set_ts(timestamp);
    }

    pub(super) fn timestamp(&self) -> Option<Timestamp> {
        self.cache.ts()
    }
}
