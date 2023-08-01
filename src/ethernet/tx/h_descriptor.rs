use core::sync::atomic::{self, Ordering};

use crate::ethernet::{raw_descriptor::RawDescriptor, Cache, PacketId};

#[cfg(feature = "ptp")]
use crate::ptp::Timestamp;

mod consts {

    #![allow(unused)]

    // Both read and write-back formats
    /// OWN bit
    pub const TXDESC_3_OWN: u32 = 1 << 31;
    /// Context Type
    pub const TXDESC_3_CTXT: u32 = 1 << 30;
    /// First descriptor
    pub const TXDESC_3_FD: u32 = 1 << 29;
    /// Last descriptor
    pub const TXDESC_3_LD: u32 = 1 << 28;

    // Read format
    /// Interrupt On Completion
    pub const TXDESC_2_IOC: u32 = 1 << 31;
    /// Transmit Timestamp Enable
    pub const TXDESC_2_TTSE: u32 = 1 << 30;
    /// Buffer 2 length shift
    pub const TXDESC_2_B2L_SHIFT: u32 = 16;
    /// Buffer 2 length mask
    pub const TXDESC_2_B2L_MASK: u32 = 0x3FFF << TXDESC_2_B2L_SHIFT;

    /// VLAN Tag Insertion or Replacement shift
    pub const TXDESC_2_VTIR_SHIFT: u32 = 14;
    /// VLAN Tag Insertion or Replacement
    #[repr(u32)]
    #[allow(non_camel_case_types)]
    pub enum TXDESC_2_VTIR {
        DontAdd = 0b00 << 14,
        RemoveTransmitVlanTag = 0b01 << TXDESC_2_VTIR_SHIFT,
        InsertVlanTag = 0b10 << TXDESC_2_VTIR_SHIFT,
        ReplaceVlanTag = 0b11 << TXDESC_2_VTIR_SHIFT,
    }
    /// VLAN Tag Insertion Or Replacement mask
    pub const TXDESC_2_VTIR_MASK: u32 = 0b11 << TXDESC_2_VTIR_SHIFT;

    /// Header or Buffer 1 length shift
    pub const TXDESC_2_HEAD_B1L_SHIFT: u32 = 0;
    /// Header or Buffer 1 length mask
    pub const TXDESC_2_HEAD_B1L_MASK: u32 = 0x3FFF << TXDESC_2_HEAD_B1L_SHIFT;

    // CRC Pad Control shift
    pub const TXDESC_3_CPC_SHIFT: u32 = 26;
    /// CRC Pad Control
    #[repr(u32)]
    #[allow(non_camel_case_types)]
    pub enum TXDESC_3_CPC {
        CRCAndPadInsertion = 0b00 << TXDESC_3_CPC_SHIFT,
        CRCInsertionOnly = 0b01 << TXDESC_3_CPC_SHIFT,
        Disabled = 0b10 << TXDESC_3_CPC_SHIFT,
        CRCReplacement = 0b11 << TXDESC_3_CPC_SHIFT,
    }
    /// CRC Pad Control mask
    pub const TXDESC_3_CPC_MASK: u32 = 0b11 << TXDESC_3_CPC_SHIFT;

    /// Checksum Insertion Control shift
    pub const TXDESC_3_CIC_SHIFT: u32 = 16;
    /// Checksum Insertion Control
    #[repr(u32)]
    #[allow(non_camel_case_types)]
    pub enum TXDESC_3_CIC {
        Disabled = 0b00 << TXDESC_3_CIC_SHIFT,
        IpHeaderOnly = 0b01 << TXDESC_3_CIC_SHIFT,
        IpHeaderAndPayloadOnly = 0b10 << TXDESC_3_CIC_SHIFT,
        IpHeaderAndPayloadAndPseudoHeader = 0b11 << TXDESC_3_CIC_SHIFT,
    }
    /// Checksum Insertion Control mask
    pub const TXDESC_3_CIC_MASK: u32 = 0b11 << TXDESC_3_CIC_SHIFT;

    /// Packet length shift
    pub const TXDESC_3_FL_SHIFT: u32 = 0;
    /// Packet length mask
    pub const TXDESC_3_FL_MASK: u32 = 0x3FFF << TXDESC_3_FL_SHIFT;

    // Write back format
    /// Tx Timestamp status
    pub const TXDESC_3_TTSS: u32 = 1 << 17;
    /// Error Summary
    pub const TXDESC_3_ES: u32 = 1 << 15;
    /// Jabber timeout
    pub const TXDESC_3_JT: u32 = 1 << 14;
    /// Packet flushed
    pub const TXDESC_3_FF: u32 = 1 << 13;
    /// Payload Checksum Error
    pub const TXDESC_3_PCE: u32 = 1 << 12;
    /// Loss of Carrier
    pub const TXDESC_3_LOC: u32 = 1 << 11;
    /// No Carrier
    pub const TXDESC_3_NC: u32 = 1 << 10;
    /// Late Collision
    pub const TXDESC_3_LC: u32 = 1 << 9;
    /// Excessive Collision
    pub const TXDESC_3_EC: u32 = 1 << 8;

    /// Collision count shift
    pub const TXDESC_3_CC_SHIFT: u32 = 4;
    /// Collision Count mask
    pub const TXDESC_3_CC_MASK: u32 = 0b1111 << TXDESC_3_CC_SHIFT;

    /// Excessive Deferral
    pub const TXDESC_3_ED: u32 = 1 << 3;
    /// Underflow error
    pub const TXDESC_3_UF: u32 = 1 << 2;
    /// Deferred Bit
    pub const TXDESC_3_DB: u32 = 1 << 1;
    /// IP Header Error
    pub const TXDESC_3_IHE: u32 = 1 << 0;
}
pub use consts::*;

/// A TX DMA Ring Descriptor
#[repr(C, align(4))]
#[derive(Clone, Copy)]
pub struct TxDescriptor {
    // TODO remove pub
    pub inner_raw: RawDescriptor,
    cache: Cache,
}

impl Default for TxDescriptor {
    fn default() -> Self {
        Self::new()
    }
}

impl TxDescriptor {
    /// Creates an zeroed TxDescriptor.
    pub const fn new() -> Self {
        Self {
            inner_raw: RawDescriptor::new(),
            cache: Cache::new(),
        }
    }

    #[allow(unused)]
    fn is_last(&self) -> bool {
        (self.inner_raw.read(3) & TXDESC_3_LD) == TXDESC_3_LD
    }

    pub(super) fn setup(&mut self) {
        // Zero-out all fields in the descriptor
        (0..4).for_each(|n| unsafe { self.inner_raw.write(n, 0) });
        self.cache.set_id_and_clear_ts(None);
    }

    pub(super) fn is_owned(&self) -> bool {
        (self.inner_raw.read(3) & TXDESC_3_OWN) == TXDESC_3_OWN
    }

    pub(super) fn is_available(&self) -> bool {
        !self.is_owned()
    }

    #[allow(unused)]
    pub(super) fn is_context(&self) -> bool {
        (self.inner_raw.read(3) & TXDESC_3_CTXT) == TXDESC_3_CTXT
    }

    /// Pass ownership to the DMA engine
    pub(super) fn send(&mut self, packet_id: Option<PacketId>, buffer: &[u8]) {
        self.set_buffer(buffer);

        if packet_id.is_some() && cfg!(feature = "ptp") {
            unsafe {
                self.inner_raw.modify(2, |w| w | TXDESC_2_TTSE);
            }
        }

        self.cache.set_id_and_clear_ts(packet_id);

        // "Preceding reads and writes cannot be moved past subsequent writes."
        atomic::fence(Ordering::Release);
        atomic::compiler_fence(Ordering::Release);

        unsafe {
            self.inner_raw.modify(2, |w| w | TXDESC_2_IOC);

            let tx_len =
                ((buffer.len() as u32) << TXDESC_3_FL_SHIFT) & TXDESC_3_FL_MASK;

            self.inner_raw.modify(3, |w| {
                w | TXDESC_3_OWN
                    | TXDESC_3_CIC::IpHeaderAndPayloadAndPseudoHeader as u32
                    | TXDESC_3_FD
                    | TXDESC_3_LD
                    | tx_len
            })
        };

        // Used to flush the store buffer as fast as possible to make the buffer available for the
        // DMA.
        #[cfg(feature = "fence")]
        atomic::fence(Ordering::SeqCst);
    }

    /// Configure the buffer to use for transmitting,
    /// setting it to `buffer`.
    fn set_buffer(&mut self, buffer: &[u8]) {
        unsafe {
            let ptr = buffer.as_ptr();

            // Set buffer pointer 1 to the provided buffer.
            self.inner_raw.write(0, ptr as u32);
            // Set buffer pointer 2 to NULL
            self.inner_raw.write(1, 0);

            self.inner_raw.modify(2, |w| {
                // Clear out B1L
                let w = w & !TXDESC_2_HEAD_B1L_MASK;
                // Clear out B2L
                let w = w & !TXDESC_2_B2L_MASK;
                // Set B1L
                w | ((buffer.len() as u32) << TXDESC_2_HEAD_B1L_SHIFT)
                    & TXDESC_2_HEAD_B1L_MASK
            });
        }
    }
}

#[cfg(feature = "ptp")]
impl TxDescriptor {
    pub(super) fn has_packet_id(&self, packet_id: &PacketId) -> bool {
        self.cache.id().as_ref() == Some(packet_id)
    }

    /// For the TxDescriptor we ignore [`Cache::ts`] because:
    /// * We're only really using the cache so that the size of RxDescriptor and TxDescriptor
    ///   is the same.
    /// * We want to be able to retrieve the timestamp immutably.
    /// * The Timestamp in the TX descriptor is valid until we perform another transmission.
    pub(super) fn timestamp(&self) -> Option<Timestamp> {
        let contains_timestamp =
            (self.inner_raw.read(3) & TXDESC_3_TTSS) == TXDESC_3_TTSS;
        #[cfg(feature = "defmt")]
        defmt::info!(
            "Is owned: {}, contains ts: {}, is last {}",
            self.is_owned(),
            contains_timestamp,
            self.is_last()
        );
        if !self.is_owned() && contains_timestamp && self.is_last() {
            let (low, high) = (self.inner_raw.read(0), self.inner_raw.read(1));
            Some(Timestamp::from_parts(high, low))
        } else {
            None
        }
    }
}
