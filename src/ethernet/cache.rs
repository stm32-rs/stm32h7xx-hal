#[cfg(feature = "ptp")]
use crate::ptp::Timestamp;

use super::PacketId;

/// A cache for timestamping information,
/// used to lessen the size of Descriptors.
#[derive(Clone, Copy)]
#[repr(C, packed)]
pub struct Cache {
    has_packet_id: bool,
    #[cfg(feature = "ptp")]
    has_timestamp: bool,

    packet_id: PacketId,
    #[cfg(feature = "ptp")]
    timestamp: Timestamp,
}

impl Default for Cache {
    fn default() -> Self {
        Self::new()
    }
}

impl Cache {
    pub const fn new() -> Self {
        Self {
            has_packet_id: false,
            packet_id: PacketId(0),

            #[cfg(feature = "ptp")]
            has_timestamp: false,
            #[cfg(feature = "ptp")]
            timestamp: Timestamp::new_raw(0),
        }
    }

    /// Set the packet ID of this [`Cache`]
    ///
    /// Removes the timestamp id if `ptp` feature is enabled.
    pub fn set_id_and_clear_ts(&mut self, packet_id: Option<PacketId>) {
        #[cfg(feature = "ptp")]
        {
            self.has_timestamp = false;
        }

        if let Some(id) = packet_id {
            self.packet_id = id;
            self.has_packet_id = true
        } else {
            self.has_packet_id = false;
        }
    }

    // NOTE(unused): this function is not used if not(feature = "ptp")
    #[allow(unused)]
    pub fn id(&self) -> Option<PacketId> {
        if self.has_packet_id {
            Some(self.packet_id)
        } else {
            None
        }
    }
}

#[cfg(feature = "ptp")]
impl Cache {
    pub fn set_ts(&mut self, timestamp: Option<Timestamp>) {
        if let Some(ts) = timestamp {
            self.timestamp = ts;
            self.has_timestamp = true;
        } else {
            self.has_timestamp = false;
        }
    }

    pub fn ts(&self) -> Option<Timestamp> {
        if self.has_timestamp {
            Some(self.timestamp)
        } else {
            None
        }
    }
}
