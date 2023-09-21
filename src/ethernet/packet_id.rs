/// A packet ID.
///
/// This packet ID can be used to obtain information about a specific
/// ethernet frame (either sent or received) from the DMA.
///
//! This implementation is derived from 0BSD-relicensed work done by 
//! Johannes Draaijer <jcdra1@gmail.com> for the 
//! [`stm32-eth`](https://github.com/stm32-rs/stm32-eth) project

#[cfg_attr(
    feature = "ptp",
    doc = "
The main use is obtaining timestamps for frames using [`EthernetDMA::poll_timestamp`](crate::EthernetDMA::poll_timestamp)
"
)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, PartialEq, Clone, Copy)]
pub struct PacketId(pub u32);

impl PacketId {
    /// The initial value for an [`Option<PacketId>`]
    pub const INIT: Option<Self> = None;
}

impl From<u32> for PacketId {
    fn from(value: u32) -> Self {
        Self(value)
    }
}

#[cfg(feature = "ptp")]
impl From<smoltcp::phy::PacketMeta> for PacketId {
    fn from(value: smoltcp::phy::PacketMeta) -> Self {
        Self(value.id)
    }
}

#[cfg(feature = "ptp")]
impl From<PacketId> for smoltcp::phy::PacketMeta {
    fn from(value: PacketId) -> Self {
        let mut meta = smoltcp::phy::PacketMeta::default();
        meta.id = value.0;
        meta
    }
}
