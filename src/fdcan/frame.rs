use core::cmp::Ordering;

use super::Id;
use super::IdReg;

use super::filter::FilterId;

use super::message_ram::enums::FrameFormat as PacFrameFormat;
use super::message_ram::{RxFifoElementHeader, TxBufferElementHeader};

use super::message_ram::enums::RemoteTransmissionRequest;
use super::message_ram::enums::{DataLength, FilterFrameMatch};

/// Type of Frame
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "unstable-defmt", derive(defmt::Format))]
pub enum FrameFormat {
    /// Frame used by Classic CAN
    Standard = 0,
    /// New frame format used by FdCan
    Fdcan = 1,
}
impl From<FrameFormat> for PacFrameFormat {
    fn from(ff: FrameFormat) -> Self {
        match ff {
            FrameFormat::Standard => PacFrameFormat::Standard,
            FrameFormat::Fdcan => PacFrameFormat::Fdcan,
        }
    }
}
impl From<PacFrameFormat> for FrameFormat {
    fn from(ff: PacFrameFormat) -> Self {
        match ff {
            PacFrameFormat::Standard => FrameFormat::Standard,
            PacFrameFormat::Fdcan => FrameFormat::Fdcan,
        }
    }
}

/// Priority of a CAN frame.
///
/// The priority of a frame is determined by the bits that are part of the *arbitration field*.
/// These consist of the frame identifier bits (including the *IDE* bit, which is 0 for extended
/// frames and 1 for standard frames), as well as the *RTR* bit, which determines whether a frame
/// is a data or remote frame. Lower values of the *arbitration field* have higher priority.
///
/// This struct wraps the *arbitration field* and implements `PartialOrd` and `Ord` accordingly,
/// ordering higher priorities greater than lower ones.
#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "unstable-defmt", derive(defmt::Format))]
pub struct FramePriority(pub(crate) IdReg);

/// Ordering is based on the Identifier and frame type (data vs. remote) and can be used to sort
/// frames by priority.
impl Ord for FramePriority {
    fn cmp(&self, other: &Self) -> Ordering {
        self.0.cmp(&other.0)
    }
}

impl PartialOrd for FramePriority {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl PartialEq for FramePriority {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == Ordering::Equal
    }
}

impl Eq for FramePriority {}

/// Header of a transmit request
#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "unstable-defmt", derive(defmt::Format))]
pub struct TxFrameHeader {
    /// Length of the data in bytes
    pub len: u8,
    /// Type of message
    pub frame_format: FrameFormat,
    /// Id
    pub id: Id,
    /// Should we use bit rate switching
    ///
    /// Not that this is a request and if the global frame_transmit is set to ClassicCanOnly
    /// this is ignored.
    pub bit_rate_switching: bool,
    //pub error_state: Option<()>, //TODO
    ///
    pub marker: Option<u8>,
}
impl From<TxFrameHeader> for IdReg {
    fn from(header: TxFrameHeader) -> IdReg {
        let id: IdReg = header.id.into();
        id.with_rtr(header.len == 0)
    }
}

pub(crate) trait MergeTxFrameHeader {
    fn merge(&self, header: TxFrameHeader);
}
impl MergeTxFrameHeader for TxBufferElementHeader {
    fn merge(&self, header: TxFrameHeader) {
        let id: IdReg = header.id.into();
        self.write(|w| {
            unsafe { w.id().bits(id.as_raw_id()) }
                .rtr()
                .bit(header.len == 0)
                .xtd()
                .set_id_type(header.id.into())
                .set_len(DataLength::new(header.len, header.frame_format.into()))
                .set_event(header.marker.into())
                .fdf()
                .set_format(header.frame_format.into())
                .brs()
                .bit(header.bit_rate_switching)
            //esi.set_error_indicator(//TODO//)
        });
    }
}

impl From<&TxBufferElementHeader> for TxFrameHeader {
    fn from(reg: &TxBufferElementHeader) -> Self {
        let reader = reg.read();
        let id = reader.id().bits();
        let rtr = reader.rtr().rtr();
        let xtd = reader.xtd().id_type();
        let len = reader.to_data_length();
        let ff: PacFrameFormat = len.into();
        TxFrameHeader {
            len: len.len(),
            frame_format: ff.into(),
            id: IdReg::from_register(id, rtr, xtd).into(),
            bit_rate_switching: reader.brs().is_with_brs(),
            //error_state: Option<()>, //TODO
            marker: reader.to_event().into(),
        }
    }
}

/// Header of a Received Frame
#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "unstable-defmt", derive(defmt::Format))]
pub struct RxFrameInfo {
    /// Length in bytes
    pub len: u8,
    /// Frame Format
    pub frame_format: FrameFormat,
    /// Id
    pub id: Id,
    /// Is this an Remote Transmit Request
    pub rtr: bool,
    /// Did this message match any filters
    pub filter_match: Option<FilterId>,
    /// was this received with bit rate switching
    pub bit_rate_switching: bool,
    //pub error_state: (), //TODO
    /// Time stamp counter
    pub time_stamp: u16,
}
impl RxFrameInfo {
    /// Transforms an RxFrameInfo into an TxFrameHeader
    pub fn to_tx_header(self, marker: Option<u8>) -> TxFrameHeader {
        TxFrameHeader {
            len: self.len,
            frame_format: self.frame_format,
            id: self.id,
            bit_rate_switching: self.bit_rate_switching,
            marker,
        }
    }
}
impl From<&RxFifoElementHeader> for RxFrameInfo {
    fn from(reg: &RxFifoElementHeader) -> Self {
        let reader = reg.read();
        let len = reader.to_data_length();
        let ff: PacFrameFormat = len.into();
        let id = reader.id().bits();
        let rtr = reader.rtr().rtr();
        let xtd = reader.xtd().id_type();
        let id = IdReg::from_register(id, rtr, xtd).to_id();
        let filter = reader.to_filter_match();
        let filter = match filter {
            FilterFrameMatch::DidNotMatch => None,
            FilterFrameMatch::DidMatch(filter) => Some(match id {
                Id::Standard(_) => FilterId::Standard(filter.into()),
                Id::Extended(_) => FilterId::Extended(filter.into()),
            }),
        };
        RxFrameInfo {
            len: len.len(),
            frame_format: ff.into(),
            id,
            rtr: rtr == RemoteTransmissionRequest::TransmitRemoteFrame,
            filter_match: filter,
            bit_rate_switching: reader.brs().is_with_brs(),
            time_stamp: reader.txts().bits(),
            //error_state //TODO
        }
    }
}
