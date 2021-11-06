//! CAN Identifiers.

use core::cmp::{Ord, Ordering};

use super::message_ram::enums::{IdType, RemoteTransmissionRequest};

/// Standard 11-bit CAN Identifier (`0..=0x7FF`).
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "unstable-defmt", derive(defmt::Format))]
pub struct StandardId(u16);

impl StandardId {
    /// CAN ID `0`, the highest priority.
    pub const ZERO: Self = Self(0);

    /// CAN ID `0x7FF`, the lowest priority.
    pub const MAX: Self = Self(0x7FF);

    /// Tries to create a `StandardId` from a raw 16-bit integer.
    ///
    /// This will return `None` if `raw` is out of range of an 11-bit integer (`> 0x7FF`).
    #[inline]
    pub const fn new(raw: u16) -> Option<Self> {
        if raw <= 0x7FF {
            Some(Self(raw))
        } else {
            None
        }
    }

    /// Creates a new `StandardId` without checking if it is inside the valid range.
    ///
    /// # Safety
    ///
    /// The caller must ensure that `raw` is in the valid range, otherwise the behavior is
    /// undefined.
    #[inline]
    pub const unsafe fn new_unchecked(raw: u16) -> Self {
        Self(raw)
    }

    /// Returns this CAN Identifier as a raw 16-bit integer.
    #[inline]
    pub(crate) fn as_raw(&self) -> u16 {
        self.0
    }
}
impl From<StandardId> for IdType {
    fn from(_id: StandardId) -> Self {
        IdType::StandardId
    }
}

/// Extended 29-bit CAN Identifier (`0..=1FFF_FFFF`).
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "unstable-defmt", derive(defmt::Format))]
pub struct ExtendedId(u32);

impl ExtendedId {
    /// CAN ID `0`, the highest priority.
    pub const ZERO: Self = Self(0);

    /// CAN ID `0x1FFFFFFF`, the lowest priority.
    pub const MAX: Self = Self(0x1FFF_FFFF);

    /// Tries to create a `ExtendedId` from a raw 32-bit integer.
    ///
    /// This will return `None` if `raw` is out of range of an 29-bit integer (`> 0x1FFF_FFFF`).
    #[inline]
    pub const fn new(raw: u32) -> Option<Self> {
        if raw <= 0x1FFF_FFFF {
            Some(Self(raw))
        } else {
            None
        }
    }

    /// Creates a new `ExtendedId` without checking if it is inside the valid range.
    ///
    /// # Safety
    ///
    /// The caller must ensure that `raw` is in the valid range, otherwise the behavior is
    /// undefined.
    #[inline]
    pub const unsafe fn new_unchecked(raw: u32) -> Self {
        Self(raw)
    }

    /// Returns this CAN Identifier as a raw 32-bit integer.
    #[inline]
    pub(crate) fn as_raw(&self) -> u32 {
        self.0
    }

    /// Returns the Base ID part of this extended identifier.
    pub fn standard_id(&self) -> StandardId {
        // ID-28 to ID-18
        StandardId((self.0 >> 18) as u16)
    }
}

impl From<ExtendedId> for IdType {
    fn from(_id: ExtendedId) -> Self {
        IdType::ExtendedId
    }
}

/// A CAN Identifier (standard or extended).
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "unstable-defmt", derive(defmt::Format))]
pub enum Id {
    /// Standard 11-bit Identifier (`0..=0x7FF`).
    Standard(StandardId),

    /// Extended 29-bit Identifier (`0..=0x1FFF_FFFF`).
    Extended(ExtendedId),
}

impl From<StandardId> for Id {
    #[inline]
    fn from(id: StandardId) -> Self {
        Id::Standard(id)
    }
}

impl From<ExtendedId> for Id {
    #[inline]
    fn from(id: ExtendedId) -> Self {
        Id::Extended(id)
    }
}

impl From<Id> for IdType {
    #[inline]
    fn from(id: Id) -> Self {
        match id {
            Id::Standard(id) => id.into(),
            Id::Extended(id) => id.into(),
        }
    }
}

/// Identifier of a CAN message.
///
/// FdCan be either a standard identifier (11bit, Range: 0..0x3FF) or a
/// extendended identifier (29bit , Range: 0..0x1FFFFFFF).
///
/// The `Ord` trait can be used to determine the frame’s priority this ID
/// belongs to.
/// Lower identifier values have a higher priority. Additionally standard frames
/// have a higher priority than extended frames and data frames have a higher
/// priority than remote frames.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "unstable-defmt", derive(defmt::Format))]
pub(crate) struct IdReg(u32);

impl IdReg {
    const STANDARD_SHIFT: u32 = 18;
    #[allow(dead_code)]
    const STANDARD_MASK: u32 = 0x1FFC0000;

    const EXTENDED_SHIFT: u32 = 0;
    const EXTENDED_MASK: u32 = 0x1FFFFFFF;

    const XTD_SHIFT: u32 = 30;
    const XTD_MASK: u32 = 1 << Self::XTD_SHIFT;

    const RTR_SHIFT: u32 = 29;
    const RTR_MASK: u32 = 1 << Self::RTR_SHIFT;

    /// Creates a new standard identifier (11bit, Range: 0..0x7FF)
    ///
    /// Panics for IDs outside the allowed range.
    fn new_standard(id: StandardId) -> Self {
        Self(u32::from(id.as_raw()) << Self::STANDARD_SHIFT)
    }

    /// Creates a new extendended identifier (29bit , Range: 0..0x1FFFFFFF).
    ///
    /// Panics for IDs outside the allowed range.
    fn new_extended(id: ExtendedId) -> Self {
        Self(id.as_raw() << Self::EXTENDED_SHIFT | (1 << Self::XTD_SHIFT))
    }

    pub(crate) fn as_raw_id(&self) -> u32 {
        self.0 & Self::EXTENDED_MASK
    }

    pub(crate) fn from_register(id: u32, rtr: RemoteTransmissionRequest, xtd: IdType) -> Self {
        let rtr: u32 = match rtr {
            RemoteTransmissionRequest::TransmitDataFrame => 0,
            RemoteTransmissionRequest::TransmitRemoteFrame => 1 << Self::RTR_SHIFT,
        };
        let xtd: u32 = match xtd {
            IdType::StandardId => 0,
            IdType::ExtendedId => 1 << Self::XTD_SHIFT,
        };
        Self(id | rtr | xtd)
    }

    /// Sets the remote transmission (RTR) flag. This marks the identifier as
    /// being part of a remote frame.
    #[must_use = "returns a new IdReg without modifying `self`"]
    pub(crate) fn with_rtr(self, rtr: bool) -> Self {
        if rtr {
            Self(self.0 | (1 << Self::RTR_SHIFT))
        } else {
            Self(self.0 & !Self::RTR_MASK)
        }
    }

    /// Returns the identifier.
    pub fn to_id(self) -> Id {
        if self.is_extended() {
            Id::Extended(unsafe { ExtendedId::new_unchecked(self.0 >> Self::EXTENDED_SHIFT) })
        } else {
            Id::Standard(unsafe {
                StandardId::new_unchecked((self.0 >> Self::STANDARD_SHIFT) as u16)
            })
        }
    }

    /// Returns `true` if the identifier is an extended identifier.
    pub fn is_extended(self) -> bool {
        (self.0 & Self::XTD_MASK) != 0
    }

    /// Returns `true` if the identifier is a standard identifier.
    pub fn is_standard(self) -> bool {
        !self.is_extended()
    }

    /// Returns `true` if the identifer is part of a remote frame (RTR bit set).
    pub(crate) fn rtr(self) -> bool {
        self.0 & Self::RTR_MASK != 0
    }
}
impl From<Id> for IdReg {
    fn from(id: Id) -> Self {
        match id {
            Id::Standard(s) => IdReg::new_standard(s),
            Id::Extended(e) => IdReg::new_extended(e),
        }
    }
}
impl From<IdReg> for Id {
    fn from(idr: IdReg) -> Self {
        idr.to_id()
    }
}
impl From<IdReg> for IdType {
    #[inline]
    fn from(id: IdReg) -> Self {
        if id.is_standard() {
            IdType::StandardId
        } else {
            IdType::ExtendedId
        }
    }
}
impl From<IdReg> for RemoteTransmissionRequest {
    #[inline]
    fn from(id: IdReg) -> Self {
        if id.rtr() {
            RemoteTransmissionRequest::TransmitRemoteFrame
        } else {
            RemoteTransmissionRequest::TransmitDataFrame
        }
    }
}

/// `IdReg` is ordered by priority.
impl Ord for IdReg {
    fn cmp(&self, other: &Self) -> Ordering {
        // When the IDs match, data frames have priority over remote frames.
        let rtr = self.rtr().cmp(&other.rtr()).reverse();

        let id_a = self.to_id();
        let id_b = other.to_id();
        match (id_a, id_b) {
            (Id::Standard(a), Id::Standard(b)) => {
                // Lower IDs have priority over higher IDs.
                a.as_raw().cmp(&b.as_raw()).reverse().then(rtr)
            }
            (Id::Extended(a), Id::Extended(b)) => a.as_raw().cmp(&b.as_raw()).reverse().then(rtr),
            (Id::Standard(a), Id::Extended(b)) => {
                // Standard frames have priority over extended frames if their Base IDs match.
                a.as_raw()
                    .cmp(&b.standard_id().as_raw())
                    .reverse()
                    .then(Ordering::Greater)
            }
            (Id::Extended(a), Id::Standard(b)) => a
                .standard_id()
                .as_raw()
                .cmp(&b.as_raw())
                .reverse()
                .then(Ordering::Less),
        }
    }
}

impl PartialOrd for IdReg {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
