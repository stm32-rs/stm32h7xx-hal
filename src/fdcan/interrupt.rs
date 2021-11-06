//! Interrupt types.

use core::ops;

#[allow(unused_imports)] // for intra-doc links only
use crate::fdcan::{FdCan, Rx};

/// FdCAN interrupt sources.
///
/// These can be individually enabled and disabled in the FdCAN peripheral. Note that each FdCAN
/// peripheral only exposes 2 interrupts to the microcontroller:
///
/// FDCANx_INTR0,
/// FDCANx_INTR1,
///
/// The interrupts available on each line can be configured using the [`config::FdCanConfig`] struct.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "unstable-defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Interrupt {
    /// Rx FIFO 0 has a new message
    RxFifo0NewMsg = 1 << 0,
    /// Rx FIFO 0 is full
    RxFifo0Full = 1 << 1,
    /// Rx FIFO 0 has lost a message
    RxFifo0MsgLost = 1 << 2,
    /// Rx FIFO 1 has a new message
    RxFifo1NewMsg = 1 << 3,
    /// Rx FIFO 1 is full
    RxFifo1Full = 1 << 4,
    /// Rx FIFO 1 has lost a message
    RxFifo1MsgLost = 1 << 5,
    /// A High Priority Message has been flagged by a filter
    RxHighPrio = 1 << 6,
    /// Transmit has been completed
    TxComplete = 1 << 7,
    /// Tx message has been cancelled
    TxCancel = 1 << 8,
    /// Tx Fifo is empty
    TxEmpty = 1 << 9,
    /// An new Event has been received in the Tx Event Fifo
    TxEventNew = 1 << 10,
    /// The TxEvent Fifo is full
    TxEventFull = 1 << 11,
    /// An Tx Event has been lost
    TxEventLost = 1 << 12,
    /// Timestamp wrap around has occurred
    TsWrapAround = 1 << 13,
    /// Message RAM access failure
    ///
    /// The flag is set when the Rx handler:
    /// has not completed acceptance filtering or storage of an accepted message until the
    /// arbitration field of the following message has been received. In this case acceptance
    /// filtering or message storage is aborted and the Rx handler starts processing of the
    /// following message. was unable to write a message to the message RAM. In this case
    /// message storage is aborted.
    /// In both cases the FIFO put index is not updated. The partly stored message is overwritten
    /// when the next message is stored to this location.
    /// The flag is also set when the Tx Handler was not able to read a message from the Message
    /// RAM in time. In this case message transmission is aborted. In case of a Tx Handler access
    /// failure the FDCAN is switched into Restricted operation Mode (see Restricted operation
    /// mode).
    MsgRamAccessFailure = 1 << 14,
    /// Timeout Occurred
    TimeoutOccurred = 1 << 15,
    /// Overflow of CAN error logging counter occurred
    ErrLogOverflow = 1 << 16,
    /// Errr Passive
    ErrPassive = 1 << 17,
    /// Warning Status
    WarningStatus = 1 << 18,
    /// Bus_Off status
    BusOff = 1 << 19,
    ///  Watchdog interrupt
    WatchdogInt = 1 << 20,
    /// Protocol error in arbitration phase (nominal bit time is used)
    ProtErrArbritation = 1 << 21,
    /// Protocol error in data phase (data bit time is used)
    ProtErrData = 1 << 22,
    /// Access to reserved address
    ReservedAccess = 1 << 23,
}

bitflags::bitflags! {
    /// A set of FdCAN interrupts.
    #[cfg_attr(feature = "unstable-defmt", derive(defmt::Format))]
    pub struct Interrupts: u32 {
        /// Rx FIFO 0 has a new message
        const RX_FIFO_0_NEW_MESSAGE = 1 << 0;
        /// Rx FIFO 0 is full
        const RX_FIFO_0_FULL = 1 << 1;
        /// Rx FIFO 0 has lost a message
        const RX_FIFO_0_MSG_LOST = 1 << 2;
        /// Rx FIFO 1 has a new message
        const RX_FIFO_1_NEW_MESSAGE = 1 << 3;
        /// Rx FIFO 1 is full
        const RX_FIFO_1_FULL = 1 << 4;
        /// Rx FIFO 1 has lost a message
        const RX_FIFO_1_MSG_LOST = 1 << 5;
        /// A High Priority Message has been flagged by a filter
        const RX_HIGH_PRIO_MSG = 1<<6;
        /// Transmit has been completed
        const TX_COMPLETE = 1<<7;
        /// Tx message has been cancelled
        const TX_CANCEL = 1<<8;
        /// Tx Fifo is empty
        const TX_EMPTY = 1<<9;
        /// An new Event has been received in the Tx Event Fifo
        const TX_EVENT_NEW = 1<<10;
        /// The TxEvent Fifo is full
        const TX_EVENT_FULL = 1<<11;
        /// An Tx Event has been lost
        const TX_EVENT_LOST = 1<<12;
        /// Timestamp wrap around has occurred
        const TS_WRAP_AROUND = 1<<13;
        /// Message RAM access failure
        ///
        /// The flag is set when the Rx handler:
        /// has not completed acceptance filtering or storage of an accepted message until the
        /// arbitration field of the following message has been received. In this case acceptance
        /// filtering or message storage is aborted and the Rx handler starts processing of the
        /// following message. was unable to write a message to the message RAM. In this case
        /// message storage is aborted.
        /// In both cases the FIFO put index is not updated. The partly stored message is overwritten
        /// when the next message is stored to this location.
        /// The flag is also set when the Tx Handler was not able to read a message from the Message
        /// RAM in time. In this case message transmission is aborted. In case of a Tx Handler access
        /// failure the FDCAN is switched into Restricted operation Mode (see Restricted operation
        /// mode).
        const MSG_RAM_ACCESS_FAILURE = 1<<14;
        /// Timeout Occurred
        const TIMEOUT_OCCURRED = 1<<15;
        /// Overflow of CAN error logging counter occurred
        const ERR_LOG_OVERFLOW = 1<<16;
        /// Err Passive
        const ERR_PASSIVE = 1<<17;
        /// Warning Status
        const WARNING_STATUS = 1<<18;
        /// Bus_Off status
        const BUS_OFF = 1<<19;
        ///  Watchdog interrupt
        const WATCHDOG_INT = 1<<20;
        /// Protocol error in arbitration phase (nominal bit time is used)
        const PROT_ERR_ARBRITATION = 1<<21;
        /// Protocol error in data phase (data bit time is used)
        const PROT_ERR_DATA = 1<<22;
        /// Access to reserved address
        const RESERVED_ACCESS = 1<<23;
    }
}

impl Interrupts {
    /// No Interrupt masks selected
    pub fn none() -> Self {
        Self::from_bits_truncate(0)
    }
}

impl From<Interrupt> for Interrupts {
    #[inline]
    fn from(i: Interrupt) -> Self {
        Self::from_bits_truncate(i as u32)
    }
}

/// Adds an interrupt to the interrupt set.
impl ops::BitOrAssign<Interrupt> for Interrupts {
    #[inline]
    fn bitor_assign(&mut self, rhs: Interrupt) {
        *self |= Self::from(rhs);
    }
}

/// There are two interrupt lines for the FdCan
/// The events linked to these can be configured
/// see `[config::FdCanConfig]`
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
#[cfg_attr(feature = "unstable-defmt", derive(defmt::Format))]
pub enum InterruptLine {
    /// Interrupt Line 0
    _0 = 0,
    /// Interrupt Line 1
    _1 = 1,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn interrupt_flags() {
        assert_eq!(
            Interrupts::from(Interrupt::TxComplete),
            Interrupts::TX_COMPLETE
        );
        assert_eq!(Interrupts::from(Interrupt::TxEmpty), Interrupts::TX_EMPTY);

        let mut ints = Interrupts::RX_FIFO0_FULL;
        ints |= Interrupt::RxFifo1Full;
        assert_eq!(ints, Interrupts::RX_FIFO0_FULL | Interrupts::RX_FIFO1_FULL);
    }
}
