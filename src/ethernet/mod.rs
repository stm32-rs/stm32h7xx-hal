//! This module implements a [smoltcp][] device interface `phy::Device` for
//! the STM32H7 series of microcontrollers.
//!
//! Multiple PHYs are supported:
//! - SMSC LAN8742a
//! - Micrel KSZ8081R
//!

#[cfg(feature = "phy_ksz8081r")]
mod ksz8081r;
#[cfg(feature = "phy_lan8742a")]
mod lan8742a;
#[cfg(not(any(feature = "phy_ksz8081r", feature = "phy_lan8742a")))]
compile_error!(
    "A least one PHY device must be enabled. Use a feature gate to
enable."
);
#[cfg(all(feature = "phy_ksz8081r", feature = "phy_lan8742a"))]
compile_error!("Cannot enable multiple PHY devices.");

mod shared;
/// PHY address
pub const ETH_PHY_ADDR: u8 = 0;

/// Station Management Interface (SMI) on an ethernet PHY
pub trait StationManagement {
    /// Read a register over SMI.
    fn smi_read(&mut self, reg: u8) -> u16;
    /// Write a register over SMI.
    fn smi_write(&mut self, reg: u8, val: u16);
}

/// Traits for an Ethernet PHY
trait PHY {
    /// Reset PHY and wait for it to come out of reset.
    fn phy_reset(&mut self);
    /// PHY initialisation.
    fn phy_init(&mut self);
}

pub use shared::{enable_interrupt, ethernet_init, interrupt_handler};
pub use shared::{DesRing, EthernetDMA, EthernetMAC};
