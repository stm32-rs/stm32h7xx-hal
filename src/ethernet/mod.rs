//! This module implements a [smoltcp][] device interface `phy::Device` for
//! the STM32H7 series of microcontrollers.
//!
//! Multiple PHYs are supported:
//! - SMSC LAN8742a
//! - Micrel KSZ8081R
//!
//! # Examples
//!
//! - [Simple link checker for the Nucleo-H743ZI2](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/ethernet-nucleo-h743zi2.rs)
//! - [Simple link checker for the STM32H747I-DISCO](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/ethernet-stm32h747i-disco.rs)
//! - [Ethernet example for the STM32H747I-DISCO using RTIC](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/ethernet-rtic-stm32h747i-disco.rs)
//! - [Ethernet example for the STM32H735G-DK using RTIC](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/ethernet-rtic-stm32h735g-dk.rs)

use crate::gpio::{alt::eth as alt, PinSpeed, Speed};

/// Station Management Interface (SMI) on an ethernet PHY
pub trait StationManagement {
    /// Read a register over SMI.
    fn smi_read(&mut self, reg: u8) -> u16;
    /// Write a register over SMI.
    fn smi_write(&mut self, reg: u8, val: u16);
}

/// Traits for an Ethernet PHY
pub trait PHY {
    /// Reset PHY and wait for it to come out of reset.
    fn phy_reset(&mut self);
    /// PHY initialisation.
    fn phy_init(&mut self);
}

mod ksz8081r;
mod lan8742a;

/// Some common implementations of the [PHY trait](PHY)
pub mod phy {
    pub use super::ksz8081r::*;
    pub use super::lan8742a::*;
}

mod eth;
pub use eth::{enable_interrupt, interrupt_handler, new, new_unchecked};
pub use eth::{DesRing, EthernetDMA, EthernetMAC};

/// Marks a set of pins used to communciate to a PHY with a Reduced Media
/// Independent Interface (RMII)
pub trait PinsRMII {
    fn set_speed(&mut self, speed: Speed);
}

// Two lanes
pub struct TwoLanesPins {
    pub ref_clk: alt::RefClk,
    pub mdio: alt::Mdio,
    pub mdc: alt::Mdc,
    pub crs_dv: alt::CrsDv,
    pub rxd0: alt::Rxd0,
    pub rxd1: alt::Rxd1,
    pub tx_en: alt::TxEn,
    pub txd0: alt::Txd0,
    pub txd1: alt::Txd1,
}

impl TwoLanesPins {
    pub fn new(
        ref_clk: impl Into<alt::RefClk>,
        mdio: impl Into<alt::Mdio>,
        mdc: impl Into<alt::Mdc>,
        crs_dv: impl Into<alt::CrsDv>,
        rxd0: impl Into<alt::Rxd0>,
        rxd1: impl Into<alt::Rxd1>,
        tx_en: impl Into<alt::TxEn>,
        txd0: impl Into<alt::Txd0>,
        txd1: impl Into<alt::Txd1>,
    ) -> Self {
        Self {
            ref_clk: ref_clk.into(),
            mdio: mdio.into(),
            mdc: mdc.into(),
            crs_dv: crs_dv.into(),
            rxd0: rxd0.into(),
            rxd1: rxd1.into(),
            tx_en: tx_en.into(),
            txd0: txd0.into(),
            txd1: txd1.into(),
        }
    }
}

impl PinsRMII for TwoLanesPins {
    fn set_speed(&mut self, speed: Speed) {
        self.ref_clk.set_speed(speed);
        self.mdio.set_speed(speed);
        self.mdc.set_speed(speed);
        self.crs_dv.set_speed(speed);
        self.rxd0.set_speed(speed);
        self.rxd1.set_speed(speed);
        self.tx_en.set_speed(speed);
        self.txd0.set_speed(speed);
        self.txd1.set_speed(speed);
    }
}

// Four lanes
pub struct FourLanesPins {
    pub ref_clk: alt::RefClk,
    pub mdio: alt::Mdio,
    pub mdc: alt::Mdc,
    pub crs_dv: alt::CrsDv,
    pub rxd0: alt::Rxd0,
    pub rxd1: alt::Rxd1,
    pub rxd2: alt::Rxd2,
    pub rxd3: alt::Rxd3,
    pub tx_en: alt::TxEn,
    pub txd0: alt::Txd0,
    pub txd1: alt::Txd1,
    pub txd2: alt::Txd2,
    pub txd3: alt::Txd3,
}

impl FourLanesPins {
    pub fn new(
        ref_clk: impl Into<alt::RefClk>,
        mdio: impl Into<alt::Mdio>,
        mdc: impl Into<alt::Mdc>,
        crs_dv: impl Into<alt::CrsDv>,
        rxd0: impl Into<alt::Rxd0>,
        rxd1: impl Into<alt::Rxd1>,
        rxd2: impl Into<alt::Rxd2>,
        rxd3: impl Into<alt::Rxd3>,
        tx_en: impl Into<alt::TxEn>,
        txd0: impl Into<alt::Txd0>,
        txd1: impl Into<alt::Txd1>,
        txd2: impl Into<alt::Txd2>,
        txd3: impl Into<alt::Txd3>,
    ) -> Self {
        Self {
            ref_clk: ref_clk.into(),
            mdio: mdio.into(),
            mdc: mdc.into(),
            crs_dv: crs_dv.into(),
            rxd0: rxd0.into(),
            rxd1: rxd1.into(),
            rxd2: rxd2.into(),
            rxd3: rxd3.into(),
            tx_en: tx_en.into(),
            txd0: txd0.into(),
            txd1: txd1.into(),
            txd2: txd2.into(),
            txd3: txd3.into(),
        }
    }
}

impl PinsRMII for FourLanesPins {
    fn set_speed(&mut self, speed: Speed) {
        self.ref_clk.set_speed(speed);
        self.mdio.set_speed(speed);
        self.mdc.set_speed(speed);
        self.crs_dv.set_speed(speed);
        self.rxd0.set_speed(speed);
        self.rxd1.set_speed(speed);
        self.rxd2.set_speed(speed);
        self.rxd3.set_speed(speed);
        self.tx_en.set_speed(speed);
        self.txd0.set_speed(speed);
        self.txd1.set_speed(speed);
        self.txd2.set_speed(speed);
        self.txd3.set_speed(speed);
    }
}
