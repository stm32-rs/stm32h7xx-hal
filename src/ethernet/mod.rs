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
pub use eth::{enable_interrupt, interrupt_handler, new_unchecked};
pub use eth::{DesRing, EthernetDMA, EthernetMAC};

/// Marks a set of pins used to communciate to a PHY with a Reduced Media
/// Independent Interface (RMII)
pub trait PinsRMII {}

impl<REF_CLK, MDIO, MDC, CRS_DV, RXD0, RXD1, TX_EN, TXD0, TXD1> PinsRMII
    for (REF_CLK, MDIO, MDC, CRS_DV, RXD0, RXD1, TX_EN, TXD0, TXD1)
where
    REF_CLK: RefClk,
    MDIO: Mdio,
    MDC: Mdc,
    CRS_DV: CrsDv,
    RXD0: Rxd0,
    RXD1: Rxd1,
    TX_EN: TxEn,
    TXD0: Txd0,
    TXD1: Txd1,
{
    // RMII
}

/// Marks a type as a REF_CLK pin
pub trait RefClk {}
/// Marks a type as a TX_CLK pin
pub trait TxClk {}
/// Marks a type as a MDIO pin
pub trait Mdio {}
/// Marks a type as a MDC pin
pub trait Mdc {}
/// Marks a type as a COL pin
pub trait Col {}
/// Marks a type as a CRS pin
pub trait Crs {}
/// Marks a type as a CRS_DV pin
pub trait CrsDv {}
/// Marks a type as a PPS_OUT pin
pub trait PpsOut {}
/// Marks a type as a RX_ER pin
pub trait RxEr {}
/// Marks a type as a TX_EN pin
pub trait TxEn {}
/// Marks a type as a RXD0 pin
pub trait Rxd0 {}
/// Marks a type as a RXD1 pin
pub trait Rxd1 {}
/// Marks a type as a RXD2 pin
pub trait Rxd2 {}
/// Marks a type as a RXD3 pin
pub trait Rxd3 {}
/// Marks a type as a TXD0 pin
pub trait Txd0 {}
/// Marks a type as a TXD1 pin
pub trait Txd1 {}
/// Marks a type as a TXD2 pin
pub trait Txd2 {}
/// Marks a type as a TXD3 pin
pub trait Txd3 {}

use crate::gpio::gpioa::{PA0, PA1, PA2, PA3, PA7};
use crate::gpio::gpiob::{PB0, PB1, PB10, PB11, PB12, PB13, PB5, PB8};
use crate::gpio::gpioc::{PC1, PC2, PC3, PC4, PC5};
use crate::gpio::gpioe::PE2;
use crate::gpio::gpiog::{PG11, PG12, PG13, PG14, PG8};
use crate::gpio::gpioh::{PH2, PH3, PH6, PH7};
#[cfg(not(feature = "rm0468"))]
use crate::gpio::gpioi::PI10;
use crate::gpio::{Alternate, AF11};

impl RefClk for PA1<Alternate<AF11>> {}

impl TxClk for PC3<Alternate<AF11>> {}

impl Mdio for PA2<Alternate<AF11>> {}

impl Mdc for PC1<Alternate<AF11>> {}

impl Col for PA3<Alternate<AF11>> {}
impl Col for PH3<Alternate<AF11>> {}

impl Crs for PA0<Alternate<AF11>> {}
impl Crs for PH2<Alternate<AF11>> {}

impl CrsDv for PA7<Alternate<AF11>> {}

impl PpsOut for PB5<Alternate<AF11>> {}
impl PpsOut for PG8<Alternate<AF11>> {}

impl RxEr for PB10<Alternate<AF11>> {}
#[cfg(not(feature = "rm0468"))]
impl RxEr for PI10<Alternate<AF11>> {}

impl TxEn for PB11<Alternate<AF11>> {}
impl TxEn for PG11<Alternate<AF11>> {}

impl Rxd0 for PC4<Alternate<AF11>> {}

impl Rxd1 for PC5<Alternate<AF11>> {}

impl Rxd2 for PB0<Alternate<AF11>> {}
impl Rxd2 for PH6<Alternate<AF11>> {}

impl Rxd3 for PB1<Alternate<AF11>> {}
impl Rxd3 for PH7<Alternate<AF11>> {}

impl Txd0 for PB12<Alternate<AF11>> {}
impl Txd0 for PG13<Alternate<AF11>> {}

impl Txd1 for PB13<Alternate<AF11>> {}
impl Txd1 for PG12<Alternate<AF11>> {}
impl Txd1 for PG14<Alternate<AF11>> {}

impl Txd2 for PC2<Alternate<AF11>> {}

impl Txd3 for PB8<Alternate<AF11>> {}
impl Txd3 for PE2<Alternate<AF11>> {}
