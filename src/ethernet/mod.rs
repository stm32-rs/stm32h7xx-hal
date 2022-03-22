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

use crate::gpio::Speed;

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
    fn set_speed(self, speed: Speed) -> Self;
}

// Two lanes
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
    fn set_speed(self, speed: Speed) -> Self {
        (
            self.0.set_speed(speed),
            self.1.set_speed(speed),
            self.2.set_speed(speed),
            self.3.set_speed(speed),
            self.4.set_speed(speed),
            self.5.set_speed(speed),
            self.6.set_speed(speed),
            self.7.set_speed(speed),
            self.8.set_speed(speed),
        )
    }
}

// Four lanes
impl<
        REF_CLK,
        MDIO,
        MDC,
        CRS_DV,
        RXD0,
        RXD1,
        RXD2,
        RXD3,
        TX_EN,
        TXD0,
        TXD1,
        TXD2,
        TXD3,
    > PinsRMII
    for (
        REF_CLK,
        MDIO,
        MDC,
        CRS_DV,
        RXD0,
        RXD1,
        RXD2,
        RXD3,
        TX_EN,
        TXD0,
        TXD1,
        TXD2,
        TXD3,
    )
where
    REF_CLK: RefClk,
    MDIO: Mdio,
    MDC: Mdc,
    CRS_DV: CrsDv,
    RXD0: Rxd0,
    RXD1: Rxd1,
    RXD2: Rxd2,
    RXD3: Rxd3,
    TX_EN: TxEn,
    TXD0: Txd0,
    TXD1: Txd1,
    TXD2: Txd2,
    TXD3: Txd3,
{
    // RMII
    fn set_speed(self, speed: Speed) -> Self {
        (
            self.0.set_speed(speed),
            self.1.set_speed(speed),
            self.2.set_speed(speed),
            self.3.set_speed(speed),
            self.4.set_speed(speed),
            self.5.set_speed(speed),
            self.6.set_speed(speed),
            self.7.set_speed(speed),
            self.8.set_speed(speed),
            self.9.set_speed(speed),
            self.10.set_speed(speed),
            self.11.set_speed(speed),
            self.12.set_speed(speed),
        )
    }
}

/// Marks a type as a REF_CLK pin
pub trait RefClk {
    fn set_speed(self, speed: Speed) -> Self;
}
/// Marks a type as a TX_CLK pin
pub trait TxClk {
    fn set_speed(self, speed: Speed) -> Self;
}
/// Marks a type as a MDIO pin
pub trait Mdio {
    fn set_speed(self, speed: Speed) -> Self;
}
/// Marks a type as a MDC pin
pub trait Mdc {
    fn set_speed(self, speed: Speed) -> Self;
}
/// Marks a type as a COL pin
pub trait Col {
    fn set_speed(self, speed: Speed) -> Self;
}
/// Marks a type as a CRS pin
pub trait Crs {
    fn set_speed(self, speed: Speed) -> Self;
}
/// Marks a type as a CRS_DV pin
pub trait CrsDv {
    fn set_speed(self, speed: Speed) -> Self;
}
/// Marks a type as a PPS_OUT pin
pub trait PpsOut {
    fn set_speed(self, speed: Speed) -> Self;
}
/// Marks a type as a RX_ER pin
pub trait RxEr {
    fn set_speed(self, speed: Speed) -> Self;
}
/// Marks a type as a TX_EN pin
pub trait TxEn {
    fn set_speed(self, speed: Speed) -> Self;
}
/// Marks a type as a RXD0 pin
pub trait Rxd0 {
    fn set_speed(self, speed: Speed) -> Self;
}
/// Marks a type as a RXD1 pin
pub trait Rxd1 {
    fn set_speed(self, speed: Speed) -> Self;
}
/// Marks a type as a RXD2 pin
pub trait Rxd2 {
    fn set_speed(self, speed: Speed) -> Self;
}
/// Marks a type as a RXD3 pin
pub trait Rxd3 {
    fn set_speed(self, speed: Speed) -> Self;
}
/// Marks a type as a TXD0 pin
pub trait Txd0 {
    fn set_speed(self, speed: Speed) -> Self;
}
/// Marks a type as a TXD1 pin
pub trait Txd1 {
    fn set_speed(self, speed: Speed) -> Self;
}
/// Marks a type as a TXD2 pin
pub trait Txd2 {
    fn set_speed(self, speed: Speed) -> Self;
}
/// Marks a type as a TXD3 pin
pub trait Txd3 {
    fn set_speed(self, speed: Speed) -> Self;
}

use crate::gpio::{self, Alternate};

macro_rules! impl_set_speed {
    ($TRAIT:ty, [$($PIN:ty),*]) => {
        $(
            impl $TRAIT for $PIN {
                fn set_speed(self, speed: Speed) -> Self {
                    self.set_speed(speed)
                }
            }
        )*
    };
}

impl_set_speed!(RefClk, [gpio::PA1<Alternate<11>>]);
impl_set_speed!(TxClk, [gpio::PC3<Alternate<11>>]);
impl_set_speed!(Mdio, [gpio::PA2<Alternate<11>>]);
impl_set_speed!(Mdc, [gpio::PC1<Alternate<11>>]);
impl_set_speed!(Col, [gpio::PA3<Alternate<11>>, gpio::PH3<Alternate<11>>]);
impl_set_speed!(Crs, [gpio::PA0<Alternate<11>>, gpio::PH2<Alternate<11>>]);
impl_set_speed!(CrsDv, [gpio::PA7<Alternate<11>>]);
impl_set_speed!(PpsOut, [gpio::PB5<Alternate<11>>, gpio::PG8<Alternate<11>>]);
impl_set_speed!(RxEr, [gpio::PB10<Alternate<11>>]);
#[cfg(not(feature = "rm0468"))]
impl_set_speed!(RxEr, [gpio::PI10<Alternate<11>>]);
impl_set_speed!(TxEn, [gpio::PB11<Alternate<11>>, gpio::PG11<Alternate<11>>]);
impl_set_speed!(Rxd0, [gpio::PC4<Alternate<11>>]);
impl_set_speed!(Rxd1, [gpio::PC5<Alternate<11>>]);
impl_set_speed!(Rxd2, [gpio::PB0<Alternate<11>>, gpio::PH6<Alternate<11>>]);
impl_set_speed!(Rxd3, [gpio::PB1<Alternate<11>>, gpio::PH7<Alternate<11>>]);
impl_set_speed!(Txd0, [gpio::PB12<Alternate<11>>, gpio::PG13<Alternate<11>>]);
impl_set_speed!(Txd1, [gpio::PB13<Alternate<11>>, gpio::PG12<Alternate<11>>, gpio::PG14<Alternate<11>>]);
impl_set_speed!(Txd2, [gpio::PC2<Alternate<11>>]);
impl_set_speed!(Txd3, [gpio::PB8<Alternate<11>>, gpio::PE2<Alternate<11>>]);
