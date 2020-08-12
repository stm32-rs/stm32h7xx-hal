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

mod eth;
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

pub use eth::{enable_interrupt, ethernet_init, interrupt_handler};
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

macro_rules! pins {
    (REF_CLK: [$($REF_CLK:ty),*] TX_CLK: [$($TX_CLK:ty),*]
     MDIO: [$($MDIO:ty),*] MDC: [$($MDC:ty),*] COL: [$($COL:ty),*]
     CRS: [$($CRS:ty),*] CRS_DV: [$($CRS_DV:ty),*] PPS_OUT: [$($PPS_OUT:ty),*]
     RX_ER: [$($RX_ER:ty),*] TX_EN: [$($TX_EN:ty),*]
     RXD0: [$($RXD0:ty),*] RXD1: [$($RXD1:ty),*] RXD2: [$($RXD2:ty),*] RXD3: [$($RXD3:ty),*]
     TXD0: [$($TXD0:ty),*] TXD1: [$($TXD1:ty),*] TXD2: [$($TXD2:ty),*] TXD3: [$($TXD3:ty),*]) => {
        $(
            impl RefClk for $REF_CLK {}
        )*
        $(
            impl TxClk for $TX_CLK {}
        )*
        $(
            impl Mdio for $MDIO {}
        )*
        $(
            impl Mdc for $MDC {}
        )*
        $(
            impl Col for $COL {}
        )*
        $(
            impl Crs for $CRS {}
        )*
        $(
            impl CrsDv for $CRS_DV {}
        )*
        $(
            impl PpsOut for $PPS_OUT {}
        )*
        $(
            impl RxEr for $RX_ER {}
        )*
        $(
            impl TxEn for $TX_EN {}
        )*
        $(
            impl Rxd0 for $RXD0 {}
        )*
        $(
            impl Rxd1 for $RXD1 {}
        )*
        $(
            impl Rxd2 for $RXD2 {}
        )*
        $(
            impl Rxd3 for $RXD3 {}
        )*
        $(
            impl Txd0 for $TXD0 {}
        )*
        $(
            impl Txd1 for $TXD1 {}
        )*
        $(
            impl Txd2 for $TXD2 {}
        )*
        $(
            impl Txd3 for $TXD3 {}
        )*
    }
}

use crate::gpio::gpioa::{PA0, PA1, PA2, PA3, PA7};
use crate::gpio::gpiob::{PB0, PB1, PB10, PB11, PB12, PB13, PB5, PB8};
use crate::gpio::gpioc::{PC1, PC2, PC3, PC4, PC5};
use crate::gpio::gpioe::PE2;
use crate::gpio::gpiog::{PG11, PG12, PG13, PG14, PG8};
use crate::gpio::gpioh::{PH2, PH3, PH6, PH7};
use crate::gpio::gpioi::PI10;
use crate::gpio::{Alternate, AF11};

pins! {
    REF_CLK: [
        PA1<Alternate<AF11>>
    ]
    TX_CLK: [
        PC3<Alternate<AF11>>
    ]
    MDIO: [
        PA2<Alternate<AF11>>
    ]
    MDC: [
        PC1<Alternate<AF11>>
    ]
    COL: [
        PA3<Alternate<AF11>>,
        PH3<Alternate<AF11>>
    ]
    CRS: [
        PA0<Alternate<AF11>>,
        PH2<Alternate<AF11>>
    ]
    CRS_DV: [
        PA7<Alternate<AF11>>
    ]
    PPS_OUT: [
        PB5<Alternate<AF11>>,
        PG8<Alternate<AF11>>
    ]
    RX_ER: [
        PB10<Alternate<AF11>>,
        PI10<Alternate<AF11>>
    ]
    TX_EN: [
        PB11<Alternate<AF11>>,
        PG11<Alternate<AF11>>
    ]
    RXD0: [
        PC4<Alternate<AF11>>
    ]
    RXD1: [
        PC5<Alternate<AF11>>
    ]
    RXD2: [
        PB0<Alternate<AF11>>,
        PH6<Alternate<AF11>>
    ]
    RXD3: [
        PB1<Alternate<AF11>>,
        PH7<Alternate<AF11>>
    ]
    TXD0: [
        PB12<Alternate<AF11>>,
        PG13<Alternate<AF11>>
    ]
    TXD1: [
        PB13<Alternate<AF11>>,
        PG12<Alternate<AF11>>,
        PG14<Alternate<AF11>>
    ]
    TXD2: [
        PC2<Alternate<AF11>>
    ]
    TXD3: [
        PB8<Alternate<AF11>>,
        PE2<Alternate<AF11>>
    ]
}
