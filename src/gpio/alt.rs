use super::{Alternate, NoPin, OpenDrain, PinMode};
use crate::gpio;

macro_rules! pin {
    ( $($(#[$docs:meta])* <$name:ident> for $(no: $NoPin:ty,)? [$(
            $(#[$attr:meta])* $PX:ident<$A:literal $(, $Otype:ident)?>,
        )*],)*) => {
        $(
            #[derive(Debug)]
            $(#[$docs])*
            pub enum $name {
                $(
                    None($NoPin),
                )?

                $(
                    $(#[$attr])*
                    $PX(gpio::$PX<Alternate<$A $(, $Otype)?>>),
                )*
            }

            impl crate::Sealed for $name { }

            #[allow(unreachable_patterns)]
            impl $name {
                pub fn is_high(&self) -> bool {
                    !self.is_low()
                }
                pub fn is_low(&self) -> bool {
                    match self {
                        $(
                            $(#[$attr])*
                            Self::$PX(p) => p.is_low(),
                        )*
                        _ => false,
                    }
                }
            }

            $(
                impl From<$NoPin> for $name {
                    fn from(p: $NoPin) -> Self {
                        Self::None(p)
                    }
                }

                #[allow(irrefutable_let_patterns)]
                impl TryFrom<$name> for $NoPin {
                    type Error = ();

                    fn try_from(a: $name) -> Result<Self, Self::Error> {
                        if let $name::None(p) = a {
                            Ok(p)
                        } else {
                            Err(())
                        }
                    }
                }
            )?

            $(
                $(#[$attr])*
                impl From<gpio::$PX> for $name {
                    fn from(p: gpio::$PX) -> Self {
                        Self::$PX(p.into_mode())
                    }
                }

                $(#[$attr])*
                impl From<gpio::$PX<Alternate<$A $(, $Otype)?>>> for $name {
                    fn from(p: gpio::$PX<Alternate<$A $(, $Otype)?>>) -> Self {
                        Self::$PX(p)
                    }
                }

                $(#[$attr])*
                #[allow(irrefutable_let_patterns)]
                impl<MODE: PinMode> TryFrom<$name> for gpio::$PX<MODE> {
                    type Error = ();

                    fn try_from(a: $name) -> Result<Self, Self::Error> {
                        if let $name::$PX(p) = a {
                            Ok(p.into_mode())
                        } else {
                            Err(())
                        }
                    }
                }
            )*
        )*
    };
}

pub mod comp1 {
    use super::*;
    pin! {
        <Out> for [
            PC5<13>,
            PE12<13>,
        ],
    }
}

pub mod comp2 {
    use super::*;
    pin! {
        <Out> for [
            PE13<13>,
            PE8<13>,
        ],
    }
}
/*
#[cfg(feature = "gpio-h747")]
pub mod comp {
    use super::*;
    pin! {
        <Tim1Bkin> for [
            PE15<13>,
        ],
    }
}
*/
pub mod crs {
    use super::*;
    pin! {
        <Sync> for [
            PB3<10>,
        ],
    }
}

#[cfg(any(feature = "gpio-h72", feature = "gpio-h747"))]
pub mod eth {
    use super::*;
    pin! {
        <Col> for [
            PA3<11>,
            PH3<11>,
        ],
        <Crs> for [
            PA0<11>,
            PH2<11>,
        ],
        <CrsDv> for [
            PA7<11>,
        ],
        <Mdc> for [
            PC1<11>,
        ],
        <Mdio> for [
            PA2<11>,
        ],
        <PpsOut> for [
            PB5<11>,
            PG8<11>,
        ],
        <RefClk> for [
            PA1<11>,
        ],
        <RxClk> for [
            PA1<11>,
        ],
        <RxDv> for [
            PA7<11>,
        ],
        <RxEr> for [
            PB10<11>,

            #[cfg(feature = "gpio-h747")]
            PI10<11>,
        ],
        <Rxd0> for [
            PC4<11>,
        ],
        <Rxd1> for [
            PC5<11>,
        ],
        <Rxd2> for [
            PB0<11>,
            PH6<11>,
        ],
        <Rxd3> for [
            PB1<11>,
            PH7<11>,
        ],
        <TxClk> for [
            #[cfg(feature = "gpio-h747")]
            PC3<11>,
        ],
        <TxEn> for [
            PB11<11>,
            PG11<11>,
        ],
        <TxEr> for [
            #[cfg(feature = "gpio-h72")]
            PA9<11>,
            #[cfg(feature = "gpio-h72")]
            PB2<11>,
        ],
        <Txd0> for [
            PB12<11>,
            PG13<11>,
        ],
        <Txd1> for [
            PB13<11>,
            PG12<11>,
            PG14<11>,
        ],
        <Txd2> for [
            #[cfg(feature = "gpio-h747")]
            PC2<11>,
        ],
        <Txd3> for [
            PB8<11>,
            PE2<11>,
        ],
    }
}

pub mod dcmi {
    use super::*;
    pin! {
        <D0> for [
            PA9<13>,
            PC6<13>,
            PH9<13>,
        ],
        <D1> for [
            PA10<13>,
            PC7<13>,
            PH10<13>,
        ],
        <D2> for [
            PC8<13>,
            PE0<13>,
            PG10<13>,
            PH11<13>,

            #[cfg(any(feature = "gpio-h72", feature = "gpio-h7a2"))]
            PB13<13>,
        ],
        <D3> for [
            PC9<13>,
            PE1<13>,
            PG11<13>,
            PH12<13>,
        ],
        <D4> for [
            PC11<13>,
            PE4<13>,
            PH14<13>,
        ],
        <D5> for [
            PB6<13>,
            PD3<13>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI4<13>,
        ],
        <D6> for [
            PB8<13>,
            PE5<13>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI6<13>,
        ],
        <D7> for [
            PB9<13>,
            PE6<13>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI7<13>,
        ],
        <D8> for [
            PC10<13>,
            PH6<13>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI1<13>,
        ],
        <D9> for [
            PC12<13>,
            PH7<13>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI2<13>,
        ],
        <D10> for [
            PB5<13>,
            PD6<13>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI3<13>,
        ],
        <D11> for [
            PD2<13>,
            PF10<13>,
            PH15<13>,
        ],
        <D12> for [
            PF11<13>,
            PG6<13>,

            #[cfg(any(feature = "gpio-h72", feature = "gpio-h7a2"))]
            PD12<13>,
        ],
        <D13> for [
            PG15<13>,
            PG7<13>,

            #[cfg(any(feature = "gpio-h72", feature = "gpio-h7a2"))]
            PD13<13>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI0<13>,
        ],
        <Hsync> for [
            PA4<13>,
            PH8<13>,
        ],
        <Pixclk> for [
            PA6<13>,
        ],
        <Vsync> for [
            PB7<13>,
            PG9<13>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI5<13>,
        ],
    }
}

pub mod debug {
    use super::*;
    pin! {
        <JtckSwclk> for [
            PA14<0>,
        ],
        <Jtdi> for [
            PA15<0>,
        ],
        <JtdoSwo> for [
            PB3<0>,
        ],
        <JtmsSwdio> for [
            PA13<0>,
        ],
        <Traceclk> for [
            PE2<0>,
        ],
        <Traced0> for [
            PC1<0>,
            PE3<0>,
            PG13<0>,
        ],
        <Traced1> for [
            PC8<0>,
            PE4<0>,
            PG14<0>,
        ],
        <Traced2> for [
            PD2<0>,
            PE5<0>,
        ],
        <Traced3> for [
            PC12<0>,
            PE6<0>,
        ],
        <Trgio> for [
            PC7<0>,
        ],
        <Jtrst> for [
            #[cfg(feature = "gpio-h72")]
            PB4<0>,
        ],
        <Trgin> for [
            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PJ7<0>,
        ],
        <Trgout> for [
            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PJ12<0>,
        ],
    }
}

pub mod dfsdm1 {
    use super::*;
    pin! {
        <Ckin0> for [
            PC0<3>,
        ],
        <Ckin1> for [
            PB13<6>,
            PB2<4>,
            PD7<6>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PC2<3>,
        ],
        <Ckin2> for [
            PB15<6>,
            PC4<3>,
            PE8<3>,
        ],
        <Ckin3> for [
            PC6<4>,
            PD8<3>,
            PE5<3>,
        ],
        <Ckin4> for [
            PC1<4>,
            PD6<3>,
            PE11<3>,
        ],
        <Ckin5> for [
            PB7<11>,
            PC10<3>,
            PE13<3>,
        ],
        <Ckin6> for [
            PD0<3>,
            PF14<3>,
        ],
        <Ckin7> for [
            PB11<6>,
            PB8<3>,
        ],
        <Ckout> for [
            PB0<6>,
            PD10<3>,
            PD3<3>,
            PE9<3>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PC2<6>,
        ],
        <Datin0> for [
            PC1<3>,
        ],
        <Datin1> for [
            PB12<6>,
            PB1<6>,
            PD6<4>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PC3<3>,
        ],
        <Datin2> for [
            PB14<6>,
            PC5<3>,
            PE7<3>,
        ],
        <Datin3> for [
            PC7<4>,
            PD9<3>,
            PE4<3>,
        ],
        <Datin4> for [
            PC0<6>,
            PD7<3>,
            PE10<3>,
        ],
        <Datin5> for [
            PB6<11>,
            PC11<3>,
            PE12<3>,
        ],
        <Datin6> for [
            PD1<3>,
            PF13<3>,
        ],
        <Datin7> for [
            PB10<6>,
            PB9<3>,
        ],
    }
}

#[cfg(feature = "gpio-h7a2")]
pub mod dfsdm2 {
    use super::*;
    pin! {
        <Ckin0> for [
            PC10<4>,
        ],
        <Ckin1> for [
            PA2<6>,
            PB13<4>,
        ],
        <Ckout> for [
            PB0<4>,
            PC12<4>,
            PD10<4>,
        ],
        <Datin0> for [
            PC11<4>,
        ],
        <Datin1> for [
            PA7<4>,
            PB12<11>,
        ],
    }
}

#[cfg(feature = "gpio-h747")]
pub mod dsihost {
    use super::*;
    pin! {
        <Te> for [
            PA15<13>,
            PB11<13>,
            PJ2<13>,
        ],
    }
}

pub mod fdcan1 {
    use super::*;
    pin! {
        <Rx> for [
            PA11<9>,
            PB8<9>,
            PD0<9>,
            PH14<9>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI9<9>,
        ],
        <Tx> for [
            PA12<9>,
            PB9<9>,
            PD1<9>,
            PH13<9>,
        ],
    }
}

pub mod fdcan2 {
    use super::*;
    pin! {
        <Rx> for [
            PB12<9>,
            PB5<9>,
        ],
        <Tx> for [
            PB13<9>,
            PB6<9>,
        ],
    }
}

#[cfg(feature = "gpio-h72")]
pub mod fdcan3 {
    use super::*;
    pin! {
        <Rx> for [
            PD12<5>,
            PF6<2>,
            PG10<2>,
        ],
        <Tx> for [
            PD13<5>,
            PF7<2>,
            PG9<2>,
        ],
    }
}

pub mod fmc {
    use super::*;
    pin! {
        <A0> for [
            PF0<12>,
        ],
        <A1> for [
            PF1<12>,
        ],
        <A2> for [
            PF2<12>,
        ],
        <A3> for [
            PF3<12>,
        ],
        <A4> for [
            PF4<12>,
        ],
        <A5> for [
            PF5<12>,
        ],
        <A6> for [
            PF12<12>,
        ],
        <A7> for [
            PF13<12>,
        ],
        <A8> for [
            PF14<12>,
        ],
        <A9> for [
            PF15<12>,
        ],
        <A10> for [
            PG0<12>,
        ],
        <A11> for [
            PG1<12>,
        ],
        <A12> for [
            PG2<12>,
        ],
        <A13> for [
            PG3<12>,
        ],
        <A14> for [
            PG4<12>,
        ],
        <A15> for [
            PG5<12>,
        ],
        <A16> for [
            PD11<12>,
        ],
        <A17> for [
            PD12<12>,
        ],
        <A18> for [
            PD13<12>,
        ],
        <A19> for [
            PE3<12>,

            #[cfg(feature = "gpio-h72")]
            PA0<12>,
        ],
        <A20> for [
            PE4<12>,
        ],
        <A21> for [
            PE5<12>,
        ],
        <A22> for [
            PE6<12>,

            #[cfg(feature = "gpio-h72")]
            PC4<1>,
        ],
        <A23> for [
            PE2<12>,
        ],
        <A24> for [
            PG13<12>,
        ],
        <A25> for [
            PG14<12>,

            #[cfg(any(feature = "gpio-h72", feature = "gpio-h7a2"))]
            PC0<9>,
        ],
        <Ba0> for [
            PG4<12>,
        ],
        <Ba1> for [
            PG5<12>,
        ],
        <Clk> for [
            PD3<12>,
        ],
        <D0> for [
            PD14<12>,
        ],
        <D1> for [
            PD15<12>,
        ],
        <D2> for [
            PD0<12>,
        ],
        <D3> for [
            PD1<12>,
        ],
        <D4> for [
            PE7<12>,
        ],
        <D5> for [
            PE8<12>,
        ],
        <D6> for [
            PE9<12>,

            #[cfg(feature = "gpio-h72")]
            PC12<1>,
        ],
        <D7> for [
            PE10<12>,

            #[cfg(feature = "gpio-h72")]
            PD2<1>,
        ],
        <D8> for [
            PE11<12>,

            #[cfg(feature = "gpio-h72")]
            PA4<12>,
        ],
        <D9> for [
            PE12<12>,

            #[cfg(feature = "gpio-h72")]
            PA5<12>,
        ],
        <D10> for [
            PE13<12>,

            #[cfg(feature = "gpio-h72")]
            PB14<12>,
        ],
        <D11> for [
            PE14<12>,

            #[cfg(feature = "gpio-h72")]
            PB15<12>,
        ],
        <D12> for [
            PE15<12>,

            #[cfg(feature = "gpio-h72")]
            PC0<1>,
        ],
        <D13> for [
            PD8<12>,
        ],
        <D14> for [
            PD9<12>,
        ],
        <D15> for [
            PD10<12>,
        ],
        <D16> for [
            PH8<12>,
        ],
        <D17> for [
            PH9<12>,
        ],
        <D18> for [
            PH10<12>,
        ],
        <D19> for [
            PH11<12>,
        ],
        <D20> for [
            PH12<12>,
        ],
        <D21> for [
            PH13<12>,
        ],
        <D22> for [
            PH14<12>,
        ],
        <D23> for [
            PH15<12>,
        ],
        <D24> for [
            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI0<12>,
        ],
        <D25> for [
            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI1<12>,
        ],
        <D26> for [
            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI2<12>,
        ],
        <D27> for [
            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI3<12>,
        ],
        <D28> for [
            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI6<12>,
        ],
        <D29> for [
            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI7<12>,
        ],
        <D30> for [
            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI9<12>,
        ],
        <D31> for [
            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI10<12>,
        ],
        <Da0> for [
            PD14<12>,
        ],
        <Da1> for [
            PD15<12>,
        ],
        <Da2> for [
            PD0<12>,
        ],
        <Da3> for [
            PD1<12>,
        ],
        <Da4> for [
            PE7<12>,
        ],
        <Da5> for [
            PE8<12>,
        ],
        <Da6> for [
            PE9<12>,

            #[cfg(feature = "gpio-h72")]
            PC12<1>,
        ],
        <Da7> for [
            PE10<12>,

            #[cfg(feature = "gpio-h72")]
            PD2<1>,
        ],
        <Da8> for [
            PE11<12>,

            #[cfg(feature = "gpio-h72")]
            PA4<12>,
        ],
        <Da9> for [
            PE12<12>,

            #[cfg(feature = "gpio-h72")]
            PA5<12>,
        ],
        <Da10> for [
            PE13<12>,

            #[cfg(feature = "gpio-h72")]
            PB14<12>,
        ],
        <Da11> for [
            PE14<12>,

            #[cfg(feature = "gpio-h72")]
            PB15<12>,
        ],
        <Da12> for [
            PE15<12>,

            #[cfg(feature = "gpio-h72")]
            PC0<1>,
        ],
        <Da13> for [
            PD8<12>,
        ],
        <Da14> for [
            PD9<12>,
        ],
        <Da15> for [
            PD10<12>,
        ],
        <Int> for [
            PG7<12>,

            #[cfg(any(feature = "gpio-h72", feature = "gpio-h7a2"))]
            PC8<10>,
        ],
        <Nbl0> for [
            PE0<12>,
        ],
        <Nbl1> for [
            PE1<12>,
        ],
        <Nbl2> for [
            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI4<12>,
        ],
        <Nbl3> for [
            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI5<12>,
        ],
        <Nce> for [
            PC8<9>,
            PG9<12>,
        ],
        <Ne1> for [
            PC7<9>,
            PD7<12>,
        ],
        <Ne2> for [
            PC8<9>,
            PG9<12>,
        ],
        <Ne3> for [
            PG10<12>,
            PG6<12>,
        ],
        <Ne4> for [
            PG12<12>,
        ],
        <Nl> for [
            PB7<12>,
        ],
        <Noe> for [
            PD4<12>,
        ],
        <Nwait> for [
            PC6<9>,
            PD6<12>,
        ],
        <Nwe> for [
            PD5<12>,
        ],
        <Sdcke0> for [
            PC5<12>,
            PH2<12>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PC3<12>,
        ],
        <Sdcke1> for [
            PB5<12>,
            PH7<12>,
        ],
        <Sdclk> for [
            PG8<12>,
        ],
        <Sdncas> for [
            PG15<12>,
        ],
        <Sdne0> for [
            PC4<12>,
            PH3<12>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PC2<12>,
        ],
        <Sdne1> for [
            PB6<12>,
            PH6<12>,
        ],
        <Sdnras> for [
            PF11<12>,
        ],
        <Sdnwe> for [
            PA7<12>,
            PC0<12>,
            PH5<12>,
        ],
        <Ale> for [
            #[cfg(any(feature = "gpio-h72", feature = "gpio-h7a2"))]
            PD12<12>,
        ],
        <Cle> for [
            #[cfg(any(feature = "gpio-h72", feature = "gpio-h7a2"))]
            PD11<12>,
        ],
    }
}

pub mod i2c1 {
    use super::*;
    pin! {
        <Scl> for [
            PB6<4, OpenDrain>,
            PB8<4, OpenDrain>,
        ],
        <Sda> for [
            PB7<4, OpenDrain>,
            PB9<4, OpenDrain>,
        ],
        <Smba> for [
            PB5<4, OpenDrain>,
        ],
    }
}

pub mod i2c2 {
    use super::*;
    pin! {
        <Scl> for [
            PB10<4, OpenDrain>,
            PF1<4, OpenDrain>,
            PH4<4, OpenDrain>,
        ],
        <Sda> for [
            PB11<4, OpenDrain>,
            PF0<4, OpenDrain>,
            PH5<4, OpenDrain>,
        ],
        <Smba> for [
            PB12<4, OpenDrain>,
            PF2<4, OpenDrain>,
            PH6<4, OpenDrain>,
        ],
    }
}

pub mod i2c3 {
    use super::*;
    pin! {
        <Scl> for [
            PA8<4, OpenDrain>,
            PH7<4, OpenDrain>,
        ],
        <Sda> for [
            PC9<4, OpenDrain>,
            PH8<4, OpenDrain>,
        ],
        <Smba> for [
            PA9<4, OpenDrain>,
            PH9<4, OpenDrain>,
        ],
    }
}

pub mod i2c4 {
    use super::*;
    pin! {
        <Scl> for [
            PB6<6, OpenDrain>,
            PB8<6, OpenDrain>,
            PD12<4, OpenDrain>,
            PF14<4, OpenDrain>,
            PH11<4, OpenDrain>,
        ],
        <Sda> for [
            PB7<6, OpenDrain>,
            PB9<6, OpenDrain>,
            PD13<4, OpenDrain>,
            PF15<4, OpenDrain>,
            PH12<4, OpenDrain>,
        ],
        <Smba> for [
            PB5<6, OpenDrain>,
            PB9<11, OpenDrain>,
            PD11<4, OpenDrain>,
            PF13<4, OpenDrain>,
            PH10<4, OpenDrain>,
        ],
    }
}

#[cfg(feature = "gpio-h72")]
pub mod i2c5 {
    use super::*;
    pin! {
        <Scl> for [
            PA8<6, OpenDrain>,
            PC11<4, OpenDrain>,
            PF1<6, OpenDrain>,
        ],
        <Sda> for [
            PC10<4, OpenDrain>,
            PC9<6, OpenDrain>,
            PF0<6, OpenDrain>,
        ],
        <Smba> for [
            PA9<6, OpenDrain>,
            PC12<4, OpenDrain>,
            PF2<6, OpenDrain>,
        ],
    }
}

pub mod i2s1 {
    use super::*;
    pin! {
        <Ck> for [
            PA5<5>,
            PB3<5>,
            PG11<5>,
        ],
        <Mck> for [
            PC4<5>,
        ],
        <Sdi> for [
            PA6<5>,
            PB4<5>,
            PG9<5>,
        ],
        <Sdo> for [
            PA7<5>,
            PB5<5>,
            PD7<5>,
        ],
        <Ws> for [
            PA15<5>,
            PA4<5>,
            PG10<5>,
        ],
    }
}

pub mod i2s2 {
    use super::*;
    pin! {
        <Ck> for [
            PA12<5>,
            PA9<5>,
            PB10<5>,
            PB13<5>,
            PD3<5>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI1<5>,
        ],
        <Mck> for [
            PC6<5>,
        ],
        <Sdi> for [
            PB14<5>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PC2<5>,
            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI2<5>,
        ],
        <Sdo> for [
            PB15<5>,
            PC1<5>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PC3<5>,
            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI3<5>,
        ],
        <Ws> for [
            PA11<5>,
            PB12<5>,
            PB4<7>,
            PB9<5>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI0<5>,
        ],
    }
}

pub mod i2s3 {
    use super::*;
    pin! {
        <Ck> for [
            PB3<6>,
            PC10<6>,
        ],
        <Mck> for [
            PC7<6>,
        ],
        <Sdi> for [
            PB4<6>,
            PC11<6>,
        ],
        <Sdo> for [
            PB2<7>,
            PB5<7>,
            PC12<6>,
            PD6<5>,
        ],
        <Ws> for [
            PA15<6>,
            PA4<6>,
        ],
    }
}

#[cfg(any(feature = "gpio-h72", feature = "gpio-h7a2"))]
pub mod i2s6 {
    use super::*;
    pin! {
        <Ck> for [
            PA5<8>,
            PB3<8>,
            PC12<5>,
            PG13<5>,
        ],
        <Mck> for [
            PA3<5>,
        ],
        <Sdi> for [
            PA6<8>,
            PB4<8>,
            PG12<5>,
        ],
        <Sdo> for [
            PA7<8>,
            PB5<8>,
            PG14<5>,
        ],
        <Ws> for [
            PA0<5>,
            PA15<7>,
            PA4<8>,
            PG8<5>,
        ],
    }
}
/*
pub mod i2s {
    use super::*;
    pin! {
        <Ckin> for [
            PC9<5>,
        ],
    }
}
*/
pub mod lptim1 {
    use super::*;
    pin! {
        <Etr> for [
            PE0<1>,
            PG14<1>,
        ],
        <In1> for [
            PD12<1>,
            PG12<1>,
        ],
        <In2> for [
            PE1<1>,
            PG11<1>,
            PH2<1>,
        ],
        <Out> for [
            PD13<1>,
            PG13<1>,
        ],
    }
}

pub mod lptim2 {
    use super::*;
    pin! {
        <Etr> for [
            PB11<3>,
            PE0<4>,
        ],
        <In1> for [
            PB10<3>,
            PD12<3>,
        ],
        <In2> for [
            PD11<3>,
        ],
        <Out> for [
            PB13<3>,
        ],
    }
}

pub mod lptim3 {
    use super::*;
    pin! {
        <Out> for [
            PA1<3>,
        ],
    }
}

pub mod lpuart1 {
    use super::*;
    pin! {
        <Cts> for [
            PA11<3>,
        ],
        <De> for [
            PA12<3>,
        ],
        <Rts> for [
            PA12<3>,
        ],
        <Rx> for [
            PA10<3>,
            PB7<8>,
        ],
        <Tx> for [
            PA9<3>,
            PB6<8>,
        ],
    }
}

pub mod ltdc {
    use super::*;
    pin! {
        <B0> for [
            PE4<14>,
            PG14<14>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PJ12<14>,
        ],
        <B1> for [
            PA10<14>,
            PG12<14>,

            #[cfg(any(feature = "gpio-h72", feature = "gpio-h7a2"))]
            PC10<10>,
            #[cfg(any(feature = "gpio-h72", feature = "gpio-h7a2"))]
            PD0<14>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PJ13<14>,
        ],
        <B2> for [
            PA3<9>,
            PC9<14>,
            PD6<14>,
            PG10<14>,

            #[cfg(any(feature = "gpio-h72", feature = "gpio-h7a2"))]
            PD2<14>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PJ14<14>,
        ],
        <B3> for [
            PA8<13>,
            PD10<14>,
            PG11<14>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PJ15<14>,
        ],
        <B4> for [
            PA10<12>,
            PE12<14>,
            PG12<9>,

            #[cfg(any(feature = "gpio-h72", feature = "gpio-h7a2"))]
            PC11<14>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI4<14>,
            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PJ13<9>,
            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PK3<14>,
        ],
        <B5> for [
            PA3<14>,

            #[cfg(feature = "gpio-h72")]
            PB5<3>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI5<14>,
            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PK4<14>,

            #[cfg(feature = "gpio-h7a2")]
            PB5<11>,
        ],
        <B6> for [
            PB8<14>,

            #[cfg(any(feature = "gpio-h72", feature = "gpio-h7a2"))]
            PA15<14>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI6<14>,
            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PK5<14>,
        ],
        <B7> for [
            PB9<14>,

            #[cfg(any(feature = "gpio-h72", feature = "gpio-h7a2"))]
            PD2<9>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI7<14>,
            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PK6<14>,
        ],
        <Clk> for [
            PE14<14>,
            PG7<14>,

            #[cfg(any(feature = "gpio-h72", feature = "gpio-h7a2"))]
            PB14<14>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI14<14>,
        ],
        <De> for [
            PE13<14>,
            PF10<14>,

            #[cfg(any(feature = "gpio-h72", feature = "gpio-h7a2"))]
            PC5<14>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PK7<14>,
        ],
        <G0> for [
            PB1<14>,
            PE5<14>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PJ7<14>,
        ],
        <G1> for [
            PB0<14>,
            PE6<14>,
            PJ8<14>,
        ],
        <G2> for [
            PA6<14>,
            PH13<14>,
            PJ9<14>,

            #[cfg(any(feature = "gpio-h72", feature = "gpio-h7a2"))]
            PC0<11>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI15<9>,
        ],
        <G3> for [
            PC9<10>,
            PE11<14>,
            PG10<9>,
            PH14<14>,
            PJ10<14>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PJ12<9>,
        ],
        <G4> for [
            PB10<14>,
            PH15<14>,
            PH4<14>,
            PJ11<14>,
        ],
        <G5> for [
            PB11<14>,
            PH4<9>,
            PK0<14>,

            #[cfg(any(feature = "gpio-h72", feature = "gpio-h7a2"))]
            PC1<14>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI0<14>,
        ],
        <G6> for [
            PC7<14>,
            PK1<14>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI11<9>,
            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI1<14>,
        ],
        <G7> for [
            PD3<14>,
            PG8<14>,
            PK2<14>,

            #[cfg(any(feature = "gpio-h72", feature = "gpio-h7a2"))]
            PB15<14>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI2<14>,
        ],
        <Hsync> for [
            PC6<14>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI10<14>,
            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI12<14>,
        ],
        <R0> for [
            PG13<14>,
            PH2<14>,

            #[cfg(any(feature = "gpio-h72", feature = "gpio-h7a2"))]
            PE0<14>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI15<14>,
        ],
        <R1> for [
            PA2<14>,
            PH3<14>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PJ0<14>,
        ],
        <R2> for [
            PA1<14>,
            PC10<14>,
            PH8<14>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PJ1<14>,
        ],
        <R3> for [
            PB0<9>,
            PH9<14>,

            #[cfg(any(feature = "gpio-h72", feature = "gpio-h7a2"))]
            PA15<9>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PJ2<14>,
        ],
        <R4> for [
            PA11<14>,
            PA5<14>,
            PH10<14>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PJ3<14>,
        ],
        <R5> for [
            PA12<14>,
            PA9<14>,
            PC0<14>,
            PH11<14>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PJ4<14>,
        ],
        <R6> for [
            PA8<14>,
            PB1<9>,
            PH12<14>,

            #[cfg(any(feature = "gpio-h72", feature = "gpio-h7a2"))]
            PC12<14>,
            #[cfg(any(feature = "gpio-h72", feature = "gpio-h7a2"))]
            PE1<14>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PJ5<14>,
        ],
        <R7> for [
            PE15<14>,
            PG6<14>,

            #[cfg(any(feature = "gpio-h72", feature = "gpio-h7a2"))]
            PC4<14>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PJ0<9>,
            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PJ6<14>,
        ],
        <Vsync> for [
            PA4<14>,

            #[cfg(any(feature = "gpio-h72", feature = "gpio-h7a2"))]
            PA7<14>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI13<14>,
            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI9<14>,
        ],
    }
}

pub mod mdios {
    use super::*;
    pin! {
        <Mdc> for [
            PA6<11>,
            PC1<12>,
        ],
        <Mdio> for [
            PA10<11>,
            PA2<12>,
        ],
    }
}

#[cfg(any(feature = "gpio-h72", feature = "gpio-h7a2"))]
pub mod octospim {
    use super::*;
    pin! {
        <P1Clk> for [
            PB2<9>,
            PF10<9>,

            #[cfg(feature = "gpio-h72")]
            PA3<12>,

            #[cfg(feature = "gpio-h7a2")]
            PA3<3>,
        ],
        <P1Dqs> for [
            PB2<10>,
            PC5<10>,

            #[cfg(feature = "gpio-h72")]
            PA1<12>,

            #[cfg(feature = "gpio-h7a2")]
            PA1<11>,
        ],
        <P1Io0> for [
            PC9<9>,
            PD11<9>,
            PF8<10>,

            #[cfg(feature = "gpio-h72")]
            PA2<6>,
            #[cfg(feature = "gpio-h72")]
            PB12<12>,
            #[cfg(feature = "gpio-h72")]
            PB1<4>,

            #[cfg(feature = "gpio-h7a2")]
            PB1<11>,
            #[cfg(feature = "gpio-h7a2")]
            PC3<9>,
        ],
        <P1Io1> for [
            PC10<9>,
            PD12<9>,
            PF9<10>,

            #[cfg(feature = "gpio-h72")]
            PB0<4>,

            #[cfg(feature = "gpio-h7a2")]
            PB0<11>,
        ],
        <P1Io2> for [
            PA7<10>,
            PE2<9>,
            PF7<10>,

            #[cfg(feature = "gpio-h72")]
            PA3<6>,
            #[cfg(feature = "gpio-h72")]
            PB13<4>,

            #[cfg(feature = "gpio-h7a2")]
            PC2<9>,
        ],
        <P1Io3> for [
            PA1<9>,
            PA6<6>,
            PD13<9>,
            PF6<10>,
        ],
        <P1Io4> for [
            PC1<10>,
            PD4<10>,
            PE7<10>,
            PH2<9>,
        ],
        <P1Io5> for [
            PD5<10>,
            PE8<10>,
            PH3<9>,
        ],
        <P1Io6> for [
            PD6<10>,
            PE9<10>,
            PG9<9>,
        ],
        <P1Io7> for [
            PD7<10>,
            PE10<10>,
            PG14<9>,
        ],
        <P1Nclk> for [
            PB12<3>,
            PF11<9>,
        ],
        <P1Ncs> for [
            PB10<9>,
            PB6<10>,
            PC11<9>,
            PE11<11>,
            PG6<10>,
        ],
        <P2Clk> for [
            PF4<9>,

            #[cfg(feature = "gpio-h7a2")]
            PI13<3>,
        ],
        <P2Dqs> for [
            PF12<9>,
            PG15<9>,
            PG7<9>,

            #[cfg(feature = "gpio-h7a2")]
            PK6<3>,
        ],
        <P2Io0> for [
            PF0<9>,
            #[cfg(feature = "gpio-h7a2")]
            PI9<3>,
        ],
        <P2Io1> for [
            PF1<9>,

            #[cfg(feature = "gpio-h7a2")]
            PI10<3>,
        ],
        <P2Io2> for [
            PF2<9>,

            #[cfg(feature = "gpio-h7a2")]
            PI11<3>,
        ],
        <P2Io3> for [
            PF3<9>,

            #[cfg(feature = "gpio-h7a2")]
            PI12<3>,
        ],
        <P2Io4> for [
            PG0<9>,

            #[cfg(feature = "gpio-h7a2")]
            PJ1<3>,
        ],
        <P2Io5> for [
            PG1<9>,

            #[cfg(feature = "gpio-h7a2")]
            PC2<11>,

            #[cfg(feature = "gpio-h7a2")]
            PJ2<3>,
        ],
        <P2Io6> for [
            PG10<3>,

            #[cfg(feature = "gpio-h7a2")]
            PC3<11>,

            #[cfg(feature = "gpio-h7a2")]
            PK3<3>,
        ],
        <P2Io7> for [
            PG11<9>,

            #[cfg(feature = "gpio-h7a2")]
            PK4<3>,
        ],
        <P2Nclk> for [
            PF5<9>,

            #[cfg(feature = "gpio-h7a2")]
            PI14<3>,
        ],
        <P2Ncs> for [
            PG12<3>,

            #[cfg(feature = "gpio-h7a2")]
            PK5<3>,
        ],
    }
}

#[cfg(feature = "gpio-h747")]
pub mod quadspi {
    use super::*;
    pin! {
        <Bk1Io0> for [
            PC9<9>,
            PD11<9>,
            PF8<10>,
        ],
        <Bk1Io1> for no:NoPin, [
            PC10<9>,
            PD12<9>,
            PF9<10>,
        ],
        <Bk1Io2> for no:NoPin, [
            PE2<9>,
            PF7<9>,
        ],
        <Bk1Io3> for no:NoPin, [
            PA1<9>,
            PD13<9>,
            PF6<9>,
        ],
        <Bk1Ncs> for [
            PB10<9>,
            PB6<10>,
            PG6<10>,
        ],
        <Bk2Io0> for [
            PE7<10>,
            PH2<9>,
        ],
        <Bk2Io1> for no:NoPin, [
            PE8<10>,
            PH3<9>,
        ],
        <Bk2Io2> for no:NoPin, [
            PE9<10>,
            PG9<9>,
        ],
        <Bk2Io3> for no:NoPin, [
            PE10<10>,
            PG14<9>,
        ],
        <Bk2Ncs> for [
            PC11<9>,
        ],
        <Clk> for [
            PB2<9>,
            PF10<9>,
        ],
    }
}

pub mod rcc {
    use super::*;
    pin! {
        <Mco1> for [
            PA8<0>,
        ],
        <Mco2> for [
            PC9<0>,
        ],
    }
}

pub mod rtc {
    use super::*;
    pin! {
        <OutAlarm> for [
            PB2<0>,
        ],
        <Refin> for [
            PB15<0>,
        ],
        <OutCalib> for [
            #[cfg(any(feature = "gpio-h72", feature = "gpio-h747"))]
            PB2<0>,
        ],
    }
}

#[cfg(any(feature = "gpio-h72", feature = "gpio-h7a2"))]
pub mod pssi {
    use super::*;
    pin! {
        <D0> for [
            PA9<13>,
            PC6<13>,
            PH9<13>,
        ],
        <D1> for [
            PA10<13>,
            PC7<13>,
            PH10<13>,
        ],
        <D2> for [
            PB13<13>,
            PC8<13>,
            PE0<13>,
            PG10<13>,
            PH11<13>,
        ],
        <D3> for [
            PC9<13>,
            PE1<13>,
            PG11<13>,
            PH12<13>,
        ],
        <D4> for [
            PC11<13>,
            PE4<13>,
            PH14<13>,
        ],
        <D5> for [
            PB6<13>,
            PD3<13>,

            #[cfg(feature = "gpio-h7a2")]
            PI4<13>,
        ],
        <D6> for [
            PB8<13>,
            PE5<13>,

            #[cfg(feature = "gpio-h7a2")]
            PI6<13>,
        ],
        <D7> for [
            PB9<13>,
            PE6<13>,

            #[cfg(feature = "gpio-h7a2")]
            PI7<13>,
        ],
        <D8> for [
            PC10<13>,
            PH6<13>,

            #[cfg(feature = "gpio-h7a2")]
            PI1<13>,
        ],
        <D9> for [
            PC12<13>,
            PH7<13>,

            #[cfg(feature = "gpio-h7a2")]
            PI2<13>,
        ],
        <D10> for [
            PB5<13>,
            PD6<13>,

            #[cfg(feature = "gpio-h7a2")]
            PI3<13>,
        ],
        <D11> for [
            PD2<13>,
            PF10<13>,
            PH15<13>,
        ],
        <D12> for [
            PD12<13>,
            PF11<13>,
            PG6<13>,
        ],
        <D13> for [
            PD13<13>,
            PG15<13>,
            PG7<13>,

            #[cfg(feature = "gpio-h7a2")]
            PI0<13>,
        ],
        <D14> for [
            PA5<13>,
            PH4<13>,

            #[cfg(feature = "gpio-h7a2")]
            PI10<13>,
        ],
        <D15> for [
            PC5<4>,
            PF10<4>,

            #[cfg(feature = "gpio-h7a2")]
            PI11<13>,
        ],
        <De> for [
            PA4<13>,
            PH8<13>,
        ],
        <Pdck> for [
            PA6<13>,
        ],
        <Rdy> for [
            PB7<13>,
            PG9<13>,

            #[cfg(feature = "gpio-h7a2")]
            PI5<13>,
        ],
    }
}

#[cfg(feature = "gpio-h7a2")]
pub mod pwr {
    use super::*;
    pin! {
        <Csleep> for [
            PC3<0>,
        ],
        <Cstop> for [
            PC2<0>,
        ],
        <Ndstop2> for [
            PA5<0>,
        ],
    }
}

pub mod sai1 {
    use super::*;
    pin! {
        <Ck1> for [
            PE2<2>,
        ],
        <Ck2> for [
            PE5<2>,
        ],
        <D1> for [
            PB2<2>,
            PC1<2>,
            PD6<2>,
            PE6<2>,
        ],
        <D2> for [
            PE4<2>,
        ],
        <D3> for [
            PC5<2>,
            PF10<2>,
        ],
        <FsA> for [
            PE4<6>,
        ],
        <FsB> for [
            PF9<6>,
        ],
        <MclkA> for [
            PE2<6>,
            PG7<6>,
        ],
        <MclkB> for [
            PF7<6>,
        ],
        <SckA> for [
            PE5<6>,
        ],
        <SckB> for [
            PF8<6>,
        ],
        <SdA> for [
            PB2<6>,
            PC1<6>,
            PD6<6>,
            PE6<6>,
        ],
        <SdB> for [
            PE3<6>,
            PF6<6>,
        ],
    }
}

#[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
pub mod sai2 {
    use super::*;
    pin! {
        <FsA> for [
            PD12<10>,
            PI7<10>,
        ],
        <FsB> for [
            PA12<8>,
            PC0<8>,
            PE13<10>,
            PG9<10>,
        ],
        <MclkA> for [
            PE0<10>,
            PI4<10>,
        ],
        <MclkB> for [
            PA1<10>,
            PE14<10>,
            PE6<10>,
            PH3<10>,
        ],
        <SckA> for [
            PD13<10>,
            PI5<10>,
        ],
        <SckB> for [
            PA2<8>,
            PE12<10>,
            PH2<10>,
        ],
        <SdA> for [
            PD11<10>,
            PI6<10>,
        ],
        <SdB> for [
            PA0<10>,
            PE11<10>,
            PF11<10>,
            PG10<10>,
        ],
    }
}

#[cfg(feature = "gpio-h747")]
pub mod sai3 {
    use super::*;
    pin! {
        <FsA> for [
            PD4<6>,
        ],
        <FsB> for [
            PD10<6>,
        ],
        <MclkA> for [
            PD15<6>,
        ],
        <MclkB> for [
            PD14<6>,
        ],
        <SckA> for [
            PD0<6>,
        ],
        <SckB> for [
            PD8<6>,
        ],
        <SdA> for [
            PD1<6>,
        ],
        <SdB> for [
            PD9<6>,
        ],
    }
}

#[cfg(any(feature = "gpio-h72", feature = "gpio-h747"))]
pub mod sai4 {
    use super::*;
    pin! {
        <Ck1> for [
            PE2<10>,
        ],
        <Ck2> for [
            PE5<10>,
        ],
        <D1> for [
            PE6<9>,

            #[cfg(feature = "gpio-h72")]
            PB2<1>,
            #[cfg(feature = "gpio-h72")]
            PC1<1>,
            #[cfg(feature = "gpio-h72")]
            PD6<1>,

            #[cfg(feature = "gpio-h747")]
            PB2<10>,
            #[cfg(feature = "gpio-h747")]
            PC1<10>,
            #[cfg(feature = "gpio-h747")]
            PD6<10>,
        ],
        <D2> for [
            PE4<10>,
        ],
        <D3> for [
            PF10<10>,

            #[cfg(feature = "gpio-h72")]
            PC5<1>,

            #[cfg(feature = "gpio-h747")]
            PC5<10>,
        ],
        <FsA> for [
            PE4<8>,

            #[cfg(feature = "gpio-h72")]
            PD12<10>,
        ],
        <FsB> for [
            PF9<8>,

            #[cfg(feature = "gpio-h72")]
            PA12<8>,
            #[cfg(feature = "gpio-h72")]
            PC0<8>,
            #[cfg(feature = "gpio-h72")]
            PE13<10>,
            #[cfg(feature = "gpio-h72")]
            PG9<10>,
        ],
        <MclkA> for [
            PE2<8>,

            #[cfg(feature = "gpio-h72")]
            PE0<10>,
        ],
        <MclkB> for [
            PF7<8>,

            #[cfg(feature = "gpio-h72")]
            PA1<10>,
            #[cfg(feature = "gpio-h72")]
            PE14<10>,
            #[cfg(feature = "gpio-h72")]
            PE6<10>,
            #[cfg(feature = "gpio-h72")]
            PH3<10>,
        ],
        <SckA> for [
            PE5<8>,

            #[cfg(feature = "gpio-h72")]
            PD13<10>,
        ],
        <SckB> for [
            PF8<8>,

            #[cfg(feature = "gpio-h72")]
            PA2<8>,
            #[cfg(feature = "gpio-h72")]
            PE12<10>,
            #[cfg(feature = "gpio-h72")]
            PH2<10>,
        ],
        <SdA> for [
            PB2<8>,
            PC1<8>,
            PD6<8>,
            PE6<8>,

            #[cfg(feature = "gpio-h72")]
            PD11<10>,
        ],
        <SdB> for [
            PE3<8>,
            PF6<8>,

            #[cfg(feature = "gpio-h72")]
            PA0<10>,
            #[cfg(feature = "gpio-h72")]
            PE11<10>,
            #[cfg(feature = "gpio-h72")]
            PF11<10>,
            #[cfg(feature = "gpio-h72")]
            PG10<10>,
        ],
    }
}

pub mod sdmmc1 {
    use super::*;
    pin! {
        <Cdir> for [
            PB9<7>,
        ],
        <Ck> for [
            PC12<12>,
        ],
        <Ckin> for [
            PB8<7>,
        ],
        <Cmd> for [
            PD2<12>,
        ],
        <D0Dir> for [
            PC6<8>,
        ],
        <D0> for [
            PC8<12>,

            #[cfg(any(feature = "gpio-h72", feature = "gpio-h7a2"))]
            PB13<12>,
        ],
        <D123Dir> for [
            PC7<8>,
        ],
        <D1> for [
            PC9<12>,
        ],
        <D2> for [
            PC10<12>,
        ],
        <D3> for [
            PC11<12>,
        ],
        <D4> for [
            PB8<12>,
        ],
        <D5> for [
            PB9<12>,
        ],
        <D6> for [
            PC6<12>,
        ],
        <D7> for [
            PC7<12>,
        ],
    }
}

pub mod sdmmc2 {
    use super::*;
    pin! {
        <Ck> for [
            PC1<9>,
            PD6<11>,
        ],
        <Cmd> for [
            PA0<9>,
            PD7<11>,
        ],
        <D0> for [
            PB14<9>,

            #[cfg(any(feature = "gpio-h72", feature = "gpio-h7a2"))]
            PG9<11>,
        ],
        <D1> for [
            PB15<9>,

            #[cfg(any(feature = "gpio-h72", feature = "gpio-h7a2"))]
            PG10<11>,
        ],
        <D2> for [
            PB3<9>,
            PG11<10>,
        ],
        <D3> for [
            PB4<9>,

            #[cfg(any(feature = "gpio-h72", feature = "gpio-h7a2"))]
            PG12<10>,
        ],
        <D4> for [
            PB8<10>,
        ],
        <D5> for [
            PB9<10>,
        ],
        <D6> for [
            PC6<10>,

            #[cfg(any(feature = "gpio-h72", feature = "gpio-h7a2"))]
            PG13<10>,
        ],
        <D7> for [
            PC7<10>,

            #[cfg(any(feature = "gpio-h72", feature = "gpio-h7a2"))]
            PG14<10>,
        ],
        <Ckin> for [
            #[cfg(feature = "gpio-h72")]
            PC4<10>,
        ],
    }
}

#[cfg(any(feature = "gpio-h72", feature = "gpio-h747"))]
pub mod spdifrx1 {
    use super::*;
    pin! {
        <In0> for [
            #[cfg(feature = "gpio-h747")]
            PD7<9>,
            #[cfg(feature = "gpio-h747")]
            PG11<8>,
        ],
        <In1> for [
            #[cfg(feature = "gpio-h72")]
            PD7<9>,
            #[cfg(feature = "gpio-h72")]
            PG11<8>,

            #[cfg(feature = "gpio-h747")]
            PD8<9>,
            #[cfg(feature = "gpio-h747")]
            PG12<8>,
        ],
        <In2> for [
            #[cfg(feature = "gpio-h72")]
            PD8<9>,
            #[cfg(feature = "gpio-h72")]
            PG12<8>,

            #[cfg(feature = "gpio-h747")]
            PC4<9>,
            #[cfg(feature = "gpio-h747")]
            PG8<8>,
        ],
        <In3> for [
            #[cfg(feature = "gpio-h72")]
            PC4<9>,
            #[cfg(feature = "gpio-h72")]
            PG8<8>,

            #[cfg(feature = "gpio-h747")]
            PC5<9>,
            #[cfg(feature = "gpio-h747")]
            PG9<8>,
        ],
        <In4> for [
            #[cfg(feature = "gpio-h72")]
            PC5<9>,
            #[cfg(feature = "gpio-h72")]
            PG9<8>,
        ],
    }
}

#[cfg(feature = "gpio-h7a2")]
pub mod spdifrx {
    use super::*;
    pin! {
        <In1> for [
            PD7<9>,
            PG11<8>,
        ],
        <In2> for [
            PD8<9>,
            PG12<8>,
        ],
        <In3> for [
            PC4<9>,
            PG8<8>,
        ],
        <In4> for [
            PC5<9>,
            PG9<8>,
        ],
    }
}

pub mod spi1 {
    use super::*;
    pin! {
        <Sck> for no:NoPin, [
            PA5<5>,
            PB3<5>,
            PG11<5>,
        ],
        <Miso> for no:NoPin, [
            PA6<5>,
            PB4<5>,
            PG9<5>,
        ],
        <Mosi> for no:NoPin, [
            PA7<5>,
            PB5<5>,
            PD7<5>,
        ],
        <Nss> for [
            PA4<5>,
            PA15<5>,
            PG10<5>,
        ],
    }
}

pub mod spi2 {
    use super::*;
    pin! {
        <Sck> for no:NoPin, [
            PA9<5>,
            PA12<5>,
            PB10<5>,
            PB13<5>,
            PD3<5>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI1<5>,
        ],
        <Miso> for no:NoPin, [
            PB14<5>,

            // #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))] TODO: check
            PC2<5>,
            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI2<5>,
        ],
        <Mosi> for no:NoPin, [
            PB15<5>,
            PC1<5>,

            // #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))] TODO: check
            PC3<5>,
            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI3<5>,
        ],
        <Nss> for [
            PA11<5>,
            PB4<7>,
            PB12<5>,
            PB9<5>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI0<5>,
        ],
    }
}

pub mod spi3 {
    use super::*;
    pin! {
        <Sck> for no:NoPin, [
            PB3<6>,
            PC10<6>,
        ],
        <Miso> for no:NoPin, [
            PB4<6>,
            PC11<6>,
        ],
        <Mosi> for no:NoPin, [
            PB2<7>,
            PB5<7>,
            PC12<6>,
            PD6<5>,
        ],
        <Nss> for [
            PA15<6>,
            PA4<6>,
        ],
    }
}

pub mod spi4 {
    use super::*;
    pin! {
        <Sck> for no:NoPin, [
            PE12<5>,
            PE2<5>,
        ],
        <Miso> for no:NoPin, [
            PE13<5>,
            PE5<5>,
        ],
        <Mosi> for no:NoPin, [
            PE14<5>,
            PE6<5>,
        ],
        <Nss> for [
            PE11<5>,
            PE4<5>,
        ],
    }
}

pub mod spi5 {
    use super::*;
    pin! {
        <Sck> for no:NoPin, [
            PF7<5>,
            PH6<5>,
            PK0<5>,
        ],
        <Miso> for no:NoPin, [
            PF8<5>,
            PH7<5>,
            PJ11<5>,
        ],
        <Mosi> for no:NoPin, [
            PF11<5>,
            PF9<5>,
            PJ10<5>,
        ],
        <Nss> for [
            PF6<5>,
            PH5<5>,
            PK1<5>,
        ],
    }
}

pub mod spi6 {
    use super::*;
    pin! {
        <Sck> for no:NoPin, [
            PA5<8>,
            PB3<8>,
            PG13<5>,

            #[cfg(any(feature = "gpio-h72", feature = "gpio-h7a2"))]
            PC12<5>,
        ],
        <Miso> for no:NoPin, [
            PA6<8>,
            PB4<8>,
            PG12<5>,
        ],
        <Mosi> for no:NoPin, [
            PA7<8>,
            PB5<8>,
            PG14<5>,
        ],
        <Nss> for [
            PA15<7>,
            PA4<8>,
            PG8<5>,

            #[cfg(any(feature = "gpio-h72", feature = "gpio-h7a2"))]
            PA0<5>,
        ],
    }
}

pub mod swpmi1 {
    use super::*;
    pin! {
        <Rx> for [
            PC8<11>,

            #[cfg(any(feature = "gpio-h72", feature = "gpio-h7a2"))]
            PC10<11>,
        ],
        <Suspend> for [
            PC9<11>,
        ],
        <Tx> for [
            PC7<11>,
        ],
    }
}

#[cfg(feature = "gpio-h747")]
pub mod sys {
    use super::*;
    pin! {
        <Jtrst> for [
            PB4<0>,
        ],
    }
}

pub mod tim1 {
    use super::*;
    pin! {
        <Bkin2> for [
            PE6<1>,
            PG4<1>,

            #[cfg(feature = "gpio-h72")]
            PA12<12>,
        ],
        <Bkin2Comp1> for [
            PE6<11>,
            PG4<11>,
        ],
        <Bkin2Comp2> for [
            PE6<11>,
            PG4<11>,
        ],
        <Bkin> for [
            PA6<1>,
            PB12<1>,
            PE15<1>,
            PK2<1>,
        ],
        <BkinComp1> for [
            PA6<12>,
            PB12<13>,
            PE15<13>,
            PK2<11>,
        ],
        <BkinComp2> for [
            PA6<12>,
            PB12<13>,
            PE15<13>,
            PK2<11>,
        ],
        <Ch1> for [
            PA8<1>,
            PE9<1>,
            PK1<1>,
        ],
        <Ch1N> for [
            PA7<1>,
            PB13<1>,
            PE8<1>,
            PK0<1>,
        ],
        <Ch2> for [
            PA9<1>,
            PE11<1>,
            PJ11<1>,
        ],
        <Ch2N> for [
            PB0<1>,
            PB14<1>,
            PE10<1>,
            PJ10<1>,
        ],
        <Ch3> for [
            PA10<1>,
            PE13<1>,
            PJ9<1>,
        ],
        <Ch3N> for [
            PB15<1>,
            PB1<1>,
            PE12<1>,
            PJ8<1>,
        ],
        <Ch4> for [
            PA11<1>,
            PE14<1>,
        ],
        <Etr> for [
            PA12<1>,
            PE7<1>,
            PG5<1>,
        ],
    }
    use crate::pac::TIM1 as TIM;
    use crate::pwm::{PinCh, C1, C2, C3, C4};
    impl PinCh<C1> for TIM {
        type Ch = Ch1;
    }
    impl PinCh<C2> for TIM {
        type Ch = Ch2;
    }
    impl PinCh<C3> for TIM {
        type Ch = Ch3;
    }
    impl PinCh<C4> for TIM {
        type Ch = Ch4;
    }
}

pub mod tim2 {
    use super::*;
    pin! {
        <Ch1> for [
            PA0<1>,
            PA15<1>,
            PA5<1>,
        ],
        <Ch2> for [
            PA1<1>,
            PB3<1>,
        ],
        <Ch3> for [
            PA2<1>,
            PB10<1>,
        ],
        <Ch4> for [
            PA3<1>,
            PB11<1>,
        ],
        <Etr> for [
            PA0<1>,
            PA15<1>,
            PA5<1>,
        ],
    }
    use crate::pac::TIM2 as TIM;
    use crate::pwm::{PinCh, C1, C2, C3, C4};
    impl PinCh<C1> for TIM {
        type Ch = Ch1;
    }
    impl PinCh<C2> for TIM {
        type Ch = Ch2;
    }
    impl PinCh<C3> for TIM {
        type Ch = Ch3;
    }
    impl PinCh<C4> for TIM {
        type Ch = Ch4;
    }
}

pub mod tim3 {
    use super::*;
    pin! {
        <Ch1> for [
            PA6<2>,
            PB4<2>,
            PC6<2>,
        ],
        <Ch2> for [
            PA7<2>,
            PB5<2>,
            PC7<2>,
        ],
        <Ch3> for [
            PB0<2>,
            PC8<2>,
        ],
        <Ch4> for [
            PB1<2>,
            PC9<2>,
        ],
        <Etr> for [
            PD2<2>,
        ],
    }
    use crate::pac::TIM3 as TIM;
    use crate::pwm::{PinCh, C1, C2, C3, C4};
    impl PinCh<C1> for TIM {
        type Ch = Ch1;
    }
    impl PinCh<C2> for TIM {
        type Ch = Ch2;
    }
    impl PinCh<C3> for TIM {
        type Ch = Ch3;
    }
    impl PinCh<C4> for TIM {
        type Ch = Ch4;
    }
}

pub mod tim4 {
    use super::*;
    pin! {
        <Ch1> for [
            PB6<2>,
            PD12<2>,
        ],
        <Ch2> for [
            PB7<2>,
            PD13<2>,
        ],
        <Ch3> for [
            PB8<2>,
            PD14<2>,
        ],
        <Ch4> for [
            PB9<2>,
            PD15<2>,
        ],
        <Etr> for [
            PE0<2>,
        ],
    }
    use crate::pac::TIM4 as TIM;
    use crate::pwm::{PinCh, C1, C2, C3, C4};
    impl PinCh<C1> for TIM {
        type Ch = Ch1;
    }
    impl PinCh<C2> for TIM {
        type Ch = Ch2;
    }
    impl PinCh<C3> for TIM {
        type Ch = Ch3;
    }
    impl PinCh<C4> for TIM {
        type Ch = Ch4;
    }
}

pub mod tim5 {
    use super::*;
    pin! {
        <Ch1> for [
            PA0<2>,
            PH10<2>,
        ],
        <Ch2> for [
            PA1<2>,
            PH11<2>,
        ],
        <Ch3> for [
            PA2<2>,
            PH12<2>,
        ],
        <Ch4> for [
            PA3<2>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI0<2>,
        ],
        <Etr> for [
            PA4<2>,
            PH8<2>,
        ],
    }
    use crate::pac::TIM5 as TIM;
    use crate::pwm::{PinCh, C1, C2, C3, C4};
    impl PinCh<C1> for TIM {
        type Ch = Ch1;
    }
    impl PinCh<C2> for TIM {
        type Ch = Ch2;
    }
    impl PinCh<C3> for TIM {
        type Ch = Ch3;
    }
    impl PinCh<C4> for TIM {
        type Ch = Ch4;
    }
}

pub mod tim8 {
    use super::*;
    pin! {
        <Bkin2> for [
            PA8<3>,
            PG3<3>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI1<3>,
        ],
        <Bkin2Comp1> for [
            PA8<12>,
            PG3<11>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI1<11>,
        ],
        <Bkin2Comp2> for [
            PA8<12>,
            PG3<11>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI1<11>,
        ],
        <Bkin> for [
            PA6<3>,
            PG2<3>,
            PK2<3>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI4<3>,
        ],
        <BkinComp1> for [
            PA6<10>,
            PG2<11>,
            PK2<10>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI4<11>,
        ],
        <BkinComp2> for [
            PA6<10>,
            PG2<11>,
            PK2<10>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI4<11>,
        ],
        <Ch1> for [
            PC6<3>,
            PJ8<3>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI5<3>,
        ],
        <Ch1N> for [
            PA5<3>,
            PA7<3>,
            PH13<3>,
            PJ9<3>,
        ],
        <Ch2> for [
            PC7<3>,
            PJ10<3>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI6<3>,
            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PJ6<3>,
        ],
        <Ch2N> for [
            PB0<3>,
            PB14<3>,
            PH14<3>,
            PJ11<3>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PJ7<3>,
        ],
        <Ch3> for [
            PC8<3>,
            PK0<3>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI7<3>,
        ],
        <Ch3N> for [
            PB15<3>,
            PB1<3>,
            PH15<3>,
            PK1<3>,
        ],
        <Ch4> for [
            PC9<3>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI2<3>,
        ],
        <Etr> for [
            PA0<3>,
            PG8<3>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI3<3>,
        ],
    }
    use crate::pac::TIM8 as TIM;
    use crate::pwm::{PinCh, C1, C2, C3, C4};
    impl PinCh<C1> for TIM {
        type Ch = Ch1;
    }
    impl PinCh<C2> for TIM {
        type Ch = Ch2;
    }
    impl PinCh<C3> for TIM {
        type Ch = Ch3;
    }
    impl PinCh<C4> for TIM {
        type Ch = Ch4;
    }
}

pub mod tim12 {
    use super::*;
    pin! {
        <Ch1> for [
            PB14<2>,
            PH6<2>,
        ],
        <Ch2> for [
            PB15<2>,
            PH9<2>,
        ],
    }
    use crate::pac::TIM12 as TIM;
    use crate::pwm::{PinCh, C1, C2};
    impl PinCh<C1> for TIM {
        type Ch = Ch1;
    }
    impl PinCh<C2> for TIM {
        type Ch = Ch2;
    }
}

pub mod tim13 {
    use super::*;
    pin! {
        <Ch1> for [
            PA6<9>,
            PF8<9>,
        ],
    }
    use crate::pac::TIM13 as TIM;
    use crate::pwm::{PinCh, C1};
    impl PinCh<C1> for TIM {
        type Ch = Ch1;
    }
}

pub mod tim14 {
    use super::*;
    pin! {
        <Ch1> for [
            PA7<9>,
            PF9<9>,
        ],
    }
    use crate::pac::TIM14 as TIM;
    use crate::pwm::{PinCh, C1};
    impl PinCh<C1> for TIM {
        type Ch = Ch1;
    }
}

pub mod tim15 {
    use super::*;
    pin! {
        <Bkin> for [
            PA0<4>,
            PE3<4>,

            #[cfg(any(feature = "gpio-h72", feature = "gpio-h7a2"))]
            PD2<4>,
        ],
        <Ch1> for [
            PA2<4>,
            PE5<4>,

            #[cfg(any(feature = "gpio-h72", feature = "gpio-h7a2"))]
            PC12<2>,
        ],
        <Ch1N> for [
            PA1<4>,
            PE4<4>,
        ],
        <Ch2> for [
            PA3<4>,
            PE6<4>,
        ],
    }
    use crate::pac::TIM15 as TIM;
    use crate::pwm::{PinCh, C1, C2};
    impl PinCh<C1> for TIM {
        type Ch = Ch1;
    }
    impl PinCh<C2> for TIM {
        type Ch = Ch2;
    }
}

pub mod tim16 {
    use super::*;
    pin! {
        <Bkin> for [
            PB4<1>,
            PF10<1>,
        ],
        <Ch1> for [
            PB8<1>,
            PF6<1>,
        ],
        <Ch1N> for [
            PB6<1>,
            PF8<1>,
        ],
    }
    use crate::pac::TIM16 as TIM;
    use crate::pwm::{PinCh, C1};
    impl PinCh<C1> for TIM {
        type Ch = Ch1;
    }
}

pub mod tim17 {
    use super::*;
    pin! {
        <Bkin> for [
            PB5<1>,
            PG6<1>,
        ],
        <Ch1> for [
            PB9<1>,
            PF7<1>,
        ],
        <Ch1N> for [
            PB7<1>,
            PF9<1>,
        ],
    }
    use crate::pac::TIM17 as TIM;
    use crate::pwm::{PinCh, C1};
    impl PinCh<C1> for TIM {
        type Ch = Ch1;
    }
}

#[cfg(feature = "gpio-h72")]
pub mod tim23 {
    use super::*;
    pin! {
        <Ch1> for [
            PF0<13>,
            PF6<13>,
            PG12<13>,
        ],
        <Ch2> for [
            PF1<13>,
            PF7<13>,
            PG13<13>,
        ],
        <Ch3> for [
            PF2<13>,
            PF8<13>,
            PG14<13>,
        ],
        <Ch4> for [
            PF3<13>,
            PF9<13>,
        ],
        <Etr> for [
            PB2<13>,
            PG3<13>,
        ],
    }
    use crate::pac::TIM23 as TIM;
    use crate::pwm::{PinCh, C1, C2, C3, C4};
    impl PinCh<C1> for TIM {
        type Ch = Ch1;
    }
    impl PinCh<C2> for TIM {
        type Ch = Ch2;
    }
    impl PinCh<C3> for TIM {
        type Ch = Ch3;
    }
    impl PinCh<C4> for TIM {
        type Ch = Ch4;
    }
}

#[cfg(feature = "gpio-h72")]
pub mod tim24 {
    use super::*;
    pin! {
        <Ch1> for [
            PF11<14>,
        ],
        <Ch2> for [
            PF12<14>,
        ],
        <Ch3> for [
            PF13<14>,
        ],
        <Ch4> for [
            PF14<14>,
        ],
        <Etr> for [
            PB3<14>,
            PG2<14>,
        ],
    }
    use crate::pac::TIM24 as TIM;
    use crate::pwm::{PinCh, C1, C2, C3, C4};
    impl PinCh<C1> for TIM {
        type Ch = Ch1;
    }
    impl PinCh<C2> for TIM {
        type Ch = Ch2;
    }
    impl PinCh<C3> for TIM {
        type Ch = Ch3;
    }
    impl PinCh<C4> for TIM {
        type Ch = Ch4;
    }
}

#[cfg(any(feature = "gpio-h72", feature = "gpio-h747"))]
pub mod lptim4 {
    use super::*;
    pin! {
        <Out> for [
            PA2<3>,
        ],
    }
}

#[cfg(any(feature = "gpio-h72", feature = "gpio-h747"))]
pub mod lptim5 {
    use super::*;
    pin! {
        <Out> for [
            PA3<3>,
        ],
    }
}

#[cfg(feature = "gpio-h747")]
pub mod hrtim {
    use super::*;
    pin! {
        <Cha1> for [
            PC6<1>,
        ],
        <Cha2> for [
            PC7<1>,
        ],
        <Chb1> for [
            PC8<1>,
        ],
        <Chb2> for [
            PA8<2>,
        ],
        <Chc1> for [
            PA9<2>,
        ],
        <Chc2> for [
            PA10<2>,
        ],
        <Chd1> for [
            PA11<2>,
        ],
        <Chd2> for [
            PA12<2>,
        ],
        <Che1> for [
            PG6<2>,
        ],
        <Che2> for [
            PG7<2>,
        ],
        <Eev10> for [
            PG13<2>,
        ],
        <Eev1> for [
            PC10<2>,
        ],
        <Eev2> for [
            PC12<2>,
        ],
        <Eev3> for [
            PD5<2>,
        ],
        <Eev4> for [
            PG11<2>,
        ],
        <Eev5> for [
            PG12<2>,
        ],
        <Eev6> for [
            PB4<3>,
        ],
        <Eev7> for [
            PB5<3>,
        ],
        <Eev8> for [
            PB6<3>,
        ],
        <Eev9> for [
            PB7<3>,
        ],
        <Flt1> for [
            PA15<2>,
        ],
        <Flt2> for [
            PC11<2>,
        ],
        <Flt3> for [
            PD4<2>,
        ],
        <Flt4> for [
            PB3<2>,
        ],
        <Flt5> for [
            PG10<2>,
        ],
        <Scin> for [
            PB11<2>,
            PE0<3>,
        ],
        <Scout> for [
            PB10<2>,
            PE1<3>,
        ],
    }
}

pub mod usart1 {
    use super::*;
    pin! {
        <Ck> for [
            PA8<7>,
        ],
        <Cts> for [
            PA11<7>,
        ],
        <De> for [
            PA12<7>,
        ],
        <Nss> for [
            PA11<7>,
        ],
        <Rts> for [
            PA12<7>,
        ],
        <Rx> for [
            PA10<7>,
            PB15<4>,
            PB7<7>,
        ],
        <Tx> for [
            PA9<7>,
            PB14<4>,
            PB6<7>,
        ],
    }
}

pub mod usart2 {
    use super::*;
    pin! {
        <Ck> for [
            PA4<7>,
            PD7<7>,
        ],
        <Cts> for [
            PA0<7>,
            PD3<7>,
        ],
        <De> for [
            PA1<7>,
            PD4<7>,
        ],
        <Nss> for [
            PA0<7>,
            PD3<7>,
        ],
        <Rts> for [
            PA1<7>,
            PD4<7>,
        ],
        <Rx> for [
            PA3<7>,
            PD6<7>,
        ],
        <Tx> for [
            PA2<7>,
            PD5<7>,
        ],
    }
}

pub mod usart3 {
    use super::*;
    pin! {
        <Ck> for [
            PB12<7>,
            PC12<7>,
            PD10<7>,
        ],
        <Cts> for [
            PB13<7>,
            PD11<7>,
        ],
        <De> for [
            PB14<7>,
            PD12<7>,
        ],
        <Nss> for [
            PB13<7>,
            PD11<7>,
        ],
        <Rts> for [
            PB14<7>,
            PD12<7>,
        ],
        <Rx> for [
            PB11<7>,
            PC11<7>,
            PD9<7>,
        ],
        <Tx> for [
            PB10<7>,
            PC10<7>,
            PD8<7>,
        ],
    }
}

pub mod usart6 {
    use super::*;
    pin! {
        <Ck> for [
            PC8<7>,
            PG7<7>,
        ],
        <Cts> for [
            PG13<7>,
            PG15<7>,
        ],
        <De> for [
            PG12<7>,
            PG8<7>,
        ],
        <Nss> for [
            PG13<7>,
            PG15<7>,
        ],
        <Rts> for [
            PG12<7>,
            PG8<7>,
        ],
        <Rx> for [
            PC7<7>,
            PG9<7>,
        ],
        <Tx> for [
            PC6<7>,
            PG14<7>,
        ],
    }
}

#[cfg(any(feature = "gpio-h72", feature = "gpio-h7a2"))]
pub mod usart10 {
    use super::*;
    pin! {
        <Ck> for [
            PE15<11>,
            PG15<11>,
        ],
        <Cts> for [
            #[cfg(feature = "gpio-h72")]
            PG13<4>,

            #[cfg(feature = "gpio-h7a2")]
            PG13<11>,
        ],
        <De> for [
            #[cfg(feature = "gpio-h72")]
            PG14<4>,

            #[cfg(feature = "gpio-h7a2")]
            PG14<11>,
        ],
        <Nss> for [
            #[cfg(feature = "gpio-h72")]
            PG13<4>,

            #[cfg(feature = "gpio-h7a2")]
            PG13<11>,
        ],
        <Rts> for [
            #[cfg(feature = "gpio-h72")]
            PG14<4>,

            #[cfg(feature = "gpio-h7a2")]
            PG14<11>,
        ],
        <Rx> for [
            #[cfg(feature = "gpio-h72")]
            PE2<4>,
            #[cfg(feature = "gpio-h72")]
            PG11<4>,

            #[cfg(feature = "gpio-h7a2")]
            PE2<11>,
            #[cfg(feature = "gpio-h7a2")]
            PG11<11>,
        ],
        <Tx> for [
            PE3<11>,

            #[cfg(feature = "gpio-h72")]
            PG12<4>,

            #[cfg(feature = "gpio-h7a2")]
            PG12<11>,
        ],
    }
}

pub mod uart4 {
    use super::*;
    pin! {
        <Cts> for [
            PB0<8>,
            PB15<8>,
        ],
        <De> for [
            PA15<8>,
            PB14<8>,
        ],
        <Rts> for [
            PA15<8>,
            PB14<8>,
        ],
        <Rx> for [
            PA11<6>,
            PA1<8>,
            PB8<8>,
            PC11<8>,
            PD0<8>,
            PH14<8>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI9<8>,
        ],
        <Tx> for [
            PA0<8>,
            PA12<6>,
            PB9<8>,
            PC10<8>,
            PD1<8>,
            PH13<8>,
        ],
    }
}

pub mod uart5 {
    use super::*;
    pin! {
        <Cts> for [
            PC9<8>,
        ],
        <De> for [
            PC8<8>,
        ],
        <Rts> for [
            PC8<8>,
        ],
        <Rx> for [
            PB12<14>,
            PB5<14>,
            PD2<8>,
        ],
        <Tx> for [
            PB13<14>,
            PB6<14>,
            PC12<8>,
        ],
    }
}

pub mod uart7 {
    use super::*;
    pin! {
        <Cts> for [
            PE10<7>,
            PF9<7>,
        ],
        <De> for [
            PE9<7>,
            PF8<7>,
        ],
        <Rts> for [
            PE9<7>,
            PF8<7>,
        ],
        <Rx> for [
            PA8<11>,
            PB3<11>,
            PE7<7>,
            PF6<7>,
        ],
        <Tx> for [
            PA15<11>,
            PB4<11>,
            PE8<7>,
            PF7<7>,
        ],
    }
}

pub mod uart8 {
    use super::*;
    pin! {
        <Cts> for [
            PD14<8>,
        ],
        <De> for [
            PD15<8>,
        ],
        <Rts> for [
            PD15<8>,
        ],
        <Rx> for [
            PE0<8>,
            PJ9<8>,
        ],
        <Tx> for [
            PE1<8>,
            PJ8<8>,
        ],
    }
}

#[cfg(any(feature = "gpio-h72", feature = "gpio-h7a2"))]
pub mod uart9 {
    use super::*;
    pin! {
        <Cts> for [
            PD0<11>,

            #[cfg(feature = "gpio-h7a2")]
            PJ4<11>,
        ],
        <De> for [
            PD13<11>,

            #[cfg(feature = "gpio-h7a2")]
            PJ3<11>,
        ],
        <Rts> for [
            PD13<11>,

            #[cfg(feature = "gpio-h7a2")]
            PJ3<11>,
        ],
        <Rx> for [
            PD14<11>,
            PG0<11>,
        ],
        <Tx> for [
            PD15<11>,
            PG1<11>,
        ],
    }
}

pub mod usb {
    use super::*;
    pin! {
        <OtgHsUlpiCk> for [
            PA5<10>,
        ],
        <OtgHsUlpiD0> for [
            PA3<10>,
        ],
        <OtgHsUlpiD1> for [
            PB0<10>,
        ],
        <OtgHsUlpiD2> for [
            PB1<10>,
        ],
        <OtgHsUlpiD3> for [
            PB10<10>,
        ],
        <OtgHsUlpiD4> for [
            PB11<10>,
        ],
        <OtgHsUlpiD5> for [
            PB12<10>,
        ],
        <OtgHsUlpiD6> for [
            PB13<10>,
        ],
        <OtgHsUlpiD7> for [
            PB5<10>,
        ],
        <OtgHsUlpiNxt> for [
            PH4<10>,

            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PC3<10>,
        ],
        <OtgHsUlpiStp> for [
            PC0<10>,
        ],

        <OtgHsId> for [
            #[cfg(any(feature = "gpio-h72", feature = "gpio-h7a2"))]
            PA10<10>,

            #[cfg(feature = "gpio-h747")]
            PB12<12>,
        ],
        <OtgHsSof> for [
            #[cfg(any(feature = "gpio-h72", feature = "gpio-h7a2"))]
            PA8<10>,

            #[cfg(feature = "gpio-h747")]
            PA4<12>,
        ],

        <OtgHsUlpiDir> for [
            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PC2<10>,
            #[cfg(any(feature = "gpio-h747", feature = "gpio-h7a2"))]
            PI11<10>,
        ],

        <OtgFsDm> for [
            #[cfg(feature = "gpio-h747")]
            PA11<10>,
        ],
        <OtgFsDp> for [
            #[cfg(feature = "gpio-h747")]
            PA12<10>,
        ],
        <OtgFsId> for [
            #[cfg(feature = "gpio-h747")]
            PA10<10>,
        ],
        <OtgFsSof> for [
            #[cfg(feature = "gpio-h747")]
            PA8<10>,
        ],
        <OtgHsDm> for [
            #[cfg(feature = "gpio-h747")]
            PB14<12>,
        ],
        <OtgHsDp> for [
            #[cfg(feature = "gpio-h747")]
            PB15<12>,
        ],
    }
}
