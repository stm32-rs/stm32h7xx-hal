mod h7;
pub use h7::*;

macro_rules! pin {
    ( $($(#[$docs:meta])* <$name:ident, $Otype:ident> for $(no: $NoPin:ident,)? [$(
        $(#[$attr:meta])* $PX:ident<$A:literal $(, Speed::$Speed:ident)?>,
    )*],)*) => {
        $(
            #[derive(Debug)]
            $(#[$docs])*
            pub enum $name {
                $(
                    None($NoPin<$Otype>),
                )?

                $(
                    $(#[$attr])*
                    $PX(gpio::$PX<$crate::gpio::Alternate<$A, $Otype>>),
                )*
            }

            impl crate::Sealed for $name { }

            #[allow(unreachable_patterns)]
            impl $crate::gpio::ReadPin for $name {
                fn is_low(&self) -> bool {
                    match self {
                        $(
                            $(#[$attr])*
                            Self::$PX(p) => p.is_low(),
                        )*
                        _ => false,
                    }
                }
            }

            #[allow(unreachable_patterns)]
            impl $crate::gpio::PinSpeed for $name {
                fn set_speed(&mut self, _speed: $crate::gpio::Speed) {
                    match self {
                        $(
                            $(#[$attr])*
                            Self::$PX(p) => p.set_speed(_speed),
                        )*
                        _ => {}
                    }
                }
            }

            #[allow(unreachable_patterns)]
            impl $crate::gpio::PinPull for $name {
                fn set_internal_resistor(&mut self, _pull: $crate::gpio::Pull) {
                    match self {
                        $(
                            $(#[$attr])*
                            Self::$PX(p) => p.set_internal_resistor(_pull),
                        )*
                        _ => {}
                    }
                }
            }

            $(
                impl From<$NoPin<$Otype>> for $name {
                    fn from(p: $NoPin<$Otype>) -> Self {
                        Self::None(p)
                    }
                }
            )?

            $(
                $(#[$attr])*
                impl<MODE> From<gpio::$PX<MODE>> for $name
                where
                    MODE: $crate::gpio::marker::NotAlt + $crate::gpio::PinMode
                {
                    fn from(p: gpio::$PX<MODE>) -> Self {
                        Self::$PX(p.into_mode() $(.speed($crate::gpio::Speed::$Speed))?)
                    }
                }

                $(#[$attr])*
                impl From<gpio::$PX<$crate::gpio::Alternate<$A, $Otype>>> for $name {
                    fn from(p: gpio::$PX<$crate::gpio::Alternate<$A, $Otype>>) -> Self {
                        Self::$PX(p $(.speed($crate::gpio::Speed::$Speed))?)
                    }
                }

                $(#[$attr])*
                #[allow(irrefutable_let_patterns)]
                impl<MODE> TryFrom<$name> for gpio::$PX<MODE>
                where
                    MODE: $crate::gpio::PinMode,
                    $crate::gpio::Alternate<$A, $Otype>: $crate::gpio::PinMode,
                {
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

    ( $($(#[$docs:meta])* <$name:ident> default:$DefaultOtype:ident for $(no: $NoPin:ident,)? [$(
            $(#[$attr:meta])* $PX:ident<$A:literal $(, Speed::$Speed:ident)?>,
    )*],)*) => {
        $(
            #[derive(Debug)]
            $(#[$docs])*
            pub enum $name<Otype = $DefaultOtype> {
                $(
                    None($NoPin<Otype>),
                )?

                $(
                    $(#[$attr])*
                    $PX(gpio::$PX<$crate::gpio::Alternate<$A, Otype>>),
                )*
            }

            impl<Otype> crate::Sealed for $name<Otype> { }

            #[allow(unreachable_patterns)]
            impl<Otype> $crate::gpio::ReadPin for $name<Otype> {
                fn is_low(&self) -> bool {
                    match self {
                        $(
                            $(#[$attr])*
                            Self::$PX(p) => p.is_low(),
                        )*
                        _ => false,
                    }
                }
            }

            #[allow(unreachable_patterns)]
            impl<Otype> $crate::gpio::PinSpeed for $name<Otype> {
                fn set_speed(&mut self, _speed: $crate::gpio::Speed) {
                    match self {
                        $(
                            $(#[$attr])*
                            Self::$PX(p) => p.set_speed(_speed),
                        )*
                        _ => {}
                    }
                }
            }

            #[allow(unreachable_patterns)]
            impl<Otype> $crate::gpio::PinPull for $name<Otype> {
                fn set_internal_resistor(&mut self, _pull: $crate::gpio::Pull) {
                    match self {
                        $(
                            $(#[$attr])*
                            Self::$PX(p) => p.set_internal_resistor(_pull),
                        )*
                        _ => {}
                    }
                }
            }

            $(
                impl<Otype> From<$NoPin<Otype>> for $name<Otype> {
                    fn from(p: $NoPin<Otype>) -> Self {
                        Self::None(p)
                    }
                }
            )?

            $(
                $(#[$attr])*
                impl<MODE, Otype> From<gpio::$PX<MODE>> for $name<Otype>
                where
                    MODE: $crate::gpio::marker::NotAlt + $crate::gpio::PinMode,
                    $crate::gpio::Alternate<$A, Otype>: $crate::gpio::PinMode,
                {
                    fn from(p: gpio::$PX<MODE>) -> Self {
                        Self::$PX(p.into_mode() $(.speed($crate::gpio::Speed::$Speed))?)
                    }
                }

                $(#[$attr])*
                impl<Otype> From<gpio::$PX<$crate::gpio::Alternate<$A, Otype>>> for $name<Otype> {
                    fn from(p: gpio::$PX<$crate::gpio::Alternate<$A, Otype>>) -> Self {
                        Self::$PX(p $(.speed($crate::gpio::Speed::$Speed))?)
                    }
                }

                $(#[$attr])*
                #[allow(irrefutable_let_patterns)]
                impl<MODE, Otype> TryFrom<$name<Otype>> for gpio::$PX<MODE>
                where
                    MODE: $crate::gpio::PinMode,
                    $crate::gpio::Alternate<$A, Otype>: $crate::gpio::PinMode,
                {
                    type Error = ();

                    fn try_from(a: $name<Otype>) -> Result<Self, Self::Error> {
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
use pin;

// CAN pins
pub trait CanCommon {
    type Rx;
    type Tx;
}

// DFSDM pins
pub trait DfsdmBasic {
    type Ckin0;
    type Ckin1;
    type Ckout;
    type Datin0;
    type Datin1;
}
pub trait DfsdmGeneral: DfsdmBasic {
    type Ckin2;
    type Ckin3;
    type Datin2;
    type Datin3;
}
pub trait DfsdmAdvanced: DfsdmGeneral {
    type Ckin4;
    type Ckin5;
    type Ckin6;
    type Ckin7;
    type Datin4;
    type Datin5;
    type Datin6;
    type Datin7;
}

// Serial pins
pub trait SerialAsync {
    /// Receive
    type Rx<Otype>;
    /// Transmit
    type Tx<Otype>;
}
/// Synchronous mode
pub trait SerialSync {
    type Ck;
}
/// Hardware flow control (RS232)
pub trait SerialRs232 {
    /// Receive
    type Cts;
    /// Transmit
    type Rts;
}

// I2C pins
pub trait I2cCommon {
    type Scl;
    type Sda;
    type Smba;
}

// I2S pins
pub trait I2sCommon {
    type Ck;
    type Sd;
    type Ws: crate::gpio::ReadPin + crate::gpio::ExtiPin;
}
pub trait I2sMaster {
    type Mck;
}
pub trait I2sExtPin {
    type ExtSd;
}

// QuadSPI pins
pub trait QuadSpiBank {
    type Io0: crate::gpio::PinSpeed;
    type Io1: crate::gpio::PinSpeed;
    type Io2: crate::gpio::PinSpeed;
    type Io3: crate::gpio::PinSpeed;
    type Ncs: crate::gpio::PinSpeed;
}

pub trait OctospiPort {
    type Clk;
    type Nclk;
    type Dqs;
    type Ncs;
    type Io0;
    type Io1;
    type Io2;
    type Io3;
    type Io4;
    type Io5;
    type Io6;
    type Io7;
}

// SAI pins

pub trait SaiChannels {
    type A: SaiChannel;
    type B: SaiChannel;
}
pub trait SaiChannel {
    type Fs;
    type Mclk;
    type Sck;
    type Sd;
}
pub trait SaiPdm {
    type D1;
    type D2;
    type D3;
    type Ck1;
    type Ck2;
}

// SPDIFRX pins

pub trait SPdifIn<const C: u8> {
    type In;
}

// SPI pins
pub trait SpiCommon {
    type Miso;
    type Mosi;
    type Nss;
    type Sck;
}

// SDMMC pins
pub trait SdmmcCommon {
    type Ck;
    type Cmd;
    type D0;
    type D1;
    type D2;
    type D3;
    type D4;
    type D5;
    type D6;
    type D7;
}

// Timer pins

/// Input capture / Output compare channel `C`
pub trait TimCPin<const C: u8> {
    type Ch<Otype>;
}

/// This trait marks which GPIO pins may be used as complementary PWM channels; it should not be directly used.
/// See the device datasheet 'Pin descriptions' chapter for which pins can be used with which timer PWM channels (or look at Implementors)
pub trait TimNCPin<const C: u8> {
    type ChN<Otype>;
}

/// Break input
pub trait TimBkin {
    type Bkin;
}

/// Break input2
pub trait TimBkin2 {
    type Bkin2;
}

/// External trigger timer input
pub trait TimEtr {
    type Etr;
}
