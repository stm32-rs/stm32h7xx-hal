//! Digital to Analog Converter (DAC)
//!
//! # Examples
//!
//! - [Simple DAC example](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/dac.rs)

use core::marker::PhantomData;
use core::mem::MaybeUninit;

use crate::gpio::{self, Analog};
use crate::hal::blocking::delay::DelayUs;
use crate::rcc::{rec, ResetEnable};
#[cfg(not(feature = "rm0455"))]
use crate::stm32::DAC as DAC1;
#[cfg(feature = "rm0455")]
use crate::stm32::{DAC1, DAC2};
use crate::traits::DacOut;

/// Enabled DAC (type state)
pub struct Enabled;
/// Enabled DAC without output buffer (type state)
pub struct EnabledUnbuffered;
/// Disabled DAC (type state)
pub struct Disabled;

pub trait ED {}
impl ED for Enabled {}
impl ED for EnabledUnbuffered {}
impl ED for Disabled {}

pub struct C1<DAC, ED> {
    _dac: PhantomData<DAC>,
    _enabled: PhantomData<ED>,
}
pub struct C2<DAC, ED> {
    _dac: PhantomData<DAC>,
    _enabled: PhantomData<ED>,
}

/// Trait for GPIO pins that can be converted to DAC output pins
pub trait Pins<DAC> {
    type Output;
}

// DAC1

impl Pins<DAC1> for gpio::PA4<Analog> {
    type Output = C1<DAC1, Disabled>;
}

impl Pins<DAC1> for gpio::PA5<Analog> {
    type Output = C2<DAC1, Disabled>;
}

impl Pins<DAC1> for (gpio::PA4<Analog>, gpio::PA5<Analog>) {
    type Output = (C1<DAC1, Disabled>, C2<DAC1, Disabled>);
}

// DAC2

#[cfg(feature = "rm0455")]
impl Pins<DAC2> for gpio::PA6<Analog> {
    type Output = C1<DAC2, Disabled>;
}

pub fn dac<PINS, DAC, REC: ResetEnable>(
    _dac: DAC,
    _pins: PINS,
    prec: REC,
) -> PINS::Output
where
    PINS: Pins<DAC>,
{
    // Enable DAC clocks and reset
    prec.enable().reset();

    #[allow(clippy::uninit_assumed_init)]
    unsafe {
        MaybeUninit::uninit().assume_init()
    }
}

macro_rules! dac {
    ($DAC:ident, $CX:ident, $en:ident, $cen:ident, $cal_flag:ident, $trim:ident,
     $mode:ident, $dhrx:ident, $dor:ident, $daccxdhr:ident) => {
        impl $CX<$DAC, Disabled> {
            pub fn enable(self) -> $CX<$DAC, Enabled> {
                let dac = unsafe { &(*$DAC::ptr()) };

                dac.mcr.modify(|_, w| unsafe { w.$mode().bits(0) });
                dac.cr.modify(|_, w| w.$en().set_bit());

                $CX {
                    _dac: PhantomData,
                    _enabled: PhantomData,
                }
            }

            pub fn enable_unbuffered(self) -> $CX<$DAC, EnabledUnbuffered> {
                let dac = unsafe { &(*$DAC::ptr()) };

                dac.mcr.modify(|_, w| unsafe { w.$mode().bits(2) });
                dac.cr.modify(|_, w| w.$en().set_bit());

                $CX {
                    _dac: PhantomData,
                    _enabled: PhantomData,
                }
            }
        }

        impl<ED> $CX<$DAC, ED> {
            /// Calibrate the DAC output buffer by performing a "User
            /// trimming" operation. It is useful when the VDDA/VREF+
            /// voltage or temperature differ from the factory trimming
            /// conditions.
            ///
            /// The calibration is only valid when the DAC channel is
            /// operating with the buffer enabled. If applied in other
            /// modes it has no effect.
            ///
            /// After the calibration operation, the DAC channel is
            /// disabled.
            pub fn calibrate_buffer<T>(
                self,
                delay: &mut T,
            ) -> $CX<$DAC, Disabled>
            where
                T: DelayUs<u32>,
            {
                let dac = unsafe { &(*$DAC::ptr()) };
                dac.cr.modify(|_, w| w.$en().clear_bit());
                dac.mcr.modify(|_, w| unsafe { w.$mode().bits(0) });
                dac.cr.modify(|_, w| w.$cen().set_bit());
                let mut trim = 0;
                while true {
                    dac.ccr.modify(|_, w| unsafe { w.$trim().bits(trim) });
                    delay.delay_us(64_u32);
                    if dac.sr.read().$cal_flag().bit() {
                        break;
                    }
                    trim += 1;
                }
                dac.cr.modify(|_, w| w.$cen().clear_bit());

                $CX {
                    _dac: PhantomData,
                    _enabled: PhantomData,
                }
            }

            /// Disable the DAC channel
            pub fn disable(self) -> $CX<$DAC, Disabled> {
                let dac = unsafe { &(*$DAC::ptr()) };
                dac.cr.modify(|_, w| w.$en().clear_bit());

                $CX {
                    _dac: PhantomData,
                    _enabled: PhantomData,
                }
            }
        }

        /// DacOut implementation available in any Enabled/Disabled state
        impl<ED> DacOut<u16> for $CX<$DAC, ED> {
            fn set_value(&mut self, val: u16) {
                let dac = unsafe { &(*$DAC::ptr()) };
                dac.$dhrx.write(|w| unsafe { w.bits(val as u32) });
            }

            fn get_value(&mut self) -> u16 {
                let dac = unsafe { &(*$DAC::ptr()) };
                dac.$dor.read().bits() as u16
            }
        }
    };
}

pub trait DacExt: Sized {
    type REC: ResetEnable;

    fn dac<PINS>(self, pins: PINS, prec: Self::REC) -> PINS::Output
    where
        PINS: Pins<Self>;
}

impl DacExt for DAC1 {
    #[cfg(not(feature = "rm0455"))]
    type REC = rec::Dac12;
    #[cfg(feature = "rm0455")]
    type REC = rec::Dac1;

    fn dac<PINS>(self, pins: PINS, prec: Self::REC) -> PINS::Output
    where
        PINS: Pins<DAC1>,
    {
        dac(self, pins, prec)
    }
}

#[cfg(feature = "rm0455")]
impl DacExt for DAC2 {
    type REC = rec::Dac2;

    fn dac<PINS>(self, pins: PINS, prec: Self::REC) -> PINS::Output
    where
        PINS: Pins<DAC2>,
    {
        dac(self, pins, prec)
    }
}

dac!(DAC1, C1, en1, cen1, cal_flag1, otrim1, mode1, dhr12r1, dor1, dacc1dhr);
dac!(DAC1, C2, en2, cen2, cal_flag2, otrim2, mode2, dhr12r2, dor2, dacc2dhr);

#[cfg(feature = "rm0455")]
dac!(DAC2, C1, en1, cen1, cal_flag1, otrim1, mode1, dhr12r1, dor1, dacc1dhr);
#[cfg(feature = "rm0455")]
dac!(DAC2, C2, en2, cen2, cal_flag2, otrim2, mode2, dhr12r2, dor2, dacc2dhr);
