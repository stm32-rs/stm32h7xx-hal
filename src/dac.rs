//! Digital to Analog converter
///
///
use core::mem::MaybeUninit;

use crate::gpio::gpioa::{PA4, PA5};
use crate::gpio::Analog;
use crate::hal::blocking::delay::DelayUs;
use crate::rcc::Ccdr;
use crate::stm32::DAC;
use crate::traits::DacPin;

pub struct C1;
pub struct C2;

pub trait DacOut<V> {
    fn set_value(&mut self, val: V);
    fn get_value(&mut self) -> V;
}

pub trait Pins<DAC> {
    type Output;
}

impl Pins<DAC> for PA4<Analog> {
    type Output = C1;
}

impl Pins<DAC> for PA5<Analog> {
    type Output = C2;
}

impl Pins<DAC> for (PA4<Analog>, PA5<Analog>) {
    type Output = (C1, C2);
}

pub fn dac<PINS>(_dac: DAC, _pins: PINS, ccdr: &mut Ccdr) -> PINS::Output
where
    PINS: Pins<DAC>,
{
    // Enable DAC clocks
    ccdr.apb1l.enr().modify(|_, w| w.dac12en().set_bit());

    // Reset DAC
    ccdr.apb1l.rstr().modify(|_, w| w.dac12rst().set_bit());
    ccdr.apb1l.rstr().modify(|_, w| w.dac12rst().clear_bit());

    #[allow(clippy::uninit_assumed_init)]
    unsafe {
        MaybeUninit::uninit().assume_init()
    }
}

macro_rules! dac {
    ($CX:ident, $en:ident, $cen:ident, $cal_flag:ident, $trim:ident,
     $mode:ident, $dhrx:ident, $dor:ident, $daccxdhr:ident) => {
        impl DacPin for $CX {
            fn enable(&mut self) {
                let dac = unsafe { &(*DAC::ptr()) };
                dac.cr.modify(|_, w| w.$en().set_bit());
            }

            fn calibrate<T>(&mut self, delay: &mut T)
            where
                T: DelayUs<u32>,
            {
                let dac = unsafe { &(*DAC::ptr()) };
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
            }
        }

        impl DacOut<u16> for $CX {
            fn set_value(&mut self, val: u16) {
                let dac = unsafe { &(*DAC::ptr()) };
                dac.$dhrx.write(|w| unsafe { w.bits(val as u32) });
            }

            fn get_value(&mut self) -> u16 {
                let dac = unsafe { &(*DAC::ptr()) };
                dac.$dor.read().bits() as u16
            }
        }
    };
}

pub trait DacExt {
    fn dac<PINS>(self, pins: PINS, ccdr: &mut Ccdr) -> PINS::Output
    where
        PINS: Pins<DAC>;
}

impl DacExt for DAC {
    fn dac<PINS>(self, pins: PINS, ccdr: &mut Ccdr) -> PINS::Output
    where
        PINS: Pins<DAC>,
    {
        dac(self, pins, ccdr)
    }
}

dac!(C1, en1, cen1, cal_flag1, otrim1, mode1, dhr12r1, dor1, dacc1dhr);
dac!(C2, en2, cen2, cal_flag2, otrim2, mode2, dhr12r2, dor2, dacc2dhr);
