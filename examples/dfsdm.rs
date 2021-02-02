// #![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate panic_itm;

use cortex_m;
use cortex_m_rt::entry;
use stm32h7xx_hal::{pac, prelude::*};

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    // Constrain and Freeze power
    let pwr = dp.PWR.constrain();
    let vos = pwr.freeze();

    // Constrain and Freeze clock
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sys_ck(100.mhz()).freeze(vos, &dp.SYSCFG);

    let mut dfsdm = dp.DFSDM.dfsdm(ccdr.peripheral.DFSDM1);

    let input_data = [4i16; 640];
    for d in input_data.iter() {
        dfsdm.write(*d);
    }

    loop {
        match dfsdm.read() {
            Ok(value) => {
                assert!(value > 0);
            }
            Err(_) => {}
        }
    }
}
