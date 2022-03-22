#![deny(warnings)]
#![no_main]
#![no_std]

use cortex_m_rt::entry;
use stm32h7xx_hal::{pac, prelude::*};

#[macro_use]
mod utilities;

use log::info;

#[entry]
fn main() -> ! {
    utilities::logger::init();
    info!("stm32h7xx-hal example - digitalRead");

    let _cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    info!("Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    let pwrcfg = example_power!(pwr).freeze();

    info!("Setup RCC...                  ");
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sys_ck(100.mhz()).freeze(pwrcfg, &dp.SYSCFG);

    // Push button configuration
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);

    let button1 = gpioc.pc5.into_pull_up_input();

    loop {
        let result = button1.is_high();
        info!("{}", result);
        cortex_m::asm::delay(10000000);
    }
}
