#![deny(warnings)] // This code runs on stm32h747i-disco and does not use example! macro.
#![allow(unused_macros)]
#![no_main]
#![no_std]

use cortex_m_rt::entry;
use stm32h7xx_hal::{pac, prelude::*};

use log::info;

#[macro_use]
mod utilities;

#[entry]
fn main() -> ! {
    utilities::logger::init();
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    // Constrain and Freeze power
    info!("Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    // let pwrcfg = example_power!(pwr).freeze();
    let pwrcfg = pwr.smps().freeze(); // This code works normally on stm32h747i-disco.

    // Constrain and Freeze clock
    // RCC (Reset and Clock Control)
    info!("Setup RCC...                  ");
    let rcc = dp.RCC.constrain();

    // CCDR (Core Clock Distribution and Reset)
    //  link: https://docs.rs/stm32h7xx-hal/latest/stm32h7xx_hal/rcc/struct.Ccdr.html
    let ccdr = rcc.sys_ck(100.MHz()).freeze(pwrcfg, &dp.SYSCFG);

    info!("");
    info!("stm32h7xx-hal example - Blinky");
    info!("");

    let gpioi = dp.GPIOI.split(ccdr.peripheral.GPIOI); // <= GPIO settings for LEDs

    // Configure gpio pins as output.
    let mut led1 = gpioi.pi12.into_push_pull_output(); // PI12 for LED1
    let mut led2 = gpioi.pi13.into_push_pull_output(); // PI13 for LED2
    let mut led3 = gpioi.pi14.into_push_pull_output(); // PI14 for LED3
    let mut led4 = gpioi.pi15.into_push_pull_output(); // PI15 for LED4

    // Get the delay provider.
    let mut delay = cp.SYST.delay(ccdr.clocks);

    loop {
        loop {
            led1.set_high();
            led2.set_low();
            led3.set_high();
            led4.set_low();
            delay.delay_ms(500_u16);

            led1.set_low();
            led2.set_high();
            led3.set_low();
            led4.set_high();
            delay.delay_ms(500_u16);
        }
    }
}
