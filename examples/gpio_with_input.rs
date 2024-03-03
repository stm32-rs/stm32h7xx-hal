//! Demonstrates the use of the GPIO method `with_input` (and similar methods)
//!
//! https://docs.rs/stm32h7xx-hal/latest/stm32h7xx_hal/gpio/struct.Pin.html#method.with_input

#![deny(warnings)]
#![no_main]
#![no_std]

use cortex_m_rt::entry;
use embedded_hal_1::delay::DelayNs; // this example uses embedded-hal v1.0
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
    let pwrcfg = example_power!(pwr).freeze();

    // Constrain and Freeze clock
    info!("Setup RCC...                  ");
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sys_ck(100.MHz()).freeze(pwrcfg, &dp.SYSCFG);

    info!("");
    info!("stm32h7xx-hal example - GPIO with_input");
    info!("");

    let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);

    // Configure PE1 as output.
    let mut led = gpioe.pe1.into_push_pull_output();

    // Get the delay provider.
    let mut delay = cp.SYST.delay(ccdr.clocks);

    loop {
        led.set_high();
        delay.delay_ms(100);

        led.set_low();
        delay.delay_ms(100);

        let is_high = led.with_input(|x| x.is_high());
        info!("LED pin high? {}", is_high);
    }
}
