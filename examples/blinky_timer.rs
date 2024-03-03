//! This example produces waveform on Pin PE1 using the TIM2 and SysTick peripherals

#![deny(warnings)]
#![no_main]
#![no_std]

#[macro_use]
mod utilities;
#[macro_use(block)]
extern crate nb;

use cortex_m_rt::entry;
use embedded_hal_1::delay::DelayNs; // this example uses embedded-hal v1.0
use stm32h7xx_hal::{pac, prelude::*};

use log::info;

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
    let ccdr = rcc.freeze(pwrcfg, &dp.SYSCFG);

    info!("");
    info!("stm32h7xx-hal example - Blinky timer");
    info!("");

    let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);

    // Configure PE1 as output.
    let mut led = gpioe.pe1.into_push_pull_output();
    led.set_low();

    // Get the delay provider.
    let mut delay = cp.SYST.delay(ccdr.clocks);

    // Configure the timer.
    let mut timer = dp.TIM2.timer(1.Hz(), ccdr.peripheral.TIM2, &ccdr.clocks);

    loop {
        for _ in 0..5 {
            // 20ms wait with timer
            led.toggle();
            timer.start(50.Hz()).unwrap();
            block!(timer.wait()).ok();

            // Delay for 500ms. Timer must operate correctly on next use.
            led.toggle();
            delay.delay_ms(500);
        }
    }
}
