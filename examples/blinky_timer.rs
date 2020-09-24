#![deny(warnings)]
#![no_main]
#![no_std]

#[path = "utilities/logger.rs"]
mod logger;
#[macro_use(block)]
extern crate nb;

use cortex_m_rt::entry;
use stm32h7xx_hal::hal::digital::v2::{OutputPin, ToggleableOutputPin};
use stm32h7xx_hal::{pac, prelude::*};

use log::info;

#[entry]
fn main() -> ! {
    logger::init();
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    // Constrain and Freeze power
    info!("Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.freeze();

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
    led.set_low().unwrap();

    // Get the delay provider.
    let mut delay = cp.SYST.delay(ccdr.clocks);

    // Configure the timer.
    let mut timer = dp.TIM2.timer(1.hz(), ccdr.peripheral.TIM2, &ccdr.clocks);

    loop {
        for _ in 0..5 {
            // 20ms wait with timer
            led.toggle().unwrap();
            timer.start(20.ms());
            block!(timer.wait()).ok();

            // Delay for 500ms. Timer must operate correctly on next
            // use.
            led.toggle().unwrap();
            delay.delay_ms(500_u16);
        }
    }
}
