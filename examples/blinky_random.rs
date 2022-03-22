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
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().expect("cannot take peripherals");

    // Constrain and Freeze power
    info!("Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    let pwrcfg = example_power!(pwr).freeze();

    // Constrain and Freeze clock
    info!("Setup RCC...                  ");
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sys_ck(100.mhz()).freeze(pwrcfg, &dp.SYSCFG);

    info!("");
    info!("stm32h7xx-hal example - Random Blinky");
    info!("");

    let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);

    // Configure PE1 as output.
    let mut led = gpioe.pe1.into_push_pull_output();

    // Get the delay provider.
    let mut delay = cp.SYST.delay(ccdr.clocks);

    // Get true random number generator
    let mut rng = dp.RNG.constrain(ccdr.peripheral.RNG, &ccdr.clocks);
    let mut random_bytes = [0u16; 3];
    match rng.fill(&mut random_bytes) {
        Ok(()) => info!("random bytes: {:?}", random_bytes),
        Err(err) => info!("RNG error: {:?}", err),
    }

    loop {
        let random_element: Result<u32, _> = rng.gen();

        match random_element {
            Ok(random) => {
                // NOTE: the result of the expression `random % 200`
                // is biased. This bias is called "modulo-bias". It is
                // acceptable here for simplicity, but may not be
                // acceptable for your application.
                let period = random % 200_u32;

                led.toggle();
                delay.delay_ms(period);
            }
            Err(err) => info!("RNG error: {:?}", err),
        }
    }
}
