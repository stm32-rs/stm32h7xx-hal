#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate panic_itm;

use cortex_m;
use cortex_m_rt::entry;
use stm32h7xx_hal::hal::digital::v2::ToggleableOutputPin;
use stm32h7xx_hal::{pac, prelude::*};

use cortex_m_log::println;
use cortex_m_log::{
    destination::Itm, printer::itm::InterruptSync as InterruptSyncItm,
};

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().expect("cannot take peripherals");
    let mut log = InterruptSyncItm::new(Itm::new(cp.ITM));

    // Constrain and Freeze power
    println!(log, "Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    let vos = pwr.freeze();

    // Constrain and Freeze clock
    println!(log, "Setup RCC...                  ");
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sys_ck(100.mhz()).freeze(vos, &dp.SYSCFG);

    println!(log, "");
    println!(log, "stm32h7xx-hal example - Random Blinky");
    println!(log, "");

    let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);

    // Configure PE1 as output.
    let mut led = gpioe.pe1.into_push_pull_output();

    // Get the delay provider.
    let mut delay = cp.SYST.delay(ccdr.clocks);

    // Get true random number generator
    let mut rng = dp.RNG.constrain(ccdr.peripheral.RNG, &ccdr.clocks);
    let mut random_bytes = [0u16; 3];
    match rng.fill(&mut random_bytes) {
        Ok(()) => println!(log, "random bytes: {:?}", random_bytes),
        Err(err) => println!(log, "RNG error: {:?}", err),
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

                led.toggle().unwrap();
                delay.delay_ms(period);
            }
            Err(err) => println!(log, "RNG error: {:?}", err),
        }
    }
}
