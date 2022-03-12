#![deny(warnings)]
#![no_main]
#![no_std]

use cortex_m_rt::entry;
#[macro_use]
mod utilities;
use stm32h7xx_hal::{pac, prelude::*};

use log::info;

use nb::block;

#[entry]
fn main() -> ! {
    utilities::logger::init();
    let dp = pac::Peripherals::take().expect("Cannot take peripherals");

    // Constrain and Freeze power
    info!("Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    let pwrcfg = example_power!(pwr).freeze();

    // Constrain and Freeze clock
    info!("Setup RCC...                  ");
    let rcc = dp.RCC.constrain();
    let ccdr = rcc
        .sys_ck(100.mhz())
        .pll1_q_ck(20_480.khz()) // SAI1 clock source
        .freeze(pwrcfg, &dp.SYSCFG);

    // Acquire GPIO peripherals. This also enables the clock for each
    // GPIO in the RCC register.
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
    let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);

    let d1 = gpioc.pc1.into_alternate();
    let ck1 = gpioe.pe2.into_alternate();
    let pins = (ck1, d1);

    info!("");
    info!("stm32h7xx-hal example - SAI PDM");
    info!("");

    // Configure SAI for PDM mode
    let mut sai =
        dp.SAI1
            .pdm(pins, 1_024.khz(), ccdr.peripheral.SAI1, &ccdr.clocks);

    loop {
        info!("0x{:04x}", 0xFFFF & block!(sai.read_data()).unwrap());
    }
}
