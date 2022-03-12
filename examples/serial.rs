#![deny(warnings)]
#![no_main]
#![no_std]

use cortex_m_rt::entry;
#[macro_use]
mod utilities;
use log::info;

use stm32h7xx_hal::{pac, prelude::*};

use core::fmt::Write;

use nb::block;

#[entry]
fn main() -> ! {
    utilities::logger::init();
    let dp = pac::Peripherals::take().unwrap();

    // Constrain and Freeze power
    info!("Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    let pwrcfg = example_power!(pwr).freeze();

    // Constrain and Freeze clock
    info!("Setup RCC...                  ");
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sys_ck(160.mhz()).freeze(pwrcfg, &dp.SYSCFG);

    // Acquire the GPIOC peripheral. This also enables the clock for
    // GPIOC in the RCC register.
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);

    let tx = gpioc.pc10.into_alternate();
    let rx = gpioc.pc11.into_alternate();

    info!("");
    info!("stm32h7xx-hal example - USART");
    info!("");

    // Configure the serial peripheral.
    let serial = dp
        .USART3
        .serial((tx, rx), 19_200.bps(), ccdr.peripheral.USART3, &ccdr.clocks)
        .unwrap();

    let (mut tx, mut rx) = serial.split();

    // core::fmt::Write is implemented for tx.
    writeln!(tx, "Hello, world!").unwrap();

    loop {
        // Echo what is received on the serial link.
        let received = block!(rx.read()).unwrap();
        block!(tx.write(received)).ok();
    }
}
