#![no_main]
#![no_std]

use cortex_m_rt::entry;
#[macro_use]
mod utilities;
use log::info;

use stm32h7xx_hal::{pac, prelude::*, serial::config::Config};

use core::fmt::Write;

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
    let clk = gpioc.pc12.into_alternate();

    info!("");
    info!("stm32h7xx-hal example - USART Advanced");
    info!("");

    // Configure the serial peripheral in synchronous mode
    let config = Config::new(115_200.bps()).lastbitclockpulse(true);
    let serial = dp
        .USART3
        .serial((tx, rx, clk), config, ccdr.peripheral.USART3, &ccdr.clocks)
        .unwrap();

    let (mut tx, _rx) = serial.split();

    loop {
        // core::fmt::Write is implemented for tx.
        writeln!(tx, "Hello, world!").unwrap();
    }
}
