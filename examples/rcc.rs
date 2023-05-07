#![deny(warnings)]
#![no_main]
#![no_std]

#[macro_use]
mod utilities;

use log::info;

use cortex_m_rt::entry;
use stm32h7xx_hal::{pac, prelude::*};

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
    let ccdr = rcc.sys_ck(100.MHz()).freeze(pwrcfg, &dp.SYSCFG);

    info!("");
    info!("stm32h7xx-hal example - RCC");
    info!("");

    // HCLK
    info!("hclk = {} MHz", ccdr.clocks.hclk().raw() as f32 / 1e6);
    assert_eq!(ccdr.clocks.hclk().raw(), 50_000_000);

    // SYS_CK
    info!("sys_ck = {} MHz", ccdr.clocks.sys_ck().raw() as f32 / 1e6);
    assert_eq!(ccdr.clocks.sys_ck().raw(), 100_000_000);

    loop {
        cortex_m::asm::nop()
    }
}
