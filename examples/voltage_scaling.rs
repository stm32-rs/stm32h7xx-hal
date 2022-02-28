#![deny(warnings)]
#![no_main]
#![no_std]

use cortex_m_rt::entry;
#[macro_use]
mod utilities;
use stm32h7xx_hal::{pac, prelude::*};

use log::info;

#[entry]
fn main() -> ! {
    utilities::logger::init();
    let dp = pac::Peripherals::take().expect("Cannot take peripherals");

    // Constrain and Freeze power
    info!("Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    // By default the voltage scaling mode is VOS1, but here we select VOS3 to
    // save power at low clock speeds.
    let pwrcfg = example_power!(pwr).vos3().freeze();

    // Constrain and Freeze clock
    info!("Setup RCC...                  ");
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sys_ck(8.mhz()).freeze(pwrcfg, &dp.SYSCFG);

    info!("");
    info!("stm32h7xx-hal example - VOS3");
    info!("");

    // HCLK
    info!("hclk = {} MHz", ccdr.clocks.hclk().0 as f32 / 1e6);
    assert_eq!(ccdr.clocks.hclk().0, 4_000_000);

    // SYS_CK
    info!("sys_ck = {} MHz", ccdr.clocks.sys_ck().0 as f32 / 1e6);
    assert_eq!(ccdr.clocks.sys_ck().0, 8_000_000);

    loop {
        cortex_m::asm::nop()
    }
}
