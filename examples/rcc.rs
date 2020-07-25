#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

use log::info;

use cortex_m_rt::entry;
use stm32h7xx_hal::{pac, prelude::*};

#[path = "utilities/logger.rs"]
mod logger;

#[entry]
fn main() -> ! {
    logger::init();
    let dp = pac::Peripherals::take().unwrap();

    // Constrain and Freeze power
    info!("Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    let vos = pwr.freeze();

    // Constrain and Freeze clock
    info!("Setup RCC...                  ");
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sys_ck(100.mhz()).freeze(vos, &dp.SYSCFG);

    info!("");
    info!("stm32h7xx-hal example - RCC");
    info!("");

    // HCLK
    info!("hclk = {} MHz", ccdr.clocks.hclk().0 as f32 / 1e6);
    assert_eq!(ccdr.clocks.hclk().0, 50_000_000);

    // SYS_CK
    info!("sys_ck = {} MHz", ccdr.clocks.sys_ck().0 as f32 / 1e6);
    assert_eq!(ccdr.clocks.sys_ck().0, 100_000_000);

    loop {}
}
