#![deny(warnings)]
#![no_main]
#![no_std]

use cortex_m_rt::entry;
#[path = "utilities/logger.rs"]
mod logger;
use stm32h7xx_hal::{pac, prelude::*, rcc};

use log::info;

#[entry]
fn main() -> ! {
    logger::init();
    let dp = pac::Peripherals::take().expect("Cannot take peripherals");

    // Constrain and Freeze power
    info!("Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.vos0(&dp.SYSCFG).freeze();

    // Constrain and Freeze clock
    // The PllConfigStrategy::Normal strategy uses the medium range VCO which has a maximum of 420 Mhz
    // Switching to PllConfigStrategy::Iterative sets the VCO to wide range to allow this clock to reach 480 Mhz
    info!("Setup RCC...                  ");
    let rcc = dp.RCC.constrain();
    let ccdr = rcc
        .sys_ck(480.mhz())
        .pll1_strategy(rcc::PllConfigStrategy::Iterative)
        .freeze(pwrcfg, &dp.SYSCFG);

    info!("");
    info!("stm32h7xx-hal example - VOS0");
    info!("");

    // HCLK
    info!("hclk = {} MHz", ccdr.clocks.hclk().0 as f32 / 1e6);
    assert_eq!(ccdr.clocks.hclk().0, 240_000_000);

    // SYS_CK
    info!("sys_ck = {} MHz", ccdr.clocks.sys_ck().0 as f32 / 1e6);
    assert_eq!(ccdr.clocks.sys_ck().0, 480_000_000);

    loop {}
}
