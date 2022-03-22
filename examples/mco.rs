#![deny(warnings)]
#![no_main]
#![no_std]

use cortex_m_rt::entry;
#[macro_use]
mod utilities;
use stm32h7xx_hal::{gpio::Speed, pac, prelude::*, rcc::PllConfigStrategy};

use log::info;

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
    let ccdr = rcc
        .sys_ck(100.mhz())
        .mco1_from_hsi48(24.mhz())
        .pll2_strategy(PllConfigStrategy::Iterative)
        .mco2_from_pll2_p_ck(25_600.khz())
        .freeze(pwrcfg, &dp.SYSCFG);

    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);

    let _mco1_pin = gpioa.pa8.into_alternate::<0>();
    let _mco2_pin = gpioc.pc9.into_alternate::<0>().set_speed(Speed::High);

    info!("");
    info!("stm32h7xx-hal example - MCO output");
    info!("");

    // SYS_CK
    info!("sys_ck = {} MHz", ccdr.clocks.sys_ck().0 as f32 / 1e6);
    assert_eq!(ccdr.clocks.sys_ck().0, 100_000_000);

    // MCO
    info!(
        "mco1 = {} MHz",
        ccdr.clocks.mco1_ck().unwrap().0 as f32 / 1e6
    );
    assert_eq!(ccdr.clocks.mco1_ck().unwrap().0, 24_000_000);

    // MCO
    info!(
        "mco2 = {} MHz",
        ccdr.clocks.mco2_ck().unwrap().0 as f32 / 1e6
    );
    assert_eq!(ccdr.clocks.mco2_ck().unwrap().0, 25_600_000);

    info!("MCO outputs running!");

    loop {
        cortex_m::asm::nop()
    }
}
