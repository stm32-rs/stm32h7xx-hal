#![deny(warnings)]
#![no_main]
#![no_std]

#[macro_use]
mod utilities;

use cortex_m_rt::entry;
use log::info;
use stm32h7xx_hal::rcc;
use stm32h7xx_hal::{gpio::Speed, pac, prelude::*};

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
        .sys_ck(100.MHz())
        .pll2_strategy(rcc::PllConfigStrategy::FractionalNotLess)
        .pll2_p_ck(12_288_000.Hz())
        .pll2_q_ck(6_144_000.Hz())
        .pll2_r_ck((48_000 * 63).Hz())
        // pll2_p / 2 --> mco2
        .mco2_from_pll2_p_ck(7.MHz())
        .freeze(pwrcfg, &dp.SYSCFG);

    // Enable MCO2 output pin
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
    let _mco2_pin = gpioc.pc9.into_alternate::<0>().set_speed(Speed::High);

    info!("");
    info!("stm32h7xx-hal example - Fractional PLL");
    info!("");

    // SYS_CK
    info!("sys_ck = {} MHz", ccdr.clocks.sys_ck().raw() as f32 / 1e6);
    assert_eq!(ccdr.clocks.sys_ck().raw(), 400_000_000);

    info!("pll2_p_ck = {}", ccdr.clocks.pll2_p_ck().unwrap());
    info!("pll2_q_ck = {}", ccdr.clocks.pll2_q_ck().unwrap());
    info!("pll2_r_ck = {}", ccdr.clocks.pll2_r_ck().unwrap());

    let _mco2_ck = ccdr.clocks.mco2_ck().unwrap().raw();

    loop {
        cortex_m::asm::nop()
    }
}
