#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate panic_itm;

use rprintln as println;
use rtt_target::{rprintln, rtt_init_print};

use cortex_m_rt::entry;
use stm32h7xx_hal::rcc;
use stm32h7xx_hal::{gpio::Speed, pac, prelude::*};

#[entry]
fn main() -> ! {
    rtt_init_print!();
    let dp = pac::Peripherals::take().unwrap();

    // Constrain and Freeze power
    println!("Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    let vos = pwr.freeze();

    // Constrain and Freeze clock
    println!("Setup RCC...                  ");
    let rcc = dp.RCC.constrain();
    let ccdr = rcc
        .use_hse(25.mhz())
        .sys_ck(400.mhz())
        .pll2_strategy(rcc::PllConfigStrategy::FractionalNotLess)
        .pll2_p_ck(12_288_000.hz())
        .pll2_q_ck(6_144_000.hz())
        .pll2_r_ck((48_000 * 63).hz())
        // pll2_p / 2 --> mco2
        .mco2_from_pll2_p_ck(7.mhz())
        .freeze(vos, &dp.SYSCFG);

    // Enable MCO2 output pin
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
    let _mco2_pin = gpioc.pc9.into_alternate_af0().set_speed(Speed::High);

    println!("");
    println!("stm32h7xx-hal example - Fractional PLL");
    println!("");

    // SYS_CK
    println!("sys_ck = {} MHz", ccdr.clocks.sys_ck().0 as f32 / 1e6);
    assert_eq!(ccdr.clocks.sys_ck().0, 400_000_000);

    println!("pll2_p_ck = {} MHz", ccdr.clocks.pll2_p_ck().unwrap().0);
    println!("pll2_q_ck = {} MHz", ccdr.clocks.pll2_q_ck().unwrap().0);
    println!("pll2_r_ck = {} MHz", ccdr.clocks.pll2_r_ck().unwrap().0);

    let _mco2_ck = ccdr.clocks.mco2_ck().unwrap().0;

    loop {}
}
