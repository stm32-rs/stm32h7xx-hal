#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate panic_itm;

use rprintln as println;
use rtt_target::{rprintln, rtt_init_print};

use cortex_m_rt::entry;
use stm32h7xx_hal::rcc;
use stm32h7xx_hal::{pac, prelude::*};

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
        .sys_ck(400.mhz())
        .pll3_strategy(rcc::PllConfigStrategy::FractionalNotLess)
        .pll3_p_ck((48_000 * 256).hz())
        .pll3_q_ck((48_000 * 128).hz())
        .pll3_r_ck((48_000 * 63).hz())
        .freeze(vos, &dp.SYSCFG);

    println!("");
    println!("stm32h7xx-hal example - Fractional PLL");
    println!("");

    // SYS_CK
    println!("sys_ck = {} MHz", ccdr.clocks.sys_ck().0 as f32 / 1e6);
    assert_eq!(ccdr.clocks.sys_ck().0, 400_000_000);

    println!(
        "pll3_p_ck = {} MHz",
        ccdr.clocks.pll3_p_ck().unwrap().0
    );
    println!(
        "pll3_q_ck = {} MHz",
        ccdr.clocks.pll3_q_ck().unwrap().0
    );
    println!(
        "pll3_r_ck = {} MHz",
        ccdr.clocks.pll3_r_ck().unwrap().0
    );
    // assert_eq!(ccdr.clocks.sys_ck().0, 100_000_000);

    loop {}
}
