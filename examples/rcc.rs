#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate panic_itm;

use cortex_m_rt::entry;
use stm32h7xx_hal::{pac, prelude::*};

use cortex_m_log::println;
use cortex_m_log::{
    destination::Itm, printer::itm::InterruptSync as InterruptSyncItm,
};

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();
    let mut log = InterruptSyncItm::new(Itm::new(cp.ITM));

    // Constrain and Freeze power
    println!(log, "Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    let vos = pwr.freeze();

    // Constrain and Freeze clock
    println!(log, "Setup RCC...                  ");
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sys_ck(100.mhz()).freeze(vos, &dp.SYSCFG);

    println!(log, "");
    println!(log, "stm32h7xx-hal example - RCC");
    println!(log, "");

    // HCLK
    println!(log, "hclk = {} MHz", ccdr.clocks.hclk().0 as f32 / 1e6);
    assert_eq!(ccdr.clocks.hclk().0, 50_000_000);

    // SYS_CK
    println!(log, "sys_ck = {} MHz", ccdr.clocks.sys_ck().0 as f32 / 1e6);
    assert_eq!(ccdr.clocks.sys_ck().0, 100_000_000);

    loop {}
}
