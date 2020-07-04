#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate panic_itm;

use cortex_m;
use cortex_m_rt::entry;
use stm32h7xx_hal::{gpio::Speed, pac, prelude::*, rcc::PllConfigStrategy};

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
    let ccdr = rcc
        .sys_ck(100.mhz())
        .mco1_from_hsi48(24.mhz())
        .pll2_strategy(PllConfigStrategy::Iterative)
        .mco2_from_pll2_p_ck(25_600.khz())
        .freeze(vos, &dp.SYSCFG);

    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);

    let _mco1_pin = gpioa.pa8.into_alternate_af0();
    let _mco2_pin = gpioc.pc9.into_alternate_af0().set_speed(Speed::High);

    println!(log, "");
    println!(log, "stm32h7xx-hal example - MCO output");
    println!(log, "");

    // SYS_CK
    println!(log, "sys_ck = {} MHz", ccdr.clocks.sys_ck().0 as f32 / 1e6);
    assert_eq!(ccdr.clocks.sys_ck().0, 100_000_000);

    // MCO
    println!(
        log,
        "mco1 = {} MHz",
        ccdr.clocks.mco1_ck().unwrap().0 as f32 / 1e6
    );
    assert_eq!(ccdr.clocks.mco1_ck().unwrap().0, 24_000_000);

    // MCO
    println!(
        log,
        "mco2 = {} MHz",
        ccdr.clocks.mco2_ck().unwrap().0 as f32 / 1e6
    );
    assert_eq!(ccdr.clocks.mco2_ck().unwrap().0, 25_600_000);

    println!(log, "MCO outputs running!");

    loop {}
}
