#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate panic_itm;

use cortex_m;
use cortex_m_rt::entry;
use stm32h7xx_hal::{pac, prelude::*, rcc};

use cortex_m_log::println;
use cortex_m_log::{
    destination::Itm, printer::itm::InterruptSync as InterruptSyncItm,
};

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().expect("Cannot take peripherals");
    let mut log = InterruptSyncItm::new(Itm::new(cp.ITM));

    // Constrain and Freeze power
    println!(log, "Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    let vos;

    // Enable VOS0 for revision v parts
    #[cfg(feature = "revision_v")]
    { vos = pwr.vos0(&dp.SYSCFG).freeze() };

    // VOS0 is only support by revision v parts
    #[cfg(not(feature = "revision_v"))]
    { vos = pwr.freeze() };

    // Constrain and Freeze clock
    // The PllConfigStrategy::Normal strategy uses the medium range VCO which has a maximum of 420 Mhz
    // Switching to PllConfigStrategy::Iterative sets the VCO to wide range to allow this clock to reach 480 Mhz
    println!(log, "Setup RCC...                  ");
    let rcc = dp.RCC.constrain();
    let ccdr = rcc
        .sys_ck(480.mhz())
        .pll1_strategy(rcc::PllConfigStrategy::Iterative)
        .freeze(vos, &dp.SYSCFG);

    println!(log, "");
    println!(log, "stm32h7xx-hal example - VOS0");
    println!(log, "");

    // HCLK
    println!(log, "hclk = {} MHz", ccdr.clocks.hclk().0 as f32 / 1e6);
    assert_eq!(ccdr.clocks.hclk().0, 240_000_000);

    // SYS_CK
    println!(log, "sys_ck = {} MHz", ccdr.clocks.sys_ck().0 as f32 / 1e6);
    assert_eq!(ccdr.clocks.sys_ck().0, 480_000_000);

    loop {}
}
