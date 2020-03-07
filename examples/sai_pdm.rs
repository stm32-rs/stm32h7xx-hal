#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate panic_itm;

use cortex_m;
use cortex_m_rt::entry;
use stm32h7xx_hal::{pac, prelude::*};

use cortex_m_log::println;
use cortex_m_log::{
    destination::Itm, printer::itm::InterruptSync as InterruptSyncItm,
};

use nb::block;

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().expect("Cannot take peripherals");
    let mut log = InterruptSyncItm::new(Itm::new(cp.ITM));

    // Constrain and Freeze power
    println!(log, "Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    let vos = pwr.freeze();

    // Constrain and Freeze clock
    println!(log, "Setup RCC...                  ");
    let rcc = dp.RCC.constrain();
    let mut ccdr = rcc
        .sys_ck(100.mhz())
        .pll1_q_ck(20_480.khz()) // SAI1 clock source
        .freeze(vos, &dp.SYSCFG);

    // Acquire GPIO peripherals. This also enables the clock for each
    // GPIO in the RCC register.
    let gpioc = dp.GPIOC.split(&mut ccdr.ahb4);
    let gpioe = dp.GPIOE.split(&mut ccdr.ahb4);

    let d1 = gpioc.pc1.into_alternate_af2();
    let ck1 = gpioe.pe2.into_alternate_af2();
    let pins = (ck1, d1);

    println!(log, "");
    println!(log, "stm32h7xx-hal example - SAI PDM");
    println!(log, "");

    // Configure SAI for PDM mode
    let mut sai = dp.SAI1.pdm(pins, 1_024.khz(), &mut ccdr);

    loop {
        println!(log, "0x{:04x}", 0xFFFF & block!(sai.read_data()).unwrap());
    }
}
