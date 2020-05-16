#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate panic_itm;

use cortex_m;
use cortex_m_rt::entry;
use stm32h7xx_hal::{pac, prelude::*, spi};

use cortex_m_log::println;
use cortex_m_log::{
    destination::Itm, printer::itm::InterruptSync as InterruptSyncItm,
};

use nb::block;

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
        .sys_ck(96.mhz())
        .pll1_q_ck(48.mhz())
        .freeze(vos, &dp.SYSCFG);

    // Acquire the GPIOC peripheral. This also enables the clock for
    // GPIOC in the RCC register.
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);

    let sck = gpioc.pc10.into_alternate_af6();
    let miso = gpioc.pc11.into_alternate_af6();
    let mosi = gpioc.pc12.into_alternate_af6();

    println!(log, "");
    println!(log, "stm32h7xx-hal example - SPI");
    println!(log, "");

    // Initialise the SPI peripheral.
    let mut spi = dp.SPI3.spi(
        (sck, miso, mosi),
        spi::MODE_0,
        3.mhz(),
        ccdr.peripheral.SPI3,
        &ccdr.clocks,
    );

    // Write fixed data
    spi.write(&[0x11u8, 0x22, 0x33]).unwrap();

    // Echo what is received on the SPI
    let mut received = 0;
    loop {
        block!(spi.send(received)).ok();
        received = block!(spi.read()).unwrap();
    }
}
