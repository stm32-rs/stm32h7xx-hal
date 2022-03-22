#![deny(warnings)]
#![no_main]
#![no_std]

use cortex_m_rt::entry;
#[macro_use]
mod utilities;
use stm32h7xx_hal::{pac, prelude::*, spi};

use log::info;

use nb::block;

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
        .sys_ck(96.mhz())
        .pll1_q_ck(48.mhz())
        .freeze(pwrcfg, &dp.SYSCFG);

    // Acquire the GPIOC peripheral. This also enables the clock for
    // GPIOC in the RCC register.
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);

    let sck = gpioc.pc10.into_alternate();
    let miso = gpioc.pc11.into_alternate();
    let mosi = gpioc.pc12.into_alternate();

    info!("");
    info!("stm32h7xx-hal example - SPI");
    info!("");

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
