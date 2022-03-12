//! This example shows off how the hardware chip select is implemented in the HAL.
//!
//! There are 4 modes:
//! - Disabled (default)
//! - EndlessTransaction
//! - WordTransaction
//! - FrameTransaction
//!
//! For more docs, see https://docs.rs/stm32h7xx-hal/latest/stm32h7xx_hal/spi/index.html
//!

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

    // Acquire the GPIOA peripheral. This also enables the clock for
    // GPIOA in the RCC register.
    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);

    let sck = gpioa.pa5.into_alternate::<5>();
    let miso = gpioa.pa6.into_alternate();
    let mosi = gpioa.pa7.into_alternate();
    // Because we want to use the hardware chip select, we need to provide that too
    let hcs = gpioa.pa4.into_alternate();

    info!("");
    info!("stm32h7xx-hal example - SPI");
    info!("");

    // Initialise the SPI peripheral.
    let mut spi = dp.SPI1.spi(
        // Give ownership of the pins
        (sck, miso, mosi, hcs),
        // Create a config with the hardware chip select given
        spi::Config::new(spi::MODE_0)
            // Put 1 us idle time between every word sent. (the max is 15 spi peripheral ticks)
            .inter_word_delay(0.000001)
            // Specify that we use the hardware cs
            .hardware_cs(spi::HardwareCS {
                // See the docs of the HardwareCSMode to see what the different modes do
                mode: spi::HardwareCSMode::EndlessTransaction,
                // Put 1 us between the CS being asserted and the first clock
                assertion_delay: 0.000001,
                // Our CS should be high when not active and low when asserted
                polarity: spi::Polarity::IdleHigh,
            }),
        3.mhz(),
        ccdr.peripheral.SPI1,
        &ccdr.clocks,
    );

    // Write fixed data.
    // The CS will automatically be pulled low before the data is sent.
    // Because the SPI is in EndlessTransaction mode, the CS will never be de-asserted.
    spi.write(&[0x11u8, 0x22, 0x33]).unwrap();

    // The CS can be de-asserted manually, though
    spi.end_transaction().unwrap();

    // Echo what is received on the SPI
    let mut received = 0;
    loop {
        block!(spi.send(received)).ok();
        received = block!(spi.read()).unwrap();
    }
}
