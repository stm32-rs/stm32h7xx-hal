//! This example shows off the FrameTransaction mode of hardware chip select functionality.
//!
//! For more docs, see https://docs.rs/stm32h7xx-hal/latest/stm32h7xx_hal/spi/index.html
//!

#![deny(warnings)]
#![no_main]
#![no_std]

use cortex_m_rt::entry;
#[macro_use]
mod utilities;
use core::num::NonZeroU16;
use spi::Spi;
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

    let sck = gpioa.pa5.into_alternate();
    let miso = gpioa.pa6.into_alternate();
    let mosi = gpioa.pa7.into_alternate();
    // Because we want to use the hardware chip select, we need to provide that too
    let hcs = gpioa.pa4.into_alternate();

    info!("");
    info!("stm32h7xx-hal example - SPI");
    info!("");

    // Initialise the SPI peripheral.
    let mut spi: Spi<_, _, u8> = dp.SPI1.spi(
        // Give ownership of the pins
        (sck, miso, mosi, hcs),
        // Create a config with the hardware chip select given
        spi::Config::new(spi::MODE_0)
            // Put 1 us idle time between every word sent
            .inter_word_delay(0.000001)
            // Specify that we use the hardware cs
            .hardware_cs(spi::HardwareCS {
                // See the docs of the HardwareCSMode to see what the different modes do
                mode: spi::HardwareCSMode::FrameTransaction,
                // Put 1 us between the CS being asserted and the first clock
                assertion_delay: 0.000001,
                // Our CS should be high when not active and low when asserted
                polarity: spi::Polarity::IdleHigh,
            }),
        3.mhz(),
        ccdr.peripheral.SPI1,
        &ccdr.clocks,
    );

    // We want to send a frame of three bytes.
    // This means that the CS should be asserted, three words should be sent and the CS should be de-asserted.
    // To do this, it takes a bit more management.

    // Prepare sending the frame
    spi.setup_transaction(NonZeroU16::new(3).unwrap()).unwrap();

    // Write fixed data.
    // The CS will automatically be pulled low before the data is sent and de-assert after the sending is done.
    for word in &[0x11u8, 0x22, 0x33] {
        block!(spi.send(*word)).unwrap();
    }

    // The way that the silicon works, we also need to clean up some flags.
    // If we don't, then the SPI will silently swallow any data you want to send.
    // So in this example that would have meant that if we also called send with a fourth byte,
    // that byte would've simply not been sent
    spi.end_transaction().unwrap();

    // When in this mode, the blocking embedded-hal interface already does this for us.
    // For example if we want to send two separate frames of different lengths, we can do this:
    spi.write(&[0, 1, 2]).unwrap();
    spi.write(&[0, 1, 2, 3, 4, 5, 6]).unwrap();

    loop {
        cortex_m::asm::nop();
    }
}
