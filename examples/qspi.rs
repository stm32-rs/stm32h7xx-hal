#![deny(warnings)]
#![no_main]
#![no_std]

#[macro_use]
mod utilities;

use cortex_m_rt::entry;
use stm32h7xx_hal::{pac, prelude::*, xspi::QspiMode};

use log::info;

#[entry]
fn main() -> ! {
    utilities::logger::init();
    let dp = pac::Peripherals::take().unwrap();

    // Constrain and Freeze power
    let pwr = dp.PWR.constrain();
    let pwrcfg = example_power!(pwr).freeze();

    // Constrain and Freeze clock
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sys_ck(96.mhz()).freeze(pwrcfg, &dp.SYSCFG);

    // Acquire the GPIO peripherals. This also enables the clock for
    // the GPIOs in the RCC register.
    let gpiog = dp.GPIOG.split(ccdr.peripheral.GPIOG);
    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);
    let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);

    let _qspi_cs = gpiog.pg6.into_alternate::<10>();

    let sck = gpiob.pb2.into_alternate();
    let io0 = gpiod.pd11.into_alternate();
    let io1 = gpiod.pd12.into_alternate();
    let io2 = gpioe.pe2.into_alternate();
    let io3 = gpiod.pd13.into_alternate();

    info!("");
    info!("stm32h7xx-hal example - QSPI");
    info!("");

    // Initialise the QSPI peripheral.
    let mut qspi = dp.QUADSPI.bank1(
        (sck, io0, io1, io2, io3),
        3.mhz(),
        &ccdr.clocks,
        ccdr.peripheral.QSPI,
    );

    qspi.configure_mode(QspiMode::FourBit).unwrap();

    loop {
        qspi.write(0x00, &[0xAA, 0x00, 0xFF]).unwrap();

        let mut read: [u8; 3] = [0; 3];
        qspi.read(0xFF, &mut read).unwrap();
    }
}
