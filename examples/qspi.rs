#![deny(warnings)]
#![no_main]
#![no_std]

#[path = "utilities/logger.rs"]
mod logger;

use cortex_m_rt::entry;
use stm32h7xx_hal::{pac, prelude::*, qspi::QspiMode};

use log::info;

#[entry]
fn main() -> ! {
    logger::init();
    let dp = pac::Peripherals::take().unwrap();

    // Constrain and Freeze power
    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.freeze();

    // Constrain and Freeze clock
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sys_ck(96_u32.MHz()).freeze(pwrcfg, &dp.SYSCFG);

    // Acquire the GPIO peripherals. This also enables the clock for
    // the GPIOs in the RCC register.
    let gpiog = dp.GPIOG.split(ccdr.peripheral.GPIOG);
    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);
    let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);

    let _qspi_cs = gpiog.pg6.into_alternate_af10();

    let sck = gpiob.pb2.into_alternate_af9();
    let io0 = gpiod.pd11.into_alternate_af9();
    let io1 = gpiod.pd12.into_alternate_af9();
    let io2 = gpioe.pe2.into_alternate_af9();
    let io3 = gpiod.pd13.into_alternate_af9();

    info!("");
    info!("stm32h7xx-hal example - QSPI");
    info!("");

    // Initialise the QSPI peripheral.
    let mut qspi = dp.QUADSPI.bank1(
        (sck, io0, io1, io2, io3),
        3_u32.MHz(),
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
