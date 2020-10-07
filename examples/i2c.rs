#![no_main]
#![no_std]

#[path = "utilities/logger.rs"]
mod logger;
use stm32h7xx_hal::{pac, prelude::*};

use cortex_m_rt::entry;

use log::info;

#[entry]
fn main() -> ! {
    logger::init();
    let dp = pac::Peripherals::take().unwrap();

    // Constrain and Freeze power
    info!("Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.freeze();

    // Constrain and Freeze clock
    info!("Setup RCC...                  ");
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sys_ck(100u32.MHz()).freeze(pwrcfg, &dp.SYSCFG);
    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);

    // Configure the SCL and the SDA pin for our I2C bus
    let scl = gpiob.pb8.into_alternate_af4().set_open_drain();
    let sda = gpiob.pb9.into_alternate_af4().set_open_drain();

    let mut i2c = dp.I2C1.i2c(
        (scl, sda),
        100u32.kHz(),
        ccdr.peripheral.I2C1,
        &ccdr.clocks,
    );

    // Echo what is received on the I2C at register 0x60
    let mut buf = [0x60];
    loop {
        buf[0] = 0x11;
        i2c.write_read(0x76, &buf.clone(), &mut buf).unwrap();
    }
}
