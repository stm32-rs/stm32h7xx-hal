#![no_main]
#![no_std]

use cortex_m_rt::entry;
#[macro_use]
mod utilities;

use embedded_hal_1::i2c::*; // this example uses embedded-hal v1.0
use stm32h7xx_hal::{pac, prelude::*};

use log::info;

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
    let ccdr = rcc.sys_ck(100.MHz()).freeze(pwrcfg, &dp.SYSCFG);
    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);

    // Configure the SCL and the SDA pin for our I2C bus
    let scl = gpiob.pb8.into_alternate_open_drain();
    let sda = gpiob.pb9.into_alternate_open_drain();

    let mut i2c =
        dp.I2C1
            .i2c((scl, sda), 100.kHz(), ccdr.peripheral.I2C1, &ccdr.clocks);

    // Echo what is received on the I2C at register 0x60
    let mut buf = [0x60];
    loop {
        buf[0] = 0x11;
        i2c.write_read(0x76, &buf.clone(), &mut buf).unwrap();
    }
}
