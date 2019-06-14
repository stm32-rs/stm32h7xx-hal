#![no_main]
#![no_std]

extern crate panic_itm;

use stm32h7xx_hal::{
    prelude::*,
    pac,
    i2c::I2c,
};

use cortex_m_rt::entry;

use cortex_m_log::println;
use cortex_m_log::{
    destination::Itm, printer::itm::InterruptSync as InterruptSyncItm,
};

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
    let mut ccdr = rcc.sys_ck(100.mhz()).freeze(vos, &dp.SYSCFG);
    let gpiob = dp.GPIOB.split(&mut ccdr.ahb4);

    // Configure the SCL and the SDA pin for our I2C bus
    let scl = gpiob.pb8.into_alternate_af4().set_open_drain();
    let sda = gpiob.pb9.into_alternate_af4().set_open_drain();

    let mut i2c = I2c::i2c1(
        dp.I2C1,
        (scl, sda),
        100.khz(),
        &ccdr
    );

    // Echo what is received on the I2C at register 0x11 and addr 0x10
    // It is expected to get 2 bytes as response here
    let mut buf = [0x11, 0x0];
    loop {
        buf[0] = 0x11;
        i2c.write_read(0x10, &buf.clone(), &mut buf).ok();
    }
}