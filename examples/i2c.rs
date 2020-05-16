#![no_main]
#![no_std]

extern crate panic_itm;

use stm32h7xx_hal::{pac, prelude::*};

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
    let ccdr = rcc.sys_ck(100.mhz()).freeze(vos, &dp.SYSCFG);
    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);

    // Configure the SCL and the SDA pin for our I2C bus
    let scl = gpiob.pb8.into_alternate_af4().set_open_drain();
    let sda = gpiob.pb9.into_alternate_af4().set_open_drain();

    let mut i2c =
        dp.I2C1
            .i2c((scl, sda), 100.khz(), ccdr.peripheral.I2C1, &ccdr.clocks);

    // Echo what is received on the I2C at register 0x60
    let mut buf = [0x60];
    loop {
        buf[0] = 0x11;
        i2c.write_read(0x76, &buf.clone(), &mut buf).unwrap();
    }
}
