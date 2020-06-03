#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate panic_itm;

use cortex_m;
use cortex_m::asm;
use cortex_m_rt::entry;
use stm32h7xx_hal::hal::Direction;
use stm32h7xx_hal::{pac, prelude::*};

use stm32h7xx_hal::traits::DacOut;

use cortex_m_log::println;
use cortex_m_log::{
    destination::Itm, printer::itm::InterruptSync as InterruptSyncItm,
};

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().expect("Cannot take peripherals");
    let mut log = InterruptSyncItm::new(Itm::new(cp.ITM));

    // Constrain and Freeze power
    println!(log, "Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    let vos = pwr.freeze();

    // Constrain and Freeze clock
    println!(log, "Setup RCC...                  ");
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sys_ck(8.mhz()).freeze(vos, &dp.SYSCFG);

    let mut delay = cp.SYST.delay(ccdr.clocks);

    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    let dac = dp.DAC.dac(gpioa.pa4, ccdr.peripheral.DAC12);

    // Calibrate output buffer then enable DAC channel
    let mut dac = dac.calibrate_buffer(&mut delay).enable();

    let mut dir = Direction::Upcounting;
    let mut val = 0;

    dac.set_value(2058);
    asm::bkpt();

    dac.set_value(4095);
    asm::bkpt();

    loop {
        dac.set_value(val);
        match val {
            0 => dir = Direction::Upcounting,
            4095 => dir = Direction::Downcounting,
            _ => (),
        };

        match dir {
            Direction::Upcounting => val += 1,
            Direction::Downcounting => val -= 1,
        }
    }
}
