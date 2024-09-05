#![deny(warnings)]
#![no_main]
#![no_std]

use cortex_m::asm;
use cortex_m_rt::entry;
use stm32h7xx_hal::hal_02::Direction;
#[macro_use]
mod utilities;
use stm32h7xx_hal::{pac, prelude::*};

use stm32h7xx_hal::traits::DacOut;

use log::info;

#[entry]
fn main() -> ! {
    utilities::logger::init();
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().expect("Cannot take peripherals");

    // Constrain and Freeze power
    info!("Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    let pwrcfg = example_power!(pwr).freeze();

    // Constrain and Freeze clock
    info!("Setup RCC...                  ");
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sys_ck(8.MHz()).freeze(pwrcfg, &dp.SYSCFG);

    let mut delay = cp.SYST.delay(ccdr.clocks);

    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);

    #[cfg(not(feature = "rm0455"))]
    let dac = dp.DAC.dac(gpioa.pa4, ccdr.peripheral.DAC12);
    #[cfg(feature = "rm0455")]
    let dac = dp.DAC1.dac(gpioa.pa4, ccdr.peripheral.DAC1);

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
