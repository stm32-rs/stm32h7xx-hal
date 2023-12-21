//! Example that identifes the reset source
//!
//! Tested on a NUCLEO-H7A3ZI-Q board
#![deny(warnings)]
#![no_main]
#![no_std]

use cortex_m_rt::entry;
#[macro_use]
mod utilities;

use embedded_hal::delay::DelayNs;
use stm32h7xx_hal::{pac, prelude::*, rcc::ResetReason};

use log::info;

#[entry]
fn main() -> ! {
    utilities::logger::init();
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    // Constrain and Freeze power
    info!("Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    let pwrcfg = example_power!(pwr).freeze();

    // Constrain and Freeze clock
    info!("Setup RCC...                  ");
    let mut rcc = dp.RCC.constrain();
    let reason = rcc.get_reset_reason();
    let ccdr = rcc.sys_ck(100.MHz()).freeze(pwrcfg, &dp.SYSCFG);

    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);

    info!("");
    info!("stm32h7xx-hal example - Reset Reason");
    info!("");

    info!("The reset reason was: {}", reason);

    // Indicate what caused the last reset
    let mut pin = match reason {
        ResetReason::PowerOnReset => gpiob.pb14.into_push_pull_output().erase(),
        ResetReason::PinReset => gpioe.pe1.into_push_pull_output().erase(),
        _ => gpiob.pb0.into_push_pull_output().erase(),
    };
    pin.set_high();

    // delay 10s
    let mut delay = cp.SYST.delay(ccdr.clocks);
    delay.delay_ms(10_000);

    // system reset
    stm32h7xx_hal::pac::SCB::sys_reset()

    // the example will restart, and pb0 will go high
}
