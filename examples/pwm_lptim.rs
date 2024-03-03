//! PWM output using a LPTIM Low Power Timer

#![deny(warnings)]
#![no_main]
#![no_std]

use cortex_m::asm;
use cortex_m_rt::entry;
#[macro_use]
mod utilities;

use embedded_hal::pwm::*;
use stm32h7xx_hal::{pac, prelude::*, rcc::rec};

use log::info;

#[entry]
fn main() -> ! {
    utilities::logger::init();
    let dp = pac::Peripherals::take().expect("Cannot take peripherals");

    // Constrain and Freeze power
    info!("Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    let pwrcfg = example_power!(pwr).freeze();

    // Constrain and Freeze clock
    info!("Setup RCC...                  ");
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sys_ck(8.MHz()).freeze(pwrcfg, &dp.SYSCFG);

    // Acquire the GPIOB peripheral. This also enables the clock for
    // GPIOB in the RCC register.
    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);

    info!("");
    info!("stm32h7xx-hal example - PWM from LPTIM");
    info!("");

    // Configure PWM at 1kHz, from LSI
    let mut pwm = dp.LPTIM2.pwm(
        gpiob.pb13.into_alternate(),
        1.kHz(),
        ccdr.peripheral
            .LPTIM2
            .kernel_clk_mux(rec::Lptim2ClkSel::Lsi),
        &ccdr.clocks,
    );
    pwm.enable().unwrap(); // must be enabled before use

    // Output PWM on PB13
    let max = pwm.max_duty_cycle();
    pwm.set_duty_cycle(max / 2).unwrap();

    info!("50%");
    pwm.enable().unwrap();
    asm::bkpt();

    info!("25%");
    pwm.set_duty_cycle(max / 4).unwrap();
    asm::bkpt();

    info!("12.5%");
    pwm.set_duty_cycle(max / 8).unwrap();
    asm::bkpt();

    info!("100%");
    pwm.set_duty_cycle(max).unwrap();
    asm::bkpt();

    loop {
        cortex_m::asm::nop()
    }
}
