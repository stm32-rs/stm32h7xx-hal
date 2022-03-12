#![deny(warnings)]
#![no_main]
#![no_std]

use cortex_m::asm;
use cortex_m_rt::entry;
#[macro_use]
mod utilities;
use stm32h7xx_hal::{pac, prelude::*};

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
    let ccdr = rcc.sys_ck(8.mhz()).freeze(pwrcfg, &dp.SYSCFG);

    // Acquire the GPIOE peripheral. This also enables the clock for
    // GPIOE in the RCC register.
    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);

    // Select PWM output pins
    let pins = (
        gpioa.pa8.into_alternate(),
        gpioa.pa9.into_alternate(),
        gpioa.pa10.into_alternate(),
    );

    info!("");
    info!("stm32h7xx-hal example - PWM");
    info!("");

    // Configure PWM at 10kHz
    let (mut pwm, ..) =
        dp.TIM1
            .pwm(pins, 10.khz(), ccdr.peripheral.TIM1, &ccdr.clocks);

    // Output PWM on PA8
    let max = pwm.get_max_duty();
    pwm.set_duty(max / 2);

    info!("50%");
    pwm.enable();
    asm::bkpt();

    info!("25%");
    pwm.set_duty(max / 4);
    asm::bkpt();

    info!("12.5%");
    pwm.set_duty(max / 8);
    asm::bkpt();

    info!("100%");
    pwm.set_duty(max);
    asm::bkpt();

    let mut pwm = dp.TIM12.pwm(
        gpiob.pb14.into_alternate(),
        10.khz(),
        ccdr.peripheral.TIM12,
        &ccdr.clocks,
    );

    // Output PWM on PB14
    let max = pwm.get_max_duty();
    pwm.set_duty(max / 2);
    pwm.enable();

    loop {
        cortex_m::asm::nop()
    }
}
