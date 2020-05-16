#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate panic_itm;

use cortex_m;
use cortex_m::asm;
use cortex_m_rt::entry;
use stm32h7xx_hal::{pac, prelude::*};

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

    // Acquire the GPIOE peripheral. This also enables the clock for
    // GPIOE in the RCC register.
    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);

    // Select PWM output pins
    let pins = (
        gpioa.pa8.into_alternate_af1(),
        gpioa.pa9.into_alternate_af1(),
        gpioa.pa10.into_alternate_af1(),
    );

    println!(log, "");
    println!(log, "stm32h7xx-hal example - PWM");
    println!(log, "");

    // Configure PWM at 10kHz
    let (mut pwm, ..) =
        dp.TIM1
            .pwm(pins, 10.khz(), ccdr.peripheral.TIM1, &ccdr.clocks);

    // Output PWM on PA8
    let max = pwm.get_max_duty();
    pwm.set_duty(max / 2);

    println!(log, "50%");
    pwm.enable();
    asm::bkpt();

    println!(log, "25%");
    pwm.set_duty(max / 4);
    asm::bkpt();

    println!(log, "12.5%");
    pwm.set_duty(max / 8);
    asm::bkpt();

    println!(log, "100%");
    pwm.set_duty(max);
    asm::bkpt();

    loop {}
}
