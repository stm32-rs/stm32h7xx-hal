//! PWM example using 'advanced' features
//!
//! This was tested on a NUCLEO-H743ZI2 board (MB1364) with the following connections:
//!
//! LD3 (PB14) - TIM12 PWM output
//! LD2 (PE1) - GPIO indicating fault status of TIM1
//! USER button (PC13) - active high button, with pull-down enabled to set idle state; press button to clear TIM1 fault
//! TIM1 fault input (PE15) - connect jumper wire to 3.3V for no fault (PWM enabled), to GND for fault (PWM disabled)
//! TIM1 PWM outputs (PE8-PE14) - connect to LEDs or oscilloscope, 0.477Hz PWM output with a variety of complementary and polarity settings
//!
//! If you start running with PE15 connected to VDD, you should see PWM on PE8-PE14.
//! If you momentarily pull PE15 low, LD2 will turn on and you will get no PWM on PE8-PE14, even if you pull PE15 high again
//! If PE15 is high and you press the USER button, TIM1 PWM will resume on PE8-PE14

#![deny(warnings)]
#![no_main]
#![no_std]

use cortex_m_rt::entry;
#[macro_use]
mod utilities;
use stm32h7xx_hal::hal::digital::v2::{InputPin, OutputPin};
use stm32h7xx_hal::pwm::FaultMonitor;
use stm32h7xx_hal::pwm::Polarity;
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
    let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);
    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);

    // Set up user button GPIO
    let button = gpioc.pc13.into_pull_down_input();

    // Select TIM1 PWM output pins
    let pins = (
        gpioe.pe9.into_alternate_af1(),
        gpioe.pe11.into_alternate_af1(),
        gpioe.pe13.into_alternate_af1(),
        gpioe.pe14.into_alternate_af1(),
    );

    let fault_pin = gpioe.pe15.into_alternate_af1();

    info!("");
    info!("stm32h7xx-hal example - PWM");
    info!("");

    // Configure TIM1 PWM
    let (mut t1control, (t1c1, t1c2, t1c3, t1c4)) = dp
        .TIM1
        .pwm_advanced(pins, ccdr.peripheral.TIM1, &ccdr.clocks)
        .prescaler(256)
        .period(65535)
        .with_deadtime(500.ns())
        .with_break_pin(fault_pin, Polarity::ActiveLow)
        .center_aligned()
        .finalize();

    let mut t1c1 = t1c1.into_complementary(gpioe.pe8.into_alternate_af1());
    let mut t1c2 = t1c2
        .into_complementary(gpioe.pe10.into_alternate_af1())
        .into_active_low();
    let mut t1c3 = t1c3
        .into_complementary(gpioe.pe12.into_alternate_af1())
        .into_comp_active_low();
    let mut t1c4 = t1c4.into_active_low();

    // Output PWM on PA8
    let max = t1c1.get_max_duty();
    t1c1.set_duty(max / 2);
    t1c2.set_duty(max / 8 * 7);
    t1c3.set_duty(max / 8);
    t1c4.set_duty(max / 2);

    info!("PE8-14 enabled");
    t1c1.enable();
    t1c2.enable();
    t1c3.enable();
    t1c4.enable();
    //asm::bkpt();

    // Configure TIM12 PWM
    let mut pwm = dp.TIM12.pwm(
        gpiob.pb14.into_alternate_af2(),
        10.khz(),
        ccdr.peripheral.TIM12,
        &ccdr.clocks,
    );

    // Output PWM on PB14
    let max = pwm.get_max_duty();
    pwm.set_duty(max / 2);
    pwm.enable();

    // Configure LD2 LED
    let mut ld2 = gpioe.pe1.into_push_pull_output();

    loop {
        if t1control.is_fault_active() {
            // Fault is active, turn on LED
            ld2.set_high().unwrap();
        } else {
            // Fault is not active
            ld2.set_low().unwrap();
        }

        if button.is_high().unwrap() {
            t1control.clear_fault();
        }
    }
}
