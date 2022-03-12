//! PWM example using 'advanced' features
//!
//! This was tested on a NUCLEO-H743ZI2 board (MB1364) with the following IO:
//!
//! PE1 (LD2) - GPIO output - indicates fault status of TIM1
//! PC13 (USER button) - GPIO input - push to reset all TIM1,8,15,16,17 fault; active high button, with pull-down enabled to set idle state
//! PE15 - TIM1 BKIN  - active low  fault input (GND is fault)
//! PE9  - TIM1 CH1   - active high PWM 1Hz    50% duty center aligned
//! PE8  - TIM1 CH1N  - active high complementary 1ms deadtime
//! PE11 - TIM1 CH2   - active low  PWM 1Hz    87% duty center aligned
//! PE10 - TIM1 CH2N  - active high complementary 1ms deadtime
//! PE13 - TIM1 CH3   - active high PWM 1Hz    12% duty center aligned
//! PE12 - TIM1 CH3N  - active low  complementary 1ms deadtime
//! PE14 - TIM1 CH4   - active low  PWM 1Hz    50% duty center aligned
//! PA0  - TIM2 CH1   - active high PWM 50Hz   75% duty right aligned
//! PA1  - TIM2 CH2   - active high PWM 50Hz   50% duty right aligned
//! PA2  - TIM2 CH3   - active high PWM 50Hz   25% duty right aligned
//! PB11 - TIM2 CH4   - active low  PWM 50Hz   75% duty right aligned
//! PB5  - TIM3 CH2   - active high PWM 1kHz   33% duty center aligned
//! PB0  - TIM3 CH3   - active high PWM 1kHz   67% duty center aligned
//! PD15 - TIM4 CH4   - active low  PWM 2kHz   40% duty left aligned
//! PA3  - TIM5 CH4   - active high PWM 2.5kHz 80% duty center aligned
//! PG3  - TIM8 BKIN2 - active high fault input (3.3V is fault)
//! PC6  - TIM8 CH1   - active high PWM 4kHz   50% duty right aligned
//! PA5  - TIM8 CH1N  - active high complementary 2us deadtime
//! PC7  - TIM8 CH2   - active high PWM 4kHz   10% duty right aligned
//! PC8  - TIM8 CH3   - active high PWM 4kHz   25% duty right aligned
//! PB1  - TIM8 CH3N  - active high complementary 2us deadtime
//! PC9  - TIM8 CH4   - active low  PWM 4kHz   75% duty right aligned
//! PB14 (LD3) - TIM12 CH1 - active low PWM 5kHz 25% duty left aligned
//! PB15 - TIM12 CH2  - active high PWM 5kHz   40% duty left aligned
//! PF8  - TIM13 CH1  - active low  PWM 6kHz   30% duty left aligned
//! PA7  - TIM14 CH1  - active high PWM 7kHz   20% duty left aligned
//! PE3  - TIM15 BKIN - active low  fault input (GND is fault)
//! PE5  - TIM15 CH1  - active low  PWM 500kHz 25% duty left aligned
//! PE4  - TIM15 CH1N - active low  complementary no deadtime
//! PE6  - TIM15 CH2  - active high PWM 500kHz 50% duty left aligned
//! PF10 - TIM16 BKIN - active high fault input (3.3V is fault)
//! PF6  - TIM16 CH1  - active low  PWM 500kHz 25% duty left aligned
//! PF7  - TIM17 CH1  - active high PWM 500kHz 50% duty left aligned
//! PF9  - TIM17 CH1N - active high complementary 250ns deadtime
//!
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
use stm32h7xx_hal::pwm::{FaultMonitor, Polarity};
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
    let ccdr = rcc.sys_ck(8.mhz()).hclk(4.mhz()).freeze(pwrcfg, &dp.SYSCFG);

    // Acquire the GPIOA-GPIOG peripherals. This also enables the clocks for
    // GPIOA-GPIOG in the RCC register.
    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
    let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);
    let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);
    let gpiof = dp.GPIOF.split(ccdr.peripheral.GPIOF);
    let gpiog = dp.GPIOG.split(ccdr.peripheral.GPIOG);

    // Set up user button GPIO
    let button = gpioc.pc13.into_pull_down_input();

    // Configure LD2 LED
    let mut ld2 = gpioe.pe1.into_push_pull_output();
    info!("");
    info!("stm32h7xx-hal example - Advanced PWM");
    info!("");

    // Configure TIM1 PWM
    let t1builder = dp
        .TIM1
        .pwm_advanced(
            (
                gpioe.pe14.into_alternate(),
                gpioe.pe11.into_alternate(),
                gpioe.pe9.into_alternate(),
                gpioe.pe13.into_alternate(),
            ),
            ccdr.peripheral.TIM1,
            &ccdr.clocks,
        )
        .prescaler(39)
        .period(49_999)
        .with_deadtime(1.ms())
        .with_break_pin(gpioe.pe15.into_alternate(), Polarity::ActiveLow)
        .center_aligned();

    let (mut t1control, (t1c4, t1c2, t1c1, t1c3)) = t1builder.finalize();

    let mut t1c1 = t1c1.into_complementary(gpioe.pe8.into_alternate());
    let mut t1c2 = t1c2
        .into_complementary(gpioe.pe10.into_alternate())
        .into_active_low();
    let mut t1c3 = t1c3
        .into_complementary(gpioe.pe12.into_alternate())
        .into_comp_active_low();
    let mut t1c4 = t1c4.into_active_low();

    // Output TIM1 PWM
    let period = t1c1.get_max_duty();
    t1c1.set_duty(period / 2);
    t1c2.set_duty(period / 8 * 7);
    t1c3.set_duty(period / 8);
    t1c4.set_duty(period / 2);

    t1c1.enable();
    t1c2.enable();
    t1c3.enable();
    t1c4.enable();

    // Configure TIM2 PWM
    let (_t2control, (mut t2c2, mut t2c1, mut t2c3, t2c4)) = dp
        .TIM2
        .pwm_advanced(
            (
                gpioa.pa1.into_alternate(),
                gpioa.pa0.into_alternate(),
                gpioa.pa2.into_alternate(),
                gpiob.pb11.into_alternate(),
            ),
            ccdr.peripheral.TIM2,
            &ccdr.clocks,
        )
        .prescaler(0)
        .period(79_999)
        .right_aligned()
        .finalize();

    let mut t2c4 = t2c4.into_active_low();

    // Output TIM2 PWM
    let period = t2c1.get_max_duty();
    t2c1.set_duty(period / 4 * 3);
    t2c2.set_duty(period / 2);
    t2c3.set_duty(period / 4);
    t2c4.set_duty(period / 4 * 3);

    t2c1.enable();
    t2c2.enable();
    t2c3.enable();
    t2c4.enable();

    // Configure TIM3 PWM
    let (_t3control, (mut t3c3, mut t3c2)) = dp
        .TIM3
        .pwm_advanced(
            (gpiob.pb0.into_alternate(), gpiob.pb5.into_alternate()),
            ccdr.peripheral.TIM3,
            &ccdr.clocks,
        )
        .frequency(1_000.hz())
        .center_aligned()
        .finalize();

    // Output TIM3 PWM
    let period = t3c2.get_max_duty();
    t3c2.set_duty(period / 3);
    t3c3.set_duty(period / 3 * 2);

    t3c2.enable();
    t3c3.enable();

    // Configure TIM4 PWM
    let (_t4control, t4c4) = dp
        .TIM4
        .pwm_advanced(
            gpiod.pd15.into_alternate(),
            ccdr.peripheral.TIM4,
            &ccdr.clocks,
        )
        .frequency(2.khz())
        .finalize();

    let mut t4c4 = t4c4.into_active_low();

    // Output TIM4 PWM
    let period = t4c4.get_max_duty();
    t4c4.set_duty(period / 10 * 4);

    t4c4.enable();

    // Configure TIM5 PWM
    let t5builder = dp
        .TIM5
        .pwm_advanced(
            gpioa.pa3.into_alternate(),
            ccdr.peripheral.TIM5,
            &ccdr.clocks,
        )
        .frequency(2_500.hz())
        .center_aligned();

    let (_t5control, mut t5c4) = t5builder.finalize();

    // Output TIM5 PWM
    let period = t5c4.get_max_duty();
    t5c4.set_duty(period / 5 * 4);

    t5c4.enable();

    // Configure TIM8 PWM
    let (mut t8control, (t8c1, mut t8c2, t8c3, t8c4)) = dp
        .TIM8
        .pwm_advanced(
            (
                gpioc.pc6.into_alternate(),
                gpioc.pc7.into_alternate(),
                gpioc.pc8.into_alternate(),
                gpioc.pc9.into_alternate(),
            ),
            ccdr.peripheral.TIM8,
            &ccdr.clocks,
        )
        .frequency(4.khz())
        .with_deadtime(2.us())
        .with_break_pin(gpiog.pg3.into_alternate(), Polarity::ActiveHigh)
        .right_aligned()
        .finalize();

    let mut t8c1 = t8c1.into_complementary(gpioa.pa5.into_alternate());
    let mut t8c3 = t8c3.into_complementary(gpiob.pb1.into_alternate());
    let mut t8c4 = t8c4.into_active_low();

    // Output TIM8 PWM
    let period = t8c1.get_max_duty();
    t8c1.set_duty(period / 2);
    t8c2.set_duty(period / 10);
    t8c3.set_duty(period / 4);
    t8c4.set_duty(period / 4 * 3);

    t8c1.enable();
    t8c2.enable();
    t8c3.enable();
    t8c4.enable();

    // Configure TIM12 PWM
    let (mut t12c2, t12c1) = dp.TIM12.pwm(
        (gpiob.pb15.into_alternate(), gpiob.pb14.into_alternate()),
        5.khz(),
        ccdr.peripheral.TIM12,
        &ccdr.clocks,
    );

    let mut t12c1 = t12c1.into_active_low();

    // Output TIM12 PWM
    let period = t12c1.get_max_duty();
    t12c1.set_duty(period / 4);
    t12c2.set_duty(period / 10 * 4);

    t12c1.enable();
    t12c2.enable();

    // Configure TIM13 PWM
    let (_t13control, t13c1) = dp
        .TIM13
        .pwm_advanced(
            gpiof.pf8.into_alternate(),
            ccdr.peripheral.TIM13,
            &ccdr.clocks,
        )
        .frequency(6.khz())
        .finalize();

    let mut t13c1 = t13c1.into_active_low();

    // Output TIM13 PWM
    let period = t13c1.get_max_duty();
    t13c1.set_duty(period / 10 * 3);

    t13c1.enable();

    // Configure TIM14 PWM
    let (_t14control, mut t14c1) = dp
        .TIM14
        .pwm_advanced(
            gpioa.pa7.into_alternate(),
            ccdr.peripheral.TIM14,
            &ccdr.clocks,
        )
        .frequency(7000.hz())
        .finalize();

    // Output TIM14 PWM
    let period = t14c1.get_max_duty();
    t14c1.set_duty(period / 5);

    t14c1.enable();

    // Configure TIM15 PWM
    let (mut t15control, (t15c1, mut t15c2)) = dp
        .TIM15
        .pwm_advanced(
            (gpioe.pe5.into_alternate(), gpioe.pe6.into_alternate()),
            ccdr.peripheral.TIM15,
            &ccdr.clocks,
        )
        .frequency(500.khz())
        .with_break_pin(gpioe.pe3.into_alternate(), Polarity::ActiveLow)
        .left_aligned()
        .finalize();

    let mut t15c1 = t15c1
        .into_complementary(gpioe.pe4.into_alternate())
        .into_active_low()
        .into_comp_active_low();

    // Output TIM15 PWM
    let period = t15c1.get_max_duty();
    t15c1.set_duty(period / 4);
    t15c2.set_duty(period / 2);

    t15c1.enable();
    t15c2.enable();

    // Configure TIM16 PWM
    let (mut t16control, t16c1) = dp
        .TIM16
        .pwm_advanced(
            gpiof.pf6.into_alternate(),
            ccdr.peripheral.TIM16,
            &ccdr.clocks,
        )
        .frequency(500.khz())
        .with_break_pin(gpiof.pf10.into_alternate(), Polarity::ActiveHigh)
        .finalize();

    let mut t16c1 = t16c1.into_active_low();

    // Output TIM16 PWM
    let period = t16c1.get_max_duty();
    t16c1.set_duty(period / 4);

    t16c1.enable();

    // Configure TIM17 PWM
    let (_t17control, t17c1) = dp
        .TIM17
        .pwm_advanced(
            gpiof.pf7.into_alternate(),
            ccdr.peripheral.TIM17,
            &ccdr.clocks,
        )
        .frequency(500.khz())
        .with_deadtime(250.ns())
        .finalize();

    let mut t17c1 = t17c1.into_complementary(gpiof.pf9.into_alternate());

    // Output TIM16 PWM
    let period = t17c1.get_max_duty();
    t17c1.set_duty(period / 2);

    t17c1.enable();

    info!("");
    info!("PWM channels enabled; see examples/pwm_advanced.rs for list of channels, pins, and settings");
    info!("");

    loop {
        if t1control.is_fault_active() {
            // Fault is active, turn on LED
            ld2.set_high();
        } else {
            // Fault is not active
            ld2.set_low();
        }

        if button.is_high() {
            t1control.clear_fault();
            t8control.clear_fault();
            t15control.clear_fault();
            t16control.clear_fault();
        }
    }
}
