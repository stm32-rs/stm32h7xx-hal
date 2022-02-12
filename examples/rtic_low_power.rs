//! Timers in RTIC with low power mode.
//!
//! After the end of init the CPU transitions into CStop mode, and D1/D2
//! (aka. CD) transition into DStop mode.
//!
//! However we set the run_d3/run_srd flag, and enable Autonomous mode on the
//! LPTIM3 PREC struture. Therefore LPTIM3 continues to run and fires an
//! interrupt that wakes the core. Following each interrupt the core returns to
//! CStop mode.
//!
//! On the first rising edge on PC13, the EXTI interrupt fires. We do not clear
//! this interrupt, so we loop in the handler forever.
//!
//! Nb. on dual core parts you will also need to ensure that CPU2 transitions to
//! DStop mode.
//!
#![deny(warnings)]
#![deny(unsafe_code)]
#![no_std]
#![no_main]

extern crate rtic;

#[macro_use]
mod utilities;

#[rtic::app(device = stm32h7xx_hal::stm32, peripherals = true)]
mod app {
    use stm32h7xx_hal::gpio::gpioc::{PC2, PC3, PC4};
    use stm32h7xx_hal::gpio::{Edge, ExtiPin, Output, PushPull};
    use stm32h7xx_hal::hal::digital::v2::{OutputPin, ToggleableOutputPin};
    use stm32h7xx_hal::prelude::*;
    use stm32h7xx_hal::rcc::LowPowerMode;
    use stm32h7xx_hal::stm32::{LPTIM3, TIM1, TIM2};
    use stm32h7xx_hal::timer::{Enabled, Event, LpTimer, Timer};

    use super::*;

    #[shared]
    struct SharedResources {}
    #[local]
    struct LocalResources {
        led1: PC2<Output<PushPull>>,
        led2: PC3<Output<PushPull>>,
        led3: PC4<Output<PushPull>>,
        timer1: Timer<TIM1>,
        timer2: Timer<TIM2>,
        timer3: LpTimer<LPTIM3, Enabled>,
    }

    #[init]
    fn init(
        ctx: init::Context,
    ) -> (SharedResources, LocalResources, init::Monotonics) {
        utilities::logger::init();
        let mut syscfg = ctx.device.SYSCFG;

        // Run D3 / SRD domain
        #[cfg(not(feature = "rm0455"))]
        ctx.device.PWR.cpucr.modify(|_, w| w.run_d3().set_bit());
        #[cfg(feature = "rm0455")] // 7b3/7a3/7b0 parts
        ctx.device.PWR.cpucr.modify(|_, w| w.run_srd().set_bit());

        let pwr = ctx.device.PWR.constrain();
        let vos = example_power!(pwr).freeze();

        // RCC
        let rcc = ctx.device.RCC.constrain();
        let ccdr = rcc
            // D3 / SRD domain
            .hclk(16.mhz()) // rcc_hclk4
            .pclk4(4.mhz()) // rcc_pclk4
            .freeze(vos, &syscfg);

        // Timers
        let mut timer1 = ctx.device.TIM1.timer(
            250.ms(),
            // Run in CSleep, but not CStop
            ccdr.peripheral.TIM1.low_power(LowPowerMode::Enabled),
            &ccdr.clocks,
        );
        timer1.listen(Event::TimeOut);

        let mut timer2 = ctx.device.TIM2.timer(
            500.ms(),
            // Run in CSleep, but not CStop
            ccdr.peripheral.TIM2.low_power(LowPowerMode::Enabled),
            &ccdr.clocks,
        );
        timer2.listen(Event::TimeOut);

        let mut timer3 = ctx
            .device
            .LPTIM3
            .timer(
                1000.ms(),
                // Run in LPTIM in D3 / SRD autonomous mode
                ccdr.peripheral.LPTIM3.low_power(LowPowerMode::Autonomous),
                &ccdr.clocks,
            )
            .pause();
        timer3.listen(Event::TimeOut);
        let timer3 = timer3.resume();

        // GPIO
        let gpioc = ctx.device.GPIOC.split(ccdr.peripheral.GPIOC);

        // Enter CStop mode
        let mut scb = ctx.core.SCB;
        scb.set_sleepdeep();

        // Wakeup
        let mut exti = ctx.device.EXTI;
        let mut wakeup = gpioc.pc13.into_floating_input();
        wakeup.make_interrupt_source(&mut syscfg);
        wakeup.trigger_on_edge(&mut exti, Edge::Rising);
        wakeup.enable_interrupt(&mut exti);

        // LEDs
        let mut led1 = gpioc.pc2.into_push_pull_output();
        let mut led2 = gpioc.pc3.into_push_pull_output();
        let mut led3 = gpioc.pc4.into_push_pull_output();

        led1.set_high().ok();
        led2.set_high().ok();
        led3.set_high().ok();

        (
            SharedResources {},
            LocalResources {
                led1,
                led2,
                led3,
                timer1,
                timer2,
                timer3,
            },
            init::Monotonics(),
        )
    }

    // RTIC inserts a default idle loop that calls asm::wfi()

    #[task(binds = EXTI15_10, local = [], priority = 1)]
    fn exti15_10_interrupt(_: exti15_10_interrupt::Context) {
        // Once the wakeup is triggered, we loop here forever
    }

    #[task(binds = TIM1_UP, local = [led1, timer1], priority = 2)]
    fn timer1_tick(ctx: timer1_tick::Context) {
        ctx.local.timer1.clear_irq();
        ctx.local.led1.toggle().unwrap();
    }

    #[task(binds = TIM2, local = [led2, timer2], priority = 2)]
    fn timer2_tick(ctx: timer2_tick::Context) {
        ctx.local.timer2.clear_irq();
        ctx.local.led2.toggle().unwrap();
    }

    #[task(binds = LPTIM3, local = [led3, timer3], priority = 2)]
    fn timer3_tick(ctx: timer3_tick::Context) {
        ctx.local.timer3.clear_irq();
        ctx.local.led3.toggle().unwrap();
    }
}
