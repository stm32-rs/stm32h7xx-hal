//! Timers using the Real-Time Interrupt-driven Concurrency (RTIC)
//! framework. Tests TIM1, TIM2, TIM12, TIM17
#![deny(warnings)]
#![deny(unsafe_code)]
#![no_std]
#![no_main]

#[macro_use]
mod utilities;

#[rtic::app(device = stm32h7xx_hal::stm32, peripherals = true)]
mod app {
    use stm32h7xx_hal::gpio::gpioi::{PI12, PI13, PI14, PI15};
    use stm32h7xx_hal::gpio::{Output, PushPull};
    use stm32h7xx_hal::prelude::*;
    use stm32h7xx_hal::stm32::{TIM1, TIM12, TIM17, TIM2};
    use stm32h7xx_hal::time::MilliSeconds;
    use stm32h7xx_hal::timer::{Event, Timer};

    use super::*;

    #[shared]
    struct SharedResources {}
    #[local]
    struct LocalResources {
        timer1: Timer<TIM1>,
        timer2: Timer<TIM2>,
        timer3: Timer<TIM12>,
        timer4: Timer<TIM17>,
        led1: PI12<Output<PushPull>>,
        led2: PI13<Output<PushPull>>,
        led3: PI14<Output<PushPull>>,
        led4: PI15<Output<PushPull>>,
    }

    #[init]
    fn init(ctx: init::Context) -> (SharedResources, LocalResources) {
        utilities::logger::init();
        let pwr = ctx.device.PWR.constrain();
        let pwrcfg = example_power!(pwr).freeze();

        // RCC
        let rcc = ctx.device.RCC.constrain();
        let ccdr = rcc.sys_ck(100.MHz()).freeze(pwrcfg, &ctx.device.SYSCFG);

        // Timers
        let mut timer1 = ctx.device.TIM1.timer(
            MilliSeconds::from_ticks(125).into_rate(),
            ccdr.peripheral.TIM1,
            &ccdr.clocks,
        );
        timer1.listen(Event::TimeOut);

        let mut timer2 = ctx.device.TIM2.timer(
            MilliSeconds::from_ticks(250).into_rate(),
            ccdr.peripheral.TIM2,
            &ccdr.clocks,
        );
        timer2.listen(Event::TimeOut);

        let mut timer3 = ctx.device.TIM12.timer(
            MilliSeconds::from_ticks(500).into_rate(),
            ccdr.peripheral.TIM12,
            &ccdr.clocks,
        );
        timer3.listen(Event::TimeOut);

        let mut timer4 = ctx.device.TIM17.timer(
            MilliSeconds::from_ticks(1000).into_rate(),
            ccdr.peripheral.TIM17,
            &ccdr.clocks,
        );
        timer4.listen(Event::TimeOut);

        // GPIO
        let gpioi = ctx.device.GPIOI.split(ccdr.peripheral.GPIOI);

        (
            SharedResources {},
            LocalResources {
                timer1,
                timer2,
                timer3,
                timer4,
                led1: gpioi.pi12.into_push_pull_output(),
                led2: gpioi.pi13.into_push_pull_output(),
                led3: gpioi.pi14.into_push_pull_output(),
                led4: gpioi.pi15.into_push_pull_output(),
            },
        )
    }

    #[task(binds = TIM1_UP, local = [led1, timer1])]
    fn timer1_tick(ctx: timer1_tick::Context) {
        ctx.local.timer1.clear_irq();
        ctx.local.led1.toggle();
    }

    #[task(binds = TIM2, local = [led2, timer2])]
    fn timer2_tick(ctx: timer2_tick::Context) {
        ctx.local.timer2.clear_irq();
        ctx.local.led2.toggle();
    }

    #[task(binds = TIM8_BRK_TIM12, local = [led3, timer3])]
    fn timer3_tick(ctx: timer3_tick::Context) {
        ctx.local.timer3.clear_irq();
        ctx.local.led3.toggle();
    }

    #[task(binds = TIM17, local = [led4, timer4])]
    fn timer4_tick(ctx: timer4_tick::Context) {
        ctx.local.timer4.clear_irq();
        ctx.local.led4.toggle();
    }
}
