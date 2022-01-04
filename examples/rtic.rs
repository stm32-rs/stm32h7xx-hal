#![deny(warnings)]
#![deny(unsafe_code)]
#![no_std]
#![no_main]

#[macro_use]
mod utilities;

#[rtic::app(device = stm32h7xx_hal::stm32, peripherals = true)]
mod app {
    use stm32h7xx_hal::gpio::gpioc::{PC13, PC3};
    use stm32h7xx_hal::gpio::{Edge, ExtiPin, Floating, Input};
    use stm32h7xx_hal::gpio::{Output, PushPull};
    use stm32h7xx_hal::hal::digital::v2::ToggleableOutputPin;
    use stm32h7xx_hal::prelude::*;

    use super::*;

    #[shared]
    struct SharedResources {}
    #[local]
    struct LocalResources {
        button: PC13<Input<Floating>>,
        led: PC3<Output<PushPull>>,
    }

    #[init]
    fn init(
        mut ctx: init::Context,
    ) -> (SharedResources, LocalResources, init::Monotonics) {
        utilities::logger::init();
        let pwr = ctx.device.PWR.constrain();
        let pwrcfg = example_power!(pwr).freeze();

        // RCC
        let rcc = ctx.device.RCC.constrain();
        let ccdr = rcc.sys_ck(100.mhz()).freeze(pwrcfg, &ctx.device.SYSCFG);

        // GPIO
        let gpioc = ctx.device.GPIOC.split(ccdr.peripheral.GPIOC);

        // Button
        let mut button = gpioc.pc13.into_floating_input();
        button.make_interrupt_source(&mut ctx.device.SYSCFG);
        button.trigger_on_edge(&mut ctx.device.EXTI, Edge::Rising);
        button.enable_interrupt(&mut ctx.device.EXTI);

        (
            SharedResources {},
            LocalResources {
                button,
                led: gpioc.pc3.into_push_pull_output(),
            },
            init::Monotonics(),
        )
    }

    #[task(binds = EXTI15_10, local = [button, led])]
    fn button_click(ctx: button_click::Context) {
        ctx.local.button.clear_interrupt_pending_bit();
        ctx.local.led.toggle().unwrap();
    }
}
