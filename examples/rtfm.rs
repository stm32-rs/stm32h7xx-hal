#![deny(warnings)]
#![deny(unsafe_code)]
#![no_std]
#![no_main]

extern crate panic_itm;
extern crate rtfm;

use stm32h7xx_hal::hal::digital::v2::ToggleableOutputPin;

use rtfm::app;
use stm32h7xx_hal::gpio::{gpioc::PC13, gpioi::PI13};
use stm32h7xx_hal::gpio::{Edge, ExtiPin, Floating, Input};
use stm32h7xx_hal::gpio::{Output, PushPull};
use stm32h7xx_hal::prelude::*;

#[app(device = stm32h7xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        button: PC13<Input<Floating>>,
        led: PI13<Output<PushPull>>,
    }

    #[init]
    fn init(mut ctx: init::Context) -> init::LateResources {
        let pwr = ctx.device.PWR.constrain();
        let vos = pwr.freeze();

        // RCC
        let rcc = ctx.device.RCC.constrain();
        let ccdr = rcc
            .use_hse(25.mhz())
            .sys_ck(400.mhz())
            .freeze(vos, &ctx.device.SYSCFG);

        // GPIO
        let gpioc = ctx.device.GPIOC.split(ccdr.peripheral.GPIOC);
        let gpioi = ctx.device.GPIOI.split(ccdr.peripheral.GPIOI);

        // Button
        let mut button = gpioc.pc13.into_floating_input();
        button.make_interrupt_source(&mut ctx.device.SYSCFG);
        button.trigger_on_edge(&mut ctx.device.EXTI, Edge::RISING);
        button.enable_interrupt(&mut ctx.device.EXTI);

        init::LateResources {
            button,
            led: gpioi.pi13.into_push_pull_output(),
        }
    }

    #[task(binds = EXTI15_10, resources = [button, led])]
    fn button_click(ctx: button_click::Context) {
        ctx.resources.button.clear_interrupt_pending_bit();
        ctx.resources.led.toggle().unwrap();
    }
};
