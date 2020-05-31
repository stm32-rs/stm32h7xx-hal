#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate panic_itm;

#[macro_use(block)]
extern crate nb;

use cortex_m;
use cortex_m_rt::entry;
use stm32h7xx_hal::hal::digital::v2::{OutputPin, ToggleableOutputPin};
use stm32h7xx_hal::{pac, prelude::*};

use cortex_m_log::println;
use cortex_m_log::{
    destination::Itm, printer::itm::InterruptSync as InterruptSyncItm,
};

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();
    let mut log = InterruptSyncItm::new(Itm::new(cp.ITM));

    // Constrain and Freeze power
    println!(log, "Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    let vos = pwr.freeze();

    // Constrain and Freeze clock
    println!(log, "Setup RCC...                  ");
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.freeze(vos, &dp.SYSCFG);

    println!(log, "");
    println!(log, "stm32h7xx-hal example - Blinky timer");
    println!(log, "");

    let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);

    // Configure PE1 as output.
    let mut led = gpioe.pe1.into_push_pull_output();
    led.set_low().unwrap();

    // Get the delay provider.
    let mut delay = cp.SYST.delay(ccdr.clocks);

    // Configure the timer.
    let mut timer = dp.TIM2.timer(1.hz(), ccdr.peripheral.TIM2, &ccdr.clocks);

    loop {
        for _ in 0..5 {
            // 20ms wait with timer
            led.toggle().unwrap();
            timer.start(20.ms());
            block!(timer.wait()).ok();

            // Delay for 500ms. Timer must operate correctly on next
            // use.
            led.toggle().unwrap();
            delay.delay_ms(500_u16);
        }
    }
}
