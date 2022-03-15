//! Example for experimenting with pinouts by shorting outputs to A0
//!
//! Also an example for using erased pins to have a single array containing most pins on stm32.
//!
//! The example iterates through each pin, setting it high then testing if A0 went high as well,
//! then resetting the tested pin to low.
//! This allows for any pin to be quickly discovered by shorting A0 and the pin in question.
//! If multiple pins connect to the same output, it will only report the first discovered pin in the array.

#![deny(warnings)]
#![no_main]
#![no_std]

use cortex_m_rt::entry;
use stm32h7xx_hal::gpio::{erased::EPin, Floating, Input};
use stm32h7xx_hal::hal::digital::v2::InputPin;
use stm32h7xx_hal::{delay::Delay, pac, prelude::*};

#[macro_use]
mod utilities;

use log::info;

#[entry]
fn main() -> ! {
    utilities::logger::init();
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    // Constrain and Freeze power
    let pwr = dp.PWR.constrain();
    let pwrcfg = example_power!(pwr).freeze();

    // Constrain and Freeze clock
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sys_ck(200.mhz()).freeze(pwrcfg, &dp.SYSCFG);

    let mut delay = Delay::new(cp.SYST, ccdr.clocks);

    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
    let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);
    let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);
    let gpiof = dp.GPIOF.split(ccdr.peripheral.GPIOF);
    let gpiog = dp.GPIOG.split(ccdr.peripheral.GPIOG);

    let mut most_pins = [
        gpioa.pa0.into_floating_input().erase(), // Used as probe pin
        gpioa.pa1.into_floating_input().erase(),
        gpioa.pa2.into_floating_input().erase(),
        gpioa.pa3.into_floating_input().erase(),
        gpioa.pa4.into_floating_input().erase(),
        gpioa.pa5.into_floating_input().erase(),
        gpioa.pa6.into_floating_input().erase(),
        gpioa.pa7.into_floating_input().erase(),
        gpioa.pa8.into_floating_input().erase(),
        gpioa.pa9.into_floating_input().erase(),
        gpioa.pa10.into_floating_input().erase(),
        gpioa.pa11.into_floating_input().erase(),
        gpioa.pa12.into_floating_input().erase(),
        gpioa.pa13.into_floating_input().erase(),
        gpioa.pa14.into_floating_input().erase(),
        gpioa.pa15.into_floating_input().erase(),
        gpiob.pb0.into_floating_input().erase(),
        gpiob.pb1.into_floating_input().erase(),
        gpiob.pb2.into_floating_input().erase(),
        gpiob.pb3.into_floating_input().erase(),
        gpiob.pb4.into_floating_input().erase(),
        gpiob.pb5.into_floating_input().erase(),
        gpiob.pb6.into_floating_input().erase(),
        gpiob.pb7.into_floating_input().erase(),
        gpiob.pb8.into_floating_input().erase(),
        gpiob.pb9.into_floating_input().erase(),
        gpiob.pb10.into_floating_input().erase(),
        gpiob.pb11.into_floating_input().erase(),
        gpiob.pb12.into_floating_input().erase(),
        gpiob.pb13.into_floating_input().erase(),
        gpiob.pb14.into_floating_input().erase(),
        gpiob.pb15.into_floating_input().erase(),
        gpioc.pc0.into_floating_input().erase(),
        gpioc.pc1.into_floating_input().erase(),
        gpioc.pc2.into_floating_input().erase(),
        gpioc.pc3.into_floating_input().erase(),
        gpioc.pc4.into_floating_input().erase(),
        gpioc.pc5.into_floating_input().erase(),
        gpioc.pc6.into_floating_input().erase(),
        gpioc.pc7.into_floating_input().erase(),
        gpioc.pc8.into_floating_input().erase(),
        gpioc.pc9.into_floating_input().erase(),
        gpioc.pc10.into_floating_input().erase(),
        gpioc.pc11.into_floating_input().erase(),
        gpioc.pc12.into_floating_input().erase(),
        gpioc.pc13.into_floating_input().erase(),
        gpioc.pc14.into_floating_input().erase(),
        gpioc.pc15.into_floating_input().erase(),
        gpiod.pd0.into_floating_input().erase(),
        gpiod.pd1.into_floating_input().erase(),
        gpiod.pd2.into_floating_input().erase(),
        gpiod.pd3.into_floating_input().erase(),
        gpiod.pd4.into_floating_input().erase(),
        gpiod.pd5.into_floating_input().erase(),
        gpiod.pd6.into_floating_input().erase(),
        gpiod.pd7.into_floating_input().erase(),
        gpiod.pd8.into_floating_input().erase(),
        gpiod.pd9.into_floating_input().erase(),
        gpiod.pd10.into_floating_input().erase(),
        gpiod.pd11.into_floating_input().erase(),
        gpiod.pd12.into_floating_input().erase(),
        gpiod.pd13.into_floating_input().erase(),
        gpiod.pd14.into_floating_input().erase(),
        gpiod.pd15.into_floating_input().erase(),
        gpioe.pe0.into_floating_input().erase(),
        gpioe.pe1.into_floating_input().erase(),
        gpioe.pe2.into_floating_input().erase(),
        gpioe.pe3.into_floating_input().erase(),
        gpioe.pe4.into_floating_input().erase(),
        gpioe.pe5.into_floating_input().erase(),
        gpioe.pe6.into_floating_input().erase(),
        gpioe.pe7.into_floating_input().erase(),
        gpioe.pe8.into_floating_input().erase(),
        gpioe.pe9.into_floating_input().erase(),
        gpioe.pe10.into_floating_input().erase(),
        gpioe.pe11.into_floating_input().erase(),
        gpioe.pe12.into_floating_input().erase(),
        gpioe.pe13.into_floating_input().erase(),
        gpioe.pe14.into_floating_input().erase(),
        gpioe.pe15.into_floating_input().erase(),
        gpiof.pf0.into_floating_input().erase(),
        gpiof.pf1.into_floating_input().erase(),
        gpiof.pf2.into_floating_input().erase(),
        gpiof.pf3.into_floating_input().erase(),
        gpiof.pf4.into_floating_input().erase(),
        gpiof.pf5.into_floating_input().erase(),
        gpiof.pf6.into_floating_input().erase(),
        gpiof.pf7.into_floating_input().erase(),
        gpiof.pf8.into_floating_input().erase(),
        gpiof.pf9.into_floating_input().erase(),
        gpiof.pf10.into_floating_input().erase(),
        gpiof.pf11.into_floating_input().erase(),
        gpiof.pf12.into_floating_input().erase(),
        gpiof.pf13.into_floating_input().erase(),
        gpiof.pf14.into_floating_input().erase(),
        gpiof.pf15.into_floating_input().erase(),
        gpiog.pg0.into_floating_input().erase(),
        gpiog.pg1.into_floating_input().erase(),
        gpiog.pg2.into_floating_input().erase(),
        gpiog.pg3.into_floating_input().erase(),
        gpiog.pg4.into_floating_input().erase(),
        gpiog.pg5.into_floating_input().erase(),
        gpiog.pg6.into_floating_input().erase(),
        gpiog.pg7.into_floating_input().erase(),
        gpiog.pg8.into_floating_input().erase(),
        gpiog.pg9.into_floating_input().erase(),
        gpiog.pg10.into_floating_input().erase(),
        gpiog.pg11.into_floating_input().erase(),
        gpiog.pg12.into_floating_input().erase(),
        gpiog.pg13.into_floating_input().erase(),
        gpiog.pg14.into_floating_input().erase(),
        gpiog.pg15.into_floating_input().erase(),
    ];

    loop {
        probe_pins(&mut most_pins);
        delay.delay_ms(100_u16);
    }
}

fn probe_pins<'a>(pins: &'a mut [EPin<Input<Floating>>]) -> Option<usize> {
    for pin in pins.iter_mut() {
        info!(
            "Pin {:?}: {}",
            pin,
            if pin.is_high().unwrap() {
                "HIGH"
            } else {
                "LOW"
            }
        )
    }

    None
}
