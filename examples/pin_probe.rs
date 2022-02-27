#![no_main]
#![no_std]

pub fn init() {}

use cortex_m_rt::entry;
use stm32h7xx_hal::gpio::{erased::EPin, Floating, Input, Output, PushPull};
use stm32h7xx_hal::hal::digital::v2::{InputPin, OutputPin};
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

    let mut input_pin = gpioa.pa0.into_floating_input().erase();

    let mut most_pins = [
        //gpioa.pa0.into_push_pull_output().erase(), // Used as probe pin
        gpioa.pa1.into_push_pull_output().erase(),
        gpioa.pa2.into_push_pull_output().erase(),
        gpioa.pa3.into_push_pull_output().erase(),
        gpioa.pa4.into_push_pull_output().erase(),
        gpioa.pa5.into_push_pull_output().erase(),
        gpioa.pa6.into_push_pull_output().erase(),
        gpioa.pa7.into_push_pull_output().erase(),
        gpioa.pa8.into_push_pull_output().erase(),
        gpioa.pa9.into_push_pull_output().erase(),
        gpioa.pa10.into_push_pull_output().erase(),
        gpioa.pa11.into_push_pull_output().erase(),
        gpioa.pa12.into_push_pull_output().erase(),
        gpioa.pa13.into_push_pull_output().erase(),
        gpioa.pa14.into_push_pull_output().erase(),
        gpioa.pa15.into_push_pull_output().erase(),
        gpiob.pb0.into_push_pull_output().erase(),
        gpiob.pb1.into_push_pull_output().erase(),
        gpiob.pb2.into_push_pull_output().erase(),
        gpiob.pb3.into_push_pull_output().erase(),
        gpiob.pb4.into_push_pull_output().erase(),
        gpiob.pb5.into_push_pull_output().erase(),
        gpiob.pb6.into_push_pull_output().erase(),
        gpiob.pb7.into_push_pull_output().erase(),
        gpiob.pb8.into_push_pull_output().erase(),
        gpiob.pb9.into_push_pull_output().erase(),
        gpiob.pb10.into_push_pull_output().erase(),
        gpiob.pb11.into_push_pull_output().erase(),
        gpiob.pb12.into_push_pull_output().erase(),
        gpiob.pb13.into_push_pull_output().erase(),
        gpiob.pb14.into_push_pull_output().erase(),
        gpiob.pb15.into_push_pull_output().erase(),
        gpioc.pc0.into_push_pull_output().erase(),
        gpioc.pc1.into_push_pull_output().erase(),
        gpioc.pc2.into_push_pull_output().erase(),
        gpioc.pc3.into_push_pull_output().erase(),
        gpioc.pc4.into_push_pull_output().erase(),
        gpioc.pc5.into_push_pull_output().erase(),
        gpioc.pc6.into_push_pull_output().erase(),
        gpioc.pc7.into_push_pull_output().erase(),
        gpioc.pc8.into_push_pull_output().erase(),
        gpioc.pc9.into_push_pull_output().erase(),
        gpioc.pc10.into_push_pull_output().erase(),
        gpioc.pc11.into_push_pull_output().erase(),
        gpioc.pc12.into_push_pull_output().erase(),
        gpioc.pc13.into_push_pull_output().erase(),
        gpioc.pc14.into_push_pull_output().erase(),
        gpioc.pc15.into_push_pull_output().erase(),
        gpiod.pd0.into_push_pull_output().erase(),
        gpiod.pd1.into_push_pull_output().erase(),
        gpiod.pd2.into_push_pull_output().erase(),
        gpiod.pd3.into_push_pull_output().erase(),
        gpiod.pd4.into_push_pull_output().erase(),
        gpiod.pd5.into_push_pull_output().erase(),
        gpiod.pd6.into_push_pull_output().erase(),
        gpiod.pd7.into_push_pull_output().erase(),
        gpiod.pd8.into_push_pull_output().erase(),
        gpiod.pd9.into_push_pull_output().erase(),
        gpiod.pd10.into_push_pull_output().erase(),
        gpiod.pd11.into_push_pull_output().erase(),
        gpiod.pd12.into_push_pull_output().erase(),
        gpiod.pd13.into_push_pull_output().erase(),
        gpiod.pd14.into_push_pull_output().erase(),
        gpiod.pd15.into_push_pull_output().erase(),
        gpioe.pe0.into_push_pull_output().erase(),
        gpioe.pe1.into_push_pull_output().erase(),
        gpioe.pe2.into_push_pull_output().erase(),
        gpioe.pe3.into_push_pull_output().erase(),
        gpioe.pe4.into_push_pull_output().erase(),
        gpioe.pe5.into_push_pull_output().erase(),
        gpioe.pe6.into_push_pull_output().erase(),
        gpioe.pe7.into_push_pull_output().erase(),
        gpioe.pe8.into_push_pull_output().erase(),
        gpioe.pe9.into_push_pull_output().erase(),
        gpioe.pe10.into_push_pull_output().erase(),
        gpioe.pe11.into_push_pull_output().erase(),
        gpioe.pe12.into_push_pull_output().erase(),
        gpioe.pe13.into_push_pull_output().erase(),
        gpioe.pe14.into_push_pull_output().erase(),
        gpioe.pe15.into_push_pull_output().erase(),
        gpiof.pf0.into_push_pull_output().erase(),
        gpiof.pf1.into_push_pull_output().erase(),
        gpiof.pf2.into_push_pull_output().erase(),
        gpiof.pf3.into_push_pull_output().erase(),
        gpiof.pf4.into_push_pull_output().erase(),
        gpiof.pf5.into_push_pull_output().erase(),
        gpiof.pf6.into_push_pull_output().erase(),
        gpiof.pf7.into_push_pull_output().erase(),
        gpiof.pf8.into_push_pull_output().erase(),
        gpiof.pf9.into_push_pull_output().erase(),
        gpiof.pf10.into_push_pull_output().erase(),
        gpiof.pf11.into_push_pull_output().erase(),
        gpiof.pf12.into_push_pull_output().erase(),
        gpiof.pf13.into_push_pull_output().erase(),
        gpiof.pf14.into_push_pull_output().erase(),
        gpiof.pf15.into_push_pull_output().erase(),
        gpiog.pg0.into_push_pull_output().erase(),
        gpiog.pg1.into_push_pull_output().erase(),
        gpiog.pg2.into_push_pull_output().erase(),
        gpiog.pg3.into_push_pull_output().erase(),
        gpiog.pg4.into_push_pull_output().erase(),
        gpiog.pg5.into_push_pull_output().erase(),
        gpiog.pg6.into_push_pull_output().erase(),
        gpiog.pg7.into_push_pull_output().erase(),
        gpiog.pg8.into_push_pull_output().erase(),
        gpiog.pg9.into_push_pull_output().erase(),
        gpiog.pg10.into_push_pull_output().erase(),
        gpiog.pg11.into_push_pull_output().erase(),
        gpiog.pg12.into_push_pull_output().erase(),
        gpiog.pg13.into_push_pull_output().erase(),
        gpiog.pg14.into_push_pull_output().erase(),
        gpiog.pg15.into_push_pull_output().erase(),
    ];

    loop {
        let found_index =
            probe_pins(&mut input_pin, &mut most_pins, &mut delay);
        info!("Pin: {:?}", found_index.map(|index| &most_pins[index]));
        delay.delay_ms(1_u16);
    }
}

fn probe_pins<'a>(
    input: &mut EPin<Input<Floating>>,
    pins: &'a mut [EPin<Output<PushPull>>],
    delay: &mut Delay,
) -> Option<usize> {
    if input.is_high().unwrap() {
        return None;
    }

    for (i, pin) in pins.iter_mut().enumerate() {
        pin.set_high().unwrap();
        delay.delay_ms(1_u16);

        if input.is_high().unwrap() {
            pin.set_low().unwrap();
            return Some(i);
        }

        pin.set_low().unwrap();
    }

    None
}
