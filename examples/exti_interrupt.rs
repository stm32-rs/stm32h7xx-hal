#![deny(warnings)]
#![no_main]
#![no_std]

use core::cell::{Cell, RefCell};
use cortex_m::interrupt::{free, Mutex};
use cortex_m::peripheral::NVIC;
use cortex_m_rt::entry;
use stm32h7xx_hal::gpio::{Edge, ExtiPin, Input, Output, PullUp, PushPull};
use stm32h7xx_hal::{interrupt, pac, prelude::*};

// LED pin
use stm32h7xx_hal::gpio::gpioa::PA1;

// Button pins
use stm32h7xx_hal::gpio::gpioc::PC5;
use stm32h7xx_hal::gpio::gpioe::PE3;

#[macro_use]
mod utilities;

use log::info;

// Semaphore for synchronization
static SEMAPHORE: Mutex<Cell<bool>> = Mutex::new(Cell::new(true));

// Setup the sharing of pins between the main loop and the interrupts
static BUTTON1_PIN: Mutex<RefCell<Option<PE3<Input<PullUp>>>>> =
    Mutex::new(RefCell::new(None));
static BUTTON2_PIN: Mutex<RefCell<Option<PC5<Input<PullUp>>>>> =
    Mutex::new(RefCell::new(None));
static LED: Mutex<RefCell<Option<PA1<Output<PushPull>>>>> =
    Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    utilities::logger::init();
    info!("stm32h7xx-hal example - EXTI Interrupt");

    let mut cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    info!("Setup PWR...");
    let pwr = dp.PWR.constrain();
    let pwrcfg = example_power!(pwr).freeze();

    info!("Setup RCC...");
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sys_ck(100.mhz()).freeze(pwrcfg, &dp.SYSCFG);

    // Push button configuration
    let mut syscfg = dp.SYSCFG;
    let mut exti = dp.EXTI;

    let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);
    let mut button1 = gpioe.pe3.into_pull_up_input();
    button1.make_interrupt_source(&mut syscfg);
    button1.trigger_on_edge(&mut exti, Edge::Rising);
    button1.enable_interrupt(&mut exti);

    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
    let mut button2 = gpioc.pc5.into_pull_up_input();
    button2.make_interrupt_source(&mut syscfg);
    button2.trigger_on_edge(&mut exti, Edge::Rising);
    button2.enable_interrupt(&mut exti);

    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    let led = gpioa.pa1.into_push_pull_output();

    // Save information needed by the interrupt handlers to the global variable
    free(|cs| {
        BUTTON1_PIN.borrow(cs).replace(Some(button1));
        BUTTON2_PIN.borrow(cs).replace(Some(button2));
        LED.borrow(cs).replace(Some(led));
    });

    // Enable the button interrupts
    unsafe {
        cp.NVIC.set_priority(interrupt::EXTI3, 1);
        cp.NVIC.set_priority(interrupt::EXTI9_5, 1);
        NVIC::unmask::<interrupt>(interrupt::EXTI3);
        NVIC::unmask::<interrupt>(interrupt::EXTI9_5);
    }

    loop {
        cortex_m::asm::nop();
    }
}

fn toggle_led(on_or_off: bool) {
    free(|cs| {
        if let Some(b) = LED.borrow(cs).borrow_mut().as_mut() {
            if on_or_off {
                b.set_high();
            } else {
                b.set_low();
            }
        }
    });
}

#[interrupt]
fn EXTI9_5() {
    info!("EXTI9_5 fired!");
    toggle_led(true);
    free(|cs| {
        if let Some(b) = BUTTON2_PIN.borrow(cs).borrow_mut().as_mut() {
            b.clear_interrupt_pending_bit()
        }

        // Signal that the interrupt fired
        SEMAPHORE.borrow(cs).set(false);
    });
}

#[interrupt]
fn EXTI3() {
    info!("EXTI3 fired!");
    toggle_led(false);
    free(|cs| {
        if let Some(b) = BUTTON1_PIN.borrow(cs).borrow_mut().as_mut() {
            b.clear_interrupt_pending_bit()
        }

        // Signal that the interrupt fired
        SEMAPHORE.borrow(cs).set(false);
    });
}
