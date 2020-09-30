//! Demonstrates a microsecond-scale 64-bit timestamp counter

#![no_main]
#![no_std]

use core::sync::atomic::{AtomicU32, Ordering};

use log::info;

use cortex_m::asm;
use cortex_m_rt::entry;

use pac::interrupt;
use stm32h7xx_hal::{pac, prelude::*, timer};

#[path = "utilities/logger.rs"]
mod logger;

pub static OVERFLOWS: AtomicU32 = AtomicU32::new(0);

#[entry]
fn main() -> ! {
    logger::init();
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    // Constrain and Freeze power
    info!("Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.freeze();

    // Constrain and Freeze clock
    info!("Setup RCC...                  ");
    let rcc = dp.RCC.constrain();

    let ccdr = rcc.sys_ck(100.mhz()).freeze(pwrcfg, &dp.SYSCFG);

    info!("");
    info!("stm32h7xx-hal example - Tick Timer");
    info!("");

    let mut timer =
        dp.TIM2
            .tick_timer(1.mhz(), ccdr.peripheral.TIM2, &ccdr.clocks);
    timer.listen(timer::Event::TimeOut);

    unsafe {
        cp.NVIC.set_priority(interrupt::TIM2, 1);
        pac::NVIC::unmask(interrupt::TIM2);
    }

    loop {
        info!("timestamp: {}", timestamp());
        asm::delay(1000);
    }
}

/// Handle timer overflow and count past 32-bits.
///
/// The interrupt should be configured at maximum priority, it won't take very long.
#[interrupt]
fn TIM2() {
    OVERFLOWS.fetch_add(1, core::sync::atomic::Ordering::SeqCst);
    let timer = unsafe { &*pac::TIM2::ptr() };
    timer.sr.modify(|_, w| w.uif().clear());
}

/// Returns the 64-bit number of microseconds since startup
pub fn timestamp() -> u64 {
    let overflows = OVERFLOWS.load(Ordering::SeqCst) as u64;
    let timer = unsafe { &*pac::TIM2::ptr() };
    let ctr = timer.cnt.read().bits() as u64;
    (overflows << 32) + ctr
}
