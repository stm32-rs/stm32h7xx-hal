//! Demonstrates a microsecond-scale 64-bit timestamp counter using the LPTIM2
//! timer

#![no_main]
#![no_std]

use core::{
    cell::RefCell,
    sync::atomic::{AtomicU32, Ordering},
};

use cortex_m::{asm, interrupt::Mutex};
use cortex_m_rt::entry;
use log::info;

use pac::interrupt;
use stm32h7xx_hal::{pac, prelude::*, timer};

#[path = "utilities/logger.rs"]
mod logger;

static OVERFLOWS: AtomicU32 = AtomicU32::new(0);
static TIMER: Mutex<
    RefCell<Option<timer::LpTimer<pac::LPTIM2, timer::Enabled>>>,
> = Mutex::new(RefCell::new(None));

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

    let ccdr = rcc.sys_ck(4.mhz()).freeze(pwrcfg, &dp.SYSCFG);

    info!("");
    info!("stm32h7xx-hal example - Tick Timer");
    info!("");

    let mut timer = dp
        .LPTIM2
        .tick_timer(10.khz(), ccdr.peripheral.LPTIM2, &ccdr.clocks)
        .pause();
    timer.listen(timer::Event::TimeOut);
    let timer = timer.resume();

    cortex_m::interrupt::free(|cs| {
        TIMER.borrow(cs).replace(Some(timer));
    });

    unsafe {
        cp.NVIC.set_priority(interrupt::LPTIM2, 1);
        pac::NVIC::unmask(interrupt::LPTIM2);
    }

    loop {
        let timest = timestamp();

        info!("timestamp: {}", timest);
        asm::delay(1000);
    }
}

/// Handle timer overflow and count past 32-bits.
///
/// The interrupt should be configured at maximum priority, it won't take very long.
#[interrupt]
fn LPTIM2() {
    OVERFLOWS.fetch_add(1, core::sync::atomic::Ordering::SeqCst);
    cortex_m::interrupt::free(|cs| {
        let mut rc = TIMER.borrow(cs).borrow_mut();
        let timer = rc.as_mut().unwrap();
        timer.clear_irq();
    })
}

/// Returns the 64-bit number of microseconds since startup
pub fn timestamp() -> u64 {
    let overflows = OVERFLOWS.load(Ordering::SeqCst) as u64;
    let ctr = cortex_m::interrupt::free(|cs| {
        let rc = TIMER.borrow(cs).borrow();
        let timer = rc.as_ref().unwrap();
        timer.counter() as u64
    });
    100 * ((overflows << 16) + ctr)
}
