//! Example of configuring the real-time clock using the internal LSI oscillator

#![no_main]
#![no_std]

use core::cell::RefCell;

use log::info;

use chrono::prelude::*;
use cortex_m::{asm, interrupt::Mutex};
use cortex_m_rt::entry;

use pac::interrupt;
use stm32h7xx_hal::{pac, prelude::*, rtc};

#[path = "utilities/logger.rs"]
mod logger;

static RTC: Mutex<RefCell<Option<rtc::Rtc>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    logger::init();
    let mut dp = pac::Peripherals::take().unwrap();

    // Constrain and Freeze power
    info!("Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    let mut pwrcfg = pwr.freeze();

    // Take the backup power domain
    let backup = pwrcfg.backup().unwrap();

    // Constrain and Freeze clock
    info!("Setup RCC...                  ");
    let rcc = dp.RCC.constrain();

    let ccdr = rcc.sys_ck(100.MHz()).freeze(pwrcfg, &dp.SYSCFG);

    info!("");
    info!("stm32h7xx-hal example - RTC");
    info!("");

    let mut rtc = rtc::Rtc::open_or_init(
        dp.RTC,
        backup.RTC,
        rtc::RtcClock::Lsi,
        &ccdr.clocks,
    );

    // TODO: Get current time from some source
    let now = NaiveDate::from_ymd(2001, 1, 1).and_hms(0, 0, 0);

    rtc.set_date_time(now);
    rtc.enable_wakeup(10);
    rtc.listen(&mut dp.EXTI, rtc::Event::Wakeup);

    unsafe {
        pac::NVIC::unmask(interrupt::RTC_WKUP);
    }

    info!("Time: {}", rtc.date_time().unwrap());

    cortex_m::interrupt::free(|cs| {
        RTC.borrow(cs).replace(Some(rtc));
    });

    loop {
        // Some debuggers have issues working when the MCU goes to standby/sleep.
        // You may want to change this to asm::nop() or try to enable low-power debugging.
        asm::wfi();
    }
}

#[interrupt]
fn RTC_WKUP() {
    let date_time = cortex_m::interrupt::free(|cs| {
        let mut rc = RTC.borrow(cs).borrow_mut();
        let rtc = rc.as_mut().unwrap();
        let date_time = rtc.date_time().unwrap();
        let exti = unsafe { &mut pac::Peripherals::steal().EXTI };
        rtc.unpend(exti, rtc::Event::Wakeup);
        date_time
    });
    info!("Time: {}", date_time);
}
