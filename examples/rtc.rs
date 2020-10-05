//! Example of configuring the real-time clock using the internal LSI oscillator

#![no_main]
#![no_std]

use log::info;

use chrono::prelude::*;
use cortex_m::asm;
use cortex_m_rt::entry;

use pac::interrupt;
use stm32h7xx_hal::{pac, prelude::*, rtc};

#[path = "utilities/logger.rs"]
mod logger;

#[entry]
fn main() -> ! {
    logger::init();
    let dp = pac::Peripherals::take().unwrap();

    // Constrain and Freeze power
    info!("Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    let mut pwrcfg = pwr.freeze();

    // Take the backup power domain
    let backup = pwrcfg.backup().unwrap();

    // Constrain and Freeze clock
    info!("Setup RCC...                  ");
    let rcc = dp.RCC.constrain();

    let ccdr = rcc.sys_ck(100.mhz()).freeze(pwrcfg, &dp.SYSCFG);

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
    let now = NaiveDate::from_ymd(2000, 1, 1).and_hms(0, 0, 0);

    rtc.set_date_time(now);
    rtc.listen(rtc::Event::Wakeup);
    rtc.enable_wakeup(10);

    unsafe {
        pac::NVIC::unmask(interrupt::RTC_WKUP);
    }

    loop {
        info!("Time: {}", rtc.time().unwrap());
        rtc.unpend(rtc::Event::Wakeup);
        asm::wfi();
    }
}

#[interrupt]
fn RTC_WKUP() {
    // Just cause the asm::wfi() above to complete so the loop runs one more time
}
