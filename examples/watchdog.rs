#![no_main]
#![no_std]

extern crate panic_itm;

use stm32h7xx_hal::{
    prelude::*,
    pac,
    watchdog::SystemWindowWatchdog,
};

use cortex_m_rt::entry;

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
    let ccdr = rcc.sys_ck(96.mhz()).freeze(vos, &dp.SYSCFG);

    let mut watchdog = SystemWindowWatchdog::new(
        dp.WWDG,
        &ccdr
    );

    println!(log, "");
    println!(log, "stm32h7xx-hal example - Watchdog");
    println!(log, "");

    // If the watchdog is working correctly this print should
    // appear again and again as the chip gets restarted
    println!(log, "Watchdog restarted!           ");

    // Enable the watchdog with a limit of 100 ms and wait forever
    // -> restart the chip
    watchdog.start(100u32);

    loop {}

}