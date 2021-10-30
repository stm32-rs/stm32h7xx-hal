#![no_main]
#![no_std]

#[macro_use]
mod utilities;
use stm32h7xx_hal::{pac, prelude::*, watchdog::SystemWindowWatchdog};

use cortex_m_rt::entry;

use log::info;

#[entry]
fn main() -> ! {
    utilities::logger::init();
    let dp = pac::Peripherals::take().unwrap();

    // Constrain and Freeze power
    info!("Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    let pwrcfg = example_power!(pwr).freeze();

    // Constrain and Freeze clock
    info!("Setup RCC...                  ");
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sys_ck(96.mhz()).freeze(pwrcfg, &dp.SYSCFG);

    #[cfg(any(feature = "rm0433", feature = "rm0455"))]
    let mut watchdog = SystemWindowWatchdog::new(dp.WWDG, &ccdr);

    // Dual core parts
    #[cfg(all(feature = "rm0399", feature = "cm7"))]
    let mut watchdog = SystemWindowWatchdog::new(dp.WWDG1, &ccdr);
    #[cfg(all(feature = "rm0399", feature = "cm4"))]
    let mut watchdog = SystemWindowWatchdog::new(dp.WWDG2, &ccdr);

    // RM0468
    #[cfg(all(feature = "rm0468"))]
    let mut watchdog = SystemWindowWatchdog::new(dp.WWDG1, &ccdr);

    info!("");
    info!("stm32h7xx-hal example - Watchdog");
    info!("");

    // If the watchdog is working correctly this print should
    // appear again and again as the chip gets restarted
    info!("Watchdog restarted!           ");

    // Enable the watchdog with a limit of 100 ms and wait forever
    // -> restart the chip
    watchdog.start(100.ms());

    loop {
        cortex_m::asm::nop()
    }
}
