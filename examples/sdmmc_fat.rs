#![no_main]
#![no_std]

use {
    embedded_sdmmc::{Controller, VolumeIdx},
    log,
    stm32h7xx_hal::{pac, prelude::*, rcc},
};

#[macro_use]
mod utilities;

// This is just a placeholder TimeSource. In a real world application
// one would probably use the RTC to provide time.
pub struct TimeSource;

impl embedded_sdmmc::TimeSource for TimeSource {
    fn get_timestamp(&self) -> embedded_sdmmc::Timestamp {
        embedded_sdmmc::Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

#[cortex_m_rt::entry]
unsafe fn main() -> ! {
    utilities::logger::init();

    // Get peripherals
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    // Constrain and Freeze power
    let pwr = dp.PWR.constrain();
    let pwrcfg = example_power!(pwr).freeze();

    // Constrain and Freeze clock
    let ccdr = dp
        .RCC
        .constrain()
        .sys_ck(480.mhz())
        .pll1_strategy(rcc::PllConfigStrategy::Iterative)
        .pll1_q_ck(100.mhz())
        .pll2_strategy(rcc::PllConfigStrategy::Iterative)
        .pll3_strategy(rcc::PllConfigStrategy::Iterative)
        .freeze(pwrcfg, &dp.SYSCFG);

    // Get the delay provider.
    let mut delay = cp.SYST.delay(ccdr.clocks);

    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);

    let mut sd = dp.SDMMC2.sdmmc(
        (
            gpiod.pd6.into_alternate(),
            gpiod.pd7.into_alternate(),
            gpiob.pb14.into_alternate(),
            gpiob.pb15.into_alternate(),
            gpiob.pb3.into_alternate(),
            gpiob.pb4.into_alternate(),
        ),
        ccdr.peripheral.SDMMC2,
        &ccdr.clocks,
    );

    // Loop until we have a card
    loop {
        // On most development boards this can be increased up to 50MHz. We choose a
        // lower frequency here so that it should work even with flying leads
        // connected to a SD card breakout.
        match sd.init_card(2.mhz()) {
            Ok(_) => break,
            Err(err) => {
                log::info!("Init err: {:?}", err);
            }
        }

        log::info!("Waiting for card...");

        delay.delay_ms(1000u32);
    }

    // See https://github.com/rust-embedded-community/embedded-sdmmc-rs for docs
    // and more examples

    let mut sd_fatfs = Controller::new(sd.sdmmc_block_device(), TimeSource);
    let sd_fatfs_volume = sd_fatfs.get_volume(VolumeIdx(0)).unwrap();
    let sd_fatfs_root_dir = sd_fatfs.open_root_dir(&sd_fatfs_volume).unwrap();
    sd_fatfs
        .iterate_dir(&sd_fatfs_volume, &sd_fatfs_root_dir, |entry| {
            log::info!("{:?}", entry);
        })
        .unwrap();
    sd_fatfs.close_dir(&sd_fatfs_volume, sd_fatfs_root_dir);

    loop {
        cortex_m::asm::nop()
    }
}
