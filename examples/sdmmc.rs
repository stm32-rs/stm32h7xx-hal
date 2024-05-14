//! SDMMC card example
//!
//! Tested on a STM32H747I-DISCO development board with a SanDisk Extreme 32 GB
//! SDHC UHS-I card.

#![deny(warnings)]
#![no_main]
#![no_std]

#[macro_use]
mod utilities;

use cortex_m_rt::entry;
use stm32h7xx_hal::gpio::Speed;
use stm32h7xx_hal::sdmmc::{SdCard, Sdmmc};
use stm32h7xx_hal::{pac, prelude::*};

use log::info;

#[entry]
fn main() -> ! {
    utilities::logger::init();
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    // Constrain and Freeze power
    info!("Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    let pwrcfg = example_power!(pwr).freeze();

    // Constrain and Freeze clock
    info!("Setup RCC...                  ");
    let rcc = dp.RCC.constrain();

    let ccdr = rcc
        .sys_ck(200.MHz())
        .pll1_q_ck(100.MHz())
        .freeze(pwrcfg, &dp.SYSCFG);

    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
    let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);

    // STM32H747I-DISCO development board
    #[cfg(feature = "rm0399")]
    let mut led = {
        let gpioi = dp.GPIOI.split(ccdr.peripheral.GPIOI);

        // Card detect pin
        let _cd = gpioi.pi8.into_pull_up_input();

        gpioi.pi12.into_push_pull_output()
    };
    #[cfg(not(feature = "rm0399"))]
    let mut led = {
        let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);
        gpioe.pe1.into_push_pull_output()
    };

    // Get the delay provider.
    let mut delay = cp.SYST.delay(ccdr.clocks);

    // SDMMC pins
    let clk = gpioc
        .pc12
        .into_alternate()
        .internal_pull_up(false)
        .speed(Speed::VeryHigh);
    let cmd = gpiod
        .pd2
        .into_alternate()
        .internal_pull_up(true)
        .speed(Speed::VeryHigh);
    let d0 = gpioc
        .pc8
        .into_alternate()
        .internal_pull_up(true)
        .speed(Speed::VeryHigh);
    let d1 = gpioc
        .pc9
        .into_alternate()
        .internal_pull_up(true)
        .speed(Speed::VeryHigh);
    let d2 = gpioc
        .pc10
        .into_alternate()
        .internal_pull_up(true)
        .speed(Speed::VeryHigh);
    let d3 = gpioc
        .pc11
        .into_alternate()
        .internal_pull_up(true)
        .speed(Speed::VeryHigh);

    // Create SDMMC
    let mut sdmmc: Sdmmc<_, SdCard> = dp.SDMMC1.sdmmc(
        (clk, cmd, d0, d1, d2, d3),
        ccdr.peripheral.SDMMC1,
        &ccdr.clocks,
    );

    // On most development boards this can be increased up to 50MHz. We choose a
    // lower frequency here so that it should work even with flying leads
    // connected to a SD card breakout.
    let bus_frequency = 2.MHz();

    // Loop until we have a card
    loop {
        match sdmmc.init(bus_frequency) {
            Ok(_) => break,
            Err(err) => {
                info!("Init err: {:?}", err);
            }
        }

        info!("Waiting for card...");

        delay.delay_ms(1000u32);
        led.toggle();
    }

    // Print card information
    info!("");
    info!("----------------------");

    let size = sdmmc.card().unwrap().size();
    info!("Size: {}", size);

    let ocr = sdmmc.card().unwrap().ocr;
    info!("{:?}", ocr);

    let scr = sdmmc.card().unwrap().scr;
    info!("{:?}", scr);

    let cid = sdmmc.card().unwrap().cid;
    info!("{:?}", cid);

    let csd = sdmmc.card().unwrap().csd;
    info!("{:?}", csd);

    let status = sdmmc.card().unwrap().status;
    info!("{:?}", status);

    info!("Bus Clock: {}", sdmmc.clock());
    info!("----------------------");
    info!("");

    cp.DWT.enable_cycle_counter();

    // Write single block test
    let write_buffer = [0x34; 512];
    let start = pac::DWT::cycle_count();
    sdmmc.write_block(0, &write_buffer).unwrap();
    let end = pac::DWT::cycle_count();
    let duration = (end - start) as f32 / ccdr.clocks.c_ck().raw() as f32;
    info!("Wrote single block at {} bytes/s", 512.0 / duration);

    // Write multiple blocks test
    let write_buffer = [0x34; 512 * 16];
    let start = pac::DWT::cycle_count();
    sdmmc.write_blocks(0, &write_buffer).unwrap();
    let end = pac::DWT::cycle_count();
    let duration = (end - start) as f32 / ccdr.clocks.c_ck().raw() as f32;
    info!("Wrote 16 blocks at {} bytes/s", (512.0 * 16.0) / duration);

    // Read single block test
    let mut buffer = [0u8; 512];
    let start = pac::DWT::cycle_count();
    sdmmc.read_block(0, &mut buffer).unwrap();
    let end = pac::DWT::cycle_count();
    let duration = (end - start) as f32 / ccdr.clocks.c_ck().raw() as f32;
    info!("Read single block at {} bytes/s", 512.0 / duration);

    // Read multiple blocks test
    let mut buffer = [0u8; 512 * 16];
    let start = pac::DWT::cycle_count();
    sdmmc.read_blocks(0, &mut buffer).unwrap();
    let end = pac::DWT::cycle_count();
    let duration = (end - start) as f32 / ccdr.clocks.c_ck().raw() as f32;
    info!("Read 16 blocks at {} bytes/s", (512.0 * 16.0) / duration);

    info!("Verification test...");
    for byte in buffer.iter() {
        assert_eq!(*byte, 0x34);
    }
    info!("Verified all blocks");

    info!("Done!");

    loop {
        cortex_m::asm::nop()
    }
}
