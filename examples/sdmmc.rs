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
        .sys_ck(200.mhz())
        .pll1_q_ck(100.mhz())
        .freeze(pwrcfg, &dp.SYSCFG);

    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
    gpiob.pb3.into_alternate::<0>();

    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
    let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);
    let gpioi = dp.GPIOI.split(ccdr.peripheral.GPIOI);

    let mut led = gpioi.pi12.into_push_pull_output();

    // Get the delay provider.
    let mut delay = cp.SYST.delay(ccdr.clocks);

    // SDMMC pins
    let clk = gpioc
        .pc12
        .into_alternate()
        .internal_pull_up(false)
        .set_speed(Speed::VeryHigh);
    let cmd = gpiod
        .pd2
        .into_alternate()
        .internal_pull_up(true)
        .set_speed(Speed::VeryHigh);
    let d0 = gpioc
        .pc8
        .into_alternate()
        .internal_pull_up(true)
        .set_speed(Speed::VeryHigh);
    let d1 = gpioc
        .pc9
        .into_alternate()
        .internal_pull_up(true)
        .set_speed(Speed::VeryHigh);
    let d2 = gpioc
        .pc10
        .into_alternate()
        .internal_pull_up(true)
        .set_speed(Speed::VeryHigh);
    let d3 = gpioc
        .pc11
        .into_alternate()
        .internal_pull_up(true)
        .set_speed(Speed::VeryHigh);

    // Card detect pin
    let _cd = gpioi.pi8.into_pull_up_input();

    // Create SDMMC
    let mut sdmmc = dp.SDMMC1.sdmmc(
        (clk, cmd, d0, d1, d2, d3),
        ccdr.peripheral.SDMMC1,
        &ccdr.clocks,
    );

    // On most development boards this can be increased up to 50MHz. We choose a
    // lower frequency here so that it should work even with flying leads
    // connected to a SD card breakout.
    let bus_frequency = 2.mhz();

    // Loop until we have a card
    loop {
        match sdmmc.init_card(bus_frequency) {
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

    // Read test
    let mut buffer = [0u8; 5120];

    cp.DWT.enable_cycle_counter();
    let start = pac::DWT::cycle_count();

    for i in 0..10 {
        // Read 10 blocks
        sdmmc.read_blocks(10 * i, &mut buffer).unwrap();
    }

    let end = pac::DWT::cycle_count();
    let duration = (end - start) as f32 / ccdr.clocks.c_ck().0 as f32;

    info!("Read 100 blocks at {} bytes/s", 5120. / duration);
    info!("");

    // Write 10 blocks
    let write_buffer = [0x34; 512];
    for i in 0..10 {
        if let Err(err) = sdmmc.write_block(i, &write_buffer) {
            info!("Failed to write block {}: {:?}", i, err);
        } else {
            info!("Wrote block {}", i);
        }

        // Read back
        sdmmc.read_blocks(0, &mut buffer).unwrap();
    }

    // Check the read
    for byte in buffer.iter() {
        assert_eq!(*byte, 0x34);
    }

    info!("Done!");

    loop {
        cortex_m::asm::nop()
    }
}
