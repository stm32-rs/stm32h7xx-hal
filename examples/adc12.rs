//! Example of using ADC1 and ADC2 together
//!
//! For an example of using ADC3, see examples/temperature.rs
//! For an example of using ADC1 alone, see examples/adc.rs

#![no_main]
#![no_std]

#[path = "utilities/logger.rs"]
mod logger;

use cortex_m;
use cortex_m_rt::entry;
use log::info;
use stm32h7xx_hal::{adc, delay::Delay, pac, prelude::*};

#[entry]
fn main() -> ! {
    logger::init();
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    // Constrain and Freeze power
    info!("Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.freeze();

    // Constrain and Freeze clock
    info!("Setup RCC...                  ");
    let rcc = dp.RCC.constrain();

    let ccdr = rcc
        .sys_ck(100_u32.MHz())
        .pll2_p_ck(4_u32.MHz()) // Default adc_ker_ck_input
        .freeze(pwrcfg, &dp.SYSCFG);

    info!("");
    info!("stm32h7xx-hal example - ADC1 and ADC2");
    info!("");

    let mut delay = Delay::new(cp.SYST, ccdr.clocks);

    // Setup ADC1 and ADC2
    let (adc1, adc2) = adc::adc12(
        dp.ADC1,
        dp.ADC2,
        &mut delay,
        ccdr.peripheral.ADC12,
        &ccdr.clocks,
    );

    let mut adc1 = adc1.enable();
    adc1.set_resolution(adc::Resolution::SIXTEENBIT);

    let mut adc2 = adc2.enable();
    adc2.set_resolution(adc::Resolution::SIXTEENBIT);

    // Setup GPIOC
    // NOTE: PC2 and PC3 are only pinned out on TFBGA packages!!
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
    let mut channel_pc2 = gpioc.pc2.into_analog(); // AIN 12
    let mut channel_pc3 = gpioc.pc3.into_analog(); // AIN 13

    loop {
        let data_pc2: u32 = adc1.read(&mut channel_pc2).unwrap();
        let data_pc3: u32 = adc2.read(&mut channel_pc3).unwrap();
        // voltage = reading * (vref/resolution)
        info!("ADC readings: {} {}", data_pc2, data_pc3);
    }
}
