//! Example for using ADC3 to read the internal temperature sensor
//!
//! For an example of using ADC1, see examples/adc.rs
//! For an example of using ADC1 and ADC2 together, see examples/adc12.rs
//! For an example of using ADC1 and ADC2 in parallel, see examples/adc12_parallel.rs

#![no_main]
#![no_std]

use cortex_m_rt::entry;
#[macro_use]
mod utilities;

use stm32h7xx_hal::{
    adc,
    delay::Delay,
    pac,
    prelude::*,
    signature::{TS_CAL_110, TS_CAL_30},
};

use log::info;

#[entry]
fn main() -> ! {
    utilities::logger::init();
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    // Constrain and Freeze power
    info!("Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    let pwrcfg = example_power!(pwr).freeze();

    // Constrain and Freeze clock
    info!("Setup RCC...                  ");
    let rcc = dp.RCC.constrain();

    let ccdr = rcc
        .sys_ck(100.MHz())
        .pll2_p_ck(4.MHz()) // Default adc_ker_ck_input
        .freeze(pwrcfg, &dp.SYSCFG);

    info!("");
    info!("stm32h7xx-hal example - Temperature");
    info!("");

    let mut delay = Delay::new(cp.SYST, ccdr.clocks);

    // Setup ADC
    #[cfg(not(feature = "rm0455"))]
    let mut adc =
        adc::Adc::adc3(dp.ADC3, &mut delay, ccdr.peripheral.ADC3, &ccdr.clocks);

    // On RM0455 parts, the temperature sensor is on ADC2
    #[cfg(feature = "rm0455")]
    let mut adc = adc::Adc::adc2(
        dp.ADC2,
        &mut delay,
        ccdr.peripheral.ADC12,
        &ccdr.clocks,
    );

    // Set resolution
    adc.set_resolution(adc::Resolution::SIXTEENBIT);

    // Setup Temperature Sensor on the disabled ADC
    let mut channel = adc::Temperature::new();
    channel.enable(&adc);
    delay.delay_us(25_u16);
    let mut adc = adc.enable();

    let vdda = 2.500; // Volts

    loop {
        let word: u32 =
            adc.read(&mut channel).expect("Temperature read failed.");

        // Average slope
        let cal =
            (110.0 - 30.0) / (TS_CAL_110::read() - TS_CAL_30::read()) as f32;
        // Calibration values are measured at VDDA = 3.3 V ± 10 mV
        let word_3v3 = word as f32 * vdda / 3.3;
        // Linear interpolation
        let temperature = cal * (word_3v3 - TS_CAL_30::read() as f32) + 30.0;

        info!("ADC reading: {}, Temperature: {:.1} °C", word, temperature);
    }
}
