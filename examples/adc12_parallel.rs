//! Example of reading a voltage with ADC1 and ADC2 on two different channels in
//! parallel
//!
//! For an example of using ADC1, see examples/adc.rs
//! For an example of using ADC3, see examples/temperature.rs
//! For an example of using ADC1 and ADC2 together, see examples/adc12.rs

#![no_main]
#![no_std]

use log::info;

use nb::block;

use cortex_m_rt::entry;

use stm32h7xx_hal::{adc, delay::Delay, pac, prelude::*, rcc::rec::AdcClkSel};

#[macro_use]
mod utilities;

#[entry]
fn main() -> ! {
    utilities::logger::init();
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    // Constrain and Freeze power
    info!("Setup PWR...");
    let pwr = dp.PWR.constrain();
    let pwrcfg = example_power!(pwr).freeze();

    // Constrain and Freeze clock
    info!("Setup RCC...");
    let rcc = dp.RCC.constrain();

    // We need to configure a clock for adc_ker_ck_input. The default
    // adc_ker_ck_input is pll2_p_ck, but we will use per_ck. Here we
    // set per_ck to 4MHz.
    //
    // The maximum adc_ker_ck_input frequency is 100MHz for revision V and 36MHz
    // otherwise
    let mut ccdr = rcc
        .sys_ck(100.MHz())
        .per_ck(4.MHz())
        .freeze(pwrcfg, &dp.SYSCFG);

    // Switch adc_ker_ck_input multiplexer to per_ck
    ccdr.peripheral.kernel_adc_clk_mux(AdcClkSel::Per);

    info!("");
    info!("stm32h7xx-hal example - ADC");
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
    adc1.set_resolution(adc::Resolution::SixteenBit);
    adc1.set_sample_time(adc::AdcSampleTime::T_387);
    let mut adc2 = adc2.enable();
    adc2.set_resolution(adc::Resolution::SixteenBit);
    adc2.set_sample_time(adc::AdcSampleTime::T_387);

    // Setup GPIOA
    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);

    // Configure pins 23 and 22 as an analog inputs
    let mut channel1 = gpioa.pa4.into_analog(); // DAISY PIN 23
    let mut channel2 = gpioa.pa5.into_analog(); // DAISY PIN 22

    loop {
        adc1.start_conversion(&mut channel1);
        adc2.start_conversion(&mut channel2);

        let data1 = block!(adc1.read_sample()).unwrap();
        let data2 = block!(adc2.read_sample()).unwrap();

        // voltage = reading * (vref/resolution)
        info!(
            "ADC1 reading: {}, voltage for Daisy pin X: {}",
            data1,
            data1 as f32 * (3.3 / adc1.slope() as f32)
        );
        info!(
            "ADC2 reading: {}, voltage for Daisy pin X: {}",
            data2,
            data2 as f32 * (3.3 / adc2.slope() as f32)
        );
    }
}
