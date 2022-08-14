//! Example of reading a voltage with ADC1
//!
//! For an example of using ADC3, see examples/temperature.rs
//! For an example of using ADC1 and ADC2 together, see examples/adc12.rs
//! For an example of using ADC1 and ADC2 in parallel, see examples/adc12_parallel.rs

#![no_main]
#![no_std]

use log::info;

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
    info!("Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    let pwrcfg = example_power!(pwr).freeze();

    // Constrain and Freeze clock
    info!("Setup RCC...                  ");
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

    // Setup ADC
    let mut adc1 = adc::Adc::adc1(
        dp.ADC1,
        4.MHz(),
        &mut delay,
        ccdr.peripheral.ADC12,
        &ccdr.clocks,
    )
    .enable();
    adc1.set_resolution(adc::Resolution::SixteenBit);

    // We can't use ADC2 here because ccdr.peripheral.ADC12 has been
    // consumed. See examples/adc12.rs

    // Setup GPIOC
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);

    // Configure pc0 as an analog input
    let mut channel = gpioc.pc0.into_analog(); // ANALOG IN 10

    loop {
        let data: u32 = adc1.read(&mut channel).unwrap();
        // voltage = reading * (vref/resolution)
        info!(
            "ADC reading: {}, voltage for nucleo: {}",
            data,
            data as f32 * (3.3 / adc1.slope() as f32)
        );
    }
}
