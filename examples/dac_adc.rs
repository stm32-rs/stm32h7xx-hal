//! Example of using the DAC to generate a voltage and the ADC to read it
//!
//! Connect a jumper between pins PA4 and PC0

#![no_main]
#![no_std]

use log::info;

use cortex_m_rt::entry;

use stm32h7xx_hal::{
    adc, delay::Delay, pac, prelude::*, rcc::rec::AdcClkSel, traits::DacOut,
};

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
    // adc_ker_ck_input is pll2_p_ck, but we will use per_ck. per_ck is sourced
    // from the 64MHz HSI
    //
    // adc_ker_ck_input is then divided by the ADC prescaler to give f_adc. The
    // maximum f_adc is 50MHz
    let mut ccdr = rcc.sys_ck(50.MHz()).freeze(pwrcfg, &dp.SYSCFG);

    // Switch adc_ker_ck_input multiplexer to per_ck
    ccdr.peripheral.kernel_adc_clk_mux(AdcClkSel::Per);

    info!("");
    info!("stm32h7xx-hal example - DAC and ADC");
    info!("");

    let mut delay = Delay::new(cp.SYST, ccdr.clocks);

    // Setup ADC
    let mut adc1 = adc::Adc::adc1(
        dp.ADC1,
        16.MHz(),
        &mut delay,
        ccdr.peripheral.ADC12,
        &ccdr.clocks,
    )
    .enable();
    adc1.set_resolution(adc::Resolution::SixteenBit);

    // We can't use ADC2 here because ccdr.peripheral.ADC12 has been
    // consumed. See examples/adc12.rs

    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);

    // Setup DAC
    #[cfg(not(feature = "rm0455"))]
    let dac = dp.DAC.dac(gpioa.pa4, ccdr.peripheral.DAC12);
    #[cfg(feature = "rm0455")]
    let dac = dp.DAC1.dac(gpioa.pa4, ccdr.peripheral.DAC1);

    // Calibrate output buffer then enable DAC channel
    let mut dac = dac.calibrate_buffer(&mut delay).enable();

    let mut channel = gpioc.pc0.into_analog(); // ANALOG IN 10

    dac.set_value(2048); // set to 50% of vdda

    loop {
        let reading: u32 = adc1.read(&mut channel).unwrap();
        // voltage = reading * (vref/resolution)
        let voltage = reading as f32 * (3.3 / adc1.slope() as f32);
        info!("ADC reading: {}, voltage for nucleo: {}", reading, voltage);

        // check voltage is really 50% of vdda
        assert!(voltage - 1.65 < 8.25e-3); // 0.5% error
    }
}
