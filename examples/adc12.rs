//! Example of using ADC1 and ADC2 together
//!
//! For an example of using ADC3, see examples/temperature.rs
//! For an example of using ADC1 alone, see examples/adc.rs

#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate panic_itm;

use cortex_m;
use cortex_m_rt::entry;

use stm32h7xx_hal::{adc, delay::Delay, pac, prelude::*};

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

    let ccdr = rcc
        .sys_ck(100.mhz())
        .pll2_p_ck(4.mhz()) // Default adc_ker_ck_input
        .freeze(vos, &dp.SYSCFG);

    println!(log, "");
    println!(log, "stm32h7xx-hal example - ADC1 and ADC2");
    println!(log, "");

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
        let data_pc2: u32 = adc1.try_read(&mut channel_pc2).unwrap();
        let data_pc3: u32 = adc2.try_read(&mut channel_pc3).unwrap();
        // voltage = reading * (vref/resolution)
        println!(log, "ADC readings: {} {}", data_pc2, data_pc3);
    }
}
