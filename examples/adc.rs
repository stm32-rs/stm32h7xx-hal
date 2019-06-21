#![deny(unsafe_code)]
#![no_main]
#![no_std]


extern crate panic_itm;

use cortex_m;
use cortex_m_rt::entry;

use stm32h7xx_hal::{
    adc,
    pac,
    delay::Delay,
    prelude::*,
};

use cortex_m_log::println;
use cortex_m_log::{
    destination::Itm, printer::itm::InterruptSync as InterruptSyncItm,
};
use cortex_m_semihosting::hprintln;

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
    // setting this per_ck to 4 mhz here as the last bit doesnt get sampled
    // if we wouldn't do that
    let mut ccdr = rcc.sys_ck(100.mhz()).per_ck(4.mhz()).freeze(vos, &dp.SYSCFG);

    println!(log, "");
    println!(log, "stm32h7xx-hal example - ADC");
    println!(log, "");

    let mut delay = Delay::new(cp.SYST, ccdr.clocks);

    // Setup ADC
    let mut adc1 = adc::Adc::adc3(dp.ADC3, &mut delay, &mut ccdr);
    adc1.set_resolution(adc::AdcSampleResolution::B_16);

    // Setup GPIOB
    let gpioc = dp.GPIOC.split(&mut ccdr.ahb4);

    // Configure pb0, pb1 as an analog input
    let mut channel = gpioc.pc0.into_analog();

    loop {
        let data: u32 = adc1.read(&mut channel).unwrap();
        // voltage = reading * (vref/resolution)
        println!(log, "ADC reading: {}, voltage for nucleo: {}", data, data as f32 * (3.3/65535.0));
        hprintln!( "ADC reading: {}, voltage for nucleo: {}", data, data as f32 * (3.3/65535.0)).unwrap();
    }
}
