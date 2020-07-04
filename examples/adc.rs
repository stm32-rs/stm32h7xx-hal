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

    // We need to configure a clock for adc_ker_ck_input. The default
    // adc_ker_ck_input is pll2_p_ck, but we will use per_ck. Here we
    // set per_ck to 4MHz.
    //
    // The maximum adc_ker_ck_input frequency is 100MHz for revision V and 36MHz
    // otherwise
    let ccdr = rcc
        .sys_ck(100.mhz())
        .per_ck(4.mhz())
        .freeze(vos, &dp.SYSCFG);

    // Switch adc_ker_ck_input multiplexer to per_ck
    let d3ccipr = &unsafe { &*pac::RCC::ptr() }.d3ccipr;
    d3ccipr.modify(|_, w| unsafe { w.adcsel().bits(0b10) });

    println!(log, "");
    println!(log, "stm32h7xx-hal example - ADC");
    println!(log, "");

    let mut delay = Delay::new(cp.SYST, ccdr.clocks);

    // Setup ADC
    let mut adc3 =
        adc::Adc::adc3(dp.ADC3, &mut delay, ccdr.peripheral.ADC3, &ccdr.clocks)
            .enable();
    adc3.set_resolution(adc::Resolution::SIXTEENBIT);

    // Setup GPIOC
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);

    // Configure pc0 as an analog input
    let mut channel = gpioc.pc0.into_analog();

    loop {
        let data: u32 = adc3.read(&mut channel).unwrap();
        // voltage = reading * (vref/resolution)
        println!(
            log,
            "ADC reading: {}, voltage for nucleo: {}",
            data,
            data as f32 * (3.3 / adc3.max_sample() as f32)
        );
    }
}
