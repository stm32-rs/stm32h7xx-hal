#![no_main]
#![no_std]

use core::{mem, mem::MaybeUninit};
use log::info;

use cortex_m_rt::entry;

use embedded_hal::adc::Channel;
use stm32h7xx_hal::{
    adc,
    delay::Delay,
    dma::{
        dma::{DmaConfig, StreamsTuple},
        Transfer,
    },
    gpio, pac,
    prelude::*,
    stm32::ADC1,
};

#[macro_use]
mod utilities;

#[entry]
fn main() -> ! {
    utilities::logger::init();
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    // 1 slot because we are only using 1 channel
    #[link_section = ".axisram"]
    static mut BUFFER: MaybeUninit<[u16; 1]> = MaybeUninit::uninit();

    let adc_buffer: &'static mut [u16; 1] = {
        // Convert an uninitialised array into an array of uninitialised
        let buf: &mut [MaybeUninit<u16>; 1] =
            unsafe { mem::transmute(&mut BUFFER) };
        // Initialise memory to valid values
        for slot in buf.iter_mut() {
            // Never create even a _temporary_ reference to uninitialised memory
            unsafe {
                slot.as_mut_ptr().write(0);
            }
        }
        unsafe { mem::transmute(buf) }
    };

    // Constrain and Freeze power
    info!("Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    let pwrcfg = example_power!(pwr).freeze();

    // Constrain and Freeze clock
    info!("Setup RCC...                  ");
    let rcc = dp.RCC.constrain();

    let ccdr = rcc
        .sys_ck(400.MHz())
        .pll2_p_ck(40.MHz())
        .freeze(pwrcfg, &dp.SYSCFG);

    info!("");
    info!("stm32h7xx-hal example - ADC to Memory DMA Continuous using Circular Mode");
    info!("");

    let mut delay = Delay::new(cp.SYST, ccdr.clocks);

    // Setup ADC
    let mut adc1 = adc::Adc::adc1(
        dp.ADC1,
        2.MHz(),
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

    // Configure pc4 as an analog input
    let mut channel = gpioc.pc4.into_analog(); // ANALOG IN 4

    let config = DmaConfig::default()
        .circular_buffer(true)
        .memory_increment(true);

    // Setup the DMA transfer on stream 0
    let streams = StreamsTuple::new(dp.DMA1, ccdr.peripheral.DMA1);
    let mut transfer: Transfer<_, _, _, _, _> =
        Transfer::init(streams.0, adc1, &mut adc_buffer[..], None, config);

    // This closure runs right after enabling the stream
    transfer.start(|adc| {
        let channel = <gpio::PC4 as Channel<ADC1>>::channel();
        adc.start_conversion_dma_circ(&[channel]); // more channels can be added to the list
    });

    loop {
        info!("{}", unsafe {
            core::ptr::read_volatile(0x2400_0000 as *const u16)
            // if more channels are used, each address will be offset by 2
        });

        delay.delay_ms(10_u16); // slow down the loop a bit
    }
}
