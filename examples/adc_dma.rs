//! Example of reading ADC1 data using DMA1/2
//!
//! For an example of using ADC3, see examples/temperature.rs
//! For an example of using ADC1 and ADC2 together, see examples/adc12.rs
//! For an example of using ADC1 and ADC2 in parallel, see examples/adc12_parallel.rs

#![no_main]
#![no_std]

use core::{mem, mem::MaybeUninit};
use log::info;

use cortex_m_rt::entry;

use stm32h7xx_hal::dma::{
    config::BurstMode,
    dma::{DmaConfig, StreamsTuple},
    Transfer,
};
use stm32h7xx_hal::{adc, delay::Delay, pac, prelude::*};

#[macro_use]
mod utilities;

#[entry]
fn main() -> ! {
    utilities::logger::init();
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    #[link_section = ".axisram"]
    static mut BUFFER: MaybeUninit<[u16; 32_768]> = MaybeUninit::uninit();

    let adc_buffer: &'static mut [u16; 32_768] = {
        // Convert an uninitialised array into an array of uninitialised
        let buf: &mut [MaybeUninit<u16>; 32_768] =
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
    info!("stm32h7xx-hal example - ADC to Memory DMA");
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

    // Configure pc0 as an analog input
    let mut channel = gpioc.pc0.into_analog(); // ANALOG IN 10

    // 4-beat bursts can be used. See RM0433 Rev 7 Section 25.4.27 Subsection
    // DMA with FIFO.
    let config = DmaConfig::default()
        .memory_increment(true)
        .peripheral_burst(BurstMode::Burst4);

    // Setup the DMA transfer on stream 0
    let streams = StreamsTuple::new(dp.DMA1, ccdr.peripheral.DMA1);
    let mut transfer: Transfer<_, _, _, _, _> =
        Transfer::init(streams.0, adc1, &mut adc_buffer[..], None, config);

    transfer.start(|adc| {
        // This closure runs right after enabling the stream

        // Start a one-shot conversion for the length of this transfer
        adc.start_conversion_dma(&mut channel, adc::AdcDmaMode::OneShot);
    });

    // Wait for transfer to complete
    while !transfer.get_transfer_complete_flag() {}

    loop {
        // Dump contents from memory address 0x2400_0000 to 0x2401_0000

        cortex_m::asm::nop()
    }
}
