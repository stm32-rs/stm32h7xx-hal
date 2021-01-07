//! Example of Memory to Memory Transfer with the Master DMA (MDMA)

#![allow(clippy::transmute_ptr_to_ptr)]
#![deny(warnings)]
#![no_main]
#![no_std]

use core::{mem, mem::MaybeUninit};

use cortex_m_rt::entry;
#[macro_use]
mod utilities;
use stm32h7xx_hal::{pac, prelude::*};

use stm32h7xx_hal::dma::{
    mdma::{MdmaConfig, MdmaIncrement, StreamsTuple},
    traits::Direction,
    MemoryToMemory, Transfer,
};

use log::info;

// The MDMA can interact with SRAM banks as well as the TCM. For this example,
// we place the source in the AXI SRAM.
//
// The runtime does not initialise AXI SRAM banks.
#[link_section = ".axisram.buffers"]
static mut SOURCE_BUFFER: MaybeUninit<[u32; 20]> = MaybeUninit::uninit();

#[entry]
fn main() -> ! {
    utilities::logger::init();
    let dp = pac::Peripherals::take().unwrap();

    // Constrain and Freeze power
    info!("Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    let pwrcfg = example_power!(pwr).freeze();

    // Constrain and Freeze clock
    info!("Setup RCC...                  ");
    let rcc = dp.RCC.constrain();
    let ccdr = rcc
        .sys_ck(100.mhz())
        .hclk(50.mhz())
        .freeze(pwrcfg, &dp.SYSCFG);

    info!("");
    info!("stm32h7xx-hal example - Memory to TCM with Master DMA");
    info!("");

    // Initialise the source buffer without taking any references to
    // uninitialisated memory
    let source_buffer: &'static mut [u32; 20] = {
        let buf: &mut [MaybeUninit<u32>; 20] =
            unsafe { mem::transmute(&mut SOURCE_BUFFER) };

        for value in buf.iter_mut() {
            unsafe {
                value.as_mut_ptr().write(0x11223344u32);
            }
        }
        unsafe { mem::transmute(buf) }
    };

    // Target buffer on the stack
    let mut target_buffer: [u32; 20] = [0; 20];

    // Setup DMA
    let streams = StreamsTuple::new(dp.MDMA, ccdr.peripheral.MDMA);

    let config = MdmaConfig::default()
        // increment by one element per transfer
        .destination_increment(MdmaIncrement::Increment)
        .source_increment(MdmaIncrement::Increment);

    let mut transfer: Transfer<_, _, MemoryToMemory<u32>, _> =
        Transfer::init_master(
            streams.0,
            MemoryToMemory::new(),
            unsafe { mem::transmute(&mut target_buffer) }, // Dest: TCM (stack)
            Some(source_buffer),                           // Source: AXISRAM
            config,
        );

    transfer.start(|_| {});

    // Wait for transfer to complete
    while !transfer.get_transfer_complete_flag() {}

    // Decompose the stream to get the source buffer back
    let (_stream, _mem2mem, _target, source_buffer_opt) = transfer.free();
    let _source_buffer = source_buffer_opt.unwrap();

    // Comparison check
    assert_eq!(target_buffer, [0x11223344; 20]);

    info!("Memory to TCM DMA completed successfully");

    loop {}
}
