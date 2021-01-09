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
    mdma::{MdmaConfig, MdmaIncrement, MdmaSize, StreamsTuple},
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

    //
    // Example 1: Memory to TCM
    //

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
    let (stream, _mem2mem, _target, source_buffer_opt) = transfer.free();
    let source_buffer = source_buffer_opt.unwrap();

    assert_eq!(target_buffer, [0x11223344; 20]);

    info!("Example 1: Memory to TCM DMA completed successfully");

    //
    // Example 2: Memory to TCM with endianess and offset
    //

    // Reset source buffer
    *source_buffer = [0xAABBCCDD; 20];

    let config = MdmaConfig::default()
        .source_increment(MdmaIncrement::Increment)
        .destination_increment(MdmaIncrement::IncrementWithOffset(
            MdmaSize::DoubleWord,
        ))
        .half_word_endianness_exchange(true);

    let mut transfer: Transfer<_, _, MemoryToMemory<u32>, _> =
        Transfer::init_master(
            stream,
            MemoryToMemory::new(),
            unsafe { mem::transmute(&mut target_buffer) }, // Dest: TCM (stack)
            Some(source_buffer),                           // Source: AXISRAM
            config,
        );

    transfer.start(|_| {});

    // Wait for transfer to complete
    while !transfer.get_transfer_complete_flag() {}

    assert_eq!(
        target_buffer,
        [
            0xCCDDAABB, 0x11223344, 0xCCDDAABB, 0x11223344, 0xCCDDAABB,
            0x11223344, 0xCCDDAABB, 0x11223344, 0xCCDDAABB, 0x11223344,
            0xCCDDAABB, 0x11223344, 0xCCDDAABB, 0x11223344, 0xCCDDAABB,
            0x11223344, 0xCCDDAABB, 0x11223344, 0xCCDDAABB, 0x11223344,
        ]
    );

    info!(
        "Example 2: Memory to TCM DMA with endianess and offset completed successfully"
    );

    //
    // Example 3: TCM to TCM with offset
    //

    let mut source_buffer_tcm = [1u8, 2, 3, 4];
    // unsafe: we must ensure source_buffer_tcm lives long enough
    let source_buffer: &'static mut [u8] =
        unsafe { mem::transmute(&mut source_buffer_tcm[..]) };
    let mut target_buffer = [0u8; 17];

    let config = MdmaConfig::default().destination_increment(
        MdmaIncrement::IncrementWithOffset(MdmaSize::DoubleWord),
    );

    let mut transfer: Transfer<_, _, MemoryToMemory<u8>, _> =
        Transfer::init_master(
            streams.1,
            MemoryToMemory::new(),
            unsafe { mem::transmute(&mut target_buffer[..]) }, // Dest: TCM (stack)
            Some(source_buffer), // Source: TCM (stack)
            config,
        );

    // Transfer length is limited to the minimum number of bytes that are valid
    // for both buffers
    assert_eq!(transfer.get_transfer_bytes(), 3);

    transfer.start(|_| {});

    // Wait for transfer to complete
    while !transfer.get_transfer_complete_flag() {}

    assert_eq!(source_buffer_tcm, [1, 2, 3, 4]);
    assert_eq!(
        target_buffer,
        [1, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 3]
    );

    info!("Example 3: TCM to TCM DMA with offset completed successfully");

    loop {}
}
