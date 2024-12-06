//! Example of Memory to Memory Transfer with the Master DMA (MDMA)

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
static mut SOURCE_BUFFER: MaybeUninit<[u32; 200]> = MaybeUninit::uninit();

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
        .sys_ck(100.MHz())
        .hclk(50.MHz())
        .freeze(pwrcfg, &dp.SYSCFG);

    info!("");
    info!("stm32h7xx-hal example - Memory to TCM with Master DMA");
    info!("");

    // Initialise the source buffer without taking any references to
    // uninitialisated memory
    let source_buffer: &'static mut [u32; 200] = {
        let buf: &mut [MaybeUninit<u32>; 200] = unsafe {
            &mut *(core::ptr::addr_of_mut!(SOURCE_BUFFER)
                as *mut [MaybeUninit<u32>; 200])
        };

        for value in buf.iter_mut() {
            unsafe {
                value.as_mut_ptr().write(0x11223344u32);
            }
        }
        unsafe { SOURCE_BUFFER.assume_init_mut() }
    };

    //
    // Example 1: Memory to TCM
    //

    // Target buffer on the stack
    let mut target_buffer: [u32; 200] = [0; 200];

    // Setup DMA
    let streams = StreamsTuple::new(dp.MDMA, ccdr.peripheral.MDMA);

    let config = MdmaConfig::default()
        // increment by one element per element
        .destination_increment(MdmaIncrement::Increment)
        .source_increment(MdmaIncrement::Increment);

    let mut transfer: Transfer<_, _, MemoryToMemory<u32>, _, _> = {
        // Extend the lifetime of our data on the stack. We assert that it lives
        // as long as this transfer
        let target: &'static mut [u32; 200] =
            unsafe { mem::transmute(&mut target_buffer) };

        Transfer::init_master(
            streams.0,
            MemoryToMemory::new(),
            target,              // Dest: TCM (stack)
            Some(source_buffer), // Source: AXISRAM
            config,
        )
    };

    // Block of 800 bytes, MDMA checks other streams every 128 bytes
    assert_eq!(transfer.get_block_length(), 800);
    assert_eq!(transfer.get_buffer_length(), 128);

    // Start block
    transfer.start(|_| {});

    // Wait for transfer to complete
    while !transfer.get_transfer_complete_flag() {}

    // Decompose the stream to get the source buffer back
    let (stream, _mem2mem, _target, _) = transfer.free();

    for a in target_buffer.iter() {
        assert_eq!(*a, 0x11223344);
    }

    info!("Example 1: Memory to TCM DMA completed successfully");

    //
    // Example 2: Memory to TCM with endianess and offset
    //

    // Reset source buffer
    let source_buffer = unsafe { SOURCE_BUFFER.assume_init_mut() };
    *source_buffer = [0xAABBCCDD; 200];

    // New target buffer on the stack
    let mut target_buffer: [u32; 20] = [0; 20];

    let config = MdmaConfig::default()
        .source_increment(MdmaIncrement::Increment)
        .destination_increment(MdmaIncrement::IncrementWithOffset(
            MdmaSize::DoubleWord,
        ))
        .half_word_endianness_exchange(true);

    let mut transfer: Transfer<_, _, MemoryToMemory<u32>, _, _> = {
        let target: &'static mut [u32; 20] =
            unsafe { mem::transmute(&mut target_buffer) };

        // Note that our source and destination buffers now have different types
        // (they are arrays with different lengths). We pass slices instead, and
        // the HAL takes the length into account.

        Transfer::init_master(
            stream,
            MemoryToMemory::new(),
            &mut target[..],              // Dest: TCM (stack)
            Some(&mut source_buffer[..]), // Source: AXISRAM
            config,
        )
    };

    // Block length is limited to the minimum number of bytes that are valid for
    // both buffers. For this configuration, it is only possible to write 40
    // bytes (10 words) to the target buffer before reaching the end.
    assert_eq!(transfer.get_block_length(), 40);

    transfer.start(|_| {});

    // Wait for transfer to complete
    while !transfer.get_transfer_complete_flag() {}

    assert_eq!(
        target_buffer,
        [
            0xCCDDAABB, 0, 0xCCDDAABB, 0, 0xCCDDAABB, 0, 0xCCDDAABB, 0,
            0xCCDDAABB, 0, 0xCCDDAABB, 0, 0xCCDDAABB, 0, 0xCCDDAABB, 0,
            0xCCDDAABB, 0, 0xCCDDAABB, 0,
        ]
    );

    info!(
        "Example 2: Memory to TCM DMA with endianess and offset completed successfully"
    );

    //
    // Example 3: TCM to TCM with offset
    //

    let mut source_buffer_tcm = [1u8, 2];
    // unsafe: we must ensure source_buffer_tcm lives long enough
    let source_buffer: &'static mut [u8] =
        unsafe { mem::transmute(&mut source_buffer_tcm[..]) };
    let mut target_buffer = [0u8; 17];

    let config = MdmaConfig::default().destination_increment(
        MdmaIncrement::IncrementWithOffset(MdmaSize::DoubleWord),
    );

    let mut transfer: Transfer<_, _, MemoryToMemory<u8>, _, _> = {
        // Be very careful when using an unsafe transmute in the init call like
        // this because the target_buffer type will be transmuted to the source
        // type. In this case it's ok at the source_buffer is a slice [u8]. But
        // if both source and target types were arrays, length of the target
        // array would be lost.

        Transfer::init_master(
            streams.1,
            MemoryToMemory::new(),
            unsafe { mem::transmute::<&mut [u8], &mut [u8]>(&mut target_buffer[..]) }, // Dest: TCM (stack)
            Some(source_buffer), // Source: TCM (stack)
            config,
        )
    };

    transfer.start(|_| {});

    // Wait for transfer to complete
    while !transfer.get_transfer_complete_flag() {}

    // Value returned by `get_block_length` decrements during the transfer,
    // reaching zero at the end
    assert_eq!(transfer.get_block_length(), 0);

    assert_eq!(source_buffer_tcm, [1, 2]);
    assert_eq!(
        target_buffer,
        [1, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0]
    );

    info!("Example 3: TCM to TCM DMA with offset completed successfully");

    loop {
        cortex_m::asm::nop()
    }
}
