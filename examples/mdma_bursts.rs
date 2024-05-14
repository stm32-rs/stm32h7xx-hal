//! Example of Memory to Memory Transfer with the Master DMA (MDMA) and multiple
//! beats per AXI burst. Using multiple beats in each burst results in fewer
//! total bursts.
//!
//! This example demonstrates a transfer with 1 beat/burst, and a 32
//! beats/burst. The latter gives an approximately 25% speedup.

#![deny(warnings)]
#![no_main]
#![no_std]

use core::mem::MaybeUninit;

use cortex_m_rt::entry;
#[macro_use]
mod utilities;
use stm32h7xx_hal::{pac, pac::DWT, prelude::*};

use stm32h7xx_hal::dma::{
    mdma::{MdmaConfig, MdmaIncrement, StreamsTuple},
    traits::{Direction, MasterStream, Stream},
    MemoryToMemory, Transfer,
};

use log::info;

// The MDMA can interact with SRAM banks as well as the TCM. For this example,
// we place the source and destination in AXI SRAM.
//
// The runtime does not initialise AXI SRAM banks.
#[link_section = ".axisram.buffers"]
static mut SOURCE_BUFFER: MaybeUninit<[u32; 200]> = MaybeUninit::uninit();
#[link_section = ".axisram.buffers"]
static mut TARGET_BUFFER: MaybeUninit<[u32; 200]> = MaybeUninit::uninit();

#[entry]
fn main() -> ! {
    utilities::logger::init();
    let cp = unsafe { cortex_m::Peripherals::steal() };
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

    // Cycle counter
    let mut dwt = cp.DWT;
    dwt.enable_cycle_counter();

    info!("");
    info!("stm32h7xx-hal example - Master DMA with longer bursts");
    info!("");

    // Initialise the source buffer without taking any references to
    // uninitialised memory
    let _source_buffer: &'static mut [u32; 200] = {
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

    // NOTE(unsafe): TARGET_BUFFER must also be initialised to prevent undefined
    // behaviour (taking a mutable reference to uninitialised memory)

    // Setup DMA
    let streams = StreamsTuple::new(dp.MDMA, ccdr.peripheral.MDMA);

    // Timed test method
    fn run_mdma_mem2mem<STREAM: MasterStream + Stream<Config = MdmaConfig>>(
        n: usize,
        m: usize,
        stream: STREAM,
        config: MdmaConfig,
    ) {
        let mut transfer: Transfer<_, _, MemoryToMemory<u32>, _, _> = {
            // unsafe: Both source and destination live at least as long as this
            // transfer
            let source: &'static mut [u32; 200] =
                unsafe { SOURCE_BUFFER.assume_init_mut() };
            let target: &'static mut [u32; 200] =
                unsafe { TARGET_BUFFER.assume_init_mut() }; // uninitialised memory

            Transfer::init_master(
                stream,
                MemoryToMemory::new(),
                target,       // Destination: AXISRAM
                Some(source), // Source: AXISRAM
                config,
            )
        };

        // Block of 800 bytes
        assert_eq!(transfer.get_block_length(), 800);

        // Burst lengths set correctly
        assert_eq!(transfer.get_source_burst_length(), m);
        assert_eq!(transfer.get_destination_burst_length(), m);

        let mut cycles = 0;
        for _ in 0..10 {
            cycles += {
                let start = DWT::cycle_count();

                // Start block
                transfer.start(|_| {});

                // Wait for transfer to complete
                while !transfer.get_transfer_complete_flag() {}

                DWT::cycle_count() - start
            };
        }

        // Decompose the stream to get the buffers back
        let (_stream0, _mem2mem, target_buffer, _) = transfer.free();

        for a in target_buffer.iter() {
            assert_eq!(*a, 0x11223344);
        }

        info!(
            "Example {}: Memory to Memory, {} beat/burst completed successfully",
            n, m
        );
        info!(
            "Cycles: {}, {:.2} cycles/B",
            cycles / 10,
            cycles as f32 / 8_000.
        );
        info!("");
    }

    //
    // Example 1: Memory to Memory, 1 beat per burst
    //
    let config_1beat = MdmaConfig::default()
        .destination_increment(MdmaIncrement::Increment)
        .source_increment(MdmaIncrement::Increment);

    //info!("Config 1: {:?}", config_1beat);

    run_mdma_mem2mem(1, 1, streams.0, config_1beat);

    //
    // Example 2: Memory to Memory, 32 beats per burst
    //
    let config_32beat = config_1beat
        .destination_burst_size(32)
        .source_burst_size(32);

    run_mdma_mem2mem(2, 32, streams.1, config_32beat);

    //
    // Example 3: Memory to Memory, 16 beats per burst
    //
    // buffer length limits the burst size
    let config_16beat = config_32beat.buffer_length(16);

    run_mdma_mem2mem(3, 16, streams.2, config_16beat);

    loop {
        cortex_m::asm::nop()
    }
}
