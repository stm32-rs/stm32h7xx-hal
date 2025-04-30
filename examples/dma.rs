//! Example of Memory to Memory Transfer with the DMA

#![deny(warnings)]
#![no_main]
#![no_std]

use core::mem::MaybeUninit;

// TODO: use core::cell::SyncUnsafeCell when stabilized rust-lang/rust#95439
use utilities::sync_unsafe_cell::SyncUnsafeCell;

use cortex_m_rt::entry;
#[macro_use]
mod utilities;
use stm32h7xx_hal::{pac, prelude::*};

use stm32h7xx_hal::dma::{
    dma::{DmaConfig, StreamsTuple},
    traits::Direction,
    MemoryToMemory, Transfer,
};

use log::info;

// DMA1/DMA2 cannot interact with our stack. Instead, buffers for use with the
// DMA must be placed somewhere that DMA1/DMA2 can access.
//
// The runtime does not initialise these SRAM banks.
#[link_section = ".sram4.buffers"]
static SOURCE_BUFFER: MaybeUninit<SyncUnsafeCell<[u32; 20]>> =
    MaybeUninit::uninit();
#[link_section = ".axisram.buffers"]
static TARGET_BUFFER: MaybeUninit<SyncUnsafeCell<[u32; 20]>> =
    MaybeUninit::uninit();

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
        .sys_ck(200.MHz())
        .pll1_q_ck(200.MHz())
        .freeze(pwrcfg, &dp.SYSCFG);

    // True RNG
    let mut rng = dp.RNG.constrain(ccdr.peripheral.RNG, &ccdr.clocks);

    info!("");
    info!("stm32h7xx-hal example - Memory to Memory DMA");
    info!("");

    // SOURCE_BUFFER is located in .axisram.buffers, which is not initialized by
    // the runtime. We must manually initialize it to a valid value, without
    // taking a reference to the uninitialized value
    unsafe {
        let cell = SOURCE_BUFFER.as_ptr();
        for i in 0..20 {
            core::ptr::addr_of_mut!((*SyncUnsafeCell::raw_get(cell))[i])
                .write(rng.gen().unwrap());
        }
    }
    // Now we can take a mutable reference to SOURCE_BUFFER. To avoid aliasing,
    // this reference must only be taken once
    let source_buffer =
        unsafe { &mut *SyncUnsafeCell::raw_get(SOURCE_BUFFER.as_ptr()) };
    // Save a copy on the stack so we can check it later
    let source_buffer_cloned = *source_buffer;

    // TARGET_BUFFER is located in .axisram.buffers, which is not initialized by
    // the runtime. We must manually initialize it to a valid value, without
    // taking a reference to the uninitialized value
    unsafe {
        let cell = TARGET_BUFFER.as_ptr();
        for i in 0..20 {
            core::ptr::addr_of_mut!((*SyncUnsafeCell::raw_get(cell))[i])
                .write(0);
        }
    }
    // Now we can take a mutable reference to TARGET_BUFFER. To avoid aliasing,
    // this reference must only be taken once
    let target_buffer =
        unsafe { &mut *SyncUnsafeCell::raw_get(TARGET_BUFFER.as_ptr()) };

    // Setup DMA
    //
    // We need to specify the transfer size with a type annotation

    let streams = StreamsTuple::new(dp.DMA1, ccdr.peripheral.DMA1);

    let config = DmaConfig::default()
        .memory_increment(true) // destination mem
        .peripheral_increment(true) // source mem
        .fifo_enable(true);

    let mut transfer: Transfer<_, _, MemoryToMemory<u32>, _, _> =
        Transfer::init(
            streams.4,
            MemoryToMemory::new(),
            target_buffer,
            Some(source_buffer),
            config,
        );

    transfer.start(|_| {});

    // Wait for transfer to complete
    while !transfer.get_transfer_complete_flag() {}

    // Take a second(!) reference to the target buffer. Because the transfer has
    // stopped, we can reason that the first reference is no longer
    // active. Therefore we can take another reference without causing aliasing
    let target_buffer: &[u32; 20] =
        unsafe { &*SyncUnsafeCell::raw_get(TARGET_BUFFER.as_ptr()) };

    // Comparison check
    assert_eq!(&source_buffer_cloned, target_buffer);

    info!("Memory to Memory DMA completed successfully");

    loop {
        cortex_m::asm::nop()
    }
}
