//! Example of Memory to Memory Transfer with the DMA

#![deny(warnings)]
#![no_main]
#![no_std]

use core::mem::MaybeUninit;

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
static mut SOURCE_BUFFER: MaybeUninit<[u32; 20]> = MaybeUninit::uninit();
#[link_section = ".axisram.buffers"]
static mut TARGET_BUFFER: MaybeUninit<[u32; 20]> = MaybeUninit::uninit();

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

    // Initialise the source buffer with truly random data, without taking any
    // references to uninitialisated memory
    let source_buffer: &'static mut [u32; 20] = {
        let buf: &mut [MaybeUninit<u32>; 20] = unsafe {
            &mut *(core::ptr::addr_of_mut!(SOURCE_BUFFER)
                as *mut [MaybeUninit<u32>; 20])
        };

        for value in buf.iter_mut() {
            unsafe {
                value.as_mut_ptr().write(rng.gen().unwrap());
            }
        }
        #[allow(static_mut_refs)] // TODO: Fix this
        unsafe {
            SOURCE_BUFFER.assume_init_mut()
        }
    };
    // Save a copy on the stack so we can check it later
    let source_buffer_cloned = *source_buffer;

    // NOTE(unsafe): TARGET_BUFFER must also be initialised to prevent undefined
    // behaviour (taking a mutable reference to uninitialised memory)

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
            unsafe {
                (*core::ptr::addr_of_mut!(TARGET_BUFFER)).assume_init_mut()
            }, // Uninitialised memory
            Some(source_buffer),
            config,
        );

    transfer.start(|_| {});

    // Wait for transfer to complete
    while !transfer.get_transfer_complete_flag() {}

    // Now the target memory is actually initialised
    #[allow(static_mut_refs)] // TODO: Fix this
    let target_buffer: &'static mut [u32; 20] =
        unsafe { TARGET_BUFFER.assume_init_mut() };

    // Comparison check
    assert_eq!(&source_buffer_cloned, target_buffer);

    info!("Memory to Memory DMA completed successfully");

    loop {
        cortex_m::asm::nop()
    }
}
