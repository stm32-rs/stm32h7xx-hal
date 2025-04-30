//! Example that transmits SPI data using the DMA
//!
//! The first part of the example transmits 10 bytes over SPI.
//!
//! The maximum transfer length for DMA1/DMA2 is limited to 65_535 items by
//! hardware. The second part of this example demonstrates splitting a transfer
//! into chunks and using the `next_transfer_with` method to start each part of
//! the transfer.
#![deny(warnings)]
#![no_main]
#![no_std]

use core::{mem, mem::MaybeUninit};

// TODO: use core::cell::SyncUnsafeCell when stabilized rust-lang/rust#95439
use utilities::sync_unsafe_cell::SyncUnsafeCell;

use cortex_m_rt::entry;
#[macro_use]
mod utilities;
use stm32h7xx_hal::{pac, prelude::*, spi};

use stm32h7xx_hal::dma::{
    dma::{DmaConfig, StreamsTuple},
    MemoryToPeripheral, Transfer,
};

use log::info;

// DMA1/DMA2 cannot interact with our stack. Instead, buffers for use with the
// DMA must be placed somewhere that DMA1/DMA2 can access. In this case we use
// AXI SRAM.
//
// The runtime does not initialise these SRAM banks
#[link_section = ".axisram.buffers"]
static SHORT_BUFFER: MaybeUninit<SyncUnsafeCell<[u8; 10]>> =
    MaybeUninit::uninit();

#[link_section = ".axisram.buffers"]
static LONG_BUFFER: MaybeUninit<SyncUnsafeCell<[u8; 0x2_0010]>> =
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

    // Acquire the GPIOC peripheral. This also enables the clock for
    // GPIOC in the RCC register.
    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);

    let sck = gpioa.pa12.into_alternate();
    let miso = gpioc.pc2.into_alternate();
    let mosi = gpioc.pc3.into_alternate();
    let _nss = gpioa.pa11.into_alternate::<5>(); // SS/CS not used in this example

    info!("");
    info!("stm32h7xx-hal example - SPI DMA");
    info!("");

    // Initialise the SPI peripheral.
    let spi: spi::Spi<_, _, u8> = dp.SPI2.spi(
        (sck, miso, mosi),
        spi::MODE_0,
        1.MHz(),
        ccdr.peripheral.SPI2,
        &ccdr.clocks,
    );

    // SPI must be disabled to configure DMA
    let spi = spi.disable();

    // SHORT_BUFFER is located in .axisram.buffers, which is not initialized by
    // the runtime. We must manually initialize it to a valid value, without
    // taking a reference to the uninitialized value
    unsafe {
        let cell = SHORT_BUFFER.as_ptr();
        for i in 0..10 {
            core::ptr::addr_of_mut!((*SyncUnsafeCell::raw_get(cell))[i])
                .write(i as u8 + 96); // 0x60, 0x61, 0x62...
        }
    }
    // Now we can take a mutable reference to SHORT_BUFFER. To avoid aliasing,
    // this reference must only be taken once
    let short_buffer: &mut [u8; 10] =
        unsafe { &mut *SyncUnsafeCell::raw_get(SHORT_BUFFER.as_ptr()) };

    // LONG_BUFFER is located in .axisram.buffers, which is not initialized by
    // the runtime. We must manually initialize it to a valid value, without
    // taking a reference to the uninitialized value
    unsafe {
        let cell = LONG_BUFFER.as_ptr();
        for i in 0..0x2_0010 {
            core::ptr::addr_of_mut!((*SyncUnsafeCell::raw_get(cell))[i])
                .write(i as u8);
        }
    }
    // Now we can take a mutable reference to LONG_BUFFER. To avoid aliasing,
    // this reference must only be taken once
    let long_buffer: &mut [u8; 0x2_0010] =
        unsafe { &mut *SyncUnsafeCell::raw_get(LONG_BUFFER.as_ptr()) };

    // Setup the DMA transfer on stream 0
    //
    // We need to specify the direction with a type annotation, since DMA
    // transfers both to and from the SPI are possible
    let streams = StreamsTuple::new(dp.DMA1, ccdr.peripheral.DMA1);

    let config = DmaConfig::default().memory_increment(true);

    let mut transfer: Transfer<_, _, MemoryToPeripheral, _, _> =
        Transfer::init(streams.0, spi, &mut short_buffer[..], None, config);

    transfer.start(|spi| {
        // This closure runs right after enabling the stream

        // Enable DMA Tx buffer by setting the TXDMAEN bit in the SPI_CFG1
        // register
        spi.enable_dma_tx();

        // Enable the SPI by setting the SPE bit
        spi.inner_mut()
            .cr1
            .write(|w| w.ssi().slave_not_selected().spe().enabled());

        // write CSTART to start a transaction in master mode
        spi.inner_mut().cr1.modify(|_, w| w.cstart().started());
    });

    // Wait for transfer to complete
    while !transfer.get_transfer_complete_flag() {}

    info!("Continuing with chunked transfer!");

    // Split the long buffer into chunks: Hardware supports 65_535 max.
    //
    // The last chunk will be length 16, compared to all the others which will
    // be length 32_768.
    for mut chunk in &mut long_buffer.chunks_mut(32_768) {
        // Using `next_transfer_with`
        let _current = transfer
            .next_transfer_with(|mut old, current, remaining| {
                // Check that we really did complete the current transfer
                assert_eq!(remaining, 0);

                mem::swap(&mut old, &mut chunk);

                (old, current)
            })
            .unwrap();

        // Using `next_transfer`: this is equivalent to the above (except the
        // assert) but less flexible
        //transfer.next_transfer(chunk).unwrap();

        // Wait for transfer to complete
        while !transfer.get_transfer_complete_flag() {}
    }

    transfer.pause(|spi| {
        // At this point, the DMA transfer is done, but the data is still in the
        // SPI output FIFO. Wait for it to complete
        while spi.inner().sr.read().txc().bit_is_clear() {}
    });

    info!("Chunked transfer complete!");

    let (_stream, _spi, _, _) = transfer.free();

    // We could re-use the stream or spi here

    loop {
        cortex_m::asm::nop()
    }
}
