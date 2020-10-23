//! Example that transmits SPI data using the DMA

#![allow(clippy::transmute_ptr_to_ptr)]
#![deny(warnings)]
#![no_main]
#![no_std]

use core::{mem, mem::MaybeUninit};

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
static mut BUFFER: MaybeUninit<[u8; 10]> = MaybeUninit::uninit();

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
        .sys_ck(400.mhz())
        .pll1_q_ck(200.mhz())
        .freeze(pwrcfg, &dp.SYSCFG);

    // Acquire the GPIOC peripheral. This also enables the clock for
    // GPIOC in the RCC register.
    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);

    let sck = gpioa.pa12.into_alternate_af5();
    let miso = gpioc.pc2.into_alternate_af5();
    let mosi = gpioc.pc3.into_alternate_af5();
    let _nss = gpioa.pa11.into_alternate_af5();

    info!("");
    info!("stm32h7xx-hal example - SPI DMA");
    info!("");

    // Initialise the SPI peripheral.
    let spi: spi::Spi<_, _, u8> = dp.SPI2.spi(
        (sck, miso, mosi),
        spi::MODE_0,
        1.mhz(),
        ccdr.peripheral.SPI2,
        &ccdr.clocks,
    );

    // SPI must be disabled to configure DMA
    let mut spi = spi.disable();

    // Initialise the source buffer, without taking any references to
    // uninitialisated memory
    let buffer: &'static mut [u8; 10] = {
        let buf: &mut [MaybeUninit<u8>; 10] =
            unsafe { mem::transmute(&mut BUFFER) };

        for (i, value) in buf.iter_mut().enumerate() {
            unsafe {
                value.as_mut_ptr().write(i as u8 + 96); // 0x60, 0x61, 0x62...
            }
        }
        unsafe { mem::transmute(buf) }
    };

    // Setup the DMA transfer on stream 0
    //
    // We need to specify the direction with a type annotation, since DMA
    // transfers both to and from the SPI are possible
    let streams = StreamsTuple::new(dp.DMA1, ccdr.peripheral.DMA1);

    let config = DmaConfig::default().memory_increment(true);

    let mut transfer: Transfer<_, _, MemoryToPeripheral, _> = Transfer::init(
        streams.0,
        spi.inner_mut(), // Mutable reference to SPI register block
        buffer,
        None,
        config,
    );

    transfer.start(|spi| {
        // This closure runs right after enabling the stream

        // Enable DMA Tx buffer by setting the TXDMAEN bit in the SPI_CFG1
        // register
        spi.cfg1.modify(|_, w| w.txdmaen().enabled());

        // Enable the SPI by setting the SPE bit
        spi.cr1
            .write(|w| w.ssi().slave_not_selected().spe().enabled());

        // write CSTART to start a transaction in master mode
        spi.cr1.modify(|_, w| w.cstart().started());
    });

    // Wait for transfer to complete
    while !transfer.get_transfer_complete_flag() {}

    info!("Transfer complete!");

    loop {}
}
