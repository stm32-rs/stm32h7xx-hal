//! I2C4 in low power mode.
//!
//!

#![allow(clippy::transmute_ptr_to_ptr)]
#![deny(warnings)]
#![no_std]
#![no_main]

use core::{mem, mem::MaybeUninit};

#[macro_use]
mod utilities;

use stm32h7xx_hal::dma::{
    bdma::{BdmaConfig, StreamsTuple},
    PeripheralToMemory, Transfer,
};

use stm32h7xx_hal::prelude::*;
use stm32h7xx_hal::{i2c, pac, pac::interrupt, rcc::LowPowerMode};

use cortex_m_rt::entry;

use log::info;

// The BDMA can only interact with SRAM4.
//
// The runtime does not initialise this SRAM bank
#[link_section = ".sram4.buffers"]
static mut BUFFER: MaybeUninit<[u8; 10]> = MaybeUninit::uninit();

#[entry]
fn main() -> ! {
    utilities::logger::init();
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().expect("Cannot take peripherals");

    // Run D3 / SRD domain
    #[cfg(not(feature = "rm0455"))]
    dp.PWR.cpucr.modify(|_, w| w.run_d3().set_bit());
    #[cfg(feature = "rm0455")]
    dp.PWR.cpucr.modify(|_, w| w.run_srd().set_bit());

    let pwr = dp.PWR.constrain();
    let pwrcfg = example_power!(pwr).freeze();

    // RCC
    let rcc = dp.RCC.constrain();
    let ccdr = rcc
        .sys_ck(200.mhz())
        // D3 / SRD domain
        .hclk(200.mhz()) // rcc_hclk4
        .pclk4(50.mhz()) // rcc_pclk4
        .freeze(pwrcfg, &dp.SYSCFG);

    // GPIO
    let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);

    // Configure the SCL and the SDA pin for our I2C bus
    let scl = gpiod.pd12.into_alternate().set_open_drain();
    let sda = gpiod.pd13.into_alternate().set_open_drain();

    let mut i2c = dp.I2C4.i2c(
        (scl, sda),
        100.khz(),
        ccdr.peripheral.I2C4.low_power(LowPowerMode::Autonomous),
        &ccdr.clocks,
    );

    // Use RX DMA
    i2c.rx_dma(true);

    // Listen for the end of i2c transactions
    i2c.clear_irq(i2c::Event::Stop);
    i2c.listen(i2c::Event::Stop);
    unsafe {
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::I2C4_EV);
    }

    // Setup the DMA transfer on stream 0
    #[cfg(not(feature = "rm0455"))]
    let streams = StreamsTuple::new(
        dp.BDMA,
        ccdr.peripheral.BDMA.low_power(LowPowerMode::Autonomous),
    );
    #[cfg(feature = "rm0455")]
    let streams = StreamsTuple::new(
        dp.BDMA2,
        ccdr.peripheral.BDMA2.low_power(LowPowerMode::Autonomous),
    );

    let config = BdmaConfig::default().memory_increment(true);

    // We need to specify the direction with a type annotation
    let mut transfer: Transfer<_, _, PeripheralToMemory, _, _> = Transfer::init(
        streams.0,
        i2c,
        unsafe { &mut BUFFER }, // uninitialised memory
        None,
        config,
    );

    transfer.start(|i2c| {
        // This closure runs right after enabling the stream

        // Issue the first part of an I2C transaction to read data from a
        // touchscreen

        // Write register index
        i2c.write(0xBA >> 1, &[0x41, 0xE4]).unwrap();

        // Start a read of 10 bytes
        i2c.master_read(0xBA >> 1, 10, i2c::Stop::Automatic);
    });

    // Enter CStop mode on wfi
    let mut scb = cp.SCB;
    scb.set_sleepdeep();

    loop {
        cortex_m::asm::wfi();
    }
}

#[interrupt]
fn I2C4_EV() {
    info!("I2C transfer complete!");

    // Look at BUFFER, which we expect to be initialised
    let buffer: &'static mut [u8; 10] = unsafe { mem::transmute(&mut BUFFER) };

    assert_eq!(buffer[0], 0xBE);

    loop {
        cortex_m::asm::nop()
    }
}
