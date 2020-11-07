//! Demo for STM32H747I-NUCLEO eval board using the Real Time Interrupt-driven Concurrency (RTIC)
//! framework.
//!
//! This example demonstrates using DMA to continuously write a data stream over SPI.
#![deny(warnings)]
#![no_main]
#![no_std]

use core::{mem, mem::MaybeUninit};

use embedded_hal::digital::v2::OutputPin;
use rtic::app;

#[macro_use]
#[allow(unused)]
mod utilities;
use log::info;

use hal::prelude::*;
use stm32h7xx_hal as hal;

// The number of bytes to transfer.
const BUFFER_SIZE: usize = 100;

// DMA1/DMA2 cannot interact with our stack. Instead, buffers for use with the
// DMA must be placed somewhere that DMA1/DMA2 can access. In this case we use
// AXI SRAM.
//
// The runtime does not initialise these SRAM banks
#[link_section = ".axisram.buffers"]
static mut BUFFER: MaybeUninit<[u8; BUFFER_SIZE]> = MaybeUninit::uninit();

#[app(device = stm32h7xx_hal::stm32, peripherals = true)]
const APP: () = {
    struct Resources {
        transfer: hal::dma::Transfer<
            hal::dma::dma::Stream0<hal::stm32::DMA1>,
            hal::spi::Spi<hal::stm32::SPI2, hal::spi::Disabled, u8>,
            hal::dma::MemoryToPeripheral,
            &'static mut [u8; BUFFER_SIZE],
        >,
    }

    #[init]
    fn init(ctx: init::Context) -> init::LateResources {
        utilities::logger::init();

        // Initialise power...
        let pwr = ctx.device.PWR.constrain();
        let pwrcfg = example_power!(pwr).freeze();

        // Initialise clocks...
        let rcc = ctx.device.RCC.constrain();
        let ccdr = rcc
            .sys_ck(200.mhz())
            .hclk(200.mhz())
            .freeze(pwrcfg, &ctx.device.SYSCFG);

        let gpiob = ctx.device.GPIOB.split(ccdr.peripheral.GPIOB);

        // Initialize a SPI transmitter on SPI2.
        let spi = {
            let mosi = gpiob.pb15.into_alternate_af5();
            let sck = gpiob.pb13.into_alternate_af5();
            let config = hal::spi::Config::new(hal::spi::MODE_0)
                .communication_mode(hal::spi::CommunicationMode::Transmitter);

            let spi: hal::spi::Spi<_, _, u8> = ctx.device.SPI2.spi(
                (sck, hal::spi::NoMiso, mosi),
                config,
                3.mhz(),
                ccdr.peripheral.SPI2,
                &ccdr.clocks,
            );

            spi.disable()
        };

        let mut cs = gpiob.pb12.into_push_pull_output();
        cs.set_high().unwrap();

        // Initialize our transmit buffer.
        let buffer: &'static mut [u8; BUFFER_SIZE] = {
            let buf: &mut [MaybeUninit<u8>; BUFFER_SIZE] =
                unsafe { mem::transmute(&mut BUFFER) };

            for (i, value) in buf.iter_mut().enumerate() {
                unsafe {
                    value.as_mut_ptr().write(i as u8 + 0x60); // 0x60, 0x61, 0x62...
                }
            }

            unsafe { mem::transmute(buf) }
        };

        let streams = hal::dma::dma::StreamsTuple::new(
            ctx.device.DMA1,
            ccdr.peripheral.DMA1,
        );

        // Configure the DMA stream to increment the memory address and generate a transfer complete
        // interrupt so we know when transmission is done.
        let config = hal::dma::dma::DmaConfig::default()
            .memory_increment(true)
            .transfer_complete_interrupt(true);

        let mut transfer: hal::dma::Transfer<
            _,
            _,
            hal::dma::MemoryToPeripheral,
            _,
        > = hal::dma::Transfer::init(streams.0, spi, buffer, None, config);

        transfer.start(|spi| {
            // For this example, we will always drive CSn as we are simply streaming data over SPI.
            cs.set_low().unwrap();

            // Enable TX DMA support, enable the SPI peripheral, and start the transaction.
            spi.inner_mut().cfg1.modify(|_, w| w.txdmaen().enabled());
            spi.inner_mut().cr1.modify(|_, w| w.spe().enabled());
            spi.inner_mut().cr1.modify(|_, w| w.cstart().started());

            // The transaction immediately begins as the TX FIFO is now being filled by DMA.
        });

        init::LateResources { transfer }
    }

    #[task(binds=DMA1_STR0, resources=[transfer])]
    fn dma_complete(ctx: dma_complete::Context) {
        info!("DMA transmission completed!");

        // If desired, the transfer can scheduled again here to continue transmitting.
        ctx.resources.transfer.clear_transfer_complete_interrupt();
    }

    #[idle]
    fn idle(_c: idle::Context) -> ! {
        loop {}
    }
};
