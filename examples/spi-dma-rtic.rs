//! Demo for STM32H747I-NUCLEO eval board using the Real Time Interrupt-driven Concurrency (RTIC)
//! framework.
//!
//! This example demonstrates using DMA to write data over a TX-only SPI interface.
#![deny(warnings)]
#![allow(clippy::type_complexity)]
#![no_main]
#![no_std]

use core::mem::MaybeUninit;

#[macro_use]
mod utilities;

// The number of bytes to transfer.
const BUFFER_SIZE: usize = 100;

// DMA1/DMA2 cannot interact with our stack. Instead, buffers for use with the
// DMA must be placed somewhere that DMA1/DMA2 can access. In this case we use
// AXI SRAM.
//
// The runtime does not initialise these SRAM banks
#[link_section = ".axisram.buffers"]
static mut BUFFER: MaybeUninit<[u8; BUFFER_SIZE]> = MaybeUninit::uninit();

#[rtic::app(device = stm32h7xx_hal::stm32, peripherals = true)]
mod app {
    use embedded_hal::digital::v2::OutputPin;
    use hal::prelude::*;
    use stm32h7xx_hal as hal;

    use super::*;

    #[shared]
    struct SharedResources {
        cs: hal::gpio::gpiob::PB12<hal::gpio::Output<hal::gpio::PushPull>>,
        transfer: hal::dma::Transfer<
            hal::dma::dma::Stream1<hal::stm32::DMA1>,
            hal::spi::Spi<hal::stm32::SPI2, hal::spi::Disabled, u8>,
            hal::dma::MemoryToPeripheral,
            &'static mut [u8; BUFFER_SIZE],
            hal::dma::DBTransfer,
        >,
    }

    #[local]
    struct LocalResources {}
    #[init]
    fn init(
        ctx: init::Context,
    ) -> (SharedResources, LocalResources, init::Monotonics) {
        utilities::logger::init();

        // Initialise power...
        let pwr = ctx.device.PWR.constrain();
        let pwrcfg = example_power!(pwr).freeze();

        // Initialise clocks...
        let rcc = ctx.device.RCC.constrain();
        let ccdr = rcc
            .sys_ck(200.mhz())
            .hclk(200.mhz())
            .pll1_q_ck(200.mhz())
            .freeze(pwrcfg, &ctx.device.SYSCFG);

        let gpiob = ctx.device.GPIOB.split(ccdr.peripheral.GPIOB);

        // Initialize a SPI transmitter on SPI2.
        let spi = {
            let mosi = gpiob
                .pb15
                .into_alternate_af5()
                .set_speed(hal::gpio::Speed::VeryHigh);
            let sck = gpiob
                .pb10
                .into_alternate_af5()
                .set_speed(hal::gpio::Speed::VeryHigh);
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

        let mut cs = gpiob
            .pb12
            .into_push_pull_output()
            .set_speed(hal::gpio::Speed::VeryHigh);
        cs.set_high().unwrap();

        // Initialize our transmit buffer.
        let buffer: &'static mut [u8; BUFFER_SIZE] = {
            let buf: &mut [MaybeUninit<u8>; BUFFER_SIZE] = unsafe {
                &mut *(&mut BUFFER as *mut MaybeUninit<[u8; BUFFER_SIZE]>
                    as *mut [MaybeUninit<u8>; BUFFER_SIZE])
            };

            for (i, value) in buf.iter_mut().enumerate() {
                unsafe {
                    value.as_mut_ptr().write(i as u8 + 0x60); // 0x60, 0x61, 0x62...
                }
            }

            unsafe {
                &mut *(buf as *mut [MaybeUninit<u8>; BUFFER_SIZE]
                    as *mut [u8; BUFFER_SIZE])
            }
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

        let transfer: hal::dma::Transfer<
            _,
            _,
            hal::dma::MemoryToPeripheral,
            _,
            _,
        > = hal::dma::Transfer::init(streams.1, spi, buffer, None, config);

        (
            SharedResources { cs, transfer },
            LocalResources {},
            init::Monotonics(),
        )
    }

    #[task(binds=DMA1_STR1, shared = [transfer, cs], priority=2)]
    fn dma_complete(mut ctx: dma_complete::Context) {
        // If desired, the transfer can scheduled again here to continue transmitting.
        let mut cs = ctx.shared.cs;
        ctx.shared.transfer.lock(|transfer| {
            cs.lock(|cs| {
                transfer.clear_transfer_complete_interrupt();
                transfer.pause(|spi| {
                    // At this point, the DMA transfer is done, but the data is still in the SPI output
                    // FIFO. Wait for it to complete before disabling CS.
                    while spi.inner().sr.read().txc().bit_is_clear() {}
                    cs.set_high().unwrap();
                });
            });
        });
    }

    #[idle(shared = [transfer, cs])]
    fn idle(mut ctx: idle::Context) -> ! {
        // Start the DMA transfer over SPI.
        let mut cs = ctx.shared.cs;
        ctx.shared.transfer.lock(|transfer| {
            cs.lock(|cs| {
                transfer.start(|spi| {
                    // Set CS low for the transfer.
                    cs.set_low().unwrap();

                    // Enable TX DMA support, enable the SPI peripheral, and start the transaction.
                    spi.enable_dma_tx();
                    spi.inner_mut().cr1.modify(|_, w| w.spe().enabled());
                    spi.inner_mut().cr1.modify(|_, w| w.cstart().started());

                    // The transaction immediately begins as the TX FIFO is now being filled by DMA.
                });
            });
        });

        loop {
            cortex_m::asm::nop();
        }
    }
}
