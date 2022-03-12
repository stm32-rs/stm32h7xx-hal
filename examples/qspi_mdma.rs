//! Example of Memory to Peripheral Transfer to the QSPI peripheral using the
//! Master DMA (MDMA)
//!
//! Tested on a STM32H747I-DISCO development board

#![deny(warnings)]
#![no_main]
#![no_std]

#[macro_use]
mod utilities;

use core::mem;

use cortex_m_rt::entry;
use stm32h7xx_hal::{pac, prelude::*, xspi, xspi::QspiMode};

use stm32h7xx_hal::dma::{
    mdma::{
        MdmaConfig, MdmaIncrement, MdmaTransferRequest, MdmaTrigger,
        StreamsTuple,
    },
    MemoryToPeripheral, Transfer,
};

use log::info;

#[entry]
fn main() -> ! {
    utilities::logger::init();
    let dp = pac::Peripherals::take().unwrap();

    // Constrain and Freeze power
    let pwr = dp.PWR.constrain();
    let pwrcfg = example_power!(pwr).freeze();

    // Constrain and Freeze clock
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sys_ck(200.mhz()).freeze(pwrcfg, &dp.SYSCFG);

    // Acquire the GPIO peripherals. This also enables the clock for
    // the GPIOs in the RCC register.
    let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);
    let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);
    let gpiof = dp.GPIOF.split(ccdr.peripheral.GPIOF);
    let gpiog = dp.GPIOG.split(ccdr.peripheral.GPIOG);

    let _qspi_cs = gpiog.pg6.into_alternate::<10>();

    let sck = gpiof.pf10.into_alternate();
    let io0 = gpiof.pf8.into_alternate();
    let io1 = gpiod.pd12.into_alternate();
    let io2 = gpioe.pe2.into_alternate();
    let io3 = gpiod.pd13.into_alternate();

    info!("");
    info!("stm32h7xx-hal example - QSPI with MDMA");
    info!("");

    let config: xspi::Config = 1.mhz().into();
    // Threshold when half full
    let config = config.fifo_threshold(16);

    // Initialise the QSPI peripheral.
    let mut qspi = dp.QUADSPI.bank1(
        (sck, io0, io1, io2, io3),
        config,
        &ccdr.clocks,
        ccdr.peripheral.QSPI,
    );
    qspi.configure_mode(QspiMode::FourBit).unwrap();
    // Disable address phase
    qspi.inner_mut()
        .ccr
        .modify(|_, w| unsafe { w.admode().bits(0) });

    // Source buffer in TCM
    let mut source_buffer: [u8; 80] = [0x4A; 80];

    for i in 0..20 {
        let j = i * 4;
        let i = i as u8;
        source_buffer[j + 0] |= ((i << 4) & 0x10) | ((i >> 1) & 1);
        source_buffer[j + 1] |= ((i << 2) & 0x10) | ((i >> 3) & 1);
        source_buffer[j + 2] |= ((i >> 0) & 0x10) | ((i >> 5) & 1);
        source_buffer[j + 3] |= ((i >> 2) & 0x10) | ((i >> 7) & 1);
    }

    // Setup DMA
    let streams = StreamsTuple::new(dp.MDMA, ccdr.peripheral.MDMA);

    let config = MdmaConfig::default()
        .source_increment(MdmaIncrement::Increment)
        .destination_increment(MdmaIncrement::Fixed)
        // Triggered by hardware each buffer
        .hardware_transfer_request(MdmaTransferRequest::QuadspiFtTrg)
        .trigger_mode(MdmaTrigger::Buffer)
        // 16-byte buffer fits in QSPI FIFO
        .buffer_length(16);

    qspi.begin_write(0x00, 80).unwrap();

    let mut transfer: Transfer<
        _,
        _,
        MemoryToPeripheral,
        &'static mut [u8; 80],
        _,
    > = Transfer::init_master(
        streams.0,
        qspi,                                          // Dest: QSPI
        unsafe { mem::transmute(&mut source_buffer) }, // Source: TCM (stack)
        None,
        config,
    );

    // Buffer is 16-bytes to fit in QSPI FIFO
    assert_eq!(transfer.get_buffer_length(), 16);

    // Start block
    transfer.start(|_| {});

    // Wait for transfer to complete
    while !transfer.get_transfer_complete_flag() {}

    loop {
        cortex_m::asm::nop();
    }
}
