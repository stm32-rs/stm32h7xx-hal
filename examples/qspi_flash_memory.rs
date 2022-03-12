//! Example of communication with flash memory over QSPI interface. This
//! specific program is made for Daisy Seed board and its IS25LP064A flash
//! memory chip. This demo provides a non-volatile restart counter. When it is
//! odd, LED will light up, when even, it stays off.

#![deny(warnings)]
#![no_main]
#![no_std]

#[macro_use]
mod utilities;

use core::mem::transmute;

use cortex_m_rt::entry;
use stm32h7xx_hal::gpio::Speed;
use stm32h7xx_hal::{
    pac, prelude::*, xspi::Qspi, xspi::QspiMode, xspi::QspiWord,
};

use log::info;

// SPI commands from IS25LP064 datasheet
const WRITE_STATUS_REGISTRY_CMD: u8 = 0x01; // WRSR
const WRITE_CMD: u8 = 0x02; // PP
const NORMAL_READ_CMD: u8 = 0x03; // NORD
const READ_STATUS_REGISTRY_CMD: u8 = 0x05; // RDSR
const WRITE_ENABLE_CMD: u8 = 0x06; // WREN
const SET_READ_PARAMETERS_CMD: u8 = 0xC0; // SRP
const SECTOR_ERASE_CMD: u8 = 0xD8; // SER

// Address in the external memory that will be used for the counter
const COUNTER_ADDRESS: u32 = 0x00;

#[entry]
fn main() -> ! {
    utilities::logger::init();
    let dp = pac::Peripherals::take().unwrap();

    // Constrain and Freeze power
    let pwr = dp.PWR.constrain();
    let pwrcfg = example_power!(pwr).freeze();

    // Constrain and Freeze clock
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sys_ck(96.mhz()).freeze(pwrcfg, &dp.SYSCFG);

    // Acquire the GPIO peripherals
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
    let gpiof = dp.GPIOF.split(ccdr.peripheral.GPIOF);
    let gpiog = dp.GPIOG.split(ccdr.peripheral.GPIOG);

    // Even though it is not directly used, CS pin must be acquired and configured
    let _qspi_cs = gpiog.pg6.into_alternate::<10>().set_speed(Speed::VeryHigh);

    let sck = gpiof.pf10.into_alternate().set_speed(Speed::VeryHigh);
    let io0 = gpiof.pf8.into_alternate().set_speed(Speed::VeryHigh);
    let io1 = gpiof.pf9.into_alternate().set_speed(Speed::VeryHigh);
    let io2 = gpiof.pf7.into_alternate().set_speed(Speed::VeryHigh);
    let io3 = gpiof.pf6.into_alternate().set_speed(Speed::VeryHigh);

    let mut led = gpioc.pc7.into_push_pull_output();

    info!("");
    info!("stm32h7xx-hal example - QSPI Flash Memory");
    info!("");

    // Initialise the QSPI peripheral
    let mut qspi = dp.QUADSPI.bank1(
        (sck, io0, io1, io2, io3),
        3.mhz(),
        &ccdr.clocks,
        ccdr.peripheral.QSPI,
    );
    qspi.configure_mode(QspiMode::OneBit).unwrap();

    // Reset configuration of the flash memory
    reset_status_registry(&mut qspi);
    reset_read_registry(&mut qspi);

    // Read the current value and increment it
    let original = read_u32(&mut qspi, COUNTER_ADDRESS);
    write_u32(&mut qspi, COUNTER_ADDRESS, original.overflowing_add(1).0);

    // Based on the original value, enable or disable LED on the board
    if original % 2 == 0 {
        led.set_low();
    } else {
        led.set_high();
    }

    loop {
        cortex_m::asm::nop();
    }
}

fn read_u32(qspi: &mut Qspi<pac::QUADSPI>, address: u32) -> u32 {
    let mut buffer: [u8; 4] = [0xFF; 4];
    qspi.read_extended(
        QspiWord::U8(NORMAL_READ_CMD),
        QspiWord::U24(address),
        QspiWord::None,
        0,
        &mut buffer,
    )
    .unwrap();
    u32::from_be_bytes(buffer)
}

fn write_u32(qspi: &mut Qspi<pac::QUADSPI>, address: u32, value: u32) {
    enable_write_operation(qspi);
    qspi.write_extended(
        QspiWord::U8(SECTOR_ERASE_CMD),
        QspiWord::U24(address),
        QspiWord::None,
        &[],
    )
    .unwrap();
    wait_for_write_completition(qspi);

    let bytes: [u8; 4] = unsafe { transmute(value.to_be()) };
    enable_write_operation(qspi);
    qspi.write_extended(
        QspiWord::U8(WRITE_CMD),
        QspiWord::U24(address),
        QspiWord::None,
        &bytes,
    )
    .unwrap();
    wait_for_write_completition(qspi);
}

fn reset_status_registry(qspi: &mut Qspi<pac::QUADSPI>) {
    enable_write_operation(qspi);

    qspi.write_extended(
        QspiWord::U8(WRITE_STATUS_REGISTRY_CMD),
        QspiWord::U8(0b00000000),
        QspiWord::None,
        &[],
    )
    .unwrap();

    wait_for_write_completition(qspi);
}

fn reset_read_registry(qspi: &mut Qspi<pac::QUADSPI>) {
    enable_write_operation(qspi);

    qspi.write_extended(
        QspiWord::U8(SET_READ_PARAMETERS_CMD),
        QspiWord::U8(0b11100000),
        QspiWord::None,
        &[],
    )
    .unwrap();

    wait_for_write_completition(qspi);
}

fn enable_write_operation(qspi: &mut Qspi<pac::QUADSPI>) {
    qspi.write_extended(
        QspiWord::U8(WRITE_ENABLE_CMD),
        QspiWord::None,
        QspiWord::None,
        &[],
    )
    .unwrap();
}

fn wait_for_write_completition(qspi: &mut Qspi<pac::QUADSPI>) {
    loop {
        let mut status: [u8; 1] = [0xFF; 1];
        qspi.read_extended(
            QspiWord::U8(READ_STATUS_REGISTRY_CMD),
            QspiWord::None,
            QspiWord::None,
            0,
            &mut status,
        )
        .unwrap();

        if status[0] & 0x01 == 0 {
            break;
        }
    }
}
