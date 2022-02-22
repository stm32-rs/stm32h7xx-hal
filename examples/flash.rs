//! Example of Flash erasing, writing and reading with the stm32h743zi (rm0433)

#![deny(warnings)]
#![no_main]
#![no_std]

use cortex_m_rt::entry;
#[macro_use]
mod utilities;
use log::info;
use stm32h7xx_hal::{pac, prelude::*, flash::Bank};

#[entry]
fn main() -> ! {
    info!("stm32h7xx-hal example - erase, write and read FLASH");

    let _cp = cortex_m::peripheral::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();

    // Skipping sector 0, erase all other sectors of user bank 1
    for i in 1..8 {
        flash.erase_sector(Bank::UserBank1, i).unwrap();
    }

    // Erase the entirety of user bank 2
    flash.erase_bank(Bank::UserBank2).unwrap();

    // Fill up write buffer
    let mut buff = [0u8; 1024 * 15];
    let mut count = 0;
    for i in 0..buff.len() {
        buff[i] = count;
        count += 1;
        if count >= 100 {
            count = 0;
        }
    }

    // Write data spanning over two banks
    flash
        .write_sector(Bank::UserBank1, 7, Bank::UserBank1.sector_size() - 256, &buff)
        .unwrap();

    // Read data spanning over two banks
    let mut read = [0u8; 1024 * 15];
    flash
        .read_sector(Bank::UserBank1, 7, Bank::UserBank1.sector_size() - 256, &mut read)
        .unwrap();
    for i in 0..read.len() {
        assert_eq!(read[i], buff[i]);
    }

    // Read bank 2 and ensure data is correct
    let mut read = [0u8; 1024 * 15 - 256];
    flash.read_sector(Bank::UserBank2, 0, 0, &mut read).unwrap();
    let out = &buff[256..];
    for i in 0..read.len() {
        assert_eq!(read[i], out[i]);
    }

    info!("Successfully erased, written and read back flash data");

    loop {}
}
