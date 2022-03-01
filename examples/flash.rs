//! Example of Flash erasing, writing and reading with the stm32h743zi (rm0433)
//! Assumes this example runs at sector 0 of bank 1 (0x0800_0000)

#![deny(warnings)]
#![no_main]
#![no_std]

use cortex_m_rt::entry;
#[macro_use]
mod utilities;
use log::info;
use stm32h7xx_hal::{flash::Bank, pac, prelude::*};

#[entry]
fn main() -> ! {
    utilities::logger::init();
    info!("stm32h7xx-hal example - erase, write and read FLASH");

    let _cp = cortex_m::peripheral::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();
    // Constrain and Freeze power
    info!("Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    let pwrcfg = example_power!(pwr).freeze();
    
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

    // Write data on bank 1 and read it back
    flash.write_sector(Bank::UserBank1, 1, 0, &buff).unwrap();
    let mut read = [0u8; 1024 * 15];
    flash.read_sector(Bank::UserBank1, 1, 0, &mut read).unwrap();
    for i in 0..read.len() {
        assert_eq!(read[i], buff[i]);
    }

    // Write data on bank 2 and read it back
    flash.write_sector(Bank::UserBank2, 0, 256, &buff).unwrap();
    let mut read = [0u8; 1024 * 15];
    flash
        .read_sector(Bank::UserBank2, 0, 256, &mut read)
        .unwrap();
    for i in 0..read.len() {
        assert_eq!(read[i], buff[i]);
    }

    info!("Successfully erased, written and read back flash data");

    loop {
        cortex_m::asm::nop()
    }
}
