//! Example of Flash erasing, writing and reading
//!
//! Tested on the following parts:
//! - STM32H7A3ZIT6Q (NUCLEO-H7A3ZI-Q)
//! - STM32H735IGK (STM32H735G-DK)
//! - STM32H747XIH (STM32H747I-DISCO)
#![deny(warnings)]
#![no_main]
#![no_std]

use cortex_m_rt::entry;
use stm32h7xx_hal::{pac, prelude::*};

// traits for read, write and erase methods
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};

#[macro_use]
mod utilities;
use log::info;

#[entry]
fn main() -> ! {
    utilities::logger::init();
    let dp = pac::Peripherals::take().unwrap();

    // Constrain and Freeze power
    info!("Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    let pwrcfg = example_power!(pwr).vos3().freeze(); // TODO

    // Constrain and Freeze clock
    info!("Setup RCC...                  ");
    let rcc = dp.RCC.constrain();
    let _ccdr = rcc.sys_ck(50.MHz()).freeze(pwrcfg, &dp.SYSCFG);

    info!("");
    info!("stm32h7xx-hal example - Flash erase, write and read");
    info!("");

    // Initialise both flash banks
    let (mut flash1, mut flash2) = dp.FLASH.split();

    // Erase 16kB at the end of user bank 1
    let len = flash1.len() as u32;
    let start = len - 16 * 1024;
    {
        let mut f = flash1.unlocked();
        f.erase(start, len).unwrap();
    }

    // Erase the entirety of user bank 2, if present
    if let Some(ref mut f2) = flash2 {
        let mut f = f2.unlocked();
        f.erase(0, 1024 * 1024).unwrap();
    }

    // Fill up write buffer
    let mut buf = [0u8; 1024 * 16];
    let mut count = 0;
    for b in buf.iter_mut() {
        *b = count;
        count += 1;
        if count >= 100 {
            count = 0;
        }
    }

    // Write data on user bank 1 and read it back
    {
        let mut f = flash1.unlocked();
        f.write(start, &buf).unwrap();
    }
    let mut read = [0u8; 1024 * 16];
    flash1.read(start, &mut read).unwrap();
    assert_eq!(read, buf);

    // Write data on user bank 2 offset 256 and read it back, if user bank 2 present
    if let Some(ref mut f2) = flash2 {
        let mut f = f2.unlocked();
        f.write(256, &buf).unwrap();
        f.read(256, &mut read).unwrap();
        assert_eq!(read, buf);
    }

    info!("Successfully erased, written and read back flash data");

    loop {
        cortex_m::asm::nop()
    }
}
