//! Example of using the CRC peripheral with various configurations
//!
//! A good resource of CRC algorithms, to assist in determining their exact settings,
//! is https://reveng.sourceforge.io/crc-catalogue/

#![no_main]
#![no_std]

use cortex_m_rt::entry;
use log::info;

use stm32h7xx_hal::crc::{Config, Crc, Polynomial};
use stm32h7xx_hal::{pac, prelude::*};

#[macro_use]
mod utilities;

/// Standard input data for testing CRCs
const CHECK_DATA: &[u8] = b"123456789";

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

    let ccdr = rcc.sys_ck(200.mhz()).freeze(pwrcfg, &dp.SYSCFG);

    info!("");
    info!("stm32h7xx-hal example - CRC");
    info!("");

    let mut crc = Crc::enable(dp.CRC, ccdr.peripheral.CRC);
    // feed standard check data and ensure result is expected
    // the default config is CRC32 MPEG2
    assert_eq!(crc.update_and_read(CHECK_DATA), 0x0376_E6E7, "MPEG2 failed");
    info!("Default configuration check passed");

    let bzip_crc = Config::new().output_xor(0xFFFF_FFFF);
    crc.set_config(&bzip_crc);

    assert_eq!(crc.update_and_read(CHECK_DATA), 0xFC89_1918, "BZIP2 failed");
    info!("BZIP2 check passed");

    let standard_crc32 = Config::new().reflect(true).output_xor(0xFFFF_FFFF);
    crc.set_config(&standard_crc32);

    assert_eq!(crc.update_and_read(CHECK_DATA), 0xCBF4_3926, "CRC32 failed");
    info!("Standard CRC32 check passed");

    // first part of an Si7020's serial number: pattern is data,crc,...
    let first_serial = [0x06, 0xA6, 0x97, 0xF4, 0xB7, 0x6E, 0x00, 0xA4];
    let si70xx_crc = Config::new()
        .polynomial(Polynomial::bits8(0x31).unwrap())
        .initial_value(0);
    crc.set_config(&si70xx_crc);

    for chunk in first_serial.chunks_exact(2) {
        assert_eq!(crc.update_and_read(&chunk[..1]) as u8, chunk[1]);
    }
    info!("Si7020 serial number CRC8 check passed");

    info!("Done.");

    loop {
        cortex_m::asm::nop()
    }
}
