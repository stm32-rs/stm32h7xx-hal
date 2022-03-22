//! OCTOSPI peripheral example in indirect mode
//!
//! Tested on a STM32H735G-DK with a Macronix MX25LM51245GXDI00

#![deny(warnings)]
#![no_main]
#![no_std]

#[macro_use]
mod utilities;

use cortex_m_rt::entry;
use stm32h7xx_hal::rcc::rec::{OctospiClkSel, OctospiClkSelGetter};
use stm32h7xx_hal::{
    pac, prelude::*, xspi::OctospiMode, xspi::OctospiWord as XW,
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
    let ccdr = rcc.sys_ck(96.mhz()).freeze(pwrcfg, &dp.SYSCFG);

    // Octospi from HCLK at 48MHz
    assert_eq!(ccdr.clocks.hclk().0, 48_000_000);
    assert_eq!(
        ccdr.peripheral.OCTOSPI1.get_kernel_clk_mux(),
        OctospiClkSel::RCC_HCLK3
    );

    // Acquire the GPIO peripherals. This also enables the clock for
    // the GPIOs in the RCC register.
    let gpiog = dp.GPIOG.split(ccdr.peripheral.GPIOG);
    let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);
    let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);
    let gpiof = dp.GPIOF.split(ccdr.peripheral.GPIOF);

    let _ncs = gpiog.pg6.into_alternate::<10>();
    let _clk = gpiof.pf10.into_alternate::<9>();
    let _io0 = gpiod.pd11.into_alternate::<9>();
    let _io1 = gpiod.pd12.into_alternate::<9>();
    let _io2 = gpioe.pe2.into_alternate::<9>();
    let _io3 = gpiod.pd13.into_alternate::<9>();
    let _io4 = gpiod.pd4.into_alternate::<10>();
    let _io5 = gpiod.pd5.into_alternate::<10>();
    let _io6 = gpiog.pg9.into_alternate::<9>();
    let _io7 = gpiod.pd7.into_alternate::<10>();

    info!("");
    info!("stm32h7xx-hal example - OCTOSPI");
    info!("");

    // Initialise the OCTOSPI peripheral.
    let mut octospi = dp.OCTOSPI1.octospi_unchecked(
        12.mhz(),
        &ccdr.clocks,
        ccdr.peripheral.OCTOSPI1,
    );

    octospi.configure_mode(OctospiMode::OneBit).unwrap();

    // RDID Read Identification. Abuses address as instruction phase, but that
    // works in SPI mode.
    let mut read: [u8; 3] = [0; 3];
    octospi.read(0x9F, &mut read).unwrap();
    info!("Read with instruction 0x9F : {:x?}", read);

    // Switch Macronix MX25LM51245GXDI00 to SDR OPI
    // Set WREN bit
    octospi
        .write_extended(XW::U8(0x06), XW::None, XW::None, &[])
        .unwrap();
    // Write Configuration Register 2
    octospi
        .write_extended(XW::U8(0x72), XW::U32(0), XW::None, &[1])
        .unwrap();
    // Change bus mode
    octospi.configure_mode(OctospiMode::EightBit).unwrap();

    // RDID Read Identification
    let mut read: [u8; 3] = [0; 3];
    octospi
        .read_extended(XW::U16(0x9F60), XW::U32(0), XW::None, 4, &mut read)
        .unwrap();

    info!("Read with instruction 0x9F60 : {:x?}", read);

    loop {}
}
