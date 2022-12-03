//! Demonstration of accessing a NAND Flash via the FMC peripheral
//!
//! Tested using a NUCLEO-H7A3ZI-Q development board with a SkyHigh S34ML08G3
//! SLC NAND on a custom breakout board
#![deny(warnings)]
#![no_main]
#![no_std]

#[macro_use]
#[allow(dead_code)]
mod utilities;

use log::info;

extern crate cortex_m;

use cortex_m_rt::entry;
use stm32h7xx_hal::gpio::Speed;
use stm32h7xx_hal::{pac, prelude::*, rcc::rec};

use stm32_fmc::devices::s34ml08g3_4kb;
use stm32_fmc::nand_device::Status;

/// Configre a pin for the FMC controller
macro_rules! fmc_pins {
    ($($pin:expr),*) => {
        (
            $(
                $pin.into_push_pull_output()
                    .speed(Speed::VeryHigh)
                    .into_alternate::<12>()
                    .internal_pull_up(true)
            ),*
        )
    };
}

#[entry]
fn main() -> ! {
    utilities::logger::init();
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    // Initialise power...
    let pwr = dp.PWR.constrain();
    let pwrcfg = example_power!(pwr).freeze();

    // Initialise clocks...
    let rcc = dp.RCC.constrain();
    let ccdr = rcc
        .sys_ck(200.MHz())
        .hclk(200.MHz()) // FMC clock from HCLK by default
        .pll2_p_ck(100.MHz())
        .pll2_r_ck(100.MHz())
        .freeze(pwrcfg, &dp.SYSCFG);

    // Get the delay provider.
    let mut delay = cp.SYST.delay(ccdr.clocks);

    // Initialise system...
    cp.SCB.enable_icache();
    cp.DWT.enable_cycle_counter();

    // Initialise IO...
    let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);
    let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);
    let gpiog = dp.GPIOG.split(ccdr.peripheral.GPIOG);

    // ----------------------------------------------------------
    // NAND Flash
    // Initialise NAND Flash...
    let nand_flash_pins = fmc_pins! {
        // A17/ALE; A16/CLE
        gpiod.pd12, gpiod.pd11,
        // D0-D7
        gpiod.pd14,
        gpiod.pd15,
        gpiod.pd0,
        gpiod.pd1,
        gpioe.pe7,
        gpioe.pe8,
        gpioe.pe9,
        gpioe.pe10,
        //
        gpiog.pg9,              // NCE
        gpiod.pd4,              // NOE
        gpiod.pd5,              // NWE
        gpiod.pd6               // NWAIT
    };
    let _wp = gpioe.pe2.into_push_pull_output().set_high(); // unprotect

    let mut fmc_nand = dp.FMC.nand(
        nand_flash_pins,
        s34ml08g3_4kb::S34ml08g3 {},
        ccdr.peripheral.FMC.kernel_clk_mux(rec::FmcClkSel::Pll2R),
        &ccdr.clocks,
    );

    // Initialise controller and get access to device
    let mut nand_device = fmc_nand.init(&mut delay);
    let id = nand_device.read_id();
    let params = nand_device.read_parameter_page();
    info!("Initialised NAND Flash");
    info!("Identifier {:?}", id);
    info!("{:?}", params);

    // Get a unqiue identifier, if the device supports it
    let unique = nand_device.read_unique_id();
    info!("Unique ID 0x{:032x}", unique);
    info!("");

    // Read first page
    let mut page = [0u8; 256];
    nand_device.page_read(0x0, false, &mut page);
    info!("First page: {:02X?}", &page);

    // Erase block
    nand_device.block_erase(0x0);
    info!("Erase complete");

    // Program the first page
    for (n, x) in page.iter_mut().enumerate() {
        *x = n as u8;
    }
    match nand_device.page_program(0x0, false, &page) {
        Status::Success(x) => info!("Write complete with 0x{:02x}", x),
        Status::Fail(x) => info!("Write failed with 0x{:02x}", x),
    }

    // Read first page again
    nand_device.page_read(0x0, false, &mut page);
    info!("First page: {:02X?}", page);
    for (n, x) in page.iter_mut().enumerate() {
        assert_eq!(*x, n as u8);
    }

    // Erase block
    nand_device.block_erase(0x0);
    info!("Erase complete");

    loop {
        cortex_m::asm::nop()
    }
}
