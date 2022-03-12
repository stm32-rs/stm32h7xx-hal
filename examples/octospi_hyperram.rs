//! Example using a OCTOSPI HyperRAM in memory-mapped mode
//!
//! Tested on a STM32H735G-DK with a Cypress S70KL1281DABHI023

#![deny(warnings)]
#![no_main]
#![no_std]

use core::mem;
use core::slice;

#[macro_use]
mod utilities;

use cortex_m_rt::entry;
use stm32h7xx_hal::rcc::rec::{OctospiClkSel, OctospiClkSelGetter};
use stm32h7xx_hal::{gpio::Speed::High, pac, prelude::*, xspi::HyperbusConfig};

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
    let ccdr = rcc.sys_ck(320.mhz()).freeze(pwrcfg, &dp.SYSCFG);

    // Octospi from HCLK at 160MHz
    assert_eq!(ccdr.clocks.hclk().0, 160_000_000);
    assert_eq!(
        ccdr.peripheral.OCTOSPI1.get_kernel_clk_mux(),
        OctospiClkSel::RCC_HCLK3
    );

    // Acquire the GPIO peripherals. This also enables the clock for
    // the GPIOs in the RCC register.
    let gpiog = dp.GPIOG.split(ccdr.peripheral.GPIOG);
    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpiof = dp.GPIOF.split(ccdr.peripheral.GPIOF);

    let _tracweswo = gpiob.pb3.into_alternate::<0>();

    let _ncs = gpiog
        .pg12
        .into_alternate::<3>()
        .set_speed(High)
        .internal_pull_up(true);
    let _dqs = gpiof
        .pf12
        .into_alternate::<9>()
        .set_speed(High)
        .internal_pull_up(true);
    let _clk = gpiof
        .pf4
        .into_alternate::<9>()
        .set_speed(High)
        .internal_pull_up(true);
    let _io0 = gpiof
        .pf0
        .into_alternate::<9>()
        .set_speed(High)
        .internal_pull_up(true);
    let _io1 = gpiof
        .pf1
        .into_alternate::<9>()
        .set_speed(High)
        .internal_pull_up(true);
    let _io2 = gpiof
        .pf2
        .into_alternate::<9>()
        .set_speed(High)
        .internal_pull_up(true);
    let _io3 = gpiof
        .pf3
        .into_alternate::<9>()
        .set_speed(High)
        .internal_pull_up(true);
    let _io4 = gpiog
        .pg0
        .into_alternate::<9>()
        .set_speed(High)
        .internal_pull_up(true);
    let _io5 = gpiog
        .pg1
        .into_alternate::<9>()
        .set_speed(High)
        .internal_pull_up(true);
    let _io6 = gpiog
        .pg10
        .into_alternate::<3>()
        .set_speed(High)
        .internal_pull_up(true);
    let _io7 = gpiog
        .pg11
        .into_alternate::<9>()
        .set_speed(High)
        .internal_pull_up(true);

    info!("");
    info!("stm32h7xx-hal example - OCTOSPI HyperRAM");
    info!("");

    // Initialise a HyperRAM on the OCTOSPI2 peripheral
    let ram_slice = unsafe {
        let hyperram_size = 16 * 1024 * 1024; // 16 MByte
        let config = HyperbusConfig::new(80.mhz())
            .device_size_bytes(24) // 16 Mbyte
            .refresh_interval(4.us())
            .read_write_recovery(4) // 50ns
            .access_initial_latency(6);

        let hyperram = dp.OCTOSPI2.octospi_hyperbus_unchecked(
            config,
            &ccdr.clocks,
            ccdr.peripheral.OCTOSPI2,
        );

        info!("Created HyperRAM..");
        info!("{}", hyperram);
        info!("");

        // Initialise and convert raw pointer to slice
        let ram_ptr: *mut u32 = hyperram.init();
        slice::from_raw_parts_mut(
            ram_ptr,
            hyperram_size / mem::size_of::<u32>(),
        )
    };

    info!("Writing checkerboard pattern...");
    for x in ram_slice.iter_mut() {
        *x = 0xAA55AA55;
    }
    info!("Reading checkerboard pattern...");
    for (i, x) in ram_slice.iter().enumerate() {
        assert_eq!(
            *x,
            0xAA55AA55,
            "Mismatch at address 0x{:x} (0x{:x} != 0xaa55aa55)",
            i * 4,
            *x
        );
    }

    info!("Writing reverse checkerboard pattern...");
    for x in ram_slice.iter_mut() {
        *x = 0x55AA55AA;
    }
    info!("Reading reverse checkerboard pattern...");
    for (i, x) in ram_slice.iter().enumerate() {
        assert_eq!(
            *x,
            0x55AA55AA,
            "Mismatch at address 0x{:x} (0x{:x} != 0x55aa55aa)",
            i * 4,
            *x
        );
    }

    info!("Success!");
    info!("");

    loop {
        cortex_m::asm::nop();
    }
}
