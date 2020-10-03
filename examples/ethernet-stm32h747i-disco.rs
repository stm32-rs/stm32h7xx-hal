//! Demo for STM32H747I-DISCO eval board
//!
//! The ethernet ring buffers are placed in SRAM3, where they can be
//! accessed by both the core and the Ethernet DMA.
//!
//! This demo doesn't use smoltcp - see the stm32h747i-disco-rtic demo
#![deny(warnings)]
#![no_main]
#![no_std]

use cortex_m;
use cortex_m_rt as rt;
use rt::{entry, exception};

#[macro_use]
mod utilities;

use log::info;

use stm32h7xx_hal::ethernet;
use stm32h7xx_hal::gpio::Speed::*;
use stm32h7xx_hal::hal::digital::v2::OutputPin;
use stm32h7xx_hal::{prelude::*, stm32, stm32::interrupt};

/// Locally administered MAC address
const MAC_ADDRESS: [u8; 6] = [0x02, 0x00, 0x11, 0x22, 0x33, 0x44];

/// Ethernet descriptor rings are a global singleton
#[link_section = ".sram3.eth"]
static mut DES_RING: ethernet::DesRing = ethernet::DesRing::new();

// the program entry point
#[entry]
fn main() -> ! {
    utilities::logger::init();
    let dp = stm32::Peripherals::take().unwrap();
    let mut cp = stm32::CorePeripherals::take().unwrap();

    // Initialise power...
    info!("Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.smps().freeze();

    // Link the SRAM3 power state to CPU1
    info!("Setup RCC...                  ");
    dp.RCC.ahb2enr.modify(|_, w| w.sram3en().set_bit());

    // Initialise clocks...
    let rcc = dp.RCC.constrain();
    let ccdr = rcc
        .sys_ck(200.mhz())
        .hclk(200.mhz())
        .freeze(pwrcfg, &dp.SYSCFG);

    // Initialise system...
    cp.SCB.invalidate_icache();
    cp.SCB.enable_icache();
    // TODO: ETH DMA coherence issues
    // cp.SCB.enable_dcache(&mut cp.CPUID);
    cp.DWT.enable_cycle_counter();

    // Initialise IO...
    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
    let gpiog = dp.GPIOG.split(ccdr.peripheral.GPIOG);
    let gpioi = dp.GPIOI.split(ccdr.peripheral.GPIOI);
    let mut link_led = gpioi.pi14.into_push_pull_output(); // LED3
    link_led.set_high().ok();

    let _rmii_ref_clk = gpioa.pa1.into_alternate_af11().set_speed(VeryHigh);
    let _rmii_mdio = gpioa.pa2.into_alternate_af11().set_speed(VeryHigh);
    let _rmii_mdc = gpioc.pc1.into_alternate_af11().set_speed(VeryHigh);
    let _rmii_crs_dv = gpioa.pa7.into_alternate_af11().set_speed(VeryHigh);
    let _rmii_rxd0 = gpioc.pc4.into_alternate_af11().set_speed(VeryHigh);
    let _rmii_rxd1 = gpioc.pc5.into_alternate_af11().set_speed(VeryHigh);
    let _rmii_tx_en = gpiog.pg11.into_alternate_af11().set_speed(VeryHigh);
    let _rmii_txd0 = gpiog.pg13.into_alternate_af11().set_speed(VeryHigh);
    let _rmii_txd1 = gpiog.pg12.into_alternate_af11().set_speed(VeryHigh);

    // Initialise ethernet...
    assert_eq!(ccdr.clocks.hclk().0, 200_000_000); // HCLK 200MHz
    assert_eq!(ccdr.clocks.pclk1().0, 100_000_000); // PCLK 100MHz
    assert_eq!(ccdr.clocks.pclk2().0, 100_000_000); // PCLK 100MHz
    assert_eq!(ccdr.clocks.pclk4().0, 100_000_000); // PCLK 100MHz

    let mac_addr = smoltcp::wire::EthernetAddress::from_bytes(&MAC_ADDRESS);
    let (_eth_dma, mut eth_mac) = unsafe {
        ethernet::new_unchecked(
            dp.ETHERNET_MAC,
            dp.ETHERNET_MTL,
            dp.ETHERNET_DMA,
            &mut DES_RING,
            mac_addr.clone(),
        )
    };
    unsafe {
        ethernet::enable_interrupt();
        cp.NVIC.set_priority(stm32::Interrupt::ETH, 196); // Mid prio
        cortex_m::peripheral::NVIC::unmask(stm32::Interrupt::ETH);
    }

    // ----------------------------------------------------------
    // Main application loop

    let mut eth_up = false;
    loop {
        // Ethernet
        let eth_last = eth_up;
        eth_up = eth_mac.phy_poll_link();
        match eth_up {
            true => link_led.set_low(),
            _ => link_led.set_high(),
        }
        .ok();

        if eth_up != eth_last {
            // Interface state change
            match eth_up {
                true => info!("Ethernet UP"),
                _ => info!("Ethernet DOWN"),
            }
        }
    }
}

#[interrupt]
fn ETH() {
    unsafe { ethernet::interrupt_handler() }
}

#[exception]
fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    panic!("HardFault at {:#?}", ef);
}

#[exception]
fn DefaultHandler(irqn: i16) {
    panic!("Unhandled exception (IRQn = {})", irqn);
}
