//! Demo for STM32H747I-DISCO eval board
//!
//! STM32H747I-DISCO: RMII TXD1 is on PG12
//!
//! The ethernet ring buffers are placed in SRAM3, where they can be
//! accessed by both the core and the Ethernet DMA.
//!
//! This demo does not use smoltcp - see the ethernet-rtic-stm32h747i-disco demo
//! for an example of smoltcp
#![deny(warnings)]
#![no_main]
#![no_std]

use core::mem::MaybeUninit;
use cortex_m_rt as rt;
use rt::{entry, exception};

// TODO: use core::cell::SyncUnsafeCell when stabilized rust-lang/rust#95439
use utilities::sync_unsafe_cell::SyncUnsafeCell;

#[macro_use]
#[allow(unused)]
mod utilities;

use log::info;

use stm32h7xx_hal::{ethernet, ethernet::PHY};
use stm32h7xx_hal::{prelude::*, stm32, stm32::interrupt};

/// Locally administered MAC address
const MAC_ADDRESS: [u8; 6] = [0x02, 0x00, 0x11, 0x22, 0x33, 0x44];

/// Ethernet descriptor rings are a global singleton
#[link_section = ".sram3.eth"]
static DES_RING: MaybeUninit<SyncUnsafeCell<ethernet::DesRing<4, 4>>> =
    MaybeUninit::uninit();

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
        .sys_ck(200.MHz())
        .hclk(200.MHz())
        .freeze(pwrcfg, &dp.SYSCFG);

    // Initialise system...
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
    link_led.set_high();

    let rmii_ref_clk = gpioa.pa1.into_alternate();
    let rmii_mdio = gpioa.pa2.into_alternate();
    let rmii_mdc = gpioc.pc1.into_alternate();
    let rmii_crs_dv = gpioa.pa7.into_alternate();
    let rmii_rxd0 = gpioc.pc4.into_alternate();
    let rmii_rxd1 = gpioc.pc5.into_alternate();
    let rmii_tx_en = gpiog.pg11.into_alternate();
    let rmii_txd0 = gpiog.pg13.into_alternate();
    let rmii_txd1 = gpiog.pg12.into_alternate();

    // Initialise ethernet...
    assert_eq!(ccdr.clocks.hclk().raw(), 200_000_000); // HCLK 200MHz
    assert_eq!(ccdr.clocks.pclk1().raw(), 100_000_000); // PCLK 100MHz
    assert_eq!(ccdr.clocks.pclk2().raw(), 100_000_000); // PCLK 100MHz
    assert_eq!(ccdr.clocks.pclk4().raw(), 100_000_000); // PCLK 100MHz

    let mac_addr = smoltcp::wire::EthernetAddress::from_bytes(&MAC_ADDRESS);
    let (_eth_dma, eth_mac) = {
        // DES_RING is located in .axisram.eth, which is not initialized by
        // the runtime. We must manually initialize it to a valid value,
        // without taking a reference to the uninitialized value
        unsafe {
            SyncUnsafeCell::raw_get(DES_RING.as_ptr())
                .write(ethernet::DesRing::new());
        }
        // Now we can take a mutable reference to DES_RING. To avoid
        // aliasing, this reference must only be taken once
        let des_ring =
            unsafe { &mut *SyncUnsafeCell::raw_get(DES_RING.as_ptr()) };

        ethernet::new(
            dp.ETHERNET_MAC,
            dp.ETHERNET_MTL,
            dp.ETHERNET_DMA,
            (
                rmii_ref_clk,
                rmii_mdio,
                rmii_mdc,
                rmii_crs_dv,
                rmii_rxd0,
                rmii_rxd1,
                rmii_tx_en,
                rmii_txd0,
                rmii_txd1,
            ),
            des_ring,
            mac_addr,
            ccdr.peripheral.ETH1MAC,
            &ccdr.clocks,
        )
    };

    // Initialise ethernet PHY...
    let mut lan8742a = ethernet::phy::LAN8742A::new(eth_mac.set_phy_addr(0));
    lan8742a.phy_reset();
    lan8742a.phy_init();

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
        eth_up = lan8742a.poll_link();
        match eth_up {
            true => link_led.set_low(),
            _ => link_led.set_high(),
        }

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
unsafe fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    panic!("HardFault at {:#?}", ef);
}

#[exception]
unsafe fn DefaultHandler(irqn: i16) {
    panic!("Unhandled exception (IRQn = {})", irqn);
}
