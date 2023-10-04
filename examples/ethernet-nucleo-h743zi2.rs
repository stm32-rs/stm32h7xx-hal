//! Ethernet demo for NUCLEO-H743ZI2 development board
//!
//! NUCLEO-H743ZI2: RMII TXD1 is on PB13
//! NUCLEO-H745I-Q: RMII TXD1 is on PB13
//!
//! The ethernet ring buffers are placed in SRAM3, where they can be
//! accessed by both the core and the Ethernet DMA.
//!
//! This demo does not use smoltcp - see the ethernet-rtic-stm32h747i-disco demo
//! for an example of smoltcp
#![deny(warnings)]
#![no_main]
#![no_std]

extern crate cortex_m_rt as rt;
use core::sync::atomic::{AtomicU32, Ordering};
use rt::{entry, exception};

extern crate cortex_m;

#[macro_use]
mod utilities;
use log::info;

use stm32h7xx_hal::rcc::CoreClocks;
use stm32h7xx_hal::{
    ethernet,
    ethernet::{
        RxDescriptor, RxDescriptorRing, TxDescriptor, TxDescriptorRing, MTU,
        PHY,
    },
};
use stm32h7xx_hal::{prelude::*, stm32, stm32::interrupt};

/// Configure SYSTICK for 1ms timebase
fn systick_init(syst: &mut stm32::SYST, clocks: CoreClocks) {
    let c_ck_mhz = clocks.c_ck().to_MHz();

    let syst_calib = 0x3E8;

    syst.set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
    syst.set_reload((syst_calib * c_ck_mhz) - 1);
    syst.enable_interrupt();
    syst.enable_counter();
}

/// ======================================================================
/// Entry point
/// ======================================================================

/// TIME is an atomic u32 that counts milliseconds. Although not used
/// here, it is very useful to have for network protocols
static TIME: AtomicU32 = AtomicU32::new(0);

/// Locally administered MAC address
const MAC_ADDRESS: [u8; 6] = [0x02, 0x00, 0x11, 0x22, 0x33, 0x44];

/// DesRing TD
const NUM_DESCRIPTORS: usize = 8;
#[link_section = ".sram3.eth"]
/// Doc
static mut TX_DESCRIPTORS: [TxDescriptor; NUM_DESCRIPTORS] =
    [TxDescriptor::new(); NUM_DESCRIPTORS];
#[link_section = ".sram3.eth"]
/// Doc
static mut TX_BUFFERS: [[u8; MTU + 2]; NUM_DESCRIPTORS] =
    [[0u8; MTU + 2]; NUM_DESCRIPTORS];
#[link_section = ".sram3.eth"]
/// Doc
static mut RX_DESCRIPTORS: [RxDescriptor; NUM_DESCRIPTORS] =
    [RxDescriptor::new(); NUM_DESCRIPTORS];
#[link_section = ".sram3.eth"]
/// Doc
static mut RX_BUFFERS: [[u8; MTU + 2]; NUM_DESCRIPTORS] =
    [[0u8; MTU + 2]; NUM_DESCRIPTORS];

// the program entry point
#[entry]
fn main() -> ! {
    utilities::logger::init();
    let dp = stm32::Peripherals::take().unwrap();
    let mut cp = stm32::CorePeripherals::take().unwrap();

    // Initialise power...
    info!("Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    let pwrcfg = example_power!(pwr).freeze();

    // Initialise SRAM3
    info!("Setup RCC...                  ");
    dp.RCC.ahb2enr.modify(|_, w| w.sram3en().set_bit());

    // Initialise clocks...
    let rcc = dp.RCC.constrain();
    let ccdr = rcc
        .sys_ck(200.MHz())
        .hclk(200.MHz())
        .pll1_r_ck(100.MHz()) // for TRACECK
        .freeze(pwrcfg, &dp.SYSCFG);

    // Get the delay provider.
    let delay = cp.SYST.delay(ccdr.clocks);

    // Initialise system...
    cp.SCB.enable_icache();
    // TODO: ETH DMA coherence issues
    // cp.SCB.enable_dcache(&mut cp.CPUID);
    cp.DWT.enable_cycle_counter();

    // Initialise IO...
    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
    let gpiog = dp.GPIOG.split(ccdr.peripheral.GPIOG);
    let mut link_led = gpiob.pb0.into_push_pull_output(); // LED1, green
    link_led.set_high();

    let rmii_ref_clk = gpioa.pa1.into_alternate();
    let rmii_mdio = gpioa.pa2.into_alternate();
    let rmii_mdc = gpioc.pc1.into_alternate();
    let rmii_crs_dv = gpioa.pa7.into_alternate();
    let rmii_rxd0 = gpioc.pc4.into_alternate();
    let rmii_rxd1 = gpioc.pc5.into_alternate();
    let rmii_tx_en = gpiog.pg11.into_alternate();
    let rmii_txd0 = gpiog.pg13.into_alternate();
    let rmii_txd1 = gpiob.pb13.into_alternate();

    let rmii_pins = (
        rmii_ref_clk,
        rmii_mdio,
        rmii_mdc,
        rmii_crs_dv,
        rmii_rxd0,
        rmii_rxd1,
        rmii_tx_en,
        rmii_txd0,
        rmii_txd1,
    );
    // Initialise ethernet...
    assert_eq!(ccdr.clocks.hclk().raw(), 200_000_000); // HCLK 200MHz
    assert_eq!(ccdr.clocks.pclk1().raw(), 100_000_000); // PCLK 100MHz
    assert_eq!(ccdr.clocks.pclk2().raw(), 100_000_000); // PCLK 100MHz
    assert_eq!(ccdr.clocks.pclk4().raw(), 100_000_000); // PCLK 100MHz

    let mac_addr = smoltcp::wire::EthernetAddress::from_bytes(&MAC_ADDRESS);
    let (rx_ring, tx_ring) = {
        // let tx_desc = unsafe { TX_DESCRIPTORS.write([TxDescriptor::new(); NUM_DESCRIPTORS]) };
        // let tx_buf = unsafe { TX_BUFFERS.write([[0u8; MTU + 2]; NUM_DESCRIPTORS]) };

        // let rx_desc = unsafe { RX_DESCRIPTORS.write([RxDescriptor::new(); NUM_DESCRIPTORS]) };
        // let rx_buf = unsafe { RX_BUFFERS.write([[0u8; MTU + 2]; NUM_DESCRIPTORS]) };

        (
            RxDescriptorRing::new(unsafe { &mut RX_DESCRIPTORS }, unsafe {
                &mut RX_BUFFERS
            }),
            TxDescriptorRing::new(unsafe { &mut TX_DESCRIPTORS }, unsafe {
                &mut TX_BUFFERS
            }),
        )
    };

    #[cfg(feature = "ptp")]
    let ethernet::Parts {
        dma: eth_dma,
        mac: eth_mac,
        ptp: _ptp,
    } = ethernet::new(
        dp.ETHERNET_MAC,
        dp.ETHERNET_MTL,
        dp.ETHERNET_DMA,
        rmii_pins,
        rx_ring,
        tx_ring,
        mac_addr,
        ccdr.peripheral.ETH1MAC,
        &ccdr.clocks,
    );

    #[cfg(not(feature = "ptp"))]
    let ethernet::Parts {
        dma: eth_dma,
        mac: eth_mac,
    } = ethernet::new(
        dp.ETHERNET_MAC,
        dp.ETHERNET_MTL,
        dp.ETHERNET_DMA,
        rmii_pins,
        rx_ring,
        tx_ring,
        mac_addr,
        ccdr.peripheral.ETH1MAC,
        &ccdr.clocks,
    );
    // let start_addend = ptp.addend();
    eth_dma.enable_interrupt();

    // Initialise ethernet PHY...
    let mut lan8742a = ethernet::phy::LAN8742A::new(eth_mac.set_phy_addr(0));
    lan8742a.phy_reset();
    lan8742a.phy_init();

    unsafe {
        cp.NVIC.set_priority(stm32::Interrupt::ETH, 196); // Mid prio
        cortex_m::peripheral::NVIC::unmask(stm32::Interrupt::ETH);
    }

    // ----------------------------------------------------------
    // Begin periodic tasks

    systick_init(&mut delay.free(), ccdr.clocks);
    unsafe {
        cp.SCB.shpr[15 - 4].write(128);
    } // systick exception priority

    // ----------------------------------------------------------
    // Main application loop

    let mut eth_up = false;
    loop {
        let _time = TIME.load(Ordering::Relaxed);

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
    ethernet::eth_interrupt_handler();
}

#[exception]
fn SysTick() {
    TIME.fetch_add(1, Ordering::Relaxed);
}

#[exception]
unsafe fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    panic!("HardFault at {:#?}", ef);
}

#[exception]
unsafe fn DefaultHandler(irqn: i16) {
    panic!("Unhandled exception (IRQn = {})", irqn);
}
