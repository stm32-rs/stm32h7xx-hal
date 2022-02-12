//! Demo for STM32H747I-DISCO eval board using the Real Time for the Masses
//! (RTIC) framework.
//!
//! This demo responds to pings on 192.168.1.99 (IP address hardcoded below)
//!
//! We use the SysTick timer to create a 1ms timebase for use with smoltcp.
//!
//! The ethernet ring buffers are placed in SRAM3, where they can be
//! accessed by both the core and the Ethernet DMA.
#![deny(warnings)]
#![no_main]
#![no_std]

#[macro_use]
#[allow(unused)]
mod utilities;
use log::info;

use core::sync::atomic::AtomicU32;

use smoltcp::iface::{
    Interface, InterfaceBuilder, Neighbor, NeighborCache, Route, Routes,
    SocketStorage,
};
use smoltcp::time::Instant;
use smoltcp::wire::{HardwareAddress, IpAddress, IpCidr, Ipv6Cidr};

use stm32h7xx_hal::{ethernet, rcc::CoreClocks, stm32};

/// Configure SYSTICK for 1ms timebase
fn systick_init(mut syst: stm32::SYST, clocks: CoreClocks) {
    let c_ck_mhz = clocks.c_ck().0 / 1_000_000;

    let syst_calib = 0x3E8;

    syst.set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
    syst.set_reload((syst_calib * c_ck_mhz) - 1);
    syst.enable_interrupt();
    syst.enable_counter();
}

/// TIME is an atomic u32 that counts milliseconds.
static TIME: AtomicU32 = AtomicU32::new(0);

/// Locally administered MAC address
const MAC_ADDRESS: [u8; 6] = [0x02, 0x00, 0x11, 0x22, 0x33, 0x44];

/// Ethernet descriptor rings are a global singleton
#[link_section = ".sram3.eth"]
static mut DES_RING: ethernet::DesRing<4, 4> = ethernet::DesRing::new();

/// Net storage with static initialisation - another global singleton
pub struct NetStorageStatic<'a> {
    ip_addrs: [IpCidr; 1],
    socket_storage: [SocketStorage<'a>; 8],
    neighbor_cache_storage: [Option<(IpAddress, Neighbor)>; 8],
    routes_storage: [Option<(IpCidr, Route)>; 1],
}
static mut STORE: NetStorageStatic = NetStorageStatic {
    // Garbage
    ip_addrs: [IpCidr::Ipv6(Ipv6Cidr::SOLICITED_NODE_PREFIX)],
    socket_storage: [SocketStorage::EMPTY; 8],
    neighbor_cache_storage: [None; 8],
    routes_storage: [None; 1],
};

pub struct Net<'a> {
    iface: Interface<'a, ethernet::EthernetDMA<'a, 4, 4>>,
}
impl<'a> Net<'a> {
    pub fn new(
        store: &'static mut NetStorageStatic<'a>,
        ethdev: ethernet::EthernetDMA<'a, 4, 4>,
        ethernet_addr: HardwareAddress,
    ) -> Self {
        // Set IP address
        store.ip_addrs =
            [IpCidr::new(IpAddress::v4(192, 168, 1, 99).into(), 0)];

        let neighbor_cache =
            NeighborCache::new(&mut store.neighbor_cache_storage[..]);
        let routes = Routes::new(&mut store.routes_storage[..]);

        let iface =
            InterfaceBuilder::new(ethdev, &mut store.socket_storage[..])
                .hardware_addr(ethernet_addr)
                .neighbor_cache(neighbor_cache)
                .ip_addrs(&mut store.ip_addrs[..])
                .routes(routes)
                .finalize();

        return Net { iface };
    }

    /// Polls on the ethernet interface. You should refer to the smoltcp
    /// documentation for poll() to understand how to call poll efficiently
    pub fn poll(&mut self, now: i64) {
        let timestamp = Instant::from_millis(now);

        self.iface
            .poll(timestamp)
            .map(|_| ())
            .unwrap_or_else(|e| info!("Poll: {:?}", e));
    }
}

#[rtic::app(device = stm32h7xx_hal::stm32, peripherals = true)]
mod app {
    use stm32h7xx_hal::hal::digital::v2::OutputPin;
    use stm32h7xx_hal::{ethernet, ethernet::PHY, gpio, prelude::*};

    use super::*;
    use core::sync::atomic::Ordering;

    #[shared]
    struct SharedResources {}
    #[local]
    struct LocalResources {
        net: Net<'static>,
        lan8742a: ethernet::phy::LAN8742A<ethernet::EthernetMAC>,
        link_led: gpio::gpioi::PI14<gpio::Output<gpio::PushPull>>,
    }

    #[init]
    fn init(
        mut ctx: init::Context,
    ) -> (SharedResources, LocalResources, init::Monotonics) {
        utilities::logger::init();
        // Initialise power...
        let pwr = ctx.device.PWR.constrain();
        let pwrcfg = pwr.smps().freeze();

        // Link the SRAM3 power state to CPU1
        ctx.device.RCC.ahb2enr.modify(|_, w| w.sram3en().set_bit());

        // Initialise clocks...
        let rcc = ctx.device.RCC.constrain();
        let ccdr = rcc
            .sys_ck(200.mhz())
            .hclk(200.mhz())
            .freeze(pwrcfg, &ctx.device.SYSCFG);

        // Initialise system...
        ctx.core.SCB.enable_icache();
        // TODO: ETH DMA coherence issues
        // ctx.core.SCB.enable_dcache(&mut ctx.core.CPUID);
        ctx.core.DWT.enable_cycle_counter();

        // Initialise IO...
        let gpioa = ctx.device.GPIOA.split(ccdr.peripheral.GPIOA);
        let gpioc = ctx.device.GPIOC.split(ccdr.peripheral.GPIOC);
        let gpiog = ctx.device.GPIOG.split(ccdr.peripheral.GPIOG);
        let gpioi = ctx.device.GPIOI.split(ccdr.peripheral.GPIOI);
        let mut link_led = gpioi.pi14.into_push_pull_output(); // LED3
        link_led.set_high().ok();

        let rmii_ref_clk = gpioa.pa1.into_alternate_af11();
        let rmii_mdio = gpioa.pa2.into_alternate_af11();
        let rmii_mdc = gpioc.pc1.into_alternate_af11();
        let rmii_crs_dv = gpioa.pa7.into_alternate_af11();
        let rmii_rxd0 = gpioc.pc4.into_alternate_af11();
        let rmii_rxd1 = gpioc.pc5.into_alternate_af11();
        let rmii_tx_en = gpiog.pg11.into_alternate_af11();
        let rmii_txd0 = gpiog.pg13.into_alternate_af11();
        let rmii_txd1 = gpiog.pg12.into_alternate_af11();

        // Initialise ethernet...
        assert_eq!(ccdr.clocks.hclk().0, 200_000_000); // HCLK 200MHz
        assert_eq!(ccdr.clocks.pclk1().0, 100_000_000); // PCLK 100MHz
        assert_eq!(ccdr.clocks.pclk2().0, 100_000_000); // PCLK 100MHz
        assert_eq!(ccdr.clocks.pclk4().0, 100_000_000); // PCLK 100MHz

        let mac_addr = smoltcp::wire::EthernetAddress::from_bytes(&MAC_ADDRESS);
        let (eth_dma, eth_mac) = unsafe {
            ethernet::new(
                ctx.device.ETHERNET_MAC,
                ctx.device.ETHERNET_MTL,
                ctx.device.ETHERNET_DMA,
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
                &mut DES_RING,
                mac_addr.clone(),
                ccdr.peripheral.ETH1MAC,
                &ccdr.clocks,
            )
        };

        // Initialise ethernet PHY...
        let mut lan8742a = ethernet::phy::LAN8742A::new(eth_mac);
        lan8742a.phy_reset();
        lan8742a.phy_init();
        // The eth_dma should not be used until the PHY reports the link is up

        unsafe { ethernet::enable_interrupt() };

        // unsafe: mutable reference to static storage, we only do this once
        let store = unsafe { &mut STORE };
        let net = Net::new(store, eth_dma, mac_addr.into());

        // 1ms tick
        systick_init(ctx.core.SYST, ccdr.clocks);

        (
            SharedResources {},
            LocalResources {
                net,
                lan8742a,
                link_led,
            },
            init::Monotonics(),
        )
    }

    #[idle(local = [lan8742a, link_led])]
    fn idle(ctx: idle::Context) -> ! {
        loop {
            // Ethernet
            match ctx.local.lan8742a.poll_link() {
                true => ctx.local.link_led.set_low(),
                _ => ctx.local.link_led.set_high(),
            }
            .ok();
        }
    }

    #[task(binds = ETH, local = [net])]
    fn ethernet_event(ctx: ethernet_event::Context) {
        unsafe { ethernet::interrupt_handler() }

        let time = TIME.load(Ordering::Relaxed);
        ctx.local.net.poll(time as i64);
    }

    #[task(binds = SysTick, priority=15)]
    fn systick_tick(_: systick_tick::Context) {
        TIME.fetch_add(1, Ordering::Relaxed);
    }
}
