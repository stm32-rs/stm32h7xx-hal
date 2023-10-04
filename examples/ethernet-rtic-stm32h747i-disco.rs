//! Demo for STM32H747I-DISCO eval board using the Real Time for the Masses
//! (RTIC) framework.
//!
//! STM32H747I-DISCO: RMII TXD1 is on PG12
//!
//! If you are using the following boards, you will need to change the TXD1 pin
//! assignment below!
//! NUCLEO-H743ZI2: RMII TXD1 is on PB13
//! NUCLEO-H745I-Q: RMII TXD1 is on PB13
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

use core::mem::MaybeUninit;
use core::ptr::addr_of_mut;
use core::sync::atomic::AtomicU32;

use smoltcp::iface::{Config, Interface, SocketSet, SocketStorage};
use smoltcp::time::Instant;
use smoltcp::wire::{HardwareAddress, IpAddress, IpCidr};

use stm32h7xx_hal::{rcc::CoreClocks, stm32};

use stm32h7xx_hal::{
    ethernet,
    ethernet::{
        RxDescriptor, RxDescriptorRing, TxDescriptor, TxDescriptorRing, MTU,
    },
};

/// Configure SYSTICK for 1ms timebase
fn systick_init(mut syst: stm32::SYST, clocks: CoreClocks) {
    let c_ck_mhz = clocks.c_ck().to_MHz();

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

/// DesRing TD
const NUM_DESCRIPTORS: usize = 8;
/// Ethernet descriptor rings are a global singleton
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

// This data will be held by Net through a mutable reference
pub struct NetStorageStatic<'a> {
    socket_storage: [SocketStorage<'a>; 8],
}
// MaybeUninit allows us write code that is correct even if STORE is not
// initialised by the runtime
static mut STORE: MaybeUninit<NetStorageStatic> = MaybeUninit::uninit();

pub struct Net<'a> {
    iface: Interface,
    ethdev: ethernet::EthernetDMA<'a, 'a>,
    sockets: SocketSet<'a>,
}
impl<'a> Net<'a> {
    pub fn new(
        store: &'a mut NetStorageStatic<'a>,
        mut ethdev: ethernet::EthernetDMA<'a, 'a>,
        ethernet_addr: HardwareAddress,
        now: Instant,
    ) -> Self {
        let config = Config::new(ethernet_addr);
        let mut iface = Interface::new(config, &mut ethdev, now);
        // Set IP address
        iface.update_ip_addrs(|addrs| {
            let _ = addrs.push(IpCidr::new(IpAddress::v4(192, 168, 1, 99), 0));
        });

        let sockets = SocketSet::new(&mut store.socket_storage[..]);

        Net::<'a> {
            iface,
            ethdev,
            sockets,
        }
    }

    /// Polls on the ethernet interface. You should refer to the smoltcp
    /// documentation for poll() to understand how to call poll efficiently
    pub fn poll(&mut self, now: i64) {
        let timestamp = Instant::from_millis(now);

        self.iface
            .poll(timestamp, &mut self.ethdev, &mut self.sockets);
    }
}

#[rtic::app(device = stm32h7xx_hal::stm32, peripherals = true)]
mod app {
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
            .sys_ck(200.MHz())
            .hclk(200.MHz())
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
        link_led.set_high();

        let rmii_ref_clk = gpioa.pa1.into_alternate();
        let rmii_mdio = gpioa.pa2.into_alternate();
        let rmii_mdc = gpioc.pc1.into_alternate();
        let rmii_crs_dv = gpioa.pa7.into_alternate();
        let rmii_rxd0 = gpioc.pc4.into_alternate();
        let rmii_rxd1 = gpioc.pc5.into_alternate();
        let rmii_tx_en = gpiog.pg11.into_alternate();
        let rmii_txd0 = gpiog.pg13.into_alternate();
        let rmii_txd1 = gpiog.pg12.into_alternate(); // STM32H747I-DISCO

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
            ctx.device.ETHERNET_MAC,
            ctx.device.ETHERNET_MTL,
            ctx.device.ETHERNET_DMA,
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
            ctx.device.ETHERNET_MAC,
            ctx.device.ETHERNET_MTL,
            ctx.device.ETHERNET_DMA,
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
        let mut lan8742a = ethernet::phy::LAN8742A::new(eth_mac);
        lan8742a.phy_reset();
        lan8742a.phy_init();
        // The eth_dma should not be used until the PHY reports the link is up

        // unsafe: mutable reference to static storage, we only do this once
        let store = unsafe {
            let store_ptr = STORE.as_mut_ptr();

            // Initialise the socket_storage field. Using `write` instead of
            // assignment via `=` to not call `drop` on the old, uninitialised
            // value
            addr_of_mut!((*store_ptr).socket_storage)
                .write([SocketStorage::EMPTY; 8]);

            // Now that all fields are initialised we can safely use
            // assume_init_mut to return a mutable reference to STORE
            STORE.assume_init_mut()
        };

        let net = Net::new(store, eth_dma, mac_addr.into(), Instant::ZERO);

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
        }
    }

    #[task(binds = ETH, local = [net])]
    fn ethernet_event(ctx: ethernet_event::Context) {
        ethernet::eth_interrupt_handler();

        let time = TIME.load(Ordering::Relaxed);
        ctx.local.net.poll(time as i64);
    }

    #[task(binds = SysTick, priority=15)]
    fn systick_tick(_: systick_tick::Context) {
        TIME.fetch_add(1, Ordering::Relaxed);
    }
}
