//! Ethernet PHY layer for the STM32H7
//!
//! As well as this implementation, another notable implementation can
//! be found as part of the [quartiq/stabilizer] project. The two
//! implementations were developed independently, but both in the same
//! year (2019) and they have many similarities.
//!
//! In particular, reference @cjbe's [notes] on ordering accesses to
//! the DMA descriptors.
//!
//! > The CPU is allowed to access normal memory writes out-of-order. Here
//! > the write to the OWN flag in the DMA descriptor (normal memory) was
//! > placed after the DMA tail pointer advance (in device memory, so not
//! > reorderable). This meant the ethernet DMA engine stalled as it saw a
//! > descriptor it did not own, and only restarted and sent the packet when
//! > the next packet was released.
//! >
//! > This fix will work as long as the CPU data cache is disabled. If we
//! > want to enable the cache, the simplest method would be to mark SRAM3
//! > as uncacheable via the MPU.
//!
//! This implementation is derived from 0BSD-relicensed work done by
//! Johannes Draaijer <jcdra1@gmail.com> for the
//! [`stm32-eth`](https://github.com/stm32-rs/stm32-eth) project
//!
//! [quartiq/stabilizer]: https://github.com/quartiq/stabilizer
//! [notes]: https://github.com/quartiq/stabilizer/commit/ab1735950b2108eaa8d51eb63efadcd2e25c35c4

use core::task::Poll;

use crate::ptp::{EthernetPTP, Timestamp};
use crate::rcc::{rec, CoreClocks, ResetEnable};
use crate::stm32;
use crate::stm32::{Interrupt, ETHERNET_DMA, NVIC};
use futures::task::AtomicWaker;

use smoltcp::{
    self,
    phy::{
        self, ChecksumCapabilities, DeviceCapabilities, PacketMeta, RxToken,
        TxToken,
    },
    time::Instant,
    wire::EthernetAddress,
};

use super::rx::{RxDescriptor, RxDescriptorRing, RxError, RxPacket, RxRing};
use super::tx::{TxDescriptor, TxDescriptorRing, TxError, TxPacket, TxRing};

use super::packet_id::PacketId;

use crate::ethernet::raw_descriptor;
use crate::{
    ethernet::{self, PinsRMII, StationManagement},
    gpio::Speed,
};

const _RXDESC_SIZE: usize = core::mem::size_of::<RxDescriptor>();
const _TXDESC_SIZE: usize = core::mem::size_of::<TxDescriptor>();

/// Assert that our descriptors have the same size.
///
/// This is necessary as we only have a single Descriptor Skip Length
/// value which applies to both TX and RX descriptors.
const _ASSERT_DESCRIPTOR_SIZES: () = assert!(_RXDESC_SIZE == _TXDESC_SIZE);

const _ASSERT_DESCRIPTOR_ALIGN: () = assert!(_RXDESC_SIZE % 4 == 0);

const DESC_WORD_SKIP: u8 =
    ((_RXDESC_SIZE / 4) - raw_descriptor::DESC_SIZE) as u8;

const _ASSERT_DESC_WORD_SKIP_SIZE: () = assert!(DESC_WORD_SKIP <= 0b111);

// 6 DMAC, 6 SMAC, 4 q tag, 2 ethernet type II, 1500 ip MTU, 4 CRC, 2
// padding
// const ETH_BUF_SIZE: usize = 1536;

pub const PTP_MAX_SIZE: usize = 76;
pub const MAX_PTP_FOLLOWERS: usize = 16;

/// Transmit and Receive Descriptor fields
#[allow(dead_code)]
mod emac_consts {
    pub const EMAC_DES3_OWN: u32 = 0x8000_0000;
    pub const EMAC_DES3_CTXT: u32 = 0x4000_0000;
    pub const EMAC_DES3_FD: u32 = 0x2000_0000;
    pub const EMAC_DES3_LD: u32 = 0x1000_0000;
    pub const EMAC_DES3_ES: u32 = 0x0000_8000;
    pub const EMAC_TDES2_IOC: u32 = 0x8000_0000;
    pub const EMAC_RDES3_IOC: u32 = 0x4000_0000;
    pub const EMAC_RDES3_PL: u32 = 0x0000_7FFF;
    pub const EMAC_RDES3_BUF1V: u32 = 0x0100_0000;
    pub const EMAC_TDES2_B1L: u32 = 0x0000_3FFF;
    pub const EMAC_DES0_BUF1AP: u32 = 0xFFFF_FFFF;
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq)]
/// This struct is returned if a packet ID is not associated
/// with any TX or RX descriptors.
pub struct PacketIdNotFound;

#[derive(Copy)]
pub struct PtpFrameWithId {
    ptp_frame: [u8; PTP_MAX_SIZE],
    packet_id: Option<PacketId>,
}

/// Ethernet DMA.
pub struct EthernetDMA<'rx, 'tx> {
    eth_dma: ETHERNET_DMA,
    // eth_mtl: crate::stm32::ETHERNET_MTL,
    rx_ring: RxRing<'rx>,
    tx_ring: TxRing<'tx>,

    #[cfg(feature = "ptp")]
    packet_id_counter: u32,
    #[cfg(feature = "ptp")]
    ptp_frame_buffer: [PtpFrameWithId; MAX_PTP_FOLLOWERS],
    #[cfg(feature = "ptp")]
    write_pos: usize,
}

/// Ethernet MAC
pub struct EthernetMAC {
    eth_mac: stm32::ETHERNET_MAC,
    eth_phy_addr: u8,
    clock_range: u8,
}

/// Access to all configured parts of the ethernet peripheral.
pub struct Parts<'rx, 'tx> {
    /// Access to and control over the ethernet MAC.
    pub mac: EthernetMAC,
    /// Access to and control over the ethernet DMA.
    pub dma: EthernetDMA<'rx, 'tx>,
    /// Access to and control over the ethernet PTP module.
    #[cfg(feature = "ptp")]
    pub ptp: EthernetPTP,
}

/// Create and initialise the ethernet driver.
///
/// You must move in ETH_MAC, ETH_MTL, ETH_DMA.
///
/// Sets up the descriptor structures, sets up the peripheral
/// clocks and GPIO configuration, and configures the ETH MAC and
/// DMA peripherals. Automatically sets slew rate to VeryHigh.
/// If you wish to use another configuration, please see
/// [new_unchecked](new_unchecked).
///
/// This method does not initialise the external PHY. However it does return an
/// [EthernetMAC](EthernetMAC) which implements the
/// [StationManagement](super::StationManagement) trait. This can be used to
/// communicate with the external PHY.
///
/// # Safety
///
/// `EthernetDMA` shall not be moved as it is initialised here
#[allow(clippy::too_many_arguments)]
pub fn new<'rx, 'tx>(
    eth_mac: stm32::ETHERNET_MAC,
    eth_mtl: stm32::ETHERNET_MTL,
    eth_dma: stm32::ETHERNET_DMA,
    mut pins: impl PinsRMII,
    rx_buffer: RxDescriptorRing<'rx>,
    tx_buffer: TxDescriptorRing<'tx>,
    mac_addr: EthernetAddress,
    prec: rec::Eth1Mac,
    clocks: &CoreClocks,
) -> Parts<'rx, 'tx> {
    pins.set_speed(Speed::VeryHigh);
    unsafe {
        new_unchecked(
            eth_mac, eth_mtl, eth_dma, rx_buffer, tx_buffer, mac_addr, prec,
            clocks,
        )
    }
}

/// Create and initialise the ethernet driver.
///
/// You must move in ETH_MAC, ETH_MTL, ETH_DMA.
///
/// Sets up the descriptor structures, sets up the peripheral
/// clocks and GPIO configuration, and configures the ETH MAC and
/// DMA peripherals.
///
/// This method does not initialise the external PHY. However it does return an
/// [EthernetMAC](EthernetMAC) which implements the
/// [StationManagement](super::StationManagement) trait. This can be used to
/// communicate with the external PHY.
///
/// All the documented interrupts in the `MMC_TX_INTERRUPT_MASK` and
/// `MMC_RX_INTERRUPT_MASK` registers are masked, since these cause unexpected
/// interrupts after a number of days of heavy ethernet traffic. If these
/// interrupts are desired, you can be unmask them in your own code after this
/// method.
///
/// # Safety
///
/// `EthernetDMA` shall not be moved as it is initialised here
pub unsafe fn new_unchecked<'rx, 'tx>(
    eth_mac: stm32::ETHERNET_MAC,
    eth_mtl: stm32::ETHERNET_MTL,
    eth_dma: stm32::ETHERNET_DMA,
    rx_buffer: RxDescriptorRing<'rx>,
    tx_buffer: TxDescriptorRing<'tx>,
    mac_addr: EthernetAddress,
    prec: rec::Eth1Mac,
    clocks: &CoreClocks,
) -> Parts<'rx, 'tx> {
    // RCC
    {
        let rcc = &*stm32::RCC::ptr();
        let syscfg = &*stm32::SYSCFG::ptr();

        // Ensure syscfg is enabled (for PMCR)
        rcc.apb4enr.modify(|_, w| w.syscfgen().set_bit());

        // AHB1 ETH1MACEN
        prec.enable();

        // Also need to enable the transmission and reception clocks, which
        // don't have prec objects. They don't have prec objects because they
        // can't be reset.
        rcc.ahb1enr
            .modify(|_, w| w.eth1txen().set_bit().eth1rxen().set_bit());

        syscfg.pmcr.modify(|_, w| w.epis().bits(0b100)); // RMII
    }

    cortex_m::interrupt::free(|_cs| {
        // reset ETH_DMA - write 1 and wait for 0
        eth_dma.dmamr.modify(|_, w| w.swr().set_bit());
        while eth_dma.dmamr.read().swr().bit_is_set() {}

        // 200 MHz
        eth_mac
            .mac1ustcr
            .modify(|_, w| w.tic_1us_cntr().bits(200 - 1));

        // Configuration Register
        eth_mac.maccr.modify(|_, w| {
            w.arpen()
                .clear_bit()
                .ipc()
                .set_bit()
                .ipg()
                .bits(0b000) // 96 bit
                .ecrsfd()
                .clear_bit()
                .dcrs()
                .clear_bit()
                .bl()
                .bits(0b00) // 19
                .prelen()
                .bits(0b00) // 7
                // CRC stripping for Type frames
                .cst()
                .set_bit()
                // Fast Ethernet speed
                .fes()
                .set_bit()
                // Duplex mode
                .dm()
                .set_bit()
                // Automatic pad/CRC stripping
                .acs()
                .set_bit()
                // Retry disable in half-duplex mode
                .dr()
                .set_bit()
        });

        eth_mac.macecr.modify(|_, w| {
            w.eipgen()
                .clear_bit()
                .usp()
                .clear_bit()
                .spen()
                .clear_bit()
                .dcrcc()
                .clear_bit()
        });
        // Set the MAC address.
        // Writes to LR trigger both registers to be loaded into the MAC,
        // so write to LR last.
        eth_mac.maca0hr.write(|w| {
            w.addrhi().bits(
                u16::from(mac_addr.0[4]) | (u16::from(mac_addr.0[5]) << 8),
            )
        });
        eth_mac.maca0lr.write(|w| {
            w.addrlo().bits(
                u32::from(mac_addr.0[0])
                    | (u32::from(mac_addr.0[1]) << 8)
                    | (u32::from(mac_addr.0[2]) << 16)
                    | (u32::from(mac_addr.0[3]) << 24),
            )
        });
        // frame filter register
        eth_mac.macpfr.modify(|_, w| {
            #[cfg(feature = "ptp")]
            {
                w.dntu()
                    .clear_bit()
                    .ipfe()
                    .clear_bit()
                    .vtfe()
                    .clear_bit()
                    .hpf()
                    .clear_bit()
                    .saf()
                    .clear_bit()
                    .saif()
                    .clear_bit()
                    .pcf()
                    .bits(0b00)
                    .dbf()
                    .clear_bit()
                    .pm()
                    .set_bit()
                    .daif()
                    .clear_bit()
                    .hmc()
                    .clear_bit()
                    .huc()
                    .clear_bit()
                    // Receive All
                    .ra()
                    .clear_bit()
                    // Promiscuous mode
                    .pr()
                    .clear_bit()
            }
            #[cfg(not(feature = "ptp"))]
            {
                w.dntu()
                    .clear_bit()
                    .ipfe()
                    .clear_bit()
                    .vtfe()
                    .clear_bit()
                    .hpf()
                    .clear_bit()
                    .saf()
                    .clear_bit()
                    .saif()
                    .clear_bit()
                    .pcf()
                    .bits(0b00)
                    .dbf()
                    .clear_bit()
                    .pm()
                    .clear_bit()
                    .daif()
                    .clear_bit()
                    .hmc()
                    .clear_bit()
                    .huc()
                    .clear_bit()
                    // Receive All
                    .ra()
                    .clear_bit()
                    // Promiscuous mode
                    .pr()
                    .clear_bit()
            }
        });
        eth_mac.macwtr.write(|w| w.pwe().clear_bit());
        // Flow Control Register
        eth_mac.macqtx_fcr.modify(|_, w| {
            // Pause time
            w.pt().bits(0x100)
        });
        eth_mac.macrx_fcr.modify(|_, w| w);

        // Mask away Ethernet MAC MMC RX/TX interrupts. These are statistics
        // counter interrupts and are enabled by default. We need to manually
        // disable various ethernet interrupts so they don't unintentionally
        // hang the device. The user is free to re-enable them later to provide
        // ethernet MAC-related statistics
        eth_mac.mmc_rx_interrupt_mask.modify(|_, w| {
            w.rxlpiuscim()
                .set_bit()
                .rxucgpim()
                .set_bit()
                .rxalgnerpim()
                .set_bit()
                .rxcrcerpim()
                .set_bit()
        });

        eth_mac.mmc_tx_interrupt_mask.modify(|_, w| {
            w.txlpiuscim()
                .set_bit()
                .txgpktim()
                .set_bit()
                .txmcolgpim()
                .set_bit()
                .txscolgpim()
                .set_bit()
        });
        // TODO: The MMC_TX/RX_INTERRUPT_MASK registers incorrectly mark
        // LPITRCIM as read-only, so svd2rust doens't generate bindings to
        // modify them. Instead, as a workaround, we manually manipulate the
        // bits
        eth_mac
            .mmc_tx_interrupt_mask
            .modify(|r, w| w.bits(r.bits() | (1 << 27)));
        eth_mac
            .mmc_rx_interrupt_mask
            .modify(|r, w| w.bits(r.bits() | (1 << 27)));

        eth_mtl.mtlrx_qomr.modify(|_, w| {
            w
                // Receive store and forward
                .rsf()
                .set_bit()
                // Dropping of TCP/IP checksum error frames disable
                .dis_tcp_ef()
                .clear_bit()
                // Forward error frames
                .fep()
                .clear_bit()
                // Forward undersized good packets
                .fup()
                .clear_bit()
        });
        eth_mtl.mtltx_qomr.modify(|_, w| {
            w
                // Transmit store and forward
                .tsf()
                .set_bit()
        });

        // operation mode register
        eth_dma.dmamr.modify(|_, w| {
            w
                // Rx Tx priority ratio 2:1
                .pr()
                .variant(0b001)
        });
        // bus mode register
        eth_dma.dmasbmr.modify(|_, w| {
            // Address-aligned beats
            w.aal().set_bit()
        });
        eth_dma
            .dmaccr
            .modify(|_, w| w.dsl().variant(DESC_WORD_SKIP));
        eth_dma.dmactx_cr.modify(|_, w| {
            w
                // Tx DMA PBL
                .txpbl()
                .bits(32)
                // Operate on second frame
                .osf()
                .clear_bit()
        });

        eth_dma.dmacrx_cr.modify(|_, w| {
            w
                // receive buffer size
                .rbsz()
                .variant(rx_buffer.first_buffer().len() as u16)
                // Rx DMA PBL
                .rxpbl()
                .bits(32)
        });

        // Ensure the DMA descriptors are committed
        cortex_m::asm::dsb();

        // Manage MAC transmission and reception
        eth_mac.maccr.modify(|_, w| {
            w.re()
                .bit(true) // Receiver Enable
                .te()
                .bit(true) // Transmiter Enable
        });
        eth_mtl.mtltx_qomr.modify(|_, w| w.ftq().set_bit());

        eth_dma
            .dmacsr
            .modify(|_, w| w.tps().set_bit().rps().set_bit()); //These bits are set if transmission is stopped.
    });

    // MAC layer

    // Set the MDC clock frequency in the range 1MHz - 2.5MHz
    let hclk_mhz = clocks.hclk().raw() / 1_000_000;
    let csr_clock_range = match hclk_mhz {
        0..=34 => 2,    // Divide by 16
        35..=59 => 3,   // Divide by 26
        60..=99 => 0,   // Divide by 42
        100..=149 => 1, // Divide by 62
        150..=249 => 4, // Divide by 102
        250..=310 => 5, // Divide by 124
        _ => panic!(
            "HCLK results in MDC clock > 2.5MHz even for the \
             highest CSR clock divider"
        ),
    };

    let mac = EthernetMAC {
        eth_mac,
        eth_phy_addr: 0,
        clock_range: csr_clock_range,
    };

    let mut dma = EthernetDMA {
        eth_dma,
        // eth_mtl,
        rx_ring: RxRing::new(rx_buffer),
        tx_ring: TxRing::new(tx_buffer),

        #[cfg(feature = "ptp")]
        packet_id_counter: 0,
        #[cfg(feature = "ptp")]
        ptp_frame_buffer: [PtpFrameWithId {
            ptp_frame: [0u8; PTP_MAX_SIZE],
            packet_id: None,
        }; MAX_PTP_FOLLOWERS],
        #[cfg(feature = "ptp")]
        write_pos: 0,
    };
    dma.rx_ring.start(&dma.eth_dma);
    dma.tx_ring.start(&dma.eth_dma);

    // Configure the ethernet PTP
    #[cfg(feature = "ptp")]
    let ptp = EthernetPTP::new(*clocks, &dma);

    Parts {
        mac,
        dma,
        #[cfg(feature = "ptp")]
        ptp,
    }
}

/// A summary of the reasons for the occurence of an
/// interrupt
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InterruptReason {
    /// A packet has arrived and is ready for processing.
    pub rx: bool,
    /// A packet was sent, and a TX slot has freed up.
    pub tx: bool,
    /// A DMA error occured.
    pub dma_error: bool,
    #[cfg(all(feature = "ptp"))]
    /// The target time configured for PTP has
    /// passed.
    pub time_passed: bool,
}

/// Handle the `ETH` interrupt.
///
/// This function wakes wakers and resets
/// interrupt bits relevant in that interrupt.
#[cfg(feature = "device-selected")]
pub fn eth_interrupt_handler() -> InterruptReason {
    let dma = EthernetDMA::interrupt_handler();

    #[cfg(feature = "ptp")]
    let is_time_trigger = EthernetPTP::interrupt_handler();

    InterruptReason {
        rx: dma.is_rx,
        tx: dma.is_tx,
        dma_error: dma.is_error,
        #[cfg(all(feature = "ptp"))]
        time_passed: is_time_trigger,
    }
}

impl EthernetMAC {
    /// Sets the SMI address to use for the PHY
    pub fn set_phy_addr(self, eth_phy_addr: u8) -> Self {
        Self {
            eth_mac: self.eth_mac,
            eth_phy_addr,
            clock_range: self.clock_range,
        }
    }
}

/// PHY Operations
impl StationManagement for EthernetMAC {
    /// Read a register over SMI.
    fn smi_read(&mut self, reg: u8) -> u16 {
        while self.eth_mac.macmdioar.read().mb().bit_is_set() {}
        self.eth_mac.macmdioar.modify(|_, w| unsafe {
            w.pa()
                .bits(self.eth_phy_addr)
                .rda()
                .bits(reg)
                .goc()
                .bits(0b11) // read
                .cr()
                .bits(self.clock_range)
                .mb()
                .set_bit()
        });
        while self.eth_mac.macmdioar.read().mb().bit_is_set() {}
        self.eth_mac.macmdiodr.read().md().bits()
    }

    /// Write a register over SMI.
    fn smi_write(&mut self, reg: u8, val: u16) {
        while self.eth_mac.macmdioar.read().mb().bit_is_set() {}
        self.eth_mac
            .macmdiodr
            .write(|w| unsafe { w.md().bits(val) });
        self.eth_mac.macmdioar.modify(|_, w| unsafe {
            w.pa()
                .bits(self.eth_phy_addr)
                .rda()
                .bits(reg)
                .goc()
                .bits(0b01) // write
                .cr()
                .bits(self.clock_range)
                .mb()
                .set_bit()
        });
        while self.eth_mac.macmdioar.read().mb().bit_is_set() {}
    }
}

/// An Ethernet RX token that can be consumed in order to receive
/// an ethernet packet.
pub struct EthRxToken<'a, 'rx> {
    rx_ring: &'a mut RxRing<'rx>,
    #[cfg(feature = "ptp")]
    meta: PacketId,
    #[cfg(feature = "ptp")]
    buf: &'a mut PtpFrameWithId,
}

impl<'dma, 'rx> RxToken for EthRxToken<'dma, 'rx> {
    fn consume<R, F>(self, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        #[cfg(feature = "ptp")]
        let meta = Some(self.meta);

        #[cfg(not(feature = "ptp"))]
        let meta = None;

        // NOTE(unwrap): an `EthRxToken` is only created when `eth.rx_available()`
        let mut packet = self.rx_ring.recv_next(meta).ok().unwrap();
        #[cfg(feature = "ptp")]
        {
            let ethertype =
                u16::from_be_bytes(packet[12..14].try_into().unwrap());
            if ethertype == 0x88F7 {
                let packet_buf = &packet[14..];
                ((self.buf.ptp_frame)[0..packet_buf.len()])
                    .copy_from_slice(packet_buf);
                self.buf.packet_id = meta;
            }
        }
        let result = f(&mut packet);
        packet.free();
        result
    }

    #[cfg(feature = "ptp")]
    fn meta(&self) -> smoltcp::phy::PacketMeta {
        self.meta.into()
    }
}

/// Just a reference to [`EthernetDMA`] for sending a
/// packet later with [`TxToken::consume()`].
pub struct EthTxToken<'a, 'tx> {
    tx_ring: &'a mut TxRing<'tx>,
    #[cfg(feature = "ptp")]
    meta: Option<PacketId>,
}

impl<'dma, 'tx> TxToken for EthTxToken<'dma, 'tx> {
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        #[cfg(feature = "ptp")]
        let meta = self.meta.map(Into::into);
        #[cfg(not(feature = "ptp"))]
        let meta = None;

        // NOTE(unwrap): an `EthTxToken` is only created if
        // there is a descriptor available for sending.
        let mut tx_packet = self.tx_ring.send_next(len, meta).ok().unwrap();
        let res = f(&mut tx_packet);
        tx_packet.send();
        res
    }

    #[cfg(feature = "ptp")]
    fn set_meta(&mut self, meta: PacketMeta) {
        self.meta = Some(meta.into());
    }
}

impl<'a, 'rx, 'tx> phy::Device for &'a mut EthernetDMA<'rx, 'tx> {
    type RxToken<'token> = EthRxToken<'token, 'rx> where Self: 'token;
    type TxToken<'token> = EthTxToken<'token, 'tx> where Self: 'token;

    fn capabilities(&self) -> DeviceCapabilities {
        let mut caps = DeviceCapabilities::default();
        caps.max_transmission_unit = ethernet::MTU;
        caps.max_burst_size = Some(1);
        caps.checksum = ChecksumCapabilities::ignored();
        caps
    }

    fn receive(
        &mut self,
        _timestamp: Instant,
    ) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        if self.tx_available() && self.rx_available() {
            #[cfg(feature = "ptp")]
            let rx_packet_id = self.next_packet_id();

            let EthernetDMA {
                rx_ring, tx_ring, ..
            } = self;

            let rx = EthRxToken {
                rx_ring,
                #[cfg(feature = "ptp")]
                meta: rx_packet_id,
                #[cfg(feature = "ptp")]
                buf: &mut self.ptp_frame_buffer[self.write_pos],
            };

            let tx = EthTxToken {
                tx_ring,
                #[cfg(feature = "ptp")]
                meta: None,
            };
            Some((rx, tx))
        } else {
            None
        }
    }

    fn transmit(&mut self, _timestamp: Instant) -> Option<Self::TxToken<'_>> {
        if self.tx_available() {
            let tx_packet_id = self.next_packet_id();

            let EthernetDMA { tx_ring, .. } = self;
            Some(EthTxToken {
                tx_ring,
                #[cfg(feature = "ptp")]
                meta: Some(tx_packet_id),
            })
        } else {
            None
        }
    }
}

/// Use this Ethernet driver with [smoltcp](https://github.com/smoltcp-rs/smoltcp)
impl<'rx, 'tx> phy::Device for EthernetDMA<'rx, 'tx> {
    type RxToken<'token> = EthRxToken<'token, 'rx> where Self: 'token;
    type TxToken<'token> = EthTxToken<'token, 'tx> where Self: 'token;

    fn capabilities(&self) -> DeviceCapabilities {
        let mut caps = DeviceCapabilities::default();
        caps.max_transmission_unit = ethernet::MTU;
        caps.max_burst_size = Some(1);
        caps.checksum = ChecksumCapabilities::ignored();
        caps
    }

    fn receive(
        &mut self,
        _timestamp: Instant,
    ) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        if self.tx_available() && self.rx_available() {
            #[cfg(feature = "ptp")]
            let rx_packet_id = self.next_packet_id();

            let EthernetDMA {
                rx_ring, tx_ring, ..
            } = self;

            let rx = EthRxToken {
                rx_ring,
                #[cfg(feature = "ptp")]
                meta: rx_packet_id,
                #[cfg(feature = "ptp")]
                buf: &mut self.ptp_frame_buffer[self.write_pos],
            };

            #[cfg(feature = "ptp")]
            {
                self.write_pos = (self.write_pos + 1) % MAX_PTP_FOLLOWERS;
            }

            let tx = EthTxToken {
                tx_ring,
                #[cfg(feature = "ptp")]
                meta: None,
            };
            Some((rx, tx))
        } else {
            None
        }
    }

    fn transmit(&mut self, _timestamp: Instant) -> Option<Self::TxToken<'_>> {
        if self.tx_available() {
            let tx_packet_id = self.next_packet_id();

            let EthernetDMA { tx_ring, .. } = self;
            Some(EthTxToken {
                tx_ring,
                #[cfg(feature = "ptp")]
                meta: Some(tx_packet_id),
            })
        } else {
            None
        }
    }
}

impl<'a, 'rx, 'tx> EthernetDMA<'rx, 'tx> {
    #[cfg(feature = "ptp")]
    pub fn get_frame_from(
        &'a self,
        clock_identity: u64,
    ) -> Option<(&'a PtpFrameWithId, usize)> {
        for i in 0..MAX_PTP_FOLLOWERS {
            if self.ptp_frame_buffer[i].packet_id.is_some() {
                // defmt::info!("buffer = {}", self.ptp_frame_buffer[i].0);
                if u64::from_be_bytes(
                    self.ptp_frame_buffer[i].ptp_frame[20..28]
                        .try_into()
                        .unwrap(),
                ) == clock_identity
                {
                    return Some((&self.ptp_frame_buffer[i], i));
                }
            }
        }
        None
    }
    #[cfg(feature = "ptp")]
    pub fn invalidate_frame_at(&'a mut self, pos: usize) {
        if pos < MAX_PTP_FOLLOWERS {
            self.ptp_frame_buffer[pos].packet_id = None;
        }
    }
    #[cfg(feature = "ptp")]
    pub fn send_ptp_frame(
        frame: &[u8],
        tx_option: Option<
            <EthernetDMA<'static, 'static> as phy::Device>::TxToken<'_>,
        >,
        meta: PacketId,
    ) {
        if let Some(mut tx_token) = tx_option {
            tx_token.set_meta(meta.into());
            tx_token.consume(frame.len(), |buf| {
                buf[..frame.len()].copy_from_slice(frame);
            });
        }
    }
    /// Return the number of packets dropped since this method was
    /// last called
    pub fn number_packets_dropped(&self) -> u32 {
        self.eth_dma.dmacmfcr.read().mfc().bits() as u32
    }

    fn eth_dma(&self) -> &ETHERNET_DMA {
        &self.eth_dma
    }

    /// Split the [`EthernetDMA`] into concurrently operating send and
    /// receive parts.
    pub fn split(&mut self) -> (&mut RxRing<'rx>, &mut TxRing<'tx>) {
        (&mut self.rx_ring, &mut self.tx_ring)
    }

    /// Enable RX and TX interrupts
    ///
    /// In your handler you must call
    /// [`EthernetDMA::interrupt_handler()`] or [`stm32_eth::eth_interrupt_handler`](crate::eth_interrupt_handler)
    /// to clear interrupt pending bits. Otherwise the interrupt will reoccur immediately.
    ///
    /// [`EthernetPTP::interrupt_handler()`]: crate::ptp::EthernetPTP::interrupt_handler
    #[cfg_attr(
        feature = "ptp",
        doc = "If you have PTP enabled, you must also call [`EthernetPTP::interrupt_handler()`] if you wish to make use of the PTP timestamp trigger feature."
    )]
    pub fn enable_interrupt(&self) {
        self.eth_dma().dmacier.modify(|_, w| {
            w
                // Normal interrupt summary enable
                .nie()
                .set_bit()
                // Receive Interrupt Enable
                .rie()
                .set_bit()
                // Transmit Interrupt Enable
                .tie()
                .set_bit()
                // Abnormal Interrupt Summary enable
                .aie()
                .set_bit()
                // Receive Buffer Unavailable
                .rbue()
                .set_bit()
                // Transmit Buffer Unavailable
                .tbue()
                .set_bit()
        });

        // Enable ethernet interrupts
        unsafe {
            NVIC::unmask(Interrupt::ETH);
        }
    }

    pub fn panic_fbe() -> ! {
        // SAFETY: we only perform atomic reads/writes through `eth_dma`.
        let eth_dma = unsafe { &*ETHERNET_DMA::ptr() };

        let tx_descriptor_addr = eth_dma.dmaccatx_dr.read().bits();
        let tx_buffer_addr = eth_dma.dmaccatx_br.read().bits();

        let rx_descriptor_addr = eth_dma.dmaccarx_dr.read().bits();
        let rx_buffer_addr = eth_dma.dmaccarx_br.read().bits();

        // TODO: add a link to a/the github issue describing this problem,
        // and how to solve it.
        panic!("Fatal bus error! Is the descriptor and buffer memory accessible by the Ethernet MAC/DMA? TXDESC: {:08X}, TXBUF: {:08X}, RXDESC: {:08X}, TXDESC: {:08X}", tx_descriptor_addr, tx_buffer_addr, rx_descriptor_addr, rx_buffer_addr);
    }

    /// Handle the DMA parts of the `ETH` interrupt.
    pub(crate) fn interrupt_handler() -> InterruptReasonSummary {
        // SAFETY: we only perform atomic reads/writes through `eth_dma`.
        let eth_dma = unsafe { &*ETHERNET_DMA::ptr() };

        let (is_rx, is_tx, is_error) = {
            // Read register
            let status = eth_dma.dmacsr.read();

            // Reset bits
            eth_dma.dmacsr.write(|w| {
                w.nis()
                    .set_bit()
                    .ais()
                    .set_bit()
                    .ti()
                    .set_bit()
                    .ri()
                    .set_bit()
                    .rbu()
                    .set_bit()
                    .tbu()
                    .set_bit()
            });

            if status.fbe().bit_is_set() {
                EthernetDMA::panic_fbe();
            }

            (
                status.ri().bit_is_set() || status.rbu().bit_is_set(),
                status.ti().bit_is_set() || status.tbu().bit_is_set(),
                status.ais().bit_is_set(),
            )
        };

        let status = InterruptReasonSummary {
            is_rx,
            is_tx,
            is_error,
        };

        #[cfg(feature = "async-await")]
        {
            if status.is_tx {
                EthernetDMA::tx_waker().wake();
            }

            if status.is_rx {
                EthernetDMA::rx_waker().wake();
            }
        }

        status
    }

    /// Try to receive a packet.
    ///
    /// If no packet is available, this function returns [`Err(RxError::WouldBlock)`](RxError::WouldBlock).
    ///
    /// It may also return another kind of [`RxError`].
    pub fn recv_next(
        &mut self,
        packet_id: Option<PacketId>,
    ) -> Result<RxPacket, RxError> {
        self.rx_ring.recv_next(packet_id.map(Into::into))
    }

    /// Is Rx DMA currently running?
    ///
    /// It stops if the ring is full. Call [`EthernetDMA::recv_next()`] to free an
    /// entry and to demand poll from the hardware.
    pub fn rx_is_running(&self) -> bool {
        RxRing::running_state().is_running()
    }

    /// Is Tx DMA currently running?
    pub fn tx_is_running(&self) -> bool {
        TxRing::is_running()
    }

    /// Try to send a packet with data.
    ///
    /// If there are no free TX slots, this function will
    /// return [`Err(TxError::WouldBlock)`](TxError::WouldBlock).
    pub fn send<F>(
        &mut self,
        length: usize,
        packet_id: Option<PacketId>,
        f: F,
    ) -> Result<(), TxError>
    where
        F: FnOnce(&mut [u8]),
    {
        let mut tx_packet = self.tx_ring.send_next(length, packet_id)?;
        f(&mut tx_packet);
        tx_packet.send();

        Ok(())
    }

    /// Check if there is a packet available for reading.
    ///
    /// If this function returns true, it is guaranteed that the
    /// next call to [`EthernetDMA::recv_next`] will return [`Ok`].
    pub fn rx_available(&mut self) -> bool {
        self.rx_ring.next_entry_available()
    }

    /// Check if sending a packet now would succeed.
    ///
    /// If this function returns true, it is guaranteed that
    /// the next call to [`EthernetDMA::send`] will return [`Ok`]
    pub fn tx_available(&mut self) -> bool {
        self.tx_ring.next_entry_available()
    }
}

impl Drop for EthernetDMA<'_, '_> {
    // On drop, stop all DMA actions.
    fn drop(&mut self) {
        self.tx_ring.stop(self.eth_dma());
        self.rx_ring.stop(self.eth_dma());
    }
}

#[cfg(feature = "async-await")]
impl<'rx, 'tx> EthernetDMA<'rx, 'tx> {
    pub(crate) fn rx_waker() -> &'static AtomicWaker {
        static WAKER: AtomicWaker = AtomicWaker::new();
        &WAKER
    }

    pub(crate) fn tx_waker() -> &'static AtomicWaker {
        static WAKER: AtomicWaker = AtomicWaker::new();
        &WAKER
    }

    /// Receive a packet.
    ///
    /// See [`RxRing::recv`].
    pub async fn recv(&mut self, packet_id: Option<PacketId>) -> RxPacket {
        self.rx_ring.recv(packet_id).await
    }

    /// Prepare a packet for sending.
    ///
    /// See [`TxRing::prepare_packet`].
    pub async fn prepare_packet(
        &mut self,
        length: usize,
        packet_id: Option<PacketId>,
    ) -> TxPacket {
        self.tx_ring.prepare_packet(length, packet_id).await
    }

    /// Wait for an RX or TX interrupt to have
    /// occured.
    pub async fn rx_or_tx(&mut self) {
        let mut polled_once = false;
        core::future::poll_fn(|ctx| {
            if polled_once {
                Poll::Ready(())
            } else {
                polled_once = true;
                EthernetDMA::rx_waker().register(ctx.waker());
                EthernetDMA::tx_waker().register(ctx.waker());
                Poll::Pending
            }
        })
        .await;
    }
}

#[cfg(feature = "ptp")]
/// PTP methods.
impl EthernetDMA<'_, '_> {
    /// Try to get the timestamp for the given packet ID.
    ///
    /// This function will attempt to find both RX and TX timestamps,
    /// so make sure that the provided packet ID is unique between the two.
    pub fn poll_timestamp(
        &self,
        packet_id: &PacketId,
    ) -> Poll<Result<Option<Timestamp>, PacketIdNotFound>> {
        // Check if it's a TX packet
        let tx = self.poll_tx_timestamp(packet_id);

        if tx != Poll::Ready(Err(PacketIdNotFound)) {
            return tx;
        }

        // It's not a TX packet, check if it's an RX packet
        Poll::Ready(self.rx_timestamp(packet_id))
    }

    /// Get the RX timestamp for the given packet ID.
    pub fn rx_timestamp(
        &self,
        packet_id: &PacketId,
    ) -> Result<Option<Timestamp>, PacketIdNotFound> {
        self.rx_ring.timestamp(packet_id)
    }

    /// Blockingly wait until the TX timestamp for
    /// the given ID is available.
    pub fn wait_for_tx_timestamp(
        &self,
        packet_id: &PacketId,
    ) -> Result<Option<Timestamp>, PacketIdNotFound> {
        self.tx_ring.wait_for_timestamp(packet_id)
    }

    /// Poll to check if the TX timestamp for the given
    /// ID is available.
    pub fn poll_tx_timestamp(
        &self,
        packet_id: &PacketId,
    ) -> Poll<Result<Option<Timestamp>, PacketIdNotFound>> {
        self.tx_ring.poll_timestamp(packet_id)
    }

    /// Get the TX timestamp for the given ID.
    #[cfg(feature = "async-await")]
    pub async fn tx_timestamp(
        &mut self,
        packet_id: &PacketId,
    ) -> Result<Option<Timestamp>, PacketIdNotFound> {
        self.tx_ring.timestamp(packet_id).await
    }

    /// Get the next packet ID.
    pub fn next_packet_id(&mut self) -> PacketId {
        let id = PacketId(self.packet_id_counter);
        self.packet_id_counter += 1;
        id
    }
}

/// A summary of the reasons for the interrupt
/// that occured
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy)]
pub struct InterruptReasonSummary {
    /// The interrupt was caused by an RX event.
    pub is_rx: bool,
    /// The interrupt was caused by an TX event.
    pub is_tx: bool,
    /// The interrupt was caused by an error event.
    pub is_error: bool,
}
