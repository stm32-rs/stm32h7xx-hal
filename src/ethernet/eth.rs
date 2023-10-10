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
//! [quartiq/stabilizer]: https://github.com/quartiq/stabilizer
//! [notes]: https://github.com/quartiq/stabilizer/commit/ab1735950b2108eaa8d51eb63efadcd2e25c35c4

use core::ptr;
#[cfg(feature = "ptp")]
use core::task::Poll;

use crate::rcc::{rec, CoreClocks, ResetEnable};
use crate::stm32;

use smoltcp::{
    self,
    phy::{self, DeviceCapabilities},
    time::Instant,
    wire::EthernetAddress,
};

use crate::{
    ethernet::{PinsRMII, StationManagement},
    gpio::Speed,
};

// 6 DMAC, 6 SMAC, 4 q tag, 2 ethernet type II, 1500 ip MTU, 4 CRC, 2
// padding
const ETH_BUF_SIZE: usize = 1536;

/// Transmit and Receive Descriptor fields
#[allow(dead_code)]
mod emac_consts {
    pub const EMAC_DES3_OWN: u32 = 0x8000_0000;
    pub const EMAC_DES3_CTXT: u32 = 0x4000_0000;
    pub const EMAC_DES3_FD: u32 = 0x2000_0000;
    pub const EMAC_DES3_LD: u32 = 0x1000_0000;
    pub const EMAC_DES3_ES: u32 = 0x0000_8000;
    pub const EMAC_TDES2_IOC: u32 = 0x8000_0000;
    pub const EMAC_TDES2_TTSE: u32 = 0x4000_0000;
    pub const EMAC_TDES3_TTSS: u32 = 0x0002_0000;
    pub const EMAC_RDES3_IOC: u32 = 0x4000_0000;
    pub const EMAC_RDES1_TSA: u32 = 0x0000_4000;
    pub const EMAC_RDES3_PL: u32 = 0x0000_7FFF;
    pub const EMAC_RDES3_BUF1V: u32 = 0x0100_0000;
    pub const EMAC_TDES2_B1L: u32 = 0x0000_3FFF;
    pub const EMAC_DES0_BUF1AP: u32 = 0xFFFF_FFFF;
}
use self::emac_consts::*;

#[cfg(feature = "ptp")]
pub use super::{Timestamp, EthernetPTP};

/// A struct to store the packet meta and the timestamp
#[derive(Clone, Copy)]
#[repr(C, packed)]
#[cfg(feature = "ptp")]
pub struct PacketInfo {
    packet_meta: Option<smoltcp::phy::PacketMeta>,
    timestamp: Option<Timestamp>,
}

#[derive(Clone, Copy, PartialEq)]
#[cfg(feature = "ptp")]
pub struct PacketMetaNotFound;

#[cfg(feature = "ptp")]
impl Default for PacketInfo {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(feature = "ptp")]
impl PacketInfo {
    pub const fn new() -> Self {
        Self {
            packet_meta: None,
            timestamp: None,
        }
    }

    pub fn set_meta_and_clear_ts(&mut self, packet_meta: Option<smoltcp::phy::PacketMeta>) {
        self.packet_meta = packet_meta;
        self.timestamp = None;
    }

    pub fn meta(&self) -> Option<smoltcp::phy::PacketMeta> {
        self.packet_meta 
    }

    pub fn ts(&self) -> Option<Timestamp> {
        self.timestamp
    }

    pub fn set_ts(&mut self, timestamp: Option<Timestamp>) {
        self.timestamp = timestamp;
    }
}
/// Transmit Descriptor representation
///
/// * tdes0: transmit buffer address
/// * tdes1:
/// * tdes2: buffer lengths
/// * tdes3: control and payload/frame length
///
/// Note that Copy and Clone are derived to support initialising an
/// array of TDes, but you may not move a TDes after its address has
/// been given to the ETH_DMA engine.
#[derive(Copy, Clone)]
#[repr(C, packed)]
struct TDes {
    tdes0: u32,
    tdes1: u32,
    tdes2: u32,
    tdes3: u32,
}

impl TDes {
    /// Initialises this TDes to point at the given buffer.
    pub fn init(&mut self) {
        self.tdes0 = 0;
        self.tdes1 = 0;
        self.tdes2 = 0;
        self.tdes3 = 0; // Owned by us
    }

    /// Return true if this TDes is not currently owned by the DMA
    pub fn available(&self) -> bool {
        self.tdes3 & EMAC_DES3_OWN == 0
    }
}

/// Store a ring of TDes and associated buffers
#[repr(C, packed)]
struct TDesRing<const TD: usize> {
    td: [TDes; TD],
    tbuf: [[u32; ETH_BUF_SIZE / 4]; TD],
    #[cfg(feature = "ptp")]
    tinfo: [PacketInfo; TD],
    tdidx: usize,
}

impl<const TD: usize> TDesRing<TD> {
    const fn new() -> Self {
        Self {
            td: [TDes {
                tdes0: 0,
                tdes1: 0,
                tdes2: 0,
                tdes3: 0,
            }; TD],
            tbuf: [[0; ETH_BUF_SIZE / 4]; TD],
            #[cfg(feature = "ptp")]
            tinfo: [PacketInfo::new(); TD],
            tdidx: 0,
        }
    }

    /// Initialise this TDesRing. Assume TDesRing is corrupt
    ///
    /// The current memory address of the buffers inside this TDesRing
    /// will be stored in the descriptors, so ensure the TDesRing is
    /// not moved after initialisation.
    pub fn init(&mut self) {
        for x in 0..TD {
            self.td[x].init();
        }
        self.tdidx = 0;

        // Initialise pointers in the DMA engine. (There will be a memory barrier later
        // before the DMA engine is enabled.)
        unsafe {
            let dma = &*stm32::ETHERNET_DMA::ptr();
            dma.dmactx_dlar
                .write(|w| w.bits(&self.td[0] as *const _ as u32));
            dma.dmactx_rlr.write(|w| w.tdrl().bits(TD as u16 - 1));
            dma.dmactx_dtpr
                .write(|w| w.bits(&self.td[0] as *const _ as u32));
        }
    }

    /// Return true if a TDes is available for use
    pub fn available(&self) -> bool {
        self.td[self.tdidx].available()
    }

    /// Release the next TDes to the DMA engine for transmission
    pub fn release(&mut self) {
        let x = self.tdidx;
        assert!(self.td[x].tdes3 & EMAC_DES3_OWN == 0); // Owned by us

        let address = ptr::addr_of!(self.tbuf[x]) as u32;

        // Read format
        self.td[x].tdes0 = address; // Buffer 1
        self.td[x].tdes1 = 0; // Not used
        assert!(self.td[x].tdes2 & !EMAC_TDES2_B1L == 0); // Not used
        assert!(self.td[x].tdes2 & EMAC_TDES2_B1L > 0); // Length must be valid
        self.td[x].tdes3 = 0;
        self.td[x].tdes3 |= EMAC_DES3_FD; // FD: Contains first buffer of packet
        self.td[x].tdes3 |= EMAC_DES3_LD; // LD: Contains last buffer of packet
        self.td[x].tdes3 |= EMAC_DES3_OWN; // Give the DMA engine ownership

        // Ensure changes to the descriptor are committed before
        // DMA engine sees tail pointer store
        cortex_m::asm::dsb();

        // Move the tail pointer (TPR) to the next descriptor
        let x = (x + 1) % TD;
        unsafe {
            let dma = &*stm32::ETHERNET_DMA::ptr();
            dma.dmactx_dtpr
                .write(|w| w.bits(&(self.td[x]) as *const _ as u32));
        }

        self.tdidx = x;
    }

    /// Access the buffer pointed to by the next TDes
    pub unsafe fn buf_as_slice_mut(&mut self, length: usize) -> &mut [u8] {
        let x = self.tdidx;

        // Set address in descriptor
        self.td[x].tdes0 = ptr::addr_of!(self.tbuf[x]) as u32; // Buffer 1

        // Set length in descriptor
        let len = core::cmp::min(length, ETH_BUF_SIZE);
        self.td[x].tdes2 = (length as u32) & EMAC_TDES2_B1L;

        // Create a raw pointer in place without an intermediate reference. Use
        // this to return a slice from the packed buffer
        let addr = ptr::addr_of_mut!(self.tbuf[x]) as *mut _;
        core::slice::from_raw_parts_mut(addr, len)
    }

    #[cfg(feature = "ptp")]
    pub fn poll_timestamp(
        &self,
        packet_meta: &smoltcp::phy::PacketMeta,
    ) -> Poll<Result<Option<Timestamp>, PacketMetaNotFound>> {
        for (idx, info) in self.tinfo.into_iter().enumerate() {
            if match info.meta() {
                Some(smoltcp::phy::PacketMeta {id, ..}) => id == packet_meta.id,
                _ => false,
            } {
                if self.td[idx].available() {
                    let timestamp = self.get_timestamp();
                    return Poll::Ready(Ok(timestamp))
                } else {
                    return Poll::Pending
                }
            }
        }
        Poll::Ready(Err(PacketMetaNotFound))
    }

    #[cfg(feature = "ptp")]
    pub fn enable_ptp_with_id(&mut self, packet_meta: Option<smoltcp::phy::PacketMeta>) {
        if packet_meta.is_some() {
            self.td[self.tdidx].tdes2 |= EMAC_TDES2_TTSE;
        }
        self.tinfo[self.tdidx].set_meta_and_clear_ts(packet_meta);
    }

    #[cfg(feature = "ptp")]
    pub fn get_timestamp(&self) -> Option<Timestamp> {
        let contains_timestamp = 
            (self.td[self.tdidx].tdes3 & EMAC_TDES3_TTSS) == EMAC_TDES3_TTSS;
        let owned = (self.td[self.tdidx].tdes3 & EMAC_DES3_OWN) == EMAC_DES3_OWN;
        let is_last = (self.td[self.tdidx].tdes3 & EMAC_DES3_LD) == EMAC_DES3_LD;

        if !owned && contains_timestamp && is_last {
            let (low, high) = (self.td[self.tdidx].tdes0, self.td[self.tdidx].tdes1);
            Some(Timestamp::from_parts(high, low))
        } else {
            None
        }
    }
}

/// Receive Descriptor representation
///
/// * rdes0: recieve buffer address
/// * rdes1:
/// * rdes2:
/// * rdes3: OWN and Status
///
/// Note that Copy and Clone are derived to support initialising an
/// array of RDes, but you may not move a RDes after its address has
/// been given to the ETH_DMA engine.
#[derive(Copy, Clone)]
#[repr(C, packed)]
struct RDes {
    rdes0: u32,
    rdes1: u32,
    rdes2: u32,
    rdes3: u32,
}

impl RDes {
    /// Initialises RDes
    pub fn init(&mut self) {
        self.rdes0 = 0;
        self.rdes1 = 0;
        self.rdes2 = 0;
        self.rdes3 = 0; // Owned by us
    }

    /// Return true if this RDes is acceptable to us
    pub fn valid(&self) -> bool {
        // Write-back descriptor is valid if:
        //
        // Contains first buffer of packet AND contains last buf of
        // packet AND no errors AND not a contex descriptor
        self.rdes3
            & (EMAC_DES3_FD | EMAC_DES3_LD | EMAC_DES3_ES | EMAC_DES3_CTXT)
            == (EMAC_DES3_FD | EMAC_DES3_LD)
    }

    /// Return true if this RDes is not currently owned by the DMA
    pub fn available(&self) -> bool {
        self.rdes3 & EMAC_DES3_OWN == 0 // Owned by us
    }
}

/// Store a ring of RDes and associated buffers
#[repr(C, packed)]
struct RDesRing<const RD: usize> {
    rd: [RDes; RD],
    rbuf: [[u32; ETH_BUF_SIZE / 4]; RD],
    #[cfg(feature = "ptp")]
    rinfo: [PacketInfo; RD],
    rdidx: usize,
}

impl<const RD: usize> RDesRing<RD> {
    const fn new() -> Self {
        Self {
            rd: [RDes {
                rdes0: 0,
                rdes1: 0,
                rdes2: 0,
                rdes3: 0,
            }; RD],
            rbuf: [[0; ETH_BUF_SIZE / 4]; RD],
            #[cfg(feature = "ptp")]
            rinfo: [PacketInfo::new(); RD],
            rdidx: 0,
        }
    }

    /// Initialise this RDesRing. Assume RDesRing is corrupt
    ///
    /// The current memory address of the buffers inside this RDesRing
    /// will be stored in the descriptors, so ensure the RDesRing is
    /// not moved after initialisation.
    pub fn init(&mut self) {
        for x in 0..RD {
            self.rd[x].init();
        }
        self.rdidx = 0;

        // Initialise pointers in the DMA engine
        unsafe {
            let dma = &*stm32::ETHERNET_DMA::ptr();
            dma.dmacrx_dlar
                .write(|w| w.bits(&self.rd[0] as *const _ as u32));
            dma.dmacrx_rlr.write(|w| w.rdrl().bits(RD as u16 - 1));
        }

        // Release descriptors to the DMA engine
        while self.available() {
            self.release()
        }
    }

    /// Return true if a RDes is available for use
    pub fn available(&self) -> bool {
        self.rd[self.rdidx].available()
    }

    /// Return true if current RDes is valid
    pub fn valid(&self) -> bool {
        self.rd[self.rdidx].valid()
    }

    /// Release the next RDes to the DMA engine
    pub fn release(&mut self) {
        let x = self.rdidx;
        assert!(self.rd[x].rdes3 & EMAC_DES3_OWN == 0); // Owned by us

        let address = ptr::addr_of!(self.rbuf[x]) as u32;

        // Read format
        self.rd[x].rdes0 = address; // Buffer 1
        self.rd[x].rdes1 = 0; // Reserved
        self.rd[x].rdes2 = 0; // Marked as invalid
        self.rd[x].rdes3 = 0;
        self.rd[x].rdes3 |= EMAC_DES3_OWN; // Give the DMA engine ownership
        self.rd[x].rdes3 |= EMAC_RDES3_BUF1V; // BUF1V: 1st buffer address is valid
        self.rd[x].rdes3 |= EMAC_RDES3_IOC; // IOC: Interrupt on complete

        // Ensure changes to the descriptor are committed before
        // DMA engine sees tail pointer store
        cortex_m::asm::dsb();

        // Move the tail pointer (TPR) to this descriptor
        unsafe {
            let dma = &*stm32::ETHERNET_DMA::ptr();
            dma.dmacrx_dtpr
                .write(|w| w.bits(&(self.rd[x]) as *const _ as u32));
        }

        // Update active descriptor
        self.rdidx = (x + 1) % RD;
    }

    /// Access the buffer pointed to by the next RDes
    ///
    /// # Safety
    ///
    /// Ensure that release() is called between subsequent calls to this
    /// function.
    #[allow(clippy::mut_from_ref)]
    pub unsafe fn buf_as_slice_mut(&self) -> &mut [u8] {
        let x = self.rdidx;

        // Write-back format
        let addr = ptr::addr_of!(self.rbuf[x]) as *mut u8;
        let len = (self.rd[x].rdes3 & EMAC_RDES3_PL) as usize;

        let len = core::cmp::min(len, ETH_BUF_SIZE);
        core::slice::from_raw_parts_mut(addr, len)
    }

    #[cfg(feature = "ptp")]
    pub fn timestamp(&self, packet_meta: &smoltcp::phy::PacketMeta) -> Result<Option<Timestamp>, PacketMetaNotFound> {
        for info in self.rinfo {
            if match info.meta() {
                Some(smoltcp::phy::PacketMeta {id, ..}) => id == packet_meta.id,
                _ => false,
            } {
                return Ok(info.ts())
            }
        }
        Err(PacketMetaNotFound)
    }

    #[cfg(feature = "ptp")]
    pub fn set_meta_and_clear_ts(&mut self, packet_meta: Option<smoltcp::phy::PacketMeta>) {
        self.rinfo[self.rdidx].set_meta_and_clear_ts(packet_meta)
    }

    #[cfg(feature = "ptp")]
    pub fn was_timestamped(&self) -> bool {
        ((self.rd[self.rdidx + 1].rdes3 & EMAC_DES3_CTXT) == EMAC_DES3_CTXT) // next one is context!
            && ((self.rd[self.rdidx].rdes1 & EMAC_RDES1_TSA) == EMAC_RDES1_TSA) // 
            && (self.rd[self.rdidx].rdes3 & EMAC_DES3_LD) == EMAC_DES3_LD // is last
    }

    #[cfg(feature = "ptp")]
    pub fn read_timestamp_from_next(&self) -> Option<Timestamp> {
        if !((self.rd[self.rdidx + 1].rdes3 & EMAC_DES3_OWN) == EMAC_DES3_OWN) { // if not is owned
            let (high, low) = (self.rd[self.rdidx + 1].rdes1, self.rd[self.rdidx+1].rdes0);
            Some(Timestamp::from_parts(high, low))
        } else {
            None
        }
    }

    #[cfg(feature = "ptp")]
    pub fn attach_timestamp(&mut self, timestamp: Option<Timestamp>) {
        self.rinfo[self.rdidx].set_ts(timestamp);
    }

    #[cfg(feature = "ptp")]
    pub fn release_timestamp_desc(&mut self) {
        self.rdidx += 1;
        self.release();
        self.rdidx -= 2;
    }
}

pub struct DesRing<const TD: usize, const RD: usize> {
    tx: TDesRing<TD>,
    rx: RDesRing<RD>,
}
impl<const TD: usize, const RD: usize> DesRing<TD, RD> {
    pub const fn new() -> Self {
        DesRing {
            tx: TDesRing::new(),
            rx: RDesRing::new(),
        }
    }
}
impl<const TD: usize, const RD: usize> Default for DesRing<TD, RD> {
    fn default() -> Self {
        Self::new()
    }
}

///
/// Ethernet DMA
///
pub struct EthernetDMA<const TD: usize, const RD: usize> {
    ring: &'static mut DesRing<TD, RD>,
    eth_dma: stm32::ETHERNET_DMA,

    #[cfg(feature = "ptp")]
    packet_meta_counter: u32,
}

#[cfg(feature = "ptp")]
impl <const TD: usize, const RD: usize> EthernetDMA<TD, RD> {
    pub fn next_packet_meta(&mut self) -> smoltcp::phy::PacketMeta {
        let mut meta = smoltcp::phy::PacketMeta::default();
        meta.id = self.packet_meta_counter;
        self.packet_meta_counter += 1;
        meta
    }
    
    pub fn rx_timestamp(
        &self,
        packet_meta: &smoltcp::phy::PacketMeta,
    ) -> Result<Option<Timestamp>, PacketMetaNotFound> {
        self.ring.rx.timestamp(packet_meta)
    }

    pub fn poll_tx_timestamp(
        &self,
        packet_meta: &smoltcp::phy::PacketMeta,
    ) -> Poll<Result<Option<Timestamp>, PacketMetaNotFound>> {
        self.ring.tx.poll_timestamp(packet_meta)
    }

}

///
/// Ethernet MAC
///
pub struct EthernetMAC {
    eth_mac: stm32::ETHERNET_MAC,
    eth_phy_addr: u8,
    clock_range: u8,
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
pub fn new<const TD: usize, const RD: usize>(
    eth_mac: stm32::ETHERNET_MAC,
    eth_mtl: stm32::ETHERNET_MTL,
    eth_dma: stm32::ETHERNET_DMA,
    mut pins: impl PinsRMII,
    ring: &'static mut DesRing<TD, RD>,
    mac_addr: EthernetAddress,
    prec: rec::Eth1Mac,
    clocks: &CoreClocks,
) -> (EthernetDMA<TD, RD>, EthernetMAC) {
    pins.set_speed(Speed::VeryHigh);
    unsafe {
        new_unchecked(eth_mac, eth_mtl, eth_dma, ring, mac_addr, prec, clocks)
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
pub unsafe fn new_unchecked<const TD: usize, const RD: usize>(
    eth_mac: stm32::ETHERNET_MAC,
    eth_mtl: stm32::ETHERNET_MTL,
    eth_dma: stm32::ETHERNET_DMA,
    ring: &'static mut DesRing<TD, RD>,
    mac_addr: EthernetAddress,
    prec: rec::Eth1Mac,
    clocks: &CoreClocks,
) -> (EthernetDMA<TD, RD>, EthernetMAC) {
    // RCC
    {
        let rcc = &*stm32::RCC::ptr();
        let syscfg = &*stm32::SYSCFG::ptr();

        // Ensure syscfg is enabled (for PMCR)
        rcc.apb4enr.modify(|_, w| w.syscfgen().set_bit());

        // Reset ETH_DMA - write 1 and wait for 0.
        // On the H723, we have to do this before prec.enable()
        // or the DMA will never come out of reset
        eth_dma.dmamr.modify(|_, w| w.swr().set_bit());
        while eth_dma.dmamr.read().swr().bit_is_set() {}

        // AHB1 ETH1MACEN
        prec.enable();

        // Also need to enable the transmission and reception clocks, which
        // don't have prec objects. They don't have prec objects because they
        // can't be reset.
        rcc.ahb1enr
            .modify(|_, w| w.eth1txen().set_bit().eth1rxen().set_bit());

        syscfg.pmcr.modify(|_, w| w.epis().bits(0b100)); // RMII
    }

    // reset ETH_MAC - write 1 then 0
    //rcc.ahb1rstr.modify(|_, w| w.eth1macrst().set_bit());
    //rcc.ahb1rstr.modify(|_, w| w.eth1macrst().clear_bit());

    cortex_m::interrupt::free(|_cs| {
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

        #[cfg(feature = "ptp")]
        let pm_value = true;
        #[cfg(not(feature = "ptp"))]
        let pm_value = false;

        // frame filter register
        eth_mac.macpfr.modify(|_, w| {
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
                .bit(pm_value)
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
            w.intm()
                .bits(0b00)
                // Rx Tx priority ratio 1:1
                .pr()
                .bits(0b000)
                .txpr()
                .clear_bit()
                .da()
                .clear_bit()
        });
        // bus mode register
        eth_dma.dmasbmr.modify(|_, w| {
            // Address-aligned beats
            w.aal()
                .set_bit()
                // Fixed burst
                .fb()
                .set_bit()
        });
        eth_dma
            .dmaccr
            .modify(|_, w| w.dsl().bits(0).pblx8().clear_bit().mss().bits(536));
        eth_dma.dmactx_cr.modify(|_, w| {
            w
                // Tx DMA PBL
                .txpbl()
                .bits(32)
                .tse()
                .clear_bit()
                // Operate on second frame
                .osf()
                .clear_bit()
        });

        eth_dma.dmacrx_cr.modify(|_, w| {
            w
                // receive buffer size
                .rbsz()
                .bits(ETH_BUF_SIZE as u16)
                // Rx DMA PBL
                .rxpbl()
                .bits(32)
                // Disable flushing of received frames
                .rpf()
                .clear_bit()
        });

        // Initialise DMA descriptors
        ring.tx.init();
        ring.rx.init();

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

        // Manage DMA transmission and reception
        eth_dma.dmactx_cr.modify(|_, w| w.st().set_bit());
        eth_dma.dmacrx_cr.modify(|_, w| w.sr().set_bit());

        eth_dma
            .dmacsr
            .modify(|_, w| w.tps().set_bit().rps().set_bit());
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

    let dma = EthernetDMA { 
        ring, 
        eth_dma,
        #[cfg(feature = "ptp")]
        packet_meta_counter: 0,
    };

    (dma, mac)
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

/// Define TxToken type and implement consume method
pub struct TxToken<'a, const TD: usize> {
    des_ring: &'a mut TDesRing<TD>,
    #[cfg(feature = "ptp")]
    packet_meta: Option<smoltcp::phy::PacketMeta>,
}

impl<'a, const TD: usize> phy::TxToken for TxToken<'a, TD> {
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        assert!(len <= ETH_BUF_SIZE);

        let result = f(unsafe { self.des_ring.buf_as_slice_mut(len) });
        #[cfg(feature = "ptp")]
        self.des_ring.enable_ptp_with_id(self.packet_meta);
        self.des_ring.release();
        result
    }

    #[cfg(feature = "ptp")]
    fn set_meta(&mut self, packet_meta: smoltcp::phy::PacketMeta) {
        self.packet_meta = Some(packet_meta)
    }
}

/// Define RxToken type and implement consume method
pub struct RxToken<'a, const RD: usize> {
    des_ring: &'a mut RDesRing<RD>,
    #[cfg(feature = "ptp")]
    packet_meta: smoltcp::phy::PacketMeta,
}

impl<'a, const RD: usize> phy::RxToken for RxToken<'a, RD> {
    fn consume<R, F>(self, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        #[cfg(feature = "ptp")]
        {
            self.des_ring.set_meta_and_clear_ts(Some(self.packet_meta));
            if self.des_ring.was_timestamped() {
                let timestamp = self.des_ring.read_timestamp_from_next();
                self.des_ring.attach_timestamp(timestamp);
                self.des_ring.release_timestamp_desc();
            }
        }
        let result = f(unsafe { self.des_ring.buf_as_slice_mut() });
        self.des_ring.release();
        result
    }

    #[cfg(feature = "ptp")]
    fn meta(&self) -> smoltcp::phy::PacketMeta {
        self.packet_meta 
    }
}

/// Implement the smoltcp Device interface
impl<const TD: usize, const RD: usize> phy::Device for EthernetDMA<TD, RD> {
    type RxToken<'a> = RxToken<'a, RD>;
    type TxToken<'a> = TxToken<'a, TD>;

    // Clippy false positive because DeviceCapabilities is non-exhaustive
    #[allow(clippy::field_reassign_with_default)]
    fn capabilities(&self) -> DeviceCapabilities {
        let mut caps = DeviceCapabilities::default();
        // ethernet frame type II (6 smac, 6 dmac, 2 ethertype),
        // sans CRC (4), 1500 IP MTU
        caps.max_transmission_unit = 1514;
        caps.max_burst_size = Some(core::cmp::min(TD, RD));
        caps
    }

    fn receive(
        &mut self,
        _timestamp: Instant,
    ) -> Option<(RxToken<RD>, TxToken<TD>)> {
        // Skip all queued packets with errors.
        while self.ring.rx.available() && !self.ring.rx.valid() {
            self.ring.rx.release()
        }

        if self.ring.rx.available() && self.ring.tx.available() {
            #[cfg(feature = "ptp")]
            let rx_packet_meta = self.next_packet_meta();
            Some((
                RxToken {
                    des_ring: &mut self.ring.rx, 
                    #[cfg(feature = "ptp")]
                    packet_meta: rx_packet_meta,
                }, 
                TxToken {
                    des_ring: &mut self.ring.tx, 
                    #[cfg(feature = "ptp")]
                    packet_meta: None
                }
            ))
        } else {
            None
        }
    }

    fn transmit(&mut self, _timestamp: Instant) -> Option<TxToken<TD>> {
        if self.ring.tx.available() {
            #[cfg(feature = "ptp")]
            let tx_packet_meta = Some(self.next_packet_meta());
            Some(
                TxToken {
                    des_ring: &mut self.ring.tx,
                    #[cfg(feature = "ptp")]
                    packet_meta: tx_packet_meta,
                }
            )
        } else {
            None
        }
    }
}

impl<const TD: usize, const RD: usize> EthernetDMA<TD, RD> {
    /// Return the number of packets dropped since this method was
    /// last called
    pub fn number_packets_dropped(&self) -> u32 {
        self.eth_dma.dmacmfcr.read().mfc().bits() as u32
    }
}

/// Clears the Ethernet interrupt flag
///
/// # Safety
///
/// This method implements a single register write to DMACSR
pub unsafe fn interrupt_handler() {
    let eth_dma = &*stm32::ETHERNET_DMA::ptr();
    eth_dma
        .dmacsr
        .write(|w| w.nis().set_bit().ri().set_bit().ti().set_bit());
    let _ = eth_dma.dmacsr.read();
    let _ = eth_dma.dmacsr.read(); // Delay 2 peripheral clocks
}

/// Enables the Ethernet Interrupt. The following interrupts are enabled:
///
/// * Normal Interrupt `NIE`
/// * Receive Interrupt `RIE`
/// * Transmit Interript `TIE`
///
/// # Safety
///
/// This method implements a single RMW to DMACIER
pub unsafe fn enable_interrupt() {
    let eth_dma = &*stm32::ETHERNET_DMA::ptr();
    eth_dma
        .dmacier
        .modify(|_, w| w.nie().set_bit().rie().set_bit().tie().set_bit());
}
