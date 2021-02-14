//! Micrel KSZ8081R Ethernet PHY

use crate::ethernet::{StationManagement, PHY};

/// Micrel KSZ8081R Ethernet PHY
pub struct KSZ8081R<MAC: StationManagement> {
    mac: MAC,
}

impl<MAC: StationManagement> PHY for KSZ8081R<MAC> {
    /// Reset PHY and wait for it to come out of reset.
    fn phy_reset(&mut self) {
        self.mac.smi_write(0x00, 1 << 15);
        while self.mac.smi_read(0x00) & (1 << 15) == (1 << 15) {}
    }

    /// PHY initialisation.
    fn phy_init(&mut self) {
        // Enable auto-negotiation
        self.mac.smi_write(0x00, 1 << 12);
    }
}

/// Public functions for the KSZ8081R
impl<MAC: StationManagement> KSZ8081R<MAC> {
    pub fn new(mac: MAC) -> Self {
        KSZ8081R { mac }
    }

    /// Poll PHY to determine link status.
    pub fn poll_link(&mut self) -> bool {
        let bsr = self.mac.smi_read(0x01);
        let bcr = self.mac.smi_read(0x00);
        let lpa = self.mac.smi_read(0x05);

        // No link without autonegotiate
        if bcr & (1 << 12) == 0 {
            return false;
        }
        // No link if link is down
        if bsr & (1 << 2) == 0 {
            return false;
        }
        // No link if remote fault
        if bsr & (1 << 4) != 0 {
            return false;
        }
        // No link if autonegotiate incomplete
        if bsr & (1 << 5) == 0 {
            return false;
        }
        // No link if other side can't do 100Mbps full duplex
        if lpa & (1 << 8) == 0 {
            return false;
        }
        // Got link
        true
    }

    pub fn link_established(&mut self) -> bool {
        self.poll_link()
    }

    pub fn block_until_link(&mut self) {
        while !self.link_established() {}
    }

    /// Enable the Link Up and Link Down interrupts on INTRP
    pub fn interrupt_enable(&mut self) {
        self.mac.smi_write(0x1b, 0xA << 8);
    }
}
