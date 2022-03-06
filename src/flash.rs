//! Flash memory
//!
//! # Examples
//!
//! - [Flash example](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/flash.rs)
//!
//! # Supported modes
//!
//!     - Standard flash operations are supported only
//!
//! # Supported devices
//!
//!     - STM32H742xI/743xI/753xI
//!     - STM32H742xG/743xG
//!
//! Note: STM32H750xB is not yet supported

use crate::stm32::{flash, FLASH};

#[cfg(all(
    any(feature = "rm0433"),
    not(feature = "stm32h750"),
    not(feature = "stm32h750v")
))]
use crate::stm32::flash::BANK;
#[cfg(all(
    any(feature = "rm0433"),
    not(feature = "stm32h750"),
    not(feature = "stm32h750v")
))]
use core;

/// Extension trait to constrain the FLASH peripheral
pub trait FlashExt {
    /// Constrains the FLASH peripheral to play nicely with the other abstractions
    fn constrain(self) -> Flash;
}

impl FlashExt for FLASH {
    fn constrain(self) -> Flash {
        Flash {
            #[cfg(all(
                any(feature = "rm0433"),
                not(feature = "stm32h750"),
                not(feature = "stm32h750v")
            ))]
            regs: self,
            acr: ACR { _0: () },
        }
    }
}

/// Constrained FLASH peripheral
/// Currently only standard flash operations are supported, secure operations are not yet supported
/// Supported devices:
///     - STM32H742xI/743xI/753xI
///     - STM32H742xG/743xG
/// Note: STM32H750xB is not yet supported
pub struct Flash {
    /// Opaque ACR register
    pub acr: ACR,
    /// Flash peripheral
    #[cfg(all(
        any(feature = "rm0433"),
        not(feature = "stm32h750"),
        not(feature = "stm32h750v")
    ))]
    regs: FLASH,
}

/// Opaque ACR register
pub struct ACR {
    _0: (),
}

impl ACR {
    #[allow(unused)]
    pub(crate) fn acr(&mut self) -> &flash::ACR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*FLASH::ptr()).acr }
    }
}

#[cfg(all(
    any(feature = "rm0433"),
    not(feature = "stm32h750"),
    not(feature = "stm32h750v")
))]
/// BANK1 start address
const BANK1_ADDR_START: usize = 0x0800_0000;
#[cfg(all(
    any(feature = "rm0433"),
    not(feature = "stm32h750"),
    not(feature = "stm32h750v")
))]
/// BANK1 end address
const BANK1_ADDR_END: usize = 0x080F_FFFF;
#[cfg(all(
    any(feature = "rm0433"),
    not(feature = "stm32h750"),
    not(feature = "stm32h750v")
))]
/// BANK2 start address
const BANK2_ADDR_START: usize = 0x0810_0000;
#[cfg(all(
    any(feature = "rm0433"),
    not(feature = "stm32h750"),
    not(feature = "stm32h750v")
))]
/// BANK2 end address
const BANK2_ADDR_END: usize = 0x081F_FFFF;
#[cfg(all(
    any(feature = "rm0433"),
    not(feature = "stm32h750"),
    not(feature = "stm32h750v")
))]
/// Sector size of 128kb
const SECTOR_SIZE: usize = 0x2_0000;

#[cfg(all(
    any(feature = "rm0433"),
    not(feature = "stm32h750"),
    not(feature = "stm32h750v")
))]
#[derive(Clone, Copy, PartialEq)]
pub enum Bank {
    /// User Bank 1
    UserBank1,
    /// User Bank 2
    UserBank2,
}

#[cfg(all(
    any(feature = "rm0433"),
    not(feature = "stm32h750"),
    not(feature = "stm32h750v")
))]
impl Bank {
    /// The start address of the bank
    fn start_address(&self) -> usize {
        match *self {
            Bank::UserBank1 => BANK1_ADDR_START,
            Bank::UserBank2 => BANK2_ADDR_START,
        }
    }
    /// The end address of the bank
    fn end_address(&self) -> usize {
        match *self {
            Bank::UserBank1 => BANK1_ADDR_END,
            Bank::UserBank2 => BANK2_ADDR_END,
        }
    }
    /// The address of a given sector of a bank
    fn sector_address(&self, sector: usize) -> usize {
        if sector > self.num_sectors() {
            panic!("Sector outside of bank range");
        }
        self.start_address() + (sector * SECTOR_SIZE)
    }
    /// The number of bytes within the bank
    pub fn size(&self) -> usize {
        match *self {
            Bank::UserBank1 => (BANK1_ADDR_END - BANK1_ADDR_START),
            Bank::UserBank2 => (BANK2_ADDR_END - BANK2_ADDR_START),
        }
    }
    /// The number of sectors within the bank, including sector zero
    pub fn num_sectors(&self) -> usize {
        self.size() / SECTOR_SIZE
    }
    /// Sector size
    pub fn sector_size(&self) -> usize {
        match *self {
            Bank::UserBank1 => SECTOR_SIZE,
            Bank::UserBank2 => SECTOR_SIZE,
        }
    }
}

#[cfg(all(
    any(feature = "rm0433"),
    not(feature = "stm32h750"),
    not(feature = "stm32h750v")
))]
#[derive(Copy, Clone, Debug)]
/// Possible error states for flash operations.
pub enum Error {
    /// Error detected (by command execution, or because no command could be executed)
    Illegal,
    /// (Legal) command failed
    Unlock,
    /// Operation out of bounds
    OutOfBounds,
}

#[cfg(all(
    any(feature = "rm0433"),
    not(feature = "stm32h750"),
    not(feature = "stm32h750v")
))]
impl Flash {
    /// Get underlying bank registers
    fn bank_registers(&self, bank: Bank) -> &BANK {
        match bank {
            Bank::UserBank1 => self.regs.bank1(),
            Bank::UserBank2 => self.regs.bank2(),
        }
    }

    /// Unlock the flash memory, must only be executed on locked flash
    fn unlock(&self, bank: Bank) -> Result<(), Error> {
        const FLASH_KEY1: u32 = 0x4567_0123;
        const FLASH_KEY2: u32 = 0xCDEF_89AB;

        let regs = self.bank_registers(bank);
        regs.keyr.write(|w| unsafe { w.bits(FLASH_KEY1) });
        regs.keyr.write(|w| unsafe { w.bits(FLASH_KEY2) });

        if regs.cr.read().lock().bit_is_clear() {
            Ok(())
        } else {
            Err(Error::Unlock)
        }
    }

    /// Lock the flash memory, must be executed after an unlock
    fn lock(&mut self, bank: Bank) {
        let regs = self.bank_registers(bank);
        regs.cr.modify(|_, w| w.lock().set_bit());
    }

    /// Read flash memory from a given bank, sector and offset by the length of buff / 4
    ///
    /// # Panics
    ///
    /// Panics if the sector is beyond the number of sectors supported by the user bank for this device
    /// Panics if the buff length is not a multiple of a 32bit word
    /// Panics if the start offset is not a multiple of a 32bit word
    pub fn read_sector(
        &mut self,
        bank: Bank,
        sector: usize,
        start: usize,
        buff: &mut [u8],
    ) -> Result<(), Error> {
        if sector > bank.num_sectors() {
            panic!("Sector out of bounds")
        }
        if buff.len() % 4 != 0 || start % 4 != 0 {
            panic!("buff length and start offset must be a multiple of a 32bit word");
        }

        let regs = self.bank_registers(bank);

        // Ensure no effective write, erase or option byte change operation is ongoing
        while regs.sr.read().bsy().bit_is_set() {}

        // H742 RM, section 4.3.8, Page 158
        // Single read sequence
        // The recommended simple read sequence is the following:
        // 1. Freely perform read accesses to any AXI-mapped area.
        // 2. The embedded Flash memory effectively executes the read operation from the read
        // command queue buffer as soon as the non-volatile memory is ready and the previously
        // requested operations on this specific bank have been served.

        // Get the address of the sector/bank
        let mut addr = bank.sector_address(sector) as *mut u32;

        unsafe {
            // Offset it by the start position
            addr = addr.add(start / 4);
            // Iterate on chunks of 32bits
            for chunk in buff.chunks_exact_mut(4) {
                if addr as usize > bank.end_address() {
                    return Err(Error::OutOfBounds);
                }
                // Read words from the flash
                let word = core::ptr::read_volatile(addr);
                chunk[0..4].copy_from_slice(&word.to_le_bytes());
                addr = addr.add(1);
            }
        }

        Ok(())
    }

    /// Erase the sector of a given bank
    ///
    /// # Panics
    ///
    /// Panics if the sector is beyond the number of sectors supported by the user bank for this device
    pub fn erase_sector(
        &mut self,
        bank: Bank,
        sector: usize,
    ) -> Result<(), Error> {
        if sector > bank.num_sectors() {
            panic!("Sector out of bounds")
        }

        // To erase a 128-Kbyte user sector, proceed as follows:
        // 1. Check and clear (optional) all the error flags due to previous programming/erase
        // operation. Refer to Section 4.7: FLASH error management for details.
        // 2. Unlock the FLASH_CR1/2 register, as described in Section 4.5.1: FLASH configuration
        // protection (only if register is not already unlocked).
        // 3. Set the SER1/2 bit and SNB1/2 bitfield in the corresponding FLASH_CR1/2 register. SER1/2 indicates a sector erase operation, while SNB1/2 contains the target sector number.
        // 4. Set the START1/2 bit in the FLASH_CR1/2 register.
        // 5. Wait for the QW1/2 bit to be cleared in the corresponding FLASH_SR1/2 register.
        // If a bank erase is requested simultaneously to the sector erase (BER1/2 bit set), the bank erase operation supersedes the sector erase operation.

        let regs = self.bank_registers(bank);

        // Ensure no effective write, erase or option byte change operation is ongoing
        while regs.sr.read().bsy().bit_is_set() {}

        // 1. Check and clear (optional) all the error flags due to previous programming/erase
        // operation. Refer to Section 4.7: FLASH error management for details.
        handle_illegal(regs)?;

        // 2. Unlock the FLASH_CR1/2 register, as described in Section 4.5.1: FLASH configuration
        self.unlock(bank)?;

        // 3. Set the SER1/2 bit and SNB1/2 bitfield in the corresponding FLASH_CR1/2 register.
        // SER1/2 indicates a sector erase operation, while SNB1/2 contains the target sector number.
        regs.cr.modify(|_, w| unsafe {
            w.ser().set_bit();
            w.snb().bits(sector as u8)
        });

        // 4. Set the START1/2 bit in the FLASH_CR1/2 register.
        regs.cr.modify(|_, w| w.start().set_bit());

        // 5. Wait for the QW1/2 bit to be cleared in the corresponding FLASH_SR1/2 register.
        while regs.sr.read().bsy().bit_is_set() {}

        self.lock(bank);

        Ok(())
    }

    /// Standard Flash bank erase
    pub fn erase_bank(&mut self, bank: Bank) -> Result<(), Error> {
        // To erase all bank sectors except for those containing secure-only and protected data, proceed as follows:
        // 1. Check and clear (optional) all the error flags due to previous programming/erase operation. Refer to Section 4.7: FLASH error management for details.
        // 2. Unlock the FLASH_CR1/2 register, as described in Section 4.5.1: FLASH configuration protection (only if register is not already unlocked).
        // 3. Set the BER1/2 bit in the FLASH_CR1/2 register corresponding to the targeted bank.
        // 4. Set the START1/2 bit in the FLASH_CR1/2 register to start the bank erase operation. Then wait until the QW1/2 bit is cleared in the corresponding FLASH_SR1/2 register.
        // BER1/2 and START1/2 bits can be set together, so above steps 3 and 4 can be merged.
        // If a sector erase is requested simultaneously to the bank erase (SER1/2 bit set), the bank erase operation supersedes the sector erase operation.

        let regs = self.bank_registers(bank);

        // Check that no Flash main memory operation is ongoing by checking the BSY bit
        while regs.sr.read().bsy().bit_is_set() {}

        // 1. Check and clear (optional) all the error flags due to previous programming/erase operation. Refer to Section 4.7: FLASH error management for details.
        handle_illegal(regs)?;

        // 2. Unlock the FLASH_CR1/2 register, as described in Section 4.5.1: FLASH configuration protection (only if register is not already unlocked).
        self.unlock(bank)?;

        // 3. Set the BER1/2 bit in the FLASH_CR1/2 register corresponding to the targeted bank.
        regs.cr.modify(|_, w| w.ber().set_bit());

        // 4. Set the START1/2 bit in the FLASH_CR1/2 register to start the bank erase operation.
        // Then wait until the QW1/2 bit is cleared in the corresponding FLASH_SR1/2 register.
        regs.cr.modify(|_, w| w.start().set_bit());
        while regs.sr.read().qw().bit_is_set() {}

        // 5. Wait for the BSY bit to be cleared in the FLASH_SR register.
        while regs.sr.read().bsy().bit_is_set() {}

        self.lock(bank);

        Ok(())
    }

    /// Write data beginning at the bank, sector and start offset
    /// Writing must occur after an erase of the flash data
    ///
    /// # Panics
    ///
    /// Panics if the sector is beyond the number of sectors supported by the user bank for this device
    /// Panics if the data length is not a multiple of a 256bit flash word
    /// Panics if the start offset is not a multiple of a 256bit flash word
    pub fn write_sector(
        &mut self,
        bank: Bank,
        sector: usize,
        start: usize,
        data: &[u8],
    ) -> Result<(), Error> {
        if sector > bank.num_sectors() {
            panic!("Sector out of bounds")
        }
        if data.len() % 256 != 0 || start % 256 != 0 {
            panic!(
                "data length and start offset must be a multiple of a 32 byte (256b) 'flash word'"
            );
        }

        // The recommended single write sequence in bank 1/2 is the following:
        // 1. Unlock the FLASH_CR1/2 register, as described in Section 4.5.1: FLASH configuration protection (only if register is not already unlocked).
        // 2. Enable write operations by setting the PG1/2 bit in the FLASH_CR1/2 register.
        // 3. Check the protection of the targeted memory area.
        // 4. Write one Flash-word corresponding to 32-byte data starting at a 32-byte aligned address.
        // 5. Check that QW1 (respectively QW2) has been raised and wait until it is reset to 0.
        // If step 4 is executed incrementally (e.g. byte per byte), the write buffer can become partially filled. In this case the application software can decide to force-write what is stored in the write buffer by using FW1/2 bit in FLASH_CR1/2 register. In this particular case, the unwritten bits are automatically set to 1. If no bit in the write buffer is cleared to 0, the FW1/2 bit has no effect.

        let regs = self.bank_registers(bank);

        // Ensure no effective write, erase or option byte change operation is ongoing
        while regs.sr.read().bsy().bit_is_set() {}

        // Check and clear (optional) all the error flags due to previous programming/erase operation. Refer to Section 4.7: FLASH error management for details.
        handle_illegal(regs)?;

        // 1. Unlock the FLASH_CR1/2 register, as described in Section 4.5.1: FLASH configuration protection (only if register is not already unlocked).
        self.unlock(bank)?;

        // 2. Enable write operations by setting the PG1/2 bit in the FLASH_CR1/2 register.
        regs.cr.modify(|_, w| w.pg().set_bit().fw().set_bit());

        // 3. Check the protection of the targeted memory area.
        // SKIP: In standard mode the entire UserBank1 and UserBank2 should have no write protections

        // 4. Write one Flash-word corresponding to 32-byte data starting at a 32-byte aligned address.
        let mut addr = bank.sector_address(sector) as *mut u32;

        // Offset it by the start position
        addr = unsafe { addr.add(start / 4) };

        // Iterate on chunks of 32bit
        for chunk in data.chunks_exact(4) {
            if addr as usize > bank.end_address() {
                // Unable to write outside of the bank end address
                // Cleanup and return out
                regs.cr.modify(|_, w| w.pg().clear_bit());
                self.lock(bank);
                return Err(Error::OutOfBounds);
            }

            unsafe {
                let word = u32::from_le_bytes(chunk.try_into().unwrap());
                core::ptr::write_volatile(addr, word);
                addr = addr.add(1);
            }

            // 5. Check that QW1 (respectively QW2) has been raised and wait until it is reset to 0.
            while regs.sr.read().qw().bit_is_set() {}
            // Additionally wait for the busy flag to clear
            while regs.sr.read().bsy().bit_is_set() {}

            // 6. Check that EOP flag is set in the FLASH_SR register (meaning that the programming
            // operation has succeed), and clear it by software.
            if regs.sr.read().eop().bit_is_set() {
                regs.sr.modify(|_, w| w.eop().set_bit()); // Clear
            }
        }

        // 7. Cleanup by clearing the PG bit
        regs.cr.modify(|_, w| w.pg().clear_bit());

        self.lock(bank);

        Ok(())
    }
}

#[cfg(all(
    any(feature = "rm0433"),
    not(feature = "stm32h750"),
    not(feature = "stm32h750v")
))]
/// Handle illegal status flags
/// TODO: Clear error flags
fn handle_illegal(regs: &BANK) -> Result<(), Error> {
    let sr = regs.sr.read();
    if sr.pgserr().bit_is_set()
        || sr.wrperr().bit_is_set()
        || sr.dbeccerr().bit_is_set()
        || sr.strberr().bit_is_set()
        || sr.operr().bit_is_set()
        || sr.rdperr().bit_is_set()
        || sr.rdserr().bit_is_set()
    {
        return Err(Error::Illegal);
    }
    Ok(())
}
