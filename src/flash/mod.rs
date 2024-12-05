//! Embedded Flash memory
//!
//! Support for Read, Write and Erase options on the Embedded Flash memory. This
//! module implements traits from the
//! [embedded-storage](https://github.com/rust-embedded-community/embedded-storage)
//! crate.
//!
//! # Examples
//!
//! - [Flash example](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/flash.rs)
//!
//! # Supported devices
//!
//! | Reference Manual | Flash Sizes | Banks | Sector Size
//! | --- | --- | --- | ---
//! | RM0433 | 128kB, 1MB, 2MB | One or Two | 128kB
//! | RM0399 | 1MB, 2MB | Two | 128kB
//! | RM0455 | 128kB, 1MB, 2MB | One or Two | 8kB
//! | RM0468 | 128kB, 512kB, 1MB | One | 128kB

use core::slice;

use crate::signature::FlashSize;
use crate::stm32::{flash::BANK, FLASH};
use embedded_storage::nor_flash;

mod operations;
pub use operations::{Error, UnlockedFlashBank};

// All sectors in the user main memory sectors have the same size. Only the
// RM0455 sub-family has 8kB sectors, the other families have 128kB
#[cfg(not(feature = "rm0455"))]
const SECTOR_SIZE: usize = 0x2_0000; // 128kB
#[cfg(feature = "rm0455")]
const SECTOR_SIZE: usize = 0x2000; // 8kB

// The write granularity is either 128 bits or 256 bits depending on the
// family. See RM0433 Rev 7 Section 4.3.9
#[cfg(not(feature = "rm0455"))]
const WRITE_SIZE: usize = 32; // 256-bit
#[cfg(feature = "rm0455")]
const WRITE_SIZE: usize = 16; // 128-bit

/// Flash memory sector
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FlashSector {
    /// Sector number
    pub number: u8,
    /// Offset from base memory address
    pub offset: usize,
}
impl FlashSector {
    /// Returns true if given offset belongs to this sector
    pub fn contains(&self, offset: usize) -> bool {
        self.offset <= offset && offset < self.offset + SECTOR_SIZE
    }
}

/// Iterator of flash memory sectors within a single bank
pub struct FlashSectorIterator {
    index: u8,
    start_sector: u8,
    start_offset: usize,
    end_offset: usize,
}
impl FlashSectorIterator {
    fn new(start_sector: u8, start_offset: usize, end_offset: usize) -> Self {
        Self {
            index: 0,
            start_sector,
            start_offset,
            end_offset,
        }
    }
}
impl Iterator for FlashSectorIterator {
    type Item = FlashSector;

    fn next(&mut self) -> Option<Self::Item> {
        if self.start_offset >= self.end_offset {
            None
        } else {
            let sector = FlashSector {
                number: self.start_sector + self.index,
                offset: self.start_offset,
            };

            self.index += 1;
            self.start_offset += SECTOR_SIZE;

            Some(sector)
        }
    }
}

/// Returns iterator of flash memory sectors for single and dual bank flash
///
/// Sectors are returned in memory order, potentially with jumps in between banks
pub fn flash_sectors(
    flash_size: usize,
    dual_bank: bool,
) -> impl Iterator<Item = FlashSector> {
    if dual_bank {
        // Second user main memory bank always starts at an offset of 0x10_0000
        FlashSectorIterator::new(0, 0, flash_size / 2).chain(
            FlashSectorIterator::new(
                0,
                0x10_0000,
                0x10_0000 + (flash_size / 2),
            ),
        )
    } else {
        // Chain an empty iterator to match types
        FlashSectorIterator::new(0, 0, flash_size)
            .chain(FlashSectorIterator::new(0, 0, 0))
    }
}

/// Flash methods implemented for `pac::FLASH`
#[allow(clippy::len_without_is_empty)]
pub trait FlashExt {
    /// Memory-mapped address
    fn address(&self) -> usize;
    /// Size in bytes
    fn len(&self) -> usize;
    /// Split the flash into its banks.
    fn split(self) -> (LockedFlashBank, Option<LockedFlashBank>);
    /// Access to memory banks.
    fn access_banks<T>(
        &mut self,
        f: impl FnOnce(&mut LockedFlashBank, Option<&mut LockedFlashBank>) -> T,
    ) -> T;
    /// Access to memory bank 1.
    fn access_bank1<F, T>(&mut self, f: F) -> T
    where
        F: FnOnce(&mut LockedFlashBank) -> T;
    /// Returns true if flash is dual bank
    fn dual_bank(&self) -> bool;
    /// Returns flash memory sector of a given offset. Returns none if offset is out of range
    fn sector(&self, offset: usize) -> Option<FlashSector>;
}

impl FlashExt for FLASH {
    fn access_bank1<F, T>(&mut self, f: F) -> T
    where
        F: FnOnce(&mut LockedFlashBank) -> T,
    {
        let len = self.len();
        let mut bank = LockedFlashBank {
            bank_index: 1,
            base: 0x0800_0000,
            len: if self.dual_bank() { len / 2 } else { len },
        };
        f(&mut bank)
    }
    fn access_banks<T>(
        &mut self,
        f: impl FnOnce(&mut LockedFlashBank, Option<&mut LockedFlashBank>) -> T,
    ) -> T {
        let len = self.len();
        let mut bank1 = LockedFlashBank {
            bank_index: 1,
            base: 0x0800_0000,
            len: if self.dual_bank() { len / 2 } else { len },
        };
        let mut bank2 = self.dual_bank().then_some(LockedFlashBank {
            bank_index: 2,
            base: 0x0810_0000,
            len: self.len() / 2,
        });
        f(&mut bank1, bank2.as_mut())
    }
    fn address(&self) -> usize {
        0x0800_0000
    }
    fn len(&self) -> usize {
        FlashSize::bytes()
    }
    fn split(self) -> (LockedFlashBank, Option<LockedFlashBank>) {
        let len = self.len();
        (
            LockedFlashBank {
                bank_index: 1,
                base: 0x0800_0000,
                len: if self.dual_bank() { len / 2 } else { len },
            },
            self.dual_bank().then_some(LockedFlashBank {
                bank_index: 2,
                base: 0x0810_0000,
                len: self.len() / 2,
            }),
        )
    }
    fn dual_bank(&self) -> bool {
        #[cfg(feature = "rm0468")]
        return false;

        #[cfg(not(feature = "rm0468"))]
        match self.len() / 1024 {
            // 1 MB devices: 512k in each bank
            1024 => true,
            // 2 MB devices: 1024k in each bank
            2048 => true,
            // All other devices 128k: one bank
            _ => false,
        }
    }
    fn sector(&self, offset: usize) -> Option<FlashSector> {
        flash_sectors(self.len(), self.dual_bank()).find(|s| s.contains(offset))
    }
}

// /// Read-only flash bank
// ///
// /// # Examples
// ///
// /// ```
// /// use stm32h7xx_hal::pac::Peripherals;
// /// use stm32h7xx_hal::flash::LockedFlashBank;
// /// use embedded_storage::nor_flash::ReadNorFlash;
// ///
// /// let dp = Peripherals::take().unwrap();
// /// let mut flash = LockedFlashBank::new(dp.FLASH);
// /// println!("Flash capacity: {}", ReadNorFlash::capacity(&flash));
// ///
// /// let mut buf = [0u8; 64];
// /// ReadNorFlash::read(&mut flash, 0x0, &mut buf).unwrap();
// /// println!("First 64 bytes of flash memory: {:?}", buf);
// /// ```
pub struct LockedFlashBank {
    bank_index: usize,
    base: usize,
    len: usize,
}

#[allow(clippy::len_without_is_empty)]
impl LockedFlashBank {
    fn registers(&self) -> &BANK {
        match self.bank_index {
            1 => unsafe { &*FLASH::ptr() }.bank1(),
            2 => unsafe { &*FLASH::ptr() }.bank2(),
            _ => unreachable!(),
        }
    }
    /// Memory-mapped address
    pub fn address(&self) -> usize {
        self.base
    }
    /// Size in bytes
    pub fn len(&self) -> usize {
        self.len
    }
    /// Returns a read-only view of flash memory
    pub fn read_all(&self) -> &[u8] {
        let ptr = self.base as *const _;
        unsafe { slice::from_raw_parts(ptr, self.len) }
    }
    /// Unlock flash for erasing/programming until this method's result is
    /// dropped
    pub fn unlocked(&mut self) -> UnlockedFlashBank {
        unlock(self.registers());
        UnlockedFlashBank {
            bank: self.registers(),
            base: self.base,
            len: self.len,
        }
    }
    /// Returns flash memory sector of a given offset. Returns none if offset is
    /// out of range
    pub fn sector(&self, offset: usize) -> Option<FlashSector> {
        FlashSectorIterator::new(0, 0, self.len()).find(|s| s.contains(offset))
    }
}

const UNLOCK_KEY1: u32 = 0x4567_0123;
const UNLOCK_KEY2: u32 = 0xCDEF_89AB;

#[allow(unused_unsafe)]
fn unlock(bank: &BANK) {
    bank.keyr().write(|w| unsafe { w.keyr().bits(UNLOCK_KEY1) });
    bank.keyr().write(|w| unsafe { w.keyr().bits(UNLOCK_KEY2) });
    assert!(!bank.cr().read().lock().bit())
}

impl nor_flash::ErrorType for LockedFlashBank {
    type Error = Error;
}
impl nor_flash::ReadNorFlash for LockedFlashBank {
    const READ_SIZE: usize = 1;

    fn read(
        &mut self,
        offset: u32,
        bytes: &mut [u8],
    ) -> Result<(), Self::Error> {
        let offset = offset as usize;
        bytes.copy_from_slice(&self.read_all()[offset..offset + bytes.len()]);
        Ok(())
    }

    fn capacity(&self) -> usize {
        self.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Test the memory layout of the STM32H742xG/743xG and STM32H745xG/STM32H747xG
    #[test]
    #[cfg(not(feature = "rm0455"))]
    fn flash_dual_bank_1m() {
        let mut sectors = flash_sectors(1 * 1024 * 1024, true);

        #[rustfmt::skip]
        assert_eq!(sectors.next(), Some(FlashSector { number: 0, offset: 0x00000 }));
        #[rustfmt::skip]
        assert_eq!(sectors.next(), Some(FlashSector { number: 1, offset: 0x20000 }));
        #[rustfmt::skip]
        assert_eq!(sectors.next(), Some(FlashSector { number: 2, offset: 0x40000 }));
        #[rustfmt::skip]
        assert_eq!(sectors.next(), Some(FlashSector { number: 3, offset: 0x60000 }));
        // Offsets 0x80000 - 0x100000 not available
        #[rustfmt::skip]
        assert_eq!(sectors.next(), Some(FlashSector { number: 0, offset: 0x100000 }));
        #[rustfmt::skip]
        assert_eq!(sectors.next(), Some(FlashSector { number: 1, offset: 0x120000 }));
        #[rustfmt::skip]
        assert_eq!(sectors.next(), Some(FlashSector { number: 2, offset: 0x140000 }));
        #[rustfmt::skip]
        assert_eq!(sectors.next(), Some(FlashSector { number: 3, offset: 0x160000 }));
        // Offsets 0x180000 - 0x200000 not available
        assert_eq!(sectors.next(), None);
    }

    /// Test the memory layout of the STM32H750xB and STM32H730
    #[test]
    #[cfg(not(feature = "rm0455"))]
    fn flash_single_bank_128k() {
        let mut sectors = flash_sectors(128 * 1024, false);

        #[rustfmt::skip]
        assert_eq!(sectors.next(), Some(FlashSector { number: 0, offset: 0x00000 }));
        assert_eq!(sectors.next(), None);
    }

    /// Test the memory layout of the STM32H7B0
    #[test]
    #[cfg(feature = "rm0455")]
    fn flash_rm0455_single_bank_128k() {
        let mut sectors = flash_sectors(128 * 1024, false);

        for n in 0..16usize {
            #[rustfmt::skip]
            assert_eq!(sectors.next(), Some(FlashSector { number: n as u8, offset: 0x2000*n }));
        }
        assert_eq!(sectors.next(), None);
    }
}
