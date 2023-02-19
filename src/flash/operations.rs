//! Erase, write, read operations for Flash

use core::sync::atomic::{fence, Ordering};
use core::{ptr, slice};

use super::FlashSectorIterator;
use crate::stm32::flash::BANK;
use embedded_storage::nor_flash::{
    ErrorType, NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash,
};

/// Flash erase/program error. From RM0433 Rev 7. Section 4.7
#[non_exhaustive]
#[derive(Debug, Clone, Copy)]
pub enum Error {
    /// The arguments are not properly aligned
    NotAligned,
    /// The arguments are out of bounds
    OutOfBounds,
    /// An illegal erase/program operation was attempted
    WriteProtection,
    /// The programming sequence was incorrect
    ProgrammingSequence,
    /// Application software wrote several times to the same byte
    Strobe,
    /// Write operation was attempted before the completion of the previous
    /// write operation OR a wrap burst request overlaps two or more 256-bit
    /// flash-word addresses
    Inconsistency,
    /// Error occurred during a write or erase operation
    Operation,
    /// Two ECC errors detected during a read
    EccDoubleDetection,
    /// Read operation from a PCROP, secure-only or RDP protected area attempted
    ReadProtection,
    /// Read operation from a secure address attempeted
    ReadSecure,
    /// Other errors
    Other,
}
impl NorFlashError for Error {
    fn kind(&self) -> NorFlashErrorKind {
        match &self {
            Error::NotAligned => NorFlashErrorKind::NotAligned,
            Error::OutOfBounds | Error::ReadProtection | Error::ReadSecure => {
                NorFlashErrorKind::OutOfBounds
            }
            _ => NorFlashErrorKind::Other,
        }
    }
}

impl Error {
    fn read(flash_bank: &BANK) -> Option<Self> {
        let sr = flash_bank.sr.read();
        if sr.pgserr().bit() {
            Some(Error::ProgrammingSequence)
        } else if sr.wrperr().bit() {
            Some(Error::WriteProtection)
        } else if sr.strberr().bit() {
            Some(Error::Strobe)
        } else if sr.incerr().bit() {
            Some(Error::Inconsistency)
        } else if sr.operr().bit() {
            Some(Error::Operation)
        } else if sr.rdperr().bit() {
            Some(Error::ReadProtection)
        } else if sr.rdserr().bit() {
            Some(Error::ReadSecure)
        } else if sr.dbeccerr().bit() {
            Some(Error::EccDoubleDetection)
        } else {
            None
        }
    }
}
fn clear_error_flags(regs: &BANK) {
    regs.sr.modify(|_, w| {
        w.pgserr()
            .set_bit()
            .wrperr()
            .set_bit()
            .strberr()
            .set_bit()
            .incerr()
            .set_bit()
            .operr()
            .set_bit()
            .rdperr()
            .set_bit()
            .rdserr()
            .set_bit()
            .dbeccerr()
            .set_bit()
    })
}

/// Result of `FlashExt::unlocked()`
///
/// # Examples
///
/// ```
/// use stm32h7xx_hal::pac::Peripherals;
/// use stm32h7xx_hal::flash::{FlashExt, LockedFlash, UnlockedFlashBank};
/// use embedded_storage::nor_flash::NorFlash;
///
/// let dp = Peripherals::take().unwrap();
/// let (mut flash, _) = dp.FLASH.split();
///
/// // Unlock flash for writing
/// let mut unlocked_flash = flash.unlocked();
///
/// // Erase the second 128 KB sector.
/// NorFlash::erase(&mut unlocked_flash, 128 * 1024, 256 * 1024).unwrap();
///
/// // Write some data at the start of the second 128 KB sector.
/// let buf = [0u8; 64];
/// NorFlash::write(&mut unlocked_flash, 128 * 1024, &buf).unwrap();
///
/// // Lock flash by dropping
/// drop(unlocked_flash);
/// ```
pub struct UnlockedFlashBank<'a> {
    pub(crate) bank: &'a BANK,
    pub(crate) base: usize,
    pub(crate) len: usize,
}

/// Automatically lock flash erase/program when leaving scope
impl Drop for UnlockedFlashBank<'_> {
    fn drop(&mut self) {
        self.bank.cr.modify(|_, w| w.lock().set_bit());
    }
}

#[allow(clippy::len_without_is_empty)]
impl UnlockedFlashBank<'_> {
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
}

impl UnlockedFlashBank<'_> {
    /// Erase a flash sector
    ///
    /// Refer to the reference manual to see which sector corresponds to which
    /// memory address. Out of bounds sectors will cause an error.
    pub fn erase_sector(&mut self, sector: u8) -> Result<(), Error> {
        if sector as usize * super::SECTOR_SIZE >= self.len() {
            return Err(Error::OutOfBounds);
        }

        // Ensure no effective write, erase or option byte change operation is ongoing
        self.wait_ready();
        // Clear all the error flags due to previous programming/erase
        clear_error_flags(self.bank);

        #[rustfmt::skip]
        self.bank.cr.modify(|_, w| unsafe {
            w
                // start
                .start().set_bit()
                // sector number
                .snb().bits(sector)
                // sector erase
                .ser().set_bit()
                // not programming
                .pg().clear_bit()
        });
        self.wait_ready();
        self.ok()
    }

    /// Program bytes with offset into flash memory
    ///
    /// This method always issues writes with an alignment and size of at least
    /// 256 bits (or 128 bits for RM0455 parts) by padding the first and last
    /// writes with 0xFF if needed.
    pub fn program<'a, I>(
        &mut self,
        offset: usize,
        bytes: I,
    ) -> Result<(), Error>
    where
        I: Iterator<Item = &'a u8>,
    {
        // prepend padding to align the offset to the write_size
        let padding = offset % super::WRITE_SIZE;
        let mut bytes = (0..padding).map(|_| &0xFFu8).chain(bytes).peekable();
        let offset = offset - padding;

        // pointer and offset in 32-bit words
        let ptr = self.base as *mut u32;
        let mut offset = offset / 4;
        let write_size = super::WRITE_SIZE / 4;

        // Ensure no effective write, erase or option byte change operation is ongoing
        self.wait_ready();
        // Clear all the error flags due to previous programming/erase
        clear_error_flags(self.bank);

        // Iterate over buffers of size `write_size`
        while bytes.peek().is_some() {
            #[rustfmt::skip]
            #[allow(unused_unsafe)]
            self.bank.cr.modify(|_, w| unsafe {
                w
                    // double-word parallelism
                    .psize().bits(0b11)
                    // not sector erase
                    .ser().clear_bit()
                    // programming
                    .pg().set_bit()
            });

            // Ensure that the write to the CR register (device memory) is
            // committed *before* we write to flash (normal memory). This
            // prevents ProgrammingSequence errors
            fence(Ordering::SeqCst);

            for _ in 0..write_size {
                let b0 = bytes.next().unwrap_or(&0xFF);
                let b1 = bytes.next().unwrap_or(&0xFF);
                let b2 = bytes.next().unwrap_or(&0xFF);
                let b3 = bytes.next().unwrap_or(&0xFF);

                let word = u32::from_le_bytes([*b0, *b1, *b2, *b3]);
                unsafe {
                    ptr::write_volatile(ptr.add(offset), word);
                }
                offset += 1;
            }

            // Ensure that the writes to flash (normal memory) are committed
            // before we start polling the status register. The write will have
            // already started once the last word was written, so this fence
            // does not cause any additional slowdown
            fence(Ordering::SeqCst);

            // The write buffer should be empty (WBNE=0) immediately, but wait
            // for this nonetheless. Then wait for the write queue
            while {
                let sr = self.bank.sr.read();
                sr.wbne().bit_is_set() | sr.qw().bit_is_set()
            } {}
            self.ok()?;
        }
        self.bank.cr.modify(|_, w| w.pg().clear_bit());
        self.wait_ready();

        self.ok()
    }

    fn wait_ready(&self) {
        while self.bank.sr.read().bsy().bit() {}
    }

    fn ok(&self) -> Result<(), Error> {
        Error::read(self.bank).map(Err).unwrap_or(Ok(()))
    }
}

impl<'a> ErrorType for UnlockedFlashBank<'a> {
    type Error = Error;
}
impl<'a> ReadNorFlash for UnlockedFlashBank<'a> {
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

impl<'a> NorFlash for UnlockedFlashBank<'a> {
    const WRITE_SIZE: usize = super::WRITE_SIZE;
    const ERASE_SIZE: usize = super::SECTOR_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        let mut current = from as usize;

        for sector in FlashSectorIterator::new(0, 0, self.len) {
            if sector.contains(current) {
                self.erase_sector(sector.number)?;
                current += Self::ERASE_SIZE;
            }
            if current >= to as usize {
                break;
            }
        }

        Ok(())
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        self.program(offset as usize, bytes.iter())
    }
}
