use crate::{
    stm32,
    time::Hertz,
    rcc::Ccdr,
};

use core::ptr;

#[derive(Debug)]
pub enum QspiMode {
    OneBit,
    TwoBit,
    FourBit,
}

#[derive(Debug)]
pub enum QspiError {
    Busy,
    InvalidFrequency,
    Unsupported,
    InvalidAddress,
    Underflow,
    InvalidClock,
}

// TODO: Implement traits for valid QSPI pins and mark them as part of the structure for validation.
pub struct Qspi {
    rb: stm32::QUADSPI,
}

impl Qspi {
    pub fn new<T>(regs: stm32::QUADSPI, ccdr: &Ccdr, frequency: T) -> Result<Self, QspiError>
    where
        T: Into<Hertz>,
    {
        // Enable quad SPI in the clocks.
        ccdr.rb.ahb3enr.modify(|_, w| w.qspien().enabled());

        let spi_kernel_ck = match Self::get_clock(&ccdr) {
            Some(freq_hz) => freq_hz.0,
            _ => return Err(QspiError::InvalidClock),
        };

        // Disable QUADSPI before configuring it.
        regs.cr.write(|w| {
            w.en().clear_bit()
        });

        // Clear all pending flags.
        regs.fcr.write(|w| {
            w
            .ctof().set_bit()
            .csmf().set_bit()
            .ctcf().set_bit()
            .ctef().set_bit()
        });

        // Configure the communication method for QSPI.
        regs.ccr.write(|w| unsafe {
            w
            .fmode().bits(0)
            .dmode().bits(0b01)
            .imode().bits(0b01)
        });

        // Ensure QSPI keeps the clock IDLE-low.
        regs.dcr.write(|w| {
            w.ckmode().clear_bit()
        });

        let spi_frequency = frequency.into().0;
        let divisor = match spi_kernel_ck / spi_frequency {
            1..=255 => {
                spi_kernel_ck / spi_frequency - 1
            },
            _ => {
                return Err(QspiError::InvalidFrequency);
            },
        };

        regs.cr.write(|w| unsafe {
            w.prescaler().bits(divisor as u8)
        });

        Ok(Qspi{rb: regs})
    }

    pub fn is_busy(&self) -> bool {
        self.rb.sr.read().busy().bit_is_set()
    }

    fn get_clock(ccdr: &Ccdr) -> Option<Hertz> {
        match ccdr.rb.d1ccipr.read().qspisel().variant() {
            stm32::rcc::d1ccipr::QSPISEL_A::RCC_HCLK3 => Some(ccdr.clocks.hclk()),
            stm32::rcc::d1ccipr::QSPISEL_A::PLL1_Q => ccdr.clocks.pll1_q_ck(),
            stm32::rcc::d1ccipr::QSPISEL_A::PLL2_R => ccdr.clocks.pll2_r_ck(),
            stm32::rcc::d1ccipr::QSPISEL_A::PER => ccdr.clocks.per_ck(),
        }
    }

    pub fn configure_mode(&mut self, mode: QspiMode) -> Result<(), QspiError> {
        if self.rb.sr.read().busy().bit_is_set() {
            return Err(QspiError::Busy);
        }

        match mode {
            QspiMode::OneBit => {
                self.rb.ccr.modify(|_, w| unsafe {
                    w.imode().bits(0b01)
                     .fmode().bits(0b01)
                });

                Ok(())
            },
            QspiMode::FourBit => {
                self.rb.ccr.modify(|_, w| unsafe {
                    w.imode().bits(0b11)
                     .fmode().bits(0b11)
                });

                Ok(())
            },
            _ => Err(QspiError::Unsupported)
        }
    }

    pub fn write(&mut self, addr: u8, data: &[u8]) -> Result<(), QspiError> {
        if self.is_busy() {
            return Err(QspiError::Busy);
        }

        // Disable SPI first
        self.rb.cr.modify(|_, w| {w.en().clear_bit()});

        // Configure the mode to indirect write and configure the instruction byte.
        self.rb.ccr.modify(|_, w| unsafe {
            w.fmode().bits(0b00)
             .instruction().bits(0x80_u8 | addr)
        });

        // TODO: Flush the write FIFO?

        // Write the length
        self.rb.dlr.write(|w| unsafe {w.dl().bits(data.len() as u32)});

        // Write data to the FIFO in a byte-wise manner.
        unsafe {
            for byte in data {
                ptr::write_volatile(&self.rb.dr as *const _ as *mut u8, *byte);
            }
        }

        // Enable the transaction
        self.rb.cr.modify(|_, w| {w.en().set_bit()});

        // Wait for the transaction to complete
        while self.is_busy() {}

        self.rb.cr.modify(|_, w| {w.en().clear_bit()});

        Ok(())
    }

    pub fn read(&mut self, addr: u8, dest: &mut [u8]) -> Result<(), QspiError> {
        if self.is_busy() {
            return Err(QspiError::Busy);
        }

        if (addr & 0x80) != 0 {
            return Err(QspiError::InvalidAddress);
        }

        // Disable SPI first?
        self.rb.cr.modify(|_, w| {w.en().clear_bit()});

        // Configure the mode to indirect read and configure the instruction byte.
        self.rb.ccr.modify(|_, w| unsafe {
            w.fmode().bits(0b01)
             .instruction().bits(addr)
        });

        // TODO: Flush the data FIFO?

        // Write the length that should be read.
        self.rb.dlr.write(|w| unsafe {
            w.dl().bits(dest.len() as u32)
        });

        // Enable the transaction
        self.rb.cr.modify(|_, w| {w.en().set_bit()});

        // Wait for the transaction to complete
        while self.is_busy() {}

        self.rb.cr.modify(|_, w| {w.en().clear_bit()});

        // Check that there's a valid number of bytes in the FIFO to be read.
        if (self.rb.sr.read().flevel().bits() as usize) < dest.len() {
            return Err(QspiError::Underflow);
        }

        // Read data from the FIFO in a byte-wise manner.
        unsafe {
            for location in dest {
                *location = ptr::read_volatile(&self.rb.dr as *const _ as *const u8);
            }
        }

        Ok(())
    }
}
