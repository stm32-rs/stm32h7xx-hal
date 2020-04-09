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
    FifoData,
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

        // Write the prescaler and select flash bank 2 - flash bank 1 is currently unsupported.
        regs.cr.write(|w| unsafe {
            w.prescaler().bits(divisor as u8)
             .fsel().set_bit()
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
                     .dmode().bits(0b01)
                });

                Ok(())
            },
            QspiMode::FourBit => {
                self.rb.ccr.modify(|_, w| unsafe {
                    w.imode().bits(0b11)
                     .dmode().bits(0b11)
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

        // Clear the transfer complete flag.
        self.rb.fcr.modify(|_ ,w| w.ctcf().set_bit());

        // Write the length
        self.rb.dlr.write(|w| unsafe {w.dl().bits(data.len() as u32 - 1)});

        // Configure the mode to indirect write and configure the instruction byte.
        self.rb.ccr.modify(|_, w| unsafe {
            w.fmode().bits(0b00)
        });

        self.rb.ccr.modify(|_, w| unsafe {
            w.instruction().bits(addr)
        });

        // Enable the transaction
        self.rb.cr.modify(|_, w| {w.en().set_bit()});

        // Write data to the FIFO in a byte-wise manner.
        unsafe {
            for byte in data {
                ptr::write_volatile(&self.rb.dr as *const _ as *mut u8, *byte);
            }
        }

        // Wait for the transaction to complete
        while self.rb.sr.read().tcf().bit_is_clear() {}

        // Check that there is no more transaction pending.
        if self.is_busy() {
            return Err(QspiError::FifoData);
        }

        self.rb.cr.modify(|_, w| {w.en().clear_bit()});

        // Clear the transfer complete flag.
        self.rb.fcr.modify(|_ ,w| w.ctcf().set_bit());

        Ok(())
    }

    pub fn read(&mut self, addr: u8, dest: &mut [u8]) -> Result<(), QspiError> {
        if self.is_busy() {
            return Err(QspiError::Busy);
        }

        // Clear the transfer complete flag.
        self.rb.fcr.modify(|_ ,w| w.ctcf().set_bit());

        // Write the length that should be read.
        self.rb.dlr.write(|w| unsafe {
            w.dl().bits(dest.len() as u32 - 1)
        });

        // Configure the mode to indirect read and configure the instruction byte.
        self.rb.ccr.modify(|_, w| unsafe {
            w.fmode().bits(0b01)
        });

        // Enable the transaction
        self.rb.cr.modify(|_, w| {w.en().set_bit()});

        // Write the instruction bits to force the read to start. This has to be done after the
        // transaction is enabled to indicate to the peripheral that we are ready to start the
        // transaction.
        self.rb.ccr.modify(|_, w| unsafe {
            w.instruction().bits(addr)
        });

        // Wait for the transaction to complete
        while self.rb.sr.read().tcf().bit_is_clear() {}

        // Check for underflow on the FIFO.
        if (self.rb.sr.read().flevel().bits() as usize) < dest.len() {
            return Err(QspiError::Underflow);
        }

        // Read data from the FIFO in a byte-wise manner.
        unsafe {
            for location in dest {
                *location = ptr::read_volatile(&self.rb.dr as *const _ as *const u8);
            }
        }

        // Check that there is no more transaction pending.
        if self.is_busy() {
            return Err(QspiError::FifoData);
        }

        self.rb.cr.modify(|_, w| {w.en().clear_bit()});

        // Clear the transfer complete flag.
        self.rb.fcr.modify(|_ ,w| w.ctcf().set_bit());

        Ok(())
    }
}
