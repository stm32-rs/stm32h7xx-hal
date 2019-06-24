use core::cmp;
use core::mem;

use crate::hal::blocking::rng;
use crate::rcc::Ccdr;
use crate::stm32::rcc::d2ccip2r;
use crate::stm32::RNG;
use crate::time::Hertz;

#[derive(Debug)]
pub enum ErrorKind {
    ClockError,
    SeedError,
}

trait KerClk {
    fn kernel_clk(ccdr: &Ccdr) -> Option<Hertz>;
}

impl KerClk for RNG {
    fn kernel_clk(ccdr: &Ccdr) -> Option<Hertz> {
        match ccdr.rb.d2ccip2r.read().rngsel() {
            d2ccip2r::RNGSELR::HSI48 => ccdr.clocks.hsi48_ck(),
            d2ccip2r::RNGSELR::PLL1_Q => ccdr.clocks.pll1_q_ck(),
            d2ccip2r::RNGSELR::LSE => unimplemented!(),
            d2ccip2r::RNGSELR::LSI => unimplemented!(),
        }
    }
}

pub trait RngExt {
    fn constrain(self, ccdr: &mut Ccdr) -> Rng;
}

impl RngExt for RNG {
    fn constrain(self, ccdr: &mut Ccdr) -> Rng {
        ccdr.ahb2.enr().modify(|_, w| w.rngen().set_bit());
        ccdr.ahb2.rstr().modify(|_, w| w.rngrst().set_bit());
        ccdr.ahb2.rstr().modify(|_, w| w.rngrst().clear_bit());

        let hclk = ccdr.clocks.hclk();
        let rng_clk =
            Self::kernel_clk(&ccdr).expect("RNG input clock not running!");

        // Otherwise clock checker will always flag an error
        // See RM0433 Rev 6 Section 33.3.6
        assert!(rng_clk.0 > hclk.0 / 32);

        self.cr.modify(|_, w| w.ced().enabled().rngen().enabled());

        Rng { rb: self }
    }
}

pub trait RngCore<W> {
    fn gen(&mut self) -> Result<W, ErrorKind>;
    fn gen_range(&mut self, low: W, high: W) -> Result<W, ErrorKind>;
    fn fill(&mut self, dest: &mut [W]) -> Result<(), ErrorKind>;
}

pub struct Rng {
    rb: RNG,
}

impl Rng {
    /// Returns 32 bits of randomness, or error
    pub fn next(&mut self) -> Result<u32, ErrorKind> {
        loop {
            let status = self.rb.sr.read();
            if status.cecs().bit() {
                return Err(ErrorKind::ClockError);
            }
            if status.secs().bit() {
                return Err(ErrorKind::SeedError);
            }
            if status.drdy().bit() {
                return Ok(self.rb.dr.read().rndata().bits());
            }
        }
    }

    pub fn release(self) -> RNG {
        self.rb
    }

    /// Return random boolean, or error
    pub fn gen_bool(&mut self) -> Result<bool, ErrorKind> {
        let val = self.next()?;
        Ok(val & 1 == 1)
    }

    /// Return random booleans in the ratio one:zero = n:(d-n), or
    /// error
    pub fn gen_ratio(
        &mut self,
        numerator: u32,
        denominator: u32,
    ) -> Result<bool, ErrorKind> {
        assert!(denominator > 0);
        let val = self.gen_range(0, denominator)?;
        Ok(numerator > val)
    }

    /// Choose random value from array, or error
    pub fn choose<'a, T>(
        &mut self,
        values: &'a [T],
    ) -> Result<&'a T, ErrorKind> {
        let val = self.gen_range(0, values.len())?;
        Ok(&values[val])
    }

    /// Choose random value from array, or error
    pub fn choose_mut<'a, T>(
        &mut self,
        values: &'a mut [T],
    ) -> Result<&'a mut T, ErrorKind> {
        let val = self.gen_range(0, values.len())?;
        Ok(&mut values[val])
    }

    /// Choose random value from array, or return error
    pub fn shuffle<T>(&mut self, values: &mut [T]) -> Result<(), ErrorKind> {
        for i in (1..values.len()).rev() {
            values.swap(i, self.gen_range(0, i + 1)?);
        }
        Ok(())
    }
}

impl rng::Read for Rng {
    type Error = ErrorKind;

    fn read(&mut self, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.fill(buffer)
    }
}

macro_rules! rng_core {
    ($($type:ty),+) => {
        $(
            impl RngCore<$type> for Rng {
                /// Returns a single element with random value, or error
                fn gen(&mut self) -> Result<$type, ErrorKind> {
                    let val = self.next()?;
                    Ok(val as $type)
                }

                /// Returns a single element with random value in range, or error
                fn gen_range(&mut self, low: $type, high: $type) -> Result<$type, ErrorKind> {
                    assert!(high > low);
                    let range = high - low;
                    let val: $type = self.gen()?;
                    Ok(low + val % range)
                }

                /// Fills buffer with random values, or return error
                fn fill(&mut self, buffer: &mut [$type]) -> Result<(), ErrorKind> {
                    const BATCH_SIZE: usize = 4 / mem::size_of::<$type>();
                    let mut i = 0_usize;
                    while i < buffer.len() {
                        let random_word = self.next()?;
                        let bytes: [$type; BATCH_SIZE] = unsafe { mem::transmute(random_word) };
                        let n = cmp::min(BATCH_SIZE, buffer.len() - i);
                        buffer[i..i + n].copy_from_slice(&bytes[..n]);
                        i += n;
                    }
                    Ok(())
                }
            }
        )+
    };
}

rng_core!(usize, u32, u16, u8);
