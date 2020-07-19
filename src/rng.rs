use core::cmp;
use core::mem;

use crate::hal::blocking::rng;
use crate::rcc::{rec, rec::RngClkSel};
use crate::rcc::{CoreClocks, ResetEnable};
use crate::stm32::RNG;
use crate::time::Hertz;

#[derive(Debug)]
pub enum ErrorKind {
    ClockError,
    SeedError,
}

trait KerClk {
    fn kernel_clk(prec: rec::Rng, clocks: &CoreClocks) -> Option<Hertz>;
}

impl KerClk for RNG {
    fn kernel_clk(prec: rec::Rng, clocks: &CoreClocks) -> Option<Hertz> {
        match prec.get_kernel_clk_mux() {
            RngClkSel::HSI48 => clocks.hsi48_ck(),
            RngClkSel::PLL1_Q => clocks.pll1_q_ck(),
            RngClkSel::LSE => unimplemented!(),
            RngClkSel::LSI => unimplemented!(),
        }
    }
}

pub trait RngExt {
    fn constrain(self, prec: rec::Rng, clocks: &CoreClocks) -> Rng;
}

impl RngExt for RNG {
    fn constrain(self, prec: rec::Rng, clocks: &CoreClocks) -> Rng {
        let prec = prec.enable().reset();

        let hclk = clocks.hclk();
        let rng_clk = Self::kernel_clk(prec, clocks)
            .expect("RNG input clock not running!");

        // Otherwise clock checker will always flag an error
        // See RM0433 Rev 6 Section 33.3.6
        assert!(rng_clk.0 > hclk.0 / 32);

        self.cr.modify(|_, w| w.ced().enabled().rngen().enabled());

        Rng { rb: self }
    }
}

pub trait RngCore<W> {
    fn gen(&mut self) -> Result<W, ErrorKind>;
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

rng_core!(u32, u16, u8);

// Test host may have > 32-bit types, which we don't consider.
#[cfg(not(test))]
rng_core!(usize);
