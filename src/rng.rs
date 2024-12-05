//! Random Number Generator
//!
//! # Examples
//!
//! - [Random Blinky](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/blinky_random.rs)

use core::cmp;
use core::mem;

use crate::hal::blocking::rng;
use crate::rcc::{rec, rec::RngClkSel};
use crate::rcc::{CoreClocks, ResetEnable};
use crate::stm32::RNG;
use crate::time::Hertz;

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ErrorKind {
    ClockError = 0,
    SeedError = 1,
}

pub trait KerClk {
    /// Return the kernel clock for the Random Number Generator
    ///
    /// # Panics
    ///
    /// Panics if the kernel clock is not running
    fn kernel_clk_unwrap(prec: rec::Rng, clocks: &CoreClocks) -> Hertz;
}

impl KerClk for RNG {
    fn kernel_clk_unwrap(prec: rec::Rng, clocks: &CoreClocks) -> Hertz {
        match prec.get_kernel_clk_mux() {
            RngClkSel::Hsi48 => {
                clocks.hsi48_ck().expect("RNG: HSI48 must be enabled")
            }
            RngClkSel::Pll1Q => {
                clocks.pll1_q_ck().expect("RNG: PLL1_Q must be enabled")
            }
            RngClkSel::Lse => unimplemented!(),
            RngClkSel::Lsi => {
                clocks.lsi_ck().expect("RNG: LSI must be enabled")
            }
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
        let rng_clk = Self::kernel_clk_unwrap(prec, clocks);

        // Otherwise clock checker will always flag an error
        // See RM0433 Rev 6 Section 33.3.6
        assert!(rng_clk > hclk / 32, "RNG: Clock too slow");

        self.cr().modify(|_, w| w.ced().enabled().rngen().enabled());

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
    pub fn value(&mut self) -> Result<u32, ErrorKind> {
        loop {
            let status = self.rb.sr().read();
            if status.cecs().bit() {
                return Err(ErrorKind::ClockError);
            }
            if status.secs().bit() {
                return Err(ErrorKind::SeedError);
            }
            if status.drdy().bit() {
                return Ok(self.rb.dr().read().rndata().bits());
            }
        }
    }

    pub fn release(self) -> RNG {
        self.rb
    }

    /// Returns a reference to the inner peripheral
    pub fn inner(&self) -> &RNG {
        &self.rb
    }

    /// Returns a mutable reference to the inner peripheral
    pub fn inner_mut(&mut self) -> &mut RNG {
        &mut self.rb
    }
}

impl core::iter::Iterator for Rng {
    type Item = u32;

    fn next(&mut self) -> Option<u32> {
        self.value().ok()
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
                    let val = self.value()?;
                    Ok(val as $type)
                }

                /// Fills buffer with random values, or return error
                fn fill(&mut self, buffer: &mut [$type]) -> Result<(), ErrorKind> {
                    const BATCH_SIZE: usize = 4 / mem::size_of::<$type>();
                    let mut i = 0_usize;
                    while i < buffer.len() {
                        let random_word = self.value()?;

                        // using to_ne_bytes does not work for u8 and would make the macro
                        // implementation more complicated
                        #[allow(clippy::transmute_num_to_bytes)]
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

// Only for types larger than 32 bits
macro_rules! rng_core_large {
    ($($type:ty),+) => {
        $(
            impl RngCore<$type> for Rng {
                fn gen(&mut self) -> Result<$type, ErrorKind> {
                    const WORDS: usize = mem::size_of::<$type>() / mem::size_of::<u32>();
                    let mut res: $type = 0;

                    for i in 0..WORDS {
                        res |= (self.value()? as $type) << (i * (mem::size_of::<u32>() * 8))
                    }

                    Ok(res)
                }

                fn fill(&mut self, dest: &mut [$type]) -> Result<(), ErrorKind> {
                    let len = dest.len() * (mem::size_of::<$type>() / mem::size_of::<u32>());
                    let ptr = dest.as_mut_ptr() as *mut u32;
                    let slice_u32 = unsafe { core::slice::from_raw_parts_mut(ptr, len) };
                    self.fill(slice_u32)
                }
            }
        )+
    };
}

macro_rules! rng_core_transmute {
    ($($type:ty = $from:ty),+) => {
        $(
            impl RngCore<$type> for Rng {
                fn gen(&mut self) -> Result<$type, ErrorKind> {
                    let num = <Self as RngCore<$from>>::gen(self)?;
                    Ok(unsafe { mem::transmute::<$from, $type>(num) })
                }

                fn fill(&mut self, dest: &mut [$type]) -> Result<(), ErrorKind> {
                    let unsigned_slice = unsafe { mem::transmute::<&mut [$type], &mut [$from]>(dest) };
                    <Self as RngCore<$from>>::fill(self, unsigned_slice)
                }
            }
        )+
    };
}

rng_core!(u8, u16, u32);

// Alignment of these types must be a multiple of mem::align_of::<32>()
rng_core_large!(u64, u128);

// A and B must have the same alignment
// rng_core_transmute!(A = B)
// assert!(mem::align_of::<A>() == mem::align_of::<B>())
rng_core_transmute!(
    i8 = u8,
    i16 = u16,
    i32 = u32,
    i64 = u64,
    i128 = u128,
    isize = usize
);

// If usize is 32 bits, use the rng_core! impl
#[cfg(target_pointer_width = "32")]
rng_core!(usize);

// If usize is 64 bits, use the rng_core_large! impl
#[cfg(target_pointer_width = "64")]
rng_core_large!(usize);

// rand_core
#[cfg(feature = "rand")]
#[cfg_attr(docsrs, doc(cfg(feature = "rand")))]
impl rand_core::RngCore for Rng {
    /// Generate a random u32
    /// Panics if RNG fails.
    fn next_u32(&mut self) -> u32 {
        self.gen().unwrap()
    }

    /// Generate a random u64
    /// Panics if RNG fails.
    fn next_u64(&mut self) -> u64 {
        self.gen().unwrap()
    }

    /// Fill a slice with random data.
    /// Panics if RNG fails.
    fn fill_bytes(&mut self, dest: &mut [u8]) {
        self.fill(dest).unwrap()
    }

    /// Try to fill a slice with random data. Return an error if RNG fails.
    fn try_fill_bytes(
        &mut self,
        dest: &mut [u8],
    ) -> Result<(), rand_core::Error> {
        self.fill(dest).map_err(|e| {
            core::num::NonZeroU32::new(
                rand_core::Error::CUSTOM_START + e as u32,
            )
            // This should never fail as long as no enum variant is equal to 0
            .expect("Internal hal error")
            .into()
        })
    }
}

#[cfg(feature = "rand")]
#[cfg_attr(docsrs, doc(cfg(feature = "rand")))]
impl rand_core::CryptoRng for Rng {}
