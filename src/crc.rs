//! Cyclic Redundancy Check (CRC)
//!
//! # Examples
//!
//! - [CRC example](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/crc.rs)

use core::convert::TryInto;
use core::fmt;

use crate::rcc::{rec, ResetEnable};
use crate::stm32::{crc, CRC};

pub trait CrcExt {
    /// Enable the CRC unit.
    fn crc(self, prec: rec::Crc) -> Crc;
}

impl CrcExt for CRC {
    fn crc(self, prec: rec::Crc) -> Crc {
        prec.enable().reset();
        Crc {
            reg: self,
            output_xor: 0,
        }
    }
}

/// The hardware CRC unit.
pub struct Crc {
    reg: CRC,
    output_xor: u32,
}

impl Crc {
    /// Set the unit's configuration, discarding previous state.
    pub fn set_config(&mut self, config: &Config) {
        self.output_xor = config.output_xor & config.poly.xor_mask();

        // manual says unit must be reset (or DR read) before change of polynomial
        // (technically only in case of ongoing calculation, but DR is buffered)
        self.reg.cr.modify(|_, w| {
            w.polysize()
                .bits(config.poly.polysize())
                .rev_in()
                .bits(config.get_reverse_input())
                .rev_out()
                .bit(config.reverse_output)
                .reset()
                .set_bit()
        });
        self.reg.pol.write(|w| w.pol().bits(config.poly.pol()));
        self.reg.init.write(|w| w.init().bits(config.initial));
        // writing to INIT sets DR to its value
    }

    /// Write data to the CRC unit. Note that CRC calculation works
    /// faster if more data is given at once.
    pub fn update(&mut self, data: &[u8]) {
        // write 4 bytes at once, then 2, then 1, as appropriate
        // in the case of a single large slice this improves speed by >3x
        let mut words = data.chunks_exact(4);
        for word in words.by_ref() {
            let word = u32::from_be_bytes(word.try_into().unwrap());
            self.reg.dr().write(|w| w.dr().bits(word));
        }

        // there will be at most 3 bytes remaining, so 1 half-word and 1 byte
        let mut half_word = words.remainder().chunks_exact(2);
        if let Some(half_word) = half_word.next() {
            let half_word = u16::from_be_bytes(half_word.try_into().unwrap());
            self.reg.dr16().write(|w| w.dr16().bits(half_word));
        }

        if let Some(byte) = half_word.remainder().first() {
            self.reg.dr8().write(|w| w.dr8().bits(*byte));
        }
    }

    /// Write data to the CRC unit, return CRC so far. This function should
    /// only be used if you need its result, as retrieving the CRC takes time.
    #[must_use = "retrieving the CRC takes time, use update() if not needed"]
    pub fn update_and_read(&mut self, data: &[u8]) -> u32 {
        self.update(data);
        self.read_crc()
    }

    /// Read the CRC and reset DR to initial value in preparation for a new CRC.
    /// This does not reset the configuration options.
    pub fn finish(&mut self) -> u32 {
        let result = self.read_crc();
        self.reg.cr.modify(|_, w| w.reset().set_bit());
        result
    }

    /// Read the CRC without resetting the unit.
    #[inline]
    pub fn read_crc(&self) -> u32 {
        self.read_crc_no_xor() ^ self.output_xor
    }

    /// Read the state of the CRC calculation. When used as the initial value
    /// of an otherwise identical CRC config, this allows resuming calculation
    /// from the current state.
    ///
    /// This is equivalent to [`read_crc()`](Self::read_crc) in the case of an
    /// algorithm that does not apply an output XOR or reverse the output bits.
    pub fn read_state(&self) -> u32 {
        let state = self.read_crc_no_xor();
        if self.reg.cr.read().rev_out().is_reversed() {
            state.reverse_bits()
        } else {
            state
        }
    }

    /// Read the CRC without applying output XOR.
    #[inline(always)]
    fn read_crc_no_xor(&self) -> u32 {
        self.reg.dr().read().dr().bits()
    }

    /// Write the independent data register. The IDR can be used as
    /// temporary storage. It is not cleared on CRC hash reset.
    ///
    /// The IDR is not involved with CRC calculation.
    pub fn set_idr(&mut self, value: u32) {
        self.reg.idr.write(|w| w.idr().bits(value));
    }

    /// Get the current value of the independent data register.
    ///
    /// The IDR is not involved with CRC calculation.
    pub fn get_idr(&self) -> u32 {
        self.reg.idr.read().idr().bits()
    }

    /// Returns a reference to the inner peripheral
    pub fn inner(&self) -> &CRC {
        &self.reg
    }

    /// Returns a mutable reference to the inner peripheral
    pub fn inner_mut(&mut self) -> &mut CRC {
        &mut self.reg
    }
}

#[macro_use]
mod macros {
    /// Generate an error if number passed is even
    macro_rules! ensure_is_odd {
        ( $x:expr ) => {
            if $x % 2 == 0 {
                Err(PolynomialError::EvenPoly)
            } else {
                Ok($x)
            }
        };
    }
}

/// A CRC polynomial.
///
/// The STM32H7 CRC unit only supports odd polynomials, and the constructors
/// will check to ensure the polynomial is odd unless the `_unchecked` variants
/// are used.
///
/// Even polynomials are essentially never used in CRCs, so you most likely
/// don't need to worry about this if you aren't creating your own algorithm.
///
/// A polynomial being even means that the least significant bit is `0`
/// in the polynomial's normal representation.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Polynomial(Poly);

impl Polynomial {
    /// Create a 7-bit polynomial. Returns an error if the polynomial passed
    /// has the MSB set or is even.
    pub fn bits7(poly: u8) -> Result<Self, PolynomialError> {
        if poly <= 0x7F {
            Ok(Self(Poly::B7(ensure_is_odd!(poly)?)))
        } else {
            Err(PolynomialError::TooLarge)
        }
    }

    /// Create an 8-bit polynomial. Returns an error if the polynomial passed is even.
    pub fn bits8(poly: u8) -> Result<Self, PolynomialError> {
        Ok(Self(Poly::B8(ensure_is_odd!(poly)?)))
    }

    /// Create a 16-bit polynomial. Returns an error if the polynomial passed is even.
    pub fn bits16(poly: u16) -> Result<Self, PolynomialError> {
        Ok(Self(Poly::B16(ensure_is_odd!(poly)?)))
    }

    /// Create a 32-bit polynomial. Returns an error if the polynomial passed is even.
    pub fn bits32(poly: u32) -> Result<Self, PolynomialError> {
        Ok(Self(Poly::B32(ensure_is_odd!(poly)?)))
    }

    /// Create a 7-bit polynomial. If the polynomial passed is even the
    /// CRC unit will give incorrect results.
    pub const fn bits7_unchecked(poly: u8) -> Self {
        Self(Poly::B7(poly))
    }

    /// Create an 8-bit polynomial. If the polynomial passed is even the
    /// CRC unit will give incorrect results.
    pub const fn bits8_unchecked(poly: u8) -> Self {
        Self(Poly::B8(poly))
    }

    /// Create a 16-bit polynomial. If the polynomial passed is even the
    /// CRC unit will give incorrect results.
    pub const fn bits16_unchecked(poly: u16) -> Self {
        Self(Poly::B16(poly))
    }

    /// Create a 32-bit polynomial. If the polynomial passed is even the
    /// CRC unit will give incorrect results.
    pub const fn bits32_unchecked(poly: u32) -> Self {
        Self(Poly::B32(poly))
    }

    /// Return POLYSIZE register value.
    const fn polysize(self) -> u8 {
        (match self.0 {
            Poly::B7(_) => crc::cr::POLYSIZE_A::POLYSIZE7,
            Poly::B8(_) => crc::cr::POLYSIZE_A::POLYSIZE8,
            Poly::B16(_) => crc::cr::POLYSIZE_A::POLYSIZE16,
            Poly::B32(_) => crc::cr::POLYSIZE_A::POLYSIZE32,
        }) as u8
    }

    /// Return POL register value.
    const fn pol(self) -> u32 {
        match self.0 {
            Poly::B7(pol) | Poly::B8(pol) => pol as u32,
            Poly::B16(pol) => pol as u32,
            Poly::B32(pol) => pol,
        }
    }

    /// Return mask for output XOR according to size.
    const fn xor_mask(self) -> u32 {
        match self.0 {
            Poly::B7(_) => 0x7F,
            Poly::B8(_) => 0xFF,
            Poly::B16(_) => 0xFFFF,
            Poly::B32(_) => 0xFFFF_FFFF,
        }
    }
}

impl Default for Polynomial {
    /// Returns the 32-bit polynomial `0x04C1_1DB7`.
    fn default() -> Self {
        Self(Poly::B32(0x04C1_1DB7))
    }
}

/// Errors generated when trying to create invalid polynomials.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PolynomialError {
    /// Tried to create an even polynomial.
    /// The hardware CRC unit only supports odd polynomials.
    EvenPoly,
    /// Tried to create a 7-bit polynomial with an 8-bit number
    /// (greater than `0x7F`).
    TooLarge,
}

impl fmt::Display for PolynomialError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match *self {
            Self::EvenPoly => f.write_str("tried to create an even polynomial"),
            Self::TooLarge => f.write_str(
                "tried to create a 7-bit polynomial with an 8-bit number",
            ),
        }
    }
}

/// Internal representation of a polynomial.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum Poly {
    /// 7-bit polynomial
    B7(u8),
    /// 8-bit polynomial
    B8(u8),
    /// 16-bit polynomial
    B16(u16),
    /// 32-bit polynomial
    B32(u32),
}

/// How to reverse the input bits.
///
/// ST refers to this as both 'reversal' and 'inversion'. If a CRC calls for
/// 'reflection' it most likely wants [`BitReversal::Byte`] and output reversal
/// enabled.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum BitReversal {
    /// Each input byte has its bits reversed. `0x1A2B3C4D` becomes `0x58D43CB2`.
    Byte = crc::cr::REV_IN_A::BYTE as u8,
    /// Bits reversed by half-word. `0x1A2B3C4D` becomes `0xD458B23C`.
    HalfWord = crc::cr::REV_IN_A::HALFWORD as u8,
    /// Bits reversed by word. `0x1A2B3C4D` becomes `0xB23CD458`.
    Word = crc::cr::REV_IN_A::WORD as u8,
}

/// CRC unit configuration.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    poly: Polynomial,
    initial: u32,
    reverse_input: Option<BitReversal>,
    reverse_output: bool,
    output_xor: u32,
}

impl Config {
    /// Creates the Config struct with the default configuration of the STM32H7 CRC unit:
    ///
    /// * `0x04C1_1DB7` polynomial
    /// * `0xFFFF_FFFF` initial value
    /// * Bits not reflected
    /// * `0` output XOR
    ///
    /// This configuration is the MPEG-2 CRC.
    pub const fn new() -> Self {
        Self {
            poly: Polynomial(Poly::B32(0x04C1_1DB7)),
            initial: 0xFFFF_FFFF,
            reverse_input: None,
            reverse_output: false,
            output_xor: 0,
        }
    }

    /// Set the polynomial.
    pub const fn polynomial(mut self, poly: Polynomial) -> Self {
        self.poly = poly;
        self
    }

    /// Set the initial value of the CRC. The CRC unit will only use the needed
    /// bits to match the polynomial; the default initial value `0xFFFF_FFFF`
    /// will actually write `0x7F` in the case of a 7-bit CRC, for example.
    pub const fn initial_value(mut self, initial: u32) -> Self {
        self.initial = initial;
        self
    }

    /// Set how to reverse the bits of the input. `None` means no reversal.
    pub const fn reverse_input(mut self, reverse: Option<BitReversal>) -> Self {
        self.reverse_input = reverse;
        self
    }

    /// Get the register value of input reversal setting.
    fn get_reverse_input(&self) -> u8 {
        self.reverse_input
            .map(|rev| rev as u8)
            .unwrap_or(crc::cr::REV_IN_A::NORMAL as u8)
    }

    /// Set whether to reverse the bits of the output.
    pub const fn reverse_output(mut self, reverse: bool) -> Self {
        self.reverse_output = reverse;
        self
    }

    /// Set whether to reflect the CRC. When enabled, reflection is
    /// [`BitReversal::Byte`] and output reversal enabled. This is simply
    /// a convenience function as many CRC algorithms call for this.
    pub const fn reflect(self, reflect: bool) -> Self {
        if reflect {
            self.reverse_input(Some(BitReversal::Byte))
                .reverse_output(true)
        } else {
            self.reverse_input(None).reverse_output(false)
        }
    }

    /// Set the value to XOR the output with. Automatically masked to the proper size.
    pub const fn output_xor(mut self, output_xor: u32) -> Self {
        self.output_xor = output_xor;
        self
    }
}

impl Default for Config {
    /// Calls [`Config::new`].
    fn default() -> Self {
        Self::new()
    }
}
