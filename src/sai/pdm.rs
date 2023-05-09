//! # Serial Audio Interface - Pulse Density Modulation
//!
//! Pulse Density Modulation. Currently only a single channel (CK1,
//! D1) is implemented, but more could be added once they've been
//! tested.
//!
//! ```
//! let d1 = gpioc.pc1.into_alternate();
//! let ck1 = gpioe.pe2.into_alternate();
//! let pins = (ck1, d1);
//!
//! // Configure SAI for PDM mode
//! let mut sai = dp.SAI1.pdm((ck1, d1), 1_024.kHz(), ccdr.peripheral.SAI1, &ccdr.clocks);
//!
//! let _ = block!(sai.read_data()).unwrap();
//! ```

use core::convert::TryInto;

use crate::rcc::CoreClocks;
use crate::sai::{GetClkSAI, Sai, SaiChannel, INTERFACE};

use crate::pac;

use crate::gpio;
use crate::time::Hertz;

pub trait PdmInstance:
    crate::Sealed
    + core::ops::Deref<Target = pac::sai1::RegisterBlock>
    + GetClkSAI
    + gpio::alt::SaiPdm
{
}

/// Trait for a valid combination of SAI PDM pins
pub trait PulseDensityPins<SAI> {
    // It is possible to specifiy less than 8 output microphone
    // channels, in order to reduce the required bit clock and the
    // memory footprint
    const MAX_MICROPHONES: u8 = 8;
    // Enable bitstream clock 1?
    const ENABLE_BITSTREAM_CLOCK_1: bool = false;
    // Enable bitstream clock 2?
    const ENABLE_BITSTREAM_CLOCK_2: bool = false;
    // Enable bitstream clock 3?
    const ENABLE_BITSTREAM_CLOCK_3: bool = false;
    // Enable bitstream clock 4?
    const ENABLE_BITSTREAM_CLOCK_4: bool = false;

    type AltPins;
    fn convert(self) -> Self::AltPins;
}

// Pin sets
impl<SAI, CK1, D1> PulseDensityPins<SAI> for (CK1, D1)
where
    SAI: PdmInstance,
    CK1: Into<SAI::Ck1>,
    D1: Into<SAI::D1>,
{
    const MAX_MICROPHONES: u8 = 2;
    const ENABLE_BITSTREAM_CLOCK_1: bool = true;

    type AltPins = (SAI::Ck1, SAI::D1);
    fn convert(self) -> Self::AltPins {
        (self.0.into(), self.1.into())
    }
}

/// Pulse Density Modulation Interface
pub struct Pdm {
    invalid_countdown: u8,
}
impl INTERFACE for Pdm {}

/// Trait to extend SAI peripherals
pub trait SaiPdmExt: Sized + PdmInstance {
    fn pdm(
        self,
        pins: impl PulseDensityPins<Self>,
        clock: Hertz,
        prec: Self::Rec,
        clocks: &CoreClocks,
    ) -> Sai<Self, Pdm>;
}

impl<SAI: PdmInstance> SaiPdmExt for SAI {
    fn pdm(
        self,
        pins: impl PulseDensityPins<Self>,
        clock: Hertz,
        prec: SAI::Rec,
        clocks: &CoreClocks,
    ) -> Sai<Self, Pdm> {
        Sai::pdm(self, pins, clock, prec, clocks)
    }
}
impl<SAI: PdmInstance> Sai<SAI, Pdm> {
    /// Read a single data word (one 'slot')
    pub fn read_data(&mut self) -> nb::Result<u32, core::convert::Infallible> {
        while self.interface.invalid_countdown > 0 {
            // Check for words to read
            if self.rb.cha().sr.read().freq().bit_is_clear() {
                return Err(nb::Error::WouldBlock);
            }

            let _ = self.rb.cha().dr.read(); // Flush
            self.interface.invalid_countdown -= 1;
        }

        // Check for words to read
        if self.rb.cha().sr.read().freq().bit_is_clear() {
            return Err(nb::Error::WouldBlock);
        }

        Ok(self.rb.cha().dr.read().bits() & 0xFFFF)
    }

    /// Initialise SAI in PDM mode
    pub fn pdm<PINS>(
        sai: SAI,
        pins: PINS,
        clock: Hertz,
        prec: SAI::Rec,
        clocks: &CoreClocks,
    ) -> Self
    where
        PINS: PulseDensityPins<SAI>,
    {
        let micnbr = match PINS::MAX_MICROPHONES {
            2 => 0, // Up to 2 microphones
            _ => unimplemented!(),
        };
        let frl = (16 * (micnbr + 1)) - 1; // Frame length
        let ds = 0b100; // 16 bits
        let nbslot: u8 = 0; // One slot

        // Calculate bit clock SCK_a
        let sck_a_hz = 2 * clock;

        // Calculate master clock MCLK_a
        let mclk_a_hz = sck_a_hz; // For NODIV = 1, SCK_a = MCLK_a

        // Calculate divider
        let ker_ck_a = SAI::sai_a_ker_ck(&prec, clocks);
        let kernel_clock_divider: u8 =
            (ker_ck_a / mclk_a_hz).try_into().expect(concat!(
                stringify!($SAIX),
                ": Kernel clock is out of range for required MCLK"
            ));

        // Configure SAI peripeheral
        let mut s = Sai {
            rb: sai,
            master_channel: SaiChannel::ChannelA,
            slave_channel: None,
            interface: Pdm {
                // count slots for 2 frames
                invalid_countdown: 2 * (nbslot + 1),
            },
        };
        // RCC enable, reset
        s.sai_rcc_init(prec);

        let _pins = pins.convert();

        // Configure block 1
        let audio_ch_a = &s.rb.cha();

        unsafe {
            audio_ch_a.cr1.modify(|_, w| {
                w.mode().master_rx(); // Master receiver
                w.prtcfg().free();
                w.ds().bits(ds);
                w.lsbfirst().clear_bit(); // MSB first
                w.ckstr().clear_bit(); // Rising edge
                w.mono().stereo(); // Stereo
                w.nodiv().no_div(); // No division from MCLK to SCK
                w.mckdiv().bits(kernel_clock_divider - 1)
            });

            audio_ch_a.frcr.modify(|_, w| {
                w.fsoff().clear_bit();
                w.fspol().set_bit(); // FS active high
                w.fsdef().clear_bit();
                w.fsall().bits(0); // Pulse width = 1 bit clock
                w.frl().bits(frl)
            });

            audio_ch_a.slotr.modify(|_, w| {
                w.fboff().bits(0); // No offset on slot
                w.slotsz().bits(0); // Equal to ACR1.DS
                w.nbslot().bits(nbslot);
                w.sloten().bits(0x1) // Bitfield
            });

            // PDM Control Register
            if PINS::ENABLE_BITSTREAM_CLOCK_1 {
                s.rb.pdmcr.modify(|_, w| {
                    w.cken1().set_bit() // CKEN1
                });
            }
            if PINS::ENABLE_BITSTREAM_CLOCK_2 {
                s.rb.pdmcr.modify(|_, w| {
                    w.cken2().set_bit() // CKEN2
                });
            }
            if PINS::ENABLE_BITSTREAM_CLOCK_3 {
                s.rb.pdmcr.modify(|_, w| {
                    w.cken3().set_bit() // CKEN3
                });
            }
            if PINS::ENABLE_BITSTREAM_CLOCK_4 {
                s.rb.pdmcr.modify(|_, w| {
                    w.cken4().set_bit() // CKEN4
                });
            }
            s.rb.pdmcr.modify(|_, w| {
                w.micnbr().bits(micnbr); // 2, 4, 6 or 8 microphones
                w.pdmen().set_bit() // Enabled
            });
        }

        // Enable SAI_A
        audio_ch_a.cr1.modify(|_, w| w.saien().enabled());

        // SAI
        s
    }
}

impl PdmInstance for pac::SAI1 {}
#[cfg(not(feature = "rm0455"))]
impl PdmInstance for pac::SAI4 {}
