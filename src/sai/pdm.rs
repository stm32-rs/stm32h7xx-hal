//! # Serial Audio Interface - Pulse Density Modulation
//!
//! Pulse Density Modulation. Currently only a single channel (CK1,
//! D1) is implemented, but more could be added once they've been
//! tested.
//!
//! ```
//! let d1 = gpioc.pc1.into_alternate_af2();
//! let ck1 = gpioe.pe2.into_alternate_af2();
//! let pins = (ck1, d1);
//!
//! // Configure SAI for PDM mode
//! let mut sai = dp.SAI1.pdm((ck1, d1), 1_024.khz(), &mut ccdr);
//!
//! let _ = block!(sai.read_data()).unwrap();
//! ```

use core::convert::TryInto;

use crate::rcc::Ccdr;
use crate::sai::{GetClkSAI, Sai, SaiChannel, INTERFACE};
use crate::stm32::{SAI1, SAI4};
use crate::time::Hertz;

use crate::Never;

use crate::gpio::gpiob::PB2;
use crate::gpio::gpioc::{PC1, PC5};
use crate::gpio::gpiod::PD6;
use crate::gpio::gpioe::{PE2, PE4, PE5, PE6};
use crate::gpio::gpiof::PF10;
use crate::gpio::{Alternate, AF10, AF2, AF9};

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
}
pub trait PulseDensityPinD1<SAI> {}
pub trait PulseDensityPinD2<SAI> {}
pub trait PulseDensityPinD3<SAI> {}
pub trait PulseDensityPinD4<SAI> {}
pub trait PulseDensityPinCK1<SAI> {}
pub trait PulseDensityPinCK2<SAI> {}
pub trait PulseDensityPinCK3<SAI> {}
pub trait PulseDensityPinCK4<SAI> {}

// Pin sets
impl<SAI, CK1, D1> PulseDensityPins<SAI> for (CK1, D1)
where
    CK1: PulseDensityPinCK1<SAI>,
    D1: PulseDensityPinD1<SAI>,
{
    const MAX_MICROPHONES: u8 = 2;
    const ENABLE_BITSTREAM_CLOCK_1: bool = true;
}

// Pin definitions
macro_rules! pins {
    ($($SAIX:ty:
       D1: [$($D1:ty),*] D2: [$($D2:ty),*]
       D3: [$($D3:ty),*] D4: [$($D4:ty),*]
       CK1: [$($CK1:ty),*] CK2: [$($CK2:ty),*]
       CK3: [$($CK3:ty),*] CK4: [$($CK4:ty),*]
    )+) => {
        $(
            $(
                impl PulseDensityPinD1<$SAIX> for $D1 {}
            )*
            $(
                impl PulseDensityPinD2<$SAIX> for $D2 {}
            )*
            $(
                impl PulseDensityPinD3<$SAIX> for $D3 {}
            )*
            $(
                impl PulseDensityPinD4<$SAIX> for $D4 {}
            )*
            $(
                impl PulseDensityPinCK1<$SAIX> for $CK1 {}
            )*
            $(
                impl PulseDensityPinCK2<$SAIX> for $CK2 {}
            )*
            $(
                impl PulseDensityPinCK3<$SAIX> for $CK3 {}
            )*
            $(
                impl PulseDensityPinCK4<$SAIX> for $CK4 {}
            )*
        )+
    }
}
// Pin definitions on STM32743ZI, perhaps there are more on
// newer/larger parts?
pins! {
    SAI1:
        D1: [
            PB2<Alternate<AF2>>,
            PC1<Alternate<AF2>>,
            PD6<Alternate<AF2>>,
            PE6<Alternate<AF2>>
        ]
        D2: [
            PE4<Alternate<AF2>>
        ]
        D3: [
            PC5<Alternate<AF2>>,
            PF10<Alternate<AF2>>
        ]
        D4: []
        CK1: [
            PE2<Alternate<AF2>>
        ]
        CK2: [
            PE5<Alternate<AF2>>
        ]
        CK3: []
        CK4: []
    SAI4:
        D1: [
            PB2<Alternate<AF10>>,
            PC1<Alternate<AF10>>,
            PD6<Alternate<AF10>>,
            PE6<Alternate<AF9>>
        ]
        D2: [
            PE4<Alternate<AF10>>
        ]
        D3: [
            PC5<Alternate<AF10>>,
            PF10<Alternate<AF10>>
        ]
        D4: []
        CK1: [
            PE2<Alternate<AF10>>
        ]
        CK2: [
            PE5<Alternate<AF10>>
        ]
        CK3: []
        CK4: []
}

/// Pulse Density Modulation Interface
pub struct PDM {
    invalid_countdown: u8,
}
impl INTERFACE for PDM {}

/// Trait to extend SAI periperhals
pub trait SaiPdmExt<SAI>: Sized {
    fn pdm<PINS, T>(
        self,
        _pins: PINS,
        clock: T,
        ccdr: &mut Ccdr,
    ) -> Sai<SAI, PDM>
    where
        PINS: PulseDensityPins<Self>,
        T: Into<Hertz>;
}

macro_rules! hal {
    ($($SAIX:ident: ($pdm_saiX:ident)),+) => {
        $(
            impl SaiPdmExt<$SAIX> for $SAIX {
                fn pdm<PINS, T>(
                    self,
                    _pins: PINS,
                    clock: T,
                    ccdr: &mut Ccdr,
                ) -> Sai<Self, PDM>
                where
                    PINS: PulseDensityPins<Self>,
                    T: Into<Hertz>,
                {
                    Sai::$pdm_saiX(self, _pins, clock.into(), ccdr)
                }
            }
            impl Sai<$SAIX, PDM> {
                /// Read a single data word (one 'slot')
                pub fn read_data(&mut self) -> nb::Result<u32, Never> {
                    while self.interface.invalid_countdown > 0 {
                        // Check for words to read
                        if self.rb.cha.sr.read().freq().bit_is_clear() {
                            return Err(nb::Error::WouldBlock);
                        }

                        let _ = self.rb.cha.dr.read(); // Flush
                        self.interface.invalid_countdown -= 1;
                    }

                    // Check for words to read
                    if self.rb.cha.sr.read().freq().bit_is_clear() {
                        return Err(nb::Error::WouldBlock);
                    }

                    Ok(self.rb.cha.dr.read().bits() & 0xFFFF)
                }

                /// Initialise SAI in PDM mode
                pub fn $pdm_saiX<PINS>(
                    sai: $SAIX,
                    _pins: PINS,
                    clock: Hertz,
                    ccdr: &mut Ccdr,
                ) -> Self
                where
                    PINS: PulseDensityPins<$SAIX>,
                {
                    let micnbr = match PINS::MAX_MICROPHONES {
                        2 => 0, // Up to 2 microphones
                        _ => unimplemented!(),
                    };
                    let frl = (16 * (micnbr + 1)) - 1; // Frame length
                    let ds = 0b100; // 16 bits
                    let nbslot: u8 = 0; // One slot

                    // Calculate bit clock SCK_a
                    let sck_a_hz = 2 * clock.0;

                    // Calculate master clock MCLK_a
                    let mclk_a_hz = sck_a_hz; // For NODIV = 1, SCK_a = MCLK_a

                    // Calculate divider
                    let ker_ck_a =
                        $SAIX::sai_a_ker_ck(&ccdr).expect("SAI kernel clock must run!");
                    let kernel_clock_divider: u8 = (ker_ck_a.0 / mclk_a_hz)
                        .try_into()
                        .expect("SAI kernel clock is out of range for required MCLK");

                    // Configure SAI peripeheral
                    let mut s = Sai {
                        rb: sai,
                        master_channel: SaiChannel::ChannelA,
                        slave_channel: None,
                        interface: PDM {
                            // count slots for 2 frames
                            invalid_countdown: 2 * (nbslot + 1),
                        },
                    };
                    // RCC enable, reset
                    s.sai_rcc_init(ccdr);

                    // Configure block 1
                    let audio_ch_a = &s.rb.cha;

                    unsafe {
                        audio_ch_a.cr1.modify(|_, w| {
                            w.mode()
                                .master_rx() // Master receiver
                                .prtcfg()
                                .free()
                                .ds()
                                .bits(ds)
                                .lsbfirst()
                                .clear_bit() // MSB first
                                .ckstr()
                                .clear_bit() // Rising edge
                                .mono()
                                .stereo() // Stereo
                                .nodiv()
                                .no_div() // No division from MCLK to SCK
                                .mckdiv()
                                .bits(kernel_clock_divider - 1)
                        });

                        audio_ch_a.frcr.modify(|_, w| {
                            w.fsoff()
                                .clear_bit()
                                .fspol()
                                .set_bit() // FS active high
                                .fsdef()
                                .clear_bit()
                                .fsall()
                                .bits(0) // Pulse width = 1 bit clock
                                .frl()
                                .bits(frl)
                        });

                        audio_ch_a.slotr.modify(|_, w| {
                            w.fboff()
                                .bits(0) // No offset on slot
                                .slotsz()
                                .bits(0) // Equal to ACR1.DS
                                .nbslot()
                                .bits(nbslot)
                                .sloten()
                                .bits(0x1) // Bitfield
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
                            w.micnbr()
                                .bits(micnbr) // 2, 4, 6 or 8 microphones
                                .pdmen()
                                .set_bit() // Enabled
                        });
                    }

                    // Enable SAI_A
                    audio_ch_a.cr1.modify(|_, w| w.saien().enabled());

                    // SAI
                    s
                }
            }
        )+
    }
}

hal! {
    SAI1: (pdm_sai1),
    SAI4: (pdm_sai4)
}
