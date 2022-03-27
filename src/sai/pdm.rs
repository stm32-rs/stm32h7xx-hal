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
//! let mut sai = dp.SAI1.pdm((ck1, d1), 1_024.khz(), ccdr.peripheral.SAI1, &ccdr.clocks);
//!
//! let _ = block!(sai.read_data()).unwrap();
//! ```

use core::convert::TryInto;

use crate::rcc::{rec, CoreClocks, ResetEnable};
use crate::sai::{GetClkSAI, Sai, SaiChannel, INTERFACE};

use crate::stm32::SAI1;
#[cfg(not(feature = "rm0455"))]
use crate::stm32::SAI4;

use crate::gpio::{self, Alternate};
use crate::time::Hertz;

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
            gpio::PB2<Alternate<2>>,
            gpio::PC1<Alternate<2>>,
            gpio::PD6<Alternate<2>>,
            gpio::PE6<Alternate<2>>
        ]
        D2: [
            gpio::PE4<Alternate<2>>
        ]
        D3: [
            gpio::PC5<Alternate<2>>,
            gpio::PF10<Alternate<2>>
        ]
        D4: []
        CK1: [
            gpio::PE2<Alternate<2>>
        ]
        CK2: [
            gpio::PE5<Alternate<2>>
        ]
        CK3: []
        CK4: []
}
#[cfg(not(feature = "rm0455"))]
pins! {
    SAI4:
        D1: [
            gpio::PB2<Alternate<10>>,
            gpio::PC1<Alternate<10>>,
            gpio::PD6<Alternate<10>>,
            gpio::PE6<Alternate<9>>
        ]
        D2: [
            gpio::PE4<Alternate<10>>
        ]
        D3: [
            gpio::PC5<Alternate<10>>,
            gpio::PF10<Alternate<10>>
        ]
        D4: []
        CK1: [
            gpio::PE2<Alternate<10>>
        ]
        CK2: [
            gpio::PE5<Alternate<10>>
        ]
        CK3: []
        CK4: []
}

/// Pulse Density Modulation Interface
pub struct Pdm {
    invalid_countdown: u8,
}
impl INTERFACE for Pdm {}

/// Trait to extend SAI peripherals
pub trait SaiPdmExt<SAI>: Sized {
    type Rec: ResetEnable;

    fn pdm<PINS, T>(
        self,
        _pins: PINS,
        clock: T,
        prec: Self::Rec,
        clocks: &CoreClocks,
    ) -> Sai<SAI, Pdm>
    where
        PINS: PulseDensityPins<Self>,
        T: Into<Hertz>;
}

macro_rules! hal {
    ($($SAIX:ident, $Rec:ident: ($pdm_saiX:ident)),+) => {
        $(
            impl SaiPdmExt<$SAIX> for $SAIX {
                type Rec = rec::$Rec;

                fn pdm<PINS, T>(
                    self,
                    _pins: PINS,
                    clock: T,
                    prec: rec::$Rec,
                    clocks: &CoreClocks,
                ) -> Sai<Self, Pdm>
                where
                    PINS: PulseDensityPins<Self>,
                    T: Into<Hertz>,
                {
                    Sai::$pdm_saiX(self, _pins, clock.into(), prec, clocks)
                }
            }
            impl Sai<$SAIX, Pdm> {
                /// Read a single data word (one 'slot')
                pub fn read_data(&mut self) -> nb::Result<u32, core::convert::Infallible> {
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
                    prec: rec::$Rec,
                    clocks: &CoreClocks,
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
                    let ker_ck_a = $SAIX::sai_a_ker_ck(&prec, clocks);
                    let kernel_clock_divider: u8 = (ker_ck_a.0 / mclk_a_hz)
                        .try_into()
                        .expect(concat!(stringify!($SAIX),
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
    SAI1, Sai1: (pdm_sai1)
}
#[cfg(not(feature = "rm0455"))]
hal! {
    SAI4, Sai4: (pdm_sai4)
}
