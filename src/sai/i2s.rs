//! # Serial Audio Interface - Inter-IC Sound
//!
//! Inter-IC Sound.
//!
use core::convert::TryInto;

use crate::rcc::{rec, CoreClocks, ResetEnable};
use crate::sai::{GetClkSAI, Sai, SaiChannel, INTERFACE};
use crate::stm32;
use crate::stm32::{SAI1, SAI2, SAI3, SAI4};
use crate::time::Hertz;

use crate::device::sai4::ch::sr;
use crate::gpio::gpioa::{PA0, PA1, PA12, PA2};
use crate::gpio::gpiob::PB2;
use crate::gpio::gpioc::{PC0, PC1};
use crate::gpio::gpiod::{
    PD0, PD1, PD10, PD11, PD12, PD13, PD14, PD15, PD4, PD6, PD8, PD9,
};
use crate::gpio::gpioe::{
    PE0, PE11, PE12, PE13, PE14, PE2, PE3, PE4, PE5, PE6,
};
use crate::gpio::gpiof::{PF11, PF6, PF7, PF8, PF9};
use crate::gpio::gpiog::{PG10, PG7, PG9};
use crate::gpio::gpioh::{PH2, PH3};
use crate::gpio::gpioi::{PI4, PI5, PI6, PI7};
use crate::gpio::{Alternate, AF10, AF6, AF8};
use stm32h7::Variant::Val;

use embedded_hal::i2s::FullDuplex;

const NUM_SLOTS: u8 = 16;

#[derive(Clone, Copy, PartialEq)]
pub enum I2SMode {
    Master = 0b00,
    Slave = 0b10,
}

#[derive(Copy, Clone, PartialEq)]
pub enum I2SDir {
    Tx = 0b00,
    Rx = 0b01,
}

#[derive(Copy, Clone)]
pub enum I2SDataSize {
    BITS_8 = 0b001,
    BITS_10 = 0b010,
    BITS_16 = 0b100,
    BITS_20 = 0b101,
    BITS_24 = 0b110,
    BITS_32 = 0b111,
}

#[derive(Copy, Clone)]
enum I2SSlotSize {
    BITS_16 = 0b01,
    BITS_32 = 0b10,
}

#[derive(Copy, Clone)]
pub enum I2SProtocol {
    LSB,
    MSB,
}

#[derive(Copy, Clone)]
pub enum I2SSync {
    Master = 0b00,
    Internal = 0b01,
    External = 0b10,
}

#[derive(Copy, Clone, Debug)]
pub enum I2SError {
    NoChannelAvailable,
}

#[derive(Copy, Clone)]
pub enum I2SOverSampling {
    Enabled = 2,
    Disabled = 1,
}

pub enum I2SClockStrobe {
    Rising,
    Falling,
}

pub trait I2SPinsChA<SAI> {}
pub trait I2SPinsChB<SAI> {}
pub trait I2SPinMclkA<SAI> {}
pub trait I2SPinMclkB<SAI> {}
pub trait I2SPinSckA<SAI> {}
pub trait I2SPinSckB<SAI> {}
pub trait I2SPinFsA<SAI> {}
pub trait I2SPinFsB<SAI> {}
pub trait I2SPinSdA<SAI> {}
pub trait I2SPinSdB<SAI> {}

impl<SAI, MCLK, SCK, FS, SD1, SD2> I2SPinsChA<SAI>
    for (MCLK, SCK, FS, SD1, Option<SD2>)
where
    MCLK: I2SPinMclkA<SAI>,
    SCK: I2SPinSckA<SAI>,
    FS: I2SPinFsA<SAI>,
    SD1: I2SPinSdA<SAI>,
    SD2: I2SPinSdB<SAI>,
{
}

impl<SAI, MCLK, SCK, FS, SD1, SD2> I2SPinsChB<SAI>
    for (MCLK, SCK, FS, SD1, Option<SD2>)
where
    MCLK: I2SPinMclkB<SAI>,
    SCK: I2SPinSckB<SAI>,
    FS: I2SPinFsB<SAI>,
    SD1: I2SPinSdB<SAI>,
    SD2: I2SPinSdA<SAI>,
{
}

pub struct I2SChanConfig {
    dir: I2SDir,
    sync_type: I2SSync,
    clock_strobe: I2SClockStrobe,
    slots: u8,
    first_bit_offset: u8,
}

impl I2SChanConfig {
    pub fn new(
        dir: I2SDir,
        sync_type: I2SSync,
        clock_strobe: I2SClockStrobe,
        slots: u8,
        first_bit_offset: u8,
    ) -> I2SChanConfig {
        I2SChanConfig {
            dir,
            sync_type,
            clock_strobe,
            slots,
            first_bit_offset,
        }
    }
}

/// I2S Interface
pub struct I2S {
    master: I2SChanConfig,
    slave: Option<I2SChanConfig>,
}
impl INTERFACE for I2S {}

/// Trait to extend SAI periperhals
pub trait SaiI2sExt<SAI>: Sized {
    type Rec: ResetEnable;
    fn i2s_ch_a<PINS, T>(
        self,
        _pins: PINS,
        audio_freq: T,
        bit_rate: I2SDataSize,
        prec: Self::Rec,
        clocks: &CoreClocks,
        over_sampling: I2SOverSampling,
        master: I2SChanConfig,
        slave: Option<I2SChanConfig>,
    ) -> Sai<SAI, I2S>
    where
        PINS: I2SPinsChA<Self>,
        T: Into<Hertz>;
    fn i2s_ch_b<PINS, T>(
        self,
        _pins: PINS,
        audio_freq: T,
        bit_rate: I2SDataSize,
        prec: Self::Rec,
        clocks: &CoreClocks,
        over_sampling: I2SOverSampling,
        master: I2SChanConfig,
        slave: Option<I2SChanConfig>,
    ) -> Sai<SAI, I2S>
    where
        PINS: I2SPinsChB<Self>,
        T: Into<Hertz>;
}

macro_rules! i2s {
    ( $($SAIX:ident, $Rec:ident: [$i2s_saiX_ch_a:ident, $i2s_saiX_ch_b:ident]),+ ) => {
        $(
            impl SaiI2sExt<$SAIX> for $SAIX {
                type Rec = rec::$Rec;
                fn i2s_ch_a<PINS, T>(
                    self,
                    _pins: PINS,
                    audio_freq: T,
                    bit_rate: I2SDataSize,
                    prec: rec::$Rec,
                    clocks: &CoreClocks,
                    over_sampling: I2SOverSampling,
                    master: I2SChanConfig,
                    slave: Option<I2SChanConfig>,
                ) -> Sai<Self, I2S>
                where
                    PINS: I2SPinsChA<Self>,
                    T: Into<Hertz>,
                {
                    Sai::$i2s_saiX_ch_a(
                        self,
                        _pins,
                        audio_freq.into(),
                        bit_rate,
                        prec,
                        clocks,
                        over_sampling,
                        master,
                        slave,
                    )
                }
                fn i2s_ch_b<PINS, T>(
                    self,
                    _pins: PINS,
                    audio_freq: T,
                    bit_rate: I2SDataSize,
                    prec: rec::$Rec,
                    clocks: &CoreClocks,
                    over_sampling: I2SOverSampling,
                    master: I2SChanConfig,
                    slave: Option<I2SChanConfig>,
                ) -> Sai<Self, I2S>
                where
                    PINS: I2SPinsChB<Self>,
                    T: Into<Hertz>,
                {
                    Sai::$i2s_saiX_ch_b(
                        self,
                        _pins,
                        audio_freq.into(),
                        bit_rate,
                        prec,
                        clocks,
                        over_sampling,
                        master,
                        slave,
                    )
                }
            }

            impl Sai<$SAIX, I2S> {
                pub fn $i2s_saiX_ch_a<PINS>(
                    sai: $SAIX,
                    _pins: PINS,
                    audio_freq: Hertz,
                    bit_rate: I2SDataSize,
                    prec: rec::$Rec,
                    clocks: &CoreClocks,
                    over_sampling: I2SOverSampling,
                    master: I2SChanConfig,
                    slave: Option<I2SChanConfig>,
                ) -> Self
                where
                    PINS: I2SPinsChA<$SAIX>,
                {
                    assert!(master.slots <= NUM_SLOTS);
                    if let Some(slave) = &slave {
                        assert!(slave.slots <= NUM_SLOTS);
                    }

                    // Clock config
                    let ker_ck_a = $SAIX::sai_a_ker_ck(&prec, clocks)
                        .expect("SAI kernel clock must run!");
                    let mclk_div =
                        (ker_ck_a.0) / (audio_freq.0 * (over_sampling as u32) * 256);
                    let mclk_div: u8 = mclk_div
                        .try_into()
                        .expect("SAI kernel clock is out of range for required MCLK");

                    // Configure SAI peripeheral
                    let mut per_sai = Sai {
                        rb: sai,
                        master_channel: SaiChannel::ChannelA,
                        slave_channel: if slave.is_some() {
                            Some(SaiChannel::ChannelB)
                        } else {
                            None
                        },
                        interface: I2S { master, slave },
                    };

                    per_sai.sai_rcc_init(prec);

                    i2s_config_channel(
                        &per_sai.rb.cha,
                        I2SMode::Master,
                        &per_sai.interface.master,
                        mclk_div,
                        bit_rate,
                    );

                    if let Some(slave) = &per_sai.interface.slave {
                        i2s_config_channel(
                            &per_sai.rb.chb,
                            I2SMode::Slave,
                            slave,
                            0,
                            bit_rate,
                        );
                    }

                    per_sai
                }

                pub fn $i2s_saiX_ch_b<PINS>(
                    sai: $SAIX,
                    _pins: PINS,
                    audio_freq: Hertz,
                    bit_rate: I2SDataSize,
                    prec: rec::$Rec,
                    clocks: &CoreClocks,
                    over_sampling: I2SOverSampling,
                    master: I2SChanConfig,
                    slave: Option<I2SChanConfig>,
                ) -> Self
                where
                    PINS: I2SPinsChB<$SAIX>,
                {
                    assert!(master.slots <= NUM_SLOTS);
                    if let Some(slave) = &slave {
                        assert!(slave.slots <= NUM_SLOTS);
                    }

                    // Clock config
                    let ker_ck_a = $SAIX::sai_b_ker_ck(&prec, clocks)
                        .expect("SAI kernel clock must run!");
                    let mclk_div =
                        (ker_ck_a.0) / (audio_freq.0 * (over_sampling as u32) * 256);
                    let mclk_div: u8 = mclk_div
                        .try_into()
                        .expect("SAI kernel clock is out of range for required MCLK");

                    // Configure SAI peripeheral
                    let mut per_sai = Sai {
                        rb: sai,
                        master_channel: SaiChannel::ChannelB,
                        slave_channel: if slave.is_some() {
                            Some(SaiChannel::ChannelA)
                        } else {
                            None
                        },
                        interface: I2S { master, slave },
                    };

                    per_sai.sai_rcc_init(prec);

                    i2s_config_channel(
                        &per_sai.rb.chb,
                        I2SMode::Master,
                        &per_sai.interface.master,
                        mclk_div,
                        bit_rate,
                    );

                    if let Some(slave) = &per_sai.interface.slave {
                        i2s_config_channel(
                            &per_sai.rb.cha,
                            I2SMode::Slave,
                            slave,
                            0,
                            bit_rate,
                        );
                    }

                    per_sai
                }

                pub fn enable(&mut self) {
                    // Enable slave first "recommended" per ref doc
                    self.slave_channel(enable_ch);
                    self.master_channel(enable_ch);
                }

                pub fn disable(&mut self) {
                    // Master must be disabled first
                    self.master_channel(|ch| ch.cr1.modify(|_, w| w.saien().disabled()));
                    self.slave_channel( |ch| ch.cr1.modify(|_, w| w.saien().disabled()));
                }
            }

            impl FullDuplex<u32> for Sai<$SAIX, I2S> {
                type Error = I2SError;

                fn try_read(&mut self) -> nb::Result<(u32, u32), Self::Error> {
                    if self.interface.master.dir == I2SDir::Rx {
                        return self.master_channel(read);
                    } else if let Some(slave) = &self.interface.slave {
                        if slave.dir == I2SDir::Rx {
                            return self.slave_channel(read).unwrap();
                        }
                    }
                    Err(nb::Error::Other(I2SError::NoChannelAvailable))
                }

                fn try_send(
                    &mut self,
                    left_word: u32,
                    right_word: u32,
                ) -> nb::Result<(), Self::Error> {
                    if self.interface.master.dir == I2SDir::Tx {
                        return self.master_channel(|audio_ch| {
                            send(left_word, right_word, audio_ch)
                        });
                    } else if let Some(slave) = &self.interface.slave {
                        if slave.dir == I2SDir::Tx {
                            return self
                                .slave_channel(|audio_ch| {
                                    send(left_word, right_word, audio_ch)
                                })
                                .unwrap();
                        }
                    }
                    Err(nb::Error::Other(I2SError::NoChannelAvailable))
                }
            }
        )+
    }
}

i2s! {
    SAI2, Sai2: [i2s_sai1_ch_a, i2s_sai1_ch_b],
    SAI3, Sai3: [i2s_sai2_ch_a, i2s_sai2_ch_b],
    SAI1, Sai1: [i2s_sai3_ch_a, i2s_sai3_ch_b],
    SAI4, Sai4: [i2s_sai4_ch_a, i2s_sai4_ch_b]
}

fn i2s_config_channel(
    audio_ch: &stm32::sai4::CH,
    mode: I2SMode,
    config: &I2SChanConfig,
    mclk_div: u8,
    bit_rate: I2SDataSize,
) {
    let clock_strobe = match config.clock_strobe {
        I2SClockStrobe::Rising => false,
        I2SClockStrobe::Falling => true,
    };

    // 16 bits in register correspond to 1 slot each max 16 slots
    let slot_en_bits: u16 = (2_u32.pow(config.slots.into()) - 1) as u16;

    let (frame_length, slot_size) = match bit_rate {
        I2SDataSize::BITS_8 => (16 * (config.slots / 2), I2SSlotSize::BITS_16),
        I2SDataSize::BITS_10 => (32 * (config.slots / 2), I2SSlotSize::BITS_16),
        I2SDataSize::BITS_16 => (32 * (config.slots / 2), I2SSlotSize::BITS_16),
        I2SDataSize::BITS_20 => (64 * (config.slots / 2), I2SSlotSize::BITS_32),
        I2SDataSize::BITS_24 => (64 * (config.slots / 2), I2SSlotSize::BITS_32),
        I2SDataSize::BITS_32 => (64 * (config.slots / 2), I2SSlotSize::BITS_32),
    };

    let mode_bits = (mode as u8) | (config.dir as u8);
    unsafe {
        audio_ch.cr1.modify(|_, w| {
            w.mode()
                .bits(mode_bits as u8)
                .prtcfg()
                .free()
                .ds()
                .bits(bit_rate as u8)
                .lsbfirst()
                .msb_first()
                .ckstr()
                .bit(clock_strobe)
                .syncen()
                .bits(config.sync_type as u8)
                .mono()
                .stereo()
                .nodiv()
                .master_clock()
                .mckdiv()
                .bits(mclk_div)
        });
        audio_ch.cr2.modify(|_, w| {
            w.fth()
                .quarter1()
                .tris()
                .clear_bit()
                .mute()
                .clear_bit()
                .muteval()
                .clear_bit()
                .muteval()
                .clear_bit()
                .cpl()
                .clear_bit()
                // CPL, unused with companding off
                .comp()
                .no_companding()
        });
        audio_ch.frcr.modify(|_, w| {
            w.frl()
                .bits(frame_length - 1)
                .fsall()
                .bits((frame_length / 2) - 1)
                .fsdef()
                .set_bit() // left/right channels enabled
                .fspol()
                .set_bit() // FS active high
                .fsoff()
                .clear_bit()
        });
        audio_ch.slotr.modify(|_, w| {
            w.fboff()
                .bits(config.first_bit_offset)
                .slotsz()
                .bits(slot_size as u8)
                .nbslot()
                .bits(config.slots - 1)
                .sloten()
                .bits(slot_en_bits)
        });
        match mode {
            I2SMode::Master => audio_ch
                .im
                .modify(|_, w| w.ovrudrie().enabled().wckcfgie().enabled()),
            I2SMode::Slave => audio_ch.im.modify(|_, w| {
                w.ovrudrie()
                    .enabled()
                    .afsdetie()
                    .enabled()
                    .lfsdetie()
                    .enabled()
                    .freqie()
                    .enabled()
            }),
        }
    }
}

fn enable_ch(audio_ch: &stm32::sai4::CH) {
    audio_ch.clrfr.write(|w| {
        w.cafsdet()
            .clear()
            .ccnrdy()
            .clear()
            .clfsdet()
            .clear()
            .cmutedet()
            .clear()
            .covrudr()
            .clear()
            .cwckcfg()
            .clear()
    });
    audio_ch.cr2.modify(|_, w| w.fflush().flush());
    audio_ch.cr1.modify(|_, w| w.saien().enabled());
}

fn read(audio_ch: &stm32::sai4::CH) -> nb::Result<(u32, u32), I2SError> {
    match audio_ch.sr.read().flvl().variant() {
        Val(sr::FLVL_A::EMPTY) => Err(nb::Error::WouldBlock),
        _ => Ok((audio_ch.dr.read().bits(), audio_ch.dr.read().bits())),
    }
}

fn send(
    left_word: u32,
    right_word: u32,
    audio_ch: &stm32::sai4::CH,
) -> nb::Result<(), I2SError> {
    // The FIFO is 8 words long. A write consists of 2 words, in stereo mode.
    // Therefore you need to wait for 3/4s to ensure 2 words are available for writing.
    match audio_ch.sr.read().flvl().variant() {
        Val(sr::FLVL_A::FULL) => Err(nb::Error::WouldBlock),
        Val(sr::FLVL_A::QUARTER4) => Err(nb::Error::WouldBlock),
        _ => {
            unsafe {
                audio_ch.dr.write(|w| w.bits(left_word).bits(right_word));
            }
            Ok(())
        }
    }
}

// Pin definitions
macro_rules! pins {
    ($($SAIX:ty:
        MCLK_A:  [$($MCLK_A:ty),*]
        SCK_A:   [$($SCK_A:ty),*]
        FS_A:    [$($FS_A:ty),*]
        SD_A:    [$($SD_A:ty),*]
        MCLK_B:  [$($MCLK_B:ty),*]
        SCK_B:   [$($SCK_B:ty),*]
        FS_B:    [$($FS_B:ty),*]
        SD_B:    [$($SD_B:ty),*]

     )+) => {
         $(
            $(
                impl I2SPinMclkA<$SAIX> for $MCLK_A {}
            )*
            $(
                impl I2SPinSckA<$SAIX> for $SCK_A {}
            )*
            $(
                impl I2SPinFsA<$SAIX> for $FS_A {}
            )*
            $(
                impl I2SPinSdA<$SAIX> for $SD_A {}
            )*
            $(
                impl I2SPinMclkB<$SAIX> for $MCLK_B {}
            )*
            $(
                impl I2SPinSckB<$SAIX> for $SCK_B {}
            )*
            $(
                impl I2SPinFsB<$SAIX> for $FS_B {}
            )*
            $(
                impl I2SPinSdB<$SAIX> for $SD_B {}
            )*
         )+
     }
}

pins! {
    SAI1:
        MCLK_A: [
            PE2<Alternate<AF6>>,
            PG7<Alternate<AF6>>
        ]
        SCK_A: [
            PE5<Alternate<AF6>>
        ]
        FS_A: [
            PE4<Alternate<AF6>>
        ]
        SD_A: [
            PB2<Alternate<AF6>>,
            PC1<Alternate<AF6>>,
            PD6<Alternate<AF6>>,
            PE6<Alternate<AF6>>
        ]
        MCLK_B: [
            PF7<Alternate<AF6>>
        ]
        SCK_B: [
            PF8<Alternate<AF6>>
        ]
        FS_B: [
            PF9<Alternate<AF6>>
        ]
        SD_B: [
            PE3<Alternate<AF6>>,
            PF6<Alternate<AF6>>
        ]
    SAI2:
        MCLK_A: [
            PE0<Alternate<AF10>>,
            PI4<Alternate<AF10>>
        ]
        SCK_A: [
            PD13<Alternate<AF10>>,
            PI5<Alternate<AF10>>
        ]
        FS_A: [
            PD12<Alternate<AF10>>,
            PI7<Alternate<AF10>>
        ]
        SD_A: [
            PD11<Alternate<AF10>>,
            PI6<Alternate<AF10>>
        ]
        MCLK_B: [
            PA1<Alternate<AF10>>,
            PE6<Alternate<AF10>>,
            PE14<Alternate<AF10>>,
            PH3<Alternate<AF10>>
        ]
        SCK_B: [
            PA2<Alternate<AF8>>,
            PE12<Alternate<AF10>>,
            PH2<Alternate<AF10>>
        ]
        FS_B: [
            PA12<Alternate<AF8>>,
            PC0<Alternate<AF8>>,
            PE13<Alternate<AF10>>,
            PG9<Alternate<AF10>>
        ]
        SD_B: [
            PA0<Alternate<AF10>>,
            PE11<Alternate<AF10>>,
            PF11<Alternate<AF10>>,
            PG10<Alternate<AF10>>
        ]
    SAI3:
        MCLK_A: [
            PD15<Alternate<AF6>>
        ]
        SCK_A: [
            PD0<Alternate<AF6>>
        ]
        FS_A: [
            PD4<Alternate<AF6>>
        ]
        SD_A: [
            PD1<Alternate<AF6>>
        ]
        MCLK_B: [
            PD14<Alternate<AF6>>
        ]
        SCK_B: [
            PD8<Alternate<AF6>>
        ]
        FS_B: [
            PD10<Alternate<AF6>>
        ]
        SD_B: [
            PD9<Alternate<AF6>>
        ]
    SAI4:
        MCLK_A: [
            PE2<Alternate<AF8>>
        ]
        SCK_A: [
            PE5<Alternate<AF8>>
        ]
        FS_A: [
            PE4<Alternate<AF8>>
        ]
        SD_A: [
            PB2<Alternate<AF8>>,
            PC1<Alternate<AF8>>,
            PD6<Alternate<AF8>>,
            PE6<Alternate<AF8>>
        ]
        MCLK_B: [
            PF7<Alternate<AF8>>
        ]
        SCK_B: [
            PF8<Alternate<AF8>>
        ]
        FS_B: [
            PF9<Alternate<AF8>>
        ]
        SD_B: [
            PE3<Alternate<AF8>>,
            PF6<Alternate<AF8>>
        ]
}
