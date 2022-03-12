//! # Serial Audio Interface - Inter-IC Sound
//!
//! Inter-IC Sound.
//!
use core::convert::TryInto;

use crate::rcc::{rec, CoreClocks, ResetEnable};
use crate::sai::{GetClkSAI, Sai, SaiChannel, CLEAR_ALL_FLAGS_BITS, INTERFACE};
use crate::stm32;
use crate::time::Hertz;

use crate::stm32::SAI1;
#[cfg(feature = "rm0455")]
use crate::stm32::SAI2;
#[cfg(feature = "rm0468")]
use crate::stm32::SAI4;
#[cfg(any(feature = "rm0433", feature = "rm0399"))]
use crate::stm32::{SAI2, SAI3, SAI4};

#[cfg(any(feature = "rm0455", feature = "rm0468"))]
use crate::device::sai1::ch::sr;
#[cfg(any(feature = "rm0433", feature = "rm0399"))]
use crate::device::sai4::ch::sr;

#[cfg(any(feature = "rm0455", feature = "rm0468"))]
type CH = stm32::sai1::CH;
#[cfg(any(feature = "rm0433", feature = "rm0399"))]
type CH = stm32::sai4::CH;

use crate::gpio::{self, Alternate};

use crate::traits::i2s::FullDuplex;
// use embedded_hal::i2s::FullDuplex;

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

#[derive(Copy, Clone, PartialEq)]
pub enum I2SProtocol {
    MSB,
    LSB,
}

#[derive(Copy, Clone)]
pub enum I2SSync {
    Master = 0b00,
    Internal = 0b01,
    External = 0b10,
}

#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum I2SError {
    NoChannelAvailable,
}

pub enum I2SClockStrobe {
    Rising,
    Falling,
}

#[derive(Copy, Clone)]
pub enum I2SCompanding {
    Disabled = 0b00,
    // Reserved = 0b01,
    ALaw = 0b10,
    MuLaw = 0b11,
}

#[derive(Copy, Clone)]
pub enum I2SComplement {
    Ones = 0,
    Twos = 1,
}

impl From<I2SComplement> for bool {
    fn from(value: I2SComplement) -> Self {
        match value {
            I2SComplement::Ones => false,
            I2SComplement::Twos => true,
        }
    }
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

/// Trait for valid combination of SAIxA pins
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

/// Trait for valid combination of SAIxB pins
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

/// I2S Config builder
pub struct I2SChanConfig {
    dir: I2SDir,
    sync_type: I2SSync,
    clock_strobe: I2SClockStrobe,
    slots: u8,
    first_bit_offset: u8,
    frame_sync_before: bool,
    frame_sync_active_high: bool,
    oversampling: bool,
    master_clock_disabled: bool,
    companding: I2SCompanding,
    complement: I2SComplement,
    protocol: I2SProtocol,
    mono_mode: bool,
    mute_repeat: bool,
    mute_counter: u8,
    tristate: bool,
    frame_size: Option<u8>,
}

impl I2SChanConfig {
    /// Create a default configuration for the I2S channel
    ///
    /// Arguments:
    /// * `dir` - The direction of data (Rx or Tx)
    pub fn new(dir: I2SDir) -> I2SChanConfig {
        I2SChanConfig {
            dir,
            sync_type: I2SSync::Master,
            clock_strobe: I2SClockStrobe::Falling,
            slots: 2,
            first_bit_offset: 0,
            frame_sync_before: false,
            frame_sync_active_high: false,
            oversampling: false,
            master_clock_disabled: false,
            companding: I2SCompanding::Disabled,
            complement: I2SComplement::Ones,
            protocol: I2SProtocol::MSB,
            mono_mode: false,
            mute_repeat: false,
            mute_counter: 0,
            tristate: false,
            frame_size: None,
        }
    }

    /// Set synchronization type, defaults to Master
    pub fn set_sync_type(mut self, sync_type: I2SSync) -> Self {
        self.sync_type = sync_type;
        self
    }

    /// Set the clock strobing edge
    pub fn set_clock_strobe(mut self, clock_strobe: I2SClockStrobe) -> Self {
        self.clock_strobe = clock_strobe;
        self
    }

    /// Set the number of slots, this is an advanced configuration.
    pub fn set_slots(mut self, slots: u8) -> Self {
        assert!(slots <= 16);
        self.slots = slots;
        self
    }

    /// Set the offset of the first data bit
    pub fn set_first_bit_offset(mut self, first_bit_offset: u8) -> Self {
        // 5 bits or less
        assert!(first_bit_offset < 0b10_0000);
        self.first_bit_offset = first_bit_offset;
        self
    }

    ///  Sets when the frame sync is asserted
    pub fn set_frame_sync_before(mut self, frame_sync_before: bool) -> Self {
        self.frame_sync_before = frame_sync_before;
        self
    }

    /// Set frame sync to active high, defaults to active low
    pub fn set_frame_sync_active_high(
        mut self,
        frame_sync_active_high: bool,
    ) -> Self {
        self.frame_sync_active_high = frame_sync_active_high;
        self
    }

    /// Enable oversampling
    ///
    /// Note: the clock frequency must be doubled when enabled
    pub fn set_oversampling(mut self, oversampling: bool) -> Self {
        self.oversampling = oversampling;
        self
    }

    /// Disable master clock generator
    pub fn disable_master_clock(mut self) -> Self {
        self.master_clock_disabled = true;
        self
    }

    /// Sets the protocol to MSB or LSB
    pub fn set_protocol(mut self, protocol: I2SProtocol) -> Self {
        self.protocol = protocol;
        self
    }

    /// Set the mono mode bit, default is to use stereo
    pub fn set_mono_mode(mut self, mono_mode: bool) -> Self {
        self.mono_mode = mono_mode;
        self
    }

    /// Sets the type of mute values
    /// * false - transmit 0
    /// * true - repeat the last values
    ///
    /// Only meaningful in Tx mode when slots <= 2 and mute is enabled
    pub fn set_mute_repeat(mut self, mute_repeat: bool) -> Self {
        if self.dir == I2SDir::Rx || self.slots > 2 {
            panic!("This only has meaning in I2S::Tx mode when slots <= 2");
        }
        self.mute_repeat = mute_repeat;
        self
    }

    /// Set the mute counter value
    /// Must be less than or equal to 64
    ///
    /// The value set is compared to the number of consecutive mute frames detected in
    /// reception. When the number of mute frames is equal to this value, the Muted even will be generated
    pub fn set_mute_counter(mut self, mute_counter: u8) -> Self {
        if self.dir == I2SDir::Tx {
            panic!("This only has meaning in I2S::Rx mode");
        }
        // 6 bits or less
        assert!(mute_counter < 0b100_0000);
        self.mute_counter = mute_counter;
        self
    }

    /// Set the tristate to release the SD output line (HI-Z) at the end of the last data bit
    pub fn set_tristate(mut self, tristate: bool) -> Self {
        self.tristate = tristate;
        self
    }

    /// Set the frame size. If None it will be automatically calculated
    pub fn set_frame_size(mut self, frame_size: Option<u8>) -> Self {
        if let Some(frame_size) = frame_size {
            assert!(frame_size >= 8);
        }
        self.frame_size = frame_size;
        self
    }
}

/// I2S Interface
pub struct I2S {
    master: I2SChanConfig,
    slave: Option<I2SChanConfig>,
}

impl INTERFACE for I2S {}

pub type I2sUsers = I2S;

impl I2sUsers {
    pub fn new(master: I2SChanConfig) -> Self {
        Self {
            master,
            slave: None,
        }
    }

    pub fn add_slave(mut self, slave: I2SChanConfig) -> Self {
        self.slave.replace(slave);
        self
    }
}

/// Trait to extend SAI peripherals
pub trait SaiI2sExt<SAI>: Sized {
    type Rec: ResetEnable;
    fn i2s_ch_a<PINS, T>(
        self,
        _pins: PINS,
        audio_freq: T,
        data_size: I2SDataSize,
        prec: Self::Rec,
        clocks: &CoreClocks,
        users: I2sUsers,
    ) -> Sai<SAI, I2S>
    where
        PINS: I2SPinsChA<Self>,
        T: Into<Hertz>;
    fn i2s_ch_b<PINS, T>(
        self,
        _pins: PINS,
        audio_freq: T,
        data_size: I2SDataSize,
        prec: Self::Rec,
        clocks: &CoreClocks,
        users: I2sUsers,
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
                    data_size: I2SDataSize,
                    prec: rec::$Rec,
                    clocks: &CoreClocks,
                    users: I2sUsers,
                ) -> Sai<Self, I2S>
                where
                    PINS: I2SPinsChA<Self>,
                    T: Into<Hertz>,
                {
                    Sai::$i2s_saiX_ch_a(
                        self,
                        _pins,
                        audio_freq.into(),
                        data_size,
                        prec,
                        clocks,
                        users,
                    )
                }
                fn i2s_ch_b<PINS, T>(
                    self,
                    _pins: PINS,
                    audio_freq: T,
                    data_size: I2SDataSize,
                    prec: rec::$Rec,
                    clocks: &CoreClocks,
                    users: I2sUsers,
                ) -> Sai<Self, I2S>
                where
                    PINS: I2SPinsChB<Self>,
                    T: Into<Hertz>,
                {
                    Sai::$i2s_saiX_ch_b(
                        self,
                        _pins,
                        audio_freq.into(),
                        data_size,
                        prec,
                        clocks,
                        users,
                    )
                }
            }

            impl Sai<$SAIX, I2S> {
                pub fn $i2s_saiX_ch_a<PINS>(
                    sai: $SAIX,
                    _pins: PINS,
                    audio_freq: Hertz,
                    data_size: I2SDataSize,
                    prec: rec::$Rec,
                    clocks: &CoreClocks,
                    users: I2sUsers,
                ) -> Self
                where
                    PINS: I2SPinsChA<$SAIX>,
                {
                    assert!(users.master.slots <= NUM_SLOTS);
                    if let Some(slave) = &users.slave {
                        assert!(slave.slots <= NUM_SLOTS);
                    }

                    // Clock config
                    let ker_ck_a = $SAIX::sai_a_ker_ck(&prec, clocks);
                    let clock_ratio = if users.master.oversampling {
                        512
                    } else {
                        256
                    };
                    let mclk_div =
                        (ker_ck_a.0) / (audio_freq.0 * clock_ratio);
                    let mclk_div: u8 = mclk_div
                        .try_into()
                        .expect(concat!(stringify!($SAIX),
                                        " A: Kernel clock is out of range for required MCLK"
                        ));

                    // Configure SAI peripheral
                    let mut per_sai = Sai {
                        rb: sai,
                        master_channel: SaiChannel::ChannelA,
                        slave_channel: if users.slave.is_some() {
                            Some(SaiChannel::ChannelB)
                        } else {
                            None
                        },
                        interface: users,
                    };

                    per_sai.sai_rcc_init(prec);

                    i2s_config_channel(
                        &per_sai.rb.cha,
                        I2SMode::Master,
                        &per_sai.interface.master,
                        mclk_div,
                        data_size,
                    );

                    if let Some(slave) = &per_sai.interface.slave {
                        i2s_config_channel(
                            &per_sai.rb.chb,
                            I2SMode::Slave,
                            slave,
                            0,
                            data_size,
                        );
                    }

                    per_sai
                }

                pub fn $i2s_saiX_ch_b<PINS>(
                    sai: $SAIX,
                    _pins: PINS,
                    audio_freq: Hertz,
                    data_size: I2SDataSize,
                    prec: rec::$Rec,
                    clocks: &CoreClocks,
                    users: I2sUsers,
                ) -> Self
                where
                    PINS: I2SPinsChB<$SAIX>,
                {
                    assert!(users.master.slots <= NUM_SLOTS);
                    if let Some(slave) = &users.slave {
                        assert!(slave.slots <= NUM_SLOTS);
                    }

                    // Clock config
                    let ker_ck_a = $SAIX::sai_b_ker_ck(&prec, clocks);
                    let clock_ratio = if users.master.oversampling {
                        512
                    } else {
                        256
                    };
                    let mclk_div =
                        (ker_ck_a.0) / (audio_freq.0 * clock_ratio);
                    let mclk_div: u8 = mclk_div
                        .try_into()
                        .expect(concat!(stringify!($SAIX),
                                        " B: Kernel clock is out of range for required MCLK"
                        ));


                    // Configure SAI peripheral
                    let mut per_sai = Sai {
                        rb: sai,
                        master_channel: SaiChannel::ChannelB,
                        slave_channel: if users.slave.is_some() {
                            Some(SaiChannel::ChannelA)
                        } else {
                            None
                        },
                        interface: users,
                    };

                    per_sai.sai_rcc_init(prec);

                    i2s_config_channel(
                        &per_sai.rb.chb,
                        I2SMode::Master,
                        &per_sai.interface.master,
                        mclk_div,
                        data_size,
                    );

                    if let Some(slave) = &per_sai.interface.slave {
                        i2s_config_channel(
                            &per_sai.rb.cha,
                            I2SMode::Slave,
                            slave,
                            0,
                            data_size,
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
                    self.master_channel(disable_ch);
                    self.slave_channel(disable_ch);
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
    SAI1, Sai1: [i2s_sai1_ch_a, i2s_sai1_ch_b]
}
#[cfg(any(feature = "rm0433", feature = "rm0399"))]
i2s! {
    SAI2, Sai2: [i2s_sai2_ch_a, i2s_sai2_ch_b],
    SAI3, Sai3: [i2s_sai3_ch_a, i2s_sai3_ch_b],
    SAI4, Sai4: [i2s_sai4_ch_a, i2s_sai4_ch_b]
}
#[cfg(feature = "rm0455")]
i2s! {
    SAI2, Sai2: [i2s_sai2_ch_a, i2s_sai2_ch_b]
}
#[cfg(feature = "rm0468")]
i2s! {
    SAI4, Sai4: [i4s_sai4_ch_a, i4s_sai4_ch_b]
}

fn i2s_config_channel(
    audio_ch: &CH,
    mode: I2SMode,
    config: &I2SChanConfig,
    mclk_div: u8,
    data_size: I2SDataSize,
) {
    let clock_strobe = match config.clock_strobe {
        I2SClockStrobe::Rising => false,
        I2SClockStrobe::Falling => true,
    };

    // 16 bits in register correspond to 1 slot each max 16 slots
    let slot_en_bits: u16 = (2_u32.pow(config.slots.into()) - 1) as u16;

    // Slots to have to be big enough to hold a words worth of data
    let slot_size = match data_size {
        I2SDataSize::BITS_8 => I2SSlotSize::BITS_16,
        I2SDataSize::BITS_10 => I2SSlotSize::BITS_16,
        I2SDataSize::BITS_16 => I2SSlotSize::BITS_16,
        I2SDataSize::BITS_20 => I2SSlotSize::BITS_32,
        I2SDataSize::BITS_24 => I2SSlotSize::BITS_32,
        I2SDataSize::BITS_32 => I2SSlotSize::BITS_32,
    };

    let frame_size = match config.frame_size {
        Some(frame_size) => frame_size,
        None => match data_size {
            I2SDataSize::BITS_8 => 16 * (config.slots / 2),
            I2SDataSize::BITS_10 => 32 * (config.slots / 2),
            I2SDataSize::BITS_16 => 32 * (config.slots / 2),
            I2SDataSize::BITS_20 => 64 * (config.slots / 2),
            I2SDataSize::BITS_24 => 64 * (config.slots / 2),
            I2SDataSize::BITS_32 => 64 * (config.slots / 2),
        },
    };

    let mode_bits = (mode as u8) | (config.dir as u8);
    unsafe {
        audio_ch.cr1.modify(|_, w| {
            w.mode()
                .bits(mode_bits as u8)
                .prtcfg()
                .free()
                .ds()
                .bits(data_size as u8)
                .lsbfirst()
                .bit(config.protocol == I2SProtocol::LSB)
                .ckstr()
                .bit(clock_strobe)
                .syncen()
                .bits(config.sync_type as u8)
                .mono()
                .bit(config.mono_mode)
                .nodiv()
                .bit(config.master_clock_disabled)
                .mckdiv()
                .bits(mclk_div)
                .osr()
                .bit(config.oversampling)
        });
        audio_ch.cr2.modify(|_, w| {
            w.fth()
                .quarter1()
                .tris()
                .bit(config.tristate)
                .mute()
                .clear_bit()
                .muteval()
                .bit(config.mute_repeat)
                .mutecnt()
                .bits(config.mute_counter)
                .cpl()
                .bit(config.complement.into())
                .comp()
                .bits(config.companding as u8)
        });
        audio_ch.frcr.modify(|_, w| {
            w.frl()
                .bits(frame_size - 1)
                .fsall()
                .bits((frame_size / 2) - 1)
                .fsdef()
                .set_bit() // left/right channels enabled
                .fspol()
                .bit(config.frame_sync_active_high)
                .fsoff()
                .bit(config.frame_sync_before)
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
    }
}

fn enable_ch(audio_ch: &CH) {
    unsafe { audio_ch.clrfr.write(|w| w.bits(CLEAR_ALL_FLAGS_BITS)) };
    audio_ch.cr2.modify(|_, w| w.fflush().flush());
    audio_ch.cr1.modify(|_, w| w.saien().enabled());
}

fn disable_ch(audio_ch: &CH) {
    audio_ch.cr1.modify(|_, w| w.saien().disabled());
    while audio_ch.cr1.read().saien().bit_is_set() {}
}

fn read(audio_ch: &CH) -> nb::Result<(u32, u32), I2SError> {
    match audio_ch.sr.read().flvl().variant() {
        Some(sr::FLVL_A::EMPTY) => Err(nb::Error::WouldBlock),
        _ => Ok((audio_ch.dr.read().bits(), audio_ch.dr.read().bits())),
    }
}

fn send(
    left_word: u32,
    right_word: u32,
    audio_ch: &CH,
) -> nb::Result<(), I2SError> {
    // The FIFO is 8 words long. A write consists of 2 words, in stereo mode.
    // Therefore you need to wait for 3/4s to ensure 2 words are available for writing.
    match audio_ch.sr.read().flvl().variant() {
        Some(sr::FLVL_A::FULL) => Err(nb::Error::WouldBlock),
        Some(sr::FLVL_A::QUARTER4) => Err(nb::Error::WouldBlock),
        _ => {
            unsafe {
                audio_ch.dr.write(|w| w.bits(left_word));
                audio_ch.dr.write(|w| w.bits(right_word));
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
            gpio::PE2<Alternate<6>>,
            gpio::PG7<Alternate<6>>
        ]
        SCK_A: [
            gpio::PE5<Alternate<6>>
        ]
        FS_A: [
            gpio::PE4<Alternate<6>>
        ]
        SD_A: [
            gpio::PB2<Alternate<6>>,
            gpio::PC1<Alternate<6>>,
            gpio::PD6<Alternate<6>>,
            gpio::PE6<Alternate<6>>
        ]
        MCLK_B: [
            gpio::PF7<Alternate<6>>
        ]
        SCK_B: [
            gpio::PF8<Alternate<6>>
        ]
        FS_B: [
            gpio::PF9<Alternate<6>>
        ]
        SD_B: [
            gpio::PE3<Alternate<6>>,
            gpio::PF6<Alternate<6>>
        ]
}
#[cfg(any(feature = "rm0433", feature = "rm0399", feature = "rm0455"))]
pins! {
    SAI2:
        MCLK_A: [
            gpio::PE0<Alternate<10>>,
            gpio::PI4<Alternate<10>>
        ]
        SCK_A: [
            gpio::PD13<Alternate<10>>,
            gpio::PI5<Alternate<10>>
        ]
        FS_A: [
            gpio::PD12<Alternate<10>>,
            gpio::PI7<Alternate<10>>
        ]
        SD_A: [
            gpio::PD11<Alternate<10>>,
            gpio::PI6<Alternate<10>>
        ]
        MCLK_B: [
            gpio::PA1<Alternate<10>>,
            gpio::PE6<Alternate<10>>,
            gpio::PE14<Alternate<10>>,
            gpio::PH3<Alternate<10>>
        ]
        SCK_B: [
            gpio::PA2<Alternate<8>>,
            gpio::PE12<Alternate<10>>,
            gpio::PH2<Alternate<10>>
        ]
        FS_B: [
            gpio::PA12<Alternate<8>>,
            gpio::PC0<Alternate<8>>,
            gpio::PE13<Alternate<10>>,
            gpio::PG9<Alternate<10>>
        ]
        SD_B: [
            gpio::PA0<Alternate<10>>,
            gpio::PE11<Alternate<10>>,
            gpio::PF11<Alternate<10>>,
            gpio::PG10<Alternate<10>>
        ]
}
#[cfg(any(feature = "rm0433", feature = "rm0399"))]
pins! {
    SAI3:
        MCLK_A: [
            gpio::PD15<Alternate<6>>
        ]
        SCK_A: [
            gpio::PD0<Alternate<6>>
        ]
        FS_A: [
            gpio::PD4<Alternate<6>>
        ]
        SD_A: [
            gpio::PD1<Alternate<6>>
        ]
        MCLK_B: [
            gpio::PD14<Alternate<6>>
        ]
        SCK_B: [
            gpio::PD8<Alternate<6>>
        ]
        FS_B: [
            gpio::PD10<Alternate<6>>
        ]
        SD_B: [
            gpio::PD9<Alternate<6>>
        ]
}
#[cfg(any(feature = "rm0433", feature = "rm0399", feature = "rm0468"))]
pins! {
    SAI4:
        MCLK_A: [
            gpio::PE2<Alternate<8>>
        ]
        SCK_A: [
            gpio::PE5<Alternate<8>>
        ]
        FS_A: [
            gpio::PE4<Alternate<8>>
        ]
        SD_A: [
            gpio::PB2<Alternate<8>>,
            gpio::PC1<Alternate<8>>,
            gpio::PD6<Alternate<8>>,
            gpio::PE6<Alternate<8>>
        ]
        MCLK_B: [
            gpio::PF7<Alternate<8>>
        ]
        SCK_B: [
            gpio::PF8<Alternate<8>>
        ]
        FS_B: [
            gpio::PF9<Alternate<8>>
        ]
        SD_B: [
            gpio::PE3<Alternate<8>>,
            gpio::PF6<Alternate<8>>
        ]
}
