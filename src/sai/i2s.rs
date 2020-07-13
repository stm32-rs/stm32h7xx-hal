//! # Serial Audio Interface - Inter-IC Sound
//!
//! Inter-IC Sound.
//!
#[allow(dead_code)]
use core::convert::TryInto;

use crate::rcc::{rec, CoreClocks, ResetEnable};
use crate::sai::{GetClkSAI, Sai, SaiChannel, INTERFACE};
use crate::stm32;
use crate::stm32::{SAI1};
use crate::time::Hertz;

// use crate::gpio::gpiob::PB2;
// use crate::gpio::gpioc::PC1;
// use crate::gpio::gpiod::PD6;
use crate::gpio::gpioe::{PE2, PE3, PE4, PE5, PE6};
use crate::gpio::gpiof::{PF7, PF8, PF9};
// use crate::gpio::gpiof::PG7;
use crate::gpio::{Alternate, AF6};
use stm32h7::Variant::Val;
use crate::device::sai4::ch::sr;

use embedded_hal::i2s::FullDuplex;
// use embedded_hal::i2s::FullDuplex as FullDuplex;

const NUM_SLOTS: u8 = 16;

#[derive(Clone, Copy, PartialEq)]
pub enum I2SMode {
    Master = 0b00,
    Slave = 0b10,
}

#[derive(Copy, Clone)]
pub enum I2SDir {
    Tx = 0b00,
    Rx = 0b01,
}

#[derive(Copy, Clone)]
pub enum I2SBitRate {
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
pub enum I2SSynchronization {
    Master = 0b00,
    Internal = 0b01,
    External = 0b10,
}

#[derive(Copy, Clone, Debug)]
pub enum I2SError {
    WouldBlock,
}

#[derive(Copy, Clone)]
pub enum I2SOverSampling {
    Enabled = 2,
    Disabled = 1,
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

/*
RM0433 Rev 7 Reference Manual
pg 2257
An I/O line controller manages a set of 4 dedicated pins (SD, SCK, FS, MCLK) for a given
audio block in the SAI. Some of these pins can be shared if the two subblocks are declared
as synchronous to leave some free to be used as general purpose I/Os. The MCLK pin can
be output, or not, depending on the application, the decoder requirement and whether the
audio block is configured as the master.
If one SAI is configured to operate synchronously with another one, even more I/Os can be
freed (except for pins SD_x).

DS12556 Rev 5
STM32H750VB STM32H750ZB
STM32H750IB STM32H750XB
TODO
-Pins for SAI2-4
*/

impl I2SPinMclkA<SAI1> for PE2<Alternate<AF6>> {}
impl I2SPinSckA<SAI1> for PE5<Alternate<AF6>> {}
impl I2SPinFsA<SAI1> for PE4<Alternate<AF6>> {}
impl I2SPinSdA<SAI1> for PE6<Alternate<AF6>> {}

impl I2SPinMclkB<SAI1> for PF7<Alternate<AF6>> {}
impl I2SPinSckB<SAI1> for PF8<Alternate<AF6>> {}
impl I2SPinFsB<SAI1> for PF9<Alternate<AF6>> {}
impl I2SPinSdB<SAI1> for PE3<Alternate<AF6>> {}

/// I2S Interface
pub struct I2S {
    master_mode: I2SMode,
    master_dir: I2SDir,
    slave_mode: Option<I2SMode>,
    slave_dir: Option<I2SDir>,
}
impl INTERFACE for I2S {}

/// Trait to extend SAI periperhals
pub trait SaiI2sExt<SAI>: Sized {
    type Rec: ResetEnable;
    fn i2s_ch_a<PINS, T>(
        self,
        _pins: PINS,
        audio_freq: T,
        bit_rate: I2SBitRate,
        prec: Self::Rec,
        clocks: &CoreClocks,
        master_channel: SaiChannel,
        master_mode: I2SMode,
        master_dir: I2SDir,
        slave_channel: Option<SaiChannel>,
        slave_mode: Option<I2SMode>,
        slave_dir: Option<I2SDir>,
    ) -> Sai<SAI, I2S>
    where
        PINS: I2SPinsChA<Self>,
        T: Into<Hertz>;
    fn i2s_ch_b<PINS, T>(
        self,
        _pins: PINS,
        audio_freq: T,
        bit_rate: I2SBitRate,
        prec: Self::Rec,
        clocks: &CoreClocks,
        master_channel: SaiChannel,
        master_mode: I2SMode,
        master_dir: I2SDir,
        slave_channel: Option<SaiChannel>,
        slave_mode: Option<I2SMode>,
        slave_dir: Option<I2SDir>,
    ) -> Sai<SAI, I2S>
    where
        PINS: I2SPinsChB<Self>,
        T: Into<Hertz>;
}

impl SaiI2sExt<SAI1> for SAI1 {
    type Rec = rec::Sai1;
    fn i2s_ch_a<PINS, T>(
        self,
        _pins: PINS,
        audio_freq: T,
        bit_rate: I2SBitRate,
        prec: rec::Sai1,
        clocks: &CoreClocks,
        master_channel: SaiChannel,
        master_mode: I2SMode,
        master_dir: I2SDir,
        slave_channel: Option<SaiChannel>,
        slave_mode: Option<I2SMode>,
        slave_dir: Option<I2SDir>,
    ) -> Sai<Self, I2S>
    where
        PINS: I2SPinsChA<Self>,
        T: Into<Hertz>,
    {
        Sai::i2s_sai1_ch_a(
            self,
            _pins,
            audio_freq.into(),
            bit_rate,
            prec,
            clocks,
            master_channel,
            master_mode,
            master_dir,
            slave_channel,
            slave_mode,
            slave_dir,
        )
    }
    fn i2s_ch_b<PINS, T>(
        self,
        _pins: PINS,
        audio_freq: T,
        bit_rate: I2SBitRate,
        prec: rec::Sai1,
        clocks: &CoreClocks,
        master_channel: SaiChannel,
        master_mode: I2SMode,
        master_dir: I2SDir,
        slave_channel: Option<SaiChannel>,
        slave_mode: Option<I2SMode>,
        slave_dir: Option<I2SDir>,
    ) -> Sai<Self, I2S>
    where
        PINS: I2SPinsChB<Self>,
        T: Into<Hertz>,
    {
        Sai::i2s_sai1_ch_b(
            self,
            _pins,
            audio_freq.into(),
            bit_rate,
            prec,
            clocks,
            master_channel,
            master_mode,
            master_dir,
            slave_channel,
            slave_mode,
            slave_dir,
        )
    }
}

impl Sai<SAI1, I2S> {
    pub fn i2s_sai1_ch_a<PINS>(
        sai: SAI1,
        _pins: PINS,
        audio_freq: Hertz,
        bit_rate: I2SBitRate,
        prec: rec::Sai1,
        clocks: &CoreClocks,
        master_channel: SaiChannel,
        master_mode: I2SMode,
        master_dir: I2SDir,
        slave_channel: Option<SaiChannel>,
        slave_mode: Option<I2SMode>,
        slave_dir: Option<I2SDir>,
    ) -> Self
    where
        PINS: I2SPinsChA<SAI1>,
    {
        // Clock config
        let nbslot: u8 = 2;
        assert!(nbslot <= NUM_SLOTS);
        let (frame_length, slot_size) = match bit_rate {
            I2SBitRate::BITS_8 => (16 * (nbslot / 2), I2SSlotSize::BITS_16),
            I2SBitRate::BITS_10 => (32 * (nbslot / 2), I2SSlotSize::BITS_16),
            I2SBitRate::BITS_16 => (32 * (nbslot / 2), I2SSlotSize::BITS_16),
            I2SBitRate::BITS_20 => (64 * (nbslot / 2), I2SSlotSize::BITS_32),
            I2SBitRate::BITS_24 => (64 * (nbslot / 2), I2SSlotSize::BITS_32),
            I2SBitRate::BITS_32 => (64 * (nbslot / 2), I2SSlotSize::BITS_32),
        };

        /* Configure Master Clock Divider using the following formula :
        - If NODIV = 1 :
            MCKDIV[5:0] = SAI_CK_x / (audio_frequncy * (frame_length + 1))
        - If NODIV = 0 :
            MCKDIV[5:0] = SAI_CK_x / (audio_frequncy * (oversampling_rate + 1) * 256) */

        let ker_ck_a = SAI1::sai_a_ker_ck(&prec, clocks)
            .expect("SAI kernel clock must run!");
        // Divider enabled
        let mclk_div = (ker_ck_a.0)
            / (audio_freq.0 * (I2SOverSampling::Disabled as u32) * 256);
        let mclk_div: u8 = mclk_div
            .try_into()
            .expect("SAI kernel clock is out of range for required MCLK");

        // Configure SAI peripeheral
        let mut per_sai = Sai {
            rb: sai,
            master_channel,
            slave_channel,
            interface: I2S {
                master_mode,
                master_dir,
                slave_mode,
                slave_dir,
            },
        };

        per_sai.sai_rcc_init(prec);

        // 16 bits in register correspond to 1 slot each max 16 slots
        let slot_en_bits: u16 = (2_u32.pow(nbslot.into()) - 1) as u16;
        // let slot_en_bits: u16 = 0xFFFF;

        //     if (protocol == SAI_I2S_LSBJUSTIFIED)
        //   {
        //     if (datasize == SAI_PROTOCOL_DATASIZE_16BITEXTENDED)
        //     {
        //       hsai->SlotInit.FirstBitOffset = 16;
        //     }
        //     if (datasize == SAI_PROTOCOL_DATASIZE_24BIT)
        //     {
        //       hsai->SlotInit.FirstBitOffset = 8;
        //     }
        //   }
        let first_bit_offset = 0;

        let master_ch = match per_sai.master_channel {
            SaiChannel::ChannelA => &per_sai.rb.cha,
            SaiChannel::ChannelB => &per_sai.rb.chb,
        };

        /*  Follow the sequence below to configure the SAI interface in DMA mode:
            1. Configure SAI and FIFO threshold levels to specify when the DMA request will be
            launched.
            2. Configure SAI DMA channel.
            3. Enable the DMA.
            4. Enable the SAI interface.
            Note: Before configuring the SAI block, the SAI DMA channel must be disabled.
        */

        i2s_config_channel(
            master_ch,
            I2SMode::Master,
            I2SDir::Tx,
            I2SSynchronization::Master,
            mclk_div,
            bit_rate,
            nbslot,
            frame_length,
            first_bit_offset,
            slot_size,
            slot_en_bits,
        );

        if let Some(slave_channel) = &per_sai.slave_channel {
            let slave_ch = match slave_channel {
                SaiChannel::ChannelA => &per_sai.rb.cha,
                SaiChannel::ChannelB => &per_sai.rb.chb,
            };
            i2s_config_channel(
                slave_ch,
                I2SMode::Slave,
                I2SDir::Rx,
                I2SSynchronization::Internal,
                0,
                bit_rate,
                nbslot,
                frame_length,
                first_bit_offset,
                slot_size,
                slot_en_bits,
            );
        }

        per_sai
    }

    pub fn i2s_sai1_ch_b<PINS>(
        sai: SAI1,
        _pins: PINS,
        _audio_freq: Hertz,
        _bit_rate: I2SBitRate,
        _prec: rec::Sai1,
        _clocks: &CoreClocks,
        master_channel: SaiChannel,
        master_mode: I2SMode,
        master_dir: I2SDir,
        slave_channel: Option<SaiChannel>,
        slave_mode: Option<I2SMode>,
        slave_dir: Option<I2SDir>,
        
    ) -> Self
    where
        PINS: I2SPinsChB<SAI1>,
    {
        let per_sai = Sai {
            rb: sai,
            master_channel,
            slave_channel,
            interface: I2S {
                master_mode,
                master_dir,
                slave_mode,
                slave_dir,
            },
        };
        per_sai
    }

    pub fn enable(&mut self) {
        // // Enable DMA
        // let dma1 = unsafe { &*stm32::DMA1::ptr() };
        // // Clear flags
        // // Hard coded to streams 0 and 1
        // if let Some(master_dma) = &self.interface.master_dma {
        //     &dma1.lifcr.write(|w| {
        //         w.cdmeif0()
        //             .clear()
        //             .cfeif0()
        //             .clear()
        //             .chtif0()
        //             .clear()
        //             .ctcif0()
        //             .clear()
        //             .cteif0()
        //             .clear()
        //             .cdmeif1()
        //             .clear()
        //             .cfeif1()
        //             .clear()
        //             .chtif1()
        //             .clear()
        //             .ctcif1()
        //             .clear()
        //             .cteif1()
        //             .clear()
        //     });
        //     if let Some(slave_dma) = &self.interface.slave_dma {
        //         dma1.st[slave_dma.stream].cr.modify(|_, w| w.en().enabled());
        //     }
        //     dma1.st[master_dma.stream].cr.modify(|_, w| w.en().enabled());
        // }

        // Enable slave first "recommended" per ref doc
        if let Some(slave_channel) = &self.slave_channel {
            self.rb.cha.clrfr.write(|w| {
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
            self.rb.chb.clrfr.write(|w| {
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
            match slave_channel {
                SaiChannel::ChannelA => {
                    self.rb.cha.cr2.modify(|_, w| w.fflush().flush());
                    self.rb.cha.cr1.modify(|_, w| w.saien().enabled());
                }
                SaiChannel::ChannelB => {
                    self.rb.chb.cr2.modify(|_, w| w.fflush().flush());
                    self.rb.chb.cr1.modify(|_, w| w.saien().enabled());
                }
            }
        };
        match self.master_channel {
            SaiChannel::ChannelA => {
                self.rb.cha.cr1.modify(|_, w| w.saien().enabled());
            }
            SaiChannel::ChannelB => {
                self.rb.chb.cr1.modify(|_, w| w.saien().enabled());
            }
        }
    }

    pub fn disable(&mut self) {
        // Master must be disabled first
        match self.master_channel {
            SaiChannel::ChannelA => {
                self.rb.cha.cr1.modify(|_, w| w.saien().disabled());
            }
            SaiChannel::ChannelB => {
                self.rb.chb.cr1.modify(|_, w| w.saien().disabled());
            }
        }
        if let Some(slave_channel) = &self.slave_channel {
            match slave_channel {
                SaiChannel::ChannelA => {
                    self.rb.cha.cr1.modify(|_, w| w.saien().disabled());
                }
                SaiChannel::ChannelB => {
                    self.rb.chb.cr1.modify(|_, w| w.saien().disabled());
                }
            }
        };
    }
}

impl<I2S> FullDuplex<u32> for Sai<SAI1, I2S> {
    type Error = I2SError;

    fn try_read(&mut self) -> nb::Result<(u32, u32), Self::Error> {
        let chan = &self.rb.chb;
        match chan.sr.read().flvl().variant() {
            Val(sr::FLVL_A::EMPTY) => Err(nb::Error::WouldBlock),
            _ => Ok((chan.dr.read().bits(), chan.dr.read().bits()))
        }
    }

    fn try_send(
        &mut self,
        left_word: u32,
        right_word: u32,
    ) -> nb::Result<(), Self::Error> {
        let chan = &self.rb.cha;
        match chan.sr.read().flvl().variant() {
            Val(sr::FLVL_A::FULL) => Err(nb::Error::WouldBlock),
            Val(sr::FLVL_A::QUARTER4) => Err(nb::Error::WouldBlock),
            _ => {
                unsafe {
                    chan.dr.write(|w| w.bits(left_word).bits(right_word) );
                }
                Ok(())
            }
        }
    }
}

fn i2s_config_channel(
    audio_ch: &stm32::sai4::CH,
    mode: I2SMode,
    dir: I2SDir,
    sync_bits: I2SSynchronization,
    mclk_div: u8,
    bit_rate: I2SBitRate,
    nbslot: u8,
    frame_length: u8,
    first_bit_offset: u8,
    slot_size: I2SSlotSize,
    slot_en_bits: u16,
) {
    /*  Compute ClockStrobing according to mode
        Tx = Falling = 1 = true
        Rx = Rising =  0 = false
    */
    let clock_strobe = true;

    let mode_bits = (mode as u8) | (dir as u8);
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
                .bits(sync_bits as u8)
                .mono()
                .stereo()
                //OUT DRIV
                //SAI EN
                .nodiv()
                .master_clock()
                .mckdiv()
                .bits(mclk_div)
            //OSR defaults to 0
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
                .bits(first_bit_offset)
                .slotsz()
                .bits(slot_size as u8)
                .nbslot()
                .bits(nbslot - 1)
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

// fn i2s_config_dma(
//     mode: I2SMode,
//     dir: I2SDir,
//     audio_ch: &stm32::sai4::CH,
//     dma: &Dma,
// ) {
//     let dma1 = unsafe { &*stm32::DMA1::ptr() };
//     let dmamux1 = unsafe { &*stm32::DMAMUX1::ptr() };
//     let stream = &dma1.st[dma.stream];

//     // Configure DMA completely
//     // Configure MUX completely
//     // Enable DMA

//     // Set direction
//     match dir {
//         I2SDir::Rx => {
//             stream.cr.modify(|_, w| w.dir().peripheral_to_memory());
//         }
//         I2SDir::Tx => {
//             stream.cr.modify(|_, w| w.dir().memory_to_peripheral());
//         }
//     }

//     // Stream pointers
//     unsafe {
//         stream.m0ar.modify(|_, w| w.bits(dma.memory_addr));
//         stream.par.modify(|_, w| w.bits(&audio_ch.dr as *const _ as u32));
//     }

//     stream.cr.modify(|_, w| {
//         w.circ()
//             .enabled() // Circular mode implies DMA flow control
//             .minc()
//             .incremented()
//             .pl()
//             .high()
//             .psize()
//             .bits32()
//             .msize()
//             .bits32()
//     });
//     stream.fcr.modify(|_, w| w.dmdis().enabled().fth().quarter());
//     // Block size * num channels * 2
//     stream
//         .ndtr
//         .modify(|_, w| w.ndt().bits(dma.length));

//     // Events
//     // if mode == I2SMode::Master {
//     //     &dmamux1.rgcr[dma.stream]
//     //         .modify(|_, w| w.gpol().rising_edge().gnbreq().bits(0b0));
//     //     &dmamux1.rgcr[dma.stream].modify(|_, w| w.ge().enabled());
//     // }
//     // Interrupts
//     stream.cr.modify(|_, w| {
//         w.dmeie()
//             .enabled()
//             .htie().enabled()
//             .teie()
//             .enabled()
//             .tcie()
//             .enabled()
//     });
//     stream.cr.modify(|_, w| w.tcie().enabled());

//     match dir {
//         I2SDir::Rx => {
//             &dmamux1.ccr[dma.stream].modify(|_, w| w.dmareq_id().sai1a_dma());
//         }
//         I2SDir::Tx => {
//             &dmamux1.ccr[dma.stream].modify(|_, w| w.dmareq_id().sai1b_dma());
//         }
//     }
// }
