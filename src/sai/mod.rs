//! Serial Audio Interface
//!
//! # Examples
//!
//! - [Configure the SAI for PDM mode](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/sai_pdm.rs)
//!
//! ## Examples using the Electro Smith Daisy Seed Board
//!
//! - [SAI with DMA](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/sai_dma_passthru.rs)
//! - [SAI with I2C](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/sai-i2s-passthru.rs)

use core::marker::PhantomData;

#[cfg(any(feature = "rm0455", feature = "rm0468"))]
use crate::stm32::sai1::CH;
#[cfg(any(feature = "rm0433", feature = "rm0399"))]
use crate::stm32::sai4::CH;

use crate::stm32::SAI1;
#[cfg(feature = "rm0455")]
use crate::stm32::SAI2;
#[cfg(feature = "rm0468")]
use crate::stm32::SAI4;
#[cfg(any(feature = "rm0433", feature = "rm0399"))]
use crate::stm32::{SAI2, SAI3, SAI4};

#[cfg(any(feature = "rm0433", feature = "rm0399"))]
use crate::rcc::rec::Sai23ClkSelGetter;

// clocks
use crate::rcc::{rec, CoreClocks, ResetEnable};
use crate::time::Hertz;

const CLEAR_ALL_FLAGS_BITS: u32 = 0b0111_0111;

pub mod dma;
pub use dma::SaiDmaExt;

mod pdm;
pub use pdm::SaiPdmExt;
mod i2s;
pub use i2s::{
    I2SChanConfig, I2SClockStrobe, I2SCompanding, I2SComplement, I2SDataSize,
    I2SDir, I2SMode, I2SProtocol, I2SSync, I2sUsers, SaiI2sExt, I2S,
};

/// Trait for associating clocks with SAI instances
pub trait GetClkSAI {
    type Rec: ResetEnable;

    /// Return the kernel clock for the SAI - A
    ///
    /// # Panics
    ///
    /// Panics if the kernel clock is not running
    fn sai_a_ker_ck(prec: &Self::Rec, clocks: &CoreClocks) -> Hertz;
    /// Return the kernel clock for the SAI - B
    ///
    /// # Panics
    ///
    /// Panics if the kernel clock is not running
    fn sai_b_ker_ck(prec: &Self::Rec, clocks: &CoreClocks) -> Hertz;
}

// Return kernel clocks for this SAI
macro_rules! impl_sai_ker_ck {
    ($Rec:ident,
       $get_mux_A:ident, $get_mux_B:ident, $AccessA:ident, $AccessB:ident:
     $($SAIX:ident),+) => {
        $(
            impl GetClkSAI for $SAIX {
                type Rec = rec::$Rec;

                /// Current kernel clock - A
                fn sai_a_ker_ck(prec: &Self::Rec, clocks: &CoreClocks) -> Hertz {
                    match prec.$get_mux_A() {
                        Some(rec::$AccessA::PLL1_Q) => {
                            clocks.pll1_q_ck().expect(
                                concat!(stringify!($SAIX), " A: PLL1_Q must be enabled")
                            )
                        }
                        Some(rec::$AccessA::PLL2_P) => {
                            clocks.pll2_p_ck().expect(
                                concat!(stringify!($SAIX), " A: PLL2_P must be enabled")
                            )
                        }
                        Some(rec::$AccessA::PLL3_P) => {
                            clocks.pll3_p_ck().expect(
                                concat!(stringify!($SAIX), " A: PLL3_P must be enabled")
                            )
                        }
                        Some(rec::$AccessA::I2S_CKIN) => unimplemented!(),
                        Some(rec::$AccessA::PER) => {
                            clocks.per_ck().expect(
                                concat!(stringify!($SAIX), " A: PER clock must be enabled")
                            )
                        }
                        _ => unreachable!(),
                    }
                }
                /// Current kernel clock - B
                fn sai_b_ker_ck(prec: &Self::Rec, clocks: &CoreClocks) -> Hertz {
                    match prec.$get_mux_B() {
                        Some(rec::$AccessB::PLL1_Q) => {
                            clocks.pll1_q_ck().expect(
                                concat!(stringify!($SAIX), " B: PLL1_Q must be enabled")
                            )
                        }
                        Some(rec::$AccessB::PLL2_P) => {
                            clocks.pll2_p_ck().expect(
                                concat!(stringify!($SAIX), " B: PLL2_P must be enabled")
                            )
                        }
                        Some(rec::$AccessB::PLL3_P) => {
                            clocks.pll3_p_ck().expect(
                                concat!(stringify!($SAIX), " B: PLL3_P must be enabled")
                            )
                        }
                        Some(rec::$AccessB::I2S_CKIN) => unimplemented!(),
                        Some(rec::$AccessB::PER) => {
                            clocks.per_ck().expect(
                                concat!(stringify!($SAIX), " B: PER clock must be enabled")
                            )
                        }
                        _ => unreachable!(),
                    }
                }
            }
        )+
    };
}
impl_sai_ker_ck! {
    Sai1, get_kernel_clk_mux, get_kernel_clk_mux, Sai1ClkSel, Sai1ClkSel: SAI1
}
#[cfg(any(feature = "rm0433", feature = "rm0399"))]
impl_sai_ker_ck! {
    Sai2, get_kernel_clk_mux, get_kernel_clk_mux, Sai23ClkSel, Sai23ClkSel: SAI2
}
#[cfg(any(feature = "rm0433", feature = "rm0399"))]
impl_sai_ker_ck! {
    Sai3, get_kernel_clk_mux, get_kernel_clk_mux, Sai23ClkSel, Sai23ClkSel: SAI3
}

#[cfg(not(feature = "rm0455"))] // Two kernel clocks for SAI4
impl_sai_ker_ck! {
    Sai4, get_kernel_clk_a_mux, get_kernel_clk_b_mux, Sai4AClkSel, Sai4BClkSel: SAI4
}
#[cfg(feature = "rm0455")] // On RM0455 parts, SAI2 has two kernel clocks instead
impl_sai_ker_ck! {
    Sai2, get_kernel_clk_a_mux, get_kernel_clk_b_mux, Sai2AClkSel, Sai2BClkSel: SAI2
}

pub trait INTERFACE {}

/// SAI Events
///
/// Each event is a possible interrupt source, if enabled
#[derive(Copy, Clone, PartialEq)]
pub enum Event {
    /// Overdue/Underrun error detection
    Overdue,
    /// Mute detected (Rx only)
    Muted,
    /// Clock not setup per frame sync rules see RM0433 Section 51.4.6: Frame synchronization
    WrongClock,
    /// Data is available / is required in the FIFO
    Data,
    /// Frame synchronization signal is detected earlier than expected
    AnticipatedFrameSync,
    /// Frame synchronization signal is not present at the right time
    LateFrameSync,
}

/// SAI Channels
#[derive(Copy, Clone, PartialEq)]
pub enum SaiChannel {
    ChannelA,
    ChannelB,
}

/// Hardware serial audio interface peripheral
pub struct Sai<SAI, INTERFACE> {
    rb: SAI,
    master_channel: SaiChannel,
    slave_channel: Option<SaiChannel>,
    interface: INTERFACE,
}

macro_rules! sai_hal {
    ($($SAIX:ident: ($saiX:ident, $Rec:ident),)+) => {
        $(
            // Common to all interfaces
            impl<INTERFACE> Sai<$SAIX, INTERFACE> {
                /// Low level RCC initialisation
                fn sai_rcc_init(&mut self, prec: rec::$Rec)
                {
                    prec.enable().reset();
                }

                /// Access to the current master channel
                fn master_channel<F, T>(&self, func: F) -> T
                    where F: FnOnce(&CH) -> T,
                {
                    match self.master_channel {
                        SaiChannel::ChannelA => func(&self.rb.cha),
                        SaiChannel::ChannelB => func(&self.rb.chb),
                    }
                }

                /// Access to the current slave channel, if set
                fn slave_channel<F, T>(&self, func: F) -> Option<T>
                    where F: FnOnce(&CH) -> T,
                {
                    match self.slave_channel {
                        Some(SaiChannel::ChannelA) => Some(func(&self.rb.cha)),
                        Some(SaiChannel::ChannelB) => Some(func(&self.rb.chb)),
                        None => None
                    }
                }

                /// Start listening for `event` on a given `channel`
                pub fn listen(&mut self, channel: SaiChannel, event: Event) {
                    let ch = match channel {
                        SaiChannel::ChannelA => &self.rb.cha,
                        SaiChannel::ChannelB => &self.rb.chb,
                    };
                    match event {
                        Event::Overdue              => ch.im.modify(|_, w| w.ovrudrie().set_bit()),
                        Event::Muted                => ch.im.modify(|_, w| w.mutedetie().set_bit()),
                        Event::WrongClock           => ch.im.modify(|_, w| w.wckcfgie().set_bit()),
                        Event::Data                 => ch.im.modify(|_, w| w.freqie().set_bit()),
                        Event::AnticipatedFrameSync => ch.im.modify(|_, w| w.afsdetie().set_bit()),
                        Event::LateFrameSync        => ch.im.modify(|_, w| w.lfsdetie().set_bit()),
                    }
                }

                /// Stop listening for `event` on a given `channel`
                pub fn unlisten(&mut self, channel: SaiChannel, event: Event) {
                    let ch = match channel {
                        SaiChannel::ChannelA => &self.rb.cha,
                        SaiChannel::ChannelB => &self.rb.chb,
                    };
                    match event {
                        Event::Overdue              => ch.im.modify(|_, w| w.ovrudrie().clear_bit()),
                        Event::Muted                => ch.im.modify(|_, w| w.mutedetie().clear_bit()),
                        Event::WrongClock           => ch.im.modify(|_, w| w.wckcfgie().clear_bit()),
                        Event::Data                 => ch.im.modify(|_, w| w.freqie().clear_bit()),
                        Event::AnticipatedFrameSync => ch.im.modify(|_, w| w.afsdetie().clear_bit()),
                        Event::LateFrameSync        => ch.im.modify(|_, w| w.lfsdetie().clear_bit()),
                    }
                    let _ = ch.im.read();
                    let _ = ch.im.read(); // Delay 2 peripheral clocks
                }

                /// Clears interrupt flag `event` on the `channel`
                ///
                /// Note: Event::Data is accepted but does nothing as that flag is cleared by reading/writing data
                pub fn clear_irq(&mut self, channel: SaiChannel, event: Event) {
                    let ch = match channel {
                        SaiChannel::ChannelA => &self.rb.cha,
                        SaiChannel::ChannelB => &self.rb.chb,
                    };
                    match event {
                        Event::Overdue              => ch.clrfr.write(|w| w.covrudr().set_bit()),
                        Event::Muted                => ch.clrfr.write(|w| w.cmutedet().set_bit()),
                        Event::WrongClock           => ch.clrfr.write(|w| w.cwckcfg().set_bit()),
                        Event::Data                 => (), // Cleared by reading/writing data
                        Event::AnticipatedFrameSync => ch.clrfr.write(|w| w.cafsdet().set_bit()),
                        Event::LateFrameSync        => ch.clrfr.write(|w| w.clfsdet().set_bit()),
                    }
                    let _ = ch.sr.read();
                    let _ = ch.sr.read(); // Delay 2 peripheral clocks
                }

                /// Clears all interrupts on the `channel`
                pub fn clear_all_irq(&mut self, channel: SaiChannel) {
                    let ch = match channel {
                        SaiChannel::ChannelA => &self.rb.cha,
                        SaiChannel::ChannelB => &self.rb.chb,
                    };
                    unsafe {
                        ch.clrfr.write(|w| w.bits(CLEAR_ALL_FLAGS_BITS));
                    }
                    let _ = ch.sr.read();
                    let _ = ch.sr.read(); // Delay 2 peripheral clocks
                }

                /// Mute `channel`, this is checked at the start of each frame
                /// Meaningful only in Tx mode
                pub fn mute(&mut self, channel: SaiChannel) {
                    match channel {
                        SaiChannel::ChannelA => &self.rb.cha.cr2.modify(|_, w| w.mute().enabled()),
                        SaiChannel::ChannelB => &self.rb.cha.cr2.modify(|_, w| w.mute().enabled()),
                    };
                }

                /// Unmute `channel`, this is checked at the start of each frame
                /// Meaningful only in Tx mode
                pub fn unmute(&mut self, channel: SaiChannel) {
                    match channel {
                        SaiChannel::ChannelA => &self.rb.cha.cr2.modify(|_, w| w.mute().disabled()),
                        SaiChannel::ChannelB => &self.rb.chb.cr2.modify(|_, w| w.mute().disabled()),
                    };
                }

                /// Used to operate the audio block(s) with an external SAI for synchronization
                /// Refer to RM0433 rev 7 section 51.4.4 for valid values
                ///
                /// In short 0-3 maps SAI1-4 with the ones pointing to self being reserved.
                /// e.g. for SAI1 1-3 are valid and 0 is invalid
                pub fn set_sync_input(&mut self, selection: u8) {
                    assert!(selection < 0b1_00);
                    unsafe { &self.rb.gcr.modify(|_, w| w.syncout().bits(selection)) };
                }

                /// Synchronization output for other SAI blocks
                pub fn set_sync_output(&mut self, channel: Option<SaiChannel>) {
                    match channel {
                        Some(SaiChannel::ChannelA) => unsafe { &self.rb.gcr.modify(|_, w| w.syncout().bits(0b01) ) },
                        Some(SaiChannel::ChannelB) => unsafe { &self.rb.gcr.modify(|_, w| w.syncout().bits(0b10) ) },
                        None                       => unsafe { &self.rb.gcr.modify(|_, w| w.syncout().bits(0b00) ) },
                    };
                }

                /// Enable DMA for the SAI peripheral.
                pub fn enable_dma(&mut self, channel: SaiChannel) {
                    match channel {
                        SaiChannel::ChannelA => self.rb.cha.cr1.modify(|_, w| w.dmaen().enabled()),
                        SaiChannel::ChannelB => self.rb.chb.cr1.modify(|_, w| w.dmaen().enabled()),
                    };
                }

                /// Releases the SAI peripheral
                pub fn free(self) -> ($SAIX, rec::$Rec) {
                    // Refer to RM0433 Rev 7 51.4.15 Disabling the SAI

                    // Master: Clear SAIEN
                    self.master_channel(|ch| {
                        ch.cr1.modify(|_, w| w.saien().disabled())
                    });

                    // Master: Wait for SAI to clear at the end of the
                    // frame
                    while self.master_channel(|ch| {
                        ch.cr1.read().saien().bit_is_set()
                    }) {}

                    // Slave: Clear SAIEN
                    self.slave_channel(|ch| {
                        ch.cr1.modify(|_, w| w.saien().disabled())
                    });

                    // Slave: Wait for SAI to clear
                    while self.slave_channel(|ch| {
                        ch.cr1.read().saien().bit_is_set()
                    }).unwrap_or(false) {}


                    (self.rb, rec::$Rec { _marker: PhantomData })
                }
            }
        )+
    }
}

sai_hal! {
    SAI1: (sai1, Sai1),
}
#[cfg(any(feature = "rm0433", feature = "rm0399"))]
sai_hal! {
    SAI2: (sai2, Sai2),
    SAI3: (sai3, Sai3),
    SAI4: (sai4, Sai4),
}
#[cfg(feature = "rm0455")]
sai_hal! {
    SAI2: (sai2, Sai2),
}
#[cfg(feature = "rm0468")]
sai_hal! {
    SAI4: (sai4, Sai4),
}
