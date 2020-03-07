//! # Serial Audio Interface

use crate::stm32::sai4::CH;
use crate::stm32::{SAI1, SAI2, SAI3, SAI4};

// clocks
use crate::rcc::Ccdr;
use crate::stm32::rcc::{d2ccip1r, d3ccipr};
use crate::time::Hertz;
use stm32h7::Variant::Val;

mod pdm;
pub use pdm::SaiPdmExt;

/// Trait for associating clocks with SAI instances
pub trait GetClkSAI {
    fn sai_a_ker_ck(ccdr: &Ccdr) -> Option<Hertz>;
    fn sai_b_ker_ck(ccdr: &Ccdr) -> Option<Hertz>;
}

// Return kernel clocks for this SAI
macro_rules! impl_sai_ker_ck {
    ($ccip:ident,
       $fieldA:ident, $fieldB:ident, $ACCESS_A:ident, $ACCESS_B:ident:
     $($SAIX:ident),+) => {
        $(
            impl GetClkSAI for $SAIX {
                /// Current kernel clock - A
                fn sai_a_ker_ck(ccdr: &Ccdr) -> Option<Hertz> {
                    match ccdr.rb.$ccip.read().$fieldA().variant() {
                        Val($ccip::$ACCESS_A::PLL1_Q) => ccdr.clocks.pll1_q_ck(),
                        Val($ccip::$ACCESS_A::PLL2_P) => ccdr.clocks.pll2_p_ck(),
                        Val($ccip::$ACCESS_A::PLL3_P) => ccdr.clocks.pll3_p_ck(),
                        Val($ccip::$ACCESS_A::I2S_CKIN) => unimplemented!(),
                        Val($ccip::$ACCESS_A::PER) => ccdr.clocks.per_ck(),
                        _ => unreachable!(),
                    }
                }
                /// Current kernel clock - B
                fn sai_b_ker_ck(ccdr: &Ccdr) -> Option<Hertz> {
                    match ccdr.rb.$ccip.read().$fieldB().variant() {
                        Val($ccip::$ACCESS_B::PLL1_Q) => ccdr.clocks.pll1_q_ck(),
                        Val($ccip::$ACCESS_B::PLL2_P) => ccdr.clocks.pll2_p_ck(),
                        Val($ccip::$ACCESS_B::PLL3_P) => ccdr.clocks.pll3_p_ck(),
                        Val($ccip::$ACCESS_B::I2S_CKIN) => unimplemented!(),
                        Val($ccip::$ACCESS_B::PER) => ccdr.clocks.per_ck(),
                        _ => unreachable!(),
                    }
                }
            }
        )+
    };
}
impl_sai_ker_ck! {
    d2ccip1r, sai1sel, sai1sel, SAI1SEL_A, SAI1SEL_A: SAI1
}
impl_sai_ker_ck! {
    d2ccip1r, sai23sel, sai23sel, SAI23SEL_A, SAI23SEL_A: SAI2, SAI3
}
impl_sai_ker_ck! {
    d3ccipr, sai4asel, sai4bsel, SAI4ASEL_A, SAI4BSEL_A: SAI4
}

pub trait INTERFACE {}

/// SAI Events
///
/// Each event is a possible interrupt sources, if enabled
pub enum Event {
    /// Data is available / is required in the FIFO
    Data,
}

/// SAI Channels
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
    ($($SAIX:ident: ($saiX:ident,
                     $apb:ident, $timXen:ident, $timXrst:ident),)+) => {
        $(
            // Common to all interfaces
            impl<INTERFACE> Sai<$SAIX, INTERFACE> {
                /// Low level RCC initialisation
                fn sai_rcc_init(&mut self, ccdr: &mut Ccdr)
                {
                    ccdr.$apb.enr().modify(|_, w| w.$timXen().set_bit());
                    ccdr.$apb.rstr().modify(|_, w| w.$timXrst().set_bit());
                    ccdr.$apb.rstr().modify(|_, w| w.$timXrst().clear_bit());

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


                /// Start listening for `event`
                pub fn listen(&mut self, event: Event) {
                    match event {
                        Event::Data => {
                            // Enable FIFO request interrupt
                            self.rb.cha.im.modify(|_, w| w.freqie().set_bit());
                        }
                    }
                }

                /// Stop listening for `event`
                pub fn unlisten(&mut self, event: Event) {
                    match event {
                        Event::Data => {
                            // Enable FIFO request interrupt
                            self.rb.cha.im.modify(|_, w| w.freqie().clear_bit());
                        }
                    }
                }

                /// Clears interrupt flag
                pub fn clear_irq(&mut self) {
                    self.rb.cha.clrfr.write(|w| {
                        // FIFO request interrupt is cleared by
                        // reading or writing data
                        w
                    });
                }

                /// Releases the SAI peripheral
                pub fn free(self) -> $SAIX {
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

                    self.rb
                }
            }
        )+
    }
}

sai_hal! {
    SAI1: (sai1, apb2, sai1en, sai1rst),
    // Uncomment when an interface is implemented for these
    // SAI2: (sai2, apb2, sai2en, sai2rst),
    // SAI3: (sai3, apb2, sai3en, sai3rst),
    SAI4: (sai4, apb4, sai4en, sai4rst),
}
