//! # Serial Audio Interface

use core::marker::PhantomData;

use crate::stm32::sai4::CH;
use crate::stm32::{SAI1, SAI4};

// clocks
use crate::rcc::{rec, CoreClocks, ResetEnable};
use crate::time::Hertz;
use stm32h7::Variant::Val;

mod pdm;
pub use pdm::SaiPdmExt;

/// Trait for associating clocks with SAI instances
pub trait GetClkSAI {
    type Rec: ResetEnable;

    fn sai_a_ker_ck(prec: &Self::Rec, clocks: &CoreClocks) -> Option<Hertz>;
    fn sai_b_ker_ck(prec: &Self::Rec, clocks: &CoreClocks) -> Option<Hertz>;
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
                fn sai_a_ker_ck(prec: &Self::Rec, clocks: &CoreClocks) -> Option<Hertz> {
                    match prec.$get_mux_A() {
                        Val(rec::$AccessA::PLL1_Q) => clocks.pll1_q_ck(),
                        Val(rec::$AccessA::PLL2_P) => clocks.pll2_p_ck(),
                        Val(rec::$AccessA::PLL3_P) => clocks.pll3_p_ck(),
                        Val(rec::$AccessA::I2S_CKIN) => unimplemented!(),
                        Val(rec::$AccessA::PER) => clocks.per_ck(),
                        _ => unreachable!(),
                    }
                }
                /// Current kernel clock - B
                fn sai_b_ker_ck(prec: &Self::Rec, clocks: &CoreClocks) -> Option<Hertz> {
                    match prec.$get_mux_B() {
                        Val(rec::$AccessB::PLL1_Q) => clocks.pll1_q_ck(),
                        Val(rec::$AccessB::PLL2_P) => clocks.pll2_p_ck(),
                        Val(rec::$AccessB::PLL3_P) => clocks.pll3_p_ck(),
                        Val(rec::$AccessB::I2S_CKIN) => unimplemented!(),
                        Val(rec::$AccessB::PER) => clocks.per_ck(),
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
// impl_sai_ker_ck! {
//     d2ccip1r, get_kernel_clk_mux, get_kernel_clk_mux, SAI23SEL_A, SAI23SEL_A: SAI2, SAI3
// }
impl_sai_ker_ck! {
    Sai4, get_kernel_clk_a_mux, get_kernel_clk_b_mux, Sai4AClkSel, Sai4BClkSel: SAI4
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
    // Uncomment when an interface is implemented for these
    // SAI2: (sai2, Sai2),
    // SAI3: (sai3, Sai3),
    SAI4: (sai4, Sai4),
}
