//! # Serial Audio Interface - DMA

use crate::stm32::SAI1;
#[cfg(feature = "rm0455")]
use crate::stm32::SAI2;
#[cfg(feature = "rm0468")]
use crate::stm32::SAI4;
#[cfg(any(feature = "rm0433", feature = "rm0399"))]
use crate::stm32::{SAI2, SAI3, SAI4};

/// Marker for Channel A
pub struct ChannelA<SAI>(SAI);

/// Marker for Channel B
pub struct ChannelB<SAI>(SAI);

/// Trait to extend SAI peripherals
pub trait SaiDmaExt<SAI>: Sized {
    fn dma_ch_a(self) -> ChannelA<SAI>;
    fn dma_ch_b(self) -> ChannelB<SAI>;
}

macro_rules! i2s_dma {
    ( $($SAIX:ident, $_Rec:ident: [$_i2s_saiX_ch_a:ident, $_i2s_saiX_ch_b:ident]),+ ) => {
        $(
            impl SaiDmaExt<$SAIX> for $SAIX {
                fn dma_ch_a(
                    self,
                ) -> ChannelA<$SAIX> {
                    ChannelA(self)
                }
                fn dma_ch_b(
                    self,
                ) -> ChannelB<$SAIX> {
                    ChannelB(self)
                }
            }
            impl core::ops::Deref for ChannelA<$SAIX> {
                type Target = $SAIX;

                fn deref(&self) -> &Self::Target {
                    &self.0
                }
            }
            impl core::ops::Deref for ChannelB<$SAIX> {
                type Target = $SAIX;

                fn deref(&self) -> &Self::Target {
                    &self.0
                }
            }
        )+
    }
}

i2s_dma! {
    SAI1, Sai1: [i2s_sai1_ch_a, i2s_sai1_ch_b]
}
#[cfg(any(feature = "rm0433", feature = "rm0399"))]
i2s_dma! {
    SAI2, Sai2: [i2s_sai2_ch_a, i2s_sai2_ch_b],
    SAI3, Sai3: [i2s_sai3_ch_a, i2s_sai3_ch_b],
    SAI4, Sai4: [i2s_sai4_ch_a, i2s_sai4_ch_b]
}
#[cfg(feature = "rm0455")]
i2s_dma! {
    SAI2, Sai2: [i2s_sai2_ch_a, i2s_sai2_ch_b]
}
#[cfg(feature = "rm0468")]
i2s_dma! {
    SAI4, Sai4: [i4s_sai4_ch_a, i4s_sai4_ch_b]
}
