//! Direct Memory Access Engine
// Adapted from
// https://github.com/stm32-rs/stm32l4xx-hal/blob/fad6d807b1a0a10bdf22549513d20b34ce2487fe/src/dma.rs
// #![allow(dead_code)]
use crate::stm32::dma1 as dmareg;

#[derive(Debug)]
pub enum Error {
    Overrun,
    BufferError,
    #[doc(hidden)]
    _Extensible,
}

pub enum Event {
    HalfTransfer,
    TransferComplete,
}

#[derive(Clone, Copy, PartialEq)]
pub enum Half {
    First,
    Second,
}

pub enum Direction {
    PeripherialToMemory,
    MemoryToPeripherial,
    MemoryToMemory,
}

/// Chanel level methods with defaults based on DmaInternal
pub trait DmaChannel<MUX>: DmaInternal<MUX> {
    fn set_peripheral_address(&mut self, address: u32, inc: bool) {
        self.par().write(|w| unsafe { w.pa().bits(address) });
        self.cr().modify(|_, w| w.pinc().bit(inc));
    }

    fn set_memory_address(&mut self, address: u32, inc: bool) {
        unsafe { self.m0ar().write(|w| w.bits(address)) };
        self.cr().modify(|_, w| w.minc().bit(inc));
    }

    fn set_transfer_length(&mut self, len: u16) {
        self.ndtr().write(|w| w.ndt().bits(len));
    }

    fn start(&mut self) {
        self.cr().modify(|_, w| w.en().set_bit());
    }

    fn stop(&mut self) {
        self.cr().modify(|_, w| w.en().clear_bit());
    }

    fn clear_all_interrupts(&self);

    fn in_progress(&self) -> bool;

    fn listen(&mut self, event: Event) {
        match event {
            Event::HalfTransfer => self.cr().modify(|_, w| w.htie().set_bit()),
            Event::TransferComplete => {
                self.cr().modify(|_, w| w.tcie().set_bit())
            }
        }
    }

    fn unlisten(&mut self, event: Event) {
        match event {
            Event::HalfTransfer => {
                self.cr().modify(|_, w| w.htie().clear_bit())
            }
            Event::TransferComplete => {
                self.cr().modify(|_, w| w.tcie().clear_bit())
            }
        }
    }

    fn set_direction(&mut self, dir: Direction) {
        match dir {
            Direction::PeripherialToMemory => {
                self.cr().modify(|_, w| w.dir().peripheral_to_memory())
            }
            Direction::MemoryToPeripherial => {
                self.cr().modify(|_, w| w.dir().memory_to_peripheral())
            }
            Direction::MemoryToMemory => {
                self.cr().modify(|_, w| w.dir().memory_to_memory())
            }
        };
    }
}

pub trait DmaInternal<MUX> {
    fn cr(&mut self) -> &dmareg::st::CR;
    fn ndtr(&mut self) -> &dmareg::st::NDTR;
    fn par(&mut self) -> &dmareg::st::PAR;
    fn m0ar(&mut self) -> &dmareg::st::M0AR;
    fn m1ar(&mut self) -> &dmareg::st::M1AR;
    fn fcr(&mut self) -> &dmareg::st::FCR;
    fn get_ndtr(&self) -> u32;
    fn dmamux(&mut self) -> &MUX;
}

pub trait DmaExt {
    type Channels;

    fn split(self) -> Self::Channels;
}

macro_rules! dma {
    ($($DMAX:ident: ($DMAMUXX:ident, $dmamuxx:ident, $dmaX:ident, {
        $($CX:ident: (
            $dmaXindex:literal,
            $isrX:ident,
            $ISR:ty,
            $ifcrX:ident,
            $IFCR:ty,
            $htifX:ident,
            $tcifX:ident,
            $dmifX:ident,
            $feifX:ident,
            $teifX:ident,
            $chtifX:ident,
            $ctcifX:ident,
            $cdmeifX:ident,
            $cfeifX:ident,
            $cteifX:ident,
        ),)+
    }),)+) => {
        $(
            pub mod $dmaX {
                use crate::stm32::{$DMAX, $DMAMUXX, dma1, $dmamuxx};
                use crate::dma::{DmaExt, DmaInternal, DmaChannel};

                pub struct Channels( $(pub $CX),+);

                $(
                    pub struct $CX;

                    impl $CX {
                        #[inline]
                        fn isr(&self) -> $ISR {
                            // NOTE(unsafe) atomic read with no side effects
                            unsafe { (*$DMAX::ptr()).$isrX.read() }
                        }

                        #[inline]
                        fn ifcr(&self) -> &$IFCR {
                            unsafe { &(*$DMAX::ptr()).$ifcrX }
                        }
                    }

                    impl DmaInternal<$dmamuxx::CCR> for $CX {
                        #[inline]
                        fn cr(&mut self) -> &dma1::st::CR {
                            unsafe { &(*$DMAX::ptr()).st[$dmaXindex].cr }
                        }

                        #[inline]
                        fn ndtr(&mut self) -> &dma1::st::NDTR {
                            unsafe { &(*$DMAX::ptr()).st[$dmaXindex].ndtr }
                        }

                        #[inline]
                        fn par(&mut self) -> &dma1::st::PAR {
                            unsafe { &(*$DMAX::ptr()).st[$dmaXindex].par}
                        }

                        #[inline]
                        fn m0ar(&mut self) -> &dma1::st::M0AR {
                            unsafe { &(*$DMAX::ptr()).st[$dmaXindex].m0ar }
                        }

                        #[inline]
                        fn m1ar(&mut self) -> &dma1::st::M1AR {
                            unsafe { &(*$DMAX::ptr()).st[$dmaXindex].m1ar }
                        }

                        #[inline]
                        fn fcr(&mut self) -> &dma1::st::FCR {
                            unsafe { &(*$DMAX::ptr()).st[$dmaXindex].fcr }
                        }

                        #[inline]
                        fn get_ndtr(&self) -> u32 {
                            // NOTE(unsafe) atomic read with no side effects
                            unsafe { (*$DMAX::ptr()).st[$dmaXindex].ndtr.read().bits() }
                        }

                        #[inline]
                        fn dmamux(&mut self) -> &$dmamuxx::CCR {
                            unsafe { &(*$DMAMUXX::ptr()).ccr[$dmaXindex] }
                        }
                    }

                    impl DmaChannel<$dmamuxx::CCR> for $CX {
                        fn in_progress(&self) -> bool {
                            self.isr().$tcifX().bit_is_clear()
                        }

                        fn clear_all_interrupts(&self) {
                            self.ifcr().write(|w| {
                                w.$chtifX().set_bit()
                                .$ctcifX().set_bit()
                                .$cdmeifX().set_bit()
                                .$cfeifX().set_bit()
                                .$cteifX().set_bit()
                            });
                        }
                    }
                )+

                impl DmaExt for $DMAX {
                    type Channels = Channels;

                    fn split(self) -> Channels {
                        Channels( $($CX { }),+)
                    }
                }
            }
        )+
    }
}

dma! {
    DMA1: (DMAMUX1, dmamux1, dma1, {
        C0: (
            0,
            lisr, dma1::lisr::R,
            lifcr, dma1::LIFCR,
            htif0,
            tcif0,
            dmif0,
            feif0,
            teif0,
            chtif0,
            ctcif0,
            cdmeif0,
            cfeif0,
            cteif0,
        ),
        C1: (
            1,
            lisr, dma1::lisr::R,
            lifcr, dma1::LIFCR,
            htif1,
            tcif1,
            dmif1,
            feif1,
            teif1,
            chtif1,
            ctcif1,
            cdmeif1,
            cfeif1,
            cteif1,
        ),
        C2: (
            2,
            lisr, dma1::lisr::R,
            lifcr, dma1::LIFCR,
            htif2,
            tcif2,
            dmif2,
            feif2,
            teif2,
            chtif2,
            ctcif2,
            cdmeif2,
            cfeif2,
            cteif2,
        ),
        C3: (
            3,
            lisr, dma1::lisr::R,
            lifcr, dma1::LIFCR,
            htif3,
            tcif3,
            dmif3,
            feif3,
            teif3,
            chtif3,
            ctcif3,
            cdmeif3,
            cfeif3,
            cteif3,
        ),
        C4: (
            4,
            hisr, dma1::hisr::R,
            hifcr, dma1::HIFCR,
            htif4,
            tcif4,
            dmif4,
            feif4,
            teif4,
            chtif4,
            ctcif4,
            cdmeif4,
            cfeif4,
            cteif4,
        ),
        C5: (
            5,
            hisr, dma1::hisr::R,
            hifcr, dma1::HIFCR,
            htif5,
            tcif5,
            dmif5,
            feif5,
            teif5,
            chtif5,
            ctcif5,
            cdmeif5,
            cfeif5,
            cteif5,
        ),
        C6: (
            6,
            hisr, dma1::hisr::R,
            hifcr, dma1::HIFCR,
            htif6,
            tcif6,
            dmif6,
            feif6,
            teif6,
            chtif6,
            ctcif6,
            cdmeif6,
            cfeif6,
            cteif6,
        ),
        C7: (
            7,
            hisr, dma1::hisr::R,
            hifcr, dma1::HIFCR,
            htif7,
            tcif7,
            dmif7,
            feif7,
            teif7,
            chtif7,
            ctcif7,
            cdmeif7,
            cfeif7,
            cteif7,
        ),
    }),
    DMA2: (DMAMUX2, dmamux2, dma2, {
        C0: (
            0,
            lisr, dma1::lisr::R,
            lifcr, dma1::LIFCR,
            htif0,
            tcif0,
            dmif0,
            feif0,
            teif0,
            chtif0,
            ctcif0,
            cdmeif0,
            cfeif0,
            cteif0,
        ),
        C1: (
            1,
            lisr, dma1::lisr::R,
            lifcr, dma1::LIFCR,
            htif1,
            tcif1,
            dmif1,
            feif1,
            teif1,
            chtif1,
            ctcif1,
            cdmeif1,
            cfeif1,
            cteif1,
        ),
        C2: (
            2,
            lisr, dma1::lisr::R,
            lifcr, dma1::LIFCR,
            htif2,
            tcif2,
            dmif2,
            feif2,
            teif2,
            chtif2,
            ctcif2,
            cdmeif2,
            cfeif2,
            cteif2,
        ),
        C3: (
            3,
            lisr, dma1::lisr::R,
            lifcr, dma1::LIFCR,
            htif3,
            tcif3,
            dmif3,
            feif3,
            teif3,
            chtif3,
            ctcif3,
            cdmeif3,
            cfeif3,
            cteif3,
        ),
        C4: (
            4,
            hisr, dma1::hisr::R,
            hifcr, dma1::HIFCR,
            htif4,
            tcif4,
            dmif4,
            feif4,
            teif4,
            chtif4,
            ctcif4,
            cdmeif4,
            cfeif4,
            cteif4,
        ),
        C5: (
            5,
            hisr, dma1::hisr::R,
            hifcr, dma1::HIFCR,
            htif5,
            tcif5,
            dmif5,
            feif5,
            teif5,
            chtif5,
            ctcif5,
            cdmeif5,
            cfeif5,
            cteif5,
        ),
        C6: (
            6,
            hisr, dma1::hisr::R,
            hifcr, dma1::HIFCR,
            htif6,
            tcif6,
            dmif6,
            feif6,
            teif6,
            chtif6,
            ctcif6,
            cdmeif6,
            cfeif6,
            cteif6,
        ),
        C7: (
            7,
            hisr, dma1::hisr::R,
            hifcr, dma1::HIFCR,
            htif7,
            tcif7,
            dmif7,
            feif7,
            teif7,
            chtif7,
            ctcif7,
            cdmeif7,
            cfeif7,
            cteif7,
        ),
    }),
}
