//! DMA1 and DMA2

use super::{
    config,
    traits::sealed::{Bits, Sealed},
    traits::*,
    CurrentBuffer, FifoLevel, MemoryToPeripheral, PeripheralToMemory,
};
use core::marker::PhantomData;

use crate::{
    pac::{self, DMA1, DMA2, DMAMUX1},
    rcc::{rec, rec::ResetEnable},
    //serial::{Rx, Tx},
};

use core::ops::Deref;

impl Sealed for DMA1 {}
impl Sealed for DMA2 {}

/// Type aliases for register blocks
pub type DMARegisterBlock = pac::dma1::RegisterBlock;
pub type DMAMUXRegisterBlock = pac::dmamux1::RegisterBlock;

/// Trait that represents an instance of a DMA peripheral
pub trait Instance: Deref<Target = DMARegisterBlock> + Sealed {
    type Rec: ResetEnable;

    /// Gives a pointer to the RegisterBlock.
    fn ptr() -> *const DMARegisterBlock;

    /// Gives a pointer to the DMAMUX used for this DMA.
    fn mux_ptr() -> *const DMAMUXRegisterBlock;

    const DMA_MUX_STREAM_OFFSET: usize;
}

// DMA1 channels 0 to 7 are connected to DMAMUX1 channels 0 to 7
impl Instance for DMA1 {
    type Rec = rec::Dma1;

    #[inline(always)]
    fn ptr() -> *const DMARegisterBlock {
        DMA1::ptr()
    }

    #[inline(always)]
    fn mux_ptr() -> *const DMAMUXRegisterBlock {
        DMAMUX1::ptr()
    }

    const DMA_MUX_STREAM_OFFSET: usize = 0;
}

// DMA2 channels 0 to 7 are connected to DMAMUX1 channels 8 to 15
impl Instance for DMA2 {
    type Rec = rec::Dma2;

    #[inline(always)]
    fn ptr() -> *const DMARegisterBlock {
        DMA2::ptr()
    }

    #[inline(always)]
    fn mux_ptr() -> *const DMAMUXRegisterBlock {
        DMAMUX1::ptr()
    }
    const DMA_MUX_STREAM_OFFSET: usize = 8;
}

/// Stream 0 on DMA1/2
pub struct Stream0<DMA> {
    _dma: PhantomData<DMA>,
}
/// Stream 1 on DMA1/2
pub struct Stream1<DMA> {
    _dma: PhantomData<DMA>,
}
/// Stream 2 on DMA1/2
pub struct Stream2<DMA> {
    _dma: PhantomData<DMA>,
}
/// Stream 3 on DMA1/2
pub struct Stream3<DMA> {
    _dma: PhantomData<DMA>,
}
/// Stream 4 on DMA1/2
pub struct Stream4<DMA> {
    _dma: PhantomData<DMA>,
}
/// Stream 5 on DMA1/2
pub struct Stream5<DMA> {
    _dma: PhantomData<DMA>,
}
/// Stream 6 on DMA1/2
pub struct Stream6<DMA> {
    _dma: PhantomData<DMA>,
}
/// Stream 7 on DMA1/2
pub struct Stream7<DMA> {
    _dma: PhantomData<DMA>,
}

impl<DMA> Sealed for Stream0<DMA> {}
impl<DMA> Sealed for Stream1<DMA> {}
impl<DMA> Sealed for Stream2<DMA> {}
impl<DMA> Sealed for Stream3<DMA> {}
impl<DMA> Sealed for Stream4<DMA> {}
impl<DMA> Sealed for Stream5<DMA> {}
impl<DMA> Sealed for Stream6<DMA> {}
impl<DMA> Sealed for Stream7<DMA> {}

/// Alias for a tuple with all DMA streams.
pub struct StreamsTuple<T>(
    pub Stream0<T>,
    pub Stream1<T>,
    pub Stream2<T>,
    pub Stream3<T>,
    pub Stream4<T>,
    pub Stream5<T>,
    pub Stream6<T>,
    pub Stream7<T>,
);

impl<I: Instance> StreamsTuple<I> {
    /// Splits the DMA peripheral into streams.
    pub fn new(_regs: I, prec: I::Rec) -> Self {
        prec.enable().reset();
        Self(
            Stream0 { _dma: PhantomData },
            Stream1 { _dma: PhantomData },
            Stream2 { _dma: PhantomData },
            Stream3 { _dma: PhantomData },
            Stream4 { _dma: PhantomData },
            Stream5 { _dma: PhantomData },
            Stream6 { _dma: PhantomData },
            Stream7 { _dma: PhantomData },
        )
    }
}

// Macro that creates a struct representing a stream on either DMA controller
//
// The implementation does the heavy lifting of mapping to the right fields on
// the stream
macro_rules! dma_stream {
    ($(($name:ident, $number:expr ,$ifcr:ident, $tcif:ident, $htif:ident,
        $teif:ident, $dmeif:ident, $feif:ident, $isr:ident, $tcisr:ident,
        $htisr:ident)
    ),+$(,)*) => {
        $(
            impl<I: Instance> Stream for $name<I> {

                const NUMBER: usize = $number;

                #[inline(always)]
                fn clear_interrupts(&mut self) {
                    //NOTE(unsafe) Atomic write with no side-effects and we only access the bits
                    // that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.$ifcr.write(|w| w
                        .$tcif().set_bit() //Clear transfer complete interrupt flag
                        .$htif().set_bit() //Clear half transfer interrupt flag
                        .$teif().set_bit() //Clear transfer error interrupt flag
                        .$dmeif().set_bit() //Clear direct mode error interrupt flag
                        .$feif().set_bit() //Clear fifo error interrupt flag
                    );
                }

                #[inline(always)]
                fn clear_transfer_complete_interrupt(&mut self) {
                    //NOTE(unsafe) Atomic write with no side-effects and we only access the bits
                    // that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.$ifcr.write(|w| w.$tcif().set_bit());
                }

                #[inline(always)]
                fn clear_half_transfer_interrupt(&mut self) {
                    //NOTE(unsafe) Atomic write with no side-effects and we only access the bits
                    // that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.$ifcr.write(|w| w.$htif().set_bit());
                }

                #[inline(always)]
                fn clear_transfer_error_interrupt(&mut self) {
                    //NOTE(unsafe) Atomic write with no side-effects and we only access the bits
                    // that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.$ifcr.write(|w| w.$teif().set_bit());
                }

                #[inline(always)]
                fn clear_direct_mode_error_interrupt(&mut self) {
                    //NOTE(unsafe) Atomic write with no side-effects and we only access the bits
                    // that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.$ifcr.write(|w| w.$dmeif().set_bit());
                }

                #[inline(always)]
                fn clear_fifo_error_interrupt(&mut self) {
                    //NOTE(unsafe) Atomic write with no side-effects and we only access the bits
                    // that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.$ifcr.write(|w| w.$feif().set_bit());
                }

                #[inline(always)]
                fn get_transfer_complete_flag() -> bool {
                    //NOTE(unsafe) Atomic read with no side effects
                    let dma = unsafe { &*I::ptr() };
                    dma.$isr.read().$tcisr().bit_is_set()
                }

                #[inline(always)]
                fn get_half_transfer_flag() -> bool {
                    //NOTE(unsafe) Atomic read with no side effects
                    let dma = unsafe { &*I::ptr() };
                    dma.$isr.read().$htisr().bit_is_set()
                }

                #[inline(always)]
                unsafe fn set_peripheral_address(&mut self, value: u32) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = &*I::ptr();
                    dma.st[Self::NUMBER].par.write(|w| w.pa().bits(value));
                }

                #[inline(always)]
                unsafe fn set_memory_address(&mut self, value: u32) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = &*I::ptr();
                    dma.st[Self::NUMBER].m0ar.write(|w| w.m0a().bits(value));
                }

                #[inline(always)]
                fn get_memory_address(&self) -> u32 {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.st[Self::NUMBER].m0ar.read().m0a().bits()
                }

                #[inline(always)]
                unsafe fn set_memory_double_buffer_address(&mut self, value: u32) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = &*I::ptr();
                    dma.st[Self::NUMBER].m1ar.write(|w| w.m1a().bits(value));
                }

                #[inline(always)]
                fn get_memory_double_buffer_address(&self) -> u32 {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.st[Self::NUMBER].m1ar.read().m1a().bits()
                }

                #[inline(always)]
                fn set_number_of_transfers(&mut self, value: u16) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.st[Self::NUMBER].ndtr.write(|w| w.ndt().bits(value));
                }

                #[inline(always)]
                fn get_number_of_transfers() -> u16 {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.st[Self::NUMBER].ndtr.read().ndt().bits()
                }

                #[inline(always)]
                unsafe fn enable(&mut self) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = &*I::ptr();
                    dma.st[Self::NUMBER].cr.modify(|_, w| w.en().set_bit());
                }

                #[inline(always)]
                fn is_enabled() -> bool {
                    //NOTE(unsafe) Atomic read with no side effects
                    let dma = unsafe { &*I::ptr() };
                    dma.st[Self::NUMBER].cr.read().en().bit_is_set()
                }

                fn disable(&mut self) {
                    if Self::is_enabled() {
                        //NOTE(unsafe) We only access the registers that belongs to the StreamX
                        let dma = unsafe { &*I::ptr() };

                        // Aborting an on-going transfer might cause interrupts to fire, disable
                        // them
                        let (tc, ht, te, dm) = Self::get_interrupts_enable();
                        self
                            .set_interrupts_enable(false, false, false, false);

                        dma.st[Self::NUMBER].cr.modify(|_, w| w.en().clear_bit());
                        while Self::is_enabled() {}

                        self.clear_interrupts();
                        self.set_interrupts_enable(tc, ht, te, dm);
                    }
                }

                #[inline(always)]
                fn set_request_line(&mut self, request_line: u8) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dmamux = unsafe { &*I::mux_ptr() };
                    unsafe {
                        dmamux.ccr[Self::NUMBER + I::DMA_MUX_STREAM_OFFSET]
                            .modify(|_, w| w.dmareq_id().bits(request_line));
                    }
                }

                #[inline(always)]
                fn set_priority(&mut self, priority: config::Priority) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.st[Self::NUMBER].cr.modify(|_, w| w.pl().bits(priority.bits()));
                }

                #[inline(always)]
                unsafe fn set_memory_size(&mut self, size: u8) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = &*I::ptr();
                    dma.st[Self::NUMBER].cr.modify(|_, w| w.msize().bits(size));
                }

                #[inline(always)]
                unsafe fn set_peripheral_size(&mut self, size: u8) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = &*I::ptr();
                    dma.st[Self::NUMBER].cr.modify(|_, w| w.psize().bits(size));
                }

                #[inline(always)]
                fn set_memory_increment(&mut self, increment: bool) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.st[Self::NUMBER].cr.modify(|_, w| w.minc().bit(increment));
                }

                #[inline(always)]
                fn set_peripheral_increment(&mut self, increment: bool) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.st[Self::NUMBER].cr.modify(|_, w| w.pinc().bit(increment));
                }

                #[inline(always)]
                fn set_direction<D: Direction>(&mut self, direction: D) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.st[Self::NUMBER].cr.modify(|_, w| unsafe { w.dir().bits(direction.bits()) });
                }

                #[inline(always)]
                #[cfg(not(feature = "rm0455"))]
                fn set_trbuff(&mut self, trbuff: bool) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.st[Self::NUMBER].cr.modify(|_, w| w.trbuff().bit(trbuff));
                }

                #[inline(always)]
                fn set_interrupts_enable(
                    &mut self,
                    transfer_complete: bool,
                    half_transfer: bool,
                    transfer_error: bool,
                    direct_mode_error: bool,
                )
                {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.st[Self::NUMBER].cr.modify(|_, w| w
                        .tcie().bit(transfer_complete)
                        .htie().bit(half_transfer)
                        .teie().bit(transfer_error)
                        .dmeie().bit(direct_mode_error)
                    );
                }

                #[inline(always)]
                fn get_interrupts_enable() -> (bool, bool, bool, bool) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    let cr = dma.st[Self::NUMBER].cr.read();
                    (cr.tcie().bit_is_set(), cr.htie().bit_is_set(),
                        cr.teie().bit_is_set(), cr.dmeie().bit_is_set())
                }

                #[inline(always)]
                fn set_transfer_complete_interrupt_enable(&mut self, transfer_complete_interrupt: bool) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.st[Self::NUMBER].cr.modify(|_, w| w.tcie().bit(transfer_complete_interrupt));
                }

                #[inline(always)]
                fn set_half_transfer_interrupt_enable(&mut self, half_transfer_interrupt: bool) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.st[Self::NUMBER].cr.modify(|_, w| w.htie().bit(half_transfer_interrupt));
                }

                #[inline(always)]
                fn set_transfer_error_interrupt_enable(&mut self, transfer_error_interrupt: bool) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.st[Self::NUMBER].cr.modify(|_, w| w.teie().bit(transfer_error_interrupt));
                }

                #[inline(always)]
                fn set_direct_mode_error_interrupt_enable(&mut self, direct_mode_error_interrupt: bool) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.st[Self::NUMBER].cr.modify(|_, w| w.dmeie().bit(direct_mode_error_interrupt));
                }

                #[inline(always)]
                fn set_fifo_error_interrupt_enable(&mut self, fifo_error_interrupt: bool) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.st[Self::NUMBER].fcr.modify(|_, w| w.feie().bit(fifo_error_interrupt));
                }

                #[inline(always)]
                fn set_double_buffer(&mut self, double_buffer: bool) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.st[Self::NUMBER].cr.modify(|_, w| w.dbm().bit(double_buffer));
                }

                #[inline(always)]
                fn set_fifo_threshold(&mut self, fifo_threshold: config::FifoThreshold) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.st[Self::NUMBER].fcr.modify(|_, w| w.fth().bits(fifo_threshold.bits()));
                }

                #[inline(always)]
                fn set_fifo_enable(&mut self, fifo_enable: bool) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    //Register is actually direct mode disable rather than fifo enable
                    dma.st[Self::NUMBER].fcr.modify(|_, w| w.dmdis().bit(fifo_enable));
                }

                #[inline(always)]
                fn set_memory_burst(&mut self, memory_burst: config::BurstMode) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.st[Self::NUMBER].cr.modify(|_, w| w.mburst().bits(memory_burst.bits()));
                }

                #[inline(always)]
                fn set_peripheral_burst(&mut self, peripheral_burst: config::BurstMode) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.st[Self::NUMBER].cr.modify(|_, w| w.pburst().bits(peripheral_burst.bits()));
                }

                #[inline(always)]
                fn fifo_level() -> FifoLevel {
                    //NOTE(unsafe) Atomic read with no side effects
                    let dma = unsafe { &*I::ptr() };
                    dma.st[Self::NUMBER].fcr.read().fs().bits().into()
                }

                fn current_buffer() -> CurrentBuffer {
                    //NOTE(unsafe) Atomic read with no side effects
                    let dma = unsafe { &*I::ptr() };
                    if dma.st[Self::NUMBER].cr.read().ct().bit_is_set() {
                        CurrentBuffer::DoubleBuffer
                    } else {
                        CurrentBuffer::FirstBuffer
                    }
                }
            }
        )+
    };
}

dma_stream!(
    (
        Stream0, 0, lifcr, ctcif0, chtif0, cteif0, cdmeif0, cfeif0, lisr,
        tcif0, htif0
    ),
    (
        Stream1, 1, lifcr, ctcif1, chtif1, cteif1, cdmeif1, cfeif1, lisr,
        tcif1, htif1
    ),
    (
        Stream2, 2, lifcr, ctcif2, chtif2, cteif2, cdmeif2, cfeif2, lisr,
        tcif2, htif2
    ),
    (
        Stream3, 3, lifcr, ctcif3, chtif3, cteif3, cdmeif3, cfeif3, lisr,
        tcif3, htif3
    ),
    (
        Stream4, 4, hifcr, ctcif4, chtif4, cteif4, cdmeif4, cfeif4, hisr,
        tcif4, htif4
    ),
    (
        Stream5, 5, hifcr, ctcif5, chtif5, cteif5, cdmeif5, cfeif5, hisr,
        tcif5, htif5
    ),
    (
        Stream6, 6, hifcr, ctcif6, chtif6, cteif6, cdmeif6, cfeif6, hisr,
        tcif6, htif6
    ),
    (
        Stream7, 7, hifcr, ctcif7, chtif7, cteif7, cdmeif7, cfeif7, hisr,
        tcif7, htif7
    ),
);

macro_rules! peripheral_register_markers {
    ($($name:ident),+ $(,)*) => {
        $(
            /// Wrapper type that indicates which register of the contained
            /// peripheral to use for DMA.
            pub struct $name<T> (pub T);

            impl<T> Deref for $name<T> {
                type Target = T;

                #[inline(always)]
                fn deref(&self) -> &T {
                    &self.0
                }
            }
        )+
    };
}

peripheral_register_markers!(CCR1, CCR2, CCR3, CCR4, DMAR, ARR);

/// Type alias for the DMA Request Multiplexer
pub type DMAReq = pac::dmamux1::ccr::DMAREQ_ID_A;

type P2M = PeripheralToMemory;
type M2P = MemoryToPeripheral;

peripheral_target_address!(
    (pac::SPI1, rxdr, u8, P2M, DMAReq::SPI1_RX_DMA),
    (pac::SPI1, txdr, u8, M2P, DMAReq::SPI1_TX_DMA),
    (pac::SPI2, rxdr, u8, P2M, DMAReq::SPI2_RX_DMA),
    (pac::SPI2, txdr, u8, M2P, DMAReq::SPI2_TX_DMA),
    (pac::SPI3, rxdr, u8, P2M, DMAReq::SPI3_RX_DMA),
    (pac::SPI3, txdr, u8, M2P, DMAReq::SPI3_TX_DMA),
    (pac::SPI4, rxdr, u8, P2M, DMAReq::SPI4_RX_DMA),
    (pac::SPI4, txdr, u8, M2P, DMAReq::SPI4_TX_DMA),
    (pac::SPI5, rxdr, u8, P2M, DMAReq::SPI5_RX_DMA),
    (pac::SPI5, txdr, u8, M2P, DMAReq::SPI5_TX_DMA)
);

peripheral_target_address!(
    (pac::USART1, rdr(TRBUFF), u8, P2M, DMAReq::USART1_RX_DMA),
    (pac::USART1, tdr(TRBUFF), u8, M2P, DMAReq::USART1_TX_DMA),
);
