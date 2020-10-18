//! BDMA

use super::{
    config,
    traits::sealed::{Bits, Sealed},
    traits::*,
    CurrentBuffer, DmaDirection, FifoLevel, MemoryToPeripheral,
    PeripheralToMemory,
};
use core::marker::PhantomData;

use crate::{
    i2c::I2c,
    pac::{self, BDMA, DMAMUX2},
    rcc::{rec, rec::ResetEnable},
    //serial::{Rx, Tx},
};

use core::ops::Deref;

impl Sealed for BDMA {}

/// Type aliases for register blocks
pub type BDMARegisterBlock = pac::bdma::RegisterBlock;
pub type DMAMUXRegisterBlock = pac::dmamux2::RegisterBlock;

/// Trait that represents an instance of a BDMA peripheral
pub trait Instance: Deref<Target = BDMARegisterBlock> + Sealed {
    type Rec: ResetEnable;

    /// Gives a pointer to the RegisterBlock.
    fn ptr() -> *const BDMARegisterBlock;

    /// Gives a pointer to the DMAMUX used for this DMA.
    fn mux_ptr() -> *const DMAMUXRegisterBlock;

    const DMA_MUX_STREAM_OFFSET: usize;
}

impl Instance for BDMA {
    type Rec = rec::Bdma;

    #[inline(always)]
    fn ptr() -> *const BDMARegisterBlock {
        BDMA::ptr()
    }

    #[inline(always)]
    fn mux_ptr() -> *const DMAMUXRegisterBlock {
        DMAMUX2::ptr()
    }

    const DMA_MUX_STREAM_OFFSET: usize = 0;
}

/// Stream 0 on BDMA
pub struct Stream0<DMA> {
    _dma: PhantomData<DMA>,
}
/// Stream 1 on BDMA
pub struct Stream1<DMA> {
    _dma: PhantomData<DMA>,
}
/// Stream 2 on BDMA
pub struct Stream2<DMA> {
    _dma: PhantomData<DMA>,
}
/// Stream 3 on BDMA
pub struct Stream3<DMA> {
    _dma: PhantomData<DMA>,
}
/// Stream 4 on BDMA
pub struct Stream4<DMA> {
    _dma: PhantomData<DMA>,
}
/// Stream 5 on BDMA
pub struct Stream5<DMA> {
    _dma: PhantomData<DMA>,
}
/// Stream 6 on BDMA
pub struct Stream6<DMA> {
    _dma: PhantomData<DMA>,
}
/// Stream 7 on BDMA
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
macro_rules! bdma_stream {
    ($(($name:ident, $number:expr,
        $ifcr:ident, $tcif:ident, $htif:ident, $teif:ident, $gif:ident,
        $isr:ident, $tcisr:ident, $htisr:ident)
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
                        .$gif().set_bit() //Clear global interrupt flag
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

                }

                #[inline(always)]
                fn clear_fifo_error_interrupt(&mut self) {

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
                    dma.ch[Self::NUMBER].par.write(|w| w.pa().bits(value));
                }

                #[inline(always)]
                unsafe fn set_memory_address(&mut self, value: u32) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = &*I::ptr();
                    dma.ch[Self::NUMBER].m0ar.write(|w| w.ma().bits(value));
                }

                #[inline(always)]
                fn get_memory_address(&self) -> u32 {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.ch[Self::NUMBER].m0ar.read().ma().bits()
                }

                #[inline(always)]
                unsafe fn set_memory_double_buffer_address(&mut self, value: u32) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = &*I::ptr();
                    dma.ch[Self::NUMBER].m1ar.write(|w| w.ma().bits(value));
                }

                #[inline(always)]
                fn get_memory_double_buffer_address(&self) -> u32 {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.ch[Self::NUMBER].m1ar.read().ma().bits()
                }

                #[inline(always)]
                fn set_number_of_transfers(&mut self, value: u16) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.ch[Self::NUMBER].ndtr.write(|w| w.ndt().bits(value));
                }

                #[inline(always)]
                fn get_number_of_transfers() -> u16 {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.ch[Self::NUMBER].ndtr.read().ndt().bits()
                }

                #[inline(always)]
                unsafe fn enable(&mut self) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = &*I::ptr();
                    dma.ch[Self::NUMBER].cr.modify(|_, w| w.en().set_bit());
                }

                #[inline(always)]
                fn is_enabled() -> bool {
                    //NOTE(unsafe) Atomic read with no side effects
                    let dma = unsafe { &*I::ptr() };
                    dma.ch[Self::NUMBER].cr.read().en().bit_is_set()
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

                        dma.ch[Self::NUMBER].cr.modify(|_, w| w.en().clear_bit());
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
                    dma.ch[Self::NUMBER].cr.modify(|_, w| w.pl().bits(priority.bits()));
                }

                #[inline(always)]
                unsafe fn set_memory_size(&mut self, size: u8) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = &*I::ptr();
                    dma.ch[Self::NUMBER].cr.modify(|_, w| w.msize().bits(size));
                }

                #[inline(always)]
                unsafe fn set_peripheral_size(&mut self, size: u8) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = &*I::ptr();
                    dma.ch[Self::NUMBER].cr.modify(|_, w| w.psize().bits(size));
                }

                #[inline(always)]
                fn set_memory_increment(&mut self, increment: bool) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.ch[Self::NUMBER].cr.modify(|_, w| w.minc().bit(increment));
                }

                #[inline(always)]
                fn set_peripheral_increment(&mut self, increment: bool) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.ch[Self::NUMBER].cr.modify(|_, w| w.pinc().bit(increment));
                }

                #[inline(always)]
                fn set_direction(&mut self, direction: DmaDirection) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.ch[Self::NUMBER].cr.modify(|_, w| {
                        match direction {
                            DmaDirection::PeripheralToMemory =>
                                w.dir().peripheral_to_memory().mem2mem().disabled(),
                            DmaDirection::MemoryToPeripheral =>
                                w.dir().memory_to_peripheral().mem2mem().disabled(),
                            DmaDirection::MemoryToMemory =>
                                w.mem2mem().enabled().dir().clear_bit(),
                        }
                    });
                }

                #[inline(always)]
                #[cfg(not(feature = "rm0455"))]
                fn set_trbuff(&mut self, _trbuff: bool) {

                }

                #[inline(always)]
                fn set_interrupts_enable(
                    &mut self,
                    transfer_complete: bool,
                    half_transfer: bool,
                    transfer_error: bool,
                    _direct_mode_error: bool,
                )
                {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.ch[Self::NUMBER].cr.modify(|_, w| w
                        .tcie().bit(transfer_complete)
                        .htie().bit(half_transfer)
                        .teie().bit(transfer_error)
                                                   // No dmeie
                    );
                }

                #[inline(always)]
                fn get_interrupts_enable() -> (bool, bool, bool, bool) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    let cr = dma.ch[Self::NUMBER].cr.read();
                    (cr.tcie().bit_is_set(), cr.htie().bit_is_set(),
                        cr.teie().bit_is_set(), false) // No dmeie
                }

                #[inline(always)]
                fn set_transfer_complete_interrupt_enable(&mut self, transfer_complete_interrupt: bool) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.ch[Self::NUMBER].cr.modify(|_, w| w.tcie().bit(transfer_complete_interrupt));
                }

                #[inline(always)]
                fn set_half_transfer_interrupt_enable(&mut self, half_transfer_interrupt: bool) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.ch[Self::NUMBER].cr.modify(|_, w| w.htie().bit(half_transfer_interrupt));
                }

                #[inline(always)]
                fn set_transfer_error_interrupt_enable(&mut self, transfer_error_interrupt: bool) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.ch[Self::NUMBER].cr.modify(|_, w| w.teie().bit(transfer_error_interrupt));
                }

                #[inline(always)]
                fn set_direct_mode_error_interrupt_enable(&mut self, _direct_mode_error_interrupt: bool) {

                }

                #[inline(always)]
                fn set_fifo_error_interrupt_enable(&mut self, _fifo_error_interrupt: bool) {

                }

                #[inline(always)]
                fn set_double_buffer(&mut self, double_buffer: bool) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.ch[Self::NUMBER].cr.modify(|_, w| w.dbm().bit(double_buffer));
                }

                #[inline(always)]
                fn set_fifo_threshold(&mut self, _fifo_threshold: config::FifoThreshold) {

                }

                #[inline(always)]
                fn set_fifo_enable(&mut self, _fifo_enable: bool) {

                }

                #[inline(always)]
                fn set_memory_burst(&mut self, _memory_burst: config::BurstMode) {

                }

                #[inline(always)]
                fn set_peripheral_burst(&mut self, _peripheral_burst: config::BurstMode) {

                }

                #[inline(always)]
                fn fifo_level() -> FifoLevel {
                    FifoLevel::Invalid
                }

                fn current_buffer() -> CurrentBuffer {
                    //NOTE(unsafe) Atomic read with no side effects
                    let dma = unsafe { &*I::ptr() };
                    if dma.ch[Self::NUMBER].cr.read().ct().bit_is_set() {
                        CurrentBuffer::DoubleBuffer
                    } else {
                        CurrentBuffer::FirstBuffer
                    }
                }
            }
        )+
    };
}

bdma_stream!(
    // Note: the field names start from one, unlike the RM where they start from
    // zero. May need updating if it gets fixed upstream.
    (Stream0, 0, ifcr, ctcif1, chtif1, cteif1, cgif1, isr, tcif1, htif1),
    (Stream1, 1, ifcr, ctcif2, chtif2, cteif2, cgif2, isr, tcif2, htif2),
    (Stream2, 2, ifcr, ctcif3, chtif3, cteif3, cgif3, isr, tcif3, htif3),
    (Stream3, 3, ifcr, ctcif4, chtif4, cteif4, cgif4, isr, tcif4, htif4),
    (Stream4, 4, ifcr, ctcif5, chtif5, cteif5, cgif5, isr, tcif5, htif5),
    (Stream5, 5, ifcr, ctcif6, chtif6, cteif6, cgif6, isr, tcif6, htif6),
    (Stream6, 6, ifcr, ctcif7, chtif7, cteif7, cgif7, isr, tcif7, htif7),
    (Stream7, 7, ifcr, ctcif8, chtif8, cteif8, cgif8, isr, tcif8, htif8),
);

/// Type alias for the DMA Request Multiplexer
pub type DMAReq = pac::dmamux2::ccr::DMAREQ_ID_A;

type P2M = PeripheralToMemory;
type M2P = MemoryToPeripheral;

peripheral_target_address!(
    (pac::LPUART1, rdr, u8, P2M, DMAReq::LPUART1_RX_DMA),
    (pac::LPUART1, tdr, u8, M2P, DMAReq::LPUART1_TX_DMA),
    (pac::SPI6, rxdr, u8, P2M, DMAReq::SPI6_RX_DMA),
    (pac::SPI6, txdr, u8, M2P, DMAReq::SPI6_TX_DMA),
    (pac::I2C4, rxdr, u8, P2M, DMAReq::I2C4_RX_DMA),
    (pac::I2C4, txdr, u8, M2P, DMAReq::I2C4_TX_DMA),
    (INNER, I2c<pac::I2C4>, rxdr, u8, P2M, DMAReq::I2C4_RX_DMA),
    (INNER, I2c<pac::I2C4>, txdr, u8, M2P, DMAReq::I2C4_TX_DMA),
);
