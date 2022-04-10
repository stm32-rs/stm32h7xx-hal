//! BDMA
//!
//! For RM0455 parts, only BDMA2 is implemented

use super::{
    config, traits,
    traits::{
        sealed::{Bits, Sealed},
        DoubleBufferedConfig, DoubleBufferedStream, TargetAddress,
    },
    CurrentBuffer, DmaDirection, MemoryToPeripheral, PeripheralToMemory,
};
use core::marker::PhantomData;

use crate::{
    i2c::I2c,
    pac,
    rcc::{rec, rec::ResetEnable},
    //serial::{Rx, Tx},
    spi,
};

#[cfg(not(feature = "rm0455"))]
use crate::pac::{BDMA, DMAMUX2};

#[cfg(feature = "rm0455")]
use crate::pac::{BDMA2, DMAMUX2};

use core::ops::Deref;

#[cfg(not(feature = "rm0455"))]
impl Sealed for BDMA {}
#[cfg(feature = "rm0455")]
impl Sealed for BDMA2 {}

/// Type aliases for register blocks
#[cfg(not(feature = "rm0455"))]
pub type BDMARegisterBlock = pac::bdma::RegisterBlock;
#[cfg(feature = "rm0455")]
pub type BDMARegisterBlock = pac::bdma2::RegisterBlock;
#[cfg(not(feature = "rm0455"))]
pub type BDMAStream = pac::bdma::CH;
#[cfg(feature = "rm0455")]
pub type BDMAStream = pac::bdma2::CH;
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

#[cfg(not(feature = "rm0455"))]
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

#[cfg(feature = "rm0455")]
impl Instance for BDMA2 {
    type Rec = rec::Bdma2;

    #[inline(always)]
    fn ptr() -> *const BDMARegisterBlock {
        BDMA2::ptr()
    }

    #[inline(always)]
    fn mux_ptr() -> *const DMAMUXRegisterBlock {
        DMAMUX2::ptr()
    }

    const DMA_MUX_STREAM_OFFSET: usize = 0;
}

/// BDMA interrupts
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BdmaInterrupts {
    transfer_complete: bool,
    transfer_error: bool,
    half_transfer: bool,
}

/// Contains the complete set of configuration for a DMA stream.
#[derive(Debug, Default, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BdmaConfig {
    pub(crate) priority: config::Priority,
    pub(crate) memory_increment: bool,
    pub(crate) peripheral_increment: bool,
    pub(crate) transfer_complete_interrupt: bool,
    pub(crate) half_transfer_interrupt: bool,
    pub(crate) transfer_error_interrupt: bool,
    pub(crate) double_buffer: bool,
}

impl DoubleBufferedConfig for BdmaConfig {
    #[inline(always)]
    fn is_double_buffered(&self) -> bool {
        self.double_buffer
    }

    #[inline(always)]
    fn is_fifo_enabled(&self) -> bool {
        false // No FIFO for BDMA
    }
}

impl BdmaConfig {
    /// Set the priority.
    #[inline(always)]
    #[must_use]
    pub fn priority(mut self, priority: config::Priority) -> Self {
        self.priority = priority;
        self
    }
    /// Set the memory_increment.
    #[inline(always)]
    #[must_use]
    pub fn memory_increment(mut self, memory_increment: bool) -> Self {
        self.memory_increment = memory_increment;
        self
    }
    /// Set the peripheral_increment.
    #[inline(always)]
    #[must_use]
    pub fn peripheral_increment(mut self, peripheral_increment: bool) -> Self {
        self.peripheral_increment = peripheral_increment;
        self
    }
    /// Set the transfer_complete_interrupt.
    #[inline(always)]
    #[must_use]
    pub fn transfer_complete_interrupt(
        mut self,
        transfer_complete_interrupt: bool,
    ) -> Self {
        self.transfer_complete_interrupt = transfer_complete_interrupt;
        self
    }
    /// Set the half_transfer_interrupt.
    #[inline(always)]
    #[must_use]
    pub fn half_transfer_interrupt(
        mut self,
        half_transfer_interrupt: bool,
    ) -> Self {
        self.half_transfer_interrupt = half_transfer_interrupt;
        self
    }
    /// Set the transfer_error_interrupt.
    #[inline(always)]
    #[must_use]
    pub fn transfer_error_interrupt(
        mut self,
        transfer_error_interrupt: bool,
    ) -> Self {
        self.transfer_error_interrupt = transfer_error_interrupt;
        self
    }
    /// Set the double_buffer.
    #[inline(always)]
    #[must_use]
    pub fn double_buffer(mut self, double_buffer: bool) -> Self {
        self.double_buffer = double_buffer;
        self
    }
}

/// Stream on the DMA controller.
pub struct StreamX<DMA, const S: u8> {
    _dma: PhantomData<DMA>,
}

impl<DMA, const S: u8> StreamX<DMA, S> {
    fn new() -> Self {
        Self { _dma: PhantomData }
    }
}

/// Stream 0 on the DMA controller.
pub type Stream0<DMA> = StreamX<DMA, 0>;
/// Stream 1 on the DMA controller.
pub type Stream1<DMA> = StreamX<DMA, 1>;
/// Stream 2 on the DMA controller.
pub type Stream2<DMA> = StreamX<DMA, 2>;
/// Stream 3 on the DMA controller.
pub type Stream3<DMA> = StreamX<DMA, 3>;
/// Stream 4 on the DMA controller.
pub type Stream4<DMA> = StreamX<DMA, 4>;
/// Stream 5 on the DMA controller.
pub type Stream5<DMA> = StreamX<DMA, 5>;
/// Stream 6 on the DMA controller.
pub type Stream6<DMA> = StreamX<DMA, 6>;
/// Stream 7 on the DMA controller.
pub type Stream7<DMA> = StreamX<DMA, 7>;

impl<DMA> Sealed for StreamX<DMA, 0> {}
impl<DMA> Sealed for StreamX<DMA, 1> {}
impl<DMA> Sealed for StreamX<DMA, 2> {}
impl<DMA> Sealed for StreamX<DMA, 3> {}
impl<DMA> Sealed for StreamX<DMA, 4> {}
impl<DMA> Sealed for StreamX<DMA, 5> {}
impl<DMA> Sealed for StreamX<DMA, 6> {}
impl<DMA> Sealed for StreamX<DMA, 7> {}

/// Alias for a tuple with all DMA streams.
pub struct StreamsTuple<DMA>(
    pub StreamX<DMA, 0>,
    pub StreamX<DMA, 1>,
    pub StreamX<DMA, 2>,
    pub StreamX<DMA, 3>,
    pub StreamX<DMA, 4>,
    pub StreamX<DMA, 5>,
    pub StreamX<DMA, 6>,
    pub StreamX<DMA, 7>,
);

impl<I: Instance> StreamsTuple<I> {
    /// Splits the DMA peripheral into streams.
    pub fn new(_regs: I, prec: I::Rec) -> Self {
        let _ = prec.enable().reset(); // drop
        Self(
            StreamX::new(),
            StreamX::new(),
            StreamX::new(),
            StreamX::new(),
            StreamX::new(),
            StreamX::new(),
            StreamX::new(),
            StreamX::new(),
        )
    }
}

// Private trait - streams in this module
trait InstanceStream {
    fn stream_clear_interrupts(&mut self);
    fn stream_clear_transfer_complete_flag(&mut self);
    fn stream_clear_transfer_complete_interrupt(&mut self);
    fn stream_clear_transfer_error_interrupt(&mut self);
    fn stream_get_transfer_complete_flag() -> bool;
    fn stream_get_half_transfer_flag() -> bool;
    fn stream_clear_half_transfer_interrupt(&mut self);
}

impl<I: Instance, const S: u8> StreamX<I, S> {
    unsafe fn stream() -> &'static BDMAStream {
        &(*I::ptr()).ch[S as usize]
    }
    unsafe fn dmamux_ccr() -> &'static pac::dmamux2::CCR {
        let dmamux = &*I::mux_ptr();
        &dmamux.ccr[S as usize + I::DMA_MUX_STREAM_OFFSET]
    }
}

impl<I: Instance, const S: u8> traits::Stream for StreamX<I, S>
where
    Self: InstanceStream + Sealed,
{
    type Config = BdmaConfig;
    type Interrupts = BdmaInterrupts;

    fn apply_config(&mut self, config: BdmaConfig) {
        self.set_priority(config.priority);
        self.set_memory_increment(config.memory_increment);
        self.set_peripheral_increment(config.peripheral_increment);
        self.set_transfer_complete_interrupt_enable(
            config.transfer_complete_interrupt,
        );
        self.set_half_transfer_interrupt_enable(config.half_transfer_interrupt);
        self.set_transfer_error_interrupt_enable(
            config.transfer_error_interrupt,
        );
        self.set_double_buffer(config.double_buffer);
    }

    #[inline(always)]
    fn clear_interrupts(&mut self) {
        self.stream_clear_interrupts()
    }
    #[inline(always)]
    fn clear_transfer_complete_flag(&mut self) {
        self.stream_clear_transfer_complete_flag()
    }
    #[inline(always)]
    fn clear_transfer_complete_interrupt(&mut self) {
        self.stream_clear_transfer_complete_interrupt();
    }
    #[inline(always)]
    fn clear_transfer_error_interrupt(&mut self) {
        self.stream_clear_transfer_error_interrupt();
    }
    #[inline(always)]
    fn get_transfer_complete_flag() -> bool {
        Self::stream_get_transfer_complete_flag()
    }

    #[inline(always)]
    unsafe fn enable(&mut self) {
        //NOTE(unsafe) We only access the registers that belongs to the StreamX
        Self::stream().cr.modify(|_, w| w.en().set_bit());
    }
    #[inline(always)]
    fn is_enabled() -> bool {
        //NOTE(unsafe) Atomic read with no side effects
        unsafe { Self::stream() }.cr.read().en().bit_is_set()
    }
    fn disable(&mut self) {
        if Self::is_enabled() {
            // Aborting an on-going transfer might cause interrupts to fire, disable
            // them
            let interrupts = Self::get_interrupts_enable();
            self.disable_interrupts();

            //NOTE(unsafe) We only access the registers that belongs to the StreamX
            unsafe { Self::stream() }
                .cr
                .modify(|_, w| w.en().clear_bit());
            while Self::is_enabled() {}

            self.clear_interrupts();
            self.enable_interrupts(interrupts);
        }
    }

    #[inline(always)]
    fn set_request_line(&mut self, request_line: u8) {
        //NOTE(unsafe) We only access the registers that belongs to the StreamX
        unsafe {
            Self::dmamux_ccr().modify(|_, w| w.dmareq_id().bits(request_line));
        }
    }
    #[inline(always)]
    fn set_priority(&mut self, priority: config::Priority) {
        //NOTE(unsafe) We only access the registers that belongs to the StreamX
        unsafe { Self::stream() }
            .cr
            .modify(|_, w| w.pl().bits(priority.bits()));
    }

    #[inline(always)]
    fn disable_interrupts(&mut self) {
        //NOTE(unsafe) We only access the registers that belongs to the StreamX
        let dmacr = &unsafe { Self::stream() }.cr;
        dmacr.modify(|_, w| {
            w.tcie().clear_bit().teie().clear_bit().htie().clear_bit()
        });
        let _ = dmacr.read();
        let _ = dmacr.read(); // Delay 2 peripheral clocks
    }
    #[inline(always)]
    fn enable_interrupts(&mut self, interrupt: Self::Interrupts) {
        //NOTE(unsafe) We only access the registers that belongs to the StreamX
        unsafe { Self::stream() }.cr.modify(|_, w| {
            w.tcie()
                .bit(interrupt.transfer_complete)
                .teie()
                .bit(interrupt.transfer_error)
                .htie()
                .bit(interrupt.half_transfer)
        });
    }

    #[inline(always)]
    fn get_interrupts_enable() -> Self::Interrupts {
        //NOTE(unsafe) We only access the registers that belongs to the StreamX
        let cr = unsafe { Self::stream() }.cr.read();

        BdmaInterrupts {
            transfer_complete: cr.tcie().bit_is_set(),
            half_transfer: cr.htie().bit_is_set(),
            transfer_error: cr.teie().bit_is_set(),
        }
    }

    #[inline(always)]
    fn set_transfer_complete_interrupt_enable(
        &mut self,
        transfer_complete_interrupt: bool,
    ) {
        //NOTE(unsafe) We only access the registers that belongs to the StreamX
        let dmacr = &unsafe { Self::stream() }.cr;
        dmacr.modify(|_, w| w.tcie().bit(transfer_complete_interrupt));
        let _ = dmacr.read();
        let _ = dmacr.read(); // Delay 2 peripheral clocks
    }
    #[inline(always)]
    fn set_transfer_error_interrupt_enable(
        &mut self,
        transfer_error_interrupt: bool,
    ) {
        //NOTE(unsafe) We only access the registers that belongs to the StreamX
        let dmacr = &unsafe { Self::stream() }.cr;
        dmacr.modify(|_, w| w.teie().bit(transfer_error_interrupt));
        let _ = dmacr.read();
        let _ = dmacr.read(); // Delay 2 peripheral clocks
    }
}

impl<I: Instance, const S: u8> DoubleBufferedStream for StreamX<I, S>
where
    Self: traits::Stream + Sealed + InstanceStream,
{
    #[inline(always)]
    unsafe fn set_peripheral_address(&mut self, value: usize) {
        //NOTE(unsafe) We only access the registers that belongs to the StreamX
        Self::stream().par.write(|w| w.pa().bits(value as u32));
    }

    #[inline(always)]
    unsafe fn set_memory_address(
        &mut self,
        buffer: CurrentBuffer,
        value: usize,
    ) {
        //NOTE(unsafe) We only access the registers that belongs to the StreamX
        match buffer {
            CurrentBuffer::Buffer0 => {
                Self::stream().m0ar.write(|w| w.ma().bits(value as u32))
            }
            CurrentBuffer::Buffer1 => {
                Self::stream().m1ar.write(|w| w.ma().bits(value as u32))
            }
        }
    }

    #[inline(always)]
    fn get_memory_address(&self, buffer: CurrentBuffer) -> usize {
        //NOTE(unsafe) We only access the registers that belongs to the StreamX
        let addr = match buffer {
            CurrentBuffer::Buffer0 => {
                unsafe { Self::stream() }.m0ar.read().ma().bits()
            }
            CurrentBuffer::Buffer1 => {
                unsafe { Self::stream() }.m1ar.read().ma().bits()
            }
        };
        addr as usize
    }

    #[inline(always)]
    fn set_number_of_transfers(&mut self, value: u16) {
        //NOTE(unsafe) We only access the registers that belongs to the StreamX
        unsafe { Self::stream() }
            .ndtr
            .write(|w| w.ndt().bits(value));
    }
    #[inline(always)]
    fn get_number_of_transfers() -> u16 {
        //NOTE(unsafe) We only access the registers that belongs to the StreamX
        unsafe { Self::stream() }.ndtr.read().ndt().bits()
    }
    #[inline(always)]
    unsafe fn set_memory_size(&mut self, size: u8) {
        //NOTE(unsafe) We only access the registers that belongs to the StreamX
        Self::stream().cr.modify(|_, w| w.msize().bits(size));
    }

    #[inline(always)]
    unsafe fn set_peripheral_size(&mut self, size: u8) {
        //NOTE(unsafe) We only access the registers that belongs to the StreamX
        Self::stream().cr.modify(|_, w| w.psize().bits(size));
    }

    #[inline(always)]
    fn set_memory_increment(&mut self, increment: bool) {
        //NOTE(unsafe) We only access the registers that belongs to the StreamX
        unsafe { Self::stream() }
            .cr
            .modify(|_, w| w.minc().bit(increment));
    }

    #[inline(always)]
    fn set_peripheral_increment(&mut self, increment: bool) {
        //NOTE(unsafe) We only access the registers that belongs to the StreamX
        unsafe { Self::stream() }
            .cr
            .modify(|_, w| w.pinc().bit(increment));
    }

    #[inline(always)]
    fn set_direction(&mut self, direction: DmaDirection) {
        //NOTE(unsafe) We only access the registers that belongs to the StreamX
        unsafe { Self::stream() }.cr.modify(|_, w| match direction {
            DmaDirection::PeripheralToMemory => {
                w.dir().peripheral_to_memory().mem2mem().disabled()
            }
            DmaDirection::MemoryToPeripheral => {
                w.dir().memory_to_peripheral().mem2mem().disabled()
            }
            DmaDirection::MemoryToMemory => {
                w.mem2mem().enabled().dir().clear_bit()
            }
        });
    }

    #[inline(always)]
    #[cfg(not(feature = "rm0455"))]
    fn set_trbuff(&mut self, _trbuff: bool) {
        // BDMA does not have a TRBUFF bit
    }

    #[inline(always)]
    fn set_circular_buffer(&mut self, circular_buffer: bool) {
        //NOTE(unsafe) We only access the registers that belongs to the StreamX
        unsafe { Self::stream() }
            .cr
            .modify(|_, w| w.circ().bit(circular_buffer));
    }

    #[inline(always)]
    fn set_double_buffer(&mut self, double_buffer: bool) {
        //NOTE(unsafe) We only access the registers that belongs to the StreamX
        unsafe { Self::stream() }
            .cr
            .modify(|_, w| w.dbm().bit(double_buffer));
    }

    #[inline(always)]
    fn get_current_buffer() -> CurrentBuffer {
        //NOTE(unsafe) Atomic read with no side effects
        if unsafe { Self::stream() }.cr.read().ct().bit_is_set() {
            CurrentBuffer::Buffer0
        } else {
            CurrentBuffer::Buffer1
        }
    }

    #[inline(always)]
    fn get_inactive_buffer() -> Option<CurrentBuffer> {
        //NOTE(unsafe) Atomic read with no side effects
        let cr = unsafe { Self::stream() }.cr.read();
        if cr.dbm().bit_is_set() {
            Some(if cr.ct().bit_is_set() {
                CurrentBuffer::Buffer0
            } else {
                CurrentBuffer::Buffer1
            })
        } else {
            None
        }
    }

    #[inline(always)]
    fn set_half_transfer_interrupt_enable(
        &mut self,
        half_transfer_interrupt: bool,
    ) {
        //NOTE(unsafe) We only access the registers that belongs to the StreamX
        let dmacr = &unsafe { Self::stream() }.cr;
        dmacr.modify(|_, w| w.htie().bit(half_transfer_interrupt));
        let _ = dmacr.read();
        let _ = dmacr.read(); // Delay 2 peripheral clocks
    }

    #[inline(always)]
    fn clear_half_transfer_interrupt(&mut self) {
        self.stream_clear_half_transfer_interrupt();
    }
    #[inline(always)]
    fn get_half_transfer_flag() -> bool {
        Self::stream_get_half_transfer_flag()
    }
}

// Macro that creates a struct representing a stream on either BDMA controller
//
// The implementation does the heavy lifting of mapping to the right fields on
// the stream
macro_rules! bdma_stream {
    ($(($name:ident, $number:expr,
        $ifcr:ident, $tcif:ident, $htif:ident, $teif:ident, $gif:ident,
        $isr:ident, $tcisr:ident, $htisr:ident, $teisr:ident, $gisr:ident)
    ),+$(,)*) => {
        $(
            impl<I: Instance> InstanceStream for StreamX<I, $number> {
                #[inline(always)]
                fn stream_clear_interrupts(&mut self) {
                    //NOTE(unsafe) Atomic write with no side-effects and we only access the bits
                    // that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.$ifcr.write(|w| w
                                    .$tcif().set_bit() //Clear transfer complete interrupt flag
                                    .$htif().set_bit() //Clear half transfer interrupt flag
                                    .$teif().set_bit() //Clear transfer error interrupt flag
                                    .$gif().set_bit() //Clear global interrupt flag
                    );
                    let _ = dma.$isr.read();
                    let _ = dma.$isr.read(); // Delay 2 peripheral clocks
                }

                #[inline(always)]
                fn stream_clear_transfer_complete_flag(&mut self) {
                    //NOTE(unsafe) Atomic write with no side-effects and we only access the bits
                    // that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.$ifcr.write(|w| w.$tcif().set_bit());
                }

                #[inline(always)]
                fn stream_clear_transfer_complete_interrupt(&mut self) {
                    self.stream_clear_transfer_complete_flag();
                    //NOTE(unsafe) Atomic read with no side-effects.
                    let dma = unsafe { &*I::ptr() };
                    let _ = dma.$isr.read();
                    let _ = dma.$isr.read(); // Delay 2 peripheral clocks
                }

                #[inline(always)]
                fn stream_clear_transfer_error_interrupt(&mut self) {
                    //NOTE(unsafe) Atomic write with no side-effects and we only access the bits
                    // that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.$ifcr.write(|w| w.$teif().set_bit());
                    let _ = dma.$isr.read();
                    let _ = dma.$isr.read(); // Delay 2 peripheral clocks
                }

                #[inline(always)]
                fn stream_get_transfer_complete_flag() -> bool {
                    //NOTE(unsafe) Atomic read with no side effects
                    let dma = unsafe { &*I::ptr() };
                    dma.$isr.read().$tcisr().bit_is_set()
                }

                #[inline(always)]
                fn stream_get_half_transfer_flag() -> bool {
                    //NOTE(unsafe) Atomic read with no side effects
                    let dma = unsafe { &*I::ptr() };
                    dma.$isr.read().$htisr().bit_is_set()
                }

                #[inline(always)]
                fn stream_clear_half_transfer_interrupt(&mut self) {
                    //NOTE(unsafe) Atomic write with no side-effects and we only access the bits
                    // that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.$ifcr.write(|w| w.$htif().set_bit());
                    let _ = dma.$isr.read();
                    let _ = dma.$isr.read(); // Delay 2 peripheral clocks
                }
            }
        )+
    };
}

#[cfg(not(feature = "rm0468"))]
bdma_stream!(
    // Note: the field names start from one, unlike the RM where they start from
    // zero. May need updating if it gets fixed upstream.
    (
        Stream0, 0, ifcr, ctcif1, chtif1, cteif1, cgif1, isr, tcif1, htif1,
        teif1, gif1
    ),
    (
        Stream1, 1, ifcr, ctcif2, chtif2, cteif2, cgif2, isr, tcif2, htif2,
        teif2, gif2
    ),
    (
        Stream2, 2, ifcr, ctcif3, chtif3, cteif3, cgif3, isr, tcif3, htif3,
        teif3, gif3
    ),
    (
        Stream3, 3, ifcr, ctcif4, chtif4, cteif4, cgif4, isr, tcif4, htif4,
        teif4, gif4
    ),
    (
        Stream4, 4, ifcr, ctcif5, chtif5, cteif5, cgif5, isr, tcif5, htif5,
        teif5, gif5
    ),
    (
        Stream5, 5, ifcr, ctcif6, chtif6, cteif6, cgif6, isr, tcif6, htif6,
        teif6, gif6
    ),
    (
        Stream6, 6, ifcr, ctcif7, chtif7, cteif7, cgif7, isr, tcif7, htif7,
        teif7, gif7
    ),
    (
        Stream7, 7, ifcr, ctcif8, chtif8, cteif8, cgif8, isr, tcif8, htif8,
        teif8, gif8
    ),
);
#[cfg(feature = "rm0468")]
bdma_stream!(
    // For this sub-familiy, the field names do match the RM.
    (
        Stream0, 0, ifcr, ctcif0, chtif0, cteif0, cgif0, isr, tcif0, htif0,
        teif0, gif0
    ),
    (
        Stream1, 1, ifcr, ctcif1, chtif1, cteif1, cgif1, isr, tcif1, htif1,
        teif1, gif1
    ),
    (
        Stream2, 2, ifcr, ctcif2, chtif2, cteif2, cgif2, isr, tcif2, htif2,
        teif2, gif2
    ),
    (
        Stream3, 3, ifcr, ctcif3, chtif3, cteif3, cgif3, isr, tcif3, htif3,
        teif3, gif3
    ),
    (
        Stream4, 4, ifcr, ctcif4, chtif4, cteif4, cgif4, isr, tcif4, htif4,
        teif4, gif4
    ),
    (
        Stream5, 5, ifcr, ctcif5, chtif5, cteif5, cgif5, isr, tcif5, htif5,
        teif5, gif5
    ),
    (
        Stream6, 6, ifcr, ctcif6, chtif6, cteif6, cgif6, isr, tcif6, htif6,
        teif6, gif6
    ),
    (
        Stream7, 7, ifcr, ctcif7, chtif7, cteif7, cgif7, isr, tcif7, htif7,
        teif7, gif7
    ),
);

/// Type alias for the DMA Request Multiplexer
///
pub type DMAReq = pac::dmamux2::ccr::DMAREQ_ID_A;

type P2M = PeripheralToMemory;
type M2P = MemoryToPeripheral;

peripheral_target_address!(
    (pac::LPUART1, rdr, u8, P2M, DMAReq::LPUART1_RX_DMA),
    (pac::LPUART1, tdr, u8, M2P, DMAReq::LPUART1_TX_DMA),
    (
        SPI: pac::SPI6,
        rxdr,
        txdr,
        [u8, u16],
        DMAReq::SPI6_RX_DMA,
        DMAReq::SPI6_TX_DMA
    ),
    (HAL: I2c<pac::I2C4>, rxdr, u8, P2M, DMAReq::I2C4_RX_DMA),
    (HAL: I2c<pac::I2C4>, txdr, u8, M2P, DMAReq::I2C4_TX_DMA),
);

#[cfg(not(feature = "rm0455"))]
peripheral_target_address!(
    (pac::SAI4, cha.dr, u32, M2P, DMAReq::SAI4_A_DMA),
    (pac::SAI4, chb.dr, u32, P2M, DMAReq::SAI4_B_DMA),
);
