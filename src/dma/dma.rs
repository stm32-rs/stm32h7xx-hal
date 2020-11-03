//! DMA1 and DMA2

use super::{
    config,
    traits::sealed::{Bits, Sealed},
    traits::*,
    CurrentBuffer, DmaDirection, FifoLevel, MemoryToPeripheral,
    PeripheralToMemory,
};
use core::marker::PhantomData;

use crate::{
    pac::{self, DMA1, DMA2, DMAMUX1},
    rcc::{rec, rec::ResetEnable},
    //serial::{Rx, Tx},
    spi,
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

/// DMA interrupts
#[derive(Debug, Clone, Copy)]
pub struct DmaInterrupts {
    transfer_complete: bool,
    transfer_error: bool,
    half_transfer: bool,
    direct_mode_error: bool,
    fifo_error: bool,
}

/// Contains configuration for a DMA stream
#[derive(Debug, Clone, Copy)]
pub struct DmaConfig {
    pub(crate) priority: config::Priority,
    pub(crate) memory_increment: bool,
    pub(crate) peripheral_increment: bool,
    pub(crate) transfer_complete_interrupt: bool,
    pub(crate) half_transfer_interrupt: bool,
    pub(crate) transfer_error_interrupt: bool,
    pub(crate) direct_mode_error_interrupt: bool,
    pub(crate) fifo_error_interrupt: bool,
    pub(crate) circular_buffer: bool,
    pub(crate) double_buffer: bool,
    pub(crate) fifo_threshold: config::FifoThreshold,
    pub(crate) fifo_enable: bool,
    pub(crate) memory_burst: config::BurstMode,
    pub(crate) peripheral_burst: config::BurstMode,
}

impl Default for DmaConfig {
    fn default() -> Self {
        Self {
            priority: config::Priority::Medium,
            memory_increment: false,
            peripheral_increment: false,
            transfer_complete_interrupt: false,
            half_transfer_interrupt: false,
            transfer_error_interrupt: false,
            direct_mode_error_interrupt: false,
            fifo_error_interrupt: false,
            circular_buffer: false,
            double_buffer: false,
            fifo_threshold: config::FifoThreshold::QuarterFull,
            fifo_enable: false,
            memory_burst: config::BurstMode::NoBurst,
            peripheral_burst: config::BurstMode::NoBurst,
        }
    }
}

impl DoubleBufferedConfig for DmaConfig {
    #[inline(always)]
    fn is_double_buffered(&self) -> bool {
        self.double_buffer
    }

    #[inline(always)]
    fn is_fifo_enabled(&self) -> bool {
        self.fifo_enable
    }
}

impl DmaConfig {
    /// Set the priority.
    #[inline(always)]
    pub fn priority(mut self, priority: config::Priority) -> Self {
        self.priority = priority;
        self
    }

    /// Set the memory_increment.
    #[inline(always)]
    pub fn memory_increment(mut self, memory_increment: bool) -> Self {
        self.memory_increment = memory_increment;
        self
    }
    /// Set the peripheral_increment.
    #[inline(always)]
    pub fn peripheral_increment(mut self, peripheral_increment: bool) -> Self {
        self.peripheral_increment = peripheral_increment;
        self
    }
    /// Set the transfer_complete_interrupt.
    #[inline(always)]
    pub fn transfer_complete_interrupt(
        mut self,
        transfer_complete_interrupt: bool,
    ) -> Self {
        self.transfer_complete_interrupt = transfer_complete_interrupt;
        self
    }
    /// Set the half_transfer_interrupt.
    #[inline(always)]
    pub fn half_transfer_interrupt(
        mut self,
        half_transfer_interrupt: bool,
    ) -> Self {
        self.half_transfer_interrupt = half_transfer_interrupt;
        self
    }
    /// Set the transfer_error_interrupt.
    #[inline(always)]
    pub fn transfer_error_interrupt(
        mut self,
        transfer_error_interrupt: bool,
    ) -> Self {
        self.transfer_error_interrupt = transfer_error_interrupt;
        self
    }
    /// Set the direct_mode_error_interrupt.
    #[inline(always)]
    pub fn direct_mode_error_interrupt(
        mut self,
        direct_mode_error_interrupt: bool,
    ) -> Self {
        self.direct_mode_error_interrupt = direct_mode_error_interrupt;
        self
    }
    /// Set the fifo_error_interrupt.
    #[inline(always)]
    pub fn fifo_error_interrupt(mut self, fifo_error_interrupt: bool) -> Self {
        self.fifo_error_interrupt = fifo_error_interrupt;
        self
    }
    /// Set the circular_buffer.
    #[inline(always)]
    pub fn circular_buffer(mut self, circular_buffer: bool) -> Self {
        self.circular_buffer = circular_buffer;
        self
    }
    /// Set the double_buffer.
    #[inline(always)]
    pub fn double_buffer(mut self, double_buffer: bool) -> Self {
        self.double_buffer = double_buffer;
        self
    }
    /// Set the fifo_threshold.
    #[inline(always)]
    pub fn fifo_threshold(
        mut self,
        fifo_threshold: config::FifoThreshold,
    ) -> Self {
        self.fifo_threshold = fifo_threshold;
        self
    }
    /// Set the fifo_enable.
    #[inline(always)]
    pub fn fifo_enable(mut self, fifo_enable: bool) -> Self {
        self.fifo_enable = fifo_enable;
        self
    }
    /// Set the memory_burst.
    #[inline(always)]
    pub fn memory_burst(mut self, memory_burst: config::BurstMode) -> Self {
        self.memory_burst = memory_burst;
        self
    }
    /// Set the peripheral_burst.
    #[inline(always)]
    pub fn peripheral_burst(
        mut self,
        peripheral_burst: config::BurstMode,
    ) -> Self {
        self.peripheral_burst = peripheral_burst;
        self
    }
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
                type Config = DmaConfig;
                type Interrupts = DmaInterrupts;

                fn apply_config(&mut self, config: DmaConfig) {
                    self.set_priority(config.priority);
                    self.set_memory_increment(config.memory_increment);
                    self
                        .set_peripheral_increment(config.peripheral_increment);
                    self.set_transfer_complete_interrupt_enable(
                        config.transfer_complete_interrupt
                    );
                    self.set_half_transfer_interrupt_enable(
                        config.half_transfer_interrupt
                    );
                    self.set_transfer_error_interrupt_enable(
                        config.transfer_error_interrupt
                    );
                    self.set_direct_mode_error_interrupt_enable(
                        config.direct_mode_error_interrupt
                    );
                    self
                        .set_fifo_error_interrupt_enable(config.fifo_error_interrupt);
                    self.set_circular_buffer(config.circular_buffer);
                    self.set_double_buffer(config.double_buffer);
                    self.set_fifo_threshold(config.fifo_threshold);
                    self.set_fifo_enable(config.fifo_enable);
                    self.set_memory_burst(config.memory_burst);
                    self.set_peripheral_burst(config.peripheral_burst);
                }

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
                fn clear_half_transfer_interrupt(&mut self) {
                    //NOTE(unsafe) Atomic write with no side-effects and we only access the bits
                    // that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.$ifcr.write(|w| w.$htif().set_bit());
                }

                #[inline(always)]
                fn clear_transfer_complete_interrupt(&mut self) {
                    //NOTE(unsafe) Atomic write with no side-effects and we only access the bits
                    // that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.$ifcr.write(|w| w.$tcif().set_bit());
                }

                #[inline(always)]
                fn clear_transfer_error_interrupt(&mut self) {
                    //NOTE(unsafe) Atomic write with no side-effects and we only access the bits
                    // that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.$ifcr.write(|w| w.$teif().set_bit());
                }

                #[inline(always)]
                fn get_half_transfer_flag() -> bool {
                    //NOTE(unsafe) Atomic read with no side effects
                    let dma = unsafe { &*I::ptr() };
                    dma.$isr.read().$htisr().bit_is_set()
                }

                #[inline(always)]
                fn get_transfer_complete_flag() -> bool {
                    //NOTE(unsafe) Atomic read with no side effects
                    let dma = unsafe { &*I::ptr() };
                    dma.$isr.read().$tcisr().bit_is_set()
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
                        let interrupts = Self::get_interrupts_enable();
                        self.disable_interrupts();

                        dma.st[Self::NUMBER].cr.modify(|_, w| w.en().clear_bit());
                        while Self::is_enabled() {}

                        self.clear_interrupts();
                        self.enable_interrupts(interrupts);
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
                fn disable_interrupts(&mut self) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.st[Self::NUMBER].cr.modify(|_, w| w
                                                   .tcie().clear_bit()
                                                   .teie().clear_bit()
                                                   .htie().clear_bit()
                                                   .dmeie().clear_bit()
                    );
                    dma.st[Self::NUMBER].fcr.modify(|_, w| w.feie().clear_bit());
                }

                #[inline(always)]
                fn enable_interrupts(&mut self, interrupt: Self::Interrupts) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.st[Self::NUMBER].cr.modify(|_, w| w
                                                   .tcie().bit(interrupt.transfer_complete)
                                                   .htie().bit(interrupt.half_transfer)
                                                   .teie().bit(interrupt.transfer_error)
                                                   .dmeie().bit(interrupt.direct_mode_error)
                    );
                    dma.st[Self::NUMBER].fcr.modify(|_, w| w.feie().bit(interrupt.fifo_error));
                }

                #[inline(always)]
                fn get_interrupts_enable() -> Self::Interrupts {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    let cr = dma.st[Self::NUMBER].cr.read();
                    let fcr = dma.st[Self::NUMBER].fcr.read();

                    DmaInterrupts {
                        transfer_complete: cr.tcie().bit_is_set(),
                        half_transfer: cr.htie().bit_is_set(),
                        transfer_error: cr.teie().bit_is_set(),
                        direct_mode_error: cr.dmeie().bit_is_set(),
                        fifo_error: fcr.feie().bit_is_set()
                    }
                }

                #[inline(always)]
                fn set_half_transfer_interrupt_enable(&mut self, half_transfer_interrupt: bool) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.st[Self::NUMBER].cr.modify(|_, w| w.htie().bit(half_transfer_interrupt));
                }

                #[inline(always)]
                fn set_transfer_complete_interrupt_enable(&mut self, transfer_complete_interrupt: bool) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.st[Self::NUMBER].cr.modify(|_, w| w.tcie().bit(transfer_complete_interrupt));
                }

                #[inline(always)]
                fn set_transfer_error_interrupt_enable(&mut self, transfer_error_interrupt: bool) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.st[Self::NUMBER].cr.modify(|_, w| w.teie().bit(transfer_error_interrupt));
                }

            }

            impl<I: Instance> DoubleBufferedStream for $name<I> {
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
                fn set_direction(&mut self, direction: DmaDirection) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.st[Self::NUMBER].cr.modify(|_, w| unsafe {
                        match direction {
                            DmaDirection::PeripheralToMemory => w.dir().bits(0),
                            DmaDirection::MemoryToPeripheral => w.dir().bits(1),
                            DmaDirection::MemoryToMemory => w.dir().bits(2),
                        }
                    });
                }

                #[inline(always)]
                #[cfg(not(feature = "rm0455"))]
                fn set_trbuff(&mut self, trbuff: bool) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.st[Self::NUMBER].cr.modify(|_, w| w.trbuff().bit(trbuff));
                }

                #[inline(always)]
                fn set_circular_buffer(&mut self, circular_buffer: bool) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.st[Self::NUMBER].cr.modify(|_, w| w.circ().bit(circular_buffer));
                }

                #[inline(always)]
                fn set_double_buffer(&mut self, double_buffer: bool) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.st[Self::NUMBER].cr.modify(|_, w| w.dbm().bit(double_buffer));
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

            impl<I: Instance> $name<I> {
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
            }

            impl<I: Instance> $name<I> {
                #[inline(always)]
                pub fn fifo_level() -> FifoLevel {
                    //NOTE(unsafe) Atomic read with no side effects
                    let dma = unsafe { &*I::ptr() };
                    dma.st[Self::NUMBER].fcr.read().fs().bits().into()
                }

                #[inline(always)]
                pub fn clear_half_transfer_interrupt(&mut self) {
                    //NOTE(unsafe) Atomic write with no side-effects and we only access the bits
                    // that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.$ifcr.write(|w| w.$htif().set_bit());
                }
                #[inline(always)]
                pub fn clear_direct_mode_error_interrupt(&mut self) {
                    //NOTE(unsafe) Atomic write with no side-effects and we only access the bits
                    // that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.$ifcr.write(|w| w.$dmeif().set_bit());
                }
                #[inline(always)]
                pub fn clear_fifo_error_interrupt(&mut self) {
                    //NOTE(unsafe) Atomic write with no side-effects and we only access the bits
                    // that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.$ifcr.write(|w| w.$feif().set_bit());
                }

                #[inline(always)]
                pub fn get_half_transfer_flag() -> bool {
                    //NOTE(unsafe) Atomic read with no side effects
                    let dma = unsafe { &*I::ptr() };
                    dma.$isr.read().$htisr().bit_is_set()
                }

                #[inline(always)]
                pub fn set_half_transfer_interrupt_enable(&mut self, half_transfer_interrupt: bool) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.st[Self::NUMBER].cr.modify(|_, w| w.htie().bit(half_transfer_interrupt));
                }

                #[inline(always)]
                pub fn set_direct_mode_error_interrupt_enable(&mut self, direct_mode_error_interrupt: bool) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.st[Self::NUMBER].cr.modify(|_, w| w.dmeie().bit(direct_mode_error_interrupt));
                }

                #[inline(always)]
                pub fn set_fifo_error_interrupt_enable(&mut self, fifo_error_interrupt: bool) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    dma.st[Self::NUMBER].fcr.modify(|_, w| w.feie().bit(fifo_error_interrupt));
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
    (SPI: pac::SPI1, rxdr, txdr, [u8, u16], DMAReq::SPI1_RX_DMA, DMAReq::SPI1_TX_DMA),
    (SPI: pac::SPI2, rxdr, txdr, [u8, u16], DMAReq::SPI2_RX_DMA, DMAReq::SPI2_TX_DMA),
    (SPI: pac::SPI3, rxdr, txdr, [u8, u16], DMAReq::SPI3_RX_DMA, DMAReq::SPI3_TX_DMA),
    (SPI: pac::SPI4, rxdr, txdr, [u8, u16], DMAReq::SPI4_RX_DMA, DMAReq::SPI4_TX_DMA),
    (SPI: pac::SPI5, rxdr, txdr, [u8, u16], DMAReq::SPI5_RX_DMA, DMAReq::SPI5_TX_DMA)
);

peripheral_target_address!(
    (pac::USART1, rdr(TRBUFF), u8, P2M, DMAReq::USART1_RX_DMA),
    (pac::USART1, tdr(TRBUFF), u8, M2P, DMAReq::USART1_TX_DMA),
);

peripheral_target_address!(
    (pac::SAI1, cha.dr, u32, M2P, DMAReq::SAI1A_DMA),
    (pac::SAI1, chb.dr, u32, P2M, DMAReq::SAI1B_DMA),
    (pac::SAI2, cha.dr, u32, M2P, DMAReq::SAI2A_DMA),
    (pac::SAI2, chb.dr, u32, P2M, DMAReq::SAI2B_DMA),
    (pac::SAI3, cha.dr, u32, M2P, DMAReq::SAI3_A_DMA),
    (pac::SAI3, chb.dr, u32, P2M, DMAReq::SAI3_B_DMA),
);
