//! Master DMA
//!
//!

use super::{
    config,
    traits::sealed::{Bits, Sealed},
    traits::*,
    MemoryToPeripheral, PeripheralToMemory,
};

use core::marker::PhantomData;
use core::mem;

use crate::{
    i2c::I2c,
    pac::{self, MDMA},
    rcc::{rec, rec::ResetEnable},
};

use core::ops::Deref;

impl Sealed for MDMA {}

/// Type aliases for register blocks
pub type MDMARegisterBlock = pac::mdma::RegisterBlock;
pub type DMAMUXRegisterBlock = pac::dmamux2::RegisterBlock;

/// Trait that represents an instance of a MDMA peripheral
pub trait Instance: Deref<Target = MDMARegisterBlock> + Sealed {
    type Rec: ResetEnable;

    /// Gives a pointer to the RegisterBlock.
    fn ptr() -> *const MDMARegisterBlock;
}

impl Instance for MDMA {
    type Rec = rec::Mdma;

    #[inline(always)]
    fn ptr() -> *const MDMARegisterBlock {
        MDMA::ptr()
    }
}

/// MDMA Source/Destination sizes
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub enum MdmaSize {
    /// Byte (8-bit)
    Byte = 0,
    /// Half-word (16-bit)
    HalfWord = 1,
    /// Word (32-bit)
    Word = 2,
    /// Double-word (64-bit)
    DoubleWord = 3,
}
impl MdmaSize {
    /// Returns MdmaSize from the size of a type. Undefined if
    /// embedded_dma::Word is implemented for types not 1, 2, 4 or 8 bytes long.
    pub fn from_type<T: Sized>() -> Self {
        match mem::size_of::<T>() {
            1 => MdmaSize::Byte,
            2 => MdmaSize::HalfWord,
            4 => MdmaSize::Word,
            8 => MdmaSize::DoubleWord,
            _ => unreachable!(),
        }
    }
}

/// MDMA increment mode
#[derive(Debug, Clone, Copy)]
pub enum MdmaIncrement {
    Fixed,
    /// Increment by one source/destination element each transfer
    Increment,
    /// Decrement by one source/destination element each transfer
    Decrement,
    /// Increment by the given offset each transfer. The offset must be larger
    /// or equal to the source/destination element size, otherwise the stream
    /// initialisation will panic
    IncrementWithOffset(MdmaSize),
    /// Decrement by the given offset each transfer. The offset must be larger
    /// or equal to the source/destination element size, otherwise the stream
    /// initialisation will panic
    DecrementWithOffset(MdmaSize),
}
impl Default for MdmaIncrement {
    fn default() -> Self {
        MdmaIncrement::Increment
    }
}

/// MDMA interrupts
#[derive(Debug, Clone, Copy)]
pub struct MdmaInterrupts {
    transfer_complete: bool,
    transfer_error: bool,
    block_transfer_complete: bool,
    block_repeat_transfer_complete: bool,
    channel_transfer_complete: bool,
}

/// Contains the complete set of configuration for a DMA stream.
#[derive(Debug, Default, Clone, Copy)]
pub struct MdmaConfig {
    pub(crate) priority: config::Priority,
    pub(crate) destination_increment: MdmaIncrement,
    pub(crate) source_increment: MdmaIncrement,
    pub(crate) transfer_complete_interrupt: bool,
    pub(crate) transfer_error_interrupt: bool,
    pub(crate) block_transfer_complete_interrupt: bool,
    pub(crate) block_repeat_transfer_complete_interrupt: bool,
    pub(crate) channel_transfer_complete_interrupt: bool,
}

impl MdmaConfig {
    /// Set the priority
    #[inline(always)]
    pub fn priority(mut self, priority: config::Priority) -> Self {
        self.priority = priority;
        self
    }
    /// Set the destination_increment
    #[inline(always)]
    pub fn destination_increment(
        mut self,
        destination_increment: MdmaIncrement,
    ) -> Self {
        self.destination_increment = destination_increment;
        self
    }
    /// Set the source_increment
    #[inline(always)]
    pub fn source_increment(mut self, source_increment: MdmaIncrement) -> Self {
        self.source_increment = source_increment;
        self
    }
    /// Set the transfer_complete_interrupt
    #[inline(always)]
    pub fn transfer_complete_interrupt(
        mut self,
        transfer_complete_interrupt: bool,
    ) -> Self {
        self.transfer_complete_interrupt = transfer_complete_interrupt;
        self
    }
    /// Set the transfer_error_interrupt
    #[inline(always)]
    pub fn transfer_error_interrupt(
        mut self,
        transfer_error_interrupt: bool,
    ) -> Self {
        self.transfer_error_interrupt = transfer_error_interrupt;
        self
    }
    /// Set the block_transfer_complete_interrupt
    #[inline(always)]
    pub fn block_transfer_complete_interrupt(
        mut self,
        block_transfer_complete_interrupt: bool,
    ) -> Self {
        self.block_transfer_complete_interrupt =
            block_transfer_complete_interrupt;
        self
    }
    /// Set the block_repeat_transfer_complete_interrupt
    #[inline(always)]
    pub fn block_repeat_transfer_complete_interrupt(
        mut self,
        block_repeat_transfer_complete_interrupt: bool,
    ) -> Self {
        self.block_repeat_transfer_complete_interrupt =
            block_repeat_transfer_complete_interrupt;
        self
    }
    /// Set the channel_transfer_complete_interrupt
    #[inline(always)]
    pub fn channel_transfer_complete_interrupt(
        mut self,
        channel_transfer_complete_interrupt: bool,
    ) -> Self {
        self.channel_transfer_complete_interrupt =
            channel_transfer_complete_interrupt;
        self
    }
}

/// Stream 0 on MDMA
pub struct Stream0<DMA> {
    _dma: PhantomData<DMA>,
}
/// Stream 1 on MDMA
pub struct Stream1<DMA> {
    _dma: PhantomData<DMA>,
}
/// Stream 2 on MDMA
pub struct Stream2<DMA> {
    _dma: PhantomData<DMA>,
}
/// Stream 3 on MDMA
pub struct Stream3<DMA> {
    _dma: PhantomData<DMA>,
}
/// Stream 4 on MDMA
pub struct Stream4<DMA> {
    _dma: PhantomData<DMA>,
}
/// Stream 5 on MDMA
pub struct Stream5<DMA> {
    _dma: PhantomData<DMA>,
}
/// Stream 6 on MDMA
pub struct Stream6<DMA> {
    _dma: PhantomData<DMA>,
}
/// Stream 7 on MDMA
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

// Macro that creates a struct representing a stream on the MDMA controller
//
// The implementation does the heavy lifting of mapping to the right fields on
// the stream
macro_rules! mdma_stream {
    ($(($name:ident, $channel:ident, $number:expr,
        $ifcr:ident, $tcif:ident, $htif:ident, $teif:ident, $gif:ident,
        $isr:ident, $tcisr:ident, $htisr:ident)
    ),+$(,)*) => {
        $(
            impl<I: Instance> Stream for $name<I> {

                const NUMBER: usize = $number;
                type Config = MdmaConfig;
                type Interrupts = MdmaInterrupts;

                fn apply_config(&mut self, config: MdmaConfig) {
                    self.set_priority(config.priority);

                    // We do not set DINCOS/SINCOS here
                    self.set_destination_increment(config.destination_increment);
                    self
                        .set_source_increment(config.source_increment);

                    self.set_transfer_complete_interrupt_enable(
                        config.transfer_complete_interrupt
                    );
                    self.set_transfer_error_interrupt_enable(
                        config.transfer_error_interrupt
                    );
                    self.set_block_transfer_complete_interrupt_enable(
                        config.block_transfer_complete_interrupt
                    );
                    self.set_block_repeat_transfer_complete_interrupt_enable(
                        config.block_repeat_transfer_complete_interrupt
                    );
                    self.set_channel_transfer_complete_interrupt_enable(
                        config.channel_transfer_complete_interrupt
                    );
                }

                #[inline(always)]
                fn clear_interrupts(&mut self) {
                    //NOTE(unsafe) Atomic write with no side-effects and we only access the bits
                    // that belongs to the StreamX
                    let mdma = unsafe { &*I::ptr() };
                    mdma.$channel.ifcr.write(|w| w
                        .cltcif().set_bit() //Clear transfer complete interrupt flag
                        .cteif().set_bit() //Clear transfer error interrupt flag

                        .cbtif().set_bit() //Clear block transfer complete flag
                        .cbrtif().set_bit() //Clear block repeat transfer complete flag
                        .cctcif().set_bit() //Clear channel transfer complete flag
                    );
                }

                #[inline(always)]
                fn clear_transfer_error_interrupt(&mut self) {
                    //NOTE(unsafe) Atomic write with no side-effects and we only access the bits
                    // that belongs to the StreamX
                    let mdma = unsafe { &*I::ptr() };
                    mdma.$channel.ifcr.write(|w| w.cteif().set_bit());
                }

                #[inline(always)]
                fn clear_transfer_complete_interrupt(&mut self) {
                    //NOTE(unsafe) Atomic write with no side-effects and we only access the bits
                    // that belongs to the StreamX
                    let mdma = unsafe { &*I::ptr() };
                    mdma.$channel.ifcr.write(|w| w.cltcif().set_bit());
                }

                #[inline(always)]
                fn get_transfer_complete_flag() -> bool {
                    //NOTE(unsafe) Atomic read with no side effects
                    let mdma = unsafe { &*I::ptr() };
                    mdma.$channel.isr.read().tcif().bit_is_set()
                }

                #[inline(always)]
                unsafe fn enable(&mut self) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let mdma = &*I::ptr();
                    mdma.$channel.cr.modify(|_, w| w.en().set_bit());

                    // If this channel is configured as software triggered, then
                    // we also active the request
                    if mdma.$channel.tcr.read().swrm().bit_is_set() {
                        mdma.$channel.cr.modify(|_, w| w.swrq().set_bit());
                    }
                }

                #[inline(always)]
                fn is_enabled() -> bool {
                    //NOTE(unsafe) Atomic read with no side effects
                    let mdma = unsafe { &*I::ptr() };
                    mdma.$channel.cr.read().en().bit_is_set()
                }

                fn disable(&mut self) {
                    if Self::is_enabled() {
                        //NOTE(unsafe) We only access the registers that belongs to the StreamX
                        let dma = unsafe { &*I::ptr() };

                        // Aborting an on-going transfer might cause interrupts to fire, disable
                        // them
                        let interrupts = Self::get_interrupts_enable();
                        self.disable_interrupts();

                        dma.$channel.cr.modify(|_, w| w.en().clear_bit());
                        while Self::is_enabled() {}

                        self.clear_interrupts();
                        self.enable_interrupts(interrupts);
                    }
                }

                #[inline(always)]
                fn set_request_line(&mut self, _request_line: u8) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX

                    // TODO: Request lines
                }

                #[inline(always)]
                fn set_priority(&mut self, priority: config::Priority) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let mdma = unsafe { &*I::ptr() };
                    mdma.$channel.cr.modify(|_, w| unsafe {
                        w.pl().bits(priority.bits())
                    });
                }

                #[inline(always)]
                fn disable_interrupts(&mut self) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let mdma = unsafe { &*I::ptr() };
                    mdma.$channel.cr.modify(|_, w| w
                                                   .tcie().clear_bit()
                                                   .teie().clear_bit()
                                                   .btie().clear_bit()
                                                   .brtie().clear_bit()
                                                   .ctcie().clear_bit()
                    );
                }

                #[inline(always)]
                fn enable_interrupts(&mut self, interrupt: Self::Interrupts) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let mdma = unsafe { &*I::ptr() };
                    mdma.$channel.cr.modify(|_, w| w
                                                   .tcie().bit(interrupt.transfer_complete)
                                                   .teie().bit(interrupt.transfer_error)
                                                   .btie().bit(interrupt.block_transfer_complete)
                                                   .brtie().bit(interrupt.block_repeat_transfer_complete)
                                                   .ctcie().bit(interrupt.channel_transfer_complete)
                    );
                }

                #[inline(always)]
                fn get_interrupts_enable() -> Self::Interrupts {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let dma = unsafe { &*I::ptr() };
                    let cr = dma.$channel.cr.read();

                    MdmaInterrupts {
                        transfer_complete: cr.tcie().bit_is_set(),
                        transfer_error: cr.teie().bit_is_set(),
                        block_transfer_complete: cr.btie().bit_is_set(),
                        block_repeat_transfer_complete: cr.brtie().bit_is_set(),
                        channel_transfer_complete: cr.teie().bit_is_set(),
                    }
                }

                #[inline(always)]
                fn set_transfer_complete_interrupt_enable(&mut self, transfer_complete_interrupt: bool) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let mdma = unsafe { &*I::ptr() };
                    mdma.$channel.cr.modify(|_, w| w.tcie().bit(transfer_complete_interrupt));
                }

                #[inline(always)]
                fn set_transfer_error_interrupt_enable(&mut self, transfer_error_interrupt: bool) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let mdma = unsafe { &*I::ptr() };
                    mdma.$channel.cr.modify(|_, w| w.teie().bit(transfer_error_interrupt));
                }
            }

            impl<I: Instance> MasterStream for $name<I> {
                #[inline(always)]
                unsafe fn set_source_address(&mut self, value: usize) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let mdma = &*I::ptr();
                    mdma.$channel.sar.write(|w| w.sar().bits(value as u32));
                    mdma.$channel.tbr.modify(|_,w| w.sbus().bit(
                        Self::is_ahb_port(value)
                    ));
                }

                #[inline(always)]
                unsafe fn set_destination_address(&mut self, value: usize) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let mdma = &*I::ptr();
                    mdma.$channel.dar.write(|w| w.dar().bits(value as u32));
                    mdma.$channel.tbr.modify(|_,w| w.dbus().bit(
                        Self::is_ahb_port(value)
                    ));
                }

                #[inline(always)]
                fn set_software_triggered(&mut self, sw_triggered: bool) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let mdma = unsafe { &*I::ptr() };
                    mdma.$channel.tcr.modify(|_, w| w.swrm().bit(sw_triggered));
                }

                #[inline(always)]
                fn set_transfer_bytes(&mut self, value: u8) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let mdma = unsafe { &*I::ptr() };
                    mdma.$channel.tcr.modify(|_, w| unsafe { w.tlen().bits(value - 1) });
                    mdma.$channel.bndtr.modify(|_, w| unsafe { w.bndt().bits(value as u32) });
                }

                /// Apply the configation structure to this
                /// stream. SINCOS/DINCOS are set based on the source_size /
                /// destination_size if not specified by the config structure.
                #[inline(always)]
                fn apply_config_with_size(&mut self,
                                              config: <Self as Stream>::Config,
                                              source_size: MdmaSize,
                                              destination_size: MdmaSize) {
                    self.apply_config(config);

                    //NOTE(unsafe) We only set the offset if it is larger or
                    // equal to the source/destination size
                    unsafe {
                        self.set_source_offset(match config.source_increment {
                            MdmaIncrement::IncrementWithOffset(source_offset) |
                            MdmaIncrement::DecrementWithOffset(source_offset) => {
                                assert!(source_offset >= source_size);

                                // TODO: If source/destination is AHB and DBURST
                                // =/ 000, destination address must be aligned
                                // with DINCOS size, else the result is
                                // unpredictable.

                                source_offset
                            },
                            _ => source_size
                        });
                        self.set_destination_offset(match config.destination_increment {
                            MdmaIncrement::IncrementWithOffset(destination_offset) |
                            MdmaIncrement::DecrementWithOffset(destination_offset) => {
                                assert!(destination_offset >= destination_size);

                                destination_offset
                            },
                            _ => destination_size
                        });
                    }
                }

                #[inline(always)]
                unsafe fn set_source_size(&mut self, size: MdmaSize) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let mdma = &*I::ptr();
                    mdma.$channel.tcr.modify(|_, w| w.ssize().bits(size as u8));
                }

                #[inline(always)]
                unsafe fn set_source_offset(&mut self, offset: MdmaSize) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let mdma = &*I::ptr();
                    mdma.$channel.tcr.modify(|_, w| w.sincos().bits(offset as u8));
                }

                #[inline(always)]
                unsafe fn set_destination_size(&mut self, size: MdmaSize) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let mdma = &*I::ptr();
                    mdma.$channel.tcr.modify(|_, w| w.dsize().bits(size as u8));
                }

                #[inline(always)]
                unsafe fn set_destination_offset(&mut self, offset: MdmaSize) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let mdma = &*I::ptr();
                    mdma.$channel.tcr.modify(|_, w| w.dincos().bits(offset as u8));
                }
            }

            impl<I: Instance> $name<I> {
                /// Returns true if the MDMA master must access `address` via
                /// the 32-bit AHB slave port on the Cortex-M7 crossbar, rather
                /// than the AXI matrix.
                ///
                /// This corresponds to the D0TCM, D1TCM and ITCM regions. These
                /// regions are not relocatable.
                #[inline(always)]
                pub fn is_ahb_port(address: usize) -> bool {
                    // On these parts is it possible to re-locate AXI SRAM to
                    // the Cortex-M7 crossbar bus. TODO: investigate how to
                    // handle this.
                    #[cfg(feature = "rm0468")]
                    panic!("MDMA not implemented for RM0468 parts yet!");

                    // 64kB ITCM
                    let is_itcm: bool = address < 0x0001_0000;
                    // 128kB DTCM
                    let is_dtcm: bool = (address >= 0x2000_0000 && address < 0x2002_0000);

                    is_itcm || is_dtcm
                }

                #[inline(always)]
                pub fn clear_block_transfer_complete_interrupt(&mut self) {
                    //NOTE(unsafe) Atomic write with no side-effects and we only access the bits
                    // that belongs to the StreamX
                    let mdma = unsafe { &*I::ptr() };
                    mdma.$channel.ifcr.write(|w| w.cbtif().set_bit());
                }

                #[inline(always)]
                pub fn get_block_transfer_complete_flag() -> bool {
                    //NOTE(unsafe) Atomic read with no side effects
                    let mdma = unsafe { &*I::ptr() };
                    mdma.$channel.isr.read().btif().bit_is_set()
                }

                #[inline(always)]
                pub fn clear_block_repeat_transfer_complete_interrupt(&mut self) {
                    //NOTE(unsafe) Atomic write with no side-effects and we only access the bits
                    // that belongs to the StreamX
                    let mdma = unsafe { &*I::ptr() };
                    mdma.$channel.ifcr.write(|w| w.cbrtif().set_bit());
                }

                #[inline(always)]
                pub fn get_block_repeat_transfer_complete_flag() -> bool {
                    //NOTE(unsafe) Atomic read with no side effects
                    let mdma = unsafe { &*I::ptr() };
                    mdma.$channel.isr.read().brtif().bit_is_set()
                }

                #[inline(always)]
                pub fn clear_channel_transfer_complete_interrupt(&mut self) {
                    //NOTE(unsafe) Atomic write with no side-effects and we only access the bits
                    // that belongs to the StreamX
                    let mdma = unsafe { &*I::ptr() };
                    mdma.$channel.ifcr.write(|w| w.cctcif().set_bit());
                }

                #[inline(always)]
                pub fn get_channel_transfer_complete_flag() -> bool {
                    //NOTE(unsafe) Atomic read with no side effects
                    let mdma = unsafe { &*I::ptr() };
                    mdma.$channel.isr.read().ctcif().bit_is_set()
                }

                #[inline(always)]
                pub fn set_block_transfer_complete_interrupt_enable(
                    &mut self, block_transfer_complete_interrupt: bool)
                {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let mdma = unsafe { &*I::ptr() };
                    mdma.$channel.cr.modify(|_, w| w.btie().bit(block_transfer_complete_interrupt));
                }

                #[inline(always)]
                pub fn set_block_repeat_transfer_complete_interrupt_enable(
                    &mut self, block_repeat_transfer_complete_interrupt: bool)
                {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let mdma = unsafe { &*I::ptr() };
                    mdma.$channel.cr.modify(|_, w| w.brtie().bit(block_repeat_transfer_complete_interrupt));
                }

                #[inline(always)]
                pub fn set_channel_transfer_complete_interrupt_enable(
                    &mut self, channel_transfer_complete_interrupt: bool)
                {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let mdma = unsafe { &*I::ptr() };
                    mdma.$channel.cr.modify(|_, w| w.ctcie().bit(channel_transfer_complete_interrupt));
                }

                #[inline(always)]
                pub fn set_destination_increment(&mut self, increment: MdmaIncrement) {
                    use MdmaIncrement::*;

                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let mdma = unsafe { &*I::ptr() };
                    mdma.$channel.tcr.modify(|_, w| unsafe {
                        w.dinc().bits(match increment {
                            Fixed => 0b00,
                            Increment | IncrementWithOffset(_) => 0b10,
                            Decrement | DecrementWithOffset(_) => 0b11,
                        })
                    });
                }

                #[inline(always)]
                pub fn set_source_increment(&mut self, increment: MdmaIncrement) {
                    use MdmaIncrement::*;

                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let mdma = unsafe { &*I::ptr() };
                    mdma.$channel.tcr.modify(|_, w| unsafe {
                        w.sinc().bits(match increment {
                            Fixed => 0b00,
                            Increment | IncrementWithOffset(_) => 0b10,
                            Decrement | DecrementWithOffset(_) => 0b11,
                        })
                    });
                }
            }
        )+
    };
}

mdma_stream!(
    // Note: the field names start from one, unlike the RM where they start from
    // zero. May need updating if it gets fixed upstream.
    (Stream0, ch0, 0, ifcr, ctcif1, chtif1, cteif1, cgif1, isr, tcif1, htif1),
    (Stream1, ch1, 1, ifcr, ctcif2, chtif2, cteif2, cgif2, isr, tcif2, htif2),
    (Stream2, ch2, 2, ifcr, ctcif3, chtif3, cteif3, cgif3, isr, tcif3, htif3),
    (Stream3, ch3, 3, ifcr, ctcif4, chtif4, cteif4, cgif4, isr, tcif4, htif4),
    (Stream4, ch4, 4, ifcr, ctcif5, chtif5, cteif5, cgif5, isr, tcif5, htif5),
    (Stream5, ch5, 5, ifcr, ctcif6, chtif6, cteif6, cgif6, isr, tcif6, htif6),
    (Stream6, ch6, 6, ifcr, ctcif7, chtif7, cteif7, cgif7, isr, tcif7, htif7),
    (Stream7, ch7, 7, ifcr, ctcif8, chtif8, cteif8, cgif8, isr, tcif8, htif8),
);

/// Type alias for the DMA Request Multiplexer
pub type DMAReq = pac::dmamux2::ccr::DMAREQ_ID_A;

type P2M = PeripheralToMemory;
type M2P = MemoryToPeripheral;

// peripheral_target_address!((
//     QSPI: pac::QUADSPI,
//     rdr,
//     u8,
//     P2M,
//     DMAReq::USART1_RX_DMA
// ),);
