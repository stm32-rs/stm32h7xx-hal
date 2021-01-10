//! Master DMA
//!
//! The Master DMA has a wide variety of configuration options for
//! packing/unpacking data and swapping endianness. It is primarily intended for
//! use with peripherals and memory regions with ports on the 64-bit AXI bus
//! matrix. Unlike DMA1/DMA2, it can access TCM memory regions via the Cortex-M7
//! AHBS port.
//!
//! ## Trigger Modes
//!
//! #### Block Trigger (default)
//!
//! In this mode, each transfer results in the transfer of up to 65536 bytes of
//! data. The MDMA checks for other higher priority every 128 bytes, although
//! the [`buffer_length`](MdmaConfig#method.buffer_length) method can be use to
//! configure a smaller interval.
//!
//! #### Buffer Trigger
//!
//! In Buffer mode, each trigger results in the transfer of up to 128 bytes of
//! data. If a larger transfer was configured, the MDMA stream must be triggered
//! again to continue the transfer. This behaviour is useful alongside hardware
//! triggers when transferring to/from peripheral FIFOs.
//!
//! In the reference manual, the words Buffer and Transfer are used
//! interchangeably, for example in the TLEN field. To avoid confusion with
//! other types of transfers, we use only the word Buffer.
//!
//! #### Repeated Block Mode
//!
//! TODO
//!
//! #### Linked List Mode
//!
//! TODO
//!
//! ## Stream Transfer Requests
//!
//! MDMA stream transfer requests can originate from either hardware or
//! software. When a stream request is asserted, it starts either a Buffer,
//! Block, Repeated Block or Linked List transfer as specified with
//! [`trigger_mode`](MdmaConfig#method.trigger_mode).
//!
//! Unlike DMA1/DMA2, it is valid to assign the same request line to multiple
//! MDMA streams. Additionally there are multiple requests lines to choose from
//! for each target peripheral. For this reason, hardware request lines are
//! specified as part of the [`MdmaConfig`](MdmaConfig) instead of being
//! inferred from the peripheral type. If no hardware request line is specified,
//! then the request line originates from software and the transfer is started
//! immediately when [`enable`](Stream0#method.enable) is called.
//!
//!

use super::{
    config,
    traits::sealed::{Bits, Sealed},
    traits::*,
    DmaDirection, MemoryToPeripheral, PeripheralToMemory,
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

/// MDMA Stream Transfer Requests
#[derive(Debug, Clone, Copy)]
pub enum MdmaTransferRequest {
    Dma1Tcif0 = 0,
    Dma1Tcif1,
    Dma1Tcif2,
    Dma1Tcif3,
    Dma1Tcif4,
    Dma1Tcif5,
    Dma1Tcif6,
    Dma1Tcif7,
    Dma2Tcif0,
    Dma2Tcif1,
    Dma2Tcif2,
    Dma2Tcif3,
    Dma2Tcif4,
    Dma2Tcif5,
    Dma2Tcif6,
    Dma2Tcif7,
    /// LTDC line interrupt
    LtdcLiIt = 16,
    /// JPEG input FIFO threshold
    JpegIftTrg,
    /// JPEG input FIFO not full
    JpegIfntTrg,
    /// JPEG output FIFO threshold
    JpegOftTrg,
    /// JPEG output FIFO not empty
    JpegOfneTrg,
    /// JPEG end of conversion
    JpegOecTrg,
    /// QUADSPI FIFO threshold
    QuadspiFtTrg,
    /// QUADSPI FIFO transfer complete
    QuadspiTcTrg,
    /// DMA2D CLUT transfer complete
    Dma2dClutTrg,
    /// DMA2D transfer complete
    Dma2dTcTrg,
    /// DMA2D transfer watermark
    Dma2dTwTrg,
    /// DSI tearing effect
    DsiTeTrg,
    /// DSI end of refresh
    DsiEorTrg,
    /// SDMMC1 internal DMA buffer end
    Sdmmc1BuffendTrg,
    /// SDMMC1 command end
    Sdmmc1CmdEndTrg,
    /// SDMMC1 end of data
    Sdmmc1DataEndTrg,
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
    /// Returns MdmaSize from the register value
    pub fn from_register(val: u8) -> Self {
        match val {
            0 => MdmaSize::Byte,
            1 => MdmaSize::HalfWord,
            2 => MdmaSize::Word,
            3 => MdmaSize::DoubleWord,
            _ => unreachable!(),
        }
    }
    /// Returns the size in bytes of the Source/Destination size
    pub fn n_bytes(&self) -> usize {
        match self {
            MdmaSize::Byte => 1,
            MdmaSize::HalfWord => 2,
            MdmaSize::Word => 4,
            MdmaSize::DoubleWord => 8,
        }
    }
}

/// MDMA increment mode
#[derive(Debug, Clone, Copy)]
pub enum MdmaIncrement {
    Fixed,
    /// Increment by one source/destination element each element
    Increment,
    /// Decrement by one source/destination element each element
    Decrement,
    /// Increment by the given offset each element. The offset must be larger
    /// or equal to the source/destination element size, otherwise the stream
    /// initialisation will panic
    IncrementWithOffset(MdmaSize),
    /// Decrement by the given offset each element. The offset must be larger
    /// or equal to the source/destination element size, otherwise the stream
    /// initialisation will panic
    DecrementWithOffset(MdmaSize),
}
impl Default for MdmaIncrement {
    fn default() -> Self {
        MdmaIncrement::Increment
    }
}

/// MDMA trigger mode
#[derive(Debug, Clone, Copy)]
pub enum MdmaTrigger {
    /// Each MDMA request triggers a buffer transfer
    Buffer = 0b00,
    /// Each MDMA request triggers a block transfer
    Block = 0b01,
    /// Each MDMA request triggers a repeated block transfer (Not Supported)
    RepeatedBlock = 0b10,
    /// Each MDMA request triggers a linked-link transfer (Not Supported)
    LinkedList = 0b11,
}
impl Default for MdmaTrigger {
    fn default() -> Self {
        MdmaTrigger::Block
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

/// Contains the complete set of configuration for a MDMA stream.
#[derive(Debug, Default, Clone, Copy)]
pub struct MdmaConfig {
    pub(crate) priority: config::Priority,
    pub(crate) destination_increment: MdmaIncrement,
    pub(crate) source_increment: MdmaIncrement,
    pub(crate) transfer_request: Option<MdmaTransferRequest>,
    pub(crate) trigger_mode: MdmaTrigger,
    pub(crate) buffer_length: Option<u8>,
    pub(crate) word_endianness_exchange: bool,
    pub(crate) half_word_endianness_exchange: bool,
    pub(crate) byte_endianness_exchange: bool,
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
    /// Set the destination increment
    #[inline(always)]
    pub fn destination_increment(
        mut self,
        destination_increment: MdmaIncrement,
    ) -> Self {
        self.destination_increment = destination_increment;
        self
    }
    /// Set the source increment
    #[inline(always)]
    pub fn source_increment(mut self, source_increment: MdmaIncrement) -> Self {
        self.source_increment = source_increment;
        self
    }
    /// Sets a hardware transfer request line. Unlike DMA1/DMA2, it is valid to
    /// use the same hardware transfer request line for multiple streams
    #[inline(always)]
    pub fn hardware_transfer_request(
        mut self,
        transfer_request: MdmaTransferRequest,
    ) -> Self {
        self.transfer_request = Some(transfer_request);
        self
    }
    /// Sets a software-triggered transfer request line. This is the default
    #[inline(always)]
    pub fn software_transfer_request(mut self) -> Self {
        self.transfer_request = None;
        self
    }
    /// Sets the trigger mode. If the trigger mode is `Buffer`, then the MDMA
    /// must be repeatedly triggered to complete a block transfer.
    #[inline(always)]
    pub fn trigger_mode(mut self, trigger: MdmaTrigger) -> Self {
        self.trigger_mode = trigger;
        self
    }
    /// Sets the length of each buffer. For the MDMA this is the length of
    /// data to be transferred without checking for requests on other channels.
    ///
    /// Normally the MDMA is configured to trigger a block transfer
    /// (TRGM=0b01). Each block consists of one of more buffers. If
    /// `trigger_mode` is set to `Buffer` instead, then each buffer must be
    /// triggered individually.
    ///
    /// The buffer length is specified in bytes, and must be a multiple of
    /// both the source and destination sizes.
    ///
    /// If the number of bytes in the block is not a multiple of the buffer
    /// length, then the final buffer will be shorter than the others.
    #[inline(always)]
    pub fn buffer_length(mut self, bytes: u8) -> Self {
        debug_assert!(
            bytes <= 128,
            "Hardware only supports buffers up to 128 bytes"
        );

        self.buffer_length = Some(bytes);
        self
    }
    /// Set word endianness exchange. Applies when the destination is
    /// 64-bit. Otherwise don't care
    #[inline(always)]
    pub fn word_endianness_exchange(mut self, exchange: bool) -> Self {
        self.word_endianness_exchange = exchange;
        self
    }
    /// Set half word endianness exchange. Applies when the destination is a
    /// 64,32-bit. Otherwise don't care
    #[inline(always)]
    pub fn half_word_endianness_exchange(mut self, exchange: bool) -> Self {
        self.half_word_endianness_exchange = exchange;
        self
    }
    /// Set byte endianness exchange. Applies when the destination is
    /// 64,32,16-bit. Otherwise don't care
    #[inline(always)]
    pub fn byte_endianness_exchange(mut self, exchange: bool) -> Self {
        self.byte_endianness_exchange = exchange;
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

                    self.set_destination_increment(config.destination_increment);
                    self
                        .set_source_increment(config.source_increment);

                    self.set_software_triggered(config.transfer_request.is_none());
                    if let Some(transfer_request) = config.transfer_request {
                        self.set_trigger_selection(transfer_request as u8);
                    }
                    self.set_trigger_mode(config.trigger_mode);

                    // Custom buffer length if specified
                    if let Some(buffer_length) = config.buffer_length {
                        // Up to 128 bytes
                        assert!(buffer_length <= 128);
                        // Length of the buffer must be less than the length
                        // of the block
                        assert!(buffer_length as u32 <= Self::get_block_bytes());
                        // Length of the buffer must be a multiple of the
                        // source size
                        assert_eq!(buffer_length as usize %
                                   Self::get_source_size().n_bytes(), 0);
                        // Length of the buffer must be a multiple of the
                        // destination size
                        assert_eq!(buffer_length as usize %
                                   Self::get_destination_size().n_bytes(), 0);

                        // Replace calculated buffer length
                        unsafe {
                            self.set_buffer_bytes(buffer_length);
                        }
                    }

                    self.set_word_endianness_exchange(
                        config.word_endianness_exchange
                    );
                    self.set_half_word_endianness_exchange(
                        config.half_word_endianness_exchange
                    );
                    self.set_byte_endianness_exchange(
                        config.byte_endianness_exchange
                    );
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
                        buffer_transfer_complete: cr.tcie().bit_is_set(),
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
                fn set_trigger_selection(&mut self, trigger: u8) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let mdma = unsafe { &*I::ptr() };
                    mdma.$channel.tbr.modify(|_, w| unsafe { w.tsel().bits(trigger) });
                }

                #[inline(always)]
                fn set_trigger_mode(&mut self, trigger_mode: MdmaTrigger) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let mdma = unsafe { &*I::ptr() };
                    mdma.$channel.tcr.modify(|_, w| unsafe {
                        w.trgm().bits(trigger_mode as u8)
                    });
                }

                #[inline(always)]
                unsafe fn set_buffer_bytes(&mut self, value: u8) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let mdma = &*I::ptr();
                    mdma.$channel.tcr.modify(|_, w| w.tlen().bits(value - 1));
                }

                #[inline(always)]
                fn get_buffer_bytes() -> u8 {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let mdma = unsafe { &*I::ptr() };
                    mdma.$channel.tcr.read().tlen().bits() + 1
                }

                #[inline(always)]
                unsafe fn set_block_bytes(&mut self, value: u32) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let mdma = &*I::ptr();
                    mdma.$channel.bndtr.modify(|_, w| w.bndt().bits(value));
                }

                #[inline(always)]
                fn get_block_bytes() -> u32 {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let mdma = unsafe { &*I::ptr() };
                    mdma.$channel.bndtr.read().bndt().bits()
                }

                fn source_destination_size_offset(
                    config: &MdmaConfig,
                    peripheral_size: MdmaSize,
                    memory_size: MdmaSize,
                    direction: DmaDirection,
                ) -> (
                    (MdmaSize, MdmaSize),
                    (MdmaSize, MdmaSize),
                ) {
                    let (source_size, destination_size) = match direction {
                        DmaDirection::PeripheralToMemory => (peripheral_size, memory_size),
                        DmaDirection::MemoryToPeripheral => (memory_size, peripheral_size),
                        DmaDirection::MemoryToMemory => (memory_size, memory_size),
                    };

                    let source_offset = match config.source_increment {
                        MdmaIncrement::IncrementWithOffset(source_offset)
                            | MdmaIncrement::DecrementWithOffset(source_offset) => {
                                assert!(source_offset >= source_size);

                                // TODO: If source/destination is AHB and DBURST
                                // =/ 000, destination address must be aligned
                                // with DINCOS size, else the result is
                                // unpredictable.

                                source_offset
                            }
                        _ => source_size,
                    };
                    let destination_offset = match config.destination_increment {
                        MdmaIncrement::IncrementWithOffset(destination_offset)
                            | MdmaIncrement::DecrementWithOffset(destination_offset) => {
                                assert!(destination_offset >= destination_size);

                                destination_offset
                            }
                        _ => destination_size,
                    };

                    (
                        (source_size, destination_size),
                        (source_offset, destination_offset),
                    )
                }

                #[inline(always)]
                unsafe fn set_source_size(&mut self, size: MdmaSize) {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let mdma = &*I::ptr();
                    mdma.$channel.tcr.modify(|_, w| w.ssize().bits(size as u8));
                }

                #[inline(always)]
                fn get_source_size() -> MdmaSize {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let mdma = unsafe { &*I::ptr() };
                    MdmaSize::from_register(mdma.$channel.tcr.read().ssize().bits())
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
                fn get_destination_size() -> MdmaSize {
                    //NOTE(unsafe) We only access the registers that belongs to the StreamX
                    let mdma = unsafe { &*I::ptr() };
                    MdmaSize::from_register(mdma.$channel.tcr.read().dsize().bits())
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
                pub fn set_word_endianness_exchange(&mut self, exchange: bool) {
                    //NOTE(unsafe) We only access the registers that belongs to
                    // the StreamX
                    let mdma = unsafe { &*I::ptr() };
                    mdma.$channel.cr.modify(|_, w| w.wex().bit(exchange));
                }
                #[inline(always)]
                pub fn set_half_word_endianness_exchange(&mut self, exchange: bool) {
                    //NOTE(unsafe) We only access the registers that belongs to
                    // the StreamX
                    let mdma = unsafe { &*I::ptr() };
                    mdma.$channel.cr.modify(|_, w| w.hex().bit(exchange));
                }
                #[inline(always)]
                pub fn set_byte_endianness_exchange(&mut self, exchange: bool) {
                    //NOTE(unsafe) We only access the registers that belongs to
                    // the StreamX
                    let mdma = unsafe { &*I::ptr() };
                    mdma.$channel.cr.modify(|_, w| w.bex().bit(exchange));
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
