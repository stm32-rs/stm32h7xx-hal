//! Direct Memory Access.
//!
//! [Transfer::init](struct.Transfer.html#method.init) is only implemented for
//! valid combinations of peripheral-stream-channel-direction, providing compile
//! time checking.
//!
//! This module implements Memory To Memory, Peripheral To Memory and Memory to
//! Peripheral transfers, double buffering is supported only for Peripheral To
//! Memory and Memory to Peripheral transfers.
//!
//! Adapted from
//! https://github.com/stm32-rs/stm32f4xx-hal/blob/master/src/dma/mod.rs

use core::{
    fmt::{self, Debug, Formatter},
    marker::PhantomData,
    mem,
    ops::Not,
    ptr,
    sync::atomic::{compiler_fence, Ordering},
};
use embedded_dma::WriteBuffer;

#[macro_use]
mod macros;

#[cfg(not(feature = "rm0455"))] // Remove when fixed upstream
pub mod dma; // DMA1 and DMA2

pub mod traits;
use traits::{sealed::Bits, Direction, Stream, TargetAddress};

/// Errors.
#[derive(PartialEq)]
pub enum DMAError<T> {
    /// DMA not ready to change buffers.
    NotReady(T),
    /// The user provided a buffer that is not big enough while double buffering.
    SmallBuffer(T),
    /// Overrun during a double buffering or circular transfer.
    Overrun(T),
}

// Manually implement `Debug`, so we can have debug information even with a
// buffer `T` that doesn't implement `Debug`. `T` is always a buffer type chosen
// by the user, because of that the debug information can be helpful even
// without knowing the inner type
impl<T> Debug for DMAError<T> {
    fn fmt(&self, f: &mut Formatter) -> fmt::Result {
        match self {
            DMAError::NotReady(_) => f.debug_tuple("NotReady").finish(),
            DMAError::SmallBuffer(_) => f.debug_tuple("SmallBuffer").finish(),
            DMAError::Overrun(_) => f.debug_tuple("Overrun").finish(),
        }
    }
}

/// Possible DMA's directions.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DmaDirection {
    /// Memory to Memory transfer.
    MemoryToMemory,
    /// Peripheral to Memory transfer.
    PeripheralToMemory,
    /// Memory to Peripheral transfer.
    MemoryToPeripheral,
}

/// DMA from a peripheral to a memory location.
#[derive(Debug, Clone, Copy)]
pub struct PeripheralToMemory;

impl Bits<u8> for PeripheralToMemory {
    #[inline(always)]
    fn bits(self) -> u8 {
        0
    }
}

impl Direction for PeripheralToMemory {
    fn new() -> Self {
        PeripheralToMemory
    }
    #[inline(always)]
    fn direction() -> DmaDirection {
        DmaDirection::PeripheralToMemory
    }
}

/// DMA from one memory location to another memory location.
#[derive(Debug, Clone, Copy)]
pub struct MemoryToMemory<T> {
    _data: PhantomData<T>,
}

impl<T> Bits<u8> for MemoryToMemory<T> {
    #[inline(always)]
    fn bits(self) -> u8 {
        2
    }
}

impl<T> Direction for MemoryToMemory<T> {
    fn new() -> Self {
        Self { _data: PhantomData }
    }
    #[inline(always)]
    fn direction() -> DmaDirection {
        DmaDirection::MemoryToMemory
    }
}

/// DMA from a memory location to a peripheral.
#[derive(Debug, Clone, Copy)]
pub struct MemoryToPeripheral;

impl Bits<u8> for MemoryToPeripheral {
    #[inline(always)]
    fn bits(self) -> u8 {
        1
    }
}

impl Direction for MemoryToPeripheral {
    fn new() -> Self {
        MemoryToPeripheral
    }
    fn direction() -> DmaDirection {
        DmaDirection::MemoryToPeripheral
    }
}

unsafe impl TargetAddress<Self> for MemoryToMemory<u8> {
    fn address(&self) -> u32 {
        unimplemented!()
    }
    type MemSize = u8;
}

unsafe impl TargetAddress<Self> for MemoryToMemory<u16> {
    fn address(&self) -> u32 {
        unimplemented!()
    }
    type MemSize = u16;
}

unsafe impl TargetAddress<Self> for MemoryToMemory<u32> {
    fn address(&self) -> u32 {
        unimplemented!()
    }
    type MemSize = u32;
}

/// How full the DMA stream's fifo is.
#[derive(Debug, Clone, Copy)]
pub enum FifoLevel {
    /// 0 < fifo_level < 1/4.
    GtZeroLtQuarter,
    /// 1/4 <= fifo_level < 1/2.
    GteQuarterLtHalf,
    /// 1/2 <= fifo_level < 3/4.
    GteHalfLtThreeQuarter,
    /// 3/4 <= fifo_level < full.
    GteThreeQuarterLtFull,
    /// Fifo is empty.
    Empty,
    /// Fifo is full.
    Full,
    /// Invalid value.
    Invalid,
}

impl From<u8> for FifoLevel {
    fn from(value: u8) -> Self {
        match value {
            0 => FifoLevel::GtZeroLtQuarter,
            1 => FifoLevel::GteQuarterLtHalf,
            2 => FifoLevel::GteHalfLtThreeQuarter,
            3 => FifoLevel::GteThreeQuarterLtFull,
            4 => FifoLevel::Empty,
            5 => FifoLevel::Full,
            _ => FifoLevel::Invalid,
        }
    }
}

/// Which DMA buffer is in use.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CurrentBuffer {
    /// The first buffer (m0ar) is in use.
    FirstBuffer,
    /// The second buffer (m1ar) is in use.
    DoubleBuffer,
}

impl Not for CurrentBuffer {
    type Output = CurrentBuffer;

    fn not(self) -> Self::Output {
        if self == CurrentBuffer::FirstBuffer {
            CurrentBuffer::DoubleBuffer
        } else {
            CurrentBuffer::FirstBuffer
        }
    }
}

/// Contains types related to DMA configuration.
pub mod config {
    use super::Bits;

    /// Priority of the DMA stream, defaults to `Medium`. If two requests have
    /// the same software priority level, the stream with the lower number takes
    /// priority over the stream with the higher number. For example, Stream 2
    /// takes priority over Stream 4.
    #[derive(Debug, Clone, Copy)]
    pub enum Priority {
        /// Low priority.
        Low,
        /// Medium priority.
        Medium,
        /// High priority.
        High,
        /// Very high priority.
        VeryHigh,
    }

    impl Bits<u8> for Priority {
        fn bits(self) -> u8 {
            match self {
                Priority::Low => 0,
                Priority::Medium => 1,
                Priority::High => 2,
                Priority::VeryHigh => 3,
            }
        }
    }

    /// The level to fill the fifo to before performing the transaction.
    #[derive(Debug, Clone, Copy)]
    pub enum FifoThreshold {
        /// 1/4 full.
        QuarterFull,
        /// 1/2 full.
        HalfFull,
        /// 3/4 full.
        ThreeQuarterFull,
        /// Full.
        Full,
    }

    impl Bits<u8> for FifoThreshold {
        fn bits(self) -> u8 {
            match self {
                FifoThreshold::QuarterFull => 0,
                FifoThreshold::HalfFull => 1,
                FifoThreshold::ThreeQuarterFull => 2,
                FifoThreshold::Full => 3,
            }
        }
    }

    /// How burst transfers are done, requires fifo enabled. Check datasheet for
    /// valid combinations.
    #[derive(Debug, Clone, Copy)]
    pub enum BurstMode {
        /// Single transfer, no burst.
        NoBurst,
        /// Burst transfer of 4 beats.
        Burst4,
        /// Burst transfer of 8 beats.
        Burst8,
        /// Burst transfer of 16 beats.
        Burst16,
    }

    impl Bits<u8> for BurstMode {
        fn bits(self) -> u8 {
            match self {
                BurstMode::NoBurst => 0,
                BurstMode::Burst4 => 1,
                BurstMode::Burst8 => 2,
                BurstMode::Burst16 => 3,
            }
        }
    }

    /// Contains the complete set of configuration for a DMA stream.
    #[derive(Debug, Clone, Copy)]
    pub struct DmaConfig {
        pub(crate) priority: Priority,
        pub(crate) memory_increment: bool,
        pub(crate) peripheral_increment: bool,
        pub(crate) transfer_complete_interrupt: bool,
        pub(crate) half_transfer_interrupt: bool,
        pub(crate) transfer_error_interrupt: bool,
        pub(crate) direct_mode_error_interrupt: bool,
        pub(crate) fifo_error_interrupt: bool,
        pub(crate) double_buffer: bool,
        pub(crate) fifo_threshold: FifoThreshold,
        pub(crate) fifo_enable: bool,
        pub(crate) memory_burst: BurstMode,
        pub(crate) peripheral_burst: BurstMode,
    }

    impl Default for DmaConfig {
        fn default() -> Self {
            Self {
                priority: Priority::Medium,
                memory_increment: false,
                peripheral_increment: false,
                transfer_complete_interrupt: false,
                half_transfer_interrupt: false,
                transfer_error_interrupt: false,
                direct_mode_error_interrupt: false,
                fifo_error_interrupt: false,
                double_buffer: false,
                fifo_threshold: FifoThreshold::QuarterFull,
                fifo_enable: false,
                memory_burst: BurstMode::NoBurst,
                peripheral_burst: BurstMode::NoBurst,
            }
        }
    }

    impl DmaConfig {
        /// Set the priority.
        #[inline(always)]
        pub fn priority(mut self, priority: Priority) -> Self {
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
        pub fn peripheral_increment(
            mut self,
            peripheral_increment: bool,
        ) -> Self {
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
        pub fn fifo_error_interrupt(
            mut self,
            fifo_error_interrupt: bool,
        ) -> Self {
            self.fifo_error_interrupt = fifo_error_interrupt;
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
        pub fn fifo_threshold(mut self, fifo_threshold: FifoThreshold) -> Self {
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
        pub fn memory_burst(mut self, memory_burst: BurstMode) -> Self {
            self.memory_burst = memory_burst;
            self
        }
        /// Set the peripheral_burst.
        #[inline(always)]
        pub fn peripheral_burst(mut self, peripheral_burst: BurstMode) -> Self {
            self.peripheral_burst = peripheral_burst;
            self
        }
    }
}

/// DMA Transfer.
pub struct Transfer<STREAM, PERIPHERAL, DIR, BUF>
where
    STREAM: Stream,
    PERIPHERAL: TargetAddress<DIR>,
    DIR: Direction,
    BUF: WriteBuffer<Word = <PERIPHERAL as TargetAddress<DIR>>::MemSize>
        + 'static,
{
    stream: STREAM,
    peripheral: PERIPHERAL,
    _direction: PhantomData<DIR>,
    buf: Option<BUF>,
    double_buf: Option<BUF>,
    // Used when double buffering
    transfer_length: u16,
}

impl<STREAM, PERIPHERAL, DIR, BUF> Transfer<STREAM, PERIPHERAL, DIR, BUF>
where
    STREAM: Stream,
    DIR: Direction,
    PERIPHERAL: TargetAddress<DIR>,
    BUF: WriteBuffer<Word = <PERIPHERAL as TargetAddress<DIR>>::MemSize>
        + 'static,
{
    /// Applies all fields in DmaConfig.
    fn apply_config(&mut self, config: config::DmaConfig) {
        let msize =
            mem::size_of::<<PERIPHERAL as TargetAddress<DIR>>::MemSize>() / 2;

        self.stream.clear_interrupts();
        self.stream.set_priority(config.priority);
        // NOTE(unsafe) These values are correct because of the invariants of TargetAddress
        unsafe {
            self.stream.set_memory_size(msize as u8);
            self.stream.set_peripheral_size(msize as u8);
        }
        self.stream.set_memory_increment(config.memory_increment);
        self.stream
            .set_peripheral_increment(config.peripheral_increment);
        self.stream.set_transfer_complete_interrupt_enable(
            config.transfer_complete_interrupt,
        );
        self.stream
            .set_half_transfer_interrupt_enable(config.half_transfer_interrupt);
        self.stream.set_transfer_error_interrupt_enable(
            config.transfer_error_interrupt,
        );
        self.stream.set_direct_mode_error_interrupt_enable(
            config.direct_mode_error_interrupt,
        );
        self.stream
            .set_fifo_error_interrupt_enable(config.fifo_error_interrupt);
        self.stream.set_double_buffer(config.double_buffer);
        self.stream.set_fifo_threshold(config.fifo_threshold);
        self.stream.set_fifo_enable(config.fifo_enable);
        self.stream.set_memory_burst(config.memory_burst);
        self.stream.set_peripheral_burst(config.peripheral_burst);
    }

    /// Configures the DMA source and destination and applies supplied
    /// configuration. In a memory to memory transfer, the `double_buf` argument
    /// is the source of the data. If double buffering is enabled, the number of
    /// transfers will be the minimum length of `memory` and `double_buf`.
    ///
    /// # Panics
    ///
    /// * When the FIFO is disabled or double buffering is enabled in
    ///   `DmaConfig` while initializing a memory to memory transfer.
    /// * When double buffering is enabled but the `double_buf` argument is
    ///   `None`.
    pub fn init(
        mut stream: STREAM,
        peripheral: PERIPHERAL,
        mut memory: BUF,
        mut double_buf: Option<BUF>,
        config: config::DmaConfig,
    ) -> Self {
        stream.disable();

        // Set peripheral to memory mode
        stream.set_direction(DIR::new());

        // Enable bufferable transfers
        #[cfg(not(feature = "rm0455"))]
        if PERIPHERAL::TRBUFF {
            stream.set_trbuff(true);
        }

        // NOTE(unsafe) We now own this buffer and we won't call any &mut
        // methods on it until the end of the DMA transfer
        let (buf_ptr, buf_len) = unsafe { memory.write_buffer() };

        // Set the memory address
        //
        // # Safety
        //
        // Must be a valid memory address
        unsafe {
            stream.set_memory_address(buf_ptr as u32);
        }

        let is_mem2mem = DIR::direction() == DmaDirection::MemoryToMemory;
        if is_mem2mem {
            // Fifo must be enabled for memory to memory
            if !config.fifo_enable {
                panic!("Fifo disabled.");
            } else if config.double_buffer {
                panic!("Double buffering enabled.");
            }
        } else {
            // Set the peripheral address
            //
            // # Safety
            //
            // Must be a valid peripheral address
            unsafe {
                stream.set_peripheral_address(peripheral.address());
            }
        }

        let db_len = if let Some(ref mut db) = double_buf {
            // NOTE(unsafe) We now own this buffer and we won't call any &mut
            // methods on it until the end of the DMA transfer

            let (db_ptr, db_len) = unsafe { db.write_buffer() };
            unsafe {
                if is_mem2mem {
                    // Double buffer is the source in mem2mem mode
                    stream.set_peripheral_address(db_ptr as u32);
                } else {
                    stream.set_memory_double_buffer_address(db_ptr as u32);
                }
            }
            Some(db_len)
        } else {
            // Double buffer mode must not be enabled if we haven't been given a
            // second buffer
            if config.double_buffer {
                panic!("No second buffer.");
            }
            None
        };

        let n_transfers = if let Some(db) = db_len {
            buf_len.min(db) as u16
        } else {
            buf_len as u16
        };
        stream.set_number_of_transfers(n_transfers);

        // Set the DMAMUX request line if needed
        if let Some(request_line) = PERIPHERAL::REQUEST_LINE {
            stream.set_request_line(request_line);
        }

        let mut transfer = Self {
            stream,
            //_channel: PhantomData,
            peripheral,
            _direction: PhantomData,
            buf: Some(memory),
            double_buf,
            transfer_length: n_transfers,
        };
        transfer.apply_config(config);

        transfer
    }

    /// Starts the transfer, the closure will be executed right after enabling
    /// the stream.
    pub fn start<F>(&mut self, f: F)
    where
        F: FnOnce(&mut PERIPHERAL),
    {
        // "Preceding reads and writes cannot be moved past subsequent writes"
        compiler_fence(Ordering::Release);

        unsafe {
            self.stream.enable();
        }
        f(&mut self.peripheral);
    }

    /// Pauses the dma stream, the closure will be executed right before
    /// disabling the stream.
    pub fn pause<F>(&mut self, f: F)
    where
        F: FnOnce(&mut PERIPHERAL),
    {
        f(&mut self.peripheral);
        self.stream.disable()
    }

    /// Changes the buffer and restarts or continues a double buffer
    /// transfer. This must be called immediately after a transfer complete
    /// event. Returns the old buffer together with its `CurrentBuffer`. If an
    /// error occurs, this method will return the new buffer with the error.
    ///
    /// This method will clear the transfer complete flag on entry, it will also
    /// clear it again if an overrun occurs during its execution. Moreover, if
    /// an overrun occurs, the stream will be disabled and the transfer error
    /// flag will be set. This method can be called before the end of an ongoing
    /// transfer only if not using double buffering, in that case, the current
    /// transfer will be canceled and a new one will be started. A `NotReady`
    /// error will be returned if this method is called before the end of a
    /// transfer while double buffering.
    pub fn next_transfer(
        &mut self,
        mut new_buf: BUF,
    ) -> Result<(BUF, CurrentBuffer), DMAError<BUF>> {
        if self.double_buf.is_some()
            && DIR::direction() != DmaDirection::MemoryToMemory
        {
            if !STREAM::get_transfer_complete_flag() {
                return Err(DMAError::NotReady(new_buf));
            }
            self.stream.clear_transfer_complete_interrupt();
            // NOTE(unsafe) We now own this buffer and we won't call any &mut
            // methods on it until the end of the DMA transfer
            let (new_buf_ptr, new_buf_len) = unsafe { new_buf.write_buffer() };

            // We can't change the transfer length while double buffering
            if new_buf_len < usize::from(self.transfer_length) {
                return Err(DMAError::SmallBuffer(new_buf));
            }

            if STREAM::current_buffer() == CurrentBuffer::DoubleBuffer {
                unsafe {
                    self.stream.set_memory_address(new_buf_ptr as u32);
                }
                // Check if an overrun occurred, the buffer address won't be
                // updated in that case
                if self.stream.get_memory_address() != new_buf_ptr as u32 {
                    self.stream.clear_transfer_complete_interrupt();
                    return Err(DMAError::Overrun(new_buf));
                }

                // "Subsequent reads and writes cannot be moved ahead of
                // preceding reads"
                compiler_fence(Ordering::Acquire);

                let old_buf = self.buf.replace(new_buf);

                // We always have a buffer, so unwrap can't fail
                return Ok((old_buf.unwrap(), CurrentBuffer::FirstBuffer));
            } else {
                unsafe {
                    self.stream
                        .set_memory_double_buffer_address(new_buf_ptr as u32);
                }
                // Check if an overrun occurred, the buffer address won't be
                // updated in that case
                if self.stream.get_memory_double_buffer_address()
                    != new_buf_ptr as u32
                {
                    self.stream.clear_transfer_complete_interrupt();
                    return Err(DMAError::Overrun(new_buf));
                }

                // "Subsequent reads and writes cannot be moved ahead of
                // preceding reads"
                compiler_fence(Ordering::Acquire);

                let old_buf = self.double_buf.replace(new_buf);

                // double buffering, unwrap can never fail
                return Ok((old_buf.unwrap(), CurrentBuffer::DoubleBuffer));
            }
        }
        self.stream.disable();
        self.stream.clear_transfer_complete_interrupt();

        // "No re-ordering of reads and writes across this point is allowed"
        compiler_fence(Ordering::SeqCst);

        // NOTE(unsafe) We now own this buffer and we won't call any &mut
        // methods on it until the end of the DMA transfer
        let buf_len = unsafe {
            let (buf_ptr, buf_len) = new_buf.write_buffer();

            self.stream.set_memory_address(buf_ptr as u32);
            buf_len
        };
        self.stream.set_number_of_transfers(buf_len as u16);
        let old_buf = self.buf.replace(new_buf);

        // "Preceding reads and writes cannot be moved past subsequent writes"
        compiler_fence(Ordering::Release);

        unsafe {
            self.stream.enable();
        }

        Ok((old_buf.unwrap(), CurrentBuffer::FirstBuffer))
    }

    /// Stops the stream and returns the underlying resources.
    pub fn free(mut self) -> (STREAM, PERIPHERAL, BUF, Option<BUF>) {
        self.stream.disable();
        compiler_fence(Ordering::SeqCst);
        self.stream.clear_interrupts();

        unsafe {
            let stream = ptr::read(&self.stream);
            let peripheral = ptr::read(&self.peripheral);
            let buf = ptr::read(&self.buf);
            let double_buf = ptr::read(&self.double_buf);
            mem::forget(self);
            (stream, peripheral, buf.unwrap(), double_buf)
        }
    }

    /// Clear all interrupts for the DMA stream.
    #[inline(always)]
    pub fn clear_interrupts(&mut self) {
        self.stream.clear_interrupts();
    }

    /// Clear transfer complete interrupt (tcif) for the DMA stream.
    #[inline(always)]
    pub fn clear_transfer_complete_interrupt(&mut self) {
        self.stream.clear_transfer_complete_interrupt();
    }

    /// Clear half transfer interrupt (htif) for the DMA stream.
    #[inline(always)]
    pub fn clear_half_transfer_interrupt(&mut self) {
        self.stream.clear_half_transfer_interrupt();
    }

    /// Clear transfer error interrupt (teif) for the DMA stream.
    #[inline(always)]
    pub fn clear_transfer_error_interrupt(&mut self) {
        self.stream.clear_transfer_error_interrupt();
    }

    /// Clear direct mode error interrupt (dmeif) for the DMA stream.
    #[inline(always)]
    pub fn clear_direct_mode_error_interrupt(&mut self) {
        self.stream.clear_direct_mode_error_interrupt();
    }

    /// Clear fifo error interrupt (feif) for the DMA stream.
    #[inline(always)]
    pub fn clear_fifo_error_interrupt(&mut self) {
        self.stream.clear_fifo_error_interrupt();
    }

    /// Get the underlying stream of the transfer.
    ///
    /// # Safety
    ///
    /// This implementation relies on several configurations points in order to
    /// be sound, this method can void that. The use of this method is
    /// discouraged.
    pub unsafe fn get_stream(&mut self) -> &mut STREAM {
        &mut self.stream
    }

    #[inline(always)]
    pub fn get_transfer_complete_flag(&self) -> bool {
        STREAM::get_transfer_complete_flag()
    }

    /// Changes the buffer and restarts or continues a double buffer
    /// transfer. This must be called immediately after a transfer complete
    /// event. The closure must return `(BUF, T)` where `BUF` is the new buffer
    /// to be used. This method can be called before the end of an ongoing
    /// transfer only if not using double buffering, in that case, the current
    /// transfer will be canceled and a new one will be started. A `NotReady`
    /// error will be returned if this method is called before the end of a
    /// transfer while double buffering and the closure won't be executed.
    ///
    /// # Panics
    ///
    /// This method will panic when double buffering and one or both of the
    /// following conditions happen:
    ///
    /// * The new buffer's length is smaller than the one used in the `init` method.
    /// * The closure `f` takes too long to return and a buffer overrun occurs.
    ///
    /// # Safety
    ///
    /// Memory corruption might occur in the previous buffer, the one passed to
    /// the closure, if an overrun occurs in double buffering mode.
    ///
    /// # Panics
    ///
    /// This method will panic if an overrun is detected while double buffering.
    pub unsafe fn next_transfer_with<F, T>(
        &mut self,
        f: F,
    ) -> Result<T, DMAError<()>>
    where
        F: FnOnce(BUF, CurrentBuffer) -> (BUF, T),
    {
        if self.double_buf.is_some()
            && DIR::direction() != DmaDirection::MemoryToMemory
        {
            if !STREAM::get_transfer_complete_flag() {
                return Err(DMAError::NotReady(()));
            }
            self.stream.clear_transfer_complete_interrupt();

            let current_buffer = STREAM::current_buffer();
            // double buffering, unwrap can never fail
            let db = if current_buffer == CurrentBuffer::DoubleBuffer {
                self.buf.take().unwrap()
            } else {
                self.double_buf.take().unwrap()
            };
            let r = f(db, !current_buffer);
            let mut new_buf = r.0;
            let (new_buf_ptr, new_buf_len) = new_buf.write_buffer();

            // We can't change the transfer length while double buffering
            assert!(
                new_buf_len >= usize::from(self.transfer_length),
                "Second Buffer not big enough"
            );

            // We don't know how long the closure took to complete, we might
            // have changed the current buffer twice (or any even number of
            // times) and got back to the same buffer we had in the beginning of
            // the method, check for that
            if STREAM::get_transfer_complete_flag() {
                // If this is true, then RAM corruption might have occurred,
                // there's nothing we can do apart from panicking.  TODO: Is
                // this the best solution ? The closure based approach seems
                // necessary if we want to support BBqueue.
                panic!("Overrun");
            }

            if current_buffer == CurrentBuffer::DoubleBuffer {
                self.stream.set_memory_address(new_buf_ptr as u32);

                // Check again if an overrun occurred, the buffer address won't
                // be updated in that case
                if self.stream.get_memory_address() != new_buf_ptr as u32 {
                    panic!("Overrun");
                }

                // "Subsequent reads and writes cannot be moved ahead of
                // preceding reads"
                compiler_fence(Ordering::Acquire);

                self.buf.replace(new_buf);
                return Ok(r.1);
            } else {
                self.stream
                    .set_memory_double_buffer_address(new_buf_ptr as u32);
                if self.stream.get_memory_double_buffer_address()
                    != new_buf_ptr as u32
                {
                    panic!("Overrun");
                }

                // "Subsequent reads and writes cannot be moved ahead of
                // preceding reads"
                compiler_fence(Ordering::Acquire);

                self.double_buf.replace(new_buf);
                return Ok(r.1);
            }
        }
        self.stream.disable();
        self.stream.clear_transfer_complete_interrupt();

        // "No re-ordering of reads and writes across this point is allowed"
        compiler_fence(Ordering::SeqCst);

        // Can never fail, we never let the Transfer without a buffer
        let old_buf = self.buf.take().unwrap();
        let r = f(old_buf, CurrentBuffer::FirstBuffer);
        let mut new_buf = r.0;

        let (buf_ptr, buf_len) = new_buf.write_buffer();
        self.stream.set_memory_address(buf_ptr as u32);
        self.stream.set_number_of_transfers(buf_len as u16);
        self.buf.replace(new_buf);

        // "Preceding reads and writes cannot be moved past subsequent writes"
        compiler_fence(Ordering::Release);

        self.stream.enable();

        Ok(r.1)
    }
}

impl<STREAM, PERIPHERAL, DIR, BUF> Drop
    for Transfer<STREAM, PERIPHERAL, DIR, BUF>
where
    STREAM: Stream,
    PERIPHERAL: TargetAddress<DIR>,
    DIR: Direction,
    BUF: WriteBuffer<Word = <PERIPHERAL as TargetAddress<DIR>>::MemSize>
        + 'static,
{
    fn drop(&mut self) {
        self.stream.disable();

        // "No re-ordering of reads and writes across this point is allowed"
        compiler_fence(Ordering::SeqCst);
    }
}
