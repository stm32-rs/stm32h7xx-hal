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
//! Given that the Cortex-M7 core is capable of reordering accesses between
//! normal and device memory, we insert DMB instructions to ensure correct
//! operation. See ARM DAI 0321A, Section 3.2 which discusses the use of DMB
//! instructions in DMA controller configuration.
//!
//! Adapted from
//! https://github.com/stm32-rs/stm32f4xx-hal/blob/master/src/dma/mod.rs

use core::{
    cmp,
    fmt::Debug,
    marker::PhantomData,
    mem,
    ops::Not,
    ptr,
    sync::atomic::{fence, Ordering},
};
use embedded_dma::{StaticReadBuffer, StaticWriteBuffer};

#[macro_use]
mod macros;

// Note: In the future, it may make sense to restructure the DMA module.
#[allow(clippy::module_inception)]
pub mod dma; // DMA1 and DMA2

pub mod bdma;
pub mod mdma;

pub mod traits;
use traits::{
    sealed::Bits, Direction, DoubleBufferedConfig, DoubleBufferedStream,
    MasterStream, Stream, TargetAddress,
};

/// Errors.
#[derive(PartialEq, Debug, Copy, Clone)]
pub enum DMAError {
    /// DMA not ready to change buffers.
    NotReady,
    /// The user provided a buffer that is not big enough while double buffering.
    SmallBuffer,
    /// DMA started transfer on the inactive buffer while the user was processing it.
    Overflow,
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

impl Direction for MemoryToPeripheral {
    fn new() -> Self {
        MemoryToPeripheral
    }
    fn direction() -> DmaDirection {
        DmaDirection::MemoryToPeripheral
    }
}

unsafe impl TargetAddress<Self> for MemoryToMemory<u8> {
    fn address(&self) -> usize {
        unimplemented!()
    }
    type MemSize = u8;
}

unsafe impl TargetAddress<Self> for MemoryToMemory<u16> {
    fn address(&self) -> usize {
        unimplemented!()
    }
    type MemSize = u16;
}

unsafe impl TargetAddress<Self> for MemoryToMemory<u32> {
    fn address(&self) -> usize {
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
    /// The first buffer (m0ar).
    Buffer0 = 0,
    /// The second buffer (m1ar).
    Buffer1 = 1,
}

impl Not for CurrentBuffer {
    type Output = CurrentBuffer;

    fn not(self) -> Self::Output {
        if self == CurrentBuffer::Buffer0 {
            CurrentBuffer::Buffer1
        } else {
            CurrentBuffer::Buffer0
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
    impl Default for Priority {
        fn default() -> Self {
            Priority::Medium
        }
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
}

/// Marker type for a transfer with a mutable source and backed by a
/// `DoubleBufferedStream`
pub struct DBTransfer;
/// Marker type for a transfer with a constant source and backed by a
/// `DoubleBufferedStream`
pub struct ConstDBTransfer;
/// Marker type for a transfer with a mutable source and backed by a
/// `MasterStream`
pub struct MasterTransfer;
/// Marker type for a transfer with a constant source and backed by a
/// `MasterStream`
pub struct ConstMasterTransfer;

/// DMA Transfer.
pub struct Transfer<STREAM, PERIPHERAL, DIR, BUF, TXFRT>
where
    STREAM: Stream,
    PERIPHERAL: TargetAddress<DIR>,
    DIR: Direction,
{
    stream: STREAM,
    peripheral: PERIPHERAL,
    _direction: PhantomData<DIR>,
    _transfer_type: PhantomData<TXFRT>,
    buf: [Option<BUF>; 2],
    // Used when double buffering
    transfer_length: u16,
}

macro_rules! db_transfer_def {
    ($Marker:ty, $init:ident, $Buffer:tt, $rw_buffer:ident $(, $mut:tt)*;
     $($constraint:stmt)*) => {
        impl<STREAM, CONFIG, PERIPHERAL, DIR, BUF>
            Transfer<STREAM, PERIPHERAL, DIR, BUF, $Marker>
        where
            STREAM: DoubleBufferedStream + Stream<Config = CONFIG>,
            CONFIG: DoubleBufferedConfig,
            DIR: Direction,
            PERIPHERAL: TargetAddress<DIR>,
            BUF: $Buffer<Word = <PERIPHERAL as TargetAddress<DIR>>::MemSize>,
        {
            /// Applies all fields in DmaConfig.
            fn apply_config(&mut self, config: CONFIG) {
                let msize = mem::size_of::<
                    <PERIPHERAL as TargetAddress<DIR>>::MemSize,
                >() / 2;

                self.stream.clear_interrupts();

                // NOTE(unsafe) These values are correct because of the
                // invariants of TargetAddress
                unsafe {
                    self.stream.set_memory_size(msize as u8);
                    self.stream.set_peripheral_size(msize as u8);
                }

                self.stream.apply_config(config);
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
            /// * When the transfer length is greater than (2^16 - 1)
            pub fn $init(
                mut stream: STREAM,
                peripheral: PERIPHERAL,
                $($mut)* memory: BUF,
                mut double_buf: Option<BUF>,
                config: CONFIG,
            ) -> Self {
                stream.disable();

                // Used in the case that we can constant `memory`
                $($constraint)*

                // Set peripheral to memory mode
                stream.set_direction(DIR::direction());

                // Enable bufferable transfers
                #[cfg(not(feature = "rm0455"))]
                if PERIPHERAL::TRBUFF {
                    stream.set_trbuff(true);
                }

                // NOTE(unsafe) We now own this buffer and we won't call any &mut
                // methods on it until the end of the DMA transfer
                let (buf_ptr, buf_len) = unsafe { memory.$rw_buffer() };

                // Set the memory address
                //
                // # Safety
                //
                // Must be a valid memory address
                unsafe {
                    stream.set_memory_address(
                        CurrentBuffer::Buffer0,
                        buf_ptr as usize,
                    );
                }

                let is_mem2mem =
                    DIR::direction() == DmaDirection::MemoryToMemory;
                if is_mem2mem {
                    // Fifo must be enabled for memory to memory
                    if !config.is_fifo_enabled() {
                        panic!("Fifo disabled.");
                    } else if config.is_double_buffered() {
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

                    let (db_ptr, db_len) = unsafe { db.$rw_buffer() };
                    unsafe {
                        if is_mem2mem {
                            // Double buffer is the source in mem2mem mode
                            stream.set_peripheral_address(db_ptr as usize);
                        } else {
                            stream.set_memory_address(
                                CurrentBuffer::Buffer1,
                                db_ptr as usize,
                            );
                        }
                    }
                    Some(db_len)
                } else {
                    if config.is_double_buffered() {
                        // Error if we expected a double buffer but none was specified
                        panic!("No second buffer.");
                    }
                    None
                };

                let n_transfers = if let Some(db) = db_len {
                    buf_len.min(db)
                } else {
                    buf_len
                };
                assert!(
                    n_transfers <= 65535,
                    "Hardware does not support more than 65535 transfers"
                );
                let n_transfers = n_transfers as u16;
                stream.set_number_of_transfers(n_transfers);

                // Set the DMAMUX request line if needed
                if let Some(request_line) = PERIPHERAL::REQUEST_LINE {
                    stream.set_request_line(request_line);
                }

                let mut transfer = Self {
                    stream,
                    peripheral,
                    _direction: PhantomData,
                    _transfer_type: PhantomData,
                    buf: [Some(memory), double_buf],
                    transfer_length: n_transfers,
                };
                transfer.apply_config(config);

                transfer
            }

            /// Changes the buffer and restarts or continues a transfer.
            /// The closure is called with the old completed buffer as arguments and
            /// must return `(BUF, T)` where `BUF` is the new buffer
            /// to be used.
            ///
            /// In normal mode (not double buffer mode):
            /// * This method restarts the transfer.
            /// * This method can be called before the end of an ongoing transfer.
            ///   In that case, the current transfer will be canceled and a new one
            ///   will be started.
            ///
            /// In double buffer mode:
            /// * This method continues a running transfer and exchanges the inactive
            ///   buffer with the closure.
            /// * This must be called immediately after a transfer complete
            ///   event to ensure no repeated transfers into/out of the same buffer.
            /// * A `NotReady` error will be returned if this method is called
            ///   before a transfer is completed and the closure won't be executed.
            /// * A `SmallBuffer` error will be returned if the size of the buffer
            ///   returned by the closure does not match the current transfer size.
            /// * The DMA may overrun and access the poison address causing a bus error
            ///   and disabling of the stream if any of following conditions happen:
            ///   * `SmallBuffer` error
            ///   * The closure `f` takes too long to return
            /// * If the buffer address poisoning itself fails because the DMA has overrun,
            ///   the closure will still be called and the buffer address is updated but
            ///   the DMA stream will error (TEIF) and disable itself.
            ///
            /// A `remaining` parameter is also passed to the closure. This indicates
            /// the number of transfers not completed in the previous DMA transfer.
            pub fn next_transfer_with<F, T>(
                &mut self,
                func: F,
            ) -> Result<T, DMAError>
            where
                F: FnOnce(BUF, CurrentBuffer, usize) -> (BUF, T),
            {
                let (single_buffer, inactive) =
                    match STREAM::get_inactive_buffer() {
                        None => {
                            // Single buffer mode
                            self.stream.disable();
                            (true, CurrentBuffer::Buffer0)
                        }
                        Some(inactive) => {
                            // Double buffer mode
                            if !STREAM::get_transfer_complete_flag() {
                                // DMA has not released a buffer
                                return Err(DMAError::NotReady);
                            }
                            // Poison the peripheral's inactive memory address to get a memory
                            // error instead of potentially silent corruption.
                            // If DMA wins the race (overrun) to the inactive buffer
                            // between reading the CT bit and poisoning the inactive address, this
                            // write will fail and lead to a transfer error (TEIF) and disable
                            // the stream.
                            // If DMA wins the race by the time we write the new valid address
                            // (below), it gets a bus error and errors/stops.
                            unsafe {
                                self.stream.set_memory_address(
                                    inactive,
                                    0xffff_ffffusize,
                                );
                            }
                            (false, inactive)
                        }
                    };

                // Protect the instruction sequence of preceding DMA disable/inactivity
                // verification/poisoning and subsequent (old completed) buffer content
                // access.
                // Cortex-M7: Also protect the corresponding data access sequence.
                // NOTE: The data cache also needs to be flushed (if enabled).
                fence(Ordering::SeqCst);

                // Check how many data in the transfer are remaining.
                let remaining_data = STREAM::get_number_of_transfers();

                // This buffer is inactive now and can be accessed.
                // NOTE(panic): We always hold ownership in lieu of the DMA peripheral.
                let buf = self.buf[inactive as usize].take().unwrap();

                let ($($mut)* buf, result) =
                    func(buf, inactive, remaining_data as usize);

                // NOTE(unsafe) We now own this buffer and we won't access it
                // until the end of the DMA transfer.
                let (buf_ptr, buf_len) = unsafe { buf.$rw_buffer() };

                // Keep ownership of the active buffer in lieu of the DMA peripheral.
                self.buf[inactive as usize].replace(buf);

                // Protect the instruction sequence of preceding (new) buffer content access
                // and subsequent DMA enable/address update. See the matching fence() above.
                fence(Ordering::SeqCst);

                if single_buffer {
                    // Set length before the writing the new valid address.
                    self.stream.set_number_of_transfers(buf_len as u16);
                } else if buf_len != usize::from(self.transfer_length) {
                    // We can't change the transfer length while double buffering
                    return Err(DMAError::SmallBuffer);
                }

                // NOTE(double buffer mode):
                // Up to here, if the DMA starts accessing the poisoned inactive buffer (overrun)
                // this will lead to a bus error and disable DMA.
                // This write can not fail since the DMA is not accessing the corresponding buffer
                // yet or has hit the poison address and errored disabling itself.
                unsafe {
                    self.stream.set_memory_address(inactive, buf_ptr as usize);
                }

                // Acknowledge the TCIF.
                self.stream.clear_transfer_complete_interrupt();

                if single_buffer {
                    unsafe {
                        self.stream.enable();
                    }
                }

                Ok(result)
            }

            /// Changes the buffer and restarts or continues a double buffer
            /// transfer. This must be called immediately after a transfer complete
            /// event. Returns the old buffer together with its `CurrentBuffer`. If an
            /// error occurs, this method will return the old or new buffer with the error.
            ///
            /// This method will clear the transfer complete flag. Moreover, if
            /// an overrun occurs, the stream will be disabled and the transfer error
            /// flag will be set. This method can be called before the end of an ongoing
            /// transfer only if not using double buffering, in that case, the current
            /// transfer will be canceled and a new one will be started. A `NotReady`
            /// error together with the new buffer will be returned if this method is called
            /// before the end of a transfer while double buffering. A `SmallBuffer` error
            /// together with the old buffer will be returned if the new buffer size does not
            /// match the ongoing transfer size.
            pub fn next_transfer(
                &mut self,
                new_buf: BUF,
            ) -> Result<(BUF, CurrentBuffer, usize), DMAError> {
                let mut buf = new_buf;
                let mut last_remaining = 0usize;

                let current =
                    self.next_transfer_with(|mut old, current, remaining| {
                        core::mem::swap(&mut old, &mut buf);
                        last_remaining = remaining;
                        (old, current)
                    })?;
                // TODO: return buf on Err
                Ok((buf, current, last_remaining))
            }

            /// Wait for the transfer of the currently active buffer to complete,
            /// then call a function on the now inactive buffer and acknowledge the
            /// transfer complete flag.
            ///
            /// # Panics
            /// This will panic then used in single buffer mode (not DBM).
            ///
            /// # Safety
            /// Memory safety is not guaranteed.
            /// The user must ensure that the user function called on the inactive
            /// buffer completes before the running DMA transfer of the active buffer
            /// completes. If the DMA wins the race to the inactive buffer
            /// a `DMAError::Overflow` is returned but processing continues.
            ///
            /// ## Memory Fencing
            /// The user function must ensure buffer access ordering
            /// against the flag accesses. Call
            /// `core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst)`
            /// before and after accessing the buffer.
            pub unsafe fn next_dbm_transfer_with<F, T>(
                &mut self,
                func: F,
            ) -> Result<T, DMAError>
            where
                F: FnOnce(&mut BUF, CurrentBuffer) -> T,
            {
                while !STREAM::get_transfer_complete_flag() { }
                self.stream.clear_transfer_complete_flag();

                // NOTE(unwrap): Panic if stream not configured in double buffer mode.
                let inactive = STREAM::get_inactive_buffer().unwrap();

                // This buffer is inactive now and can be accessed.
                // NOTE(unwrap): We always hold ownership in lieu of the DMA peripheral.
                let buf = self.buf[inactive as usize].as_mut().unwrap();

                let result = func(buf, inactive);

                if STREAM::get_transfer_complete_flag() {
                    Err(DMAError::Overflow)
                } else {
                    Ok(result)
                }
            }

            /// Clear half transfer interrupt (htif) for the DMA stream.
            #[inline(always)]
            pub fn clear_half_transfer_interrupt(&mut self) {
                self.stream.clear_half_transfer_interrupt();
            }

            #[inline(always)]
            pub fn get_half_transfer_flag(&self) -> bool {
                STREAM::get_half_transfer_flag()
            }
        }
    };
}

db_transfer_def!(DBTransfer, init, StaticWriteBuffer, write_buffer, mut;);
db_transfer_def!(ConstDBTransfer, init_const, StaticReadBuffer, read_buffer;
                 assert!(DIR::direction() != DmaDirection::PeripheralToMemory));

impl<STREAM, CONFIG, PERIPHERAL, DIR, BUF, TXFRT>
    Transfer<STREAM, PERIPHERAL, DIR, BUF, TXFRT>
where
    STREAM: Stream<Config = CONFIG>,
    DIR: Direction,
    PERIPHERAL: TargetAddress<DIR>,
{
    /// Starts the transfer, the closure will be executed right after enabling
    /// the stream.
    pub fn start<F>(&mut self, f: F)
    where
        F: FnOnce(&mut PERIPHERAL),
    {
        // Preserve the instruction and bus ordering of preceding buffer access
        // to the subsequent access by the DMA peripheral due to enabling it.
        fence(Ordering::SeqCst);

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

    /// Stops the stream and returns the underlying resources.
    pub fn free(mut self) -> (STREAM, PERIPHERAL, BUF, Option<BUF>) {
        self.stream.disable();

        // Protect the instruction and bus sequence of the preceding disable and
        // the subsequent buffer access.
        fence(Ordering::SeqCst);

        self.stream.clear_interrupts();

        unsafe {
            let stream = ptr::read(&self.stream);
            let peripheral = ptr::read(&self.peripheral);
            let buf = ptr::read(&self.buf[0]);
            let double_buf = ptr::read(&self.buf[1]);
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

    /// Clear transfer error interrupt (teif) for the DMA stream.
    #[inline(always)]
    pub fn clear_transfer_error_interrupt(&mut self) {
        self.stream.clear_transfer_error_interrupt();
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
}

impl<STREAM, PERIPHERAL, DIR, BUF, TXFRT> Drop
    for Transfer<STREAM, PERIPHERAL, DIR, BUF, TXFRT>
where
    STREAM: Stream,
    PERIPHERAL: TargetAddress<DIR>,
    DIR: Direction,
{
    fn drop(&mut self) {
        self.stream.disable();

        // Protect the instruction and bus sequence of the preceding disable and
        // the subsequent buffer access.
        fence(Ordering::SeqCst);
    }
}

// -------- MDMA --------

impl<STREAM, CONFIG, PERIPHERAL, DIR, BUF, BUF_WORD>
    Transfer<STREAM, PERIPHERAL, DIR, BUF>
where
    STREAM: MasterStream + Stream<Config = CONFIG>,
    DIR: Direction,
    PERIPHERAL: TargetAddress<DIR>,
    BUF: StaticWriteBuffer<Word = BUF_WORD>, // Buf can be sized independently
                                             // from the peripheral
{
    /// For a given configuration, determine the size and offset for the source
    /// and destination
    ///
    /// Returns ((s_size, d_size), (s_offset, d_offset))
    fn source_destination_size_offset(
        config: &CONFIG,
    ) -> (
        (mdma::MdmaSize, mdma::MdmaSize),
        (mdma::MdmaSize, mdma::MdmaSize),
    ) {
        let peripheral_size = mdma::MdmaSize::from_type::<
            <PERIPHERAL as TargetAddress<DIR>>::MemSize,
        >();
        let memory_size = mdma::MdmaSize::from_type::<BUF_WORD>();

        STREAM::source_destination_size_offset(
            &config,
            peripheral_size,
            memory_size,
            DIR::direction(),
        )
    }

    /// Applies all fields in MdmaConfig.
    fn apply_config_master(&mut self, config: CONFIG) {
        self.stream.clear_interrupts();

        let (
            (source_size, destination_size),
            (source_offset, destination_offset),
        ) = Self::source_destination_size_offset(&config);

        // NOTE(unsafe) These values are correct for the generic types on
        // Transfer
        unsafe {
            self.stream.set_source_size(source_size);
            self.stream.set_destination_size(destination_size);
            self.stream.set_source_offset(source_offset);
            self.stream.set_destination_offset(destination_offset);
        }

        // Apply config, including offsets for this combination of configation
        // and source/destination sizes
        self.stream.apply_config(config);
    }

    /// Calculates the maximum number of bytes that can be transferred in an
    /// MDMA block. The length is reffered to the source size
    ///
    /// * `s_len`: The number of input words of s_size available. `None` if the
    /// source is a peripheral
    /// * `d_len`: The number of input words of d_size available. `None` if the
    /// destination is a peripheral
    ///
    /// `s_len` and `d_len` cannot both be peripherals (None)
    fn m_number_of_bytes(
        config: &CONFIG,
        s_len: Option<usize>,
        d_len: Option<usize>,
    ) -> usize {
        let ((s_size, d_size), (s_offset, d_offset)) =
            Self::source_destination_size_offset(&config);

        let len_to_bytes = |len, size: usize, offset: usize| {
            let bytes = len * size;
            // Include a virtual gap at the end
            let plus_gap = bytes + offset - size;
            // Bytes = Number of elements * SOURCE data size
            (plus_gap / offset) * s_size.n_bytes()
        };

        let s_bytes = s_len
            .map(|len| len_to_bytes(len, s_size.n_bytes(), s_offset.n_bytes()));
        let d_bytes = d_len
            .map(|len| len_to_bytes(len, d_size.n_bytes(), d_offset.n_bytes()));

        // Ignore None - from a memory point of view, we can read infinite bytes
        // from a peripheral
        match (s_bytes, d_bytes) {
            (Some(s), Some(d)) => cmp::min(s, d),
            (Some(s), None) => s,
            (None, Some(d)) => d,
            (None, None) => unreachable!(),
        }
    }

    /// Configures the MDMA source and destination and applies supplied
    /// configuration. In a memory to memory transfer, the `second_buf` argument
    /// is the source of the data
    ///
    /// # Panics
    ///
    /// * When a memory-memory transfer is specified but the `second_buf`
    /// argument is `None`.
    ///
    /// * When the length is greater than 65536 bytes.
    ///
    /// * When `config` specifies a `source_increment` that is smaller than the
    /// source size.
    ///
    /// * When `config` specifies a `destination_increment` that is smaller than
    /// the destination size.
    ///
    /// * When `config` specifies a `transfer_length` that is not a multiple of
    /// both the source and destination sizes.
    pub fn init_master(
        mut stream: STREAM,
        peripheral: PERIPHERAL,
        mut memory: BUF,
        mut second_buf: Option<BUF>,
        config: CONFIG,
    ) -> Self {
        stream.disable();

        // NOTE(unsafe) We now own this buffer and we won't call any &mut
        // methods on it until the end of the DMA transfer
        let (buf_ptr, buf_len) = unsafe { memory.write_buffer() };

        let (source_len, destination_len) = match DIR::direction() {
            DmaDirection::MemoryToMemory => {
                // must have 2nd buffer
                if let Some(ref mut sb) = second_buf {
                    // NOTE(unsafe) We now own this buffer and we won't call any &mut
                    // methods on it until the end of the DMA transfer

                    let (sb_ptr, sb_len) = unsafe { sb.write_buffer() };

                    // second buffer is the source in mem2mem mode
                    unsafe {
                        stream.set_source_address(sb_ptr as usize);
                        stream.set_destination_address(buf_ptr as usize);
                    }

                    (Some(sb_len), Some(buf_len))
                } else {
                    panic!("must have second buffer");
                }
            }
            DmaDirection::MemoryToPeripheral => {
                // Set the source/destination address
                //
                // # Safety
                //
                // Must be a valid source/destination address
                unsafe {
                    stream.set_source_address(buf_ptr as usize);
                    stream.set_destination_address(peripheral.address());
                }

                (Some(buf_len), None)
            }
            DmaDirection::PeripheralToMemory => {
                // Set the source/destination address
                //
                // # Safety
                //
                // Must be a valid source/destination address
                unsafe {
                    stream.set_source_address(peripheral.address());
                    stream.set_destination_address(buf_ptr as usize);
                }

                (None, Some(buf_len))
            }
        };

        // Set block length
        let block_number_of_bytes =
            Self::m_number_of_bytes(&config, source_len, destination_len);
        assert!(
            block_number_of_bytes <= 65536,
            "Hardware does not support more than 65536 bytes in a single transfer"
        );
        // Set transfer length (within the block). If block_number_of_bytes is
        // not a integer multiple of transfer_bytes, the last transfer will be
        // shorter
        let transfer_bytes = cmp::min(128, block_number_of_bytes);
        // This is overriden if set in `config`

        //NOTE(unsafe) Configuration (Number of bytes, size, offset) configured
        // to be within both source and destination buffers
        unsafe {
            stream.set_transfer_bytes(transfer_bytes as u8);
            stream.set_block_bytes(block_number_of_bytes as u32);
        }

        let mut transfer = Self {
            stream,
            peripheral,
            _direction: PhantomData,
            buf: [Some(memory), second_buf],
            transfer_length: 0, // Not used by master dma
        };
        transfer.apply_config_master(config);

        transfer
    }

    #[inline(always)]
    pub fn get_transfer_bytes(&self) -> u8 {
        STREAM::get_transfer_bytes()
    }
    #[inline(always)]
    pub fn get_block_bytes(&self) -> u32 {
        STREAM::get_block_bytes()
    }
}
