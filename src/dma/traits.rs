//! Traits for DMA types
//!
//! Adapted from
//! https://github.com/stm32-rs/stm32f4xx-hal/blob/master/src/dma/traits.rs

use super::*;

pub(crate) mod sealed {
    /// Converts value to bits for setting a register value.
    pub trait Bits<T> {
        /// Returns the bit value.
        fn bits(self) -> T;
    }
    pub trait Sealed {}
}
use sealed::Sealed;

/// Trait for DMA streams types.
pub trait Stream: Sealed {
    /// Number of the register stream.
    const NUMBER: usize;

    /// Clear all interrupts for the DMA stream.
    fn clear_interrupts(&mut self);

    /// Clear transfer complete interrupt (tcif) for the DMA stream.
    fn clear_transfer_complete_interrupt(&mut self);

    /// Clear half transfer interrupt (htif) for the DMA stream.
    fn clear_half_transfer_interrupt(&mut self);

    /// Clear transfer error interrupt (teif) for the DMA stream.
    fn clear_transfer_error_interrupt(&mut self);

    /// Clear direct mode error interrupt (dmeif) for the DMA stream.
    fn clear_direct_mode_error_interrupt(&mut self);

    /// Clear fifo error interrupt (feif) for the DMA stream.
    fn clear_fifo_error_interrupt(&mut self);

    /// Get transfer complete flag.
    fn get_transfer_complete_flag() -> bool;

    /// Get half transfer flag.
    fn get_half_transfer_flag() -> bool;

    /// Set the peripheral address (par) for the DMA stream.
    unsafe fn set_peripheral_address(&mut self, value: u32);

    /// Set the memory address (m0ar) for the DMA stream.
    unsafe fn set_memory_address(&mut self, value: u32);

    /// Get the memory address (m0ar) for the DMA stream.
    fn get_memory_address(&self) -> u32;

    /// Set the double buffer address (m1ar) for the DMA stream.
    unsafe fn set_memory_double_buffer_address(&mut self, value: u32);

    /// Get the double buffer address (m1ar) for the DMA stream.
    fn get_memory_double_buffer_address(&self) -> u32;

    /// Set the number of transfers (ndt) for the DMA stream.
    fn set_number_of_transfers(&mut self, value: u16);

    /// Get the number of transfers (ndt) for the DMA stream.
    fn get_number_of_transfers() -> u16;

    /// Enable the DMA stream.
    ///
    /// # Safety
    ///
    /// The user must ensure that all registers are properly configured.
    unsafe fn enable(&mut self);

    /// Returns the state of the DMA stream.
    fn is_enabled() -> bool;

    /// Disable the DMA stream.
    ///
    /// Disabling the stream during an on-going transfer needs to be performed
    /// in a certain way to prevent problems if the stream is to be re-enabled
    /// shortly after, because of that, this method will also clear all the
    /// stream's interrupt flags if the stream is active.
    fn disable(&mut self);

    /// Sets the corresponding DMAMUX request line for this stream
    fn set_request_line(&mut self, request_line: u8);

    /// Set the priority (pl) the DMA stream.
    fn set_priority(&mut self, priority: config::Priority);

    /// Set the memory size (msize) for the DMA stream.
    ///
    /// # Safety
    ///
    /// This must have the same alignment of the buffer used in the transfer.
    ///
    /// Valid values:
    ///     * 0 -> byte
    ///     * 1 -> half word
    ///     * 2 -> word
    unsafe fn set_memory_size(&mut self, size: u8);

    /// Set the peripheral memory size (psize) for the DMA stream.
    ///
    /// # Safety
    ///
    /// This must have the same alignment of the peripheral data used in the
    /// transfer.
    ///
    /// Valid values:
    ///     * 0 -> byte
    ///     * 1 -> half word
    ///     * 2 -> word
    unsafe fn set_peripheral_size(&mut self, size: u8);

    /// Enable/disable memory increment (minc) for the DMA stream.
    fn set_memory_increment(&mut self, increment: bool);

    /// Enable/disable peripheral increment (pinc) for the DMA stream.
    fn set_peripheral_increment(&mut self, increment: bool);

    /// Set the direction (dir) of the DMA stream.
    fn set_direction(&mut self, direction: DmaDirection);

    #[cfg(not(feature = "rm0455"))]
    /// Enable bufferable transfers
    fn set_trbuff(&mut self, trbuff: bool);

    /// Convenience method to configure the 4 common interrupts for the DMA stream.
    fn set_interrupts_enable(
        &mut self,
        transfer_complete: bool,
        half_transfer: bool,
        transfer_error: bool,
        direct_mode_error: bool,
    );

    /// Convenience method to get the value of the 4 common interrupts for the
    /// DMA stream.
    ///
    /// The order of the returns are: `transfer_complete`, `half_transfer`,
    /// `transfer_error` and `direct_mode_error`.
    fn get_interrupts_enable() -> (bool, bool, bool, bool);

    /// Enable/disable the transfer complete interrupt (tcie) of the DMA stream.
    fn set_transfer_complete_interrupt_enable(
        &mut self,
        transfer_complete_interrupt: bool,
    );

    /// Enable/disable the half transfer interrupt (htie) of the DMA stream.
    fn set_half_transfer_interrupt_enable(
        &mut self,
        half_transfer_interrupt: bool,
    );

    /// Enable/disable the transfer error interrupt (teie) of the DMA stream.
    fn set_transfer_error_interrupt_enable(
        &mut self,
        transfer_error_interrupt: bool,
    );

    /// Enable/disable the direct mode error interrupt (dmeie) of the DMA stream.
    fn set_direct_mode_error_interrupt_enable(
        &mut self,
        direct_mode_error_interrupt: bool,
    );

    /// Enable/disable the fifo error interrupt (feie) of the DMA stream.
    fn set_fifo_error_interrupt_enable(&mut self, fifo_error_interrupt: bool);

    /// Enable/disable the double buffer (dbm) of the DMA stream.
    fn set_double_buffer(&mut self, double_buffer: bool);

    /// Set the fifo threshold (fcr.fth) of the DMA stream.
    fn set_fifo_threshold(&mut self, fifo_threshold: config::FifoThreshold);

    /// Enable/disable the fifo (dmdis) of the DMA stream.
    fn set_fifo_enable(&mut self, fifo_enable: bool);

    /// Set memory burst mode (mburst) of the DMA stream.
    fn set_memory_burst(&mut self, memory_burst: config::BurstMode);

    /// Set peripheral burst mode (pburst) of the DMA stream.
    fn set_peripheral_burst(&mut self, peripheral_burst: config::BurstMode);

    /// Get the current fifo level (fs) of the DMA stream.
    fn fifo_level() -> FifoLevel;

    /// Get which buffer is currently in use by the DMA.
    fn current_buffer() -> CurrentBuffer;
}

/// DMA direction.
pub trait Direction {
    /// Creates a new instance of the type.
    fn new() -> Self;

    /// Returns the `DmaDirection` of the type.
    fn direction() -> DmaDirection;
}

/// Mark a target that the DMA can use. This is a peripheral (PeripheralToMemory
/// or MemoryToPeripheral) or a memory (MemoryToMemory)
///
/// This trait is generic over transfer direction, so a given target can
/// implement this trait multiple times for different directions.
///
/// Each implementation has an associated memory size (u32/u16/u8) and
/// optionally an associated request line in the DMA's DMAMUX.
///
/// # Safety
///
/// Both the memory size and the address must be correct for the memory region
/// and for the DMA.
pub unsafe trait TargetAddress<D: Direction> {
    /// Memory size of the target address
    type MemSize;

    /// The address to be used by the DMA stream
    fn address(&self) -> u32;

    /// An optional associated request line
    const REQUEST_LINE: Option<u8> = None;

    /// Mark that the TRBUFF bit must be set for this target
    const TRBUFF: bool = false;
}
