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

/// Minimal trait for DMA streams
pub trait Stream: Sealed {
    /// Number of the stream register
    const NUMBER: usize;

    /// Configuration structure for this stream.
    type Config;

    /// Structure representing interrupts
    type Interrupts: Copy + Debug;

    /// Apply the configation structure to this stream.
    fn apply_config(&mut self, config: Self::Config);

    /// Clear all interrupts for the DMA stream.
    fn clear_interrupts(&mut self);

    /// Clear half transfer interrupt (htif) for the DMA stream.
    fn clear_half_transfer_interrupt(&mut self);

    /// Clear transfer complete interrupt (tcif) for the DMA stream.
    fn clear_transfer_complete_interrupt(&mut self);

    /// Clear transfer error interrupt (teif) for the DMA stream.
    fn clear_transfer_error_interrupt(&mut self);

    /// Get half transfer flag.
    fn get_half_transfer_flag() -> bool;

    /// Get transfer complete flag.
    fn get_transfer_complete_flag() -> bool;

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

    /// Sets the request or trigger line for this stream
    fn set_request_line(&mut self, request_line: u8);

    /// Set the priority the DMA stream.
    fn set_priority(&mut self, priority: config::Priority);

    /// Disable all interrupts for the DMA stream.
    fn disable_interrupts(&mut self);

    /// Configure interrupts for the DMA stream
    fn enable_interrupts(&mut self, interrupts: Self::Interrupts);

    /// Get the value of all the interrupts for this DMA stream
    fn get_interrupts_enable() -> Self::Interrupts;

    /// Enable/disable the half transfer interrupt (htie) of the DMA stream.
    fn set_half_transfer_interrupt_enable(
        &mut self,
        transfer_complete_interrupt: bool,
    );

    /// Enable/disable the transfer complete interrupt (tcie) of the DMA stream.
    fn set_transfer_complete_interrupt_enable(
        &mut self,
        transfer_complete_interrupt: bool,
    );

    /// Enable/disable the transfer error interrupt (teie) of the DMA stream.
    fn set_transfer_error_interrupt_enable(
        &mut self,
        transfer_error_interrupt: bool,
    );
}

/// Trait for Double-Buffered DMA streams
pub trait DoubleBufferedStream: Stream + Sealed {
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

    /// Enable/disable memory increment (minc) for the DMA stream.
    fn set_memory_increment(&mut self, increment: bool);

    /// Enable/disable peripheral increment (pinc) for the DMA stream.
    fn set_peripheral_increment(&mut self, increment: bool);

    /// Set the number of transfers (ndt) for the DMA stream.
    fn set_number_of_transfers(&mut self, value: u16);

    /// Get the number of transfers (ndt) for the DMA stream.
    fn get_number_of_transfers() -> u16;

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
    ///     * 3 -> double workd
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

    /// Set the direction (dir) of the DMA stream.
    fn set_direction(&mut self, direction: DmaDirection);

    #[cfg(not(feature = "rm0455"))]
    /// Enable bufferable transfers
    fn set_trbuff(&mut self, trbuff: bool);

    /// Enable/disable circular buffering for the DMA stream.
    fn set_circular_buffer(&mut self, circular_buffer: bool);

    /// Enable/disable the double buffer (dbm) of the DMA stream.
    fn set_double_buffer(&mut self, double_buffer: bool);

    /// Get which buffer is currently in use by the DMA.
    fn current_buffer() -> CurrentBuffer;
}

/// Trait for Master DMA streams
///
/// TODO
#[allow(unused)]
pub trait MasterStream: Stream + Sealed {}

/// Trait for the configuration of Double-Buffered DMA streams
pub trait DoubleBufferedConfig {
    /// Returns if the Double Buffer is enabled in the configuration
    fn is_double_buffered(&self) -> bool;

    /// Returns if the FIFO is enabled in the configuation
    fn is_fifo_enabled(&self) -> bool;
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
