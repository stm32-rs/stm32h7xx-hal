//! Traits for an LCD-TFT display controller

/// A word type for the display memory buffer
pub trait PixelWord: Copy {}
impl PixelWord for u8 {}
impl PixelWord for u16 {}
impl PixelWord for u32 {}

/// Pixel memory layouts
///
/// * `L8`: 8-bit luminance or CLUT
/// * `AL44`: 4-bit alpha + 4-bit luminance
/// * `AL88`: 8-bit alpha + 8-bit luminance
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PixelFormat {
    ARGB8888 = 0,
    RGB888,
    RGB565,
    ARGB1555,
    ARGB4444,
    L8,
    AL44,
    AL88,
}

/// Display configuration parameters
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DisplayConfiguration {
    pub active_width: u16,
    pub active_height: u16,

    pub h_back_porch: u16,
    pub h_front_porch: u16,
    pub v_back_porch: u16,
    pub v_front_porch: u16,

    pub h_sync: u16,
    pub v_sync: u16,

    /// horizontal synchronization: `false`: active low, `true`: active high
    pub h_sync_pol: bool,
    /// vertical synchronization: `false`: active low, `true`: active high
    pub v_sync_pol: bool,
    /// data enable: `false`: active low, `true`: active high
    pub not_data_enable_pol: bool,
    /// pixel_clock: `false`: active low, `true`: active high
    pub pixel_clock_pol: bool,
}

/// Accessible layers
///
/// * `L1`: layer 1
/// * `L2`: layer 2
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Layer {
    L1,
    L2,
}

/// A microcontroller peripheral that drives a LCD-TFT display
pub trait DisplayController {
    /// Initialize the controller with a given configuration
    fn init(&mut self, config: DisplayConfiguration);

    /// Returns the clock frequency (Hz) of the controller
    fn clock(&self) -> u32;
}

/// A layer of a microcontroller peripheral that drives a LCD-TFT display.
///
/// May be implemented alongside `DisplayController` if the LCD-TFT display
/// peripheral only supports one layer.
pub trait DisplayControllerLayer {
    /// Enable this display layer.
    ///
    /// # Safety
    ///
    /// [To be completed by implementation]
    unsafe fn enable<T: PixelWord>(
        &mut self,
        start_ptr: *const T,
        pixel_format: PixelFormat,
    );

    /// Swap the framebuffer to a new one.
    ///
    /// # Safety
    ///
    /// `start_ptr` must point to a location that can be accessed by the LTDC
    /// peripheral, with sufficient length for the framebuffer.
    unsafe fn swap_framebuffer<T: PixelWord>(&mut self, start_ptr: *const T);

    /// Indicates that a framebuffer swap is pending. In this situation, memory
    /// we previously supplied to
    /// [`swap_framebuffer`](#method.swap_framebuffer), before the most recent
    /// call, is still owned by the display.
    fn is_swap_pending(&self) -> bool;

    /// Resizes the framebuffer pitch. This does not change the output window
    /// size. The shadow registers are reloaded immediately.
    ///
    /// The framebuffer pitch is the increment from the start of one line of
    /// pixels to the start of the next line.
    ///
    /// # Safety
    ///
    /// The caller must ensure that enough memory is allocated for the resulting
    /// framebuffer size
    unsafe fn resize_buffer_pitch(&mut self, width: u32);
}
