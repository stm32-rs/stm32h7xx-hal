//! Traits for an LCD-TFT display controller

/// A word type for the display memory buffer
pub trait PixelWord {}
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

    /// Configure one layer of the display
    ///
    /// # Safety
    ///
    /// [To be completed by implementation]
    unsafe fn config_layer<T: PixelWord>(
        &mut self,
        layer: Layer,
        fb_start_ptr: *const T,
        pixel_format: PixelFormat,
    );

    /// Returns the clock frequency (Hz) of the controller
    fn clock(&self) -> u32;
}
