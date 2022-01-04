//! HAL for LTDC

use core::marker::PhantomData;
use core::ops::Deref;

use crate::rcc::rec::ResetEnable;
use crate::rcc::{rec, CoreClocks};
use crate::stm32;
use crate::stm32::LTDC;
use embedded_display_controller::{
    DisplayConfiguration, DisplayController, DisplayControllerLayer,
    PixelFormat, PixelWord,
};

mod ltdc_blending_options {
    #![allow(unused)]
    pub const LTDC_BLENDING_FACTOR1_CA: u8 = 0x04; // Constant alpha
    pub const LTDC_BLENDING_FACTOR1_PA_X_CA: u8 = 0x06; // Pixel alpha * Constant alpha
    pub const LTDC_BLENDING_FACTOR2_CA: u8 = 0x05; // 1 - constant alpha
    pub const LTDC_BLENDING_FACTOR2_PA_X_CA: u8 = 0x07; // 1 - (pixel alpha * constant alpha)
}

/// LCD-TFT Display controller
pub struct Ltdc {
    ltdc: LTDC,
    config: Option<DisplayConfiguration>,
    lcd_clock: u32,
}

macro_rules! declare_layer {
    ($(($LAYER:ident, $Layer:ident, $layer:ident, $doc:expr),)+) => {
        $(
            /// Marker for ownership of
            #[doc=$doc]
            pub struct $LAYER {
                _marker: PhantomData<*const ()>,
            }
            unsafe impl Send for $LAYER {}
            impl Deref for $LAYER {
                type Target = stm32::ltdc::LAYER;
                #[inline(always)]
                fn deref(&self) -> &Self::Target {
                    unsafe { &(*LTDC::ptr()).$layer }
                }
            }
            #[doc=$doc]
            /// of the LCD-TFT display controller
            pub struct $Layer {
                layer: $LAYER,
                bytes_per_pixel: u16,

                // window parameters
                window_x0: u16,
                window_x1: u16,
                window_y0: u16,
                window_y1: u16,
            }
        )*
    }
}
declare_layer! {
    (LAYER1, LtdcLayer1, layer1, "Layer 1"),
    (LAYER2, LtdcLayer2, layer2, "Layer 2"),
}

impl Ltdc {
    /// New Ltdc. Reset and Enable LTDC block.
    ///
    /// # Panics
    ///
    /// Panics if the pixel clock LCD_CLK is running. The LCD_CLK is
    /// necessary to ensure method calls do not stall the APB bus.
    pub fn new(
        ltdc: LTDC,
        ltdc_rec: rec::Ltdc,
        core_clocks: &CoreClocks,
    ) -> Self {
        // See Errata ES0445 Rev 1 Section 2.6.1
        let lcd_clock =
            core_clocks.pll3_r_ck().expect("PLL3 R clock must run!").0;

        // Enable peripheral
        ltdc_rec.enable().reset();

        Ltdc {
            ltdc,
            config: None,
            lcd_clock,
        }
    }

    /// Enables the shadow register reload interrupt
    pub fn listen(&mut self) {
        self.ltdc.ier.modify(|_, w| w.rrie().set_bit());
    }

    /// Clear interrupt flags
    pub fn unpend() {
        // unsafe: clear write-one interrupt flag
        unsafe { (*LTDC::ptr()).icr.write(|w| w.crrif().set_bit()) };
    }

    /// Returns a reference to the inner peripheral
    pub fn inner(&self) -> &LTDC {
        &self.ltdc
    }

    /// Returns a mutable reference to the inner peripheral
    pub fn inner_mut(&mut self) -> &mut LTDC {
        &mut self.ltdc
    }
}

impl DisplayController for Ltdc {
    /// Initialize the LTDC with a given configuration
    ///
    /// # Panics
    ///
    /// Panics if the LTDC GCR register is not in a reset state. Accesses
    /// LTDC registers, so if the pclk | aclk | ker_ck are not running this
    /// function will stall the bus.
    fn init(&mut self, config: DisplayConfiguration) {
        // Check bus access
        assert!(self.ltdc.gcr.read().bits() == 0x2220); // Reset value

        // Configure the HS, VS, DE and PC polarity
        self.ltdc.gcr.modify(|_, w| {
            w.hspol()
                .bit(config.h_sync_pol)
                .vspol()
                .bit(config.v_sync_pol)
                .depol()
                .bit(config.not_data_enable_pol)
                .pcpol()
                .bit(config.pixel_clock_pol)
        });

        // Set synchronization pulse width
        self.ltdc.sscr.modify(|_, w| {
            w.vsh()
                .bits(config.v_sync - 1)
                .hsw()
                .bits(config.h_sync - 1)
        });

        // Set accumulated back porch
        self.ltdc.bpcr.modify(|_, w| {
            w.avbp()
                .bits(config.v_sync + config.v_back_porch - 1)
                .ahbp()
                .bits(config.h_sync + config.h_back_porch - 1)
        });

        // Set accumulated active width
        let aa_height =
            config.v_sync + config.v_back_porch + config.active_height - 1;
        let aa_width =
            config.h_sync + config.h_back_porch + config.active_width - 1;
        self.ltdc
            .awcr
            .modify(|_, w| w.aah().bits(aa_height).aaw().bits(aa_width));

        // Set total width and height
        let total_height: u16 = config.v_sync
            + config.v_back_porch
            + config.active_height
            + config.v_front_porch
            - 1;
        let total_width: u16 = config.h_sync
            + config.h_back_porch
            + config.active_width
            + config.h_front_porch
            - 1;
        self.ltdc.twcr.modify(|_, w| {
            w.totalh().bits(total_height).totalw().bits(total_width)
        });

        // Set the background color value
        self.ltdc.bccr.reset();

        // Enable the Transfer Error and FIFO underrun interrupts
        self.ltdc
            .ier
            .modify(|_, w| w.terrie().set_bit().fuie().set_bit());

        // Enable LTDC by setting LTDCEN bit
        self.ltdc.gcr.modify(|_, w| w.ltdcen().set_bit());

        // Save config
        self.config = Some(config);
    }

    fn clock(&self) -> u32 {
        self.lcd_clock
    }
}

impl Ltdc {
    /// Split the LTDC peripheral into two layers.
    ///
    /// Must have already called `init()`
    pub fn split(self) -> LtdcLayer1 {
        let config = self.config.as_ref().unwrap();

        let window_x0 = 0;
        let window_x1 = config.active_width;
        let window_y0 = 0;
        let window_y1 = config.active_height;

        let layer1 = LtdcLayer1 {
            layer: LAYER1 {
                _marker: PhantomData,
            },
            bytes_per_pixel: 0,
            window_x0,
            window_x1,
            window_y0,
            window_y1,
        };

        layer1
    }
}

macro_rules! impl_layer {
    ($Layer:ident) => {
        impl DisplayControllerLayer for $Layer {
            /// Configures and enables the layer
            ///
            /// # Safety
            ///
            /// `start_ptr` must point to a location that can be accessed by
            /// the LTDC peripheral, with sufficient length for the framebuffer.
            unsafe fn enable<T: PixelWord>(
                &mut self,
                start_ptr: *const T,
                pixel_format: PixelFormat,
            ) {
                use PixelFormat::*;
                let layer = &self.layer;

                {
                    // unsafe: read-only
                    let ltdc = &*LTDC::ptr();

                    // Configure the horizontal start and stop position
                    let h_win_start =
                        self.window_x0 + ltdc.bpcr.read().ahbp().bits() + 1;
                    let h_win_stop =
                        self.window_x1 + ltdc.bpcr.read().ahbp().bits();
                    layer.whpcr.modify(|_, w| {
                        w.whstpos().bits(h_win_start).whsppos().bits(h_win_stop)
                    });

                    // Configure the vertical start and stop position
                    let v_win_start =
                        self.window_y0 + ltdc.bpcr.read().avbp().bits() + 1;
                    let v_win_stop =
                        self.window_y1 + ltdc.bpcr.read().avbp().bits();
                    layer.wvpcr.modify(|_, w| {
                        w.wvstpos().bits(v_win_start).wvsppos().bits(v_win_stop)
                    });
                }

                // Set the pixel format
                layer.pfcr.modify(|_, w| w.pf().bits(pixel_format as u8));

                // Set the default color value
                layer.dccr.reset(); // Transparent black

                // Set the global constant alpha value
                let alpha = 0xFF;
                layer.cacr.modify(|_, w| w.consta().bits(alpha));

                // Set the blending factors
                let blending_factor1 =
                    ltdc_blending_options::LTDC_BLENDING_FACTOR1_PA_X_CA;
                let blending_factor2 =
                    ltdc_blending_options::LTDC_BLENDING_FACTOR2_PA_X_CA;
                layer.bfcr.modify(|_, w| {
                    w.bf1().bits(blending_factor1).bf2().bits(blending_factor2)
                });

                // Set frame buffer
                layer.cfbar.modify(|_, w| w.cfbadd().bits(start_ptr as u32));

                // Calculate framebuffer pitch in bytes
                self.bytes_per_pixel = match pixel_format {
                    ARGB8888 => 4,
                    RGB888 => 3,
                    RGB565 | ARGB4444 | ARGB1555 | AL88 => 2,
                    _ => 1,
                };

                let width = self.window_x1 - self.window_x0;
                let height = self.window_y1 - self.window_y0;

                // Framebuffer pitch and line length
                layer.cfblr.modify(|_, w| {
                    w.cfbp()
                        .bits(width * self.bytes_per_pixel)
                        .cfbll()
                        .bits(width * self.bytes_per_pixel + 7)
                });

                // Framebuffer line number
                layer.cfblnr.modify(|_, w| w.cfblnbr().bits(height));

                // Enable LTDC_Layer by setting LEN bit
                layer.cr.modify(|_, w| w.len().set_bit());

                // Reload this layer immediately
                (*LTDC::ptr()).srcr.write(|w| w.imr().set_bit());
            }

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
            ///
            /// # Panics
            ///
            /// Panics if the number of bytes per pixel multiplied by `width` is >= 8192
            unsafe fn resize_buffer_pitch(&mut self, width: u32) {
                assert!(self.bytes_per_pixel > 0);
                let pitch_bytes = (width as u16) * self.bytes_per_pixel;

                assert!(
                    pitch_bytes < 8192,
                    "Maximum buffer pitch is 8191 bytes"
                );

                // Modify CFBP
                self.layer.cfblr.modify(|_, w| w.cfbp().bits(pitch_bytes));

                // Immediate reload
                (*LTDC::ptr()).srcr.write(|w| w.imr().set_bit());
            }

            /// Swap the framebuffer to a new one.
            ///
            /// # Safety
            ///
            /// `start_ptr` must point to a location that can be accessed by
            /// the LTDC peripheral, with sufficient length for the framebuffer.
            unsafe fn swap_framebuffer<T: PixelWord>(
                &mut self,
                start_ptr: *const T,
            ) {
                // Set the new frame buffer address
                self.layer
                    .cfbar
                    .modify(|_, w| w.cfbadd().bits(start_ptr as u32));

                // Configure a shadow reload for the next blanking period
                (*LTDC::ptr()).srcr.write(|w| w.vbr().set_bit());
            }

            /// Indicates if a framebuffer swap is pending. In this situation,
            /// memory we previously supplied to
            /// [`swap_framebuffer`](#method.swap_framebuffer), before the most
            /// recent call, is still 'owned' by the display.
            fn is_swap_pending(&self) -> bool {
                // Ensure previous writes to VBR are comitted before reading it
                cortex_m::asm::dsb();

                // unsafe: Read bit
                unsafe { (*LTDC::ptr()).srcr.read().vbr().bit_is_set() }
            }
        }
    };
}
impl_layer! { LtdcLayer1 }
impl_layer! { LtdcLayer2 }
