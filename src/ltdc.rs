//! HAL for LTDC

use crate::rcc::rec::ResetEnable;
use crate::rcc::{rec, CoreClocks};
use crate::stm32;
use crate::stm32::LTDC;
use crate::traits::lcd_display::{
    DisplayConfiguration, DisplayController, Layer, PixelFormat, PixelWord,
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
    pub(crate) ltdc: LTDC,
    config: Option<DisplayConfiguration>,
    lcd_clock: u32,
}

impl Ltdc {
    /// New Ltdc. Reset and Enable LTDC block.
    ///
    /// # Panics
    ///
    /// Panics if the pixel clock LCD_CLK is running. The LCD_CLK is
    /// nessesary to ensure method calls do not stall the APB bus.
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
        let ltdc = unsafe { &*stm32::LTDC::ptr() };
        ltdc.icr.write(|w| w.crrif().set_bit());
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

        // unsafe: bit ranges not defined for fields
        unsafe {
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
        }

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

    /// Configure one layer of the display
    ///
    /// For simplicity, the layer is the same size as the screen
    ///
    /// # Safety
    ///
    /// `fb_start_address` must point to a location that can be accessed by
    /// the LTDC peripheral, with sufficient length for the framebuffer.
    unsafe fn config_layer<T: PixelWord>(
        &mut self,
        layer: Layer,
        fb_start_ptr: *const T,
        pixel_format: PixelFormat,
    ) {
        use PixelFormat::*;

        let layer = match layer {
            Layer::L1 => &self.ltdc.layer1,
            Layer::L2 => &self.ltdc.layer2,
        };

        let config = self.config.as_ref().unwrap();

        let width = config.active_width;
        let height = config.active_height;
        let window_x0 = 0;
        let window_x1 = width;
        let window_y0 = 0;
        let window_y1 = height;

        // Configure the horizontal start and stop position
        let h_win_start = window_x0 + self.ltdc.bpcr.read().ahbp().bits() + 1;
        let h_win_stop = window_x1 + self.ltdc.bpcr.read().ahbp().bits();
        layer.whpcr.modify(|_, w| {
            w.whstpos().bits(h_win_start).whsppos().bits(h_win_stop)
        });

        // Configure the vertical start and stop position
        let v_win_start = window_y0 + self.ltdc.bpcr.read().avbp().bits() + 1;
        let v_win_stop = window_y1 + self.ltdc.bpcr.read().avbp().bits();
        layer.wvpcr.modify(|_, w| {
            w.wvstpos().bits(v_win_start).wvsppos().bits(v_win_stop)
        });

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
        layer
            .cfbar
            .modify(|_, w| w.cfbadd().bits(fb_start_ptr as u32));

        // Configure the color frame buffer pitch in bytes
        let bytes_per_pixel = match pixel_format {
            ARGB8888 => 4,
            RGB888 => 3,
            RGB565 | ARGB4444 | ARGB1555 | AL88 => 2,
            _ => 1,
        };

        // Configure framebuffer length
        layer.cfblr.modify(|_, w| {
            w.cfbp()
                .bits(width * bytes_per_pixel)
                .cfbll()
                .bits((window_x1 - window_x0) * bytes_per_pixel + 7)
        });

        // Configure the frame buffer line number
        layer.cfblnr.modify(|_, w| w.cfblnbr().bits(height));

        // Enable LTDC_Layer by setting LEN bit
        layer.cr.modify(|_, w| w.len().set_bit());

        // Reload this layer immediatly
        self.ltdc.srcr.write(|w| w.imr().set_bit());
    }

    fn clock(&self) -> u32 {
        self.lcd_clock
    }
}

impl Ltdc {
    /// Swap the framebuffer to a new one.
    ///
    /// # Safety
    ///
    /// `fb_start_address` must point to a location that can be accessed by
    /// the LTDC peripheral, with sufficient length for the framebuffer.
    pub unsafe fn swap_framebuffer<T: PixelWord>(
        &mut self,
        layer: Layer,
        fb_start_ptr: *const T,
    ) {
        let layer = match layer {
            Layer::L1 => &self.ltdc.layer1,
            Layer::L2 => &self.ltdc.layer2,
        };

        // Set the new frame buffer address
        layer
            .cfbar
            .modify(|_, w| w.cfbadd().bits(fb_start_ptr as u32));

        // Configure a shadow reload for the next blanking period
        self.ltdc.srcr.write(|w| w.vbr().set_bit());
    }

    /// Indicates if a framebuffer swap is pending. In this situation,
    /// memory we previously supplied to
    /// [`swap_framebuffer`](#method.swap_framebuffer), before the most
    /// recent call, is still 'owned' by the display.
    pub fn is_swap_pending(&self) -> bool {
        // Ensure previous writes to VBR are comitted before reading it
        cortex_m::asm::dsb();

        self.ltdc.srcr.read().vbr().bit_is_set()
    }
}
