use core::mem;

/// A display with swappable framebuffers
pub struct BufferedDisplay<'a, LY> {
    layer: LY,
    front_buffer: &'a mut [u32],
    back_buffer: &'a mut [u32],
    width: i32,
    height: i32,
}

/// An individual display layer, borrowing from `BufferedDisplay`
pub struct DisplayBuffer<'a, 'p> {
    /// Underlying buffer
    pub buf: &'p mut &'a mut [u32],
    pub width: i32,
    pub height: i32,
}

impl<'a, LY> BufferedDisplay<'a, LY>
where
    LY: embedded_display_controller::DisplayControllerLayer,
{
    pub fn new(
        mut layer: LY,
        front_buffer: &'a mut [u32],
        back_buffer: &'a mut [u32],
        width: usize,
        height: usize,
    ) -> Self {
        // Safety: the frame buffer has the right size
        unsafe {
            layer.enable(
                front_buffer.as_ptr() as *const u8,
                embedded_display_controller::PixelFormat::ARGB8888,
            );
        }

        BufferedDisplay {
            layer,
            front_buffer,
            back_buffer,
            width: width as i32,
            height: height as i32,
        }
    }

    /// Swaps frame buffers
    ///
    /// # Safety
    ///
    /// Does not wait for the swap to actually occur, and hence layer accesses
    /// after this may write to the wrong layer. For a safe version, use
    /// swap_layer_wait
    pub unsafe fn swap_layer(&mut self) {
        // Have been filling back buffer
        self.layer.swap_framebuffer(self.back_buffer.as_ptr());
        // Swap the back buffer to the front and visa versa
        mem::swap(&mut self.back_buffer, &mut self.front_buffer);
    }

    /// Swaps frame buffers then waits for the swap to occour on the next
    /// vertical blanking period
    pub fn swap_layer_wait(&mut self) {
        // unsafe: we wait for the swap to occour, so current
        // displayed buffer is protected
        unsafe { self.swap_layer() };
        while self.layer.is_swap_pending() {}
    }

    /// Access to layer via closure
    pub fn layer<F, T>(&mut self, func: F) -> T
    where
        F: FnOnce(&mut DisplayBuffer) -> T,
    {
        // Create a layer that lives until the end of this call
        let mut layer = DisplayBuffer {
            buf: &mut self.back_buffer,
            width: self.width,
            height: self.height,
        };
        func(&mut layer)
    }
}

use embedded_graphics::{
    geometry,
    pixelcolor::raw::{RawData, RawU24},
    pixelcolor::Rgb888,
    Pixel,
};

// Implement DrawTarget for
impl embedded_graphics::draw_target::DrawTarget for DisplayBuffer<'_, '_> {
    type Color = Rgb888;
    type Error = ();

    /// Draw a pixel
    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for pixel in pixels {
            let Pixel(point, color) = pixel;
            let raw: RawU24 = color.into();
            let rgb: u32 = 0xFF00_0000u32 | raw.into_inner();

            if point.x >= 0
                && point.y >= 0
                && point.x < self.width
                && point.y < self.height
            {
                let index = point.y * self.width + point.x;
                self.buf[index as usize] = rgb;
            } else {
                // Ignore invalid points
            }
        }

        Ok(())
    }
}
impl geometry::OriginDimensions for DisplayBuffer<'_, '_> {
    /// Return the size of the display
    fn size(&self) -> geometry::Size {
        geometry::Size::new(self.width as u32, self.height as u32)
    }
}

impl DisplayBuffer<'_, '_> {
    /// Clears the buffer
    pub fn clear(&mut self) {
        let pixels = self.width * self.height;

        for a in self.buf[..pixels as usize].iter_mut() {
            *a = 0xFF00_0000u32; // Solid black
        }
    }
}
