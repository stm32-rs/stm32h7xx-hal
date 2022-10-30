//! This example uses the embedded-graphics library to draw text and an image on
//! an external display. The external display is connected through the LTDC
//! peripheral.
//!
//! Tested on a STM32H735G-DK development board with a ST MB1315 Display
//! (supplied together with the development kit)

#![deny(warnings)]
#![no_main]
#![no_std]

use core::{mem, slice};

#[macro_use]
mod utilities;
use log::info;

extern crate cortex_m;
extern crate cortex_m_rt as rt;
use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};

use stm32h7xx_hal::gpio::Speed;
use stm32h7xx_hal::rcc::CoreClocks;
use stm32h7xx_hal::{ltdc, xspi};
use stm32h7xx_hal::{prelude::*, stm32};

use embedded_display_controller::DisplayController;

use embedded_graphics::image::Image;
use embedded_graphics::mono_font::{ascii, MonoTextStyle};
use embedded_graphics::prelude::*;
use embedded_graphics::text::Text;

use numtoa::NumToA;
use tinybmp::Bmp;

const WIDTH: usize = 480;
const HEIGHT: usize = 272;

/// Configure SYSTICK for 1ms timebase
fn systick_init(syst: &mut stm32::SYST, clocks: CoreClocks) {
    let c_ck_mhz = clocks.c_ck().to_MHz();

    let syst_calib = 0x3E8;

    syst.set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
    syst.set_reload((syst_calib * c_ck_mhz) - 1);
    syst.enable_interrupt();
    syst.enable_counter();
}

/// TIME is an atomic u32 that counts milliseconds. Although not used
/// here, it is very useful to have for network protocols
static TIME: AtomicU32 = AtomicU32::new(0);

/// Configure a pin for the FMC controller
macro_rules! pins_alternate_high_speed {
    ($($func:ident:  $port:ident.$pin:ident:  $af:expr);*) => {
        (
            $(
                $port.$pin.into_alternate::<$af>()
                    .speed(Speed::High)
                    .internal_pull_up(true)
            ),*
        )
    };
}

// the program entry point
#[entry]
fn main() -> ! {
    utilities::logger::init();
    let dp = stm32::Peripherals::take().unwrap();
    let mut cp = stm32::CorePeripherals::take().unwrap();

    // Constrain and Freeze power
    info!("Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    let pwrcfg = example_power!(pwr).freeze();

    // Constrain and Freeze clock
    info!("Setup RCC...                  ");
    let rcc = dp.RCC.constrain();
    let ccdr = rcc
        .sys_ck(400.MHz())
        // Octo SPI
        .pll2_p_ck(400.MHz() / 5)
        .pll2_q_ck(400.MHz() / 2)
        .pll2_r_ck(400.MHz() / 2)
        // LTDC
        .pll3_p_ck(800.MHz() / 2)
        .pll3_q_ck(800.MHz() / 2)
        .pll3_r_ck(800.MHz() / 83)
        .freeze(pwrcfg, &dp.SYSCFG);

    // Get frequency of LTDC pixel clock
    let pll3_r = ccdr.clocks.pll3_r_ck().expect("pll3 must run!");

    // Get the delay provider.
    let mut delay = cp.SYST.delay(ccdr.clocks);

    // Initialise system...
    cp.SCB.invalidate_icache();
    cp.SCB.enable_icache();
    //cp.SCB.enable_dcache(&mut cp.CPUID); // TODO invalidate dcache when writing to the display
    cp.DWT.enable_cycle_counter();

    // Initialise IO...
    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
    let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);
    let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);
    let gpiof = dp.GPIOF.split(ccdr.peripheral.GPIOF);
    let gpiog = dp.GPIOG.split(ccdr.peripheral.GPIOG);
    let gpioh = dp.GPIOH.split(ccdr.peripheral.GPIOH);

    let _tracweswo = gpiob.pb3.into_alternate::<0>();

    // pin setup for HyperRAM
    let _ = pins_alternate_high_speed! {
        _ncs: gpiog.pg12: 3;
        _dqs: gpiof.pf12: 9;
        _clk: gpiof.pf4: 9;
        _io0: gpiof.pf0: 9;
        _io1: gpiof.pf1: 9;
        _io2: gpiof.pf2: 9;
        _io3: gpiof.pf3: 9;
        _io4: gpiog.pg0: 9;
        _io5: gpiog.pg1: 9;
        _io6: gpiog.pg10: 3;
        _io7: gpiog.pg11: 9
    };

    let hyperram_size = 16 * 1024 * 1024; // 16 MByte
    let config = xspi::HyperbusConfig::new(40.MHz())
        .device_size_bytes(24) // 16 Mbyte
        .refresh_interval(4.micros())
        .read_write_recovery(4) // 50ns
        .access_initial_latency(6);

    let hyperram = dp.OCTOSPI2.octospi_hyperbus_unchecked(
        config,
        &ccdr.clocks,
        ccdr.peripheral.OCTOSPI2,
    );

    let (fb1, fb2) = unsafe {
        // Initialise controller and SDRAM
        let ram_ptr: *mut u32 = hyperram.init();
        let ram_slice = slice::from_raw_parts_mut(
            ram_ptr,
            hyperram_size / mem::size_of::<u32>(),
        );

        let fb_size = WIDTH * HEIGHT;

        let mut lcd_framebuffers = ram_slice.chunks_exact_mut(fb_size);
        let fb1 = lcd_framebuffers.next().unwrap();
        let fb2 = lcd_framebuffers.next().unwrap();

        (fb1, fb2)
    };

    info!("Initialised HyperRAM...");

    // ----------------------------------------------------------
    // Initialise LCD...

    // setup LTDC  (LTDC_MspInit)
    let _ = pins_alternate_high_speed! {
        _i: gpioa.pa3: 14;
        _i: gpioa.pa4: 14;
        _i: gpioa.pa6: 14;
        _i: gpiob.pb0: 14;
        _i: gpiob.pb1: 14;
        _i: gpiob.pb8: 14;
        _i: gpiob.pb9: 14;
        _i: gpioc.pc6: 14;
        _i: gpioc.pc7: 14;
        _i: gpiod.pd0: 14;
        _i: gpiod.pd3: 14;
        _i: gpiod.pd6: 14;
        _i: gpioe.pe0: 14;
        _i: gpioe.pe1: 14;
        _i: gpioe.pe11: 14;
        _i: gpioe.pe12: 14;
        _i: gpioe.pe15: 14;
        _i: gpiog.pg7: 14;
        _i: gpiog.pg14: 14;
        _i: gpioh.ph3: 14;
        _i: gpioh.ph8: 14;
        _i: gpioh.ph9: 14;
        _i: gpioh.ph10: 14;
        _i: gpioh.ph11: 14;
        _i: gpioh.ph15: 14;
        _i: gpioa.pa8: 13;
        _i: gpioh.ph4: 9
    };

    let mut lcd_disp_en = gpioe.pe13.into_push_pull_output();
    let mut lcd_disp_ctrl = gpiod.pd10.into_push_pull_output();
    let mut lcd_bl_ctrl = gpiog.pg15.into_push_pull_output();

    delay.delay_ms(40u8);

    let mut ltdc = ltdc::Ltdc::new(dp.LTDC, ccdr.peripheral.LTDC, &ccdr.clocks);

    const RK043FN48H_HSYNC: u16 = 41; // Horizontal synchronization
    const RK043FN48H_HBP: u16 = 13; // Horizontal back porch
    const RK043FN48H_HFP: u16 = 32; // Horizontal front porch
    const RK043FN48H_VSYNC: u16 = 10; // Vertical synchronization
    const RK043FN48H_VBP: u16 = 2; // Vertical back porch
    const RK043FN48H_VFP: u16 = 2; // Vertical front porch

    ltdc.init(embedded_display_controller::DisplayConfiguration {
        active_width: WIDTH as _,
        active_height: HEIGHT as _,
        h_back_porch: RK043FN48H_HBP - 11, // -11 from MX_LTDC_Init
        h_front_porch: RK043FN48H_HFP,
        v_back_porch: RK043FN48H_VBP,
        v_front_porch: RK043FN48H_VFP,
        h_sync: RK043FN48H_HSYNC,
        v_sync: RK043FN48H_VSYNC,
        h_sync_pol: false,
        v_sync_pol: false,
        not_data_enable_pol: false,
        pixel_clock_pol: false,
    });

    let layer = ltdc.split();
    let mut disp = BufferedDisplay::new(layer, fb1, fb2);

    lcd_disp_en.set_low();
    lcd_disp_ctrl.set_high();
    lcd_bl_ctrl.set_high();

    info!("Initialised Display...");

    // ----------------------------------------------------------
    // Begin periodic tasks

    systick_init(&mut delay.free(), ccdr.clocks);
    unsafe {
        cp.SCB.shpr[15 - 4].write(128);
    } // systick exception priority

    // ----------------------------------------------------------
    // Main application loop

    loop {
        // Draw on a double buffered display
        disp.layer(|draw| {
            draw.clear();

            // Text
            let text_style =
                MonoTextStyle::new(&ascii::FONT_9X18, RgbColor::WHITE);
            Text::new("Hello Rust!", Point::new(100, 100), text_style)
                .draw(draw)
                .unwrap();

            // Text variable
            let mut buffer = [0u8; 20];
            Text::new(
                pll3_r.raw().numtoa_str(10, &mut buffer),
                Point::new(100, 140),
                text_style,
            )
            .draw(draw)
            .unwrap();

            // Image
            let ferris =
                Bmp::from_slice(include_bytes!("./ferris.bmp")).unwrap();
            let ferris = Image::new(&ferris, Point::new(106, 20));
            ferris.draw(draw).unwrap();
        });
        disp.swap_layer_wait();
    }
}

/// A display with swappable framebuffers
pub struct BufferedDisplay<'a, LY> {
    layer: LY,
    frame_buffer_seq: u8,
    frame_buffer_a: &'a mut [u32],
    frame_buffer_b: &'a mut [u32],
}

/// An individual display layer, borrowing from `BufferedDisplay`
pub struct DisplayBuffer<'a, 'p>(
    /// Underlying buffer
    pub &'p mut &'a mut [u32],
);

impl<'a, LY> BufferedDisplay<'a, LY>
where
    LY: embedded_display_controller::DisplayControllerLayer,
{
    pub fn new(
        mut layer: LY,
        frame_buffer_a: &'a mut [u32],
        frame_buffer_b: &'a mut [u32],
    ) -> Self {
        // Safety: the frame buffer has the right size
        unsafe {
            layer.enable(
                frame_buffer_a.as_ptr() as *const u8,
                embedded_display_controller::PixelFormat::ARGB8888,
            );
        }

        BufferedDisplay {
            layer,
            frame_buffer_seq: 0,
            frame_buffer_a,
            frame_buffer_b,
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
        if self.frame_buffer_seq & 1 == 0 {
            // Have been filling buffer A
            self.layer.swap_framebuffer(self.frame_buffer_a.as_ptr());
            // Fill buffer B
            self.frame_buffer_seq = 1;
        } else {
            // Have been filling buffer B
            self.layer.swap_framebuffer(self.frame_buffer_b.as_ptr());
            // Fill buffer A
            self.frame_buffer_seq = 0;
        }
    }

    /// Swaps frame buffers then waits for the swap to occour on the next
    /// vertical blanking period
    pub fn swap_layer_wait(&mut self) {
        // unsafe: we wait for the swap to occour, so current
        // displayed buffer is protected
        unsafe { self.swap_layer() };
        while self.layer.is_swap_pending() {}
    }

    /// Access to layer 1 via closure
    pub fn layer<F, T>(&mut self, func: F) -> T
    where
        F: FnOnce(&mut DisplayBuffer) -> T,
    {
        // Double buffering
        let buffer = if self.frame_buffer_seq & 1 == 0 {
            &mut self.frame_buffer_a
        } else {
            &mut self.frame_buffer_b
        };

        // Create a layer that lives until the end of this call
        let mut layer = DisplayBuffer(buffer);
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
                && point.x < (WIDTH as i32)
                && point.y < (HEIGHT as i32)
            {
                let index = (point.y * (WIDTH as i32)) + point.x;
                self.0[index as usize] = rgb;
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
        geometry::Size::new(HEIGHT as u32, WIDTH as u32)
    }
}

impl DisplayBuffer<'_, '_> {
    /// Clears the buffer
    pub fn clear(&mut self) {
        let pixels = WIDTH * HEIGHT;

        for a in self.0[..pixels as usize].iter_mut() {
            *a = 0xFF00_0000u32; // Solid black
        }
    }
}

#[exception]
fn SysTick() {
    TIME.fetch_add(1, Ordering::Relaxed);
}

#[exception]
unsafe fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    panic!("HardFault at {:#?}", ef);
}

#[exception]
unsafe fn DefaultHandler(irqn: i16) {
    panic!("Unhandled exception (IRQn = {})", irqn);
}
