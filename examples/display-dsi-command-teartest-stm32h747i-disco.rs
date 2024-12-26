//! This example uses the embedded-graphics library to draw fast moving objects on
//! an external display. The external display is connected through the DSI link.
//! DSI Adapted command mode is used to refresh the display only when needed.
//!
//! When using the display in Portrait mode, no diagonal tearing line should be visible.
//!
//! While in Landscape mode diagonal line is visible, because display is still refreshed
//! in 480px lines from display's own graphics RAM, while writes are now 800px high.
//!
//! Run command: cargo embed --release --features="stm32h747cm7,dsi,log,ltdc,fmc,example-smps,log-rtt,rt" --example display-dsi-command-teartest-stm32h747i-disco
//!
//! Tested on a STM32H747I-DISCO development board with a ST MB1166 Display
//! (supplied together with the development kit).
//! Display Controller: OTM8009A, LCD: KJD KM-040TMP-02, Frida FRD397B2509

// #![deny(warnings)]
#![no_main]
#![no_std]

use core::{mem, slice};

#[macro_use]
mod utilities;
mod utilities_display;

use log::info;
use otm8009a::Otm8009AConfig;
use stm32h7xx_hal::dsi::{ColorCoding, DsiChannel, DsiConfig, DsiPllConfig};

extern crate cortex_m;
extern crate cortex_m_rt as rt;
use cortex_m_rt::{entry, exception};

use crate::utilities_display::display_target::BufferedDisplay;
use stm32h7xx_hal::gpio::Speed;
use stm32h7xx_hal::ltdc;
use stm32h7xx_hal::stm32::rcc::d1ccipr::FMCSEL_A;
use stm32h7xx_hal::{prelude::*, stm32};

use embedded_display_controller::DisplayController;

use embedded_graphics::pixelcolor::Rgb888;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{PrimitiveStyleBuilder, Rectangle};

use crate::utilities::mpu_config::init_mpu;
use crate::utilities_display::display_primitives::{
    colored_label, display_test,
};
use crate::utilities_display::write::write_to::WriteTo;
use core::fmt::Write;
use embedded_display_controller::DisplayConfiguration;
use otm8009a::Otm8009A;
use stm32h7xx_hal::dsi::{
    DsiCmdModeTransmissionKind, DsiHost, DsiInterrupts, DsiMode, DsiPhyTimers,
    LaneCount,
};
use stm32h7xx_hal::rcc::PllConfigStrategy;

// Remember to use correct display controller orientation, Portrait in this case
pub const WIDTH: usize = 480;
pub const HEIGHT: usize = 800;

pub const DISPLAY_CONFIGURATION: DisplayConfiguration = DisplayConfiguration {
    active_width: WIDTH as _,
    active_height: HEIGHT as _,
    h_back_porch: 34,
    h_front_porch: 34,
    v_back_porch: 15,
    v_front_porch: 16,
    h_sync: 2,
    v_sync: 1,
    h_sync_pol: false,
    v_sync_pol: false,
    not_data_enable_pol: false,
    pixel_clock_pol: false,
};

/// Configure a pin for the FMC controller
macro_rules! fmc_pins {
    ($($pin:expr),*) => {
        (
            $(
                $pin.into_push_pull_output()
                    .speed(Speed::VeryHigh)
                    .into_alternate::<12>()
                    .internal_pull_up(true)
            ),*
        )
    };
}

#[entry]
fn main() -> ! {
    utilities::logger::init();

    let dp = stm32::Peripherals::take().unwrap();
    let mut cp = stm32::CorePeripherals::take().unwrap();

    // Constrain and Freeze power
    info!("Setup PWR...");
    let pwr = dp.PWR.constrain();
    let pwrcfg = example_power!(pwr).vos0(&dp.SYSCFG).freeze();

    // Constrain and Freeze clock
    info!("Setup RCC...");
    let rcc = dp.RCC.constrain();

    // Important for DSI PLL to configure this correctly.
    // Disco board uses an oscillator while Eval - a crystal.
    let hse_freq = 25.MHz();
    let rcc = rcc.use_hse(hse_freq).bypass_hse();
    // Any clock should work in adapted command mode, the faster the better.
    let ltdc_freq = 42_000.kHz();

    let ccdr = rcc
        .sys_ck(400.MHz())
        // FMC will run at 100MHz, as this clock is further divided by 2
        .pll2_p_ck(200.MHz())
        // .pll2_q_ck(200.MHz())
        .pll2_r_ck(200.MHz())
        .pll2_strategy(PllConfigStrategy::Iterative)
        // LTDC
        .pll3_p_ck(400.MHz())
        .pll3_q_ck(400.MHz())
        .pll3_r_ck(ltdc_freq)
        .pll3_strategy(PllConfigStrategy::Iterative)
        .freeze(pwrcfg, &dp.SYSCFG);

    // Get frequency of LTDC pixel clock
    info!("pll3_r_ck: {:?}", ccdr.clocks.pll3_r_ck());
    let _pll3_r = ccdr.clocks.pll3_r_ck().expect("pll3 must run!");

    // Get the delay provider.
    let mut delay = cp.SYST.delay(ccdr.clocks);

    // Initialise system...
    cp.SCB.invalidate_icache();
    cp.SCB.enable_icache();
    //cp.SCB.enable_dcache(&mut cp.CPUID); // TODO invalidate dcache when writing to the display
    cp.DWT.enable_cycle_counter();

    // Initialise IO...
    // let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    // let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
    // let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
    let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);
    let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);
    let gpiof = dp.GPIOF.split(ccdr.peripheral.GPIOF);
    let gpiog = dp.GPIOG.split(ccdr.peripheral.GPIOG);
    let gpioh = dp.GPIOH.split(ccdr.peripheral.GPIOH);
    let gpioi = dp.GPIOI.split(ccdr.peripheral.GPIOI);
    let gpioj = dp.GPIOJ.split(ccdr.peripheral.GPIOJ);

    let _syscfg = dp.SYSCFG;
    let _exti = dp.EXTI;

    // MPU config for SDRAM write-through
    let sdram_size = 32 * 1024 * 1024;
    init_mpu(cp.MPU, &mut cp.SCB, sdram_size);

    // pin setup for SDRAM
    let sdram_pins = fmc_pins! {
        // A0-A11
        gpiof.pf0, gpiof.pf1, gpiof.pf2, gpiof.pf3,
        gpiof.pf4, gpiof.pf5, gpiof.pf12, gpiof.pf13,
        gpiof.pf14, gpiof.pf15, gpiog.pg0, gpiog.pg1,
        // BA0-BA1
        gpiog.pg4, gpiog.pg5,
        // D0-D31
        gpiod.pd14, gpiod.pd15, gpiod.pd0, gpiod.pd1,
        gpioe.pe7, gpioe.pe8, gpioe.pe9, gpioe.pe10,
        gpioe.pe11, gpioe.pe12, gpioe.pe13, gpioe.pe14,
        gpioe.pe15, gpiod.pd8, gpiod.pd9, gpiod.pd10,
        gpioh.ph8, gpioh.ph9, gpioh.ph10, gpioh.ph11,
        gpioh.ph12, gpioh.ph13, gpioh.ph14, gpioh.ph15,
        gpioi.pi0, gpioi.pi1, gpioi.pi2, gpioi.pi3,
        gpioi.pi6, gpioi.pi7, gpioi.pi9, gpioi.pi10,
        // NBL0 - NBL3
        gpioe.pe0, gpioe.pe1, gpioi.pi4, gpioi.pi5,
        gpioh.ph7,              // SDCKE1
        gpiog.pg8,              // SDCLK
        gpiog.pg15,             // SDNCAS
        gpioh.ph6,              // SDNE1 (!CS)
        gpiof.pf11,             // SDRAS
        gpioh.ph5               // SDNWE
    };

    let fmc_ccdr = ccdr.peripheral.FMC.kernel_clk_mux(FMCSEL_A::Pll2R);
    // TODO: incorrect for disco!
    let sdram_chip = stm32_fmc::devices::is42s32800g_6::Is42s32800g {};
    let mut sdram = dp.FMC.sdram(
        sdram_pins,
        sdram_chip,
        // ccdr.peripheral.FMC,
        fmc_ccdr,
        &ccdr.clocks,
    );

    let (fb1, fb2) = unsafe {
        // Initialise controller and SDRAM
        let ram_ptr: *mut u32 = sdram.init(&mut delay);
        slice::from_raw_parts_mut(ram_ptr, sdram_size / mem::size_of::<u32>())
            .fill(0);

        let fb_size = WIDTH * HEIGHT;
        let bank_size_words = 8 * 1024 * 1024 / 4;
        let fb1 = slice::from_raw_parts_mut(ram_ptr, fb_size);
        // Offset the second buffer into another SDRAM bank - this saves a bit of time if DMA2D is used
        // to clear the fb while update is ongoing.
        let fb2 =
            slice::from_raw_parts_mut(ram_ptr.offset(bank_size_words), fb_size);
        (fb1, fb2)
    };

    info!("Initialised SDRAM...");

    // Initialise LCD...
    // Display controller reset through dedicated IO
    let mut display_reset = gpiog.pg3.into_push_pull_output();
    display_reset.set_low();
    delay.delay_ms(20u32);
    display_reset.set_high();
    delay.delay_ms(10u32);

    // Display backlight enable
    let mut display_backlight_en = gpioj.pj12.into_push_pull_output();
    display_backlight_en.set_high();

    // Display controller TE (hw tear effect sync) pin as input
    // let _display_te = gpioj.pj2.into_alternate::<13>();
    let _display_te = gpioj.pj2.into_input();
    // display_te.make_interrupt_source(&mut syscfg);
    // display_te.trigger_on_edge(&mut exti, Edge::Rising);
    // display_te.enable_interrupt(&mut exti);

    let mut ltdc = ltdc::Ltdc::new(dp.LTDC, ccdr.peripheral.LTDC, &ccdr.clocks);
    ltdc.init(DISPLAY_CONFIGURATION);

    let layer = ltdc.split();
    let mut disp = BufferedDisplay::new(layer, fb1, fb2, WIDTH, HEIGHT);

    // Fin = 25MHz ->/idf = 5MHz ->*2 = 10MHz ->*ndiv = 1GHz ->/2 = 500MHz ->/odf = 500MHz (500Mbps per lane); pix clk (/8) = 62.5MHz
    let dsi_pll_config = unsafe { DsiPllConfig::manual(100, 5, 0, 4) };

    let dsi_config = DsiConfig {
        mode: DsiMode::AdaptedCommand { tear_effect: None },
        lane_count: LaneCount::DoubleLane,
        channel: DsiChannel::Ch0,
        hse_freq,
        ltdc_freq,
        interrupts: DsiInterrupts::None,
        color_coding_host: ColorCoding::TwentyFourBits,
        color_coding_wrapper: ColorCoding::TwentyFourBits,
        lp_size: 4, // for OTM8009A
        vlp_size: 4,
    };
    let mut dsi_host = DsiHost::init(
        dsi_pll_config,
        DISPLAY_CONFIGURATION,
        dsi_config,
        dp.DSIHOST,
        ccdr.peripheral.DSI,
        &ccdr.clocks,
    )
    .expect("DSI host failed to init");
    dsi_host.set_command_mode_transmission_kind(
        DsiCmdModeTransmissionKind::AllInLowPower,
    );

    // Enable DSI host
    dsi_host.start();
    dsi_host.enable_bus_turn_around(); // Must be before read attempts

    dsi_host.configure_phy_timers(DsiPhyTimers {
        dataline_hs2lp: 35,
        dataline_lp2hs: 35,
        clock_hs2lp: 35,
        clock_lp2hs: 35,
        dataline_max_read_time: 0,
        stop_wait_time: 10,
    });

    let otm8009a_config = Otm8009AConfig {
        frame_rate: otm8009a::FrameRate::_70Hz,
        // NOTE: In Landscape mode diagonal tearing line will be visible when fast changing content is present
        mode: otm8009a::Mode::Portrait,
        color_map: otm8009a::ColorMap::Rgb,
        cols: WIDTH as u16,
        rows: HEIGHT as u16,
    };
    let mut glass_ctrl = Otm8009A::new();
    glass_ctrl
        .init(&mut dsi_host, otm8009a_config, &mut delay)
        .unwrap();
    glass_ctrl.enable_te_output(533, &mut dsi_host).unwrap();

    // Not sure if this is needed
    dsi_host.set_command_mode_transmission_kind(
        DsiCmdModeTransmissionKind::AllInHighSpeed,
    );
    dsi_host.force_rx_low_power(true);

    let mut dsi_refresh_handle = dsi_host.refresh_handle();
    info!("Initialised Display...");

    let mut x = 0;
    let mut y = 0;
    let mut frame = 0;
    let style_green = PrimitiveStyleBuilder::new()
        .fill_color(Rgb888::GREEN)
        .build();
    let mut buf = [0u8; 64];

    loop {
        // Draw on a double buffered display
        disp.layer(|draw| {
            draw.clear();

            Rectangle::new(Point::new(x, 0), Size::new(100, HEIGHT as u32))
                .into_styled(style_green)
                .draw(draw)
                .unwrap();

            Rectangle::new(Point::new(0, y), Size::new(WIDTH as u32, 100))
                .into_styled(style_green)
                .draw(draw)
                .unwrap();

            x += 5;
            y += 5;
            if x >= WIDTH as i32 {
                x = 0;
            }
            if y >= HEIGHT as i32 {
                y = 0;
            }

            let mut buf = WriteTo::new(&mut buf);
            write!(&mut buf, "f: {frame}").unwrap();
            frame += 1;
            colored_label(buf.into_str().unwrap(), 50, 20, Rgb888::RED, draw)
                .unwrap();

            display_test(draw).unwrap();
        });
        disp.swap_layer_wait();
        dsi_refresh_handle.refresh_now();
        delay.delay_ms(500u32);
    }
}

#[exception]
unsafe fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    panic!("HardFault at {:#?}", ef);
}

#[exception]
unsafe fn DefaultHandler(irqn: i16) {
    panic!("Unhandled exception (IRQn = {})", irqn);
}
