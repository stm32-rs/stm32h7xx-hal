// This demo code runs on the Electro Smith Daisy Seed board
// https://www.electro-smith.com/daisy
#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate panic_itm;

use rtic::app;

use cortex_m::asm::delay as delay_cycles;

pub use stm32h7xx_hal::hal::digital::v2::OutputPin;
use stm32h7xx_hal::prelude::*;
use stm32h7xx_hal::stm32::rcc::d2ccip1r::SAI1SEL_A;
use stm32h7xx_hal::time::{Hertz, U32Ext};
use stm32h7xx_hal::{sai, stm32};
use stm32h7xx_hal::sai::SaiI2sExt;

pub const AUDIO_SAMPLE_HZ: Hertz = Hertz(48_000);
// Using PLL3_P for SAI1 clock
// The rate should be equal to sample rate * 256
const PLL3_P_HZ: Hertz = Hertz(AUDIO_SAMPLE_HZ.0 * 257);

#[app( device = stm32h7xx_hal::stm32, peripherals = true )]
const APP: () = {
    struct Resources {
        audio: sai::Sai<stm32::SAI1, sai::I2S>,
    }

    #[init]
    fn init(ctx: init::Context) -> init::LateResources {
        let pwr = ctx.device.PWR.constrain();
        let vos = pwr.freeze();

        // Clocks
        let ccdr = ctx
            .device
            .RCC
            .constrain()
            .use_hse(16.mhz())
            .sys_ck(400.mhz())
            .pclk1(100.mhz()) // DMA clock
            .pll3_p_ck(PLL3_P_HZ)
            .freeze(vos, &ctx.device.SYSCFG);

        let gpiob = ctx.device.GPIOB.split(ccdr.peripheral.GPIOB);
        let gpioe = ctx.device.GPIOE.split(ccdr.peripheral.GPIOE);
        let sai1_pins = (
            gpioe.pe2.into_alternate_af6(),       // MCLK_A
            gpioe.pe5.into_alternate_af6(),       // SCK_A
            gpioe.pe4.into_alternate_af6(),       // FS_A
            gpioe.pe6.into_alternate_af6(),       // SD_A
            Some(gpioe.pe3.into_alternate_af6()), // SD_B
        );

        // Reset the codec chip
        // Hold it low for ~1ms
        let mut codec = gpiob.pb11.into_push_pull_output();
        codec.set_low().unwrap();
        delay_cycles(400_000);
        codec.set_high().unwrap();

        let sai1_rec = ccdr.peripheral.SAI1.kernel_clk_mux(SAI1SEL_A::PLL3_P);

        let mut audio = ctx.device.SAI1.i2s_ch_a(
            sai1_pins,
            AUDIO_SAMPLE_HZ,
            sai::I2SBitRate::BITS_24,
            sai1_rec,
            &ccdr.clocks,
            sai::SaiChannel::ChannelA,
            sai::I2SMode::Master,
            sai::I2SDir::Tx,
            Some(sai::SaiChannel::ChannelB),
            Some(sai::I2SMode::Slave),
            Some(sai::I2SDir::Rx),
        );
        audio.enable();
        // Jump start audio transmission
        audio.try_send(0, 0).unwrap();

        init::LateResources { audio: audio }
    }

    #[task( binds = SAI1, resources =  [audio] )]
    fn passthru(ctx: passthru::Context) {
        if let Ok((left, right)) = ctx.resources.audio.try_read() {
            ctx.resources.audio.try_send(left, right).unwrap();
        }
    }
};

