// This demo code runs on the Electro Smith Daisy Seed board
// https://www.electro-smith.com/daisy
#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

use rtic::app;

extern crate panic_itm;

use cortex_m::asm::nop;

use cortex_m::asm::delay as delay_cycles;

use stm32h7xx_hal::device;
pub use stm32h7xx_hal::hal::digital::v2::OutputPin;
use stm32h7xx_hal::prelude::*;
use stm32h7xx_hal::sai::{
    self, I2SChanConfig, I2SDataSize, I2SDir, I2SSync, Sai, SaiChannel,
    SaiI2sExt, I2S,
};
use stm32h7xx_hal::stm32;
use stm32h7xx_hal::stm32::rcc::d2ccip1r::SAI1SEL_A;
use stm32h7xx_hal::time::Hertz;

use stm32h7xx_hal::traits::i2s::FullDuplex;

pub const AUDIO_SAMPLE_HZ: Hertz = Hertz(48_000);
// Using PLL3_P for SAI1 clock
// The rate should be equal to sample rate * 256
// But not less than so targetting 257
const PLL3_P_HZ: Hertz = Hertz(AUDIO_SAMPLE_HZ.0 * 257);

#[app( device = stm32h7xx_hal::stm32, peripherals = true )]
const APP: () = {
    struct Resources {
        audio: Sai<stm32::SAI1, I2S>,
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
            .use_hse(16u32.MHz())
            .sys_ck(400u32.MHz())
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

        // Use PLL3_P for the SAI1 clock
        let sai1_rec = ccdr.peripheral.SAI1.kernel_clk_mux(SAI1SEL_A::PLL3_P);
        let master_config =
            I2SChanConfig::new(I2SDir::Tx).set_frame_sync_active_high(true);
        let slave_config = I2SChanConfig::new(I2SDir::Rx)
            .set_sync_type(I2SSync::Internal)
            .set_frame_sync_active_high(true);

        let mut audio = ctx.device.SAI1.i2s_ch_a(
            sai1_pins,
            AUDIO_SAMPLE_HZ,
            I2SDataSize::BITS_24,
            sai1_rec,
            &ccdr.clocks,
            master_config,
            Some(slave_config),
        );

        // Setup cache
        // Sound breaks up without this enabled
        let mut core = device::CorePeripherals::take().unwrap();
        core.SCB.invalidate_icache();
        core.SCB.enable_icache();

        audio.listen(SaiChannel::ChannelB, sai::Event::Data);
        audio.enable();
        // Jump start audio transmission
        audio.try_send(0, 0).unwrap();

        init::LateResources { audio }
    }

    #[task( binds = SAI1, resources =  [audio] )]
    fn passthru(ctx: passthru::Context) {
        if let Ok((left, right)) = ctx.resources.audio.try_read() {
            ctx.resources.audio.try_send(left, right).unwrap();
        }
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        // Don't go to sleep
        loop {
            nop();
        }
    }
};
