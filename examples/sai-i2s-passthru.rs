// This demo code runs on the Electro Smith Daisy Seed board
// https://www.electro-smith.com/daisy
#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

#[macro_use]
mod utilities;

use stm32h7xx_hal::time::Hertz;
pub const AUDIO_SAMPLE_HZ: Hertz = Hertz(48_000);
// Using PLL3_P for SAI1 clock
// The rate should be equal to sample rate * 256
// But not less than so targetting 257
const PLL3_P_HZ: Hertz = Hertz(AUDIO_SAMPLE_HZ.0 * 257);

#[rtic::app( device = stm32h7xx_hal::stm32, peripherals = true )]
mod app {
    use cortex_m::asm::delay as delay_cycles;
    use cortex_m::asm::nop;

    use stm32h7xx_hal::hal::digital::v2::OutputPin;
    use stm32h7xx_hal::prelude::*;
    use stm32h7xx_hal::rcc::rec::Sai1ClkSel;
    use stm32h7xx_hal::sai::{
        self, I2SChanConfig, I2SDataSize, I2SDir, I2SSync, I2sUsers, Sai,
        SaiChannel, SaiI2sExt, I2S,
    };
    use stm32h7xx_hal::traits::i2s::FullDuplex;
    use stm32h7xx_hal::{stm32, time::U32Ext};

    use super::*;
    use log::info;

    #[shared]
    struct SharedResources {
        #[lock_free]
        audio: Sai<stm32::SAI1, I2S>,
    }
    #[local]
    struct LocalResources {}

    #[init]
    fn init(
        mut ctx: init::Context,
    ) -> (SharedResources, LocalResources, init::Monotonics) {
        utilities::logger::init();
        let pwr = ctx.device.PWR.constrain();
        let vos = example_power!(pwr).freeze();

        // Clocks
        let ccdr = ctx
            .device
            .RCC
            .constrain()
            .use_hse(16.mhz())
            .sys_ck(400.mhz())
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
        let sai1_rec = ccdr.peripheral.SAI1.kernel_clk_mux(Sai1ClkSel::PLL3_P);
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
            I2sUsers::new(master_config).add_slave(slave_config),
        );

        // Setup cache
        // Sound breaks up without this enabled
        ctx.core.SCB.enable_icache();

        audio.listen(SaiChannel::ChannelB, sai::Event::Data);
        audio.enable();
        // Jump start audio
        // Each of the audio blocks in the SAI are enabled by SAIEN bit in the SAI_xCR1 register.
        // As soon as this bit is active, the transmitter or the receiver is sensitive
        // to the activity on the clock line, data line and synchronization line in slave mode.
        // In master TX mode, enabling the audio block immediately generates the bit clock for the
        // external slaves even if there is no data in the FIFO, However FS signal generation
        // is conditioned by the presence of data in the FIFO.
        // After the FIFO receives the first data to transmit, this data is output to external slaves.
        // If there is no data to transmit in the FIFO, 0 values are then sent in the audio frame
        // with an underrun flag generation.
        // From the reference manual (rev7 page 2259)
        audio.try_send(0, 0).unwrap();
        info!("Startup complete!");

        (
            SharedResources { audio },
            LocalResources {},
            init::Monotonics(),
        )
    }

    #[task(binds = SAI1, shared = [audio] )]
    fn passthru(ctx: passthru::Context) {
        if let Ok((left, right)) = ctx.shared.audio.try_read() {
            ctx.shared.audio.try_send(left, right).unwrap();
        }
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        // Don't go to sleep
        loop {
            nop();
        }
    }
}
