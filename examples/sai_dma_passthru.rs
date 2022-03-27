// This demo code runs on the Electro Smith Daisy Seed board
// https://www.electro-smith.com/daisy
#![allow(unused_macros)]
#![deny(warnings)]
// #![deny(unsafe_code)]
#![no_main]
#![no_std]

use cortex_m::asm;

use cortex_m_rt::entry;

use hal::device;
use hal::dma;
use hal::rcc::rec::Sai1ClkSel;
use hal::sai::{self, I2sUsers, SaiChannel, SaiI2sExt};
use hal::stm32;
use hal::{pac, prelude::*};
use stm32h7xx_hal as hal;
use stm32h7xx_hal::rcc;
use stm32h7xx_hal::time::{Hertz, MegaHertz};
use stm32h7xx_hal::traits::i2s::FullDuplex;

use pac::interrupt;

use log::info;

#[macro_use]
mod utilities;

// = global constants =========================================================

// 32 samples * 2 audio channels * 2 buffers
const DMA_BUFFER_LENGTH: usize = 48 * 2 * 2;

const AUDIO_SAMPLE_HZ: Hertz = Hertz(48_000);

// Using PLL3_P for SAI1 clock
// The rate should be equal to sample rate * 256
// But not less than so targetting 257
const PLL3_P_HZ: Hertz = Hertz(AUDIO_SAMPLE_HZ.0 * 257);

// = static data ==============================================================

#[link_section = ".sram3"]
static mut TX_BUFFER: [u32; DMA_BUFFER_LENGTH] = [0; DMA_BUFFER_LENGTH];
#[link_section = ".sram3"]
static mut RX_BUFFER: [u32; DMA_BUFFER_LENGTH] = [0; DMA_BUFFER_LENGTH];
pub const CLOCK_RATE_HZ: Hertz = Hertz(400_000_000_u32);

const HSE_CLOCK_MHZ: MegaHertz = MegaHertz(16);

// PCLKx
const PCLK_HZ: Hertz = Hertz(CLOCK_RATE_HZ.0 / 4);
// PLL1
const PLL1_P_HZ: Hertz = CLOCK_RATE_HZ;

// = entry ====================================================================

#[entry]
fn main() -> ! {
    utilities::logger::init();

    // - initialize power & clocks ----------------------------------------

    let dp = hal::pac::Peripherals::take().unwrap();
    let pwr = dp.PWR.constrain();
    let vos = pwr.freeze();
    let ccdr = dp
        .RCC
        .constrain()
        .use_hse(HSE_CLOCK_MHZ)
        .sys_ck(CLOCK_RATE_HZ)
        .pclk1(PCLK_HZ) // DMA clock
        // PLL1
        .pll1_strategy(rcc::PllConfigStrategy::Iterative)
        .pll1_p_ck(PLL1_P_HZ)
        // PLL3
        .pll3_strategy(rcc::PllConfigStrategy::Iterative)
        .pll3_p_ck(PLL3_P_HZ)
        .freeze(vos, &dp.SYSCFG);

    let mut core = device::CorePeripherals::take().unwrap();
    core.SCB.enable_icache();

    // enable sai1 peripheral and set clock to pll3
    let sai1_rec = ccdr.peripheral.SAI1.kernel_clk_mux(Sai1ClkSel::PLL3_P);

    // - configure pins ---------------------------------------------------

    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
    let mut ak4556_reset = gpiob.pb11.into_push_pull_output();

    let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);
    let sai1_pins = (
        gpioe.pe2.into_alternate(),       // MCLK_A
        gpioe.pe5.into_alternate(),       // SCK_A
        gpioe.pe4.into_alternate(),       // FS_A
        gpioe.pe6.into_alternate(),       // SD_A
        Some(gpioe.pe3.into_alternate()), // SD_B
    );

    // - configure dma1 -------------------------------------------------------

    let dma1_streams =
        dma::dma::StreamsTuple::new(dp.DMA1, ccdr.peripheral.DMA1);

    // dma1 stream 0
    let tx_buffer: &'static mut [u32; DMA_BUFFER_LENGTH] =
        unsafe { &mut TX_BUFFER };
    let dma_config = dma::dma::DmaConfig::default()
        .priority(dma::config::Priority::High)
        .memory_increment(true)
        .peripheral_increment(false)
        .circular_buffer(true)
        .fifo_enable(false);
    let mut dma1_str0: dma::Transfer<_, _, dma::MemoryToPeripheral, _, _> =
        dma::Transfer::init(
            dma1_streams.0,
            unsafe { pac::Peripherals::steal().SAI1.dma_ch_b() }, // Channel B
            tx_buffer,
            None,
            dma_config,
        );

    // dma1 stream 1
    let rx_buffer: &'static mut [u32; DMA_BUFFER_LENGTH] =
        unsafe { &mut RX_BUFFER };
    let dma_config = dma_config
        .transfer_complete_interrupt(true)
        .half_transfer_interrupt(true);
    let mut dma1_str1: dma::Transfer<_, _, dma::PeripheralToMemory, _, _> =
        dma::Transfer::init(
            dma1_streams.1,
            unsafe { pac::Peripherals::steal().SAI1.dma_ch_a() }, // Channel A
            rx_buffer,
            None,
            dma_config,
        );

    // - configure sai ----------------------------------------------------

    // configure sai for FS: 48 KHz, bits: 24, Data Format: MSB Justified, LRCK Order: Hi/Lo
    let sai1_tx_config = sai::I2SChanConfig::new(sai::I2SDir::Tx)
        .set_frame_sync_active_high(true)
        .set_clock_strobe(sai::I2SClockStrobe::Falling);

    let sai1_rx_config = sai::I2SChanConfig::new(sai::I2SDir::Rx)
        .set_sync_type(sai::I2SSync::Internal)
        .set_frame_sync_active_high(true)
        .set_clock_strobe(sai::I2SClockStrobe::Rising);

    let mut sai1 = dp.SAI1.i2s_ch_a(
        sai1_pins,
        AUDIO_SAMPLE_HZ,
        sai::I2SDataSize::BITS_24,
        sai1_rec,
        &ccdr.clocks,
        I2sUsers::new(sai1_tx_config).add_slave(sai1_rx_config),
    );

    // - reset ak4556 codec -----------------------------------------------

    ak4556_reset.set_low();
    asm::delay(480_000); // ~ 1ms (datasheet specifies minimum 150ns)
    ak4556_reset.set_high();

    // - start audio ------------------------------------------------------

    // unmask interrupt handler for dma 1, stream 1
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::DMA1_STR1);
    }

    dma1_str1.start(|_sai1_rb| {
        sai1.enable_dma(SaiChannel::ChannelB);
    });

    dma1_str0.start(|sai1_rb| {
        sai1.enable_dma(SaiChannel::ChannelA);

        // wait until sai1's fifo starts to receive data
        info!("sai1 fifo waiting to receive data");
        while sai1_rb.cha.sr.read().flvl().is_empty() {}
        info!("audio started");

        sai1.enable();
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
        sai1.try_send(0, 0).unwrap();
    });

    // - dma1 stream 1 interrupt handler --------------------------------------

    type TransferDma1Str1 = dma::Transfer<
        dma::dma::Stream1<stm32::DMA1>,
        sai::dma::ChannelA<stm32::SAI1>,
        dma::PeripheralToMemory,
        &'static mut [u32; DMA_BUFFER_LENGTH],
        dma::DBTransfer,
    >;

    static mut TRANSFER_DMA1_STR1: Option<TransferDma1Str1> = None;
    unsafe {
        TRANSFER_DMA1_STR1 = Some(dma1_str1);
        info!(
            "{:?}, {:?}",
            &TX_BUFFER[0] as *const u32, &RX_BUFFER[0] as *const u32
        );
    }

    #[interrupt]
    fn DMA1_STR1() {
        let tx_buffer: &'static mut [u32; DMA_BUFFER_LENGTH] =
            unsafe { &mut TX_BUFFER };
        let rx_buffer: &'static mut [u32; DMA_BUFFER_LENGTH] =
            unsafe { &mut RX_BUFFER };

        let stereo_block_length = tx_buffer.len() / 2;

        if let Some(transfer) = unsafe { &mut TRANSFER_DMA1_STR1 } {
            let skip = if transfer.get_half_transfer_flag() {
                transfer.clear_half_transfer_interrupt();
                (0, stereo_block_length)
            } else if transfer.get_transfer_complete_flag() {
                transfer.clear_transfer_complete_interrupt();
                (stereo_block_length, 0)
            } else {
                return;
            };

            // pass thru
            let mut index = 0;
            while index < stereo_block_length {
                let tx0 = index + skip.0;
                let tx1 = tx0 + 1;
                let rx0 = index + skip.1;
                let rx1 = rx0 + 1;

                tx_buffer[tx0] = rx_buffer[rx0];
                tx_buffer[tx1] = rx_buffer[rx1];

                index += 2;
            }
        }
    }

    // - main loop ------------------------------------------------------------

    loop {
        asm::wfi();
    }
}
