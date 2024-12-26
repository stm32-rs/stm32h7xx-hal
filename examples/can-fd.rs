//! CAN-FD demo
//!
//! Demonstrates CAN-FD with Bit Rate Switching. The nominal bitrate is 500kHz,
//! but this increases to 1MHz for the data phase.
//!
//! Defaults to loopback mode. To test with other devices, enable normal mode by
//! uncommenting this line:
//!
//! ```
//! let mut can = can.into_normal();
//! ```
//!
//! Bit Rate Switching (BRS) can be disabled for all frames by changing
//! `FrameTransmissionConfig::AllowFdCanAndBRS` to
//! `FrameTransmissionConfig::AllowFdCan`.
//!
//! Tested on a MB1520-H735I-B02 development board.
//!
//! Logic Analyser trace:
//! https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/support/pulseview-can-fd-with-brs.png

#![no_main]
#![no_std]

use crate::hal::{
    gpio::{GpioExt as _, Speed},
    nb::block,
    pac,
    prelude::*,
    rcc,
    rcc::rec,
};
use fdcan::{
    config::{DataBitTiming, FrameTransmissionConfig, NominalBitTiming},
    filter::{StandardFilter, StandardFilterSlot},
    frame::{FrameFormat, TxFrameHeader},
    id::StandardId,
};
use stm32h7xx_hal as hal;

use core::num::{NonZeroU16, NonZeroU8};

use cortex_m_rt::entry;

use log::info;

#[macro_use]
mod utilities;

#[entry]
fn main() -> ! {
    utilities::logger::init();

    // Kernel Clock 32MHz, Bit rate: 500kBit/s, Sample Point 87.5%
    // Value was calculated with http://www.bittiming.can-wiki.info/
    // TODO: use the can_bit_timings crate
    let nominal_bit_timing = NominalBitTiming {
        prescaler: NonZeroU16::new(4).unwrap(),
        seg1: NonZeroU8::new(13).unwrap(),
        seg2: NonZeroU8::new(2).unwrap(),
        sync_jump_width: NonZeroU8::new(1).unwrap(),
    };

    // Kernel Clock 32MHz, Bit rate: 1MBit/s, Sample Point 87.5%
    // Value was calculated with http://www.bittiming.can-wiki.info/
    // TODO: use the can_bit_timings crate
    let data_bit_timing = DataBitTiming {
        prescaler: NonZeroU8::new(2).unwrap(),
        seg1: NonZeroU8::new(13).unwrap(),
        seg2: NonZeroU8::new(2).unwrap(),
        sync_jump_width: NonZeroU8::new(1).unwrap(),
        transceiver_delay_compensation: true,
    };

    let dp = pac::Peripherals::take().unwrap();
    let cp =
        cortex_m::Peripherals::take().expect("cannot take core peripherals");

    // Constrain and Freeze power
    info!("Setup PWR...                  ");
    let pwr = dp.PWR.constrain();
    let pwrcfg = example_power!(pwr).freeze();

    // Constrain and Freeze clock
    info!("Setup RCC...                  ");
    let rcc = dp.RCC.constrain();
    let ccdr = rcc
        .sys_ck(192.MHz())
        .pll1_strategy(rcc::PllConfigStrategy::Iterative)
        .pll1_q_ck(32.MHz())
        .freeze(pwrcfg, &dp.SYSCFG);

    // Setup fdcan_tq_ck = 32MHz
    assert_eq!(ccdr.clocks.pll1_q_ck().unwrap().raw(), 32_000_000);
    let fdcan_prec = ccdr
        .peripheral
        .FDCAN
        .kernel_clk_mux(rec::FdcanClkSel::Pll1Q);

    let mut delay = cp.SYST.delay(ccdr.clocks);

    let can1 = {
        info!("Init CAN 1");
        let gpioh = dp.GPIOH.split(ccdr.peripheral.GPIOH);
        let rx = gpioh.ph14.into_alternate().speed(Speed::VeryHigh);
        let tx = gpioh.ph13.into_alternate().speed(Speed::VeryHigh);

        info!("-- Create CAN 1 instance");
        dp.FDCAN1.fdcan(tx, rx, fdcan_prec)
    };

    // let can2 = {
    //     info!("Init CAN 2");
    //     let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
    //     let rx = gpiob.pb5.into_alternate().speed(Speed::VeryHigh);
    //     let tx = gpiob.pb6.into_alternate().speed(Speed::VeryHigh);

    //     info!("-- Create CAN 2 instance");
    //     dp.FDCAN2.fdcan(tx, rx, fdcan_prec)
    // };

    let mut can = can1;
    can.set_protocol_exception_handling(false);

    info!("-- Configure nominal timing");
    can.set_nominal_bit_timing(nominal_bit_timing);

    info!("-- Configure data phase timing");
    can.set_data_bit_timing(data_bit_timing);

    info!("-- Configure Filters");
    can.set_standard_filter(
        StandardFilterSlot::_0,
        StandardFilter::accept_all_into_fifo0(),
    );

    info!("-- Apply CAN configuration");
    let config = can
        .get_config()
        .set_frame_transmit(FrameTransmissionConfig::AllowFdCanAndBRS);
    can.apply_config(config);

    info!("-- Set CAN into loopback mode");
    let mut can = can.into_external_loopback();
    //let mut can = can.into_normal();

    info!("Create Message Data");
    let mut buffer = [
        0xA1, 0xA2, 0xAA, 0xAA, 0xFF, 0xFF, 0xFF, 0xFF, 0x0, 0x0, 0x0, 0x0,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    ];
    info!("Create Message Header");
    let header = TxFrameHeader {
        len: 8,
        id: StandardId::new(0x1).unwrap().into(),
        frame_format: FrameFormat::Fdcan,
        bit_rate_switching: true,
        marker: None,
    };
    info!("Initial Header: {:#X?}", &header);

    info!("Transmit initial message");
    block!(can.transmit(header, &buffer)).unwrap();

    loop {
        let rxheader = block!(can.receive0(&mut buffer)).unwrap();
        info!("Received Header: {:#X?}", rxheader);
        info!("received data: {:X?}", &buffer);

        delay.delay_ms(1_u16);
        block!(can.transmit(rxheader.unwrap().to_tx_header(None), &buffer))
            .unwrap();
        info!("Transmit: {:X?}", buffer);
    }
}
