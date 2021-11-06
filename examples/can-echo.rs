#![no_main]
#![no_std]

use crate::hal::{
    fdcan::{
        config::NominalBitTiming,
        filter::{StandardFilter, StandardFilterSlot},
        frame::{FrameFormat, TxFrameHeader},
        id::StandardId,
        FdCan,
    },
    gpio::{GpioExt as _, Speed},
    nb::block,
    rcc::{Config, RccExt, SysClockSrc},
    stm32::Peripherals,
    time::U32Ext,
};
use stm32g4xx_hal as hal;

use core::num::{NonZeroU16, NonZeroU8};

use cortex_m_rt::entry;

use log::info;

#[macro_use]
mod utils;

#[entry]
fn main() -> ! {
    utils::logger::init();

    info!("Start");

    // APB1 (HSE): 24MHz, Bit rate: 125kBit/s, Sample Point 87.5%
    // Value was calculated with http://www.bittiming.can-wiki.info/
    // TODO: use the can_bit_timings crate
    let btr = NominalBitTiming {
        prescaler: NonZeroU16::new(12).unwrap(),
        seg1: NonZeroU8::new(13).unwrap(),
        seg2: NonZeroU8::new(2).unwrap(),
        sync_jump_width: NonZeroU8::new(1).unwrap(),
    };

    info!("Init Clocks");

    let dp = Peripherals::take().unwrap();
    let _cp = cortex_m::Peripherals::take().expect("cannot take core peripherals");
    let rcc = dp.RCC.constrain();
    let mut rcc = rcc.freeze(Config::new(SysClockSrc::HSE(24.mhz())));

    info!("Split GPIO");

    let gpiob = dp.GPIOB.split(&mut rcc);

    let can1 = {
        info!("Init CAN 1");
        let rx = gpiob.pb8.into_alternate().set_speed(Speed::VeryHigh);
        let tx = gpiob.pb9.into_alternate().set_speed(Speed::VeryHigh);

        info!("-- Create CAN 1 instance");
        let can = FdCan::new(dp.FDCAN1, tx, rx, &rcc);

        info!("-- Set CAN 1 in Config Mode");
        let mut can = can.into_config_mode();
        can.set_protocol_exception_handling(false);

        info!("-- Configure nominal timing");
        can.set_nominal_bit_timing(btr);

        info!("-- Configure Filters");
        can.set_standard_filter(
            StandardFilterSlot::_0,
            StandardFilter::accept_all_into_fifo0(),
        );

        info!("-- Current Config: {:#?}", can.get_config());

        info!("-- Set CAN1 in to normal mode");
        // can.into_external_loopback()
        can.into_normal()
    };

    // let can2 = {
    //     info!("Init CAN 2");
    //     let rx = gpiob.pb5.into_alternate().set_speed(Speed::VeryHigh);
    //     let tx = gpiob.pb13.into_alternate().set_speed(Speed::VeryHigh);

    //     info!("-- Create CAN 2 instance");
    //     let can = FdCan::new(dp.FDCAN2, tx, rx, &rcc);

    //     info!("-- Set CAN in Config Mode");
    //     let mut can = can.into_config_mode();

    //     info!("-- Configure nominal timing");
    //     can.set_nominal_bit_timing(btr);

    //     info!("-- Configure Filters");
    //     can.set_standard_filter(
    //         StandardFilterSlot::_0,
    //         StandardFilter::accept_all_into_fifo0(),
    //     );

    //     info!("-- Set CAN2 in to normal mode");
    //     // can.into_external_loopback()
    //     can.into_normal()
    // };

    let mut can = can1;

    info!("Create Message Data");
    let mut buffer = [0xAAAAAAAA, 0xFFFFFFFF, 0x0, 0x0, 0x0, 0x0];
    info!("Create Message Header");
    let header = TxFrameHeader {
        len: 2 * 4,
        id: StandardId::new(0x1).unwrap().into(),
        frame_format: FrameFormat::Standard,
        bit_rate_switching: false,
        marker: None,
    };
    info!("Initial Header: {:#X?}", &header);

    info!("Transmit initial message");
    block!(can.transmit(header, &mut |b| {
        let len = b.len();
        b[..len].clone_from_slice(&buffer[..len]);
    },))
    .unwrap();

    loop {
        if let Ok(rxheader) = block!(can.receive0(&mut |h, b| {
            info!("Received Header: {:#X?}", &h);
            info!("received data: {:X?}", &b);

            for (i, d) in b.iter().enumerate() {
                buffer[i] = *d;
            }
            h
        })) {
            block!(
                can.transmit(rxheader.unwrap().to_tx_header(None), &mut |b| {
                    let len = b.len();
                    b[..len].clone_from_slice(&buffer[..len]);
                    info!("Transmit: {:X?}", b);
                })
            )
            .unwrap();
        }
    }
}
