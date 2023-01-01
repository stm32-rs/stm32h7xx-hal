//! Example of half-duplex USART communication between two USART's.
//!
//! Connect the USART3_TX (PC10) and USART6_TX (PC6) and an external pullup resistor together.
//! i.e., connect one end of a resistor to a 3.3V pin and connect both PC10 and PC6 pins to the other end
//! of the resistor. 330 ohm resistor worked but exact value is not critical.
//!
//! It will print "Hello, world!" over and over again one character at a time.
#![deny(warnings)]
#![no_main]
#![no_std]

use cortex_m_rt::entry;
#[macro_use]
mod utilities;
use log::info;

use stm32h7xx_hal::{pac, prelude::*, serial};

use nb::block;

#[entry]
fn main() -> ! {
    utilities::logger::init();
    let dp = pac::Peripherals::take().unwrap();

    // Constrain and Freeze power
    let pwr = dp.PWR.constrain();
    let pwrcfg = example_power!(pwr).freeze();

    // Constrain and Freeze clock
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sys_ck(160.MHz()).freeze(pwrcfg, &dp.SYSCFG);

    // Acquire the GPIOC peripheral. This also enables the clock for
    // GPIOC in the RCC register.
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);

    let usart3txpin = gpioc.pc10.into_alternate::<7>();
    let usart3rxpin = gpioc.pc11.into_alternate::<7>();
    let usart6txpin = gpioc.pc6.into_alternate::<7>();
    let usart6rxpin = gpioc.pc7.into_alternate::<7>();

    info!("stm32h7xx-hal example - USART half-duplex");

    // Configure the serial peripheral.
    let config = serial::config::Config::new(9600.bps())
        .parity_none()
        .halfduplex(true)
        .stopbits(serial::config::StopBits::Stop1);

    let mut usart3 = dp
        .USART3
        .serial(
            (usart3txpin, usart3rxpin),
            config,
            ccdr.peripheral.USART3,
            &ccdr.clocks,
        )
        .unwrap();

    let mut usart6 = dp
        .USART6
        .serial(
            (usart6txpin, usart6rxpin),
            config,
            ccdr.peripheral.USART6,
            &ccdr.clocks,
        )
        .unwrap();

    loop {
        // Tx from usart3
        for c in "Hello, world!".chars() {
            let _res = block!(usart3.write(c as u8)).unwrap();
            let received = block!(usart6.read()).unwrap();
            info!("usart6 rx {}", received as char);
        }

        // Tx from usart6
        for c in "Hello, world!".chars() {
            let _res = block!(usart6.write(c as u8)).unwrap();
            let received = block!(usart3.read()).unwrap();
            info!("usart3 rx {}", received as char);
        }
    }
}
