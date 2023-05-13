//! CDC-ACM serial port example using polling in a busy loop
//!
//! Tested with a NUCLEO-H723ZG
//!
//! This example uses the USB1 peripheral. On parts that have multiple USB
//! OTG_HS peripherals the USB1 D+/D- pins are located on PB14 and PB15. If your
//! development board uses PA11 and PA12 instead, you should adapt the example
//! to use the USB2 peripheral together with PA11 and PA12. This applies to the
//! NUCLEO-H743ZI2 board.
//!
#![no_std]
#![no_main]

#[macro_use]
#[allow(unused)]
mod utilities;

use cortex_m_rt::entry;

use stm32h7xx_hal::rcc::rec::UsbClkSel;
use stm32h7xx_hal::usb_hs::{UsbBus, USB1};
use stm32h7xx_hal::{prelude::*, stm32};

use usb_device::prelude::*;

static mut EP_MEMORY: [u32; 1024] = [0; 1024];

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().unwrap();

    // Power
    let pwr = dp.PWR.constrain();
    let vos = example_power!(pwr).freeze();

    // RCC
    let rcc = dp.RCC.constrain();
    let mut ccdr = rcc.sys_ck(80.MHz()).freeze(vos, &dp.SYSCFG);

    // 48MHz CLOCK
    let _ = ccdr.clocks.hsi48_ck().expect("HSI48 must run");
    ccdr.peripheral.kernel_usb_clk_mux(UsbClkSel::Hsi48);

    // If your hardware uses the internal USB voltage regulator in ON mode, you
    // should uncomment this block.
    // unsafe {
    //     let pwr = &*stm32::PWR::ptr();
    //     pwr.cr3.modify(|_, w| w.usbregen().set_bit());
    //     while pwr.cr3.read().usb33rdy().bit_is_clear() {}
    // }

    // IO
    #[cfg(any(feature = "rm0433", feature = "rm0399"))]
    let (pin_dm, pin_dp) = {
        let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
        (gpiob.pb14, gpiob.pb15)
    };

    #[cfg(any(feature = "rm0455", feature = "rm0468"))]
    let (pin_dm, pin_dp) = {
        let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
        (gpioa.pa11, gpioa.pa12)
    };

    let usb = USB1::new(
        dp.OTG1_HS_GLOBAL,
        dp.OTG1_HS_DEVICE,
        dp.OTG1_HS_PWRCLK,
        pin_dm,
        pin_dp,
        ccdr.peripheral.USB1OTG,
        &ccdr.clocks,
    );

    let usb_bus = UsbBus::new(usb, unsafe { &mut EP_MEMORY });

    let mut serial = usbd_serial::SerialPort::new(&usb_bus);

    let mut usb_dev =
        UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")
            .device_class(usbd_serial::USB_CLASS_CDC)
            .build();

    loop {
        if !usb_dev.poll(&mut [&mut serial]) {
            continue;
        }

        let mut buf = [0u8; 64];

        match serial.read(&mut buf) {
            Ok(count) if count > 0 => {
                // Echo back in upper case
                for c in buf[0..count].iter_mut() {
                    if 0x61 <= *c && *c <= 0x7a {
                        *c &= !0x20;
                    }
                }

                let mut write_offset = 0;
                while write_offset < count {
                    match serial.write(&buf[write_offset..count]) {
                        Ok(len) if len > 0 => {
                            write_offset += len;
                        }
                        _ => {}
                    }
                }
            }
            _ => {}
        }
    }
}
