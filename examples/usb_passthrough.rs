//! Dual CDC-ACM serial port example using polling in a busy loop.
//!
//! Characters written to one serial port appear on both.
//!
//! Note: This example must be built in release mode to work reliably
#![no_std]
#![no_main]

use panic_itm as _;

use cortex_m_rt::entry;

use stm32h7xx_hal::rcc::rec::UsbClkSel;
use stm32h7xx_hal::usb_hs::{UsbBus, USB1, USB2};
use stm32h7xx_hal::{prelude::*, stm32};

use usb_device::prelude::*;

static mut EP_MEMORY_1: [u32; 1024] = [0; 1024];
static mut EP_MEMORY_2: [u32; 1024] = [0; 1024];

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().unwrap();

    // Power
    let pwr = dp.PWR.constrain();
    let vos = pwr.freeze();

    // RCC
    let rcc = dp.RCC.constrain();
    let mut ccdr = rcc.sys_ck(120.mhz()).freeze(vos, &dp.SYSCFG);

    // 48MHz CLOCK
    let _ = ccdr.clocks.hsi48_ck().expect("HSI48 must run");
    ccdr.peripheral.kernel_usb_clk_mux(UsbClkSel::HSI48);

    // IO
    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);

    let usb1 = USB1 {
        usb_global: dp.OTG1_HS_GLOBAL,
        usb_device: dp.OTG1_HS_DEVICE,
        usb_pwrclk: dp.OTG1_HS_PWRCLK,
        pin_dm: gpiob.pb14.into_alternate_af12(),
        pin_dp: gpiob.pb15.into_alternate_af12(),
        prec: ccdr.peripheral.USB1OTG,
        hclk: ccdr.clocks.hclk(),
    };
    let usb2 = USB2 {
        usb_global: dp.OTG2_HS_GLOBAL,
        usb_device: dp.OTG2_HS_DEVICE,
        usb_pwrclk: dp.OTG2_HS_PWRCLK,
        pin_dm: gpioa.pa11.into_alternate_af10(),
        pin_dp: gpioa.pa12.into_alternate_af10(),
        prec: ccdr.peripheral.USB2OTG,
        hclk: ccdr.clocks.hclk(),
    };

    // Port 1
    let usb1_bus = UsbBus::new(usb1, unsafe { &mut EP_MEMORY_1 });
    let mut serial1 = usbd_serial::SerialPort::new(&usb1_bus);
    let mut usb1_dev =
        UsbDeviceBuilder::new(&usb1_bus, UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST PORT 1")
            .device_class(usbd_serial::USB_CLASS_CDC)
            .build();

    // Port 2
    let usb2_bus = UsbBus::new(usb2, unsafe { &mut EP_MEMORY_2 });
    let mut serial2 = usbd_serial::SerialPort::new(&usb2_bus);
    let mut usb2_dev =
        UsbDeviceBuilder::new(&usb2_bus, UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST PORT 2")
            .device_class(usbd_serial::USB_CLASS_CDC)
            .build();

    loop {
        let mut buf = [0u8; 64];

        // Port 1
        if usb1_dev.poll(&mut [&mut serial1]) {
            match serial1.read(&mut buf) {
                Ok(count) if count > 0 => {
                    // Write to both ports
                    write_serial(&mut serial1, &buf, count);
                    write_serial(&mut serial2, &buf, count);
                }
                _ => {}
            }
        }

        // Port 2
        if usb2_dev.poll(&mut [&mut serial2]) {
            match serial2.read(&mut buf) {
                Ok(count) if count > 0 => {
                    // Write to both ports
                    write_serial(&mut serial1, &buf, count);
                    write_serial(&mut serial2, &buf, count);
                }
                _ => {}
            }
        }
    }
}

fn write_serial<P: usb_device::bus::UsbBus>(
    serial: &mut usbd_serial::SerialPort<P>,
    buf: &[u8],
    count: usize,
) {
    if serial.rts() {
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
}
