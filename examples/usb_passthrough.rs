//! Dual CDC-ACM serial port example using polling in a busy loop.
//!
//! Characters written to one serial port appear on both.
//!
//! This example uses both USB1 and USB2. This is only possible on devices that
//! have the USB2 peripheral.
#![deny(warnings)]
#![allow(static_mut_refs)]
#![no_std]
#![no_main]

use panic_itm as _;

use core::mem::MaybeUninit;

use cortex_m_rt::entry;

use stm32h7xx_hal::rcc::rec::UsbClkSel;
use stm32h7xx_hal::usb_hs::{UsbBus, USB1, USB2};
use stm32h7xx_hal::{prelude::*, stm32};

use usb_device::prelude::*;

static mut EP_MEMORY_1: MaybeUninit<[u32; 1024]> = MaybeUninit::uninit();
static mut EP_MEMORY_2: MaybeUninit<[u32; 1024]> = MaybeUninit::uninit();

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().unwrap();

    // Power
    let pwr = dp.PWR.constrain();
    let vos = pwr.freeze();

    // RCC
    let rcc = dp.RCC.constrain();
    let mut ccdr = rcc.sys_ck(120.MHz()).freeze(vos, &dp.SYSCFG);

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
    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);

    let usb1 = USB1::new(
        dp.OTG1_HS_GLOBAL,
        dp.OTG1_HS_DEVICE,
        dp.OTG1_HS_PWRCLK,
        gpiob.pb14.into_alternate(),
        gpiob.pb15.into_alternate(),
        ccdr.peripheral.USB1OTG,
        &ccdr.clocks,
    );

    let usb2 = USB2::new(
        dp.OTG2_HS_GLOBAL,
        dp.OTG2_HS_DEVICE,
        dp.OTG2_HS_PWRCLK,
        gpioa.pa11.into_alternate(),
        gpioa.pa12.into_alternate(),
        ccdr.peripheral.USB2OTG,
        &ccdr.clocks,
    );

    // Initialise EP_MEMORY_1 to zero
    unsafe {
        let buf: &mut [MaybeUninit<u32>; 1024] =
            &mut *(core::ptr::addr_of_mut!(EP_MEMORY_1) as *mut _);
        for value in buf.iter_mut() {
            value.as_mut_ptr().write(0);
        }
    }
    // Initialise EP_MEMORY_2 to zero
    unsafe {
        let buf: &mut [MaybeUninit<u32>; 1024] =
            &mut *(core::ptr::addr_of_mut!(EP_MEMORY_2) as *mut _);
        for value in buf.iter_mut() {
            value.as_mut_ptr().write(0);
        }
    }

    // Port 1
    let usb1_bus = UsbBus::new(usb1, unsafe { EP_MEMORY_1.assume_init_mut() });
    let mut serial1 = usbd_serial::SerialPort::new(&usb1_bus);
    let mut usb1_dev =
        UsbDeviceBuilder::new(&usb1_bus, UsbVidPid(0x16c0, 0x27dd))
            .strings(&[usb_device::device::StringDescriptors::default()
                .manufacturer("Fake company")
                .product("Serial port")
                .serial_number("TEST PORT 1")])
            .unwrap()
            .device_class(usbd_serial::USB_CLASS_CDC)
            .build();

    // Port 2
    let usb2_bus = UsbBus::new(usb2, unsafe { EP_MEMORY_2.assume_init_mut() });
    let mut serial2 = usbd_serial::SerialPort::new(&usb2_bus);
    let mut usb2_dev =
        UsbDeviceBuilder::new(&usb2_bus, UsbVidPid(0x16c0, 0x27dd))
            .strings(&[usb_device::device::StringDescriptors::default()
                .manufacturer("Fake company")
                .product("Serial port")
                .serial_number("TEST PORT 1")])
            .unwrap()
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
