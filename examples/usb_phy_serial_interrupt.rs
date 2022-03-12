//! CDC-ACM serial port example using an external phy and interrupts
#![no_std]
#![no_main]

use {
    core::cell::RefCell,
    cortex_m::interrupt::{free as interrupt_free, Mutex},
    stm32h7xx_hal::{
        interrupt, pac,
        prelude::*,
        rcc::rec::UsbClkSel,
        stm32,
        usb_hs::{UsbBus, USB1_ULPI},
    },
    usb_device::prelude::*,
    usb_device::{bus::UsbBusAllocator, device::UsbDevice},
    usbd_serial::{DefaultBufferStore, SerialPort},
};

#[macro_use]
#[allow(unused)]
mod utilities;

pub const VID: u16 = 0x2341;
pub const PID: u16 = 0x025b;

pub static mut USB_MEMORY_1: [u32; 1024] = [0u32; 1024];
pub static mut USB_BUS_ALLOCATOR: Option<UsbBusAllocator<UsbBus<USB1_ULPI>>> =
    None;
pub static SERIAL_PORT: Mutex<
    RefCell<
        Option<
            SerialPort<
                UsbBus<USB1_ULPI>,
                DefaultBufferStore,
                DefaultBufferStore,
            >,
        >,
    >,
> = Mutex::new(RefCell::new(None));
pub static USB_DEVICE: Mutex<RefCell<Option<UsbDevice<UsbBus<USB1_ULPI>>>>> =
    Mutex::new(RefCell::new(None));

#[cortex_m_rt::entry]
unsafe fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = stm32::Peripherals::take().unwrap();

    // Power
    let pwr = dp.PWR.constrain();
    let vos = example_power!(pwr).freeze();

    // RCC
    let rcc = dp.RCC.constrain();
    let mut ccdr = rcc.sys_ck(80.mhz()).freeze(vos, &dp.SYSCFG);

    // 48MHz CLOCK
    let _ = ccdr.clocks.hsi48_ck().expect("HSI48 must run");
    ccdr.peripheral.kernel_usb_clk_mux(UsbClkSel::HSI48);

    // If your hardware uses the internal USB voltage regulator in ON mode, you
    // should uncomment this block.
    // unsafe {
    //     let pwr = &*stm32::PWR::ptr();
    //     pwr.cr3.modify(|_, w| w.usbregen().set_bit());
    //     while pwr.cr3.read().usb33rdy().bit_is_clear() {}
    // }

    // Get the delay provider.
    let mut delay = cp.SYST.delay(ccdr.clocks);

    // GPIO
    let (
        gpioa,
        gpiob,
        gpioc,
        _gpiod,
        _gpioe,
        _gpiof,
        _gpiog,
        gpioh,
        gpioi,
        gpioj,
        _gpiok,
    ) = {
        (
            dp.GPIOA.split(ccdr.peripheral.GPIOA),
            dp.GPIOB.split(ccdr.peripheral.GPIOB),
            dp.GPIOC.split(ccdr.peripheral.GPIOC),
            dp.GPIOD.split(ccdr.peripheral.GPIOD),
            dp.GPIOE.split(ccdr.peripheral.GPIOE),
            dp.GPIOF.split(ccdr.peripheral.GPIOF),
            dp.GPIOG.split(ccdr.peripheral.GPIOG),
            dp.GPIOH.split(ccdr.peripheral.GPIOH),
            dp.GPIOI.split(ccdr.peripheral.GPIOI),
            dp.GPIOJ.split(ccdr.peripheral.GPIOJ),
            dp.GPIOK.split(ccdr.peripheral.GPIOK),
        )
    };

    // It is very likely that the board you're using has an external
    // clock source. Sometimes you have to enable them by yourself.
    // This is the case on the Arduino Portenta H7.
    let mut oscen = gpioh.ph1.into_push_pull_output();
    delay.delay_ms(10u32);
    oscen.set_high();
    // Wait for osc to be stable
    delay.delay_ms(100u32);

    // Set USB OTG pin floating
    let mut _usb_otg = gpioj.pj6.into_floating_input();

    // Reset USB Phy
    let mut usb_phy_rst = gpioj.pj4.into_push_pull_output();
    usb_phy_rst.set_low();
    delay.delay_ms(10u8);
    usb_phy_rst.set_high();
    delay.delay_ms(10u8);

    // Enable USB OTG_HS interrupt
    cortex_m::peripheral::NVIC::unmask(pac::Interrupt::OTG_HS);

    let usb = USB1_ULPI::new(
        dp.OTG1_HS_GLOBAL,
        dp.OTG1_HS_DEVICE,
        dp.OTG1_HS_PWRCLK,
        gpioa.pa5.into_alternate(),
        gpioi.pi11.into_alternate(),
        gpioh.ph4.into_alternate(),
        gpioc.pc0.into_alternate(),
        gpioa.pa3.into_alternate(),
        gpiob.pb0.into_alternate(),
        gpiob.pb1.into_alternate(),
        gpiob.pb10.into_alternate(),
        gpiob.pb11.into_alternate(),
        gpiob.pb12.into_alternate(),
        gpiob.pb13.into_alternate(),
        gpiob.pb5.into_alternate(),
        ccdr.peripheral.USB1OTG,
        &ccdr.clocks,
    );

    USB_BUS_ALLOCATOR = Some(UsbBus::new(usb, &mut USB_MEMORY_1));

    let usb_serial =
        usbd_serial::SerialPort::new(USB_BUS_ALLOCATOR.as_ref().unwrap());

    let usb_dev = UsbDeviceBuilder::new(
        USB_BUS_ALLOCATOR.as_ref().unwrap(),
        UsbVidPid(VID, PID),
    )
    .manufacturer("Fake company")
    .product("Serial port")
    .serial_number("TEST")
    .device_class(usbd_serial::USB_CLASS_CDC)
    .max_packet_size_0(64)
    .build();

    interrupt_free(|cs| {
        USB_DEVICE.borrow(cs).replace(Some(usb_dev));
        SERIAL_PORT.borrow(cs).replace(Some(usb_serial));
    });

    loop {
        cortex_m::asm::nop();
    }
}

#[interrupt]
fn OTG_HS() {
    interrupt_free(|cs| {
        if let (Some(port), Some(device)) = (
            SERIAL_PORT.borrow(cs).borrow_mut().as_mut(),
            USB_DEVICE.borrow(cs).borrow_mut().as_mut(),
        ) {
            if device.poll(&mut [port]) {
                let mut buf = [0u8; 64];
                match port.read(&mut buf) {
                    Ok(count) if count > 0 => {
                        // Echo back in upper case
                        for c in buf[0..count].iter_mut() {
                            if 0x61 <= *c && *c <= 0x7a {
                                *c &= !0x20;
                            }
                        }

                        let mut write_offset = 0;
                        while write_offset < count {
                            match port.write(&buf[write_offset..count]) {
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
    })
}
