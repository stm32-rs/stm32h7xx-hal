//! CDC-ACM serial port example using polling in a busy loop
//!
#![deny(warnings)]
#![no_std]
#![no_main]

#[macro_use]
mod utilities;

#[rtic::app(device = stm32h7xx_hal::stm32, peripherals = true)]
mod app {
    use stm32h7xx_hal::gpio::gpioe::PE1;
    use stm32h7xx_hal::gpio::{Output, PushPull};
    use stm32h7xx_hal::prelude::*;
    use stm32h7xx_hal::rcc::rec::UsbClkSel;
    use stm32h7xx_hal::usb_hs::{UsbBus, USB1};
    use usb_device::prelude::*;

    static mut EP_MEMORY: [u32; 1024] = [0; 1024];
    use super::utilities;

    #[shared]
    struct SharedResources {}
    #[local]
    struct LocalResources {
        usb: (
            UsbDevice<'static, UsbBus<USB1>>,
            usbd_serial::SerialPort<'static, UsbBus<USB1>>,
        ),
        led: PE1<Output<PushPull>>,
    }

    #[init]
    fn init(
        ctx: init::Context,
    ) -> (SharedResources, LocalResources, init::Monotonics) {
        utilities::logger::init();
        let pwr = ctx.device.PWR.constrain();
        let pwrcfg = example_power!(pwr).freeze();

        // RCC
        let rcc = ctx.device.RCC.constrain();
        let mut ccdr = rcc.sys_ck(80.mhz()).freeze(pwrcfg, &ctx.device.SYSCFG);

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

        // IO
        #[cfg(any(feature = "rm0433", feature = "rm0399"))]
        let (pin_dm, pin_dp) = {
            let gpiob = ctx.device.GPIOB.split(ccdr.peripheral.GPIOB);
            (gpiob.pb14.into_alternate(), gpiob.pb15.into_alternate())
        };

        #[cfg(any(feature = "rm0455", feature = "rm0468"))]
        let (pin_dm, pin_dp) = {
            let gpioa = ctx.device.GPIOA.split(ccdr.peripheral.GPIOA);
            (gpioa.pa11.into_alternate(), gpioa.pa12.into_alternate())
        };

        let led = ctx.device.GPIOE.split(ccdr.peripheral.GPIOE).pe1;
        let usb = USB1::new(
            ctx.device.OTG1_HS_GLOBAL,
            ctx.device.OTG1_HS_DEVICE,
            ctx.device.OTG1_HS_PWRCLK,
            pin_dm,
            pin_dp,
            ccdr.peripheral.USB1OTG,
            &ccdr.clocks,
        );

        let usb_bus = cortex_m::singleton!(
            : usb_device::class_prelude::UsbBusAllocator<UsbBus<USB1>> =
                UsbBus::new(usb, unsafe { &mut EP_MEMORY })
        )
        .unwrap();
        let serial = usbd_serial::SerialPort::new(usb_bus);
        let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")
            .device_class(usbd_serial::USB_CLASS_CDC)
            .build();
        let usb = (usb_dev, serial);

        (
            SharedResources {},
            LocalResources {
                usb,
                led: led.into_push_pull_output(),
            },
            init::Monotonics(),
        )
    }

    #[task(binds = OTG_HS, local = [usb,led])]
    fn usb_event(mut ctx: usb_event::Context) {
        let (usb_dev, serial) = &mut ctx.local.usb;
        ctx.local.led.set_high();

        loop {
            if !usb_dev.poll(&mut [serial]) {
                ctx.local.led.set_low();
                return;
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
}
