//! USB OTG peripherals
//!
//! Requires the `usb_hs` feature.
//!
//! Note that only full-speed mode is supported,
//! external high-speed PHY is not supported.

use crate::rcc;
use crate::stm32;

use crate::gpio::{
    gpioa::{PA11, PA12},
    Alternate, AF10,
};
#[cfg(not(feature = "rm0455"))]
use crate::gpio::{
    gpiob::{PB14, PB15},
    AF12,
};

use crate::time::Hertz;

pub use synopsys_usb_otg::UsbBus;
use synopsys_usb_otg::UsbPeripheral;

#[cfg(not(feature = "rm0455"))]
pub struct USB1 {
    pub usb_global: stm32::OTG1_HS_GLOBAL,
    pub usb_device: stm32::OTG1_HS_DEVICE,
    pub usb_pwrclk: stm32::OTG1_HS_PWRCLK,
    pub pin_dm: PB14<Alternate<AF12>>,
    pub pin_dp: PB15<Alternate<AF12>>,
    pub prec: rcc::rec::Usb1Otg,
    pub hclk: Hertz,
}
#[cfg(not(feature = "rm0455"))]
pub struct USB2 {
    pub usb_global: stm32::OTG2_HS_GLOBAL,
    pub usb_device: stm32::OTG2_HS_DEVICE,
    pub usb_pwrclk: stm32::OTG2_HS_PWRCLK,
    pub pin_dm: PA11<Alternate<AF10>>,
    pub pin_dp: PA12<Alternate<AF10>>,
    pub prec: rcc::rec::Usb2Otg,
    pub hclk: Hertz,
}

#[cfg(feature = "rm0455")]
pub struct USB1 {
    pub usb_global: stm32::OTG1_HS_GLOBAL,
    pub usb_device: stm32::OTG1_HS_DEVICE,
    pub usb_pwrclk: stm32::OTG1_HS_PWRCLK,
    pub pin_dm: PA11<Alternate<AF10>>,
    pub pin_dp: PA12<Alternate<AF10>>,
    pub prec: rcc::rec::Usb1Otg,
    pub hclk: Hertz,
}

macro_rules! usb_peripheral {
    ($USB:ident, $GLOBAL:ident, $en:ident, $rst:ident) => {
        unsafe impl Sync for $USB {}

        unsafe impl UsbPeripheral for $USB {
            const REGISTERS: *const () = stm32::$GLOBAL::ptr() as *const ();

            const HIGH_SPEED: bool = true;
            const FIFO_DEPTH_WORDS: usize = 1024;

            const ENDPOINT_COUNT: usize = 9;

            fn enable() {
                let pwr = unsafe { &*stm32::PWR::ptr() };
                let rcc = unsafe { &*stm32::RCC::ptr() };

                cortex_m::interrupt::free(|_| {
                    // USB Regulator in BYPASS mode
                    pwr.cr3.modify(|_, w| w.usb33den().set_bit());

                    // Enable USB peripheral
                    rcc.ahb1enr.modify(|_, w| w.$en().set_bit());

                    // Reset USB peripheral
                    rcc.ahb1rstr.modify(|_, w| w.$rst().set_bit());
                    rcc.ahb1rstr.modify(|_, w| w.$rst().clear_bit());
                });
            }

            fn ahb_frequency_hz(&self) -> u32 {
                // For correct operation, the AHB frequency should be higher
                // than 30MHz. See RM0433 Rev 7. Section 57.4.4. This is checked
                // by the UsbBus implementation in synopsys-usb-otg.

                self.hclk.0
            }
        }
    };
}

usb_peripheral! {
    USB1, OTG1_HS_GLOBAL, usb1otgen, usb1otgrst
}
pub type Usb1BusType = UsbBus<USB1>;

#[cfg(not(feature = "rm0455"))]
usb_peripheral! {
    USB2, OTG2_HS_GLOBAL, usb2otgen, usb2otgrst
}
#[cfg(not(feature = "rm0455"))]
pub type Usb2BusType = UsbBus<USB2>;
