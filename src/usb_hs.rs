//! USB OTG peripherals
//!
//! ## ULPI Transciever Delay
//!
//! Some ULPI PHYs like the Microchip USB334x series require a delay between the
//! ULPI register write that initiates the HS Chirp and the subsequent transmit
//! command, otherwise the HS Chirp does not get executed and the deivce
//! enumerates in FS mode. The STM32H7 series supports adding this delay to work
//! with the affected PHYs. Enable the `synopsys-usb-otg/xcvrdly` feature to add
//! this delay.
//!
//! # Examples
//!
//! - [USB Serial Port](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/usb_serial.rs)
//! - [USB Passthrough Examples](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/usb_passthrough.rs)
//! - [USB Serial Port with Interrupts and RTIC](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/usb_rtic.rs)

use crate::rcc;
use crate::stm32;

use crate::gpio::{self, alt::otg_hs as alt};

use crate::time::Hertz;

pub use synopsys_usb_otg::UsbBus;
use synopsys_usb_otg::UsbPeripheral;

pub struct USB1 {
    pub usb_global: stm32::OTG1_HS_GLOBAL,
    pub usb_device: stm32::OTG1_HS_DEVICE,
    pub usb_pwrclk: stm32::OTG1_HS_PWRCLK,
    pub prec: rcc::rec::Usb1Otg,
    pub hclk: Hertz,
}
impl USB1 {
    pub fn new(
        usb_global: stm32::OTG1_HS_GLOBAL,
        usb_device: stm32::OTG1_HS_DEVICE,
        usb_pwrclk: stm32::OTG1_HS_PWRCLK,
        pin_dm: impl Into<alt::Dm>,
        pin_dp: impl Into<alt::Dp>,
        prec: rcc::rec::Usb1Otg,
        clocks: &rcc::CoreClocks,
    ) -> Self {
        let _ = pin_dm.into();
        let _ = pin_dp.into();
        Self::new_unchecked(usb_global, usb_device, usb_pwrclk, prec, clocks)
    }
    pub fn new_unchecked(
        usb_global: stm32::OTG1_HS_GLOBAL,
        usb_device: stm32::OTG1_HS_DEVICE,
        usb_pwrclk: stm32::OTG1_HS_PWRCLK,
        prec: rcc::rec::Usb1Otg,
        clocks: &rcc::CoreClocks,
    ) -> Self {
        USB1 {
            usb_global,
            usb_device,
            usb_pwrclk,
            prec,
            hclk: clocks.hclk(),
        }
    }
}

#[cfg(any(feature = "rm0433", feature = "rm0399"))]
pub struct USB2 {
    pub usb_global: stm32::OTG2_HS_GLOBAL,
    pub usb_device: stm32::OTG2_HS_DEVICE,
    pub usb_pwrclk: stm32::OTG2_HS_PWRCLK,
    pub prec: rcc::rec::Usb2Otg,
    pub hclk: Hertz,
}
#[cfg(any(feature = "rm0433", feature = "rm0399"))]
impl USB2 {
    pub fn new(
        usb_global: stm32::OTG2_HS_GLOBAL,
        usb_device: stm32::OTG2_HS_DEVICE,
        usb_pwrclk: stm32::OTG2_HS_PWRCLK,
        pin_dm: impl Into<gpio::alt::otg_fs::Dm>,
        pin_dp: impl Into<gpio::alt::otg_fs::Dp>,
        prec: rcc::rec::Usb2Otg,
        clocks: &rcc::CoreClocks,
    ) -> Self {
        let _ = pin_dm.into();
        let _ = pin_dp.into();
        Self::new_unchecked(usb_global, usb_device, usb_pwrclk, prec, clocks)
    }
    pub fn new_unchecked(
        usb_global: stm32::OTG2_HS_GLOBAL,
        usb_device: stm32::OTG2_HS_DEVICE,
        usb_pwrclk: stm32::OTG2_HS_PWRCLK,
        prec: rcc::rec::Usb2Otg,
        clocks: &rcc::CoreClocks,
    ) -> Self {
        USB2 {
            usb_global,
            usb_device,
            usb_pwrclk,
            prec,
            hclk: clocks.hclk(),
        }
    }
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

                self.hclk.raw()
            }
        }
    };
}

usb_peripheral! {
    USB1, OTG1_HS_GLOBAL, usb1otgen, usb1otgrst
}
pub type Usb1BusType = UsbBus<USB1>;

#[cfg(any(feature = "rm0433", feature = "rm0399"))]
usb_peripheral! {
    USB2, OTG2_HS_GLOBAL, usb2otgen, usb2otgrst
}
#[cfg(any(feature = "rm0433", feature = "rm0399"))]
pub type Usb2BusType = UsbBus<USB2>;

pub struct USB1_ULPI {
    pub usb_global: stm32::OTG1_HS_GLOBAL,
    pub usb_device: stm32::OTG1_HS_DEVICE,
    pub usb_pwrclk: stm32::OTG1_HS_PWRCLK,
    pub prec: rcc::rec::Usb1Otg,
    pub hclk: Hertz,
}

impl USB1_ULPI {
    /// Automatically sets all upli pins to gpio speed VeryHigh.
    /// If you wish to use another configuration,
    /// please see [new_unchecked](USB1_ULPI::new_unchecked).
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        usb_global: stm32::OTG1_HS_GLOBAL,
        usb_device: stm32::OTG1_HS_DEVICE,
        usb_pwrclk: stm32::OTG1_HS_PWRCLK,
        ulpi_clk: impl Into<alt::UlpiCk>,
        ulpi_dir: impl Into<alt::UlpiDir>,
        ulpi_nxt: impl Into<alt::UlpiNxt>,
        ulpi_stp: impl Into<alt::UlpiStp>,
        ulpi_d0: impl Into<alt::UlpiD0>,
        ulpi_d1: impl Into<alt::UlpiD1>,
        ulpi_d2: impl Into<alt::UlpiD2>,
        ulpi_d3: impl Into<alt::UlpiD3>,
        ulpi_d4: impl Into<alt::UlpiD4>,
        ulpi_d5: impl Into<alt::UlpiD5>,
        ulpi_d6: impl Into<alt::UlpiD6>,
        ulpi_d7: impl Into<alt::UlpiD7>,
        prec: rcc::rec::Usb1Otg,
        clocks: &rcc::CoreClocks,
    ) -> Self {
        let _ = ulpi_clk.into();
        let _ = ulpi_dir.into();
        let _ = ulpi_nxt.into();
        let _ = ulpi_stp.into();
        let _ = ulpi_d0.into();
        let _ = ulpi_d1.into();
        let _ = ulpi_d2.into();
        let _ = ulpi_d3.into();
        let _ = ulpi_d4.into();
        let _ = ulpi_d5.into();
        let _ = ulpi_d6.into();
        let _ = ulpi_d7.into();
        Self::new_unchecked(usb_global, usb_device, usb_pwrclk, prec, clocks)
    }
    pub fn new_unchecked(
        usb_global: stm32::OTG1_HS_GLOBAL,
        usb_device: stm32::OTG1_HS_DEVICE,
        usb_pwrclk: stm32::OTG1_HS_PWRCLK,
        prec: rcc::rec::Usb1Otg,
        clocks: &rcc::CoreClocks,
    ) -> Self {
        USB1_ULPI {
            usb_global,
            usb_device,
            usb_pwrclk,
            prec,
            hclk: clocks.hclk(),
        }
    }
}

unsafe impl Sync for USB1_ULPI {}

unsafe impl UsbPeripheral for USB1_ULPI {
    const REGISTERS: *const () = stm32::OTG1_HS_GLOBAL::ptr() as *const ();

    const HIGH_SPEED: bool = true;
    const FIFO_DEPTH_WORDS: usize = 1024;
    const ENDPOINT_COUNT: usize = 9;

    fn enable() {
        let rcc = unsafe { &*stm32::RCC::ptr() };

        cortex_m::interrupt::free(|_| {
            // Enable USB peripheral
            rcc.ahb1enr.modify(|_, w| w.usb1otgen().enabled());

            // Enable ULPI Clock
            rcc.ahb1enr.modify(|_, w| w.usb1ulpien().enabled());

            // Reset USB peripheral
            rcc.ahb1rstr.modify(|_, w| w.usb1otgrst().set_bit());
            rcc.ahb1rstr.modify(|_, w| w.usb1otgrst().clear_bit());
        });
    }

    fn ahb_frequency_hz(&self) -> u32 {
        self.hclk.raw()
    }

    fn phy_type(&self) -> synopsys_usb_otg::PhyType {
        synopsys_usb_otg::PhyType::ExternalHighSpeed
    }
}
pub type Usb1UlpiBusType = UsbBus<USB1_ULPI>;
