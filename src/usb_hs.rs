//! USB OTG peripherals
//!
//! Requires the `usb_hs` feature.
//!
//! # Examples
//!
//! - [USB Serial Port](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/usb_serial.rs)
//! - [USB Passthrough Examples](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/usb_passthrough.rs)

use crate::rcc;
use crate::stm32;

use crate::gpio::{
    gpioa::{PA11, PA12, PA3, PA5},
    gpiob::{PB0, PB1, PB10, PB11, PB12, PB13, PB5},
    gpioc::{PC0, PC2, PC3},
    gpioh::PH4,
    Alternate, Speed, AF10,
};

#[cfg(not(feature = "rm0468"))]
use crate::gpio::gpioi::PI11;

#[cfg(any(feature = "rm0433", feature = "rm0399"))]
use crate::gpio::{
    gpiob::{PB14, PB15},
    AF12,
};

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
    #[cfg(any(feature = "rm0433", feature = "rm0399"))]
    pub fn new(
        usb_global: stm32::OTG1_HS_GLOBAL,
        usb_device: stm32::OTG1_HS_DEVICE,
        usb_pwrclk: stm32::OTG1_HS_PWRCLK,
        _pin_dm: PB14<Alternate<AF12>>,
        _pin_dp: PB15<Alternate<AF12>>,
        prec: rcc::rec::Usb1Otg,
        clocks: &rcc::CoreClocks,
    ) -> Self {
        Self::new_unchecked(usb_global, usb_device, usb_pwrclk, prec, clocks)
    }
    #[cfg(any(feature = "rm0455", feature = "rm0468"))]
    pub fn new(
        usb_global: stm32::OTG1_HS_GLOBAL,
        usb_device: stm32::OTG1_HS_DEVICE,
        usb_pwrclk: stm32::OTG1_HS_PWRCLK,
        _pin_dm: PA11<Alternate<AF10>>,
        _pin_dp: PA12<Alternate<AF10>>,
        prec: rcc::rec::Usb1Otg,
        clocks: &rcc::CoreClocks,
    ) -> Self {
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
        _pin_dm: PA11<Alternate<AF10>>,
        _pin_dp: PA12<Alternate<AF10>>,
        prec: rcc::rec::Usb2Otg,
        clocks: &rcc::CoreClocks,
    ) -> Self {
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

                self.hclk.0
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

pub enum Usb1UlpiDirPin {
    PC2(PC2<Alternate<AF10>>),
    #[cfg(not(feature = "rm0468"))]
    PI11(PI11<Alternate<AF10>>),
}

impl Usb1UlpiDirPin {
    fn set_speed(self, speed: Speed) -> Usb1UlpiDirPin {
        match self {
            Usb1UlpiDirPin::PC2(pin) => {
                Usb1UlpiDirPin::PC2(pin.set_speed(speed))
            }
            #[cfg(not(feature = "rm0468"))]
            Usb1UlpiDirPin::PI11(pin) => {
                Usb1UlpiDirPin::PI11(pin.set_speed(speed))
            }
        }
    }
}

#[cfg(not(feature = "rm0468"))]
impl From<PI11<Alternate<AF10>>> for Usb1UlpiDirPin {
    fn from(v: PI11<Alternate<AF10>>) -> Self {
        Usb1UlpiDirPin::PI11(v)
    }
}

impl From<PC2<Alternate<AF10>>> for Usb1UlpiDirPin {
    fn from(v: PC2<Alternate<AF10>>) -> Self {
        Usb1UlpiDirPin::PC2(v)
    }
}

pub enum Usb1UlpiNxtPin {
    PC3(PC3<Alternate<AF10>>),
    PH4(PH4<Alternate<AF10>>),
}

impl Usb1UlpiNxtPin {
    fn set_speed(self, speed: Speed) -> Usb1UlpiNxtPin {
        match self {
            Usb1UlpiNxtPin::PC3(pin) => {
                Usb1UlpiNxtPin::PC3(pin.set_speed(speed))
            }
            Usb1UlpiNxtPin::PH4(pin) => {
                Usb1UlpiNxtPin::PH4(pin.set_speed(speed))
            }
        }
    }
}

impl From<PH4<Alternate<AF10>>> for Usb1UlpiNxtPin {
    fn from(v: PH4<Alternate<AF10>>) -> Self {
        Usb1UlpiNxtPin::PH4(v)
    }
}

impl From<PC3<Alternate<AF10>>> for Usb1UlpiNxtPin {
    fn from(v: PC3<Alternate<AF10>>) -> Self {
        Usb1UlpiNxtPin::PC3(v)
    }
}

impl USB1_ULPI {
    /// Automatically sets all upli pins to gpio speed VeryHigh.
    /// If you wish to use another configuration,
    /// please see [new_unchecked](USB1_ULPI::new_unchecked).
    pub fn new(
        usb_global: stm32::OTG1_HS_GLOBAL,
        usb_device: stm32::OTG1_HS_DEVICE,
        usb_pwrclk: stm32::OTG1_HS_PWRCLK,
        ulpi_clk: PA5<Alternate<AF10>>,
        ulpi_dir: impl Into<Usb1UlpiDirPin>,
        ulpi_nxt: impl Into<Usb1UlpiNxtPin>,
        ulpi_stp: PC0<Alternate<AF10>>,
        ulpi_d0: PA3<Alternate<AF10>>,
        ulpi_d1: PB0<Alternate<AF10>>,
        ulpi_d2: PB1<Alternate<AF10>>,
        ulpi_d3: PB10<Alternate<AF10>>,
        ulpi_d4: PB11<Alternate<AF10>>,
        ulpi_d5: PB12<Alternate<AF10>>,
        ulpi_d6: PB13<Alternate<AF10>>,
        ulpi_d7: PB5<Alternate<AF10>>,
        prec: rcc::rec::Usb1Otg,
        clocks: &rcc::CoreClocks,
    ) -> Self {
        ulpi_clk.set_speed(Speed::VeryHigh);
        ulpi_dir.into().set_speed(Speed::VeryHigh);
        ulpi_nxt.into().set_speed(Speed::VeryHigh);
        ulpi_stp.set_speed(Speed::VeryHigh);
        ulpi_d0.set_speed(Speed::VeryHigh);
        ulpi_d1.set_speed(Speed::VeryHigh);
        ulpi_d2.set_speed(Speed::VeryHigh);
        ulpi_d3.set_speed(Speed::VeryHigh);
        ulpi_d4.set_speed(Speed::VeryHigh);
        ulpi_d5.set_speed(Speed::VeryHigh);
        ulpi_d6.set_speed(Speed::VeryHigh);
        ulpi_d7.set_speed(Speed::VeryHigh);
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
        self.hclk.0
    }

    fn phy_type(&self) -> synopsys_usb_otg::PhyType {
        synopsys_usb_otg::PhyType::ExternalHighSpeed
    }
}
pub type Usb1UlpiBusType = UsbBus<USB1_ULPI>;
