//! USB OTG peripherals
//!
//! Requires the `usb_hs` feature.

use crate::rcc;
use crate::stm32;

use crate::gpio::{
    gpioa::{PA11, PA12, PA3, PA5},
    gpiob::{PB0, PB1, PB10, PB11, PB12, PB13, PB5},
    gpioc::{PC0, PC2, PC3},
    gpioh::PH4,
    gpioi::PI11,
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

pub struct USB1 {
    pub usb_global: stm32::OTG1_HS_GLOBAL,
    pub usb_device: stm32::OTG1_HS_DEVICE,
    pub usb_pwrclk: stm32::OTG1_HS_PWRCLK,
    pub prec: rcc::rec::Usb1Otg,
    pub hclk: Hertz,
}
impl USB1 {
    #[cfg(not(feature = "rm0455"))]
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
    #[cfg(feature = "rm0455")]
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

#[cfg(not(feature = "rm0455"))]
pub struct USB2 {
    pub usb_global: stm32::OTG2_HS_GLOBAL,
    pub usb_device: stm32::OTG2_HS_DEVICE,
    pub usb_pwrclk: stm32::OTG2_HS_PWRCLK,
    pub prec: rcc::rec::Usb2Otg,
    pub hclk: Hertz,
}
#[cfg(not(feature = "rm0455"))]
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

#[cfg(not(feature = "rm0455"))]
usb_peripheral! {
    USB2, OTG2_HS_GLOBAL, usb2otgen, usb2otgrst
}
#[cfg(not(feature = "rm0455"))]
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
    PI11(PI11<Alternate<AF10>>),
}

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
    pub fn new(
        usb_global: stm32::OTG1_HS_GLOBAL,
        usb_device: stm32::OTG1_HS_DEVICE,
        usb_pwrclk: stm32::OTG1_HS_PWRCLK,
        _ulpi_clk: PA5<Alternate<AF10>>,
        _ulpi_dir: impl Into<Usb1UlpiDirPin>,
        _ulpi_nxt: impl Into<Usb1UlpiNxtPin>,
        _ulpi_stp: PC0<Alternate<AF10>>,
        _ulpi_d0: PA3<Alternate<AF10>>,
        _ulpi_d1: PB0<Alternate<AF10>>,
        _ulpi_d2: PB1<Alternate<AF10>>,
        _ulpi_d3: PB10<Alternate<AF10>>,
        _ulpi_d4: PB11<Alternate<AF10>>,
        _ulpi_d5: PB12<Alternate<AF10>>,
        _ulpi_d6: PB13<Alternate<AF10>>,
        _ulpi_d7: PB5<Alternate<AF10>>,
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
