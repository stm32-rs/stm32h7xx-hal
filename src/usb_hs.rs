//! USB OTG peripherals
//!
//! Requires the `usb_hs` feature.
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

use crate::gpio::{self, Alternate, Speed};

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
        _pin_dm: gpio::PB14<Alternate<12>>,
        _pin_dp: gpio::PB15<Alternate<12>>,
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
        _pin_dm: gpio::PA11<Alternate<10>>,
        _pin_dp: gpio::PA12<Alternate<10>>,
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
        _pin_dm: gpio::PA11<Alternate<10>>,
        _pin_dp: gpio::PA12<Alternate<10>>,
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
    PC2(gpio::PC2<Alternate<10>>),
    #[cfg(not(feature = "rm0468"))]
    PI11(gpio::PI11<Alternate<10>>),
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
impl From<gpio::PI11<Alternate<10>>> for Usb1UlpiDirPin {
    fn from(v: gpio::PI11<Alternate<10>>) -> Self {
        Usb1UlpiDirPin::PI11(v)
    }
}

impl From<gpio::PC2<Alternate<10>>> for Usb1UlpiDirPin {
    fn from(v: gpio::PC2<Alternate<10>>) -> Self {
        Usb1UlpiDirPin::PC2(v)
    }
}

pub enum Usb1UlpiNxtPin {
    PC3(gpio::PC3<Alternate<10>>),
    PH4(gpio::PH4<Alternate<10>>),
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

impl From<gpio::PH4<Alternate<10>>> for Usb1UlpiNxtPin {
    fn from(v: gpio::PH4<Alternate<10>>) -> Self {
        Usb1UlpiNxtPin::PH4(v)
    }
}

impl From<gpio::PC3<Alternate<10>>> for Usb1UlpiNxtPin {
    fn from(v: gpio::PC3<Alternate<10>>) -> Self {
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
        ulpi_clk: gpio::PA5<Alternate<10>>,
        ulpi_dir: impl Into<Usb1UlpiDirPin>,
        ulpi_nxt: impl Into<Usb1UlpiNxtPin>,
        ulpi_stp: gpio::PC0<Alternate<10>>,
        ulpi_d0: gpio::PA3<Alternate<10>>,
        ulpi_d1: gpio::PB0<Alternate<10>>,
        ulpi_d2: gpio::PB1<Alternate<10>>,
        ulpi_d3: gpio::PB10<Alternate<10>>,
        ulpi_d4: gpio::PB11<Alternate<10>>,
        ulpi_d5: gpio::PB12<Alternate<10>>,
        ulpi_d6: gpio::PB13<Alternate<10>>,
        ulpi_d7: gpio::PB5<Alternate<10>>,
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
