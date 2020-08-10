//! *See the examples folder for more usage examples*
//!
//! This Hardware Abstraction Layer (HAL) provides the following functionality:
//!
//! Setup and Configuration
//!
//! * [Power Configuration](crate::pwr)
//! * [Reset and Clock Control](crate::rcc)
//!
//! Digital IO
//!
//! * [General Purpose Input / Output](crate::gpio)
//! * [External interrupt controller](crate::exti)
//!
//! Analog IO
//!
//! * [Analog to Digital Converter (ADC)](crate::adc)
//! * [Digital to Analog Converter (DAC)](crate::dac)
//!
//! Digital Busses
//!
//! * [Inter Integrated Circuit (I2C)](crate::i2c)
//! * [Serial Peripheral Interface (SPI)](crate::spi)
//! * [Serial Data (USART/UART)](crate::serial)
//! * [Serial Audio Interface](crate::sai)
//! * [Quad SPI](crate::qspi) Feature gate `qspi`
//! * [Ethernet](crate::ethernet) Feature gate `ethernet`
//!
//! External Memory
//!
//! * [Flexible Memory Controller (FMC)](crate::fmc) Feature gate `fmc`
//! * [SD Card (SDMMC)](crate::sdmmc) Feature gate `sdmmc`
//!
//! Timing functions
//!
//! * [Pulse Width Modulation (PWM)](crate::pwm)
//! * [Quadrature Encoder Interface](crate::qei)
//! * [Timers](crate::timer)
//! * [Delays](crate::delay)
//!
//! Others
//!
//! * [Random Number Generator](crate::rng)
//! * [System Window Watchdog](crate::watchdog)

#![cfg_attr(not(test), no_std)]
#![allow(non_camel_case_types)]

extern crate paste;

#[derive(Debug)]
pub enum Never {}

#[cfg(not(feature = "device-selected"))]
compile_error!(
    "This crate requires one of the following device features enabled:
        stm32h742
        stm32h743
        stm32h753
        stm32h750
        stm32h742v
        stm32h743v
        stm32h753v
        stm32h750v
        stm32h747cm7
        stm32h757cm7
        stm32h7b3
        stm32h7b0
        stm32h7a3
"
);

pub use embedded_hal as hal;
pub mod traits;

pub use nb;
pub use nb::block;

// Single core
#[cfg(any(
    feature = "stm32h742",
    feature = "stm32h743",
    feature = "stm32h750",
))]
pub use stm32h7::stm32h743 as stm32;
#[cfg(any(
    feature = "stm32h742v",
    feature = "stm32h743v",
    feature = "stm32h750v",
))]
pub use stm32h7::stm32h743v as stm32;

// Single core with crypto
#[cfg(any(feature = "stm32h753",))]
pub use stm32h7::stm32h753 as stm32;
#[cfg(any(feature = "stm32h753v",))]
pub use stm32h7::stm32h753v as stm32;

// Dual core
#[cfg(any(feature = "stm32h747cm7",))]
pub use stm32h7::stm32h747cm7 as stm32;
#[cfg(any(feature = "stm32h757cm7",))]
pub use stm32h7::stm32h757cm7 as stm32;
// TODO(rm0399): soundness of PeripheralREC macro in rcc/rec.rs

// High Memory Integration
#[cfg(any(
    feature = "stm32h7b3",
    feature = "stm32h7a3",
    feature = "stm32h7b0",
))]
pub use stm32h7::stm32h7b3 as stm32;

#[cfg(all(feature = "rm0433", feature = "rm0399"))]
compile_error!("Cannot not select both rm0433 and rm0399");

#[cfg(all(feature = "cm7", feature = "cm4"))]
compile_error!("Cannot not select both CM7 and CM4");

#[cfg(feature = "device-selected")]
pub use crate::stm32 as pac;
#[cfg(feature = "device-selected")]
pub use crate::stm32 as device;

// Enable use of interrupt macro
#[cfg(feature = "rt")]
pub use crate::stm32::interrupt;

#[cfg(feature = "device-selected")]
pub mod adc;
#[cfg(feature = "device-selected")]
pub mod dac;
#[cfg(feature = "device-selected")]
pub mod delay;
#[cfg(feature = "device-selected")]
pub mod dma;
#[cfg(all(feature = "device-selected", feature = "ethernet"))]
pub mod ethernet;
#[cfg(feature = "device-selected")]
pub mod exti;
#[cfg(feature = "device-selected")]
pub mod flash;
#[cfg(all(feature = "device-selected", feature = "fmc"))]
pub mod fmc;
#[cfg(feature = "device-selected")]
pub mod gpio;
#[cfg(feature = "device-selected")]
pub mod i2c;
#[cfg(feature = "device-selected")]
pub mod prelude;
#[cfg(feature = "device-selected")]
pub mod pwm;
#[cfg(feature = "device-selected")]
pub mod pwr;
#[cfg(feature = "device-selected")]
pub mod qei;
#[cfg(all(
    feature = "device-selected",
    feature = "quadspi",
    not(feature = "rm0455")
))]
pub mod qspi;
#[cfg(feature = "device-selected")]
pub mod rcc;
#[cfg(feature = "device-selected")]
pub mod rng;
#[cfg(all(feature = "device-selected", feature = "rtc"))]
pub mod rtc;
#[cfg(feature = "device-selected")]
pub mod sai;
#[cfg(all(feature = "device-selected", feature = "sdmmc"))]
pub mod sdmmc;
#[cfg(feature = "device-selected")]
pub mod serial;
#[cfg(feature = "device-selected")]
pub mod signature;
#[cfg(feature = "device-selected")]
pub mod spi;
#[cfg(feature = "device-selected")]
pub mod time;
#[cfg(feature = "device-selected")]
pub mod timer;
#[cfg(all(feature = "device-selected", feature = "usb_hs"))]
pub mod usb_hs;
#[cfg(feature = "device-selected")]
pub mod watchdog;
