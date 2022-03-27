//! Prelude
pub use embedded_hal::prelude::*;

pub use crate::adc::AdcExt as _stm32h7xx_hal_adc_AdcExt;
#[cfg(feature = "can")]
pub use crate::can::CanExt as _stm32h7xx_hal_can_CanExt;
#[cfg(feature = "crc")]
pub use crate::crc::CrcExt as _stm32h7xx_hal_crc_CrcExt;
pub use crate::dac::DacExt as _stm32h7xx_hal_dac_DacExt;
pub use crate::delay::DelayExt as _stm32h7xx_hal_delay_DelayExt;
pub use crate::exti::ExtiExt as _stm32h7xx_hal_delay_ExtiExt;
pub use crate::flash::FlashExt as _stm32h7xx_hal_flash_FlashExt;
#[cfg(feature = "fmc")]
pub use crate::fmc::FmcExt as _stm32h7xx_hal_fmc_FmcExt;
pub use crate::gpio::GpioExt as _stm32h7xx_hal_gpio_GpioExt;
pub use crate::i2c::I2cExt as _stm32h7xx_hal_i2c_I2cExt;
pub use crate::pwm::PwmAdvExt as _stm32_hal_pwm_PwmAdvExt;
pub use crate::pwm::PwmExt as _stm32_hal_pwm_PwmExt;
pub use crate::pwr::PwrExt as _stm32h7xx_hal_pwr_PwrExt;
pub use crate::rcc::RccExt as _stm32h7xx_hal_rcc_RccExt;
pub use crate::rng::RngCore as _stm32h7xx_hal_rng_RngCore;
pub use crate::rng::RngExt as _stm32h7xx_hal_rng_RngExt;
pub use crate::sai::SaiDmaExt as _stm32h7xx_hal_spi_SaiDmaExt;
pub use crate::sai::SaiI2sExt as _stm32h7xx_hal_spi_SaiI2sExt;
pub use crate::sai::SaiPdmExt as _stm32h7xx_hal_spi_SaiPdmExt;
#[cfg(feature = "sdmmc")]
pub use crate::sdmmc::SdmmcExt as _stm32h7xx_hal_sdmmc_SdmmcExt;
pub use crate::serial::SerialExt as _stm32h7xx_hal_serial_SerialExt;
pub use crate::spi::HalDisabledSpi as _stm32h7xx_hal_spi_HalDisabledSpi;
pub use crate::spi::HalEnabledSpi as _stm32h7xx_hal_spi_HalEnabledSpi;
pub use crate::spi::HalSpi as _stm32h7xx_hal_spi_HalSpi;
pub use crate::spi::SpiExt as _stm32h7xx_hal_spi_SpiExt;
pub use crate::time::U32Ext as _stm32h7xx_hal_time_U32Ext;
pub use crate::timer::TimerExt as _stm32h7xx_hal_timer_TimerExt;
#[cfg(all(feature = "xspi"))]
pub use crate::xspi::XspiExt as _stm32h7xx_hal_xspi_XspiExt;
