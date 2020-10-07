//! Prelude
pub use embedded_hal::prelude::*;

pub use crate::adc::AdcExt as _stm32h7xx_hal_adc_AdcExt;
pub use crate::dac::DacExt as _stm32h7xx_hal_dac_DacExt;
pub use crate::delay::DelayExt as _stm32h7xx_hal_delay_DelayExt;
pub use crate::exti::ExtiExt as _stm32h7xx_hal_delay_ExtiExt;
pub use crate::flash::FlashExt as _stm32h7xx_hal_flash_FlashExt;
#[cfg(feature = "fmc")]
pub use crate::fmc::FmcExt as _stm32h7xx_hal_fmc_FmcExt;
pub use crate::gpio::GpioExt as _stm32h7xx_hal_gpio_GpioExt;
pub use crate::i2c::I2cExt as _stm32h7xx_hal_i2c_I2cExt;
pub use crate::pwm::PwmExt as _stm32_hal_pwm_PwmExt;
pub use crate::pwr::PwrExt as _stm32h7xx_hal_pwr_PwrExt;
#[cfg(feature = "quadspi")]
pub use crate::qspi::QspiExt as _stm32h7xx_hal_qspi_QspiExt;
pub use crate::rcc::RccExt as _stm32h7xx_hal_rcc_RccExt;
pub use crate::rng::RngCore as _stm32h7xx_hal_rng_RngCore;
pub use crate::rng::RngExt as _stm32h7xx_hal_rng_RngExt;
pub use crate::sai::SaiPdmExt as _stm32h7xx_hal_spi_SaiPdmExt;
#[cfg(feature = "sdmmc")]
pub use crate::sdmmc::SdmmcExt as _stm32h7xx_hal_sdmmc_SdmmcExt;
pub use crate::serial::SerialExt as _stm32h7xx_hal_serial_SerialExt;
pub use crate::spi::SpiExt as _stm32h7xx_hal_spi_SpiExt;
pub use crate::timer::TimerExt as _stm32h7xx_hal_timer_TimerExt;
pub use embedded_time::duration::Extensions as _stm32h7xx_hal_time_DurationExtensions;
pub use embedded_time::rate::Extensions as _stm32h7xx_hal_time_RateExtensions;
