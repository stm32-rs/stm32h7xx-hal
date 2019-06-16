//! Prelude
pub use embedded_hal::prelude::*;

pub use crate::delay::DelayExt as _stm32h7xx_hal_delay_DelayExt;
pub use crate::flash::FlashExt as _stm32h7xx_hal_flash_FlashExt;
pub use crate::gpio::GpioExt as _stm32h7xx_hal_gpio_GpioExt;
pub use crate::pwr::PwrExt as _stm32h7xx_hal_pwr_PwrExt;
pub use crate::rcc::RccExt as _stm32h7xx_hal_rcc_RccExt;
pub use crate::serial::SerialExt as _stm32h7xx_hal_serial_SerialExt;
pub use crate::spi::SpiExt as _stm32h7xx_hal_spi_SpiExt;
pub use crate::time::U32Ext as _stm32h7xx_hal_time_U32Ext;
