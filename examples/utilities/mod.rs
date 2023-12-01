//! Utilities for examples

pub mod logger;
#[macro_use]
mod power;

#[cfg(feature = "fmc")]
pub mod mpu_config;

#[cfg(feature = "ltdc")]
pub mod display_target;
#[cfg(feature = "ltdc")]
pub mod write;
#[cfg(feature = "ltdc")]
pub mod display_primitives;