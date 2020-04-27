# Changelog

## [Unreleased]

## [v0.5.0] 2020-04-27

* pac: Upgrade to stm32-rs v0.11.0
* pwr: Remove need for unsafe, add documentation describing Run* mode
* rcc: Add a safe interface for the user and other crates to access the RCC
* spi: 16-bit word support
* i2c: Remove reference to I2C1 in macro generic for all I2Cs
* Add HAL for Serial Audio Interface, PDM mode
* Implement default blocking write for serial

## [v0.4.0] 2020-03-20

* pac: Upgrade to stm32-rs v0.10.0
* rcc: Add iterative PLL configuration strategy
* rcc: Improve documentation
* pwr: configure core supply on parts with SMPS (dual core). Prevents
  lock-up without unsafe for certain hardware configurations
* i2c: wait for end of previous address phase before setting START
* i2c: return immediately when TXIS / RXNE is set
* timers: implement TIM3/4/5/6/7/8/12/13/14/15/16/17
* gpio: fix error in PF7 definition
* add MSRV 1.39.0

## [v0.3.0] 2019-12-27

* timer: add method to clear interrupt flag
* gpio: initialise gpio in the correct typestate
* i2c: flush the TXDR register when a NACK condition occurs
* Fix `clear_interrupt_pending_bit()`
* i2c: detect not-acknowledge error, clear errors when detected
* Run CI against stable since Rust 1.37.0
* Upgrade to stm32-rs v0.9.0 (including svd2rust v0.16)
* Started Changelog

[Unreleased]: https://github.com/stm32-rs/stm32h7xx-hal/compare/v0.5.0...HEAD
[v0.5.0]: https://github.com/stm32-rs/stm32h7xx-hal/compare/v0.4.0...v0.5.0
[v0.4.0]: https://github.com/stm32-rs/stm32h7xx-hal/compare/v0.3.0...v0.4.0
[v0.3.0]: https://github.com/stm32-rs/stm32h7xx-hal/compare/v0.2.1...v0.3.0
