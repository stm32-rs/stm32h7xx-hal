# Changelog

## [Unreleased]

## [v0.7.1] 2020-09-04

* Update docs

## [v0.7.0] 2020-09-03

* **Breaking**: Fix PWM pin types. See #110
* Add QSPI support
* Add SDMMC support
* Add Ethernet support
* Add FMC support
* spi: add new configuration options
* i2c: fix i2c bus clock frequency calculation
* i2c: extend write duration until transaction finishes
* timer: Fix timer first cycle invalid #72
* timer: Add set_timeout method
* adc: add method to initialise ADC1 and ADC2 together, see examples
* Added method to switch to VOS0 (480MHz)
* Allow external HSE clock input (bypass mode)
* Use ACLK (AXI clock) frequency for calculating flash waitstates
* pll: Add fractional strategies
* pll: fix very subtle error in PLL Q,R frequencies
* serial: rename usart method to serial, will be depreciated in future
* examples: Added logging framework
* MSRV increased to 1.43.0

## [v0.6.0] 2020-06-25

* **Breaking:** Peripheral driver constructors now consume a peripheralREC
  singleton. Previously they inconsistently took `&mut` or `&` references
  to the CCDR struct itself. See the examples for the new constructors
* **Breaking:** Erase pins from peripheral driver types. `free()` method return
  types may have changed.
* Expose `new_unchecked` methods for instantiating peripheral drivers without
  providing pins
* Improved top-level docs
* Add DAC peripheral driver
* Added MCO pins
* Add `.free()` method for ADC
* MSRV increased to 1.41.0
* rtfm crate was renamed to rtic
* Added alternate flash size definitions to memory.x
* Bump dependency versions: cortex-m, cortex-m-rt, cast, paste, bare-metal

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

[Unreleased]: https://github.com/stm32-rs/stm32h7xx-hal/compare/v0.7.1...HEAD
[v0.7.0]: https://github.com/stm32-rs/stm32h7xx-hal/compare/v0.7.0...v0.7.1
[v0.7.0]: https://github.com/stm32-rs/stm32h7xx-hal/compare/v0.6.0...v0.7.0
[v0.6.0]: https://github.com/stm32-rs/stm32h7xx-hal/compare/v0.5.0...v0.6.0
[v0.5.0]: https://github.com/stm32-rs/stm32h7xx-hal/compare/v0.4.0...v0.5.0
[v0.4.0]: https://github.com/stm32-rs/stm32h7xx-hal/compare/v0.3.0...v0.4.0
[v0.3.0]: https://github.com/stm32-rs/stm32h7xx-hal/compare/v0.2.1...v0.3.0
