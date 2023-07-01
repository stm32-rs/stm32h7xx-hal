# Changelog

## Unreleased
* bors bot replaced with GH merge queue
* Update `smoltcp` dependency to `0.9.0`
* MSRV increased to 1.65.0
* add `IntoAf` trait to restrict `into_alternate` [#346]

## [v0.14.0] 2023-03-22

* **Breaking** flash: Rework API to support all parts and use
  [embedded-storage](https://github.com/rust-embedded-community/embedded-storage)
  traits [#415]
* **Breaking** sdmmc: Bump `embedded-sdmmc` to 0.4 [#421]
* gpio: Make `Alternate` variants readable [#409]
* lptim: Add LPTIM traits [#419]
* pll: Add bounds check on PLL R/Q dividers [#407]
* serial: Add support for half-duplex mode [#408]
* Add missing Defmt impls [#418]
* Bump `cortex-m` dependency to `0.7.7` and add `critical-section-single-core` feature [#426]
* Update `bitflags` dependency to `2.0.0` [#426]
* MSRV increased to 1.62.0

## [v0.13.1] 2022-11-06

* i2c: Wait for automatic stop condition to occour before returning [#402]
* MSRV increased to 1.61.0

## [v0.13.0] 2022-10-06

* **Breaking** Require frequency when initialising adc, use this to set the prescaler [#379]
* **Breaking** Remove `POL` and `NPOL` generic type parameters from `Pwm` types [#381]
* adc: Fix internal channel allocations for RM0468 parts [#379]
* adc: Remove unnecessary multiple reads of the CR register [#364]
* adc: Add DMA TargetAddress implementations [#389]
* adc: Poll LDORDY bit before ADC calibration [#386]
* flash: Rewrite `write_sector` to correctly write data in 256 bit chunks [#371]
* flash: Use Ordering fence to prevent inconsistency errors when writing to flash sector [#382]
* iwdt: Added HAL implementation. Changed the name of the other implementation to `system_watchdog` [#376]
* lptim: Fixes for low power times as pwm outputs [#385]
* pac: Upgrade to stm32-rs v0.15.1 [#370]
* octospi: Add support for octospi1 on rm0455 parts [#370]
* sdmmc: Use sdio-host v0.9, add support for eMMC Extended CSD register [#378]
* serial: Add support for uart9 and usart10 [#370]
* spi: Remove potential UB in transfer method [#390]
* rcc: Added a way to read the chip's reset reason [#377]

## [v0.12.2] 2022-05-06

* gpio: Fix `with_<mode>` methods [#359]
* sdmmc: Fix back-to-back writes [#361]
* flash: Expose `start_address`, `end_address` and `sector_address` methods [#357]

## [v0.12.1] 2022-04-19

* Expose `PinMode` trait [#354]

## [v0.12.0] 2022-04-16

* **Breaking**: Use const-generics for `GPIO`, add `DynamicPin`, `ErasedPin`,
  inherent impls for `embedded-hal` like methods by-default, default modes
  [#334]
* **Breaking**: Remove generic PullUp/PullDown/Floating parameter from
  `Input`. Use `internal_resistor()` method instead, add universal
  `into_mode::<MODE>()` pin mode converter, Rename `set_speed()` method to
  `speed()` `AFx` is now alias to `Alternate<x>` [#347]
* **Breaking**: Use [fugit](https://docs.rs/fugit/0.3.5/fugit/index.html) crate
  for duration and rate units. `.mhz()` -> `.MHz()`, `.khz()` -> `.kHz()`,
  `.hz()` -> `.Hz()`. Methods that previously accepted durations (`.ms()`,
  `.us()`) now need explicit conversion with
  [`.into_rate()`](https://docs.rs/fugit/0.3.5/fugit/struct.Duration.html#method.into_rate)
  [#321]
* MSRV increased to 1.59
* Replace crate::Never with core::convert::Infallible [#339]
* Added some [defmt](https://defmt.ferrous-systems.com/) support [#304]
* Add support for FDCAN peripheral using the
  [fdcan](https://github.com/stm32-rs/fdcan) crate [#277]
* Add `inner()` and `inner_mut()` methods to more peripherals [#337]
* pwr: Add backup domain voltage regulator control [#303]
* pwr: More support for VOS0, and support for VOS2 and VOS3 [#316] [#327]
* rcc: Add logging (`debug` level) behind `log` feature gate [#318]
* systick: Implement
  [`Countdown`](https://docs.rs/embedded-hal/latest/embedded_hal/timer/trait.CountDown.html)
  for SysTick [#310]
* adc: Add `slope` method and depreciate `max_sample` [#344]
* flash: added flash implementation [#322]
* timer: Implemented tick_timer method for LP Timers [#172]
* timer: `counter()` method on LP Timers returns u16 [#172]
* timer: Fix bug in `counter()` TIM1/8/12/13/14 where return value would also
  include UIF bit [#172]
* sdmmc: Update sdio-host to the latest version, add eMMC support from sdio-host [#341]
* serial: Add support for reconfiguring serial port settings [#324]
* **Note:** The serial port structs (`Serial`/`Rx`) are no longer zero-sized
* serial: Add support for swapping Tx/Rx pins [#324]
* serial: Add `join()` to combine `Tx`+`Rx` back into `Serial` [#324]
* serial: Add `inverttx` and `invertrx` configuration methods [#340]
* serial: Add serial Rx and Tx fifo capacity and Fifo interrupt event [#350]
* sai: Fix error in I2S `send` implementation when using both channels [#311]
* spi: add DuplexFailed error [#328]
* spi: Fix arithmetic error in spi divider calculation [#328]
* qspi/octospi: Add `begin_write_extended` and `begin_read_extended` methods [#345]
* qei: Include REC in the `free()` method [#325]

## [v0.11.0] 2021-12-18

* **Breaking**: Simplified API for reading device signature
  values. `VAL::get().read()` becomes `VAL::read()`
* **Breaking**: SAI `sai_[ab]_ker_ck` methods now return `Hertz` rather than
  `Option<Hertz>` and if the clock is stopped they panic with a message.
* **Breaking**: Rename the `quadspi` flag to `xspi`
* **Breaking**: serial: manual control over data length removed, now set automatically based on parity [#297]
* Upgrade to cortex-m-rt v0.7.0 [#257] [#264]
* **Breaking**: If you use RTIC v0.5, you need to depend on it with
  `default-features = false, features = ["cortex-m-7"]`. See
  [rtic-rs/cortex-m-rtic#509](https://github.com/rtic-rs/cortex-m-rtic/pull/509)
* Add support for OCTOSPI on supported parts, also using the `xspi` feature flag [#230]
* Add support for STM32H735 (feature flag `stm32h735` [#235]
* Rename the PeripheralREC object for BDMA2 on 7B3, 7B0, 7A3 parts from BDMA to BDMA2
* Fixed clippy lints:
   * Added safety docs for some DMA functions
   * Implemented additional conversion utilities for `time`
   * **Breaking**: Changed I2S constructors to take less arguments
* MSRV increased to 1.52.0
* Add "rt" to the default features [#287]
* adc: Allow parallel execution of multiple ADCs through `start_conversion()` [#250]
* dma: Add support for MDMA [#186]
* ethernet: `ethernet::DesRing` and `ethernet::EthernetDMA` require generic
  constants to specify how many transmit / receive buffers to include in
  `ethernet::DesRing`. To replicate the previous behaviour, use `DesRing<4, 4>` [#247]
* ethernet: Avoid creating intermediate references to packed fields [#225]
* ethernet: Upgrade smoltcp to v0.8.0 [#292]
* ethernet: Fix MAC address filtering [#249]
* gpio: Implement IoPin [#256]
* gpio: Add InputPin implementation for Alternate Pins [#244]
* i2c: Fix scldec/sdadel and adjust i2c divider calculations [#252]
* pac: Upgrade to stm32-rs v0.14.0 [#240]
* quadspi: Add support for flash memory
* rng: Implement rand_core::RngCore and support all integers in Rng [#284]
* sdmmc: Implement [embedded-sdmmc](https://github.com/rust-embedded-community/embedded-sdmmc-rs) traits [#262]
* spi: use the FIFO in Transfer and Write implementations [#269]
* serial: Improve sync mode support, add additional config options [#261] [#267]
* serial: Fix parity configuration option [#242]

## [v0.10.0] 2021-07-21

* **Breaking**: Don't reset peripheral in DMA1/2 `StreamsTuple::new()` method #229
* adc: Add `free()` method for ADC12 #213
* dma: Fast double-buffered DMA `next_dbm_transfer_with()` method #226
* ethernet: Fix feature set for smoltcp dependency #221
* pwm: Add advanced PWM functions #175
* rtc: Set LSEBYP before enabling LSE #211
* serial: implement `fmt::Write` for `Serial<USART` #214
* spi: Add more hardware CS features #216
* timers: Better calculations for `set_timeout_ticks` #208

## [v0.9.0] 2021-03-12

* Updates `cortex-m` to v0.7.1. `cortex-m` v0.6.5+ [are forward compatible with
  v0.7.0+][cm6-changelog] except for CBP, ITM, MPU, NVIC, SCB. If you have
  problems, run `cargo update` to try to switch to `cortex-m` v0.7 in other
  dependencies. Something like `cargo tree` is very useful to track down
  remaining uses of `cortex-m` v0.6.
* **Breaking**: Add new/new_unchecked methods for USB structures, remove pin
  types from structure
* Add support for USB1_ULPI #184
* Add support for LTDC #81
* Add support for CRC #199
* Add support for DMA1, DMA2, BDMA #153
* pac: Upgrade to stm32-rs v0.13.0
* spi: Update CS management #159
* spi: Support 32-bit frames #200
* qspi: Sample on falling edges #161
* qspi: Add configuration support #176
* timers: Add `DelayFromCountDownTimer` #170
* ethernet: update smoltcp v0.6 -> v0.7
* usb: add support for external ULPI PHYs #184
* rtc: fix unlisten method
* serial: add clear_idle and is_busy methods #201

[cm6-changelog]: (https://github.com/rust-embedded/cortex-m/blob/master/CHANGELOG.md#v065---2021-01-24)

## [v0.8.0] 2020-10-26

* **Breaking**: Ethernet PHY configuration feature flags removed. The user must
  now instantiate an instance of the PHY type in order to configure the PHY.
* pac: Upgrade to stm32-rs v0.12.0
* devices: Add support for 7B3, 7B0, 7A3
* Add USB support
* Add I2S support
* Add RTC support
* Add LPTIM support
* Add DMA support, but the current API is depreciated and will be replaced
* timer: add tick_timer and set_tick_freq to configure the timer's counter frequency [#144](https://github.com/stm32-rs/stm32h7xx-hal/pull/144)
* Add RTC support [#143](https://github.com/stm32-rs/stm32h7xx-hal/pull/143)
* pwr: add PowerConfiguration to ensure VoltageScale isn't modified from pwr.freeze() to rcc.freeze() [#141](https://github.com/stm32-rs/stm32h7xx-hal/pull/141)
* impl Copy, Clone, PartialEq for enums [#139](https://github.com/stm32-rs/stm32h7xx-hal/pull/139)
* ethernet: automatically configure MDC clock based on hclk
* time: add types for microseconds and nanoseconds
* rec: add `low_power` methods for configuring peripherals

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

[Unreleased]: https://github.com/stm32-rs/stm32h7xx-hal/compare/v0.14.0...HEAD
[v0.14.0]: https://github.com/stm32-rs/stm32h7xx-hal/compare/v0.13.1...v0.14.0
[v0.13.1]: https://github.com/stm32-rs/stm32h7xx-hal/compare/v0.13.0...v0.13.1
[v0.13.0]: https://github.com/stm32-rs/stm32h7xx-hal/compare/v0.12.2...v0.13.0
[v0.12.2]: https://github.com/stm32-rs/stm32h7xx-hal/compare/v0.12.1...v0.12.2
[v0.12.1]: https://github.com/stm32-rs/stm32h7xx-hal/compare/v0.12.0...v0.12.1
[v0.12.0]: https://github.com/stm32-rs/stm32h7xx-hal/compare/v0.11.0...v0.12.0
[v0.11.0]: https://github.com/stm32-rs/stm32h7xx-hal/compare/v0.10.0...v0.11.0
[v0.10.0]: https://github.com/stm32-rs/stm32h7xx-hal/compare/v0.9.0...v0.10.0
[v0.9.0]: https://github.com/stm32-rs/stm32h7xx-hal/compare/v0.8.0...v0.9.0
[v0.8.0]: https://github.com/stm32-rs/stm32h7xx-hal/compare/v0.7.1...v0.8.0
[v0.7.1]: https://github.com/stm32-rs/stm32h7xx-hal/compare/v0.7.0...v0.7.1
[v0.7.0]: https://github.com/stm32-rs/stm32h7xx-hal/compare/v0.6.0...v0.7.0
[v0.6.0]: https://github.com/stm32-rs/stm32h7xx-hal/compare/v0.5.0...v0.6.0
[v0.5.0]: https://github.com/stm32-rs/stm32h7xx-hal/compare/v0.4.0...v0.5.0
[v0.4.0]: https://github.com/stm32-rs/stm32h7xx-hal/compare/v0.3.0...v0.4.0
[v0.3.0]: https://github.com/stm32-rs/stm32h7xx-hal/compare/v0.2.1...v0.3.0

[#172]: https://github.com/stm32-rs/stm32h7xx-hal/pull/172
[#186]: https://github.com/stm32-rs/stm32h7xx-hal/pull/186
[#225]: https://github.com/stm32-rs/stm32h7xx-hal/pull/225
[#230]: https://github.com/stm32-rs/stm32h7xx-hal/pull/230
[#235]: https://github.com/stm32-rs/stm32h7xx-hal/pull/235
[#240]: https://github.com/stm32-rs/stm32h7xx-hal/pull/240
[#242]: https://github.com/stm32-rs/stm32h7xx-hal/pull/242
[#244]: https://github.com/stm32-rs/stm32h7xx-hal/pull/244
[#247]: https://github.com/stm32-rs/stm32h7xx-hal/pull/247
[#249]: https://github.com/stm32-rs/stm32h7xx-hal/pull/249
[#250]: https://github.com/stm32-rs/stm32h7xx-hal/pull/250
[#252]: https://github.com/stm32-rs/stm32h7xx-hal/pull/252
[#256]: https://github.com/stm32-rs/stm32h7xx-hal/pull/256
[#257]: https://github.com/stm32-rs/stm32h7xx-hal/pull/257
[#261]: https://github.com/stm32-rs/stm32h7xx-hal/pull/261
[#262]: https://github.com/stm32-rs/stm32h7xx-hal/pull/262
[#264]: https://github.com/stm32-rs/stm32h7xx-hal/pull/264
[#267]: https://github.com/stm32-rs/stm32h7xx-hal/pull/267
[#269]: https://github.com/stm32-rs/stm32h7xx-hal/pull/269
[#277]: https://github.com/stm32-rs/stm32h7xx-hal/pull/277
[#284]: https://github.com/stm32-rs/stm32h7xx-hal/pull/284
[#287]: https://github.com/stm32-rs/stm32h7xx-hal/pull/287
[#292]: https://github.com/stm32-rs/stm32h7xx-hal/pull/292
[#297]: https://github.com/stm32-rs/stm32h7xx-hal/pull/297
[#303]: https://github.com/stm32-rs/stm32h7xx-hal/pull/303
[#304]: https://github.com/stm32-rs/stm32h7xx-hal/pull/304
[#310]: https://github.com/stm32-rs/stm32h7xx-hal/pull/310
[#311]: https://github.com/stm32-rs/stm32h7xx-hal/pull/311
[#316]: https://github.com/stm32-rs/stm32h7xx-hal/pull/316
[#318]: https://github.com/stm32-rs/stm32h7xx-hal/pull/318
[#321]: https://github.com/stm32-rs/stm32h7xx-hal/pull/321
[#322]: https://github.com/stm32-rs/stm32h7xx-hal/pull/322
[#324]: https://github.com/stm32-rs/stm32h7xx-hal/pull/324
[#325]: https://github.com/stm32-rs/stm32h7xx-hal/pull/325
[#327]: https://github.com/stm32-rs/stm32h7xx-hal/pull/327
[#328]: https://github.com/stm32-rs/stm32h7xx-hal/pull/328
[#334]: https://github.com/stm32-rs/stm32h7xx-hal/pull/334
[#337]: https://github.com/stm32-rs/stm32h7xx-hal/pull/337
[#339]: https://github.com/stm32-rs/stm32h7xx-hal/pull/339
[#340]: https://github.com/stm32-rs/stm32h7xx-hal/pull/340
[#341]: https://github.com/stm32-rs/stm32h7xx-hal/pull/341
[#344]: https://github.com/stm32-rs/stm32h7xx-hal/pull/344
[#345]: https://github.com/stm32-rs/stm32h7xx-hal/pull/345
[#346]: https://github.com/stm32-rs/stm32h7xx-hal/pull/346
[#347]: https://github.com/stm32-rs/stm32h7xx-hal/pull/347
[#350]: https://github.com/stm32-rs/stm32h7xx-hal/pull/350
[#354]: https://github.com/stm32-rs/stm32h7xx-hal/pull/354
[#357]: https://github.com/stm32-rs/stm32h7xx-hal/pull/357
[#359]: https://github.com/stm32-rs/stm32h7xx-hal/pull/359
[#361]: https://github.com/stm32-rs/stm32h7xx-hal/pull/361
[#364]: https://github.com/stm32-rs/stm32h7xx-hal/pull/364
[#370]: https://github.com/stm32-rs/stm32h7xx-hal/pull/370
[#371]: https://github.com/stm32-rs/stm32h7xx-hal/pull/371
[#376]: https://github.com/stm32-rs/stm32h7xx-hal/pull/376
[#377]: https://github.com/stm32-rs/stm32h7xx-hal/pull/377
[#378]: https://github.com/stm32-rs/stm32h7xx-hal/pull/378
[#379]: https://github.com/stm32-rs/stm32h7xx-hal/pull/379
[#381]: https://github.com/stm32-rs/stm32h7xx-hal/pull/381
[#382]: https://github.com/stm32-rs/stm32h7xx-hal/pull/382
[#385]: https://github.com/stm32-rs/stm32h7xx-hal/pull/385
[#386]: https://github.com/stm32-rs/stm32h7xx-hal/pull/386
[#389]: https://github.com/stm32-rs/stm32h7xx-hal/pull/389
[#390]: https://github.com/stm32-rs/stm32h7xx-hal/pull/390
[#402]: https://github.com/stm32-rs/stm32h7xx-hal/pull/402
[#407]: https://github.com/stm32-rs/stm32h7xx-hal/pull/407
[#408]: https://github.com/stm32-rs/stm32h7xx-hal/pull/408
[#409]: https://github.com/stm32-rs/stm32h7xx-hal/pull/409
[#415]: https://github.com/stm32-rs/stm32h7xx-hal/pull/415
[#418]: https://github.com/stm32-rs/stm32h7xx-hal/pull/418
[#419]: https://github.com/stm32-rs/stm32h7xx-hal/pull/419
[#421]: https://github.com/stm32-rs/stm32h7xx-hal/pull/421
[#426]: https://github.com/stm32-rs/stm32h7xx-hal/pull/426
