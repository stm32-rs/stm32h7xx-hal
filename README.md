[![docs](https://docs.rs/stm32h7xx-hal/badge.svg)](https://docs.rs/stm32h7xx-hal)
[![Bors enabled](https://bors.tech/images/badge_small.svg)](https://app.bors.tech/repositories/12691)
[![Build status](https://travis-ci.com/stm32-rs/stm32h7xx-hal.svg?branch=master)](https://travis-ci.com/stm32-rs/stm32h7xx-hal)

# [Documentation](https://docs.rs/stm32h7xx-hal)

# stm32h7xx-hal

**This crate has support for most commonly used peripherals, but is
not complete.  Collaboration on this crate is highly welcome as are
pull requests!**

`stm32h7xx-hal` contains a hardware abstraction on top of the
peripheral access API for the STMicro STM32H7 series
microcontrollers. The selection of the MCU is done by feature gates,
typically specified by board support crates.

The currently supported feature gates are:

*   `stm32h743` ✔️
*   `stm32h753` ✔️

Feature gates for the `stm32h742`, `stm32h750` also exist but may not be
complete.

In 2019 ST released hardware Revision V of the STM32H742, STM32H743,
STM32H750 and STM32H753 ([eevblog][]). This hardware revision makes
breaking hardware changes, documented in [AN5312][]. These parts are
supported with the following feature gates:

*   `stm32h743v` ✔️
*   `stm32h753v` ✔️

Again, feature gates `stm32h742v`, `stm32h750v` also exist.

There is also support for dual core parts. Currently only the
Cortex-M7 core is supported.

*   `stm32h747cm7` ✔️
*   `stm32h757cm7` ✔️

The idea behind this crate is to gloss over the slight differences in
the various peripherals available on those MCUs so a HAL can be
written for all chips in that same family without having to cut and
paste crates for every single model.

This crate relies on Adam Greig's fantastic [`stm32h7`][] Peripheral
Access Crate (PAC) to provide appropriate register definitions. This
crate implements a partial set of the [`embedded-hal`][] traits.

Much of the implementation was adapted from other HAL crates in the
[stm32-rs organisation][stm32-rs].

Dependencies
--------

1. Rustup toolchain installer

    https://rustup.rs


Configure Toolchain
--------

`$ rustup target add thumbv7em-none-eabihf`

Build Examples
--------

You will need to change `stm32h743` to match your hardware.

`$ cargo build --release --examples --features stm32h743,rt`

Run an Example
--------

`$ cargo run --release --features stm32h743,rt --example blinky`

This will start `arm-none-eabi-gdb`.

License
--------

[0-clause BSD license](LICENSE-0BSD.txt).

[`stm32h7`]: https://crates.io/crates/stm32h7
[stm32-rs]: https://github.com/stm32-rs
[`embedded-hal`]: https://github.com/rust-embedded/embedded-hal
[AN5312]: https://www.st.com/resource/en/application_note/dm00609692.pdf
[eevblog]: https://www.eevblog.com/forum/microcontrollers/stm32h7-series-revision-beware-of-the-changes!/
