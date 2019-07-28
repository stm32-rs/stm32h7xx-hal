# stm32h7xx-hal

**This crate is highly incomplete and it is likely that you will need to add to
it to use most peripherals. Collaboration on this crate is highly welcome as
are pull requests!**

🚧 *Work in progress*

❗ *Beta or nightly required*

`stm32h7xx-hal` contains a hardware abstraction on top of the
peripheral access API for the STMicro STM32H7 series
microcontrollers. The selection of the MCU is done by feature gates,
typically specified by board support crates.

The idea behind this crate is to gloss over the slight differences in the
various peripherals available on those MCUs so a HAL can be written for all
chips in that same family without having to cut and paste crates for every
single model.

This crate relies on Adam Greigs fantastic [`stm32h7`][] crate to provide
appropriate register definitions and implements a partial set of the
[`embedded-hal`][] traits.

Almost all of the implementation was shamelessly adapted from the
[`stm32f30x-hal`][] crate by Jorge Aparicio.

Single Core Parts
--------

The currently supported parts are:

*   stm32h743 ✔️
*   stm32h753 ✔️

Feature gates for the stm32h742, stm32h750 also exist but may not be
complete.

In 2019 ST released hardware Revision V of the stm32h742, stm32h743,
stm32h750 and stm32h753 ([eevblog][]). This hardware revision makes
breaking hardware changes, documented in [AN5312][]. These parts are
supported with the following feature gates:

*   stm32h743v ✔️
*   stm32h753v ✔️

Again, feature gates stm32h742v, stm32h750v also exist.

Dual Core Parts
--------

Currently only the Cortex-M7 core is supported.

*   stm32h747cm7 ✔️
*   stm32h757cm7 ✔️

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
[`stm32f30x-hal`]: https://github.com/japaric/stm32f30x-hal
[`embedded-hal`]: https://github.com/japaric/embedded-hal
[AN5312]: https://www.st.com/resource/en/application_note/dm00609692.pdf
[eevblog]: https://www.eevblog.com/forum/microcontrollers/stm32h7-series-revision-beware-of-the-changes!/
