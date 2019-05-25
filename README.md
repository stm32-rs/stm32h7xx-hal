# stm32h7xx-hal

**This crate is highly incomplete and it is likely that you will need to add to
it to use most peripherals. Collaboration on this crate is highly welcome as
are pull requests!**

`stm32h7xx-hal` contains a hardware abstraction on top of the peripheral access
API for the STMicro STM32H7 series microcontrollers. The selection of the MCU
is done by feature gates, typically specified by board support crates.
Currently supported configurations are:

*   stm32h743 ✔️ YES!

While stm32h742, stm32h750, and stm32h753 remain unsupported.

The idea behind this crate is to gloss over the slight differences in the
various peripherals available on those MCUs so a HAL can be written for all
chips in that same family without having to cut and paste crates for every
single model.

This crate relies on Adam Greigs fantastic [`stm32h7`][] crate to provide
appropriate register definitions and implements a partial set of the
[`embedded-hal`][] traits.

Almost all of the implementation was shamelessly adapted from the
[`stm32f30x-hal`][] crate by Jorge Aparicio.

[`stm32h7`]: https://crates.io/crates/stm32h7
[`stm32f30x-hal`]: https://github.com/japaric/stm32f30x-hal
[`embedded-hal`]: https://github.com/japaric/embedded-hal

## License

[0-clause BSD license](LICENSE-0BSD.txt).
