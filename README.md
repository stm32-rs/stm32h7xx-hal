# [Documentation](https://docs.rs/stm32h7xx-hal)

stm32h7xx-hal
=============

[![docs.rs](https://docs.rs/stm32h7xx-hal/badge.svg)](https://docs.rs/stm32h7xx-hal)
[![Bors enabled](https://bors.tech/images/badge_small.svg)](https://app.bors.tech/repositories/12691)
[![Travis](https://travis-ci.com/stm32-rs/stm32h7xx-hal.svg?branch=master)](https://travis-ci.com/stm32-rs/stm32h7xx-hal)
[![Crates.io](https://img.shields.io/crates/v/stm32h7xx-hal.svg)](https://crates.io/crates/stm32h7xx-hal)

[_stm32h7xx-hal_](https://github.com/stm32-rs/stm32h7xx-hal) contains
a hardware abstraction layer on top of the peripheral access API for
the STMicro STM32H7xx family of microcontrollers. The idea behind this
crate is to gloss over the slight differences in the various
peripherals available on those MCUs so a HAL can be written for all
chips in that same family without having to cut and paste crates for
every single model.

This crate relies on Adam Greig's fantastic [stm32h7][] crate to provide
appropriate register definitions, and implements a partial set of the
[embedded-hal][] traits. Much of the implementation was adapted from
other HAL crates in the [stm32-rs organisation][stm32-rs].

Collaboration on this crate is highly welcome, as are pull requests!


Supported Configurations
------------------------

* __stm32h743v__ (Revision V: stm32h743, stm32h742, stm32h750)
* __stm32h753v__
* __stm32h743__ (Revision Y: stm32h743, stm32h742, stm32h750)
* __stm32h753__
* __stm32h747cm7__ (stm32h747, stm32h757)


#### Single core parts (Cortex M7)
In 2019 ST released hardware Revision V of the stm32h742, stm32h743,
stm32h750 and stm32h753 ([eevblog][]). This hardware revision makes
breaking hardware changes, documented in [AN5312][]. If you have a
device purchased since mid-2019, you likely want to use the feature
gate ending in a __v__.

#### Dual core parts (Cortex M7 + Cortex M4)
On dual core parts, currently only the Cortex M7 core is supported.

Getting Started
---------------

The `examples` folder contains several example programs. To compile
them, one must specify the target device as cargo feature:
```
$ cargo build --features=stm32h743v,rt
```

To use stm32h7xx-hal as a dependency in a standalone project the
target device feature must be specified in the `Cargo.toml` file:
```
[dependencies]
cortex-m = "0.6.0"
cortex-m-rt = "0.6.10"
stm32h7xx-hal = {version = "0.3", features = ["stm32h743v","rt"]}
```

If you are unfamiliar with embedded development using Rust, there are
a number of fantastic resources available to help.

- [Embedded Rust Documentation](https://docs.rust-embedded.org/)
- [The Embedded Rust Book](https://docs.rust-embedded.org/book/)
- [Rust Embedded FAQ](https://docs.rust-embedded.org/faq.html)
- [rust-embedded/awesome-embedded-rust](https://github.com/rust-embedded/awesome-embedded-rust)


Minimum supported Rust version
------------------------------

The minimum supported Rust version at the moment is **1.39.0**. Older
versions **may** compile, especially when some features are not used
in your application.

Changelog
---------

See [CHANGELOG.md](CHANGELOG.md).


License
-------

0-Clause BSD License, see [LICENSE-0BSD.txt](LICENSE-0BSD.txt) for more details.

[stm32h7]: https://crates.io/crates/stm32h7
[stm32-rs]: https://github.com/stm32-rs
[embedded-hal]: https://github.com/rust-embedded/embedded-hal
[AN5312]: https://www.st.com/resource/en/application_note/dm00609692.pdf
[eevblog]: https://www.eevblog.com/forum/microcontrollers/stm32h7-series-revision-beware-of-the-changes!/
