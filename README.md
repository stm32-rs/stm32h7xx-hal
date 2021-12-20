# [Documentation](https://docs.rs/stm32h7xx-hal)

stm32h7xx-hal
=============

[![docs.rs](https://docs.rs/stm32h7xx-hal/badge.svg)](https://docs.rs/stm32h7xx-hal)
[![Bors enabled](https://bors.tech/images/badge_small.svg)](https://app.bors.tech/repositories/12691)
[![CI](https://github.com/stm32-rs/stm32h7xx-hal/workflows/Continuous%20integration/badge.svg)](https://github.com/stm32-rs/stm32h7xx-hal/actions)
[![Crates.io](https://img.shields.io/crates/v/stm32h7xx-hal.svg)](https://crates.io/crates/stm32h7xx-hal)
![Minimum rustc version](https://img.shields.io/badge/rustc-1.52.0+-yellow.svg)

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

* __stm32h743v__ (Revision V: stm32h742, stm32h743, stm32h750)
* __stm32h753v__ (Revision V: stm32h753
* __stm32h747cm7__ (stm32h745, stm32h747, stm32h755, stm32h757)
* __stm32h7a3__
* __stm32h7b0__
* __stm32h7b3__
* __stm32h735__ (stm32h723, stm32h725, stm32h730, stm32h733, stm32h735)

#### Old revision STM32H742/743/750/753 parts

In 2019 ST released hardware Revision V of the stm32h742, stm32h743, stm32h750
and stm32h753 ([eevblog][]). This hardware revision makes breaking hardware
changes, documented in [AN5312][]. If you have a device purchased before
mid-2019, check the revision code on your device. Parts from the previous
revision (Revision Y) are supported by feature gates without the 'v'
suffix. (__stm32h743__, __stm32h753__)

#### Dual core parts (Cortex M7 + Cortex M4)

On dual core parts, currently only the Cortex M7 core is supported.

#### Family-specific memory.x

Each H7 device family has a somewhat different memory layout. To make starting
a project easier, a specific example memory.x file is included for each family.
You can use these to replace the memory.x included in this crate for the examples.

The memory layout for each device family is specified in their respective
reference manual (RM). The table below relates the various parts to their
applicable memory.x and reference manual. 

RM     | memory.x                     | Applicable devices
-------|------------------------------|----------------------------------------
RM0399 | memory_745_747_755_757.x     | stm32h745, stm32h747, stm32h755, stm32h757
RM0433 | memory_742.x                 | stm32h742
RM0433 | memory_743_750_753.x         | stm32h743, stm32h750, stm32h753
RM0455 | memory_7A3_7B0_7B3.x         | stm32h7a3, stm32h7b0, stm32h7b3
RM0468 | memory_723_725_730_733_735.x | stm32h723, stm32h725, stm32h730, stm32h733, stm32h735

To use these files, substitute memory.x by the applicable one and update the
flash memory size as indicated below.

⚠️: If you use [flip-link](https://github.com/knurling-rs/flip-link) for stack
overflow protection, there is one more change to make. Flip-link does not (yet)
support the use of region aliases and expects an entry called RAM in the MEMORY
block. The work-around is to comment-out the "REGION_ALIAS(RAM, DTCM)" line and
manually substitute the RAM label in the respective MEMORY entry. Eg: replace
DTCM by RAM.

#### Flash memory size

By default this crate assumes a 2Mbyte flash size. To set a smaller limit for
linker errors, uncomment the correct `FLASH` section definition in memory.x

Getting Started
---------------

The [examples folder](examples/) contains several example programs. To compile
them, specify the target device in a cargo feature:

```
$ cargo build --features=stm32h743v,rt
```

See the [Examples README](examples/README.md) for more details.

To use stm32h7xx-hal as a dependency in a standalone project the
target device feature must be specified in the `Cargo.toml` file:
```toml
[dependencies]
cortex-m = "0.7.1"
cortex-m-rt = "0.6.12"
stm32h7xx-hal = {version = "0.10.0", features = ["stm32h743v","rt"]}
```

If you are unfamiliar with embedded development using Rust, there are
a number of fantastic resources available to help.

- [Embedded Rust Documentation](https://docs.rust-embedded.org/)
- [The Embedded Rust Book](https://docs.rust-embedded.org/book/)
- [Rust Embedded FAQ](https://docs.rust-embedded.org/faq.html)
- [rust-embedded/awesome-embedded-rust](https://github.com/rust-embedded/awesome-embedded-rust)

Hardware
--------

Below is a short list of publicly available and documented STM32H7
development boards. Note that including them on this list does not
mean they have been successfully tested with this crate. Some boards
have a Board Support Crate (BSP) offering pin mappings and additional
functionality.

Board | Manufacturer | BSP / Examples?
---|---|---
[NUCLEO-H743ZI](https://www.st.com/en/evaluation-tools/nucleo-h743zi.html) | ST | [Examples](https://github.com/astraw/nucleo-h743zi)
[NUCLEO-H745ZI-Q](https://www.st.com/en/evaluation-tools/nucleo-h745zi-q.html) | ST | [BSP](https://github.com/antoinevg/nucleo-h745zi)
[STM32H743I-EVAL](https://www.st.com/en/evaluation-tools/stm32h743i-eval.html) | ST |
[STM32H747I-EVAL](https://www.st.com/en/evaluation-tools/stm32h747i-eval.html) | ST |
[STM32H747I-DISCO](https://www.st.com/en/evaluation-tools/stm32h747i-disco.html) | ST |
[Daisy Seed](https://www.electro-smith.com/daisy/daisy) | Electrosmith | [BSP](https://github.com/antoinevg/daisy_bsp)
[Portenta H7](https://store.arduino.cc/portenta-h7) ⚠️ | Arduino |
[OpenH743I-C](https://www.waveshare.com/openh743i-c-standard.htm) | Waveshare |
[Toasty](https://www.tindie.com/products/webtronics/toasty-480mhz-stm32-usb-development-board/) | Webtronics |

⚠️: Programming this board via its USB connector requires interacting with
an unknown proprietary(?) bootloader. This bootloader may make it difficult
or impossible for you to load binaries not approved by Arduino. Alternative
programming interfaces are only available on the high density connectors.

Minimum supported Rust version
------------------------------

The Minimum Supported Rust Version (MSRV) at the moment is **1.52.0**. Older
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
