Examples
======

## Starting your own project

Although you can compile the provided examples directly, for your own project it
is recommended to make a new binary crate or start one based on existing board
specific examples / BSPs. Most examples will require tweaks to work for your
particular board. You should copy the examples into your binary crate and make
any necessary adjustments.

If you can find a suitable BSP, these usually also supply files which make
development easier. They might also contain board specific code or adaptions
required to make the examples work for your particular board.

The hello world of embedded development is usually to blink a LED. This example
is contained within the [examples folder](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/blinky.rs).

1. Make sure you have the required Rust cross-compiler installed. If you have not
   done this yet, you can run the following command to install it

   ```sh
   rustup target add thumbv7em-none-eabihf
   ```

2. Create a new binary crate with `cargo init`
3. To ensure that `cargo build` cross-compiles, it is recommended to create
   a `.cargo/config.toml` file. You can use the [`config.toml` provided
   in the cortex-m-quickstart](https://github.com/rust-embedded/cortex-m-quickstart/blob/master/.cargo/config.toml)
   repository and uncomment `target = "thumbv7em-none-eabihf"`.
4. Copy `memory.x` into your project. This file contains information required by
   the linker.
5. Copy `blinky.rs` to the `src/main.rs` in your binary crate and remove
   references to the `utilities` module.
6. Add a dependency on the HAL in `Cargo.toml`. Make sure to replace
   the configuration feature `stm32h743v` to match the part you are using:

   ```toml
   [dependencies.stm32h7xx-hal]
   version = "^0"
   features = ["stm32h743v"]
   ```

   You should also add dependencies for the other crates you need, such as
   `cortex-m-rt` or `embedded-hal`.

7. Build the application with

   ```sh
   cargo build
   ```

8. Flashing the MCU typically works differently for different boards. You can
   usually find instructions in the board specific crates or BSPs.

## Logging

Example specific features have been defined to enable different logging outputs for the examples.
If no logging feature is selected logging will be disabled and the panic handler will be halt.
Supported logging methods are:

### RTT (Real Time Trace)

Compile with the feature log-rtt

```
cargo build --features=stm32h750v,rt,log-rtt --examples
```

### Semihosting

Compile with the feature log-semihost. Note this method of logging is very slow.

```
cargo build --features=stm32h750v,rt,log-semihost --examples
```


### ITM (Instrumentation Trace Macrocell)
Compile with the feature log-itm

```
cargo build --features=stm32h750v,rt,log-itm --examples
```

Note that you may need to configure your debugger to output ITM, and/or
configure the ITM yourself. See [ITM.md](ITM.md)

If you select this feature flag, then the call to `logger::init()` internally
configures the ITM peripheral. If you also interact with the ITM peripheral
yourself, you should be aware that it has already been configured.

## Parts with a SMPS

For these parts, the program needs to know more about the power supply
scheme in order to successfully transition from Run* mode to Run mode. For
an explaination of Run* mode, see RM0433 Rev 7 Section 6.6.1 "System/D3
domain modes".

For your own code, see the
[documentation](https://docs.rs/stm32h7xx-hal/latest/stm32h7xx_hal/pwr/index.html#smps)
for the builder methods on `pwr`. However to make things easier for the
examples, there are feature flags that set common power configurations.

Flag | Situation | Applicable Boards (non-exhaustive)
---|---|---
`example-smps` | Board uses Internal SMPS | Any Nucleo with `-Q` suffix
`example-ldo` | Board uses LDO, internal SMPS unconnected |
none | Parts without internal SMPS |

The results of using the wrong power configuration for your hardware are
undefined(!). If you can still access the debug port, load a simple example
with the correct power configuration and power cycle the board.
