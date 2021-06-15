Examples
======

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

## Logging

Example specific features have been defined to enable different logging outputs for the examples. If no logging feature is selected logging will be disabled and the panic handler will be halt. Supported logging methods are:

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
