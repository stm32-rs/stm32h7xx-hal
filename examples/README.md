Examples
======

## Parts with a SMPS

For these parts, the program needs to know more about the power supply
scheme in order to successfully transition from Run* mode to Run mode. For
en explaination of Run* mode, see RM0433 Rev 7 Section 6.6.1 "System/D3
domain modes".

You can specify the power supply scheme through a builder method on
PWR. For instance if your board uses the internal LDO to supply VCORE, you
can replace:

```rust
let pwrcfg = pwr.freeze();
```

with

```rust
let pwrcfg = pwr.ldo().freeze();
```

The results of calling the wrong builder method for your hardware are
undefined(!).

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
