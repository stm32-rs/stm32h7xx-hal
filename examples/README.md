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
let vos = pwr.freeze();
```

with

```rust
let vos = pwr.ldo().freeze();
```

The results of calling the wrong builder method for your hardware are
undefined(!).
