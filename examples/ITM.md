# ITM (Instrumentation Trace Macrocell)

ITM support varies between debuggers. See also [this issue](https://github.com/stm32-rs/stm32h7xx-hal/issues/7).

## OpenOCD

ITM support for the H7 is [current broken in OpenOCD](https://sourceforge.net/p/openocd/tickets/266/).

There is a fix documented [here](https://gist.github.com/diondokter/5740aeb145e123c5b4dac7c7b32e36f6).

## Blackmagic

Enable ITM receive on the blackmagic:

```
monitor trace
```

However you will need to configure the ITM output yourself, [see here for
ideas](https://gist.github.com/richardeoin/3f74e4959baefa26f2c93cdc3850581c).
