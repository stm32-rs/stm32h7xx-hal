//! Macros used for implementing DMAs

// Convenience macro for implementing target addresses on peripherals
macro_rules! peripheral_target_address {
    ($(
        $peripheral:tt
    ),+ $(,)*) => {
        $(
            peripheral_target_instance!($peripheral);
        )+
    };
}
macro_rules! peripheral_target_instance {
    // PAC target
    (($peripheral:ty, $register:ident, $size:ty,
      $dir:ty $(, $mux:expr)*)) => {
        unsafe impl TargetAddress<$dir> for $peripheral {
            #[inline(always)]
            fn address(&self) -> usize {
                &self.$register() as *const _ as usize
            }

            type MemSize = $size;
            $(
                const REQUEST_LINE: Option<u8> = Some($mux as u8);
            )*
        }
    };

    // PAC target where register is within a channel
    (($peripheral:ty, $channel:ident.$register:ident, $size:ty,
      $dir:ty $(, $mux:expr)*)) => {
        unsafe impl TargetAddress<$dir> for $peripheral {
            #[inline(always)]
            fn address(&self) -> usize {
                &self.$channel().$register() as *const _ as usize
            }

            type MemSize = $size;
            $(
                const REQUEST_LINE: Option<u8> = Some($mux as u8);
            )*
        }
    };

    // PAC and HAL targets, generic
    ((HAL: $hal:ident<$peripheral:ty $(, $generic:ty)*>, $register:ident, $size:ty,
      $dir:ty $(, $mux:expr)*)) => {

        // PAC implementation
        unsafe impl TargetAddress<$dir> for $peripheral {
            #[inline(always)]
            fn address(&self) -> usize {
                &self.$register() as *const _ as usize
            }

            type MemSize = $size;
            $(
                const REQUEST_LINE: Option<u8> = Some($mux as u8);
            )*
        }

        // HAL implementation
        unsafe impl TargetAddress<$dir> for $hal<$peripheral $(, $generic)*> {
            #[inline(always)]
            fn address(&self) -> usize {
                &self.inner().$register() as *const _ as usize
            }

            type MemSize = $size;
            $(
                const REQUEST_LINE: Option<u8> = Some($mux as u8);
            )*
        }
    };

    // HAL target only, generic
    ((INNER: $hal:ty, $register:ident, $size:ty,
      $dir:ty $(, $mux:expr)*)) => {

        // HAL implementation
        unsafe impl TargetAddress<$dir> for $hal {
            #[inline(always)]
            fn address(&self) -> usize {
                &self.inner().$register() as *const _ as usize
            }

            type MemSize = $size;
            $(
                const REQUEST_LINE: Option<u8> = Some($mux as u8);
            )*
        }
    };

    // PAC and HAL targets for SPI peripheral
    ((SPI: $peripheral:ty, $rxreg:ident, $txreg:ident, [$($size:ty),+],
      $rxmux:expr, $txmux:expr)) => {

        // Access via PAC peripheral structures implies u8 sizing, as the sizing
        // is unknown.
        unsafe impl TargetAddress<M2P> for $peripheral {
            #[inline(always)]
            fn address(&self) -> usize {
                &self.$txreg() as *const _ as usize
            }

            type MemSize = u8;

            const REQUEST_LINE: Option<u8> = Some($txmux as u8);
        }

        unsafe impl TargetAddress<P2M> for $peripheral {
            #[inline(always)]
            fn address(&self) -> usize {
                &self.$rxreg() as *const _ as usize
            }

            type MemSize = u8;

            const REQUEST_LINE: Option<u8> = Some($rxmux as u8);
        }

        // HAL implementation For each size
        $(
            unsafe impl TargetAddress<M2P> for spi::Spi<$peripheral, spi::Disabled, $size> {
                #[inline(always)]
                fn address(&self) -> usize {
                    use spi::HalSpi;
                    &self.inner().$txreg() as *const _ as usize
                }

                type MemSize = $size;

                const REQUEST_LINE: Option<u8> = Some($txmux as u8);
            }

            unsafe impl TargetAddress<P2M> for spi::Spi<$peripheral, spi::Disabled, $size> {
                #[inline(always)]
                fn address(&self) -> usize {
                    use spi::HalSpi;
                    &self.inner().$rxreg() as *const _ as usize
                }

                type MemSize = $size;

                const REQUEST_LINE: Option<u8> = Some($rxmux as u8);
            }
        )+
    };

    // PAC and HAL targets for Serial Peripherals
    ((SERIAL: $peripheral:ty, $rxreg:ident, $txreg:ident, $rxmux:expr, $txmux:expr)) => {
        unsafe impl TargetAddress<M2P> for $peripheral {
            #[inline(always)]
            fn address(&self) -> usize {
                &self.$txreg() as *const _ as usize
            }

            type MemSize = u8;

            const REQUEST_LINE: Option<u8> = Some($txmux as u8);
            const TRBUFF: bool = true;
        }

        unsafe impl TargetAddress<P2M> for $peripheral {
            #[inline(always)]
            fn address(&self) -> usize {
                &self.$rxreg() as *const _ as usize
            }

            type MemSize = u8;

            const REQUEST_LINE: Option<u8> = Some($rxmux as u8);
            const TRBUFF: bool = true;
        }

        unsafe impl TargetAddress<M2P> for serial::Serial<$peripheral> {
            #[inline(always)]
            fn address(&self) -> usize {
                &self.usart.$txreg() as *const _ as usize
            }

            type MemSize = u8;

            const REQUEST_LINE: Option<u8> = Some($txmux as u8);
            const TRBUFF: bool = true;
        }

        unsafe impl TargetAddress<P2M> for serial::Serial<$peripheral> {
            #[inline(always)]
            fn address(&self) -> usize {
                &self.usart.$rxreg() as *const _ as usize
            }

            type MemSize = u8;

            const REQUEST_LINE: Option<u8> = Some($rxmux as u8);
            const TRBUFF: bool = true;
        }

        unsafe impl TargetAddress<M2P> for serial::Tx<$peripheral> {
            #[inline(always)]
            fn address(&self) -> usize {
                // unsafe: only the Tx part accesses the Tx register
                &unsafe { &*<$peripheral>::ptr() }.$txreg() as *const _ as usize
            }

            type MemSize = u8;

            const REQUEST_LINE: Option<u8> = Some($txmux as u8);
            const TRBUFF: bool = true;
        }

        unsafe impl TargetAddress<P2M> for serial::Rx<$peripheral> {
            #[inline(always)]
            fn address(&self) -> usize {
                // unsafe: only the Rx part accesses the Rx register
                &unsafe { &*<$peripheral>::ptr() }.$rxreg() as *const _ as usize
            }

            type MemSize = u8;

            const REQUEST_LINE: Option<u8> = Some($rxmux as u8);
            const TRBUFF: bool = true;
        }
    };
}
