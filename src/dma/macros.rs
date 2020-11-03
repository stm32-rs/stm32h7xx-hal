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
    (($peripheral:ty, $register:ident $(($TRBUFF:ident))*, $size:ty,
      $dir:ty $(, $mux:expr)*)) => {
        unsafe impl TargetAddress<$dir> for $peripheral {
            #[inline(always)]
            fn address(&self) -> u32 {
                &self.$register as *const _ as u32
            }

            type MemSize = $size;
            $(
                const REQUEST_LINE: Option<u8> = Some($mux as u8);
            )*
                $(
                    const $TRBUFF: bool = true;
                )*
        }
    };

    ((SPI: $peripheral:ty, $rxreg:ident, $txreg:ident, [$($size:ty),+], $rxmux:expr, $txmux:expr)) => {
        // Access via PAC peripheral structures implies u8 sizing, as the sizing is unknown.
        unsafe impl TargetAddress<M2P> for $peripheral {
            #[inline(always)]
            fn address(&self) -> u32 {
                &self.$txreg as *const _ as u32
            }

            type MemSize = u8;

            const REQUEST_LINE: Option<u8> = Some($txmux as u8);
        }

        unsafe impl TargetAddress<P2M> for $peripheral {
            #[inline(always)]
            fn address(&self) -> u32 {
                &self.$rxreg as *const _ as u32
            }

            type MemSize = u8;

            const REQUEST_LINE: Option<u8> = Some($rxmux as u8);
        }

        // For each size
        $(
        unsafe impl TargetAddress<M2P> for spi::Spi<$peripheral, spi::Disabled, $size> {
            #[inline(always)]
            fn address(&self) -> u32 {
                &self.inner().$txreg as *const _ as u32
            }

            type MemSize = $size;

            const REQUEST_LINE: Option<u8> = Some($txmux as u8);
        }

        unsafe impl TargetAddress<P2M> for spi::Spi<$peripheral, spi::Disabled, $size> {
            #[inline(always)]
            fn address(&self) -> u32 {
                &self.inner().$rxreg as *const _ as u32
            }

            type MemSize = $size;

            const REQUEST_LINE: Option<u8> = Some($rxmux as u8);
        }
        )+
    };

    ((INNER: $peripheral:ty, $register:ident $(($TRBUFF:ident))*, $size:ty,
      $dir:ty $(, $mux:expr)*)) => {
        unsafe impl TargetAddress<$dir> for $peripheral {
            #[inline(always)]
            fn address(&self) -> u32 {
                &self.inner().$register as *const _ as u32
            }

            type MemSize = $size;
            $(
                const REQUEST_LINE: Option<u8> = Some($mux as u8);
            )*
                $(
                    const $TRBUFF: bool = true;
                )*
        }
    };

    (($peripheral:ty, $channel:ident.$register:ident, $size:ty,
      $dir:ty $(, $mux:expr)*)) => {
        unsafe impl TargetAddress<$dir> for $peripheral {
            #[inline(always)]
            fn address(&self) -> u32 {
                &self.$channel.$register as *const _ as u32
            }

            type MemSize = $size;
            $(
                const REQUEST_LINE: Option<u8> = Some($mux as u8);
            )*
        }
    };
}
