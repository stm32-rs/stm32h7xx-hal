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
