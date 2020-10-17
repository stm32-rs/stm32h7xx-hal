//! Macros used for implementing DMAs

// Convenience macro for implementing target addresses on peripherals
macro_rules! peripheral_target_address {
    ($(
        ($peripheral:ty, $register:ident $(($TRBUFF:ident))*, $size:ty,
         $dir:ty $(, $mux:expr)*)
    ),+ $(,)*) => {
        $(
            unsafe impl TargetAddress<$dir> for &mut $peripheral {
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
        )+
    };
}
