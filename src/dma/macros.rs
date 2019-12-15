macro_rules! type_state {
    ($trait:ident, $($type_state:ident),*) => {
        pub unsafe trait $trait: core::fmt::Debug + PartialEq + Eq + Clone + Copy {}

        $(
            #[derive(Debug, PartialEq, Eq, Clone, Copy)]
            pub struct $type_state;

            unsafe impl $trait for $type_state {}
        )*
    };
}

macro_rules! bool_enum {
    ($name:ident, $doc:tt, $v_false:ident, $v_true:ident) => {
        #[doc=$doc]
        #[derive(Debug, PartialEq, Eq, Clone, Copy)]
        pub enum $name {
            $v_false,
            $v_true,
        }

        impl From<$name> for bool {
            fn from(val: $name) -> bool {
                val == $name::$v_true
            }
        }

        impl From<bool> for $name {
            fn from(val: bool) -> Self {
                if val {
                    $name::$v_true
                } else {
                    $name::$v_false
                }
            }
        }
    };
}

macro_rules! int_enum {
    ($name:ident <=> $ty:ty, $doc:tt, $($variant:ident <=> $num:tt),*) => {
        #[doc=$doc]
        #[derive(Debug, PartialEq, Eq, Clone, Copy)]
        pub enum $name {
            $(
                $variant,
            )*
        }

        impl From<$name> for $ty {
            fn from(val: $name) -> $ty {
                match val {
                    $(
                        $name::$variant => $num,
                    )*
                }
            }
        }

        impl core::convert::TryFrom<$ty> for $name {
            type Error = &'static str;

            fn try_from(val: $ty) -> Result<Self, &'static str> {
                match val {
                    $(
                        $num => Ok($name::$variant),
                    )*
                    _ => Err("Conversion failed"),
                }
            }
        }
    };
}

/// Macro for generating structs closely related to integers.
///
/// Attention: If you don't want to limit the length of the stored value,
/// set `$len = 0`.
macro_rules! int_struct {
    ($name:ident, $int_type:ident, $len:tt, $doc:tt) => {
        int_struct! { @INNER $name, $doc, $int_type, $len }
    };
    (@INNER $name:ident, $doc:tt, $int_type:ident, 0) => {
        #[doc=$doc]
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        pub struct $name(pub $int_type);

        impl $name {
            pub fn new(val: $int_type) -> Self {
                $name(val)
            }

            pub fn set_value(&mut self, val: $int_type) {
                self.0 = val;
            }

            pub fn value(self) -> $int_type {
                self.0
            }
        }

        impl From<$int_type> for $name {
            fn from(val: $int_type) -> Self {
                $name::new(val)
            }
        }

        impl From<$name> for $int_type {
            fn from(val: $name) -> $int_type {
                val.value()
            }
        }
    };
    (@INNER $name:ident, $doc:tt, $int_type:ident, $len:tt) => {
        #[doc=$doc]
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        pub struct $name($int_type);

        impl $name {
            pub fn new(val: $int_type) -> Self {
                if !$name::is_val_valid(val) {
                    // Expression `(1 << $len) - 1` overflows if $len = len of $int_type
                    panic!("The given value is incompatible with this type (max_value={}, got={})", (1 << $len) - 1, val);
                }

                $name(val)
            }

            pub fn value(self) -> $int_type {
                self.0
            }

            pub fn set_value(&mut self, val: $int_type) {
                if !$name::is_val_valid(val) {
                    panic!("The given value is incompatible with this type (max_value={}, got={})", (1 << $len) - 1, val);
                }

                self.0 = val;
            }

            fn is_val_valid(val: $int_type) -> bool {
                val >> ($len - 1) <= 0b1
            }
        }

        impl From<$name> for $int_type {
            fn from(val: $name) -> $int_type {
                val.value()
            }
        }

        impl core::convert::TryFrom<$int_type> for $name {
            // FIXME
            type Error = &'static str;

            fn try_from(val: $int_type) -> Result<Self, &'static str> {
                if $name::is_val_valid(val) {
                    Ok($name::new(val))
                } else {
                    Err("Conversion failed.")
                }
            }
        }
    };
}
