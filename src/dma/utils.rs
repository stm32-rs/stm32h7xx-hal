use core::fmt::Debug;

/// Used to enable usage of derive macros for structs using Self as Phantom Type.
pub trait PhantomType: Debug + PartialEq + Eq + Clone + Copy {}
