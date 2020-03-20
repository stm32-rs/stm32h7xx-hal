use core::fmt;
use crate::private;

pub trait DefaultTraits: fmt::Debug + PartialEq + Eq + Clone + Copy {}

impl<T> DefaultTraits for T
where
    T: fmt::Debug + PartialEq + Eq + Clone + Copy + private::Sealed,
{}