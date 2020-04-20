use core::fmt;

pub trait DefaultTraits: Copy + Clone + PartialEq + Eq + fmt::Debug {}

impl<T: Copy + Clone + PartialEq + Eq + fmt::Debug> DefaultTraits for T {}
