use crate::private;
use core::fmt;

pub trait DefaultTraits: fmt::Debug + PartialEq + Eq + Clone + Copy {}

impl<T> DefaultTraits for T where T: fmt::Debug + PartialEq + Eq + Clone + Copy {}
