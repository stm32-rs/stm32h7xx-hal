use core::fmt::Debug;

pub trait TypeState: Debug + PartialEq + Eq + Clone + Copy {}
