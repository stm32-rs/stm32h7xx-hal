//! Internal Module

/// Protects public traits from being implemented outside this crate
///
/// # Usage
///
/// ```
/// pub trait InternalTrait: Sealed {}
///
/// pub struct Foo;
///
/// // Enables `Foo` to implement `InternalTrait`.
/// impl Sealed for Foo {}
/// impl InternalTrait for Foo {}
/// ```
pub trait Sealed {}

impl Sealed for crate::stm32::DMA1 {}
impl Sealed for crate::stm32::DMA2 {}
