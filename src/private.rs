//! Internal Module

/// Protects public traits from being implemented outside this crate
///
/// # Usage
///
/// ```
/// pub trait InternalTrait: private::Sealed {}
///
/// pub struct Foo;
///
/// // Enables `Foo` to implement `InternalTrait`.
/// impl private::Sealed for Foo {}
/// impl InternalTrait for Foo {}
/// ```
pub trait Sealed {}

impl Sealed for crate::stm32::DMA1 {}
impl Sealed for crate::stm32::DMA2 {}
