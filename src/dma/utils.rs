use core::fmt;
use core::ops::Deref;

pub trait DefaultTraits: Copy + Clone + PartialEq + Eq + fmt::Debug {}

impl<T: Copy + Clone + PartialEq + Eq + fmt::Debug> DefaultTraits for T {}

/// Guarantees that the stored reference is unique (i.e. not shared).
///
/// # Example
///
/// ```rust
/// # use core::cell::Cell;
///
/// pub struct ShallBeSync<'a> {
///     cell: UniqueRef<'a, Cell<u8>>,
/// }
///
/// impl<'a> ShallBeSync<'a> {
///     // Some sync-compliant methods
/// }
///
/// // This wouldn't be safe if multiple references to `cell` would coexist
/// unsafe impl Sync for ShallBeSync<'_> {}
/// ```
pub struct UniqueRef<'a, T> {
    inner: &'a T,
}

impl<'a, T> UniqueRef<'a, T> {
    #[allow(dead_code)]
    pub fn new(x: &'a mut T) -> Self {
        Self { inner: x }
    }

    pub unsafe fn new_unchecked(x: &'a T) -> Self {
        Self { inner: x }
    }
}

impl<T> Deref for UniqueRef<'_, T> {
    type Target = T;

    fn deref(&self) -> &T {
        &self.inner
    }
}
