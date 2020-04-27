/// Guarantees that the stored reference is unique (i.e. not shared).
///
/// # Example
///
/// ```rust
/// # use core::cell::Cell;
///
/// pub struct ShallBeSync<'a> {
///     cell: &'a Cell<u8>,
/// }
///
/// impl<'a> ShallBeSync<'a> {
///     pub fn new(cell: UniqueRef<'a, Cell<u8>>) -> Self {
///         Self {
///             cell: cell.into_inner(),
///         }
///     }
/// }
///
/// // This wouldn't be safe if multiple references to `cell` could coexist upon initialization
/// unsafe impl Sync for ShallBeSync<'_> {}
/// ```
#[derive(Debug)]
pub struct UniqueRef<'a, T>
where
    T: ?Sized,
{
    inner: &'a T,
}

#[allow(dead_code)]
impl<'a, T> UniqueRef<'a, T>
where
    T: ?Sized,
{
    pub fn new(x: &'a mut T) -> Self {
        Self { inner: x }
    }

    pub unsafe fn new_unchecked(x: &'a T) -> Self {
        Self { inner: x }
    }

    pub fn get(&mut self) -> &T {
        &self.inner
    }

    pub unsafe fn get_unchecked(&self) -> &T {
        &*self.inner
    }

    /// Releases the inner reference. After this call, the reference may be duplicated again.
    pub fn into_inner(self) -> &'a T {
        self.inner
    }
}
