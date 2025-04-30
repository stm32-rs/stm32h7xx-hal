#![allow(unused)]
#![allow(unsafe_code)]

/// Create our own SyncUnsafeCell
///
/// TODO: Replace with core::cell::SyncUnsafeCell when stabilized
/// rust-lang/rust#95439
#[repr(transparent)]
pub struct SyncUnsafeCell<T>(core::cell::UnsafeCell<T>);

unsafe impl<T> Sync for SyncUnsafeCell<T> {}
impl<T> SyncUnsafeCell<T> {
    /// Constructs a new instance of UnsafeCell which will wrap the specified value
    pub const fn new(value: T) -> Self {
        Self(core::cell::UnsafeCell::new(value))
    }
    /// Get a mutable pointer to the wrapped value, without creating a temporary
    /// reference
    pub unsafe fn raw_get(this: *const Self) -> *mut T {
        let ucell: *const core::cell::UnsafeCell<T> =
            core::ptr::addr_of!((*this).0);

        core::cell::UnsafeCell::raw_get(ucell)
    }
}
