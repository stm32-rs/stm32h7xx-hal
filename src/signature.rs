//! Device electronic signature
//!
//! (stored in system flash memory)

/// This is the test voltage, in millivolts of the calibration done at
/// the factory
pub const VDDA_CALIB: u32 = 3300;

macro_rules! define_ptr_type {
    ($name: ident, $ptr: expr) => {
        impl $name {
            fn ptr() -> *const Self {
                $ptr as *const _
            }

            /// Returns a wrapped reference to the value in flash
            /// memory
            pub fn get() -> &'static Self {
                unsafe { &*Self::ptr() }
            }
        }
    };
}

/// Uniqure Device ID register
#[derive(Hash, Debug)]
#[repr(C)]
pub struct Uid(u128);

#[cfg(not(feature = "rm0455"))]
define_ptr_type!(Uid, 0x1FF1_E800);
#[cfg(feature = "rm0455")]
define_ptr_type!(Uid, 0x08FF_F800);

impl Uid {
    /// Read Unique Deivce ID
    pub fn read(&self) -> u128 {
        self.0
    }
}

/// Size of integrated flash
#[derive(Debug)]
#[repr(C)]
pub struct FlashSize(u16);

#[cfg(not(feature = "rm0455"))]
define_ptr_type!(FlashSize, 0x1FF1_E880);
#[cfg(feature = "rm0455")]
define_ptr_type!(FlashSize, 0x08FF_F80C);

impl FlashSize {
    /// Read flash size in kilobytes
    pub fn kilo_bytes(&self) -> u16 {
        self.0
    }

    /// Read flash size in bytes
    pub fn bytes(&self) -> usize {
        usize::from(self.kilo_bytes()) * 1024
    }
}

/// ADC VREF calibration value is stored in at the factory
#[derive(Debug)]
#[repr(C)]
pub struct VREFIN_CAL(u16);

#[cfg(not(feature = "rm0455"))]
define_ptr_type!(VREFIN_CAL, 0x1FF1_E860);
#[cfg(feature = "rm0455")]
define_ptr_type!(VREFIN_CAL, 0x08FF_F810);

impl VREFIN_CAL {
    /// Read calibration value
    pub fn read(&self) -> u16 {
        self.0
    }
}

/// A temperature reading taken at 30°C stored at the factory
#[derive(Debug)]
#[repr(C)]
pub struct TS_CAL_30(u16);

#[cfg(not(feature = "rm0455"))]
define_ptr_type!(TS_CAL_30, 0x1FF1_E820);
#[cfg(feature = "rm0455")]
define_ptr_type!(TS_CAL_30, 0x08FF_F814);

impl TS_CAL_30 {
    /// Read calibration value
    pub fn read(&self) -> u16 {
        self.0
    }
}

/// A temperature reading taken at 110°C stored at the factory
#[derive(Debug)]
#[repr(C)]
pub struct TS_CAL_110(u16);

#[cfg(not(feature = "rm0455"))]
define_ptr_type!(TS_CAL_110, 0x1FF1_E840);
#[cfg(feature = "rm0455")]
define_ptr_type!(TS_CAL_110, 0x08FF_F818);

impl TS_CAL_110 {
    /// Read calibration value
    pub fn read(&self) -> u16 {
        self.0
    }
}
