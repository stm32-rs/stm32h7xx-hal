//! Device electronic signature
//!
//! (stored in system flash memory)

/// This is the test voltage, in millivolts of the calibration done at
/// the factory
pub const VDDA_CALIB: u32 = 3300;

/// Uniqure Device ID register
pub struct Uid;

#[cfg(not(feature = "rm0455"))]
const UID_PTR: *const u8 = 0x1FF1_E800 as _;
#[cfg(feature = "rm0455")]
const UID_PTR: *const u8 = 0x08FF_F800 as _;

impl Uid {
    /// Read Unique Deivce ID
    pub fn read() -> &'static [u8; 12] {
        unsafe { &*UID_PTR.cast::<[u8; 12]>() }
    }
}

/// Size of integrated flash
pub struct FlashSize;

#[cfg(not(feature = "rm0455"))]
const FLASH_SIZE_PTR: *const u16 = 0x1FF1_E880 as _;
#[cfg(feature = "rm0455")]
const FLASH_SIZE_PTR: *const u16 = 0x08FF_F80C as _;

impl FlashSize {
    /// Read flash size in kilobytes
    pub fn kilo_bytes() -> u16 {
        unsafe { *FLASH_SIZE_PTR }
    }

    /// Read flash size in bytes
    pub fn bytes() -> usize {
        usize::from(Self::kilo_bytes()) * 1024
    }
}

/// ADC VREF calibration value is stored in at the factory
pub struct VREFIN_CAL;

#[cfg(not(feature = "rm0455"))]
const VREFIN_CAL_PTR: *const u16 = 0x1FF1_E860 as _;
#[cfg(feature = "rm0455")]
const VREFIN_CAL_PTR: *const u16 = 0x08FF_F810 as _;

impl VREFIN_CAL {
    /// Read calibration value
    pub fn read() -> u16 {
        unsafe { *VREFIN_CAL_PTR }
    }
}

/// A temperature reading taken at 30°C stored at the factory
pub struct TS_CAL_30;

#[cfg(not(feature = "rm0455"))]
const TS_CAL_30_PTR: *const u16 = 0x1FF1_E820 as _;
#[cfg(feature = "rm0455")]
const TS_CAL_30_PTR: *const u16 = 0x08FF_F814 as _;

impl TS_CAL_30 {
    /// Read calibration value
    pub fn read() -> u16 {
        unsafe { *TS_CAL_30_PTR }
    }
}

/// A temperature reading taken at 110°C stored at the factory
pub struct TS_CAL_110;

#[cfg(not(feature = "rm0455"))]
const TS_CAL_110_PTR: *const u16 = 0x1FF1_E840 as _;
#[cfg(feature = "rm0455")]
const TS_CAL_110_PTR: *const u16 = 0x08FF_F818 as _;

impl TS_CAL_110 {
    /// Read calibration value
    pub fn read() -> u16 {
        unsafe { *TS_CAL_110_PTR }
    }
}
