//! Analog to Digital Converter (ADC)
//!
//! ADC1 and ADC2 share a reset line. To initialise both of them, use the
//! [`adc12`](adc12) method.
//!
//! # Examples
//!
//! - [Reading a voltage using ADC1](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/adc.rs)
//! - [Reading a temperature using ADC3](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/temperature.rs)
//! - [Using ADC1 and ADC2 together](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/adc12.rs)
//! - [Using ADC1 and ADC2 in parallel](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/adc12_parallel.rs)
//! - [Using ADC1 through DMA](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/adc_dma.rs)

use crate::hal::adc::{Channel, OneShot};
use crate::hal::blocking::delay::DelayUs;

use core::convert::Infallible;
use core::marker::PhantomData;

use nb::block;

use crate::stm32::ADC12_COMMON;
use crate::stm32::{ADC1, ADC2};
#[cfg(not(feature = "rm0455"))]
use crate::stm32::{ADC3, ADC3_COMMON};

#[cfg(feature = "rm0455")]
use crate::stm32::adc12_common::ccr::PRESC_A;
#[cfg(not(feature = "rm0455"))]
use crate::stm32::adc3_common::ccr::PRESC_A;

use crate::gpio::{self, Analog};
use crate::pwr::{current_vos, VoltageScale};
use crate::rcc::rec::AdcClkSelGetter;
use crate::rcc::{rec, CoreClocks, ResetEnable};
use crate::time::Hertz;

#[cfg(any(feature = "rm0433", feature = "rm0399"))]
pub type Resolution = crate::stm32::adc3::cfgr::RES_A;
#[cfg(any(feature = "rm0455", feature = "rm0468"))]
pub type Resolution = crate::stm32::adc1::cfgr::RES_A;

trait NumberOfBits {
    fn number_of_bits(&self) -> u32;
}

impl NumberOfBits for Resolution {
    fn number_of_bits(&self) -> u32 {
        match *self {
            Resolution::EightBit => 8,
            Resolution::TenBit => 10,
            Resolution::TwelveBit => 12,
            Resolution::FourteenBit => 14,
            _ => 16,
        }
    }
}

/// Enabled ADC (type state)
pub struct Enabled;
/// Disabled ADC (type state)
pub struct Disabled;

pub trait ED {}
impl ED for Enabled {}
impl ED for Disabled {}

pub struct Adc<ADC, ED> {
    rb: ADC,
    sample_time: AdcSampleTime,
    resolution: Resolution,
    lshift: AdcLshift,
    clock: Hertz,
    current_channel: Option<u8>,
    _enabled: PhantomData<ED>,
}

/// ADC DMA modes
///
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AdcDmaMode {
    OneShot,
    Circular,
}

/// ADC sampling time
///
/// Options for the sampling time, each is T + 0.5 ADC clock cycles.
//
// Refer to RM0433 Rev 7 - Chapter 25.4.13
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(non_camel_case_types)]
pub enum AdcSampleTime {
    /// 1.5 cycles sampling time
    T_1,
    /// 2.5 cycles sampling time
    T_2,
    /// 8.5 cycles sampling time
    T_8,
    /// 16.5 cycles sampling time
    T_16,
    /// 32.5 cycles sampling time
    T_32,
    /// 64.5 cycles sampling time
    T_64,
    /// 387.5 cycles sampling time
    T_387,
    /// 810.5 cycles sampling time
    T_810,
}

impl Default for AdcSampleTime {
    fn default() -> Self {
        AdcSampleTime::T_32
    }
}
impl AdcSampleTime {
    /// Returns the number of half clock cycles represented by this sampling time
    fn clock_cycles_x2(&self) -> u32 {
        let x = match self {
            AdcSampleTime::T_1 => 1,
            AdcSampleTime::T_2 => 2,
            AdcSampleTime::T_8 => 8,
            AdcSampleTime::T_16 => 16,
            AdcSampleTime::T_32 => 32,
            AdcSampleTime::T_64 => 64,
            AdcSampleTime::T_387 => 387,
            AdcSampleTime::T_810 => 810,
        };
        (2 * x) + 1
    }
}

// Refer to RM0433 Rev 7 - Chapter 25.4.13
impl From<AdcSampleTime> for u8 {
    fn from(val: AdcSampleTime) -> u8 {
        match val {
            AdcSampleTime::T_1 => 0b000,
            AdcSampleTime::T_2 => 0b001,
            AdcSampleTime::T_8 => 0b010,
            AdcSampleTime::T_16 => 0b011,
            AdcSampleTime::T_32 => 0b100,
            AdcSampleTime::T_64 => 0b101,
            AdcSampleTime::T_387 => 0b110,
            AdcSampleTime::T_810 => 0b111,
        }
    }
}

/// The place in the sequence a given channel should be captured
#[derive(Debug, PartialEq, PartialOrd, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Sequence {
    /// 1
    One,
    /// 2
    Two,
    /// 3
    Three,
    /// 4
    Four,
    /// 5
    Five,
    /// 6
    Six,
    /// 7
    Seven,
    /// 8
    Eight,
    /// 9
    Nine,
    /// 10
    Ten,
    /// 11
    Eleven,
    /// 12
    Twelve,
    /// 13
    Thirteen,
    /// 14
    Fourteen,
    /// 15
    Fifteen,
    /// 16
    Sixteen,
}

impl From<Sequence> for u8 {
    fn from(s: Sequence) -> u8 {
        match s {
            Sequence::One => 0,
            Sequence::Two => 1,
            Sequence::Three => 2,
            Sequence::Four => 3,
            Sequence::Five => 4,
            Sequence::Six => 5,
            Sequence::Seven => 6,
            Sequence::Eight => 7,
            Sequence::Nine => 8,
            Sequence::Ten => 9,
            Sequence::Eleven => 10,
            Sequence::Twelve => 11,
            Sequence::Thirteen => 12,
            Sequence::Fourteen => 13,
            Sequence::Fifteen => 14,
            Sequence::Sixteen => 15,
        }
    }
}

impl From<u8> for Sequence {
    fn from(bits: u8) -> Self {
        match bits {
            0 => Sequence::One,
            1 => Sequence::Two,
            2 => Sequence::Three,
            3 => Sequence::Four,
            4 => Sequence::Five,
            5 => Sequence::Six,
            6 => Sequence::Seven,
            7 => Sequence::Eight,
            8 => Sequence::Nine,
            9 => Sequence::Ten,
            10 => Sequence::Eleven,
            11 => Sequence::Twelve,
            12 => Sequence::Thirteen,
            13 => Sequence::Fourteen,
            14 => Sequence::Fifteen,
            15 => Sequence::Sixteen,
            _ => unimplemented!(),
        }
    }
}

/// ADC LSHIFT\[3:0\] of the converted value
///
/// Only values in range of 0..=15 are allowed.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AdcLshift(u8);

impl AdcLshift {
    pub fn new(lshift: u8) -> Self {
        if lshift > 15 {
            panic!("LSHIFT[3:0] must be in range of 0..=15");
        }

        AdcLshift(lshift)
    }

    pub fn value(self) -> u8 {
        self.0
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AdcCalOffset(u16);

impl AdcCalOffset {
    pub fn value(self) -> u16 {
        self.0
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AdcCalLinear([u32; 6]);

impl AdcCalLinear {
    pub fn value(self) -> [u32; 6] {
        self.0
    }
}

macro_rules! adc_pins {
    ($ADC:ident, $($input:ty => $chan:expr),+ $(,)*) => {
        $(
            impl Channel<$ADC> for $input {
                type ID = u8;

                fn channel() -> u8 {
                    $chan
                }
            }
        )+
    };
}

macro_rules! adc_internal {
    ([$INT_ADC:ident, $INT_ADC_COMMON:ident]; $($input:ty => ($chan:expr, $en:ident)),+ $(,)*) => {
        $(
            impl $input {
                pub fn new() -> Self {
                    Self {}
                }

                /// Enables the internal voltage/sensor
                /// ADC must be disabled.
                pub fn enable(&mut self, _adc: &Adc<$INT_ADC, Disabled>) {

                    let common = unsafe { &*$INT_ADC_COMMON::ptr() };

                    common.ccr.modify(|_, w| w.$en().enabled());
                }
                /// Disables the internal voltage/sdissor
                /// ADC must be disabled.
                pub fn disable(&mut self, _adc: &Adc<$INT_ADC, Disabled>) {

                    let common = unsafe { &*$INT_ADC_COMMON::ptr() };

                    common.ccr.modify(|_, w| w.$en().disabled());
                }
            }

            adc_pins!($INT_ADC, $input => $chan);
        )+
    };
}

/// Vref internal signal
#[derive(Default)]
pub struct Vrefint;
/// Vbat internal signal
#[derive(Default)]
pub struct Vbat;
/// Internal temperature sensor
#[derive(Default)]
pub struct Temperature;

// Not implementing Pxy_C adc pins
// Just implmenting INPx pins (INNx defaulting to V_ref-)
//
// Refer to DS12110 Rev 7 - Chapter 5 (Table 9)
adc_pins!(ADC1,
    // 0, 1 are Pxy_C pins
    gpio::PF11<Analog> => 2,
    gpio::PA6<Analog> => 3,
    gpio::PC4<Analog> => 4,
    gpio::PB1<Analog> => 5,
    gpio::PF12<Analog> => 6,
    gpio::PA7<Analog> => 7,
    gpio::PC5<Analog> => 8,
    gpio::PB0<Analog> => 9,
    gpio::PC0<Analog> => 10,
    gpio::PC1<Analog> => 11,
    gpio::PC2<Analog> => 12,
    gpio::PC3<Analog> => 13,
    gpio::PA2<Analog> => 14,
    gpio::PA3<Analog> => 15,
    gpio::PA0<Analog> => 16,
    gpio::PA1<Analog> => 17,
    gpio::PA4<Analog> => 18,
    gpio::PA5<Analog> => 19,
);

adc_pins!(ADC2,
    // 0, 1 are Pxy_C pins
    gpio::PF13<Analog> => 2,
    gpio::PA6<Analog> => 3,
    gpio::PC4<Analog> => 4,
    gpio::PB1<Analog> => 5,
    gpio::PF14<Analog> => 6,
    gpio::PA7<Analog> => 7,
    gpio::PC5<Analog> => 8,
    gpio::PB0<Analog> => 9,
    gpio::PC0<Analog> => 10,
    gpio::PC1<Analog> => 11,
    gpio::PC2<Analog> => 12,
    gpio::PC3<Analog> => 13,
    gpio::PA2<Analog> => 14,
    gpio::PA3<Analog> => 15,
    // 16, 17 are dac_outX
    gpio::PA4<Analog> => 18,
    gpio::PA5<Analog> => 19,
);

#[cfg(feature = "rm0455")]
adc_internal!(
    [ADC2, ADC12_COMMON];

    Vbat => (14, vbaten),
    Temperature => (18, vsenseen),
    Vrefint => (19, vrefen)
);

// -------- ADC3 --------

#[cfg(any(feature = "rm0433", feature = "rm0399"))]
adc_pins!(ADC3,
    // 0, 1 are Pxy_C pins

    // EB: These two are a bit of "hack" (normally not supported by hal).
    // EB: Requires the PC2/3_C to be connected. Should not be included in any patches...
    // EB: Use these because they are faster and we then don't need to mess with syscfg
    // EB: (on mainboard)
    gpio::PC2<Analog> => 0, // Force PC2 to use channel 0 (PC2_C)
    gpio::PC3<Analog> => 1, // Add PC3_C on channel 0

    gpio::PF9<Analog> => 2,
    gpio::PF7<Analog> => 3,
    gpio::PF5<Analog> => 4,
    gpio::PF3<Analog> => 5,
    gpio::PF10<Analog> => 6,
    gpio::PF8<Analog> => 7,
    gpio::PF6<Analog> => 8,
    gpio::PF4<Analog> => 9,
    gpio::PC0<Analog> => 10,
    gpio::PC1<Analog> => 11,
    //gpio::PC2<Analog> => 12,
    gpio::PH2<Analog> => 13,
    gpio::PH3<Analog> => 14,
    gpio::PH4<Analog> => 15,
    gpio::PH5<Analog> => 16,
);
#[cfg(any(feature = "rm0433", feature = "rm0399"))]
adc_internal!(
    [ADC3, ADC3_COMMON];

    Vbat => (17, vbaten),
    Temperature => (18, vsenseen),
    Vrefint => (19, vrefen)
);

// ADC 3 not present on RM0455 parts

#[cfg(feature = "rm0468")]
adc_pins!(ADC3,
    // 0, 1 are Pxy_C pins
    gpio::PF9<Analog> => 2,
    gpio::PF7<Analog> => 3,
    gpio::PF5<Analog> => 4,
    gpio::PF3<Analog> => 5,
    gpio::PF10<Analog> => 6,
    gpio::PF8<Analog> => 7,
    gpio::PF6<Analog> => 8,
    gpio::PF4<Analog> => 9,
    gpio::PC0<Analog> => 10,
    gpio::PC1<Analog> => 11,
    gpio::PC2<Analog> => 12,
    gpio::PH2<Analog> => 13,
    gpio::PH3<Analog> => 14,
    gpio::PH4<Analog> => 15,
    // Although ADC3_INP16 appears in device datasheets (on PH5), RM0468 Rev 7
    // Figure 231 does not show ADC3_INP16
);
#[cfg(feature = "rm0468")]
adc_internal!(
    [ADC3, ADC3_COMMON];

    Vbat => (16, vbaten),
    Temperature => (17, vsenseen),
    Vrefint => (18, vrefen)
);

pub trait AdcExt<ADC>: Sized {
    type Rec: ResetEnable;

    fn adc(
        self,
        f_adc: impl Into<Hertz>,
        delay: &mut impl DelayUs<u8>,
        prec: Self::Rec,
        clocks: &CoreClocks,
    ) -> Adc<ADC, Disabled>;
}

/// Stored ADC config can be restored using the `Adc::restore_cfg` method
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct StoredConfig(AdcSampleTime, Resolution, AdcLshift);

#[cfg(feature = "defmt")]
impl defmt::Format for StoredConfig {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(
            fmt,
            "StoredConfig({:?}, {:?}, {:?})",
            self.0,
            defmt::Debug2Format(&self.1),
            self.2
        )
    }
}

/// Returns the frequency of the current adc_ker_ck
///
/// # Panics
///
/// Panics if the kernel clock is not running
fn kernel_clk_unwrap(
    prec: &impl AdcClkSelGetter,
    clocks: &CoreClocks,
) -> Hertz {
    match prec.get_kernel_clk_mux() {
        Some(rec::AdcClkSel::Pll2P) => {
            clocks.pll2_p_ck().expect("ADC: PLL2_P must be enabled")
        }
        Some(rec::AdcClkSel::Pll3R) => {
            clocks.pll3_r_ck().expect("ADC: PLL3_R must be enabled")
        }
        Some(rec::AdcClkSel::Per) => {
            clocks.per_ck().expect("ADC: PER clock must be enabled")
        }
        _ => unreachable!(),
    }
}

// ADC12 is a unique case where a single reset line is used to control two
// peripherals that have separate peripheral definitions in the SVD.

/// Initialise ADC12 together
///
/// Sets all configurable parameters to one-shot defaults,
/// performs a boot-time calibration.
pub fn adc12(
    adc1: ADC1,
    adc2: ADC2,
    f_adc: impl Into<Hertz>,
    delay: &mut impl DelayUs<u8>,
    prec: rec::Adc12,
    clocks: &CoreClocks,
) -> (Adc<ADC1, Disabled>, Adc<ADC2, Disabled>) {
    // Consume ADC register block, produce ADC1/2 with default settings
    let mut adc1 = Adc::<ADC1, Disabled>::default_from_rb(adc1);
    let mut adc2 = Adc::<ADC2, Disabled>::default_from_rb(adc2);

    // Check adc_ker_ck_input
    kernel_clk_unwrap(&prec, clocks);

    // Enable AHB clock
    let prec = prec.enable();

    // Power Down
    adc1.power_down();
    adc2.power_down();

    // Reset peripheral
    let prec = prec.reset();

    // Power Up, Preconfigure and Calibrate
    adc1.power_up(delay);
    adc2.power_up(delay);
    let f_adc = adc1.configure_clock(f_adc.into(), prec, clocks); // ADC12_COMMON
    adc2.clock = f_adc;
    adc1.preconfigure();
    adc2.preconfigure();
    adc1.calibrate();
    adc2.calibrate();

    (adc1, adc2)
}

/// Free both ADC1 and ADC2 along with PREC.
///
/// Since ADC1 and ADC2 are controlled together, they are freed together.
pub fn free_adc12<ED>(
    adc1: Adc<ADC1, ED>,
    adc2: Adc<ADC2, ED>,
) -> (ADC1, ADC2, rec::Adc12) {
    (
        adc1.rb,
        adc2.rb,
        rec::Adc12 {
            _marker: PhantomData,
        },
    )
}

#[cfg(not(feature = "rm0455"))]
/// Freeing both the peripheral and PREC is possible for ADC3
impl<ED> Adc<ADC3, ED> {
    /// Releases the ADC peripheral
    pub fn free(self) -> (ADC3, rec::Adc3) {
        (
            self.rb,
            rec::Adc3 {
                _marker: PhantomData,
            },
        )
    }
}

#[allow(unused_macros)]
macro_rules! adc_hal {
    ($(
        $ADC:ident, $ADC_COMMON:ident: (
            $adcX: ident,
            $Rec:ident
            $(, $ldordy:ident )*
        )
    ),+ $(,)*) => {
        $(
            impl AdcExt<$ADC> for $ADC {
                type Rec = rec::$Rec;

	            fn adc(self,
                       f_adc: impl Into<Hertz>,
                       delay: &mut impl DelayUs<u8>,
                       prec: rec::$Rec,
                       clocks: &CoreClocks) -> Adc<$ADC, Disabled>
	            {
	                Adc::$adcX(self, f_adc, delay, prec, clocks)
	            }
	        }

            impl Adc<$ADC, Disabled> {
                /// Initialise ADC
                ///
                /// Sets all configurable parameters to one-shot defaults,
                /// performs a boot-time calibration.
                pub fn $adcX(adc: $ADC, f_adc: impl Into<Hertz>, delay: &mut impl DelayUs<u8>,
                             prec: rec::$Rec, clocks: &CoreClocks
                ) -> Self {
                    // Consume ADC register block, produce Self with default
                    // settings
                    let mut adc = Self::default_from_rb(adc);

                    // Enable AHB clock
                    let prec = prec.enable();

                    // Power Down
                    adc.power_down();

                    // Reset peripheral
                    let prec = prec.reset();

                    // Power Up, Preconfigure and Calibrate
                    adc.power_up(delay);
                    adc.configure_clock(f_adc.into(), prec, clocks);
                    adc.preconfigure();
                    adc.calibrate();

                    adc
                }
                /// Creates ADC with default settings
                fn default_from_rb(rb: $ADC) -> Self {
                    Self {
                        rb,
                        sample_time: AdcSampleTime::default(),
                        resolution: Resolution::SixteenBit,
                        lshift: AdcLshift::default(),
                        clock: Hertz::from_raw(0),
                        current_channel: None,
                        _enabled: PhantomData,
                    }
                }
                /// Sets the clock configuration for this ADC. This is common
                /// between ADC1 and ADC2, so the prec block is used to ensure
                /// this method can only be called on one of the ADCs (or both,
                /// using the [adc12](#method.adc12) method).
                ///
                /// Only `CKMODE[1:0]` = 0 is supported
                fn configure_clock(&mut self, f_adc: Hertz, prec: rec::$Rec, clocks: &CoreClocks) -> Hertz {
                    let ker_ck = kernel_clk_unwrap(&prec, clocks);

                    let max_ker_ck = match current_vos() {
                        // See RM0468 Rev 3 Table 56.
                        #[cfg(feature = "rm0468")]
                        VoltageScale::Scale0 | VoltageScale::Scale1 => 160_000_000,
                        #[cfg(feature = "rm0468")]
                        VoltageScale::Scale2 => 60_000_000,
                        #[cfg(feature = "rm0468")]
                        VoltageScale::Scale3 => 40_000_000,

                        // See RM0433 Rev 7 Table 59.
                        #[cfg(not(feature = "rm0468"))]
                        VoltageScale::Scale0 | VoltageScale::Scale1 => 80_000_000,
                        #[cfg(not(feature = "rm0468"))]
                        VoltageScale::Scale2 | VoltageScale::Scale3 => 40_000_000
                    };
                    assert!(ker_ck.raw() <= max_ker_ck,
                            "Kernel clock violates maximum frequency defined in Reference Manual. \
                             Can result in erroneous ADC readings");

                    let f_adc = self.configure_clock_unchecked(f_adc, prec, clocks);

                    // Maximum ADC clock speed. With BOOST = 0 there is a no
                    // minimum frequency given in part datasheets
                    assert!(f_adc.raw() <= 50_000_000);

                    f_adc
                }

                /// No clock checks
                fn configure_clock_unchecked(&mut self, f_adc: Hertz, prec: rec::$Rec, clocks: &CoreClocks) -> Hertz {
                    let ker_ck = kernel_clk_unwrap(&prec, clocks);

                    // Target mux output. See RM0433 Rev 7 - Figure 136.
                    #[cfg(feature = "revision_v")]
                    let f_target = f_adc.raw() * 2;

                    #[cfg(not(feature = "revision_v"))]
                    let f_target = f_adc.raw();

                    let (divider, presc) = match (ker_ck.raw() + f_target - 1) / f_target {
                        1 => (1, PRESC_A::Div1),
                        2 => (2, PRESC_A::Div2),
                        3..=4 => (4, PRESC_A::Div4),
                        5..=6 => (6, PRESC_A::Div6),
                        7..=8 => (8, PRESC_A::Div8),
                        9..=10 => (10, PRESC_A::Div10),
                        11..=12 => (12, PRESC_A::Div12),
                        13..=16 => (16, PRESC_A::Div16),
                        17..=32 => (32, PRESC_A::Div32),
                        33..=64 => (64, PRESC_A::Div64),
                        65..=128 => (128, PRESC_A::Div128),
                        129..=256 => (256, PRESC_A::Div256),
                        _ => panic!("Selecting the ADC clock required a prescaler > 256, \
                                     which is not possible in hardware. Either increase the ADC \
                                     clock frequency or decrease the kernel clock frequency"),
                    };
                    unsafe { &*$ADC_COMMON::ptr() }.ccr.modify(|_, w| w.presc().variant(presc));

                    // Calculate actual value. See RM0433 Rev 7 - Figure 136.
                    #[cfg(feature = "revision_v")]
                    let f_adc = Hertz::from_raw(ker_ck.raw() / (divider * 2));

                    // Calculate actual value Revison Y. See RM0433 Rev 7 - Figure 137.
                    #[cfg(not(feature = "revision_v"))]
                    let f_adc = Hertz::from_raw(ker_ck.raw() / divider);

                    self.clock = f_adc;
                    f_adc
                }

                /// Disables Deeppowerdown-mode and enables voltage regulator
                ///
                /// Note: After power-up, a [`calibration`](#method.calibrate) shall be run
                pub fn power_up(&mut self, delay: &mut impl DelayUs<u8>) {
                    // Refer to RM0433 Rev 7 - Chapter 25.4.6
                    self.rb.cr.modify(|_, w|
                        w.deeppwd().clear_bit()
                            .advregen().set_bit()
                    );
                    delay.delay_us(10_u8);

                    // check LDORDY bit if present
                    $(
                        #[cfg(feature = "revision_v")]
                        while {
                            let $ldordy = self.rb.isr.read().bits() & 0x1000;
                            $ldordy == 0
                        }{}
                    )*
                }

                /// Enables Deeppowerdown-mode and disables voltage regulator
                ///
                /// Note: This resets the [`calibration`](#method.calibrate) of the ADC
                pub fn power_down(&mut self) {
                    // Refer to RM0433 Rev 7 - Chapter 25.4.6
                    self.rb.cr.modify(|_, w|
                        w.deeppwd().set_bit()
                            .advregen().clear_bit()
                    );
                }

                /// Calibrates the ADC in single channel mode
                ///
                /// Note: The ADC must be disabled
                pub fn calibrate(&mut self) {
                    // Refer to RM0433 Rev 7 - Chapter 25.4.8
                    self.check_calibration_conditions();

                    // single channel (INNx equals to V_ref-)
                    self.rb.cr.modify(|_, w|
                        w.adcaldif().clear_bit()
                            .adcallin().set_bit()
                    );
                    // calibrate
                    self.rb.cr.modify(|_, w| w.adcal().set_bit());
                    while self.rb.cr.read().adcal().bit_is_set() {}
                }

                fn check_calibration_conditions(&self) {
                    let cr = self.rb.cr.read();
                    if cr.aden().bit_is_set() {
                        panic!("Cannot start calibration when the ADC is enabled");
                    }
                    if cr.deeppwd().bit_is_set() {
                        panic!("Cannot start calibration when the ADC is in deeppowerdown-mode");
                    }
                    if cr.advregen().bit_is_clear() {
                        panic!("Cannot start calibration when the ADC voltage regulator is disabled");
                    }
                }

                /// Configuration process prior to enabling the ADC
                ///
                /// Note: the ADC must be disabled
                fn preconfigure(&mut self) {
                    self.configure_channels_dif_mode();
                }

                /// Sets channels to single ended mode
                fn configure_channels_dif_mode(&mut self) {
                    self.rb.difsel.reset();
                }

                /// Configuration process immediately after enabling the ADC
                fn configure(&mut self) {
                    // Single conversion mode, Software trigger
                    // Refer to RM0433 Rev 7 - Chapters 25.4.15, 25.4.19
                    self.rb.cfgr.modify(|_, w|
                        w.cont().clear_bit()
                            .exten().disabled()
                            .discen().set_bit()
                    );

                    // Enables boost mode for highest possible clock frequency
                    //
                    // Refer to RM0433 Rev 7 - Chapter 25.4.3
                    #[cfg(not(feature = "revision_v"))]
                    self.rb.cr.modify(|_, w| w.boost().set_bit());
                    #[cfg(feature = "revision_v")]
                    self.rb.cr.modify(|_, w| {
                        if self.clock.raw() <= 6_250_000 {
                            w.boost().lt6_25()
                        } else if self.clock.raw() <= 12_500_000 {
                            w.boost().lt12_5()
                        } else if self.clock.raw() <= 25_000_000 {
                            w.boost().lt25()
                        } else {
                            w.boost().lt50()
                        }
                    });
                }

                /// Enable ADC
                pub fn enable(mut self) -> Adc<$ADC, Enabled> {
                    // Refer to RM0433 Rev 7 - Chapter 25.4.9
                    self.rb.isr.modify(|_, w| w.adrdy().set_bit());
                    self.rb.cr.modify(|_, w| w.aden().set_bit());
                    while self.rb.isr.read().adrdy().bit_is_clear() {}
                    self.rb.isr.modify(|_, w| w.adrdy().set_bit());

                    self.configure();

                    Adc {
                        rb: self.rb,
                        sample_time: self.sample_time,
                        resolution: self.resolution,
                        lshift: self.lshift,
                        clock: self.clock,
                        current_channel: None,
                        _enabled: PhantomData,
                    }
                }
            }

            impl Adc<$ADC, Enabled> {
                fn stop_regular_conversion(&mut self) {
                    self.rb.cr.modify(|_, w| w.adstp().set_bit());
                    while self.rb.cr.read().adstp().bit_is_set() {}
                }

                fn stop_injected_conversion(&mut self) {
                    self.rb.cr.modify(|_, w| w.jadstp().set_bit());
                    while self.rb.cr.read().jadstp().bit_is_set() {}
                }

                fn set_chan_smp(&mut self, chan: u8) {
                    let t = self.get_sample_time().into();
                    if chan <= 9 {
                        self.rb.smpr1.modify(|_, w| match chan {
                            0 => w.smp0().bits(t),
                            1 => w.smp1().bits(t),
                            2 => w.smp2().bits(t),
                            3 => w.smp3().bits(t),
                            4 => w.smp4().bits(t),
                            5 => w.smp5().bits(t),
                            6 => w.smp6().bits(t),
                            7 => w.smp7().bits(t),
                            8 => w.smp8().bits(t),
                            9 => w.smp9().bits(t),
                            _ => unreachable!(),
                        })
                    } else {
                        self.rb.smpr2.modify(|_, w| match chan {
                            10 => w.smp10().bits(t),
                            11 => w.smp11().bits(t),
                            12 => w.smp12().bits(t),
                            13 => w.smp13().bits(t),
                            14 => w.smp14().bits(t),
                            15 => w.smp15().bits(t),
                            16 => w.smp16().bits(t),
                            17 => w.smp17().bits(t),
                            18 => w.smp18().bits(t),
                            19 => w.smp19().bits(t),
                            _ => unreachable!(),
                        })
                    }
                }

                // This method starts a conversion sequence on the given channel
                fn start_conversion_common(&mut self, chan: u8) {
                    self.check_conversion_conditions();

                    // Set LSHIFT[3:0]
                    self.rb.cfgr2.modify(|_, w| w.lshift().bits(self.get_lshift().value()));

                    // Select channel (with preselection, refer to RM0433 Rev 7 - Chapter 25.4.12)
                    self.rb.pcsel.modify(|r, w| unsafe { w.pcsel().bits(r.pcsel().bits() | (1 << chan)) });
                    self.set_chan_smp(chan);
                    self.rb.sqr1.modify(|_, w| unsafe {
                        w.sq1().bits(chan)
                            .l().bits(0)
                    });
                    self.current_channel = Some(chan);

                    // Perform conversion
                    self.rb.cr.modify(|_, w| w.adstart().set_bit());
                }

                /// Reset the sequence
                #[inline(always)]
                pub fn reset_sequence(&mut self) {
                    //The reset state is One conversion selected
                    self.rb.sqr1.modify(|_, w| w.l().bits(Sequence::One.into()));
                }

                /// Returns the current sequence length. Primarily useful for configuring DMA.
                #[inline(always)]
                pub fn sequence_length(&mut self) -> u8 {
                    self.rb.sqr1.read().l().bits() + 1
                }

                /// Configure sequence
                ///
                /// Example:
                /// ```
                /// let mut adc1 = adc1.enable();
                /// adc1.set_resolution(adc::Resolution::SixteenBit);
                /// // Configure sequence
                /// adc1.reset_sequence();
                /// adc1.configure_sequence(&mut adcpin1, adc::Sequence::One,  adc::AdcSampleTime::T_16);
                /// adc1.configure_sequence(&mut adcpin2, adc::Sequence::Two,  adc::AdcSampleTime::T_64);
                /// adc1.configure_sequence(&mut adcpin3, adc::Sequence::Three,  adc::AdcSampleTime::T_16);
                /// ```
                pub fn configure_sequence<PIN>(&mut self, _pin: &mut PIN, sequence: Sequence, sample_time: AdcSampleTime)
                    where PIN: Channel<$ADC, ID = u8>,
                {
                    let chan = PIN::channel();
                    assert!(chan <= 19);

                    //Check the sequence is long enough
                    self.rb.sqr1.modify(|r, w| {
                        let prev: Sequence = r.l().bits().into();
                        if prev < sequence {
                            w.l().bits(sequence.into())
                        } else {
                            w
                        }
                    });

                    //Set the channel in the right sequence field
                    match sequence {
                        Sequence::One      => self.rb.sqr1.modify(|_, w| unsafe {w.sq1().bits(chan) }),
                        Sequence::Two      => self.rb.sqr1.modify(|_, w| unsafe {w.sq2().bits(chan) }),
                        Sequence::Three    => self.rb.sqr1.modify(|_, w| unsafe {w.sq3().bits(chan) }),
                        Sequence::Four     => self.rb.sqr1.modify(|_, w| unsafe {w.sq4().bits(chan) }),
                        Sequence::Five     => self.rb.sqr2.modify(|_, w| unsafe {w.sq5().bits(chan) }),
                        Sequence::Six      => self.rb.sqr2.modify(|_, w| unsafe {w.sq6().bits(chan) }),
                        Sequence::Seven    => self.rb.sqr2.modify(|_, w| unsafe {w.sq7().bits(chan) }),
                        Sequence::Eight    => self.rb.sqr2.modify(|_, w| unsafe {w.sq8().bits(chan) }),
                        Sequence::Nine     => self.rb.sqr2.modify(|_, w| unsafe {w.sq9().bits(chan) }),
                        Sequence::Ten      => self.rb.sqr3.modify(|_, w| unsafe {w.sq10().bits(chan) }),
                        Sequence::Eleven   => self.rb.sqr3.modify(|_, w| unsafe {w.sq11().bits(chan) }),
                        Sequence::Twelve   => self.rb.sqr3.modify(|_, w| unsafe {w.sq12().bits(chan) }),
                        Sequence::Thirteen => self.rb.sqr3.modify(|_, w| unsafe {w.sq13().bits(chan) }),
                        Sequence::Fourteen => self.rb.sqr3.modify(|_, w| unsafe {w.sq14().bits(chan) }),
                        Sequence::Fifteen  => self.rb.sqr4.modify(|_, w| unsafe {w.sq15().bits(chan) }),
                        Sequence::Sixteen  => self.rb.sqr4.modify(|_, w| unsafe {w.sq16().bits(chan) }),
                    }

                    //Set the sample time for the channel
                    let st = u8::from(sample_time);
                    match chan {
                        0 => self.rb.smpr1.modify(|_, w| w.smp0().bits(st) ),
                        1 => self.rb.smpr1.modify(|_, w| w.smp1().bits(st) ),
                        2 => self.rb.smpr1.modify(|_, w| w.smp2().bits(st) ),
                        3 => self.rb.smpr1.modify(|_, w| w.smp3().bits(st) ),
                        4 => self.rb.smpr1.modify(|_, w| w.smp4().bits(st) ),
                        5 => self.rb.smpr1.modify(|_, w| w.smp5().bits(st) ),
                        6 => self.rb.smpr1.modify(|_, w| w.smp6().bits(st) ),
                        7 => self.rb.smpr1.modify(|_, w| w.smp7().bits(st) ),
                        8 => self.rb.smpr1.modify(|_, w| w.smp8().bits(st) ),
                        9 => self.rb.smpr1.modify(|_, w| w.smp9().bits(st) ),
                        10 => self.rb.smpr2.modify(|_, w| w.smp10().bits(st) ),
                        11 => self.rb.smpr2.modify(|_, w| w.smp11().bits(st) ),
                        12 => self.rb.smpr2.modify(|_, w| w.smp12().bits(st) ),
                        13 => self.rb.smpr2.modify(|_, w| w.smp13().bits(st) ),
                        14 => self.rb.smpr2.modify(|_, w| w.smp14().bits(st) ),
                        15 => self.rb.smpr2.modify(|_, w| w.smp15().bits(st) ),
                        16 => self.rb.smpr2.modify(|_, w| w.smp16().bits(st) ),
                        17 => self.rb.smpr2.modify(|_, w| w.smp17().bits(st) ),
                        18 => self.rb.smpr2.modify(|_, w| w.smp18().bits(st) ),
                        _ => unimplemented!(),
                    }

                    // Select channel (with preselection, refer to RM0433 Rev 7 - Chapter 25.4.12)
                    self.rb.pcsel.modify(|r, w| unsafe { w.pcsel().bits(r.pcsel().bits() | (1 << chan)) });

                }

                /// Starts sequence conversion in DMA mode.
                ///
                /// This method starts a conversion sequence with DMA
                /// enabled. The DMA mode selected depends on the [`AdcDmaMode`] specified.
                /// Waits for the hardware to indicate it's actually started.
                #[inline(always)]
                pub fn start_conversion_sequence_dma(&mut self, mode: AdcDmaMode) {
                    self.check_conversion_conditions();

                    // Set DMA mode
                    self.rb.cfgr.modify(|_, w| w.dmngt().bits(match mode {
                        AdcDmaMode::OneShot => 0b01,
                        AdcDmaMode::Circular => 0b11,
                    }));

                    // Set resolution
                    self.rb.cfgr.modify(|_, w| unsafe { w.res().bits(self.get_resolution().into()) });
                    // Set continuous mode
                    self.rb.cfgr.modify(|_, w| w.cont().set_bit().discen().clear_bit() );

                    // Set LSHIFT[3:0]
                    self.rb.cfgr2.modify(|_, w| w.lshift().bits(self.get_lshift().value()));

                    //Start conversion
                    self.rb.cr.modify(|_, w| w.adstart().set_bit());

                    while !self.rb.cr.read().adstart().bit_is_set() {}
                }

                /// Start conversion
                ///
                /// This method starts a conversion sequence on the given pin.
                /// The value can be then read through the `read_sample` method.
                // Refer to RM0433 Rev 7 - Chapter 25.4.16
                pub fn start_conversion<PIN>(&mut self, _pin: &mut PIN)
                    where PIN: Channel<$ADC, ID = u8>,
                {
                    let chan = PIN::channel();
                    assert!(chan <= 19);

                    // Set resolution
                    self.rb.cfgr.modify(|_, w| unsafe { w.res().bits(self.get_resolution().into()) });
                    // Set discontinuous mode
                    self.rb.cfgr.modify(|_, w| w.cont().clear_bit().discen().set_bit());

                    self.start_conversion_common(chan);
                }

                /// Start conversion in DMA mode
                ///
                /// This method starts a conversion sequence with DMA
                /// enabled. The DMA mode selected depends on the [`AdcDmaMode`] specified.
                pub fn start_conversion_dma<PIN>(&mut self, _pin: &mut PIN, mode: AdcDmaMode)
                    where PIN: Channel<$ADC, ID = u8>,
                {
                    let chan = PIN::channel();
                    assert!(chan <= 19);

                    // Set resolution
                    self.rb.cfgr.modify(|_, w| unsafe { w.res().bits(self.get_resolution().into()) });


                    self.rb.cfgr.modify(|_, w| w.dmngt().bits(match mode {
                        AdcDmaMode::OneShot => 0b01,
                        AdcDmaMode::Circular => 0b11,
                    }));

                    // Set continuous mode
                    self.rb.cfgr.modify(|_, w| w.cont().set_bit().discen().clear_bit() );

                    self.start_conversion_common(chan);
                }


                /// Read sample
                ///
                /// `nb::Error::WouldBlock` in case the conversion is still
                /// progressing.
                // Refer to RM0433 Rev 7 - Chapter 25.4.16
                pub fn read_sample(&mut self) -> nb::Result<u32, Infallible> {
                    let chan = self.current_channel.expect("No channel was selected, use start_conversion first");

                    // Check if the conversion is finished
                    if self.rb.isr.read().eoc().bit_is_clear() {
                        return Err(nb::Error::WouldBlock);
                    }

                    // Disable preselection of this channel, refer to RM0433 Rev 7 - Chapter 25.4.12
                    self.rb.pcsel.modify(|r, w| unsafe { w.pcsel().bits(r.pcsel().bits() & !(1 << chan)) });
                    self.current_channel = None;

                    // Retrieve result
                    let result = self.rb.dr.read().bits();
                    nb::Result::Ok(result)
                }

                fn check_conversion_conditions(&self) {
                    let cr = self.rb.cr.read();
                    // Ensure that no conversions are ongoing
                    if cr.adstart().bit_is_set() {
                        panic!("Cannot start conversion because a regular conversion is ongoing");
                    }
                    if cr.jadstart().bit_is_set() {
                        panic!("Cannot start conversion because an injected conversion is ongoing");
                    }
                    // Ensure that the ADC is enabled
                    if cr.aden().bit_is_clear() {
                        panic!("Cannot start conversion because ADC is currently disabled");
                    }
                    if cr.addis().bit_is_set() {
                        panic!("Cannot start conversion because there is a pending request to disable the ADC");
                    }
                }

                /// Disable ADC
                pub fn disable(mut self) -> Adc<$ADC, Disabled> {
                    let cr = self.rb.cr.read();
                    // Refer to RM0433 Rev 7 - Chapter 25.4.9
                    if cr.adstart().bit_is_set() {
                        self.stop_regular_conversion();
                    }
                    if cr.jadstart().bit_is_set() {
                        self.stop_injected_conversion();
                    }

                    self.rb.cr.modify(|_, w| w.addis().set_bit());
                    while self.rb.cr.read().aden().bit_is_set() {}

                    Adc {
                        rb: self.rb,
                        sample_time: self.sample_time,
                        resolution: self.resolution,
                        lshift: self.lshift,
                        clock: self.clock,
                        current_channel: None,
                        _enabled: PhantomData,
                    }
                }
            }

            impl<ED> Adc<$ADC, ED> {
                /// Save current ADC config
                pub fn save_cfg(&mut self) -> StoredConfig {
                    StoredConfig(self.get_sample_time(), self.get_resolution(), self.get_lshift())
                }

                /// Restore saved ADC config
                pub fn restore_cfg(&mut self, cfg: StoredConfig) {
                    self.set_sample_time(cfg.0);
                    self.set_resolution(cfg.1);
                    self.set_lshift(cfg.2);
                }

                /// Reset the ADC config to default, return existing config
                pub fn default_cfg(&mut self) -> StoredConfig {
                    let cfg = self.save_cfg();
                    self.set_sample_time(AdcSampleTime::default());
                    self.set_resolution(Resolution::SixteenBit);
                    self.set_lshift(AdcLshift::default());
                    cfg
                }

                /// The current ADC clock frequency. Defined as f_ADC in device datasheets
                ///
                /// The value returned by this method will always be equal or
                /// lower than the `f_adc` passed to [`init`](#method.init)
                pub fn clock_frequency(&self) -> Hertz {
                    self.clock
                }

                /// The current ADC sampling frequency. This is the reciprocal of Tconv
                pub fn sampling_frequency(&self) -> Hertz {
                    let sample_cycles_x2 = self.sample_time.clock_cycles_x2();

                    // TODO: Exception for RM0468 ADC3
                    // let sar_cycles_x2 = match self.resolution {
                    //     Resolution::SixBit => 13, // 6.5
                    //     Resolution::EightBit => 17, // 8.5
                    //     Resolution::TenBit => 21,   // 10.5
                    //     _ => 25,                    // 12.5
                    // };

                    let sar_cycles_x2 = match self.resolution {
                        Resolution::EightBit => 9, // 4.5
                        Resolution::TenBit => 11,  // 5.5
                        Resolution::TwelveBit => 13, // 6.5
                        Resolution::FourteenBit => 15, // 7.5
                        _ => 17,                       // 8.5
                    };

                    let cycles = (sample_cycles_x2 + sar_cycles_x2) / 2;
                    self.clock / cycles
                }

                /// Get ADC samping time
                pub fn get_sample_time(&self) -> AdcSampleTime {
                    self.sample_time
                }

                /// Get ADC sampling resolution
                pub fn get_resolution(&self) -> Resolution {
                    self.resolution
                }

                /// Get ADC lshift value
                pub fn get_lshift(&self) -> AdcLshift {
                    self.lshift
                }

                /// Set ADC sampling time
                ///
                /// Options can be found in [AdcSampleTime](crate::adc::AdcSampleTime).
                pub fn set_sample_time(&mut self, t_samp: AdcSampleTime) {
                    self.sample_time = t_samp;
                }

                /// Set ADC sampling resolution
                pub fn set_resolution(&mut self, res: Resolution) {
                    self.resolution = res;
                }

                /// Set ADC lshift
                ///
                /// LSHIFT\[3:0\] must be in range of 0..=15
                pub fn set_lshift(&mut self, lshift: AdcLshift) {
                    self.lshift = lshift;
                }

                /// Returns the largest possible sample value for the current ADC configuration
                ///
                /// Using this value as the denominator when calculating
                /// transfer functions results in a gain error, and thus should
                /// be avoided. Use the [slope](#method.slope) method instead.
                #[deprecated(since = "0.12.0", note = "See the slope() method instead")]
                pub fn max_sample(&self) -> u32 {
                    ((1 << self.get_resolution().number_of_bits() as u32) - 1) << self.get_lshift().value() as u32
                }

                /// Returns the slope for the current ADC configuration. 1 LSB = Vref / slope
                ///
                /// This value can be used in calcuations involving the transfer function of
                /// the ADC. For example, to calculate an estimate for the
                /// applied voltage of an ADC channel referenced to voltage
                /// `vref`
                ///
                /// ```
                /// let v = adc.read(&ch).unwrap() as f32 * vref / adc.slope() as f32;
                /// ```
                pub fn slope(&self) -> u32 {
                    1 << (self.get_resolution().number_of_bits() as u32 + self.get_lshift().value() as u32)
                }


                /// Returns the offset calibration value for single ended channel
                pub fn read_offset_calibration_value(&self) -> AdcCalOffset {
                    AdcCalOffset(self.rb.calfact.read().calfact_s().bits())
                }

                /// Returns the linear calibration values stored in an array in the following order:
                /// LINCALRDYW1 -> result\[0\]
                /// ...
                /// LINCALRDYW6 -> result\[5\]
                pub fn read_linear_calibration_values(&mut self) -> AdcCalLinear {
                    // Refer to RM0433 Rev 7 - Chapter 25.4.8
                    self.check_linear_read_conditions();

                    // Read 1st block of linear correction
                    self.rb.cr.modify(|_, w| w.lincalrdyw1().clear_bit());
                    while self.rb.cr.read().lincalrdyw1().bit_is_set() {}
                    let res_1 = self.rb.calfact2.read().lincalfact().bits();

                    // Read 2nd block of linear correction
                    self.rb.cr.modify(|_, w| w.lincalrdyw2().clear_bit());
                    while self.rb.cr.read().lincalrdyw2().bit_is_set() {}
                    let res_2 = self.rb.calfact2.read().lincalfact().bits();

                    // Read 3rd block of linear correction
                    self.rb.cr.modify(|_, w| w.lincalrdyw3().clear_bit());
                    while self.rb.cr.read().lincalrdyw3().bit_is_set() {}
                    let res_3 = self.rb.calfact2.read().lincalfact().bits();

                    // Read 4th block of linear correction
                    self.rb.cr.modify(|_, w| w.lincalrdyw4().clear_bit());
                    while self.rb.cr.read().lincalrdyw4().bit_is_set() {}
                    let res_4 = self.rb.calfact2.read().lincalfact().bits();

                    // Read 5th block of linear correction
                    self.rb.cr.modify(|_, w| w.lincalrdyw5().clear_bit());
                    while self.rb.cr.read().lincalrdyw5().bit_is_set() {}
                    let res_5 = self.rb.calfact2.read().lincalfact().bits();

                    // Read 6th block of linear correction
                    self.rb.cr.modify(|_, w| w.lincalrdyw6().clear_bit());
                    while self.rb.cr.read().lincalrdyw6().bit_is_set() {}
                    let res_6 = self.rb.calfact2.read().lincalfact().bits();

                    AdcCalLinear([res_1, res_2, res_3, res_4, res_5, res_6])
                }

                fn check_linear_read_conditions(&self) {
                    let cr = self.rb.cr.read();
                    // Ensure the ADC is enabled and is not in deeppowerdown-mode
                    if cr.deeppwd().bit_is_set() {
                        panic!("Cannot read linear calibration value when the ADC is in deeppowerdown-mode");
                    }
                    if cr.advregen().bit_is_clear() {
                        panic!("Cannot read linear calibration value when the voltage regulator is disabled");
                    }
                    if cr.aden().bit_is_clear() {
                        panic!("Cannot read linear calibration value when the ADC is disabled");
                    }
                }

                /// Returns a reference to the inner peripheral
                pub fn inner(&self) -> &$ADC {
                    &self.rb
                }

                /// Returns a mutable reference to the inner peripheral
                pub fn inner_mut(&mut self) -> &mut $ADC {
                    &mut self.rb
                }
            }

            impl<WORD, PIN> OneShot<$ADC, WORD, PIN> for Adc<$ADC, Enabled>
            where
                WORD: From<u32>,
                PIN: Channel<$ADC, ID = u8>,
            {
                type Error = ();

                fn read(&mut self, pin: &mut PIN) -> nb::Result<WORD, Self::Error> {
                    self.start_conversion(pin);
                    let res = block!(self.read_sample()).unwrap();
                    Ok(res.into())
                }
            }
        )+
    }
}

adc_hal!(
    ADC1,
    ADC12_COMMON: (adc1, Adc12, ldordy),
    ADC2,
    ADC12_COMMON: (adc2, Adc12, ldordy),
);

#[cfg(any(feature = "rm0433", feature = "rm0399"))]
adc_hal!(ADC3, ADC3_COMMON: (adc3, Adc3, ldordy));
#[cfg(feature = "rm0468")]
adc_hal!(ADC3, ADC3_COMMON: (adc3, Adc3));
