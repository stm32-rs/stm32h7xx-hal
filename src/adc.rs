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

use crate::hal::adc::{Channel, OneShot};
use crate::hal::blocking::delay::DelayUs;

use core::convert::Infallible;
use core::marker::PhantomData;

use nb::block;

#[cfg(feature = "rm0455")]
use crate::stm32::ADC12_COMMON;
use crate::stm32::{ADC1, ADC2};
#[cfg(not(feature = "rm0455"))]
use crate::stm32::{ADC3, ADC3_COMMON};

use crate::gpio::{self, Analog};
use crate::rcc::rec::AdcClkSelGetter;
use crate::rcc::{rec, CoreClocks, ResetEnable};
use crate::time::Hertz;

#[cfg(not(feature = "revision_v"))]
const ADC_KER_CK_MAX: u32 = 36_000_000;

#[cfg(feature = "revision_v")]
const ADC_KER_CK_MAX: u32 = 100_000_000;

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
            Resolution::EIGHTBIT => 8,
            Resolution::TENBIT => 10,
            Resolution::TWELVEBIT => 12,
            Resolution::FOURTEENBIT => 14,
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
    current_channel: Option<u8>,
    _enabled: PhantomData<ED>,
}

/// ADC sampling time
///
/// Options for the sampling time, each is T + 0.5 ADC clock cycles.
//
// Refer to RM0433 Rev 6 - Chapter 24.4.13
#[derive(Clone, Copy, Debug, PartialEq)]
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

impl AdcSampleTime {
    pub fn default() -> Self {
        AdcSampleTime::T_32
    }
}

// Refer to RM0433 Rev 6 - Chapter 24.4.13
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

/// ADC LSHIFT\[3:0\] of the converted value
///
/// Only values in range of 0..=15 are allowed.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AdcLshift(u8);

impl AdcLshift {
    pub fn new(lshift: u8) -> Self {
        if lshift > 15 {
            panic!("LSHIFT[3:0] must be in range of 0..=15");
        }

        AdcLshift(lshift)
    }

    pub fn default() -> Self {
        AdcLshift(0)
    }

    pub fn value(self) -> u8 {
        self.0
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AdcCalOffset(u16);

impl AdcCalOffset {
    pub fn value(self) -> u16 {
        self.0
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
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

#[cfg(not(feature = "rm0455"))]
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
    gpio::PH5<Analog> => 16,
);
#[cfg(not(feature = "rm0455"))]
adc_internal!(
    [ADC3, ADC3_COMMON];

    Vbat => (17, vbaten),
    Temperature => (18, vsenseen),
    Vrefint => (19, vrefen)
);

#[cfg(feature = "rm0455")]
adc_internal!(
    [ADC2, ADC12_COMMON];

    Vbat => (14, vbaten),
    Temperature => (18, vsenseen),
    Vrefint => (19, vrefen)
);

pub trait AdcExt<ADC>: Sized {
    type Rec: ResetEnable;

    fn adc(
        self,
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

/// Get and check the adc_ker_ck_input
fn check_clock(prec: &impl AdcClkSelGetter, clocks: &CoreClocks) -> Hertz {
    // Select Kernel Clock
    let adc_clock = match prec.get_kernel_clk_mux() {
        Some(rec::AdcClkSel::PLL2_P) => {
            clocks.pll2_p_ck().expect("ADC: PLL2_P must be enabled")
        }
        Some(rec::AdcClkSel::PLL3_R) => {
            clocks.pll3_r_ck().expect("ADC: PLL3_R must be enabled")
        }
        Some(rec::AdcClkSel::PER) => {
            clocks.per_ck().expect("ADC: PER clock must be enabled")
        }
        _ => unreachable!(),
    };

    // Check against datasheet requirements
    assert!(
        adc_clock.0 <= ADC_KER_CK_MAX,
        "adc_ker_ck_input is too fast"
    );

    adc_clock
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
    delay: &mut impl DelayUs<u8>,
    prec: rec::Adc12,
    clocks: &CoreClocks,
) -> (Adc<ADC1, Disabled>, Adc<ADC2, Disabled>) {
    // Consume ADC register block, produce ADC1/2 with default settings
    let mut adc1 = Adc::<ADC1, Disabled>::default_from_rb(adc1);
    let mut adc2 = Adc::<ADC2, Disabled>::default_from_rb(adc2);

    // Check adc_ker_ck_input
    check_clock(&prec, clocks);

    // Enable AHB clock
    let prec = prec.enable();

    // Power Down
    adc1.power_down();
    adc2.power_down();

    // Reset peripheral
    prec.reset();

    // Power Up, Preconfigure and Calibrate
    adc1.power_up(delay);
    adc2.power_up(delay);
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
        $ADC:ident: (
            $adcX: ident,
            $Rec:ident
        )
    ),+ $(,)*) => {
        $(
            impl AdcExt<$ADC> for $ADC {
                type Rec = rec::$Rec;

	            fn adc(self,
                       delay: &mut impl DelayUs<u8>,
                       prec: rec::$Rec,
                       clocks: &CoreClocks) -> Adc<$ADC, Disabled>
	            {
	                Adc::$adcX(self, delay, prec, clocks)
	            }
	        }

            impl Adc<$ADC, Disabled> {
                /// Initialise ADC
                ///
                /// Sets all configurable parameters to one-shot defaults,
                /// performs a boot-time calibration.
                pub fn $adcX(adc: $ADC, delay: &mut impl DelayUs<u8>,
                             prec: rec::$Rec, clocks: &CoreClocks
                ) -> Self {
                    // Consume ADC register block, produce Self with default
                    // settings
                    let mut adc = Self::default_from_rb(adc);

                    // Check adc_ker_ck_input
                    check_clock(&prec, clocks);

                    // Enable AHB clock
                    let prec = prec.enable();

                    // Power Down
                    adc.power_down();

                    // Reset peripheral
                    prec.reset();

                    // Power Up, Preconfigure and Calibrate
                    adc.power_up(delay);
                    adc.preconfigure();
                    adc.calibrate();

                    adc
                }
                /// Creates ADC with default settings
                fn default_from_rb(rb: $ADC) -> Self {
                    Self {
                        rb,
                        sample_time: AdcSampleTime::default(),
                        resolution: Resolution::SIXTEENBIT,
                        lshift: AdcLshift::default(),
                        current_channel: None,
                        _enabled: PhantomData,
                    }
                }
                /// Disables Deeppowerdown-mode and enables voltage regulator
                ///
                /// Note: After power-up, a [`calibration`](#method.calibrate) shall be run
                pub fn power_up(&mut self, delay: &mut impl DelayUs<u8>) {
                    // Refer to RM0433 Rev 6 - Chapter 24.4.6
                    self.rb.cr.modify(|_, w|
                        w.deeppwd().clear_bit()
                            .advregen().set_bit()
                    );
                    delay.delay_us(10_u8);
                }

                /// Enables Deeppowerdown-mode and disables voltage regulator
                ///
                /// Note: This resets the [`calibration`](#method.calibrate) of the ADC
                pub fn power_down(&mut self) {
                    // Refer to RM0433 Rev 6 - Chapter 24.4.6
                    self.rb.cr.modify(|_, w|
                        w.deeppwd().set_bit()
                            .advregen().clear_bit()
                    );
                }

                /// Calibrates the ADC in single channel mode
                ///
                /// Note: The ADC must be disabled
                pub fn calibrate(&mut self) {
                    // Refer to RM0433 Rev 6 - Chapter 24.4.8
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
                    if self.rb.cr.read().aden().bit_is_set() {
                        panic!("Cannot start calibration when the ADC is enabled");
                    }
                    if self.rb.cr.read().deeppwd().bit_is_set() {
                        panic!("Cannot start calibration when the ADC is in deeppowerdown-mode");
                    }
                    if self.rb.cr.read().advregen().bit_is_clear() {
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
                    // Refer to RM0433 Rev 6 - Chapters 24.4.15, 24.4.19
                    self.rb.cfgr.modify(|_, w|
                        w.cont().clear_bit()
                            .exten().disabled()
                            .discen().set_bit()
                    );

                    // Enables boost mode for highest possible clock frequency
                    //
                    // Refer to RM0433 Rev 6 - Chapter 24.4.3
                    #[cfg(not(feature = "revision_v"))]
                    self.rb.cr.modify(|_, w| w.boost().set_bit());
                    #[cfg(feature = "revision_v")]
                    self.rb.cr.modify(|_, w| w.boost().lt50());
                }

                /// Enable ADC
                pub fn enable(mut self) -> Adc<$ADC, Enabled> {
                    // Refer to RM0433 Rev 6 - Chapter 24.4.9
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
                    match chan {
                        0 => self.rb.smpr1.modify(|_, w| w.smp0().bits(self.get_sample_time().into())),
                        1 => self.rb.smpr1.modify(|_, w| w.smp1().bits(self.get_sample_time().into())),
                        2 => self.rb.smpr1.modify(|_, w| w.smp2().bits(self.get_sample_time().into())),
                        3 => self.rb.smpr1.modify(|_, w| w.smp3().bits(self.get_sample_time().into())),
                        4 => self.rb.smpr1.modify(|_, w| w.smp4().bits(self.get_sample_time().into())),
                        5 => self.rb.smpr1.modify(|_, w| w.smp5().bits(self.get_sample_time().into())),
                        6 => self.rb.smpr1.modify(|_, w| w.smp6().bits(self.get_sample_time().into())),
                        7 => self.rb.smpr1.modify(|_, w| w.smp7().bits(self.get_sample_time().into())),
                        8 => self.rb.smpr1.modify(|_, w| w.smp8().bits(self.get_sample_time().into())),
                        9 => self.rb.smpr1.modify(|_, w| w.smp9().bits(self.get_sample_time().into())),
                        10 => self.rb.smpr2.modify(|_, w| w.smp10().bits(self.get_sample_time().into())),
                        11 => self.rb.smpr2.modify(|_, w| w.smp11().bits(self.get_sample_time().into())),
                        12 => self.rb.smpr2.modify(|_, w| w.smp12().bits(self.get_sample_time().into())),
                        13 => self.rb.smpr2.modify(|_, w| w.smp13().bits(self.get_sample_time().into())),
                        14 => self.rb.smpr2.modify(|_, w| w.smp14().bits(self.get_sample_time().into())),
                        15 => self.rb.smpr2.modify(|_, w| w.smp15().bits(self.get_sample_time().into())),
                        16 => self.rb.smpr2.modify(|_, w| w.smp16().bits(self.get_sample_time().into())),
                        17 => self.rb.smpr2.modify(|_, w| w.smp17().bits(self.get_sample_time().into())),
                        18 => self.rb.smpr2.modify(|_, w| w.smp18().bits(self.get_sample_time().into())),
                        19 => self.rb.smpr2.modify(|_, w| w.smp19().bits(self.get_sample_time().into())),
                        _ => unreachable!(),
                    }
                }

                /// Start conversion
                ///
                /// This method will start reading sequence on the given pin.
                /// The value can be then read through the `read_sample` method.
                // Refer to RM0433 Rev 6 - Chapter 24.4.16
                pub fn start_conversion<PIN>(&mut self, _pin: &mut PIN)
                    where PIN: Channel<$ADC, ID = u8>,
                {
                    let chan = PIN::channel();
                    assert!(chan <= 19);

                    self.check_conversion_conditions();

                    // Set resolution
                    self.rb.cfgr.modify(|_, w| unsafe { w.res().bits(self.get_resolution().into()) });

                    // Set LSHIFT[3:0]
                    self.rb.cfgr2.modify(|_, w| w.lshift().bits(self.get_lshift().value()));

                    // Select channel (with preselection, refer to RM0433 Rev 6 - Chapter 24.4.12)
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

                /// Read sample
                ///
                /// `nb::Error::WouldBlock` in case the conversion is still
                /// progressing.
                // Refer to RM0433 Rev 6 - Chapter 24.4.16
                pub fn read_sample(&mut self) -> nb::Result<u32, Infallible> {
                    let chan = self.current_channel.expect("No channel was selected, use start_conversion first");

                    // Check if the conversion is finished
                    if self.rb.isr.read().eoc().bit_is_clear() {
                        return Err(nb::Error::WouldBlock);
                    }

                    // Disable preselection of this channel, refer to RM0433 Rev 6 - Chapter 24.4.12
                    self.rb.pcsel.modify(|r, w| unsafe { w.pcsel().bits(r.pcsel().bits() & !(1 << chan)) });
                    self.current_channel = None;

                    // Retrieve result
                    let result = self.rb.dr.read().bits();
                    nb::Result::Ok(result)
                }

                fn check_conversion_conditions(&self) {
                    // Ensure that no conversions are ongoing
                    if self.rb.cr.read().adstart().bit_is_set() {
                        panic!("Cannot start conversion because a regular conversion is ongoing");
                    }
                    if self.rb.cr.read().jadstart().bit_is_set() {
                        panic!("Cannot start conversion because an injected conversion is ongoing");
                    }
                    // Ensure that the ADC is enabled
                    if self.rb.cr.read().aden().bit_is_clear() {
                        panic!("Cannot start conversion because ADC is currently disabled");
                    }
                    if self.rb.cr.read().addis().bit_is_set() {
                        panic!("Cannot start conversion because there is a pending request to disable the ADC");
                    }
                }

                /// Disable ADC
                pub fn disable(mut self) -> Adc<$ADC, Disabled> {
                    // Refer to RM0433 Rev 6 - Chapter 24.4.9
                    if self.rb.cr.read().adstart().bit_is_set() {
                        self.stop_regular_conversion();
                    }
                    if self.rb.cr.read().jadstart().bit_is_set() {
                        self.stop_injected_conversion();
                    }

                    self.rb.cr.modify(|_, w| w.addis().set_bit());
                    while self.rb.cr.read().aden().bit_is_set() {}

                    Adc {
                        rb: self.rb,
                        sample_time: self.sample_time,
                        resolution: self.resolution,
                        lshift: self.lshift,
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
                    self.set_resolution(Resolution::SIXTEENBIT);
                    self.set_lshift(AdcLshift::default());
                    cfg
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

                /// Returns the largest possible sample value for the current settings
                pub fn max_sample(&self) -> u32 {
                    ((1 << self.get_resolution().number_of_bits() as u32) - 1) << self.get_lshift().value() as u32
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
                    // Refer to RM0433 Rev 6 - Chapter 24.4.8 (Page 920)
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
                    // Ensure the ADC is enabled and is not in deeppowerdown-mode
                    if self.rb.cr.read().deeppwd().bit_is_set() {
                        panic!("Cannot read linear calibration value when the ADC is in deeppowerdown-mode");
                    }
                    if self.rb.cr.read().advregen().bit_is_clear() {
                        panic!("Cannot read linear calibration value when the voltage regulator is disabled");
                    }
                    if self.rb.cr.read().aden().bit_is_clear() {
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
    ADC1: (adc1, Adc12), // ADC1
    ADC2: (adc2, Adc12), // ADC2
);

#[cfg(not(feature = "rm0455"))]
adc_hal!(ADC3: (adc3, Adc3));
