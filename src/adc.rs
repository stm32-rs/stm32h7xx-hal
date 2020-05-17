use crate::hal::adc::{Channel, OneShot};
use crate::hal::blocking::delay::DelayUs;

use core::marker::PhantomData;

use crate::stm32;
use crate::stm32::{ADC1, ADC2, ADC3, ADC3_COMMON};

use crate::delay::Delay;
use crate::gpio::gpioa::{PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7};
use crate::gpio::gpiob::{PB0, PB1};
use crate::gpio::gpioc::{PC0, PC1, PC2, PC3, PC4, PC5};
use crate::gpio::gpiof::{
    PF10, PF11, PF12, PF13, PF14, PF3, PF4, PF5, PF6, PF7, PF8, PF9,
};
use crate::gpio::gpioh::{PH2, PH3, PH4, PH5};
use crate::gpio::Analog;
use crate::rcc::{rec, CoreClocks, ResetEnable};

#[cfg(not(feature = "revision_v"))]
const ADC_KER_CK_MAX: u32 = 36_000_000;

#[cfg(feature = "revision_v")]
const ADC_KER_CK_MAX: u32 = 100_000_000;

pub type Resolution = crate::stm32::adc3::cfgr::RES_A;
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
    _enabled: PhantomData<ED>,
}

/// ADC sampling time
///
/// Options for the sampling time, each is T + 0.5 ADC clock cycles.
//
// Refer to RM0433 Rev 6 - Chapter 24.4.13
#[derive(Clone, Copy, Debug, PartialEq)]
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
pub struct AdcCalOffset(u16);

impl AdcCalOffset {
    pub fn value(self) -> u16 {
        self.0
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
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
    ($($input:ty => ($chan:expr, $en:ident)),+ $(,)*) => {
        $(
            impl $input {
                pub fn new() -> Self {
                    Self {}
                }

                /// Enables the internal voltage/sensor
                /// ADC must be disabled.
                pub fn enable(&mut self, _adc: &Adc<ADC3, Disabled>) {

                    let common = unsafe { &*ADC3_COMMON::ptr() };

                    common.ccr.modify(|_, w| w.$en().enabled());
                }
                /// Disables the internal voltage/sdissor
                /// ADC must be disabled.
                pub fn disable(&mut self, _adc: &Adc<ADC3, Disabled>) {

                    let common = unsafe { &*ADC3_COMMON::ptr() };

                    common.ccr.modify(|_, w| w.$en().disabled());
                }
            }

            adc_pins!(ADC3, $input => $chan);
        )+
    };
}

/// Vref internal signal
pub struct Vrefint;
/// Vbat internal signal
pub struct Vbat;
/// Internal temperature sensor
pub struct Temperature;

// Not implementing Pxy_C adc pins
// Just implmenting INPx pins (INNx defaulting to V_ref-)
//
// Refer to DS12110 Rev 7 - Chapter 5 (Table 9)
adc_pins!(ADC1,
          // 0, 1 are Pxy_C pins
          PF11<Analog> => 2,
          PA6<Analog> => 3,
          PC4<Analog> => 4,
          PB1<Analog> => 5,
          PF12<Analog> => 6,
          PA7<Analog> => 7,
          PC5<Analog> => 8,
          PB0<Analog> => 9,
          PC0<Analog> => 10,
          PC1<Analog> => 11,
          PC2<Analog> => 12,
          PC3<Analog> => 13,
          PA2<Analog> => 14,
          PA3<Analog> => 15,
          PA0<Analog> => 16,
          PA1<Analog> => 17,
          PA4<Analog> => 18,
          PA5<Analog> => 19,
);

adc_pins!(ADC2,
          // 0, 1 are Pxy_C pins
          PF13<Analog> => 2,
          PA6<Analog> => 3,
          PC4<Analog> => 4,
          PB1<Analog> => 5,
          PF14<Analog> => 6,
          PA7<Analog> => 7,
          PC5<Analog> => 8,
          PB0<Analog> => 9,
          PC0<Analog> => 10,
          PC1<Analog> => 11,
          PC2<Analog> => 12,
          PC3<Analog> => 13,
          PA2<Analog> => 14,
          PA3<Analog> => 15,
          // 16, 17 are dac_outX
          PA4<Analog> => 18,
          PA5<Analog> => 19,
);

adc_pins!(ADC3,
          // 0, 1 are Pxy_C pins
          PF9<Analog> => 2,
          PF7<Analog> => 3,
          PF5<Analog> => 4,
          PF3<Analog> => 5,
          PF10<Analog> => 6,
          PF8<Analog> => 7,
          PF6<Analog> => 8,
          PF4<Analog> => 9,
          PC0<Analog> => 10,
          PC1<Analog> => 11,
          PC2<Analog> => 12,
          PH2<Analog> => 13,
          PH3<Analog> => 14,
          PH4<Analog> => 15,
          PH5<Analog> => 16,
);
adc_internal!(
          Vbat => (17, vbaten),
          Temperature => (18, vsenseen),
          Vrefint => (19, vrefen)
);

pub trait AdcExt<ADC>: Sized {
    type Rec: ResetEnable;

    fn adc(
        self,
        delay: &mut Delay,
        prec: Self::Rec,
        clocks: &CoreClocks,
    ) -> Adc<ADC, Disabled>;
}

/// Stored ADC config can be restored using the `Adc::restore_cfg` method
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct StoredConfig(AdcSampleTime, Resolution, AdcLshift);

#[allow(unused_macros)]
macro_rules! adc_hal {
    ($(
        $ADC:ident: (
            $adcX: ident,
            $Rec:ident,
            $COMMON:ident
        )
    ),+ $(,)*) => {
        $(
            impl AdcExt<$ADC> for $ADC {
                type Rec = rec::$Rec;

	            fn adc(self,
                       delay: &mut Delay,
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
                pub fn $adcX(adc: $ADC, delay: &mut Delay,
                             prec: rec::$Rec, clocks: &CoreClocks
                ) -> Self {
                    let mut s = Self {
                        rb: adc,
                        sample_time: AdcSampleTime::default(),
                        resolution: Resolution::SIXTEENBIT,
                        lshift: AdcLshift::default(),
                        _enabled: PhantomData,
                    };

                    // Select Kernel Clock
                    s.enable_clock(
                        clocks.per_ck().expect("per_ck is not running!").0
                    );

                    // Enable AHB clock
                    let prec = prec.enable();

                    // Power Down
                    s.power_down();

                    // Reset periperal
                    prec.reset();

                    // Power Up, Preconfigure and Calibrate
                    s.power_up(delay);
                    s.preconfigure();
                    s.calibrate();

                    s
                }

                fn enable_clock(&mut self, per_ck: u32) {
                    // Set per_ck as adc clock, TODO: we might want to
                    // change this so we can also use other clocks as
                    // input for this
                    assert!(per_ck <= ADC_KER_CK_MAX, "per_ck is not running or too fast");
                    let d3ccipr = &unsafe { &*stm32::RCC::ptr() }.d3ccipr;

                    d3ccipr.modify(|_, w| unsafe { w.adcsel().bits(0b10) });
                }

                /// Disables Deeppowerdown-mode and enables voltage regulator
                ///
                /// Note: After power-up, a [`calibration`](#method.calibrate) shall be run
                pub fn power_up(&mut self, delay: &mut Delay) {
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

                // Refer to RM0433 Rev 6 - Chapter 24.4.16
                fn convert(&mut self, chan: u8) -> u32 {
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

                    // Perform conversion
                    self.rb.cr.modify(|_, w| w.adstart().set_bit());

                    // Wait until conversion finished
                    while self.rb.isr.read().eoc().bit_is_clear() {}

                    // Disable preselection of this channel, refer to RM0433 Rev 6 - Chapter 24.4.12
                    self.rb.pcsel.modify(|r, w| unsafe { w.pcsel().bits(r.pcsel().bits() & !(1 << chan)) });

                    // Retrieve result
                    let result = self.rb.dr.read().bits();
                    result
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
                        _enabled: PhantomData,
                    }
                }
            }

            impl<ED> Adc<$ADC, ED> {

                /// Releases the ADC peripheral
                pub fn free(self) -> ($ADC, rec::$Rec) {
                    (self.rb, rec::$Rec { _marker: PhantomData })
                }

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
            }

            impl<WORD, PIN> OneShot<$ADC, WORD, PIN> for Adc<$ADC, Enabled>
            where
                WORD: From<u32>,
                PIN: Channel<$ADC, ID = u8>,
            {
                type Error = ();

                fn read(&mut self, _pin: &mut PIN) -> nb::Result<WORD, Self::Error> {
                    let res = self.convert(PIN::channel());
                    Ok(res.into())
                }
            }
        )+
    }
}

adc_hal!(
    ADC1: (adc1, Adc12, ADC12_COMMON),
    ADC2: (adc2, Adc12, ADC12_COMMON),
    ADC3: (adc3, Adc3, ADC3_COMMON),
);
