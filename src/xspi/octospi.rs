//! Octo SPI (OCTOSPI) bus
//!
//! See the parent module for documentation

use crate::{
    gpio::{
        gpioa::{PA1, PA2, PA3, PA6, PA7},
        gpiob::{PB0, PB1, PB10, PB12, PB13, PB2, PB6},
        gpioc::{PC1, PC10, PC11, PC2, PC3, PC5, PC9},
        gpiod::{PD11, PD12, PD13, PD4, PD5, PD6, PD7},
        gpioe::{PE10, PE11, PE2, PE7, PE8, PE9},
        gpiof::{
            PF0, PF1, PF10, PF11, PF12, PF2, PF3, PF4, PF5, PF6, PF7, PF8, PF9,
        },
        gpiog::{PG0, PG1, PG10, PG11, PG12, PG14, PG15, PG6, PG7, PG9},
        gpioh::{PH2, PH3},
        Alternate, AF0, AF10, AF11, AF12, AF3, AF4, AF6, AF9,
    },
    rcc::{rec, CoreClocks, ResetEnable},
    stm32,
};

use super::{Config, Octospi, SamplingEdge};

pub trait PinClk<OSPI> {}
pub trait PinNclk<OSPI> {}
pub trait PinDQS<OSPI> {}
pub trait PinNCS<OSPI> {}
pub trait PinIo0<OSPI> {}
pub trait PinIo1<OSPI> {}
pub trait PinIo2<OSPI> {}
pub trait PinIo3<OSPI> {}
pub trait PinIo4<OSPI> {}
pub trait PinIo5<OSPI> {}
pub trait PinIo6<OSPI> {}
pub trait PinIo7<OSPI> {}

// impl<OSPI, CLK, NCS, IO0, IO1, IO2, IO3> PinsQuad<OSPI>
//     for (CLK, NCS, IO0, IO1, IO2, IO3)
// where
//     CLK: PinClk<OSPI>,
//     NCS: PinNCS<OSPI>,
//     IO0: PinIo0<OSPI>,
//     IO1: PinIo1<OSPI>,
//     IO2: PinIo2<OSPI>,
//     IO3: PinIo3<OSPI>,
// {
// }

macro_rules! pins {
    (
        $($BANK:ident:
            CLK: [$($CLK:ty),*] NCLK: [$($NCLK:ty),*] DQS: [$($DQS:ty),*] NCS: [$($NCS:ty),*]
                IO0: [$($IO0:ty),*] IO1: [$($IO1:ty),*] IO2: [$($IO2:ty),*] IO3: [$($IO3:ty),*]
                IO4: [$($IO4:ty),*] IO5: [$($IO5:ty),*] IO6: [$($IO6:ty),*] IO7: [$($IO7:ty),*]
        )*
    ) => {
        $(
            $(
                impl PinClk<stm32::$BANK> for $CLK {}
            )*
            $(
                impl PinNclk<stm32::$BANK> for $NCLK {}
            )*
            $(
                impl PinDQS<stm32::$BANK> for $DQS {}
            )*
            $(
                impl PinNCS<stm32::$BANK> for $NCS {}
            )*
            $(
                impl PinIo0<stm32::$BANK> for $IO0 {}
            )*
            $(
                impl PinIo1<stm32::$BANK> for $IO1 {}
            )*
            $(
                impl PinIo2<stm32::$BANK> for $IO2 {}
            )*
            $(
                impl PinIo3<stm32::$BANK> for $IO3 {}
            )*
            $(
                impl PinIo4<stm32::$BANK> for $IO4 {}
            )*
            $(
                impl PinIo5<stm32::$BANK> for $IO5 {}
            )*
            $(
                impl PinIo6<stm32::$BANK> for $IO6 {}
            )*
            $(
                impl PinIo7<stm32::$BANK> for $IO7 {}
            )*
        )*
    };
}

#[cfg(any(feature = "rm0468"))] // TODO
pins! {
    OCTOSPI1:
        CLK: [
            PA3<Alternate<AF12>>,
            PB2<Alternate<AF9>>,
            PF10<Alternate<AF9>>
        ]
        NCLK: [
            PB12<Alternate<AF3>>,
            PF11<Alternate<AF9>>
        ]
        DQS: [
            PA1<Alternate<AF12>>,
            PB2<Alternate<AF10>>,
            PC5<Alternate<AF10>>
        ]
        NCS: [
            PB6<Alternate<AF10>>,
            PB10<Alternate<AF9>>,
            PC11<Alternate<AF9>>,
            PE11<Alternate<AF11>>,
            PG6<Alternate<AF10>>
        ]
        IO0: [
            PA2<Alternate<AF6>>,
            PB1<Alternate<AF4>>,
            PB12<Alternate<AF12>>,
            PC3<Alternate<AF9>>,
            PC3<Alternate<AF0>>,
            PC9<Alternate<AF9>>,
            PD11<Alternate<AF9>>,
            PF8<Alternate<AF10>>
        ]
        IO1: [
            PB0<Alternate<AF4>>,
            PC10<Alternate<AF9>>,
            PD12<Alternate<AF9>>,
            PF9<Alternate<AF10>>
        ]
        IO2: [
            PA3<Alternate<AF6>>,
            PA7<Alternate<AF10>>,
            PB13<Alternate<AF4>>,
            PC2<Alternate<AF9>>,
            PC2<Alternate<AF0>>,
            PE2<Alternate<AF9>>,
            PF7<Alternate<AF10>>
        ]
        IO3: [
            PA1<Alternate<AF9>>,
            PA6<Alternate<AF6>>,
            PD13<Alternate<AF9>>,
            PF6<Alternate<AF10>>
        ]
        IO4: [
            PC1<Alternate<AF10>>,
            PD4<Alternate<AF10>>,
            PE7<Alternate<AF10>>,
            PH2<Alternate<AF9>>
        ]
        IO5: [
            PC2<Alternate<AF4>>,
            PC2<Alternate<AF0>>,
            PD5<Alternate<AF10>>,
            PE8<Alternate<AF10>>,
            PH3<Alternate<AF9>>
        ]
        IO6: [
            PC3<Alternate<AF4>>,
            PC3<Alternate<AF0>>,
            PD6<Alternate<AF10>>,
            PE9<Alternate<AF10>>,
            PG9<Alternate<AF9>>
        ]
        IO7: [
            PD7<Alternate<AF10>>,
            PE10<Alternate<AF10>>,
            PG14<Alternate<AF9>>
        ]
}
pins! {
    OCTOSPI2:
        CLK: [
            PF4<Alternate<AF9>>
        ]
        NCLK: [
            PF5<Alternate<AF9>>
        ]
        DQS: [
            PF12<Alternate<AF9>>,
            PG7<Alternate<AF9>>,
            PG15<Alternate<AF9>>
        ]
        NCS: [
            PG12<Alternate<AF3>>
        ]
        IO0: [
            PF0<Alternate<AF9>>
        ]
        IO1: [
            PF1<Alternate<AF9>>
        ]
        IO2: [
            PF2<Alternate<AF9>>
        ]
        IO3: [
            PF3<Alternate<AF9>>
        ]
        IO4: [
            PG0<Alternate<AF9>>
        ]
        IO5: [
            PG1<Alternate<AF9>>
        ]
        IO6: [
            PG10<Alternate<AF3>>
        ]
        IO7: [
            PG11<Alternate<AF9>>
        ]
}

pub trait OctospiExt<OSPI>: Sized {
    /// The `ResetEnable` singleton for this peripheral
    type Rec: ResetEnable;

    /// Create and enable the Octospi peripheral
    fn octospi_unchecked<CONFIG>(
        self,
        config: CONFIG,
        clocks: &CoreClocks,
        prec: Self::Rec,
    ) -> Octospi<OSPI>
    where
        CONFIG: Into<Config>;
}

macro_rules! octospi_impl {
    ($name:ident, $peripheral:ty, $rec:ty) => {
        impl Octospi<$peripheral> {
            pub fn $name<CONFIG>(
                regs: $peripheral,
                config: CONFIG,
                clocks: &CoreClocks,
                prec: $rec,
            ) -> Self
            where
                CONFIG: Into<Config>,
            {
                prec.enable();

                // Disable OCTOSPI before configuring it.
                regs.cr.write(|w| w.en().clear_bit());

                let spi_kernel_ck = match Self::get_clock(clocks) {
                    Some(freq_hz) => freq_hz.0,
                    _ => panic!("OCTOSPI kernel clock not running!"),
                };

                while regs.sr.read().busy().bit_is_set() {}

                let config: Config = config.into();

                // Clear all pending flags.
                regs.fcr.write(|w| {
                    w.ctof()
                        .set_bit()
                        .csmf()
                        .set_bit()
                        .ctcf()
                        .set_bit()
                        .ctef()
                        .set_bit()
                });

                // Configure the communication method for OCTOSPI.
                regs.cr.write(|w| unsafe {
                    w.fmode()
                        .bits(0) // indirect mode
                        .fthres()
                        .bits(config.fifo_threshold - 1)
                });

                regs.dcr1.write(|w| unsafe {
                    w.mtyp()
                        .bits(2) // standard mode
                        // Configure the FSIZE to maximum. It appears that even when addressing
                        // is not used, the flash size violation may still trigger.
                        .devsize()
                        .bits(0x1F)
                });

                // Communications configuration register
                regs.ccr.write(|w| unsafe {
                    w.dmode()
                        .bits(config.mode.reg_value())
                        .admode()
                        .bits(config.mode.reg_value())
                        .adsize()
                        .bits(0) // Eight-bit address
                        .imode()
                        .bits(0) // No instruction phase
                });

                // Prescaler
                let spi_frequency = config.frequency.0;
                let divisor =
                    match (spi_kernel_ck + spi_frequency - 1) / spi_frequency {
                        divisor @ 1..=256 => divisor - 1,
                        _ => panic!("Invalid OCTOSPI frequency requested"),
                    };
                regs.dcr2
                    .write(|w| unsafe { w.prescaler().bits(divisor as u8) });

                // Note that we default to setting SSHIFT (sampling on the falling
                // edge). This is because it appears that the QSPI may have signal
                // contention issues when reading with zero dummy cycles. Setting SSHIFT
                // forces the read to occur on the falling edge instead of the rising
                // edge. Refer to https://github.com/quartiq/stabilizer/issues/101 for
                // more information
                //
                // SSHIFT must not be set in DDR mode.
                regs.tcr.write(|w| unsafe {
                    w.sshift()
                        .bit(config.sampling_edge == SamplingEdge::Falling)
                        .dcyc()
                        .bits(config.dummy_cycles)
                });

                // Enable the peripheral
                regs.cr.modify(|_, w| w.en().set_bit());

                Octospi {
                    rb: regs,
                    mode: config.mode,
                }
            }
        }

        impl OctospiExt<$peripheral> for $peripheral {
            type Rec = $rec;

            fn octospi_unchecked<CONFIG>(
                self,
                config: CONFIG,
                clocks: &CoreClocks,
                prec: Self::Rec,
            ) -> Octospi<$peripheral>
            where
                CONFIG: Into<Config>,
            {
                Octospi::$name(self, config, clocks, prec)
            }
        }
    };
}

#[cfg(any(feature = "rm0468"))] // TODO feature = "rm0455"
octospi_impl! { octospi1_unchecked, stm32::OCTOSPI1, rec::Octospi1 }
octospi_impl! { octospi2_unchecked, stm32::OCTOSPI2, rec::Octospi2 }
