//! Quad SPI (QSPI) bus
//!
//! See the parent module for documentation

use crate::{
    gpio::{self, Alternate},
    rcc::{rec, CoreClocks, ResetEnable},
    stm32,
};

use super::{Bank, Config, Qspi, SamplingEdge};

/// Used to indicate that an IO pin is not used by the QSPI interface.
pub struct NoIo {}

/// Indicates a set of pins can be used for the QSPI interface on bank 1.
pub trait PinsBank1 {}
pub trait PinIo0Bank1 {}
pub trait PinIo1Bank1 {}
pub trait PinIo2Bank1 {}
pub trait PinIo3Bank1 {}

/// Indicates a set of pins can be used for the QSPI interface on bank 2.
pub trait PinsBank2 {}
pub trait PinSckBank2 {}
pub trait PinIo0Bank2 {}
pub trait PinIo1Bank2 {}
pub trait PinIo2Bank2 {}
pub trait PinIo3Bank2 {}

pub trait PinSck {}

impl<SCK, IO0, IO1, IO2, IO3> PinsBank1 for (SCK, IO0, IO1, IO2, IO3)
where
    SCK: PinSck,
    IO0: PinIo0Bank1,
    IO1: PinIo1Bank1,
    IO2: PinIo2Bank1,
    IO3: PinIo3Bank1,
{
}

impl<SCK, IO0, IO1, IO2, IO3> PinsBank2 for (SCK, IO0, IO1, IO2, IO3)
where
    SCK: PinSck,
    IO0: PinIo0Bank2,
    IO1: PinIo1Bank2,
    IO2: PinIo2Bank2,
    IO3: PinIo3Bank2,
{
}

macro_rules! pins {
    (Bank1: [IO0: [$($IO0:ty),*] IO1: [$($IO1:ty),*] IO2: [$($IO2:ty),*] IO3: [$($IO3:ty),*]]) => {
        $(
            impl PinIo0Bank1 for $IO0 {}
        )*
        $(
            impl PinIo1Bank1 for $IO1 {}
        )*
        $(
            impl PinIo2Bank1 for $IO2 {}
        )*
        $(
            impl PinIo3Bank1 for $IO3 {}
        )*
    };

    (Bank2: [IO0: [$($IO0:ty),*] IO1: [$($IO1:ty),*] IO2: [$($IO2:ty),*] IO3: [$($IO3:ty),*]]) => {
        $(
            impl PinIo0Bank2 for $IO0 {}
        )*
        $(
            impl PinIo1Bank2 for $IO1 {}
        )*
        $(
            impl PinIo2Bank2 for $IO2 {}
        )*
        $(
            impl PinIo3Bank2 for $IO3 {}
        )*
    };

    (SCK: [$($SCK:ty),*], Bank1: $bank1:tt, Bank2: $bank2:tt) => {
        $(
            impl PinSck for $SCK {}
        )*
        pins!(Bank1: $bank1);
        pins!(Bank2: $bank2);
    };
}

pins! {
    SCK: [
        gpio::PB2<Alternate<9>>,
        gpio::PF10<Alternate<9>>
    ],
    Bank1: [
        IO0: [
            gpio::PC9<Alternate<9>>,
            gpio::PD11<Alternate<9>>,
            gpio::PF8<Alternate<10>>
        ]
        IO1: [
            gpio::PC10<Alternate<9>>,
            gpio::PD12<Alternate<9>>,
            gpio::PF9<Alternate<10>>,
            NoIo
        ]
        IO2: [
            gpio::PE2<Alternate<9>>,
            gpio::PF7<Alternate<9>>,
            NoIo
        ]
        IO3: [
            gpio::PA1<Alternate<9>>,
            gpio::PD13<Alternate<9>>,
            gpio::PF6<Alternate<9>>,
            NoIo
        ]
    ],
    Bank2: [
        IO0: [
            gpio::PE7<Alternate<10>>,
            gpio::PF8<Alternate<10>>,
            gpio::PH2<Alternate<9>>
        ]
        IO1: [
            gpio::PE8<Alternate<10>>,
            gpio::PF9<Alternate<10>>,
            gpio::PH3<Alternate<9>>,
            NoIo
        ]
        IO2: [
            gpio::PE9<Alternate<10>>,
            gpio::PG9<Alternate<9>>,
            NoIo
        ]
        IO3: [
            gpio::PE10<Alternate<10>>,
            gpio::PG14<Alternate<9>>,
            NoIo
        ]
    ]
}

pub trait QspiExt {
    fn bank1<CONFIG, PINS>(
        self,
        _pins: PINS,
        config: CONFIG,
        clocks: &CoreClocks,
        prec: rec::Qspi,
    ) -> Qspi<stm32::QUADSPI>
    where
        CONFIG: Into<Config>,
        PINS: PinsBank1;

    fn bank2<CONFIG, PINS>(
        self,
        _pins: PINS,
        config: CONFIG,
        clocks: &CoreClocks,
        prec: rec::Qspi,
    ) -> Qspi<stm32::QUADSPI>
    where
        CONFIG: Into<Config>,
        PINS: PinsBank2;

    fn qspi_unchecked<CONFIG>(
        self,
        config: CONFIG,
        bank: Bank,
        clocks: &CoreClocks,
        prec: rec::Qspi,
    ) -> Qspi<stm32::QUADSPI>
    where
        CONFIG: Into<Config>;
}

impl Qspi<stm32::QUADSPI> {
    pub fn qspi_unchecked<CONFIG>(
        regs: stm32::QUADSPI,
        config: CONFIG,
        bank: Bank,
        clocks: &CoreClocks,
        prec: rec::Qspi,
    ) -> Self
    where
        CONFIG: Into<Config>,
    {
        prec.enable();

        // Disable QUADSPI before configuring it.
        regs.cr.write(|w| w.en().clear_bit());

        let spi_kernel_ck = Self::kernel_clk_unwrap(clocks).0;

        while regs.sr.read().busy().bit_is_set() {}

        let config: Config = config.into();

        // Configure the FSIZE to maximum. It appears that even when addressing is not used, the
        // flash size violation may still trigger.
        regs.dcr.write(|w| unsafe { w.fsize().bits(0x1F) });

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

        // Configure the communication method for QSPI.
        regs.ccr.write(|w| unsafe {
            w.fmode()
                .bits(0) // indirect mode
                .dmode()
                .bits(config.mode.reg_value())
                .admode()
                .bits(config.mode.reg_value())
                .adsize()
                .bits(0) // Eight-bit address
                .imode()
                .bits(0) // No instruction phase
                .dcyc()
                .bits(config.dummy_cycles)
        });

        let spi_frequency = config.frequency.0;
        let divisor = match (spi_kernel_ck + spi_frequency - 1) / spi_frequency
        {
            divisor @ 1..=256 => divisor - 1,
            _ => panic!("Invalid QSPI frequency requested"),
        };

        // Write the prescaler and the SSHIFT bit.
        //
        // Note that we default to setting SSHIFT (sampling on the falling
        // edge). This is because it appears that the QSPI may have signal
        // contention issues when reading with zero dummy cycles. Setting SSHIFT
        // forces the read to occur on the falling edge instead of the rising
        // edge. Refer to https://github.com/quartiq/stabilizer/issues/101 for
        // more information
        //
        // SSHIFT must not be set in DDR mode.
        regs.cr.write(|w| unsafe {
            w.prescaler()
                .bits(divisor as u8)
                .sshift()
                .bit(config.sampling_edge == SamplingEdge::Falling)
                .fthres()
                .bits(config.fifo_threshold - 1)
        });

        match bank {
            Bank::One => regs.cr.modify(|_, w| w.fsel().clear_bit()),
            Bank::Two => regs.cr.modify(|_, w| w.fsel().set_bit()),
            Bank::Dual => regs.cr.modify(|_, w| w.dfm().set_bit()),
        }

        // Enable ther peripheral
        regs.cr.modify(|_, w| w.en().set_bit());

        Qspi {
            rb: regs,
            mode: config.mode,
        }
    }
}

impl QspiExt for stm32::QUADSPI {
    fn bank1<CONFIG, PINS>(
        self,
        _pins: PINS,
        config: CONFIG,
        clocks: &CoreClocks,
        prec: rec::Qspi,
    ) -> Qspi<stm32::QUADSPI>
    where
        CONFIG: Into<Config>,
        PINS: PinsBank1,
    {
        Qspi::qspi_unchecked(self, config, Bank::One, clocks, prec)
    }

    fn bank2<CONFIG, PINS>(
        self,
        _pins: PINS,
        config: CONFIG,
        clocks: &CoreClocks,
        prec: rec::Qspi,
    ) -> Qspi<stm32::QUADSPI>
    where
        CONFIG: Into<Config>,
        PINS: PinsBank2,
    {
        Qspi::qspi_unchecked(self, config, Bank::Two, clocks, prec)
    }

    fn qspi_unchecked<CONFIG>(
        self,
        config: CONFIG,
        bank: Bank,
        clocks: &CoreClocks,
        prec: rec::Qspi,
    ) -> Qspi<stm32::QUADSPI>
    where
        CONFIG: Into<Config>,
    {
        Qspi::qspi_unchecked(self, config, bank, clocks, prec)
    }
}
