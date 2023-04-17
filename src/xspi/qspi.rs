//! Quad SPI (QSPI) bus
//!
//! See the parent module for documentation

use crate::{
    gpio::{self, alt::quadspi as alt, Alternate},
    rcc::{rec, CoreClocks, ResetEnable},
    stm32,
};

use super::{Bank, Config, Qspi, SamplingEdge};

/// Used to indicate that an IO pin is not used by the QSPI interface.
pub use gpio::NoPin as NoIo;

/// Indicates a set of pins can be used for the QSPI interface on bank 1.
pub trait PinsBank1 {
    type AltPins;
    fn convert(self) -> Self::AltPins;
}

/// Indicates a set of pins can be used for the QSPI interface on bank 2.
pub trait PinsBank2 {
    type AltPins;
    fn convert(self) -> Self::AltPins;
}

impl<SCK, IO0, IO1, IO2, IO3> PinsBank1 for (SCK, IO0, IO1, IO2, IO3)
where
    SCK: Into<alt::Clk>,
    IO0: Into<alt::Bk1Io0>,
    IO1: Into<alt::Bk1Io1>,
    IO2: Into<alt::Bk1Io2>,
    IO3: Into<alt::Bk1Io3>,
{
    type AltPins =
        (alt::Clk, alt::Bk1Io0, alt::Bk1Io1, alt::Bk1Io2, alt::Bk1Io3);
    fn convert(self) -> Self::AltPins {
        (
            self.0.into(),
            self.1.into(),
            self.2.into(),
            self.3.into(),
            self.4.into(),
        )
    }
}

impl<SCK, IO0, IO1, IO2, IO3> PinsBank2 for (SCK, IO0, IO1, IO2, IO3)
where
    SCK: Into<alt::Clk>,
    IO0: Into<alt::Bk2Io0>,
    IO1: Into<alt::Bk2Io1>,
    IO2: Into<alt::Bk2Io2>,
    IO3: Into<alt::Bk2Io3>,
{
    type AltPins =
        (alt::Clk, alt::Bk2Io0, alt::Bk2Io1, alt::Bk2Io2, alt::Bk2Io3);
    fn convert(self) -> Self::AltPins {
        (
            self.0.into(),
            self.1.into(),
            self.2.into(),
            self.3.into(),
            self.4.into(),
        )
    }
}

pub trait QspiExt: Sized {
    fn bank1(
        self,
        pins: impl PinsBank1,
        config: impl Into<Config>,
        clocks: &CoreClocks,
        prec: rec::Qspi,
    ) -> Qspi<stm32::QUADSPI> {
        let _pins = pins.convert();
        Self::qspi_unchecked(self, config, Bank::One, clocks, prec)
    }

    fn bank2(
        self,
        pins: impl PinsBank2,
        config: impl Into<Config>,
        clocks: &CoreClocks,
        prec: rec::Qspi,
    ) -> Qspi<stm32::QUADSPI> {
        let _pins = pins.convert();
        Self::qspi_unchecked(self, config, Bank::Two, clocks, prec)
    }

    fn qspi_unchecked(
        self,
        config: impl Into<Config>,
        bank: Bank,
        clocks: &CoreClocks,
        prec: rec::Qspi,
    ) -> Qspi<stm32::QUADSPI>;
}

impl Qspi<stm32::QUADSPI> {
    pub fn new_unchecked<CONFIG>(
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

        let spi_kernel_ck = Self::kernel_clk_unwrap(clocks).raw();

        while regs.sr.read().busy().bit_is_set() {}

        let config: Config = config.into();

        // Configure the FSIZE to maximum. It appears that even when addressing is not used, the
        // flash size violation may still trigger.
        regs.dcr.write(|w| unsafe { w.fsize().bits(0x1F) });

        // Clear all pending flags.
        regs.fcr.write(|w| {
            w.ctof().set_bit();
            w.csmf().set_bit();
            w.ctcf().set_bit();
            w.ctef().set_bit()
        });

        // Configure the communication method for QSPI.
        regs.ccr.write(|w| unsafe {
            w.fmode().bits(0); // indirect mode
            w.dmode().bits(config.mode.reg_value());
            w.admode().bits(config.mode.reg_value());
            w.adsize().bits(0); // Eight-bit address
            w.imode().bits(0); // No instruction phase
            w.dcyc().bits(config.dummy_cycles)
        });

        let spi_frequency = config.frequency.raw();
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
            w.prescaler().bits(divisor as u8);
            w.sshift()
                .bit(config.sampling_edge == SamplingEdge::Falling);
            w.fthres().bits(config.fifo_threshold - 1)
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
    fn qspi_unchecked(
        self,
        config: impl Into<Config>,
        bank: Bank,
        clocks: &CoreClocks,
        prec: rec::Qspi,
    ) -> Qspi<stm32::QUADSPI> {
        Qspi::new_unchecked(self, config, bank, clocks, prec)
    }
}
