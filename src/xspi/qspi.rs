//! Quad SPI (QSPI) bus
//!
//! See the parent module for documentation

use crate::{
    gpio::{
        self, alt::quadspi as alt, alt::QuadSpiBank as QB, PinSpeed, Speed,
    },
    rcc::{rec, CoreClocks, ResetEnable},
    stm32,
};
use alt::{Bank1, Bank2};

use super::{Bank, Config, Qspi, SamplingEdge};

pub trait SingleBank {
    const BANK: Bank;
}

impl SingleBank for Bank1 {
    const BANK: Bank = Bank::One;
}

impl SingleBank for Bank2 {
    const BANK: Bank = Bank::Two;
}

/// Used to indicate that an IO pin is not used by the QSPI interface.
pub use gpio::NoPin as NoIo;

pub trait QspiExt: Sized {
    fn bank1(
        self,
        pins: (
            impl Into<alt::Clk>,
            impl Into<<Bank1 as QB>::Io0>,
            impl Into<<Bank1 as QB>::Io1>,
            impl Into<<Bank1 as QB>::Io2>,
            impl Into<<Bank1 as QB>::Io3>,
            Option<impl Into<<Bank1 as QB>::Ncs>>,
        ),
        config: impl Into<Config>,
        clocks: &CoreClocks,
        prec: rec::Qspi,
    ) -> Qspi<stm32::QUADSPI> {
        Self::single_bank::<Bank1>(self, pins, config, clocks, prec)
    }
    fn bank2(
        self,
        pins: (
            impl Into<alt::Clk>,
            impl Into<<Bank2 as QB>::Io0>,
            impl Into<<Bank2 as QB>::Io1>,
            impl Into<<Bank2 as QB>::Io2>,
            impl Into<<Bank2 as QB>::Io3>,
            Option<impl Into<<Bank2 as QB>::Ncs>>,
        ),
        config: impl Into<Config>,
        clocks: &CoreClocks,
        prec: rec::Qspi,
    ) -> Qspi<stm32::QUADSPI> {
        Self::single_bank::<Bank2>(self, pins, config, clocks, prec)
    }
    fn single_bank<B: QB + SingleBank>(
        self,
        pins: (
            impl Into<alt::Clk>,
            impl Into<B::Io0>,
            impl Into<B::Io1>,
            impl Into<B::Io2>,
            impl Into<B::Io3>,
            Option<impl Into<B::Ncs>>,
        ),
        config: impl Into<Config>,
        clocks: &CoreClocks,
        prec: rec::Qspi,
    ) -> Qspi<stm32::QUADSPI> {
        let _pins = (
            pins.0.into().speed(Speed::VeryHigh),
            pins.1.into().speed(Speed::VeryHigh),
            pins.2.into().speed(Speed::VeryHigh),
            pins.3.into().speed(Speed::VeryHigh),
            pins.4.into().speed(Speed::VeryHigh),
            pins.5.map(|p| p.into().speed(Speed::VeryHigh)),
        );
        Self::qspi_unchecked(self, config, B::BANK, clocks, prec)
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
