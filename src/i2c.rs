//! Inter Integrated Circuit (I2C)
//!
//! # Examples
//!
//! - [I2C simple examples](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/i2c.rs)
//! - [I2C example using I2C4 and BDMA](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/i2c4_bdma.rs)

use core::cmp;
use core::marker::PhantomData;

use crate::gpio::{self, Alternate, OpenDrain};
use crate::hal::blocking::i2c::{Read, Write, WriteRead};
use crate::rcc::{rec, CoreClocks, ResetEnable};
use crate::stm32::{I2C1, I2C2, I2C3, I2C4};
use crate::time::Hertz;
use cast::u16;

/// I2C Events
///
/// Each event is a possible interrupt source, if enabled
#[derive(Copy, Clone, PartialEq)]
pub enum Event {
    /// (TXIE)
    Transmit,
    /// (RXIE)
    Receive,
    /// (TCIE)
    TransferComplete,
    /// Stop detection (STOPIE)
    Stop,
    /// (ERRIE)
    Errors,
    /// Not Acknowledge received (NACKIE)
    NotAcknowledge,
}

/// I2C Stop Configuration
///
/// Peripheral options for generating the STOP condition
#[derive(Copy, Clone, PartialEq)]
pub enum Stop {
    /// Software end mode: Must write register to generate STOP condition
    Software,
    /// Automatic end mode: A STOP condition is automatically generated once the
    /// configured number of bytes have been transferred
    Automatic,
}

/// I2C error
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// Bus error
    Bus,
    /// Arbitration loss
    Arbitration,
    /// No ack received
    NotAcknowledge,
    // Overrun, // slave mode only
    // Pec, // SMBUS mode only
    // Timeout, // SMBUS mode only
    // Alert, // SMBUS mode only
}

/// A trait to represent the SCL Pin of an I2C Port
pub trait PinScl<I2C> {}

/// A trait to represent the SDL Pin of an I2C Port
pub trait PinSda<I2C> {}

/// A trait to represent the collection of pins required for an I2C port
pub trait Pins<I2C> {}

impl<I2C, SCL, SDA> Pins<I2C> for (SCL, SDA)
where
    SCL: PinScl<I2C>,
    SDA: PinSda<I2C>,
{
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct I2c<I2C> {
    i2c: I2C,
}

pub trait I2cExt<I2C>: Sized {
    type Rec: ResetEnable;

    fn i2c<PINS, F>(
        self,
        _pins: PINS,
        frequency: F,
        prec: Self::Rec,
        clocks: &CoreClocks,
    ) -> I2c<I2C>
    where
        PINS: Pins<I2C>,
        F: Into<Hertz>;

    fn i2c_unchecked<F>(
        self,
        frequency: F,
        prec: Self::Rec,
        clocks: &CoreClocks,
    ) -> I2c<I2C>
    where
        F: Into<Hertz>;
}

// Sequence to flush the TXDR register. This resets the TXIS and TXE
// flags
macro_rules! flush_txdr {
    ($i2c:expr) => {
        // If a pending TXIS flag is set, write dummy data to TXDR
        if $i2c.isr.read().txis().bit_is_set() {
            $i2c.txdr.write(|w| w.txdata().bits(0));
        }

        // If TXDR is not flagged as empty, write 1 to flush it
        if $i2c.isr.read().txe().is_not_empty() {
            $i2c.isr.write(|w| w.txe().set_bit());
        }
    };
}

macro_rules! busy_wait {
    ($i2c:expr, $flag:ident, $variant:ident) => {
        loop {
            let isr = $i2c.isr.read();

            if isr.$flag().$variant() {
                break;
            } else if isr.berr().is_error() {
                $i2c.icr.write(|w| w.berrcf().set_bit());
                return Err(Error::Bus);
            } else if isr.arlo().is_lost() {
                $i2c.icr.write(|w| w.arlocf().set_bit());
                return Err(Error::Arbitration);
            } else if isr.nackf().bit_is_set() {
                $i2c.icr.write(|w| w.stopcf().set_bit().nackcf().set_bit());
                flush_txdr!($i2c);
                return Err(Error::NotAcknowledge);
            } else {
                // try again
            }
        }
    };
}

// Calculate I2C timing for Analog Filter ON, Digital Filter OFF
macro_rules! i2c_timing {
    ($i2cclk:ident, $freq:ident) => {{
        // Refer to RM0433 Rev 7 Figure 539 for setup and hold timing:
        //
        // t_I2CCLK = 1 / PCLK1
        // t_PRESC  = (PRESC + 1) * t_I2CCLK
        // t_SCLL   = (SCLL + 1) * t_PRESC
        // t_SCLH   = (SCLH + 1) * t_PRESC
        //
        // t_SYNC1 + t_SYNC2 > 4 * t_I2CCLK
        // t_SCL ~= t_SYNC1 + t_SYNC2 + t_SCLL + t_SCLH
        let ratio = $i2cclk / $freq;

        // For the standard-mode configuration method, we must have a ratio of 4
        // or higher
        assert!(
            ratio >= 4,
            "The I2C PCLK must be at least 4 times the bus frequency!"
        );

        let (presc_reg, scll, sclh, sdadel, scldel) = if $freq > 100_000 {
            // Fast-mode (Fm) or Fast-mode Plus (Fm+)
            // here we pick SCLL + 1 = 2 * (SCLH + 1)

            // Prescaler, 96 ticks for sclh/scll. Round up then subtract 1
            let presc_reg = ((ratio - 1) / 96) as u8;
            // ratio < 1200 by pclk 120MHz max., therefore presc < 16

            // Actual precale value selected
            let presc = (presc_reg + 1) as u32;

            let sclh = ((ratio / presc) - 3) / 3;
            let scll = (2 * (sclh + 1)) - 1;

            let (sdadel, scldel) = if $freq > 400_000 {
                // Fast-mode Plus (Fm+)
                assert!($i2cclk >= 17_000_000); // See table in datsheet

                let sdadel = $i2cclk / 8_000_000 / presc;
                let scldel = $i2cclk / 4_000_000 / presc - 1;

                (sdadel, scldel)
            } else {
                // Fast-mode (Fm)
                assert!($i2cclk >= 8_000_000); // See table in datsheet

                let sdadel = $i2cclk / 3_000_000 / presc;
                let scldel = $i2cclk / 1_000_000 / presc - 1;

                (sdadel, scldel)
            };

            (
                presc_reg,
                scll as u8,
                sclh as u8,
                sdadel as u8,
                scldel as u8,
            )
        } else {
            // Standard-mode (Sm)
            // here we pick SCLL = SCLH
            assert!($i2cclk >= 2_000_000); // See table in datsheet

            // Prescaler, 128 or 256 ticks for sclh/scll. Round up then
            // subtract 1
            let presc_reg = (ratio - 1)
                / if $freq < 8000 {
                    256
                } else if $freq < 80_000 {
                    128
                } else {
                    64
                };
            let presc_reg = cmp::min(presc_reg, 15) as u8;

            // Actual prescale value selected
            let presc = (presc_reg + 1) as u32;

            let sclh = ((ratio / presc) - 2) / 2;
            let scll = sclh;

            // Speed check
            assert!(
                sclh < 256,
                "The I2C PCLK is too fast for this bus frequency!"
            );

            let sdadel = $i2cclk / 2_000_000 / presc;
            let scldel = $i2cclk / 500_000 / presc - 1;

            (
                presc_reg,
                scll as u8,
                sclh as u8,
                sdadel as u8,
                scldel as u8,
            )
        };

        // Sanity check
        assert!(presc_reg < 16);

        // Keep values within reasonable limits for fast per_ck
        let sdadel = cmp::max(sdadel, 1);
        let scldel = cmp::max(scldel, 4);

        let sdadel = cmp::min(sdadel, 15);
        let scldel = cmp::min(scldel, 15);

        (presc_reg, scll, sclh, sdadel, scldel)
    }};
}

macro_rules! i2c {
    ($($I2CX:ident: ($i2cX:ident, $Rec:ident, $pclkX:ident),)+) => {
        $(
            impl I2c<$I2CX> {
                /// Create and initialise a new I2C peripheral.
                ///
                /// The frequency of the I2C bus clock is specified by `frequency`.
                ///
                /// # Panics
                ///
                /// Panics if the ratio between `frequency` and the i2c_ker_ck
                /// is out of bounds. The acceptable range is [4, 8192].
                ///
                /// Panics if the `frequency` is too fast. The maximum is 1MHz.
                pub fn $i2cX<F> (
                    i2c: $I2CX,
                    frequency: F,
                    prec: rec::$Rec,
                    clocks: &CoreClocks
                ) -> Self where
                    F: Into<Hertz>,
                {
                    prec.enable().reset();

                    let freq: u32 = frequency.into().0;

                    // Maximum f_SCL for Fast-mode Plus (Fm+)
                    assert!(freq <= 1_000_000);

                    let i2c_clk: u32 = clocks.$pclkX().0;

                    // Clear PE bit in I2C_CR1
                    i2c.cr1.modify(|_, w| w.pe().clear_bit());

                    // Enable the Analog Noise Filter by setting
                    // ANFOFF (Analog Noise Filter OFF) to 0.  This is
                    // usually enabled by default
                    i2c.cr1.modify(|_, w| w.anfoff().clear_bit());

                    // Configure timing
                    let (presc_reg, scll, sclh, sdadel, scldel) = i2c_timing!(i2c_clk, freq);
                    i2c.timingr.write(|w|
                        w.presc()
                            .bits(presc_reg)
                            .scll()
                            .bits(scll)
                            .sclh()
                            .bits(sclh)
                            .sdadel()
                            .bits(sdadel)
                            .scldel()
                            .bits(scldel)
                    );

                    // Enable the peripheral
                    i2c.cr1.write(|w| w.pe().set_bit());

                    I2c { i2c }
                }

                /// Returns a reference to the inner peripheral
                pub fn inner(&self) -> &$I2CX {
                    &self.i2c
                }

                /// Returns a mutable reference to the inner peripheral
                pub fn inner_mut(&mut self) -> &mut $I2CX {
                    &mut self.i2c
                }

                /// Enable or disable the DMA mode for reception
                pub fn rx_dma(&mut self, enable: bool) {
                    self.i2c.cr1.modify(|_,w| w.rxdmaen().bit(enable));
                }

                /// Enable or disable the DMA mode for transmission
                pub fn tx_dma(&mut self, enable: bool) {
                    self.i2c.cr1.modify(|_,w| w.txdmaen().bit(enable));
                }

                /// Start listening for `event`
                pub fn listen(&mut self, event: Event) {
                    self.i2c.cr1.modify(|_,w| {
                        match event {
                            Event::Transmit => w.txie().set_bit(),
                            Event::Receive => w.rxie().set_bit(),
                            Event::TransferComplete => w.tcie().set_bit(),
                            Event::Stop => w.stopie().set_bit(),
                            Event::Errors => w.errie().set_bit(),
                            Event::NotAcknowledge => w.nackie().set_bit(),
                        }
                    });
                }

                /// Stop listening for `event`
                pub fn unlisten(&mut self, event: Event) {
                    self.i2c.cr1.modify(|_,w| {
                        match event {
                            Event::Transmit => w.txie().clear_bit(),
                            Event::Receive => w.rxie().clear_bit(),
                            Event::TransferComplete => w.tcie().clear_bit(),
                            Event::Stop => w.stopie().clear_bit(),
                            Event::Errors => w.errie().clear_bit(),
                            Event::NotAcknowledge => w.nackie().clear_bit(),
                        }
                    });
                    let _ = self.i2c.cr1.read();
                    let _ = self.i2c.cr1.read(); // Delay 2 peripheral clocks
                }

                /// Clears interrupt flag for `event`
                pub fn clear_irq(&mut self, event: Event) {
                    self.i2c.icr.write(|w| {
                        match event {
                            Event::Stop => w.stopcf().set_bit(),
                            Event::Errors => w
                                .berrcf().set_bit()
                                .arlocf().set_bit()
                                .ovrcf().set_bit(),
                            Event::NotAcknowledge => w.nackcf().set_bit(),
                            _ => w
                        }
                    });
                    let _ = self.i2c.isr.read();
                    let _ = self.i2c.isr.read(); // Delay 2 peripheral clocks
                }

                /// Releases the I2C peripheral
                pub fn free(self) -> ($I2CX, rec::$Rec) {
                    (self.i2c, rec::$Rec { _marker: PhantomData })
                }
            }

            /// Master controller methods
            ///
            /// These infallible methods are used to begin or end parts of
            /// transactions, but do __not__ read or write the data
            /// registers. If you want to perform an entire transcation see the
            /// [Read](I2c#impl-Read) and [Write](I2c#impl-Write)
            /// implementations.
            ///
            /// If a previous transcation is still in progress, then these
            /// methods will block until that transcation is complete. A
            /// previous transaction can still be "in progress" up to 50% of a
            /// bus cycle after a ACK/NACK event. Otherwise these methods return
            /// immediately.
            impl I2c<$I2CX> {
                /// Master read
                ///
                /// Perform an I2C start and prepare to receive `length` bytes.
                ///
                /// ```
                /// Master: ST SAD+R  ...  (SP)
                /// Slave:            ...
                /// ```
                pub fn master_read(&mut self, addr: u8, length: usize, stop: Stop) {
                    assert!(length < 256 && length > 0);

                    // Wait for any previous address sequence to end
                    // automatically. This could be up to 50% of a bus
                    // cycle (ie. up to 0.5/freq)
                    while self.i2c.cr2.read().start().bit_is_set() {};

                    // Set START and prepare to receive bytes into
                    // `buffer`. The START bit can be set even if the bus
                    // is BUSY or I2C is in slave mode.
                    self.i2c.cr2.write(|w| {
                        w.sadd()
                            .bits((addr << 1 | 0) as u16)
                            .rd_wrn()
                            .read()
                            .nbytes()
                            .bits(length as u8)
                            .start()
                            .set_bit()
                            .autoend()
                            .bit(stop == Stop::Automatic)
                    });
                }
                /// Master write
                ///
                /// Perform an I2C start and prepare to send `length` bytes.
                ///
                /// ```
                /// Master: ST SAD+W  ...  (SP)
                /// Slave:            ...
                /// ```
                pub fn master_write(&mut self, addr: u8, length: usize, stop: Stop) {
                    assert!(length < 256 && length > 0);

                    // Wait for any previous address sequence to end
                    // automatically. This could be up to 50% of a bus
                    // cycle (ie. up to 0.5/freq)
                    while self.i2c.cr2.read().start().bit_is_set() {};

                    // Set START and prepare to send `bytes`. The
                    // START bit can be set even if the bus is BUSY or
                    // I2C is in slave mode.
                    self.i2c.cr2.write(|w| {
                        w.start()
                            .set_bit()
                            .sadd()
                            .bits(u16(addr << 1 | 0))
                            .add10().clear_bit()
                            .rd_wrn()
                            .write()
                            .nbytes()
                            .bits(length as u8)
                            .autoend()
                            .bit(stop == Stop::Automatic)
                    });
                }

                /// Master restart
                ///
                /// Performs an I2C restart following a write phase and prepare
                /// to receive `length` bytes. The I2C peripheral is configured
                /// to provide an automatic stop.
                ///
                /// ```
                /// Master: ...  SR  SAD+R  ...  (SP)
                /// Slave:  ...             ...
                /// ```
                pub fn master_re_start(&mut self, addr: u8, length: usize, stop: Stop) {
                    assert!(length < 256 && length > 0);

                    self.i2c.cr2.write(|w| {
                        w.sadd()
                            .bits(u16(addr << 1 | 1))
                            .add10().clear_bit()
                            .rd_wrn()
                            .read()
                            .nbytes()
                            .bits(length as u8)
                            .start()
                            .set_bit()
                            .autoend()
                            .bit(stop == Stop::Automatic)
                    });
                }

                /// Master stop
                ///
                /// Generate a stop condition.
                ///
                /// ```
                /// Master: ...  SP
                /// Slave:  ...
                /// ```
                pub fn master_stop(&mut self) {
                    self.i2c.cr2.write(|w| w.stop().set_bit());
                }
            }

            impl I2cExt<$I2CX> for $I2CX {
                type Rec = rec::$Rec;

                /// Create and initialise a new I2C peripheral.
                ///
                /// A tuple of pins `(scl, sda)` for this I2C peripheral should
                /// be passed as `pins`. This function sets each pin to
                /// open-drain mode.
                ///
                /// The frequency of the I2C bus clock is specified by `frequency`.
                ///
                /// # Panics
                ///
                /// Panics if the ratio between `frequency` and the i2c_ker_ck
                /// is out of bounds. The acceptable range is [4, 8192].
                ///
                /// Panics if the `frequency` is too fast. The maximum is 1MHz.
                fn i2c<PINS, F>(self, _pins: PINS, frequency: F,
                                prec: rec::$Rec,
                                clocks: &CoreClocks) -> I2c<$I2CX>
                where
                    PINS: Pins<$I2CX>,
                    F: Into<Hertz>
                {
                    I2c::$i2cX(self, frequency, prec, clocks)
                }

                /// Create and initialise a new I2C peripheral. No pin types are
                /// required.
                ///
                /// The frequency of the I2C bus clock is specified by `frequency`.
                ///
                /// # Panics
                ///
                /// Panics if the ratio between `frequency` and the i2c_ker_ck
                /// is out of bounds. The acceptable range is [4, 8192].
                ///
                /// Panics if the `frequency` is too fast. The maximum is 1MHz.
                fn i2c_unchecked<F>(self, frequency: F,
                                    prec: rec::$Rec,
                                    clocks: &CoreClocks) -> I2c<$I2CX>
                where
                    F: Into<Hertz>
                {
                    I2c::$i2cX(self, frequency, prec, clocks)
                }
            }

            impl Write for I2c<$I2CX> {
                type Error = Error;

                fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
                    // TODO support transfers of more than 255 bytes
                    assert!(bytes.len() < 256 && bytes.len() > 0);

                    // I2C start
                    //
                    // ST SAD+W
                    self.master_write(addr, bytes.len(), Stop::Software);

                    for byte in bytes {
                        // Wait until we are allowed to send data
                        // (START has been ACKed or last byte when
                        // through)
                        busy_wait!(self.i2c, txis, is_empty);

                        // Put byte on the wire
                        self.i2c.txdr.write(|w| w.txdata().bits(*byte));
                    }

                    // Wait until the write finishes
                    busy_wait!(self.i2c, tc, is_complete);

                    // Stop
                    self.master_stop();

                    Ok(())
                }
            }

            impl WriteRead for I2c<$I2CX> {
                type Error = Error;

                fn write_read(
                    &mut self,
                    addr: u8,
                    bytes: &[u8],
                    buffer: &mut [u8],
                ) -> Result<(), Error> {
                    // TODO support transfers of more than 255 bytes
                    assert!(bytes.len() < 256 && bytes.len() > 0);
                    assert!(buffer.len() < 256 && buffer.len() > 0);

                    // I2C start
                    //
                    // ST SAD+W
                    self.master_write(addr, bytes.len(), Stop::Software);

                    for byte in bytes {
                        // Wait until we are allowed to send data
                        // (START has been ACKed or last byte went through)
                        busy_wait!(self.i2c, txis, is_empty);

                        // Put byte on the wire
                        self.i2c.txdr.write(|w| w.txdata().bits(*byte));
                    }

                    // Wait until the write finishes before beginning to read.
                    busy_wait!(self.i2c, tc, is_complete);

                    // I2C re-start
                    //
                    // SR  SAD+R
                    self.master_re_start(addr, buffer.len(), Stop::Automatic);

                    for byte in buffer {
                        // Wait until we have received something
                        busy_wait!(self.i2c, rxne, is_not_empty);

                        *byte = self.i2c.rxdr.read().rxdata().bits();
                    }

                    // automatic STOP

                    Ok(())
                }
            }

            impl Read for I2c<$I2CX> {
                type Error = Error;

                fn read(
                    &mut self,
                    addr: u8,
                    buffer: &mut [u8],
                ) -> Result<(), Error> {
                    // TODO support transfers of more than 255 bytes
                    assert!(buffer.len() < 256 && buffer.len() > 0);

                    self.master_read(addr, buffer.len(), Stop::Automatic);

                    for byte in buffer {
                        // Wait until we have received something
                        busy_wait!(self.i2c, rxne, is_not_empty);

                        *byte = self.i2c.rxdr.read().rxdata().bits();
                    }

                    // automatic STOP

                    Ok(())
                }
            }
        )+
    };
}

macro_rules! pins {
    ($($I2CX:ty: SCL: [$($SCL:ty),*] SDA: [$($SDA:ty),*])+) => {
        $(
            $(
                impl PinScl<$I2CX> for $SCL { }
            )*
            $(
                impl PinSda<$I2CX> for $SDA { }
            )*
        )+
    }
}

pins! {
    I2C1:
        SCL: [
            gpio::PB6<Alternate<4, OpenDrain>>,
            gpio::PB8<Alternate<4, OpenDrain>>
        ]

        SDA: [
            gpio::PB7<Alternate<4, OpenDrain>>,
            gpio::PB9<Alternate<4, OpenDrain>>
        ]

    I2C2:
        SCL: [
            gpio::PB10<Alternate<4, OpenDrain>>,
            gpio::PF1<Alternate<4, OpenDrain>>,
            gpio::PH4<Alternate<4, OpenDrain>>
        ]

        SDA: [
            gpio::PB11<Alternate<4, OpenDrain>>,
            gpio::PF0<Alternate<4, OpenDrain>>,
            gpio::PH5<Alternate<4, OpenDrain>>
        ]

    I2C3:
        SCL: [
            gpio::PA8<Alternate<4, OpenDrain>>,
            gpio::PH7<Alternate<4, OpenDrain>>
        ]

        SDA: [
            gpio::PC9<Alternate<4, OpenDrain>>,
            gpio::PH8<Alternate<4, OpenDrain>>
        ]

    I2C4:
        SCL: [
            gpio::PD12<Alternate<4, OpenDrain>>,
            gpio::PF14<Alternate<4, OpenDrain>>,
            gpio::PH11<Alternate<4, OpenDrain>>,
            gpio::PB6<Alternate<6, OpenDrain>>,
            gpio::PB8<Alternate<6, OpenDrain>>
        ]

        SDA: [
            gpio::PB7<Alternate<6, OpenDrain>>,
            gpio::PB9<Alternate<6, OpenDrain>>,
            gpio::PD13<Alternate<4, OpenDrain>>,
            gpio::PF15<Alternate<4, OpenDrain>>,
            gpio::PH12<Alternate<4, OpenDrain>>
        ]
}

i2c!(
    I2C1: (i2c1, I2c1, pclk1),
    I2C2: (i2c2, I2c2, pclk1),
    I2C3: (i2c3, I2c3, pclk1),
    I2C4: (i2c4, I2c4, pclk4),
);

#[cfg(test)]
mod tests {
    use core::cmp;

    /// Runs a timing testcase over PCLK and I2C clock ranges
    fn i2c_timing_testcase<F>(f: F)
    where
        F: Fn(u32, u32),
    {
        let i2c_timing_tests = [
            // (i2c_clk, range of bus frequencies to test)
            (2_000_000, (4_000..=100_000)), // Min PCLK
            (8_000_000, (4_000..=400_000)), // Slowest PCLK for fast mode
            (16_000_000, (4_000..=400_000)), // Default H7 PCLK = 16MHz
            (32_000_000, (4_000..=1_000_000)),
            (79_876_135, (10_000..=1_000_000)),
            (100_000_000, (13_000..=1_000_000)),
            (120_000_000, (15_000..=1_000_000)), // Max PCLK
        ];

        for (clock, freq_range) in i2c_timing_tests.iter() {
            for freq in freq_range.clone().step_by(1_000) {
                f(*clock, freq)
            }
        }
    }

    #[test]
    /// Test the SCL frequency is within the expected range
    fn i2c_frequency() {
        i2c_timing_testcase(|i2c_clk: u32, freq: u32| {
            let (presc_reg, scll, sclh, _, _) = i2c_timing!(i2c_clk, freq);

            // Timing parameters
            let presc = (presc_reg + 1) as f32;
            let t_i2c_clk = 1. / (i2c_clk as f32);
            let freq = freq as f32;

            // Estimate minimum sync times. Analog filter on, 2 i2c_clk cycles
            let t_af_min = 50e-9_f32; // Analog filter 50ns. From H7 Datasheet
            let t_sync1 = t_af_min + 2. * t_i2c_clk;
            let t_sync2 = t_af_min + 2. * t_i2c_clk;

            // See RM0433 Rev 7 Section 47.4.9
            let t_high_low = sclh as f32 + 1. + scll as f32 + 1.;
            let t_scl = t_sync1 + t_sync2 + (t_high_low * presc * t_i2c_clk);
            let f_scl = 1. / t_scl;

            let error = (freq - f_scl) / freq;
            println!(
                "Set SCL = {} Actual = {} Error {:.1}%",
                freq,
                f_scl,
                100. * error
            );

            // We must generate a bus frequency less than or equal to that
            // specified. Tolerate a 2% error
            assert!(f_scl <= 1.02 * freq);

            // But it should not be too much less than specified
            assert!(f_scl > 0.8 * freq);
        });
    }

    #[test]
    /// Test that the low period of SCL is greater than the minimum specification
    fn i2c_scl_low() {
        i2c_timing_testcase(|i2c_clk: u32, freq: u32| {
            let (presc_reg, scll, _, _, _) = i2c_timing!(i2c_clk, freq);

            // Timing parameters
            let presc = (presc_reg + 1) as f32;
            let t_i2c_clk = 1. / (i2c_clk as f32);
            let freq = freq as f32;
            let t_scll = (scll as f32 + 1.) * presc * t_i2c_clk;

            // From I2C Specification Table 10
            //
            // UM10204 rev 6.: https://www.nxp.com/docs/en/user-guide/UM10204.pdf
            let t_scll_minimum = match freq {
                x if x <= 100_000. => 4.7e-6, // Standard mode (Sm)
                x if x <= 400_000. => 1.3e-6, // Fast mode (Fm)
                _ => 0.5e-6,                  // Fast mode Plus (Fm+)
            };

            println!("Target {} Hz; SCLL {}", freq, scll);
            println!(
                "T SCL LOW {:.2e}; MINIMUM {:.2e}",
                t_scll, t_scll_minimum
            );
            assert!(t_scll >= t_scll_minimum);
        });
    }

    #[test]
    /// Test the SDADEL value is greater than the minimum specification
    fn i2c_sdadel_minimum() {
        i2c_timing_testcase(|i2c_clk: u32, freq: u32| {
            let (presc_reg, _, _, sdadel, _) = i2c_timing!(i2c_clk, freq);

            // Timing parameters
            let presc = (presc_reg + 1) as f32;
            let t_i2c_clk = 1. / (i2c_clk as f32);
            let freq = freq as f32;
            let t_sdadel = (sdadel as f32) * presc * t_i2c_clk;

            // From I2C Specification Table 10
            //
            // UM10204 rev 6.: https://www.nxp.com/docs/en/user-guide/UM10204.pdf
            let t_fall_max = match freq {
                x if x <= 100_000. => 300e-9, // Standard mode (Sm)
                x if x <= 400_000. => 300e-9, // Fast mode (Fm)
                _ => 120e-9,                  // Fast mode Plus (Fm+)
            };

            let t_af_min = 50e-9_f32; // Analog filter min 50ns. From H7 Datasheet
            let hddat_min = 0.;

            // From RM0433 Rev 7 Section 47.4.5
            //
            // tSDADEL >= {tf + tHD;DAT(min) - tAF(min) - [(DNF + 3) x tI2CCLK]}
            let t_sdadel_minimim =
                t_fall_max + hddat_min - t_af_min - (3. * t_i2c_clk);

            println!("Target {} Hz; SDADEL {}", freq, sdadel);
            println!(
                "T SDA DELAY {:.2e} MINIMUM {:.2e}",
                t_sdadel, t_sdadel_minimim
            );
            assert!(sdadel <= 15);
            assert!(t_sdadel >= t_sdadel_minimim);
        });
    }

    #[test]
    /// Test the SDADEL value is less than the maximum specification
    fn i2c_sdadel_maximum() {
        i2c_timing_testcase(|i2c_clk: u32, freq: u32| {
            let (presc_reg, _, _, sdadel, _) = i2c_timing!(i2c_clk, freq);

            // Timing parameters
            let presc = (presc_reg + 1) as f32;
            let t_i2c_clk = 1. / (i2c_clk as f32);
            let freq = freq as f32;
            let t_sdadel = (sdadel as f32) * presc * t_i2c_clk;

            let t_hddat_max = match freq {
                x if x <= 100_000. => 3.45e-6, // Standard mode (Sm)
                x if x <= 400_000. => 0.9e-6,  // Fast mode (Fm)
                _ => 0.45e-6,                  // Fast mode Plus (Fm+)
            };
            let t_af_max = 80e-9_f32; // Analog filter max 80ns. From H7 Datasheet

            // From RM0433 Rev 7 Section 47.4.5
            //
            // tSDADEL <= {tHD;DAT(max) - tAF(max) - [(DNF + 4) x tI2CCLK]}
            let t_sdadel_maximum = t_hddat_max - t_af_max - (4. * t_i2c_clk);

            println!("Target {} Hz; SDADEL {}", freq, sdadel);
            println!(
                "T SDA DELAY {:.2e} MAXIMUM {:.2e}",
                t_sdadel, t_sdadel_maximum
            );
            assert!(sdadel <= 15);
            assert!(t_sdadel <= t_sdadel_maximum);
        });
    }

    #[test]
    /// Test the SCLDEL value is greater than the minimum specification
    fn i2c_scldel_minimum() {
        i2c_timing_testcase(|i2c_clk: u32, freq: u32| {
            let (presc_reg, _, _, _, scldel) = i2c_timing!(i2c_clk, freq);

            // Timing parameters
            let presc = (presc_reg + 1) as f32;
            let t_i2c_clk = 1. / (i2c_clk as f32);
            let freq = freq as f32;
            let t_scldel = (scldel as f32) * presc * t_i2c_clk;

            // From I2C Specification Table 10
            //
            // UM10204 rev 6.: https://www.nxp.com/docs/en/user-guide/UM10204.pdf
            let t_rise_max = match freq {
                x if x <= 100_000. => 1000e-9, // Standard mode (Sm)
                x if x <= 400_000. => 300e-9,  // Fast mode (Fm)
                _ => 120e-9,                   // Fast mode Plus (Fm+)
            };
            let t_sudat_min = match freq {
                x if x <= 100_000. => 250e-9, // Standard mode (Sm)
                x if x <= 400_000. => 100e-9, // Fast mode (Fm)
                _ => 50e-9,                   // Fast mode Plus (Fm+)
            };

            // From RM0433 Rev 7 Section 47.4.5
            //
            // tSCLDEL >= tr + tSU;DAT(min)
            let t_scldel_minimum = t_rise_max + t_sudat_min;

            println!("Target {} Hz; SCLDEL {}", freq, scldel);
            println!(
                "T SCL DELAY {:.2e} MINIMUM {:.2e}",
                t_scldel, t_scldel_minimum
            );
            assert!(scldel <= 15);
            assert!(t_scldel >= t_scldel_minimum);
        });
    }
}
