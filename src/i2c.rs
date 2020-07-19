//! Inter Integrated Circuit implementation

use core::marker::PhantomData;

use crate::gpio::gpioa::PA8;
use crate::gpio::gpiob::{PB10, PB11, PB6, PB7, PB8, PB9};
use crate::gpio::gpioc::PC9;
use crate::gpio::gpiod::{PD12, PD13};
use crate::gpio::gpiof::{PF0, PF1, PF14, PF15};
use crate::gpio::gpioh::{PH11, PH12, PH4, PH5, PH7, PH8};
use crate::gpio::{Alternate, AF4, AF6};
use crate::hal::blocking::i2c::{Read, Write, WriteRead};
use crate::rcc::{rec, CoreClocks, ResetEnable};
use crate::stm32::{I2C1, I2C2, I2C3, I2C4};
use crate::time::Hertz;
use cast::{u16, u8};

/// I2C error
#[derive(Debug)]
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
    #[doc(hidden)]
    _Extensible,
}

/// A trait to represent the SCL Pin of an I2C Port
pub trait PinScl<I2C> {}

/// A trait to represent the SDL Pin of an I2C Port
pub trait PinSda<I2C> {}

pub trait Pins<I2C> {}

impl<I2C, SCL, SDA> Pins<I2C> for (SCL, SDA)
where
    SCL: PinScl<I2C>,
    SDA: PinSda<I2C>,
{
}

#[derive(Debug)]
pub struct I2c<I2C> {
    i2c: I2C,
}

pub trait I2cExt<I2C>: Sized {
    type Rec: ResetEnable;

    fn i2c<PINS, F>(
        self,
        _pins: PINS,
        freq: F,
        prec: Self::Rec,
        clocks: &CoreClocks,
    ) -> I2c<I2C>
    where
        PINS: Pins<I2C>,
        F: Into<Hertz>;

    fn i2c_unchecked<F>(
        self,
        freq: F,
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

macro_rules! i2c {
    ($($I2CX:ident: ($i2cX:ident, $Rec:ident, $pclkX:ident),)+) => {
        $(
            impl I2c<$I2CX> {
                /// Basically a new function for an I2C peripheral
                pub fn $i2cX<F> (
                    i2c: $I2CX,
                    freq: F,
                    prec: rec::$Rec,
                    clocks: &CoreClocks
                ) -> Self where
                    F: Into<Hertz>,
                {
                    prec.enable().reset();

                    let freq = freq.into().0;

                    assert!(freq <= 1_000_000);

                    let i2cclk = clocks.$pclkX().0;

                    // Clear PE bit in I2C_CR1
                    i2c.cr1.modify(|_, w| w.pe().clear_bit());

                    // Enable the Analog Noise Filter by setting
                    // ANFOFF (Analog Noise Filter OFF) to 0.  This is
                    // usually enabled by default
                    i2c.cr1.modify(|_, w| w.anfoff().clear_bit());

                    // Refer to RM0433 Rev 6 - Figure 539 for setup and hold timing:
                    // TODO review compliance with the timing requirements of I2C
                    // t_I2CCLK = 1 / PCLK1
                    // t_PRESC  = (PRESC + 1) * t_I2CCLK
                    // t_SCLL   = (SCLL + 1) * t_PRESC
                    // t_SCLH   = (SCLH + 1) * t_PRESC
                    //
                    // t_SYNC1 + t_SYNC2 > 4 * t_I2CCLK
                    // t_SCL ~= t_SYNC1 + t_SYNC2 + t_SCLL + t_SCLH
                    let ratio = i2cclk / freq - 4;
                    let (presc, scll, sclh, sdadel, scldel) = if freq > 100_000 {
                        // fast-mode or fast-mode plus
                        // here we pick SCLL + 1 = 2 * (SCLH + 1)
                        let presc = ratio / 387;

                        let sclh = ((ratio / (presc + 1)) - 3) / 3;
                        let scll = 2 * (sclh + 1) - 1;

                        let (sdadel, scldel) = if freq > 400_000 {
                            // fast-mode plus
                            let sdadel = 0;
                            let scldel = i2cclk / 4_000_000 / (presc + 1) - 1;

                            (sdadel, scldel)
                        } else {
                            // fast-mode
                            let sdadel = i2cclk / 8_000_000 / (presc + 1);
                            let scldel = i2cclk / 2_000_000 / (presc + 1) - 1;

                            (sdadel, scldel)
                        };

                        (presc, scll, sclh, sdadel, scldel)
                    } else {
                        // standard-mode
                        // here we pick SCLL = SCLH
                        let presc = ratio / 514;
                        let sclh = ((ratio / (presc + 1)) - 2) / 2;
                        let scll = sclh;

                        let sdadel = i2cclk / 2_000_000 / (presc + 1);
                        let scldel = i2cclk / 800_000 / (presc + 1) - 1;

                        (presc, scll, sclh, sdadel, scldel)
                    };

                    let presc = u8(presc).unwrap();
                    //assert!(presc < 16);
                    let scldel = u8(scldel).unwrap();
                    //assert!(scldel < 16);
                    let sdadel = u8(sdadel).unwrap();
                    //assert!(sdadel < 16);
                    let sclh = u8(sclh).unwrap();
                    let scll = u8(scll).unwrap();

                    // Configure for "fast mode" (400 KHz)
                    i2c.timingr.write(|w|
                        w.presc()
                            .bits(presc)
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

                /// Releases the I2C peripheral
                pub fn free(self) -> ($I2CX, rec::$Rec) {
                    (self.i2c, rec::$Rec { _marker: PhantomData })
                }
            }

            impl I2cExt<$I2CX> for $I2CX {
                type Rec = rec::$Rec;

                fn i2c<PINS, F>(self, _pins: PINS, freq: F,
                                prec: rec::$Rec,
                                clocks: &CoreClocks) -> I2c<$I2CX>
                where
                    PINS: Pins<$I2CX>,
                    F: Into<Hertz>
                {
                    I2c::$i2cX(self, freq, prec, clocks)
                }

                fn i2c_unchecked<F>(self, freq: F,
                                    prec: rec::$Rec,
                                    clocks: &CoreClocks) -> I2c<$I2CX>
                where
                    F: Into<Hertz>
                {
                    I2c::$i2cX(self, freq, prec, clocks)
                }
            }

            impl Write for I2c<$I2CX> {
                type Error = Error;

                fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
                    // TODO support transfers of more than 255 bytes
                    assert!(bytes.len() < 256 && bytes.len() > 0);

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
                            .bits(bytes.len() as u8)
                            .autoend()
                            .automatic()
                    });

                    for byte in bytes {
                        // Wait until we are allowed to send data
                        // (START has been ACKed or last byte when
                        // through)
                        busy_wait!(self.i2c, txis, is_empty);

                        // Put byte on the wire
                        self.i2c.txdr.write(|w| w.txdata().bits(*byte));
                    }
                    // automatic STOP

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
                            .bits(bytes.len() as u8)
                            .autoend()
                            .software()
                    });

                    for byte in bytes {
                        // Wait until we are allowed to send data
                        // (START has been ACKed or last byte went through)
                        busy_wait!(self.i2c, txis, is_empty);

                        // Put byte on the wire
                        self.i2c.txdr.write(|w| w.txdata().bits(*byte));
                    }

                    // Wait until the write finishes before beginning to read.
                    busy_wait!(self.i2c, tc, is_complete);

                    // reSTART and prepare to receive bytes into `buffer`
                    self.i2c.cr2.write(|w| {
                        w.sadd()
                            .bits(u16(addr << 1 | 1))
                            .add10().clear_bit()
                            .rd_wrn()
                            .read()
                            .nbytes()
                            .bits(buffer.len() as u8)
                            .start()
                            .set_bit()
                            .autoend()
                            .automatic()
                    });

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
                        .bits(buffer.len() as u8)
                        .start()
                        .set_bit()
                        .autoend()
                        .automatic()
                });

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
                impl PinScl<$I2CX> for $SCL {}
            )*
            $(
                impl PinSda<$I2CX> for $SDA {}
            )*
        )+
    }
}

pins! {
    I2C1:
        SCL: [
            PB6<Alternate<AF4>>,
            PB8<Alternate<AF4>>
        ]

        SDA: [
            PB7<Alternate<AF4>>,
            PB9<Alternate<AF4>>
        ]

    I2C2:
        SCL: [
            PB10<Alternate<AF4>>,
            PF1<Alternate<AF4>>,
            PH4<Alternate<AF4>>
        ]

        SDA: [
            PB11<Alternate<AF4>>,
            PF0<Alternate<AF4>>,
            PH5<Alternate<AF4>>
        ]

    I2C3:
        SCL: [
            PA8<Alternate<AF4>>,
            PH7<Alternate<AF4>>
        ]

        SDA: [
            PC9<Alternate<AF4>>,
            PH8<Alternate<AF4>>
        ]

    I2C4:
        SCL: [
            PD12<Alternate<AF4>>,
            PF14<Alternate<AF4>>,
            PH11<Alternate<AF4>>,
            PB6<Alternate<AF6>>,
            PB8<Alternate<AF6>>
        ]

        SDA: [
            PB7<Alternate<AF6>>,
            PB9<Alternate<AF6>>,
            PD13<Alternate<AF4>>,
            PF15<Alternate<AF4>>,
            PH12<Alternate<AF4>>
        ]
}

i2c!(
    I2C1: (i2c1, I2c1, pclk1),
    I2C2: (i2c2, I2c2, pclk1),
    I2C3: (i2c3, I2c3, pclk1),
    I2C4: (i2c4, I2c4, pclk4),
);
