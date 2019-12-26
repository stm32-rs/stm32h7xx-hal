//! Inter Integrated Circuit implementation

use crate::gpio::gpioa::PA8;
use crate::gpio::gpiob::{PB10, PB11, PB6, PB7, PB8, PB9};
use crate::gpio::gpioc::PC9;
use crate::gpio::gpiod::{PD12, PD13};
use crate::gpio::gpiof::{PF0, PF1, PF14, PF15};
use crate::gpio::gpioh::{PH11, PH12, PH4, PH5, PH7, PH8};
use crate::gpio::{Alternate, AF4, AF6};
use crate::hal::blocking::i2c::{Read, Write, WriteRead};
use crate::rcc::Ccdr;
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
pub unsafe trait PinScl<I2C> {}

/// A trait to represent the SDL Pin of an I2C Port
pub unsafe trait PinSda<I2C> {}

pub trait Pins<I2C> {}

impl<I2C, SCL, SDA> Pins<I2C> for (SCL, SDA)
where
    SCL: PinScl<I2C>,
    SDA: PinSda<I2C>,
{
}

#[derive(Debug)]
pub struct I2c<I2C, PINS> {
    i2c: I2C,
    pins: PINS,
}

pub trait I2cExt<I2C>: Sized {
    fn i2c<PINS, F>(self, pins: PINS, freq: F, ccdr: &Ccdr) -> I2c<I2C, PINS>
    where
        PINS: Pins<I2C>,
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
        if $i2c.isr.read().txe().bit_is_clear() {
            $i2c.isr.write(|w| w.txe().set_bit());
        }
    };
}

// Wait until specified flag is raised, or an error condition occours
macro_rules! busy_wait {
    ($i2c:expr, $flag:ident) => {
        loop {
            let isr = $i2c.isr.read();

            if isr.berr().bit_is_set() {
                $i2c.icr.write(|w| w.berrcf().set_bit());
                return Err(Error::Bus);
            } else if isr.arlo().bit_is_set() {
                $i2c.icr.write(|w| w.arlocf().set_bit());
                return Err(Error::Arbitration);
            } else if isr.nackf().bit_is_set() {
                $i2c.icr.write(|w| w.stopcf().set_bit().nackcf().set_bit());
                flush_txdr!($i2c);
                return Err(Error::NotAcknowledge);
            } else if isr.$flag().bit_is_set() {
                break;
            } else {
                // try again
            }
        }
    };
}

macro_rules! i2c {
    ($($I2CX:ident: ($i2cX:ident, $i2cXen:ident, $i2cXrst:ident, $apbXenr:ident, $apbXrstr:ident, $pclkX:ident),)+) => {
        $(
            impl<PINS> I2c<$I2CX, PINS> {
                /// Basically a new function for an I2C peripheral
                pub fn $i2cX<F> (
                    i2c: $I2CX,
                    pins: PINS,
                    freq: F,
                    ccdr: &Ccdr
                ) -> Self where
                    F: Into<Hertz>,
                    PINS: Pins<$I2CX>,
                {
                    ccdr.rb.$apbXenr.modify(|_, w| w.$i2cXen().set_bit());
                    ccdr.rb.$apbXrstr.modify(|_, w| w.$i2cXrst().set_bit());
                    ccdr.rb.$apbXrstr.modify(|_, w| w.$i2cXrst().clear_bit());

                    let freq = freq.into().0;

                    assert!(freq <= 1_000_000);

                    let i2cclk = ccdr.clocks.$pclkX().0;

                    // Refer to RM0433 Rev 6 - Figure 539 for this:
                    // Clear PE bit in I2C_CR1
                    unsafe { &(*I2C1::ptr()).cr1.modify(|_, w| w.pe().clear_bit())};

                    // Enable the Analog Noise Filter by setting ANFOFF (Analog Noise Filter OFF) to 0
                    // This is usually enabled by default but you never know
                    unsafe { &(*I2C1::ptr()).cr1.modify(|_, w| w.anfoff().clear_bit())};

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

                    I2c { i2c, pins}

                }

                /// Releases the I2C peripheral and associated pins
                pub fn free(self) -> ($I2CX, PINS) {
                    (self.i2c, self.pins)
                }
            }

            impl I2cExt<$I2CX> for $I2CX {
                fn i2c<PINS, F>(self,
                                pins:PINS,
                                freq:F,
                                ccdr: &Ccdr) -> I2c<$I2CX, PINS>
                where
                    PINS: Pins<$I2CX>,
                    F: Into<Hertz>
                {
                    I2c::$i2cX(self, pins, freq, ccdr)
                }
            }

            impl<PINS> Write for I2c<$I2CX, PINS> {
                type Error = Error;

                fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
                    // TODO support transfers of more than 255 bytes
                    assert!(bytes.len() < 256 && bytes.len() > 0);

                    // START and prepare to send `bytes`
                    self.i2c.cr2.write(|w| {
                        w.start()
                            .set_bit()
                            .sadd()
                            .bits(u16(addr << 1 | 0))
                            .add10().clear_bit()
                            .rd_wrn()
                            .clear_bit()
                            .nbytes()
                            .bits(bytes.len() as u8)
                            .autoend()
                            .set_bit()
                    });

                    for byte in bytes {
                        // Wait until we are allowed to send data
                        // (START has been ACKed or last byte when
                        // through)
                        busy_wait!(self.i2c, txis);
                        // Put byte on the wire
                        self.i2c.txdr.write(|w| w.txdata().bits(*byte));
                    }
                    // automatic STOP

                    Ok(())
                }
            }

            impl<PINS> WriteRead for I2c<$I2CX, PINS> {
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

                    // TODO do we have to explicitly wait here if the bus is busy (e.g. another
                    // master is communicating)?

                    // START and prepare to send `bytes`
                    self.i2c.cr2.write(|w| {
                        w.start()
                            .set_bit()
                            .sadd()
                            .bits(u16(addr << 1 | 0))
                            .add10().clear_bit()
                            .rd_wrn()
                            .clear_bit()
                            .nbytes()
                            .bits(bytes.len() as u8)
                            .autoend()
                            .clear_bit()

                    });

                    for byte in bytes {
                        // Wait until we are allowed to send data
                        // (START has been ACKed or last byte when
                        // through)
                        busy_wait!(self.i2c, txis);
                        // Put byte on the wire
                        self.i2c.txdr.write(|w| w.txdata().bits(*byte));
                    }

                    // Wait until the last transmission is finished
                    busy_wait!(self.i2c, tc);

                    // reSTART and prepare to receive bytes into `buffer`
                    self.i2c.cr2.write(|w| {
                        w.sadd()
                            .bits(u16(addr << 1 | 1))
                            .add10().clear_bit()
                            .rd_wrn()
                            .set_bit()
                            .nbytes()
                            .bits(buffer.len() as u8)
                            .start()
                            .set_bit()
                            .autoend()
                            .set_bit()
                    });

                    for byte in buffer {
                        // Wait until we have received something
                        busy_wait!(self.i2c, rxne);

                        *byte = self.i2c.rxdr.read().rxdata().bits();
                    }

                    // automatic STOP

                    Ok(())
                }
            }

            impl<PINS> Read for I2c<$I2CX, PINS> {
            type Error = Error;

            fn read(
                &mut self,
                addr: u8,
                buffer: &mut [u8],
            ) -> Result<(), Error> {
                // TODO support transfers of more than 255 bytes
                assert!(buffer.len() < 256 && buffer.len() > 0);

                // TODO do we have to explicitly wait here if the bus is busy (e.g. another
                // master is communicating)?

                // reSTART and prepare to receive bytes into `buffer`
                self.i2c.cr2.write(|w| {
                    w.sadd()
                        .bits((addr << 1 | 0) as u16)
                        .rd_wrn()
                        .set_bit()
                        .nbytes()
                        .bits(buffer.len() as u8)
                        .start()
                        .set_bit()
                        .autoend()
                        .set_bit()
                });

                for byte in buffer {
                    // Wait until we have received something
                    busy_wait!(self.i2c, rxne);

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
                unsafe impl PinScl<$I2CX> for $SCL {}
            )*
            $(
                unsafe impl PinSda<$I2CX> for $SDA {}
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
    I2C1: (i2c1, i2c1en, i2c1rst, apb1lenr, apb1lrstr, pclk1),
    I2C2: (i2c2, i2c2en, i2c2rst, apb1lenr, apb1lrstr, pclk1),
    I2C3: (i2c3, i2c3en, i2c3rst, apb1lenr, apb1lrstr, pclk1),
    I2C4: (i2c4, i2c4en, i2c4rst, apb4enr, apb4rstr, pclk4),
);
