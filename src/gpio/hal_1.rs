use super::{
    dynamic::PinModeError, marker, DynamicPin, ErasedPin, Output,
    PartiallyErasedPin, Pin,
};

use embedded_hal::digital::{ErrorKind, ErrorType};
use embedded_hal::digital::{InputPin, OutputPin, StatefulOutputPin};

/// Error type for GPIO
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub enum GpioError {
    /// [DynamicPin] For operations unsupported in current mode
    IncorrectMode,
    /// Error
    Other,
}
impl embedded_hal::digital::Error for GpioError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}
impl From<PinModeError> for GpioError {
    fn from(_: PinModeError) -> GpioError {
        GpioError::IncorrectMode
    }
}

// Implementations for `Pin`

impl<const P: char, const N: u8, M> ErrorType for Pin<P, N, M> {
    type Error = GpioError;
}

impl<const P: char, const N: u8, MODE> OutputPin for Pin<P, N, Output<MODE>> {
    #[inline(always)]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_high();
        Ok(())
    }

    #[inline(always)]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set_low();
        Ok(())
    }
}

impl<const P: char, const N: u8, MODE> StatefulOutputPin
    for Pin<P, N, Output<MODE>>
{
    #[inline(always)]
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(!self._is_set_low())
    }

    #[inline(always)]
    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(self._is_set_low())
    }
    #[inline(always)]
    fn toggle(&mut self) -> Result<(), Self::Error> {
        self.toggle();
        Ok(())
    }
}

impl<const P: char, const N: u8, MODE> InputPin for Pin<P, N, MODE>
where
    MODE: marker::Readable,
{
    #[inline(always)]
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(!self._is_low())
    }

    #[inline(always)]
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(self._is_low())
    }
}

// Implementations for `ErasedPin`

impl<M> ErrorType for ErasedPin<M> {
    type Error = GpioError;
}

impl<MODE> OutputPin for ErasedPin<Output<MODE>> {
    #[inline(always)]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_high();
        Ok(())
    }

    #[inline(always)]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set_low();
        Ok(())
    }
}

impl<MODE> StatefulOutputPin for ErasedPin<Output<MODE>> {
    #[inline(always)]
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(ErasedPin::is_set_high(self))
    }

    #[inline(always)]
    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(ErasedPin::is_set_low(self))
    }
    #[inline(always)]
    fn toggle(&mut self) -> Result<(), Self::Error> {
        self.toggle();
        Ok(())
    }
}

impl<MODE> InputPin for ErasedPin<MODE>
where
    MODE: marker::Readable,
{
    #[inline(always)]
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(ErasedPin::is_high(self))
    }

    #[inline(always)]
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(ErasedPin::is_low(self))
    }
}

// Implementations for `PartiallyErasedPin`

impl<const P: char, M> ErrorType for PartiallyErasedPin<P, M> {
    type Error = GpioError;
}

impl<const P: char, MODE> OutputPin for PartiallyErasedPin<P, Output<MODE>> {
    #[inline(always)]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_high();
        Ok(())
    }

    #[inline(always)]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set_low();
        Ok(())
    }
}

impl<const P: char, MODE> StatefulOutputPin
    for PartiallyErasedPin<P, Output<MODE>>
{
    #[inline(always)]
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(PartiallyErasedPin::is_set_high(self))
    }

    #[inline(always)]
    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(PartiallyErasedPin::is_set_low(self))
    }
    #[inline(always)]
    fn toggle(&mut self) -> Result<(), Self::Error> {
        self.toggle();
        Ok(())
    }
}

impl<const P: char, MODE> InputPin for PartiallyErasedPin<P, MODE>
where
    MODE: marker::Readable,
{
    #[inline(always)]
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(PartiallyErasedPin::is_high(self))
    }

    #[inline(always)]
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(PartiallyErasedPin::is_low(self))
    }
}

// Implementations for `DynamicPin`

impl<const P: char, const N: u8> ErrorType for DynamicPin<P, N> {
    type Error = GpioError;
}

impl<const P: char, const N: u8> OutputPin for DynamicPin<P, N> {
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_high()?;
        Ok(())
    }
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set_low()?;
        Ok(())
    }
}

impl<const P: char, const N: u8> InputPin for DynamicPin<P, N> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(DynamicPin::is_high(self)?)
    }
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(DynamicPin::is_low(self)?)
    }
}
