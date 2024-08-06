use core::convert::Infallible;

use super::{
    dynamic::PinModeError, marker, DynamicPin, ErasedPin, Input, OpenDrain,
    Output, PartiallyErasedPin, Pin, PinMode, PinState,
};

// Implementations for `Pin`

impl<const P: char, const N: u8, MODE> embedded_hal_02::digital::v2::OutputPin
    for Pin<P, N, Output<MODE>>
{
    type Error = Infallible;

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

impl<const P: char, const N: u8, MODE>
    embedded_hal_02::digital::v2::StatefulOutputPin
    for Pin<P, N, Output<MODE>>
{
    #[inline(always)]
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        Ok(self.is_set_high())
    }

    #[inline(always)]
    fn is_set_low(&self) -> Result<bool, Self::Error> {
        Ok(self.is_set_low())
    }
}

impl<const P: char, const N: u8, MODE>
    embedded_hal_02::digital::v2::ToggleableOutputPin
    for Pin<P, N, Output<MODE>>
{
    type Error = Infallible;

    #[inline(always)]
    fn toggle(&mut self) -> Result<(), Self::Error> {
        self.toggle();
        Ok(())
    }
}

impl<const P: char, const N: u8, MODE> embedded_hal_02::digital::v2::InputPin
    for Pin<P, N, MODE>
where
    MODE: marker::Readable,
{
    type Error = Infallible;

    #[inline(always)]
    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(self.is_high())
    }

    #[inline(always)]
    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(self.is_low())
    }
}

impl<const P: char, const N: u8> embedded_hal_02::digital::v2::IoPin<Self, Self>
    for Pin<P, N, Output<OpenDrain>>
{
    type Error = Infallible;
    fn into_input_pin(self) -> Result<Self, Self::Error> {
        Ok(self)
    }
    fn into_output_pin(mut self, state: PinState) -> Result<Self, Self::Error> {
        self.set_state(state);
        Ok(self)
    }
}

impl<const P: char, const N: u8, Otype>
    embedded_hal_02::digital::v2::IoPin<Pin<P, N, Input>, Self>
    for Pin<P, N, Output<Otype>>
where
    Output<Otype>: PinMode,
{
    type Error = Infallible;
    fn into_input_pin(self) -> Result<Pin<P, N, Input>, Self::Error> {
        Ok(self.into_input())
    }
    fn into_output_pin(mut self, state: PinState) -> Result<Self, Self::Error> {
        self.set_state(state);
        Ok(self)
    }
}

impl<const P: char, const N: u8, Otype>
    embedded_hal_02::digital::v2::IoPin<Self, Pin<P, N, Output<Otype>>>
    for Pin<P, N, Input>
where
    Output<Otype>: PinMode,
{
    type Error = Infallible;
    fn into_input_pin(self) -> Result<Self, Self::Error> {
        Ok(self)
    }
    fn into_output_pin(
        mut self,
        state: PinState,
    ) -> Result<Pin<P, N, Output<Otype>>, Self::Error> {
        self._set_state(state);
        Ok(self.into_mode())
    }
}

// Implementations for `ErasedPin`

impl<MODE> embedded_hal_02::digital::v2::OutputPin for ErasedPin<Output<MODE>> {
    type Error = core::convert::Infallible;

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

impl<MODE> embedded_hal_02::digital::v2::StatefulOutputPin
    for ErasedPin<Output<MODE>>
{
    #[inline(always)]
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        Ok(self.is_set_high())
    }

    #[inline(always)]
    fn is_set_low(&self) -> Result<bool, Self::Error> {
        Ok(self.is_set_low())
    }
}

impl<MODE> embedded_hal_02::digital::v2::ToggleableOutputPin
    for ErasedPin<Output<MODE>>
{
    type Error = Infallible;

    #[inline(always)]
    fn toggle(&mut self) -> Result<(), Self::Error> {
        self.toggle();
        Ok(())
    }
}

impl<MODE> embedded_hal_02::digital::v2::InputPin for ErasedPin<MODE>
where
    MODE: marker::Readable,
{
    type Error = core::convert::Infallible;

    #[inline(always)]
    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(self.is_high())
    }

    #[inline(always)]
    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(self.is_low())
    }
}

// Implementations for `PartiallyErasedPin`

impl<const P: char, MODE> embedded_hal_02::digital::v2::OutputPin
    for PartiallyErasedPin<P, Output<MODE>>
{
    type Error = Infallible;

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

impl<const P: char, MODE> embedded_hal_02::digital::v2::StatefulOutputPin
    for PartiallyErasedPin<P, Output<MODE>>
{
    #[inline(always)]
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        Ok(self.is_set_high())
    }

    #[inline(always)]
    fn is_set_low(&self) -> Result<bool, Self::Error> {
        Ok(self.is_set_low())
    }
}

impl<const P: char, MODE> embedded_hal_02::digital::v2::ToggleableOutputPin
    for PartiallyErasedPin<P, Output<MODE>>
{
    type Error = Infallible;

    #[inline(always)]
    fn toggle(&mut self) -> Result<(), Self::Error> {
        self.toggle();
        Ok(())
    }
}

impl<const P: char, MODE> embedded_hal_02::digital::v2::InputPin
    for PartiallyErasedPin<P, MODE>
where
    MODE: marker::Readable,
{
    type Error = Infallible;

    #[inline(always)]
    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(self.is_high())
    }

    #[inline(always)]
    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(self.is_low())
    }
}

// Implementations for `DynamicPin`

impl<const P: char, const N: u8> embedded_hal_02::digital::v2::OutputPin
    for DynamicPin<P, N>
{
    type Error = PinModeError;
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.set_high()
    }
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.set_low()
    }
}

impl<const P: char, const N: u8> embedded_hal_02::digital::v2::InputPin
    for DynamicPin<P, N>
{
    type Error = PinModeError;
    fn is_high(&self) -> Result<bool, Self::Error> {
        self.is_high()
    }
    fn is_low(&self) -> Result<bool, Self::Error> {
        self.is_low()
    }
}
