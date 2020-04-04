use crate::hal::blocking::delay::DelayUs;

/// A single DAC channel.
///
/// This is the same trait definition as found in some other stm32 HALs.
pub trait DacPin {
    fn enable(&mut self);
    fn calibrate<T>(&mut self, delay: &mut T)
    where
        T: DelayUs<u32>;
}
