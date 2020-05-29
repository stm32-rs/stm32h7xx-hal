//! DAC Traits

/// A single DAC output channel.
///
/// This is the same trait definition as found in some other stm32 HALs.
pub trait DacOut<V> {
    fn set_value(&mut self, val: V);
    fn get_value(&mut self) -> V;
}
