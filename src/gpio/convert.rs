use super::*;

impl<const P: char, const N: u8, const A: u8>
    Pin<P, N, Alternate<A, PushPull>>
{
    /// Turns pin alternate configuration pin into open drain
    pub fn set_open_drain(self) -> Pin<P, N, Alternate<A, OpenDrain>> {
        self.into_mode()
    }
}

impl<
        const P: char,
        const N: u8,
        MODE: PinMode + marker::NotAlt,
        const A: u8,
        Otype,
    > From<Pin<P, N, MODE>> for Pin<P, N, Alternate<A, Otype>>
where
    Alternate<A, Otype>: PinMode,
    Self: marker::IntoAf<A>,
{
    #[inline(always)]
    fn from(f: Pin<P, N, MODE>) -> Self {
        f.into_mode()
    }
}

impl<const P: char, const N: u8, const A: u8, const B: u8>
    From<Pin<P, N, Alternate<B, PushPull>>>
    for Pin<P, N, Alternate<A, OpenDrain>>
where
    Self: marker::IntoAf<A>,
{
    #[inline(always)]
    fn from(f: Pin<P, N, Alternate<B, PushPull>>) -> Self {
        f.into_mode()
    }
}

impl<const P: char, const N: u8, Otype> From<Pin<P, N, Output<Otype>>>
    for Pin<P, N, Input>
where
    Output<Otype>: PinMode,
{
    #[inline(always)]
    fn from(f: Pin<P, N, Output<Otype>>) -> Self {
        f.into_mode()
    }
}

impl<const P: char, const N: u8> From<Pin<P, N, Analog>> for Pin<P, N, Input> {
    #[inline(always)]
    fn from(f: Pin<P, N, Analog>) -> Self {
        f.into_mode()
    }
}

impl<const P: char, const N: u8, const A: u8, Otype, MODE>
    From<Pin<P, N, Alternate<A, Otype>>> for Pin<P, N, MODE>
where
    Alternate<A, Otype>: PinMode,
    MODE: PinMode + marker::NotAlt,
{
    #[inline(always)]
    fn from(f: Pin<P, N, Alternate<A, Otype>>) -> Self {
        f.into_mode()
    }
}

impl<const P: char, const N: u8, Otype> From<Pin<P, N, Input>>
    for Pin<P, N, Output<Otype>>
where
    Output<Otype>: PinMode,
{
    #[inline(always)]
    fn from(f: Pin<P, N, Input>) -> Self {
        f.into_mode()
    }
}

impl<const P: char, const N: u8, Otype> From<Pin<P, N, Analog>>
    for Pin<P, N, Output<Otype>>
where
    Output<Otype>: PinMode,
{
    #[inline(always)]
    fn from(f: Pin<P, N, Analog>) -> Self {
        f.into_mode()
    }
}

impl<const P: char, const N: u8> From<Pin<P, N, Output<PushPull>>>
    for Pin<P, N, Output<OpenDrain>>
{
    #[inline(always)]
    fn from(f: Pin<P, N, Output<PushPull>>) -> Self {
        f.into_mode()
    }
}

impl<const P: char, const N: u8> From<Pin<P, N, Output<OpenDrain>>>
    for Pin<P, N, Output<PushPull>>
{
    #[inline(always)]
    fn from(f: Pin<P, N, Output<OpenDrain>>) -> Self {
        f.into_mode()
    }
}

impl<const P: char, const N: u8> From<Pin<P, N, Input>> for Pin<P, N, Analog> {
    #[inline(always)]
    fn from(f: Pin<P, N, Input>) -> Self {
        f.into_mode()
    }
}

impl<const P: char, const N: u8, Otype> From<Pin<P, N, Output<Otype>>>
    for Pin<P, N, Analog>
where
    Output<Otype>: PinMode,
{
    #[inline(always)]
    fn from(f: Pin<P, N, Output<Otype>>) -> Self {
        f.into_mode()
    }
}

macro_rules! af {
    ($($into_alternate_af:ident: $A:literal;)+) => {
        $(
            #[doc="Configures the pin to operate in alternate function "]
            #[doc=stringify!($A)]
            #[doc=" mode"]
            #[deprecated(since = "0.12.0", note = "Use the .into_alternate() method instead")]
            pub fn $into_alternate_af(self) -> Pin<P, N, Alternate<$A, PushPull>> where
                Self: marker::IntoAf<$A>,
            {
                self.into_alternate::<$A>()
            }
        )+
    }
}

impl<const P: char, const N: u8, MODE: PinMode> Pin<P, N, MODE> {
    /// Configures the pin to operate alternate mode
    pub fn into_alternate<const A: u8>(
        self,
    ) -> Pin<P, N, Alternate<A, PushPull>>
    where
        Self: marker::IntoAf<A>,
    {
        self.into_mode()
    }

    af! {
        into_alternate_af0: 0;
        into_alternate_af1: 1;
        into_alternate_af2: 2;
        into_alternate_af3: 3;
        into_alternate_af4: 4;
        into_alternate_af5: 5;
        into_alternate_af6: 6;
        into_alternate_af7: 7;
        into_alternate_af8: 8;
        into_alternate_af9: 9;
        into_alternate_af10: 10;
        into_alternate_af11: 11;
        into_alternate_af12: 12;
        into_alternate_af13: 13;
        into_alternate_af14: 14;
        into_alternate_af15: 15;
    }

    /// Configures the pin to operate in alternate open drain mode
    #[allow(path_statements)]
    pub fn into_alternate_open_drain<const A: u8>(
        self,
    ) -> Pin<P, N, Alternate<A, OpenDrain>>
    where
        Self: marker::IntoAf<A>,
    {
        self.into_mode()
    }

    /// Configures the pin to operate as a input pin
    pub fn into_input(self) -> Pin<P, N, Input> {
        self.into_mode()
    }

    /// Configures the pin to operate as a floating input pin
    pub fn into_floating_input(self) -> Pin<P, N, Input> {
        self.into_mode().internal_resistor(Pull::None)
    }

    /// Configures the pin to operate as a pulled down input pin
    pub fn into_pull_down_input(self) -> Pin<P, N, Input> {
        self.into_mode().internal_resistor(Pull::Down)
    }

    /// Configures the pin to operate as a pulled up input pin
    pub fn into_pull_up_input(self) -> Pin<P, N, Input> {
        self.into_mode().internal_resistor(Pull::Up)
    }

    /// Configures the pin to operate as an open drain output pin
    /// Initial state will be low.
    pub fn into_open_drain_output(self) -> Pin<P, N, Output<OpenDrain>> {
        self.into_mode()
    }

    /// Configures the pin to operate as an open-drain output pin.
    /// `initial_state` specifies whether the pin should be initially high or low.
    pub fn into_open_drain_output_in_state(
        mut self,
        initial_state: PinState,
    ) -> Pin<P, N, Output<OpenDrain>> {
        self._set_state(initial_state);
        self.into_mode()
    }

    /// Configures the pin to operate as an push pull output pin
    /// Initial state will be low.
    pub fn into_push_pull_output(mut self) -> Pin<P, N, Output<PushPull>> {
        self._set_low();
        self.into_mode()
    }

    /// Configures the pin to operate as an push-pull output pin.
    /// `initial_state` specifies whether the pin should be initially high or low.
    pub fn into_push_pull_output_in_state(
        mut self,
        initial_state: PinState,
    ) -> Pin<P, N, Output<PushPull>> {
        self._set_state(initial_state);
        self.into_mode()
    }

    /// Configures the pin to operate as an analog input pin
    pub fn into_analog(self) -> Pin<P, N, Analog> {
        self.into_mode()
    }

    /// Configures the pin as a pin that can change between input
    /// and output without changing the type. It starts out
    /// as a floating input
    pub fn into_dynamic(self) -> DynamicPin<P, N> {
        self.into_floating_input();
        DynamicPin::new(Dynamic::InputFloating)
    }

    /// Puts `self` into mode `M`.
    ///
    /// This violates the type state constraints from `MODE`, so callers must
    /// ensure they use this properly.
    #[inline(always)]
    pub(super) fn mode<M: PinMode>(&mut self) {
        let offset = 2 * N;
        unsafe {
            if MODE::OTYPER != M::OTYPER {
                if let Some(otyper) = M::OTYPER {
                    (*Gpio::<P>::ptr()).otyper().modify(|r, w| {
                        w.bits(r.bits() & !(0b1 << N) | (otyper << N))
                    });
                }
            }

            if MODE::AFR != M::AFR {
                if let Some(afr) = M::AFR {
                    if N < 8 {
                        let offset2 = 4 * { N };
                        (*Gpio::<P>::ptr()).afrl().modify(|r, w| {
                            w.bits(
                                (r.bits() & !(0b1111 << offset2))
                                    | (afr << offset2),
                            )
                        });
                    } else {
                        let offset2 = 4 * { N - 8 };
                        (*Gpio::<P>::ptr()).afrh().modify(|r, w| {
                            w.bits(
                                (r.bits() & !(0b1111 << offset2))
                                    | (afr << offset2),
                            )
                        });
                    }
                }
            }

            if MODE::MODER != M::MODER {
                (*Gpio::<P>::ptr()).moder().modify(|r, w| {
                    w.bits(
                        (r.bits() & !(0b11 << offset)) | (M::MODER << offset),
                    )
                });
            }
        }
    }

    #[inline(always)]
    /// Converts pin into specified mode
    pub fn into_mode<M: PinMode>(mut self) -> Pin<P, N, M> {
        self.mode::<M>();
        Pin::new()
    }
}

impl<const P: char, const N: u8, MODE> Pin<P, N, MODE>
where
    MODE: PinMode,
{
    fn with_mode<M, F, R>(&mut self, f: F) -> R
    where
        M: PinMode,
        F: FnOnce(&mut Pin<P, N, M>) -> R,
    {
        self.mode::<M>(); // change physical mode, without changing typestate

        // This will reset the pin back to the original mode when dropped.
        // (so either when `with_mode` returns or when `f` unwinds)
        let mut resetti = ResetMode::<P, N, M, MODE>::new();

        f(&mut resetti.pin)
    }

    /// Temporarily configures this pin as a input.
    ///
    /// The closure `f` is called with the reconfigured pin. After it returns,
    /// the pin will be configured back.
    pub fn with_input<R>(
        &mut self,
        f: impl FnOnce(&mut Pin<P, N, Input>) -> R,
    ) -> R {
        self.with_mode(f)
    }

    /// Temporarily configures this pin as an analog pin.
    ///
    /// The closure `f` is called with the reconfigured pin. After it returns,
    /// the pin will be configured back.
    pub fn with_analog<R>(
        &mut self,
        f: impl FnOnce(&mut Pin<P, N, Analog>) -> R,
    ) -> R {
        self.with_mode(f)
    }

    /// Temporarily configures this pin as an open drain output.
    ///
    /// The closure `f` is called with the reconfigured pin. After it returns,
    /// the pin will be configured back.
    /// The value of the pin after conversion is undefined. If you
    /// want to control it, use `with_open_drain_output_in_state`
    pub fn with_open_drain_output<R>(
        &mut self,
        f: impl FnOnce(&mut Pin<P, N, Output<OpenDrain>>) -> R,
    ) -> R {
        self.with_mode(f)
    }

    /// Temporarily configures this pin as an open drain output .
    ///
    /// The closure `f` is called with the reconfigured pin. After it returns,
    /// the pin will be configured back.
    /// Note that the new state is set slightly before conversion
    /// happens. This can cause a short output glitch if switching
    /// between output modes
    pub fn with_open_drain_output_in_state<R>(
        &mut self,
        state: PinState,
        f: impl FnOnce(&mut Pin<P, N, Output<OpenDrain>>) -> R,
    ) -> R {
        self._set_state(state);
        self.with_mode(f)
    }

    /// Temporarily configures this pin as a push-pull output.
    ///
    /// The closure `f` is called with the reconfigured pin. After it returns,
    /// the pin will be configured back.
    /// The value of the pin after conversion is undefined. If you
    /// want to control it, use `with_push_pull_output_in_state`
    pub fn with_push_pull_output<R>(
        &mut self,
        f: impl FnOnce(&mut Pin<P, N, Output<PushPull>>) -> R,
    ) -> R {
        self.with_mode(f)
    }

    /// Temporarily configures this pin as a push-pull output.
    ///
    /// The closure `f` is called with the reconfigured pin. After it returns,
    /// the pin will be configured back.
    /// Note that the new state is set slightly before conversion
    /// happens. This can cause a short output glitch if switching
    /// between output modes
    pub fn with_push_pull_output_in_state<R>(
        &mut self,
        state: PinState,
        f: impl FnOnce(&mut Pin<P, N, Output<PushPull>>) -> R,
    ) -> R {
        self._set_state(state);
        self.with_mode(f)
    }
}

/// Wrapper around a pin that transitions the pin to mode ORIG when dropped
struct ResetMode<const P: char, const N: u8, CURRENT: PinMode, ORIG: PinMode> {
    pub pin: Pin<P, N, CURRENT>,
    _mode: PhantomData<ORIG>,
}
impl<const P: char, const N: u8, CURRENT: PinMode, ORIG: PinMode>
    ResetMode<P, N, CURRENT, ORIG>
{
    fn new() -> Self {
        Self {
            pin: Pin::new(),
            _mode: PhantomData,
        }
    }
}
impl<const P: char, const N: u8, CURRENT: PinMode, ORIG: PinMode> Drop
    for ResetMode<P, N, CURRENT, ORIG>
{
    fn drop(&mut self) {
        self.pin.mode::<ORIG>();
    }
}

/// Marker trait for valid pin modes (type state).
///
/// This trait is sealed and cannot be implemented by outside types
pub trait PinMode: crate::Sealed {
    // These constants are used to implement the pin configuration code.
    // They are not part of public API.

    #[doc(hidden)]
    const MODER: u32 = u32::MAX;
    #[doc(hidden)]
    const OTYPER: Option<u32> = None;
    #[doc(hidden)]
    const AFR: Option<u32> = None;
}

impl crate::Sealed for Input {}
impl PinMode for Input {
    const MODER: u32 = 0b00;
}

impl crate::Sealed for Analog {}
impl PinMode for Analog {
    const MODER: u32 = 0b11;
}

impl<Otype> crate::Sealed for Output<Otype> {}
impl PinMode for Output<OpenDrain> {
    const MODER: u32 = 0b01;
    const OTYPER: Option<u32> = Some(0b1);
}

impl PinMode for Output<PushPull> {
    const MODER: u32 = 0b01;
    const OTYPER: Option<u32> = Some(0b0);
}

impl<const A: u8, Otype> crate::Sealed for Alternate<A, Otype> {}
impl<const A: u8> PinMode for Alternate<A, OpenDrain> {
    const MODER: u32 = 0b10;
    const OTYPER: Option<u32> = Some(0b1);
    const AFR: Option<u32> = Some(A as _);
}

impl<const A: u8> PinMode for Alternate<A, PushPull> {
    const MODER: u32 = 0b10;
    const OTYPER: Option<u32> = Some(0b0);
    const AFR: Option<u32> = Some(A as _);
}
