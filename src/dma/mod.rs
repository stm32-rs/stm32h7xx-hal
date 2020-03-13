//! DMA

// TODO: Remove when merging.
#![warn(clippy::all)]

#[macro_use]
mod macros;
pub mod channel;
pub mod mux;
pub mod safe_transfer;
pub mod stream;

use self::channel::{
    ChannelId, C0, C1, C10, C11, C12, C13, C14, C15, C2, C3, C4, C5, C6, C7,
    C8, C9,
};
use self::mux::request_gen::{
    Disabled as GenDisabled, G0, G1, G2, G3, G4, G5, G6, G7,
};
use self::mux::request_ids::{
    ReqNone, RequestId as RequestIdTrait, RequestIdSome,
};
use self::mux::shared::{MuxIsr, RequestGenIsr};
use self::mux::{
    EgDisabled, EgED as EgEDTrait, EgEnabled, MuxShared, NbReq, OverrunError,
    RequestGenerator, RequestId, SyncDisabled, SyncED as SyncEDTrait,
    SyncEnabled, SyncId, SyncOverrunInterrupt, SyncPolarity,
};
use self::safe_transfer::{
    check_buffer, check_double_buffer, configure_safe_transfer, mut_ptr_memory,
    mut_ptr_peripheral, set_memory_impl, set_peripheral_impl, DoubleBuffer,
    MemoryBufferStatic, Ongoing, Payload, PayloadPort, PeripheralBufferStatic,
    PointerPort, Start, TransferState,
};
use self::stm32::dma1::ST;
use self::stm32::dmamux1::CCR;
use self::stm32::{DMA1, DMA2, RCC};
use self::stream::{
    BufferMode, CircularMode, CurrentTarget, DirectModeErrorInterrupt,
    Disabled, Disabling, Enabled, Error, Event, FifoErrorInterrupt,
    FifoThreshold, FlowController, HalfTransferInterrupt, IntoNum, IsrCleared,
    IsrState as IsrStateTrait, IsrUncleared, M0a, M1a, MBurst, MSize, Minc,
    Ndt, NotDisabled, PBurst, PSize, Pa, Pinc, Pincos, PriorityLevel,
    StreamIsr, TransferCompleteInterrupt, TransferDirection,
    TransferErrorInterrupt, TransferMode, ED as EDTrait,
};
use crate::nb::{self, block, Error as NbError};
use crate::rcc::Ccdr;
use core::convert::{Infallible, TryFrom, TryInto};
use core::marker::PhantomData;
use core::mem;
// TODO: Remove when merging. Necessary for me as I'm using CLion with rust plugin, which doesn't support conditionally imported items yet.
use stm32h7::stm32h743 as stm32;
use stm32h7::stm32h743::DMAMUX1;

/// Marker Trait for DMA peripherals
pub unsafe trait DMATrait: Send {}
unsafe impl DMATrait for DMA1 {}
unsafe impl DMATrait for DMA2 {}

/// DMA Channel
pub struct Channel<CXX, DMA, StreamED, IsrState, ReqId, SyncED, EgED>
where
    CXX: ChannelId<DMA = DMA>,
    DMA: DMATrait,
    StreamED: EDTrait,
    IsrState: IsrStateTrait,
    ReqId: RequestIdTrait,
    SyncED: SyncEDTrait,
    EgED: EgEDTrait,
{
    pub stream: Stream<CXX, DMA, StreamED, IsrState>,
    pub mux: DmaMux<CXX, ReqId, SyncED, EgED>,
}

impl<CXX, DMA, StreamED, IsrState, ReqId, SyncED, EgED>
    Channel<CXX, DMA, StreamED, IsrState, ReqId, SyncED, EgED>
where
    CXX: ChannelId<DMA = DMA>,
    DMA: DMATrait,
    StreamED: EDTrait,
    IsrState: IsrStateTrait,
    ReqId: RequestIdTrait,
    SyncED: SyncEDTrait,
    EgED: EgEDTrait,
{
    /// Exposes the stream as owned value in a closure
    pub fn stream_owned<F, NewStreamED, NewIsrState>(
        self,
        op: F,
    ) -> Channel<CXX, DMA, NewStreamED, NewIsrState, ReqId, SyncED, EgED>
    where
        F: FnOnce(
            Stream<CXX, DMA, StreamED, IsrState>,
        ) -> Stream<CXX, DMA, NewStreamED, NewIsrState>,
        NewStreamED: EDTrait,
        NewIsrState: IsrStateTrait,
    {
        let new_stream = op(self.stream);

        Channel {
            stream: new_stream,
            mux: self.mux,
        }
    }

    /// Exposes the mux as owned value in a closure
    pub fn mux_owned<F, NewReqId, NewSyncED, NewEgED>(
        self,
        op: F,
    ) -> Channel<CXX, DMA, StreamED, IsrState, NewReqId, NewSyncED, NewEgED>
    where
        F: FnOnce(
            DmaMux<CXX, ReqId, SyncED, EgED>,
        ) -> DmaMux<CXX, NewReqId, NewSyncED, NewEgED>,
        NewReqId: RequestIdTrait,
        NewSyncED: SyncEDTrait,
        NewEgED: EgEDTrait,
    {
        let new_mux = op(self.mux);

        Channel {
            stream: self.stream,
            mux: new_mux,
        }
    }
}

/// DMA Stream
pub struct Stream<CXX, DMA, ED, IsrState>
where
    CXX: ChannelId<DMA = DMA>,
    DMA: DMATrait,
    ED: EDTrait,
    IsrState: IsrStateTrait,
{
    /// This field *must not* be mutated using shared references
    rb: &'static ST,
    config_ndt: Ndt,
    _phantom_data: PhantomData<(CXX, DMA, ED, IsrState)>,
}

impl<CXX, DMA> Stream<CXX, DMA, Disabled, IsrCleared>
where
    CXX: ChannelId<DMA = DMA>,
    DMA: DMATrait,
{
    /// Creates an instance of a Stream in initial state.
    ///
    /// Should only be called after RCC-Reset of the DMA.
    fn after_reset(rb: &'static ST) -> Self {
        Stream {
            rb,
            config_ndt: Ndt::default(),
            _phantom_data: PhantomData,
        }
    }
}

impl<CXX, DMA, ED, IsrState> Stream<CXX, DMA, ED, IsrState>
where
    CXX: ChannelId<DMA = DMA>,
    DMA: DMATrait,
    ED: EDTrait,
    IsrState: IsrStateTrait,
{
    /// Returns the id of the Stream
    pub fn id(&self) -> usize {
        CXX::STREAM_ID
    }

    /// Performs a *volatile* read of the stream enable bit
    pub fn is_enabled(&self) -> bool {
        self.rb.cr.read().en().bit_is_set()
    }

    /// Returns the Transfer Complete Interrupt config flag
    pub fn transfer_complete_interrupt(&self) -> TransferCompleteInterrupt {
        self.rb.cr.read().tcie().bit().into()
    }

    /// Sets the Transfer Complete Interrupt config flag
    pub fn set_transfer_complete_interrupt(
        &mut self,
        tc_intrpt: TransferCompleteInterrupt,
    ) {
        self.rb.cr.modify(|_, w| w.tcie().bit(tc_intrpt.into()));
    }

    /// Returns the Half Transfer Interrupt config flag
    pub fn half_transfer_interrupt(&self) -> HalfTransferInterrupt {
        self.rb.cr.read().htie().bit().into()
    }

    /// Sets the Half Transfer Interrupt config flag
    pub fn set_half_transfer_interrupt(
        &mut self,
        ht_intrpt: HalfTransferInterrupt,
    ) {
        self.rb.cr.modify(|_, w| w.htie().bit(ht_intrpt.into()));
    }

    /// Returns the Transfer Error Interrupt config flag
    pub fn transfer_error_interrupt(&self) -> TransferErrorInterrupt {
        self.rb.cr.read().teie().bit().into()
    }

    /// Sets the Transfer Error Interrupt config flag
    pub fn set_transfer_error_interrupt(
        &mut self,
        te_intrpt: TransferErrorInterrupt,
    ) {
        self.rb.cr.modify(|_, w| w.teie().bit(te_intrpt.into()));
    }

    /// Returns the Direct Mode Error Interrupt config flag
    pub fn direct_mode_error_interrupt(&self) -> DirectModeErrorInterrupt {
        self.rb.cr.read().dmeie().bit().into()
    }

    /// Sets the Direct Mode Error Interrupt config flag
    pub fn set_direct_mode_error_interrupt(
        &mut self,
        dme_intrpt: DirectModeErrorInterrupt,
    ) {
        self.rb.cr.modify(|_, w| w.dmeie().bit(dme_intrpt.into()));
    }

    /// Returns the Fifo Error Interrupt config flag
    pub fn fifo_error_interrupt(&self) -> FifoErrorInterrupt {
        self.rb.fcr.read().feie().bit().into()
    }

    /// Sets the Fifo Error Interrupt config flag
    pub fn set_fifo_error_interrupt(&mut self, fe_intrpt: FifoErrorInterrupt) {
        self.rb.fcr.modify(|_, w| w.feie().bit(fe_intrpt.into()));
    }

    /// Returns the Flow Controller
    ///
    /// **Note** (for disabled streams):
    /// This config value gets forced to `Dma`, if `transfer_direction` is `M2M`.
    ///
    /// This invariant are covered in `effective_flow_controller` (defined for disabled streams).
    pub fn flow_controller(&self) -> FlowController {
        self.rb.cr.read().pfctrl().bit().into()
    }

    /// Returns the Transfer Direction
    pub fn transfer_direction(&self) -> TransferDirection {
        self.rb.cr.read().dir().bits().try_into().unwrap()
    }

    /// Returns the Circular Mode
    ///
    /// **Note** (for disabled streams):
    /// This config value gets forced
    ///
    /// - **by hardware** to `Enabled`, if `buffer_mode` is `DoubleBuffer`.
    /// - **by software** to `Disabled` if `transfer_direction` is `M2M`.
    ///
    /// These invariants is covered in `effective_circular_mode`.
    pub fn circular_mode(&self) -> CircularMode {
        self.rb.cr.read().circ().bit().into()
    }

    /// Returns the Peripheral Increment config flag
    pub fn pinc(&self) -> Pinc {
        self.rb.cr.read().pinc().bit().into()
    }

    /// Returns the Memory Increment config flag
    pub fn minc(&self) -> Minc {
        self.rb.cr.read().minc().bit().into()
    }

    /// Returns the Peripheral Size
    pub fn p_size(&self) -> PSize {
        self.rb.cr.read().psize().bits().try_into().unwrap()
    }

    /// Returns the Memory Size
    ///
    /// **Note** (for disabled streams):
    /// This config value gets forced to `p_size`, if `transfer_mode` is `Direct`.
    ///
    /// This invariant is covered in `effective_m_size`.
    pub fn m_size(&self) -> MSize {
        self.rb.cr.read().msize().bits().try_into().unwrap()
    }

    /// Returns the Peripheral Increment Offset
    ///
    /// **Note** (for disabled and enabled streams):
    ///
    /// - This config value has no meaning, if `pinc` is `Fixed`.
    /// - This config value gets forced to `PSize`, if `p_burst` is not `Single`.
    ///
    /// These invariants are covered in `effective_pincos` (defined for disabled and enabled streams).
    pub fn pincos(&self) -> Pincos {
        self.rb.cr.read().pincos().bit().into()
    }

    /// Returns the Priority Level
    pub fn priority_level(&self) -> PriorityLevel {
        self.rb.cr.read().pl().bits().try_into().unwrap()
    }

    /// Returns the Buffer Mode
    ///
    /// **Note** (for disabled streams):
    /// This config value gets forced to `Regular` **by software** (to avoid TransferError-Flag), if
    ///
    /// - `TransferDirection` is `M2M` or
    /// - `FlowController` is `Peripheral`
    ///
    /// These invariants are covered in `effective_buffer_mode` (defined for disabled streams).
    pub fn buffer_mode(&self) -> BufferMode {
        self.rb.cr.read().dbm().bit().into()
    }

    /// Returns the Current Target
    ///
    /// **Note** (for disabled and enabled streams):
    /// This config value has no meaning, if `BufferMode` is `Regular`.
    ///
    /// This invariant is covered in `effective_current_target`.
    pub fn current_target(&self) -> CurrentTarget {
        self.rb.cr.read().ct().bit().into()
    }

    /// Returns the effective Current Target
    ///
    /// Read the documentation of `current_target` for more details.
    ///
    /// **Note**: If this config value has no meaning (because `BufferMode` is `Regular`),
    /// the method returns `CurrentTarget::M0a`.
    pub fn effective_current_target(&self) -> CurrentTarget {
        if self.buffer_mode() == BufferMode::Regular {
            CurrentTarget::M0a
        } else {
            self.current_target()
        }
    }

    /// Returns the Peripheral Burst config flag
    ///
    /// **Note** (for disabled streams):
    /// This config value gets forced to `Single`, if `transfer_mode` is `Direct`.
    ///
    /// This invariant is covered in `effective_p_burst` (defined for disabled streams).
    pub fn p_burst(&self) -> PBurst {
        self.rb.cr.read().pburst().bits().try_into().unwrap()
    }

    /// Returns the Memory Burst config flag
    ///
    /// **Note** (for disabled streams):
    /// This config value gets forced to `Single`, if `transfer_mode` is `Direct`.
    ///
    /// This invariant is covered in `effective_m_burst` (defined for disabled streams).
    pub fn m_burst(&self) -> MBurst {
        self.rb.cr.read().mburst().bits().try_into().unwrap()
    }

    /// Returns the content of the NDT register
    pub fn ndt(&self) -> Ndt {
        self.rb.ndtr.read().ndt().bits().into()
    }

    /// Sets the content of the NDT register
    pub fn configured_ndt(&self) -> Ndt {
        self.config_ndt
    }

    /// Returns the Peripheral Address
    pub fn pa(&self) -> Pa {
        self.rb.par.read().pa().bits().into()
    }

    /// Returns the Memory-0 Address
    pub fn m0a(&self) -> M0a {
        self.rb.m0ar.read().m0a().bits().into()
    }

    /// Returns the Memory-1 Address
    ///
    /// **Note** (for disabled and enabled streams):
    /// This config value has no meaning, if `buffer_mode` is `Regular`.
    ///
    /// This invariant is covered in `effective_m1a` (defined for disabled and enabled streams).
    pub fn m1a(&self) -> M1a {
        self.rb.m1ar.read().m1a().bits().into()
    }

    /// Returns the effective Memory-1 Address
    ///
    /// Read the documentation of `m1a` for more details.
    pub fn effective_m1a(&self) -> Option<M1a> {
        if self.buffer_mode() == BufferMode::Regular {
            None
        } else {
            Some(self.m1a())
        }
    }

    /// Returns the Fifo Threshold
    ///
    /// **Note** (for disabled and enabled streams):
    /// This config value has no meaning, if `transfer_mode` is `Direct`.
    ///
    /// This invariant is covered in `effective_fifo_threshold` (defined for disabled and enabled streams).
    pub fn fifo_threshold(&self) -> FifoThreshold {
        self.rb.fcr.read().fth().bits().try_into().unwrap()
    }

    /// Returns the effective Fifo Threshold.
    ///
    /// If the Fifo Threshold has no meaning, `None` is being returned.
    ///
    /// Read the documentation of `fifo_threshold` for more details.
    pub fn effective_fifo_threshold(&self) -> Option<FifoThreshold> {
        if self.transfer_mode() == TransferMode::Direct {
            None
        } else {
            Some(self.fifo_threshold())
        }
    }

    /// Returns the Transfer Mode (`Direct` or `Fifo` Mode)
    pub fn transfer_mode(&self) -> TransferMode {
        self.rb.fcr.read().dmdis().bit().into()
    }

    /// Performs the volatile write to the `M0a` register
    fn impl_set_m0a(&mut self, m0a: M0a) {
        unsafe {
            self.rb.m0ar.modify(|_, w| w.m0a().bits(m0a.into()));
        }
    }

    /// Performs the volatile write to the `M1a` register
    fn impl_set_m1a(&mut self, m1a: M1a) {
        unsafe {
            self.rb.m0ar.modify(|_, w| w.m0a().bits(m1a.into()));
        }
    }

    /// Transmutes the state of `self`
    fn transmute<NewED, NewIsrState>(
        self,
    ) -> Stream<CXX, DMA, NewED, NewIsrState>
    where
        NewED: EDTrait,
        NewIsrState: IsrStateTrait,
    {
        Stream {
            rb: self.rb,
            config_ndt: self.config_ndt,
            _phantom_data: PhantomData,
        }
    }
}

impl<CXX, DMA, IsrState> Stream<CXX, DMA, Disabled, IsrState>
where
    CXX: ChannelId<DMA = DMA>,
    DMA: DMATrait,
    IsrState: IsrStateTrait,
{
    /// Sets the Flow Controller
    pub fn set_flow_controller(&mut self, flow_controller: FlowController) {
        self.rb
            .cr
            .modify(|_, w| w.pfctrl().bit(flow_controller.into()));
    }

    /// Sets the Transfer Direction
    pub fn set_transfer_direction(&mut self, transfer_dir: TransferDirection) {
        unsafe {
            self.rb.cr.modify(|_, w| w.dir().bits(transfer_dir.into()));
        }
    }

    /// Sets the Circular Mode
    pub fn set_circular_mode(&mut self, circ_mode: CircularMode) {
        self.rb.cr.modify(|_, w| w.circ().bit(circ_mode.into()));
    }

    /// Sets the Peripheral Increment config flag
    pub fn set_pinc(&mut self, pinc: Pinc) {
        self.rb.cr.modify(|_, w| w.pinc().bit(pinc.into()));
    }

    /// Sets the Memory Increment config flag
    pub fn set_minc(&mut self, minc: Minc) {
        self.rb.cr.modify(|_, w| w.minc().bit(minc.into()));
    }

    /// Sets the Peripheral Size
    pub fn set_p_size(&mut self, p_size: PSize) {
        unsafe {
            self.rb.cr.modify(|_, w| w.psize().bits(p_size.into()));
        }
    }

    /// Sets the Memory Size
    pub fn set_m_size(&mut self, m_size: MSize) {
        unsafe {
            self.rb.cr.modify(|_, w| w.msize().bits(m_size.into()));
        }
    }

    /// Sets the Peripheral Increment Offset
    pub fn set_pincos(&mut self, pincos: Pincos) {
        self.rb.cr.modify(|_, w| w.pincos().bit(pincos.into()));
    }

    /// Sets the Priority Level
    pub fn set_priority_level(&mut self, priority_level: PriorityLevel) {
        unsafe {
            self.rb.cr.modify(|_, w| w.pl().bits(priority_level.into()));
        }
    }

    /// Sets the Buffer Mode (`Direct` or `Fifo` mode)
    pub fn set_buffer_mode(&mut self, buffer_mode: BufferMode) {
        self.rb.cr.modify(|_, w| w.dbm().bit(buffer_mode.into()));
    }

    /// Sets the Current Target
    pub fn set_current_target(&mut self, current_target: CurrentTarget) {
        self.rb.cr.modify(|_, w| w.ct().bit(current_target.into()));
    }

    /// Sets the Peripheral Burst
    pub fn set_p_burst(&mut self, p_burst: PBurst) {
        unsafe {
            self.rb.cr.modify(|_, w| w.pburst().bits(p_burst.into()));
        }
    }

    /// Sets the Memory Burst
    pub fn set_m_burst(&mut self, m_burst: MBurst) {
        unsafe {
            self.rb.cr.modify(|_, w| w.mburst().bits(m_burst.into()));
        }
    }

    /// Sets the NDT register
    pub fn set_ndt(&mut self, ndt: Ndt) {
        self.config_ndt = ndt;

        unsafe {
            self.rb.ndtr.modify(|_, w| w.ndt().bits(ndt.into()));
        }
    }

    /// Sets the Peripheral Address
    pub fn set_pa(&mut self, pa: Pa) {
        unsafe {
            self.rb.par.modify(|_, w| w.pa().bits(pa.into()));
        }
    }

    /// Sets the Memory-0 Address
    pub fn set_m0a(&mut self, m0a: M0a) {
        self.impl_set_m0a(m0a);
    }

    /// Sets the Memory-1 Address
    pub fn set_m1a(&mut self, m1a: M1a) {
        self.impl_set_m1a(m1a);
    }

    /// Sets the Fifo Threshold
    pub fn set_fifo_threshold(&mut self, fifo_threshold: FifoThreshold) {
        unsafe {
            self.rb
                .fcr
                .modify(|_, w| w.fth().bits(fifo_threshold.into()));
        }
    }

    /// Sets the Transfer Mode
    pub fn set_transfer_mode(&mut self, transfer_mode: TransferMode) {
        self.rb
            .fcr
            .modify(|_, w| w.dmdis().bit(transfer_mode.into()));
    }

    /// Returns the effective Flow Controller
    ///
    /// Read the documentation of `flow_controller` for more details.
    pub fn effective_flow_controller(&self) -> FlowController {
        if self.transfer_direction() == TransferDirection::M2M {
            FlowController::Dma
        } else {
            self.flow_controller()
        }
    }

    /// Returns the effective Circular Mode
    ///
    /// Read the documentation of `circular_mode` for more details.
    pub fn effective_circular_mode(&self) -> CircularMode {
        if self.buffer_mode() == BufferMode::DoubleBuffer {
            CircularMode::Enabled
        } else {
            self.circular_mode()
        }
    }

    /// Returns the effective Memory Size
    ///
    /// Read the documentation of `m_size` for more details.
    pub fn effective_m_size(&self) -> MSize {
        if self.transfer_mode() == TransferMode::Direct {
            match self.p_size() {
                PSize::Byte => MSize::Byte,
                PSize::HalfWord => MSize::HalfWord,
                PSize::Word => MSize::Word,
            }
        } else {
            self.m_size()
        }
    }

    /// Returns the effective Peripheral Increment Offset
    ///
    /// Read the documentation of `pincos` for more details.
    pub fn effective_pincos(&self) -> Option<Pincos> {
        if self.pinc() == Pinc::Fixed {
            return None;
        }

        if self.transfer_mode() == TransferMode::Direct
            || self.p_burst() != PBurst::Single
        {
            Some(Pincos::PSize)
        } else {
            Some(self.pincos())
        }
    }

    /// Returns the effective Peripheral Burst
    ///
    /// Read the documentation of `p_burst` for more details.
    pub fn effective_p_burst(&self) -> PBurst {
        if self.transfer_mode() == TransferMode::Direct {
            PBurst::Single
        } else {
            self.p_burst()
        }
    }

    /// Returns the effective Memory Burst
    ///
    /// Read the documentation of `m_burst` for more details.
    pub fn effective_m_burst(&self) -> MBurst {
        if self.transfer_mode() == TransferMode::Direct {
            MBurst::Single
        } else {
            self.m_burst()
        }
    }
}

impl<CXX, DMA, ED, IsrState> Stream<CXX, DMA, ED, IsrState>
where
    CXX: ChannelId<DMA = DMA>,
    DMA: DMATrait,
    ED: NotDisabled,
    IsrState: IsrStateTrait,
{
    /// Sets the Memory-0 Address on the fly
    ///
    /// # Panic
    ///
    /// This panics if the stream is not in Double Buffer Mode.
    pub fn set_m0a(&mut self, m0a: M0a) -> nb::Result<(), Infallible> {
        self.check_double_buffer();

        if self.current_target() == CurrentTarget::M0a && self.is_enabled() {
            return Err(NbError::WouldBlock);
        }

        self.impl_set_m0a(m0a);

        Ok(())
    }

    /// Sets the Memory-1 Address on the fly
    ///
    /// # Panic
    ///
    /// This panics if the stream is not in Double Buffer Mode.
    pub fn set_m1a(&mut self, m1a: M1a) -> nb::Result<(), Infallible> {
        self.check_double_buffer();

        if self.current_target() == CurrentTarget::M1a && self.is_enabled() {
            return Err(NbError::WouldBlock);
        }

        self.impl_set_m1a(m1a);

        Ok(())
    }

    /// Checks if the stream is in Double Buffer Mode
    fn check_double_buffer(&self) {
        if self.buffer_mode() == BufferMode::Regular {
            panic!("The buffer must be in double buffer mode to be changed on the fly.");
        }
    }
}

impl<CXX, DMA> Stream<CXX, DMA, Disabled, IsrCleared>
where
    CXX: ChannelId<DMA = DMA>,
    DMA: DMATrait,
{
    /// Checks the config for data integrity and enables the stream
    ///
    /// # Safety
    ///
    /// Aliasing rules aren't enforced.
    pub unsafe fn enable(self) -> Stream<CXX, DMA, Enabled, IsrUncleared> {
        self.check_config();

        self.enable_unchecked()
    }

    /// Enables the stream without checking the config
    ///
    /// Consider using the checked version instead (`enable`).
    ///
    /// # Safety
    ///
    /// - Aliasing rules aren't enforced
    /// - Config is not checked for guaranteeing data integrity
    pub unsafe fn enable_unchecked(
        self,
    ) -> Stream<CXX, DMA, Enabled, IsrUncleared> {
        self.rb.cr.modify(|_, w| w.en().set_bit());

        self.transmute()
    }

    /// Checks the config for data integrity
    fn check_config(&self) {
        if self.effective_circular_mode() == CircularMode::Enabled {
            self.check_config_circular();
        }

        if self.transfer_mode() == TransferMode::Fifo {
            self.check_config_fifo();
        }

        self.check_ndt();
    }

    /// Checks the circular config.
    //
    // Reference: RM0433 Rev 6 - Chapter 15.3.10
    fn check_config_circular(&self) {
        // Check for clashing config values
        if self.transfer_direction() == TransferDirection::M2M
            || self.flow_controller() == FlowController::Peripheral
        {
            panic!("For circular streams, the transfer direction must not be `M2M` and the FlowController must not be `Peripheral`.");
        }

        // Check invariants
        if self.transfer_mode() == TransferMode::Fifo {
            let ndt = self.ndt().value() as usize;
            let m_burst = self.m_burst().into_num();
            let p_burst = self.p_burst().into_num();
            let m_size = self.m_size().into_num();
            let p_size = self.p_size().into_num();

            if self.m_burst() != MBurst::Single
                && ndt % (m_burst * m_size / p_size) != 0
            {
                panic!(
                    "Data integrity not guaranteed, because \
                    `num_data_items != Multiple of (m_burst * (m_size / p_size))`"
                );
            }

            if ndt % (p_burst * p_size) != 0 {
                panic!(
                    "Data integrity not guaranteed, because \
                     `num_data_items != Multiple of (p_burst * p_size)`"
                );
            }
        } else {
            let ndt = self.ndt().value() as usize;
            let p_size = self.p_size().into_num();

            if ndt % p_size != 0 {
                panic!(
                    "Data integrity not guaranteed, because \
                     `num_data_items != Multiple of (p_size)`"
                );
            }
        }
    }

    /// Checks the fifo config.
    fn check_config_fifo(&self) {
        if self.m_burst() != MBurst::Single {
            self.check_config_fifo_m_burst();
        }

        if self.p_burst() != PBurst::Single {
            self.check_config_fifo_p_burst();
        }
    }

    /// Checks the memory config of fifo stream.
    //
    // Reference: RM0433 Rev 6 - Chapter 15.3.14
    fn check_config_fifo_m_burst(&self) {
        let m_size = self.m_size().into_num();
        let m_burst = self.m_burst().into_num();
        // Fifo Size in bytes
        let fifo_size = self.fifo_threshold().into_num() * 4;

        if m_size * m_burst > fifo_size {
            panic!("FIFO configuration invalid, because `msize * mburst > fifo_size`");
        }

        if fifo_size % (m_size * m_burst) != 0 {
            panic!("FIFO configuration invalid, because `fifo_size % (msize * mburst) != 0`");
        }
    }

    /// Checks the peripheral config of fifio stream.
    //
    // Reference: RM0433 Rev 6 - Chapter 15.3.14
    fn check_config_fifo_p_burst(&self) {
        let p_burst = self.p_burst().into_num();
        let p_size = self.p_size().into_num();
        // 4 Words = 16 Bytes
        const FULL_FIFO_BYTES: usize = 16;

        if p_burst * p_size == FULL_FIFO_BYTES
            && self.fifo_threshold() == FifoThreshold::F3_4
        {
            panic!(
                "FIFO configuration invalid, because \
                 `pburst * psize == FULL_FIFO_SIZE` and \
                 `fifo_threshhold == 3/4`"
            );
        }
    }

    /// Checks the NDT register
    //
    // Reference: RM0433 Rev 6 - Chapter 15.3.12
    fn check_ndt(&self) {
        let m_size = self.m_size().into_num();
        let p_size = self.p_size().into_num();
        let ndt = self.config_ndt.value() as usize;

        if m_size > p_size && ndt % (m_size / p_size) != 0 {
            panic!("`NDT` must be a multiple of (`m_size / p_size`).");
        }
    }
}

impl<CXX, DMA, IsrState> Stream<CXX, DMA, Enabled, IsrState>
where
    CXX: ChannelId<DMA = DMA>,
    DMA: DMATrait,
    IsrState: IsrStateTrait,
{
    /// Disables the stream
    pub fn disable(self) -> Stream<CXX, DMA, Disabling, IsrState> {
        self.rb.cr.modify(|_, w| w.en().clear_bit());

        self.transmute()
    }

    /// Returns the effective Peripheral Increment Offset
    ///
    /// Read the documentation of `pincos` for more details.
    pub fn effective_pincos(&self) -> Option<Pincos> {
        if self.pinc() == Pinc::Fixed {
            None
        } else {
            Some(self.pincos())
        }
    }
}

impl<CXX, DMA, IsrState> Stream<CXX, DMA, Disabling, IsrState>
where
    CXX: ChannelId<DMA = DMA>,
    DMA: DMATrait,
    IsrState: IsrStateTrait,
{
    /// Block current thread until stream has been disabled.
    pub fn await_disabled(self) -> Stream<CXX, DMA, Disabled, IsrState> {
        while self.rb.cr.read().en().bit_is_set() {}

        self.transmute()
    }
}

impl<CXX, DMA, ED> Stream<CXX, DMA, ED, IsrUncleared>
where
    CXX: ChannelId<DMA = DMA>,
    DMA: DMATrait,
    ED: EDTrait,
{
    /// Returns the contents of the isr.
    ///
    /// * If there is no error, an optional event is returned as `Ok`
    /// * If there are errors, the errors are being wrapped into `Error` and returned as `Err`
    pub fn check_isr(
        &self,
        isr: &StreamIsr<DMA>,
    ) -> Result<Option<Event>, Error> {
        let transfer_error = self.transfer_error_flag(isr);
        let direct_mode_error = self.direct_mode_error_flag(isr);
        let fifo_error = self.fifo_error_flag(isr);

        let event = if self.transfer_complete_flag(isr) {
            Some(Event::TransferComplete)
        } else if self.half_transfer_flag(isr) {
            Some(Event::HalfTransfer)
        } else {
            None
        };

        let crashed = !self.is_enabled() && self.ndt().value() != 0;

        if transfer_error || direct_mode_error || fifo_error {
            Err(Error {
                transfer_error,
                direct_mode_error,
                fifo_error,
                event,
                crashed,
            })
        } else {
            Ok(event)
        }
    }

    /// Returns the Transfer Complete flag
    pub fn transfer_complete_flag(&self, isr: &StreamIsr<DMA>) -> bool {
        match self.id() {
            0 => isr.lisr.read().tcif0().bit_is_set(),
            1 => isr.lisr.read().tcif1().bit_is_set(),
            2 => isr.lisr.read().tcif2().bit_is_set(),
            3 => isr.lisr.read().tcif3().bit_is_set(),
            4 => isr.hisr.read().tcif4().bit_is_set(),
            5 => isr.hisr.read().tcif5().bit_is_set(),
            6 => isr.hisr.read().tcif6().bit_is_set(),
            7 => isr.hisr.read().tcif7().bit_is_set(),
            _ => unreachable!(),
        }
    }

    /// Returns the Half Transfer flag
    pub fn half_transfer_flag(&self, isr: &StreamIsr<DMA>) -> bool {
        match self.id() {
            0 => isr.lisr.read().htif0().bit_is_set(),
            1 => isr.lisr.read().htif1().bit_is_set(),
            2 => isr.lisr.read().htif2().bit_is_set(),
            3 => isr.lisr.read().htif3().bit_is_set(),
            4 => isr.hisr.read().htif4().bit_is_set(),
            5 => isr.hisr.read().htif5().bit_is_set(),
            6 => isr.hisr.read().htif6().bit_is_set(),
            7 => isr.hisr.read().htif7().bit_is_set(),
            _ => unreachable!(),
        }
    }

    /// Returns the Transfer Error flag
    pub fn transfer_error_flag(&self, isr: &StreamIsr<DMA>) -> bool {
        match self.id() {
            0 => isr.lisr.read().teif0().bit_is_set(),
            1 => isr.lisr.read().teif1().bit_is_set(),
            2 => isr.lisr.read().teif2().bit_is_set(),
            3 => isr.lisr.read().teif3().bit_is_set(),
            4 => isr.hisr.read().teif4().bit_is_set(),
            5 => isr.hisr.read().teif5().bit_is_set(),
            6 => isr.hisr.read().teif6().bit_is_set(),
            7 => isr.hisr.read().teif7().bit_is_set(),
            _ => unreachable!(),
        }
    }

    /// Returns the Direct Mode Error flag
    pub fn direct_mode_error_flag(&self, isr: &StreamIsr<DMA>) -> bool {
        match self.id() {
            0 => isr.lisr.read().dmeif0().bit_is_set(),
            1 => isr.lisr.read().dmeif1().bit_is_set(),
            2 => isr.lisr.read().dmeif2().bit_is_set(),
            3 => isr.lisr.read().dmeif3().bit_is_set(),
            4 => isr.hisr.read().dmeif4().bit_is_set(),
            5 => isr.hisr.read().dmeif5().bit_is_set(),
            6 => isr.hisr.read().dmeif6().bit_is_set(),
            7 => isr.hisr.read().dmeif7().bit_is_set(),
            _ => unreachable!(),
        }
    }

    /// Returns the Fifo Error flag
    pub fn fifo_error_flag(&self, isr: &StreamIsr<DMA>) -> bool {
        match self.id() {
            0 => isr.lisr.read().feif0().bit_is_set(),
            1 => isr.lisr.read().feif1().bit_is_set(),
            2 => isr.lisr.read().feif2().bit_is_set(),
            3 => isr.lisr.read().feif3().bit_is_set(),
            4 => isr.hisr.read().feif4().bit_is_set(),
            5 => isr.hisr.read().feif5().bit_is_set(),
            6 => isr.hisr.read().feif6().bit_is_set(),
            7 => isr.hisr.read().feif7().bit_is_set(),
            _ => unreachable!(),
        }
    }

    /// Performs the ISR clear
    fn clear_isr_impl(&self, isr: &mut StreamIsr<DMA>) {
        self.clear_transfer_complete(isr);
        self.clear_half_transfer(isr);
        self.clear_transfer_error(isr);
        self.clear_direct_mode_error(isr);
        self.clear_fifo_error(isr);
    }

    /// Clears the Transfer Complete flag
    pub fn clear_transfer_complete(&self, isr: &mut StreamIsr<DMA>) {
        match self.id() {
            0 => {
                isr.lifcr.write(|w| w.ctcif0().set_bit());
            }
            1 => {
                isr.lifcr.write(|w| w.ctcif1().set_bit());
            }
            2 => {
                isr.lifcr.write(|w| w.ctcif2().set_bit());
            }
            3 => {
                isr.lifcr.write(|w| w.ctcif3().set_bit());
            }
            4 => {
                isr.hifcr.write(|w| w.ctcif4().set_bit());
            }
            5 => {
                isr.hifcr.write(|w| w.ctcif5().set_bit());
            }
            6 => {
                isr.hifcr.write(|w| w.ctcif6().set_bit());
            }
            7 => {
                isr.hifcr.write(|w| w.ctcif7().set_bit());
            }
            _ => unreachable!(),
        }
    }

    /// Clears the Half Transfer flag
    pub fn clear_half_transfer(&self, isr: &mut StreamIsr<DMA>) {
        match self.id() {
            0 => {
                isr.lifcr.write(|w| w.chtif0().set_bit());
            }
            1 => {
                isr.lifcr.write(|w| w.chtif1().set_bit());
            }
            2 => {
                isr.lifcr.write(|w| w.chtif2().set_bit());
            }
            3 => {
                isr.lifcr.write(|w| w.chtif3().set_bit());
            }
            4 => {
                isr.hifcr.write(|w| w.chtif4().set_bit());
            }
            5 => {
                isr.hifcr.write(|w| w.chtif5().set_bit());
            }
            6 => {
                isr.hifcr.write(|w| w.chtif6().set_bit());
            }
            7 => {
                isr.hifcr.write(|w| w.chtif7().set_bit());
            }
            _ => unreachable!(),
        }
    }

    /// Clears the Transfer Error flag
    pub fn clear_transfer_error(&self, isr: &mut StreamIsr<DMA>) {
        match self.id() {
            0 => {
                isr.lifcr.write(|w| w.cteif0().set_bit());
            }
            1 => {
                isr.lifcr.write(|w| w.cteif1().set_bit());
            }
            2 => {
                isr.lifcr.write(|w| w.cteif2().set_bit());
            }
            3 => {
                isr.lifcr.write(|w| w.cteif3().set_bit());
            }
            4 => {
                isr.hifcr.write(|w| w.cteif4().set_bit());
            }
            5 => {
                isr.hifcr.write(|w| w.cteif5().set_bit());
            }
            6 => {
                isr.hifcr.write(|w| w.cteif6().set_bit());
            }
            7 => {
                isr.hifcr.write(|w| w.cteif7().set_bit());
            }
            _ => unreachable!(),
        }
    }

    /// Clears the Direct Mode Error flag
    pub fn clear_direct_mode_error(&self, isr: &mut StreamIsr<DMA>) {
        match self.id() {
            0 => {
                isr.lifcr.write(|w| w.cdmeif0().set_bit());
            }
            1 => {
                isr.lifcr.write(|w| w.cdmeif1().set_bit());
            }
            2 => {
                isr.lifcr.write(|w| w.cdmeif2().set_bit());
            }
            3 => {
                isr.lifcr.write(|w| w.cdmeif3().set_bit());
            }
            4 => {
                isr.hifcr.write(|w| w.cdmeif4().set_bit());
            }
            5 => {
                isr.hifcr.write(|w| w.cdmeif5().set_bit());
            }
            6 => {
                isr.hifcr.write(|w| w.cdmeif6().set_bit());
            }
            7 => {
                isr.hifcr.write(|w| w.cdmeif7().set_bit());
            }
            _ => unreachable!(),
        }
    }

    /// Clears the Fifo Error flag
    pub fn clear_fifo_error(&self, isr: &mut StreamIsr<DMA>) {
        match self.id() {
            0 => {
                isr.lifcr.write(|w| w.cfeif0().set_bit());
            }
            1 => {
                isr.lifcr.write(|w| w.cfeif1().set_bit());
            }
            2 => {
                isr.lifcr.write(|w| w.cfeif2().set_bit());
            }
            3 => {
                isr.lifcr.write(|w| w.cfeif3().set_bit());
            }
            4 => {
                isr.hifcr.write(|w| w.cfeif4().set_bit());
            }
            5 => {
                isr.hifcr.write(|w| w.cfeif5().set_bit());
            }
            6 => {
                isr.hifcr.write(|w| w.cfeif6().set_bit());
            }
            7 => {
                isr.hifcr.write(|w| w.cfeif7().set_bit());
            }
            _ => unreachable!(),
        }
    }
}

impl<CXX, DMA> Stream<CXX, DMA, Disabled, IsrUncleared>
where
    CXX: ChannelId<DMA = DMA>,
    DMA: DMATrait,
{
    /// Clears the ISR
    pub fn clear_isr(
        self,
        isr: &mut StreamIsr<DMA>,
    ) -> Stream<CXX, DMA, Disabled, IsrCleared> {
        self.clear_isr_impl(isr);

        self.transmute()
    }
}

impl<CXX, DMA, ED> Stream<CXX, DMA, ED, IsrUncleared>
where
    CXX: ChannelId<DMA = DMA>,
    DMA: DMATrait,
    ED: NotDisabled,
{
    /// Clears the ISR
    pub fn clear_isr(&self, isr: &mut StreamIsr<DMA>) {
        self.clear_isr_impl(isr);
    }
}

unsafe impl<CXX, DMA, ED, IsrState> Send for Stream<CXX, DMA, ED, IsrState>
where
    CXX: ChannelId<DMA = DMA>,
    DMA: DMATrait,
    ED: EDTrait,
    IsrState: IsrStateTrait,
{
}

unsafe impl<CXX, DMA, ED, IsrState> Sync for Stream<CXX, DMA, ED, IsrState>
where
    CXX: ChannelId<DMA = DMA>,
    DMA: DMATrait,
    ED: EDTrait,
    IsrState: IsrStateTrait,
{
}

/// DMA Mux
pub struct DmaMux<CXX, ReqId, SyncED, EgED>
where
    CXX: ChannelId,
    ReqId: RequestIdTrait,
    SyncED: SyncEDTrait,
    EgED: EgEDTrait,
{
    /// This field *must not* be mutated using shared references
    rb: &'static CCR,
    req_id: ReqId,
    _phantom_data: PhantomData<(CXX, SyncED, EgED)>,
}

impl<CXX> DmaMux<CXX, ReqNone, SyncDisabled, EgDisabled>
where
    CXX: ChannelId,
{
    /// Creates an instance of a DMA Mux in initial state.
    ///
    /// Should only be called after RCC-reset of the DMA.
    fn after_reset(rb: &'static CCR) -> Self {
        DmaMux {
            rb,
            req_id: ReqNone,
            _phantom_data: PhantomData,
        }
    }
}

impl<CXX, ReqId, SyncED, EgED> DmaMux<CXX, ReqId, SyncED, EgED>
where
    CXX: ChannelId,
    ReqId: RequestIdTrait,
    SyncED: SyncEDTrait,
    EgED: EgEDTrait,
{
    /// Returns the id of the DMA Mux
    pub fn id(&self) -> usize {
        CXX::MUX_ID
    }

    /// Returns the request id assigned to this Mux
    pub fn request_id(&self) -> RequestId {
        debug_assert_eq!(
            ReqId::REQUEST_ID,
            RequestId::try_from(self.rb.read().dmareq_id().bits()).unwrap(),
            "DmaMux is in invalid state, because \
            `ReqId::REQUEST_ID` ({:?}) != Volatile request id ({:?})",
            ReqId::REQUEST_ID,
            RequestId::try_from(self.rb.read().dmareq_id().bits()).unwrap(),
        );

        ReqId::REQUEST_ID
    }

    /// Returns the Sync Overrun Interrupt config flag
    pub fn sync_overrun_interrupt(&self) -> SyncOverrunInterrupt {
        self.rb.read().soie().bit().into()
    }

    /// Sets the Sync Overrun Interrupt config flag
    pub fn set_sync_overrun_interrupt(
        &mut self,
        sync_overrun_intrpt: SyncOverrunInterrupt,
    ) {
        self.rb
            .modify(|_, w| w.soie().bit(sync_overrun_intrpt.into()));
    }

    /// Returns the Synchronization Polarity
    pub fn sync_polarity(&self) -> SyncPolarity {
        self.rb.read().spol().bits().try_into().unwrap()
    }

    /// Sets the Synchronization Polarity
    pub fn set_sync_polarity(&mut self, sync_polarity: SyncPolarity) {
        self.rb.modify(|_, w| w.spol().bits(sync_polarity.into()));
    }

    /// Returns the number of requests
    pub fn nbreq(&self) -> NbReq {
        self.rb.read().nbreq().bits().try_into().unwrap()
    }

    /// Returns the Synchronization ID
    pub fn sync_id(&self) -> SyncId {
        self.rb.read().sync_id().bits().try_into().unwrap()
    }

    /// Sets the Synchronization ID
    pub fn set_sync_id(&mut self, sync_id: SyncId) {
        unsafe {
            self.rb.modify(|_, w| w.sync_id().bits(sync_id.into()));
        }
    }

    /// Performs the request id write
    fn set_req_id_impl(&mut self, request_id: RequestId) {
        unsafe {
            self.rb.modify(|_, w| w.dmareq_id().bits(request_id.into()));
        }
    }

    /// Transmutes the state of the DMA Mux
    fn transmute<NewSyncED, NewEgED>(
        self,
    ) -> DmaMux<CXX, ReqId, NewSyncED, NewEgED>
    where
        NewSyncED: SyncEDTrait,
        NewEgED: EgEDTrait,
    {
        DmaMux {
            rb: self.rb,
            req_id: self.req_id,
            _phantom_data: PhantomData,
        }
    }
}

impl<CXX, ReqId> DmaMux<CXX, ReqId, SyncDisabled, EgDisabled>
where
    CXX: ChannelId,
    ReqId: RequestIdTrait,
{
    /// Sets the number of requests
    pub fn set_nbreq(&mut self, nbreq: NbReq) {
        self.rb.modify(|_, w| w.nbreq().bits(nbreq.into()));
    }
}

impl<CXX, ReqId, EgED> DmaMux<CXX, ReqId, SyncDisabled, EgED>
where
    CXX: ChannelId,
    ReqId: RequestIdTrait,
    EgED: EgEDTrait,
{
    /// Enables synchronization
    pub fn enable_sync(self) -> DmaMux<CXX, ReqId, SyncEnabled, EgED> {
        self.rb.modify(|_, w| w.se().set_bit());

        self.transmute()
    }
}

impl<CXX, ReqId, EgED> DmaMux<CXX, ReqId, SyncEnabled, EgED>
where
    CXX: ChannelId,
    ReqId: RequestIdTrait,
    EgED: EgEDTrait,
{
    /// Disables synchronization
    pub fn disable_sync(self) -> DmaMux<CXX, ReqId, SyncDisabled, EgED> {
        self.rb.modify(|_, w| w.se().clear_bit());

        self.transmute()
    }
}

impl<CXX, ReqId, SyncED> DmaMux<CXX, ReqId, SyncED, EgDisabled>
where
    CXX: ChannelId,
    ReqId: RequestIdTrait,
    SyncED: SyncEDTrait,
{
    /// Enables event generation
    pub fn enable_event_gen(self) -> DmaMux<CXX, ReqId, SyncED, EgEnabled> {
        self.rb.modify(|_, w| w.ege().set_bit());

        self.transmute()
    }
}

impl<CXX, ReqId, SyncED> DmaMux<CXX, ReqId, SyncED, EgEnabled>
where
    CXX: ChannelId,
    ReqId: RequestIdTrait,
    SyncED: SyncEDTrait,
{
    /// Disables event generation
    pub fn disable_event_gen(self) -> DmaMux<CXX, ReqId, SyncED, EgDisabled> {
        self.rb.modify(|_, w| w.ege().clear_bit());

        self.transmute()
    }
}

impl<CXX, SyncED, EgED> DmaMux<CXX, ReqNone, SyncED, EgED>
where
    CXX: ChannelId,
    SyncED: SyncEDTrait,
    EgED: EgEDTrait,
{
    /// Sets request id
    pub fn set_req_id<NewReqId>(
        mut self,
        req_id: NewReqId,
    ) -> DmaMux<CXX, NewReqId, SyncED, EgED>
    where
        NewReqId: RequestIdSome,
    {
        self.set_req_id_impl(NewReqId::REQUEST_ID);

        DmaMux {
            req_id,
            rb: self.rb,
            _phantom_data: PhantomData,
        }
    }
}

impl<CXX, ReqId, SyncED, EgED> DmaMux<CXX, ReqId, SyncED, EgED>
where
    CXX: ChannelId,
    ReqId: RequestIdSome,
    SyncED: SyncEDTrait,
    EgED: EgEDTrait,
{
    /// Unsets request id, defaulting to `ReqNone` and returning the old one
    pub fn unset_req_id(
        mut self,
    ) -> (DmaMux<CXX, ReqNone, SyncED, EgED>, ReqId) {
        self.set_req_id_impl(ReqNone::REQUEST_ID);

        let old_req_id = self.req_id;
        let new_dma_mux = DmaMux {
            rb: self.rb,
            req_id: ReqNone,
            _phantom_data: PhantomData,
        };

        (new_dma_mux, old_req_id)
    }

    /// Replaces the request id
    pub fn replace_req_id<NewReqId>(
        mut self,
        req_id: NewReqId,
    ) -> (DmaMux<CXX, NewReqId, SyncED, EgED>, ReqId)
    where
        NewReqId: RequestIdSome,
    {
        self.set_req_id_impl(NewReqId::REQUEST_ID);

        let old_req_id = self.req_id;
        let new_dma_mux = DmaMux {
            req_id,
            rb: self.rb,
            _phantom_data: PhantomData,
        };

        (new_dma_mux, old_req_id)
    }
}

impl<CXX, ReqId, SyncED, EgED> DmaMux<CXX, ReqId, SyncED, EgED>
where
    CXX: ChannelId,
    ReqId: RequestIdTrait,
    SyncED: SyncEDTrait,
    EgED: EgEDTrait,
{
    /// Checks the ISR for errors
    pub fn check_isr(&self, mux_isr: &MuxIsr) -> Result<(), OverrunError> {
        if self.is_sync_overrun(mux_isr) {
            Err(OverrunError)
        } else {
            Ok(())
        }
    }

    /// Returns the Sync Overrun flag
    pub fn is_sync_overrun(&self, mux_isr: &MuxIsr) -> bool {
        let mask: u16 = 1 << self.id() as u16;

        mux_isr.csr.read().sof().bits() & mask != 0
    }

    /// Clears the ISR
    pub fn clear_isr(&self, mux_isr: &mut MuxIsr) {
        match self.id() {
            0 => mux_isr.cfr.write(|w| w.csof0().set_bit()),
            1 => mux_isr.cfr.write(|w| w.csof1().set_bit()),
            2 => mux_isr.cfr.write(|w| w.csof2().set_bit()),
            3 => mux_isr.cfr.write(|w| w.csof3().set_bit()),
            4 => mux_isr.cfr.write(|w| w.csof4().set_bit()),
            5 => mux_isr.cfr.write(|w| w.csof5().set_bit()),
            6 => mux_isr.cfr.write(|w| w.csof6().set_bit()),
            7 => mux_isr.cfr.write(|w| w.csof7().set_bit()),
            8 => mux_isr.cfr.write(|w| w.csof8().set_bit()),
            9 => mux_isr.cfr.write(|w| w.csof9().set_bit()),
            10 => mux_isr.cfr.write(|w| w.csof10().set_bit()),
            11 => mux_isr.cfr.write(|w| w.csof11().set_bit()),
            12 => mux_isr.cfr.write(|w| w.csof12().set_bit()),
            13 => mux_isr.cfr.write(|w| w.csof13().set_bit()),
            14 => mux_isr.cfr.write(|w| w.csof14().set_bit()),
            15 => mux_isr.cfr.write(|w| w.csof15().set_bit()),
            _ => unreachable!(),
        }
    }
}

unsafe impl<CXX, ReqId, SyncED, EgED> Send for DmaMux<CXX, ReqId, SyncED, EgED>
where
    CXX: ChannelId,
    ReqId: RequestIdTrait,
    SyncED: SyncEDTrait,
    EgED: EgEDTrait,
{
}

unsafe impl<CXX, ReqId, SyncED, EgED> Sync for DmaMux<CXX, ReqId, SyncED, EgED>
where
    CXX: ChannelId,
    ReqId: RequestIdTrait,
    SyncED: SyncEDTrait,
    EgED: EgEDTrait,
{
}

/// Memory-safe DMA transfer for single-buffer
pub struct SafeTransfer<'wo, Peripheral, Memory, State>
where
    Peripheral: Payload,
    Memory: Payload,
    State: TransferState,
{
    peripheral: PeripheralBufferStatic<'wo, Peripheral>,
    memory: MemoryBufferStatic<Memory>,
    state: State,
}

impl<'wo, Peripheral, Memory> SafeTransfer<'wo, Peripheral, Memory, Start>
where
    Peripheral: Payload,
    Memory: Payload,
{
    /// Initializes new DMA transfer without starting it yet
    pub fn new(
        peripheral: PeripheralBufferStatic<'wo, Peripheral>,
        memory: MemoryBufferStatic<Memory>,
    ) -> Self {
        check_buffer(&peripheral, &memory);

        SafeTransfer {
            peripheral,
            memory,
            state: Start,
        }
    }
}

impl<'wo, Peripheral, Memory, State>
    SafeTransfer<'wo, Peripheral, Memory, State>
where
    Peripheral: Payload,
    Memory: Payload,
    State: TransferState,
{
    /// Returns peripheral buffer
    pub fn peripheral(&self) -> &PeripheralBufferStatic<'wo, Peripheral> {
        &self.peripheral
    }

    /// Returns memory buffer
    pub fn memory(&self) -> &MemoryBufferStatic<Memory> {
        &self.memory
    }

    /// Sets the content of destination buffer
    ///
    /// # Safety
    ///
    /// The caller must ensure, that the DMA is currently not modifying this address.
    pub unsafe fn set_dest(
        &mut self,
        index: Option<usize>,
        payload: PayloadPort<Peripheral, Memory>,
    ) {
        if self.peripheral.is_write() {
            set_peripheral_impl(
                &mut self.peripheral,
                index,
                payload.peripheral(),
            );
        } else {
            set_memory_impl(&mut self.memory, index, payload.memory());
        }
    }

    /// Returns pointer to destination buffer
    pub fn dest_ptr(
        &mut self,
        index: Option<usize>,
    ) -> PointerPort<Peripheral, Memory> {
        if self.peripheral.is_write() {
            let ptr = mut_ptr_peripheral(&mut self.peripheral, index);

            PointerPort::Peripheral(ptr)
        } else {
            let ptr = mut_ptr_memory(&mut self.memory, index);

            PointerPort::Memory(ptr)
        }
    }
}

impl<'wo, Source, Dest> SafeTransfer<'wo, Source, Dest, Start>
where
    Source: Payload,
    Dest: Payload,
{
    /// Starts the transfer
    pub fn start<CXX, DMA>(
        self,
        mut stream: Stream<CXX, DMA, Disabled, IsrCleared>,
    ) -> SafeTransfer<'wo, Source, Dest, Ongoing<CXX, DMA>>
    where
        CXX: ChannelId<DMA = DMA>,
        DMA: DMATrait,
    {
        configure_safe_transfer(&mut stream, &self.peripheral, &self.memory);
        stream.set_buffer_mode(BufferMode::Regular);

        SafeTransfer {
            peripheral: self.peripheral,
            memory: self.memory,
            state: Ongoing {
                stream: unsafe { stream.enable() },
            },
        }
    }
}

impl<Source, Dest, CXX, DMA> SafeTransfer<'_, Source, Dest, Ongoing<CXX, DMA>>
where
    Source: Payload,
    Dest: Payload,
    CXX: ChannelId<DMA = DMA>,
    DMA: DMATrait,
{
    /// Returns the stream assigned to the transfer
    pub fn stream(&self) -> &Stream<CXX, DMA, Enabled, IsrUncleared> {
        &self.state.stream
    }

    /// Sets the Transfer Complete Interrupt config flag of the assigned stream
    pub fn set_transfer_complete_interrupt(
        &mut self,
        tc_intrpt: TransferCompleteInterrupt,
    ) {
        self.state.stream.set_transfer_complete_interrupt(tc_intrpt);
    }

    /// Sets the Half Transfer Interrupt config flag of the assigned stream
    pub fn set_half_transfer_interrupt(
        &mut self,
        ht_intrpt: HalfTransferInterrupt,
    ) {
        self.state.stream.set_half_transfer_interrupt(ht_intrpt);
    }

    /// Sets the Transfer Error Interrupt config flag of the assigned stream
    pub fn set_transfer_error_interrupt(
        &mut self,
        te_intrpt: TransferErrorInterrupt,
    ) {
        self.state.stream.set_transfer_error_interrupt(te_intrpt);
    }

    /// Sets the Direct Mode Error Interrupt config flag of the assigned stream
    pub fn set_direct_mode_error_interrupt(
        &mut self,
        dme_intrpt: DirectModeErrorInterrupt,
    ) {
        self.state
            .stream
            .set_direct_mode_error_interrupt(dme_intrpt);
    }

    /// Sets the Fifo Error Interrupt config flag of the assigned stream
    pub fn set_fifo_error_interrupt(&mut self, fe_intrpt: FifoErrorInterrupt) {
        self.state.stream.set_fifo_error_interrupt(fe_intrpt);
    }

    /// Stops the transfer, returning the stream
    pub fn stop(self) -> Stream<CXX, DMA, Disabled, IsrUncleared> {
        self.state.stream.disable().await_disabled()
    }
}

/// Memory-safe DMA transfer for double buffer
pub struct SafeTransferDoubleBuffer<'wo, Peripheral, Memory, State>
where
    Peripheral: Payload,
    Memory: Payload,
    State: TransferState,
{
    peripheral: PeripheralBufferStatic<'wo, Peripheral>,
    memories: [MemoryBufferStatic<Memory>; 2],
    state: State,
}

impl<'wo, Peripheral, Memory>
    SafeTransferDoubleBuffer<'wo, Peripheral, Memory, Start>
where
    Peripheral: Payload,
    Memory: Payload,
{
    /// Initializes new DMA transfer without starting it yet
    pub fn new(
        peripheral: PeripheralBufferStatic<'wo, Peripheral>,
        memories: [MemoryBufferStatic<Memory>; 2],
    ) -> Self {
        check_buffer(&peripheral, &memories[0]);
        check_double_buffer(&memories);

        SafeTransferDoubleBuffer {
            peripheral,
            memories,
            state: Start,
        }
    }
}

impl<'wo, Peripheral, Memory, State>
    SafeTransferDoubleBuffer<'wo, Peripheral, Memory, State>
where
    Peripheral: Payload,
    Memory: Payload,
    State: TransferState,
{
    /// Returns peripheral buffer
    pub fn peripheral(&self) -> &PeripheralBufferStatic<'wo, Peripheral> {
        &self.peripheral
    }

    /// Returns memory buffers
    pub fn memories(&self) -> &[MemoryBufferStatic<Memory>; 2] {
        &self.memories
    }

    /// Sets the content of destination buffer
    pub unsafe fn set_dest(
        &mut self,
        index: Option<usize>,
        double_buffer: Option<DoubleBuffer>,
        payload: PayloadPort<Peripheral, Memory>,
    ) {
        if self.peripheral.is_write() {
            set_peripheral_impl(
                &mut self.peripheral,
                index,
                payload.peripheral(),
            );
        } else {
            set_memory_impl(
                &mut self.memories[double_buffer.unwrap().index()],
                index,
                payload.memory(),
            );
        }
    }

    /// Returns a pointer to the destination buffer
    pub fn dest_ptr(
        &mut self,
        index: Option<usize>,
        double_buffer: Option<DoubleBuffer>,
    ) -> PointerPort<Peripheral, Memory> {
        if self.peripheral.is_write() {
            let ptr = mut_ptr_peripheral(&mut self.peripheral, index);

            PointerPort::Peripheral(ptr)
        } else {
            let ptr = mut_ptr_memory(
                &mut self.memories[double_buffer.unwrap().index()],
                index,
            );

            PointerPort::Memory(ptr)
        }
    }
}

impl<'wo, Peripheral, Memory>
    SafeTransferDoubleBuffer<'wo, Peripheral, Memory, Start>
where
    Peripheral: Payload,
    Memory: Payload,
{
    /// Starts the transfer
    pub fn start<CXX, DMA>(
        self,
        mut stream: Stream<CXX, DMA, Disabled, IsrCleared>,
    ) -> SafeTransferDoubleBuffer<'wo, Peripheral, Memory, Ongoing<CXX, DMA>>
    where
        CXX: ChannelId<DMA = DMA>,
        DMA: DMATrait,
    {
        stream.set_buffer_mode(BufferMode::DoubleBuffer);
        stream.set_m1a(M1a(self.memories[1].as_ptr(Some(0)) as u32));

        configure_safe_transfer(
            &mut stream,
            &self.peripheral,
            &self.memories[0],
        );

        SafeTransferDoubleBuffer {
            peripheral: self.peripheral,
            memories: self.memories,
            state: Ongoing {
                stream: unsafe { stream.enable() },
            },
        }
    }
}

impl<Peripheral, Memory, CXX, DMA>
    SafeTransferDoubleBuffer<'_, Peripheral, Memory, Ongoing<CXX, DMA>>
where
    Peripheral: Payload,
    Memory: Payload,
    CXX: ChannelId<DMA = DMA>,
    DMA: DMATrait,
{
    /// Returns the stream assigned to the transfer
    pub fn stream(&self) -> &Stream<CXX, DMA, Enabled, IsrUncleared> {
        &self.state.stream
    }

    /// Sets the Transfer Complete Interrupt config flag
    pub fn set_transfer_complete_interrupt(
        &mut self,
        tc_intrpt: TransferCompleteInterrupt,
    ) {
        self.state.stream.set_transfer_complete_interrupt(tc_intrpt);
    }

    /// Sets the Half Transfer Interrupt config flag
    pub fn set_half_transfer_interrupt(
        &mut self,
        ht_intrpt: HalfTransferInterrupt,
    ) {
        self.state.stream.set_half_transfer_interrupt(ht_intrpt);
    }

    /// Sets the Transfer Error Interrupt config flag
    pub fn set_transfer_error_interrupt(
        &mut self,
        te_intrpt: TransferErrorInterrupt,
    ) {
        self.state.stream.set_transfer_error_interrupt(te_intrpt);
    }

    /// Sets the Direct Mode Error Interrupt config flag
    pub fn set_direct_mode_error_interrupt(
        &mut self,
        dme_intrpt: DirectModeErrorInterrupt,
    ) {
        self.state
            .stream
            .set_direct_mode_error_interrupt(dme_intrpt);
    }

    /// Sets the Fifo Error Interrupt config flag
    pub fn set_fifo_error_interrupt(&mut self, fe_intrpt: FifoErrorInterrupt) {
        self.state.stream.set_fifo_error_interrupt(fe_intrpt);
    }

    /// Sets the memory-0 address
    pub fn set_m0a(&mut self, m0a: MemoryBufferStatic<Memory>) {
        let ptr = m0a.as_ptr(Some(0));

        mem::replace(&mut self.memories[0], m0a);

        check_double_buffer(&self.memories);

        block!(self.state.stream.set_m0a(M0a(ptr as u32))).unwrap();
    }

    /// Sets the memory-1 address
    pub fn set_m1a(&mut self, m1a: MemoryBufferStatic<Memory>) {
        let ptr = m1a.as_ptr(Some(0));

        mem::replace(&mut self.memories[1], m1a);

        check_double_buffer(&self.memories);

        block!(self.state.stream.set_m1a(M1a(ptr as u32))).unwrap();
    }

    /// Stops the transfer
    pub fn stop(self) -> Stream<CXX, DMA, Disabled, IsrUncleared> {
        self.state.stream.disable().await_disabled()
    }
}

pub type ChannelsDma1 = (
    Channel<C0, DMA1, Disabled, IsrCleared, ReqNone, SyncDisabled, EgDisabled>,
    Channel<C1, DMA1, Disabled, IsrCleared, ReqNone, SyncDisabled, EgDisabled>,
    Channel<C2, DMA1, Disabled, IsrCleared, ReqNone, SyncDisabled, EgDisabled>,
    Channel<C3, DMA1, Disabled, IsrCleared, ReqNone, SyncDisabled, EgDisabled>,
    Channel<C4, DMA1, Disabled, IsrCleared, ReqNone, SyncDisabled, EgDisabled>,
    Channel<C5, DMA1, Disabled, IsrCleared, ReqNone, SyncDisabled, EgDisabled>,
    Channel<C6, DMA1, Disabled, IsrCleared, ReqNone, SyncDisabled, EgDisabled>,
    Channel<C7, DMA1, Disabled, IsrCleared, ReqNone, SyncDisabled, EgDisabled>,
);

pub type ChannelsDma2 = (
    Channel<C8, DMA2, Disabled, IsrCleared, ReqNone, SyncDisabled, EgDisabled>,
    Channel<C9, DMA2, Disabled, IsrCleared, ReqNone, SyncDisabled, EgDisabled>,
    Channel<C10, DMA2, Disabled, IsrCleared, ReqNone, SyncDisabled, EgDisabled>,
    Channel<C11, DMA2, Disabled, IsrCleared, ReqNone, SyncDisabled, EgDisabled>,
    Channel<C12, DMA2, Disabled, IsrCleared, ReqNone, SyncDisabled, EgDisabled>,
    Channel<C13, DMA2, Disabled, IsrCleared, ReqNone, SyncDisabled, EgDisabled>,
    Channel<C14, DMA2, Disabled, IsrCleared, ReqNone, SyncDisabled, EgDisabled>,
    Channel<C15, DMA2, Disabled, IsrCleared, ReqNone, SyncDisabled, EgDisabled>,
);

pub type RequestGenerators = (
    RequestGenerator<G0, GenDisabled>,
    RequestGenerator<G1, GenDisabled>,
    RequestGenerator<G2, GenDisabled>,
    RequestGenerator<G3, GenDisabled>,
    RequestGenerator<G4, GenDisabled>,
    RequestGenerator<G5, GenDisabled>,
    RequestGenerator<G6, GenDisabled>,
    RequestGenerator<G7, GenDisabled>,
);

/// Container for shared items across the dma
pub struct DmaShared {
    pub stream_isr_dma_1: StreamIsr<DMA1>,
    pub stream_isr_dma_2: StreamIsr<DMA2>,
    pub mux_shared: MuxShared,
}

/// Contains all channels, request generators and the shared items
pub struct Dma {
    /// Channels for DMA1
    pub channels_dma_1: ChannelsDma1,
    /// Channels for DMA2
    pub channels_dma_2: ChannelsDma2,
    /// Shared items for both DMAs
    pub dma_shared: DmaShared,
    /// All request generators
    pub request_generators: RequestGenerators,
    /// Do not access this field. This is stored in case the user want's the peripheral back.
    _dma_1: DMA1,
    /// Do not access this field. This is stored in case the user want's the peripheral back.
    _dma_2: DMA2,
    /// Do not access this field. This is stored in case the user want's the peripheral back.
    _dma_mux: DMAMUX1,
}

impl Dma {
    /// Initializes the DMA-HAL. This is the entrypoint of the HAL.
    pub fn new(
        dma_1: DMA1,
        dma_2: DMA2,
        mut dma_mux: DMAMUX1,
        ccdr: &mut Ccdr,
    ) -> Self {
        Dma::reset_dma(&mut ccdr.rb);
        Dma::enable_dma(&mut ccdr.rb);

        Dma::reset_mux(&mut dma_mux);

        let dma1_rb = unsafe { &*DMA1::ptr() };
        let dma2_rb = unsafe { &*DMA2::ptr() };
        let dma_mux_rb = unsafe { &*DMAMUX1::ptr() };

        let stream_isr_dma_1 = StreamIsr::new(
            &dma1_rb.lisr,
            &dma1_rb.hisr,
            &dma1_rb.lifcr,
            &dma1_rb.hifcr,
        );
        let stream_isr_dma_2 = StreamIsr::new(
            &dma2_rb.lisr,
            &dma2_rb.hisr,
            &dma2_rb.lifcr,
            &dma2_rb.hifcr,
        );

        let mux_isr = MuxIsr {
            csr: &dma_mux_rb.csr,
            cfr: &dma_mux_rb.cfr,
        };
        let req_gen_isr =
            RequestGenIsr::new(&dma_mux_rb.rgsr, &dma_mux_rb.rgcfr);
        let mux_shared = MuxShared::new(mux_isr, req_gen_isr);

        let dma_shared = DmaShared {
            stream_isr_dma_1,
            stream_isr_dma_2,
            mux_shared,
        };

        let channels_dma_1 = (
            Channel {
                stream: Stream::after_reset(&dma1_rb.st[0]),
                mux: DmaMux::after_reset(&dma_mux_rb.ccr[0]),
            },
            Channel {
                stream: Stream::after_reset(&dma1_rb.st[1]),
                mux: DmaMux::after_reset(&dma_mux_rb.ccr[1]),
            },
            Channel {
                stream: Stream::after_reset(&dma1_rb.st[2]),
                mux: DmaMux::after_reset(&dma_mux_rb.ccr[2]),
            },
            Channel {
                stream: Stream::after_reset(&dma1_rb.st[3]),
                mux: DmaMux::after_reset(&dma_mux_rb.ccr[3]),
            },
            Channel {
                stream: Stream::after_reset(&dma1_rb.st[4]),
                mux: DmaMux::after_reset(&dma_mux_rb.ccr[4]),
            },
            Channel {
                stream: Stream::after_reset(&dma1_rb.st[5]),
                mux: DmaMux::after_reset(&dma_mux_rb.ccr[5]),
            },
            Channel {
                stream: Stream::after_reset(&dma1_rb.st[6]),
                mux: DmaMux::after_reset(&dma_mux_rb.ccr[6]),
            },
            Channel {
                stream: Stream::after_reset(&dma1_rb.st[7]),
                mux: DmaMux::after_reset(&dma_mux_rb.ccr[7]),
            },
        );

        let channels_dma_2 = (
            Channel {
                stream: Stream::after_reset(&dma2_rb.st[0]),
                mux: DmaMux::after_reset(&dma_mux_rb.ccr[8]),
            },
            Channel {
                stream: Stream::after_reset(&dma2_rb.st[1]),
                mux: DmaMux::after_reset(&dma_mux_rb.ccr[9]),
            },
            Channel {
                stream: Stream::after_reset(&dma2_rb.st[2]),
                mux: DmaMux::after_reset(&dma_mux_rb.ccr[10]),
            },
            Channel {
                stream: Stream::after_reset(&dma2_rb.st[3]),
                mux: DmaMux::after_reset(&dma_mux_rb.ccr[11]),
            },
            Channel {
                stream: Stream::after_reset(&dma2_rb.st[4]),
                mux: DmaMux::after_reset(&dma_mux_rb.ccr[12]),
            },
            Channel {
                stream: Stream::after_reset(&dma2_rb.st[5]),
                mux: DmaMux::after_reset(&dma_mux_rb.ccr[13]),
            },
            Channel {
                stream: Stream::after_reset(&dma2_rb.st[6]),
                mux: DmaMux::after_reset(&dma_mux_rb.ccr[14]),
            },
            Channel {
                stream: Stream::after_reset(&dma2_rb.st[7]),
                mux: DmaMux::after_reset(&dma_mux_rb.ccr[15]),
            },
        );

        let request_generators = (
            RequestGenerator::after_reset(&dma_mux_rb.rgcr[0]),
            RequestGenerator::after_reset(&dma_mux_rb.rgcr[1]),
            RequestGenerator::after_reset(&dma_mux_rb.rgcr[2]),
            RequestGenerator::after_reset(&dma_mux_rb.rgcr[3]),
            RequestGenerator::after_reset(&dma_mux_rb.rgcr[4]),
            RequestGenerator::after_reset(&dma_mux_rb.rgcr[5]),
            RequestGenerator::after_reset(&dma_mux_rb.rgcr[6]),
            RequestGenerator::after_reset(&dma_mux_rb.rgcr[7]),
        );

        Dma {
            channels_dma_1,
            channels_dma_2,
            dma_shared,
            request_generators,
            _dma_1: dma_1,
            _dma_2: dma_2,
            _dma_mux: dma_mux,
        }
    }

    /// Resets the DMA
    fn reset_dma(rcc: &mut RCC) {
        rcc.ahb1rstr.modify(|_, w| w.dma1rst().set_bit());
        rcc.ahb1rstr.modify(|_, w| w.dma1rst().clear_bit());
        rcc.ahb1rstr.modify(|_, w| w.dma2rst().set_bit());
        rcc.ahb1rstr.modify(|_, w| w.dma2rst().clear_bit());
    }

    /// Enables the DMA clock
    fn enable_dma(rcc: &mut RCC) {
        rcc.ahb1enr.modify(|_, w| w.dma1en().set_bit());
        rcc.ahb1enr.modify(|_, w| w.dma2en().set_bit());
    }

    /// Resets the MUX by manually clearing all bits
    fn reset_mux(mux: &mut DMAMUX1) {
        for ccr in mux.ccr.iter() {
            ccr.reset();
        }

        mux.cfr.write(|w| {
            w.csof0()
                .set_bit()
                .csof1()
                .set_bit()
                .csof2()
                .set_bit()
                .csof3()
                .set_bit()
                .csof4()
                .set_bit()
                .csof5()
                .set_bit()
                .csof6()
                .set_bit()
                .csof7()
                .set_bit()
                .csof8()
                .set_bit()
                .csof9()
                .set_bit()
                .csof10()
                .set_bit()
                .csof11()
                .set_bit()
                .csof12()
                .set_bit()
                .csof13()
                .set_bit()
                .csof14()
                .set_bit()
                .csof15()
                .set_bit()
        });

        for rgcr in mux.rgcr.iter() {
            rgcr.reset();
        }

        mux.rgcfr.write(|w| {
            w.cof0()
                .set_bit()
                .cof1()
                .set_bit()
                .cof2()
                .set_bit()
                .cof3()
                .set_bit()
                .cof4()
                .set_bit()
                .cof5()
                .set_bit()
                .cof6()
                .set_bit()
                .cof7()
                .set_bit()
        });
    }
}

pub trait DmaExt: DMATrait {
    type Other: DMATrait;

    fn dma(
        self,
        other_dma: Self::Other,
        dma_mux: DMAMUX1,
        ccdr: &mut Ccdr,
    ) -> Dma;
}

impl DmaExt for DMA1 {
    type Other = DMA2;

    fn dma(self, dma_2: DMA2, dma_mux: DMAMUX1, ccdr: &mut Ccdr) -> Dma {
        Dma::new(self, dma_2, dma_mux, ccdr)
    }
}

impl DmaExt for DMA2 {
    type Other = DMA1;

    fn dma(self, dma_1: DMA1, dma_mux: DMAMUX1, ccdr: &mut Ccdr) -> Dma {
        Dma::new(dma_1, self, dma_mux, ccdr)
    }
}
