//! DMA Stream

pub mod config;

use self::config::{
    BufferMode, BufferModeConf, CircularMode, CircularModeConf, CurrentTarget,
    DirectConf, DirectModeErrorInterrupt, DoubleBufferConf, FifoConf,
    FifoErrorInterrupt, FifoThreshold, FlowController, FlowControllerConf,
    HalfTransferInterrupt, IntoNum, M0a, M1a, MBurst, MSize, Minc, Ndt,
    NotM2MConf, PBurst, PBurstConf, PSize, Pa, Pinc, PincConf, Pincos,
    PriorityLevel, TransferCompleteInterrupt, TransferDirection,
    TransferDirectionConf, TransferErrorInterrupt, TransferMode,
    TransferModeConf,
};
use super::{ChannelId, DmaPeripheral};
use crate::nb::Error as NbError;
use crate::stm32::dma1::{HIFCR, HISR, LIFCR, LISR, ST};
use core::convert::{Infallible, TryInto};
use core::marker::PhantomData;

pub use self::config::Config;

/// DMA Stream
pub struct Stream<CXX, ED, IsrState>
where
    CXX: ChannelId,
    ED: IED,
    IsrState: IIsrState,
{
    /// This field *must not* be mutated using shared references
    rb: &'static mut ST,
    config_ndt: Ndt,
    _phantom_data: PhantomData<(CXX, ED, IsrState)>,
}

impl<CXX> Stream<CXX, Disabled, IsrCleared>
where
    CXX: ChannelId,
{
    /// Creates an instance of a Stream in initial state.
    ///
    /// Should only be called after RCC-Reset of the DMA.
    pub(super) fn after_reset(rb: &'static mut ST) -> Self {
        Stream {
            rb,
            config_ndt: Ndt::default(),
            _phantom_data: PhantomData,
        }
    }
}

impl<CXX, ED, IsrState> Stream<CXX, ED, IsrState>
where
    CXX: ChannelId,
    ED: IED,
    IsrState: IIsrState,
{
    pub fn config(&self) -> Config {
        Config {
            transfer_complete_interrupt: self.transfer_complete_interrupt(),
            half_transfer_interrupt: self.half_transfer_interrupt(),
            transfer_error_interrupt: self.transfer_error_interrupt(),
            direct_mode_error_interrupt: self.direct_mode_error_interrupt(),
            fifo_error_interrupt: self.fifo_error_interrupt(),
            minc: self.minc(),
            priority_level: self.priority_level(),
            p_size: self.p_size(),
            ndt: self.ndt(),
            pa: self.pa(),
            m0a: self.m0a(),
            transfer_direction: self.transfer_direction_config(),
        }
    }

    fn transfer_direction_config(&self) -> TransferDirectionConf {
        match self.transfer_direction() {
            TransferDirection::P2M => {
                TransferDirectionConf::P2M(self.not_m2m_config())
            }
            TransferDirection::M2P => {
                TransferDirectionConf::M2P(self.not_m2m_config())
            }
            TransferDirection::M2M => {
                TransferDirectionConf::M2M(self.fifo_config())
            }
        }
    }

    fn not_m2m_config(&self) -> NotM2MConf {
        NotM2MConf {
            transfer_mode: self.transfer_mode_config(),
            flow_controller: self.flow_controller_config(),
        }
    }

    fn transfer_mode_config(&self) -> TransferModeConf {
        match self.transfer_mode() {
            TransferMode::Direct => {
                TransferModeConf::Direct(self.direct_conf())
            }
            TransferMode::Fifo => TransferModeConf::Fifo(self.fifo_config()),
        }
    }

    fn flow_controller_config(&self) -> FlowControllerConf {
        match self.flow_controller() {
            FlowController::Dma => {
                FlowControllerConf::Dma(self.circular_mode_config())
            }
            FlowController::Peripheral => FlowControllerConf::Peripheral,
        }
    }

    fn circular_mode_config(&self) -> CircularModeConf {
        match self.circular_mode() {
            CircularMode::Disabled => CircularModeConf::Disabled,
            CircularMode::Enabled => {
                CircularModeConf::Enabled(self.buffer_mode_config())
            }
        }
    }

    fn buffer_mode_config(&self) -> BufferModeConf {
        match self.buffer_mode() {
            BufferMode::Regular => BufferModeConf::Regular,
            BufferMode::DoubleBuffer => {
                BufferModeConf::DoubleBuffer(self.double_buffer_config())
            }
        }
    }

    fn double_buffer_config(&self) -> DoubleBufferConf {
        DoubleBufferConf {
            current_target: self.current_target(),
            m1a: self.m1a().unwrap(),
        }
    }

    fn direct_conf(&self) -> DirectConf {
        DirectConf { pinc: self.pinc() }
    }

    fn fifo_config(&self) -> FifoConf {
        FifoConf {
            fifo_threshold: self.fifo_threshold().unwrap(),
            p_burst: self.p_burst_config(),
            m_burst: self.m_burst(),
            m_size: self.m_size(),
        }
    }

    fn p_burst_config(&self) -> PBurstConf {
        match self.p_burst() {
            PBurst::Single => PBurstConf::Single(self.pinc_conf()),
            PBurst::Incr4 => PBurstConf::Incr4(self.pinc()),
            PBurst::Incr8 => PBurstConf::Incr8(self.pinc()),
            PBurst::Incr16 => PBurstConf::Incr16(self.pinc()),
        }
    }

    fn pinc_conf(&self) -> PincConf {
        match self.pinc() {
            Pinc::Fixed => PincConf::Fixed,
            Pinc::Incremented => PincConf::Incremented(self.pincos()),
        }
    }

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
    fn set_transfer_complete_interrupt(
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
    fn set_half_transfer_interrupt(
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
    fn set_transfer_error_interrupt(
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
    fn set_direct_mode_error_interrupt(
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
    fn set_fifo_error_interrupt(&mut self, fe_intrpt: FifoErrorInterrupt) {
        self.rb.fcr.modify(|_, w| w.feie().bit(fe_intrpt.into()));
    }

    /// Returns the Flow Controller
    pub fn flow_controller(&self) -> FlowController {
        self.rb.cr.read().pfctrl().bit().into()
    }

    /// Returns the Transfer Direction
    pub fn transfer_direction(&self) -> TransferDirection {
        self.rb.cr.read().dir().bits().try_into().unwrap()
    }

    /// Returns the Circular Mode
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
    pub fn m_size(&self) -> MSize {
        self.rb.cr.read().msize().bits().try_into().unwrap()
    }

    /// Returns the Peripheral Increment Offset
    pub fn pincos(&self) -> Pincos {
        self.rb.cr.read().pincos().bit().into()
    }

    /// Returns the Priority Level
    pub fn priority_level(&self) -> PriorityLevel {
        self.rb.cr.read().pl().bits().try_into().unwrap()
    }

    /// Returns the Buffer Mode
    pub fn buffer_mode(&self) -> BufferMode {
        self.rb.cr.read().dbm().bit().into()
    }

    /// Returns the Current Target
    pub fn current_target(&self) -> CurrentTarget {
        self.rb.cr.read().ct().bit().into()
    }

    /// Returns the Peripheral Burst config flag
    pub fn p_burst(&self) -> PBurst {
        self.rb.cr.read().pburst().bits().try_into().unwrap()
    }

    /// Returns the Memory Burst config flag
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
    pub fn m1a(&self) -> Option<M1a> {
        if self.buffer_mode() == BufferMode::DoubleBuffer {
            Some(self.rb.m1ar.read().m1a().bits().into())
        } else {
            None
        }
    }

    /// Returns the Fifo Threshold
    pub fn fifo_threshold(&self) -> Option<FifoThreshold> {
        if self.transfer_mode() == TransferMode::Fifo {
            Some(self.rb.fcr.read().fth().bits().try_into().unwrap())
        } else {
            None
        }
    }

    /// Returns the Transfer Mode (`Direct` or `Fifo` Mode)
    pub fn transfer_mode(&self) -> TransferMode {
        self.rb.fcr.read().dmdis().bit().into()
    }

    /// Performs the volatile write to the `M0a` register
    fn impl_set_m0a(&mut self, m0a: M0a) {
        self.rb.m0ar.modify(|_, w| w.m0a().bits(m0a.into()));
    }

    /// Performs the volatile write to the `M1a` register
    fn impl_set_m1a(&mut self, m1a: M1a) {
        self.rb.m0ar.modify(|_, w| w.m0a().bits(m1a.into()));
    }

    /// Transmutes the state of `self`
    fn transmute<NewED, NewIsrState>(self) -> Stream<CXX, NewED, NewIsrState>
    where
        NewED: IED,
        NewIsrState: IIsrState,
    {
        Stream {
            rb: self.rb,
            config_ndt: self.config_ndt,
            _phantom_data: PhantomData,
        }
    }
}

impl<CXX, IsrState> Stream<CXX, Disabled, IsrState>
where
    CXX: ChannelId,
    IsrState: IIsrState,
{
    pub fn apply_config(&mut self, config: Config) {
        self.set_transfer_complete_interrupt(
            config.transfer_complete_interrupt,
        );
        self.set_half_transfer_interrupt(config.half_transfer_interrupt);
        self.set_transfer_error_interrupt(config.transfer_error_interrupt);
        self.set_direct_mode_error_interrupt(
            config.direct_mode_error_interrupt,
        );
        self.set_fifo_error_interrupt(config.fifo_error_interrupt);
        self.set_pinc(config.pinc());
        self.set_minc(config.minc);
        self.set_priority_level(config.priority_level);
        self.set_p_size(config.p_size);
        self.set_ndt(config.ndt);
        self.set_pa(config.pa);
        self.set_m0a(config.m0a);

        self.set_transfer_direction(config.transfer_direction());
        self.set_transfer_mode(config.transfer_mode());
        self.set_flow_controller(config.flow_controller());
        self.set_circular_mode(config.circular_mode());
        self.set_buffer_mode(config.buffer_mode());

        if let Some(fifo_threshold) = config.fifo_threshold() {
            self.set_fifo_threshold(fifo_threshold);
        }
        self.set_p_burst(config.p_burst());
        self.set_m_burst(config.m_burst());
        self.set_m_size(config.m_size());

        if let Some(pincos) = config.pincos() {
            self.set_pincos(pincos);
        }

        self.set_current_target(config.current_target());
        if let Some(m1a) = config.m1a() {
            self.set_m1a(m1a);
        }
    }

    /// Sets the Flow Controller
    fn set_flow_controller(&mut self, flow_controller: FlowController) {
        self.rb
            .cr
            .modify(|_, w| w.pfctrl().bit(flow_controller.into()));
    }

    /// Sets the Transfer Direction
    fn set_transfer_direction(&mut self, transfer_dir: TransferDirection) {
        unsafe {
            self.rb.cr.modify(|_, w| w.dir().bits(transfer_dir.into()));
        }
    }

    /// Sets the Circular Mode
    fn set_circular_mode(&mut self, circ_mode: CircularMode) {
        self.rb.cr.modify(|_, w| w.circ().bit(circ_mode.into()));
    }

    /// Sets the Peripheral Increment config flag
    fn set_pinc(&mut self, pinc: Pinc) {
        self.rb.cr.modify(|_, w| w.pinc().bit(pinc.into()));
    }

    /// Sets the Memory Increment config flag
    fn set_minc(&mut self, minc: Minc) {
        self.rb.cr.modify(|_, w| w.minc().bit(minc.into()));
    }

    /// Sets the Peripheral Size
    fn set_p_size(&mut self, p_size: PSize) {
        unsafe {
            self.rb.cr.modify(|_, w| w.psize().bits(p_size.into()));
        }
    }

    /// Sets the Memory Size
    fn set_m_size(&mut self, m_size: MSize) {
        unsafe {
            self.rb.cr.modify(|_, w| w.msize().bits(m_size.into()));
        }
    }

    /// Sets the Peripheral Increment Offset
    fn set_pincos(&mut self, pincos: Pincos) {
        self.rb.cr.modify(|_, w| w.pincos().bit(pincos.into()));
    }

    /// Sets the Priority Level
    fn set_priority_level(&mut self, priority_level: PriorityLevel) {
        self.rb.cr.modify(|_, w| w.pl().bits(priority_level.into()));
    }

    /// Sets the Buffer Mode (`Direct` or `Fifo` mode)
    fn set_buffer_mode(&mut self, buffer_mode: BufferMode) {
        self.rb.cr.modify(|_, w| w.dbm().bit(buffer_mode.into()));
    }

    /// Sets the Current Target
    fn set_current_target(&mut self, current_target: CurrentTarget) {
        self.rb.cr.modify(|_, w| w.ct().bit(current_target.into()));
    }

    /// Sets the Peripheral Burst
    fn set_p_burst(&mut self, p_burst: PBurst) {
        self.rb.cr.modify(|_, w| w.pburst().bits(p_burst.into()));
    }

    /// Sets the Memory Burst
    fn set_m_burst(&mut self, m_burst: MBurst) {
        self.rb.cr.modify(|_, w| w.mburst().bits(m_burst.into()));
    }

    /// Sets the NDT register
    fn set_ndt(&mut self, ndt: Ndt) {
        self.config_ndt = ndt;

        self.rb.ndtr.modify(|_, w| w.ndt().bits(ndt.into()));
    }

    /// Sets the Peripheral Address
    fn set_pa(&mut self, pa: Pa) {
        self.rb.par.modify(|_, w| w.pa().bits(pa.into()));
    }

    /// Sets the Memory-0 Address
    fn set_m0a(&mut self, m0a: M0a) {
        self.impl_set_m0a(m0a);
    }

    /// Sets the Memory-1 Address
    fn set_m1a(&mut self, m1a: M1a) {
        self.impl_set_m1a(m1a);
    }

    /// Sets the Fifo Threshold
    fn set_fifo_threshold(&mut self, fifo_threshold: FifoThreshold) {
        self.rb
            .fcr
            .modify(|_, w| w.fth().bits(fifo_threshold.into()));
    }

    /// Sets the Transfer Mode
    fn set_transfer_mode(&mut self, transfer_mode: TransferMode) {
        self.rb
            .fcr
            .modify(|_, w| w.dmdis().bit(transfer_mode.into()));
    }
}

impl<CXX, IsrState> Stream<CXX, Enabled, IsrState>
where
    CXX: ChannelId,
    IsrState: IIsrState,
{
    /// Sets the Memory-0 Address on the fly
    ///
    /// # Safety
    ///
    /// Aliasing rules aren't enforced.
    ///
    /// # Panic
    ///
    /// This panics if the stream is not in Double Buffer Mode.
    pub unsafe fn set_m0a(&mut self, m0a: M0a) -> nb::Result<(), Infallible> {
        self.check_double_buffer();

        if self.current_target() == CurrentTarget::M0a && self.is_enabled() {
            return Err(NbError::WouldBlock);
        }

        self.impl_set_m0a(m0a);

        Ok(())
    }

    /// Sets the Memory-1 Address on the fly
    ///
    /// # Safety
    ///
    /// Aliasing rules aren't enforced.
    ///
    /// # Panic
    ///
    /// This panics if the stream is not in Double Buffer Mode.
    pub unsafe fn set_m1a(&mut self, m1a: M1a) -> nb::Result<(), Infallible> {
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

impl<CXX> Stream<CXX, Disabled, IsrCleared>
where
    CXX: ChannelId,
{
    /// Checks the config for data integrity and enables the stream
    ///
    /// # Safety
    ///
    /// Aliasing rules aren't enforced.
    pub unsafe fn enable(self) -> Stream<CXX, Enabled, IsrUncleared> {
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
    pub unsafe fn enable_unchecked(self) -> Stream<CXX, Enabled, IsrUncleared> {
        self.rb.cr.modify(|_, w| w.en().set_bit());

        self.transmute()
    }

    /// Checks the config for data integrity
    fn check_config(&self) {
        if self.circular_mode() == CircularMode::Enabled {
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
        let fifo_size = self.fifo_threshold().unwrap().into_num() * 4;

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
            && self.fifo_threshold().unwrap() == FifoThreshold::F3_4
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

impl<CXX, IsrState> Stream<CXX, Enabled, IsrState>
where
    CXX: ChannelId,
    IsrState: IIsrState,
{
    /// Disables the stream
    pub fn disable(self) -> Stream<CXX, Disabled, IsrState> {
        self.rb.cr.modify(|_, w| w.en().clear_bit());

        while self.rb.cr.read().en().bit_is_set() {}

        self.transmute()
    }
}

impl<CXX, ED> Stream<CXX, ED, IsrUncleared>
where
    CXX: ChannelId,
    ED: IED,
{
    /// Returns the contents of the isr.
    ///
    /// * If there is no error, an optional event is returned as `Ok`
    /// * If there are errors, the errors are being wrapped into `Error` and returned as `Err`
    pub fn check_isr(
        &self,
        isr: &StreamIsr<CXX::DMA>,
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
    pub fn transfer_complete_flag(&self, isr: &StreamIsr<CXX::DMA>) -> bool {
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
    pub fn half_transfer_flag(&self, isr: &StreamIsr<CXX::DMA>) -> bool {
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
    pub fn transfer_error_flag(&self, isr: &StreamIsr<CXX::DMA>) -> bool {
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
    pub fn direct_mode_error_flag(&self, isr: &StreamIsr<CXX::DMA>) -> bool {
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
    pub fn fifo_error_flag(&self, isr: &StreamIsr<CXX::DMA>) -> bool {
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
    fn clear_isr_impl(&self, isr: &mut StreamIsr<CXX::DMA>) {
        self.clear_transfer_complete(isr);
        self.clear_half_transfer(isr);
        self.clear_transfer_error(isr);
        self.clear_direct_mode_error(isr);
        self.clear_fifo_error(isr);
    }

    /// Clears the Transfer Complete flag
    pub fn clear_transfer_complete(&self, isr: &mut StreamIsr<CXX::DMA>) {
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
    pub fn clear_half_transfer(&self, isr: &mut StreamIsr<CXX::DMA>) {
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
    pub fn clear_transfer_error(&self, isr: &mut StreamIsr<CXX::DMA>) {
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
    pub fn clear_direct_mode_error(&self, isr: &mut StreamIsr<CXX::DMA>) {
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
    pub fn clear_fifo_error(&self, isr: &mut StreamIsr<CXX::DMA>) {
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

impl<CXX> Stream<CXX, Disabled, IsrUncleared>
where
    CXX: ChannelId,
{
    /// Clears the ISR
    pub fn clear_isr(
        self,
        isr: &mut StreamIsr<CXX::DMA>,
    ) -> Stream<CXX, Disabled, IsrCleared> {
        self.clear_isr_impl(isr);

        self.transmute()
    }
}

impl<CXX> Stream<CXX, Enabled, IsrUncleared>
where
    CXX: ChannelId,
{
    /// Clears the ISR
    pub fn clear_isr(&self, isr: &mut StreamIsr<CXX::DMA>) {
        self.clear_isr_impl(isr);
    }

    pub fn wait_until_completed(
        &self,
        isr: &StreamIsr<CXX::DMA>,
    ) -> nb::Result<(), Error> {
        match self.check_isr(isr) {
            Ok(Some(Event::TransferComplete)) => Ok(()),
            Err(err) => Err(NbError::Other(err)),
            _ => Err(NbError::WouldBlock),
        }
    }

    pub fn wait_until_completed_clear(
        &self,
        isr: &mut StreamIsr<CXX::DMA>,
    ) -> nb::Result<(), Error> {
        let res = self.wait_until_completed(isr);

        self.clear_isr_if_not_blocking(res, isr);

        res
    }

    pub fn wait_until_half_transfer(
        &self,
        isr: &StreamIsr<CXX::DMA>,
    ) -> nb::Result<(), Error> {
        match self.check_isr(isr) {
            Ok(Some(Event::HalfTransfer)) => Ok(()),
            Err(err) => Err(NbError::Other(err)),
            _ => Err(NbError::WouldBlock),
        }
    }

    pub fn wait_until_half_transfer_clear(
        &self,
        isr: &mut StreamIsr<CXX::DMA>,
    ) -> nb::Result<(), Error> {
        let res = self.wait_until_half_transfer(isr);

        self.clear_isr_if_not_blocking(res, isr);

        res
    }

    pub fn wait_until_next_half(
        &self,
        isr: &StreamIsr<CXX::DMA>,
    ) -> nb::Result<(), Error> {
        match self.check_isr(isr) {
            Ok(event) => match event {
                Some(Event::HalfTransfer) | Some(Event::TransferComplete) => {
                    Ok(())
                }
                None => Err(NbError::WouldBlock),
            },
            Err(err) => Err(NbError::Other(err)),
        }
    }

    pub fn wait_until_next_half_clear(
        &self,
        isr: &mut StreamIsr<CXX::DMA>,
    ) -> nb::Result<(), Error> {
        let res = self.wait_until_next_half(isr);

        self.clear_isr_if_not_blocking(res, isr);

        res
    }

    fn clear_isr_if_not_blocking(
        &self,
        res: nb::Result<(), Error>,
        isr: &mut StreamIsr<CXX::DMA>,
    ) {
        if !matches!(res, Err(NbError::WouldBlock)) {
            self.clear_isr(isr);
        }
    }
}

unsafe impl<CXX, ED, IsrState> Send for Stream<CXX, ED, IsrState>
where
    CXX: ChannelId,
    ED: IED,
    IsrState: IIsrState,
{
}

unsafe impl<CXX, ED, IsrState> Sync for Stream<CXX, ED, IsrState>
where
    CXX: ChannelId,
    ED: IED,
    IsrState: IIsrState,
{
}

type_state! {
    IED, Disabled, Enabled
}

type_state! {
    IIsrState, IsrCleared, IsrUncleared
}

pub struct StreamIsr<DMA>
where
    DMA: DmaPeripheral,
{
    lisr: &'static LISR,
    hisr: &'static HISR,
    /// This field *must not* be mutated using shared references
    lifcr: &'static mut LIFCR,
    /// This field *must not* be mutated using shared references
    hifcr: &'static mut HIFCR,
    _phantom_data: PhantomData<DMA>,
}

impl<DMA> StreamIsr<DMA>
where
    DMA: DmaPeripheral,
{
    pub(super) fn new(
        lisr: &'static LISR,
        hisr: &'static HISR,
        lifcr: &'static mut LIFCR,
        hifcr: &'static mut HIFCR,
    ) -> Self {
        StreamIsr {
            lisr,
            hisr,
            lifcr,
            hifcr,
            _phantom_data: PhantomData,
        }
    }
}

unsafe impl<DMA> Send for StreamIsr<DMA> where DMA: DmaPeripheral {}
unsafe impl<DMA> Sync for StreamIsr<DMA> where DMA: DmaPeripheral {}

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum Event {
    HalfTransfer,
    TransferComplete,
}

#[derive(Debug, Clone, Copy)]
pub struct Error {
    pub transfer_error: bool,
    pub direct_mode_error: bool,
    pub fifo_error: bool,
    pub event: Option<Event>,
    pub crashed: bool,
}
