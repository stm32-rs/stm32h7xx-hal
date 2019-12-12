#![warn(clippy::all)]

#[macro_use]
mod macros;
pub mod channel;
pub mod stream;
mod utils;

use self::channel::ChannelId;
use self::stm32::dma1::ST;
use self::stm32::{DMA1, DMA2};
use self::stream::{
    BufferMode, CircularMode, CurrentTarget, DirectModeErrorInterrupt,
    Disabled, Disabling, Enabled, Error, Event, FifoErrorInterrupt,
    FifoThreshold, FlowController, HalfTransferInterrupt, IntoNum, IsrCleared,
    IsrState as IsrStateTrait, IsrUncleared, M0a, M1a, MBurst, MSize, Minc,
    Ndt, NotDisabled, PBurst, PSize, Pa, Pinc, Pincos, PriorityLevel,
    StreamIsr, TransferCompleteInterrupt, TransferDirection,
    TransferErrorInterrupt, TransferMode, ED as EDTrait,
};
use crate::nb::{self, Error as NbError};
use core::convert::{Infallible, TryInto};
use core::marker::PhantomData;
use stm32h7::stm32h743 as stm32;

pub unsafe trait DMATrait {}
unsafe impl DMATrait for DMA1 {}
unsafe impl DMATrait for DMA2 {}

pub struct Stream<CXX, DMA, ED, IsrState>
where
    CXX: ChannelId,
    DMA: DMATrait,
    ED: EDTrait,
    IsrState: IsrStateTrait,
{
    rb: &'static ST,
    _phantom_data: PhantomData<(CXX, DMA, ED, IsrState)>,
}

impl<CXX, DMA> Stream<CXX, DMA, Disabled, IsrCleared>
where
    CXX: ChannelId,
    DMA: DMATrait,
{
    fn new(rb: &'static ST) -> Self {
        Stream {
            rb,
            _phantom_data: PhantomData,
        }
    }
}

impl<CXX, DMA, ED, IsrState> Stream<CXX, DMA, ED, IsrState>
where
    CXX: ChannelId,
    DMA: DMATrait,
    ED: EDTrait,
    IsrState: IsrStateTrait,
{
    pub fn id(&self) -> usize {
        CXX::STREAM_ID
    }

    pub fn is_enabled(&self) -> bool {
        self.rb.cr.read().en().bit_is_set()
    }

    pub fn transfer_complete_interrupt(&self) -> TransferCompleteInterrupt {
        self.rb.cr.read().tcie().bit().into()
    }

    pub fn half_transfer_interrupt(&self) -> HalfTransferInterrupt {
        self.rb.cr.read().htie().bit().into()
    }

    pub fn transfer_error_interrupt(&self) -> TransferErrorInterrupt {
        self.rb.cr.read().teie().bit().into()
    }

    pub fn direct_mode_error_interrupt(&self) -> DirectModeErrorInterrupt {
        self.rb.cr.read().dmeie().bit().into()
    }

    pub fn fifo_error_interrupt(&self) -> FifoErrorInterrupt {
        self.rb.fcr.read().feie().bit().into()
    }

    pub fn flow_controller(&self) -> FlowController {
        self.rb.cr.read().pfctrl().bit().into()
    }

    pub fn transfer_direction(&self) -> TransferDirection {
        self.rb.cr.read().dir().bits().try_into().unwrap()
    }

    pub fn circular_mode(&self) -> CircularMode {
        self.rb.cr.read().circ().bit().into()
    }

    pub fn pinc(&self) -> Pinc {
        self.rb.cr.read().pinc().bit().into()
    }

    pub fn minc(&self) -> Minc {
        self.rb.cr.read().minc().bit().into()
    }

    pub fn p_size(&self) -> PSize {
        self.rb.cr.read().psize().bits().try_into().unwrap()
    }

    pub fn m_size(&self) -> MSize {
        self.rb.cr.read().msize().bits().try_into().unwrap()
    }

    pub fn pincos(&self) -> Pincos {
        self.rb.cr.read().pincos().bit().into()
    }

    pub fn priority_level(&self) -> PriorityLevel {
        self.rb.cr.read().pl().bits().try_into().unwrap()
    }

    pub fn buffer_mode(&self) -> BufferMode {
        self.rb.cr.read().dbm().bit().into()
    }

    pub fn current_target(&self) -> CurrentTarget {
        self.rb.cr.read().ct().bit().into()
    }

    pub fn p_burst(&self) -> PBurst {
        self.rb.cr.read().pburst().bits().try_into().unwrap()
    }

    pub fn m_burst(&self) -> MBurst {
        self.rb.cr.read().mburst().bits().try_into().unwrap()
    }

    pub fn ndt(&self) -> Ndt {
        self.rb.ndtr.read().ndt().bits().into()
    }

    pub fn pa(&self) -> Pa {
        self.rb.par.read().pa().bits().into()
    }

    pub fn m0a(&self) -> M0a {
        self.rb.m0ar.read().m0a().bits().into()
    }

    pub fn m1a(&self) -> M1a {
        self.rb.m1ar.read().m1a().bits().into()
    }

    pub fn fifo_threshold(&self) -> FifoThreshold {
        self.rb.fcr.read().fth().bits().try_into().unwrap()
    }

    pub fn transfer_mode(&self) -> TransferMode {
        self.rb.fcr.read().dmdis().bit().into()
    }

    fn impl_set_m0a(&mut self, m0a: M0a) {
        unsafe {
            self.rb.m0ar.modify(|_, w| w.m0a().bits(m0a.into()));
        }
    }

    fn impl_set_m1a(&mut self, m1a: M1a) {
        unsafe {
            self.rb.m0ar.modify(|_, w| w.m0a().bits(m1a.into()));
        }
    }

    fn transmute<NewED, NewIsrState>(
        self,
    ) -> Stream<CXX, DMA, NewED, NewIsrState>
    where
        NewED: EDTrait,
        NewIsrState: IsrStateTrait,
    {
        Stream {
            rb: self.rb,
            _phantom_data: PhantomData,
        }
    }
}

impl<CXX, DMA, IsrState> Stream<CXX, DMA, Disabled, IsrState>
where
    CXX: ChannelId,
    DMA: DMATrait,
    IsrState: IsrStateTrait,
{
    pub fn set_flow_controller(&mut self, flow_controller: FlowController) {
        self.rb
            .cr
            .modify(|_, w| w.pfctrl().bit(flow_controller.into()));
    }

    pub fn set_transfer_direction(&mut self, transfer_dir: TransferDirection) {
        unsafe {
            self.rb.cr.modify(|_, w| w.dir().bits(transfer_dir.into()));
        }
    }

    pub fn set_circular_mode(&mut self, circ_mode: CircularMode) {
        self.rb.cr.modify(|_, w| w.circ().bit(circ_mode.into()));
    }

    pub fn set_pinc(&mut self, pinc: Pinc) {
        self.rb.cr.modify(|_, w| w.pinc().bit(pinc.into()));
    }

    pub fn set_minc(&mut self, minc: Minc) {
        self.rb.cr.modify(|_, w| w.minc().bit(minc.into()));
    }

    pub fn set_p_size(&mut self, p_size: PSize) {
        unsafe {
            self.rb.cr.modify(|_, w| w.psize().bits(p_size.into()));
        }
    }

    pub fn set_m_size(&mut self, m_size: MSize) {
        unsafe {
            self.rb.cr.modify(|_, w| w.msize().bits(m_size.into()));
        }
    }

    pub fn set_pincos(&mut self, pincos: Pincos) {
        self.rb.cr.modify(|_, w| w.pincos().bit(pincos.into()));
    }

    pub fn set_priority_level(&mut self, priority_level: PriorityLevel) {
        unsafe {
            self.rb.cr.modify(|_, w| w.pl().bits(priority_level.into()));
        }
    }

    pub fn set_buffer_mode(&mut self, buffer_mode: BufferMode) {
        self.rb.cr.modify(|_, w| w.dbm().bit(buffer_mode.into()));
    }

    pub fn set_current_target(&mut self, current_target: CurrentTarget) {
        self.rb.cr.modify(|_, w| w.ct().bit(current_target.into()));
    }

    pub fn set_p_burst(&mut self, p_burst: PBurst) {
        unsafe {
            self.rb.cr.modify(|_, w| w.pburst().bits(p_burst.into()));
        }
    }

    pub fn set_m_burst(&mut self, m_burst: MBurst) {
        unsafe {
            self.rb.cr.modify(|_, w| w.mburst().bits(m_burst.into()));
        }
    }

    pub fn set_ndt(&mut self, ndt: Ndt) {
        unsafe {
            self.rb.ndtr.modify(|_, w| w.ndt().bits(ndt.into()));
        }
    }

    pub fn set_pa(&mut self, pa: Pa) {
        unsafe {
            self.rb.par.modify(|_, w| w.pa().bits(pa.into()));
        }
    }

    pub fn set_m0a(&mut self, m0a: M0a) {
        self.impl_set_m0a(m0a);
    }

    pub fn set_m1a(&mut self, m1a: M1a) {
        self.impl_set_m1a(m1a);
    }

    pub fn set_fifo_threshold(&mut self, fifo_threshold: FifoThreshold) {
        unsafe {
            self.rb
                .fcr
                .modify(|_, w| w.fth().bits(fifo_threshold.into()));
        }
    }

    pub fn set_transfer_mode(&mut self, transfer_mode: TransferMode) {
        self.rb
            .fcr
            .modify(|_, w| w.dmdis().bit(transfer_mode.into()));
    }
}

impl<CXX, DMA, ED, IsrState> Stream<CXX, DMA, ED, IsrState>
where
    CXX: ChannelId,
    DMA: DMATrait,
    ED: NotDisabled,
    IsrState: IsrStateTrait,
{
    pub fn set_m0a(&mut self, m0a: M0a) -> nb::Result<(), Infallible> {
        self.check_buffer_mode_addr_change();

        if self.current_target() == CurrentTarget::M0a && self.is_enabled() {
            return Err(NbError::WouldBlock);
        }

        self.impl_set_m0a(m0a);

        Ok(())
    }

    pub fn set_m1a(&mut self, m1a: M1a) -> nb::Result<(), Infallible> {
        self.check_buffer_mode_addr_change();

        if self.current_target() == CurrentTarget::M1a && self.is_enabled() {
            return Err(NbError::WouldBlock);
        }

        self.impl_set_m1a(m1a);

        Ok(())
    }

    fn check_buffer_mode_addr_change(&self) {
        if self.buffer_mode() == BufferMode::Regular {
            panic!("The buffer must be in double buffer mode to be changed on the fly.");
        }
    }
}

impl<CXX, DMA> Stream<CXX, DMA, Disabled, IsrCleared>
where
    CXX: ChannelId,
    DMA: DMATrait,
{
    /// # Safety
    ///
    /// Aliasing rules aren't enforced
    pub unsafe fn enable(self) -> Stream<CXX, DMA, Enabled, IsrUncleared> {
        self.check_config();

        self.enable_unchecked()
    }

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

    fn check_config(&self) {
        if self.circular_mode() == CircularMode::Enabled {
            self.check_config_circular();
        }
        if self.transfer_mode() == TransferMode::Fifo {
            self.check_config_fifo();
        }
    }

    fn check_config_circular(&self) {
        if self.transfer_mode() == TransferMode::Fifo {
            let ndt = self.ndt().value();
            let m_burst = self.m_burst().into_num() as u16;
            let p_burst = self.p_burst().into_num() as u16;
            let m_size = self.m_size().into_num() as u16;
            let p_size = self.p_size().into_num() as u16;

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
            let ndt = self.ndt().value();
            let p_size = self.p_size().into_num() as u16;

            if ndt % p_size != 0 {
                panic!(
                    "Data integrity not guaranteed, because \
                     `num_data_items != Multiple of (p_size)`"
                );
            }
        }
    }

    fn check_config_fifo(&self) {
        if self.m_burst() != MBurst::Single {
            self.check_config_fifo_m_burst();
        }

        if self.p_burst() != PBurst::Single {
            self.check_config_fifo_p_burst();
        }
    }

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
}

impl<CXX, DMA, IsrState> Stream<CXX, DMA, Enabled, IsrState>
where
    CXX: ChannelId,
    DMA: DMATrait,
    IsrState: IsrStateTrait,
{
    pub fn disable(self) -> Stream<CXX, DMA, Disabling, IsrState> {
        self.rb.cr.modify(|_, w| w.en().clear_bit());

        self.transmute()
    }
}

impl<CXX, DMA, IsrState> Stream<CXX, DMA, Disabling, IsrState>
where
    CXX: ChannelId,
    DMA: DMATrait,
    IsrState: IsrStateTrait,
{
    pub fn await_disabled(self) -> Stream<CXX, DMA, Disabled, IsrState> {
        while self.rb.cr.read().en().bit_is_set() {}

        self.transmute()
    }
}

impl<CXX, DMA, ED> Stream<CXX, DMA, ED, IsrUncleared>
where
    CXX: ChannelId,
    DMA: DMATrait,
    ED: EDTrait,
{
    pub fn check_isr(
        &self,
        isr: &StreamIsr<DMA>,
    ) -> Result<Option<Event>, Error> {
        let transfer_error = self.is_transfer_error(isr);
        let direct_mode_error = self.is_direct_mode_error(isr);
        let fifo_error = self.is_fifo_error(isr);

        let event = if self.is_transfer_completed(isr) {
            Some(Event::TransferComplete)
        } else if self.is_half_transfer(isr) {
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

    pub fn is_transfer_completed(&self, isr: &StreamIsr<DMA>) -> bool {
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

    pub fn is_half_transfer(&self, isr: &StreamIsr<DMA>) -> bool {
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

    pub fn is_transfer_error(&self, isr: &StreamIsr<DMA>) -> bool {
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

    pub fn is_direct_mode_error(&self, isr: &StreamIsr<DMA>) -> bool {
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

    pub fn is_fifo_error(&self, isr: &StreamIsr<DMA>) -> bool {
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

    fn clear_isr_impl(&self, isr: &mut StreamIsr<DMA>) {
        self.clear_transfer_complete(isr);
        self.clear_half_transfer(isr);
        self.clear_transfer_error(isr);
        self.clear_direct_mode_error(isr);
        self.clear_fifo_error(isr);
    }

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
    CXX: ChannelId,
    DMA: DMATrait,
{
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
    CXX: ChannelId,
    DMA: DMATrait,
    ED: NotDisabled,
{
    pub fn clear_isr(&self, isr: &mut StreamIsr<DMA>) {
        self.clear_isr_impl(isr);
    }
}

unsafe impl<CXX, DMA, ED, IsrState> Sync for Stream<CXX, DMA, ED, IsrState>
where
    CXX: ChannelId,
    DMA: DMATrait,
    ED: EDTrait,
    IsrState: IsrStateTrait,
{
}
