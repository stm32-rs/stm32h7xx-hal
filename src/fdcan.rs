#![deny(missing_docs)]

//! FdCAN Operations

/// Configuration of an FdCAN instance
pub mod config;
#[cfg(feature = "embedded-can-03")]
mod embedded_can;
/// Filtering of CAN Messages
pub mod filter;
/// Header and info of transmitted and receiving frames
pub mod frame;
/// Standard and Extended Id
pub mod id;
/// Interrupt Line Information
pub mod interrupt;
mod message_ram;

use id::{Id, IdReg};

use crate::rcc::{rec, rec::ResetEnable};
use crate::stm32::fdcan1::RegisterBlock;
use config::{
    DataBitTiming, FdCanConfig, FrameTransmissionConfig, GlobalFilter,
    NominalBitTiming, TimestampSource,
};
use filter::{
    ActivateFilter as _, ExtendedFilter, ExtendedFilterSlot, StandardFilter,
    StandardFilterSlot, EXTENDED_FILTER_MAX, STANDARD_FILTER_MAX,
};
use frame::MergeTxFrameHeader;
use frame::{RxFrameInfo, TxFrameHeader};
use interrupt::{Interrupt, InterruptLine, Interrupts};

use message_ram::MsgRamExt;
use message_ram::RxFifoElement;

use core::cmp::Ord;
use core::convert::Infallible;
use core::convert::TryFrom;
use core::marker::PhantomData;
use core::ptr::NonNull;

mod sealed {
    /// A TX pin configured for CAN communication
    pub trait Tx<CAN> {}
    /// An RX pin configured for CAN communication
    pub trait Rx<CAN> {}

    pub trait Sealed {}
}

/// An FdCAN peripheral instance.
///
/// This trait is meant to be implemented for a HAL-specific type that represent ownership of
/// the CAN peripheral (and any pins required by it, although that is entirely up to the HAL).
///
/// # Safety
///
/// It is only safe to implement this trait, when:
///
/// * The implementing type has ownership of the peripheral, preventing any other accesses to the
///   register block.
/// * `REGISTERS` is a pointer to that peripheral's register block and can be safely accessed for as
///   long as ownership or a borrow of the implementing type is present.
pub unsafe trait Instance: MsgRamExt {
    /// Pointer to the instance's register block.
    const REGISTERS: *mut RegisterBlock;
}

/// Indicates if an Receive Overflow has occurred
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "unstable-defmt", derive(defmt::Format))]
pub enum ReceiveErrorOverflow {
    /// No overflow has occurred
    Normal(u8),
    /// An overflow has occurred
    Overflow(u8),
}

///Error Counters
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "unstable-defmt", derive(defmt::Format))]
#[allow(dead_code)]
pub struct ErrorCounters {
    /// General CAN error counter
    can_errors: u8,
    /// Receive CAN error counter
    receive_err: ReceiveErrorOverflow,
    /// Transmit CAN error counter
    transmit_err: u8,
}

/// Loopback Mode
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "unstable-defmt", derive(defmt::Format))]
enum LoopbackMode {
    None,
    Internal,
    External,
}

/// Bus Activity
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "unstable-defmt", derive(defmt::Format))]
pub enum Activity {
    /// Node is Synchronizing
    Synchronizing = 0b00,
    /// Node is Idle
    Idle = 0b01,
    /// Node is receiver only
    Receiver = 0b10,
    /// Node is transmitter only
    Transmitter = 0b11,
}
impl TryFrom<u8> for Activity {
    type Error = ();
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0b000 => Ok(Self::Synchronizing),
            0b001 => Ok(Self::Idle),
            0b010 => Ok(Self::Receiver),
            0b011 => Ok(Self::Transmitter),
            _ => Err(()),
        }
    }
}

/// Indicates the type of the last error which occurred on the CAN bus
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "unstable-defmt", derive(defmt::Format))]
pub enum LastErrorCode {
    /// There has been no error since last read
    NoError = 0b000,
    /// More than 5 equal bits in sequence are not allowed
    StuffError = 0b001,
    /// a fixed format part of ta received frame had the wrong format
    FormError = 0b010,
    /// message tramsitted by this node was not acknowledged by another
    AckError = 0b011,
    /// During transmit, the node wanted to send a 1 but monitored a 0
    Bit1Error = 0b100,
    /// During transmit, the node wanted to send a 0 but monitored a 1
    Bit0Error = 0b101,
    /// CRC checksum of a received message was incorrect
    CRCError = 0b110,
    /// No CAN bus event detected since last read
    NoChange = 0b111,
}
impl TryFrom<u8> for LastErrorCode {
    type Error = ();
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0b000 => Ok(Self::NoError),
            0b001 => Ok(Self::StuffError),
            0b010 => Ok(Self::FormError),
            0b011 => Ok(Self::AckError),
            0b100 => Ok(Self::Bit1Error),
            0b101 => Ok(Self::Bit0Error),
            0b110 => Ok(Self::CRCError),
            0b111 => Ok(Self::NoChange),
            _ => Err(()),
        }
    }
}

/// Some status indications regarding the FDCAN protocl
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "unstable-defmt", derive(defmt::Format))]
pub struct ProtocolStatus {
    /// Type of current activity
    activity: Activity,
    /// Transmitter delay companstation
    transmitter_delay_comp: u8,
    /// But Off Status
    bus_off_status: bool,
    /// Shows if Error counters are aat their limit of 96
    error_warning: bool,
    /// Shows if the node send and active error flag (false) or stays silent (true).
    error_passive_state: bool,
    /// Indicates te last type of error which occurred on the CAN bus.
    last_error: LastErrorCode,
}

/// Allows for Transmit Operations
pub trait Transmit {}
/// Allows for Receive Operations
pub trait Receive {}

/// Allows for the FdCan Instance to be released or to enter ConfigMode
pub struct PoweredDownMode;
/// Allows for the configuration for the Instance
pub struct ConfigMode;
/// This mode can be used for a “Hot Selftest”, meaning the FDCAN can be tested without
/// affecting a running CAN system connected to the FDCAN_TX and FDCAN_RX pins. In this
/// mode, FDCAN_RX pin is disconnected from the FDCAN and FDCAN_TX pin is held
/// recessive.
pub struct InternalLoopbackMode;
impl Transmit for InternalLoopbackMode {}
impl Receive for InternalLoopbackMode {}
/// This mode is provided for hardware self-test. To be independent from external stimulation,
/// the FDCAN ignores acknowledge errors (recessive bit sampled in the acknowledge slot of a
/// data / remote frame) in Loop Back mode. In this mode the FDCAN performs an internal
/// feedback from its transmit output to its receive input. The actual value of the FDCAN_RX
/// input pin is disregarded by the FDCAN. The transmitted messages can be monitored at the
/// FDCAN_TX transmit pin.
pub struct ExternalLoopbackMode;
impl Transmit for ExternalLoopbackMode {}
impl Receive for ExternalLoopbackMode {}
/// The normal use of the FdCan instance after configurations
pub struct NormalOperationMode;
impl Transmit for NormalOperationMode {}
impl Receive for NormalOperationMode {}
/// In Restricted operation mode the node is able to receive data and remote frames and to give
/// acknowledge to valid frames, but it does not send data frames, remote frames, active error
/// frames, or overload frames. In case of an error condition or overload condition, it does not
/// send dominant bits, instead it waits for the occurrence of bus idle condition to resynchronize
/// itself to the CAN communication. The error counters for transmit and receive are frozen while
/// error logging (can_errors) is active. TODO: automatically enter in this mode?
pub struct RestrictedOperationMode;
impl Receive for RestrictedOperationMode {}
///  In Bus monitoring mode (for more details refer to ISO11898-1, 10.12 Bus monitoring),
/// the FDCAN is able to receive valid data frames and valid remote frames, but cannot start a
/// transmission. In this mode, it sends only recessive bits on the CAN bus. If the FDCAN is
/// required to send a dominant bit (ACK bit, overload flag, active error flag), the bit is
/// rerouted internally so that the FDCAN can monitor it, even if the CAN bus remains in recessive
/// state. In Bus monitoring mode the TXBRP register is held in reset state. The Bus monitoring
/// mode can be used to analyze the traffic on a CAN bus without affecting it by the transmission
/// of dominant bits.
pub struct BusMonitoringMode;
impl Receive for BusMonitoringMode {}
/// Test mode must be used for production tests or self test only. The software control for
/// FDCAN_TX pin interferes with all CAN protocol functions. It is not recommended to use test
/// modes for application.
pub struct TestMode;

/// Interface to a FdCAN peripheral.
pub struct FdCan<I: Instance, MODE> {
    control: FdCanControl<I, MODE>,
}

impl<I, MODE> FdCan<I, MODE>
where
    I: Instance,
{
    fn create_can<NEW_MODE>(
        config: FdCanConfig,
        instance: I,
    ) -> FdCan<I, NEW_MODE> {
        FdCan {
            control: FdCanControl {
                config,
                instance,
                _mode: core::marker::PhantomData,
            },
        }
    }

    fn into_can_mode<NEW_MODE>(self) -> FdCan<I, NEW_MODE> {
        FdCan {
            control: FdCanControl {
                config: self.control.config,
                instance: self.control.instance,
                _mode: core::marker::PhantomData,
            },
        }
    }

    /// Returns a reference to the peripheral instance.
    ///
    /// This allows accessing HAL-specific data stored in the instance type.
    #[inline]
    pub fn instance(&mut self) -> &mut I {
        &mut self.control.instance
    }

    #[inline]
    fn registers(&self) -> &RegisterBlock {
        unsafe { &*I::REGISTERS }
    }

    #[inline]
    fn msg_ram_mut(&mut self) -> &mut message_ram::RegisterBlock {
        self.instance().msg_ram_mut()
    }

    #[inline]
    fn reset_msg_ram(&mut self) {
        self.msg_ram_mut().reset();
    }

    #[inline]
    fn enter_init_mode(&mut self) {
        let can = self.registers();

        can.cccr.modify(|_, w| w.init().set_bit());
        while can.cccr.read().init().bit_is_clear() {}
        can.cccr.modify(|_, w| w.cce().set_bit());
    }

    /// Returns the current FDCAN config settings
    #[inline]
    pub fn get_config(&self) -> FdCanConfig {
        self.control.config
    }

    /// Enables or disables loopback mode: Internally connects the TX and RX
    /// signals together.
    #[inline]
    fn set_loopback_mode(&mut self, mode: LoopbackMode) {
        let (test, mon, lbck) = match mode {
            LoopbackMode::None => (false, false, false),
            LoopbackMode::Internal => (true, true, true),
            LoopbackMode::External => (true, false, true),
        };

        self.set_test_mode(test);
        self.set_bus_monitoring_mode(mon);

        let can = self.registers();
        can.test.modify(|_, w| w.lbck().bit(lbck));
    }

    /// Enables or disables silent mode: Disconnects the TX signal from the pin.
    #[inline]
    fn set_bus_monitoring_mode(&mut self, enabled: bool) {
        let can = self.registers();
        can.cccr.modify(|_, w| w.mon().bit(enabled));
    }

    #[inline]
    fn set_restricted_operations(&mut self, enabled: bool) {
        let can = self.registers();
        can.cccr.modify(|_, w| w.asm().bit(enabled));
    }

    #[inline]
    fn set_normal_operations(&mut self, _enabled: bool) {
        self.set_loopback_mode(LoopbackMode::None);
    }

    #[inline]
    fn set_test_mode(&mut self, enabled: bool) {
        let can = self.registers();
        can.cccr.modify(|_, w| w.test().bit(enabled));
    }

    #[inline]
    fn set_power_down_mode(&mut self, enabled: bool) {
        let can = self.registers();
        can.cccr.modify(|_, w| w.csr().bit(enabled));
        while can.cccr.read().csa().bit() != enabled {}
    }

    /// Enable/Disable the specific Interrupt Line
    #[inline]
    pub fn enable_interrupt_line(
        &mut self,
        line: InterruptLine,
        enabled: bool,
    ) {
        let can = self.registers();
        match line {
            InterruptLine::_0 => can.ile.modify(|_, w| w.eint0().bit(enabled)),
            InterruptLine::_1 => can.ile.modify(|_, w| w.eint1().bit(enabled)),
        }
    }

    /// Starts listening for a CAN interrupt.
    #[inline]
    pub fn enable_interrupt(&mut self, interrupt: Interrupt) {
        self.enable_interrupts(Interrupts::from_bits_truncate(interrupt as u32))
    }

    /// Starts listening for a set of CAN interrupts.
    #[inline]
    pub fn enable_interrupts(&mut self, interrupts: Interrupts) {
        self.registers()
            .ie
            .modify(|r, w| unsafe { w.bits(r.bits() | interrupts.bits()) })
    }

    /// Stops listening for a CAN interrupt.
    pub fn disable_interrupt(&mut self, interrupt: Interrupt) {
        self.disable_interrupts(Interrupts::from_bits_truncate(
            interrupt as u32,
        ))
    }

    /// Stops listening for a set of CAN interrupts.
    #[inline]
    pub fn disable_interrupts(&mut self, interrupts: Interrupts) {
        self.registers()
            .ie
            .modify(|r, w| unsafe { w.bits(r.bits() & !interrupts.bits()) })
    }

    /// Retrieve the CAN error counters
    #[inline]
    pub fn error_counters(&self) -> ErrorCounters {
        self.control.error_counters()
    }

    /// Set an Standard Address CAN filter into slot 'id'
    #[inline]
    pub fn set_standard_filter(
        &mut self,
        slot: StandardFilterSlot,
        filter: StandardFilter,
    ) {
        self.msg_ram_mut().filters.flssa[slot as usize].activate(filter);
    }

    /// Set an array of Standard Address CAN filters and overwrite the current set
    pub fn set_standard_filters(
        &mut self,
        filters: &[StandardFilter; STANDARD_FILTER_MAX as usize],
    ) {
        for (i, f) in filters.iter().enumerate() {
            self.msg_ram_mut().filters.flssa[i].activate(*f);
        }
    }

    /// Set an Extended Address CAN filter into slot 'id'
    #[inline]
    pub fn set_extended_filter(
        &mut self,
        slot: ExtendedFilterSlot,
        filter: ExtendedFilter,
    ) {
        self.msg_ram_mut().filters.flesa[slot as usize].activate(filter);
    }

    /// Set an array of Extended Address CAN filters and overwrite the current set
    pub fn set_extended_filters(
        &mut self,
        filters: &[ExtendedFilter; EXTENDED_FILTER_MAX as usize],
    ) {
        for (i, f) in filters.iter().enumerate() {
            self.msg_ram_mut().filters.flesa[i].activate(*f);
        }
    }

    /// Retrieve the current protocol status
    pub fn get_protocol_status(&self) -> ProtocolStatus {
        let psr = self.registers().psr.read();
        ProtocolStatus {
            activity: Activity::try_from(0 /*psr.act().bits()*/).unwrap(), //TODO: stm32g4 does not allow reading from this register
            transmitter_delay_comp: psr.tdcv().bits(),
            bus_off_status: psr.bo().bit_is_set(),
            error_warning: psr.ew().bit_is_set(),
            error_passive_state: psr.ep().bit_is_set(),
            last_error: LastErrorCode::try_from(psr.lec().bits()).unwrap(),
        }
    }

    /// Clear specified interrupt
    #[inline]
    pub fn clear_interrupt(&mut self, interrupt: Interrupt) {
        self.control.clear_interrupt(interrupt)
    }

    /// Clear specified interrupts
    #[inline]
    pub fn clear_interrupts(&mut self, interrupts: Interrupts) {
        self.control.clear_interrupts(interrupts)
    }

    /// Splits this `FdCan` instance into transmitting and receiving halves, by reference.
    #[inline]
    #[allow(clippy::type_complexity)]
    fn split_by_ref_generic(
        &mut self,
    ) -> (
        &mut FdCanControl<I, MODE>,
        &mut Tx<I, MODE>,
        &mut Rx<I, MODE, Fifo0>,
        &mut Rx<I, MODE, Fifo1>,
    ) {
        // Safety: We take `&mut self` and the return value lifetimes are tied to `self`'s lifetime.
        let tx = unsafe { Tx::conjure_by_ref() };
        let rx0 = unsafe { Rx::conjure_by_ref() };
        let rx1 = unsafe { Rx::conjure_by_ref() };
        (&mut self.control, tx, rx0, rx1)
    }

    /// Consumes this `FdCan` instance and splits it into transmitting and receiving halves.
    #[inline]
    #[allow(clippy::type_complexity)]
    fn split_generic(
        self,
    ) -> (
        FdCanControl<I, MODE>,
        Tx<I, MODE>,
        Rx<I, MODE, Fifo0>,
        Rx<I, MODE, Fifo1>,
    ) {
        // Safety: We must be carefull to not let them use each others registers
        unsafe { (self.control, Tx::conjure(), Rx::conjure(), Rx::conjure()) }
    }

    /// Combines an FdCanControl, Tx and the two Rx instances back into an FdCan instance
    #[inline]
    #[allow(clippy::type_complexity)]
    pub fn combine(
        t: (
            FdCanControl<I, MODE>,
            Tx<I, MODE>,
            Rx<I, MODE, Fifo0>,
            Rx<I, MODE, Fifo1>,
        ),
    ) -> Self {
        Self::create_can(t.0.config, t.0.instance)
    }
}

/// Select an FDCAN Clock Source
pub enum FdCanClockSource {
    /// Select HSE as the FDCAN clock source
    HSE = 0b00,
    /// Select PLL "Q" clock as the FDCAN clock source
    PLLQ = 0b01,
    /// Select "P" clock as the FDCAN clock source
    PCLK = 0b10,
    //Reserved = 0b10,
}

impl<I> FdCan<I, PoweredDownMode>
where
    I: Instance,
{
    /// Creates a CAN interface.
    ///
    #[inline]
    pub fn new<TX, RX>(
        can_instance: I,
        _tx: TX,
        _rx: RX,
        prec: rec::Fdcan,
    ) -> Self
    where
        TX: sealed::Tx<I>,
        RX: sealed::Rx<I>,
    {
        // Enable APB1 peripheral clock
        prec.enable();

        // Assert
        //assert_eq!(prec.get_kernel_clk_mux(), Some(rec::FdcanClkSel::PLL1_Q));

        let can = Self::create_can(FdCanConfig::default(), can_instance);
        let reg = can.registers();
        assert!(reg.endn.read().bits() == 0x87654321_u32);
        can
    }

    /// Moves out of PoweredDownMode and into ConfigMode
    #[inline]
    pub fn into_config_mode(mut self) -> FdCan<I, ConfigMode> {
        self.set_power_down_mode(false);
        self.enter_init_mode();

        self.reset_msg_ram();

        let can = self.registers();

        // Framework specific settings are set here. //

        // set TxBuffer to Queue Mode;
        // TODO: don't require this.
        // can.txbc.write(|w| w.tfqm().set_bit());
        //FIXME: stm32g4 has the wrong layout here!
        //We should be able to use the above,
        //But right now, we just set the 24th bit.
        can.txbc.write(|w| unsafe { w.bits(1_u32 << 24) });

        // set standard filters list size to 28
        // set extended filters list size to 8
        // REQUIRED: we use the memory map as if these settings are set
        // instead of re-calculating them.
        can.sidfc
            .modify(|_, w| unsafe { w.lss().bits(STANDARD_FILTER_MAX) });
        can.xidfc
            .modify(|_, w| unsafe { w.lse().bits(EXTENDED_FILTER_MAX) });

        // Set memory map on H7 to match G4
        message_ram::set_message_ram_layout(can);

        for fid in 0..STANDARD_FILTER_MAX {
            self.set_standard_filter(
                (fid as u8).into(),
                StandardFilter::disable(),
            );
        }
        for fid in 0..EXTENDED_FILTER_MAX {
            self.set_extended_filter(fid.into(), ExtendedFilter::disable());
        }

        self.into_can_mode()
    }

    /// Disables the CAN interface and returns back the raw peripheral it was created from.
    #[inline]
    pub fn free(mut self) -> I {
        self.disable_interrupts(Interrupts::all());

        //TODO check this!
        self.enter_init_mode();
        self.set_power_down_mode(true);
        self.control.instance
    }
}

impl<I> FdCan<I, ConfigMode>
where
    I: Instance,
{
    #[inline]
    fn leave_init_mode(&mut self) {
        self.apply_config(self.control.config);

        let can = self.registers();
        can.cccr.modify(|_, w| w.cce().clear_bit());
        can.cccr.modify(|_, w| w.init().clear_bit());
        while can.cccr.read().init().bit_is_set() {}
    }

    /// Moves out of ConfigMode and into InternalLoopbackMode
    #[inline]
    pub fn into_internal_loopback(mut self) -> FdCan<I, InternalLoopbackMode> {
        self.set_loopback_mode(LoopbackMode::Internal);
        self.leave_init_mode();

        self.into_can_mode()
    }

    /// Moves out of ConfigMode and into ExternalLoopbackMode
    #[inline]
    pub fn into_external_loopback(mut self) -> FdCan<I, ExternalLoopbackMode> {
        self.set_loopback_mode(LoopbackMode::External);
        self.leave_init_mode();

        self.into_can_mode()
    }

    /// Moves out of ConfigMode and into RestrictedOperationMode
    #[inline]
    pub fn into_restricted(mut self) -> FdCan<I, RestrictedOperationMode> {
        self.set_restricted_operations(true);
        self.leave_init_mode();

        self.into_can_mode()
    }

    /// Moves out of ConfigMode and into NormalOperationMode
    #[inline]
    pub fn into_normal(mut self) -> FdCan<I, NormalOperationMode> {
        self.set_normal_operations(true);
        self.leave_init_mode();

        self.into_can_mode()
    }

    /// Moves out of ConfigMode and into BusMonitoringMode
    #[inline]
    pub fn into_bus_monitoring(mut self) -> FdCan<I, BusMonitoringMode> {
        self.set_bus_monitoring_mode(true);
        self.leave_init_mode();

        self.into_can_mode()
    }

    /// Moves out of ConfigMode and into Testmode
    #[inline]
    pub fn into_test_mode(mut self) -> FdCan<I, TestMode> {
        self.set_test_mode(true);
        self.leave_init_mode();

        self.into_can_mode()
    }

    /// Moves out of ConfigMode and into PoweredDownmode
    #[inline]
    pub fn into_powered_down(mut self) -> FdCan<I, PoweredDownMode> {
        self.set_power_down_mode(true);
        self.leave_init_mode();

        self.into_can_mode()
    }

    /// Applies the settings of a new FdCanConfig
    /// See `[FdCanConfig]` for more information
    #[inline]
    pub fn apply_config(&mut self, config: FdCanConfig) {
        self.set_data_bit_timing(config.dbtr);
        self.set_nominal_bit_timing(config.nbtr);
        self.set_automatic_retransmit(config.automatic_retransmit);
        self.set_transmit_pause(config.transmit_pause);
        self.set_frame_transmit(config.frame_transmit);
        self.set_interrupt_line_config(config.interrupt_line_config);
        self.set_non_iso_mode(config.non_iso_mode);
        self.set_edge_filtering(config.edge_filtering);
        self.set_protocol_exception_handling(
            config.protocol_exception_handling,
        );
        self.set_global_filter(config.global_filter);
    }

    /// Configures the bit timings.
    ///
    /// You can use <http://www.bittiming.can-wiki.info/> to calculate the `btr` parameter. Enter
    /// parameters as follows:
    ///
    /// - *Clock Rate*: The input clock speed to the CAN peripheral (*not* the CPU clock speed).
    ///   This is the clock rate of the peripheral bus the CAN peripheral is attached to (eg. APB1).
    /// - *Sample Point*: Should normally be left at the default value of 87.5%.
    /// - *SJW*: Should normally be left at the default value of 1.
    ///
    /// Then copy the `CAN_BUS_TIME` register value from the table and pass it as the `btr`
    /// parameter to this method.
    #[inline]
    pub fn set_nominal_bit_timing(&mut self, btr: NominalBitTiming) {
        self.control.config.nbtr = btr;

        let can = self.registers();
        can.nbtp.write(|w| unsafe {
            w.nbrp()
                .bits(btr.nbrp() - 1)
                .ntseg1()
                .bits(btr.ntseg1() - 1)
                .ntseg2()
                .bits(btr.ntseg2() - 1)
                .nsjw()
                .bits(btr.nsjw() - 1)
        });
    }

    /// Configures the data bit timings for the FdCan Variable Bitrates.
    /// This is not used when frame_transmit is set to anything other than AllowFdCanAndBRS.
    #[inline]
    pub fn set_data_bit_timing(&mut self, btr: DataBitTiming) {
        self.control.config.dbtr = btr;

        let can = self.registers();
        can.dbtp.write(|w| unsafe {
            w.dbrp()
                .bits(btr.dbrp() - 1)
                .dtseg1()
                .bits(btr.dtseg1() - 1)
                .dtseg2()
                .bits(btr.dtseg2() - 1)
                .dsjw()
                .bits(btr.dsjw() - 1)
        });
    }

    /// Enables or disables automatic retransmission of messages
    ///
    /// If this is enabled, the CAN peripheral will automatically try to retransmit each frame
    /// util it can be sent. Otherwise, it will try only once to send each frame.
    ///
    /// Automatic retransmission is enabled by default.
    #[inline]
    pub fn set_automatic_retransmit(&mut self, enabled: bool) {
        let can = self.registers();
        can.cccr.modify(|_, w| w.dar().bit(!enabled));
        self.control.config.automatic_retransmit = enabled;
    }

    /// Configures the transmit pause feature
    /// See `[FdCanConfig]` for more information
    #[inline]
    pub fn set_transmit_pause(&mut self, enabled: bool) {
        let can = self.registers();
        can.cccr.modify(|_, w| w.dar().bit(!enabled));
        self.control.config.transmit_pause = enabled;
    }

    /// Configures non-iso mode
    /// See `[FdCanConfig]` for more information
    #[inline]
    pub fn set_non_iso_mode(&mut self, enabled: bool) {
        let can = self.registers();
        can.cccr.modify(|_, w| w.niso().bit(enabled));
        self.control.config.non_iso_mode = enabled;
    }

    /// Configures edge filtering
    /// See `[FdCanConfig]` for more information
    #[inline]
    pub fn set_edge_filtering(&mut self, enabled: bool) {
        let can = self.registers();
        can.cccr.modify(|_, w| w.efbi().bit(enabled));
        self.control.config.edge_filtering = enabled;
    }

    /// Configures frame transmission mode
    /// See `[FdCanConfig]` for more information
    #[inline]
    pub fn set_frame_transmit(&mut self, fts: FrameTransmissionConfig) {
        let (fdoe, brse) = match fts {
            FrameTransmissionConfig::ClassicCanOnly => (false, false),
            FrameTransmissionConfig::AllowFdCan => (true, false),
            FrameTransmissionConfig::AllowFdCanAndBRS => (true, true),
        };

        let can = self.registers();
        can.cccr.modify(|_, w| w.fdoe().bit(fdoe).bse().bit(brse));

        self.control.config.frame_transmit = fts;
    }

    /// Configures the interrupt lines
    /// See `[FdCanConfig]` for more information
    #[inline]
    pub fn set_interrupt_line_config(&mut self, l0int: Interrupts) {
        let can = self.registers();

        can.ils.modify(|_, w| unsafe { w.bits(l0int.bits()) });

        self.control.config.interrupt_line_config = l0int;
    }

    /// Sets the protocol exception handling on/off
    #[inline]
    pub fn set_protocol_exception_handling(&mut self, enabled: bool) {
        let can = self.registers();

        can.cccr.modify(|_, w| w.pxhd().bit(!enabled));

        self.control.config.protocol_exception_handling = enabled;
    }

    /// Configures and resets the timestamp counter
    #[inline]
    pub fn set_timestamp_counter_source(&mut self, select: TimestampSource) {
        let (tcp, tss) = match select {
            TimestampSource::None => (0, 0b00),
            TimestampSource::Prescaler(p) => (p as u8, 0b01),
            TimestampSource::FromTIM3 => (0, 0b10),
        };
        self.registers()
            .tscc
            .write(|w| unsafe { w.tcp().bits(tcp).tss().bits(tss) });

        self.control.config.timestamp_source = select;
    }

    /// Configures the global filter settings
    #[inline]
    pub fn set_global_filter(&mut self, filter: GlobalFilter) {
        self.registers().gfc.modify(|_, w| {
            unsafe {
                w.anfs()
                    .bits(filter.handle_standard_frames as u8)
                    .anfe()
                    .bits(filter.handle_extended_frames as u8)
            }
            .rrfs()
            .bit(filter.reject_remote_standard_frames)
            .rrfe()
            .bit(filter.reject_remote_extended_frames)
        });
    }

    /// Returns the current FdCan timestamp counter
    #[inline]
    pub fn timestamp(&self) -> u16 {
        self.control.timestamp()
    }
}

impl<I> FdCan<I, InternalLoopbackMode>
where
    I: Instance,
{
    /// Returns out of InternalLoopbackMode and back into ConfigMode
    #[inline]
    pub fn into_config_mode(mut self) -> FdCan<I, ConfigMode> {
        self.set_loopback_mode(LoopbackMode::None);
        self.enter_init_mode();

        self.into_can_mode()
    }
}

impl<I> FdCan<I, ExternalLoopbackMode>
where
    I: Instance,
{
    /// Returns out of ExternalLoopbackMode and back into ConfigMode
    #[inline]
    pub fn into_config_mode(mut self) -> FdCan<I, ConfigMode> {
        self.set_loopback_mode(LoopbackMode::None);
        self.enter_init_mode();

        self.into_can_mode()
    }
}

impl<I> FdCan<I, NormalOperationMode>
where
    I: Instance,
{
    /// Returns out of NormalOperationMode and back into ConfigMode
    #[inline]
    pub fn into_config_mode(mut self) -> FdCan<I, ConfigMode> {
        self.set_normal_operations(false);
        self.enter_init_mode();

        self.into_can_mode()
    }
}

impl<I> FdCan<I, RestrictedOperationMode>
where
    I: Instance,
{
    /// Returns out of RestrictedOperationMode and back into ConfigMode
    #[inline]
    pub fn into_config_mode(mut self) -> FdCan<I, ConfigMode> {
        self.set_restricted_operations(false);
        self.enter_init_mode();

        self.into_can_mode()
    }
}

impl<I> FdCan<I, BusMonitoringMode>
where
    I: Instance,
{
    /// Returns out of BusMonitoringMode and back into ConfigMode
    #[inline]
    pub fn into_config_mode(mut self) -> FdCan<I, ConfigMode> {
        self.set_bus_monitoring_mode(false);
        self.enter_init_mode();

        self.into_can_mode()
    }
}

/// states of the test.tx register
pub enum TestTransmitPinState {
    /// CAN core has control (default)
    CoreHasControl = 0b00,
    /// Sample point can be monitored
    ShowSamplePoint = 0b01,
    /// Set to Dominant (0) Level
    SetDominant = 0b10,
    /// Set to Recessive (1) Level
    SetRecessive = 0b11,
}

impl<I> FdCan<I, TestMode>
where
    I: Instance,
{
    /// Returns out of TestMode and back into ConfigMode
    #[inline]
    pub fn into_config_mode(mut self) -> FdCan<I, ConfigMode> {
        self.set_test_mode(false);
        self.enter_init_mode();

        self.into_can_mode()
    }

    /// Gets the state of the receive pin to either Dominant (false), or Recessive (true)
    pub fn get_receive_pin(&mut self) -> bool {
        let can = self.registers();

        can.test.read().rx().bit_is_set()
    }

    /// Sets the state of the transmit pin according to TestTransmitPinState
    pub fn set_transmit_pin(&mut self, state: TestTransmitPinState) {
        let can = self.registers();

        //SAFE: state has all possible values, and this can only occur in TestMode
        can.test.modify(|_, w| unsafe { w.tx().bits(state as u8) });
    }
}

impl<I, M> FdCan<I, M>
where
    I: Instance,
    M: Transmit + Receive,
{
    /// Splits this `FdCan` instance into transmitting and receiving halves, by reference.
    #[inline]
    #[allow(clippy::type_complexity)]
    pub fn split_by_ref(
        &mut self,
    ) -> (
        &mut FdCanControl<I, M>,
        &mut Tx<I, M>,
        &mut Rx<I, M, Fifo0>,
        &mut Rx<I, M, Fifo1>,
    ) {
        self.split_by_ref_generic()
    }

    /// Consumes this `FdCan` instance and splits it into transmitting and receiving halves.
    #[allow(clippy::type_complexity)]
    pub fn split(
        self,
    ) -> (
        FdCanControl<I, M>,
        Tx<I, M>,
        Rx<I, M, Fifo0>,
        Rx<I, M, Fifo1>,
    ) {
        self.split_generic()
    }
}

impl<I, M> FdCan<I, M>
where
    I: Instance,
    M: Transmit,
{
    /// Puts a CAN frame in a free transmit mailbox for transmission on the bus.
    ///
    /// Frames are transmitted to the bus based on their priority (identifier).
    /// Transmit order is preserved for frames with identical identifiers.
    /// If all transmit mailboxes are full, this overwrites the mailbox with
    /// the lowest priority.
    #[inline]
    pub fn transmit<WTX>(
        &mut self,
        frame: TxFrameHeader,
        write: &mut WTX,
    ) -> nb::Result<Option<()>, Infallible>
    where
        WTX: FnMut(&mut [u32]),
    {
        // Safety: We have a `&mut self` and have unique access to the peripheral.
        unsafe { Tx::<I, M>::conjure().transmit(frame, write) }
    }

    /// Puts a CAN frame in a free transmit mailbox for transmission on the bus.
    ///
    /// Frames are transmitted to the bus based on their priority (identifier).
    /// Transmit order is preserved for frames with identical identifiers.
    /// If all transmit mailboxes are full, `pending` is called with the mailbox,
    /// header and data of the to-be-replaced frame.
    pub fn transmit_preserve<PTX, WTX, P>(
        &mut self,
        frame: TxFrameHeader,
        write: &mut WTX,
        pending: &mut PTX,
    ) -> nb::Result<Option<P>, Infallible>
    where
        PTX: FnMut(Mailbox, TxFrameHeader, &[u32]) -> P,
        WTX: FnMut(&mut [u32]),
    {
        // Safety: We have a `&mut self` and have unique access to the peripheral.
        unsafe {
            Tx::<I, M>::conjure().transmit_preserve(frame, write, pending)
        }
    }

    /// Returns `true` if no frame is pending for transmission.
    #[inline]
    pub fn is_transmitter_idle(&self) -> bool {
        // Safety: Read-only operation.
        unsafe { Tx::<I, M>::conjure().is_idle() }
    }

    /// Attempts to abort the sending of a frame that is pending in a mailbox.
    ///
    /// If there is no frame in the provided mailbox, or its transmission succeeds before it can be
    /// aborted, this function has no effect and returns `false`.
    ///
    /// If there is a frame in the provided mailbox, and it is canceled successfully, this function
    /// returns `true`.
    #[inline]
    pub fn abort(&mut self, mailbox: Mailbox) -> bool {
        // Safety: We have a `&mut self` and have unique access to the peripheral.
        unsafe { Tx::<I, M>::conjure().abort(mailbox) }
    }
}

impl<I, M> FdCan<I, M>
where
    I: Instance,
    M: Receive,
{
    /// Returns a received frame from FIFO_0 if available.
    #[inline]
    pub fn receive0<RECV, R>(
        &mut self,
        receive: &mut RECV,
    ) -> nb::Result<ReceiveOverrun<R>, Infallible>
    where
        RECV: FnMut(RxFrameInfo, &[u32]) -> R,
    {
        // Safety: We have a `&mut self` and have unique access to the peripheral.
        unsafe { Rx::<I, M, Fifo0>::conjure().receive(receive) }
    }

    /// Returns a received frame from FIFO_1 if available.
    #[inline]
    pub fn receive1<RECV, R>(
        &mut self,
        receive: &mut RECV,
    ) -> nb::Result<ReceiveOverrun<R>, Infallible>
    where
        RECV: FnMut(RxFrameInfo, &[u32]) -> R,
    {
        // Safety: We have a `&mut self` and have unique access to the peripheral.
        unsafe { Rx::<I, M, Fifo1>::conjure().receive(receive) }
    }
}

/// FdCanControl Struct
/// Used to house some information during an FdCan split.
/// and can be used for some generic information retrieval during operation.
pub struct FdCanControl<I, MODE>
where
    I: Instance,
{
    config: FdCanConfig,
    instance: I,
    _mode: PhantomData<MODE>,
}
impl<I, MODE> FdCanControl<I, MODE>
where
    I: Instance,
{
    #[inline]
    fn registers(&self) -> &RegisterBlock {
        unsafe { &*I::REGISTERS }
    }

    /// Returns the current error counters
    #[inline]
    pub fn error_counters(&self) -> ErrorCounters {
        let can = self.registers();
        let cel: u8 = can.ecr.read().cel().bits();
        let rp: bool = can.ecr.read().rp().bits();
        let rec: u8 = can.ecr.read().rec().bits();
        let tec: u8 = can.ecr.read().tec().bits();

        ErrorCounters {
            can_errors: cel,
            transmit_err: tec,
            receive_err: match rp {
                false => ReceiveErrorOverflow::Normal(rec),
                true => ReceiveErrorOverflow::Overflow(rec),
            },
        }
    }

    /// Returns the current FdCan Timestamp counter
    #[inline]
    pub fn timestamp(&self) -> u16 {
        self.registers().tscv.read().tsc().bits()
    }

    /// Clear specified interrupt
    #[inline]
    pub fn clear_interrupt(&mut self, interrupt: Interrupt) {
        let can = self.registers();
        can.ir.write(|w| unsafe { w.bits(interrupt as u32) });
    }

    /// Clear specified interrupts
    #[inline]
    pub fn clear_interrupts(&mut self, interrupts: Interrupts) {
        let can = self.registers();
        can.ir.write(|w| unsafe { w.bits(interrupts.bits()) });
    }
}

/// Interface to the CAN transmitter part.
pub struct Tx<I, MODE> {
    _can: PhantomData<I>,
    _mode: PhantomData<MODE>,
}

impl<I, MODE> Tx<I, MODE>
where
    I: Instance,
{
    #[inline]
    unsafe fn conjure() -> Self {
        Self {
            _can: PhantomData,
            _mode: PhantomData,
        }
    }

    /// Creates a `&mut Self` out of thin air.
    ///
    /// This is only safe if it is the only way to access a `Tx<I>`.
    #[inline]
    unsafe fn conjure_by_ref<'a>() -> &'a mut Self {
        // Cause out of bounds access when `Self` is not zero-sized.
        #[allow(clippy::unnecessary_operation)]
        [()][core::mem::size_of::<Self>()];

        // Any aligned pointer is valid for ZSTs.
        &mut *NonNull::dangling().as_ptr()
    }

    #[inline]
    fn registers(&self) -> &RegisterBlock {
        unsafe { &*I::REGISTERS }
    }

    #[inline]
    fn tx_msg_ram(&self) -> &message_ram::Transmit {
        unsafe { &(*I::MSG_RAM).transmit }
    }

    #[inline]
    fn tx_msg_ram_mut(&mut self) -> &mut message_ram::Transmit {
        unsafe { &mut (*I::MSG_RAM).transmit }
    }

    /// Puts a CAN frame in a transmit mailbox for transmission on the bus.
    ///
    /// Frames are transmitted to the bus based on their priority (identifier). Transmit order is
    /// preserved for frames with identical identifiers.
    ///
    /// If all transmit mailboxes are full, a higher priority frame can replace a lower-priority
    /// frame, which is returned via the closure 'pending'. If 'pending' is called; it's return value
    /// is returned via Option<P>, if it is not, None is returned.
    /// If there are only higher priority frames in the queue, this returns Err::WouldBlock
    pub fn transmit<WTX>(
        &mut self,
        frame: TxFrameHeader,
        write: &mut WTX,
    ) -> nb::Result<Option<()>, Infallible>
    where
        WTX: FnMut(&mut [u32]),
    {
        self.transmit_preserve(frame, write, &mut |_, _, _| ())
    }

    /// As Transmit, but if there is a pending frame, `pending` will be called so that the frame can
    /// be preserved.
    pub fn transmit_preserve<PTX, WTX, P>(
        &mut self,
        frame: TxFrameHeader,
        write: &mut WTX,
        pending: &mut PTX,
    ) -> nb::Result<Option<P>, Infallible>
    where
        PTX: FnMut(Mailbox, TxFrameHeader, &[u32]) -> P,
        WTX: FnMut(&mut [u32]),
    {
        let can = self.registers();
        let queue_is_full = self.tx_queue_is_full();

        let id = frame.into();

        // If the queue is full,
        // Discard the first slot with a lower priority message
        let (idx, pending_frame) = if queue_is_full {
            if self.is_available(Mailbox::_0, id) {
                (
                    Mailbox::_0,
                    self.abort_pending_mailbox(Mailbox::_0, pending),
                )
            } else if self.is_available(Mailbox::_1, id) {
                (
                    Mailbox::_1,
                    self.abort_pending_mailbox(Mailbox::_1, pending),
                )
            } else if self.is_available(Mailbox::_2, id) {
                (
                    Mailbox::_2,
                    self.abort_pending_mailbox(Mailbox::_2, pending),
                )
            } else {
                // For now we bail when there is no lower priority slot available
                // Can this lead to priority inversion?
                return Err(nb::Error::WouldBlock);
            }
        } else {
            // Read the Write Pointer
            let idx = can.txfqs.read().tfqpi().bits();

            (Mailbox::new(idx), None)
        };

        self.write_mailbox(idx, frame, write);

        Ok(pending_frame)
    }

    /// Returns if the tx queue is able to accept new messages without having to cancel an existing one
    #[inline]
    pub fn tx_queue_is_full(&self) -> bool {
        self.registers().txfqs.read().tfqf().bit()
    }

    /// Returns `Ok` when the mailbox is free or if it contains pending frame with a
    /// lower priority (higher ID) than the identifier `id`.
    #[inline]
    fn is_available(&self, idx: Mailbox, id: IdReg) -> bool {
        if self.has_pending_frame(idx) {
            //read back header section
            let header: TxFrameHeader =
                (&self.tx_msg_ram().tbsa[idx as usize].header).into();
            let old_id: IdReg = header.into();

            id > old_id
        } else {
            true
        }
    }

    #[inline]
    fn write_mailbox<TX, R>(
        &mut self,
        idx: Mailbox,
        tx_header: TxFrameHeader,
        transmit: TX,
    ) -> R
    where
        TX: FnOnce(&mut [u32]) -> R,
    {
        let tx_ram = self.tx_msg_ram_mut();

        // Clear mail slot; mainly for debugging purposes.
        tx_ram.tbsa[idx as usize].reset();

        // Calculate length of data in words
        let data_len = ((tx_header.len as usize) + 3) / 4;

        //set header section
        tx_ram.tbsa[idx as usize].header.merge(tx_header);

        //set data
        let result = transmit(&mut tx_ram.tbsa[idx as usize].data[0..data_len]);

        // Set <idx as Mailbox> as ready to transmit
        self.registers().txbar.modify(|r, w| unsafe {
            w.ar().bits(r.ar().bits() | 1 << (idx as u32))
        });

        result
    }

    #[inline]
    fn abort_pending_mailbox<PTX, R>(
        &mut self,
        idx: Mailbox,
        pending: PTX,
    ) -> Option<R>
    where
        PTX: FnOnce(Mailbox, TxFrameHeader, &[u32]) -> R,
    {
        if self.abort(idx) {
            let tx_ram = self.tx_msg_ram();

            //read back header section
            let header = (&tx_ram.tbsa[idx as usize].header).into();
            Some(pending(idx, header, &tx_ram.tbsa[idx as usize].data))
        } else {
            // Abort request failed because the frame was already sent (or being sent) on
            // the bus. All mailboxes are now free. This can happen for small prescaler
            // values (e.g. 1MBit/s bit timing with a source clock of 8MHz) or when an ISR
            // has preempted the execution.
            None
        }
    }

    /// Attempts to abort the sending of a frame that is pending in a mailbox.
    ///
    /// If there is no frame in the provided mailbox, or its transmission succeeds before it can be
    /// aborted, this function has no effect and returns `false`.
    ///
    /// If there is a frame in the provided mailbox, and it is canceled successfully, this function
    /// returns `true`.
    #[inline]
    fn abort(&mut self, idx: Mailbox) -> bool {
        let can = self.registers();

        // Check if there is a request pending to abort
        if self.has_pending_frame(idx) {
            let idx: u8 = idx.into();
            let idx: u32 = idx as u32;

            // Abort Request
            can.txbcr.write(|w| unsafe { w.cr().bits(idx) });

            // Wait for the abort request to be finished.
            loop {
                if can.txbcf.read().cf().bits() & idx != 0 {
                    // Return false when a transmission has occured
                    break can.txbto.read().to().bits() & idx == 0;
                }
            }
        } else {
            false
        }
    }

    #[inline]
    fn has_pending_frame(&self, idx: Mailbox) -> bool {
        let can = self.registers();
        let idx: u8 = idx.into();
        let idx: u32 = idx as u32;

        can.txbrp.read().trp().bits() & idx != 0
    }

    /// Returns `true` if no frame is pending for transmission.
    #[inline]
    pub fn is_idle(&self) -> bool {
        let can = self.registers();
        can.txbrp.read().trp().bits() == 0x0
    }

    /// Clears the transmission complete flag.
    #[inline]
    pub fn clear_transmission_completed_flag(&mut self) {
        let can = self.registers();
        can.ir.write(|w| w.tc().set_bit());
    }

    /// Clears the transmission cancelled flag.
    #[inline]
    pub fn clear_transmission_cancelled_flag(&mut self) {
        let can = self.registers();
        can.ir.write(|w| w.tcf().set_bit());
    }
}

#[doc(hidden)]
pub trait FifoNr: sealed::Sealed {
    const NR: usize;
}
#[doc(hidden)]
pub struct Fifo0;
impl sealed::Sealed for Fifo0 {}
impl FifoNr for Fifo0 {
    const NR: usize = 0;
}
#[doc(hidden)]
pub struct Fifo1;
impl sealed::Sealed for Fifo1 {}
impl FifoNr for Fifo1 {
    const NR: usize = 1;
}

/// Notes whether an overrun has occurred.
/// Since both arms contain T, this can be 'unwrap'ed without causing a panic.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "unstable-defmt", derive(defmt::Format))]
pub enum ReceiveOverrun<T> {
    /// No overrun has occured
    NoOverrun(T),
    /// an overrun has occurered
    Overrun(T),
}
impl<T> ReceiveOverrun<T> {
    /// unwraps itself and returns T
    /// in contradiction to Option::unwrap, this does not panic
    /// since both elements contain T.
    #[inline]
    pub fn unwrap(self) -> T {
        match self {
            ReceiveOverrun::NoOverrun(t) | ReceiveOverrun::Overrun(t) => t,
        }
    }
}

/// Interface to the CAN receiver part.
pub struct Rx<I, MODE, FIFONR>
where
    FIFONR: FifoNr,
{
    _can: PhantomData<I>,
    _mode: PhantomData<MODE>,
    _nr: PhantomData<FIFONR>,
}

impl<I, MODE, FIFONR> Rx<I, MODE, FIFONR>
where
    FIFONR: FifoNr,
    I: Instance,
{
    #[inline]
    unsafe fn conjure() -> Self {
        Self {
            _can: PhantomData,
            _mode: PhantomData,
            _nr: PhantomData,
        }
    }

    /// Creates a `&mut Self` out of thin air.
    ///
    /// This is only safe if it is the only way to access an `Rx<I>`.
    #[inline]
    unsafe fn conjure_by_ref<'a>() -> &'a mut Self {
        // Cause out of bounds access when `Self` is not zero-sized.
        #[allow(clippy::unnecessary_operation)]
        [()][core::mem::size_of::<Self>()];

        // Any aligned pointer is valid for ZSTs.
        &mut *NonNull::dangling().as_ptr()
    }

    /// Returns a received frame if available.
    ///
    /// Returns `Err` when a frame was lost due to buffer overrun.
    pub fn receive<RECV, R>(
        &mut self,
        receive: &mut RECV,
    ) -> nb::Result<ReceiveOverrun<R>, Infallible>
    where
        RECV: FnMut(RxFrameInfo, &[u32]) -> R,
    {
        if !self.rx_fifo_is_empty() {
            let mbox = self.get_rx_mailbox();
            let idx: usize = mbox.into();
            let mailbox: &RxFifoElement = &self.rx_msg_ram().fxsa[idx];

            let header: RxFrameInfo = (&mailbox.header).into();
            let word_len = (header.len + 3) / 4;
            let result =
                Ok(receive(header, &mailbox.data[0..word_len as usize]));
            self.release_mailbox(mbox);

            if self.has_overrun() {
                result.map(ReceiveOverrun::Overrun)
            } else {
                result.map(ReceiveOverrun::NoOverrun)
            }
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    #[inline]
    fn registers(&self) -> &RegisterBlock {
        unsafe { &*I::REGISTERS }
    }

    #[inline]
    fn rx_msg_ram(&self) -> &message_ram::Receive {
        unsafe { &(&(*I::MSG_RAM).receive)[FIFONR::NR] }
    }

    #[inline]
    fn has_overrun(&self) -> bool {
        let can = self.registers();
        match FIFONR::NR {
            0 => can.rxf0s.read().rf0l().bit(),
            1 => can.rxf1s.read().rf1l().bit(),
            _ => unreachable!(),
        }
    }

    /// Returns if the fifo contains any new messages.
    #[inline]
    pub fn rx_fifo_is_empty(&self) -> bool {
        let can = self.registers();
        match FIFONR::NR {
            0 => can.rxf0s.read().f0fl().bits() == 0,
            1 => can.rxf1s.read().f1fl().bits() == 0,
            _ => unreachable!(),
        }
    }

    #[inline]
    fn release_mailbox(&mut self, idx: Mailbox) {
        unsafe {
            (*I::MSG_RAM).receive[FIFONR::NR].fxsa[idx as u8 as usize].reset();
        }

        let can = self.registers();
        match FIFONR::NR {
            0 => can.rxf0a.write(|w| unsafe { w.f0ai().bits(idx.into()) }),
            1 => can.rxf1a.write(|w| unsafe { w.f1ai().bits(idx.into()) }),
            _ => unreachable!(),
        }
    }

    #[inline]
    fn get_rx_mailbox(&self) -> Mailbox {
        let can = self.registers();
        let idx = match FIFONR::NR {
            0 => can.rxf0s.read().f0gi().bits(),
            1 => can.rxf1s.read().f1gi().bits(),
            _ => unreachable!(),
        };
        Mailbox::new(idx)
    }
}

/// The three mailboxes.
/// These are used for the transmit queue
/// and the two Receive FIFOs
#[derive(Debug, Copy, Clone, Ord, PartialOrd, Eq, PartialEq)]
#[cfg_attr(feature = "unstable-defmt", derive(defmt::Format))]
pub enum Mailbox {
    /// Transmit mailbox 0
    _0 = 0,
    /// Transmit mailbox 1
    _1 = 1,
    /// Transmit mailbox 2
    _2 = 2,
}
impl Mailbox {
    #[inline]
    fn new(idx: u8) -> Self {
        match idx & 0b11 {
            0 => Mailbox::_0,
            1 => Mailbox::_1,
            2 => Mailbox::_2,
            _ => unreachable!(),
        }
    }
}
impl From<Mailbox> for u8 {
    #[inline]
    fn from(m: Mailbox) -> Self {
        m as u8
    }
}
impl From<Mailbox> for usize {
    #[inline]
    fn from(m: Mailbox) -> Self {
        m as u8 as usize
    }
}

mod impls {
    use super::sealed;
    use crate::gpio::gpioa::{PA11, PA12};
    use crate::gpio::gpiob::{PB12, PB13, PB5, PB6, PB8, PB9};
    use crate::gpio::gpiod::{PD0, PD1};
    use crate::gpio::gpioh::{PH13, PH14};
    use crate::gpio::{Alternate, AF9};

    /// Implements sealed::{Tx,Rx} for pins associated with a CAN peripheral
    macro_rules! pins {
        ($PER:ident =>
            (TX: [ $($( #[ $pmetatx:meta ] )* $tx:ty),+ $(,)? ],
             RX: [ $($( #[ $pmetarx:meta ] )* $rx:ty),+ $(,)? ])) => {
            $(
                $( #[ $pmetatx ] )*
                impl sealed::Tx<crate::stm32::$PER> for $tx {}
            )+
            $(
                $( #[ $pmetarx ] )*
                impl sealed::Rx<crate::stm32::$PER> for $rx {}
            )+
        };
    }

    pins! {
        FDCAN1 => (
            TX: [
                PA12<Alternate<AF9>>,
                PB9<Alternate<AF9>>,
                PD1<Alternate<AF9>>,
                PH13<Alternate<AF9>>
            ],
            RX: [
                PA11<Alternate<AF9>>,
                PB8<Alternate<AF9>>,
                PD0<Alternate<AF9>>,
                PH14<Alternate<AF9>>
            ]
        )
    }
    pins! {
        FDCAN2 => (
            TX: [
                PB6<Alternate<AF9>>,
                PB13<Alternate<AF9>>
            ],
            RX: [
                PB5<Alternate<AF9>>,
                PB12<Alternate<AF9>>
            ]
        )
    }

    mod fdcan1 {
        use crate::fdcan::message_ram;
        use crate::stm32::FDCAN1;
        use crate::{fdcan, stm32};

        unsafe impl fdcan::Instance for FDCAN1 {
            const REGISTERS: *mut stm32::fdcan1::RegisterBlock =
                FDCAN1::ptr() as *mut _;
        }
        unsafe impl message_ram::MsgRamExt for FDCAN1 {
            const MSG_RAM: *mut message_ram::RegisterBlock =
                (0x4000_ac00 as *mut _);
        }
    }

    mod fdcan2 {
        use crate::fdcan::message_ram;
        use crate::stm32::FDCAN2;
        use crate::{fdcan, stm32};

        unsafe impl fdcan::Instance for FDCAN2 {
            const REGISTERS: *mut stm32::fdcan1::RegisterBlock =
                FDCAN2::ptr() as *mut _;
        }
        unsafe impl message_ram::MsgRamExt for FDCAN2 {
            const MSG_RAM: *mut message_ram::RegisterBlock =
                (0x4000_b800 as *mut _); // FDCAN1 + 3kB
        }
    }
}
