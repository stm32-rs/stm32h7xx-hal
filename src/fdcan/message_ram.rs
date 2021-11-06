pub(crate) mod common;
pub(crate) mod enums;
pub(crate) mod generic;

/// Number of Receive Fifos configured by this module
pub const RX_FIFOS_MAX: u8 = 2;
/// Number of Receive Messages per RxFifo configured by this module
pub const RX_FIFO_MAX: u8 = 3;
/// Number of Transmit Messages configured by this module
pub const TX_FIFO_MAX: u8 = 3;
/// Number of Transmit Events configured by this module
pub const TX_EVENT_MAX: u8 = 3;
/// Number of Standard Filters configured by this module
pub const STANDARD_FILTER_MAX: u8 = 28;
/// Number of Extended Filters configured by this module
pub const EXTENDED_FILTER_MAX: u8 = 8;

/// Configure Message RAM layout on H7 to match the fixed sized used on G4
///
/// These are protected bits, write access is only possible when bit CCE and bit
/// INIT for FDCAN_CCCR are set to 1
pub fn set_message_ram_layout(
    can: &crate::stm32::fdcan1::RegisterBlock,
) -> u16 {
    let mut word_adr = 0u16;

    // 11-bit filter
    can.sidfc.modify(|_, w| unsafe { w.flssa().bits(word_adr) });
    word_adr += STANDARD_FILTER_MAX as u16;
    // 29-bit filter
    can.xidfc.modify(|_, w| unsafe { w.flesa().bits(word_adr) });
    word_adr += 2 * EXTENDED_FILTER_MAX as u16;
    // Rx FIFO 0
    can.rxf0c.modify(|_, w| unsafe {
        w.f0sa().bits(word_adr).f0s().bits(RX_FIFO_MAX)
    });
    word_adr += 18 * RX_FIFO_MAX as u16;
    // Rx FIFO 1
    can.rxf1c.modify(|_, w| unsafe {
        w.f1sa().bits(word_adr).f1s().bits(RX_FIFO_MAX)
    });
    word_adr += 18 * RX_FIFO_MAX as u16;
    // Rx buffer - see below
    // Tx event FIFO
    can.txefc.modify(|_, w| unsafe {
        w.efsa().bits(word_adr).efs().bits(TX_EVENT_MAX)
    });
    word_adr += 2 * TX_EVENT_MAX as u16;
    // Tx buffers
    can.txbc.modify(|_, w| unsafe {
        w.tbsa().bits(word_adr).tfqs().bits(TX_FIFO_MAX)
    });
    word_adr += 18 * TX_FIFO_MAX as u16;

    // Rx Buffer - not used
    can.rxbc.modify(|_, w| unsafe { w.rbsa().bits(word_adr) });

    // TX event FIFO?
    // Trigger memory?

    // Set the element sizes to 16 bytes
    can.rxesc.modify(|_, w| unsafe {
        w.rbds().bits(0b111).f1ds().bits(0b111).f0ds().bits(0b111)
    });
    can.txesc.modify(|_, w| unsafe { w.tbds().bits(0b111) });

    word_adr
}

/// MessageRam Overlay
#[repr(C)]
pub struct RegisterBlock {
    pub(crate) filters: Filters,
    pub(crate) receive: [Receive; RX_FIFOS_MAX as usize],
    pub(crate) transmit: Transmit,
}
impl RegisterBlock {
    pub fn reset(&mut self) {
        self.filters.reset();
        self.receive[0].reset();
        self.receive[1].reset();
        self.transmit.reset();
    }
}

#[repr(C)]
pub(crate) struct Filters {
    pub(crate) flssa: [StandardFilter; STANDARD_FILTER_MAX as usize],
    pub(crate) flesa: [ExtendedFilter; EXTENDED_FILTER_MAX as usize],
}
impl Filters {
    pub fn reset(&mut self) {
        for sf in &mut self.flssa {
            sf.reset();
        }
        for ef in &mut self.flesa {
            ef.reset();
        }
    }
}

#[repr(C)]
pub(crate) struct Receive {
    pub(crate) fxsa: [RxFifoElement; RX_FIFO_MAX as usize],
}
impl Receive {
    pub fn reset(&mut self) {
        for fe in &mut self.fxsa {
            fe.reset();
        }
    }
}

#[repr(C)]
pub(crate) struct Transmit {
    pub(crate) efsa: [TxEventElement; TX_EVENT_MAX as usize],
    pub(crate) tbsa: [TxBufferElement; TX_FIFO_MAX as usize],
}
impl Transmit {
    pub fn reset(&mut self) {
        for ee in &mut self.efsa {
            ee.reset();
        }
        for be in &mut self.tbsa {
            be.reset();
        }
    }
}

pub(crate) mod standard_filter;
pub(crate) type StandardFilterType = u32;
pub(crate) type StandardFilter =
    generic::Reg<StandardFilterType, _StandardFilter>;
pub(crate) struct _StandardFilter;
impl generic::Readable for StandardFilter {}
impl generic::Writable for StandardFilter {}

pub(crate) mod extended_filter;
pub(crate) type ExtendedFilterType = [u32; 2];
pub(crate) type ExtendedFilter =
    generic::Reg<ExtendedFilterType, _ExtendedFilter>;
pub(crate) struct _ExtendedFilter;
impl generic::Readable for ExtendedFilter {}
impl generic::Writable for ExtendedFilter {}

pub(crate) mod txevent_element;
pub(crate) type TxEventElementType = [u32; 2];
pub(crate) type TxEventElement =
    generic::Reg<TxEventElementType, _TxEventElement>;
pub(crate) struct _TxEventElement;
impl generic::Readable for TxEventElement {}
impl generic::Writable for TxEventElement {}

pub(crate) mod rxfifo_element;
#[repr(C)]
pub(crate) struct RxFifoElement {
    pub(crate) header: RxFifoElementHeader,
    pub(crate) data: [u32; 16], // TODO: Does this need to be volatile?
}
impl RxFifoElement {
    pub(crate) fn reset(&mut self) {
        self.header.reset();
        self.data = [0; 16];
    }
}
pub(crate) type RxFifoElementHeaderType = [u32; 2];
pub(crate) type RxFifoElementHeader =
    generic::Reg<RxFifoElementHeaderType, _RxFifoElement>;
pub(crate) struct _RxFifoElement;
impl generic::Readable for RxFifoElementHeader {}
impl generic::Writable for RxFifoElementHeader {}

pub(crate) mod txbuffer_element;
#[repr(C)]
pub(crate) struct TxBufferElement {
    pub(crate) header: TxBufferElementHeader,
    pub(crate) data: [u32; 16], // TODO: Does this need to be volatile?
}
impl TxBufferElement {
    pub(crate) fn reset(&mut self) {
        self.header.reset();
        self.data = [0; 16];
    }
}
pub(crate) type TxBufferElementHeader =
    generic::Reg<TxBufferElementHeaderType, _TxBufferElement>;
pub(crate) type TxBufferElementHeaderType = [u32; 2];
pub(crate) struct _TxBufferElement;
impl generic::Readable for TxBufferElementHeader {}
impl generic::Writable for TxBufferElementHeader {}

/// Accessor for the FdCan Message Ram Area
pub unsafe trait MsgRamExt {
    const MSG_RAM: *mut RegisterBlock;
    fn msg_ram(&self) -> &RegisterBlock {
        unsafe { &*Self::MSG_RAM }
    }
    fn msg_ram_mut(&mut self) -> &mut RegisterBlock {
        unsafe { &mut *Self::MSG_RAM }
    }
}

// Ensure the RegisterBlock is the same size as on pg 1957 of RM0440.
static_assertions::assert_eq_size!(Filters, [u32; 28 + 16]);
static_assertions::assert_eq_size!(Receive, [u32; 54]);
static_assertions::assert_eq_size!(Transmit, [u32; 6 + 54]);
static_assertions::assert_eq_size!(
    RegisterBlock,
    [u32; 28 /*Standard Filters*/ +16 /*Extended Filters*/ +54 /*RxFifo0*/ +54 /*RxFifo1*/ +6 /*TxEvent*/ +54 /*TxFifo */]
);
