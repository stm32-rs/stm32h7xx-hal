use super::Serial;
use crate::dma::mux::request_ids::{
    RequestIdSome, Uart4RxDma, Uart4TxDma, Uart5RxDma, Uart5TxDma, Uart7RxDma,
    Uart7TxDma, Uart8RxDma, Uart8TxDma, Usart1RxDma, Usart1TxDma, Usart2RxDma,
    Usart2TxDma, Usart3RxDma, Usart3TxDma, Usart6RxDma, Usart6TxDma,
};
use crate::dma::mux::{EgED as IEgED, SyncED as ISyncED};
use crate::dma::safe_transfer::{
    FixedBuffer, FixedBufferR, MemoryBufferStatic, Ongoing as TransferOngoing,
    Payload, PeripheralBuffer, Start as TransferStart,
};
use crate::dma::stream::{
    Disabled as StreamDisabled, IsrCleared as StreamIsrCleared, StreamIsr,
};
use crate::dma::{Channel, ChannelId, DmaMux, SafeTransfer};
use crate::stm32::{UART4, UART5, UART7, UART8};
use crate::stm32::{USART1, USART2, USART3, USART6};
use core::ops::Deref;
use stm32h7::stm32h743::usart1::RegisterBlock;

pub trait SerialRequest: RequestIdSome {}

impl SerialRequest for Usart1RxDma {}
impl SerialRequest for Usart1TxDma {}
impl SerialRequest for Usart2RxDma {}
impl SerialRequest for Usart2TxDma {}
impl SerialRequest for Usart3RxDma {}
impl SerialRequest for Usart3TxDma {}
impl SerialRequest for Usart6RxDma {}
impl SerialRequest for Usart6TxDma {}

impl SerialRequest for Uart4RxDma {}
impl SerialRequest for Uart4TxDma {}
impl SerialRequest for Uart5RxDma {}
impl SerialRequest for Uart5TxDma {}
impl SerialRequest for Uart7RxDma {}
impl SerialRequest for Uart7TxDma {}
impl SerialRequest for Uart8RxDma {}
impl SerialRequest for Uart8TxDma {}

pub trait SerialChannelDma {
    type Rx: SerialRequest;
    type Tx: SerialRequest;
}

impl SerialChannelDma for USART1 {
    type Rx = Usart1RxDma;
    type Tx = Usart1TxDma;
}

impl SerialChannelDma for USART2 {
    type Rx = Usart2RxDma;
    type Tx = Usart2TxDma;
}

impl SerialChannelDma for USART3 {
    type Rx = Usart3RxDma;
    type Tx = Usart3TxDma;
}

impl SerialChannelDma for USART6 {
    type Rx = Usart6RxDma;
    type Tx = Usart6TxDma;
}

impl SerialChannelDma for UART4 {
    type Rx = Uart4RxDma;
    type Tx = Uart4TxDma;
}

impl SerialChannelDma for UART5 {
    type Rx = Uart5RxDma;
    type Tx = Uart5TxDma;
}

impl SerialChannelDma for UART7 {
    type Rx = Uart7RxDma;
    type Tx = Uart7TxDma;
}

impl SerialChannelDma for UART8 {
    type Rx = Uart8RxDma;
    type Tx = Uart8TxDma;
}

pub trait IState {}

pub struct Start<CXX, ReqId, SyncED, EgED>
where
    CXX: ChannelId,
    ReqId: SerialRequest,
    SyncED: ISyncED,
    EgED: IEgED,
{
    channel:
        Channel<CXX, StreamDisabled, StreamIsrCleared, ReqId, SyncED, EgED>,
}

pub struct Initialized<CXX, ReqId, SyncED, EgED, Peripheral, Memory>
where
    CXX: ChannelId,
    ReqId: SerialRequest,
    SyncED: ISyncED,
    EgED: IEgED,
    Peripheral: Payload,
    Memory: Payload,
{
    channel:
        Channel<CXX, StreamDisabled, StreamIsrCleared, ReqId, SyncED, EgED>,
    transfer: SafeTransfer<'static, Peripheral, Memory, TransferStart>,
}

pub struct Ongoing<CXX, ReqId, SyncED, EgED, Peripheral, Memory>
where
    CXX: ChannelId,
    ReqId: SerialRequest,
    SyncED: ISyncED,
    EgED: IEgED,
    Peripheral: Payload,
    Memory: Payload,
{
    mux: DmaMux<CXX, ReqId, SyncED, EgED>,
    transfer: SafeTransfer<'static, Peripheral, Memory, TransferOngoing<CXX>>,
}

impl<CXX, ReqId, SyncED, EgED> IState for Start<CXX, ReqId, SyncED, EgED>
where
    CXX: ChannelId,
    ReqId: SerialRequest,
    SyncED: ISyncED,
    EgED: IEgED,
{
}

impl<CXX, ReqId, SyncED, EgED, Peripheral, Memory> IState
    for Initialized<CXX, ReqId, SyncED, EgED, Peripheral, Memory>
where
    CXX: ChannelId,
    ReqId: SerialRequest,
    SyncED: ISyncED,
    EgED: IEgED,
    Peripheral: Payload,
    Memory: Payload,
{
}

impl<CXX, ReqId, SyncED, EgED, Peripheral, Memory> IState
    for Ongoing<CXX, ReqId, SyncED, EgED, Peripheral, Memory>
where
    CXX: ChannelId,
    ReqId: SerialRequest,
    SyncED: ISyncED,
    EgED: IEgED,
    Peripheral: Payload,
    Memory: Payload,
{
}

pub struct SerialDmaRx<USART, PINS, State>
where
    USART: SerialChannelDma + Deref<Target = RegisterBlock>,
    State: IState,
{
    serial: Serial<USART, PINS>,
    state: State,
}

impl<USART, PINS, CXX, SyncED, EgED>
    SerialDmaRx<USART, PINS, Start<CXX, USART::Rx, SyncED, EgED>>
where
    USART: SerialChannelDma + Deref<Target = RegisterBlock>,
    CXX: ChannelId,
    SyncED: ISyncED,
    EgED: IEgED,
{
    pub fn new(
        serial: Serial<USART, PINS>,
        channel: Channel<
            CXX,
            StreamDisabled,
            StreamIsrCleared,
            USART::Rx,
            SyncED,
            EgED,
        >,
    ) -> Self {
        let mut s = Self {
            serial,
            state: Start { channel },
        };

        s.enable_dma_mode();

        s
    }

    pub fn init<Peripheral, Memory>(
        self,
        memory: MemoryBufferStatic<Memory>,
    ) -> SerialDmaRx<
        USART,
        PINS,
        Initialized<CXX, USART::Rx, SyncED, EgED, Peripheral, Memory>,
    >
    where
        Peripheral: Payload,
        Memory: Payload,
    {
        if !memory.is_write() {
            panic!("The memory buffer must be a Write-Buffer.");
        }

        let pa = &self.serial.usart.rdr as *const _ as *const Peripheral;
        let peripheral = PeripheralBuffer::Fixed(FixedBuffer::Read(
            FixedBufferR::new(unsafe { &*pa }),
        ));

        let transfer = SafeTransfer::new(peripheral, memory);

        SerialDmaRx {
            serial: self.serial,
            state: Initialized {
                transfer,
                channel: self.state.channel,
            },
        }
    }

    pub fn free(
        mut self,
    ) -> (
        Serial<USART, PINS>,
        Channel<CXX, StreamDisabled, StreamIsrCleared, USART::Rx, SyncED, EgED>,
    ) {
        self.disable_dma_mode();

        (self.serial, self.state.channel)
    }

    fn enable_dma_mode(&mut self) {
        self.serial.usart.cr3.modify(|_, w| w.dmar().set_bit());
    }

    fn disable_dma_mode(&mut self) {
        self.serial.usart.cr3.modify(|_, w| w.dmar().clear_bit());
    }
}

impl<USART, PINS, CXX, SyncED, EgED, Peripheral, Memory>
    SerialDmaRx<
        USART,
        PINS,
        Initialized<CXX, USART::Rx, SyncED, EgED, Peripheral, Memory>,
    >
where
    USART: SerialChannelDma + Deref<Target = RegisterBlock>,
    CXX: ChannelId,
    SyncED: ISyncED,
    EgED: IEgED,
    Peripheral: Payload,
    Memory: Payload,
{
    pub fn start(
        self,
    ) -> SerialDmaRx<
        USART,
        PINS,
        Ongoing<CXX, USART::Rx, SyncED, EgED, Peripheral, Memory>,
    > {
        let transfer = self.state.transfer.start(self.state.channel.stream);

        SerialDmaRx {
            serial: self.serial,
            state: Ongoing {
                mux: self.state.channel.mux,
                transfer,
            },
        }
    }
}

impl<USART, PINS, CXX, SyncED, EgED, Peripheral, Memory>
    SerialDmaRx<
        USART,
        PINS,
        Ongoing<CXX, USART::Rx, SyncED, EgED, Peripheral, Memory>,
    >
where
    USART: SerialChannelDma + Deref<Target = RegisterBlock>,
    CXX: ChannelId,
    SyncED: ISyncED,
    EgED: IEgED,
    Peripheral: Payload,
    Memory: Payload,
{
    pub fn transfer(
        &self,
    ) -> &SafeTransfer<'static, Peripheral, Memory, TransferOngoing<CXX>> {
        &self.state.transfer
    }

    pub fn transfer_mut(
        &mut self,
    ) -> &mut SafeTransfer<'static, Peripheral, Memory, TransferOngoing<CXX>>
    {
        &mut self.state.transfer
    }

    pub fn mux(&self) -> &DmaMux<CXX, USART::Rx, SyncED, EgED> {
        &self.state.mux
    }

    pub fn stop(
        self,
        isr: &mut StreamIsr<CXX::DMA>,
    ) -> SerialDmaRx<USART, PINS, Start<CXX, USART::Rx, SyncED, EgED>> {
        let stream = self.state.transfer.stop().clear_isr(isr);
        let channel = Channel {
            stream,
            mux: self.state.mux,
        };

        SerialDmaRx {
            serial: self.serial,
            state: Start { channel },
        }
    }
}
