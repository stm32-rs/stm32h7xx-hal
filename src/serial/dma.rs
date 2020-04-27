use super::Serial;
use crate::dma::mux::request_ids::{
    RequestIdSome, Uart4RxDma, Uart4TxDma, Uart5RxDma, Uart5TxDma, Uart7RxDma,
    Uart7TxDma, Uart8RxDma, Uart8TxDma, Usart1RxDma, Usart1TxDma, Usart2RxDma,
    Usart2TxDma, Usart3RxDma, Usart3TxDma, Usart6RxDma, Usart6TxDma,
};
use crate::dma::transfer::buffer::{
    FixedBuffer, FixedBufferMut, FixedBufferRef, MemoryBufferType,
};
use crate::dma::transfer::config::{
    MemoryToPeripheral, PeripheralToMemory, TransferDirection,
};
use crate::dma::transfer::{Buffer, Buffers, Byte, Payload};
use crate::private;
use crate::stm32::usart1::RegisterBlock;
use crate::stm32::{UART4, UART5, UART7, UART8};
use crate::stm32::{USART1, USART2, USART3, USART6};

use core::cell::UnsafeCell;
use core::marker::PhantomData;
use core::ops::Deref;

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

pub trait RxRequest: SerialRequest {}
pub trait TxRequest: SerialRequest {}

impl RxRequest for Usart1RxDma {}
impl TxRequest for Usart1TxDma {}

impl RxRequest for Usart2RxDma {}
impl TxRequest for Usart2TxDma {}

impl RxRequest for Usart3RxDma {}
impl TxRequest for Usart3TxDma {}

impl RxRequest for Uart4RxDma {}
impl TxRequest for Uart4TxDma {}

impl RxRequest for Uart5RxDma {}
impl TxRequest for Uart5TxDma {}

impl RxRequest for Usart6RxDma {}
impl TxRequest for Usart6TxDma {}

impl RxRequest for Uart7RxDma {}
impl TxRequest for Uart7TxDma {}

impl RxRequest for Uart8RxDma {}
impl TxRequest for Uart8TxDma {}

pub trait SerialChannelDma {
    type Rx: RxRequest;
    type Tx: TxRequest;
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

pub trait SerialDirection<Peripheral, Memory>: private::Sealed
where
    Peripheral: Payload<Size = Byte>,
    Memory: Payload,
{
    #[doc(hidden)]
    unsafe fn transfer_direction<USART, PINS>(
        serial: &Serial<USART, PINS>,
        memory_buffer: MemoryBufferType<Memory>,
    ) -> TransferDirection<'static, Peripheral, Memory>
    where
        USART: Deref<Target = RegisterBlock>;
}

pub struct Rx;
pub struct Tx;

impl private::Sealed for Rx {}
impl private::Sealed for Tx {}

impl<Peripheral, Memory> SerialDirection<Peripheral, Memory> for Rx
where
    Peripheral: Payload<Size = Byte>,
    Memory: Payload,
{
    #[doc(hidden)]
    unsafe fn transfer_direction<USART, PINS>(
        serial: &Serial<USART, PINS>,
        memory_buffer: MemoryBufferType<Memory>,
    ) -> TransferDirection<'static, Peripheral, Memory>
    where
        USART: Deref<Target = RegisterBlock>,
    {
        let ptr = &serial.usart.rdr as *const _ as *const Peripheral;
        let peripheral_buffer =
            Buffer::Fixed(FixedBuffer::Ref(FixedBufferRef::new(&*ptr)));
        let buffers = Buffers {
            peripheral_buffer,
            memory_buffer,
        };

        TransferDirection::P2M(PeripheralToMemory::new(buffers))
    }
}

impl<Peripheral, Memory> SerialDirection<Peripheral, Memory> for Tx
where
    Peripheral: Payload<Size = Byte>,
    Memory: Payload,
{
    #[doc(hidden)]
    unsafe fn transfer_direction<USART, PINS>(
        serial: &Serial<USART, PINS>,
        memory_buffer: MemoryBufferType<Memory>,
    ) -> TransferDirection<'static, Peripheral, Memory>
    where
        USART: Deref<Target = RegisterBlock>,
    {
        let reg_cell =
            &*(&serial.usart.tdr as *const _ as *const UnsafeCell<u32>);
        let ptr = reg_cell.get() as *mut Peripheral;

        let peripheral_buffer =
            Buffer::Fixed(FixedBuffer::Mut(FixedBufferMut::new(&mut *ptr)));
        let buffers = Buffers {
            peripheral_buffer,
            memory_buffer,
        };

        TransferDirection::M2P(MemoryToPeripheral::new(buffers))
    }
}

pub struct Config<Direction, Peripheral, Memory>
where
    Direction: SerialDirection<Peripheral, Memory>,
    Peripheral: Payload<Size = Byte>,
    Memory: Payload,
{
    memory_buffer: MemoryBufferType<Memory>,
    _phantom: PhantomData<(Direction, Peripheral)>,
}

impl<Direction, Peripheral, Memory> Config<Direction, Peripheral, Memory>
where
    Direction: SerialDirection<Peripheral, Memory>,
    Peripheral: Payload<Size = Byte>,
    Memory: Payload,
{
    unsafe fn build_transfer_direction<USART, PINS>(
        serial: &Serial<USART, PINS>,
        memory_buffer: MemoryBufferType<Memory>,
    ) -> TransferDirection<'static, Peripheral, Memory>
    where
        USART: Deref<Target = RegisterBlock>,
    {
        Direction::transfer_direction(serial, memory_buffer)
    }
}
