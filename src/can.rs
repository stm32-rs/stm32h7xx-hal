//! # Controller Area Network (CAN) Interface
//!

use crate::fdcan;
use crate::fdcan::message_ram;
// use crate::stm32::{self, FDCAN1, FDCAN2, FDCAN3};
use crate::rcc::Rcc;
use crate::stm32;

mod sealed {
    // Prevent creation outside of this module
    pub trait Sealed {}
    /// A TX pin configured for CAN communication
    pub trait Tx<CAN> {}
    /// An RX pin configured for CAN communication
    pub trait Rx<CAN> {}
}

/// Implements sealed::{Tx,Rx} for pins associated with a CAN peripheral
macro_rules! pins {
    ($PER:ident =>
        (tx: [ $($( #[ $pmetatx:meta ] )* $tx:ident<$txaf:ident>),+ $(,)? ],
         rx: [ $($( #[ $pmetarx:meta ] )* $rx:ident<$rxaf:ident>),+ $(,)? ])) => {
        $(
            $( #[ $pmetatx ] )*
            impl crate::can::sealed::Tx<$PER> for $tx<crate::gpio::Alternate<$txaf>> {}
        )+
        $(
            $( #[ $pmetarx ] )*
            impl crate::can::sealed::Rx<$PER> for $rx<crate::gpio::Alternate<$rxaf>> {}
        )+
    };
}

mod fdcan1 {
    use super::FdCan;
    use crate::fdcan;
    use crate::fdcan::message_ram;
    use crate::gpio::{
        gpioa::{PA11, PA12},
        gpiob::{PB8, PB9},
        gpiod::{PD0, PD1},
        AF9,
    };
    use crate::rcc::Rcc;
    use crate::stm32;
    use crate::stm32::FDCAN1;

    // All STM32G4 models with CAN support these pins
    pins! {
        FDCAN1 => (
            tx: [
                PA12<AF9>,
                PB9<AF9>,
                PD1<AF9>,
            ],
            rx: [
                PA11<AF9>,
                PB8<AF9>,
                PD0<AF9>,
            ]
        )
    }

    unsafe impl fdcan::Instance for FdCan<FDCAN1> {
        const REGISTERS: *mut stm32::fdcan::RegisterBlock =
            FDCAN1::ptr() as *mut _;
    }

    unsafe impl message_ram::MsgRamExt for FdCan<FDCAN1> {
        const MSG_RAM: *mut message_ram::RegisterBlock =
            (0x4000_a400 as *mut _);
    }

    /// Implements sealed::Sealed and Enable for a CAN peripheral (e.g. CAN1)
    impl crate::can::sealed::Sealed for crate::stm32::FDCAN1 {}
    impl crate::can::Enable for crate::stm32::FDCAN1 {
        #[inline(always)]
        fn enable(rcc: &Rcc) {
            // TODO: make this configurable
            // Select P clock as FDCAN clock source
            rcc.rb.ccipr.modify(|_, w| {
                // This is sound, as `0b10` is a valid value for this field.
                unsafe {
                    w.fdcansel().bits(0b10);
                }

                w
            });

            // Enable peripheral
            rcc.rb.apb1enr1.modify(|_, w| w.fdcanen().set_bit());
        }
    }
}

#[cfg(any(
    feature = "stm32g471",
    feature = "stm32g473",
    feature = "stm32g474",
    feature = "stm32g483",
    feature = "stm32g484",
    feature = "stm32g491",
    feature = "stm32g4A1",
))]
mod fdcan2 {
    use super::FdCan;
    use crate::fdcan;
    use crate::fdcan::message_ram;
    use crate::gpio::{
        gpiob::{PB12, PB13, PB5, PB6},
        AF9,
    };
    use crate::rcc::Rcc;
    use crate::stm32::{self, FDCAN2};

    pins! {
        FDCAN2 => (
            tx: [
                PB6<AF9>,
                PB13<AF9>,
            ],
            rx: [
                PB5<AF9>,
                PB12<AF9>,
        ])
    }

    unsafe impl fdcan::Instance for FdCan<FDCAN2> {
        const REGISTERS: *mut stm32::fdcan::RegisterBlock =
            FDCAN2::ptr() as *mut _;
    }

    unsafe impl message_ram::MsgRamExt for FdCan<FDCAN2> {
        // const MSG_RAM: *mut message_ram::RegisterBlock = (0x4000_a754 as *mut _);
        const MSG_RAM: *mut message_ram::RegisterBlock =
            (0x4000_a750 as *mut _);
    }

    impl crate::can::sealed::Sealed for crate::stm32::FDCAN2 {}
    impl crate::can::Enable for crate::stm32::FDCAN2 {
        #[inline(always)]
        fn enable(rcc: &Rcc) {
            // Enable peripheral
            rcc.rb.apb1enr1.modify(|_, w| w.fdcanen().set_bit());

            // TODO: make this configurable
            // Select P clock as FDCAN clock source
            rcc.rb.ccipr.modify(|_, w| {
                // This is sound, as `0b10` is a valid value for this field.
                unsafe {
                    w.fdcansel().bits(0b10);
                }

                w
            });
        }
    }
}

#[cfg(any(
    feature = "stm32g473",
    feature = "stm32g474",
    feature = "stm32g483",
    feature = "stm32g484",
))]
mod fdcan3 {
    use super::FdCan;
    use crate::fdcan;
    use crate::fdcan::message_ram;
    use crate::gpio::{
        gpioa::{PA15, PA8},
        gpiob::{PB3, PB4},
        AF11,
    };
    use crate::rcc::Rcc;
    use crate::stm32::{self, FDCAN3};

    pins! {
        FDCAN3 => (
            tx: [
                PA15<AF11>,
                PB4<AF11>,
            ],
            rx: [
                PA8<AF11>,
                PB3<AF11>,
        ])
    }

    unsafe impl fdcan::Instance for FdCan<FDCAN3> {
        const REGISTERS: *mut stm32::fdcan::RegisterBlock =
            FDCAN3::ptr() as *mut _;
    }

    unsafe impl message_ram::MsgRamExt for FdCan<FDCAN3> {
        const MSG_RAM: *mut message_ram::RegisterBlock =
            (0x4000_aaa0 as *mut _);
    }
}

/// Enable/disable peripheral
pub trait Enable: sealed::Sealed {
    /// Enables this peripheral by setting the associated enable bit in an RCC enable register
    fn enable(rcc: &Rcc);
}

/// Interface to the CAN peripheral.
pub struct FdCan<Instance> {
    _peripheral: Instance,
}

impl<Instance> FdCan<Instance>
where
    Instance: Enable,
{
    /// Creates a CAN interface.
    pub fn new<TX, RX>(
        can: Instance,
        tx: TX,
        rx: RX,
        rcc: &Rcc,
    ) -> FdCan<Instance>
    where
        TX: sealed::Tx<Instance>,
        RX: sealed::Rx<Instance>,
    {
        Instance::enable(rcc);
        //TODO: Set Speed to VeryHigh?
        FdCan { _peripheral: can }
    }

    pub fn new_unchecked(can: Instance, rcc: &Rcc) -> FdCan<Instance> {
        Instance::enable(rcc);
        FdCan { _peripheral: can }
    }
}
