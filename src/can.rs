//! CAN and FD-CAN
//!
//! The module implements CAN and CAN-FD support using the [fdcan] crate.
//!
//! # Message RAM
//!
//! The Message RAM allocation is fixed as follows
//!
//! | Section | Size
//! | --- | ---
//! | Standard 11-bit filters | 28
//! | Extended 29-bit filters | 8
//! | Rx FIFO 0 | 3 elements
//! | Rx FIFO 1 | 3 elements
//! | Tx Buffers | 3 elements
//!
//! # Usage
//!
//! In `Cargo.toml`
//! ```toml
//! fdcan = "^0.1"
//! ```
//!
//! Initialisation
//! ```
//! use stm32h7xx_hal::prelude::*;
//! use fdcan::{
//!    config::NominalBitTiming,
//!    filter::{StandardFilter, StandardFilterSlot},
//!    id::StandardId,
//!    FdCan,
//! };
//!
//! let mut can: FdCan<_, fdcan::ConfigMode> = dp.FDCAN1.fdcan(tx, rx, fdcan_prec);
//! ```
//!
//! [fdcan]: https://docs.rs/fdcan
//!
//! # Examples
//!
//! - [Basic Example](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/can-echo.rs)
//! - [CAN-FD with Bit Rate Switching](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/can-fd.rs)

use crate::rcc::{rec, rec::ResetEnable};

/// Storage type for the CAN controller
#[derive(Debug)]
pub struct Can<FDCAN> {
    rb: FDCAN,
}
impl<FDCAN> Can<FDCAN> {
    /// Returns a reference to the inner peripheral
    fn inner(&self) -> &FDCAN {
        &self.rb
    }
}

pub trait Instance: crate::Sealed + crate::gpio::alt::CanCommon {}

/// Extension trait for CAN controller
pub trait CanExt: Sized + Instance
where
    Can<Self>: fdcan::Instance,
{
    fn fdcan(
        self,
        tx: impl Into<Self::Tx>,
        rx: impl Into<Self::Rx>,
        prec: rec::Fdcan,
    ) -> fdcan::FdCan<Can<Self>, fdcan::ConfigMode> {
        let _pins = (tx.into(), rx.into());
        self.fdcan_unchecked(prec)
    }

    fn fdcan_unchecked(
        self,
        prec: rec::Fdcan,
    ) -> fdcan::FdCan<Can<Self>, fdcan::ConfigMode>;
}

/// Configure Message RAM layout on H7 to match the fixed sized used on G4
///
/// These are protected bits, write access is only possible when bit CCE and bit
/// INIT for FDCAN_CCCR are set to 1
macro_rules! message_ram_layout {
    ($can:ident, $start_word_addr:expr) => {
        use fdcan::message_ram::*;
        let mut word_adr: u16 = $start_word_addr;

        // 11-bit filter
        $can.sidfc
            .modify(|_, w| unsafe { w.flssa().bits(word_adr) });
        word_adr += STANDARD_FILTER_MAX as u16;
        // 29-bit filter
        $can.xidfc
            .modify(|_, w| unsafe { w.flesa().bits(word_adr) });
        word_adr += 2 * EXTENDED_FILTER_MAX as u16;
        // Rx FIFO 0
        $can.rxf0c.modify(|_, w| unsafe {
            w.f0sa()
                .bits(word_adr)
                .f0s()
                .bits(RX_FIFO_MAX)
                .f0wm()
                .bits(RX_FIFO_MAX)
        });
        word_adr += 18 * RX_FIFO_MAX as u16;
        // Rx FIFO 1
        $can.rxf1c.modify(|_, w| unsafe {
            w.f1sa()
                .bits(word_adr)
                .f1s()
                .bits(RX_FIFO_MAX)
                .f1wm()
                .bits(RX_FIFO_MAX)
        });
        word_adr += 18 * RX_FIFO_MAX as u16;
        // Rx buffer - see below
        // Tx event FIFO
        $can.txefc.modify(|_, w| unsafe {
            w.efsa()
                .bits(word_adr)
                .efs()
                .bits(TX_EVENT_MAX)
                .efwm()
                .bits(TX_EVENT_MAX)
        });
        word_adr += 2 * TX_EVENT_MAX as u16;
        // Tx buffers
        $can.txbc.modify(|_, w| unsafe {
            w.tbsa().bits(word_adr).tfqs().bits(TX_FIFO_MAX)
        });
        word_adr += 18 * TX_FIFO_MAX as u16;

        // Rx Buffer - not used
        $can.rxbc.modify(|_, w| unsafe { w.rbsa().bits(word_adr) });

        // TX event FIFO?
        // Trigger memory?

        // Set the element sizes to 16 bytes
        $can.rxesc.modify(|_, w| unsafe {
            w.rbds().bits(0b111).f1ds().bits(0b111).f0ds().bits(0b111)
        });
        $can.txesc.modify(|_, w| unsafe { w.tbds().bits(0b111) });
    };
}

mod fdcan1 {
    use super::{rec, Can, CanExt, ResetEnable};
    use crate::stm32::FDCAN1;

    impl crate::Sealed for FDCAN1 {}
    impl super::Instance for FDCAN1 {}

    impl Can<FDCAN1> {
        pub fn fdcan1(
            rb: FDCAN1,
            prec: rec::Fdcan,
        ) -> fdcan::FdCan<Self, fdcan::ConfigMode> {
            prec.enable(); // Enable APB1 peripheral clock

            // Initialisation and RAM layout configuation
            let mut fdcan = fdcan::FdCan::new(Self { rb }).into_config_mode();
            let can = fdcan.instance().inner();
            message_ram_layout!(can, 0x000);

            fdcan
        }
    }
    impl CanExt for FDCAN1 {
        fn fdcan_unchecked(
            self,
            prec: rec::Fdcan,
        ) -> fdcan::FdCan<Can<Self>, fdcan::ConfigMode> {
            Can::fdcan1(self, prec)
        }
    }
    unsafe impl fdcan::Instance for Can<FDCAN1> {
        const REGISTERS: *mut fdcan::RegisterBlock = FDCAN1::ptr() as *mut _;
    }
    unsafe impl fdcan::message_ram::Instance for Can<FDCAN1> {
        const MSG_RAM: *mut fdcan::message_ram::RegisterBlock =
            (0x4000_ac00 as *mut _);
    }
}

mod fdcan2 {
    use super::{rec, Can, CanExt, ResetEnable};
    use crate::stm32::FDCAN2;

    impl crate::Sealed for FDCAN2 {}
    impl super::Instance for FDCAN2 {}

    impl Can<FDCAN2> {
        pub fn fdcan2(
            rb: FDCAN2,
            prec: rec::Fdcan,
        ) -> fdcan::FdCan<Self, fdcan::ConfigMode> {
            prec.enable(); // Enable APB1 peripheral clock

            // Initialisation and RAM layout configuation
            let mut fdcan = fdcan::FdCan::new(Self { rb }).into_config_mode();
            let can = fdcan.instance().inner();
            message_ram_layout!(can, 0x400); // + 1k words = 4kB

            fdcan
        }
    }
    impl CanExt for FDCAN2 {
        fn fdcan_unchecked(
            self,
            prec: rec::Fdcan,
        ) -> fdcan::FdCan<Can<Self>, fdcan::ConfigMode> {
            Can::fdcan2(self, prec)
        }
    }
    unsafe impl fdcan::Instance for Can<FDCAN2> {
        const REGISTERS: *mut fdcan::RegisterBlock = FDCAN2::ptr() as *mut _;
    }
    unsafe impl fdcan::message_ram::Instance for Can<FDCAN2> {
        const MSG_RAM: *mut fdcan::message_ram::RegisterBlock =
            ((0x4000_ac00 + 0x1000) as *mut _); // FDCAN1 + 4kB
    }
}
