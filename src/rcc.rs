//! Reset and Clock Control

use crate::stm32::{rcc, RCC};

use crate::flash::ACR;
use crate::time::Hertz;

/*

Currently, this whole module does not attempt to allow setting any clocks
but only providing currect information from the default power-on state.

Check Fig 46 "Core and bus clock generation" in the reference manual for
information (p 336).

"After a system reset, the HSI is selected as system clock and all PLLs are
switched OFF."

HSI is 64 MHz.

*/

/// Extension trait that constrains the `RCC` peripheral
pub trait RccExt {
    /// Constrains the `RCC` peripheral so it plays nicely with the other abstractions
    fn constrain(self) -> Rcc;
}

impl RccExt for RCC {
    fn constrain(self) -> Rcc {
        Rcc {
            ahb3: AHB3 { _0: () },
            ahb4: AHB4 { _0: () },
            apb1: APB1 { _0: () },
            apb2: APB2 { _0: () },
            apb4: APB4 { _0: () },
            cfgr: CFGR {
                // TODO: put the clocks here
            },
        }
    }
}

/// Constrained RCC peripheral
pub struct Rcc {
    /// AMBA High-performance Bus (AHB3) registers
    pub ahb3: AHB3,
    /// AMBA High-performance Bus (AHB4) registers
    pub ahb4: AHB4,
    /// Advanced Peripheral Bus 1 (APB1) registers
    pub apb1: APB1,
    /// Advanced Peripheral Bus 2 (APB2) registers
    pub apb2: APB2,
    /// Advanced Peripheral Bus 4 (APB4) registers
    pub apb4: APB4,
    /// Clock configuration
    pub cfgr: CFGR,
}

/// AMBA High-performance Bus (AHB) registers
pub struct AHB3 {
    _0: (),
}

impl AHB3 {
    #[allow(unused)]
    pub(crate) fn enr(&mut self) -> &rcc::AHB3ENR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).ahb3enr }
    }

    #[allow(unused)]
    pub(crate) fn rstr(&mut self) -> &rcc::AHB3RSTR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).ahb3rstr }
    }
}

/// AMBA High-performance Bus (AHB) registers
pub struct AHB4 {
    _0: (),
}

impl AHB4 {
    pub(crate) fn enr(&mut self) -> &rcc::AHB4ENR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).ahb4enr }
    }

    pub(crate) fn rstr(&mut self) -> &rcc::AHB4RSTR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).ahb4rstr }
    }
}

/// Advanced Peripheral Bus 1 (APB1) registers
pub struct APB1 {
    _0: (),
}

impl APB1 {
    pub(crate) fn lenr(&mut self) -> &rcc::APB1LENR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).apb1lenr }
    }

    pub(crate) fn lrstr(&mut self) -> &rcc::APB1LRSTR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).apb1lrstr }
    }
}

/// Advanced Peripheral Bus 2 (APB2) registers
pub struct APB2 {
    _0: (),
}

impl APB2 {
    pub(crate) fn enr(&mut self) -> &rcc::APB2ENR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).apb2enr }
    }

    pub(crate) fn rstr(&mut self) -> &rcc::APB2RSTR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).apb2rstr }
    }
}

/// Advanced Peripheral Bus 4 (APB4) registers
pub struct APB4 {
    _0: (),
}

impl APB4 {
    #[allow(unused)]
    pub(crate) fn enr(&mut self) -> &rcc::APB4ENR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).apb4enr }
    }

    #[allow(unused)]
    pub(crate) fn rstr(&mut self) -> &rcc::APB4RSTR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*RCC::ptr()).apb4rstr }
    }
}

const HSI: u32 = 64_000_000; // Hz

/// Clock configuration
pub struct CFGR {
    // TODO: put the clocks here
}

impl CFGR {
    /// Freezes the clock configuration, making it effective
    pub fn freeze(self, _acr: &mut ACR) -> Clocks {
        Clocks {
            sys_ck: Hertz(HSI),
            c_ck: Hertz(HSI),
            aclk: Hertz(HSI),
            hclk: Hertz(HSI),
            hclk3: Hertz(HSI),
            hclk4: Hertz(HSI),
            pclk1: Hertz(HSI),
            pclk2: Hertz(HSI),
            pclk3: Hertz(HSI),
            pclk4: Hertz(HSI),
        }
    }
}

/// Frozen clock frequencies
///
/// The existence of this value indicates that the clock configuration can no longer be changed
#[derive(Clone, Copy)]
pub struct Clocks {
    sys_ck: Hertz,
    c_ck: Hertz,
    aclk: Hertz,
    hclk: Hertz,
    hclk3: Hertz,
    hclk4: Hertz,
    pclk1: Hertz,
    pclk2: Hertz,
    pclk3: Hertz,
    pclk4: Hertz,
}

impl Clocks {
    /// Returns the system clock frequency
    pub fn sys_ck(&self) -> Hertz {
        self.sys_ck
    }

    /// Returns the CPU clock frequency
    pub fn c_ck(&self) -> Hertz {
        self.c_ck
    }

    /// Returns the AXI clock frequency
    pub fn aclk(&self) -> Hertz {
        self.aclk
    }

    /// Returns AHB1 and AHB2 clock frequency
    pub fn hclk(&self) -> Hertz {
        self.hclk
    }

    /// Returns AHB3 clock frequency
    pub fn hclk3(&self) -> Hertz {
        self.hclk3
    }

    /// Returns AHB4 clock frequency
    pub fn hclk4(&self) -> Hertz {
        self.hclk4
    }

    /// Returns APB1 clock frequency
    pub fn pclk1(&self) -> Hertz {
        self.pclk1
    }

    /// Returns APB2 clock frequency
    pub fn pclk2(&self) -> Hertz {
        self.pclk2
    }

    /// Returns PCLK3 clock frequency
    pub fn pclk3(&self) -> Hertz {
        self.pclk3
    }

    /// Returns PCLK4 clock frequency
    pub fn pclk4(&self) -> Hertz {
        self.pclk4
    }

}
