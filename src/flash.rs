//! Flash memory

use crate::stm32::{flash, FLASH};

/// Extension trait to constrain the FLASH peripheral
pub trait FlashExt {
    /// Constrains the FLASH peripheral to play nicely with the other abstractions
    fn constrain(self) -> Flash;
}

impl FlashExt for FLASH {
    fn constrain(self) -> Flash {
        Flash {
            acr: ACR { _0: () },
        }
    }
}

/// Constrained FLASH peripheral
pub struct Flash {
    /// Opaque ACR register
    pub acr: ACR,
}

/// Opaque ACR register
pub struct ACR {
    _0: (),
}

impl ACR {
    #[allow(unused)]
    pub(crate) fn acr(&mut self) -> &flash::ACR {
        // NOTE(unsafe) this proxy grants exclusive access to this register
        unsafe { &(*FLASH::ptr()).acr }
    }
}
