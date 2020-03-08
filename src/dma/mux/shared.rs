//! DMA Mux shared access objects

use super::super::stm32::dmamux1::{CFR, CSR, RGCFR, RGSR};

pub struct MuxIsr {
    pub(in super::super) csr: &'static CSR,
    /// This field *must not* be mutated using shared references
    pub(in super::super) cfr: &'static CFR,
}

unsafe impl Send for MuxIsr {}
unsafe impl Sync for MuxIsr {}

pub struct RequestGenIsr {
    pub(super) rgsr: &'static RGSR,
    /// This field *must not* be mutated using shared references
    pub(super) rgcfr: &'static RGCFR,
}

impl RequestGenIsr {
    pub(in super::super) fn new(
        rgsr: &'static RGSR,
        rgcfr: &'static RGCFR,
    ) -> Self {
        RequestGenIsr { rgsr, rgcfr }
    }
}

unsafe impl Send for RequestGenIsr {}
unsafe impl Sync for RequestGenIsr {}
