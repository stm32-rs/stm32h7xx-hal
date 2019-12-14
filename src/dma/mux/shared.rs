use super::super::stm32::dmamux1::{CFR, CSR, RGCFR, RGSR};

pub struct MuxIsr {
    /// This field *must not* be mutated through shared references
    pub(in super::super) csr: &'static mut CSR,
    /// This field *must not* be mutated through shared references
    pub(in super::super) cfr: &'static mut CFR,
}

unsafe impl Sync for MuxIsr {}

pub struct RequestGenIsr {
    pub(super) rgsr: &'static mut RGSR,
    pub(super) rgcfr: &'static mut RGCFR,
}

unsafe impl Sync for RequestGenIsr {}
