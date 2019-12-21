use super::super::stm32::dmamux1::{CFR, CSR, RGCFR, RGSR};

pub struct MuxIsr {
    pub(in super::super) csr: &'static CSR,
    /// This field *must not* be mutated using shared references
    pub(in super::super) cfr: &'static mut CFR,
}

unsafe impl Send for MuxIsr {}
unsafe impl Sync for MuxIsr {}

pub struct RequestGenIsr {
    pub(super) rgsr: &'static RGSR,
    /// This field *must not* be mutated using shared references
    pub(super) rgcfr: &'static mut RGCFR,
}

unsafe impl Send for RequestGenIsr {}
unsafe impl Sync for RequestGenIsr {}
