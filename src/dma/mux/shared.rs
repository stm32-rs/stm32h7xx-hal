use super::super::stm32::dmamux1::{CFR, CSR, RGSR, RGCFR};

pub struct MuxIsr {
    pub(in super::super) csr: &'static mut CSR,
    pub(in super::super) cfr: &'static mut CFR,
}

unsafe impl Sync for MuxIsr {}

pub struct RequestGenIsr {
    pub(in super::super) rgsr: &'static mut RGSR,
    pub(in super::super) rgcfr: &'static mut RGCFR,
}

unsafe impl Sync for RequestGenIsr {}
