use super::super::stm32::dmamux1::{CFR, CSR};

pub struct MuxIsr {
    pub(in super::super) csr: &'static CSR,
    pub(in super::super) cfr: &'static CFR,
}

unsafe impl Sync for MuxIsr {}
