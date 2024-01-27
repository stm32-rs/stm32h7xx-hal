use cortex_m::peripheral::{MPU, SCB};

#[allow(unused, unsafe_code)]
pub fn init_mpu(mpu: MPU, scb: &mut SCB, sdram_size: usize) {
    // Refer to ARM®v7-M Architecture Reference Manual ARM DDI 0403
    // Version E.b Section B3.5
    const MEMFAULTENA: u32 = 1 << 16;

    unsafe {
        /* Make sure outstanding transfers are done */
        cortex_m::asm::dmb();

        scb.shcsr.modify(|r| r & !MEMFAULTENA);

        /* Disable the MPU and clear the control register*/
        mpu.ctrl.write(0);
    }

    const REGION_NUMBER0: u32 = 0x00;
    const REGION_BASE_ADDRESS: u32 = 0xD000_0000;

    const REGION_FULL_ACCESS: u32 = 0x03;
    const REGION_CACHEABLE: u32 = 0x01;
    const REGION_WRITE_BACK: u32 = 0x01;
    const REGION_ENABLE: u32 = 0x01;

    assert_eq!(
        sdram_size & (sdram_size - 1),
        0,
        "SDRAM memory region size must be a power of 2"
    );
    assert_eq!(
        sdram_size & 0x1F,
        0,
        "SDRAM memory region size must be 32 bytes or more"
    );
    fn log2minus1(sz: u32) -> u32 {
        for i in 5..=31 {
            if sz == (1 << i) {
                return i - 1;
            }
        }
        panic!("Unknown SDRAM memory region size!");
    }

    //info!("SDRAM Memory Size 0x{:x}", log2minus1(size as u32));

    // Configure region 0
    //
    // Cacheable, outer and inner write-back, no write allocate. So
    // reads are cached, but writes always write all the way to SDRAM
    unsafe {
        mpu.rnr.write(REGION_NUMBER0);
        mpu.rbar.write(REGION_BASE_ADDRESS);
        mpu.rasr.write(
            (REGION_FULL_ACCESS << 24)
                | (REGION_CACHEABLE << 17)
                | (REGION_WRITE_BACK << 16)
                | (log2minus1(sdram_size as u32) << 1)
                | REGION_ENABLE,
        );
    }

    const MPU_ENABLE: u32 = 0x01;
    const MPU_DEFAULT_MMAP_FOR_PRIVILEGED: u32 = 0x04;

    // Enable
    unsafe {
        mpu.ctrl
            .modify(|r| r | MPU_DEFAULT_MMAP_FOR_PRIVILEGED | MPU_ENABLE);

        scb.shcsr.modify(|r| r | MEMFAULTENA);

        // Ensure MPU settings take effect
        cortex_m::asm::dsb();
        cortex_m::asm::isb();
    }
}
