//! FMC Example
//!
//! Tested on a STM32H747I-DISCO
#![deny(warnings)]
#![no_main]
#![no_std]

use core::mem;
use core::slice;

#[macro_use]
#[allow(dead_code)]
mod utilities;

extern crate cortex_m;

use cortex_m_rt::entry;
use stm32h7xx_hal::gpio::{alt::fmc as alt, Speed};
use stm32h7xx_hal::{pac, prelude::*};

use stm32_fmc::devices::is42s32800g_6;

/// Configre a pin for the FMC controller
macro_rules! fmc_pins {
    ($($alt:ty: $pin:expr),*) => {
        (
            $(
                <$alt>::from($pin.into_alternate()
                    .internal_pull_up(true))
            ),*
        )
    };
}

#[entry]
fn main() -> ! {
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    // Initialise power...
    let pwr = dp.PWR.constrain();
    let pwrcfg = example_power!(pwr).freeze();

    // Initialise clocks...
    let rcc = dp.RCC.constrain();
    let ccdr = rcc
        .sys_ck(200.MHz())
        .hclk(200.MHz()) // FMC clock from HCLK by default
        .freeze(pwrcfg, &dp.SYSCFG);

    // Get the delay provider.
    let mut delay = cp.SYST.delay(ccdr.clocks);

    // Initialise system...
    cp.SCB.enable_icache();
    // See Errata Sheet 2.2.1
    //cp.SCB.enable_dcache(&mut cp.CPUID);
    cp.DWT.enable_cycle_counter();

    // Initialise IO...
    let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);
    let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);
    let gpiof = dp.GPIOF.split(ccdr.peripheral.GPIOF);
    let gpiog = dp.GPIOG.split(ccdr.peripheral.GPIOG);
    let gpioh = dp.GPIOH.split(ccdr.peripheral.GPIOH);
    let gpioi = dp.GPIOI.split(ccdr.peripheral.GPIOI);

    // ----------------------------------------------------------
    // Configure MPU for external SDRAM
    // MPU config for SDRAM write-through
    let sdram_size = 32 * 1024 * 1024;

    {
        let mpu = cp.MPU;
        let scb = &mut cp.SCB;
        let size = sdram_size;
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
            size & (size - 1),
            0,
            "SDRAM memory region size must be a power of 2"
        );
        assert_eq!(
            size & 0x1F,
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
                    | (log2minus1(size as u32) << 1)
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

    // ----------------------------------------------------------
    // SDRAM
    // Initialise SDRAM...
    let sdram_pins = fmc_pins! {
        // A0-A11
        alt::A0: gpiof.pf0,  alt::A1: gpiof.pf1,
        alt::A2: gpiof.pf2,  alt::A3: gpiof.pf3,
        alt::A4: gpiof.pf4,  alt::A5: gpiof.pf5,
        alt::A6: gpiof.pf12, alt::A7: gpiof.pf13,
        alt::A8: gpiof.pf14, alt::A9: gpiof.pf15,
        alt::A10: gpiog.pg0, alt::A11: gpiog.pg1,
        // BA0-BA1
        alt::Ba0: gpiog.pg4, alt::Ba1: gpiog.pg5,
        // D0-D31
        alt::D0: gpiod.pd14,  alt::D1: gpiod.pd15,
        alt::D2: gpiod.pd0,   alt::D3: gpiod.pd1,
        alt::D4: gpioe.pe7,   alt::D5: gpioe.pe8,
        alt::D6: gpioe.pe9,   alt::D7: gpioe.pe10,
        alt::D8: gpioe.pe11,  alt::D9: gpioe.pe12,
        alt::D10: gpioe.pe13, alt::D11: gpioe.pe14,
        alt::D12: gpioe.pe15, alt::D13: gpiod.pd8,
        alt::D14: gpiod.pd9,  alt::D15: gpiod.pd10,
        alt::D16: gpioh.ph8,  alt::D17: gpioh.ph9,
        alt::D18: gpioh.ph10, alt::D19: gpioh.ph11,
        alt::D20: gpioh.ph12, alt::D21: gpioh.ph13,
        alt::D22: gpioh.ph14, alt::D23: gpioh.ph15,
        alt::D24: gpioi.pi0,  alt::D25: gpioi.pi1,
        alt::D26: gpioi.pi2,  alt::D27: gpioi.pi3,
        alt::D28: gpioi.pi6,  alt::D29: gpioi.pi7,
        alt::D30: gpioi.pi9,  alt::D31: gpioi.pi10,
        // NBL0 - NBL3
        alt::Nbl0: gpioe.pe0, alt::Nbl1: gpioe.pe1,
        alt::Nbl2: gpioi.pi4, alt::Nbl3: gpioi.pi5,
        alt::Sdcke1: gpioh.ph7,
        alt::Sdclk: gpiog.pg8,
        alt::Sdncas: gpiog.pg15,
        alt::Sdne1: gpioh.ph6,
        alt::Sdnras: gpiof.pf11,             // SDRAS
        alt::Sdnwe: gpioh.ph5               // SDNWE
    };

    let mut sdram = dp.FMC.sdram(
        sdram_pins,
        is42s32800g_6::Is42s32800g {},
        ccdr.peripheral.FMC,
        &ccdr.clocks,
    );

    let ram_slice = unsafe {
        // Initialise controller and SDRAM
        let ram_ptr: *mut u32 = sdram.init(&mut delay);

        // Get 16-bit words
        let ram_ptr = ram_ptr as *mut u16;

        // Convert raw pointer to slice
        let ram_slice = slice::from_raw_parts_mut(ram_ptr, sdram_size);

        // Return a 4-word slice
        let size = mem::size_of::<u16>() * 4usize;
        let mut chunks = ram_slice.chunks_exact_mut(size);
        chunks.next().unwrap()
    };

    // ----------------------------------------------------------
    // Use memory in SDRAM

    ram_slice[0] = 1u16;
    ram_slice[1] = 2;
    ram_slice[2] = 3;
    ram_slice[3] = 4;

    assert_eq!(ram_slice[0], 1);

    loop {
        cortex_m::asm::nop()
    }
}
