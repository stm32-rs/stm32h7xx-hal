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
use stm32h7xx_hal::gpio::Speed;
use stm32h7xx_hal::{pac, prelude::*};

use stm32_fmc::devices::is42s32800g_6;

/// Configre a pin for the FMC controller
macro_rules! fmc_pins {
    ($($pin:expr),*) => {
        (
            $(
                $pin.into_push_pull_output()
                    .set_speed(Speed::VeryHigh)
                    .into_alternate_af12()
                    .internal_pull_up(true)
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
        .sys_ck(200.mhz())
        .hclk(200.mhz()) // FMC clock from HCLK by default
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
        gpiof.pf0, gpiof.pf1, gpiof.pf2, gpiof.pf3,
        gpiof.pf4, gpiof.pf5, gpiof.pf12, gpiof.pf13,
        gpiof.pf14, gpiof.pf15, gpiog.pg0, gpiog.pg1,
        // BA0-BA1
        gpiog.pg4, gpiog.pg5,
        // D0-D31
        gpiod.pd14, gpiod.pd15, gpiod.pd0, gpiod.pd1,
        gpioe.pe7, gpioe.pe8, gpioe.pe9, gpioe.pe10,
        gpioe.pe11, gpioe.pe12, gpioe.pe13, gpioe.pe14,
        gpioe.pe15, gpiod.pd8, gpiod.pd9, gpiod.pd10,
        gpioh.ph8, gpioh.ph9, gpioh.ph10, gpioh.ph11,
        gpioh.ph12, gpioh.ph13, gpioh.ph14, gpioh.ph15,
        gpioi.pi0, gpioi.pi1, gpioi.pi2, gpioi.pi3,
        gpioi.pi6, gpioi.pi7, gpioi.pi9, gpioi.pi10,
        // NBL0 - NBL3
        gpioe.pe0, gpioe.pe1, gpioi.pi4, gpioi.pi5,
        gpioh.ph7,              // SDCKE1
        gpiog.pg8,              // SDCLK
        gpiog.pg15,             // SDNCAS
        gpioh.ph6,              // SDNE1 (!CS)
        gpiof.pf11,             // SDRAS
        gpioh.ph5               // SDNWE
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
