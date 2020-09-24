#![deny(warnings)]
#![no_main]
#![no_std]

#[path = "utilities/logger.rs"]
mod logger;

use cortex_m_rt::entry;
use stm32h7xx_hal::rcc::{rec, rec::I2c123ClkSel, ResetEnable};
use stm32h7xx_hal::{pac, prelude::*};

fn enable_fdcan(rec: rec::Fdcan) {
    // Enable and set individual kernel clock to PLL1 Q CK
    rec.enable().kernel_clk_mux(rec::FdcanClkSel::PLL1_Q);

    // rec is dropped here, and can never be changed again
}

#[entry]
fn main() -> ! {
    logger::init();
    let dp = pac::Peripherals::take().unwrap();

    // Constrain and Freeze power
    let pwr = dp.PWR.constrain();
    let pwrcfg = pwr.freeze();

    // Constrain and Freeze clock
    let rcc = dp.RCC.constrain();
    let mut ccdr = rcc
        .sys_ck(100.mhz())
        .pll1_q_ck(4.mhz())
        .pll3_p_ck(4.mhz())
        .pll3_r_ck(4.mhz())
        .freeze(pwrcfg, &dp.SYSCFG);

    // Set group kernel clock to PLL3 R CK. Needs mutable ccdr
    ccdr.peripheral.kernel_i2c123_clk_mux(I2c123ClkSel::PLL3_R);

    // (now ccdr can be immutable)

    enable_fdcan(ccdr.peripheral.FDCAN);

    // Compile error: value used here after move
    //ccdr.peripheral.FDCAN.disable();

    // Compile error: value borrowed here after partial move
    //
    // Can't change group clocks - ccdr.peripheral has been partially used
    //ccdr.peripheral.kernel_i2c123_clk_mux(I2c123ClkSel::HSI_KER);

    loop {}
}
