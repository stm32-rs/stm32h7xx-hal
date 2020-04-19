#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate panic_itm;

use cortex_m_rt::entry;
use stm32h7xx_hal::rcc::{rec, ResetEnable};
use stm32h7xx_hal::{pac, prelude::*};

fn enable_fdcan(rec: rec::Fdcan) {
    rec.enable().kernel_clk_mux(rec::FdcanClkSel::PLL1_Q);

    // rec is dropped here, and can never be changed again
}

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    // Constrain and Freeze power
    let pwr = dp.PWR.constrain();
    let vos = pwr.freeze();

    // Constrain and Freeze clock
    let rcc = dp.RCC.constrain();
    let prec = rcc.take_peripherals().unwrap();
    let _ccdr = rcc.sys_ck(100.mhz()).freeze(vos, &dp.SYSCFG);

    enable_fdcan(prec.FDCAN);

    //prec.FDCAN.disable(); // Compile error

    loop {}
}
