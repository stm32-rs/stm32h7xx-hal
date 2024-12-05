//! A module that can capture the RCC registers and read the reason why the mcu
//! has reset

use core::fmt::Display;

/// Gets and clears the reason of why the mcu was reset
#[rustfmt::skip]
pub fn get_reset_reason(rcc: &mut crate::stm32::RCC) -> ResetReason {
    let reset_reason = rcc.rsr().read();

    // Clear the register
    rcc.rsr().modify(|_, w| w.rmvf().clear_bit());

    #[cfg(not(feature = "rm0455"))]
    // See R0433 Rev 7 Section 8.4.4 Reset source identification
    match (
        reset_reason.lpwrrstf().is_reset_occurred(),
        reset_reason.wwdg1rstf().is_reset_occurred(),
        reset_reason.iwdg1rstf().is_reset_occurred(),
        reset_reason.sftrstf().is_reset_occurred(),
        reset_reason.porrstf().is_reset_occurred(),
        reset_reason.pinrstf().is_reset_occurred(),
        reset_reason.borrstf().is_reset_occurred(),
        reset_reason.d2rstf().is_reset_occurred(),
        reset_reason.d1rstf().is_reset_occurred(),
        reset_reason.cpurstf().is_reset_occurred(),
    ) {
        (false, false, false, false, true, true, true, true, true, true) => {
            ResetReason::PowerOnReset
        }
        (false, false, false, false, false, true, false, false, false, true) => {
            ResetReason::PinReset
        }
        (false, false, false, false, false, true, true, false, false, true) => {
            ResetReason::BrownoutReset
        }
        (false, false, false, true, false, true, false, false, false, true) => {
            ResetReason::SystemReset
        }
        (false, false, false, false, false, false, false, false, false, true) => {
            ResetReason::CpuReset
        }
        (false, true, false, false, false, false, false, false, false, false) | (false, true, false, false, false, true, false, false, false, true) => {
            ResetReason::WindowWatchdogReset
        }
        (false, false, true, false, false, true, false, false, false, true) => {
            ResetReason::IndependentWatchdogReset
        }
        (false, true, true, false, false, true, false, false, false, true) => {
            ResetReason::GenericWatchdogReset
        }
        (false, false, false, false, false, false, false, false, true, false) => {
            ResetReason::D1ExitsDStandbyMode
        }
        (false, false, false, false, false, false, false, true, false, false) => {
            ResetReason::D2ExitsDStandbyMode
        }
        (true, false, false, false, false, true, false, false, false, true) => {
            ResetReason::D1EntersDStandbyErroneouslyOrCpuEntersCStopErroneously
        }
        _ => ResetReason::Unknown {
            rcc_rsr: reset_reason.bits(),
        },
    }
    #[cfg(feature = "rm0455")]
    // See RM0455 Rev 6 Section 8.4.4 Reset source identification
    match (
        reset_reason.lpwrrstf().is_reset_occourred(),
        reset_reason.wwdgrstf().is_reset_occourred(),
        reset_reason.iwdgrstf().is_reset_occourred(),
        reset_reason.sftrstf().is_reset_occourred(),
        reset_reason.porrstf().is_reset_occourred(),
        reset_reason.pinrstf().is_reset_occourred(),
        reset_reason.borrstf().is_reset_occourred(),
        reset_reason.cdrstf().is_reset_occourred(),
    ) {
        (false, false, false, false, true, true, true, true) => {
            ResetReason::PowerOnReset
        }
        (false, false, false, false, false, true, false, false) => {
            ResetReason::PinReset
        }
        (false, false, false, false, false, true, true, false) => {
            ResetReason::BrownoutReset
        }
        (false, false, false, true, false, true, false, false) => {
            ResetReason::SystemReset
        }
        (false, true, false, false, false, true, false, false) => {
            ResetReason::WindowWatchdogReset
        }
        (false, false, true, false, false, true, false, false) => {
            ResetReason::IndependentWatchdogReset
        }
        (false, true, true, false, false, true, false, false) => {
            ResetReason::GenericWatchdogReset
        }
        (true, false, false, false, false, true, false, false) => {
            ResetReason::D1EntersDStandbyErroneouslyOrCpuEntersCStopErroneously
        }
        _ => ResetReason::Unknown {
            rcc_rsr: reset_reason.bits(),
        },
    }
}

/// Gives the reason why the mcu was reset
#[derive(Debug, Copy, Clone)]
pub enum ResetReason {
    /// The mcu went from not having power to having power and resetting
    PowerOnReset,
    /// The reset pin was asserted
    PinReset,
    /// The brownout detector triggered
    BrownoutReset,
    /// The software did a soft reset through the SCB peripheral
    SystemReset,
    /// The software did a soft reset through the RCC periperal
    CpuReset,
    /// The window watchdog triggered
    WindowWatchdogReset,
    /// The independent watchdog triggered
    IndependentWatchdogReset,
    /// Either of the two watchdogs triggered (but we don't know which one)
    GenericWatchdogReset,
    /// The DStandby mode was exited
    D1ExitsDStandbyMode,
    /// The DStandby mode was exited
    D2ExitsDStandbyMode,
    /// A state has been entered erroneously
    D1EntersDStandbyErroneouslyOrCpuEntersCStopErroneously,
    /// The reason could not be determined
    Unknown {
        /// The raw register value
        rcc_rsr: u32,
    },
}

impl Display for ResetReason {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            ResetReason::PowerOnReset => write!(f, "Power-on reset (pwr_por_rst)"),
            ResetReason::PinReset => write!(f, "Pin reset (NRST)"),
            ResetReason::BrownoutReset => write!(f, "Brownout reset (pwr_bor_rst)"),
            ResetReason::SystemReset => write!(f, "System reset generated by CPU (SFTRESET)"),
            ResetReason::CpuReset => write!(f, "CPU reset (CPURST)"),
            ResetReason::WindowWatchdogReset => write!(f, "WWDG1 reset (wwdg1_out_rst)"),
            ResetReason::IndependentWatchdogReset => write!(f, "IWDG1 reset (iwdg1_out_rst)"),
            ResetReason::GenericWatchdogReset => write!(f, "IWDG1 or WWDG1 reset"),
            ResetReason::D1ExitsDStandbyMode => write!(f, "D1 exits DStandby mode"),
            ResetReason::D2ExitsDStandbyMode => write!(f, "D2 exits DStandby mode"),
            ResetReason::D1EntersDStandbyErroneouslyOrCpuEntersCStopErroneously => write!(
                f,
                "D1 erroneously enters DStandby mode or CPU erroneously enters CStop mode"
            ),
            ResetReason::Unknown { rcc_rsr } => write!(
                f,
                "Could not determine the cause. RCC RSR bits were 0x{:X}",
                rcc_rsr
            ),
        }
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for ResetReason {
    fn format(&self, f: defmt::Formatter<'_>) {
        match self {
            ResetReason::PowerOnReset => defmt::intern!("Power-on reset (pwr_por_rst)").format(f),
            ResetReason::PinReset => defmt::intern!("Pin reset (NRST)").format(f),
            ResetReason::BrownoutReset => defmt::intern!("Brownout reset (pwr_bor_rst)").format(f),
            ResetReason::SystemReset => defmt::intern!("System reset generated by CPU (SFTRESET)").format(f),
            ResetReason::CpuReset => defmt::intern!("CPU reset (CPURST)").format(f),
            ResetReason::WindowWatchdogReset => defmt::intern!("WWDG1 reset (wwdg1_out_rst)").format(f),
            ResetReason::IndependentWatchdogReset => defmt::intern!("IWDG1 reset (iwdg1_out_rst)").format(f),
            ResetReason::GenericWatchdogReset => defmt::intern!("IWDG1 or WWDG1 reset").format(f),
            ResetReason::D1ExitsDStandbyMode => defmt::intern!("D1 exits DStandby mode").format(f),
            ResetReason::D2ExitsDStandbyMode => defmt::intern!("D2 exits DStandby mode").format(f),
            ResetReason::D1EntersDStandbyErroneouslyOrCpuEntersCStopErroneously => defmt::intern!(
                "D1 erroneously enters DStandby mode or CPU erroneously enters CStop mode"
            ).format(f),
            ResetReason::Unknown { rcc_rsr } => defmt::write!(
                f,
                "Could not determine the cause. RCC RSR bits were 0x{:X}",
                rcc_rsr
            ),
        }
    }
}
