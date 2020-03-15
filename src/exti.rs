//! External interrupt controller
use crate::stm32::EXTI;

/// EXTI trigger event
#[derive(PartialEq, PartialOrd, Clone, Copy)]
pub enum Event {
    GPIO0 = 0,
    GPIO1 = 1,
    GPIO2 = 2,
    GPIO3 = 3,
    GPIO4 = 4,
    GPIO5 = 5,
    GPIO6 = 6,
    GPIO7 = 7,
    GPIO8 = 8,
    GPIO9 = 9,
    GPIO10 = 10,
    GPIO11 = 11,
    GPIO12 = 12,
    GPIO13 = 13,
    GPIO14 = 14,
    GPIO15 = 15,
    PVD = 16,
    RTC_ALARM = 17,
    RTC_OTHER = 18,
    RTC_WAKEUP = 19,
    COMP1 = 20,
    COMP2 = 21,
    I2C1 = 22,
    I2C2 = 23,
    I2C3 = 24,
    I2C4 = 25,
    USART1 = 26,
    USART2 = 27,
    USART3 = 28,
    USART6 = 29,
    UART4 = 30,
    UART5 = 31,
    UART7 = 32,
    UART8 = 33,
    LPUART1_RX = 34,
    LPUART1_TX = 35,
    SPI1_WAKEUP = 36,
    SPI2_WAKEUP = 37,
    SPI3_WAKEUP = 38,
    SPI4_WAKEUP = 39,
    SPI5_WAKEUP = 40,
    SPI6_WAKEUP = 41,
    MDIO = 42,
    USB1 = 43,
    USB2 = 44,
    LPTIM1 = 47,
    LPTIM2 = 48,
    LPTIM2_OUTPUT = 49,
    LPTIM3 = 50,
    LPTIM3_OUTPUT = 51,
    LPTIM4 = 52,
    LPTIM5 = 53,
    SWPMI = 54,
    WKUP1 = 55,
    WKUP2 = 56,
    WKUP3 = 57,
    WKUP4 = 58,
    WKUP5 = 59,
    WKUP6 = 60,
    RCC = 61,
    I2C4_EVENT = 62,
    I2C4_ERROR = 63,
    LPUART1 = 64,
    SPI6 = 65,
    BDMA_CH0 = 66,
    BDMA_CH1 = 67,
    BDMA_CH2 = 68,
    BDMA_CH3 = 69,
    BDMA_CH4 = 70,
    BDMA_CH5 = 71,
    BDMA_CH6 = 72,
    BDMA_CH7 = 73,
    DMAMUX2 = 74,
    ADC3 = 75,
    SAI4 = 76,
    HSEM0 = 77,
    HSEM1 = 78,
    CM7_SEV = 79,
    CM4_SEV = 80,
    WWDG1 = 82,
    WWDG2 = 84,
    HDMI_CEC = 85,
    ETHERNET = 86,
    HSECSS = 87,
}

/// Return an EXTI register for the current CPU
#[cfg(any(feature = "singlecore"))]
macro_rules! reg_for_cpu {
    ($self:ident, imr1) => {
        $self.cpuimr1
    };
    ($self:ident, imr2) => {
        $self.cpuimr2
    };
    ($self:ident, imr3) => {
        $self.cpuimr3
    };
    ($self:ident, emr1) => {
        $self.cpuemr1
    };
    ($self:ident, emr2) => {
        $self.cpuemr2
    };
    ($self:ident, emr3) => {
        $self.cpuemr3
    };
    ($self:ident, pr1) => {
        $self.cpupr1
    };
    ($self:ident, pr2) => {
        $self.cpupr2
    };
    ($self:ident, pr3) => {
        $self.cpupr3
    };
}

#[cfg(all(feature = "dualcore", feature = "cm7"))]
macro_rules! reg_for_cpu {
    ($self:ident, imr1) => {
        $self.c1imr1
    };
    ($self:ident, imr2) => {
        $self.c1imr2
    };
    ($self:ident, imr3) => {
        $self.c1imr3
    };
    ($self:ident, emr1) => {
        $self.c1emr1
    };
    ($self:ident, emr2) => {
        $self.c1emr2
    };
    ($self:ident, emr3) => {
        $self.c1emr3
    };
    ($self:ident, pr1) => {
        $self.c1pr1
    };
    ($self:ident, pr2) => {
        $self.c1pr2
    };
    ($self:ident, pr3) => {
        $self.c1pr3
    };
}

#[cfg(all(feature = "dualcore", feature = "cm4"))]
macro_rules! reg_for_cpu {
    ($self:ident, imr1) => {
        $self.c2imr1
    };
    ($self:ident, imr2) => {
        $self.c2imr2
    };
    ($self:ident, imr3) => {
        $self.c2imr3
    };
    ($self:ident, emr1) => {
        $self.c2emr1
    };
    ($self:ident, emr2) => {
        $self.c2emr2
    };
    ($self:ident, emr3) => {
        $self.c2emr3
    };
    ($self:ident, pr1) => {
        $self.c2pr1
    };
    ($self:ident, pr2) => {
        $self.c2pr2
    };
    ($self:ident, pr3) => {
        $self.c2pr3
    };
}

pub trait ExtiExt {
    fn listen(&self, ev: Event);
    fn unlisten(&self, ev: Event);
    fn is_pending(&self, ev: Event) -> bool;
    fn unpend(&self, ev: Event);
}

impl ExtiExt for EXTI {
    /// CPU Interrupt Enable
    fn listen(&self, ev: Event) {
        let line = ev as u8;

        unsafe {
            match line {
                0..=31 => reg_for_cpu!(self, imr1)
                    .modify(|r, w| w.bits(r.bits() | (1 << line))),
                32..=44 | 46..=63 => reg_for_cpu!(self, imr2)
                    .modify(|r, w| w.bits(r.bits() | (1 << (line - 32)))),
                64..=80 | 82 | 84..=88 => reg_for_cpu!(self, imr3)
                    .modify(|r, w| w.bits(r.bits() | (1 << (line - 64)))),
                _ => {}
            }
        }
    }

    /// CPU Interrupt Disable
    fn unlisten(&self, ev: Event) {
        let line = ev as u8;

        unsafe {
            match line {
                0..=31 => reg_for_cpu!(self, imr1)
                    .modify(|r, w| w.bits(r.bits() & !(1 << line))),
                32..=44 | 46..=63 => reg_for_cpu!(self, imr2)
                    .modify(|r, w| w.bits(r.bits() & !(1 << (line - 32)))),
                64..=80 | 82 | 84..=88 => reg_for_cpu!(self, imr3)
                    .modify(|r, w| w.bits(r.bits() & !(1 << (line - 64)))),
                _ => {}
            }
        }
    }

    /// Indicate if the interrupt is currently pending
    ///
    /// Configurable events only
    fn is_pending(&self, ev: Event) -> bool {
        let line = ev as u8;

        match line {
            0..=19 | 20 | 21 => {
                reg_for_cpu!(self, pr1).read().bits() & (1 << line) != 0
            }
            49 | 51 => {
                reg_for_cpu!(self, pr2).read().bits() & (1 << (line - 32)) != 0
            }
            82 | 84 | 85 | 86 => {
                reg_for_cpu!(self, pr3).read().bits() & (1 << (line - 64)) != 0
            }
            _ => false,
        }
    }

    /// Clear interrupt and pending flag
    ///
    /// Configurable events only
    fn unpend(&self, ev: Event) {
        let line = ev as u8;

        unsafe {
            match line {
                0..=19 | 20 | 21 => {
                    reg_for_cpu!(self, pr1).write(|w| w.bits(1 << line))
                }
                49 | 51 => {
                    reg_for_cpu!(self, pr2).write(|w| w.bits(1 << (line - 32)))
                }
                82 | 84 | 85 | 86 => {
                    reg_for_cpu!(self, pr3).write(|w| w.bits(1 << (line - 64)))
                }
                _ => {}
            }
        }
    }
}
