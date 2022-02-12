#![cfg_attr(feature = "log-itm", allow(unsafe_code))]

cfg_if::cfg_if! {
    if #[cfg(any(feature = "log-itm"))] {
        use panic_itm as _;

        use lazy_static::lazy_static;
        use log::LevelFilter;

        pub use cortex_m_log::log::Logger;

        use cortex_m_log::{
            destination::Itm as ItmDest,
            printer::itm::InterruptSync,
            modes::InterruptFree,
            printer::itm::ItmSync
        };

        lazy_static! {
            static ref LOGGER: Logger<ItmSync<InterruptFree>> = Logger {
                level: LevelFilter::Info,
                inner: unsafe {
                    InterruptSync::new(
                        ItmDest::new(cortex_m::Peripherals::steal().ITM)
                    )
                },
            };
        }

        pub fn init() {
            cortex_m_log::log::init(&LOGGER).unwrap();
        }

    }
    else if #[cfg(any(feature = "log-rtt"))] {
        use panic_rtt_target as _;

        use log::{Level, Metadata, Record, LevelFilter};
        use rtt_target::{rprintln, rtt_init_print};

        pub struct Logger {
            level: Level,
        }

        static LOGGER: Logger = Logger {
            level: Level::Info,
        };

        pub fn init() {
            rtt_init_print!();
            log::set_logger(&LOGGER).map(|()| log::set_max_level(LevelFilter::Info)).unwrap();
        }

        impl log::Log for Logger {
            fn enabled(&self, metadata: &Metadata) -> bool {
                metadata.level() <= self.level

            }

            fn log(&self, record: &Record) {
                rprintln!("{} - {}", record.level(), record.args());
            }

            fn flush(&self) {}
        }
    }
    else if #[cfg(any(feature = "log-semihost"))] {
        use panic_semihosting as _;

        use lazy_static::lazy_static;
        use log::LevelFilter;

        pub use cortex_m_log::log::Logger;
        use cortex_m_log::printer::semihosting;
        use cortex_m_log::printer::semihosting::Semihosting;
        use cortex_m_log::modes::InterruptOk;
        use cortex_m_semihosting::hio::HStdout;

        lazy_static! {
            static ref LOGGER: Logger<Semihosting<InterruptOk, HStdout>> = Logger {
                level: LevelFilter::Info,
                inner: semihosting::InterruptOk::<_>::stdout().expect("Get Semihosting stdout"),
            };
        }

        pub fn init() {
            cortex_m_log::log::init(&LOGGER).unwrap();
        }
    }
    else {
        use panic_halt as _;
        pub fn init() {}
    }
}
