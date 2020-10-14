//! Power Configuration for examples

macro_rules! example_power {
    ($pwr:ident) => {{
        cfg_if::cfg_if! {
            if #[cfg(all(feature = "smps", feature = "example-smps"))] {
                $pwr.smps()
            } else if #[cfg(all(feature = "smps", feature = "example-ldo"))] {
                $pwr.ldo()
            } else {
                $pwr
            }
        }
    }};
}
