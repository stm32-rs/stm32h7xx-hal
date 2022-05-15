//! # SD MultiMediaCard interface (SDMMC)
//!
//! For HDHC / SDXC / SDUC cards. SDSC cards are not supported.
//!
//! The H7 has two SDMMC peripherals, `SDMMC1` and `SDMMC2`.
//!
//! # Examples
//!
//! - [SDMMC example application](https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/sdmmc.rs)
//!
//! ## IO Setup
//!
//! For high speed signalling (bus clock > 16MHz), the IO speed needs to be
//! increased from the default.
//!
//! ```
//! use stm32h7xx_hal::gpio::Speed;
//!
//! let d0 = d0.set_speed(Speed::VeryHigh);
//! ```
//!
//! ## Usage
//!
//! By default the SDMMC bus clock is derived from the `pll1_q_ck`. This can be
//! set when initialising the RCC.
//!
//! ```
//! let ccdr = rcc
//!     .pll1_q_ck(100.MHz())
//!     .freeze(pwrcfg, &dp.SYSCFG);
//! ```
//!
//! There is an [extension trait](crate::sdmmc::SdmmcExt) implemented for the
//! `SDMMC1` and `SDMMC2` peripherals for easy initialisation.
//!
//! ```
//! // Create SDMMC
//! let mut sdmmc = dp.SDMMC1.sdmmc(
//!     (clk, cmd, d0, d1, d2, d3),
//!     ccdr.peripheral.SDMMC1,
//!     &ccdr.clocks,
//! );
//! ```
//!
//! The next step is to initialise a card. The bus speed is also set.
//!
//! ```
//! if let Err(err) = sdmmc.init(10.MHz()) {
//!     info!("Init err: {:?}", err);
//! }
//! ```
//!
//! The [`card()`](crate::sdmmc::Sdmmc::card) method returns useful information about
//! the card.
//!
//! ```
//! let card = sdmmc.card();
//! if let Some(card) = sdmmc.card() {
//!     info!("SD Card Connected: {:?}", card);
//! }
//! ```

// Adapted from stm32f4xx-hal
// https://github.com/stm32-rs/stm32f4xx-hal/blob/master/src/sdio.rs

use core::fmt;

use sdio_host::{
    common_cmd::{self, ResponseLen},
    emmc::{
        CardCapacity, CardStatus, CurrentState, ExtCSD, CID, CSD, EMMC, OCR,
        RCA,
    },
    emmc_cmd,
    sd::{SDStatus, CIC, SCR, SD},
    sd_cmd, Cmd,
};

use crate::time::Hertz;

use crate::gpio::{self, Alternate};
use crate::rcc::rec::{ResetEnable, SdmmcClkSelGetter};
use crate::rcc::{rec, CoreClocks};
use crate::stm32::{SDMMC1, SDMMC2};

pub trait PinClk<SDMMC> {}
pub trait PinCmd<SDMMC> {}
pub trait PinD0<SDMMC> {}
pub trait PinD1<SDMMC> {}
pub trait PinD2<SDMMC> {}
pub trait PinD3<SDMMC> {}
pub trait PinD4<SDMMC> {}
pub trait PinD5<SDMMC> {}
pub trait PinD6<SDMMC> {}
pub trait PinD7<SDMMC> {}

pub trait Pins<SDMMC> {
    const BUSWIDTH: Buswidth;
}

impl<SDMMC, CLK, CMD, D0, D1, D2, D3, D4, D5, D6, D7> Pins<SDMMC>
    for (CLK, CMD, D0, D1, D2, D3, D4, D5, D6, D7)
where
    CLK: PinClk<SDMMC>,
    CMD: PinCmd<SDMMC>,
    D0: PinD0<SDMMC>,
    D1: PinD1<SDMMC>,
    D2: PinD2<SDMMC>,
    D3: PinD3<SDMMC>,
    D4: PinD4<SDMMC>,
    D5: PinD5<SDMMC>,
    D6: PinD6<SDMMC>,
    D7: PinD7<SDMMC>,
{
    const BUSWIDTH: Buswidth = Buswidth::Eight;
}

impl<SDMMC, CLK, CMD, D0, D1, D2, D3> Pins<SDMMC> for (CLK, CMD, D0, D1, D2, D3)
where
    CLK: PinClk<SDMMC>,
    CMD: PinCmd<SDMMC>,
    D0: PinD0<SDMMC>,
    D1: PinD1<SDMMC>,
    D2: PinD2<SDMMC>,
    D3: PinD3<SDMMC>,
{
    const BUSWIDTH: Buswidth = Buswidth::Four;
}

impl<SDMMC, CLK, CMD, D0> Pins<SDMMC> for (CLK, CMD, D0)
where
    CLK: PinClk<SDMMC>,
    CMD: PinCmd<SDMMC>,
    D0: PinD0<SDMMC>,
{
    const BUSWIDTH: Buswidth = Buswidth::One;
}

macro_rules! pins {
    ($($SDMMCX:ty: CLK: [$($CLK:ty),*] CMD: [$($CMD:ty),*]
       D0: [$($D0:ty),*] D1: [$($D1:ty),*] D2: [$($D2:ty),*] D3: [$($D3:ty),*]
       D4: [$($D4:ty),*] D5: [$($D5:ty),*] D6: [$($D6:ty),*] D7: [$($D7:ty),*]
       CKIN: [$($CKIN:ty),*] CDIR: [$($CDIR:ty),*]
       D0DIR: [$($D0DIR:ty),*] D123DIR: [$($D123DIR:ty),*]
    )+) => {
        $(
            $(
                impl PinClk<$SDMMCX> for $CLK {}
            )*
                $(
                    impl PinCmd<$SDMMCX> for $CMD {}
                )*
                $(
                    impl PinD0<$SDMMCX> for $D0 {}
                )*
                $(
                    impl PinD1<$SDMMCX> for $D1 {}
                )*
                $(
                    impl PinD2<$SDMMCX> for $D2 {}
                )*
                $(
                    impl PinD3<$SDMMCX> for $D3 {}
                )*
                $(
                    impl PinD4<$SDMMCX> for $D4 {}
                )*
                $(
                    impl PinD5<$SDMMCX> for $D5 {}
                )*
                $(
                    impl PinD6<$SDMMCX> for $D6 {}
                )*
                $(
                    impl PinD7<$SDMMCX> for $D7 {}
                )*
        )+
    }
}

pins! {
    SDMMC1:
        CLK: [gpio::PC12<Alternate<12>>]
        CMD: [gpio::PD2<Alternate<12>>]
        D0: [gpio::PC8<Alternate<12>>]
        D1: [gpio::PC9<Alternate<12>>]
        D2: [gpio::PC10<Alternate<12>>]
        D3: [gpio::PC11<Alternate<12>>]
        D4: [gpio::PB8<Alternate<12>>]
        D5: [gpio::PB9<Alternate<12>>]
        D6: [gpio::PC6<Alternate<12>>]
        D7: [gpio::PC7<Alternate<12>>]
        CKIN: [gpio::PB8<Alternate<7>>]
        CDIR: [gpio::PB9<Alternate<7>>]
        D0DIR: [gpio::PC6<Alternate<8>>]
        D123DIR: [gpio::PC7<Alternate<8>>]
    SDMMC2:
        CLK: [gpio::PC1<Alternate<9>>, gpio::PD6<Alternate<11>>]
        CMD: [gpio::PA0<Alternate<9>>, gpio::PD7<Alternate<11>>]
        D0: [gpio::PB14<Alternate<9>>]
        D1: [gpio::PB15<Alternate<9>>]
        D2: [gpio::PB3<Alternate<9>>, gpio::PG11<Alternate<10>>]
        D3: [gpio::PB4<Alternate<9>>]
        D4: [gpio::PB8<Alternate<10>>]
        D5: [gpio::PB9<Alternate<10>>]
        D6: [gpio::PC6<Alternate<10>>]
        D7: [gpio::PC7<Alternate<10>>]
        CKIN: []
        CDIR: []
        D0DIR: []
        D123DIR: []
}

/// The signalling scheme used on the SDMMC bus
#[non_exhaustive]
#[allow(missing_docs)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Signalling {
    SDR12,
    SDR25,
    SDR50,
    SDR104,
    DDR50,
}
impl Default for Signalling {
    fn default() -> Self {
        Signalling::SDR12
    }
}

/// Possible bus widths
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Buswidth {
    One = 0,
    Four = 1,
    Eight = 2,
}

/// Errors
#[non_exhaustive]
#[allow(missing_docs)]
#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    Timeout,
    SoftwareTimeout,
    UnsupportedCardVersion,
    UnsupportedCardType,
    Crc,
    DataCrcFail,
    RxOverFlow,
    NoCard,
    BadClock,
    SignallingSwitchFailed,
}

#[derive(Clone, Copy, Default)]
/// SD Card
pub struct SdCard {
    /// The type of this card
    pub capacity: CardCapacity,
    /// Relative Card Address
    pub rca: RCA<SD>,
    /// Operation Conditions Register
    pub ocr: OCR<SD>,
    /// Card ID
    pub cid: CID<SD>,
    /// Card Specific Data
    pub csd: CSD<SD>,
    /// SD CARD Configuration Register
    pub scr: SCR,
    /// SD Status
    pub status: SDStatus,
}
impl SdCard {
    /// Size in bytes
    pub fn size(&self) -> u64 {
        self.csd.card_size()
    }
}

#[derive(Clone, Copy, Default)]
/// eMMC
pub struct Emmc {
    pub ocr: OCR<EMMC>,
    pub rca: RCA<EMMC>,
    pub cid: CID<EMMC>,
    pub csd: CSD<EMMC>,
    pub ext_csd: ExtCSD,
}

macro_rules! err_from_datapath_sm {
    ($status:ident) => {
        if $status.dcrcfail().bit() {
            return Err(Error::DataCrcFail);
        } else if $status.rxoverr().bit() {
            return Err(Error::RxOverFlow);
        } else if $status.dtimeout().bit() {
            return Err(Error::Timeout);
        }
    };
}

#[derive(Clone, Copy, Debug)]
/// Indicates transfer direction
enum Dir {
    CardToHost,
    HostToCard,
}

#[derive(Clone, Copy, Debug)]
/// SDMMC Power Control
enum PowerCtrl {
    Off = 0b00,
    On = 0b11,
}

/// SDMMC device
pub struct Sdmmc<SDMMC, P: SdmmcPeripheral> {
    sdmmc: SDMMC,
    /// SDMMC kernel clock
    ker_ck: Hertz,
    /// AHB clock
    hclk: Hertz,
    /// Data bus width
    bus_width: Buswidth,
    /// Current clock to card
    clock: Hertz,
    /// Current signalling scheme to card
    signalling: Signalling,
    /// Address (RCA) currently assigned to card. For eMMC cards this is
    /// assigned by us (the host), for SD cards it is only ever retrieved from
    /// the card response
    card_rca: u16,
    /// Card
    card: Option<P>,
}
impl<SDMMC, P: SdmmcPeripheral> fmt::Debug for Sdmmc<SDMMC, P> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("SDMMC Peripheral")
            .field("Card detected", &self.card.is_some())
            .field("Bus Width (bits)", &self.bus_width)
            .field("Signalling", &self.signalling)
            .field("Bus Clock", &self.clock)
            .finish()
    }
}

/// Extension trait for SDMMC peripherals
pub trait SdmmcExt<SDMMC, P: SdmmcPeripheral>: Sized {
    /// The `ResetEnable` singleton for this peripheral
    type Rec: ResetEnable;

    /// Create and enable the Sdmmc device. Initially the bus is clocked at
    /// <400kHz, so that SD cards can be initialised.
    fn sdmmc<PINS>(
        self,
        _pins: PINS,
        prec: Self::Rec,
        clocks: &CoreClocks,
    ) -> Sdmmc<SDMMC, P>
    where
        PINS: Pins<SDMMC>;

    /// Create and enable the Sdmmc device. Initially the bus is clocked
    /// <400kHz, so that SD cards can be initialised. `bus_width` is the bus
    /// width to configure on this interface.
    fn sdmmc_unchecked(
        self,
        bus_width: Buswidth,
        prec: Self::Rec,
        clocks: &CoreClocks,
    ) -> Sdmmc<SDMMC, P>;
}

impl<S, P: SdmmcPeripheral> Sdmmc<S, P> {
    /// Calculate clock divisor. Returns a SDMMC_CK less than or equal to
    /// `sdmmc_ck` in Hertz.
    ///
    /// Returns `(clk_div, clk_f)`, where `clk_div` is the divisor register
    /// value and `clk_f` is the resulting new clock frequency.
    fn clk_div(ker_ck: Hertz, sdmmc_ck: u32) -> Result<(u16, Hertz), Error> {
        match (ker_ck.raw() + sdmmc_ck - 1) / sdmmc_ck {
            0 | 1 => Ok((0, ker_ck)),
            x @ 2..=2046 => {
                let clk_div = ((x + 1) / 2) as u16;
                let clk = Hertz::from_raw(ker_ck.raw() / (clk_div as u32 * 2));

                Ok((clk_div, clk))
            }
            _ => Err(Error::BadClock),
        }
    }

    /// Returns a reference to the inner peripheral
    pub fn inner(&self) -> &S {
        &self.sdmmc
    }

    /// Returns a mutable reference to the inner peripheral
    pub fn inner_mut(&mut self) -> &mut S {
        &mut self.sdmmc
    }
}

macro_rules! sdmmc {
    ($($SDMMCX:ident: ($sdmmcX:ident, $Rec:ident),)+) => {
        $(
            impl<P: SdmmcPeripheral> SdmmcExt<$SDMMCX, P> for $SDMMCX {
                type Rec = rec::$Rec;

                fn sdmmc<PINS>(self, _pins: PINS,
                               prec: rec::$Rec,
                               clocks: &CoreClocks) -> Sdmmc<$SDMMCX, P>
                where
                    PINS: Pins<$SDMMCX>,
                {
                    Sdmmc::$sdmmcX(self, PINS::BUSWIDTH, prec, clocks)
                }

                fn sdmmc_unchecked(self, bus_width: Buswidth,
                                   prec: rec::$Rec,
                                   clocks: &CoreClocks) -> Sdmmc<$SDMMCX, P>
                {
                    Sdmmc::$sdmmcX(self, bus_width, prec, clocks)
                }
            }

            impl<P: SdmmcPeripheral> Sdmmc<$SDMMCX, P> {
                /// Sets the CLKDIV field in CLKCR. Updates clock field in self
                fn clkcr_set_clkdiv(
                    &mut self,
                    freq: u32,
                    width: Buswidth,
                ) -> Result<(), Error> {
                    let (clkdiv, new_clock) = Self::clk_div(self.ker_ck, freq)?;
                    // Enforce AHB and SDMMC_CK clock relation. See RM0433 Rev 7
                    // Section 55.5.8
                    let sdmmc_bus_bandwidth = new_clock.raw() * (width as u32);
                    debug_assert!(self.hclk.raw() > 3 * sdmmc_bus_bandwidth / 32);
                    self.clock = new_clock;

                    // CPSMACT and DPSMACT must be 0 to set CLKDIV
                    while self.sdmmc.star.read().dpsmact().bit_is_set()
                        || self.sdmmc.star.read().cpsmact().bit_is_set()
                    {}

                    self.sdmmc
                        .clkcr
                        .modify(|_, w| unsafe { w.clkdiv().bits(clkdiv) });

                    Ok(())
                }

                /// Initialise SDMMC peripheral
                pub fn $sdmmcX(
                    sdmmc: $SDMMCX,
                    bus_width: Buswidth,
                    prec: rec::$Rec,
                    clocks: &CoreClocks,
                ) -> Self {
                    // Enable and reset peripheral
                    let prec = prec.enable().reset();

                    let hclk = clocks.hclk();
                    let ker_ck = match prec.get_kernel_clk_mux() {
                        rec::SdmmcClkSel::Pll1Q => {
                            clocks.pll1_q_ck().expect(
                                concat!(stringify!($SDMMCX), ": PLL1_Q must be enabled")
                            )
                        }
                        rec::SdmmcClkSel::Pll2R => {
                            clocks.pll2_r_ck().expect(
                                concat!(stringify!($SDMMCX), ": PLL2_R must be enabled")
                            )
                        }
                    };


                    // For tuning the phase of the receive sampling clock, a
                    // DLYB block can be connected between sdmmc_io_in_ck and
                    // sdmmc_fb_ck

                    // While the SD/SDIO card or eMMC is in identification mode,
                    // the SDMMC_CK frequency must be less than 400 kHz.
                    let (clkdiv, clock) = Self::clk_div(ker_ck, 400_000)
                        .expect("SDMMC too slow. Cannot be generated from ker_ck");

                    // Configure clock
                    sdmmc.clkcr.write(|w| unsafe {
                        w.widbus()
                            .bits(0) // 1-bit wide bus
                            .clkdiv()
                            .bits(clkdiv)
                            .pwrsav() // power saving
                            .clear_bit()
                            .negedge() // dephasing selection bit
                            .clear_bit()
                            .hwfc_en() // hardware flow control
                            .set_bit()
                    });

                    // Power off, writen 00: Clock to the card is stopped;
                    // D[7:0], CMD, and CK are driven high.
                    sdmmc
                        .power
                        .modify(|_, w| unsafe { w.pwrctrl().bits(PowerCtrl::Off as u8) });

                    Sdmmc {
                        sdmmc,
                        ker_ck,
                        hclk,
                        bus_width,
                        clock,
                        signalling: Default::default(),
                        card_rca: 0,
                        card: None,
                    }

                    // drop prec: ker_ck can no longer be modified
                }

                fn power_card(&mut self, state : PowerCtrl) {
                    self.sdmmc
                        .power
                        .modify(|_, w| unsafe { w.pwrctrl().bits(state as u8) });

                }

                /// Get a reference to the initialized card
                ///
                /// # Errors
                ///
                /// Returns Error::NoCard if [`init_card`](#method.init_card)
                /// has not previously succeeded
                pub fn card(&self) -> Result<&P, Error> {
                    self.card.as_ref().ok_or(Error::NoCard)
                }

                /// Get the current SDMMC bus clock
                ///
                pub fn clock(&self) -> Hertz {
                    self.clock
                }

                /// Start a transfer
                fn start_datapath_transfer(
                    &self,
                    length_bytes: u32,
                    block_size: u8,
                    direction: Dir,
                ) {
                    assert!(block_size <= 14, "Block size up to 2^14 bytes");

                    // Block Size must be greater than 0 ( != 1 byte) in DDR mode
                    let ddr = self.sdmmc.clkcr.read().ddr().bit_is_set();
                    assert!(
                        !ddr || block_size != 0,
                        "Block size must be >= 1, or >= 2 in DDR mode"
                    );

                    let dtdir = match direction {
                        Dir::CardToHost => true,
                        _ => false,
                    };

                    // Command AND Data state machines must be idle
                    while self.sdmmc.star.read().dpsmact().bit_is_set()
                        || self.sdmmc.star.read().cpsmact().bit_is_set()
                    {}

                    // Data timeout, in bus cycles
                    self.sdmmc
                        .dtimer
                        .write(|w| unsafe { w.datatime().bits(5_000_000) });
                    // Data length, in bytes
                    self.sdmmc
                        .dlenr
                        .write(|w| unsafe { w.datalength().bits(length_bytes) });
                    // Transfer
                    self.sdmmc.dctrl.write(|w| unsafe {
                        w.dblocksize()
                            .bits(block_size) // 2^n bytes block size
                            .dtdir()
                            .bit(dtdir)
                            .dten()
                            .set_bit() // Enable transfer
                    });
                }

                /// Read block from card.
                ///
                /// `address` is the block address.
                pub fn read_block(
                    &mut self,
                    address: u32,
                    buffer: &mut [u8; 512],
                ) -> Result<(), Error> {
                    let _card = self.card()?;

                    self.cmd(common_cmd::set_block_length(512))?; // CMD16

                    // Setup read command
                    self.start_datapath_transfer(512, 9, Dir::CardToHost);
                    self.cmd(common_cmd::read_single_block(address))?;

                    let mut i = 0;
                    let mut status;
                    while {
                        status = self.sdmmc.star.read();
                        !(status.rxoverr().bit()
                          || status.dcrcfail().bit()
                          || status.dtimeout().bit()
                          || status.dataend().bit())
                    } {
                        if status.rxfifohf().bit() {
                            for _ in 0..8 {
                                let bytes = self.sdmmc.fifor.read().bits().to_le_bytes();
                                buffer[i..i + 4].copy_from_slice(&bytes);
                                i += 4;
                            }
                        }

                        if i >= buffer.len() {
                            break;
                        }
                    }

                    err_from_datapath_sm!(status);

                    Ok(())
                }

                /// Read mutliple blocks from card. The length of the buffer
                /// must be multiple of 512.
                ///
                /// `address` is the block address.
                pub fn read_blocks(
                    &mut self,
                    address: u32,
                    buffer: &mut [u8],
                ) -> Result<(), Error> {
                    let _card = self.card()?;

                    assert!(buffer.len() % 512 == 0,
                            "Buffer length must be a multiple of 512");
                    let n_blocks = buffer.len() / 512;
                    self.cmd(common_cmd::set_block_length(512))?; // CMD16

                    // Setup read command
                    self.start_datapath_transfer(512 * n_blocks as u32, 9, Dir::CardToHost);
                    self.cmd(common_cmd::read_multiple_blocks(address))?;

                    let mut i = 0;
                    let mut status;
                    while {
                        status = self.sdmmc.star.read();
                        !(status.rxoverr().bit()
                          || status.dcrcfail().bit()
                          || status.dtimeout().bit()
                          || status.dataend().bit())
                    } {
                        if status.rxfifohf().bit() {
                            for _ in 0..8 {
                                let bytes = self.sdmmc.fifor.read().bits().to_le_bytes();
                                buffer[i..i + 4].copy_from_slice(&bytes);
                                i += 4;
                            }
                        }

                        if i >= buffer.len() {
                            break;
                        }
                    }

                    self.cmd(common_cmd::stop_transmission())?; // CMD12

                    err_from_datapath_sm!(status);

                    Ok(())
                }

                /// Write block to card. Buffer must be 512 bytes
                pub fn write_block(
                    &mut self,
                    address: u32,
                    buffer: &[u8; 512]
                ) -> Result<(), Error> {
                    let _card = self.card()?;

                    self.cmd(common_cmd::set_block_length(512))?; // CMD16

                    // Setup write command
                    self.start_datapath_transfer(512, 9, Dir::HostToCard);
                    self.cmd(common_cmd::write_single_block(address))?; // CMD24

                    let mut i = 0;
                    let mut status;
                    while {
                        status = self.sdmmc.star.read();
                        !(status.txunderr().bit()
                          || status.dcrcfail().bit()
                          || status.dtimeout().bit()
                          || status.dataend().bit())
                    } {
                        if status.txfifohe().bit() {
                            for _ in 0..8 {
                                let mut wb = [0u8; 4];
                                wb.copy_from_slice(&buffer[i..i + 4]);
                                let word = u32::from_le_bytes(wb);
                                self.sdmmc.fifor.write(|w| unsafe { w.bits(word) });
                                i += 4;
                            }
                        }

                        if i >= buffer.len() {
                            break;
                        }
                    }

                    err_from_datapath_sm!(status);
                    self.clear_static_interrupt_flags();

                    let mut timeout: u32 = 0xFFFF_FFFF;

                    // Try to read card status (CMD13)
                    while timeout > 0 {
                        if self.card_ready()? {
                            return Ok(());
                        }
                        timeout -= 1;
                    }
                    Err(Error::SoftwareTimeout)
                }

                /// Query the card status (CMD13, returns R1)
                ///
                fn read_status(&self) -> Result<CardStatus<P>, Error> {
                    // CMD13
                    self.cmd(common_cmd::card_status(self.card_rca, false))?;

                    let r1 = self.sdmmc.resp1r.read().bits();
                    Ok(CardStatus::from(r1))
                }

                /// CMD13: Check if card is done writing/reading and back in transfer state
                fn card_ready(&mut self) -> Result<bool, Error> {
                    Ok(self.read_status()?.state() == CurrentState::Transfer)
                }

                /// CMD7: Select the card with `address` and place it into the _Tranfer State_
                ///
                /// When called with rca = 0, deselects the card (and ignores)
                fn select_card(&self, rca: u16) -> Result<(), Error> {
                    let r = self.cmd(common_cmd::select_card(rca));
                    match (r, rca) {
                        (Err(Error::Timeout), 0) => Ok(()),
                        _ => r,
                    }
                }

                fn app_cmd<R: common_cmd::Resp>(&self, acmd: Cmd<R>) -> Result<(), Error> {
                    self.cmd(common_cmd::app_cmd(self.card_rca))?;
                    self.cmd(acmd)
                }

                /// Clear "static flags" in interrupt clear register
                fn clear_static_interrupt_flags(&self) {
                    self.sdmmc.icr.modify(|_, w| {
                        w.dcrcfailc()
                            .set_bit()
                            .dtimeoutc()
                            .set_bit()
                            .txunderrc()
                            .set_bit()
                            .rxoverrc()
                            .set_bit()
                            .dataendc()
                            .set_bit()
                            .dholdc()
                            .set_bit()
                            .dbckendc()
                            .set_bit()
                            .dabortc()
                            .set_bit()
                            .idmatec()
                            .set_bit()
                            .idmabtcc()
                            .set_bit()
                    });
                }

                /// Send command to card
                fn cmd<R: common_cmd::Resp>(&self, cmd: Cmd<R>) -> Result<(), Error> {
                    // Clear interrupts
                    self.sdmmc.icr.modify(|_, w| {
                        w.ccrcfailc() // CRC FAIL
                            .set_bit()
                            .ctimeoutc() // TIMEOUT
                            .set_bit()
                            .cmdrendc() // CMD R END
                            .set_bit()
                            .cmdsentc() // CMD SENT
                            .set_bit()
                            .dataendc()
                            .set_bit()
                            .dbckendc()
                            .set_bit()
                            .dcrcfailc()
                            .set_bit()
                            .dtimeoutc()
                            .set_bit()
                            .sdioitc() // SDIO IT
                            .set_bit()
                            .rxoverrc()
                            .set_bit()
                            .txunderrc()
                            .set_bit()
                    });

                    // CP state machine must be idle
                    while self.sdmmc.star.read().cpsmact().bit_is_set() {}

                    // Command arg
                    self.sdmmc
                        .argr
                        .write(|w| unsafe { w.cmdarg().bits(cmd.arg) });

                    // Determine what kind of response the CPSM should wait for
                    let waitresp = match cmd.response_len() {
                        ResponseLen::Zero => 0,
                        ResponseLen::R48 => 1, // short response, expect CMDREND or CCRCFAIL
                        ResponseLen::R136 => 3, // long response, expect CMDREND or CCRCFAIL
                    };

                    // Special mode in CP State Machine
                    // CMD12: Stop Transmission
                    let cpsm_stop_transmission = (cmd.cmd == 12);

                    // Command index and start CP State Machine
                    self.sdmmc.cmdr.write(|w| unsafe {
                        w.waitint()
                            .clear_bit()
                            .waitresp() // No / Short / Long
                            .bits(waitresp)
                            .cmdstop() // CPSM Stop Transmission
                            .bit(cpsm_stop_transmission)
                            .cmdindex()
                            .bits(cmd.cmd)
                            .cpsmen()
                            .set_bit()
                    });

                    let mut timeout: u32 = 0xFFFF_FFFF;

                    let mut status;
                    if cmd.response_len() == ResponseLen::Zero {
                        // Wait for CMDSENT or a timeout
                        while {
                            status = self.sdmmc.star.read();
                            !(status.ctimeout().bit() || status.cmdsent().bit())
                                && timeout > 0
                        } {
                            timeout -= 1;
                        }
                    } else {
                        // Wait for CMDREND or CCRCFAIL or a timeout
                        while {
                            status = self.sdmmc.star.read();
                            !(status.ctimeout().bit()
                              || status.cmdrend().bit()
                              || status.ccrcfail().bit())
                                && timeout > 0
                        } {
                            timeout -= 1;
                        }
                    }

                    if status.ctimeout().bit_is_set() {
                        return Err(Error::Timeout);
                    } else if timeout == 0 {
                        return Err(Error::SoftwareTimeout);
                    } else if status.ccrcfail().bit() {
                        return Err(Error::Crc);
                    }

                    Ok(())
                }
            }

            impl Sdmmc<$SDMMCX, SdCard> {
                /// Initializes card (if present) and sets the bus at the
                /// specified frequency.
                pub fn init(&mut self, freq: impl Into<Hertz>) -> Result<(), Error> {
                    let freq = freq.into();

                    // Enable power to card
                    self.power_card(PowerCtrl::On);

                    // Send card to idle state
                    self.cmd(common_cmd::idle())?;

                    // Check if cards supports CMD8 (with pattern)
                    self.cmd(sd_cmd::send_if_cond(1, 0xAA))?;
                    let cic = CIC::from(self.sdmmc.resp1r.read().bits());

                    let mut card = if cic.pattern() == 0xAA {
                        // Card echoed back the pattern. Must be at least v2
                        SdCard::default()
                    } else {
                        return Err(Error::UnsupportedCardVersion);
                    };

                    let ocr = loop {
                        // Initialize card

                        // Host support: 3.2-3.3V
                        let voltage_window = 1 << 5;

                        // ACMD41
                        match self.app_cmd(sd_cmd::sd_send_op_cond(true, false, true, voltage_window)) {
                            Ok(_) => (),
                            Err(Error::Crc) => (),
                            Err(err) => return Err(err),
                        }
                        let ocr = OCR::from(self.sdmmc.resp1r.read().bits());
                        if !ocr.is_busy() {
                            // Power up done
                            break ocr;
                        }
                    };

                    if ocr.high_capacity() {
                        // Card is SDHC or SDXC or SDUC
                        card.capacity = CardCapacity::HighCapacity;
                    } else {
                        return Err(Error::UnsupportedCardType);
                    }
                    card.ocr = ocr;

                    // Get CID
                    self.cmd(common_cmd::all_send_cid())?; // CMD2
                    let cid = ((self.sdmmc.resp1r.read().bits() as u128) << 96)
                        | ((self.sdmmc.resp2r.read().bits() as u128) << 64)
                        | ((self.sdmmc.resp3r.read().bits() as u128) << 32)
                        | self.sdmmc.resp4r.read().bits() as u128;
                    card.cid = cid.into();

                    // Get RCA
                    self.cmd(sd_cmd::send_relative_address())?;
                    card.rca = RCA::from(self.sdmmc.resp1r.read().bits());

                    // Get CSD
                    self.cmd(common_cmd::send_csd(card.rca.address()))?;
                    let csd = ((self.sdmmc.resp1r.read().bits() as u128) << 96)
                        | ((self.sdmmc.resp2r.read().bits() as u128) << 64)
                        | ((self.sdmmc.resp3r.read().bits() as u128) << 32)
                        | self.sdmmc.resp4r.read().bits() as u128;
                    card.csd = csd.into();

                    // Select and get RCA
                    self.select_card(card.rca.address())?;
                    card.scr = self.get_scr(card.rca.address())?;

                    // Replace
                    let _ = self.card.replace(card);
                    // self.app_cmd will now select this card
                    self.card_rca = card.rca.address();

                    // Set bus width
                    let (width, acmd_arg) = match self.bus_width {
                        Buswidth::Eight => unimplemented!(),
                        Buswidth::Four if card.scr.bus_width_four() => (Buswidth::Four, 2),
                        _ => (Buswidth::One, 0),
                    };
                    self.app_cmd(sd_cmd::cmd6(acmd_arg))?; // ACMD6: Bus Width

                    // CPSMACT and DPSMACT must be 0 to set WIDBUS
                    while self.sdmmc.star.read().dpsmact().bit_is_set()
                        || self.sdmmc.star.read().cpsmact().bit_is_set()
                    {}
                    self.sdmmc.clkcr.modify(|_, w| unsafe {
                        w.widbus().bits(match width {
                            Buswidth::One => 0,
                            Buswidth::Four => 1,
                            Buswidth::Eight => 2,
                        })
                    });

                    // Set Clock
                    if freq.raw() <= 25_000_000 {
                        // Final clock frequency
                        self.clkcr_set_clkdiv(freq.raw(), width)?;
                    } else {
                        // Switch to max clock for SDR12
                        self.clkcr_set_clkdiv(25_000_000, width)?;
                    }

                    // Read status
                    self.read_sd_status()?;

                    if freq.raw() > 25_000_000 {
                        // Switch to SDR25
                        self.signalling = self.switch_signalling_mode(Signalling::SDR25)?;

                        if self.signalling == Signalling::SDR25 {
                            // Set final clock frequency
                            self.clkcr_set_clkdiv(freq.raw(), width)?;

                            if !self.card_ready()? {
                                return Err(Error::SignallingSwitchFailed);
                            }
                        }
                    }

                    // Read status after signalling change
                    self.read_sd_status()?;

                    Ok(())
                }

                /// Reads the SD Status (ACMD13)
                ///
                fn read_sd_status(&mut self) -> Result<(), Error> {
                    self.cmd(common_cmd::set_block_length(64))?; // CMD16

                    // Prepare the transfer
                    self.start_datapath_transfer(64, 6, Dir::CardToHost);
                    self.app_cmd(common_cmd::card_status(self.card_rca, false))?; // ACMD13

                    let mut status = [0u32; 16];
                    let mut idx = 0;
                    let mut sta_reg;
                    while {
                        sta_reg = self.sdmmc.star.read();
                        !(sta_reg.rxoverr().bit()
                          || sta_reg.dcrcfail().bit()
                          || sta_reg.dtimeout().bit()
                          || sta_reg.dbckend().bit())
                    } {
                        if sta_reg.rxfifohf().bit() {
                            for _ in 0..8 {
                                status[15-idx] = u32::from_be(
                                    self.sdmmc.fifor.read().bits());

                                idx += 1;
                            }
                        }

                        if idx == status.len() {
                            break;
                        }
                    }

                    err_from_datapath_sm!(sta_reg);

                    let card = self.card.as_mut().ok_or(Error::NoCard)?;
                    card.status = status.into();

                    Ok(())
                }

                /// Get SD CARD Configuration Register (SCR)
                fn get_scr(&self, rca: u16) -> Result<SCR, Error> {
                    // Read the the 64-bit SCR register
                    self.cmd(common_cmd::set_block_length(8))?; // CMD16
                    self.cmd(common_cmd::app_cmd(rca))?;

                    self.start_datapath_transfer(8, 3, Dir::CardToHost);
                    self.cmd(sd_cmd::send_scr())?;

                    let mut scr = [0; 2];
                    let mut i = 0;
                    let mut status;
                    while {
                        status = self.sdmmc.star.read();

                        !(status.rxoverr().bit()
                          || status.dcrcfail().bit()
                          || status.dtimeout().bit()
                          || status.dbckend().bit())
                    } {
                        if status.rxfifoe().bit_is_clear() {
                            // FIFO not empty
                            scr[i] = self.sdmmc.fifor.read().bits();
                            i += 1;
                        }

                        if i == 2 {
                            break;
                        }
                    }

                    err_from_datapath_sm!(status);

                    Ok(SCR::from(scr))
                }

                /// Switch mode using CMD6.
                ///
                /// Attempt to set a new signalling mode. The selected
                /// signalling mode is returned. Expects the current clock
                /// frequency to be > 12.5MHz.
                fn switch_signalling_mode(
                    &self,
                    signalling: Signalling,
                ) -> Result<Signalling, Error> {
                    // NB PLSS v7_10 4.3.10.4: "the use of SET_BLK_LEN command is not
                    // necessary"

                    let set_function = 0x8000_0000
                        | match signalling {
                            // See PLSS v7_10 Table 4-11
                            Signalling::DDR50 => 0xFF_FF04,
                            Signalling::SDR104 => 0xFF_1F03,
                            Signalling::SDR50 => 0xFF_1F02,
                            Signalling::SDR25 => 0xFF_FF01,
                            Signalling::SDR12 => 0xFF_FF00,
                        };

                    // Prepare the transfer
                    self.start_datapath_transfer(64, 6, Dir::CardToHost);
                    self.cmd(sd_cmd::cmd6(set_function))?; // CMD6

                    let mut status = [0u32; 16];
                    let mut idx = 0;
                    let mut sta_reg;
                    while {
                        sta_reg = self.sdmmc.star.read();
                        !(sta_reg.rxoverr().bit()
                          || sta_reg.dcrcfail().bit()
                          || sta_reg.dtimeout().bit()
                          || sta_reg.dbckend().bit())
                    } {
                        if sta_reg.rxfifohf().bit() {
                            for _ in 0..8 {
                                status[idx] = self.sdmmc.fifor.read().bits();
                                idx += 1;
                            }
                        }

                        if idx == status.len() {
                            break;
                        }
                    }

                    err_from_datapath_sm!(sta_reg);

                    // Host is allowed to use the new functions at least 8
                    // clocks after the end of the switch command
                    // transaction. We know the current clock period is < 80ns,
                    // so a total delay of 640ns is required here
                    for _ in 0..300 {
                        cortex_m::asm::nop();
                    }

                    // Support Bits of Functions in Function Group 1
                    let _support_bits = u32::from_be(status[3]) >> 16;
                    // Function Selection of Function Group 1
                    let selection = (u32::from_be(status[4]) >> 24) & 0xF;

                    match selection {
                        0 => Ok(Signalling::SDR12),
                        1 => Ok(Signalling::SDR25),
                        2 => Ok(Signalling::SDR50),
                        3 => Ok(Signalling::SDR104),
                        4 => Ok(Signalling::DDR50),
                        _ => Err(Error::UnsupportedCardType),
                    }
                }

                #[cfg(feature = "sdmmc-fatfs")]
                pub fn sdmmc_block_device(self) -> SdmmcBlockDevice<Sdmmc<$SDMMCX, SdCard>> {
                    SdmmcBlockDevice {
                        sdmmc: core::cell::RefCell::new(self)
                    }
                }
            }

            #[cfg(feature = "sdmmc-fatfs")]
            impl embedded_sdmmc::BlockDevice for SdmmcBlockDevice<Sdmmc<$SDMMCX, SdCard>> {
                type Error = Error;

                fn read(
                    &self,
                    blocks: &mut [embedded_sdmmc::Block],
                    start_block_idx: embedded_sdmmc::BlockIdx,
                    _reason: &str,
                ) -> Result<(), Self::Error> {
                    let start = start_block_idx.0;
                    let mut sdmmc = self.sdmmc.borrow_mut();
                    for block_idx in start..(start + blocks.len() as u32) {
                        sdmmc.read_block(
                            block_idx,
                            &mut blocks[(block_idx - start) as usize].contents,
                        )?;
                    }
                    Ok(())
                }

                fn write(
                    &self,
                    blocks: &[embedded_sdmmc::Block],
                    start_block_idx: embedded_sdmmc::BlockIdx,
                ) -> Result<(), Self::Error> {
                    let start = start_block_idx.0;
                    let mut sdmmc = self.sdmmc.borrow_mut();
                    for block_idx in start..(start + blocks.len() as u32) {
                        sdmmc.write_block(block_idx, &blocks[(block_idx - start) as usize].contents)?;
                    }
                    Ok(())
                }

                fn num_blocks(&self) -> Result<embedded_sdmmc::BlockCount, Self::Error> {
                    let sdmmc = self.sdmmc.borrow_mut();
                    Ok(embedded_sdmmc::BlockCount((sdmmc.card()?.size() / 512u64) as u32))
                }

            }

            impl Sdmmc<$SDMMCX, Emmc> {
                /// Reads the Extended CSD register CMD8
                ///
                pub fn read_extended_csd(&mut self) -> Result<ExtCSD, Error> {
                    // start transfer
                    self.start_datapath_transfer(512, 9, Dir::CardToHost);
                    self.cmd(emmc_cmd::send_ext_csd())?; // CMD8

                    let mut buffer = [0u32; 128];
                    let mut idx = 0;
                    let mut sta_reg;
                    while {
                        sta_reg = self.sdmmc.star.read();
                        !(sta_reg.rxoverr().bit()
                          || sta_reg.dcrcfail().bit()
                          || sta_reg.dtimeout().bit()
                          || sta_reg.dbckend().bit())
                    } {
                        if sta_reg.rxfifohf().bit() {
                            for _ in 0..8 {
                                buffer[idx] = u32::from_be(
                                    self.sdmmc.fifor.read().bits());

                                idx += 1;
                            }
                        }

                        if idx == buffer.len() {
                            break;
                        }
                    }

                    err_from_datapath_sm!(sta_reg);

                    // wait for card to be ready again
                    while !self.card_ready()? {}

                    Ok(ExtCSD::from(buffer))
                }

                /// Initializes eMMC device (if present) and sets the bus at the specified frequency
                pub fn init(&mut self, freq: impl Into<Hertz>) -> Result<(), Error> {
                    let card_addr: RCA<EMMC> = RCA::from(1u16);

                    // Enable power to card
                    self.power_card(PowerCtrl::On);

                    // Enable clock - Already?

                    // CMD0: Send card to idle state
                    self.cmd(common_cmd::idle())?;

                    let ocr = loop {
                        // Initialize card

                        // 3.2-3.3V
                        match self.cmd(emmc_cmd::send_op_cond(0b01000000111111111000000000000000)) {
                            Ok(_) => (),
                            Err(Error::Crc) => (),
                            Err(err) => return Err(err),
                        };
                        let ocr = OCR::from(self.sdmmc.resp1r.read().bits());
                        if !ocr.is_busy() {
                            break ocr;
                        }
                    };

                    // CMD2: Get CID
                    self.cmd(common_cmd::all_send_cid())?;
                    let mut cid = [0; 4];
                    cid[3] = self.sdmmc.resp1r.read().bits();
                    cid[2] = self.sdmmc.resp2r.read().bits();
                    cid[1] = self.sdmmc.resp3r.read().bits();
                    cid[0] = self.sdmmc.resp4r.read().bits();
                    let cid = CID::from(cid);

                    // CMD3: Assign address
                    self.cmd(emmc_cmd::assign_relative_address(card_addr.address()))?;
                    self.card_rca = card_addr.address();

                    // CMD9: Send CSD
                    self.cmd(common_cmd::send_csd(self.card_rca))?;

                    let mut csd = [0; 4];
                    csd[3] = self.sdmmc.resp1r.read().bits();
                    csd[2] = self.sdmmc.resp2r.read().bits();
                    csd[1] = self.sdmmc.resp3r.read().bits();
                    csd[0] = self.sdmmc.resp4r.read().bits();
                    let csd = CSD::from(csd);

                    // CMD7: Place card in the transfer state
                    self.select_card(self.card_rca)?;

                    while !self.card_ready()? {}

                    // Get extended CSD
                    let ext_csd = self.read_extended_csd()?;

                    let card = Emmc {
                        ocr,
                        rca: card_addr,
                        cid,
                        csd,
                        ext_csd,
                    };
                    self.card.replace(card);

                    self.set_bus(self.bus_width, freq)?;
                    Ok(())
                }

                pub fn set_bus(&mut self, width: Buswidth, freq: impl Into<Hertz>) -> Result<(), Error> {
                    // Use access mode 0b11 to write a value of 0x02 to byte
                    // 183. Cmd Set is 0 (not used).
                    self.cmd(emmc_cmd::modify_ext_csd(
                        emmc_cmd::AccessMode::WriteByte,
                        183,
                        width as u8,
                    ))?;

                    // CMD6 is R1b, so wait for the card to be ready again
                    // before proceeding.
                    while !self.card_ready()? {}

                    self.sdmmc.clkcr.modify(|_, w| unsafe {
                        w.widbus().bits(match width {
                            Buswidth::One => 0,
                            Buswidth::Four => 1,
                            Buswidth::Eight => 2,
                        })
                    });
                    // Set CLKDIV
                    self.clkcr_set_clkdiv(freq.into().raw(), width)?;

                    Ok(())
                }
            }

        )+
    };
}

sdmmc! {
    SDMMC1: (sdmmc1, Sdmmc1),
    SDMMC2: (sdmmc2, Sdmmc2),
}

impl SdmmcPeripheral for SdCard {
    fn get_address(&self) -> u16 {
        self.rca.address()
    }
    fn get_capacity(&self) -> CardCapacity {
        self.capacity
    }
}

impl SdmmcPeripheral for Emmc {
    fn get_address(&self) -> u16 {
        self.rca.address()
    }
    fn get_capacity(&self) -> CardCapacity {
        CardCapacity::HighCapacity
    }
}

pub trait SdmmcPeripheral {
    fn get_address(&self) -> u16;
    fn get_capacity(&self) -> CardCapacity;
}

#[cfg(feature = "sdmmc-fatfs")]
pub struct SdmmcBlockDevice<SDMMC> {
    sdmmc: core::cell::RefCell<SDMMC>,
}

#[cfg(feature = "sdmmc-fatfs")]
impl<SDMMC> SdmmcBlockDevice<SDMMC> {
    pub fn free(self) -> SDMMC {
        self.sdmmc.into_inner()
    }
}
