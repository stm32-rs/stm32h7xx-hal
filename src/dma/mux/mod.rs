//! DMA Mux

pub mod request_gen;

use self::request_gen::RequestGenIsr;
use self::request_ids::{ReqNone, RequestId as IRequestId, RequestIdSome};
use super::utils::UniqueRef;
use super::ChannelId;
use crate::stm32::dmamux1::{CCR, CFR, CSR};
use core::convert::{TryFrom, TryInto};
use core::marker::PhantomData;

pub use self::request_gen::RequestGenerator;

/// DMA Mux
pub struct Mux<CXX, ReqId, SyncED, EgED>
where
    CXX: ChannelId,
    ReqId: IRequestId,
    SyncED: ISyncED,
    EgED: IEgED,
{
    /// This field *must not* be mutated using shared references
    rb: UniqueRef<'static, CCR>,
    req_id: ReqId,
    _phantom_data: PhantomData<(CXX, SyncED, EgED)>,
}

impl<CXX> Mux<CXX, ReqNone, SyncDisabled, EgDisabled>
where
    CXX: ChannelId,
{
    /// Creates an instance of a DMA Mux in initial state.
    ///
    /// Should only be called after RCC-reset of the DMA.
    pub(super) fn after_reset(rb: UniqueRef<'static, CCR>) -> Self {
        Mux {
            rb,
            req_id: ReqNone,
            _phantom_data: PhantomData,
        }
    }
}

impl<CXX, ReqId, SyncED, EgED> Mux<CXX, ReqId, SyncED, EgED>
where
    CXX: ChannelId,
    ReqId: IRequestId,
    SyncED: ISyncED,
    EgED: IEgED,
{
    /// Returns the id of the DMA Mux
    pub fn id(&self) -> usize {
        CXX::MUX_ID
    }

    /// Returns the request id assigned to this Mux
    pub fn request_id(&self) -> RequestId {
        debug_assert_eq!(
            ReqId::REQUEST_ID,
            RequestId::try_from(self.rb.read().dmareq_id().bits()).unwrap(),
            "DmaMux is in invalid state, because \
            `ReqId::REQUEST_ID` ({:?}) != Volatile request id ({:?})",
            ReqId::REQUEST_ID,
            RequestId::try_from(self.rb.read().dmareq_id().bits()).unwrap(),
        );

        ReqId::REQUEST_ID
    }

    /// Returns the Sync Overrun Interrupt config flag
    pub fn sync_overrun_interrupt(&self) -> SyncOverrunInterrupt {
        self.rb.read().soie().bit().into()
    }

    /// Sets the Sync Overrun Interrupt config flag
    pub fn set_sync_overrun_interrupt(
        &mut self,
        sync_overrun_intrpt: SyncOverrunInterrupt,
    ) {
        self.rb
            .modify(|_, w| w.soie().bit(sync_overrun_intrpt.into()));
    }

    /// Returns the Synchronization Polarity
    pub fn sync_polarity(&self) -> SyncPolarity {
        self.rb.read().spol().bits().try_into().unwrap()
    }

    /// Sets the Synchronization Polarity
    pub fn set_sync_polarity(&mut self, sync_polarity: SyncPolarity) {
        self.rb.modify(|_, w| w.spol().bits(sync_polarity.into()));
    }

    /// Returns the number of requests
    pub fn nbreq(&self) -> NbReq {
        self.rb.read().nbreq().bits().try_into().unwrap()
    }

    /// Returns the Synchronization ID
    pub fn sync_id(&self) -> SyncId {
        self.rb.read().sync_id().bits().try_into().unwrap()
    }

    /// Sets the Synchronization ID
    pub fn set_sync_id(&mut self, sync_id: SyncId) {
        unsafe {
            self.rb.modify(|_, w| w.sync_id().bits(sync_id.into()));
        }
    }

    /// Performs the request id write
    fn set_req_id_impl(&mut self, request_id: RequestId) {
        unsafe {
            self.rb.modify(|_, w| w.dmareq_id().bits(request_id.into()));
        }
    }

    /// Transmutes the state of the DMA Mux
    fn transmute<NewSyncED, NewEgED>(
        self,
    ) -> Mux<CXX, ReqId, NewSyncED, NewEgED>
    where
        NewSyncED: ISyncED,
        NewEgED: IEgED,
    {
        Mux {
            rb: self.rb,
            req_id: self.req_id,
            _phantom_data: PhantomData,
        }
    }
}

impl<CXX, ReqId> Mux<CXX, ReqId, SyncDisabled, EgDisabled>
where
    CXX: ChannelId,
    ReqId: IRequestId,
{
    /// Sets the number of requests
    pub fn set_nbreq(&mut self, nbreq: NbReq) {
        self.rb.modify(|_, w| w.nbreq().bits(nbreq.into()));
    }
}

impl<CXX, ReqId, EgED> Mux<CXX, ReqId, SyncDisabled, EgED>
where
    CXX: ChannelId,
    ReqId: IRequestId,
    EgED: IEgED,
{
    /// Enables synchronization
    pub fn enable_sync(self) -> Mux<CXX, ReqId, SyncEnabled, EgED> {
        self.rb.modify(|_, w| w.se().set_bit());

        self.transmute()
    }
}

impl<CXX, ReqId, EgED> Mux<CXX, ReqId, SyncEnabled, EgED>
where
    CXX: ChannelId,
    ReqId: IRequestId,
    EgED: IEgED,
{
    /// Disables synchronization
    pub fn disable_sync(self) -> Mux<CXX, ReqId, SyncDisabled, EgED> {
        self.rb.modify(|_, w| w.se().clear_bit());

        self.transmute()
    }
}

impl<CXX, ReqId, SyncED> Mux<CXX, ReqId, SyncED, EgDisabled>
where
    CXX: ChannelId,
    ReqId: IRequestId,
    SyncED: ISyncED,
{
    /// Enables event generation
    pub fn enable_event_gen(self) -> Mux<CXX, ReqId, SyncED, EgEnabled> {
        self.rb.modify(|_, w| w.ege().set_bit());

        self.transmute()
    }
}

impl<CXX, ReqId, SyncED> Mux<CXX, ReqId, SyncED, EgEnabled>
where
    CXX: ChannelId,
    ReqId: IRequestId,
    SyncED: ISyncED,
{
    /// Disables event generation
    pub fn disable_event_gen(self) -> Mux<CXX, ReqId, SyncED, EgDisabled> {
        self.rb.modify(|_, w| w.ege().clear_bit());

        self.transmute()
    }
}

impl<CXX, SyncED, EgED> Mux<CXX, ReqNone, SyncED, EgED>
where
    CXX: ChannelId,
    SyncED: ISyncED,
    EgED: IEgED,
{
    /// Sets request id
    pub fn set_req_id<NewReqId>(
        mut self,
        req_id: NewReqId,
    ) -> Mux<CXX, NewReqId, SyncED, EgED>
    where
        NewReqId: RequestIdSome,
    {
        self.set_req_id_impl(NewReqId::REQUEST_ID);

        Mux {
            req_id,
            rb: self.rb,
            _phantom_data: PhantomData,
        }
    }
}

impl<CXX, ReqId, SyncED, EgED> Mux<CXX, ReqId, SyncED, EgED>
where
    CXX: ChannelId,
    ReqId: RequestIdSome,
    SyncED: ISyncED,
    EgED: IEgED,
{
    /// Unsets request id, defaulting to `ReqNone` and returning the old one
    pub fn unset_req_id(mut self) -> (Mux<CXX, ReqNone, SyncED, EgED>, ReqId) {
        self.set_req_id_impl(ReqNone::REQUEST_ID);

        let old_req_id = self.req_id;
        let new_dma_mux = Mux {
            rb: self.rb,
            req_id: ReqNone,
            _phantom_data: PhantomData,
        };

        (new_dma_mux, old_req_id)
    }

    /// Replaces the request id
    pub fn replace_req_id<NewReqId>(
        mut self,
        req_id: NewReqId,
    ) -> (Mux<CXX, NewReqId, SyncED, EgED>, ReqId)
    where
        NewReqId: RequestIdSome,
    {
        self.set_req_id_impl(NewReqId::REQUEST_ID);

        let old_req_id = self.req_id;
        let new_dma_mux = Mux {
            req_id,
            rb: self.rb,
            _phantom_data: PhantomData,
        };

        (new_dma_mux, old_req_id)
    }
}

impl<CXX, ReqId, SyncED, EgED> Mux<CXX, ReqId, SyncED, EgED>
where
    CXX: ChannelId,
    ReqId: IRequestId,
    SyncED: ISyncED,
    EgED: IEgED,
{
    /// Checks the ISR for errors
    pub fn check_isr(&self, mux_isr: &MuxIsr) -> Result<(), OverrunError> {
        if self.is_sync_overrun(mux_isr) {
            Err(OverrunError)
        } else {
            Ok(())
        }
    }

    /// Returns the Sync Overrun flag
    pub fn is_sync_overrun(&self, mux_isr: &MuxIsr) -> bool {
        mux_isr.csr.read().sof0().bit_is_set()
    }

    /// Clears the ISR
    pub fn clear_isr(&self, mux_isr: &mut MuxIsr) {
        match self.id() {
            0 => mux_isr.cfr.write(|w| w.csof0().set_bit()),
            1 => mux_isr.cfr.write(|w| w.csof1().set_bit()),
            2 => mux_isr.cfr.write(|w| w.csof2().set_bit()),
            3 => mux_isr.cfr.write(|w| w.csof3().set_bit()),
            4 => mux_isr.cfr.write(|w| w.csof4().set_bit()),
            5 => mux_isr.cfr.write(|w| w.csof5().set_bit()),
            6 => mux_isr.cfr.write(|w| w.csof6().set_bit()),
            7 => mux_isr.cfr.write(|w| w.csof7().set_bit()),
            8 => mux_isr.cfr.write(|w| w.csof8().set_bit()),
            9 => mux_isr.cfr.write(|w| w.csof9().set_bit()),
            10 => mux_isr.cfr.write(|w| w.csof10().set_bit()),
            11 => mux_isr.cfr.write(|w| w.csof11().set_bit()),
            12 => mux_isr.cfr.write(|w| w.csof12().set_bit()),
            13 => mux_isr.cfr.write(|w| w.csof13().set_bit()),
            14 => mux_isr.cfr.write(|w| w.csof14().set_bit()),
            15 => mux_isr.cfr.write(|w| w.csof15().set_bit()),
            _ => unreachable!(),
        }
    }
}

unsafe impl<CXX, ReqId, SyncED, EgED> Send for Mux<CXX, ReqId, SyncED, EgED>
where
    CXX: ChannelId,
    ReqId: IRequestId,
    SyncED: ISyncED,
    EgED: IEgED,
{
}

unsafe impl<CXX, ReqId, SyncED, EgED> Sync for Mux<CXX, ReqId, SyncED, EgED>
where
    CXX: ChannelId,
    ReqId: IRequestId,
    SyncED: ISyncED,
    EgED: IEgED,
{
}

type_state! {
    ISyncED, SyncDisabled, SyncEnabled
}

type_state! {
    IEgED, EgDisabled, EgEnabled
}

bool_enum! {
    SyncOverrunInterrupt, "Synchronization overrun interrupt", Disabled (D), Enabled
}

int_enum! {
    SyncPolarity <=> u8,
    "Synchronization Polarity",
    NoEvent <=> 0b00 (D),
    RisingEdge <=> 0b01,
    FallingEdge <=> 0b10,
    RisingFallingEdge <=> 0b11
}

int_struct! {
    NbReq, u8, 5, "Number of Requests", 0
}

int_enum! {
    SyncId <=> u8,
    "Synchronization Identification",
    DmaMux1Evt0 <=> 0b000 (D),
    DmaMux1Evt1 <=> 0b001,
    DmaMux1Evt2 <=> 0b010,
    Lptim1Out <=> 0b011,
    Lptim2Out <=> 0b100,
    Lptim3Out <=> 0b101,
    Extit0 <=> 0b110,
    Tim12Trgo <=> 0b111
}

macro_rules! request_id {
    ($($request_id:ident => ($name:ident, $id:tt)),*) => {
        use request_ids::{
            $(
                $request_id,
            )*
        };

        pub struct RequestIds {
            $(
                pub $name: $request_id,
            )*
        }

        impl RequestIds {
            const fn new() -> Self {
                RequestIds {
                    $(
                        $name: $request_id::new(),
                    )*
                }
            }
        }

        int_enum! {
            RequestId <=> u8,
            "Request Id",
            None <=> 0 (D),
            $(
                $request_id <=> $id
            ),*
        }

        pub mod request_ids {
            use super::RequestId as RequestIdEnum;
            use crate::private;

            pub trait RequestId: Send + private::Sealed {
                const REQUEST_ID: RequestIdEnum;
            }

            #[derive(Clone, Copy)]
            pub struct ReqNone;

            impl private::Sealed for ReqNone {}
            impl RequestId for ReqNone {
                const REQUEST_ID: RequestIdEnum = RequestIdEnum::None;
            }

            pub trait RequestIdSome: RequestId + private::Sealed {}

            $(
                pub struct $request_id {
                    _priv: (),
                }

                impl $request_id {
                    pub(super) const fn new() -> Self {
                        $request_id {
                            _priv: (),
                        }
                    }
                }

                impl private::Sealed for $request_id {}
                impl RequestId for $request_id {
                    const REQUEST_ID: RequestIdEnum = RequestIdEnum::$request_id;
                }

                impl RequestIdSome for $request_id {}
            )*
        }
    };
}

request_id! {
    DmaMux1ReqGen0 => (dma_mux_1_req_gen_0, 1),
    DmaMux1ReqGen1 => (dma_mux_1_req_gen_1, 2),
    DmaMux1ReqGen2 => (dma_mux_1_req_gen_2, 3),
    DmaMux1ReqGen3 => (dma_mux_1_req_gen_3, 4),
    DmaMux1ReqGen4 => (dma_mux_1_req_gen_4, 5),
    DmaMux1ReqGen5 => (dma_mux_1_req_gen_5, 6),
    DmaMux1ReqGen6 => (dma_mux_1_req_gen_6, 7),
    DmaMux1ReqGen7 => (dma_mux_1_req_gen_7, 8),
    Adc1Dma => (adc_1_dma, 9),
    Adc2Dma => (adc_2_dma, 10),
    Tim1Ch1 => (tim_1_ch_1, 11),
    Tim1Ch2 => (tim_1_ch_2, 12),
    Tim1Ch3 => (tim_1_ch_3, 13),
    Tim1Ch4 => (tim_1_ch_4, 14),
    Tim1Up => (tim_1_up, 15),
    Tim1Trig => (tim_1_trig, 16),
    Tim1Com => (tim_1_com, 17),
    Tim2Ch1 => (tim_2_ch_1, 18),
    Tim2Ch2 => (tim_2_ch_2, 19),
    Tim2Ch3 => (tim_2_ch_3, 20),
    Tim2Ch4 => (tim_2_ch_4, 21),
    Tim2Up => (tim_2_up, 22),
    Tim3Ch1 => (tim_3_ch_1, 23),
    Tim3Ch2 => (tim_3_ch_2, 24),
    Tim3Ch3 => (tim_3_ch_3, 25),
    Tim3Ch4 => (tim_3_ch_4, 26),
    Tim3Up => (tim_3_up, 27),
    Tim3Trig => (tim_3_trig, 28),
    Tim4Ch1 => (tim_4_ch1, 29),
    Tim4Ch2 => (tim_4_ch2, 30),
    Tim4Ch3 => (tim_4_ch3, 31),
    Tim4Up => (tim_4_up, 32),
    I2c1RxDma => (i2c_1_rx_dma, 33),
    I2c1TxDma => (i2c_1_tx_dma, 34),
    I2c2RxDma => (i2c_2_rx_dma, 35),
    I2c2TxDma => (i2c_2_tx_dma, 36),
    Spi1RxDma => (spi_1_rx_dma, 37),
    Spi1TxDma => (spi_1_tx_dma, 38),
    Spi2RxDma => (spi_2_rx_dma, 39),
    Spi2TxDma => (spi_2_tx_dma, 40),
    Usart1RxDma => (usart_1_rx_dma, 41),
    Usart1TxDma => (usart_1_tx_dma, 42),
    Usart2RxDma => (usart_2_rx_dma, 43),
    Usart2TxDma => (usart_2_tx_dma, 44),
    Usart3RxDma => (usart_3_rx_dma, 45),
    Usart3TxDma => (usart_3_tx_dma, 46),
    Tim8Ch1 => (tim_8_ch_1, 47),
    Tim8Ch2 => (tim_8_ch_2, 48),
    Tim8Ch3 => (tim_8_ch_3, 49),
    Tim8Ch4 => (tim_8_ch_4, 50),
    Tim8Up => (tim_8_up, 51),
    Tim8Trig => (tim_8_trig, 52),
    Tim8Com => (tim_8_com, 53),
    // 54 Reserved
    Tim5Ch1 => (tim_5_ch_1, 55),
    Tim5Ch2 => (tim_5_ch_2, 56),
    Tim5Ch3 => (tim_5_ch_3, 57),
    Tim5Ch4 => (tim_5_ch_4, 58),
    Tim5Up => (tim_5_up, 59),
    Tim5Trig => (tim_5_trig, 60),
    Spi3RxDma => (spi_3_rx_dma, 61),
    Spi3TxDma => (spi_3_tx_dma, 62),
    Uart4RxDma => (uart_4_rx_dma, 63),
    Uart4TxDma => (uart_4_tx_dma, 64),
    Uart5RxDma => (uart_5_rx_dma, 65),
    Uart5TxDma => (uart_5_tx_dma, 66),
    DacCh1Dma => (dac_ch_1_dma, 67),
    DacCh2Dma => (dac_ch_2_dma, 68),
    Tim6Up => (tim_6_up, 69),
    Tim7Up => (tim_7_up, 70),
    Usart6RxDma => (usart_6_rx_dma, 71),
    Usart6TxDma => (usart_6_tx_dma, 72),
    I2c3RxDma => (i2c_3_rx_dma, 73),
    I2c3TxDma => (i2c_3_tx_dma, 74),
    DcmiDma => (dcmi_dma, 75),
    CrypInDma => (cryp_in_dma, 76),
    CrypOutDma => (cryp_out_dma, 77),
    HashInDma => (hash_in_dma, 78),
    Uart7RxDma => (uart_7_rx_dma, 79),
    Uart7TxDma => (uart_7_tx_dma, 80),
    Uart8RxDma => (uart_8_rx_dma, 81),
    Uart8TxDma => (uart_8_tx_dma, 82),
    Spi4RxDma => (spi_4_rx_dma, 83),
    Spi4TxDma => (spi_4_tx_dma, 84),
    Spi5RxDma => (spi_5_rx_dma, 85),
    Spi5TxDma => (spi_5_tx_dma, 86),
    Sai1aDma => (sai_1a_dma, 87),
    Sai1bDma => (sai_1b_dma, 88),
    Sai2aDma => (sai_2a_dma, 89),
    Sai2bDma => (sai_2b_dma, 90),
    SwpmiRxDma => (swpmi_rx_dma, 91),
    SwpmiTxDma => (swpmi_tx_dma, 92),
    SpdifrxDatDma => (spdifrx_dat_dma, 93),
    SpdifrxCtrlDma => (spdifrx_ctrl_dma, 94),
    HrReq1 => (hr_req_1, 95),
    HrReq2 => (hr_req_2, 96),
    HrReq3 => (hr_req_3, 97),
    HrReq4 => (hr_req_4, 98),
    HrReq5 => (hr_req_5, 99),
    HrReq6 => (hr_req_6, 100),
    Dfsdm1Dma0 => (dfsdm_1_dma_0, 101),
    Dfsdm1Dma1 => (dfsdm_1_dma_1, 102),
    Dfsdm1Dma2 => (dfsdm_1_dma_2, 103),
    Dfsdm1Dma3 => (dfsdm_1_dma_3, 104),
    Tim15Ch1 => (tim_15_ch_1, 105),
    Tim15Up => (tim_15_up, 106),
    Tim15Trig => (tim_15_trig, 107),
    Tim15Com => (tim_15_com, 108),
    Tim16Ch1 => (tim_16_ch_1, 109),
    Tim16Up => (tim_16_up, 110),
    Tim17Ch1 => (tim_17_ch_1, 111),
    Tim17Up => (tim_17_up, 112),
    Sai3aDma => (sai_3a_dma, 113),
    Sai3bDma => (sai_3b_dma, 114),
    Adc3Dma => (adc_3_dma, 115)
    // [116; 127] reserved
}

pub struct MuxShared {
    pub mux_isr: MuxIsr,
    pub req_gen_isr: RequestGenIsr,
    pub request_ids: RequestIds,
}

impl MuxShared {
    pub(super) fn new(mux_isr: MuxIsr, req_gen_isr: RequestGenIsr) -> Self {
        MuxShared {
            mux_isr,
            req_gen_isr,
            request_ids: RequestIds::new(),
        }
    }
}

pub struct MuxIsr {
    csr: &'static CSR,
    /// This field *must not* be mutated using shared references
    cfr: UniqueRef<'static, CFR>,
}

impl MuxIsr {
    pub(super) fn new(csr: &'static CSR, cfr: UniqueRef<'static, CFR>) -> Self {
        Self { csr, cfr }
    }
}

unsafe impl Send for MuxIsr {}
unsafe impl Sync for MuxIsr {}

pub struct OverrunError;
