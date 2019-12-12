type_state! {
    SyncED, SyncDisabled, SyncEnabled
}

type_state! {
    EgED, EgDisabled, EgEnabled
}

bool_enum! {
    SyncOverrunInterrupt, "Synchronization overrun interrupt", Disabled, Enabled
}

int_enum! {
    SyncPolarity <=> u8,
    "Synchronization Polarity",
    NoEvent <=> 0b00,
    RisingEdge <=> 0b01,
    FallingEdge <=> 0b10,
    RisingFallingEdge <=> 0b11
}

int_struct! {
    NbReq, u8, 5, "Number of Requests"
}

int_enum! {
    SyncId <=> u8,
    "Synchronization Identification",
    DmaMux1Evt0 <=> 0b000,
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
            ReqNone,
            $(
                $request_id,
            )*
        };

        pub struct RequestIds {
            pub none: ReqNone,
            $(
                pub $name: $request_id,
            )*
        }

        impl RequestIds {
            const fn new() -> Self {
                RequestIds {
                    none: ReqNone,
                    $(
                        $name: $request_id::new(),
                    )*
                }
            }
        }

        int_enum! {
            RequestId <=> u8,
            "Request Id",
            None <=> 0,
            $(
                $request_id <=> $id
            ),*
        }

        pub mod request_ids {
            use super::RequestId as RequestIdEnum;

            pub unsafe trait RequestId {
                const REQUEST_ID: RequestIdEnum;

                fn request_id(&self) -> RequestIdEnum {
                    Self::REQUEST_ID
                }
            }

            #[derive(Clone, Copy)]
            pub struct ReqNone;

            unsafe impl RequestId for ReqNone {
                const REQUEST_ID: RequestIdEnum = RequestIdEnum::None;
            }

            pub unsafe trait RequestIdSome: RequestId {}

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

                unsafe impl RequestId for $request_id {
                    const REQUEST_ID: RequestIdEnum = RequestIdEnum::$request_id;
                }

                unsafe impl RequestIdSome for $request_id {}
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
