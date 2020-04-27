//! DMA

// TODO: Remove when merging.
#![warn(clippy::all)]
#![allow(clippy::missing_safety_doc)]

#[macro_use]
mod macros;
pub mod mux;
pub mod stream;
pub mod transfer;
mod utils;

use self::mux::request_gen::{
    Disabled as GenDisabled, RequestGenIsr, G0, G1, G2, G3, G4, G5, G6, G7,
};
use self::mux::request_ids::{ReqNone, RequestId as IRequestId};
use self::mux::MuxIsr;
use self::mux::{
    EgDisabled, IEgED, ISyncED, MuxId, MuxShared, RequestGenerator,
    SyncDisabled,
};
use self::stream::{Disabled, IIsrState, IsrCleared, StreamId, StreamIsr, IED};
use crate::private;
use crate::rcc::Ccdr;
use crate::stm32::{DMA1, DMA2, RCC};
use stm32h7::stm32h743::DMAMUX1;

pub use self::mux::Mux;
pub use self::stream::Stream;
pub use self::transfer::Transfer;

use crate::utils::UniqueRef;

/// Marker Trait for DMA peripherals
pub trait DmaPeripheral: private::Sealed {}
impl DmaPeripheral for DMA1 {}
impl DmaPeripheral for DMA2 {}

pub trait ChannelId: Send + private::Sealed {
    const STREAM_ID: StreamId;
    const MUX_ID: MuxId;

    type DMA: DmaPeripheral;
}

macro_rules! channels {
    ($($channel:ident => [$stream:tt, $mux:tt, $dma:ident]),*) => {
        $(
            pub struct $channel {
                _private: (),
            }

            impl crate::private::Sealed for $channel {}

            impl ChannelId for $channel {
                const STREAM_ID: StreamId = StreamId::$stream;
                const MUX_ID: MuxId = MuxId::$mux;

                type DMA = $dma;
            }
        )*
    };
}

channels! {
    C0 => [S_0, M_0, DMA1],
    C1 => [S_1, M_1, DMA1],
    C2 => [S_2, M_2, DMA1],
    C3 => [S_3, M_3, DMA1],
    C4 => [S_4, M_4, DMA1],
    C5 => [S_5, M_5, DMA1],
    C6 => [S_6, M_6, DMA1],
    C7 => [S_7, M_7, DMA1],
    C8 => [S_0, M_8, DMA2],
    C9 => [S_1, M_9, DMA2],
    C10 => [S_2, M_10, DMA2],
    C11 => [S_3, M_11, DMA2],
    C12 => [S_4, M_12, DMA2],
    C13 => [S_5, M_13, DMA2],
    C14 => [S_6, M_14, DMA2],
    C15 => [S_7, M_15, DMA2]
}

/// DMA Channel
pub struct Channel<CXX, StreamED, IsrState, ReqId, SyncED, EgED>
where
    CXX: ChannelId,
    StreamED: IED,
    IsrState: IIsrState,
    ReqId: IRequestId,
    SyncED: ISyncED,
    EgED: IEgED,
{
    pub stream: Stream<CXX, StreamED, IsrState>,
    pub mux: Mux<CXX, ReqId, SyncED, EgED>,
}

impl<CXX, StreamED, IsrState, ReqId, SyncED, EgED>
    Channel<CXX, StreamED, IsrState, ReqId, SyncED, EgED>
where
    CXX: ChannelId,
    StreamED: IED,
    IsrState: IIsrState,
    ReqId: IRequestId,
    SyncED: ISyncED,
    EgED: IEgED,
{
    /// Exposes the stream as owned value in a closure
    pub fn stream_owned<F, NewStreamED, NewIsrState>(
        self,
        op: F,
    ) -> Channel<CXX, NewStreamED, NewIsrState, ReqId, SyncED, EgED>
    where
        F: FnOnce(
            Stream<CXX, StreamED, IsrState>,
        ) -> Stream<CXX, NewStreamED, NewIsrState>,
        NewStreamED: IED,
        NewIsrState: IIsrState,
    {
        let new_stream = op(self.stream);

        Channel {
            stream: new_stream,
            mux: self.mux,
        }
    }

    /// Exposes the mux as owned value in a closure
    pub fn mux_owned<F, NewReqId, NewSyncED, NewEgED>(
        self,
        op: F,
    ) -> Channel<CXX, StreamED, IsrState, NewReqId, NewSyncED, NewEgED>
    where
        F: FnOnce(
            Mux<CXX, ReqId, SyncED, EgED>,
        ) -> Mux<CXX, NewReqId, NewSyncED, NewEgED>,
        NewReqId: IRequestId,
        NewSyncED: ISyncED,
        NewEgED: IEgED,
    {
        let new_mux = op(self.mux);

        Channel {
            stream: self.stream,
            mux: new_mux,
        }
    }
}

pub type ChannelsDma1 = (
    Channel<C0, Disabled, IsrCleared, ReqNone, SyncDisabled, EgDisabled>,
    Channel<C1, Disabled, IsrCleared, ReqNone, SyncDisabled, EgDisabled>,
    Channel<C2, Disabled, IsrCleared, ReqNone, SyncDisabled, EgDisabled>,
    Channel<C3, Disabled, IsrCleared, ReqNone, SyncDisabled, EgDisabled>,
    Channel<C4, Disabled, IsrCleared, ReqNone, SyncDisabled, EgDisabled>,
    Channel<C5, Disabled, IsrCleared, ReqNone, SyncDisabled, EgDisabled>,
    Channel<C6, Disabled, IsrCleared, ReqNone, SyncDisabled, EgDisabled>,
    Channel<C7, Disabled, IsrCleared, ReqNone, SyncDisabled, EgDisabled>,
);

pub type ChannelsDma2 = (
    Channel<C8, Disabled, IsrCleared, ReqNone, SyncDisabled, EgDisabled>,
    Channel<C9, Disabled, IsrCleared, ReqNone, SyncDisabled, EgDisabled>,
    Channel<C10, Disabled, IsrCleared, ReqNone, SyncDisabled, EgDisabled>,
    Channel<C11, Disabled, IsrCleared, ReqNone, SyncDisabled, EgDisabled>,
    Channel<C12, Disabled, IsrCleared, ReqNone, SyncDisabled, EgDisabled>,
    Channel<C13, Disabled, IsrCleared, ReqNone, SyncDisabled, EgDisabled>,
    Channel<C14, Disabled, IsrCleared, ReqNone, SyncDisabled, EgDisabled>,
    Channel<C15, Disabled, IsrCleared, ReqNone, SyncDisabled, EgDisabled>,
);

pub type RequestGenerators = (
    RequestGenerator<G0, GenDisabled>,
    RequestGenerator<G1, GenDisabled>,
    RequestGenerator<G2, GenDisabled>,
    RequestGenerator<G3, GenDisabled>,
    RequestGenerator<G4, GenDisabled>,
    RequestGenerator<G5, GenDisabled>,
    RequestGenerator<G6, GenDisabled>,
    RequestGenerator<G7, GenDisabled>,
);

/// Container for shared items across the dma
pub struct DmaShared {
    pub stream_isr_dma_1: StreamIsr<DMA1>,
    pub stream_isr_dma_2: StreamIsr<DMA2>,
    pub mux_shared: MuxShared,
}

/// Contains all channels, request generators and the shared items
pub struct Dma {
    /// Channels for DMA1
    pub channels_dma_1: ChannelsDma1,
    /// Channels for DMA2
    pub channels_dma_2: ChannelsDma2,
    /// Shared items for both DMAs
    pub dma_shared: DmaShared,
    /// All request generators
    pub request_generators: RequestGenerators,
    /// Do not access this field. This is stored in case the user want's the peripheral back.
    _dma_1: DMA1,
    /// Do not access this field. This is stored in case the user want's the peripheral back.
    _dma_2: DMA2,
    /// Do not access this field. This is stored in case the user want's the peripheral back.
    _dma_mux: DMAMUX1,
}

impl Dma {
    /// Initializes the DMA-HAL. This is the entrypoint of the HAL.
    pub fn new(
        dma_1: DMA1,
        dma_2: DMA2,
        mut dma_mux: DMAMUX1,
        ccdr: &mut Ccdr,
    ) -> Self {
        Dma::reset_dma(&mut ccdr.rb);
        Dma::enable_dma(&mut ccdr.rb);

        Dma::reset_mux(&mut dma_mux);

        let dma1_rb = unsafe { &*DMA1::ptr() };
        let dma2_rb = unsafe { &*DMA2::ptr() };
        let dma_mux_rb = unsafe { &*DMAMUX1::ptr() };

        let stream_isr_dma_1 = StreamIsr::new(
            &dma1_rb.lisr,
            &dma1_rb.hisr,
            unsafe { UniqueRef::new_unchecked(&dma1_rb.lifcr) },
            unsafe { UniqueRef::new_unchecked(&dma1_rb.hifcr) },
        );
        let stream_isr_dma_2 = StreamIsr::new(
            &dma2_rb.lisr,
            &dma2_rb.hisr,
            unsafe { UniqueRef::new_unchecked(&dma2_rb.lifcr) },
            unsafe { UniqueRef::new_unchecked(&dma2_rb.hifcr) },
        );

        let mux_isr = MuxIsr::new(&dma_mux_rb.csr, unsafe {
            UniqueRef::new_unchecked(&dma_mux_rb.cfr)
        });
        let req_gen_isr = RequestGenIsr::new(&dma_mux_rb.rgsr, unsafe {
            UniqueRef::new_unchecked(&dma_mux_rb.rgcfr)
        });
        let mux_shared = MuxShared::new(mux_isr, req_gen_isr);

        let dma_shared = DmaShared {
            stream_isr_dma_1,
            stream_isr_dma_2,
            mux_shared,
        };

        let channels_dma_1 = unsafe {
            (
                Channel {
                    stream: Stream::after_reset(UniqueRef::new_unchecked(
                        &dma1_rb.st[0],
                    )),
                    mux: Mux::after_reset(UniqueRef::new_unchecked(
                        &dma_mux_rb.ccr[0],
                    )),
                },
                Channel {
                    stream: Stream::after_reset(UniqueRef::new_unchecked(
                        &dma1_rb.st[1],
                    )),
                    mux: Mux::after_reset(UniqueRef::new_unchecked(
                        &dma_mux_rb.ccr[1],
                    )),
                },
                Channel {
                    stream: Stream::after_reset(UniqueRef::new_unchecked(
                        &dma1_rb.st[2],
                    )),
                    mux: Mux::after_reset(UniqueRef::new_unchecked(
                        &dma_mux_rb.ccr[2],
                    )),
                },
                Channel {
                    stream: Stream::after_reset(UniqueRef::new_unchecked(
                        &dma1_rb.st[3],
                    )),
                    mux: Mux::after_reset(UniqueRef::new_unchecked(
                        &dma_mux_rb.ccr[3],
                    )),
                },
                Channel {
                    stream: Stream::after_reset(UniqueRef::new_unchecked(
                        &dma1_rb.st[4],
                    )),
                    mux: Mux::after_reset(UniqueRef::new_unchecked(
                        &dma_mux_rb.ccr[4],
                    )),
                },
                Channel {
                    stream: Stream::after_reset(UniqueRef::new_unchecked(
                        &dma1_rb.st[5],
                    )),
                    mux: Mux::after_reset(UniqueRef::new_unchecked(
                        &dma_mux_rb.ccr[5],
                    )),
                },
                Channel {
                    stream: Stream::after_reset(UniqueRef::new_unchecked(
                        &dma1_rb.st[6],
                    )),
                    mux: Mux::after_reset(UniqueRef::new_unchecked(
                        &dma_mux_rb.ccr[6],
                    )),
                },
                Channel {
                    stream: Stream::after_reset(UniqueRef::new_unchecked(
                        &dma1_rb.st[7],
                    )),
                    mux: Mux::after_reset(UniqueRef::new_unchecked(
                        &dma_mux_rb.ccr[7],
                    )),
                },
            )
        };

        let channels_dma_2 = unsafe {
            (
                Channel {
                    stream: Stream::after_reset(UniqueRef::new_unchecked(
                        &dma1_rb.st[0],
                    )),
                    mux: Mux::after_reset(UniqueRef::new_unchecked(
                        &dma_mux_rb.ccr[8],
                    )),
                },
                Channel {
                    stream: Stream::after_reset(UniqueRef::new_unchecked(
                        &dma1_rb.st[1],
                    )),
                    mux: Mux::after_reset(UniqueRef::new_unchecked(
                        &dma_mux_rb.ccr[9],
                    )),
                },
                Channel {
                    stream: Stream::after_reset(UniqueRef::new_unchecked(
                        &dma1_rb.st[2],
                    )),
                    mux: Mux::after_reset(UniqueRef::new_unchecked(
                        &dma_mux_rb.ccr[10],
                    )),
                },
                Channel {
                    stream: Stream::after_reset(UniqueRef::new_unchecked(
                        &dma1_rb.st[3],
                    )),
                    mux: Mux::after_reset(UniqueRef::new_unchecked(
                        &dma_mux_rb.ccr[11],
                    )),
                },
                Channel {
                    stream: Stream::after_reset(UniqueRef::new_unchecked(
                        &dma1_rb.st[4],
                    )),
                    mux: Mux::after_reset(UniqueRef::new_unchecked(
                        &dma_mux_rb.ccr[12],
                    )),
                },
                Channel {
                    stream: Stream::after_reset(UniqueRef::new_unchecked(
                        &dma1_rb.st[5],
                    )),
                    mux: Mux::after_reset(UniqueRef::new_unchecked(
                        &dma_mux_rb.ccr[13],
                    )),
                },
                Channel {
                    stream: Stream::after_reset(UniqueRef::new_unchecked(
                        &dma1_rb.st[6],
                    )),
                    mux: Mux::after_reset(UniqueRef::new_unchecked(
                        &dma_mux_rb.ccr[14],
                    )),
                },
                Channel {
                    stream: Stream::after_reset(UniqueRef::new_unchecked(
                        &dma1_rb.st[7],
                    )),
                    mux: Mux::after_reset(UniqueRef::new_unchecked(
                        &dma_mux_rb.ccr[15],
                    )),
                },
            )
        };

        let request_generators = unsafe {
            (
                RequestGenerator::after_reset(UniqueRef::new_unchecked(
                    &dma_mux_rb.rgcr[0],
                )),
                RequestGenerator::after_reset(UniqueRef::new_unchecked(
                    &dma_mux_rb.rgcr[1],
                )),
                RequestGenerator::after_reset(UniqueRef::new_unchecked(
                    &dma_mux_rb.rgcr[2],
                )),
                RequestGenerator::after_reset(UniqueRef::new_unchecked(
                    &dma_mux_rb.rgcr[3],
                )),
                RequestGenerator::after_reset(UniqueRef::new_unchecked(
                    &dma_mux_rb.rgcr[4],
                )),
                RequestGenerator::after_reset(UniqueRef::new_unchecked(
                    &dma_mux_rb.rgcr[5],
                )),
                RequestGenerator::after_reset(UniqueRef::new_unchecked(
                    &dma_mux_rb.rgcr[6],
                )),
                RequestGenerator::after_reset(UniqueRef::new_unchecked(
                    &dma_mux_rb.rgcr[7],
                )),
            )
        };

        Dma {
            channels_dma_1,
            channels_dma_2,
            dma_shared,
            request_generators,
            _dma_1: dma_1,
            _dma_2: dma_2,
            _dma_mux: dma_mux,
        }
    }

    /// Resets the DMA
    fn reset_dma(rcc: &mut RCC) {
        rcc.ahb1rstr.modify(|_, w| w.dma1rst().set_bit());
        rcc.ahb1rstr.modify(|_, w| w.dma1rst().clear_bit());
        rcc.ahb1rstr.modify(|_, w| w.dma2rst().set_bit());
        rcc.ahb1rstr.modify(|_, w| w.dma2rst().clear_bit());
    }

    /// Enables the DMA clock
    fn enable_dma(rcc: &mut RCC) {
        rcc.ahb1enr.modify(|_, w| w.dma1en().set_bit());
        rcc.ahb1enr.modify(|_, w| w.dma2en().set_bit());
    }

    /// Resets the MUX by manually clearing all bits
    fn reset_mux(mux: &mut DMAMUX1) {
        for ccr in mux.ccr.iter() {
            ccr.reset();
        }

        mux.cfr.write(|w| {
            w.csof0()
                .set_bit()
                .csof1()
                .set_bit()
                .csof2()
                .set_bit()
                .csof3()
                .set_bit()
                .csof4()
                .set_bit()
                .csof5()
                .set_bit()
                .csof6()
                .set_bit()
                .csof7()
                .set_bit()
                .csof8()
                .set_bit()
                .csof9()
                .set_bit()
                .csof10()
                .set_bit()
                .csof11()
                .set_bit()
                .csof12()
                .set_bit()
                .csof13()
                .set_bit()
                .csof14()
                .set_bit()
                .csof15()
                .set_bit()
        });

        for rgcr in mux.rgcr.iter() {
            rgcr.reset();
        }

        mux.rgcfr.write(|w| {
            w.cof0()
                .set_bit()
                .cof1()
                .set_bit()
                .cof2()
                .set_bit()
                .cof3()
                .set_bit()
                .cof4()
                .set_bit()
                .cof5()
                .set_bit()
                .cof6()
                .set_bit()
                .cof7()
                .set_bit()
        });
    }

    pub fn free(self) -> (DMA1, DMA2, DMAMUX1) {
        (self._dma_1, self._dma_2, self._dma_mux)
    }
}

unsafe impl Sync for Dma {}

pub trait DmaExt: DmaPeripheral {
    type Other: DmaPeripheral;

    fn dma(
        self,
        other_dma: Self::Other,
        dma_mux: DMAMUX1,
        ccdr: &mut Ccdr,
    ) -> Dma;
}

impl DmaExt for DMA1 {
    type Other = DMA2;

    fn dma(self, dma_2: DMA2, dma_mux: DMAMUX1, ccdr: &mut Ccdr) -> Dma {
        Dma::new(self, dma_2, dma_mux, ccdr)
    }
}

impl DmaExt for DMA2 {
    type Other = DMA1;

    fn dma(self, dma_1: DMA1, dma_mux: DMAMUX1, ccdr: &mut Ccdr) -> Dma {
        Dma::new(dma_1, self, dma_mux, ccdr)
    }
}
