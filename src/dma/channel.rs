use super::stm32::{DMA1, DMA2};
use super::DMATrait;

pub unsafe trait ChannelId: Send {
    const STREAM_ID: usize;
    const MUX_ID: usize;

    type DMA: DMATrait;
}

macro_rules! channels {
    ($($channel:ident => [$stream:tt, $mux:tt, $dma:ident]),*) => {
        $(
            pub struct $channel;

            unsafe impl ChannelId for $channel {
                const STREAM_ID: usize = $stream;
                const MUX_ID: usize = $mux;

                type DMA = $dma;
            }
        )*
    };
}

channels! {
    C0 => [0, 0, DMA1],
    C1 => [1, 1, DMA1],
    C2 => [2, 2, DMA1],
    C3 => [3, 3, DMA1],
    C4 => [4, 4, DMA1],
    C5 => [5, 5, DMA1],
    C6 => [6, 6, DMA1],
    C7 => [7, 7, DMA1],
    C8 => [0, 8, DMA2],
    C9 => [1, 9, DMA2],
    C10 => [2, 10, DMA2],
    C11 => [3, 11, DMA2],
    C12 => [4, 12, DMA2],
    C13 => [5, 13, DMA2],
    C14 => [6, 14, DMA2],
    C15 => [7, 15, DMA2]
}
