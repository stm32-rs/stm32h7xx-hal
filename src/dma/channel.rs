pub trait ChannelId {
    const STREAM_ID: usize;
    const MUX_ID: usize;
}

macro_rules! channels {
    ($($channel:ident => [$stream:tt, $mux:tt]),*) => {
        $(
            pub struct $channel;

            impl ChannelId for $channel {
                const STREAM_ID: usize = $stream;
                const MUX_ID: usize = $mux;
            }
        )*
    };
}

channels! {
    C0 => [0, 0],
    C1 => [1, 1],
    C2 => [2, 2],
    C3 => [3, 3],
    C4 => [4, 4],
    C5 => [5, 5],
    C6 => [6, 6],
    C7 => [7, 7],
    C8 => [0, 8],
    C9 => [1, 9],
    C10 => [2, 10],
    C11 => [3, 11],
    C12 => [4, 12],
    C13 => [5, 13],
    C14 => [6, 14],
    C15 => [7, 15]
}
