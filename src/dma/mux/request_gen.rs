use super::super::utils::TypeState;

type_state! {
    ED, Disabled, Enabled
}

pub unsafe trait GenId {
    const ID: usize;
}

macro_rules! gen_ids {
    ($($name:ident => $id:tt),*) => {
        $(
            pub struct $name;

            unsafe impl GenId for $name {
                const ID:usize = $id;
            }
        )*
    };
}

gen_ids! {
    G0 => 0,
    G1 => 1,
    G2 => 2,
    G3 => 3,
    G4 => 4,
    G5 => 5,
    G6 => 6,
    G7 => 7
}
