use crate::{ExtendedId, Frame, StandardId};

#[test]
fn data_greater_remote() {
    let id = StandardId::new(0).unwrap();

    let data_frame = Frame::new_data(id, []);
    let remote_frame = Frame::new_remote(id, 0);
    assert!(data_frame.is_data_frame());
    assert!(remote_frame.is_remote_frame());

    assert!(data_frame.priority() > remote_frame.priority());
}

#[test]
fn lower_ids_win_arbitration() {
    let zero = Frame::new_data(StandardId::new(0).unwrap(), []);
    let one = Frame::new_data(StandardId::new(1).unwrap(), []);
    assert!(zero.is_standard());
    assert!(!zero.is_extended());
    assert!(one.is_standard());
    assert!(!one.is_extended());
    assert!(zero.priority() > one.priority());

    // Standard IDs have priority over Extended IDs if the Base ID matches.
    let ext_one = Frame::new_data(
        ExtendedId::new(0b00000000001_000000000000000000).unwrap(),
        [],
    );
    assert!(!ext_one.is_standard());
    assert!(ext_one.is_extended());
    assert!(one.priority() > ext_one.priority());
    assert!(zero.priority() > ext_one.priority());

    // Ext. ID with Base ID 0 has priority over Standard ID 1.
    let ext_zero = Frame::new_data(
        ExtendedId::new(0b00000000000_100000000000000000).unwrap(),
        [],
    );
    assert!(!ext_zero.is_standard());
    assert!(ext_zero.is_extended());
    assert!(one.priority() < ext_zero.priority());
    // ...but not over Standard ID 0.
    assert!(zero.priority() > ext_zero.priority());
}

#[test]
fn highest_standard_higher_prio_than_highest_ext() {
    let std = Frame::new_data(StandardId::MAX, []);
    let ext = Frame::new_data(ExtendedId::MAX, []);

    assert!(std.is_standard());
    assert!(!std.is_extended());
    assert!(!ext.is_standard());
    assert!(ext.is_extended());
    assert!(std.priority() > ext.priority());
}

#[test]
fn data_neq_remote() {
    let id = StandardId::new(0).unwrap();

    let data_frame = Frame::new_data(id, []);
    let remote_frame = Frame::new_remote(id, 0);

    assert_ne!(data_frame, remote_frame);
}

#[test]
fn remote_eq_remote_ignores_data() {
    let mut remote1 = Frame::new_remote(StandardId::MAX, 7);
    let mut remote2 = Frame::new_remote(StandardId::MAX, 7);

    remote1.data.bytes = [0xAA; 8];
    remote2.data.bytes = [0x55; 8];

    assert_eq!(remote1, remote2);
}

#[test]
fn max_len() {
    Frame::new_data(StandardId::MAX, [0; 8]);
    Frame::new_remote(StandardId::MAX, 8);
}
