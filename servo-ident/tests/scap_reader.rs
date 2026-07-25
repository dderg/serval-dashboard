use servo_ident::scap::Scap;

fn synthetic(version: i64) -> Vec<u8> {
    let header = format!(
        "{{\"version\":{version},\"cycle_ns\":250000,\"record_size\":17,\
         \"drives\":[{{\"name\":\"d0\",\"counts_per_mm\":1000.0}},\
         {{\"name\":\"d1\",\"counts_per_mm\":1000.0}}],\
         \"channels\":[{{\"name\":\"cycle_index\",\"dtype\":\"u64\",\"offset\":0}},\
         {{\"name\":\"flags\",\"dtype\":\"u8\",\"offset\":8}},\
         {{\"name\":\"following_error\",\"dtype\":\"i32\",\"offset\":9}}]}}"
    );
    let mut b = header.into_bytes();
    b.push(b'\n');
    for r in 0..3i32 {
        b.extend_from_slice(&(r as u64).to_le_bytes());
        b.push(2u8);
        b.extend_from_slice(&(r * 10).to_le_bytes());
        b.extend_from_slice(&(-(r * 10)).to_le_bytes());
    }
    b.push(0xAB); // trailing partial record, must be dropped
    b
}

#[test]
fn reads_per_drive_channels_and_drops_partial_record() {
    let cap = Scap::from_bytes(&synthetic(2)).unwrap();
    assert_eq!(cap.n_records, 3);
    assert_eq!(cap.fs(), 4000.0);
    assert_eq!(cap.drive_names(), vec!["d0", "d1"]);
    assert_eq!(cap.read_i64(0, "following_error").unwrap(), vec![0, 10, 20]);
    assert_eq!(
        cap.read_i64(1, "following_error").unwrap(),
        vec![0, -10, -20]
    );
    assert_eq!(cap.read_i64(0, "flags").unwrap(), vec![2, 2, 2]);
    assert_eq!(cap.read_i64(0, "cycle_index").unwrap(), vec![0, 1, 2]);
}

#[test]
fn zstd_compressed_parses_identically_to_raw() {
    // A capture compressed with zstd (as kalico writes `.scap.zst` inline)
    // must decode to exactly the same records as the raw bytes, detected by
    // the zstd magic and not by any file extension.
    let raw = synthetic(2);
    let compressed = zstd::encode_all(&raw[..], 3).unwrap();
    assert_eq!(&compressed[..4], &[0x28, 0xB5, 0x2F, 0xFD]);
    let from_raw = Scap::from_bytes(&raw).unwrap();
    let from_zstd = Scap::from_bytes(&compressed).unwrap();
    assert_eq!(from_zstd.n_records, from_raw.n_records);
    assert_eq!(from_zstd.drive_names(), from_raw.drive_names());
    assert_eq!(
        from_zstd.read_i64(0, "following_error").unwrap(),
        from_raw.read_i64(0, "following_error").unwrap()
    );
    assert_eq!(
        from_zstd.read_i64(1, "following_error").unwrap(),
        from_raw.read_i64(1, "following_error").unwrap()
    );
    assert_eq!(
        from_zstd.read_i64(0, "cycle_index").unwrap(),
        from_raw.read_i64(0, "cycle_index").unwrap()
    );
}

#[test]
fn rejects_unsupported_version() {
    assert!(Scap::from_bytes(&synthetic(3)).is_err());
}

#[test]
fn accepts_version_one() {
    assert!(Scap::from_bytes(&synthetic(1)).is_ok());
}

#[test]
fn rejects_failed_capture_path() {
    let err = Scap::load("/nonexistent/run.failed.scap").unwrap_err();
    assert!(err.contains("FAILED capture"), "{err}");
}

#[test]
fn rejects_missing_channel() {
    let cap = Scap::from_bytes(&synthetic(2)).unwrap();
    assert!(cap.read_i64(0, "no_such_channel").is_err());
    assert!(!cap.has_channel("no_such_channel"));
}
