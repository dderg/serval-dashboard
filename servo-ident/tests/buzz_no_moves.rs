//! A buzz-family capture is a locked-rotor position wiggle, not commanded
//! moves, so the motion-active flag may never latch and the capture can carry
//! zero moving segments. The analyzer must still produce PSD/residual metrics
//! over the whole capture for buzz-family experiments (`require_moving =
//! false`), while genuinely move-based experiments (`require_moving = true`)
//! keep erroring when no moves are present.

use servo_ident::analyze::analyze_capture;
use servo_ident::metrics::{DEFAULT_SETTLE_BAND_COUNTS, DEFAULT_TORQUE_LIMIT_PER_MILLE};
use servo_ident::scap::Scap;

/// One-drive capture whose target never moves and whose motion-active flag is
/// never set — the following error is a pure position wiggle (a buzz tone).
fn buzz_capture(n: usize) -> Scap {
    // Global: cycle_index(u64)@0, flags(u8)@8 -> prefix 9.
    // Per-drive block (16 bytes): target_counts@9, position_actual@13,
    // following_error@17, torque_actual@21. record_size 25.
    let header = "{\"version\":2,\"cycle_ns\":250000,\"record_size\":25,\
         \"drives\":[{\"name\":\"motor_a\",\"counts_per_mm\":1000.0}],\
         \"channels\":[{\"name\":\"cycle_index\",\"dtype\":\"u64\",\"offset\":0},\
         {\"name\":\"flags\",\"dtype\":\"u8\",\"offset\":8},\
         {\"name\":\"target_counts\",\"dtype\":\"i32\",\"offset\":9},\
         {\"name\":\"position_actual\",\"dtype\":\"i32\",\"offset\":13},\
         {\"name\":\"following_error\",\"dtype\":\"i32\",\"offset\":17},\
         {\"name\":\"torque_actual\",\"dtype\":\"i32\",\"offset\":21}]}";
    let mut b = header.as_bytes().to_vec();
    b.push(b'\n');
    for r in 0..n {
        // A ~130 Hz wiggle at fs = 4000 Hz: a position tone about a fixed
        // point, exactly what a locked-rotor buzz produces.
        let phase = 2.0 * std::f64::consts::PI * 130.0 * r as f64 / 4000.0;
        let ferr = (40.0 * phase.sin()).round() as i32;
        b.extend_from_slice(&(r as u64).to_le_bytes());
        b.push(0u8); // flags: motion-active never set -> no moving segments
        b.extend_from_slice(&0i32.to_le_bytes()); // target_counts: stationary
        b.extend_from_slice(&(-ferr).to_le_bytes()); // position_actual
        b.extend_from_slice(&ferr.to_le_bytes()); // following_error
        b.extend_from_slice(&5i32.to_le_bytes()); // torque_actual
    }
    Scap::from_bytes(&b).unwrap()
}

#[test]
fn buzz_family_analyzes_without_moving_segments() {
    let cap = buzz_capture(256);
    let (step, plot) = analyze_capture(
        &cap,
        "v0",
        DEFAULT_SETTLE_BAND_COUNTS,
        DEFAULT_TORQUE_LIMIT_PER_MILLE,
        None,
        None,
        None,
        0,
        None,
        false, // buzz-family: moves not required
    )
    .expect("buzz capture analyzes with require_moving = false");

    // PSD over the whole capture is computed and carries the wiggle tone.
    assert!(!plot.psd.freq_hz.is_empty(), "PSD grid present");
    let drive = step.drives.get("motor_a").expect("drive present");
    // Move-based metrics are simply absent — no commanded moves to segment.
    assert!(drive.metrics.moves.is_empty(), "no per-move metrics");
    assert!(!drive.psd_peaks.is_empty(), "residual PSD peaks present");
    assert_eq!(drive.metrics.samples, 256);
}

#[test]
fn move_based_still_errors_without_moving_segments() {
    let cap = buzz_capture(256);
    let err = analyze_capture(
        &cap,
        "v0",
        DEFAULT_SETTLE_BAND_COUNTS,
        DEFAULT_TORQUE_LIMIT_PER_MILLE,
        None,
        None,
        None,
        0,
        None,
        true, // move-based: moves required
    )
    .expect_err("move-based analysis must error when no moves are present");
    assert!(
        err.contains("no moving segments"),
        "unexpected error: {err}"
    );
}
