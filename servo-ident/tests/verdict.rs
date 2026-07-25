use std::collections::BTreeMap;

use serde_json::json;

use servo_ident::analyze::compute_verdict;
use servo_ident::results::{Applied, Manifest, PlotPsd, PlotStep, Step, StepResult, Verdict};

fn step_result(name: &str, flags: &[&str]) -> StepResult {
    StepResult {
        name: name.to_string(),
        drives: BTreeMap::new(),
        combined: None,
        accel: None,
        differential: None,
        ringdown: None,
        compliance: None,
        flags: flags.iter().map(|s| s.to_string()).collect(),
    }
}

fn manifest_step(name: &str, swept: serde_json::Value) -> Step {
    Step {
        name: name.to_string(),
        swept,
        applied: vec![Applied {
            servo: "motor_a".to_string(),
            addr: "0x2001.0x01".to_string(),
            ty: "u16".to_string(),
            value: json!(1),
        }],
        capture: format!("step_{name}.scap"),
        accel: None,
        stops: None,
    }
}

fn manifest(experiment: &str, stroke_plan: serde_json::Value, steps: Vec<Step>) -> Manifest {
    Manifest {
        version: 1,
        experiment: experiment.to_string(),
        command: None,
        tag: String::new(),
        axis: None,
        kinematics: None,
        belts: None,
        stroke_plan,
        ff_lead_us: 0.0,
        ff_lead_cycles: 0,
        spatial: None,
        motors: Vec::new(),
        steps,
    }
}

/// Most arms never look at the stroke plan or the plot series; this keeps
/// their call sites at the old shape.
fn verdict(experiment: &str, steps: &[StepResult], msteps: Vec<Step>) -> Result<Verdict, String> {
    compute_verdict(&manifest(experiment, json!({}), msteps), steps, &[])
}

#[test]
fn gain_sweep_picks_highest_clean_speed() {
    let steps = vec![step_result("a", &[]), step_result("b", &[])];
    let msteps = vec![
        manifest_step("a", json!({"speed": 550})),
        manifest_step("b", json!({"speed": 700})),
    ];
    let v = verdict("gain_sweep", &steps, msteps).unwrap();
    assert_eq!(v.recommended_step.as_deref(), Some("b"));
    assert!(v.apply.is_some());
}

#[test]
fn gain_sweep_skips_resonant_top_step() {
    let steps = vec![
        step_result("a", &[]),
        step_result("b", &["resonance_detected"]),
    ];
    let msteps = vec![
        manifest_step("a", json!({"speed": 550})),
        manifest_step("b", json!({"speed": 700})),
    ];
    let v = verdict("gain_sweep", &steps, msteps).unwrap();
    assert_eq!(v.recommended_step.as_deref(), Some("a"));
}

#[test]
fn gain_sweep_null_when_all_flagged() {
    let steps = vec![
        step_result("a", &["torque_saturated"]),
        step_result("b", &["resonance_detected"]),
    ];
    let msteps = vec![
        manifest_step("a", json!({"speed": 550})),
        manifest_step("b", json!({"speed": 700})),
    ];
    let v = verdict("gain_sweep", &steps, msteps).unwrap();
    assert_eq!(v.recommended_step, None);
    assert!(v.apply.is_none());
    assert!(!v.reason.is_empty());
}

#[test]
fn refine_sweep_uses_single_swept_value() {
    let steps = vec![step_result("lo", &[]), step_result("hi", &[])];
    let msteps = vec![
        manifest_step("lo", json!({"gain": 600})),
        manifest_step("hi", json!({"gain": 800})),
    ];
    let v = verdict("refine_sweep", &steps, msteps).unwrap();
    assert_eq!(v.recommended_step.as_deref(), Some("hi"));
}

#[test]
fn accel_sweep_ports_recommend() {
    let steps = vec![
        step_result("a1", &[]),
        step_result("a2", &[]),
        step_result("a3", &["torque_saturated"]),
    ];
    let msteps = vec![
        manifest_step("a1", json!({"accel": 10000})),
        manifest_step("a2", json!({"accel": 20000})),
        manifest_step("a3", json!({"accel": 30000})),
    ];
    let v = verdict("accel_sweep", &steps, msteps).unwrap();
    assert_eq!(v.recommended_step.as_deref(), Some("a2"));
}

#[test]
fn inertia_sweep_defers_to_human() {
    let v = verdict(
        "inertia_sweep",
        &[step_result("a", &[])],
        vec![manifest_step("a", json!({"ratio": 200}))],
    )
    .unwrap();
    assert_eq!(v.recommended_step, None);
    assert!(v.reason.contains("overshoot"));
}

#[test]
fn tracking_and_grid_are_not_sweeps() {
    for exp in [
        "tracking",
        "inertia_grid",
        "dynamics_fit",
        "dynamics_sweep",
        "strain_map",
        "strain_response",
        "strain_tune",
    ] {
        let v = verdict(
            exp,
            &[step_result("a", &[])],
            vec![manifest_step("a", json!({}))],
        )
        .unwrap();
        assert_eq!(v.recommended_step, None);
        assert_eq!(v.reason, "not a sweep");
    }
}

#[test]
fn dynamics_refine_defers_to_the_host_macro() {
    let v = verdict(
        "dynamics_refine",
        &[step_result("a", &[])],
        vec![manifest_step("a", json!({"scale": 0.95}))],
    )
    .unwrap();
    assert_eq!(v.recommended_step, None);
    assert!(v.reason.contains("SERVO_REFINE_DYNAMICS"));
}

#[test]
fn dynamics_tune_defers_to_the_host_macro() {
    let v = verdict(
        "dynamics_tune",
        &[step_result("a", &[])],
        vec![manifest_step("a", json!({"accel": 25000.0}))],
    )
    .unwrap();
    assert_eq!(v.recommended_step, None);
    assert!(v.reason.contains("SERVO_TUNE_DYNAMICS"));
}

#[test]
fn unknown_experiment_fails_loud() {
    assert!(verdict("bogus", &[], Vec::new()).is_err());
}

#[test]
fn every_host_experiment_string_clears_the_whitelist() {
    // Every `experiment` the Python host passes to `_run_scope` — the scope
    // analyzes each run before its command returns, so a string missing from
    // `compute_verdict`'s match fails on the bench, after the motion, instead
    // of here. Arms needing rich step data may still Err on empty steps;
    // what this pins is that none of them is "unknown".
    for exp in [
        "gain_sweep",
        "accel_sweep",
        "inertia_sweep",
        "inertia_grid",
        "dynamics_fit",
        "dynamics_sweep",
        "dynamics_tune",
        "compliance",
        "pin_sweep",
        "pin_compare",
        "tracking",
        "differential",
        "ringdown",
        "strain_map",
        "strain_response",
        "strain_tune",
    ] {
        let v = verdict(exp, &[], Vec::new());
        assert!(
            !matches!(&v, Err(e) if e.contains("unknown experiment")),
            "{exp} fell through compute_verdict's whitelist"
        );
    }
}

// ---- pin_sweep ------------------------------------------------------------

fn pin_step(name: &str, residual_mm: Option<f64>) -> StepResult {
    use servo_ident::metrics::{Metrics, TorqueSummary};
    use servo_ident::resonance::Resonance;
    use servo_ident::results::DriveResult;
    let mut sr = step_result(name, &[]);
    sr.drives.insert(
        "motor_a".to_string(),
        DriveResult {
            metrics: Metrics {
                samples: 100,
                moves: Vec::new(),
                torque_saturation_pct: 0.0,
                torque: TorqueSummary {
                    peak: 0,
                    peak_pct_rated: 0.0,
                    moving_samples: 0,
                    rail_detected: false,
                    rail_level: 0,
                    rail_samples: 0,
                    rail_pct_moving: 0.0,
                    rail_ms: 0.0,
                    longest_burst_ms: 0.0,
                },
                ferr_crosscheck_max: 0,
                ff_velocity_offset_max: None,
                ff_torque_offset_max: None,
                pin_residual_mm: residual_mm,
                pin_phase_deg: None,
            },
            psd_peaks: Vec::new(),
            resonance: Resonance {
                detected: false,
                ratio: 0.0,
                peak_hz: 0.0,
            },
        },
    );
    sr
}

#[test]
fn pin_sweep_recommends_min_residual_step() {
    let steps = vec![
        pin_step("v0", Some(0.004)),
        pin_step("v1", Some(0.0012)),
        pin_step("v2", Some(0.009)),
    ];
    let msteps = vec![
        manifest_step("v0", json!({"zeta": 0.02})),
        manifest_step("v1", json!({"zeta": 0.05})),
        manifest_step("v2", json!({"zeta": 0.1})),
    ];
    let v = verdict("pin_sweep", &steps, msteps).unwrap();
    assert_eq!(v.recommended_step.as_deref(), Some("v1"));
    assert!(
        v.reason.contains("min residual 1.20 um at v1"),
        "{}",
        v.reason
    );
}

#[test]
fn pin_sweep_without_pin_channels_recommends_nothing() {
    let steps = vec![pin_step("v0", None), pin_step("v1", None)];
    let msteps = vec![
        manifest_step("v0", json!({"zeta": 0.02})),
        manifest_step("v1", json!({"zeta": 0.05})),
    ];
    let v = verdict("pin_sweep", &steps, msteps).unwrap();
    assert!(v.recommended_step.is_none());
    assert!(
        v.reason.contains("no step carries pin residual"),
        "{}",
        v.reason
    );
}

// ---- pin_compare ----------------------------------------------------------

/// Uniform PSD grid every compare plot shares; the swept band below is
/// 70-200 Hz, so the 240/280 Hz bins are out of band.
const COMPARE_GRID_HZ: [f64; 8] = [0.0, 40.0, 80.0, 120.0, 160.0, 200.0, 240.0, 280.0];

/// mm²/Hz putting a single-sided tone of `amp_um` into one Welch bin of
/// width `df`: the inverse of the arm's amplitude conversion.
fn psd_for_amp_um(amp_um: f64, df: f64) -> f64 {
    let amp_mm = amp_um * 1e-3;
    amp_mm * amp_mm / (2.0 * 1.5 * df)
}

fn plot_step_with_psd(name: &str, grid: Vec<f64>, psd: Vec<f64>) -> PlotStep {
    let mut cartesian = BTreeMap::new();
    cartesian.insert("y".to_string(), psd);
    PlotStep {
        name: name.to_string(),
        fs_hz: 4000.0,
        stride: 1,
        t_s: Vec::new(),
        moving: Vec::new(),
        drives: BTreeMap::new(),
        combined: None,
        accel: None,
        differential: None,
        ringdown: None,
        compliance: None,
        path: None,
        psd: PlotPsd {
            freq_hz: grid,
            per_drive: BTreeMap::new(),
            cartesian: Some(cartesian),
            accel: None,
        },
    }
}

fn compare_plot(name: &str, in_band_amp_um: f64) -> PlotStep {
    // The in-band peak rides the 120 Hz bin; both out-of-band bins carry a
    // tone ten times larger, so a scorer ignoring the swept band would rank
    // every step by garbage the operator never asked about.
    let mut psd = vec![0.0; COMPARE_GRID_HZ.len()];
    psd[3] = psd_for_amp_um(in_band_amp_um, 40.0);
    psd[6] = psd_for_amp_um(in_band_amp_um * 10.0, 40.0);
    psd[7] = psd_for_amp_um(in_band_amp_um * 10.0, 40.0);
    plot_step_with_psd(name, COMPARE_GRID_HZ.to_vec(), psd)
}

/// Fine grid (df = 5 Hz) for FREQ-compare tests: the migration tolerance is
/// two bins, which the coarse grid above cannot resolve.
fn freq_plot(name: &str, amp_um: f64, tone_hz: f64) -> PlotStep {
    let grid: Vec<f64> = (0..=40).map(|k| k as f64 * 5.0).collect();
    let mut psd = vec![0.0; grid.len()];
    psd[(tone_hz / 5.0).round() as usize] = psd_for_amp_um(amp_um, 5.0);
    plot_step_with_psd(name, grid, psd)
}

fn compare_plan(param: &str) -> serde_json::Value {
    json!({
        "mode": "y",
        "param": param,
        "freq_start": 70.0,
        "freq_end": 200.0,
    })
}

/// The bench shape that motivated the arm (compare_20260725_170124): the
/// in-band ferr peak is a U over zeta while the settled-tail residual is
/// monotone in the pin gain — the residuals here rank the largest zeta
/// first, and the verdict must ignore them.
#[test]
fn pin_compare_scores_the_swept_band_not_the_residual_tail() {
    let steps = vec![
        pin_step("zeta0p03", Some(0.000040)),
        pin_step("zeta0p04", Some(0.000035)),
        pin_step("zeta0p045", Some(0.000033)),
        pin_step("zeta0p07", Some(0.000030)),
    ];
    let plots = vec![
        compare_plot("zeta0p03", 2.63),
        compare_plot("zeta0p04", 1.59),
        compare_plot("zeta0p045", 1.57),
        compare_plot("zeta0p07", 2.34),
    ];
    let msteps = vec![
        manifest_step("zeta0p03", json!({"value": 0.03})),
        manifest_step("zeta0p04", json!({"value": 0.04})),
        manifest_step("zeta0p045", json!({"value": 0.045})),
        manifest_step("zeta0p07", json!({"value": 0.07})),
    ];
    let m = manifest("pin_compare", compare_plan("ZETA"), msteps);
    let v = compute_verdict(&m, &steps, &plots).unwrap();
    // zeta0p045 scores best, but ZETA takes the lowest value within 15% —
    // zeta0p04 — matching the staircase picker's under-driving rule.
    assert_eq!(
        v.recommended_step.as_deref(),
        Some("zeta0p04"),
        "{}",
        v.reason
    );
    assert!(v.reason.contains("flattest in-band ferr"), "{}", v.reason);
    assert!(v.reason.contains("lowest ZETA within 15%"), "{}", v.reason);
    assert!(v.reason.contains("@ 120 Hz"), "{}", v.reason);
}

#[test]
fn pin_compare_lead_takes_the_outright_minimum() {
    // lead0 is within 15% of lead600's score; the ZETA tie rule would take
    // the lower value, but LEAD is not a gain and must take the minimum.
    let steps = vec![pin_step("lead0", None), pin_step("lead600", None)];
    let plots = vec![compare_plot("lead0", 1.9), compare_plot("lead600", 1.8)];
    let msteps = vec![
        manifest_step("lead0", json!({"value": 0.0})),
        manifest_step("lead600", json!({"value": 600.0})),
    ];
    let m = manifest("pin_compare", compare_plan("LEAD"), msteps);
    let v = compute_verdict(&m, &steps, &plots).unwrap();
    assert_eq!(
        v.recommended_step.as_deref(),
        Some("lead600"),
        "{}",
        v.reason
    );
}

/// Bench frequency-ladder signature (2026-07-25): every too-high model f_b
/// parks its worst tone at one fixed physical frequency (~135 here, NOT its
/// own f_b), so a winner whose tone still sits with the failing steps'
/// means the frequency is off, and a winner at the ladder floor means the
/// ladder should extend - both notes must fire together here.
#[test]
fn pin_compare_freq_flags_an_unmigrated_tone_and_the_ladder_floor() {
    let steps = vec![
        pin_step("freq136", None),
        pin_step("freq134", None),
        pin_step("freq133", None),
    ];
    let plots = vec![
        freq_plot("freq136", 3.0, 135.0),
        freq_plot("freq134", 2.5, 135.0),
        freq_plot("freq133", 2.1, 135.0),
    ];
    let msteps = vec![
        manifest_step("freq136", json!({"value": 136.0})),
        manifest_step("freq134", json!({"value": 134.0})),
        manifest_step("freq133", json!({"value": 133.0})),
    ];
    let m = manifest("pin_compare", compare_plan("FREQ"), msteps);
    let v = compute_verdict(&m, &steps, &plots).unwrap();
    assert_eq!(
        v.recommended_step.as_deref(),
        Some("freq133"),
        "{}",
        v.reason
    );
    assert!(v.reason.contains("has not migrated"), "{}", v.reason);
    assert!(v.reason.contains("ladder floor"), "{}", v.reason);
}

#[test]
fn pin_compare_freq_migrated_tone_drops_the_off_note() {
    // The winner's worst tone moved to unrelated background (160 Hz) while
    // the failing step's sits at 135: the frequency is right, only the
    // floor note remains (nothing below it was tested).
    let steps = vec![pin_step("freq133", None), pin_step("freq130", None)];
    let plots = vec![
        freq_plot("freq133", 2.1, 135.0),
        freq_plot("freq130", 1.6, 160.0),
    ];
    let msteps = vec![
        manifest_step("freq133", json!({"value": 133.0})),
        manifest_step("freq130", json!({"value": 130.0})),
    ];
    let m = manifest("pin_compare", compare_plan("FREQ"), msteps);
    let v = compute_verdict(&m, &steps, &plots).unwrap();
    assert_eq!(
        v.recommended_step.as_deref(),
        Some("freq130"),
        "{}",
        v.reason
    );
    assert!(!v.reason.contains("has not migrated"), "{}", v.reason);
    assert!(v.reason.contains("ladder floor"), "{}", v.reason);
}

#[test]
fn pin_compare_freq_flat_ladder_reports_a_tie() {
    // Both tones already in the background and scores within 25%: the
    // frequencies are equivalent - say so instead of a false "still off"
    // (both tones sit at the same background frequency).
    let steps = vec![pin_step("freq131", None), pin_step("freq130", None)];
    let plots = vec![
        freq_plot("freq131", 1.62, 160.0),
        freq_plot("freq130", 1.59, 160.0),
    ];
    let msteps = vec![
        manifest_step("freq131", json!({"value": 131.0})),
        manifest_step("freq130", json!({"value": 130.0})),
    ];
    let m = manifest("pin_compare", compare_plan("FREQ"), msteps);
    let v = compute_verdict(&m, &steps, &plots).unwrap();
    assert!(v.reason.contains("nearly tie"), "{}", v.reason);
    assert!(!v.reason.contains("has not migrated"), "{}", v.reason);
}

/// A manifest without a chirp plan (older build, foreign run) cannot be
/// ranked - but the verdict must degrade to a named no-recommendation, not
/// an Err: a verdict Err fails the whole analyze and takes the charts (the
/// comparison's actual product) down with it.
#[test]
fn pin_compare_without_a_band_plan_degrades_to_no_recommendation() {
    let steps = vec![pin_step("zeta0p04", None)];
    let plots = vec![compare_plot("zeta0p04", 1.59)];
    let msteps = vec![manifest_step("zeta0p04", json!({"value": 0.04}))];
    let m = manifest("pin_compare", json!({}), msteps);
    let v = compute_verdict(&m, &steps, &plots).unwrap();
    assert!(v.recommended_step.is_none());
    assert!(
        v.reason.contains("cannot rank the sweep") && v.reason.contains("stroke_plan.freq_start"),
        "{}",
        v.reason
    );
}
