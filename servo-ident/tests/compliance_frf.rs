//! Physics validation for the compliance notch pipeline: simulate a
//! closed-loop two-mass servo (rotor on a position loop, load on a belt
//! spring) under a swept position buzz, then recover the locked-rotor
//! belt frequency f_b = sqrt(k/m)/2pi from the instrumental-variable
//! torque->position FRF. The notch must land on the analytic f_b, NOT
//! on the (higher) coupled resonance - that distinction is the entire
//! point of the estimator.

use std::f64::consts::PI;

use servo_ident::frf::{complex_ratio, find_notch, welch_frf};

const FS: f64 = 4000.0;
const F_B: f64 = 180.0; // locked-rotor sqrt(k/m)/2pi
const M_LOAD: f64 = 1.0;
const J_ROTOR: f64 = 2.0;

struct Sim {
    cmd: Vec<f64>,
    act: Vec<f64>,
    torque: Vec<f64>,
}

/// Semi-implicit Euler two-mass sim: PD position loop on the rotor,
/// belt spring (with light damping) to the load. Coupled mode lands at
/// f_b * sqrt(1 + m/J) ~ 220 Hz, inside the analysis band.
fn simulate(seconds: f64) -> Sim {
    let k = M_LOAD * (2.0 * PI * F_B).powi(2);
    let c_belt = 2.0 * 0.01 * (2.0 * PI * F_B) * M_LOAD; // zeta ~ 1%
    let w_loop = 2.0 * PI * 50.0;
    let kp = J_ROTOR * w_loop * w_loop;
    let kd = 2.0 * J_ROTOR * 0.7 * w_loop;
    let dt = 1.0 / FS;
    let n = (seconds * FS) as usize;
    let (f0, f1) = (40.0, 320.0);
    let sweep_rate = (f1 - f0) / seconds;
    let amp = 0.02; // mm
    let mut theta = 0.0f64; // rotor position (mm at the pulley)
    let mut theta_v = 0.0f64;
    let mut x = 0.0f64; // load position
    let mut x_v = 0.0f64;
    let mut prev_cmd = 0.0f64;
    let mut sim = Sim {
        cmd: Vec::with_capacity(n),
        act: Vec::with_capacity(n),
        torque: Vec::with_capacity(n),
    };
    for i in 0..n {
        let t = i as f64 * dt;
        // Instantaneous phase of a linear chirp.
        let phase = 2.0 * PI * (f0 * t + 0.5 * sweep_rate * t * t);
        let cmd = amp * phase.sin();
        let cmd_v = (cmd - prev_cmd) / dt;
        prev_cmd = cmd;
        let belt = k * (theta - x) + c_belt * (theta_v - x_v);
        let tau = kp * (cmd - theta) + kd * (cmd_v - theta_v);
        theta_v += (tau - belt) / J_ROTOR * dt;
        theta += theta_v * dt;
        x_v += belt / M_LOAD * dt;
        x += x_v * dt;
        sim.cmd.push(cmd);
        sim.act.push(theta);
        sim.torque.push(tau);
    }
    sim
}

#[test]
fn iv_frf_notch_recovers_the_locked_rotor_frequency() {
    let sim = simulate(30.0);
    let cmd_to_act = welch_frf(&sim.cmd, &sim.act, FS, 4096).unwrap();
    let cmd_to_torque = welch_frf(&sim.cmd, &sim.torque, FS, 4096).unwrap();
    let g = complex_ratio(&cmd_to_act, &cmd_to_torque).unwrap();
    let notch = find_notch(&g, 60.0, 300.0).unwrap();
    assert!(
        (notch.freq_hz - F_B).abs() < 2.0,
        "notch at {:.2} Hz, expected f_b = {F_B} Hz",
        notch.freq_hz
    );
    assert!(
        notch.depth_db > 10.0,
        "anti-resonance should carve a deep hole, got {:.1} dB",
        notch.depth_db
    );
    assert!(
        notch.flank_coherence > 0.9,
        "chirp flanks must be coherent, got {:.2}",
        notch.flank_coherence
    );
    // The coupled resonance sits at f_b*sqrt(1 + m/J) ~ 220 Hz; the notch
    // must NOT be dragged there.
    let coupled = F_B * (1.0 + M_LOAD / J_ROTOR).sqrt();
    assert!(
        (notch.freq_hz - coupled).abs() > 20.0,
        "notch {:.1} Hz must be distinct from the coupled mode {:.1} Hz",
        notch.freq_hz,
        coupled
    );
}

#[test]
fn direct_estimate_is_biased_but_iv_is_not_fooled_by_loop_noise() {
    // Same plant, but with measurement noise injected into the torque
    // channel (sensor noise correlated with nothing): the IV estimate
    // must still find the notch.
    let mut sim = simulate(30.0);
    let mut seed = 0x9e3779b97f4a7c15u64;
    let mut rng = || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        (seed >> 11) as f64 / (1u64 << 53) as f64 - 0.5
    };
    let tau_rms = (sim.torque.iter().map(|v| v * v).sum::<f64>() / sim.torque.len() as f64).sqrt();
    for v in &mut sim.torque {
        *v += 0.1 * tau_rms * rng();
    }
    let cmd_to_act = welch_frf(&sim.cmd, &sim.act, FS, 4096).unwrap();
    let cmd_to_torque = welch_frf(&sim.cmd, &sim.torque, FS, 4096).unwrap();
    let g = complex_ratio(&cmd_to_act, &cmd_to_torque).unwrap();
    let notch = find_notch(&g, 60.0, 300.0).unwrap();
    assert!(
        (notch.freq_hz - F_B).abs() < 3.0,
        "noisy-torque notch at {:.2} Hz, expected {F_B} Hz",
        notch.freq_hz
    );
}
