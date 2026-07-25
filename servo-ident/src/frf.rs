//! Differential belt-pair FRF, ported from `scripts/servo_diff_report.py`:
//! the H1 Welch estimate with coherence (Hann window, half overlap,
//! mean-removed segments), half-power damping, and coherent mode picking.
//! The estimator conventions must match the Python so mode selection is
//! comparable within fp tolerance.

use core::f64::consts::SQRT_2;
use core::ops::Range;

use crate::psd::{fft_pow2, hanning};
use crate::results::DifferentialMode;
use crate::scap::Scap;

pub const MIN_NPERSEG: usize = 256;
pub const MIN_SEGMENTS: usize = 4;
pub const COHERENCE_MIN: f64 = 0.5;
pub const MAX_MODES: usize = 5;
pub const DEFAULT_NPERSEG: usize = 4096;

#[derive(Debug)]
pub struct DifferentialSeries {
    pub pair: Vec<String>,
    pub cmd_mm: Vec<f64>,
    pub act_mm: Vec<f64>,
    pub torque: Vec<f64>,
}

pub fn differential_series(cap: &Scap) -> Result<DifferentialSeries, String> {
    let drives = &cap.header.drives;
    if drives.len() != 2 {
        return Err(format!(
            "differential capture must hold exactly the two pair drives, got {}: [{}]",
            drives.len(),
            cap.drive_names().join(", ")
        ));
    }
    let mut per_drive = Vec::with_capacity(2);
    for (idx, d) in drives.iter().enumerate() {
        if d.counts_per_mm <= 0.0 {
            return Err(format!(
                "drive {:?} has non-positive counts_per_mm {}",
                d.name, d.counts_per_mm
            ));
        }
        let sign = if d.invert { -1.0 } else { 1.0 };
        let scale = sign / d.counts_per_mm;
        let cmd_mm: Vec<f64> = cap
            .read_f64(idx, "target_counts")?
            .iter()
            .map(|v| v * scale)
            .collect();
        let act_mm: Vec<f64> = cap
            .read_f64(idx, "position_actual")?
            .iter()
            .map(|v| v * scale)
            .collect();
        let torque: Vec<f64> = cap
            .read_f64(idx, "torque_actual")?
            .iter()
            .map(|v| v * sign)
            .collect();
        per_drive.push((cmd_mm, act_mm, torque));
    }
    let (b_cmd, b_act, b_torque) = per_drive.pop().unwrap();
    let (a_cmd, a_act, a_torque) = per_drive.pop().unwrap();
    let sub = |a: &[f64], b: &[f64]| a.iter().zip(b).map(|(&x, &y)| x - y).collect();
    Ok(DifferentialSeries {
        pair: cap.drive_names(),
        cmd_mm: sub(&a_cmd, &b_cmd),
        act_mm: sub(&a_act, &b_act),
        torque: sub(&a_torque, &b_torque),
    })
}

fn median(values: &[f64]) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    let mut sorted = values.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).expect("differential command holds a NaN"));
    let mid = sorted.len() / 2;
    if sorted.len() % 2 == 1 {
        sorted[mid]
    } else {
        0.5 * (sorted[mid - 1] + sorted[mid])
    }
}

pub fn active_slice(cmd: &[f64]) -> Result<Range<usize>, String> {
    let center = median(cmd);
    let dev: Vec<f64> = cmd.iter().map(|&v| (v - center).abs()).collect();
    let peak = dev.iter().fold(0.0_f64, |m, &v| m.max(v));
    if peak <= 0.0 {
        return Err(
            "capture holds no differential excitation (differential command \
             is flat); was the anti-phase buzz armed on this pair?"
                .to_string(),
        );
    }
    let threshold = 0.05 * peak;
    let first = dev.iter().position(|&d| d > threshold).unwrap();
    let last = dev.iter().rposition(|&d| d > threshold).unwrap();
    Ok(first..last + 1)
}

fn welch_segment_length(n: usize, mut nperseg: usize) -> Result<usize, String> {
    if !nperseg.is_power_of_two() {
        return Err(format!("nperseg {nperseg} is not a power of two"));
    }
    while nperseg * (MIN_SEGMENTS + 1) / 2 > n && nperseg > MIN_NPERSEG {
        nperseg /= 2;
    }
    if nperseg < MIN_NPERSEG || n < nperseg * (MIN_SEGMENTS + 1) / 2 {
        return Err(format!(
            "capture too short for a Welch FRF: {n} active samples but \
             {MIN_SEGMENTS} segments of {MIN_NPERSEG} are needed; sweep longer or slower"
        ));
    }
    Ok(nperseg)
}

#[derive(Debug)]
pub struct Frf {
    pub freqs: Vec<f64>,
    pub re: Vec<f64>,
    pub im: Vec<f64>,
    pub coherence: Vec<f64>,
    pub segments: usize,
}

impl Frf {
    pub fn magnitude(&self) -> Vec<f64> {
        self.re
            .iter()
            .zip(&self.im)
            .map(|(&r, &i)| libm::hypot(r, i))
            .collect()
    }
}

fn windowed_rfft(seg: &[f64], win: &[f64], bins: usize) -> (Vec<f64>, Vec<f64>) {
    let mean = seg.iter().sum::<f64>() / seg.len() as f64;
    let mut re: Vec<f64> = seg.iter().zip(win).map(|(&v, &w)| (v - mean) * w).collect();
    let mut im = vec![0.0; seg.len()];
    fft_pow2(&mut re, &mut im);
    re.truncate(bins);
    im.truncate(bins);
    (re, im)
}

pub fn welch_frf(x: &[f64], y: &[f64], fs: f64, nperseg: usize) -> Result<Frf, String> {
    if x.len() != y.len() {
        return Err(format!(
            "excitation and response lengths differ ({} vs {})",
            x.len(),
            y.len()
        ));
    }
    let nperseg = welch_segment_length(x.len(), nperseg)?;
    let step = nperseg / 2;
    let win = hanning(nperseg);
    let bins = nperseg / 2 + 1;
    let mut pxx = vec![0.0; bins];
    let mut pyy = vec![0.0; bins];
    let mut pxy_re = vec![0.0; bins];
    let mut pxy_im = vec![0.0; bins];
    let mut segments = 0usize;
    let mut start = 0usize;
    while start + nperseg <= x.len() {
        let (fx_re, fx_im) = windowed_rfft(&x[start..start + nperseg], &win, bins);
        let (fy_re, fy_im) = windowed_rfft(&y[start..start + nperseg], &win, bins);
        for b in 0..bins {
            pxx[b] += fx_re[b] * fx_re[b] + fx_im[b] * fx_im[b];
            pyy[b] += fy_re[b] * fy_re[b] + fy_im[b] * fy_im[b];
            pxy_re[b] += fx_re[b] * fy_re[b] + fx_im[b] * fy_im[b];
            pxy_im[b] += fx_re[b] * fy_im[b] - fx_im[b] * fy_re[b];
        }
        segments += 1;
        start += step;
    }
    let freqs: Vec<f64> = (0..bins).map(|b| b as f64 * fs / nperseg as f64).collect();
    let mut re = vec![0.0; bins];
    let mut im = vec![0.0; bins];
    let mut coherence = vec![0.0; bins];
    for b in 0..bins {
        if pxx[b] > 0.0 {
            re[b] = pxy_re[b] / pxx[b];
            im[b] = pxy_im[b] / pxx[b];
        }
        let denom = pxx[b] * pyy[b];
        if denom > 0.0 {
            coherence[b] = (pxy_re[b] * pxy_re[b] + pxy_im[b] * pxy_im[b]) / denom;
        }
    }
    Ok(Frf {
        freqs,
        re,
        im,
        coherence,
        segments,
    })
}

fn half_power_crossing(
    freqs: &[f64],
    mag: &[f64],
    i: usize,
    target: f64,
    direction: isize,
) -> Option<f64> {
    let n = mag.len() as isize;
    let mut j = i as isize;
    while 0 < j && j < n - 1 && mag[j as usize] > target {
        j += direction;
    }
    if mag[j as usize] > target {
        return None;
    }
    let prev = (j - direction) as usize;
    let (f0, f1) = (freqs[prev], freqs[j as usize]);
    let (m0, m1) = (mag[prev], mag[j as usize]);
    if m0 == m1 {
        return Some(f1);
    }
    Some(f0 + (f1 - f0) * (m0 - target) / (m0 - m1))
}

pub fn half_power_damping(freqs: &[f64], mag: &[f64], i_peak: usize) -> Option<f64> {
    let target = mag[i_peak] / SQRT_2;
    let lo = half_power_crossing(freqs, mag, i_peak, target, -1)?;
    let hi = half_power_crossing(freqs, mag, i_peak, target, 1)?;
    if freqs[i_peak] <= 0.0 {
        return None;
    }
    Some((hi - lo) / (2.0 * freqs[i_peak]))
}

pub fn find_modes(frf: &Frf, lo: f64, hi: f64) -> Result<Vec<DifferentialMode>, String> {
    let band: Vec<usize> = (0..frf.freqs.len())
        .filter(|&i| frf.freqs[i] >= lo && frf.freqs[i] <= hi)
        .collect();
    if !band.iter().any(|&i| frf.coherence[i] >= COHERENCE_MIN) {
        let max_coh = band
            .iter()
            .map(|&i| frf.coherence[i])
            .fold(0.0_f64, f64::max);
        return Err(format!(
            "no coherent differential response in {lo:.0}..{hi:.0} Hz \
             (max coherence {max_coh:.2}); raise AMPLITUDE or check that the \
             buzz really ran anti-phase on this pair"
        ));
    }
    let mag = frf.magnitude();
    let mut candidates: Vec<usize> = Vec::new();
    if band.len() > 2 {
        for &i in &band[1..band.len() - 1] {
            if mag[i] > mag[i - 1] && mag[i] >= mag[i + 1] && frf.coherence[i] >= COHERENCE_MIN {
                candidates.push(i);
            }
        }
    }
    candidates.sort_by(|&a, &b| {
        mag[b]
            .partial_cmp(&mag[a])
            .unwrap_or(core::cmp::Ordering::Equal)
    });
    let mut modes: Vec<DifferentialMode> = Vec::new();
    for i in candidates {
        let near_existing = modes
            .iter()
            .any(|m| (frf.freqs[i] - m.freq_hz).abs() < 3.0_f64.max(0.05 * m.freq_hz));
        if near_existing {
            continue;
        }
        modes.push(DifferentialMode {
            freq_hz: frf.freqs[i],
            gain: mag[i],
            gain_db: 20.0 * libm::log10(mag[i]),
            damping: half_power_damping(&frf.freqs, &mag, i),
            coherence: frf.coherence[i],
        });
        if modes.len() >= MAX_MODES {
            break;
        }
    }
    modes.sort_by(|a, b| a.freq_hz.partial_cmp(&b.freq_hz).unwrap());
    Ok(modes)
}

/// A mode-projected series: capture channels weighted by one frame row.
/// The weights are the RAW-drive-frame columns (the `spatial_frame`
/// convention — each motor's invert sign is already folded into its
/// column), so positions and torque are taken unsigned here: counts are
/// scaled by 1/counts_per_mm only, never by the invert flag. Only the
/// relative sign between input and output matters for the FRF, and both
/// sides get the identical weighting.
#[derive(Debug)]
pub struct ModeSeries {
    pub cmd_mm: Vec<f64>,
    pub act_mm: Vec<f64>,
    pub torque: Vec<f64>,
}

pub fn mode_series(cap: &Scap, axes: &[String], weights: &[f64]) -> Result<ModeSeries, String> {
    if axes.len() != weights.len() {
        return Err(format!(
            "spatial frame lists {} axes but the row has {} weights",
            axes.len(),
            weights.len()
        ));
    }
    let n = cap.n_records;
    let mut cmd_mm = vec![0.0; n];
    let mut act_mm = vec![0.0; n];
    let mut torque = vec![0.0; n];
    let mut used = 0usize;
    for (idx, d) in cap.header.drives.iter().enumerate() {
        let Some(pos) = axes.iter().position(|a| a == &d.name) else {
            continue;
        };
        let w = weights[pos];
        if w == 0.0 {
            continue;
        }
        if d.counts_per_mm <= 0.0 {
            return Err(format!(
                "drive {:?} has non-positive counts_per_mm {}",
                d.name, d.counts_per_mm
            ));
        }
        let scale = w / d.counts_per_mm;
        for (out, v) in cmd_mm.iter_mut().zip(cap.read_f64(idx, "target_counts")?) {
            *out += v * scale;
        }
        for (out, v) in act_mm.iter_mut().zip(cap.read_f64(idx, "position_actual")?) {
            *out += v * scale;
        }
        for (out, v) in torque.iter_mut().zip(cap.read_f64(idx, "torque_actual")?) {
            *out += v * w;
        }
        used += 1;
    }
    if used == 0 {
        return Err(format!(
            "capture drives [{}] share no weighted axis with the spatial frame",
            cap.drive_names().join(", ")
        ));
    }
    Ok(ModeSeries {
        cmd_mm,
        act_mm,
        torque,
    })
}

/// Instrumental-variable FRF: with both `num = S_ry/S_rr` and
/// `den = S_ru/S_rr` estimated against the same noise-free reference r,
/// the ratio is `S_ry/S_ru = G(u→y)` free of the closed-loop bias a
/// direct `S_uy/S_uu` estimate picks up when u is generated inside the
/// loop. Coherence is the elementwise minimum of the two legs — a bin is
/// only trustworthy when the reference explains both signals.
pub fn complex_ratio(num: &Frf, den: &Frf) -> Result<Frf, String> {
    if num.freqs.len() != den.freqs.len() {
        return Err(format!(
            "FRF grids differ ({} vs {} bins) - both legs must share the Welch segmentation",
            num.freqs.len(),
            den.freqs.len()
        ));
    }
    let bins = num.freqs.len();
    let mut re = vec![0.0; bins];
    let mut im = vec![0.0; bins];
    let mut coherence = vec![0.0; bins];
    for b in 0..bins {
        let d2 = den.re[b] * den.re[b] + den.im[b] * den.im[b];
        if d2 > 0.0 {
            re[b] = (num.re[b] * den.re[b] + num.im[b] * den.im[b]) / d2;
            im[b] = (num.im[b] * den.re[b] - num.re[b] * den.im[b]) / d2;
        }
        coherence[b] = num.coherence[b].min(den.coherence[b]);
    }
    Ok(Frf {
        freqs: num.freqs.clone(),
        re,
        im,
        coherence,
        segments: num.segments.min(den.segments),
    })
}

/// The locked-rotor anti-resonance: at `f_b = sqrt(k_belt/m_load)/2pi`
/// the load is a perfectly tuned absorber — no applied torque can move
/// the rotor — so the torque→rotor-position FRF has a zero there. Plant
/// zeros are invariant under feedback, so the notch survives any loop
/// gain. Coherence AT the notch is naturally poor (the output is nearly
/// zero), so the gate is on the flanks a quarter octave to each side.
#[derive(Debug, Clone)]
pub struct Notch {
    pub freq_hz: f64,
    pub depth_db: f64,
    pub flank_coherence: f64,
}

pub fn find_notch(g: &Frf, lo: f64, hi: f64) -> Result<Notch, String> {
    let band: Vec<usize> = (0..g.freqs.len())
        .filter(|&i| g.freqs[i] >= lo && g.freqs[i] <= hi)
        .collect();
    if band.len() < 8 {
        return Err(format!(
            "FRF band {lo:.0}..{hi:.0} Hz holds only {} bins; sweep longer or widen the band",
            band.len()
        ));
    }
    let mag = g.magnitude();
    let db = |v: f64| 20.0 * libm::log10(v.max(1e-12));
    let mut mags_db: Vec<f64> = band.iter().map(|&i| db(mag[i])).collect();
    let i_min = band[1..band.len() - 1]
        .iter()
        .enumerate()
        .min_by(|(_, &a), (_, &b)| {
            mag[a]
                .partial_cmp(&mag[b])
                .unwrap_or(core::cmp::Ordering::Equal)
        })
        .map(|(k, _)| band[k + 1])
        .ok_or_else(|| "notch search band is empty".to_string())?;
    // Depth relative to the band median: a real anti-resonance carves a
    // deep local hole; a flat coherent response has nothing to report.
    mags_db.sort_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
    let median_db = mags_db[mags_db.len() / 2];
    let depth_db = median_db - db(mag[i_min]);
    // Flank coherence: quarter-octave to each side of the candidate.
    let f0 = g.freqs[i_min];
    let flank = |target: f64| -> f64 {
        let i = (0..g.freqs.len())
            .min_by(|&a, &b| {
                (g.freqs[a] - target)
                    .abs()
                    .partial_cmp(&(g.freqs[b] - target).abs())
                    .unwrap_or(core::cmp::Ordering::Equal)
            })
            .unwrap_or(i_min);
        g.coherence[i]
    };
    let flank_coherence = flank(f0 / 1.19).min(flank(f0 * 1.19));
    // Parabolic refinement on log-magnitude through the minimum bin.
    let freq_hz = if i_min > 0 && i_min + 1 < g.freqs.len() {
        let (m0, m1, m2) = (db(mag[i_min - 1]), db(mag[i_min]), db(mag[i_min + 1]));
        let denom = m0 - 2.0 * m1 + m2;
        if denom > 0.0 {
            let delta = 0.5 * (m0 - m2) / denom;
            g.freqs[i_min] + delta.clamp(-0.5, 0.5) * (g.freqs[1] - g.freqs[0])
        } else {
            g.freqs[i_min]
        }
    } else {
        g.freqs[i_min]
    };
    Ok(Notch {
        freq_hz,
        depth_db,
        flank_coherence,
    })
}
