//! GSM 06.20 clause 4.1.8.1–4.1.8.4 — the open-loop long-term
//! predictor lag search over the spectrally weighted input speech
//! `y(n)`: per-subframe integer peaks (4.1.8.1), the fractional /
//! submultiple / multiple candidate-peak lists with the
//! harmonic-noise-weighting parameters (4.1.8.2), the frame lag
//! trajectory search under the delta-coding constraints (4.1.8.3)
//! and the voicing-mode decision (4.1.8.4).
//!
//! Everything here is a pure function of the weighted speech; the
//! closed-loop search (4.1.8.5) that needs the long-term filter
//! state lives with the encoder proper.
//!
//! ## Readings pinned from the printed clause
//!
//! * eqs. (51)–(53)/(63)–(65)/(73)–(75)/(85)–(87): the correlation
//!   interpolation is `C_I(k) = Σ_{i=0}^{5} g_j(i)·C(⌈k⌉ − 3 + i)`
//!   with `j = 6(⌈k⌉ − k)` — a **ceiling**, so phase `j` of the
//!   6th-order filter moves the lag *down* from `⌈k⌉`;
//! * eq. (67)/(89): the submultiple/multiple acceptance threshold
//!   prints as `C_I²/G_I > R − R/10^x, x = 7,5·log10(R/(R −
//!   C²peak/Gpeak))`. Read literally (`10^x`) the candidate would
//!   need a prediction gain of 7,5× the peak's in dB — never true —
//!   so the exponent is taken as the dB form `10^(x/10)`: the
//!   candidate's open-loop prediction gain must exceed **0,75 × the
//!   integer peak's gain in dB**;
//! * clause 4.1.8.3 steps 3d/4d maximise `C_I/√G_I` (negative
//!   `C_I` allowed) while the anchors rank by `C_I²/G_I`;
//! * the forward range `−7..+6` / backward `−6..+7` levels leaves
//!   the ±1-level closed-loop refinement inside the `−8..+7` delta
//!   codes of annex A.1.4.

use super::tables::{INTERP_FILTER_6, LAG_TABLE};
use super::{HR_SUBFRAMES, HR_SUBFRAME_SAMPLES};

/// Samples per subframe.
const NS: usize = HR_SUBFRAME_SAMPLES;

/// Weighted-speech history the search keeps in front of a frame:
/// `Lmax + Pg/2 + 1` covers the deepest correlation tap (eq. (42),
/// `k ≤ Lmax + Pg/2 − 1 = 144`) and the harmonic-weighting delay.
pub(super) const Y_HIST: usize = 147;

/// Clause 3.2: `Lmin` = 21, `Lmax` = 142 samples.
const LMIN: i32 = 21;
const LMAX: i32 = 142;

/// Eq. (42)/(43) correlation-lag range `Lmin − Pg/2 ..= Lmax + Pg/2 − 1`.
const KMIN: i32 = LMIN - 3;
const KMAX: i32 = LMAX + 3 - 1;
const NK: usize = (KMAX - KMIN + 1) as usize;

/// Per-subframe correlation arrays `C(k,m)`, `G(k,m)` over
/// `k = KMIN..=KMAX`, and the subframe energy `R(0,m)`.
struct SubCorr {
    c: [f64; NK],
    g: [f64; NK],
    r: f64,
}

impl SubCorr {
    fn compute(y: &[f64], start: usize) -> Self {
        let seg = &y[start..start + NS];
        let mut c = [0f64; NK];
        let mut g = [0f64; NK];
        for (slot, k) in (KMIN..=KMAX).enumerate() {
            let k = k as usize;
            let mut cc = 0.0;
            let mut gg = 0.0;
            for n in 0..NS {
                let past = y[start + n - k];
                cc += seg[n] * past;
                gg += past * past;
            }
            c[slot] = cc;
            g[slot] = gg;
        }
        let r = seg.iter().map(|v| v * v).sum();
        Self { c, g, r }
    }

    #[inline]
    fn at(&self, k: i32) -> (f64, f64) {
        let i = (k - KMIN) as usize;
        (self.c[i], self.g[i])
    }

    /// Eqs. (51)/(52): interpolated `(C_I, G_I)` at a lag in
    /// 1/6-sample units (any value with `⌈k⌉ ∈ [Lmin, Lmax]`).
    fn interp(&self, k6: i32) -> (f64, f64) {
        let kc = k6.div_euclid(6) + if k6.rem_euclid(6) == 0 { 0 } else { 1 };
        let j = (6 * kc - k6) as usize;
        let mut ci = 0.0;
        let mut gi = 0.0;
        for (i, row) in INTERP_FILTER_6.iter().enumerate() {
            let w = row[j] as f64 / 32768.0;
            let (c, g) = self.at(kc - 3 + i as i32);
            ci += w * c;
            gi += w * g;
        }
        (ci, gi)
    }

    /// `C²/G` at an integer lag when both are positive, else `None`.
    fn ratio_pos(&self, k: i32) -> Option<f64> {
        let (c, g) = self.at(k);
        if c > 0.0 && g > 0.0 {
            Some(c * c / g)
        } else {
            None
        }
    }
}

/// Open-loop prediction gain `10·log10(R / (R − x))` in dB for a
/// prediction term `x = C²/G` (eqs. (47)/(94)); saturates when the
/// prediction is perfect.
fn gain_db(r: f64, x: f64) -> f64 {
    let rem = r - x;
    if r <= 0.0 {
        0.0
    } else if rem <= 0.0 {
        f64::INFINITY
    } else {
        10.0 * (r / rem).log10()
    }
}

/// One candidate peak of the 4.1.8.2 list: its allowable-lag table
/// level and the interpolated correlation terms.
#[derive(Clone, Copy, Debug)]
struct Peak {
    level: usize,
    ci: f64,
    gi: f64,
}

/// Index of an allowable lag (1/6-sample units) in [`LAG_TABLE`].
#[cfg(test)]
fn lag_level(k6: i32) -> Option<usize> {
    LAG_TABLE.binary_search(&(k6 as i16)).ok()
}

/// The open-loop decisions for one frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct OpenLoop {
    /// Annex A.1.1 voicing mode (0 = unvoiced).
    pub mode: u8,
    /// Selected frame lag trajectory as allowable-lag levels
    /// (meaningful for `mode != 0`).
    pub levels: [usize; HR_SUBFRAMES],
    /// Clause 4.1.9 harmonic-noise-weighting lag `L_pitch,m`
    /// (1/6-sample units) and coefficient `λ_hnw,m` per subframe.
    pub lpitch: [i32; HR_SUBFRAMES],
    pub lambda: [f64; HR_SUBFRAMES],
}

/// Run clauses 4.1.8.1–4.1.8.4 on one frame. `y` holds the
/// weighted speech: [`Y_HIST`] history samples followed by the
/// frame's 160.
pub(super) fn open_loop_search(y: &[f64]) -> OpenLoop {
    debug_assert!(y.len() >= Y_HIST + HR_SUBFRAMES * NS);
    let corr: Vec<SubCorr> = (0..HR_SUBFRAMES)
        .map(|m| SubCorr::compute(y, Y_HIST + m * NS))
        .collect();

    // 4.1.8.1 steps 2-6: integer peaks.
    let mut peak0 = [(LMIN, 0.0f64, 1.0f64); HR_SUBFRAMES];
    for (m, sc) in corr.iter().enumerate() {
        let mut best: Option<(f64, i32)> = None;
        for k in LMIN..=LMAX {
            if let Some(v) = sc.ratio_pos(k) {
                if best.map_or(true, |(b, _)| v > b) {
                    best = Some((v, k));
                }
            }
        }
        if let Some((_, k)) = best {
            let (c, g) = sc.at(k);
            peak0[m] = (k, c, g);
        }
    }

    // Step 7/8: frame prediction gain and the unvoiced decision
    // (eqs. (47)/(96)).
    let r_sum: f64 = corr.iter().map(|s| s.r).sum();
    let e_sum: f64 = corr
        .iter()
        .zip(peak0.iter())
        .map(|(s, &(_, c, g))| s.r - c * c / g)
        .sum();
    let pv = gain_db(r_sum, r_sum - e_sum);
    let unvoiced = OpenLoop {
        mode: 0,
        levels: [0; HR_SUBFRAMES],
        lpitch: [LMIN * 6; HR_SUBFRAMES],
        lambda: [0.0; HR_SUBFRAMES],
    };
    if pv.is_nan() || pv < 1.7 {
        return unvoiced;
    }

    // 4.1.8.2: candidate peak lists + harmonic weighting parameters.
    let mut peaks: Vec<Vec<Peak>> = Vec::with_capacity(HR_SUBFRAMES);
    let mut lpitch = [LMIN * 6; HR_SUBFRAMES];
    let mut lambda = [0.0f64; HR_SUBFRAMES];
    for m in 0..HR_SUBFRAMES {
        let sc = &corr[m];
        let (l0, c0, g0) = peak0[m];
        let mut list: Vec<Peak> = Vec::new();

        // Step 3: allowable lags within ±1 (exclusive) of the
        // integer peak.
        let Some(first) = best_allowable(sc, 6 * (l0 - 1), 6 * (l0 + 1)) else {
            peaks.push(list);
            continue;
        };
        list.push(first);
        // Threshold of eq. (67)/(89): 0,75 × the integer peak's
        // open-loop gain (dB).
        let thresh_db = 0.75 * gain_db(sc.r, c0 * c0 / g0);
        let accept = |p: &Peak| gain_db(sc.r, p.ci * p.ci / p.gi) > thresh_db;

        // Steps 4-11: submultiples of the fractional peak.
        let l1 = LAG_TABLE[first.level] as f64 / 6.0;
        let mut j = 2;
        loop {
            let k1 = (l1 / j as f64).round() as i32;
            if k1 < LMIN {
                break;
            }
            if let Some(p) = neighbourhood_peak(sc, k1) {
                if accept(&p) {
                    list.push(p);
                }
            }
            j += 1;
        }

        // Step 12/13: full-resolution search around the shortest
        // lag found → L_pitch,m and λ_hnw,m.
        let shortest = LAG_TABLE[list.last().unwrap().level] as i32;
        let lo = (LMIN * 6 - 1).max(shortest - 6);
        let hi = (LMAX * 6 + 1).min(shortest + 6);
        let mut best: Option<(f64, i32, f64, f64)> = None;
        for k6 in lo + 1..hi {
            let (ci, gi) = sc.interp(k6);
            if ci > 0.0 && gi > 0.0 {
                let v = ci * ci / gi;
                if best.map_or(true, |(b, ..)| v > b) {
                    best = Some((v, k6, ci, gi));
                }
            }
        }
        if let Some((_, k6, ci, gi)) = best {
            lpitch[m] = k6;
            lambda[m] = 0.4 * ci / gi;
        }

        // Steps 14-21: multiples of L_pitch.
        let lp = lpitch[m] as f64 / 6.0;
        let mut j = 2;
        loop {
            let k1 = (lp * j as f64).round() as i32;
            if k1 > LMAX {
                break;
            }
            if let Some(p) = neighbourhood_peak(sc, k1) {
                if accept(&p) {
                    list.push(p);
                }
            }
            j += 1;
        }
        peaks.push(list);
    }

    // 4.1.8.3: frame lag trajectory search.
    let mut trajectories: Vec<([usize; HR_SUBFRAMES], f64)> = Vec::new();
    for m in 0..HR_SUBFRAMES {
        let mut anchors = 0;
        let mut ranked: Vec<Peak> = peaks[m].clone();
        ranked.sort_by(|a, b| {
            (b.ci * b.ci / b.gi)
                .partial_cmp(&(a.ci * a.ci / a.gi))
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        for p in ranked {
            if anchors == 2 {
                break;
            }
            // "not crossed by a trajectory evaluated previously".
            if trajectories.iter().any(|(t, _)| t[m] == p.level) {
                continue;
            }
            anchors += 1;
            let mut t = [0usize; HR_SUBFRAMES];
            t[m] = p.level;
            let mut err = corr[m].r - p.ci * p.ci / p.gi;
            // Forward search.
            for s in m + 1..HR_SUBFRAMES {
                let base = t[s - 1] as i32;
                let (lvl, e) = range_best(&corr[s], base - 7, base + 6);
                t[s] = lvl;
                err += e;
            }
            // Backward search.
            for s in (0..m).rev() {
                let base = t[s + 1] as i32;
                let (lvl, e) = range_best(&corr[s], base - 6, base + 7);
                t[s] = lvl;
                err += e;
            }
            trajectories.push((t, err));
        }
    }
    let Some(&(levels, _)) = trajectories
        .iter()
        .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
    else {
        return unvoiced;
    };

    // 4.1.8.4: voicing mode from the per-subframe gains of the
    // selected trajectory (eqs. (94)/(97)-(99)).
    let mut pm = [0f64; HR_SUBFRAMES];
    for m in 0..HR_SUBFRAMES {
        let (ci, gi) = corr[m].interp(LAG_TABLE[levels[m]] as i32);
        let x = if gi > 0.0 { ci * ci / gi } else { 0.0 };
        pm[m] = gain_db(corr[m].r, x);
    }
    let mode = if pm.iter().all(|&p| p >= 7.0) {
        3
    } else if pm.iter().all(|&p| p >= 3.5) {
        2
    } else {
        1
    };
    OpenLoop {
        mode,
        levels,
        lpitch,
        lambda,
    }
}

/// Step 3/9/19 helper: the allowable lag strictly inside
/// `(lo6, hi6)` (1/6 units) maximising `C_I²/G_I` with both terms
/// positive.
fn best_allowable(sc: &SubCorr, lo6: i32, hi6: i32) -> Option<Peak> {
    let mut best: Option<(f64, Peak)> = None;
    let start = LAG_TABLE.partition_point(|&l| (l as i32) <= lo6);
    for (off, &l) in LAG_TABLE[start..].iter().enumerate() {
        let k6 = l as i32;
        if k6 >= hi6 {
            break;
        }
        let (ci, gi) = sc.interp(k6);
        if ci > 0.0 && gi > 0.0 {
            let v = ci * ci / gi;
            if best.map_or(true, |(b, _)| v > b) {
                best = Some((
                    v,
                    Peak {
                        level: start + off,
                        ci,
                        gi,
                    },
                ));
            }
        }
    }
    best.map(|(_, p)| p)
}

/// Steps 7-9 / 17-19: the integer `C²/G` maximum within ±3 of `k1`
/// (clipped to the allowable range), rejected unless `C > 0`,
/// `G > 0` and it is a local peak against both integer neighbours;
/// then the best allowable lag within ±1 (exclusive) of it.
fn neighbourhood_peak(sc: &SubCorr, k1: i32) -> Option<Peak> {
    let lo = LMIN.max(k1 - 3);
    let hi = LMAX.min(k1 + 3);
    let mut best: Option<(f64, i32)> = None;
    for k in lo..=hi {
        let (c, g) = sc.at(k);
        if g > 0.0 {
            let v = c * c / g;
            if best.map_or(true, |(b, _)| v > b) {
                best = Some((v, k));
            }
        }
    }
    let (v, k) = best?;
    let (c, g) = sc.at(k);
    if c <= 0.0 || g <= 0.0 {
        return None;
    }
    // Eqs. (60)/(61): peak test against the neighbours.
    let side = |kk: i32| {
        let (c, g) = sc.at(kk);
        if g > 0.0 {
            c * c / g
        } else {
            0.0
        }
    };
    if side(k - 1) > v || side(k + 1) > v {
        return None;
    }
    best_allowable(sc, 6 * (k - 1), 6 * (k + 1))
}

/// Steps 3b-3d / 4b-4d: within the clipped level range, the
/// allowable lag maximising `C_I/√G_I` (negative `C_I` allowed);
/// returns the level and the open-loop error `R − C_I²/G_I` it
/// contributes.
fn range_best(sc: &SubCorr, lo: i32, hi: i32) -> (usize, f64) {
    let lo = lo.max(0) as usize;
    let hi = (hi.min(LAG_TABLE.len() as i32 - 1)) as usize;
    let mut best: Option<(f64, usize, f64)> = None;
    for (level, &lag) in LAG_TABLE.iter().enumerate().take(hi + 1).skip(lo) {
        let (ci, gi) = sc.interp(lag as i32);
        let (score, err) = if gi > 0.0 {
            (ci / gi.sqrt(), sc.r - ci * ci / gi)
        } else {
            (f64::NEG_INFINITY, sc.r)
        };
        if best.map_or(true, |(b, ..)| score > b) {
            best = Some((score, level, err));
        }
    }
    let (_, level, err) = best.unwrap_or((0.0, lo, sc.r));
    (level, err)
}

/// Table level of an integer lag (all integers 21..=142 are
/// allowable lags).
#[cfg(test)]
fn integer_level(lag: i32) -> usize {
    lag_level(lag * 6).expect("integer lags are allowable")
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A clean periodic signal: the search must land on its period
    /// at every subframe and call it strongly voiced.
    #[test]
    fn periodic_input_is_strongly_voiced_at_its_period() {
        let period: i32 = 60;
        let mut y = vec![0f64; Y_HIST + 160];
        for (n, v) in y.iter_mut().enumerate() {
            let ph = (n as i32 % period) as f64 / period as f64;
            // A pulse-train-ish waveform with some harmonics.
            *v =
                (ph * std::f64::consts::TAU).sin() + 0.5 * (2.0 * ph * std::f64::consts::TAU).cos();
        }
        let ol = open_loop_search(&y);
        assert_eq!(ol.mode, 3, "{ol:?}");
        // The 6th-order correlation interpolator may put the peak
        // of C_I²/G_I half a sample off the integer period.
        for &lvl in &ol.levels {
            assert!((LAG_TABLE[lvl] as i32 - period * 6).abs() <= 3, "{ol:?}");
        }
        for &lp in &ol.lpitch {
            assert!((lp - period * 6).abs() <= 3, "{ol:?}");
        }
        for &l in &ol.lambda {
            assert!(
                l > 0.3 && l <= 0.41,
                "λ_hnw for a periodic signal ≈ 0,4: {l}"
            );
        }
    }

    /// White noise has no long-term prediction gain: MODE 0.
    #[test]
    fn noise_input_is_unvoiced() {
        let mut state = 0x1234_5678u32;
        let mut y = vec![0f64; Y_HIST + 160];
        for v in y.iter_mut() {
            state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            *v = (state >> 8) as f64 / (1u32 << 24) as f64 - 0.5;
        }
        let ol = open_loop_search(&y);
        assert_eq!(ol.mode, 0);
    }

    /// Silence never panics and is unvoiced.
    #[test]
    fn silence_is_unvoiced() {
        let y = vec![0f64; Y_HIST + 160];
        assert_eq!(open_loop_search(&y).mode, 0);
    }

    /// Consecutive trajectory lags always stay codable by the annex
    /// A.1.4 delta range even before closed-loop refinement.
    #[test]
    fn trajectory_respects_delta_coding_margin() {
        let mut y = vec![0f64; Y_HIST + 160];
        let len = y.len() as f64;
        for (n, v) in y.iter_mut().enumerate() {
            // Period sliding from 40 to 48 samples across the frame.
            let p = 40.0 + 8.0 * (n as f64 / len);
            *v = ((n as f64) * std::f64::consts::TAU / p).sin();
        }
        let ol = open_loop_search(&y);
        if ol.mode != 0 {
            for w in ol.levels.windows(2) {
                let d = w[1] as i32 - w[0] as i32;
                assert!((-7..=6).contains(&d), "{ol:?}");
            }
        }
    }

    #[test]
    fn integer_levels_exist() {
        assert_eq!(integer_level(21), 0);
        assert_eq!(integer_level(142), 255);
    }

    /// The correlation interpolator reproduces the eq. (51)/(53)
    /// ceiling convention: at an integer lag the phase is 0 and the
    /// centre tap sits on that lag.
    #[test]
    fn interpolation_phase_convention() {
        let mut y = vec![0f64; Y_HIST + 160];
        y[Y_HIST - 30] = 1.0;
        y[Y_HIST + 5] = 1.0;
        let sc = SubCorr::compute(&y, Y_HIST);
        // C(35) = 1 from the pulse pair; C_I at exactly 35 samples
        // is g_0(3)·C(35) plus zero neighbours.
        let (ci, _) = sc.interp(35 * 6);
        assert!((ci - INTERP_FILTER_6[3][0] as f64 / 32768.0).abs() < 1e-12);
        // 34 5/6 samples: ⌈k⌉ = 35, j = 1 — taps g_1(i) on C(32..=37).
        let (ci, _) = sc.interp(35 * 6 - 1);
        assert!((ci - INTERP_FILTER_6[3][1] as f64 / 32768.0).abs() < 1e-12);
    }
}
