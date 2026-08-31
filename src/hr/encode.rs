//! GSM 06.20 half-rate encoder — the frame-parameter analysis
//! chain of clauses 4.1.1–4.1.6 (ETSI EN 300 969, staged): input
//! high-pass filtering, segmentation, the FLAT covariance-lattice
//! reflection-coefficient derivation, the three-segment
//! reflection-coefficient vector quantization via the AFLAT
//! recursion, frame-energy (R0) coding, and the soft-interpolation
//! (INT_LPC) decision.
//!
//! **Scope.** This is the encoder's *groundwork* arc: the per-frame
//! parameters `R0`, `LPC1..LPC3` and `INT_LPC`. The per-subframe
//! excitation analysis (open/closed-loop lag search, voicing mode
//! selection, VSELP code search, gain quantization — clauses
//! 4.1.7–4.1.11) is the follow-up arc.
//!
//! Like the decoder (see `hr::decode`), the chain implements the
//! printed floating-point equations over the staged ROM tables in
//! double precision; the bit-exact arithmetic lives in the unstaged
//! GSM 06.06 ANSI-C, so the achievable validation bar is measured
//! parameter agreement against the staged GSM 06.07 encoder
//! references, pinned in `tests/conformance_hr_encode_params.rs`.

use super::decode::{step_down, step_up, vq_index};
use super::tables::*;
use super::HR_FRAME_SAMPLES;

/// Short-term predictor order (annex A.2 `Np`).
const NP: usize = 10;

/// Analysis buffer length (clause 4.1.2: *"the previous 195 input
/// high pass filtered speech samples"*).
const BUF: usize = 195;

/// Clause 4.1.2 `NA` = 170, the analysis-interval length; the
/// clause 4.1.3 covariance sum runs `n = Np ..= NA`.
const NA: usize = 170;

/// The frame-level parameters the clause 4.1.1–4.1.6 chain derives.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrameAnalysis {
    /// Clause 4.1.5 eq. (29): the 5-bit frame-energy code.
    pub r0: u8,
    /// Clause 4.1.4: the three reflection-coefficient VQ codes.
    pub lpc1: u16,
    pub lpc2: u16,
    pub lpc3: u8,
    /// Clause 4.1.6: the soft-interpolation bit.
    pub int_lpc: bool,
}

/// Clause 4.1.1: fourth-order pole-zero high-pass filter (120 Hz),
/// two cascaded biquads with an incorporated gain of 0,5. The Q15
/// coefficients are the staged ROM words ([`HIGHPASS_COEFFS`],
/// storage order per section `(b0, b1, b2, a2, a1)`), which match
/// the printed equations (3)/(4) exactly.
#[derive(Debug, Clone, Default)]
struct HighPass {
    x1: [f64; 2],
    y1: [f64; 2],
    y2: [f64; 2],
    x2in: [f64; 2],
}

impl HighPass {
    fn process(&mut self, x: f64) -> f64 {
        let c = |i: usize| HIGHPASS_COEFFS[i] as f64 / 32768.0;
        // Section 1 (eq. (3)): y1 = b10 x + b11 x' + b12 x'' +
        // 2 a11 y1' + 2 a12 y1''. The Q15 ROM words store the
        // recursive coefficients HALVED so |a| < 1 fits Q15; the
        // doubled values give the double real pole at 0,926 (and
        // 0,965 for section 2) that realises the printed 120 Hz
        // high-pass — as stored they would describe a 1 kHz
        // resonator, not a high-pass.
        let y1 = c(0) * x
            + c(1) * self.x1[0]
            + c(2) * self.x1[1]
            + 2.0 * c(4) * self.y1[0]
            + 2.0 * c(3) * self.y1[1];
        self.x1 = [x, self.x1[0]];
        self.y1 = [y1, self.y1[0]];
        // Section 2 (eq. (4)).
        let y2 = c(5) * y1
            + c(6) * self.x2in[0]
            + c(7) * self.x2in[1]
            + 2.0 * c(9) * self.y2[0]
            + 2.0 * c(8) * self.y2[1];
        self.x2in = [y1, self.x2in[0]];
        self.y2 = [y2, self.y2[0]];
        y2
    }
}

/// The clause 4.1.1–4.1.6 frame analyzer.
#[derive(Debug, Clone)]
pub struct HrAnalyzer {
    hp: HighPass,
    /// Clause 4.1.2 sample buffer `s(0..=194)`, `s(0)` oldest.
    buf: [f64; BUF],
    /// Previous frame's quantized direct-form set + INT_LPC state
    /// (clause 4.1.6).
    prev_alpha: [f64; NP],
}

impl Default for HrAnalyzer {
    fn default() -> Self {
        Self::new()
    }
}

impl HrAnalyzer {
    /// Fresh analyzer (all-zero filter and buffer state).
    pub fn new() -> Self {
        Self {
            hp: HighPass::default(),
            buf: [0.0; BUF],
            prev_alpha: [0.0; NP],
        }
    }

    /// Reset to the initial state.
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Analyze one 160-sample frame (13-bit left-justified i16, the
    /// GSM 06.07 input convention) into the frame-level parameter
    /// codes.
    // The covariance/window/interpolation loops index multiple
    // arrays by the same symmetric (i, k) pair; iterator forms
    // obscure the spec equations.
    #[allow(clippy::needless_range_loop)]
    pub fn analyze_frame(&mut self, samples: &[i16; HR_FRAME_SAMPLES]) -> FrameAnalysis {
        // Clauses 4.1.1/4.1.2: high-pass + buffer shift.
        self.buf.copy_within(HR_FRAME_SAMPLES.., 0);
        for (i, &s) in samples.iter().enumerate() {
            self.buf[BUF - HR_FRAME_SAMPLES + i] = self.hp.process(s as f64 / 32768.0);
        }

        // Clause 4.1.3 STEP 1: covariance matrix (eq. (5)).
        let mut phi = [[0f64; NP + 1]; NP + 1];
        for i in 0..=NP {
            for k in i..=NP {
                let mut acc = 0.0;
                for n in NP..=NA {
                    acc += self.buf[n + 24 - i] * self.buf[n + 24 - k];
                }
                phi[i][k] = acc;
                phi[k][i] = acc;
            }
        }

        // Clause 4.1.5 eqs. (27)-(29): frame-energy code. Rmax is
        // the square of the maximum 13-bit sample amplitude (4096)
        // while the samples are carried left-justified in 16-bit
        // words, so the normalised-domain energy scales by
        // (32768/4096)^2 = 64 — pinned by the staged references
        // (without it every R0 code sits exactly 9 below the
        // corpus, i.e. -18,06 dB).
        let r0_energy = 64.0 * (phi[0][0] + phi[NP][NP]) / 320.0;
        let r0 = if r0_energy <= 0.0 {
            0
        } else {
            let rdb = 10.0 * r0_energy.log10();
            (((rdb + 66.0) / 2.0).round()).clamp(0.0, 31.0) as u8
        };

        // Clause 4.1.3 STEP 2: window (table 1, staged Q31 words).
        let mut phiw = phi;
        for i in 0..=NP {
            for k in 0..=NP {
                let d = i.abs_diff(k);
                if d > 0 {
                    phiw[i][k] *= FLAT_SST_COEFFS[d - 1] as f64 / 2147483648.0;
                }
            }
        }

        // Clause 4.1.3 STEPs 3-9: the FLAT recursion (eqs. (7)-(13)).
        let mut f = [[0f64; NP]; NP];
        let mut b = [[0f64; NP]; NP];
        let mut c = [[0f64; NP]; NP];
        for i in 0..NP {
            for k in 0..NP {
                f[i][k] = phiw[i][k];
                b[i][k] = phiw[i + 1][k + 1];
                c[i][k] = phiw[i][k + 1];
            }
        }
        let mut r_opt = [0f64; NP];
        for j in 1..=NP {
            let m = NP - j;
            let num = c[0][0] + c[m][m];
            let den = f[0][0] + b[0][0] + f[m][m] + b[m][m];
            let rj = if den.abs() > 1e-30 {
                (-2.0 * num / den).clamp(-0.999_99, 0.999_99)
            } else {
                0.0
            };
            r_opt[j - 1] = rj;
            if j == NP {
                break;
            }
            let bound = NP - j;
            let (fp, bp, cp) = (f, b, c);
            for i in 0..bound {
                for k in 0..bound {
                    f[i][k] = fp[i][k] + rj * (cp[i][k] + cp[k][i]) + rj * rj * bp[i][k];
                    b[i][k] = bp[i + 1][k + 1]
                        + rj * (cp[i + 1][k + 1] + cp[k + 1][i + 1])
                        + rj * rj * fp[i + 1][k + 1];
                    c[i][k] =
                        cp[i][k + 1] + rj * (bp[i][k + 1] + fp[i][k + 1]) + rj * rj * cp[k + 1][i];
                }
            }
        }

        // Clause 4.1.4.1: autocorrelation sequence R(i) of the
        // optimal reflection coefficients (unit-energy model), then
        // the three-segment AFLAT VQ search.
        let rr = rc_to_autocorr(&r_opt);
        let (lpc1, lpc2, lpc3, rq) = vq_search(&rr);

        // Clause 4.1.6: the soft-interpolation decision. Build the
        // interpolated and uninterpolated per-subframe sets from the
        // previous and current quantized coefficients, inverse
        // filter the frame, and pick the lower residual energy
        // (ties go to uninterpolated).
        let alpha_cur = step_up(&rq);
        let mut e_interp = 0.0;
        let mut e_flat = 0.0;
        for sf in 0..4 {
            let a_int = if sf == 3 {
                alpha_cur
            } else {
                let del = SOFT_INTERP_CURRENT[sf] as f64 / 32768.0;
                let mut a = [0f64; NP];
                for i in 0..NP {
                    a[i] = self.prev_alpha[i] + del * (alpha_cur[i] - self.prev_alpha[i]);
                }
                if step_down(&a).is_some() {
                    a
                } else if sf == 0 {
                    self.prev_alpha
                } else {
                    alpha_cur
                }
            };
            let a_unint = if sf == 0 { self.prev_alpha } else { alpha_cur };
            // Inverse filter the frame's samples (the newest 160 in
            // the buffer) over this subframe.
            let base = BUF - HR_FRAME_SAMPLES + sf * 40;
            for n in 0..40 {
                let idx = base + n;
                let mut p_int = 0.0;
                let mut p_flat = 0.0;
                for i in 0..NP {
                    let past = self.buf[idx - 1 - i];
                    p_int += a_int[i] * past;
                    p_flat += a_unint[i] * past;
                }
                let s = self.buf[idx];
                e_interp += (s - p_int) * (s - p_int);
                e_flat += (s - p_flat) * (s - p_flat);
            }
        }
        let int_lpc = e_interp < e_flat;

        self.prev_alpha = alpha_cur;
        FrameAnalysis {
            r0,
            lpc1,
            lpc2,
            lpc3,
            int_lpc,
        }
    }
}

/// Reflection coefficients → normalised autocorrelation sequence
/// `R(0..=Np)` (`R(0) = 1`), the inverse Levinson recursion clause
/// 4.1.4.1 presupposes (*"Compute the autocorrelation sequence
/// R(i), from the optimal reflection coefficients"*).
fn rc_to_autocorr(r: &[f64; NP]) -> [f64; NP + 1] {
    // Build each order's direct-form coefficients A_n(z) = 1 + Σ a_k z^-k.
    // Sign pairing: the AFLAT stage recursion (eq. (21)) reduces
    // P(0) by (1 - r²) per stage exactly when R(1) = -r1·R(0), i.e.
    // A_n(z) is built with +r on the diagonal (pinned by the
    // `aflat_residual_minimum_at_source` test).
    let mut a = [[0f64; NP + 1]; NP + 1];
    for n in 1..=NP {
        a[n] = a[n - 1];
        a[n][n] = r[n - 1];
        for k in 1..n {
            a[n][k] = a[n - 1][k] + r[n - 1] * a[n - 1][n - k];
        }
    }
    let mut rr = [0f64; NP + 1];
    rr[0] = 1.0;
    for n in 1..=NP {
        let mut acc = 0.0;
        for k in 1..=n {
            acc += a[n][k] * rr[n - k];
        }
        rr[n] = -acc;
    }
    rr
}

/// One AFLAT stage update (eqs. (21)/(22)) over full-width arrays
/// (`P` indexed `0..=Np`, `V` offset by `Np-1`).
fn aflat_stage(p: &mut [f64; NP + 1], v: &mut [f64; 2 * NP - 1], rj: f64) {
    let vat = |v: &[f64; 2 * NP - 1], i: isize| v[(i + NP as isize - 1) as usize];
    let pp = *p;
    let pv = *v;
    for (i, slot) in p.iter_mut().take(NP).enumerate() {
        *slot = (1.0 + rj * rj) * pp[i] + rj * (vat(&pv, i as isize) + vat(&pv, -(i as isize)));
    }
    for i in (1 - (NP as isize))..(NP as isize - 1) {
        v[(i + NP as isize - 1) as usize] = vat(&pv, i + 1)
            + rj * rj * vat(&pv, -i - 1)
            + 2.0 * rj * pp[(i + 1).unsigned_abs().min(NP)];
    }
}

/// Clause 4.1.4/4.1.4.1: the three-segment reflection-coefficient
/// vector quantizer search — prequantizer (best four), then the
/// four associated VQ subsets — driven by the AFLAT residual `Er`.
/// Returns the three codes and the quantized reflection set.
fn vq_search(rr: &[f64; NP + 1]) -> (u16, u16, u8, [f64; NP]) {
    // Segment layout: stages, tables, sizes.
    struct Seg {
        n_coeff: usize,
        preq: &'static [i16],
        preq_rows: usize,
        vq: &'static [i16],
        subset: usize,
    }
    let segs = [
        Seg {
            n_coeff: 3,
            preq: &RC_PREQ_SEG1,
            preq_rows: 64,
            vq: &RC_VQ_SEG1,
            subset: 32,
        },
        Seg {
            n_coeff: 3,
            preq: &RC_PREQ_SEG2,
            preq_rows: 32,
            vq: &RC_VQ_SEG2,
            subset: 16,
        },
        Seg {
            n_coeff: 4,
            preq: &RC_PREQ_SEG3,
            preq_rows: 16,
            vq: &RC_VQ_SEG3,
            subset: 16,
        },
    ];

    // AFLAT initial conditions (eqs. (15)/(16)).
    let mut p = [0f64; NP + 1];
    p.copy_from_slice(rr);
    // Eq. (16)/(160): V0(i) = R(i+1) over 1-Np <= i <= Np-1 (the
    // autocorrelation is symmetric, so negative arguments fold).
    let mut v = [0f64; 2 * NP - 1];
    for i in (1 - (NP as isize))..(NP as isize) {
        v[(i + NP as isize - 1) as usize] = rr[(i + 1).unsigned_abs()];
    }

    let mut codes = [0u16; 3];
    let mut rq = [0f64; NP];
    let mut coeff_base = 0usize;
    for (kseg, seg) in segs.iter().enumerate() {
        let scalar = |idx: usize| RC_SCALAR_DEQUANT[idx] as f64 / 32768.0;
        let candidate = |table: &[i16], row: usize| -> [f64; 4] {
            let mut rs = [0f64; 4];
            for (c, slot) in rs.iter_mut().take(seg.n_coeff).enumerate() {
                *slot = scalar(vq_index(table, row * seg.n_coeff + c));
            }
            rs
        };
        let eval = |rs: &[f64; 4]| -> f64 {
            let mut ep = p;
            let mut ev = v;
            for r in rs.iter().take(seg.n_coeff) {
                aflat_stage(&mut ep, &mut ev, *r);
            }
            ep[0]
        };

        // Prequantizer pass: keep the four lowest-distortion rows.
        let mut best4: Vec<(f64, usize)> = Vec::with_capacity(seg.preq_rows);
        for q in 0..seg.preq_rows {
            best4.push((eval(&candidate(seg.preq, q)), q));
        }
        best4.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
        best4.truncate(4);

        // VQ subsets of the four selected prequantizer rows.
        let mut best = (f64::INFINITY, 0usize);
        for &(_, q) in &best4 {
            for row in q * seg.subset..(q + 1) * seg.subset {
                let er = eval(&candidate(seg.vq, row));
                if er < best.0 {
                    best = (er, row);
                }
            }
        }
        codes[kseg] = best.1 as u16;

        // Advance the lattice through this segment with the chosen
        // quantized coefficients (steps 11-14, eqs. (24)-(26)).
        let chosen = candidate(seg.vq, best.1);
        for (c, r) in chosen.iter().take(seg.n_coeff).enumerate() {
            rq[coeff_base + c] = *r;
            aflat_stage(&mut p, &mut v, *r);
        }
        coeff_base += seg.n_coeff;
    }
    (codes[0], codes[1], codes[2] as u8, rq)
}

#[cfg(test)]
mod tests {
    use super::super::decode::dequant_reflection;
    use super::*;

    /// The VQ search is self-consistent: quantizing the exact
    /// dequantized value of a codebook row recovers reflection
    /// coefficients close to it (the search minimizes the AFLAT
    /// residual, so the chosen vector's residual can only be
    /// lower-or-equal than the seed row's).
    #[test]
    fn vq_search_self_consistency() {
        let seed = dequant_reflection(700, 300, 150);
        let rr = rc_to_autocorr(&seed);
        let (l1, l2, l3, rq) = vq_search(&rr);
        let re = dequant_reflection(l1, l2, l3);
        assert_eq!(rq, re, "returned set must equal the coded rows");
        for (a, b) in seed.iter().zip(rq.iter()) {
            assert!(
                (a - b).abs() < 0.2,
                "quantized rc far from seed: {a} vs {b}"
            );
        }
    }

    /// `rc_to_autocorr` inverts `step_up`-style modeling: running
    /// the AFLAT lattice over the exact source coefficients yields
    /// the residual Π(1-r²) (the minimum), and any perturbed set a
    /// larger one.
    #[test]
    fn aflat_residual_minimum_at_source() {
        let r = [0.6, -0.4, 0.3, -0.2, 0.15, -0.1, 0.08, -0.05, 0.03, -0.02];
        let rr = rc_to_autocorr(&r);
        let mut p = [0f64; NP + 1];
        p.copy_from_slice(&rr);
        let mut v = [0f64; 2 * NP - 1];
        for i in (1 - (NP as isize))..(NP as isize) {
            v[(i + NP as isize - 1) as usize] = rr[(i + 1).unsigned_abs()];
        }
        let (mut p2, mut v2) = (p, v);
        for rj in r {
            aflat_stage(&mut p2, &mut v2, rj);
        }
        let e_min: f64 = r.iter().map(|x| 1.0 - x * x).product();
        assert!((p2.first().unwrap() - e_min).abs() < 1e-9);

        let mut worse = r;
        worse[0] += 0.1;
        let (mut p3, mut v3) = (p, v);
        for rj in worse {
            aflat_stage(&mut p3, &mut v3, rj);
        }
        assert!(p3[0] > p2[0]);
    }

    /// R0 coding: a full-scale-ish frame codes high, silence codes 0.
    #[test]
    fn r0_extremes() {
        let mut an = HrAnalyzer::new();
        let silent = an.analyze_frame(&[0i16; HR_FRAME_SAMPLES]);
        assert_eq!(silent.r0, 0);
        let mut an = HrAnalyzer::new();
        let mut loud = [0i16; HR_FRAME_SAMPLES];
        for (i, s) in loud.iter_mut().enumerate() {
            // Strong in-band tone (1 kHz at 8 kHz), well above the
            // 120 Hz high-pass corner.
            *s = ((i as f64 * std::f64::consts::TAU / 8.0).sin() * 16000.0) as i16 & !7;
        }
        let _ = an.analyze_frame(&loud);
        let out = an.analyze_frame(&loud);
        assert!(out.r0 >= 25, "loud tone must code high, got {}", out.r0);
    }
}
