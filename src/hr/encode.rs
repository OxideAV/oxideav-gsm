//! GSM 06.20 half-rate encoder — clause 4.1 of ETSI EN 300 969
//! (staged): the frame-parameter analysis chain of clauses
//! 4.1.1–4.1.6 ([`HrAnalyzer`]: input high-pass filtering,
//! segmentation, the FLAT covariance-lattice reflection-coefficient
//! derivation, the three-segment AFLAT vector quantization,
//! frame-energy coding and the soft-interpolation decision) and,
//! on top of it, the complete per-subframe excitation analysis
//! ([`HrEncoder`]): the spectral noise weighting filter (4.1.7),
//! the open-loop lag search / trajectory / voicing mode
//! ([`super::lag`], 4.1.8.1–4.1.8.4), the closed-loop lag search
//! (4.1.8.5), harmonic noise weighting (4.1.9), the VSELP code
//! searches (4.1.10, [`super::search`]) and the multimode `{P0,GS}`
//! gain quantization (4.1.11), plus the clause 5.3 encoder homing.
//!
//! ## Segmentation (clause 4.1.2)
//!
//! The analysis buffer holds 195 high-pass filtered samples; *"the
//! oldest 160 samples in the buffer correspond to the next frame of
//! samples to be encoded"* while *"the analysis interval comprises
//! the most recent 170 samples"*. The encoder therefore codes each
//! input frame with a 35-sample look-ahead: parameter frame `f`
//! carries input samples `160·f − 35 .. 160·f + 125`, and a decoder
//! reproduces the input delayed by 35 samples — exactly the offset
//! the staged GSM 06.07 references show between `SEQxx.INP` and
//! `SEQxx.OUT`.
//!
//! ## Conformance posture
//!
//! Like the decoder (see `hr::decode`), the chain implements the
//! printed floating-point equations over the staged ROM tables in
//! double precision; the bit-exact arithmetic lives in the unstaged
//! GSM 06.06 ANSI-C, so the achievable validation bar is measured
//! per-parameter agreement against the staged GSM 06.07 encoder
//! references (`tests/conformance_hr_encode_params.rs`) plus the
//! sample-exact self round trip through [`super::HrDecoder`] on the
//! homing protocol.
//!
//! ## Readings pinned from the printed clause
//!
//! * clause 4.1.1 prints the high-pass coefficients halved: taken
//!   literally, eq. (3)/(4) with the printed values is not a
//!   120 Hz high-pass at all (a resonator with −38 dB at 500 Hz);
//!   doubling every coefficient (the words are Q14) gives the
//!   stated fourth-order 120 Hz high-pass with the *"incorporated
//!   gain of 0,5"* — a −6 dB passband — exactly. The coded signal
//!   itself is carried at unity passband gain (the 0,5 undone) with
//!   the clause 4.1.5 `Rmax = 4096²` ([`super::R0_RMAX`]): the R0
//!   codes agree with the corpus either way, but the eq. (132)
//!   energy estimate behind the `{P0,GS}` search and the decoder's
//!   excitation level pin this scaling (see `HighPass`);
//! * the clause 4.1.6 residual comparison and every subframe
//!   operation run over the coded frame `s(0..160)` of the buffer,
//!   with the ten samples that precede it kept for the inverse
//!   filters.

use super::decode::{
    adaptive_codebook, decode_r0, dequant_reflection, frac_delay_6, hr_decoder_homing_frame,
    step_down, step_up, vq_index, HR_ENCODER_HOMING_SAMPLE,
};
use super::lag::{open_loop_search, OpenLoop, Y_HIST};
use super::search::{code_search, decorrelate, gain_search, GainInputs};
use super::tables::*;
use super::{
    HrParameters, SubframeParams, HR_FRAME_SAMPLES, HR_SUBFRAMES, HR_SUBFRAME_SAMPLES, R0_RMAX,
};

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
/// two cascaded biquads with an incorporated gain of 0,5. The ROM
/// words ([`HIGHPASS_COEFFS`], storage order per section
/// `(b0, b1, b2, a2, a1)`) equal the printed eq. (3)/(4) values in
/// Q15 — and both are **halved**: used as printed the cascade is a
/// resonator (−38 dB at 500 Hz, −12 dB at 1 kHz), while doubling
/// every coefficient (reading the words as Q14) yields a real
/// fourth-order 120 Hz high-pass whose passband sits at exactly the
/// stated −6 dB (double real poles at 0,926 and 0,965).
#[derive(Debug, Clone, Default)]
struct HighPass {
    x1: [f64; 2],
    y1: [f64; 2],
    y2: [f64; 2],
    x2in: [f64; 2],
}

impl HighPass {
    fn process(&mut self, x: f64) -> f64 {
        // Q14: every word doubled relative to its Q15 reading.
        let c = |i: usize| HIGHPASS_COEFFS[i] as f64 / 16384.0;
        // Section 1 (eq. (3)).
        let y1 = c(0) * x
            + c(1) * self.x1[0]
            + c(2) * self.x1[1]
            + c(4) * self.y1[0]
            + c(3) * self.y1[1];
        self.x1 = [x, self.x1[0]];
        self.y1 = [y1, self.y1[0]];
        // Section 2 (eq. (4)).
        let y2 = c(5) * y1
            + c(6) * self.x2in[0]
            + c(7) * self.x2in[1]
            + c(9) * self.y2[0]
            + c(8) * self.y2[1];
        self.x2in = [y1, self.x2in[0]];
        self.y2 = [y2, self.y2[0]];
        // The coded-signal domain is the filtered input at unity
        // passband gain: the printed 0,5 gain is undone here. Pinned
        // by the staged references — the R0 codes agree either way
        // (R0 is relative to Rmax), but the eq. (132) energy
        // estimate feeding the {P0,GS} search and the decoder's
        // excitation level only match the corpus with the coded
        // signal at the input level (GSP0 agreement with every other
        // parameter forced: 7% at −6 dB versus 77% here; decoder
        // output level ≈ the reference's, which sits at the input
        // level).
        2.0 * y2
    }
}

/// Where a subframe's direct-form coefficient set comes from
/// (clause 4.1.6): the previous frame's set, the current frame's,
/// or the table-2 interpolation of the two. The spectral noise
/// weighting filter mirrors the same choice (clause 4.1.7).
#[doc(hidden)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CoefSource {
    Previous,
    Current,
    Interpolated,
}

/// Everything the frame analysis derives that the excitation
/// analysis consumes.
#[doc(hidden)]
#[derive(Debug, Clone)]
pub struct FrameSets {
    pub codes: FrameAnalysis,
    /// The current frame's quantized reflection coefficients and
    /// direct-form set.
    pub refl_q: [f64; NP],
    pub alpha_cur: [f64; NP],
    /// The per-subframe direct-form sets and their provenance.
    pub sub_alpha: [[f64; NP]; HR_SUBFRAMES],
    #[doc(hidden)]
    pub sub_src: [CoefSource; HR_SUBFRAMES],
}

/// The clause 4.1.1–4.1.6 frame analyzer.
#[derive(Debug, Clone)]
pub struct HrAnalyzer {
    hp: HighPass,
    /// Clause 4.1.2 sample buffer `s(0..=194)`, `s(0)` oldest; the
    /// coded frame is `s(0..160)`.
    buf: [f64; BUF],
    /// The ten samples preceding `s(0)` (inverse-filter memory for
    /// the clause 4.1.6 comparison and the weighting stage).
    pre: [f64; NP],
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
            pre: [0.0; NP],
            prev_alpha: [0.0; NP],
        }
    }

    /// Reset to the initial state.
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Analyze one 160-sample input frame (13-bit left-justified
    /// i16, the GSM 06.07 input convention) into the frame-level
    /// parameter codes of the frame the buffer now codes (clause
    /// 4.1.2: the input's last 35 samples are look-ahead).
    pub fn analyze_frame(&mut self, samples: &[i16; HR_FRAME_SAMPLES]) -> FrameAnalysis {
        self.analyze_frame_sets(samples).codes
    }

    /// The high-pass filtered coded frame `s(0..160)` in the
    /// normalised sample domain (13-bit full scale = 1,0), with the
    /// ten preceding samples.
    pub(super) fn coded_frame(&self) -> (&[f64; NP], &[f64]) {
        (&self.pre, &self.buf[..HR_FRAME_SAMPLES])
    }

    /// Sample `s(m)` of the coded frame for `m ≥ -10`.
    #[inline]
    fn sample(&self, m: isize) -> f64 {
        if m >= 0 {
            self.buf[m as usize]
        } else {
            self.pre[(NP as isize + m) as usize]
        }
    }

    pub(super) fn analyze_frame_sets(&mut self, samples: &[i16; HR_FRAME_SAMPLES]) -> FrameSets {
        self.analyze_frame_sets_forced(samples, None)
    }

    /// Diagnostics: analyze, then substitute the given frame codes
    /// (and everything derived from them, including the
    /// soft-interpolation choice) for the ones found.
    // The covariance/window/interpolation loops index multiple
    // arrays by the same symmetric (i, k) pair; iterator forms
    // obscure the spec equations.
    #[allow(clippy::needless_range_loop)]
    #[doc(hidden)]
    pub fn analyze_frame_sets_forced(
        &mut self,
        samples: &[i16; HR_FRAME_SAMPLES],
        forced: Option<FrameAnalysis>,
    ) -> FrameSets {
        // Clauses 4.1.1/4.1.2: high-pass + buffer shift. The frame
        // being shifted out (s(0..160)) leaves its last ten samples
        // as the inverse-filter memory of the next coded frame.
        self.pre
            .copy_from_slice(&self.buf[HR_FRAME_SAMPLES - NP..HR_FRAME_SAMPLES]);
        self.buf.copy_within(HR_FRAME_SAMPLES.., 0);
        for (i, &s) in samples.iter().enumerate() {
            // 13-bit left-justified in 16 bits: s/32768 = s13/4096.
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

        // Clause 4.1.5 eqs. (27)-(29): frame-energy code, relative
        // to Rmax = the square of the filtered signal's maximum
        // amplitude (the −6 dB high-pass halves the 13-bit full
        // scale). Pinned by the staged references: every R0 code
        // lands within one 2 dB step of the corpus.
        let r0_energy = (phi[0][0] + phi[NP][NP]) / 320.0 / R0_RMAX;
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
        let (mut lpc1, mut lpc2, mut lpc3, mut rq) = vq_search(&rr);
        let mut r0 = r0;
        if let Some(f) = forced {
            r0 = f.r0;
            lpc1 = f.lpc1;
            lpc2 = f.lpc2;
            lpc3 = f.lpc3;
            rq = dequant_reflection(lpc1, lpc2, lpc3);
        }

        // Clause 4.1.6: the soft-interpolation decision. Build the
        // interpolated and uninterpolated per-subframe sets from the
        // previous and current quantized coefficients, inverse
        // filter the coded frame s(0..160), and pick the lower
        // residual energy (ties go to uninterpolated).
        let alpha_cur = step_up(&rq);
        let mut int_sets = [[0f64; NP]; HR_SUBFRAMES];
        let mut int_src = [CoefSource::Current; HR_SUBFRAMES];
        let mut flat_sets = [[0f64; NP]; HR_SUBFRAMES];
        let mut flat_src = [CoefSource::Current; HR_SUBFRAMES];
        let mut e_interp = 0.0;
        let mut e_flat = 0.0;
        for sf in 0..HR_SUBFRAMES {
            let (a_int, src) = if sf == HR_SUBFRAMES - 1 {
                (alpha_cur, CoefSource::Current)
            } else {
                let del = SOFT_INTERP_CURRENT[sf] as f64 / 32768.0;
                let mut a = [0f64; NP];
                for i in 0..NP {
                    a[i] = self.prev_alpha[i] + del * (alpha_cur[i] - self.prev_alpha[i]);
                }
                if step_down(&a).is_some() {
                    (a, CoefSource::Interpolated)
                } else if sf == 0 {
                    (self.prev_alpha, CoefSource::Previous)
                } else {
                    (alpha_cur, CoefSource::Current)
                }
            };
            let (a_unint, usrc) = if sf == 0 {
                (self.prev_alpha, CoefSource::Previous)
            } else {
                (alpha_cur, CoefSource::Current)
            };
            int_sets[sf] = a_int;
            int_src[sf] = src;
            flat_sets[sf] = a_unint;
            flat_src[sf] = usrc;
            for n in 0..HR_SUBFRAME_SAMPLES {
                let idx = (sf * HR_SUBFRAME_SAMPLES + n) as isize;
                let mut p_int = 0.0;
                let mut p_flat = 0.0;
                for i in 0..NP {
                    let past = self.sample(idx - 1 - i as isize);
                    p_int += a_int[i] * past;
                    p_flat += a_unint[i] * past;
                }
                let s = self.buf[idx as usize];
                e_interp += (s - p_int) * (s - p_int);
                e_flat += (s - p_flat) * (s - p_flat);
            }
        }
        let mut int_lpc = e_interp < e_flat;
        if let Some(f) = forced {
            int_lpc = f.int_lpc;
        }
        let (sub_alpha, sub_src) = if int_lpc {
            (int_sets, int_src)
        } else {
            (flat_sets, flat_src)
        };

        self.prev_alpha = alpha_cur;
        FrameSets {
            codes: FrameAnalysis {
                r0,
                lpc1,
                lpc2,
                lpc3,
                int_lpc,
            },
            refl_q: rq,
            alpha_cur,
            sub_alpha,
            sub_src,
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

// ─── Clause 4.1.7–4.1.11: the excitation analysis ───

/// Samples per subframe (annex A.2 `Ns`).
const NS: usize = HR_SUBFRAME_SAMPLES;

/// Long-term filter history length (clause 4.1.8.5: the deepest
/// interpolator tap for `Lmax` reaches 147 samples back).
const HIST: usize = 147;

/// Clause 4.1.7: the spectral noise weighting coefficients `α̃_i`
/// for one direct-form set `α`: the zero-state response `h3(n)` of
/// the cascade 1/A(z) · A(z/0,93) · 1/A(z/0,7) over `Ns` samples
/// (eqs. (32)–(35)), its autocorrelation (eq. (36)) and the AFLAT
/// recursion (eqs. (37)–(41)) back to reflection coefficients,
/// converted to direct form (step 7). The 0,93ⁱ / 0,7ⁱ
/// bandwidth-expansion weights are the staged ROM words
/// ([`SNW_COEFFS`], Q15).
fn weighting_coefficients(alpha: &[f64; NP]) -> [f64; NP] {
    let w93 = |i: usize| SNW_COEFFS[i] as f64 / 32768.0;
    let w70 = |i: usize| SNW_COEFFS[NP + i] as f64 / 32768.0;
    let mut h1 = [0f64; NS];
    let mut h2 = [0f64; NS];
    let mut h3 = [0f64; NS];
    for n in 0..NS {
        let mut acc = if n == 0 { 1.0 } else { 0.0 };
        for i in 0..NP.min(n) {
            acc += alpha[i] * h1[n - 1 - i];
        }
        h1[n] = acc;
        let mut acc2 = h1[n];
        for i in 0..NP.min(n) {
            acc2 -= w93(i) * alpha[i] * h1[n - 1 - i];
        }
        h2[n] = acc2;
        let mut acc3 = h2[n];
        for i in 0..NP.min(n) {
            acc3 += w70(i) * alpha[i] * h3[n - 1 - i];
        }
        h3[n] = acc3;
    }
    let mut rh = [0f64; NP + 1];
    for (i, r) in rh.iter_mut().enumerate() {
        *r = (i..NS).map(|n| h3[n] * h3[n - i]).sum();
    }
    if rh[0] <= 0.0 {
        return [0.0; NP];
    }
    // AFLAT over the autocorrelation (eqs. (37)-(41)).
    let mut p = rh;
    let mut v = [0f64; 2 * NP - 1];
    for i in (1 - (NP as isize))..(NP as isize) {
        v[(i + NP as isize - 1) as usize] = rh[(i + 1).unsigned_abs()];
    }
    let mut r = [0f64; NP];
    for rj in r.iter_mut() {
        let p0 = p[0];
        let v0 = v[NP - 1];
        *rj = if p0.abs() > 1e-300 {
            (-v0 / p0).clamp(-0.999_999, 0.999_999)
        } else {
            0.0
        };
        aflat_stage(&mut p, &mut v, *rj);
    }
    step_up(&r)
}

/// Zero-state response of the all-pole filter `1/(1 − Σ a_i z⁻ⁱ)`
/// to `x` over one subframe.
fn zsr_allpole(a: &[f64; NP], x: &[f64; NS]) -> [f64; NS] {
    let mut y = [0f64; NS];
    for n in 0..NS {
        let mut acc = x[n];
        for i in 0..NP.min(n) {
            acc += a[i] * y[n - 1 - i];
        }
        y[n] = acc;
    }
    y
}

/// Zero-input response of the same filter from its memory
/// (`mem[i]` = output at `n − 1 − i`).
fn zir_allpole(a: &[f64; NP], mem: &[f64; NP]) -> [f64; NS] {
    let mut y = [0f64; NS];
    for n in 0..NS {
        let mut acc = 0.0;
        for i in 0..NP {
            let past = if n > i { y[n - 1 - i] } else { mem[i - n] };
            acc += a[i] * past;
        }
        y[n] = acc;
    }
    y
}

/// Clause 4.1.9 eqs. (106)/(107): apply `C(z) = 1 − λ z^{−L_pitch}`
/// to `cur` (history `hist` for `n < 0`; zero history for a
/// zero-state response).
fn harmonic_weight(hist: &[f64], cur: &[f64; NS], lambda: f64, lpitch: i32) -> [f64; NS] {
    if lambda == 0.0 {
        return *cur;
    }
    let mut out = [0f64; NS];
    for n in 0..NS {
        out[n] = cur[n] - lambda * frac_delay_6(hist, cur, n as isize, lpitch);
    }
    out
}

/// Clause 4.1.10 eq. (108): the (unweighted) VSELP codevector for
/// a codeword, in the codebook's raw ROM units.
fn codevector(basis: &[[i16; 40]], codeword: u16) -> [f64; NS] {
    let mut u = [0f64; NS];
    for (m, v) in basis.iter().enumerate() {
        let sign = if (codeword >> m) & 1 == 1 { 1.0 } else { -1.0 };
        for (n, &s) in v.iter().enumerate() {
            u[n] += sign * s as f64;
        }
    }
    u
}

/// Shift a history buffer left by one subframe and append `new`.
fn shift_append(hist: &mut [f64; HIST], new: &[f64; NS]) {
    hist.copy_within(NS.., 0);
    hist[HIST - NS..].copy_from_slice(new);
}

/// Diagnostics: reference values to substitute at each stage of
/// the encoder (teacher forcing against a conformance corpus).
#[doc(hidden)]
#[derive(Debug, Clone, Copy, Default)]
pub struct HrForce {
    pub frame: Option<FrameAnalysis>,
    /// (mode, absolute lag levels per subframe).
    pub lags: Option<(u8, [usize; HR_SUBFRAMES])>,
    /// Voiced 9-bit codes, or unvoiced (code1, code2) pairs.
    pub codes: Option<[u16; HR_SUBFRAMES]>,
    pub codes2: Option<[u16; HR_SUBFRAMES]>,
    pub gsp0: Option<[u8; HR_SUBFRAMES]>,
}

/// GSM 06.20 half-rate speech encoder (clause 4.1): frame analysis
/// through [`HrAnalyzer`] followed by the per-subframe excitation
/// analysis, producing one annex-A parameter frame per 160-sample
/// input frame (clause 5.3 encoder homing applied).
#[derive(Debug, Clone)]
pub struct HrEncoder {
    an: HrAnalyzer,
    /// Previous frame's spectral-noise-weighting set `α̃` (clause
    /// 4.1.7 interpolation mirror).
    prev_wa: [f64; NP],
    /// Previous frame's decoded energy and quantized reflection set
    /// (eqs. (131a)/(132)).
    prev_r0q: f64,
    prev_refl: [f64; NP],
    /// Weighting filter `W(z)` all-pole memory (past `y`) and the
    /// weighted-speech history in front of the frame.
    w_mem: [f64; NP],
    y_hist: [f64; Y_HIST],
    /// `H(z)` memory (past weighted synthetic excitation) and its
    /// history for the harmonic-weighting zero-input response.
    h_mem: [f64; NP],
    h_hist: [f64; HIST],
    /// Long-term filter state `r(n)` (clause 4.2.5 mirror).
    ltp_hist: [f64; HIST],
}

impl Default for HrEncoder {
    fn default() -> Self {
        Self::new()
    }
}

impl HrEncoder {
    /// Fresh encoder in the clause 5.5 home state.
    pub fn new() -> Self {
        Self {
            an: HrAnalyzer::new(),
            prev_wa: [0.0; NP],
            prev_r0q: 0.0,
            prev_refl: [0.0; NP],
            w_mem: [0.0; NP],
            y_hist: [0.0; Y_HIST],
            h_mem: [0.0; NP],
            h_hist: [0.0; HIST],
            ltp_hist: [0.0; HIST],
        }
    }

    /// Reset to the home state (clause 5.3 step 2).
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Encode one 160-sample input frame (13-bit left-justified
    /// i16) into the annex-A parameter frame. An encoder homing
    /// frame (every sample `0008` hex, clause 5.2) yields the
    /// decoder homing frame and resets the encoder (clause 5.3).
    pub fn encode_frame(&mut self, samples: &[i16; HR_FRAME_SAMPLES]) -> HrParameters {
        if samples.iter().all(|&s| s == HR_ENCODER_HOMING_SAMPLE) {
            self.reset();
            return hr_decoder_homing_frame();
        }
        self.encode_frame_no_homing(samples)
    }

    /// Encode one frame without the homing check.
    pub fn encode_frame_no_homing(&mut self, samples: &[i16; HR_FRAME_SAMPLES]) -> HrParameters {
        self.encode_frame_forced(samples, &HrForce::default())
    }

    /// Diagnostics: encode with reference values substituted per
    /// [`HrForce`].
    // The filter loops index coefficient and sample arrays by the
    // same lag offsets; iterator forms obscure the spec equations.
    #[allow(clippy::needless_range_loop)]
    #[doc(hidden)]
    pub fn encode_frame_forced(
        &mut self,
        samples: &[i16; HR_FRAME_SAMPLES],
        force: &HrForce,
    ) -> HrParameters {
        let sets = self.an.analyze_frame_sets_forced(samples, force.frame);
        let codes = sets.codes;

        // Clause 4.1.7: weighting coefficients once per frame, then
        // per subframe mirroring the clause 4.1.6 choice.
        let wa_cur = weighting_coefficients(&sets.alpha_cur);
        let mut sub_wa = [[0f64; NP]; HR_SUBFRAMES];
        for sf in 0..HR_SUBFRAMES {
            sub_wa[sf] = match sets.sub_src[sf] {
                CoefSource::Previous => self.prev_wa,
                CoefSource::Current => wa_cur,
                CoefSource::Interpolated => {
                    let del = SOFT_INTERP_CURRENT[sf] as f64 / 32768.0;
                    let mut a = [0f64; NP];
                    for i in 0..NP {
                        a[i] = self.prev_wa[i] + del * (wa_cur[i] - self.prev_wa[i]);
                    }
                    a
                }
            };
        }

        // Weighted speech y(n) = W(z) s(n) = H(z)[A(z) s(n)] over the
        // coded frame, with continuous filter memories.
        let (pre, frame) = self.an.coded_frame();
        let sample = |m: isize| -> f64 {
            if m >= 0 {
                frame[m as usize]
            } else {
                pre[(NP as isize + m) as usize]
            }
        };
        let mut y_all = [0f64; Y_HIST + HR_FRAME_SAMPLES];
        y_all[..Y_HIST].copy_from_slice(&self.y_hist);
        for sf in 0..HR_SUBFRAMES {
            let a = &sets.sub_alpha[sf];
            let wa = &sub_wa[sf];
            for n in 0..NS {
                let idx = (sf * NS + n) as isize;
                let mut res = sample(idx);
                for i in 0..NP {
                    res -= a[i] * sample(idx - 1 - i as isize);
                }
                let mut y = res;
                for i in 0..NP {
                    y += wa[i] * self.w_mem[i];
                }
                self.w_mem.copy_within(..NP - 1, 1);
                self.w_mem[0] = y;
                y_all[Y_HIST + sf * NS + n] = y;
            }
        }

        // Clauses 4.1.8.1-4.1.8.4.
        let mut ol: OpenLoop = open_loop_search(&y_all);
        let forced_lags = force.lags.is_some();
        if let Some((m, lv)) = force.lags {
            ol.mode = m;
            ol.levels = lv;
        }
        let mode = ol.mode;

        // Eq. (132) energy estimate inputs.
        let r0q_cur = decode_r0(codes.r0);

        let mut lag_levels = [0usize; HR_SUBFRAMES];
        let mut code1 = [0u8; HR_SUBFRAMES];
        let mut code2 = [0u8; HR_SUBFRAMES];
        let mut vcode = [0u16; HR_SUBFRAMES];
        let mut gsp0 = [0u8; HR_SUBFRAMES];

        for sf in 0..HR_SUBFRAMES {
            let wa = &sub_wa[sf];
            let mut y_sf = [0f64; NS];
            y_sf.copy_from_slice(&y_all[Y_HIST + sf * NS..Y_HIST + (sf + 1) * NS]);
            let y_before = &y_all[..Y_HIST + sf * NS];
            let zir = zir_allpole(wa, &self.h_mem);

            let (r0q_eff, refl_eff) = if sf == 0 {
                (self.prev_r0q, &self.prev_refl)
            } else {
                (r0q_cur, &sets.refl_q)
            };
            let mut rs = NS as f64 * r0q_eff;
            for r in refl_eff.iter() {
                rs *= 1.0 - r * r;
            }

            let (c0, c1, w0, w1, target);
            if mode == 0 {
                // Target p(n) = W(z)s − ZIR of H(z).
                let mut p = [0f64; NS];
                for n in 0..NS {
                    p[n] = y_sf[n] - zir[n];
                }
                // First VSELP codebook.
                let mut q1: Vec<[f64; NS]> = BASIS_VECTORS_MODE0[0]
                    .iter()
                    .map(|v| {
                        let mut x = [0f64; NS];
                        for (n, &s) in v.iter().enumerate() {
                            x[n] = s as f64;
                        }
                        zsr_allpole(wa, &x)
                    })
                    .collect();
                let mut i_code = code_search(&q1, &p);
                if let Some(c) = force.codes {
                    i_code = c[sf];
                }
                let f_i = {
                    let mut f = [0f64; NS];
                    for (m, qm) in q1.iter().enumerate() {
                        let sg = if (i_code >> m) & 1 == 1 { 1.0 } else { -1.0 };
                        for n in 0..NS {
                            f[n] += sg * qm[n];
                        }
                    }
                    f
                };
                // Second codebook, decorrelated against f_I.
                let mut q2: Vec<[f64; NS]> = BASIS_VECTORS_MODE0[1]
                    .iter()
                    .map(|v| {
                        let mut x = [0f64; NS];
                        for (n, &s) in v.iter().enumerate() {
                            x[n] = s as f64;
                        }
                        zsr_allpole(wa, &x)
                    })
                    .collect();
                let q2_raw = q2.clone();
                decorrelate(&mut q2, &f_i);
                let mut h_code = code_search(&q2, &p);
                if let Some(c) = force.codes2 {
                    h_code = c[sf];
                }
                let f_h = {
                    let mut f = [0f64; NS];
                    for (m, qm) in q2_raw.iter().enumerate() {
                        let sg = if (h_code >> m) & 1 == 1 { 1.0 } else { -1.0 };
                        for n in 0..NS {
                            f[n] += sg * qm[n];
                        }
                    }
                    f
                };
                q1.clear();
                code1[sf] = i_code as u8;
                code2[sf] = h_code as u8;
                c0 = codevector(&BASIS_VECTORS_MODE0[0], i_code);
                c1 = codevector(&BASIS_VECTORS_MODE0[1], h_code);
                w0 = f_i;
                w1 = f_h;
                target = p;
            } else {
                // Clause 4.1.8.5: closed-loop lag search over the
                // three levels around the trajectory lag (two at the
                // table ends), restricted to what the annex A.1.4
                // delta code can carry.
                let mut p = [0f64; NS];
                for n in 0..NS {
                    p[n] = y_sf[n] - zir[n];
                }
                let centre = ol.levels[sf] as i32;
                let mut best: Option<(f64, usize, [f64; NS], [f64; NS])> = None;
                for cand in centre - 1..=centre + 1 {
                    if !(0..LAG_TABLE.len() as i32).contains(&cand) {
                        continue;
                    }
                    if forced_lags && cand != centre {
                        continue;
                    }
                    if sf > 0 && !forced_lags {
                        let d = cand - lag_levels[sf - 1] as i32;
                        if !(-8..=7).contains(&d) {
                            continue;
                        }
                    }
                    let b_l = adaptive_codebook(&self.ltp_hist, LAG_TABLE[cand as usize] as i32);
                    let b_w = zsr_allpole(wa, &b_l);
                    let c: f64 = b_w.iter().zip(p.iter()).map(|(a, b)| a * b).sum();
                    let g: f64 = b_w.iter().map(|v| v * v).sum();
                    let score = if g > 0.0 {
                        c / g.sqrt()
                    } else {
                        f64::NEG_INFINITY
                    };
                    if best.as_ref().map_or(true, |(b, ..)| score > *b) {
                        best = Some((score, cand as usize, b_l, b_w));
                    }
                }
                let (_, level, b_l, b_w) =
                    best.expect("at least the trajectory lag is a candidate");
                lag_levels[sf] = level;

                // Clause 4.1.9: harmonic noise weighting on the
                // target, the ZIR and every filtered vector.
                let lambda = ol.lambda[sf];
                let lpitch = ol.lpitch[sf];
                let yc = harmonic_weight(y_before, &y_sf, lambda, lpitch);
                let zir_c = harmonic_weight(&self.h_hist, &zir, lambda, lpitch);
                let mut pc = [0f64; NS];
                for n in 0..NS {
                    pc[n] = yc[n] - zir_c[n];
                }
                let zero_hist = [0f64; NS];
                let b_wc = harmonic_weight(&zero_hist, &b_w, lambda, lpitch);
                let q_raw: Vec<[f64; NS]> = BASIS_VECTORS_MODE123
                    .iter()
                    .map(|v| {
                        let mut x = [0f64; NS];
                        for (n, &s) in v.iter().enumerate() {
                            x[n] = s as f64;
                        }
                        harmonic_weight(&zero_hist, &zsr_allpole(wa, &x), lambda, lpitch)
                    })
                    .collect();
                let mut q = q_raw.clone();
                decorrelate(&mut q, &b_wc);
                let mut code = code_search(&q, &pc);
                if let Some(c) = force.codes {
                    code = c[sf];
                }
                let f_i = {
                    let mut f = [0f64; NS];
                    for (m, qm) in q_raw.iter().enumerate() {
                        let sg = if (code >> m) & 1 == 1 { 1.0 } else { -1.0 };
                        for n in 0..NS {
                            f[n] += sg * qm[n];
                        }
                    }
                    f
                };
                vcode[sf] = code;
                c0 = b_l;
                c1 = codevector(&BASIS_VECTORS_MODE123, code);
                w0 = b_wc;
                w1 = f_i;
                target = pc;
            }

            // Clause 4.1.11: joint gain quantization.
            let mut gc = gain_search(
                mode,
                &GainInputs {
                    p: &target,
                    c0: &c0,
                    c1: &c1,
                    w0: &w0,
                    w1: &w1,
                    rs,
                },
            );
            if let Some(g) = force.gsp0 {
                gc = super::search::gain_from_code(mode, g[sf], &c0, &c1, rs);
            }
            gsp0[sf] = gc.code;

            // Reconstruct the excitation exactly as the decoder does
            // (eq. (127)) and advance the states: long-term filter
            // (clause 4.2.5) and the weighted synthetic excitation
            // through H(z) with its memory.
            let mut ex = [0f64; NS];
            for n in 0..NS {
                ex[n] = gc.beta * c0[n] + gc.gamma * c1[n];
            }
            shift_append(&mut self.ltp_hist, &ex);
            let hx = zsr_allpole(wa, &ex);
            let mut h_out = [0f64; NS];
            for n in 0..NS {
                h_out[n] = zir[n] + hx[n];
            }
            for i in 0..NP {
                self.h_mem[i] = h_out[NS - 1 - i];
            }
            shift_append(&mut self.h_hist, &h_out);
        }

        // Frame-level state.
        self.y_hist
            .copy_from_slice(&y_all[HR_FRAME_SAMPLES..HR_FRAME_SAMPLES + Y_HIST]);
        self.prev_wa = wa_cur;
        self.prev_r0q = r0q_cur;
        self.prev_refl = sets.refl_q;

        let sub = if mode == 0 {
            SubframeParams::Unvoiced { code1, code2, gsp0 }
        } else {
            let mut lag_delta = [0u8; 3];
            for sf in 1..HR_SUBFRAMES {
                lag_delta[sf - 1] = (lag_levels[sf] as i32 - lag_levels[sf - 1] as i32 + 8) as u8;
            }
            SubframeParams::Voiced {
                lag1: lag_levels[0] as u8,
                lag_delta,
                code: vcode,
                gsp0,
            }
        };
        HrParameters {
            r0: codes.r0,
            lpc1: codes.lpc1,
            lpc2: codes.lpc2,
            lpc3: codes.lpc3,
            int_lpc: codes.int_lpc,
            mode_code: mode,
            sub,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::super::decode::dequant_reflection;
    use super::*;

    /// Clause 4.1.7: the AFLAT-fitted all-pole H(z) reproduces the
    /// impulse response of the three-filter cascade closely over
    /// the subframe.
    #[test]
    fn weighting_filter_fits_cascade_impulse_response() {
        let r = dequant_reflection(700, 300, 150);
        let alpha = step_up(&r);
        let wa = weighting_coefficients(&alpha);
        // Cascade impulse response h3 (eqs. (33)-(35)).
        let w93 = |i: usize| SNW_COEFFS[i] as f64 / 32768.0;
        let w70 = |i: usize| SNW_COEFFS[NP + i] as f64 / 32768.0;
        let (mut h1, mut h2, mut h3) = ([0f64; NS], [0f64; NS], [0f64; NS]);
        for n in 0..NS {
            let mut a = if n == 0 { 1.0 } else { 0.0 };
            for i in 0..NP.min(n) {
                a += alpha[i] * h1[n - 1 - i];
            }
            h1[n] = a;
            let mut b = h1[n];
            for i in 0..NP.min(n) {
                b -= w93(i) * alpha[i] * h1[n - 1 - i];
            }
            h2[n] = b;
            let mut c = h2[n];
            for i in 0..NP.min(n) {
                c += w70(i) * alpha[i] * h3[n - 1 - i];
            }
            h3[n] = c;
        }
        let mut imp = [0f64; NS];
        imp[0] = 1.0;
        let hf = zsr_allpole(&wa, &imp);
        let e_h: f64 = h3.iter().map(|v| v * v).sum();
        let e_d: f64 = h3
            .iter()
            .zip(hf.iter())
            .map(|(a, b)| (a - b) * (a - b))
            .sum();
        eprintln!(
            "h3 vs fitted: rel err {:.4}; h3[..6]={:?} fit[..6]={:?}",
            e_d / e_h,
            &h3[..6],
            &hf[..6]
        );
        assert!(e_d / e_h < 0.05, "fit rel err {}", e_d / e_h);
    }

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
