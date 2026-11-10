//! GSM 06.20 half-rate encoder — clause 4.1 of ETSI EN 300 969
//! (staged): the frame-parameter analysis chain of clauses
//! 4.1.1–4.1.6 ([`HrAnalyzer`]: input high-pass filtering,
//! segmentation, the FLAT covariance-lattice reflection-coefficient
//! derivation, the three-segment AFLAT vector quantization,
//! frame-energy coding and the soft-interpolation decision) and,
//! on top of it, the complete per-subframe excitation analysis
//! ([`HrEncoder`]): the spectral noise weighting filter (4.1.7),
//! the open-loop lag search / trajectory / voicing mode
//! (`super::olfx`, 4.1.8.1–4.1.8.4), the closed-loop lag search
//! (4.1.8.5), harmonic noise weighting (4.1.9), the VSELP code
//! searches (4.1.10) and the multimode `{P0,GS}` gain quantization
//! (4.1.11) (`super::subfx`), plus the clause 5.3 encoder homing.
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
//! The front end — clause 4.1.1 high-pass, 4.1.2 buffer, 4.1.3
//! FLAT, 4.1.5 R0 and the 4.1.4 AFLAT VQ search — runs in the
//! clause 5.1 fixed-point arithmetic (`super::front`, the staged
//! `halfrate-traces/hr-fixed-point-schedules.md` Part B) and
//! reproduces the R0 / LPC words of every `SEQ01..03.COD` frame.
//! The open-loop lag search, trajectory and voicing mode
//! (`super::olfx`, §C.1 rules 1–20), the closed-loop lag search,
//! C(z), H(z) responses, decorrelation, the VSELP code searches, the
//! `{P0,GS}` search and the encoder's local excitation
//! (`super::subfx`, §C.1 rules 21–24 and §C.2) are fixed-point too.
//! Two stages remain floating point because their fixed-point
//! schedules are not staged: the clause 4.1.6 residual-energy
//! comparison behind INT_LPC (on the exact fixed-point coefficient
//! words and stability checks) and the clause 4.1.7 spectral noise
//! weighting coefficients / weighted speech `y(n)` (rounded to
//! 16-bit words at the high-pass output scale before the searches).
//! The measured per-parameter agreement is pinned by
//! `tests/conformance_hr_encode_params.rs`.

//! ## Readings pinned from the printed clause
//!
//! * clause 4.1.1: each biquad's sum is doubled (the effective
//!   recursion coefficients are twice the printed words), section 1
//!   halves its input and section 2 doubles its output — an overall
//!   0,5 passband gain (erratum G1). The floating-point stages see
//!   the coded signal at unity gain (`y/16384` in the normalised
//!   13-bit domain), where clause 4.1.5's `Rmax = 4096²` holds;
//! * the clause 4.1.6 residual comparison and every subframe
//!   operation run over the coded frame `s(0..160)` of the buffer,
//!   with the ten samples that precede it kept for the inverse
//!   filters.

use super::decode::{
    HR_ENCODER_HOMING_SAMPLE, adaptive_codebook_words, codevector_words, dequant_reflection_words,
    hr_decoder_homing_frame, interp_words, rs_estimate, scale_vector, shift_append_words,
    step_down_words, step_up, step_up_words,
};
use super::front::FrontEnd;
use super::olfx::{OL_HIST, OpenLoopState, open_loop_fx};

/// Weighted-speech history kept in front of a frame (the open-loop
/// search's deepest correlation tap, n = −145).
const Y_HIST: usize = OL_HIST;
use super::fx::{add, shr, sub};
use super::subfx::{
    GspInputs, HIST as SUB_HIST, closed_loop, code_search_fx, combine, cz, decorrelate_fx,
    gsp0_search, hz_filter, hz_zir, hz_zsr, rho_plain, target_shift,
};
use super::tables::*;
use super::{HR_FRAME_SAMPLES, HR_SUBFRAME_SAMPLES, HR_SUBFRAMES, HrParameters, SubframeParams};

/// Short-term predictor order (annex A.2 `Np`).
const NP: usize = 10;

/// Analysis buffer length (clause 4.1.2: *"the previous 195 input
/// high pass filtered speech samples"*).
const BUF: usize = 195;

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
    /// The same reflection coefficients as Q15 words.
    pub refl_w: [i16; NP],
    pub alpha_cur: [f64; NP],
    /// The per-subframe direct-form sets and their provenance.
    pub sub_alpha: [[f64; NP]; HR_SUBFRAMES],
    #[doc(hidden)]
    pub sub_src: [CoefSource; HR_SUBFRAMES],
}

/// The clause 4.1.1–4.1.6 frame analyzer.
#[derive(Debug, Clone)]
pub struct HrAnalyzer {
    /// Clauses 4.1.1-4.1.5 in the fixed-point arithmetic.
    front: FrontEnd,
    /// Clause 4.1.2 sample buffer `s(0..=194)`, `s(0)` oldest; the
    /// coded frame is `s(0..160)`.
    buf: [f64; BUF],
    /// The ten samples preceding `s(0)` (inverse-filter memory for
    /// the clause 4.1.6 comparison and the weighting stage).
    pre: [f64; NP],
    /// Previous frame's quantized direct-form set + INT_LPC state
    /// (clause 4.1.6).
    prev_alpha: [f64; NP],
    prev_a_w: [i16; NP],
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
            front: FrontEnd::new(),
            buf: [0.0; BUF],
            pre: [0.0; NP],
            prev_alpha: [0.0; NP],
            prev_a_w: [0; NP],
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
        // Clauses 4.1.1-4.1.5 in the fixed-point arithmetic
        // (`super::front`): high-pass, buffer shift, FLAT, R0 and the
        // AFLAT VQ search. The frame being shifted out (s(0..160))
        // leaves its last ten samples as the inverse-filter memory of
        // the next coded frame.
        self.pre
            .copy_from_slice(&self.buf[HR_FRAME_SAMPLES - NP..HR_FRAME_SAMPLES]);
        self.front.push(samples);
        // The f64 stages run on the coded signal at unity passband
        // gain in the normalised domain (13-bit full scale = 1,0):
        // the 16-bit high-pass words carry the 0,5 gain, so
        // y/16384.
        for (d, &w) in self.buf.iter_mut().zip(self.front.buf.iter()) {
            *d = w as f64 / 16384.0;
        }
        let fr = self.front.analyse();
        let (mut r0, mut lpc1, mut lpc2, mut lpc3) = (fr.r0, fr.lpc.0, fr.lpc.1, fr.lpc.2);
        let mut rq_w = fr.rq;
        if let Some(f) = forced {
            r0 = f.r0;
            lpc1 = f.lpc1;
            lpc2 = f.lpc2;
            lpc3 = f.lpc3;
            rq_w = dequant_reflection_words(lpc1, lpc2, lpc3);
        }
        let mut rq = [0f64; NP];
        for (d, &w) in rq.iter_mut().zip(rq_w.iter()) {
            *d = w as f64 / 32768.0;
        }

        // Clause 4.1.6: the soft-interpolation decision. Build the
        // interpolated and uninterpolated per-subframe sets from the
        // previous and current quantized coefficients, inverse
        // filter the coded frame s(0..160), and pick the lower
        // residual energy (ties go to uninterpolated).
        // The coefficient sets are the Q11 direct-form words of the
        // fixed-point step-up (with the saturation fallback), and the
        // interpolated sets the `mac_r` interpolation of those words
        // with the fixed-point stability check (schedules §A.1/§A.3).
        let (mut a_cur_w, flagged) = step_up_words(&rq_w);
        if flagged {
            a_cur_w = self.prev_a_w;
        }
        let to_alpha = |w: &[i16; NP]| w.map(|v| -(v as f64) / 2048.0);
        let alpha_cur = to_alpha(&a_cur_w);
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
                let aw = interp_words(&a_cur_w, &self.prev_a_w, sf);
                if !step_down_words(&aw).1 {
                    (to_alpha(&aw), CoefSource::Interpolated)
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
        self.prev_a_w = a_cur_w;
        FrameSets {
            codes: FrameAnalysis {
                r0,
                lpc1,
                lpc2,
                lpc3,
                int_lpc,
            },
            refl_q: rq,
            refl_w: rq_w,
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
#[cfg(test)]
pub(super) fn rc_to_autocorr(r: &[f64; NP]) -> [f64; NP + 1] {
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
pub(super) fn aflat_stage(p: &mut [f64; NP + 1], v: &mut [f64; 2 * NP - 1], rj: f64) {
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

// ─── Clause 4.1.7–4.1.11: the excitation analysis ───

/// Samples per subframe (annex A.2 `Ns`).
const NS: usize = HR_SUBFRAME_SAMPLES;

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
#[cfg(test)]
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
    /// Previous frame's quantized reflection words, direct-form words
    /// and R0 amplitude word (eqs. (131a)/(132)).
    prev_refl_w: [i16; NP],
    prev_a_w: [i16; NP],
    prev_amp: i16,
    /// Weighting filter `W(z)` all-pole memory (past `y`) and the
    /// weighted-speech history in front of the frame.
    w_mem: [f64; NP],
    y_hist: [f64; Y_HIST],
    /// `H(z)` memory (past weighted synthetic excitation, words) and
    /// its history for the harmonic-weighting zero-input response.
    h_mem_w: [i16; NP],
    h_hist_w: [i16; SUB_HIST],
    /// Long-term filter state `r(n)` (encoder-side local excitation).
    ltp_w: [i16; SUB_HIST],
    /// Open-loop search state (previous frame's subframe energies)
    /// and the previous frame's R0 code.
    ol_state: OpenLoopState,
    prev_r0: u8,
    /// Diagnostics: the last frame's open-loop trajectory / Lpitch
    /// (1/6 units) and closed-loop lags.
    last_lags: HrLagDiag,
}

/// Diagnostics of the last encoded frame's lag decisions (1/6-sample
/// units; 0 when unvoiced).
#[doc(hidden)]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct HrLagDiag {
    pub mode: u8,
    pub traj6: [i32; HR_SUBFRAMES],
    pub pitch6: [i32; HR_SUBFRAMES],
    pub cl6: [i32; HR_SUBFRAMES],
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
            ol_state: OpenLoopState::default(),
            prev_r0: 0,
            last_lags: HrLagDiag::default(),
            prev_refl_w: [0; NP],
            prev_a_w: [0; NP],
            prev_amp: 0,
            w_mem: [0.0; NP],
            y_hist: [0.0; Y_HIST],
            h_mem_w: [0; NP],
            h_hist_w: [0; SUB_HIST],
            ltp_w: [0; SUB_HIST],
        }
    }

    /// Reset to the home state (clause 5.3 step 2).
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Diagnostics: the last frame's lag decisions.
    #[doc(hidden)]
    pub fn last_lags(&self) -> HrLagDiag {
        self.last_lags
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

        if std::env::var("HRDBG").is_ok() {
            let mut e = [0f64; 4];
            for sf in 0..4 {
                for n in 0..NS {
                    let w = y_all[Y_HIST + sf * NS + n] * 16384.0;
                    e[sf] += w * w;
                }
            }
            eprintln!("YDBG sumsq(words@16384) {:?}", e);
        }
        // Clauses 4.1.8.1-4.1.8.4.
        let mut yw = [0i16; OL_HIST + HR_FRAME_SAMPLES];
        for (d, &v) in yw.iter_mut().zip(y_all.iter()) {
            *d = (v * 16384.0).round().clamp(-32768.0, 32767.0) as i16;
        }
        let olf = open_loop_fx(&yw, &mut self.ol_state, self.prev_r0, codes.r0);
        let (mut mode, mut ol_levels) = (olf.mode, olf.levels);
        let forced_lags = force.lags.is_some();
        if let Some((m, lv)) = force.lags {
            mode = m;
            ol_levels = lv;
        }

        // sqrt(RS) per subframe (eqs. (131)/(132), the decoder's
        // schedule §A.3 on the quantised words).
        let refl_w = sets.refl_w;
        let amp = R0_DECODE_TABLE[2 * (codes.r0 as usize & 31)];
        let (mut a_cur_w, flagged) = step_up_words(&refl_w);
        let mut refl_eff = refl_w;
        if flagged {
            a_cur_w = self.prev_a_w;
            refl_eff = self.prev_refl_w;
        }
        let rs_cur = rs_estimate(&refl_eff, amp);
        let rs_prev = rs_estimate(&self.prev_refl_w, self.prev_amp);
        let mut sub_rs = [rs_cur; HR_SUBFRAMES];
        for k in 0..HR_SUBFRAMES - 1 {
            sub_rs[k] = if codes.int_lpc {
                let ai = interp_words(&a_cur_w, &self.prev_a_w, k);
                let (rk, unstable) = step_down_words(&ai);
                if !unstable {
                    rs_estimate(&rk, if k == 0 { self.prev_amp } else { amp })
                } else if k == 0 {
                    rs_prev
                } else {
                    rs_cur
                }
            } else if k == 0 {
                rs_prev
            } else {
                rs_cur
            };
        }

        let mut lag_levels = [0usize; HR_SUBFRAMES];
        let mut code1 = [0u8; HR_SUBFRAMES];
        let mut code2 = [0u8; HR_SUBFRAMES];
        let mut vcode = [0u16; HR_SUBFRAMES];
        let mut gsp0 = [0u8; HR_SUBFRAMES];

        for sf in 0..HR_SUBFRAMES {
            // Q11 H(z) words of the subframe's weighting set.
            let aw: [i16; NP] =
                sub_wa[sf].map(|v| (-v * 2048.0).round().clamp(-32768.0, 32767.0) as i16);
            let ys = OL_HIST + sf * NS;
            let mut y_sf = [0i16; NS];
            y_sf.copy_from_slice(&yw[ys..ys + NS]);
            let zir = hz_zir(&aw, &self.h_mem_w);
            let mut p = [0i16; NS];
            for n in 0..NS {
                p[n] = sub(y_sf[n], zir[n]);
            }
            let s = target_shift(&p);
            let ps = p.map(|v| shr(v, s));

            let (c0, c1, w0, w1, target);
            if mode == 0 {
                let q1: Vec<[i16; NS]> = BASIS_VECTORS_MODE0[0]
                    .iter()
                    .map(|v| hz_zsr(&aw, v))
                    .collect();
                let mut i_code = code_search_fx(&q1, &ps);
                if let Some(c) = force.codes {
                    i_code = c[sf] & 0x7f;
                }
                let b = combine(&q1, i_code);
                let mut q2: Vec<[i16; NS]> = BASIS_VECTORS_MODE0[1]
                    .iter()
                    .map(|v| hz_zsr(&aw, v))
                    .collect();
                decorrelate_fx(&mut q2, &b);
                let mut h_code = code_search_fx(&q2, &ps);
                if let Some(c) = force.codes2 {
                    h_code = c[sf] & 0x7f;
                }
                code1[sf] = i_code as u8;
                code2[sf] = h_code as u8;
                c0 = codevector_words(&BASIS_VECTORS_MODE0[0], i_code);
                c1 = codevector_words(&BASIS_VECTORS_MODE0[1], h_code);
                w0 = hz_zsr(&aw, &c0);
                w1 = hz_zsr(&aw, &c1);
                target = ps;
            } else {
                // Clause 4.1.8.5 (rules 21-24).
                let cl = closed_loop(&self.ltp_w, &aw, ol_levels[sf], &ps, forced_lags);
                lag_levels[sf] = cl.level;
                // Clause 4.1.9: C(z) on the target, the ZIR and the
                // filtered vectors.
                let (lp6, hnw) = (olf.lpitch6[sf], olf.hnw[sf]);
                let yc = cz(&y_sf, Some(&yw[..ys]), lp6, hnw);
                let zc = cz(&zir, Some(&self.h_hist_w[..]), lp6, hnw);
                let mut pc = [0i16; NS];
                for n in 0..NS {
                    pc[n] = shr(sub(yc[n], zc[n]), s);
                }
                let b_full = adaptive_codebook_words(&self.ltp_w, LAG_TABLE[cl.level]);
                let bw = cz(&hz_zsr(&aw, &b_full), None, lp6, hnw).map(|v| shr(v, cl.t));
                let mut q: Vec<[i16; NS]> = BASIS_VECTORS_MODE123
                    .iter()
                    .map(|v| cz(&hz_zsr(&aw, v), None, lp6, hnw))
                    .collect();
                decorrelate_fx(&mut q, &bw);
                let mut code = code_search_fx(&q, &pc);
                if let Some(c) = force.codes {
                    code = c[sf] & 0x1ff;
                }
                vcode[sf] = code;
                c0 = cl.b;
                c1 = codevector_words(&BASIS_VECTORS_MODE123, code);
                w0 = bw;
                w1 = cz(&hz_zsr(&aw, &c1), None, lp6, hnw);
                target = pc;
            }

            // Clause 4.1.11.1.
            let mut g = gsp0_search(
                mode,
                &GspInputs {
                    p: &target,
                    c0: &c0,
                    c1: &c1,
                    w0: &w0,
                    w1: &w1,
                    s,
                    sqrt_rs: sub_rs[sf],
                },
            );
            if let Some(gf) = force.gsp0 {
                g = gf[sf] & 31;
            }
            gsp0[sf] = g;

            // Local excitation (encoder-side rule: plain-energy ρ on
            // the shifted LTP vector) and state updates.
            let row = &GSP0_VQ[mode as usize][g as usize];
            let (y0, _) = scale_vector(&c0, row[0], rho_plain(&c0, sub_rs[sf]));
            let (y1, _) = scale_vector(&c1, row[1], rho_plain(&c1, sub_rs[sf]));
            let mut ex = [0i16; NS];
            for n in 0..NS {
                ex[n] = add(y0[n], y1[n]);
            }
            shift_append_words(&mut self.ltp_w, &ex);
            let h_out = hz_filter(&aw, &ex, &mut self.h_mem_w);
            shift_append_words(&mut self.h_hist_w, &h_out);
        }

        // Frame-level state.
        self.y_hist
            .copy_from_slice(&y_all[HR_FRAME_SAMPLES..HR_FRAME_SAMPLES + Y_HIST]);
        self.prev_wa = wa_cur;
        self.prev_r0 = codes.r0;
        self.prev_refl_w = refl_eff;
        self.prev_a_w = a_cur_w;
        self.prev_amp = amp;
        self.last_lags = if mode == 0 {
            HrLagDiag::default()
        } else {
            HrLagDiag {
                mode,
                traj6: olf.levels.map(|l| LAG_TABLE[l] as i32),
                pitch6: olf.lpitch6,
                cl6: lag_levels.map(|l| LAG_TABLE[l] as i32),
            }
        };

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
