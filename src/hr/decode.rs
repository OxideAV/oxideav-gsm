//! GSM 06.20 half-rate speech decoder — clause 4.2 of ETSI
//! EN 300 969 (staged at `docs/audio/gsm/`), driven by the annex-A
//! parameter frame ([`super::HrParameters`]) and the codec's ROM
//! data ([`super::tables`]).
//!
//! The pipeline is the spec's figure 5: parameter decode (the
//! clause 4.1.4/4.1.5/4.1.6 dequantisation legs) → combined
//! excitation generation (clauses 4.1.8.5/4.1.10/4.1.11.1 + 4.2.1)
//! → adaptive pitch prefilter (4.2.2) → synthesis filter (4.2.3) →
//! adaptive spectral postfilter with AGC (4.2.4) → long-term state
//! update (4.2.5), plus the clause 5 decoder-homing protocol.
//!
//! ## Conformance posture
//!
//! EN 300 969 is a *functional* description: its equations carry
//! floating-point constants (0,3 / 0,75 / 0,9875 …) and clause 5.1
//! places the bit-exact arithmetic in the GSM 06.06 ANSI-C
//! deliverable — which is deliberately **not** staged (only its ROM
//! **data** is, per the clean-room posture of
//! `docs/audio/gsm/halfrate-tables/`). This decoder therefore
//! implements the printed equations in double precision over the
//! staged ROM tables. Because the long-term predictor feeds the
//! decoder's own past excitation back into the excitation
//! (clause 4.2.5), any sub-LSB arithmetic difference compounds over
//! a sequence, so byte equality with the GSM 06.07 `*.OUT`
//! references is not an achievable bar for a non-bit-exact
//! implementation; `tests/conformance_hr_decode.rs` instead pins
//! the clause 5 homing behaviour exactly and the measured
//! per-frame waveform agreement as a regression floor.
//!
//! ## Empirically resolved readings
//!
//! The printed description leaves several codings ambiguous
//! (byte/bit orders inside the ROM extracts, the postfilter
//! numerator source). Each was resolved against the staged GSM
//! 06.07 corpus (decoded-vs-reference per-frame correlation over
//! SEQ01–SEQ04; details in the harness):
//!
//! * packed reflection-VQ words hold the earlier scalar-quantiser
//!   index in the **high** byte;
//! * clause 4.1.10 codeword bit `m` (θ_im, basis vector `v_m`) is
//!   the codeword's bit `m − 1` counted from the **LSB**;
//! * annex A.1.4 delta-lag codes are excess-8 (`code − 8` levels);
//! * the clause 4.2.4 numerator is the SST-smoothed version of the
//!   **unweighted** coefficient set `α_i` (measured marginally
//!   above the alternative literal reading, the SST of the
//!   0,75-weighted denominator set — the two differ only through
//!   the near-unity SST window);
//! * eq. (131a) is applied literally: subframe 1 draws `R'q(0)`
//!   (and the eq. (132) reflection product) from the previous
//!   frame.

use super::tables::*;
use super::{HrParameters, SubframeParams, HR_FRAME_SAMPLES, HR_SUBFRAMES, HR_SUBFRAME_SAMPLES};

/// Short-term predictor order (annex A.2 `Np`).
const NP: usize = 10;

/// Samples per subframe (annex A.2 `Ns`).
const NS: usize = HR_SUBFRAME_SAMPLES;

/// Long-term filter history length: the deepest sample the
/// fractional-lag interpolator can address is `n - I - 5` with
/// `I = Lmax = 142` (clause 4.1.8.5 eq. (100), `Pf = 10`), i.e.
/// `-147`.
const HIST: usize = 147;

/// The clause 5.2 encoder-homing-frame sample value (`0008` hex,
/// 13-bit left-justified in a 16-bit word) — the decoder's output
/// for the second and further consecutive decoder homing frames.
pub const HR_ENCODER_HOMING_SAMPLE: i16 = 0x0008;

/// The decoder homing frame's 18 annex-A parameter words, as fixed
/// by the staged GSM 06.07 `SEQ05.DEC` sequence (clause 5.2: *"The
/// decoder homing frame has a fixed set of speech parameters as
/// defined in test sequence SEQ05"*).
pub const HR_DECODER_HOMING_WORDS: [u16; 18] = [
    0, 881, 350, 195, 1, 0, 71, 74, 0, 9, 38, 7, 0, 0, 0, 0, 0, 0,
];

/// `true` when the parameter frame is the clause 5.2 decoder homing
/// frame.
pub fn is_hr_decoder_homing_frame(p: &HrParameters) -> bool {
    p.to_cod_words() == HR_DECODER_HOMING_WORDS
}

/// The decoder homing frame as an [`HrParameters`] value.
pub fn hr_decoder_homing_frame() -> HrParameters {
    HrParameters::from_cod_words(&HR_DECODER_HOMING_WORDS)
}

// ─── Parameter dequantisation ───

/// Clause 4.1.4: unpack one 8-bit scalar-dequantiser index from the
/// packed reflection-coefficient VQ tables (two indices per 16-bit
/// word, high byte first, C storage order).
#[inline]
fn vq_index(table: &[i16], flat: usize) -> usize {
    let w = table[flat / 2] as u16;
    if flat % 2 == 0 {
        (w >> 8) as usize
    } else {
        (w & 0xFF) as usize
    }
}

/// Clause 4.1.4: decode the three LPC vector-quantiser codes into
/// the ten reflection coefficients, via the 256-entry scalar
/// dequantiser (Q15 → f64).
fn dequant_reflection(lpc1: u16, lpc2: u16, lpc3: u8) -> [f64; NP] {
    let mut r = [0f64; NP];
    let scalar = |idx: usize| RC_SCALAR_DEQUANT[idx] as f64 / 32768.0;
    for (c, slot) in r[..3].iter_mut().enumerate() {
        *slot = scalar(vq_index(&RC_VQ_SEG1, lpc1 as usize * 3 + c));
    }
    for (c, slot) in r[3..6].iter_mut().enumerate() {
        *slot = scalar(vq_index(&RC_VQ_SEG2, lpc2 as usize * 3 + c));
    }
    for (c, slot) in r[6..].iter_mut().enumerate() {
        *slot = scalar(vq_index(&RC_VQ_SEG3, lpc3 as usize * 4 + c));
    }
    r
}

/// Reflection coefficients → direct-form LPC coefficients `α_i` for
/// the clause 4.2.3 synthesis form `s(n) = ex_ps(n) + Σ α_i s(n-i)`
/// (eq. (154) adds the prediction, so `α` is the plus convention).
/// The step-up recursion inverts the FLAT/AFLAT lattice whose stage
/// equations (10)/(162) define `r_j` with the leading minus sign;
/// the sign pairing here is the one validated by the >14 dB
/// prediction gain the dequantised sets achieve on the staged
/// GSM 06.07 encoder-input speech.
fn step_up(r: &[f64; NP]) -> [f64; NP] {
    let mut a = [0f64; NP];
    for j in 0..NP {
        let mut next = a;
        next[j] = -r[j];
        for i in 0..j {
            next[i] = a[i] - r[j] * a[j - 1 - i];
        }
        a = next;
    }
    // `a` holds A(z) = 1 + Σ a_i z^-i; the synthesis form needs
    // α_i = -a_i.
    let mut alpha = [0f64; NP];
    for i in 0..NP {
        alpha[i] = -a[i];
    }
    alpha
}

/// Direct-form → reflection coefficients (step-down), used for the
/// clause 4.1.6 stability check of interpolated coefficient sets.
/// Returns `None` when any |r_j| ≥ 1 (unstable filter).
fn step_down(alpha: &[f64; NP]) -> Option<[f64; NP]> {
    let mut a = [0f64; NP];
    for i in 0..NP {
        a[i] = -alpha[i];
    }
    let mut r = [0f64; NP];
    for j in (0..NP).rev() {
        let k = a[j];
        // NaN (a degenerate interpolated set) is treated as
        // unstable, like any |r| >= 1.
        if !k.is_finite() || k.abs() >= 0.999_999 {
            return None;
        }
        r[j] = -k;
        let denom = 1.0 - k * k;
        let prev = a;
        for i in 0..j {
            a[i] = (prev[i] - k * prev[j - 1 - i]) / denom;
        }
    }
    Some(r)
}

/// Clause 4.1.5 eq. (30): decode the 5-bit R0 code into the average
/// signal power `R(0)_q` relative to full scale (`Rmax = 1` in the
/// normalised sample domain used internally). Code 0 is the −66 dB
/// floor; the home state treats it as silence.
#[inline]
fn decode_r0(r0: u8) -> f64 {
    if r0 == 0 {
        return 0.0;
    }
    10f64.powf((2.0 * r0 as f64 - 66.0) / 10.0)
}

// ─── Fractional-lag interpolation (clause 4.1.8.5 eq. (100)) ───

/// Evaluate the sequence (history `hist` for n < 0, `cur` for
/// n ≥ 0) delayed by `lag_sixths` 1/6-sample units at position `n`:
/// the single-tap integer case, or the 10th-order 6-phase
/// interpolating FIR `f̃_j(i)` ([`INTERP_FILTER_10`], Q15) centred
/// per its phase-`j` tap centroid (`n - I - 5 + i`, `L = I + j/6`).
fn interp_delayed(hist: &[f64; HIST], cur: &[f64], n: isize, lag_sixths: i32) -> f64 {
    let i_part = (lag_sixths / 6) as isize;
    let phase = (lag_sixths % 6) as usize;
    let fetch = |m: isize| -> f64 {
        if m < 0 {
            let idx = m + HIST as isize;
            if idx < 0 {
                0.0
            } else {
                hist[idx as usize]
            }
        } else {
            cur[m as usize]
        }
    };
    if phase == 0 {
        // Clause 4.1.8.5: the 0th phase has a single non-zero tap.
        return fetch(n - i_part);
    }
    let mut acc = 0.0;
    for (i, row) in INTERP_FILTER_10.iter().enumerate() {
        acc += row[phase] as f64 / 32768.0 * fetch(n - i_part - 5 + i as isize);
    }
    acc
}

// ─── Codevector construction (clause 4.1.10 eq. (108)) ───

/// Build the VSELP codevector for `codeword` over `basis`: the sum
/// of the basis vectors, each signed by the corresponding codeword
/// bit (θ_im = +1 when set; bit m of the codeword counted from the
/// LSB pairs with basis vector v_m).
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

// ─── SST numerator for the adaptive spectral postfilter (4.2.4) ───

/// Derive the spectrally-smoothed numerator coefficients from the
/// (uninterpolated) direct-form set: take the autocorrelation of
/// the coefficient sequence of `1 − Σ α_i z^-i`, apply the SST
/// window ([`FLAT_SST_COEFFS`], Q31), and run the clause 4.2.4
/// AFLAT recursion (eqs. (159)–(164)) back to reflection
/// coefficients, then step up.
fn sst_numerator(alpha: &[f64; NP]) -> [f64; NP] {
    // Coefficient sequence a_0..a_10 of the polynomial.
    let mut c = [0f64; NP + 1];
    c[0] = 1.0;
    for i in 0..NP {
        c[i + 1] = -alpha[i];
    }
    // Autocorrelation of the coefficient sequence.
    let mut rr = [0f64; NP + 1];
    for lag in 0..=NP {
        let mut acc = 0.0;
        for k in 0..=(NP - lag) {
            acc += c[k] * c[k + lag];
        }
        rr[lag] = acc;
    }
    // SST bandwidth expansion (eq. (158)); W(0) = 1.
    for lag in 1..=NP {
        rr[lag] *= FLAT_SST_COEFFS[lag - 1] as f64 / 2147483648.0;
    }
    // AFLAT recursion (eqs. (159)-(164)): P over 0..=Np,
    // V over 1-Np..=Np-1 (stored offset by Np-1).
    let mut p = rr;
    let mut v = [0f64; 2 * NP - 1];
    for i in (1 - (NP as isize))..(NP as isize) {
        v[(i + NP as isize - 1) as usize] = rr[i.unsigned_abs()];
    }
    let vat = |v: &[f64; 2 * NP - 1], i: isize| v[(i + NP as isize - 1) as usize];
    let mut refl = [0f64; NP];
    for (j, slot) in refl.iter_mut().enumerate() {
        let rj = if p[0].abs() > 1e-30 {
            (-vat(&v, 0) / p[0]).clamp(-0.999_99, 0.999_99)
        } else {
            0.0
        };
        *slot = rj;
        if j == NP - 1 {
            break;
        }
        let bound = NP - j - 1;
        let mut np2 = [0f64; NP + 1];
        for (i, slot) in np2.iter_mut().take(bound).enumerate() {
            *slot = (1.0 + rj * rj) * p[i] + rj * (vat(&v, i as isize) + vat(&v, -(i as isize)));
        }
        let mut nv = [0f64; 2 * NP - 1];
        for i in (1 - bound as isize)..(bound as isize) {
            let val = vat(&v, i + 1)
                + rj * rj * vat(&v, -i - 1)
                + 2.0 * rj * p[(i + 1).unsigned_abs().min(NP)];
            nv[(i + NP as isize - 1) as usize] = val;
        }
        p = np2;
        v = nv;
    }
    step_up(&refl)
}

// ─── Decoder state ───

/// GSM 06.20 half-rate speech decoder state (clause 4.2).
#[derive(Debug, Clone)]
pub struct HrDecoder {
    /// Long-term filter state `r(n)`, `n = -147..-1` (clause 4.2.5).
    ltp_hist: [f64; HIST],
    /// Pitch-prefilter output history (the clause 4.2.2 recursion
    /// references its own past output at the fractional lag).
    pre_hist: [f64; HIST],
    /// Synthesis filter memory `s(n-1)..s(n-10)` (clause 4.2.3).
    synth: [f64; NP],
    /// Postfilter numerator input memory (`s`), denominator output
    /// memory (`s''`), and brightness memory (clause 4.2.4).
    pf_in: [f64; NP],
    pf_out: [f64; NP],
    pf_bright: f64,
    /// AGC smoothed scale `S'scale` (eq. (168)).
    agc: f64,
    /// Previous frame's direct-form coefficients / SST numerator /
    /// decoded frame energy / reflection coefficients (clauses
    /// 4.1.6 and 4.1.11.1 eqs. (131a)/(132)).
    prev_alpha: [f64; NP],
    prev_num: [f64; NP],
    prev_r0q: f64,
    prev_refl: [f64; NP],
    /// Clause 5.4 homing: `true` while the decoder sits in its home
    /// state (fresh, reset, or right after a decoder homing frame).
    home: bool,
}

impl Default for HrDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl HrDecoder {
    /// Fresh decoder in the clause 5.6 home state.
    pub fn new() -> Self {
        Self {
            ltp_hist: [0.0; HIST],
            pre_hist: [0.0; HIST],
            synth: [0.0; NP],
            pf_in: [0.0; NP],
            pf_out: [0.0; NP],
            pf_bright: 0.0,
            agc: 1.0,
            prev_alpha: [0.0; NP],
            prev_num: [0.0; NP],
            prev_r0q: 0.0,
            prev_refl: [0.0; NP],
            home: true,
        }
    }

    /// Reset to the home state (clause 5.4 step 2).
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Whether the decoder is in its clause 5.6 home state.
    pub fn is_home(&self) -> bool {
        self.home
    }

    /// Decode one parameter frame into 160 PCM samples (13-bit
    /// left-justified in i16, the GSM 06.07 output convention),
    /// applying the clause 5.4 decoder-homing protocol: a decoder
    /// homing frame decodes normally (with the output substituted
    /// by the encoder homing frame when the decoder was already in
    /// its home state) and then resets the decoder.
    pub fn decode_frame(&mut self, p: &HrParameters) -> [i16; HR_FRAME_SAMPLES] {
        let was_home = self.home;
        let mut out = self.decode_frame_no_homing(p);
        if is_hr_decoder_homing_frame(p) {
            if was_home {
                out = [HR_ENCODER_HOMING_SAMPLE; HR_FRAME_SAMPLES];
            }
            self.reset();
        }
        out
    }

    /// Decode one parameter frame without the homing protocol.
    pub fn decode_frame_no_homing(&mut self, p: &HrParameters) -> [i16; HR_FRAME_SAMPLES] {
        self.home = false;
        let mode = p.mode_code & 3;

        // ── Frame-level parameter decode ──
        let refl = dequant_reflection(p.lpc1, p.lpc2, p.lpc3);
        let alpha_cur = step_up(&refl);
        let r0q_cur = decode_r0(p.r0);
        let num_cur = sst_numerator(&alpha_cur);

        // Clause 4.1.6: per-subframe coefficient sets (soft
        // interpolation with the stability fallback), mirrored onto
        // the postfilter numerator per clause 4.2.4's "same
        // interpolation scheme … as … the LPC synthesis
        // coefficients".
        let mut sub_alpha = [[0f64; NP]; HR_SUBFRAMES];
        let mut sub_num = [[0f64; NP]; HR_SUBFRAMES];
        for j in 0..HR_SUBFRAMES {
            let (a, num) = if j == HR_SUBFRAMES - 1 {
                (alpha_cur, num_cur)
            } else if p.int_lpc {
                let del = SOFT_INTERP_CURRENT[j] as f64 / 32768.0;
                let mut a = [0f64; NP];
                let mut num = [0f64; NP];
                for i in 0..NP {
                    a[i] = self.prev_alpha[i] + del * (alpha_cur[i] - self.prev_alpha[i]);
                    num[i] = self.prev_num[i] + del * (num_cur[i] - self.prev_num[i]);
                }
                if step_down(&a).is_some() {
                    (a, num)
                } else if j == 0 {
                    // Unstable interpolated set: subframe 1 falls
                    // back to the previous frame's coefficients,
                    // later subframes to the current frame's.
                    (self.prev_alpha, self.prev_num)
                } else {
                    (alpha_cur, num_cur)
                }
            } else if j == 0 {
                // INT_LPC = 0 (Del(j,0) = {0,1,1,1}): subframe 1
                // uses the previous frame's coefficients.
                (self.prev_alpha, self.prev_num)
            } else {
                (alpha_cur, num_cur)
            };
            sub_alpha[j] = a;
            sub_num[j] = num;
        }

        // ── Subframe loop ──
        let mut out = [0i16; HR_FRAME_SAMPLES];
        let mut lag_idx: usize = 0;
        for sf in 0..HR_SUBFRAMES {
            // Excitation vectors c0/c1 per clause 4.1.11.1.
            let (c0, c1, gsp0, lag_sixths): ([f64; NS], [f64; NS], u8, Option<i32>) = match &p.sub {
                SubframeParams::Unvoiced { code1, code2, gsp0 } => {
                    let u1 = codevector(&BASIS_VECTORS_MODE0[0], code1[sf] as u16);
                    let u2 = codevector(&BASIS_VECTORS_MODE0[1], code2[sf] as u16);
                    (u1, u2, gsp0[sf], None)
                }
                SubframeParams::Voiced {
                    lag1,
                    lag_delta,
                    code,
                    gsp0,
                } => {
                    if sf == 0 {
                        lag_idx = *lag1 as usize;
                    } else {
                        // Annex A.1.4: excess-8 delta of allowable
                        // lag levels; deltas that would point
                        // outside the 256-entry table clamp (the
                        // SEQ04 channel-error leg drives this).
                        let d = lag_delta[sf - 1] as i32 - 8;
                        lag_idx = (lag_idx as i32 + d).clamp(0, 255) as usize;
                    }
                    let lag = LAG_TABLE[lag_idx] as i32;
                    let b_l = self.adaptive_codevector(lag);
                    let u = codevector(&BASIS_VECTORS_MODE123, code[sf]);
                    (b_l, u, gsp0[sf], Some(lag))
                }
            };

            // Gains (clause 4.1.11.1): RS = Ns · R'q(0) · Π(1-r_i²)
            // (eq. (132)), with subframe 1 drawing the energy and
            // reflection product from the previous frame
            // (eq. (131a)), then eqs. (145)/(146):
            //   βq = √(RS·GS·P0     / Rx(0)),
            //   γq = √(RS·GS·(1-P0) / Rx(1)),
            // where √(GS·P0) and √(GS·(1-P0)) are the first two
            // GSP0 codebook components (Q13 — see `tables`).
            let (r0q_eff, refl_eff) = if sf == 0 {
                (self.prev_r0q, &self.prev_refl)
            } else {
                (r0q_cur, &refl)
            };
            let mut rs = NS as f64 * r0q_eff;
            for r in refl_eff.iter() {
                rs *= 1.0 - r * r;
            }
            let rx0: f64 = c0.iter().map(|x| x * x).sum();
            let rx1: f64 = c1.iter().map(|x| x * x).sum();
            let gs = &GSP0_VQ[mode as usize][gsp0 as usize];
            let sq_gs_p0 = gs[0] as f64 / 8192.0;
            let sq_gs_1mp0 = gs[1] as f64 / 8192.0;
            // Eqs. (147)-(149): an all-zero long-term state (rx0 =
            // 0 for a voiced subframe) disables the first vector.
            let beta = if rx0 > 1e-30 {
                (rs / rx0).sqrt() * sq_gs_p0
            } else {
                0.0
            };
            let gamma = if rx1 > 1e-30 {
                (rs / rx1).sqrt() * sq_gs_1mp0
            } else {
                0.0
            };

            // Combined excitation (eq. (127)).
            let mut ex = [0f64; NS];
            for n in 0..NS {
                ex[n] = beta * c0[n] + gamma * c1[n];
            }

            // Adaptive pitch prefilter (clause 4.2.2, MODE ≠ 0):
            // ex_p(n) = ex(n) + ζ·ex_p(n-L) with
            // ζ = 0,3·min(β,1)·√P0 (eq. (151), √P0 from the staged
            // per-mode lookup), then the eq. (152) energy
            // renormalisation.
            let mut exp = [0f64; NS];
            if let Some(lag) = lag_sixths {
                let zeta = 0.3
                    * beta.min(1.0)
                    * (SQRT_P0[(mode - 1) as usize][gsp0 as usize] as f64 / 32768.0);
                for n in 0..NS {
                    let delayed = interp_delayed(&self.pre_hist, &exp, n as isize, lag);
                    exp[n] = ex[n] + zeta * delayed;
                }
            } else {
                exp = ex;
            }
            let e_in: f64 = ex.iter().map(|x| x * x).sum();
            let e_out: f64 = exp.iter().map(|x| x * x).sum();
            let pscale = if e_out > 1e-30 {
                (e_in / e_out).sqrt()
            } else {
                1.0
            };

            // Synthesis filter (clause 4.2.3 eq. (154)).
            let alpha = &sub_alpha[sf];
            let mut s = [0f64; NS];
            for n in 0..NS {
                let mut acc = pscale * exp[n];
                for i in 0..NP {
                    let past = if n > i {
                        s[n - 1 - i]
                    } else {
                        self.synth[i - n]
                    };
                    acc += alpha[i] * past;
                }
                s[n] = acc;
            }
            for i in 0..NP {
                self.synth[i] = s[NS - 1 - i];
            }

            // Adaptive spectral postfilter (clause 4.2.4):
            // numerator FIR with the SST-smoothed set (eq. (165)),
            // denominator IIR with the 0,75^i weighting
            // (eq. (166)), brightness (eq. (167)), AGC (eq. (168)).
            let num = &sub_num[sf];
            let mut wden = [0f64; NP];
            let mut w = 1.0;
            for i in 0..NP {
                w *= 0.75;
                wden[i] = w * alpha[i];
            }
            let mut post = [0f64; NS];
            let mut bright = [0f64; NS];
            for n in 0..NS {
                let mut acc = s[n];
                for i in 0..NP {
                    let past = if n > i {
                        s[n - 1 - i]
                    } else {
                        self.pf_in[i - n]
                    };
                    acc -= num[i] * past;
                }
                for i in 0..NP {
                    let past = if n > i {
                        post[n - 1 - i]
                    } else {
                        self.pf_out[i - n]
                    };
                    acc += wden[i] * past;
                }
                post[n] = acc;
                let prev = if n > 0 { post[n - 1] } else { self.pf_bright };
                bright[n] = acc - 0.2 * prev;
            }
            for i in 0..NP {
                self.pf_in[i] = s[NS - 1 - i];
                self.pf_out[i] = post[NS - 1 - i];
            }
            self.pf_bright = post[NS - 1];

            let in_e: f64 = s.iter().map(|x| x * x).sum();
            let out_e: f64 = bright.iter().map(|x| x * x).sum();
            let sscale = if out_e > 1e-30 {
                (in_e / out_e).sqrt()
            } else {
                1.0
            };
            for (n, b) in bright.iter().enumerate() {
                self.agc = 0.9875 * self.agc + 0.0125 * sscale;
                let y = b * self.agc;
                // 13-bit left-justified output (GSM 06.07 table 4).
                let s13 = (y * 4096.0).round().clamp(-4096.0, 4095.0) as i16;
                out[sf * NS + n] = s13 << 3;
            }

            // Update the long-term filter state with the combined
            // excitation (clause 4.2.5 eqs. (169)/(170)) and the
            // prefilter history with its own output.
            shift_append(&mut self.ltp_hist, &ex);
            shift_append(&mut self.pre_hist, &exp);
        }

        self.prev_alpha = alpha_cur;
        self.prev_num = num_cur;
        self.prev_r0q = r0q_cur;
        self.prev_refl = refl;
        out
    }

    /// Clause 4.1.8.5 eqs. (100)/(101): the adaptive-codebook
    /// output `b_L(n)` for a (possibly fractional) lag in
    /// 1/6-sample units, computed in order from 0 so that
    /// in-subframe references resolve to already-generated samples.
    fn adaptive_codevector(&self, lag_sixths: i32) -> [f64; NS] {
        let mut b = [0f64; NS];
        for n in 0..NS {
            b[n] = interp_delayed(&self.ltp_hist, &b, n as isize, lag_sixths);
        }
        b
    }
}

/// Shift a history buffer left by one subframe and append the new
/// 40 samples (clause 4.2.5 eqs. (169)/(170)).
fn shift_append(hist: &mut [f64; HIST], new: &[f64; NS]) {
    hist.copy_within(NS.., 0);
    hist[HIST - NS..].copy_from_slice(new);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn homing_frame_roundtrip() {
        let h = hr_decoder_homing_frame();
        assert!(is_hr_decoder_homing_frame(&h));
        assert_eq!(h.to_cod_words(), HR_DECODER_HOMING_WORDS);
    }

    /// Clause 5.4: from the home state, a decoder homing frame's
    /// output is the encoder homing frame; the decoder stays home.
    #[test]
    fn homing_from_home_substitutes_encoder_homing_frame() {
        let mut dec = HrDecoder::new();
        assert!(dec.is_home());
        let out = dec.decode_frame(&hr_decoder_homing_frame());
        assert_eq!(out, [HR_ENCODER_HOMING_SAMPLE; HR_FRAME_SAMPLES]);
        assert!(dec.is_home());
    }

    /// Clause 5.4: a homing frame reaching a non-homed decoder
    /// decodes normally (output unspecified) and then resets; the
    /// next homing frame substitutes.
    #[test]
    fn homing_resets_non_homed_decoder() {
        let mut dec = HrDecoder::new();
        let mut speechy = hr_decoder_homing_frame();
        speechy.r0 = 20;
        let _ = dec.decode_frame(&speechy);
        assert!(!dec.is_home());
        let _ = dec.decode_frame(&hr_decoder_homing_frame());
        assert!(dec.is_home());
        let out = dec.decode_frame(&hr_decoder_homing_frame());
        assert_eq!(out, [HR_ENCODER_HOMING_SAMPLE; HR_FRAME_SAMPLES]);
    }

    /// The decoder is total over every syntactically valid frame:
    /// all fields at their extremes decode without panic and the
    /// output stays 13-bit left-justified.
    #[test]
    fn totality_over_extreme_frames() {
        let mut dec = HrDecoder::new();
        for mode in 0..4u8 {
            for fill in [0u16, 0xFFFF] {
                let mut w = [fill & 0x7FF; 18];
                w[5] = mode as u16;
                let p = HrParameters::from_cod_words(&w);
                let out = dec.decode_frame_no_homing(&p);
                for s in out {
                    assert_eq!(s % 8, 0, "13-bit left-justified output");
                }
            }
        }
    }

    /// Step-up/step-down are inverse recursions.
    #[test]
    fn step_up_down_roundtrip() {
        let r = [0.5, -0.3, 0.2, -0.1, 0.05, 0.0, 0.1, -0.2, 0.3, -0.4];
        let a = step_up(&r);
        let r2 = step_down(&a).expect("stable set");
        for (x, y) in r.iter().zip(r2.iter()) {
            assert!((x - y).abs() < 1e-12);
        }
    }

    /// Eq. (30) R0 decode: code 31 → −4 dB, code 1 → −64 dB, code 0
    /// is the silence floor.
    #[test]
    fn r0_decode_matches_printed_equation() {
        assert!((decode_r0(31) - 10f64.powf(-0.4)).abs() < 1e-12);
        assert!((decode_r0(1) - 10f64.powf(-6.4)).abs() < 1e-15);
        assert_eq!(decode_r0(0), 0.0);
    }
}
