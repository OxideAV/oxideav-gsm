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
//! EN 300 969 is a *functional* description; clause 5.1 places the
//! bit-exact arithmetic in GSM 06.06. [`HrDecoder`] carries that
//! arithmetic as the operator-level rules staged in
//! `docs/audio/gsm/halfrate-traces/hr-fixed-point-schedules.md`
//! (Part A), written in the Recommendation's own vocabulary, over the
//! staged ROM tables: 16-bit words / 32-bit accumulators with the
//! saturating operators of [`super::fx`]. It reproduces every output
//! word of the GSM 06.07 `SEQ01`–`SEQ04.OUT` references
//! (`tests/conformance_hr_decode.rs`).
//!
//! Points where the arithmetic departs from a literal reading of
//! the printed equations (schedules §A, errata #409):
//!
//! * the decoder never forms R(0): it uses the R0 amplitude word
//!   `table[2·R0]` directly, and `sqrt(RS)` = `8·A·sqrt(Π(1−r²))`
//!   (eq. (132)'s `Ns` appears as 64);
//! * the GSP0 √-components are Q14 words (erratum G3);
//! * a saturating reflection → direct-form conversion makes the
//!   frame reuse the previous frame's coefficient sets;
//! * the postfilter numerator is the AFLAT-smoothed version of the
//!   *widened* denominator, with the SST window words of §A.2;
//! * the pitch prefilter applies the 10-tap phase column even at
//!   integer lags, and its memory holds its unscaled output;
//! * the postfilter AGC word starts from 0 after a reset and only its
//!   16-bit word survives a subframe boundary.
//!
//! The `f64` helpers `step_up` / `step_down` serve the encoder's
//! remaining floating-point stages (INT_LPC decision, weighting).

#![allow(clippy::needless_range_loop)]

use super::fx::*;
use super::tables::*;
use super::{HR_FRAME_SAMPLES, HR_SUBFRAME_SAMPLES, HR_SUBFRAMES, HrParameters, SubframeParams};

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
pub(super) fn vq_index(table: &[i16], flat: usize) -> usize {
    let w = table[flat / 2] as u16;
    if flat.is_multiple_of(2) {
        (w >> 8) as usize
    } else {
        (w & 0xFF) as usize
    }
}

/// Clause 4.1.4: decode the three LPC vector-quantiser codes into
/// the ten reflection coefficients (Q15 → f64).
#[cfg(test)]
pub(super) fn dequant_reflection(lpc1: u16, lpc2: u16, lpc3: u8) -> [f64; NP] {
    dequant_reflection_words(lpc1, lpc2, lpc3).map(|w| w as f64 / 32768.0)
}

/// Reflection coefficients → direct-form LPC coefficients `α_i` for
/// the clause 4.2.3 synthesis form `s(n) = ex_ps(n) + Σ α_i s(n-i)`
/// (eq. (154) adds the prediction, so `α` is the plus convention).
/// The staged codebooks hold the reflection coefficients in the
/// convention of the FLAT/AFLAT lattices of clauses 4.1.3/4.1.4
/// (the `j`-th stage of `A(z) = 1 + Σ a_i z⁻ⁱ` has `a_j = r_j`):
/// the step-up here is the plain Levinson recursion. The opposite
/// pairing mirrors the spectrum (`A(−z)`) — nearly harmless on
/// flat unvoiced frames, but it turns every voiced set into a
/// non-predicting filter (measured −1 dB "prediction gain" against
/// the +10 dB the coefficient sets carry on the staged GSM 06.07
/// encoder-input speech, versus +10 dB with this pairing).
pub(super) fn step_up(r: &[f64; NP]) -> [f64; NP] {
    let mut a = [0f64; NP];
    for j in 0..NP {
        let mut next = a;
        next[j] = r[j];
        for i in 0..j {
            next[i] = a[i] + r[j] * a[j - 1 - i];
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

// ─── Fixed-point building blocks (schedules Part A) ───

/// Eq. (156) bandwidth-widening words `⌊0,75^i·32768⌋`, i = 1..10.
const PF_DEN_WEIGHTS: [i16; NP] = [
    0x6000, 0x4800, 0x3600, 0x2880, 0x1E60, 0x16C8, 0x1116, 0x0CD0, 0x099C, 0x0735,
];

/// Eq. (158) postfilter SST window words `W(1..10)` (Q15, applied
/// with a 32×16 multiply). EN 300 969 does not print them; they are
/// given by the staged half-rate schedules (§A.2 step 2.3).
const PF_SST_WEIGHTS: [i16; NP] = [
    0x7F5C, 0x7D76, 0x7A5B, 0x7622, 0x70EC, 0x6ADD, 0x641F, 0x5CDD, 0x5546, 0x4D86,
];

/// The R0-based energy right shift `k(R0)` (schedules §0.2), shared
/// by the postfilter and the encoder's open-loop search.
#[doc(hidden)]
pub fn r0_energy_shift(r0: u8) -> i16 {
    match r0 {
        0..=18 => 0,
        19..=21 => 2,
        22..=23 => 3,
        24 => 4,
        25..=26 => 5,
        27..=28 => 6,
        29 => 7,
        _ => 8,
    }
}

/// Clause 4.1.4 reflection-coefficient decode (schedules §A.1 step
/// 2): the three VQ codes → ten Q15 reflection-coefficient words.
pub(super) fn dequant_reflection_words(lpc1: u16, lpc2: u16, lpc3: u8) -> [i16; NP] {
    let mut r = [0i16; NP];
    for k in 0..3 {
        r[k] = RC_SCALAR_DEQUANT[vq_index(&RC_VQ_SEG1, 3 * lpc1 as usize + k)];
        r[3 + k] = RC_SCALAR_DEQUANT[vq_index(&RC_VQ_SEG2, 3 * lpc2 as usize + k)];
    }
    for k in 0..4 {
        r[6 + k] = RC_SCALAR_DEQUANT[vq_index(&RC_VQ_SEG3, 4 * lpc3 as usize + k)];
    }
    r
}

/// The 32-bit step-up (reflection → direct form) of schedules §A.1
/// step 3. Returns the 32-bit coefficients (`a_i` scaled by 2¹⁶
/// over the Q11 words) after the last stage and the saturation flag.
pub(super) fn step_up_long(r: &[i16; NP]) -> ([i32; NP], bool) {
    let mut a = [0i32; NP];
    let mut flag = false;
    for i in 0..NP {
        let mut next = a;
        next[i] = l_mult(0x0800, r[i]);
        for j in 0..i {
            next[j] = l_add(l_mpy_ls(a[i - j - 1], r[i]), a[j]);
        }
        for &l in next.iter().take(i + 1) {
            if l == i32::MAX || l == i32::MIN {
                flag = true;
            }
        }
        a = next;
    }
    (a, flag)
}

/// Step-up to Q11 direct-form words, with the §A.1 saturation flag.
pub(super) fn step_up_words(r: &[i16; NP]) -> ([i16; NP], bool) {
    let (a, mut flag) = step_up_long(r);
    let mut w = [0i16; NP];
    for i in 0..NP {
        w[i] = round(a[i]);
        if w[i] == i16::MAX || w[i] == i16::MIN {
            flag = true;
        }
    }
    (w, flag)
}

/// The A → r step-down of schedules §A.2 step 2.1 (Q11 words in,
/// Q15 reflection words out), with the §A.3 stability flag.
pub(super) fn step_down_words(d: &[i16; NP]) -> ([i16; NP], bool) {
    let sat = |w: i16| w == i16::MAX || w == i16::MIN;
    let mut a = *d;
    let mut rho = [0i16; NP];
    let mut flag = false;
    for i in (1..NP).rev() {
        rho[i] = shl(a[i], 4);
        let (q, c, s) = if rho[i] == i16::MIN {
            flag = true;
            (0i16, 0i16, 2i16)
        } else {
            if sat(rho[i]) {
                flag = true;
            }
            let l = l_msu(i32::MAX, rho[i], rho[i]);
            let n = norm_l(l);
            let q = divide_s(0x2000, extract_h(l_shl(l, n)));
            let c = mult_r(rho[i], q);
            if sat(c) {
                flag = true;
            }
            (q, c, 2 + n)
        };
        let old = a;
        for j in 0..i {
            a[j] = round(l_shl(l_msu(l_mult(old[j], q), old[i - j - 1], c), s));
            if sat(a[j]) {
                flag = true;
            }
        }
    }
    rho[0] = shl(a[0], 4);
    if sat(rho[0]) {
        flag = true;
    }
    (rho, flag)
}

/// r → model autocorrelation `R(0..10)` (schedules §A.2 step 2.2,
/// also the encoder's §B.4 step 1), `R(0) = 0x7fffffff`.
pub(super) fn rc_to_autocorr_long(r: &[i16; NP]) -> [i32; NP + 1] {
    let mut rr = [0i32; NP + 1];
    rr[0] = i32::MAX;
    let mut a = [0i32; NP];
    for i in 0..NP {
        let mut next = a;
        next[i] = l_mult(0x0800, r[i]);
        for j in 0..i {
            next[j] = l_add(l_mpy_ls(a[i - j - 1], r[i]), a[j]);
        }
        a = next;
        let mut acc = l_negate(l_mpy_ll(a[0], rr[i]));
        for j in 1..=i {
            acc = l_sub(acc, l_mpy_ll(a[j], rr[i - j]));
        }
        rr[i + 1] = l_shl(acc, 4);
    }
    rr
}

/// The AFLAT division rule (schedules §A.2 step 2.4): `None` when the
/// recursion stops.
#[inline]
fn aflat_divide(num: i16, den: i16) -> Option<i16> {
    let a = abs_s(num);
    if den <= 0 || a >= den {
        return None;
    }
    let r = divide_s(a, den);
    Some(if num >= 0 { negate(r) } else { r })
}

/// Postfilter numerator from the widened denominator words
/// (schedules §A.2 step 2: step-down, r → R, SST window, AFLAT
/// recursion, step-up).
fn postfilter_numerator(den: &[i16; NP]) -> [i16; NP] {
    let (rho, _) = step_down_words(den);
    let rr = rc_to_autocorr_long(&rho);
    let mut rs = [0i32; NP + 1];
    rs[0] = rr[0];
    for i in 1..=NP {
        rs[i] = l_mpy_ls(rr[i], PF_SST_WEIGHTS[i - 1]);
    }
    let mut rq = [0i16; NP];
    let first = aflat_divide(round(rs[1]), round(rs[0]));
    if let Some(r0) = first {
        rq[0] = r0;
        for v in rs.iter_mut() {
            *v = l_shr(*v, 2);
        }
        // P(i), i = 0..9; V(i), i = -9..9 stored at offset 9.
        let mut p = [0i32; NP];
        let mut v = [0i32; 2 * NP - 1];
        p.copy_from_slice(&rs[..NP]);
        v[9..9 + NP].copy_from_slice(&rs[1..=NP]);
        for i in 1..NP {
            v[9 - i] = rs[i - 1];
        }
        let mut r = r0;
        for j in 0..NP {
            if j > 0 {
                let n = norm_l(p[0]);
                let num = round(l_shl(v[9], n));
                let den = round(l_shl(p[0], n));
                match aflat_divide(num, den) {
                    Some(x) => {
                        r = x;
                        rq[j] = x;
                    }
                    None => break,
                }
            }
            if j == NP - 1 {
                break;
            }
            let b = (8 - j) as isize;
            let r2 = mult_r(r, r);
            let vi = |i: isize| v[(9 + i) as usize];
            let mut np = p;
            let mut nv = v;
            for i in 0..=b {
                let iu = i as usize;
                np[iu] = l_add(
                    l_add(l_add(l_mpy_ls(vi(i), r), p[iu]), l_mpy_ls(p[iu], r2)),
                    l_mpy_ls(vi(-i), r),
                );
            }
            for i in -b..=b {
                nv[(9 + i) as usize] = l_add(
                    l_shl(l_mpy_ls(p[(i + 1).unsigned_abs()], r), 1),
                    l_add(l_mpy_ls(vi(-i - 1), r2), vi(i + 1)),
                );
            }
            p = np;
            v = nv;
        }
    }
    step_up_words(&rq).0
}

/// RS_est (schedules §A.3): `sqrt(RS)` as a normalised pair from the
/// reflection coefficients and the R0 amplitude word.
pub(super) fn rs_estimate(r: &[i16; NP], amp: i16) -> (i16, i16) {
    let mut p: i16 = i16::MIN;
    let mut s: i16 = -6;
    for &ri in r.iter() {
        let t = mac_r(i32::MIN, ri, ri);
        let l = l_mult(t, p);
        let n = norm_s(extract_h(l));
        p = round(l_shl(l, n));
        s += n;
    }
    p = abs_s(p);
    let mut q = sqroot(l_deposit_h(p));
    let half = extract_h(l_shr(l_deposit_h(s), 1));
    if s & 1 != 0 {
        q = mult_r(q, 0x5a82);
    }
    let l = l_mult(amp, q);
    let n = norm_l(l);
    (round(l_shl(l, n)), half + n)
}

/// Plain energy `E(x)` (schedules §0.2): normalised long and its
/// `norm_l`, `(0, 0)` for silence.
pub(super) fn energy_plain(x: &[i16; NS]) -> (i32, i16) {
    let mut l = 0i32;
    for &v in x.iter() {
        l = l_mac(l, v, v);
    }
    if l == 0 {
        return (0, 0);
    }
    normalise(l)
}

/// Shifted energy `E_k(x)` (schedules §0.2).
fn energy_shifted(x: &[i16; NS], k: i16) -> (i32, i16) {
    let mut l = 0i32;
    let e = if k <= 1 {
        for &v in x.iter() {
            l = l_mac(l, v, shr(v, k));
        }
        k
    } else {
        let k2 = shift_r(k, -1);
        for &v in x.iter() {
            let s = shr(v, k2);
            l = l_mac(l, s, s);
        }
        2 * k2
    };
    if l == 0 {
        return (0, 0);
    }
    let (ln, n) = normalise(l);
    (ln, n - e)
}

/// ρ = sqrt(RS)/sqrt(energy) (schedules §A.6 step 1) from an energy
/// pair `(L, n)`.
pub(super) fn rho_from_energy(l: i32, n: i16, rs: (i16, i16)) -> (i16, i16) {
    let t = sqroot(l);
    let mut n = n;
    let mut l = if n & 1 != 0 {
        n -= 1;
        l_mult(0x5a82, t)
    } else {
        l_deposit_h(t)
    };
    let m = norm_l(l);
    n = (n >> 1) + m;
    l = l_shl(l, m);
    let q = divide_s(shr(rs.0, 1), round(l));
    let sh = rs.1 - n - 1;
    let m2 = norm_s(q);
    (shl(q, m2), sh + m2)
}

/// Excitation scaling by a GSP0 table word (schedules §A.6 step 2).
/// Returns the scaled vector and the gain word (β for the LTP
/// vector).
pub(super) fn scale_vector(x: &[i16; NS], t: i16, rho: (i16, i16)) -> ([i16; NS], i16) {
    let mut l = l_mult(t, rho.0);
    let n = norm_s(extract_h(l));
    l = l_shl(l, n);
    let g_sh = n + rho.1 - 1;
    let mut y = [0i16; NS];
    if g_sh >= 0 {
        if g_sh > 0 {
            l = l_shr(l, g_sh);
        }
        let g = round(l);
        for (o, &v) in y.iter_mut().zip(x.iter()) {
            *o = extract_h(l_mac(0x8000, g, v));
        }
        (y, g)
    } else {
        let g = round(l);
        let pre = l_shl(0x8000, g_sh);
        for (o, &v) in y.iter_mut().zip(x.iter()) {
            *o = extract_h(l_shr(l_mac(pre, g, v), g_sh));
        }
        (y, i16::MAX)
    }
}

/// Automatic gain control (schedules §A.9): the gain word
/// `sqrt(E_in/E_out)/2` (Q15).
fn agc_gain(y: &[i16; NS], e_in: (i16, i16), k: i16) -> i16 {
    let (l, sh) = energy_shifted(y, k);
    if l == 0 {
        return 0;
    }
    let sh_out = add(sh, 1);
    let m_out = round(l);
    let m_in = shr(e_in.0, 1);
    let mut q = divide_s(m_in, m_out);
    let n = norm_s(q);
    q = shl(q, n);
    let mut s = add(sub(e_in.1, sh_out), n);
    let mut t = sqroot(l_deposit_h(q));
    if s + 2 < 0 {
        return i16::MAX;
    }
    if s & 1 != 0 {
        t = mult(t, 0x5a82);
    }
    s = (s >> 1) + 1;
    if s > 0 {
        t = shr(t, s);
    }
    t
}

/// VSELP codevector (schedules §A.5, eq. (108)): bit `i` (LSB first)
/// set → θ_i = +1.
pub(super) fn codevector_words(basis: &[[i16; 40]], codeword: u16) -> [i16; NS] {
    let mut u = [0i16; NS];
    for (n, o) in u.iter_mut().enumerate() {
        let mut l = 0i32;
        for (i, v) in basis.iter().enumerate() {
            let s: i16 = if (codeword >> i) & 1 == 1 {
                0x4000
            } else {
                -0x4000
            };
            l = l_mac(l, s, v[n]);
        }
        *o = extract_h(l_shl(l, 1));
    }
    u
}

/// Adaptive-codebook vector `b_L(n)` (schedules §A.4): passes of
/// multiples of the lag, integer phases copy, fractional phases use
/// the 10-tap interpolator with round-half-up.
pub(super) fn adaptive_codebook_words(hist: &[i16; HIST], lag6: i16) -> [i16; NS] {
    let mut b = [0i16; NS];
    let mut n = 0usize;
    let mut run = 0i16;
    while n < NS {
        run = add(run, lag6);
        let ip = (run / 6) as usize;
        let jj = (run % 6) as usize;
        let end = ip.min(NS);
        let fetch = |b: &[i16; NS], m: isize| -> i16 {
            if m < 0 {
                hist[(HIST as isize + m) as usize]
            } else {
                b[m as usize]
            }
        };
        while n < end {
            let base = n as isize - ip as isize;
            b[n] = if jj == 0 {
                fetch(&b, base)
            } else {
                let mut l = 0x8000i32;
                for (i, row) in INTERP_FILTER_10.iter().enumerate() {
                    l = l_mac(l, fetch(&b, base - 5 + i as isize), row[jj]);
                }
                extract_h(l)
            };
            n += 1;
        }
    }
    b
}

/// One all-pole filter pass with the Q11 direct-form words (schedules
/// §A.10): `mem` holds `y(n−1)..y(n−10)` and is updated.
pub(super) fn all_pole(a: &[i16; NP], x: &[i16; NS], mem: &mut [i16; NP]) -> [i16; NS] {
    let mut y = [0i16; NS];
    for n in 0..NS {
        let mut l = 0x0800i32;
        for k in 0..NP {
            let past = if n > k { y[n - 1 - k] } else { mem[k - n] };
            l = l_msu(l, past, a[k]);
        }
        l = l_shl(l, 4);
        l = l_msu(l, x[n], i16::MIN);
        y[n] = extract_h(l);
    }
    for k in 0..NP {
        mem[k] = y[NS - 1 - k];
    }
    y
}

/// The postfilter numerator FIR (schedules §A.11 step 2): `mem`
/// holds the last ten inputs and is updated.
fn all_zero(a: &[i16; NP], x: &[i16; NS], mem: &mut [i16; NP]) -> [i16; NS] {
    let mut y = [0i16; NS];
    for n in 0..NS {
        let mut l = 0x0800i32;
        for k in 0..NP {
            let past = if n > k { x[n - 1 - k] } else { mem[k - n] };
            l = l_mac(l, past, a[k]);
        }
        l = l_shl(l, 4);
        l = l_msu(l, x[n], i16::MIN);
        y[n] = extract_h(l);
    }
    for k in 0..NP {
        mem[k] = x[NS - 1 - k];
    }
    y
}

/// Soft-interpolate two Q11 word sets (schedules §A.2 step 4 /
/// §A.3).
#[inline]
pub(super) fn interp_words(new: &[i16; NP], old: &[i16; NP], k: usize) -> [i16; NP] {
    let mut o = [0i16; NP];
    for i in 0..NP {
        o[i] = mac_r(
            l_mult(new[i], SOFT_INTERP_CURRENT[k]),
            old[i],
            SOFT_INTERP_PREVIOUS[k],
        );
    }
    o
}

/// Shift a word history left by one subframe and append 40 samples
/// (clause 4.2.5 eqs. (169)/(170)).
pub(super) fn shift_append_words(hist: &mut [i16; HIST], new: &[i16; NS]) {
    hist.copy_within(NS.., 0);
    hist[HIST - NS..].copy_from_slice(new);
}

/// Per-frame intermediate signals of the last decoded frame, at the
/// stage boundaries of schedules Part D.1 (debug/conformance aid).
#[doc(hidden)]
#[derive(Debug, Clone)]
pub struct HrDecodeStages {
    /// ex(n), the combined excitation (§A.7).
    pub excitation: [i16; HR_FRAME_SAMPLES],
    /// ex_ps(n), the pitch-prefilter output (§A.8).
    pub prefilter: [i16; HR_FRAME_SAMPLES],
    /// s(n), the synthesis-filter output (§A.10).
    pub synth: [i16; HR_FRAME_SAMPLES],
    /// The postfilter output before the 13-bit mask (§A.11).
    pub post16: [i16; HR_FRAME_SAMPLES],
}

impl Default for HrDecodeStages {
    fn default() -> Self {
        Self {
            excitation: [0; HR_FRAME_SAMPLES],
            prefilter: [0; HR_FRAME_SAMPLES],
            synth: [0; HR_FRAME_SAMPLES],
            post16: [0; HR_FRAME_SAMPLES],
        }
    }
}

// ─── Decoder state ───

/// GSM 06.20 half-rate speech decoder state (clause 4.2), carried in
/// the 16/32-bit fixed-point arithmetic of clause 5.1.
#[derive(Debug, Clone)]
pub struct HrDecoder {
    /// Long-term filter state `r(n)`, `n = -147..-1` (clause 4.2.5).
    ltp_hist: [i16; HIST],
    /// Pitch-prefilter state: its own unscaled output (§A.8).
    pre_hist: [i16; HIST],
    /// Synthesis filter memory `s(n-1)..s(n-10)` (clause 4.2.3).
    synth: [i16; NP],
    /// Postfilter numerator input memory, denominator output memory
    /// and post-emphasis memory (clause 4.2.4).
    pf_in: [i16; NP],
    pf_out: [i16; NP],
    pf_emph: i16,
    /// Running AGC word `S'scale` (eq. (168)).
    agc: i16,
    /// Previous frame's reflection words, direct-form words,
    /// postfilter numerator / denominator words and R0 amplitude
    /// word (clauses 4.1.6 and 4.1.11.1 eqs. (131a)/(132)).
    prev_refl: [i16; NP],
    prev_a: [i16; NP],
    prev_num: [i16; NP],
    prev_den: [i16; NP],
    prev_amp: i16,
    /// Clause 5.4 homing: `true` while the decoder sits in its home
    /// state (fresh, reset, or right after a decoder homing frame).
    home: bool,
    /// Stage signals of the last frame.
    stages: HrDecodeStages,
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
            ltp_hist: [0; HIST],
            pre_hist: [0; HIST],
            synth: [0; NP],
            pf_in: [0; NP],
            pf_out: [0; NP],
            pf_emph: 0,
            agc: 0,
            prev_refl: [0; NP],
            prev_a: [0; NP],
            prev_num: [0; NP],
            prev_den: [0; NP],
            prev_amp: 0,
            home: true,
            stages: HrDecodeStages::default(),
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

    /// Stage signals of the most recently decoded frame.
    #[doc(hidden)]
    pub fn last_stages(&self) -> &HrDecodeStages {
        &self.stages
    }

    /// Decode one parameter frame into 160 PCM samples (13-bit
    /// left-justified in i16, the GSM 06.07 output convention),
    /// applying the clause 5.4 decoder-homing protocol: a decoder
    /// homing frame decodes normally (with the output substituted
    /// by the encoder homing frame when the decoder was already in
    /// its home state) and then resets the decoder.
    pub fn decode_frame(&mut self, p: &HrParameters) -> [i16; HR_FRAME_SAMPLES] {
        let was_home = self.home;
        if is_hr_decoder_homing_frame(p) {
            let out = if was_home {
                [HR_ENCODER_HOMING_SAMPLE; HR_FRAME_SAMPLES]
            } else {
                self.decode_frame_no_homing(p)
            };
            self.reset();
            return out;
        }
        self.decode_frame_no_homing(p)
    }

    /// Decode one parameter frame without the homing protocol.
    pub fn decode_frame_no_homing(&mut self, p: &HrParameters) -> [i16; HR_FRAME_SAMPLES] {
        self.home = false;
        let mode = p.mode_code & 3;
        let amp = R0_DECODE_TABLE[2 * (p.r0 as usize & 31)];

        // ── Frame parameters (§A.1, §A.2) ──
        let mut refl = dequant_reflection_words(p.lpc1, p.lpc2, p.lpc3);
        let (mut a_cur, flagged) = step_up_words(&refl);
        let (num_cur, den_cur);
        if flagged {
            refl = self.prev_refl;
            a_cur = self.prev_a;
            num_cur = self.prev_num;
            den_cur = self.prev_den;
        } else {
            let mut d = [0i16; NP];
            for i in 0..NP {
                d[i] = mult_r(a_cur[i], PF_DEN_WEIGHTS[i]);
            }
            den_cur = d;
            num_cur = postfilter_numerator(&d);
        }

        // ── Per-subframe coefficient sets and sqrt(RS) (§A.2 step
        // 4, §A.3) ──
        let mut sub_a = [[0i16; NP]; HR_SUBFRAMES];
        let mut sub_num = [[0i16; NP]; HR_SUBFRAMES];
        let mut sub_den = [[0i16; NP]; HR_SUBFRAMES];
        let mut sub_rs = [(0i16, 0i16); HR_SUBFRAMES];
        let rs_cur = rs_estimate(&refl, amp);
        let rs_prev = rs_estimate(&self.prev_refl, self.prev_amp);
        for k in 0..HR_SUBFRAMES {
            let use_prev = |k: usize| k == 0;
            if k == HR_SUBFRAMES - 1 {
                sub_a[k] = a_cur;
                sub_num[k] = num_cur;
                sub_den[k] = den_cur;
                sub_rs[k] = rs_cur;
            } else if p.int_lpc {
                let ai = interp_words(&a_cur, &self.prev_a, k);
                let (rk, unstable) = step_down_words(&ai);
                if !unstable {
                    sub_a[k] = ai;
                    sub_num[k] = interp_words(&num_cur, &self.prev_num, k);
                    sub_den[k] = interp_words(&den_cur, &self.prev_den, k);
                    sub_rs[k] = rs_estimate(&rk, if k == 0 { self.prev_amp } else { amp });
                } else if use_prev(k) {
                    sub_a[k] = self.prev_a;
                    sub_num[k] = self.prev_num;
                    sub_den[k] = self.prev_den;
                    sub_rs[k] = rs_prev;
                } else {
                    sub_a[k] = a_cur;
                    sub_num[k] = num_cur;
                    sub_den[k] = den_cur;
                    sub_rs[k] = rs_cur;
                }
            } else if use_prev(k) {
                sub_a[k] = self.prev_a;
                sub_num[k] = self.prev_num;
                sub_den[k] = self.prev_den;
                sub_rs[k] = rs_prev;
            } else {
                sub_a[k] = a_cur;
                sub_num[k] = num_cur;
                sub_den[k] = den_cur;
                sub_rs[k] = rs_cur;
            }
        }

        let pf_shift = r0_energy_shift(p.r0 & 31);
        let mut out = [0i16; HR_FRAME_SAMPLES];
        let mut level: i32 = 0;
        for sf in 0..HR_SUBFRAMES {
            let rs = sub_rs[sf];
            let off = sf * NS;
            // ── Excitation (§A.4–§A.7) ──
            let (ex, beta, lag6, gsp0) = match &p.sub {
                SubframeParams::Unvoiced { code1, code2, gsp0 } => {
                    let g = gsp0[sf] as usize & 31;
                    let row = &GSP0_VQ[0][g];
                    let u1 = codevector_words(&BASIS_VECTORS_MODE0[0], code1[sf] as u16);
                    let u2 = codevector_words(&BASIS_VECTORS_MODE0[1], code2[sf] as u16);
                    let (e1, n1) = energy_plain(&u1);
                    let (e2, n2) = energy_plain(&u2);
                    let (y1, _) = scale_vector(&u1, row[0], rho_from_energy(e1, n1, rs));
                    let (y2, _) = scale_vector(&u2, row[1], rho_from_energy(e2, n2, rs));
                    let mut ex = [0i16; NS];
                    for n in 0..NS {
                        ex[n] = add(y2[n], y1[n]);
                    }
                    (ex, 0i16, 0i16, g)
                }
                SubframeParams::Voiced {
                    lag1,
                    lag_delta,
                    code,
                    gsp0,
                } => {
                    if sf == 0 {
                        level = *lag1 as i32;
                    } else {
                        level = (level + lag_delta[sf - 1] as i32 - 8).clamp(0, 255);
                    }
                    let lag6 = LAG_TABLE[level as usize];
                    let g = gsp0[sf] as usize & 31;
                    let row = &GSP0_VQ[mode as usize][g];
                    let b = adaptive_codebook_words(&self.ltp_hist, lag6);
                    let u = codevector_words(&BASIS_VECTORS_MODE123, code[sf] & 0x1ff);
                    let e = sub(shl(rs.1, 1), 3);
                    let (eb, nb) = if e < 0 {
                        energy_shifted(&b, negate(e))
                    } else {
                        energy_plain(&b)
                    };
                    let (eu, nu) = energy_plain(&u);
                    let (yb, beta) = scale_vector(&b, row[0], rho_from_energy(eb, nb, rs));
                    let (yu, _) = scale_vector(&u, row[1], rho_from_energy(eu, nu, rs));
                    let mut ex = [0i16; NS];
                    for n in 0..NS {
                        ex[n] = add(yb[n], yu[n]);
                    }
                    (ex, beta, lag6, g)
                }
            };
            self.stages.excitation[off..off + NS].copy_from_slice(&ex);

            // ── Adaptive pitch prefilter (§A.8) ──
            let ex_ps = if mode != 0 {
                let mut e = add(negate(shl(rs.1, 1)), 3);
                let e_in = if e > 0 {
                    let (l, sh) = energy_shifted(&ex, e);
                    (round(l), sh)
                } else {
                    e = 0;
                    let (l, n) = energy_plain(&ex);
                    (round(l), n)
                };
                let s = SQRT_P0[(mode - 1) as usize][gsp0];
                let min = if s > beta { beta } else { s };
                let xi = mult_r(0x2666, min);
                let ip = (lag6 / 6) as isize;
                let jj = (lag6 % 6) as usize;
                let mut c = [0i16; NP];
                for i in 0..NP {
                    c[i] = mult_r(INTERP_FILTER_10[i][jj], xi);
                }
                let mut exp = [0i16; NS];
                for n in 0..NS {
                    let fetch = |exp: &[i16; NS], m: isize| -> i16 {
                        if m < 0 {
                            self.pre_hist[(HIST as isize + m) as usize]
                        } else {
                            exp[m as usize]
                        }
                    };
                    let base = n as isize - ip - 5;
                    let mut l = l_deposit_h(ex[n]);
                    for i in 0..NP - 1 {
                        l = l_mac(l, fetch(&exp, base + i as isize), c[i]);
                    }
                    exp[n] = mac_r(l, fetch(&exp, base + 9), c[9]);
                }
                let g = agc_gain(&exp, e_in, e);
                let mut y = [0i16; NS];
                for n in 0..NS {
                    y[n] = round(l_shl(l_mult(exp[n], g), 1));
                }
                shift_append_words(&mut self.pre_hist, &exp);
                y
            } else {
                shift_append_words(&mut self.pre_hist, &ex);
                ex
            };
            self.stages.prefilter[off..off + NS].copy_from_slice(&ex_ps);

            // ── Synthesis filter (§A.10) ──
            let s = all_pole(&sub_a[sf], &ex_ps, &mut self.synth);
            self.stages.synth[off..off + NS].copy_from_slice(&s);

            // ── Adaptive spectral postfilter (§A.11) ──
            let (l, sh) = energy_shifted(&s, pf_shift);
            let e_in = (round(l), sh);
            let sbar = all_zero(&sub_num[sf], &s, &mut self.pf_in);
            let stil = all_pole(&sub_den[sf], &sbar, &mut self.pf_out);
            let mut shat = [0i16; NS];
            let mut prev = self.pf_emph;
            for n in 0..NS {
                shat[n] = msu_r(l_deposit_h(stil[n]), prev, 0x199a);
                prev = stil[n];
            }
            self.pf_emph = stil[NS - 1];
            let target = agc_gain(&shat, e_in, pf_shift);
            let mut g = self.agc;
            let mut lg = l_deposit_h(g);
            for n in 0..NS {
                lg = l_msu(lg, g, 0x019a);
                lg = l_mac(lg, target, 0x019a);
                g = extract_h(lg);
                let o = extract_h(l_shl(l_mult(g, shat[n]), 2));
                self.stages.post16[off + n] = o;
                out[off + n] = o & !7;
            }
            self.agc = g;

            // ── Long-term state update (§A.4 step 6) ──
            shift_append_words(&mut self.ltp_hist, &ex);
        }

        self.prev_refl = refl;
        self.prev_a = a_cur;
        self.prev_num = num_cur;
        self.prev_den = den_cur;
        self.prev_amp = amp;
        out
    }
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

    /// Random parameter frames (every field over its full range)
    /// decode without arithmetic panics in debug builds, and the
    /// output stays 13-bit left-justified.
    #[test]
    fn random_frames_are_total() {
        let mut dec = HrDecoder::new();
        let mut x: u32 = 0x1234_5678;
        let mut next = || {
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            x
        };
        for _ in 0..3000 {
            let w: [u16; 18] = core::array::from_fn(|_| next() as u16);
            let p = HrParameters::from_cod_words(&w);
            for s in dec.decode_frame_no_homing(&p) {
                assert_eq!(s & 7, 0);
            }
        }
    }

    /// The fixed-point step-up / step-down pair agree on a stable
    /// set (reflection words survive the round trip within 2 LSB of
    /// Q15 after the Q11 word rounding).
    #[test]
    fn fixed_step_up_down_roundtrip() {
        let r: [i16; NP] = [
            -30000, 25000, -12000, 7000, -2000, 3800, -3400, 2600, -3800, 4200,
        ];
        let (a, flag) = step_up_words(&r);
        assert!(!flag);
        let (r2, unstable) = step_down_words(&a);
        assert!(!unstable);
        for (x, y) in r.iter().zip(r2.iter()) {
            assert!((x - y).abs() < 400, "{x} vs {y}");
        }
    }
}
