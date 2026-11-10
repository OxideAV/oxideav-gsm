//! GSM 06.20 encoder front end in the clause 5.1 fixed-point
//! arithmetic: the clause 4.1.1 high-pass filter, the clause 4.1.2
//! segmentation buffer, the clause 4.1.3 FLAT covariance lattice,
//! the clause 4.1.5 R0 coding and the clause 4.1.4 / 4.1.4.1 AFLAT
//! vector-quantiser search — as the operator rules of
//! `docs/audio/gsm/halfrate-traces/hr-fixed-point-schedules.md`
//! Part B, over the staged ROM tables.

#![allow(clippy::needless_range_loop)]

use super::HR_FRAME_SAMPLES;
use super::decode::{rc_to_autocorr_long, vq_index};
use super::fx::*;
use super::tables::*;

/// Short-term predictor order (annex A.2 `Np`).
const NP: usize = 10;

/// Clause 4.1.2 buffer length.
pub(super) const BUF: usize = 195;

/// One biquad section of the clause 4.1.1 high-pass filter with its
/// double-precision output memory (schedules §B.1).
#[derive(Debug, Clone, Copy, Default)]
struct HpSection {
    /// x'(n−1), x'(n−2).
    x: [i16; 2],
    /// High words y_hi(n−1), y_hi(n−2).
    yhi: [i16; 2],
    /// 14-bit low parts y_lo(n−1), y_lo(n−2).
    ylo: [i16; 2],
}

impl HpSection {
    /// One sample through the section; returns the 32-bit sum after
    /// the left shift of step 5.
    #[inline]
    fn step(&mut self, xin: i16, c: &[i16]) -> i32 {
        let (b0, b1, b2, a2, a1) = (c[0], c[1], c[2], c[3], c[4]);
        let mut l = l_mult(self.ylo[1], a2);
        l = l_mac(l, self.ylo[0], a1);
        l = l_shr(l, 14);
        l = l_mac(l, self.yhi[1], a2);
        l = l_mac(l, self.yhi[0], a1);
        l = l_mac(l, b0, self.x[1]);
        l = l_mac(l, b1, self.x[0]);
        l = l_mac(l, b2, xin);
        l = l_shl(l, 1);
        self.x = [xin, self.x[0]];
        self.yhi = [extract_h(l), self.yhi[0]];
        self.ylo = [shr(extract_l(l), 2) & 0x3fff, self.ylo[0]];
        l
    }
}

/// The clause 4.1.1 high-pass filter: two cascaded biquads (gain
/// 0,5 overall).
#[derive(Debug, Clone, Copy, Default)]
pub(super) struct HighPassFx {
    s: [HpSection; 2],
}

impl HighPassFx {
    #[inline]
    pub(super) fn process(&mut self, x: i16) -> i16 {
        let x1 = shr(x & !7, 1);
        let l1 = self.s[0].step(x1, &HIGHPASS_COEFFS[0..5]);
        let y1 = round(l1);
        let l2 = self.s[1].step(y1, &HIGHPASS_COEFFS[5..10]);
        round(l_shl(l2, 1))
    }
}

/// Front-end outputs for one frame.
#[doc(hidden)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrontFrame {
    /// The R0 code.
    pub r0: u8,
    /// The three reflection-coefficient VQ codes.
    pub lpc: (u16, u16, u8),
    /// FLAT's unquantised reflection coefficients (Q15).
    pub r_flat: [i16; NP],
    /// The quantised reflection coefficients r̂1..r̂10 (Q15).
    pub rq: [i16; NP],
    /// Input shift σ and covariance normalisation π (diagnostics).
    pub sigma: i16,
    pub pi: i16,
    /// The 32-bit unquantised R0 amplitude (diagnostics).
    pub r0_amp: i32,
}

/// Fixed-point encoder front end (clauses 4.1.1–4.1.5).
#[derive(Debug, Clone)]
pub(super) struct FrontEnd {
    hp: HighPassFx,
    /// `s(0..=194)` of clause 4.1.2, high-pass output words.
    pub(super) buf: [i16; BUF],
}

impl Default for FrontEnd {
    fn default() -> Self {
        Self::new()
    }
}

impl FrontEnd {
    pub(super) fn new() -> Self {
        Self {
            hp: HighPassFx::default(),
            buf: [0; BUF],
        }
    }

    /// Shift the buffer and filter 160 new input samples into
    /// `s(35..194)`.
    pub(super) fn push(&mut self, samples: &[i16; HR_FRAME_SAMPLES]) {
        self.buf.copy_within(HR_FRAME_SAMPLES.., 0);
        for (i, &x) in samples.iter().enumerate() {
            self.buf[BUF - HR_FRAME_SAMPLES + i] = self.hp.process(x);
        }
    }

    /// FLAT + R0 + AFLAT VQ over the current buffer.
    pub(super) fn analyse(&self) -> FrontFrame {
        let (r_flat, r0, sigma, pi, r0_amp) = flat(&self.buf);
        let (lpc, rq) = aflat_vq(&r_flat);
        FrontFrame {
            r0,
            lpc,
            r_flat,
            rq,
            sigma,
            pi,
            r0_amp,
        }
    }
}

/// Clauses 4.1.3 and 4.1.5 (schedules §B.2, §B.3): returns FLAT's
/// reflection words, the R0 code, σ, π and the R0 amplitude.
fn flat(buf: &[i16; BUF]) -> ([i16; NP], u8, i16, i16, i32) {
    // Analysis interval a(j) = s(25 + j), j = 0..169.
    let mut a = [0i16; 170];
    a.copy_from_slice(&buf[25..195]);

    // Pre-scaling (§B.2).
    let mut p = 0x0011_7f0b_i32;
    for &x in a.iter().take(160) {
        let s = shr(x, 3);
        p = l_mac(p, s, s);
    }
    let mut or = p;
    for m in 0..10 {
        let o = shr(a[m], 3);
        let n = shr(a[m + 160], 3);
        p = l_msu(p, o, o);
        p = l_mac(p, n, n);
        or |= p;
    }
    let v = 6 - norm_l(or);
    let sigma: i16 = if v >= 0 { ((v + 1) >> 1) + 1 } else { 0 };
    for x in a.iter_mut() {
        *x = shr(*x, sigma);
    }

    // Covariance φ(i,k), i ≤ k, by diagonals with end-point
    // correction (eq. (5), 160 terms).
    let mut phi = [[0i32; NP + 1]; NP + 1];
    for d in 0..=NP {
        let mut l = 0i32;
        for n in 0..160 {
            l = l_mac(l, a[10 + n], a[10 + n - d]);
        }
        phi[0][d] = l;
        for i in 0..NP - d {
            l = l_msu(l, a[169 - i], a[169 - i - d]);
            l = l_mac(l, a[9 - i], a[9 - i - d]);
            phi[i + 1][i + 1 + d] = l;
        }
    }
    for i in 0..=NP {
        for k in 0..i {
            phi[i][k] = phi[k][i];
        }
    }
    let mut ord = 0i32;
    for i in 0..=NP {
        ord |= phi[i][i];
    }
    let pi = norm_l(ord);

    // R0 (§B.3).
    let mut e = l_add(l_shl(phi[0][0], pi - 1), l_shl(phi[NP][NP], pi - 1));
    let mut extra = 0i16;
    if e & 0x4000_0000 == 0 && e != 0 {
        e = l_shl(e, 1);
        extra = 1;
    }
    let cov_shift = 2 * sigma - (pi - 1) - extra - 1;
    let r0_amp = if e == 0 {
        0
    } else {
        let s = sqroot(e);
        let mut x = 8 - cov_shift;
        let mut l = if x & 1 != 0 {
            l_mult(s, 0x5a82)
        } else {
            l_deposit_h(s)
        };
        x >>= 1;
        if x > 0 {
            l = l_shr(l, x);
        }
        if x < 0 { 0 } else { l }
    };
    let rounded = round(r0_amp);
    let mut r0 = 31u8;
    for k in 0..31usize {
        if rounded < R0_DECODE_TABLE[2 * k + 1] {
            r0 = k as u8;
            break;
        }
    }

    // Normalise + window (eq. (6), §B.2).
    let sh = pi - 2;
    let mut w = [[0i32; NP + 1]; NP + 1];
    for i in 0..=NP {
        for k in 0..=NP {
            let mut x = l_shl(phi[i][k], sh);
            let d = i.abs_diff(k);
            if d > 0 {
                x = l_mpy_ll(x, FLAT_SST_COEFFS[d - 1]);
            }
            w[i][k] = x;
        }
    }

    let mut f = [[0i32; NP]; NP];
    let mut b = [[0i32; NP]; NP];
    let mut c = [[0i32; NP]; NP];
    for i in 0..NP {
        for k in 0..NP {
            f[i][k] = w[i][k];
            b[i][k] = w[i + 1][k + 1];
            c[i][k] = w[i][k + 1];
        }
    }

    let mut r = [0i16; NP];
    let mut psi = 0i16;
    for j in 0..NP {
        let dim = NP - j;
        if psi != 0 {
            for i in 0..dim {
                for k in 0..dim {
                    f[i][k] = l_shl(f[i][k], psi);
                    b[i][k] = l_shl(b[i][k], psi);
                    c[i][k] = l_shl(c[i][k], psi);
                }
            }
        }
        let m = dim - 1;
        let num = l_add(c[0][0], c[m][m]);
        let den = l_shr(l_add(l_add(l_add(f[0][0], b[0][0]), f[m][m]), b[m][m]), 1);
        let ns = norm_s(extract_h(den));
        let num_w = round(l_shl(num, ns));
        let den_w = round(l_shl(den, ns));
        let an = abs_s(num_w);
        if den_w <= 0 || an >= den_w {
            break;
        }
        let q = divide_s(an, den_w);
        let rj = if num_w > 0 { negate(q) } else { q };
        r[j] = rj;
        if j == NP - 1 {
            break;
        }
        // Stage update (eqs. (11)-(13)).
        let r2 = mult_r(rj, rj);
        let (fp, bp, cp) = (f, b, c);
        let nd = dim - 1;
        for i in 0..nd {
            for k in 0..nd {
                let x = l_mpy_ls(l_add(cp[i][k], cp[k][i]), rj);
                f[i][k] = l_add(l_add(fp[i][k], x), l_mpy_ls(bp[i][k], r2));
                let x1 = l_mpy_ls(l_add(cp[i + 1][k + 1], cp[k + 1][i + 1]), rj);
                b[i][k] = l_add(l_add(bp[i + 1][k + 1], x1), l_mpy_ls(fp[i + 1][k + 1], r2));
                let y = l_mpy_ls(l_add(bp[i][k + 1], fp[i][k + 1]), rj);
                c[i][k] = l_add(l_add(cp[i][k + 1], y), l_mpy_ls(cp[k + 1][i], r2));
            }
        }
        let mut o = 0i32;
        for l in 0..nd {
            o |= f[l][l];
            o |= b[l][l];
        }
        psi = if o <= 0 { 0 } else { norm_l(o) - 2 };
    }
    (r, r0, sigma, pi, r0_amp)
}

/// Read the `n`-coefficient candidate `row` of a packed table.
#[inline]
fn candidate(table: &[i16], row: usize, n: usize) -> [i16; 4] {
    let mut o = [0i16; 4];
    for (c, slot) in o.iter_mut().take(n).enumerate() {
        *slot = RC_SCALAR_DEQUANT[vq_index(table, row * n + c)];
    }
    o
}

/// Candidate residual error `Er` (eqs. (19)–(23), schedules §B.4
/// step 4) over the 16-bit initial conditions `p(0..order)`,
/// `v(−(order−1)..order−1)` (offset `order − 1`).
fn candidate_error(p0: &[i16; 4], v0: &[i16; 7], rc: &[i16; 4], order: usize) -> i16 {
    let off = 3isize;
    let mut p = *p0;
    let mut v = *v0;
    let mut r2 = [0i16; 4];
    for j in 0..order {
        r2[j] = mult_r(rc[j], rc[j]);
    }
    let vi = |v: &[i16; 7], i: isize| v[(i + off) as usize];
    for j in 0..order - 1 {
        let b = (order - j - 2) as isize;
        let (r, rr) = (rc[j], r2[j]);
        let (pp, vp) = (p, v);
        for i in 0..=b {
            let iu = i as usize;
            let mut l = 0x8000i32;
            l = l_mac(l, vi(&vp, i), r);
            l = l_mac(l, vi(&vp, -i), r);
            l = l_mac(l, pp[iu], rr);
            l = l_msu(l, pp[iu], i16::MIN);
            p[iu] = extract_h(l);
        }
        if p[0] >= 0x4000 {
            return i16::MAX;
        }
        for i in -b..=b {
            let pa = pp[(i + 1).unsigned_abs()];
            let mut l = 0x8000i32;
            l = l_msu(l, vi(&vp, i + 1), i16::MIN);
            l = l_mac(l, rr, vi(&vp, -i - 1));
            l = l_mac(l, r, pa);
            l = l_mac(l, r, pa);
            v[(i + off) as usize] = extract_h(l);
        }
    }
    let r = rc[order - 1];
    let rr = r2[order - 1];
    let mut l = 0x8000i32;
    l = l_mac(l, vi(&v, 0), r);
    l = l_mac(l, vi(&v, 0), r);
    l = l_mac(l, p[0], rr);
    l = l_msu(l, p[0], i16::MIN);
    extract_h(l)
}

/// Clause 4.1.4 / 4.1.4.1 AFLAT VQ search (schedules §B.4).
fn aflat_vq(r_flat: &[i16; NP]) -> ((u16, u16, u8), [i16; NP]) {
    struct Seg {
        order: usize,
        preq: &'static [i16],
        preq_rows: usize,
        vq: &'static [i16],
        subset: usize,
    }
    let segs = [
        Seg {
            order: 3,
            preq: &RC_PREQ_SEG1,
            preq_rows: 64,
            vq: &RC_VQ_SEG1,
            subset: 32,
        },
        Seg {
            order: 3,
            preq: &RC_PREQ_SEG2,
            preq_rows: 32,
            vq: &RC_VQ_SEG2,
            subset: 16,
        },
        Seg {
            order: 4,
            preq: &RC_PREQ_SEG3,
            preq_rows: 16,
            vq: &RC_VQ_SEG3,
            subset: 16,
        },
    ];
    let rr = rc_to_autocorr_long(r_flat);
    // 32-bit P(0..9) and V(−9..9) (offset 9).
    let mut pf = [0i32; NP];
    let mut vf = [0i32; 2 * NP - 1];
    for i in 0..NP {
        pf[i] = l_shr(rr[i], 2);
    }
    for i in -9isize..=8 {
        vf[(i + 9) as usize] = pf[(i + 1).unsigned_abs()];
    }
    vf[18] = l_shr(rr[NP], 2);

    let mut codes = [0usize; 3];
    let mut rq = [0i16; NP];
    let mut base_coeff = 0usize;
    for (ks, seg) in segs.iter().enumerate() {
        let order = seg.order;
        let mut p16 = [0i16; 4];
        let mut v16 = [0i16; 7];
        for i in 0..order {
            p16[i] = round(pf[i]);
        }
        let o = order as isize - 1;
        for i in -o..=o {
            v16[(i + 3) as usize] = round(vf[(i + 9) as usize]);
        }
        // Prequantiser: best four, strict improvement.
        let mut best4: [(i16, usize); 4] = [(i16::MAX, usize::MAX); 4];
        let mut filled = 0usize;
        for q in 0..seg.preq_rows {
            let er = candidate_error(&p16, &v16, &candidate(seg.preq, q, order), order);
            // Find insertion position: first slot whose error is
            // strictly greater than er.
            let mut pos = filled;
            for s in 0..filled {
                if er < best4[s].0 {
                    pos = s;
                    break;
                }
            }
            if pos < 4 {
                for s in (pos + 1..4).rev() {
                    best4[s] = best4[s - 1];
                }
                best4[pos] = (er, q);
                if filled < 4 {
                    filled += 1;
                }
            }
        }
        let mut best = (i16::MAX, usize::MAX);
        for &(_, q) in best4.iter().take(filled) {
            let mut rb = (0i16, usize::MAX);
            for row in q * seg.subset..(q + 1) * seg.subset {
                let er = candidate_error(&p16, &v16, &candidate(seg.vq, row, order), order);
                if rb.1 == usize::MAX || er < rb.0 {
                    rb = (er, row);
                }
            }
            if best.1 == usize::MAX || rb.0 < best.0 {
                best = rb;
            }
        }
        codes[ks] = best.1;
        let chosen = candidate(seg.vq, best.1, order);
        rq[base_coeff..base_coeff + order].copy_from_slice(&chosen[..order]);

        // Update for the next segment (steps 11-14, §B.4 step 6).
        if ks < 2 {
            let l = base_coeff + 1;
            for j in 0..order {
                let r = chosen[j];
                let r2 = mult_r(r, r);
                let b = (10 - l - j - 1) as isize;
                let vi = |v: &[i32; 2 * NP - 1], i: isize| v[(i + 9) as usize];
                let (pp, vp) = (pf, vf);
                let pnew = |i: usize| {
                    l_add(
                        l_add(
                            l_add(l_mpy_ls(vi(&vp, i as isize), r), pp[i]),
                            l_mpy_ls(pp[i], r2),
                        ),
                        l_mpy_ls(vi(&vp, -(i as isize)), r),
                    )
                };
                let p0 = pnew(0);
                let n = norm_s(extract_h(p0)) - 2;
                pf[0] = l_shl(p0, n);
                for i in 1..=b as usize {
                    pf[i] = l_shl(pnew(i), n);
                }
                for i in -b..=b {
                    let x = l_add(
                        l_shl(l_mpy_ls(pp[(i + 1).unsigned_abs()], r), 1),
                        l_add(l_mpy_ls(vi(&vp, -i - 1), r2), vi(&vp, i + 1)),
                    );
                    vf[(i + 9) as usize] = l_shl(x, n);
                }
            }
        }
        base_coeff += order;
    }
    ((codes[0] as u16, codes[1] as u16, codes[2] as u8), rq)
}

/// Stand-alone fixed-point front end (diagnostics / conformance).
#[doc(hidden)]
#[derive(Debug, Clone, Default)]
pub struct HrFrontEnd {
    fe: FrontEnd,
}

impl HrFrontEnd {
    pub fn new() -> Self {
        Self::default()
    }

    /// Filter one input frame into the buffer and analyse it.
    pub fn push_and_analyse(&mut self, samples: &[i16; HR_FRAME_SAMPLES]) -> FrontFrame {
        self.fe.push(samples);
        self.fe.analyse()
    }

    /// The current high-pass output buffer `s(0..=194)`.
    pub fn buffer(&self) -> &[i16; BUF] {
        &self.fe.buf
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The high-pass rejects DC and passes a mid-band tone at the
    /// announced 0,5 gain.
    #[test]
    fn highpass_dc_and_passband() {
        let mut hp = HighPassFx::default();
        let mut last = 0;
        for _ in 0..4000 {
            last = hp.process(8000);
        }
        // A small fixed-point residue remains (−38 dB re the 0,5-gain
        // passband level).
        assert!(last.abs() <= 64, "DC leak {last}");
        let mut hp = HighPassFx::default();
        let mut peak = 0i16;
        for n in 0..8000 {
            let x =
                (8000.0 * (2.0 * core::f64::consts::PI * 1000.0 * n as f64 / 8000.0).sin()) as i16;
            let y = hp.process(x);
            if n > 4000 {
                peak = peak.max(y.abs());
            }
        }
        assert!((3800..=4200).contains(&peak), "1 kHz peak {peak}");
    }

    /// Silence codes R0 = 0 with all-zero reflection coefficients;
    /// a loud frame codes a high R0.
    #[test]
    fn r0_extremes() {
        let mut fe = HrFrontEnd::new();
        let f = fe.push_and_analyse(&[0; HR_FRAME_SAMPLES]);
        assert_eq!(f.r0, 0);
        assert_eq!(f.r_flat, [0; NP]);
        let mut fe = HrFrontEnd::new();
        let mut loud = [0i16; HR_FRAME_SAMPLES];
        for (n, s) in loud.iter_mut().enumerate() {
            *s = if (n / 10) % 2 == 0 { 30000 } else { -30000 };
        }
        fe.push_and_analyse(&loud);
        let f = fe.push_and_analyse(&loud);
        assert!(f.r0 >= 28, "loud R0 {}", f.r0);
    }
}
