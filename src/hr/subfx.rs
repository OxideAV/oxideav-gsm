//! GSM 06.20 encoder subframe stages in the clause 5.1 fixed-point
//! arithmetic: the clause 4.1.8.5 closed-loop lag search, the
//! clause 4.1.9 harmonic noise weighting C(z), the H(z) zero-state
//! responses, the clause 4.1.10.1 decorrelation, the clause 4.1.10.2
//! VSELP code search and the clause 4.1.11.1 `{P0,GS}` search, as
//! the operator rules of
//! `docs/audio/gsm/halfrate-traces/hr-fixed-point-schedules.md`
//! §C.1 rules 21–24 and §C.2.

#![allow(clippy::needless_range_loop)]

use super::HR_SUBFRAME_SAMPLES as NS;
use super::decode::{adaptive_codebook_words, energy_plain, rho_from_energy};
use super::fx::*;
use super::tables::{GSP0_VQ, INTERP_FILTER_6, LAG_TABLE};

const NP: usize = 10;

/// Long-term / weighted histories (words), as in the decoder.
pub(super) const HIST: usize = 147;

/// H(z) zero-state response of `x` with the Q11 direct-form words
/// `a` (schedules §C.2 "H(z) zero-state response").
pub(super) fn hz_zsr(a: &[i16; NP], x: &[i16; NS]) -> [i16; NS] {
    let mut mem = [0i16; NP];
    hz_filter(a, x, &mut mem)
}

/// H(z) with memory (the synthesis-filter schedule): `mem` holds
/// `y(n−1)..y(n−10)` and is updated.
pub(super) fn hz_filter(a: &[i16; NP], x: &[i16; NS], mem: &mut [i16; NP]) -> [i16; NS] {
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

/// H(z) zero-input response from `mem`.
pub(super) fn hz_zir(a: &[i16; NP], mem: &[i16; NP]) -> [i16; NS] {
    let mut m = *mem;
    hz_filter(a, &[0; NS], &mut m)
}

/// The six C(z) tap words for a pitch lag (1/6 units) and the stored
/// λ_hnw/2 word.
fn cz_taps(lpitch6: i32, hnw: i16) -> [i16; 6] {
    let j = (lpitch6 % 6) as usize;
    let mut c = [0i16; 6];
    for i in 0..6 {
        c[i] = mult_r(negate(INTERP_FILTER_6[i][j]), hnw);
    }
    c
}

/// C(z) (eqs. (104)–(107)) over one subframe. `hist` supplies the
/// samples before the subframe (newest last); `None` = zero state.
pub(super) fn cz(x: &[i16; NS], hist: Option<&[i16]>, lpitch6: i32, hnw: i16) -> [i16; NS] {
    let ip = (lpitch6 / 6) as isize;
    if hist.is_none() && ip >= 42 {
        return *x;
    }
    let c = cz_taps(lpitch6, hnw);
    let mut y = [0i16; NS];
    for n in 0..NS {
        let mut l = l_mac(16384, x[n], 0x4000);
        let mut any = false;
        for i in (0..6).rev() {
            let m = n as isize - ip - 3 + i as isize;
            let v = if m >= 0 {
                x[m as usize]
            } else if let Some(h) = hist {
                let idx = h.len() as isize + m;
                if idx < 0 {
                    continue;
                }
                h[idx as usize]
            } else {
                continue;
            };
            any = true;
            l = l_mac(l, v, c[i]);
        }
        y[n] = if any { extract_h(l_shl(l, 1)) } else { x[n] };
    }
    y
}

/// The target / LTP down-shift `{0,1,2}` from a bound energy word.
fn shift_from_bound(w: i16) -> i16 {
    if w <= 0x7ff {
        0
    } else if w <= 0x1fff {
        1
    } else {
        2
    }
}

/// Target shift `s` (schedules §C.2).
pub(super) fn target_shift(p: &[i16; NS]) -> i16 {
    let mut l = 0x001d_ff4c_i32;
    for &v in p.iter() {
        let s = shr(v, 2);
        l = l_mac(l, s, s);
    }
    shift_from_bound(round(l))
}

#[inline]
fn shr_vec(x: &[i16; NS], s: i16) -> [i16; NS] {
    x.map(|v| shr(v, s))
}

/// Closed-loop lag search result.
pub(super) struct ClosedLoop {
    pub level: usize,
    /// b_L (unfiltered), shifted by `t`.
    pub b: [i16; NS],
    pub t: i16,
}

/// Rules 21–24: the closed-loop search over the trajectory level ±1.
/// `p` is the pre-C(z) target already shifted by `s`.
pub(super) fn closed_loop(
    ltp: &[i16; HIST],
    a: &[i16; NP],
    centre: usize,
    p: &[i16; NS],
    only_centre: bool,
) -> ClosedLoop {
    let lo = if only_centre {
        centre
    } else {
        centre.saturating_sub(1)
    };
    let hi = if only_centre {
        centre
    } else {
        (centre + 1).min(255)
    };
    let mut cand: Vec<(usize, [i16; NS], [i16; NS])> = Vec::with_capacity(3);
    for lv in lo..=hi {
        let b = adaptive_codebook_words(ltp, LAG_TABLE[lv]);
        let bf = hz_zsr(a, &b);
        cand.push((lv, b, bf));
    }
    // Rule 23: common scaling.
    let mut l = 0x001d_ff4c_i32;
    for &v in cand[0].2.iter() {
        let s = shr(v, 2);
        l = l_mac(l, s, s);
    }
    for c in cand.iter().skip(1) {
        let s = shr(c.2[0], 2);
        l = l_mac(l, s, s);
    }
    let t = shift_from_bound(round(l));
    for c in cand.iter_mut() {
        c.1 = shr_vec(&c.1, t);
        c.2 = shr_vec(&c.2, t);
    }
    // Rule 24.
    let mut cs = Vec::with_capacity(3);
    let mut gs = Vec::with_capacity(3);
    let mut or_c = 0i32;
    let mut or_g = 0i32;
    for c in cand.iter() {
        let mut lc = 0i32;
        let mut lg = 0i32;
        for n in 0..NS {
            lg = l_mac(lg, c.2[n], c.2[n]);
            lc = l_mac(lc, c.2[n], p[n]);
        }
        or_c |= l_abs(lc);
        or_g |= lg;
        cs.push(lc);
        gs.push(lg);
    }
    let nc = norm_l(or_c);
    let ng = norm_l(or_g);
    let mut sel = 0usize;
    let mut best: (i16, i16) = (0, 0);
    for i in 0..cand.len() {
        let cw = round(l_shl(cs[i], nc));
        let gw = round(l_shl(gs[i], ng));
        let mut cc = mult_r(cw, cw);
        if cw < 0 {
            cc = negate(cc);
        }
        if i == 0 {
            best = (cc, gw);
            continue;
        }
        if gw > 0 && l_msu(l_mult(cc, best.1), gw, best.0) > 0 {
            best = (cc, gw);
            sel = i;
        }
    }
    let (level, b, _) = cand.swap_remove(sel);
    ClosedLoop { level, b, t }
}

/// Decorrelation (schedules §C.2 4.1.10.1) of `q` against `b`.
pub(super) fn decorrelate_fx(q: &mut [[i16; NS]], b: &[i16; NS]) {
    let mut l = 0i32;
    for &v in b.iter() {
        l = l_mac(l, v, v);
    }
    let e = norm_l(l);
    let gw = extract_h(l_shl(l, e));
    if gw == 0 {
        return;
    }
    for qm in q.iter_mut() {
        let mut lc = 0i32;
        for n in 0..NS {
            lc = l_mac(lc, b[n], qm[n]);
        }
        let c = norm_l(lc);
        lc = l_shl(lc, c);
        let mut r = divide_s(round(l_shr(l_abs(lc), 1)), gw);
        if lc > 0 {
            r = negate(r);
        }
        let mut k = c - 1 - e;
        let n = norm_s(r);
        r = shl(r, n);
        k += n;
        let sigma = if k > 0 {
            r = shift_r(r, -k);
            0
        } else {
            -k
        };
        for n in 0..NS {
            let base = l_msu(0x8000, qm[n], i16::MIN);
            qm[n] = extract_h(l_mac(base, r, shl(b[n], sigma)));
        }
    }
}

/// Σθ_m·x_m (bit m set → +1), the codevector construction rule.
pub(super) fn combine(x: &[[i16; NS]], code: u16) -> [i16; NS] {
    let mut u = [0i16; NS];
    for n in 0..NS {
        let mut l = 0i32;
        for (m, v) in x.iter().enumerate() {
            let s: i16 = if (code >> m) & 1 == 1 {
                0x4000
            } else {
                -0x4000
            };
            l = l_mac(l, s, v[n]);
        }
        u[n] = extract_h(l_shl(l, 1));
    }
    u
}

/// Fast VSELP code search (schedules §C.2 4.1.10.2).
pub(super) fn code_search_fx(q: &[[i16; NS]], p: &[i16; NS]) -> u16 {
    let mb = q.len();
    let mut rr = vec![0i32; mb];
    let mut sum_abs = 0i32;
    for m in 0..mb {
        let mut l = 0i32;
        for n in 0..NS {
            l = l_mac(l, q[m][n], p[n]);
        }
        rr[m] = l;
        sum_abs = l_add(sum_abs, l_abs(l));
    }
    let s1 = norm_l(sum_abs) - 1;
    let rh: Vec<i32> = rr.iter().map(|&r| l_shl(r, s1)).collect();
    let mut lc0 = 0i32;
    for &r in rh.iter() {
        lc0 = l_sub(lc0, r);
    }
    let c0 = extract_h(lc0);
    let rt: Vec<i16> = rh.iter().map(|&r| round(l_shl(r, 1))).collect();

    let mut e = 0i32;
    for m in 0..mb {
        for n in 0..NS {
            e = l_mac(e, q[m][n], q[m][n]);
        }
    }
    let s2 = norm_l(e) - 4;
    let mut gacc = l_shl(e, s2);
    let mut dt = vec![vec![0i16; mb]; mb];
    for m in 0..mb {
        for j in m + 1..mb {
            let mut d = 0i32;
            for n in 0..NS {
                d = l_mac(d, q[m][n], q[j][n]);
            }
            let dd = l_shl(l_shl(d, s2), 1);
            gacc = l_add(dd, gacc);
            let w = round(l_shl(dd, 1));
            dt[m][j] = w;
            dt[j][m] = w;
        }
    }
    let g0 = extract_h(gacc);

    let mut theta = vec![false; mb]; // false = −1
    let mut lc = l_deposit_h(c0);
    let mut lg = l_deposit_h(g0);
    let mut ccmax = mult_r(c0, c0);
    let mut gmax = g0;
    let mut best = 0usize;
    let top = mb - 1;
    for k in 1usize..(1 << (mb - 1)) {
        let v = k.trailing_zeros() as usize;
        theta[v] = !theta[v];
        let pos = theta[v];
        lc = if pos {
            l_add(lc, l_deposit_h(rt[v]))
        } else {
            l_sub(lc, l_deposit_h(rt[v]))
        };
        for step in 1..top {
            let j = (v + step) % top;
            let d2 = shl(dt[v][j], 1);
            let a = if pos { d2 } else { negate(d2) };
            let b: i16 = if theta[j] { 0x4000 } else { -0x4000 };
            lg = l_mac(lg, a, b);
        }
        let dtop = l_deposit_h(dt[v][top]);
        lg = if pos {
            l_sub(lg, dtop)
        } else {
            l_add(lg, dtop)
        };
        let cw = extract_h(lc);
        let cc = mult_r(cw, cw);
        let g = extract_h(lg);
        if l_msu(l_mult(g, ccmax), gmax, cc) < 0 {
            ccmax = cc;
            gmax = g;
            best = k;
        }
    }
    let code = (best ^ (best >> 1)) as u16;
    let mut l = 0i32;
    for m in 0..mb {
        let x = l_shl(rh[m], 1);
        l = if (code >> m) & 1 == 1 {
            l_add(l, x)
        } else {
            l_sub(l, x)
        };
    }
    if l < 0 {
        code ^ ((1u16 << mb) - 1)
    } else {
        code
    }
}

/// Normalised correlation pair (schedules §C.2 conventions).
fn ncorr(x: &[i16; NS], y: &[i16; NS]) -> (i16, i16) {
    let mut l = 0i32;
    for n in 0..NS {
        l = l_mac(l, x[n], y[n]);
    }
    if l == 0 {
        return (0, 0);
    }
    let n = norm_l(l);
    (round(l_shl(l, n)), n)
}

/// Normalised product: mantissa and the added normalisation count.
#[inline]
fn nprod(m1: i16, m2: i16) -> (i16, i16) {
    let l = l_mult(m1, m2);
    let k = norm_s(extract_h(l));
    (round(l_shl(l, k)), k)
}

/// A two-step 3-factor product `(a×b)×c` with every shift count.
fn prod3(a: (i16, i16), b: (i16, i16), c: (i16, i16)) -> (i16, i16) {
    let (m1, k1) = nprod(a.0, b.0);
    let (m2, k2) = nprod(m1, c.0);
    (m2, a.1 + b.1 + c.1 + k1 + k2)
}

/// Align `(man, sh)` words to the minimum shift.
fn align(v: &[(i16, i16)]) -> Vec<i16> {
    let min = v.iter().map(|x| x.1).min().unwrap_or(0);
    v.iter()
        .map(|&(m, s)| round(l_shr(l_deposit_h(m), s - min)))
        .collect()
}

/// Inputs to the GSP0 search.
pub(super) struct GspInputs<'a> {
    pub p: &'a [i16; NS],
    pub c0: &'a [i16; NS],
    pub c1: &'a [i16; NS],
    pub w0: &'a [i16; NS],
    pub w1: &'a [i16; NS],
    /// The target shift `s`.
    pub s: i16,
    pub sqrt_rs: (i16, i16),
}

/// ρ_k of an unweighted vector (plain energy).
pub(super) fn rho_plain(x: &[i16; NS], rs: (i16, i16)) -> (i16, i16) {
    let (l, n) = energy_plain(x);
    rho_from_energy(l, n, rs)
}

/// Clause 4.1.11.1 GSP0 search (schedules §C.2).
pub(super) fn gsp0_search(mode: u8, g: &GspInputs<'_>) -> u8 {
    let rpc0 = ncorr(g.p, g.w0);
    let rpc1 = ncorr(g.p, g.w1);
    let rcc01 = ncorr(g.w0, g.w1);
    let rcc00 = ncorr(g.w0, g.w0);
    let rcc11 = ncorr(g.w1, g.w1);
    let rpp = ncorr(g.p, g.p);

    // χ (eq. (134)).
    let mut t3 = prod3(rpc0, rpc1, rcc01);
    t3.1 -= 1;
    let terms = [
        prod3(rcc00, rpp, rcc11),
        prod3(rcc01, rcc01, rpp),
        prod3(rpc1, rpc1, rcc00),
        t3,
        prod3(rpc0, rcc11, rpc0),
    ];
    let w = align(&terms);
    let num = sub(w[0], w[1]);
    let den = add(sub(w[2], w[3]), w[4]);
    let chi: i16 = if num <= 0 || den <= 0 || sub(num, den) <= 0 {
        0x4000
    } else {
        let s = norm_s(num);
        let num = shr(shl(num, s), 1);
        let mut x = s - 1;
        let s2 = norm_s(den);
        let den = shl(den, s2);
        x -= s2;
        let q = divide_s(num, den);
        let s3 = norm_s(q);
        x += s3;
        let r = sqroot(l_shl(l_deposit_h(q), s3));
        let l = if x & 1 != 0 {
            x -= 1;
            l_mult(0x5a82, r)
        } else {
            l_deposit_h(r)
        };
        x >>= 1;
        let s4 = norm_s(extract_h(l));
        x += s4;
        let mut cw = round(l_shl(l, s4));
        // χ ≥ 2 (x < −1) and χ above √2 both cap at √2.
        if x < -1 || (x == -1 && cw > 0x5a82) {
            cw = 0x5a82;
        }
        cw
    };
    let apply_chi = |v: (i16, i16)| -> (i16, i16) {
        let l = l_mult(chi, v.0);
        let k = norm_s(extract_h(l));
        (round(l_shl(l, k)), v.1 + k - 1)
    };
    let xr0 = apply_chi(rpc0);
    let xr1 = apply_chi(rpc1);

    let rho0 = rho_plain(g.c0, g.sqrt_rs);
    let rho1 = rho_plain(g.c1, g.sqrt_rs);
    let prod2 = |a: (i16, i16), b: (i16, i16)| -> (i16, i16) {
        let (m, k) = nprod(a.0, b.0);
        (m, a.1 + b.1 + k)
    };
    let ta = prod2(xr0, rho0);
    let tb = prod2(xr1, rho1);
    let mut tc = prod3(rcc01, rho0, rho1);
    tc.1 += g.s;
    let mut td = prod3(rcc00, rho0, rho0);
    td.1 += g.s;
    let mut te = prod3(rcc11, rho1, rho1);
    te.1 += g.s;
    let w = align(&[ta, tb, tc, td, te]);

    let table = &GSP0_VQ[(mode & 3) as usize];
    let mut best = i32::MIN;
    let mut idx = 0usize;
    for (i, row) in table.iter().enumerate() {
        let mut v = l_mult(row[0], w[0]);
        for k in 1..5 {
            v = l_mac(v, row[k], w[k]);
        }
        if l_sub(best, v) < 0 {
            best = v;
            idx = i;
        }
    }
    idx as u8
}

#[cfg(test)]
mod tests {
    use super::*;

    fn basis(seed: u32, m: usize) -> Vec<[i16; NS]> {
        let mut state = seed;
        (0..m)
            .map(|_| {
                let mut v = [0i16; NS];
                for x in v.iter_mut() {
                    state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                    *x = ((state >> 16) as i16) >> 5;
                }
                v
            })
            .collect()
    }

    /// A target that IS a codevector (top bit clear or set) is
    /// recovered, with the complement rule fixing the sign.
    #[test]
    fn code_search_recovers_planted_codeword() {
        let q = basis(7, 9);
        for code in [0u16, 0x1FF, 0x155, 0x0A3] {
            let p = combine(&q, code).map(|v| shr(v, 2));
            assert_eq!(code_search_fx(&q, &p), code, "code {code:#x}");
        }
    }

    /// Decorrelation leaves every vector (nearly) orthogonal to `b`.
    #[test]
    fn decorrelate_orthogonalises() {
        let mut q = basis(11, 5);
        let b = basis(5, 1)[0];
        decorrelate_fx(&mut q, &b);
        let eb: f64 = b.iter().map(|&v| v as f64 * v as f64).sum();
        for qm in &q {
            let dot: f64 = qm
                .iter()
                .zip(b.iter())
                .map(|(&x, &y)| x as f64 * y as f64)
                .sum();
            let eq: f64 = qm.iter().map(|&v| v as f64 * v as f64).sum();
            assert!(dot.abs() / (eb * eq).sqrt() < 0.01, "{dot}");
        }
    }

    /// The zero-state C(z) leaves a long-lag vector untouched and
    /// H(z) with zero coefficients is the identity.
    #[test]
    fn filters_identity_cases() {
        let x = basis(3, 1)[0];
        assert_eq!(cz(&x, None, 6 * 50, 10000), x);
        assert_eq!(hz_zsr(&[0; NP], &x), x);
        assert_eq!(hz_zir(&[0; NP], &[5; NP]), [0; NS]);
    }

    /// GSP0: a target equal to the weighted LTP vector prefers a
    /// high-P0 entry over a pure-codebook one.
    #[test]
    fn gsp0_prefers_matching_vector() {
        let vs = basis(21, 2);
        let (c0, c1) = (vs[0], vs[1]);
        let g = gsp0_search(
            3,
            &GspInputs {
                p: &c0,
                c0: &c0,
                c1: &c1,
                w0: &c0,
                w1: &c1,
                s: 0,
                sqrt_rs: (20000, 4),
            },
        );
        let row = &GSP0_VQ[3][g as usize];
        assert!(row[0] > row[1], "entry {g}: {row:?}");
    }
}
