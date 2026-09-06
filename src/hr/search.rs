//! GSM 06.20 clauses 4.1.10 (VSELP code search) and 4.1.11
//! (multimode `{P0,GS}` gain vector quantization) — pure functions
//! over the weighted target `p(n)`, the weighted excitation vectors
//! and the staged codebooks.
//!
//! The code search evaluates every codeword directly from the
//! decorrelated filtered basis vectors (eqs. (119)–(122)); the
//! Gray-code update of eqs. (123)/(124) is an implementation
//! shortcut for the same maximisation of eq. (117). Complementary
//! codewords tie on eq. (117); the sign of `C_i` then picks the
//! member that yields a positive gain (clause 4.1.10.2).

use super::tables::GSP0_VQ;
use super::HR_SUBFRAME_SAMPLES;

/// Samples per subframe.
const NS: usize = HR_SUBFRAME_SAMPLES;

/// Clause 4.1.10.1 eqs. (110)–(112): decorrelate the filtered basis
/// vectors `q_m` against the weighted first excitation vector
/// `b"` (the long-term vector, or the first-codebook codevector in
/// MODE 0). A zero `b"` leaves the vectors untouched.
pub(super) fn decorrelate(q: &mut [[f64; NS]], b: &[f64; NS]) {
    let gamma: f64 = b.iter().map(|v| v * v).sum();
    if gamma <= 0.0 {
        return;
    }
    for qm in q.iter_mut() {
        let psi: f64 = b.iter().zip(qm.iter()).map(|(x, y)| x * y).sum();
        let s = psi / gamma;
        for (v, bb) in qm.iter_mut().zip(b.iter()) {
            *v -= s * bb;
        }
    }
}

/// Clause 4.1.10.2: the codeword `i` (over `M = q.len()` bits)
/// maximising `C_i²/G_i` (eq. (117)) for the target `p`, with the
/// complement chosen when `C_i < 0` so the optimal gain is positive.
/// Codeword bit `m` (LSB = bit 0) signs basis vector `q[m]`
/// (`θ_im = +1` when set) — the convention the decoder realises.
pub(super) fn code_search(q: &[[f64; NS]], p: &[f64; NS]) -> u16 {
    let m_bits = q.len();
    // Eq. (119)/(120) correlation terms.
    let mut r = vec![0f64; m_bits];
    let mut d = vec![vec![0f64; m_bits]; m_bits];
    for m in 0..m_bits {
        r[m] = q[m].iter().zip(p.iter()).map(|(a, b)| a * b).sum();
        for j in 0..=m {
            let v: f64 = q[m].iter().zip(q[j].iter()).map(|(a, b)| a * b).sum();
            d[m][j] = v;
            d[j][m] = v;
        }
    }
    let mut best = (f64::NEG_INFINITY, 0.0f64, 0u16);
    let mut theta = vec![1.0f64; m_bits];
    for code in 0..(1u32 << m_bits) {
        for (m, t) in theta.iter_mut().enumerate() {
            *t = if (code >> m) & 1 == 1 { 1.0 } else { -1.0 };
        }
        let mut c = 0.0;
        let mut g = 0.0;
        for m in 0..m_bits {
            c += theta[m] * r[m];
            let mut acc = 0.0;
            for j in 0..m_bits {
                acc += theta[j] * d[m][j];
            }
            g += theta[m] * acc;
        }
        if g <= 0.0 {
            continue;
        }
        // Eq. (126): C_i²·G_best > C_best²·G_i, kept in ratio form.
        let score = c * c / g;
        if score > best.0 {
            best = (score, c, code as u16);
        }
    }
    let (_, c, code) = best;
    if c < 0.0 {
        code ^ ((1u16 << m_bits) - 1)
    } else {
        code
    }
}

/// Inputs of the clause 4.1.11.1 gain quantizer for one subframe.
pub(super) struct GainInputs<'a> {
    /// Weighted target `p(n)`.
    pub p: &'a [f64; NS],
    /// Unweighted excitation vectors `c_0`, `c_1` (eq. (127)).
    pub c0: &'a [f64; NS],
    pub c1: &'a [f64; NS],
    /// Their weighted versions `c'_0`, `c'_1`.
    pub w0: &'a [f64; NS],
    pub w1: &'a [f64; NS],
    /// Eq. (132) `RS`.
    pub rs: f64,
}

/// Result of the gain search: the 5-bit code and the reconstructed
/// quantized gains of eqs. (145)/(146).
#[derive(Clone, Copy, Debug)]
pub(super) struct GainChoice {
    pub code: u8,
    pub beta: f64,
    pub gamma: f64,
}

/// Clause 4.1.11.1: search the MODE's 32-entry `{P0,GS}` codebook
/// for the vector minimising the weighted error of eq. (135) (the
/// constant `χ²R_pp` dropped), with the eq. (147) special case for
/// an all-zero first vector.
pub(super) fn gain_search(mode: u8, g: &GainInputs<'_>) -> GainChoice {
    let dot =
        |a: &[f64; NS], b: &[f64; NS]| -> f64 { a.iter().zip(b.iter()).map(|(x, y)| x * y).sum() };
    let rx0 = dot(g.c0, g.c0);
    let rx1 = dot(g.c1, g.c1);
    let rpc0 = dot(g.p, g.w0);
    let rpc1 = dot(g.p, g.w1);
    let rcc00 = dot(g.w0, g.w0);
    let rcc11 = dot(g.w1, g.w1);
    let rcc01 = dot(g.w0, g.w1);
    let rpp = dot(g.p, g.p);

    // Jointly optimal (unquantized) gains for the eq. (134) bias.
    let det = rcc00 * rcc11 - rcc01 * rcc01;
    let (b_opt, g_opt) = if det.abs() > 1e-300 * rcc00.max(rcc11).max(1.0) && det > 0.0 {
        (
            (rpc0 * rcc11 - rpc1 * rcc01) / det,
            (rpc1 * rcc00 - rpc0 * rcc01) / det,
        )
    } else {
        (
            if rcc00 > 0.0 { rpc0 / rcc00 } else { 0.0 },
            if rcc11 > 0.0 { rpc1 / rcc11 } else { 0.0 },
        )
    };
    let e_opt = b_opt * b_opt * rcc00 + g_opt * g_opt * rcc11 + 2.0 * b_opt * g_opt * rcc01;
    let chi = if e_opt > 0.0 {
        (rpp / e_opt).sqrt().clamp(1.0, 2f64.sqrt())
    } else {
        1.0
    };

    let table = &GSP0_VQ[(mode & 3) as usize];
    let zero_first = rx0 <= 0.0;
    let rx1_safe = if rx1 > 0.0 { rx1 } else { 1.0 };
    let rx0_safe = if rx0 > 0.0 { rx0 } else { 1.0 };
    // Eqs. (136)-(140).
    let a = 2.0 * chi * rpc0 * (g.rs / rx0_safe).sqrt();
    let b = 2.0 * chi * rpc1 * (g.rs / rx1_safe).sqrt();
    let c = 2.0 * rcc01 * g.rs / (rx0_safe * rx1_safe).sqrt();
    let d = g.rs * rcc00 / rx0_safe;
    let e = g.rs * rcc11 / rx1_safe;

    let mut best = (f64::INFINITY, 0usize);
    for (idx, row) in table.iter().enumerate() {
        // √(GS·P0) and √(GS·(1−P0)) — the first two codebook
        // components (Q14, see `decode`); the remaining three are
        // their products.
        let sp = row[0] as f64 / 16384.0;
        let sq = row[1] as f64 / 16384.0;
        let err = if zero_first {
            -b * sq + e * sq * sq
        } else {
            -a * sp - b * sq + c * sp * sq + d * sp * sp + e * sq * sq
        };
        if err < best.0 {
            best = (err, idx);
        }
    }
    let row = &table[best.1];
    let sp = row[0] as f64 / 16384.0;
    let sq = row[1] as f64 / 16384.0;
    GainChoice {
        code: best.1 as u8,
        beta: if zero_first {
            0.0
        } else {
            (g.rs / rx0).sqrt() * sp
        },
        gamma: if rx1 > 0.0 {
            (g.rs / rx1).sqrt() * sq
        } else {
            0.0
        },
    }
}

/// Reconstruct the eq. (145)/(146) gains for a given code.
pub(super) fn gain_from_code(
    mode: u8,
    code: u8,
    c0: &[f64; NS],
    c1: &[f64; NS],
    rs: f64,
) -> GainChoice {
    let rx0: f64 = c0.iter().map(|v| v * v).sum();
    let rx1: f64 = c1.iter().map(|v| v * v).sum();
    let row = &GSP0_VQ[(mode & 3) as usize][(code & 31) as usize];
    let sp = row[0] as f64 / 16384.0;
    let sq = row[1] as f64 / 16384.0;
    GainChoice {
        code: code & 31,
        beta: if rx0 > 0.0 {
            (rs / rx0).sqrt() * sp
        } else {
            0.0
        },
        gamma: if rx1 > 0.0 {
            (rs / rx1).sqrt() * sq
        } else {
            0.0
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn basis(seed: u32, m: usize) -> Vec<[f64; NS]> {
        let mut state = seed;
        (0..m)
            .map(|_| {
                let mut v = [0f64; NS];
                for x in v.iter_mut() {
                    state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                    *x = (state >> 8) as f64 / (1u32 << 24) as f64 - 0.5;
                }
                v
            })
            .collect()
    }

    /// Searching for a target that IS a codevector recovers its
    /// codeword (or the complement with the sign fixed to positive
    /// gain, which is the same codeword by construction).
    #[test]
    fn code_search_recovers_planted_codeword() {
        let q = basis(7, 9);
        for code in [0u16, 0x1FF, 0x155, 0x0A3] {
            let mut p = [0f64; NS];
            for (m, qm) in q.iter().enumerate() {
                let s = if (code >> m) & 1 == 1 { 1.0 } else { -1.0 };
                for n in 0..NS {
                    p[n] += s * qm[n];
                }
            }
            assert_eq!(code_search(&q, &p), code);
        }
    }

    /// The chosen codeword always has a non-negative correlation
    /// with the target (positive optimal gain).
    #[test]
    fn code_search_gain_is_positive() {
        let q = basis(3, 7);
        let mut p = [0f64; NS];
        for (n, v) in p.iter_mut().enumerate() {
            *v = ((n * 7) % 11) as f64 - 5.0;
        }
        let code = code_search(&q, &p);
        let mut c = 0.0;
        for (m, qm) in q.iter().enumerate() {
            let s = if (code >> m) & 1 == 1 { 1.0 } else { -1.0 };
            for n in 0..NS {
                c += s * qm[n] * p[n];
            }
        }
        assert!(c >= 0.0);
    }

    /// Decorrelation makes every vector orthogonal to `b`.
    #[test]
    fn decorrelate_orthogonalises() {
        let mut q = basis(11, 5);
        let b = basis(5, 1)[0];
        decorrelate(&mut q, &b);
        for qm in &q {
            let dot: f64 = qm.iter().zip(b.iter()).map(|(x, y)| x * y).sum();
            assert!(dot.abs() < 1e-9);
        }
    }

    /// Gain search: with a target equal to the weighted first
    /// vector scaled, the chosen entry puts (nearly) all the energy
    /// on P0 → β ≈ scale, γ small.
    #[test]
    fn gain_search_prefers_matching_vector() {
        let vs = basis(21, 2);
        let (c0, c1) = (vs[0], vs[1]);
        let mut p = [0f64; NS];
        for n in 0..NS {
            p[n] = 3.0 * c0[n];
        }
        let rx0: f64 = c0.iter().map(|v| v * v).sum();
        // RS = the actual excitation energy makes GS ≈ 1.
        let rs = 9.0 * rx0;
        let g = gain_search(
            3,
            &GainInputs {
                p: &p,
                c0: &c0,
                c1: &c1,
                w0: &c0,
                w1: &c1,
                rs,
            },
        );
        assert!((g.beta - 3.0).abs() < 0.6, "{g:?}");
        assert!(g.gamma.abs() < 1.0, "{g:?}");
    }

    /// Eq. (147): a zero first vector forces β = 0 without panics.
    #[test]
    fn gain_search_zero_first_vector() {
        let vs = basis(31, 1);
        let zero = [0f64; NS];
        let p = vs[0];
        let g = gain_search(
            1,
            &GainInputs {
                p: &p,
                c0: &zero,
                c1: &vs[0],
                w0: &zero,
                w1: &vs[0],
                rs: 1.0,
            },
        );
        assert_eq!(g.beta, 0.0);
        assert!(g.gamma > 0.0);
    }
}
