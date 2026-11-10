//! 16/32-bit saturating fixed-point operators of the GSM 06.20
//! half-rate arithmetic (EN 300 969 clause 5.1 places the bit-exact
//! arithmetic in GSM 06.06; the operator semantics used here are the
//! ones written out in `docs/audio/gsm/halfrate-traces/
//! hr-fixed-point-schedules.md` §0.1).
//!
//! "Word" = `i16`, "long" = `i32`. Every long operation saturates to
//! `[−2³¹, 2³¹−1]` and every word operation to `[−32768, 32767]`.
//!
//! These are deliberately separate from the full-rate `crate::arith`
//! helpers: a few operators (`divide_s`, the shift edge cases, the
//! 32×16 / 32×32 multiplies) differ in their corner-case behaviour.

#![allow(dead_code)]

/// Saturate a 64-bit intermediate to a long.
#[inline]
pub(crate) fn sat32(x: i64) -> i32 {
    x.clamp(i32::MIN as i64, i32::MAX as i64) as i32
}

/// Saturate a 32-bit intermediate to a word.
#[inline]
pub(crate) fn sat16(x: i32) -> i16 {
    x.clamp(i16::MIN as i32, i16::MAX as i32) as i16
}

#[inline]
pub(crate) fn add(a: i16, b: i16) -> i16 {
    a.saturating_add(b)
}

#[inline]
pub(crate) fn sub(a: i16, b: i16) -> i16 {
    a.saturating_sub(b)
}

#[inline]
pub(crate) fn negate(a: i16) -> i16 {
    if a == i16::MIN { i16::MAX } else { -a }
}

#[inline]
pub(crate) fn abs_s(a: i16) -> i16 {
    if a == i16::MIN { i16::MAX } else { a.abs() }
}

#[inline]
pub(crate) fn l_negate(a: i32) -> i32 {
    if a == i32::MIN { i32::MAX } else { -a }
}

#[inline]
pub(crate) fn l_abs(a: i32) -> i32 {
    if a == i32::MIN { i32::MAX } else { a.abs() }
}

#[inline]
pub(crate) fn l_add(a: i32, b: i32) -> i32 {
    a.saturating_add(b)
}

#[inline]
pub(crate) fn l_sub(a: i32, b: i32) -> i32 {
    a.saturating_sub(b)
}

/// `L_mult(a,b)` = 2·a·b, with (−32768)·(−32768) → 0x7fffffff.
#[inline]
pub(crate) fn l_mult(a: i16, b: i16) -> i32 {
    if a == i16::MIN && b == i16::MIN {
        i32::MAX
    } else {
        (a as i32 * b as i32) << 1
    }
}

#[inline]
pub(crate) fn l_mac(l: i32, a: i16, b: i16) -> i32 {
    l_add(l, l_mult(a, b))
}

#[inline]
pub(crate) fn l_msu(l: i32, a: i16, b: i16) -> i32 {
    l_sub(l, l_mult(a, b))
}

/// High 16 bits (floor division by 65536).
#[inline]
pub(crate) fn extract_h(l: i32) -> i16 {
    (l >> 16) as i16
}

/// Low 16 bits as a signed word.
#[inline]
pub(crate) fn extract_l(l: i32) -> i16 {
    l as i16
}

#[inline]
pub(crate) fn l_deposit_h(a: i16) -> i32 {
    (a as i32) << 16
}

#[inline]
pub(crate) fn l_deposit_l(a: i16) -> i32 {
    a as i32
}

/// `round(L)` = `extract_h(L_add(L, 0x8000))`.
#[inline]
pub(crate) fn round(l: i32) -> i16 {
    extract_h(l_add(l, 0x8000))
}

#[inline]
pub(crate) fn mult(a: i16, b: i16) -> i16 {
    extract_h(l_mult(a, b))
}

#[inline]
pub(crate) fn mult_r(a: i16, b: i16) -> i16 {
    round(l_mult(a, b))
}

#[inline]
pub(crate) fn mac_r(l: i32, a: i16, b: i16) -> i16 {
    round(l_mac(l, a, b))
}

#[inline]
pub(crate) fn msu_r(l: i32, a: i16, b: i16) -> i16 {
    round(l_msu(l, a, b))
}

/// Word left shift (saturating); a negative count shifts right.
#[inline]
pub(crate) fn shl(a: i16, n: i16) -> i16 {
    if n < 0 {
        return shr(a, n.saturating_neg());
    }
    if n >= 16 {
        return if a > 0 {
            i16::MAX
        } else if a < 0 {
            i16::MIN
        } else {
            0
        };
    }
    sat16((a as i32) << n)
}

/// Word arithmetic right shift; a negative count shifts left.
#[inline]
pub(crate) fn shr(a: i16, n: i16) -> i16 {
    if n < 0 {
        return shl(a, n.saturating_neg());
    }
    if n >= 15 {
        return if a < 0 { -1 } else { 0 };
    }
    a >> n
}

/// Long left shift (saturating); a negative count shifts right.
#[inline]
pub(crate) fn l_shl(l: i32, n: i16) -> i32 {
    if n < 0 {
        return l_shr(l, n.saturating_neg());
    }
    if n >= 32 {
        return if l > 0 {
            i32::MAX
        } else if l < 0 {
            i32::MIN
        } else {
            0
        };
    }
    sat32((l as i64) << n)
}

/// Long arithmetic right shift; a negative count shifts left.
#[inline]
pub(crate) fn l_shr(l: i32, n: i16) -> i32 {
    if n < 0 {
        return l_shl(l, n.saturating_neg());
    }
    if n >= 31 {
        return if l < 0 { -1 } else { 0 };
    }
    l >> n
}

/// `shift_r(x, n)`: for `n < 0` a rounding right shift by `−n`
/// (⌊x/2^k + ½⌋); for `n ≥ 0` a plain (saturating) left shift.
#[inline]
pub(crate) fn shift_r(a: i16, n: i16) -> i16 {
    if n >= 0 {
        return shl(a, n);
    }
    let k = n.saturating_neg();
    if k >= 15 {
        return 0;
    }
    let r = shr(a, k);
    let bit = (a >> (k - 1)) & 1;
    add(r, bit)
}

/// Number of left shifts bringing a non-zero long into
/// `[2³⁰, 2³¹)` or `[−2³¹, −2³⁰)`; 0 for 0.
#[inline]
pub(crate) fn norm_l(l: i32) -> i16 {
    if l == 0 {
        return 0;
    }
    if l == -1 {
        return 31;
    }
    let v = if l < 0 { !l } else { l };
    (v.leading_zeros() as i16) - 1
}

/// `norm_l(a·65536)`.
#[inline]
pub(crate) fn norm_s(a: i16) -> i16 {
    norm_l(l_deposit_h(a))
}

/// `divide_s(a,b)` = ⌊32768·a/b⌋ for `0 ≤ a < b`; `0x7fff` when
/// `a == b`; **0** for any invalid input (`a < 0`, `b < 0`, `a > b`).
#[inline]
pub(crate) fn divide_s(a: i16, b: i16) -> i16 {
    if a < 0 || b < 0 || a > b {
        return 0;
    }
    if a == b {
        return i16::MAX;
    }
    // b > a >= 0 so b > 0.
    (((a as i32) << 15) / b as i32) as i16
}

/// 32×16 multiply: ≈ L·w/32768 (schedules §0.1 `L_mpy_ls`).
#[inline]
pub(crate) fn l_mpy_ls(l: i32, w: i16) -> i32 {
    let lo = (shr(extract_l(l), 1)) & 0x7fff;
    l_mac(l_shr(l_mult(w, lo), 15), w, extract_h(l))
}

/// 32×32 multiply: ≈ L1·L2/2³¹ (schedules §0.1 `L_mpy_ll`).
#[inline]
pub(crate) fn l_mpy_ll(l1: i32, l2: i32) -> i32 {
    let lo1 = shr(extract_l(l1), 1) & 0x7fff;
    let lo2 = shr(extract_l(l2), 1) & 0x7fff;
    let h1 = extract_h(l1);
    let h2 = extract_h(l2);
    let low = l_shr(l_mult(lo1, lo2), 16);
    let mut mid = l_add(
        l_add(l_shr(l_mult(h1, lo2), 1), low),
        l_shr(l_mult(h2, lo1), 1),
    );
    mid = l_shr(mid, 14);
    l_mac(mid, h1, h2)
}

/// Square root of a normalised long in `[2³⁰, 2³¹)` by the six-term
/// expansion of schedules §0.1.
pub(crate) fn sqroot(y: i32) -> i16 {
    let t = l_sub(l_shr(y, 1), 0x4000_0000);
    let h = extract_h(t);
    let mut acc = l_sub(t, i32::MIN);
    let q2 = l_msu(0, h, h);
    let h2 = extract_h(q2);
    acc = l_add(acc, l_shr(q2, 1));
    let q3 = l_msu(0, h, h2);
    let h3 = extract_h(q3);
    acc = l_add(acc, l_shr(q3, 1));
    let h4 = round(l_mult(h, h3));
    let h5 = round(l_msu(0, h2, h3));
    acc = l_add(l_msu(0, 0x5000, h4), acc);
    mac_r(acc, 0x7000, h5)
}

/// Normalise a long: returns `(L << n, n)` with `n = norm_l(L)`.
#[inline]
pub(crate) fn normalise(l: i32) -> (i32, i16) {
    let n = norm_l(l);
    (l_shl(l, n), n)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn norm_edges() {
        assert_eq!(norm_l(0), 0);
        assert_eq!(norm_l(1), 30);
        assert_eq!(norm_l(-1), 31);
        assert_eq!(norm_l(0x4000_0000), 0);
        assert_eq!(norm_l(i32::MIN), 0);
        assert_eq!(norm_l(-0x4000_0000), 1);
        assert_eq!(norm_l(-0x4000_0001), 0);
        assert_eq!(norm_s(1), 14);
        assert_eq!(norm_s(-1), 15);
        assert_eq!(norm_s(0), 0);
    }

    #[test]
    fn divide_s_edges() {
        assert_eq!(divide_s(1, 2), 16384);
        assert_eq!(divide_s(5, 5), 0x7fff);
        assert_eq!(divide_s(6, 5), 0);
        assert_eq!(divide_s(-1, 5), 0);
    }

    #[test]
    fn sqroot_close() {
        for &v in &[0x4000_0000i32, 0x5000_0000, 0x6000_0000, 0x7fff_ffff] {
            let s = sqroot(v) as f64 / 32768.0;
            let exact = (v as f64 / 2147483648.0).sqrt();
            assert!((s - exact).abs() < 2e-3, "{v:#x}: {s} vs {exact}");
        }
    }

    #[test]
    fn shift_r_rounds() {
        assert_eq!(shift_r(5, -1), 3);
        assert_eq!(shift_r(4, -1), 2);
        assert_eq!(shift_r(-5, -1), -2);
    }

    #[test]
    fn mpy_close() {
        let l = 0x3456_789a;
        let w = 0x6000i16;
        let exact = (l as f64) * (w as f64) / 32768.0;
        assert!((l_mpy_ls(l, w) as f64 - exact).abs() < 4.0);
        let l2 = 0x2345_6789;
        let exact = (l as f64) * (l2 as f64) / 2147483648.0;
        assert!((l_mpy_ll(l, l2) as f64 - exact).abs() < 8.0);
    }
}
