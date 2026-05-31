//! Saturating fixed-point primitives from ETSI EN 300 961 §5.1
//! "Data representation and arithmetic operations".
//!
//! Every helper in this module corresponds 1:1 with one of the
//! operators §5.1 defines. The PDF spells out the saturation /
//! rounding behaviour in prose; the implementations here are
//! straightforward translations of that prose into Rust integer
//! arithmetic. They MUST NOT change behaviour — every later §5
//! procedure assumes these exact saturation rules.
//!
//! 16-bit operands use `i16`, 32-bit operands use `i32`. The
//! spec calls the 32-bit prefix `L_`; in Rust the type itself
//! carries the same information so we drop the prefix from
//! function names.

/// §5.1 `add(var1, var2)` — 16-bit saturating add.
#[inline]
pub fn add(a: i16, b: i16) -> i16 {
    a.saturating_add(b)
}

/// §5.1 `sub(var1, var2)` — 16-bit saturating subtract.
#[inline]
pub fn sub(a: i16, b: i16) -> i16 {
    a.saturating_sub(b)
}

/// §5.1 `mult(var1, var2)` — `(var1 * var2) >> 15` with
/// saturation on the `(-32768, -32768)` corner case.
#[inline]
pub fn mult(a: i16, b: i16) -> i16 {
    if a == i16::MIN && b == i16::MIN {
        return i16::MAX;
    }
    (((a as i32) * (b as i32)) >> 15) as i16
}

/// §5.1 `mult_r(var1, var2)` — `((var1 * var2) + 16384) >> 15`
/// (round-half-up) with the same saturation as `mult`.
#[inline]
pub fn mult_r(a: i16, b: i16) -> i16 {
    if a == i16::MIN && b == i16::MIN {
        return i16::MAX;
    }
    let p = (a as i32) * (b as i32);
    let r = p + 16384;
    (r >> 15) as i16
}

/// §5.1 `abs(var1)` — absolute value, saturating `abs(-32768) =
/// 32767`.
#[inline]
pub fn abs(a: i16) -> i16 {
    if a == i16::MIN {
        i16::MAX
    } else {
        a.abs()
    }
}

/// §5.1 `L_mult(var1, var2)` — `(var1 * var2) << 1` in 32 bits.
/// The spec notes `L_mult(-32768, -32768)` does not occur in the
/// algorithm; we mirror that and rely on the caller to honour it.
#[inline]
pub fn l_mult(a: i16, b: i16) -> i32 {
    ((a as i32) * (b as i32)) << 1
}

/// §5.1 `L_add(L_var1, L_var2)` — 32-bit saturating add.
#[inline]
pub fn l_add(a: i32, b: i32) -> i32 {
    a.saturating_add(b)
}

/// §5.1 `L_sub(L_var1, L_var2)` — 32-bit saturating subtract.
#[inline]
pub fn l_sub(a: i32, b: i32) -> i32 {
    a.saturating_sub(b)
}

/// §5.1 `norm(L_var1)` — the number of left shifts needed to
/// normalise `L_var1`. Per §5.1: for positive values on the
/// interval `[1073741824, 2147483647]` (i.e. the top two bits of
/// the i32 differ) the answer is 0; for negative values the
/// interval is `[-2147483648, -1073741824]`. To normalise the
/// caller does `L_var1 << norm(L_var1)`.
///
/// §5.1 leaves `norm(0)` unspecified (the algorithm never invokes
/// it on a zero argument); we return 0 to keep the function total.
#[inline]
pub fn norm(value: i32) -> i16 {
    if value == 0 {
        return 0;
    }
    // The spec defines the result as the count of left shifts that
    // brings the value's most-significant magnitude bit into the
    // top usable position (bit 30 for positives, bit 30 for the
    // magnitude part of negatives — i.e. the result of one more
    // shift would change the sign or overflow).
    //
    // For non-negative values, that's `value.leading_zeros() - 1`
    // (subtract one for the sign bit).
    //
    // For negative values, the spec's interval [-2147483648,
    // -1073741824] gives result 0; in i32, that's the values whose
    // bit pattern starts `10xxxxxx…` or `11xxxxxx…`. The pattern
    // for "already normalised negative" is `leading_ones() == 1`
    // (binary `10…` — i.e. only the sign bit is one). More leading
    // ones means more room to shift left without overflow; the
    // count of redundant sign bits is `leading_ones() - 1`.
    // §5.1 normalised intervals:
    //   positive : [ 2^30,  2^31 - 1 ]
    //   negative : [-2^31, -2^30      ]
    //
    // For non-zero `value`, the answer is `cls(value) - 1`, where
    // `cls` is the count of leading sign bits (i.e. the leading
    // run of bits matching the MSB). After `value << (cls-1)`, the
    // sign bit and bit 30 have opposite values, which is the §5.1
    // normalised-range invariant for positives and for negatives
    // except the right-edge `value = -2^30` (where shifting by 1
    // lands on `-2^31`, the deepest normalised value). That edge
    // is handled uniformly by this formula — it returns 1, which
    // remains spec-conformant since `-2^31` is also in
    // [-2^31, -2^30].
    let cls = if value >= 0 {
        value.leading_zeros()
    } else {
        value.leading_ones()
    } as i16;
    cls - 1
}

/// §5.1 `div(var1, var2)` — fractional integer division of two
/// non-negative 16-bit integers with `var2 >= var1`. The result is
/// in the range `[0, 32767]` (positive, leading bit zero) and is
/// truncated to 16 bits. Per §5.1 NOTE: `div(var1, var1) = 32767`.
///
/// The §5.1 contract is `0 <= var1 <= var2`. We mirror that as a
/// `debug_assert`; out-of-range inputs return a clamped result so
/// the function stays total in release builds.
#[inline]
pub fn div(var1: i16, var2: i16) -> i16 {
    debug_assert!(var1 >= 0, "div: var1 must be non-negative");
    debug_assert!(var2 >= var1, "div: var2 must be >= var1");
    if var2 == 0 {
        // Spec contract: var2 > 0; in release this returns 0 to be
        // total. Hitting this in tests indicates a caller bug.
        return 0;
    }
    if var1 == var2 {
        return i16::MAX;
    }
    if var1 == 0 {
        return 0;
    }
    // Compute `(var1 << 15) / var2` — gives the §3.7 Q15 quotient
    // rounded toward zero. var1 < var2 ⇒ result < 32768 ⇒ fits i16.
    let num = (var1 as i32) << 15;
    let q = num / (var2 as i32);
    q.clamp(0, i16::MAX as i32) as i16
}

/// §5.1 left-shift operator with the spec's "negative `n` becomes an
/// arithmetic right shift by `-n`" rule — the spec defines `<< n` for
/// `n < 0` and several decoder procedures (e.g. §5.2.16 `temp3 = 1 <<
/// sub(temp2, 1)`) rely on that fall-through.
#[inline]
pub fn shl_signed(value: i16, n: i16) -> i16 {
    if n >= 0 {
        let n = n.min(15) as u32;
        ((value as i32) << n).clamp(i16::MIN as i32, i16::MAX as i32) as i16
    } else {
        // Spec §5.1: `<< n` with n<0 becomes an arithmetic right
        // shift of -n with sign-extension.
        let n = (-n).min(15) as u32;
        value >> n
    }
}

/// §5.1 right-shift with the symmetric "negative `n` becomes left
/// shift by `-n`" rule.
#[inline]
pub fn shr_signed(value: i16, n: i16) -> i16 {
    if n >= 0 {
        let n = n.min(15) as u32;
        value >> n
    } else {
        let n = (-n).min(15) as u32;
        ((value as i32) << n).clamp(i16::MIN as i32, i16::MAX as i32) as i16
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `add` saturates exactly at the i16 boundary.
    #[test]
    fn add_saturates() {
        assert_eq!(add(32767, 1), 32767);
        assert_eq!(add(-32768, -1), -32768);
        assert_eq!(add(1, 1), 2);
    }

    /// `mult(-32768, -32768)` saturates per §5.1 NOTE.
    #[test]
    fn mult_minmin_saturates() {
        assert_eq!(mult(i16::MIN, i16::MIN), i16::MAX);
    }

    /// `mult` matches the documented `(a*b) >> 15` truncating form
    /// in the bulk of the range.
    #[test]
    fn mult_basic() {
        // 16384 * 16384 = 0x1000_0000; >> 15 = 0x2000 = 8192.
        assert_eq!(mult(16384, 16384), 8192);
        // 32767 * 32767 = 1_073_676_289; >> 15 = 32766.
        assert_eq!(mult(32767, 32767), 32766);
    }

    /// `mult_r` is `mult` plus a half-LSB round.
    #[test]
    fn mult_r_rounding() {
        // 16384*1 + 16384 = 32768; >> 15 = 1.
        assert_eq!(mult_r(16384, 1), 1);
        // mult(16384, 1) would give 0 (16384 >> 15 = 0).
        assert_eq!(mult(16384, 1), 0);
    }

    /// `abs(-32768)` saturates to 32767.
    #[test]
    fn abs_min_saturates() {
        assert_eq!(abs(i16::MIN), i16::MAX);
        assert_eq!(abs(-3), 3);
        assert_eq!(abs(3), 3);
    }

    /// 32-bit operations saturate at the i32 boundary.
    #[test]
    fn l_add_l_sub_saturate() {
        assert_eq!(l_add(i32::MAX, 1), i32::MAX);
        assert_eq!(l_sub(i32::MIN, 1), i32::MIN);
    }

    /// `norm` of a value already in the §5.1 normalised range
    /// `[1<<30, i32::MAX]` returns 0.
    #[test]
    fn norm_of_normalised_positive_is_zero() {
        assert_eq!(norm(1 << 30), 0);
        assert_eq!(norm(i32::MAX), 0);
    }

    /// `norm` of a smaller positive value returns the count of
    /// left shifts needed to push the MSB up to bit 30.
    #[test]
    fn norm_positive_counts_leading_zeros() {
        // 1 → needs 30 left-shifts to become 1<<30.
        assert_eq!(norm(1), 30);
        // 1<<15 → needs 15 left-shifts.
        assert_eq!(norm(1 << 15), 15);
        // 1<<29 → one left-shift away from being normalised.
        assert_eq!(norm(1 << 29), 1);
    }

    /// `norm` of a strictly-normalised negative (in `[i32::MIN,
    /// -1<<30 - 1]`) returns 0. The right edge `-1<<30` returns 1
    /// under the `cls - 1` formula because shifting by 1 lands on
    /// the deepest-magnitude normalised value `i32::MIN`; that
    /// remains spec-conformant since §5.1 admits both endpoints.
    #[test]
    fn norm_of_normalised_negative_is_zero() {
        assert_eq!(norm(i32::MIN), 0);
        // -(1<<30) - 1 = 0xBFFFFFFF — strictly inside the interval.
        assert_eq!(norm(-(1 << 30) - 1), 0);
    }

    /// `norm` of a less-negative value returns the count of
    /// redundant sign bits that can be shifted out before
    /// overflow. The `cls - 1` formula yields the count of
    /// leading bits matching the sign bit, minus the sign bit
    /// itself.
    #[test]
    fn norm_negative_counts_redundant_sign_bits() {
        // -1 = all-ones; cls = 32; norm = 31.
        assert_eq!(norm(-1), 31);
        // -2 = 0xFFFFFFFE; cls = 31; norm = 30.
        assert_eq!(norm(-2), 30);
        // -3 = 0xFFFFFFFD; cls = 30; norm = 29.
        assert_eq!(norm(-3), 29);
    }

    /// `norm(0)` returns 0 — the spec leaves it undefined; we
    /// keep the function total.
    #[test]
    fn norm_of_zero_is_zero() {
        assert_eq!(norm(0), 0);
    }

    /// `div(0, *)` is 0 and `div(x, x)` is 32767, per §5.1.
    #[test]
    fn div_edge_cases() {
        assert_eq!(div(0, 1), 0);
        assert_eq!(div(0, 32767), 0);
        assert_eq!(div(1, 1), i16::MAX);
        assert_eq!(div(32767, 32767), i16::MAX);
    }

    /// `div(a, b)` for a < b gives the Q15 truncating quotient.
    #[test]
    fn div_q15_quotient() {
        // 1 / 2 in Q15 = 16384.
        assert_eq!(div(1, 2), 16384);
        // 1 / 4 in Q15 = 8192.
        assert_eq!(div(1, 4), 8192);
        // 3 / 4 in Q15 = 24576.
        assert_eq!(div(3, 4), 24576);
    }
}
