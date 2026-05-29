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
}
