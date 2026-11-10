//! GSM 06.20 clauses 4.1.8.1–4.1.8.4 (open-loop lag search, frame
//! lag trajectory, voicing mode) in the clause 5.1 fixed-point
//! arithmetic, following the operator rules of
//! `docs/audio/gsm/halfrate-traces/hr-fixed-point-schedules.md`
//! §C.1 (rules 1–20) over 16-bit weighted-speech words.
//!
//! Departures from a literal reading of the printed clause that the
//! rules carry (errata #409 H8–H11, G2):
//!
//! * the Pv test is linear, `Σ min(1, C²/G) − 0,32391·ΣR > 0`;
//! * the eq. (67)/(89) threshold is `R·(1 − (1 − C²/(R·G))^0,75)`
//!   with the ratio clamped at 0,001, by fixed-point log2/exp2;
//! * the submultiple / multiple peak test is a ±4 integer window
//!   whose maximum must not sit at an end;
//! * the trajectory delta windows are −6..+5 (forward) and −5..+6
//!   (backward) levels, only the two highest peaks per subframe
//!   anchor (a crossed one still uses its slot), and the best
//!   trajectory maximises the clamped, truncated Σ C²/G;
//! * the voicing thresholds are linear (`0x6676`, `0x46d3` against
//!   R·G).

#![allow(clippy::needless_range_loop)]

use super::fx::*;
use super::tables::{INTERP_FILTER_6, LAG_TABLE};
use super::{HR_SUBFRAME_SAMPLES as NS, HR_SUBFRAMES};

/// Weighted-speech history words in front of the frame (n = −145..−1).
pub(super) const OL_HIST: usize = 145;

const LMIN: i32 = 21;
const LMAX: i32 = 142;
/// C(k) range k = 18..=145.
const KLO: i32 = 18;
const KHI: i32 = 145;
const NK: usize = (KHI - KLO + 1) as usize;

/// Normalised energy pair (man, sh) of rule 1.
pub(super) type EnergyPair = (i16, i16);

/// Energy of a 40-sample word vector with the R0-based shift
/// (rule 1), as a rounded normalised pair.
pub(super) fn energy_pair(x: &[i16], k: i16) -> EnergyPair {
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
    let n = norm_l(l);
    (round(l_shl(l, n)), n - e)
}

/// The rule 5 argmax state: `(ccmax, gmax)`.
#[derive(Clone, Copy, Debug)]
struct ArgMax {
    cc: i16,
    g: i16,
}

impl ArgMax {
    const SEED: ArgMax = ArgMax { cc: 0, g: 0x3f };

    /// Rule 5 strict replacement test for a positive candidate.
    #[inline]
    fn better(&self, cc: i16, g: i16) -> bool {
        l_msu(l_mult(cc, self.g), self.cc, g) > 0
    }
}

/// Per-subframe correlation arrays (rules 3/4).
struct Sub {
    c: [i16; NK],
    g: [i16; NK],
    r: i16,
}

impl Sub {
    #[inline]
    fn cg(&self, k: i32) -> (i16, i16) {
        let i = (k - KLO) as usize;
        (self.c[i], self.g[i])
    }

    /// Rule 7: interpolated (C_I, G_I) at a lag in 1/6 units.
    fn interp(&self, k6: i32) -> (i16, i16) {
        let ceil = (k6 + 5) / 6;
        let base = ceil - 1;
        let j = (6 * ceil - k6) as usize;
        let mut lc = 0x8000i32;
        let mut lg = 0x8000i32;
        for i in 0..6 {
            let (c, g) = self.cg(base - 2 + i as i32);
            lc = l_mac(lc, c, INTERP_FILTER_6[i][j]);
            lg = l_mac(lg, g, INTERP_FILTER_6[i][j]);
        }
        (extract_h(lc), extract_h(lg))
    }
}

/// Largest table level whose lag is ≤ `lag6` (level 0 below the
/// table).
fn floor_level(lag6: i32) -> usize {
    let p = LAG_TABLE.partition_point(|&l| (l as i32) <= lag6);
    p.saturating_sub(1)
}

/// Table levels strictly inside `(lo6, hi6)`.
fn levels_inside(lo6: i32, hi6: i32) -> core::ops::Range<usize> {
    let a = LAG_TABLE.partition_point(|&l| (l as i32) <= lo6);
    let b = LAG_TABLE.partition_point(|&l| (l as i32) < hi6);
    a..b.max(a)
}

/// A candidate peak: lag (1/6 units) and its interpolated terms.
#[derive(Clone, Copy, Debug)]
struct Peak {
    lag6: i32,
    c: i16,
    g: i16,
}

/// Rule 8 / rule 10: the best table lag strictly within ±1 sample of
/// `centre6`, starting from the given seed. Only `C_I > 0`,
/// `G_I > 0` candidates count.
fn best_table_lag(sc: &Sub, centre6: i32, seed: ArgMax) -> Option<Peak> {
    let mut am = seed;
    let mut best = None;
    for lv in levels_inside(centre6 - 6, centre6 + 6) {
        let lag6 = LAG_TABLE[lv] as i32;
        let (c, g) = sc.interp(lag6);
        if c <= 0 || g <= 0 {
            continue;
        }
        let cc = mult_r(c, c);
        if am.better(cc, g) {
            am = ArgMax { cc, g };
            best = Some(Peak { lag6, c, g });
        }
    }
    best
}

/// Rule 11 peak test: rule 5 over the integer window `k1 ± 4`
/// (clipped to 20..143); a peak only when the maximum is interior.
fn integer_peak(sc: &Sub, k1: i32) -> Option<i32> {
    let lo = (k1 - 4).max(20);
    let hi = (k1 + 4).min(143);
    let mut am = ArgMax::SEED;
    let mut best = None;
    for k in lo..=hi {
        let (c, g) = sc.cg(k);
        if c <= 0 || g <= 0 {
            continue;
        }
        let cc = mult_r(c, c);
        if am.better(cc, g) {
            am = ArgMax { cc, g };
            best = Some(k);
        }
    }
    let k = best?;
    if k == lo || k == hi { None } else { Some(k) }
}

/// Rule 9: the eq. (67)/(89) threshold word.
fn threshold(r: i16, ccmax: i16, gmax: i16) -> i16 {
    let rg = l_mult(r, gmax);
    let rgw = extract_h(rg);
    let d = l_sub(rg, l_deposit_h(ccmax));
    let mut x: i16 = if d > 0 {
        divide_s(extract_h(d), rgw)
    } else {
        0x21
    };
    if x <= 0x21 {
        x = 0x21;
    }
    // log2(x) in Q10.
    let n = norm_l(l_deposit_h(x));
    let u = extract_h(l_shl(l_deposit_h(x), n));
    let mut a = l_mult(mult_r(u, u), -0x2b2a);
    a = l_mac(a, u, 0x7fc5);
    a = l_add(a, l_deposit_h(-0x54d0));
    a = l_add(l_shr(a, 3) & 0x03ff_ffff, l_deposit_h(shl(-(n + 1), 10)));
    let lg = round(a);
    let e = l_mult(lg, 0x6000);
    // exp2(e) in Q15.
    let h = extract_h(e);
    let t = l_mult(h, 0x20);
    let int = extract_h(t);
    let f = t & 0xffff;
    let v = extract_l(l_shr(f - 65536, 1));
    let mut p = l_mult(mult_r(v, v), 0x15ef);
    p = l_mac(p, v, 0x556f);
    p = l_add(p, l_deposit_h(0x7fbd));
    let cnt = -1 - int;
    if cnt > 0 {
        p = l_shr(p, cnt);
    }
    let pw = extract_h(p);
    extract_h(l_shr(l_msu(l_deposit_h(r), r, pw), 1))
}

/// Rule 13: the stored harmonic-noise-weighting word `0,2·C/G`.
fn hnw_word(c: i16, g: i16) -> i16 {
    let nc = norm_s(c);
    let ng = norm_s(g);
    let a = round(l_shr(l_mult(shl(c, nc), 0x3333), 1));
    let mut w = divide_s(a, shl(g, ng));
    if ng > nc {
        w = shl(w, ng - nc);
    } else if ng < nc {
        w = shift_r(w, ng - nc);
    }
    w
}

/// Per-frame open-loop result.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct OpenLoopFx {
    pub mode: u8,
    pub levels: [usize; HR_SUBFRAMES],
    /// Lpitch (1/6 units) and the stored λ_hnw/2 word per subframe.
    pub lpitch6: [i32; HR_SUBFRAMES],
    pub hnw: [i16; HR_SUBFRAMES],
    /// R(0,m) words (rule 3).
    pub r0m: [i16; HR_SUBFRAMES],
    /// The scaling shift of rule 2.
    pub scale: i16,
}

/// Open-loop state carried across frames: the previous frame's four
/// subframe energy pairs.
#[derive(Clone, Copy, Debug, Default)]
pub(super) struct OpenLoopState {
    pub prev_pairs: [EnergyPair; HR_SUBFRAMES],
}

/// R0 index → energy shift (rule 1).
pub(super) use super::decode::r0_energy_shift;

/// Run rules 1–20 on `y` = 145 history words followed by the
/// frame's 160 weighted-speech words.
pub(super) fn open_loop_fx(
    y: &[i16; OL_HIST + 160],
    st: &mut OpenLoopState,
    prev_r0: u8,
    cur_r0: u8,
) -> OpenLoopFx {
    // Rule 1.
    let mut pairs = [(0i16, 0i16); HR_SUBFRAMES];
    for m in 0..HR_SUBFRAMES {
        let k = r0_energy_shift(if m == 0 { prev_r0 } else { cur_r0 });
        let s = OL_HIST + m * NS;
        pairs[m] = energy_pair(&y[s..s + NS], k);
    }
    // Rule 2.
    let mut best: Option<EnergyPair> = None;
    for &(man, sh) in st.prev_pairs.iter().chain(pairs.iter()) {
        if man <= 0 {
            continue;
        }
        best = match best {
            None => Some((man, sh)),
            Some((bm, bs)) => {
                if sh < bs || (sh == bs && man > bm) {
                    Some((man, sh))
                } else {
                    Some((bm, bs))
                }
            }
        };
    }
    st.prev_pairs = pairs;
    let scale = best.map(|(_, sh)| (sh >> 1) - 1).unwrap_or(0);
    let mut ys = [0i16; OL_HIST + 160];
    for (d, &s) in ys.iter_mut().zip(y.iter()) {
        *d = shl(s, scale);
    }
    let yat = |n: i32| ys[(n + OL_HIST as i32) as usize];

    // Rule 3: G windows starting at n = −145..102.
    let mut gwin = [0i16; 248];
    let mut acc = 0i32;
    for n in -145..-105 {
        acc = l_mac(acc, yat(n), yat(n));
    }
    gwin[0] = extract_h(acc);
    for t in 1..248i32 {
        let out = yat(-145 + t - 1);
        let inn = yat(-145 + t + 39);
        acc = l_msu(acc, out, out);
        acc = l_mac(acc, inn, inn);
        gwin[t as usize] = extract_h(acc);
    }
    let gstart = |start: i32| gwin[(start + 145) as usize];
    let mut subs: Vec<Sub> = Vec::with_capacity(HR_SUBFRAMES);
    for m in 0..HR_SUBFRAMES as i32 {
        let base = 40 * m;
        let r = if m < 3 {
            gstart(base)
        } else {
            let mut l = 0i32;
            for n in 120..160 {
                l = l_mac(l, yat(n), yat(n));
            }
            extract_h(l)
        };
        let mut c = [0i16; NK];
        let mut g = [0i16; NK];
        for k in KLO..=KHI {
            let mut l = 0i32;
            for n in 0..40 {
                l = l_mac(l, yat(base + n), yat(base + n - k));
            }
            c[(k - KLO) as usize] = extract_h(l);
            g[(k - KLO) as usize] = gstart(base - k);
        }
        subs.push(Sub { c, g, r });
    }
    let mut r0m = [0i16; HR_SUBFRAMES];
    for m in 0..HR_SUBFRAMES {
        r0m[m] = subs[m].r;
    }

    // Rule 5: integer peaks.
    let mut ipk: [Option<(i32, ArgMax)>; HR_SUBFRAMES] = [None; HR_SUBFRAMES];
    for m in 0..HR_SUBFRAMES {
        let mut am = ArgMax::SEED;
        let mut best = None;
        for k in LMIN..=LMAX {
            let (c, g) = subs[m].cg(k);
            if c <= 0 || g <= 0 {
                continue;
            }
            let cc = mult_r(c, c);
            if am.better(cc, g) {
                am = ArgMax { cc, g };
                best = Some(k);
            }
        }
        ipk[m] = best.map(|k| (k, am));
    }

    let unvoiced = OpenLoopFx {
        mode: 0,
        levels: [0; HR_SUBFRAMES],
        lpitch6: [126; HR_SUBFRAMES],
        hnw: [0; HR_SUBFRAMES],
        r0m,
        scale,
    };

    // Rule 6: Pv test.
    let mut l = 0i32;
    let mut sq: i16 = 0;
    for m in 0..HR_SUBFRAMES {
        l = l_mac(l, subs[m].r, -0x2976);
        if let Some((_, am)) = ipk[m] {
            let q = if am.cc < am.g {
                divide_s(am.cc, am.g)
            } else {
                0x7fff
            };
            sq = add(sq, q);
        }
    }
    l = l_add(l, l_deposit_h(sq));
    if l <= 0 {
        return unvoiced;
    }

    // 4.1.8.2: peak lists, Lpitch and λ_hnw per subframe.
    let mut lists: Vec<Vec<Peak>> = vec![Vec::new(); HR_SUBFRAMES];
    let mut lpitch6 = [126i32; HR_SUBFRAMES];
    let mut hnw = [0i16; HR_SUBFRAMES];
    for m in 0..HR_SUBFRAMES {
        let sc = &subs[m];
        let Some((lp, am0)) = ipk[m] else {
            continue;
        };
        // Rule 8.
        let Some(first) = best_table_lag(sc, 6 * lp, ArgMax::SEED) else {
            continue;
        };
        let mut list = vec![first];
        // Rule 9.
        let t = threshold(sc.r, am0.cc, am0.g);
        let tseed = ArgMax { cc: t, g: 0x4000 };
        // Rule 11: submultiples.
        let w_j: [i16; 5] = [0x0aab, 0x071c, 0x0555, 0x0444, 0x038e];
        for &w in w_j.iter() {
            let k1 = mult_r(first.lag6 as i16, w) as i32;
            if k1 < LMIN {
                break;
            }
            if let Some(kp) = integer_peak(sc, k1)
                && let Some(p) = best_table_lag(sc, 6 * kp, tseed)
            {
                list.push(p);
            }
        }
        // Rule 12: Lpitch.
        let centre = list.last().unwrap().lag6;
        let mut am = ArgMax::SEED;
        let mut found: Option<(i32, i16, i16)> = None;
        for k6 in (centre - 5).max(126)..=(centre + 5).min(852) {
            let (c, g) = sc.interp(k6);
            if c <= 0 || g <= 0 {
                continue;
            }
            let cc = mult_r(c, c);
            if am.better(cc, g) {
                am = ArgMax { cc, g };
                found = Some((k6, c, g));
            }
        }
        let (lpl, cpl, gpl) = found.unwrap_or((126, 0, 0x3f));
        lpitch6[m] = lpl;
        // Rule 13.
        hnw[m] = hnw_word(cpl, gpl);
        // Rule 14: multiples.
        let mut k1s = lpl;
        loop {
            k1s += lpl;
            let k1 = mult_r(sat16(k1s), 0x1555) as i32;
            if k1 > LMAX {
                break;
            }
            if let Some(kp) = integer_peak(sc, k1)
                && let Some(p) = best_table_lag(sc, 6 * kp, tseed)
            {
                list.push(p);
            }
        }
        // Rule 15: sort by C²/G, largest first, dropping duplicates.
        let mut sorted: Vec<Peak> = Vec::new();
        let mut left = list;
        loop {
            let mut am = ArgMax::SEED;
            let mut pick: Option<usize> = None;
            for (i, p) in left.iter().enumerate() {
                let cc = mult_r(p.c, p.c);
                if p.g > 0 && am.better(cc, p.g) {
                    am = ArgMax { cc, g: p.g };
                    pick = Some(i);
                }
            }
            let Some(i) = pick else { break };
            let p = left.remove(i);
            if !sorted.iter().any(|q| q.lag6 == p.lag6) {
                sorted.push(p);
            }
        }
        lists[m] = sorted;
    }

    // 4.1.8.3: trajectories.
    struct Traj {
        lv: [usize; HR_SUBFRAMES],
        cc: [i16; HR_SUBFRAMES],
        g: [i16; HR_SUBFRAMES],
    }
    let mut trajs: Vec<Traj> = Vec::new();
    let window = |sc: &Sub, lo: i32, hi: i32| -> (usize, i16, i16) {
        let lo = lo.max(0) as usize;
        let hi = hi.min(255) as usize;
        let mut sel: Option<(usize, i16, i16)> = None;
        for lv in lo..=hi {
            let (c, g) = sc.interp(LAG_TABLE[lv] as i32);
            let mut cc = mult_r(c, c);
            if c < 0 {
                cc = negate(cc);
            }
            match sel {
                None => sel = Some((lv, cc, g)),
                Some((_, bcc, bg)) => {
                    if g > 0 && l_msu(l_mult(cc, bg), g, bcc) > 0 {
                        sel = Some((lv, cc, g));
                    }
                }
            }
        }
        sel.unwrap()
    };
    let mut best_t: Option<usize> = None;
    let mut best_total: i16 = i16::MIN;
    for m in 0..HR_SUBFRAMES {
        for p in lists[m].iter().take(2) {
            let lv0 = floor_level(p.lag6);
            if trajs.iter().any(|t| t.lv[m] == lv0) {
                continue;
            }
            let mut t = Traj {
                lv: [0; HR_SUBFRAMES],
                cc: [0; HR_SUBFRAMES],
                g: [0; HR_SUBFRAMES],
            };
            t.lv[m] = lv0;
            t.cc[m] = mult_r(p.c, p.c);
            t.g[m] = p.g;
            for s in m + 1..HR_SUBFRAMES {
                let c = t.lv[s - 1] as i32;
                let (lv, cc, g) = window(&subs[s], c - 6, c + 5);
                t.lv[s] = lv;
                t.cc[s] = cc;
                t.g[s] = g;
            }
            for s in (0..m).rev() {
                let c = t.lv[s + 1] as i32;
                let (lv, cc, g) = window(&subs[s], c - 5, c + 6);
                t.lv[s] = lv;
                t.cc[s] = cc;
                t.g[s] = g;
            }
            // Rule 19.
            let mut total: i16 = 0;
            for s in 0..HR_SUBFRAMES {
                let (cc, g) = (t.cc[s], t.g[s]);
                if g <= 0 {
                    continue;
                }
                let a = abs_s(cc);
                if a > g {
                    total = if cc < 0 {
                        sub(total, 0x7fff)
                    } else {
                        add(total, 0x7fff)
                    };
                } else {
                    let q = divide_s(a, g);
                    total = if cc < 0 { sub(total, q) } else { add(total, q) };
                }
            }
            if total > best_total {
                best_total = total;
                best_t = Some(trajs.len());
            }
            trajs.push(t);
        }
    }
    let Some(bi) = best_t else {
        return OpenLoopFx {
            lpitch6,
            hnw,
            ..unvoiced
        };
    };
    let t = &trajs[bi];

    // Rule 20.
    let mut mode = 3u8;
    let mut lowest = 2u8;
    for m in 0..HR_SUBFRAMES {
        let rg = mult_r(t.g[m], subs[m].r);
        let base = l_deposit_h(t.cc[m]);
        if l_mac(base, rg, -0x6676) < 0 {
            mode = lowest;
            if l_mac(base, rg, -0x46d3) < 0 {
                mode = 1;
                lowest = 1;
            }
        }
    }
    OpenLoopFx {
        mode,
        levels: t.lv,
        lpitch6,
        hnw,
        r0m,
        scale,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(y: &[i16; OL_HIST + 160]) -> OpenLoopFx {
        let mut st = OpenLoopState::default();
        // Two passes so the previous-frame energies are populated.
        let _ = open_loop_fx(y, &mut st, 20, 20);
        open_loop_fx(y, &mut st, 20, 20)
    }

    /// A clean periodic signal lands on its period at every subframe
    /// and is strongly voiced.
    #[test]
    fn periodic_input_is_strongly_voiced_at_its_period() {
        let period = 60usize;
        let mut y = [0i16; OL_HIST + 160];
        for (n, v) in y.iter_mut().enumerate() {
            let ph = (n % period) as f64 / period as f64;
            *v = (4000.0
                * ((ph * core::f64::consts::TAU).sin()
                    + 0.5 * (2.0 * ph * core::f64::consts::TAU).cos())) as i16;
        }
        let ol = run(&y);
        assert_eq!(ol.mode, 3, "{ol:?}");
        for &lv in &ol.levels {
            assert!(
                (LAG_TABLE[lv] as i32 - 6 * period as i32).abs() <= 3,
                "{ol:?}"
            );
        }
        for &w in &ol.hnw {
            // 0,2·C/G ≈ 0,2 for a periodic signal.
            assert!((5000..=7000).contains(&w), "{ol:?}");
        }
    }

    /// White noise and silence are unvoiced.
    #[test]
    fn noise_and_silence_are_unvoiced() {
        let mut state = 0x1234_5678u32;
        let mut y = [0i16; OL_HIST + 160];
        for v in y.iter_mut() {
            state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            *v = ((state >> 16) as i16) >> 3;
        }
        assert_eq!(run(&y).mode, 0);
        assert_eq!(run(&[0; OL_HIST + 160]).mode, 0);
    }

    /// Trajectory lags stay within the narrowed delta windows.
    #[test]
    fn trajectory_respects_delta_windows() {
        let mut y = [0i16; OL_HIST + 160];
        let len = y.len() as f64;
        for (n, v) in y.iter_mut().enumerate() {
            let p = 40.0 + 8.0 * (n as f64 / len);
            *v = (6000.0 * ((n as f64) * core::f64::consts::TAU / p).sin()) as i16;
        }
        let ol = run(&y);
        if ol.mode != 0 {
            for w in ol.levels.windows(2) {
                let d = w[1] as i32 - w[0] as i32;
                assert!((-6..=6).contains(&d), "{ol:?}");
            }
        }
    }

    /// Rule 9 threshold: a peak explaining almost all the energy
    /// clamps the ratio at 0,001 and leaves ≈ R/2.
    #[test]
    fn threshold_clamped_ratio() {
        // SEQ01 frame 5 subframe 0 values from the staged trace rows
        // of the rule: R = 3167, CC = 37, G = 382 → 1574.
        assert_eq!(threshold(3167, 37, 382), 1574);
        // R = 3360, CC = 260, G = 2561 → 1636.
        assert_eq!(threshold(3360, 260, 2561), 1636);
    }

    /// Rule 13 word for Cp = 1097, Gp = 382 is 18816.
    #[test]
    fn hnw_word_matches_rule() {
        assert_eq!(hnw_word(1097, 382), 18816);
        assert_eq!(hnw_word(0, 0x3f), 0);
    }
}
