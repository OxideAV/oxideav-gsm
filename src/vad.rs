//! GSM 06.32 Voice Activity Detector (full rate) — ETSI EN 300 965.
//!
//! The VAD indicates whether each 20 ms frame produced by the speech
//! encoder contains speech. Its output is the binary `vad` flag the
//! GSM 06.31 TX DTX handler consumes. The implementation follows the
//! **computational details** of EN 300 965 clause 3 (the normative
//! bit-exact description; clause 2 is the high-level overview) and is
//! validated bit-exactly against the annex A.2/B digital test
//! sequences (`tests/conformance_vad.rs`).
//!
//! The VAD taps four encoder-internal variables per frame (clause 3):
//! `L_ACF[0..8]` and `scalauto` from the §5.2.4 autocorrelation, the
//! four per-sub-segment LTP lags `Nc`, and the §5.2.2
//! offset-compensated signal frame `sof[0..159]` — bundled by
//! [`crate::EncoderState::encode_frame_with_vad_tap`] as
//! [`crate::VadTap`].
//!
//! Clause structure implemented here:
//!
//! * **3.1** adaptive filtering + energy computation → pseudo-float
//!   `pvad` (energy of the adaptively filtered signal) and `acf0`
//!   (input energy);
//! * **3.2** ACF averaging over 4-frame windows (`L_av0`, `L_av1`);
//! * **3.3** predictor values `rav1[0..8]` (Schur → step-up →
//!   autocorrelated predictors);
//! * **3.4** spectral comparison → `stat` (spectral stationarity);
//! * **3.5** periodicity detection → `ptch`;
//! * **3.6** threshold adaptation (`thvad`, adaptive filter `rvad`);
//! * **3.7** VAD decision (`vvad = pvad > thvad`);
//! * **3.8** VAD hangover addition → final `vad` flag;
//! * **3.9** periodicity updating (per-sub-segment lag comparison);
//! * **3.10** information tone detection (downlink only) → `tone`.
//!
//! Clause 3 fixes the evaluation order: 3.9 (periodicity updating)
//! and 3.10 (tone detection) are *"done after the processing of the
//! current speech encoder frame"* — their outputs feed the **next**
//! frame's clauses 3.5/3.6, which is why `oldlagcount` /
//! `veryoldlagcount` / `tone` are Table 3.1 state variables.
//!
//! Pseudo-floating point: `pvad`, `thvad` and `acf0` are represented
//! as `(exponent, mantissa)` pairs with `value = 2^e · (m / 32768)`
//! and a normalized mantissa `m ≥ 16384` (clause 3). Comparisons are
//! exponent-first, and the single pseudo-float addition is the
//! `pvad + margin` computation of clause 3.6.

use crate::arith::{abs, add, div, l_add, l_mult, l_sub, mult, mult_r, norm, shr_signed, sub};
use crate::bitstream::FRAME_SAMPLES;
use crate::encoder::VadTap;

/// Table 3.2 (clause 3.10) — the Hanning window half `hann[0..=79]`;
/// the window is symmetric (`hann[159-i] = hann[i]` per the 3.10.1
/// windowing loop).
pub const HANN: [i16; 80] = [
    0, 12, 51, 114, 204, 318, 458, 622, 811, 1025, 1262, 1523, 1807, 2114, 2444, 2795, 3167, 3560,
    3972, 4405, 4856, 5325, 5811, 6314, 6832, 7365, 7913, 8473, 9046, 9631, 10226, 10831, 11444,
    12065, 12693, 13326, 13964, 14607, 15251, 15898, 16545, 17192, 17838, 18482, 19122, 19758,
    20389, 21014, 21631, 22240, 22840, 23430, 24009, 24575, 25130, 25670, 26196, 26707, 27201,
    27679, 28139, 28581, 29003, 29406, 29789, 30151, 30491, 30809, 31105, 31377, 31626, 31852,
    32053, 32230, 32382, 32509, 32611, 32688, 32739, 32764,
];

// Table 3.2 (clause 3.6) — pseudo-float constants.
const E_PTH: i16 = 19;
const M_PTH: i16 = 18750;
const E_PLEV: i16 = 20;
const M_PLEV: i16 = 25000;
const E_MARGIN: i16 = 27;
const M_MARGIN: i16 = 19531;

/// 32-bit shift with a possibly negative left-shift count: `v << n`
/// for `n >= 0`, `v >> -n` otherwise (arithmetic). Clause 3.4's
/// `( L_av0[i] << shift-3 )` needs the negative direction when
/// `norm(L_av0[0]) < 3`.
fn l_shl_s(v: i32, n: i16) -> i32 {
    if n >= 0 {
        v << (n as u32)
    } else {
        v >> ((-n) as u32)
    }
}

/// Which link direction the VAD serves (clause 3.10).
///
/// *"Tone is only calculated for the VAD in the downlink. In the
/// uplink VAD tone=0."* Information-tone detection prevents the
/// threshold adapting on network tones (ringing etc.), which only
/// occur downlink.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum VadMode {
    /// Downlink VAD — clause 3.10 tone detection runs each frame.
    #[default]
    Downlink,
    /// Uplink VAD — `tone = 0` always.
    Uplink,
}

/// GSM 06.32 VAD state — the Table 3.1 memory variables.
#[derive(Debug, Clone)]
pub struct Vad {
    mode: VadMode,
    // Adaptive filter coefficients (3.1, 3.6).
    rvad: [i16; 9],
    normrvad: i16,
    // Delay lines of the autocorrelation coefficients (3.2).
    l_sacf: [i32; 27],
    l_sav0: [i32; 36],
    pt_sacf: i16,
    pt_sav0: i16,
    // Distance measure (3.4).
    l_lastdm: i32,
    // Periodicity counters (3.5, 3.9).
    oldlagcount: i16,
    veryoldlagcount: i16,
    // Adaptive threshold (3.6).
    e_thvad: i16,
    m_thvad: i16,
    // Counter for adaptation (3.6).
    adaptcount: i16,
    // Hangover flags (3.8).
    burstcount: i16,
    hangcount: i16,
    // LTP lag memory (3.9).
    oldlag: i16,
    // Tone detection (3.10).
    tone: i16,
}

impl Default for Vad {
    fn default() -> Self {
        Self::new(VadMode::Downlink)
    }
}

impl Vad {
    /// Fresh VAD in the Table 3.1 reset state.
    pub fn new(mode: VadMode) -> Self {
        Self {
            mode,
            rvad: [24576, -16384, 4096, 0, 0, 0, 0, 0, 0],
            normrvad: 7,
            l_sacf: [0; 27],
            l_sav0: [0; 36],
            pt_sacf: 0,
            pt_sav0: 0,
            l_lastdm: 0,
            oldlagcount: 0,
            veryoldlagcount: 0,
            e_thvad: 20,
            m_thvad: 31250,
            adaptcount: 0,
            burstcount: 0,
            hangcount: -1,
            oldlag: 40,
            tone: 0,
        }
    }

    /// Reset every Table 3.1 variable to its initial value, keeping
    /// the configured [`VadMode`]. This is the VAD's part of the §4.3
    /// encoder-homing reset (GSM 06.10 §4.3 step 2 lists the VAD
    /// among the sub-modules to home).
    pub fn reset(&mut self) {
        *self = Self::new(self.mode);
    }

    /// Process one frame's encoder tap and return the final `vad`
    /// flag (hangover included, clause 3.8).
    pub fn process_frame(&mut self, tap: &VadTap) -> bool {
        // 3.1 — adaptive filtering and energy computation.
        let (e_pvad, m_pvad, e_acf0, m_acf0, scalvad) = self.energy_computation(tap);
        // 3.2 — ACF averaging.
        let (l_av0, l_av1) = self.acf_averaging(&tap.l_acf, scalvad);
        // 3.3 — predictor values computation.
        let (rav1, normrav1) = predictor_values(&l_av1);
        // 3.4 — spectral comparison.
        let stat = self.spectral_comparison(&l_av0, &rav1, normrav1);
        // 3.5 — periodicity detection.
        let ptch: i16 = if add(self.oldlagcount, self.veryoldlagcount) >= 4 {
            1
        } else {
            0
        };
        // 3.6 — threshold adaptation.
        self.threshold_adaptation(e_pvad, m_pvad, e_acf0, m_acf0, stat, ptch, &rav1, normrav1);
        // 3.7 — VAD decision.
        let mut vvad: i16 = 0;
        if e_pvad > self.e_thvad {
            vvad = 1;
        }
        if e_pvad == self.e_thvad && m_pvad > self.m_thvad {
            vvad = 1;
        }
        // 3.8 — VAD hangover addition.
        let vad = self.hangover(vvad);
        // 3.9 — periodicity updating (after the VAD decision, with
        // the current frame's LTP lags; feeds the next frame's 3.5).
        self.periodicity_updating(&tap.lags);
        // 3.10 — tone detection (downlink only; feeds the next
        // frame's 3.6).
        self.tone = match self.mode {
            VadMode::Downlink => tone_detection(&tap.sof),
            VadMode::Uplink => 0,
        };

        vad == 1
    }

    /// Clause 3.1 — compute the pseudo-float `pvad` (energy of the
    /// adaptively filtered signal) and `acf0` (input energy), plus
    /// `scalvad` for clause 3.2.
    fn energy_computation(&self, tap: &VadTap) -> (i16, i16, i16, i16, i16) {
        let scalvad = if tap.scalauto < 0 { 0 } else { tap.scalauto };

        if tap.l_acf[0] == 0 {
            return (-32768, 0, -32768, 0, scalvad);
        }

        // Re-normalization of the L_ACF[0..8].
        let normacf = norm(tap.l_acf[0]);
        let mut sacf = [0i16; 9];
        for (slot, &acf) in sacf.iter_mut().zip(tap.l_acf.iter()) {
            *slot = ((acf << (normacf as u32)) >> 19) as i16;
        }

        // Computation of e_acf0 and m_acf0.
        let mut e_acf0 = add(32, scalvad << 1);
        e_acf0 = sub(e_acf0, normacf);
        let m_acf0 = sacf[0] << 3;

        // Computation of e_pvad and m_pvad.
        let mut e_pvad = add(e_acf0, 14);
        e_pvad = sub(e_pvad, self.normrvad);

        let mut l_temp: i32 = 0;
        for (&sc, &rv) in sacf.iter().zip(self.rvad.iter()).skip(1) {
            l_temp = l_add(l_temp, l_mult(sc, rv));
        }
        l_temp = l_add(l_temp, l_mult(sacf[0], self.rvad[0]) >> 1);

        if l_temp <= 0 {
            l_temp = 1;
        }
        let normprod = norm(l_temp);
        e_pvad = sub(e_pvad, normprod);
        let m_pvad = ((l_temp << (normprod as u32)) >> 16) as i16;

        (e_pvad, m_pvad, e_acf0, m_acf0, scalvad)
    }

    /// Clause 3.2 — ACF averaging: 4-frame sums `L_av0` (including
    /// the current frame) and `L_av1` (the previous window, delayed
    /// by 4 frames — used for the predictor computation because
    /// `av0` may contain speech, clause 2.2.3).
    fn acf_averaging(&mut self, l_acf: &[i32; 9], scalvad: i16) -> ([i32; 9], [i32; 9]) {
        let scal = sub(10, scalvad << 1);
        let mut l_av0 = [0i32; 9];
        let mut l_av1 = [0i32; 9];
        for i in 0..=8 {
            let l_temp = l_acf[i] >> (scal as u32);
            l_av0[i] = l_add(self.l_sacf[i], l_temp);
            l_av0[i] = l_add(self.l_sacf[i + 9], l_av0[i]);
            l_av0[i] = l_add(self.l_sacf[i + 18], l_av0[i]);
            self.l_sacf[(self.pt_sacf as usize) + i] = l_temp;
            l_av1[i] = self.l_sav0[(self.pt_sav0 as usize) + i];
            self.l_sav0[(self.pt_sav0 as usize) + i] = l_av0[i];
        }
        // Update of the array pointers.
        if self.pt_sacf == 18 {
            self.pt_sacf = 0;
        } else {
            self.pt_sacf = add(self.pt_sacf, 9);
        }
        if self.pt_sav0 == 27 {
            self.pt_sav0 = 0;
        } else {
            self.pt_sav0 = add(self.pt_sav0, 9);
        }
        (l_av0, l_av1)
    }

    /// Clause 3.4 — spectral comparison: distortion measure between
    /// the current averaged spectrum (`L_av0`) and the predictor
    /// values (`rav1`); sets `stat` when the frame-to-frame change
    /// is below the threshold.
    fn spectral_comparison(&mut self, l_av0: &[i32; 9], rav1: &[i16; 9], normrav1: i16) -> i16 {
        // Re-normalize L_av0[0..8].
        let mut sav0 = [0i16; 9];
        if l_av0[0] == 0 {
            sav0 = [4095; 9];
        } else {
            let shift = norm(l_av0[0]);
            for i in 0..=8 {
                sav0[i] = (l_shl_s(l_av0[i], sub(shift, 3)) >> 16) as i16;
            }
        }

        // Compute partial sum of dm.
        let mut l_sump: i32 = 0;
        for i in 1..=8 {
            l_sump = l_add(l_sump, l_mult(rav1[i], sav0[i]));
        }

        // Compute the division of the partial sum by sav0[0].
        let mut l_temp = if l_sump < 0 { l_sub(0, l_sump) } else { l_sump };
        let mut l_dm: i32;
        let shift: i16;
        if l_temp == 0 {
            l_dm = 0;
            shift = 0;
        } else {
            sav0[0] <<= 3;
            shift = norm(l_temp);
            let mut temp = ((l_temp << (shift as u32)) >> 16) as i16;
            let divshift: i16;
            if sav0[0] >= temp {
                divshift = 0;
                temp = div(temp, sav0[0]);
            } else {
                divshift = 1;
                temp = sub(temp, sav0[0]);
                temp = div(temp, sav0[0]);
            }
            l_dm = if divshift == 1 { 32768 } else { 0 };
            l_dm = l_add(l_dm, temp as i32) << 1;
            if l_sump < 0 {
                l_dm = l_sub(0, l_dm);
            }
        }

        // Re-normalization and final computation of L_dm.
        l_dm <<= 14;
        l_dm >>= shift as u32;
        l_dm = l_add(l_dm, (rav1[0] as i32) << 11);
        l_dm >>= normrav1 as u32;

        // Compute the difference and save L_dm.
        l_temp = l_sub(l_dm, self.l_lastdm);
        self.l_lastdm = l_dm;
        if l_temp < 0 {
            l_temp = l_sub(0, l_temp);
        }
        l_temp = l_sub(l_temp, 3277);

        // Evaluation of the stat flag.
        if l_temp < 0 {
            1
        } else {
            0
        }
    }

    /// Clause 3.6 — threshold adaptation, following the figure 2.2
    /// flowchart with the Table 3.2 pseudo-float constants.
    #[allow(clippy::too_many_arguments)]
    fn threshold_adaptation(
        &mut self,
        e_pvad: i16,
        m_pvad: i16,
        e_acf0: i16,
        m_acf0: i16,
        stat: i16,
        ptch: i16,
        rav1: &[i16; 9],
        normrav1: i16,
    ) {
        // Test if acf0 < pth; if yes set thvad to plev.
        let mut comp: i16 = 0;
        if e_acf0 < E_PTH {
            comp = 1;
        }
        if e_acf0 == E_PTH && m_acf0 < M_PTH {
            comp = 1;
        }
        if comp == 1 {
            self.e_thvad = E_PLEV;
            self.m_thvad = M_PLEV;
            return;
        }

        // Test if an adaptation is needed.
        let mut comp: i16 = 0;
        if ptch == 1 {
            comp = 1;
        }
        if stat == 0 {
            comp = 1;
        }
        if self.tone == 1 {
            comp = 1;
        }
        if comp == 1 {
            self.adaptcount = 0;
            return;
        }

        // Incrementation of adaptcount.
        self.adaptcount = add(self.adaptcount, 1);
        if self.adaptcount <= 8 {
            return;
        }

        // Computation of thvad - (thvad/dec).
        self.m_thvad = sub(self.m_thvad, self.m_thvad >> 5);
        if self.m_thvad < 16384 {
            self.m_thvad <<= 1;
            self.e_thvad = sub(self.e_thvad, 1);
        }

        // Computation of pvad*fac (fac = 3.0: 3·m/2, exponent + 1).
        let mut l_temp = l_add(m_pvad as i32, m_pvad as i32);
        l_temp = l_add(l_temp, m_pvad as i32);
        l_temp >>= 1;
        let mut e_temp = add(e_pvad, 1);
        if l_temp > 32767 {
            l_temp >>= 1;
            e_temp = add(e_temp, 1);
        }
        let m_temp = l_temp as i16;

        // Test if thvad < pvad*fac.
        let mut comp: i16 = 0;
        if self.e_thvad < e_temp {
            comp = 1;
        }
        if self.e_thvad == e_temp && self.m_thvad < m_temp {
            comp = 1;
        }

        // Computation of minimum(thvad + thvad/inc, pvad*fac).
        if comp == 1 {
            // Compute thvad + (thvad/inc).
            let l_temp = l_add(self.m_thvad as i32, (self.m_thvad >> 4) as i32);
            if l_temp > 32767 {
                self.m_thvad = (l_temp >> 1) as i16;
                self.e_thvad = add(self.e_thvad, 1);
            } else {
                self.m_thvad = l_temp as i16;
            }
            let mut comp2: i16 = 0;
            if e_temp < self.e_thvad {
                comp2 = 1;
            }
            if e_temp == self.e_thvad && m_temp < self.m_thvad {
                comp2 = 1;
            }
            if comp2 == 1 {
                self.e_thvad = e_temp;
                self.m_thvad = m_temp;
            }
        }

        // Computation of pvad + margin (the one pseudo-float
        // addition of the algorithm).
        let (e_temp, m_temp): (i16, i16);
        if e_pvad == E_MARGIN {
            let l_temp = l_add(m_pvad as i32, M_MARGIN as i32);
            m_temp = (l_temp >> 1) as i16;
            e_temp = add(e_pvad, 1);
        } else if e_pvad > E_MARGIN {
            let t = sub(e_pvad, E_MARGIN);
            // §5.1 ">>" semantics: an i16 shifted by >= 15 collapses
            // to its sign fill (0 for the positive mantissas here).
            let t = shr_signed(M_MARGIN, t);
            let l_temp = l_add(m_pvad as i32, t as i32);
            if l_temp > 32767 {
                e_temp = add(e_pvad, 1);
                m_temp = (l_temp >> 1) as i16;
            } else {
                e_temp = e_pvad;
                m_temp = l_temp as i16;
            }
        } else {
            let t = sub(E_MARGIN, e_pvad);
            // Same §5.1 large-count collapse; e_pvad can sit far
            // below E_MARGIN on very quiet frames (POLE2 corpus).
            let t = shr_signed(m_pvad, t);
            let l_temp = l_add(M_MARGIN as i32, t as i32);
            if l_temp > 32767 {
                e_temp = add(E_MARGIN, 1);
                m_temp = (l_temp >> 1) as i16;
            } else {
                e_temp = E_MARGIN;
                m_temp = l_temp as i16;
            }
        }

        // Test if thvad > pvad + margin.
        let mut comp: i16 = 0;
        if self.e_thvad > e_temp {
            comp = 1;
        }
        if self.e_thvad == e_temp && self.m_thvad > m_temp {
            comp = 1;
        }
        if comp == 1 {
            self.e_thvad = e_temp;
            self.m_thvad = m_temp;
        }

        // Initialize new rvad[0..8] in memory.
        self.normrvad = normrav1;
        self.rvad = *rav1;

        // Set adaptcount to adp + 1.
        self.adaptcount = 9;
    }

    /// Clause 3.8 — VAD hangover addition (burstconst = 3,
    /// hangconst = 5).
    fn hangover(&mut self, vvad: i16) -> i16 {
        if vvad == 1 {
            self.burstcount = add(self.burstcount, 1);
        } else {
            self.burstcount = 0;
        }

        if self.burstcount >= 3 {
            self.hangcount = 5;
            self.burstcount = 3;
        }

        let mut vad = vvad;
        if self.hangcount >= 0 {
            vad = 1;
            self.hangcount = sub(self.hangcount, 1);
        }
        vad
    }

    /// Clause 3.9 — periodicity updating from the four current LTP
    /// lags (`lags[0]` = first sub-segment).
    fn periodicity_updating(&mut self, lags: &[i16; 4]) {
        let mut lagcount: i16 = 0;
        for &lag in lags.iter() {
            // Search the maximum and minimum of consecutive lags.
            let (minlag, maxlag) = if self.oldlag > lag {
                (lag, self.oldlag)
            } else {
                (self.oldlag, lag)
            };
            // Compute smallag (modulo operation not defined).
            let mut smallag = maxlag;
            for _ in 0..3 {
                if smallag >= minlag {
                    smallag = sub(smallag, minlag);
                }
            }
            // Minimum of smallag and minlag - smallag.
            let temp = sub(minlag, smallag);
            if temp < smallag {
                smallag = temp;
            }
            if smallag < 2 {
                lagcount = add(lagcount, 1);
            }
            // Save the current LTP lag.
            self.oldlag = lag;
        }
        // Update the veryoldlagcount and oldlagcount.
        self.veryoldlagcount = self.oldlagcount;
        self.oldlagcount = lagcount;
    }

    /// Inspect the most recent clause 3.10 tone flag (downlink mode).
    pub fn tone(&self) -> bool {
        self.tone == 1
    }
}

/// Clause 3.3 — predictor values computation: Schur recursion
/// (3.3.1), step-up (3.3.2), autocorrelated predictors (3.3.3).
/// Returns `(rav1[0..=8], normrav1)`.
fn predictor_values(l_av1: &[i32; 9]) -> ([i16; 9], i16) {
    // 3.3.1 — Schur recursion (identical to the RPE-LTP one but on
    // L_av1) producing vpar[1..=8].
    let mut vpar = [0i16; 9];
    'schur: {
        if l_av1[0] == 0 {
            break 'schur;
        }
        let temp = norm(l_av1[0]);
        let mut sacf = [0i16; 9];
        for k in 0..=8 {
            sacf[k] = ((l_av1[k] << (temp as u32)) >> 16) as i16;
        }

        // Initialize array P[..] and K[..] for the recursion.
        let mut k_arr = [0i16; 9];
        for i in 1..=7 {
            k_arr[9 - i] = sacf[i];
        }
        let mut p = [0i16; 9];
        p[..=8].copy_from_slice(&sacf[..=8]);

        // Compute reflection coefficients.
        // The spec's Schur recursion couples the loop index to three
        // arrays at shifted offsets; keep the indexed form.
        #[allow(clippy::needless_range_loop)]
        for n in 1..=8usize {
            if p[0] < abs(p[1]) {
                break 'schur;
            }
            vpar[n] = div(abs(p[1]), p[0]);
            if p[1] > 0 {
                vpar[n] = sub(0, vpar[n]);
            }
            if n == 8 {
                break 'schur;
            }
            // Schur recursion.
            p[0] = add(p[0], mult_r(p[1], vpar[n]));
            for m in 1..=(8 - n) {
                p[m] = add(p[m + 1], mult_r(k_arr[9 - m], vpar[n]));
                k_arr[9 - m] = add(k_arr[9 - m], mult_r(p[m + 1], vpar[n]));
            }
        }
    }

    // 3.3.2 — step-up procedure to obtain the aav1[0..=8].
    let mut l_coef = [0i32; 9];
    let mut l_work = [0i32; 9];
    l_coef[0] = 16384 << 15;
    l_coef[1] = (vpar[1] as i32) << 14;
    for m in 2..=8usize {
        for i in 1..m {
            let temp = (l_coef[m - i] >> 16) as i16; // takes the msb
            l_work[i] = l_add(l_coef[i], l_mult(vpar[m], temp));
        }
        l_coef[1..m].copy_from_slice(&l_work[1..m]);
        l_coef[m] = (vpar[m] as i32) << 14;
    }

    // Keep the aav1[0..=8] on 13 bits for the next clause.
    let mut aav1 = [0i16; 9];
    for i in 0..=8 {
        aav1[i] = (l_coef[i] >> 19) as i16;
    }

    // 3.3.3 — computation of the rav1[0..=8].
    let mut l_work = [0i32; 9];
    for i in 0..=8usize {
        for k in 0..=(8 - i) {
            l_work[i] = l_add(l_work[i], l_mult(aav1[k], aav1[k + i]));
        }
    }

    let normrav1 = if l_work[0] == 0 { 0 } else { norm(l_work[0]) };
    let mut rav1 = [0i16; 9];
    for i in 0..=8 {
        rav1[i] = ((l_work[i] << (normrav1 as u32)) >> 16) as i16;
    }
    (rav1, normrav1)
}

/// Clause 3.10 — information tone detection on the offset-compensated
/// signal frame `sof[0..=159]`. Returns the `tone` flag (0/1).
///
/// A second-order pole-frequency test rejects low-frequency noise
/// resonances (pole below 385 Hz ⇒ noise), then a fourth-order
/// prediction-gain test classifies the frame as a tone when the
/// normalized prediction error is below the threshold (equivalent to
/// a prediction gain above 13,5 dB).
fn tone_detection(sof: &[i16; FRAME_SAMPLES]) -> i16 {
    // 3.10.1 — windowing (Hanning; symmetric table half).
    let mut sofh = [0i16; FRAME_SAMPLES];
    for i in 0..80 {
        sofh[i] = mult_r(sof[i], HANN[i]);
        sofh[159 - i] = mult_r(sof[159 - i], HANN[i]);
    }

    // 3.10.2 — auto-correlation with dynamic scaling (five lags).
    let mut smax: i16 = 0;
    for &v in sofh.iter() {
        let t = abs(v);
        if t > smax {
            smax = t;
        }
    }
    let scalauto: i16 = if smax == 0 {
        0
    } else {
        sub(4, norm((smax as i32) << 16))
    };
    if scalauto > 0 {
        let temp = 16384i16 >> (sub(scalauto, 1) as u32);
        for slot in sofh.iter_mut() {
            *slot = mult_r(*slot, temp);
        }
    }
    let mut l_acfh = [0i32; 5];
    for k in 0..=4 {
        let mut acc: i32 = 0;
        for i in k..FRAME_SAMPLES {
            acc = l_add(acc, l_mult(sofh[i], sofh[i - k]));
        }
        l_acfh[k] = acc;
    }

    // 3.10.3 — reflection coefficients rc[1..=4] (Schur, order 4).
    let mut rc = [0i16; 5];
    'schur: {
        if l_acfh[0] == 0 {
            break 'schur;
        }
        let temp = norm(l_acfh[0]);
        let mut sacf = [0i16; 5];
        for k in 0..=4 {
            sacf[k] = ((l_acfh[k] << (temp as u32)) >> 16) as i16;
        }
        let mut k_arr = [0i16; 5];
        for i in 1..=3 {
            k_arr[5 - i] = sacf[i];
        }
        let mut p = [0i16; 5];
        p[..=4].copy_from_slice(&sacf[..=4]);
        // Index-coupled Schur recursion, as above.
        #[allow(clippy::needless_range_loop)]
        for n in 1..=4usize {
            if p[0] < abs(p[1]) {
                break 'schur;
            }
            rc[n] = div(abs(p[1]), p[0]);
            if p[1] > 0 {
                rc[n] = sub(0, rc[n]);
            }
            if n == 4 {
                break 'schur;
            }
            p[0] = add(p[0], mult_r(p[1], rc[n]));
            for m in 1..=(4 - n) {
                p[m] = add(p[m + 1], mult_r(k_arr[5 - m], rc[n]));
                k_arr[5 - m] = add(k_arr[5 - m], mult_r(p[m + 1], rc[n]));
            }
        }
    }

    // 3.10.4 — filter coefficient calculation a[1..=2].
    let temp = rc[1] >> 2;
    let a1 = add(temp, mult_r(rc[2], temp));
    let a2 = rc[2] >> 2;

    // 3.10.5 — pole frequency test.
    let l_den = l_mult(a1, a1);
    let l_temp = (a2 as i32) << 16;
    let l_num = l_sub(l_temp, l_den);

    // If pole is not complex then exit.
    if l_num <= 0 {
        return 0;
    }
    // If pole frequency is less than 385 Hz then exit.
    if a1 < 0 {
        let temp = (l_den >> 16) as i16;
        let l_den = l_mult(temp, 3189);
        let l_temp = l_sub(l_num, l_den);
        if l_temp < 0 {
            return 0;
        }
    }

    // 3.10.6 — prediction gain test.
    let mut prederr: i16 = 32767;
    for &r in rc.iter().take(5).skip(1) {
        let temp = mult(r, r);
        let temp = sub(32767, temp);
        prederr = mult(prederr, temp);
    }
    let temp = sub(prederr, 1464);
    if temp < 0 {
        1
    } else {
        0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::EncoderState;

    fn tap_for(pcm: &[i16; FRAME_SAMPLES], enc: &mut EncoderState) -> VadTap {
        enc.encode_frame_with_vad_tap(pcm).1
    }

    /// Table 3.1 — reset state values.
    #[test]
    fn reset_state_matches_table_3_1() {
        let v = Vad::new(VadMode::Downlink);
        assert_eq!(v.rvad, [24576, -16384, 4096, 0, 0, 0, 0, 0, 0]);
        assert_eq!(v.normrvad, 7);
        assert_eq!((v.e_thvad, v.m_thvad), (20, 31250));
        assert_eq!((v.burstcount, v.hangcount), (0, -1));
        assert_eq!(v.oldlag, 40);
        assert_eq!(v.tone, 0);
        assert_eq!(v.adaptcount, 0);
    }

    /// Silence at the encoder input drives `acf0` far below `pth`
    /// (clause 3.6 first branch: threshold pinned to `plev`) and the
    /// VAD must report no speech once the initial hangover expired.
    #[test]
    fn silence_is_not_speech() {
        let mut enc = EncoderState::new();
        let mut vad = Vad::new(VadMode::Downlink);
        let quiet = [0i16; FRAME_SAMPLES];
        let mut last = true;
        for _ in 0..20 {
            let tap = tap_for(&quiet, &mut enc);
            last = vad.process_frame(&tap);
        }
        assert!(!last, "silence must yield vad = 0");
        // Threshold pinned at plev by the acf0 < pth branch.
        assert_eq!((vad.e_thvad, vad.m_thvad), (E_PLEV, M_PLEV));
    }

    /// Loud broadband input keeps `pvad` above any adapted threshold:
    /// the VAD must report speech.
    #[test]
    fn loud_noise_burst_is_speech() {
        let mut enc = EncoderState::new();
        let mut vad = Vad::new(VadMode::Downlink);
        // Deterministic loud pseudo-noise, 13-bit aligned.
        let mut state = 0x1234_5678u32;
        for _ in 0..10 {
            let pcm: [i16; FRAME_SAMPLES] = core::array::from_fn(|_| {
                state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                (((state >> 16) as i16) >> 2) & !0b111
            });
            let tap = tap_for(&pcm, &mut enc);
            assert!(vad.process_frame(&tap), "loud noise burst must trip vvad");
        }
    }

    /// Clause 3.8 hangover: after >= 3 speech blocks, the hangover
    /// keeps vad = 1 for 5 further frames when vvad drops, then vad
    /// releases. (hangcount is set to 5 on the burst's third frame
    /// and already decremented once *during* that frame, so 5 —
    /// not 6 — post-burst frames remain.)
    #[test]
    fn hangover_shape_after_speech_burst() {
        let mut vad = Vad::new(VadMode::Uplink);
        // Drive the hangover machine directly.
        for _ in 0..3 {
            assert_eq!(vad.hangover(1), 1);
        }
        // vvad drops; 5 frames stay vad=1 (hangcount 4..=0)…
        for k in 0..5 {
            assert_eq!(vad.hangover(0), 1, "hangover frame {k}");
        }
        // …then release.
        assert_eq!(vad.hangover(0), 0);
    }

    /// A short (< burstconst) vvad spike gets no hangover.
    #[test]
    fn short_burst_gets_no_hangover() {
        let mut vad = Vad::new(VadMode::Uplink);
        // Consume the initial hangcount = -1 (no hangover pending).
        assert_eq!(vad.hangover(1), 1);
        assert_eq!(vad.hangover(1), 1);
        assert_eq!(
            vad.hangover(0),
            0,
            "2-frame burst < burstconst: no hangover"
        );
    }

    /// Clause 3.9/3.5: steady periodic lags across two frames set
    /// `ptch` (>= 4 matching lag pairs over the two frames).
    #[test]
    fn steady_lags_raise_ptch() {
        let mut vad = Vad::new(VadMode::Uplink);
        vad.periodicity_updating(&[60, 60, 60, 60]);
        // The first pair compares against the Table 3.1 oldlag = 40
        // (fold of 60 against 40 leaves 20 >= lthresh), so only the
        // three steady 60/60 pairs count.
        assert_eq!(vad.oldlagcount, 3);
        vad.periodicity_updating(&[60, 60, 60, 60]);
        assert_eq!(add(vad.oldlagcount, vad.veryoldlagcount), 7);
        // 7 >= nthresh = 4 ⇒ ptch would be raised.
        // Multiples of the lag also count (the modulo fold).
        let mut vad2 = Vad::new(VadMode::Uplink);
        vad2.periodicity_updating(&[40, 80, 40, 120]);
        assert_eq!(vad2.oldlagcount, 4, "harmonic lags fold to < lthresh");
    }

    /// Clause 3.10: a pure mid-band tone must set the tone flag; a
    /// broadband noise frame must not.
    #[test]
    fn tone_detection_flags_a_tone_not_noise() {
        // ~1 kHz tone at 8 kHz: sin approximated by a table-free
        // triangle is too harmonic-rich; use a sampled sinusoid.
        let mut sine = [0i16; FRAME_SAMPLES];
        for (i, s) in sine.iter_mut().enumerate() {
            let phase = (i as f64) * 2.0 * std::f64::consts::PI * 1000.0 / 8000.0;
            *s = ((phase.sin() * 8000.0) as i16) & !0b111;
        }
        assert_eq!(tone_detection(&sine), 1, "pure tone must be flagged");

        let mut state = 0x8765_4321u32;
        let noise: [i16; FRAME_SAMPLES] = core::array::from_fn(|_| {
            state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (((state >> 16) as i16) >> 2) & !0b111
        });
        assert_eq!(tone_detection(&noise), 0, "broadband noise is not a tone");
    }

    /// Uplink mode never sets the tone flag (clause 3.10: "In the
    /// uplink VAD tone=0").
    #[test]
    fn uplink_mode_forces_tone_zero() {
        let mut enc = EncoderState::new();
        let mut vad = Vad::new(VadMode::Uplink);
        let mut sine = [0i16; FRAME_SAMPLES];
        for (i, s) in sine.iter_mut().enumerate() {
            let phase = (i as f64) * 2.0 * std::f64::consts::PI * 1000.0 / 8000.0;
            *s = ((phase.sin() * 8000.0) as i16) & !0b111;
        }
        let tap = tap_for(&sine, &mut enc);
        let _ = vad.process_frame(&tap);
        assert!(!vad.tone());
    }
}
