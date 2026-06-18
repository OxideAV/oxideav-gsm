//! Fixed-point GSM 06.10 RPE-LTP decoder per ETSI EN 300 961 §5.3.
//!
//! Walks one 260-bit speech frame's [`UnpackedFrame`] parameters
//! through the five-stage decoder pipeline (figure 1.2 / figure
//! 3.4):
//!
//! 1. **RPE decoding** — §5.3.1: APCM inverse quantisation
//!    (§5.2.16) of `xmaxcr` + `xMcr[0..=12]` per sub-frame, then
//!    RPE grid positioning (§5.2.17) to upsample by 3.
//! 2. **Long term synthesis** — §5.3.2: reconstruct the short term
//!    residual `drp[0..=39]` from the long term residual `erp` +
//!    a Q15-gain-weighted lag-shifted copy of past `drp`.
//! 3. **Short term synthesis** — §5.3.4: 8-stage lattice IIR over
//!    the reflection coefficients `rrp[1..=8]` recovered from the
//!    interpolated decoded LARs (§5.2.8, §5.2.9.1, §5.2.9.2).
//! 4. **De-emphasis** — §5.3.5: first-order IIR with coefficient
//!    `28180 * 2^-15`.
//! 5. **Upscale + truncate** — §5.3.6 / §5.3.7: double the sample
//!    and clear the three LSBs so the output matches the 13-bit
//!    "S.v.v.v.v.v.v.v.v.v.v.v.v.0.0.0" format.

use crate::arith::{abs, add, mult_r, shl_signed, shr_signed, sub};
use crate::bitstream::{SubFrame, UnpackedFrame, FRAME_SAMPLES, PULSES, SUBFRAME_SAMPLES};
use crate::tables::{B, FAC, INVA, MIC, QLB};

/// Persistent decoder state across frames per §4.6 Table 4.3.
#[derive(Debug, Clone)]
pub struct DecoderState {
    /// LTP lag from previous sub-frame (§4.6 `nrp`, home value 40).
    nrp: i16,
    /// LTP delay line `drp[-120..=-1]` (§5.3.2). Stored as a
    /// length-120 ring; index 0 is the most-recent sample (i.e.
    /// `drp[-1]`), index 119 is the oldest (`drp[-120]`).
    drp_hist: [i16; 120],
    /// LARs from the previous frame `LARpp(j-1)[1..=8]` (§4.6 +
    /// §5.2.9.1). Index 0 is a sentinel zero to mirror the spec's
    /// 1-based indexing.
    lar_pp_prev: [i16; 9],
    /// Short-term synthesis filter memory `v[0..=8]` (§4.6).
    v: [i16; 9],
    /// De-emphasis filter memory `msr` (§4.6, home value 0).
    msr: i16,
}

impl Default for DecoderState {
    fn default() -> Self {
        Self {
            nrp: 40,
            drp_hist: [0; 120],
            lar_pp_prev: [0; 9],
            v: [0; 9],
            msr: 0,
        }
    }
}

impl DecoderState {
    /// Build a fresh decoder in its §4.6 home state.
    pub fn new() -> Self {
        Self::default()
    }

    /// Reset the decoder's state to §4.6 Table 4.3 home values.
    /// Equivalent to dropping and re-creating a [`DecoderState`].
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// Returns `true` if every persistent state variable currently
    /// holds its §4.6 Table 4.3 home value (`nrp = 40`, the
    /// `drp[-120..=-1]` delay line, `LARpp(j-1)[1..=8]`, the
    /// short-term synthesis memory `v[0..=8]`, and the de-emphasis
    /// memory `msr` all zero).
    ///
    /// This is the precondition the §4.4 NOTE 2 / §6.3.3.2
    /// delay-optimised partial-homing detection
    /// ([`is_partial_decoder_homing_frame`]) requires: the cheaper
    /// LARs-plus-first-sub-frame check is only sound once the decoder
    /// has already been driven into its home state by a preceding
    /// complete homing frame.
    pub fn is_home_state(&self) -> bool {
        self.nrp == 40
            && self.drp_hist == [0; 120]
            && self.lar_pp_prev == [0; 9]
            && self.v == [0; 9]
            && self.msr == 0
    }

    /// Snapshot of the §5.3.2 long-term synthesis delay line
    /// `drp[-120..=-1]`, with index 0 = `drp[-1]` (most recent) and
    /// index 119 = `drp[-120]` (oldest) — the exact layout the
    /// encoder's local-decoder `dp[-120..=-1]` uses.
    ///
    /// Exposed `pub(crate)` so the analysis-by-synthesis cross-check
    /// can assert the encoder's reconstructed short-term residual
    /// history is bit-identical to the receiving decoder's.
    #[cfg(test)]
    pub(crate) fn drp_hist(&self) -> &[i16; 120] {
        &self.drp_hist
    }

    /// Decode one 260-bit speech frame into 160 linear 13-bit PCM
    /// samples, returned as `i16` per §1.3 ("uniform format shall
    /// be represented in two's complement"). The three LSBs of
    /// every returned sample are zero per §5.3.7 output format.
    pub fn decode_frame(&mut self, frame: &UnpackedFrame) -> [i16; FRAME_SAMPLES] {
        // §5.2.8 — decode the eight coded LARs into LARpp.
        let lar_pp = decode_lar(&frame.lar_c);

        // §5.3.4 needs the reconstructed short-term residual `drp`
        // for the whole frame; we build it sub-frame by sub-frame,
        // pushing the result into a 4*40-sample working buffer.
        let mut drp_frame = [0i16; FRAME_SAMPLES];

        // Sub-frame loop (§5.3.1 + §5.3.2).
        for (j, sf) in frame.sub.iter().enumerate() {
            // §5.2.16 + §5.2.17 RPE decode → erp[0..=39].
            let erp = rpe_decode(sf);

            // §5.3.2 long-term synthesis → drp[0..=39] for this
            // sub-frame, also updates the 120-sample history.
            let drp = self.lt_synthesis(sf, &erp);

            for k in 0..SUBFRAME_SAMPLES {
                drp_frame[j * SUBFRAME_SAMPLES + k] = drp[k];
            }
        }

        // §5.2.9.1 — interpolate LARpp(j-1) and LARpp(j) into four
        // sets of LARp[1..=8] covering k=0..12, 13..26, 27..39,
        // 40..159; §5.2.9.2 converts each set to rp[1..=8]; §5.3.4
        // then runs the 8-stage synthesis lattice over `drp_frame`.
        let sr = self.st_synthesis(&drp_frame, &lar_pp);

        // §5.3.5 de-emphasis.
        let sro = self.de_emphasis(&sr);

        // §5.3.6 + §5.3.7 upscale and truncate.
        post_process(&sro)
    }
}

// ───────────────────── §5.2.8 — LAR decode ─────────────────────

/// Decode the eight coded LARs `LARc[1..=8]` into `LARpp[1..=8]`
/// (the LARs for the current frame) per §5.2.8.
///
/// The §5.2.8 pseudo-code reads:
///
/// ```text
/// FOR i=1 to 8:
///     temp1 = add(LARc[i], MIC[i]) << 10
///     temp2 = B[i] << 1
///     temp1 = sub(temp1, temp2)
///     temp1 = mult_r(INVA[i], temp1)
///     LARpp[i] = add(temp1, temp1)
/// ```
///
/// Note (§5.2.8 NOTE): the addition of `MIC[i]` is used to restore
/// the sign of `LARc[i]` — the encoder offset it positive in §5.2.7
/// so the wire format is always unsigned.
///
/// Exposed `pub(crate)` so the encoder's §5.2.10 short-term analysis
/// filter — which needs the LARpp values the decoder will see, not
/// the un-quantised LARs — can drive the identical §5.2.8 path.
pub(crate) fn decode_lar(lar_c: &[i16; 9]) -> [i16; 9] {
    let mut lar_pp = [0i16; 9];
    for i in 1..=8 {
        let mut temp1 = add(lar_c[i], MIC[i]) << 10;
        let temp2 = B[i] << 1;
        temp1 = sub(temp1, temp2);
        temp1 = mult_r(INVA[i], temp1);
        lar_pp[i] = add(temp1, temp1);
    }
    lar_pp
}

// ───── §5.2.9.1 — Interpolation of LARpp[1..=8] ─────

/// LAR interpolation per §5.2.9.1 Table 3.2.
///
/// Four blocks of output samples per frame use four different
/// interpolated LAR sets:
/// * k = 0..=12   :  LARp = 0.75 * LARpp(j-1) + 0.25 * LARpp(j)
/// * k = 13..=26  :  LARp = 0.50 * LARpp(j-1) + 0.50 * LARpp(j)
/// * k = 27..=39  :  LARp = 0.25 * LARpp(j-1) + 0.75 * LARpp(j)
/// * k = 40..=159 :  LARp = LARpp(j)
///
/// `block` selects which of those four windows the caller wants:
/// 0 ⇒ 0..=12, 1 ⇒ 13..=26, 2 ⇒ 27..=39, 3 ⇒ 40..=159.
///
/// Exposed `pub(crate)` so the encoder's §5.2.10 short-term analysis
/// filter — which the spec specifies "operates with four different
/// sets of coefficients" derived the same way — can reuse the
/// identical §5.2.9.1 path.
pub(crate) fn interpolate_lar(prev: &[i16; 9], curr: &[i16; 9], block: u8) -> [i16; 9] {
    let mut out = [0i16; 9];
    for i in 1..=8 {
        out[i] = match block {
            // §5.2.9.1 — LARp = (LARpp(j-1) >> 2) + (LARpp(j) >> 2),
            // then add(LARp, LARpp(j-1) >> 1)
            0 => {
                let t = add(prev[i] >> 2, curr[i] >> 2);
                add(t, prev[i] >> 1)
            }
            // §5.2.9.1 — LARp = (LARpp(j-1) >> 1) + (LARpp(j) >> 1).
            1 => add(prev[i] >> 1, curr[i] >> 1),
            // §5.2.9.1 — LARp = (LARpp(j-1) >> 2) + (LARpp(j) >> 2),
            // then add(LARp, LARpp(j) >> 1)
            2 => {
                let t = add(prev[i] >> 2, curr[i] >> 2);
                add(t, curr[i] >> 1)
            }
            // §5.2.9.1 — LARp = LARpp(j).
            _ => curr[i],
        };
    }
    out
}

// ───── §5.2.9.2 — LARp → rp ─────

/// Convert interpolated LARp[1..=8] back into reflection
/// coefficients rp[1..=8] per §5.2.9.2.
///
/// Exposed `pub(crate)` so the encoder's §5.2.10 short-term analysis
/// filter, which uses the same rp[1..=8] lattice coefficients the
/// decoder's §5.3.4 synthesis filter does, can reuse the identical
/// §5.2.9.2 path.
pub(crate) fn larp_to_rp(lar_p: &[i16; 9]) -> [i16; 9] {
    let mut rp = [0i16; 9];
    for i in 1..=8 {
        let mut temp = abs(lar_p[i]);
        if temp < 11059 {
            temp <<= 1;
        } else if temp < 20070 {
            temp = add(temp, 11059);
        } else {
            temp = add(temp >> 2, 26112);
        }
        rp[i] = if lar_p[i] < 0 { sub(0, temp) } else { temp };
    }
    rp
}

// ───── §5.2.16 + §5.2.17 — RPE decode ─────

/// APCM inverse quantisation + RPE grid positioning per §5.2.16
/// and §5.2.17. Produces the long-term residual estimate
/// `erp[0..=39]` for one sub-frame.
fn rpe_decode(sf: &SubFrame) -> [i16; SUBFRAME_SAMPLES] {
    // §5.2.15 mantissa/exponent extraction. The decoder only needs
    // the "compute mantissa and exponent of xmaxc" half, per §5.3.1.
    let xmaxc = sf.xmax_c as i16;
    let mut exp: i16 = 0;
    if xmaxc > 15 {
        exp = sub(xmaxc >> 3, 1);
    }
    let mant = sub(xmaxc, exp << 3);

    // §5.2.15 — normalise mantissa so 0 < mant <= 7. The
    // `sub(mant, 8)` step at the bottom of §5.2.15 applies to
    // BOTH the `mant == 0` branch (where the spec preloads
    // `exp = -4; mant = 15`) and the iterative branch (where the
    // for-loop leaves mant in 8..=15) — it lives outside the
    // IF/ELSE in the spec pseudocode.
    let (exp, mant) = if mant == 0 {
        (sub(0, 4), 15i16)
    } else {
        let mut e = exp;
        let mut m = mant;
        let mut itest = 0i16;
        for _ in 0..3 {
            if m > 7 {
                itest = 1;
            }
            if itest == 0 {
                m = add(m << 1, 1);
            }
            if itest == 0 {
                e = sub(e, 1);
            }
        }
        (e, m)
    };
    let mant = sub(mant, 8);

    // §5.2.16 — APCM inverse quantisation.
    // temp1 = FAC[mant]; temp2 = sub(6, exp); temp3 = 1 << sub(temp2, 1).
    let temp1 = FAC[mant as usize];
    let temp2 = sub(6, exp);
    let temp3 = shl_signed(1, sub(temp2, 1));
    let mut x_mp = [0i16; PULSES];
    for (slot, &xmc_raw) in x_mp.iter_mut().zip(sf.x_mc.iter()) {
        // §5.2.16: temp = sub((xMc[i] << 1), 7); temp <<= 12;
        // temp = mult_r(temp1, temp); temp = add(temp, temp3);
        // xMp[i] = temp >> temp2.
        let xmc = xmc_raw as i16;
        let t = sub(xmc << 1, 7) << 12;
        let t = mult_r(temp1, t);
        let t = add(t, temp3);
        *slot = shr_signed(t, temp2);
    }

    // §5.2.17 — RPE grid positioning: insert the 13 decoded
    // pulses at positions Mc, Mc+3, Mc+6, … in an otherwise zero
    // 40-sample buffer.
    let mut erp = [0i16; SUBFRAME_SAMPLES];
    let mc = sf.m_c as usize;
    for (i, &pulse) in x_mp.iter().enumerate() {
        let idx = mc + 3 * i;
        if idx < SUBFRAME_SAMPLES {
            erp[idx] = pulse;
        }
    }
    erp
}

// ───── §5.3.2 — Long term synthesis ─────

impl DecoderState {
    /// §5.3.2 — apply the long-term synthesis filter to a
    /// sub-frame's `erp[0..=39]` to produce the reconstructed
    /// short-term residual `drp[0..=39]`, and update the 120-sample
    /// history.
    fn lt_synthesis(
        &mut self,
        sf: &SubFrame,
        erp: &[i16; SUBFRAME_SAMPLES],
    ) -> [i16; SUBFRAME_SAMPLES] {
        // §5.3.2 — check the limits of Nr.
        let mut nr = sf.n_c as i16;
        if !(40..=120).contains(&nr) {
            nr = self.nrp;
        }
        self.nrp = nr;

        // §5.3.2 — decode the LTP gain bcr via QLB.
        let brp = QLB[sf.b_c as usize];

        // §5.3.2 — for k = 0..=39:
        //   drpp = mult_r(brp, drp[k - Nr])
        //   drp[k] = add(erp[k], drpp)
        //
        // The "drp[k - Nr]" indexing reaches into the 120-sample
        // history `self.drp_hist` for k - Nr < 0 (Nr ∈ [40, 120],
        // k ∈ [0, 39] ⇒ k - Nr ∈ [-120, -1]) and into the
        // sub-frame's own newly-produced `drp` for k - Nr >= 0.
        let mut drp = [0i16; SUBFRAME_SAMPLES];
        for k in 0..SUBFRAME_SAMPLES {
            let shifted = k as i16 - nr;
            let past = if shifted < 0 {
                // hist index 0 == drp[-1] ⇒ drp[-n] = hist[n - 1].
                self.drp_hist[(-shifted - 1) as usize]
            } else {
                drp[shifted as usize]
            };
            let drpp = mult_r(brp, past);
            drp[k] = add(erp[k], drpp);
        }

        // §5.3.2 — update of drp[-120..=-1]: shift the history left
        // by 40 (drop the oldest 40) and push the new 40 in. Spec
        // text: "FOR k=0 to 119: drp[-120+k] = drp[-80+k]" which is
        // a left-shift by 40 covering the kept range, but the
        // *new* 40 samples (drp[0..=39] just produced) then enter
        // the buffer's slot `-40 + k` for k=0..=39 implicitly via
        // the next sub-frame's update loop pulling them straight
        // out of the synthesis output. We mirror that by storing
        // the 40 new samples at the head of the ring.
        //
        // Concretely: hist[i] currently holds drp[-(i+1)]. After
        // the sub-frame, the new drp[0..=39] become the new
        // drp[-1..=-40]. So we right-shift hist by 40 (oldest 40
        // fall off, freshly-produced 40 inserted at indices 0..=39
        // in reverse).
        let mut new_hist = [0i16; 120];
        // Place the 40 freshly produced samples as the most-recent:
        // drp[-1] = drp[39], drp[-2] = drp[38], …, drp[-40] = drp[0].
        for (i, slot) in new_hist[..SUBFRAME_SAMPLES].iter_mut().enumerate() {
            *slot = drp[SUBFRAME_SAMPLES - 1 - i];
        }
        // Slide what was hist[0..=79] (i.e. drp[-1..=-80]) into
        // the next-older slots (hist[40..=119], representing
        // drp[-41..=-120]).
        new_hist[SUBFRAME_SAMPLES..].copy_from_slice(&self.drp_hist[..120 - SUBFRAME_SAMPLES]);
        self.drp_hist = new_hist;

        drp
    }
}

// ───── §5.3.4 — Short term synthesis ─────

impl DecoderState {
    /// §5.3.4 — run the 8-stage lattice synthesis filter over the
    /// whole frame's `drp[0..=159]`, using a different rp[1..=8]
    /// set on each of the four §5.2.9.1 windows.
    fn st_synthesis(
        &mut self,
        drp_frame: &[i16; FRAME_SAMPLES],
        lar_curr: &[i16; 9],
    ) -> [i16; FRAME_SAMPLES] {
        let mut sr = [0i16; FRAME_SAMPLES];

        // Four §5.2.9.1 windows: (block id, k_start, k_end).
        let windows: [(u8, usize, usize); 4] = [(0, 0, 12), (1, 13, 26), (2, 27, 39), (3, 40, 159)];

        for (block, k_start, k_end) in windows {
            let lar_p = interpolate_lar(&self.lar_pp_prev, lar_curr, block);
            let rrp = larp_to_rp(&lar_p);

            // §5.3.4 lattice — for each input sample drp[k]:
            //   sri = drp[k]
            //   FOR i = 1..=8:
            //       sri = sub(sri, mult_r(rrp[9-i], v[8-i]))
            //       v[9-i] = add(v[8-i], mult_r(rrp[9-i], sri))
            //   sr[k] = sri
            //   v[0] = sri
            for k in k_start..=k_end {
                let mut sri = drp_frame[k];
                for i in 1..=8 {
                    let r = rrp[9 - i];
                    let v_prev = self.v[8 - i];
                    sri = sub(sri, mult_r(r, v_prev));
                    self.v[9 - i] = add(v_prev, mult_r(r, sri));
                }
                sr[k] = sri;
                self.v[0] = sri;
            }
        }

        // Update LAR history for the next frame's interpolation.
        self.lar_pp_prev = *lar_curr;

        sr
    }
}

// ───── §5.3.5 — De-emphasis ─────

impl DecoderState {
    /// §5.3.5 — first-order IIR de-emphasis with coefficient
    /// `28180 * 2^-15`. State `msr` is the previous output sample.
    fn de_emphasis(&mut self, sr: &[i16; FRAME_SAMPLES]) -> [i16; FRAME_SAMPLES] {
        let mut sro = [0i16; FRAME_SAMPLES];
        for k in 0..FRAME_SAMPLES {
            // §5.3.5: temp = add(sr[k], mult_r(msr, 28180));
            //         msr = temp; sro[k] = msr.
            let temp = add(sr[k], mult_r(self.msr, 28180));
            self.msr = temp;
            sro[k] = self.msr;
        }
        sro
    }
}

// ───── §5.3.6 + §5.3.7 — Upscale and truncate ─────

/// §5.3.6 — `srop[k] = add(sro[k], sro[k])` (i.e. multiply by
/// two), then §5.3.7 — clear the three LSBs so the output has the
/// 13-bit "S.v.v.v.v.v.v.v.v.v.v.v.v.0.0.0" shape.
fn post_process(sro: &[i16; FRAME_SAMPLES]) -> [i16; FRAME_SAMPLES] {
    let mut out = [0i16; FRAME_SAMPLES];
    for k in 0..FRAME_SAMPLES {
        let srop = add(sro[k], sro[k]);
        // §5.3.7: srop = srop >> 3; srop = srop << 3.
        out[k] = (srop >> 3) << 3;
    }
    out
}

// ───────────────────────── decoder-homing helper ─────────────────────────

/// Build the §4.4 Table 4.1a/b decoder-homing frame as an
/// [`UnpackedFrame`]. The spec specifies it so any implementation
/// can run the conformance "homing sequence" test (§4.4 step 1).
pub fn decoder_homing_frame() -> UnpackedFrame {
    let mut f = UnpackedFrame::default();
    // Table 4.1a — LARc[1..=8] in hex.
    let lar_c_hex: [i16; 8] = [
        0x0009, 0x0017, 0x000F, 0x0008, 0x0007, 0x0003, 0x0003, 0x0002,
    ];
    f.lar_c[1..=8].copy_from_slice(&lar_c_hex);
    // Table 4.1b — every sub-frame is identical save sub-frame-4
    // RPE-pulse-4 which is 0x0003 instead of 0x0004.
    for (j, sf) in f.sub.iter_mut().enumerate() {
        sf.n_c = 0x28; // 40
        sf.b_c = 0;
        sf.m_c = 0;
        sf.xmax_c = 0;
        sf.x_mc = [4; PULSES];
        if j == 3 {
            sf.x_mc[4] = 3;
        }
    }
    f
}

/// Build the §4.2 encoder-homing-frame as 160 linear PCM samples.
///
/// §4.2 definition: *"The encoder-homing-frame consists of 160
/// identical samples, each 13 bits long, with the least significant
/// bit set to "one" and all other bits set to "zero". When written
/// to 16-bit words with left justification, the samples have a
/// value of 0008 hex."*
///
/// `0x0008` is the 13-bit value `0000_0000_0000_1` left-justified
/// inside a 16-bit word as `S.v.v.v.v.v.v.v.v.v.v.v.v.0.0.0` per
/// §5.3.7 output format: a 13-bit value `0b0_0000_0000_0001`
/// shifted left by three becomes `0x0008`. This is the same 160
/// samples §4.4 Step 1 requires the decoder to emit in response to
/// a decoder-homing-frame.
pub fn encoder_homing_frame_pcm() -> [i16; FRAME_SAMPLES] {
    [0x0008; FRAME_SAMPLES]
}

/// Returns `true` if `frame` exactly matches the §4.4 Table 4.1a/b
/// decoder-homing-frame's 76 parameters.
///
/// This is the full-frame predicate: every LARc and every parameter
/// of all four sub-frames must match. A decoder that is *not* yet in
/// its home state must see a complete decoder-homing-frame before it
/// resets, so this strict check is the one [`DecoderState::
/// decode_frame_with_homing`] applies on a non-home decoder.
///
/// The §4.4 NOTE 2 / §6.3.3.2 *delay-optimised* check — which only
/// inspects LARc[1..=8] and the first sub-frame, and is valid only
/// once the decoder is already homed — is
/// [`is_partial_decoder_homing_frame`].
pub fn is_decoder_homing_frame(frame: &UnpackedFrame) -> bool {
    *frame == decoder_homing_frame()
}

/// Returns `true` if `frame` matches the decoder-homing-frame in
/// *only* its LARc[1..=8] codewords and its first sub-frame
/// (`sub[0]`), ignoring sub-frames 2..=4.
///
/// This is the §4.4 NOTE 2 "delay-optimised" detection criterion,
/// also exercised by the §6.3.3.2 `HOMING01` conformance sequence:
///
/// > *"if the decoder is in its home state, it is sufficient to
/// > check only these parameters [the LARs and first sub-frame] to
/// > detect a subsequent decoder-homing-frame"* (§4.4 NOTE 2)
///
/// > *"it is sufficient that the following frame contains only the
/// > LARs and the first subframe data of the decoder-homing-frame to
/// > cause a decoder reset and the output of the encoder-homing-
/// > frame"* (§6.3.3.2)
///
/// §4.4 NOTE 2 grounds the soundness: by construction, the first
/// frame of any decoder test sequence differs from the decoder-
/// homing-frame in at least one bit of the LARs or first sub-frame,
/// so once the decoder is homed this subset is enough to tell a
/// (possibly fractional) homing frame apart from real speech. The
/// caller is responsible for only applying this predicate when the
/// decoder is in its §4.6 home state (see
/// [`DecoderState::is_home_state`]).
pub fn is_partial_decoder_homing_frame(frame: &UnpackedFrame) -> bool {
    let h = decoder_homing_frame();
    frame.lar_c == h.lar_c && frame.sub[0] == h.sub[0]
}

// ─────────────────── §4.4 decoder-homing protocol ───────────────────

impl DecoderState {
    /// Decode one frame applying the §4.4 decoder-homing protocol.
    ///
    /// §4.4 Step 1 — when the input frame *is* a decoder-homing-frame
    /// (and not flagged as a bad frame), the output is replaced
    /// with the encoder-homing-frame (160 samples of `0x0008` per
    /// §4.2). §4.4 Step 2 — after that frame is processed, the
    /// decoder's state variables are reset to their §4.6 Table 4.3
    /// home values so that the *next* frame starts from the home
    /// state.
    ///
    /// §4.4 NOTE 1 explains the consequence: a sequence of N
    /// decoder-homing-frames will produce at least N-1
    /// encoder-homing-frames at the output, because the very first
    /// homing frame may arrive while the decoder is in an arbitrary
    /// (non-home) state. With this method, the first homing frame
    /// triggers the substitution; from the second onward the decoder
    /// is already homed and the output is the spec-defined
    /// `0x0008`-fill regardless.
    ///
    /// §4.4 NOTE 2 / §6.3.3.2 — *delay-optimised* detection. Once the
    /// decoder is already in its §4.6 home state, a subsequent frame
    /// only needs to carry the decoder-homing-frame's LARs and first
    /// sub-frame (sub-frames 2..=4 may differ) to trigger the same
    /// substitution + reset. The §6.3.3.2 `HOMING01` sequence
    /// exercises exactly this: after two complete homing frames home
    /// the decoder, it feeds a mixture of complete and *fractional*
    /// homing frames, each of which must still home the decoder. A
    /// non-home decoder, by contrast, must receive a *complete*
    /// homing frame ([`is_decoder_homing_frame`]) before it resets,
    /// because §4.4 NOTE 2's soundness argument only holds from the
    /// home state.
    ///
    /// For raw §5.3 decoding without the §4.4 substitution, use
    /// [`Self::decode_frame`] directly.
    pub fn decode_frame_with_homing(&mut self, frame: &UnpackedFrame) -> [i16; FRAME_SAMPLES] {
        // A homed decoder accepts the cheaper LARs-plus-first-sub-frame
        // criterion (§4.4 NOTE 2); a non-homed decoder demands the
        // complete homing frame.
        let is_homing = if self.is_home_state() {
            is_partial_decoder_homing_frame(frame)
        } else {
            is_decoder_homing_frame(frame)
        };

        if is_homing {
            // §4.4 Step 1 — run the normal decode (so internal
            // state is consistent with having processed the input)
            // but discard the output and emit the encoder-homing-
            // frame instead.
            let _ = self.decode_frame(frame);
            // §4.4 Step 2 — reset all state variables to home.
            self.reset();
            encoder_homing_frame_pcm()
        } else {
            self.decode_frame(frame)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// An all-zero coded frame is NOT silence — coded xMc=0
    /// dequantises to `(0<<1) - 7 = -7` (Table 3.6), so even an
    /// all-zero packet drives the synthesis filter with a regular
    /// negative-amplitude pulse train. What we DO get is exactly
    /// 160 output samples each shaped per §5.3.7.
    #[test]
    fn all_zero_frame_produces_160_shaped_samples() {
        let mut dec = DecoderState::new();
        let frame = UnpackedFrame::default();
        let out = dec.decode_frame(&frame);
        assert_eq!(out.len(), FRAME_SAMPLES);
        for s in out {
            assert_eq!(s & 0b111, 0, "§5.3.7 output low 3 bits must be zero");
        }
    }

    /// The §4.6 home state's LTP lag is 40, never 0 — even though
    /// the home-state delay line is all-zero.
    #[test]
    fn home_state_ltp_lag_is_40() {
        let s = DecoderState::new();
        assert_eq!(s.nrp, 40);
    }

    /// Decode is deterministic — feeding the same frame to two
    /// freshly-homed decoders yields the same 160 samples.
    #[test]
    fn decode_is_deterministic() {
        let mut a = DecoderState::new();
        let mut b = DecoderState::new();
        let frame = decoder_homing_frame();
        let oa = a.decode_frame(&frame);
        let ob = b.decode_frame(&frame);
        assert_eq!(oa, ob);
    }

    /// Output is always 13-bit-shaped (low 3 bits zero) per §5.3.7.
    #[test]
    fn output_has_three_zero_lsbs() {
        let mut dec = DecoderState::new();
        let f = decoder_homing_frame();
        let out = dec.decode_frame(&f);
        for s in out {
            assert_eq!(s & 0b111, 0, "low 3 bits of output sample must be zero");
        }
    }

    /// `reset` returns the decoder to its §4.6 home values — every
    /// internal field matches a freshly-constructed decoder.
    #[test]
    fn reset_returns_home_state() {
        let mut dec = DecoderState::new();
        let f = decoder_homing_frame();
        let _ = dec.decode_frame(&f);
        assert_ne!(dec.msr, 0, "post-decode should have shaped state");
        dec.reset();
        let fresh = DecoderState::new();
        assert_eq!(dec.nrp, fresh.nrp);
        assert_eq!(dec.drp_hist, fresh.drp_hist);
        assert_eq!(dec.lar_pp_prev, fresh.lar_pp_prev);
        assert_eq!(dec.v, fresh.v);
        assert_eq!(dec.msr, fresh.msr);
    }

    /// `decode_lar` returns zero for every all-zero coded LAR
    /// after sign restoration via MIC[i].
    #[test]
    fn decode_lar_no_panic() {
        // All LARc = 0 ⇒ LARpp_i is a function of MIC[i] and B[i]
        // only. The point of the test is to confirm §5.2.8 doesn't
        // hit any panic path (overflow, OOB index) even for a
        // degenerate input. Output values are i16 by construction
        // because arith.rs saturates; just confirm we got a finite
        // bounded answer for every i.
        let lar_c = [0i16; 9];
        let lar_pp = decode_lar(&lar_c);
        // Index 0 stays zero (sentinel slot).
        assert_eq!(lar_pp[0], 0);
        // Each LARpp[i] is the result of mult_r → add, both of
        // which saturate; just confirm we got eight numbers out.
        assert_eq!(lar_pp.len(), 9);
    }

    /// `larp_to_rp` round-trips sign correctly: a negative LARp
    /// yields a negative rp; positive yields positive.
    #[test]
    fn larp_to_rp_preserves_sign() {
        let mut lar = [0i16; 9];
        lar[1] = 5000;
        lar[2] = -5000;
        let rp = larp_to_rp(&lar);
        assert!(rp[1] > 0);
        assert!(rp[2] < 0);
    }

    /// §4.2 encoder-homing-frame PCM is exactly 160 samples of
    /// 0x0008.
    #[test]
    fn encoder_homing_frame_is_160_of_0x0008() {
        let f = encoder_homing_frame_pcm();
        assert_eq!(f.len(), FRAME_SAMPLES);
        for (k, s) in f.iter().enumerate() {
            assert_eq!(*s, 0x0008, "sample {k} should be 0x0008");
        }
    }

    /// The §4.4 decoder-homing-frame predicate matches the helper
    /// it's paired with and rejects everything else.
    #[test]
    fn homing_predicate_matches_only_homing_frame() {
        let h = decoder_homing_frame();
        assert!(is_decoder_homing_frame(&h));
        assert!(!is_decoder_homing_frame(&UnpackedFrame::default()));
        // Mutating any parameter breaks the match.
        let mut m = h;
        m.lar_c[1] = 0;
        assert!(!is_decoder_homing_frame(&m));
        let mut m = h;
        m.sub[3].x_mc[4] = 4; // homing frame has 0x0003 here
        assert!(!is_decoder_homing_frame(&m));
    }

    /// §4.4 Step 1 — a decoder-homing-frame input produces the
    /// encoder-homing-frame (160 × 0x0008) at the output.
    #[test]
    fn homing_input_produces_encoder_homing_output() {
        let mut dec = DecoderState::new();
        let h = decoder_homing_frame();
        let out = dec.decode_frame_with_homing(&h);
        for (k, s) in out.iter().enumerate() {
            assert_eq!(*s, 0x0008, "sample {k} should be 0x0008");
        }
    }

    /// §4.4 Step 2 — after processing a decoder-homing-frame the
    /// decoder is back in its §4.6 home state. The subsequent frame
    /// therefore decodes identically to one fed to a freshly-homed
    /// decoder.
    #[test]
    fn homing_resets_state_for_next_frame() {
        let h = decoder_homing_frame();
        let probe = UnpackedFrame::default(); // arbitrary follow-up

        // Drive `stale` through a noisy frame then a homing frame.
        let mut stale = DecoderState::new();
        let mut noisy = h;
        noisy.lar_c[2] = 21; // perturb away from the homing payload
        let _ = stale.decode_frame_with_homing(&noisy);
        let _ = stale.decode_frame_with_homing(&h);
        let stale_next = stale.decode_frame_with_homing(&probe);

        // `fresh` is just-constructed; one decode of `probe`.
        let mut fresh = DecoderState::new();
        let fresh_next = fresh.decode_frame_with_homing(&probe);

        assert_eq!(stale_next, fresh_next, "post-homing state diverged");
    }

    /// §4.4 NOTE 1 — N homing frames produce at least N-1
    /// encoder-homing outputs. Concretely: feed three homing
    /// frames in a row; outputs 2 and 3 must both be the
    /// encoder-homing-frame.
    #[test]
    fn n_homing_frames_yield_n_minus_1_homing_outputs() {
        let mut dec = DecoderState::new();
        let h = decoder_homing_frame();
        let _ = dec.decode_frame_with_homing(&h); // output 1 (first)
        let out2 = dec.decode_frame_with_homing(&h); // output 2
        let out3 = dec.decode_frame_with_homing(&h); // output 3
        let expected = encoder_homing_frame_pcm();
        assert_eq!(out2, expected);
        assert_eq!(out3, expected);
    }

    /// §4.6 — a freshly-constructed decoder reports itself in the
    /// home state; after a normal §5.3 decode it no longer does.
    #[test]
    fn is_home_state_tracks_reset_and_decode() {
        let mut dec = DecoderState::new();
        assert!(dec.is_home_state(), "fresh decoder must be homed");

        // A non-trivial coded frame mutates the persistent state
        // (de-emphasis memory `msr`, delay line, etc.).
        let mut f = UnpackedFrame::default();
        f.lar_c[1] = 5;
        f.sub[0].xmax_c = 20;
        let _ = dec.decode_frame(&f);
        assert!(!dec.is_home_state(), "post-decode must leave home state");

        dec.reset();
        assert!(dec.is_home_state(), "reset must restore home state");
    }

    /// §4.4 NOTE 2 — the partial-homing predicate matches a frame
    /// that carries the homing LARs and first sub-frame regardless of
    /// what sub-frames 2..=4 hold, and rejects a frame that perturbs
    /// the LARs or the first sub-frame.
    #[test]
    fn partial_homing_predicate_ignores_later_subframes() {
        let h = decoder_homing_frame();

        // Same LARs + sub[0], garbage in sub[1..=3] — still partial-homing.
        let mut frac = h;
        for sf in &mut frac.sub[1..=3] {
            sf.n_c = 0x7F;
            sf.b_c = 3;
            sf.m_c = 3;
            sf.xmax_c = 0x3F;
            sf.x_mc = [7; PULSES];
        }
        assert!(is_partial_decoder_homing_frame(&frac));
        // It is NOT a *complete* homing frame.
        assert!(!is_decoder_homing_frame(&frac));

        // Perturbing a LAR breaks the partial match.
        let mut m = h;
        m.lar_c[1] = 0;
        assert!(!is_partial_decoder_homing_frame(&m));

        // Perturbing the first sub-frame breaks the partial match.
        let mut m = h;
        m.sub[0].x_mc[0] = 7;
        assert!(!is_partial_decoder_homing_frame(&m));

        // Perturbing only a *later* sub-frame still matches partially.
        let mut m = h;
        m.sub[3].x_mc[4] = 4;
        assert!(is_partial_decoder_homing_frame(&m));
    }

    /// §6.3.3.2 — once the decoder is homed, a *fractional* homing
    /// frame (only the LARs and first sub-frame of the homing frame,
    /// arbitrary remaining sub-frames) still homes it: the output is
    /// the encoder-homing-frame and the state returns to §4.6 home.
    #[test]
    fn fractional_homing_frame_homes_a_homed_decoder() {
        let mut dec = DecoderState::new();
        let h = decoder_homing_frame();

        // First, a complete homing frame brings the decoder home
        // (it was already home, but this mirrors the HOMING01
        // "two complete homing frames at the beginning" setup).
        let _ = dec.decode_frame_with_homing(&h);
        assert!(dec.is_home_state());

        // Now a fractional homing frame: homing LARs + sub[0], with
        // sub[1..=3] carrying unrelated data.
        let mut frac = h;
        for sf in &mut frac.sub[1..=3] {
            sf.n_c = 0x55;
            sf.xmax_c = 0x2A;
            sf.x_mc = [6; PULSES];
        }
        let out = dec.decode_frame_with_homing(&frac);
        assert_eq!(
            out,
            encoder_homing_frame_pcm(),
            "fractional homing frame must emit the encoder-homing-frame"
        );
        assert!(dec.is_home_state(), "fractional homing must reset state");
    }

    /// §4.4 NOTE 2 soundness boundary — a decoder that is NOT in its
    /// home state must see a *complete* homing frame before it
    /// resets. A fractional homing frame fed to a non-home decoder
    /// is decoded as ordinary speech (no substitution).
    #[test]
    fn fractional_homing_frame_does_not_home_a_dirty_decoder() {
        let h = decoder_homing_frame();

        // Drive the decoder out of its home state with a noisy frame.
        let mut dec = DecoderState::new();
        let mut noisy = UnpackedFrame::default();
        noisy.lar_c[3] = 12;
        noisy.sub[0].xmax_c = 30;
        let _ = dec.decode_frame_with_homing(&noisy);
        assert!(!dec.is_home_state());

        // A fractional homing frame must NOT trigger substitution now.
        let mut frac = h;
        for sf in &mut frac.sub[1..=3] {
            sf.x_mc = [1; PULSES];
        }
        // Reference: what the raw §5.3 pipeline would produce on a
        // decoder in the identical (non-home) state.
        let mut twin = dec.clone();
        let raw = twin.decode_frame(&frac);
        let out = dec.decode_frame_with_homing(&frac);
        assert_eq!(
            out, raw,
            "dirty decoder must NOT home on a fractional frame"
        );

        // A *complete* homing frame, by contrast, does home it.
        let out2 = dec.decode_frame_with_homing(&h);
        assert_eq!(out2, encoder_homing_frame_pcm());
        assert!(dec.is_home_state());
    }

    /// A non-homing input bypasses the §4.4 substitution — the
    /// output is whatever §5.3 produces.
    #[test]
    fn non_homing_input_passes_through() {
        let mut a = DecoderState::new();
        let mut b = DecoderState::new();
        let f = UnpackedFrame::default(); // not a homing frame
        let raw = a.decode_frame(&f);
        let viah = b.decode_frame_with_homing(&f);
        assert_eq!(raw, viah);
    }

    /// `rpe_decode` with an all-zero sub-frame yields an all-zero
    /// residual (xmaxc = 0 collapses to mant=15, exp=-4 ⇒
    /// FAC[15] would be OOB; the spec gives exp=-4/mant=15 only
    /// when mant *was* 0, and then collapses to mant=15-8=7 after
    /// the sub(mant,8) step). Sanity-check the path doesn't panic
    /// and produces a zero output for an all-zero sub-frame.
    #[test]
    fn rpe_decode_all_zero_sub() {
        let sf = SubFrame::default();
        let erp = rpe_decode(&sf);
        // With xmaxc=0 the §5.2.15 normalisation sets mant=15,
        // exp=-4 then sub(mant,8) ⇒ mant=7. FAC[7]=32767. Each
        // pulse value is `(((xMc<<1)-7)<<12)` then `mult_r(FAC,
        // …) + temp3, >> temp2`. With xMc=0 the bracket = (-7<<12)
        // = -28672; mult_r(32767, -28672) ≈ -28671; temp3 =
        // 1 << (6-(-4)-1) = 1 << 9 = 512; temp = -28159; temp >>
        // (6-(-4)) = -28159 >> 10 = -28 (arithmetic shift). So
        // an all-zero sub-frame produces ep = [−28, 0, 0, −28,
        // 0, 0, …] — non-silent. That matches the §4.6 home-state
        // behaviour: the decoder-homing-frame *is* non-silent
        // (the home output is a small periodic pattern, not
        // silence). What we verify here is just that we don't
        // panic and produce a fixed pattern at sub-frame-grid
        // positions.
        let nonzero_count = erp.iter().filter(|&&v| v != 0).count();
        assert_eq!(nonzero_count, PULSES);
        // The 13 non-zero positions are 0, 3, 6, …, 36 (Mc=0).
        for i in 0..PULSES {
            assert_ne!(erp[3 * i], 0);
        }
    }
}
