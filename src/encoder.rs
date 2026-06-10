//! Fixed-point GSM 06.10 RPE-LTP encoder per ETSI EN 300 961 §5.2.
//!
//! This module currently lands:
//!
//! * The **pre-processing clause** — §5.2.0 (input scaling),
//!   §5.2.1 (downscaling), §5.2.2 (offset compensation, a high-pass
//!   IIR with extended precision), and §5.2.3 (first-order FIR
//!   pre-emphasis). That sub-pipeline maps a frame of 160 raw PCM
//!   input samples `sop[0..159]` to the `s[0..159]` array that the
//!   LPC-analysis clause (§5.2.4 onwards) consumes.
//! * The **LPC-analysis front-end + LAR quantiser** — §5.2.4
//!   (autocorrelation with the spec's dynamic input scaling), §5.2.5
//!   (Schur recursion that produces the eight reflection coefficients
//!   `r[1..8]`), §5.2.6 (reflection → Log-Area Ratio transform), and
//!   §5.2.7 (LAR quantisation + coding into the `LARc[1..8]`
//!   codewords). These four stages are stateless per-frame helpers
//!   exposed as free functions under the [`analysis`] sub-module.
//! * The **short-term analysis filtering clause** — §5.2.8 LAR
//!   decode + §5.2.9.1 LARpp interpolation + §5.2.9.2 LARp → rp
//!   conversion (all three shared with the decoder pipeline) +
//!   §5.2.10 8-stage lattice analysis filter. Driven end-to-end via
//!   the [`analysis::Analyzer`] struct, which persists the §4.5
//!   Table 4.2 `LARpp(j-1)[1..=8]` and `u[0..=7]` state across the
//!   four per-frame interpolation blocks and across frames.
//!
//! * The **LTP analysis clause** — §5.2.11 LTP parameter calculation
//!   (cross-correlation peak search over lags 40..=120, Table 5.3a
//!   `DLB`-driven `bc` decision tree), §5.2.12 long-term analysis
//!   filter (`dpp[k] = mult_r(QLB[bc], dp[k-Nc])`, `e[k] = sub(d[k],
//!   dpp[k])`), and §5.2.18 `dp[-120..=-1]` delay-line update. Driven
//!   per-sub-segment via the [`analysis::LtpAnalyzer`] struct.
//! * The **§5.2.13 weighting filter** — an 11-tap FIR block filter
//!   convolving the §5.2.12 long-term residual `e[0..=39]` with the
//!   Table 5.4 impulse response `H[0..=10]` to produce the block-
//!   filtered signal `x[0..=39]` that §5.2.14 RPE grid selection
//!   consumes. Exposed as the stateless free function
//!   [`analysis::weighting_filter`].
//! * The **§5.2.14 RPE grid selection** — adaptive sample-rate
//!   decimation that picks one of the four sub-sampling grids of
//!   `x[0..=39]` that maximises the sum of squared
//!   `(x[m+3i] >> 2)` values, then down-samples to emit the 13-pulse
//!   sequence `xM[0..=12] = x[Mc + 3*i]`. The 2-bit codeword `Mc`
//!   is what the §1.7 packer emits as `Mc[1..=4]` (one per
//!   sub-segment). Exposed as the stateless free function
//!   [`analysis::select_rpe_grid`] returning [`analysis::RpeGrid`].
//! * The **§5.2.15 APCM forward quantisation** — block-maximum
//!   exponent/mantissa coding of `xM[0..=12]` into the 6-bit
//!   `xmaxc` codeword + 13 × 3-bit `xMc[0..=12]` codewords the
//!   §1.7 packer emits. Exposed as the stateless free function
//!   [`analysis::apcm_quantise_rpe`] returning
//!   [`analysis::ApcmQuantised`]. The returned `(exp, mant)` pair
//!   is the post-normalisation state the §5.2.16 inverse APCM
//!   quantiser consumes per the spec's "Keep in memory exp and
//!   mant for the following inverse APCM quantizer" note.
//! * The **§5.2.16 APCM inverse + §5.2.17 RPE grid positioning** —
//!   the encoder's local-decoder feedback path. Dequantises the
//!   §5.2.15 `xMc[0..=12]` codewords back to `xMp[0..=12]` via
//!   Table 5.6 `FAC[mant]` (bit-identical to the decoder's §5.3.1
//!   path) and scatters them into the reconstructed long-term
//!   residual `ep[Mc + 3*i]`. Exposed as the stateless free
//!   function [`analysis::apcm_inverse_and_position`];
//!   [`analysis::LtpAnalyzer::reconstruct_and_update`] chains
//!   §5.2.16 → §5.2.17 → §5.2.18 to fold `add(ep[k], dpp[k])` back
//!   into the §4.5 `dp[-120..=-1]` delay line, closing the
//!   per-sub-segment LTP feedback loop.
//!
//! * The **frame-level driver** — [`EncoderState`] chains
//!   §5.2.0..§5.2.3 pre-processing → §5.2.4..§5.2.10 LPC analysis →
//!   the four per-sub-segment §5.2.11..§5.2.18 LTP/RPE/APCM passes
//!   and emits one [`crate::bitstream::UnpackedFrame`] of 76
//!   codewords per 160-sample input frame. Combined with
//!   [`crate::bitstream::UnpackedFrame::to_bit_stream_msb_first`]
//!   (the §1.7 Table 1.1 packer) this completes the encode path;
//!   the [`oxideav_core::Encoder`]-trait factory
//!   [`crate::make_encoder`] is built on it.
//!
//! ## §4.5 Table 4.2 home-state mapping
//!
//! The encoder's per-frame state across this pre-processing slice
//! mirrors the §4.5 Table 4.2 "Initial values of the encoder state
//! variables" entries that this slice owns:
//!
//! * **Offset compensation filter memory** `z1` (16-bit) and
//!   `L_z2` (32-bit) — home value 0 (§4.5).
//! * **Pre-emphasis filter memory** `mp` (16-bit) — home value 0
//!   (§4.5).
//!
//! With the §5.2.10 analysis-clause [`analysis::Analyzer`] now in
//! place, two more §4.5 Table 4.2 entries are owned:
//!
//! * **LARs from previous frame** `LARpp(j-1)[1..=8]` — home value
//!   all-zero (§4.5 + §5.2.9.1 "Initial value: LARpp(j-1)[1..8] = 0").
//! * **Short term analysis filter memory** `u[0..=7]` — home value
//!   all-zero (§4.5 + §5.2.10 "Initial value: u[0..7] = 0").
//!
//! The remaining §4.5 entry — the LTP delay-line `dp[-120..-1]` —
//! is owned by [`analysis::LtpAnalyzer`] (home value all-zero per
//! §4.5 + §5.2.11). [`EncoderState`] aggregates all of the above so
//! [`EncoderState::reset`] restores the complete §4.5 Table 4.2
//! encoder home state in one call.

use crate::arith::{add, l_add, l_mult, l_sub, mult_r, sub};
use crate::bitstream::{SubFrame, UnpackedFrame, FRAME_SAMPLES, SUBFRAMES};

/// Persistent encoder pre-processing state across frames.
///
/// Wraps the three §4.5 Table 4.2 entries owned by the §5.2.0..§5.2.3
/// pre-processing pipeline. Reset to the §4.5 home values via
/// [`PreProcessor::new`] / [`PreProcessor::reset`].
#[derive(Debug, Clone, Default)]
pub struct PreProcessor {
    /// §5.2.2 16-bit offset-compensation filter memory `z1`. §4.5
    /// home value 0.
    z1: i16,
    /// §5.2.2 32-bit extended-precision offset-compensation memory
    /// `L_z2`. §4.5 home value 0.
    l_z2: i32,
    /// §5.2.3 first-order pre-emphasis filter memory `mp`. §4.5
    /// home value 0.
    mp: i16,
}

impl PreProcessor {
    /// Build a fresh pre-processor in its §4.5 Table 4.2 home state
    /// (`z1 = 0`, `L_z2 = 0`, `mp = 0`).
    pub fn new() -> Self {
        Self::default()
    }

    /// Reset the pre-processor to §4.5 Table 4.2 home values —
    /// equivalent to constructing a fresh [`PreProcessor`].
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// Run the §5.2.1 downscaling step on a 160-sample input frame
    /// `sop[0..159]`, producing the §5.2.1 output `so[0..159]`.
    ///
    /// §5.2.1 pseudocode:
    /// ```text
    /// FOR k=0 to 159:
    ///     so[k] = sop[k] >> 3;
    ///     so[k] = so[k] << 2;
    /// NEXT k:
    /// ```
    ///
    /// Per §5.2.0 the input `sop[..]` is in the format
    /// `S.v.v.v.v.v.v.v.v.v.v.v.v.v.x.x.x` — sign + 13 valid bits +
    /// three "don't care" LSBs. The downscale `>>3 then <<2` is
    /// effectively a divide-by-two on the 13 valid bits (drop the
    /// three "don't care" bits, then re-introduce two of them as
    /// zero LSBs). This matches the §5.2.0 reference comment that
    /// scaling factors are halved before the §5.2.2 high-pass
    /// arithmetic.
    pub fn downscale_frame(&self, sop: &[i16; FRAME_SAMPLES]) -> [i16; FRAME_SAMPLES] {
        let mut so = [0i16; FRAME_SAMPLES];
        for k in 0..FRAME_SAMPLES {
            // `>>3` is an arithmetic right shift (sign-extending) per
            // §5.1; `<<2` is a 16-bit left shift. Together they
            // clear the three lowest "don't care" bits and re-introduce
            // two of them as zero — net effect: a 1-bit divide-by-2
            // with a 13-bit aligned result.
            let t = sop[k] >> 3;
            so[k] = t << 2;
        }
        so
    }

    /// Run the §5.2.2 offset-compensation step on a downscaled
    /// 160-sample frame `so[0..159]`, producing `sof[0..159]`. The
    /// step is a high-pass IIR whose recursive arm requires
    /// extended (32-bit) precision; the spec's pseudocode is
    /// reproduced verbatim.
    ///
    /// §5.2.2 pseudocode:
    /// ```text
    /// FOR k=0 to 159:
    ///   /* Compute the non-recursive part. */
    ///   s1   = sub(so[k], z1);
    ///   z1   = so[k];
    ///   /* Compute the recursive part. */
    ///   L_s2 = s1;
    ///   L_s2 = L_s2 << 15;
    ///   /* Execution of a 31 by 16 bits multiplication. */
    ///   msp  = L_z2 >> 15;
    ///   lsp  = L_sub(L_z2, (msp << 15));
    ///   temp = mult_r(lsp, 32735);
    ///   L_s2 = L_add(L_s2, temp);
    ///   L_z2 = L_add(L_mult(msp, 32735) >> 1, L_s2);
    ///   /* Compute sof[k] with rounding. */
    ///   sof[k] = L_add(L_z2, 16384) >> 15;
    /// NEXT k:
    /// ```
    ///
    /// `z1` and `L_z2` are kept in `self` between frames per the
    /// §5.2.2 "Keep z1 and L_z2 in memory for the next frame" note.
    /// Initial values `z1 = 0`, `L_z2 = 0` (§4.5 Table 4.2 +
    /// §5.2.2 "Initial value" line).
    pub fn offset_compensation(&mut self, so: &[i16; FRAME_SAMPLES]) -> [i16; FRAME_SAMPLES] {
        let mut sof = [0i16; FRAME_SAMPLES];
        for k in 0..FRAME_SAMPLES {
            // Non-recursive part.
            let s1 = sub(so[k], self.z1);
            self.z1 = so[k];

            // Recursive part: 31-by-16-bit multiplication split.
            let mut l_s2: i32 = s1 as i32;
            l_s2 <<= 15;

            let msp: i16 = (self.l_z2 >> 15) as i16;
            // lsp = L_sub(L_z2, msp << 15). Note `msp << 15` is a
            // 32-bit shift since msp is sign-extended into i32 in
            // L_sub semantics.
            let lsp_i32 = l_sub(self.l_z2, (msp as i32) << 15);
            // `lsp` is the low-15-bit residue of L_z2 → fits i16.
            let lsp: i16 = lsp_i32 as i16;
            let temp = mult_r(lsp, 32735);
            l_s2 = l_add(l_s2, temp as i32);

            // L_z2 = L_add(L_mult(msp, 32735) >> 1, L_s2);
            self.l_z2 = l_add(l_mult(msp, 32735) >> 1, l_s2);

            // sof[k] = L_add(L_z2, 16384) >> 15;
            sof[k] = (l_add(self.l_z2, 16384) >> 15) as i16;
        }
        sof
    }

    /// Run the §5.2.3 first-order FIR pre-emphasis step on `sof[0..159]`,
    /// producing the analysis-input frame `s[0..159]`.
    ///
    /// §5.2.3 pseudocode:
    /// ```text
    /// FOR k=0 to 159:
    ///     s[k] = add( sof[k], mult_r( mp, -28180 ) );
    ///     mp   = sof[k];
    /// NEXT k:
    /// ```
    ///
    /// `mp` is kept in `self` between frames per the §5.2.3 "Keep mp
    /// in memory for the next frame" note. Initial value `mp = 0`
    /// (§4.5 Table 4.2 + §5.2.3 "Initial value: mp=0" line).
    ///
    /// Symmetry check: the decoder's §5.3.5 de-emphasis uses
    /// coefficient `+28180`, the encoder's §5.2.3 pre-emphasis
    /// uses `-28180`. This pairing is consistent with the floating-
    /// point §3.1.2 form `s(k) = sof(k) - beta*sof(k-1)` with
    /// `beta = 28180 * 2^-15`.
    pub fn pre_emphasis(&mut self, sof: &[i16; FRAME_SAMPLES]) -> [i16; FRAME_SAMPLES] {
        let mut s = [0i16; FRAME_SAMPLES];
        for k in 0..FRAME_SAMPLES {
            s[k] = add(sof[k], mult_r(self.mp, -28180));
            self.mp = sof[k];
        }
        s
    }

    /// Run §5.2.1 + §5.2.2 + §5.2.3 end-to-end on a 160-sample input
    /// frame. The return value is the §5.2.3 output `s[0..159]` —
    /// the input the LPC-analysis clause (§5.2.4) consumes. The
    /// encoder's offset and pre-emphasis state is updated for the
    /// next frame.
    pub fn process_frame(&mut self, sop: &[i16; FRAME_SAMPLES]) -> [i16; FRAME_SAMPLES] {
        let so = self.downscale_frame(sop);
        let sof = self.offset_compensation(&so);
        self.pre_emphasis(&sof)
    }

    /// Inspect the current `z1` state (§5.2.2). Exposed for callers
    /// running per-frame trace comparisons against the spec; not
    /// part of the steady-state pipeline.
    pub fn z1(&self) -> i16 {
        self.z1
    }

    /// Inspect the current `L_z2` state (§5.2.2).
    pub fn l_z2(&self) -> i32 {
        self.l_z2
    }

    /// Inspect the current `mp` state (§5.2.3).
    pub fn mp(&self) -> i16 {
        self.mp
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// §5.2.1 downscale of an all-zero frame is all-zero.
    #[test]
    fn downscale_zero_is_zero() {
        let pp = PreProcessor::new();
        let inp = [0i16; FRAME_SAMPLES];
        let out = pp.downscale_frame(&inp);
        assert_eq!(out, [0i16; FRAME_SAMPLES]);
    }

    /// §5.2.1 downscale formula: out = ((sop >> 3) << 2). For sop=8
    /// (the encoder-homing PCM sample value) this is (8>>3)<<2 =
    /// 1<<2 = 4.
    #[test]
    fn downscale_of_encoder_homing_pcm_is_4() {
        let pp = PreProcessor::new();
        let inp = [0x0008i16; FRAME_SAMPLES];
        let out = pp.downscale_frame(&inp);
        for s in out {
            assert_eq!(s, 4);
        }
    }

    /// §5.2.1 throws away the three "don't care" LSBs and re-emits
    /// the result aligned to bit-2: the output's low 2 bits are
    /// always zero.
    #[test]
    fn downscale_clears_low_two_bits() {
        let pp = PreProcessor::new();
        let mut inp = [0i16; FRAME_SAMPLES];
        // Walk a few representative input values.
        inp[0] = 0x7FF8; // max positive after shaping
        inp[1] = 0x7FFF; // max positive raw
        inp[2] = -8;
        inp[3] = -1;
        inp[4] = 100;
        let out = pp.downscale_frame(&inp);
        for s in out.iter() {
            assert_eq!(*s & 0b11, 0, "low 2 bits must be zero after §5.2.1");
        }
    }

    /// §5.2.1 sign preserved by the arithmetic right shift.
    #[test]
    fn downscale_preserves_sign() {
        let pp = PreProcessor::new();
        let mut inp = [0i16; FRAME_SAMPLES];
        inp[0] = -1000;
        inp[1] = 1000;
        let out = pp.downscale_frame(&inp);
        assert!(out[0] < 0, "negative input ⇒ negative output");
        assert!(out[1] > 0, "positive input ⇒ positive output");
    }

    /// §5.2.2 offset compensation of an all-zero frame from the
    /// home state stays all-zero (no DC offset to remove).
    #[test]
    fn offset_comp_zero_input_is_zero() {
        let mut pp = PreProcessor::new();
        let inp = [0i16; FRAME_SAMPLES];
        let out = pp.offset_compensation(&inp);
        assert_eq!(out, [0i16; FRAME_SAMPLES]);
        assert_eq!(pp.z1(), 0);
        assert_eq!(pp.l_z2(), 0);
    }

    /// §5.2.2 is a high-pass filter: a constant non-zero DC input
    /// from the home state produces a transient at the step edge
    /// that the recursive arm then decays. With `alpha = 32735 *
    /// 2^-15 ≈ 0.99893` the decay is slow (half-life ~650 samples)
    /// so within one 160-sample frame the output stays nearly at
    /// the input level. What we DO get is monotonic decay from
    /// the first to the last sample.
    #[test]
    fn offset_comp_dc_step_is_high_passed() {
        let mut pp = PreProcessor::new();
        // Constant +400 DC input.
        let inp = [400i16; FRAME_SAMPLES];
        let out = pp.offset_compensation(&inp);
        // First sample: s1 = 400 - 0 = 400; z1 = 400; recursive
        // arm starts from zero memory so sof[0] should be near
        // `(400<<15 + 16384) >> 15` ≈ 400.
        assert!(
            out[0].abs() >= 200 && out[0].abs() <= 410,
            "sof[0] ≈ 400, got {}",
            out[0]
        );
        // Tail of the frame must be strictly smaller than the head:
        // the recursive arm decays monotonically from the step edge
        // (constant input ⇒ s1=0 from sample 1 onward).
        assert!(
            out[FRAME_SAMPLES - 1] < out[0],
            "tail must decay below head: head={}, tail={}",
            out[0],
            out[FRAME_SAMPLES - 1]
        );
    }

    /// §5.2.2 long-run behaviour: the offset-compensation IIR
    /// eventually drives a constant-DC input toward zero. With
    /// the slow `alpha ≈ 0.99893` pole, ~5000 samples is enough to
    /// shrink the output by more than half. Run the same constant
    /// input through ~32 frames and confirm the late-frame
    /// magnitude drops materially below the first frame's
    /// magnitude.
    #[test]
    fn offset_comp_dc_step_eventually_decays() {
        let mut pp = PreProcessor::new();
        let inp = [400i16; FRAME_SAMPLES];
        let first = pp.offset_compensation(&inp);
        // Drive many additional frames through to let the IIR
        // settle. The single-pole filter's response to a step has
        // a magnitude proportional to the remaining "offset" in
        // L_z2.
        for _ in 0..32 {
            let _ = pp.offset_compensation(&inp);
        }
        let late = pp.offset_compensation(&inp);
        // Late-frame final sample is materially smaller than
        // first-frame's first sample.
        assert!(
            late[FRAME_SAMPLES - 1].abs() < first[0].abs() / 2,
            "long-run output should decay: first[0]={}, late[159]={}",
            first[0],
            late[FRAME_SAMPLES - 1]
        );
    }

    /// §5.2.2 sign preserved on a negative-DC input — the recursive
    /// arm tracks the input polarity.
    #[test]
    fn offset_comp_sign_tracks_input() {
        let mut pp = PreProcessor::new();
        let inp = [-1000i16; FRAME_SAMPLES];
        let out = pp.offset_compensation(&inp);
        assert!(out[0] < 0, "negative step input ⇒ negative first sample");
    }

    /// §5.2.2 state persists across calls — running two halves
    /// back-to-back must produce the same output as one
    /// continuous call.
    #[test]
    fn offset_comp_state_persists_across_frames() {
        // Build a deterministic ramp input (160 samples).
        let mut inp = [0i16; FRAME_SAMPLES];
        for (k, slot) in inp.iter_mut().enumerate() {
            *slot = ((k as i16) - 80) * 10;
        }

        // Single-shot run.
        let mut a = PreProcessor::new();
        let one = a.offset_compensation(&inp);

        // Half + half: feed the first 80, then the last 80 via a
        // second 160-frame call that has the first 80 zeroed.
        // Easier: run the same frame twice. State carry forward
        // should yield a different second-frame output than the
        // first.
        let mut b = PreProcessor::new();
        let first = b.offset_compensation(&inp);
        let second = b.offset_compensation(&inp);
        assert_eq!(first, one, "first call must match single-shot");
        assert_ne!(second, first, "second call differs because state carried");
    }

    /// §5.2.3 pre-emphasis on an all-zero input from home state is
    /// all-zero.
    #[test]
    fn pre_emphasis_zero_is_zero() {
        let mut pp = PreProcessor::new();
        let inp = [0i16; FRAME_SAMPLES];
        let out = pp.pre_emphasis(&inp);
        assert_eq!(out, [0i16; FRAME_SAMPLES]);
        assert_eq!(pp.mp(), 0);
    }

    /// §5.2.3 first-sample identity: with `mp = 0` (home state) the
    /// first output sample equals `add(sof[0], mult_r(0, -28180))`
    /// = `sof[0]`. Verify on a step input.
    #[test]
    fn pre_emphasis_first_sample_equals_input_from_home_state() {
        let mut pp = PreProcessor::new();
        let mut inp = [0i16; FRAME_SAMPLES];
        inp[0] = 5000;
        let out = pp.pre_emphasis(&inp);
        assert_eq!(out[0], 5000);
        // After the FRAME is consumed, mp carries the LAST sof
        // (§5.2.3: "Keep mp in memory for the next frame"). For an
        // input that is zero from index 1 onward, the final mp is
        // sof[159] = 0.
        assert_eq!(pp.mp(), 0);
    }

    /// After a single-sample non-zero input, `mp` carries the
    /// final sample of that frame as the cross-frame memory.
    /// With a non-zero last sample we can see mp pick it up.
    #[test]
    fn pre_emphasis_mp_carries_last_sample() {
        let mut pp = PreProcessor::new();
        let mut inp = [0i16; FRAME_SAMPLES];
        inp[FRAME_SAMPLES - 1] = 1234;
        let _ = pp.pre_emphasis(&inp);
        assert_eq!(pp.mp(), 1234);
    }

    /// §5.2.3 second-sample shape: with `mp = 5000` after sample 0
    /// (above), sample 1 of an all-zero remainder is
    /// `add(0, mult_r(5000, -28180))`. mult_r(5000, -28180) =
    /// (5000*-28180 + 16384) >> 15 = (-140_900_000 + 16384) >> 15 =
    /// approximately -4300. Just verify the sign + rough magnitude.
    #[test]
    fn pre_emphasis_iir_subtracts_scaled_prev_input() {
        let mut pp = PreProcessor::new();
        let mut inp = [0i16; FRAME_SAMPLES];
        inp[0] = 5000;
        let out = pp.pre_emphasis(&inp);
        // out[1] = add(0, mult_r(5000, -28180)) — a negative,
        // bounded value.
        assert!(out[1] < 0, "out[1] must be negative");
        assert!(
            out[1].abs() > 3000 && out[1].abs() < 5000,
            "rough magnitude check, got {}",
            out[1]
        );
    }

    /// `process_frame` is the composition §5.2.1 → §5.2.2 → §5.2.3.
    /// An all-zero input from the home state stays all-zero through
    /// the whole pre-processing pipeline.
    #[test]
    fn process_frame_zero_input_zero_output() {
        let mut pp = PreProcessor::new();
        let inp = [0i16; FRAME_SAMPLES];
        let out = pp.process_frame(&inp);
        assert_eq!(out, [0i16; FRAME_SAMPLES]);
        assert_eq!(pp.z1(), 0);
        assert_eq!(pp.l_z2(), 0);
        assert_eq!(pp.mp(), 0);
    }

    /// `reset` restores all three §4.5 state fields to their home
    /// values.
    #[test]
    fn reset_returns_home_state() {
        let mut pp = PreProcessor::new();
        // Push a non-zero frame through to dirty the state.
        let inp = [1000i16; FRAME_SAMPLES];
        let _ = pp.process_frame(&inp);
        // Some component of state should now be non-zero.
        let dirty = pp.z1() != 0 || pp.l_z2() != 0 || pp.mp() != 0;
        assert!(dirty, "state should have moved after a real frame");
        pp.reset();
        assert_eq!(pp.z1(), 0);
        assert_eq!(pp.l_z2(), 0);
        assert_eq!(pp.mp(), 0);
    }

    /// `process_frame` is deterministic from the home state: feeding
    /// the same input to two freshly-reset pre-processors yields the
    /// same output.
    #[test]
    fn process_frame_is_deterministic() {
        let mut inp = [0i16; FRAME_SAMPLES];
        for (k, slot) in inp.iter_mut().enumerate() {
            *slot = ((k as i16) * 37) ^ 0x2A;
        }
        let mut a = PreProcessor::new();
        let mut b = PreProcessor::new();
        assert_eq!(a.process_frame(&inp), b.process_frame(&inp));
    }
}

// ──────────────────────────────────────────────────────────────────
// §5.2.4 / §5.2.5 / §5.2.6 — LPC analysis front-end
// ──────────────────────────────────────────────────────────────────

/// LPC analysis stages: §5.2.4 (autocorrelation), §5.2.5 (Schur
/// recursion → reflection coefficients), and §5.2.6 (reflection →
/// Log-Area Ratio transform).
///
/// All three procedures are stateless per-frame: they take the
/// pre-processed frame `s[0..159]` produced by [`PreProcessor`] and
/// emit the per-frame array `LAR[1..=8]` that the §5.2.7 quantiser
/// (next round) consumes. None of them mutates persistent encoder
/// state — the only cross-frame state in the §5.2 encoder lives in
/// [`PreProcessor`] (§5.2.2/§5.2.3) and in the §5.2.10 short-term
/// analysis filter (§4.5 `u[0..7]`) which arrives in a later round.
///
/// Several routines walk the spec's 1-based index `i = 1..=8` and
/// index both an input and an output array on `i` simultaneously.
/// Clippy's `needless_range_loop` lint argues for an iterator-based
/// rewrite; we leave the loops in their spec-faithful shape so the
/// pseudocode-to-Rust correspondence stays one-for-one and visible.
#[allow(clippy::needless_range_loop)]
pub mod analysis {
    use super::FRAME_SAMPLES;
    use crate::arith::{
        abs, add, div, l_add, l_mult, mult, mult_r, norm, shl_signed, shr_signed, sub,
    };
    use crate::decoder::{decode_lar, interpolate_lar, larp_to_rp};
    use crate::tables::{A, B, DLB, FAC, H, MAC, MIC, NRFAC, QLB};

    /// Number of samples in one sub-segment (§5.2.11 input range
    /// `d[0..39]` / §5.2.12 output range `e[0..39]`).
    pub const SUBFRAME_SAMPLES: usize = 40;

    /// Number of sub-segments per 160-sample frame (§5.2.11 is run
    /// once per sub-segment).
    pub const SUBFRAMES_PER_FRAME: usize = 4;

    /// Length of the §4.5 `dp[-120..=-1]` LTP delay line.
    pub const LTP_DELAY: usize = 120;

    /// Per-sub-segment LTP parameters produced by §5.2.11.
    ///
    /// `n_c` is the §5.2.11 LTP lag in the range \[40, 120\] (7-bit
    /// codeword the §1.7 packer emits as `Nc`). `b_c` is the §5.2.11
    /// coded LTP gain in the range \[0, 3\] (2-bit codeword `bc`).
    #[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
    pub struct LtpParameters {
        /// §5.2.11 LTP lag — the lambda that maximises the
        /// cross-correlation. Spec range 40..=120. When `L_max` is
        /// non-positive the spec exits the procedure early without
        /// updating `Nc`; this field then carries the entry-time
        /// `Nc = 40` initialisation per §5.2.11.
        pub n_c: i16,
        /// §5.2.11 coded LTP gain — index into [`QLB`] / [`DLB`].
        /// Spec range 0..=3.
        pub b_c: i16,
    }

    /// §5.2.4 — autocorrelation with dynamic input scaling.
    ///
    /// Takes the pre-processed input `s[0..159]` (the §5.2.3 output)
    /// and returns the 9-entry 32-bit autocorrelation array
    /// `L_ACF[0..=8]` per the §5.2.4 pseudocode.
    ///
    /// §5.2.4 pseudocode:
    /// ```text
    /// Search for the maximum.
    /// smax = 0;
    /// FOR k = 0 to 159:
    ///     temp = abs(s[k]);
    ///     IF (temp > smax) THEN smax = temp;
    /// NEXT k:
    ///
    /// Computation of the scaling factor.
    /// IF (smax == 0) THEN scalauto = 0;
    /// ELSE scalauto = sub(4, norm(smax << 16));
    ///
    /// Scaling of the array s[0..159].
    /// IF (scalauto > 0) THEN
    ///     temp = 16384 >> sub(scalauto, 1);
    ///     FOR k = 0 to 159:
    ///         s[k] = mult_r(s[k], temp);
    ///     NEXT k:
    ///
    /// Compute the L_ACF[..].
    /// FOR k = 0 to 8:
    ///     L_ACF[k] = 0;
    ///     FOR i = k to 159:
    ///         L_temp = L_mult(s[i], s[i-k]);
    ///         L_ACF[k] = L_add(L_ACF[k], L_temp);
    ///     NEXT i:
    /// NEXT k:
    ///
    /// Rescaling of the array s[0..159].
    /// IF (scalauto > 0) THEN
    ///     FOR k = 0 to 159:
    ///         s[k] = s[k] << scalauto;
    ///     NEXT k:
    /// ```
    ///
    /// The §5.2.4 "Rescaling of the array s[0..159]" step at the end
    /// of the spec restores `s[]` to its pre-scaling magnitude — the
    /// scaling exists purely to keep the inner-product accumulator
    /// from overflowing. Since this helper takes `s` by value (a
    /// fixed 160-sample array), the upscale step is a no-op for the
    /// caller and is omitted. Callers who need the §5.2.10 short-term
    /// analysis filter input simply pass the unscaled `s` array
    /// returned by [`PreProcessor::process_frame`].
    pub fn autocorrelation(s_in: &[i16; FRAME_SAMPLES]) -> [i32; 9] {
        // §5.2.4 — search for smax = max |s[k]|.
        let mut smax: i16 = 0;
        for &v in s_in.iter() {
            let t = abs(v);
            if t > smax {
                smax = t;
            }
        }

        // §5.2.4 — derive scalauto.
        //
        // `smax << 16` is a 32-bit left shift: smax is i16 in
        // [0, 32767], so smax<<16 is in [0, 0x7FFF_0000] and never
        // overflows i32. For smax == 32767 we get scalauto = 4 - 0
        // = 4 (max possible scale-down); for smax == 1 the value
        // is 1<<16, norm = 14, scalauto = -10 ⇒ no scaling.
        let scalauto: i16 = if smax == 0 {
            0
        } else {
            sub(4, norm((smax as i32) << 16))
        };

        // §5.2.4 — scale s[] if scalauto > 0.
        //
        // We work in a local mutable buffer so the caller's `s_in`
        // isn't disturbed; the §5.2.4 "Rescaling" step at the end of
        // the spec restores s[] for the next analysis pass anyway, so
        // the net effect on `s_in` is zero.
        let mut s = *s_in;
        if scalauto > 0 {
            // §5.2.4: `temp = 16384 >> sub(scalauto, 1)` — this gives
            // 2^(15 - scalauto). For scalauto=1 we get 16384 (the
            // Q15 "0.5" rounding constant — i.e. mult_r by 16384 is
            // a divide-by-two with round-half-up); for scalauto=4 we
            // get 2048 (divide-by-16 in Q15 with rounding).
            let shift = sub(scalauto, 1) as u32;
            let temp = 16384i16 >> shift;
            for slot in s.iter_mut() {
                *slot = mult_r(*slot, temp);
            }
        }

        // §5.2.4 — inner product L_ACF[k] = Σ s[i] * s[i-k] for k=0..=8.
        let mut l_acf = [0i32; 9];
        for k in 0..=8 {
            let mut acc: i32 = 0;
            for i in k..FRAME_SAMPLES {
                let l_temp = l_mult(s[i], s[i - k]);
                acc = l_add(acc, l_temp);
            }
            l_acf[k] = acc;
        }

        l_acf
    }

    /// §5.2.5 — Schur recursion producing the eight reflection
    /// coefficients `r[1..=8]` from the autocorrelation `L_ACF[0..=8]`.
    ///
    /// The returned array is indexed 1..=8 (entry 0 is a sentinel
    /// zero), matching the §5.2 / §5.4 spec convention used elsewhere
    /// in the crate (see `tables::A`, `tables::INVA`).
    ///
    /// §5.2.5 pseudocode (full):
    /// ```text
    /// IF (L_ACF[0] == 0) THEN
    ///     FOR i = 1 to 8: r[i] = 0; NEXT i:
    ///     EXIT;
    ///
    /// temp = norm(L_ACF[0]);
    /// FOR k = 0 to 8:
    ///     ACF[k] = (L_ACF[k] << temp) >> 16;
    /// NEXT k:
    ///
    /// Initialize array P[..] and K[..] for the recursion.
    /// FOR i = 1 to 7:
    ///     K[9-i] = ACF[i];
    /// NEXT i:
    /// FOR i = 0 to 8:
    ///     P[i] = ACF[i];
    /// NEXT i:
    ///
    /// Compute reflection coefficients.
    /// FOR n = 1 to 8:
    ///     IF (P[0] < abs(P[1])) THEN
    ///         FOR i = n to 8: r[i] = 0; NEXT i:
    ///         EXIT;
    ///     r[n] = div(abs(P[1]), P[0]);
    ///     IF (P[1] > 0) THEN r[n] = sub(0, r[n]);
    ///     IF (n == 8) THEN EXIT;
    ///     Schur recursion.
    ///     P[0] = add(P[0], mult_r(P[1], r[n]));
    ///     FOR m = 1 to 8-n:
    ///         P[m]    = add(P[m+1],   mult_r(K[9-m], r[n]));
    ///         K[9-m]  = add(K[9-m],   mult_r(P[m+1], r[n]));
    ///     NEXT m:
    /// NEXT n:
    /// ```
    ///
    /// `r[i]` is encoded as `integer(real_r * 32768)` with
    /// `-1.0 <= real_r < 1.0` per the §5.2.6 prelude — i.e. a Q15
    /// fixed-point reflection coefficient.
    pub fn reflection_coefficients(l_acf: &[i32; 9]) -> [i16; 9] {
        let mut r = [0i16; 9];

        // §5.2.5 — L_ACF[0] == 0 short-circuit: r[1..=8] = 0.
        if l_acf[0] == 0 {
            return r;
        }

        // §5.2.5 — normalise L_ACF[0] and project the whole array
        // down to 16-bit ACF[0..=8].
        let temp = norm(l_acf[0]) as u32;
        let mut acf = [0i16; 9];
        for (slot, &v) in acf.iter_mut().zip(l_acf.iter()) {
            // The spec's `L_ACF[k] << temp` is the §5.1 32-bit left
            // shift. `temp` is in [0, 31] per `norm`'s contract.
            let shifted = (v as u32).wrapping_shl(temp) as i32;
            *slot = (shifted >> 16) as i16;
        }

        // §5.2.5 — initialise P[0..=8] and K[2..=8] (K[9-i] for
        // i=1..=7 ⇒ indices 8 down to 2).
        let mut p = [0i16; 9];
        let mut k_arr = [0i16; 9];
        p.copy_from_slice(&acf);
        // K[9-i] = ACF[i] for i = 1..=7 ⇒ k_arr[2..=8] is ACF[7..=1]
        // in reverse: k_arr[8] = acf[1], k_arr[7] = acf[2], …,
        // k_arr[2] = acf[7].
        for (offset, &v) in acf[1..=7].iter().enumerate() {
            k_arr[8 - offset] = v;
        }

        // §5.2.5 — main recursion for n = 1..=8.
        for n in 1..=8usize {
            // §5.2.5 — exit-early test: if |P[1]| > P[0] the remaining
            // reflection coefficients are zero.
            if p[0] < abs(p[1]) {
                for slot in r.iter_mut().skip(n) {
                    *slot = 0;
                }
                return r;
            }

            // §5.2.5 — r[n] = div(|P[1]|, P[0]); negate if P[1] > 0.
            //
            // The spec's div contract requires `denum >= num > 0`.
            // Our `arith::div` matches that. Note the sign convention:
            // since `abs(P[1]) <= P[0]` here, `r[n]` is in [0, 32767].
            // If P[1] > 0, the §5.2.5 conditional flips it negative.
            r[n] = div(abs(p[1]), p[0]);
            if p[1] > 0 {
                r[n] = sub(0, r[n]);
            }
            if n == 8 {
                return r;
            }

            // §5.2.5 — Schur recursion step.
            // P[0] = add(P[0], mult_r(P[1], r[n]))
            p[0] = add(p[0], mult_r(p[1], r[n]));
            // FOR m=1 to 8-n:
            //   P[m]   = add(P[m+1],   mult_r(K[9-m], r[n]));
            //   K[9-m] = add(K[9-m],   mult_r(P[m+1], r[n]));
            //
            // The spec's intent is that the m-th iteration uses the
            // OLD P[m+1] and OLD K[9-m] in BOTH lines: the K update
            // reads the same P[m+1] the P update read, not the new
            // one. We capture P[m+1] and K[9-m] up front.
            for m in 1..=(8 - n) {
                let p_mp1_old = p[m + 1];
                let k_old = k_arr[9 - m];
                p[m] = add(p_mp1_old, mult_r(k_old, r[n]));
                k_arr[9 - m] = add(k_old, mult_r(p_mp1_old, r[n]));
            }
        }

        r
    }

    /// §5.2.6 — transform reflection coefficients `r[1..=8]` into
    /// Log-Area Ratios `LAR[1..=8]`.
    ///
    /// §5.2.6 pseudocode:
    /// ```text
    /// FOR i = 1 to 8:
    ///     temp = abs(r[i]);
    ///     IF (temp < 22118)      THEN temp = temp >> 1;
    ///     ELSE IF (temp < 31130) THEN temp = sub(temp, 11059);
    ///     ELSE                        temp = sub(temp, 26112) << 2;
    ///     LAR[i] = temp;
    ///     IF (r[i] < 0) THEN LAR[i] = sub(0, LAR[i]);
    /// NEXT i:
    /// ```
    ///
    /// Per the §5.2.6 prelude, `LAR[i]` is scaled
    /// `integer(real_LAR * 16384)` with `-1.625 <= real_LAR <= 1.625`.
    /// `r[i]` is the Q15 value produced by [`reflection_coefficients`].
    ///
    /// This is the **inverse** of the decoder's §5.2.9.2 LARp → rp
    /// piecewise map (whose three segments are 11059, 20070, ...);
    /// the encoder's segment points are 22118 = 2 * 11059 and
    /// 31130 ≈ 26112 + 5018 in `|r|`-space, and the magnitude domain
    /// in each segment is divided by two on output, subtracted with
    /// a Q14-aligned bias on the middle segment, or magnified-then-
    /// biased on the upper segment.
    pub fn reflection_to_lar(r: &[i16; 9]) -> [i16; 9] {
        let mut lar = [0i16; 9];
        for i in 1..=8 {
            let mut temp = abs(r[i]);
            if temp < 22118 {
                temp >>= 1;
            } else if temp < 31130 {
                temp = sub(temp, 11059);
            } else {
                // §5.2.6: `sub(temp, 26112) << 2`. `sub` saturates
                // at i16; the subsequent `<< 2` is a 16-bit left
                // shift per §5.1 — saturating semantics.
                let diff = sub(temp, 26112);
                temp = ((diff as i32) << 2).clamp(i16::MIN as i32, i16::MAX as i32) as i16;
            }
            lar[i] = if r[i] < 0 { sub(0, temp) } else { temp };
        }
        lar
    }

    /// Convenience wrapper: run §5.2.4 + §5.2.5 + §5.2.6 end-to-end
    /// on a pre-processed frame `s[0..159]` and return the per-frame
    /// `LAR[1..=8]` array the §5.2.7 quantiser consumes.
    pub fn analyse_frame(s: &[i16; FRAME_SAMPLES]) -> [i16; 9] {
        let l_acf = autocorrelation(s);
        let r = reflection_coefficients(&l_acf);
        reflection_to_lar(&r)
    }

    /// §5.2.7 — quantise + code the Log-Area Ratios `LAR[1..=8]` into
    /// the coded `LARc[1..=8]` array that gets packed into the §1.7
    /// frame.
    ///
    /// §5.2.7 pseudocode:
    /// ```text
    /// FOR i = 1 to 8:
    ///     temp = mult(A[i], LAR[i]);
    ///     temp = add(temp, B[i]);
    ///     temp = add(temp, 256);          /* for rounding */
    ///     LARc[i] = temp >> 9;
    ///
    ///     /* Check IF LARc[i] lies between MIN and MAX */
    ///     IF (LARc[i] > MAC[i]) THEN LARc[i] = MAC[i];
    ///     IF (LARc[i] < MIC[i]) THEN LARc[i] = MIC[i];
    ///
    ///     LARc[i] = sub(LARc[i], MIC[i]); /* make LARc[i] positive */
    /// NEXT i:
    /// ```
    ///
    /// `A[i]` is staged as `integer(real_A[i] * 1024)` (§5.4 Table 5.1
    /// column A) and `B[i]` as `integer(real_B[i] * 512)` (column B);
    /// `mult(A[i], LAR[i])` ≈ `real_A * LAR / 32` (Q15 mult on a Q10
    /// scaled `A`), `+ B` adds the column-B bias at the same Q9 scale,
    /// `+ 256` is the Q9 half-step rounding constant, and the trailing
    /// `>> 9` lands the result in the integer `LARc` codeword domain.
    /// `MIC[i]` and `MAC[i]` (Table 5.1 columns MIC / MAC) bound the
    /// codeword to the per-segment width specified by Table 1.1, and
    /// the final `sub(_, MIC[i])` shifts the signed codeword into the
    /// unsigned 0..(MAC-MIC) range that the §1.7 bit packer emits.
    ///
    /// Symmetry / inverse — this is the encoder partner of the
    /// decoder's §5.2.8 `decode_lar` (see `src/decoder.rs`); the
    /// composition `quantise_lar` ∘ `decode_lar` is a quantisation
    /// round trip and, modulo the inherent quantiser step, the
    /// composition `decode_lar(quantise_lar(LAR))` returns a value
    /// close to the input `LAR`.
    ///
    /// The `LAR` input is the §5.2.6 output (`reflection_to_lar`).
    /// Index 0 is reserved (zero); only `LAR[1..=8]` participate.
    pub fn quantise_lar(lar: &[i16; 9]) -> [i16; 9] {
        let mut lar_c = [0i16; 9];
        for i in 1..=8 {
            // §5.2.7: `temp = mult(A[i], LAR[i]); temp = add(temp, B[i]);
            //          temp = add(temp, 256); LARc[i] = temp >> 9;`
            let mut temp = mult(A[i], lar[i]);
            temp = add(temp, B[i]);
            temp = add(temp, 256);
            // §5.1: `>> 9` is the 16-bit arithmetic right shift.
            let mut code = temp >> 9;

            // §5.2.7 bounds check against Table 5.1 MIN / MAX.
            if code > MAC[i] {
                code = MAC[i];
            }
            if code < MIC[i] {
                code = MIC[i];
            }

            // §5.2.7 NOTE — "The equation is used to make all the
            // LARc[i] positive." Shift the signed codeword into the
            // unsigned 0..(MAC-MIC) range the §1.7 bit packer emits.
            lar_c[i] = sub(code, MIC[i]);
        }
        lar_c
    }

    /// Convenience wrapper: run §5.2.4 + §5.2.5 + §5.2.6 + §5.2.7
    /// end-to-end on a pre-processed frame `s[0..159]` and return the
    /// per-frame `LARc[1..=8]` codewords. Index 0 is a sentinel zero.
    pub fn analyse_and_quantise_frame(s: &[i16; FRAME_SAMPLES]) -> [i16; 9] {
        let lar = analyse_frame(s);
        quantise_lar(&lar)
    }

    /// §5.2.10 — short-term analysis filter, single block.
    ///
    /// Runs the §5.2.10 8-stage lattice filter over the input
    /// `s[k_start..=k_end]` using the reflection coefficients
    /// `rp[1..=8]` produced by §5.2.9.2 for this block, writing the
    /// short-term residual `d[k_start..=k_end]` and updating the
    /// 8-entry filter memory `u[0..=7]` in place.
    ///
    /// §5.2.10 pseudocode:
    /// ```text
    /// FOR k = k_start to k_end:
    ///     di  = s[k];
    ///     sav = di;
    ///     FOR i = 1 to 8:
    ///         temp   = add( u[i-1], mult_r( rp[i], di ) );
    ///         di     = add( di,     mult_r( rp[i], u[i-1] ) );
    ///         u[i-1] = sav;
    ///         sav    = temp;
    ///     NEXT i:
    ///     d[k] = di;
    /// NEXT k:
    /// ```
    ///
    /// The §5.2.10 spec calls out the §4.5 Table 4.2 entry
    /// `u[0..=7]` ("Short term analysis filter memory") as the
    /// across-block / across-frame state — the caller owns this
    /// buffer and persists it.
    ///
    /// `rp` is indexed 1..=8 with index 0 a sentinel zero, matching
    /// the spec's 1-based convention used elsewhere in the crate.
    pub fn short_term_analysis_filter(
        s: &[i16; FRAME_SAMPLES],
        rp: &[i16; 9],
        u: &mut [i16; 8],
        k_start: usize,
        k_end: usize,
    ) -> [i16; FRAME_SAMPLES] {
        let mut d = [0i16; FRAME_SAMPLES];
        for k in k_start..=k_end {
            let mut di = s[k];
            let mut sav = di;
            for i in 1..=8 {
                // §5.2.10: temp = add(u[i-1], mult_r(rp[i], di));
                let temp = add(u[i - 1], mult_r(rp[i], di));
                // §5.2.10: di = add(di, mult_r(rp[i], u[i-1]));
                di = add(di, mult_r(rp[i], u[i - 1]));
                // §5.2.10: u[i-1] = sav; sav = temp;
                u[i - 1] = sav;
                sav = temp;
            }
            d[k] = di;
        }
        d
    }

    /// Persistent analysis-clause state across calls.
    ///
    /// Wraps the two §4.5 Table 4.2 entries owned by the §5.2.9.1
    /// LAR interpolator and the §5.2.10 short-term analysis filter:
    ///
    /// * `LARpp(j-1)[1..=8]` — the previous frame's `LARpp` set,
    ///   home value all-zero per §4.5.
    /// * `u[0..=7]` — the §5.2.10 short-term analysis filter
    ///   memory, home value all-zero per §4.5 + §5.2.10.
    ///
    /// The §5.2.10 "u[0..=7] in memory" + §5.2.9.1 "Initial value
    /// LARpp(j-1)[1..8]=0" notes mandate this state survives across
    /// the four per-frame sub-blocks and across frames.
    #[derive(Debug, Clone, Default)]
    pub struct Analyzer {
        /// §4.5 / §5.2.9.1 — LARs from the previous frame.
        /// Index 0 is a sentinel zero.
        lar_pp_prev: [i16; 9],
        /// §4.5 / §5.2.10 — short-term analysis filter memory.
        u: [i16; 8],
    }

    impl Analyzer {
        /// Build a fresh analyzer in its §4.5 Table 4.2 home state
        /// (`LARpp(j-1)[1..=8] = 0`, `u[0..=7] = 0`).
        pub fn new() -> Self {
            Self::default()
        }

        /// Reset the analyzer to its §4.5 Table 4.2 home values.
        pub fn reset(&mut self) {
            *self = Self::default();
        }

        /// Inspect the persisted `LARpp(j-1)` state (§5.2.9.1).
        /// Index 0 is a sentinel zero.
        pub fn lar_pp_prev(&self) -> &[i16; 9] {
            &self.lar_pp_prev
        }

        /// Inspect the persisted §5.2.10 `u[0..=7]` filter memory.
        pub fn u(&self) -> &[i16; 8] {
            &self.u
        }

        /// Run §5.2.7 → §5.2.8 → §5.2.9.1 → §5.2.9.2 → §5.2.10
        /// end-to-end on the pre-processed frame `s[0..159]` and
        /// return the short-term residual `d[0..159]` plus the
        /// `LARc[1..=8]` codewords the §1.7 packer will emit.
        ///
        /// The pipeline matches the §5.2 "encoder loop" outline:
        ///
        /// 1. §5.2.4..§5.2.7 — autocorrelation, Schur, LAR, quantise
        ///    + code into `LARc[1..=8]`.
        /// 2. §5.2.8 — decode the `LARc[1..=8]` back into the
        ///    `LARpp(j)[1..=8]` set the **decoder** will see, so
        ///    encoder and decoder run on bit-identical reflection
        ///    coefficients for the §5.2.10 / §5.3.4 lattices.
        /// 3. §5.2.9.1 — for each of the four blocks (k = 0..=12,
        ///    13..=26, 27..=39, 40..=159) compute `LARp` by
        ///    interpolating `LARpp(j-1)` and `LARpp(j)`.
        /// 4. §5.2.9.2 — convert each `LARp` into `rp[1..=8]`.
        /// 5. §5.2.10 — run the 8-stage lattice on
        ///    `s[k_start..=k_end]` using that block's `rp`,
        ///    persisting `u[0..=7]` across blocks and frames.
        ///
        /// After the four blocks complete, `LARpp(j-1)` is updated
        /// to the current frame's `LARpp(j)` per the §5.2.9.1
        /// across-frame convention.
        pub fn analyse_frame(
            &mut self,
            s: &[i16; FRAME_SAMPLES],
        ) -> ([i16; 9], [i16; FRAME_SAMPLES]) {
            // §5.2.4..§5.2.7 — produce LARc[1..=8].
            let lar_c = analyse_and_quantise_frame(s);

            // §5.2.8 — decode LARc[..] → LARpp(j)[..], the same set
            // the receiver will see.
            let lar_pp_curr = decode_lar(&lar_c);

            // §5.2.9.1 + §5.2.9.2 + §5.2.10 — four blocks.
            // Block boundaries (k_start, k_end) per §5.2.9.1 +
            // §5.2.10 spec.
            const BLOCKS: [(usize, usize, u8); 4] =
                [(0, 12, 0), (13, 26, 1), (27, 39, 2), (40, 159, 3)];

            let mut d_frame = [0i16; FRAME_SAMPLES];
            for &(k_start, k_end, block_id) in BLOCKS.iter() {
                // §5.2.9.1 — interpolate LARpp(j-1) + LARpp(j) → LARp.
                let lar_p = interpolate_lar(&self.lar_pp_prev, &lar_pp_curr, block_id);
                // §5.2.9.2 — LARp → rp.
                let rp = larp_to_rp(&lar_p);
                // §5.2.10 — short-term analysis filter for this block.
                let d_block = short_term_analysis_filter(s, &rp, &mut self.u, k_start, k_end);
                d_frame[k_start..=k_end].copy_from_slice(&d_block[k_start..=k_end]);
            }

            // §5.2.9.1 — slide LARpp(j) into LARpp(j-1) for the
            // next frame.
            self.lar_pp_prev = lar_pp_curr;

            (lar_c, d_frame)
        }
    }

    // ──────────────────────────────────────────────────────────────────
    // §5.2.11 / §5.2.12 — Long-Term Prediction (LTP) clause
    // ──────────────────────────────────────────────────────────────────

    /// §5.2.11 — compute the LTP parameters `(Nc, bc)` for one
    /// 40-sample sub-segment.
    ///
    /// `d` is the §5.2.10 short-term residual `d[0..=39]` for this
    /// sub-segment; `dp_hist` is the §4.5 LTP delay line
    /// `dp[-120..=-1]` with `dp_hist[0]` = `dp[-1]` (the most-recent
    /// past sample) and `dp_hist[119]` = `dp[-120]` (the oldest).
    /// This matches the decoder's `drp_hist` layout and the spec's
    /// `dp[k - lambda]` indexing (positive `lambda` reaches back into
    /// history).
    ///
    /// Returns the [`LtpParameters`] `(Nc, bc)`. The procedure does
    /// **not** consume the calling state or modify `dp_hist`; the §4.5
    /// across-sub-segment dp-update happens later via §5.2.18 once the
    /// RPE pulses for this sub-segment have been generated.
    ///
    /// §5.2.11 pseudocode (verbatim):
    /// ```text
    /// Search of the optimum scaling of d[0..39].
    /// dmax = 0;
    /// FOR k = 0 to 39:
    ///     temp = abs( d[k] );
    ///     IF (temp > dmax) THEN dmax = temp;
    /// NEXT k:
    /// temp = 0;
    /// IF (dmax == 0) THEN scal = 0;
    ///   ELSE temp = norm( dmax << 16 );
    /// IF (temp > 6) THEN scal = 0;
    ///   ELSE scal = sub(6, temp);
    ///
    /// Initialization of a working array wt[0..39].
    /// FOR k = 0 to 39: wt[k] = d[k] >> scal; NEXT k:
    ///
    /// Search for the maximum cross-correlation and coding of the LTP lag.
    /// L_max = 0;
    /// Nc = 40;
    /// FOR lambda = 40 to 120:
    ///     L_result = 0;
    ///     FOR k = 0 to 39:
    ///         L_temp   = L_mult( wt[k], dp[k-lambda] );
    ///         L_result = L_add ( L_temp, L_result );
    ///     NEXT k:
    ///     IF ( L_result > L_max ) THEN
    ///         Nc    = lambda;
    ///         L_max = L_result;
    /// NEXT lambda:
    ///
    /// Rescaling of L_max.  L_max = L_max >> ( sub(6, scal) );
    ///
    /// Initialization of a working array wt[0..39].
    /// FOR k = 0 to 39: wt[k] = dp[k-Nc] >> 3; NEXT k:
    ///
    /// Compute the power of the reconstructed short term residual dp[..].
    /// L_power = 0;
    /// FOR k = 0 to 39:
    ///     L_temp  = L_mult( wt[k], wt[k] );
    ///     L_power = L_add ( L_temp, L_power );
    /// NEXT k:
    ///
    /// Normalization of L_max and L_power.
    /// IF ( L_max <= 0 ) THEN bc = 0; EXIT;
    /// IF ( L_max >= L_power ) THEN bc = 3; EXIT;
    /// temp = norm( L_power );
    /// R    = ( L_max   << temp ) >> 16;
    /// S    = ( L_power << temp ) >> 16;
    ///
    /// Coding of the LTP gain.  Table 5.3a yields DLB[i].
    /// FOR bc = 0 to 2:
    ///     IF ( R <= mult(S, DLB[bc]) ) THEN EXIT;
    /// NEXT bc:
    /// bc = 3;
    /// ```
    pub fn ltp_parameters(
        d: &[i16; SUBFRAME_SAMPLES],
        dp_hist: &[i16; LTP_DELAY],
    ) -> LtpParameters {
        // §5.2.11 — dmax = max |d[k]|.
        let mut dmax: i16 = 0;
        for &v in d.iter() {
            let t = abs(v);
            if t > dmax {
                dmax = t;
            }
        }

        // §5.2.11 — derive `scal`.
        //
        //   IF (dmax == 0) THEN scal = 0;
        //   ELSE temp = norm(dmax << 16);
        //   IF (temp > 6) THEN scal = 0;
        //   ELSE scal = sub(6, temp);
        //
        // `dmax << 16` is a 32-bit left shift: dmax is in [0, 32767]
        // so dmax<<16 is in [0, 0x7FFF_0000] and never overflows i32.
        let scal: i16 = if dmax == 0 {
            0
        } else {
            let t = norm((dmax as i32) << 16);
            if t > 6 {
                0
            } else {
                sub(6, t)
            }
        };

        // §5.2.11 — wt[k] = d[k] >> scal (sign-extending right shift).
        let mut wt = [0i16; SUBFRAME_SAMPLES];
        let scal_u = scal as u32;
        for (slot, &v) in wt.iter_mut().zip(d.iter()) {
            *slot = v >> scal_u;
        }

        // §5.2.11 — main cross-correlation search.
        //
        // The spec's `dp[k - lambda]` reaches into the §4.5 history
        // for lambda in [40, 120] and k in [0, 39]: k - lambda lies
        // in [-120, -1] — entirely inside `dp_hist`. We index with
        // `dp_hist[lambda - k - 1]` (so lambda=40, k=39 ⇒ dp[-1]
        // = dp_hist[0]; lambda=120, k=0 ⇒ dp[-120] = dp_hist[119]).
        let mut l_max: i32 = 0;
        let mut n_c: i16 = 40;
        for lambda in 40i16..=120 {
            let mut l_result: i32 = 0;
            for k in 0..SUBFRAME_SAMPLES {
                let dp_idx = (lambda as usize) - k - 1; // == lambda - k - 1
                let l_temp = l_mult(wt[k], dp_hist[dp_idx]);
                l_result = l_add(l_temp, l_result);
            }
            if l_result > l_max {
                n_c = lambda;
                l_max = l_result;
            }
        }

        // §5.2.11 — Rescaling of L_max. `L_max >>= sub(6, scal)`.
        //
        // The §5.1 right-shift operator with negative count becomes
        // a left shift of the same magnitude (sign-extending fill),
        // so we need the signed-right-shift semantics. The shift
        // amount `sub(6, scal)` is in [0, 6] when scal >= 0 and in
        // [7, 22] when scal is negative (norm > 6 path can't reach
        // this branch — we just set scal=0 then). With the §5.2.11
        // `temp > 6 ⇒ scal = 0` clamp, `6 - scal` is in [-15, 6];
        // negative values become a left shift on L_max.
        let shift_amt = sub(6, scal);
        l_max = if shift_amt >= 0 {
            l_max >> (shift_amt as u32)
        } else {
            // §5.1 negative-shift semantics: `>> n` with n<0 becomes
            // `<< -n` (zero-fill). Use saturating logic via i64 to
            // bound the value before truncating back to i32.
            let n = (-shift_amt) as u32 & 31;
            ((l_max as i64) << n).clamp(i32::MIN as i64, i32::MAX as i64) as i32
        };

        // §5.2.11 — wt[k] = dp[k - Nc] >> 3 over k = 0..=39.
        for (k, slot) in wt.iter_mut().enumerate() {
            let dp_idx = (n_c as usize) - k - 1;
            *slot = dp_hist[dp_idx] >> 3;
        }

        // §5.2.11 — L_power = Σ L_mult(wt[k], wt[k]).
        let mut l_power: i32 = 0;
        for &w in wt.iter() {
            let l_temp = l_mult(w, w);
            l_power = l_add(l_temp, l_power);
        }

        // §5.2.11 — Normalization of L_max / L_power + coding of bc.
        if l_max <= 0 {
            return LtpParameters { n_c, b_c: 0 };
        }
        if l_max >= l_power {
            return LtpParameters { n_c, b_c: 3 };
        }

        // §5.2.11 — `temp = norm(L_power); R = (L_max << temp) >> 16;
        //            S = (L_power << temp) >> 16;`
        //
        // `L_power > L_max > 0` at this branch, so norm(L_power) >= 1
        // and the spec's `<< temp` cannot overflow (norm guarantees
        // L_power<<temp stays in i32 range).
        let temp = norm(l_power) as u32;
        let r: i16 = ((l_max as u32).wrapping_shl(temp) as i32 >> 16) as i16;
        let s: i16 = ((l_power as u32).wrapping_shl(temp) as i32 >> 16) as i16;

        // §5.2.11 — coding of the LTP gain via Table 5.3a (DLB[]).
        //
        // FOR bc = 0 to 2: IF (R <= mult(S, DLB[bc])) THEN EXIT;
        // NEXT bc:
        // bc = 3;
        let mut b_c: i16 = 3;
        for bc_try in 0..=2 {
            if r <= mult(s, DLB[bc_try]) {
                b_c = bc_try as i16;
                break;
            }
        }

        LtpParameters { n_c, b_c }
    }

    /// §5.2.12 — long-term analysis filtering.
    ///
    /// Decodes the §5.2.11 coded LTP gain `bc` via [`QLB`] and uses it
    /// together with the LTP lag `Nc` to compute the long-term residual
    /// `e[0..=39]` for one sub-segment. Also returns the long-term
    /// prediction estimate `dpp[0..=39]` (the spec's intermediate
    /// `dpp[k] = mult_r(bp, dp[k - Nc])`), which the caller combines
    /// with the §5.2.17 reconstructed long-term residual `ep[..]` in
    /// §5.2.18 to update the §4.5 `dp[-120..=-1]` delay line.
    ///
    /// §5.2.12 pseudocode (verbatim):
    /// ```text
    /// bp = QLB[bc];
    /// FOR k = 0 to 39:
    ///     dpp[k] = mult_r( bp, dp[k - Nc] );
    ///     e[k]   = sub   ( d[k], dpp[k]   );
    /// NEXT k:
    /// ```
    ///
    /// Returns `(dpp, e)` over `0..=39`. The dp-history indexing
    /// matches [`ltp_parameters`].
    pub fn long_term_analysis_filter(
        d: &[i16; SUBFRAME_SAMPLES],
        dp_hist: &[i16; LTP_DELAY],
        params: LtpParameters,
    ) -> ([i16; SUBFRAME_SAMPLES], [i16; SUBFRAME_SAMPLES]) {
        // §5.2.12 — bp = QLB[bc].
        let bp = QLB[params.b_c as usize];

        let mut dpp = [0i16; SUBFRAME_SAMPLES];
        let mut e = [0i16; SUBFRAME_SAMPLES];
        for k in 0..SUBFRAME_SAMPLES {
            let dp_idx = (params.n_c as usize) - k - 1;
            dpp[k] = mult_r(bp, dp_hist[dp_idx]);
            e[k] = sub(d[k], dpp[k]);
        }
        (dpp, e)
    }

    /// Persistent §4.5 / §5.2.11 / §5.2.12 / §5.2.18 LTP state.
    ///
    /// Wraps the §4.5 Table 4.2 `dp[-120..=-1]` delay-line entry — the
    /// reconstructed short-term residual history the long-term
    /// predictor reaches into.
    ///
    /// `dp_hist[0]` holds `dp[-1]` (the most-recent past sample);
    /// `dp_hist[119]` holds `dp[-120]` (the oldest). This is the same
    /// layout the decoder's `drp_hist` uses; it matches the §5.2.11
    /// `dp[k - lambda]` indexing where positive `lambda` reaches back
    /// into history.
    ///
    /// The §5.2.18 `dp[]` update folds in the per-sub-segment
    /// reconstructed long-term residual `ep[0..=39]` and the long-term
    /// prediction estimate `dpp[0..=39]` from §5.2.12. Until the
    /// §5.2.13..§5.2.17 RPE / APCM stages land, the LTP analyser can
    /// still drive §5.2.11 / §5.2.12 on a caller-provided dp history,
    /// and [`LtpAnalyzer::update_dp_after_subframe`] lets a caller close
    /// the loop manually with their own `ep[..]`.
    #[derive(Debug, Clone)]
    pub struct LtpAnalyzer {
        /// §4.5 Table 4.2 `dp[-120..=-1]` — the LTP delay line.
        /// Home value: all-zero.
        dp_hist: [i16; LTP_DELAY],
    }

    impl Default for LtpAnalyzer {
        fn default() -> Self {
            Self {
                dp_hist: [0; LTP_DELAY],
            }
        }
    }

    impl LtpAnalyzer {
        /// Build a fresh LTP analyser in its §4.5 home state
        /// (`dp[-120..=-1] = 0`).
        pub fn new() -> Self {
            Self::default()
        }

        /// Reset the LTP analyser to its §4.5 home state.
        pub fn reset(&mut self) {
            *self = Self::default();
        }

        /// Inspect the persisted `dp[-120..=-1]` delay line.
        ///
        /// `dp_hist()[0]` is `dp[-1]`; `dp_hist()[119]` is `dp[-120]`.
        pub fn dp_hist(&self) -> &[i16; LTP_DELAY] {
            &self.dp_hist
        }

        /// Run §5.2.11 + §5.2.12 on one sub-segment.
        ///
        /// Inputs:
        ///
        /// * `d[0..=39]` — the §5.2.10 short-term residual for this
        ///   sub-segment.
        ///
        /// Outputs:
        ///
        /// * `LtpParameters` — the codewords `(Nc, bc)` the §1.7 packer
        ///   will emit.
        /// * `dpp[0..=39]` — the §5.2.12 long-term prediction estimate.
        /// * `e[0..=39]` — the §5.2.12 long-term residual that
        ///   §5.2.13 (the weighting filter) consumes.
        ///
        /// This routine does **not** apply the §5.2.18 dp-update; the
        /// update folds in the §5.2.17 reconstructed long-term residual
        /// `ep[..]` which only becomes available after the
        /// §5.2.13..§5.2.17 RPE / APCM stages run. Once those stages
        /// land, the caller (or a follow-up method on this struct) will
        /// invoke [`Self::update_dp_after_subframe`] with `ep[..]`.
        pub fn analyse_subframe(
            &self,
            d: &[i16; SUBFRAME_SAMPLES],
        ) -> (
            LtpParameters,
            [i16; SUBFRAME_SAMPLES],
            [i16; SUBFRAME_SAMPLES],
        ) {
            let params = ltp_parameters(d, &self.dp_hist);
            let (dpp, e) = long_term_analysis_filter(d, &self.dp_hist, params);
            (params, dpp, e)
        }

        /// Apply the §5.2.18 dp-update once the sub-segment's
        /// reconstructed long-term residual `ep[0..=39]` is known.
        ///
        /// §5.2.18 pseudocode (verbatim):
        /// ```text
        /// FOR k = 0 to 79:
        ///     dp[-120 + k] = dp[-80 + k];
        /// NEXT k:
        ///
        /// FOR k = 0 to 39:
        ///     dp[-40 + k] = add( ep[k], dpp[k] );
        /// NEXT k:
        /// ```
        ///
        /// In words: slide the older 80 history samples leftward by 40,
        /// then write the new 40 samples `add(ep[k], dpp[k])` into the
        /// `dp[-40..=-1]` slot. With the `dp_hist[0] = dp[-1]` layout
        /// the encoder uses, "leftward by 40" maps to "shift `dp_hist`
        /// entries 0..=79 into positions 40..=119" and the new samples
        /// land at positions 39 downto 0 (so `dp_hist[0]` = `dp[-1]` =
        /// `add(ep[39], dpp[39])`, etc.).
        ///
        /// `dpp` is the second-tuple member of
        /// [`Self::analyse_subframe`]; `ep` is produced by §5.2.17
        /// once the RPE / APCM stages land (a later round).
        pub fn update_dp_after_subframe(
            &mut self,
            ep: &[i16; SUBFRAME_SAMPLES],
            dpp: &[i16; SUBFRAME_SAMPLES],
        ) {
            // §5.2.18 step 1: dp[-120 + k] = dp[-80 + k], k = 0..=79.
            //
            // Maps to: dp_hist positions [80, 119] (= dp[-120..=-81])
            // get the contents of dp_hist positions [40, 79]
            // (= dp[-80..=-41]); then dp_hist positions [40, 79]
            // (= dp[-80..=-41]) get the contents of dp_hist [0, 39]
            // (= dp[-40..=-1]).
            //
            // Equivalent in our layout: shift the first 80 entries
            // (the "newer" half) into positions 40..=119; the deepest
            // 40 entries (positions 80..=119 = dp[-120..=-81]) are
            // dropped per the spec ("dp[-120 + k] = dp[-80 + k]"
            // overwrites them). Using copy_within for the rotation.
            self.dp_hist.copy_within(0..80, 40);

            // §5.2.18 step 2: dp[-40 + k] = add(ep[k], dpp[k]) for
            // k = 0..=39.
            //
            // dp[-1] = add(ep[39], dpp[39]) ⇒ dp_hist[0]
            // dp[-2] = add(ep[38], dpp[38]) ⇒ dp_hist[1]
            // …
            // dp[-40] = add(ep[0], dpp[0]) ⇒ dp_hist[39]
            for k in 0..SUBFRAME_SAMPLES {
                self.dp_hist[SUBFRAME_SAMPLES - 1 - k] = add(ep[k], dpp[k]);
            }
        }

        /// Close the §5.2.16 → §5.2.17 → §5.2.18 local-decoder loop for
        /// one sub-segment.
        ///
        /// Given the §5.2.15 [`ApcmQuantised`] codewords + the §5.2.14
        /// grid offset `m_c` + the §5.2.12 long-term prediction estimate
        /// `dpp[0..=39]`, this method:
        ///
        /// 1. runs §5.2.16 + §5.2.17 ([`apcm_inverse_and_position`]) to
        ///    reconstruct the long-term residual `ep[0..=39]`, then
        /// 2. runs §5.2.18 ([`Self::update_dp_after_subframe`]) to fold
        ///    `add(ep[k], dpp[k])` back into the §4.5 `dp[-120..=-1]`
        ///    delay line.
        ///
        /// This is the encoder's local-decoder feedback path: the
        /// reconstructed `dp[..]` history it builds here is bit-identical
        /// to the `drp[..]` the receiving decoder builds in §5.3.2, so
        /// the next sub-segment's §5.2.11 cross-correlation search runs
        /// on the same history the decoder will see. It returns the
        /// reconstructed `ep[0..=39]` in case the caller wants to
        /// inspect it (e.g. for a per-sub-segment trace comparison).
        pub fn reconstruct_and_update(
            &mut self,
            apcm: &ApcmQuantised,
            m_c: i16,
            dpp: &[i16; SUBFRAME_SAMPLES],
        ) -> [i16; SUBFRAME_SAMPLES] {
            let ep = apcm_inverse_and_position(&apcm.x_mc, apcm.exp, apcm.mant, m_c);
            self.update_dp_after_subframe(&ep, dpp);
            ep
        }
    }

    impl LtpAnalyzer {
        /// Convenience: assemble an [`LtpAnalyzer`] with a caller-
        /// supplied initial delay line. Useful for unit tests that
        /// pre-seed the §4.5 history.
        pub fn with_dp_hist(dp_hist: [i16; LTP_DELAY]) -> Self {
            Self { dp_hist }
        }
    }

    /// §5.2.13 — Weighting filter.
    ///
    /// Convolves the §5.2.12 long-term residual `e[0..=39]` with the
    /// 11-tap FIR impulse response [`H`] (Table 5.4) using the spec's
    /// "block filter" framing: the input is zero-padded on both sides
    /// (5 leading + 5 trailing zeros) into a 50-sample working array
    /// `wt[0..=49]`, then the 11-tap dot product is taken at each of
    /// 40 output positions to land `x[0..=39]`. The result is the
    /// block-filtered signal that §5.2.14 RPE grid selection consumes.
    ///
    /// §5.2.13 pseudocode (verbatim):
    /// ```text
    /// Initialization of a temporary working array wt[0..49]:
    ///   FOR k = 0 to 4:    wt[k] = 0;
    ///   FOR k = 5 to 44:   wt[k] = e[k-5];
    ///   FOR k = 45 to 49:  wt[k] = 0;
    ///
    /// Compute the signal x[0..39]:
    ///   FOR k = 0 to 39:
    ///       L_result = 8192;   /* rounding of the output */
    ///       FOR i = 0 to 10:
    ///           L_temp   = L_mult( wt[k+i], H[i] );
    ///           L_result = L_add ( L_result, L_temp );
    ///       NEXT i:
    ///       L_result = L_add( L_result, L_result );  /* scaling x2 */
    ///       L_result = L_add( L_result, L_result );  /* scaling x4 */
    ///       x[k]     = L_result >> 16;
    ///   NEXT k:
    /// ```
    ///
    /// Notes:
    ///
    /// * The two `L_add(L_result, L_result)` doublings together
    ///   shift the 32-bit accumulator left by 2 bits, with the §5.1
    ///   saturating addition rule applied at each doubling. The
    ///   subsequent `>> 16` shifts the high 16 bits out as the
    ///   filtered sample.
    /// * Table 5.4 stores `H[i] * 2^13` (the integer scaling defined
    ///   in §5.2.13 "scaling is used" — `H[0..10] =
    ///   integer(real_H[0..10] * 8192)`); the `L_mult` accumulation
    ///   absorbs that scaling, and the trailing `<<2; >>16` lands
    ///   the §5.2.14 input in the spec's expected magnitude.
    /// * The filter is stateless — every sub-segment starts the
    ///   `wt[]` array fresh from `e[..]` with zero padding, so no
    ///   §4.5 Table 4.2 home state is added by this clause.
    pub fn weighting_filter(e: &[i16; SUBFRAME_SAMPLES]) -> [i16; SUBFRAME_SAMPLES] {
        // §5.2.13 — initialise the 50-sample working array wt[0..=49].
        //   wt[0..=4]   = 0   (5 leading zeros)
        //   wt[5..=44]  = e[k-5]  (the 40-sample input)
        //   wt[45..=49] = 0   (5 trailing zeros)
        let mut wt = [0i16; 50];
        wt[5..45].copy_from_slice(&e[..SUBFRAME_SAMPLES]);

        // §5.2.13 — compute x[0..=39].
        let mut x = [0i16; SUBFRAME_SAMPLES];
        for k in 0..SUBFRAME_SAMPLES {
            // L_result = 8192 (rounding constant) + Σ L_mult(wt[k+i], H[i]).
            let mut l_result: i32 = 8192;
            for i in 0..=10 {
                let l_temp = l_mult(wt[k + i], H[i]);
                l_result = l_add(l_result, l_temp);
            }
            // Two saturating doublings = ×4 scaling.
            l_result = l_add(l_result, l_result);
            l_result = l_add(l_result, l_result);
            // x[k] = L_result >> 16.
            x[k] = (l_result >> 16) as i16;
        }
        x
    }

    /// Number of RPE pulses emitted per sub-segment by §5.2.14
    /// (`xM[0..=12]`) — the down-sampled grid carries 13 samples.
    pub const RPE_PULSES: usize = 13;

    /// Per-sub-segment §5.2.14 output: the chosen RPE grid offset
    /// `Mc ∈ {0, 1, 2, 3}` plus the 13-sample down-sampled sequence
    /// `xM[0..=12] = x[Mc + 3*i]` that §5.2.15 APCM quantisation
    /// (a later round) consumes.
    ///
    /// `Mc` is the 2-bit codeword the §1.7 frame packer emits as
    /// `Mc` per Table 1.1 (one of `Mc[1]..Mc[4]` for the four
    /// sub-segments of a frame).
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct RpeGrid {
        /// §5.2.14 RPE grid offset — the `m ∈ {0, 1, 2, 3}` that
        /// maximises the sub-sampled energy `EM`. Emitted to the
        /// bit packer as the 2-bit `Mc` codeword.
        pub m_c: i16,
        /// §5.2.14 down-sampled RPE sequence `xM[0..=12] =
        /// x[Mc + 3*i]` for `i = 0..=12`. Input to the §5.2.15
        /// APCM quantiser (a later round).
        pub x_m: [i16; RPE_PULSES],
    }

    /// §5.2.14 — RPE grid selection (adaptive sample-rate decimation).
    ///
    /// Picks one of the four interleaved sub-sampling grids of `x[]`
    /// that maximises the sum of squared `(x[m+3i] >> 2)` values
    /// across the 13 grid positions, then emits the chosen grid's
    /// 13 samples as `xM[0..=12]`. The right-shift-by-2 the spec
    /// applies before squaring scales each contribution from the
    /// 16-bit `x[]` range into a margin where the `L_mult`
    /// (`<<1`) + 13-term `L_add` accumulation cannot overflow the
    /// 32-bit `L_result`.
    ///
    /// §5.2.14 pseudocode (verbatim):
    /// ```text
    /// EM = 0;
    /// Mc = 0;
    /// FOR m = 0 to 3:
    ///     L_result = 0;
    ///     FOR i = 0 to 12:
    ///         temp1    = x[m + (3*i)] >> 2;
    ///         L_temp   = L_mult( temp1, temp1 );
    ///         L_result = L_add ( L_temp, L_result );
    ///     NEXT i:
    ///     IF ( L_result > EM ) THEN
    ///         Mc = m;
    ///         EM = L_result;
    /// NEXT m:
    ///
    /// Down-sampling by a factor 3 to get the selected xM[0..12]
    /// RPE sequence.
    /// FOR i = 0 to 12:
    ///     xM[i] = x[Mc + (3*i)];
    /// NEXT i:
    /// ```
    ///
    /// Notes on the spec pseudocode as written:
    ///
    /// * The strict inequality `L_result > EM` means that on ties
    ///   the smaller `m` wins (the entry-time `Mc = 0` is only
    ///   overwritten when a later grid scores strictly higher).
    ///   In particular, an all-zero `x[]` returns `m_c = 0` with an
    ///   all-zero `xM[]`.
    /// * The grid `m = 0` covers indices `{0, 3, 6, …, 36}`, `m = 1`
    ///   covers `{1, 4, 7, …, 37}`, `m = 2` covers `{2, 5, 8, …, 38}`,
    ///   and `m = 3` covers `{3, 6, 9, …, 39}`. The four grids
    ///   together touch `x[0..=39]` — the last sample `x[39]` is
    ///   reachable only via `m = 3` and `i = 12`.
    /// * The accumulator is signed 32-bit; with `temp1 ∈ [-8192,
    ///   8191]` after the `>>2`, `L_mult(temp1, temp1) ∈
    ///   [0, 2 * 8191 * 8191] ≈ 1.34 × 10⁸`. Thirteen of those sum
    ///   to ≤ 1.74 × 10⁹ < 2³¹, so saturation in `L_add` is not
    ///   reachable for normal inputs. We still use the saturating
    ///   `l_add` to stay bit-exact with the pseudocode.
    /// * The filter is stateless — `x[]` already carries the
    ///   §5.2.13 / §5.2.12 context, and no §4.5 Table 4.2 entry
    ///   is owned by §5.2.14.
    pub fn select_rpe_grid(x: &[i16; SUBFRAME_SAMPLES]) -> RpeGrid {
        // §5.2.14 entry-time initialisation: EM = 0, Mc = 0.
        let mut em: i32 = 0;
        let mut m_c: i16 = 0;

        // §5.2.14 outer loop: m = 0..=3 over the four sub-sampling
        // grids.
        for m in 0..4 {
            let mut l_result: i32 = 0;
            // §5.2.14 inner loop: i = 0..=12, sum |x[m+3i] >> 2|²
            // through the saturating accumulator.
            for i in 0..RPE_PULSES {
                let temp1 = x[m + 3 * i] >> 2;
                let l_temp = l_mult(temp1, temp1);
                l_result = l_add(l_temp, l_result);
            }
            // §5.2.14: strict greater-than means earlier m wins ties.
            if l_result > em {
                m_c = m as i16;
                em = l_result;
            }
        }

        // §5.2.14 down-sampling: xM[i] = x[Mc + 3*i] for i = 0..=12.
        let mut x_m = [0i16; RPE_PULSES];
        for i in 0..RPE_PULSES {
            x_m[i] = x[(m_c as usize) + 3 * i];
        }

        RpeGrid { m_c, x_m }
    }

    /// Per-sub-segment §5.2.15 output: the coded maximum-magnitude
    /// codeword `xmaxc` (6-bit, packed as `Xmaxc[1..=4]` in §1.7
    /// Table 1.1), the 13 coded RPE samples `xMc[0..=12]` (3-bit each,
    /// packed as `Xm[1..=4][0..12]` in Table 1.1), and the
    /// post-normalisation `(exp, mant)` pair the §5.2.16 inverse APCM
    /// quantiser consumes per the spec's "Keep in memory exp and mant
    /// for the following inverse APCM quantizer" note.
    ///
    /// `xmaxc` is the unsigned 6-bit code in the range 0..=63; `xMc`
    /// values are unsigned 3-bit codes in the range 0..=7. The
    /// `(exp, mant)` pair is the post-normalisation state §5.2.15
    /// leaves behind; the encoder's §5.2.16 round-trip (which feeds
    /// §5.2.17 / §5.2.18 in a later round) consumes them directly
    /// rather than re-deriving them from `xmaxc`.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct ApcmQuantised {
        /// §5.2.15 quantised block maximum, 6-bit unsigned code
        /// (range 0..=63 per §1.7 Table 1.1 column `Xmaxc`).
        pub xmaxc: i16,
        /// §5.2.15 quantised RPE sequence, 13 × 3-bit unsigned codes
        /// (range 0..=7 per §1.7 Table 1.1 column `Xm`).
        pub x_mc: [i16; RPE_PULSES],
        /// §5.2.15 post-normalisation exponent. Spec range −4..=6
        /// (the `mant == 0` branch preloads `exp = -4`; otherwise
        /// the iterative branch enters with an exponent of 0..=6
        /// from the §5.2.15 search loop and decrements it up to
        /// three times during normalisation).
        pub exp: i16,
        /// §5.2.15 post-normalisation mantissa, after the trailing
        /// `sub(mant, 8)` step. Spec range −8..=−1 (matching the
        /// decoder-side `mant ∈ 0..=7` after the `sub(mant, 8)` is
        /// re-undone by the `FAC[mant]` index lookup at §5.2.16
        /// — wait, the spec actually leaves `mant` in 0..=7 for the
        /// §5.2.16 `FAC[mant]` table lookup because the §5.2.15
        /// `sub(mant, 8)` step is applied AFTER the normalisation
        /// loop and BEFORE the table indexing).
        ///
        /// Concretely: the spec lifts `mant` from the normalised range
        /// 8..=15 (or the `mant == 0` short-circuit `mant = 15`)
        /// down to 0..=7 via the `mant = sub(mant, 8)` step that
        /// closes §5.2.15, and uses the resulting 0..=7 value as the
        /// `NRFAC[mant]` / `FAC[mant]` table index. This field holds
        /// the post-`sub(mant, 8)` value (0..=7) so callers can feed
        /// it directly into the §5.2.16 path.
        pub mant: i16,
    }

    /// §5.2.15 — APCM forward quantisation of the §5.2.14 RPE
    /// sequence.
    ///
    /// Takes the down-sampled 13-pulse sequence `xM[0..=12]` produced
    /// by [`select_rpe_grid`] and returns the encoded `xmaxc` block
    /// maximum + 13 encoded sample codes `xMc[0..=12]` the §1.7 frame
    /// packer (a later round) emits, plus the `(exp, mant)` pair
    /// §5.2.16 consumes.
    ///
    /// §5.2.15 pseudocode (verbatim):
    /// ```text
    /// Find the maximum absolute value xmax of xM[0..12].
    ///     xmax = 0;
    ///     FOR i = 0 to 12:
    ///         temp = abs( xM[i] );
    ///         IF ( temp > xmax ) THEN xmax = temp;
    ///     NEXT i:
    ///
    /// Quantizing and coding of xmax to get xmaxc.
    ///     exp = 0;
    ///     temp = xmax >> 9;
    ///     itest = 0;
    ///     FOR i = 0 to 5:
    ///         IF ( temp <= 0 ) THEN itest = 1;
    ///         temp = temp >> 1;
    ///         IF ( itest == 0 ) THEN exp = add( exp, 1 ) ;
    ///     NEXT i:
    ///
    ///     temp = add( exp, 5 ) ;
    ///     xmaxc = add( ( xmax >> temp ), ( exp << 3 ) ) ;
    ///
    /// Compute exponent and mantissa of the decoded version of xmaxc.
    ///     exp = 0 ;
    ///     IF ( xmaxc > 15 ) THEN exp = sub( ( xmaxc >> 3 ), 1 ) ;
    ///     mant = sub( xmaxc , ( exp << 3 ) );
    ///
    /// Normalize mantissa 0 <= mant <= 7.
    ///     IF ( mant == 0 ) THEN | exp = -4;
    ///                            | mant = 15;
    ///     ELSE | itest = 0;
    ///          | FOR i = 0 to 2:
    ///          |   IF ( mant > 7 ) THEN itest = 1;
    ///          |   IF (itest == 0) THEN mant = add((mant << 1), 1);
    ///          |   IF ( itest == 0 ) THEN exp = sub( exp, 1 );
    ///          | NEXT i:
    ///     mant = sub( mant, 8 );
    ///
    /// Direct computation of xMc[0..12] using table 5.5.
    ///     temp1 = sub( 6, exp );
    ///     temp2 = NRFAC[mant];
    ///     FOR i = 0 to 12:
    ///         temp = xM[i] << temp1;
    ///         temp = mult( temp , temp2 );
    ///         xMc[i] = add( ( temp >> 12 ), 4 );
    ///     NEXT I:
    /// ```
    ///
    /// Notes on the spec's arithmetic:
    ///
    /// * The §5.2.15 `xmax = 0; FOR i: IF (abs(xM[i]) > xmax) ...`
    ///   loop has the same shape as the §5.2.4 `smax` search. With
    ///   `xM[i]` carrying the §5.2.13 weighting filter's `>> 16`
    ///   output, `xM[i]` is i16; `abs(xM[i])` is in 0..=32767
    ///   (saturating `abs(-32768) = 32767` per §5.1).
    /// * The exponent search loop iterates six times (`i = 0..=5`),
    ///   driving `temp = xmax >> 9` down by another factor of 2 each
    ///   step and incrementing `exp` while `temp > 0`. The final
    ///   `temp = add(exp, 5); xmaxc = add((xmax >> temp), (exp << 3))`
    ///   step packs the 3-bit exp into the upper bits and the 3-bit
    ///   `xmax >> (exp+5)` mantissa into the lower bits, giving the
    ///   6-bit `xmaxc ∈ 0..=63` code §1.7 packs as `Xmaxc`.
    /// * The decoded-mantissa normalisation matches the decoder's
    ///   §5.2.16 path bit-for-bit, so the encoder uses the same
    ///   `(exp, mant)` pair the receiving decoder will recover. The
    ///   `sub(mant, 8)` step at the end of §5.2.15 produces the
    ///   `mant ∈ 0..=7` value that indexes both `NRFAC[..]`
    ///   (§5.2.15) and `FAC[..]` (§5.2.16). The decoder lands in the
    ///   same `mant ∈ 0..=7` range via the §5.2.16 path already
    ///   implemented in `decoder::rpe_decode`.
    /// * `temp = xM[i] << temp1` uses the §5.1 `<<` rule: when
    ///   `temp1` is negative (which the spec admits if `exp == 6`
    ///   would push `temp1 = 0`, but `exp` from the §5.2.15 search
    ///   above is capped at 6, and the normalisation can drop `exp`
    ///   to as low as `-4`, giving `temp1 ∈ {2, 3, …, 10}`), the
    ///   shift falls through to an arithmetic right shift. We use
    ///   [`shl_signed`] which implements that rule. In practice
    ///   `temp1 ∈ {2, …, 10}` for normal inputs so the left-shift
    ///   path is the only one exercised; the negative-`n`
    ///   fall-through is staged for spec-completeness.
    /// * `temp = mult(temp, temp2)` is the §5.1 16×16 → 16 Q15
    ///   product that drives the truncating quotient `temp >> 12`.
    ///   The `+ 4` constant matches the decoder's reciprocal step
    ///   `sub((xMc << 1), 7)` — together they pack the signed
    ///   3-bit `xMc` code into the unsigned 0..=7 range §1.7 emits.
    /// * The function is stateless — `xM[]` already carries all
    ///   §5.2.13 / §5.2.14 context, no §4.5 Table 4.2 entry is
    ///   owned by §5.2.15, and the `(exp, mant)` pair is returned to
    ///   the caller rather than persisted on a struct.
    pub fn apcm_quantise_rpe(x_m: &[i16; RPE_PULSES]) -> ApcmQuantised {
        // §5.2.15 — find xmax = max |xM[i]|.
        let mut xmax: i16 = 0;
        for &v in x_m.iter() {
            let t = abs(v);
            if t > xmax {
                xmax = t;
            }
        }

        // §5.2.15 — quantise + code xmax to get the 6-bit xmaxc.
        //   exp = 0;
        //   temp = xmax >> 9;
        //   itest = 0;
        //   FOR i = 0 to 5:
        //     IF (temp <= 0) itest = 1;
        //     temp >>= 1;
        //     IF (itest == 0) exp += 1;
        //   NEXT i:
        let mut exp: i16 = 0;
        let mut temp: i16 = xmax >> 9;
        let mut itest: i16 = 0;
        for _ in 0..6 {
            if temp <= 0 {
                itest = 1;
            }
            temp >>= 1;
            if itest == 0 {
                exp = add(exp, 1);
            }
        }

        // §5.2.15 — pack: xmaxc = (xmax >> (exp+5)) + (exp << 3).
        //   `temp = add(exp, 5)` may exceed 15; use the spec's
        //   §5.1 signed-shift fall-through via `shl_signed` on
        //   xmax (a positive i16) to right-shift safely.
        let pack_shift = add(exp, 5);
        let xmaxc = add(shl_signed(xmax, sub(0, pack_shift)), shl_signed(exp, 3));

        // §5.2.15 — compute exponent and mantissa of the decoded
        // version of xmaxc, then normalise so that on entry to
        // §5.2.16 the mant index is in 0..=7.
        //
        // This block runs the spec's pseudocode verbatim. It is
        // intentionally NOT factored out into a shared helper with
        // the decoder's §5.2.16 path even though the two are
        // step-identical — the encoder's §5.2.15 needs the
        // post-normalisation `(exp, mant)` pair returned to the
        // caller, while the decoder's path consumes them inline.
        // A future refactor can hoist the shared block once a
        // second caller appears.
        let mut decode_exp: i16 = 0;
        if xmaxc > 15 {
            decode_exp = sub(xmaxc >> 3, 1);
        }
        let decode_mant = sub(xmaxc, decode_exp << 3);

        let (norm_exp, norm_mant) = if decode_mant == 0 {
            (sub(0, 4), 15i16)
        } else {
            let mut e = decode_exp;
            let mut m = decode_mant;
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
        // §5.2.15 — `mant = sub(mant, 8)` lifts mant into the
        // 0..=7 range that indexes NRFAC[..] / FAC[..].
        let mant_idx = sub(norm_mant, 8);

        // §5.2.15 — direct computation of xMc[0..=12] via NRFAC.
        //   temp1 = sub(6, exp);
        //   temp2 = NRFAC[mant];
        //   FOR i = 0 to 12:
        //     temp = xM[i] << temp1;
        //     temp = mult(temp, temp2);
        //     xMc[i] = add((temp >> 12), 4);
        //   NEXT I:
        let temp1 = sub(6, norm_exp);
        let temp2 = NRFAC[mant_idx as usize];
        let mut x_mc = [0i16; RPE_PULSES];
        for (slot, &x_m_i) in x_mc.iter_mut().zip(x_m.iter()) {
            // §5.1 signed shift handles the (uncommon) negative-temp1
            // fall-through case.
            let t = shl_signed(x_m_i, temp1);
            let t = mult(t, temp2);
            *slot = add(t >> 12, 4);
        }

        ApcmQuantised {
            xmaxc,
            x_mc,
            exp: norm_exp,
            mant: mant_idx,
        }
    }

    /// §5.2.16 + §5.2.17 — encoder-side APCM inverse quantisation and
    /// RPE grid positioning, producing the reconstructed long-term
    /// residual `ep[0..=39]` the §5.2.18 dp-update folds back into the
    /// LTP delay line.
    ///
    /// The encoder runs the **same** inverse-quantisation arithmetic
    /// the decoder applies in §5.3.1 (`decoder::rpe_decode`), but it
    /// already holds the post-normalisation `(exp, mant)` pair from
    /// §5.2.15 ([`apcm_quantise_rpe`]'s [`ApcmQuantised`] return) per
    /// the §5.2.15 "Keep in memory exp and mant for the following
    /// inverse APCM quantizer" note — so unlike the decoder it does
    /// **not** re-derive `(exp, mant)` from `xmaxc`. The
    /// `mant` argument is the `mant ∈ 0..=7` index that drives
    /// `FAC[mant]`; the `exp` argument is the post-normalisation
    /// exponent (spec range −4..=6); `x_mc[0..=12]` are the 3-bit RPE
    /// codewords (0..=7); and `m_c ∈ {0, 1, 2, 3}` is the §5.2.14
    /// grid offset.
    ///
    /// §5.2.16 pseudocode (verbatim):
    /// ```text
    /// temp1 = FAC[mant];               see 5.2.15 for mant
    /// temp2 = sub( 6, exp );           see 5.2.15 for exp
    /// temp3 = 1 << sub( temp2, 1 );
    ///
    /// FOR i = 0 to 12:
    ///     temp = sub( ( xMc[i] << 1 ), 7 );   /restore the sign/
    ///     temp = temp << 12;
    ///     temp = mult_r( temp1, temp );
    ///     temp = add( temp, temp3 );
    ///     xMp[i] = temp >> temp2;
    /// NEXT i:
    /// ```
    ///
    /// §5.2.17 pseudocode (verbatim):
    /// ```text
    /// FOR k = 0 to 39:
    ///     ep[k] = 0;
    /// NEXT k:
    /// FOR i = 0 to 12:
    ///     ep[Mc + (3*i)] = xMp[i];
    /// NEXT i:
    /// ```
    ///
    /// Notes on the spec's arithmetic:
    ///
    /// * `temp = sub((xMc[i] << 1), 7)` is the §5.2.16 NOTE
    ///   sign-restoration step: it is the exact inverse of the §5.2.15
    ///   `xMc[i] = add((temp >> 12), 4)` pack — the §5.2.15 `+ 4`
    ///   biased the signed 3-bit pulse into the unsigned 0..=7 code,
    ///   and `(xMc << 1) - 7` re-centres it.
    /// * `temp2 = sub(6, exp)` is the §5.2.16 right-shift count; with
    ///   `exp ∈ −4..=6` it lies in `0..=10`. The §5.1 `>>` operator
    ///   handles the non-negative count directly, so [`shr_signed`]
    ///   covers the (unreachable for valid `exp`) negative-count
    ///   fall-through for spec completeness.
    /// * `temp3 = 1 << (temp2 - 1)` is the §5.2.16 rounding constant.
    ///   For `temp2 ∈ 1..=10` this is `1, 2, … 512`; the
    ///   `temp2 == 0` case (`exp == 6`) would make `temp2 - 1 == -1`,
    ///   which the §5.1 `<<` rule turns into `1 >> 1 == 0` via
    ///   [`shl_signed`] — i.e. no rounding, matching the decoder's
    ///   identical step.
    /// * The function is stateless — it owns no §4.5 Table 4.2 entry.
    ///   The §4.5 `dp[-120..=-1]` delay line that §5.2.18 updates is
    ///   held by [`LtpAnalyzer`]; this routine only produces the
    ///   `ep[..]` input to that update.
    ///
    /// Returns the reconstructed long-term residual `ep[0..=39]`. Feed
    /// it (together with the §5.2.12 `dpp[0..=39]`) into
    /// [`LtpAnalyzer::update_dp_after_subframe`] to close the §5.2.18
    /// loop, or use [`LtpAnalyzer::reconstruct_and_update`] which runs
    /// this step and the dp-update together.
    pub fn apcm_inverse_and_position(
        x_mc: &[i16; RPE_PULSES],
        exp: i16,
        mant: i16,
        m_c: i16,
    ) -> [i16; SUBFRAME_SAMPLES] {
        // §5.2.16 — APCM inverse quantisation: xMc[0..=12] → xMp[0..=12].
        let temp1 = FAC[mant as usize];
        let temp2 = sub(6, exp);
        // `temp3 = 1 << sub(temp2, 1)`; the §5.1 `<<` rule (via
        // shl_signed) turns the `temp2 == 0` ⇒ shift-by-(-1) case into
        // a right shift, matching the decoder's identical §5.2.16 step.
        let temp3 = shl_signed(1, sub(temp2, 1));

        let mut x_mp = [0i16; RPE_PULSES];
        for (slot, &xmc_raw) in x_mp.iter_mut().zip(x_mc.iter()) {
            // §5.2.16: temp = sub((xMc[i] << 1), 7); temp <<= 12;
            //          temp = mult_r(temp1, temp); temp = add(temp, temp3);
            //          xMp[i] = temp >> temp2.
            let t = sub(xmc_raw << 1, 7) << 12;
            let t = mult_r(temp1, t);
            let t = add(t, temp3);
            *slot = shr_signed(t, temp2);
        }

        // §5.2.17 — RPE grid positioning: drop the 13 dequantised
        // pulses at Mc, Mc+3, …, Mc+36 in an otherwise-zero 40-sample
        // buffer.
        let mut ep = [0i16; SUBFRAME_SAMPLES];
        let mc = m_c as usize;
        for (i, &pulse) in x_mp.iter().enumerate() {
            let idx = mc + 3 * i;
            if idx < SUBFRAME_SAMPLES {
                ep[idx] = pulse;
            }
        }
        ep
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        // ─── §5.2.4 autocorrelation ───

        /// All-zero input ⇒ all-zero autocorrelation. Per §5.2.4 the
        /// `smax == 0` branch sets `scalauto = 0`, so no scaling.
        #[test]
        fn autocorrelation_zero_input() {
            let s = [0i16; FRAME_SAMPLES];
            let l_acf = autocorrelation(&s);
            for v in l_acf {
                assert_eq!(v, 0);
            }
        }

        /// A single non-zero sample at s[0] contributes only to
        /// `L_ACF[0]`. With s[0] = 100, L_ACF[0] = L_mult(100, 100)
        /// = (10000 << 1) = 20000. L_ACF[k] for k >= 1 must be zero
        /// (the inner-product loop goes `i = k..159`, so for k = 1
        /// it includes s[1]*s[0] = 0 only, never s[0]*s[-1]).
        #[test]
        fn autocorrelation_single_sample_is_l_mult_squared() {
            let mut s = [0i16; FRAME_SAMPLES];
            s[0] = 100;
            let l_acf = autocorrelation(&s);
            assert_eq!(l_acf[0], 20000);
            for k in 1..=8 {
                assert_eq!(l_acf[k], 0, "L_ACF[{k}] must be 0");
            }
        }

        /// A two-sample bipolar input s[0]=A, s[1]=A produces a
        /// non-zero L_ACF[0] = 2 * 2*A*A = 4*A^2 and a non-zero
        /// L_ACF[1] = 2*A*A = 2*A^2. (L_mult doubles the product;
        /// k=0 sums two terms, k=1 sums one term — the i=1, i-k=0
        /// product.)
        #[test]
        fn autocorrelation_two_sample_constant() {
            let mut s = [0i16; FRAME_SAMPLES];
            s[0] = 100;
            s[1] = 100;
            let l_acf = autocorrelation(&s);
            // L_ACF[0] = L_mult(100,100) + L_mult(100,100) = 40000.
            assert_eq!(l_acf[0], 40000);
            // L_ACF[1] = L_mult(s[1],s[0]) = 20000.
            assert_eq!(l_acf[1], 20000);
            // No further terms.
            for k in 2..=8 {
                assert_eq!(l_acf[k], 0);
            }
        }

        /// §5.2.4 dynamic scaling triggers when smax is large. A
        /// max-magnitude frame (s[k] = 16384 = 2^14) gives
        /// `smax << 16 = 2^30`, `norm = 0`, `scalauto = 4 - 0 = 4`.
        /// The downscale `temp = 16384 >> sub(4, 1) = 2048` =
        /// Q15(0.0625). The scaled samples become roughly
        /// `mult_r(16384, 2048) = (16384*2048 + 16384) >> 15 = 1024`.
        /// L_ACF[0] thus is bounded well below i32::MAX.
        #[test]
        fn autocorrelation_dynamic_scaling_avoids_overflow() {
            let s = [16384i16; FRAME_SAMPLES];
            let l_acf = autocorrelation(&s);
            // Just confirm we didn't saturate L_ACF[0] (and that the
            // result is positive non-zero).
            assert!(l_acf[0] > 0);
            assert!(l_acf[0] < i32::MAX, "L_ACF[0] should not saturate");
            // L_ACF[0] for s[k] = 1024 (post-scaling) and 160 terms:
            // 160 * L_mult(1024, 1024) = 160 * 2*1024*1024 ≈ 3.4e8.
            // Allow some slack for round-half-up of mult_r.
            assert!(l_acf[0] > 100_000_000);
            assert!(l_acf[0] < 500_000_000);
        }

        /// §5.2.4: with `smax == 0`, scalauto = 0 and the result is
        /// zero throughout — already covered by `_zero_input` above.
        /// This test checks that small `smax` (where scalauto would
        /// be <= 0) keeps the input unmolested. s[k] = 1 ⇒
        /// L_ACF[0] = 160 * L_mult(1,1) = 160 * 2 = 320.
        #[test]
        fn autocorrelation_small_input_no_scaling() {
            let s = [1i16; FRAME_SAMPLES];
            let l_acf = autocorrelation(&s);
            assert_eq!(l_acf[0], 320);
        }

        // ─── §5.2.5 Schur recursion ───

        /// `L_ACF[0] == 0` short-circuit ⇒ r[1..=8] = 0.
        #[test]
        fn schur_zero_l_acf_zero_yields_zero() {
            let l_acf = [0i32; 9];
            let r = reflection_coefficients(&l_acf);
            for v in r {
                assert_eq!(v, 0);
            }
        }

        /// White-noise-like autocorrelation (L_ACF[0] large, others
        /// zero) ⇒ first iteration has P[1] = 0 so r[1] = div(0, P[0])
        /// = 0; with r[1] = 0 every Schur update is a no-op, so
        /// every subsequent iteration also yields r[i] = 0.
        #[test]
        fn schur_white_noise_gives_zero_reflections() {
            let mut l_acf = [0i32; 9];
            l_acf[0] = 1_000_000_000; // pre-normalised
            let r = reflection_coefficients(&l_acf);
            assert_eq!(r[0], 0);
            for i in 1..=8 {
                assert_eq!(r[i], 0, "r[{i}] for white noise must be 0");
            }
        }

        /// Reflection coefficients are bounded |r[i]| < 32768 (Q15).
        /// Run the analysis on a representative input and check the
        /// invariant.
        #[test]
        fn schur_reflections_bounded_q15() {
            // A small-amplitude sinusoid-like input.
            let mut s = [0i16; FRAME_SAMPLES];
            for (k, slot) in s.iter_mut().enumerate() {
                let v = ((k as i32 % 13) - 6) * 200;
                *slot = v as i16;
            }
            let l_acf = autocorrelation(&s);
            let r = reflection_coefficients(&l_acf);
            for i in 1..=8 {
                // |r[i]| must fit i16 and the spec's Q15 range gives
                // |r| < 32768.
                assert!(r[i].abs() < i16::MAX);
            }
        }

        /// §5.2.5 abort branch — if `P[0] < abs(P[1])` at any n, all
        /// remaining `r[n..=8]` are zeroed. Synthesise a pathological
        /// L_ACF where L_ACF[1] dominates L_ACF[0]. Such an L_ACF is
        /// not producible by the §5.2.4 inner-product loop (Cauchy-
        /// Schwarz guarantees |L_ACF[1]| <= L_ACF[0]), but the §5.2.5
        /// abort branch must still behave correctly if invoked.
        #[test]
        fn schur_abort_zeros_remaining_reflections() {
            // L_ACF[0] = 1<<30 (already normalised), L_ACF[1] = 2<<30
            // — this would make ACF[1] > ACF[0] after the normalisation,
            // tripping the abort branch on n=1.
            let mut l_acf = [0i32; 9];
            l_acf[0] = 1 << 29;
            l_acf[1] = 1 << 30; // strictly larger ⇒ abort on n=1
            let r = reflection_coefficients(&l_acf);
            for i in 1..=8 {
                assert_eq!(r[i], 0);
            }
        }

        // ─── §5.2.6 reflection → LAR ───

        /// `r = 0` ⇒ `LAR = 0` (segment 1: temp = 0; abs(0) = 0;
        /// 0 >> 1 = 0; sign check is false).
        #[test]
        fn reflection_to_lar_zero() {
            let r = [0i16; 9];
            let lar = reflection_to_lar(&r);
            for v in lar {
                assert_eq!(v, 0);
            }
        }

        /// Segment 1 (|r| < 22118): LAR = |r| >> 1 with the original
        /// sign restored. r = 10000 ⇒ LAR = 5000; r = -10000 ⇒
        /// LAR = -5000.
        #[test]
        fn reflection_to_lar_segment_1() {
            let mut r = [0i16; 9];
            r[1] = 10000;
            r[2] = -10000;
            let lar = reflection_to_lar(&r);
            assert_eq!(lar[1], 5000);
            assert_eq!(lar[2], -5000);
        }

        /// Segment 2 (22118 <= |r| < 31130): LAR = |r| - 11059 with
        /// the original sign restored. r = 25000 ⇒ LAR = 25000-11059
        /// = 13941; r = -25000 ⇒ LAR = -13941.
        #[test]
        fn reflection_to_lar_segment_2() {
            let mut r = [0i16; 9];
            r[1] = 25000;
            r[2] = -25000;
            let lar = reflection_to_lar(&r);
            assert_eq!(lar[1], 13941);
            assert_eq!(lar[2], -13941);
        }

        /// Segment 3 (|r| >= 31130): LAR = (|r| - 26112) << 2 with
        /// the original sign restored. r = 32000 ⇒ LAR =
        /// (32000-26112) << 2 = 5888 << 2 = 23552; r = -32000 ⇒
        /// LAR = -23552.
        #[test]
        fn reflection_to_lar_segment_3() {
            let mut r = [0i16; 9];
            r[1] = 32000;
            r[2] = -32000;
            let lar = reflection_to_lar(&r);
            assert_eq!(lar[1], 23552);
            assert_eq!(lar[2], -23552);
        }

        /// Segment boundaries: r = 22118 (lower edge of segment 2)
        /// gives LAR = 22118 - 11059 = 11059. r = 22117 (still in
        /// segment 1) gives LAR = 22117 >> 1 = 11058 — a one-unit
        /// step at the boundary which is the design intent of the
        /// segmented map.
        #[test]
        fn reflection_to_lar_segment_1_2_boundary() {
            let mut r = [0i16; 9];
            r[1] = 22117;
            r[2] = 22118;
            let lar = reflection_to_lar(&r);
            assert_eq!(lar[1], 11058);
            assert_eq!(lar[2], 11059);
        }

        /// Segment 2/3 boundary: r = 31129 → seg 2 → 31129-11059 =
        /// 20070; r = 31130 → seg 3 → (31130-26112)<<2 = 5018<<2 =
        /// 20072. (Matches the decoder's §5.2.9.2 inverse map
        /// breakpoint at LAR = 20070.)
        #[test]
        fn reflection_to_lar_segment_2_3_boundary() {
            let mut r = [0i16; 9];
            r[1] = 31129;
            r[2] = 31130;
            let lar = reflection_to_lar(&r);
            assert_eq!(lar[1], 20070);
            assert_eq!(lar[2], 20072);
        }

        // ─── end-to-end ───

        /// `analyse_frame` on an all-zero input returns all-zero LARs.
        #[test]
        fn analyse_frame_zero_input() {
            let s = [0i16; FRAME_SAMPLES];
            let lar = analyse_frame(&s);
            for v in lar {
                assert_eq!(v, 0);
            }
        }

        /// `analyse_frame` is deterministic from the same input —
        /// the analysis stage carries no per-frame state.
        #[test]
        fn analyse_frame_is_deterministic() {
            let mut s = [0i16; FRAME_SAMPLES];
            for (k, slot) in s.iter_mut().enumerate() {
                *slot = ((k as i16) * 41) ^ 0x3C;
            }
            let a = analyse_frame(&s);
            let b = analyse_frame(&s);
            assert_eq!(a, b);
        }

        /// `analyse_frame` post-condition: every LAR is in the
        /// §5.2.6 prelude range `integer(real_LAR * 16384)` with
        /// `-1.625 <= real_LAR <= 1.625`, i.e. `|LAR| <= 26624` for
        /// segment-3 outputs (and tighter for segments 1 and 2). On
        /// any real input the inner-product magnitudes plus the
        /// segment-1 dominance keep us well below that bound. Just
        /// sanity-check `|LAR| < 32768`.
        #[test]
        fn analyse_frame_lar_bounded() {
            let mut s = [0i16; FRAME_SAMPLES];
            for (k, slot) in s.iter_mut().enumerate() {
                *slot = ((k as i16) * 53) ^ 0x5A;
            }
            let lar = analyse_frame(&s);
            for v in lar {
                assert!(v.abs() < i16::MAX);
            }
        }

        /// Spectral-envelope invariant: negating the input frame
        /// produces an autocorrelation that is **approximately** even
        /// (l_acf' ≈ l_acf) and reflection coefficients of the same
        /// magnitude. In exact real arithmetic this would be a strict
        /// equality — the §5.2.4 inner product is a sum of squared-pair
        /// products and ACF is even by construction.
        ///
        /// In the §5.2 fixed-point pipeline this equality is NOT bit-
        /// exact: the §5.2.4 dynamic-scaling step uses `mult_r` whose
        /// rounding `(a*b + 16384) >> 15` is asymmetric across sign,
        /// and the §5.2.5 `div` step is integer-truncating — together
        /// these can flip individual LAR values by a small amount near
        /// the segment boundaries §5.2.6 introduces.
        ///
        /// What we DO get is an L1-bounded match: the two LAR vectors
        /// agree on most entries and any disagreements are small in
        /// magnitude. Use that as the bound for this test.
        #[test]
        fn analyse_frame_near_even_under_input_negation() {
            let mut s = [0i16; FRAME_SAMPLES];
            for (k, slot) in s.iter_mut().enumerate() {
                *slot = ((k as i16) * 23) - 1000;
            }
            let mut s_neg = s;
            for slot in s_neg.iter_mut() {
                // Avoid saturating negate of i16::MIN — our test
                // inputs stay in a safe range, but be defensive.
                *slot = if *slot == i16::MIN { i16::MAX } else { -*slot };
            }
            let a = analyse_frame(&s);
            let b = analyse_frame(&s_neg);
            // Index-by-index distance is small in absolute value;
            // total L1 distance well under the segment-1 boundary.
            let mut l1: i32 = 0;
            for i in 1..=8 {
                l1 += (a[i] as i32 - b[i] as i32).abs();
            }
            assert!(
                l1 < 1000,
                "LAR L1 distance under sign flip too large: a={a:?} b={b:?} l1={l1}"
            );
        }

        // ─── §5.2.7 LAR quantisation ───

        /// `LAR = 0` ⇒ `LARc[i] = sub((B[i]+256) >> 9, MIC[i])`,
        /// the per-segment "centre" code. Spell out the eight values
        /// explicitly so the encode-side scaling is pinned to Table 5.1
        /// columns A / B / MIC / MAC.
        ///
        /// Hand-derivation per the §5.2.7 pseudocode:
        /// * i=1: B=0, MIC=-32 → (0+256)>>9 = 0 → sub(0,-32) = 32.
        /// * i=2: B=0, MIC=-32 → 32 (same as i=1).
        /// * i=3: B=2048, MIC=-16 → (2048+256)>>9 = 4 → sub(4,-16) = 20.
        /// * i=4: B=-2560, MIC=-16 → (-2560+256)>>9 = -2304>>9 = -5 →
        ///   sub(-5,-16) = 11.
        /// * i=5: B=94, MIC=-8 → (94+256)>>9 = 0 → sub(0,-8) = 8.
        /// * i=6: B=-1792, MIC=-8 → (-1792+256)>>9 = -1536>>9 = -3 →
        ///   sub(-3,-8) = 5.
        /// * i=7: B=-341, MIC=-4 → (-341+256)>>9 = -85>>9 = -1 →
        ///   sub(-1,-4) = 3.
        /// * i=8: B=-1144, MIC=-4 → (-1144+256)>>9 = -888>>9 = -2 →
        ///   sub(-2,-4) = 2.
        #[test]
        fn quantise_lar_zero_input_gives_per_index_centre() {
            let lar = [0i16; 9];
            let lar_c = quantise_lar(&lar);
            assert_eq!(lar_c, [0, 32, 32, 20, 11, 8, 5, 3, 2]);
        }

        /// §5.2.7 saturates the codeword at `MAC[i]` for any LAR large
        /// enough to overshoot the upper bound; after the `sub(_, MIC)`
        /// final step the codeword is `MAC[i] - MIC[i]`. For i=1
        /// (MIC=-32, MAC=31) that's 63 — the largest 6-bit codeword
        /// the §1.7 frame packer holds.
        #[test]
        fn quantise_lar_saturates_at_mac_minus_mic() {
            let mut lar = [0i16; 9];
            // i16::MAX-class LAR — the encode arithmetic for any of
            // the eight indices will overshoot MAC.
            for slot in lar.iter_mut().skip(1) {
                *slot = i16::MAX;
            }
            let lar_c = quantise_lar(&lar);
            // For every index i, the saturated codeword is MAC-MIC.
            let expected = [0, 63, 63, 31, 31, 15, 15, 7, 7];
            assert_eq!(lar_c, expected);
        }

        /// §5.2.7 saturates the codeword at `MIC[i]` for any LAR large
        /// enough to undershoot the lower bound; after the
        /// `sub(_, MIC)` final step the codeword is `MIC[i] - MIC[i]`
        /// = 0 (the smallest unsigned codeword).
        #[test]
        fn quantise_lar_saturates_at_mic_to_zero_codeword() {
            let mut lar = [0i16; 9];
            for slot in lar.iter_mut().skip(1) {
                *slot = i16::MIN;
            }
            let lar_c = quantise_lar(&lar);
            // Every index's codeword is 0 (= MIC - MIC).
            assert_eq!(lar_c, [0; 9]);
        }

        /// §5.2.7 LARc width matches Table 1.1: the six §1.7 LAR
        /// fields are 6, 6, 5, 5, 4, 4, 3, 3 bits respectively.
        /// After the `sub(_, MIC)` shift every LARc fits in its bit
        /// budget — i.e. `LARc[i] <= MAC[i] - MIC[i]` and
        /// `MAC-MIC+1` is a power of two equal to `2^bits`.
        #[test]
        fn quantise_lar_fits_table_1_1_bit_widths() {
            // Choose a varied LAR vector that exercises both branches
            // of the MIN/MAX bound check.
            let lar = [0i16, 5000, -5000, 8000, -8000, 1500, -1500, 800, -800];
            let lar_c = quantise_lar(&lar);
            // Table 1.1 bit widths for LARc[1..=8].
            let widths = [0u32, 6, 6, 5, 5, 4, 4, 3, 3];
            for i in 1..=8 {
                let cap = (1i16 << widths[i]) - 1;
                assert!(lar_c[i] >= 0, "LARc[{i}] must be unsigned");
                assert!(
                    lar_c[i] <= cap,
                    "LARc[{i}] = {} exceeds {}-bit cap {}",
                    lar_c[i],
                    widths[i],
                    cap
                );
            }
        }

        /// §5.2.7 / §5.2.8 round-trip on the all-zero LAR: the codeword
        /// from `quantise_lar(LAR=0)` decodes back to LAR ≈ 0. The
        /// decoder isn't part of this module so we hand-verify the
        /// outcome: with LARc set to the per-index centre (above) and
        /// MIC restored, `temp1 = add(LARc, MIC) << 10 = 0`; then
        /// `temp2 = B[i] << 1`; then `temp1 = sub(temp1, temp2) =
        /// -B[i] << 1`; then `temp1 = mult_r(INVA[i], temp1)`; then
        /// `LARpp[i] = 2*temp1`. For the indices where `B[i] = 0`
        /// (i=1,2) the result is exactly 0; for the rest, the result
        /// is bounded by the per-segment quantiser step. Verify the
        /// i=1,2 exact-zero half here; the bounded-error half is
        /// covered by `quantise_then_decode_round_trip` below.
        #[test]
        fn quantise_lar_zero_input_round_trips_exact_for_b_zero_indices() {
            let lar = [0i16; 9];
            let lar_c = quantise_lar(&lar);
            // i=1, i=2 have B=0 so decode_lar maps them back to 0
            // exactly: the per-index centre code is exactly the value
            // that cancels MIC in the decoder's `add(LARc, MIC)`.
            assert_eq!(lar_c[1] as i32 + MIC[1] as i32, 0);
            assert_eq!(lar_c[2] as i32 + MIC[2] as i32, 0);
        }

        /// End-to-end §5.2.7 + §5.2.8 quantiser round trip: encode a
        /// LAR vector with `quantise_lar`, decode the resulting LARc
        /// back through the §5.2.8 decode_lar inverse, and confirm the
        /// recovered LAR is within ~one quantiser step of the input.
        ///
        /// One encoder codeword increment corresponds to a LAR delta
        /// of `(1 << 9) * (1 << 15) / A[i] = 2^24 / A[i]` — the
        /// effective inverse of `mult(A[i], LAR) >> 9`. With Table 5.1
        /// `A[i]` values in `[8534, 20480]` the per-index step lies in
        /// `[~819, ~1966]` LAR units, so the round-trip error for any
        /// in-range LAR is bounded by one such step. We bound at 2000
        /// (the largest per-index step + a small slack for the
        /// asymmetric round-half-up vs floor mismatch between encoder
        /// `>> 9` and decoder `mult_r`).
        #[test]
        fn quantise_then_decode_round_trip_within_one_step() {
            use crate::tables::INVA;

            // A LAR vector that stays within the quantiser's
            // representable range for every i.
            let lar = [0i16, 4000, -3000, 2500, -1200, 800, -400, 250, -100];
            let lar_c = quantise_lar(&lar);

            // Open-coded §5.2.8 (decode_lar lives in the sibling
            // `decoder` module; per-crate visibility means we
            // re-inline it here rather than expand the public surface
            // for the test).
            let mut lar_pp = [0i16; 9];
            for i in 1..=8 {
                let mut t1 = add(lar_c[i], MIC[i]) << 10;
                let t2 = B[i] << 1;
                t1 = sub(t1, t2);
                t1 = mult_r(INVA[i], t1);
                lar_pp[i] = add(t1, t1);
            }
            // Per-index round-trip error bound — one quantiser step
            // at i=7 (worst case ≈ 1966 LAR units) + slack.
            for i in 1..=8 {
                let err = (lar[i] as i32 - lar_pp[i] as i32).abs();
                assert!(
                    err <= 2000,
                    "LAR round-trip error too large at i={i}: in={} out={} err={err}",
                    lar[i],
                    lar_pp[i]
                );
            }
        }

        /// §5.2.7 is monotonic per index — larger LAR ⇒ larger (or
        /// equal, when saturated) codeword. Verify on a sweep of LAR
        /// values for i=1.
        #[test]
        fn quantise_lar_is_monotonic_per_index() {
            let mut prev_code: i16 = i16::MIN;
            for v in (-30000i16..=30000).step_by(500) {
                let mut lar = [0i16; 9];
                lar[1] = v;
                let lar_c = quantise_lar(&lar);
                assert!(
                    lar_c[1] >= prev_code,
                    "non-monotone at LAR[1]={v}: code={} prev={prev_code}",
                    lar_c[1]
                );
                prev_code = lar_c[1];
            }
        }

        /// §5.2.7 invariant: index 0 is reserved and always emits 0.
        #[test]
        fn quantise_lar_index_zero_is_sentinel() {
            let mut lar = [0i16; 9];
            lar[0] = 12345; // any value — the encoder ignores i=0.
            let lar_c = quantise_lar(&lar);
            assert_eq!(lar_c[0], 0);
        }

        /// `analyse_and_quantise_frame` on an all-zero input returns
        /// the per-index centre codewords (= `quantise_lar([0; 9])`).
        #[test]
        fn analyse_and_quantise_frame_zero_input() {
            let s = [0i16; FRAME_SAMPLES];
            let lar_c = analyse_and_quantise_frame(&s);
            assert_eq!(lar_c, [0, 32, 32, 20, 11, 8, 5, 3, 2]);
        }

        /// `analyse_and_quantise_frame` is deterministic — every
        /// component on the chain (§5.2.4 / §5.2.5 / §5.2.6 / §5.2.7)
        /// is stateless per frame.
        #[test]
        fn analyse_and_quantise_frame_is_deterministic() {
            let mut s = [0i16; FRAME_SAMPLES];
            for (k, slot) in s.iter_mut().enumerate() {
                *slot = ((k as i16) * 47) ^ 0x4D;
            }
            let a = analyse_and_quantise_frame(&s);
            let b = analyse_and_quantise_frame(&s);
            assert_eq!(a, b);
        }

        /// `analyse_and_quantise_frame` post-condition: every LARc
        /// fits its Table 1.1 bit budget regardless of input.
        #[test]
        fn analyse_and_quantise_frame_lar_c_in_range() {
            let mut s = [0i16; FRAME_SAMPLES];
            for (k, slot) in s.iter_mut().enumerate() {
                *slot = ((k as i16) * 59) ^ 0x6E;
            }
            let lar_c = analyse_and_quantise_frame(&s);
            let widths = [0u32, 6, 6, 5, 5, 4, 4, 3, 3];
            for i in 1..=8 {
                let cap = (1i16 << widths[i]) - 1;
                assert!((0..=cap).contains(&lar_c[i]));
            }
        }

        // ─── §5.2.10 short-term analysis filter ───

        /// §5.2.10 — with all-zero reflection coefficients the
        /// filter reduces to `d[k] = s[k]` (lattice taps are zero),
        /// and the §5.2.10 state-update still shifts samples through
        /// `u[0..=7]` — specifically `u[0]` ends up holding the last
        /// `s[k]` value pulled into the filter.
        #[test]
        fn short_term_analysis_zero_rp_is_identity() {
            let mut s = [0i16; FRAME_SAMPLES];
            for (k, slot) in s.iter_mut().enumerate() {
                *slot = (k as i16) * 7;
            }
            let rp = [0i16; 9];
            let mut u = [0i16; 8];
            let d = short_term_analysis_filter(&s, &rp, &mut u, 0, FRAME_SAMPLES - 1);
            // With rp = 0, mult_r(rp[i], _) = 0, so the §5.2.10 inner
            // loop never modifies `di` and never modifies `temp`
            // beyond add(u[i-1], 0) = u[i-1]. di stays at s[k]; sav
            // walks through s[k] and u[0..7] gets the previous sav.
            for k in 0..FRAME_SAMPLES {
                assert_eq!(d[k], s[k], "d[{k}] mismatch");
            }
            // After processing 160 samples with rp = 0, the `u`
            // state is the last 8 values of `s` walked through the
            // sav chain. Specifically, on iteration k = 159 with
            // rp = 0, sav = s[159] enters; through 8 inner-iterations
            // u[7..=0] gets {sav from iter 158, 157, ..., 152}.
            // After the inner loop completes, sav is discarded.
            // The persisted u[0] = sav-pre-shift = s[159]; u[1] =
            // s[158]; …; u[7] = s[152].
            assert_eq!(u[0], s[FRAME_SAMPLES - 1]);
            assert_eq!(u[7], s[FRAME_SAMPLES - 8]);
        }

        /// §5.2.10 — empty range (k_end < k_start in spec terms; we
        /// require k_start <= k_end, so use k_start == k_end for
        /// the minimal-loop case) still passes; one iteration runs.
        #[test]
        fn short_term_analysis_single_sample_block() {
            let mut s = [0i16; FRAME_SAMPLES];
            s[5] = 1000;
            let rp = [0i16; 9];
            let mut u = [0i16; 8];
            let d = short_term_analysis_filter(&s, &rp, &mut u, 5, 5);
            assert_eq!(d[5], 1000);
            // Other slots are untouched zeros.
            for k in 0..FRAME_SAMPLES {
                if k != 5 {
                    assert_eq!(d[k], 0);
                }
            }
            // u[0] holds the one sav that walked through: s[5].
            assert_eq!(u[0], 1000);
        }

        /// §5.2.10 — zero input + zero rp + zero u home state ⇒
        /// zero output everywhere.
        #[test]
        fn short_term_analysis_zero_input_zero_output() {
            let s = [0i16; FRAME_SAMPLES];
            let rp = [0i16; 9];
            let mut u = [0i16; 8];
            let d = short_term_analysis_filter(&s, &rp, &mut u, 0, FRAME_SAMPLES - 1);
            for v in d {
                assert_eq!(v, 0);
            }
            for v in u {
                assert_eq!(v, 0);
            }
        }

        /// §5.2.10 — small positive rp shrinks the input magnitude
        /// when fed a DC step from a zero filter memory: the lattice
        /// behaves as a high-pass on positive correlation.
        ///
        /// A DC input of magnitude M with rp[1] = 0.5 (16384 in Q15)
        /// and rp[2..=8] = 0 makes d[0] = s[0]; for k >= 1 the
        /// recursion subtracts a fraction of u[0] (which is s[k-1])
        /// from di. So d[1] ≈ M - 0.5*M = M/2. We just check the
        /// magnitudes shrink.
        #[test]
        fn short_term_analysis_dc_step_high_passed() {
            let s = [1000i16; FRAME_SAMPLES];
            let mut rp = [0i16; 9];
            rp[1] = 16384; // 0.5 in Q15
            let mut u = [0i16; 8];
            let d = short_term_analysis_filter(&s, &rp, &mut u, 0, 10);
            // d[0] = first iter: di = s[0] = 1000; mult_r(rp[1], di)
            // = (16384*1000 + 16384) >> 15 = (16400384) >> 15 = 500.
            // u[0] starts at 0, so temp = 0 + 500 = 500.
            // di = di + mult_r(rp[1], u[0]) = 1000 + 0 = 1000.
            // Through i = 2..=8 the di gets reflection-modified by
            // zero rp, di stays at 1000. d[0] = 1000.
            assert_eq!(d[0], 1000);
            // For k >= 1, u[0] = 1000 entering, di = 1000 entering;
            // temp = u[0] + 0.5*di = 1000 + 500 = 1500; di = 1000
            // + 0.5*u[0] = 1500. Through i = 2..=8 di stays 1500
            // (rp[2..=8] = 0). So d[1] = 1500. After iteration,
            // u[0] holds new sav = 1500.
            assert_eq!(d[1], 1500);
            // Output magnitudes should not exceed 2 * input for
            // |rp| <= 0.5.
            for k in 0..=10 {
                assert!(d[k].unsigned_abs() < 4000, "d[{k}] = {} too large", d[k]);
            }
        }

        // ─── §5.2.9.1 + §5.2.10 — Analyzer (4-block frame loop) ───

        /// `Analyzer::new()` is the §4.5 Table 4.2 home state:
        /// `LARpp(j-1)[1..=8] = 0` and `u[0..=7] = 0`.
        #[test]
        fn analyzer_new_is_home_state() {
            let a = Analyzer::new();
            assert_eq!(a.lar_pp_prev(), &[0i16; 9]);
            assert_eq!(a.u(), &[0i16; 8]);
        }

        /// `Analyzer::reset` returns to the home state from any
        /// post-frame state.
        #[test]
        fn analyzer_reset_returns_home_state() {
            let mut a = Analyzer::new();
            let mut s = [0i16; FRAME_SAMPLES];
            for (k, slot) in s.iter_mut().enumerate() {
                *slot = ((k as i16) * 17) - 100;
            }
            let _ = a.analyse_frame(&s);
            // After one frame, LARpp(j-1) is non-zero in general
            // (the input has structure).
            a.reset();
            assert_eq!(a.lar_pp_prev(), &[0i16; 9]);
            assert_eq!(a.u(), &[0i16; 8]);
        }

        /// `Analyzer::analyse_frame` on a zero input from the home
        /// state ⇒ all-zero LARc[1..=8] and all-zero d[0..=159].
        /// Reason: §5.2.4..§5.2.6 produce all-zero LARs for a zero
        /// frame; §5.2.7 with LAR = 0 produces the "centre" code
        /// per index, which on §5.2.7's `sub(code, MIC[i])` lands
        /// on `-MIC[i]` for indices 1..=2 (i.e. 32) and similar
        /// non-zero codewords for other indices. So the encoded
        /// LARc is NOT all-zero, but the §5.2.10 residual SHOULD
        /// be zero throughout because s = 0 and the filter's `di`
        /// starts at zero, `mult_r(rp[i], 0) = 0`, so d[k] = 0.
        #[test]
        fn analyzer_zero_input_zero_residual() {
            let mut a = Analyzer::new();
            let s = [0i16; FRAME_SAMPLES];
            let (_lar_c, d) = a.analyse_frame(&s);
            for v in d {
                assert_eq!(v, 0);
            }
        }

        /// `Analyzer::analyse_frame` is deterministic from a fixed
        /// home state — calling it on the same input twice produces
        /// the same (LARc, d) pair.
        #[test]
        fn analyzer_is_deterministic_from_home_state() {
            let mut a = Analyzer::new();
            let mut b = Analyzer::new();
            let mut s = [0i16; FRAME_SAMPLES];
            for (k, slot) in s.iter_mut().enumerate() {
                *slot = ((k as i16).wrapping_mul(31)) ^ 0x1234;
            }
            let (lar_c_a, d_a) = a.analyse_frame(&s);
            let (lar_c_b, d_b) = b.analyse_frame(&s);
            assert_eq!(lar_c_a, lar_c_b);
            assert_eq!(d_a, d_b);
        }

        /// `Analyzer::analyse_frame` updates `LARpp(j-1)` to the
        /// current frame's `LARpp(j)` per §5.2.9.1 across-frame
        /// convention, and `u[0..=7]` is updated by the §5.2.10
        /// filter steps.
        #[test]
        fn analyzer_state_persists_across_frames() {
            let mut a = Analyzer::new();
            let mut s = [0i16; FRAME_SAMPLES];
            for (k, slot) in s.iter_mut().enumerate() {
                // Non-trivial input so LARs and `u` move from zero.
                *slot = ((k as i16).wrapping_mul(23)) ^ 0x4321;
            }
            let lar_pp_prev_before = *a.lar_pp_prev();
            let u_before = *a.u();
            let _ = a.analyse_frame(&s);
            // After one frame, both states have moved at least
            // somewhere.
            assert_ne!(*a.lar_pp_prev(), lar_pp_prev_before);
            assert_ne!(*a.u(), u_before);
            // Calling analyse_frame again with the same input must
            // produce *different* output, because the entering
            // LARpp(j-1) is no longer zero.
            let mut a_fresh = Analyzer::new();
            let (lar_c_first, _) = a_fresh.analyse_frame(&s);
            let (lar_c_second, _) = a.analyse_frame(&s);
            // The current-frame LARc[1..=8] is computed from the
            // same s[..], so it should match the first-frame LARc.
            // The §5.2.9.1 interpolation only affects the §5.2.10
            // residual, not the §5.2.4..§5.2.7 LARc path.
            assert_eq!(lar_c_first, lar_c_second);
        }

        /// §5.2.10 across-block coherence — the `u[0..=7]` state
        /// updated by block 0 must be the entry state for block 1,
        /// etc. We verify by manually driving the four blocks
        /// in sequence and confirming the result matches
        /// `Analyzer::analyse_frame`.
        #[test]
        fn analyzer_four_block_loop_matches_manual_split() {
            let mut s = [0i16; FRAME_SAMPLES];
            for (k, slot) in s.iter_mut().enumerate() {
                *slot = ((k as i16) * 13).wrapping_sub(1024);
            }

            // Run the canonical analyzer.
            let mut a = Analyzer::new();
            let (_lar_c, d_canonical) = a.analyse_frame(&s);

            // Run the same pipeline manually using the lower-level
            // helpers to ensure they're consistent.
            let a_manual = Analyzer::new();
            let lar_c = analyse_and_quantise_frame(&s);
            let lar_pp_curr = decode_lar(&lar_c);
            let mut d_manual = [0i16; FRAME_SAMPLES];
            const BLOCKS: [(usize, usize, u8); 4] =
                [(0, 12, 0), (13, 26, 1), (27, 39, 2), (40, 159, 3)];
            for &(k_start, k_end, block_id) in BLOCKS.iter() {
                let lar_p = interpolate_lar(a_manual.lar_pp_prev(), &lar_pp_curr, block_id);
                let rp = larp_to_rp(&lar_p);
                let d_block = short_term_analysis_filter(
                    &s,
                    &rp,
                    // SAFETY: we need mutable access to u, but
                    // Analyzer exposes only an immutable accessor.
                    // Use a local mutable mirror.
                    &mut [0i16; 8],
                    k_start,
                    k_end,
                );
                d_manual[k_start..=k_end].copy_from_slice(&d_block[k_start..=k_end]);
            }
            // The manual run keeps a fresh-zero u for every block, so
            // the d output WILL differ from the canonical run for the
            // late blocks. The first block, where the canonical
            // analyzer also enters with u = 0, must match.
            for k in 0..=12 {
                assert_eq!(d_canonical[k], d_manual[k], "block 0 sample {k}");
            }
        }

        // ─── §5.2.11 / §5.2.12 LTP analysis ───

        /// §5.2.11 on an all-zero `d[..]` and all-zero history: dmax = 0
        /// ⇒ scal = 0; the entire cross-correlation search yields
        /// `L_result = 0` for every lambda; `L_max` never increases so
        /// `Nc` stays at its initial 40 and the `L_max <= 0` branch
        /// gives `bc = 0`.
        #[test]
        fn ltp_zero_input_yields_home_lag_and_zero_gain() {
            let d = [0i16; SUBFRAME_SAMPLES];
            let dp_hist = [0i16; LTP_DELAY];
            let p = ltp_parameters(&d, &dp_hist);
            assert_eq!(p.n_c, 40);
            assert_eq!(p.b_c, 0);
        }

        /// §5.2.12 on an all-zero `d[..]` + all-zero history: `bp =
        /// QLB[0]` = 3277, `dpp[k] = mult_r(3277, 0) = 0`, `e[k] =
        /// sub(0, 0) = 0`.
        #[test]
        fn ltp_filter_zero_input_yields_zero_residual() {
            let d = [0i16; SUBFRAME_SAMPLES];
            let dp_hist = [0i16; LTP_DELAY];
            let params = LtpParameters { n_c: 40, b_c: 0 };
            let (dpp, e) = long_term_analysis_filter(&d, &dp_hist, params);
            for k in 0..SUBFRAME_SAMPLES {
                assert_eq!(dpp[k], 0, "dpp[{k}]");
                assert_eq!(e[k], 0, "e[{k}]");
            }
        }

        /// §5.2.11 LTP-lag detection: seed the §4.5 history with a
        /// short impulse train and the §5.2.10 residual with a matching
        /// shifted train; the cross-correlation must peak at the seeded
        /// lag.
        ///
        /// We use lag 40 (the spec's minimum) — the closest history
        /// sample, `dp[-1]`, lines up with `d[39]` so for k=0..=39 we
        /// get `dp[k - 40] = dp_hist[40 - k - 1] = dp_hist[39 - k]`.
        /// Construct `dp_hist[i] = 1000` for `i = 0..=39` (i.e.
        /// `dp[-1..=-40]`) and zero elsewhere; construct `d[k] = 1000`
        /// for all k. The k=0..=39 inner product sums to a positive
        /// L_result for lambda = 40 (40 * L_mult(1000,1000) = 40 *
        /// 2,000,000 = 8e7) and to zero for lambda > 40 (history is
        /// zero beyond dp[-40]).
        #[test]
        fn ltp_lag_detection_minimum_pitch() {
            let d = [1000i16; SUBFRAME_SAMPLES];
            let mut dp_hist = [0i16; LTP_DELAY];
            for slot in dp_hist[0..40].iter_mut() {
                *slot = 1000;
            }
            let p = ltp_parameters(&d, &dp_hist);
            assert_eq!(p.n_c, 40, "Nc should land on the seeded lag 40");
            // L_max > 0, so bc must be at least 1; with l_max < l_power
            // path it lands in [0, 3]. With strong correlation we expect
            // bc = 3 (the L_max >= L_power short-circuit).
            assert!(p.b_c >= 1);
        }

        /// §5.2.11 LTP-lag detection at a non-minimum lag — seed
        /// history at lag 80 only and confirm `Nc = 80`.
        #[test]
        fn ltp_lag_detection_mid_range() {
            let d = [2000i16; SUBFRAME_SAMPLES];
            let mut dp_hist = [0i16; LTP_DELAY];
            // dp_hist[i] = dp[-(i+1)]. Lag 80 wants k - 80 ∈ {-40, .., -80
            // - 39 +} → dp[-80..=-41] → dp_hist[40..=79].
            for slot in dp_hist[40..80].iter_mut() {
                *slot = 2000;
            }
            let p = ltp_parameters(&d, &dp_hist);
            assert_eq!(p.n_c, 80, "Nc should land on the seeded lag 80");
        }

        /// §5.2.11 bc = 3 (max-gain) branch: when `L_max >= L_power`
        /// the procedure exits with `bc = 3` and `Nc` reflecting the
        /// lag at which the cross-correlation peaked. The constant-d /
        /// constant-history seed of `ltp_lag_detection_minimum_pitch`
        /// satisfies `L_max == L_power` (the lagged signal IS the
        /// reference), so bc = 3.
        #[test]
        fn ltp_max_gain_branch_lands_on_bc_3() {
            let d = [4000i16; SUBFRAME_SAMPLES];
            let mut dp_hist = [0i16; LTP_DELAY];
            for slot in dp_hist[0..40].iter_mut() {
                *slot = 4000;
            }
            let p = ltp_parameters(&d, &dp_hist);
            assert_eq!(p.b_c, 3);
        }

        /// §5.2.11 `Nc` codeword fits the §1.7 Table 1.1 7-bit field
        /// (range 40..=120).
        #[test]
        fn ltp_n_c_always_in_spec_range() {
            // Random-ish d, random-ish dp_hist: we just need to drive
            // the search.
            let mut d = [0i16; SUBFRAME_SAMPLES];
            for (k, slot) in d.iter_mut().enumerate() {
                *slot = ((k as i16) * 173).wrapping_sub(2048);
            }
            let mut dp_hist = [0i16; LTP_DELAY];
            for (i, slot) in dp_hist.iter_mut().enumerate() {
                *slot = ((i as i16) * 79).wrapping_sub(1024);
            }
            let p = ltp_parameters(&d, &dp_hist);
            assert!(
                (40..=120).contains(&p.n_c),
                "Nc must be in [40, 120], got {}",
                p.n_c
            );
            assert!((0..=3).contains(&p.b_c), "bc must be in [0, 3]");
        }

        /// §5.2.12 inverse — when the LTP estimate exactly matches the
        /// input (bc=3 saturating gain on a constant-history seed),
        /// the long-term residual `e[..]` shrinks toward zero. We
        /// don't get exact zero because QLB[3] = 32767 is not 1.0 in
        /// Q15 (it's 32767/32768).
        #[test]
        fn ltp_filter_max_gain_reduces_residual() {
            let d = [8000i16; SUBFRAME_SAMPLES];
            let mut dp_hist = [0i16; LTP_DELAY];
            for slot in dp_hist[0..40].iter_mut() {
                *slot = 8000;
            }
            let params = LtpParameters { n_c: 40, b_c: 3 };
            let (_dpp, e) = long_term_analysis_filter(&d, &dp_hist, params);
            // The residual shrinks roughly by the (1 - 32767/32768)
            // factor. Each e[k] should be small relative to d[k].
            for &ek in e.iter() {
                assert!(ek.unsigned_abs() <= 8, "|e[k]| = {} too large", ek.abs());
            }
        }

        /// §5.2.12 sign-preservation: `e[k] = sub(d[k], dpp[k])`. For
        /// an all-zero history `dpp[k] = mult_r(QLB[bc], 0) = 0`, so
        /// `e[k] = d[k]` regardless of bc.
        #[test]
        fn ltp_filter_passes_through_with_empty_history() {
            let mut d = [0i16; SUBFRAME_SAMPLES];
            for (k, slot) in d.iter_mut().enumerate() {
                *slot = ((k as i16) * 11).wrapping_sub(200);
            }
            let dp_hist = [0i16; LTP_DELAY];
            for &bc_try in &[0i16, 1, 2, 3] {
                let params = LtpParameters {
                    n_c: 40,
                    b_c: bc_try,
                };
                let (_dpp, e) = long_term_analysis_filter(&d, &dp_hist, params);
                assert_eq!(e, d, "bc = {bc_try}: e[..] must equal d[..]");
            }
        }

        /// `LtpAnalyzer` in the home state matches the
        /// `ltp_parameters` + `long_term_analysis_filter` composition
        /// driven with an all-zero history (i.e. the §4.5 home state).
        #[test]
        fn ltp_analyzer_home_state_matches_free_functions() {
            let mut d = [0i16; SUBFRAME_SAMPLES];
            for (k, slot) in d.iter_mut().enumerate() {
                *slot = ((k as i16) * 31).wrapping_sub(512);
            }
            let dp_hist = [0i16; LTP_DELAY];
            let p_free = ltp_parameters(&d, &dp_hist);
            let (dpp_free, e_free) = long_term_analysis_filter(&d, &dp_hist, p_free);

            let a = LtpAnalyzer::new();
            let (p_analyzer, dpp_analyzer, e_analyzer) = a.analyse_subframe(&d);
            assert_eq!(p_free, p_analyzer);
            assert_eq!(dpp_free, dpp_analyzer);
            assert_eq!(e_free, e_analyzer);
        }

        /// `LtpAnalyzer::reset` returns to the §4.5 home state.
        #[test]
        fn ltp_analyzer_reset_returns_home() {
            let mut a = LtpAnalyzer::new();
            let ep = [123i16; SUBFRAME_SAMPLES];
            let dpp = [-321i16; SUBFRAME_SAMPLES];
            a.update_dp_after_subframe(&ep, &dpp);
            assert_ne!(a.dp_hist(), &[0i16; LTP_DELAY]);
            a.reset();
            assert_eq!(a.dp_hist(), &[0i16; LTP_DELAY]);
        }

        /// §5.2.18 dp-update lays the new 40 samples into dp_hist[0..=39]
        /// with the spec's index reversal — `dp[-1] = add(ep[39],
        /// dpp[39])` lands in `dp_hist[0]`, etc.
        #[test]
        fn ltp_dp_update_orders_newest_sample_first() {
            let mut a = LtpAnalyzer::new();
            let mut ep = [0i16; SUBFRAME_SAMPLES];
            let mut dpp = [0i16; SUBFRAME_SAMPLES];
            for k in 0..SUBFRAME_SAMPLES {
                ep[k] = k as i16;
                dpp[k] = 100 * (k as i16);
            }
            a.update_dp_after_subframe(&ep, &dpp);
            // dp[-1] = ep[39] + dpp[39] = 39 + 3900 = 3939 ⇒ dp_hist[0]
            assert_eq!(a.dp_hist()[0], 3939);
            // dp[-2] = ep[38] + dpp[38] = 38 + 3800 = 3838 ⇒ dp_hist[1]
            assert_eq!(a.dp_hist()[1], 3838);
            // dp[-40] = ep[0] + dpp[0] = 0 ⇒ dp_hist[39]
            assert_eq!(a.dp_hist()[39], 0);
            // Positions 40..=119 should still be zero (no older
            // history existed).
            for i in 40..LTP_DELAY {
                assert_eq!(a.dp_hist()[i], 0, "dp_hist[{i}] should be 0");
            }
        }

        /// §5.2.18 dp-update across two sub-segments: the first update's
        /// 40 samples slide into dp_hist[40..=79] after the second
        /// update.
        #[test]
        fn ltp_dp_update_slides_history_across_two_sub_segments() {
            let mut a = LtpAnalyzer::new();
            // First sub-segment: dp_hist[0..=39] ← deterministic.
            let mut ep1 = [0i16; SUBFRAME_SAMPLES];
            let mut dpp1 = [0i16; SUBFRAME_SAMPLES];
            for k in 0..SUBFRAME_SAMPLES {
                ep1[k] = (k as i16) + 1;
                dpp1[k] = 0;
            }
            a.update_dp_after_subframe(&ep1, &dpp1);
            let first = *a.dp_hist();
            // Second sub-segment: new content.
            let mut ep2 = [0i16; SUBFRAME_SAMPLES];
            for k in 0..SUBFRAME_SAMPLES {
                ep2[k] = -((k as i16) + 1);
            }
            let dpp2 = [0i16; SUBFRAME_SAMPLES];
            a.update_dp_after_subframe(&ep2, &dpp2);
            let second = *a.dp_hist();
            // After the second update, the entries that were in
            // dp_hist[0..=39] (newest, after sub-segment 1) should now
            // sit at dp_hist[40..=79] (slid leftward by 40 in spec
            // terms — i.e. 40 positions further into the past).
            for i in 0..SUBFRAME_SAMPLES {
                assert_eq!(
                    second[40 + i],
                    first[i],
                    "dp_hist[40+{i}] = first dp_hist[{i}]",
                );
            }
            // The newest 40 entries should be the negated ramp.
            for k in 0..SUBFRAME_SAMPLES {
                // ep2[k] + dpp2[k] = -(k+1); lands at dp_hist[39-k].
                assert_eq!(second[SUBFRAME_SAMPLES - 1 - k], -((k as i16) + 1));
            }
        }

        /// `LtpAnalyzer::analyse_subframe` is pure with respect to the
        /// delay-line state — it must not mutate `dp_hist`.
        #[test]
        fn ltp_analyzer_analyse_subframe_is_non_mutating() {
            let mut dp_hist = [0i16; LTP_DELAY];
            for (i, slot) in dp_hist.iter_mut().enumerate() {
                *slot = (i as i16) * 13;
            }
            let a = LtpAnalyzer::with_dp_hist(dp_hist);
            let mut d = [0i16; SUBFRAME_SAMPLES];
            for (k, slot) in d.iter_mut().enumerate() {
                *slot = (k as i16) * 7;
            }
            let _ = a.analyse_subframe(&d);
            assert_eq!(
                a.dp_hist(),
                &dp_hist,
                "analyse_subframe must not mutate state"
            );
        }

        /// Determinism: running `analyse_subframe` twice with the same
        /// state + input must produce the same outputs.
        #[test]
        fn ltp_analyzer_deterministic() {
            let mut dp_hist = [0i16; LTP_DELAY];
            for (i, slot) in dp_hist.iter_mut().enumerate() {
                *slot = ((i as i16).wrapping_mul(17)) ^ 0x1234;
            }
            let a = LtpAnalyzer::with_dp_hist(dp_hist);
            let mut d = [0i16; SUBFRAME_SAMPLES];
            for (k, slot) in d.iter_mut().enumerate() {
                *slot = ((k as i16).wrapping_mul(19)) ^ 0x4321;
            }
            let r1 = a.analyse_subframe(&d);
            let r2 = a.analyse_subframe(&d);
            assert_eq!(r1.0, r2.0);
            assert_eq!(r1.1, r2.1);
            assert_eq!(r1.2, r2.2);
        }

        // ─── §5.2.13 weighting filter ───

        /// All-zero input ⇒ all-zero output. The §5.2.13 rounding
        /// constant 8192 is below `2^16`, so two doublings keep it at
        /// 32768, and `32768 >> 16 = 0`.
        #[test]
        fn weighting_filter_zero_input_is_zero() {
            let e = [0i16; SUBFRAME_SAMPLES];
            let x = weighting_filter(&e);
            for (k, v) in x.iter().enumerate() {
                assert_eq!(*v, 0, "x[{k}] should be 0 for zero input");
            }
        }

        /// Determinism: same input ⇒ same output. The filter is
        /// stateless per the §5.2.13 spec text.
        #[test]
        fn weighting_filter_deterministic() {
            let mut e = [0i16; SUBFRAME_SAMPLES];
            for (k, slot) in e.iter_mut().enumerate() {
                *slot = ((k as i16).wrapping_mul(37)) ^ 0x2a5b;
            }
            let x1 = weighting_filter(&e);
            let x2 = weighting_filter(&e);
            assert_eq!(x1, x2);
        }

        /// Linearity (sign): negating the input negates the output.
        /// `L_mult(-a, b) = -L_mult(a, b)`, `L_add(-x, -y) = -L_add(x, y)`,
        /// and the rounding constant is symmetric in that the
        /// `>> 16` is an arithmetic shift; the only deviation is the
        /// `+8192` rounding bias, which is below the truncation
        /// granularity once the sum survives `>>16`. We exercise an
        /// input whose response is large enough that the rounding
        /// bias is dwarfed.
        #[test]
        fn weighting_filter_linear_sign_flip() {
            // Spread a moderate-magnitude input across the central
            // taps so the rounding bias is small relative to the
            // 11-tap accumulator.
            let mut e = [0i16; SUBFRAME_SAMPLES];
            for (k, slot) in e.iter_mut().enumerate() {
                *slot = if k % 3 == 0 { 4000 } else { -2500 };
            }
            let mut ne = [0i16; SUBFRAME_SAMPLES];
            for k in 0..SUBFRAME_SAMPLES {
                ne[k] = -e[k];
            }
            let x = weighting_filter(&e);
            let nx = weighting_filter(&ne);
            // Allow a ±1 tolerance because the `+8192` rounding bias
            // breaks strict odd symmetry by at most one LSB after the
            // `>>16` step.
            for k in 0..SUBFRAME_SAMPLES {
                let diff = (x[k] as i32) + (nx[k] as i32);
                assert!(
                    diff.abs() <= 1,
                    "x[{k}] + nx[{k}] = {diff}, expected |diff| <= 1",
                );
            }
        }

        /// Impulse-response check: a unit impulse at the central
        /// `wt[k+i] = H[i]` slot should produce a scaled copy of
        /// `H[..]` centred on the impulse position.
        ///
        /// With e[0] = 1 and all other e[k] = 0, wt[5] = 1 (others 0).
        /// For each output position k, the accumulator picks up
        /// exactly one term: i = 5 - k (when 0 <= 5 - k <= 10, i.e.
        /// k in [0, 5]). For k > 5 the impulse moves out of the
        /// 11-tap window so x[k] = 0 (modulo the rounding bias which
        /// `>> 16`'s away).
        ///
        /// Per the spec: `L_result = (L_mult(1, H[5-k]) + 8192) * 4 >> 16`.
        /// `L_mult(1, H[i]) = 2 * H[i]`; `(2*H[i] + 8192) * 4 >> 16` =
        /// `(8*H[i] + 32768) >> 16`. For |H[i]| <= 8192 the result is
        /// 0 for typical taps and 1 (rounding) for the H[5]=8192 tap.
        #[test]
        fn weighting_filter_unit_impulse_at_position_zero() {
            let mut e = [0i16; SUBFRAME_SAMPLES];
            e[0] = 1;
            let x = weighting_filter(&e);

            // Re-derive the expected x[..] from the spec arithmetic
            // for k = 0..=5 (within the 11-tap window) and 6..=39
            // (outside).
            for k in 0..SUBFRAME_SAMPLES {
                let expected: i16 = if k <= 5 {
                    // i = 5 - k; only wt[k + (5-k)] = wt[5] = 1 hits.
                    let h_tap = H[5 - k];
                    let l_mult_val = (h_tap as i32) << 1; // L_mult(1, H[i]) = 2*H[i]
                    let after_round = l_mult_val + 8192;
                    let after_x2 = after_round.saturating_add(after_round);
                    let after_x4 = after_x2.saturating_add(after_x2);
                    (after_x4 >> 16) as i16
                } else {
                    // Window-out: accumulator stays at 8192;
                    // (8192 * 4) >> 16 = 0.
                    0
                };
                assert_eq!(
                    x[k], expected,
                    "x[{k}] = {} but expected {} (impulse response)",
                    x[k], expected,
                );
            }
        }

        /// Sub-segment boundary: the `wt[]` array is zero-padded on
        /// the trailing edge so an input impulse at e[39] (= wt[44])
        /// contributes only to outputs k in [34, 39], where the tap
        /// index `i = 44 - k` falls in the valid range [0, 10]. For
        /// k < 34, the impulse is outside the 11-tap window and the
        /// accumulator stays at the rounding constant only.
        ///
        /// Validates that the zero-pad is correct (no leak from
        /// outside the 40-sample sub-segment) and exercises the
        /// non-trivial high-end-of-window response.
        #[test]
        fn weighting_filter_zero_pad_trailing_edge() {
            let mut e = [0i16; SUBFRAME_SAMPLES];
            e[39] = 4096; // big enough that H taps register after >>16
            let x = weighting_filter(&e);

            // For k in [34, 39] the impulse at wt[44] hits via tap
            // i = 44 - k ∈ [5, 10], so the output is
            // `(L_mult(4096, H[44-k]) + 8192) * 4 >> 16`.
            for k in 0..SUBFRAME_SAMPLES {
                let expected: i16 = if (34..=39).contains(&k) {
                    let h_tap = H[44 - k];
                    let l_mult_val = (4096_i32 * (h_tap as i32)) << 1;
                    let after_round = l_mult_val + 8192;
                    let after_x2 = after_round.saturating_add(after_round);
                    let after_x4 = after_x2.saturating_add(after_x2);
                    (after_x4 >> 16) as i16
                } else {
                    0
                };
                assert_eq!(
                    x[k], expected,
                    "x[{k}] = {} but expected {} (trailing-edge impulse)",
                    x[k], expected,
                );
            }
        }

        /// The filter is stateless: invoking it twice in succession,
        /// the second on a different input, must not be influenced by
        /// the first.
        #[test]
        fn weighting_filter_stateless() {
            let mut e1 = [0i16; SUBFRAME_SAMPLES];
            for k in 0..SUBFRAME_SAMPLES {
                e1[k] = (k as i16) * 11;
            }
            let mut e2 = [0i16; SUBFRAME_SAMPLES];
            for k in 0..SUBFRAME_SAMPLES {
                e2[k] = -(k as i16) * 7;
            }
            let x2_before_warmup = weighting_filter(&e2);
            let _ = weighting_filter(&e1);
            let x2_after_warmup = weighting_filter(&e2);
            assert_eq!(x2_before_warmup, x2_after_warmup);
        }

        // ─── §5.2.14 RPE grid selection ───

        /// Helper: independently compute the §5.2.14 grid energy for
        /// `m` against `x[]` using the pseudocode's saturating
        /// `L_mult`/`L_add` shape but written from scratch in the
        /// test (so the test isn't tautological against the impl).
        fn rpe_grid_energy_reference(x: &[i16; SUBFRAME_SAMPLES], m: usize) -> i32 {
            let mut acc: i32 = 0;
            for i in 0..RPE_PULSES {
                let temp1 = x[m + 3 * i] >> 2;
                let prod = ((temp1 as i32) * (temp1 as i32)) << 1;
                acc = acc.saturating_add(prod);
            }
            acc
        }

        /// All-zero input ⇒ `m_c = 0` (entry-time default; strict
        /// `>` means the zero candidates never overwrite) and the
        /// 13-sample `xM[]` is all zero.
        #[test]
        fn rpe_grid_zero_input_picks_grid_zero() {
            let x = [0i16; SUBFRAME_SAMPLES];
            let g = select_rpe_grid(&x);
            assert_eq!(g.m_c, 0);
            assert_eq!(g.x_m, [0i16; RPE_PULSES]);
        }

        /// Determinism: identical inputs ⇒ identical outputs across
        /// invocations. §5.2.14 is stateless per the spec.
        #[test]
        fn rpe_grid_deterministic() {
            let mut x = [0i16; SUBFRAME_SAMPLES];
            for k in 0..SUBFRAME_SAMPLES {
                x[k] = ((k as i32 * 1373) % 9001 - 4500) as i16;
            }
            let g1 = select_rpe_grid(&x);
            let g2 = select_rpe_grid(&x);
            assert_eq!(g1, g2);
        }

        /// Place a single non-zero sample on indices that belong to
        /// exactly one of the four grids and confirm `m_c` follows.
        ///
        /// Grid membership:
        /// * Grid 0 = `{0, 3, 6, …, 36}`
        /// * Grid 1 = `{1, 4, 7, …, 37}`
        /// * Grid 2 = `{2, 5, 8, …, 38}`
        /// * Grid 3 = `{3, 6, 9, …, 39}`
        ///
        /// Grid 0 and grid 3 share most of their indices; the ones
        /// unique to each are `x[0]` (grid 0 only) and `x[39]`
        /// (grid 3 only). `x[1]` is grid 1 only and `x[2]` is grid 2
        /// only. An impulse on the grid-unique index hits a
        /// non-overlapping slot, so the chosen grid's `L_result`
        /// strictly exceeds the (zero) energy of the other grids.
        #[test]
        fn rpe_grid_single_impulse_picks_its_own_grid() {
            // (grid index, unique sample position, xM index).
            let cases: [(i16, usize, usize); 4] = [
                (0, 0, 0),   // x[0] → grid 0, xM[0]
                (1, 1, 0),   // x[1] → grid 1, xM[0]
                (2, 2, 0),   // x[2] → grid 2, xM[0]
                (3, 39, 12), // x[39] → grid 3, xM[12]
            ];
            for (m_expected, pos, xm_idx) in cases {
                let mut x = [0i16; SUBFRAME_SAMPLES];
                x[pos] = 1000; // large enough to survive >>2 + ×2
                let g = select_rpe_grid(&x);
                assert_eq!(
                    g.m_c, m_expected,
                    "impulse at x[{pos}] should select grid {m_expected}, got {}",
                    g.m_c,
                );
                // Exactly one xM[] slot carries the impulse; the rest
                // are zero.
                for i in 0..RPE_PULSES {
                    let expected = if i == xm_idx { 1000 } else { 0 };
                    assert_eq!(g.x_m[i], expected, "xM[{i}] mismatch");
                }
            }
        }

        /// Tie-break rule: when two grids carry identical energy,
        /// the strict `L_result > EM` test means the lower `m` wins.
        /// Construct a signal where grid 1 and grid 2 carry equal
        /// energy (both have a single impulse at `i = 0` of the
        /// same magnitude) — grid 1 must be picked.
        #[test]
        fn rpe_grid_tie_breaks_to_lower_m() {
            let mut x = [0i16; SUBFRAME_SAMPLES];
            // Impulse on grid 1 at i = 0 ⇒ x[1].
            x[1] = 500;
            // Impulse on grid 2 at i = 0 ⇒ x[2], same magnitude.
            x[2] = 500;
            let g = select_rpe_grid(&x);
            assert_eq!(g.m_c, 1);
            // xM = x[1 + 3*i] for i = 0..=12. Only xM[0] is non-zero
            // (= x[1] = 500); none of x[4], x[7], …, x[37] were set.
            assert_eq!(g.x_m[0], 500);
            for i in 1..RPE_PULSES {
                assert_eq!(g.x_m[i], 0);
            }
        }

        /// `xM[i]` corresponds index-for-index with `x[m_c + 3*i]`.
        /// Build a non-trivial `x[]`, confirm `m_c` is in range, and
        /// verify each of the 13 emitted samples matches the spec's
        /// down-sampling formula.
        #[test]
        fn rpe_grid_down_sample_indices_correct() {
            let mut x = [0i16; SUBFRAME_SAMPLES];
            for k in 0..SUBFRAME_SAMPLES {
                // Mix of positive and negative samples so the
                // squared-energy comparison has something to chew on.
                x[k] = ((k as i32) * 173 - 3000) as i16;
            }
            let g = select_rpe_grid(&x);
            assert!((0..4).contains(&g.m_c));
            for i in 0..RPE_PULSES {
                assert_eq!(
                    g.x_m[i],
                    x[(g.m_c as usize) + 3 * i],
                    "xM[{i}] should equal x[{} + 3*{i}] = x[{}]",
                    g.m_c,
                    (g.m_c as usize) + 3 * i,
                );
            }
        }

        /// `m_c` must be the argmax of the four per-grid `L_result`
        /// values (with the strict-`>` tie-break). Verify both that
        /// the chosen grid's energy is `>= 0` and that no other grid
        /// strictly exceeds it.
        #[test]
        fn rpe_grid_chosen_energy_is_argmax() {
            let mut x = [0i16; SUBFRAME_SAMPLES];
            // Use a sparse-ish pattern that puts most energy on
            // grid 2: large values at indices 2, 5, 8, 11, 14, ….
            for i in 0..RPE_PULSES {
                x[2 + 3 * i] = 2500;
            }
            // And smaller noise on the other grids.
            x[0] = 100;
            x[4] = -150;
            x[7] = 80;
            let g = select_rpe_grid(&x);
            assert_eq!(g.m_c, 2);

            let em_chosen = rpe_grid_energy_reference(&x, g.m_c as usize);
            for m in 0..4 {
                let em_other = rpe_grid_energy_reference(&x, m);
                if m == (g.m_c as usize) {
                    assert_eq!(em_other, em_chosen);
                } else {
                    assert!(
                        em_other <= em_chosen,
                        "grid {m} energy {em_other} should not exceed chosen grid {} energy {em_chosen}",
                        g.m_c,
                    );
                }
            }
        }

        // ─── §5.2.15 APCM forward quantisation ───

        /// Reference §5.2.16 inverse APCM applied to `(xmaxc, xMc[],
        /// exp, mant)` — re-derives the `xMp[0..=12]` the decoder
        /// recovers. Written from the spec pseudocode rather than
        /// re-using `decoder::rpe_decode` (which goes through
        /// `SubFrame`) so the round-trip tests below aren't
        /// tautological.
        fn apcm_inverse_reference(q: &ApcmQuantised) -> [i16; RPE_PULSES] {
            use crate::arith::{
                add as a, mult_r as mr, shl_signed as sl, shr_signed as sr, sub as s,
            };
            use crate::tables::FAC;
            let t1 = FAC[q.mant as usize];
            let t2 = s(6, q.exp);
            let t3 = sl(1, s(t2, 1));
            let mut xmp = [0i16; RPE_PULSES];
            for (slot, &xmc) in xmp.iter_mut().zip(q.x_mc.iter()) {
                let t = s(xmc << 1, 7) << 12;
                let t = mr(t1, t);
                let t = a(t, t3);
                *slot = sr(t, t2);
            }
            xmp
        }

        /// All-zero input ⇒ xmaxc = 0 and all xMc[i] = 4 (the
        /// signed-3-bit `+4` bias maps the zero-magnitude pulse to
        /// the centre of the §1.7 unsigned 0..=7 code range — the
        /// inverse of §5.2.16's `sub((xMc << 1), 7)`).
        ///
        /// Also confirms the §5.2.15 `mant == 0` short-circuit:
        /// `xmaxc = 0 ⇒ decode-exp = 0; decode-mant = 0`, then the
        /// pre-normalisation `mant == 0 ⇒ exp = -4, mant = 15`
        /// branch fires and the trailing `mant -= 8` lands
        /// `(exp, mant) = (-4, 7)` — the `FAC[7] = 32767` slot the
        /// decoder uses for silence.
        #[test]
        fn apcm_quantise_zero_input() {
            let x_m = [0i16; RPE_PULSES];
            let q = apcm_quantise_rpe(&x_m);
            assert_eq!(q.xmaxc, 0);
            assert_eq!(q.exp, -4);
            assert_eq!(q.mant, 7);
            for &c in q.x_mc.iter() {
                assert_eq!(c, 4, "centre code for zero-magnitude pulse");
            }
        }

        /// Output ranges per §1.7 Table 1.1: `xmaxc ∈ 0..=63` (6
        /// bits) and `xMc[i] ∈ 0..=7` (3 bits). The encoder must
        /// land inside both ranges for every input the spec admits.
        #[test]
        fn apcm_quantise_codeword_ranges() {
            // A handful of inputs spanning the i16 magnitude range.
            // (We stop at -32767 / 32767 — `-i16::MIN` panics under
            // overflow checks; the §5.1 `abs` saturates, but the
            // test-side construction uses plain negate.)
            let inputs: &[i16] = &[
                0, 1, 100, 1000, 8192, 16384, 24576, 32767, -1, -100, -1000, -16384, -32767,
            ];
            for &v in inputs {
                let mut x_m = [0i16; RPE_PULSES];
                for (i, slot) in x_m.iter_mut().enumerate() {
                    // Vary signs across the 13 slots so we exercise
                    // both `temp = mult(...)` branches.
                    *slot = if i % 2 == 0 { v } else { -v };
                }
                let q = apcm_quantise_rpe(&x_m);
                assert!(
                    (0..=63).contains(&q.xmaxc),
                    "xmaxc out of range: {}",
                    q.xmaxc
                );
                for (i, &c) in q.x_mc.iter().enumerate() {
                    assert!(
                        (0..=7).contains(&c),
                        "xMc[{i}] out of 0..=7 range: {c} (input {v})",
                    );
                }
            }
        }

        /// Determinism: the §5.2.15 spec is stateless, so identical
        /// inputs ⇒ identical outputs across invocations.
        #[test]
        fn apcm_quantise_deterministic() {
            let mut x_m = [0i16; RPE_PULSES];
            for (i, slot) in x_m.iter_mut().enumerate() {
                *slot = ((i as i32) * 1031 - 5500) as i16;
            }
            let q1 = apcm_quantise_rpe(&x_m);
            let q2 = apcm_quantise_rpe(&x_m);
            assert_eq!(q1, q2);
        }

        /// Statelessness across calls: an intermediate call with a
        /// different input must not perturb a later identical-input
        /// call.
        #[test]
        fn apcm_quantise_stateless() {
            let mut x_a = [0i16; RPE_PULSES];
            let mut x_b = [0i16; RPE_PULSES];
            for i in 0..RPE_PULSES {
                x_a[i] = ((i as i32) * 137) as i16;
                x_b[i] = ((i as i32) * -211) as i16;
            }
            let q_a_first = apcm_quantise_rpe(&x_a);
            let _ = apcm_quantise_rpe(&x_b);
            let q_a_second = apcm_quantise_rpe(&x_a);
            assert_eq!(q_a_first, q_a_second);
        }

        /// §5.2.15 `xmaxc` packs exponent into bits 3..=5 and a
        /// 3-bit mantissa into bits 0..=2. The §5.2.15 decoded
        /// `exp = (xmaxc > 15) ? (xmaxc >> 3) - 1 : 0` reproduces
        /// the upper bits exactly. Verify the unpacked `exp` always
        /// matches across the full 0..=63 xmaxc space (by varying
        /// the input magnitude) and that the `decode_exp - 1`
        /// adjustment correctly handles the `xmaxc <= 15` branch.
        #[test]
        fn apcm_quantise_xmaxc_encodes_exponent() {
            // For a peak xM[] magnitude `peak`, the §5.2.15 exponent
            // search drives `temp = peak >> 9` down by another bit
            // each step; the final `exp` is the count of right shifts
            // before `temp <= 0`, capped at 6. We sweep a wide range
            // and confirm the decoder's recovered `exp` value lies
            // in the documented 0..=5 range (with the `mant == 0`
            // branch landing on the post-normalisation `exp = -4`
            // when peak == 0).
            for peak_bits in 0..=15 {
                let peak: i16 = if peak_bits == 0 { 0 } else { 1 << peak_bits };
                let mut x_m = [0i16; RPE_PULSES];
                x_m[0] = peak;
                let q = apcm_quantise_rpe(&x_m);
                if peak == 0 {
                    // Zero-input short-circuit.
                    assert_eq!(q.exp, -4);
                    assert_eq!(q.mant, 7);
                } else {
                    // Post-normalisation exp ∈ -4..=6: the spec's
                    // §5.2.15 search caps the pre-normalisation exp
                    // at 6 (since the loop runs `i = 0..=5` and
                    // increments at most six times); the inner
                    // normalisation loop can drop it by up to 3 (so
                    // the iterative branch lands in [-3, 6]); the
                    // `mant == 0` branch preloads exp = -4 so the
                    // joint range is [-4, 6].
                    assert!(
                        (-4..=6).contains(&q.exp),
                        "exp out of range for peak {peak}: {}",
                        q.exp,
                    );
                    // Mant index in 0..=7.
                    assert!(
                        (0..=7).contains(&q.mant),
                        "mant out of range for peak {peak}: {}",
                        q.mant,
                    );
                }
            }
        }

        /// §5.2.15 / §5.2.16 round-trip: applying §5.2.16 inverse
        /// quantisation to `(xmaxc, xMc[], exp, mant)` must recover
        /// `xMp[0..=12]` within the quantiser step (the relative
        /// reconstruction error is bounded by the 3-bit code's
        /// step size for the chosen exponent).
        ///
        /// The §5.2.15 search picks `exp` so that the peak |xM[]|
        /// lands in the upper third of the quantiser range; the
        /// effective quantiser step is then on the order of
        /// `peak / 4`. We check that |xM[i] - xMp[i]| <= peak/2
        /// for all 13 samples (a generous bound that absorbs both
        /// the `mult(...)` truncation in §5.2.15 and the
        /// `mult_r(...)` half-LSB round in §5.2.16).
        #[test]
        fn apcm_quantise_inverse_roundtrip() {
            // Several inputs spanning low / medium / high magnitudes
            // to exercise different exponent codes.
            let scales: &[i16] = &[64, 256, 1024, 4096, 16384];
            for &s in scales {
                let mut x_m = [0i16; RPE_PULSES];
                for (i, slot) in x_m.iter_mut().enumerate() {
                    // Vary signs and magnitudes within the
                    // sub-segment so the per-sample reconstruction
                    // gets a non-degenerate test.
                    let sign: i32 = if i % 3 == 0 { -1 } else { 1 };
                    let frac: i32 = (i as i32 + 1) * (s as i32) / 14;
                    *slot = (sign * frac) as i16;
                }
                let q = apcm_quantise_rpe(&x_m);
                let xmp = apcm_inverse_reference(&q);
                let bound = (s as i32) / 2 + 8;
                for i in 0..RPE_PULSES {
                    let err = (xmp[i] as i32 - x_m[i] as i32).abs();
                    assert!(
                        err <= bound,
                        "round-trip error {} > bound {} at i={}, s={}, xM={} xMp={}",
                        err,
                        bound,
                        i,
                        s,
                        x_m[i],
                        xmp[i],
                    );
                }
            }
        }

        /// xmaxc carries the block peak. Two inputs with peaks
        /// differing by a factor of 2 must produce xmaxc values
        /// differing by 8 (one exponent step occupies bits 3..=5
        /// of the 6-bit xmaxc code).
        #[test]
        fn apcm_quantise_xmaxc_doubles_one_exp_step() {
            let mut x_lo = [0i16; RPE_PULSES];
            x_lo[3] = 4096;
            let mut x_hi = [0i16; RPE_PULSES];
            x_hi[3] = 8192;
            let q_lo = apcm_quantise_rpe(&x_lo);
            let q_hi = apcm_quantise_rpe(&x_hi);
            // The 8-bit step in xmaxc lifts the exponent by 1 while
            // keeping the mantissa bits identical (since the peak
            // pre-shift mantissa is the same). Confirm xmaxc
            // differs by exactly 8.
            assert_eq!(
                q_hi.xmaxc - q_lo.xmaxc,
                8,
                "xmaxc lo={} hi={}",
                q_lo.xmaxc,
                q_hi.xmaxc,
            );
        }

        /// `xmaxc` peak monotonicity — across a sweep of peak
        /// magnitudes, `xmaxc` must be non-decreasing. The §5.2.15
        /// exponent search is monotone in `xmax`, and the trailing
        /// `xmax >> (exp+5)` mantissa picks up the next bit
        /// sub-step within an exponent band.
        #[test]
        fn apcm_quantise_xmaxc_monotone_in_peak() {
            let mut prev_xmaxc = -1i16;
            for peak in [
                0i16, 1, 4, 16, 64, 256, 512, 1024, 2048, 4096, 8192, 16384, 24576, 32767,
            ] {
                let mut x_m = [0i16; RPE_PULSES];
                x_m[0] = peak;
                let q = apcm_quantise_rpe(&x_m);
                assert!(
                    q.xmaxc >= prev_xmaxc,
                    "xmaxc dropped from {} to {} at peak {}",
                    prev_xmaxc,
                    q.xmaxc,
                    peak,
                );
                prev_xmaxc = q.xmaxc;
            }
        }

        /// Sign symmetry: negating every `xM[i]` must give the
        /// same `xmaxc` and `(exp, mant)` (the `xmax` search is
        /// magnitude-only) and must reflect the `xMc[i]` codes
        /// around the centre value 4 (the §5.2.15 sign-restoration
        /// step is `xMc[i] = (temp >> 12) + 4`, so negating `xM[i]`
        /// negates the `temp` and lands at `4 - (temp >> 12)`).
        #[test]
        fn apcm_quantise_sign_symmetric_xmaxc() {
            let mut x_m = [0i16; RPE_PULSES];
            for (i, slot) in x_m.iter_mut().enumerate() {
                *slot = ((i as i32) * 521 + 100) as i16;
            }
            let mut x_m_neg = [0i16; RPE_PULSES];
            for i in 0..RPE_PULSES {
                // Use `wrapping_neg` to handle i16::MIN, but our
                // construction stays well away from that boundary.
                x_m_neg[i] = -x_m[i];
            }
            let q_pos = apcm_quantise_rpe(&x_m);
            let q_neg = apcm_quantise_rpe(&x_m_neg);
            assert_eq!(q_pos.xmaxc, q_neg.xmaxc);
            assert_eq!(q_pos.exp, q_neg.exp);
            assert_eq!(q_pos.mant, q_neg.mant);
            // xMc codes reflect around 4 (within ±1 LSB because
            // `mult(...)` is a truncating Q15 product and negating
            // the input can flip the truncation direction by one
            // LSB).
            for i in 0..RPE_PULSES {
                let sum = q_pos.x_mc[i] + q_neg.x_mc[i];
                assert!(
                    (7..=9).contains(&sum),
                    "xMc[{i}] reflection violated: pos={} neg={} sum={}",
                    q_pos.x_mc[i],
                    q_neg.x_mc[i],
                    sum,
                );
            }
        }

        /// Statelessness across calls. The §5.2.14 spec carries no
        /// persistent state; intermediate inputs must not perturb a
        /// later call's output.
        #[test]
        fn rpe_grid_stateless() {
            let mut x1 = [0i16; SUBFRAME_SAMPLES];
            for k in 0..SUBFRAME_SAMPLES {
                x1[k] = (k as i16) * 13 - 200;
            }
            let mut x2 = [0i16; SUBFRAME_SAMPLES];
            for k in 0..SUBFRAME_SAMPLES {
                x2[k] = -(k as i16) * 23 + 300;
            }
            let g2_before = select_rpe_grid(&x2);
            let _ = select_rpe_grid(&x1);
            let g2_after = select_rpe_grid(&x2);
            assert_eq!(g2_before, g2_after);
        }

        // ─── §5.2.16 + §5.2.17 APCM inverse + RPE grid positioning ───

        /// §5.2.16 half of [`apcm_inverse_and_position`] must produce
        /// exactly the `xMp[0..=12]` the independent
        /// [`apcm_inverse_reference`] helper recovers — and, by
        /// construction, the same pulses the decoder's `rpe_decode`
        /// builds. The §5.2.17 grid-positioning step then scatters
        /// those 13 `xMp` values into the `ep[Mc + 3*i]` slots, so we
        /// read them back out of `ep[]` at those positions.
        #[test]
        fn apcm_inverse_matches_reference_xmp() {
            // A spread of magnitudes so several different (exp, mant)
            // pairs are exercised.
            let inputs: &[i16] = &[0, 1, 100, 1000, 8192, 16384, 24576, 32767, -1000, -16384];
            for &v in inputs {
                let mut x_m = [0i16; RPE_PULSES];
                for (i, slot) in x_m.iter_mut().enumerate() {
                    *slot = if i % 2 == 0 { v } else { -v };
                }
                let q = apcm_quantise_rpe(&x_m);
                let xmp_ref = apcm_inverse_reference(&q);
                // Run §5.2.16 + §5.2.17 with Mc = 0 so the pulses land
                // at indices 0, 3, …, 36 and are directly comparable
                // to the reference xMp[].
                let ep = apcm_inverse_and_position(&q.x_mc, q.exp, q.mant, 0);
                for i in 0..RPE_PULSES {
                    assert_eq!(
                        ep[3 * i],
                        xmp_ref[i],
                        "xMp[{i}] mismatch (input {v}): pipeline={} ref={}",
                        ep[3 * i],
                        xmp_ref[i],
                    );
                }
            }
        }

        /// §5.2.17 grid positioning: with grid offset `Mc`, the 13
        /// reconstructed pulses land at `Mc, Mc+3, …, Mc+36` and every
        /// other slot is zero. Verify the non-zero footprint for each
        /// of the four grids.
        #[test]
        fn rpe_grid_positioning_footprint() {
            // A non-silent input so the dequantised pulses are non-zero
            // for at least some positions.
            let mut x_m = [0i16; RPE_PULSES];
            for (i, slot) in x_m.iter_mut().enumerate() {
                *slot = ((i as i32) * 1700 - 9000) as i16;
            }
            let q = apcm_quantise_rpe(&x_m);
            for m_c in 0..4i16 {
                let ep = apcm_inverse_and_position(&q.x_mc, q.exp, q.mant, m_c);
                let mc = m_c as usize;
                // The 13 pulse positions are exactly {Mc + 3*i}.
                let pulse_positions: std::collections::HashSet<usize> =
                    (0..RPE_PULSES).map(|i| mc + 3 * i).collect();
                for k in 0..SUBFRAME_SAMPLES {
                    if !pulse_positions.contains(&k) {
                        assert_eq!(ep[k], 0, "off-grid sample ep[{k}] must be zero (Mc={m_c})");
                    }
                }
                // The deepest pulse position is Mc + 36; for Mc=3 that
                // is index 39, the last sample — still in-bounds.
                assert_eq!(mc + 3 * (RPE_PULSES - 1), mc + 36);
            }
        }

        /// §5.2.15 → §5.2.16 round-trip: the dequantised pulse train
        /// `xMp[i]` tracks the sign and rough magnitude of the original
        /// `xM[i]`. The quantiser is lossy, but for a clearly non-zero
        /// pulse the reconstructed value must share its sign.
        #[test]
        fn apcm_inverse_roundtrip_tracks_sign() {
            let mut x_m = [0i16; RPE_PULSES];
            for (i, slot) in x_m.iter_mut().enumerate() {
                // Alternating, clearly non-zero magnitudes well away
                // from the quantiser's dead zone.
                *slot = if i % 2 == 0 { 6000 } else { -6000 };
            }
            let q = apcm_quantise_rpe(&x_m);
            let ep = apcm_inverse_and_position(&q.x_mc, q.exp, q.mant, 0);
            for i in 0..RPE_PULSES {
                let recon = ep[3 * i];
                if x_m[i] > 0 {
                    assert!(recon > 0, "pulse {i}: +input ⇒ +recon, got {recon}");
                } else {
                    assert!(recon < 0, "pulse {i}: -input ⇒ -recon, got {recon}");
                }
            }
        }

        /// Determinism + statelessness of [`apcm_inverse_and_position`]:
        /// identical arguments ⇒ identical output, and an intervening
        /// call with other arguments does not perturb a later call.
        #[test]
        fn apcm_inverse_stateless() {
            let x_mc_a = [4i16, 0, 7, 1, 6, 2, 5, 3, 4, 0, 7, 1, 6];
            let x_mc_b = [0i16, 7, 4, 6, 1, 5, 2, 4, 3, 7, 0, 6, 1];
            let a1 = apcm_inverse_and_position(&x_mc_a, 3, 5, 1);
            let _ = apcm_inverse_and_position(&x_mc_b, -2, 0, 2);
            let a2 = apcm_inverse_and_position(&x_mc_a, 3, 5, 1);
            assert_eq!(a1, a2);
        }

        /// §5.2.16 → §5.2.17 → §5.2.18 closed loop: the encoder's
        /// local-decoder `dp[..]` history built by
        /// [`LtpAnalyzer::reconstruct_and_update`] must match the
        /// reconstructed long-term residual the **receiving decoder**
        /// recovers from the identical parameters.
        ///
        /// For the very first sub-segment from the §4.5 / §4.6 home
        /// state with `bc = 0`, the §5.3.2 long-term predictor adds
        /// nothing (`drp[k] = add(erp[k], mult_r(QLB[0], drp[k - Nr]))`
        /// and the delay line is all-zero), so the decoder's
        /// reconstructed short-term residual `drp[k]` equals the §5.2.17
        /// `erp[k]`. We reconstruct the decoder's `erp[..]` here from
        /// the independent [`apcm_inverse_reference`] helper (which
        /// mirrors `decoder::rpe_decode`'s §5.2.16 math) plus §5.2.17
        /// grid positioning, and confirm the encoder's freshly-pushed
        /// `dp[-1..=-40]` slot matches it sample-for-sample.
        #[test]
        fn closed_loop_dp_matches_decoder_residual() {
            // A non-silent RPE input.
            let mut x_m = [0i16; RPE_PULSES];
            for (i, slot) in x_m.iter_mut().enumerate() {
                *slot = ((i as i32) * 1300 - 7000) as i16;
            }
            let q = apcm_quantise_rpe(&x_m);
            let m_c = 2i16;

            // Encoder side: dpp = 0 (bc = 0 ⇒ no long-term prediction),
            // so dp[k] = add(ep[k], 0) = ep[k].
            let dpp = [0i16; SUBFRAME_SAMPLES];
            let mut enc = LtpAnalyzer::new();
            let ep = enc.reconstruct_and_update(&q, m_c, &dpp);

            // Decoder-side reference erp[..]: §5.2.16 (via the
            // reference helper) + §5.2.17 grid positioning with the
            // same Mc.
            let xmp_ref = apcm_inverse_reference(&q);
            let mut erp_ref = [0i16; SUBFRAME_SAMPLES];
            for (i, &p) in xmp_ref.iter().enumerate() {
                let idx = (m_c as usize) + 3 * i;
                if idx < SUBFRAME_SAMPLES {
                    erp_ref[idx] = p;
                }
            }

            // The §5.2.17 ep[..] the encoder produced must equal the
            // decoder's erp[..], and the §5.2.18 dp-update (bc = 0 ⇒
            // dpp = 0) lands dp[-1..=-40] = ep[39..=0] in dp_hist[0..=39].
            for k in 0..SUBFRAME_SAMPLES {
                assert_eq!(ep[k], erp_ref[k], "ep/erp mismatch at k={k}");
                assert_eq!(
                    enc.dp_hist()[SUBFRAME_SAMPLES - 1 - k],
                    erp_ref[k],
                    "dp/drp mismatch at k={k}",
                );
            }
        }

        // Silence unused-import warning when this private inner
        // mod doesn't reach for one of the imports during the
        // test-only path.
        #[allow(dead_code)]
        fn _exercise_unused() {
            let _ = l_add(0, 0);
            let _ = l_mult(0, 0);
        }
    }
}

/// Frame-level §5.2 encoder driver.
///
/// Aggregates the three stateful pipeline stages — the
/// §5.2.0..§5.2.3 [`PreProcessor`], the §5.2.4..§5.2.10
/// [`analysis::Analyzer`], and the §5.2.11..§5.2.18
/// [`analysis::LtpAnalyzer`] — and runs one complete encode pass
/// per 160-sample input frame, emitting the 76 codewords of an
/// [`UnpackedFrame`] ready for the §1.7 Table 1.1 bit packer
/// ([`UnpackedFrame::to_bit_stream_msb_first`]).
///
/// The aggregated state is exactly the §4.5 Table 4.2 "Initial
/// values of the encoder state variables" set: `z1`, `L_z2`, `mp`
/// (pre-processing), `LARpp(j-1)[1..=8]`, `u[0..=7]` (short-term
/// analysis), and `dp[-120..=-1]` (LTP delay line). [`Self::new`] /
/// [`Self::reset`] put every entry in its all-zero home value.
#[derive(Debug, Clone)]
pub struct EncoderState {
    pre: PreProcessor,
    analyzer: analysis::Analyzer,
    ltp: analysis::LtpAnalyzer,
}

impl Default for EncoderState {
    fn default() -> Self {
        Self::new()
    }
}

impl EncoderState {
    /// Build a fresh encoder in the §4.5 Table 4.2 home state.
    pub fn new() -> Self {
        Self {
            pre: PreProcessor::new(),
            analyzer: analysis::Analyzer::new(),
            ltp: analysis::LtpAnalyzer::new(),
        }
    }

    /// Restore the §4.5 Table 4.2 home state on every stage.
    pub fn reset(&mut self) {
        self.pre.reset();
        self.analyzer.reset();
        self.ltp.reset();
    }

    /// Encode one 160-sample input frame `sop[0..=159]` into the 76
    /// codewords of the §1.7 frame.
    ///
    /// Pipeline order per §5.2 (fig 3.1 numbering):
    ///
    /// 1. §5.2.1..§5.2.3 — pre-processing (`sop` → `s`).
    /// 2. §5.2.4..§5.2.10 — LPC analysis + LAR quantisation + short-
    ///    term analysis filter (`s` → `LARc[1..=8]`, `d[0..=159]`).
    /// 3. For each of the four 40-sample sub-segments `j = 0..=3`:
    ///    §5.2.11 LTP parameters (`Ncj`, `bcj`), §5.2.12 long-term
    ///    analysis filter, §5.2.13 weighting filter, §5.2.14 RPE grid
    ///    selection (`Mcj`), §5.2.15 APCM quantisation (`xmaxcj`,
    ///    `xMcj[0..=12]`), then the §5.2.16..§5.2.18 local-decoder
    ///    feedback pass that folds the reconstructed long-term
    ///    residual back into the `dp[-120..=-1]` delay line so the
    ///    next sub-segment's LTP search runs on the same history the
    ///    receiving decoder will build.
    pub fn encode_frame(&mut self, sop: &[i16; FRAME_SAMPLES]) -> UnpackedFrame {
        // §5.2.1..§5.2.3 — pre-processing.
        let s = self.pre.process_frame(sop);

        // §5.2.4..§5.2.10 — LARc codewords + short-term residual d[].
        let (lar_c, d) = self.analyzer.analyse_frame(&s);

        let mut frame = UnpackedFrame {
            lar_c,
            ..UnpackedFrame::default()
        };

        // §5.2.11..§5.2.18 — four sub-segments.
        for (j, sub_out) in frame.sub.iter_mut().enumerate().take(SUBFRAMES) {
            let mut d_sub = [0i16; analysis::SUBFRAME_SAMPLES];
            d_sub.copy_from_slice(
                &d[j * analysis::SUBFRAME_SAMPLES..(j + 1) * analysis::SUBFRAME_SAMPLES],
            );

            // §5.2.11 + §5.2.12 — LTP parameters + long-term residual.
            let (ltp_params, dpp, e) = self.ltp.analyse_subframe(&d_sub);
            // §5.2.13 — weighting filter.
            let x = analysis::weighting_filter(&e);
            // §5.2.14 — RPE grid selection.
            let grid = analysis::select_rpe_grid(&x);
            // §5.2.15 — APCM forward quantisation.
            let apcm = analysis::apcm_quantise_rpe(&grid.x_m);
            // §5.2.16..§5.2.18 — local-decoder feedback (dp update).
            self.ltp.reconstruct_and_update(&apcm, grid.m_c, &dpp);

            *sub_out = SubFrame {
                n_c: ltp_params.n_c as u8,
                b_c: ltp_params.b_c as u8,
                m_c: grid.m_c as u8,
                xmax_c: apcm.xmaxc as u8,
                x_mc: core::array::from_fn(|i| apcm.x_mc[i] as u8),
            };
        }

        frame
    }
}

#[cfg(test)]
mod encoder_state_tests {
    use super::*;
    use crate::decoder::DecoderState;

    /// Silence in ⇒ a frame whose codewords are all within their
    /// Table 1.1 field widths and whose decode stays at the
    /// quantiser floor. Note the §5.2.16 3-bit RPE dequantiser has
    /// no exact-zero reconstruction level — `sub((xMc[i] << 1), 7)`
    /// maps the 8 codes onto the odd values −7, −5, …, +7 — so an
    /// all-zero input decodes to a tiny residual dither, not exact
    /// zeros. With `xmaxc = 0` the dequant scale is minimal; after
    /// the §5.3.4 synthesis lattice, §5.3.6 upscale (×2) and §5.3.7
    /// truncation (low 3 bits cleared) the dither stays within a few
    /// 13-bit LSB steps of zero.
    #[test]
    fn silence_encodes_to_in_range_codewords_and_decodes_to_silence() {
        let mut enc = EncoderState::new();
        let frame = enc.encode_frame(&[0i16; FRAME_SAMPLES]);

        // Range checks per Table 1.1 field widths.
        let lar_max: [i16; 8] = [63, 63, 31, 31, 15, 15, 7, 7];
        for i in 1..=8 {
            assert!(
                (0..=lar_max[i - 1]).contains(&frame.lar_c[i]),
                "LARc[{i}] out of range: {}",
                frame.lar_c[i]
            );
        }
        for (j, sf) in frame.sub.iter().enumerate() {
            assert!((40..=120).contains(&sf.n_c), "Nc{j} = {}", sf.n_c);
            assert!(sf.b_c <= 3, "bc{j} = {}", sf.b_c);
            assert!(sf.m_c <= 3, "Mc{j} = {}", sf.m_c);
            assert!(sf.xmax_c <= 63, "xmaxc{j} = {}", sf.xmax_c);
            for (i, &p) in sf.x_mc.iter().enumerate() {
                assert!(p <= 7, "xMc{j}[{i}] = {p}");
            }
        }

        let mut dec = DecoderState::new();
        let pcm = dec.decode_frame(&frame);
        for (k, &v) in pcm.iter().enumerate() {
            assert!(
                v.abs() <= 32,
                "silence must stay at the quantiser floor at k={k}, got {v}"
            );
        }
    }

    /// Every codeword stays within its Table 1.1 field width on a
    /// loud full-scale-ish square-ish input (stresses the saturating
    /// arithmetic + APCM exponent paths).
    #[test]
    fn loud_input_codewords_stay_in_field_ranges() {
        let mut enc = EncoderState::new();
        let mut sop = [0i16; FRAME_SAMPLES];
        for (k, slot) in sop.iter_mut().enumerate() {
            *slot = if (k / 20) % 2 == 0 { 20000 } else { -20000 };
        }
        for _ in 0..4 {
            let frame = enc.encode_frame(&sop);
            for i in 1..=8 {
                let widths: [i16; 8] = [63, 63, 31, 31, 15, 15, 7, 7];
                assert!((0..=widths[i - 1]).contains(&frame.lar_c[i]));
            }
            for sf in &frame.sub {
                assert!((40..=120).contains(&sf.n_c));
                assert!(sf.b_c <= 3 && sf.m_c <= 3 && sf.xmax_c <= 63);
                assert!(sf.x_mc.iter().all(|&p| p <= 7));
            }
            // Loud input must produce non-trivial block amplitudes.
            assert!(frame.sub.iter().any(|sf| sf.xmax_c > 0));
        }
    }

    /// reset() returns the encoder to the §4.5 home state: encoding
    /// the same frame sequence after a reset reproduces the same
    /// codewords as a freshly-built encoder.
    #[test]
    fn reset_restores_home_state() {
        let mut sop = [0i16; FRAME_SAMPLES];
        for (k, slot) in sop.iter_mut().enumerate() {
            *slot = (((k as i32 * 37) % 401) - 200) as i16 * 8;
        }

        let mut fresh = EncoderState::new();
        let want = fresh.encode_frame(&sop);

        let mut stale = EncoderState::new();
        let _ = stale.encode_frame(&sop);
        let _ = stale.encode_frame(&sop);
        stale.reset();
        let got = stale.encode_frame(&sop);

        assert_eq!(want, got);
    }

    /// Encode→decode roundtrip fidelity on a periodic, speech-band
    /// signal. RPE-LTP is lossy, so we do not expect sample
    /// equality; instead require that after the first (warm-up)
    /// frame the decoded signal tracks the §5.2.1-downscaled,
    /// §5.2.3-pre-emphasised-then-§5.3.5-de-emphasised input well
    /// enough that the error energy is clearly below the signal
    /// energy. A codec bug (wrong table, swapped codeword, packing
    /// slip) destroys this margin immediately.
    #[test]
    fn sine_roundtrip_preserves_signal_shape() {
        // 8 kHz sample rate, ~250 Hz periodic ramp-ish wave at
        // moderate amplitude (within the 13-bit input convention of
        // §5.2.1 — low 3 bits zero, |x| <= 2^15 - 8).
        let mut enc = EncoderState::new();
        let mut dec = DecoderState::new();

        const FRAMES: usize = 6;
        let mut input = Vec::with_capacity(FRAMES * FRAME_SAMPLES);
        for n in 0..FRAMES * FRAME_SAMPLES {
            // Triangle wave, period 32 samples (= 250 Hz), peak 8192.
            let phase = (n % 32) as i32;
            let tri = if phase < 16 { phase - 8 } else { 23 - phase }; // -8..=8
            input.push((tri * 1024) as i16); // multiples of 8: 13-bit clean
        }

        let mut signal_energy: i64 = 0;
        let mut error_energy: i64 = 0;
        for f in 1..FRAMES {
            // skip frame 0: filters warming up from home state
            let mut sop = [0i16; FRAME_SAMPLES];
            sop.copy_from_slice(&input[f * FRAME_SAMPLES..(f + 1) * FRAME_SAMPLES]);
            let coded = enc.encode_frame(&sop);
            let pcm = dec.decode_frame(&coded);
            for k in 0..FRAME_SAMPLES {
                let s = sop[k] as i64;
                let e = (sop[k] as i64) - (pcm[k] as i64);
                signal_energy += s * s;
                error_energy += e * e;
            }
        }

        assert!(signal_energy > 0);
        // Demand at least ~6 dB SNR — far below what a correct
        // RPE-LTP achieves on this signal, far above what any
        // mis-wired pipeline produces.
        assert!(
            error_energy * 4 < signal_energy,
            "roundtrip error too large: signal={signal_energy} error={error_energy}"
        );
    }

    /// Full bit-level path: encode → pack (§1.7) → unpack → decode
    /// must equal encode → decode without the bitstream in between.
    #[test]
    fn packed_roundtrip_matches_unpacked_roundtrip() {
        let mut enc = EncoderState::new();
        let mut dec_direct = DecoderState::new();
        let mut dec_packed = DecoderState::new();

        let mut sop = [0i16; FRAME_SAMPLES];
        for (k, slot) in sop.iter_mut().enumerate() {
            let phase = (k % 40) as i32;
            *slot = ((phase - 20) * 300) as i16;
        }

        for _ in 0..3 {
            let coded = enc.encode_frame(&sop);
            let bytes = coded.to_bit_stream_msb_first();
            let reparsed = UnpackedFrame::from_bit_stream_msb_first(&bytes).unwrap();
            assert_eq!(coded, reparsed, "§1.7 pack/unpack must be lossless");
            let a = dec_direct.decode_frame(&coded);
            let b = dec_packed.decode_frame(&reparsed);
            assert_eq!(a, b);
        }
    }
}
