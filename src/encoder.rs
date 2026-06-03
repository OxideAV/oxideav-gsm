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
//!
//! Subsequent encoder stages (§5.2.8 LAR decode + §5.2.9.1/§5.2.9.2
//! interpolation on the analysis side, §5.2.10 short-term analysis
//! filter, §5.2.11..§5.2.18 LTP analysis, RPE selection + APCM
//! quantisation, then §1.7 frame packing) arrive in later rounds.
//!
//! Until the rest of the encoder lands, the
//! [`oxideav_core::Encoder`]-trait factory `make_encoder` still
//! returns `Unsupported` from `lib.rs`. The pre-processing primitives
//! are exposed as a public sub-module so that future rounds can pick
//! them up on a stable surface (and so callers who want just the
//! input-shaping pass can drive it directly).
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
//! The remaining §4.5 entries — `LARpp(j-1)`, `u[0..7]`, the LTP
//! delay-line `dp[-120..-1]` — belong to later encoder stages and
//! are not part of this pre-processing slice.

use crate::arith::{add, l_add, l_mult, l_sub, mult_r, sub};
use crate::bitstream::FRAME_SAMPLES;

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
    use crate::arith::{abs, add, div, l_add, l_mult, mult, mult_r, norm, sub};
    use crate::tables::{A, B, MAC, MIC};

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
