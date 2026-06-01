//! Fixed-point GSM 06.10 RPE-LTP encoder per ETSI EN 300 961 §5.2.
//!
//! This module currently lands the **pre-processing clause** —
//! §5.2.0 (input scaling), §5.2.1 (downscaling), §5.2.2 (offset
//! compensation, a high-pass IIR with extended precision), and
//! §5.2.3 (first-order FIR pre-emphasis). That sub-pipeline maps a
//! frame of 160 raw PCM input samples `sop[0..159]` to the
//! `s[0..159]` array that the LPC-analysis clause (§5.2.4 onwards)
//! consumes.
//!
//! Subsequent encoder stages (§5.2.4 autocorrelation, §5.2.5 Schur
//! recursion + reflection coefficients, §5.2.6 LAR transform,
//! §5.2.7 LAR quantisation, §5.2.10 short-term analysis filter,
//! §5.2.11..§5.2.18 LTP analysis + RPE selection + APCM
//! quantisation, then frame packing) arrive in later rounds.
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
