//! Encoder 20 ms framing synchronization per ETSI EN 300 961 §6.3.3.3.
//!
//! When a GSM full-rate speech *encoder* is tested as a black box,
//! "usually there is no information available about where the encoder
//! starts its 20 ms segments of speech input" (§6.3.3.3). The spec
//! describes a two-step procedure — **bit synchronization** followed
//! by **frame synchronization** — that recovers both alignments using
//! nothing but the codec-homing feature already implemented in this
//! crate (§4.2 encoder-homing-frame, §4.3 encoder homing, §4.4
//! decoder-homing-frame).
//!
//! This module implements **both** synchronization steps of §6.3.3.3
//! that are fully specified textually and depend only on features this
//! crate owns:
//!
//! * **bit synchronization** — the 13-trial homing-frame sweep
//!   ([`find_bit_sync`] / [`run_bit_sync_trial`]); and
//! * **frame synchronization** — the 160-position special-frame sweep
//!   ([`FrameSyncTable`] / [`find_frame_sync`]), which recovers the
//!   0..159 sample retardation of the encoder's 20 ms window once bit
//!   synchronization is known.
//!
//! The actual `BITSYNC.INP` / `SEQSYNC.INP` / `SYNCxxx.COD` reference
//! *binary* sequences live in the ETSI conformance archive (not staged
//! under `docs/audio/gsm/`), so this module does not ship those exact
//! bytes. Instead it implements the §6.3.3.3 *procedure* that built
//! them and the *invariant* the spec states they satisfy (160 distinct
//! output frames), parameterised on any caller-supplied "special
//! synchronization frame". The §6.3.3.4 reference-file formats and
//! sizes are recorded in [`SyncFormats`].
//!
//! ## Frame synchronization (§6.3.3.3)
//!
//! > *"Once bit synchronization is found, frame synchronization can be
//! > found by inputting one special frame that delivers 160 different
//! > output frames, depending on the 160 different positions that this
//! > frame can possibly have with respect to the encoder framing."*
//!
//! > *"This special synchronization frame was found by taking one input
//! > frame and shifting it through the positions 0 to 159. The
//! > corresponding 160 encoded speech frames were calculated and it was
//! > verified that all 160 output frames were different. When shifting
//! > the input synchronization frame, the samples at the beginning were
//! > set to 0x0008 hex, which corresponds to the samples of the
//! > encoder-homing-frame."* (§6.3.3.3)
//!
//! > *"The corresponding 160 different output frames are given in
//! > SYNC000.COD through SYNC159.COD. The three digit number in the
//! > filename indicates the number of samples by which the input is
//! > retarded with respect to the encoder framing. By a corresponding
//! > shift in the opposite direction, alignment with the encoder
//! > framing can be attained."* (§6.3.3.3)
//!
//! [`FrameSyncTable::build`] reproduces that construction: for each
//! retardation `r ∈ 0..160` it forms the encoder's 20 ms window as the
//! special frame retarded by `r` samples — the leading `r` slots filled
//! with the homing-frame value `0x0008`, the remaining `160 - r` slots
//! taken from the head of the special frame — and encodes it from a
//! freshly homed encoder (the §6.3.3.3 procedure precedes the special
//! frame with encoder-homing-frames, leaving the encoder in its §4.5
//! home state). [`FrameSyncTable::all_distinct`] checks the spec's
//! stated property, and [`find_frame_sync`] recovers `r` from an
//! observed `SYNCxxx.COD` output frame by table match.
//!
//! ## Bit synchronization (§6.3.3.3)
//!
//! > *"The input to the speech encoder is a series of 13 bit long
//! > words (104 kbits/s, 13 bit linear PCM). When starting to test
//! > the speech encoder, no knowledge is available of bit
//! > synchronization, i.e., where the encoder expects its least
//! > significant bits, and where it expects the most significant
//! > bits."*
//!
//! The transmitted material is a continuous stream of bits; the
//! encoder reads it as a sequence of words. Because the word boundary
//! is unknown, the recovered words can be off by 0..12 bit positions.
//!
//! > *"Since there are only 13 possibilities for bit synchronization,
//! > after a maximum of 13 trials bit synchronization can be reached.
//! > In each trial three consecutive encoder-homing-frames are input
//! > to the encoder. If the decoder-homing-frame is not detected at
//! > the output, the relative bit position of the three input frames
//! > is shifted by one and another trial is performed. As soon as the
//! > decoder-homing-frame is detected at the output, bit
//! > synchronization is found, and the first step can be terminated."*
//!
//! > *"The reason why three consecutive encoder-homing-frames are
//! > needed is that frame synchronization is not known at this stage.
//! > To be sure that the encoder reads two complete homing frames,
//! > three frames have to be input. Wherever the encoder has its
//! > 20 ms segmentation, it will always read at least two complete
//! > encoder-homing-frames."* (§6.3.3.3)
//!
//! Two consecutive encoder-homing-frames produce a decoder-homing-
//! frame at the encoder output (§4.2 / §4.3 NOTE: *"Applying a
//! sequence of N encoder-homing-frames will cause at least N-1
//! decoder-homing-frames at the output of the speech encoder."*).
//! Feeding three encoder-homing-frames therefore yields at least one
//! decoder-homing-frame at the correct alignment — and at a wrong
//! bit alignment the reconstructed words are not encoder-homing-
//! frames, so no decoder-homing-frame appears. That presence/absence
//! is the bit-sync decision.

use crate::bitstream::{UnpackedFrame, FRAME_SAMPLES};
use crate::decoder::is_decoder_homing_frame;
use crate::encoder::EncoderState;

/// The number of distinct bit alignments the encoder input can have.
///
/// §6.3.3.3: *"there are only 13 possibilities for bit
/// synchronization"* — the 13 bit positions of a 13-bit linear PCM
/// word.
pub const BIT_SYNC_TRIALS: usize = 13;

/// The number of bits in one linear PCM input word (§5.2.0 / §6.3.3.3
/// "13 bit long words").
pub const PCM_WORD_BITS: usize = 13;

/// The number of consecutive encoder-homing-frames fed per bit-sync
/// trial.
///
/// §6.3.3.3: *"In each trial three consecutive encoder-homing-frames
/// are input to the encoder […] To be sure that the encoder reads two
/// complete homing frames, three frames have to be input."*
pub const HOMING_FRAMES_PER_TRIAL: usize = 3;

/// Outcome of a single bit-synchronization trial.
///
/// A trial feeds [`HOMING_FRAMES_PER_TRIAL`] encoder-homing-frames at
/// one candidate bit alignment and reports whether the encoder
/// produced the §4.4 decoder-homing-frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BitSyncTrial {
    /// The candidate alignment under test (`0..BIT_SYNC_TRIALS`):
    /// how many bit positions the recovered words are shifted from
    /// the encoder's expected alignment.
    pub shift: usize,
    /// Whether a §4.4 decoder-homing-frame appeared in the encoder
    /// output during this trial. `true` ⇒ this `shift` is the
    /// correct bit alignment.
    pub decoder_homing_detected: bool,
}

/// Run one bit-synchronization trial (§6.3.3.3) on a fresh encoder.
///
/// `frames` is the triplet of 160-sample input frames recovered at
/// the candidate `shift` (the spec's *"three consecutive encoder-
/// homing-frames […] shifted by one"*). The encoder is homed by
/// applying §4.3 encoder homing to each frame, and the trial succeeds
/// when any of the three output frames is the §4.4 decoder-homing-
/// frame.
///
/// A fresh [`EncoderState`] is used per trial: §6.3.3.3 has the
/// encoder reach its home state via the first homing frame of the
/// triplet, so each trial is self-contained.
pub fn run_bit_sync_trial(
    shift: usize,
    frames: &[[i16; FRAME_SAMPLES]; HOMING_FRAMES_PER_TRIAL],
) -> BitSyncTrial {
    let mut enc = EncoderState::new();
    let mut detected = false;
    for f in frames.iter() {
        let out = enc.encode_frame_with_homing(f);
        if is_decoder_homing_frame(&out) {
            detected = true;
        }
    }
    BitSyncTrial {
        shift,
        decoder_homing_detected: detected,
    }
}

/// Find the encoder's bit alignment by sweeping the
/// [`BIT_SYNC_TRIALS`] candidate shifts (§6.3.3.3 bit
/// synchronization).
///
/// `triplet_for_shift(shift)` must return the three input frames the
/// encoder would receive at the given candidate alignment — for the
/// reference procedure these are the `BITSYNC.INP` triplets, but any
/// caller-supplied bit-shifting front-end works. The function returns
/// `Some(shift)` for the **first** trial in `0..BIT_SYNC_TRIALS` that
/// produces the §4.4 decoder-homing-frame, or `None` if no alignment
/// does (which, per §6.3.3.3, should not happen for a conforming
/// encoder fed genuine homing-frame triplets).
pub fn find_bit_sync<F>(mut triplet_for_shift: F) -> Option<usize>
where
    F: FnMut(usize) -> [[i16; FRAME_SAMPLES]; HOMING_FRAMES_PER_TRIAL],
{
    (0..BIT_SYNC_TRIALS).find(|&shift| {
        let frames = triplet_for_shift(shift);
        run_bit_sync_trial(shift, &frames).decoder_homing_detected
    })
}

/// The homing-frame sample value `0x0008` (§4.2 / §6.3.3.3) used to
/// fill the "samples at the beginning" of a retarded special
/// synchronization frame.
///
/// §6.3.3.3: *"When shifting the input synchronization frame, the
/// samples at the beginning were set to 0x0008 hex, which corresponds
/// to the samples of the encoder-homing-frame."*
pub const SYNC_LEADING_FILL: i16 = 0x0008;

/// The 160 reference encoder output frames of the §6.3.3.3 frame-
/// synchronization sweep (`SYNC000.COD..SYNC159.COD`).
///
/// Entry `r` (`0..`[`SyncFormats::FRAME_SYNC_POSITIONS`]) is the
/// encoder output when the special synchronization frame is retarded by
/// `r` samples relative to the encoder's 20 ms framing — exactly what
/// the spec's `SYNCr.COD` file holds. The table is built by encoding,
/// from a freshly homed encoder, the retarded window
/// [`retard_special_frame`] produces for each `r`.
///
/// Holding the table lets [`find_frame_sync`] map a captured
/// `SYNCxxx.COD` output frame back to its retardation `r`; per §6.3.3.3
/// alignment is then attained "by a corresponding shift in the opposite
/// direction".
#[derive(Debug, Clone)]
pub struct FrameSyncTable {
    /// `outputs[r]` is the encoder output for retardation `r`.
    outputs: Vec<UnpackedFrame>,
}

/// Form the encoder's 20 ms input window when the `special` frame is
/// retarded by `r` samples (§6.3.3.3).
///
/// A retardation of `r` means the encoder's window starts `r` samples
/// *before* the special frame begins; those `r` leading window slots
/// are filled with the homing-frame value [`SYNC_LEADING_FILL`]
/// (§6.3.3.3 "the samples at the beginning were set to 0x0008 hex"),
/// and the remaining `160 - r` slots carry the first `160 - r` samples
/// of `special`. `r == 0` is the special frame itself; `r == 160`
/// (not a valid sweep index) would be an all-fill homing frame.
pub fn retard_special_frame(special: &[i16; FRAME_SAMPLES], r: usize) -> [i16; FRAME_SAMPLES] {
    let r = r.min(FRAME_SAMPLES);
    let mut win = [SYNC_LEADING_FILL; FRAME_SAMPLES];
    // The non-fill tail copies the head of the special frame.
    win[r..].copy_from_slice(&special[..FRAME_SAMPLES - r]);
    win
}

impl FrameSyncTable {
    /// Build the 160-entry reference table for a given special
    /// synchronization frame, reproducing the §6.3.3.3 construction.
    ///
    /// Each entry is encoded from a **fresh** [`EncoderState`] (the
    /// §4.5 home state), matching the procedure's "reset by one
    /// encoder-homing-frame" precondition: the homing frames that
    /// precede the special frame in `SEQSYNC.INP` leave the encoder
    /// homed, so the special frame is encoded against home-state
    /// history.
    pub fn build(special: &[i16; FRAME_SAMPLES]) -> Self {
        let mut outputs = Vec::with_capacity(SyncFormats::FRAME_SYNC_POSITIONS);
        for r in 0..SyncFormats::FRAME_SYNC_POSITIONS {
            let win = retard_special_frame(special, r);
            let mut enc = EncoderState::new();
            outputs.push(enc.encode_frame(&win));
        }
        Self { outputs }
    }

    /// The reference output frame for retardation `r`
    /// (the `SYNCr.COD` content). Returns `None` for `r` outside
    /// `0..`[`SyncFormats::FRAME_SYNC_POSITIONS`].
    pub fn output(&self, r: usize) -> Option<&UnpackedFrame> {
        self.outputs.get(r)
    }

    /// All [`SyncFormats::FRAME_SYNC_POSITIONS`] reference frames.
    pub fn outputs(&self) -> &[UnpackedFrame] {
        &self.outputs
    }

    /// Whether all 160 reference output frames are pairwise distinct —
    /// the §6.3.3.3 property that makes the special frame usable for
    /// frame synchronization ("it was verified that all 160 output
    /// frames were different").
    ///
    /// A special frame that fails this is unsuitable: two retardations
    /// would produce the same `SYNCxxx.COD`, leaving [`find_frame_sync`]
    /// unable to disambiguate them.
    pub fn all_distinct(&self) -> bool {
        for (i, a) in self.outputs.iter().enumerate() {
            for b in &self.outputs[i + 1..] {
                if a == b {
                    return false;
                }
            }
        }
        true
    }

    /// Recover the retardation `r` of an observed encoder output
    /// `observed` (a captured `SYNCxxx.COD`) by table match.
    ///
    /// Returns the index `r` of the matching reference frame, or `None`
    /// if `observed` matches no entry (which, for a conforming encoder
    /// fed the genuine special frame, should not happen). When the
    /// table [`all_distinct`](Self::all_distinct), the match is unique.
    pub fn match_output(&self, observed: &UnpackedFrame) -> Option<usize> {
        self.outputs.iter().position(|o| o == observed)
    }
}

/// Recover the 20 ms frame retardation of a black-box encoder
/// (§6.3.3.3 frame synchronization).
///
/// `special` is the §6.3.3.3 special synchronization frame; `observed`
/// is the `SYNCxxx.COD` output frame the encoder under test produced
/// for the special frame at its (unknown) framing offset. The function
/// builds the reference [`FrameSyncTable`] from `special` and returns
/// the retardation `r` whose reference output matches `observed`.
///
/// Per §6.3.3.3, alignment with the encoder framing is then attained
/// "by a corresponding shift in the opposite direction" — i.e. advance
/// the encoder input by `r` samples.
pub fn find_frame_sync(special: &[i16; FRAME_SAMPLES], observed: &UnpackedFrame) -> Option<usize> {
    FrameSyncTable::build(special).match_output(observed)
}

/// The §6.3.3.4 reference-sequence file formats and sizes.
///
/// These describe the conformance archive's synchronization
/// sequences. The binary files themselves are **not** staged under
/// `docs/audio/gsm/`; the constants here record the spec's stated
/// layout so a future round that stages the archive can validate
/// against it.
#[derive(Debug, Clone, Copy)]
pub struct SyncFormats;

impl SyncFormats {
    /// `BITSYNC.INP` — *"13 frame triplets […] 13 bit left justified
    /// with the three least significant bits set to zero"* (§6.3.3.4).
    ///
    /// `SIZE = 13 * 3 * 160 * 2 bytes = 12 480 bytes`.
    pub const BITSYNC_INP_BYTES: usize =
        BIT_SYNC_TRIALS * HOMING_FRAMES_PER_TRIAL * FRAME_SAMPLES * 2;

    /// `SEQSYNC.INP` — *"3 encoder reset frames and the special
    /// synchronization frame"* (§6.3.3.4).
    ///
    /// `SIZE = 4 * 160 * 2 bytes = 1 280 bytes`.
    pub const SEQSYNC_INP_BYTES: usize = 4 * FRAME_SAMPLES * 2;

    /// `SYNCxxx.COD` — *"1 encoder output frame each […] 16 bit words
    /// right justified"* (§6.3.3.4).
    ///
    /// `SIZE = 76 * 2 bytes = 152 bytes` (the 76 §1.7 Table 1.1
    /// parameters, each in its own 16-bit word).
    pub const SYNC_COD_BYTES: usize = 76 * 2;

    /// The number of distinct frame-synchronization output files
    /// (`SYNC000.COD..SYNC159.COD`) — one per possible sample offset
    /// of the special synchronization frame relative to the encoder
    /// framing (§6.3.3.3 *"160 different output frames, depending on
    /// the 160 different positions"*).
    pub const FRAME_SYNC_POSITIONS: usize = FRAME_SAMPLES;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::decoder::encoder_homing_frame_pcm;

    /// A triplet of three encoder-homing-frames at the correct
    /// alignment (`shift == 0`), and an obviously non-homing triplet
    /// for every other shift. This mirrors the §6.3.3.3 mechanism
    /// without needing the unstaged `BITSYNC.INP`: only the correct
    /// alignment reconstructs genuine encoder-homing-frames.
    fn synthetic_triplet(shift: usize) -> [[i16; FRAME_SAMPLES]; HOMING_FRAMES_PER_TRIAL] {
        if shift == 0 {
            let h = encoder_homing_frame_pcm();
            [h, h, h]
        } else {
            // A wrong bit alignment reconstructs garbage words — model
            // it as a ramp that is plainly not the homing frame.
            let garble: [i16; FRAME_SAMPLES] =
                core::array::from_fn(|i| ((i as i16).wrapping_mul(8)) & 0x7ff8);
            [garble, garble, garble]
        }
    }

    #[test]
    fn three_homing_frames_yield_decoder_homing_output() {
        // §6.3.3.3: feeding three encoder-homing-frames must produce
        // at least one decoder-homing-frame at the output (the second
        // and third each trigger one, per §4.3 NOTE "N in ⇒ N-1 out").
        let h = encoder_homing_frame_pcm();
        let trial = run_bit_sync_trial(0, &[h, h, h]);
        assert!(trial.decoder_homing_detected);
        assert_eq!(trial.shift, 0);
    }

    #[test]
    fn two_homing_frames_are_enough_for_one_decoder_homing_output() {
        // §4.3 NOTE: N encoder-homing-frames ⇒ at least N-1
        // decoder-homing-frames. From a fresh (home) encoder the
        // *first* homing frame already encodes to the decoder-homing-
        // frame (§4.3 Step 1 construction sentence), so even the first
        // of the triplet is detected here. The salient property the
        // procedure relies on is that *some* output frame is the
        // decoder-homing-frame.
        let h = encoder_homing_frame_pcm();
        let mut enc = EncoderState::new();
        let out1 = enc.encode_frame_with_homing(&h);
        assert!(
            is_decoder_homing_frame(&out1),
            "from the home state the encoder-homing-frame encodes to the decoder-homing-frame"
        );
    }

    #[test]
    fn wrong_alignment_does_not_detect_homing() {
        // A non-homing triplet must not be mistaken for bit sync.
        let garble: [i16; FRAME_SAMPLES] =
            core::array::from_fn(|i| ((i as i16).wrapping_mul(8)) & 0x7ff8);
        let trial = run_bit_sync_trial(5, &[garble, garble, garble]);
        assert!(!trial.decoder_homing_detected);
        assert_eq!(trial.shift, 5);
    }

    #[test]
    fn find_bit_sync_returns_correct_shift() {
        // Only shift 0 carries genuine homing frames ⇒ that is the
        // first (and only) trial that detects the decoder-homing-frame.
        assert_eq!(find_bit_sync(synthetic_triplet), Some(0));
    }

    #[test]
    fn find_bit_sync_locates_a_nonzero_shift() {
        // Model the correct alignment landing at shift 7: every other
        // shift garbles, shift 7 reconstructs homing frames. The sweep
        // must return the first detecting trial, i.e. 7.
        let detector = |shift: usize| -> [[i16; FRAME_SAMPLES]; HOMING_FRAMES_PER_TRIAL] {
            if shift == 7 {
                let h = encoder_homing_frame_pcm();
                [h, h, h]
            } else {
                let g: [i16; FRAME_SAMPLES] =
                    core::array::from_fn(|i| ((i as i16).wrapping_mul(8)) & 0x7ff8);
                [g, g, g]
            }
        };
        assert_eq!(find_bit_sync(detector), Some(7));
    }

    #[test]
    fn find_bit_sync_none_when_no_alignment_homes() {
        // No alignment ever reconstructs a homing frame ⇒ no sync.
        let g: [i16; FRAME_SAMPLES] =
            core::array::from_fn(|i| ((i as i16).wrapping_mul(8)) & 0x7ff8);
        assert_eq!(find_bit_sync(|_| [g, g, g]), None);
    }

    #[test]
    fn bit_sync_sweeps_at_most_thirteen_trials() {
        // §6.3.3.3: "after a maximum of 13 trials bit synchronization
        // can be reached." The sweep range is exactly the 13 bit
        // positions of a 13-bit word.
        assert_eq!(BIT_SYNC_TRIALS, PCM_WORD_BITS);
        let mut seen = 0usize;
        let _ = find_bit_sync(|shift| {
            seen = seen.max(shift + 1);
            let g: [i16; FRAME_SAMPLES] =
                core::array::from_fn(|i| ((i as i16).wrapping_mul(8)) & 0x7ff8);
            [g, g, g]
        });
        assert_eq!(seen, BIT_SYNC_TRIALS);
    }

    // ─── §6.3.3.3 frame synchronization ───

    /// A candidate "special synchronization frame": a rich, position-
    /// dependent waveform in the 13-bit linear PCM range (low three
    /// bits cleared per the §6.3.3.4 input format "13 bit left
    /// justified with the three least significant bits set to zero").
    /// The spec's own special frame is empirically found; any frame
    /// whose 160 retarded encodings come out pairwise distinct serves
    /// the same role, which `frame_sync_table_outputs_are_distinct`
    /// asserts for this one.
    fn candidate_special_frame() -> [i16; FRAME_SAMPLES] {
        core::array::from_fn(|i| {
            let n = i as i32;
            // A blend of two incommensurate ramps + a quadratic term so
            // adjacent positions differ markedly through the analysis
            // filters; kept inside ±4096 then aligned to the 13-bit grid.
            let v = (n * 53 + (n * n) / 7 + (n % 11) * 211) % 4096 - 2048;
            ((v as i16) << 3) & 0x7ff8 | if v < 0 { 0x8000u16 as i16 } else { 0 }
        })
    }

    #[test]
    fn retard_fills_leading_samples_with_homing_value() {
        // §6.3.3.3: retardation r fills the first r window slots with
        // 0x0008 and slides the special frame's head into the rest.
        let special: [i16; FRAME_SAMPLES] = core::array::from_fn(|i| ((i as i16) << 3) & 0x7ff8);
        let win = retard_special_frame(&special, 5);
        for w in win.iter().take(5) {
            assert_eq!(*w, SYNC_LEADING_FILL);
        }
        // Slot 5 onward carries special[0], special[1], …
        for (k, w) in win.iter().enumerate().skip(5) {
            assert_eq!(*w, special[k - 5]);
        }
    }

    #[test]
    fn retard_zero_is_the_special_frame_itself() {
        let special = candidate_special_frame();
        assert_eq!(retard_special_frame(&special, 0), special);
    }

    #[test]
    fn retard_full_is_all_homing_fill() {
        // r == 160 ⇒ every slot is the leading fill (an all-homing
        // frame); clamped to FRAME_SAMPLES so larger r behaves the same.
        let special = candidate_special_frame();
        assert_eq!(
            retard_special_frame(&special, FRAME_SAMPLES),
            [SYNC_LEADING_FILL; FRAME_SAMPLES]
        );
        assert_eq!(
            retard_special_frame(&special, FRAME_SAMPLES + 9),
            [SYNC_LEADING_FILL; FRAME_SAMPLES]
        );
    }

    #[test]
    fn frame_sync_table_has_160_entries() {
        let table = FrameSyncTable::build(&candidate_special_frame());
        assert_eq!(table.outputs().len(), SyncFormats::FRAME_SYNC_POSITIONS);
        assert!(table.output(0).is_some());
        assert!(table.output(159).is_some());
        assert!(table.output(160).is_none());
    }

    #[test]
    fn frame_sync_table_outputs_are_distinct() {
        // §6.3.3.3: the special frame "delivers 160 different output
        // frames" — "it was verified that all 160 output frames were
        // different." Our candidate special frame must satisfy that
        // invariant, otherwise frame sync would be ambiguous.
        let table = FrameSyncTable::build(&candidate_special_frame());
        assert!(
            table.all_distinct(),
            "candidate special frame must yield 160 distinct encoder outputs"
        );
    }

    #[test]
    fn find_frame_sync_recovers_every_retardation() {
        // Round-trip the §6.3.3.3 recovery: for each retardation r,
        // the encoder output of the r-retarded special frame must map
        // back to exactly r. This is the heart of the frame-sync step.
        let special = candidate_special_frame();
        let table = FrameSyncTable::build(&special);
        for r in 0..SyncFormats::FRAME_SYNC_POSITIONS {
            let observed = table.output(r).unwrap();
            assert_eq!(
                table.match_output(observed),
                Some(r),
                "retardation {r} must be recovered uniquely"
            );
            // The standalone front-end re-encodes from scratch and must
            // agree (it rebuilds the same table internally).
            let win = retard_special_frame(&special, r);
            let mut enc = EncoderState::new();
            let out = enc.encode_frame(&win);
            assert_eq!(find_frame_sync(&special, &out), Some(r));
        }
    }

    #[test]
    fn find_frame_sync_none_for_foreign_output() {
        // An output frame that is not in the reference table (here the
        // §4.4 decoder-homing-frame, which is not a SYNCxxx.COD entry
        // for this special frame) yields no match.
        let special = candidate_special_frame();
        let foreign = crate::decoder::decoder_homing_frame();
        assert_eq!(find_frame_sync(&special, &foreign), None);
    }

    #[test]
    fn sync_format_sizes_match_spec() {
        // §6.3.3.4 stated sizes.
        assert_eq!(SyncFormats::BITSYNC_INP_BYTES, 12_480);
        assert_eq!(SyncFormats::SEQSYNC_INP_BYTES, 1_280);
        assert_eq!(SyncFormats::SYNC_COD_BYTES, 152);
        assert_eq!(SyncFormats::FRAME_SYNC_POSITIONS, 160);
        assert_eq!(HOMING_FRAMES_PER_TRIAL, 3);
    }
}
