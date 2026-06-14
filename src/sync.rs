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
//! This module implements **bit synchronization**: the part of
//! §6.3.3.3 that is fully specified textually and depends only on
//! features this crate owns. The actual `BITSYNC.INP` /
//! `SEQSYNC.INP` / `SYNCxxx.COD` reference sequences live in the
//! ETSI conformance archive (not staged under `docs/audio/gsm/`), so
//! the frame-synchronization sweep that recovers the 0..159 sample
//! offset against `SYNC000.COD..SYNC159.COD` is deferred to a
//! follow-up round; the formats are documented in [`SyncFormats`].
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

use crate::bitstream::FRAME_SAMPLES;
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
