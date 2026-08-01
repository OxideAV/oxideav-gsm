//! GSM 06.31 Discontinuous Transmission (DTX) for full-rate speech —
//! ETSI EN 300 964 — plus the GSM 06.12 §5.2 SID-frame encoding and
//! the SID code-word field (GSM 06.12 clause 4 / GSM 05.03 table 2).
//!
//! ## The SID frame (GSM 06.12 §5.2)
//!
//! A SID (SIlence Descriptor) frame carries the comfort-noise
//! parameters inside an ordinary 260-bit traffic frame:
//!
//! * the eight LAR codewords hold the §5.1 **mean** LARs, encoded by
//!   the GSM 06.10 §5.2.7 quantiser;
//! * the four block amplitudes hold the single §5.1 **mean** `xmax`,
//!   encoded by §5.2.15 and *"repeated four times inside the frame"*;
//! * the **SID code word** — 95 bits, all zero — fills the SID field:
//!   the positions of those 95 encoded RPE-pulse `Xmc` bits that are
//!   in error-protection class I (GSM 05.03 table 2);
//! * *"The remaining bits in the SID frame are set to zero"* — so the
//!   LTP lags/gains, grid positions, and the class-II `Xmc` bits are
//!   zero too. In parameter terms a SID frame is exactly: mean LARs +
//!   four mean-`xmax` codewords + every other parameter zero
//!   ([`make_sid_frame`]).
//!
//! ## The SID field (GSM 05.03 table 2)
//!
//! The 95 class-I `Xmc` bits, from the table's importance listing:
//!
//! * bit 2 (the MSB of each 3-bit `Xmc` codeword) of **all 52** RPE
//!   pulses (parameters 13..25, 30..42, 47..59, 64..76): 52 bits;
//! * bit 1 of the 39 pulses of sub-frames 1..3 (parameters 13..25,
//!   30..42, 47..59): 39 bits;
//! * bit 1 of the first four pulses of sub-frame 4 (parameters
//!   64..67): 4 bits.
//!
//! 52 + 39 + 4 = 95. The receive-side SID detector compares exactly
//! these bit positions against the all-zero code word
//! ([`sid_field_deviation`]), yielding the ternary SID flag of
//! GSM 06.31 §6.1.1 ([`SidFlag`]).
//!
//! ## The TX DTX handler (GSM 06.31 §5.1.1)
//!
//! [`TxDtxHandler`] schedules speech vs SID frames from the VAD flag:
//!
//! * `VAD=1` — the speech frame passes with `SP=1`;
//! * end of a speech burst — it takes `N+1` (= 5, with the GSM 06.12
//!   `N = 4` averaging window) consecutive `VAD=0` frames to make a
//!   new updated SID frame available. Normally the first `N` frames
//!   after the burst pass as speech (`SP=1`, the DTX *hangover*), and
//!   the first new SID frame goes out as frame `N+1` (`SP=0`);
//! * **short-burst rule** — if fewer than 24 frames elapsed since the
//!   last SID was computed and passed, the hangover is skipped: the
//!   *last* SID frame is repeatedly passed (`SP=0`) until the new
//!   updated SID is available;
//! * once the first SID has gone out, updated SID frames are computed
//!   and passed **every** frame (`SP=0`) as long as `VAD=0`;
//! * after a reset, *"all frames before the reset … have to be treated
//!   as if there would have been speech frames for an infinitely long
//!   time"* — the first `N` frames are always `SP=1` even if `VAD=0`.
//!
//! The whole transmit side is validated bit-exactly by
//! `tests/conformance_vad.rs`: the EN 300 965 corpus `*.COD` files
//! carry the SP flag in bit 15 of LAR(2) and hold the TX DTX
//! handler's output — real SID frames included — for all 20 cases.
//!
//! ## Receive side (GSM 06.31 §6.1)
//!
//! [`classify_rx_frame`] implements the table-1 classification from
//! the `(BFI, SID, TAF)` flags, mapping each traffic frame to the
//! action the RX DTX handler takes ([`RxClassification`]); it is the
//! bridge to the crate's existing [`crate::DtxReceiver`].

use crate::bitstream::{SubFrame, UnpackedFrame, PULSES};
use crate::comfort_noise::{NoiseEvaluator, NoiseFrameParameters, SidParameters};

/// Number of bits in the SID code word (GSM 06.12 §5.2: *"The SID
/// code word consists of 95 bits which are all zero."*).
pub const SID_CODE_WORD_BITS: u32 = 95;

/// GSM 06.12 §5.1 / GSM 06.31 §5.1.1 — the SID averaging window
/// length `N` (frames marked VAD=0).
pub const SID_AVERAGING_FRAMES: usize = crate::comfort_noise::NOISE_EVAL_FRAMES;

/// GSM 06.31 §5.1.1 — the elapsed-frame threshold of the short-burst
/// rule: *"if … less than 24 frames have elapsed since the last SID
/// frame was computed and passed to the RSS, then this last SID frame
/// shall repeatedly be passed"*.
pub const SHORT_BURST_ELAPSED_LIMIT: u32 = 24;

/// Count the deviation of a frame's SID field from the all-zero SID
/// code word: the number of ones among the 95 class-I `Xmc` bits
/// (GSM 06.12 clause 4 + GSM 05.03 table 2; see the module docs for
/// the position list).
pub fn sid_field_deviation(frame: &UnpackedFrame) -> u32 {
    let mut n = 0u32;
    for (s, sub) in frame.sub.iter().enumerate() {
        for (p, &xmc) in sub.x_mc.iter().enumerate() {
            // Bit 2 (MSB) — class I in every sub-frame.
            n += ((xmc >> 2) & 1) as u32;
            // Bit 1 — class I in sub-frames 1..3 and for the first
            // four pulses of sub-frame 4.
            if s < 3 || p < 4 {
                n += ((xmc >> 1) & 1) as u32;
            }
        }
    }
    n
}

/// The ternary SID flag of GSM 06.31 §6.1.1, computed by the SID
/// frame detector from the bit deviation `n` of the SID field:
///
/// * `SID=2` when `n < 2` — a valid SID frame (when `BFI=0`);
/// * `SID=1` when `2 <= n < 16`;
/// * `SID=0` when `n >= 16` — not a SID frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SidFlag {
    /// `n >= 16` — the frame is not a SID frame.
    Sid0,
    /// `2 <= n < 16`.
    Sid1,
    /// `n < 2`.
    Sid2,
}

/// Run the GSM 06.31 §6.1.1 SID frame detector on a traffic frame.
pub fn sid_flag(frame: &UnpackedFrame) -> SidFlag {
    match sid_field_deviation(frame) {
        0..=1 => SidFlag::Sid2,
        2..=15 => SidFlag::Sid1,
        _ => SidFlag::Sid0,
    }
}

/// Build the GSM 06.12 §5.2 SID frame for a set of evaluated noise
/// parameters: mean LAR codewords, the mean block amplitude repeated
/// in all four sub-frames, and **every** other parameter zero (the
/// all-zero SID code word in the class-I `Xmc` positions plus the
/// "remaining bits … set to zero" rule).
pub fn make_sid_frame(params: &SidParameters) -> UnpackedFrame {
    let mut f = UnpackedFrame {
        lar_c: params.lar_cr,
        ..UnpackedFrame::default()
    };
    for (sub, &xc) in f.sub.iter_mut().zip(params.xmax_cr.iter()) {
        *sub = SubFrame {
            n_c: 0,
            b_c: 0,
            m_c: 0,
            xmax_c: xc,
            x_mc: [0; PULSES],
        };
    }
    f
}

/// Recover the [`SidParameters`] carried by a (valid) SID frame — the
/// receive-side inverse of [`make_sid_frame`], feeding the GSM 06.12
/// §6.1 comfort-noise generator.
pub fn sid_frame_parameters(frame: &UnpackedFrame) -> SidParameters {
    SidParameters::new(frame.lar_c, core::array::from_fn(|s| frame.sub[s].xmax_c))
}

/// One TX DTX handler decision (GSM 06.31 §5.1.1): the frame actually
/// passed to the radio subsystem, with its SP flag.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TxFrame {
    /// `SP=1` — the speech-encoder output frame passes unchanged
    /// (genuine speech, or the DTX hangover period).
    Speech,
    /// `SP=0` — a SID frame replaces the encoder output.
    Sid(UnpackedFrame),
}

impl TxFrame {
    /// The GSM 06.31 §5.1 SP flag: `true` for a speech frame.
    pub fn sp(&self) -> bool {
        matches!(self, TxFrame::Speech)
    }
}

/// GSM 06.31 §5.1.1 TX DTX handler state.
#[derive(Debug, Clone)]
pub struct TxDtxHandler {
    evaluator: NoiseEvaluator,
    /// Consecutive VAD=0 frames in the current pause (0 during
    /// speech).
    vad_zero_run: u32,
    /// Frames elapsed since the last SID frame was computed and
    /// passed (saturating; starts at the short-burst limit so the
    /// after-reset behaviour is the §5.1.1 "speech frames for an
    /// infinitely long time" hangover path).
    n_elapsed: u32,
    /// The last SID parameters computed and passed.
    last_sid: Option<SidParameters>,
    /// Whether the current pause runs in short-burst repeat mode
    /// (latched at the end of the speech burst).
    repeat_mode: bool,
}

impl Default for TxDtxHandler {
    fn default() -> Self {
        Self::new()
    }
}

impl TxDtxHandler {
    /// Fresh handler in the after-reset state (§5.1.1: the first `N`
    /// frames are always `SP=1`, even if `VAD=0`).
    pub fn new() -> Self {
        Self {
            evaluator: NoiseEvaluator::new(),
            vad_zero_run: 0,
            n_elapsed: SHORT_BURST_ELAPSED_LIMIT,
            last_sid: None,
            repeat_mode: false,
        }
    }

    /// Reset to the after-reset state (the DTX handler's part of the
    /// GSM 06.10 §4.3 encoder homing, which lists DTX among the
    /// sub-modules to home).
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Process one frame: the VAD flag plus the frame's unquantised
    /// GSM 06.12 §5.1 noise parameters, returning the §5.1.1
    /// scheduling decision.
    pub fn process_frame(&mut self, vad: bool, params: &NoiseFrameParameters) -> TxFrame {
        if vad {
            // Speech: the window of consecutive VAD=0 frames breaks.
            self.vad_zero_run = 0;
            self.evaluator.reset();
            self.n_elapsed = self.n_elapsed.saturating_add(1);
            return TxFrame::Speech;
        }

        self.vad_zero_run += 1;

        let out = if self.vad_zero_run > SID_AVERAGING_FRAMES as u32 {
            // A new updated SID frame is available (N+1 consecutive
            // VAD=0 frames). §5.1.1/§5.1 window semantics, fixed
            // bit-exactly by the EN 300 965 corpus `*.COD` streams:
            // the SID passed *instead of* frame `j` averages the four
            // VAD=0 frames **preceding** `j` — the frame being
            // replaced is not part of its own average (it enters the
            // window for the *next* frame's update). This is also
            // why §5.1.1 needs N+1 frames to make a SID available.
            let sid = self.evaluator.evaluate().expect("window holds >= N frames");
            self.last_sid = Some(sid);
            self.n_elapsed = 0;
            TxFrame::Sid(make_sid_frame(&sid))
        } else {
            // Pause frames 1..=N. These count towards the elapsed
            // total *before* the short-burst decision is latched: the
            // §5.1.1 "less than 24 frames have elapsed since the last
            // SID frame was computed and passed" span runs from the
            // SID frame to the current (first pause) frame inclusive
            // — pinned by the GOOD_SP corpus case, where a pause
            // beginning exactly 24 frames after the last SID takes
            // the hangover path.
            self.n_elapsed = self.n_elapsed.saturating_add(1);
            if self.vad_zero_run == 1 {
                // End of a speech burst: latch the §5.1.1 short-burst
                // decision on the elapsed count.
                self.repeat_mode = self.n_elapsed < SHORT_BURST_ELAPSED_LIMIT;
            }
            if let (true, Some(sid)) = (self.repeat_mode, &self.last_sid) {
                // §5.1.1 short-burst rule: repeat the last SID frame
                // until the new updated SID is available.
                TxFrame::Sid(make_sid_frame(sid))
            } else {
                // Normal hangover: pass the speech-encoder output,
                // SP=1.
                TxFrame::Speech
            }
        };

        // The current frame's parameters join the window only after
        // this frame's decision (see above).
        self.evaluator.push_frame(*params);
        out
    }
}

/// Receive-side classification of one traffic frame (GSM 06.31 §6.1
/// table 1 + §3.2 definitions), from the radio-subsystem flags
/// `(BFI, SID, TAF)`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RxClassification {
    /// Good traffic frame that is not an accepted SID frame: pass it
    /// to the speech decoder.
    GoodSpeechFrame,
    /// Good traffic frame whose SID field deviates in less than 2 bit
    /// positions: start/update comfort noise from its parameters.
    ValidSidFrame,
    /// Accepted SID frame that is not valid (`SID=1` with `BFI=0`, or
    /// any accepted SID with `BFI=1`): §6.1.2 — substitute the last
    /// valid SID and apply the valid-SID procedure (comfort noise
    /// continues, no parameter update).
    InvalidSidFrame,
    /// Bad frame while passing speech: apply the GSM 06.11
    /// substitution-and-muting procedure.
    LostSpeechFrame,
    /// Bad frame while generating comfort noise with a SID frame
    /// expected (`TAF=1`): apply GSM 06.11 (comfort-noise leg).
    LostSidFrame,
    /// Bad frame while generating comfort noise, no SID expected:
    /// §6.1.2 — ignored (comfort noise continues).
    IgnoredUnusableFrame,
}

/// Classify one received traffic frame per GSM 06.31 §6.1 table 1.
///
/// `bfi` is the Bad Frame Indication, `taf` the Time Alignment Flag,
/// and `generating_comfort_noise` says whether the RX DTX handler is
/// currently in its comfort-noise state (which decides how unusable
/// frames are treated, §3.2's *lost SID* vs *lost speech* vs the
/// §6.1.2 ignore rule). The SID flag is computed from the frame's SID
/// field.
pub fn classify_rx_frame(
    frame: &UnpackedFrame,
    bfi: bool,
    taf: bool,
    generating_comfort_noise: bool,
) -> RxClassification {
    let sid = sid_flag(frame);
    match (bfi, sid) {
        (false, SidFlag::Sid2) => RxClassification::ValidSidFrame,
        (false, SidFlag::Sid1) => RxClassification::InvalidSidFrame,
        (false, SidFlag::Sid0) => RxClassification::GoodSpeechFrame,
        (true, SidFlag::Sid1 | SidFlag::Sid2) => RxClassification::InvalidSidFrame,
        (true, SidFlag::Sid0) => {
            // Unusable frame.
            if generating_comfort_noise {
                if taf {
                    RxClassification::LostSidFrame
                } else {
                    RxClassification::IgnoredUnusableFrame
                }
            } else {
                RxClassification::LostSpeechFrame
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bitstream::SUBFRAMES;
    use crate::decoder::decoder_homing_frame;

    fn quiet_params(seed: i16) -> NoiseFrameParameters {
        NoiseFrameParameters::new(
            [0, seed, -seed, 3, -3, 2, -2, 1, -1],
            [40 + seed, 41, 42, 43],
        )
    }

    /// GSM 06.12 §5.2 — a SID frame's non-LAR/non-xmax parameters are
    /// all zero, and its SID field deviation is zero.
    #[test]
    fn sid_frame_shape_and_zero_deviation() {
        let sid = SidParameters::new([0, 10, -5, 3, -2, 1, 0, 2, -1], [17; SUBFRAMES]);
        let f = make_sid_frame(&sid);
        for sub in &f.sub {
            assert_eq!((sub.n_c, sub.b_c, sub.m_c), (0, 0, 0));
            assert_eq!(sub.x_mc, [0; PULSES]);
            assert_eq!(sub.xmax_c, 17);
        }
        assert_eq!(sid_field_deviation(&f), 0);
        assert_eq!(sid_flag(&f), SidFlag::Sid2);
        // Round-trip back to the parameters.
        assert_eq!(sid_frame_parameters(&f).lar_cr, sid.lar_cr);
        assert_eq!(sid_frame_parameters(&f).xmax_cr, sid.xmax_cr);
    }

    /// GSM 05.03 table 2 — the SID field holds exactly 95 bits: all
    /// 52 Xmc MSBs, bit 1 of sub-frames 1..3, and bit 1 of the first
    /// four pulses of sub-frame 4.
    #[test]
    fn sid_field_is_95_bits() {
        // Set every Xmc bit and count the field.
        let mut f = UnpackedFrame::default();
        for sub in f.sub.iter_mut() {
            sub.x_mc = [0b111; PULSES];
        }
        assert_eq!(sid_field_deviation(&f), SID_CODE_WORD_BITS);
        // Bit 0 is never part of the field.
        let mut f0 = UnpackedFrame::default();
        for sub in f0.sub.iter_mut() {
            sub.x_mc = [0b001; PULSES];
        }
        assert_eq!(sid_field_deviation(&f0), 0);
        // Bit 1 of the last nine pulses of sub-frame 4 is class II.
        let mut f1 = UnpackedFrame::default();
        f1.sub[3].x_mc = [0b010; PULSES];
        assert_eq!(sid_field_deviation(&f1), 4);
    }

    /// GSM 06.31 §6.1.1 — SID flag thresholds at n < 2 / n < 16.
    #[test]
    fn sid_flag_thresholds() {
        let sid = make_sid_frame(&SidParameters::default());
        assert_eq!(sid_flag(&sid), SidFlag::Sid2);

        // One deviating class-I bit: still a valid SID frame.
        let mut one = sid;
        one.sub[0].x_mc[0] = 0b100;
        assert_eq!(sid_field_deviation(&one), 1);
        assert_eq!(sid_flag(&one), SidFlag::Sid2);

        // Two deviations: invalid-but-accepted.
        let mut two = one;
        two.sub[1].x_mc[5] = 0b100;
        assert_eq!(sid_flag(&two), SidFlag::Sid1);

        // A real speech-like frame deviates massively.
        assert_eq!(sid_flag(&decoder_homing_frame()), SidFlag::Sid0);
        // 16 deviations: not a SID frame.
        let mut many = sid;
        for p in 0..13 {
            many.sub[0].x_mc[p] = 0b100;
        }
        for p in 0..3 {
            many.sub[1].x_mc[p] = 0b100;
        }
        assert_eq!(sid_field_deviation(&many), 16);
        assert_eq!(sid_flag(&many), SidFlag::Sid0);
    }

    /// §5.1.1 after-reset behaviour: even with VAD=0 from the first
    /// frame, the first N frames are SP=1 (hangover), then the first
    /// SID goes out as frame N+1.
    #[test]
    fn after_reset_hangover_then_first_sid() {
        let mut tx = TxDtxHandler::new();
        for n in 0..SID_AVERAGING_FRAMES {
            let out = tx.process_frame(false, &quiet_params(n as i16));
            assert_eq!(out, TxFrame::Speech, "reset-hangover frame {n} is SP=1");
        }
        let out = tx.process_frame(false, &quiet_params(9));
        assert!(matches!(out, TxFrame::Sid(_)), "frame N+1 is the first SID");
    }

    /// §5.1.1 continuous updating: once the first SID is out, every
    /// further VAD=0 frame passes an updated SID frame (SP=0).
    #[test]
    fn continuous_sid_updates_during_pause() {
        let mut tx = TxDtxHandler::new();
        for n in 0..12 {
            let out = tx.process_frame(false, &quiet_params(n));
            if n >= SID_AVERAGING_FRAMES as i16 {
                assert!(!out.sp(), "pause frame {n} must be SP=0");
            }
        }
    }

    /// §5.1.1 normal hangover after a long speech burst
    /// (n_elapsed >= 24): N SP=1 frames, then the SID.
    #[test]
    fn long_burst_gets_hangover() {
        let mut tx = TxDtxHandler::new();
        // Long speech burst.
        for _ in 0..30 {
            assert_eq!(tx.process_frame(true, &quiet_params(0)), TxFrame::Speech);
        }
        // Pause: N hangover frames…
        for n in 0..SID_AVERAGING_FRAMES {
            assert_eq!(
                tx.process_frame(false, &quiet_params(n as i16)),
                TxFrame::Speech,
                "hangover frame {n}"
            );
        }
        // …then the first new SID.
        assert!(!tx.process_frame(false, &quiet_params(7)).sp());
    }

    /// §5.1.1 short-burst rule: with < 24 frames since the last SID,
    /// the last SID frame is repeated (SP=0, same parameters) until a
    /// new updated SID is available.
    #[test]
    fn short_burst_repeats_last_sid() {
        let mut tx = TxDtxHandler::new();
        // Reach steady DTX (SID out, n_elapsed = 0).
        for n in 0..10 {
            let _ = tx.process_frame(false, &quiet_params(n));
        }
        let TxFrame::Sid(last) = tx.process_frame(false, &quiet_params(3)) else {
            panic!("expected steady-state SID");
        };
        // Short speech spike (1 frame).
        assert_eq!(tx.process_frame(true, &quiet_params(50)), TxFrame::Speech);
        // Pause frames 1..=N: the *previous* SID is repeated verbatim.
        for n in 0..SID_AVERAGING_FRAMES {
            let out = tx.process_frame(false, &quiet_params(n as i16));
            assert_eq!(
                out,
                TxFrame::Sid(last),
                "repeat frame {n} must re-send the last SID"
            );
        }
        // Frame N+1: a new updated SID (may or may not equal the old
        // parameters; it must be SP=0).
        assert!(!tx.process_frame(false, &quiet_params(2)).sp());
    }

    /// GSM 06.31 §6.1 table 1 — RX classification.
    #[test]
    fn rx_classification_table() {
        let sid = make_sid_frame(&SidParameters::default());
        let speech = decoder_homing_frame(); // massively deviating field
        let mut invalid = sid;
        invalid.sub[0].x_mc[0] = 0b100;
        invalid.sub[0].x_mc[1] = 0b100;

        use RxClassification::*;
        assert_eq!(classify_rx_frame(&sid, false, false, false), ValidSidFrame);
        assert_eq!(
            classify_rx_frame(&invalid, false, false, false),
            InvalidSidFrame
        );
        assert_eq!(
            classify_rx_frame(&speech, false, false, false),
            GoodSpeechFrame
        );
        // BFI=1 on an accepted SID: invalid SID.
        assert_eq!(classify_rx_frame(&sid, true, false, true), InvalidSidFrame);
        // Unusable frames: context decides.
        assert_eq!(
            classify_rx_frame(&speech, true, false, false),
            LostSpeechFrame
        );
        assert_eq!(classify_rx_frame(&speech, true, true, true), LostSidFrame);
        assert_eq!(
            classify_rx_frame(&speech, true, false, true),
            IgnoredUnusableFrame
        );
    }
}
