//! GSM 06.12 comfort-noise generation — receive-side §6.1.
//!
//! Source: ETSI EN 300 963 V8.0.1 (2000-11), *Full rate speech;
//! Comfort noise aspect for full rate speech traffic channels
//! (GSM 06.12)*, staged at
//! `docs/audio/gsm/etsi-en-300963-gsm-06.12-comfort-noise.pdf`.
//!
//! When Discontinuous Transmission (DTX) cuts the radio link at the
//! end of a speech burst, the background acoustic noise that was
//! travelling with the speech disappears abruptly — an effect §4
//! ("General") describes as "very annoying for the listener". To
//! mask it, the receive side synthesises *comfort noise* whose level
//! and spectrum approximate the transmit-side background noise. The
//! noise parameters are estimated on the transmit side (§5) and sent
//! in a special **SID (Silence Descriptor) frame** before the link is
//! cut, and refreshed at a low rate afterwards.
//!
//! This module implements the **§6.1 receive-side generation**, which
//! is the half of GSM 06.12 that depends only on features this crate
//! already owns (the §5.3 RPE-LTP speech decoder of GSM 06.10) plus
//! the SID-frame-supplied noise parameters. Per §6.1, comfort noise
//! is produced by driving the standard speech decoder with a frame
//! whose parameters are substituted as follows:
//!
//! | Parameter            | §6.1 value                                   |
//! |----------------------|----------------------------------------------|
//! | RPE pulses `Xmcr`    | random integers, uniform in **[1, 6]**       |
//! | Grid positions `Mcr` | random integers, uniform in **[0, 3]**       |
//! | LTP gains `bcr`      | **0**                                        |
//! | LTP lags `Ncr`       | **40, 120, 40, 120** for sub-segments 1..4   |
//! | Block ampl. `Xmaxcr` | the four values **received in the SID frame**|
//! | LAR coeffs `LARcr`   | the values **received in the SID frame**     |
//!
//! "With these parameters, the speech decoder now performs the
//! standard operations described in GSM 06.10 and synthesizes comfort
//! noise" (§6.1).
//!
//! ## What is *not* here (docs gap)
//!
//! GSM 06.12 §5 (transmit side — background-acoustic-noise evaluation
//! and SID-frame *encoding*) and the detection of a "valid SID frame"
//! on receive both depend on companion specifications that are **not**
//! staged under `docs/audio/gsm/`:
//!
//! * The §5.1 averaging of `LAR(i)` / `xmax` over `N = 4` VAD=0 frames
//!   needs the **VAD flag**, defined in GSM 06.32 (not staged).
//! * The §5.2 SID-frame layout — the 95-bit all-zero "SID code word"
//!   and the positions at which it is inserted — is defined by *"those
//!   95 bits of the encoded RPE-pulses Xmc which are in the error
//!   protection class I (see GSM 05.03, table 2)"*. GSM 05.03 (channel
//!   coding) is **not** staged, so neither SID-frame construction nor
//!   the "valid SID frame" receive-side detector can be implemented
//!   clean-room here.
//! * The DTX scheduling that decides when comfort noise is generated
//!   or updated (§6 opening, §6.1 closing) is defined in GSM 06.31
//!   (not staged).
//!
//! Accordingly this module takes the already-decoded SID parameters
//! ([`SidParameters`]) as its input and implements the §6.1 frame
//! synthesis and decoder driving — the spec-grounded part — leaving
//! SID-frame parsing/scheduling for a follow-up round once GSM 05.03
//! / 06.31 / 06.32 are staged.

use crate::bitstream::{SubFrame, UnpackedFrame, SUBFRAMES};
use crate::decoder::DecoderState;
use crate::FRAME_SAMPLES;

/// The four §6.1 LTP lag values `Ncr` for sub-segments 1..4:
/// *"The LTP lag values (Ncr) of the 4 sub-segments are set to 40,
/// 120, 40 and 120 respectively."*
pub const COMFORT_NOISE_NCR: [u8; SUBFRAMES] = [40, 120, 40, 120];

/// The §6.1 RPE-pulse range: *"random integer sequence, uniformly
/// distributed between 1 and 6"* — inclusive on both ends.
pub const RPE_PULSE_MIN: u8 = 1;
/// See [`RPE_PULSE_MIN`].
pub const RPE_PULSE_MAX: u8 = 6;

/// The §6.1 grid-position range: *"set to random integer values,
/// uniformly distributed between 0 and 3"* — inclusive on both ends.
pub const GRID_POSITION_MIN: u8 = 0;
/// See [`GRID_POSITION_MIN`].
pub const GRID_POSITION_MAX: u8 = 3;

/// The noise parameters carried in a GSM 06.12 SID frame, in the
/// already-decoded codeword form the §6.1 substitution consumes.
///
/// Per §5.2 the SID frame replaces the log-area-ratio codewords with
/// the §5.1 mean LARs and the four sub-frame block amplitudes with the
/// §5.1 mean `xmax` *repeated four times*. The receive side (§6.1)
/// uses *"the log area ratio parameters (LARcr) … received in the SID
/// frame"* and *"the 4 block amplitude values (Xmaxcr) … received in
/// the SID frame"* verbatim, so this struct holds them in the exact
/// `LARc` / `xmaxc` codeword form an [`UnpackedFrame`] carries.
///
/// `lar_cr[0]` is a sentinel zero so `lar_cr[1..=8]` mirrors the
/// spec's 1-based `LARcr(i)` indexing, matching [`UnpackedFrame::
/// lar_c`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SidParameters {
    /// `LARcr[1..=8]` — the SID-frame log-area-ratio codewords
    /// (entry 0 is a sentinel zero for 1-based indexing).
    pub lar_cr: [i16; 9],
    /// `Xmaxcr[1..=4]` — the four sub-frame block-amplitude codewords.
    /// Per §5.2 these are one §5.1 mean value repeated four times in a
    /// real SID frame, but the §6.1 receiver simply uses "the 4 block
    /// amplitude values received", so all four slots are honoured as
    /// given.
    pub xmax_cr: [u8; SUBFRAMES],
}

impl Default for SidParameters {
    /// All-zero noise parameters (silence): every LAR codeword and
    /// every block amplitude zero. Driving the §6.1 generator from
    /// this state still injects the §6.1 random RPE excitation, so the
    /// output is low-level noise rather than digital silence.
    fn default() -> Self {
        Self {
            lar_cr: [0; 9],
            xmax_cr: [0; SUBFRAMES],
        }
    }
}

impl SidParameters {
    /// Build [`SidParameters`] from the eight LAR codewords and four
    /// block amplitudes a decoded SID frame supplies. `lar_cr` is the
    /// 1-based `LARcr[1..=8]`; index 0 is ignored and forced to the
    /// sentinel zero.
    pub fn new(lar_cr: [i16; 9], xmax_cr: [u8; SUBFRAMES]) -> Self {
        let mut lar = lar_cr;
        lar[0] = 0;
        Self {
            lar_cr: lar,
            xmax_cr,
        }
    }
}

/// Deterministic linear-congruential pseudo-random source for the
/// §6.1 random RPE pulses and grid positions.
///
/// EN 300 963 §6.1 specifies only that the RPE pulses and grid
/// positions are *"locally generated random integer sequence[s]"* —
/// the generator itself is explicitly an implementation choice, not a
/// normative algorithm (no seed, table, or recurrence is given in the
/// spec). A deterministic LCG is used here so comfort-noise output is
/// reproducible and unit-testable; any uniform source over the
/// required ranges satisfies the spec equally.
///
/// The recurrence is the Numerical-Recipes 32-bit LCG
/// `x ← 1664525·x + 1013904223 (mod 2^32)`; uniform integers are drawn
/// from the high bits (which have the longest period) and mapped onto
/// the closed range `[lo, hi]` by modulo reduction over the small
/// `hi - lo + 1` span (≤ 6 values, so modulo bias is negligible and,
/// more importantly, irrelevant — the spec only asks for "uniformly
/// distributed" comfort-noise excitation, not a statistically certified
/// RNG).
#[derive(Debug, Clone)]
pub struct NoiseRng {
    state: u32,
}

impl NoiseRng {
    /// Seed the generator. Any 32-bit seed is acceptable; the spec
    /// does not constrain it. A zero seed is mapped to a fixed
    /// non-zero constant so the LCG never sticks at the all-zero
    /// fixed point of its multiplicative part.
    pub fn new(seed: u32) -> Self {
        Self {
            state: if seed == 0 { 0x9E37_79B9 } else { seed },
        }
    }

    /// Advance the LCG and return the next 32-bit state word.
    #[inline]
    fn next_u32(&mut self) -> u32 {
        self.state = self
            .state
            .wrapping_mul(1_664_525)
            .wrapping_add(1_013_904_223);
        self.state
    }

    /// Draw a uniform integer in the closed range `[lo, hi]`
    /// (`lo <= hi`). Drawn from the high bits of the LCG word.
    #[inline]
    fn uniform(&mut self, lo: u8, hi: u8) -> u8 {
        debug_assert!(lo <= hi);
        let span = (hi - lo) as u32 + 1;
        lo + ((self.next_u32() >> 16) % span) as u8
    }

    /// Draw one §6.1 RPE pulse codeword, uniform in `[1, 6]`.
    #[inline]
    pub fn rpe_pulse(&mut self) -> u8 {
        self.uniform(RPE_PULSE_MIN, RPE_PULSE_MAX)
    }

    /// Draw one §6.1 grid-position codeword, uniform in `[0, 3]`.
    #[inline]
    pub fn grid_position(&mut self) -> u8 {
        self.uniform(GRID_POSITION_MIN, GRID_POSITION_MAX)
    }
}

/// Build one §6.1 comfort-noise [`UnpackedFrame`] from the SID
/// parameters and a random source.
///
/// Implements the §6.1 parameter substitution table verbatim:
///
/// * `lar_c` ← `sid.lar_cr` (received LARs),
/// * each sub-frame `xmax_c` ← `sid.xmax_cr[j]` (received amplitudes),
/// * each sub-frame `n_c` ← [`COMFORT_NOISE_NCR`] (`40, 120, 40, 120`),
/// * each sub-frame `b_c` ← `0` (LTP gains set to zero),
/// * each sub-frame `m_c` ← random `[0, 3]` (grid positions),
/// * each pulse `x_mc[i]` ← random `[1, 6]` (RPE pulses).
///
/// The returned frame is ready to feed [`DecoderState::decode_frame`]:
/// *"the speech decoder now performs the standard operations described
/// in GSM 06.10 and synthesizes comfort noise"* (§6.1).
pub fn comfort_noise_frame(sid: &SidParameters, rng: &mut NoiseRng) -> UnpackedFrame {
    let mut f = UnpackedFrame {
        lar_c: sid.lar_cr,
        sub: [SubFrame::default(); SUBFRAMES],
    };
    f.lar_c[0] = 0; // keep the sentinel slot clean

    for (j, sf) in f.sub.iter_mut().enumerate() {
        sf.n_c = COMFORT_NOISE_NCR[j];
        sf.b_c = 0;
        sf.m_c = rng.grid_position();
        sf.xmax_c = sid.xmax_cr[j];
        for pulse in sf.x_mc.iter_mut() {
            *pulse = rng.rpe_pulse();
        }
    }

    f
}

/// Receive-side §6.1 comfort-noise generator.
///
/// Wraps a [`DecoderState`] (the GSM 06.10 §5.3 RPE-LTP speech
/// decoder) together with the current SID noise parameters and a
/// random source. Each [`Self::generate_frame`] call builds one §6.1
/// comfort-noise frame ([`comfort_noise_frame`]) and runs the standard
/// decoder over it, yielding 160 linear PCM samples of comfort noise.
///
/// The generator owns its own `DecoderState`, so the inter-frame
/// synthesis-filter / LTP / de-emphasis memory carries across comfort-
/// noise frames exactly as it would for speech — which is what makes
/// the synthesised noise continuous rather than restarting each frame.
///
/// Parameter *updating* on receipt of a fresh SID frame is
/// [`Self::update_sid`]. Per §6.1 the spec notes the parameters
/// *"should preferably be interpolated over a few frames to obtain
/// smooth transitions"*; that interpolation policy is governed by
/// GSM 06.31 (not staged), so [`Self::update_sid`] performs the plain
/// replacement the spec mandates and leaves any smoothing to a
/// follow-up round once GSM 06.31 is available.
#[derive(Debug)]
pub struct ComfortNoiseGenerator {
    decoder: DecoderState,
    sid: SidParameters,
    rng: NoiseRng,
}

impl ComfortNoiseGenerator {
    /// Build a comfort-noise generator seeded with the given SID
    /// parameters and PRNG seed. The wrapped decoder starts in its
    /// §4.6 home state.
    pub fn new(sid: SidParameters, seed: u32) -> Self {
        Self {
            decoder: DecoderState::new(),
            sid,
            rng: NoiseRng::new(seed),
        }
    }

    /// Replace the active SID noise parameters (§6.1: *"the 4 block
    /// amplitude values … and the log area ratio parameters … used are
    /// those received in the SID frame"*; updating happens *"each time
    /// a valid SID frame is received"*).
    ///
    /// Plain replacement — the §6.1 "interpolate over a few frames"
    /// smoothing is a GSM 06.31 concern and is deferred (see the type
    /// docs).
    pub fn update_sid(&mut self, sid: SidParameters) {
        self.sid = sid;
    }

    /// The SID parameters currently driving comfort-noise generation.
    pub fn sid(&self) -> &SidParameters {
        &self.sid
    }

    /// Generate one 20 ms comfort-noise frame: build the §6.1
    /// substituted [`UnpackedFrame`] and synthesise it through the
    /// standard §5.3 decoder. Returns 160 linear 13-bit PCM samples
    /// (the three LSBs cleared per §5.3.7, as for any decoded frame).
    pub fn generate_frame(&mut self) -> [i16; FRAME_SAMPLES] {
        let frame = comfort_noise_frame(&self.sid, &mut self.rng);
        self.decoder.decode_frame(&frame)
    }

    /// The most recently built §6.1 comfort-noise frame's parameters,
    /// without decoding — useful for callers that want to inspect or
    /// re-pack the substituted codewords. Advances the PRNG exactly as
    /// [`Self::generate_frame`] would.
    pub fn next_frame_parameters(&mut self) -> UnpackedFrame {
        comfort_noise_frame(&self.sid, &mut self.rng)
    }

    /// Reset the wrapped decoder to its §4.6 home state (e.g. on a
    /// codec-homing event) without disturbing the SID parameters or
    /// the PRNG.
    pub fn reset_decoder(&mut self) {
        self.decoder.reset();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// §6.1: LTP lags are fixed at 40, 120, 40, 120 across the four
    /// sub-segments, and LTP gains are all zero.
    #[test]
    fn ncr_and_bcr_match_spec() {
        let sid = SidParameters::default();
        let mut rng = NoiseRng::new(1);
        let f = comfort_noise_frame(&sid, &mut rng);
        assert_eq!(
            [f.sub[0].n_c, f.sub[1].n_c, f.sub[2].n_c, f.sub[3].n_c],
            [40, 120, 40, 120]
        );
        for sf in &f.sub {
            assert_eq!(sf.b_c, 0, "LTP gain bcr must be 0 per §6.1");
        }
    }

    /// §6.1: random RPE pulses are uniform in [1, 6] and grid
    /// positions in [0, 3] — every emitted codeword must lie in range,
    /// across many frames so the RNG range mapping is exercised
    /// thoroughly.
    #[test]
    fn random_pulses_and_grids_in_range() {
        let sid = SidParameters::default();
        let mut rng = NoiseRng::new(0xDEAD_BEEF);
        for _ in 0..1000 {
            let f = comfort_noise_frame(&sid, &mut rng);
            for sf in &f.sub {
                assert!(
                    (GRID_POSITION_MIN..=GRID_POSITION_MAX).contains(&sf.m_c),
                    "grid position {} out of [0,3]",
                    sf.m_c
                );
                for &p in &sf.x_mc {
                    assert!(
                        (RPE_PULSE_MIN..=RPE_PULSE_MAX).contains(&p),
                        "RPE pulse {p} out of [1,6]"
                    );
                }
            }
        }
    }

    /// The random draws actually cover the full closed ranges (so the
    /// generator isn't accidentally clamped to a sub-range). Over many
    /// draws every value 1..=6 and 0..=3 must appear at least once.
    #[test]
    fn random_ranges_are_fully_covered() {
        let mut rng = NoiseRng::new(12345);
        let mut seen_pulse = [false; 7]; // indices 1..=6 used
        let mut seen_grid = [false; 4]; // indices 0..=3 used
        for _ in 0..10_000 {
            seen_pulse[rng.rpe_pulse() as usize] = true;
            seen_grid[rng.grid_position() as usize] = true;
        }
        for v in RPE_PULSE_MIN..=RPE_PULSE_MAX {
            assert!(seen_pulse[v as usize], "RPE pulse value {v} never drawn");
        }
        for v in GRID_POSITION_MIN..=GRID_POSITION_MAX {
            assert!(seen_grid[v as usize], "grid value {v} never drawn");
        }
    }

    /// §6.1: the received LARs and block amplitudes are used verbatim.
    #[test]
    fn sid_parameters_passed_through() {
        let lar = [0, 9, 23, 15, 8, 7, 3, 3, 2]; // sentinel + LARcr[1..=8]
        let xmax = [11, 22, 33, 44];
        let sid = SidParameters::new(lar, xmax);
        let mut rng = NoiseRng::new(7);
        let f = comfort_noise_frame(&sid, &mut rng);
        assert_eq!(f.lar_c, lar);
        for (j, sf) in f.sub.iter().enumerate() {
            assert_eq!(sf.xmax_c, xmax[j], "Xmaxcr[{}] not passed through", j + 1);
        }
    }

    /// `SidParameters::new` forces the sentinel slot to zero even if
    /// the caller passes garbage there.
    #[test]
    fn sid_new_clears_sentinel() {
        let sid = SidParameters::new([999, 1, 2, 3, 4, 5, 6, 7, 8], [0; 4]);
        assert_eq!(sid.lar_cr[0], 0);
        assert_eq!(sid.lar_cr[1], 1);
    }

    /// The deterministic RNG is reproducible: two generators with the
    /// same seed and SID produce bit-identical comfort-noise frames.
    #[test]
    fn deterministic_for_fixed_seed() {
        let sid = SidParameters::default();
        let mut a = ComfortNoiseGenerator::new(sid, 42);
        let mut b = ComfortNoiseGenerator::new(sid, 42);
        for _ in 0..16 {
            assert_eq!(a.generate_frame(), b.generate_frame());
        }
    }

    /// Different seeds diverge (the excitation is genuinely
    /// randomised, not a constant).
    #[test]
    fn different_seeds_diverge() {
        let sid = SidParameters::default();
        let mut a = ComfortNoiseGenerator::new(sid, 1);
        let mut b = ComfortNoiseGenerator::new(sid, 2);
        let mut differed = false;
        for _ in 0..16 {
            if a.generate_frame() != b.generate_frame() {
                differed = true;
                break;
            }
        }
        assert!(differed, "two seeds produced identical noise");
    }

    /// Even from all-zero SID parameters the generator produces a
    /// non-silent signal: the §6.1 random RPE excitation drives the
    /// synthesis filter, so at least some sample is non-zero.
    #[test]
    fn all_zero_sid_still_produces_noise() {
        let sid = SidParameters::default();
        let mut g = ComfortNoiseGenerator::new(sid, 999);
        let mut any_nonzero = false;
        for _ in 0..8 {
            if g.generate_frame().iter().any(|&s| s != 0) {
                any_nonzero = true;
            }
        }
        assert!(any_nonzero, "all-zero SID gave digital silence");
    }

    /// `generate_frame` and `decode_frame(comfort_noise_frame(..))`
    /// agree when driven by an equivalently-seeded RNG and a fresh
    /// decoder — i.e. the generator is exactly "build §6.1 frame, run
    /// §5.3 decoder", with no hidden extra state.
    #[test]
    fn generator_equals_manual_decode() {
        let sid = SidParameters::new([0, 1, 2, 3, 4, 5, 6, 7, 8], [3, 5, 7, 9]);

        let mut g = ComfortNoiseGenerator::new(sid, 0xABCD);

        let mut rng = NoiseRng::new(0xABCD);
        let mut dec = DecoderState::new();

        for _ in 0..8 {
            let manual = dec.decode_frame(&comfort_noise_frame(&sid, &mut rng));
            assert_eq!(g.generate_frame(), manual);
        }
    }

    /// Updating the SID parameters takes effect on the next generated
    /// frame's codewords.
    #[test]
    fn update_sid_takes_effect() {
        let mut g = ComfortNoiseGenerator::new(SidParameters::default(), 5);
        let _ = g.next_frame_parameters();
        let new_sid = SidParameters::new([0, 1, 2, 3, 4, 5, 6, 7, 8], [10, 20, 30, 40]);
        g.update_sid(new_sid);
        assert_eq!(g.sid(), &new_sid);
        let f = g.next_frame_parameters();
        assert_eq!(f.lar_c, new_sid.lar_cr);
        assert_eq!(f.sub[0].xmax_c, 10);
        assert_eq!(f.sub[3].xmax_c, 40);
    }

    /// A comfort-noise frame round-trips through the §1.7 bit packer:
    /// all substituted codewords are within their Table 1.1 field
    /// widths (Ncr ≤ 120 < 128 fits the 7-bit Nc field; pulses ≤ 6 <
    /// 8 fit the 3-bit field; grids ≤ 3 fit the 2-bit field), so the
    /// frame the §6.1 generator builds is a legal on-wire GSM frame.
    #[test]
    fn comfort_noise_frame_packs_legally() {
        let sid = SidParameters::new([0, 9, 23, 15, 8, 7, 3, 3, 2], [5, 5, 5, 5]);
        let mut rng = NoiseRng::new(2024);
        for _ in 0..64 {
            let f = comfort_noise_frame(&sid, &mut rng);
            let bytes = f.to_bit_stream_msb_first();
            let g = UnpackedFrame::from_bit_stream_msb_first(&bytes).unwrap();
            assert_eq!(f, g, "comfort-noise frame did not survive §1.7 packing");
        }
    }

    /// `reset_decoder` returns the wrapped decoder to its home state
    /// but leaves SID + RNG untouched.
    #[test]
    fn reset_decoder_homes_decoder_only() {
        let sid = SidParameters::new([0, 1, 2, 3, 4, 5, 6, 7, 8], [3, 3, 3, 3]);
        let mut g = ComfortNoiseGenerator::new(sid, 77);
        // Run a few frames to dirty the decoder state.
        for _ in 0..4 {
            let _ = g.generate_frame();
        }
        g.reset_decoder();
        assert!(g.decoder.is_home_state());
        assert_eq!(g.sid(), &sid);
    }
}
