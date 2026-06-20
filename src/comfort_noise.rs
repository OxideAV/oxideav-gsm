//! GSM 06.12 comfort-noise — transmit-side §5.1 + receive-side §6.1.
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
//! This module implements both halves of GSM 06.12 that the staged
//! EN 300 963 PDF fully defines:
//!
//! * the **§5.1 transmit-side background-acoustic-noise evaluation**
//!   plus the **§5.2 parameter encoding** ([`NoiseEvaluator`]) — the
//!   `N = 4`-frame averaging of the unquantised LARs and block maxima,
//!   re-encoded *"as described in GSM 06.10"* into the
//!   [`SidParameters`] codewords;
//! * the **§6.1 receive-side comfort-noise generation**
//!   ([`ComfortNoiseGenerator`]) — driving the §5.3 RPE-LTP speech
//!   decoder with the SID-supplied noise parameters and the §6.1
//!   substituted excitation.
//!
//! Per §6.1, comfort noise is produced by driving the standard speech
//! decoder with a frame whose parameters are substituted as follows:
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
//! Three parts of the DTX comfort-noise path depend on companion
//! specifications that are **not** staged under `docs/audio/gsm/`, so
//! they are deliberately left out:
//!
//! * **Which frames are VAD = 0.** §5.1 averages only frames *"marked
//!   with VAD = 0"*; the Voice Activity Detection algorithm producing
//!   that mark is **GSM 06.32** (not staged). [`NoiseEvaluator`]
//!   therefore averages the VAD = 0 frames the caller supplies rather
//!   than detecting voice activity itself.
//! * **The §5.2 SID-frame *bit* layout.** The §5.2 *parameter*
//!   encoding (mean LARs / block amplitude → codewords) is implemented;
//!   what remains is the SID *bit* layout — the 95-bit all-zero "SID
//!   code word" inserted at *"those 95 bits of the encoded RPE-pulses
//!   Xmc which are in the error protection class I (see GSM 05.03,
//!   table 2)"*. **GSM 05.03 table 2** (not staged) holds those
//!   positions, so the transmit side stops at the §5.2 [`SidParameters`]
//!   codewords and does not pack a SID bitstream, nor implement the
//!   matching receive-side "valid SID frame" detector.
//! * **DTX scheduling.** When a SID frame is emitted or comfort noise
//!   updated (§6 opening, §6.1 closing) is defined in **GSM 06.31**
//!   (not staged).
//!
//! Accordingly the transmit side ([`NoiseEvaluator`]) emits the §5.2
//! [`SidParameters`] codewords and the receive side
//! ([`ComfortNoiseGenerator`]) consumes them — the full §5.1 → §5.2 →
//! §6.1 parameter loop — leaving the §5.2 SID *bit*-packing, the VAD
//! mark, and the DTX scheduling for a follow-up round once GSM 05.03 /
//! 06.31 / 06.32 are staged.

use crate::bitstream::{SubFrame, UnpackedFrame, SUBFRAMES};
use crate::decoder::DecoderState;
use crate::encoder::analysis::{code_xmax, quantise_lar};
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

/// Default number of frames over which [`SidInterpolator`] ramps the
/// SID noise parameters from their previous to their newly-received
/// values, implementing the §6.1 recommendation that on update *"the
/// parameters above should preferably be interpolated over a few frames
/// to obtain smooth transitions"*.
///
/// §6.1 says only *"a few frames"* — it does not fix the count, so this
/// is an implementation choice (like the [`NoiseRng`] generator itself).
/// Four was chosen to match the `N = 4` averaging window the transmit
/// side (§5.1) accumulates a SID frame over, so the receive-side ramp
/// spans the same time scale as the parameter-estimation window. Any
/// positive count satisfies the spec equally; callers can override via
/// [`ComfortNoiseGenerator::set_interpolation_frames`].
pub const DEFAULT_INTERPOLATION_FRAMES: u8 = 4;

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

/// Smooth-transition interpolator for the §6.1 comfort-noise update.
///
/// §6.1 closes with: *"When updating the comfort noise, the parameters
/// above should preferably be interpolated over a few frames to obtain
/// smooth transitions."* On receipt of a fresh SID frame the noise
/// level (the four `Xmaxcr` block amplitudes) and spectrum (the eight
/// `LARcr` log-area-ratio codewords) generally differ from the values
/// currently driving the generator; switching them abruptly would
/// re-introduce the very "modulation of the background noise" §4 warns
/// is "very annoying for the listener". This interpolator linearly
/// ramps each codeword from its previous value to the newly-received
/// value over [`Self::frames`] frames so the transition is gradual.
///
/// Only the SID-carried parameters are ramped. The §6.1 RPE pulses,
/// grid positions, fixed LTP lags (`40,120,40,120`), and zero LTP gains
/// are re-drawn / fixed every frame and carry no old→new transition, so
/// "the parameters above" that benefit from smoothing are exactly
/// `LARcr` and `Xmaxcr`.
///
/// The ramp count is an implementation choice — §6.1 specifies only
/// *"a few frames"* ([`DEFAULT_INTERPOLATION_FRAMES`]). With a count of
/// `f`, the `n`-th frame after the update (`n = 1..=f`) uses
/// `prev + (next - prev) * n / f` (rounded to nearest), so frame `f`
/// lands exactly on the new value and frames past `f` hold it.
#[derive(Debug, Clone)]
pub struct SidInterpolator {
    /// Parameters in effect before the most recent update (the ramp
    /// origin). Equal to `target` once the ramp has completed.
    prev: SidParameters,
    /// Newly-received parameters (the ramp destination).
    target: SidParameters,
    /// Total ramp length in frames; `0` disables interpolation (each
    /// update takes effect immediately).
    frames: u8,
    /// Frames already emitted since the update: `0` before the first
    /// post-update frame, saturating at `frames` once the ramp is done.
    elapsed: u8,
}

impl SidInterpolator {
    /// Start with `sid` already fully in effect (no ramp in progress)
    /// and a ramp length of `frames`.
    pub fn new(sid: SidParameters, frames: u8) -> Self {
        Self {
            prev: sid,
            target: sid,
            frames,
            elapsed: frames, // already settled on `target`
        }
    }

    /// Begin ramping from the parameters currently in effect (the value
    /// [`Self::current`] would return *now*) to `target` over
    /// [`Self::frames`] frames. If a ramp is already in progress its
    /// in-flight interpolated value becomes the new ramp origin, so
    /// back-to-back SID updates chain smoothly instead of snapping.
    pub fn update(&mut self, target: SidParameters) {
        self.prev = self.current();
        self.target = target;
        self.elapsed = 0;
        if self.frames == 0 {
            // No ramp: settle immediately.
            self.prev = target;
            self.elapsed = 0;
        }
    }

    /// Set the ramp length (`0` disables interpolation). Does not
    /// disturb a ramp already in progress beyond re-clamping `elapsed`.
    pub fn set_frames(&mut self, frames: u8) {
        self.frames = frames;
        if self.elapsed > frames {
            self.elapsed = frames;
        }
    }

    /// The current ramp length in frames.
    pub fn frames(&self) -> u8 {
        self.frames
    }

    /// The parameters in effect for the current frame, interpolated
    /// `prev → target` by `elapsed / frames`. Returns `target` once the
    /// ramp has completed (or immediately when `frames == 0`).
    pub fn current(&self) -> SidParameters {
        if self.frames == 0 || self.elapsed >= self.frames {
            return self.target;
        }
        let n = self.elapsed as i32;
        let f = self.frames as i32;
        let mut out = SidParameters::default();
        for i in 1..=8 {
            out.lar_cr[i] = lerp_round(
                self.prev.lar_cr[i] as i32,
                self.target.lar_cr[i] as i32,
                n,
                f,
            ) as i16;
        }
        for j in 0..SUBFRAMES {
            out.xmax_cr[j] = lerp_round(
                self.prev.xmax_cr[j] as i32,
                self.target.xmax_cr[j] as i32,
                n,
                f,
            ) as u8;
        }
        out
    }

    /// Advance the ramp by one frame (call once per generated frame,
    /// after reading [`Self::current`]). Saturates at [`Self::frames`].
    pub fn advance(&mut self) {
        if self.elapsed < self.frames {
            self.elapsed += 1;
        }
    }

    /// `true` once the ramp has reached `target` (or interpolation is
    /// disabled).
    pub fn is_settled(&self) -> bool {
        self.frames == 0 || self.elapsed >= self.frames
    }
}

/// Linear interpolation `a + (b - a) * n / f`, rounded to nearest
/// (round-half-away-from-zero), for `0 <= n <= f`, `f > 0`. The
/// rounding is symmetric so a `prev → target` ramp and the reverse
/// `target → prev` ramp visit the same intermediate magnitudes.
#[inline]
fn lerp_round(a: i32, b: i32, n: i32, f: i32) -> i32 {
    let num = (b - a) * n;
    let half = f / 2;
    let delta = if num >= 0 {
        (num + half) / f
    } else {
        (num - half) / f
    };
    a + delta
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
/// [`Self::update_sid`]. Per §6.1 *"when updating the comfort noise, the
/// parameters above should preferably be interpolated over a few frames
/// to obtain smooth transitions"*: the generator ramps the SID-carried
/// LARs and block amplitudes from their previous to their new values
/// over [`DEFAULT_INTERPOLATION_FRAMES`] frames (a [`SidInterpolator`]).
/// The ramp length is an implementation choice the spec leaves open
/// (*"a few frames"*); [`Self::set_interpolation_frames`] overrides it,
/// and a length of `0` reproduces the plain immediate replacement.
///
/// (The §6 *scheduling* of when a valid SID frame arrives is a GSM 06.31
/// concern, not staged; this generator implements the §6.1 generation
/// and the §6.1 update-smoothing, and leaves the schedule to the
/// caller's `update_sid` cadence.)
#[derive(Debug)]
pub struct ComfortNoiseGenerator {
    decoder: DecoderState,
    interp: SidInterpolator,
    rng: NoiseRng,
}

impl ComfortNoiseGenerator {
    /// Build a comfort-noise generator seeded with the given SID
    /// parameters and PRNG seed, using the default §6.1 update-smoothing
    /// ramp length ([`DEFAULT_INTERPOLATION_FRAMES`]). The wrapped
    /// decoder starts in its §4.6 home state, and the initial SID
    /// parameters are already fully in effect (no ramp in progress).
    pub fn new(sid: SidParameters, seed: u32) -> Self {
        Self {
            decoder: DecoderState::new(),
            interp: SidInterpolator::new(sid, DEFAULT_INTERPOLATION_FRAMES),
            rng: NoiseRng::new(seed),
        }
    }

    /// Apply a freshly received SID frame, beginning the §6.1
    /// smooth-transition ramp from the parameters currently in effect to
    /// `sid` over the configured number of frames (*"the parameters
    /// above should preferably be interpolated over a few frames to
    /// obtain smooth transitions"*). Updating happens *"each time a
    /// valid SID frame is received"*; with the ramp length set to `0`
    /// this is the plain immediate replacement.
    pub fn update_sid(&mut self, sid: SidParameters) {
        self.interp.update(sid);
    }

    /// Set the §6.1 update-smoothing ramp length in frames (`0` disables
    /// interpolation — each [`Self::update_sid`] then takes effect
    /// immediately). §6.1 specifies only *"a few frames"*, so the count
    /// is an implementation choice.
    pub fn set_interpolation_frames(&mut self, frames: u8) {
        self.interp.set_frames(frames);
    }

    /// The §6.1 update-smoothing ramp length currently in effect.
    pub fn interpolation_frames(&self) -> u8 {
        self.interp.frames()
    }

    /// The SID parameters in effect for the *next* frame — the ramp's
    /// current interpolated value, which equals the most recently
    /// received SID frame once the ramp has settled.
    pub fn sid(&self) -> SidParameters {
        self.interp.current()
    }

    /// `true` once the §6.1 update ramp has reached the most recently
    /// received SID parameters (or interpolation is disabled).
    pub fn is_settled(&self) -> bool {
        self.interp.is_settled()
    }

    /// Generate one 20 ms comfort-noise frame: build the §6.1
    /// substituted [`UnpackedFrame`] (from the interpolated SID
    /// parameters in effect this frame) and synthesise it through the
    /// standard §5.3 decoder. Returns 160 linear 13-bit PCM samples
    /// (the three LSBs cleared per §5.3.7, as for any decoded frame).
    /// Advances the §6.1 update ramp by one frame.
    pub fn generate_frame(&mut self) -> [i16; FRAME_SAMPLES] {
        let sid = self.interp.current();
        let frame = comfort_noise_frame(&sid, &mut self.rng);
        let pcm = self.decoder.decode_frame(&frame);
        self.interp.advance();
        pcm
    }

    /// The next §6.1 comfort-noise frame's parameters, without decoding
    /// — useful for callers that want to inspect or re-pack the
    /// substituted codewords. Advances the PRNG and the §6.1 update ramp
    /// exactly as [`Self::generate_frame`] would.
    pub fn next_frame_parameters(&mut self) -> UnpackedFrame {
        let sid = self.interp.current();
        let frame = comfort_noise_frame(&sid, &mut self.rng);
        self.interp.advance();
        frame
    }

    /// Reset the wrapped decoder to its §4.6 home state (e.g. on a
    /// codec-homing event) without disturbing the SID parameters or
    /// the PRNG.
    pub fn reset_decoder(&mut self) {
        self.decoder.reset();
    }
}

/// One classified frame arriving at the §6 receive side during
/// Discontinuous Transmission.
///
/// GSM 06.12 §6 says comfort-noise generation *"is started or updated
/// whenever a **valid SID frame** is received"*, and §4 describes the
/// background-noise parameters being sent *"before the radio
/// transmission is cut and at a regular low rate afterwards"* — so
/// between SID frames (and across the radio cut) the receiver gets no
/// usable speech payload and must keep generating comfort noise from the
/// last received parameters.
///
/// To dispatch correctly the receiver needs the arriving frame already
/// *classified* into one of three kinds. The classification itself is a
/// **documented spec gap**: telling a valid SID frame apart from a
/// speech frame requires detecting the §5.2 "SID code word" — the 95
/// all-zero bits at the GSM 05.03 table-2 class-I positions — and the
/// *scheduling* of which radio frames carry SID / speech / nothing is
/// **GSM 06.31**. Neither GSM 05.03 nor GSM 06.31 is staged under
/// `docs/audio/gsm/`, so — exactly as [`NoiseEvaluator`] accepts the
/// VAD = 0 mark from the caller rather than running the (unstaged) GSM
/// 06.32 VAD itself — [`DtxReceiver`] accepts the *classified* frame
/// from the caller and implements only the §6 dispatch the staged
/// EN 300 963 PDF fully defines.
#[derive(Debug, Clone)]
pub enum RxFrame {
    /// A normal §1.7 speech frame: decode it through the standard §5.3
    /// RPE-LTP speech decoder. Receiving speech ends any comfort-noise
    /// period.
    Speech(Box<UnpackedFrame>),
    /// A *valid SID frame* carrying fresh §5.2 comfort-noise parameters.
    /// Per §6 this *"start[s] or update[s]"* comfort-noise generation:
    /// the first one opens a comfort-noise period, each subsequent one
    /// updates the parameters (§6.1 smoothly interpolated).
    Sid(SidParameters),
    /// No usable payload this frame — the radio link is cut, or a frame
    /// between SID updates carries nothing. Inside a comfort-noise period
    /// the receiver keeps synthesising noise from the last SID
    /// parameters (§4 *"at a regular low rate afterwards"*); see
    /// [`DtxReceiver::receive`] for the (out-of-06.12-scope) behaviour
    /// before any SID has been seen.
    NoData,
}

/// The §6 receive-side DTX state: whether the receiver is currently
/// decoding speech or synthesising comfort noise.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DtxState {
    /// Decoding ordinary speech frames (no comfort-noise period active,
    /// or one just ended on a speech frame). This is the initial state.
    Speech,
    /// A comfort-noise period is active: a valid SID frame has been
    /// received and the receiver is synthesising §6.1 comfort noise,
    /// updated on each further SID and continued across [`RxFrame::
    /// NoData`] frames until speech resumes.
    ComfortNoise,
}

/// GSM 06.12 §6 receive-side Discontinuous-Transmission dispatcher.
///
/// Source: ETSI EN 300 963 V8.0.1 §6 *"Functions on the receive side"*,
/// §6.1 *"Comfort noise generation and updating"*.
///
/// §6 defines the receive-side behaviour during DTX as a two-state
/// process driven by the kind of frame that arrives:
///
/// * a **speech** frame is decoded normally (§5.3) and ends any
///   comfort-noise period;
/// * a **valid SID** frame *"start[s] or update[s]"* comfort-noise
///   generation (§6 opening) — the first opens the period, each later
///   one updates the §6.1 parameters (smoothly interpolated, §6.1
///   closing);
/// * **no data** (the radio cut, or a gap between SID updates) keeps the
///   comfort-noise generator running from the last SID parameters (§4
///   *"at a regular low rate afterwards"*).
///
/// The crucial correctness property is **filter continuity**: §6.1 says
/// comfort noise *"performs the standard operations described in GSM
/// 06.10"* — it drives *the* speech decoder, the same one the speech
/// frames use. So [`DtxReceiver`] holds a **single** [`DecoderState`]
/// shared between speech decode and comfort-noise synthesis. When a
/// speech burst gives way to a comfort-noise period (or vice versa), the
/// §5.3 short-term synthesis lattice, the §5.3.2 long-term delay line,
/// and the §5.3.5 de-emphasis memory carry straight across the
/// transition — which is exactly what stops the boundary from clicking
/// (the *"modulation of the background noise"* §4 warns is *"very
/// annoying for the listener"*). Wrapping the existing
/// [`ComfortNoiseGenerator`] would *not* give this, because that type
/// owns a private decoder; the receiver therefore drives the §6.1
/// [`comfort_noise_frame`] substitution and the shared decoder directly.
///
/// Every [`Self::receive`] call consumes one classified [`RxFrame`] and
/// returns 160 linear PCM samples — speech, or comfort noise, depending
/// on the frame and the current [`DtxState`].
///
/// ## What this dispatcher does *not* decide (documented spec gap)
///
/// [`DtxReceiver`] implements the §6 *dispatch*; it does **not** classify
/// the incoming frames, because classification depends on companion
/// specs not staged under `docs/audio/gsm/`:
///
/// * Telling a **valid SID frame** from a speech frame means detecting
///   the §5.2 "SID code word" (95 all-zero bits at the class-I positions
///   of **GSM 05.03 table 2**) — not staged.
/// * The **SID / speech / no-data scheduling** on the radio path is
///   **GSM 06.31** — not staged (§5 and §6 both defer to it explicitly).
///
/// So the caller hands [`Self::receive`] an already-classified
/// [`RxFrame`], the same way [`NoiseEvaluator`] is handed VAD = 0 frames
/// rather than running the (unstaged GSM 06.32) VAD. Once GSM 05.03 /
/// 06.31 are staged, a thin classifier can sit in front of this
/// dispatcher without changing its §6 logic.
#[derive(Debug)]
pub struct DtxReceiver {
    /// The single §5.3 speech decoder shared by speech decode and §6.1
    /// comfort-noise synthesis — the source of the cross-transition
    /// filter continuity §4 needs.
    decoder: DecoderState,
    /// The §6.1 parameter interpolator: holds the comfort-noise SID
    /// parameters currently in effect and ramps them on update.
    interp: SidInterpolator,
    /// The §6.1 random source for the substituted RPE pulses / grid
    /// positions.
    rng: NoiseRng,
    /// Whether a comfort-noise period is currently active.
    state: DtxState,
    /// `true` once at least one valid SID frame has opened a
    /// comfort-noise period since construction / [`Self::reset`]. Until
    /// then there are no comfort-noise parameters to fall back on, so a
    /// [`RxFrame::NoData`] cannot synthesise §6.1 noise.
    sid_seen: bool,
}

impl DtxReceiver {
    /// Build a receive-side DTX dispatcher in the [`DtxState::Speech`]
    /// state with a fresh §4.6 home-state decoder and the default §6.1
    /// update-smoothing ramp ([`DEFAULT_INTERPOLATION_FRAMES`]).
    ///
    /// `seed` seeds the §6.1 comfort-noise PRNG; any value is acceptable
    /// (the spec leaves the generator an implementation choice — see
    /// [`NoiseRng`]). No comfort-noise parameters exist yet; the first
    /// [`RxFrame::Sid`] supplies them.
    pub fn new(seed: u32) -> Self {
        Self {
            decoder: DecoderState::new(),
            interp: SidInterpolator::new(SidParameters::default(), DEFAULT_INTERPOLATION_FRAMES),
            rng: NoiseRng::new(seed),
            state: DtxState::Speech,
            sid_seen: false,
        }
    }

    /// The current §6 receive-side DTX state.
    pub fn state(&self) -> DtxState {
        self.state
    }

    /// `true` while a comfort-noise period is active (the last decisive
    /// frame was a valid SID or a no-data continuation of one).
    pub fn is_generating_comfort_noise(&self) -> bool {
        self.state == DtxState::ComfortNoise
    }

    /// `true` once at least one valid SID frame has been received since
    /// construction / [`Self::reset`] (so [`RxFrame::NoData`] has §6.1
    /// parameters to synthesise from).
    pub fn has_sid(&self) -> bool {
        self.sid_seen
    }

    /// The §6.1 comfort-noise SID parameters currently in effect (the
    /// interpolator's current value). Before any SID has been received
    /// this is [`SidParameters::default`] (all-zero).
    pub fn sid(&self) -> SidParameters {
        self.interp.current()
    }

    /// Set the §6.1 update-smoothing ramp length in frames (`0` disables
    /// interpolation — each SID update then takes effect immediately).
    /// §6.1 specifies only *"a few frames"*, so the count is an
    /// implementation choice ([`DEFAULT_INTERPOLATION_FRAMES`]).
    pub fn set_interpolation_frames(&mut self, frames: u8) {
        self.interp.set_frames(frames);
    }

    /// The §6.1 update-smoothing ramp length currently in effect.
    pub fn interpolation_frames(&self) -> u8 {
        self.interp.frames()
    }

    /// Reset the receiver: home the shared §5.3 decoder, drop the
    /// comfort-noise parameters, and return to [`DtxState::Speech`]. The
    /// PRNG and ramp length are preserved. Use this on a §4.4 codec
    /// homing event or to restart a stream.
    pub fn reset(&mut self) {
        self.decoder.reset();
        self.interp = SidInterpolator::new(SidParameters::default(), self.interp.frames());
        self.state = DtxState::Speech;
        self.sid_seen = false;
    }

    /// Dispatch one classified [`RxFrame`] per §6 and return its 160
    /// linear PCM samples.
    ///
    /// * [`RxFrame::Speech`] — decode the frame through the shared §5.3
    ///   speech decoder and return to / stay in [`DtxState::Speech`]. A
    ///   speech frame ends any comfort-noise period.
    /// * [`RxFrame::Sid`] — *"start or update"* comfort noise (§6): the
    ///   parameters are applied to the §6.1 interpolator (ramped from the
    ///   value currently in effect, so a mid-burst update is smooth),
    ///   the state becomes [`DtxState::ComfortNoise`], and one §6.1
    ///   comfort-noise frame is synthesised through the shared decoder.
    /// * [`RxFrame::NoData`] — inside a comfort-noise period, keep
    ///   synthesising §6.1 comfort noise from the parameters in effect
    ///   (§4 *"at a regular low rate afterwards"*), advancing the §6.1
    ///   update ramp. Before any SID has opened a period (`!has_sid()`),
    ///   there are no comfort-noise parameters and no speech to conceal;
    ///   §6.12 does not cover this case (it is GSM 06.31 bad-frame
    ///   handling, not staged), so the shared decoder is driven with the
    ///   all-zero comfort parameters and the state stays
    ///   [`DtxState::Speech`] — a defined, click-free fallback that does
    ///   not pretend to implement the unstaged 06.31 substitution/muting.
    pub fn receive(&mut self, frame: RxFrame) -> [i16; FRAME_SAMPLES] {
        match frame {
            RxFrame::Speech(f) => {
                self.state = DtxState::Speech;
                self.decoder.decode_frame(&f)
            }
            RxFrame::Sid(sid) => {
                // §6: "started or updated whenever a valid SID frame is
                // received". The first SID opens the period; a later one
                // ramps from the value currently in effect.
                if self.sid_seen {
                    self.interp.update(sid);
                } else {
                    // First SID: snap straight to it (there is no prior
                    // comfort-noise value to ramp from) by re-seating the
                    // interpolator with the new value already settled.
                    self.interp = SidInterpolator::new(sid, self.interp.frames());
                    self.sid_seen = true;
                }
                self.state = DtxState::ComfortNoise;
                self.synthesise_comfort_noise()
            }
            RxFrame::NoData => {
                if self.sid_seen && self.state == DtxState::ComfortNoise {
                    self.synthesise_comfort_noise()
                } else {
                    // Out of 06.12 scope (no SID yet → GSM 06.31 bad-frame
                    // handling). Drive the decoder with the all-zero
                    // comfort parameters so output stays continuous;
                    // remain in Speech state.
                    let sid = self.interp.current();
                    let cn = comfort_noise_frame(&sid, &mut self.rng);
                    self.decoder.decode_frame(&cn)
                }
            }
        }
    }

    /// Build one §6.1 comfort-noise frame from the parameters currently
    /// in effect and synthesise it through the shared §5.3 decoder,
    /// advancing the §6.1 update ramp by one frame.
    fn synthesise_comfort_noise(&mut self) -> [i16; FRAME_SAMPLES] {
        let sid = self.interp.current();
        let cn = comfort_noise_frame(&sid, &mut self.rng);
        let pcm = self.decoder.decode_frame(&cn);
        self.interp.advance();
        pcm
    }
}

/// The §5.1 averaging-window length: comfort-noise parameters are
/// evaluated over *"N = 4 consecutive frames marked with VAD = 0"*.
pub const NOISE_EVAL_FRAMES: usize = 4;

/// One full-rate speech frame's *unquantised* noise parameters, as
/// §5.1 consumes them for the background-acoustic-noise evaluation.
///
/// §5.1 is explicit that it *"uses the **unquantized** block amplitude
/// and Log Area Ratio (LAR) parameters of the full rate speech encoder,
/// defined in 4.2.15 and 4.2.6 of GSM 06.10"* — i.e. the §5.2.6 LAR
/// values *before* the §5.2.7 quantiser, and the §5.2.15 block maximum
/// `xmax = max|xM[i]|` *before* the `xmaxc` coding step. The means are
/// taken on these unquantised values and only then *"encoded as
/// described in GSM 06.10"* (§5.2). Averaging the *coded* `LARc` /
/// `xmaxc` instead would be a different (and incorrect) quantity.
///
/// * `lar[1..=8]` is the §5.2.6 [`analysis::reflection_to_lar`] /
///   [`analysis::analyse_frame`] output (entry 0 is the 1-based
///   sentinel).
/// * `xmax[1..=4]` is the §5.2.15 block maximum of each of the four
///   sub-segments — the [`analysis::ApcmQuantised::xmax`] field.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NoiseFrameParameters {
    /// `LAR[1..=8]` — the frame's eight *unquantised* log-area ratios
    /// (the §5.2.6 output; entry 0 is a sentinel zero for 1-based
    /// indexing).
    pub lar: [i16; 9],
    /// `xmax[1..=4]` — the four sub-segments' *unquantised* block
    /// maxima (the §5.2.15 `xmax` search result of each sub-segment).
    pub xmax: [i16; SUBFRAMES],
}

impl NoiseFrameParameters {
    /// Build the §5.1 input from one frame's *unquantised* encoder
    /// intermediates: the eight §5.2.6 LARs (`lar[1..=8]`; index 0 is
    /// forced to the sentinel zero) and the four sub-segments' §5.2.15
    /// block maxima `xmax[0..=3]`.
    ///
    /// During normal encoding these are exactly the
    /// [`analysis::analyse_frame`] output (the per-frame LARs) and the
    /// [`analysis::ApcmQuantised::xmax`] field of each sub-segment's
    /// [`analysis::apcm_quantise_rpe`] result.
    pub fn new(lar: [i16; 9], xmax: [i16; SUBFRAMES]) -> Self {
        let mut lar = lar;
        lar[0] = 0;
        Self { lar, xmax }
    }
}

/// GSM 06.12 §5.1 transmit-side background-acoustic-noise evaluator.
///
/// Source: ETSI EN 300 963 V8.0.1 §5 *"Functions on the transmit
/// side"*, §5.1 *"Background acoustic noise evaluation"*.
///
/// When Discontinuous Transmission detects a speech pause, the
/// transmit side must describe the residual background noise so the
/// receive side ([`ComfortNoiseGenerator`]) can resynthesise it. §5.1
/// specifies the noise description as the arithmetic mean of the
/// *unquantised* coded-speech noise parameters over the `N = 4`
/// consecutive frames marked `VAD = 0`:
///
/// * **Log-Area Ratios** —
///   `mean(LAR(i)) = (1/N) · Σ_{n=1}^{N} LAR[j−n](i)`,  i = 1..8.
///   Each of the eight §5.2.6 LAR values is averaged independently
///   across the four frames.
/// * **Block amplitude** —
///   `mean(xmax) = (1/(4N)) · Σ_{n=1}^{N} Σ_{i=1}^{4} xmax[j−n](i)`.
///   A *single* mean is taken over **all** `4·N = 16` sub-segment block
///   maxima (four sub-segments × four frames).
///
/// §5.2 then *encodes those means "as described in GSM 06.10"*: the
/// mean LARs through the §5.2.7 [`analysis::quantise_lar`] quantiser
/// and the single mean block amplitude through the §5.2.15
/// [`analysis::code_xmax`] coder, the latter *"repeated four times
/// inside the frame"* — so the [`SidParameters`] this evaluator emits
/// already carries `LARcr` / `Xmaxcr` codewords ready for the §6.1
/// receive side, with the same `Xmaxcr` value in all four slots.
///
/// This evaluator implements the part of §5 the staged EN 300 963 PDF
/// fully defines: the §5.1 averaging plus the §5.2 *parameter*
/// re-encoding (everything except the §5.2 SID *bit* layout, which
/// needs the unstaged GSM 05.03 — see below). It takes the per-frame
/// *unquantised* noise parameters of the VAD = 0 frames as input
/// ([`NoiseFrameParameters`], built from the encoder's §5.2.6 /
/// §5.2.15 intermediates) and emits the [`SidParameters`] the existing
/// §6.1 receive side consumes — closing the §5.1 → §6.1 loop.
///
/// ## What this evaluator does *not* do (documented spec gap)
///
/// §5.1 says the averaged frames are *"marked with VAD = 0"*; the VAD
/// (Voice Activity Detection) algorithm that produces that mark is
/// **GSM 06.32**, which is not staged under `docs/audio/gsm/`. This
/// evaluator therefore accepts the VAD = 0 frames as given by the
/// caller rather than detecting voice activity itself. Likewise the
/// §5.2 *bit-level* SID-frame layout — inserting the 95-bit all-zero
/// "SID code word" at *"those 95 bits of the encoded RPE-pulses Xmc
/// which are in the error protection class I (see GSM 05.03, table
/// 2)"* — depends on **GSM 05.03 table 2**, also not staged, so the
/// transmit side stops at the §5.1 parameter mean (the part the staged
/// PDF defines) and emits [`SidParameters`] rather than a packed SID
/// bitstream. The DTX scheduling that decides *when* a SID frame is
/// emitted is **GSM 06.31** (not staged).
///
/// The §5.1 equations are exact arithmetic means of integer-valued
/// parameters; the staged PDF does not state the rounding direction of
/// the final integer division. Round-to-nearest (ties away from zero,
/// symmetric for the signed LARs) is used — the natural reading of
/// *"mean"* — and is the only quantity left to implementation choice
/// here, on the same footing as the receive-side [`NoiseRng`]
/// generator. No staged conformance vector exercises the SID parameter
/// values, so the choice is not bit-pinned by the spec.
#[derive(Debug, Clone)]
pub struct NoiseEvaluator {
    /// Fixed `N`-slot ring of the most recent VAD = 0 frame parameters.
    /// `count` says how many slots are populated (0..=[`NOISE_EVAL_FRAMES`]);
    /// `next` is the write cursor. The averaging is order-independent, so
    /// the ring needs no explicit oldest-first ordering.
    window: [NoiseFrameParameters; NOISE_EVAL_FRAMES],
    count: usize,
    next: usize,
}

impl Default for NoiseEvaluator {
    fn default() -> Self {
        Self::new()
    }
}

impl NoiseEvaluator {
    /// Build a fresh evaluator with an empty averaging window.
    pub fn new() -> Self {
        Self {
            window: [NoiseFrameParameters {
                lar: [0; 9],
                xmax: [0; SUBFRAMES],
            }; NOISE_EVAL_FRAMES],
            count: 0,
            next: 0,
        }
    }

    /// Discard all accumulated VAD = 0 frames (e.g. on a speech burst,
    /// which restarts the §5.1 window).
    pub fn reset(&mut self) {
        self.count = 0;
        self.next = 0;
    }

    /// Number of VAD = 0 frames currently held in the averaging window
    /// (0..=[`NOISE_EVAL_FRAMES`]).
    pub fn len(&self) -> usize {
        self.count
    }

    /// Whether the averaging window holds no frames yet.
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    /// `true` once [`NOISE_EVAL_FRAMES`] (= 4) VAD = 0 frames have been
    /// pushed — i.e. a §5.1 SID frame can be evaluated over a full
    /// window. [`Self::evaluate`] also yields a value for a partial
    /// window (averaging only the frames seen so far); this predicate
    /// just reports whether the canonical `N = 4` window is complete.
    pub fn is_ready(&self) -> bool {
        self.count >= NOISE_EVAL_FRAMES
    }

    /// Push one VAD = 0 frame's noise parameters into the §5.1 window.
    ///
    /// The window keeps only the [`NOISE_EVAL_FRAMES`] most recent
    /// frames (`j−1 … j−N`); pushing a fifth frame evicts the oldest,
    /// so the evaluator always averages over the latest four VAD = 0
    /// frames as §5.1 specifies (*"the previous frames"*).
    pub fn push_frame(&mut self, params: NoiseFrameParameters) {
        self.window[self.next] = params;
        self.next = (self.next + 1) % NOISE_EVAL_FRAMES;
        if self.count < NOISE_EVAL_FRAMES {
            self.count += 1;
        }
    }

    /// Evaluate §5.1 + §5.2 over the frames accumulated so far,
    /// returning the [`SidParameters`] the §6.1 receive side consumes.
    ///
    /// Steps:
    ///
    /// 1. **§5.1 means.** Average each of the eight *unquantised* LARs
    ///    independently across the `n` frames, and take a single mean
    ///    over all `4·n` *unquantised* sub-segment block maxima.
    /// 2. **§5.2 encoding.** Encode the eight mean LARs through the
    ///    §5.2.7 [`analysis::quantise_lar`] quantiser, and the single
    ///    mean block amplitude through the §5.2.15
    ///    [`analysis::code_xmax`] coder, writing that one `Xmaxcr`
    ///    codeword into all four slots per §5.2 *"repeated four times
    ///    inside the frame"*.
    ///
    /// Returns `None` only when no frame has been pushed (an empty mean
    /// is undefined). With a partial window (1..3 frames) it averages
    /// what is present; with the full `N = 4` window it is the canonical
    /// §5.1 evaluation.
    pub fn evaluate(&self) -> Option<SidParameters> {
        let n = self.count;
        if n == 0 {
            return None;
        }
        let frames = &self.window[..n];

        // §5.1 mean(LAR(i)) — average each of the eight unquantised LARs
        // independently across the n frames. Slot 0 stays the 1-based
        // sentinel.
        let mut lar_mean = [0i16; 9];
        for (i, slot) in lar_mean.iter_mut().enumerate().take(9).skip(1) {
            let sum: i32 = frames.iter().map(|f| f.lar[i] as i32).sum();
            *slot = mean_round(sum, n as i32) as i16;
        }
        // §5.2 — encode the mean LARs "as described in GSM 06.10"
        // (§5.2.7 quantiser → LARcr codewords).
        let lar_cr = quantise_lar(&lar_mean);

        // §5.1 mean(xmax) — one mean over all 4·n unquantised sub-segment
        // block maxima.
        let xmax_sum: i32 = frames
            .iter()
            .flat_map(|f| f.xmax.iter())
            .map(|&x| x as i32)
            .sum();
        let xmax_mean = mean_round(xmax_sum, (SUBFRAMES * n) as i32) as i16;
        // §5.2 — encode the mean block amplitude (§5.2.15 coder), then
        // repeat the single Xmaxcr codeword across the four slots.
        let xmax_cr = code_xmax(xmax_mean) as u8;

        Some(SidParameters {
            lar_cr,
            xmax_cr: [xmax_cr; SUBFRAMES],
        })
    }
}

/// Round-to-nearest integer division (ties away from zero) for the
/// §5.1 arithmetic means. `denom > 0`; `numer` may be negative (LAR
/// codewords are signed). See [`NoiseEvaluator`] for why the rounding
/// direction is an implementation choice the spec leaves open.
fn mean_round(numer: i32, denom: i32) -> i32 {
    debug_assert!(denom > 0);
    if numer >= 0 {
        (numer + denom / 2) / denom
    } else {
        -((-numer + denom / 2) / denom)
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

    /// Updating the SID parameters with interpolation disabled takes
    /// effect immediately on the next generated frame's codewords.
    #[test]
    fn update_sid_takes_effect_immediately_without_interp() {
        let mut g = ComfortNoiseGenerator::new(SidParameters::default(), 5);
        g.set_interpolation_frames(0);
        let _ = g.next_frame_parameters();
        let new_sid = SidParameters::new([0, 1, 2, 3, 4, 5, 6, 7, 8], [10, 20, 30, 40]);
        g.update_sid(new_sid);
        assert_eq!(g.sid(), new_sid);
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
        assert_eq!(g.sid(), sid);
    }

    /// §6.1 smooth transition: with the default 4-frame ramp, after an
    /// update the SID parameters in effect step monotonically from the
    /// previous value (the first post-update frame still holds prev,
    /// n=0) up to the new value, landing exactly on the new value at the
    /// frame `f` read and holding it afterwards.
    #[test]
    fn update_sid_ramps_smoothly() {
        // prev all-zero, target xmax = 80 across the board: the reads
        // visit 0, 20, 40, 60, 80 (80*n/4) over the ramp, settling at 80.
        let prev = SidParameters::default();
        let target = SidParameters::new([0; 9], [80, 80, 80, 80]);
        let mut g = ComfortNoiseGenerator::new(prev, 9);
        assert_eq!(g.interpolation_frames(), DEFAULT_INTERPOLATION_FRAMES);
        g.update_sid(target);

        let expected = [0u8, 20, 40, 60, 80];
        for (i, &e) in expected.iter().enumerate() {
            let s = g.sid();
            assert_eq!(s.xmax_cr, [e, e, e, e], "frame {} ramp value", i + 1);
            let _ = g.next_frame_parameters(); // advance the ramp
        }
        // Past the ramp the new value holds.
        assert!(g.is_settled());
        assert_eq!(g.sid().xmax_cr, [80, 80, 80, 80]);
    }

    /// §6.1 ramp on the LARs too: a clean halfway point is hit at the
    /// midpoint of an even-length ramp.
    #[test]
    fn lar_ramp_midpoint() {
        let prev = SidParameters::new([0, 0, 0, 0, 0, 0, 0, 0, 0], [0; 4]);
        let target = SidParameters::new([0, 40, 20, -40, 0, 0, 0, 0, 0], [0; 4]);
        let mut interp = SidInterpolator::new(prev, 4);
        interp.update(target);
        // frame 1: n=0 -> prev
        assert_eq!(interp.current().lar_cr[1..=3], [0, 0, 0]);
        interp.advance();
        // frame 2: n=1 -> a quarter
        assert_eq!(interp.current().lar_cr[1..=3], [10, 5, -10]);
        interp.advance();
        // frame 3: n=2 -> halfway
        assert_eq!(interp.current().lar_cr[1..=3], [20, 10, -20]);
        interp.advance();
        // frame 4: n=3 -> three quarters
        assert_eq!(interp.current().lar_cr[1..=3], [30, 15, -30]);
        interp.advance();
        // frame 5: settled on target
        assert_eq!(interp.current().lar_cr[1..=3], [40, 20, -40]);
        assert!(interp.is_settled());
    }

    /// Disabling interpolation (`frames == 0`) makes every update snap.
    #[test]
    fn zero_frames_snaps() {
        let mut interp = SidInterpolator::new(SidParameters::default(), 0);
        let target = SidParameters::new([0; 9], [50, 50, 50, 50]);
        interp.update(target);
        assert!(interp.is_settled());
        assert_eq!(interp.current().xmax_cr, [50, 50, 50, 50]);
    }

    /// A second update mid-ramp re-bases the ramp from the in-flight
    /// interpolated value rather than snapping back to the old origin.
    #[test]
    fn chained_update_rebases_from_current() {
        let mut interp = SidInterpolator::new(SidParameters::default(), 4);
        interp.update(SidParameters::new([0; 9], [80, 80, 80, 80]));
        interp.advance(); // n=1 -> 20
        interp.advance(); // n=2 -> 40
        let mid = interp.current().xmax_cr[0];
        assert_eq!(mid, 40);
        // New update from the in-flight value (40) toward 0.
        interp.update(SidParameters::new([0; 9], [0, 0, 0, 0]));
        // First post-update frame still sits at the old in-flight value.
        assert_eq!(interp.current().xmax_cr[0], 40);
        interp.advance();
        // Ramps 40 -> 0 over 4 frames: n=1 -> 30.
        assert_eq!(interp.current().xmax_cr[0], 30);
    }

    /// The §6.1 ramp produces no out-of-range codewords: every
    /// interpolated frame still packs legally through §1.7.
    #[test]
    fn ramped_frames_pack_legally() {
        let mut g = ComfortNoiseGenerator::new(
            SidParameters::new([0, 9, 23, 15, 8, 7, 3, 3, 2], [3, 3, 3, 3]),
            2024,
        );
        g.update_sid(SidParameters::new(
            [0, 50, 40, 30, 20, 10, 5, 4, 3],
            [60, 60, 60, 60],
        ));
        for _ in 0..12 {
            let f = g.next_frame_parameters();
            let bytes = f.to_bit_stream_msb_first();
            let back = UnpackedFrame::from_bit_stream_msb_first(&bytes).unwrap();
            assert_eq!(
                f, back,
                "ramped comfort-noise frame did not survive §1.7 packing"
            );
        }
    }

    // ----- §6 receive-side DTX dispatch (DtxReceiver) -----

    fn speech_frame() -> Box<UnpackedFrame> {
        // A non-homing, non-trivial speech frame: distinct LARs and a
        // varied sub-frame so it exercises the §5.3 pipeline.
        let mut f = UnpackedFrame {
            lar_c: [0, 30, 20, 18, 12, 10, 6, 4, 3],
            sub: [SubFrame::default(); SUBFRAMES],
        };
        for (j, sf) in f.sub.iter_mut().enumerate() {
            sf.n_c = 40 + j as u8 * 10;
            sf.b_c = 1;
            sf.m_c = (j % 4) as u8;
            sf.xmax_c = 20 + j as u8;
            for (i, p) in sf.x_mc.iter_mut().enumerate() {
                *p = (i as u8) % 8;
            }
        }
        Box::new(f)
    }

    /// A fresh receiver starts in the Speech state with no SID seen.
    #[test]
    fn receiver_starts_in_speech_state() {
        let r = DtxReceiver::new(1);
        assert_eq!(r.state(), DtxState::Speech);
        assert!(!r.is_generating_comfort_noise());
        assert!(!r.has_sid());
        assert_eq!(r.sid(), SidParameters::default());
    }

    /// A speech frame is decoded and keeps the receiver in Speech state;
    /// it produces exactly the same PCM as the bare §5.3 decoder.
    #[test]
    fn speech_frame_matches_bare_decoder() {
        let mut r = DtxReceiver::new(5);
        let f = speech_frame();
        let got = r.receive(RxFrame::Speech(f.clone()));

        let mut dec = DecoderState::new();
        let want = dec.decode_frame(&f);
        assert_eq!(got, want);
        assert_eq!(r.state(), DtxState::Speech);
        assert!(!r.has_sid());
    }

    /// A valid SID frame opens a comfort-noise period: state flips to
    /// ComfortNoise, has_sid() becomes true, and the parameters in effect
    /// are the received SID (first SID snaps, no ramp).
    #[test]
    fn sid_opens_comfort_noise_period() {
        let mut r = DtxReceiver::new(9);
        let sid = SidParameters::new([0, 9, 23, 15, 8, 7, 3, 3, 2], [5, 5, 5, 5]);
        let _ = r.receive(RxFrame::Sid(sid));
        assert_eq!(r.state(), DtxState::ComfortNoise);
        assert!(r.is_generating_comfort_noise());
        assert!(r.has_sid());
        assert_eq!(r.sid(), sid);
    }

    /// Inside a comfort-noise period, NoData keeps synthesising noise
    /// from the last SID and stays in ComfortNoise state.
    #[test]
    fn no_data_continues_comfort_noise() {
        let mut r = DtxReceiver::new(11);
        let sid = SidParameters::new([0, 9, 23, 15, 8, 7, 3, 3, 2], [7, 7, 7, 7]);
        let _ = r.receive(RxFrame::Sid(sid));
        let mut any_nonzero = false;
        for _ in 0..8 {
            let pcm = r.receive(RxFrame::NoData);
            assert_eq!(r.state(), DtxState::ComfortNoise);
            if pcm.iter().any(|&s| s != 0) {
                any_nonzero = true;
            }
        }
        assert!(any_nonzero, "comfort noise was digital silence");
    }

    /// A speech frame ends a comfort-noise period (state returns to
    /// Speech) but leaves has_sid() set.
    #[test]
    fn speech_ends_comfort_noise_period() {
        let mut r = DtxReceiver::new(13);
        let sid = SidParameters::new([0, 1, 2, 3, 4, 5, 6, 7, 8], [4, 4, 4, 4]);
        let _ = r.receive(RxFrame::Sid(sid));
        assert_eq!(r.state(), DtxState::ComfortNoise);
        let _ = r.receive(RxFrame::Speech(speech_frame()));
        assert_eq!(r.state(), DtxState::Speech);
        assert!(r.has_sid());
    }

    /// A second SID *updates* (does not re-open) the period and is ramped
    /// from the value currently in effect (smooth §6.1 transition).
    #[test]
    fn second_sid_updates_with_ramp() {
        let mut r = DtxReceiver::new(17);
        let first = SidParameters::new([0; 9], [0, 0, 0, 0]);
        let second = SidParameters::new([0; 9], [80, 80, 80, 80]);
        let _ = r.receive(RxFrame::Sid(first));
        // First SID is in effect immediately.
        assert_eq!(r.sid().xmax_cr, [0, 0, 0, 0]);
        // The synthesise step inside receive() advanced the ramp once,
        // but the first SID had no ramp (snapped), so it stays settled.
        let _ = r.receive(RxFrame::Sid(second));
        // The update ramps from 0 toward 80 over DEFAULT frames; the SID
        // synthesised in this very call already advanced the ramp once,
        // so the value in effect for the *next* frame is 20 (80*1/4).
        assert_eq!(r.sid().xmax_cr, [20, 20, 20, 20]);
    }

    /// NoData before any SID is the out-of-06.12-scope fallback: state
    /// stays Speech, output is well-formed 160 samples, no panic.
    #[test]
    fn no_data_before_sid_is_safe_fallback() {
        let mut r = DtxReceiver::new(19);
        let pcm = r.receive(RxFrame::NoData);
        assert_eq!(pcm.len(), FRAME_SAMPLES);
        assert_eq!(r.state(), DtxState::Speech);
        assert!(!r.has_sid());
        // §5.3.7 output format still holds on the fallback path.
        for s in pcm {
            assert_eq!(s & 0b111, 0);
        }
    }

    /// reset() homes the decoder, drops the SID, and returns to Speech.
    #[test]
    fn reset_returns_to_speech_and_drops_sid() {
        let mut r = DtxReceiver::new(21);
        let sid = SidParameters::new([0, 1, 2, 3, 4, 5, 6, 7, 8], [3, 3, 3, 3]);
        let _ = r.receive(RxFrame::Sid(sid));
        for _ in 0..3 {
            let _ = r.receive(RxFrame::NoData);
        }
        r.reset();
        assert_eq!(r.state(), DtxState::Speech);
        assert!(!r.has_sid());
        assert_eq!(r.sid(), SidParameters::default());
        assert!(r.decoder.is_home_state());
    }

    /// Two receivers with the same seed driven by the same RxFrame
    /// sequence produce bit-identical PCM (deterministic comfort noise).
    #[test]
    fn receiver_is_deterministic_for_fixed_seed() {
        let seq = || {
            vec![
                RxFrame::Speech(speech_frame()),
                RxFrame::Sid(SidParameters::new(
                    [0, 9, 23, 15, 8, 7, 3, 3, 2],
                    [6, 6, 6, 6],
                )),
                RxFrame::NoData,
                RxFrame::NoData,
                RxFrame::Sid(SidParameters::new(
                    [0, 5, 10, 8, 6, 4, 3, 2, 1],
                    [10, 10, 10, 10],
                )),
                RxFrame::NoData,
                RxFrame::Speech(speech_frame()),
            ]
        };
        let mut a = DtxReceiver::new(123);
        let mut b = DtxReceiver::new(123);
        for (fa, fb) in seq().into_iter().zip(seq()) {
            assert_eq!(a.receive(fa), b.receive(fb));
        }
    }

    /// Filter continuity: the comfort-noise frame synthesised right after
    /// a speech burst uses the decoder memory the speech burst left
    /// behind. Driving a *fresh* decoder with the same §6.1 frame gives a
    /// different result, proving the shared-decoder memory carries over.
    #[test]
    fn comfort_noise_inherits_speech_decoder_memory() {
        // Receiver A: speech burst, then a SID.
        let mut r = DtxReceiver::new(31);
        for _ in 0..4 {
            let _ = r.receive(RxFrame::Speech(speech_frame()));
        }
        let sid = SidParameters::new([0, 9, 23, 15, 8, 7, 3, 3, 2], [8, 8, 8, 8]);
        let warm = r.receive(RxFrame::Sid(sid));

        // Reproduce the *same* §6.1 frame on a fresh (home-state) decoder.
        let mut rng = NoiseRng::new(31);
        let cn = comfort_noise_frame(&sid, &mut rng);
        let mut cold = DecoderState::new();
        let cold_pcm = cold.decode_frame(&cn);

        assert_ne!(
            warm, cold_pcm,
            "comfort noise ignored the carried-over speech decoder memory"
        );
    }

    // ----- §5.1 transmit-side background-acoustic-noise evaluation -----

    use crate::encoder::analysis::{code_xmax, quantise_lar};

    /// §5.1: `mean_round` is round-to-nearest, ties away from zero, and
    /// symmetric for negative LARs.
    #[test]
    fn mean_round_is_nearest_ties_away() {
        assert_eq!(mean_round(0, 4), 0);
        assert_eq!(mean_round(6, 4), 2); // 1.5 → 2 (tie up)
        assert_eq!(mean_round(5, 4), 1); // 1.25 → 1
        assert_eq!(mean_round(7, 4), 2); // 1.75 → 2
        assert_eq!(mean_round(-6, 4), -2); // tie away from zero
        assert_eq!(mean_round(-5, 4), -1);
        assert_eq!(mean_round(10, 4), 3); // 2.5 → 3
    }

    /// §5.1: an empty window has no defined mean.
    #[test]
    fn evaluate_empty_window_is_none() {
        let e = NoiseEvaluator::new();
        assert!(e.is_empty());
        assert!(!e.is_ready());
        assert_eq!(e.len(), 0);
        assert!(e.evaluate().is_none());
    }

    /// `NoiseFrameParameters::new` forces the 1-based sentinel slot 0.
    #[test]
    fn noise_frame_params_force_sentinel() {
        let np = NoiseFrameParameters::new([99, 1, 2, 3, 4, 5, 6, 7, 8], [10, 20, 30, 40]);
        assert_eq!(np.lar[0], 0);
        assert_eq!(&np.lar[1..=8], &[1, 2, 3, 4, 5, 6, 7, 8]);
        assert_eq!(np.xmax, [10, 20, 30, 40]);
    }

    /// §5.1 + §5.2: averaging four identical frames yields that frame's
    /// mean, which §5.2 re-encodes through `quantise_lar` / `code_xmax`.
    /// The single mean(xmax) codeword is replicated across all four slots.
    #[test]
    fn evaluate_identical_frames_is_quantised_identity() {
        let mut e = NoiseEvaluator::new();
        let lar = [0, 2000, -1500, 800, -400, 300, -200, 100, -50];
        let xmax = [4096, 4096, 4096, 4096];
        for _ in 0..NOISE_EVAL_FRAMES {
            e.push_frame(NoiseFrameParameters::new(lar, xmax));
        }
        assert!(e.is_ready());
        assert_eq!(e.len(), NOISE_EVAL_FRAMES);
        let sid = e.evaluate().unwrap();
        // §5.2: the mean (= the common value) is encoded, not stored raw.
        assert_eq!(sid.lar_cr, quantise_lar(&lar));
        let expect_xmax = code_xmax(4096) as u8;
        assert_eq!(sid.xmax_cr, [expect_xmax; SUBFRAMES]);
    }

    /// §5.1: each LAR(i) is averaged independently across the N=4 frames,
    /// on the *unquantised* values, before the §5.2 quantiser.
    #[test]
    fn evaluate_lar_averaged_per_coefficient() {
        let mut e = NoiseEvaluator::new();
        for k in 1..=NOISE_EVAL_FRAMES as i16 {
            let lar = core::array::from_fn(|i| if i == 0 { 0 } else { k * (i as i16) * 100 });
            e.push_frame(NoiseFrameParameters::new(lar, [0; SUBFRAMES]));
        }
        // mean over {100i, 200i, 300i, 400i} = 1000i/4 = 250i.
        let mut mean_lar = [0i16; 9];
        for (i, slot) in mean_lar.iter_mut().enumerate().skip(1) {
            *slot = mean_round(1000 * i as i32, 4) as i16;
        }
        let sid = e.evaluate().unwrap();
        assert_eq!(sid.lar_cr, quantise_lar(&mean_lar));
    }

    /// §5.1: mean(xmax) is a *single* mean over all 4·N = 16 sub-segment
    /// block maxima, not a per-sub-segment average. §5.2 then repeats the
    /// one encoded codeword across the four slots.
    #[test]
    fn evaluate_xmax_single_mean_over_all_subsegments() {
        let mut e = NoiseEvaluator::new();
        // frame maxima 1024/2048/3072/4096 (all four sub-segments each)
        // → 16 values, mean = (1024+2048+3072+4096)/4 = 2560.
        for v in [1024i16, 2048, 3072, 4096] {
            e.push_frame(NoiseFrameParameters::new([0; 9], [v; SUBFRAMES]));
        }
        let sid = e.evaluate().unwrap();
        assert_eq!(sid.xmax_cr, [code_xmax(2560) as u8; SUBFRAMES]);
    }

    /// §5.1: the window keeps only the four most recent VAD=0 frames;
    /// a fifth push evicts the oldest.
    #[test]
    fn window_evicts_oldest_beyond_n() {
        let mut e = NoiseEvaluator::new();
        // Push 5 frames; first (xmax=0) is evicted, leaving 1024/2048/
        // 3072/4096 → mean 2560.
        for v in [0i16, 1024, 2048, 3072, 4096] {
            e.push_frame(NoiseFrameParameters::new([0; 9], [v; SUBFRAMES]));
        }
        assert_eq!(e.len(), NOISE_EVAL_FRAMES);
        let sid = e.evaluate().unwrap();
        assert_eq!(sid.xmax_cr, [code_xmax(2560) as u8; SUBFRAMES]);
    }

    /// A partial window (fewer than N frames) averages only what is
    /// present — useful at the start of a silence period.
    #[test]
    fn evaluate_partial_window() {
        let mut e = NoiseEvaluator::new();
        e.push_frame(NoiseFrameParameters::new(
            [0, 400, 400, 400, 400, 400, 400, 400, 400],
            [1024; SUBFRAMES],
        ));
        e.push_frame(NoiseFrameParameters::new(
            [0, 800, 800, 800, 800, 800, 800, 800, 800],
            [3072; SUBFRAMES],
        ));
        assert!(!e.is_ready());
        let sid = e.evaluate().unwrap();
        // mean LAR = 600 for each; mean xmax = (1024+3072)/2 = 2048.
        assert_eq!(
            sid.lar_cr,
            quantise_lar(&[0, 600, 600, 600, 600, 600, 600, 600, 600])
        );
        assert_eq!(sid.xmax_cr, [code_xmax(2048) as u8; SUBFRAMES]);
    }

    /// `reset` clears the window so a new silence period restarts §5.1.
    #[test]
    fn reset_clears_window() {
        let mut e = NoiseEvaluator::new();
        e.push_frame(NoiseFrameParameters::new([0; 9], [2048; SUBFRAMES]));
        e.reset();
        assert!(e.is_empty());
        assert!(e.evaluate().is_none());
    }

    /// The §5.1 evaluator's `xmax` input comes from the §5.2.15 block
    /// maximum the encoder exposes on `ApcmQuantised::xmax`, and
    /// re-coding it reproduces the encoder's own `xmaxc`.
    #[test]
    fn xmax_field_matches_encoder_block_maximum() {
        use crate::encoder::analysis::apcm_quantise_rpe;
        let x_m = [
            120i16, -300, 50, 0, 700, -40, 12, -9, 333, -1000, 5, 88, -250,
        ];
        let apcm = apcm_quantise_rpe(&x_m);
        // The block maximum is max|xM[i]| = 1000.
        assert_eq!(apcm.xmax, 1000);
        // Coding that block maximum reproduces the encoder's xmaxc.
        assert_eq!(code_xmax(apcm.xmax), apcm.xmaxc);
    }

    /// End-to-end §5.1 → §5.2 → §6.1: a SID evaluated on the transmit
    /// side drives the receive-side comfort-noise generator, and every
    /// frame it emits packs legally through §1.7.
    #[test]
    fn transmit_eval_feeds_receive_generator() {
        let mut e = NoiseEvaluator::new();
        for k in 0..NOISE_EVAL_FRAMES as i16 {
            let lar = core::array::from_fn(|i| if i == 0 { 0 } else { (i as i16) * 200 + k * 30 });
            e.push_frame(NoiseFrameParameters::new(lar, [(k + 1) * 1024; SUBFRAMES]));
        }
        let sid = e.evaluate().unwrap();
        // The §5.2-encoded codewords must already be legal §1.7 fields.
        let mut g = ComfortNoiseGenerator::new(sid, 7);
        for _ in 0..8 {
            let f = g.next_frame_parameters();
            let bytes = f.to_bit_stream_msb_first();
            let back = UnpackedFrame::from_bit_stream_msb_first(&bytes).unwrap();
            assert_eq!(
                f, back,
                "transmit-derived comfort-noise frame did not round-trip"
            );
        }
    }
}
