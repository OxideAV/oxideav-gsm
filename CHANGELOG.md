# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased]

### Added

- **GSM 06.12 §6 receive-side DTX dispatcher `DtxReceiver`
  (2026-06-20).** Implements the receive-side Discontinuous-Transmission
  dispatch the staged EN 300 963 §6 fully defines: comfort-noise
  generation *"is started or updated whenever a valid SID frame is
  received"*, continued across the radio cut *"at a regular low rate
  afterwards"* (§4), and ended when speech resumes. New public types:
  `RxFrame` (a classified receive frame — `Speech` / `Sid` / `NoData`),
  `DtxState` (`Speech` | `ComfortNoise`), and `DtxReceiver`. The
  dispatcher holds **one** shared `DecoderState` across speech decode
  and §6.1 comfort-noise synthesis, so the §5.3 synthesis lattice,
  §5.3.2 long-term delay line, and §5.3.5 de-emphasis memory carry
  continuously across the speech ⇄ noise boundary — the cross-transition
  continuity §4 needs to avoid the *"very annoying"* background-noise
  modulation. `receive(RxFrame)` dispatches one frame; `receive_stream`
  drives a whole sequence; `RxFrame::speech_from_bytes` bridges a 33-byte
  §1.7 frame into the dispatcher. Frame *classification* (valid-SID
  detection, SID/speech/no-data scheduling) is left to the caller, as
  `NoiseEvaluator` leaves the VAD mark to the caller — the underlying
  detectors are the documented spec gaps (GSM 05.03 table 2 + GSM 06.31,
  neither staged). 18 new tests (14 unit + 4 integration in
  `tests/dtx_comfort_noise.rs`) cover the state machine, speech ==
  bare-decoder PCM, SID open/update-with-ramp, no-data continuation,
  shared-decoder filter continuity, byte bridge, stream driver, and the
  full §5.1 → §5.2 → §6.1 → §6 transmit-to-receive loop.

- **GSM 06.12 §5.1 + §5.2 transmit-side comfort-noise evaluation
  (2026-06-20).** New `NoiseEvaluator` implements the §5.1 background-
  acoustic-noise averaging *plus* the §5.2 parameter encoding that
  builds a SID frame's noise parameters over the `N = 4` most recent
  VAD = 0 frames. Per §5.1 the means are taken on the **unquantised**
  parameters — *"the unquantized block amplitude and Log Area Ratio
  (LAR) parameters … defined in 4.2.15 and 4.2.6"*:
  `mean(LAR(i)) = (1/N)·Σ LAR[j−n](i)` for the eight §5.2.6 LARs (each
  averaged independently) and the single
  `mean(xmax) = (1/(4N))·Σ_n Σ_i xmax[j−n](i)` over all `4·N = 16`
  §5.2.15 sub-segment block maxima. §5.2 then encodes those means *"as
  described in GSM 06.10"*: the mean LARs through the §5.2.7
  `quantise_lar` quantiser and the single mean block amplitude through
  the new §5.2.15 `code_xmax` coder (factored out of `apcm_quantise_rpe`
  and now exposed), replicated across the four `SidParameters::xmax_cr`
  slots per §5.2's *"repeated four times"*. Frames feed in via
  `push_frame(NoiseFrameParameters::new(lar, xmax))` — `lar` the
  encoder's §5.2.6 `analyse_frame` output, `xmax` the four sub-segments'
  §5.2.15 `ApcmQuantised::xmax` block maxima (a new field on that
  struct) — into a fixed 4-slot ring that evicts the oldest beyond `N`;
  `evaluate` emits the `SidParameters` the existing §6.1
  `ComfortNoiseGenerator` consumes, closing the §5.1 → §5.2 → §6.1
  transmit-to-receive comfort-noise parameter loop end-to-end. The
  means are exact integer averages; the staged PDF leaves the final-
  division rounding direction unstated, so round-to-nearest (ties away
  from zero, symmetric for the signed LARs) is used — the only quantity
  left to implementation choice, on the same footing as the receive-side
  `NoiseRng`. New public surface: `NoiseEvaluator`,
  `NoiseFrameParameters`, `NOISE_EVAL_FRAMES`, `analysis::code_xmax`,
  `ApcmQuantised::xmax`. 11 new comfort-noise tests cover the rounding
  helper, identity/partial/full windows, per-coefficient unquantised LAR
  averaging, the single-mean xmax semantics, oldest-frame eviction, the
  `ApcmQuantised::xmax` ↔ `code_xmax` consistency, and an end-to-end
  §5.1→§5.2→§6.1 round-trip. Still docs-blocked (unstaged): the VAD flag
  (GSM 06.32), the §5.2 SID-frame *bit* layout incl. the class-I 95-bit
  code word (GSM 05.03 table 2), and DTX scheduling (GSM 06.31).

- **§6.3.2 Table 6.7 decoder-overflow saturation pin (2026-06-19).**
  Table 6.7 ("List of tested overflows points for sequence 1 (decoder
  part)") records that the §5.3.6 output-scaling `add(sro[k], sro[k])`
  overflows **16691 times** in conformance sequence 1 — so the doubling
  must use the §5.1 *saturating* `add`, not a wrapping double. A new
  `upscaling_saturates_per_table_6_7` test drives `post_process` past
  the i16 ceiling in both directions: `sro = 20000` clamps to 32767
  (→ §5.3.7-shaped 32760), `sro = -20000` clamps to -32768, and a
  no-overflow control confirms ordinary samples are unaffected. A
  wrapping double would sign-flip (`40000 as i16 = -25536`) — the exact
  corruption Table 6.7's overflow handling exists to prevent.

- **§6.3.2 Table 6.5 spec-boundary conformance pins (2026-06-19).**
  Table 6.5 ("Errors specially detected by sequence 4/Config 1") states,
  for each critical breakpoint, the *incorrect* statement next to the
  *correct* one the SEQ04 conformance vector flushes out. Five new tests
  pin the correct boundary as an executable assertion so any regression
  at a spec-named pitfall fails loudly even without the (unstaged)
  SEQ04 corpus:
  - §5.2.9.2 / §5.3.3 `larp_to_rp` segment breakpoints — `11059` (not
    `11058`) and `20070` (not `20069`), on both the positive and
    negative sides. These are the decoder-side reflection-coefficient
    reconstruction boundaries.
  - §5.2.5 reflection-coefficient abort test — the **strict** `P[0] <
    abs(P[1])` (not `<=`): the boundary case `P[0] == abs(P[1])` must
    *not* abort, yielding `r[1] = -32767` (Q15 unity) rather than `0`.
  - §5.2.4 autocorrelation loop range — `k = 0..=159` (not `0..=158`):
    `s[159]` must contribute to `L_ACF[0]`.
  Pre-existing tests already covered the encoder-side §5.2.6 segment
  boundaries; these extend the coverage to the decoder-side §5.2.9.2
  inverse and the §5.2.4/§5.2.5 ranges/comparisons Table 6.5 enumerates.

- **End-to-end §6.2/§6.3.3.1 codec-homing conformance harness
  (`tests/conformance_homing.rs`) (2026-06-19).** Reconstructs the two
  digital test vectors the staged EN 300 961 PDF fully defines —
  `SEQ06H.INP` (§6.3.3.1: "one encoder-homing-frame", §4.2 160 × `0x0008`)
  and `SEQ06H.COD` (§6.3.3.1: "one decoder-homing-frame", §4.4 Table
  4.1a/b) — and drives them through the **public registry adapters**
  (`make_encoder` / `make_decoder` + `Packet`/`Frame`), exercising the
  whole §5.2 encode → §1.7 pack → §1.7 unpack → §5.3 decode chain at the
  byte level rather than only at the internal struct boundary the
  in-crate unit tests cover. Nine integration tests pin:
  - **§6.2.1 Configuration 1 (encoder under test)** — from the §4.5 home
    state the encoder maps `SEQ06H.INP` bit-exactly to `SEQ06H.COD`; and
    the "first frame undefined, subsequent frames defined" property holds
    over a five-frame repeated-homing stream.
  - **§6.2.2 Configuration 2 (decoder under test)** — `SEQ06H.COD`
    decodes to the encoder-homing-frame (160 × `0x0008`); the "two
    leading homing frames define subsequent output" property is shown
    history-independent (a clean decoder and a decoder pre-soiled with a
    noisy frame produce identical post-homing speech samples); the
    §6.3.3.2 fractional-homing-frame reset works through the `Packet`
    adapter; and a multi-frame coded stream decodes to §5.3.7-shaped
    output with correct `NeedMore` discipline.
  - **§4.1 loop-back** — `SEQ06H.INP` → encode → `SEQ06H.COD` → decode →
    `SEQ06H.INP` closes the circle through both registry adapters with
    byte-exact endpoints. These are the only §6 conformance vectors that
    are fully specified inside the staged PDF; the bulk SEQ01..SEQ05
    `*.INP`/`*.COD`/`*.OUT` corpus ships in the unstaged ETSI conformance
    archive and remains a docs-staging followup.

- **Analysis-by-synthesis bit-exactness cross-check (§5.2.16..§5.2.18
  vs §5.3.2) (2026-06-18).** Pins the encoder's load-bearing
  correctness invariant: the local-decoder LTP delay line
  `dp[-120..=-1]` the §5.2.16..§5.2.18 feedback loop reconstructs must
  be **bit-identical** to the receiving decoder's §5.3.2 `drp[-120..=-1]`
  after every frame. If they diverged by even one LSB the §5.2.11
  cross-correlation lag search of the next sub-segment would optimise
  against a history the decoder never reconstructs and the coded stream
  would drift. The new
  `local_decoder_dp_history_matches_decoder_drp_history_every_frame`
  test encodes 12 frames each of five signal classes (silence, periodic
  triangle, broadband two-tone-ish ramp mix, alternating loud square,
  pseudo-random), carries each output over the real §1.7 bitstream into
  a decoder, and asserts the two 120-sample histories are equal at every
  frame — exercising non-zero `bc`, all four sub-segments, and many
  frames of state carry-over. (Earlier coverage pinned only the
  `bc = 0`, first-sub-segment, home-state case.)
  - `local_decoder_history_stays_locked_across_homing` extends the
    invariant through a complete §4.3/§4.4 homing event, documenting the
    spec's asymmetric reset timing: a single homing frame homes the
    encoder (§4.3 Step 2) but not the decoder — per the §4.3 NOTE the
    decoder only sees a genuine decoder-homing-frame on the *second*
    consecutive homing frame — so the histories legitimately differ for
    one frame, then re-lock once both reach the all-zero home state.
  - New `pub(crate)` test-only accessors `EncoderState::ltp_dp_hist`
    and `DecoderState::drp_hist` expose the two delay lines (both in the
    `dp_hist[0] = dp[-1]` newest-first layout) for the cross-check; no
    public-API surface added.

- **GSM 06.12 §6.1 comfort-noise update-smoothing interpolation
  (2026-06-17).** Implements the §6.1 sentence *"when updating the
  comfort noise, the parameters above should preferably be interpolated
  over a few frames to obtain smooth transitions."* On `update_sid()`
  the new `SidInterpolator` linearly ramps the SID-carried LAR codewords
  (`LARcr[1..=8]`) and the four block amplitudes (`Xmaxcr[1..=4]`) from
  their in-effect values to the newly-received ones over
  `DEFAULT_INTERPOLATION_FRAMES` (= 4) frames, so the comfort-noise level
  and spectrum transition gradually instead of stepping abruptly (the §4
  "modulation of the background noise" the smoothing exists to avoid).
  - Only the SID-carried parameters are ramped — the §6.1 random RPE
    pulses / grid positions and the fixed `Ncr`/`bcr=0` carry no old→new
    transition.
  - The 4-frame default is the spec's open *"a few frames"* implementation
    choice (matched to the §5.1 `N = 4` transmit-averaging window);
    `ComfortNoiseGenerator::set_interpolation_frames` overrides it, and a
    length of `0` reproduces the previous plain immediate replacement.
  - A mid-ramp `update_sid()` re-bases the ramp from the in-flight
    interpolated value, so chained SID updates stay smooth.
  - `ComfortNoiseGenerator::sid()` now returns the interpolated
    parameters in effect for the next frame (by value); `is_settled()`
    reports whether the ramp has reached the latest SID frame.
  This corrects an earlier note that mislabelled §6.1 update-smoothing as
  a GSM 06.31 concern: 06.31 governs only the *scheduling* of when a
  valid SID frame arrives; the smoothing instruction is in 06.12 §6.1
  itself and is now implemented.

- **GSM 06.12 §6.1 receive-side comfort-noise generation (2026-06-16).**
  New `comfort_noise` module implementing the receive-side comfort-noise
  synthesis of ETSI EN 300 963 (GSM 06.12), the DTX silence-descriptor
  noise generator that masks the abrupt loss of background acoustic
  noise when the radio link is cut. Per §6.1 the standard GSM 06.10
  §5.3 speech decoder is driven with a frame whose parameters are
  substituted: RPE pulses → random integers in [1, 6], grid positions
  → random in [0, 3], LTP gains `bcr` → 0, LTP lags `Ncr` → 40/120/40/
  120, and the LARs + four block amplitudes → the values received in the
  SID frame.
  - `SidParameters` — the SID-frame LAR + block-amplitude codewords in
    `UnpackedFrame` codeword form.
  - `NoiseRng` — deterministic LCG source for the §6.1 "locally
    generated random integer sequence" (the generator is an
    implementation choice; the spec only requires uniform draws).
  - `comfort_noise_frame(sid, rng) -> UnpackedFrame` — builds the §6.1
    substituted frame.
  - `ComfortNoiseGenerator` — wraps a `DecoderState` + SID params + RNG;
    `generate_frame()` synthesises one 20 ms comfort-noise frame,
    `update_sid()` applies a freshly received SID frame, `reset_decoder()`
    homes the wrapped decoder.
  - `COMFORT_NOISE_NCR` = `[40, 120, 40, 120]`.
  The §5 transmit side (background-noise evaluation + SID-frame
  *encoding*) and receive-side "valid SID frame" detection depend on
  GSM 06.32 (VAD), GSM 05.03 (channel-coding class-I bit positions of
  the 95-bit SID code word), and GSM 06.31 (DTX scheduling /
  interpolation), none of which are staged — those remain a docs gap.

- **§6.3.3.3 encoder-framing bit synchronization (2026-06-14).** When
  the encoder is tested as a black box, the 20 ms framing and the
  13-bit-word bit alignment of its input are unknown. The new `sync`
  module implements the spec's bit-synchronization step: sweep the 13
  candidate bit alignments, feeding three §4.2 encoder-homing-frames at
  each, and take the first alignment that produces the §4.4
  decoder-homing-frame at the encoder output as bit sync.
  - `run_bit_sync_trial(shift, &[frame; 3]) -> BitSyncTrial` — one trial.
  - `find_bit_sync(|shift| -> [frame; 3]) -> Option<usize>` — sweep the
    `BIT_SYNC_TRIALS` (= `PCM_WORD_BITS` = 13) shifts.
  - `SyncFormats` — §6.3.3.4 reference-sequence sizes (`BITSYNC.INP`,
    `SEQSYNC.INP`, `SYNCxxx.COD`) as compile-time constants for a future
    round that stages the (currently unstaged) ETSI conformance archive.
  The §6.3.3.3 *frame*-synchronization sweep (the `SYNCxxx.COD` corpus)
  remains deferred until that archive is staged.

- **§4.4 NOTE 2 / §6.3.3.2 delay-optimised partial decoder-homing
  detection (2026-06-14).** Once the decoder is already in its §4.6
  home state, a subsequent frame only needs to carry the
  decoder-homing-frame's LARs and first sub-frame — sub-frames 2..=4
  may hold arbitrary data — to trigger the same encoder-homing-frame
  substitution + state reset. This is the criterion the §6.3.3.2
  `HOMING01` conformance sequence exercises (two complete homing
  frames followed by a mixture of complete and *fractional* homing
  frames, each of which must still home the decoder), and the
  "delay-optimized implementation in the TRAU uplink direction" §4.4
  NOTE 2 calls out.
  - `is_partial_decoder_homing_frame(frame) -> bool` — the §4.4
    NOTE 2 predicate: matches `LARc[1..=8]` + `sub[0]` against the
    decoder-homing-frame, ignoring sub-frames 2..=4.
  - `DecoderState::is_home_state() -> bool` — true when every §4.6
    Table 4.3 state variable (`nrp`, `drp[-120..=-1]`,
    `LARpp(j-1)[1..=8]`, `v[0..=8]`, `msr`) holds its home value.
    This is the soundness precondition the partial check requires.
  - `DecoderState::decode_frame_with_homing` now applies the cheaper
    partial check when `is_home_state()`, and the full-frame
    `is_decoder_homing_frame` check otherwise (a non-homed decoder
    must still receive a *complete* homing frame before it resets, as
    §4.4 NOTE 2's soundness argument only holds from the home state).
  - `is_partial_decoder_homing_frame` is re-exported from the crate
    root.
  Four unit tests pin: `is_home_state` tracking reset/decode; the
  partial predicate ignoring later sub-frames while rejecting LAR /
  first-sub-frame perturbations; a fractional homing frame homing an
  already-homed decoder; and the soundness boundary — a fractional
  homing frame fed to a *dirty* (non-home) decoder is decoded as
  ordinary speech and does NOT home it, while the subsequent complete
  homing frame does.

- **§4.3 encoder homing (2026-06-12).**
  - `is_encoder_homing_frame` — §4.2 predicate for the
    encoder-homing-frame (160 identical samples of `0x0008`, the
    13-bit value with only its least significant bit set, written
    left-justified into a 16-bit word).
  - `EncoderState::encode_frame_with_homing` — §4.3 protocol: the
    frame encodes through the normal §5.2 pipeline (no output
    substitution on the encode side), then, if the input was the
    encoder-homing-frame, every §4.5 Table 4.2 state variable is
    reset to its home value so the next frame starts from the home
    state. Wired into the `oxideav_core::Encoder` adapter behind
    `make_encoder`, mirroring the §4.4 decoder-side homing already
    applied behind `make_decoder`.
  - Tests pin the §4.3 Step 1 construction sentence — from the home
    state the encoder-homing-frame encodes **bit-exactly** to the
    §4.4 Table 4.1a/b decoder-homing-frame, a spec-supplied
    conformance vector covering the whole §5.2 pipeline — plus the
    §4.3 NOTE "N homing frames in → N−1 decoder-homing-frames out"
    property, the home-state reset (post-homing encode equals a
    fresh encoder bit-for-bit), and the §4.1 loop-back interplay
    (encoder homing output → §1.7 bitstream → §4.4 homing decoder →
    encoder-homing-frame again).

### Documentation

- **`docs/audio/gsm/etsi-gsm-06.12-comfort-noise.pdf` staging
  erratum (2026-06-12).** The staged PDF is, per its own title page
  and clause structure, ETSI EN 300 969 V8.0.1 (2000-11) *Half rate
  speech; Half rate speech transcoding (GSM 06.20)* — not the GSM
  06.12 full-rate comfort-noise spec the filename suggests. It
  contains no comfort-noise/SID/DTX material, so comfort-noise
  support remains docs-blocked; the README spec-reference list now
  records this.

- **§1.7 frame packer + frame-level encoder driver + `make_encoder`
  (2026-06-11).** Completes the encode path end-to-end:
  - `UnpackedFrame::to_bit_stream_msb_first() -> [u8; 33]` packs the
    76 codewords into the 260-bit `b1..b260` stream per §1.7
    Table 1.1 — parameters in order of occurrence (eight LARc with
    widths 6/6/5/5/4/4/3/3, then four 56-bit sub-frames of `Nc`/`bc`/
    `Mc`/`xmaxc`/`xMc[0..=12]`), each parameter LSB-first within its
    bit range per the table's "(LSB-MSB)" column, `b1` in the MSB of
    byte 0. The 4 spare bits (b261..b264) of byte 32 stay zero. The
    exact mirror of `from_bit_stream_msb_first`; pack∘unpack and
    unpack∘pack identities are tested at both the struct and byte
    level.
  - `EncoderState` — the frame-level §5.2 driver aggregating
    `PreProcessor` (§5.2.0..§5.2.3), `analysis::Analyzer`
    (§5.2.4..§5.2.10), and `analysis::LtpAnalyzer`
    (§5.2.11..§5.2.18). `encode_frame(&[i16; 160]) -> UnpackedFrame`
    runs pre-processing, LPC analysis + LAR quantisation, then the
    four per-sub-segment LTP → weighting → RPE grid → APCM →
    local-decoder-feedback passes and emits the 76 codewords.
    `new`/`reset` restore the complete §4.5 Table 4.2 encoder home
    state in one call.
  - `make_encoder` — real `oxideav_core::Encoder` factory (mono S16
    8 kHz in, one 33-byte packet per 160-sample frame out, with
    sample-accurate pts/duration in the 1/8000 time base; input
    framing need not align to the 20 ms codec frame; `flush`
    zero-pads a trailing partial frame). The registry entry now
    advertises `encode = true` alongside the decoder. Tested with
    silence/loud-input codeword-range checks, reset-determinism,
    a ≥6 dB encode→decode roundtrip floor on a periodic signal, and
    a bit-level encode → pack → unpack → decode equivalence test.

- **§5.2.16 + §5.2.17 encoder APCM inverse + RPE grid positioning
  (2026-06-10).** Lands the encoder's local-decoder feedback path
  that closes the §5.2.18 LTP delay-line loop. Exposed as the
  stateless free function `analysis::apcm_inverse_and_position` plus
  the `analysis::LtpAnalyzer::reconstruct_and_update` method:
  - `apcm_inverse_and_position(x_mc: &[i16; 13], exp, mant, m_c)
    -> [i16; 40]` runs §5.2.16 (`temp1 = FAC[mant]; temp2 =
    sub(6, exp); temp3 = 1 << sub(temp2, 1)`, then per pulse
    `temp = sub((xMc[i] << 1), 7) << 12; temp = mult_r(temp1, temp);
    temp = add(temp, temp3); xMp[i] = temp >> temp2`) followed by
    §5.2.17 grid positioning (`ep[Mc + 3*i] = xMp[i]` in an
    otherwise-zero 40-sample buffer). Consumes the post-normalisation
    `(exp, mant)` pair §5.2.15 returns directly rather than
    re-deriving them from `xmaxc`, per the §5.2.15 "Keep in memory
    exp and mant for the following inverse APCM quantizer" note. The
    §5.2.16 arithmetic is bit-identical to the decoder's §5.3.1
    `decoder::rpe_decode` path.
  - `LtpAnalyzer::reconstruct_and_update(apcm, m_c, dpp) -> [i16; 40]`
    runs §5.2.16 → §5.2.17 → §5.2.18 in one call: reconstructs
    `ep[0..=39]`, then folds `add(ep[k], dpp[k])` back into the §4.5
    `dp[-120..=-1]` delay line via the existing
    `update_dp_after_subframe`. This is the encoder's local-decoder
    history; a `bc = 0` first-sub-segment test confirms the resulting
    `dp[..]` is bit-exact with the decoder's reconstructed residual.
  - With §5.2.16/§5.2.17 in place the per-sub-segment §5.2.11..§5.2.18
    LTP feedback loop is now closeable end-to-end; only the §1.7 frame
    packer remains before `make_encoder` can land.

- **§5.2.15 encoder APCM forward quantisation (2026-06-08).** Lands
  the §5.2.15 block-maximum + 3-bit pulse coding step that follows
  §5.2.14. Exposed as the stateless free function
  `analysis::apcm_quantise_rpe` and the new `analysis::ApcmQuantised
  { xmaxc, x_mc, exp, mant }` return type:
  - `apcm_quantise_rpe(x_m: &[i16; 13]) -> ApcmQuantised` runs the
    §5.2.15 pseudocode verbatim: find `xmax = max |xM[i]|`; pack
    the 6-bit `xmaxc` codeword via the six-step `temp = xmax >> 9`
    exponent search and the final
    `xmaxc = (xmax >> (exp+5)) + (exp << 3)` step (3-bit exp +
    3-bit mantissa); re-derive the decoded `(exp, mant)` pair
    (with the §5.2.15 `mant == 0 ⇒ exp = -4, mant = 15` short-
    circuit and the iterative normalisation that drops `mant` into
    8..=15 while decrementing `exp` up to three times); then
    direct-code each pulse via
    `temp = xM[i] << (6 - exp); temp = mult(temp, NRFAC[mant]);
    xMc[i] = (temp >> 12) + 4`.
  - Returns the 6-bit `xmaxc ∈ 0..=63` codeword + 13 × 3-bit
    `xMc[i] ∈ 0..=7` codewords the §1.7 frame packer (later round)
    emits as `Xmaxc[1..=4]` + `Xm[1..=4][0..12]` per Table 1.1.
  - Also returns the post-normalisation `(exp, mant)` pair so the
    §5.2.16 inverse APCM quantiser (later round) can consume them
    directly, per the spec's "Keep in memory exp and mant for the
    following inverse APCM quantizer" note. `mant ∈ 0..=7` after
    the closing `sub(mant, 8)` step indexes both `NRFAC[..]`
    (§5.2.15) and `FAC[..]` (§5.2.16).
  - Backed by §5.4 Table 5.5 `NRFAC[0..=7]` (already staged in
    `tables.rs` for the staging audit; the encoder now actually
    consumes it).
  - Re-exported from the crate root as `ApcmQuantised` for
    ergonomic access alongside the existing `LtpParameters` /
    `LtpAnalyzer` / `RpeGrid` re-exports.
  Eight unit tests cover the zero-input ⇒ `xmaxc = 0`, `(exp, mant)
  = (-4, 7)`, and all `xMc[i] = 4` centre-code invariant; the §1.7
  Table 1.1 codeword-range invariants (`xmaxc ∈ 0..=63`,
  `xMc[i] ∈ 0..=7`) across an i16 magnitude sweep; the §5.2.15
  determinism and statelessness contracts; the `(exp, mant)` range
  check across the peak-magnitude sweep (`exp ∈ -4..=6`,
  `mant ∈ 0..=7`); the §5.2.15 / §5.2.16 round-trip bound (the
  reconstructed `xMp[i]` recovered by an independently-written
  §5.2.16 helper lands within the per-exponent quantiser step of
  the input `xM[i]`); the `xmaxc` doubling-step lift (a 2× peak
  shifts `xmaxc` by exactly 8 — one exponent band); the `xmaxc`
  monotonicity across a 14-step peak sweep; and the sign symmetry
  check (negating every `xM[i]` keeps `xmaxc` / `(exp, mant)`
  identical and reflects the `xMc[i]` codes around the centre
  value 4 within ±1 LSB).
  `make_encoder` still returns `Unsupported` — §5.2.16 encoder-side
  APCM inverse, §5.2.17 RPE grid positioning, and §1.7 frame
  packing arrive in later rounds.

- **§5.2.14 encoder RPE grid selection (2026-06-07).** Lands the
  adaptive sample-rate decimation step that follows §5.2.13. Exposed
  as the stateless free function `analysis::select_rpe_grid` and the
  new `analysis::RpeGrid { m_c, x_m }` return type:
  - `select_rpe_grid(x: &[i16; 40]) -> RpeGrid` runs the §5.2.14
    pseudocode verbatim: for each grid offset `m = 0..=3`,
    accumulate `L_result = Σ L_mult(x[m+3i] >> 2, x[m+3i] >> 2)`
    for `i = 0..=12` through the saturating 32-bit `L_add`; the
    strict `L_result > EM` comparison keeps the lower `m` on ties
    (the entry-time `Mc = 0` default propagates when all four
    grids carry equal energy).
  - The chosen grid is then down-sampled by a factor of 3:
    `xM[i] = x[Mc + 3*i]` for `i = 0..=12`. `xM` is the 13-sample
    sequence the §5.2.15 APCM quantiser (a later round) consumes;
    `Mc` is the 2-bit codeword the §1.7 packer emits as one of
    `Mc[1..=4]` per Table 1.1.
  - Re-exported from the crate root as `RpeGrid` for ergonomic
    access alongside the existing `LtpParameters` / `LtpAnalyzer`
    re-exports.
  - The new constant `analysis::RPE_PULSES = 13` formalises the
    spec's 13-pulse RPE sequence length so later stages
    (§5.2.15..§5.2.17) don't have to repeat the literal.
  Seven unit tests cover the zero-input ⇒ `m_c = 0` invariant, the
  determinism of the stateless selector, the four single-impulse
  argmax cases (one impulse per grid-unique index — `x[0]`, `x[1]`,
  `x[2]`, `x[39]` — landing on grids 0, 1, 2, 3 respectively), the
  strict-`>` tie-break (equal-energy impulses on grid 1 and grid 2
  select grid 1), the down-sampling formula `xM[i] = x[Mc + 3*i]`
  on a varied input, an argmax check (place most of the energy on
  grid 2 and confirm it wins) against an independently-written
  reference computation, and statelessness across calls.
  `make_encoder` still returns `Unsupported` — §5.2.15..§5.2.17
  APCM quantisation + RPE grid positioning and §1.7 frame packing
  arrive in later rounds.

- **§5.2.13 encoder weighting filter (2026-06-05).** Lands the §5.2.13
  block-filter clause as the stateless free function
  `analysis::weighting_filter`:
  - `weighting_filter(e: &[i16; 40]) -> [i16; 40]` runs the §5.2.13
    pseudocode verbatim: build the 50-sample working array
    `wt[0..=49]` with 5 leading zeros, `wt[5..=44] = e[0..=39]`, and
    5 trailing zeros; for each `k = 0..=39` accumulate
    `L_result = 8192 + Σ L_mult(wt[k+i], H[i])` for `i = 0..=10`,
    apply two saturating doublings (`L_add(L_result, L_result)`
    twice, the spec's `×2` then `×4` scaling), and emit
    `x[k] = L_result >> 16`.
  - Backed by §5.4 Table 5.4 `H[0..=10]` (already staged in
    `tables.rs` for the staging audit; the encoder now actually
    consumes it).
  - The filter is stateless — no §4.5 Table 4.2 home state is
    added; each sub-segment starts the `wt[]` array fresh from
    `e[..]` with zero padding.
  - The output `x[0..=39]` is the input the §5.2.14 RPE grid
    selection step (a later round) consumes; the spec's diagram
    in §3.1 (figure 3.1) and the §5.2.13 → §5.2.14 chain in §5.2
    both feed `x[..]` directly into the grid-selection
    sub-procedure.
  Six unit tests cover the zero-input ⇒ zero-output invariant, the
  determinism of the stateless filter, the approximate odd-symmetry
  under input negation (±1 LSB tolerance for the `+8192` rounding
  bias), an end-positioned unit impulse at `e[0]` against an
  independently-derived spec-arithmetic expected response, the
  trailing-edge zero-pad correctness via an impulse at `e[39]`, and
  a stateless-across-calls check (the result of two successive calls
  on the same input is invariant to a different input being
  filtered in between). `make_encoder` still returns `Unsupported`
  — §5.2.14..§5.2.17 RPE selection + APCM quantisation + RPE grid
  positioning and §1.7 frame packing arrive in later rounds.

- **§5.2.11 / §5.2.12 / §5.2.18 encoder LTP analysis clause
  (2026-06-04).** Lands the Long-Term Prediction analysis stages
  as the `analysis::LtpAnalyzer` struct + free functions
  `analysis::ltp_parameters` and
  `analysis::long_term_analysis_filter`:
  - `ltp_parameters(d, dp_hist) -> LtpParameters` runs §5.2.11
    end-to-end on a 40-sample short-term residual sub-segment
    and the §4.5 `dp[-120..=-1]` delay line: dynamic input
    scaling (`scal = sub(6, norm(dmax << 16))` clamped to 0
    when `norm > 6` or `dmax == 0`), cross-correlation peak
    search over `lambda = 40..=120` driving `(Nc, L_max)`,
    `L_max` rescaling by `sub(6, scal)` (with §5.1 negative-
    shift fall-through), the `wt[k] = dp[k-Nc] >> 3` power
    accumulator `L_power = Σ L_mult(wt, wt)`, and the §5.2.11
    `bc` decision tree against Table 5.3a `DLB[..]` (`L_max <= 0
    ⇒ bc = 0`; `L_max >= L_power ⇒ bc = 3`; otherwise the first
    `bc ∈ {0, 1, 2}` with `R <= mult(S, DLB[bc])`).
  - `long_term_analysis_filter(d, dp_hist, params) ->
    (dpp, e)` runs §5.2.12: `bp = QLB[bc]`,
    `dpp[k] = mult_r(bp, dp[k-Nc])`, `e[k] = sub(d[k], dpp[k])`.
    Returns `dpp` alongside `e` since §5.2.18 needs both.
  - `LtpAnalyzer::analyse_subframe(d) -> (LtpParameters,
    dpp[0..=39], e[0..=39])` is the per-sub-segment driver
    using the persisted `dp[-120..=-1]` history.
  - `LtpAnalyzer::update_dp_after_subframe(ep, dpp)` lands
    §5.2.18: slide the older 80 dp-history entries leftward by
    40 and write `add(ep[k], dpp[k])` into the `dp[-40..=-1]`
    slot. The §4.5 home value `dp[-120..=-1] = 0` is the default
    state of `LtpAnalyzer::new()`.
  - `LtpAnalyzer::reset` returns to the §4.5 home state.
  - Re-exported as `oxideav_gsm::{LtpAnalyzer, LtpParameters}`.
  Fourteen unit tests cover the §5.2.11 zero-input ⇒ `(Nc=40,
  bc=0)` invariant, the lag-40 (minimum) and lag-80 (mid-range)
  cross-correlation peak detection on seeded delay-line content,
  the `bc=3` saturating-gain branch, the `Nc ∈ [40, 120]` /
  `bc ∈ [0, 3]` codeword-range invariant on pseudorandom inputs,
  the §5.2.12 zero-history pass-through (`e[k] = d[k]` regardless
  of `bc`), the maximum-gain residual-shrinking property, the
  `LtpAnalyzer` home-state equivalence with the free functions,
  reset round-trip, `analyse_subframe` non-mutating contract,
  determinism, and two §5.2.18 dp-update tests covering the
  newest-sample-first ordering (`dp_hist[0] = dp[-1]`) and the
  across-sub-segment slide behaviour. `make_encoder` still
  returns `Unsupported` — §5.2.13..§5.2.17 weighting filter +
  RPE selection + APCM quantisation and §1.7 frame packing
  arrive in later rounds.

- **§5.2.8..§5.2.10 encoder short-term analysis filter (2026-06-04).**
  Lands the §5.2 short-term analysis filtering clause as the
  `analysis::Analyzer` struct + `analysis::short_term_analysis_filter`
  free function:
  - `short_term_analysis_filter(s, rp, u, k_start, k_end)` runs the
    §5.2.10 8-stage lattice over `s[k_start..=k_end]` with the
    block's reflection coefficients `rp[1..=8]`, updating the
    persistent filter memory `u[0..=7]` (§4.5 Table 4.2 / §5.2.10
    "Initial value: u[0..7] = 0") in place. Inner loop is the
    spec's verbatim `temp = add(u[i-1], mult_r(rp[i], di)); di =
    add(di, mult_r(rp[i], u[i-1])); u[i-1] = sav; sav = temp;`
    sequence.
  - `Analyzer::analyse_frame(s) -> (LARc[1..=8], d[0..159])` runs
    §5.2.7 → §5.2.8 → §5.2.9.1 → §5.2.9.2 → §5.2.10 end-to-end on a
    pre-processed frame. It quantises + codes the LARs, decodes
    them back into `LARpp(j)` via the §5.2.8 path (so encoder and
    decoder see bit-identical reflection coefficients), then for
    each of the four §5.2.9.1 blocks (k = 0..=12, 13..=26,
    27..=39, 40..=159) interpolates `LARpp(j-1)` with `LARpp(j)`,
    converts to `rp[1..=8]` via §5.2.9.2, and runs the §5.2.10
    filter to produce that block's short-term residual `d[..]`.
    After the four blocks complete, `LARpp(j-1)` is updated to
    `LARpp(j)` for the next frame.
  - `Analyzer` persists `LARpp(j-1)[1..=8]` (§4.5 / §5.2.9.1
    "Initial value: LARpp(j-1)[1..8] = 0") and `u[0..=7]` (§4.5 /
    §5.2.10) across calls. `Analyzer::new()` / `Analyzer::reset()`
    return the §4.5 home state.
  - The §5.2.8 LAR decode, §5.2.9.1 LAR interpolation, and §5.2.9.2
    LARp → rp helpers — already implemented in `src/decoder.rs` for
    the §5.3 pipeline — are now `pub(crate)` so the encoder reuses
    the same code path. No behaviour change to the decoder.
  Ten unit tests cover the §5.2.10 identity behaviour under zero
  reflection coefficients, single-sample blocks, the DC-step
  high-pass shape with `rp[1] = 0.5`, the `Analyzer` home state
  invariant, the `reset` round trip, the zero-input → zero-residual
  invariant, end-to-end determinism, cross-frame state evolution,
  the §5.2.4..§5.2.7 LARc determinism across frames, and a
  block-0 split-vs-canonical equivalence check.
  `make_encoder` still returns `Unsupported` — §5.2.11..§5.2.18
  LTP / RPE / APCM and §1.7 frame packing arrive in later rounds.

- **§5.2.7 encoder LAR quantisation + coding (2026-06-03).** Adds the
  `analysis::quantise_lar` free function to the existing
  `encoder::analysis` sub-module, plus the end-to-end
  `analysis::analyse_and_quantise_frame` convenience wrapper:
  - `quantise_lar(LAR[1..=8]) -> LARc[1..=8]` runs the four-step
    §5.2.7 pseudocode: `temp = mult(A[i], LAR[i]); temp = add(temp,
    B[i]); temp = add(temp, 256); LARc[i] = temp >> 9;` followed by
    the §5.2.7 `MAC[i]` / `MIC[i]` bounds check and the final
    `sub(LARc[i], MIC[i])` that lifts the signed codeword into the
    unsigned 0..(MAC-MIC) range the §1.7 bit packer emits.
  - Backed by §5.4 Table 5.1 columns A / B / MIC / MAC which were
    already staged in `tables.rs` for the §5.2.8 decoder partner.
  - `analyse_and_quantise_frame(s)` runs §5.2.4 → §5.2.5 → §5.2.6 →
    §5.2.7 end-to-end on a pre-processed frame, giving the §1.7
    frame packer the `LARc[1..=8]` codeword array.
  Ten unit tests cover the per-index "centre" code for LAR=0 (one
  arithmetic value pinned per i), upper / lower saturation (the
  encoder lands on `MAC[i]-MIC[i]` and 0 respectively), Table 1.1
  bit-width compliance (every codeword fits its 3/4/5/6-bit field),
  the §5.2.7 / §5.2.8 round-trip bound (decode of a quantised LAR
  recovers a value within one quantiser step), per-index monotonicity,
  the sentinel-zero invariant at index 0, and the end-to-end
  determinism + range invariants on `analyse_and_quantise_frame`.
  `make_encoder` still returns `Unsupported` — §5.2.10 short-term
  analysis filter + §5.2.11..§5.2.18 LTP / RPE / APCM + §1.7 frame
  packer arrive in later rounds.

- **§5.2.4..§5.2.6 encoder LPC-analysis front-end (2026-06-03).**
  Adds the autocorrelation → Schur → LAR transform stages as the
  public `encoder::analysis` sub-module (re-exported as
  `oxideav_gsm::analysis`):
  - §5.2.4 `autocorrelation` — Dynamic-scaled inner-product
    `L_ACF[0..=8] = Σ s[i] * s[i-k]`. Scaling factor
    `scalauto = sub(4, norm(smax << 16))` whenever `smax > 0`;
    when `scalauto > 0` the input is downscaled by
    `mult_r(s[k], 16384 >> (scalauto-1))` before accumulating.
  - §5.2.5 `reflection_coefficients` — Schur recursion driving
    `r[n] = div(|P[1]|, P[0])` (Q15) with sign restored from
    `P[1] > 0`, the `L_ACF[0] == 0` short-circuit, and the
    early-exit branch when `P[0] < |P[1]|`.
  - §5.2.6 `reflection_to_lar` — Three-segment piecewise map
    of `|r[i]|` into `LAR[i]` (breakpoints 22118 / 31130) with
    the sign of `r[i]` restored. Inverse of the decoder's
    §5.2.9.2 lookup.
  - `analyse_frame` — Convenience wrapper running §5.2.4 →
    §5.2.5 → §5.2.6 end-to-end on a pre-processed frame.
  Twelve unit tests cover the §5.2.4 dynamic scaling, the
  §5.2.5 zero-L_ACF and abort branches, segment 1/2/3 boundaries
  in §5.2.6, end-to-end determinism, and the approximate
  even-symmetry property under input negation. `make_encoder`
  still returns `Unsupported` — §5.2.7 quantisation + §5.2.10
  short-term analysis filter + §5.2.11..§5.2.18 LTP/RPE/APCM +
  §1.7 frame packer arrive in later rounds.

- **§5.2.0..§5.2.3 encoder pre-processing pipeline (2026-06-02).**
  Lands the encoder's input-shaping stage as a public `PreProcessor`
  struct in `encoder::PreProcessor` (re-exported as
  `oxideav_gsm::PreProcessor`):
  - §5.2.1 `downscale_frame` — `>>3` then `<<2` per the §5.2.1
    pseudocode, dropping the three "don't care" LSBs of the §5.2.0
    input format and re-emitting at the bit-2 alignment the §5.2.2
    IIR consumes.
  - §5.2.2 `offset_compensation` — the high-pass IIR with extended
    (32-bit) precision recursive arm. `z1` / `L_z2` state are
    persisted across frames per §5.2.2 + §4.5 Table 4.2 home values.
  - §5.2.3 `pre_emphasis` — first-order FIR with coefficient
    `-28180 * 2^-15`. `mp` state persisted per §5.2.3 + §4.5
    Table 4.2 home values. Pairs symmetrically with the decoder's
    §5.3.5 de-emphasis (`+28180`).
  - `process_frame` — single-call convenience that runs the three
    stages end-to-end on a 160-sample input frame.
  - `reset` returns the pre-processor to §4.5 home values.
  Twelve unit tests cover monotone decay of the §5.2.2 IIR step
  response, cross-frame state persistence, determinism from the
  home state, sign preservation, and the §5.2.3 first-sample
  identity. The §5.2.4..§5.2.18 stages (LPC analysis, LTP, RPE,
  APCM, frame packing) arrive in later rounds; `make_encoder` still
  returns `Unsupported`.

- **§4.4 decoder-homing protocol + §5.1 `norm` / `div` primitives
  (2026-06-01).** Wires the §4.4 in-band homing protocol into the
  decoder: a §4.4 Table 4.1a/b decoder-homing-frame at the input
  produces the §4.2 encoder-homing-frame (160 samples of `0x0008`)
  at the output and resets the decoder's §4.6 state. Exposed as
  `DecoderState::decode_frame_with_homing` (raw `decode_frame`
  remains available for callers that want pre-protocol §5.3
  output) and wired into the `make_decoder` `oxideav_core::Decoder`
  adapter so codec-registry callers also get conformance-correct
  behaviour. Helpers `encoder_homing_frame_pcm()` and
  `is_decoder_homing_frame()` are public.
  Also adds the two remaining §5.1 arithmetic primitives —
  `norm(L_var1) -> i16` (count of left shifts to normalise) and
  `div(var1, var2) -> i16` (Q15 fractional integer division with
  the §5.1 `var2 >= var1 >= 0` contract). Neither is used by the
  §5.3 decoder pipeline yet; they're staged for the §5.2 encoder
  that arrives in a later round.

- **First clean-room decoder slice (2026-05-29).** Implements the GSM
  06.10 RPE-LTP §5.3 fixed-point decoder pipeline against the staged
  ETSI EN 300 961 V8.1.1 spec under `docs/audio/gsm/`. Covers:
  - §5.1 saturating arithmetic primitives (`add`, `sub`, `mult`,
    `mult_r`, `L_add`, `L_mult`, `abs`, signed-`shl`/`shr`).
  - §5.4 quantisation tables (Table 5.1 A/B/MIC/MAC, Table 5.2 INVA,
    Table 5.3a/b DLB/QLB, Table 5.4 H, Table 5.5 NRFAC, Table 5.6 FAC).
  - §1.7 Table 1.1 frame unpack for the 76-parameter 260-bit speech
    frame.
  - §5.2.8 LAR decode, §5.2.9.1 LAR interpolation, §5.2.9.2 LAR →
    reflection coefficient conversion.
  - §5.2.15/§5.2.16 APCM inverse quantisation and §5.2.17 RPE grid
    positioning.
  - §5.3.2 long-term synthesis filter with 120-sample history.
  - §5.3.4 8-stage short-term synthesis lattice.
  - §5.3.5 de-emphasis IIR + §5.3.6/§5.3.7 upscale and output
    truncation to the 13-bit `S.v.v.v.v.v.v.v.v.v.v.v.v.0.0.0`
    format.
  - §4.6 Table 4.3 home-state machinery + `decoder_homing_frame()`
    helper for §4.4 conformance prep.
- `oxideav_core::Decoder` adapter (`make_decoder` factory + workspace
  registry registration under codec id `"gsm"`).

### Changed

- Public API surface lands real `Decoder` paths; `Error::NotImplemented`
  is no longer the universal return.

### Removed

- Scaffold-only `Error::NotImplemented` placeholder code path
  (replaced by a working decoder).

## [0.0.8] - 2026-05-25

### Changed

- **Reset to orphan-rebuild scaffold.** The prior implementation was
  retired under the workspace clean-room policy: its source and
  licensing declared the codec was modelled on an external reference
  implementation, whose provenance the clean-room policy does not
  permit. All public APIs returned `Error::NotImplemented` pending a
  clean-room rebuild against a staged ETSI GSM 06.10 specification.
