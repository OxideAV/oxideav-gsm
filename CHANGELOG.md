# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased]

### Added

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
