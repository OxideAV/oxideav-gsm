# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased]

### Added

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
