# oxideav-gsm

[![CI](https://github.com/OxideAV/oxideav-gsm/actions/workflows/ci.yml/badge.svg)](https://github.com/OxideAV/oxideav-gsm/actions/workflows/ci.yml) [![crates.io](https://img.shields.io/crates/v/oxideav-gsm.svg)](https://crates.io/crates/oxideav-gsm) [![docs.rs](https://docs.rs/oxideav-gsm/badge.svg)](https://docs.rs/oxideav-gsm) [![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

Pure-Rust **GSM 06.10 RPE-LTP** speech codec — the original 13 kbit/s GSM
Full Rate voice codec (20 ms frames, 160 samples at 8 kHz mono).

## Status

| Direction | Coverage | Notes |
|-----------|----------|-------|
| Decoder   | §5.3 pipeline + §4.4 homing protocol (incl. §4.4 NOTE 2 / §6.3.3.2 partial detection) + §6.2/§6.3.3.1 conformance harness — **black-box validated sample-exact** (480 000+ samples identical to an independent decoder binary, including saturation-heavy extreme streams where a second decoder binary deviates) | Fixed-point pipeline (frame unpack, LAR decode, LAR interpolation, APCM inverse, RPE grid positioning, long-term + short-term lattice synthesis, de-emphasis, §5.3.7 output shaping) plus §4.4 decoder-homing-frame detection and substitution — both the full-frame check and the §4.4 NOTE 2 / §6.3.3.2 delay-optimised partial detection (LARs + first sub-frame only, valid from the home state). The §6.2 verification configurations now run end-to-end against the §6.3.3.1 SEQ06H homing vectors (the only §6 vectors fully defined in the staged PDF) through the public registry adapters, plus a §6.3.3.2 `HOMING01`-shaped state-machine harness (`tests/conformance_homing01.rs`) that reconstructs the spec's described mixture — two complete decoder-homing-frames then interleaved complete + fractional homing frames and speech — and pins the §4.4 / §4.4-NOTE-2 reset machine frame-by-frame (non-home homes only on complete; homed homes on fractional; every reset emits 160 × `0x0008`; post-reset output is history-independent). The §6.3.2 Table 6.5/6.6/6.7 spec-named boundary statements are pinned in full: the Table 6.5 mutation points (§5.2.7 `A[]`/`MAC[]`/`MIC[]`, §5.2.11 LTP-gain `mult`/`DLB[]`/opt-scaling range, §5.2.16 `FAC[]`, §5.2.4/§5.2.5/§5.2.9.2 ranges + comparisons), the Table 6.6 `smax == 0` / `L_ACF[0] == 0` paths, and all four Table 6.7 decoder-overflow `add` points (§5.3.2 / §5.3.4 / §5.3.5 / §5.3.6 saturation). The §6.3.2 SEQ05 out-of-range-`Nr` robustness ("the delay value Nr … takes in this sequence its value in [0,127]") is pinned: every `Nc ∈ [0,127]` is swept through §5.3.2 and the spec's "check the limits of Nr" rule verified (in-range used verbatim, out-of-range reuses the previous valid lag, never an OOB delay-line index). The §6.1 Table 6.1 **word-oriented conformance I/O format** is also implemented (`confio`): the right-justified 76-word `*.COD` frame (152 bytes) and the left-justified `*.INP`/`*.OUT` PCM frames (320 bytes), little- and big-endian, with a cross-format equivalence proof against the §1.7 in-band stream — so the ETSI `*.INP`/`*.COD`/`*.OUT` files can be read directly once staged. The bulk SEQ01..SEQ05 *binary* test sequences remain pending — they ship in the `en_300961v080101p0.ZIP` archive ETSI distributes alongside the PDF; staging it is a docs followup. |
| Encoder   | §5.2 pipeline complete (§5.2.0..§5.2.18 + §1.7 packer) + §4.3 homing + §6.3.3.3 bit-sync + frame-sync — **black-box validated bit-exact** (byte-for-byte against an independent encoder binary over 6 200+ frames: quiet/mid/loud sweeps, full-scale/max-slew/impulse extremes, clipped random noise; a checked-in 740-frame fixture corpus pins it in CI) | Full fixed-point encode path: pre-processing, LPC analysis (autocorrelation → Schur → LAR quantisation), short-term analysis lattice, per-sub-segment LTP analysis + long-term filter, weighting filter, RPE grid selection, APCM forward quantisation, the §5.2.16..§5.2.18 local-decoder feedback loop, and the §1.7 Table 1.1 frame packer. `make_encoder` returns a working `oxideav_core::Encoder` (mono S16 8 kHz in, 33-byte frames out) with §4.3 encoder homing applied. §6.3.3.3 encoder-framing synchronization is complete in **both** steps: **bit-synchronization** (`find_bit_sync` / `run_bit_sync_trial`) and **frame-synchronization** (`FrameSyncTable` / `find_frame_sync` / `retard_special_frame`) — the 160-position special-frame sweep that recovers the encoder's 20 ms retardation, reproducing the §6.3.3.3 construction and pinning the spec's "all 160 output frames different" invariant + the full retardation round-trip (parameterised on the caller's special frame, since the `SEQSYNC.INP` / `SYNCxxx.COD` *binary* corpus is unstaged). The §6 binary conformance sequences (SEQ01..SEQ05 + the `SYNCxxx.COD` reference bytes) remain unstaged; in their place the checked-in black-box fixture corpus (`tests/blackbox_fixtures.rs`) enforces byte-for-byte equality with an independent encoder binary in CI — see the black-box validation section. |
| Comfort noise (GSM 06.12) | §5.1 transmit evaluation + §5.2 parameter encoding + §6.1 receive generation + §6 receive-side DTX dispatch | **Transmit side** (`NoiseEvaluator`): the §5.1 `N = 4`-frame averaging of the **unquantised** parameters — per-coefficient `mean(LAR(i))` and the single `mean(xmax)` over all 16 sub-segment block maxima — then the §5.2 re-encoding *"as described in GSM 06.10"* (§5.2.7 `quantise_lar` + §5.2.15 `code_xmax`, replicated × 4) producing the `SidParameters` a SID carries. **Receive side** (`ComfortNoiseGenerator`): the §6.1 substitution (random RPE [1,6] / grid [0,3], `bcr = 0`, `Ncr = 40/120/40/120`, SID-carried LARs + block amplitudes) driving the §5.3 decoder, plus the §6.1 update-smoothing interpolation. **§6 DTX dispatch** (`DtxReceiver`): the receive-side two-state machine — speech frames decoded normally, a *valid SID* frame *"start[s] or update[s]"* comfort noise, `NoData` (radio cut / inter-SID gap) continues it *"at a regular low rate"* (§4), speech ends it — sharing **one** `DecoderState` across speech decode and §6.1 synthesis for click-free §4 cross-transition continuity. Frame *classification* is left to the caller (`RxFrame::Speech/Sid/NoData`), as `NoiseEvaluator` leaves VAD to the caller. Closes the §5.1 → §5.2 → §6.1 loop and the §6 dispatch. Still docs-blocked: the VAD flag (GSM 06.32), the §5.2 SID-frame *bit* layout incl. the class-I 95-bit code word (GSM 05.03 table 2), and DTX scheduling (GSM 06.31) — none staged. |

## Implementation

Built clean-room from ETSI **EN 300 961 V8.1.1 (2000-11)** — the published
form of GSM 06.10. The PDF is staged under `docs/audio/gsm/` and is the
sole source of every quantiser, table, arithmetic step, and pseudocode
line in the crate. There is no transcription from any reference
implementation.

The decoder is a direct translation of the §5.3 fixed-point pipeline:

* **§5.2.8** LAR decode: `LARpp[i] = 2 * mult_r(INVA[i], (LARc[i] + MIC[i]) * 1024 − 2*B[i])` using Table 5.1 (A, B, MIC, MAC) and Table 5.2 (INVA).
* **§5.2.9.1** LAR interpolation: 0.75/0.25, 0.50/0.50, 0.25/0.75, 1.0 mixes of LARpp(j−1)/LARpp(j) for the four sample windows.
* **§5.2.9.2** LAR → reflection coefficients via the three-segment piecewise lookup.
* **§5.2.16** APCM inverse quantisation with the §5.2.15 mantissa/exponent normalisation block. Table 5.6 (FAC) drives the dequant scaling.
* **§5.2.17** RPE grid positioning — drop the 13 dequantised pulses at Mc, Mc+3, …, Mc+36 in an otherwise-zero 40-sample buffer.
* **§5.3.2** Long-term synthesis filter — `drp[k] = erp[k] + mult_r(QLB[bc], drp[k − Nr])` with QLB from Table 5.3b and a 120-sample delay-line history.
* **§5.3.4** Short-term synthesis filter — 8-stage lattice over the interpolated reflection coefficients.
* **§5.3.5** De-emphasis — first-order IIR with coefficient `28180 * 2^-15`.
* **§5.3.6 / §5.3.7** Upscale + output truncation — double the sample and clear the low three bits per the 13-bit two's complement output format.

Arithmetic primitives (`add`, `sub`, `mult`, `mult_r`, `L_add`,
`L_mult`, `L_sub`, `abs`, signed-`shl`/`shr`, `norm`, `div`) are
saturating per §5.1 and live in `src/arith.rs`. `norm` and `div`
are not used by the §5.3 decoder pipeline but land alongside the
others so the §5.2 encoder slice that consumes them in a later
round has a stable, tested fixed-point surface to build on.

## Encoder pre-processing (§5.2.0..§5.2.3)

The encoder's input-shaping pipeline is implemented as
[`PreProcessor`] in `src/encoder.rs`. It maps a frame of 160 raw
PCM input samples `sop[0..159]` (§5.2.0 format
`S.v.v.v.v.v.v.v.v.v.v.v.v.v.x.x.x`) to the analysis-clause input
`s[0..159]`:

* **§5.2.1 `downscale_frame`** — clear the three "don't care" LSBs
  via `>>3 then <<2`, leaving a 13-bit-aligned sample.
* **§5.2.2 `offset_compensation`** — high-pass IIR with the spec's
  31×16-bit recursive arm split. `z1` (16-bit) and `L_z2` (32-bit)
  state are carried across frames per the §5.2.2 "Keep z1 and L_z2
  in memory" note + §4.5 Table 4.2 home value 0.
* **§5.2.3 `pre_emphasis`** — first-order FIR with coefficient
  `mult_r(mp, -28180)`. `mp` (16-bit) state carried across frames
  per the §5.2.3 "Keep mp in memory" note + §4.5 home value 0.
  Symmetric to the decoder's §5.3.5 `+28180` de-emphasis pole.

`PreProcessor::process_frame` runs §5.2.1 → §5.2.2 → §5.2.3 end-
to-end for callers who only want the pre-processing output.

## Encoder LPC analysis + LAR quantisation (§5.2.4..§5.2.7)

The LPC analysis front-end is implemented under the public
[`analysis`] sub-module of `src/encoder.rs`. It consumes the
[`PreProcessor`] output `s[0..159]` and emits the per-frame
`LAR[1..=8]` array, then optionally runs the §5.2.7 quantiser to
emit the `LARc[1..=8]` codewords the §1.7 bit packer consumes:

* **§5.2.4 `autocorrelation`** — Compute the dynamic-scaled
  inner product `L_ACF[0..=8] = Σ s[i] * s[i-k]` for `k = 0..=8`.
  The dynamic scaling computes `scalauto = sub(4, norm(smax << 16))`
  for the frame's max-magnitude `smax`, then applies
  `s[k] = mult_r(s[k], 16384 >> (scalauto-1))` whenever
  `scalauto > 0` to keep the accumulator from overflowing. The
  clause's closing "Rescaling of the array s[0..159]" step
  (`s[k] = s[k] << scalauto`, saturating per §5.1) is applied **in
  place** (`autocorrelation` takes `&mut s`): it is a *lossy*
  round-trip — the low `scalauto` bits round away — and the spec
  pipeline hands exactly this round-tripped array to the §5.2.10
  short-term analysis filter, so it is part of the bit-exact
  contract. (An earlier revision ran §5.2.10 on the pristine
  §5.2.3 output; that deviation only manifested on loud input,
  `smax ≥ 2048`, and was found by black-box byte-exact comparison.)
* **§5.2.5 `reflection_coefficients`** — Schur recursion driving
  `r[i] = div(|P[1]|, P[0])` (Q15) with sign restored from
  `P[1]`, plus the spec's early-exit branch when `P[0] < |P[1]|`
  zeroes the remaining `r[i]`. Includes the `L_ACF[0] == 0`
  short-circuit (all reflection coefficients zero).
* **§5.2.6 `reflection_to_lar`** — Piecewise map of `|r[i]|`
  into `LAR[i]`:
  - `|r| < 22118` → `LAR = |r| >> 1`
  - `22118 ≤ |r| < 31130` → `LAR = |r| - 11059`
  - `|r| ≥ 31130` → `LAR = (|r| - 26112) << 2`

  with the sign of `r[i]` restored at the end. This is the inverse
  of the decoder's §5.2.9.2 LARp → rp lookup; the segment-2/3
  breakpoint matches the decoder's `temp < 20070` boundary
  (20070 = 31129 - 11059, the encoder's segment-2/3 transition
  value).
* **§5.2.7 `quantise_lar`** — Scale, bias, round, bound, and shift
  the eight `LAR[i]` values into the `LARc[i]` codewords the §1.7
  bit packer emits:
  - `temp = mult(A[i], LAR[i])` — Q15 product against Table 5.1
    column A (`real_A[i] * 1024`).
  - `temp = add(temp, B[i])` — bias from Table 5.1 column B
    (`real_B[i] * 512`).
  - `temp = add(temp, 256); LARc[i] = temp >> 9` — Q9 round-half-up
    landing the signed code value.
  - Clamp to `[MIC[i], MAC[i]]` per Table 5.1, then
    `sub(LARc[i], MIC[i])` shifts the bound code into the unsigned
    0..(MAC-MIC) range the §1.7 bit packer holds. This is the
    inverse of the decoder's §5.2.8 `decode_lar` (driven by Table
    5.2 INVA).

`analyse_frame` runs §5.2.4 → §5.2.5 → §5.2.6 end-to-end and
returns `LAR[1..=8]`; `analyse_and_quantise_frame` extends the
chain through §5.2.7 and returns `LARc[1..=8]`. All four helpers
are stateless.

## Encoder short-term analysis filter (§5.2.8..§5.2.10)

The short-term analysis filtering clause is implemented as the
[`analysis::Analyzer`] struct and the
[`analysis::short_term_analysis_filter`] free function in
`src/encoder.rs`. [`Analyzer`] runs the full chain §5.2.7 →
§5.2.8 → §5.2.9.1 → §5.2.9.2 → §5.2.10 on a pre-processed input
frame `s[0..159]` and returns `(LARc[1..=8], d[0..159])` where
`d` is the short-term residual that §5.2.11 LTP analysis
consumes:

* **§5.2.8** — `decode_lar(LARc) → LARpp(j)` runs the spec's
  `LARpp[i] = 2 * mult_r(INVA[i], (LARc[i] + MIC[i]) * 1024 - 2*B[i])`
  loop so encoder and decoder see bit-identical reflection
  coefficients. (The §5.2.8 helper itself is shared with the
  decoder pipeline at `pub(crate)` scope.)
* **§5.2.9.1** — for each of the four blocks (k = 0..=12,
  13..=26, 27..=39, 40..=159) the interpolator mixes
  `LARpp(j-1)` and `LARpp(j)` per Table 3.2 weights
  0.75/0.25, 0.50/0.50, 0.25/0.75, 1.0. (Shared with the
  decoder at `pub(crate)` scope.)
* **§5.2.9.2** — each interpolated `LARp` is converted to `rp`
  via the inverse of §5.2.6's piecewise map (segment points
  11059 / 20070). (Shared with the decoder.)
* **§5.2.10 `short_term_analysis_filter`** — runs the 8-stage
  lattice over `s[k_start..=k_end]`:
  ```text
  FOR k = k_start to k_end:
      di = s[k]; sav = di;
      FOR i = 1 to 8:
          temp = add(u[i-1], mult_r(rp[i], di));
          di   = add(di,     mult_r(rp[i], u[i-1]));
          u[i-1] = sav; sav = temp;
      NEXT i:
      d[k] = di;
  NEXT k:
  ```
  `u[0..=7]` is the §4.5 Table 4.2 short-term analysis filter
  memory and is persisted across blocks and across frames by the
  [`Analyzer`] caller.

`Analyzer::new()` returns the §4.5 home state (`LARpp(j-1) = 0`,
`u = 0`). `Analyzer::reset()` returns to the home state.

## Encoder LTP clause (§5.2.11..§5.2.12, §5.2.18)

The Long-Term Prediction clause is implemented as the
[`analysis::LtpAnalyzer`] struct + two free functions:
[`analysis::ltp_parameters`] (§5.2.11) and
[`analysis::long_term_analysis_filter`] (§5.2.12). Per sub-segment
(40 samples), the analyser consumes `d[0..=39]` (the §5.2.10
short-term residual for that sub-segment) and emits
`(LtpParameters, dpp[0..=39], e[0..=39])`:

* **§5.2.11 `ltp_parameters`** — Compute the LTP parameters
  `(Nc, bc)`:
  - Search for the optimum scaling of `d[0..=39]`:
    `scal = sub(6, norm(dmax << 16))` clamped to 0 when
    `norm > 6` or `dmax == 0`.
  - `wt[k] = d[k] >> scal`; then for each candidate lag
    `lambda = 40..=120` accumulate
    `L_result = Σ L_mult(wt[k], dp[k-lambda])` and track the
    maximum `L_max` with its `Nc = lambda` argmax. The §4.5
    delay line `dp[-120..=-1]` is held in `LtpAnalyzer`.
  - Rescale: `L_max >>= sub(6, scal)` (the §5.1 negative-shift
    rule applies when `scal > 6`).
  - `wt[k] = dp[k-Nc] >> 3`, then `L_power = Σ L_mult(wt[k],
    wt[k])`.
  - `bc` decision tree: `L_max <= 0 ⇒ bc = 0`;
    `L_max >= L_power ⇒ bc = 3`; otherwise
    `temp = norm(L_power); R = (L_max << temp) >> 16;
    S = (L_power << temp) >> 16;` and the first
    `bc ∈ {0, 1, 2}` for which `R <= mult(S, DLB[bc])` (Table
    5.3a) is the codeword, else `bc = 3`.
* **§5.2.12 `long_term_analysis_filter`** — Decode `bp =
  QLB[bc]` (Table 5.3b) and compute, for `k = 0..=39`:
  `dpp[k] = mult_r(bp, dp[k-Nc])`,
  `e[k] = sub(d[k], dpp[k])`. `dpp[..]` is returned alongside
  `e[..]` because §5.2.18 needs it to update the delay line.
* **§5.2.18 `LtpAnalyzer::update_dp_after_subframe`** — Once
  the §5.2.17 reconstructed long-term residual `ep[0..=39]` is
  known (a later round), slide the older 80 entries of the
  delay line leftward by 40 and write `add(ep[k], dpp[k])`
  into the `dp[-40..=-1]` slot. The §4.5 `dp_hist` is stored
  with `dp_hist[0] = dp[-1]` (the newest history sample), so
  `dp[-1] = add(ep[39], dpp[39])` lands in `dp_hist[0]`.

`LtpAnalyzer::new()` returns the §4.5 home state
(`dp[-120..=-1] = 0`). `LtpAnalyzer::reset()` returns to the home
state.

## Encoder weighting filter (§5.2.13)

The §5.2.13 weighting filter is implemented as the stateless free
function [`analysis::weighting_filter`] in `src/encoder.rs`. It
convolves the §5.2.12 long-term residual `e[0..=39]` with the 11-tap
FIR impulse response `H[0..=10]` of Table 5.4 (already staged in
`src/tables.rs`) using the spec's "block filter" framing:

* A 50-sample working array `wt[0..=49]` is built with 5 leading
  zeros (`wt[0..=4] = 0`), the 40 input samples (`wt[5..=44] =
  e[0..=39]`), and 5 trailing zeros (`wt[45..=49] = 0`).
* For each output position `k = 0..=39`, the 11-tap dot product
  `L_result = 8192 + Σ L_mult(wt[k+i], H[i])` is accumulated with
  the spec's `+8192` rounding constant.
* Two saturating doublings (`L_add(L_result, L_result)` twice)
  apply the spec's `×4` scaling.
* `x[k] = L_result >> 16` lands the block-filtered output sample.

The filter is stateless — every sub-segment starts the `wt[]` array
fresh from `e[..]` with zero padding, so no §4.5 Table 4.2 home
state is added by this clause. The output `x[0..=39]` is the input
the §5.2.14 RPE grid selection step consumes.

## Encoder RPE grid selection (§5.2.14)

The §5.2.14 adaptive sample-rate decimation step is implemented as
the stateless free function [`analysis::select_rpe_grid`] in
`src/encoder.rs`. It picks one of four interleaved sub-sampling
grids of the §5.2.13 weighted signal `x[0..=39]` and emits the
down-sampled 13-pulse sequence the §5.2.15 APCM quantiser
consumes:

* For each grid offset `m ∈ {0, 1, 2, 3}`, accumulate the
  squared-energy proxy `L_result = Σ L_mult(x[m+3i] >> 2,
  x[m+3i] >> 2)` for `i = 0..=12` through the §5.1 saturating
  32-bit `L_add`. The `>> 2` pre-shift scales each contribution
  into a margin where the 13-term sum cannot overflow the 32-bit
  accumulator.
* The strict `L_result > EM` comparison keeps the lower `m` on
  ties — the entry-time `Mc = 0` default propagates when no later
  grid scores strictly higher (an all-zero input therefore yields
  `m_c = 0`).
* The chosen grid is down-sampled by a factor of 3:
  `xM[i] = x[Mc + 3*i]` for `i = 0..=12`.

The function returns the `RpeGrid { m_c, x_m }` struct (also
re-exported from the crate root): `m_c` is the 2-bit codeword the
§1.7 frame packer emits as one of `Mc[1..=4]` per Table 1.1, and
`x_m[0..=12]` is the 13-sample RPE sequence (`RPE_PULSES = 13`
constant) that §5.2.15 APCM quantisation (a later round) consumes.
The selector is stateless — `x[]` already carries all §5.2.12 /
§5.2.13 context, and no §4.5 Table 4.2 entry is owned by §5.2.14.

## Encoder APCM forward quantisation (§5.2.15)

The §5.2.15 APCM forward-quantisation step is implemented as the
stateless free function [`analysis::apcm_quantise_rpe`] in
`src/encoder.rs`. It consumes the §5.2.14 down-sampled 13-pulse
sequence `xM[0..=12]` and emits the [`ApcmQuantised`] struct holding
the encoded codewords plus the `(exp, mant)` pair §5.2.16 needs:

* **Block peak** — `xmax = max |xM[i]|` for `i = 0..=12`. This is
  the saturating `abs` shape the §5.2.4 autocorrelation already uses.
* **Coding `xmaxc`** — `exp = 0; temp = xmax >> 9;` then a six-step
  loop increments `exp` once per step until `temp <= 0` (with the
  spec's `itest` latch turning further increments into no-ops). The
  6-bit codeword is `xmaxc = (xmax >> (exp+5)) + (exp << 3)` —
  3-bit exponent in bits 3..=5, 3-bit mantissa in bits 0..=2. The
  range is `xmaxc ∈ 0..=63`, matching the `Xmaxc[1..=4]` column of
  §1.7 Table 1.1.
* **Decoding back to `(exp, mant)`** — re-derives the same exponent
  and mantissa the §5.2.16 inverse APCM will recover at the decoder.
  Spec branch: `exp = (xmaxc > 15) ? (xmaxc >> 3) - 1 : 0`,
  `mant = xmaxc - (exp << 3)`; then the normalisation loop drops
  `mant` into the 8..=15 range while decrementing `exp`. The
  trailing `sub(mant, 8)` lands `mant ∈ 0..=7` for the `NRFAC[..]`
  / `FAC[..]` table indexing.
* **Direct coding of `xMc[0..=12]`** — for each `i = 0..=12`:
  - `temp = xM[i] << (6 - exp)` — Q15-style normalisation by the
    exponent. The §5.1 `<<` operator with a possibly-negative
    operand falls through to an arithmetic right shift via
    [`shl_signed`], matching the spec semantics.
  - `temp = mult(temp, NRFAC[mant])` — 16×16 → 16 Q15 truncating
    product against Table 5.5 (the inverse-mantissa column).
  - `xMc[i] = (temp >> 12) + 4` — round-toward-zero followed by the
    `+4` bias that maps the signed 3-bit pulse to the unsigned
    0..=7 code §1.7 Table 1.1 emits as `Xm[1..=4][0..12]`. This is
    the inverse of the decoder's §5.2.16
    `temp = sub((xMc[i] << 1), 7)` sign-restoration step.

The function is stateless — `xM[]` already carries all §5.2.13 /
§5.2.14 context, and no §4.5 Table 4.2 entry is owned by §5.2.15.
The `(exp, mant)` pair is returned to the caller alongside the
codewords so the §5.2.16 inverse APCM and §5.2.17 RPE grid
positioning (later rounds) can consume them directly, matching the
spec's "Keep in memory exp and mant for the following inverse APCM
quantizer" note.

The `(exp, mant)` pair is returned to the caller alongside the
codewords so the §5.2.16 inverse APCM and §5.2.17 RPE grid
positioning consume them directly, matching the spec's "Keep in
memory exp and mant for the following inverse APCM quantizer" note.

## Encoder APCM inverse + RPE grid positioning (§5.2.16..§5.2.17)

The encoder's local-decoder feedback path is implemented as the
stateless free function [`analysis::apcm_inverse_and_position`] in
`src/encoder.rs`. It consumes the §5.2.15 codewords (`xMc[0..=12]`)
plus the post-normalisation `(exp, mant)` pair and the §5.2.14 grid
offset `Mc`, and produces the reconstructed long-term residual
`ep[0..=39]` that §5.2.18 folds back into the LTP delay line:

* **§5.2.16 APCM inverse quantisation** — `temp1 = FAC[mant]`
  (Table 5.6), `temp2 = sub(6, exp)`, `temp3 = 1 << sub(temp2, 1)`;
  then per pulse `temp = sub((xMc[i] << 1), 7) << 12;
  temp = mult_r(temp1, temp); temp = add(temp, temp3);
  xMp[i] = temp >> temp2`. The `sub((xMc << 1), 7)` step restores
  the pulse sign — the exact inverse of §5.2.15's `add((temp >> 12),
  4)` pack. Unlike the decoder's §5.3.1 path (which re-derives
  `(exp, mant)` from `xmaxc`), the encoder re-uses the `(exp, mant)`
  pair §5.2.15 already left in memory; the arithmetic is otherwise
  bit-identical to `decoder::rpe_decode`.
* **§5.2.17 RPE grid positioning** — drop the 13 dequantised pulses
  at `Mc, Mc+3, …, Mc+36` in an otherwise-zero 40-sample buffer
  `ep[Mc + 3*i] = xMp[i]`.

[`analysis::LtpAnalyzer::reconstruct_and_update`] chains §5.2.16 →
§5.2.17 → §5.2.18 in one call: it reconstructs `ep[0..=39]`, then
folds `add(ep[k], dpp[k])` (with the §5.2.12 prediction estimate
`dpp[..]`) back into the §4.5 `dp[-120..=-1]` delay line via the
existing `update_dp_after_subframe`. The history it builds is the
encoder's local-decoder copy of the reconstructed short-term
residual — bit-exact with the `drp[..]` the receiving decoder builds
in §5.3.2, so the next sub-segment's §5.2.11 cross-correlation search
runs on the same history the decoder sees.

This **analysis-by-synthesis bit-exactness** is the encoder's
load-bearing correctness invariant and is now pinned for the general
case: `local_decoder_dp_history_matches_decoder_drp_history_every_frame`
encodes 12 frames each of five signal classes (silence, periodic
triangle, broadband ramp mix, alternating loud square, pseudo-random),
carries every output over the real §1.7 bitstream into a decoder, and
asserts the two 120-sample LTP delay lines are **equal at every frame**
— exercising non-zero `bc`, all four sub-segments, and many frames of
state carry-over. (Earlier coverage pinned only the `bc = 0`,
first-sub-segment, home-state case.) A companion test carries the
invariant through a complete §4.3/§4.4 homing event, documenting the
spec's asymmetric reset timing (a single homing frame homes the encoder
but not the decoder — per the §4.3 NOTE the decoder only sees a genuine
decoder-homing-frame on the *second* consecutive homing frame — so the
histories legitimately differ for one frame, then re-lock at the
all-zero home state).

With §5.2.16/§5.2.17 in place the per-sub-segment §5.2.11..§5.2.18
LTP feedback loop closes end-to-end.

## Encoder frame packing + frame-level driver (§1.7)

`UnpackedFrame::to_bit_stream_msb_first` packs the 76 codewords into
the 260-bit `b1..b260` stream per §1.7 Table 1.1 — the exact mirror of
the unpacker (parameters in order of occurrence, each LSB-first within
its bit range per the table's "(LSB-MSB)" column, `b1` in the MSB of
byte 0; the 4 spare bits of byte 32 stay zero). Both pack∘unpack and
unpack∘pack identities are tested at the struct and byte level.

`EncoderState` is the frame-level driver: it aggregates the
`PreProcessor`, `analysis::Analyzer`, and `analysis::LtpAnalyzer`
stages (i.e. the complete §4.5 Table 4.2 encoder state) and
`encode_frame(&[i16; 160]) -> UnpackedFrame` runs §5.2.1..§5.2.3 →
§5.2.4..§5.2.10 → four per-sub-segment §5.2.11..§5.2.18 passes per
frame. `make_encoder` wraps it behind the `oxideav_core::Encoder`
trait: mono S16 8 kHz input (any framing — samples are rebuffered to
the 20 ms codec frame), one 33-byte packet per 160 samples with
sample-accurate pts/duration in the 1/8000 time base, and `flush`
zero-pads a trailing partial frame. Roundtrip tests pin a ≥6 dB
encode→decode error floor on a periodic signal and bit-level
equivalence of the packed and unpacked decode paths.

## Codec homing (§4.3 / §4.4)

The decoder implements the §4.4 in-band homing protocol via
[`DecoderState::decode_frame_with_homing`] (also wired into the
`oxideav_core::Decoder` adapter on `make_decoder`):

* If the input frame matches the §4.4 Table 4.1a/b
  decoder-homing-frame, the output is replaced with the §4.2
  encoder-homing-frame (160 samples of `0x0008`) and the
  decoder's state is reset to the §4.6 Table 4.3 home values.
* Otherwise the frame passes straight to the §5.3 pipeline.

Per §4.4 NOTE 1, this gives the "N homing frames in → N-1 homing
frames out" property the spec calls for, since the first homing
frame triggers a state reset that the second arrives into.
`decode_frame` is kept as the raw §5.3 entry point for callers
who want pre-homing pipeline output.

§4.4 NOTE 2 / §6.3.3.2 — the **delay-optimised partial homing
detection** — is also implemented. Once the decoder is already in
its §4.6 home state (queryable via [`DecoderState::is_home_state`]),
a subsequent frame only needs to carry the decoder-homing-frame's
LARs and **first** sub-frame — sub-frames 2..=4 may hold arbitrary
data — to trigger the same encoder-homing-frame substitution + state
reset. This is the criterion the §6.3.3.2 `HOMING01` conformance
sequence exercises (after two complete homing frames home the
decoder it feeds a mixture of complete and *fractional* homing
frames, each of which must still home the decoder) and the
"delay-optimized implementation in the TRAU uplink direction" §4.4
NOTE 2 names. [`is_partial_decoder_homing_frame`] is the predicate;
`decode_frame_with_homing` applies it only when homed, falling back
to the full-frame [`is_decoder_homing_frame`] check otherwise (a
non-homed decoder must still receive a *complete* homing frame, since
§4.4 NOTE 2's soundness argument only holds from the home state).

The encoder side is [`EncoderState::encode_frame_with_homing`]
(wired into the `oxideav_core::Encoder` adapter on `make_encoder`):
per §4.3, an input frame matching the §4.2 encoder-homing-frame
(160 samples of `0x0008`, detected by [`is_encoder_homing_frame`])
encodes normally — no output substitution on this side — and then
resets every §4.5 Table 4.2 state variable to its home value.
§4.3 Step 1's construction sentence is pinned as a test: from the
home state, the encoder-homing-frame encodes **bit-exactly** to the
§4.4 Table 4.1a/b decoder-homing-frame — a spec-supplied
conformance vector exercising the entire §5.2 pipeline (all eight
LARc codewords, and Nc/bc/Mc/xmaxc/xMc of all four sub-frames,
including the lone `xMc[4] = 0x0003` deviation in sub-frame 4).
Further tests pin the §4.3 NOTE "N in → N-1 out" property and the
§4.1 loop-back interplay (second encoder-homing output → §1.7
bitstream → homing decoder → encoder-homing-frame again). §4.3
Step 2 also names VAD and DTX among the sub-modules to home; this
crate implements neither (GSM 06.32 / GSM 06.31 are separate,
unstaged specifications), so the reset covers the complete §4.5
Table 4.2 set the encoder owns.

## Encoder framing synchronization (§6.3.3.3)

When the encoder is tested as a black box "there is no information
available about where the encoder starts its 20 ms segments of speech
input" (§6.3.3.3). The spec recovers both bit and frame alignment using
only the codec-homing feature. The crate's [`sync`] module implements the
**bit-synchronization** half — the part that is fully specified textually
and depends only on features this crate owns:

* **Bit synchronization** — there are 13 possible bit alignments of the
  13-bit linear PCM input word. Each trial feeds **three** consecutive
  §4.2 encoder-homing-frames at one candidate alignment (three, not two,
  because frame sync is unknown — so the encoder is guaranteed to read at
  least two *complete* homing frames whatever its 20 ms segmentation),
  and checks whether the §4.4 decoder-homing-frame appears at the encoder
  output. The first alignment that produces it is bit sync.
  - `run_bit_sync_trial(shift, &[frame; 3]) -> BitSyncTrial` — one trial.
  - `find_bit_sync(|shift| -> [frame; 3]) -> Option<usize>` — sweep the
    `BIT_SYNC_TRIALS` (= `PCM_WORD_BITS` = 13) candidate shifts and return
    the first that homes the output.
* **Reference-sequence formats** — [`SyncFormats`] records the §6.3.3.4
  stated sizes (`BITSYNC.INP` = 12 480 B, `SEQSYNC.INP` = 1 280 B,
  `SYNCxxx.COD` = 152 B, 160 frame-sync positions). The binary sequences
  themselves are **not** staged under `docs/audio/gsm/` (they ship in the
  ETSI conformance archive), so the *frame*-synchronization sweep that
  recovers the 0..159 sample offset against `SYNC000.COD..SYNC159.COD` is
  deferred to a follow-up round once that corpus is staged.

## End-to-end conformance harness (§6.2 / §6.3.3.1)

`tests/conformance_homing.rs` runs the spec's own §6.2 verification
configurations against the only §6 digital test vectors that are fully
specified **inside the staged PDF**: §6.3.3.1's `SEQ06H.INP` ("one
encoder-homing-frame", §4.2 — 160 PCM words of `0x0008`) and
`SEQ06H.COD` ("one decoder-homing-frame", §4.4 Table 4.1a/b, packed
into the §1.7 33-byte `b1..b260` stream). Both are reconstructed from
the spec and driven through the **public registry adapters**
(`make_encoder` / `make_decoder` with real `Packet` / `Frame` values),
so the whole §5.2 encode → §1.7 pack → §1.7 unpack → §5.3 decode chain
runs end-to-end at the byte level — the in-crate unit tests pin the
same relationships only at the internal `UnpackedFrame` boundary.

* **§6.2.1 Configuration 1 (encoder under test)** — from the §4.5 home
  state, the encoder maps `SEQ06H.INP` bit-exactly to `SEQ06H.COD`; the
  §6.2.1 "first output frame undefined, all subsequent identical"
  property is exercised over a repeated-homing stream.
* **§6.2.2 Configuration 2 (decoder under test)** — `SEQ06H.COD`
  decodes to the encoder-homing-frame (160 × `0x0008`); the "two leading
  homing frames define the subsequent output" property is shown
  **history-independent** (a clean decoder and one pre-soiled with a
  noisy frame yield identical post-homing samples); the §6.3.3.2
  fractional-homing reset works through the `Packet` adapter; and a
  multi-frame coded stream decodes to §5.3.7-shaped output with correct
  `NeedMore` discipline.
* **§4.1 loop-back** — `SEQ06H.INP` → encode → `SEQ06H.COD` → decode →
  `SEQ06H.INP` closes the circle through both adapters with byte-exact
  endpoints.

The bulk SEQ01..SEQ05 `*.INP`/`*.COD`/`*.OUT` corpus (the LPC / LTP /
overflow / critical-path sequences of §6.3.1 / §6.3.2) ships in the
unstaged ETSI conformance archive (`en_300961v080101p0.ZIP`); running
those is a docs-staging followup. The SEQ06H homing vectors are the
spec-complete subset that needs no external corpus.

Although the SEQ01..SEQ05 *binary* corpus is unstaged, the **invariants**
the §6.3.1 / §6.3.2 Table 6.2 / 6.5 / 6.6 / 6.7 design enumerates are
fully specified inside the PDF — each row is either an *"incorrect
statement / correct statement"* pair naming an exact §5.4 table cell,
comparison direction, or loop range (Table 6.5/6.6), or a named
overflow point that must saturate (Table 6.2/6.7).
`tests/conformance_seq04.rs` (18 tests) + the in-crate unit modules pin
every one directly against the public API + §5.4 constants:

* **Table 6.5** mutation points — §5.2.7 `A[4]/A[5]/A[6]/A[8]` +
  `MAC[2]`/`MIC[2]` clamps, §5.2.16 `FAC[2..7]`, §5.2.11 LTP-gain
  `mult`-not-`mult_r` decision ladder + `DLB[0]/DLB[1]` levels +
  opt-scaling range `k = 0..=39`, §5.2.4/§5.2.5/§5.2.9.2 ranges +
  comparisons.
* **Table 6.6** SEQ04-only paths — `smax == 0`, `L_ACF[0] == 0`,
  `P[0] < abs(P[1])`.
* **Table 6.2** coder-overflow saturations — §5.2.11 / §5.2.15
  `abs(i16::MIN) = +32767`, §5.2.12 `sub`, §5.2.13 ×2/×4 doubling,
  §5.2.18 `add`.
* **Table 6.7** decoder-overflow saturations — §5.3.2 / §5.3.4 / §5.3.5 /
  §5.3.6 `add`.
* **SEQ05** — the encoder never emits an out-of-range LTP lag `Nc`.

So the off-by-one table mutations, flipped comparisons, short loops, and
non-saturating overflows the SEQ01/04/05 vectors would detect are
detected here too — independently of the corpus.

## Comfort noise (GSM 06.12 §5.1 + §6.1)

The transmit-side **background-acoustic-noise evaluation** (§5.1) and
the receive-side **comfort-noise generation** (§6.1) of ETSI EN 300 963
(GSM 06.12) are implemented as the [`comfort_noise`] module. During
Discontinuous Transmission (DTX) the radio link is cut at the end of a
speech burst; the background acoustic noise that travelled with the
speech would vanish abruptly — "very annoying for the listener" (§4).
The receiver masks this by synthesising comfort noise whose level and
spectrum approximate the transmit-side background noise, using the
noise parameters carried in a **SID (Silence Descriptor) frame**.

Per §6.1, comfort noise is produced by driving the standard GSM 06.10
§5.3 RPE-LTP speech decoder with a frame whose parameters are
substituted:

| Parameter            | §6.1 value                                |
|----------------------|-------------------------------------------|
| RPE pulses `Xmcr`    | random integers, uniform in **[1, 6]**    |
| Grid positions `Mcr` | random integers, uniform in **[0, 3]**    |
| LTP gains `bcr`      | **0**                                     |
| LTP lags `Ncr`       | **40, 120, 40, 120** (sub-segments 1..4)  |
| Block ampl. `Xmaxcr` | the four values **received in the SID frame** |
| LAR coeffs `LARcr`   | the values **received in the SID frame**  |

* **[`SidParameters`]** holds the SID-frame LAR codewords
  (`LARcr[1..=8]`) and the four block amplitudes (`Xmaxcr[1..=4]`) in
  the same `UnpackedFrame` codeword form the decoder consumes.
* **[`NoiseRng`]** is a deterministic LCG source for the §6.1 "locally
  generated random integer sequence". §6.1 leaves the generator an
  implementation choice (no seed/table/recurrence is normative); a
  reproducible LCG is used so the comfort noise is unit-testable.
* **[`comfort_noise_frame`]`(sid, rng)`** builds the §6.1 substituted
  [`UnpackedFrame`].
* **[`ComfortNoiseGenerator`]** wraps a [`DecoderState`], a
  [`SidInterpolator`], and the RNG. `generate_frame()` builds one §6.1
  frame and runs the standard §5.3 decoder, yielding 160 PCM samples —
  the inter-frame synthesis/LTP/de-emphasis memory carries across frames
  so the noise is continuous. `update_sid()` applies a freshly received
  SID frame; `reset_decoder()` homes the wrapped decoder on a
  codec-homing event.
* **§6.1 smooth-transition update** — *"when updating the comfort noise,
  the parameters above should preferably be interpolated over a few
  frames to obtain smooth transitions"* (§6.1). On `update_sid()` the
  generator linearly ramps the SID-carried LARs (`LARcr[1..=8]`) and the
  four block amplitudes (`Xmaxcr[1..=4]`) from their in-effect values to
  the newly-received ones over [`DEFAULT_INTERPOLATION_FRAMES`] (= 4)
  frames, avoiding the abrupt noise-level/spectrum step that would
  re-introduce the §4 "modulation of the background noise". Only the
  SID-carried parameters are ramped — the random RPE pulses / grid
  positions and the fixed `Ncr`/`bcr=0` carry no old→new transition. The
  ramp length is the spec's open *"a few frames"* implementation choice
  (4 matches the §5.1 `N = 4` transmit averaging window);
  `set_interpolation_frames()` overrides it, and `0` reproduces the
  plain immediate replacement. A mid-ramp `update_sid()` re-bases from
  the in-flight interpolated value so chained SID updates stay smooth.
  This smoothing is the §6.1 sentence itself — distinct from the §6
  *scheduling* of when a valid SID arrives, which is the (unstaged)
  GSM 06.31 concern.

### §5.1 transmit-side background-acoustic-noise evaluation

§5.1 builds a SID's noise parameters from the **unquantised** coded-
speech parameters of the silent frames — *"the unquantized block
amplitude and Log Area Ratio (LAR) parameters … defined in 4.2.15 and
4.2.6"*. [`NoiseEvaluator`] takes the two §5.1 means over the `N = 4`
most recent VAD = 0 frames (window length [`NOISE_EVAL_FRAMES`]):

* **Log-Area Ratios** —
  `mean(LAR(i)) = (1/N) · Σ LAR[j−n](i)`, `i = 1..8`: each of the
  eight unquantised §5.2.6 LARs averaged independently across the four
  frames.
* **Block amplitude** —
  `mean(xmax) = (1/(4N)) · Σ_n Σ_i xmax[j−n](i)`: a *single* mean over
  **all** `4·N = 16` unquantised sub-segment block maxima (the §5.2.15
  `xmax` search result of each sub-segment).

§5.2 then **encodes those means** *"as described in GSM 06.10"*: the
mean LARs through the §5.2.7 `quantise_lar` quantiser and the single
mean block amplitude through the §5.2.15 `code_xmax` coder, the latter
*"repeated four times inside the frame"* — so [`SidParameters`] already
carries `LARcr` / `Xmaxcr` codewords (the same `Xmaxcr` in all four
slots). Averaging the *coded* codewords instead would be a different,
incorrect quantity, which is why the evaluator consumes unquantised
input.

Frames feed in via `push_frame(NoiseFrameParameters::new(lar, xmax))`,
where `lar` is the encoder's §5.2.6 `analyse_frame` output and `xmax`
the four sub-segments' §5.2.15 `ApcmQuantised::xmax` block maxima;
`evaluate()` returns the [`SidParameters`] the §6.1 receive side
consumes — closing the §5.1 → §6.1 loop end-to-end. The §5.1 means are
exact integer averages; the staged PDF leaves the final-division
rounding direction unstated, so round-to-nearest (ties away from zero,
symmetric for the signed LARs) is used — the only quantity left to
implementation choice here, on the same footing as the receive-side
[`NoiseRng`].

### Still docs-blocked

Three pieces remain unstaged under `docs/audio/gsm/`: the **VAD flag**
that marks which frames are silent (GSM 06.32); the §5.2 SID-frame
*bit* layout — the 95-bit all-zero "SID code word" inserted at "the 95
bits of the encoded RPE-pulses Xmc … in error protection class I (see
GSM 05.03, table 2)" (GSM 05.03, channel coding) — and the matching
receive-side "valid SID frame" detector; and the DTX scheduling that
decides *when* a SID is emitted (GSM 06.31). The transmit side
therefore stops at the §5.1 parameter mean ([`SidParameters`]) rather
than packing a SID bitstream.

## Robustness & round-trip quality gates

Four test suites harden every untrusted-input surface and pin the
whole-codec fidelity that a spec-level regression would break:

* **`tests/robustness_decode.rs`** — no-panic / §5.3.7-output-shaping
  fuzz over every byte surface that reaches the §5.3 decoder: the §1.7
  packed 260-bit stream, the §6.1 word-oriented `*.COD` words, and both
  little- and big-endian `*.COD` byte parsers, plus the `make_decoder`
  registry adapter. Sweeps every `Nc ∈ 0..=127` in each sub-frame (the
  §5.3.2 out-of-range-lag limit check), every `xmaxc ∈ 0..=63` (§5.3.1
  APCM inverse `exp`/`mant` derivation), every `(Mc, bc)`, and extreme
  LARc codewords, confirming the pipeline saturates rather than
  indexing out of bounds.
* **`tests/roundtrip_quality.rs`** — six speech-band signal classes
  (tones, two-tone, AM tone, decaying bursts) each round-tripped
  encode→decode and gated on a per-class segmental-SNR floor set below
  the correct fixed-point pipeline and far above any mis-wired one.
  Because the codec is pure integer arithmetic the floors are hard, not
  statistical. Also pins determinism, §1.7 bitstream transparency, and
  the silence noise floor (the 3-bit RPE code has no exact-zero level —
  §5.2.16 sign-restores to the odd integers ±1..±7 — so silence settles
  to a bounded ≈±16 floor, not to zero).
* **`tests/dtx_robustness.rs`** — long random speech/SID/no-data
  streams through the §6 [`DtxReceiver`]: no-panic + shaping on every
  dispatch (including hostile out-of-range SID parameters), the §6
  two-state-machine transitions, §6.1 generator determinism, and
  `reset()` history-independence.
* **`tests/encoder_robustness.rs`** — the §5.2 encoder's PCM-input
  surface: extreme amplitudes (`i16::MIN`/`MAX`, max-slew) drive the
  §5.2.4 / §5.2.11 `abs(i16::MIN)` saturation path; the `make_encoder`
  adapter's rebuffering conserves samples (packet count
  `= ceil(total/160)`, pts `+160`/packet) and rejects malformed input.

## Public API

Two API tiers (the workspace's dual-API convention):

```rust
// Registry path — install into the workspace runtime.
let mut ctx = oxideav_core::RuntimeContext::new();
oxideav_gsm::register(&mut ctx);

// Direct factories — bypass the registry.
let params = oxideav_core::CodecParameters::audio(oxideav_core::CodecId::new("gsm"));
let mut decoder = oxideav_gsm::make_decoder(&params).unwrap();
let mut encoder = oxideav_gsm::make_encoder(&params).unwrap();
```

The lower-level building blocks are public for callers who want to drive
the pipeline directly:

```rust
use oxideav_gsm::{DecoderState, EncoderState, UnpackedFrame};

let mut enc = EncoderState::new();
let coded: UnpackedFrame = enc.encode_frame(&pcm_in_160);
let bytes_33: [u8; 33] = coded.to_bit_stream_msb_first();

let frame = UnpackedFrame::from_bit_stream_msb_first(&bytes_33).unwrap();
let mut dec = DecoderState::new();
let pcm: [i16; 160] = dec.decode_frame(&frame);
```

## Carriage formats

Three packet-level packagings of the same 76 coded parameters are
implemented; all are proven mutually transparent (identical decoded
PCM) by tests:

* **In-band `b1..b260`** — `UnpackedFrame::from_bit_stream_msb_first`
  / `to_bit_stream_msb_first`: the spec's abstract 260-bit stream
  packed MSB-first into 33 bytes (4 trailing spare bits zero). The
  bit numbering matches §1.7 Table 1.1 verbatim. This is the spec's
  own contractual interface (§6.1) and the crate's historical
  adapter format.
* **De-facto `.gsm` byte-frame** — `UnpackedFrame::from_gsm_byte_frame`
  / `to_gsm_byte_frame`: the 33-byte layout raw `.gsm` files use in
  the wild — a constant `0xD` marker nibble in the high nibble of
  byte 0, then the 76 Table 1.1 parameters in order, each MSB-first
  within its field (264 bits exactly, no spare nibble).
* **MS-GSM block** — `UnpackedFrame::pair_from_msgsm_block` /
  `pair_to_msgsm_block`: the 65-byte two-frame block WAVE format tag
  `0x0031` carries (`wBlockAlign` = 65, 320 samples per block) — no
  marker, LSB-first fields, bits filling each byte from its
  least-significant bit, frame B starting mid-byte at bit 260.

The latter two layouts are **not** specified in EN 300 961. They were
derived *empirically* — black-box validator binaries encoded known
PCM, the produced bytes were parsed under candidate layouts, and the
accepted layout is the one that matches this crate's §5.2 encoder
parameter-for-parameter (thousands of frames, quiet through clipped
extremes). The checked-in fixture corpus (`tests/fixtures/` +
`tests/blackbox_fixtures.rs`) pins both byte-for-byte forever.

The codec adapters select the packing via `CodecParameters::extradata`
(a codec-defined field): empty or `b"inband"` (default), `b"gsm"`,
or `b"msgsm"` — see [`FramePacking`]. The MS-GSM encoder emits one
65-byte packet per two input frames and completes a trailing lone
frame with an encoded silence frame on flush.

## Black-box conformance validation

GSM 06.10 defines a fully deterministic fixed-point pipeline, so any
two conformant implementations must agree byte-for-byte (encode) and
sample-for-sample (decode). With the ETSI §6.3 binary test sequences
still unstaged, the crate is validated against independent
implementations invoked as opaque black boxes:

* **Encoder** — byte-exact against an independent encoder binary on
  every stream tried: 6 200+ frames spanning quiet/mid/loud
  staircases, full-scale squares, max-slew alternation, an
  `i16::MIN` plateau, impulse trains, chirps, and amplitude-swept
  random noise clipping at the rails. A 740-frame subset is checked
  in as fixtures and enforced in CI (both `.gsm` and MS-GSM
  packagings).
* **Decoder** — sample-exact against the same implementation's
  decoder over 480 000+ samples of the above corpus. A second,
  unrelated black-box decoder agrees everywhere except
  saturation-heavy clipped-random material, where it diverges from
  both this crate and the reference — two independent
  implementations against one corroborate this crate's §5.3
  arithmetic (whose Table 6.7 saturation points are additionally
  pinned by unit tests straight from the spec text).
* The §4.3 construction sentence ("from the home state, the
  encoder-homing-frame encodes to the decoder-homing-frame") also
  holds on the black-box encoder's first frame; the black box does
  not implement the §4.3 *reset* (a GSM-system feature file
  transcoders omit), and its subsequent homing frames track this
  crate's raw non-homing `encode_frame` output exactly — further
  confirming the §5.2 chain across encoder state carry-over.

## Spec reference

* `docs/audio/gsm/etsi-gsm-06.10-rpe-ltp.pdf` — ETSI EN 300 961 V8.1.1 (2000-11), GSM 06.10 RPE-LTP transcoding.
* `docs/audio/gsm/etsi-en-300963-gsm-06.12-comfort-noise.pdf` — ETSI
  EN 300 963 V8.0.1 (2000-11), GSM 06.12 *Comfort noise aspect for
  full rate speech traffic channels*. The genuine full-rate
  comfort-noise (DTX/SID) spec; its §6.1 receive-side generation is
  now implemented (see the comfort-noise section below). (An earlier
  staging mislabelled the GSM 06.20 half-rate deliverable as "06.12";
  that erratum has since been corrected in `docs/audio/gsm/`.)
* `docs/audio/gsm/etsi-en-300969-gsm-06.20-half-rate.pdf` — ETSI
  EN 300 969 V8.0.1 (2000-11), GSM 06.20 *Half rate speech
  transcoding* (VSELP). Staged for reference only; not implemented by
  this crate.
* `docs/audio/gsm/README.md` — staging notes.

## License

MIT — see [LICENSE](LICENSE).
