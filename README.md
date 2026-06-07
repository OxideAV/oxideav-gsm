# oxideav-gsm

Pure-Rust **GSM 06.10 RPE-LTP** speech codec — the original 13 kbit/s GSM
Full Rate voice codec (20 ms frames, 160 samples at 8 kHz mono).

## Status

| Direction | Coverage | Notes |
|-----------|----------|-------|
| Decoder   | §5.3 pipeline + §4.4 homing protocol | Fixed-point pipeline (frame unpack, LAR decode, LAR interpolation, APCM inverse, RPE grid positioning, long-term + short-term lattice synthesis, de-emphasis, §5.3.7 output shaping) plus §4.4 decoder-homing-frame detection and substitution. Conformance against §6 test sequences still pending — those sequences are distributed in the `en_300961v080101p0.ZIP` archive that ETSI ships alongside the PDF; staging it is a docs followup. |
| Encoder   | §5.2.0..§5.2.14 pre-processing + LPC analysis + LAR quantisation + short-term analysis filter + LTP analysis + long-term analysis filter + weighting filter + RPE grid selection | `PreProcessor` lands §5.2.1 downscaling + §5.2.2 offset-compensation high-pass IIR (32-bit recursive arm) + §5.2.3 first-order FIR pre-emphasis. State (`z1`, `L_z2`, `mp`) carried across frames per §5.2.2/§5.2.3 + §4.5 Table 4.2 home values. The `analysis` sub-module lands §5.2.4 autocorrelation (with dynamic input scaling), §5.2.5 Schur recursion (reflection coefficients `r[1..=8]`), §5.2.6 reflection → Log-Area Ratio transform, §5.2.7 LAR quantisation + coding (`quantise_lar` produces the `LARc[1..=8]` codewords the §1.7 bit packer consumes), and §5.2.10 short-term analysis filter (8-stage lattice). The `Analyzer` struct runs §5.2.7 → §5.2.8 → §5.2.9.1 → §5.2.9.2 → §5.2.10 end-to-end on a pre-processed frame, persisting `LARpp(j-1)[1..=8]` and `u[0..=7]` per §4.5 Table 4.2 across the four sub-blocks and across frames; it returns `(LARc[1..=8], d[0..=159])` where `d` is the short-term residual §5.2.11 LTP analysis consumes. The `LtpAnalyzer` struct lands §5.2.11 LTP parameter calculation (cross-correlation peak search across lags 40..=120 with dynamic scaling, `bc` decision tree via Table 5.3a `DLB`), §5.2.12 long-term analysis filter (`dpp[k] = mult_r(QLB[bc], dp[k-Nc])`, `e[k] = sub(d[k], dpp[k])`), and §5.2.18 `dp[-120..=-1]` delay-line update; per sub-segment it emits `(LtpParameters, dpp[0..=39], e[0..=39])`. §5.2.13 weighting filter is in place as the stateless `weighting_filter` free function — an 11-tap FIR block filter with zero padding (5 leading + 5 trailing zeros) convolving `e[0..=39]` with Table 5.4 `H[0..=10]` to produce the block-filtered signal `x[0..=39]` the §5.2.14 RPE grid selection consumes. §5.2.14 RPE grid selection is in place as the stateless `select_rpe_grid` free function returning `RpeGrid { m_c, x_m }` — picks the sub-sampling grid `m ∈ {0, 1, 2, 3}` that maximises the squared-energy proxy `Σ L_mult(x[m+3i] >> 2, x[m+3i] >> 2)` and down-samples to emit `xM[0..=12] = x[Mc + 3*i]`, the 13-pulse sequence §5.2.15 APCM quantisation consumes. §5.2.15..§5.2.17 APCM + RPE grid positioning, and §1.7 frame packing arrive in later rounds. `make_encoder` still returns `Unsupported`. |

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
  `scalauto > 0` to keep the accumulator from overflowing.
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

Until §5.2.15..§5.2.17 (APCM quantisation + APCM inverse + RPE
grid positioning) and the §1.7 frame packer arrive, `make_encoder`
still returns `Unsupported`.

## Codec homing (§4.4)

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

## Public API

Two API tiers (the workspace's dual-API convention):

```rust
// Registry path — install into the workspace runtime.
let mut ctx = oxideav_core::RuntimeContext::new();
oxideav_gsm::register(&mut ctx);

// Direct factory — bypass the registry.
let params = oxideav_core::CodecParameters::audio(oxideav_core::CodecId::new("gsm"));
let mut decoder = oxideav_gsm::make_decoder(&params).unwrap();
```

The lower-level building blocks are public for callers who want to drive
the pipeline directly:

```rust
use oxideav_gsm::{DecoderState, UnpackedFrame};

let frame = UnpackedFrame::from_bit_stream_msb_first(&bytes_33).unwrap();
let mut dec = DecoderState::new();
let pcm: [i16; 160] = dec.decode_frame(&frame);
```

## Carriage format

`UnpackedFrame::from_bit_stream_msb_first` accepts a 33-byte buffer
holding the spec's `b1..b260` stream packed MSB-first. That bit numbering
matches §1.7 Table 1.1 verbatim. The specific 33-byte container variants
used in the wild (the `.gsm` byte format, RTP payload type 3, MS-GSM WAV
`0x31` block) wrap these 260 bits with per-container framing that is
**not** specified in EN 300 961 itself. Those wrappers will be addressed
in a follow-up round once trace docs are staged.

## Spec reference

* `docs/audio/gsm/etsi-gsm-06.10-rpe-ltp.pdf` — ETSI EN 300 961 V8.1.1 (2000-11), GSM 06.10 RPE-LTP transcoding.
* `docs/audio/gsm/etsi-gsm-06.12-comfort-noise.pdf` — ETSI EN 300 969 V8.0.1 (2000-11), GSM 06.12 comfort noise (not yet wired in).
* `docs/audio/gsm/README.md` — staging notes.

## License

MIT — see [LICENSE](LICENSE).
