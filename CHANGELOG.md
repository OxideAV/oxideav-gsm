# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased]

### Added

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
