# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased]

## [0.0.9](https://github.com/OxideAV/oxideav-gsm/compare/v0.0.8...v0.0.9) - 2026-05-24

### Other

- wrap Display write! to satisfy rustfmt (scaffold)
- orphan-rebuild scaffold — clean-room reset (post-audit 2026-05-25)

### Changed

- **Reset to orphan-rebuild scaffold (2026-05-25).** The prior
  implementation was retired under the workspace clean-room policy: its
  source and licensing declared the codec was modelled on an external
  reference implementation, whose provenance the clean-room policy does
  not permit. All public APIs now return `Error::NotImplemented` pending
  a clean-room rebuild against a staged ETSI GSM 06.10 specification.
