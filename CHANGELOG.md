# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased]

### Changed

- **Reset to orphan-rebuild scaffold (2026-05-25).** The prior
  implementation was retired under the workspace clean-room policy: its
  source and licensing declared the codec was modelled on an external
  reference implementation, whose provenance the clean-room policy does
  not permit. All public APIs now return `Error::NotImplemented` pending
  a clean-room rebuild against a staged ETSI GSM 06.10 specification.
