# oxideav-gsm

Pure-Rust GSM Full Rate (ETSI GSM 06.10, RPE-LTP) speech codec.

## Status: orphan-rebuild scaffold (reset 2026-05-25)

The previous implementation was retired under the OxideAV clean-room
policy. Its source and licensing declared that the codec was modelled on
an external reference implementation, and the clean-room policy does not
permit consulting any external implementation's source for any reason.
Because that provenance could not be defended, the implementation was
removed and the crate reset to this scaffold.

The crate will be re-built from scratch against a staged ETSI GSM 06.10
(RPE-LTP) specification in a future clean-room round, once that
specification is staged under `docs/audio/gsm/`. Until then every public
API returns `Error::NotImplemented`.

## License

MIT — see [LICENSE](LICENSE).
