# oxideav-gsm

Pure-Rust **GSM 06.10 Full Rate (RPE-LTP)** speech codec — decoder +
encoder, plus the MS WAV-49 33-byte framing variant.

Designed for embedded use in media pipelines: zero C dependencies, no
FFI, no `*-sys` crates.

Originally part of the [oxideav](https://github.com/KarpelesLab/oxideav)
framework (a 100% pure-Rust media transcoding + streaming stack);
extracted to its own crate for independent publication.

## Usage

```toml
[dependencies]
oxideav-gsm = "0.0.3"
```

The crate plugs into [`oxideav-codec`](https://crates.io/crates/oxideav-codec)'s
`CodecRegistry`:

```rust
let mut reg = oxideav_codec::CodecRegistry::new();
oxideav_gsm::register(&mut reg);
```

Decoder id: `"gsm"` — input packets are 33-byte GSM-FR frames (or 65-byte
WAV-49 pairs), output is 16-bit PCM at 8 kHz mono. Encoder takes the
inverse route.

## Status

Full spec coverage for the single mode GSM 06.10 defines. AMR family
codecs (NB, WB, etc.) are out of scope.

## License

MIT — see [LICENSE](LICENSE).
