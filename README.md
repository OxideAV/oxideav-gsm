# oxideav-gsm

Pure-Rust **GSM 06.10 Full Rate (RPE-LTP)** speech codec — decoder +
encoder, plus the Microsoft WAV-49 framing variant. Zero C
dependencies.

Part of the [oxideav](https://github.com/OxideAV/oxideav-workspace)
framework but usable standalone.

## Installation

```toml
[dependencies]
oxideav-core = "0.0"
oxideav-codec = "0.0"
oxideav-gsm = "0.0"
```

## Quick use

GSM-FR is 8 kHz mono, 13 kbit/s, one 20 ms frame = 160 S16 PCM samples
packed into 33 bytes (260 payload bits + a 4-bit `0xD` magic pad).
The WAV-49 variant concatenates two standard frames into a 65-byte
packet with the magic nibbles stripped — this is what `.wav` files
with `WAVE_FORMAT_GSM610` carry.

```rust
use oxideav_codec::CodecRegistry;
use oxideav_core::{CodecId, CodecParameters, Frame, Packet, TimeBase};

let mut reg = CodecRegistry::new();
oxideav_gsm::register(&mut reg);

let mut params = CodecParameters::audio(CodecId::new("gsm"));
params.sample_rate = Some(8_000);
params.channels = Some(1);

let mut dec = reg.make_decoder(&params)?;
// `gsm_frame` is a 33-byte GSM-FR payload.
dec.send_packet(&Packet::new(0, TimeBase::new(1, 8_000), gsm_frame))?;
let Frame::Audio(a) = dec.receive_frame()? else { unreachable!() };
// `a.data[0]` is 160 S16 PCM samples (little-endian) at 8 kHz mono.
# Ok::<(), oxideav_core::Error>(())
```

### Encoder

```rust
let mut enc = reg.make_encoder(&params)?;
enc.send_frame(&Frame::Audio(pcm_160_samples))?;
let pkt = enc.receive_packet()?;   // 33 bytes, starts with nibble 0xD
```

The encoder is fully deterministic: re-running it on the same PCM
input yields byte-identical packets.

### WAV-49 (Microsoft) framing

Register the `gsm_ms` codec id separately; packets are 65 bytes each
and produce two `AudioFrame`s per `send_packet` (call `receive_frame`
twice).

```rust
let mut params = CodecParameters::audio(CodecId::new("gsm_ms"));
params.sample_rate = Some(8_000);
params.channels = Some(1);

let mut dec = reg.make_decoder(&params)?;
dec.send_packet(&Packet::new(0, TimeBase::new(1, 8_000), wav49_block))?;
let Frame::Audio(a0) = dec.receive_frame()? else { unreachable!() };
let Frame::Audio(a1) = dec.receive_frame()? else { unreachable!() };
# Ok::<(), oxideav_core::Error>(())
```

### Codec IDs

- `"gsm"` — canonical ETSI 33-byte frames, each with the `0xD` magic.
- `"gsm_ms"` — Microsoft WAV-49 65-byte blocks (two frames, no magic).

Both expose decoder and encoder implementations. Accepted parameters:
8 kHz, mono, `SampleFormat::S16`.

## Status

Full coverage of the single mode GSM 06.10 defines: LAR quantisation,
long-term predictor (Nc 40..=120, bc/Mc grids), regular-pulse
excitation with 13 × 3-bit pulses per 40-sample sub-frame,
pre-/post-processing, and fixed-point saturating math throughout.
AMR family codecs (NB, WB, etc.) are out of scope — they are distinct
codecs despite the "GSM" name association.

Tests cover bit-unpacking, the ETSI reference tables, encode→decode
round-trips for both framings, encoder determinism, and (when a
`libgsm`-enabled ffmpeg is present) cross-decoding our bitstream with
ffmpeg.

## License

MIT — see [LICENSE](LICENSE).
