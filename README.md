# oxideav-gsm

Pure-Rust **GSM 06.10 Full Rate (RPE-LTP)** speech codec — decoder +
encoder, plus the MS WAV-49 33-byte framing variant. Zero C
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

GSM-FR is 8 kHz mono · 33 bytes per 20 ms frame (160 samples) · lossy
speech codec. Fixed bitrate of 13.2 kbit/s.

```rust
use oxideav_codec::CodecRegistry;
use oxideav_core::{CodecId, CodecParameters, Frame, Packet, TimeBase};

let mut reg = CodecRegistry::new();
oxideav_gsm::register(&mut reg);

let mut params = CodecParameters::audio(CodecId::new("gsm"));
params.sample_rate = Some(8_000);
params.channels = Some(1);

let mut dec = reg.make_decoder(&params)?;
// gsm_frame is a 33-byte GSM-FR frame (or a 65-byte WAV-49 pair)
dec.send_packet(&Packet::new(0, TimeBase::new(1, 8_000), gsm_frame))?;
let Frame::Audio(a) = dec.receive_frame()? else { unreachable!() };
// `a.data[0]` is 160 S16 PCM samples at 8 kHz mono.
# Ok::<(), oxideav_core::Error>(())
```

Encoder takes the inverse route:

```rust
let params = /* same as above */;
let mut enc = reg.make_encoder(&params)?;
enc.send_frame(&Frame::Audio(pcm_160_samples))?;
let pkt = enc.receive_packet()?;   // 33 bytes
```

### Codec IDs

- **Standard GSM-FR**: `"gsm"` — 33-byte frames
- **WAV-49**: same codec id, but input packets are 65-byte pairs of
  two GSM frames with Microsoft's odd/even layout

## Status

Full spec coverage for the single mode GSM 06.10 defines. AMR family
codecs (NB, WB, etc.) are out of scope.

## License

MIT — see [LICENSE](LICENSE).
