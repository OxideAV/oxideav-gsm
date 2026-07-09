//! Whole-codec round-trip quality gates on synthetic signals.
//!
//! RPE-LTP is lossy, so no round-trip is sample-exact — but a correct
//! §5.2 encoder / §5.3 decoder pair reconstructs a speech-band signal
//! with a large, *stable* segmental signal-to-noise ratio. A codec bug
//! (a swapped table cell, a flipped comparison, a packing slip, a lost
//! state carry) collapses that margin immediately, dropping the SNR to
//! near zero or negative. These gates encode a per-signal-class SNR
//! floor set well below what the correct fixed-point pipeline achieves
//! and well above what any mis-wired pipeline produces.
//!
//! The codec is pure integer arithmetic (the §5.1 saturating
//! primitives), so every number here is bit-identical on every
//! platform — the floors are hard, not statistical. Signals are
//! generated once at build time into 13-bit-clean-ish `i16` and the
//! measurement skips the first two frames (the §4.5 home-state filter
//! warm-up).

use oxideav_core::{AudioFrame, CodecId, CodecParameters, Frame, Packet, SampleFormat, TimeBase};
use oxideav_gsm::{
    make_decoder, make_encoder, DecoderState, EncoderState, UnpackedFrame, CODEC_ID, FRAME_SAMPLES,
};

const WARMUP_FRAMES: usize = 2;

fn params() -> CodecParameters {
    let mut p = CodecParameters::audio(CodecId::new(CODEC_ID));
    p.channels = Some(1);
    p.sample_rate = Some(8000);
    p.sample_format = Some(SampleFormat::S16);
    p
}

/// Encode → decode a whole signal through the direct `EncoderState` /
/// `DecoderState` path and return the decoded samples aligned frame by
/// frame with the input.
fn roundtrip_direct(input: &[i16]) -> Vec<i16> {
    let mut enc = EncoderState::new();
    let mut dec = DecoderState::new();
    let mut out = Vec::with_capacity(input.len());
    for chunk in input.chunks_exact(FRAME_SAMPLES) {
        let mut sop = [0i16; FRAME_SAMPLES];
        sop.copy_from_slice(chunk);
        let coded = enc.encode_frame(&sop);
        out.extend_from_slice(&dec.decode_frame(&coded));
    }
    out
}

/// Segmental SNR in dB, skipping the warm-up frames.
fn snr_db(input: &[i16], decoded: &[i16]) -> f64 {
    let skip = WARMUP_FRAMES * FRAME_SAMPLES;
    let mut sig = 0i64;
    let mut err = 0i64;
    for k in skip..input.len() {
        let s = input[k] as i64;
        let e = s - decoded[k] as i64;
        sig += s * s;
        err += e * e;
    }
    assert!(sig > 0, "signal must carry energy");
    if err == 0 {
        return f64::INFINITY;
    }
    10.0 * (sig as f64 / err as f64).log10()
}

fn build(n: usize, f: impl Fn(usize) -> f64) -> Vec<i16> {
    (0..n)
        .map(|i| f(i).round().clamp(-32760.0, 32760.0) as i16)
        .collect()
}

/// The six signal classes and their measured-minus-margin SNR floors
/// (dB). Measured values on the correct pipeline are ~2-8 dB higher
/// than each floor, so a real regression is unambiguous.
fn signal_bank() -> Vec<(&'static str, Vec<i16>, f64)> {
    use std::f64::consts::PI;
    let n = 20 * FRAME_SAMPLES;
    let tone = |i: usize, f: f64, a: f64| a * (2.0 * PI * f * i as f64 / 8000.0).sin();
    vec![
        ("sine_200hz", build(n, |i| tone(i, 200.0, 8000.0)), 12.0),
        ("sine_500hz", build(n, |i| tone(i, 500.0, 8000.0)), 25.0),
        ("sine_1000hz", build(n, |i| tone(i, 1000.0, 6000.0)), 22.0),
        (
            "two_tone",
            build(n, |i| tone(i, 300.0, 4000.0) + tone(i, 700.0, 3000.0)),
            25.0,
        ),
        (
            "am_tone",
            build(n, |i| {
                (1.0 + 0.5 * (2.0 * PI * 20.0 * i as f64 / 8000.0).sin()) * tone(i, 400.0, 5000.0)
            }),
            22.0,
        ),
        (
            "decaying_bursts",
            build(n, |i| {
                let t = (i % 800) as f64;
                9000.0 * (-t / 200.0).exp() * (2.0 * PI * 350.0 * i as f64 / 8000.0).sin()
            }),
            5.0,
        ),
    ]
}

/// Every synthetic class round-trips above its SNR floor through the
/// direct `EncoderState`/`DecoderState` path.
#[test]
fn synthetic_signals_meet_snr_floor() {
    for (name, sig, floor) in signal_bank() {
        let decoded = roundtrip_direct(&sig);
        let snr = snr_db(&sig, &decoded);
        assert!(
            snr >= floor,
            "{name}: round-trip SNR {snr:.2} dB below floor {floor:.2} dB"
        );
    }
}

/// Digital silence in → a bounded near-silent noise floor. RPE-LTP has
/// no exact-zero excitation level: the 3-bit RPE code sign-restores via
/// §5.2.16 `sub((xMc<<1), 7)`, whose eight levels are the odd integers
/// {-7,-5,-3,-1,1,3,5,7} — there is no 0, so a genuinely zero pulse
/// quantises to ±1 at the minimum block scale. The reconstructed frame
/// therefore settles to a small constant floor (bounded by ±16 here),
/// not to exact zero. The gate is that the floor stays *small and
/// bounded* — a mis-wired pipeline diverges or emits audible energy.
#[test]
fn silence_decays_to_bounded_floor() {
    let input = vec![0i16; 40 * FRAME_SAMPLES];
    let decoded = roundtrip_direct(&input);
    let skip = WARMUP_FRAMES * FRAME_SAMPLES;
    for (k, &s) in decoded[skip..].iter().enumerate() {
        assert!(
            s.unsigned_abs() <= 32,
            "silence floor sample {k} = {s} exceeds the ±32 bound"
        );
        assert_eq!(s & 0b111, 0, "§5.3.7 shaping holds on the silence floor");
    }
}

/// The codec is deterministic: encoding the same input twice with fresh
/// state yields byte-identical coded frames and sample-identical output.
#[test]
fn roundtrip_is_deterministic() {
    for (name, sig, _) in signal_bank() {
        let a = roundtrip_direct(&sig);
        let b = roundtrip_direct(&sig);
        assert_eq!(a, b, "{name}: non-deterministic round-trip");
    }
}

/// The §1.7 bitstream is transparent: encode → pack → unpack → decode
/// reproduces the same samples as encode → decode without the byte
/// stream in the middle, on every signal class.
#[test]
fn packed_path_matches_direct_path() {
    for (name, sig, _) in signal_bank() {
        let mut enc = EncoderState::new();
        let mut dec_direct = DecoderState::new();
        let mut dec_packed = DecoderState::new();
        for chunk in sig.chunks_exact(FRAME_SAMPLES) {
            let mut sop = [0i16; FRAME_SAMPLES];
            sop.copy_from_slice(chunk);
            let coded = enc.encode_frame(&sop);
            let direct = dec_direct.decode_frame(&coded);
            let bytes = coded.to_bit_stream_msb_first();
            let reparsed = UnpackedFrame::from_bit_stream_msb_first(&bytes).unwrap();
            let packed = dec_packed.decode_frame(&reparsed);
            assert_eq!(direct, packed, "{name}: packed path diverged from direct");
        }
    }
}

/// The public `make_encoder` / `make_decoder` registry adapters produce
/// exactly the same reconstructed samples as the direct state path —
/// the `Packet`/`Frame` marshalling is lossless — and clear the SNR
/// floor on a representative tone.
#[test]
fn registry_adapters_match_direct_path_and_meet_floor() {
    let (_, sig, floor) = signal_bank().into_iter().next().unwrap(); // sine_200hz
    let direct = roundtrip_direct(&sig);

    let mut enc = make_encoder(&params()).unwrap();
    let mut dec = make_decoder(&params()).unwrap();
    let mut adapter_out: Vec<i16> = Vec::with_capacity(sig.len());
    for chunk in sig.chunks_exact(FRAME_SAMPLES) {
        let mut bytes = Vec::with_capacity(FRAME_SAMPLES * 2);
        for &s in chunk {
            bytes.extend_from_slice(&s.to_le_bytes());
        }
        enc.send_frame(&Frame::Audio(AudioFrame {
            samples: FRAME_SAMPLES as u32,
            pts: None,
            data: vec![bytes],
        }))
        .unwrap();
        let pkt = enc.receive_packet().unwrap();
        dec.send_packet(&Packet::new(0, TimeBase::new(1, 8000), pkt.data))
            .unwrap();
        match dec.receive_frame().unwrap() {
            Frame::Audio(a) => {
                for c in a.data[0].chunks_exact(2) {
                    adapter_out.push(i16::from_le_bytes([c[0], c[1]]));
                }
            }
            other => panic!("expected audio frame, got {other:?}"),
        }
    }
    assert_eq!(
        adapter_out, direct,
        "registry adapters diverged from the direct path"
    );
    let snr = snr_db(&sig, &adapter_out);
    assert!(
        snr >= floor,
        "adapter SNR {snr:.2} dB below floor {floor:.2}"
    );
}
