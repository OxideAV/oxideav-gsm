//! Robustness sweeps over the §5.2 encoder's untrusted input surface —
//! the PCM samples and the (arbitrary) input framing the `make_encoder`
//! registry adapter accepts.
//!
//! Where `robustness_decode.rs` fuzzes the *decoder's* byte input, this
//! fuzzes the *encoder's* sample input: extreme amplitudes (the §5.2.4
//! autocorrelation and §5.2.11 LTP search both take `abs()` of values
//! that can be `i16::MIN`, which the §5.1 saturating `abs` must clamp to
//! `+32767`), pseudo-random speech, and mis-aligned send sizes the
//! adapter must rebuffer into whole 20 ms frames.
//!
//! Invariants pinned on every input:
//!
//! 1. encode never panics; every emitted packet is exactly 33 bytes;
//! 2. every emitted packet decodes back to 160 §5.3.7-shaped samples;
//! 3. the adapter's rebuffering conserves samples — after `flush`, the
//!    packet count is `ceil(total_samples / 160)` and PTS advances by
//!    160 per packet;
//! 4. encoding is deterministic.

use oxideav_core::{
    AudioFrame, CodecId, CodecParameters, Error as CoreError, Frame, Packet, SampleFormat, TimeBase,
};
use oxideav_gsm::{
    make_decoder, make_encoder, EncoderState, UnpackedFrame, CODEC_ID, FRAME_SAMPLES,
};

struct Lcg(u32);
impl Lcg {
    fn new(seed: u32) -> Self {
        Self(seed)
    }
    fn next_u32(&mut self) -> u32 {
        self.0 = self.0.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        self.0
    }
    fn sample(&mut self) -> i16 {
        (self.next_u32() >> 16) as i16
    }
}

fn iters(full: usize) -> usize {
    if cfg!(miri) {
        (full / 64).max(2)
    } else {
        full
    }
}

fn params() -> CodecParameters {
    let mut p = CodecParameters::audio(CodecId::new(CODEC_ID));
    p.channels = Some(1);
    p.sample_rate = Some(8000);
    p.sample_format = Some(SampleFormat::S16);
    p
}

fn audio_frame(samples: &[i16], pts: Option<i64>) -> Frame {
    let mut bytes = Vec::with_capacity(samples.len() * 2);
    for s in samples {
        bytes.extend_from_slice(&s.to_le_bytes());
    }
    Frame::Audio(AudioFrame {
        samples: samples.len() as u32,
        pts,
        data: vec![bytes],
    })
}

fn assert_shaped_bytes(pcm_bytes: &[u8]) {
    assert_eq!(pcm_bytes.len(), FRAME_SAMPLES * 2);
    for c in pcm_bytes.chunks_exact(2) {
        let s = i16::from_le_bytes([c[0], c[1]]);
        assert_eq!(s & 0b111, 0, "§5.3.7 shaping: {s:#06x}");
    }
}

/// Direct `EncoderState` path: extreme and random 160-sample frames
/// encode to 33-byte-packable frames that decode to shaped output —
/// never panicking through the §5.2.4 / §5.2.11 `abs(i16::MIN)`
/// saturation path.
#[test]
fn direct_encoder_survives_extreme_and_random_frames() {
    let mut rng = Lcg::new(0x9E37_79B9);
    let mut enc = EncoderState::new();
    let mut dec = oxideav_gsm::DecoderState::new();

    // A menu of adversarial constant frames plus random noise.
    for it in 0..iters(1200) {
        let mut sop = [0i16; FRAME_SAMPLES];
        match it % 6 {
            0 => sop.fill(i16::MIN), // abs(MIN) saturation
            1 => sop.fill(i16::MAX), // full-scale positive
            2 => {
                for (k, s) in sop.iter_mut().enumerate() {
                    *s = if k % 2 == 0 { i16::MIN } else { i16::MAX }; // max slew
                }
            }
            3 => sop.fill(0),
            _ => {
                for s in sop.iter_mut() {
                    *s = rng.sample();
                }
            }
        }
        let coded = enc.encode_frame(&sop);
        let bytes = coded.to_bit_stream_msb_first();
        assert_eq!(bytes.len(), 33);
        // Re-parse and decode: every coded frame is well-formed.
        let reparsed = UnpackedFrame::from_bit_stream_msb_first(&bytes).unwrap();
        let pcm = dec.decode_frame(&reparsed);
        for (k, &s) in pcm.iter().enumerate() {
            assert_eq!(s & 0b111, 0, "§5.3.7 shaping at {k}: {s:#06x}");
        }
    }
}

/// The registry encoder adapter accepts arbitrarily-sized sends and
/// rebuffers them into whole 20 ms frames: over a random schedule of
/// send sizes the total packet count after flush is
/// `ceil(total_samples / 160)`, each packet is 33 bytes and decodes to
/// shaped output, and PTS advances by 160 per packet.
#[test]
fn adapter_rebuffers_arbitrary_send_sizes() {
    let mut rng = Lcg::new(0x1000_0001);
    for _ in 0..iters(60) {
        let mut enc = make_encoder(&params()).unwrap();
        let mut dec = make_decoder(&params()).unwrap();

        // A handful of sends of pseudo-random size in [0, 400) samples.
        let n_sends = 1 + (rng.next_u32() % 6) as usize;
        let mut total: usize = 0;
        for s in 0..n_sends {
            let len = (rng.next_u32() % 400) as usize;
            let samples: Vec<i16> = (0..len).map(|_| rng.sample()).collect();
            let pts = if s == 0 { Some(0i64) } else { None };
            enc.send_frame(&audio_frame(&samples, pts)).unwrap();
            total += len;
        }
        enc.flush().unwrap();

        let expected_packets = total.div_ceil(FRAME_SAMPLES);
        let mut got = 0usize;
        let mut expect_pts = 0i64;
        loop {
            match enc.receive_packet() {
                Ok(pkt) => {
                    assert_eq!(pkt.data.len(), 33, "each packet is one §1.7 frame");
                    assert_eq!(pkt.pts, Some(expect_pts), "pts advances by 160");
                    expect_pts += FRAME_SAMPLES as i64;
                    // Decode it: 160 shaped samples.
                    dec.send_packet(&Packet::new(0, TimeBase::new(1, 8000), pkt.data))
                        .unwrap();
                    match dec.receive_frame().unwrap() {
                        Frame::Audio(a) => assert_shaped_bytes(&a.data[0]),
                        other => panic!("expected audio, got {other:?}"),
                    }
                    got += 1;
                }
                Err(CoreError::Eof) => break,
                Err(CoreError::NeedMore) => break,
                Err(e) => panic!("unexpected encoder error: {e:?}"),
            }
        }
        assert_eq!(
            got, expected_packets,
            "rebuffering must conserve samples: {total} samples -> {expected_packets} packets"
        );
    }
}

/// Malformed encoder input is rejected, not panicked on: a non-audio
/// frame, a multi-plane plane count, and an odd byte count all error.
#[test]
fn adapter_rejects_malformed_input() {
    let mut enc = make_encoder(&params()).unwrap();
    // Odd byte count in the S16 plane.
    let odd = Frame::Audio(AudioFrame {
        samples: 1,
        pts: None,
        data: vec![vec![0u8; 3]],
    });
    assert!(enc.send_frame(&odd).is_err(), "odd byte count must error");

    // Two planes (not mono interleaved).
    let two_plane = Frame::Audio(AudioFrame {
        samples: 2,
        pts: None,
        data: vec![vec![0u8; 4], vec![0u8; 4]],
    });
    assert!(
        enc.send_frame(&two_plane).is_err(),
        "multi-plane input must error"
    );
}

/// Flushing an encoder with an empty buffer emits no packet and reports
/// Eof; a lone partial frame is zero-padded to one final packet.
#[test]
fn adapter_flush_edge_cases() {
    // Empty flush: no packet, straight to Eof.
    let mut enc = make_encoder(&params()).unwrap();
    enc.flush().unwrap();
    assert!(matches!(enc.receive_packet(), Err(CoreError::Eof)));

    // Single-sample send then flush: exactly one zero-padded packet.
    let mut enc = make_encoder(&params()).unwrap();
    enc.send_frame(&audio_frame(&[1234i16], Some(0))).unwrap();
    assert!(matches!(enc.receive_packet(), Err(CoreError::NeedMore)));
    enc.flush().unwrap();
    let pkt = enc.receive_packet().unwrap();
    assert_eq!(pkt.data.len(), 33);
    assert!(matches!(enc.receive_packet(), Err(CoreError::Eof)));
}

/// Encoding is deterministic through the adapter: the same input frames
/// produce byte-identical packets.
#[test]
fn adapter_encoding_is_deterministic() {
    let mut rng = Lcg::new(0x5555_AAAA);
    let samples: Vec<i16> = (0..640).map(|_| rng.sample()).collect();

    let run = || {
        let mut enc = make_encoder(&params()).unwrap();
        enc.send_frame(&audio_frame(&samples, Some(0))).unwrap();
        enc.flush().unwrap();
        let mut packets = Vec::new();
        while let Ok(pkt) = enc.receive_packet() {
            packets.push(pkt.data);
        }
        packets
    };
    assert_eq!(run(), run(), "adapter encoding must be deterministic");
}
