//! Robustness / fuzz sweeps over the GSM 06.10 decode entry points.
//!
//! Every parameter that reaches the §5.3 decoder arrives through one
//! of four untrusted byte surfaces: the §1.7 packed 260-bit speech
//! frame ([`UnpackedFrame::from_bit_stream_msb_first`]), the §6.1
//! word-oriented conformance frame (`confio`'s `*.COD` parsers), the
//! de-facto `.gsm` byte-frame ([`UnpackedFrame::from_gsm_byte_frame`]),
//! and the MS-GSM 65-byte block
//! ([`UnpackedFrame::pair_from_msgsm_block`]). All feed arbitrary
//! attacker-controlled bytes into the fixed-point pipeline. The spec constrains the *encoder's* output codeword
//! ranges (§1.7 Table 1.1 field widths, §5.3.2 lag range 40..=120),
//! but a decoder fed a corrupt or hostile stream must still terminate
//! and emit a well-formed frame — §5.3.2 itself carries the
//! out-of-range-lag limit check ("check the limits of Nr") precisely
//! because `Nc` can arrive outside 40..=120.
//!
//! These tests assert the decoder's two hard invariants on *every*
//! reachable input:
//!
//! 1. **Termination without panic** — no index-out-of-bounds from a
//!    hostile `Mc`/`Nc`/`xmaxc`, no arithmetic panic (the §5.1
//!    primitives saturate rather than overflow).
//! 2. **§5.3.7 output shaping** — exactly 160 samples, each with its
//!    low three bits cleared (the 13-bit left-justified output word).
//!
//! No external corpus is needed: the input space is swept
//! exhaustively over the small fields and pseudo-randomly (a
//! deterministic in-test LCG, no external RNG) over the packed byte
//! stream, so the sweep is reproducible and miri-safe. Iteration
//! counts scale down under miri so the interpreter stays fast.

use oxideav_core::{AudioFrame, CodecId, CodecParameters, Frame, Packet, SampleFormat, TimeBase};
use oxideav_gsm::{
    cod_bytes_be_to_unpacked, cod_bytes_le_to_unpacked, cod_words_to_unpacked, make_decoder,
    unpacked_to_cod_bytes_be, unpacked_to_cod_bytes_le, unpacked_to_cod_words, DecoderState,
    SubFrame, UnpackedFrame, CODEC_ID, FRAME_SAMPLES,
};

/// Deterministic Numerical-Recipes LCG — reproducible, no external
/// RNG, miri-safe.
struct Lcg(u32);
impl Lcg {
    fn new(seed: u32) -> Self {
        Self(seed)
    }
    fn next_u32(&mut self) -> u32 {
        self.0 = self.0.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        self.0
    }
    fn byte(&mut self) -> u8 {
        (self.next_u32() >> 24) as u8
    }
}

/// Scale heavy sweeps down under the miri interpreter.
fn iters(full: usize) -> usize {
    if cfg!(miri) {
        (full / 128).max(4)
    } else {
        full
    }
}

/// The §5.3.7 output invariant: exactly 160 samples, low three bits
/// clear on every one.
fn assert_output_shaped(pcm: &[i16]) {
    assert_eq!(pcm.len(), FRAME_SAMPLES, "decoder must emit 160 samples");
    for (k, &s) in pcm.iter().enumerate() {
        assert_eq!(
            s & 0b111,
            0,
            "§5.3.7 output word {k} must have its low 3 bits clear (got {s:#06x})"
        );
    }
}

fn params() -> CodecParameters {
    let mut p = CodecParameters::audio(CodecId::new(CODEC_ID));
    p.channels = Some(1);
    p.sample_rate = Some(8000);
    p.sample_format = Some(SampleFormat::S16);
    p
}

// ───────────────── packed §1.7 stream: no-panic fuzz ─────────────────

/// Any 33-byte buffer decodes to a §5.3.7-shaped frame without
/// panicking — including buffers whose codeword fields carry values
/// the §5.2 encoder never emits (e.g. `Nc` outside 40..=120, which
/// §5.3.2's limit check must absorb).
#[test]
fn packed_stream_fuzz_never_panics_and_stays_shaped() {
    let mut rng = Lcg::new(0xC0FF_EE01);
    let mut dec = DecoderState::new();
    for _ in 0..iters(4000) {
        let mut bytes = [0u8; 33];
        for b in bytes.iter_mut() {
            *b = rng.byte();
        }
        let frame = UnpackedFrame::from_bit_stream_msb_first(&bytes).unwrap();
        // A fresh decoder per iteration would hide state-dependent
        // OOB; carry the state across the whole random stream so the
        // §5.3.2 delay line sees hostile lags accumulate.
        let pcm = dec.decode_frame(&frame);
        assert_output_shaped(&pcm);
    }
}

/// Oversized buffers (longer than 33 bytes) are accepted and only the
/// first 260 bits are consumed; trailing garbage never changes the
/// decoded frame or panics.
#[test]
fn packed_stream_ignores_trailing_bytes() {
    let mut rng = Lcg::new(0x1357_9BDF);
    for _ in 0..iters(512) {
        let mut base = [0u8; 33];
        for b in base.iter_mut() {
            *b = rng.byte();
        }
        base[32] &= 0xF0; // clear the spare nibble (b261..b264)
        let mut long = base.to_vec();
        for _ in 0..64 {
            long.push(rng.byte());
        }
        let a = UnpackedFrame::from_bit_stream_msb_first(&base).unwrap();
        let b = UnpackedFrame::from_bit_stream_msb_first(&long).unwrap();
        assert_eq!(a, b, "trailing bytes must not affect the parsed frame");
    }
}

// ───────────── exhaustive small-field decode robustness ─────────────

/// Sweep every `Nc ∈ 0..=127` (the full 7-bit field, well outside the
/// §5.3.2 error-free 40..=120 range) in each of the four sub-frame
/// positions and assert the §5.3.2 limit check keeps the 120-sample
/// delay line index in bounds — no panic, output stays shaped.
#[test]
fn every_nc_value_in_every_subframe_is_safe() {
    for sf in 0..4 {
        for nc in 0u8..=127 {
            let mut f = UnpackedFrame::default();
            // Give the RPE pulses some energy so the long-term filter
            // actually reaches back into the delay line at lag Nc.
            f.sub[sf] = SubFrame {
                n_c: nc,
                b_c: 3,
                m_c: 1,
                xmax_c: 40,
                x_mc: [6; 13],
            };
            let mut dec = DecoderState::new();
            // Two frames: the first fills the delay line, the second
            // reads back at the hostile lag.
            assert_output_shaped(&dec.decode_frame(&f));
            assert_output_shaped(&dec.decode_frame(&f));
        }
    }
}

/// Sweep every `xmaxc ∈ 0..=63` (the full 6-bit block-amplitude field)
/// through the §5.3.1 APCM inverse `(exp, mant)` derivation — the
/// exponent/mantissa normalisation loop must terminate and index the
/// FAC table in range for all 64 codewords.
#[test]
fn every_xmaxc_value_is_safe() {
    for xmax in 0u8..=63 {
        let mut f = UnpackedFrame::default();
        for sf in 0..4 {
            f.sub[sf] = SubFrame {
                n_c: 80,
                b_c: 2,
                m_c: (xmax & 3),
                xmax_c: xmax,
                x_mc: [7; 13],
            };
        }
        let mut dec = DecoderState::new();
        assert_output_shaped(&dec.decode_frame(&f));
    }
}

/// Sweep every `(Mc, bc)` combination — 4 × 4 — with maximal pulses so
/// the §5.3.3 grid positioning writes at `Mc + 3*12 = Mc + 36` (≤ 39)
/// and the §5.3.2 long-term filter runs at full gain.
#[test]
fn every_grid_and_gain_combo_is_safe() {
    for mc in 0u8..=3 {
        for bc in 0u8..=3 {
            let mut f = UnpackedFrame::default();
            for sf in 0..4 {
                f.sub[sf] = SubFrame {
                    n_c: 100,
                    b_c: bc,
                    m_c: mc,
                    xmax_c: 55,
                    x_mc: [7; 13],
                };
            }
            let mut dec = DecoderState::new();
            for _ in 0..3 {
                assert_output_shaped(&dec.decode_frame(&f));
            }
        }
    }
}

/// Every `LARc[i]` at its field maximum (all-ones codeword) drives the
/// §5.2.8 LAR decode + §5.2.9.2 reflection-coefficient lookup at the
/// segment extremes — must stay shaped and panic-free.
#[test]
fn extreme_lar_codewords_are_safe() {
    let lar_max: [i16; 8] = [63, 63, 31, 31, 15, 15, 7, 7];
    // All-max, all-zero, and a checkerboard of the two.
    for pattern in 0u8..3 {
        let mut f = UnpackedFrame::default();
        for i in 1..=8 {
            f.lar_c[i] = match pattern {
                0 => lar_max[i - 1],
                1 => 0,
                _ => {
                    if i % 2 == 0 {
                        lar_max[i - 1]
                    } else {
                        0
                    }
                }
            };
        }
        for sf in 0..4 {
            f.sub[sf].n_c = 60;
            f.sub[sf].xmax_c = 30;
            f.sub[sf].x_mc = [4; 13];
        }
        let mut dec = DecoderState::new();
        for _ in 0..3 {
            assert_output_shaped(&dec.decode_frame(&f));
        }
    }
}

// ───────────── confio word/byte parser robustness ─────────────

/// `cod_words_to_unpacked` masks every word to its Table 6.1b field
/// width, so arbitrary 16-bit words (high bits set) decode to the same
/// frame as the low-bits-only words and never carry an out-of-range
/// value into the pipeline.
#[test]
fn cod_words_high_bits_are_masked_and_decode_safely() {
    let mut rng = Lcg::new(0xBEEF_5A5A);
    let mut dec = DecoderState::new();
    for _ in 0..iters(2000) {
        let mut words = [0u16; 76];
        for w in words.iter_mut() {
            *w = (rng.next_u32() & 0xFFFF) as u16;
        }
        let f = cod_words_to_unpacked(&words);
        // Masking invariant: re-encoding the decoded frame's words
        // reproduces the *masked* words exactly.
        let reencoded = unpacked_to_cod_words(&f);
        for (i, (&orig, &re)) in words.iter().zip(reencoded.iter()).enumerate() {
            let mask = (1u32 << oxideav_gsm::COD_FIELD_WIDTHS[i]) as u16 - 1;
            assert_eq!(orig & mask, re, "word {i} lost/gained bits under masking");
        }
        assert_output_shaped(&dec.decode_frame(&f));
    }
}

/// The little- and big-endian `*.COD` byte parsers accept any
/// 152-byte buffer, agree with the word-level parser, and their output
/// decodes to a shaped frame. Short buffers are rejected.
#[test]
fn cod_byte_parsers_fuzz_and_short_reject() {
    let mut rng = Lcg::new(0x0DDB_A11E);
    let mut dec = DecoderState::new();
    for _ in 0..iters(1500) {
        let mut bytes = [0u8; 152];
        for b in bytes.iter_mut() {
            *b = rng.byte();
        }
        let le = cod_bytes_le_to_unpacked(&bytes).unwrap();
        let be = cod_bytes_be_to_unpacked(&bytes).unwrap();
        // LE and BE reinterpret the same bytes as different words, so
        // the frames differ in general — but each must decode safely.
        assert_output_shaped(&dec.decode_frame(&le));
        assert_output_shaped(&dec.decode_frame(&be));

        // Cross-check: rebuild the LE bytes from the parsed frame; the
        // masked round-trip must reproduce them (high/unused bits of
        // each word are zero after masking, matching a re-serialise).
        let masked_words = unpacked_to_cod_words(&le);
        let mut expected = [0u8; 152];
        for (w, chunk) in masked_words.iter().zip(expected.chunks_exact_mut(2)) {
            chunk.copy_from_slice(&w.to_le_bytes());
        }
        assert_eq!(unpacked_to_cod_bytes_le(&le), expected);
        let _ = unpacked_to_cod_bytes_be(&be);
    }
    // Every length below 152 is a ShortFrame; empty and off-by-one.
    for len in [0usize, 1, 75, 151] {
        let buf = vec![0u8; len];
        assert!(cod_bytes_le_to_unpacked(&buf).is_err(), "len {len} LE");
        assert!(cod_bytes_be_to_unpacked(&buf).is_err(), "len {len} BE");
    }
}

// ───────────── registry adapter: hostile packet fuzz ─────────────

/// The public `make_decoder` registry adapter must survive a stream of
/// hostile 33-byte packets: no panic, every emitted audio frame is
/// 160 samples of §5.3.7-shaped S16 little-endian PCM.
#[test]
fn registry_decoder_survives_hostile_packet_stream() {
    let mut rng = Lcg::new(0xF00D_1234);
    let mut dec = make_decoder(&params()).unwrap();
    for _ in 0..iters(600) {
        let mut data = vec![0u8; 33];
        for b in data.iter_mut() {
            *b = rng.byte();
        }
        dec.send_packet(&Packet::new(0, TimeBase::new(1, 8000), data))
            .unwrap();
        match dec.receive_frame().unwrap() {
            Frame::Audio(AudioFrame { samples, data, .. }) => {
                assert_eq!(samples, FRAME_SAMPLES as u32);
                assert_eq!(data.len(), 1);
                let pcm: Vec<i16> = data[0]
                    .chunks_exact(2)
                    .map(|c| i16::from_le_bytes([c[0], c[1]]))
                    .collect();
                assert_output_shaped(&pcm);
            }
            other => panic!("expected audio frame, got {other:?}"),
        }
    }
}

// ───────── byte-frame + MS-GSM packet surfaces (extradata packings) ─────────

/// Any 33-byte buffer carrying the 0xD marker parses through
/// `from_gsm_byte_frame` and decodes §5.3.7-shaped without panicking;
/// buffers without the marker are rejected, never mis-decoded.
#[test]
fn gsm_byte_frame_fuzz_never_panics_and_stays_shaped() {
    let mut rng = Lcg::new(0xB17E_F00D);
    let mut dec = DecoderState::new();
    for i in 0..iters(4000) {
        let mut bytes = [0u8; 33];
        for b in bytes.iter_mut() {
            *b = rng.byte();
        }
        if i % 2 == 0 {
            // Half the sweep carries the marker so the payload path
            // runs; the other half exercises the rejection path on
            // arbitrary first nibbles (some randomly valid).
            bytes[0] = (bytes[0] & 0x0F) | 0xD0;
        }
        match UnpackedFrame::from_gsm_byte_frame(&bytes) {
            Ok(frame) => {
                let pcm = dec.decode_frame(&frame);
                assert_output_shaped(&pcm);
            }
            Err(e) => {
                assert_ne!(bytes[0] >> 4, 0xD, "marker frames must parse: {e}");
            }
        }
    }
}

/// Any 65-byte buffer parses as an MS-GSM block (the layout has no
/// marker — every bit pattern is a syntactically valid block) and
/// both frames decode §5.3.7-shaped without panicking.
#[test]
fn msgsm_block_fuzz_never_panics_and_stays_shaped() {
    let mut rng = Lcg::new(0x6535_6535);
    let mut dec = DecoderState::new();
    for _ in 0..iters(2000) {
        let mut bytes = [0u8; 65];
        for b in bytes.iter_mut() {
            *b = rng.byte();
        }
        let (a, b) = UnpackedFrame::pair_from_msgsm_block(&bytes).unwrap();
        assert_output_shaped(&dec.decode_frame(&a));
        assert_output_shaped(&dec.decode_frame(&b));
    }
}

/// The registry adapter under the `b"gsm"` and `b"msgsm"` packings
/// survives hostile packet streams: parse failures surface as `Err`,
/// never as a panic, and every emitted frame is shaped PCM of the
/// packing's sample count.
#[test]
fn registry_decoder_survives_hostile_packets_all_packings() {
    for (packing, unit, samples_per_pkt) in [(&b"gsm"[..], 33usize, 160u32), (b"msgsm", 65, 320)] {
        let mut rng = Lcg::new(0xAD00_0001);
        let mut p = params();
        p.extradata = packing.to_vec();
        let mut dec = make_decoder(&p).unwrap();
        let mut decoded = 0usize;
        for i in 0..iters(400) {
            let mut data = vec![0u8; unit];
            for b in data.iter_mut() {
                *b = rng.byte();
            }
            if packing == b"gsm" && i % 2 == 0 {
                data[0] = (data[0] & 0x0F) | 0xD0;
            }
            dec.send_packet(&Packet::new(0, TimeBase::new(1, 8000), data))
                .unwrap();
            match dec.receive_frame() {
                Ok(Frame::Audio(AudioFrame { samples, data, .. })) => {
                    decoded += 1;
                    assert_eq!(samples, samples_per_pkt);
                    let pcm: Vec<i16> = data[0]
                        .chunks_exact(2)
                        .map(|c| i16::from_le_bytes([c[0], c[1]]))
                        .collect();
                    assert_eq!(pcm.len(), samples_per_pkt as usize);
                    for (k, &s) in pcm.iter().enumerate() {
                        assert_eq!(s & 7, 0, "sample {k} must be §5.3.7-shaped");
                    }
                }
                Ok(other) => panic!("expected audio frame, got {other:?}"),
                Err(_) => {
                    // Marker-less byte-frames are rejected; that is
                    // the contract, not a robustness failure.
                    assert_eq!(packing, b"gsm");
                }
            }
        }
        assert!(decoded > 0, "packing {packing:?} never decoded anything");
    }
}
