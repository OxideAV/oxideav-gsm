//! End-to-end GSM 06.10 RPE-LTP conformance harness against the
//! spec's own §6.3.3.1 codec-homing test vectors.
//!
//! ETSI EN 300 961 §6 ("Digital test sequences") defines two
//! verification configurations:
//!
//! * **§6.2.1 Configuration 1** — encoder under test: a `*.INP`
//!   PCM sequence is fed to the encoder, whose coded output is
//!   compared against the reference `*.COD` file.
//! * **§6.2.2 Configuration 2** — decoder under test: a `*.COD`
//!   coded sequence is fed to the decoder, whose `srop[..]` output
//!   is compared against the reference `*.OUT` file.
//!
//! The bulk SEQ01..SEQ05 vectors ship in the ETSI conformance
//! archive that is *not* staged under `docs/audio/gsm/`, so those
//! can't be run here. But §6.3.3.1 defines **two vectors entirely
//! within the staged PDF**:
//!
//! > *"SEQ06H.INP contains one encoder-homing-frame. SEQ06H.COD
//! > contains one decoder-homing-frame."* (§6.3.3.1)
//!
//! and the encoder-homing-frame (§4.2: 160 samples of `0x0008`) and
//! the decoder-homing-frame (§4.4 Table 4.1a/b) are both fully
//! specified. This harness reconstructs those two vectors from the
//! spec, and drives them through the **public registry adapters**
//! (`make_encoder` / `make_decoder` + `Packet`/`Frame`), so the
//! whole §5.2 encode → §1.7 pack → §1.7 unpack → §5.3 decode chain
//! is exercised end-to-end at the byte level — not just at the
//! internal struct boundary the in-crate unit tests cover.
//!
//! Spec relationships pinned here (all in-PDF, no external corpus):
//!
//! 1. §4.3 step 1 — from the home state the encoder maps SEQ06H.INP
//!    bit-exactly to SEQ06H.COD (§6.3.3.1's construction is exactly
//!    this: SEQ06H.COD is "one decoder-homing-frame").
//! 2. §4.4 step 1 — feeding SEQ06H.COD to the decoder substitutes
//!    the encoder-homing-frame (160 × `0x0008`) at the output.
//! 3. §6.2.1 / §6.2.2 "first frame undefined, rest defined" — the
//!    Configuration-1/2 homing-reset property holds frame-by-frame
//!    over a multi-frame stream.
//! 4. §6.3.3.2 — once homed, a *fractional* decoder-homing-frame
//!    (LARs + first sub-frame only) still homes the decoder, at the
//!    byte level through the `Packet` adapter.

use oxideav_core::{
    AudioFrame, CodecId, CodecParameters, Error as CoreError, Frame, Packet, SampleFormat, TimeBase,
};
use oxideav_gsm::{
    decoder_homing_frame, encoder_homing_frame_pcm, make_decoder, make_encoder, UnpackedFrame,
    CODEC_ID, FRAME_SAMPLES,
};

// ───────────────────────── spec vectors ─────────────────────────

/// §4.2 encoder-homing-frame as the `*.INP` byte form used by the
/// §6 test sequences: 160 little-endian S16 samples of `0x0008`
/// (the §6.1 / §6.3.3.4 "13 bit left justified" PCM word). This is
/// `SEQ06H.INP` (§6.3.3.1: "one encoder-homing-frame").
fn seq06h_inp() -> Vec<u8> {
    let pcm = encoder_homing_frame_pcm();
    let mut bytes = Vec::with_capacity(FRAME_SAMPLES * 2);
    for s in pcm {
        bytes.extend_from_slice(&s.to_le_bytes());
    }
    bytes
}

/// §4.4 Table 4.1a/b decoder-homing-frame packed into the §1.7
/// 33-byte `b1..b260` stream — i.e. `SEQ06H.COD` (§6.3.3.1: "one
/// decoder-homing-frame").
fn seq06h_cod() -> [u8; 33] {
    decoder_homing_frame().to_bit_stream_msb_first()
}

fn params() -> CodecParameters {
    let mut p = CodecParameters::audio(CodecId::new(CODEC_ID));
    p.channels = Some(1);
    p.sample_rate = Some(8000);
    p.sample_format = Some(SampleFormat::S16);
    p
}

fn audio_frame(pcm: &[i16], pts: Option<i64>) -> Frame {
    let mut bytes = Vec::with_capacity(pcm.len() * 2);
    for s in pcm {
        bytes.extend_from_slice(&s.to_le_bytes());
    }
    Frame::Audio(AudioFrame {
        samples: pcm.len() as u32,
        pts,
        data: vec![bytes],
    })
}

fn packet(data: Vec<u8>) -> Packet {
    Packet::new(0, TimeBase::new(1, 8000), data)
}

/// Decode one 33-byte coded frame through the public registry
/// decoder adapter and return its 160 little-endian S16 samples.
fn decode_one(dec: &mut Box<dyn oxideav_core::Decoder>, cod: &[u8]) -> Vec<u8> {
    dec.send_packet(&packet(cod.to_vec())).unwrap();
    match dec.receive_frame().unwrap() {
        Frame::Audio(a) => {
            assert_eq!(a.samples, FRAME_SAMPLES as u32);
            assert_eq!(a.data.len(), 1);
            a.data.into_iter().next().unwrap()
        }
        _ => panic!("expected audio frame"),
    }
}

/// Encode one 160-sample PCM frame through the public registry
/// encoder adapter and return the 33-byte coded packet.
fn encode_one(enc: &mut Box<dyn oxideav_core::Encoder>, pcm: &[i16; 160]) -> Vec<u8> {
    enc.send_frame(&audio_frame(pcm, None)).unwrap();
    match enc.receive_packet() {
        Ok(pkt) => {
            assert_eq!(pkt.data.len(), 33);
            pkt.data
        }
        Err(e) => panic!("encoder produced no packet: {e:?}"),
    }
}

// ──────────────────────── §6.3.3.1 vectors ────────────────────────

/// §4.2 / §6.3.3.4 — `SEQ06H.INP` is exactly 160 S16 words of
/// `0x0008` (320 bytes), the encoder-homing-frame's PCM form.
#[test]
fn seq06h_inp_is_160_words_of_0x0008() {
    let inp = seq06h_inp();
    assert_eq!(inp.len(), FRAME_SAMPLES * 2, "one 20 ms frame of S16 PCM");
    for pair in inp.chunks_exact(2) {
        assert_eq!(
            i16::from_le_bytes([pair[0], pair[1]]),
            0x0008,
            "every encoder-homing-frame sample is 0x0008 (§4.2)"
        );
    }
}

/// §6.3.3.1 — `SEQ06H.COD` round-trips through the §1.7 unpacker
/// back to the §4.4 Table 4.1a/b decoder-homing-frame, confirming
/// the 33-byte vector is the genuine decoder-homing-frame on the
/// wire (not just at the struct level).
#[test]
fn seq06h_cod_unpacks_to_the_decoder_homing_frame() {
    let cod = seq06h_cod();
    let reparsed = UnpackedFrame::from_bit_stream_msb_first(&cod).unwrap();
    assert_eq!(
        reparsed,
        decoder_homing_frame(),
        "SEQ06H.COD must unpack to the Table 4.1a/b decoder-homing-frame"
    );
}

// ───────────────── §6.2.1 Configuration 1 (encoder) ─────────────────

/// §4.3 step 1 / §6.3.3.1 — from the §4.5 home state, the encoder
/// maps `SEQ06H.INP` (one encoder-homing-frame) bit-exactly to
/// `SEQ06H.COD` (one decoder-homing-frame), at the byte level
/// through the public `make_encoder` registry adapter.
///
/// This is the spec's own §6.3.3.1 construction sentence pinned
/// end-to-end: SEQ06H.COD *is* "one decoder-homing-frame", and a
/// freshly-reset encoder fed the encoder-homing-frame must produce
/// it.
#[test]
fn config1_encoder_maps_seq06h_inp_to_seq06h_cod() {
    let mut enc = make_encoder(&params()).unwrap();
    let inp = encoder_homing_frame_pcm();
    let coded = encode_one(&mut enc, &inp);
    assert_eq!(
        coded.as_slice(),
        seq06h_cod().as_slice(),
        "§4.3/§6.3.3.1: encoder-homing-frame must encode to SEQ06H.COD"
    );
}

/// §6.2.1 "codec homing implemented" — feeding *two* leading
/// encoder-homing-frames resets the encoder by software (the §4.3
/// homing path), so from the second frame onward the coded output
/// is the defined decoder-homing-frame. The §6.2.1 rule "the first
/// speech encoder output frame is undefined … all subsequent must
/// be identical" is exercised here: outputs 2..N are all SEQ06H.COD.
#[test]
fn config1_repeated_homing_input_yields_defined_output_after_first() {
    let mut enc = make_encoder(&params()).unwrap();
    let inp = encoder_homing_frame_pcm();
    let want = seq06h_cod();

    // Feed five consecutive encoder-homing-frames.
    let mut outs = Vec::new();
    for _ in 0..5 {
        outs.push(encode_one(&mut enc, &inp));
    }
    // From the §4.3-reset state (which the very first homing frame
    // establishes after it is encoded), every output is SEQ06H.COD.
    // Output 1 is already defined here because make_encoder starts
    // in the home state, but the spec only guarantees outputs 2..N;
    // assert that weaker-but-sufficient property.
    for (i, out) in outs.iter().enumerate().skip(1) {
        assert_eq!(
            out.as_slice(),
            want.as_slice(),
            "§6.2.1: encoder output frame {} must be the defined SEQ06H.COD",
            i + 1
        );
    }
}

// ───────────────── §6.2.2 Configuration 2 (decoder) ─────────────────

/// §4.4 step 1 / §6.3.3.1 — feeding `SEQ06H.COD` (the
/// decoder-homing-frame) to the decoder substitutes the
/// encoder-homing-frame (160 × `0x0008`) at the output, at the byte
/// level through the public `make_decoder` registry adapter. This is
/// the §6.2.2 decoder-under-test configuration run on the only `*.COD`
/// vector the staged PDF fully defines.
#[test]
fn config2_decoder_emits_encoder_homing_frame_for_seq06h_cod() {
    let mut dec = make_decoder(&params()).unwrap();
    let out = decode_one(&mut dec, &seq06h_cod());
    assert_eq!(
        out,
        seq06h_inp(),
        "§4.4: SEQ06H.COD decodes to 160 × 0x0008"
    );
}

/// §6.2.2 "codec homing implemented" — "Each *H.COD file includes
/// two homing frames at the start … the first speech decoder output
/// frame is undefined … all subsequent must be identical." Here we
/// build a small `*H.COD`-shaped stream: two leading decoder-homing
/// frames, then a non-homing speech frame. The two homing frames
/// both emit the encoder-homing-frame, and — critically — the third
/// (speech) frame decodes from the *homed* state, so it is identical
/// whether or not it was preceded by an arbitrarily dirty history.
#[test]
fn config2_two_leading_homing_frames_define_subsequent_output() {
    let speech = {
        // A non-homing coded frame: perturb the homing payload so it
        // is decoded as ordinary speech, not substituted.
        let mut f = decoder_homing_frame();
        f.lar_c[1] = (f.lar_c[1] + 5) & 0x3F;
        f.sub[0].xmax_c = 21;
        f.to_bit_stream_msb_first()
    };
    let ehf = seq06h_inp();

    // Stream A: clean decoder, two homing frames, then speech.
    let mut a = make_decoder(&params()).unwrap();
    assert_eq!(decode_one(&mut a, &seq06h_cod()), ehf, "homing frame 1");
    assert_eq!(decode_one(&mut a, &seq06h_cod()), ehf, "homing frame 2");
    let a_speech = decode_one(&mut a, &speech);

    // Stream B: same suffix but preceded by a noisy frame that
    // leaves the decoder in an arbitrary (non-home) state first.
    let mut b = make_decoder(&params()).unwrap();
    let noisy = {
        let mut f = UnpackedFrame::default();
        f.lar_c[3] = 12;
        f.sub[0].xmax_c = 33;
        f.sub[2].n_c = 90;
        f.to_bit_stream_msb_first()
    };
    let _ = decode_one(&mut b, &noisy);
    assert_eq!(
        decode_one(&mut b, &seq06h_cod()),
        ehf,
        "homing frame 1 (dirty)"
    );
    assert_eq!(
        decode_one(&mut b, &seq06h_cod()),
        ehf,
        "homing frame 2 (dirty)"
    );
    let b_speech = decode_one(&mut b, &speech);

    // §6.2.2: the post-homing speech frame is fully defined — the two
    // leading homing frames erased the divergent history, so A and B
    // produce identical samples regardless of what preceded them.
    assert_eq!(
        a_speech, b_speech,
        "§6.2.2: output after two leading homing frames must be defined (history-independent)"
    );

    // And the defined output is genuinely 13-bit shaped (§5.3.7).
    for pair in a_speech.chunks_exact(2) {
        let s = i16::from_le_bytes([pair[0], pair[1]]);
        assert_eq!(s & 0b111, 0, "§5.3.7 output low 3 bits must be zero");
    }
}

/// §6.3.3.2 — once the decoder is homed, a *fractional*
/// decoder-homing-frame (only the LARs and first sub-frame of the
/// homing frame; remaining sub-frames arbitrary) still homes it.
/// Driven end-to-end through the `Packet` decoder adapter.
#[test]
fn config2_fractional_homing_frame_homes_a_homed_decoder() {
    let mut dec = make_decoder(&params()).unwrap();

    // Bring the decoder home with a complete homing frame.
    assert_eq!(decode_one(&mut dec, &seq06h_cod()), seq06h_inp());

    // Build a fractional homing frame: keep the homing LARs and
    // sub-frame 0, fill sub-frames 1..=3 with unrelated data.
    let frac = {
        let mut f = decoder_homing_frame();
        for sf in &mut f.sub[1..=3] {
            sf.n_c = 0x55;
            sf.xmax_c = 0x2A;
            sf.x_mc = [6; oxideav_gsm::PULSES];
        }
        f.to_bit_stream_msb_first()
    };
    assert_eq!(
        decode_one(&mut dec, &frac),
        seq06h_inp(),
        "§6.3.3.2: fractional homing frame must emit the encoder-homing-frame"
    );
}

// ─────────────── §6.2 loop-back (encoder ⇄ decoder) ───────────────

/// §4.1 loop-back — the SEQ06H vectors close a full circle through
/// the public registry adapters: encode the encoder-homing-frame
/// (Config 1) to get SEQ06H.COD, then decode that (Config 2) back to
/// the encoder-homing-frame. The byte forms match the §6.3.3.1
/// reference vectors at both ends.
#[test]
fn loopback_seq06h_through_both_registry_adapters() {
    let mut enc = make_encoder(&params()).unwrap();
    let mut dec = make_decoder(&params()).unwrap();

    let inp = encoder_homing_frame_pcm();
    let coded = encode_one(&mut enc, &inp);
    assert_eq!(
        coded.as_slice(),
        seq06h_cod().as_slice(),
        "encode == SEQ06H.COD"
    );

    let decoded = decode_one(&mut dec, &coded);
    assert_eq!(decoded, seq06h_inp(), "decode(SEQ06H.COD) == SEQ06H.INP");
}

/// §6.2 sustained-stream sanity — a multi-frame coded stream
/// delivered as a single multi-frame packet decodes to the right
/// total sample count with every sample 13-bit shaped (§5.3.7), and
/// a trailing `NeedMore`/`Eof` discipline that a pipeline relies on.
#[test]
fn config2_multiframe_stream_decodes_to_shaped_output() {
    let mut dec = make_decoder(&params()).unwrap();

    // Four distinct coded frames concatenated into one packet.
    let mut stream = Vec::new();
    for k in 0..4u8 {
        let mut f = UnpackedFrame::default();
        f.lar_c[1] = (k as i16 * 3) & 0x3F;
        f.sub[0].xmax_c = 10 + k;
        f.sub[1].n_c = 50 + k;
        f.sub[2].m_c = k & 0x3;
        stream.extend_from_slice(&f.to_bit_stream_msb_first());
    }
    dec.send_packet(&packet(stream)).unwrap();
    let frame = dec.receive_frame().unwrap();
    match frame {
        Frame::Audio(a) => {
            assert_eq!(a.samples, (FRAME_SAMPLES * 4) as u32);
            for pair in a.data[0].chunks_exact(2) {
                let s = i16::from_le_bytes([pair[0], pair[1]]);
                assert_eq!(s & 0b111, 0, "§5.3.7 shaping");
            }
        }
        _ => panic!("expected audio frame"),
    }
    // No more buffered packets ⇒ NeedMore (not Eof — not flushed).
    assert!(matches!(dec.receive_frame(), Err(CoreError::NeedMore)));
}
