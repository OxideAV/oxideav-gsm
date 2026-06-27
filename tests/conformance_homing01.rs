//! §6.3.3.2 `HOMING01` decoder-homing state-machine conformance.
//!
//! ETSI EN 300 961 §6.3.3.2 ("Sequence for an extensive test of the
//! decoder homing") describes the `HOMING01.COD/OUT` reference vector:
//!
//! > *"If the decoder receives a complete decoder-homing-frame (which
//! > is not marked as a bad frame), then it is sufficient that the
//! > following frame contains only the LARs and the first subframe
//! > data of the decoder-homing-frame to cause a decoder reset and the
//! > output of the encoder-homing-frame. To check this behaviour the
//! > test sequence HOMING01.COD/OUT was produced. As the test
//! > sequences SEQ01H...05H the sequence HOMING01 contains 2 complete
//! > decoder-homing-frames at the beginning and inside there is a
//! > mixture of complete and fractional (incomplete)
//! > decoder-homing-frames."* (§6.3.3.2)
//!
//! The reference `HOMING01.COD` / `HOMING01.OUT` *binary* files ship in
//! the ETSI conformance archive, which is **not** staged under
//! `docs/audio/gsm/`. But the sequence's *defining behaviour* — and the
//! exact §4.4 / §4.4-NOTE-2 state-machine transitions it checks — are
//! fully specified in the staged PDF. This harness reconstructs a
//! `HOMING01`-shaped coded stream from the spec (two leading complete
//! decoder-homing-frames, then a mixture of complete + fractional homing
//! frames interleaved with ordinary speech) and drives it through the
//! **public registry decoder adapter** (`make_decoder` + `Packet`),
//! asserting the §6.3.3.2 reset behaviour frame-by-frame at the byte
//! level.
//!
//! The state-machine rules pinned here (all in-PDF):
//!
//! 1. §4.4 step 1 — a **non-home** decoder homes only on a **complete**
//!    decoder-homing-frame; a fractional one is decoded as ordinary
//!    speech (NOTE 2's soundness argument holds only from the home
//!    state).
//! 2. §4.4 NOTE 2 / §6.3.3.2 — a **homed** decoder homes on a
//!    **fractional** homing frame (LARs + first sub-frame of the homing
//!    frame, sub-frames 2..=4 arbitrary).
//! 3. §4.4 step 2 — every homing event emits the encoder-homing-frame
//!    (160 × `0x0008`) and resets all state to §4.6 home.
//! 4. §6.2.2 history-independence — the post-homing output is fully
//!    defined regardless of the divergent history that preceded it.

use oxideav_core::{CodecId, CodecParameters, Frame, Packet, SampleFormat, TimeBase};
use oxideav_gsm::{
    decoder_homing_frame, encoder_homing_frame_pcm, make_decoder, UnpackedFrame, CODEC_ID,
    FRAME_SAMPLES, PULSES,
};

fn params() -> CodecParameters {
    let mut p = CodecParameters::audio(CodecId::new(CODEC_ID));
    p.channels = Some(1);
    p.sample_rate = Some(8000);
    p.sample_format = Some(SampleFormat::S16);
    p
}

fn packet(data: Vec<u8>) -> Packet {
    Packet::new(0, TimeBase::new(1, 8000), data)
}

/// 160 little-endian S16 samples of `0x0008` — the encoder-homing-frame
/// PCM (§4.2), i.e. one `HOMING01.OUT` reset frame.
fn ehf_bytes() -> Vec<u8> {
    let mut v = Vec::with_capacity(FRAME_SAMPLES * 2);
    for s in encoder_homing_frame_pcm() {
        v.extend_from_slice(&s.to_le_bytes());
    }
    v
}

/// `SEQ06H.COD` — one complete decoder-homing-frame on the wire.
fn complete_homing_cod() -> [u8; 33] {
    decoder_homing_frame().to_bit_stream_msb_first()
}

/// A **fractional** decoder-homing-frame: the homing LARs and sub-frame
/// 0 intact, sub-frames 1..=3 carrying unrelated data. Per §4.4 NOTE 2
/// this homes a *homed* decoder, but is plain speech to a non-home one.
fn fractional_homing_cod(salt: u8) -> [u8; 33] {
    let mut f = decoder_homing_frame();
    for (i, sf) in f.sub[1..=3].iter_mut().enumerate() {
        sf.n_c = 0x40 + salt + i as u8;
        sf.b_c = (salt + i as u8) & 0x3;
        sf.m_c = (salt.wrapping_add(i as u8)) & 0x3;
        sf.xmax_c = 0x10 + salt;
        sf.x_mc = [(salt as usize % 8) as u8; PULSES];
    }
    f.to_bit_stream_msb_first()
}

/// An ordinary (non-homing) speech frame.
fn speech_cod(salt: u8) -> [u8; 33] {
    let mut f = UnpackedFrame::default();
    f.lar_c[1] = (salt as i16 * 3 + 1) & 0x3F;
    f.lar_c[3] = (salt as i16 + 5) & 0x1F;
    f.sub[0].xmax_c = 12 + (salt & 0x1F);
    f.sub[0].n_c = 60 + salt;
    f.sub[1].m_c = salt & 0x3;
    f.sub[2].x_mc = [3; PULSES];
    f.to_bit_stream_msb_first()
}

/// Decode one 33-byte coded frame through the registry adapter and
/// return its 160 little-endian S16 samples.
fn decode_one(dec: &mut Box<dyn oxideav_core::Decoder>, cod: &[u8]) -> Vec<u8> {
    dec.send_packet(&packet(cod.to_vec())).unwrap();
    match dec.receive_frame().unwrap() {
        Frame::Audio(a) => {
            assert_eq!(a.samples, FRAME_SAMPLES as u32);
            a.data.into_iter().next().unwrap()
        }
        _ => panic!("expected audio frame"),
    }
}

fn is_reset_output(out: &[u8]) -> bool {
    out == ehf_bytes().as_slice()
}

/// §6.3.3.2 — the full `HOMING01`-shaped sequence. Two complete homing
/// frames at the start, then a mixture of complete + fractional homing
/// frames interleaved with speech. Every homing event (whichever form)
/// must emit the encoder-homing-frame; speech frames in between must
/// NOT (they decode normally).
#[test]
fn homing01_full_sequence_resets_on_every_homing_event() {
    let mut dec = make_decoder(&params()).unwrap();

    // Each step: (label, coded frame, expect_reset_output).
    // The "expect" column encodes the §4.4 state machine: a homing
    // event only happens when the *current* decoder state admits the
    // frame as homing (complete always; fractional only when homed).
    //
    // State trace (H = homed at frame entry):
    //   start: H
    enum Step {
        Complete,
        Fractional(u8),
        Speech(u8),
    }
    use Step::*;

    // HOMING01 shape: 2 complete leaders, then the mixture.
    let seq = [
        Complete,      // 1: H  -> reset, stays H
        Complete,      // 2: H  -> reset, stays H   (2 complete leaders)
        Fractional(1), // 3: H  -> reset (NOTE 2),   stays H
        Speech(7),     // 4: H  -> speech, leaves H
        Complete,      // 5: !H -> reset (complete),  back to H
        Fractional(2), // 6: H  -> reset (NOTE 2),    stays H
        Fractional(3), // 7: H  -> reset (NOTE 2),    stays H
        Speech(9),     // 8: H  -> speech, leaves H
        Fractional(4), // 9: !H -> NOT homing -> speech, stays !H
        Speech(11),    // 10:!H -> speech, stays !H
        Complete,      // 11:!H -> reset (complete),  back to H
        Speech(13),    // 12:H  -> speech, leaves H
        Complete,      // 13:!H -> reset (complete),  back to H
    ];

    // Track homed-state expectation alongside the decoder.
    let mut homed = true; // make_decoder starts in §4.6 home state.
    for (i, step) in seq.iter().enumerate() {
        let (cod, expect_reset): ([u8; 33], bool) = match step {
            Complete => {
                // Complete homing frame always homes, from any state.
                (complete_homing_cod(), true)
            }
            Fractional(s) => {
                // Fractional homes ONLY a homed decoder (§4.4 NOTE 2).
                (fractional_homing_cod(*s), homed)
            }
            Speech(s) => (speech_cod(*s), false),
        };
        let out = decode_one(&mut dec, &cod);
        assert_eq!(
            is_reset_output(&out),
            expect_reset,
            "frame {} ({}): reset expectation",
            i + 1,
            match step {
                Complete => "complete-homing".to_string(),
                Fractional(s) => format!("fractional-homing[{s}]"),
                Speech(s) => format!("speech[{s}]"),
            }
        );
        // Update the homed expectation: a reset event ⇒ homed; a
        // decoded speech frame ⇒ not homed.
        homed = expect_reset;
    }
}

/// §4.4 step 1 — a **non-home** decoder must NOT home on a fractional
/// frame; it decodes it as ordinary speech. This is the §6.3.3.2
/// soundness boundary: NOTE 2's reduced criterion is only valid from
/// the home state.
#[test]
fn fractional_does_not_home_a_nonhome_decoder() {
    let mut dec = make_decoder(&params()).unwrap();
    // Leave the home state with a speech frame.
    let s = decode_one(&mut dec, &speech_cod(5));
    assert!(!is_reset_output(&s), "speech frame is not a reset");
    // Now a fractional homing frame: must decode as speech, not reset.
    let frac = decode_one(&mut dec, &fractional_homing_cod(1));
    assert!(
        !is_reset_output(&frac),
        "§4.4 NOTE 2: a non-home decoder must not home on a fractional frame"
    );
    // A *complete* homing frame, by contrast, homes it.
    let comp = decode_one(&mut dec, &complete_homing_cod());
    assert!(
        is_reset_output(&comp),
        "§4.4 step 1: a complete homing frame homes a non-home decoder"
    );
}

/// §6.2.2 history-independence — the post-homing speech output is fully
/// defined: two decoders fed the same `HOMING01` suffix but different
/// prefixes converge to identical samples after the homing event.
#[test]
fn homing01_output_is_history_independent_after_reset() {
    let suffix = [speech_cod(3), speech_cod(8), complete_homing_cod()];
    // We compare the frame *after* the trailing complete homing reset.
    let probe = speech_cod(21);

    // Decoder A: clean start, then suffix.
    let mut a = make_decoder(&params()).unwrap();
    for c in &suffix {
        let _ = decode_one(&mut a, c);
    }
    let a_out = decode_one(&mut a, &probe);

    // Decoder B: a noisy divergent prefix, then the SAME suffix.
    let mut b = make_decoder(&params()).unwrap();
    for s in 30..36u8 {
        let _ = decode_one(&mut b, &speech_cod(s));
    }
    for c in &suffix {
        let _ = decode_one(&mut b, c);
    }
    let b_out = decode_one(&mut b, &probe);

    assert_eq!(
        a_out, b_out,
        "§6.2.2: output after a homing reset must be history-independent"
    );
    // And shaped per §5.3.7.
    for pair in a_out.chunks_exact(2) {
        let v = i16::from_le_bytes([pair[0], pair[1]]);
        assert_eq!(v & 0b111, 0, "§5.3.7 output low 3 bits must be zero");
    }
}

/// §4.4 step 2 — every reset output is exactly 160 × `0x0008`
/// (the encoder-homing-frame), regardless of which homing form
/// triggered it.
#[test]
fn every_reset_output_is_the_encoder_homing_frame() {
    let mut dec = make_decoder(&params()).unwrap();
    // Complete homing.
    assert!(is_reset_output(&decode_one(
        &mut dec,
        &complete_homing_cod()
    )));
    // Fractional homing from the now-homed state, several salts.
    for s in 0..5u8 {
        let out = decode_one(&mut dec, &fractional_homing_cod(s));
        assert!(is_reset_output(&out), "fractional reset {s}");
        // Byte-exact: each is 0x0008.
        for pair in out.chunks_exact(2) {
            assert_eq!(i16::from_le_bytes([pair[0], pair[1]]), 0x0008);
        }
    }
}
