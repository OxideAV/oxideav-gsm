//! GSM 06.20 half-rate decoder vs the staged GSM 06.07 decoder
//! conformance vectors (`tests/fixtures/etsi-hr/disk2/`).
//!
//! What is pinned, exactly:
//!
//! * every output word of SEQ01–SEQ04 (bit-exact decoding, the
//!   clause 5.1 conformance bar);
//! * the clause 5 homing protocol — `SEQ05.DEC` (the decoder
//!   homing frame) reproduces the encoder homing frame output from
//!   the home state, and a mid-stream homing frame fully resets the
//!   decoder.

#![cfg(not(miri))]

use oxideav_gsm::HrParameters;
use oxideav_gsm::hr::{HR_PARAMS_PER_FRAME, HrDecoder};
use std::path::PathBuf;

const DEC_STRIDE: usize = HR_PARAMS_PER_FRAME + 4;

fn fixture_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("fixtures")
        .join("etsi-hr")
}

fn corpus_present() -> bool {
    let present = fixture_dir().is_dir();
    if !present {
        eprintln!("etsi-hr corpus not present (published package?) — skipping");
    }
    present
}

fn read_words(rel: &str) -> Vec<u16> {
    let bytes =
        std::fs::read(fixture_dir().join(rel)).unwrap_or_else(|e| panic!("fixture {rel}: {e}"));
    bytes
        .as_chunks::<2>()
        .0
        .iter()
        .map(|c| u16::from_le_bytes([c[0], c[1]]))
        .collect()
}

/// Decode one `.DEC` stream and return (decoded, reference) PCM.
fn run_sequence(seq: &str) -> (Vec<i16>, Vec<i16>) {
    let dec_words = read_words(&format!("disk2/{seq}.DEC"));
    let out_words = read_words(&format!("disk2/{seq}.OUT"));
    assert_eq!(dec_words.len() % DEC_STRIDE, 0);
    let mut dec = HrDecoder::new();
    let mut decoded = Vec::new();
    for frame in dec_words.as_chunks::<DEC_STRIDE>().0 {
        let mut params = [0u16; HR_PARAMS_PER_FRAME];
        params.copy_from_slice(&frame[..HR_PARAMS_PER_FRAME]);
        // Error-free sequences: BFI = UFI = SID = 0 (frame[18..21]).
        assert_eq!(&frame[18..21], &[0, 0, 0], "{seq}: error-free corpus");
        let p = HrParameters::from_cod_words(&params);
        decoded.extend_from_slice(&dec.decode_frame(&p));
    }
    let reference: Vec<i16> = out_words.iter().map(|&w| w as i16).collect();
    assert_eq!(decoded.len(), reference.len(), "{seq}: length");
    (decoded, reference)
}

/// The two leading homing frames of every `.DEC` sequence must
/// reproduce the reference exactly: the first homing frame finds
/// the decoder already in its home state (fresh construction), so
/// both decode to the encoder homing frame, as the `.OUT`
/// references do.
#[test]
fn leading_homing_frames_are_sample_exact() {
    if !corpus_present() {
        return;
    }
    for seq in ["SEQ01", "SEQ02", "SEQ03", "SEQ04"] {
        let (decoded, reference) = run_sequence(seq);
        assert_eq!(
            &decoded[..320],
            &reference[..320],
            "{seq}: leading homing frames must be sample-exact"
        );
    }
}

/// Bit-exact decoding: every frame of SEQ01–SEQ04 (3 621 speech
/// frames plus the leading homing frames) must reproduce the
/// reference `.OUT` words exactly. The decoder carries the clause
/// 5.1 fixed-point arithmetic per the staged half-rate schedules
/// (`docs/audio/gsm/halfrate-traces/hr-fixed-point-schedules.md`
/// Part A).
#[test]
fn hr_decoder_bit_exact_vs_etsi_out_references() {
    if !corpus_present() {
        return;
    }
    let mut total = 0usize;
    for seq in ["SEQ01", "SEQ02", "SEQ03", "SEQ04"] {
        let (decoded, reference) = run_sequence(seq);
        let frames = decoded.len() / 160;
        let exact = (0..frames)
            .filter(|f| decoded[f * 160..(f + 1) * 160] == reference[f * 160..(f + 1) * 160])
            .count();
        eprintln!("{seq}: {exact}/{frames} frames bit-exact");
        if let Some(i) = decoded
            .iter()
            .zip(reference.iter())
            .position(|(a, b)| a != b)
        {
            panic!(
                "{seq}: first mismatch at sample {i} (frame {}): decoded {} vs reference {}",
                i / 160,
                decoded[i],
                reference[i]
            );
        }
        total += frames;
    }
    assert_eq!(total, 2359 + 781 + 413 + 76);
}

/// SEQ05.DEC is the lone decoder homing frame: feeding it
/// repeatedly from a fresh (home-state) decoder yields the encoder
/// homing frame every time (clause 5.4 + GSM 06.07 clause 6.2.3).
#[test]
fn seq05_homing_frame_behaviour() {
    if !corpus_present() {
        return;
    }
    let words = read_words("disk2/SEQ05.DEC");
    assert_eq!(words.len(), DEC_STRIDE);
    let mut params = [0u16; HR_PARAMS_PER_FRAME];
    params.copy_from_slice(&words[..HR_PARAMS_PER_FRAME]);
    let p = HrParameters::from_cod_words(&params);
    assert!(oxideav_gsm::hr::is_hr_decoder_homing_frame(&p));
    let mut dec = HrDecoder::new();
    for _ in 0..3 {
        let out = dec.decode_frame(&p);
        assert!(out.iter().all(|&s| s == 0x0008));
    }
}

/// A decoder-homing frame mid-stream resets the decoder: decoding
/// [homing, X] from a warmed decoder equals decoding [homing, X]
/// from a fresh one on the X frame.
#[test]
fn midstream_homing_resets_state() {
    if !corpus_present() {
        return;
    }
    let dec_words = read_words("disk2/SEQ01.DEC");
    let homing = {
        let words = read_words("disk2/SEQ05.DEC");
        let mut params = [0u16; HR_PARAMS_PER_FRAME];
        params.copy_from_slice(&words[..HR_PARAMS_PER_FRAME]);
        HrParameters::from_cod_words(&params)
    };
    let x = {
        let mut params = [0u16; HR_PARAMS_PER_FRAME];
        params.copy_from_slice(&dec_words[10 * DEC_STRIDE..10 * DEC_STRIDE + 18]);
        HrParameters::from_cod_words(&params)
    };

    let mut warmed = HrDecoder::new();
    for f in 2..8 {
        let mut params = [0u16; HR_PARAMS_PER_FRAME];
        params.copy_from_slice(&dec_words[f * DEC_STRIDE..f * DEC_STRIDE + 18]);
        let _ = warmed.decode_frame(&HrParameters::from_cod_words(&params));
    }
    let _ = warmed.decode_frame(&homing);
    let warmed_x = warmed.decode_frame(&x);

    let mut fresh = HrDecoder::new();
    let _ = fresh.decode_frame(&homing);
    let fresh_x = fresh.decode_frame(&x);

    assert_eq!(warmed_x, fresh_x, "homing must fully reset decoder state");
}
