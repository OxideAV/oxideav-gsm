//! GSM 06.32 VAD digital test sequences — the official 20-case corpus
//! (ETSI EN 300 965 clause 4 + annexes A.2/B), run bit-exactly.
//!
//! `tests/fixtures/etsi-vad/` stages the EN 300 965 V8.0.1 electronic
//! attachment's data files (see the fixtures README). Each test case
//! is a triple:
//!
//! * `*.INP` — 13-bit uniform-PCM speech-encoder input (16-bit LE
//!   words, 160 per frame);
//! * `*.COD` — the encoder's coded parameters (76 words per frame)
//!   with the VAD flag in bit 15 of the first word (LAR(1)) and the
//!   GSM 06.31 SP flag in bit 15 of the second (LAR(2)) — clause 4.1;
//! * `*.VAD` — ASCII, one `"<0|1> \r\n"` record per frame: the
//!   reference VAD flag.
//!
//! Configuration (clause 4.1): the VAD runs in conjunction with the
//! GSM 06.10 encoder (already verified bit-exactly by
//! `tests/conformance_etsi_fr.rs`), consuming the encoder-internal
//! variables (`L_ACF`, `scalauto`, `Nc` lags, `sof`). The corpus
//! includes `FREQ_SW` (tone detector via frequency sweep), which is
//! "downlink VAD only" (clause A.2.1) — the whole corpus is run in
//! downlink mode, i.e. with clause 3.10 tone detection active.
//!
//! Every case asserts, frame by frame:
//!
//! 1. our VAD flag equals the `*.VAD` reference record;
//! 2. our VAD flag equals bit 15 of the `*.COD` LAR(1) word (the
//!    same information carried in the parameter file);
//! 3. our *encoder* reproduces the `*.COD` frame bit-exactly wherever
//!    the frame is a speech frame (SP=1) — during DTX (SP=0) the
//!    `*.COD` holds SID frames produced by the GSM 06.31 TX DTX
//!    handler instead of the raw encoder output, so those are
//!    exercised by the TX-DTX conformance harness, not here.

#![cfg(not(miri))]

use oxideav_gsm::{EncoderState, UnpackedFrame, Vad, VadMode, FRAME_SAMPLES};
use std::path::PathBuf;

fn fixture_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("fixtures")
        .join("etsi-vad")
}

fn corpus_present() -> bool {
    let present = fixture_dir().is_dir();
    if !present {
        eprintln!("etsi-vad corpus not present (published package?) — skipping");
    }
    present
}

fn read_pcm_frames(name: &str) -> Vec<[i16; FRAME_SAMPLES]> {
    let bytes =
        std::fs::read(fixture_dir().join(name)).unwrap_or_else(|e| panic!("fixture {name}: {e}"));
    assert_eq!(bytes.len() % 320, 0, "{name}: partial frame");
    bytes
        .chunks_exact(320)
        .map(|c| {
            let mut f = [0i16; FRAME_SAMPLES];
            for (s, ch) in f.iter_mut().zip(c.chunks_exact(2)) {
                *s = i16::from_le_bytes([ch[0], ch[1]]);
            }
            f
        })
        .collect()
}

/// One 76-word `*.COD` frame with its flag bits: `(words, vad, sp)`.
struct CodFrame {
    words: [u16; 76],
    vad: bool,
    sp: bool,
}

impl CodFrame {
    /// The frame's 76 parameters with the flag bits masked off.
    fn params(&self) -> UnpackedFrame {
        let mut w = self.words;
        w[0] &= 0x7FFF;
        w[1] &= 0x7FFF;
        oxideav_gsm::cod_words_to_unpacked(&w)
    }
}

fn read_cod_frames(name: &str) -> Vec<CodFrame> {
    let bytes =
        std::fs::read(fixture_dir().join(name)).unwrap_or_else(|e| panic!("fixture {name}: {e}"));
    assert_eq!(bytes.len() % 152, 0, "{name}: partial frame");
    bytes
        .chunks_exact(152)
        .map(|c| {
            let mut words = [0u16; 76];
            for (w, ch) in words.iter_mut().zip(c.chunks_exact(2)) {
                *w = u16::from_le_bytes([ch[0], ch[1]]);
            }
            CodFrame {
                words,
                vad: words[0] & 0x8000 != 0,
                sp: words[1] & 0x8000 != 0,
            }
        })
        .collect()
}

/// Parse the ASCII `*.VAD` file: one `"<0|1> \r\n"` record per frame.
fn read_vad_flags(name: &str) -> Vec<bool> {
    let bytes =
        std::fs::read(fixture_dir().join(name)).unwrap_or_else(|e| panic!("fixture {name}: {e}"));
    let text = String::from_utf8(bytes).unwrap_or_else(|e| panic!("fixture {name}: {e}"));
    text.split_ascii_whitespace()
        .map(|tok| match tok {
            "0" => false,
            "1" => true,
            other => panic!("{name}: unexpected VAD record {other:?}"),
        })
        .collect()
}

/// Run one corpus case end-to-end: encoder + downlink VAD over the
/// `*.INP` frames, checked against `*.VAD` and the `*.COD` flag bits
/// and speech-frame parameters.
fn run_case(case: &str) {
    if !corpus_present() {
        return;
    }
    let inp = read_pcm_frames(&format!("{case}.INP"));
    let cod = read_cod_frames(&format!("{case}.COD"));
    let vad_ref = read_vad_flags(&format!("{case}.VAD"));
    assert_eq!(inp.len(), cod.len(), "{case}: INP/COD frame count");
    assert_eq!(inp.len(), vad_ref.len(), "{case}: INP/VAD frame count");

    let mut enc = EncoderState::new();
    let mut vad = Vad::new(VadMode::Downlink);
    for (n, (pcm, (want_cod, want_vad))) in
        inp.iter().zip(cod.iter().zip(vad_ref.iter())).enumerate()
    {
        let (frame, tap) = enc.encode_frame_with_vad_tap(pcm);
        let got_vad = vad.process_frame(&tap);
        assert_eq!(
            got_vad, *want_vad,
            "{case}: VAD flag diverges from *.VAD at frame {n}"
        );
        assert_eq!(
            got_vad, want_cod.vad,
            "{case}: *.VAD and *.COD LAR(1) bit 15 disagree at frame {n}"
        );
        if want_cod.sp {
            assert_eq!(
                frame,
                want_cod.params(),
                "{case}: speech-frame parameters diverge at frame {n}"
            );
        }
    }
}

// Clause A.2.1 recommends running the initial sequences first: a
// fault in the VAD decision makes every other sequence fail.
#[test]
fn adapt_i1() {
    run_case("ADAPT_I1");
}

#[test]
fn adapt_i2() {
    run_case("ADAPT_I2");
}

#[test]
fn adapt_m1() {
    run_case("ADAPT_M1");
}

#[test]
fn adapt_m2() {
    run_case("ADAPT_M2");
}

#[test]
fn spec_a1() {
    run_case("SPEC_A1");
}

#[test]
fn spec_a2() {
    run_case("SPEC_A2");
}

#[test]
fn spec_c1() {
    run_case("SPEC_C1");
}

#[test]
fn spec_c2() {
    run_case("SPEC_C2");
}

#[test]
fn spec_c3() {
    run_case("SPEC_C3");
}

#[test]
fn spec_c4() {
    run_case("SPEC_C4");
}

#[test]
fn freq_sw() {
    run_case("FREQ_SW");
}

#[test]
fn pred1() {
    run_case("PRED1");
}

#[test]
fn pred2() {
    run_case("PRED2");
}

#[test]
fn pitch1() {
    run_case("PITCH1");
}

#[test]
fn pitch2() {
    run_case("PITCH2");
}

#[test]
fn pole1() {
    run_case("POLE1");
}

#[test]
fn pole2() {
    run_case("POLE2");
}

#[test]
fn safety() {
    run_case("SAFETY");
}

#[test]
fn good_sp() {
    run_case("GOOD_SP");
}

#[test]
fn bad_sp() {
    run_case("BAD_SP");
}
