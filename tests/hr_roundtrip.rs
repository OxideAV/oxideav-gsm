//! GSM 06.20 encoder → decoder self round trip: the clause 5
//! homing protocol sample-exact in both directions, encoder
//! determinism, and the waveform agreement of our own decode of our
//! own encode against the (35-sample delayed) input speech —
//! reported next to the reference decoder's agreement with the
//! same input where the corpus is present.

#![cfg(not(miri))]

use oxideav_gsm::hr::{
    hr_decoder_homing_frame, is_hr_decoder_homing_frame, HrDecoder, HrEncoder,
    HR_ENCODER_HOMING_SAMPLE,
};
use oxideav_gsm::HrParameters;
use std::path::PathBuf;

fn fixture_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("fixtures")
        .join("etsi-hr")
}

fn read_words(rel: &str) -> Option<Vec<i16>> {
    let bytes = std::fs::read(fixture_dir().join(rel)).ok()?;
    Some(
        bytes
            .chunks_exact(2)
            .map(|c| i16::from_le_bytes([c[0], c[1]]))
            .collect(),
    )
}

fn corr(a: &[i16], b: &[i16]) -> f64 {
    let (mut d, mut ea, mut eb) = (0f64, 0f64, 0f64);
    for (x, y) in a.iter().zip(b) {
        let (x, y) = (*x as f64, *y as f64);
        d += x * y;
        ea += x * x;
        eb += y * y;
    }
    if ea > 0.0 && eb > 0.0 {
        d / (ea * eb).sqrt()
    } else {
        0.0
    }
}

/// A deterministic speech-like test signal: a decaying pulse train
/// through a two-formant resonator, with a slow pitch glide.
fn synthetic_speech(frames: usize) -> Vec<i16> {
    let n = frames * 160;
    let mut out = Vec::with_capacity(n);
    let (mut y1, mut y2, mut z1, mut z2) = (0f64, 0f64, 0f64, 0f64);
    let mut next_pulse = 0usize;
    for i in 0..n {
        let period = 60.0 + 20.0 * ((i as f64) / n as f64);
        let mut x = 0.0;
        if i == next_pulse {
            x = 1.0;
            next_pulse = i + period.round() as usize;
        }
        // Resonators near 500 Hz and 1500 Hz.
        let (r1, w1) = (0.95, std::f64::consts::TAU * 500.0 / 8000.0);
        let (r2, w2) = (0.93, std::f64::consts::TAU * 1500.0 / 8000.0);
        let a = x + 2.0 * r1 * w1.cos() * y1 - r1 * r1 * y2;
        y2 = y1;
        y1 = a;
        let b = a + 2.0 * r2 * w2.cos() * z1 - r2 * r2 * z2;
        z2 = z1;
        z1 = b;
        let env = if i < 160 * 2 { i as f64 / 320.0 } else { 1.0 };
        let s = (b * 400.0 * env).clamp(-4095.0, 4095.0) as i16;
        out.push(s << 3);
    }
    out
}

/// Clause 5.3: encoder homing frames produce the decoder homing
/// frame (from the second one on, and — since the encoder starts
/// in its home state — from the first here); clause 5.4: the
/// decoder maps them back to encoder homing frames sample-exactly.
#[test]
fn homing_round_trip_is_sample_exact() {
    let mut enc = HrEncoder::new();
    let mut dec = HrDecoder::new();
    let homing_in = [HR_ENCODER_HOMING_SAMPLE; 160];
    for _ in 0..3 {
        let p = enc.encode_frame(&homing_in);
        assert!(is_hr_decoder_homing_frame(&p));
        assert_eq!(p, hr_decoder_homing_frame());
        let out = dec.decode_frame(&p);
        assert_eq!(out, homing_in);
    }
    // After speech, a homing frame still resets both sides: the
    // frame after it encodes exactly as from a fresh encoder.
    let speech = synthetic_speech(6);
    let mut fresh = HrEncoder::new();
    let frame = |k: usize| -> [i16; 160] { speech[k * 160..(k + 1) * 160].try_into().unwrap() };
    for k in 0..4 {
        let _ = enc.encode_frame(&frame(k));
    }
    let p = enc.encode_frame(&homing_in);
    assert!(is_hr_decoder_homing_frame(&p));
    for k in 4..6 {
        assert_eq!(enc.encode_frame(&frame(k)), fresh.encode_frame(&frame(k)));
    }
}

/// The corpus' SEQ05.INP (encoder homing sequence): every frame
/// codes as the decoder homing frame.
#[test]
fn seq05_homing_sequence_codes_as_decoder_homing_frames() {
    let Some(inp) = read_words("disk1/SEQ05.INP") else {
        eprintln!("etsi-hr corpus not present — skipping");
        return;
    };
    let mut enc = HrEncoder::new();
    for f in inp.chunks_exact(160) {
        let p = enc.encode_frame(f.try_into().unwrap());
        assert!(is_hr_decoder_homing_frame(&p));
    }
}

/// Encoding is deterministic and the annex-B / conformance-word
/// packings of the encoder's output round-trip.
#[test]
fn encoder_is_deterministic_and_packable() {
    let speech = synthetic_speech(12);
    let mut a = HrEncoder::new();
    let mut b = HrEncoder::new();
    let mut modes_seen = [false; 4];
    for f in speech.chunks_exact(160) {
        let f: &[i16; 160] = f.try_into().unwrap();
        let pa = a.encode_frame(f);
        let pb = b.encode_frame(f);
        assert_eq!(pa, pb);
        modes_seen[pa.mode_code as usize] = true;
        assert_eq!(HrParameters::from_bits(&pa.to_bits()).unwrap(), pa);
        assert_eq!(HrParameters::from_cod_words(&pa.to_cod_words()), pa);
    }
    assert!(modes_seen.iter().any(|&m| m), "some frame was coded");
}

/// Our decode of our encode tracks the input (delayed by the
/// clause 4.1.2 look-ahead of 35 samples) on the synthetic signal
/// (measured ≈ 0,50 when the chain landed — the decoder's adaptive
/// postfilter reshapes the waveform, so even a perfect codec does
/// not reach 1,0 here).
#[test]
fn synthetic_round_trip_tracks_input() {
    let speech = synthetic_speech(24);
    let mut enc = HrEncoder::new();
    let mut dec = HrDecoder::new();
    let mut out = Vec::new();
    for f in speech.chunks_exact(160) {
        let p = enc.encode_frame(f.try_into().unwrap());
        out.extend_from_slice(&dec.decode_frame(&p));
    }
    let mut c = 0.0;
    let mut n = 0;
    for k in 4..24 {
        c += corr(
            &out[k * 160..(k + 1) * 160],
            &speech[k * 160 - 35..(k + 1) * 160 - 35],
        );
        n += 1;
    }
    let mean = c / n as f64;
    eprintln!("synthetic round trip mean per-frame correlation {mean:.3}");
    assert!(mean >= 0.42, "round trip correlation {mean:.3}");
}

/// Corpus speech: our decode of our encode vs the input (delayed
/// 35 samples), next to the reference decoder's own agreement with
/// the same input (`SEQxx.OUT`, ≈ 0,52–0,68 per sequence). Measured
/// ≈ 0,36–0,52 per sequence (0,40 overall) when the chain landed;
/// the floor guards the whole analysis-by-synthesis loop.
#[test]
fn corpus_round_trip_tracks_input() {
    let Some(_) = read_words("disk1/SEQ01.INP") else {
        eprintln!("etsi-hr corpus not present — skipping");
        return;
    };
    let mut total = 0.0;
    let mut n_all = 0usize;
    for seq in ["SEQ01", "SEQ02", "SEQ03"] {
        let inp = read_words(&format!("disk1/{seq}.INP")).unwrap();
        let refo = read_words(&format!("disk2/{seq}.OUT")).unwrap();
        let mut enc = HrEncoder::new();
        let mut dec = HrDecoder::new();
        let mut out = Vec::with_capacity(inp.len());
        for f in inp.chunks_exact(160) {
            let p = enc.encode_frame(f.try_into().unwrap());
            out.extend_from_slice(&dec.decode_frame(&p));
        }
        let frames = inp.len() / 160;
        let (mut ours, mut refc) = (0.0, 0.0);
        for k in 3..frames {
            let target = &inp[k * 160 - 35..(k + 1) * 160 - 35];
            ours += corr(&out[k * 160..(k + 1) * 160], target);
            refc += corr(&refo[k * 160..(k + 1) * 160], target);
        }
        let n = (frames - 3) as f64;
        eprintln!(
            "{seq}: our encode→decode vs input {:.3}; reference decoder vs input {:.3}",
            ours / n,
            refc / n
        );
        total += ours;
        n_all += frames - 3;
    }
    let mean = total / n_all as f64;
    eprintln!("corpus round trip mean per-frame correlation {mean:.3}");
    assert!(mean >= 0.34, "round trip correlation {mean:.3}");
}
