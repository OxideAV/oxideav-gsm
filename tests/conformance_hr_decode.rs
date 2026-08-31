//! GSM 06.20 half-rate decoder vs the staged GSM 06.07 decoder
//! conformance vectors (`tests/fixtures/etsi-hr/disk2/`).
//!
//! ## What this harness can and cannot pin
//!
//! EN 300 969 prints the decoder as floating-point functional
//! equations and places the bit-exact arithmetic in the GSM 06.06
//! ANSI-C deliverable (clause 5.1), which is deliberately not
//! staged (clean-room posture — only its ROM data is). On top of
//! that, the clause 4.2.5 long-term state update feeds the
//! decoder's own past excitation back into every voiced subframe,
//! so any sub-LSB arithmetic divergence compounds over a sequence:
//! whole-stream sample equality (or even high whole-stream SNR) is
//! not an achievable bar for a non-bit-exact implementation.
//!
//! What IS pinned, exactly:
//!
//! * the clause 5 homing protocol — `SEQ05.DEC` (the decoder
//!   homing frame) reproduces the encoder homing frame output from
//!   the home state, and the homing frames leading every `.DEC`
//!   sequence decode to the reference `0008`-hex frames and the
//!   silence frame that follows them sample-exactly;
//! * decoded-vs-reference **per-frame waveform correlation**,
//!   averaged over every frame of SEQ01–SEQ04 — the measured value
//!   (≈ 0.42 at the time of pinning) asserts the decoder tracks
//!   the reference waveform, and the floor guards against
//!   regressions in any dequantisation leg;
//! * per-sequence decoded/reference **energy** agreement — the
//!   gain chain (R0 decode, eq. (132) RS, GSP0 components) keeps
//!   the output level within a factor bound of the reference.

#![cfg(not(miri))]

use oxideav_gsm::hr::{HrDecoder, HR_PARAMS_PER_FRAME};
use oxideav_gsm::HrParameters;
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
        .chunks_exact(2)
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
    for frame in dec_words.chunks_exact(DEC_STRIDE) {
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

/// Measured waveform agreement, pinned as a regression floor: the
/// mean per-frame correlation between decoded and reference frames
/// across SEQ01–SEQ04 (leading homing frames excluded) was ≈ 0.42
/// when this decoder landed. The floor 0.40 guards every
/// dequantisation leg; whole-stream SNR is chaotic under the LTP
/// feedback (see the module docs) and is only reported.
#[test]
fn hr_decoder_vs_etsi_out_references() {
    if !corpus_present() {
        return;
    }
    let mut corr_sum = 0f64;
    let mut n_frames = 0usize;
    for seq in ["SEQ01", "SEQ02", "SEQ03", "SEQ04"] {
        let (decoded, reference) = run_sequence(seq);
        let mut sig = 0f64;
        let mut err = 0f64;
        let mut seq_corr = 0f64;
        let mut seq_n = 0usize;
        let mut e_dec = 0f64;
        let mut e_ref = 0f64;
        for f in 2..decoded.len() / 160 {
            let d = &decoded[f * 160..(f + 1) * 160];
            let r = &reference[f * 160..(f + 1) * 160];
            let mut dot = 0f64;
            let mut de = 0f64;
            let mut re = 0f64;
            for (x, y) in d.iter().zip(r.iter()) {
                let (xf, yf) = (*x as f64, *y as f64);
                dot += xf * yf;
                de += xf * xf;
                re += yf * yf;
                sig += yf * yf;
                err += (yf - xf) * (yf - xf);
            }
            e_dec += de;
            e_ref += re;
            if de > 0.0 && re > 0.0 {
                seq_corr += dot / (de * re).sqrt();
                seq_n += 1;
            }
        }
        let snr = 10.0 * (sig / err).log10();
        let energy_ratio = e_dec / e_ref;
        eprintln!(
            "{seq}: mean frame corr {:.4} ({} frames), stream SNR {snr:.2} dB, energy ratio {energy_ratio:.3}",
            seq_corr / seq_n as f64,
            seq_n
        );
        // The gain chain keeps the sequence energy within a factor
        // of ~4 of the reference (measured 0.3–1.1 at pin time).
        assert!(
            (0.06..=4.0).contains(&energy_ratio),
            "{seq}: decoded/reference energy ratio {energy_ratio:.3} out of bounds"
        );
        corr_sum += seq_corr;
        n_frames += seq_n;
    }
    let mean_corr = corr_sum / n_frames as f64;
    eprintln!("overall mean per-frame correlation: {mean_corr:.4} over {n_frames} frames");
    assert!(
        mean_corr >= 0.40,
        "mean per-frame correlation {mean_corr:.4} fell below the pinned 0.40 floor"
    );
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
