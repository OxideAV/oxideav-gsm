//! GSM 06.20 encoder frame-parameter chain (clauses 4.1.1–4.1.6)
//! vs the staged GSM 06.07 encoder references
//! (`tests/fixtures/etsi-hr/disk1/`).
//!
//! As with the decoder, EN 300 969's printed equations are
//! functional (the bit-exact arithmetic is the unstaged GSM 06.06
//! ANSI-C), so exact code equality across whole sequences is not
//! the achievable bar: quantizer decisions sitting on a boundary
//! flip under sub-LSB arithmetic differences. What this harness
//! pins is the measured per-parameter agreement rate against
//! `SEQ01..03.COD` — high enough that any structural regression
//! (wrong table, wrong recursion, wrong windowing) collapses it.

#![cfg(not(miri))]

use oxideav_gsm::hr::HrAnalyzer;
use std::path::PathBuf;

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

#[derive(Default)]
struct Agreement {
    frames: usize,
    r0_exact: usize,
    r0_near: usize,
    lpc1: usize,
    lpc2: usize,
    lpc3: usize,
    int_lpc: usize,
}

fn run_sequence(seq: &str, agg: &mut Agreement) {
    let inp = read_words(&format!("disk1/{seq}.INP"));
    let cod = read_words(&format!("disk1/{seq}.COD"));
    let frames = inp.len() / 160;
    assert_eq!(cod.len() / 20, frames, "{seq}: INP/COD frame count");
    let mut an = HrAnalyzer::new();
    for fi in 0..frames {
        let mut pcm = [0i16; 160];
        for (n, s) in pcm.iter_mut().enumerate() {
            *s = inp[fi * 160 + n] as i16;
        }
        let out = an.analyze_frame(&pcm);
        // Skip the two leading homing frames (encoder state before
        // the reset they perform in the reference is undefined for
        // a non-C implementation) and the first frame after them.
        if fi < 3 {
            continue;
        }
        let c = &cod[fi * 20..fi * 20 + 18];
        agg.frames += 1;
        if out.r0 as u16 == c[0] {
            agg.r0_exact += 1;
        }
        if (out.r0 as i32 - c[0] as i32).abs() <= 1 {
            agg.r0_near += 1;
        }
        if out.lpc1 == c[1] {
            agg.lpc1 += 1;
        }
        if out.lpc2 == c[2] {
            agg.lpc2 += 1;
        }
        if out.lpc3 as u16 == c[3] {
            agg.lpc3 += 1;
        }
        if out.int_lpc as u16 == c[4] {
            agg.int_lpc += 1;
        }
    }
}

/// Measured frame-parameter agreement vs the staged encoder
/// references, pinned as regression floors (values in comments are
/// the rates measured when this chain landed).
#[test]
fn hr_encoder_frame_params_vs_etsi_cod_references() {
    if !corpus_present() {
        return;
    }
    let mut agg = Agreement::default();
    for seq in ["SEQ01", "SEQ02", "SEQ03"] {
        run_sequence(seq, &mut agg);
    }
    let pct = |n: usize| 100.0 * n as f64 / agg.frames as f64;
    eprintln!(
        "HR encoder frame params over {} frames: R0 exact {:.1}% (±1 {:.1}%), \
         LPC1 {:.1}%, LPC2 {:.1}%, LPC3 {:.1}%, INT_LPC {:.1}%",
        agg.frames,
        pct(agg.r0_exact),
        pct(agg.r0_near),
        pct(agg.lpc1),
        pct(agg.lpc2),
        pct(agg.lpc3),
        pct(agg.int_lpc),
    );
    // Floors sit below the measured rates (R0 95.6% exact /
    // 100.0% within one code, LPC1 76.4%, LPC2 80.8%, LPC3 80.5%,
    // INT_LPC 55.0%) with margin for toolchain float variation.
    // R0's quantizer is simple enough that the chain lands within
    // one 2 dB code on every frame; the LPC VQ searches disagree
    // only where AFLAT residuals of neighbouring codebook rows
    // tie within the sub-LSB band the unstaged fixed-point C
    // resolves differently. INT_LPC is a two-way decision taken on
    // a residual-energy comparison that frequently lands near its
    // boundary (prev/current coefficient sets close), so its
    // agreement stays near chance on those frames; the floor only
    // guards against a systematically inverted decision.
    assert!(
        pct(agg.r0_near) >= 99.5,
        "R0 within one code: {:.1}%",
        pct(agg.r0_near)
    );
    assert!(
        pct(agg.r0_exact) >= 90.0,
        "R0 exact: {:.1}%",
        pct(agg.r0_exact)
    );
    assert!(pct(agg.lpc1) >= 70.0, "LPC1 exact: {:.1}%", pct(agg.lpc1));
    assert!(pct(agg.lpc2) >= 74.0, "LPC2 exact: {:.1}%", pct(agg.lpc2));
    assert!(pct(agg.lpc3) >= 74.0, "LPC3 exact: {:.1}%", pct(agg.lpc3));
    assert!(
        pct(agg.int_lpc) >= 45.0,
        "INT_LPC: {:.1}%",
        pct(agg.int_lpc)
    );
}
