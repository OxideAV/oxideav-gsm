//! GSM 06.20 half-rate frame layer, validated structurally against
//! the GSM 06.07 (EN 300 968) test-sequence corpus.
//!
//! `tests/fixtures/etsi-hr/` stages the 13-bit uniform-PCM corpus
//! disks 1–2 (see the fixtures README): the encoder legs
//! (`disk1/SEQ01..03.INP/.COD`, plus `SEQ05.INP`) and the decoder
//! legs (`disk2/SEQ01..05.DEC/.OUT`).
//!
//! Observed word framing (confirmed against every staged file):
//!
//! * `*.INP`/`*.OUT` — 16-bit LE PCM words, 160 per 20 ms frame;
//! * `*.COD` — 20 words per frame: the 18 annex-A parameters in
//!   table A.1 order, then two flag words (both constant 1 across
//!   the non-DTX sequences — the transmit-side VAD/SP pair of the
//!   half-rate DTX system);
//! * `*.DEC` — 22 words per frame: the 18 parameters, then four
//!   receive-side flag words (three constant 0 in these error-free
//!   sequences — the error/DTX indications — and one toggling
//!   periodically — the SACCH time-alignment flag).
//!
//! With the half-rate *decoder* not yet implemented, this harness
//! pins the frame layer: every staged coded frame parses into
//! [`HrParameters`], every field fits its annex-A width (the
//! word→params→word round-trip is the proof), and the annex-B
//! `b1..b112` packing round-trips every real frame of the corpus.

#![cfg(not(miri))]

use oxideav_gsm::{HrParameters, SubframeParams, HR_PARAMS_PER_FRAME};
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

/// Split a word stream into (params, flags) frames of `stride` words
/// (18 parameters + `stride - 18` flag words).
fn frames(rel: &str, stride: usize) -> Vec<([u16; HR_PARAMS_PER_FRAME], Vec<u16>)> {
    let words = read_words(rel);
    assert_eq!(words.len() % stride, 0, "{rel}: partial frame");
    words
        .chunks_exact(stride)
        .map(|c| {
            let mut p = [0u16; HR_PARAMS_PER_FRAME];
            p.copy_from_slice(&c[..HR_PARAMS_PER_FRAME]);
            (p, c[HR_PARAMS_PER_FRAME..].to_vec())
        })
        .collect()
}

/// Validate one coded stream: parse, width-check (via round-trip),
/// annex-B pack/unpack round-trip. Returns the per-mode frame counts.
fn validate_stream(rel: &str, stride: usize, flag_max: u16) -> [usize; 4] {
    let mut mode_count = [0usize; 4];
    for (n, (params, flags)) in frames(rel, stride).into_iter().enumerate() {
        let p = HrParameters::from_cod_words(&params);
        // Every field must fit its annex-A width: if any staged word
        // exceeded it, the masked round-trip would differ.
        assert_eq!(
            p.to_cod_words(),
            params,
            "{rel}: frame {n} has an out-of-width parameter word"
        );
        // Annex B b1..b112 round-trip on real data.
        let bits = p.to_bits();
        let q = HrParameters::from_bits(&bits).unwrap();
        assert_eq!(p, q, "{rel}: frame {n} annex-B round-trip");
        // Flag words stay tiny (0/1 indications).
        for (k, f) in flags.iter().enumerate() {
            assert!(
                *f <= flag_max,
                "{rel}: frame {n} flag word {k} = {f} out of range"
            );
        }
        mode_count[(p.mode_code & 3) as usize] += 1;
    }
    mode_count
}

/// `disk1/*.COD` — encoder reference outputs: 20 words per frame,
/// counts matching the paired `*.INP`.
#[test]
fn cod_streams_parse_and_roundtrip() {
    if !corpus_present() {
        return;
    }
    let mut total_modes = [0usize; 4];
    for seq in ["SEQ01", "SEQ02", "SEQ03"] {
        let modes = validate_stream(&format!("disk1/{seq}.COD"), 20, 1);
        let inp = read_words(&format!("disk1/{seq}.INP"));
        let cod = read_words(&format!("disk1/{seq}.COD"));
        assert_eq!(
            inp.len() / 160,
            cod.len() / 20,
            "{seq}: INP/COD frame count"
        );
        for (t, m) in total_modes.iter_mut().zip(modes.iter()) {
            *t += m;
        }
    }
    // The corpus exercises both the unvoiced (table B.1) and voiced
    // (table B.2) layouts.
    assert!(total_modes[0] > 0, "no MODE=0 frames seen");
    assert!(
        total_modes[1] + total_modes[2] + total_modes[3] > 0,
        "no voiced frames seen"
    );
}

/// `disk2/*.DEC` — decoder input sequences: 22 words per frame,
/// counts matching the paired `*.OUT` where present.
#[test]
fn dec_streams_parse_and_roundtrip() {
    if !corpus_present() {
        return;
    }
    for seq in ["SEQ01", "SEQ02", "SEQ03", "SEQ04", "SEQ05"] {
        let _ = validate_stream(&format!("disk2/{seq}.DEC"), 22, 1);
        let out = fixture_dir().join(format!("disk2/{seq}.OUT"));
        if out.exists() {
            let out_words = read_words(&format!("disk2/{seq}.OUT"));
            let dec_words = read_words(&format!("disk2/{seq}.DEC"));
            assert_eq!(
                out_words.len() / 160,
                dec_words.len() / 22,
                "{seq}: OUT/DEC frame count"
            );
        }
    }
}

/// The three error/DTX indications of the `*.DEC` flag block are all
/// zero across the error-free SEQ01..SEQ05 sequences; the fourth
/// (the periodic time-alignment flag) toggles.
#[test]
fn dec_flag_block_shape() {
    if !corpus_present() {
        return;
    }
    let mut saw_toggle = false;
    for seq in ["SEQ01", "SEQ02", "SEQ03", "SEQ04"] {
        for (_, flags) in frames(&format!("disk2/{seq}.DEC"), 22) {
            assert_eq!(flags.len(), 4);
            assert_eq!(&flags[..3], &[0, 0, 0], "{seq}: error flags must be 0");
            if flags[3] == 1 {
                saw_toggle = true;
            }
        }
    }
    assert!(saw_toggle, "the alignment flag must raise periodically");
}

/// Every voiced frame's LAG_1 code and every parameter of both MODE
/// splits appears within its documented range across the corpus —
/// a distribution sanity check on the field decoding (a wrong bit
/// split would push values out of range or collapse them).
#[test]
fn parameter_ranges_across_corpus() {
    if !corpus_present() {
        return;
    }
    let mut max_code9 = 0u16;
    let mut max_code7 = 0u8;
    for seq in ["SEQ01", "SEQ02", "SEQ03"] {
        for (params, _) in frames(&format!("disk1/{seq}.COD"), 20) {
            let p = HrParameters::from_cod_words(&params);
            match p.sub {
                SubframeParams::Unvoiced { code1, code2, gsp0 } => {
                    for s in 0..4 {
                        max_code7 = max_code7.max(code1[s]).max(code2[s]);
                        assert!(gsp0[s] < 32);
                    }
                }
                SubframeParams::Voiced {
                    lag1,
                    lag_delta,
                    code,
                    gsp0,
                } => {
                    // 8-bit lag code indexes the 256-entry allowable
                    // lag table.
                    let _ = oxideav_gsm::hr::tables::LAG_TABLE[lag1 as usize];
                    for d in lag_delta {
                        assert!(d < 16);
                    }
                    for s in 0..4 {
                        max_code9 = max_code9.max(code[s]);
                        assert!(gsp0[s] < 32);
                    }
                }
            }
        }
    }
    assert!(max_code9 < 512);
    assert!(max_code7 < 128);
}
