//! ETSI EN 300 961 §6 digital test sequences — the official full-rate
//! conformance corpus, run bit-exactly.
//!
//! `tests/fixtures/etsi-fr/` stages the 13-bit uniform-PCM corpus from
//! the EN 300 961 V8.1.1 electronic attachment (see the fixtures
//! README for provenance + SHA-256 manifest): `Seq01..Seq05` and their
//! homing-prefixed `Seq01h..Seq05h` variants, the homing vectors
//! `Seq06h` / `Homing01`, and the §6.3.3.3 synchronization sequences
//! `Bitsync.inp` / `Seqsync.inp` / `Sync000.cod..Sync159.cod`.
//!
//! File format (§6.1 Table 6.1a/b/c, confirmed against the corpus):
//! 16-bit **little-endian** words; `*.inp`/`*.out` carry one
//! left-justified 13-bit PCM sample per word (3 LSBs zero), `*.cod`
//! carries one right-justified coded parameter per word, 76 words per
//! frame in §1.7 Table 1.1 order.
//!
//! The two §6.2 verification configurations:
//!
//! * **Configuration 1 (encoder under test)** — feed `SeqNN.inp`;
//!   every output frame must equal the corresponding `SeqNN.cod`
//!   frame.
//! * **Configuration 2 (decoder under test)** — feed `SeqNN.cod`;
//!   every output frame must equal the corresponding `SeqNN.out`
//!   frame.
//!
//! §6.2: a reset (RS) to the clause-5 initial state precedes each
//! sequence; the `*h` variants instead prepend two codec-homing
//! frames, for which *"the first output frame is undefined and need
//! not match"* — every subsequent frame must. Since this crate homes
//! by construction (`EncoderState::new()` / `DecoderState::new()` are
//! the §4.5/§4.6 home states), both the plain and the `*h` sequences
//! run from a fresh state here, and the `*h` comparison starts at
//! frame 1 per the spec's allowance.

#![cfg(not(miri))]

use oxideav_gsm::{
    cod_bytes_le_to_unpacked, inp_bytes_le_to_pcm, is_decoder_homing_frame, run_bit_sync_trial,
    DecoderState, EncoderState, FrameSyncTable, UnpackedFrame, COD_BYTES_PER_FRAME, FRAME_SAMPLES,
    PCM_BYTES_PER_FRAME,
};
use std::path::PathBuf;

fn fixture_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("fixtures")
        .join("etsi-fr")
}

/// The corpus lives in the git repository but is excluded from the
/// published package (`Cargo.toml` `exclude`). When the directory is
/// absent entirely (a package build), the conformance tests skip; when
/// it is present, any missing or malformed file is a hard failure.
fn corpus_present() -> bool {
    let present = fixture_dir().is_dir();
    if !present {
        eprintln!("etsi-fr corpus not present (published package?) — skipping");
    }
    present
}

fn read_pcm_frames(name: &str) -> Vec<[i16; FRAME_SAMPLES]> {
    let bytes =
        std::fs::read(fixture_dir().join(name)).unwrap_or_else(|e| panic!("fixture {name}: {e}"));
    assert_eq!(
        bytes.len() % PCM_BYTES_PER_FRAME,
        0,
        "{name}: partial frame"
    );
    bytes
        .chunks_exact(PCM_BYTES_PER_FRAME)
        .map(|c| inp_bytes_le_to_pcm(c).unwrap())
        .collect()
}

fn read_cod_frames(name: &str) -> Vec<UnpackedFrame> {
    let bytes =
        std::fs::read(fixture_dir().join(name)).unwrap_or_else(|e| panic!("fixture {name}: {e}"));
    assert_eq!(
        bytes.len() % COD_BYTES_PER_FRAME,
        0,
        "{name}: partial frame"
    );
    bytes
        .chunks_exact(COD_BYTES_PER_FRAME)
        .map(|c| cod_bytes_le_to_unpacked(c).unwrap())
        .collect()
}

/// §6.2 Configuration 1: encode `<seq>.inp`, require bit-exact
/// equality with `<seq>.cod` from `first_checked` on.
fn run_encoder_config(seq: &str, first_checked: usize) {
    if !corpus_present() {
        return;
    }
    let inp = read_pcm_frames(&format!("{seq}.inp"));
    let cod = read_cod_frames(&format!("{seq}.cod"));
    assert_eq!(inp.len(), cod.len(), "{seq}: frame count mismatch");
    let mut enc = EncoderState::new();
    for (n, (pcm, want)) in inp.iter().zip(cod.iter()).enumerate() {
        let got = enc.encode_frame_with_homing(pcm);
        if n >= first_checked {
            assert_eq!(
                got, *want,
                "{seq}: encoder output diverges from reference at frame {n}"
            );
        }
    }
}

/// §6.2 Configuration 2: decode `<seq>.cod`, require sample-exact
/// equality with `<seq>.out` from `first_checked` on.
fn run_decoder_config(seq: &str, first_checked: usize) {
    if !corpus_present() {
        return;
    }
    let cod = read_cod_frames(&format!("{seq}.cod"));
    let out = read_pcm_frames(&format!("{seq}.out"));
    assert_eq!(cod.len(), out.len(), "{seq}: frame count mismatch");
    let mut dec = DecoderState::new();
    for (n, (frame, want)) in cod.iter().zip(out.iter()).enumerate() {
        let got = dec.decode_frame_with_homing(frame);
        if n >= first_checked {
            assert_eq!(
                got, *want,
                "{seq}: decoder output diverges from reference at frame {n}"
            );
        }
    }
}

// ─── §6.3.1/§6.3.2 SEQ01..SEQ05, plain (reset-first) variants ───

#[test]
fn seq01_encoder_bit_exact() {
    run_encoder_config("Seq01", 0);
}

#[test]
fn seq02_encoder_bit_exact() {
    run_encoder_config("Seq02", 0);
}

#[test]
fn seq03_encoder_bit_exact() {
    run_encoder_config("Seq03", 0);
}

#[test]
fn seq04_encoder_bit_exact() {
    run_encoder_config("Seq04", 0);
}

#[test]
fn seq01_decoder_sample_exact() {
    run_decoder_config("Seq01", 0);
}

#[test]
fn seq02_decoder_sample_exact() {
    run_decoder_config("Seq02", 0);
}

#[test]
fn seq03_decoder_sample_exact() {
    run_decoder_config("Seq03", 0);
}

#[test]
fn seq04_decoder_sample_exact() {
    run_decoder_config("Seq04", 0);
}

/// §6.3.2: SEQ05 is decoder-only ("a `.cod` and `.out` but no
/// `.inp`") — it scans all code values for every parameter including
/// out-of-range LTP lags `Nr ∈ [0,127]`.
#[test]
fn seq05_decoder_sample_exact() {
    run_decoder_config("Seq05", 0);
}

// ─── §6.2 homing variants: two codec-homing frames prepended; the
// first output frame is undefined and need not match. ───

#[test]
fn seq01h_encoder_bit_exact() {
    run_encoder_config("Seq01h", 1);
}

#[test]
fn seq02h_encoder_bit_exact() {
    run_encoder_config("Seq02h", 1);
}

#[test]
fn seq03h_encoder_bit_exact() {
    run_encoder_config("Seq03h", 1);
}

#[test]
fn seq04h_encoder_bit_exact() {
    run_encoder_config("Seq04h", 1);
}

#[test]
fn seq01h_decoder_sample_exact() {
    run_decoder_config("Seq01h", 1);
}

#[test]
fn seq02h_decoder_sample_exact() {
    run_decoder_config("Seq02h", 1);
}

#[test]
fn seq03h_decoder_sample_exact() {
    run_decoder_config("Seq03h", 1);
}

#[test]
fn seq04h_decoder_sample_exact() {
    run_decoder_config("Seq04h", 1);
}

#[test]
fn seq05h_decoder_sample_exact() {
    run_decoder_config("Seq05h", 1);
}

// ─── §6.3.3 homing vectors ───

/// `Seq06h.inp` is exactly one §4.2 encoder-homing-frame and
/// `Seq06h.cod` exactly one §4.4 decoder-homing-frame — the corpus
/// files must equal the frames this crate reconstructs from the spec
/// tables.
#[test]
fn seq06h_matches_spec_reconstruction() {
    if !corpus_present() {
        return;
    }
    let inp = read_pcm_frames("Seq06h.inp");
    assert_eq!(inp.len(), 1);
    assert_eq!(inp[0], oxideav_gsm::encoder_homing_frame_pcm());

    let cod = read_cod_frames("Seq06h.cod");
    assert_eq!(cod.len(), 1);
    assert_eq!(cod[0], oxideav_gsm::decoder_homing_frame());
    assert!(is_decoder_homing_frame(&cod[0]));
}

/// §6.2 Configuration 1 on the homing vector: from the home state the
/// encoder maps `Seq06h.inp` to `Seq06h.cod` (§4.3 Step 1's
/// construction sentence, now pinned against the reference bytes).
#[test]
fn seq06h_encoder_bit_exact() {
    if !corpus_present() {
        return;
    }
    let inp = read_pcm_frames("Seq06h.inp");
    let cod = read_cod_frames("Seq06h.cod");
    let mut enc = EncoderState::new();
    assert_eq!(enc.encode_frame_with_homing(&inp[0]), cod[0]);
}

/// `Homing01.cod` / `Homing01.out` — the §6.3.3.2 partial-homing
/// mixture: after a complete decoder-homing frame, fractional homing
/// frames (LARs + first sub-frame only) must still reset the decoder.
#[test]
fn homing01_decoder_sample_exact() {
    run_decoder_config("Homing01", 0);
}

// ─── §6.3.3.3 / §6.3.3.4 synchronization sequences ───

/// `Seqsync.inp` holds *"3 encoder reset frames and the special
/// synchronization frame"* (§6.3.3.4). The reset frames are §4.2
/// encoder-homing-frames; the fourth frame is the special frame.
#[test]
fn seqsync_layout_matches_spec() {
    if !corpus_present() {
        return;
    }
    let frames = read_pcm_frames("Seqsync.inp");
    assert_eq!(frames.len(), 4);
    let homing = oxideav_gsm::encoder_homing_frame_pcm();
    for (n, f) in frames.iter().take(3).enumerate() {
        assert_eq!(*f, homing, "Seqsync frame {n} must be a reset frame");
    }
    assert_ne!(frames[3], homing, "the special frame is not a homing frame");
    for (k, &s) in frames[3].iter().enumerate() {
        assert_eq!(s & 0b111, 0, "special frame sample {k}: 3 LSBs must be 0");
    }
}

/// The heart of §6.3.3.3 frame synchronization: encoding the special
/// frame retarded by `r` samples from a homed encoder must reproduce
/// `SYNCrrr.COD` bit-exactly, the outputs are pairwise distinct, and
/// the table lookup recovers each `r`.
///
/// **Staged-corpus defect, r = 150..=159**: the staged
/// `Sync150.cod..Sync159.cod` are byte-for-byte identical to
/// `Sync050.cod..Sync059.cod` (verifiable directly from the files).
/// That contradicts §6.3.3.3's own statement — *"it was verified that
/// all 160 output frames were different"* — so those ten files cannot
/// be the true SYNC15x vectors (the duplication has the shape of a
/// digit slip in whichever packaging step produced them). This test
/// therefore pins bit-exactness for the 150 self-consistent files
/// `r ∈ 0..150` and pins the defect itself for the rest, so a
/// re-staging of corrected files will fail loudly here and the range
/// can be widened back to `0..160`.
#[test]
fn sync000_to_sync159_bit_exact() {
    if !corpus_present() {
        return;
    }
    let special = read_pcm_frames("Seqsync.inp")[3];
    let table = FrameSyncTable::build(&special);
    assert!(table.all_distinct(), "§6.3.3.3: all 160 outputs differ");
    for r in 0..150 {
        let want = read_cod_frames(&format!("Sync{r:03}.cod"));
        assert_eq!(want.len(), 1);
        assert_eq!(
            table.output(r).unwrap(),
            &want[0],
            "SYNC{r:03}.COD mismatch"
        );
        assert_eq!(table.match_output(&want[0]), Some(r));
    }
    // The defective tail: staged SYNC15x == staged SYNC05x. If this
    // ever fails, corrected files were staged — extend the loop above
    // to 0..160 and delete this block.
    for r in 150..160 {
        let staged = read_cod_frames(&format!("Sync{r:03}.cod"));
        let dup = read_cod_frames(&format!("Sync{:03}.cod", r - 100));
        assert_eq!(
            staged,
            dup,
            "staged Sync{r:03}.cod no longer duplicates Sync{:03}.cod — \
             corrected vectors appear to have been staged; widen this test to 0..160",
            r - 100
        );
    }
}

/// `Bitsync.inp` holds 13 frame triplets (§6.3.3.4), one per candidate
/// bit alignment. Exactly one triplet reconstructs genuine
/// encoder-homing-frames; running the §6.3.3.3 trial over each triplet
/// must detect the decoder-homing-frame for that one alone.
#[test]
fn bitsync_trials_detect_exactly_one_alignment() {
    if !corpus_present() {
        return;
    }
    let frames = read_pcm_frames("Bitsync.inp");
    assert_eq!(frames.len(), 13 * 3);
    let mut detected = Vec::new();
    for shift in 0..13 {
        let triplet = [
            frames[shift * 3],
            frames[shift * 3 + 1],
            frames[shift * 3 + 2],
        ];
        let trial = run_bit_sync_trial(shift, &triplet);
        if trial.decoder_homing_detected {
            detected.push(shift);
        }
    }
    assert_eq!(
        detected.len(),
        1,
        "exactly one bit alignment must home: {detected:?}"
    );
    // The detected triplet is the one made of genuine homing frames.
    let r = detected[0];
    let homing = oxideav_gsm::encoder_homing_frame_pcm();
    for k in 0..3 {
        assert_eq!(frames[r * 3 + k], homing);
    }
    // And the sweep front-end finds the same alignment.
    let found = oxideav_gsm::find_bit_sync(|shift| {
        [
            frames[shift * 3],
            frames[shift * 3 + 1],
            frames[shift * 3 + 2],
        ]
    });
    assert_eq!(found, Some(r));
}
