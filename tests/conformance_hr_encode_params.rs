//! GSM 06.20 encoder (clauses 4.1.1–4.1.11) vs the staged GSM 06.07
//! encoder references (`tests/fixtures/etsi-hr/disk1/`).
//!
//! EN 300 969's printed equations are functional (the bit-exact
//! arithmetic is the unstaged GSM 06.06 ANSI-C), so exact code
//! equality across whole sequences is not the achievable bar:
//! quantizer decisions sitting on a boundary flip under sub-LSB
//! arithmetic differences, and every such flip feeds the long-term
//! filter state the following subframes search against. What this
//! harness pins is the measured per-parameter agreement rate
//! against `SEQ01..03.COD`, twice over:
//!
//! * **end to end** — the encoder alone, every decision its own;
//! * **stage-isolated** — each search run with the reference's
//!   upstream parameters substituted (frame parameters → lags →
//!   codes), so the open-loop/closed-loop lag search, the VSELP
//!   code searches and the `{P0,GS}` quantizer are each measured
//!   against the reference on the reference's own inputs.
//!
//! Every floor sits below the rate measured when the chain landed
//! (quoted in the assertions) with margin for toolchain float
//! variation; any structural regression (wrong table, recursion,
//! window, sign convention or scaling) collapses several at once.

#![cfg(not(miri))]

use oxideav_gsm::hr::{FrameAnalysis, HrEncoder, HrForce, HR_DECODER_HOMING_WORDS};
use std::collections::BTreeMap;
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

/// Which reference parameters to substitute upstream of the
/// measured stage.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Stage {
    /// Nothing forced: the encoder end to end.
    EndToEnd,
    /// Reference R0/LPC/INT_LPC: measures the lag search + MODE.
    FrameForced,
    /// + reference MODE/lags: measures the code searches.
    LagsForced,
    /// + reference codes: measures the {P0,GS} quantizer.
    CodesForced,
}

#[derive(Default)]
struct Rates(BTreeMap<&'static str, (usize, usize)>);

impl Rates {
    fn bump(&mut self, key: &'static str, ok: bool) {
        let e = self.0.entry(key).or_insert((0, 0));
        e.1 += 1;
        if ok {
            e.0 += 1;
        }
    }
    fn pct(&self, key: &str) -> f64 {
        let (ok, n) = self.0.get(key).copied().unwrap_or((0, 0));
        if n == 0 {
            0.0
        } else {
            100.0 * ok as f64 / n as f64
        }
    }
}

/// Absolute allowable-lag levels of a voiced parameter frame.
fn levels(w: &[u16]) -> [usize; 4] {
    let mut lv = [0usize; 4];
    let mut l = w[6] as i32;
    for (sf, slot) in lv.iter_mut().enumerate() {
        if sf > 0 {
            l += w[6 + 3 * sf] as i32 - 8;
        }
        *slot = l.clamp(0, 255) as usize;
    }
    lv
}

fn run(stage: Stage) -> Rates {
    let mut rates = Rates::default();
    for seq in ["SEQ01", "SEQ02", "SEQ03"] {
        let inp = read_words(&format!("disk1/{seq}.INP"));
        let cod = read_words(&format!("disk1/{seq}.COD"));
        let frames = inp.len() / 160;
        assert_eq!(cod.len() / 20, frames, "{seq}: INP/COD frame count");
        let mut enc = HrEncoder::new();
        for fi in 0..frames {
            let mut pcm = [0i16; 160];
            for (n, s) in pcm.iter_mut().enumerate() {
                *s = inp[fi * 160 + n] as i16;
            }
            let refw = &cod[fi * 20..fi * 20 + 18];
            let rmode = refw[5] as u8;
            let mut force = HrForce::default();
            if stage != Stage::EndToEnd {
                force.frame = Some(FrameAnalysis {
                    r0: refw[0] as u8,
                    lpc1: refw[1],
                    lpc2: refw[2],
                    lpc3: refw[3] as u8,
                    int_lpc: refw[4] != 0,
                });
            }
            if matches!(stage, Stage::LagsForced | Stage::CodesForced) {
                force.lags = Some((rmode, if rmode != 0 { levels(refw) } else { [0; 4] }));
            }
            if stage == Stage::CodesForced {
                if rmode != 0 {
                    force.codes = Some([refw[7], refw[10], refw[13], refw[16]]);
                } else {
                    force.codes = Some([refw[6], refw[9], refw[12], refw[15]]);
                    force.codes2 = Some([refw[7], refw[10], refw[13], refw[16]]);
                }
            }
            let homing = pcm.iter().all(|&s| s == 8);
            let p = if stage == Stage::EndToEnd || homing {
                enc.encode_frame(&pcm)
            } else {
                enc.encode_frame_forced(&pcm, &force)
            };
            let ours = p.to_cod_words();
            // The two leading encoder homing frames: the second one
            // finds the encoder in its home state, so its output is
            // the decoder homing frame — exactly as the reference.
            if fi == 1 {
                assert_eq!(
                    ours, HR_DECODER_HOMING_WORDS,
                    "{seq}: frame 1 is a homing frame"
                );
                assert_eq!(&refw[..18], &HR_DECODER_HOMING_WORDS[..]);
            }
            // Skip the homing frames and the first frame after them
            // (the reference encoder's state right after a reset is
            // that of the bit-exact C).
            if fi < 3 {
                continue;
            }
            rates.bump("r0", ours[0] == refw[0]);
            rates.bump("r0_near", (ours[0] as i32 - refw[0] as i32).abs() <= 1);
            rates.bump("lpc1", ours[1] == refw[1]);
            rates.bump("lpc2", ours[2] == refw[2]);
            rates.bump("lpc3", ours[3] == refw[3]);
            rates.bump("int_lpc", ours[4] == refw[4]);
            rates.bump("mode", ours[5] == refw[5]);
            if ours[5] != refw[5] {
                continue;
            }
            if rmode != 0 {
                let (lo, lr) = (levels(&ours), levels(refw));
                rates.bump("lag1", ours[6] == refw[6]);
                for sf in 0..4 {
                    rates.bump("lag_level", lo[sf] == lr[sf]);
                    rates.bump("lag_level_pm1", (lo[sf] as i32 - lr[sf] as i32).abs() <= 1);
                    rates.bump("code9", ours[7 + 3 * sf] == refw[7 + 3 * sf]);
                    rates.bump("gsp0_v", ours[8 + 3 * sf] == refw[8 + 3 * sf]);
                }
            } else {
                for sf in 0..4 {
                    rates.bump("code1", ours[6 + 3 * sf] == refw[6 + 3 * sf]);
                    rates.bump("code2", ours[7 + 3 * sf] == refw[7 + 3 * sf]);
                    rates.bump("gsp0_u", ours[8 + 3 * sf] == refw[8 + 3 * sf]);
                }
            }
        }
    }
    rates
}

fn report(name: &str, r: &Rates) {
    let line: Vec<String> =
        r.0.iter()
            .map(|(k, (ok, n))| format!("{k} {:.1}% ({ok}/{n})", 100.0 * *ok as f64 / *n as f64))
            .collect();
    eprintln!("HR encoder [{name}]: {}", line.join(", "));
}

/// End to end: every decision the encoder's own. Measured when the
/// excitation chain landed: R0 95,6% (±1 100%), LPC1 76,4%, LPC2
/// 80,8%, LPC3 80,5%, INT_LPC 91,5%, MODE 91,3%, LAG_1 46,3%, lag
/// levels 44,3% (±1 65,6%), CODE1 80,8%, CODE2 77,0%, voiced CODE
/// 20,3%, unvoiced GSP0 68,5%, voiced GSP0 36,4%.
#[test]
fn hr_encoder_end_to_end_vs_etsi_cod_references() {
    if !corpus_present() {
        return;
    }
    let r = run(Stage::EndToEnd);
    report("end to end", &r);
    let floors = [
        ("r0", 90.0),
        ("r0_near", 99.5),
        ("lpc1", 70.0),
        ("lpc2", 74.0),
        ("lpc3", 74.0),
        ("int_lpc", 85.0),
        ("mode", 85.0),
        ("lag1", 38.0),
        ("lag_level_pm1", 58.0),
        ("code1", 72.0),
        ("code2", 68.0),
        ("code9", 14.0),
        ("gsp0_u", 60.0),
        ("gsp0_v", 28.0),
    ];
    for (k, floor) in floors {
        assert!(
            r.pct(k) >= floor,
            "{k}: {:.1}% below the {floor}% floor",
            r.pct(k)
        );
    }
}

/// Lag search + MODE on the reference's frame parameters (measured
/// MODE 92,4%, lag levels ±1 70,0%).
#[test]
fn hr_lag_search_on_reference_frame_parameters() {
    if !corpus_present() {
        return;
    }
    let r = run(Stage::FrameForced);
    report("frame forced", &r);
    assert!(r.pct("mode") >= 86.0, "mode {:.1}%", r.pct("mode"));
    assert!(
        r.pct("lag_level_pm1") >= 62.0,
        "lag_level_pm1 {:.1}%",
        r.pct("lag_level_pm1")
    );
}

/// VSELP code searches on the reference's lags (measured CODE1
/// 91,4%, CODE2 88,4%, voiced CODE 41,1%).
#[test]
fn hr_code_search_on_reference_lags() {
    if !corpus_present() {
        return;
    }
    let r = run(Stage::LagsForced);
    report("lags forced", &r);
    assert!(r.pct("code1") >= 84.0, "code1 {:.1}%", r.pct("code1"));
    assert!(r.pct("code2") >= 80.0, "code2 {:.1}%", r.pct("code2"));
    assert!(r.pct("code9") >= 34.0, "code9 {:.1}%", r.pct("code9"));
}

/// {P0,GS} quantizer on the reference's codes (measured unvoiced
/// 82,2%, voiced 68,7%).
#[test]
fn hr_gain_quantizer_on_reference_codes() {
    if !corpus_present() {
        return;
    }
    let r = run(Stage::CodesForced);
    report("codes forced", &r);
    assert!(r.pct("gsp0_u") >= 74.0, "gsp0_u {:.1}%", r.pct("gsp0_u"));
    assert!(r.pct("gsp0_v") >= 60.0, "gsp0_v {:.1}%", r.pct("gsp0_v"));
}
