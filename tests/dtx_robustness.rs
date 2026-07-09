//! Robustness sweeps over the GSM 06.12 §6 receive-side DTX dispatcher
//! and the §6.1 comfort-noise generator.
//!
//! [`DtxReceiver`] consumes an already-classified stream of
//! [`RxFrame`]s (speech / valid-SID / no-data) and drives a single
//! shared §5.3 decoder so the filter memory carries across every
//! speech↔comfort-noise transition. These tests hammer that state
//! machine with long pseudo-random frame streams (deterministic in-test
//! LCG, no external RNG) and pin the invariants the staged EN 300 963
//! §6 text fixes:
//!
//! * every dispatch returns 160 §5.3.7-shaped samples and never panics,
//!   whatever the frame kind or the (possibly out-of-range) SID
//!   parameters;
//! * the §6 two-state machine transitions exactly as specified —
//!   speech ends a comfort-noise period, a valid SID opens/continues
//!   one, no-data continues an open period but cannot open one;
//! * the §6.1 generator is deterministic for a fixed seed, and
//!   [`DtxReceiver::reset`] returns the receiver to a history-
//!   independent home state (the §4.4 codec-homing behaviour).

use oxideav_gsm::{
    DecoderState, DtxReceiver, DtxState, RxFrame, SidParameters, UnpackedFrame, FRAME_SAMPLES,
};

struct Lcg(u32);
impl Lcg {
    fn new(seed: u32) -> Self {
        Self(seed)
    }
    fn next_u32(&mut self) -> u32 {
        self.0 = self.0.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        self.0
    }
}

fn iters(full: usize) -> usize {
    if cfg!(miri) {
        (full / 64).max(4)
    } else {
        full
    }
}

fn assert_shaped(pcm: &[i16; FRAME_SAMPLES]) {
    for (k, &s) in pcm.iter().enumerate() {
        assert_eq!(s & 0b111, 0, "§5.3.7 shaping at sample {k}: {s:#06x}");
    }
}

/// A random §1.7 speech frame parsed from random bytes.
fn random_speech(rng: &mut Lcg) -> RxFrame {
    let mut bytes = [0u8; 33];
    for b in bytes.iter_mut() {
        *b = (rng.next_u32() >> 24) as u8;
    }
    RxFrame::speech_from_bytes(&bytes).unwrap()
}

/// A random *valid-shape* SID: LARc within the §1.7 field widths, block
/// amplitudes in the 6-bit `xmaxc` range.
fn random_sid(rng: &mut Lcg) -> SidParameters {
    let lar_max: [i16; 8] = [63, 63, 31, 31, 15, 15, 7, 7];
    let mut lar_cr = [0i16; 9];
    for (i, m) in lar_max.iter().enumerate() {
        lar_cr[i + 1] = (rng.next_u32() as i16).rem_euclid(m + 1);
    }
    let xmax_cr = core::array::from_fn(|_| ((rng.next_u32() >> 26) & 0x3F) as u8);
    SidParameters::new(lar_cr, xmax_cr)
}

/// Pick one of the three frame kinds pseudo-randomly.
fn random_frame(rng: &mut Lcg) -> RxFrame {
    match rng.next_u32() % 3 {
        0 => random_speech(rng),
        1 => RxFrame::Sid(random_sid(rng)),
        _ => RxFrame::NoData,
    }
}

// ───────────────── no-panic / shaping fuzz ─────────────────

/// A long random speech/SID/no-data stream never panics and every
/// emitted frame is §5.3.7-shaped.
#[test]
fn random_rx_stream_never_panics_and_stays_shaped() {
    let mut rng = Lcg::new(0x5EED_0D7C);
    let mut rx = DtxReceiver::new(0xABCD_1234);
    for _ in 0..iters(3000) {
        let frame = random_frame(&mut rng);
        let pcm = rx.receive(frame);
        assert_shaped(&pcm);
    }
}

/// Out-of-range SID parameters (arbitrary `i16` LARc, full-byte block
/// amplitudes) are absorbed by the saturating §5.1 arithmetic and the
/// field masking — comfort-noise synthesis stays shaped and panic-free.
#[test]
fn hostile_sid_parameters_are_safe() {
    let mut rng = Lcg::new(0xDEAD_BEEF);
    let mut rx = DtxReceiver::new(7);
    for _ in 0..iters(1500) {
        let lar_cr = core::array::from_fn(|i| if i == 0 { 0 } else { rng.next_u32() as i16 });
        let xmax_cr = core::array::from_fn(|_| (rng.next_u32() >> 24) as u8);
        let sid = SidParameters::new(lar_cr, xmax_cr);
        assert_shaped(&rx.receive(RxFrame::Sid(sid)));
        assert_shaped(&rx.receive(RxFrame::NoData));
    }
}

// ───────────────── §6 state-machine invariants ─────────────────

/// The §6 two-state machine transitions exactly as the spec text
/// dictates over an adversarial hand-built sequence.
#[test]
fn dtx_state_machine_transitions() {
    let mut rng = Lcg::new(0x1122_3344);
    let mut rx = DtxReceiver::new(0);

    // Initial state: Speech, no SID seen.
    assert_eq!(rx.state(), DtxState::Speech);
    assert!(!rx.has_sid());

    // No-data before any SID stays in Speech (out-of-06.12-scope path).
    rx.receive(RxFrame::NoData);
    assert_eq!(rx.state(), DtxState::Speech, "no-data cannot open a period");
    assert!(!rx.has_sid());

    // A speech frame stays Speech.
    rx.receive(random_speech(&mut rng));
    assert_eq!(rx.state(), DtxState::Speech);

    // First valid SID opens the comfort-noise period.
    rx.receive(RxFrame::Sid(random_sid(&mut rng)));
    assert_eq!(rx.state(), DtxState::ComfortNoise);
    assert!(rx.has_sid());

    // No-data now continues the open period.
    rx.receive(RxFrame::NoData);
    assert_eq!(rx.state(), DtxState::ComfortNoise, "no-data continues CN");

    // A further SID updates but keeps the period open.
    rx.receive(RxFrame::Sid(random_sid(&mut rng)));
    assert_eq!(rx.state(), DtxState::ComfortNoise);

    // Speech ends the comfort-noise period.
    rx.receive(random_speech(&mut rng));
    assert_eq!(rx.state(), DtxState::Speech, "speech ends the CN period");
    // A SID was still seen this stream, so the fallback is unchanged.
    assert!(rx.has_sid());

    // reset() drops everything back to the home state.
    rx.reset();
    assert_eq!(rx.state(), DtxState::Speech);
    assert!(!rx.has_sid());
}

// ───────────────── determinism / homing ─────────────────

/// Two receivers with the same seed, fed the identical classified
/// stream, produce sample-identical output — the §6.1 generator is
/// deterministic.
#[test]
fn dtx_is_deterministic_for_fixed_seed() {
    let build = || {
        let mut rng = Lcg::new(0x0BAD_F00D);
        let mut frames = Vec::new();
        for _ in 0..iters(400) {
            frames.push(random_frame(&mut rng));
        }
        frames
    };
    let mut a = DtxReceiver::new(0x2468);
    let mut b = DtxReceiver::new(0x2468);
    let fa = build();
    let fb = build();
    let out_a = a.receive_stream(fa);
    let out_b = b.receive_stream(fb);
    assert_eq!(out_a, out_b, "same seed + same stream must be identical");
}

/// After `reset()` the receiver reproduces a fresh receiver's output on
/// the same speech stream — the shared §5.3 decoder is homed and the
/// output is history-independent (the §4.4 codec-homing property, at the
/// DTX-dispatcher level).
#[test]
fn reset_yields_history_independent_output() {
    // A fixed speech stream (parsed from a deterministic byte pattern).
    let speech: Vec<RxFrame> = {
        let mut rng = Lcg::new(0x3141_5926);
        (0..12).map(|_| random_speech(&mut rng)).collect()
    };

    // Fresh receiver output.
    let mut fresh = DtxReceiver::new(99);
    let want: Vec<i16> = fresh.receive_stream(speech.clone());

    // A receiver soiled with a comfort-noise burst, then reset, must
    // reproduce `want` exactly on the same speech stream.
    let mut soiled = DtxReceiver::new(99);
    let mut noise_rng = Lcg::new(0x2718);
    for _ in 0..5 {
        soiled.receive(RxFrame::Sid(random_sid(&mut noise_rng)));
        soiled.receive(RxFrame::NoData);
    }
    soiled.reset();
    let got: Vec<i16> = soiled.receive_stream(speech);
    assert_eq!(got, want, "reset must give history-independent output");
}

/// Sanity: a speech-only stream through the DTX receiver equals the same
/// stream through a bare `DecoderState` — the dispatcher adds nothing to
/// pure-speech decoding.
#[test]
fn speech_only_matches_bare_decoder() {
    let mut rng = Lcg::new(0xC0DE_C0DE);
    let frames: Vec<UnpackedFrame> = (0..iters(200))
        .map(|_| {
            let mut bytes = [0u8; 33];
            for b in bytes.iter_mut() {
                *b = (rng.next_u32() >> 24) as u8;
            }
            UnpackedFrame::from_bit_stream_msb_first(&bytes).unwrap()
        })
        .collect();

    let mut rx = DtxReceiver::new(0);
    let mut bare = DecoderState::new();
    for f in &frames {
        let via_rx = rx.receive(RxFrame::Speech(Box::new(*f)));
        let via_bare = bare.decode_frame(f);
        assert_eq!(via_rx, via_bare, "DTX speech path must equal bare decode");
    }
}
