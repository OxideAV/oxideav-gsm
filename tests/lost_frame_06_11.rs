//! GSM 06.11 (EN 300 962) — substitution and muting of lost frames
//! for full-rate speech channels, as wired into the GSM 06.31
//! §6.1.2 RX DTX handler (`RxDtxHandler`).
//!
//! The staged EN 300 962 requirements (clause 5) + example solution
//! (clause 6):
//!
//! * §5.1 first lost speech frame → repetition of the previous good
//!   speech frame at the decoder input;
//! * §5.2 subsequent lost speech frames → gradual muting, silencing
//!   the output after at most 320 ms (16 frames), via the clause 6
//!   `Xmaxcr − 4` per-frame decrement and random grid positions,
//!   then the table-1 silence frame;
//! * §5.3 first lost SID frame → substituted by the last valid SID
//!   frame, valid-SID procedure applied;
//! * §5.4 second lost SID frame → the same muting on the comfort
//!   noise; further lost SID frames maintain the muting.

use oxideav_gsm::{
    make_sid_frame, silence_frame, RxClassification, RxDtxHandler, SidParameters,
    MUTING_XMAXC_DECREMENT,
};

const SUBFRAMES: usize = 4;

/// A voiced, energetic speech frame (massively deviating SID field,
/// so it classifies as speech).
fn loud_speech_frame() -> oxideav_gsm::UnpackedFrame {
    let mut f = oxideav_gsm::UnpackedFrame {
        lar_c: [0, 30, 20, 12, 8, 6, 4, 3, 2],
        ..Default::default()
    };
    for (j, sub) in f.sub.iter_mut().enumerate() {
        sub.n_c = 60 + j as u8;
        sub.b_c = 2;
        sub.m_c = 1;
        sub.xmax_c = 40;
        sub.x_mc = [5; 13];
    }
    f
}

fn frame_energy(pcm: &[i16]) -> f64 {
    pcm.iter().map(|&s| (s as f64) * (s as f64)).sum()
}

/// GSM 06.11 clause 6 table 1: the silence frame's pinned encoded
/// parameters.
#[test]
fn silence_frame_matches_table_1() {
    let f = silence_frame();
    assert_eq!(f.lar_c, [0, 42, 39, 21, 10, 9, 4, 3, 2]);
    for sub in &f.sub {
        assert_eq!(sub.b_c, 0, "LTP gain = 0");
        assert_eq!(sub.n_c, 40, "LTP lag = 40");
        assert_eq!(sub.m_c, 1, "grid position = 1");
        assert_eq!(sub.xmax_c, 0, "block amplitude = 0");
        assert_eq!(sub.x_mc, [3, 4, 3, 4, 4, 3, 3, 3, 3, 4, 4, 3, 3]);
    }
}

/// §5.1 + clause 6: the first lost speech frame is replaced at the
/// speech-decoder input by the previous good speech frame — the
/// output equals a twin receiver that was fed the good frame again.
#[test]
fn first_lost_speech_frame_repeats_previous_good_frame() {
    let speech = loud_speech_frame();

    let mut lossy = RxDtxHandler::new(11);
    let _ = lossy.receive_traffic_frame(&speech, false, false);
    let (class, lost_out) = lossy.receive_traffic_frame(&speech, true, false);
    assert_eq!(class, RxClassification::LostSpeechFrame);

    let mut clean = RxDtxHandler::new(11);
    let _ = clean.receive_traffic_frame(&speech, false, false);
    let (_, good_out) = clean.receive_traffic_frame(&speech, false, false);

    assert_eq!(
        lost_out, good_out,
        "first substitution must be a verbatim repetition"
    );
}

/// §5.2 + clause 6: subsequent lost speech frames mute the output —
/// silenced after at most 16 frames (320 ms), with the level trend
/// downward throughout, ending at the silence-frame steady state.
#[test]
fn subsequent_lost_speech_frames_mute_within_320ms() {
    let speech = loud_speech_frame();
    let mut rx = RxDtxHandler::new(3);
    let (_, good) = rx.receive_traffic_frame(&speech, false, false);
    let good_e = frame_energy(&good);
    assert!(good_e > 0.0);

    let mut energies = Vec::new();
    for _ in 0..20 {
        let (class, out) = rx.receive_traffic_frame(&speech, true, false);
        assert_eq!(class, RxClassification::LostSpeechFrame);
        energies.push(frame_energy(&out));
    }
    // xmax_c = 40 → 10 decrements to reach 0; well within the 16
    // frame (320 ms) bound. Compare against a silence-frame steady
    // state on a twin decoder chain.
    let mut silent_rx = RxDtxHandler::new(3);
    let _ = silent_rx.receive_traffic_frame(&speech, false, false);
    let mut floor_e = 0.0;
    for _ in 0..20 {
        let (_, out) = silent_rx.receive_traffic_frame(&silence_frame(), false, false);
        floor_e = frame_energy(&out);
    }
    let muted_e = energies[15];
    assert!(
        muted_e <= floor_e * 4.0 + 1e3,
        "output must be silenced after 320 ms: frame-16 energy {muted_e}, silence floor {floor_e}"
    );
    // Deep decay well before the bound (the first frames can sit at
    // a saturation plateau while the LTP feedback drains).
    assert!(energies[12] < good_e / 100.0);
    // Maintained thereafter.
    assert!(energies[19] <= energies[15] * 1.5 + 1e3);
}

/// §5.1: lost speech frames are not delivered to the decoder — the
/// substitution ignores the received (corrupt) frame content.
#[test]
fn lost_speech_frame_content_is_ignored() {
    let speech = loud_speech_frame();
    let mut corrupt = speech;
    corrupt.lar_c = [0, 1, 2, 3, 4, 5, 6, 7, 0];
    for sub in corrupt.sub.iter_mut() {
        sub.xmax_c = 63;
        sub.x_mc = [7; 13];
    }

    let mut a = RxDtxHandler::new(7);
    let _ = a.receive_traffic_frame(&speech, false, false);
    let (_, out_a) = a.receive_traffic_frame(&corrupt, true, false);

    let mut b = RxDtxHandler::new(7);
    let _ = b.receive_traffic_frame(&speech, false, false);
    let (_, out_b) = b.receive_traffic_frame(&speech, true, false);

    assert_eq!(out_a, out_b, "BFI frame payload must not matter");
}

/// §5.3: a single lost SID frame behaves exactly like re-receiving
/// the last valid SID frame.
#[test]
fn first_lost_sid_substitutes_last_valid_sid() {
    let sid = SidParameters::new([0, 42, 39, 21, 10, 9, 4, 3, 2], [20; SUBFRAMES]);
    let sid_frame = make_sid_frame(&sid);
    let junk = loud_speech_frame(); // BFI payload, content ignored

    let mut lossy = RxDtxHandler::new(5);
    let _ = lossy.receive_traffic_frame(&sid_frame, false, false);
    let (class, lost_out) = lossy.receive_traffic_frame(&junk, true, true);
    assert_eq!(class, RxClassification::LostSidFrame);

    let mut clean = RxDtxHandler::new(5);
    let _ = clean.receive_traffic_frame(&sid_frame, false, false);
    let (_, good_out) = clean.receive_traffic_frame(&sid_frame, false, false);

    assert_eq!(
        lost_out, good_out,
        "§5.3 substitution must equal a re-received SID"
    );
}

/// §5.4: from the second lost SID frame the comfort noise mutes
/// gradually (silenced within 320 ms) and the muting is maintained.
#[test]
fn subsequent_lost_sid_frames_mute_comfort_noise() {
    let sid = SidParameters::new([0, 42, 39, 21, 10, 9, 4, 3, 2], [48; SUBFRAMES]);
    let sid_frame = make_sid_frame(&sid);
    let junk = loud_speech_frame();

    let mut rx = RxDtxHandler::new(9);
    let (_, noise) = rx.receive_traffic_frame(&sid_frame, false, false);
    let noise_e = frame_energy(&noise);
    assert!(noise_e > 0.0);

    // Every frame carries TAF=1 so each is a lost SID frame
    // (48 / 4 = 12 decrements < 16 frames = 320 ms).
    let mut energies = Vec::new();
    for _ in 0..24 {
        let (class, out) = rx.receive_traffic_frame(&junk, true, true);
        assert_eq!(class, RxClassification::LostSidFrame);
        energies.push(frame_energy(&out));
    }
    assert!(
        energies[17] < noise_e / 16.0,
        "comfort noise must be essentially silenced within 320 ms of muting onset: {} vs {}",
        energies[17],
        noise_e
    );
    // Maintained for subsequent lost SID frames.
    assert!(energies[23] <= energies[17] * 1.5 + 1e3);
}

/// §6.1.2 interplay: unusable frames with no SID expected (TAF=0)
/// stay ignored — they neither advance the §5.4 muting nor end it.
#[test]
fn ignored_unusable_frames_do_not_advance_sid_muting() {
    let sid = SidParameters::new([0, 42, 39, 21, 10, 9, 4, 3, 2], [40; SUBFRAMES]);
    let sid_frame = make_sid_frame(&sid);
    let junk = loud_speech_frame();

    let mut rx = RxDtxHandler::new(13);
    let _ = rx.receive_traffic_frame(&sid_frame, false, false);
    // Two lost SIDs start the muting…
    let _ = rx.receive_traffic_frame(&junk, true, true);
    let _ = rx.receive_traffic_frame(&junk, true, true);
    // …then a run of ignored unusable frames (TAF=0).
    for _ in 0..3 {
        let (class, _) = rx.receive_traffic_frame(&junk, true, false);
        assert_eq!(class, RxClassification::IgnoredUnusableFrame);
    }
    // A further lost SID continues from where the muting stood: the
    // handler is still in its loss burst, not reset.
    let (class, _) = rx.receive_traffic_frame(&junk, true, true);
    assert_eq!(class, RxClassification::LostSidFrame);
}

/// A good frame ends any loss burst: after recovery, a fresh loss
/// burst starts again from the §5.1 repetition (not from a stale
/// muting state).
#[test]
fn good_frame_resets_loss_burst() {
    let speech = loud_speech_frame();
    let mut rx = RxDtxHandler::new(21);
    let _ = rx.receive_traffic_frame(&speech, false, false);
    // Deep loss burst → muted.
    for _ in 0..18 {
        let _ = rx.receive_traffic_frame(&speech, true, false);
    }
    // Recovery.
    let (class, good) = rx.receive_traffic_frame(&speech, false, false);
    assert_eq!(class, RxClassification::GoodSpeechFrame);
    assert!(frame_energy(&good) > 0.0);
    // New burst: first substitution repeats the good frame again —
    // its energy is far above the muted floor.
    let (_, first_lost) = rx.receive_traffic_frame(&speech, true, false);
    assert!(
        frame_energy(&first_lost) > frame_energy(&good) / 8.0,
        "fresh burst must restart from repetition, not stay muted"
    );
}

/// The muting decrement constant is the clause 6 value.
#[test]
fn muting_decrement_is_4() {
    assert_eq!(MUTING_XMAXC_DECREMENT, 4);
}
