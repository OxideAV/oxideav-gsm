//! Integration tests for the GSM 06.12 DTX / comfort-noise path,
//! driven entirely through the crate's *public* API.
//!
//! Source: ETSI EN 300 963 V8.0.1 (GSM 06.12), staged at
//! `docs/audio/gsm/etsi-en-300963-gsm-06.12-comfort-noise.pdf`.
//!
//! These exercise the full receive-side §6 dispatch
//! ([`oxideav_gsm::DtxReceiver`]) and its closure with the transmit-side
//! §5.1 background-noise evaluator ([`oxideav_gsm::NoiseEvaluator`]):
//! the §5.1 means → §5.2 SID parameters → §6.1 comfort-noise synthesis
//! loop, plus the §6 SPEECH ⇄ COMFORT-NOISE state machine across a
//! realistic DTX cycle.

use oxideav_gsm::{
    DecoderState, DtxReceiver, DtxState, NoiseEvaluator, NoiseFrameParameters, RxFrame, SubFrame,
    UnpackedFrame, FRAME_SAMPLES, NOISE_EVAL_FRAMES, SUBFRAMES,
};

/// A non-homing, non-trivial speech frame whose §5.3 decode is
/// observable (distinct LARs + varied sub-frames).
fn speech_frame(salt: u8) -> Box<UnpackedFrame> {
    let mut f = UnpackedFrame {
        lar_c: [0, 28 + salt as i16, 20, 18, 12, 10, 6, 4, 3],
        sub: [SubFrame::default(); SUBFRAMES],
    };
    for (j, sf) in f.sub.iter_mut().enumerate() {
        sf.n_c = 40 + j as u8 * 7;
        sf.b_c = 1;
        sf.m_c = (j as u8) % 4;
        sf.xmax_c = 18 + j as u8 + salt;
        for (i, p) in sf.x_mc.iter_mut().enumerate() {
            *p = ((i as u8).wrapping_add(salt)) % 8;
        }
    }
    Box::new(f)
}

/// Full DTX cycle through the public API: a speech burst, a SID frame
/// closing the burst, a stretch of no-data (radio cut) during which
/// comfort noise continues, a refreshing SID, more no-data, and finally
/// speech resuming. Every step must yield 160 PCM samples shaped per
/// §5.3.7, and the §6 state machine must follow speech ⇄ comfort-noise.
#[test]
fn full_dtx_cycle_states_and_output() {
    let mut rx = DtxReceiver::new(0xC0FFEE);

    // --- speech burst (state stays Speech) ---
    for k in 0..5u8 {
        let pcm = rx.receive(RxFrame::Speech(speech_frame(k)));
        assert_eq!(pcm.len(), FRAME_SAMPLES);
        assert_eq!(rx.state(), DtxState::Speech);
    }
    assert!(!rx.has_sid());

    // --- SID closes the burst, opens comfort noise ---
    let sid1 = build_sid(&[0, 9, 23, 15, 8, 7, 3, 3, 2], 1024);
    let _ = rx.receive(RxFrame::Sid(sid1));
    assert_eq!(rx.state(), DtxState::ComfortNoise);
    assert!(rx.has_sid());
    assert!(rx.is_generating_comfort_noise());

    // --- radio cut: no-data frames continue comfort noise ---
    let mut any_nonzero = false;
    for _ in 0..10 {
        let pcm = rx.receive(RxFrame::NoData);
        assert_eq!(pcm.len(), FRAME_SAMPLES);
        assert_eq!(rx.state(), DtxState::ComfortNoise);
        for s in pcm {
            assert_eq!(s & 0b111, 0, "§5.3.7 output low 3 bits must be zero");
        }
        if pcm.iter().any(|&s| s != 0) {
            any_nonzero = true;
        }
    }
    assert!(any_nonzero, "comfort noise produced digital silence");

    // --- refreshing SID updates parameters (still comfort noise) ---
    let sid2 = build_sid(&[0, 5, 10, 8, 6, 4, 3, 2, 1], 2048);
    let _ = rx.receive(RxFrame::Sid(sid2));
    assert_eq!(rx.state(), DtxState::ComfortNoise);
    for _ in 0..6 {
        let _ = rx.receive(RxFrame::NoData);
        assert_eq!(rx.state(), DtxState::ComfortNoise);
    }

    // --- speech resumes (back to Speech) ---
    let pcm = rx.receive(RxFrame::Speech(speech_frame(9)));
    assert_eq!(pcm.len(), FRAME_SAMPLES);
    assert_eq!(rx.state(), DtxState::Speech);
    assert!(rx.has_sid(), "has_sid stays set after a speech resume");
}

/// Build SID parameters the way the transmit side would: average N
/// identical VAD=0 frames through the §5.1 evaluator, so the SID
/// codewords are the §5.2-encoded means rather than hand-picked values.
fn build_sid(lar: &[i16; 9], xmax: i16) -> oxideav_gsm::SidParameters {
    let mut eval = NoiseEvaluator::new();
    for _ in 0..NOISE_EVAL_FRAMES {
        eval.push_frame(NoiseFrameParameters::new(*lar, [xmax; SUBFRAMES]));
    }
    eval.evaluate().expect("non-empty window")
}

/// Closes the §5.1 → §5.2 → §6.1 loop end-to-end through the public API:
/// the transmit-side evaluator's SID drives the receive-side generator,
/// and the synthesised comfort noise is non-silent and well-formed.
#[test]
fn transmit_evaluator_feeds_receiver() {
    // Accumulate four VAD=0 frames with rising block maxima.
    let mut eval = NoiseEvaluator::new();
    for k in 0..NOISE_EVAL_FRAMES as i16 {
        let lar = core::array::from_fn(|i| if i == 0 { 0 } else { (i as i16) * 150 + k * 20 });
        eval.push_frame(NoiseFrameParameters::new(lar, [(k + 1) * 800; SUBFRAMES]));
    }
    assert!(eval.is_ready());
    let sid = eval.evaluate().expect("non-empty window");

    let mut rx = DtxReceiver::new(7);
    let _ = rx.receive(RxFrame::Sid(sid));
    assert_eq!(rx.state(), DtxState::ComfortNoise);
    assert_eq!(rx.sid(), sid, "first SID is in effect immediately");

    let mut any_nonzero = false;
    for _ in 0..16 {
        let pcm = rx.receive(RxFrame::NoData);
        if pcm.iter().any(|&s| s != 0) {
            any_nonzero = true;
        }
    }
    assert!(any_nonzero, "transmit-derived comfort noise was silent");
}

/// The shared decoder makes comfort noise continuous across a speech →
/// noise boundary: the first comfort-noise frame after a speech burst
/// differs from the same §6.1 frame synthesised on a cold decoder. (We
/// reproduce the cold path with the same parameters but a fresh
/// DtxReceiver that has seen no speech.)
#[test]
fn comfort_noise_is_continuous_across_speech_boundary() {
    let sid = build_sid(&[0, 9, 23, 15, 8, 7, 3, 3, 2], 1500);

    // Warm: speech burst then SID, same seed.
    let mut warm = DtxReceiver::new(99);
    for k in 0..4u8 {
        let _ = warm.receive(RxFrame::Speech(speech_frame(k)));
    }
    let warm_cn = warm.receive(RxFrame::Sid(sid));

    // Cold: no speech, just the SID, same seed.
    let mut cold = DtxReceiver::new(99);
    let cold_cn = cold.receive(RxFrame::Sid(sid));

    assert_ne!(
        warm_cn, cold_cn,
        "comfort noise ignored the carried-over speech decoder state"
    );
}

/// A homing decoder reset via `DtxReceiver::reset` truly re-homes the
/// shared §5.3 decoder: a speech frame decoded right after reset matches
/// the same frame on a brand-new bare decoder.
#[test]
fn reset_rehomes_shared_decoder() {
    let mut rx = DtxReceiver::new(4);
    // Dirty the decoder with a speech burst + comfort noise.
    for k in 0..3u8 {
        let _ = rx.receive(RxFrame::Speech(speech_frame(k)));
    }
    let _ = rx.receive(RxFrame::Sid(build_sid(&[0, 1, 2, 3, 4, 5, 6, 7, 8], 900)));
    for _ in 0..3 {
        let _ = rx.receive(RxFrame::NoData);
    }

    rx.reset();
    assert_eq!(rx.state(), DtxState::Speech);
    assert!(!rx.has_sid());

    let probe = speech_frame(2);
    let after_reset = rx.receive(RxFrame::Speech(probe.clone()));

    let mut bare = DecoderState::new();
    let bare_pcm = bare.decode_frame(&probe);
    assert_eq!(
        after_reset, bare_pcm,
        "reset did not return the shared decoder to its home state"
    );
}
