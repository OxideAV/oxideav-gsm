//! §6.1 / §6.2 conformance harness over the word-oriented `*.INP` /
//! `*.COD` / `*.OUT` test-sequence formats (the `confio` module).
//!
//! ETSI EN 300 961 §6.2 defines two test configurations:
//!
//! * **§6.2.1 Configuration 1** — encoder under test: feed a `*.INP`
//!   PCM sequence, compare the encoder's coded output against the
//!   reference `*.COD`.
//! * **§6.2.2 Configuration 2** — decoder under test: feed a `*.COD`
//!   coded sequence, compare the decoder's `srop[..]` output against
//!   the reference `*.OUT`.
//!
//! The ETSI reference `*.INP`/`*.COD`/`*.OUT` *binary* files are not
//! staged under `docs/audio/gsm/`. This harness instead exercises the
//! §6.1 Table 6.1 *format machinery* end-to-end against frames the
//! crate builds itself: multi-frame `*.INP`/`*.COD`/`*.OUT` byte
//! streams that round-trip through `confio`, plus the §6.3.2 SEQ05
//! property "scan all possible codes for each parameter" driven through
//! the word-oriented decoder input. When the reference corpus is staged,
//! the same converters consume it directly.

use oxideav_gsm::{
    cod_bytes_le_to_unpacked, encoder_homing_frame_pcm, inp_bytes_le_to_pcm, pcm_to_inp_bytes_le,
    unpacked_to_cod_bytes_le, DecoderState, EncoderState, UnpackedFrame, COD_BYTES_PER_FRAME,
    FRAME_SAMPLES, PCM_BYTES_PER_FRAME, PULSES, SUBFRAMES,
};

/// §6.2.1 Configuration 1 over the §6.1 word formats: an `*.INP` PCM
/// stream → encoder → `*.COD` word stream, with every coded frame
/// recoverable by the §6.1 reader. Drives the encoder-homing-frame
/// (whose `*.COD` is the decoder-homing-frame) so the byte form is
/// independently checkable.
#[test]
fn config1_inp_stream_encodes_to_cod_stream() {
    // Build a 3-frame *.INP: two encoder-homing-frames then a ramp.
    let h = encoder_homing_frame_pcm();
    let ramp: [i16; FRAME_SAMPLES] = core::array::from_fn(|i| (((i as i16) - 80) << 3) & 0x7ff8);
    let frames_in = [h, h, ramp];

    let mut inp_stream = Vec::new();
    for f in &frames_in {
        inp_stream.extend_from_slice(&pcm_to_inp_bytes_le(f));
    }
    assert_eq!(inp_stream.len(), frames_in.len() * PCM_BYTES_PER_FRAME);

    // Encode frame-by-frame, emit the *.COD word stream.
    let mut enc = EncoderState::new();
    let mut cod_stream = Vec::new();
    for chunk in inp_stream.chunks_exact(PCM_BYTES_PER_FRAME) {
        let pcm = inp_bytes_le_to_pcm(chunk).unwrap();
        let frame = enc.encode_frame_with_homing(&pcm);
        cod_stream.extend_from_slice(&unpacked_to_cod_bytes_le(&frame));
    }
    assert_eq!(cod_stream.len(), frames_in.len() * COD_BYTES_PER_FRAME);

    // Every *.COD frame must parse back to the same parameters the
    // encoder produced (the §6.1 reader is the inverse of the writer).
    let mut enc2 = EncoderState::new();
    for (i, chunk) in cod_stream.chunks_exact(COD_BYTES_PER_FRAME).enumerate() {
        let reparsed = cod_bytes_le_to_unpacked(chunk).unwrap();
        let direct = enc2.encode_frame_with_homing(&frames_in[i]);
        assert_eq!(
            reparsed, direct,
            "*.COD frame {i} must match encoder output"
        );
    }
}

/// §6.2.2 Configuration 2 over the §6.1 word formats: a `*.COD` word
/// stream → decoder → `*.OUT` PCM stream, every output frame 13-bit
/// shaped (§5.3.7 / Table 6.1c three low bits zero).
#[test]
fn config2_cod_stream_decodes_to_out_stream() {
    // Build a 4-frame *.COD: distinct speech frames.
    let mut cod_stream = Vec::new();
    for k in 0..4u8 {
        let mut f = UnpackedFrame::default();
        f.lar_c[1] = (k as i16 * 5 + 1) & 0x3F;
        f.sub[0].xmax_c = 10 + k;
        f.sub[1].n_c = 55 + k;
        f.sub[2].m_c = k & 0x3;
        f.sub[3].x_mc = [(k % 8); PULSES];
        cod_stream.extend_from_slice(&unpacked_to_cod_bytes_le(&f));
    }

    let mut dec = DecoderState::new();
    let mut out_stream = Vec::new();
    for chunk in cod_stream.chunks_exact(COD_BYTES_PER_FRAME) {
        let frame = cod_bytes_le_to_unpacked(chunk).unwrap();
        let pcm = dec.decode_frame(&frame);
        out_stream.extend_from_slice(&pcm_to_inp_bytes_le(&pcm));
    }
    assert_eq!(out_stream.len(), 4 * PCM_BYTES_PER_FRAME);

    // §6.1 Table 6.1c — every *.OUT word's three low bits are zero.
    for chunk in out_stream.chunks_exact(2) {
        let w = i16::from_le_bytes([chunk[0], chunk[1]]);
        assert_eq!(w & 0b111, 0, "§6.1 Table 6.1c: *.OUT 3 LSBs zero");
    }
}

/// §6.2 loop-back over the word formats: encode the encoder-homing-frame
/// (Config 1), write its `*.COD`, read it back, decode (Config 2), and
/// recover the encoder-homing-frame `*.OUT` — the full circle through
/// the §6.1 converters at the byte level.
#[test]
fn loopback_through_confio_word_formats() {
    let mut enc = EncoderState::new();
    let mut dec = DecoderState::new();

    let inp = encoder_homing_frame_pcm();
    let inp_bytes = pcm_to_inp_bytes_le(&inp);

    let coded = enc.encode_frame(&inp_bytes_le_to_pcm(&inp_bytes).unwrap());
    let cod_bytes = unpacked_to_cod_bytes_le(&coded);
    assert_eq!(cod_bytes.len(), COD_BYTES_PER_FRAME);

    let reparsed = cod_bytes_le_to_unpacked(&cod_bytes).unwrap();
    // §4.4: feeding the decoder-homing-frame substitutes the
    // encoder-homing-frame.
    let out = dec.decode_frame_with_homing(&reparsed);
    let out_bytes = pcm_to_inp_bytes_le(&out);
    assert_eq!(
        out_bytes,
        pcm_to_inp_bytes_le(&encoder_homing_frame_pcm()),
        "loop-back recovers the encoder-homing-frame *.OUT"
    );
}

/// §6.3.2 SEQ05 — "scan all possible codes for each parameter."
/// SEQ05 is an artificial `*.COD` covering "the entire range of
/// codewords values" with `Nr ∈ [0,127]`. We scan each parameter
/// field across its full range (one parameter swept at a time, others
/// fixed) through the §6.1 `*.COD` reader into the §5.3 decoder, and
/// require: no panic, the right sample count, and §5.3.7 shaping.
#[test]
fn seq05_full_codeword_scan_through_cod_reader() {
    // xMc ∈ [0,7], Mc ∈ [0,3], bc ∈ [0,3], xmaxc ∈ [0,63],
    // Nc ∈ [0,127], LARc per-field widths. We sweep the widest few.
    let scan_one = |mutate: &dyn Fn(&mut UnpackedFrame, u8), max: u8| {
        let mut dec = DecoderState::new();
        for v in 0..=max {
            let mut f = UnpackedFrame::default();
            mutate(&mut f, v);
            let cod = unpacked_to_cod_bytes_le(&f);
            let parsed = cod_bytes_le_to_unpacked(&cod).unwrap();
            let out = dec.decode_frame(&parsed);
            assert_eq!(out.len(), FRAME_SAMPLES);
            for s in out {
                assert_eq!(s & 0b111, 0, "§5.3.7 shaping holds during scan");
            }
        }
    };

    // Nc full range [0,127] — the SEQ05 headline (non-allowed lags).
    for sf in 0..SUBFRAMES {
        scan_one(&|f, v| f.sub[sf].n_c = v, 127);
    }
    // xmaxc full range [0,63].
    scan_one(&|f, v| f.sub[0].xmax_c = v, 63);
    // bc / Mc full range [0,3].
    scan_one(&|f, v| f.sub[1].b_c = v & 0x3, 3);
    scan_one(&|f, v| f.sub[2].m_c = v & 0x3, 3);
    // LARc[1] full 6-bit range [0,63].
    scan_one(&|f, v| f.lar_c[1] = v as i16, 63);
    // xMc[0] full 3-bit range [0,7] across all sub-frames.
    scan_one(&|f, v| f.sub[3].x_mc[0] = v & 0x7, 7);
}
