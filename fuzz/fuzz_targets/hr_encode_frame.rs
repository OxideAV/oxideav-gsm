#![no_main]

//! Arbitrary bytes reinterpreted as PCM through the GSM 06.20
//! half-rate encoder — the full clause 4.1 analysis chain (FLAT /
//! AFLAT, open- and closed-loop lag search, VSELP code searches,
//! {P0,GS} quantization, homing) must be total over any input
//! (saturated, DC, silence, homing frames), its annex-B packing
//! must be lossless on genuine encoder output, and the decoder must
//! account 160 samples per frame.

use libfuzzer_sys::fuzz_target;
use oxideav_gsm::hr::{HrDecoder, HrEncoder, HR_FRAME_SAMPLES};
use oxideav_gsm::HrParameters;

fuzz_target!(|data: &[u8]| {
    let mut enc = HrEncoder::new();
    let mut dec = HrDecoder::new();
    for chunk in data.chunks_exact(2 * HR_FRAME_SAMPLES).take(6) {
        let mut pcm = [0i16; HR_FRAME_SAMPLES];
        for (s, b) in pcm.iter_mut().zip(chunk.chunks_exact(2)) {
            *s = i16::from_le_bytes([b[0], b[1]]);
        }
        let p = enc.encode_frame(&pcm);
        let bytes = p.to_bits();
        let q = HrParameters::from_bits(&bytes).expect("14 bytes always parse");
        assert_eq!(p, q, "annex-B pack/parse must be lossless on encoder output");
        assert_eq!(q.to_cod_words(), p.to_cod_words());
        let out = dec.decode_frame(&q);
        assert_eq!(out.len(), HR_FRAME_SAMPLES);
        for s in out {
            assert_eq!(s % 8, 0, "output must stay 13-bit left-justified");
        }
    }
});
