#![no_main]

//! Arbitrary bytes reinterpreted as PCM through the GSM 06.10
//! encoder, packed, re-parsed and decoded — the §5.2 analysis
//! chain, §1.7 packer and §5.3 decoder must be closed under any
//! input (saturated samples included), with the packer/parser
//! byte-exact on real encoder output.

use libfuzzer_sys::fuzz_target;
use oxideav_gsm::{DecoderState, EncoderState, UnpackedFrame};

fuzz_target!(|data: &[u8]| {
    let mut enc = EncoderState::new();
    let mut dec = DecoderState::new();
    for chunk in data.chunks_exact(320).take(8) {
        let mut sop = [0i16; 160];
        for (s, b) in sop.iter_mut().zip(chunk.chunks_exact(2)) {
            *s = i16::from_le_bytes([b[0], b[1]]);
        }
        let coded = enc.encode_frame_with_homing(&sop);
        // §1.7 packer closure on genuine encoder output.
        let bytes = coded.to_bit_stream_msb_first();
        let reparsed = UnpackedFrame::from_bit_stream_msb_first(&bytes).unwrap();
        assert_eq!(coded, reparsed, "packer/parser must be byte-exact");
        let out = dec.decode_frame_with_homing(&reparsed);
        assert_eq!(out.len(), 160);
    }
});
