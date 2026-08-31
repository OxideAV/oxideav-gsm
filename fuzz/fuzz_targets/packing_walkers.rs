#![no_main]

//! The three full-rate packing parsers as hostile-input walkers:
//! parse arbitrary bytes and, where a frame parses, assert the
//! re-pack round trip is byte-identical — the parsers are lossless
//! on their accepted domain (spare/marker bits included). The
//! GSM 06.07-style conformance word layout is walked the same way.

use libfuzzer_sys::fuzz_target;
use oxideav_gsm::{cod_words_to_unpacked, unpacked_to_cod_words, UnpackedFrame};

fuzz_target!(|data: &[u8]| {
    // In-band 33-byte frames: always parse; repack must reproduce
    // the payload bits (the 4 trailing spare bits are zeroed, so
    // compare through a second parse).
    for chunk in data.chunks_exact(33) {
        let f = UnpackedFrame::from_bit_stream_msb_first(chunk).unwrap();
        let repacked = f.to_bit_stream_msb_first();
        let g = UnpackedFrame::from_bit_stream_msb_first(&repacked).unwrap();
        assert_eq!(f, g, "in-band repack round trip");

        // .gsm byte-frame: gated by the 0xD marker.
        if let Ok(f) = UnpackedFrame::from_gsm_byte_frame(chunk) {
            let repacked = f.to_gsm_byte_frame();
            let g = UnpackedFrame::from_gsm_byte_frame(&repacked).unwrap();
            assert_eq!(f, g, ".gsm repack round trip");
        }

        // Conformance word layout: parameters survive the word trip.
        let words = unpacked_to_cod_words(&f);
        let h = cod_words_to_unpacked(&words);
        assert_eq!(f, h, "conformance-word round trip");
    }
    for chunk in data.chunks_exact(65) {
        if let Ok((a, b)) = UnpackedFrame::pair_from_msgsm_block(chunk) {
            let repacked = UnpackedFrame::pair_to_msgsm_block(&a, &b);
            let (a2, b2) = UnpackedFrame::pair_from_msgsm_block(&repacked).unwrap();
            assert_eq!((a, b), (a2, b2), "MS-GSM repack round trip");
        }
    }
});
