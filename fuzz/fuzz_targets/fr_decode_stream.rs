#![no_main]

//! Arbitrary byte streams through the GSM 06.10 full-rate decoder
//! under all three frame packings, with the §4.4 homing protocol
//! live. Every 33-byte in-band frame is syntactically valid (fields
//! are fixed-width), so the in-band leg must be total; the .gsm
//! byte-frame leg additionally exercises the 0xD-marker reject path
//! and the MS-GSM leg the 65-byte two-frame split.

use libfuzzer_sys::fuzz_target;
use oxideav_gsm::{DecoderState, UnpackedFrame};

fuzz_target!(|data: &[u8]| {
    let Some((&sel, stream)) = data.split_first() else {
        return;
    };
    let mut dec = DecoderState::new();
    match sel % 3 {
        0 => {
            for chunk in stream.chunks_exact(33) {
                let f = UnpackedFrame::from_bit_stream_msb_first(chunk)
                    .expect("33-byte in-band frames are always parseable");
                let out = dec.decode_frame_with_homing(&f);
                assert_eq!(out.len(), 160);
            }
        }
        1 => {
            for chunk in stream.chunks_exact(33) {
                // The 0xD marker nibble gates parsing; a parsed
                // frame must decode totally.
                if let Ok(f) = UnpackedFrame::from_gsm_byte_frame(chunk) {
                    let out = dec.decode_frame_with_homing(&f);
                    assert_eq!(out.len(), 160);
                }
            }
        }
        _ => {
            for chunk in stream.chunks_exact(65) {
                if let Ok((a, b)) = UnpackedFrame::pair_from_msgsm_block(chunk) {
                    let _ = dec.decode_frame_with_homing(&a);
                    let _ = dec.decode_frame_with_homing(&b);
                }
            }
        }
    }
});
