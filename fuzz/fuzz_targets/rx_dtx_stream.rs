#![no_main]

//! Fuzz-driven traffic-frame/BFI/TAF sequences through the GSM
//! 06.31 §6.1.2 RX DTX handler with the GSM 06.11 substitution-and-
//! muting legs. Each step consumes one control byte (BFI/TAF/reset
//! choice) plus a 33-byte in-band frame; asserts the table-1
//! classification is consistent with the flags and the output
//! accounting holds under any interleaving of losses, SIDs, homing
//! frames, and resets.

use libfuzzer_sys::fuzz_target;
use oxideav_gsm::{RxClassification, RxDtxHandler, UnpackedFrame};

fuzz_target!(|data: &[u8]| {
    let mut rx = RxDtxHandler::new(0xF00D);
    let mut it = data.chunks_exact(34);
    for step in it.by_ref() {
        let ctl = step[0];
        let bfi = ctl & 1 != 0;
        let taf = ctl & 2 != 0;
        if ctl & 4 != 0 && ctl & 8 != 0 {
            rx.reset();
            continue;
        }
        let frame = UnpackedFrame::from_bit_stream_msb_first(&step[1..]).unwrap();
        let (class, out) = rx.receive_traffic_frame(&frame, bfi, taf);
        assert_eq!(out.len(), 160);
        // Flag/classification consistency (GSM 06.31 §6.1 table 1).
        match class {
            RxClassification::GoodSpeechFrame | RxClassification::ValidSidFrame => assert!(!bfi),
            RxClassification::LostSpeechFrame
            | RxClassification::LostSidFrame
            | RxClassification::IgnoredUnusableFrame => assert!(bfi),
            RxClassification::InvalidSidFrame => {}
        }
        if class == RxClassification::LostSidFrame {
            assert!(taf, "lost SID requires an expected SID (TAF=1)");
        }
    }
});
