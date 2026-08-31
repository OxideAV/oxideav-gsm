#![no_main]

//! Arbitrary annex-B half-rate frames through the GSM 06.20
//! decoder. Every 14-byte b1..b112 frame is syntactically valid
//! (annex A fields are fixed-width), so the decoder must be total
//! over the whole parameter space — including out-of-table delta
//! lags (the SEQ04 channel-error shape) and extreme GSP0/LPC codes.
//! Asserts the 13-bit left-justified output convention and the
//! annex-B pack/parse round trip on the way in.

use libfuzzer_sys::fuzz_target;
use oxideav_gsm::hr::HrDecoder;
use oxideav_gsm::HrParameters;

fuzz_target!(|data: &[u8]| {
    let mut dec = HrDecoder::new();
    for chunk in data.chunks_exact(14) {
        let p = HrParameters::from_bits(chunk).expect("14 bytes always parse");
        // Annex-B round trip: parse → pack → parse is stable.
        let packed = p.to_bits();
        let q = HrParameters::from_bits(&packed).unwrap();
        assert_eq!(p, q, "annex-B pack/parse must round-trip");
        let out = dec.decode_frame(&p);
        for s in out {
            assert_eq!(s % 8, 0, "output must stay 13-bit left-justified");
        }
    }
});
