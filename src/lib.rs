//! # oxideav-gsm
//!
//! Clean-room implementation of the **GSM 06.10 RPE-LTP** speech
//! codec — the original 13 kbit/s GSM Full Rate voice codec, 20 ms
//! frames of 160 samples at 8 kHz.
//!
//! This crate decodes the spec-defined 260-bit speech frame
//! (§1.7 Table 1.1 of ETSI EN 300 961, staged under
//! `docs/audio/gsm/`) into 160 linear 13-bit PCM samples per the
//! §5.3 fixed-point decoder, §5.3.5 de-emphasis, and §5.3.7
//! output format. Every value, every quantiser, every arithmetic
//! step comes from the staged ETSI deliverable.
//!
//! ## Public API
//!
//! Two API tiers per the workspace's dual-API convention:
//!
//! * **Registry path**: [`register`] / [`codec::register_codecs`]
//!   install a `Decoder` factory under the canonical codec id
//!   `"gsm"` in the workspace [`oxideav_core::CodecRegistry`].
//! * **Direct factory**: [`make_decoder`] returns a boxed
//!   `Decoder` for a given [`oxideav_core::CodecParameters`].
//!
//! Beneath those, the lower-level building blocks are also public
//! for callers who want to drive the decode pipeline without going
//! through a `Packet`/`Frame` adapter:
//!
//! * [`UnpackedFrame`] +
//!   [`UnpackedFrame::from_bit_stream_msb_first`] — parse the
//!   260-bit speech frame's 76 parameters.
//! * [`DecoderState`] + [`DecoderState::decode_frame`] — run the
//!   §5.3 fixed-point pipeline on one parsed frame.
//!
//! ## Encoder
//!
//! Not yet implemented (the §3.1 + §5.2 encoder is a separate
//! body of work and arrives in a later round). Calling
//! [`make_encoder`] returns an `Unsupported` error.
//!
//! ## Carriage format
//!
//! [`UnpackedFrame::from_bit_stream_msb_first`] accepts a 33-byte
//! buffer holding the spec's `b1..b260` stream packed MSB-first.
//! That layout matches the `b`-numbered bit positions §1.7 Table
//! 1.1 spells out verbatim. The specific 33-byte container
//! variants used in the wild (the `.gsm` byte format, RTP payload
//! type 3, MS-GSM WAV `0x31` block) wrap these 260 bits with
//! different per-container framing that is **not** specified in
//! EN 300 961 itself — those wrappers will be addressed in a
//! follow-up round once trace docs for them are staged.

#![deny(unsafe_code)]
#![warn(missing_debug_implementations)]

pub mod arith;
pub mod bitstream;
pub mod codec;
pub mod decoder;
pub mod encoder;
pub mod error;
pub mod tables;

pub use bitstream::{SubFrame, UnpackedFrame, FRAME_BITS, FRAME_SAMPLES, PULSES, SUBFRAMES};
pub use codec::{make_decoder, CODEC_ID};
pub use decoder::{
    decoder_homing_frame, encoder_homing_frame_pcm, is_decoder_homing_frame, DecoderState,
};
pub use encoder::PreProcessor;
pub use error::Error;

use oxideav_core::{CodecParameters, Encoder, Result, RuntimeContext};

/// Direct-factory placeholder for the GSM 06.10 encoder. Returns an
/// unsupported-error for now — the §3.1 / §5.2 encoder lands in a
/// later round.
pub fn make_encoder(_params: &CodecParameters) -> Result<Box<dyn Encoder>> {
    Err(oxideav_core::Error::unsupported(
        "oxideav-gsm: encoder not yet implemented; decoder only in this round",
    ))
}

/// Install the GSM 06.10 RPE-LTP decoder into the runtime context.
pub fn register(ctx: &mut RuntimeContext) {
    codec::register_codecs(&mut ctx.codecs);
}

oxideav_core::register!("gsm", register);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn register_does_not_panic() {
        let mut ctx = RuntimeContext::new();
        register(&mut ctx);
    }

    #[test]
    fn make_encoder_returns_unsupported() {
        let p = CodecParameters::audio(oxideav_core::CodecId::new("gsm"));
        assert!(make_encoder(&p).is_err());
    }
}
