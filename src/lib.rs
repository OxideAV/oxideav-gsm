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
//! Partial — §5.2.0..§5.2.3 (pre-processing pipeline) and §5.2.4 /
//! §5.2.5 / §5.2.6 / §5.2.7 / §5.2.10 (autocorrelation → Schur →
//! LAR transform → LAR quantisation + coding → short-term analysis
//! filter) are available as the public [`PreProcessor`] struct and
//! the [`analysis`] sub-module respectively. The
//! [`analysis::Analyzer`] struct runs §5.2.7 → §5.2.8 → §5.2.9.1 →
//! §5.2.9.2 → §5.2.10 end-to-end on a pre-processed frame and
//! emits the `LARc[1..=8]` codewords plus the short-term residual
//! `d[0..=159]` that §5.2.11 (LTP analysis) consumes. The §5.2.11
//! LTP parameter calculation, §5.2.12 long-term analysis filter,
//! and §5.2.18 `dp[-120..=-1]` delay-line update are exposed as
//! [`LtpAnalyzer`] / [`LtpParameters`]: per sub-segment, the
//! analyser produces the `(Nc, bc)` codewords, the long-term
//! prediction estimate `dpp[0..=39]`, and the long-term residual
//! `e[0..=39]` that the §5.2.13 weighting filter consumes. The
//! §5.2.13 weighting filter is also in place as the stateless free
//! function [`analysis::weighting_filter`]: it convolves
//! `e[0..=39]` with the Table 5.4 11-tap impulse response and emits
//! the block-filtered signal `x[0..=39]` that §5.2.14 RPE grid
//! selection consumes. §5.2.14 RPE grid selection is exposed as
//! the stateless free function [`analysis::select_rpe_grid`]
//! returning [`analysis::RpeGrid`]: it picks the sub-sampling grid
//! offset `Mc ∈ {0, 1, 2, 3}` that maximises the down-sampled
//! energy of `x[]` and emits the 13-pulse sequence
//! `xM[0..=12] = x[Mc + 3*i]` that §5.2.15 APCM quantisation
//! consumes. §5.2.15 APCM forward quantisation of the RPE sequence
//! is exposed as the stateless free function
//! [`analysis::apcm_quantise_rpe`] returning the
//! [`ApcmQuantised`] struct: it picks the 6-bit block-maximum
//! codeword `xmaxc` and the 13 3-bit codewords `xMc[0..=12]` the
//! §1.7 frame packer emits, plus the post-normalisation
//! `(exp, mant)` pair §5.2.16 inverse APCM quantisation consumes.
//! §5.2.16 encoder-side APCM inverse + §5.2.17 RPE grid positioning
//! are exposed as the stateless free function
//! [`analysis::apcm_inverse_and_position`]: it dequantises
//! `xMc[0..=12]` back to `xMp[0..=12]` via Table 5.6 `FAC[mant]`
//! (bit-identical to the decoder's §5.3.1 path) and scatters them
//! into the reconstructed long-term residual `ep[Mc + 3*i]`.
//! [`analysis::LtpAnalyzer::reconstruct_and_update`] chains
//! §5.2.16 → §5.2.17 → §5.2.18 to close the per-sub-segment LTP
//! delay-line feedback loop. Only the §1.7 frame packer remains
//! before [`make_encoder`] can land; it still returns an
//! `Unsupported` error while that stage lands.
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
pub use encoder::analysis::{ApcmQuantised, LtpAnalyzer, LtpParameters, RpeGrid};
pub use encoder::{analysis, PreProcessor};
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
