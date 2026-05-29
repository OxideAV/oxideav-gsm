//! Codec-registry adapter — wires the §5.3 decoder behind the
//! `oxideav_core::Decoder` trait so the GSM 06.10 RPE-LTP codec can
//! be reached via the workspace codec registry alongside the direct
//! `make_decoder` factory.

use std::collections::VecDeque;

use oxideav_core::{
    AudioFrame, CodecCapabilities, CodecId, CodecInfo, CodecParameters, CodecRegistry, Decoder,
    Error as CoreError, Frame, Packet, Result,
};

use crate::bitstream::{UnpackedFrame, FRAME_SAMPLES};
use crate::decoder::DecoderState;

/// Canonical codec id for GSM 06.10 RPE-LTP — the conventional
/// string token tooling uses for the GSM Full Rate codec.
pub const CODEC_ID: &str = "gsm";

/// Build a boxed [`Decoder`] for GSM 06.10 RPE-LTP with the given
/// codec parameters. Direct-factory entry point — the
/// [`register_codecs`] path installs this same function into the
/// codec registry, so callers who don't want a registry lookup may
/// invoke this directly with `params` they constructed manually.
pub fn make_decoder(params: &CodecParameters) -> Result<Box<dyn Decoder>> {
    let channels = params.channels.unwrap_or(1);
    if channels != 1 {
        return Err(CoreError::unsupported(
            "GSM 06.10 RPE-LTP decoder: only mono input is defined by §1.3",
        ));
    }
    Ok(Box::new(GsmDecoder {
        codec_id: params.codec_id.clone(),
        state: DecoderState::new(),
        pending: VecDeque::new(),
        eof: false,
    }))
}

/// Register the GSM codec id into the workspace registry.
pub fn register_codecs(reg: &mut CodecRegistry) {
    let mut caps = CodecCapabilities::audio("oxideav-gsm");
    caps.decode = true;
    caps.lossy = true;
    caps.max_sample_rate = Some(8000);
    caps.max_channels = Some(1);
    reg.register(
        CodecInfo::new(CodecId::new(CODEC_ID))
            .capabilities(caps)
            .decoder(make_decoder),
    );
}

/// Adapter between the `oxideav_core::Decoder` trait and the
/// §5.3 fixed-point decoder.
struct GsmDecoder {
    codec_id: CodecId,
    state: DecoderState,
    /// Packets waiting to be decoded. We buffer rather than
    /// requiring strict one-packet-at-a-time call shape so a
    /// container that delivers multiple GSM frames in one packet
    /// (e.g. one 33-byte frame per packet vs N frames per packet)
    /// works either way.
    pending: VecDeque<Packet>,
    eof: bool,
}

impl Decoder for GsmDecoder {
    fn codec_id(&self) -> &CodecId {
        &self.codec_id
    }

    fn send_packet(&mut self, packet: &Packet) -> Result<()> {
        self.pending.push_back(packet.clone());
        Ok(())
    }

    fn receive_frame(&mut self) -> Result<Frame> {
        let Some(pkt) = self.pending.pop_front() else {
            return if self.eof {
                Err(CoreError::Eof)
            } else {
                Err(CoreError::NeedMore)
            };
        };
        if pkt.data.len() < 33 {
            return Err(CoreError::invalid(format!(
                "oxideav-gsm: packet shorter than the 33-byte minimum (got {})",
                pkt.data.len()
            )));
        }

        // §1.5 — 20 ms frame ⇒ 160 PCM samples per 260-bit frame.
        // Concatenate whole frames found in this packet so a
        // muxer that delivers multiple frames per packet still
        // works.
        let n_frames = pkt.data.len() / 33;
        let mut pcm = Vec::with_capacity(FRAME_SAMPLES * n_frames * 2);
        for f in 0..n_frames {
            let bytes = &pkt.data[f * 33..(f + 1) * 33];
            let frame = UnpackedFrame::from_bit_stream_msb_first(bytes)
                .map_err(|e| CoreError::invalid(e.to_string()))?;
            let samples = self.state.decode_frame(&frame);
            for s in samples {
                pcm.extend_from_slice(&s.to_le_bytes());
            }
        }
        Ok(Frame::Audio(AudioFrame {
            samples: (FRAME_SAMPLES * n_frames) as u32,
            pts: pkt.pts,
            data: vec![pcm],
        }))
    }

    fn flush(&mut self) -> Result<()> {
        self.eof = true;
        Ok(())
    }

    fn reset(&mut self) -> Result<()> {
        self.state.reset();
        self.pending.clear();
        self.eof = false;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxideav_core::{CodecId, CodecParameters, SampleFormat, TimeBase};

    fn audio_params() -> CodecParameters {
        let mut p = CodecParameters::audio(CodecId::new(CODEC_ID));
        p.channels = Some(1);
        p.sample_rate = Some(8000);
        p.sample_format = Some(SampleFormat::S16);
        p
    }

    fn empty_packet(data: Vec<u8>) -> Packet {
        Packet::new(0, TimeBase::new(1, 8000), data)
    }

    #[test]
    fn make_decoder_rejects_stereo() {
        let mut p = audio_params();
        p.channels = Some(2);
        assert!(make_decoder(&p).is_err());
    }

    #[test]
    fn make_decoder_accepts_mono() {
        let p = audio_params();
        assert!(make_decoder(&p).is_ok());
    }

    #[test]
    fn decoder_pumps_one_frame() {
        let p = audio_params();
        let mut dec = make_decoder(&p).unwrap();
        let pkt = empty_packet(vec![0u8; 33]);
        dec.send_packet(&pkt).unwrap();
        let frame = dec.receive_frame().unwrap();
        match frame {
            Frame::Audio(a) => {
                assert_eq!(a.samples, FRAME_SAMPLES as u32);
                assert_eq!(a.data.len(), 1);
                assert_eq!(a.data[0].len(), FRAME_SAMPLES * 2);
            }
            _ => panic!("expected audio frame"),
        }
    }

    #[test]
    fn decoder_rejects_short_packet() {
        let p = audio_params();
        let mut dec = make_decoder(&p).unwrap();
        let pkt = empty_packet(vec![0u8; 32]);
        dec.send_packet(&pkt).unwrap();
        assert!(dec.receive_frame().is_err());
    }

    #[test]
    fn decoder_decodes_multiple_frames_per_packet() {
        let p = audio_params();
        let mut dec = make_decoder(&p).unwrap();
        // Three frames back-to-back in one packet (99 bytes).
        let pkt = empty_packet(vec![0u8; 33 * 3]);
        dec.send_packet(&pkt).unwrap();
        let frame = dec.receive_frame().unwrap();
        match frame {
            Frame::Audio(a) => {
                assert_eq!(a.samples, (FRAME_SAMPLES * 3) as u32);
            }
            _ => panic!("expected audio frame"),
        }
    }

    /// `reset` zeroes per-frame state. After reset, decoding the
    /// same input as a freshly-constructed decoder gives the same
    /// bytes (deterministic + state-clean).
    #[test]
    fn reset_returns_decoder_to_home_state() {
        let p = audio_params();
        let pkt = empty_packet(vec![0u8; 33]);

        // Fresh decoder: capture the output for an all-zero frame.
        let mut fresh = make_decoder(&p).unwrap();
        fresh.send_packet(&pkt).unwrap();
        let fresh_out = match fresh.receive_frame().unwrap() {
            Frame::Audio(a) => a.data,
            _ => panic!("expected audio frame"),
        };

        // Stale decoder: feed a noisy frame first, then reset, then
        // feed the same all-zero frame. The output must match the
        // fresh decoder bit-for-bit.
        let mut stale = make_decoder(&p).unwrap();
        let noisy = empty_packet(vec![0xFFu8; 33]);
        stale.send_packet(&noisy).unwrap();
        let _ = stale.receive_frame();
        stale.reset().unwrap();
        stale.send_packet(&pkt).unwrap();
        let stale_out = match stale.receive_frame().unwrap() {
            Frame::Audio(a) => a.data,
            _ => panic!("expected audio frame"),
        };

        assert_eq!(fresh_out, stale_out);
    }
}
