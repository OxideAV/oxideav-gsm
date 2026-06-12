//! Codec-registry adapter — wires the §5.3 decoder and the §5.2
//! encoder behind the `oxideav_core::Decoder` / `oxideav_core::Encoder`
//! traits so the GSM 06.10 RPE-LTP codec can be reached via the
//! workspace codec registry alongside the direct `make_decoder` /
//! `make_encoder` factories.

use std::collections::VecDeque;

use oxideav_core::{
    AudioFrame, CodecCapabilities, CodecId, CodecInfo, CodecParameters, CodecRegistry, Decoder,
    Encoder, Error as CoreError, Frame, Packet, Result, SampleFormat, TimeBase,
};

use crate::bitstream::{UnpackedFrame, FRAME_SAMPLES};
use crate::decoder::DecoderState;
use crate::encoder::EncoderState;

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

/// Build a boxed [`Encoder`] for GSM 06.10 RPE-LTP with the given
/// codec parameters. Direct-factory entry point — the
/// [`register_codecs`] path installs this same function into the
/// codec registry.
///
/// Accepts mono S16 input at 8 kHz (§1.3 / §1.5: 20 ms frames of
/// 160 samples at 8 kHz); `None` parameters default to those
/// values. Each 160-sample input frame produces one 33-byte packet
/// holding the §1.7 260-bit frame packed MSB-first (4 trailing
/// spare bits zero).
pub fn make_encoder(params: &CodecParameters) -> Result<Box<dyn Encoder>> {
    let channels = params.channels.unwrap_or(1);
    if channels != 1 {
        return Err(CoreError::unsupported(
            "GSM 06.10 RPE-LTP encoder: only mono input is defined by §1.3",
        ));
    }
    let rate = params.sample_rate.unwrap_or(8000);
    if rate != 8000 {
        return Err(CoreError::unsupported(
            "GSM 06.10 RPE-LTP encoder: §1.5 defines an 8 kHz sampling rate only",
        ));
    }
    if let Some(fmt) = params.sample_format {
        if fmt != SampleFormat::S16 {
            return Err(CoreError::unsupported(
                "GSM 06.10 RPE-LTP encoder: input must be S16 (§5.2.1 13-bit-in-16 convention)",
            ));
        }
    }

    let mut output_params = CodecParameters::audio(CodecId::new(CODEC_ID));
    output_params.channels = Some(1);
    output_params.sample_rate = Some(8000);
    output_params.sample_format = Some(SampleFormat::S16);

    Ok(Box::new(GsmEncoder {
        output_params,
        state: EncoderState::new(),
        sample_buf: Vec::new(),
        pending: VecDeque::new(),
        next_pts: None,
        flushed: false,
    }))
}

/// Register the GSM codec id into the workspace registry.
pub fn register_codecs(reg: &mut CodecRegistry) {
    let mut caps = CodecCapabilities::audio("oxideav-gsm");
    caps.decode = true;
    caps.encode = true;
    caps.lossy = true;
    caps.max_sample_rate = Some(8000);
    caps.max_channels = Some(1);
    reg.register(
        CodecInfo::new(CodecId::new(CODEC_ID))
            .capabilities(caps)
            .decoder(make_decoder)
            .encoder(make_encoder),
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
            // Apply §4.4 decoder-homing protocol: a decoder-homing
            // frame substitutes the encoder-homing-frame output and
            // resets the decoder state. Pass-through for normal
            // frames.
            let samples = self.state.decode_frame_with_homing(&frame);
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

/// Adapter between the `oxideav_core::Encoder` trait and the §5.2
/// fixed-point encoder + §1.7 bit packer.
struct GsmEncoder {
    output_params: CodecParameters,
    state: EncoderState,
    /// Mono S16 samples buffered until a whole 160-sample §1.5
    /// frame is available (input frames need not align to the
    /// 20 ms codec framing).
    sample_buf: Vec<i16>,
    /// Encoded 33-byte packets awaiting `receive_packet`.
    pending: VecDeque<Packet>,
    /// PTS (in the 1/8000 time base) of the first sample currently
    /// sitting in `sample_buf` / of the next packet to be emitted.
    next_pts: Option<i64>,
    flushed: bool,
}

impl GsmEncoder {
    /// Encode every whole 160-sample frame buffered so far into a
    /// 33-byte packet.
    fn drain_whole_frames(&mut self) {
        while self.sample_buf.len() >= FRAME_SAMPLES {
            let mut sop = [0i16; FRAME_SAMPLES];
            sop.copy_from_slice(&self.sample_buf[..FRAME_SAMPLES]);
            self.sample_buf.drain(..FRAME_SAMPLES);
            self.emit_frame(&sop);
        }
    }

    fn emit_frame(&mut self, sop: &[i16; FRAME_SAMPLES]) {
        // §4.3 encoder homing: an encoder-homing-frame input encodes
        // normally and then resets the encoder to its §4.5 home
        // state (mirroring the §4.4 path `decode` already applies).
        let coded = self.state.encode_frame_with_homing(sop);
        let bytes = coded.to_bit_stream_msb_first();
        let mut pkt = Packet::new(0, TimeBase::new(1, 8000), bytes.to_vec());
        pkt.pts = self.next_pts;
        pkt.dts = self.next_pts;
        pkt.duration = Some(FRAME_SAMPLES as i64);
        pkt.flags.keyframe = true; // every GSM frame is independently decodable entry point
        if let Some(pts) = self.next_pts.as_mut() {
            *pts += FRAME_SAMPLES as i64;
        }
        self.pending.push_back(pkt);
    }
}

impl Encoder for GsmEncoder {
    fn codec_id(&self) -> &CodecId {
        &self.output_params.codec_id
    }

    fn output_params(&self) -> &CodecParameters {
        &self.output_params
    }

    fn send_frame(&mut self, frame: &Frame) -> Result<()> {
        let audio = match frame {
            Frame::Audio(a) => a,
            _ => {
                return Err(CoreError::invalid(
                    "oxideav-gsm: encoder accepts audio frames only",
                ))
            }
        };
        if audio.data.len() != 1 {
            return Err(CoreError::invalid(format!(
                "oxideav-gsm: expected 1 interleaved mono S16 plane, got {}",
                audio.data.len()
            )));
        }
        let bytes = &audio.data[0];
        if bytes.len() % 2 != 0 {
            return Err(CoreError::invalid(
                "oxideav-gsm: S16 plane has an odd byte count",
            ));
        }
        if self.next_pts.is_none() && self.sample_buf.is_empty() {
            // Anchor the output timeline on the first buffered sample.
            self.next_pts = audio.pts.or(Some(0));
        }
        self.sample_buf.reserve(bytes.len() / 2);
        for pair in bytes.chunks_exact(2) {
            self.sample_buf.push(i16::from_le_bytes([pair[0], pair[1]]));
        }
        self.drain_whole_frames();
        Ok(())
    }

    fn receive_packet(&mut self) -> Result<Packet> {
        if let Some(pkt) = self.pending.pop_front() {
            return Ok(pkt);
        }
        if self.flushed {
            Err(CoreError::Eof)
        } else {
            Err(CoreError::NeedMore)
        }
    }

    fn flush(&mut self) -> Result<()> {
        // §1.5 frames are whole 160-sample units; a trailing partial
        // frame is zero-padded to a full frame (the padding decodes
        // to the quantiser floor, i.e. near-silence).
        if !self.sample_buf.is_empty() {
            let mut sop = [0i16; FRAME_SAMPLES];
            let n = self.sample_buf.len().min(FRAME_SAMPLES);
            sop[..n].copy_from_slice(&self.sample_buf[..n]);
            self.sample_buf.clear();
            self.emit_frame(&sop);
        }
        self.flushed = true;
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

    // ─── encoder adapter ───

    fn audio_frame(samples: &[i16], pts: Option<i64>) -> Frame {
        let mut bytes = Vec::with_capacity(samples.len() * 2);
        for s in samples {
            bytes.extend_from_slice(&s.to_le_bytes());
        }
        Frame::Audio(AudioFrame {
            samples: samples.len() as u32,
            pts,
            data: vec![bytes],
        })
    }

    #[test]
    fn make_encoder_rejects_stereo_and_wrong_rate() {
        let mut p = audio_params();
        p.channels = Some(2);
        assert!(make_encoder(&p).is_err());

        let mut p = audio_params();
        p.sample_rate = Some(16000);
        assert!(make_encoder(&p).is_err());
    }

    #[test]
    fn make_encoder_accepts_mono_8k() {
        assert!(make_encoder(&audio_params()).is_ok());
    }

    /// 160 input samples ⇒ exactly one 33-byte packet with the
    /// §1.5 timing (pts carried, duration 160 in 1/8000).
    #[test]
    fn encoder_emits_one_packet_per_160_samples() {
        let mut enc = make_encoder(&audio_params()).unwrap();
        enc.send_frame(&audio_frame(&[0i16; 160], Some(4800)))
            .unwrap();
        let pkt = enc.receive_packet().unwrap();
        assert_eq!(pkt.data.len(), 33);
        assert_eq!(pkt.pts, Some(4800));
        assert_eq!(pkt.duration, Some(160));
        // No second packet yet.
        assert!(matches!(enc.receive_packet(), Err(CoreError::NeedMore)));
    }

    /// Input framing need not align to the 20 ms codec frame: two
    /// 100-sample sends produce one packet (160) with 40 samples
    /// left buffered; flush zero-pads them into a final packet,
    /// then signals Eof.
    #[test]
    fn encoder_rebuffers_misaligned_input_and_pads_on_flush() {
        let mut enc = make_encoder(&audio_params()).unwrap();
        enc.send_frame(&audio_frame(&[100i16; 100], Some(0)))
            .unwrap();
        assert!(matches!(enc.receive_packet(), Err(CoreError::NeedMore)));
        enc.send_frame(&audio_frame(&[100i16; 100], None)).unwrap();
        let p1 = enc.receive_packet().unwrap();
        assert_eq!(p1.pts, Some(0));
        assert!(matches!(enc.receive_packet(), Err(CoreError::NeedMore)));
        enc.flush().unwrap();
        let p2 = enc.receive_packet().unwrap();
        assert_eq!(p2.pts, Some(160));
        assert_eq!(p2.data.len(), 33);
        assert!(matches!(enc.receive_packet(), Err(CoreError::Eof)));
    }

    /// Trait-level encode → decode roundtrip: every packet decodes
    /// to 160 samples and the decoded audio is non-trivial for a
    /// non-trivial input.
    #[test]
    fn encoder_decoder_roundtrip_via_traits() {
        let p = audio_params();
        let mut enc = make_encoder(&p).unwrap();
        let mut dec = make_decoder(&p).unwrap();

        // Two frames of a triangle wave (period 32, peak ±8192).
        let mut samples = Vec::with_capacity(320);
        for n in 0..320i32 {
            let phase = n % 32;
            let tri = if phase < 16 { phase - 8 } else { 23 - phase };
            samples.push((tri * 1024) as i16);
        }
        enc.send_frame(&audio_frame(&samples, Some(0))).unwrap();
        enc.flush().unwrap();

        let mut decoded_energy: i64 = 0;
        for _ in 0..2 {
            let pkt = enc.receive_packet().unwrap();
            dec.send_packet(&pkt).unwrap();
            match dec.receive_frame().unwrap() {
                Frame::Audio(a) => {
                    assert_eq!(a.samples, 160);
                    for pair in a.data[0].chunks_exact(2) {
                        let v = i16::from_le_bytes([pair[0], pair[1]]) as i64;
                        decoded_energy += v * v;
                    }
                }
                _ => panic!("expected audio frame"),
            }
        }
        assert!(matches!(enc.receive_packet(), Err(CoreError::Eof)));
        assert!(decoded_energy > 0, "decoded signal must carry energy");
    }

    /// Registry exposes both directions for the "gsm" id.
    #[test]
    fn registry_has_encoder_and_decoder() {
        let mut reg = CodecRegistry::new();
        register_codecs(&mut reg);
        let id = CodecId::new(CODEC_ID);
        assert!(reg.has_decoder(&id));
        assert!(reg.has_encoder(&id));
    }
}
