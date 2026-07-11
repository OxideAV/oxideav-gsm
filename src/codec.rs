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

use crate::bitstream::{UnpackedFrame, FRAME_SAMPLES, GSM_BYTE_FRAME_LEN, MSGSM_BLOCK_LEN};
use crate::decoder::DecoderState;
use crate::encoder::EncoderState;

/// Canonical codec id for GSM 06.10 RPE-LTP — the conventional
/// string token tooling uses for the GSM Full Rate codec.
pub const CODEC_ID: &str = "gsm";

/// Packet-level frame packaging spoken by the adapters, selected via
/// `CodecParameters::extradata` (a codec-defined field per its core
/// docs). The coded parameters are identical in all three — only the
/// bit/byte packaging differs.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum FramePacking {
    /// Empty extradata or `b"inband"` — the spec's abstract §1.7
    /// `b1..b260` stream packed MSB-first into 33 bytes (4 spare
    /// trailing bits zero). The crate's historical adapter format.
    #[default]
    InBand,
    /// `b"gsm"` — the de-facto 33-byte `.gsm` byte-frame (0xD marker
    /// nibble, MSB-first fields) used by raw `.gsm` files. See
    /// [`UnpackedFrame::from_gsm_byte_frame`].
    ByteFrame,
    /// `b"msgsm"` — the MS-GSM 65-byte two-frame block (WAVE format
    /// tag `0x0031`, 320 samples per block). See
    /// [`UnpackedFrame::pair_from_msgsm_block`].
    Msgsm,
}

impl FramePacking {
    /// Parse the `CodecParameters::extradata` selector.
    fn from_params(params: &CodecParameters) -> Result<Self> {
        match params.extradata.as_slice() {
            b"" | b"inband" => Ok(Self::InBand),
            b"gsm" => Ok(Self::ByteFrame),
            b"msgsm" => Ok(Self::Msgsm),
            other => Err(CoreError::unsupported(format!(
                "oxideav-gsm: unknown frame-packing extradata {:?} (expected \"inband\", \"gsm\", or \"msgsm\")",
                String::from_utf8_lossy(other)
            ))),
        }
    }

    /// Packet payload unit in bytes.
    fn unit_len(self) -> usize {
        match self {
            Self::InBand | Self::ByteFrame => GSM_BYTE_FRAME_LEN,
            Self::Msgsm => MSGSM_BLOCK_LEN,
        }
    }
}

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
    let packing = FramePacking::from_params(params)?;
    Ok(Box::new(GsmDecoder {
        codec_id: params.codec_id.clone(),
        packing,
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
/// values. The packet payload format is selected by
/// [`FramePacking`] via `params.extradata`: by default each
/// 160-sample input frame produces one 33-byte packet holding the
/// §1.7 260-bit frame packed MSB-first (4 trailing spare bits
/// zero); `b"gsm"` emits de-facto `.gsm` byte-frames instead, and
/// `b"msgsm"` emits one 65-byte MS-GSM block (320 samples,
/// pts/duration spanning both frames) per two input frames.
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

    let packing = FramePacking::from_params(params)?;

    let mut output_params = CodecParameters::audio(CodecId::new(CODEC_ID));
    output_params.channels = Some(1);
    output_params.sample_rate = Some(8000);
    output_params.sample_format = Some(SampleFormat::S16);
    // Advertise the selected packing to muxers the same way it was
    // selected.
    output_params.extradata = params.extradata.clone();

    Ok(Box::new(GsmEncoder {
        output_params,
        packing,
        state: EncoderState::new(),
        sample_buf: Vec::new(),
        half_block: None,
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
    /// Packet-level frame packaging (extradata-selected).
    packing: FramePacking,
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
        let unit = self.packing.unit_len();
        if pkt.data.len() < unit {
            return Err(CoreError::invalid(format!(
                "oxideav-gsm: packet shorter than the {unit}-byte minimum (got {})",
                pkt.data.len()
            )));
        }

        // §1.5 — 20 ms frame ⇒ 160 PCM samples per 260-bit frame.
        // Concatenate whole payload units found in this packet so a
        // muxer that delivers multiple frames/blocks per packet
        // still works.
        let n_units = pkt.data.len() / unit;
        let mut frames: Vec<UnpackedFrame> = Vec::with_capacity(n_units * 2);
        for u in 0..n_units {
            let bytes = &pkt.data[u * unit..(u + 1) * unit];
            match self.packing {
                FramePacking::InBand => frames.push(
                    UnpackedFrame::from_bit_stream_msb_first(bytes)
                        .map_err(|e| CoreError::invalid(e.to_string()))?,
                ),
                FramePacking::ByteFrame => frames.push(
                    UnpackedFrame::from_gsm_byte_frame(bytes)
                        .map_err(|e| CoreError::invalid(e.to_string()))?,
                ),
                FramePacking::Msgsm => {
                    let (a, b) = UnpackedFrame::pair_from_msgsm_block(bytes)
                        .map_err(|e| CoreError::invalid(e.to_string()))?;
                    frames.push(a);
                    frames.push(b);
                }
            }
        }

        let mut pcm = Vec::with_capacity(FRAME_SAMPLES * frames.len() * 2);
        for frame in &frames {
            // Apply §4.4 decoder-homing protocol: a decoder-homing
            // frame substitutes the encoder-homing-frame output and
            // resets the decoder state. Pass-through for normal
            // frames.
            let samples = self.state.decode_frame_with_homing(frame);
            for s in samples {
                pcm.extend_from_slice(&s.to_le_bytes());
            }
        }
        Ok(Frame::Audio(AudioFrame {
            samples: (FRAME_SAMPLES * frames.len()) as u32,
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
    /// Packet-level frame packaging (extradata-selected).
    packing: FramePacking,
    state: EncoderState,
    /// Mono S16 samples buffered until a whole 160-sample §1.5
    /// frame is available (input frames need not align to the
    /// 20 ms codec framing).
    sample_buf: Vec<i16>,
    /// MS-GSM only: the first (already-encoded) frame of a 65-byte
    /// block waiting for its partner, with the pts of its first
    /// sample.
    half_block: Option<(UnpackedFrame, Option<i64>)>,
    /// Encoded packets awaiting `receive_packet`.
    pending: VecDeque<Packet>,
    /// PTS (in the 1/8000 time base) of the first sample currently
    /// sitting in `sample_buf` / of the next packet to be emitted.
    next_pts: Option<i64>,
    flushed: bool,
}

impl GsmEncoder {
    /// Encode every whole 160-sample frame buffered so far and emit
    /// packets per the selected [`FramePacking`].
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
        // Consume this frame's 160-sample slot on the running output
        // timeline up front; packets carry the pts captured before
        // the advance.
        let pts = self.next_pts;
        if let Some(p) = self.next_pts.as_mut() {
            *p += FRAME_SAMPLES as i64;
        }
        match self.packing {
            FramePacking::InBand => {
                let bytes = coded.to_bit_stream_msb_first();
                self.emit_packet(bytes.to_vec(), pts, FRAME_SAMPLES as i64);
            }
            FramePacking::ByteFrame => {
                let bytes = coded.to_gsm_byte_frame();
                self.emit_packet(bytes.to_vec(), pts, FRAME_SAMPLES as i64);
            }
            FramePacking::Msgsm => match self.half_block.take() {
                None => {
                    // First frame of a 65-byte block: stash it with
                    // its pts until the partner frame arrives.
                    self.half_block = Some((coded, pts));
                }
                Some((first, first_pts)) => {
                    // The packet spans both frames: pts of the first
                    // frame, duration 320.
                    let bytes = UnpackedFrame::pair_to_msgsm_block(&first, &coded);
                    self.emit_packet(bytes.to_vec(), first_pts, 2 * FRAME_SAMPLES as i64);
                }
            },
        }
    }

    fn emit_packet(&mut self, data: Vec<u8>, pts: Option<i64>, duration: i64) {
        let mut pkt = Packet::new(0, TimeBase::new(1, 8000), data);
        pkt.pts = pts;
        pkt.dts = pts;
        pkt.duration = Some(duration);
        pkt.flags.keyframe = true; // every GSM frame is an independently decodable entry point
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
        // MS-GSM blocks are whole 65-byte / two-frame units; a
        // trailing lone frame is completed with an encoded silence
        // frame (the container's sample count — e.g. the WAVE `fact`
        // chunk — is the muxer's business).
        if self.half_block.is_some() {
            self.emit_frame(&[0i16; FRAME_SAMPLES]);
            debug_assert!(self.half_block.is_none());
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

    // ─── extradata-selected frame packing ───

    fn params_with_packing(extradata: &[u8]) -> CodecParameters {
        let mut p = audio_params();
        p.extradata = extradata.to_vec();
        p
    }

    /// Unknown extradata tokens are rejected on both factories;
    /// the three defined tokens (and empty) are accepted.
    #[test]
    fn packing_selector_validation() {
        for good in [&b""[..], b"inband", b"gsm", b"msgsm"] {
            assert!(make_decoder(&params_with_packing(good)).is_ok());
            assert!(make_encoder(&params_with_packing(good)).is_ok());
        }
        assert!(make_decoder(&params_with_packing(b"wav49")).is_err());
        assert!(make_encoder(&params_with_packing(b"bogus")).is_err());
    }

    /// Collect all of an encoder's pending packets after a flush.
    fn drain_packets(enc: &mut Box<dyn Encoder>) -> Vec<Packet> {
        let mut out = Vec::new();
        loop {
            match enc.receive_packet() {
                Ok(p) => out.push(p),
                Err(CoreError::Eof) => break,
                Err(e) => panic!("unexpected encoder error: {e}"),
            }
        }
        out
    }

    /// Decode every packet through a fresh decoder with the given
    /// packing and return the PCM.
    fn decode_via(packing: &[u8], packets: &[Packet]) -> Vec<i16> {
        let mut dec = make_decoder(&params_with_packing(packing)).unwrap();
        let mut pcm = Vec::new();
        for pkt in packets {
            dec.send_packet(pkt).unwrap();
            match dec.receive_frame().unwrap() {
                Frame::Audio(a) => {
                    for pair in a.data[0].chunks_exact(2) {
                        pcm.push(i16::from_le_bytes([pair[0], pair[1]]));
                    }
                }
                _ => panic!("expected audio"),
            }
        }
        pcm
    }

    fn triangle(samples: usize) -> Vec<i16> {
        (0..samples as i32)
            .map(|n| {
                let phase = n % 32;
                let tri = if phase < 16 { phase - 8 } else { 23 - phase };
                (tri * 1024) as i16
            })
            .collect()
    }

    /// The three packings carry the identical parameter stream: the
    /// same input encodes and decodes to the identical PCM under all
    /// three, with the expected packet shapes.
    #[test]
    fn packings_are_transparent_reencodings() {
        let samples = triangle(640); // 4 frames = 2 MS-GSM blocks
        let mut reference: Option<Vec<i16>> = None;
        for (packing, unit, per_pkt_frames) in [
            (&b""[..], 33usize, 1usize),
            (b"gsm", 33, 1),
            (b"msgsm", 65, 2),
        ] {
            let mut enc = make_encoder(&params_with_packing(packing)).unwrap();
            enc.send_frame(&audio_frame(&samples, Some(0))).unwrap();
            enc.flush().unwrap();
            let packets = drain_packets(&mut enc);
            assert_eq!(packets.len(), 4 / per_pkt_frames);
            for (i, pkt) in packets.iter().enumerate() {
                assert_eq!(pkt.data.len(), unit);
                assert_eq!(pkt.pts, Some((i * per_pkt_frames * 160) as i64));
                assert_eq!(pkt.duration, Some((per_pkt_frames * 160) as i64));
            }
            let pcm = decode_via(packing, &packets);
            assert_eq!(pcm.len(), 640);
            match &reference {
                None => reference = Some(pcm),
                Some(r) => assert_eq!(&pcm, r, "packing {packing:?} must be transparent"),
            }
        }
    }

    /// MS-GSM: an odd number of input frames is completed with an
    /// encoded silence frame on flush, so the last packet is still a
    /// whole 65-byte block.
    #[test]
    fn msgsm_flush_completes_trailing_half_block() {
        let mut enc = make_encoder(&params_with_packing(b"msgsm")).unwrap();
        enc.send_frame(&audio_frame(&triangle(480), Some(0)))
            .unwrap(); // 3 frames
                       // Frames 1+2 complete a block immediately; frame 3 sits in
                       // the half-block buffer until flush.
        let first = enc.receive_packet().unwrap();
        assert_eq!(first.data.len(), 65);
        assert!(matches!(enc.receive_packet(), Err(CoreError::NeedMore)));
        enc.flush().unwrap();
        let mut packets = vec![first];
        packets.extend(drain_packets(&mut enc));
        assert_eq!(packets.len(), 2);
        assert_eq!(packets[0].data.len(), 65);
        assert_eq!(packets[1].data.len(), 65);
        assert_eq!(packets[1].pts, Some(320));
        assert_eq!(packets[1].duration, Some(320));
        // The completed block still decodes to 320 samples (the tail
        // 160 being the encoded-silence filler).
        let pcm = decode_via(b"msgsm", &packets);
        assert_eq!(pcm.len(), 640);
    }

    /// The byte-frame decoder path enforces the 0xD marker.
    #[test]
    fn byte_frame_decoder_rejects_bad_marker() {
        let mut dec = make_decoder(&params_with_packing(b"gsm")).unwrap();
        let pkt = empty_packet(vec![0u8; 33]); // marker nibble 0x0
        dec.send_packet(&pkt).unwrap();
        assert!(dec.receive_frame().is_err());
    }
}
