//! ETSI EN 300 961 §6.1 conformance test-sequence I/O formats.
//!
//! The §6 digital test sequences use a **word-oriented** on-disk format
//! distinct from the §1.7 in-band 260-bit speech frame: every value is
//! stored in its own 16-bit word. §6.1 (Table 6.1a/b/c) fixes the
//! justification of each word:
//!
//! > *"Table 6.1 defines the input and output signals for the test
//! > sequences. The words defined in this table use 16 bits. The left
//! > or right justification is indicated in the table. The codewords
//! > described in the table correspond to one frame of coder input or
//! > decoder output signal."* (§6.1)
//!
//! Three file types (§6.1):
//!
//! * `*.INP` — encoder input `sop[k]`: **13 bits, left** justified
//!   (Table 6.1a). One 16-bit word per PCM sample, 160 words/frame.
//! * `*.COD` — coded parameters `LARc[1..=8]` + four sub-frames of
//!   `Nc bc Mc xmaxc xMc[0..=12]`: each **right** justified
//!   (Table 6.1b). 76 words/frame.
//! * `*.OUT` — decoder output `srop[k]`: **13 bits, left** justified,
//!   *"The 3 LSB's of the 16 bits are equal to 0"* (Table 6.1c). One
//!   16-bit word per sample, 160 words/frame.
//!
//! This module converts between those word-oriented frames and the
//! crate's typed structures, so a conformance harness that stages the
//! ETSI `*.INP` / `*.COD` / `*.OUT` files (currently unstaged under
//! `docs/audio/gsm/`) can read and write them directly. It is the
//! counterpart to [`crate::bitstream`], which handles the §1.7 *packed*
//! 260-bit in-band stream rather than the §6.1 word-oriented files.
//!
//! ## `*.COD` justification (Table 6.1b)
//!
//! Every coded parameter is **right** justified in its 16-bit word:
//! the value occupies the low bits of the word and the high bits are
//! zero. So `LARc[1]` (6 bits) is `0x0000..=0x003F`, `Nc` (7 bits) is
//! `0x0000..=0x007F`, and so on. The 76 words appear in §1.7 Table 1.1
//! order: the eight LARc, then the four sub-frames, each as
//! `Nc, bc, Mc, xmaxc, xMc[0], …, xMc[12]`.
//!
//! Byte order within each 16-bit word follows the conformance archive
//! convention; this module reads/writes **little-endian** words (the
//! native byte order of the linear-PCM `.INP`/`.OUT` companions), and a
//! big-endian variant is offered explicitly where a caller needs it.

use crate::bitstream::{SubFrame, UnpackedFrame, FRAME_SAMPLES, PULSES, SUBFRAMES};
use crate::error::Error;

/// Number of 16-bit words in one `*.COD` frame (§6.1 Table 6.1b):
/// 8 LARc + 4 × (Nc, bc, Mc, xmaxc, 13 × xMc) = 8 + 4 × 17 = 76.
pub const COD_WORDS_PER_FRAME: usize = 76;

/// Number of bytes in one `*.COD` frame: [`COD_WORDS_PER_FRAME`] × 2
/// (§6.3.3.4 *"SIZE (SYNCXXX.COD) = 76 * 2 bytes = 152 bytes"*).
pub const COD_BYTES_PER_FRAME: usize = COD_WORDS_PER_FRAME * 2;

/// Number of 16-bit words in one `*.INP` / `*.OUT` PCM frame:
/// one per [`FRAME_SAMPLES`] sample.
pub const PCM_WORDS_PER_FRAME: usize = FRAME_SAMPLES;

/// Number of bytes in one `*.INP` / `*.OUT` PCM frame.
pub const PCM_BYTES_PER_FRAME: usize = PCM_WORDS_PER_FRAME * 2;

/// The §1.7 Table 1.1 per-parameter bit widths, in `*.COD` word order:
/// eight LARc then, per sub-frame, `Nc bc Mc xmaxc` and 13 `xMc`.
const fn cod_field_widths() -> [u8; COD_WORDS_PER_FRAME] {
    let mut w = [0u8; COD_WORDS_PER_FRAME];
    // LARc[1..=8]: 6,6,5,5,4,4,3,3.
    let lar: [u8; 8] = [6, 6, 5, 5, 4, 4, 3, 3];
    let mut i = 0;
    while i < 8 {
        w[i] = lar[i];
        i += 1;
    }
    // Four sub-frames of Nc(7) bc(2) Mc(2) xmaxc(6) then 13 × xMc(3).
    let mut sf = 0;
    while sf < SUBFRAMES {
        let base = 8 + sf * 17;
        w[base] = 7; // Nc
        w[base + 1] = 2; // bc
        w[base + 2] = 2; // Mc
        w[base + 3] = 6; // xmaxc
        let mut p = 0;
        while p < PULSES {
            w[base + 4 + p] = 3;
            p += 1;
        }
        sf += 1;
    }
    w
}

/// The 76 per-word field widths of a `*.COD` frame (Table 6.1b),
/// in word order.
pub const COD_FIELD_WIDTHS: [u8; COD_WORDS_PER_FRAME] = cod_field_widths();

/// Flatten an [`UnpackedFrame`] into its 76 `*.COD` parameter words,
/// in §1.7 Table 1.1 order (eight LARc, then four sub-frames). Each
/// returned value is the raw right-justified codeword (Table 6.1b).
pub fn unpacked_to_cod_words(frame: &UnpackedFrame) -> [u16; COD_WORDS_PER_FRAME] {
    let mut out = [0u16; COD_WORDS_PER_FRAME];
    for (i, slot) in out.iter_mut().take(8).enumerate() {
        *slot = frame.lar_c[i + 1] as u16;
    }
    for (sf, sub) in frame.sub.iter().enumerate() {
        let base = 8 + sf * 17;
        out[base] = sub.n_c as u16;
        out[base + 1] = sub.b_c as u16;
        out[base + 2] = sub.m_c as u16;
        out[base + 3] = sub.xmax_c as u16;
        for (p, &pulse) in sub.x_mc.iter().enumerate() {
            out[base + 4 + p] = pulse as u16;
        }
    }
    out
}

/// Rebuild an [`UnpackedFrame`] from 76 `*.COD` parameter words.
///
/// Each word is masked to its Table 6.1b field width before use, so a
/// `*.COD` word whose unused high bits are non-zero (the §6.1 words are
/// right-justified; high bits should be zero) cannot bleed into the
/// decoded value. The §6.3.2 SEQ05 vector deliberately scans the full
/// codeword range — including `Nc ∈ [0,127]` rather than the
/// error-free `[40,120]` — so the masking keeps a 7-bit `Nc` field
/// intact and leaves the out-of-range-lag handling to the §5.3.2
/// decoder (which clamps it).
pub fn cod_words_to_unpacked(words: &[u16; COD_WORDS_PER_FRAME]) -> UnpackedFrame {
    let mut f = UnpackedFrame::default();
    for (i, &word) in words.iter().take(8).enumerate() {
        let mask = (1u16 << COD_FIELD_WIDTHS[i]) - 1;
        f.lar_c[i + 1] = (word & mask) as i16;
    }
    for sf in 0..SUBFRAMES {
        let base = 8 + sf * 17;
        let m = |idx: usize| -> u16 { words[idx] & ((1u16 << COD_FIELD_WIDTHS[idx]) - 1) };
        let sub = SubFrame {
            n_c: m(base) as u8,
            b_c: m(base + 1) as u8,
            m_c: m(base + 2) as u8,
            xmax_c: m(base + 3) as u8,
            x_mc: core::array::from_fn(|p| m(base + 4 + p) as u8),
        };
        f.sub[sf] = sub;
    }
    f
}

/// Serialise an [`UnpackedFrame`] to the 152-byte little-endian `*.COD`
/// frame (§6.1 Table 6.1b / §6.3.3.4).
pub fn unpacked_to_cod_bytes_le(frame: &UnpackedFrame) -> [u8; COD_BYTES_PER_FRAME] {
    let words = unpacked_to_cod_words(frame);
    let mut bytes = [0u8; COD_BYTES_PER_FRAME];
    for (w, chunk) in words.iter().zip(bytes.chunks_exact_mut(2)) {
        chunk.copy_from_slice(&w.to_le_bytes());
    }
    bytes
}

/// Serialise an [`UnpackedFrame`] to the 152-byte big-endian `*.COD`
/// frame (the same Table 6.1b words, network byte order).
pub fn unpacked_to_cod_bytes_be(frame: &UnpackedFrame) -> [u8; COD_BYTES_PER_FRAME] {
    let words = unpacked_to_cod_words(frame);
    let mut bytes = [0u8; COD_BYTES_PER_FRAME];
    for (w, chunk) in words.iter().zip(bytes.chunks_exact_mut(2)) {
        chunk.copy_from_slice(&w.to_be_bytes());
    }
    bytes
}

/// Parse a 152-byte little-endian `*.COD` frame into an
/// [`UnpackedFrame`]. Returns [`Error::ShortFrame`] when fewer than
/// [`COD_BYTES_PER_FRAME`] bytes are supplied.
pub fn cod_bytes_le_to_unpacked(bytes: &[u8]) -> Result<UnpackedFrame, Error> {
    if bytes.len() < COD_BYTES_PER_FRAME {
        return Err(Error::ShortFrame);
    }
    let mut words = [0u16; COD_WORDS_PER_FRAME];
    for (w, chunk) in words.iter_mut().zip(bytes.chunks_exact(2)) {
        *w = u16::from_le_bytes([chunk[0], chunk[1]]);
    }
    Ok(cod_words_to_unpacked(&words))
}

/// Parse a 152-byte big-endian `*.COD` frame into an [`UnpackedFrame`].
pub fn cod_bytes_be_to_unpacked(bytes: &[u8]) -> Result<UnpackedFrame, Error> {
    if bytes.len() < COD_BYTES_PER_FRAME {
        return Err(Error::ShortFrame);
    }
    let mut words = [0u16; COD_WORDS_PER_FRAME];
    for (w, chunk) in words.iter_mut().zip(bytes.chunks_exact(2)) {
        *w = u16::from_be_bytes([chunk[0], chunk[1]]);
    }
    Ok(cod_words_to_unpacked(&words))
}

/// Serialise 160 PCM samples to a 320-byte little-endian `*.INP` /
/// `*.OUT` frame.
///
/// Table 6.1a/c: `sop`/`srop` are 13-bit values **left** justified into
/// 16-bit words; for `*.OUT` the three low bits are zero (§5.3.7 output
/// shaping already guarantees this for decoder output). The samples are
/// stored verbatim — the §4.2 / §5.3.7 left-justified 16-bit form `<<3`
/// is already what the codec produces (e.g. `0x0008` for the homing
/// sample), so no extra shift is applied here.
pub fn pcm_to_inp_bytes_le(pcm: &[i16; FRAME_SAMPLES]) -> [u8; PCM_BYTES_PER_FRAME] {
    let mut bytes = [0u8; PCM_BYTES_PER_FRAME];
    for (s, chunk) in pcm.iter().zip(bytes.chunks_exact_mut(2)) {
        chunk.copy_from_slice(&s.to_le_bytes());
    }
    bytes
}

/// Parse a 320-byte little-endian `*.INP` / `*.OUT` frame into 160 PCM
/// samples. Returns [`Error::ShortFrame`] on too few bytes.
pub fn inp_bytes_le_to_pcm(bytes: &[u8]) -> Result<[i16; FRAME_SAMPLES], Error> {
    if bytes.len() < PCM_BYTES_PER_FRAME {
        return Err(Error::ShortFrame);
    }
    let mut pcm = [0i16; FRAME_SAMPLES];
    for (s, chunk) in pcm.iter_mut().zip(bytes.chunks_exact(2)) {
        *s = i16::from_le_bytes([chunk[0], chunk[1]]);
    }
    Ok(pcm)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::decoder::decoder_homing_frame;

    #[test]
    fn cod_frame_is_76_words_152_bytes() {
        assert_eq!(COD_WORDS_PER_FRAME, 76);
        assert_eq!(COD_BYTES_PER_FRAME, 152);
        // Matches the §6.3.3.4 stated SYNCxxx.COD size.
        assert_eq!(COD_BYTES_PER_FRAME, crate::SyncFormats::SYNC_COD_BYTES);
    }

    #[test]
    fn cod_field_widths_sum_to_260_bits() {
        // The 76 right-justified words carry the same 260 information
        // bits as the §1.7 in-band frame.
        let total: u32 = COD_FIELD_WIDTHS.iter().map(|&w| w as u32).sum();
        assert_eq!(total, crate::FRAME_BITS as u32);
    }

    #[test]
    fn cod_words_roundtrip_unpacked() {
        let f = decoder_homing_frame();
        let words = unpacked_to_cod_words(&f);
        // LARc words are right-justified (== the raw codeword).
        assert_eq!(words[0], f.lar_c[1] as u16);
        // First sub-frame Nc is word index 8.
        assert_eq!(words[8], f.sub[0].n_c as u16);
        let g = cod_words_to_unpacked(&words);
        assert_eq!(f, g);
    }

    #[test]
    fn cod_words_are_right_justified() {
        // Every word must fit in its Table 6.1b field width (high bits
        // zero) for a frame produced by the codec.
        let f = decoder_homing_frame();
        let words = unpacked_to_cod_words(&f);
        for (i, &w) in words.iter().enumerate() {
            let max = (1u32 << COD_FIELD_WIDTHS[i]) - 1;
            assert!(
                (w as u32) <= max,
                "word {i} = {w} exceeds {}-bit field",
                COD_FIELD_WIDTHS[i]
            );
        }
    }

    #[test]
    fn cod_bytes_le_roundtrip() {
        let f = decoder_homing_frame();
        let bytes = unpacked_to_cod_bytes_le(&f);
        assert_eq!(bytes.len(), 152);
        let g = cod_bytes_le_to_unpacked(&bytes).unwrap();
        assert_eq!(f, g);
    }

    #[test]
    fn cod_bytes_be_roundtrip() {
        let f = decoder_homing_frame();
        let bytes = unpacked_to_cod_bytes_be(&f);
        let g = cod_bytes_be_to_unpacked(&bytes).unwrap();
        assert_eq!(f, g);
    }

    #[test]
    fn cod_le_and_be_differ_only_in_byte_order() {
        let f = decoder_homing_frame();
        let le = unpacked_to_cod_bytes_le(&f);
        let be = unpacked_to_cod_bytes_be(&f);
        for chunk in (0..76).map(|i| i * 2) {
            assert_eq!(le[chunk], be[chunk + 1]);
            assert_eq!(le[chunk + 1], be[chunk]);
        }
    }

    #[test]
    fn cod_short_frame_rejected() {
        assert!(matches!(
            cod_bytes_le_to_unpacked(&[0u8; 151]),
            Err(Error::ShortFrame)
        ));
        assert!(matches!(
            cod_bytes_be_to_unpacked(&[]),
            Err(Error::ShortFrame)
        ));
    }

    #[test]
    fn seq05_nr_full_range_survives_cod_roundtrip() {
        // §6.3.2 SEQ05: "the delay value Nr belonging to [40,120] in an
        // error-free transmission condition, takes in this sequence its
        // value in [0,127]." A 7-bit Nc field must survive the *.COD
        // roundtrip unchanged across the whole [0,127] range so the
        // §5.3.2 decoder can apply its own out-of-range handling.
        for nc in 0u8..=127 {
            let mut f = UnpackedFrame::default();
            f.sub[0].n_c = nc;
            f.sub[3].n_c = 127 - nc;
            let words = unpacked_to_cod_words(&f);
            let g = cod_words_to_unpacked(&words);
            assert_eq!(g.sub[0].n_c, nc);
            assert_eq!(g.sub[3].n_c, 127 - nc);
        }
    }

    #[test]
    fn pcm_inp_roundtrip_and_homing_sample() {
        let pcm = crate::encoder_homing_frame_pcm();
        let bytes = pcm_to_inp_bytes_le(&pcm);
        assert_eq!(bytes.len(), 320);
        // Every encoder-homing sample is the left-justified 0x0008 word.
        assert_eq!(u16::from_le_bytes([bytes[0], bytes[1]]), 0x0008);
        let back = inp_bytes_le_to_pcm(&bytes).unwrap();
        assert_eq!(back, pcm);
    }

    #[test]
    fn out_low_three_bits_zero_for_decoder_output() {
        // §5.3.7 / Table 6.1c — the decoder output's three low bits are
        // zero; a *.OUT frame written from such output preserves that.
        let mut dec = crate::DecoderState::new();
        let out = dec.decode_frame(&UnpackedFrame::default());
        let bytes = pcm_to_inp_bytes_le(&out);
        for chunk in bytes.chunks_exact(2) {
            let w = i16::from_le_bytes([chunk[0], chunk[1]]);
            assert_eq!(w & 0b111, 0, "§6.1 Table 6.1c: 3 LSBs zero");
        }
    }

    #[test]
    fn pcm_short_frame_rejected() {
        assert!(matches!(
            inp_bytes_le_to_pcm(&[0u8; 319]),
            Err(Error::ShortFrame)
        ));
    }

    /// The §6.1 word-oriented `*.COD` format and the §1.7 in-band
    /// 33-byte stream carry the **same 76 parameters**: round-tripping
    /// an arbitrary frame through one and then the other must be
    /// identity. This is the cross-format equivalence the two
    /// representations must satisfy.
    #[test]
    fn cod_words_agree_with_inband_packing() {
        // A patterned frame exercising every field (distinct in-range
        // values), the same shape the §1.7 packer round-trips.
        let mut f = UnpackedFrame::default();
        let lar_max: [i16; 8] = [63, 63, 31, 31, 15, 15, 7, 7];
        for i in 1..=8 {
            f.lar_c[i] = (lar_max[i - 1] - (i as i16 - 1)).max(1);
        }
        for (s, sf) in f.sub.iter_mut().enumerate() {
            let s8 = s as u8;
            sf.n_c = 40 + 17 * s8;
            sf.b_c = s8 & 0x3;
            sf.m_c = (3 - s8) & 0x3;
            sf.xmax_c = 5 + 13 * s8;
            for (p, slot) in sf.x_mc.iter_mut().enumerate() {
                *slot = ((p as u8) + s8) % 8;
            }
        }

        // §6.1 *.COD round-trip.
        let via_cod = cod_words_to_unpacked(&unpacked_to_cod_words(&f));
        // §1.7 in-band round-trip.
        let via_inband =
            UnpackedFrame::from_bit_stream_msb_first(&f.to_bit_stream_msb_first()).unwrap();

        assert_eq!(via_cod, f, "*.COD round-trip must be identity");
        assert_eq!(via_inband, f, "§1.7 in-band round-trip must be identity");
        assert_eq!(
            via_cod, via_inband,
            "both carriage formats must decode to the same 76 parameters"
        );
    }

    /// §5.1 non-valid-bit robustness (parameter side). The spec
    /// requires: *"At the receiving part it shall therefore be ensured
    /// that only valid bits … (two to seven bits for coded parameters)
    /// are used. In verification tests, the testing system may
    /// introduce random bit at non valid places inside these … or
    /// parameters (MSBs) to test this function."* So a `*.COD` word
    /// whose **high** (unused) bits are set to garbage must decode to
    /// the same parameters as the clean word — the reader masks each
    /// word to its Table 6.1b field width.
    #[test]
    fn cod_reader_ignores_garbage_high_bits() {
        let clean = decoder_homing_frame();
        let clean_words = unpacked_to_cod_words(&clean);

        // Set every bit ABOVE each field's width (the "MSBs … non valid
        // places") to 1, leaving the valid low bits intact.
        let mut dirty = clean_words;
        for (i, w) in dirty.iter_mut().enumerate() {
            let valid_mask = (1u16 << COD_FIELD_WIDTHS[i]) - 1;
            *w |= !valid_mask; // pollute all high bits
        }
        // The polluted words must decode to the identical frame.
        let decoded = cod_words_to_unpacked(&dirty);
        assert_eq!(
            decoded, clean,
            "§5.1: garbage in a *.COD word's MSBs must be ignored"
        );

        // And the same through the byte path.
        let mut bytes = [0u8; COD_BYTES_PER_FRAME];
        for (w, chunk) in dirty.iter().zip(bytes.chunks_exact_mut(2)) {
            chunk.copy_from_slice(&w.to_le_bytes());
        }
        assert_eq!(cod_bytes_le_to_unpacked(&bytes).unwrap(), clean);
    }

    /// §5.1 non-valid-bit robustness (sample side). The spec also
    /// allows random bits "inside these samples (3 LSBs)" of the
    /// 13-bit PCM. The §5.2.1 encoder downscale clears those three
    /// low bits, so an `*.INP` frame with garbage in the 3 LSBs of
    /// every sample encodes identically to the clean one.
    #[test]
    fn encoder_ignores_garbage_sample_lsbs() {
        let clean: [i16; FRAME_SAMPLES] =
            core::array::from_fn(|i| (((i as i16) - 80) << 3) & 0x7ff8);
        // Pollute the 3 LSBs of every sample.
        let dirty: [i16; FRAME_SAMPLES] = core::array::from_fn(|i| clean[i] | 0b111);

        let mut a = crate::EncoderState::new();
        let mut b = crate::EncoderState::new();
        let ca = a.encode_frame(&clean);
        let cb = b.encode_frame(&dirty);
        assert_eq!(
            ca, cb,
            "§5.1 / §5.2.1: garbage in a sample's 3 LSBs must not affect the coded output"
        );
    }
}
