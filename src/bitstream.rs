//! Frame structure and bit unpacking per ETSI EN 300 961 §1.7
//! Table 1.1 "Encoder output parameters in order of occurrence and
//! bit allocation within the speech frame of 260 bits/20 ms".
//!
//! Per §1.7 the encoder produces a fixed-order sequence of 76
//! coded parameters totalling 260 bits per 20 ms speech frame. The
//! parameters group into three families:
//!
//! 1. **Eight LAR (Log.-Area Ratio) coefficients** `LARc[1..=8]`,
//!    sent once per frame. Their widths are 6/6/5/5/4/4/3/3 bits
//!    (36 bits total, b1..b36).
//! 2. **Four sub-frames**, each carrying nine parameters (LTP lag
//!    `Nc`, LTP gain `bc`, RPE grid `Mc`, block amplitude
//!    `xmaxc`, and thirteen 3-bit RPE pulses `xMc[0..=12]`) for
//!    56 bits per sub-frame and 224 bits total (b37..b260).
//!
//! Table 1.1 lists each parameter's start/end bit position in the
//! frame. The column header reads "(LSB-MSB)" — the on-wire ordering
//! of each parameter is least-significant bit first within its
//! allocated bit range.
//!
//! Per §6.1 the frame is transmitted as "words … of 16 bits" with
//! left justification, i.e. the abstract 260-bit stream `b1..b260`
//! is the contractual interface; how that stream is then packed
//! into bytes (the `.gsm` 33-byte byte format, MS-GSM WAV chunk,
//! TRAU 320-bit, …) is **outside** the scope of EN 300 961.
//!
//! This module therefore exposes three layers:
//!
//! * [`UnpackedFrame`] — the typed structure of 76 parameters that
//!   §5.3 decoder consumes directly.
//! * [`UnpackedFrame::from_bit_stream_msb_first`] — a 260-bit stream
//!   unpacker whose only assumption is "stream is `b1..b260` packed
//!   MSB-first into a `[u8; 33]`-or-larger byte buffer" (the bit
//!   positions match Table 1.1 verbatim).
//! * [`UnpackedFrame::from_gsm_byte_frame`] /
//!   [`UnpackedFrame::to_gsm_byte_frame`] — the de-facto 33-byte
//!   byte-frame used by raw `.gsm` files in the wild. This layout is
//!   not in the staged spec; it was derived **empirically** by
//!   black-box comparison against validator binaries (encode known
//!   PCM with an independent encoder binary, parse the produced
//!   bytes under candidate layouts, and require a parameter-for-
//!   parameter match with this crate's §5.2 encoder over thousands
//!   of frames — the match is exact, see `tests/blackbox_fixtures.rs`).
//!   Layout: a constant `0xD` marker nibble in the high nibble of
//!   byte 0, then the 76 Table 1.1 parameters in order of
//!   occurrence, each packed **MSB-first within its field** (unlike
//!   the spec's in-band LSB-first-within-field convention), bits
//!   filled MSB-first through the bytes; 4 + 260 = 264 bits fill the
//!   33 bytes exactly, so there is no spare nibble.

use crate::error::Error;

/// Speech-frame length in PCM samples (§1.5 — 20 ms at 8 kHz).
pub const FRAME_SAMPLES: usize = 160;

/// Speech-frame length in coded bits (§1.7 Table 1.1).
pub const FRAME_BITS: usize = 260;

/// Number of sub-frames per speech frame (§3.1.12).
pub const SUBFRAMES: usize = 4;

/// Number of RPE pulses per sub-frame (§3.1.20).
pub const PULSES: usize = 13;

/// Number of short-term residual samples per sub-frame (§3.1.12 —
/// `kj = k0 + j*40` with `k = 0,..,39`).
pub const SUBFRAME_SAMPLES: usize = 40;

/// One of the four 56-bit sub-frame parameter blocks per §1.7
/// Table 1.1 (rows 9..76).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct SubFrame {
    /// Coded LTP lag `Ncj` (§1.7 Table 1.1 col "Var. name" `N1..N4`).
    /// 7 bits. Valid (post-limit-check) range per §5.3.2 is 40..=120.
    pub n_c: u8,
    /// Coded LTP gain `bcj` (`b1..b4`). 2 bits.
    pub b_c: u8,
    /// RPE grid position `Mcj` (`M1..M4`). 2 bits.
    pub m_c: u8,
    /// Coded block amplitude `xmaxcj` (`Xmax1..Xmax4`). 6 bits.
    pub xmax_c: u8,
    /// Thirteen 3-bit normalised RPE-pulse codes `xMc[0..=12]`.
    pub x_mc: [u8; PULSES],
}

/// One parsed 260-bit speech frame, ready to feed §5.3 decoder.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct UnpackedFrame {
    /// Coded Log.-Area Ratios `LARc[1..=8]` (entry 0 is a sentinel
    /// zero so the array indexing matches the spec's 1-based
    /// notation throughout §5).
    pub lar_c: [i16; 9],
    /// Four 56-bit sub-frame parameter blocks (§3.1.12).
    pub sub: [SubFrame; SUBFRAMES],
}

impl UnpackedFrame {
    /// Decode one 260-bit speech frame from a buffer holding bits
    /// `b1..b260` packed MSB-first (i.e. bit `b1` is the most
    /// significant bit of `bytes[0]`, bit `b8` the least
    /// significant bit of `bytes[0]`, bit `b9` the MSB of
    /// `bytes[1]`, …).
    ///
    /// Returns [`Error::ShortFrame`] when `bytes.len() < 33`.
    ///
    /// Each parameter's own bit order is "(LSB-MSB)" per Table 1.1:
    /// the parameter's least significant bit is the bit with the
    /// LOWER `b`-index. So if a 6-bit parameter occupies bits
    /// b1..b6, then b1 is its LSB and b6 is its MSB.
    pub fn from_bit_stream_msb_first(bytes: &[u8]) -> Result<Self, Error> {
        if bytes.len() < 33 {
            return Err(Error::ShortFrame);
        }
        let mut r = BitReader::new(bytes);
        let mut f = UnpackedFrame::default();

        // ---- LAR parameters, b1..b36 (Table 1.1 rows 1..8) ----
        // Widths 6,6,5,5,4,4,3,3. Each parameter is read LSB-first.
        // The wire form is unsigned (the encoder's §5.2.7 step
        // offsets LARc by MIC[i] to keep it non-negative); §5.2.8
        // re-adds MIC[i] to restore the sign, so we just store the
        // raw value.
        let lar_widths: [u8; 8] = [6, 6, 5, 5, 4, 4, 3, 3];
        for (i, &w) in lar_widths.iter().enumerate() {
            f.lar_c[i + 1] = r.read_lsb_first(w) as i16;
        }

        // ---- Four sub-frames (Table 1.1 rows 9..76) ----
        for sf in f.sub.iter_mut() {
            sf.n_c = r.read_lsb_first(7) as u8;
            sf.b_c = r.read_lsb_first(2) as u8;
            sf.m_c = r.read_lsb_first(2) as u8;
            sf.xmax_c = r.read_lsb_first(6) as u8;
            for slot in sf.x_mc.iter_mut() {
                *slot = r.read_lsb_first(3) as u8;
            }
        }

        Ok(f)
    }

    /// Pack this frame's 76 parameters into the 260-bit stream
    /// `b1..b260`, returned as 33 bytes with `b1` in the most
    /// significant bit of byte 0 — the exact mirror of
    /// [`Self::from_bit_stream_msb_first`].
    ///
    /// Bit order follows §1.7 Table 1.1 verbatim: parameters are
    /// emitted in order of occurrence (eight LARc, then four
    /// 56-bit sub-frames of `Nc`, `bc`, `Mc`, `xmaxc`,
    /// `xMc[0..=12]`), and within each parameter the bit with the
    /// LOWER `b`-index is the parameter's LSB (the table's
    /// "(LSB-MSB)" column).
    ///
    /// Since 33 bytes hold 264 bits and the spec only defines
    /// `b1..b260`, the trailing 4 bits of byte 32 are left zero —
    /// EN 300 961 does not assign them (per §6.1 only the abstract
    /// 260-bit stream is contractual; any per-container use of the
    /// spare nibble is that container's business).
    ///
    /// Each parameter value is masked to its Table 1.1 field width
    /// before emission, so out-of-range struct values cannot bleed
    /// into neighbouring parameters; a debug assertion flags such
    /// values in debug builds (the §5.2 encoder never produces
    /// them).
    pub fn to_bit_stream_msb_first(&self) -> [u8; 33] {
        let mut w = BitWriter::new();

        // ---- LAR parameters, b1..b36 (Table 1.1 rows 1..8) ----
        let lar_widths: [u8; 8] = [6, 6, 5, 5, 4, 4, 3, 3];
        for (i, &width) in lar_widths.iter().enumerate() {
            w.write_lsb_first(self.lar_c[i + 1] as u32, width);
        }

        // ---- Four sub-frames (Table 1.1 rows 9..76) ----
        for sf in &self.sub {
            w.write_lsb_first(sf.n_c as u32, 7);
            w.write_lsb_first(sf.b_c as u32, 2);
            w.write_lsb_first(sf.m_c as u32, 2);
            w.write_lsb_first(sf.xmax_c as u32, 6);
            for &pulse in &sf.x_mc {
                w.write_lsb_first(pulse as u32, 3);
            }
        }

        debug_assert_eq!(w.bit_pos, FRAME_BITS);
        w.bytes
    }

    /// Decode one frame from the de-facto 33-byte `.gsm` byte-frame
    /// (see the module docs): a `0xD` marker nibble in the high
    /// nibble of byte 0, then the 76 Table 1.1 parameters in order,
    /// each packed MSB-first within its field.
    ///
    /// Returns [`Error::ShortFrame`] when `bytes.len() < 33` and
    /// [`Error::BadByteFrameMagic`] when the marker nibble is not
    /// `0xD`.
    ///
    /// This layout is **not** part of EN 300 961 — it was derived
    /// empirically from validator binaries treated as black boxes
    /// and is pinned byte-for-byte by the checked-in fixtures in
    /// `tests/blackbox_fixtures.rs`.
    pub fn from_gsm_byte_frame(bytes: &[u8]) -> Result<Self, Error> {
        if bytes.len() < GSM_BYTE_FRAME_LEN {
            return Err(Error::ShortFrame);
        }
        let mut r = BitReader::new(bytes);
        if r.read_msb_first(4) != GSM_BYTE_FRAME_MAGIC as u32 {
            return Err(Error::BadByteFrameMagic);
        }

        let mut f = UnpackedFrame::default();
        let lar_widths: [u8; 8] = [6, 6, 5, 5, 4, 4, 3, 3];
        for (i, &w) in lar_widths.iter().enumerate() {
            f.lar_c[i + 1] = r.read_msb_first(w) as i16;
        }
        for sf in f.sub.iter_mut() {
            sf.n_c = r.read_msb_first(7) as u8;
            sf.b_c = r.read_msb_first(2) as u8;
            sf.m_c = r.read_msb_first(2) as u8;
            sf.xmax_c = r.read_msb_first(6) as u8;
            for slot in sf.x_mc.iter_mut() {
                *slot = r.read_msb_first(3) as u8;
            }
        }
        Ok(f)
    }

    /// Pack this frame into the de-facto 33-byte `.gsm` byte-frame —
    /// the exact mirror of [`Self::from_gsm_byte_frame`]: marker
    /// nibble `0xD`, then the 76 Table 1.1 parameters in order, each
    /// MSB-first within its field. 4 + 260 bits fill the 33 bytes
    /// exactly.
    ///
    /// Each parameter value is masked to its Table 1.1 field width
    /// before emission (with a debug assertion on overflow), matching
    /// [`Self::to_bit_stream_msb_first`].
    pub fn to_gsm_byte_frame(&self) -> [u8; GSM_BYTE_FRAME_LEN] {
        let mut w = BitWriter::new();
        w.write_msb_first(GSM_BYTE_FRAME_MAGIC as u32, 4);

        let lar_widths: [u8; 8] = [6, 6, 5, 5, 4, 4, 3, 3];
        for (i, &width) in lar_widths.iter().enumerate() {
            w.write_msb_first(self.lar_c[i + 1] as u32, width);
        }
        for sf in &self.sub {
            w.write_msb_first(sf.n_c as u32, 7);
            w.write_msb_first(sf.b_c as u32, 2);
            w.write_msb_first(sf.m_c as u32, 2);
            w.write_msb_first(sf.xmax_c as u32, 6);
            for &pulse in &sf.x_mc {
                w.write_msb_first(pulse as u32, 3);
            }
        }

        debug_assert_eq!(w.bit_pos, GSM_BYTE_FRAME_LEN * 8);
        w.bytes
    }
}

/// Length in bytes of the de-facto `.gsm` byte-frame (and of the
/// spec's own 260-bit frame rounded up): 33.
pub const GSM_BYTE_FRAME_LEN: usize = 33;

/// Marker nibble carried in the high nibble of byte 0 of every
/// de-facto `.gsm` byte-frame.
pub const GSM_BYTE_FRAME_MAGIC: u8 = 0xD;

/// Bit-stream reader over a byte buffer holding `b1..bN` packed
/// MSB-first (`b1` is the most-significant bit of `bytes[0]`).
///
/// Each `read_lsb_first(n)` call emits an n-bit field in the
/// parameter's own LSB-first order, matching the "(LSB-MSB)" column
/// of §1.7 Table 1.1.
struct BitReader<'a> {
    bytes: &'a [u8],
    /// Index of the next bit to emit, counted as a position in the
    /// `b1..bN` stream (zero-indexed: `bit_pos = 0` means the next
    /// bit out is `b1`).
    bit_pos: usize,
}

impl<'a> BitReader<'a> {
    fn new(bytes: &'a [u8]) -> Self {
        Self { bytes, bit_pos: 0 }
    }

    /// Read `n` bits in the parameter's own LSB-first order
    /// (i.e. the first bit emitted is the LSB of the result), as
    /// specified by Table 1.1's "(LSB-MSB)" column.
    fn read_lsb_first(&mut self, n: u8) -> u32 {
        debug_assert!(n <= 16);
        let mut out: u32 = 0;
        for i in 0..n {
            out |= (self.pull_bit() as u32) << i;
        }
        out
    }

    /// Read `n` bits in the parameter's MSB-first order (the first
    /// bit pulled is the MSB of the result) — the de-facto `.gsm`
    /// byte-frame field convention.
    fn read_msb_first(&mut self, n: u8) -> u32 {
        debug_assert!(n <= 16);
        let mut out: u32 = 0;
        for _ in 0..n {
            out = (out << 1) | self.pull_bit() as u32;
        }
        out
    }

    /// Pull one bit MSB-first off the byte stream.
    #[inline]
    fn pull_bit(&mut self) -> u8 {
        let byte_idx = self.bit_pos / 8;
        let bit_in_byte = self.bit_pos % 8;
        self.bit_pos += 1;
        if byte_idx < self.bytes.len() {
            (self.bytes[byte_idx] >> (7 - bit_in_byte)) & 1
        } else {
            0
        }
    }
}

/// Bit-stream writer mirroring [`BitReader`]: emits `b1..bN`
/// packed MSB-first into a 33-byte frame buffer (`b1` is the
/// most-significant bit of `bytes[0]`).
struct BitWriter {
    bytes: [u8; 33],
    /// Index of the next bit to write, as a zero-indexed position
    /// in the `b1..bN` stream (mirrors [`BitReader::bit_pos`]).
    bit_pos: usize,
}

impl BitWriter {
    fn new() -> Self {
        Self {
            bytes: [0u8; 33],
            bit_pos: 0,
        }
    }

    /// Write the low `n` bits of `v` in the parameter's own
    /// LSB-first order (i.e. the first bit pushed onto the stream
    /// is the LSB of `v`), matching Table 1.1's "(LSB-MSB)"
    /// column. Bits of `v` above `n` are masked off (and trip a
    /// debug assertion — the §5.2 codeword ranges never set them).
    fn write_lsb_first(&mut self, v: u32, n: u8) {
        debug_assert!(n <= 16);
        debug_assert_eq!(v >> n, 0, "codeword exceeds its Table 1.1 field width");
        for i in 0..n {
            self.push_bit(((v >> i) & 1) as u8);
        }
    }

    /// Write the low `n` bits of `v` in the parameter's MSB-first
    /// order (the first bit pushed is the MSB of `v`'s n-bit field)
    /// — the de-facto `.gsm` byte-frame field convention. Bits of
    /// `v` above `n` are masked off (debug-asserted, mirroring
    /// [`Self::write_lsb_first`]).
    fn write_msb_first(&mut self, v: u32, n: u8) {
        debug_assert!(n <= 16);
        debug_assert_eq!(v >> n, 0, "codeword exceeds its field width");
        for i in (0..n).rev() {
            self.push_bit(((v >> i) & 1) as u8);
        }
    }

    /// Push one bit MSB-first onto the byte stream.
    #[inline]
    fn push_bit(&mut self, bit: u8) {
        let byte_idx = self.bit_pos / 8;
        let bit_in_byte = self.bit_pos % 8;
        self.bit_pos += 1;
        if bit & 1 != 0 {
            self.bytes[byte_idx] |= 1 << (7 - bit_in_byte);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Empty / undersized buffer must yield ShortFrame.
    #[test]
    fn short_frame_rejected() {
        assert!(matches!(
            UnpackedFrame::from_bit_stream_msb_first(&[0u8; 32]),
            Err(Error::ShortFrame)
        ));
        assert!(matches!(
            UnpackedFrame::from_bit_stream_msb_first(&[]),
            Err(Error::ShortFrame)
        ));
    }

    /// All-zero frame unpacks to an all-zero parameter struct.
    #[test]
    fn all_zero_frame_unpacks_to_default() {
        let f = UnpackedFrame::from_bit_stream_msb_first(&[0u8; 33]).unwrap();
        for i in 1..=8 {
            assert_eq!(f.lar_c[i], 0, "LARc[{i}]");
        }
        for s in 0..SUBFRAMES {
            assert_eq!(f.sub[s].n_c, 0);
            assert_eq!(f.sub[s].b_c, 0);
            assert_eq!(f.sub[s].m_c, 0);
            assert_eq!(f.sub[s].xmax_c, 0);
            for j in 0..PULSES {
                assert_eq!(f.sub[s].x_mc[j], 0);
            }
        }
    }

    /// Tease apart Table 1.1's bit allocations by walking a
    /// "one bit set at a time" probe through the stream and
    /// checking which parameter field reacts. b1 is the LSB of
    /// LARc[1] (a 6-bit field) — and b1 = MSB of byte 0.
    #[test]
    fn b1_is_lar1_lsb() {
        let mut bytes = [0u8; 33];
        bytes[0] = 0b1000_0000; // set b1 (= byte0 MSB) only
        let f = UnpackedFrame::from_bit_stream_msb_first(&bytes).unwrap();
        assert_eq!(f.lar_c[1], 1, "b1 should land in LARc[1] bit 0");
        assert_eq!(f.lar_c[2], 0);
    }

    /// b6 is the MSB of LARc[1] (6-bit field). After b1 the
    /// stream's b6 lives at bit-in-byte 5 of byte 0.
    #[test]
    fn b6_is_lar1_msb() {
        let mut bytes = [0u8; 33];
        bytes[0] = 0b0000_0100; // set b6 only (bit-in-byte 5)
        let f = UnpackedFrame::from_bit_stream_msb_first(&bytes).unwrap();
        assert_eq!(f.lar_c[1], 1 << 5, "b6 should land in LARc[1] bit 5");
    }

    /// Right at the end of LAR1: bits b1..b6 all one ⇒ LARc[1] =
    /// 0b111111 = 63. That matches the 6-bit field width.
    #[test]
    fn lar1_full_ones_fills_six_bits() {
        let mut bytes = [0u8; 33];
        bytes[0] = 0b1111_1100; // b1..b6 = 1
        let f = UnpackedFrame::from_bit_stream_msb_first(&bytes).unwrap();
        assert_eq!(f.lar_c[1], 0b11_1111);
        assert_eq!(f.lar_c[2], 0); // b7..b12 still zero
    }

    /// b7..b12 form LARc[2]. Setting only b12 leaves LARc[2] =
    /// 0b10_0000 = 32 (its MSB).
    #[test]
    fn b12_is_lar2_msb() {
        let mut bytes = [0u8; 33];
        // b12 sits at bit-in-byte (12-1)%8 = 3 of byte 1 ((12-1)/8 = 1).
        bytes[1] = 0b0001_0000; // bit-in-byte 3
        let f = UnpackedFrame::from_bit_stream_msb_first(&bytes).unwrap();
        assert_eq!(f.lar_c[2], 1 << 5);
        assert_eq!(f.lar_c[1], 0);
    }

    /// Sub-frame 1 starts at b37 (= byte 4, bit-in-byte 4). N1 is
    /// 7 bits wide ⇒ setting just b37 puts a 1 in `sub[0].n_c`'s
    /// LSB.
    #[test]
    fn b37_is_n1_lsb() {
        let mut bytes = [0u8; 33];
        // b37: byte idx (37-1)/8 = 4, bit-in-byte (37-1)%8 = 4
        bytes[4] = 0b0000_1000; // (7 - 4) = 3
        let f = UnpackedFrame::from_bit_stream_msb_first(&bytes).unwrap();
        assert_eq!(f.sub[0].n_c, 1);
    }

    // ─── §1.7 packer (to_bit_stream_msb_first) ───

    /// Build a fully-populated frame with a distinct, in-range value
    /// in every one of the 76 parameter slots so a packing slip in
    /// any field shows up as a roundtrip mismatch.
    fn patterned_frame() -> UnpackedFrame {
        let mut f = UnpackedFrame::default();
        // LARc widths 6,6,5,5,4,4,3,3 ⇒ max 63,63,31,31,15,15,7,7.
        let lar_max: [i16; 8] = [63, 63, 31, 31, 15, 15, 7, 7];
        for i in 1..=8 {
            // Distinct values, each below its field max.
            f.lar_c[i] = (lar_max[i - 1] - (i as i16 - 1)).max(1);
        }
        for (s, sf) in f.sub.iter_mut().enumerate() {
            let s8 = s as u8;
            sf.n_c = 40 + 17 * s8; // §5.3.2 valid lag range 40..=120
            sf.b_c = s8 & 0b11;
            sf.m_c = (3 - s8) & 0b11;
            sf.xmax_c = 5 + 13 * s8; // ≤ 44 < 63
            for (i, slot) in sf.x_mc.iter_mut().enumerate() {
                *slot = ((i as u8) + s8) % 8; // 3-bit range
            }
        }
        f
    }

    /// pack ∘ unpack = identity on a frame exercising every field.
    #[test]
    fn pack_unpack_roundtrip_patterned() {
        let f = patterned_frame();
        let bytes = f.to_bit_stream_msb_first();
        let g = UnpackedFrame::from_bit_stream_msb_first(&bytes).unwrap();
        assert_eq!(f, g);
    }

    /// unpack ∘ pack = identity on the byte level: take an arbitrary
    /// 33-byte buffer whose trailing 4 bits (b261..b264, outside the
    /// §1.7 frame) are zero, unpack it, re-pack it, and require the
    /// identical bytes back. Uses a deterministic LCG fill so all
    /// 260 payload bits get exercised across the 8 iterations.
    #[test]
    fn unpack_pack_roundtrip_bytes() {
        let mut seed: u32 = 0x1234_5678;
        for round in 0..8 {
            let mut bytes = [0u8; 33];
            for b in bytes.iter_mut() {
                // Numerical-recipes-style LCG; byte from the high bits.
                seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                *b = (seed >> 24) as u8;
            }
            bytes[32] &= 0xF0; // clear b261..b264 (not part of the frame)
            let f = UnpackedFrame::from_bit_stream_msb_first(&bytes).unwrap();
            let repacked = f.to_bit_stream_msb_first();
            assert_eq!(bytes, repacked, "byte roundtrip failed on round {round}");
        }
    }

    /// All-zero frame packs to all-zero bytes (and the spare nibble
    /// b261..b264 stays zero).
    #[test]
    fn pack_all_zero_frame() {
        let bytes = UnpackedFrame::default().to_bit_stream_msb_first();
        assert_eq!(bytes, [0u8; 33]);
    }

    /// Mirror of `b1_is_lar1_lsb`: LARc[1] = 1 must set exactly b1
    /// (the MSB of byte 0).
    #[test]
    fn pack_lar1_lsb_lands_on_b1() {
        let mut f = UnpackedFrame::default();
        f.lar_c[1] = 1;
        let bytes = f.to_bit_stream_msb_first();
        assert_eq!(bytes[0], 0b1000_0000);
        for &b in &bytes[1..] {
            assert_eq!(b, 0);
        }
    }

    /// Mirror of `b37_is_n1_lsb`: sub[0].n_c = 1 must set exactly
    /// b37 (byte 4, bit-in-byte 4).
    #[test]
    fn pack_n1_lsb_lands_on_b37() {
        let mut f = UnpackedFrame::default();
        f.sub[0].n_c = 1;
        let bytes = f.to_bit_stream_msb_first();
        assert_eq!(bytes[4], 0b0000_1000);
        for (i, &b) in bytes.iter().enumerate() {
            if i != 4 {
                assert_eq!(b, 0, "byte {i} should be untouched");
            }
        }
    }

    /// The last frame parameter — sub[3].x_mc[12], occupying
    /// b258..b260 — must land in the high nibble of byte 32, with
    /// the spare low nibble (b261..b264) zero. b258 is its LSB:
    /// byte (258-1)/8 = 32, bit-in-byte (258-1)%8 = 1.
    #[test]
    fn pack_final_pulse_lands_in_high_nibble_of_byte_32() {
        let mut f = UnpackedFrame::default();
        f.sub[3].x_mc[12] = 0b111;
        let bytes = f.to_bit_stream_msb_first();
        // b258 (bit-in-byte 1), b259 (2), b260 (3) all set.
        assert_eq!(bytes[32], 0b0111_0000);
        assert_eq!(bytes[32] & 0x0F, 0, "spare nibble must stay zero");
    }

    // ─── de-facto `.gsm` byte-frame ───

    /// An all-zero frame packs to the marker nibble and nothing else.
    #[test]
    fn byte_frame_all_zero_is_magic_only() {
        let bytes = UnpackedFrame::default().to_gsm_byte_frame();
        assert_eq!(bytes[0], 0xD0);
        for &b in &bytes[1..] {
            assert_eq!(b, 0);
        }
    }

    /// Field bit-order pin: LARc[1] = 1 occupies the 6-bit field at
    /// bit positions 4..=9 MSB-first, so its LSB (the only set bit)
    /// lands at absolute bit 9 = byte 1, bit-in-byte 1.
    #[test]
    fn byte_frame_fields_are_msb_first() {
        let mut f = UnpackedFrame::default();
        f.lar_c[1] = 1;
        let bytes = f.to_gsm_byte_frame();
        assert_eq!(bytes[0], 0xD0);
        assert_eq!(bytes[1], 0b0100_0000);
        for &b in &bytes[2..] {
            assert_eq!(b, 0);
        }

        // And the MSB of the same field lands right after the magic:
        // absolute bit 4 = byte 0, bit-in-byte 4.
        let mut f = UnpackedFrame::default();
        f.lar_c[1] = 0b10_0000;
        let bytes = f.to_gsm_byte_frame();
        assert_eq!(bytes[0], 0xD8);
        for &b in &bytes[1..] {
            assert_eq!(b, 0);
        }
    }

    /// pack ∘ unpack = identity on a frame exercising every field.
    #[test]
    fn byte_frame_roundtrip_patterned() {
        let f = patterned_frame();
        let bytes = f.to_gsm_byte_frame();
        let g = UnpackedFrame::from_gsm_byte_frame(&bytes).unwrap();
        assert_eq!(f, g);
    }

    /// The byte-frame and the in-band 260-bit stream carry the same
    /// 76 parameters: packing the same frame both ways and unpacking
    /// each with its own reader yields equal structs.
    #[test]
    fn byte_frame_equivalent_to_in_band_stream() {
        let f = patterned_frame();
        let a = UnpackedFrame::from_gsm_byte_frame(&f.to_gsm_byte_frame()).unwrap();
        let b = UnpackedFrame::from_bit_stream_msb_first(&f.to_bit_stream_msb_first()).unwrap();
        assert_eq!(a, b);
    }

    /// Marker-nibble and length enforcement.
    #[test]
    fn byte_frame_rejects_bad_magic_and_short_input() {
        let f = patterned_frame();
        let mut bytes = f.to_gsm_byte_frame();
        bytes[0] = (bytes[0] & 0x0F) | 0xC0; // corrupt the marker
        assert!(matches!(
            UnpackedFrame::from_gsm_byte_frame(&bytes),
            Err(Error::BadByteFrameMagic)
        ));
        assert!(matches!(
            UnpackedFrame::from_gsm_byte_frame(&[0xD0; 32]),
            Err(Error::ShortFrame)
        ));
    }

    /// unpack ∘ pack = byte identity over LCG-random byte-frames
    /// carrying the 0xD marker (all 260 payload bits exercised).
    #[test]
    fn byte_frame_unpack_pack_roundtrip_bytes() {
        let mut seed: u32 = 0x0BAD_F00D;
        for round in 0..8 {
            let mut bytes = [0u8; GSM_BYTE_FRAME_LEN];
            for b in bytes.iter_mut() {
                seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                *b = (seed >> 24) as u8;
            }
            bytes[0] = (bytes[0] & 0x0F) | 0xD0;
            let f = UnpackedFrame::from_gsm_byte_frame(&bytes).unwrap();
            let repacked = f.to_gsm_byte_frame();
            assert_eq!(bytes, repacked, "byte roundtrip failed on round {round}");
        }
    }
}
