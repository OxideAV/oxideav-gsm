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
//! into bytes (the `.gsm` 33-byte byte format, RTP payload type 3,
//! MS-GSM WAV chunk, TRAU 320-bit, …) is **outside** the scope of
//! EN 300 961. The on-wire byte layout for those specific container
//! forms is NOT specified in the staged PDF and is reported as a
//! docs gap.
//!
//! This module therefore exposes two layers:
//!
//! * [`UnpackedFrame`] — the typed structure of 76 parameters that
//!   §5.3 decoder consumes directly.
//! * [`UnpackedFrame::from_bit_stream_msb_first`] — a 260-bit stream
//!   unpacker whose only assumption is "stream is `b1..b260` packed
//!   MSB-first into a `[u8; 33]`-or-larger byte buffer." That is the
//!   widely-used in-band convention shared by every container the
//!   spec lists in §6.1 (the bit positions match Table 1.1 verbatim);
//!   only the per-container framing wrapper outside those 260 bits
//!   varies, and that wrapper is the docs gap above.

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
}

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
}
