//! GSM 06.20 half-rate speech codec (VSELP) — ETSI EN 300 969.
//!
//! **Status: foundation.** This module family begins the half-rate
//! arc with the normative frame layer and the codec's ROM data:
//!
//! * [`tables`] — the bit-exact ROM tables (reflection-coefficient
//!   VQ/prequantizer/scalar-dequantizer, R0 decode, allowable lags,
//!   interpolating filters, GSP0 gain VQ, sqrt(P0), VSELP basis
//!   vectors, high-pass/SST/noise-weighting/soft-interpolation
//!   coefficients), transcribed from the staged data extracts;
//! * [`HrParameters`] — the 18 codec parameters of a 20 ms frame
//!   (annex A table A.1), with the MODE-dependent subframe split
//!   ([`SubframeParams`]);
//! * annex B bit packing — [`HrParameters::to_bits`] /
//!   [`HrParameters::from_bits`]: the normative `b1..b112` order of
//!   occurrence over the Abis for both the unvoiced (table B.1) and
//!   voiced (table B.2) layouts, packed MSB-first (`b1` = MSB of
//!   byte 0, 14 bytes);
//! * conformance word I/O — [`HrParameters::from_cod_words`] /
//!   [`HrParameters::to_cod_words`]: the 18-parameter word layout the
//!   GSM 06.07 test sequences use (one right-justified parameter per
//!   16-bit word, in table A.1 order), as carried inside the `*.COD`
//!   (18 + 2 flag words) and `*.DEC` (18 + 4 flag words) frames.
//!
//! The decode chain (excitation generation, adaptive pitch prefilter,
//! synthesis filter, adaptive spectral postfilter — clauses 4.2.x)
//! and the encoder are follow-up work; the GSM 06.07 corpus staged
//! under `tests/fixtures/etsi-hr/` provides the bit-exact references
//! for both directions.
//!
//! Basic coder parameters (annex A.2): 8 kHz sampling, `NF` = 160
//! samples (20 ms) per frame, `Ns` = 40 samples (5 ms) per subframe,
//! short-term predictor order `Np` = 10, data rate 5,6 kbit/s =
//! [`HR_FRAME_BITS`] bits per frame.

pub mod decode;
pub mod tables;

pub use decode::{
    hr_decoder_homing_frame, is_hr_decoder_homing_frame, HrDecoder, HR_DECODER_HOMING_WORDS,
    HR_ENCODER_HOMING_SAMPLE,
};

use crate::error::Error;

/// Number of bits per 20 ms half-rate frame (annex A.1: *"each 20 ms
/// speech frame consists of 112 bits"*).
pub const HR_FRAME_BITS: usize = 112;

/// Number of bytes holding one packed `b1..b112` frame.
pub const HR_FRAME_BYTES: usize = HR_FRAME_BITS / 8;

/// Samples per half-rate frame (annex A.2 `NF`).
pub const HR_FRAME_SAMPLES: usize = 160;

/// Samples per subframe (annex A.2 `Ns`).
pub const HR_SUBFRAME_SAMPLES: usize = 40;

/// Subframes per frame.
pub const HR_SUBFRAMES: usize = 4;

/// Number of coded parameters per frame (annex A.1: *"the encoder
/// derives 18 parameters"* — 6 frame parameters + 12 subframe
/// parameters in either MODE split).
pub const HR_PARAMS_PER_FRAME: usize = 18;

/// The four voicing modes (annex A.1.6).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VoicingMode {
    /// MODE = 0 — unvoiced: the adaptive codebook (long-term
    /// predictor) and the VSELP codebook are replaced by two other
    /// VSELP codebooks.
    Unvoiced = 0,
    /// MODE = 1 — slightly voiced.
    SlightlyVoiced = 1,
    /// MODE = 2 — moderately voiced.
    ModeratelyVoiced = 2,
    /// MODE = 3 — strongly voiced.
    StronglyVoiced = 3,
}

impl VoicingMode {
    /// Build from the 2-bit MODE code.
    pub fn from_code(code: u8) -> Self {
        match code & 0b11 {
            0 => VoicingMode::Unvoiced,
            1 => VoicingMode::SlightlyVoiced,
            2 => VoicingMode::ModeratelyVoiced,
            _ => VoicingMode::StronglyVoiced,
        }
    }

    /// The 2-bit MODE code.
    pub fn code(self) -> u8 {
        self as u8
    }

    /// `true` for MODE 1..3 (the voiced layouts of table B.2).
    pub fn is_voiced(self) -> bool {
        !matches!(self, VoicingMode::Unvoiced)
    }
}

/// The MODE-dependent subframe parameters (annex A table A.1).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SubframeParams {
    /// MODE = 0: per subframe, two sequentially-searched VSELP
    /// codewords `I` (`CODE1_x`, 7 bits) and `H` (`CODE2_x`, 7 bits)
    /// plus the 5-bit `{P0,GS}` gain code `GSP0_x`.
    Unvoiced {
        /// `CODE1_1..CODE1_4` — first-codebook codeword `I`.
        code1: [u8; HR_SUBFRAMES],
        /// `CODE2_1..CODE2_4` — second-codebook codeword `H`.
        code2: [u8; HR_SUBFRAMES],
        /// `GSP0_1..GSP0_4` — `{P0,GS}` gain codes.
        gsp0: [u8; HR_SUBFRAMES],
    },
    /// MODE = 1..3: per subframe, the LTP lag (8-bit code for the
    /// first subframe, 4-bit delta codes after — annex A.1.4), the
    /// 9-bit VSELP codeword and the 5-bit gain code.
    Voiced {
        /// `LAG_1` — 8-bit lag code for the first subframe (lag range
        /// 21..142, possibly fractional).
        lag1: u8,
        /// `LAG_2..LAG_4` — 4-bit delta codes relative to the
        /// preceding subframe's coded lag (−8..+7 allowable-lag
        /// levels).
        lag_delta: [u8; 3],
        /// `CODE_1..CODE_4` — 9-bit VSELP codewords `I`.
        code: [u16; HR_SUBFRAMES],
        /// `GSP0_1..GSP0_4` — `{P0,GS}` gain codes.
        gsp0: [u8; HR_SUBFRAMES],
    },
}

/// The 18 codec parameters of one 20 ms half-rate frame (annex A
/// table A.1).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HrParameters {
    /// `R0` — 5-bit frame-energy code.
    pub r0: u8,
    /// `LPC1` — 11-bit reflection-coefficient vector index (r1..r3).
    pub lpc1: u16,
    /// `LPC2` — 9-bit reflection-coefficient vector index (r4..r6).
    pub lpc2: u16,
    /// `LPC3` — 8-bit reflection-coefficient vector index (r7..r10).
    pub lpc3: u8,
    /// `INT_LPC` — the soft-interpolation bit (clause 4.1.6).
    pub int_lpc: bool,
    /// The raw 2-bit MODE code (annex A.1.1). 0 pairs with
    /// [`SubframeParams::Unvoiced`]; 1..3 share the voiced layout,
    /// so the code is carried explicitly to survive round-trips.
    pub mode_code: u8,
    /// The MODE-dependent subframe parameters.
    pub sub: SubframeParams,
}

impl HrParameters {
    /// The frame's voicing mode.
    pub fn mode(&self) -> VoicingMode {
        match self.sub {
            SubframeParams::Unvoiced { .. } => VoicingMode::Unvoiced,
            SubframeParams::Voiced { .. } => {
                // The 2-bit code is carried separately in the coded
                // frame; `mode_code` preserves it.
                VoicingMode::from_code(self.mode_code)
            }
        }
    }
}

/// Internal: bit-writer over the `b1..b112` stream (b1 = MSB of
/// byte 0), per the FR convention of §1.7 applied to annex B.
struct BitWriter {
    bytes: [u8; HR_FRAME_BYTES],
    pos: usize,
}

impl BitWriter {
    fn new() -> Self {
        Self {
            bytes: [0; HR_FRAME_BYTES],
            pos: 0,
        }
    }
    /// Write `width` bits of `value`, MSB first (annex B lists every
    /// field "(MSB - LSB)").
    fn put(&mut self, value: u16, width: usize) {
        for k in (0..width).rev() {
            let bit = (value >> k) & 1;
            let byte = self.pos / 8;
            let off = 7 - (self.pos % 8);
            self.bytes[byte] |= (bit as u8) << off;
            self.pos += 1;
        }
    }
}

/// Internal: bit-reader over the `b1..b112` stream.
struct BitReader<'a> {
    bytes: &'a [u8],
    pos: usize,
}

impl<'a> BitReader<'a> {
    fn get(&mut self, width: usize) -> u16 {
        let mut v = 0u16;
        for _ in 0..width {
            let byte = self.pos / 8;
            let off = 7 - (self.pos % 8);
            v = (v << 1) | (((self.bytes[byte] >> off) & 1) as u16);
            self.pos += 1;
        }
        v
    }
}

impl HrParameters {
    /// Pack into the annex B `b1..b112` frame (14 bytes, `b1` = MSB
    /// of byte 0). Both table B.1 (unvoiced) and table B.2 (voiced)
    /// share the frame-bit prefix `R0 LPC1 LPC2 LPC3 INT_LPC MODE`
    /// (b1..b36); the subframe fields follow per MODE.
    pub fn to_bits(&self) -> [u8; HR_FRAME_BYTES] {
        let mut w = BitWriter::new();
        w.put(self.r0 as u16, 5);
        w.put(self.lpc1, 11);
        w.put(self.lpc2, 9);
        w.put(self.lpc3 as u16, 8);
        w.put(self.int_lpc as u16, 1);
        w.put(self.mode_code as u16, 2);
        match &self.sub {
            SubframeParams::Unvoiced { code1, code2, gsp0 } => {
                for s in 0..HR_SUBFRAMES {
                    w.put(code1[s] as u16, 7);
                    w.put(code2[s] as u16, 7);
                    w.put(gsp0[s] as u16, 5);
                }
            }
            SubframeParams::Voiced {
                lag1,
                lag_delta,
                code,
                gsp0,
            } => {
                for s in 0..HR_SUBFRAMES {
                    if s == 0 {
                        w.put(*lag1 as u16, 8);
                    } else {
                        w.put(lag_delta[s - 1] as u16, 4);
                    }
                    w.put(code[s], 9);
                    w.put(gsp0[s] as u16, 5);
                }
            }
        }
        debug_assert_eq!(w.pos, HR_FRAME_BITS);
        w.bytes
    }

    /// Parse an annex B `b1..b112` frame. Requires at least
    /// [`HR_FRAME_BYTES`] bytes.
    pub fn from_bits(bytes: &[u8]) -> Result<Self, Error> {
        if bytes.len() < HR_FRAME_BYTES {
            return Err(Error::ShortFrame);
        }
        let mut r = BitReader { bytes, pos: 0 };
        let r0 = r.get(5) as u8;
        let lpc1 = r.get(11);
        let lpc2 = r.get(9);
        let lpc3 = r.get(8) as u8;
        let int_lpc = r.get(1) != 0;
        let mode = r.get(2) as u8;
        let sub = if mode == 0 {
            let mut code1 = [0u8; HR_SUBFRAMES];
            let mut code2 = [0u8; HR_SUBFRAMES];
            let mut gsp0 = [0u8; HR_SUBFRAMES];
            for s in 0..HR_SUBFRAMES {
                code1[s] = r.get(7) as u8;
                code2[s] = r.get(7) as u8;
                gsp0[s] = r.get(5) as u8;
            }
            SubframeParams::Unvoiced { code1, code2, gsp0 }
        } else {
            let mut lag1 = 0u8;
            let mut lag_delta = [0u8; 3];
            let mut code = [0u16; HR_SUBFRAMES];
            let mut gsp0 = [0u8; HR_SUBFRAMES];
            for s in 0..HR_SUBFRAMES {
                if s == 0 {
                    lag1 = r.get(8) as u8;
                } else {
                    lag_delta[s - 1] = r.get(4) as u8;
                }
                code[s] = r.get(9);
                gsp0[s] = r.get(5) as u8;
            }
            SubframeParams::Voiced {
                lag1,
                lag_delta,
                code,
                gsp0,
            }
        };
        Ok(Self {
            r0,
            lpc1,
            lpc2,
            lpc3,
            int_lpc,
            mode_code: mode,
            sub,
        })
    }

    /// Flatten into the 18 conformance parameter words, in annex A
    /// table A.1 order: `R0 LPC1 LPC2 LPC3 INT_LPC MODE` then, per
    /// subframe, the MODE split — `(LAG_x | –) CODEx… GSP0_x`. This
    /// is the layout carried inside the GSM 06.07 `*.COD` / `*.DEC`
    /// word files (each parameter right-justified in its own 16-bit
    /// word).
    pub fn to_cod_words(&self) -> [u16; HR_PARAMS_PER_FRAME] {
        let mut w = [0u16; HR_PARAMS_PER_FRAME];
        w[0] = self.r0 as u16;
        w[1] = self.lpc1;
        w[2] = self.lpc2;
        w[3] = self.lpc3 as u16;
        w[4] = self.int_lpc as u16;
        w[5] = self.mode_code as u16;
        match &self.sub {
            SubframeParams::Unvoiced { code1, code2, gsp0 } => {
                for s in 0..HR_SUBFRAMES {
                    w[6 + 3 * s] = code1[s] as u16;
                    w[7 + 3 * s] = code2[s] as u16;
                    w[8 + 3 * s] = gsp0[s] as u16;
                }
            }
            SubframeParams::Voiced {
                lag1,
                lag_delta,
                code,
                gsp0,
            } => {
                for s in 0..HR_SUBFRAMES {
                    w[6 + 3 * s] = if s == 0 {
                        *lag1 as u16
                    } else {
                        lag_delta[s - 1] as u16
                    };
                    w[7 + 3 * s] = code[s];
                    w[8 + 3 * s] = gsp0[s] as u16;
                }
            }
        }
        w
    }

    /// Rebuild from the 18 conformance parameter words (see
    /// [`Self::to_cod_words`]). Each word is masked to its annex A
    /// field width.
    pub fn from_cod_words(words: &[u16; HR_PARAMS_PER_FRAME]) -> Self {
        let mode = (words[5] & 0b11) as u8;
        let sub = if mode == 0 {
            SubframeParams::Unvoiced {
                code1: core::array::from_fn(|s| (words[6 + 3 * s] & 0x7F) as u8),
                code2: core::array::from_fn(|s| (words[7 + 3 * s] & 0x7F) as u8),
                gsp0: core::array::from_fn(|s| (words[8 + 3 * s] & 0x1F) as u8),
            }
        } else {
            SubframeParams::Voiced {
                lag1: (words[6] & 0xFF) as u8,
                lag_delta: core::array::from_fn(|s| (words[9 + 3 * s] & 0x0F) as u8),
                code: core::array::from_fn(|s| words[7 + 3 * s] & 0x1FF),
                gsp0: core::array::from_fn(|s| (words[8 + 3 * s] & 0x1F) as u8),
            }
        };
        Self {
            r0: (words[0] & 0x1F) as u8,
            lpc1: words[1] & 0x7FF,
            lpc2: words[2] & 0x1FF,
            lpc3: (words[3] & 0xFF) as u8,
            int_lpc: words[4] & 1 != 0,
            mode_code: mode,
            sub,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::tables::*;
    use super::*;

    fn sample_voiced() -> HrParameters {
        HrParameters {
            r0: 21,
            lpc1: 0x5A5,
            lpc2: 0x123,
            lpc3: 0xC7,
            int_lpc: true,
            mode_code: 2,
            sub: SubframeParams::Voiced {
                lag1: 0xB4,
                lag_delta: [3, 15, 8],
                code: [0x1F0, 0x0A5, 0x155, 0x0FF],
                gsp0: [31, 0, 17, 9],
            },
        }
    }

    fn sample_unvoiced() -> HrParameters {
        HrParameters {
            r0: 5,
            lpc1: 881,
            lpc2: 350,
            lpc3: 195,
            int_lpc: true,
            mode_code: 0,
            sub: SubframeParams::Unvoiced {
                code1: [71, 9, 127, 0],
                code2: [74, 38, 0, 127],
                gsp0: [0, 7, 31, 16],
            },
        }
    }

    /// Annex A.1: 112 bits per frame; the field widths of both MODE
    /// layouts sum to exactly that.
    #[test]
    fn bit_budget_is_112() {
        let frame_bits = 5 + 11 + 9 + 8 + 1 + 2; // 36
        let voiced_sub = 8 + 4 * 3 + (9 + 5) * 4; // 76
        let unvoiced_sub = (7 + 7 + 5) * 4; // 76
        assert_eq!(frame_bits + voiced_sub, HR_FRAME_BITS);
        assert_eq!(frame_bits + unvoiced_sub, HR_FRAME_BITS);
        assert_eq!(HR_FRAME_BYTES, 14);
    }

    /// Annex B round-trip for both layouts.
    #[test]
    fn bits_roundtrip_both_modes() {
        for p in [sample_voiced(), sample_unvoiced()] {
            let bytes = p.to_bits();
            let q = HrParameters::from_bits(&bytes).unwrap();
            assert_eq!(p, q);
        }
    }

    /// Table B.1/B.2 prefix: R0 in b1..b5 means the first byte's top
    /// five bits are R0's MSB-first code; MODE lands in b35..b36.
    #[test]
    fn annex_b_bit_positions() {
        let mut p = sample_voiced();
        p.r0 = 0b10110;
        let bytes = p.to_bits();
        assert_eq!(bytes[0] >> 3, 0b10110, "R0 occupies b1..b5");
        // MODE = 2 at b35..b36 — bits 34..35 (0-based), i.e. byte 4
        // offsets 2..3 from the MSB.
        let b35 = (bytes[4] >> (7 - 2)) & 1;
        let b36 = (bytes[4] >> (7 - 3)) & 1;
        assert_eq!((b35 << 1) | b36, 2, "MODE code at b35..b36");
    }

    /// Word-layout round-trip for both layouts.
    #[test]
    fn cod_words_roundtrip() {
        for p in [sample_voiced(), sample_unvoiced()] {
            let words = p.to_cod_words();
            let q = HrParameters::from_cod_words(&words);
            assert_eq!(p, q);
        }
    }

    /// Short input is rejected.
    #[test]
    fn short_frame_rejected() {
        assert!(HrParameters::from_bits(&[0u8; 13]).is_err());
    }

    // ─── ROM table shape/content checks against spec-stated facts ───

    /// Annex A.2 / clause 4.1.4 sizes.
    #[test]
    fn table_shapes_match_spec() {
        assert_eq!(R0_DECODE_TABLE.len(), 63);
        assert_eq!(RC_SCALAR_DEQUANT.len(), 256);
        assert_eq!(RC_VQ_SEG1.len(), 2048 * 3 / 2); // packed 2 idx/word
        assert_eq!(RC_VQ_SEG2.len(), 512 * 3 / 2);
        assert_eq!(RC_VQ_SEG3.len(), 256 * 4 / 2);
        assert_eq!(RC_PREQ_SIZE, [64, 32, 16]);
        assert_eq!(RC_VQ_SUBSET_SIZE, [32, 16, 16]);
        assert_eq!(LAG_TABLE.len(), 256);
        assert_eq!(GSP0_VQ.len(), 4);
        assert_eq!(SQRT_P0.len(), 3);
        assert_eq!(BASIS_VECTORS_MODE0.len(), 2);
        assert_eq!(BASIS_VECTORS_MODE123.len(), 9);
    }

    /// Clause 4.1.8 table 3: lag 21 codes as 126 (1/6-sample units),
    /// the first range steps by 2 (resolution 1/3) through 136, then
    /// steps by 1 from 138 (lag 23, resolution 1/6). The table is
    /// strictly increasing.
    #[test]
    fn lag_table_matches_printed_table_3() {
        assert_eq!(&LAG_TABLE[..7], &[126, 128, 130, 132, 134, 136, 138]);
        // 21 * 6 = 126; the last lag is 142 ⇒ 852.
        assert_eq!(LAG_TABLE[255], 142 * 6);
        for w in LAG_TABLE.windows(2) {
            assert!(w[0] < w[1], "allowable lags strictly increase");
        }
    }

    /// Clause 4.1.1 equations (3)/(4): the printed high-pass biquad
    /// coefficients in Q15.
    #[test]
    fn highpass_matches_printed_equations() {
        assert_eq!(
            HIGHPASS_COEFFS,
            [10979, -21954, 10979, -14071, 30347, 10979, -21936, 10979, -15385, 31632]
        );
    }

    /// Clause 4.1.6 table 2: the soft-interpolation weights in Q15,
    /// current + previous summing to 1.0 (32767-ish; the Q15 pairs
    /// are exact complements of 32768 except the saturated last).
    #[test]
    fn soft_interpolation_weights_match_table_2() {
        assert_eq!(SOFT_INTERP_CURRENT, [9830, 20316, 30147, 32767]);
        assert_eq!(SOFT_INTERP_PREVIOUS, [22938, 12452, 2621, 0]);
    }

    /// The scalar de-quantizer holds Q15 reflection coefficients and
    /// must be monotone (a quantizer table).
    #[test]
    fn rc_scalar_dequantizer_is_monotone() {
        for w in RC_SCALAR_DEQUANT.windows(2) {
            assert!(w[0] <= w[1], "scalar dequantizer must be monotone");
        }
    }

    /// R0 decoding (clause 4.1.5): 32 codes spanning −66..−4 dB in
    /// 2 dB steps; the decode table has 2·32 − 1 = 63 rows.
    #[test]
    fn r0_decode_table_shape() {
        assert_eq!(R0_DECODE_TABLE.len(), 2 * 32 - 1);
    }
}
