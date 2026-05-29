//! Fixed-point quantisation tables from ETSI EN 300 961 (GSM 06.10
//! RPE-LTP) §5.4 "Tables used in the fixed point implementation of
//! the RPE-LTP coder and decoder".
//!
//! Every value here is read directly from the staged spec PDF
//! (`docs/audio/gsm/etsi-gsm-06.10-rpe-ltp.pdf`, §5.4 tables 5.1 –
//! 5.6). They are integer scalings of the floating-point quantiser
//! parameters described in §3 and are used unchanged in §5.2.8 (LAR
//! decode), §5.2.16 (APCM inverse quantisation) and §5.3.2 (LTP gain
//! decode).

/// §5.4 Table 5.1 col A — multiplier in LAR coding/decoding eq.
/// (3.6) / (3.7). Indexed 1..=8 (entry 0 is a sentinel zero).
pub const A: [i16; 9] = [0, 20480, 20480, 20480, 20480, 13964, 15360, 8534, 9036];

/// §5.4 Table 5.1 col B — additive bias in LAR coding/decoding eq.
/// (3.6) / (3.7). Same indexing as [`A`].
pub const B: [i16; 9] = [0, 0, 0, 2048, -2560, 94, -1792, -341, -1144];

/// §5.4 Table 5.1 col MIC — minimum permissible value of LARc[i].
pub const MIC: [i16; 9] = [0, -32, -32, -16, -16, -8, -8, -4, -4];

/// §5.4 Table 5.1 col MAC — maximum permissible value of LARc[i].
pub const MAC: [i16; 9] = [0, 31, 31, 15, 15, 7, 7, 3, 3];

/// §5.4 Table 5.2 — `INVA[i] = integer((32768*8) / real_A[i])` used
/// by the LAR decode procedure §5.2.8. Indexed 1..=8.
pub const INVA: [i16; 9] = [0, 13107, 13107, 13107, 13107, 19223, 17476, 31454, 29708];

/// §5.4 Table 5.3a — `DLB[bc]` decision-level levels of the LTP
/// gain quantiser, indexed by 4-state coded LTP gain `bc` (0..=3).
/// Only the first three entries (0..=2) are used by the encoder's
/// decision tree; the table is included in full for completeness.
pub const DLB: [i16; 4] = [6554, 16384, 26214, 32767];

/// §5.4 Table 5.3b — `QLB[bc]` quantisation levels of the LTP gain
/// quantiser. The decoder reads `QLB[bc]` per §5.3.2 to recover the
/// Q15 gain `brp` used in long-term synthesis.
pub const QLB: [i16; 4] = [3277, 11469, 21299, 32767];

/// §5.4 Table 5.4 — coefficients `H[i]` of the encoder's RPE
/// weighting filter §5.2.13. Included so that the table is staged
/// alongside the decoder tables, but unused by the decoder.
pub const H: [i16; 11] = [-134, -374, 0, 2054, 5741, 8192, 5741, 2054, 0, -374, -134];

/// §5.4 Table 5.5 — `NRFAC[i]`, the inverse-mantissa values used
/// by the encoder's APCM forward quantisation §5.2.15. Included so
/// the table is staged alongside the decoder tables, but unused by
/// the decoder.
pub const NRFAC: [i16; 8] = [29128, 26215, 23832, 21846, 20165, 18725, 17476, 16384];

/// §5.4 Table 5.6 — `FAC[i]`, the direct-mantissa values used by
/// the decoder's APCM inverse quantisation §5.2.16 / §5.3.1 to
/// recover the un-normalised RPE-pulse magnitudes from the coded
/// `xMc[i]` indices.
pub const FAC: [i16; 8] = [18431, 20479, 22527, 24575, 26623, 28671, 30719, 32767];

#[cfg(test)]
mod tests {
    use super::*;

    /// Table 5.1 must have eight live entries (index 1..=8).
    #[test]
    fn table_5_1_shape() {
        assert_eq!(A.len(), 9);
        assert_eq!(B.len(), 9);
        assert_eq!(MIC.len(), 9);
        assert_eq!(MAC.len(), 9);
        for i in 1..=8 {
            assert!(MIC[i] < MAC[i], "MIC[{i}] < MAC[{i}]");
        }
    }

    /// LAR-1/-2 widest range (±32 = 6 bits) ↔ Table 1.1 column
    /// "Number of bits" col 6.
    #[test]
    fn lar1_lar2_six_bits() {
        assert_eq!(MAC[1] - MIC[1] + 1, 64);
        assert_eq!(MAC[2] - MIC[2] + 1, 64);
    }

    /// LAR-7/-8 narrowest range (±4 = 3 bits) ↔ Table 1.1 col 3.
    #[test]
    fn lar7_lar8_three_bits() {
        assert_eq!(MAC[7] - MIC[7] + 1, 8);
        assert_eq!(MAC[8] - MIC[8] + 1, 8);
    }

    /// FAC[i] is monotonically increasing with i.
    #[test]
    fn fac_monotonic() {
        for i in 1..FAC.len() {
            assert!(FAC[i] > FAC[i - 1]);
        }
    }

    /// QLB[i] is monotonically increasing with i (Q15 gain levels).
    #[test]
    fn qlb_monotonic() {
        for i in 1..QLB.len() {
            assert!(QLB[i] > QLB[i - 1]);
        }
    }
}
