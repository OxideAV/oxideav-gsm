//! Crate-local error type.

/// Errors surfaced by the GSM 06.10 decoder.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// The crate has been reset to a scaffold pending clean-room
    /// rebuild; reserved for APIs that are not yet wired up.
    NotImplemented,
    /// Caller fed fewer than 33 bytes — a GSM 06.10 frame is 260
    /// bits (rounded up to 33 bytes per §1.7 Table 1.1).
    ShortFrame,
    /// The 33-byte `.gsm` byte-frame did not carry the 0xD marker
    /// nibble in the high nibble of byte 0 (see
    /// [`crate::UnpackedFrame::from_gsm_byte_frame`]).
    BadByteFrameMagic,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::NotImplemented => write!(f, "oxideav-gsm: feature not yet implemented"),
            Self::ShortFrame => write!(
                f,
                "oxideav-gsm: input shorter than the 33-byte minimum for one 260-bit frame"
            ),
            Self::BadByteFrameMagic => write!(
                f,
                "oxideav-gsm: .gsm byte-frame missing the 0xD marker nibble"
            ),
        }
    }
}

impl std::error::Error for Error {}
