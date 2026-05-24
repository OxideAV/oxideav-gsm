//! # oxideav-gsm
//!
//! **Status:** orphan-rebuild scaffold (reset 2026-05-25).
//!
//! The prior implementation was retired under the workspace clean-room
//! policy: its source and licensing declared the codec was modelled on
//! an external reference implementation, whose provenance the clean-room
//! policy does not permit. The policy forbids consulting any external
//! implementation's source for any reason, so the provenance could not
//! be defended. The crate will be re-implemented from scratch against a
//! staged ETSI GSM 06.10 (RPE-LTP) specification in a future clean-room
//! round, once that specification is staged under `docs/audio/gsm/`.
//!
//! Every public API currently returns [`Error::NotImplemented`].

#![warn(missing_debug_implementations)]

use oxideav_core::RuntimeContext;

/// Crate-local error type. Until the clean-room rebuild lands every
/// public API path returns [`Error::NotImplemented`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// The crate has been reset to a scaffold pending clean-room
    /// rebuild; no decoder or encoder functionality is wired up yet.
    NotImplemented,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "oxideav-gsm: orphan-rebuild scaffold — no codec wired up")
    }
}

impl std::error::Error for Error {}

/// No-op codec registration — the orphan-rebuild scaffold registers
/// nothing into the runtime context.
pub fn register(_ctx: &mut RuntimeContext) {}

oxideav_core::register!("gsm", register);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scaffold_error_displays() {
        assert_eq!(Error::NotImplemented, Error::NotImplemented);
        assert!(!format!("{}", Error::NotImplemented).is_empty());
    }
}
