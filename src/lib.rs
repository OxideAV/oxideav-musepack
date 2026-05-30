//! Pure-Rust Musepack audio codec.
//!
//! **Clean-room rebuild in progress** (orphan `master` post the
//! 2026-05-06 docs audit). The crate is being grown back up against
//! the staged structural spec at
//! `docs/audio/musepack/musepack-sv7-sv8-spec.md` plus the numeric
//! tables under `docs/audio/musepack/tables/` (CSV + `.meta`
//! sidecars, extracted under the *Feist v. Rural* (1991) facts-only
//! exception by a walled extraction round — see
//! `docs/audio/musepack/provenance/01-musepack-table-extraction.md`).
//!
//! ## Format outline (overview-level)
//!
//! Musepack ships in two incompatible stream-format generations:
//!
//! - **SV7** (Stream Version 7, aka *MPEGplus / MP+*, c. 1997-2005):
//!   subband filter inherited from MPEG-1 Layer 2 (32-band polyphase)
//!   plus replaced bit-allocation, quantisation, and Huffman coding.
//!   Filename `.mpc` or legacy `.mp+`.
//! - **SV8** (c. 2008-): different bitstream packaging (KEY / SIZE /
//!   PAYLOAD packets, magic `MPCK`) and updated entropy coding.
//!   Same subband filter and psychoacoustic model as SV7; the upgrade
//!   is mainly in container framing, gapless-playback metadata, and
//!   chapter support.
//!
//! Both targets are ReplayGain-tagged by default. Stream-format level
//! 3 (8 channels) is supported in principle though almost never used.
//!
//! ## Module surface so far
//!
//! - [`requant`] — SV7 §2.5 / §2.6 requantiser constants:
//!   `RES_BITS[18]`, `QUANTIZER_OFFSET_D[19]`,
//!   `DEQUANT_COEFFICIENT_C[19]`, and `SCF_STEP_RATIO`.
//!
//! Header parsing, frame body walking, Huffman decoding, CNS noise
//! substitution, and the synthesis filterbank are still pending.
//! See `CHANGELOG.md` `[Unreleased]` for the gap list.

#![forbid(unsafe_code)]

pub mod requant;

/// Crate-local error type. Concrete variants land as the Implementer
/// rounds populate the codec pipeline.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Error {
    /// Reserved placeholder. Replaced by real variants in round 1.
    NotImplemented,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::NotImplemented => f.write_str(
                "oxideav-musepack: clean-room rebuild in progress — see crates/oxideav-musepack/README.md",
            ),
        }
    }
}

impl std::error::Error for Error {}

/// Crate-local `Result` alias.
pub type Result<T> = core::result::Result<T, Error>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn error_display_points_at_readme() {
        let s = format!("{}", Error::NotImplemented);
        assert!(
            s.contains("clean-room rebuild"),
            "Error::NotImplemented Display should mention the clean-room rebuild status; got: {s}"
        );
        assert!(
            s.contains("README.md"),
            "Error::NotImplemented Display should point at the crate README; got: {s}"
        );
    }

    #[test]
    fn error_is_std_error() {
        // Compile-time check: Error implements std::error::Error.
        fn assert_error<E: std::error::Error>() {}
        assert_error::<Error>();
    }

    #[test]
    fn error_is_clone_and_eq() {
        let a = Error::NotImplemented;
        let b = a.clone();
        assert_eq!(a, b);
    }

    #[test]
    fn result_alias_resolves() {
        let ok: Result<u32> = Ok(7);
        let err: Result<u32> = Err(Error::NotImplemented);
        assert_eq!(ok, Ok(7));
        assert_eq!(err, Err(Error::NotImplemented));
    }
}
