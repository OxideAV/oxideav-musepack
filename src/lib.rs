//! Pure-Rust Musepack audio codec.
//!
//! **Round 0 — clean-room rebuild scaffold.** This is a fresh orphan
//! `master`; the previous implementation was retired alongside the
//! OxideAV docs audit dated 2026-05-06. See `README.md` for the
//! rebuild scope, the round-186 reassessment of the docs blocker,
//! and the strict-isolation clean-room workspace the Implementer
//! rounds will draw from.
//!
//! ## Format outline (overview-level, from staged `docs/audio/musepack/`)
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
//! ## Why this crate is a stub
//!
//! The byte-level field maps for both stream versions and the
//! Huffman / CNS / SCF numeric tables live on the project-shipped
//! `trac.musepack.net` wiki and the libmpcdec / mpcenc reference
//! source. Per the workspace clean-room policy
//! (`docs/audio/musepack/README.md`), project-shipped docs from
//! copyrighted-but-permissive licences are link-only — Implementer
//! agents do not read them. The unblock path is either a clean-room
//! observer-trace session or a docs-collaborator round that
//! transcribes the numeric tables from libmpcdec to
//! `docs/audio/musepack/tables/` under the *Feist v. Rural* (1991)
//! data-extraction exception (mirroring `docs/audio/g729/tables/`).
//! See `CHANGELOG.md` `[Unreleased]` "Blocked" for the field-by-field
//! gap list.

#![forbid(unsafe_code)]

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
