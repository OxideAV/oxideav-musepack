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
//! - [`framing`] — SV7 / SV8 stream-magic identification and the
//!   SV8 packet outer-frame walker (key + varint size).
//! - [`huffman`] — SV7 `mpc_huffman`-shape entropy tables
//!   (`sv7-huffman-bandtype-header` / `sv7-huffman-scfi` /
//!   `sv7-huffman-dscf` / `sv7-huffman-q{1..=7}`) plus a
//!   left-justified-code linear decoder and an MSB-first bit
//!   reader. The `[2][N]` quantiser tables are exposed both as the
//!   full concatenated array and as per-context slices.
//! - [`cns`] — CNS / noise-substitution two-LFSR PRNG and the
//!   256-byte parity-of-popcount lookup that drives it
//!   (`cns-prng-parity` + `cns-prng-params`).
//! - [`sv7_band_decode`] — SV7 §2.5 per-band sample-decode dispatch
//!   for the unambiguous cases (`-1` CNS / `0` empty / `3..=7`
//!   Huffman-per-sample / `8..=17` linear-PCM escape) plus a
//!   classifier enum covering every spec case (grouped cases `1` /
//!   `2` are flagged but not wired — their per-codeword sample-
//!   unpack convention is GAP in the structural prose).
//!
//! Per-field header decoding, the SV7 per-frame 20-bit length
//! prefix + "read in 32-LSB units" packing, the SV8 canonical-
//! huffman entropy layer (`sv8-canonical-*` + `sv8-symbols-*`),
//! the SCF base-index decode, and the synthesis filterbank are
//! still pending. See `CHANGELOG.md` `[Unreleased]` for the gap
//! list.

#![forbid(unsafe_code)]

pub mod cns;
pub mod framing;
pub mod huffman;
pub mod requant;
pub mod sv7_band_decode;

/// Crate-local error type. Concrete variants land as the Implementer
/// rounds populate the codec pipeline.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Error {
    /// Reserved placeholder. Replaced by real variants in round 1.
    NotImplemented,
    /// The input did not start with the expected stream magic
    /// (`MP+` for SV7 or `MPCK` for SV8).
    InvalidMagic,
    /// The input ran out before the requested item could be parsed.
    UnexpectedEof,
    /// The SV7 stream version byte's low nibble was not
    /// [`framing::SV7_VERSION_NIBBLE`]. The full version byte is
    /// included so a caller can log which version was rejected.
    UnsupportedVersion(u8),
    /// A varint kept its continuation bit set past the maximum
    /// supported byte length.
    VarintTooLong,
    /// The peeked 16-bit code window did not match any row of the
    /// supplied SV7 Huffman table — a malformed bitstream or a
    /// wrong-context table for the current sample.
    HuffmanNoMatch,
    /// A per-band sample-decode dispatcher was called with a
    /// `band_type` value that is either outside the structurally-
    /// documented range or in a case that is not yet wired
    /// (currently SV7 §2.5 cases 1 / 2 — grouped codewords — whose
    /// per-codeword sample-unpack convention is DOCS-GAP, plus an
    /// invalid `ctx` value for the cases that take one). The
    /// out-of-range value is reported so callers can log which
    /// `band_type` was rejected.
    UnsupportedBandType(i8),
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::NotImplemented => f.write_str(
                "oxideav-musepack: clean-room rebuild in progress — see crates/oxideav-musepack/README.md",
            ),
            Error::InvalidMagic => f.write_str(
                "oxideav-musepack: input does not start with the SV7 (MP+) or SV8 (MPCK) magic",
            ),
            Error::UnexpectedEof => {
                f.write_str("oxideav-musepack: unexpected end of input while parsing")
            }
            Error::UnsupportedVersion(byte) => write!(
                f,
                "oxideav-musepack: unsupported SV7 stream version (version byte {byte:#04x})",
            ),
            Error::VarintTooLong => f.write_str(
                "oxideav-musepack: varint exceeded the supported maximum byte length",
            ),
            Error::HuffmanNoMatch => f.write_str(
                "oxideav-musepack: no SV7 Huffman table entry matched the peeked code window",
            ),
            Error::UnsupportedBandType(bt) => write!(
                f,
                "oxideav-musepack: unsupported or out-of-range band_type {bt} for the sample-decode dispatcher",
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
