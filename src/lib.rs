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
//! - [`reconstruct`] — SV7 §2.6 per-sample reconstruction
//!   primitives (centring of PCM-escape raw levels by subtracting
//!   `D`; per-band dequant multiply by `C / 65536`; CNS dequant
//!   path keyed off `DEQUANT_COEFFICIENT_C[0]`). The downstream SCF
//!   multiply, M/S undo, and synthesis filterbank are out of scope
//!   here.
//! - [`scf`] — SV7 §2.4 SCF coding-method decoder: reads the
//!   per-non-zero-band SCFI selector VLC, classifies it into a
//!   granule-coverage schedule (mirroring Layer-II SCFSI per §1
//!   lines 79-82), then reads N DSCF deltas and reconstructs the
//!   three per-granule SCF indices given a per-band base anchor.
//! - [`sv7_band_header`] — SV7 §2.3 per-band header loop walker:
//!   reads the `band_type` Huffman VLC per channel (stereo: left
//!   first, then right) and the conditional 1-bit `msflag` that
//!   follows iff at least one channel's `band_type` is non-zero,
//!   over `0..=max_band`. Returns a `BandHeader { band_type:
//!   [RawBandTypeVlc; 2], ms_flag: Option<bool> }` sequence. The
//!   raw VLC value is wrapped in [`sv7_band_header::RawBandTypeVlc`]
//!   to keep the §2.3-VLC-symbol → §2.5-dispatcher-case remap
//!   honest (the remap shape is DOCS-GAP and not yet wired).
//! - [`sv8_band_decode`] — SV8 §3.4 per-band sample-decode case
//!   classifier mirroring [`sv7_band_decode::BandDecodeCase`] for
//!   the SV8 ladder shape (`Cns` / `Empty` / `SparseBand` /
//!   `Grouped3` / `Grouped2` / `ContextHuffmanPerSample` /
//!   `LargeCoeffEscape` / `OutOfRange`). Pure structural dispatch:
//!   one `const fn` plus two predicate helpers
//!   ([`sv8_band_decode::case_emits_samples`],
//!   [`sv8_band_decode::case_uses_first_order_context`]) routing
//!   `band_type` to its §3.4 `switch` arm. The per-case sample
//!   decoders live downstream of the SV8 canonical-Huffman entropy
//!   layer (`sv8-canonical-*` + `sv8-symbols-*` tables, staged
//!   under `docs/audio/musepack/tables/`).
//! - [`packet_stream`] — SV8 §3.1/§3.2 packet-stream walker on top
//!   of [`framing::parse_packet_header`]. `PacketStream::new` takes
//!   the post-`MPCK` slice plus a [`packet_stream::PacketSizeConvention`]
//!   pick (the GAP varint convention) and yields one
//!   [`packet_stream::PacketRef`] per call until the `SE`
//!   terminator. Payload bytes are surfaced as opaque borrows
//!   over the input slice — the per-payload field maps (`SH` /
//!   `RG` / `EI` / `SO` / `ST`) remain GAP per §3.2.
//! - [`typed_packet`] — typed §3.2 packet surface: each known
//!   2-byte key maps to a per-kind borrowed newtype
//!   ([`typed_packet::StreamHeaderPacket`] / `ReplayGainPacket` /
//!   `EncoderInfoPacket` / `SeekTableOffsetPacket` /
//!   `SeekTablePacket` / `AudioPacket` / `StreamEndPacket`), all
//!   wrapped in a [`typed_packet::TypedPacket`] sum that callers can
//!   `match` instead of re-validating raw `PacketKey` strings.
//!   Payload bytes remain opaque borrows over the input — field
//!   maps continue to be GAP per §3.2.
//! - [`stream_shape`] — SV8 stream-shape observer: walks a complete
//!   `MPCK`-prefixed byte buffer via [`framing::parse_sv8_magic`] +
//!   [`packet_stream::PacketStream`] + [`typed_packet::TypedPacket`]
//!   and surfaces a [`stream_shape::StreamShape`] summary of
//!   per-§3.2-kind counts, cumulative opaque payload bytes, and
//!   first/last seen packet kinds. Pure observer — no payload
//!   interpretation, no ordering enforcement.
//! - [`sv8_huffman`] — SV8 §3.4 / §3.5 canonical Huffman
//!   length-tables and paired int8 symbol maps wired as typed
//!   statics. Exposes 21 [`sv8_huffman::Sv8CanonicalTable`] views
//!   (`Bands`, `Res-{1,2}`, `Scfi-{1,2}`, `Dscf-{1,2}`, `Q1`,
//!   `Q2-{1,2}`, `Q3`, `Q4`, `Q5-{1,2}`..`Q8-{1,2}`, `Q9up`) plus
//!   a [`sv8_huffman::Sv8TableRole`] enum + first-order context
//!   dispatcher [`sv8_huffman::table_for_role`]. The cumulative-
//!   index → symbol-index decoder walk is a structural §3.4
//!   DOCS-GAP and is not wired this round — see the module-level
//!   docs for the spec gap.
//!
//! Per-field header decoding (including the per-band SCF anchor
//! the [`scf`] module currently takes as an argument), the SV7
//! per-frame 20-bit length prefix + "read in 32-LSB units"
//! packing, the SV8 canonical-Huffman cumulative-index decoder
//! walk (tables now wired via [`sv8_huffman`]; the per-row
//! sub-index arithmetic remains §3.4 DOCS-GAP), the SCF index →
//! gain anchor for §2.6, and the synthesis filterbank are still
//! pending. See `CHANGELOG.md` `[Unreleased]` for the gap list.

#![forbid(unsafe_code)]

pub mod cns;
pub mod framing;
pub mod huffman;
pub mod packet_stream;
pub mod reconstruct;
pub mod requant;
pub mod scf;
pub mod stream_shape;
pub mod sv7_band_decode;
pub mod sv7_band_header;
pub mod sv8_band_decode;
pub mod sv8_huffman;
pub mod typed_packet;

/// Total subband samples per frame per channel, inherited from
/// MPEG-1 Layer II (32 polyphase subbands × 36 samples per band).
///
/// Per `docs/audio/musepack/musepack-sv7-sv8-spec.md` §1 lines 65-71
/// ("One frame contains 36 × 32 = 1152 subband samples") this value
/// is identical for SV7 and SV8 — only the entropy / framing layer
/// differs between the two stream versions; the underlying sample
/// geometry is shared.
pub const SAMPLES_PER_FRAME_PER_CHANNEL: usize = 1152;

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
    /// The SV7 §2.4 SCFI VLC decoded a value outside the
    /// structurally-documented `0..=3` range. The offending raw
    /// value is included for diagnostic logging.
    InvalidScfCodingMethod(i8),
    /// The §2.3 band-type header loop was driven with a `max_band`
    /// parameter above the Layer-II 32-subband heritage's inclusive
    /// upper bound (`SV7_MAX_BAND_INCLUSIVE == 31`). The offending
    /// value is included for diagnostic logging.
    MaxBandOutOfRange(u8),
    /// A per-band decoder was driven with a `nch` (channel count)
    /// other than 1 or 2. The offending value is included for
    /// diagnostic logging. Multi-channel streams (the SH-packet
    /// "level 3 = 8 channels" SV8 upgrade) need a separate decode
    /// path that is not wired this round.
    ChannelCountInvalid(u8),
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
            Error::InvalidScfCodingMethod(raw) => write!(
                f,
                "oxideav-musepack: SCFI VLC produced value {raw} outside the spec §2.4 0..=3 range",
            ),
            Error::MaxBandOutOfRange(value) => write!(
                f,
                "oxideav-musepack: max_band {value} exceeds the spec §1 Layer-II 32-subband inclusive bound 31",
            ),
            Error::ChannelCountInvalid(nch) => write!(
                f,
                "oxideav-musepack: channel count {nch} is not 1 (mono) or 2 (stereo) at the §2.3 band-header layer",
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

    #[test]
    fn samples_per_frame_per_channel_matches_layer_two_heritage() {
        // §1 lines 65-71: 32 subbands × 36 samples = 1152.
        assert_eq!(SAMPLES_PER_FRAME_PER_CHANNEL, 1152);
        assert_eq!(
            SAMPLES_PER_FRAME_PER_CHANNEL,
            sv7_band_header::SV7_SUBBAND_COUNT * sv7_band_decode::SAMPLES_PER_BAND,
        );
    }
}
