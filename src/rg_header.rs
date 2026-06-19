//! SV8 `RG` (ReplayGain) packet payload field-map decoder.
//!
//! The `RG` packet carries the bitstream-level loudness metadata that
//! the SV7 fixed header packed inline (the ReplayGain title/album
//! gain + peak quad). In SV8 those fields moved out of the `SH`
//! stream header into their own `RG` packet.
//!
//! Up to this module the [`crate::typed_packet::ReplayGainPacket`]
//! wrapper held only the opaque payload slice. This module decodes
//! that slice into [`ReplayGainFields`].
//!
//! Source-of-record (facts only):
//!
//! - `docs/audio/musepack/spec/musepack-headers-and-coding.md` §2 —
//!   the `RG` payload field-map: "8-bit version (must be 1), then four
//!   16-bit fields in order: title gain, title peak, album gain,
//!   album peak."
//! - `docs/audio/musepack/spec/musepack-headers-and-coding.md` §4 —
//!   SV8 bytes are read in natural order (no SV7-style word-swap).
//!
//! # Layout (§2)
//!
//! ```text
//! [ version : 8 ][ title_gain : 16 ][ title_peak : 16 ]
//! [ album_gain : 16 ][ album_peak : 16 ]
//! ```
//!
//! Every field is byte-aligned, so the payload decodes straight off
//! the slice as one version byte followed by four big-endian 16-bit
//! words. The gain/peak values are surfaced **verbatim** (the raw
//! 16-bit quantities) — the dB/linear rescaling the consumer applies
//! to ReplayGain values is a metadata concern outside this codec's
//! bitstream scope, and the staged facts do not pin a rescale
//! formula here.

use crate::{Error, Result};

/// The required SV8 `RG` packet version value
/// (`spec/musepack-headers-and-coding.md` §2 — "8-bit version (must
/// be 1)").
pub const SV8_REPLAYGAIN_VERSION: u8 = 1;

/// Number of bytes a complete `RG` payload occupies: one version byte
/// plus four 16-bit fields.
pub const RG_PAYLOAD_LEN: usize = 1 + 4 * 2;

/// Decoded SV8 `RG` (ReplayGain) payload fields (§2).
///
/// All four gain/peak quantities are the **raw** 16-bit values as
/// stored; no dB or linear rescaling is applied (see the module
/// docs).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ReplayGainFields {
    /// Packet version byte (§2). Always [`SV8_REPLAYGAIN_VERSION`] for
    /// a successfully decoded `RG`.
    pub version: u8,
    /// ReplayGain title gain — raw 16-bit value (§2, field order 1).
    pub title_gain: u16,
    /// ReplayGain title peak — raw 16-bit value (§2, field order 2).
    pub title_peak: u16,
    /// ReplayGain album gain — raw 16-bit value (§2, field order 3).
    pub album_gain: u16,
    /// ReplayGain album peak — raw 16-bit value (§2, field order 4).
    pub album_peak: u16,
}

impl ReplayGainFields {
    /// Decode an `RG` payload slice (the bytes between the packet's
    /// size varint and the next packet) into its fields.
    ///
    /// # Errors
    ///
    /// - [`Error::UnexpectedEof`] if the slice is shorter than
    ///   [`RG_PAYLOAD_LEN`] bytes.
    /// - [`Error::InvalidReplayGainVersion`] if the version byte is
    ///   not [`SV8_REPLAYGAIN_VERSION`].
    pub fn parse(payload: &[u8]) -> Result<Self> {
        let bytes = payload.get(..RG_PAYLOAD_LEN).ok_or(Error::UnexpectedEof)?;

        let version = bytes[0];
        if version != SV8_REPLAYGAIN_VERSION {
            return Err(Error::InvalidReplayGainVersion(version));
        }

        // Four big-endian 16-bit fields immediately after the version
        // byte, in order: title gain, title peak, album gain, album
        // peak (§2, §4 natural byte order).
        let be16 = |off: usize| -> u16 { u16::from_be_bytes([bytes[off], bytes[off + 1]]) };

        Ok(Self {
            version,
            title_gain: be16(1),
            title_peak: be16(3),
            album_gain: be16(5),
            album_peak: be16(7),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build an `RG` payload from explicit field values.
    fn build(version: u8, tg: u16, tp: u16, ag: u16, ap: u16) -> Vec<u8> {
        let mut v = Vec::with_capacity(RG_PAYLOAD_LEN);
        v.push(version);
        v.extend_from_slice(&tg.to_be_bytes());
        v.extend_from_slice(&tp.to_be_bytes());
        v.extend_from_slice(&ag.to_be_bytes());
        v.extend_from_slice(&ap.to_be_bytes());
        v
    }

    #[test]
    fn parses_canonical_payload() {
        let payload = build(1, 0x1234, 0x5678, 0x9ABC, 0xDEF0);
        let rg = ReplayGainFields::parse(&payload).expect("valid RG");
        assert_eq!(rg.version, 1);
        assert_eq!(rg.title_gain, 0x1234);
        assert_eq!(rg.title_peak, 0x5678);
        assert_eq!(rg.album_gain, 0x9ABC);
        assert_eq!(rg.album_peak, 0xDEF0);
    }

    #[test]
    fn field_order_matches_spec() {
        // Distinct values per field so an order swap would surface.
        let payload = build(1, 1, 2, 3, 4);
        let rg = ReplayGainFields::parse(&payload).unwrap();
        assert_eq!(
            (rg.title_gain, rg.title_peak, rg.album_gain, rg.album_peak),
            (1, 2, 3, 4)
        );
    }

    #[test]
    fn rejects_wrong_version() {
        let payload = build(2, 0, 0, 0, 0);
        assert_eq!(
            ReplayGainFields::parse(&payload),
            Err(Error::InvalidReplayGainVersion(2))
        );
    }

    #[test]
    fn rejects_truncated() {
        // One byte short of a full payload.
        let payload = vec![0u8; RG_PAYLOAD_LEN - 1];
        assert_eq!(ReplayGainFields::parse(&payload), Err(Error::UnexpectedEof));
    }

    #[test]
    fn ignores_trailing_bytes() {
        // A longer payload parses the leading RG_PAYLOAD_LEN bytes and
        // ignores any trailing padding.
        let mut payload = build(1, 0xAAAA, 0xBBBB, 0xCCCC, 0xDDDD);
        payload.extend_from_slice(&[0xFF, 0xFF, 0xFF]);
        let rg = ReplayGainFields::parse(&payload).unwrap();
        assert_eq!(rg.title_gain, 0xAAAA);
        assert_eq!(rg.album_peak, 0xDDDD);
    }

    #[test]
    fn payload_len_constant_is_nine() {
        assert_eq!(RG_PAYLOAD_LEN, 9);
    }
}
