//! SV8 `EI` (Encoder Info) packet payload field-map decoder.
//!
//! The `EI` packet carries encoder identification — the quality
//! profile, the perceptual-noise-substitution (PNS) enable flag, and
//! the three-component encoder version (major / minor / build). In
//! SV7 the profile + encoder-version byte lived inline in the fixed
//! header; SV8 moves them into this dedicated packet.
//!
//! Up to this module the [`crate::typed_packet::EncoderInfoPacket`]
//! wrapper held only the opaque payload slice. This module decodes
//! that slice into [`EncoderInfoFields`].
//!
//! Source-of-record (facts only):
//!
//! - `docs/audio/musepack/spec/musepack-headers-and-coding.md` §2 —
//!   the `EI` payload field-map: "7-bit profile (the stored value is
//!   profile×8, i.e. divide by 8 to recover the fractional profile),
//!   1-bit PNS (noise-substitution) flag, then three 8-bit bytes —
//!   encoder major, minor, build — packed into a 32-bit version word
//!   as `(major<<24)|(minor<<16)|(build<<8)`."
//! - `docs/audio/musepack/spec/musepack-headers-and-coding.md` §4 —
//!   SV8 bytes are read in natural order (no SV7-style word-swap).
//!
//! # Layout (§2)
//!
//! ```text
//! [ profile×8 : 7 ][ pns : 1 ][ major : 8 ][ minor : 8 ][ build : 8 ]
//! ```
//!
//! The first byte packs the 7-bit `profile×8` field in its high seven
//! bits and the PNS flag in its low bit. The next three whole bytes
//! are the encoder version components.

use crate::{Error, Result};

/// Number of bytes a complete `EI` payload occupies: one packed
/// profile+PNS byte plus three version bytes.
pub const EI_PAYLOAD_LEN: usize = 1 + 3;

/// Decoded SV8 `EI` (Encoder Info) payload fields (§2).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EncoderInfoFields {
    /// The raw 7-bit profile field exactly as stored — i.e.
    /// `profile × 8` (§2). Use [`Self::profile`] to recover the
    /// fractional profile value, or [`Self::profile_int`] for the
    /// integer part.
    pub profile_raw: u8,
    /// Perceptual-noise-substitution enable flag (§2, 1-bit).
    pub pns: bool,
    /// Encoder major version (§2, first of the three version bytes).
    pub major: u8,
    /// Encoder minor version (§2, second version byte).
    pub minor: u8,
    /// Encoder build number (§2, third version byte).
    pub build: u8,
}

impl EncoderInfoFields {
    /// Decode an `EI` payload slice (the bytes between the packet's
    /// size varint and the next packet) into its fields.
    ///
    /// # Errors
    ///
    /// - [`Error::UnexpectedEof`] if the slice is shorter than
    ///   [`EI_PAYLOAD_LEN`] bytes.
    pub fn parse(payload: &[u8]) -> Result<Self> {
        let bytes = payload.get(..EI_PAYLOAD_LEN).ok_or(Error::UnexpectedEof)?;

        // Byte 0: high 7 bits are profile×8, low bit is PNS (§2, §4
        // natural byte order, MSB-first within the byte).
        let first = bytes[0];
        let profile_raw = first >> 1;
        let pns = (first & 0x01) != 0;

        Ok(Self {
            profile_raw,
            pns,
            major: bytes[1],
            minor: bytes[2],
            build: bytes[3],
        })
    }

    /// The fractional encoder quality profile: the stored field
    /// divided by 8 (§2 — "the stored value is profile×8, i.e. divide
    /// by 8 to recover the fractional profile").
    pub fn profile(&self) -> f32 {
        f32::from(self.profile_raw) / 8.0
    }

    /// The integer part of the encoder quality profile
    /// (`profile_raw / 8`, truncated).
    pub fn profile_int(&self) -> u8 {
        self.profile_raw >> 3
    }

    /// The encoder version packed into a 32-bit word as §2 specifies:
    /// `(major << 24) | (minor << 16) | (build << 8)`.
    pub fn version_word(&self) -> u32 {
        (u32::from(self.major) << 24) | (u32::from(self.minor) << 16) | (u32::from(self.build) << 8)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build an `EI` payload from a raw profile field, PNS flag, and
    /// version triple.
    fn build(profile_raw: u8, pns: bool, major: u8, minor: u8, build_n: u8) -> Vec<u8> {
        let first = (profile_raw << 1) | (pns as u8);
        vec![first, major, minor, build_n]
    }

    #[test]
    fn parses_canonical_payload() {
        // profile_raw = 80 (= profile 10.0), pns set, version 1.30.0.
        let payload = build(80, true, 1, 30, 0);
        let ei = EncoderInfoFields::parse(&payload).expect("valid EI");
        assert_eq!(ei.profile_raw, 80);
        assert!(ei.pns);
        assert_eq!(ei.major, 1);
        assert_eq!(ei.minor, 30);
        assert_eq!(ei.build, 0);
    }

    #[test]
    fn profile_divides_by_eight() {
        // 84 / 8 = 10.5 fractional; integer part 10.
        let payload = build(84, false, 0, 0, 0);
        let ei = EncoderInfoFields::parse(&payload).unwrap();
        assert_eq!(ei.profile(), 10.5);
        assert_eq!(ei.profile_int(), 10);
    }

    #[test]
    fn pns_flag_is_low_bit() {
        let on = EncoderInfoFields::parse(&build(40, true, 0, 0, 0)).unwrap();
        let off = EncoderInfoFields::parse(&build(40, false, 0, 0, 0)).unwrap();
        assert!(on.pns);
        assert!(!off.pns);
        // The profile field is unaffected by the PNS bit.
        assert_eq!(on.profile_raw, off.profile_raw);
        assert_eq!(on.profile_raw, 40);
    }

    #[test]
    fn version_word_packs_per_spec() {
        let ei = EncoderInfoFields::parse(&build(0, false, 0x12, 0x34, 0x56)).unwrap();
        assert_eq!(ei.version_word(), 0x1234_5600);
    }

    #[test]
    fn profile_raw_uses_full_seven_bits() {
        // Maximum 7-bit profile field = 127.
        let ei = EncoderInfoFields::parse(&build(127, false, 0, 0, 0)).unwrap();
        assert_eq!(ei.profile_raw, 127);
    }

    #[test]
    fn rejects_truncated() {
        let payload = vec![0u8; EI_PAYLOAD_LEN - 1];
        assert_eq!(
            EncoderInfoFields::parse(&payload),
            Err(Error::UnexpectedEof)
        );
    }

    #[test]
    fn ignores_trailing_bytes() {
        let mut payload = build(72, true, 2, 5, 9);
        payload.push(0xFF);
        let ei = EncoderInfoFields::parse(&payload).unwrap();
        assert_eq!(ei.profile_int(), 9);
        assert_eq!((ei.major, ei.minor, ei.build), (2, 5, 9));
    }

    #[test]
    fn payload_len_constant_is_four() {
        assert_eq!(EI_PAYLOAD_LEN, 4);
    }
}
