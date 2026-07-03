//! SV7 (`MP+`) fixed-header **encode** — the exact inverse of
//! [`crate::sv7_header::Sv7HeaderFields::parse`].
//!
//! The SV7 fixed header is a 4-byte `MP+`+version prefix followed by a
//! 168-bit field span (`docs/audio/musepack/spec/musepack-headers-and-coding.md`
//! §1), read MSB-first over the §4 32-bit-word byte-swapped framing with
//! the field reads beginning at bit 32 (the first bit after the prefix
//! word). This module *produces* that layout:
//!
//! - [`write_sv7_header_fields`] appends the **logical** (post-word-swap)
//!   200-bit header run — the reversed prefix word, then the 17 fields in
//!   the exact §1 order and widths — to an [`Sv7BitWriter`], so a
//!   whole-stream composer can continue the §1.1 continuous audio bit run
//!   directly after field 17 in the same writer.
//! - [`encode_sv7_header`] serialises a standalone on-disk header: the
//!   logical run word-swapped back to raw byte order (§4 is involutive),
//!   28 bytes (the 25 logical bytes rounded up to the 32-bit word grid,
//!   zero-padded — the pad bits are the first three body bytes' positions
//!   in a full stream, zero here).
//!
//! Every field is range-validated before any bit is written (fail-loud,
//! never silently-masked): the §1 sanity gate `1 ≤ max_band ≤ 31` and
//! each narrow field's declared bit width. Round-trip is proven in the
//! tests: `parse(encode(fields)) == fields` for every field.
//!
//! Source-of-record (facts only):
//!
//! - `docs/audio/musepack/spec/musepack-headers-and-coding.md` §1 — the
//!   field order/widths and the sanity gate; §4 — the 32-bit-word
//!   byte-swap framing and "field reads begin after the 4-byte prefix".
//! - No new format facts: this is the algebraic inverse of the already
//!   grounded `sv7_header` parse, round-tripped against it bit-for-bit.

use crate::framing::{SV7_MAGIC, SV7_VERSION_NIBBLE};
use crate::sv7_bitwriter::Sv7BitWriter;
use crate::sv7_header::{Sv7HeaderFields, SV7_MAX_BAND_INCLUSIVE};
use crate::{Error, Result};

/// Total logical header length in bits: the 32-bit `MP+`+version prefix
/// word plus the 168-bit §1 field span.
pub const SV7_HEADER_BITS: u64 = 32 + SV7_HEADER_FIELD_BITS;

/// The §1 field span (fields 1..17) in bits:
/// `32+1+1+6+4+2+2 + 5×16 + 1+11+1+19+8 = 168`.
pub const SV7_HEADER_FIELD_BITS: u64 = 168;

/// On-disk length of a standalone encoded header: the 25 logical bytes
/// (200 bits) rounded up to the §4 32-bit word grid.
pub const SV7_HEADER_DISK_LEN: usize = 28;

/// The default SV7 version byte: low nibble = [`SV7_VERSION_NIBBLE`],
/// high nibble 0. [`crate::sv7_header::Sv7HeaderFields::parse`] accepts
/// any byte whose low nibble is 7 (the high nibble is a minor-version
/// tag it ignores), so encoders that need a specific high nibble can use
/// [`write_sv7_header_fields`] / [`encode_sv7_header_with_version`].
pub const SV7_DEFAULT_VERSION_BYTE: u8 = SV7_VERSION_NIBBLE;

/// Validate every [`Sv7HeaderFields`] field against its §1 bit width and
/// the `1 ≤ max_band ≤ 31` sanity gate, without writing anything.
///
/// # Errors
///
/// - [`Error::MaxBandOutOfRange`] if `max_band` fails the §1 gate.
/// - [`Error::HeaderFieldOutOfRange`] naming the first field whose value
///   does not fit its declared width.
pub fn validate_sv7_header_fields(fields: &Sv7HeaderFields) -> Result<()> {
    if !(1..=SV7_MAX_BAND_INCLUSIVE).contains(&fields.max_band) {
        return Err(Error::MaxBandOutOfRange(fields.max_band));
    }
    // Each narrow field must fit its §1 width (the wide u16/u32 fields
    // fill their width exactly and cannot overflow).
    if fields.profile > 0x0F {
        return Err(Error::HeaderFieldOutOfRange("profile"));
    }
    if fields.link > 0x03 {
        return Err(Error::HeaderFieldOutOfRange("link"));
    }
    if fields.sample_freq_index > 0x03 {
        return Err(Error::HeaderFieldOutOfRange("sample_freq_index"));
    }
    if fields.last_frame_samples > 0x7FF {
        return Err(Error::HeaderFieldOutOfRange("last_frame_samples"));
    }
    if fields.reserved > 0x7_FFFF {
        return Err(Error::HeaderFieldOutOfRange("reserved"));
    }
    Ok(())
}

/// Append the **logical** (post-word-swap) 200-bit SV7 fixed header to
/// `writer`: the prefix word, then fields 1..17 in §1 order.
///
/// The §4 swap reverses each aligned 4-byte group, so the *raw* on-disk
/// prefix `['M', 'P', '+', version]` appears in the logical run as the
/// reversed word `[version, '+', 'P', 'M']` — that is what this writes,
/// so that `word_swap(logical)` yields a stream whose first bytes are the
/// `MP+` magic [`Sv7HeaderFields::parse`] validates on the raw input.
///
/// After this returns, `writer` is positioned at bit
/// [`SV7_HEADER_BITS`] — exactly where the §1.1 continuous audio bit run
/// begins — so a whole-stream composer appends frame bodies directly.
///
/// # Errors
///
/// - [`Error::UnsupportedVersion`] if `version_byte`'s low nibble is not
///   7 (such a header would be rejected by the parser).
/// - [`Error::MaxBandOutOfRange`] / [`Error::HeaderFieldOutOfRange`] from
///   [`validate_sv7_header_fields`]. Nothing is written on error.
pub fn write_sv7_header_fields(
    writer: &mut Sv7BitWriter,
    fields: &Sv7HeaderFields,
    version_byte: u8,
) -> Result<()> {
    if version_byte & 0x0F != SV7_VERSION_NIBBLE {
        return Err(Error::UnsupportedVersion(version_byte));
    }
    validate_sv7_header_fields(fields)?;

    // Prefix word, logical order (raw ['M','P','+',ver] reversed by §4).
    writer.write_bits(u32::from(version_byte), 8);
    writer.write_bits(u32::from(SV7_MAGIC[2]), 8); // '+'
    writer.write_bits(u32::from(SV7_MAGIC[1]), 8); // 'P'
    writer.write_bits(u32::from(SV7_MAGIC[0]), 8); // 'M'

    // Field 1: frame count, 32 bits as two 16-bit halves, high first
    // (§1 field 1 / §4 two-16-bit-read assembly convention).
    writer.write_bits(fields.frame_count >> 16, 16);
    writer.write_bits(fields.frame_count & 0xFFFF, 16);
    // Fields 2..7: the packed control word.
    writer.write_bits(u32::from(fields.intensity_stereo), 1); // field 2
    writer.write_bits(u32::from(fields.mid_side), 1); // field 3
    writer.write_bits(u32::from(fields.max_band), 6); // field 4
    writer.write_bits(u32::from(fields.profile), 4); // field 5
    writer.write_bits(u32::from(fields.link), 2); // field 6
    writer.write_bits(u32::from(fields.sample_freq_index), 2); // field 7
                                                               // Fields 8..12: max-level + the four 16-bit ReplayGain values.
    writer.write_bits(u32::from(fields.max_level), 16); // field 8
    writer.write_bits(u32::from(fields.title_gain), 16); // field 9
    writer.write_bits(u32::from(fields.title_peak), 16); // field 10
    writer.write_bits(u32::from(fields.album_gain), 16); // field 11
    writer.write_bits(u32::from(fields.album_peak), 16); // field 12
                                                         // Fields 13..17: gapless / seek flags, reserved, encoder version.
    writer.write_bits(u32::from(fields.true_gapless), 1); // field 13
    writer.write_bits(u32::from(fields.last_frame_samples), 11); // field 14
    writer.write_bits(u32::from(fields.fast_seek), 1); // field 15
    writer.write_bits(fields.reserved, 19); // field 16
    writer.write_bits(u32::from(fields.encoder_version), 8); // field 17

    Ok(())
}

/// Serialise a standalone on-disk SV7 fixed header
/// ([`SV7_HEADER_DISK_LEN`] bytes) with the default version byte.
///
/// The result begins with the raw `MP+` magic + version byte and
/// round-trips through [`Sv7HeaderFields::parse`]. The final three bytes
/// are the §4 word-grid zero pad (in a full stream those positions carry
/// the first audio-body bits — see the whole-stream composer).
///
/// # Errors
///
/// See [`write_sv7_header_fields`].
pub fn encode_sv7_header(fields: &Sv7HeaderFields) -> Result<Vec<u8>> {
    encode_sv7_header_with_version(fields, SV7_DEFAULT_VERSION_BYTE)
}

/// [`encode_sv7_header`] with an explicit version byte (low nibble must
/// be 7; the high nibble is the minor-version tag the parser ignores).
///
/// # Errors
///
/// See [`write_sv7_header_fields`].
pub fn encode_sv7_header_with_version(
    fields: &Sv7HeaderFields,
    version_byte: u8,
) -> Result<Vec<u8>> {
    let mut writer = Sv7BitWriter::new();
    write_sv7_header_fields(&mut writer, fields, version_byte)?;
    debug_assert_eq!(writer.bit_len(), SV7_HEADER_BITS);
    // 200 bits finish to 25 logical bytes; the §4 swap zero-extends the
    // trailing partial word, so the raw header is 28 bytes.
    let logical = writer.finish();
    let raw = crate::sv7_word_swap::word_swap_sv7_body(&logical);
    debug_assert_eq!(raw.len(), SV7_HEADER_DISK_LEN);
    Ok(raw)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A header with every field carrying a distinct, width-filling
    /// value, to pin each field's position and width in one round-trip.
    fn busy_fields() -> Sv7HeaderFields {
        Sv7HeaderFields {
            frame_count: 0xDEAD_BEEF,
            intensity_stereo: true,
            mid_side: true,
            max_band: 31,
            profile: 15,
            link: 3,
            sample_freq_index: 3,
            max_level: 0xA55A,
            title_gain: 0x1234,
            title_peak: 0x5678,
            album_gain: 0x9ABC,
            album_peak: 0xDEF0,
            true_gapless: true,
            last_frame_samples: 0x7FF,
            fast_seek: true,
            reserved: 0x7_FFFF,
            encoder_version: 0xC3,
        }
    }

    fn quiet_fields() -> Sv7HeaderFields {
        Sv7HeaderFields {
            frame_count: 3,
            max_band: 20,
            profile: 10,
            sample_freq_index: 0,
            encoder_version: 0x71,
            ..Default::default()
        }
    }

    #[test]
    fn encode_parse_round_trips_every_field() {
        for fields in [busy_fields(), quiet_fields()] {
            let raw = encode_sv7_header(&fields).expect("encode");
            let parsed = Sv7HeaderFields::parse(&raw).expect("parse");
            assert_eq!(parsed, fields);
        }
    }

    #[test]
    fn single_bit_fields_round_trip_independently() {
        // Flip each 1-bit field alone to prove none of them alias.
        let base = quiet_fields();
        for i in 0..4 {
            let mut fields = base;
            match i {
                0 => fields.intensity_stereo = true,
                1 => fields.mid_side = true,
                2 => fields.true_gapless = true,
                _ => fields.fast_seek = true,
            }
            let raw = encode_sv7_header(&fields).unwrap();
            assert_eq!(Sv7HeaderFields::parse(&raw).unwrap(), fields);
        }
    }

    #[test]
    fn output_is_28_bytes_and_starts_with_raw_magic() {
        let raw = encode_sv7_header(&quiet_fields()).unwrap();
        assert_eq!(raw.len(), SV7_HEADER_DISK_LEN);
        assert_eq!(&raw[..3], b"MP+");
        assert_eq!(raw[3], SV7_DEFAULT_VERSION_BYTE);
    }

    #[test]
    fn custom_version_byte_high_nibble_survives_parse() {
        // Low nibble 7 with a non-zero high nibble is accepted by the
        // parser (it checks only the low nibble).
        let raw = encode_sv7_header_with_version(&quiet_fields(), 0x17).unwrap();
        assert_eq!(raw[3], 0x17);
        assert_eq!(
            Sv7HeaderFields::parse(&raw).unwrap(),
            quiet_fields(),
            "fields unaffected by the version high nibble",
        );
    }

    #[test]
    fn rejects_version_byte_with_wrong_low_nibble() {
        let err = encode_sv7_header_with_version(&quiet_fields(), 0x08).err();
        assert_eq!(err, Some(Error::UnsupportedVersion(0x08)));
    }

    #[test]
    fn rejects_max_band_outside_sanity_gate() {
        for bad in [0u8, 32] {
            let mut fields = quiet_fields();
            fields.max_band = bad;
            assert_eq!(
                encode_sv7_header(&fields),
                Err(Error::MaxBandOutOfRange(bad)),
            );
        }
    }

    #[test]
    fn rejects_each_overwide_narrow_field() {
        type Poke = fn(&mut Sv7HeaderFields);
        let cases: [(&str, Poke); 5] = [
            ("profile", |f| f.profile = 16),
            ("link", |f| f.link = 4),
            ("sample_freq_index", |f| f.sample_freq_index = 4),
            ("last_frame_samples", |f| f.last_frame_samples = 0x800),
            ("reserved", |f| f.reserved = 0x8_0000),
        ];
        for (name, poke) in cases {
            let mut fields = quiet_fields();
            poke(&mut fields);
            assert_eq!(
                encode_sv7_header(&fields),
                Err(Error::HeaderFieldOutOfRange(name)),
                "field {name}",
            );
        }
    }

    #[test]
    fn writer_lands_exactly_at_the_body_bit_position() {
        let mut w = Sv7BitWriter::new();
        write_sv7_header_fields(&mut w, &busy_fields(), SV7_DEFAULT_VERSION_BYTE).unwrap();
        assert_eq!(w.bit_len(), SV7_HEADER_BITS);
        assert_eq!(SV7_HEADER_BITS, 200);
        assert_eq!(SV7_HEADER_FIELD_BITS, 168);
    }

    #[test]
    fn nothing_written_when_validation_fails() {
        let mut w = Sv7BitWriter::new();
        let mut fields = quiet_fields();
        fields.max_band = 0;
        assert!(write_sv7_header_fields(&mut w, &fields, SV7_DEFAULT_VERSION_BYTE).is_err());
        assert!(w.is_empty());
    }

    #[test]
    fn validate_accepts_every_boundary_value() {
        let mut fields = busy_fields();
        assert!(validate_sv7_header_fields(&fields).is_ok());
        fields.max_band = 1;
        assert!(validate_sv7_header_fields(&fields).is_ok());
    }
}
