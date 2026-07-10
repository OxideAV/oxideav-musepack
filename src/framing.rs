//! SV7 / SV8 stream-identification and SV8 packet framing.
//!
//! This module covers the parts of the Musepack container that are
//! **structurally** specified by independent sources — the stream
//! magic bytes for both generations, the SV7 stream-version nibble,
//! and the SV8 packet-key vocabulary plus the
//! `[2-byte key][varint size][payload]` packet outer frame.
//!
//! Source-of-record:
//!
//! - `docs/audio/musepack/musepack-sv7-sv8-spec.md`
//!   * §2.1 — SV7 identification (`MP+` magic + version byte, low
//!     nibble `7`).
//!   * §3.1 — SV8 packet (chunk) framing — `MPCK` stream magic and
//!     the `[ASCII key][varint size][payload]` outer frame.
//!   * §3.2 — SV8 packet kinds (`SH`, `RG`, `EI`, `SO`, `ST`, `AP`,
//!     `SE`).
//!
//! # What is GAP and intentionally NOT implemented here
//!
//! The structural spec (and the original §2.1 / §3.2 / §3.4 GAP
//! tables) leaves the **bit-precise field layouts inside each
//! header / packet body** to a future observer-trace round (the
//! pending Musepack observer-trace task). In particular:
//!
//! - The SV7 fixed-header field map **after** the magic + version
//!   byte (sample-count, intensity / MS flags, `max_band`,
//!   encoder-profile / quality, gapless trailing-sample count,
//!   ReplayGain title/album gain+peak) is documented only on the
//!   project's walled Trac `SV7Specification` page and is GAP here.
//!   This module's [`SV7Header::parse_magic`] returns only the
//!   stream-version nibble plus a slice over the rest of the fixed
//!   header — it does **not** decode those fields.
//! - The SV7 per-frame 20-bit length prefix and the
//!   "read in 32-LSB units" bitstream packing are documented in the
//!   structural spec but are part of the frame-body decoder, not
//!   the container header, and are not implemented in this module.
//! - The SV8 `SH` / `RG` / `EI` / `SO` / `ST` payload field maps
//!   are GAP per the spec's §3.2 table. The packet-key enum and
//!   the outer-frame walker classify and skip them; their inner
//!   bytes are returned as opaque slices.
//! - The SV8 varint **convention** (whether the size field is
//!   inclusive of the key + size header, and the exact byte order
//!   of multi-byte encodings) is the one outer-frame detail the
//!   spec flags as GAP. The walker here implements the **standard
//!   continuation-bit big-endian** varint shape that the spec
//!   describes structurally and exposes both interpretations of
//!   the decoded size as separate fields on [`PacketHeader`] so a
//!   caller can pick the right one once the observer trace lands.
//!
//! All values and shapes in this module come from the staged
//! material under `docs/audio/musepack/`.

use crate::{Error, Result};

// ─── SV7 ────────────────────────────────────────────────────────

/// SV7 stream magic — ASCII `MP+`.
///
/// Per `docs/audio/musepack/musepack-sv7-sv8-spec.md` §2.1.
pub const SV7_MAGIC: [u8; 3] = *b"MP+";

/// SV7 stream-version nibble.
///
/// Per the structural spec §2.1 the byte after the `MP+` magic
/// encodes the stream version, with the low nibble equal to this
/// constant for an SV7 stream.
pub const SV7_VERSION_NIBBLE: u8 = 7;

/// SV7 version-byte **PNS/CNS stream flag** (bit `0x10`).
///
/// Per the staged CNS fixture notes
/// (`docs/audio/musepack/fixtures/cns-pns/notes.md`, "CNS confirmation
/// … Version-byte flag"): when the encoder engages PNS / Clear Noise
/// Substitution it sets this bit in the version byte — the stream
/// reads `MP+ 0x17` instead of `MP+ 0x07`. The flag is informational
/// for a decoder (CNS bands are self-describing via their `Res == -1`
/// band type); a stream decodes identically either way. The remaining
/// high-nibble bits (`0x20`/`0x40`/`0x80`) stay GAP.
pub const SV7_VERSION_PNS_FLAG: u8 = 0x10;

/// What [`SV7Header::parse_magic`] managed to recognise at the
/// start of a candidate SV7 stream.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SV7Header<'a> {
    /// The full version byte that follows the `MP+` magic. The low
    /// nibble is the SV7 version (always [`SV7_VERSION_NIBBLE`]
    /// when this struct is returned by [`SV7Header::parse_magic`]);
    /// bit `0x10` of the high nibble is the PNS/CNS stream flag
    /// ([`SV7_VERSION_PNS_FLAG`], see [`SV7Header::pns`]); the other
    /// high-nibble bits are GAP.
    pub version_byte: u8,
    /// All bytes after the `MP+` magic and the version byte. The
    /// internal field map for these bytes — sample count,
    /// intensity / MS flags, `max_band`, encoder profile, gapless
    /// trailing-sample count, ReplayGain — is GAP and **not**
    /// decoded by this module.
    pub remaining_header_bytes: &'a [u8],
}

impl<'a> SV7Header<'a> {
    /// Recognise an SV7 stream at the start of `input`.
    ///
    /// Returns the version byte and a slice over the rest of the
    /// fixed header without interpreting it. Errors with
    /// [`Error::InvalidMagic`] if the leading three bytes are not
    /// `MP+`, [`Error::UnexpectedEof`] if there are fewer than
    /// four bytes available, and [`Error::UnsupportedVersion`] if
    /// the low nibble of the version byte is not
    /// [`SV7_VERSION_NIBBLE`].
    pub fn parse_magic(input: &'a [u8]) -> Result<Self> {
        if input.len() < SV7_MAGIC.len() + 1 {
            return Err(Error::UnexpectedEof);
        }
        if input[..SV7_MAGIC.len()] != SV7_MAGIC {
            return Err(Error::InvalidMagic);
        }
        let version_byte = input[SV7_MAGIC.len()];
        if version_byte & 0x0F != SV7_VERSION_NIBBLE {
            return Err(Error::UnsupportedVersion(version_byte));
        }
        Ok(Self {
            version_byte,
            remaining_header_bytes: &input[SV7_MAGIC.len() + 1..],
        })
    }

    /// The low nibble of [`Self::version_byte`] — always
    /// [`SV7_VERSION_NIBBLE`] for a successfully parsed SV7 stream.
    pub fn stream_version(&self) -> u8 {
        self.version_byte & 0x0F
    }

    /// Whether the version byte carries the PNS/CNS stream flag
    /// ([`SV7_VERSION_PNS_FLAG`], bit `0x10`): the encoder marked this
    /// stream as using noise substitution (`MP+ 0x17`). Informational —
    /// CNS bands announce themselves per-band via `Res == -1`.
    pub fn pns(&self) -> bool {
        self.version_byte & SV7_VERSION_PNS_FLAG != 0
    }
}

// ─── SV8 ────────────────────────────────────────────────────────

/// SV8 stream magic — ASCII `MPCK`.
///
/// Per `docs/audio/musepack/musepack-sv7-sv8-spec.md` §3.1.
pub const SV8_MAGIC: [u8; 4] = *b"MPCK";

/// SV8 packet kind, identified by its 2-character ASCII key.
///
/// The complete vocabulary per spec §3.2 is `{SH, RG, EI, SO, ST,
/// AP, SE}`. Any other key is held in the [`PacketKey::Unknown`]
/// variant as its raw two bytes for forward compatibility — the
/// observer-trace round is allowed to surface additional keys.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PacketKey {
    /// `SH` — Stream Header. First payload packet, analogue of the
    /// SV7 fixed header. Payload field map is GAP (spec §3.2).
    StreamHeader,
    /// `RG` — ReplayGain loudness metadata. Payload field sizes
    /// are GAP (spec §3.2).
    ReplayGain,
    /// `EI` — Encoder Info (profile / quality / PNS flag / encoder
    /// version). Payload is GAP.
    EncoderInfo,
    /// `SO` — Seek-table Offset. A single offset pointing to where
    /// the `ST` table sits in the stream. Payload is GAP.
    SeekTableOffset,
    /// `ST` — Seek Table proper (entry count + delta-coded
    /// offsets). Payload is GAP.
    SeekTable,
    /// `AP` — Audio Packet, carrying the SV8 entropy-coded frame
    /// body. Inner structure is documented at spec §3.4 / §3.5;
    /// the Huffman tables are staged in `docs/audio/musepack/tables/`.
    AudioPacket,
    /// `SE` — Stream End terminator.
    StreamEnd,
    /// An ASCII key not in the §3.2 vocabulary. The raw two bytes
    /// are preserved so a caller can log them.
    Unknown([u8; 2]),
}

impl PacketKey {
    /// Decode a 2-byte ASCII packet key.
    pub fn from_bytes(bytes: [u8; 2]) -> Self {
        match &bytes {
            b"SH" => Self::StreamHeader,
            b"RG" => Self::ReplayGain,
            b"EI" => Self::EncoderInfo,
            b"SO" => Self::SeekTableOffset,
            b"ST" => Self::SeekTable,
            b"AP" => Self::AudioPacket,
            b"SE" => Self::StreamEnd,
            _ => Self::Unknown(bytes),
        }
    }

    /// The 2-byte ASCII key as it appears in the stream.
    pub fn as_bytes(&self) -> [u8; 2] {
        match self {
            Self::StreamHeader => *b"SH",
            Self::ReplayGain => *b"RG",
            Self::EncoderInfo => *b"EI",
            Self::SeekTableOffset => *b"SO",
            Self::SeekTable => *b"ST",
            Self::AudioPacket => *b"AP",
            Self::StreamEnd => *b"SE",
            Self::Unknown(b) => *b,
        }
    }

    /// True if this is a known §3.2 packet kind (not
    /// [`PacketKey::Unknown`]).
    pub fn is_known(&self) -> bool {
        !matches!(self, Self::Unknown(_))
    }
}

/// A decoded SV8 packet outer-frame header.
///
/// The packet is laid out as `[2-byte key][varint size][payload]`
/// per spec §3.1. The `size` varint convention — specifically
/// whether the decoded value counts the key + size bytes — is GAP
/// per spec §3.1, so this struct exposes both interpretations:
/// [`Self::raw_size`] (the literal varint value) and
/// [`Self::header_len`] (the number of bytes consumed by the key
/// plus the size varint themselves). Callers can compute the
/// payload range either way.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PacketHeader {
    /// The 2-byte ASCII packet key, classified.
    pub key: PacketKey,
    /// The literal varint value decoded from the size field.
    pub raw_size: u64,
    /// Bytes consumed by the key (2) plus the size varint itself.
    /// The payload starts at this offset relative to the start of
    /// the packet.
    pub header_len: usize,
}

impl PacketHeader {
    /// Payload length assuming the spec §3.1 sentence "total packet
    /// length (key + size field + payload)" is the convention in
    /// force (i.e. the varint is inclusive of the header bytes).
    pub fn payload_len_inclusive(&self) -> Option<u64> {
        (self.raw_size).checked_sub(self.header_len as u64)
    }

    /// Payload length assuming the varint is exclusive of the
    /// header bytes (the alternative convention). Used until the
    /// observer-trace round confirms which interpretation is right.
    pub fn payload_len_exclusive(&self) -> u64 {
        self.raw_size
    }
}

/// Parse an SV8 packet outer-frame header from the start of
/// `input`. On success returns the header and the offset at which
/// the payload begins.
///
/// Per spec §3.1 the size field is a continuation-bit big-endian
/// varint — each byte carries 7 payload bits with a high bit set
/// on all bytes except the last. This routine accepts up to a
/// 9-byte varint (sufficient for 63 bits of size, well past any
/// realistic Musepack packet length).
pub fn parse_packet_header(input: &[u8]) -> Result<PacketHeader> {
    if input.len() < 2 {
        return Err(Error::UnexpectedEof);
    }
    let key = PacketKey::from_bytes([input[0], input[1]]);
    let (raw_size, size_len) = parse_varint(&input[2..])?;
    Ok(PacketHeader {
        key,
        raw_size,
        header_len: 2 + size_len,
    })
}

/// Decode a continuation-bit big-endian varint from the start of
/// `input`. Returns `(value, bytes_consumed)`.
///
/// Each byte contributes its low 7 bits to the value (high-order
/// chunk first); the high bit is the continuation flag, set on
/// every byte except the last. Accepts at most 9 bytes (up to 63
/// bits of payload).
pub fn parse_varint(input: &[u8]) -> Result<(u64, usize)> {
    const MAX_BYTES: usize = 9;
    let mut value: u64 = 0;
    for (i, &byte) in input.iter().take(MAX_BYTES).enumerate() {
        // Guard against overflow on the 10th 7-bit chunk: at i==9
        // we'd be shifting 63 bits of payload, fine; the loop cap
        // above already limits us to i<=8 anyway.
        value = (value << 7) | u64::from(byte & 0x7F);
        if byte & 0x80 == 0 {
            return Ok((value, i + 1));
        }
    }
    if input.len() < MAX_BYTES {
        Err(Error::UnexpectedEof)
    } else {
        Err(Error::VarintTooLong)
    }
}

/// Recognise an SV8 stream by its leading `MPCK` magic. Returns
/// the offset at which the first packet starts (i.e. immediately
/// after the magic).
pub fn parse_sv8_magic(input: &[u8]) -> Result<usize> {
    if input.len() < SV8_MAGIC.len() {
        return Err(Error::UnexpectedEof);
    }
    if input[..SV8_MAGIC.len()] != SV8_MAGIC {
        return Err(Error::InvalidMagic);
    }
    Ok(SV8_MAGIC.len())
}

/// Stream generation, identified by leading magic bytes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StreamKind {
    /// SV7 — `MP+` magic. Frame container is non-byte-aligned and
    /// not implemented here.
    Sv7,
    /// SV8 — `MPCK` magic. Outer packet frame is implemented by
    /// [`parse_packet_header`].
    Sv8,
}

/// Identify the stream generation from leading magic bytes without
/// decoding any header fields.
pub fn identify_stream(input: &[u8]) -> Result<StreamKind> {
    if input.len() >= SV8_MAGIC.len() && input[..SV8_MAGIC.len()] == SV8_MAGIC {
        return Ok(StreamKind::Sv8);
    }
    if input.len() >= SV7_MAGIC.len() && input[..SV7_MAGIC.len()] == SV7_MAGIC {
        return Ok(StreamKind::Sv7);
    }
    if input.len() < SV8_MAGIC.len() {
        return Err(Error::UnexpectedEof);
    }
    Err(Error::InvalidMagic)
}

#[cfg(test)]
mod tests {
    use super::*;

    // ─── SV7 magic + version ────────────────────────────────

    #[test]
    fn sv7_magic_constant_is_ascii_mp_plus() {
        assert_eq!(&SV7_MAGIC, b"MP+");
        assert_eq!(SV7_MAGIC, [0x4D, 0x50, 0x2B]);
    }

    #[test]
    fn sv7_parse_magic_accepts_low_nibble_seven() {
        // `MP+` followed by 0x17 (high nibble 1, low nibble 7) +
        // some opaque header bytes.
        let buf = [b'M', b'P', b'+', 0x17, 0xAA, 0xBB, 0xCC];
        let h = SV7Header::parse_magic(&buf).expect("magic accepted");
        assert_eq!(h.version_byte, 0x17);
        assert_eq!(h.stream_version(), 7);
        assert_eq!(h.remaining_header_bytes, &[0xAA, 0xBB, 0xCC]);
    }

    #[test]
    fn sv7_pns_flag_is_version_byte_bit_0x10() {
        // Fixture-pinned (cns-pns notes.md): 0x17 = PNS stream, 0x07 =
        // plain SV7; other high-nibble bits do not trip the flag.
        for (version, pns) in [(0x07u8, false), (0x17, true), (0x27, false), (0x37, true)] {
            let buf = [b'M', b'P', b'+', version];
            let h = SV7Header::parse_magic(&buf).expect("magic accepted");
            assert_eq!(h.pns(), pns, "version byte {version:#04x}");
            assert_eq!(h.stream_version(), 7);
        }
        assert_eq!(SV7_VERSION_PNS_FLAG, 0x10);
    }

    #[test]
    fn sv7_parse_magic_rejects_wrong_magic() {
        let buf = [b'M', b'P', b'4', 0x07];
        assert_eq!(SV7Header::parse_magic(&buf), Err(Error::InvalidMagic));
    }

    #[test]
    fn sv7_parse_magic_rejects_wrong_version_nibble() {
        // Low nibble is 6 (SV6 / MPEGplus 6), not 7.
        let buf = [b'M', b'P', b'+', 0x06];
        assert_eq!(
            SV7Header::parse_magic(&buf),
            Err(Error::UnsupportedVersion(0x06)),
        );
    }

    #[test]
    fn sv7_parse_magic_rejects_short_input() {
        // Only the 3-byte magic, no version byte.
        let buf = *b"MP+";
        assert_eq!(SV7Header::parse_magic(&buf), Err(Error::UnexpectedEof));
    }

    // ─── SV8 magic ──────────────────────────────────────────

    #[test]
    fn sv8_magic_constant_is_ascii_mpck() {
        assert_eq!(&SV8_MAGIC, b"MPCK");
        assert_eq!(SV8_MAGIC, [0x4D, 0x50, 0x43, 0x4B]);
    }

    #[test]
    fn sv8_parse_magic_accepts_and_returns_offset() {
        let buf = *b"MPCKSH";
        assert_eq!(parse_sv8_magic(&buf), Ok(4));
    }

    #[test]
    fn sv8_parse_magic_rejects_wrong_magic() {
        let buf = [b'M', b'P', b'+', 0x07];
        assert_eq!(parse_sv8_magic(&buf), Err(Error::InvalidMagic));
    }

    #[test]
    fn sv8_parse_magic_rejects_short_input() {
        let buf = *b"MPC";
        assert_eq!(parse_sv8_magic(&buf), Err(Error::UnexpectedEof));
    }

    // ─── Packet keys ────────────────────────────────────────

    #[test]
    fn packet_key_round_trips_every_known_kind() {
        for (bytes, kind) in [
            (*b"SH", PacketKey::StreamHeader),
            (*b"RG", PacketKey::ReplayGain),
            (*b"EI", PacketKey::EncoderInfo),
            (*b"SO", PacketKey::SeekTableOffset),
            (*b"ST", PacketKey::SeekTable),
            (*b"AP", PacketKey::AudioPacket),
            (*b"SE", PacketKey::StreamEnd),
        ] {
            assert_eq!(PacketKey::from_bytes(bytes), kind);
            assert_eq!(kind.as_bytes(), bytes);
            assert!(kind.is_known());
        }
    }

    #[test]
    fn packet_key_unknown_preserves_raw_bytes() {
        let k = PacketKey::from_bytes(*b"XX");
        assert_eq!(k, PacketKey::Unknown(*b"XX"));
        assert_eq!(k.as_bytes(), *b"XX");
        assert!(!k.is_known());
    }

    // ─── Varint ─────────────────────────────────────────────

    #[test]
    fn varint_single_byte_low() {
        // Smallest non-zero single-byte value.
        let (v, n) = parse_varint(&[0x01]).unwrap();
        assert_eq!((v, n), (1, 1));
    }

    #[test]
    fn varint_single_byte_max() {
        // Largest single-byte value (high bit clear, 7 bits set).
        let (v, n) = parse_varint(&[0x7F]).unwrap();
        assert_eq!((v, n), (127, 1));
    }

    #[test]
    fn varint_two_byte_continuation() {
        // 0x81 = continuation + low 7 bits 0x01;
        // 0x00 = terminator with low 7 bits 0x00.
        // Big-endian 7-bit packing: value = (1 << 7) | 0 = 128.
        let (v, n) = parse_varint(&[0x81, 0x00]).unwrap();
        assert_eq!((v, n), (128, 2));
    }

    #[test]
    fn varint_three_byte() {
        // 0x82, 0x80, 0x01 -> (2 << 14) | (0 << 7) | 1 = 32769.
        let (v, n) = parse_varint(&[0x82, 0x80, 0x01]).unwrap();
        assert_eq!((v, n), (32769, 3));
    }

    #[test]
    fn varint_truncated_continuation() {
        // Continuation flag on the last byte we have -> EOF.
        let (e_short, e_long) = (
            parse_varint(&[0x80]).unwrap_err(),
            parse_varint(&[0x80; 9]).unwrap_err(),
        );
        assert_eq!(e_short, Error::UnexpectedEof);
        assert_eq!(e_long, Error::VarintTooLong);
    }

    // ─── Packet header walker ───────────────────────────────

    #[test]
    fn packet_header_parses_sh_with_single_byte_size() {
        // SH packet, size varint = 0x10 (16), then 14 payload bytes
        // (under the inclusive convention) or 16 payload bytes
        // (under the exclusive convention). The header itself is 3
        // bytes (2 key + 1 varint size). Payload bytes are not
        // included in this test buffer — only the header is parsed.
        let buf = [b'S', b'H', 0x10];
        let h = parse_packet_header(&buf).unwrap();
        assert_eq!(h.key, PacketKey::StreamHeader);
        assert_eq!(h.raw_size, 16);
        assert_eq!(h.header_len, 3);
        assert_eq!(h.payload_len_inclusive(), Some(13));
        assert_eq!(h.payload_len_exclusive(), 16);
    }

    #[test]
    fn packet_header_parses_ap_with_two_byte_size() {
        // AP packet, size varint 0x81 0x00 = 128.
        let buf = [b'A', b'P', 0x81, 0x00];
        let h = parse_packet_header(&buf).unwrap();
        assert_eq!(h.key, PacketKey::AudioPacket);
        assert_eq!(h.raw_size, 128);
        assert_eq!(h.header_len, 4);
        assert_eq!(h.payload_len_inclusive(), Some(124));
        assert_eq!(h.payload_len_exclusive(), 128);
    }

    #[test]
    fn packet_header_parses_stream_end() {
        // SE terminator with a zero-length size varint.
        let buf = [b'S', b'E', 0x00];
        let h = parse_packet_header(&buf).unwrap();
        assert_eq!(h.key, PacketKey::StreamEnd);
        assert_eq!(h.raw_size, 0);
        assert_eq!(h.header_len, 3);
        // Inclusive convention would underflow at a sub-header
        // size — None signals "size too small to be inclusive."
        assert_eq!(h.payload_len_inclusive(), None);
        assert_eq!(h.payload_len_exclusive(), 0);
    }

    #[test]
    fn packet_header_rejects_short_input() {
        assert_eq!(parse_packet_header(b"S"), Err(Error::UnexpectedEof));
        assert_eq!(parse_packet_header(b"AP"), Err(Error::UnexpectedEof));
    }

    // ─── Stream identification ──────────────────────────────

    #[test]
    fn identify_stream_distinguishes_sv7_and_sv8() {
        assert_eq!(identify_stream(b"MPCK..").unwrap(), StreamKind::Sv8);
        assert_eq!(identify_stream(b"MP+\x07..").unwrap(), StreamKind::Sv7);
        assert_eq!(identify_stream(b"RIFFwave"), Err(Error::InvalidMagic));
        assert_eq!(identify_stream(b"MP"), Err(Error::UnexpectedEof));
    }

    // ─── Synthetic SV8 packet-stream walk ───────────────────

    #[test]
    fn walk_synthetic_sv8_packet_stream() {
        // Build a minimal SV8 packet stream: MPCK + SH(opaque 4
        // bytes) + AP(opaque 8 bytes) + SE(empty), all sized under
        // the EXCLUSIVE-convention reading.
        let mut buf: Vec<u8> = Vec::new();
        buf.extend_from_slice(&SV8_MAGIC);
        // SH, size=4, then 4 opaque bytes.
        buf.extend_from_slice(b"SH");
        buf.push(0x04);
        buf.extend_from_slice(&[0xDE, 0xAD, 0xBE, 0xEF]);
        // AP, size=8, then 8 opaque bytes.
        buf.extend_from_slice(b"AP");
        buf.push(0x08);
        buf.extend_from_slice(&[0; 8]);
        // SE, size=0.
        buf.extend_from_slice(b"SE");
        buf.push(0x00);

        // Walk it.
        let mut offset = parse_sv8_magic(&buf).unwrap();
        let mut seen: Vec<PacketKey> = Vec::new();
        while offset < buf.len() {
            let h = parse_packet_header(&buf[offset..]).unwrap();
            seen.push(h.key);
            let advance = h.header_len + h.payload_len_exclusive() as usize;
            offset += advance;
        }
        assert_eq!(offset, buf.len());
        assert_eq!(
            seen,
            vec![
                PacketKey::StreamHeader,
                PacketKey::AudioPacket,
                PacketKey::StreamEnd,
            ],
        );
    }
}
