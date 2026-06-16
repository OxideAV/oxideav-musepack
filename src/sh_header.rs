//! SV8 `SH` (Stream Header) packet payload field-map decoder.
//!
//! The `SH` packet is the first payload packet of an SV8 stream and is
//! the analogue of the SV7 fixed header: it carries the stream-wide
//! parameters every later packet presumes — sample rate, channel count,
//! highest coded subband, stream M/S enable, audio-block size, and the
//! total / leading-silence sample counts.
//!
//! Up to this module the [`crate::typed_packet::StreamHeaderPacket`]
//! wrapper held only the opaque payload slice. This module decodes that
//! slice into [`StreamHeaderFields`].
//!
//! Source-of-record (facts only):
//!
//! - `docs/audio/musepack/spec/musepack-headers-and-coding.md` §2 —
//!   the `SH` payload field-map (CRC, stream version, sample count,
//!   beginning silence, sample-freq index, `max_band − 1`,
//!   `channels − 1`, mid/side, block power), the −1 / +1 field biases,
//!   and the {44100, 48000, 37800, 32000} Hz sample-rate table.
//! - `docs/audio/musepack/spec/musepack-headers-and-coding.md` §3 —
//!   the varint packing used by the sample-count / silence fields.
//! - `docs/audio/musepack/spec/musepack-headers-and-coding.md` §4 —
//!   SV8 bytes are read in natural order (no SV7-style word-swap), so
//!   the payload is decoded straight off the slice.
//!
//! # Layout (§2)
//!
//! The payload is byte-structured: the CRC and version occupy five
//! whole bytes, the two varints are byte-granular, and the trailing
//! `3 + 5 + 4 + 1 + 3 = 16` bits pack into exactly two bytes:
//!
//! ```text
//! [ CRC : 32 ][ version : 8 ][ sample_count : varint ]
//! [ beginning_silence : varint ][ freq_idx:3 max_band-1:5
//!                                 channels-1:4 ms:1 block_power:3 ]
//! ```
//!
//! The CRC value itself is surfaced verbatim; this module does **not**
//! validate it against the payload (the CRC-32 polynomial / coverage
//! beyond "from the byte after the CRC to the packet end" is a separate
//! concern and the staged facts do not pin the polynomial).

use crate::framing::parse_varint;
use crate::huffman::Sv7BitReader;
use crate::{Error, Result};

/// The SV8 sample-rate table — index → Hz.
///
/// Per `spec/musepack-headers-and-coding.md` §2 (field 5) only the
/// first four indices are defined: `{44100, 48000, 37800, 32000}` Hz.
pub const SV8_SAMPLE_RATES: [u32; 4] = [44100, 48000, 37800, 32000];

/// The required SV8 stream-version value carried in the `SH` payload
/// (`spec/musepack-headers-and-coding.md` §2, field 2).
pub const SV8_STREAM_VERSION: u8 = 8;

/// Decoded SV8 `SH` (Stream Header) payload fields (§2).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StreamHeaderFields {
    /// CRC-32 over the payload from the byte after the CRC to the
    /// packet end (§2, field 1). Surfaced verbatim; not validated by
    /// this module.
    pub crc: u32,
    /// Stream-version byte (§2, field 2). Always
    /// [`SV8_STREAM_VERSION`] for a successfully decoded `SH`.
    pub stream_version: u8,
    /// Total decoded sample count (§2, field 3 — varint).
    pub sample_count: u64,
    /// Leading samples to discard (encoder priming / gapless start;
    /// §2, field 4 — varint).
    pub beginning_silence: u64,
    /// Index into [`SV8_SAMPLE_RATES`] (§2, field 5).
    pub sample_freq_index: u8,
    /// Highest coded subband — the raw `max_band − 1` field already
    /// **un-biased** to `max_band` (§2, field 6, stored biased by −1).
    pub max_band: u8,
    /// Channel count — the raw `channels − 1` field already
    /// **un-biased** to `channels` (§2, field 7, stored biased by −1).
    pub channels: u8,
    /// Stream-wide M/S enable (§2, field 8). When set, per-band M/S
    /// flags appear in audio packets.
    pub mid_side: bool,
    /// Audio-block size exponent as stored (§2, field 9). The
    /// effective block exponent is `block_power × 2`; see
    /// [`Self::frames_per_audio_packet`].
    pub block_power: u8,
}

impl StreamHeaderFields {
    /// Decode an `SH` payload slice (the bytes between the packet's
    /// size varint and the next packet) into its fields.
    ///
    /// # Errors
    ///
    /// - [`Error::UnexpectedEof`] if the slice is shorter than any
    ///   field requires.
    /// - [`Error::VarintTooLong`] propagated from the sample-count or
    ///   beginning-silence varint.
    /// - [`Error::InvalidStreamVersion`] if the version byte is not
    ///   [`SV8_STREAM_VERSION`].
    pub fn parse(payload: &[u8]) -> Result<Self> {
        // Fields 1 + 2: CRC (32) and version (8) — five whole bytes,
        // read MSB-first off the natural-order SV8 byte stream (§4).
        let mut reader = Sv7BitReader::new(payload);
        // CRC is 32 bits; the reader caps a single fixed read at 16
        // bits (its mpc_huffman sizing), so assemble from two 16-bit
        // reads, high half first (§4 notes 32-bit quantities are
        // assembled from two 16-bit reads).
        let crc_hi = reader.read_bits(16)? as u32;
        let crc_lo = reader.read_bits(16)? as u32;
        let crc = (crc_hi << 16) | crc_lo;

        let stream_version = reader.read_bits(8)? as u8;
        if stream_version != SV8_STREAM_VERSION {
            return Err(Error::InvalidStreamVersion(stream_version));
        }

        // Fields 3 + 4 are byte-aligned varints (§3). After CRC (4
        // bytes) + version (1 byte) the reader sits on a byte
        // boundary, so the varints start at payload[5].
        let varint_region = payload.get(5..).ok_or(Error::UnexpectedEof)?;
        let (sample_count, n1) = parse_varint(varint_region)?;
        let after_count = varint_region.get(n1..).ok_or(Error::UnexpectedEof)?;
        let (beginning_silence, n2) = parse_varint(after_count)?;

        // Field 5..9: the packed 16-bit tail right after the two
        // varints. freq_idx:3, max_band-1:5, channels-1:4, ms:1,
        // block_power:3.
        let tail = after_count.get(n2..).ok_or(Error::UnexpectedEof)?;
        let mut tail_reader = Sv7BitReader::new(tail);
        let sample_freq_index = tail_reader.read_bits(3)? as u8;
        let max_band_biased = tail_reader.read_bits(5)? as u8;
        let channels_biased = tail_reader.read_bits(4)? as u8;
        let mid_side = tail_reader.read_bits(1)? != 0;
        let block_power = tail_reader.read_bits(3)? as u8;

        Ok(Self {
            crc,
            stream_version,
            sample_count,
            beginning_silence,
            sample_freq_index,
            // §2 biases: stored value + 1.
            max_band: max_band_biased + 1,
            channels: channels_biased + 1,
            mid_side,
            block_power,
        })
    }

    /// The decoded sample rate in Hz, or `None` if
    /// [`Self::sample_freq_index`] is outside the four defined
    /// indices (§2: only `0..=3` map to a rate).
    pub fn sample_rate_hz(&self) -> Option<u32> {
        SV8_SAMPLE_RATES
            .get(self.sample_freq_index as usize)
            .copied()
    }

    /// Number of audio frames carried by each `AP` packet, derived
    /// from the block-power field: `2^(block_power × 2)` (§2, field 9
    /// — "effective block exponent = field × 2").
    pub fn frames_per_audio_packet(&self) -> u64 {
        1u64 << (u32::from(self.block_power) * 2)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Field spec for [`Spec::build`]; defaults to a minimal valid
    /// stereo 44100 Hz `SH` payload.
    struct Spec {
        crc: u32,
        version: u8,
        sample_count_byte: u8,
        silence_byte: u8,
        freq_idx: u8,
        max_band_biased: u8,
        channels_biased: u8,
        ms: bool,
        block_power: u8,
    }

    impl Default for Spec {
        fn default() -> Self {
            Self {
                crc: 0,
                version: 8,
                sample_count_byte: 0,
                silence_byte: 0,
                freq_idx: 0,
                max_band_biased: 5,
                channels_biased: 1,
                ms: false,
                block_power: 0,
            }
        }
    }

    impl Spec {
        /// Serialise this spec into an `SH` payload byte slice.
        fn build(&self) -> Vec<u8> {
            let mut v = Vec::new();
            v.extend_from_slice(&self.crc.to_be_bytes());
            v.push(self.version);
            // Single-byte varints (high bit clear).
            v.push(self.sample_count_byte & 0x7F);
            v.push(self.silence_byte & 0x7F);
            // Packed tail: freq(3) | maxband-1(5) | ch-1(4) | ms(1) | bp(3)
            let bits: u16 = ((self.freq_idx as u16 & 0x7) << 13)
                | ((self.max_band_biased as u16 & 0x1F) << 8)
                | ((self.channels_biased as u16 & 0xF) << 4)
                | ((self.ms as u16) << 3)
                | (self.block_power as u16 & 0x7);
            v.extend_from_slice(&bits.to_be_bytes());
            v
        }
    }

    #[test]
    fn parses_canonical_stereo_44100() {
        // freq idx 0 → 44100, max_band-1 = 25 → 26, channels-1 = 1 → 2,
        // ms set, block_power 0.
        let payload = Spec {
            crc: 0xDEAD_BEEF,
            sample_count_byte: 5,
            silence_byte: 2,
            max_band_biased: 25,
            ms: true,
            ..Spec::default()
        }
        .build();
        let sh = StreamHeaderFields::parse(&payload).expect("valid SH");
        assert_eq!(sh.crc, 0xDEAD_BEEF);
        assert_eq!(sh.stream_version, 8);
        assert_eq!(sh.sample_count, 5);
        assert_eq!(sh.beginning_silence, 2);
        assert_eq!(sh.sample_freq_index, 0);
        assert_eq!(sh.sample_rate_hz(), Some(44100));
        assert_eq!(sh.max_band, 26);
        assert_eq!(sh.channels, 2);
        assert!(sh.mid_side);
        assert_eq!(sh.block_power, 0);
        assert_eq!(sh.frames_per_audio_packet(), 1);
    }

    #[test]
    fn field_biases_unbias_by_plus_one() {
        // max_band-1 = 0 → max_band 1; channels-1 = 0 → channels 1.
        let payload = Spec {
            freq_idx: 3,
            max_band_biased: 0,
            channels_biased: 0,
            ..Spec::default()
        }
        .build();
        let sh = StreamHeaderFields::parse(&payload).unwrap();
        assert_eq!(sh.max_band, 1);
        assert_eq!(sh.channels, 1);
        assert!(!sh.mid_side);
        assert_eq!(sh.sample_freq_index, 3);
        assert_eq!(sh.sample_rate_hz(), Some(32000));
    }

    #[test]
    fn block_power_drives_frames_per_packet() {
        // effective exponent = block_power × 2 → frames = 2^(bp×2).
        for (bp, frames) in [(0u8, 1u64), (1, 4), (2, 16), (3, 64)] {
            let payload = Spec {
                freq_idx: 1,
                max_band_biased: 10,
                block_power: bp,
                ..Spec::default()
            }
            .build();
            let sh = StreamHeaderFields::parse(&payload).unwrap();
            assert_eq!(sh.block_power, bp);
            assert_eq!(sh.frames_per_audio_packet(), frames);
        }
    }

    #[test]
    fn all_four_sample_rates_decode() {
        for (idx, hz) in SV8_SAMPLE_RATES.iter().enumerate() {
            let payload = Spec {
                freq_idx: idx as u8,
                ..Spec::default()
            }
            .build();
            let sh = StreamHeaderFields::parse(&payload).unwrap();
            assert_eq!(sh.sample_rate_hz(), Some(*hz));
        }
    }

    #[test]
    fn unknown_freq_index_has_no_rate() {
        // idx 5 is outside the four defined entries.
        let payload = Spec {
            freq_idx: 5,
            ..Spec::default()
        }
        .build();
        let sh = StreamHeaderFields::parse(&payload).unwrap();
        assert_eq!(sh.sample_freq_index, 5);
        assert_eq!(sh.sample_rate_hz(), None);
    }

    #[test]
    fn multibyte_varints_advance_correctly() {
        // sample_count = two-byte varint 0x81,0x00 → (1<<7)|0 = 128;
        // beginning_silence = single byte 3.
        let mut payload = Vec::new();
        payload.extend_from_slice(&0u32.to_be_bytes());
        payload.push(8); // version
        payload.push(0x81); // varint continuation
        payload.push(0x00); // varint final → 128
        payload.push(0x03); // silence varint → 3
                            // tail: freq 2, max_band-1 4, channels-1 1, ms 0, bp 0
        let bits: u16 = (2u16 << 13) | (4u16 << 8) | (1u16 << 4);
        payload.extend_from_slice(&bits.to_be_bytes());
        let sh = StreamHeaderFields::parse(&payload).unwrap();
        assert_eq!(sh.sample_count, 128);
        assert_eq!(sh.beginning_silence, 3);
        assert_eq!(sh.sample_freq_index, 2);
        assert_eq!(sh.sample_rate_hz(), Some(37800));
        assert_eq!(sh.max_band, 5);
        assert_eq!(sh.channels, 2);
    }

    #[test]
    fn rejects_wrong_stream_version() {
        let payload = Spec {
            version: 7,
            ..Spec::default()
        }
        .build();
        assert_eq!(
            StreamHeaderFields::parse(&payload),
            Err(Error::InvalidStreamVersion(7))
        );
    }

    #[test]
    fn rejects_truncated_before_crc() {
        assert_eq!(
            StreamHeaderFields::parse(&[0x00, 0x00, 0x00]),
            Err(Error::UnexpectedEof)
        );
    }

    #[test]
    fn rejects_truncated_before_tail() {
        // CRC + version + two varints but no packed tail bytes.
        let payload = vec![0, 0, 0, 0, 8, 0, 0];
        assert_eq!(
            StreamHeaderFields::parse(&payload),
            Err(Error::UnexpectedEof)
        );
    }

    #[test]
    fn rejects_truncated_mid_varint() {
        // CRC + version + a varint byte with continuation set but no
        // following byte.
        let payload = vec![0, 0, 0, 0, 8, 0x80];
        assert_eq!(
            StreamHeaderFields::parse(&payload),
            Err(Error::UnexpectedEof)
        );
    }
}
