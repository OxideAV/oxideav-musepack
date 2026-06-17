//! SV7 (`MP+`) fixed-header field-map decoder.
//!
//! The SV7 fixed header is the analogue of the SV8 `SH` packet: it
//! carries every stream-wide parameter the per-frame decode loop
//! presumes — the audio frame count (and from it the total sample
//! count), the stream-wide intensity / mid-side flags, the highest
//! coded subband (`max_band`), the encoder profile / link, the
//! sample-rate index, the ReplayGain title / album gain+peak quad, the
//! true-gapless flag plus the 11-bit last-frame valid-sample count, the
//! fast-seek flag, and the encoder version byte.
//!
//! Up to this module [`crate::framing::SV7Header`] recognised only the
//! `MP+` magic + version byte and handed back the rest of the header as
//! an opaque slice. This module decodes that fixed header into
//! [`Sv7HeaderFields`].
//!
//! Source-of-record (facts only):
//!
//! - `docs/audio/musepack/spec/musepack-headers-and-coding.md` §1 — the
//!   SV7 fixed-header field-map: the exact field order and bit widths
//!   (frame count 32, intensity 1, mid/side 1, max_band 6, profile 4,
//!   link 2, sample-freq index 2, max-level 16, the four ReplayGain
//!   16-bit fields, true-gapless 1, last-frame-samples 11, fast-seek 1,
//!   reserved 19, encoder version 8), the "channels is always 2" and
//!   "block power fixed at 0" derived facts, and the header sanity gate
//!   (`1 ≤ max_band ≤ 31`).
//! - `docs/audio/musepack/spec/musepack-headers-and-coding.md` §4 — the
//!   SV7 32-bit-word framing: SV7 bytes are pre-swapped in 32-bit
//!   little-endian word units (each aligned 4-byte group reversed)
//!   before the MSB-first bit reader sees them, and the header field
//!   reads begin at the first bit *after* the 4-byte `MP+`+version
//!   prefix (i.e. at bit 32 of the word-swapped buffer).
//! - `docs/audio/musepack/musepack-sv7-sv8-spec.md` §1 — the
//!   {44100, 48000, 37800, 32000} Hz sample-rate table (shared with
//!   SV8) and the 1152-samples-per-frame geometry.

use crate::huffman::Sv7BitReader;
use crate::{Error, Result};

/// The SV7 sample-rate table — index → Hz.
///
/// Per `spec/musepack-headers-and-coding.md` §1 (field 7) the 2-bit
/// sample-freq index selects one of `{44100, 48000, 37800, 32000}` Hz
/// — the same four rates SV8 carries.
pub const SV7_SAMPLE_RATES: [u32; 4] = [44100, 48000, 37800, 32000];

/// PCM samples per SV7 frame per channel (`musepack-sv7-sv8-spec.md`
/// §1: `36 × 32 = 1152`).
pub const SV7_SAMPLES_PER_FRAME: u64 = 1152;

/// SV7 is stereo-only: the channel count is fixed at 2
/// (`spec/musepack-headers-and-coding.md` §1, "derived facts").
pub const SV7_CHANNELS: u8 = 2;

/// Inclusive upper bound on `max_band` from the Layer-II 32-subband
/// heritage (`spec/musepack-headers-and-coding.md` §1 sanity gate:
/// `1 ≤ max_band ≤ 31`).
pub const SV7_MAX_BAND_INCLUSIVE: u8 = 31;

/// Decoded SV7 fixed-header fields (§1).
///
/// Every multi-bit field has been un-packed from the word-swapped
/// header bitstream; the four ReplayGain quantities are surfaced as the
/// raw 16-bit values the header carries (later rescaling is a separate
/// concern). Fields a conformant decoder reads but does not use
/// (`max_level`, `link`, `reserved`) are surfaced verbatim rather than
/// dropped.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Sv7HeaderFields {
    /// Number of audio frames (§1, field 1). Each frame carries
    /// [`SV7_SAMPLES_PER_FRAME`] samples per channel.
    pub frame_count: u32,
    /// Intensity-stereo flag (§1, field 2). The spec expects 0
    /// (unused in practice); surfaced verbatim.
    pub intensity_stereo: bool,
    /// Stream-wide mid/side enable (§1, field 3). When set, per-band
    /// M/S flags appear in frames.
    pub mid_side: bool,
    /// Highest subband index actually coded (§1, field 4).
    pub max_band: u8,
    /// Encoder profile / quality preset (§1, field 5; informational).
    pub profile: u8,
    /// Link / stream-link indicator (§1, field 6; skipped by the
    /// decoder, surfaced verbatim).
    pub link: u8,
    /// Index into [`SV7_SAMPLE_RATES`] (§1, field 7).
    pub sample_freq_index: u8,
    /// Maximum input-PCM level (§1, field 8; skipped by the decoder,
    /// surfaced verbatim).
    pub max_level: u16,
    /// ReplayGain title gain, raw (§1, field 9).
    pub title_gain: u16,
    /// ReplayGain title peak, raw (§1, field 10).
    pub title_peak: u16,
    /// ReplayGain album gain, raw (§1, field 11).
    pub album_gain: u16,
    /// ReplayGain album peak, raw (§1, field 12).
    pub album_peak: u16,
    /// True-gapless flag (§1, field 13). When set,
    /// [`Self::last_frame_samples`] is meaningful.
    pub true_gapless: bool,
    /// Valid sample count in the final frame (§1, field 14). A value
    /// of 0 means the last frame is a full [`SV7_SAMPLES_PER_FRAME`].
    pub last_frame_samples: u16,
    /// Fast-seek-enabled indicator (§1, field 15).
    pub fast_seek: bool,
    /// Reserved / unused 19 bits (§1, field 16; surfaced verbatim).
    pub reserved: u32,
    /// Encoder version byte (§1, field 17).
    pub encoder_version: u8,
}

impl Sv7HeaderFields {
    /// Decode the SV7 fixed header from `input`, where `input` begins
    /// at the `MP+` magic (the full stream prefix, or at least the
    /// fixed-header span).
    ///
    /// Per §4 the SV7 byte stream is framed in 32-bit little-endian
    /// word units that are byte-swapped before the MSB-first bit
    /// reader sees them, and per §1 the field reads start at the first
    /// bit after the 4-byte `MP+`+version prefix. This method applies
    /// the word-swap to the header span, then walks the 17 fields in
    /// order from bit 32.
    ///
    /// # Errors
    ///
    /// - [`Error::InvalidMagic`] if `input` does not start with the
    ///   `MP+` magic.
    /// - [`Error::UnsupportedVersion`] if the version byte's low nibble
    ///   is not 7.
    /// - [`Error::UnexpectedEof`] if the input is shorter than the
    ///   21-byte field span requires.
    /// - [`Error::MaxBandOutOfRange`] if the decoded `max_band` falls
    ///   outside the §1 sanity gate `1 ≤ max_band ≤ 31`.
    pub fn parse(input: &[u8]) -> Result<Self> {
        use crate::framing::{SV7_MAGIC, SV7_VERSION_NIBBLE};

        // The field span is 168 bits (21 bytes) and begins after the
        // 4-byte prefix, so the logical header is 25 bytes. Because §4
        // word-swaps in 32-bit groups, the last logical byte (the
        // encoder-version, byte 24) is carried in the seventh on-disk
        // word, so the parse needs the full word-aligned 28-byte span.
        const PREFIX_LEN: usize = SV7_MAGIC.len() + 1; // MP+ + version
        const FIELD_BYTES: usize = 21;
        const LOGICAL_LEN: usize = PREFIX_LEN + FIELD_BYTES; // 25
                                                             // Round up to the 32-bit word grid that the swap operates on.
        const HEADER_LEN: usize = LOGICAL_LEN.next_multiple_of(4); // 28

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
        if input.len() < HEADER_LEN {
            return Err(Error::UnexpectedEof);
        }

        // §4: lay the header bytes into 32-bit little-endian word units
        // with an in-place byte-swap (each aligned 4-byte group
        // reversed). Pad the final partial word with zeros so the swap
        // is well-defined; the field reader never consumes past the
        // declared 200-bit (prefix + fields) span.
        let swapped = word_swap_sv7(&input[..HEADER_LEN]);

        let mut reader = Sv7BitReader::new(&swapped);

        // §1: skip the 4-byte `MP+`+version prefix word (now the first
        // word of the swapped buffer) and begin at bit 32.
        let _prefix = read_u32(&mut reader)?;

        // Field 1: frame count, 32 bits, read as two 16-bit halves,
        // high half first (§1, field 1; §4 — 32-bit quantities are
        // assembled from two 16-bit reads).
        let frame_count = read_u32(&mut reader)?;

        // Fields 2..7: the packed control word.
        let intensity_stereo = reader.read_bits(1)? != 0; // field 2
        let mid_side = reader.read_bits(1)? != 0; // field 3
        let max_band = reader.read_bits(6)? as u8; // field 4
        let profile = reader.read_bits(4)? as u8; // field 5
        let link = reader.read_bits(2)? as u8; // field 6
        let sample_freq_index = reader.read_bits(2)? as u8; // field 7

        // Fields 8..12: max-level + the four 16-bit ReplayGain values.
        let max_level = reader.read_bits(16)?; // field 8
        let title_gain = reader.read_bits(16)?; // field 9
        let title_peak = reader.read_bits(16)?; // field 10
        let album_gain = reader.read_bits(16)?; // field 11
        let album_peak = reader.read_bits(16)?; // field 12

        // Fields 13..17: gapless / seek flags, reserved, encoder ver.
        let true_gapless = reader.read_bits(1)? != 0; // field 13
        let last_frame_samples = reader.read_bits(11)?; // field 14
        let fast_seek = reader.read_bits(1)? != 0; // field 15
                                                   // Reserved 19 bits: split into 16 + 3 (read_bits caps at 16).
        let reserved_hi = reader.read_bits(16)? as u32; // field 16a
        let reserved_lo = reader.read_bits(3)? as u32; // field 16b
        let reserved = (reserved_hi << 3) | reserved_lo;
        let encoder_version = reader.read_bits(8)? as u8; // field 17

        // §1 sanity gate: 1 ≤ max_band ≤ 31. (Channels is always 2 and
        // the sample-freq index always maps to a non-zero rate, so
        // max_band is the only field that can fail the gate.)
        if !(1..=SV7_MAX_BAND_INCLUSIVE).contains(&max_band) {
            return Err(Error::MaxBandOutOfRange(max_band));
        }

        Ok(Self {
            frame_count,
            intensity_stereo,
            mid_side,
            max_band,
            profile,
            link,
            sample_freq_index,
            max_level,
            title_gain,
            title_peak,
            album_gain,
            album_peak,
            true_gapless,
            last_frame_samples,
            fast_seek,
            reserved,
            encoder_version,
        })
    }

    /// The decoded sample rate in Hz, or `None` if
    /// [`Self::sample_freq_index`] is outside the four defined indices
    /// (§1: only `0..=3` map to a rate).
    pub fn sample_rate_hz(&self) -> Option<u32> {
        SV7_SAMPLE_RATES
            .get(self.sample_freq_index as usize)
            .copied()
    }

    /// Channel count — always [`SV7_CHANNELS`] (SV7 is stereo-only,
    /// §1 derived fact).
    pub fn channels(&self) -> u8 {
        SV7_CHANNELS
    }

    /// Total decoded sample count per channel, before any gapless /
    /// synth-delay adjustment: `frame_count × 1152` (§1, field 1).
    pub fn total_samples(&self) -> u64 {
        u64::from(self.frame_count) * SV7_SAMPLES_PER_FRAME
    }
}

/// Read a 32-bit big-endian-assembled value from two 16-bit MSB-first
/// reads, high half first (§4 — the reader caps a single fixed read at
/// 16 bits, so 32-bit quantities are assembled from two halves).
fn read_u32(reader: &mut Sv7BitReader<'_>) -> Result<u32> {
    let hi = reader.read_bits(16)? as u32;
    let lo = reader.read_bits(16)? as u32;
    Ok((hi << 16) | lo)
}

/// Lay `bytes` into 32-bit little-endian word units with an in-place
/// byte-swap: each aligned 4-byte group is reversed (§4). A final
/// partial group is zero-padded to a full word before being reversed,
/// so the returned buffer length is rounded up to a multiple of 4.
fn word_swap_sv7(bytes: &[u8]) -> Vec<u8> {
    let words = bytes.len().div_ceil(4);
    let mut out = vec![0u8; words * 4];
    for w in 0..words {
        let mut word = [0u8; 4];
        for (j, slot) in word.iter_mut().enumerate() {
            let src = w * 4 + j;
            if src < bytes.len() {
                *slot = bytes[src];
            }
        }
        word.reverse();
        out[w * 4..w * 4 + 4].copy_from_slice(&word);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::framing::{SV7_MAGIC, SV7_VERSION_NIBBLE};

    /// Builder for a logical SV7 fixed header. Fields are serialised
    /// MSB-first into the 168-bit field span, then the whole 25-byte
    /// header (prefix + span) is word-swapped so [`Sv7HeaderFields::parse`]
    /// recovers the same values.
    #[derive(Clone)]
    struct Spec {
        frame_count: u32,
        intensity_stereo: bool,
        mid_side: bool,
        max_band: u8,
        profile: u8,
        link: u8,
        sample_freq_index: u8,
        max_level: u16,
        title_gain: u16,
        title_peak: u16,
        album_gain: u16,
        album_peak: u16,
        true_gapless: bool,
        last_frame_samples: u16,
        fast_seek: bool,
        reserved: u32,
        encoder_version: u8,
    }

    impl Default for Spec {
        fn default() -> Self {
            Self {
                frame_count: 0,
                intensity_stereo: false,
                mid_side: false,
                max_band: 20,
                profile: 10,
                link: 0,
                sample_freq_index: 0,
                max_level: 0,
                title_gain: 0,
                title_peak: 0,
                album_gain: 0,
                album_peak: 0,
                true_gapless: false,
                last_frame_samples: 0,
                fast_seek: false,
                reserved: 0,
                encoder_version: 0,
            }
        }
    }

    /// A tiny MSB-first bit packer for building test headers.
    struct BitWriter {
        acc: u64,
        nbits: u32,
        out: Vec<u8>,
    }

    impl BitWriter {
        fn new() -> Self {
            Self {
                acc: 0,
                nbits: 0,
                out: Vec::new(),
            }
        }

        fn put(&mut self, value: u32, width: u32) {
            debug_assert!(width <= 32);
            // Mask `value` to `width` bits and append MSB-first.
            let masked = if width == 32 {
                value as u64
            } else {
                (value as u64) & ((1u64 << width) - 1)
            };
            self.acc = (self.acc << width) | masked;
            self.nbits += width;
            while self.nbits >= 8 {
                self.nbits -= 8;
                self.out.push((self.acc >> self.nbits) as u8);
            }
        }

        fn finish(mut self) -> Vec<u8> {
            if self.nbits > 0 {
                let pad = 8 - self.nbits;
                self.out.push((self.acc << pad) as u8);
                self.nbits = 0;
            }
            self.out
        }
    }

    impl Spec {
        /// Serialise into a complete word-swapped SV7 header buffer.
        fn build(&self) -> Vec<u8> {
            let mut bw = BitWriter::new();
            // Field 1: frame count as two 16-bit halves, high first.
            bw.put(self.frame_count >> 16, 16);
            bw.put(self.frame_count & 0xFFFF, 16);
            bw.put(self.intensity_stereo as u32, 1);
            bw.put(self.mid_side as u32, 1);
            bw.put(self.max_band as u32, 6);
            bw.put(self.profile as u32, 4);
            bw.put(self.link as u32, 2);
            bw.put(self.sample_freq_index as u32, 2);
            bw.put(self.max_level as u32, 16);
            bw.put(self.title_gain as u32, 16);
            bw.put(self.title_peak as u32, 16);
            bw.put(self.album_gain as u32, 16);
            bw.put(self.album_peak as u32, 16);
            bw.put(self.true_gapless as u32, 1);
            bw.put(self.last_frame_samples as u32, 11);
            bw.put(self.fast_seek as u32, 1);
            bw.put(self.reserved, 19);
            bw.put(self.encoder_version as u32, 8);
            let field_bytes = bw.finish();
            assert_eq!(field_bytes.len(), 21, "field span must be 21 bytes");

            // §4: `parse` validates the `MP+` magic on the raw disk
            // bytes (natural order) and then word-swaps the whole
            // header span for the bit reader, reading fields from
            // bit 32 (after the prefix word). The prefix occupies its
            // own 4-byte word, so the on-disk field span is the
            // word-swap of the logical MSB-first field bitstream. We
            // build the on-disk header as natural prefix + swapped
            // field span.
            let field_disk = super::word_swap_sv7(&field_bytes);
            let mut disk = Vec::with_capacity(4 + field_disk.len());
            disk.extend_from_slice(&SV7_MAGIC);
            disk.push(SV7_VERSION_NIBBLE);
            disk.extend_from_slice(&field_disk);
            disk
        }
    }

    #[test]
    fn parses_canonical_header() {
        let spec = Spec {
            frame_count: 0x0001_2345,
            mid_side: true,
            max_band: 25,
            profile: 11,
            link: 2,
            sample_freq_index: 0,
            max_level: 0xBEEF,
            title_gain: 0x1111,
            title_peak: 0x2222,
            album_gain: 0x3333,
            album_peak: 0x4444,
            true_gapless: true,
            last_frame_samples: 0x2BC, // 700, < 2048
            fast_seek: true,
            reserved: 0x5_AAAA & 0x7_FFFF,
            encoder_version: 0x71,
            ..Default::default()
        };
        let buf = spec.build();
        let h = Sv7HeaderFields::parse(&buf).expect("valid header");

        assert_eq!(h.frame_count, 0x0001_2345);
        assert!(!h.intensity_stereo);
        assert!(h.mid_side);
        assert_eq!(h.max_band, 25);
        assert_eq!(h.profile, 11);
        assert_eq!(h.link, 2);
        assert_eq!(h.sample_freq_index, 0);
        assert_eq!(h.max_level, 0xBEEF);
        assert_eq!(h.title_gain, 0x1111);
        assert_eq!(h.title_peak, 0x2222);
        assert_eq!(h.album_gain, 0x3333);
        assert_eq!(h.album_peak, 0x4444);
        assert!(h.true_gapless);
        assert_eq!(h.last_frame_samples, 0x2BC);
        assert!(h.fast_seek);
        assert_eq!(h.reserved, 0x5_AAAA & 0x7_FFFF);
        assert_eq!(h.encoder_version, 0x71);
    }

    #[test]
    fn derived_facts() {
        let spec = Spec {
            frame_count: 100,
            sample_freq_index: 2, // 37800
            ..Default::default()
        };
        let h = Sv7HeaderFields::parse(&spec.build()).unwrap();
        assert_eq!(h.channels(), 2);
        assert_eq!(h.sample_rate_hz(), Some(37800));
        assert_eq!(h.total_samples(), 100 * 1152);
    }

    #[test]
    fn each_sample_rate_index_maps() {
        for (idx, &rate) in SV7_SAMPLE_RATES.iter().enumerate() {
            let spec = Spec {
                sample_freq_index: idx as u8,
                ..Default::default()
            };
            let h = Sv7HeaderFields::parse(&spec.build()).unwrap();
            assert_eq!(h.sample_rate_hz(), Some(rate));
        }
    }

    #[test]
    fn rejects_bad_magic() {
        let mut buf = Spec::default().build();
        buf[0] ^= 0xFF; // corrupt the first magic byte ('M')
        assert_eq!(Sv7HeaderFields::parse(&buf), Err(Error::InvalidMagic));
    }

    #[test]
    fn rejects_short_input() {
        assert_eq!(Sv7HeaderFields::parse(&[]), Err(Error::UnexpectedEof));
        // Valid prefix but truncated field span.
        let mut buf = Spec::default().build();
        buf.truncate(12);
        assert_eq!(Sv7HeaderFields::parse(&buf), Err(Error::UnexpectedEof));
    }

    #[test]
    fn rejects_max_band_zero() {
        let spec = Spec {
            max_band: 0,
            ..Default::default()
        };
        assert_eq!(
            Sv7HeaderFields::parse(&spec.build()),
            Err(Error::MaxBandOutOfRange(0))
        );
    }

    #[test]
    fn accepts_max_band_boundaries() {
        for mb in [1u8, SV7_MAX_BAND_INCLUSIVE] {
            let spec = Spec {
                max_band: mb,
                ..Default::default()
            };
            let h = Sv7HeaderFields::parse(&spec.build()).unwrap();
            assert_eq!(h.max_band, mb);
        }
    }

    #[test]
    fn word_swap_reverses_each_group() {
        let input = [1u8, 2, 3, 4, 5, 6, 7, 8];
        let out = word_swap_sv7(&input);
        assert_eq!(out, vec![4, 3, 2, 1, 8, 7, 6, 5]);
    }

    #[test]
    fn word_swap_zero_pads_partial_group() {
        let input = [1u8, 2, 3, 4, 9];
        let out = word_swap_sv7(&input);
        // Second group is [9,0,0,0] reversed → [0,0,0,9].
        assert_eq!(out, vec![4, 3, 2, 1, 0, 0, 0, 9]);
    }
}
