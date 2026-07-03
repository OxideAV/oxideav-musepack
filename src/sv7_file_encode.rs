//! SV7 whole-stream (`.mpc` file) **encode** — the §1 fixed header and
//! the §1.1 continuous audio bit run composed into one raw byte buffer.
//!
//! [`crate::sv7_header_encode`] writes the 200-bit fixed header;
//! [`crate::sv7_stereo_frame_encode::encode_sv7_stereo_frame`] writes one
//! stereo frame **body**. This module joins them into a complete SV7
//! stream:
//!
//! 1. the logical header run (prefix word + the 17 §1 fields), ending at
//!    bit 200;
//! 2. per §1.1 ("the audio data is consumed directly from the
//!    word-swapped bit stream" — one continuous non-byte-aligned bit
//!    run), every frame body back-to-back, starting at the first bit
//!    after header field 17;
//! 3. the §4 32-bit-word byte-swap over the whole logical run (the swap
//!    is one transform over the entire stream — header and body share
//!    the same word grid), zero-padding the trailing partial word.
//!
//! The result begins with the raw `MP+` magic and decodes end-to-end
//! with the whole-file decoder ([`crate::sv7_file_decode`]) — and its
//! header parses with [`crate::sv7_header::Sv7HeaderFields::parse`].
//!
//! # Scope / standing gaps
//!
//! The composed layout is **self-consistent and spec-grounded** (§1
//! fields → §1.1 continuous run → §4 swap), and every bit of it
//! round-trips through the crate's own grounded decode path. What has
//! *not* been validated against third-party material is byte-for-byte
//! interop with externally-encoded files (no SV7 fixture corpus exists
//! under `docs/audio/musepack/`), and §1.1's mention of an 11-bit
//! last-frame-sample field read *from the stream* at the total-sample
//! boundary is not pinned to an exact bit position — this writer carries
//! that quantity in header field 14 only (the field the parser
//! surfaces). The absolute SCF anchor and the M/S-undo arithmetic remain
//! the documented §2.6 DOCS-GAPs; the encode side threads them exactly
//! like the decoders (an `anchor` knob; M/S flags written verbatim).
//!
//! Source-of-record (facts only):
//! `docs/audio/musepack/spec/musepack-headers-and-coding.md` §1 (header
//! fields), §1.1 (continuous non-aligned bit run, no per-frame length
//! prefix), §4 (word swap), §5 (frame bodies). No new format facts —
//! pure composition of the grounded encode sub-walks.

use crate::sv7_bitwriter::Sv7BitWriter;
use crate::sv7_frame_encode::Sv7EncBand;
use crate::sv7_header::Sv7HeaderFields;
use crate::sv7_header_encode::{write_sv7_header_fields, SV7_DEFAULT_VERSION_BYTE};
use crate::sv7_stereo_frame_encode::encode_sv7_stereo_frame;
use crate::{Error, Result};

/// One stereo frame's encode input: the two channels' per-band specs
/// (ascending subband order, both of length `max_band + 1`) and the
/// per-band M/S flags (same length; emitted into the §5.1 header only
/// for bands with a non-zero channel, and only when the stream-wide M/S
/// flag is set).
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Sv7EncStereoFrame {
    /// Left channel band specs, bands `0..=max_band`.
    pub left: Vec<Sv7EncBand>,
    /// Right channel band specs, bands `0..=max_band`.
    pub right: Vec<Sv7EncBand>,
    /// Per-band M/S flags, bands `0..=max_band`.
    pub ms_flags: Vec<bool>,
}

impl Sv7EncStereoFrame {
    /// An all-silent frame covering `band_count` subbands (every band
    /// [`Sv7EncBand::Empty`], no M/S flags set).
    pub fn silent(band_count: usize) -> Self {
        Self {
            left: vec![Sv7EncBand::Empty; band_count],
            right: vec![Sv7EncBand::Empty; band_count],
            ms_flags: vec![false; band_count],
        }
    }
}

/// Encode a complete SV7 `.mpc` stream with the default version byte.
/// See [`encode_sv7_file_with_version`].
///
/// # Errors
///
/// See [`encode_sv7_file_with_version`].
pub fn encode_sv7_file(
    header: &Sv7HeaderFields,
    frames: &[Sv7EncStereoFrame],
    anchor: u8,
) -> Result<Vec<u8>> {
    encode_sv7_file_with_version(header, frames, anchor, SV7_DEFAULT_VERSION_BYTE)
}

/// Encode a complete SV7 `.mpc` stream: the §1 fixed header followed by
/// the §1.1 continuous audio bit run of every frame in `frames`, §4
/// word-swapped into raw on-disk byte order.
///
/// `header` supplies every §1 field; its `frame_count` must equal
/// `frames.len()` and its `mid_side` flag gates the per-band M/S bits in
/// each frame's §5.1 header. Each frame's band vectors must cover
/// exactly `header.max_band + 1` subbands — the §5 band loop the decoder
/// runs is sized by the header's `max_band` (field 4), so a frame with a
/// different band count would not survive its own header. `anchor` is
/// the §2.6 absolute-SCF-anchor GAP knob (pass 0 for the relative
/// convention; it seeds each channel's first coded band exactly as
/// [`crate::sv7_stream::Sv7StreamDecoder`] does on decode).
///
/// # Errors
///
/// - [`Error::UnsupportedVersion`] / [`Error::MaxBandOutOfRange`] /
///   [`Error::HeaderFieldOutOfRange`] from the header layer, including
///   `HeaderFieldOutOfRange("frame_count")` when `header.frame_count`
///   disagrees with `frames.len()`.
/// - [`Error::MaxBandOutOfRange`] for a frame whose band count is not
///   `header.max_band + 1`.
/// - Any frame-body encode error ([`Error::SampleOutOfRange`],
///   [`Error::SymbolNotEncodable`], [`Error::UnsupportedBandType`], …).
pub fn encode_sv7_file_with_version(
    header: &Sv7HeaderFields,
    frames: &[Sv7EncStereoFrame],
    anchor: u8,
    version_byte: u8,
) -> Result<Vec<u8>> {
    if header.frame_count as usize != frames.len() {
        return Err(Error::HeaderFieldOutOfRange("frame_count"));
    }

    let mut writer = Sv7BitWriter::new();
    // §1: the 200-bit fixed header (validates every field).
    write_sv7_header_fields(&mut writer, header, version_byte)?;

    // §1.1: the continuous audio bit run, one frame body after another,
    // starting at the first bit after header field 17.
    let bands = header.max_band as usize + 1;
    for frame in frames {
        if frame.left.len() != bands || frame.right.len() != bands || frame.ms_flags.len() != bands
        {
            let implied = frame
                .left
                .len()
                .max(frame.right.len())
                .max(frame.ms_flags.len())
                .saturating_sub(1)
                .min(u8::MAX as usize) as u8;
            return Err(Error::MaxBandOutOfRange(implied));
        }
        encode_sv7_stereo_frame(
            &mut writer,
            &frame.left,
            &frame.right,
            &frame.ms_flags,
            header.mid_side,
            i32::from(anchor),
        )?;
    }

    // §4: one word-swap over the whole logical run (zero-padding the
    // trailing partial word) yields the raw on-disk stream.
    let logical = writer.finish();
    Ok(crate::sv7_word_swap::word_swap_sv7_body(&logical))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::huffman::{sv7_q3_ctx, Sv7BitReader};
    use crate::sv7_band_decode::SAMPLES_PER_BAND;
    use crate::sv7_header_encode::SV7_HEADER_BITS;
    use crate::sv7_stream::{Sv7StreamDecoder, STEREO_FRAME_PCM_LEN};

    /// A representative test-only M/S undo (not a claim about the GAP
    /// Musepack arithmetic).
    fn test_undo(m: f64, s: f64) -> (f64, f64) {
        (m + s, m - s)
    }

    fn header(frame_count: u32, max_band: u8, mid_side: bool) -> Sv7HeaderFields {
        Sv7HeaderFields {
            frame_count,
            mid_side,
            max_band,
            profile: 10,
            sample_freq_index: 0,
            encoder_version: 0x71,
            ..Default::default()
        }
    }

    fn q3_levels() -> [i32; SAMPLES_PER_BAND] {
        let a: Vec<i32> = sv7_q3_ctx(0).iter().map(|e| e.value as i32).collect();
        core::array::from_fn(|i| a[i % a.len()])
    }

    /// Position a reader over the word-swapped whole file at the first
    /// body bit (bit 200), returning the swapped buffer.
    fn swapped_body_reader(raw: &[u8]) -> Vec<u8> {
        crate::sv7_word_swap::word_swap_sv7_body(raw)
    }

    fn skip_header_bits(reader: &mut Sv7BitReader<'_>) {
        let mut left = SV7_HEADER_BITS;
        while left > 0 {
            let n = left.min(16) as u8;
            reader.read_bits(n).expect("header span present");
            left -= u64::from(n);
        }
    }

    #[test]
    fn silent_file_header_parses_and_body_decodes_silent() {
        let hdr = header(2, 3, false);
        let frames = vec![Sv7EncStereoFrame::silent(4); 2];
        let raw = encode_sv7_file(&hdr, &frames, 0).expect("encode");

        // Header round-trips off the raw bytes.
        assert_eq!(&raw[..3], b"MP+");
        assert_eq!(Sv7HeaderFields::parse(&raw).unwrap(), hdr);
        assert_eq!(raw.len() % 4, 0, "word-aligned on-disk length");

        // Body decodes from bit 200 of the swapped stream.
        let swapped = swapped_body_reader(&raw);
        let mut reader = Sv7BitReader::new(&swapped);
        skip_header_bits(&mut reader);
        let mut dec = Sv7StreamDecoder::from_header(&hdr, 0, test_undo).unwrap();
        let pcm = dec.decode_frames(&mut reader, 2).unwrap();
        assert_eq!(pcm.len(), 2 * STEREO_FRAME_PCM_LEN);
        assert!(pcm.iter().all(|&s| s == 0.0));
    }

    #[test]
    fn coded_file_pcm_matches_body_only_reference() {
        // Two frames mixing empty / CNS / coded bands, stream M/S on.
        let coded = |scf0: i32| Sv7EncBand::Coded {
            band_type: 3,
            ctx: 0,
            scf: [scf0, scf0 + 1, scf0 + 1],
            levels: q3_levels(),
        };
        let frame_a = Sv7EncStereoFrame {
            left: vec![coded(7), Sv7EncBand::Cns, Sv7EncBand::Empty],
            right: vec![coded(9), Sv7EncBand::Empty, Sv7EncBand::Cns],
            ms_flags: vec![true, false, false],
        };
        let frame_b = Sv7EncStereoFrame {
            left: vec![Sv7EncBand::Empty, coded(12), Sv7EncBand::Cns],
            right: vec![coded(5), Sv7EncBand::Cns, Sv7EncBand::Empty],
            ms_flags: vec![false, true, true],
        };
        let hdr = header(2, 2, true);
        let frames = vec![frame_a.clone(), frame_b.clone()];
        let anchor = 4u8;
        let raw = encode_sv7_file(&hdr, &frames, anchor).expect("encode");

        // Whole-file path.
        let swapped = swapped_body_reader(&raw);
        let mut reader = Sv7BitReader::new(&swapped);
        skip_header_bits(&mut reader);
        let mut dec = Sv7StreamDecoder::from_header(&hdr, anchor, test_undo).unwrap();
        let file_pcm = dec.decode_frames(&mut reader, 2).unwrap();

        // Reference: the same bodies encoded standalone (no header, no
        // whole-file swap), decoded by the same driver.
        let mut w = Sv7BitWriter::new();
        for f in &frames {
            encode_sv7_stereo_frame(
                &mut w,
                &f.left,
                &f.right,
                &f.ms_flags,
                hdr.mid_side,
                i32::from(anchor),
            )
            .unwrap();
        }
        let mut body = w.finish();
        body.extend_from_slice(&[0, 0, 0, 0]);
        let mut ref_reader = Sv7BitReader::new(&body);
        let mut ref_dec = Sv7StreamDecoder::from_header(&hdr, anchor, test_undo).unwrap();
        let ref_pcm = ref_dec.decode_frames(&mut ref_reader, 2).unwrap();

        assert_eq!(file_pcm.len(), 2 * STEREO_FRAME_PCM_LEN);
        assert_eq!(file_pcm, ref_pcm);
        assert!(
            file_pcm.iter().any(|&s| s != 0.0),
            "coded audio is non-silent"
        );
    }

    #[test]
    fn body_starts_immediately_after_header_field_17() {
        // A single frame whose first body bits are non-zero (band-0 Res
        // raw 4-bit values 3, 3): prove they land at bit 200 exactly.
        let coded = Sv7EncBand::Coded {
            band_type: 3,
            ctx: 0,
            scf: [7, 7, 7],
            levels: q3_levels(),
        };
        let hdr = header(1, 1, false);
        let frames = vec![Sv7EncStereoFrame {
            left: vec![coded.clone(), Sv7EncBand::Empty],
            right: vec![coded, Sv7EncBand::Empty],
            ms_flags: vec![false, false],
        }];
        let raw = encode_sv7_file(&hdr, &frames, 0).unwrap();
        let swapped = swapped_body_reader(&raw);
        let mut reader = Sv7BitReader::new(&swapped);
        skip_header_bits(&mut reader);
        // §5.1 band 0: left Res then right Res as raw 4-bit values.
        assert_eq!(reader.read_bits(4).unwrap(), 3);
        assert_eq!(reader.read_bits(4).unwrap(), 3);
    }

    #[test]
    fn rejects_frame_count_mismatch() {
        let hdr = header(2, 1, false);
        let frames = vec![Sv7EncStereoFrame::silent(2)];
        assert_eq!(
            encode_sv7_file(&hdr, &frames, 0),
            Err(Error::HeaderFieldOutOfRange("frame_count")),
        );
    }

    #[test]
    fn rejects_frame_band_count_disagreeing_with_max_band() {
        let hdr = header(1, 3, false); // decoder will walk 4 bands
        let frames = vec![Sv7EncStereoFrame::silent(2)];
        assert_eq!(
            encode_sv7_file(&hdr, &frames, 0),
            Err(Error::MaxBandOutOfRange(1)),
        );
    }

    #[test]
    fn propagates_header_validation_failure() {
        let mut hdr = header(0, 5, false);
        hdr.profile = 16;
        assert_eq!(
            encode_sv7_file(&hdr, &[], 0),
            Err(Error::HeaderFieldOutOfRange("profile")),
        );
    }

    #[test]
    fn zero_frame_file_is_just_the_header() {
        let hdr = header(0, 5, false);
        let raw = encode_sv7_file(&hdr, &[], 0).unwrap();
        assert_eq!(raw.len(), crate::sv7_header_encode::SV7_HEADER_DISK_LEN);
        assert_eq!(Sv7HeaderFields::parse(&raw).unwrap(), hdr);
    }

    #[test]
    fn silent_helper_builds_matching_lengths() {
        let f = Sv7EncStereoFrame::silent(7);
        assert_eq!(f.left.len(), 7);
        assert_eq!(f.right.len(), 7);
        assert_eq!(f.ms_flags.len(), 7);
        assert!(f.left.iter().all(|b| *b == Sv7EncBand::Empty));
    }
}
