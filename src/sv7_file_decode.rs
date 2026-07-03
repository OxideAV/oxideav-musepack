//! SV7 whole-stream (`.mpc` file) **decode** — raw bytes in, header +
//! interleaved PCM out.
//!
//! This is the SV7 counterpart of [`crate::sv8_decode`] and the missing
//! integration layer above [`crate::sv7_stream::Sv7StreamDecoder`]: it
//! owns the *whole-stream positioning* that the stream driver leaves to
//! its caller.
//!
//! Layout walked (all grounded in
//! `docs/audio/musepack/spec/musepack-headers-and-coding.md`):
//!
//! 1. **§1 fixed header** — parsed off the raw bytes
//!    ([`crate::sv7_header::Sv7HeaderFields::parse`]): magic, the 17
//!    fields, sanity gate.
//! 2. **§4 word swap** — one 32-bit-LE-word byte-swap over the whole
//!    stream (header and body share the word grid).
//! 3. **§1.1 continuous audio bit run** — the frame bodies begin at the
//!    first bit after header field 17 (bit 200 of the swapped stream;
//!    §1 fixes the field span at 168 bits after the 4-byte prefix) and
//!    run back-to-back with no per-frame length prefix. Exactly
//!    `frame_count` (§1 field 1) frames are decoded.
//! 4. **Gapless trim** — when the §1 true-gapless flag (field 13) is
//!    set and the last-frame valid-sample count (field 14) is non-zero,
//!    the final frame contributes only that many samples per channel
//!    (`0` means a full 1152).
//!
//! # Scope / standing gaps
//!
//! Decoding is exact for streams composed per
//! [`crate::sv7_file_encode`] (round-trip proven in the tests). For
//! externally-encoded files the body positioning follows the most direct
//! reading of §1/§1.1 (the continuous run starts immediately after field
//! 17), but no in-repo SV7 fixture corpus exists to cross-validate that
//! byte-for-byte, and §1.1's in-stream 11-bit last-frame-sample read is
//! not pinned to an exact bit position (this decoder uses header field
//! 14, the quantity the parser surfaces). The absolute SCF anchor and
//! the M/S-undo arithmetic remain the documented §2.6 DOCS-GAPs,
//! threaded as the same `anchor` / `undo` knobs the stream driver takes.

use crate::huffman::Sv7BitReader;
use crate::sv7_header::{Sv7HeaderFields, SV7_SAMPLES_PER_FRAME};
use crate::sv7_header_encode::SV7_HEADER_BITS;
use crate::sv7_stream::Sv7StreamDecoder;
use crate::{Error, Result};

/// A fully decoded SV7 stream: the parsed §1 header and the interleaved
/// `L, R, …` PCM of every frame (gapless-trimmed per §1 fields 13/14).
#[derive(Debug, Clone, PartialEq)]
pub struct Sv7DecodedFile {
    /// The parsed §1 fixed-header fields.
    pub header: Sv7HeaderFields,
    /// Frames decoded (equals `header.frame_count` on success).
    pub frames_decoded: u64,
    /// Interleaved stereo PCM, `L, R, …` — `2 ×` the per-channel total
    /// after the gapless trim. Relative loudness (the absolute SCF
    /// anchor is GAP).
    pub pcm: Vec<f64>,
}

/// Decode a complete SV7 `.mpc` stream from `bytes`.
///
/// `anchor` is the §2.6 absolute-SCF-anchor GAP knob and `undo` the
/// §2.6 M/S-undo arithmetic GAP closure — the same two knobs
/// [`Sv7StreamDecoder`] threads (pass `0` and the identity-of-choice
/// until the docs pin them).
///
/// # Errors
///
/// - [`Error::InvalidMagic`] / [`Error::UnsupportedVersion`] /
///   [`Error::MaxBandOutOfRange`] / [`Error::UnexpectedEof`] from the
///   §1 header parse.
/// - [`Error::UnexpectedEof`] if the audio run ends before
///   `frame_count` frames were decoded (truncated file).
/// - [`Error::HeaderFieldOutOfRange`]`("last_frame_samples")` if the
///   header declares true-gapless with a last-frame sample count above
///   the 1152-sample frame geometry (a count the frame cannot carry).
/// - Any frame-body decode error (no-match VLC, out-of-range band-type
///   / SCFI, reader starvation mid-frame).
pub fn decode_sv7_file<U>(bytes: &[u8], anchor: u8, undo: U) -> Result<Sv7DecodedFile>
where
    U: Fn(f64, f64) -> (f64, f64),
{
    // 1. §1 fixed header off the raw bytes.
    let header = Sv7HeaderFields::parse(bytes)?;

    // Gate the gapless trim before decoding anything: a last-frame
    // sample count above the frame geometry cannot be honoured.
    let last = u64::from(header.last_frame_samples);
    if header.true_gapless && last > SV7_SAMPLES_PER_FRAME {
        return Err(Error::HeaderFieldOutOfRange("last_frame_samples"));
    }

    // 2. §4: one word-swap over the whole stream. Then append one word
    // of zero slack: the entropy decoder peeks 16 bits ahead of every
    // VLC ([`Sv7BitReader::peek16`]), so a code that ends within the
    // final 15 bits of the run would otherwise starve the lookahead.
    // This is reader slack only (the frame loop is bounded by the §1
    // frame count, so the pad bits are never decoded as data).
    let mut swapped = crate::sv7_word_swap::word_swap_sv7_body(bytes);
    swapped.extend_from_slice(&[0u8; 4]);
    let mut reader = Sv7BitReader::new(&swapped);

    // 3. §1.1: the audio run starts at the first bit after field 17.
    skip_bits(&mut reader, SV7_HEADER_BITS)?;

    let mut decoder = Sv7StreamDecoder::from_header(&header, anchor, undo)?;
    let frame_count = u64::from(header.frame_count);
    let mut pcm = decoder.decode_frames(&mut reader, frame_count)?;
    let frames_decoded = decoder.frames_decoded();
    if frames_decoded < frame_count {
        // The continuous run ran out before the declared frame count —
        // a truncated file (the clean stop `decode_frames` allows is
        // only clean for an open-ended caller; the header told us
        // exactly how many frames to expect).
        return Err(Error::UnexpectedEof);
    }

    // 4. §1 fields 13/14: gapless trim of the final frame.
    pcm.truncate((2 * header.effective_total_samples()) as usize);

    Ok(Sv7DecodedFile {
        header,
        frames_decoded,
        pcm,
    })
}

/// Advance `reader` by `n` bits (the reader's single-read cap is 16).
fn skip_bits(reader: &mut Sv7BitReader<'_>, mut n: u64) -> Result<()> {
    while n > 0 {
        let step = n.min(16) as u8;
        reader.read_bits(step)?;
        n -= u64::from(step);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::huffman::sv7_q3_ctx;
    use crate::sv7_band_decode::SAMPLES_PER_BAND;
    use crate::sv7_file_encode::{encode_sv7_file, Sv7EncStereoFrame};
    use crate::sv7_frame_encode::Sv7EncBand;
    use crate::sv7_stream::STEREO_FRAME_PCM_LEN;

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
            sample_freq_index: 1,
            encoder_version: 0x71,
            ..Default::default()
        }
    }

    fn q3_levels() -> [i32; SAMPLES_PER_BAND] {
        let a: Vec<i32> = sv7_q3_ctx(0).iter().map(|e| e.value as i32).collect();
        core::array::from_fn(|i| a[i % a.len()])
    }

    fn coded(band_type: i8, scf0: i32) -> Sv7EncBand {
        Sv7EncBand::Coded {
            band_type,
            ctx: 0,
            scf: [scf0, scf0 + 1, scf0],
            levels: q3_levels(),
        }
    }

    /// Two mixed frames over three bands, stream M/S on.
    fn busy_frames() -> Vec<Sv7EncStereoFrame> {
        vec![
            Sv7EncStereoFrame {
                left: vec![coded(3, 7), Sv7EncBand::Cns, Sv7EncBand::Empty],
                right: vec![coded(3, 9), Sv7EncBand::Empty, Sv7EncBand::Cns],
                ms_flags: vec![true, false, false],
            },
            Sv7EncStereoFrame {
                left: vec![Sv7EncBand::Empty, coded(3, 12), Sv7EncBand::Cns],
                right: vec![coded(3, 5), Sv7EncBand::Cns, Sv7EncBand::Empty],
                ms_flags: vec![false, true, true],
            },
        ]
    }

    #[test]
    fn silent_file_round_trips() {
        let hdr = header(2, 4, false);
        let frames = vec![Sv7EncStereoFrame::silent(5); 2];
        let raw = encode_sv7_file(&hdr, &frames, 0).unwrap();
        let out = decode_sv7_file(&raw, 0, test_undo).unwrap();
        assert_eq!(out.header, hdr);
        assert_eq!(out.frames_decoded, 2);
        assert_eq!(out.pcm.len(), 2 * STEREO_FRAME_PCM_LEN);
        assert!(out.pcm.iter().all(|&s| s == 0.0));
    }

    #[test]
    fn coded_file_round_trips_and_matches_stream_driver() {
        let hdr = header(2, 2, true);
        let frames = busy_frames();
        let anchor = 4u8;
        let raw = encode_sv7_file(&hdr, &frames, anchor).unwrap();
        let out = decode_sv7_file(&raw, anchor, test_undo).unwrap();
        assert_eq!(out.header, hdr);
        assert_eq!(out.frames_decoded, 2);
        assert_eq!(out.pcm.len(), 2 * STEREO_FRAME_PCM_LEN);
        assert!(out.pcm.iter().any(|&s| s != 0.0));

        // Reference: the same file through the manual pipeline (swap +
        // lookahead slack + skip 200 bits + stream driver).
        let mut swapped = crate::sv7_word_swap::word_swap_sv7_body(&raw);
        swapped.extend_from_slice(&[0u8; 4]);
        let mut reader = Sv7BitReader::new(&swapped);
        skip_bits(&mut reader, SV7_HEADER_BITS).unwrap();
        let mut dec = Sv7StreamDecoder::from_header(&hdr, anchor, test_undo).unwrap();
        let ref_pcm = dec.decode_frames(&mut reader, 2).unwrap();
        assert_eq!(out.pcm, ref_pcm);
    }

    #[test]
    fn gapless_trims_final_frame() {
        let mut hdr = header(3, 1, false);
        hdr.true_gapless = true;
        hdr.last_frame_samples = 500;
        let frames = vec![Sv7EncStereoFrame::silent(2); 3];
        let raw = encode_sv7_file(&hdr, &frames, 0).unwrap();
        let out = decode_sv7_file(&raw, 0, test_undo).unwrap();
        assert_eq!(out.frames_decoded, 3);
        assert_eq!(out.pcm.len(), 2 * (2 * 1152 + 500));
    }

    #[test]
    fn gapless_zero_means_full_final_frame() {
        let mut hdr = header(2, 1, false);
        hdr.true_gapless = true;
        hdr.last_frame_samples = 0;
        let frames = vec![Sv7EncStereoFrame::silent(2); 2];
        let raw = encode_sv7_file(&hdr, &frames, 0).unwrap();
        let out = decode_sv7_file(&raw, 0, test_undo).unwrap();
        assert_eq!(out.pcm.len(), 2 * STEREO_FRAME_PCM_LEN);
    }

    #[test]
    fn non_gapless_ignores_last_frame_samples() {
        // §1 field 13: field 14 is meaningful only when true-gapless is
        // set — no trim without it.
        let mut hdr = header(2, 1, false);
        hdr.true_gapless = false;
        hdr.last_frame_samples = 500;
        let frames = vec![Sv7EncStereoFrame::silent(2); 2];
        let raw = encode_sv7_file(&hdr, &frames, 0).unwrap();
        let out = decode_sv7_file(&raw, 0, test_undo).unwrap();
        assert_eq!(out.pcm.len(), 2 * STEREO_FRAME_PCM_LEN);
    }

    #[test]
    fn rejects_gapless_count_above_frame_geometry() {
        let mut hdr = header(1, 1, false);
        hdr.true_gapless = true;
        hdr.last_frame_samples = 1153;
        let frames = vec![Sv7EncStereoFrame::silent(2)];
        // Encode with the trim gate bypassed: build the same header but
        // with a legal count, then decode with the bad one patched in
        // is not possible on immutable bytes — instead encode directly
        // (the encoder validates only the 11-bit width, 1153 fits).
        let raw = encode_sv7_file(&hdr, &frames, 0).unwrap();
        assert_eq!(
            decode_sv7_file(&raw, 0, test_undo),
            Err(Error::HeaderFieldOutOfRange("last_frame_samples")),
        );
    }

    #[test]
    fn truncated_file_is_unexpected_eof() {
        let hdr = header(2, 2, true);
        let raw = encode_sv7_file(&hdr, &busy_frames(), 0).unwrap();
        // Cut the file mid-body (keep the header + a little).
        let cut = &raw[..32.min(raw.len())];
        assert_eq!(
            decode_sv7_file(cut, 0, test_undo),
            Err(Error::UnexpectedEof),
        );
    }

    #[test]
    fn bad_magic_is_rejected() {
        let hdr = header(1, 1, false);
        let mut raw = encode_sv7_file(&hdr, &[Sv7EncStereoFrame::silent(2)], 0).unwrap();
        raw[0] ^= 0xFF;
        assert_eq!(
            decode_sv7_file(&raw, 0, test_undo),
            Err(Error::InvalidMagic)
        );
    }

    #[test]
    fn zero_frame_file_decodes_to_empty_pcm() {
        let hdr = header(0, 5, false);
        let raw = encode_sv7_file(&hdr, &[], 0).unwrap();
        let out = decode_sv7_file(&raw, 0, test_undo).unwrap();
        assert_eq!(out.frames_decoded, 0);
        assert!(out.pcm.is_empty());
    }

    #[test]
    fn ms_undo_closure_reaches_the_output() {
        // The same M/S-flagged file decoded under two different undo
        // closures must differ — proof the closure is applied to the
        // file-level path.
        let hdr = header(1, 1, true);
        let frames = vec![Sv7EncStereoFrame {
            left: vec![coded(3, 20), Sv7EncBand::Empty],
            right: vec![coded(3, 6), Sv7EncBand::Empty],
            ms_flags: vec![true, false],
        }];
        let raw = encode_sv7_file(&hdr, &frames, 0).unwrap();
        let a = decode_sv7_file(&raw, 0, test_undo).unwrap();
        let b = decode_sv7_file(&raw, 0, |m, s| (m, s)).unwrap();
        assert_eq!(a.header, b.header);
        assert_ne!(a.pcm, b.pcm);
    }
}
