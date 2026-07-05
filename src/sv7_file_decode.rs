//! SV7 whole-stream (`.mpc` file) **decode** — raw bytes in, header +
//! interleaved PCM out, over the corpus-pinned wire layout.
//!
//! Layout walked (positions pinned by the `tests/fixtures/sv7/` corpus;
//! see [`crate::sv7_file_encode`] for the writer's description):
//!
//! 1. **§1 fixed header** — parsed off the raw bytes
//!    ([`crate::sv7_header::Sv7HeaderFields::parse`]), bits 0–199.
//! 2. **§4 word swap** — one 32-bit-LE-word byte-swap over the whole
//!    stream (header and body share the word grid).
//! 3. **Per frame** — a 20-bit body bit-length prefix, then exactly
//!    that many body bits. The decoder decodes each body with the
//!    persistent [`crate::sv7_stream::Sv7StreamDecoder`] state and
//!    **verifies** the consumed bit count against the prefix — a
//!    mismatch means the parse diverged from the wire syntax and fails
//!    loudly ([`crate::Error::FrameBitLengthMismatch`]) instead of
//!    emitting garbage.
//! 4. **11-bit trailer** — after the final body, the in-stream
//!    last-frame valid-sample count. On a true-gapless stream it must
//!    equal §1 header field 14
//!    ([`crate::Error::LastFrameTrailerMismatch`] otherwise; every
//!    corpus stream matches, including the literal `1152` full-frame
//!    case).
//! 5. **Ignored tail** — anything after the trailer (mppenc appends an
//!    undeclared flush frame on some streams) is skipped.
//! 6. **Gapless trim** — when the §1 true-gapless flag (field 13) is
//!    set and the last-frame valid-sample count (field 14) is non-zero,
//!    the final frame contributes only that many samples per channel.
//!
//! Output PCM is in the **signed-16-bit domain** (the corpus-pinned
//! absolute reconstruction — see
//! [`crate::reconstruct::sv7_absolute_scf_gain`]); use
//! [`Sv7DecodedFile::pcm_s16`] for playback-ready samples. Against the
//! FFmpeg `mpc7` oracle the decoded corpus streams match to within
//! ±1 LSB on every sample (~75% bit-exact; the residue is the oracle's
//! f32 DSP vs this crate's f64 synthesis).

use crate::huffman::Sv7BitReader;
use crate::sv7_file_encode::{SV7_FRAME_LENGTH_PREFIX_BITS, SV7_LAST_FRAME_TRAILER_BITS};
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
    /// The in-stream 11-bit last-frame trailer (`None` for a zero-frame
    /// stream, which carries no trailer).
    pub stream_last_frame_samples: Option<u16>,
    /// Interleaved stereo PCM, `L, R, …` — `2 ×` the per-channel total
    /// after the gapless trim, in the signed-16-bit domain (`f64`
    /// values; round + clamp via [`Self::pcm_s16`]).
    pub pcm: Vec<f64>,
}

impl Sv7DecodedFile {
    /// The decoded PCM as interleaved `i16` samples: each value rounded
    /// half-away-from-zero and clamped to the `i16` range.
    #[must_use]
    pub fn pcm_s16(&self) -> Vec<i16> {
        self.pcm
            .iter()
            .map(|&v| v.round().clamp(f64::from(i16::MIN), f64::from(i16::MAX)) as i16)
            .collect()
    }
}

/// Decode a complete SV7 `.mpc` stream from `bytes`.
///
/// # Errors
///
/// - [`Error::InvalidMagic`] / [`Error::UnsupportedVersion`] /
///   [`Error::MaxBandOutOfRange`] / [`Error::UnexpectedEof`] from the
///   §1 header parse.
/// - [`Error::UnexpectedEof`] if the stream ends before the declared
///   frame count's prefixes/bodies/trailer (truncated file).
/// - [`Error::HeaderFieldOutOfRange`]`("last_frame_samples")` if the
///   header declares true-gapless with a last-frame sample count above
///   the 1152-sample frame geometry.
/// - [`Error::FrameBitLengthMismatch`] if a frame body consumed a
///   different bit count than its 20-bit prefix declared.
/// - [`Error::LastFrameTrailerMismatch`] if the 11-bit trailer
///   disagrees with header field 14 on a true-gapless stream.
/// - Any frame-body decode error (no-match VLC, out-of-range band-type
///   / SCFI, reader starvation mid-frame).
pub fn decode_sv7_file(bytes: &[u8]) -> Result<Sv7DecodedFile> {
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
    // VLC, so a code that ends within the final 15 bits of the run
    // would otherwise starve the lookahead. This is reader slack only
    // (the frame loop is bounded by the verified per-frame budgets, so
    // the pad bits are never decoded as data).
    let mut swapped = crate::sv7_word_swap::word_swap_sv7_body(bytes);
    swapped.extend_from_slice(&[0u8; 4]);
    let mut reader = Sv7BitReader::new(&swapped);
    let total_bits = reader.bits_remaining();

    // 3. The framed audio run starts at the first bit after field 17.
    skip_bits(&mut reader, SV7_HEADER_BITS)?;

    let mut decoder = Sv7StreamDecoder::from_header(&header)?;
    let frame_count = u64::from(header.frame_count);
    let mut pcm = Vec::new();
    for frame in 0..frame_count {
        // 20-bit body bit-length prefix.
        let hi = u32::from(reader.read_bits(16)?);
        let lo = u32::from(reader.read_bits(SV7_FRAME_LENGTH_PREFIX_BITS - 16)?);
        let declared = (hi << (SV7_FRAME_LENGTH_PREFIX_BITS - 16)) | lo;
        let start = total_bits - reader.bits_remaining();
        let frame_pcm = decoder.decode_frame(&mut reader)?;
        let consumed = (total_bits - reader.bits_remaining() - start) as u32;
        if consumed != declared {
            return Err(Error::FrameBitLengthMismatch {
                frame: frame as u32,
                declared,
                consumed,
            });
        }
        pcm.extend_from_slice(&frame_pcm);
    }
    let frames_decoded = decoder.frames_decoded();

    // 4. The 11-bit last-frame trailer (absent on zero-frame streams).
    let stream_last_frame_samples = if frame_count > 0 {
        let trailer = reader.read_bits(SV7_LAST_FRAME_TRAILER_BITS)?;
        if header.true_gapless && trailer != header.last_frame_samples {
            return Err(Error::LastFrameTrailerMismatch {
                header: header.last_frame_samples,
                stream: trailer,
            });
        }
        Some(trailer)
    } else {
        None
    };
    // 5. Anything after the trailer (flush frame, padding) is ignored.

    // 6. §1 fields 13/14: gapless trim of the final frame.
    pcm.truncate((2 * header.effective_total_samples()) as usize);

    Ok(Sv7DecodedFile {
        header,
        frames_decoded,
        stream_last_frame_samples,
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
    use crate::sv7_file_encode::{encode_sv7_file, Sv7EncStereoFrame, Sv7FileWriter};
    use crate::sv7_frame_encode::Sv7EncBand;
    use crate::sv7_stream::STEREO_FRAME_PCM_LEN;

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
                left: vec![
                    coded(3, 7),
                    Sv7EncBand::Cns { scf: [4, 4, 4] },
                    Sv7EncBand::Empty,
                ],
                right: vec![
                    coded(3, 9),
                    Sv7EncBand::Empty,
                    Sv7EncBand::Cns { scf: [6, 6, 6] },
                ],
                ms_flags: vec![true, false, false],
            },
            Sv7EncStereoFrame {
                left: vec![
                    Sv7EncBand::Empty,
                    coded(3, 12),
                    Sv7EncBand::Cns { scf: [5, 5, 5] },
                ],
                right: vec![
                    coded(3, 5),
                    Sv7EncBand::Cns { scf: [7, 7, 7] },
                    Sv7EncBand::Empty,
                ],
                ms_flags: vec![false, true, true],
            },
        ]
    }

    #[test]
    fn silent_file_round_trips() {
        let hdr = header(2, 4, false);
        let frames = vec![Sv7EncStereoFrame::silent(5); 2];
        let raw = encode_sv7_file(&hdr, &frames).unwrap();
        let out = decode_sv7_file(&raw).unwrap();
        assert_eq!(out.header, hdr);
        assert_eq!(out.frames_decoded, 2);
        assert_eq!(out.stream_last_frame_samples, Some(0));
        assert_eq!(out.pcm.len(), 2 * STEREO_FRAME_PCM_LEN);
        assert!(out.pcm.iter().all(|&s| s == 0.0));
        assert!(out.pcm_s16().iter().all(|&s| s == 0));
    }

    #[test]
    fn coded_file_round_trips_and_verifies_every_prefix() {
        let hdr = header(2, 2, true);
        let raw = encode_sv7_file(&hdr, &busy_frames()).unwrap();
        let out = decode_sv7_file(&raw).unwrap();
        assert_eq!(out.header, hdr);
        assert_eq!(out.frames_decoded, 2);
        assert_eq!(out.pcm.len(), 2 * STEREO_FRAME_PCM_LEN);
        assert!(out.pcm.iter().any(|&s| s != 0.0));
    }

    #[test]
    fn gapless_trims_final_frame() {
        let mut hdr = header(3, 1, false);
        hdr.true_gapless = true;
        hdr.last_frame_samples = 500;
        let frames = vec![Sv7EncStereoFrame::silent(2); 3];
        let raw = encode_sv7_file(&hdr, &frames).unwrap();
        let out = decode_sv7_file(&raw).unwrap();
        assert_eq!(out.frames_decoded, 3);
        assert_eq!(out.stream_last_frame_samples, Some(500));
        assert_eq!(out.pcm.len(), 2 * (2 * 1152 + 500));
    }

    #[test]
    fn gapless_zero_means_full_final_frame() {
        let mut hdr = header(2, 1, false);
        hdr.true_gapless = true;
        hdr.last_frame_samples = 0;
        let frames = vec![Sv7EncStereoFrame::silent(2); 2];
        let raw = encode_sv7_file(&hdr, &frames).unwrap();
        let out = decode_sv7_file(&raw).unwrap();
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
        let raw = encode_sv7_file(&hdr, &frames).unwrap();
        let out = decode_sv7_file(&raw).unwrap();
        assert_eq!(out.pcm.len(), 2 * STEREO_FRAME_PCM_LEN);
    }

    #[test]
    fn rejects_gapless_count_above_frame_geometry() {
        let mut hdr = header(1, 1, false);
        hdr.true_gapless = true;
        hdr.last_frame_samples = 1153;
        let frames = vec![Sv7EncStereoFrame::silent(2)];
        // The encoder validates only the 11-bit width, 1153 fits.
        let raw = encode_sv7_file(&hdr, &frames).unwrap();
        assert_eq!(
            decode_sv7_file(&raw),
            Err(Error::HeaderFieldOutOfRange("last_frame_samples")),
        );
    }

    #[test]
    fn truncated_file_is_unexpected_eof() {
        let hdr = header(2, 2, true);
        let raw = encode_sv7_file(&hdr, &busy_frames()).unwrap();
        // Cut the file mid-body (keep the header + a little).
        let cut = &raw[..32.min(raw.len())];
        assert_eq!(decode_sv7_file(cut), Err(Error::UnexpectedEof));
    }

    #[test]
    fn corrupted_prefix_fails_loud_as_length_mismatch() {
        let hdr = header(2, 2, true);
        let raw = encode_sv7_file(&hdr, &busy_frames()).unwrap();
        // Flip a low bit of frame 0's 20-bit prefix. The prefix occupies
        // logical bits 200..220 — logical byte 27 holds bits 216..224,
        // which lives at on-disk index 24 (word 6 reversed: 27 -> 24).
        let mut bad = raw.clone();
        bad[24] ^= 0x10;
        match decode_sv7_file(&bad) {
            Err(Error::FrameBitLengthMismatch { frame, .. }) => assert_eq!(frame, 0),
            // Depending on the flip the body may starve first — also a
            // loud failure, never silent garbage.
            Err(Error::UnexpectedEof | Error::HuffmanNoMatch) => {}
            other => panic!("expected loud failure, got {other:?}"),
        }
    }

    #[test]
    fn bad_magic_is_rejected() {
        let hdr = header(1, 1, false);
        let mut raw = encode_sv7_file(&hdr, &[Sv7EncStereoFrame::silent(2)]).unwrap();
        raw[0] ^= 0xFF;
        assert_eq!(decode_sv7_file(&raw), Err(Error::InvalidMagic));
    }

    #[test]
    fn zero_frame_file_decodes_to_empty_pcm() {
        let hdr = header(0, 5, false);
        let raw = encode_sv7_file(&hdr, &[]).unwrap();
        let out = decode_sv7_file(&raw).unwrap();
        assert_eq!(out.frames_decoded, 0);
        assert_eq!(out.stream_last_frame_samples, None);
        assert!(out.pcm.is_empty());
    }

    #[test]
    fn trailing_bytes_after_trailer_are_ignored() {
        // mppenc appends an undeclared flush frame after the trailer on
        // some streams; any tail must be ignored.
        let hdr = header(1, 1, false);
        let mut raw = encode_sv7_file(&hdr, &[Sv7EncStereoFrame::silent(2)]).unwrap();
        raw.extend_from_slice(&[0xAB; 64]);
        let out = decode_sv7_file(&raw).unwrap();
        assert_eq!(out.frames_decoded, 1);
    }

    /// Build one coded band for `band_type` with arm-valid levels:
    /// grouped digits for 1/2, table-alphabet values for 3..=7 (per
    /// context), raw-unsigned `band_type − 1`-bit levels for the
    /// PCM-escape ladder 8..=17. `seed` varies the pattern per band.
    fn band_for(band_type: i8, ctx: usize, seed: i32, scf: [i32; 3]) -> Sv7EncBand {
        use crate::huffman::{sv7_q4_ctx, sv7_q5_ctx, sv7_q6_ctx, sv7_q7_ctx};
        let levels: [i32; SAMPLES_PER_BAND] = match band_type {
            1 => core::array::from_fn(|i| (i as i32 + seed).rem_euclid(3) - 1),
            2 => core::array::from_fn(|i| (i as i32 + seed).rem_euclid(5) - 2),
            3..=7 => {
                let table = match band_type {
                    3 => sv7_q3_ctx(ctx),
                    4 => sv7_q4_ctx(ctx),
                    5 => sv7_q5_ctx(ctx),
                    6 => sv7_q6_ctx(ctx),
                    _ => sv7_q7_ctx(ctx),
                };
                let mut alpha: Vec<i32> = table.iter().map(|e| e.value as i32).collect();
                alpha.dedup();
                core::array::from_fn(|i| alpha[(i + seed as usize) % alpha.len()])
            }
            8..=17 => {
                let span = 1i32 << (band_type - 1);
                core::array::from_fn(|i| (i as i32 * 7 + seed).rem_euclid(span))
            }
            _ => unreachable!("coded band_type only"),
        };
        Sv7EncBand::Coded {
            band_type,
            ctx,
            scf,
            levels,
        }
    }

    /// An SCF triple whose sharing pattern drives SCFI case `i % 4`
    /// (all-coded / share-tail / share-head / share-all), values within
    /// the raw-6-bit DSCF escape reach (0..=63).
    fn scf_pattern(i: usize) -> [i32; 3] {
        let b = 5 + ((i as i32 * 3) % 40);
        match i % 4 {
            0 => [b, b + 4, b + 1], // SCFI 0: all three coded
            1 => [b, b + 4, b + 4], // SCFI 1: SCF[2] copies SCF[1]
            2 => [b, b, b + 6],     // SCFI 2: SCF[1] copies SCF[0]
            _ => [b, b, b],         // SCFI 3: both copy
        }
    }

    /// A channel covering the given band-type ladder.
    fn ladder_channel(types: &[i8], ctx: usize, seed: i32) -> Vec<Sv7EncBand> {
        types
            .iter()
            .enumerate()
            .map(|(i, &bt)| match bt {
                -1 => Sv7EncBand::Cns {
                    scf: scf_pattern(i),
                },
                0 => Sv7EncBand::Empty,
                _ => band_for(bt, ctx, seed + i as i32, scf_pattern(i)),
            })
            .collect()
    }

    /// Every §5.4 band-type arm (CNS −1, empty 0, grouped 1/2,
    /// per-sample Huffman 3..=7 on both contexts, the full PCM-escape
    /// ladder 8..=17), all four SCFI sharing cases, and per-band M/S
    /// flags — through the whole-file writer and decoder with every
    /// frame's 20-bit budget verified.
    #[test]
    fn every_band_type_arm_survives_the_file_layer() {
        // 19 bands: two walks that each visit every band type exactly
        // once (left on ctx 0, right on ctx 1). The §5.1 header can
        // carry −1 / 16 / 17 only through the delta chain (band-0 raw
        // and the escape are 4-bit absolutes, 0..=15), so both walks
        // keep every step within delta −5..=3 of its predecessor —
        // except one deliberate escape in the rotated variants.
        let ladder: Vec<i8> = vec![
            3, -1, 0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17,
        ];
        let right_walk: Vec<i8> = vec![
            7, 2, -1, 0, 1, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17,
        ];
        let max_band = (ladder.len() - 1) as u8; // 18
        let ms_flags: Vec<bool> = (0..ladder.len()).map(|i| i % 2 == 0).collect();

        // Second frame rotates the walks so band/type pairings (and the
        // per-band SCF memory) differ across frames; the rotation
        // introduces one large negative jump that exercises the §5.1
        // raw-absolute escape (17 → 3 and 17 → 7, both within 0..=15).
        let mut rotated = ladder.clone();
        rotated.rotate_left(5);
        let mut rotated_rev = right_walk.clone();
        rotated_rev.rotate_left(5);

        let frames = vec![
            Sv7EncStereoFrame {
                left: ladder_channel(&ladder, 0, 0),
                right: ladder_channel(&right_walk, 1, 3),
                ms_flags: ms_flags.clone(),
            },
            Sv7EncStereoFrame {
                left: ladder_channel(&rotated, 1, 11),
                right: ladder_channel(&rotated_rev, 0, 7),
                ms_flags: ms_flags.iter().map(|f| !f).collect(),
            },
        ];
        let hdr = header(2, max_band, true);

        let raw = encode_sv7_file(&hdr, &frames).expect("encode");
        // Determinism: the composer is a pure function of its inputs.
        assert_eq!(raw, encode_sv7_file(&hdr, &frames).unwrap());

        let out = decode_sv7_file(&raw).expect("decode");
        assert_eq!(out.header, hdr);
        assert_eq!(out.frames_decoded, 2);
        assert_eq!(out.pcm.len(), 2 * STEREO_FRAME_PCM_LEN);
        assert!(out.pcm.iter().any(|&s| s != 0.0));
    }

    /// The incremental builder path produces the same decoded stream as
    /// the one-shot for the all-arm corpus (positioning + gapless).
    #[test]
    fn builder_all_arm_file_decodes_with_gapless_trim() {
        let ladder: Vec<i8> = vec![
            3, -1, 0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17,
        ];
        let max_band = (ladder.len() - 1) as u8;
        let frame = Sv7EncStereoFrame {
            left: ladder_channel(&ladder, 0, 1),
            right: ladder_channel(&ladder, 1, 2),
            ms_flags: vec![false; ladder.len()],
        };
        let mut w = Sv7FileWriter::new(header(0, max_band, false)).unwrap();
        w.push_frame(&frame).unwrap();
        w.push_frame(&frame).unwrap();
        let raw = w.finish_gapless(700).unwrap();

        let out = decode_sv7_file(&raw).unwrap();
        assert_eq!(out.frames_decoded, 2);
        assert!(out.header.true_gapless);
        assert_eq!(out.stream_last_frame_samples, Some(700));
        assert_eq!(out.header.effective_total_samples(), 1152 + 700);
        assert_eq!(out.pcm.len(), 2 * (1152 + 700));
        assert!(out.pcm.iter().any(|&s| s != 0.0));
    }

    #[test]
    fn ms_flagged_file_differs_from_unflagged() {
        // The same band data with and without the M/S flag must decode
        // differently — proof the pinned undo is applied at file level.
        let mk = |ms: bool| {
            let hdr = header(1, 1, true);
            let frames = vec![Sv7EncStereoFrame {
                left: vec![coded(3, 20), Sv7EncBand::Empty],
                right: vec![coded(3, 6), Sv7EncBand::Empty],
                ms_flags: vec![ms, false],
            }];
            decode_sv7_file(&encode_sv7_file(&hdr, &frames).unwrap()).unwrap()
        };
        let with_ms = mk(true);
        let without = mk(false);
        assert_ne!(with_ms.pcm, without.pcm);
    }

    #[test]
    fn pcm_s16_rounds_and_clamps() {
        let file = Sv7DecodedFile {
            header: header(0, 1, false),
            frames_decoded: 0,
            stream_last_frame_samples: None,
            pcm: vec![0.4, -0.5, 100_000.0, -100_000.0, 32767.4, -32768.4],
        };
        assert_eq!(file.pcm_s16(), vec![0, -1, 32767, -32768, 32767, -32768]);
    }
}
