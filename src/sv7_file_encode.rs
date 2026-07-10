//! SV7 whole-stream (`.mpc` file) **encode** — the §1 fixed header, the
//! corpus-pinned per-frame framing, and the 11-bit stream trailer
//! composed into one raw byte buffer.
//!
//! # The wire layout (§1.1, corpus-verified)
//!
//! The staged §1.1 (framing corrections, docs commit `0f1b6a2`)
//! documents the whole-file layout, originally pinned by the SV7
//! fixture corpus (`tests/fixtures/sv7/`; independent mppenc 1.16
//! streams):
//!
//! 1. **bits 0–199** — the §1 fixed header
//!    ([`crate::sv7_header_encode`]);
//! 2. **per frame** — a **20-bit body bit-length prefix** followed by
//!    exactly that many body bits (the §5 frame body,
//!    [`crate::sv7_stereo_frame_encode::encode_sv7_stereo_frame`]).
//!    Walking the corpus's prefix chain lands every frame boundary and
//!    the stream trailer exactly, across all 72 frames of 4 files;
//! 3. **after the last frame body** — an **11-bit trailer** carrying
//!    the last-frame valid-sample count (equal to §1 header field 14 on
//!    every corpus stream — including the literal `1152` on the
//!    exact-multiple fixture);
//! 4. zero padding to the 32-bit word grid, then the §4 whole-stream
//!    word swap ([`crate::sv7_word_swap`]) yields the on-disk bytes.
//!
//! (mppenc additionally appends one undeclared *flush* frame after the
//! trailer on some streams — `[20-bit length][body]` again — which
//! decoders ignore; this writer does not emit one, and the decoder
//! ignores any tail after the trailer.)
//!
//! The result begins with the raw `MP+` magic, parses with
//! [`crate::sv7_header::Sv7HeaderFields::parse`], and decodes end-to-end
//! with the whole-file decoder ([`crate::sv7_file_decode`]) including
//! its per-frame bit-budget verification.
//!
//! Source-of-record: `docs/audio/musepack/spec/musepack-headers-and-coding.md`
//! §1 (header fields), §1.1 (the 20-bit per-frame length prefix, the
//! band-major pass order, and the in-stream 11-bit last-frame
//! trailer), §4 (word swap), §5 (frame bodies).

use crate::sv7_bitwriter::Sv7BitWriter;
use crate::sv7_frame_encode::Sv7EncBand;
use crate::sv7_header::Sv7HeaderFields;
use crate::sv7_header_encode::write_sv7_header_fields;
use crate::sv7_stereo_frame::Sv7ScfMemory;
use crate::sv7_stereo_frame_encode::encode_sv7_stereo_frame;
use crate::{Error, Result};

/// Width of the per-frame body bit-length prefix (corpus-pinned §1.1).
pub const SV7_FRAME_LENGTH_PREFIX_BITS: u8 = 20;

/// Width of the post-last-frame stream trailer carrying the last-frame
/// valid-sample count (§1.1; corpus-pinned position).
pub const SV7_LAST_FRAME_TRAILER_BITS: u8 = 11;

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

/// Encode a complete SV7 `.mpc` stream with the fields-implied version
/// byte ([`Sv7HeaderFields::version_byte`]: `0x07`, or `0x17` when
/// `header.pns` is set). See [`encode_sv7_file_with_version`].
///
/// # Errors
///
/// See [`encode_sv7_file_with_version`].
pub fn encode_sv7_file(header: &Sv7HeaderFields, frames: &[Sv7EncStereoFrame]) -> Result<Vec<u8>> {
    encode_sv7_file_with_version(header, frames, header.version_byte())
}

/// Encode a complete SV7 `.mpc` stream: the §1 fixed header, each frame
/// as `[20-bit body bit length][body]`, the 11-bit last-frame trailer
/// (header field 14) after the final body, §4 word-swapped into raw
/// on-disk byte order.
///
/// `header` supplies every §1 field; its `frame_count` must equal
/// `frames.len()` and its `mid_side` flag gates the per-band M/S bits in
/// each frame's §5.1 header. Each frame's band vectors must cover
/// exactly `header.max_band + 1` subbands. A zero-frame stream is just
/// the header (no trailer).
///
/// # Errors
///
/// - [`Error::UnsupportedVersion`] / [`Error::MaxBandOutOfRange`] /
///   [`Error::HeaderFieldOutOfRange`] from the header layer, including
///   `HeaderFieldOutOfRange("frame_count")` when `header.frame_count`
///   disagrees with `frames.len()`.
/// - [`Error::MaxBandOutOfRange`] for a frame whose band count is not
///   `header.max_band + 1`.
/// - [`Error::HeaderFieldOutOfRange`]`("frame_bit_length")` for a frame
///   body larger than the 20-bit prefix can carry (> 2²⁰ − 1 bits;
///   unreachable for real frame geometries).
/// - Any frame-body encode error ([`Error::SampleOutOfRange`],
///   [`Error::SymbolNotEncodable`], [`Error::UnsupportedBandType`], …).
pub fn encode_sv7_file_with_version(
    header: &Sv7HeaderFields,
    frames: &[Sv7EncStereoFrame],
    version_byte: u8,
) -> Result<Vec<u8>> {
    if header.frame_count as usize != frames.len() {
        return Err(Error::HeaderFieldOutOfRange("frame_count"));
    }

    let mut writer = Sv7BitWriter::new();
    // §1: the 200-bit fixed header (validates every field).
    write_sv7_header_fields(&mut writer, header, version_byte)?;

    // Per frame: 20-bit body bit-length prefix, then the body.
    let bands = header.max_band as usize + 1;
    let mut scf = Sv7ScfMemory::new();
    for frame in frames {
        write_prefixed_frame(&mut writer, frame, bands, header.mid_side, &mut scf)?;
    }

    // The 11-bit last-frame trailer follows the final body.
    if !frames.is_empty() {
        writer.write_bits(
            u32::from(header.last_frame_samples),
            SV7_LAST_FRAME_TRAILER_BITS,
        );
    }

    // §4: one word-swap over the whole logical run (zero-padding the
    // trailing partial word) yields the raw on-disk stream.
    let logical = writer.finish();
    Ok(crate::sv7_word_swap::word_swap_sv7_body(&logical))
}

/// Assemble one frame body in a scratch writer (so its exact bit length
/// is known), then emit `[20-bit length][body]` into `writer`.
fn write_prefixed_frame(
    writer: &mut Sv7BitWriter,
    frame: &Sv7EncStereoFrame,
    bands: usize,
    stream_ms: bool,
    scf: &mut Sv7ScfMemory,
) -> Result<()> {
    if frame.left.len() != bands || frame.right.len() != bands || frame.ms_flags.len() != bands {
        let implied = frame
            .left
            .len()
            .max(frame.right.len())
            .max(frame.ms_flags.len())
            .saturating_sub(1)
            .min(u8::MAX as usize) as u8;
        return Err(Error::MaxBandOutOfRange(implied));
    }
    let mut body = Sv7BitWriter::new();
    encode_sv7_stereo_frame(
        &mut body,
        &frame.left,
        &frame.right,
        &frame.ms_flags,
        stream_ms,
        scf,
    )?;
    let bits = body.bit_len();
    if bits >= (1 << SV7_FRAME_LENGTH_PREFIX_BITS) {
        return Err(Error::HeaderFieldOutOfRange("frame_bit_length"));
    }
    writer.write_bits(bits as u32, SV7_FRAME_LENGTH_PREFIX_BITS);
    writer.append(&body);
    Ok(())
}

/// Incremental SV7 `.mpc` stream builder — the push-frame counterpart
/// of the one-shot [`encode_sv7_file`].
///
/// The §1 header carries the frame count (field 1), which an
/// incremental encoder does not know until the last frame is pushed.
/// This builder exploits the fact that the §1 field span ends at
/// logical bit 200 — a whole **byte** boundary (25 bytes), though not a
/// word boundary — so the framed audio run can be accumulated in its
/// own continuous bit run and the header serialised afterwards,
/// prepended byte-for-byte: `finish` produces output identical to the
/// one-shot composer (byte-equality is test-proven).
///
/// `template` supplies every §1 field except `frame_count` (overridden
/// with the pushed-frame count at finish) and — for
/// [`Sv7FileWriter::finish_gapless`] — the true-gapless flag and
/// last-frame sample count (fields 13/14, overridden by that method;
/// field 14 is also what the 11-bit stream trailer carries).
#[derive(Debug, Clone)]
pub struct Sv7FileWriter {
    template: Sv7HeaderFields,
    version_byte: u8,
    body: Sv7BitWriter,
    scf: Sv7ScfMemory,
    frames: u32,
}

impl Sv7FileWriter {
    /// Start a builder with the template-implied version byte
    /// ([`Sv7HeaderFields::version_byte`]: `0x07`, or `0x17` when
    /// `template.pns` is set). See [`Sv7FileWriter::with_version`].
    ///
    /// # Errors
    ///
    /// See [`Sv7FileWriter::with_version`].
    pub fn new(template: Sv7HeaderFields) -> Result<Self> {
        Self::with_version(template, template.version_byte())
    }

    /// Start a builder from a §1 header `template` (validated
    /// immediately, fail-loud).
    ///
    /// # Errors
    ///
    /// - [`Error::UnsupportedVersion`] if `version_byte`'s low nibble is
    ///   not 7.
    /// - [`Error::HeaderFieldOutOfRange`]`("pns")` if `version_byte`'s
    ///   `0x10` PNS-flag bit contradicts `template.pns`.
    /// - [`Error::MaxBandOutOfRange`] / [`Error::HeaderFieldOutOfRange`]
    ///   for a template field outside its §1 width (the template's
    ///   `frame_count` is ignored — it is overridden at finish).
    pub fn with_version(template: Sv7HeaderFields, version_byte: u8) -> Result<Self> {
        if version_byte & 0x0F != crate::framing::SV7_VERSION_NIBBLE {
            return Err(Error::UnsupportedVersion(version_byte));
        }
        if (version_byte & crate::framing::SV7_VERSION_PNS_FLAG != 0) != template.pns {
            return Err(Error::HeaderFieldOutOfRange("pns"));
        }
        crate::sv7_header_encode::validate_sv7_header_fields(&template)?;
        Ok(Self {
            template,
            version_byte,
            body: Sv7BitWriter::new(),
            scf: Sv7ScfMemory::new(),
            frames: 0,
        })
    }

    /// Append one stereo frame (`[20-bit length][body]`) to the framed
    /// audio run.
    ///
    /// # Errors
    ///
    /// - [`Error::MaxBandOutOfRange`] if the frame's band vectors do not
    ///   cover exactly `template.max_band + 1` subbands.
    /// - [`Error::HeaderFieldOutOfRange`]`("frame_count")` if the pushed
    ///   count would overflow the 32-bit §1 frame-count field.
    /// - Any frame-body encode error.
    pub fn push_frame(&mut self, frame: &Sv7EncStereoFrame) -> Result<()> {
        if self.frames == u32::MAX {
            return Err(Error::HeaderFieldOutOfRange("frame_count"));
        }
        let bands = self.template.max_band as usize + 1;
        write_prefixed_frame(
            &mut self.body,
            frame,
            bands,
            self.template.mid_side,
            &mut self.scf,
        )?;
        self.frames += 1;
        Ok(())
    }

    /// Number of frames pushed so far.
    #[must_use]
    pub fn frames_pushed(&self) -> u32 {
        self.frames
    }

    /// Serialise the complete raw `.mpc` stream: the §1 header (with
    /// `frame_count` = the pushed count), the framed audio run, the
    /// 11-bit trailer (template field 14), §4 word-swapped. Consumes
    /// the builder.
    ///
    /// # Errors
    ///
    /// Header-layer validation errors (the template was validated at
    /// construction, so these only fire for values that became invalid
    /// through [`Sv7FileWriter::finish_gapless`]'s overrides — see
    /// there).
    pub fn finish(self) -> Result<Vec<u8>> {
        let mut header = self.template;
        header.frame_count = self.frames;
        let mut writer = Sv7BitWriter::new();
        write_sv7_header_fields(&mut writer, &header, self.version_byte)?;
        // The header run is exactly 200 bits = 25 whole bytes, so the
        // framed audio run (whose logical bit 0 is stream bit 200)
        // appends byte-for-byte.
        let mut logical = writer.finish();
        debug_assert_eq!(logical.len(), 25);
        let mut tail = self.body;
        if self.frames > 0 {
            tail.write_bits(
                u32::from(header.last_frame_samples),
                SV7_LAST_FRAME_TRAILER_BITS,
            );
        }
        logical.extend_from_slice(&tail.finish());
        Ok(crate::sv7_word_swap::word_swap_sv7_body(&logical))
    }

    /// [`Sv7FileWriter::finish`] with the §1 gapless fields set: the
    /// true-gapless flag (field 13) and the final frame's valid-sample
    /// count (field 14, mirrored into the 11-bit stream trailer; the
    /// corpus encoder writes the literal count — `1152` for a full
    /// final frame, not `0`).
    ///
    /// # Errors
    ///
    /// - [`Error::HeaderFieldOutOfRange`]`("last_frame_samples")` if
    ///   `last_frame_samples` exceeds the 1152-sample frame geometry.
    /// - See [`Sv7FileWriter::finish`].
    pub fn finish_gapless(mut self, last_frame_samples: u16) -> Result<Vec<u8>> {
        if u64::from(last_frame_samples) > crate::sv7_header::SV7_SAMPLES_PER_FRAME {
            return Err(Error::HeaderFieldOutOfRange("last_frame_samples"));
        }
        self.template.true_gapless = true;
        self.template.last_frame_samples = last_frame_samples;
        self.finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::huffman::{sv7_q3_ctx, Sv7BitReader};
    use crate::sv7_band_decode::SAMPLES_PER_BAND;
    use crate::sv7_header_encode::SV7_HEADER_BITS;

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

    fn skip_bits(reader: &mut Sv7BitReader<'_>, mut n: u64) {
        while n > 0 {
            let step = n.min(16) as u8;
            reader.read_bits(step).expect("bits present");
            n -= u64::from(step);
        }
    }

    fn read20(reader: &mut Sv7BitReader<'_>) -> u32 {
        let hi = reader.read_bits(16).unwrap() as u32;
        let lo = reader.read_bits(4).unwrap() as u32;
        (hi << 4) | lo
    }

    #[test]
    fn silent_file_layout_prefix_body_trailer() {
        let hdr = {
            let mut h = header(2, 3, false);
            h.true_gapless = true;
            h.last_frame_samples = 700;
            h
        };
        let frames = vec![Sv7EncStereoFrame::silent(4); 2];
        let raw = encode_sv7_file(&hdr, &frames).expect("encode");

        // Header round-trips off the raw bytes.
        assert_eq!(&raw[..3], b"MP+");
        assert_eq!(Sv7HeaderFields::parse(&raw).unwrap(), hdr);
        assert_eq!(raw.len() % 4, 0, "word-aligned on-disk length");

        // Walk the corpus-pinned layout: [20-bit len][body] per frame,
        // then the 11-bit trailer. A silent max_band=3 frame body is
        // 4 + 4 raw band-0 bits + 3 delta-0 VLC bits × 2 channels = 14.
        let swapped = crate::sv7_word_swap::word_swap_sv7_body(&raw);
        let mut r = Sv7BitReader::new(&swapped);
        skip_bits(&mut r, SV7_HEADER_BITS);
        for _ in 0..2 {
            let len = read20(&mut r);
            assert_eq!(len, 14, "silent frame body bits");
            skip_bits(&mut r, u64::from(len));
        }
        assert_eq!(r.read_bits(11).unwrap(), 700, "trailer");
    }

    #[test]
    fn frame_prefixes_match_body_bit_lengths_for_coded_frames() {
        let coded = |scf0: i32| Sv7EncBand::Coded {
            band_type: 3,
            ctx: 0,
            scf: [scf0, scf0 + 1, scf0 + 1],
            levels: q3_levels(),
        };
        let frame_a = Sv7EncStereoFrame {
            left: vec![
                coded(7),
                Sv7EncBand::Cns { scf: [9, 9, 9] },
                Sv7EncBand::Empty,
            ],
            right: vec![
                coded(9),
                Sv7EncBand::Empty,
                Sv7EncBand::Cns { scf: [4, 5, 4] },
            ],
            ms_flags: vec![true, false, false],
        };
        let frame_b = Sv7EncStereoFrame {
            left: vec![
                Sv7EncBand::Empty,
                coded(12),
                Sv7EncBand::Cns { scf: [8, 8, 8] },
            ],
            right: vec![
                coded(5),
                Sv7EncBand::Cns { scf: [6, 6, 6] },
                Sv7EncBand::Empty,
            ],
            ms_flags: vec![false, true, true],
        };
        let hdr = header(2, 2, true);
        let raw = encode_sv7_file(&hdr, &[frame_a, frame_b]).expect("encode");

        // Chain the prefixes: each must land exactly on the next, and
        // the file must end at trailer + word padding.
        let swapped = crate::sv7_word_swap::word_swap_sv7_body(&raw);
        let mut r = Sv7BitReader::new(&swapped);
        let total = r.bits_remaining();
        skip_bits(&mut r, SV7_HEADER_BITS);
        for _ in 0..2 {
            let len = read20(&mut r);
            assert!(len > 0);
            skip_bits(&mut r, u64::from(len));
        }
        let _trailer = r.read_bits(11).unwrap();
        let pos = total - r.bits_remaining();
        assert!(total - pos < 32, "only word padding may remain");
    }

    #[test]
    fn body_starts_immediately_after_header_field_17() {
        // The first 20 bits after the header are frame 0's length
        // prefix, followed by the §5.1 band-0 raw Res values.
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
        let raw = encode_sv7_file(&hdr, &frames).unwrap();
        let swapped = crate::sv7_word_swap::word_swap_sv7_body(&raw);
        let mut reader = Sv7BitReader::new(&swapped);
        skip_bits(&mut reader, SV7_HEADER_BITS);
        let _len = read20(&mut reader);
        // §5.1 band 0: left Res then right Res as raw 4-bit values.
        assert_eq!(reader.read_bits(4).unwrap(), 3);
        assert_eq!(reader.read_bits(4).unwrap(), 3);
    }

    #[test]
    fn rejects_frame_count_mismatch() {
        let hdr = header(2, 1, false);
        let frames = vec![Sv7EncStereoFrame::silent(2)];
        assert_eq!(
            encode_sv7_file(&hdr, &frames),
            Err(Error::HeaderFieldOutOfRange("frame_count")),
        );
    }

    #[test]
    fn rejects_frame_band_count_disagreeing_with_max_band() {
        let hdr = header(1, 3, false); // decoder will walk 4 bands
        let frames = vec![Sv7EncStereoFrame::silent(2)];
        assert_eq!(
            encode_sv7_file(&hdr, &frames),
            Err(Error::MaxBandOutOfRange(1)),
        );
    }

    #[test]
    fn propagates_header_validation_failure() {
        let mut hdr = header(0, 5, false);
        hdr.profile = 16;
        assert_eq!(
            encode_sv7_file(&hdr, &[]),
            Err(Error::HeaderFieldOutOfRange("profile")),
        );
    }

    #[test]
    fn zero_frame_file_is_just_the_header() {
        let hdr = header(0, 5, false);
        let raw = encode_sv7_file(&hdr, &[]).unwrap();
        assert_eq!(raw.len(), crate::sv7_header_encode::SV7_HEADER_DISK_LEN);
        assert_eq!(Sv7HeaderFields::parse(&raw).unwrap(), hdr);
    }

    /// Two mixed frames over three bands for builder-vs-one-shot tests.
    fn builder_frames() -> Vec<Sv7EncStereoFrame> {
        let coded = |scf0: i32| Sv7EncBand::Coded {
            band_type: 3,
            ctx: 0,
            scf: [scf0, scf0 + 1, scf0],
            levels: q3_levels(),
        };
        vec![
            Sv7EncStereoFrame {
                left: vec![
                    coded(7),
                    Sv7EncBand::Cns { scf: [3, 3, 3] },
                    Sv7EncBand::Empty,
                ],
                right: vec![
                    coded(9),
                    Sv7EncBand::Empty,
                    Sv7EncBand::Cns { scf: [2, 2, 2] },
                ],
                ms_flags: vec![true, false, false],
            },
            Sv7EncStereoFrame {
                left: vec![
                    Sv7EncBand::Empty,
                    coded(12),
                    Sv7EncBand::Cns { scf: [5, 5, 5] },
                ],
                right: vec![
                    coded(5),
                    Sv7EncBand::Cns { scf: [7, 7, 7] },
                    Sv7EncBand::Empty,
                ],
                ms_flags: vec![false, true, true],
            },
        ]
    }

    #[test]
    fn builder_output_is_byte_identical_to_one_shot() {
        let mut hdr = header(2, 2, true);
        hdr.true_gapless = false;
        let frames = builder_frames();
        let one_shot = encode_sv7_file(&hdr, &frames).unwrap();

        // The builder's template frame_count is ignored; set it wrong on
        // purpose to prove the override.
        let mut template = hdr;
        template.frame_count = 999;
        let mut w = Sv7FileWriter::new(template).unwrap();
        for f in &frames {
            w.push_frame(f).unwrap();
        }
        assert_eq!(w.frames_pushed(), 2);
        let built = w.finish().unwrap();

        assert_eq!(built, one_shot);
    }

    #[test]
    fn builder_zero_frames_is_header_only() {
        let hdr = header(0, 5, false);
        let built = Sv7FileWriter::new(hdr).unwrap().finish().unwrap();
        assert_eq!(built, encode_sv7_file(&hdr, &[]).unwrap());
    }

    #[test]
    fn builder_rejects_bad_template_immediately() {
        let mut hdr = header(0, 5, false);
        hdr.link = 4;
        assert_eq!(
            Sv7FileWriter::new(hdr).err(),
            Some(Error::HeaderFieldOutOfRange("link")),
        );
        assert_eq!(
            Sv7FileWriter::with_version(header(0, 5, false), 0x08).err(),
            Some(Error::UnsupportedVersion(0x08)),
        );
    }

    #[test]
    fn builder_rejects_wrong_band_count_frame() {
        let mut w = Sv7FileWriter::new(header(0, 3, false)).unwrap();
        assert_eq!(
            w.push_frame(&Sv7EncStereoFrame::silent(2)),
            Err(Error::MaxBandOutOfRange(1)),
        );
        assert_eq!(w.frames_pushed(), 0);
    }

    #[test]
    fn builder_finish_gapless_sets_fields_13_and_14_and_trailer() {
        let mut w = Sv7FileWriter::new(header(0, 1, false)).unwrap();
        for _ in 0..3 {
            w.push_frame(&Sv7EncStereoFrame::silent(2)).unwrap();
        }
        let raw = w.finish_gapless(500).unwrap();
        let parsed = Sv7HeaderFields::parse(&raw).unwrap();
        assert_eq!(parsed.frame_count, 3);
        assert!(parsed.true_gapless);
        assert_eq!(parsed.last_frame_samples, 500);

        // The 11-bit trailer mirrors field 14.
        let swapped = crate::sv7_word_swap::word_swap_sv7_body(&raw);
        let mut r = Sv7BitReader::new(&swapped);
        skip_bits(&mut r, SV7_HEADER_BITS);
        for _ in 0..3 {
            let len = read20(&mut r);
            skip_bits(&mut r, u64::from(len));
        }
        assert_eq!(r.read_bits(11).unwrap(), 500);
    }

    #[test]
    fn builder_finish_gapless_rejects_count_above_frame_geometry() {
        let w = Sv7FileWriter::new(header(0, 1, false)).unwrap();
        assert_eq!(
            w.finish_gapless(1153).err(),
            Some(Error::HeaderFieldOutOfRange("last_frame_samples")),
        );
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
