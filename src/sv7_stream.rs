//! SV7 multi-frame stream driver: frames → PCM with persistent state.
//!
//! [`crate::sv7_stereo_frame::decode_sv7_stereo_frame`] decodes **one**
//! frame body into a pre-M/S-undo [`crate::ms_stereo::StereoSubbandMatrix`].
//! A whole SV7 stream is a run of such frames, and turning that run into
//! continuous PCM needs four things to thread *across* frame boundaries:
//!
//! 1. **The synthesis filterbank overlap.** The 32-band polyphase
//!    synthesis window (§1 / §2.6) reaches back into the `V` blocks
//!    matrixed during the previous 15 frames. Resetting the filter
//!    between frames would zero that overlap and inject a click every
//!    1152 samples. [`Sv7StreamDecoder`] holds one persistent
//!    [`crate::synthesis::MultiChannelSynthesis`] (two filters) reused
//!    across every frame.
//! 2. **The CNS PRNG.** The noise-substitution generator (§2.5 case −1)
//!    is a free-running two-LFSR PRNG whose state advances with every
//!    noise band of every frame, both channels. A single shared
//!    [`crate::cns::CnsPrng`] threads it.
//! 3. **The per-band SCF memory.** The corpus-pinned `SCF[0]` reference
//!    is the same subband's previous-frame `SCF[2]`
//!    ([`crate::sv7_stereo_frame::Sv7ScfMemory`]) — inherently
//!    cross-frame state.
//! 4. **The bit reader.** SV7 frame bodies are *not* byte-aligned
//!    (§2.2): each body is a non-aligned bit run. The driver decodes
//!    every frame from the **same** [`crate::huffman::Sv7BitReader`], so
//!    each frame resumes exactly where the previous left off. (On the
//!    wire each body is *preceded by a 20-bit bit-length prefix* — the
//!    whole-file layer [`crate::sv7_file_decode`] consumes those and
//!    verifies each frame against its budget; this driver decodes
//!    bodies only.)
//!
//! The §2.6 M/S undo uses the corpus-pinned arithmetic
//! ([`crate::ms_stereo::undo_ms_stereo_pinned`]: `L = M + S`,
//! `R = M − S`), and the reconstruction is the corpus-pinned absolute
//! law, so the produced PCM is in the **signed-16-bit domain** — round
//! and clamp to `i16` for playback (see
//! [`crate::sv7_file_decode::Sv7DecodedFile::pcm_s16`]) — with the
//! four state carriers above threading every frame.
//!
//! Source-of-record: `docs/audio/musepack/musepack-sv7-sv8-spec.md` §1
//! (filterbank overlap, 1152-sample frame), §2.2 (frames not
//! byte-aligned), §2.5 (CNS), §2.6 (reconstruction step order);
//! `docs/audio/musepack/spec/musepack-headers-and-coding.md` §1
//! (`max_band`, stream M/S, stereo-only), §4 (word packing), §5; pass
//! layout + SCF memory + M/S arithmetic fixture-corpus-pinned
//! ([`crate::sv7_stereo_frame`]).

use crate::cns::CnsPrng;
use crate::huffman::Sv7BitReader;
use crate::ms_stereo::undo_ms_stereo_pinned;
use crate::sv7_stereo_frame::{decode_sv7_stereo_frame, Sv7ScfMemory};
use crate::synthesis::{synthesize_stereo_frame_interleaved, MultiChannelSynthesis};
use crate::{Error, Result, SAMPLES_PER_FRAME_PER_CHANNEL};

/// PCM samples produced by one stereo frame, interleaved `L, R, …`
/// (`2 × 1152` values).
pub const STEREO_FRAME_PCM_LEN: usize = 2 * SAMPLES_PER_FRAME_PER_CHANNEL;

/// A persistent SV7 stereo-stream decoder: it owns the cross-frame state
/// (the two synthesis filters, the CNS PRNG, the per-band SCF memory)
/// and decodes frame bodies one at a time from a caller-positioned bit
/// reader into interleaved s16-domain PCM.
pub struct Sv7StreamDecoder {
    /// Highest coded subband (`1..=31`, SV7 fixed-header field 4).
    max_band: u8,
    /// Stream-wide M/S enable (SV7 fixed-header field 3).
    stream_ms: bool,
    /// Persistent two-channel synthesis state (filterbank overlap).
    synthesis: MultiChannelSynthesis,
    /// Shared free-running CNS PRNG.
    cns: CnsPrng,
    /// Corpus-pinned per-band cross-frame SCF memory.
    scf: Sv7ScfMemory,
    /// Count of frames decoded so far (for diagnostics / total-sample
    /// bookkeeping by the caller).
    frames_decoded: u64,
}

impl Sv7StreamDecoder {
    /// Build a stream decoder for an SV7 stereo stream.
    ///
    /// `max_band` / `stream_ms` come from the SV7 fixed header
    /// ([`crate::sv7_header::Sv7HeaderFields::max_band`] / `mid_side`).
    ///
    /// # Errors
    ///
    /// - [`Error::MaxBandOutOfRange`] if `max_band` exceeds the §1
    ///   Layer-II 32-subband inclusive bound (31).
    pub fn new(max_band: u8, stream_ms: bool) -> Result<Self> {
        if max_band > crate::sv7_band_header::SV7_MAX_BAND_INCLUSIVE {
            return Err(Error::MaxBandOutOfRange(max_band));
        }
        Ok(Self {
            max_band,
            stream_ms,
            // SV7 is stereo-only (§1 derived fact).
            synthesis: MultiChannelSynthesis::new(2)?,
            cns: CnsPrng::new(),
            scf: Sv7ScfMemory::new(),
            frames_decoded: 0,
        })
    }

    /// Build a stream decoder straight from a parsed SV7 fixed header,
    /// pulling `max_band` and the stream-wide M/S flag from the header
    /// fields (§1, fields 4 and 3).
    ///
    /// # Errors
    ///
    /// - [`Error::MaxBandOutOfRange`] if the header's `max_band` exceeds
    ///   the §1 bound (a header from `parse` has already passed this
    ///   gate, so this only fires for a hand-constructed `header`).
    pub fn from_header(header: &crate::sv7_header::Sv7HeaderFields) -> Result<Self> {
        Self::new(header.max_band, header.mid_side)
    }

    /// The number of frames decoded so far.
    #[must_use]
    pub fn frames_decoded(&self) -> u64 {
        self.frames_decoded
    }

    /// Reset the synthesis overlap, PRNG, and SCF memory to their
    /// stream-start state (e.g. at a stream seek). Does not change
    /// `max_band` / `stream_ms`.
    pub fn reset(&mut self) {
        self.synthesis.reset();
        self.cns = CnsPrng::new();
        self.scf.reset();
        self.frames_decoded = 0;
    }

    /// Decode one stereo frame body from `reader` (positioned at the
    /// body's first bit, after any length prefix) into
    /// [`STEREO_FRAME_PCM_LEN`] interleaved `L, R, …` s16-domain PCM
    /// samples, advancing the persistent synthesis / PRNG / SCF state.
    ///
    /// The full per-frame pipeline (§2.6 step order):
    ///
    /// 1. the corpus-pinned four-pass frame decode + absolute
    ///    reconstruction ([`decode_sv7_stereo_frame`]);
    /// 2. the pinned §2.6 M/S undo over the flagged subbands;
    /// 3. the §2.6 synthesis filterbank, both channels through their
    ///    persistent filters, interleaved.
    ///
    /// # Errors
    ///
    /// Propagates every error of [`decode_sv7_stereo_frame`] (reader
    /// starvation, no-match VLC, out-of-range band-type / SCFI) and of
    /// the synthesis interleave.
    pub fn decode_frame(&mut self, reader: &mut Sv7BitReader<'_>) -> Result<Vec<f64>> {
        // 1. Four-pass decode + absolute reconstruction.
        let mut frame = decode_sv7_stereo_frame(
            reader,
            self.max_band,
            self.stream_ms,
            &mut self.scf,
            &mut self.cns,
        )?;

        // 2. Pinned §2.6 M/S undo over the flagged subbands.
        undo_ms_stereo_pinned(&mut frame.channels, &frame.ms_flags)?;

        // 3. §2.6 synthesis filterbank, interleaved, persistent overlap.
        let pcm = synthesize_stereo_frame_interleaved(
            &mut self.synthesis,
            &frame.channels[0],
            &frame.channels[1],
        )?;

        self.frames_decoded += 1;
        Ok(pcm)
    }

    /// Decode up to `max_frames` frame bodies from `reader`,
    /// concatenating the interleaved PCM. Stops early (without error)
    /// when `reader` no longer has the bits for another frame body.
    ///
    /// **Note:** this entry point expects *back-to-back bodies with no
    /// 20-bit length prefixes* (the crate's pre-corpus self-framing, and
    /// the natural shape for synthetic body runs in tests). Real `.mpc`
    /// files prefix every body — use
    /// [`crate::sv7_file_decode::decode_sv7_file`] for those.
    ///
    /// # Errors
    ///
    /// Propagates a mid-frame decode error (a frame that started but
    /// could not finish) — distinct from the clean "no more frames"
    /// stop, which returns the PCM decoded so far.
    pub fn decode_frames(
        &mut self,
        reader: &mut Sv7BitReader<'_>,
        max_frames: u64,
    ) -> Result<Vec<f64>> {
        let mut pcm = Vec::new();
        for _ in 0..max_frames {
            if reader.is_empty() {
                break;
            }
            match self.decode_frame(reader) {
                Ok(frame_pcm) => pcm.extend_from_slice(&frame_pcm),
                // A frame that began but starved mid-decode at the very
                // end of the stream is a clean stop, not a hard error:
                // the continuous bit run ran out between frames.
                Err(Error::UnexpectedEof) if reader.is_empty() => break,
                Err(e) => return Err(e),
            }
        }
        Ok(pcm)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// MSB-first bit packer (mirrors the frame-decode tests).
    struct Packer {
        bytes: Vec<u8>,
        acc: u32,
        nbits: u8,
    }

    impl Packer {
        fn new() -> Self {
            Packer {
                bytes: Vec::new(),
                acc: 0,
                nbits: 0,
            }
        }
        fn push_raw(&mut self, value: u32, length: u8) {
            for i in (0..length).rev() {
                let bit = (value >> i) & 1;
                self.acc = (self.acc << 1) | bit;
                self.nbits += 1;
                if self.nbits == 8 {
                    self.bytes.push(self.acc as u8);
                    self.acc = 0;
                    self.nbits = 0;
                }
            }
        }
        fn finish(mut self) -> Vec<u8> {
            if self.nbits > 0 {
                self.bytes.push((self.acc << (8 - self.nbits)) as u8);
            }
            for _ in 0..4 {
                self.bytes.push(0);
            }
            self.bytes
        }
    }

    /// One all-silent stereo frame body for `max_band == 0`: both
    /// channels' band-0 raw-4-bit Res == 0, stream M/S off.
    fn silent_frame_bits(p: &mut Packer) {
        p.push_raw(0, 4); // left band-0 Res = 0
        p.push_raw(0, 4); // right band-0 Res = 0
    }

    #[test]
    fn new_rejects_max_band_out_of_range() {
        let err = Sv7StreamDecoder::new(32, false).err();
        assert_eq!(err, Some(Error::MaxBandOutOfRange(32)));
    }

    #[test]
    fn new_succeeds_for_valid_max_band() {
        let dec = Sv7StreamDecoder::new(20, true).unwrap();
        assert_eq!(dec.frames_decoded(), 0);
    }

    #[test]
    fn single_silent_frame_yields_silent_pcm() {
        let mut p = Packer::new();
        silent_frame_bits(&mut p);
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let mut dec = Sv7StreamDecoder::new(0, false).unwrap();
        let pcm = dec.decode_frame(&mut r).unwrap();
        assert_eq!(pcm.len(), STEREO_FRAME_PCM_LEN);
        assert!(pcm.iter().all(|&s| s == 0.0));
        assert_eq!(dec.frames_decoded(), 1);
    }

    #[test]
    fn multi_silent_frame_overlap_stays_silent_and_counts() {
        // Three back-to-back silent frames in one continuous reader.
        let mut p = Packer::new();
        for _ in 0..3 {
            silent_frame_bits(&mut p);
        }
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let mut dec = Sv7StreamDecoder::new(0, false).unwrap();
        let mut total = 0;
        for _ in 0..3 {
            let pcm = dec.decode_frame(&mut r).unwrap();
            assert!(pcm.iter().all(|&s| s == 0.0));
            total += pcm.len();
        }
        assert_eq!(total, 3 * STEREO_FRAME_PCM_LEN);
        assert_eq!(dec.frames_decoded(), 3);
    }

    #[test]
    fn decode_frames_concatenates_and_stops_at_eof() {
        let mut p = Packer::new();
        for _ in 0..2 {
            silent_frame_bits(&mut p);
        }
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let mut dec = Sv7StreamDecoder::new(0, false).unwrap();
        // Ask for many more frames than the stream carries; it stops
        // cleanly when the bits run out.
        let pcm = dec.decode_frames(&mut r, 100).unwrap();
        // At least the two silent frames decoded; trailing zero padding
        // may admit additional all-silence frames before the reader is
        // exhausted, so assert a lower bound and silence.
        assert!(pcm.len() >= 2 * STEREO_FRAME_PCM_LEN);
        assert_eq!(pcm.len() % STEREO_FRAME_PCM_LEN, 0);
        assert!(pcm.iter().all(|&s| s == 0.0));
        assert!(dec.frames_decoded() >= 2);
    }

    #[test]
    fn reset_clears_frame_count_and_state() {
        let mut p = Packer::new();
        silent_frame_bits(&mut p);
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let mut dec = Sv7StreamDecoder::new(0, false).unwrap();
        dec.decode_frame(&mut r).unwrap();
        assert_eq!(dec.frames_decoded(), 1);
        dec.reset();
        assert_eq!(dec.frames_decoded(), 0);
        assert_eq!(dec.scf, Sv7ScfMemory::new());
    }

    #[test]
    fn pcm_len_constant_is_two_channels_of_a_frame() {
        assert_eq!(STEREO_FRAME_PCM_LEN, 2 * 1152);
    }

    #[test]
    fn from_header_pulls_max_band_and_ms_flag() {
        use crate::sv7_header::Sv7HeaderFields;
        let header = Sv7HeaderFields {
            max_band: 17,
            mid_side: true,
            ..Default::default()
        };
        let dec = Sv7StreamDecoder::from_header(&header).unwrap();
        assert_eq!(dec.max_band, 17);
        assert!(dec.stream_ms);
        assert_eq!(dec.frames_decoded(), 0);
    }

    #[test]
    fn from_header_rejects_out_of_range_max_band() {
        use crate::sv7_header::Sv7HeaderFields;
        let header = Sv7HeaderFields {
            max_band: 32,
            ..Default::default()
        };
        let err = Sv7StreamDecoder::from_header(&header).err();
        assert_eq!(err, Some(Error::MaxBandOutOfRange(32)));
    }
}
