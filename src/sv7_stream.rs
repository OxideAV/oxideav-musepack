//! SV7 multi-frame stream driver: frames → PCM with persistent state.
//!
//! [`crate::sv7_stereo_frame::decode_sv7_stereo_frame`] decodes **one**
//! frame body into a pre-M/S-undo [`crate::ms_stereo::StereoSubbandMatrix`].
//! A whole SV7 stream is a run of such frames, and turning that run into
//! continuous PCM needs three things to thread *across* frame boundaries:
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
//! 3. **The bit reader.** SV7 frames are *not* byte-aligned (§2.2): the
//!    body is one continuous non-aligned bit run. The driver decodes
//!    every frame from the **same** [`crate::huffman::Sv7BitReader`], so
//!    each frame resumes exactly where the previous left off.
//!
//! # The M/S-undo arithmetic stays a caller closure (GAP)
//!
//! §2.6's M/S-undo step is gated per band by the §5.1 `msflag` (decoded
//! into [`crate::sv7_stereo_frame::Sv7StereoFrame::ms_flags`]) but the
//! per-sample mid/side → left/right arithmetic is **not specified under
//! `docs/audio/musepack/`** — the long-standing DOCS-GAP this crate
//! threads as a [`crate::ms_stereo::undo_ms_stereo`] closure. The driver
//! takes that closure once at construction and applies it to every
//! frame, so a single edit wires the real arithmetic when a trace lands.
//!
//! # The SV7 body bit-alignment is NOT assumed here
//!
//! Per §2.2 / §4 the SV7 audio body is a continuous bit run packed in
//! 32-bit little-endian word units that are byte-swapped before the bit
//! reader sees them, and the exact word-grid offset at which the body
//! begins relative to the 200-bit fixed header is **not pinned by the
//! staged material** (no SV7 fixture corpus exists in-repo to validate a
//! whole-stream word-swap boundary). This driver therefore takes a
//! [`crate::huffman::Sv7BitReader`] the caller has **already positioned**
//! at the frame body — it owns the per-frame *decode loop* and the
//! *persistent state*, not the byte-level body extraction. The header
//! parse ([`crate::sv7_header::Sv7HeaderFields::parse`]) and the
//! whole-stream word-swap that produces the body reader are the caller's
//! (and a future round's, once a fixture pins the boundary).
//!
//! Source-of-record (facts only):
//!
//! - `docs/audio/musepack/musepack-sv7-sv8-spec.md` §1 (filterbank
//!   overlap, 1152-sample frame), §2.2 (frames not byte-aligned), §2.5
//!   (CNS), §2.6 (reconstruction step order).
//! - `docs/audio/musepack/spec/musepack-headers-and-coding.md` §1
//!   (`max_band`, stream M/S, stereo-only), §4 (word packing), §5.

use crate::cns::CnsPrng;
use crate::huffman::Sv7BitReader;
use crate::ms_stereo::undo_ms_stereo;
use crate::sv7_stereo_frame::decode_sv7_stereo_frame;
use crate::synthesis::{synthesize_stereo_frame_interleaved, MultiChannelSynthesis};
use crate::{Error, Result, SAMPLES_PER_FRAME_PER_CHANNEL};

/// PCM samples produced by one stereo frame, interleaved `L, R, …`
/// (`2 × 1152` values).
pub const STEREO_FRAME_PCM_LEN: usize = 2 * SAMPLES_PER_FRAME_PER_CHANNEL;

/// A persistent SV7 stereo-stream decoder: it owns the cross-frame state
/// (the two synthesis filters, the CNS PRNG, the M/S-undo closure) and
/// decodes frames one at a time from a caller-positioned bit reader into
/// interleaved PCM.
///
/// The decoder is parameterised by the M/S-undo closure `U` (the §2.6
/// GAP arithmetic, see the module docs). Construct it once for a stream,
/// then call [`Sv7StreamDecoder::decode_frame`] per frame; the synthesis
/// overlap and PRNG state thread automatically.
pub struct Sv7StreamDecoder<U>
where
    U: Fn(f64, f64) -> (f64, f64),
{
    /// Highest coded subband (`1..=31`, SV7 fixed-header field 4).
    max_band: u8,
    /// Stream-wide M/S enable (SV7 fixed-header field 3).
    stream_ms: bool,
    /// §2.6 absolute SCF anchor — GAP, threaded as a constant (`0` for
    /// the relative-loudness convention).
    anchor: u8,
    /// Persistent two-channel synthesis state (filterbank overlap).
    synthesis: MultiChannelSynthesis,
    /// Shared free-running CNS PRNG.
    cns: CnsPrng,
    /// The §2.6 M/S-undo per-sample arithmetic (GAP closure).
    undo: U,
    /// Count of frames decoded so far (for diagnostics / total-sample
    /// bookkeeping by the caller).
    frames_decoded: u64,
}

impl<U> Sv7StreamDecoder<U>
where
    U: Fn(f64, f64) -> (f64, f64),
{
    /// Build a stream decoder for an SV7 stereo stream.
    ///
    /// `max_band` / `stream_ms` come from the SV7 fixed header
    /// ([`crate::sv7_header::Sv7HeaderFields::max_band`] / `mid_side`).
    /// `anchor` is the §2.6 absolute SCF anchor (GAP; pass `0`). `undo`
    /// is the per-band M/S-undo arithmetic closure (GAP).
    ///
    /// # Errors
    ///
    /// - [`Error::MaxBandOutOfRange`] if `max_band` exceeds the §1
    ///   Layer-II 32-subband inclusive bound (31).
    pub fn new(max_band: u8, stream_ms: bool, anchor: u8, undo: U) -> Result<Self> {
        if max_band > crate::sv7_band_header::SV7_MAX_BAND_INCLUSIVE {
            return Err(Error::MaxBandOutOfRange(max_band));
        }
        Ok(Self {
            max_band,
            stream_ms,
            anchor,
            // SV7 is stereo-only (§1 derived fact).
            synthesis: MultiChannelSynthesis::new(2)?,
            cns: CnsPrng::new(),
            undo,
            frames_decoded: 0,
        })
    }

    /// The number of frames decoded so far.
    #[must_use]
    pub fn frames_decoded(&self) -> u64 {
        self.frames_decoded
    }

    /// Reset the synthesis overlap and PRNG to their startup state (e.g.
    /// at a stream seek). Does not change `max_band` / `stream_ms` /
    /// `anchor`.
    pub fn reset(&mut self) {
        self.synthesis.reset();
        self.cns = CnsPrng::new();
        self.frames_decoded = 0;
    }

    /// Decode one stereo frame from `reader` (positioned at the frame
    /// body) into [`STEREO_FRAME_PCM_LEN`] interleaved `L, R, …` PCM
    /// samples, advancing the persistent synthesis / PRNG state.
    ///
    /// The full per-frame pipeline (§2.6 step order):
    ///
    /// 1. §5 stereo frame decode + per-channel reconstruction
    ///    ([`decode_sv7_stereo_frame`]) → pre-M/S-undo channel matrices +
    ///    per-band `ms_flags`;
    /// 2. §2.6 M/S-undo over the flagged subbands (the GAP closure);
    /// 3. §2.6 synthesis filterbank, both channels through their
    ///    persistent filters, interleaved.
    ///
    /// # Errors
    ///
    /// Propagates every error of [`decode_sv7_stereo_frame`] (reader
    /// starvation, no-match VLC, out-of-range band-type / SCFI) and of
    /// the synthesis interleave.
    pub fn decode_frame(&mut self, reader: &mut Sv7BitReader<'_>) -> Result<Vec<f64>> {
        // 1. Decode + per-channel reconstruct (pre-M/S-undo).
        let mut frame = decode_sv7_stereo_frame(
            reader,
            self.max_band,
            self.stream_ms,
            // §2.6 SCF anchor (GAP); the same value seeds each channel.
            self.anchor as i32,
            self.anchor,
            &mut self.cns,
        )?;

        // 2. §2.6 M/S undo over the flagged subbands (GAP closure).
        undo_ms_stereo(&mut frame.channels, &frame.ms_flags, &self.undo)?;

        // 3. §2.6 synthesis filterbank, interleaved, persistent overlap.
        let pcm = synthesize_stereo_frame_interleaved(
            &mut self.synthesis,
            &frame.channels[0],
            &frame.channels[1],
        )?;

        self.frames_decoded += 1;
        Ok(pcm)
    }

    /// Decode up to `max_frames` frames from `reader`, concatenating the
    /// interleaved PCM. Stops early (without error) when `reader` no
    /// longer has the bits for another frame body — the natural
    /// end-of-stream for a continuous SV7 bit run.
    ///
    /// Returns the concatenated `L, R, …` PCM. Use [`Self::frames_decoded`]
    /// to recover how many frames were actually decoded.
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

    /// A representative `L = M + S` / `R = M − S` undo used only in
    /// tests (not a claim about the GAP Musepack arithmetic).
    fn test_undo(m: f64, s: f64) -> (f64, f64) {
        (m + s, m - s)
    }

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
        let err = Sv7StreamDecoder::new(32, false, 0, test_undo).err();
        assert_eq!(err, Some(Error::MaxBandOutOfRange(32)));
    }

    #[test]
    fn new_succeeds_for_valid_max_band() {
        let dec = Sv7StreamDecoder::new(20, true, 0, test_undo).unwrap();
        assert_eq!(dec.frames_decoded(), 0);
    }

    #[test]
    fn single_silent_frame_yields_silent_pcm() {
        let mut p = Packer::new();
        silent_frame_bits(&mut p);
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let mut dec = Sv7StreamDecoder::new(0, false, 0, test_undo).unwrap();
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
        let mut dec = Sv7StreamDecoder::new(0, false, 0, test_undo).unwrap();
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
        let mut dec = Sv7StreamDecoder::new(0, false, 0, test_undo).unwrap();
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
        let mut dec = Sv7StreamDecoder::new(0, false, 0, test_undo).unwrap();
        dec.decode_frame(&mut r).unwrap();
        assert_eq!(dec.frames_decoded(), 1);
        dec.reset();
        assert_eq!(dec.frames_decoded(), 0);
    }

    #[test]
    fn pcm_len_constant_is_two_channels_of_a_frame() {
        assert_eq!(STEREO_FRAME_PCM_LEN, 2 * 1152);
    }
}
