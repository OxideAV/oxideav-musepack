//! SV8 stream drivers: frames → PCM with persistent state.
//!
//! Two drivers live here:
//!
//! - [`Sv8StreamDecoder`] — the **real-stream** driver (round 419):
//!   whole `AP` packets in the fixture-pinned two-channel frame layout
//!   ([`crate::sv8_stereo_frame`]), M/S undo, absolute loudness,
//!   interleaved or mono output. This is the path
//!   [`crate::sv8_decode::decode_sv8_stream`] drives.
//! - [`Sv8MonoStreamDecoder`] — the earlier **single-channel-body**
//!   primitive, retained for callers decoding a synthetic one-channel
//!   frame run (real SV8 streams always code two channels per body —
//!   see [`crate::sv8_stereo_frame`] — so this is not a real-stream
//!   path).
//!
//! The SV8 counterpart of [`crate::sv7_stream`]. An SV8
//! audio (`AP`) packet carries
//! [`crate::sh_header::StreamHeaderFields::frames_per_audio_packet`]
//! frames, and turning that run into continuous PCM needs the same two
//! pieces of state threaded across frame boundaries that the SV7 driver
//! threads:
//!
//! 1. **The synthesis filterbank overlap.** SV8 reuses the SV7 / Layer-II
//!    32-band polyphase synthesis filter unchanged (spec §3, §1), whose
//!    window reaches back 15 frames; one persistent
//!    [`crate::synthesis::SynthesisFilter`] is reused across every frame
//!    so the overlap is continuous (no click every 1152 samples).
//! 2. **The CNS PRNG.** The noise-substitution generator (§6.4 / §7
//!    `Res == -1`) is the same free-running two-LFSR PRNG shared with
//!    SV7; a single [`crate::cns::CnsPrng`] threads it across frames.
//!
//! Unlike SV7, SV8 is **byte-natural** (no 32-LSB word swap, §4) and an
//! `AP` packet payload is a self-contained byte slice, so the per-frame
//! bit reader is a single [`crate::huffman::Sv7BitReader`] over the `AP`
//! payload — the same continuous-bit-run discipline, but without the
//! word-swap subtlety the SV7 body carries.
//!
//! # Per-frame `nbands` / `new_block` are caller-supplied
//!
//! §6.2 reads each frame's `Max_used_Band` (the keyframe log code
//! [`crate::sv8_band_header::decode_keyframe_max_used_band`] or the
//! non-key delta [`crate::sv8_band_header::decode_nonkey_max_used_band`])
//! and forces the per-band "new-block" flag set on a key frame. The
//! exact per-frame *position* of those reads within an `AP` packet
//! (once-per-packet vs. per-frame, and the keyframe → non-key transition
//! inside a multi-frame packet) is not pinned cell-for-cell by the
//! staged material. This driver therefore takes `(nbands, new_block)`
//! **per frame** from the caller — exactly as the underlying
//! [`crate::sv8_reconstruct::decode_and_reconstruct_sv8_channel`] does —
//! and owns the per-frame decode + persistent-state threading. The
//! caller (who has resolved the keyframe structure) supplies the
//! schedule.
//!
//! # [`Sv8MonoStreamDecoder`] scope: single-channel bodies only
//!
//! The mono driver walks **one channel** per frame body. Real SV8
//! streams always code two channels per body (fixture-pinned, r419 —
//! [`crate::sv8_stereo_frame`]), so this primitive serves synthetic
//! single-channel runs and per-channel experiments;
//! [`Sv8StreamDecoder`] is the real-stream path.
//!
//! Source-of-record (facts only):
//!
//! - `docs/audio/musepack/musepack-sv7-sv8-spec.md` §1 (filterbank
//!   overlap), §3 (SV8 reuses the SV7 signal path / filterbank), §3.1
//!   (`AP` packet), §3.4 (CNS), §2.6 (reconstruction order).
//! - `docs/audio/musepack/spec/musepack-headers-and-coding.md` §2
//!   (`block_power` → frames-per-`AP`), §4 (SV8 byte-natural), §6.

use crate::cns::CnsPrng;
use crate::huffman::Sv7BitReader;
use crate::ms_stereo::undo_ms_stereo_pinned;
use crate::sh_header::StreamHeaderFields;
use crate::sv8_reconstruct::decode_and_reconstruct_sv8_channel;
use crate::sv8_stereo_frame::{
    decode_sv8_stereo_frame, reconstruct_sv8_stereo_frame, Sv8FrameState,
};
use crate::synthesis::{synthesize_frame_channel, MultiChannelSynthesis, SynthesisFilter};
use crate::{Error, Result, SAMPLES_PER_FRAME_PER_CHANNEL};

/// One frame's §6.2 decode parameters: the used-band count and the §6.3
/// per-band new-block flag (set on a key frame).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Sv8FrameParams {
    /// §6.2 `Max_used_Band` — the count of coded bands for this frame.
    pub nbands: u8,
    /// §6.3 "new-block" flag (scalefactors coded absolutely; forced set
    /// on a key frame).
    pub new_block: bool,
}

/// A persistent SV8 **mono** stream decoder: it owns the cross-frame
/// state (one synthesis filter, the CNS PRNG) and decodes frames one at a
/// time from a caller-positioned bit reader into PCM.
///
/// Construct it once for a stream / channel, then call
/// [`Sv8MonoStreamDecoder::decode_frame`] per frame with that frame's
/// [`Sv8FrameParams`]; the synthesis overlap and PRNG state thread
/// automatically.
#[derive(Debug, Clone)]
pub struct Sv8MonoStreamDecoder {
    /// §2.6 absolute SCF anchor — GAP, threaded as a constant (`0` for
    /// the relative-loudness convention).
    anchor: i32,
    /// Persistent single-channel synthesis state (filterbank overlap).
    filter: SynthesisFilter,
    /// Shared free-running CNS PRNG.
    cns: CnsPrng,
    /// Count of frames decoded so far.
    frames_decoded: u64,
}

impl Sv8MonoStreamDecoder {
    /// Build a mono stream decoder. `anchor` is the §2.6 absolute SCF
    /// anchor (GAP; pass `0` for the relative-loudness convention).
    #[must_use]
    pub fn new(anchor: i32) -> Self {
        Self {
            anchor,
            filter: SynthesisFilter::new(),
            cns: CnsPrng::new(),
            frames_decoded: 0,
        }
    }

    /// The number of frames decoded so far.
    #[must_use]
    pub fn frames_decoded(&self) -> u64 {
        self.frames_decoded
    }

    /// Reset the synthesis overlap and PRNG to their startup state (e.g.
    /// at a stream seek / keyframe boundary). Does not change `anchor`.
    pub fn reset(&mut self) {
        self.filter.reset();
        self.cns = CnsPrng::new();
        self.frames_decoded = 0;
    }

    /// Decode one mono frame from `reader` (positioned at the frame body
    /// within the `AP` payload) into [`SAMPLES_PER_FRAME_PER_CHANNEL`]
    /// PCM samples, advancing the persistent synthesis / PRNG state.
    ///
    /// Pipeline: §6 frame-body decode + §2.6/§3.6 reconstruction
    /// ([`decode_and_reconstruct_sv8_channel`]) → §2.6 synthesis
    /// filterbank through the persistent filter.
    ///
    /// # Errors
    ///
    /// Propagates every error of
    /// [`decode_and_reconstruct_sv8_channel`] (band-resolution walk,
    /// SCFI / DSCF decode, sample decode, EOF, out-of-range
    /// band-type / subband).
    pub fn decode_frame(
        &mut self,
        reader: &mut Sv7BitReader<'_>,
        params: Sv8FrameParams,
    ) -> Result<[f64; SAMPLES_PER_FRAME_PER_CHANNEL]> {
        let matrix = decode_and_reconstruct_sv8_channel(
            reader,
            params.nbands,
            params.new_block,
            &mut self.cns,
            self.anchor,
        )?;
        let pcm = synthesize_frame_channel(&mut self.filter, &matrix);
        self.frames_decoded += 1;
        Ok(pcm)
    }

    /// Decode a run of frames whose per-frame [`Sv8FrameParams`] are given
    /// in `schedule`, concatenating the PCM. One entry per frame (e.g.
    /// the [`crate::sh_header::StreamHeaderFields::frames_per_audio_packet`]
    /// frames of an `AP` packet).
    ///
    /// # Errors
    ///
    /// Propagates a mid-frame decode error from [`Self::decode_frame`].
    pub fn decode_frames(
        &mut self,
        reader: &mut Sv7BitReader<'_>,
        schedule: &[Sv8FrameParams],
    ) -> Result<Vec<f64>> {
        let mut pcm = Vec::with_capacity(schedule.len() * SAMPLES_PER_FRAME_PER_CHANNEL);
        for &params in schedule {
            let frame = self.decode_frame(reader, params)?;
            pcm.extend_from_slice(&frame);
        }
        Ok(pcm)
    }
}

/// The **real-stream** SV8 driver (round 419): decodes whole `AP`
/// packets of a stream in the fixture-pinned two-channel frame layout
/// ([`crate::sv8_stereo_frame`]), threading every piece of cross-frame
/// state, and emits absolute s16-domain PCM.
///
/// Owns:
///
/// - the two-channel synthesis filterbank state (the body always codes
///   two channels — fixture-pinned — and the overlap spans 15 frames,
///   so both filters persist across frames *and* packets);
/// - the free-running CNS PRNG (§7, shared across bands / channels /
///   frames in decode order, as in SV7);
/// - the [`Sv8FrameState`] SCF memory + `Max_used_Band` reference,
///   **reset at every `AP` boundary**: each packet opens with a key
///   frame whose scalefactors are coded absolutely (spec §3.3
///   keyframes; fixture-pinned — the multi-packet corpus stream decodes
///   bit-exact under the reset and desynchronises without it).
///
/// Output-channel mapping: the frame body is always two channels; the
/// `SH` channel count selects the output — `2` emits interleaved
/// `L, R, …` after the §2.6 M/S undo, `1` emits the first decoded
/// channel (for a mono stream the encoder's two body channels carry
/// the same signal; oracle-validated by the r419 mono corpus stream).
#[derive(Debug, Clone)]
pub struct Sv8StreamDecoder {
    /// `SH` highest-coded-subband field (debiased).
    max_band: u8,
    /// `SH` stream-wide mid/side flag.
    stream_ms: bool,
    /// `SH` output channel count (1 or 2).
    channels: u8,
    /// Persistent two-channel synthesis state.
    synthesis: MultiChannelSynthesis,
    /// Free-running CNS PRNG.
    cns: CnsPrng,
    /// Cross-frame SCF memory + Max_used_Band reference.
    state: Sv8FrameState,
    /// Total frames decoded.
    frames_decoded: u64,
}

impl Sv8StreamDecoder {
    /// Build a stream decoder from parsed `SH` fields.
    ///
    /// # Errors
    ///
    /// [`Error::ChannelCountInvalid`] for a channel count other than 1
    /// or 2 (multi-channel SV8 needs a body layout this crate has no
    /// fixture evidence for).
    pub fn from_header(fields: &StreamHeaderFields) -> Result<Self> {
        if !(1..=2).contains(&fields.channels) {
            return Err(Error::ChannelCountInvalid(fields.channels));
        }
        Ok(Self {
            max_band: fields.max_band,
            stream_ms: fields.mid_side,
            channels: fields.channels,
            synthesis: MultiChannelSynthesis::new(2)?,
            cns: CnsPrng::new(),
            state: Sv8FrameState::new(),
            frames_decoded: 0,
        })
    }

    /// Total frames decoded so far.
    #[must_use]
    pub fn frames_decoded(&self) -> u64 {
        self.frames_decoded
    }

    /// The output channel count (1 or 2, from the `SH` header).
    #[must_use]
    pub fn output_channels(&self) -> u8 {
        self.channels
    }

    /// Decode one `AP` packet payload holding `frames` frames into
    /// interleaved (or mono) s16-domain PCM.
    ///
    /// `frames` is `min(frames_per_audio_packet, frames remaining in
    /// the stream)` — the packet layer knows the stream totals
    /// ([`crate::sv8_decode::decode_sv8_stream`] computes this). Frame
    /// 0 of the packet is the key frame; the packet-boundary state
    /// reset happens here.
    ///
    /// # Errors
    ///
    /// Every error of [`decode_sv8_stereo_frame`] /
    /// [`reconstruct_sv8_stereo_frame`] plus the §2.6 M/S undo and
    /// synthesis bounds (unreachable for a well-formed stream).
    pub fn decode_audio_packet(&mut self, payload: &[u8], frames: u64) -> Result<Vec<f64>> {
        // The canonical decoder always peeks a 16-bit window; the last
        // codeword of an exactly-sized payload needs slack bytes to
        // peek into (a valid prefix code never *consumes* past its
        // codeword, so the pad bits are never decoded as data).
        let mut framed = payload.to_vec();
        framed.extend_from_slice(&[0, 0]);
        let mut reader = Sv7BitReader::new(&framed);

        // §3.3: every AP opens with a key frame; the SCF memory and
        // Max_used_Band reference restart.
        self.state.reset();

        let nch = usize::from(self.channels);
        let mut pcm = Vec::with_capacity(frames as usize * SAMPLES_PER_FRAME_PER_CHANNEL * nch);
        for f in 0..frames {
            let keyframe = f == 0;
            let frame = decode_sv8_stereo_frame(
                &mut reader,
                self.max_band,
                keyframe,
                self.stream_ms,
                &mut self.state,
                &mut self.cns,
            )?;
            let mut stereo = reconstruct_sv8_stereo_frame(&frame)?;
            undo_ms_stereo_pinned(&mut stereo, &frame.ms_flags)?;
            let left = self.synthesis.synthesize_channel_frame(0, &stereo[0])?;
            let right = self.synthesis.synthesize_channel_frame(1, &stereo[1])?;
            if nch == 2 {
                for (&l, &r) in left.iter().zip(right.iter()) {
                    pcm.push(l);
                    pcm.push(r);
                }
            } else {
                pcm.extend_from_slice(&left);
            }
            self.frames_decoded += 1;
        }
        Ok(pcm)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sv8_huffman::{Sv8CanonicalTable, SV8_RES_1_TABLE};

    /// MSB-first left-justified bit packer (mirrors the SV8 sub-walk
    /// tests).
    struct BitPacker {
        bytes: Vec<u8>,
        acc: u32,
        nbits: u8,
    }

    impl BitPacker {
        fn new() -> Self {
            BitPacker {
                bytes: Vec::new(),
                acc: 0,
                nbits: 0,
            }
        }
        fn push(&mut self, pattern: u16, length: u8) {
            for i in 0..length {
                let bit = (pattern >> (15 - i)) & 1;
                self.acc = (self.acc << 1) | bit as u32;
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
            self.bytes.push(0);
            self.bytes.push(0);
            self.bytes
        }
    }

    /// Find a `(pattern, length)` codeword of `table` decoding to `target`.
    fn codeword_for_symbol(table: &Sv8CanonicalTable, target: i8) -> Option<(u16, u8)> {
        let mut upper: u32 = 0x1_0000;
        for e in table.lengths.iter() {
            if e.length == 0 {
                continue;
            }
            let step = 1u32 << (16 - e.length as u32);
            let mut pat = e.code as u32;
            while pat < upper {
                let mut p = BitPacker::new();
                p.push(pat as u16, e.length);
                let bytes = p.finish();
                let mut r = Sv7BitReader::new(&bytes);
                if table.decode(&mut r).unwrap() == target {
                    return Some((pat as u16, e.length));
                }
                pat += step;
            }
            upper = e.code as u32;
        }
        None
    }

    /// Pack `n` empty bands (each res-1 sym 0 ⇒ band_type 0): a silent
    /// frame body that reads only the resolution sweep.
    fn silent_frame_bits(p: &mut BitPacker, nbands: u8) {
        let (code, len) = codeword_for_symbol(&SV8_RES_1_TABLE, 0).expect("res-1 sym 0");
        for _ in 0..nbands {
            p.push(code, len);
        }
    }

    #[test]
    fn new_starts_at_zero_frames() {
        let dec = Sv8MonoStreamDecoder::new(0);
        assert_eq!(dec.frames_decoded(), 0);
    }

    #[test]
    fn single_silent_frame_yields_silent_pcm() {
        let mut p = BitPacker::new();
        silent_frame_bits(&mut p, 3);
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let mut dec = Sv8MonoStreamDecoder::new(0);
        let pcm = dec
            .decode_frame(
                &mut r,
                Sv8FrameParams {
                    nbands: 3,
                    new_block: true,
                },
            )
            .unwrap();
        assert_eq!(pcm.len(), SAMPLES_PER_FRAME_PER_CHANNEL);
        assert!(pcm.iter().all(|&s| s == 0.0));
        assert_eq!(dec.frames_decoded(), 1);
    }

    #[test]
    fn multi_silent_frame_overlap_stays_silent() {
        // Three silent frames back-to-back through one persistent filter.
        let mut p = BitPacker::new();
        for _ in 0..3 {
            silent_frame_bits(&mut p, 2);
        }
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let mut dec = Sv8MonoStreamDecoder::new(0);
        let schedule = [Sv8FrameParams {
            nbands: 2,
            new_block: true,
        }; 3];
        let pcm = dec.decode_frames(&mut r, &schedule).unwrap();
        assert_eq!(pcm.len(), 3 * SAMPLES_PER_FRAME_PER_CHANNEL);
        assert!(pcm.iter().all(|&s| s == 0.0));
        assert_eq!(dec.frames_decoded(), 3);
    }

    #[test]
    fn decode_frames_empty_schedule_yields_no_pcm() {
        let mut r = Sv7BitReader::new(&[0xFF; 4]);
        let mut dec = Sv8MonoStreamDecoder::new(0);
        let pcm = dec.decode_frames(&mut r, &[]).unwrap();
        assert!(pcm.is_empty());
        assert_eq!(dec.frames_decoded(), 0);
    }

    #[test]
    fn reset_clears_frame_count() {
        let mut p = BitPacker::new();
        silent_frame_bits(&mut p, 1);
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let mut dec = Sv8MonoStreamDecoder::new(0);
        dec.decode_frame(
            &mut r,
            Sv8FrameParams {
                nbands: 1,
                new_block: true,
            },
        )
        .unwrap();
        assert_eq!(dec.frames_decoded(), 1);
        dec.reset();
        assert_eq!(dec.frames_decoded(), 0);
    }

    #[test]
    fn cns_threads_across_frames_via_persistent_prng() {
        // Two CNS frames (band_type -1) in a row: the second frame's PRNG
        // output must differ from the first (the shared PRNG advanced),
        // proving the state threads. res-1 raw 16 ⇒ band_type -1.
        let (code, len) = codeword_for_symbol(&SV8_RES_1_TABLE, 16).expect("res-1 sym 16");
        let mut p = BitPacker::new();
        // Two frames, each a single CNS band (nbands = 1).
        p.push(code, len);
        p.push(code, len);
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let mut dec = Sv8MonoStreamDecoder::new(0);
        let params = Sv8FrameParams {
            nbands: 1,
            new_block: true,
        };
        let f0 = dec.decode_frame(&mut r, params).unwrap();
        let f1 = dec.decode_frame(&mut r, params).unwrap();
        // The two frames carry independent PRNG draws ⇒ their PCM differs
        // (the filterbank is deterministic, so identical subband input
        // would give identical PCM — different input proves threading).
        assert_ne!(f0, f1);
        assert_eq!(dec.frames_decoded(), 2);
    }
}
