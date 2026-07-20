//! SV8 **two-channel** audio-packet frame-body decode — the real-stream
//! frame layout, fixture-pinned (round 419).
//!
//! Earlier rounds grounded every SV8 *sub-walk* (§6.2 band resolutions,
//! §6.3 SCFI/DSCF, §6.4 sample arms) but could not pin how a real frame
//! body composes them across bands and channels. The r419 SV8 corpus —
//! the staged SV7 fixtures losslessly transcoded to SV8 (spec §3.6:
//! identical quantised payload) plus fresh reference-encoder streams,
//! both black-box producers — pins the whole composition: every decoded
//! quantity below reproduces the SV7 §5 ground truth for all 92 corpus
//! frames, and the alternatives desynchronise within one band.
//!
//! # The pinned frame-body layout
//!
//! One SV8 frame inside an `AP` packet is, in order:
//!
//! 1. **`Max_used_Band`** — on a **key frame** (the first frame of an
//!    `AP` packet) a §6.5 bounded log code over the count range
//!    `0..=max_band+1`
//!    ([`crate::sv8_band_header::decode_keyframe_max_used_band`]); on a
//!    non-key frame the `Bands`-table delta off the previous frame's
//!    value ([`crate::sv8_band_header::decode_nonkey_max_used_band`]).
//! 2. **Band-resolution header** — §6.2, decoded top-down with the two
//!    channels interleaved per band
//!    ([`crate::sv8_band_header::decode_band_resolutions_stereo_grounded`]).
//! 3. **M/S bitmap** — only when the stream-wide M/S flag is set: the
//!    §6.2 count + enumerative selection over the bands with at least
//!    one non-zero channel
//!    ([`crate::sv8_band_header::decode_sv8_ms_flags`]), applied
//!    mask-MSB-to-**lowest**-band (fixture-pinned orientation).
//! 4. **SCFI pass** — ascending bands: for each band with at least one
//!    non-zero channel, one §6.3 SCFI decode
//!    ([`crate::sv8_scf_header::decode_sv8_scfi`]) whose context is the
//!    non-zero-channel count and whose packed value splits into the
//!    per-channel selectors. CNS bands (`Res == −1`) **participate**
//!    (fixture-pinned by the transcoded CNS stream — the SV7 §5.2
//!    "`Res ≠ 0`" gate carries over to SV8).
//! 5. **DSCF pass** — ascending bands, left channel then right per
//!    band: each non-zero band/channel reconstructs its three granule
//!    SCF indices ([`crate::sv8_dscf_loop::decode_sv8_band_scf`]). The
//!    "new-block" flag is **key frame or first use**: set on every
//!    band of a key frame, and on any band/channel coded for the first
//!    time since the key frame (fixture-pinned — a band re-entering
//!    the coded set mid-packet reads the absolute raw-7-bit base, not
//!    a delta off stale state). Otherwise `SCF[0]` deltas off the
//!    **same band's same-channel previous-frame `SCF[2]`** — the
//!    temporal predictor of SV7 erratum E1, carried by
//!    [`Sv8FrameState`].
//! 6. **Sample pass** — ascending bands, left then right: each
//!    non-zero band/channel decodes its 36 levels
//!    ([`crate::sv8_band_decode::decode_sv8_band_grounded`]); the CNS
//!    PRNG threads across bands/channels/frames in decode order.
//!
//! Passes 4–6 are **separate band-major sweeps** (like the SV7 §1.1(b)
//! frame layout), not one fused per-band loop — fixture-pinned: the
//! fused alternatives desynchronise at the first band whose SCFI
//! schedule differs between channels.
//!
//! # Two-channel bodies are the only real-stream shape
//!
//! Fixture-pinned (r419): an SV8 stream whose `SH` header declares
//! **one** channel still codes **two channels** in every frame body
//! (the reference-encoder mono stream decodes bit-exactly under this
//! two-channel layout, and starves under a single-channel reading).
//! The `SH` channel count selects the *output* channels, not the body
//! shape; [`crate::sv8_stream`] owns that mapping.
//!
//! Source-of-record: `docs/audio/musepack/spec/musepack-headers-and-coding.md`
//! §6.2/§6.3/§6.4 (the grounded sub-walks) + §2 (`SH` fields);
//! `docs/audio/musepack/musepack-sv7-sv8-spec.md` §2.3/§2.6/§3.4/§3.6
//! (frame-body pass shape, lossless SV7↔SV8 payload identity). The
//! cross-band/channel composition itself is pinned by black-box fixture
//! behaviour (the r419 corpus under `tests/fixtures/sv8/`), the same
//! method as the r390 SV7 wire pinning — no decoder source consulted.

use crate::cns::CnsPrng;
use crate::frame_reconstruct::zero_subband_matrix;
use crate::huffman::Sv7BitReader;
use crate::ms_stereo::StereoSubbandMatrix;
use crate::reconstruct::reconstruct_sv8_band_absolute;
use crate::scf::SCF_GRANULES_PER_BAND;
use crate::sv7_band_decode::SAMPLES_PER_BAND;
use crate::sv7_band_header::SV7_SUBBAND_COUNT;
use crate::sv8_band_decode::decode_sv8_band_grounded;
use crate::sv8_band_header::{
    decode_band_resolutions_stereo_grounded, decode_keyframe_max_used_band,
    decode_nonkey_max_used_band, decode_sv8_ms_flags,
};
use crate::sv8_dscf_loop::decode_sv8_band_scf;
use crate::sv8_scf_header::decode_sv8_scfi;
use crate::{Error, Result};

/// Number of channels an SV8 frame body always codes (fixture-pinned —
/// see the module docs).
pub const SV8_BODY_CHANNELS: usize = 2;

/// Cross-frame decode state one SV8 audio-packet run carries: the
/// per-band per-channel temporal SCF memory (the §6.3 / E1 predictor)
/// with its "first use" tracking, and the previous frame's
/// `Max_used_Band` (the §6.2 non-key delta reference).
///
/// [`Sv8FrameState::reset`] returns the state to the key-frame posture
/// (every band "first use", no previous band count) — call it at every
/// `AP` packet boundary: fixture-pinned (r419), each `AP` opens with a
/// key frame whose scalefactors are coded absolutely, which is what
/// makes `AP` packets independently decodable (spec §3.3 keyframes).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Sv8FrameState {
    /// Per-channel per-band `SCF[2]` of the last frame that coded the
    /// band; `None` = not coded since the last reset (⇒ "first use").
    scf_mem: [[Option<i32>; SV7_SUBBAND_COUNT]; SV8_BODY_CHANNELS],
    /// Previous frame's `Max_used_Band` (§6.2 non-key reference).
    last_nbands: u8,
}

impl Sv8FrameState {
    /// Fresh state in the key-frame posture.
    #[must_use]
    pub fn new() -> Self {
        Self {
            scf_mem: [[None; SV7_SUBBAND_COUNT]; SV8_BODY_CHANNELS],
            last_nbands: 0,
        }
    }

    /// Return to the key-frame posture (every band "first use").
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// The previous frame's `Max_used_Band`.
    #[must_use]
    pub fn last_nbands(&self) -> u8 {
        self.last_nbands
    }

    /// The stored temporal reference for `(ch, band)`: the `SCF[2]` of
    /// the last frame that coded the band, or `None` since the last
    /// reset ("first use" pending). Shared with the encode side
    /// ([`crate::sv8_stereo_frame_encode`]), which must track the same
    /// state to choose the absolute-vs-delta `SCF[0]` form.
    pub(crate) fn scf_ref(&self, ch: usize, band: usize) -> Option<i32> {
        self.scf_mem[ch][band]
    }

    /// Record `(ch, band)`'s `SCF[2]` after its DSCF layer.
    pub(crate) fn note_scf2(&mut self, ch: usize, band: usize, scf2: i32) {
        self.scf_mem[ch][band] = Some(scf2);
    }

    /// Record the frame's `Max_used_Band` (the §6.2 non-key reference).
    pub(crate) fn note_nbands(&mut self, nbands: u8) {
        self.last_nbands = nbands;
    }
}

impl Default for Sv8FrameState {
    fn default() -> Self {
        Self::new()
    }
}

/// One decoded SV8 two-channel frame body: the structured output of
/// [`decode_sv8_stereo_frame`], everything indexed by ascending band
/// `0..nbands`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Sv8StereoFrameDecode {
    /// §6.2 `Max_used_Band` — the count of coded bands.
    pub nbands: u8,
    /// Per-band `[left, right]` signed band types (`-1..=17`).
    pub res: Vec<[i8; 2]>,
    /// Per-band M/S flag (all `false` when the stream M/S flag is off).
    pub ms_flags: Vec<bool>,
    /// Per-band per-channel §6.3 SCFI selectors (`0..=3`; zero for an
    /// empty channel). For a band with one non-zero channel both slots
    /// carry that channel's selector (the §6.3 packed value
    /// degenerates); recorded so a re-encode can reproduce the exact
    /// SCFI codewords.
    pub scfi: Vec<[u8; 2]>,
    /// Per-band per-channel three §6.3 granule SCF indices (zeroed for
    /// an empty channel).
    pub granule_scf: Vec<[[i32; SCF_GRANULES_PER_BAND]; 2]>,
    /// Per-band per-channel 36 decoded sample levels (zeroed for an
    /// empty channel; signed/centred for every arm).
    pub levels: Vec<[[i32; SAMPLES_PER_BAND]; 2]>,
}

/// Decode one SV8 two-channel frame body in the fixture-pinned layout
/// (see the module docs): `Max_used_Band`, the §6.2 stereo resolution
/// walk, the §6.2 M/S bitmap, then the three band-major passes (SCFI,
/// DSCF, samples).
///
/// `max_band` is the `SH` header's highest-coded-subband field
/// (debiased); `keyframe` is true for the first frame of an `AP` packet
/// (the caller must [`Sv8FrameState::reset`] `state` at the packet
/// boundary); `stream_ms` is the `SH` mid-side flag; `cns` is the
/// shared noise PRNG.
///
/// # Errors
///
/// - [`Error::UnexpectedEof`] if the reader starves in any pass.
/// - [`Error::HuffmanNoMatch`] if a canonical peek matches no row.
/// - [`Error::MaxBandOutOfRange`] from the §6.2 `Max_used_Band` reads.
/// - [`Error::UnsupportedBandType`] for a band type outside `-1..=17`.
/// - [`Error::InvalidScfCodingMethod`] from the §6.3 SCF layer.
pub fn decode_sv8_stereo_frame(
    reader: &mut Sv7BitReader<'_>,
    max_band: u8,
    keyframe: bool,
    stream_ms: bool,
    state: &mut Sv8FrameState,
    cns: &mut CnsPrng,
) -> Result<Sv8StereoFrameDecode> {
    // 1. §6.2 Max_used_Band.
    let nbands = if keyframe {
        decode_keyframe_max_used_band(reader, max_band)?
    } else {
        decode_nonkey_max_used_band(reader, state.last_nbands)?
    };
    state.last_nbands = nbands;
    let nb = nbands as usize;
    if nb > SV7_SUBBAND_COUNT {
        return Err(Error::MaxBandOutOfRange(nbands));
    }

    // 2. §6.2 stereo band-resolution walk (top-down, L/R interleaved).
    let res = decode_band_resolutions_stereo_grounded(reader, nbands)?;

    // 3. §6.2 M/S bitmap over the bands with a non-zero channel.
    let mut ms_flags = vec![false; nb];
    if stream_ms {
        let scope: Vec<usize> = (0..nb)
            .filter(|&b| res[b][0] != 0 || res[b][1] != 0)
            .collect();
        let flags = decode_sv8_ms_flags(reader, scope.len() as u8)?;
        // flags[0] = lowest band in scope (fixture-pinned orientation).
        for (&b, &f) in scope.iter().zip(flags.iter()) {
            ms_flags[b] = f;
        }
    }

    // 4. SCFI pass (ascending bands; CNS bands participate).
    let mut scfi = vec![[0u8; 2]; nb];
    for b in 0..nb {
        let chs: Vec<usize> = (0..SV8_BODY_CHANNELS)
            .filter(|&ch| res[b][ch] != 0)
            .collect();
        if chs.is_empty() {
            continue;
        }
        let sel = decode_sv8_scfi(reader, chs.len() as u8)?;
        let split = if chs.len() == 2 {
            [sel.left, sel.right]
        } else {
            [sel.left, sel.left]
        };
        for (k, &ch) in chs.iter().enumerate() {
            scfi[b][ch] = split[k];
        }
    }

    // 5. DSCF pass (ascending bands, left then right; temporal memory +
    //    key-frame-or-first-use absolute base).
    let mut granule_scf = vec![[[0i32; SCF_GRANULES_PER_BAND]; 2]; nb];
    for b in 0..nb {
        for ch in 0..SV8_BODY_CHANNELS {
            if res[b][ch] == 0 {
                continue;
            }
            let new_block = keyframe || state.scf_mem[ch][b].is_none();
            let scf = decode_sv8_band_scf(
                reader,
                scfi[b][ch],
                new_block,
                state.scf_mem[ch][b].unwrap_or(0),
            )?;
            state.scf_mem[ch][b] = Some(scf[SCF_GRANULES_PER_BAND - 1]);
            granule_scf[b][ch] = scf;
        }
    }

    // 6. Sample pass (ascending bands, left then right).
    let mut levels = vec![[[0i32; SAMPLES_PER_BAND]; 2]; nb];
    for b in 0..nb {
        for ch in 0..SV8_BODY_CHANNELS {
            if res[b][ch] != 0 {
                decode_sv8_band_grounded(reader, res[b][ch], cns, &mut levels[b][ch])?;
            }
        }
    }

    Ok(Sv8StereoFrameDecode {
        nbands,
        res,
        ms_flags,
        scfi,
        granule_scf,
        levels,
    })
}

/// Reconstruct a decoded SV8 two-channel frame body into the pair of
/// absolute s16-domain subband matrices the §2.6 M/S undo + synthesis
/// steps consume.
///
/// Per band/channel the absolute law is the SV7-shared
/// [`reconstruct_sv8_band_absolute`] (`level × C[Res+1] ×
/// SCF_STEP_RATIO^(scf−1)`); empty channels and bands past `nbands`
/// stay silent.
///
/// # Errors
///
/// [`Error::UnsupportedBandType`] for a band type outside `-1..=17`
/// (unreachable for output of [`decode_sv8_stereo_frame`]).
pub fn reconstruct_sv8_stereo_frame(frame: &Sv8StereoFrameDecode) -> Result<StereoSubbandMatrix> {
    let mut out = [zero_subband_matrix(), zero_subband_matrix()];
    for (b, res) in frame.res.iter().enumerate().take(frame.nbands as usize) {
        for (ch, matrix) in out.iter_mut().enumerate() {
            if res[ch] != 0 {
                reconstruct_sv8_band_absolute(
                    res[ch],
                    &frame.levels[b][ch],
                    frame.granule_scf[b][ch],
                    &mut matrix[b],
                )?;
            }
        }
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sv8_huffman::{table_for_role, Sv8CanonicalTable, Sv8TableRole, SV8_RES_1_TABLE};

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
            self.bytes.push(0);
            self.bytes.push(0);
            self.bytes
        }
    }

    /// Find a `(pattern, length)` codeword of `table` decoding to
    /// `target`.
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

    /// Reference phased-binary log-code encoder (§6.5) for value `v` in
    /// `0..max`, MSB-first `(pattern, length)`.
    fn log_encode(v: u32, max: u32) -> (u16, u8) {
        if max <= 1 {
            return (0, 0);
        }
        let mut bitlen: u8 = 0;
        while (1u32 << bitlen) < max {
            bitlen += 1;
        }
        let lost = (1u32 << bitlen) - max;
        if v < lost {
            let len = bitlen - 1;
            ((v as u16) << (16 - len), len)
        } else {
            let code = v + lost;
            ((code as u16) << (16 - bitlen), bitlen)
        }
    }

    /// Push a keyframe `Max_used_Band` codeword for count `v`.
    fn push_keyframe_mub(p: &mut BitPacker, v: u32, max_band: u8) {
        let (pat, len) = log_encode(v, max_band as u32 + 2);
        if len > 0 {
            p.push(pat, len);
        }
    }

    #[test]
    fn state_new_is_keyframe_posture() {
        let st = Sv8FrameState::new();
        assert_eq!(st.last_nbands(), 0);
        assert_eq!(st, Sv8FrameState::default());
    }

    #[test]
    fn zero_band_keyframe_decodes_to_empty_frame() {
        let max_band = 4;
        let mut p = BitPacker::new();
        push_keyframe_mub(&mut p, 0, max_band);
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let mut st = Sv8FrameState::new();
        let mut cns = CnsPrng::new();
        let f = decode_sv8_stereo_frame(&mut r, max_band, true, true, &mut st, &mut cns).unwrap();
        assert_eq!(f.nbands, 0);
        assert!(f.res.is_empty());
        assert!(f.ms_flags.is_empty());
        // Reconstruction of an empty frame is silence.
        let m = reconstruct_sv8_stereo_frame(&f).unwrap();
        assert_eq!(m[0], zero_subband_matrix());
        assert_eq!(m[1], zero_subband_matrix());
    }

    #[test]
    fn all_empty_bands_read_res_only_and_stay_silent() {
        // nbands = 3, every band [0, 0]: six res-1 sym-0 codewords
        // (top-down, L/R interleaved), then an M/S bitmap over zero
        // non-zero bands (reads nothing), no SCF/sample bits.
        let max_band = 4;
        let (r0, l0) = codeword_for_symbol(&SV8_RES_1_TABLE, 0).expect("res-1 sym 0");
        let mut p = BitPacker::new();
        push_keyframe_mub(&mut p, 3, max_band);
        for _ in 0..3 {
            p.push(r0, l0); // left
            p.push(r0, l0); // right
        }
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let mut st = Sv8FrameState::new();
        let mut cns = CnsPrng::new();
        let f = decode_sv8_stereo_frame(&mut r, max_band, true, true, &mut st, &mut cns).unwrap();
        assert_eq!(f.nbands, 3);
        assert_eq!(f.res, vec![[0, 0]; 3]);
        assert_eq!(f.ms_flags, vec![false; 3]);
        assert!(f
            .levels
            .iter()
            .all(|b| b.iter().all(|ch| ch.iter().all(|&s| s == 0))));
    }

    #[test]
    fn coded_band_reads_scfi_dscf_and_samples_in_band_major_passes() {
        // One band, both channels band_type 3 (grouped-2 nibble pairs):
        // res: top band L raw 3, R raw 3 (ctx 0 both).
        // M/S: scope 1 band → log cnt over 0..2 → cnt 0 (no ms).
        // SCFI: both non-zero → scfi-2 packed; pick value (3,3) = 15.
        // DSCF: key frame → raw7 absolute per channel.
        // samples: 18 q3 codewords per channel.
        let max_band = 2;
        let (r3, l3) = codeword_for_symbol(&SV8_RES_1_TABLE, 3).expect("res-1 sym 3");
        let scfi2 = table_for_role(Sv8TableRole::Scfi, 1).unwrap();
        let (s15, sl15) = codeword_for_symbol(scfi2, 15).expect("scfi-2 sym 15");
        let q3 = table_for_role(Sv8TableRole::Q3, 0).unwrap();
        let q3e = q3.lengths[0];

        let mut p = BitPacker::new();
        push_keyframe_mub(&mut p, 1, max_band);
        p.push(r3, l3); // L res
        p.push(r3, l3); // R res
                        // M/S bitmap: tot = 1 → log code over 0..2 → 1 bit; cnt 0.
        p.push_raw(0, 1);
        // SCFI pass: one scfi-2 codeword for (3,3).
        p.push(s15, sl15);
        // DSCF pass: L raw7 = 40, R raw7 = 50 (key frame absolutes).
        p.push_raw(40, 7);
        p.push_raw(50, 7);
        // Sample pass: 18 grouped-2 codewords per channel.
        for _ in 0..36 {
            p.push(q3e.code, q3e.length);
        }
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let mut st = Sv8FrameState::new();
        let mut cns = CnsPrng::new();
        let f = decode_sv8_stereo_frame(&mut r, max_band, true, true, &mut st, &mut cns).unwrap();
        assert_eq!(f.nbands, 1);
        assert_eq!(f.res, vec![[3, 3]]);
        assert_eq!(f.ms_flags, vec![false]);
        // scfi 3 → all three granules share SCF[0] = raw7 − 6.
        assert_eq!(f.granule_scf[0][0], [34, 34, 34]);
        assert_eq!(f.granule_scf[0][1], [44, 44, 44]);
        // The DSCF memory now carries both channels' SCF[2].
        assert_eq!(st.scf_mem[0][0], Some(34));
        assert_eq!(st.scf_mem[1][0], Some(44));
        assert_eq!(st.last_nbands(), 1);
    }

    #[test]
    fn first_use_band_reads_absolute_base_on_non_key_frame() {
        // Two frames: frame 0 (key) has band 0 empty in the right
        // channel; frame 1 (non-key) codes the right channel for the
        // first time → its SCF[0] must be the raw7 absolute (first
        // use), while the left channel deltas off its frame-0 memory.
        let max_band = 1;
        let (r3, l3) = codeword_for_symbol(&SV8_RES_1_TABLE, 3).expect("res-1 sym 3");
        let (r0, l0) = codeword_for_symbol(&SV8_RES_1_TABLE, 0).expect("res-1 sym 0");
        let scfi1 = table_for_role(Sv8TableRole::Scfi, 0).unwrap();
        let (s3, sl3) = codeword_for_symbol(scfi1, 3).expect("scfi-1 sym 3");
        let scfi2 = table_for_role(Sv8TableRole::Scfi, 1).unwrap();
        let (s15, sl15) = codeword_for_symbol(scfi2, 15).expect("scfi-2 sym 15");
        let q3 = table_for_role(Sv8TableRole::Q3, 0).unwrap();
        let q3e = q3.lengths[0];
        let dscf2 = table_for_role(Sv8TableRole::Dscf, 1).unwrap();
        let (d30, dl30) = codeword_for_symbol(dscf2, 30).expect("dscf-2 sym 30");

        let mut p = BitPacker::new();
        // ── frame 0 (key): nbands 1, res [3, 0].
        push_keyframe_mub(&mut p, 1, max_band);
        p.push(r3, l3); // L res 3
        p.push(r0, l0); // R res 0
        p.push_raw(0, 1); // M/S: tot 1 → 1-bit log cnt = 0
        p.push(s3, sl3); // SCFI: single non-zero channel → scfi-1, sel 3
        p.push_raw(40, 7); // DSCF: key → raw7 40 → SCF 34
        for _ in 0..18 {
            p.push(q3e.code, q3e.length); // L samples
        }
        // ── frame 1 (non-key): Bands delta 0 keeps nbands 1.
        let bands = table_for_role(Sv8TableRole::Bands, 0).unwrap();
        let (b0, bl0) = codeword_for_symbol(bands, 0).expect("bands sym 0");
        p.push(b0, bl0);
        // res: L delta 0 (stays 3, ctx from above=3 ⇒ res-2), R raw…
        // Top-down decode: this is the top band again (nb = 1): both
        // channels read ctx 0 raw values. L raw 3, R raw 3.
        p.push(r3, l3);
        p.push(r3, l3);
        p.push_raw(0, 1); // M/S: tot 1 → cnt 0
        p.push(s15, sl15); // SCFI: both non-zero → scfi-2 (3,3)
                           // DSCF L: non-key + known memory (34) → dscf-2 delta 30:
                           // ((34 − 25 + 30) & 127) − 6 = 33.
        p.push(d30, dl30);
        // DSCF R: first use → raw7 absolute 50 → 44.
        p.push_raw(50, 7);
        for _ in 0..36 {
            p.push(q3e.code, q3e.length); // L + R samples
        }
        let bytes = p.finish();

        let mut r = Sv7BitReader::new(&bytes);
        let mut st = Sv8FrameState::new();
        let mut cns = CnsPrng::new();
        let f0 = decode_sv8_stereo_frame(&mut r, max_band, true, true, &mut st, &mut cns).unwrap();
        assert_eq!(f0.res, vec![[3, 0]]);
        assert_eq!(f0.granule_scf[0][0], [34, 34, 34]);
        assert_eq!(st.scf_mem[1][0], None, "right channel never coded yet");

        let f1 = decode_sv8_stereo_frame(&mut r, max_band, false, true, &mut st, &mut cns).unwrap();
        assert_eq!(f1.res, vec![[3, 3]]);
        assert_eq!(
            f1.granule_scf[0][0],
            [33, 33, 33],
            "left: temporal delta off frame-0 SCF[2]"
        );
        assert_eq!(
            f1.granule_scf[0][1],
            [44, 44, 44],
            "right: first-use absolute raw7"
        );
    }

    #[test]
    fn ms_bitmap_applies_mask_msb_to_lowest_band() {
        // Two non-zero bands; a bitmap flagging only the lowest one.
        // tot = 2 → log cnt over 0..3: bitlen 2, lost 1 → cnt 1 is
        // code ≥ lost: 2 bits '10'. Then the enumerative subset picks
        // 1 of 2 positions: C(2,1) = 2 → 1 bit; mask bit 1 (MSB) is
        // the lowest band → enumerative code 1 ('1').
        let max_band = 2;
        let (r3, l3) = codeword_for_symbol(&SV8_RES_1_TABLE, 3).expect("res-1 sym 3");
        let res2 = table_for_role(Sv8TableRole::Res, 1).unwrap();
        let (rd0, rdl0) = codeword_for_symbol(res2, 0).expect("res-2 sym 0");
        let scfi2 = table_for_role(Sv8TableRole::Scfi, 1).unwrap();
        let (s15, sl15) = codeword_for_symbol(scfi2, 15).expect("scfi-2 sym 15");
        let q3 = table_for_role(Sv8TableRole::Q3, 0).unwrap();
        let q3e = q3.lengths[0];

        let mut p = BitPacker::new();
        push_keyframe_mub(&mut p, 2, max_band);
        // Res top-down: band 1 (top) L raw 3, R raw 3; band 0 deltas 0
        // off above (ctx 1: above = 3 > 2 ⇒ res-2 table).
        p.push(r3, l3);
        p.push(r3, l3);
        p.push(rd0, rdl0);
        p.push(rd0, rdl0);
        // M/S: cnt = 1 (log '10'), enumerative 1 bit = 1 → mask MSB →
        // lowest band flagged.
        p.push_raw(2, 2);
        p.push_raw(1, 1);
        // SCFI + DSCF + samples for two both-channel bands.
        for _ in 0..2 {
            p.push(s15, sl15);
        }
        for _ in 0..4 {
            p.push_raw(40, 7);
        }
        for _ in 0..72 {
            p.push(q3e.code, q3e.length);
        }
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let mut st = Sv8FrameState::new();
        let mut cns = CnsPrng::new();
        let f = decode_sv8_stereo_frame(&mut r, max_band, true, true, &mut st, &mut cns).unwrap();
        assert_eq!(f.res, vec![[3, 3], [3, 3]]);
        assert_eq!(f.ms_flags, vec![true, false], "mask MSB is band 0");
    }

    #[test]
    fn stream_ms_off_reads_no_bitmap() {
        // Same single-band frame as the coded-band test but stream_ms
        // off: no M/S bits anywhere.
        let max_band = 2;
        let (r3, l3) = codeword_for_symbol(&SV8_RES_1_TABLE, 3).expect("res-1 sym 3");
        let scfi2 = table_for_role(Sv8TableRole::Scfi, 1).unwrap();
        let (s15, sl15) = codeword_for_symbol(scfi2, 15).expect("scfi-2 sym 15");
        let q3 = table_for_role(Sv8TableRole::Q3, 0).unwrap();
        let q3e = q3.lengths[0];

        let mut p = BitPacker::new();
        push_keyframe_mub(&mut p, 1, max_band);
        p.push(r3, l3);
        p.push(r3, l3);
        p.push(s15, sl15);
        p.push_raw(40, 7);
        p.push_raw(50, 7);
        for _ in 0..36 {
            p.push(q3e.code, q3e.length);
        }
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let mut st = Sv8FrameState::new();
        let mut cns = CnsPrng::new();
        let f = decode_sv8_stereo_frame(&mut r, max_band, true, false, &mut st, &mut cns).unwrap();
        assert_eq!(f.nbands, 1);
        assert_eq!(f.ms_flags, vec![false]);
        assert_eq!(f.granule_scf[0][0], [34, 34, 34]);
        assert_eq!(f.granule_scf[0][1], [44, 44, 44]);
    }

    #[test]
    fn reconstruction_applies_absolute_law_per_channel() {
        use crate::requant::{DEQUANT_COEFFICIENT_C, SCF_STEP_RATIO};

        // Hand-build a decoded frame: band 0, L band_type 3 levels 1,
        // scf [1,1,1] (unity gain); R empty.
        let mut f = Sv8StereoFrameDecode {
            nbands: 1,
            res: vec![[3, 0]],
            ms_flags: vec![false],
            scfi: vec![[3, 3]],
            granule_scf: vec![[[1, 1, 1], [0, 0, 0]]],
            levels: vec![[[1; SAMPLES_PER_BAND], [0; SAMPLES_PER_BAND]]],
        };
        let m = reconstruct_sv8_stereo_frame(&f).unwrap();
        let c = DEQUANT_COEFFICIENT_C[4]; // band_type 3
        assert!((m[0][0][0] - c).abs() < 1e-9, "unity gain at index 1");
        assert!(m[1][0].iter().all(|&s| s == 0.0), "empty right channel");

        // A different SCF index scales by the pinned ratio.
        f.granule_scf[0][0] = [2, 2, 2];
        let m2 = reconstruct_sv8_stereo_frame(&f).unwrap();
        assert!((m2[0][0][0] - c * SCF_STEP_RATIO).abs() < 1e-9);
    }

    #[test]
    fn rejects_res_walk_starvation() {
        let max_band = 4;
        let mut p = BitPacker::new();
        push_keyframe_mub(&mut p, 3, max_band);
        // No res codewords at all: the walk starves.
        let mut bytes = p.bytes.clone();
        if p.nbits > 0 {
            bytes.push((p.acc << (8 - p.nbits)) as u8);
        }
        let mut r = Sv7BitReader::new(&bytes);
        let mut st = Sv8FrameState::new();
        let mut cns = CnsPrng::new();
        assert!(matches!(
            decode_sv8_stereo_frame(&mut r, max_band, true, true, &mut st, &mut cns),
            Err(Error::UnexpectedEof)
        ));
    }
}
