//! SV7 stereo (two-channel) frame-body assembler + reconstruction —
//! the corpus-pinned pass order.
//!
//! Decodes one SV7 frame body for both channels and hands back the
//! reconstructed per-channel subband matrices
//! ([`crate::ms_stereo::StereoSubbandMatrix`]) plus the per-band M/S
//! flags — the input the §2.6 M/S-undo step
//! ([`crate::ms_stereo::undo_ms_stereo_pinned`]) and the synthesis
//! filterbank consume.
//!
//! # Phase ordering — four band-major passes
//!
//! The staged `spec/musepack-headers-and-coding.md` §1.1(b) (framing
//! corrections, docs commit `0f1b6a2`) documents the frame body as
//! **four sequential band-major passes, channel-minor within each
//! band** — the layout this crate pinned in round 390 with the SV7
//! fixture corpus (`tests/fixtures/sv7/`, independent mppenc 1.16
//! streams whose per-frame 20-bit bit-length prefixes give an exact
//! per-frame bit budget; every alternative — whole-channel sweeps,
//! combined SCFI+DSCF pass, channel-major passes — diverges on most
//! frames):
//!
//! 1. **§5.1 `Res` header** — both channels interleaved per band plus
//!    the per-band M/S bit
//!    ([`crate::sv7_band_header::decode_res_header_grounded`]);
//! 2. **SCFI pass** — for each band `0..=max_band`, for each channel
//!    with `Res ≠ 0`: one SCFI selector VLC
//!    ([`crate::sv7_scf_decode::decode_sv7_scfi`]);
//! 3. **DSCF pass** — same iteration order: each band/channel's
//!    `1..=3` DSCF indices per its SCFI case
//!    ([`crate::sv7_scf_decode::decode_sv7_band_dscf`]);
//! 4. **samples pass** — same iteration order: each band's 36 sample
//!    levels per its `Res` arm (with the 1-bit context selector for the
//!    grouped / per-sample-Huffman arms), CNS bands filling from the
//!    shared PRNG.
//!
//! Under this layout every corpus frame — including all 20 frames of
//! the CNS-bearing `cns-pns` fixture — consumes **exactly** its
//! declared 20-bit prefix bit count.
//!
//! # The SCF[0] reference — per-band memory across frames
//!
//! The `SCF[0]` delta reference is the **same subband's `SCF[2]` from
//! the previous frame**, held per channel ([`Sv7ScfMemory`],
//! zero-initialised at stream start), not the previous band of the
//! same frame — §5.3 as corrected by erratum **E1**
//! (`docs/audio/musepack/musepack-errata.md`), which this crate's r390
//! corpus work surfaced: with per-band temporal memory the decoded PCM
//! matches the FFmpeg oracle to ±1 LSB on every corpus stream, while
//! the within-frame chain produces correlation ≈ 0.2.
//!
//! # CNS bands carry the SCF layer — wire-proven
//!
//! §5.2 reads SCFI "for each channel whose `Res ≠ 0`" — which includes
//! the CNS band (`Res == -1`) — and the structural spec says the noise
//! band is "scaled by the band's scalefactor"
//! (`musepack-sv7-sv8-spec.md` §2.5 notes). This module therefore reads
//! SCFI + DSCF for CNS bands exactly like coded bands. The `cns-pns`
//! fixture (`tests/sv7_cns_corpus.rs`; 215 CNS band-instances across
//! 18 frames) proves the convention on a real mppenc PNS stream: every
//! frame decodes exactly on its 20-bit bit budget, which is only
//! possible if CNS bands take part in both scalefactor passes and read
//! zero sample-pass bits. (The oracle's noise *waveform* is not
//! comparable — see the fixture suite docs.)
//!
//! # Reconstruction — the corpus-pinned absolute law
//!
//! Each non-empty band reconstructs via
//! [`crate::reconstruct::reconstruct_sv7_band_absolute`]:
//! `sample = level × C[Res + 1] × SCF_STEP_RATIO^(scf − 1)`, directly
//! in the signed-16-bit output domain (the previously-GAP absolute
//! anchor, resolved empirically — see [`crate::reconstruct`]).
//!
//! Source-of-record: `docs/audio/musepack/spec/musepack-headers-and-coding.md`
//! §1.1 (pass order) + §5.1–§5.5 (layer structure, VLC tables, arms)
//! with erratum E1 (`SCF[0]` reference), +
//! `docs/audio/musepack/musepack-sv7-sv8-spec.md` §2.5/§2.6; the
//! absolute gain anchor remains corpus-pinned
//! (`tests/fixtures/sv7/`).

use crate::cns::CnsPrng;
use crate::frame_reconstruct::zero_subband_matrix;
use crate::huffman::Sv7BitReader;
use crate::ms_stereo::StereoSubbandMatrix;
use crate::reconstruct::reconstruct_sv7_band_absolute;
use crate::sv7_band_decode::{band_type_uses_context_selector, decode_sv7_band, SAMPLES_PER_BAND};
use crate::sv7_band_header::{decode_res_header_grounded, SV7_SUBBAND_COUNT};
use crate::sv7_scf_decode::{decode_sv7_band_dscf, decode_sv7_scfi};
use crate::{Error, Result};

/// Per-channel, per-subband SCF memory threading across frames: entry
/// `[ch][b]` is subband `b`'s most recent `SCF[2]` for channel `ch` —
/// the corpus-pinned delta reference for that subband's next `SCF[0]`.
/// Zero-initialised at stream start (frame 0 deltas off 0, with the
/// §5.3 raw-6-bit escape available for absolute placement).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Sv7ScfMemory {
    state: [[i32; SV7_SUBBAND_COUNT]; 2],
}

impl Sv7ScfMemory {
    /// Fresh stream-start memory (all references 0).
    #[must_use]
    pub fn new() -> Self {
        Self {
            state: [[0; SV7_SUBBAND_COUNT]; 2],
        }
    }

    /// The current reference for channel `ch`, subband `b`.
    #[must_use]
    pub fn reference(&self, ch: usize, b: usize) -> i32 {
        self.state[ch][b]
    }

    /// Record subband `b`'s new `SCF[2]` for channel `ch`.
    pub fn update(&mut self, ch: usize, b: usize, scf2: i32) {
        self.state[ch][b] = scf2;
    }

    /// Reset to the stream-start state (e.g. after a seek).
    pub fn reset(&mut self) {
        self.state = [[0; SV7_SUBBAND_COUNT]; 2];
    }
}

impl Default for Sv7ScfMemory {
    fn default() -> Self {
        Self::new()
    }
}

/// The decoded structure of one SV7 stereo frame body, before the §2.6
/// M/S-undo step.
///
/// `channels[0]` is the left/mid channel's reconstructed subband matrix
/// and `channels[1]` the right/side channel's (which role each subband
/// plays is given by `ms_flags`). `ms_flags[b]` is the §5.1 per-band M/S
/// flag for subband `b`: `true` ⇒ subband `b` is coded mid/side and must
/// be run through [`crate::ms_stereo::undo_ms_stereo_pinned`]; `false` ⇒
/// it is already left/right. Sample values are in the signed-16-bit
/// domain (see [`crate::reconstruct::sv7_absolute_scf_gain`]).
#[derive(Debug, Clone, PartialEq)]
pub struct Sv7StereoFrame {
    /// The two channels' reconstructed subband matrices (pre-M/S-undo).
    pub channels: StereoSubbandMatrix,
    /// Per-band M/S flags, ascending band order, length `max_band + 1`.
    pub ms_flags: Vec<bool>,
}

/// Decode + reconstruct one SV7 **stereo** frame body into a
/// [`Sv7StereoFrame`], in the corpus-pinned four-pass order (see the
/// module docs).
///
/// `reader` is positioned at the start of the frame body (the first bit
/// of the §5.1 header — i.e. *after* the frame's 20-bit bit-length
/// prefix, which the whole-file layer consumes). `max_band` /
/// `stream_ms` come from the §1 fixed header. `scf` is the cross-frame
/// per-band SCF memory (one per stream, zero-initialised); `cns` the
/// shared CNS PRNG, advanced by every noise band in pass order.
///
/// # Errors
///
/// - [`Error::MaxBandOutOfRange`] if `max_band` exceeds the §1
///   Layer-II 32-subband inclusive bound.
/// - [`Error::UnexpectedEof`] / [`Error::HuffmanNoMatch`] if the reader
///   starves or a peek matches no table row in any pass.
/// - [`Error::UnsupportedBandType`] for a `Res` outside `-1..=17`.
/// - [`Error::InvalidScfCodingMethod`] propagated from the SCFI pass.
pub fn decode_sv7_stereo_frame(
    reader: &mut Sv7BitReader<'_>,
    max_band: u8,
    stream_ms: bool,
    scf: &mut Sv7ScfMemory,
    cns: &mut CnsPrng,
) -> Result<Sv7StereoFrame> {
    // Pass 1 — §5.1: shared band-type header, both channels + M/S bits.
    let header = decode_res_header_grounded(reader, max_band, 2, stream_ms)?;
    let n = header.len();
    let res: Vec<[i8; 2]> = header.iter().map(|b| b.res).collect();
    let ms_flags: Vec<bool> = header.iter().map(|b| b.ms_flag.unwrap_or(false)).collect();

    // Pass 2 — SCFI selectors, band-major / channel-minor, for every
    // non-zero band (coded *and* CNS).
    let mut scfi = vec![[0u8; 2]; n];
    for (b, r) in res.iter().enumerate() {
        for ch in 0..2 {
            if r[ch] != 0 {
                scfi[b][ch] = decode_sv7_scfi(reader)?;
            }
        }
    }

    // Pass 3 — DSCF chains, same order, referencing the per-band memory.
    let mut granule_scf = vec![[[0i32; 3]; 2]; n];
    for (b, r) in res.iter().enumerate() {
        for ch in 0..2 {
            if r[ch] != 0 {
                let band_scf = decode_sv7_band_dscf(reader, scfi[b][ch], scf.reference(ch, b))?;
                granule_scf[b][ch] = band_scf.indices;
                scf.update(ch, b, band_scf.last_index());
            }
        }
    }

    // Pass 4 — sample levels, same order.
    let mut levels = vec![[[0i32; SAMPLES_PER_BAND]; 2]; n];
    for (b, r) in res.iter().enumerate() {
        for ch in 0..2 {
            let band_type = r[ch];
            if band_type == 0 {
                continue;
            }
            let ctx = if band_type_uses_context_selector(band_type) {
                (reader.read_bits(1)? & 1) as usize
            } else {
                0
            };
            decode_sv7_band(reader, band_type, cns, ctx, &mut levels[b][ch])?;
        }
    }

    // Reconstruction — corpus-pinned absolute law per non-empty band.
    let mut channels = [zero_subband_matrix(), zero_subband_matrix()];
    for (b, r) in res.iter().enumerate() {
        if b >= SV7_SUBBAND_COUNT {
            return Err(Error::MaxBandOutOfRange(b as u8));
        }
        for ch in 0..2 {
            if r[ch] == 0 {
                continue;
            }
            reconstruct_sv7_band_absolute(
                r[ch],
                &levels[b][ch],
                granule_scf[b][ch],
                &mut channels[ch][b],
            )?;
        }
    }

    Ok(Sv7StereoFrame { channels, ms_flags })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame_reconstruct::zero_subband_matrix;
    use crate::huffman::{SV7_Q3_TABLE, SV7_SCFI_TABLE};
    use crate::requant::{DEQUANT_COEFFICIENT_C, SCF_STEP_RATIO};

    /// MSB-first bit packer mirroring the sibling frame-decode tests.
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
            // Trailing peek-padding (the reader always peeks 16 bits).
            for _ in 0..4 {
                self.bytes.push(0);
            }
            self.bytes
        }
    }

    /// SCFI codeword for value 3 (only SCF[0] coded).
    fn scfi3() -> (u16, u8) {
        (0x0000, 2)
    }
    /// DSCF codeword for symbol 0 (no-op delta).
    fn dscf0() -> (u16, u8) {
        (0x9000, 4)
    }
    /// DSCF codeword for symbol +1.
    fn dscf1() -> (u16, u8) {
        (0xa000, 3)
    }

    #[test]
    fn all_silent_stereo_frame_reconstructs_to_silence() {
        // max_band = 0: one band, both channels raw-4-bit Res = 0.
        let mut p = Packer::new();
        p.push_raw(0, 4); // left band-0 Res = 0
        p.push_raw(0, 4); // right band-0 Res = 0
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let mut cns = CnsPrng::new();
        let mut scf = Sv7ScfMemory::new();
        let frame = decode_sv7_stereo_frame(&mut r, 0, false, &mut scf, &mut cns).unwrap();
        assert_eq!(frame.channels[0], zero_subband_matrix());
        assert_eq!(frame.channels[1], zero_subband_matrix());
        assert_eq!(frame.ms_flags, vec![false]);
        // Silent bands never touch the SCF memory.
        assert_eq!(scf, Sv7ScfMemory::new());
    }

    /// One coded band per channel: the pass order on the wire is
    /// [header][SCFI L][SCFI R][DSCF L][DSCF R][samples L][samples R].
    #[test]
    fn pass_order_scfi_then_dscf_then_samples() {
        let (scfi_c, scfi_l) = scfi3();
        let (d0_c, d0_l) = dscf0();
        let (d1_c, d1_l) = dscf1();
        let (q3_c, q3_l) = (SV7_Q3_TABLE[0].code, SV7_Q3_TABLE[0].length);

        let mut p = Packer::new();
        // §5.1 header: left Res=3, right Res=3 (raw 4-bit each), M/S bit.
        p.push_raw(3, 4);
        p.push_raw(3, 4);
        p.push_raw(1, 1);
        // SCFI pass: L then R.
        p.push(scfi_c, scfi_l);
        p.push(scfi_c, scfi_l);
        // DSCF pass: L (delta 0 → SCF 0), R (delta +1 → SCF 1).
        p.push(d0_c, d0_l);
        p.push(d1_c, d1_l);
        // Samples pass: L selector + 36 q3, R selector + 36 q3.
        p.push_raw(0, 1);
        for _ in 0..36 {
            p.push(q3_c, q3_l);
        }
        p.push_raw(0, 1);
        for _ in 0..36 {
            p.push(q3_c, q3_l);
        }
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let mut cns = CnsPrng::new();
        let mut scf = Sv7ScfMemory::new();
        let frame = decode_sv7_stereo_frame(&mut r, 0, true, &mut scf, &mut cns).unwrap();
        assert_eq!(frame.ms_flags, vec![true]);

        // Same levels, but the right channel's SCF index is 1 (unity
        // gain) while the left's is 0 (one step louder): the absolute
        // law makes L = R / SCF_STEP_RATIO.
        let level = SV7_Q3_TABLE[0].value as f64;
        let c = DEQUANT_COEFFICIENT_C[4];
        let want_l = level * c * SCF_STEP_RATIO.powi(-1);
        let want_r = level * c;
        for k in 0..SAMPLES_PER_BAND {
            assert!((frame.channels[0][0][k] - want_l).abs() < 1e-9, "L {k}");
            assert!((frame.channels[1][0][k] - want_r).abs() < 1e-9, "R {k}");
        }
        // Memory recorded each channel's SCF[2].
        assert_eq!(scf.reference(0, 0), 0);
        assert_eq!(scf.reference(1, 0), 1);
    }

    /// The SCF[0] reference is per-band memory across frames: decoding
    /// the same coded-band bits twice yields a second frame whose SCF
    /// deltas ride on the first frame's SCF[2].
    #[test]
    fn scf_memory_threads_across_frames() {
        let (scfi_c, scfi_l) = scfi3();
        let (d1_c, d1_l) = dscf1();
        let (q3_c, q3_l) = (SV7_Q3_TABLE[0].code, SV7_Q3_TABLE[0].length);

        let frame_bits = |p: &mut Packer| {
            p.push_raw(3, 4);
            p.push_raw(3, 4);
            // stream_ms off ⇒ no M/S bit.
            p.push(scfi_c, scfi_l); // SCFI L
            p.push(scfi_c, scfi_l); // SCFI R
            p.push(d1_c, d1_l); // DSCF L: +1 off memory
            p.push(d1_c, d1_l); // DSCF R: +1 off memory
            p.push_raw(0, 1);
            for _ in 0..36 {
                p.push(q3_c, q3_l);
            }
            p.push_raw(0, 1);
            for _ in 0..36 {
                p.push(q3_c, q3_l);
            }
        };
        let mut p = Packer::new();
        frame_bits(&mut p);
        frame_bits(&mut p);
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let mut cns = CnsPrng::new();
        let mut scf = Sv7ScfMemory::new();
        let f1 = decode_sv7_stereo_frame(&mut r, 0, false, &mut scf, &mut cns).unwrap();
        assert_eq!(scf.reference(0, 0), 1, "frame 1: 0 + 1");
        let f2 = decode_sv7_stereo_frame(&mut r, 0, false, &mut scf, &mut cns).unwrap();
        assert_eq!(scf.reference(0, 0), 2, "frame 2: 1 + 1");
        // Higher SCF index = quieter: frame 2 is one ratio step down.
        let x1 = f1.channels[0][0][0];
        let x2 = f2.channels[0][0][0];
        assert!((x2 / x1 - SCF_STEP_RATIO).abs() < 1e-9, "{x2} / {x1}");
    }

    /// CNS bands read SCFI + DSCF (the §5.2 "Res ≠ 0" gate) and scale
    /// the PRNG noise by the per-granule gain.
    #[test]
    fn cns_band_reads_scf_layer_and_scales_noise() {
        let (scfi_c, scfi_l) = scfi3();
        let (d0_c, d0_l) = dscf0();
        // max_band = 1: band0 Res=0 raw, band1 delta -1 → CNS.
        let mut p = Packer::new();
        p.push_raw(0, 4); // L band0
        p.push_raw(0, 4); // R band0
        p.push(0x0000, 2); // L band1 delta -1 → Res -1
        p.push(0x0000, 2); // R band1 delta -1 → Res -1
                           // SCFI pass: band1 L, band1 R (band0 silent).
        p.push(scfi_c, scfi_l);
        p.push(scfi_c, scfi_l);
        // DSCF pass: band1 L (delta 0 → SCF 0), band1 R (delta 0).
        p.push(d0_c, d0_l);
        p.push(d0_c, d0_l);
        // Samples pass: CNS reads no bits (PRNG).
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let mut cns = CnsPrng::new();
        let mut scf = Sv7ScfMemory::new();
        let frame = decode_sv7_stereo_frame(&mut r, 1, false, &mut scf, &mut cns).unwrap();

        // Reference: left drains 36 PRNG samples, then right.
        let mut ref_cns = CnsPrng::new();
        let mut left_ref = [0_i32; 36];
        ref_cns.fill_samples(&mut left_ref);
        let mut right_ref = [0_i32; 36];
        ref_cns.fill_samples(&mut right_ref);
        assert_eq!(cns.state(), ref_cns.state());
        // Scaled by C[0] × gain(SCF 0) = C[0] × ratio^(−1).
        let gain = DEQUANT_COEFFICIENT_C[0] * SCF_STEP_RATIO.powi(-1);
        for k in 0..SAMPLES_PER_BAND {
            assert!(
                (frame.channels[0][1][k] - left_ref[k] as f64 * gain).abs() < 1e-9,
                "L {k}"
            );
            assert!(
                (frame.channels[1][1][k] - right_ref[k] as f64 * gain).abs() < 1e-9,
                "R {k}"
            );
        }
        // The two channels' noise rows differ (PRNG advanced between).
        assert_ne!(frame.channels[0][1], frame.channels[1][1]);
    }

    #[test]
    fn rejects_max_band_out_of_range() {
        let mut r = Sv7BitReader::new(&[0xFF; 8]);
        let mut cns = CnsPrng::new();
        let mut scf = Sv7ScfMemory::new();
        assert_eq!(
            decode_sv7_stereo_frame(&mut r, 32, false, &mut scf, &mut cns),
            Err(crate::Error::MaxBandOutOfRange(32))
        );
    }

    #[test]
    fn frame_pairs_with_pinned_ms_undo() {
        let mut p = Packer::new();
        p.push_raw(0, 4);
        p.push_raw(0, 4);
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let mut cns = CnsPrng::new();
        let mut scf = Sv7ScfMemory::new();
        let mut frame = decode_sv7_stereo_frame(&mut r, 0, false, &mut scf, &mut cns).unwrap();
        crate::ms_stereo::undo_ms_stereo_pinned(&mut frame.channels, &frame.ms_flags).unwrap();
        assert_eq!(frame.channels[0], zero_subband_matrix());
        // Touch the SCFI table for import parity.
        assert_eq!(SV7_SCFI_TABLE.len(), 4);
    }

    #[test]
    fn scf_memory_default_and_reset() {
        let mut m = Sv7ScfMemory::default();
        m.update(1, 30, 42);
        assert_eq!(m.reference(1, 30), 42);
        m.reset();
        assert_eq!(m, Sv7ScfMemory::new());
    }
}
