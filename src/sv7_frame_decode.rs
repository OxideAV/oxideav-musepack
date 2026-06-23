//! SV7 single-channel audio-frame-body assembler.
//!
//! The SV7 counterpart of [`crate::sv8_frame_decode`]: it joins the
//! grounded SV7 sub-walks into a single per-channel frame-body decode, a
//! sequence of [`crate::frame_reconstruct::BandLevels`] records — one per
//! coded subband — ready for [`crate::frame_reconstruct::reconstruct_frame_channel`]
//! (dequant + per-granule SCF multiply).
//!
//! # Phase ordering (the documented §5 frame-body layout)
//!
//! The staged `spec/musepack-headers-and-coding.md` §5 lays the SV7
//! frame body out as: §5.1 the per-band `Res` (band-type) header, §5.2 /
//! §5.3 the per-non-zero-band SCFI + DSCF scalefactor layer, and §5.4 the
//! per-band quantised samples. This assembler walks one channel's bands
//! in that order: for each band, given the channel's already-decoded
//! `Res`, it
//!
//! 1. emits a silent record for `Res == 0` (empty) — no SCF / sample
//!    reads;
//! 2. fills 36 samples from the shared CNS PRNG for `Res == -1` (noise),
//!    with no SCF layer (the noise band carries no scalefactor);
//! 3. for a coded band (`Res` in `1..=17`), decodes the §5.3 SCF indices
//!    ([`decode_sv7_band_scf`], threading the previous band's `SCF[2]`
//!    forward per §5.3), reads the §5.4 **1-bit context selector** when
//!    the band-type uses one (cases `1` / `2` / `3..=7`), then decodes
//!    the 36 sample levels ([`decode_sv7_band`]).
//!
//! # The §5.4 1-bit context selector
//!
//! §5.4 reads, **before** the sample codewords of a `Res`-`1` / `2` /
//! `3..=7` band, a single raw bit picking one of the band-type's two
//! context tables. The CNS (`-1`), empty (`0`), and linear-PCM-escape
//! (`8..=17`) arms read **no** selector — they take no context. This
//! assembler reads the selector bit exactly for the
//! [`band_type_uses_context_selector`] cases and passes it as the `ctx`
//! argument to [`decode_sv7_band`]; the no-context arms pass `ctx = 0`,
//! which those arms ignore.
//!
//! # Scope: single channel
//!
//! Like [`crate::sv8_frame_decode::decode_sv8_frame_channel`], this walks
//! **one channel**. §5.1 reads both channels' `Res` interleaved per band;
//! §5.3 / §5.4 then say "Left channel is decoded first, then right." This
//! assembler takes a single channel's `Res` sequence (the left or right
//! column of [`crate::sv7_band_header::decode_res_header_grounded`]) and
//! decodes that channel's SCF-then-samples per band. The **cross-channel
//! interleaving** (whether the whole SCF+sample body is two back-to-back
//! channel sweeps, or the channels interleave per band) and the **M/S
//! undo** that follows reconstruction remain GAP — the same gaps the SV8
//! assembler and [`crate::ms_stereo`] document.
//!
//! # The SCF anchor
//!
//! §5.3 threads each band's `SCF[0]` off the previous band's `SCF[2]`;
//! the **first** band's reference (the channel's starting SCF anchor) is
//! the absolute SCF anchor that §2.6 lists as GAP. This assembler takes
//! it as a `first_scf_ref` argument (default `0`), matching the
//! relative-anchor convention [`crate::frame_reconstruct`] uses.
//!
//! Source-of-record: `docs/audio/musepack/spec/musepack-headers-and-coding.md`
//! §5.1–§5.4, and the sibling modules under
//! `crates/oxideav-musepack/src/`. No new format facts are introduced —
//! this is composition of the already-grounded sub-walks in the
//! documented §5 phase order.

use crate::cns::CnsPrng;
use crate::frame_reconstruct::BandLevels;
use crate::huffman::Sv7BitReader;
use crate::reconstruct::GRANULES_PER_BAND;
use crate::sv7_band_decode::{band_type_case, decode_sv7_band, BandDecodeCase, SAMPLES_PER_BAND};
use crate::sv7_band_header::SV7_SUBBAND_COUNT;
use crate::sv7_scf_decode::decode_sv7_band_scf;
use crate::{Error, Result};

/// True iff a `band_type`'s §5.4 sample decode is preceded by the 1-bit
/// context selector: the grouped (`1` / `2`) and per-sample Huffman
/// (`3..=7`) cases. The CNS (`-1`), empty (`0`), and linear-PCM-escape
/// (`8..=17`) cases read no selector.
pub const fn band_type_uses_context_selector(band_type: i8) -> bool {
    matches!(
        band_type_case(band_type),
        BandDecodeCase::Grouped3 | BandDecodeCase::Grouped2 | BandDecodeCase::HuffmanPerSample
    )
}

/// Decode one channel's SV7 frame body into a sequence of
/// per-coded-subband [`BandLevels`] records, given that channel's
/// already-decoded per-band `Res` (band_type) sequence.
///
/// `res_per_band[b]` is subband `b`'s `Res` for this channel (the §5.1
/// [`crate::sv7_band_header::decode_res_header_grounded`] output column).
/// Its length is the frame's band count and must not exceed
/// [`SV7_SUBBAND_COUNT`].
///
/// For each band in ascending order:
///
/// - **`Res == 0`** (empty): no record is emitted (the subband stays
///   silent — [`reconstruct_frame_channel`] zero-fills absent subbands).
/// - **`Res == -1`** (CNS): a record with 36 PRNG samples and a zero SCF
///   triple (the noise band carries no scalefactor layer).
/// - **otherwise** (coded, `Res` in `1..=17`): decode the §5.3 SCF
///   (threading the previous coded band's `SCF[2]`), read the §5.4
///   context-selector bit when [`band_type_uses_context_selector`], then
///   decode the 36 sample levels.
///
/// `first_scf_ref` is the §5.3 `SCF[0]` reference for the **first** coded
/// band (the channel's SCF anchor, GAP per §2.6; pass `0` for the
/// relative-anchor convention). `cns` is the shared CNS PRNG, advanced by
/// every noise band so its state carries across bands exactly.
///
/// # Errors
///
/// - [`Error::MaxBandOutOfRange`] if `res_per_band` is longer than
///   [`SV7_SUBBAND_COUNT`].
/// - [`Error::UnexpectedEof`] / [`Error::HuffmanNoMatch`] if the reader
///   starves or a peek matches no row in any phase.
/// - [`Error::UnsupportedBandType`] for a `Res` outside `-1..=17` (the
///   §5.4 switch domain).
/// - [`Error::InvalidScfCodingMethod`] propagated from the §5.3 SCF
///   decode.
pub fn decode_sv7_frame_channel(
    reader: &mut Sv7BitReader<'_>,
    res_per_band: &[i8],
    first_scf_ref: i32,
    cns: &mut CnsPrng,
) -> Result<Vec<BandLevels>> {
    if res_per_band.len() > SV7_SUBBAND_COUNT {
        return Err(Error::MaxBandOutOfRange(res_per_band.len() as u8));
    }

    let mut out = Vec::new();
    let mut prev_scf2 = first_scf_ref;
    for (subband, &band_type) in res_per_band.iter().enumerate() {
        match band_type_case(band_type) {
            BandDecodeCase::Empty => {
                // Silent subband: no SCF / sample reads, no record.
            }
            BandDecodeCase::Cns => {
                // Noise band: 36 PRNG samples, no SCF layer.
                let mut levels = [0_i32; SAMPLES_PER_BAND];
                decode_sv7_band(reader, band_type, cns, 0, &mut levels)?;
                out.push(BandLevels {
                    subband,
                    band_type,
                    levels,
                    granule_scf: [0; GRANULES_PER_BAND],
                });
            }
            BandDecodeCase::OutOfRange => {
                return Err(Error::UnsupportedBandType(band_type));
            }
            _ => {
                // Coded band: §5.3 SCF, then §5.4 context selector +
                // samples.
                let scf = decode_sv7_band_scf(reader, prev_scf2)?;
                prev_scf2 = scf.last_index();
                let ctx = if band_type_uses_context_selector(band_type) {
                    (reader.read_bits(1)? & 1) as usize
                } else {
                    0
                };
                let mut levels = [0_i32; SAMPLES_PER_BAND];
                decode_sv7_band(reader, band_type, cns, ctx, &mut levels)?;
                out.push(BandLevels {
                    subband,
                    band_type,
                    levels,
                    granule_scf: scf_indices_to_u32(scf.indices),
                });
            }
        }
    }
    Ok(out)
}

/// Widen the §5.3 signed `[i32; 3]` SCF indices into the `[u32; 3]`
/// triple [`BandLevels`] carries. Negative indices (possible from a
/// delta chain that dipped below zero before the absolute anchor is
/// applied) saturate at `0` — the relative-anchor convention keeps the
/// inter-granule spacing exact, and the reconstruction layer validates
/// the final index against the SCF ladder.
fn scf_indices_to_u32(indices: [i32; GRANULES_PER_BAND]) -> [u32; GRANULES_PER_BAND] {
    let mut out = [0_u32; GRANULES_PER_BAND];
    for (slot, &idx) in out.iter_mut().zip(indices.iter()) {
        *slot = idx.max(0) as u32;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::huffman::{SV7_DSCF_TABLE, SV7_Q1_TABLE, SV7_Q3_TABLE, SV7_SCFI_TABLE};

    /// MSB-first packer (mirrors the other SV7 sub-walk tests).
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
            self.bytes.push(0);
            self.bytes.push(0);
            self.bytes
        }
    }

    /// First (shortest, top-of-table) codeword of a table.
    fn first_code(table: &[crate::huffman::Sv7Entry]) -> (u16, u8) {
        (table[0].code, table[0].length)
    }

    /// SCFI codeword for value 3 (only SCF[0] coded — simplest SCF).
    fn scfi3() -> (u16, u8) {
        (0x0000, 2)
    }
    /// DSCF codeword for symbol 0 (no-op delta).
    fn dscf0() -> (u16, u8) {
        (0x9000, 4)
    }

    #[test]
    fn context_selector_predicate_matches_spec_cases() {
        // Selector for grouped (1,2) and per-sample Huffman (3..=7).
        for bt in [1, 2, 3, 4, 5, 6, 7] {
            assert!(band_type_uses_context_selector(bt), "bt {bt}");
        }
        // No selector for CNS, empty, PCM-escape.
        for bt in [-1, 0, 8, 12, 17] {
            assert!(!band_type_uses_context_selector(bt), "bt {bt}");
        }
    }

    #[test]
    fn empty_bands_emit_no_records() {
        let mut r = Sv7BitReader::new(&[0xFF; 8]);
        let mut cns = CnsPrng::new();
        let out = decode_sv7_frame_channel(&mut r, &[0, 0, 0], 0, &mut cns).unwrap();
        assert!(out.is_empty());
    }

    #[test]
    fn rejects_too_many_bands() {
        let mut r = Sv7BitReader::new(&[0xFF; 8]);
        let mut cns = CnsPrng::new();
        let res = vec![0_i8; SV7_SUBBAND_COUNT + 1];
        assert_eq!(
            decode_sv7_frame_channel(&mut r, &res, 0, &mut cns),
            Err(Error::MaxBandOutOfRange((SV7_SUBBAND_COUNT + 1) as u8))
        );
    }

    #[test]
    fn cns_band_fills_from_prng_without_scf_and_advances_state() {
        // A single Res == -1 band reads no SCF / selector — just the PRNG.
        let mut r = Sv7BitReader::new(&[0xFF; 8]);
        let mut cns = CnsPrng::new();
        let out = decode_sv7_frame_channel(&mut r, &[-1], 0, &mut cns).unwrap();
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].band_type, -1);
        assert_eq!(out[0].granule_scf, [0, 0, 0]);

        let mut direct = [0_i32; SAMPLES_PER_BAND];
        let mut cns2 = CnsPrng::new();
        cns2.fill_samples(&mut direct);
        assert_eq!(out[0].levels, direct);
        assert_eq!(cns.state(), cns2.state());
    }

    #[test]
    fn rejects_band_type_out_of_range() {
        let mut r = Sv7BitReader::new(&[0xFF; 8]);
        let mut cns = CnsPrng::new();
        assert_eq!(
            decode_sv7_frame_channel(&mut r, &[18], 0, &mut cns),
            Err(Error::UnsupportedBandType(18))
        );
        assert_eq!(
            decode_sv7_frame_channel(&mut r, &[-2], 0, &mut cns),
            Err(Error::UnsupportedBandType(-2))
        );
    }

    #[test]
    fn coded_huffman_band_reads_scf_then_selector_then_samples() {
        // One Res == 3 band (per-sample Huffman, uses the context
        // selector). Stream order: [SCFI=3][DSCF=0][selector bit][36 q3].
        let (scfi_c, scfi_l) = scfi3();
        let (dscf_c, dscf_l) = dscf0();
        let (q3_c, q3_l) = first_code(&SV7_Q3_TABLE);

        let mut p = Packer::new();
        p.push(scfi_c, scfi_l);
        p.push(dscf_c, dscf_l);
        p.push_raw(0, 1); // context selector = 0
        for _ in 0..SAMPLES_PER_BAND {
            p.push(q3_c, q3_l);
        }
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let mut cns = CnsPrng::new();
        let out = decode_sv7_frame_channel(&mut r, &[3], 100, &mut cns).unwrap();
        assert_eq!(out.len(), 1);
        let band = &out[0];
        assert_eq!(band.band_type, 3);
        assert_eq!(band.subband, 0);
        // SCFI 3 ⇒ all granules share SCF[0] = 100 + 0 = 100.
        assert_eq!(band.granule_scf, [100, 100, 100]);
        // Every q3 level lands; the first row's symbol fills all 36.
        let want = SV7_Q3_TABLE[0].value as i32;
        assert!(band.levels.iter().all(|&s| s == want));
    }

    #[test]
    fn pcm_escape_band_reads_scf_but_no_selector() {
        // Res == 8 (linear-PCM escape, 7 bits/sample, no selector).
        // Stream: [SCFI=3][DSCF=0][36 × 7-bit raw], no selector bit.
        let (scfi_c, scfi_l) = scfi3();
        let (dscf_c, dscf_l) = dscf0();
        let mut p = Packer::new();
        p.push(scfi_c, scfi_l);
        p.push(dscf_c, dscf_l);
        for _ in 0..SAMPLES_PER_BAND {
            p.push_raw(42, 7); // band_type 8 ⇒ 7 raw bits/sample
        }
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let mut cns = CnsPrng::new();
        let out = decode_sv7_frame_channel(&mut r, &[8], 0, &mut cns).unwrap();
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].band_type, 8);
        // Raw level 42 stored verbatim (pre-centring) for every sample.
        assert!(out[0].levels.iter().all(|&s| s == 42));
    }

    #[test]
    fn prev_band_scf2_threads_across_coded_bands() {
        // Two Res == 1 grouped bands (each uses the selector + 12 q1
        // codewords). SCFI 0 on each so SCF[2] is independently coded and
        // band 1's SCF[0] folds off band 0's SCF[2].
        let scfi0 = (0x4000_u16, 3_u8); // SCFI value 0
                                        // DSCF deltas: band0 → +1,+1,+1 ; band1 → +2,0,0.
        let dscf_p1 = (0xa000_u16, 3_u8); // +1
        let dscf_p2 = (0x4000_u16, 3_u8); // +2
        let dscf_z = dscf0();
        let (q1_c, q1_l) = first_code(&SV7_Q1_TABLE);

        let mut p = Packer::new();
        // band 0
        p.push(scfi0.0, scfi0.1);
        p.push(dscf_p1.0, dscf_p1.1); // SCF[0]=0+1=1
        p.push(dscf_p1.0, dscf_p1.1); // SCF[1]=1+1=2
        p.push(dscf_p1.0, dscf_p1.1); // SCF[2]=2+1=3
        p.push_raw(0, 1); // selector
        for _ in 0..12 {
            p.push(q1_c, q1_l);
        }
        // band 1
        p.push(scfi0.0, scfi0.1);
        p.push(dscf_p2.0, dscf_p2.1); // SCF[0]=3+2=5 (off band0 SCF[2])
        p.push(dscf_z.0, dscf_z.1); // SCF[1]=5+0=5
        p.push(dscf_z.0, dscf_z.1); // SCF[2]=5+0=5
        p.push_raw(1, 1); // selector ctx 1
        for _ in 0..12 {
            p.push(q1_c, q1_l);
        }
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let mut cns = CnsPrng::new();
        let out = decode_sv7_frame_channel(&mut r, &[1, 1], 0, &mut cns).unwrap();
        assert_eq!(out.len(), 2);
        assert_eq!(out[0].granule_scf, [1, 2, 3]);
        assert_eq!(out[1].granule_scf, [5, 5, 5]);
        assert_eq!(out[0].subband, 0);
        assert_eq!(out[1].subband, 1);
    }

    #[test]
    fn empty_bands_interleave_with_coded_bands_keeping_subband_index() {
        // [empty, CNS, empty]: only the CNS band emits a record, at
        // subband index 1.
        let mut r = Sv7BitReader::new(&[0xFF; 16]);
        let mut cns = CnsPrng::new();
        let out = decode_sv7_frame_channel(&mut r, &[0, -1, 0], 0, &mut cns).unwrap();
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].subband, 1);
        assert_eq!(out[0].band_type, -1);
    }

    #[test]
    fn scf_indices_saturate_negative_to_zero() {
        // A delta chain dipping below zero (anchor 0, delta -2 via the
        // DSCF table) saturates to 0 in the BandLevels u32 triple.
        let (scfi_c, scfi_l) = scfi3();
        let dscf_neg2 = (0x0000_u16, 3_u8); // DSCF symbol -2
        let (q3_c, q3_l) = first_code(&SV7_Q3_TABLE);
        let mut p = Packer::new();
        p.push(scfi_c, scfi_l);
        p.push(dscf_neg2.0, dscf_neg2.1);
        p.push_raw(0, 1);
        for _ in 0..SAMPLES_PER_BAND {
            p.push(q3_c, q3_l);
        }
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let mut cns = CnsPrng::new();
        let out = decode_sv7_frame_channel(&mut r, &[3], 0, &mut cns).unwrap();
        // SCF[0] = 0 + (-2) = -2 → saturates to 0 in the u32 triple.
        assert_eq!(out[0].granule_scf, [0, 0, 0]);
    }

    #[test]
    fn propagates_eof_when_sample_phase_starves() {
        // Res == 3 promises SCF + selector + 36 samples but the stream
        // ends after the SCF. The sample peek starves.
        let (scfi_c, scfi_l) = scfi3();
        let (dscf_c, dscf_l) = dscf0();
        let mut p = Packer::new();
        p.push(scfi_c, scfi_l);
        p.push(dscf_c, dscf_l);
        // Flush without trailing peek padding ⇒ later reads starve.
        let mut bytes = p.bytes.clone();
        if p.nbits > 0 {
            bytes.push((p.acc << (8 - p.nbits)) as u8);
        }
        let mut r = Sv7BitReader::new(&bytes);
        let mut cns = CnsPrng::new();
        assert_eq!(
            decode_sv7_frame_channel(&mut r, &[3], 0, &mut cns),
            Err(Error::UnexpectedEof)
        );
        // Touch SV7_DSCF_TABLE / SV7_SCFI_TABLE for import parity.
        assert_eq!(SV7_DSCF_TABLE.len(), 16);
        assert_eq!(SV7_SCFI_TABLE.len(), 4);
    }
}
