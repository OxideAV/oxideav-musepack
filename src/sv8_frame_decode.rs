//! SV8 single-channel audio-packet frame-body assembler.
//!
//! Each grounded SV8 sub-walk landed in earlier rounds covers one phase
//! of the §3.4/§3.5/§6 frame body in isolation:
//!
//! - [`crate::sv8_band_header::decode_band_resolutions_grounded`] — the
//!   §6.2 per-band band-resolution (`band_type`) walk (top-down decode,
//!   signed wrap, ascending output).
//! - [`crate::sv8_scf_header::decode_sv8_scfi`] — the §6.3 SCFI selector
//!   decode (context + L/R packed split).
//! - [`crate::sv8_dscf_loop::decode_sv8_band_scf`] — the §6.3 DSCF →
//!   per-granule SCF-index reconstruction.
//! - [`crate::sv8_band_decode::decode_sv8_band_grounded`] — the §3.4 /
//!   §6.4 per-band sample decode (every case arm grounded).
//!
//! This module is the **integrating layer** that joins them into a
//! single per-channel frame-body decode: a sequence of
//! [`Sv8BandDecode`] records, one per coded subband, each carrying its
//! `band_type`, three reconstructed SCF indices, and 36 decoded sample
//! levels — the structured input the §2.6 / §3.6 reconstruction step
//! (dequant + per-granule SCF multiply + the synthesis filterbank) will
//! consume.
//!
//! # Phase ordering (the documented frame-body layout)
//!
//! `musepack-sv7-sv8-spec.md` lays the SV7 frame body out as **three
//! separate per-band sweeps** — §2.3 band-type headers, then §2.4 SCF
//! coding, then §2.5 quantised samples ("after all bands are decoded:
//! requantise …", §2.6). This module follows that documented phase
//! layout for SV8:
//!
//! 1. **Resolution sweep** — one [`decode_band_resolutions_grounded`]
//!    call reads every coded band's `band_type` (§6.2).
//! 2. **SCFI + DSCF + sample sweep** — then, per non-zero band in
//!    ascending order: one §6.3 SCFI decode, the §6.3 per-granule SCF
//!    reconstruction, and the §3.4 sample decode. (The §6.3 SCFI and
//!    DSCF reads sit immediately before the band's samples; §3.5 frames
//!    them as the band's own scalefactor layer.)
//!
//! The cross-phase ordering between SCF and samples (whether SV8 reads
//! *all* bands' SCFs before *any* samples, as SV7 §2.4→§2.5 suggests, or
//! interleaves per band) is not pinned cell-for-cell by the staged
//! material — see the "Still GAP" note below.
//!
//! # Scope: single channel
//!
//! This assembler walks **one channel**. The §3.4 prose reproduces only
//! the per-band sample `switch`, not a channel loop, and the SV8
//! per-channel interleaving (whether L and R bands alternate per band,
//! or each channel is a full sweep) is GAP — the same gap
//! [`crate::sv8_band_header`] documents. A mono stream, or one channel of
//! a stereo stream whose channel layout the caller has already resolved,
//! decodes fully here; the multi-channel composition (and the M/S undo
//! that follows it) is left to a future round once the channel-loop shape
//! is pinned. For the single-channel SCFI the band's non-zero-channel
//! count is `1`, so [`decode_sv8_scfi`] reads the `scfi-1` context and
//! the SCFI value is the channel's selector directly.
//!
//! # Still GAP downstream
//!
//! - **Cross-phase SCF/sample ordering** and **per-channel
//!   interleaving** (above).
//! - **The §6.3 "new-block" flag source.** §6.2 forces it set on every
//!   key frame (scalefactors coded absolutely); on a non-key frame it is
//!   a per-band flag whose bitstream position the staged material does
//!   not pin. This assembler takes `new_block` as a caller argument
//!   (set it `true` for a key frame).
//! - **Dequant + per-granule SCF multiply + synthesis filterbank** — the
//!   reconstruction beyond the structured per-band decode, documented in
//!   `crate::reconstruct` / `crate::frame_reconstruct` (and partly GAP:
//!   the absolute SCF anchor gain and the Layer-II synthesis window).
//!
//! No new format facts are introduced: this is pure composition of the
//! already-grounded sub-walks in the documented phase order.
//!
//! Source-of-record: `docs/audio/musepack/musepack-sv7-sv8-spec.md`
//! §2.3–§2.6 (frame-body phase layout) + §1 (32-subband geometry), and
//! `spec/musepack-headers-and-coding.md` §6.2 / §6.3 / §6.4 (the grounded
//! sub-walks). The only project material crossed is that staged `docs/`
//! content and the sibling modules under
//! `crates/oxideav-musepack/src/`.

use crate::cns::CnsPrng;
use crate::huffman::Sv7BitReader;
use crate::scf::SCF_GRANULES_PER_BAND;
use crate::sv7_band_decode::SAMPLES_PER_BAND;
use crate::sv7_band_header::SV7_SUBBAND_COUNT;
use crate::sv8_band_decode::{decode_sv8_band_grounded, sv8_band_type_case, Sv8BandDecodeCase};
use crate::sv8_band_header::decode_band_resolutions_grounded;
use crate::sv8_dscf_loop::decode_sv8_band_scf;
use crate::sv8_scf_header::decode_sv8_scfi;
use crate::{Error, Result};

/// One coded subband's decoded SV8 frame-body data for one channel.
///
/// The structured output of [`decode_sv8_frame_channel`], ready for the
/// §2.6 / §3.6 reconstruction step (dequant + per-granule SCF multiply).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Sv8BandDecode {
    /// Subband index (`0..`[`SV7_SUBBAND_COUNT`]).
    pub subband: usize,
    /// The §6.2 signed `band_type` (`-1..=17`) this band decoded to.
    pub band_type: i8,
    /// The three §6.3 per-granule SCF indices (granules 0, 1, 2).
    /// Empty / silent for a `band_type == 0` band (no SCF layer).
    pub granule_scf: [i32; SCF_GRANULES_PER_BAND],
    /// The 36 decoded sample levels (already centred for the grouped /
    /// context arms; raw-but-signed for the escape arm; PRNG-derived for
    /// CNS). All-zero for an empty band.
    pub levels: [i32; SAMPLES_PER_BAND],
}

/// Decode one channel's SV8 audio-packet frame body into a sequence of
/// per-coded-subband [`Sv8BandDecode`] records.
///
/// Composes the grounded SV8 sub-walks in the documented frame-body
/// phase order (see the module docs):
///
/// 1. One [`decode_band_resolutions_grounded`] call reads `nbands`
///    band-resolutions (the §6.2 used-band count, typically the
///    [`crate::sv8_band_header::decode_used_subbands`] result) into the
///    ascending-band-order `band_type` sequence.
/// 2. For each band in ascending order:
///    - **`band_type == 0`** (empty): emit a silent record (no SCF /
///      sample reads).
///    - **`band_type == -1`** (CNS): fill 36 samples from `cns`; no SCF
///      layer (the noise band carries no scalefactor selector).
///    - **otherwise** (a coded band): decode the §6.3 SCFI selector
///      ([`decode_sv8_scfi`] with `nonzero_channels = 1` for this single
///      channel), reconstruct the three §6.3 SCF indices
///      ([`decode_sv8_band_scf`], threading the previous band's `SCF[2]`
///      as the next band's `SCF[0]` reference), then decode the band's
///      36 sample levels ([`decode_sv8_band_grounded`]).
///
/// `new_block` is the §6.3 per-band "new-block" flag (forced set on a key
/// frame; see the module docs). `cns` is the shared CNS PRNG, advanced by
/// every noise band so its state carries across bands exactly.
///
/// The `SCF[0]` reference for the **first** coded band is `0` (no
/// previous band); §6.2 forces a key frame's first band to `new_block`
/// anyway, so the reference is unused there.
///
/// # Errors
///
/// - [`Error::UnexpectedEof`] if the reader starves in any phase.
/// - [`Error::HuffmanNoMatch`] if a canonical peek matches no row
///   (unreachable for the staged tables).
/// - [`Error::MaxBandOutOfRange`] if `nbands > `[`SV7_SUBBAND_COUNT`].
/// - [`Error::UnsupportedBandType`] from the §6.2 walk or the §3.4
///   sample decode for a `band_type` outside `-1..=17`.
/// - [`Error::InvalidScfCodingMethod`] / [`Error::ChannelCountInvalid`]
///   propagated from the §6.3 SCFI / SCF decode.
pub fn decode_sv8_frame_channel(
    reader: &mut Sv7BitReader<'_>,
    nbands: u8,
    new_block: bool,
    cns: &mut CnsPrng,
) -> Result<Vec<Sv8BandDecode>> {
    if nbands as usize > SV7_SUBBAND_COUNT {
        return Err(Error::MaxBandOutOfRange(nbands));
    }

    // Phase 1: resolution sweep (§6.2, ascending band order).
    let band_types = decode_band_resolutions_grounded(reader, nbands)?;

    // Phase 2: per-band SCFI + DSCF + samples, ascending band order.
    let mut out = Vec::with_capacity(band_types.len());
    let mut prev_scf2: i32 = 0;
    for (subband, &band_type) in band_types.iter().enumerate() {
        let mut levels = [0_i32; SAMPLES_PER_BAND];
        let granule_scf = match sv8_band_type_case(band_type) {
            Sv8BandDecodeCase::Empty => {
                // No SCF layer, no sample reads — a silent band.
                [0_i32; SCF_GRANULES_PER_BAND]
            }
            Sv8BandDecodeCase::Cns => {
                // Noise band: samples from the PRNG, no SCF selector.
                decode_sv8_band_grounded(reader, band_type, cns, &mut levels)?;
                [0_i32; SCF_GRANULES_PER_BAND]
            }
            Sv8BandDecodeCase::OutOfRange => {
                return Err(Error::UnsupportedBandType(band_type));
            }
            _ => {
                // Coded band: SCFI selector, SCF indices, then samples.
                let scfi = decode_sv8_scfi(reader, 1)?;
                let scf = decode_sv8_band_scf(reader, scfi.left, new_block, prev_scf2)?;
                prev_scf2 = scf[SCF_GRANULES_PER_BAND - 1];
                decode_sv8_band_grounded(reader, band_type, cns, &mut levels)?;
                scf
            }
        };
        out.push(Sv8BandDecode {
            subband,
            band_type,
            granule_scf,
            levels,
        });
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sv8_huffman::{
        table_for_role, Sv8CanonicalTable, Sv8TableRole, SV8_RES_1_TABLE, SV8_RES_2_TABLE,
        SV8_SCFI_1_TABLE,
    };

    /// MSB-first left-justified bit packer (mirrors the SV8 sub-walk
    /// tests): `push` a `length`-bit codeword from the top of `pattern`;
    /// `push_raw` a right-justified raw field; `finish` flushes + appends
    /// two zero bytes so `peek16` never starves mid-decode.
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

    /// Decode `table`'s row-`r` codeword once, returning the symbol.
    fn symbol_for_row(table: &Sv8CanonicalTable, r: usize) -> i8 {
        let e = table.lengths[r];
        let mut p = BitPacker::new();
        p.push(e.code, e.length);
        let bytes = p.finish();
        let mut reader = Sv7BitReader::new(&bytes);
        table.decode(&mut reader).expect("single-codeword decode")
    }

    /// Find a `(pattern, length)` codeword of `table` whose decoded
    /// symbol equals `target`.
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

    /// Spec-replica §6.3 fold: `((prev − 25 + delta) & 127) − 6`.
    fn fold_ref(prev: i32, delta: i32) -> i32 {
        ((prev - 25 + delta) & 127) - 6
    }

    /// The §6.2 signed wrap for a top-band raw value (values > 15 wrap −17).
    fn wrap_ref(v: i32) -> i8 {
        if v > 15 {
            (v - 17) as i8
        } else {
            v as i8
        }
    }

    #[test]
    fn zero_bands_decodes_to_empty_sequence() {
        let mut reader = Sv7BitReader::new(&[0xFF; 4]);
        let mut cns = CnsPrng::new();
        let out = decode_sv8_frame_channel(&mut reader, 0, true, &mut cns).unwrap();
        assert!(out.is_empty());
    }

    #[test]
    fn rejects_nbands_above_subband_count() {
        let mut reader = Sv7BitReader::new(&[0xFF; 16]);
        let mut cns = CnsPrng::new();
        assert_eq!(
            decode_sv8_frame_channel(&mut reader, 33, true, &mut cns),
            Err(Error::MaxBandOutOfRange(33))
        );
    }

    #[test]
    fn single_empty_band_emits_silent_record_and_reads_only_resolution() {
        // Find a res-1 row whose top-band wrap is band_type 0 (empty):
        // raw value 0 wraps to 0.
        let (code, len) = codeword_for_symbol(&SV8_RES_1_TABLE, 0).expect("res-1 has symbol 0");
        let mut p = BitPacker::new();
        p.push(code, len);
        let bytes = p.finish();
        let mut reader = Sv7BitReader::new(&bytes);
        let mut cns = CnsPrng::new();
        let out = decode_sv8_frame_channel(&mut reader, 1, true, &mut cns).unwrap();
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].band_type, 0);
        assert_eq!(out[0].subband, 0);
        assert_eq!(out[0].granule_scf, [0, 0, 0]);
        assert!(out[0].levels.iter().all(|&s| s == 0));
    }

    #[test]
    fn single_cns_band_fills_samples_and_advances_prng_without_scf() {
        // band_type -1: raw value 16 wraps to -1 (the CNS case).
        let (code, len) = codeword_for_symbol(&SV8_RES_1_TABLE, 16).expect("res-1 has symbol 16");
        let mut p = BitPacker::new();
        p.push(code, len);
        let bytes = p.finish();
        let mut reader = Sv7BitReader::new(&bytes);

        let mut cns = CnsPrng::new();
        let out = decode_sv8_frame_channel(&mut reader, 1, true, &mut cns).unwrap();
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].band_type, -1);
        assert_eq!(out[0].granule_scf, [0, 0, 0]); // CNS carries no SCF layer

        // The same PRNG walked directly must match the decoded levels.
        let mut direct = [0_i32; SAMPLES_PER_BAND];
        let mut cns2 = CnsPrng::new();
        cns2.fill_samples(&mut direct);
        assert_eq!(out[0].levels, direct);
        assert_eq!(cns.state(), cns2.state());
    }

    #[test]
    fn single_coded_band_decodes_scfi_scf_then_samples() {
        // A single coded band, band_type 5 (context per-sample VLC).
        // Stream order: [res VLC] [SCFI VLC] [SCF: new-block raw7] [36 samples].
        // band_type 5: pick res-1 raw value 5 (wraps to 5).
        let (res_code, res_len) = codeword_for_symbol(&SV8_RES_1_TABLE, 5).expect("res-1 sym 5");
        assert_eq!(wrap_ref(5), 5);

        // SCFI: single channel ⇒ scfi-1; pick value 2 (schedule: SCF[1]
        // shared, SCF[2] coded).
        let (scfi_code, scfi_len) =
            codeword_for_symbol(&SV8_SCFI_1_TABLE, 2).expect("scfi-1 sym 2");

        // SCF: new_block ⇒ SCF[0] = raw7 − 6; scfi 2 ⇒ SCF[1] = SCF[0],
        // SCF[2] = fold(SCF[0], dscf-1 delta). Pick a dscf-1 delta symbol.
        let dscf1 = table_for_role(Sv8TableRole::Dscf, 0).unwrap();
        let (scf2_code, scf2_len) = codeword_for_symbol(dscf1, 3).expect("dscf-1 sym 3");
        let abs: u32 = 60;

        // Samples: 36 copies of q5 ctx-1's shortest codeword (the grounded
        // context decoder starts in ctx 1). Over-provision: the context
        // may switch tables mid-band, so build a long run of a codeword
        // present in both halves — use ctx-1 row 0; if the context flips
        // to ctx-0 the decode still consumes a valid codeword as long as
        // we feed enough. Simplest: feed q5 ctx-1 row 0 repeatedly and
        // assert only the structural outcome (record shape), not exact
        // sample values.
        let q5_1 = table_for_role(Sv8TableRole::Q5, 1).unwrap();
        let q5_0 = table_for_role(Sv8TableRole::Q5, 0).unwrap();
        let e1 = q5_1.lengths[0];
        let e0 = q5_0.lengths[0];

        let mut p = BitPacker::new();
        p.push(res_code, res_len);
        p.push(scfi_code, scfi_len);
        p.push_raw(abs, 7);
        p.push(scf2_code, scf2_len);
        // Feed 36 sample codewords; alternate ctx-1/ctx-0 shortest rows so
        // whichever the context model selects has a valid codeword waiting.
        // The grounded decoder is deterministic, so just provide a generous
        // run of ctx-1 row 0 (valid) followed by ctx-0 row 0 spares.
        for _ in 0..SAMPLES_PER_BAND {
            p.push(e1.code, e1.length);
        }
        for _ in 0..SAMPLES_PER_BAND {
            p.push(e0.code, e0.length);
        }
        let bytes = p.finish();

        let mut reader = Sv7BitReader::new(&bytes);
        let mut cns = CnsPrng::new();
        let out = decode_sv8_frame_channel(&mut reader, 1, true, &mut cns).unwrap();
        assert_eq!(out.len(), 1);
        let band = &out[0];
        assert_eq!(band.band_type, 5);
        assert_eq!(band.subband, 0);
        // SCF: new-block raw7 − 6 for SCF[0]; SCF[1] shared; SCF[2] folded.
        let scf0 = abs as i32 - 6;
        let scf2 = fold_ref(scf0, 3);
        assert_eq!(band.granule_scf, [scf0, scf0, scf2]);
        // 36 sample levels populated (every q5 symbol is in -7..=7).
        assert!(band.levels.iter().all(|&s| (-7..=7).contains(&s)));
    }

    #[test]
    fn multi_band_threads_prev_scf2_across_coded_bands() {
        // Two coded bands. The second band's SCF[0] non-new-block delta
        // must fold off the first band's SCF[2]. Use band_type 0 samples?
        // No — band_type 0 is empty. Use two band_type-3 grouped2 bands
        // (no context accumulator, simplest sample stream).
        //
        // Resolution sweep is top-down: top band ctx 0, lower band ctx
        // from band-above (>2 ⇒ ctx 1). To get two band_type-3 bands the
        // top decodes to 3 and the lower folds to 3 as well. Drive via
        // explicit res rows replicated from the §6.2 grounded walk.

        // Top band: res-1 raw value 3 ⇒ band_type 3.
        let (top_code, top_len) = codeword_for_symbol(&SV8_RES_1_TABLE, 3).expect("res-1 sym 3");
        // band-above is 3 (>2) ⇒ lower band reads ctx 1 (res-2). We need a
        // res-2 raw delta d such that wrap(3 + d) == 3 ⇒ d == 0.
        let (low_code, low_len) = codeword_for_symbol(&SV8_RES_2_TABLE, 0).expect("res-2 sym 0");

        // Per coded band (ascending order: band 0 then band 1):
        //   SCFI scfi-1 value 3 (SCF[1]=SCF[0], SCF[2]=SCF[1]) ⇒ only SCF[0].
        let (scfi_code, scfi_len) =
            codeword_for_symbol(&SV8_SCFI_1_TABLE, 3).expect("scfi-1 sym 3");
        // Both bands drive the non-new-block dscf-2 SCF[0] path
        // (`new_block = false`), so band 1's SCF[0] folds off band 0's
        // SCF[2] and the prev-SCF2 threading is observable.
        let dscf2 = table_for_role(Sv8TableRole::Dscf, 1).unwrap();
        let (b1_code, b1_len) = codeword_for_symbol(dscf2, 7).expect("dscf-2 sym 7");

        // Samples: band_type 3 ⇒ 18 grouped2 codewords each. q3 shortest row.
        let q3 = table_for_role(Sv8TableRole::Q3, 0).unwrap();
        let q3e = q3.lengths[0];

        let (b0_code, b0_len) = codeword_for_symbol(dscf2, 2).expect("dscf-2 sym 2");
        let mut p = BitPacker::new();
        p.push(top_code, top_len);
        p.push(low_code, low_len);
        // band 0 (lower): SCFI, dscf-2 delta (non-new-block), 18 samples.
        p.push(scfi_code, scfi_len);
        p.push(b0_code, b0_len);
        for _ in 0..18 {
            p.push(q3e.code, q3e.length);
        }
        // band 1 (top): SCFI, dscf-2 delta, 18 samples.
        p.push(scfi_code, scfi_len);
        p.push(b1_code, b1_len);
        for _ in 0..18 {
            p.push(q3e.code, q3e.length);
        }
        let bytes = p.finish();

        let mut reader = Sv7BitReader::new(&bytes);
        let mut cns = CnsPrng::new();
        let out = decode_sv8_frame_channel(&mut reader, 2, false, &mut cns).unwrap();
        assert_eq!(out.len(), 2);
        assert_eq!(out[0].band_type, 3);
        assert_eq!(out[1].band_type, 3);

        // band 0 SCF[0] = fold(prev_scf2=0, delta 2); scfi 3 ⇒ all shared.
        let b0_scf0 = fold_ref(0, 2);
        assert_eq!(out[0].granule_scf, [b0_scf0, b0_scf0, b0_scf0]);
        // band 1 SCF[0] = fold(band0.SCF[2], delta 7).
        let b1_scf0 = fold_ref(b0_scf0, 7);
        assert_eq!(out[1].granule_scf, [b1_scf0, b1_scf0, b1_scf0]);
    }

    #[test]
    fn propagates_eof_in_resolution_phase() {
        // nbands 2 but only a fragment of one res codeword.
        let e = SV8_RES_1_TABLE.lengths[0];
        let mut p = BitPacker::new();
        p.push(e.code, e.length);
        let mut bytes = p.bytes.clone();
        if p.nbits > 0 {
            bytes.push((p.acc << (8 - p.nbits)) as u8);
        }
        // No trailing zero bytes: after band 0 the band-1 res peek starves.
        let mut reader = Sv7BitReader::new(&bytes);
        let mut cns = CnsPrng::new();
        assert_eq!(
            decode_sv8_frame_channel(&mut reader, 2, true, &mut cns),
            Err(Error::UnexpectedEof)
        );
    }

    #[test]
    fn subbands_are_numbered_ascending_from_zero() {
        // Three empty bands: every record is silent, numbered 0, 1, 2.
        let (code, len) = codeword_for_symbol(&SV8_RES_1_TABLE, 0).expect("res-1 sym 0");
        // top band raw 0 ⇒ 0; lower bands ctx 0 (above 0 not >2), delta 0
        // ⇒ stays 0. Feed three res-1 sym-0 codewords.
        let _ = symbol_for_row(&SV8_RES_1_TABLE, 0); // touch helper for parity
        let mut p = BitPacker::new();
        for _ in 0..3 {
            p.push(code, len);
        }
        let bytes = p.finish();
        let mut reader = Sv7BitReader::new(&bytes);
        let mut cns = CnsPrng::new();
        let out = decode_sv8_frame_channel(&mut reader, 3, true, &mut cns).unwrap();
        assert_eq!(out.len(), 3);
        for (i, band) in out.iter().enumerate() {
            assert_eq!(band.subband, i);
            assert_eq!(band.band_type, 0);
            assert!(band.levels.iter().all(|&s| s == 0));
        }
    }
}
