//! SV8 §3.6 / §2.6 frame-decode → reconstructed subband-sample bridge.
//!
//! [`crate::sv8_frame_decode::decode_sv8_frame_channel`] produces one
//! channel's frame body as a sequence of [`Sv8BandDecode`] records — one
//! per coded subband, each carrying its signed `band_type`, the three
//! §6.3 per-granule SCF indices, and the 36 decoded sample levels. This
//! module is the **reconstruction bridge** that turns that structured
//! decode into the per-channel [`SubbandMatrix`] of `f64` subband samples
//! the remaining §2.6 steps (M/S undo, then the synthesis filterbank)
//! consume — the SV8 counterpart of [`crate::frame_reconstruct`] for SV7.
//!
//! # Why a dedicated SV8 path (not [`crate::frame_reconstruct`])
//!
//! The spec (§3, "SV8 reuses the SV7 *signal path* … same band-type →
//! quantiser → scalefactor decode shape") makes the **quantiser** shared:
//! a SV8 `Res` value uses the same [`crate::requant::DEQUANT_COEFFICIENT_C`]
//! /[`crate::requant::QUANTIZER_OFFSET_D`] entry as the SV7 band-type of
//! the same number. Two things differ from the SV7 reconstruction path,
//! so this module exists rather than reusing
//! [`crate::frame_reconstruct::reconstruct_frame_channel`]:
//!
//! 1. **Level centring.** The SV7 PCM-escape arm (`band_type` 8..=17)
//!    emits *unsigned* raw levels that
//!    [`crate::reconstruct::centre_pcm_band`] must centre by subtracting
//!    `D` before dequant. The SV8 sample decode emits **already-signed,
//!    already-centred** levels for *every* arm — the §6.4 large-coefficient
//!    escape (`Res ≥ 9`) carries the sign in its symbol's top 8 bits
//!    ([`crate::sv8_sample_decode`]), so there is no per-arm centring
//!    branch: every coded band dequantises its levels directly.
//! 2. **SCF index range.** The §6.3 DSCF fold
//!    `SCF = ((prev − 25 + delta) & 127) − 6` recenters by `−6`, so a SV8
//!    SCF index is a **signed** value in `−6..=121`, outside the SV7 `u8`
//!    ladder. This path uses the signed SCF gain primitives
//!    ([`crate::reconstruct::apply_granule_scf_relative_signed`]).
//!
//! Bands the frame did not code (subbands past the last [`Sv8BandDecode`])
//! reconstruct to silence — the §6.2 used-band convention (an absent band
//! contributes 36 zero subband samples).
//!
//! # Still GAP downstream
//!
//! - **Absolute SCF anchor gain** — the per-granule SCF multiply here is
//!   *relative* to a caller `anchor`; the absolute reference-index gain is
//!   GAP (see [`crate::reconstruct`]). Relative loudness between granules
//!   and between anchor-sharing bands is exact.
//! - **M/S undo** ([`crate::ms_stereo`]) and the **synthesis filterbank**
//!   remain GAP / out-of-scope-of-`docs/audio/musepack/` respectively.
//!
//! Source-of-record (facts only): `docs/audio/musepack/musepack-sv7-sv8-spec.md`
//! §1 (32-subband geometry), §3 (SV8 reuses the SV7 signal path), §2.6
//! (reconstruction step order); `spec/musepack-headers-and-coding.md` §6.3
//! (signed SCF fold) + §6.4 (signed sample arms). No new format facts are
//! introduced: this is pure composition of the already-grounded SV8
//! sub-walks and the SV7-shared quantiser over the documented geometry.

use crate::cns::CnsPrng;
use crate::frame_reconstruct::{zero_subband_matrix, SubbandMatrix};
use crate::huffman::Sv7BitReader;
use crate::reconstruct::{apply_granule_scf_relative_signed, dequantise_band, dequantise_cns_band};
use crate::sv7_band_decode::SAMPLES_PER_BAND;
use crate::sv7_band_header::SV7_SUBBAND_COUNT;
use crate::sv8_frame_decode::{decode_sv8_frame_channel, Sv8BandDecode};
use crate::{Error, Result};

/// Reconstruct one SV8 band's 36 `f64` subband samples from its decoded
/// levels + signed per-granule SCF indices, into `out`.
///
/// Dispatches on the band's signed `band_type` (`Res`):
///
/// - **`-1`** (CNS / noise): the levels are PRNG-derived; dequantise by
///   the CNS coefficient ([`dequantise_cns_band`]). A CNS band carries no
///   SCF layer, so `granule_scf` is the all-zero placeholder
///   [`decode_sv8_frame_channel`] emits and the SCF multiply (anchor 0,
///   all-zero indices) is the identity — the noise level is set entirely
///   by the dequant constant.
/// - **`0`** (empty): the levels are all zero; dequant of zero is zero —
///   the band reconstructs to silence regardless of SCF.
/// - **`1..=17`** (coded): dequantise the already-centred levels by the
///   `Res` quantiser ([`dequantise_band`]), then apply the three signed
///   per-granule SCF gains relative to `anchor`
///   ([`apply_granule_scf_relative_signed`]).
///
/// # Errors
///
/// [`Error::UnsupportedBandType`] for a `band_type` outside the
/// structurally-enumerated `-1..=17` range.
pub fn reconstruct_sv8_band(
    band: &Sv8BandDecode,
    anchor: i32,
    out: &mut [f64; SAMPLES_PER_BAND],
) -> Result<()> {
    match band.band_type {
        -1 => {
            // CNS: PRNG levels, CNS dequant constant, no SCF layer.
            dequantise_cns_band(&band.levels, out);
            apply_granule_scf_relative_signed(anchor, band.granule_scf, out);
            Ok(())
        }
        0 => {
            // Empty band: silence (dequant of zero is zero).
            out.fill(0.0);
            Ok(())
        }
        1..=17 => {
            dequantise_band(band.band_type, &band.levels, out)?;
            apply_granule_scf_relative_signed(anchor, band.granule_scf, out);
            Ok(())
        }
        other => Err(Error::UnsupportedBandType(other)),
    }
}

/// Reconstruct one channel's full subband-sample matrix from the SV8
/// frame-decode output of every coded band.
///
/// Each [`Sv8BandDecode`] lands in its own `subband` row of an
/// initially-silent [`SubbandMatrix`] via [`reconstruct_sv8_band`].
/// Subbands not present in `bands` (uncoded) keep their zero row.
///
/// `anchor` is the shared signed SCF-ladder reference the per-granule
/// gains are taken relative to (the absolute anchor value is still GAP per
/// §2.6 — see [`crate::reconstruct`]); a fixed anchor makes relative
/// loudness between granules and between bands exact, with the whole
/// channel offset by the single global constant the GAP anchor would
/// supply.
///
/// # Errors
///
/// - [`Error::MaxBandOutOfRange`] if any band names a `subband`
///   `>= SV7_SUBBAND_COUNT`.
/// - [`Error::UnsupportedBandType`] from [`reconstruct_sv8_band`] for a
///   `band_type` outside `-1..=17`.
pub fn reconstruct_sv8_frame_channel(
    bands: &[Sv8BandDecode],
    anchor: i32,
) -> Result<SubbandMatrix> {
    let mut matrix = zero_subband_matrix();
    for band in bands {
        if band.subband >= SV7_SUBBAND_COUNT {
            return Err(Error::MaxBandOutOfRange(band.subband as u8));
        }
        reconstruct_sv8_band(band, anchor, &mut matrix[band.subband])?;
    }
    Ok(matrix)
}

/// End-to-end SV8 single-channel frame decode → reconstructed
/// subband-sample matrix.
///
/// Composes [`decode_sv8_frame_channel`] (the §2.3–§2.6 / §6 frame-body
/// assembler) with [`reconstruct_sv8_frame_channel`] (this module's
/// dequant + per-granule SCF bridge): reads one channel's frame body off
/// `reader` and returns the per-channel [`SubbandMatrix`] ready for the
/// §2.6 M/S undo + synthesis-filterbank steps.
///
/// `nbands` is the §6.2 used-band count, `new_block` the §6.3 per-band
/// new-block flag (set `true` for a key frame, where scalefactors are
/// coded absolutely), and `cns` the shared CNS PRNG advanced by every
/// noise band. `anchor` is the still-GAP absolute SCF reference (see the
/// module docs); a fixed value makes relative loudness exact.
///
/// This is the single integration point from raw frame-body bits to
/// reconstructed subband samples for a mono stream (or one already-
/// resolved channel of a stereo stream); the multi-channel composition +
/// M/S undo follow once their bitstream shape is pinned (GAP — see
/// [`crate::sv8_frame_decode`]).
///
/// # Errors
///
/// Propagates every error of [`decode_sv8_frame_channel`] (band-resolution
/// walk, SCFI / DSCF decode, sample decode, EOF) and
/// [`reconstruct_sv8_frame_channel`] (out-of-range subband / band-type).
pub fn decode_and_reconstruct_sv8_channel(
    reader: &mut Sv7BitReader<'_>,
    nbands: u8,
    new_block: bool,
    cns: &mut CnsPrng,
    anchor: i32,
) -> Result<SubbandMatrix> {
    let bands = decode_sv8_frame_channel(reader, nbands, new_block, cns)?;
    reconstruct_sv8_frame_channel(&bands, anchor)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::reconstruct::{dequantise_band as deq, DEQUANT_DIVISOR};
    use crate::requant::{DEQUANT_COEFFICIENT_C, SCF_STEP_RATIO};
    use crate::scf::SCF_GRANULES_PER_BAND;

    fn band(
        subband: usize,
        band_type: i8,
        granule_scf: [i32; SCF_GRANULES_PER_BAND],
        levels: [i32; SAMPLES_PER_BAND],
    ) -> Sv8BandDecode {
        Sv8BandDecode {
            subband,
            band_type,
            granule_scf,
            levels,
        }
    }

    #[test]
    fn empty_band_list_reconstructs_to_silence() {
        let m = reconstruct_sv8_frame_channel(&[], 0).unwrap();
        assert_eq!(m, zero_subband_matrix());
    }

    #[test]
    fn empty_band_type_is_silent_row() {
        let m = reconstruct_sv8_frame_channel(&[band(5, 0, [0, 0, 0], [123; SAMPLES_PER_BAND])], 0)
            .unwrap();
        // band_type 0 forces silence regardless of (stray) level data.
        assert!(m[5].iter().all(|&s| s == 0.0));
        assert_eq!(m, zero_subband_matrix());
    }

    #[test]
    fn uncoded_subbands_stay_zero() {
        // Only subband 7 is coded; every other row is silent.
        let mut levels = [0i32; SAMPLES_PER_BAND];
        for (i, s) in levels.iter_mut().enumerate() {
            *s = (i as i32 % 7) - 3; // band_type 5 range is wide enough
        }
        let m = reconstruct_sv8_frame_channel(&[band(7, 5, [0, 0, 0], levels)], 0).unwrap();
        for (b, row) in m.iter().enumerate() {
            if b != 7 {
                assert!(row.iter().all(|&s| s == 0.0));
            }
        }
    }

    #[test]
    fn coded_band_dequantises_already_centred_levels_directly() {
        // SV8 levels are already signed/centred: dequant must NOT centre.
        // band_type 5: C = DEQUANT_COEFFICIENT_C[band_type_index(5)=6].
        let levels = [3i32; SAMPLES_PER_BAND];
        let m = reconstruct_sv8_frame_channel(&[band(2, 5, [0, 0, 0], levels)], 0).unwrap();
        // anchor 0, all SCF 0 ⇒ pure dequant, no scaling.
        let mut expected = [0.0; SAMPLES_PER_BAND];
        deq(5, &levels, &mut expected).unwrap();
        assert_eq!(m[2], expected);
        // Spot-check the exact dequant value: 3 * C / 65536.
        let c = DEQUANT_COEFFICIENT_C[6];
        assert!((m[2][0] - 3.0 * c / DEQUANT_DIVISOR).abs() < 1e-9);
    }

    #[test]
    fn signed_negative_scf_indices_scale_each_granule() {
        // SV8 SCF range includes negatives. anchor -6, granules -6,-5,-4.
        let levels = [1i32; SAMPLES_PER_BAND];
        let m = reconstruct_sv8_frame_channel(&[band(0, 3, [-6, -5, -4], levels)], -6).unwrap();
        // band_type 3 dequant base value:
        let c = DEQUANT_COEFFICIENT_C[4]; // index for band_type 3
        let base = c / DEQUANT_DIVISOR;
        // granule 0 (== anchor) ⇒ unscaled; later granules one/two steps down.
        assert!((m[0][0] - base).abs() < 1e-9);
        assert!((m[0][12] - base * SCF_STEP_RATIO).abs() < 1e-9);
        assert!((m[0][24] - base * SCF_STEP_RATIO * SCF_STEP_RATIO).abs() < 1e-9);
        // Higher index = quieter.
        assert!(m[0][0].abs() > m[0][12].abs());
        assert!(m[0][12].abs() > m[0][24].abs());
    }

    #[test]
    fn cns_band_uses_cns_dequant_and_lands_in_its_row() {
        // band_type -1: levels are PRNG-style; CNS dequant constant.
        let levels = [100i32; SAMPLES_PER_BAND];
        let m = reconstruct_sv8_frame_channel(&[band(3, -1, [0, 0, 0], levels)], 0).unwrap();
        // CNS dequant uses DEQUANT_COEFFICIENT_C[0].
        let mut expected = [0.0; SAMPLES_PER_BAND];
        dequantise_cns_band(&levels, &mut expected);
        assert_eq!(m[3], expected);
        // Other rows silent.
        for (b, row) in m.iter().enumerate() {
            if b != 3 {
                assert!(row.iter().all(|&s| s == 0.0));
            }
        }
    }

    #[test]
    fn multiple_bands_land_in_their_own_rows() {
        let l3 = [1i32; SAMPLES_PER_BAND];
        let l4 = [-1i32; SAMPLES_PER_BAND];
        let m = reconstruct_sv8_frame_channel(
            &[band(0, 3, [0, 0, 0], l3), band(31, 4, [0, 0, 0], l4)],
            0,
        )
        .unwrap();
        let mut e0 = [0.0; SAMPLES_PER_BAND];
        deq(3, &l3, &mut e0).unwrap();
        let mut e31 = [0.0; SAMPLES_PER_BAND];
        deq(4, &l4, &mut e31).unwrap();
        assert_eq!(m[0], e0);
        assert_eq!(m[31], e31);
        // Sign is preserved between rows.
        assert!(m[0][0] > 0.0);
        assert!(m[31][0] < 0.0);
    }

    #[test]
    fn rejects_subband_out_of_range() {
        let r = reconstruct_sv8_frame_channel(
            &[band(SV7_SUBBAND_COUNT, 3, [0, 0, 0], [0; SAMPLES_PER_BAND])],
            0,
        );
        assert_eq!(r, Err(Error::MaxBandOutOfRange(SV7_SUBBAND_COUNT as u8)));
    }

    #[test]
    fn rejects_out_of_range_band_type() {
        for bt in [-2i8, 18, i8::MAX, i8::MIN] {
            let r =
                reconstruct_sv8_frame_channel(&[band(1, bt, [0, 0, 0], [0; SAMPLES_PER_BAND])], 0);
            assert!(matches!(r, Err(Error::UnsupportedBandType(_))));
        }
    }

    #[test]
    fn per_band_helper_matches_frame_path() {
        // The single-band helper must agree with the frame-channel walk.
        let levels = [2i32; SAMPLES_PER_BAND];
        let b = band(9, 6, [0, 1, 2], levels);
        let mut row = [0.0; SAMPLES_PER_BAND];
        reconstruct_sv8_band(&b, 0, &mut row).unwrap();
        let m = reconstruct_sv8_frame_channel(&[b], 0).unwrap();
        assert_eq!(m[9], row);
    }

    #[test]
    fn end_to_end_zero_bands_is_silent_matrix() {
        // nbands 0 ⇒ no coded bands ⇒ silent matrix straight from bits.
        let mut reader = Sv7BitReader::new(&[0xFF; 4]);
        let mut cns = CnsPrng::new();
        let m = decode_and_reconstruct_sv8_channel(&mut reader, 0, true, &mut cns, 0).unwrap();
        assert_eq!(m, zero_subband_matrix());
    }

    #[test]
    fn end_to_end_matches_two_step_path() {
        // The end-to-end helper must equal decode-then-reconstruct run
        // separately on the same bits + PRNG state. Use an all-empty
        // three-band frame (deterministic from a simple bit stream): the
        // res-1 symbol-0 codeword decodes to band_type 0 for each band.
        use crate::sv8_huffman::SV8_RES_1_TABLE;

        // Build three res-1 symbol-0 codewords (band_type 0). Reuse the
        // table's shortest row that maps to symbol 0 by scanning rows.
        let (code, len) = {
            // Find a codeword whose decode is symbol 0.
            let mut found = None;
            let mut upper: u32 = 0x1_0000;
            'outer: for e in SV8_RES_1_TABLE.lengths.iter() {
                if e.length == 0 {
                    continue;
                }
                let step = 1u32 << (16 - e.length as u32);
                let mut pat = e.code as u32;
                while pat < upper {
                    let mut bytes = Vec::new();
                    let mut acc = 0u32;
                    let mut nbits = 0u8;
                    for i in 0..e.length {
                        let bit = (pat >> (15 - i as u32)) & 1;
                        acc = (acc << 1) | bit;
                        nbits += 1;
                        if nbits == 8 {
                            bytes.push(acc as u8);
                            acc = 0;
                            nbits = 0;
                        }
                    }
                    if nbits > 0 {
                        bytes.push((acc << (8 - nbits)) as u8);
                    }
                    bytes.push(0);
                    bytes.push(0);
                    let mut r = Sv7BitReader::new(&bytes);
                    if SV8_RES_1_TABLE.decode(&mut r).unwrap() == 0 {
                        found = Some((pat as u16, e.length));
                        break 'outer;
                    }
                    pat += step;
                }
                upper = e.code as u32;
            }
            found.expect("res-1 has a symbol-0 codeword")
        };

        // Pack three of them (res sweep is top-down; symbol-0 stays 0).
        let mut bytes = Vec::new();
        let mut acc = 0u32;
        let mut nbits = 0u8;
        for _ in 0..3 {
            for i in 0..len {
                let bit = (code >> (15 - i)) & 1;
                acc = (acc << 1) | bit as u32;
                nbits += 1;
                if nbits == 8 {
                    bytes.push(acc as u8);
                    acc = 0;
                    nbits = 0;
                }
            }
        }
        if nbits > 0 {
            bytes.push((acc << (8 - nbits)) as u8);
        }
        bytes.push(0);
        bytes.push(0);

        let mut r1 = Sv7BitReader::new(&bytes);
        let mut cns1 = CnsPrng::new();
        let end_to_end =
            decode_and_reconstruct_sv8_channel(&mut r1, 3, true, &mut cns1, 0).unwrap();

        let mut r2 = Sv7BitReader::new(&bytes);
        let mut cns2 = CnsPrng::new();
        let bands = decode_sv8_frame_channel(&mut r2, 3, true, &mut cns2).unwrap();
        let two_step = reconstruct_sv8_frame_channel(&bands, 0).unwrap();

        assert_eq!(end_to_end, two_step);
        // Three empty bands ⇒ still a silent matrix.
        assert_eq!(end_to_end, zero_subband_matrix());
    }
}
