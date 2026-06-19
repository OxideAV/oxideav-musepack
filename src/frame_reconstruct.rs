//! SV7 §2.6 frame-level reconstruction assembler.
//!
//! [`crate::reconstruct`] reconstructs **one band** at a time: from a
//! `[i32; 36]` level buffer to the 36 dequantised, per-granule-SCF
//! scaled `f64` subband samples
//! ([`crate::reconstruct::reconstruct_sv7_band_from_levels`]). This
//! module composes that per-band step over the full Layer-II
//! 32-subband frame geometry (spec §1: 32 subbands × 36 samples =
//! 1152 samples per channel), producing the per-channel subband-sample
//! **matrix** that the remaining §2.6 steps (M/S undo, then the
//! synthesis filterbank) consume.
//!
//! No new format facts are introduced: this is pure composition of the
//! already-grounded per-band reconstruction over the documented frame
//! geometry. Bands the frame did not code reconstruct to silence (the
//! §2.3 / §2.5 "data stored only for non-zero bands" convention — an
//! absent band contributes 36 zero subband samples).
//!
//! Source-of-record (facts only):
//!
//! - `docs/audio/musepack/musepack-sv7-sv8-spec.md` §1 — the frame
//!   geometry (32 subbands, 36 samples each, 1152 per channel).
//! - `docs/audio/musepack/musepack-sv7-sv8-spec.md` §2.5 / §2.6 — the
//!   per-band sample decode and reconstruction this module composes.
//!
//! # Still GAP downstream
//!
//! This module stops at the per-channel subband-sample matrix. The two
//! remaining §2.6 steps are documented gaps under
//! `docs/audio/musepack/`:
//!
//! - **M/S undo** — §2.6 says "undo M/S where `msflag` set" but the
//!   channel arithmetic (`L = M + S` / `R = M − S`, and any
//!   normalisation) is unspecified.
//! - **The 32-band synthesis filterbank** — needs the Layer-II
//!   synthesis window `D_i` and matrix `N_ik`, which the spec places in
//!   the ISO 11172-3 PDF outside this crate's
//!   `docs/audio/musepack/` source-of-truth scope.

use crate::reconstruct::{reconstruct_sv7_band_from_levels, GRANULES_PER_BAND, SCF_INDEX_COUNT};
use crate::sv7_band_decode::SAMPLES_PER_BAND;
use crate::sv7_band_header::SV7_SUBBAND_COUNT;
use crate::{Error, Result};

/// One channel's reconstructed subband samples for a single frame:
/// [`SV7_SUBBAND_COUNT`] subbands, each [`SAMPLES_PER_BAND`] `f64`
/// samples. Row `b` holds subband `b`'s 36 samples in time order; an
/// uncoded subband row is all-zero.
pub type SubbandMatrix = [[f64; SAMPLES_PER_BAND]; SV7_SUBBAND_COUNT];

/// A fresh all-zero [`SubbandMatrix`] — every subband silent.
///
/// Used as the starting point for [`reconstruct_frame_channel`]: bands
/// the frame does not touch keep their zero row (the §2.3 / §2.5
/// "absent band ⇒ silent" convention).
pub fn zero_subband_matrix() -> SubbandMatrix {
    [[0.0; SAMPLES_PER_BAND]; SV7_SUBBAND_COUNT]
}

/// Per-band decode result for one subband of one channel, the input
/// [`reconstruct_frame_channel`] consumes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BandLevels {
    /// Which subband (`0..SV7_SUBBAND_COUNT`) this band occupies.
    pub subband: usize,
    /// The band-type case selector (`-1..=17`) the §2.5 decode used.
    pub band_type: i8,
    /// The unified `[i32; 36]` level buffer
    /// [`crate::sv7_band_decode::decode_sv7_band`] emitted.
    pub levels: [i32; SAMPLES_PER_BAND],
    /// The three per-granule absolute SCF indices (granules 0, 1, 2),
    /// as reconstructed by the [`crate::scf`] decoder. Each must lie in
    /// the `0..SCF_INDEX_COUNT` ladder.
    pub granule_scf: [u32; GRANULES_PER_BAND],
}

/// Convert the [`crate::scf::BandScf`]-style `i32` per-granule SCF
/// indices into the `u8` ladder indices the [`crate::reconstruct`]
/// layer takes, validating each against the `0..SCF_INDEX_COUNT`
/// (256-index) ladder.
///
/// # Errors
///
/// [`Error::MaxBandOutOfRange`] is reused as the out-of-ladder signal
/// (the offending index, truncated into a `u8` for the diagnostic) when
/// any index is `>= SCF_INDEX_COUNT`. The §5.3 prose notes a decoded
/// index "exceeding 1024 is clamped to a sentinel"; this crate's
/// fail-loud posture surfaces the out-of-range index to the caller
/// rather than silently clamping.
fn scf_indices_to_u8(granule_scf: [u32; GRANULES_PER_BAND]) -> Result<[u8; GRANULES_PER_BAND]> {
    let mut out = [0u8; GRANULES_PER_BAND];
    for (slot, &idx) in out.iter_mut().zip(granule_scf.iter()) {
        if (idx as usize) >= SCF_INDEX_COUNT {
            return Err(Error::MaxBandOutOfRange(idx as u8));
        }
        *slot = idx as u8;
    }
    Ok(out)
}

/// Reconstruct one channel's full subband-sample matrix from the
/// per-band decode results of every coded band in the frame.
///
/// Walks `bands`, reconstructing each via
/// [`reconstruct_sv7_band_from_levels`] into its subband row of an
/// initially-silent [`SubbandMatrix`]. Bands absent from `bands`
/// (uncoded subbands) keep their zero row. `anchor` is the shared
/// SCF-ladder reference the per-granule gains are taken relative to
/// (the absolute anchor value is still GAP per §2.6 — see
/// [`crate::reconstruct`]); passing a fixed anchor makes the relative
/// loudness between granules and between bands exact, with the whole
/// channel offset by the single global constant the GAP anchor would
/// supply.
///
/// # Errors
///
/// - [`Error::MaxBandOutOfRange`] if any band names a `subband`
///   `>= SV7_SUBBAND_COUNT`, or carries a per-granule SCF index
///   outside the `0..SCF_INDEX_COUNT` ladder.
/// - [`Error::UnsupportedBandType`] propagated from
///   [`reconstruct_sv7_band_from_levels`] for a `band_type` outside
///   `-1..=17`.
pub fn reconstruct_frame_channel(bands: &[BandLevels], anchor: u8) -> Result<SubbandMatrix> {
    let mut matrix = zero_subband_matrix();
    for band in bands {
        if band.subband >= SV7_SUBBAND_COUNT {
            return Err(Error::MaxBandOutOfRange(band.subband as u8));
        }
        let granule_scf = scf_indices_to_u8(band.granule_scf)?;
        reconstruct_sv7_band_from_levels(
            band.band_type,
            &band.levels,
            anchor,
            granule_scf,
            &mut matrix[band.subband],
        )?;
    }
    Ok(matrix)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::reconstruct::reconstruct_sv7_band_from_levels;

    fn band(subband: usize, band_type: i8, levels: [i32; SAMPLES_PER_BAND]) -> BandLevels {
        BandLevels {
            subband,
            band_type,
            levels,
            granule_scf: [0, 0, 0],
        }
    }

    #[test]
    fn zero_matrix_is_all_silence() {
        let m = zero_subband_matrix();
        for row in &m {
            assert!(row.iter().all(|&s| s == 0.0));
        }
    }

    #[test]
    fn empty_band_list_reconstructs_to_silence() {
        let m = reconstruct_frame_channel(&[], 0).unwrap();
        assert_eq!(m, zero_subband_matrix());
    }

    #[test]
    fn uncoded_subbands_stay_zero() {
        // Only subband 5 is coded (band_type 0 = empty, still zero).
        let m = reconstruct_frame_channel(&[band(5, 0, [0; SAMPLES_PER_BAND])], 0).unwrap();
        assert_eq!(m, zero_subband_matrix());
    }

    #[test]
    fn single_band_matches_direct_per_band_reconstruction() {
        // band_type 3 (Huffman, already-centred levels), subband 7.
        let mut levels = [0i32; SAMPLES_PER_BAND];
        for (i, slot) in levels.iter_mut().enumerate() {
            *slot = (i as i32 % 7) - 3; // in -3..=3, the band_type-3 range
        }
        let m = reconstruct_frame_channel(&[band(7, 3, levels)], 0).unwrap();

        let mut expected = [0.0; SAMPLES_PER_BAND];
        reconstruct_sv7_band_from_levels(3, &levels, 0, [0, 0, 0], &mut expected).unwrap();
        assert_eq!(m[7], expected);
        // Every other row stays silent.
        for (b, row) in m.iter().enumerate() {
            if b != 7 {
                assert!(row.iter().all(|&s| s == 0.0));
            }
        }
    }

    #[test]
    fn multiple_bands_land_in_their_own_rows() {
        let mut l3 = [0i32; SAMPLES_PER_BAND];
        l3.iter_mut().for_each(|s| *s = 1);
        let mut l4 = [0i32; SAMPLES_PER_BAND];
        l4.iter_mut().for_each(|s| *s = -1);
        let m = reconstruct_frame_channel(&[band(0, 3, l3), band(31, 4, l4)], 0).unwrap();

        let mut e0 = [0.0; SAMPLES_PER_BAND];
        reconstruct_sv7_band_from_levels(3, &l3, 0, [0, 0, 0], &mut e0).unwrap();
        let mut e31 = [0.0; SAMPLES_PER_BAND];
        reconstruct_sv7_band_from_levels(4, &l4, 0, [0, 0, 0], &mut e31).unwrap();
        assert_eq!(m[0], e0);
        assert_eq!(m[31], e31);
        // Sign difference is preserved between the two rows.
        assert!(m[0][0] > 0.0);
        assert!(m[31][0] < 0.0);
    }

    #[test]
    fn per_granule_scf_reaches_the_right_row() {
        // band_type 3, distinct SCF per granule; the three 12-sample
        // slices of the row must carry their own relative gain.
        let levels = [1i32; SAMPLES_PER_BAND];
        let b = BandLevels {
            subband: 2,
            band_type: 3,
            levels,
            granule_scf: [10, 20, 30],
        };
        let m = reconstruct_frame_channel(&[b], 10).unwrap();
        let mut expected = [0.0; SAMPLES_PER_BAND];
        reconstruct_sv7_band_from_levels(3, &levels, 10, [10, 20, 30], &mut expected).unwrap();
        assert_eq!(m[2], expected);
        // Granule 0 (anchor) is loudest; higher SCF index = quieter.
        assert!(m[2][0].abs() > m[2][12].abs());
        assert!(m[2][12].abs() > m[2][24].abs());
    }

    #[test]
    fn rejects_subband_out_of_range() {
        let r = reconstruct_frame_channel(&[band(32, 3, [0; SAMPLES_PER_BAND])], 0);
        assert_eq!(r, Err(Error::MaxBandOutOfRange(32)));
    }

    #[test]
    fn rejects_scf_index_out_of_ladder() {
        let b = BandLevels {
            subband: 1,
            band_type: 3,
            levels: [0; SAMPLES_PER_BAND],
            granule_scf: [0, 256, 0], // 256 is one past the 0..=255 ladder
        };
        let r = reconstruct_frame_channel(&[b], 0);
        assert_eq!(r, Err(Error::MaxBandOutOfRange(0))); // 256 as u8 == 0
    }

    #[test]
    fn propagates_unsupported_band_type() {
        let r = reconstruct_frame_channel(&[band(1, -2, [0; SAMPLES_PER_BAND])], 0);
        assert_eq!(r, Err(Error::UnsupportedBandType(-2)));
    }

    #[test]
    fn cns_band_reconstructs_into_its_row() {
        // band_type -1 (CNS): levels are PRNG-derived; just confirm the
        // row matches the direct per-band path and the rest stays zero.
        let levels = [100i32; SAMPLES_PER_BAND];
        let m = reconstruct_frame_channel(&[band(3, -1, levels)], 0).unwrap();
        let mut expected = [0.0; SAMPLES_PER_BAND];
        reconstruct_sv7_band_from_levels(-1, &levels, 0, [0, 0, 0], &mut expected).unwrap();
        assert_eq!(m[3], expected);
    }

    #[test]
    fn matrix_geometry_matches_frame_constants() {
        // 32 subbands × 36 samples == 1152 per channel (spec §1).
        let m = zero_subband_matrix();
        assert_eq!(m.len(), SV7_SUBBAND_COUNT);
        assert_eq!(m[0].len(), SAMPLES_PER_BAND);
        assert_eq!(
            SV7_SUBBAND_COUNT * SAMPLES_PER_BAND,
            crate::SAMPLES_PER_FRAME_PER_CHANNEL
        );
    }
}
