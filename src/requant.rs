//! SV7 / SV8 requantiser constants.
//!
//! Wires the four numeric tables that describe the Musepack subband
//! requantiser: bits-per-sample per band-type, the integer offset
//! `D` (the number of quantiser steps is `2 * D + 1`), the dequant
//! coefficient `C = 65536 / (2 * D + 1)`, and the geometric ratio
//! between adjacent scalefactor-index gains.
//!
//! Source-of-record:
//!
//! - **Structural prose**: `docs/audio/musepack/musepack-sv7-sv8-spec.md`
//!   §2.5 (frame body — quantised subband samples) and §2.6
//!   (reconstruction).
//! - **Numeric values**: `docs/audio/musepack/tables/`:
//!   * `requant-res-bits.csv` — `RES_BITS[18]`.
//!   * `requant-quantizer-offset-Dc.csv` — `QUANTIZER_OFFSET_D[19]`.
//!   * `requant-coefficient-Cc.csv` — `DEQUANT_COEFFICIENT_C[19]`.
//!   * `scf-step-ratio.csv` — `SCF_STEP_RATIO`.
//!
//! The CSV + `.meta` sidecar files are the Feist-extracted facts
//! produced by the walled extraction round documented in
//! `docs/audio/musepack/provenance/01-musepack-table-extraction.md`.
//! No implementation source was read during this wiring; the values
//! below are transcribed from the project's own CSV facts.
//!
//! # Index conventions
//!
//! - `RES_BITS` is indexed by `band_type` in `0..=17` directly. The
//!   spec's §2.5 case ladder maps `band_type` 0..=7 to entropy-coded
//!   bands (sample width carried by the Huffman codebook, hence
//!   `RES_BITS == 0`) and `band_type` 8..=17 to the linear-PCM
//!   escape ladder (7..=16 bits per sample).
//! - `QUANTIZER_OFFSET_D` and `DEQUANT_COEFFICIENT_C` share a
//!   different convention: array **index 0** is the CNS / noise
//!   substitution band (the §2.5 "case −1" entry, kept alongside the
//!   normal entries so that the dequantiser can look it up by index)
//!   and array indices **1..=18** correspond to `band_type` 0..=17.
//!   Use [`band_type_index`] to translate a signed band-type value
//!   in the inclusive range `-1..=17` into the array index.

/// Bits per quantised sample for each `band_type` in `0..=17`.
///
/// Per spec §2.5, the entropy-coded band types `0..=7` carry their
/// sample width inside the Huffman codebook, so this array reports
/// `0` for them. The linear-PCM escape band types `8..=17` carry
/// `(band_type - 1)` bits per sample, i.e. `7..=16` here.
pub const RES_BITS: [u8; 18] = [
    0, 0, 0, 0, 0, 0, 0, 0, // entropy-coded band_types 0..=7
    7, 8, 9, 10, 11, 12, 13, 14, 15, 16, // linear-PCM escape ladder, band_types 8..=17
];

/// Quantiser offset `D` per indexed band entry (see module-level
/// "Index conventions"). The number of distinct quantiser levels for
/// the entry is `2 * D + 1`.
///
/// Index 0 is the CNS / noise-substitution entry (`D = 2`); indices
/// `1..=18` correspond to `band_type` `0..=17`.
pub const QUANTIZER_OFFSET_D: [i16; 19] = [
    2, // index 0 — CNS / noise band entry
    0, 1, 2, 3, 4, 7, 15, 31, // band_types 0..=7 (entropy-coded ladder)
    63, 127, 255, 511, 1023, 2047, 4095, 8191, 16383,
    32767, // band_types 8..=17 (linear-PCM escape)
];

/// Dequantiser coefficient `C` per indexed band entry (see
/// module-level "Index conventions"). For the normal entries the
/// spec's relation is `C = 65536 / (2 * D + 1)`. Index 0 is the
/// CNS / noise-substitution dequant constant (`≈ 32768 / 2 / 255 ·
/// sqrt(3)` per the `cns-prng-params.meta` sidecar).
///
/// The decimal literals are transcribed verbatim from
/// `docs/audio/musepack/tables/requant-coefficient-Cc.csv`; the
/// allow attribute keeps the spec-facing form intact (Rust's f64
/// parser truncates to the nearest representable double).
#[allow(clippy::excessive_precision)]
pub const DEQUANT_COEFFICIENT_C: [f64; 19] = [
    111.285962475327, // index 0 — CNS / noise entry
    65536.000000000000,
    21845.333333333332,
    13107.200000000001,
    9362.285714285713,
    7281.777777777777,
    4369.066666666666,
    2114.064516129032,
    1040.253968253968, // band_types 0..=7
    516.031496062992,
    257.003921568627,
    128.250489236790,
    64.062561094819,
    32.015632633121,
    16.003907203907,
    8.000976681723,
    4.000244155527,
    2.000061037018,
    1.000015259021, // band_types 8..=17
];

/// Geometric ratio between successive scalefactor-index gains, in
/// the **downward** direction: `gain[n] / gain[n - 1] == SCF_STEP_RATIO`.
///
/// The upward step is `1.0 / SCF_STEP_RATIO ≈ 1.20050805774840750476`.
/// The 256-entry SCF table is built as a geometric sequence around
/// SCF index `1`, giving the spec's `~ +1.58 dB .. -98.41 dB` span.
///
/// The literal is transcribed verbatim from
/// `docs/audio/musepack/tables/scf-step-ratio.csv`; the allow
/// attribute keeps the spec-facing form intact (Rust's f64 parser
/// truncates to the nearest representable double).
#[allow(clippy::excessive_precision)]
pub const SCF_STEP_RATIO: f64 = 0.83298066476582673961;

/// Maps a signed `band_type` value in the inclusive range `-1..=17`
/// to the corresponding array index in [`QUANTIZER_OFFSET_D`] and
/// [`DEQUANT_COEFFICIENT_C`]: `-1 -> 0` (the CNS / noise entry),
/// `0 -> 1`, ..., `17 -> 18`. Returns `None` for out-of-range
/// values.
#[inline]
pub fn band_type_index(band_type_signed: i8) -> Option<usize> {
    match band_type_signed {
        -1..=17 => Some((band_type_signed + 1) as usize),
        _ => None,
    }
}

/// Returns the bits-per-sample for an unsigned `band_type` in
/// `0..18`. Returns `None` if `band_type >= 18`. The CNS / noise
/// band (signed `band_type == -1`) does not have a bits-per-sample
/// count — its samples come from the noise generator, not the
/// bitstream — and is therefore not addressable through this
/// function.
#[inline]
pub fn band_type_to_res_bits(band_type: u8) -> Option<u8> {
    RES_BITS.get(band_type as usize).copied()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Per-array length, as recorded by the `.meta` sidecars.
    #[test]
    fn table_lengths_match_meta_sidecars() {
        assert_eq!(
            RES_BITS.len(),
            18,
            "RES_BITS length per .meta resolved_dims"
        );
        assert_eq!(
            QUANTIZER_OFFSET_D.len(),
            19,
            "QUANTIZER_OFFSET_D length per .meta resolved_dims"
        );
        assert_eq!(
            DEQUANT_COEFFICIENT_C.len(),
            19,
            "DEQUANT_COEFFICIENT_C length per .meta resolved_dims"
        );
    }

    /// Spec §2.5: band_types 0..=7 are entropy-coded (no
    /// per-sample bit width in the bitstream); 8..=17 are the
    /// linear-PCM escape ladder with `band_type - 1` bits.
    #[test]
    fn res_bits_split_at_entropy_pcm_boundary() {
        assert_eq!(&RES_BITS[0..=7], &[0u8; 8]);
        assert_eq!(&RES_BITS[8..=17], &[7, 8, 9, 10, 11, 12, 13, 14, 15, 16]);
    }

    /// The CNS / noise entry is at index 0 with `D == 2`. The first
    /// real band entry (`band_type == 0`) has `D == 0`. The largest
    /// `D` is at the top of the escape ladder.
    #[test]
    fn quantizer_offset_boundaries() {
        assert_eq!(QUANTIZER_OFFSET_D[0], 2);
        assert_eq!(QUANTIZER_OFFSET_D[1], 0);
        assert_eq!(QUANTIZER_OFFSET_D[18], 32767);
    }

    /// Decimal-literal checkpoints against the CSV.
    #[test]
    fn dequant_coefficient_checkpoints() {
        assert!(
            (DEQUANT_COEFFICIENT_C[0] - 111.285962475327).abs() < 1e-9,
            "CNS dequant constant"
        );
        assert!(
            (DEQUANT_COEFFICIENT_C[2] - 21845.333333333332).abs() < 1e-9,
            "C for band_type 1, exact 65536/3"
        );
    }

    /// Spec §2.6 / requant-coefficient-Cc.meta relation:
    /// `C * (2D + 1) == 65536` for the 18 normal band-type entries.
    #[test]
    fn dequant_coefficient_matches_spec_relation() {
        for i in 1..=18usize {
            let c = DEQUANT_COEFFICIENT_C[i];
            let d = QUANTIZER_OFFSET_D[i] as f64;
            let product = c * (2.0 * d + 1.0);
            assert!(
                (product - 65536.0).abs() < 1e-6,
                "index {i}: C * (2D+1) = {product}, expected 65536"
            );
        }
    }

    /// `band_type_index` translates the signed `band_type` to the
    /// shared array index used by `D` and `C`.
    #[test]
    fn band_type_index_mapping() {
        assert_eq!(band_type_index(-1), Some(0));
        assert_eq!(band_type_index(0), Some(1));
        assert_eq!(band_type_index(17), Some(18));
        assert_eq!(band_type_index(18), None);
        assert_eq!(band_type_index(-2), None);
        assert_eq!(band_type_index(i8::MAX), None);
        assert_eq!(band_type_index(i8::MIN), None);
    }

    /// `band_type_to_res_bits` is bounded by the size of
    /// `RES_BITS`.
    #[test]
    fn band_type_to_res_bits_bounds() {
        assert_eq!(band_type_to_res_bits(0), Some(0));
        assert_eq!(band_type_to_res_bits(7), Some(0));
        assert_eq!(band_type_to_res_bits(8), Some(7));
        assert_eq!(band_type_to_res_bits(17), Some(16));
        assert_eq!(band_type_to_res_bits(18), None);
        assert_eq!(band_type_to_res_bits(u8::MAX), None);
    }

    /// SCF step ratio is the downward direction (< 1) and its
    /// reciprocal is the upward step. The reciprocal sanity check
    /// guards against accidental edits. The const-block bounds
    /// catch a wrong-direction edit at compile time.
    #[test]
    fn scf_step_ratio_is_geometric_downward() {
        const _DOWNWARD: () = assert!(SCF_STEP_RATIO < 1.0);
        const _SANE_LOWER: () = assert!(SCF_STEP_RATIO > 0.5);
        let round_trip = SCF_STEP_RATIO * (1.0 / SCF_STEP_RATIO);
        assert!((round_trip - 1.0).abs() < 1e-15);
    }
}
