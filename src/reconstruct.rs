//! SV7 §2.6 sample reconstruction primitives.
//!
//! Wires the per-sample dequantisation step that follows the per-band
//! level decode of [`crate::sv7_band_decode`]. The structural spec
//! §2.6 (`docs/audio/musepack/musepack-sv7-sv8-spec.md`) describes the
//! reconstruction as: "requantise each sample by the quantiser implied
//! by its `band_type`, multiply by the band/granule scalefactor [...],
//! undo M/S where `msflag` set, then run the inherited 32-band
//! synthesis subband filter". The constants for the requantise step
//! are already wired in [`crate::requant`]:
//!
//! - `QUANTIZER_OFFSET_D[i]` — integer offset `D` per indexed band
//!   entry (number of quantiser steps = `2 * D + 1`).
//! - `DEQUANT_COEFFICIENT_C[i]` — dequant coefficient
//!   `C = 65536 / (2 * D + 1)`.
//!
//! This module covers **only** the per-sample dequantise step (the
//! product `centred_level * C / 65536`). The downstream SCF multiply,
//! M/S undo, and synthesis filterbank are not in scope for this
//! round.
//!
//! # Centring convention
//!
//! Two cases come out of the §2.5 per-band sample decode:
//!
//! - **Huffman path** (band_types 3..=7): the staged Q3..Q7 tables
//!   already produce signed `i8` levels in `-D..=D` (e.g. band_type 3
//!   has `D = 3` so values are `-3..=3`; band_type 7 has `D = 31` so
//!   values are `-31..=31`). These levels are *already centred*;
//!   no further centring is needed before the dequant multiply.
//! - **Linear-PCM escape path** (band_types 8..=17): the raw level
//!   read off the bitstream is *unsigned* in `0..=2*D`. The §2.5
//!   prose specifies the "linear quantiser" produces a centred level
//!   in `-D..=D`; the centring step is therefore the subtraction
//!   `centred = raw_unsigned - D`. This module exposes the centring
//!   step as a separate function so the PCM-escape decoder can
//!   convert its `[i32; 36]` raw-level buffer into a centred
//!   `[i32; 36]` buffer before dequantising.
//!
//! The CNS / noise-substitution path (band_type == -1) is handled
//! separately by [`crate::cns`] — those samples come out of the PRNG
//! already in `-510..=510` and use the CNS dequant constant at
//! `DEQUANT_COEFFICIENT_C[0]`.
//!
//! # Where the SCF multiply lives
//!
//! §2.6's structural step is `sample * C * scf_gain`, but the
//! 256-entry scalefactor-index → gain table needs an anchor point
//! (the gain at the reference index) that the structural prose
//! does not pin down — only the geometric step ratio
//! [`crate::requant::SCF_STEP_RATIO`] between adjacent indices is
//! independently specified. The SCF table construction is therefore
//! deferred to a later round; this module's output is the
//! pre-SCF-multiply dequantised sample.

use crate::requant::{band_type_index, DEQUANT_COEFFICIENT_C, QUANTIZER_OFFSET_D};
use crate::sv7_band_decode::SAMPLES_PER_BAND;
use crate::{Error, Result};

/// Divisor in the §2.6 dequant relation `sample = centred_level * C / 65536`.
///
/// Tied to the requantiser table relation `C = 65536 / (2 * D + 1)`
/// (see [`crate::requant::DEQUANT_COEFFICIENT_C`]). Stored as `f64`
/// because the dequant arithmetic is floating point.
pub const DEQUANT_DIVISOR: f64 = 65536.0;

/// Centre a single PCM-escape raw level by subtracting `D` for the
/// given `band_type`. Returns [`Error::UnsupportedBandType`] if
/// `band_type` is outside the linear-PCM escape range `8..=17`.
///
/// The escape ladder packs `band_type - 1` unsigned bits per sample
/// into the raw level; the centred result is the signed value in the
/// inclusive range `-D..=D` per the §2.5 / §2.6 "linear quantiser"
/// description.
#[inline]
pub fn centre_pcm_level(band_type: i8, raw_unsigned: i32) -> Result<i32> {
    if !(8..=17).contains(&band_type) {
        return Err(Error::UnsupportedBandType(band_type));
    }
    // band_type in 8..=17 -> index in 9..=18.
    let idx = band_type_index(band_type).ok_or(Error::UnsupportedBandType(band_type))?;
    let d = QUANTIZER_OFFSET_D[idx] as i32;
    Ok(raw_unsigned - d)
}

/// Centre an entire 36-sample PCM-escape band in place.
///
/// Subtracts `D = QUANTIZER_OFFSET_D[band_type + 1]` from every
/// sample of `buf`. Returns [`Error::UnsupportedBandType`] for a
/// `band_type` outside `8..=17`.
///
/// The result satisfies `buf[i] ∈ -D..=D` whenever the input was a
/// valid `(band_type - 1)`-bit unsigned raw level (i.e. in
/// `0..=2D`). Inputs outside that range are not rejected here — the
/// PCM-escape reader bounds the raw level structurally, and bounds
/// checking it again would make the function panicky on legitimate
/// CNS-style "wider range" callers.
pub fn centre_pcm_band(band_type: i8, buf: &mut [i32; SAMPLES_PER_BAND]) -> Result<()> {
    if !(8..=17).contains(&band_type) {
        return Err(Error::UnsupportedBandType(band_type));
    }
    let idx = band_type_index(band_type).ok_or(Error::UnsupportedBandType(band_type))?;
    let d = QUANTIZER_OFFSET_D[idx] as i32;
    for slot in buf.iter_mut() {
        *slot -= d;
    }
    Ok(())
}

/// Dequantise a single already-centred sample level for a given
/// `band_type` in the normal entropy/PCM range `0..=17`.
///
/// Returns `centred_level * C / 65536` where
/// `C = DEQUANT_COEFFICIENT_C[band_type + 1]`. The CNS / noise
/// band (signed `band_type == -1`) has its own dequant path keyed off
/// `DEQUANT_COEFFICIENT_C[0]`; pass `band_type == -1` to use it.
///
/// Returns [`Error::UnsupportedBandType`] for `band_type` outside
/// `-1..=17`.
#[inline]
pub fn dequantise_sample(band_type: i8, centred_level: i32) -> Result<f64> {
    let idx = band_type_index(band_type).ok_or(Error::UnsupportedBandType(band_type))?;
    let c = DEQUANT_COEFFICIENT_C[idx];
    Ok(centred_level as f64 * c / DEQUANT_DIVISOR)
}

/// Dequantise a 36-sample band of already-centred levels (Huffman
/// path: `band_type` in `3..=7`; PCM-escape path: caller first runs
/// [`centre_pcm_band`] then this) into `out`.
///
/// Returns [`Error::UnsupportedBandType`] for a `band_type` outside
/// the structurally-documented `0..=17` quantiser-bearing range.
/// Use [`dequantise_cns_band`] for the CNS / noise band
/// (`band_type == -1`).
pub fn dequantise_band(
    band_type: i8,
    centred: &[i32; SAMPLES_PER_BAND],
    out: &mut [f64; SAMPLES_PER_BAND],
) -> Result<()> {
    if !(0..=17).contains(&band_type) {
        return Err(Error::UnsupportedBandType(band_type));
    }
    let idx = band_type_index(band_type).ok_or(Error::UnsupportedBandType(band_type))?;
    let c = DEQUANT_COEFFICIENT_C[idx];
    for (dst, &src) in out.iter_mut().zip(centred.iter()) {
        *dst = src as f64 * c / DEQUANT_DIVISOR;
    }
    Ok(())
}

/// Dequantise a 36-sample band of Huffman-coded levels (`band_type`
/// 3..=7). Convenience wrapper over [`dequantise_band`] that
/// accepts the `[i8; 36]` shape returned by
/// [`crate::sv7_band_decode::decode_huffman_band`] — the Q3..Q7
/// tables produce signed `i8` levels that are already centred.
pub fn dequantise_huffman_band(
    band_type: i8,
    huffman_levels: &[i8; SAMPLES_PER_BAND],
    out: &mut [f64; SAMPLES_PER_BAND],
) -> Result<()> {
    if !(3..=7).contains(&band_type) {
        return Err(Error::UnsupportedBandType(band_type));
    }
    let idx = band_type_index(band_type).ok_or(Error::UnsupportedBandType(band_type))?;
    let c = DEQUANT_COEFFICIENT_C[idx];
    for (dst, &src) in out.iter_mut().zip(huffman_levels.iter()) {
        *dst = src as f64 * c / DEQUANT_DIVISOR;
    }
    Ok(())
}

/// Dequantise a 36-sample CNS / noise band (`band_type == -1`).
///
/// The CNS PRNG (see [`crate::cns::CnsPrng`]) emits samples in
/// `-510..=510`; this multiplies them by the CNS dequant coefficient
/// at `DEQUANT_COEFFICIENT_C[0]` (`= 111.285962475327`, per the
/// `cns-prng-params.meta` notes line, anchored to
/// `32768 / 2 / 255 * sqrt(3)`).
pub fn dequantise_cns_band(
    cns_levels: &[i32; SAMPLES_PER_BAND],
    out: &mut [f64; SAMPLES_PER_BAND],
) {
    let c = DEQUANT_COEFFICIENT_C[0];
    for (dst, &src) in out.iter_mut().zip(cns_levels.iter()) {
        *dst = src as f64 * c / DEQUANT_DIVISOR;
    }
}

/// Helper: return the `D` (= `QUANTIZER_OFFSET_D[band_type + 1]`)
/// associated with a PCM-escape `band_type` in `8..=17`. Returns
/// `None` outside the PCM-escape range.
#[inline]
pub fn pcm_escape_d(band_type: i8) -> Option<i32> {
    if !(8..=17).contains(&band_type) {
        return None;
    }
    let idx = band_type_index(band_type)?;
    Some(QUANTIZER_OFFSET_D[idx] as i32)
}

/// Compile-time sanity: keep the dequant divisor synced with the
/// requantiser-table relation. The CSV-extracted coefficient for
/// band_type 0 (index 1) is exactly `65536.0`; if either value moves,
/// the spec relation no longer holds.
#[inline]
fn _sanity_band0_c_is_divisor() {
    // Not exposed; just a guarded local assertion the test below
    // also verifies.
    assert!((DEQUANT_COEFFICIENT_C[1] - DEQUANT_DIVISOR).abs() < 1e-9);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cns::CnsPrng;
    use crate::requant::QUANTIZER_OFFSET_D;

    // ─── PCM centring ──────────────────────────────────────

    #[test]
    fn centre_pcm_level_subtracts_d_for_band_type_8() {
        // band_type 8: D = QUANTIZER_OFFSET_D[9] = 63. Raw range
        // 0..=126 (7 unsigned bits). Centred range -63..=63.
        let d = QUANTIZER_OFFSET_D[9] as i32;
        assert_eq!(d, 63);
        assert_eq!(centre_pcm_level(8, 0).unwrap(), -d);
        assert_eq!(centre_pcm_level(8, d).unwrap(), 0);
        assert_eq!(centre_pcm_level(8, 2 * d).unwrap(), d);
    }

    #[test]
    fn centre_pcm_level_subtracts_d_for_band_type_17() {
        // band_type 17: D = QUANTIZER_OFFSET_D[18] = 32767.
        let d = QUANTIZER_OFFSET_D[18] as i32;
        assert_eq!(d, 32767);
        assert_eq!(centre_pcm_level(17, 0).unwrap(), -d);
        assert_eq!(centre_pcm_level(17, d).unwrap(), 0);
        assert_eq!(centre_pcm_level(17, 2 * d).unwrap(), d);
    }

    #[test]
    fn centre_pcm_level_rejects_out_of_range_band_types() {
        for bt in [-2_i8, -1, 0, 1, 2, 3, 7, 18, i8::MAX, i8::MIN] {
            assert!(matches!(
                centre_pcm_level(bt, 0),
                Err(Error::UnsupportedBandType(_)),
            ));
        }
    }

    #[test]
    fn centre_pcm_band_in_place_round_trip() {
        // Ramp 0..=126 (band_type 8, 7 unsigned bits): after centring,
        // values should be -63..=63 in order.
        let mut buf = [0_i32; SAMPLES_PER_BAND];
        for (i, slot) in buf.iter_mut().enumerate() {
            *slot = (i as i32) * 2;
        }
        // Highest in [0, 2D] for band_type 8 (2D=126) is buf[35]=70.
        // We only need to verify the subtraction is applied
        // uniformly, not assert clamping.
        centre_pcm_band(8, &mut buf).unwrap();
        let d = QUANTIZER_OFFSET_D[9] as i32;
        assert_eq!(d, 63);
        for (i, &v) in buf.iter().enumerate() {
            assert_eq!(v, (i as i32) * 2 - d);
        }
    }

    #[test]
    fn centre_pcm_band_rejects_out_of_range_band_types() {
        let mut buf = [0_i32; SAMPLES_PER_BAND];
        for bt in [-1_i8, 0, 3, 7, 18] {
            assert!(matches!(
                centre_pcm_band(bt, &mut buf),
                Err(Error::UnsupportedBandType(_)),
            ));
        }
    }

    // ─── Single-sample dequantisation ──────────────────────

    #[test]
    fn dequantise_sample_band_type_0_is_identity_scaled_by_c() {
        // band_type 0: D = 0, C = 65536. centred level can only be 0.
        // For other inputs (e.g. a stress test), C/65536 = 1, so
        // dequant == centred_level.
        let val = dequantise_sample(0, 0).unwrap();
        assert!((val - 0.0).abs() < 1e-12);
        let val = dequantise_sample(0, 5).unwrap();
        assert!((val - 5.0).abs() < 1e-9, "C / 65536 should be 1.0");
    }

    #[test]
    fn dequantise_sample_band_type_3_uses_correct_c() {
        // band_type 3: D = QUANTIZER_OFFSET_D[4] = 3 (one above
        // band_type 2's D=2 in the entropy ladder), 2D+1 = 7,
        // C = 65536/7 ≈ 9362.285714. dequantise_sample(3, 3) =
        // 3 * C / 65536 = 3/7 ≈ 0.428571.
        let d = QUANTIZER_OFFSET_D[band_type_index(3).unwrap()] as i32;
        assert_eq!(d, 3);
        let val = dequantise_sample(3, d).unwrap();
        let expected = d as f64 / (2.0 * d as f64 + 1.0);
        assert!((val - expected).abs() < 1e-9, "got {val}, want {expected}");
        let val = dequantise_sample(3, -d).unwrap();
        assert!((val + expected).abs() < 1e-9, "got {val}");
    }

    #[test]
    fn dequantise_sample_band_type_17_uses_correct_c() {
        // band_type 17: D = 32767, 2D+1 = 65535, C = 65536/65535
        // ≈ 1.00001526. dequant of D should be ~D / 65535 * 65536 / 65536
        // = D / 65535 ≈ 0.499992...
        let val = dequantise_sample(17, 32767).unwrap();
        // Expected: 32767 * (65536/65535) / 65536 = 32767/65535 ≈ 0.49999237
        let expected = 32767.0_f64 / 65535.0;
        assert!((val - expected).abs() < 1e-9, "got {val}, want {expected}");
    }

    #[test]
    fn dequantise_sample_cns_band_uses_c0() {
        // band_type -1 -> index 0 -> C = 111.285962475327.
        // Dequant of 0 = 0; dequant of 510 = 510 * C / 65536.
        let val = dequantise_sample(-1, 0).unwrap();
        assert!(val.abs() < 1e-12);
        let val = dequantise_sample(-1, 510).unwrap();
        let expected = 510.0_f64 * 111.285962475327 / 65536.0;
        assert!((val - expected).abs() < 1e-9, "got {val}, want {expected}");
    }

    #[test]
    fn dequantise_sample_rejects_out_of_range() {
        for bt in [-2_i8, 18, i8::MAX, i8::MIN] {
            assert!(matches!(
                dequantise_sample(bt, 0),
                Err(Error::UnsupportedBandType(_)),
            ));
        }
    }

    // ─── Whole-band dequantisation ─────────────────────────

    #[test]
    fn dequantise_band_matches_single_sample_path() {
        let mut centred = [0_i32; SAMPLES_PER_BAND];
        for (i, slot) in centred.iter_mut().enumerate() {
            // Span -D..=D for band_type 5 (D=4): use signed ramp.
            *slot = (i as i32) - 18;
        }
        let mut out = [0.0_f64; SAMPLES_PER_BAND];
        dequantise_band(5, &centred, &mut out).unwrap();
        for i in 0..SAMPLES_PER_BAND {
            let expected = dequantise_sample(5, centred[i]).unwrap();
            assert!(
                (out[i] - expected).abs() < 1e-12,
                "sample {i}: got {} want {}",
                out[i],
                expected
            );
        }
    }

    #[test]
    fn dequantise_band_rejects_negative_band_type() {
        // Use dequantise_cns_band for CNS; dequantise_band only
        // handles 0..=17.
        let centred = [0_i32; SAMPLES_PER_BAND];
        let mut out = [0.0_f64; SAMPLES_PER_BAND];
        assert!(matches!(
            dequantise_band(-1, &centred, &mut out),
            Err(Error::UnsupportedBandType(_)),
        ));
        assert!(matches!(
            dequantise_band(18, &centred, &mut out),
            Err(Error::UnsupportedBandType(_)),
        ));
    }

    #[test]
    fn dequantise_huffman_band_round_trips_signed_i8() {
        // band_type 3 Q3 values lie in -D..=D = -2..=2. Build a
        // synthetic 36-sample signed pattern and dequantise.
        let mut huffman_levels = [0_i8; SAMPLES_PER_BAND];
        for (i, slot) in huffman_levels.iter_mut().enumerate() {
            *slot = ((i as i32 - 18) % 3) as i8; // values in -2..=2
        }
        let mut out = [0.0_f64; SAMPLES_PER_BAND];
        dequantise_huffman_band(3, &huffman_levels, &mut out).unwrap();
        let c = DEQUANT_COEFFICIENT_C[band_type_index(3).unwrap()];
        for i in 0..SAMPLES_PER_BAND {
            let expected = huffman_levels[i] as f64 * c / DEQUANT_DIVISOR;
            assert!((out[i] - expected).abs() < 1e-12);
        }
    }

    #[test]
    fn dequantise_huffman_band_rejects_outside_3_7() {
        let huffman_levels = [0_i8; SAMPLES_PER_BAND];
        let mut out = [0.0_f64; SAMPLES_PER_BAND];
        for bt in [-1_i8, 0, 1, 2, 8, 17, 18] {
            assert!(matches!(
                dequantise_huffman_band(bt, &huffman_levels, &mut out),
                Err(Error::UnsupportedBandType(_)),
            ));
        }
    }

    #[test]
    fn dequantise_cns_band_uses_c0_constant() {
        // Take a fresh CnsPrng walk, then dequantise.
        let mut prng = CnsPrng::new();
        let mut cns_levels = [0_i32; SAMPLES_PER_BAND];
        prng.fill_samples(&mut cns_levels);
        let mut out = [0.0_f64; SAMPLES_PER_BAND];
        dequantise_cns_band(&cns_levels, &mut out);
        let c = DEQUANT_COEFFICIENT_C[0];
        for i in 0..SAMPLES_PER_BAND {
            let expected = cns_levels[i] as f64 * c / DEQUANT_DIVISOR;
            assert!((out[i] - expected).abs() < 1e-12);
        }
        // CNS samples have a known bound -510..=510 from cns.rs; the
        // dequantised magnitude is therefore bounded by 510 * C / 65536.
        let max_mag = 510.0_f64 * c / DEQUANT_DIVISOR;
        for &v in out.iter() {
            assert!(
                v.abs() <= max_mag + 1e-9,
                "CNS dequant out of expected magnitude bound"
            );
        }
    }

    // ─── pcm_escape_d helper ───────────────────────────────

    #[test]
    fn pcm_escape_d_matches_quantizer_offset_table() {
        for bt in 8_i8..=17 {
            let d = pcm_escape_d(bt).expect("PCM range");
            let idx = band_type_index(bt).unwrap();
            assert_eq!(d, QUANTIZER_OFFSET_D[idx] as i32);
        }
        for bt in [-1_i8, 0, 7, 18] {
            assert!(pcm_escape_d(bt).is_none());
        }
    }

    // ─── Cross-module integration: PCM-escape decode -> centre -> dequant ───

    #[test]
    fn pcm_escape_decode_then_centre_then_dequant_round_trips() {
        use crate::huffman::Sv7BitReader;
        use crate::sv7_band_decode::decode_linear_pcm_band;

        // band_type 8 -> 7 bits per sample. Build a stream where each
        // sample's raw level encodes its position modulo 2D+1.
        let two_d_plus_1 = (2 * QUANTIZER_OFFSET_D[9] + 1) as u32; // 127
        let expected_raw: Vec<u32> = (0..SAMPLES_PER_BAND as u32)
            .map(|i| i % two_d_plus_1)
            .collect();
        let mut bits = Vec::new();
        let mut acc: u32 = 0;
        let mut nbits: u32 = 0;
        for &v in &expected_raw {
            acc = (acc << 7) | v;
            nbits += 7;
            while nbits >= 8 {
                let shift = nbits - 8;
                bits.push((acc >> shift) as u8);
                acc &= (1 << shift) - 1;
                nbits -= 8;
            }
        }
        if nbits > 0 {
            bits.push((acc << (8 - nbits)) as u8);
        }

        // Decode raw.
        let mut reader = Sv7BitReader::new(&bits);
        let mut raw = [0_i32; SAMPLES_PER_BAND];
        decode_linear_pcm_band(&mut reader, 8, &mut raw).expect("decode");
        for i in 0..SAMPLES_PER_BAND {
            assert_eq!(raw[i] as u32, expected_raw[i]);
        }

        // Centre.
        centre_pcm_band(8, &mut raw).expect("centre");
        let d = QUANTIZER_OFFSET_D[9] as i32;
        for i in 0..SAMPLES_PER_BAND {
            let want = expected_raw[i] as i32 - d;
            assert_eq!(raw[i], want, "sample {i} centred");
            assert!((-d..=d).contains(&raw[i]));
        }

        // Dequant.
        let mut out = [0.0_f64; SAMPLES_PER_BAND];
        dequantise_band(8, &raw, &mut out).expect("dequant");
        let c = DEQUANT_COEFFICIENT_C[9];
        for i in 0..SAMPLES_PER_BAND {
            let expected = raw[i] as f64 * c / DEQUANT_DIVISOR;
            assert!((out[i] - expected).abs() < 1e-12);
        }
    }

    // ─── _sanity_band0_c_is_divisor doesn't drift ──────────

    #[test]
    fn band_type_0_dequant_coefficient_equals_divisor() {
        // The relation C = 65536 / (2D+1) with D=0 gives C = 65536,
        // which is exactly DEQUANT_DIVISOR. This invariant is what
        // lets dequantise_sample(0, x) == x.
        assert!((DEQUANT_COEFFICIENT_C[1] - DEQUANT_DIVISOR).abs() < 1e-9);
        // And call the internal sanity helper so dead-code analysis
        // can't drop it.
        _sanity_band0_c_is_divisor();
    }
}
