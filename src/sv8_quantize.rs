//! SV8 **forward quantisation** — subband samples → (SCF indices,
//! quantised levels), the exact inverse of the corpus-pinned absolute
//! reconstruction law.
//!
//! The decode side reconstructs `sample = level × C[Res + 1] ×
//! SCF_STEP_RATIO^(scf − 1)` in the s16 domain
//! ([`crate::reconstruct::reconstruct_sv8_band_absolute`], pinned by
//! the r390/r419 corpora). This module inverts it for the encoder:
//!
//! - [`choose_granule_scf`] picks, per 12-sample granule, the **largest**
//!   SCF index whose gain still spans the granule's peak — the largest
//!   index has the smallest gain (the ratio is < 1), i.e. the finest
//!   quantisation step that keeps every level inside the `±D[Res]`
//!   alphabet.
//! - [`quantize_granule`] / [`quantize_band`] round each sample to the
//!   nearest level at that step and clamp to `±D[Res]` (the clamp only
//!   engages when the SCF range itself is exhausted at
//!   [`SV8_SCF_MIN`], i.e. the input exceeds the format's headroom).
//! - [`band_type_for_peak`] is the **encoder bit-allocation policy**:
//!   the smallest `Res` whose quantiser meets a caller-supplied
//!   absolute noise-step target (a flat s16-domain noise floor). Pure
//!   policy — any `Res` produces a valid stream; the choice only
//!   trades rate against quantisation noise.
//!
//! Everything here is the algebraic inverse of decode-side facts
//! already in the crate ([`crate::requant`] constants + the absolute
//! gain law); no new format facts. Round-trip error bounds are gated
//! by the module tests.
//!
//! Source-of-record: `docs/audio/musepack/musepack-sv7-sv8-spec.md`
//! §2.6/§3.6 (reconstruction law, shared SV7/SV8 signal model) and the
//! staged `tables/requant-*` + `tables/scf-step-ratio` facts, via
//! [`crate::requant`] / [`crate::reconstruct`].

use crate::reconstruct::sv7_absolute_scf_gain;
use crate::requant::{band_type_index, DEQUANT_COEFFICIENT_C, QUANTIZER_OFFSET_D, SCF_STEP_RATIO};
use crate::scf::SCF_GRANULES_PER_BAND;
use crate::sv7_band_decode::SAMPLES_PER_BAND;
use crate::{Error, Result};

/// Samples per scalefactor granule (3 granules × 12 = 36 per band,
/// the Layer-II inheritance — spec §1).
pub const SAMPLES_PER_GRANULE: usize = SAMPLES_PER_BAND / SCF_GRANULES_PER_BAND;

/// Lowest SV8 SCF index: the §6.3 fold `((… ) & 127) − 6` bottoms out
/// at `−6` (raw 7-bit 0).
pub const SV8_SCF_MIN: i32 = -6;

/// Highest SV8 SCF index: the §6.3 fold tops out at `127 − 6 = 121`.
pub const SV8_SCF_MAX: i32 = 121;

/// The quantisation step (s16-domain size of one level increment) for
/// `band_type` at SCF index `scf`: `C[band_type + 1] ×
/// SCF_STEP_RATIO^(scf − 1)`.
///
/// # Errors
///
/// [`Error::UnsupportedBandType`] for a `band_type` outside `1..=17`
/// (only sample-bearing coded band types quantise).
#[inline]
pub fn quant_step(band_type: i8, scf: i32) -> Result<f64> {
    if !(1..=17).contains(&band_type) {
        return Err(Error::UnsupportedBandType(band_type));
    }
    let idx = band_type_index(band_type).ok_or(Error::UnsupportedBandType(band_type))?;
    Ok(DEQUANT_COEFFICIENT_C[idx] * sv7_absolute_scf_gain(scf))
}

/// Pick the largest SCF index in [`SV8_SCF_MIN`]`..=`[`SV8_SCF_MAX`]
/// whose gain still spans `peak`: the finest step with
/// `D[band_type] × step(scf) ≥ peak`, so every rounded level fits the
/// `±D` alphabet.
///
/// A non-positive `peak` (an all-zero granule) returns
/// [`SV8_SCF_MAX`] (the quietest gain; the levels are zero anyway). A
/// `peak` beyond the format's headroom saturates at [`SV8_SCF_MIN`]
/// (the caller's levels will clamp).
///
/// # Errors
///
/// [`Error::UnsupportedBandType`] for a `band_type` outside `1..=17`.
pub fn choose_granule_scf(band_type: i8, peak: f64) -> Result<i32> {
    if !(1..=17).contains(&band_type) {
        return Err(Error::UnsupportedBandType(band_type));
    }
    let idx = band_type_index(band_type).ok_or(Error::UnsupportedBandType(band_type))?;
    let span = DEQUANT_COEFFICIENT_C[idx] * f64::from(QUANTIZER_OFFSET_D[idx]);
    if peak <= 0.0 {
        return Ok(SV8_SCF_MAX);
    }
    // Need gain(scf) = ratio^(scf−1) ≥ peak / span; with ratio < 1 the
    // log bound is an upper bound on scf (the largest fitting index).
    let t = peak / span;
    let mut scf =
        ((t.ln() / SCF_STEP_RATIO.ln()).floor() as i32 + 1).clamp(SV8_SCF_MIN, SV8_SCF_MAX);
    // Float-fuzz guard: step down (louder) until the span really fits.
    while scf > SV8_SCF_MIN && span * sv7_absolute_scf_gain(scf) < peak {
        scf -= 1;
    }
    Ok(scf)
}

/// Quantise one granule's 12 samples at `(band_type, scf)`: nearest
/// level, clamped to `±D[band_type]`.
///
/// # Errors
///
/// [`Error::UnsupportedBandType`] for a `band_type` outside `1..=17`.
pub fn quantize_granule(band_type: i8, scf: i32, samples: &[f64], out: &mut [i32]) -> Result<()> {
    debug_assert_eq!(samples.len(), SAMPLES_PER_GRANULE);
    debug_assert_eq!(out.len(), SAMPLES_PER_GRANULE);
    let step = quant_step(band_type, scf)?;
    let idx = band_type_index(band_type).ok_or(Error::UnsupportedBandType(band_type))?;
    let d = i32::from(QUANTIZER_OFFSET_D[idx]);
    for (o, &x) in out.iter_mut().zip(samples.iter()) {
        let level = (x / step).round();
        *o = (level as i32).clamp(-d, d);
    }
    Ok(())
}

/// The result of quantising one band: the three per-granule SCF
/// indices and the 36 levels.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QuantizedBand {
    /// Per-granule SCF indices (each in
    /// [`SV8_SCF_MIN`]`..=`[`SV8_SCF_MAX`]).
    pub scf: [i32; SCF_GRANULES_PER_BAND],
    /// The 36 quantised levels, granule-major, each within
    /// `±D[band_type]`.
    pub levels: [i32; SAMPLES_PER_BAND],
}

/// Quantise one band's 36 subband samples at `band_type`: choose each
/// granule's SCF from its own peak, then round the levels.
///
/// All-zero granules **reuse a neighbouring coded granule's SCF**
/// (the previous one, or the first following non-zero granule) so the
/// SCFI layer can share indices instead of coding a meaningless jump
/// to the quietest gain; an all-zero *band* uses [`SV8_SCF_MAX`]
/// throughout (callers normally give such bands `Res = 0` instead).
///
/// # Errors
///
/// [`Error::UnsupportedBandType`] for a `band_type` outside `1..=17`.
pub fn quantize_band(band_type: i8, samples: &[f64; SAMPLES_PER_BAND]) -> Result<QuantizedBand> {
    let mut peaks = [0.0_f64; SCF_GRANULES_PER_BAND];
    for (g, peak) in peaks.iter_mut().enumerate() {
        for &x in &samples[g * SAMPLES_PER_GRANULE..(g + 1) * SAMPLES_PER_GRANULE] {
            *peak = peak.max(x.abs());
        }
    }

    // Per-granule SCF, with zero-granule neighbour reuse: coded
    // granules choose their own index, zero granules copy the previous
    // coded one (forward fill), and leading zero granules copy the
    // first coded one (backward fill).
    let mut scf = [SV8_SCF_MAX; SCF_GRANULES_PER_BAND];
    let mut have = [false; SCF_GRANULES_PER_BAND];
    for g in 0..SCF_GRANULES_PER_BAND {
        if peaks[g] > 0.0 {
            scf[g] = choose_granule_scf(band_type, peaks[g])?;
            have[g] = true;
        }
    }
    for g in 1..SCF_GRANULES_PER_BAND {
        if !have[g] && have[g - 1] {
            scf[g] = scf[g - 1];
            have[g] = true;
        }
    }
    for g in (0..SCF_GRANULES_PER_BAND - 1).rev() {
        if !have[g] && have[g + 1] {
            scf[g] = scf[g + 1];
            have[g] = true;
        }
    }

    let mut levels = [0_i32; SAMPLES_PER_BAND];
    for (g, &granule_scf) in scf.iter().enumerate() {
        let range = g * SAMPLES_PER_GRANULE..(g + 1) * SAMPLES_PER_GRANULE;
        quantize_granule(
            band_type,
            granule_scf,
            &samples[range.clone()],
            &mut levels[range],
        )?;
    }
    Ok(QuantizedBand { scf, levels })
}

/// Encoder bit-allocation policy: the smallest sample-bearing
/// `band_type` whose quantiser keeps the step at or below
/// `step_target` (an absolute s16-domain noise-step budget), or `0`
/// (silent band) when the peak itself sinks below half the target
/// step (zeroing the band then costs no more than coding it).
///
/// Returns `17` (the finest quantiser) when even it cannot reach the
/// target — the achieved step is then `peak / 32767`.
///
/// Pure encoder policy: with the granule SCF chosen by
/// [`choose_granule_scf`], the realised step is
/// `peak / D ≤ step ≤ peak / (D × SCF_STEP_RATIO)`, so requiring
/// `D ≥ peak / step_target` bounds the step by
/// `step_target / SCF_STEP_RATIO` in the worst SCF-granularity case.
#[must_use]
pub fn band_type_for_peak(peak: f64, step_target: f64) -> i8 {
    debug_assert!(step_target > 0.0);
    if peak < 0.5 * step_target {
        return 0;
    }
    let needed = peak / step_target;
    for bt in 1..=17_i8 {
        let idx = band_type_index(bt).expect("1..=17 in range");
        if f64::from(QUANTIZER_OFFSET_D[idx]) >= needed {
            return bt;
        }
    }
    17
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::reconstruct::reconstruct_sv8_band_absolute;

    /// Deterministic pseudo-random f64 in [-1, 1) (xorshift64*).
    struct Rng(u64);
    impl Rng {
        fn next(&mut self) -> f64 {
            let mut x = self.0;
            x ^= x >> 12;
            x ^= x << 25;
            x ^= x >> 27;
            self.0 = x;
            let v = x.wrapping_mul(0x2545_F491_4F6C_DD1D);
            ((v >> 11) as f64) / ((1u64 << 53) as f64) * 2.0 - 1.0
        }
    }

    /// The chosen SCF is maximal: the span fits the peak, and the next
    /// finer step (scf + 1) would not — except at the range ends.
    #[test]
    fn granule_scf_is_maximal_fitting_index() {
        for bt in 1..=17_i8 {
            let idx = band_type_index(bt).unwrap();
            let span = DEQUANT_COEFFICIENT_C[idx] * f64::from(QUANTIZER_OFFSET_D[idx]);
            for &peak in &[0.5, 3.0, 100.0, 5_000.0, 32_767.0, 40_000.0] {
                let scf = choose_granule_scf(bt, peak).unwrap();
                assert!(
                    (SV8_SCF_MIN..=SV8_SCF_MAX).contains(&scf),
                    "bt {bt} peak {peak}"
                );
                if scf > SV8_SCF_MIN {
                    assert!(
                        span * sv7_absolute_scf_gain(scf) >= peak,
                        "bt {bt} peak {peak}: chosen scf {scf} must span the peak"
                    );
                }
                if scf < SV8_SCF_MAX {
                    assert!(
                        span * sv7_absolute_scf_gain(scf + 1) < peak || scf == SV8_SCF_MIN,
                        "bt {bt} peak {peak}: scf {scf} must be maximal"
                    );
                }
            }
        }
    }

    #[test]
    fn zero_peak_takes_quietest_index() {
        assert_eq!(choose_granule_scf(5, 0.0).unwrap(), SV8_SCF_MAX);
        assert_eq!(choose_granule_scf(17, -1.0).unwrap(), SV8_SCF_MAX);
    }

    #[test]
    fn rejects_non_sample_band_types() {
        for bt in [-1, 0, 18, i8::MAX] {
            assert!(matches!(
                choose_granule_scf(bt, 1.0),
                Err(Error::UnsupportedBandType(_))
            ));
            assert!(matches!(
                quant_step(bt, 1),
                Err(Error::UnsupportedBandType(_))
            ));
        }
    }

    /// Quantise → absolute-reconstruct round trip: the error of every
    /// sample stays within half a quantisation step (plus float fuzz)
    /// for every band type, at random peaks.
    #[test]
    fn round_trip_error_within_half_step() {
        let mut rng = Rng(0x5EED_CAFE_0123_4567);
        for bt in 1..=17_i8 {
            for &peak in &[2.0, 57.0, 1_000.0, 30_000.0] {
                let mut samples = [0.0_f64; SAMPLES_PER_BAND];
                for s in samples.iter_mut() {
                    *s = rng.next() * peak;
                }
                let q = quantize_band(bt, &samples).unwrap();
                let mut recon = [0.0_f64; SAMPLES_PER_BAND];
                reconstruct_sv8_band_absolute(bt, &q.levels, q.scf, &mut recon).unwrap();
                for g in 0..SCF_GRANULES_PER_BAND {
                    let step = quant_step(bt, q.scf[g]).unwrap();
                    for k in g * SAMPLES_PER_GRANULE..(g + 1) * SAMPLES_PER_GRANULE {
                        let err = (recon[k] - samples[k]).abs();
                        assert!(
                            err <= 0.5 * step * (1.0 + 1e-9),
                            "bt {bt} peak {peak} sample {k}: err {err} > step/2 {}",
                            0.5 * step
                        );
                    }
                }
            }
        }
    }

    /// Levels always fit the ±D alphabet, even for inputs past the
    /// format headroom (the clamp engages at SV8_SCF_MIN).
    #[test]
    fn levels_fit_alphabet_even_past_headroom() {
        let mut rng = Rng(99);
        for bt in 1..=17_i8 {
            let idx = band_type_index(bt).unwrap();
            let d = i32::from(QUANTIZER_OFFSET_D[idx]);
            let mut samples = [0.0_f64; SAMPLES_PER_BAND];
            for s in samples.iter_mut() {
                *s = rng.next() * 500_000.0; // way past s16
            }
            let q = quantize_band(bt, &samples).unwrap();
            assert!(q.levels.iter().all(|&l| l.abs() <= d), "bt {bt}");
            assert!(
                q.scf.iter().all(|&s| s == SV8_SCF_MIN),
                "bt {bt}: saturated"
            );
        }
    }

    /// Zero granules reuse a neighbour's SCF so the SCFI layer can
    /// share; an all-zero band quantises to all-zero levels.
    #[test]
    fn zero_granules_share_neighbour_scf() {
        let mut samples = [0.0_f64; SAMPLES_PER_BAND];
        // Only granule 1 carries signal.
        for s in samples[SAMPLES_PER_GRANULE..2 * SAMPLES_PER_GRANULE].iter_mut() {
            *s = 500.0;
        }
        let q = quantize_band(5, &samples).unwrap();
        assert_eq!(q.scf[0], q.scf[1], "leading zero granule shares");
        assert_eq!(q.scf[2], q.scf[1], "trailing zero granule shares");
        assert!(q.levels[..SAMPLES_PER_GRANULE].iter().all(|&l| l == 0));

        let silent = quantize_band(5, &[0.0; SAMPLES_PER_BAND]).unwrap();
        assert_eq!(silent.scf, [SV8_SCF_MAX; 3]);
        assert!(silent.levels.iter().all(|&l| l == 0));
    }

    /// The policy: silent below half a step; monotonically finer with
    /// louder peaks; snaps to the smallest fitting D.
    #[test]
    fn band_type_policy_thresholds() {
        let t = 2.0;
        assert_eq!(band_type_for_peak(0.0, t), 0);
        assert_eq!(band_type_for_peak(0.9, t), 0);
        assert_eq!(band_type_for_peak(1.5, t), 1); // D=1 spans 0.75 steps
        assert_eq!(band_type_for_peak(4.0, t), 2); // needs D ≥ 2
        assert_eq!(band_type_for_peak(8.0, t), 4); // needs D ≥ 4
        assert_eq!(band_type_for_peak(14.0, t), 5); // needs D ≥ 7
        assert_eq!(band_type_for_peak(32_767.0, 1.0), 17);
        assert_eq!(band_type_for_peak(1.0e9, 1.0), 17, "saturates at 17");
        // Monotone in peak.
        let mut prev = 0;
        for p in 0..2_000 {
            let bt = band_type_for_peak(f64::from(p), t);
            assert!(bt >= prev, "policy must be monotone");
            prev = bt;
        }
    }

    /// A full-scale band at band_type 17 reconstructs with s16-LSB
    /// level accuracy (the finest quantiser's step at unity SCF is
    /// ≈ 1 LSB).
    #[test]
    fn full_scale_finest_quantiser_is_lsb_accurate() {
        let mut rng = Rng(3);
        let mut samples = [0.0_f64; SAMPLES_PER_BAND];
        for s in samples.iter_mut() {
            *s = rng.next() * 32_000.0;
        }
        let q = quantize_band(17, &samples).unwrap();
        let mut recon = [0.0_f64; SAMPLES_PER_BAND];
        reconstruct_sv8_band_absolute(17, &q.levels, q.scf, &mut recon).unwrap();
        for (k, (&r, &x)) in recon.iter().zip(samples.iter()).enumerate() {
            assert!((r - x).abs() <= 0.7, "sample {k}: {r} vs {x}");
        }
    }
}
