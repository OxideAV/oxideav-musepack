//! Perceptual per-band bit allocation (SMR-driven step targets) —
//! **pure encoder policy**, shared by both encoder generations.
//!
//! The flat `step_target` allocation spends the same absolute noise
//! step in every subband, which over-spends on loud bands (whose
//! quantisation noise is buried under the band's own signal) and on
//! quiet bands adjacent to loud ones (whose noise the neighbour
//! masks). This module derives a per-band step-target vector from the
//! frame's own band energies instead: each band's tolerable noise
//! floor is the maximum of
//!
//! - the band's **masked** floor — the in-frame band powers spread
//!   into neighbouring bands with direction-dependent decay, then
//!   lowered by a quality-scaled signal-to-mask margin, and
//! - a quality-scaled **absolute** floor (noise that stays below the
//!   quiet threshold is tolerable regardless of masking),
//!
//! and the step target follows from the uniform-quantiser noise law
//! (noise power ≈ `step² / 12`). One `quality` knob (0 = coarsest,
//! 10 = finest) scales both floors, giving the encoders a measured
//! rate ladder ([`crate::sv8_file_encode::Sv8EncoderSettings`] /
//! [`crate::sv7_pcm_encode::Sv7EncoderSettings`], `quality`).
//!
//! All constants here are policy choices validated by this crate's own
//! round-trip / oracle gates — they are **not** format facts (nothing
//! about the allocation is visible on the wire beyond the resulting
//! `Res` / SCF choices, which any decoder accepts).

use crate::frame_reconstruct::SubbandMatrix;
use crate::sv7_band_decode::SAMPLES_PER_BAND;
use crate::sv7_band_header::SV7_SUBBAND_COUNT;

/// Lowest meaningful quality; values below clamp here.
pub const SMR_QUALITY_MIN: f64 = 0.0;

/// Highest meaningful quality; values above clamp here.
pub const SMR_QUALITY_MAX: f64 = 10.0;

/// Masking-spread decay toward **higher** bands, dB per band (a loud
/// band masks the bands above it more strongly than those below).
const SPREAD_UP_DB_PER_BAND: f64 = 12.0;

/// Masking-spread decay toward **lower** bands, dB per band.
const SPREAD_DOWN_DB_PER_BAND: f64 = 24.0;

/// Finest step target the policy will request (s16 LSBs) — below
/// this the format's finest quantisers are already saturated.
const MIN_STEP: f64 = 0.25;

/// Coarsest step target the policy will request (s16 LSBs).
const MAX_STEP: f64 = 8192.0;

/// The signal-to-mask margin in dB at `quality`: how far below a
/// band's masked floor the quantisation noise must sit.
fn margin_db(quality: f64) -> f64 {
    10.0 + 6.0 * quality
}

/// The absolute-floor step at `quality` (s16 LSBs): the step whose
/// noise is taken as inaudible regardless of masking.
fn floor_step(quality: f64) -> f64 {
    0.4 * ((SMR_QUALITY_MAX - quality) / 2.0).exp2()
}

/// Mean-square power of one band's 36 subband samples.
fn band_power(band: &[f64; SAMPLES_PER_BAND]) -> f64 {
    band.iter().map(|&x| x * x).sum::<f64>() / SAMPLES_PER_BAND as f64
}

/// Per-band quantisation-step targets for one stereo frame at
/// `quality` (clamped to [`SMR_QUALITY_MIN`]`..=`[`SMR_QUALITY_MAX`]).
/// Bands `0..nbands` are analysed (the caller's `max_band + 1`);
/// entries past `nbands` hold the absolute floor. For mono, pass the
/// same matrix twice (the two channels' powers are joined by max, so
/// the duplicate is free).
///
/// The result plugs into the per-band `Res` policy
/// ([`crate::sv8_quantize::band_type_for_peak`]) exactly where the
/// flat `step_target` did.
#[must_use]
pub fn smr_step_targets(
    left: &SubbandMatrix,
    right: &SubbandMatrix,
    nbands: usize,
    quality: f64,
) -> [f64; SV7_SUBBAND_COUNT] {
    let q = quality.clamp(SMR_QUALITY_MIN, SMR_QUALITY_MAX);
    let nb = nbands.min(SV7_SUBBAND_COUNT);

    // Joint band power: masking is estimated over the louder of the
    // two channels so a per-band M/S election downstream can never
    // starve the quieter one.
    let mut mask = [0.0_f64; SV7_SUBBAND_COUNT];
    for b in 0..nb {
        mask[b] = band_power(&left[b]).max(band_power(&right[b]));
    }

    // Directional spreading as running max-decay passes: a masker's
    // contribution to band b ± k is its power attenuated by the
    // per-band slope, and the running form composes the slopes
    // without an O(n²) pairwise sweep.
    let up = 10.0_f64.powf(-SPREAD_UP_DB_PER_BAND / 10.0);
    let down = 10.0_f64.powf(-SPREAD_DOWN_DB_PER_BAND / 10.0);
    for b in 1..nb {
        mask[b] = mask[b].max(mask[b - 1] * up);
    }
    for b in (0..nb.saturating_sub(1)).rev() {
        mask[b] = mask[b].max(mask[b + 1] * down);
    }

    let margin = 10.0_f64.powf(-margin_db(q) / 10.0);
    let floor = floor_step(q).clamp(MIN_STEP, MAX_STEP);
    let floor_noise = floor * floor / 12.0;
    let mut steps = [floor; SV7_SUBBAND_COUNT];
    for b in 0..nb {
        let allowed_noise = (mask[b] * margin).max(floor_noise);
        steps[b] = (12.0 * allowed_noise).sqrt().clamp(MIN_STEP, MAX_STEP);
    }
    steps
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame_reconstruct::zero_subband_matrix;

    fn matrix_with_band(band: usize, amp: f64) -> SubbandMatrix {
        let mut m = zero_subband_matrix();
        for (k, v) in m[band].iter_mut().enumerate() {
            // A full-band deterministic waveform with peak ≈ amp.
            *v = amp * if k % 2 == 0 { 1.0 } else { -0.7 };
        }
        m
    }

    #[test]
    fn quiet_bands_far_from_a_masker_get_the_absolute_floor() {
        let m = matrix_with_band(5, 8000.0);
        let steps = smr_step_targets(&m, &m, 32, 5.0);
        // Band 20 is 15 bands above the masker: fully floored.
        assert!(
            (steps[20] - floor_step(5.0)).abs() < 1e-12,
            "step[20] = {}",
            steps[20]
        );
        // The masker's own band tolerates far more noise.
        assert!(steps[5] > 20.0 * steps[20], "step[5] = {}", steps[5]);
    }

    #[test]
    fn masking_spreads_more_upward_than_downward() {
        let m = matrix_with_band(10, 8000.0);
        let steps = smr_step_targets(&m, &m, 32, 5.0);
        assert!(
            steps[11] > steps[9],
            "up {} vs down {}",
            steps[11],
            steps[9]
        );
        assert!(steps[11] < steps[10]);
    }

    #[test]
    fn higher_quality_means_finer_steps_everywhere() {
        let mut m = matrix_with_band(3, 6000.0);
        let loud = matrix_with_band(14, 2500.0);
        for b in 0..32 {
            for k in 0..crate::sv7_band_decode::SAMPLES_PER_BAND {
                m[b][k] += loud[b][k] + 3.0;
            }
        }
        let lo = smr_step_targets(&m, &m, 32, 2.0);
        let hi = smr_step_targets(&m, &m, 32, 8.0);
        for b in 0..32 {
            assert!(hi[b] < lo[b], "band {b}: {} !< {}", hi[b], lo[b]);
        }
    }

    #[test]
    fn quality_is_clamped_and_channels_join_by_max() {
        let l = matrix_with_band(7, 5000.0);
        let r = zero_subband_matrix();
        let a = smr_step_targets(&l, &r, 32, -3.0);
        let b = smr_step_targets(&l, &r, 32, 0.0);
        assert_eq!(a, b);
        // Swapping the channels changes nothing.
        let c = smr_step_targets(&r, &l, 32, 0.0);
        assert_eq!(a, c);
    }

    #[test]
    fn steps_stay_within_the_policy_bounds() {
        let loud = matrix_with_band(0, 32767.0);
        let steps = smr_step_targets(&loud, &loud, 32, 0.0);
        assert!(steps.iter().all(|&s| (MIN_STEP..=MAX_STEP).contains(&s)));
        let silent = zero_subband_matrix();
        let steps = smr_step_targets(&silent, &silent, 32, 10.0);
        assert!(steps.iter().all(|&s| (MIN_STEP..=MAX_STEP).contains(&s)));
    }
}
