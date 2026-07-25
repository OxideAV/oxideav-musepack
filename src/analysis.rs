//! The 32-band polyphase **analysis subband filter** — the forward
//! counterpart of [`crate::synthesis`], the encoder's front end.
//!
//! Musepack (SV7 and SV8 alike) inherits the MPEG-1 Layer I/II
//! filterbank pair unchanged
//! (`docs/audio/musepack/musepack-sv7-sv8-spec.md` §1: "32-band
//! polyphase analysis/synthesis filterbank. The analysis filterbank
//! splits the PCM stream into 32 equally spaced subbands; the synthesis
//! filterbank reconstructs PCM"). The synthesis half is already wired
//! ([`crate::synthesis::SynthesisFilter`], ISO Figure A.2 + the
//! Table 3-B.3 window transcribed as
//! [`crate::synthesis::SYNTHESIS_WINDOW`]); this module adds the
//! matching forward transform so PCM can enter the subband domain for
//! encoding.
//!
//! # Structure
//!
//! The analysis filter is the classic polyphase forward form of the
//! same cosine-modulated pseudo-QMF bank (general DSP knowledge — the
//! structural spec's source S4 — parameterised entirely by data already
//! in this crate):
//!
//! 1. **Shift** — a 512-sample input FIFO `X` advances by 32: the 32
//!    new PCM samples enter with the **newest sample at `X[0]`**
//!    (`X[i] = input[31 − i]`), older audio sliding up toward `X[511]`.
//! 2. **Window** — `Z[i] = X[i] · C[i]` with the 512-tap analysis
//!    window `C`. The analysis prototype is the **same lowpass
//!    prototype** as the synthesis window: [`analysis_window`] returns
//!    `SYNTHESIS_WINDOW[i] / 32` (the ÷32 compensates the 32× the
//!    synthesis side's matrix + window fold applies across a
//!    critically-sampled 32-band pair; the round-trip unity gain is
//!    test-pinned below).
//! 3. **Partial sums** — `Y[j] = Σ_{k=0..7} Z[j + 64k]`, `j = 0..63`,
//!    folding the 512 windowed taps onto one 64-phase period.
//! 4. **Matrix** — `S[sb] = Σ_{j=0..63} M[sb][j] · Y[j]` with
//!    `M[sb][j] = cos[(2·sb + 1)·(j − 16)·π / 64]`
//!    ([`analysis_matrix_coefficient`]) — the forward twin of the
//!    synthesis `N_ik = cos[(16 + i)·(2k + 1)·π / 64]`
//!    ([`crate::synthesis::matrix_coefficient`]).
//!
//! Each [`AnalysisFilter::analyze`] call consumes 32 PCM samples and
//! produces one 32-subband sample vector (one time slot);
//! [`analyze_frame_channel`] drives 36 slots to fill one frame's
//! [`crate::frame_reconstruct::SubbandMatrix`] (32 × 36 = 1152 PCM
//! samples per frame per channel, spec §1).
//!
//! # Validation (in-crate, empirical)
//!
//! The convention above is **pinned by round-trip through the
//! oracle-validated synthesis filter**: analysis → synthesis
//! reproduces the input delayed by exactly
//! [`ANALYSIS_SYNTHESIS_DELAY`] samples at unity gain, with the
//! reconstruction error at ≈ −84 dB on full-band white noise (the
//! pseudo-QMF pair's aliasing-cancellation ripple plus the Table
//! 3-B.3 print precision — the window is printed to 9 decimals in
//! 1/65536 steps) — see the module tests. Any wrong sign/order/scale
//! variant collapses that round trip to single-digit SNR, so the
//! tests discriminate the convention completely. No external decoder
//! or encoder source was consulted.
//!
//! # Source-of-record
//!
//! - `docs/audio/musepack/musepack-sv7-sv8-spec.md` §1 (S3/S4) — the
//!   shared 32-band polyphase analysis/synthesis pair and the frame
//!   geometry.
//! - The prototype window data: [`crate::synthesis::SYNTHESIS_WINDOW`]
//!   (ISO Table 3-B.3, already transcribed in-crate).
//! - The round-trip behaviour of [`crate::synthesis::SynthesisFilter`]
//!   (oracle-validated ±1 LSB by the SV7/SV8 corpora) pins the rest.

use crate::frame_reconstruct::{zero_subband_matrix, SubbandMatrix};
use crate::sv7_band_decode::SAMPLES_PER_BAND;
use crate::synthesis::{SUBBANDS, SYNTHESIS_WINDOW, WINDOW_LEN};
use crate::SAMPLES_PER_FRAME_PER_CHANNEL;
use core::f64::consts::PI;

/// End-to-end delay of the analysis → synthesis round trip, in PCM
/// samples: the reconstructed signal is the input delayed by exactly
/// this many samples (at unity gain).
///
/// `481 = 512 − 32 + 1`: the two 512-tap polyphase halves overlap all
/// but one sample of their combined support. Pinned empirically by the
/// module tests (the delay scan finds the correlation peak at 481 and
/// unity gain there). Identical by construction to the decoder-side
/// [`crate::synthesis::SYNTHESIS_PRIME_SAMPLES`] skip — the decoder
/// discards exactly the pair's warm-up.
pub const ANALYSIS_SYNTHESIS_DELAY: usize = crate::synthesis::SYNTHESIS_PRIME_SAMPLES;

/// The analysis window `C[i]`: the shared lowpass prototype scaled to
/// make the critically-sampled analysis+synthesis pair unity-gain
/// (`SYNTHESIS_WINDOW[i] / 32`).
#[inline]
#[must_use]
pub fn analysis_window(i: usize) -> f64 {
    SYNTHESIS_WINDOW[i] / 32.0
}

/// The analysis matrixing coefficient
/// `M[sb][j] = cos[(2·sb + 1)·(j − 16)·π / 64]` for `sb` in `0..32`
/// (the subband index) and `j` in `0..64` (the folded phase index) —
/// the forward twin of [`crate::synthesis::matrix_coefficient`].
#[inline]
#[must_use]
pub fn analysis_matrix_coefficient(sb: usize, j: usize) -> f64 {
    debug_assert!(sb < SUBBANDS && j < 64);
    (((2 * sb + 1) as f64) * ((j as f64) - 16.0) * PI / 64.0).cos()
}

/// The persistent 32-band polyphase analysis subband filter.
///
/// Holds the 512-entry input FIFO `X`, zero-initialised at startup
/// (mirroring the synthesis `V` FIFO's startup state). One
/// [`AnalysisFilter`] instance is driven per audio channel across a
/// stream; its `X` state carries the inter-call overlap the windowed
/// sums need.
#[derive(Clone)]
pub struct AnalysisFilter {
    /// The input FIFO. `x[0]` is the most recent PCM sample; older
    /// samples slide up toward `x[511]`.
    x: [f64; WINDOW_LEN],
}

impl Default for AnalysisFilter {
    fn default() -> Self {
        Self::new()
    }
}

impl core::fmt::Debug for AnalysisFilter {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("AnalysisFilter")
            .field("x_len", &self.x.len())
            .finish()
    }
}

impl AnalysisFilter {
    /// A fresh filter with the `X` FIFO zero-initialised.
    #[must_use]
    pub fn new() -> Self {
        Self {
            x: [0.0; WINDOW_LEN],
        }
    }

    /// Run one time slot of the analysis filter: consume the 32 PCM
    /// samples `input` (in playback order — `input[31]` is the newest)
    /// and produce the 32 subband samples for this slot, advancing the
    /// internal `X` FIFO.
    pub fn analyze(&mut self, input: &[f64; SUBBANDS]) -> [f64; SUBBANDS] {
        // Step 1 — shift the FIFO up by 32 and insert the new samples
        // newest-first: X[0] = input[31] … X[31] = input[0].
        self.x.copy_within(0..WINDOW_LEN - SUBBANDS, SUBBANDS);
        for (i, slot) in self.x[..SUBBANDS].iter_mut().enumerate() {
            *slot = input[SUBBANDS - 1 - i];
        }

        // Steps 2 + 3 — window by C and fold the 512 taps onto the 64
        // phases: Y[j] = Σ_{k=0..7} X[j + 64k] · C[j + 64k].
        let mut y = [0.0_f64; 64];
        for (j, yj) in y.iter_mut().enumerate() {
            let mut acc = 0.0_f64;
            for k in 0..8 {
                let idx = j + 64 * k;
                acc += self.x[idx] * analysis_window(idx);
            }
            *yj = acc;
        }

        // Step 4 — matrix the 64 phases into the 32 subband samples.
        let mut s = [0.0_f64; SUBBANDS];
        for (sb, out) in s.iter_mut().enumerate() {
            let mut acc = 0.0_f64;
            for (j, &yj) in y.iter().enumerate() {
                acc += analysis_matrix_coefficient(sb, j) * yj;
            }
            *out = acc;
        }
        s
    }

    /// Reset the `X` FIFO to all-zero (the startup state).
    pub fn reset(&mut self) {
        self.x = [0.0; WINDOW_LEN];
    }
}

/// Run the analysis filterbank over one channel-frame of PCM
/// ([`SAMPLES_PER_FRAME_PER_CHANNEL`] = 1152 samples), producing the
/// frame's [`SubbandMatrix`] — row `b` is subband `b`'s 36 time-ordered
/// subband samples, exactly the orientation the reconstruction /
/// synthesis side consumes.
///
/// `filter` carries the inter-frame FIFO overlap, so the same instance
/// must be reused across consecutive frames of one channel (the forward
/// twin of [`crate::synthesis::synthesize_frame_channel`]).
pub fn analyze_frame_channel(
    filter: &mut AnalysisFilter,
    pcm: &[f64; SAMPLES_PER_FRAME_PER_CHANNEL],
) -> SubbandMatrix {
    debug_assert_eq!(SAMPLES_PER_BAND, 36);
    let mut matrix = zero_subband_matrix();
    for slot in 0..SAMPLES_PER_BAND {
        let mut chunk = [0.0_f64; SUBBANDS];
        chunk.copy_from_slice(&pcm[slot * SUBBANDS..(slot + 1) * SUBBANDS]);
        let s = filter.analyze(&chunk);
        for (band, &v) in s.iter().enumerate() {
            matrix[band][slot] = v;
        }
    }
    matrix
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::synthesis::SynthesisFilter;

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

    /// Push `pcm` through analysis → synthesis and return the output.
    fn round_trip(pcm: &[f64]) -> Vec<f64> {
        assert_eq!(pcm.len() % SUBBANDS, 0);
        let mut a = AnalysisFilter::new();
        let mut s = SynthesisFilter::new();
        let mut out = Vec::with_capacity(pcm.len());
        for chunk in pcm.chunks_exact(SUBBANDS) {
            let mut slot = [0.0_f64; SUBBANDS];
            slot.copy_from_slice(chunk);
            let sub = a.analyze(&slot);
            out.extend_from_slice(&s.synthesize(&sub));
        }
        out
    }

    /// The round trip reconstructs the input delayed by exactly
    /// [`ANALYSIS_SYNTHESIS_DELAY`] samples at unity gain: white-noise
    /// input, reconstruction error better than 80 dB below the signal
    /// (measured ≈ 84 dB — the pair's aliasing-cancellation ripple at
    /// the window's print precision; wrong conventions collapse to
    /// single digits). This pins the analysis convention (window
    /// scale, FIFO order, matrix phase) against the oracle-validated
    /// synthesis.
    #[test]
    fn round_trip_is_unity_gain_at_pinned_delay() {
        let n = 8192;
        let mut rng = Rng(0x1234_5678_9ABC_DEF0);
        let pcm: Vec<f64> = (0..n).map(|_| rng.next() * 30000.0).collect();
        let out = round_trip(&pcm);

        let d = ANALYSIS_SYNTHESIS_DELAY;
        let mut sig = 0.0_f64;
        let mut err = 0.0_f64;
        // Skip the filterbank warm-up (one full 512-tap support) and
        // stop before the un-flushed tail.
        for i in 512..(n - d) {
            let x = pcm[i];
            let y = out[i + d];
            sig += x * x;
            err += (y - x) * (y - x);
        }
        assert!(sig > 0.0);
        let snr_db = 10.0 * (sig / err).log10();
        assert!(
            snr_db > 80.0,
            "analysis→synthesis round-trip SNR {snr_db:.1} dB (want > 80 dB)"
        );
    }

    /// Delay scan: of all candidate lags in 0..1200, the pinned delay
    /// is the unique best-correlation lag and its gain is unity to
    /// three decimals. Guards the delay constant itself.
    #[test]
    fn delay_scan_peaks_at_pinned_constant() {
        let n = 4096;
        let mut rng = Rng(0xDEAD_BEEF_CAFE_1234);
        let pcm: Vec<f64> = (0..n).map(|_| rng.next()).collect();
        let out = round_trip(&pcm);

        let mut best = (0usize, f64::MIN);
        for lag in 0..1200usize {
            let mut corr = 0.0_f64;
            for i in 512..(n - lag.max(600)) {
                corr += pcm[i] * out[i + lag];
            }
            if corr > best.1 {
                best = (lag, corr);
            }
        }
        assert_eq!(
            best.0, ANALYSIS_SYNTHESIS_DELAY,
            "correlation peak must sit at the pinned delay"
        );

        // Gain at the peak: least-squares fit y ≈ g·x.
        let d = ANALYSIS_SYNTHESIS_DELAY;
        let mut num = 0.0_f64;
        let mut den = 0.0_f64;
        for i in 512..(n - d) {
            num += pcm[i] * out[i + d];
            den += pcm[i] * pcm[i];
        }
        let gain = num / den;
        assert!(
            (gain - 1.0).abs() < 1e-3,
            "round-trip gain {gain} must be unity"
        );
    }

    /// A pure sine confined to one subband's frequency range comes out
    /// of the analysis concentrated in that subband: the target band
    /// carries almost all the energy.
    #[test]
    fn sine_concentrates_in_matching_subband() {
        // Subband b spans [b, b+1)·π/32 in normalised frequency; put a
        // tone in the middle of band 3: ω = 3.5·π/32.
        let omega = 3.5 * PI / 32.0;
        let n = 4096;
        let pcm: Vec<f64> = (0..n).map(|i| (omega * i as f64).sin()).collect();

        let mut a = AnalysisFilter::new();
        let mut band_energy = [0.0_f64; SUBBANDS];
        for (c, chunk) in pcm.chunks_exact(SUBBANDS).enumerate() {
            let mut slot = [0.0_f64; SUBBANDS];
            slot.copy_from_slice(chunk);
            let s = a.analyze(&slot);
            if c < 16 {
                continue; // warm-up
            }
            for (b, &v) in s.iter().enumerate() {
                band_energy[b] += v * v;
            }
        }
        let total: f64 = band_energy.iter().sum();
        assert!(
            band_energy[3] / total > 0.99,
            "band 3 must dominate: {:?}",
            &band_energy[..8]
        );
    }

    /// The frame driver fills the matrix in the synthesis-side
    /// orientation: [band][slot], 36 slots of 32 bands, matching a
    /// slot-by-slot manual drive.
    #[test]
    fn frame_driver_matches_manual_slots() {
        let mut rng = Rng(42);
        let mut pcm = [0.0_f64; SAMPLES_PER_FRAME_PER_CHANNEL];
        for v in pcm.iter_mut() {
            *v = rng.next() * 1000.0;
        }
        let mut f1 = AnalysisFilter::new();
        let matrix = analyze_frame_channel(&mut f1, &pcm);

        let mut f2 = AnalysisFilter::new();
        for slot in 0..SAMPLES_PER_BAND {
            let mut chunk = [0.0_f64; SUBBANDS];
            chunk.copy_from_slice(&pcm[slot * SUBBANDS..(slot + 1) * SUBBANDS]);
            let s = f2.analyze(&chunk);
            for (band, &v) in s.iter().enumerate() {
                assert_eq!(matrix[band][slot], v, "band {band} slot {slot}");
            }
        }
    }

    /// Reset returns the FIFO to the startup posture: identical output
    /// for identical input after a reset.
    #[test]
    fn reset_restores_startup_state() {
        let mut rng = Rng(7);
        let mut filter = AnalysisFilter::new();
        let mut first = Vec::new();
        let mut input = [0.0_f64; SUBBANDS];
        for v in input.iter_mut() {
            *v = rng.next();
        }
        first.extend_from_slice(&filter.analyze(&input));
        filter.analyze(&input); // advance state
        filter.reset();
        let again = filter.analyze(&input);
        assert_eq!(first.as_slice(), again.as_slice());
    }
}
