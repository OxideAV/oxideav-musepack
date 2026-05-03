//! 32-band polyphase synthesis filter bank, shared by Musepack SV7 and
//! SV8.
//!
//! This is the standard MPEG-1 Audio synthesis filter (ISO/IEC 11172-3
//! Annex 3-A.1, Table B.3) — Musepack's `mpcdata.h` reuses
//! `ff_mpa_enwindow[257]` from MPEG audio with no modification, see the
//! trace report §4.3 / §5 / sidecar §5.
//!
//! Each invocation consumes 32 subband samples and produces 32 PCM
//! output samples while carrying a 1024-sample FIFO `v` across calls.
//! Frame layout: 1152 PCM samples = 32 bands × 36 subband samples (the
//! Musepack "subframe" boundary divides each subband into three groups
//! of 12).
//!
//! The 257 stored values below are the upper half of the 512-tap
//! symmetric prototype filter — i.e. they are numeric constants from
//! ISO/IEC 11172-3, not copyrightable, and the same values appear
//! verbatim in `dist10` (public-domain), `libmpg123` (LGPL), and every
//! other MPEG-1 Audio implementation.

use std::sync::OnceLock;

/// Per-channel synthesis state. The 1024-sample FIFO `v` accumulates
/// matrixed subband samples; it must persist across frames within a
/// single Musepack stream (one state per channel).
pub struct SynthesisState {
    v: Box<[f32; 1024]>,
}

impl Default for SynthesisState {
    fn default() -> Self {
        Self {
            v: Box::new([0.0; 1024]),
        }
    }
}

impl SynthesisState {
    pub fn new() -> Self {
        Self::default()
    }

    /// Feed 32 subband samples → emit 32 PCM samples (`f32`, in
    /// roughly `[-1.0, 1.0]`). Caller multiplies by 32767 and clamps
    /// to `i16` for s16 output.
    pub fn synthesize(&mut self, subbands: &[f32; 32], out: &mut [f32; 32]) {
        // 1. Shift the FIFO down by 64 (drop the oldest 64 entries,
        // make room at the head).
        for i in (64..1024).rev() {
            self.v[i] = self.v[i - 64];
        }

        // 2. Matrix step: v[0..64] = N · subbands[0..32], where
        // N[i][k] = cos((2k+1)(16+i) * π / 64).
        let m = matrix();
        for i in 0..64 {
            let mut acc = 0.0f32;
            for k in 0..32 {
                acc += m[i][k] * subbands[k];
            }
            self.v[i] = acc;
        }

        // 3. Build the 512-sample u[] tap layout from the FIFO.
        let mut u = [0.0f32; 512];
        for i in 0..8 {
            for j in 0..32 {
                u[64 * i + j] = self.v[128 * i + j];
                u[64 * i + 32 + j] = self.v[128 * i + 96 + j];
            }
        }

        // 4. Window with the standard MPEG-1 Audio synthesis window.
        for i in 0..512 {
            u[i] *= SYNTH_WINDOW_D[i];
        }

        // 5. Sum 16 windowed taps per output sample.
        for j in 0..32 {
            let mut acc = 0.0f32;
            for i in 0..16 {
                acc += u[32 * i + j];
            }
            out[j] = acc;
        }
    }
}

static MATRIX_STORAGE: OnceLock<[[f32; 32]; 64]> = OnceLock::new();

fn matrix() -> &'static [[f32; 32]; 64] {
    MATRIX_STORAGE.get_or_init(|| {
        let mut m = [[0.0f32; 32]; 64];
        let pi = std::f64::consts::PI;
        for i in 0..64 {
            for k in 0..32 {
                let angle = ((2 * k + 1) as f64) * ((16 + i) as f64) * pi / 64.0;
                m[i][k] = angle.cos() as f32;
            }
        }
        m
    })
}

/// 257-tap upper half of the 512-tap symmetric MPEG-1 Audio prototype
/// filter `D[]` (`ff_mpa_enwindow` in FFmpeg's terminology). Q15 fixed
/// point — divide by 65536 to get the floating-point tap value.
///
/// Reference: trace sidecar `data/musepack-vlc-tables.md` §5.1; ISO/IEC
/// 11172-3 Table 3-B.3.
#[rustfmt::skip]
pub const ENWINDOW: [i32; 257] = [
       0,    -1,    -1,    -1,    -1,    -1,    -1,    -2,
      -2,    -2,    -2,    -3,    -3,    -4,    -4,    -5,
      -5,    -6,    -7,    -7,    -8,    -9,   -10,   -11,
     -13,   -14,   -16,   -17,   -19,   -21,   -24,   -26,
     -29,   -31,   -35,   -38,   -41,   -45,   -49,   -53,
     -58,   -63,   -68,   -73,   -79,   -85,   -91,   -97,
    -104,  -111,  -117,  -125,  -132,  -139,  -147,  -154,
    -161,  -169,  -176,  -183,  -190,  -196,  -202,  -208,
     213,   218,   222,   225,   227,   228,   228,   227,
     224,   221,   215,   208,   200,   189,   177,   163,
     146,   127,   106,    83,    57,    29,    -2,   -36,
     -72,  -111,  -153,  -197,  -244,  -294,  -347,  -401,
    -459,  -519,  -581,  -645,  -711,  -779,  -848,  -919,
    -991, -1064, -1137, -1210, -1283, -1356, -1428, -1498,
   -1567, -1634, -1698, -1759, -1817, -1870, -1919, -1962,
   -2001, -2032, -2057, -2075, -2085, -2087, -2080, -2063,
    2037,  2000,  1952,  1893,  1822,  1739,  1644,  1535,
    1414,  1280,  1131,   970,   794,   605,   402,   185,
     -45,  -288,  -545,  -814, -1095, -1388, -1692, -2006,
   -2330, -2663, -3004, -3351, -3705, -4063, -4425, -4788,
   -5153, -5517, -5879, -6237, -6589, -6935, -7271, -7597,
   -7910, -8209, -8491, -8755, -8998, -9219, -9416, -9585,
   -9727, -9838, -9916, -9959, -9966, -9935, -9863, -9750,
   -9592, -9389, -9139, -8840, -8492, -8092, -7640, -7134,
    6574,  5959,  5288,  4561,  3776,  2935,  2037,  1082,
      70,  -998, -2122, -3300, -4533, -5818, -7154, -8540,
   -9975,-11455,-12980,-14548,-16155,-17799,-19478,-21189,
  -22929,-24694,-26482,-28289,-30112,-31947,-33791,-35640,
  -37489,-39336,-41176,-43006,-44821,-46617,-48390,-50137,
  -51853,-53534,-55178,-56778,-58333,-59838,-61289,-62684,
  -64019,-65290,-66494,-67629,-68692,-69679,-70590,-71420,
  -72169,-72835,-73415,-73908,-74313,-74630,-74856,-74992,
   75038,
];

/// Unfolded 512-tap synthesis window, in floating point. Built lazily
/// from [`ENWINDOW`] at first call. Index `i ∈ [0..512)`.
pub fn synth_window_d() -> &'static [f32; 512] {
    static WINDOW: OnceLock<[f32; 512]> = OnceLock::new();
    WINDOW.get_or_init(|| {
        let mut w = [0.0f32; 512];
        // Upper half: i ∈ [0..256] from ENWINDOW[i] / 65536.
        for i in 0..=256 {
            w[i] = ENWINDOW[i] as f32 / 65536.0;
        }
        // Mirror with sign flips per the unfold rule from
        // trace sidecar §5.2: window[512 - i] = ±enwindow[i] with
        // sign = +1 if (i & 63) == 0 else -1, for i ∈ [1..256].
        for i in 1..256 {
            let val = ENWINDOW[i] as f32 / 65536.0;
            let sign = if (i & 63) == 0 { 1.0 } else { -1.0 };
            w[512 - i] = sign * val;
        }
        w
    })
}

/// Convenience alias matching the `synthesize` window field name. We
/// can't have `static const` slices indexed lazily in Rust without a
/// `OnceLock` + accessor, so this just delegates to
/// [`synth_window_d`] above.
pub static SYNTH_WINDOW_D: SynthWindowD = SynthWindowD;

/// Type-erased indexer for the lazily-built synthesis window. Acts
/// like `&'static [f32; 512]` for the purposes of `[i]` indexing.
pub struct SynthWindowD;

impl std::ops::Index<usize> for SynthWindowD {
    type Output = f32;
    fn index(&self, i: usize) -> &Self::Output {
        &synth_window_d()[i]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn synth_window_spot_check() {
        let w = synth_window_d();
        // ENWINDOW[0] = 0 → w[0] = 0.0.
        assert!(w[0].abs() < 1e-7);
        // ENWINDOW[64] = +213/65536 ≈ 0.00325 (the first positive
        // entry — see vlc-tables.md §5.1).
        let expected = 213.0 / 65536.0;
        assert!((w[64] - expected).abs() < 1e-6, "w[64] = {}", w[64]);
    }

    #[test]
    fn synth_zero_input_zero_output() {
        let mut state = SynthesisState::new();
        let sb = [0.0f32; 32];
        let mut out = [0.0f32; 32];
        for _ in 0..40 {
            state.synthesize(&sb, &mut out);
        }
        for v in out {
            assert!(v.abs() < 1e-6, "non-zero output for zero input: {v}");
        }
    }
}
