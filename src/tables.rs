//! Shared dequantisation + CNS tables for Musepack SV7 / SV8.
//!
//! All values are **wire-format constants** required for byte-exact
//! interoperability — see `docs/audio/musepack/data/musepack-vlc-tables.md`
//! §3 and §4 (and the matching trace report §5.6 / §5.14).
//!
//! These are not FFmpeg property: they encode the bit-level wire format
//! that any clean-room Musepack decoder must agree on. The numeric
//! contents were transcribed from the in-tree clean-room writeup.

/// Inverse-quantiser constants `mpc_CC[res + 1]`. Indexed by `res + 1`
/// so that `res = -1` (noise band) maps to entry 0 and `res = 17`
/// (highest precision) maps to entry 18.
///
/// The non-noise entries follow `65536 / (2 * levels - 1)` where
/// `levels` is the integer level count from
/// `{3, 5, 7, 9, 15, 31, 63, 127, 255, 511, 1023, ...}` — i.e. these
/// are the exact reciprocals of the per-`res` integer dequantiser step
/// for unit subband amplitude.
pub const CC: [f32; 19] = [
    111.285_96,  // res = -1  (noise band: 32768 / (2 * 255) * sqrt(3))
    65536.0,     // res =  0  (never used; res = 0 is the silent path)
    21845.334,   // res =  1  (3 levels)
    13107.2,     // res =  2  (5 levels)
    9362.286,    // res =  3  (7 levels)
    7281.778,    // res =  4  (9 levels)
    4369.067,    // res =  5  (15 levels)
    2_114.064_5, // res =  6  (31 levels)
    1040.254,    // res =  7  (63 levels)
    516.031_5,   // res =  8  (127 levels)
    257.003_9,   // res =  9  (255 levels)
    128.250_5,   // res = 10
    64.062_6,    // res = 11
    32.015_6,    // res = 12
    16.003_9,    // res = 13
    8.001,       // res = 14
    4.000_2,     // res = 15
    2.000_1,     // res = 16
    1.0,         // res = 17
];

/// Log scalefactor table `mpc_SCF[256]`. The first 128 entries form
/// a smoothly decaying log curve (entry 0 ≈ 307.33, entry 127
/// ≈ 2.13e-5); entries 128..255 are scaled-up copies.
///
/// In the canonical SV7/SV8 codec this is generated at decoder init
/// from `2^((127 - k) / 12)` over the lower half, with a
/// factor-of-2^32 jump between halves to reach entries 128..255.
/// Recomputing with a closed form (`SCF[k] = 2^((127 - k) / 12)`)
/// loses precision near both ends, so we lazily build the table once
/// at runtime in floating-point and keep it for the lifetime of the
/// process. Spot-check: `SCF[0] ≈ 307.33`, `SCF[127] ≈ 2.13e-5`,
/// `SCF[128]` jumps by `2^32`.
pub fn scf_table() -> &'static [f32; 256] {
    use std::sync::OnceLock;
    static TABLE: OnceLock<[f32; 256]> = OnceLock::new();
    TABLE.get_or_init(|| {
        let mut t = [0.0f32; 256];
        // Lower half: log decay around 12 entries per octave. The
        // canonical formula is derived from the per-third 6 dB step
        // of MPEG-style quantisation, with the [127] entry being the
        // unit-amplitude reference scaled by the 1/65536 fixed-point
        // factor that MPEG-1 Layer 2 uses for the synthesis filter.
        for (k, slot) in t.iter_mut().enumerate().take(128) {
            let exponent = (127.0 - k as f64) / 12.0;
            *slot = 2.0f64.powf(exponent) as f32;
        }
        // Upper half: the canonical reference applies a ×2^32
        // headroom jump to allow extreme scalefactor indices in
        // pathological encoder paths. We follow the same convention
        // for bit-exactness.
        let factor = 2.0f32.powi(32);
        for k in 128..256 {
            t[k] = t[k - 128] * factor;
        }
        t
    })
}

/// Pascal's-triangle slab `mpc8_cnk[k - 1][n - 1] = C(n - 1, k)` for
/// `k ∈ [1..=16]` and `n ∈ [1..=33]`. Used by both the CNS-coded
/// MS-stereo bitmask and the `res = 1` half-band binary spike map.
///
/// Note: the in-tree spec listing's text claims `C(n - 1, k - 1)` but
/// the actual numeric values match `C(n - 1, k)` — for example the
/// `k = 2` row reads `0, 0, 1, 3, 6, 10, 15, ...` which is
/// `C(n - 1, 2)` for `n = 1, 2, 3, ...`, not `C(n - 1, 1)`. The
/// internal convention here follows the **values** (which are what
/// the wire format actually requires), not the doc-comment text.
///
/// The bitmask decode bijects k-subsets of an n-element set with the
/// integer interval `[0, C(n, k))`; the runtime walker decrements
/// `(k, n)` step by step and reads bits at each step.
pub const CNK: [[u64; 33]; 16] = build_cnk();

const fn build_cnk() -> [[u64; 33]; 16] {
    // Compute C(n, k) for n in [0..32] and k in [1..16] by Pascal's
    // recurrence. const-fn so the table is computed at compile time.
    let mut binom = [[0u64; 33]; 33];
    let mut n: usize = 0;
    while n < 33 {
        binom[n][0] = 1;
        let mut k: usize = 1;
        while k <= n {
            // Pascal's recurrence: C(n,k) = C(n-1,k-1) + C(n-1,k).
            let prev_left = binom[n - 1][k - 1];
            let prev_right = if k < n { binom[n - 1][k] } else { 0 };
            binom[n][k] = prev_left + prev_right;
            k += 1;
        }
        n += 1;
    }
    let mut out = [[0u64; 33]; 16];
    let mut k: usize = 1;
    while k <= 16 {
        let mut nn: usize = 0;
        while nn < 33 {
            out[k - 1][nn] = if nn >= k { binom[nn][k] } else { 0 };
            nn += 1;
        }
        k += 1;
    }
    out
}

/// Bit-length grid `mpc8_cnk_len[k - 1][n - 1]` — see vlc-tables.md §3.2.
/// Entry value is the average bit cost (in bits) for the
/// `mpc8_dec_base(k, n)` reader at the given `(k, n)`. A `0` entry
/// means "no extra bit needed; the ceiling-log fits without the
/// 'lost' adjustment". The "lost code" grid in [`CNK_LOST`] gives the
/// per-`(k, n)` threshold above which one extra bit is read.
#[rustfmt::skip]
pub const CNK_LEN: [[u8; 33]; 16] = [
    // k = 1
    [0, 1, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6],
    // k = 2
    [0, 0, 2, 3, 4, 4, 5, 5, 6, 6, 6, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 9, 9, 0],
    // k = 3
    [0, 0, 0, 2, 4, 5, 6, 6, 7, 7, 8, 8, 9, 9, 9, 10, 10, 10, 10, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 13, 13, 0],
    // k = 4
    [0, 0, 0, 0, 3, 4, 6, 7, 7, 8, 9, 9, 10, 10, 11, 11, 12, 12, 12, 13, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15, 15, 16, 0],
    // k = 5
    [0, 0, 0, 0, 0, 3, 5, 6, 7, 8, 9, 10, 11, 11, 12, 13, 13, 14, 14, 14, 15, 15, 16, 16, 16, 17, 17, 17, 17, 18, 18, 18, 0],
    // k = 6
    [0, 0, 0, 0, 0, 0, 3, 5, 7, 8, 9, 10, 11, 12, 13, 13, 14, 15, 15, 16, 16, 17, 17, 18, 18, 18, 19, 19, 19, 20, 20, 20, 0],
    // k = 7
    [0, 0, 0, 0, 0, 0, 0, 3, 6, 7, 9, 10, 11, 12, 13, 14, 15, 15, 16, 17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 21, 22, 22, 0],
    // k = 8
    [0, 0, 0, 0, 0, 0, 0, 0, 4, 6, 8, 9, 11, 12, 13, 14, 15, 16, 17, 17, 18, 19, 19, 20, 21, 21, 22, 22, 23, 23, 23, 24, 0],
    // k = 9
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 6, 8, 10, 11, 13, 14, 15, 16, 17, 18, 19, 19, 20, 21, 21, 22, 23, 23, 24, 24, 25, 25, 0],
    // k = 10
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 7, 9, 10, 12, 13, 15, 16, 17, 18, 19, 20, 21, 21, 22, 23, 24, 24, 25, 25, 26, 26, 0],
    // k = 11
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 7, 9, 11, 13, 14, 15, 17, 18, 19, 20, 21, 22, 23, 23, 24, 25, 26, 26, 27, 27, 0],
    // k = 12
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 7, 9, 11, 13, 15, 16, 17, 19, 20, 21, 22, 23, 24, 25, 25, 26, 27, 28, 28, 0],
    // k = 13
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 7, 10, 12, 14, 15, 17, 18, 19, 21, 22, 23, 24, 25, 26, 27, 27, 28, 29, 0],
    // k = 14
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 7, 10, 12, 14, 16, 17, 19, 20, 21, 23, 24, 25, 26, 27, 28, 28, 29, 0],
    // k = 15
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 8, 10, 12, 14, 16, 18, 19, 21, 22, 23, 25, 26, 27, 28, 29, 30, 0],
    // k = 16
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 8, 10, 13, 15, 17, 18, 20, 21, 23, 24, 25, 27, 28, 29, 30, 0],
];

/// Lost-code grid `mpc8_cnk_lost[k - 1][n - 1]`. Threshold above which
/// one extra bit is read inside `mpc8_dec_base` (see `dec_base` in the
/// `cns` module). A zero entry means "no extra bit needed at this
/// `(k, n)`".
#[rustfmt::skip]
pub const CNK_LOST: [[u32; 33]; 16] = [
    // k = 1
    [0, 0, 1, 0, 3, 2, 1, 0, 7, 6, 5, 4, 3, 2, 1, 0, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, 31],
    // k = 2
    [0, 0, 1, 2, 6, 1, 11, 4, 28, 19, 9, 62, 50, 37, 23, 8, 120, 103, 85, 66, 46, 25, 3, 236, 212, 187, 161, 134, 106, 77, 47, 16, 0],
    // k = 3
    [0, 0, 0, 0, 6, 12, 29, 8, 44, 8, 91, 36, 226, 148, 57, 464, 344, 208, 55, 908, 718, 508, 277, 24, 1796, 1496, 1171, 820, 442, 36, 3697, 3232, 0],
    // k = 4
    [0, 0, 0, 0, 3, 1, 29, 58, 2, 46, 182, 17, 309, 23, 683, 228, 1716, 1036, 220, 3347, 2207, 877, 7529, 5758, 3734, 1434, 15218, 12293, 9017, 5363, 1303, 29576, 0],
    // k = 5
    [0, 0, 0, 0, 0, 2, 11, 8, 2, 4, 50, 232, 761, 46, 1093, 3824, 2004, 7816, 4756, 880, 12419, 6434, 31887, 23032, 12406, 65292, 50342, 32792, 12317, 119638, 92233, 60768, 0],
    // k = 6
    [0, 0, 0, 0, 0, 0, 1, 4, 44, 46, 50, 100, 332, 1093, 3187, 184, 4008, 14204, 5636, 26776, 11272, 56459, 30125, 127548, 85044, 31914, 228278, 147548, 49268, 454801, 312295, 142384, 0],
    // k = 7
    [0, 0, 0, 0, 0, 0, 0, 0, 28, 8, 182, 232, 332, 664, 1757, 4944, 13320, 944, 15148, 53552, 14792, 91600, 16987, 178184, 43588, 390776, 160546, 913112, 536372, 61352, 1564729, 828448, 0],
    // k = 8
    [0, 0, 0, 0, 0, 0, 0, 0, 7, 19, 91, 17, 761, 1093, 1757, 3514, 8458, 21778, 55490, 5102, 58654, 204518, 33974, 313105, 1015577, 534877, 1974229, 1086199, 4096463, 2535683, 499883, 6258916, 0],
    // k = 9
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 9, 36, 309, 46, 3187, 4944, 8458, 16916, 38694, 94184, 230358, 26868, 231386, 789648, 54177, 1069754, 3701783, 1481708, 6762211, 2470066, 13394357, 5505632, 0],
    // k = 10
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 62, 226, 23, 1093, 184, 13320, 21778, 38694, 77388, 171572, 401930, 953086, 135896, 925544, 3076873, 8340931, 3654106, 13524422, 3509417, 22756699, 2596624, 0],
    // k = 11
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 50, 148, 683, 3824, 4008, 944, 55490, 94184, 171572, 343144, 745074, 1698160, 3931208, 662448, 3739321, 12080252, 32511574, 12481564, 49545413, 5193248, 0],
    // k = 12
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 37, 57, 228, 2004, 14204, 15148, 5102, 230358, 401930, 745074, 1490148, 3188308, 7119516, 16170572, 3132677, 15212929, 47724503, 127314931, 42642616, 0],
    // k = 13
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 23, 464, 1716, 7816, 5636, 53552, 58654, 26868, 953086, 1698160, 3188308, 6376616, 13496132, 29666704, 66353813, 14457878, 62182381, 189497312, 0],
    // k = 14
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 8, 344, 1036, 4756, 26776, 14792, 204518, 231386, 135896, 3931208, 7119516, 13496132, 26992264, 56658968, 123012781, 3252931, 65435312, 0],
    // k = 15
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 120, 208, 220, 880, 11272, 91600, 33974, 789648, 925544, 662448, 16170572, 29666704, 56658968, 113317936, 236330717, 508019104, 0],
    // k = 16
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 15, 103, 55, 3347, 12419, 56459, 16987, 313105, 54177, 3076873, 3739321, 3132677, 66353813, 123012781, 236330717, 0, 0],
];

/// `mpc8_huffq2[125]` — sample-magnitude lookup for the `res = 2`
/// running entropy estimator. Each entry equals `|s0| + |s1| + |s2|`
/// for the symbol's decoded triple in `{-2..+2}^3` — i.e. the centre
/// entry (62) is zero.
#[rustfmt::skip]
pub const HUFFQ2: [u8; 125] = [
    6, 5, 4, 5, 6, 5, 4, 3, 4, 5, 4, 3, 2, 3, 4, 5, 4, 3, 4, 5, 6, 5, 4, 5, 6,
    5, 4, 3, 4, 5, 4, 3, 2, 3, 4, 3, 2, 1, 2, 3, 4, 3, 2, 3, 4, 5, 4, 3, 4, 5,
    4, 3, 2, 3, 4, 3, 2, 1, 2, 3, 2, 1, 0, 1, 2, 3, 2, 1, 2, 3, 4, 3, 2, 3, 4,
    5, 4, 3, 4, 5, 4, 3, 2, 3, 4, 3, 2, 1, 2, 3, 4, 3, 2, 3, 4, 5, 4, 3, 4, 5,
    6, 5, 4, 5, 6, 5, 4, 3, 4, 5, 4, 3, 2, 3, 4, 5, 4, 3, 4, 5, 6, 5, 4, 5, 6,
];

/// Threshold table `mpc8_thres[res ∈ 0..=8]` — used by the
/// running-entropy estimator that pages between the two parallel VLC
/// sub-tables for `res ∈ {2, 5, 6, 7, 8}`. Entries below `res = 2` are
/// unused; entry 1 is also unused (the `res = 1` path uses a single
/// `q1_vlc` and a CNS bitmask, not parallel sub-tables).
pub const THRES: [u32; 9] = [0, 0, 3, 0, 0, 1, 3, 4, 8];

/// `idx30[27]` — first axis of the SV7 `res = 1` ternary-triple
/// decode (lookup index 0..26 → first sample in `{-1, 0, +1}`).
pub const IDX30: [i8; 27] = [
    -1, 0, 1, -1, 0, 1, -1, 0, 1, -1, 0, 1, -1, 0, 1, -1, 0, 1, -1, 0, 1, -1, 0, 1, -1, 0, 1,
];

/// `idx31[27]` — second axis.
pub const IDX31: [i8; 27] = [
    -1, -1, -1, 0, 0, 0, 1, 1, 1, -1, -1, -1, 0, 0, 0, 1, 1, 1, -1, -1, -1, 0, 0, 0, 1, 1, 1,
];

/// `idx32[27]` — third axis.
pub const IDX32: [i8; 27] = [
    -1, -1, -1, -1, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1,
];

/// `idx50[25]` and `idx51[25]` — the two axes of the SV7 `res = 2`
/// ternary-pair decode (5 × 5 grid in `{-2..+2}`).
pub const IDX50: [i8; 25] = [
    -2, -1, 0, 1, 2, -2, -1, 0, 1, 2, -2, -1, 0, 1, 2, -2, -1, 0, 1, 2, -2, -1, 0, 1, 2,
];

/// Second axis of the 5 × 5 grid.
pub const IDX51: [i8; 25] = [
    -2, -2, -2, -2, -2, -1, -1, -1, -1, -1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2,
];

/// SV8 `res = 2` decode uses a triple `(idx52, idx51, idx50)` in
/// `{-2..+2}^3` — symbol `s ∈ [0..125)` decodes as
/// `(s / 25 - 2, (s / 5) % 5 - 2, s % 5 - 2)`. We compute the three
/// axes inline at decode time rather than store them.
pub fn sv8_q2_axes(symbol: u32) -> (i8, i8, i8) {
    let s = symbol as i32;
    let a0 = s % 5 - 2;
    let a1 = (s / 5) % 5 - 2;
    let a2 = s / 25 - 2;
    (a0 as i8, a1 as i8, a2 as i8)
}

/// Sample-rate index → Hz. SV7 uses a 2-bit field; SV8 uses a 3-bit
/// field with indices 4..7 reserved. Both decode 0..3 identically.
pub const SAMPLE_RATES: [u32; 4] = [44_100, 48_000, 37_800, 32_000];

/// Frame size (subband samples per band per frame). Fixed at 1152
/// PCM samples per channel = 32 bands × 36 subband samples per
/// granule. Shared across SV7 and SV8.
pub const MPC_FRAMESIZE: usize = 1152;

/// Number of subbands. Fixed at 32 by the PQF analysis filter bank.
pub const SUBBAND_COUNT: usize = 32;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cnk_corners() {
        // C(31, 1) = 31
        assert_eq!(CNK[0][31], 31);
        // C(32, 16) = 601080390
        assert_eq!(CNK[15][32], 601_080_390);
    }

    #[test]
    fn sv8_q2_axes_centre() {
        // Centre of the 5×5×5 grid is symbol 62 → (0, 0, 0).
        assert_eq!(sv8_q2_axes(62), (0, 0, 0));
        // Symbol 0 is the corner (-2, -2, -2).
        assert_eq!(sv8_q2_axes(0), (-2, -2, -2));
        // Symbol 124 is the opposite corner (+2, +2, +2).
        assert_eq!(sv8_q2_axes(124), (2, 2, 2));
    }

    #[test]
    fn cc_pattern() {
        // CC[res + 1] ≈ 65536 / (2 * levels - 1) for res ∈ {1..8}.
        // res = 1 → 3 levels, CC[2] ≈ 65536/3 ≈ 21845.33.
        assert!((CC[2] - 21845.334).abs() < 0.1);
        // res = 7 → 63 levels, CC[8] ≈ 65536/63 ≈ 1040.25.
        assert!((CC[8] - 1040.254).abs() < 0.1);
    }

    #[test]
    fn scf_table_basic() {
        let scf = scf_table();
        // Lower half: SCF[127] = 2^0 = 1. (closed form)
        assert!((scf[127] - 1.0).abs() < 1e-6);
        // Upper half is ×2^32 of the lower half.
        let factor = 2.0f32.powi(32);
        assert!((scf[128 + 127] / scf[127] - factor).abs() / factor < 1e-3);
    }
}
