//! SV8 §6.4.2 first-order context model — now grounded.
//!
//! The earlier [`crate::sv8_sample_decode`] context arms
//! ([`decode_sv8_context_band`](crate::sv8_sample_decode::decode_sv8_context_band)
//! for `band_type` 5..=8 and
//! [`decode_sv8_grouped3_band`](crate::sv8_sample_decode::decode_sv8_grouped3_band)
//! for `band_type` 2) took their table-pair selection as a
//! **caller-supplied knob**: §3.4 of the structural spec only said the
//! per-sample table was "chosen by the previously decoded sample" and
//! did not pin the predicate, and the §3.4 case-2 (`q2`) pair-selection
//! rule was wholly unspecified.
//!
//! The staged `spec/musepack-headers-and-coding.md` **§6.4.2** now pins
//! both, as *facts*:
//!
//! - There is a running first-order context accumulator `idx`. Per
//!   `band_type` (`Res`) it is **initialised to `2 × thres[Res]`** and
//!   selects, per sample (cases 5..=8) or per group (case 2), the
//!   **context-1** table when `idx > thres[Res]`, else **context-0**.
//! - The per-`Res` thresholds are: `Res 2 → 3`, `Res 5 → 1`,
//!   `Res 6 → 3`, `Res 7 → 4`, `Res 8 → 8`.
//! - **Cases 5..=8** (one sample per codeword): after decoding each
//!   sample `q`, the accumulator is updated
//!   `idx = (idx >> 1) + |q|` — a decaying sum of recent magnitudes.
//! - **Case 2** (three samples per codeword, base-5 product index
//!   `tmp` in `0..=124`): the update is `idx = (idx >> 1) + var[tmp]`
//!   where `var[tmp]` is the **summed magnitude of the three samples**
//!   that the product index `tmp` encodes (the §6.4.2 "fixed 125-entry
//!   magnitude table"). Because the three samples are the base-5 digits
//!   of `tmp` minus 2 (the same un-bundling SV7 case 2 / SV8 case 2
//!   uses), `var[tmp]` is a *computed* quantity, not a new table:
//!   `var[tmp] = |d0−2| + |d1−2| + |d2−2|` over the three base-5 digits
//!   of `tmp`.
//!
//! Source-of-record (facts only, all under `docs/audio/musepack/`):
//!
//! - `spec/musepack-headers-and-coding.md` §6.4.2 — the `idx` init /
//!   update / table-select rules, the per-`Res` thresholds, and the
//!   case-2 magnitude-table-by-`tmp` rule.
//! - `spec/musepack-headers-and-coding.md` §5.5 / §6.4.2 — the base-5
//!   un-bundling of `tmp` into the three centred samples (also pinned
//!   by [`crate::sv8_sample_decode::unpack_grouped3_symbol`]).
//!
//! These constants and the [`Sv8Context`] accumulator let the SV8
//! sample-decode arms run their *own* table selection from the
//! bitstream, removing the GAP-knob closures from the canonical decode
//! path.

/// Number of distinct `band_type` (`Res`) values that carry a §6.4.2
/// first-order context: `2` and `5..=8`. The threshold lookup
/// [`context_threshold`] returns `None` for any other `band_type`.
pub const CONTEXT_BAND_TYPES: usize = 5;

/// The §6.4.2 per-`Res` context threshold `thres[Res]`.
///
/// Returns the threshold for the context-bearing band types
/// (`Res 2 → 3`, `Res 5 → 1`, `Res 6 → 3`, `Res 7 → 4`, `Res 8 → 8`),
/// or `None` for any `band_type` that does not use a first-order
/// context (the CNS / empty / sparse / grouped-2 / escape arms select
/// their tables without an accumulator).
#[must_use]
pub const fn context_threshold(band_type: i8) -> Option<u32> {
    match band_type {
        2 => Some(3),
        5 => Some(1),
        6 => Some(3),
        7 => Some(4),
        8 => Some(8),
        _ => None,
    }
}

/// The §6.4.2 case-2 magnitude table value `var[tmp]`: the summed
/// magnitude of the three centred samples that the base-5 product
/// index `tmp` (in `0..=124`) encodes.
///
/// The three samples are the base-5 digits of `tmp` each minus 2
/// (`tmp % 5 − 2`, `(tmp / 5) % 5 − 2`, `tmp / 25 − 2`), matching the
/// un-bundling in
/// [`crate::sv8_sample_decode::unpack_grouped3_symbol`]; this returns
/// `|d0| + |d1| + |d2|`. Computed, not tabulated — §6.4.2 calls it a
/// "fixed 125-entry magnitude table" but its entries are exactly this
/// function of `tmp`, so no new numeric table is introduced.
///
/// `tmp` outside `0..=124` returns `0` (defensive; the q2 symbol maps
/// the staged tables ship are exact permutations of `0..=124`, so this
/// is unreachable from a valid stream).
#[must_use]
pub const fn case2_magnitude(tmp: i32) -> u32 {
    if tmp < 0 || tmp > 124 {
        return 0;
    }
    let d0 = (tmp % 5 - 2).unsigned_abs();
    let d1 = (tmp / 5 % 5 - 2).unsigned_abs();
    let d2 = (tmp / 25 - 2).unsigned_abs();
    d0 + d1 + d2
}

/// The §6.4.2 running first-order context accumulator.
///
/// Constructed for a context-bearing `band_type` via [`Self::new`],
/// which initialises `idx = 2 × thres[band_type]`. [`Self::table_ctx`]
/// reports the context index (0 or 1) the *next* decode should use
/// (`idx > thres ? 1 : 0`); after that decode the caller folds in the
/// just-decoded magnitude via [`Self::update_sample`] (cases 5..=8) or
/// [`Self::update_group`] (case 2).
#[derive(Debug, Clone, Copy)]
pub struct Sv8Context {
    /// The running accumulator `idx`.
    idx: u32,
    /// The per-`Res` threshold `thres[Res]` (fixed for this band).
    thres: u32,
}

impl Sv8Context {
    /// Build the §6.4.2 accumulator for `band_type`, initialised to
    /// `idx = 2 × thres[band_type]`.
    ///
    /// Returns `None` for a `band_type` that does not carry a
    /// first-order context (i.e. anything [`context_threshold`] does
    /// not cover).
    #[must_use]
    pub const fn new(band_type: i8) -> Option<Self> {
        match context_threshold(band_type) {
            Some(thres) => Some(Sv8Context {
                idx: thres.wrapping_mul(2),
                thres,
            }),
            None => None,
        }
    }

    /// The context index (0 or 1) for the *next* table decode:
    /// context-1 when `idx > thres`, else context-0 (§6.4.2).
    #[must_use]
    pub const fn table_ctx(&self) -> u8 {
        (self.idx > self.thres) as u8
    }

    /// Fold a just-decoded sample magnitude `|q|` into the accumulator
    /// for the cases-5..=8 path: `idx = (idx >> 1) + |q|` (§6.4.2).
    pub fn update_sample(&mut self, q: i8) {
        self.idx = (self.idx >> 1) + (q as i32).unsigned_abs();
    }

    /// Fold a just-decoded case-2 group's summed magnitude into the
    /// accumulator: `idx = (idx >> 1) + var[tmp]` where `var[tmp]` is
    /// [`case2_magnitude`] of the base-5 product index `tmp` (§6.4.2).
    pub fn update_group(&mut self, tmp: i32) {
        self.idx = (self.idx >> 1) + case2_magnitude(tmp);
    }

    /// The current accumulator value (test / introspection helper).
    #[must_use]
    pub const fn idx(&self) -> u32 {
        self.idx
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn thresholds_match_spec_table() {
        // §6.4.2: Res 2 → 3, 5 → 1, 6 → 3, 7 → 4, 8 → 8.
        assert_eq!(context_threshold(2), Some(3));
        assert_eq!(context_threshold(5), Some(1));
        assert_eq!(context_threshold(6), Some(3));
        assert_eq!(context_threshold(7), Some(4));
        assert_eq!(context_threshold(8), Some(8));
    }

    #[test]
    fn non_context_band_types_have_no_threshold() {
        for bt in [-1_i8, 0, 1, 3, 4, 9, 10, 17] {
            assert_eq!(context_threshold(bt), None, "band_type {bt}");
        }
    }

    #[test]
    fn new_initialises_idx_to_twice_threshold() {
        // §6.4.2: idx init = 2 × thres[Res].
        assert_eq!(Sv8Context::new(2).unwrap().idx(), 6);
        assert_eq!(Sv8Context::new(5).unwrap().idx(), 2);
        assert_eq!(Sv8Context::new(6).unwrap().idx(), 6);
        assert_eq!(Sv8Context::new(7).unwrap().idx(), 8);
        assert_eq!(Sv8Context::new(8).unwrap().idx(), 16);
    }

    #[test]
    fn new_rejects_non_context_band_types() {
        assert!(Sv8Context::new(3).is_none());
        assert!(Sv8Context::new(9).is_none());
    }

    #[test]
    fn initial_table_ctx_is_one_for_every_context_band() {
        // idx init = 2·thres > thres for thres ≥ 1, so the first decode
        // always starts in context-1.
        for bt in [2_i8, 5, 6, 7, 8] {
            assert_eq!(
                Sv8Context::new(bt).unwrap().table_ctx(),
                1,
                "band_type {bt}"
            );
        }
    }

    #[test]
    fn table_ctx_picks_one_above_threshold_zero_at_or_below() {
        let mut c = Sv8Context::new(5).unwrap(); // thres = 1, idx = 2
        assert_eq!(c.table_ctx(), 1); // 2 > 1
        c.update_sample(0); // idx = (2 >> 1) + 0 = 1
        assert_eq!(c.idx(), 1);
        assert_eq!(c.table_ctx(), 0); // 1 > 1 is false
        c.update_sample(3); // idx = (1 >> 1) + 3 = 3
        assert_eq!(c.idx(), 3);
        assert_eq!(c.table_ctx(), 1); // 3 > 1
    }

    #[test]
    fn update_sample_uses_absolute_value() {
        let mut a = Sv8Context::new(6).unwrap(); // idx = 6
        let mut b = Sv8Context::new(6).unwrap();
        a.update_sample(-4);
        b.update_sample(4);
        assert_eq!(a.idx(), b.idx());
        assert_eq!(a.idx(), (6 >> 1) + 4);
    }

    #[test]
    fn case2_magnitude_sums_three_base5_digit_magnitudes() {
        // tmp = 62 = 2·25 + 2·5 + 2 → digits (2,2,2) → samples (0,0,0)
        // → magnitude 0 (the all-zero triplet).
        assert_eq!(case2_magnitude(62), 0);
        // tmp = 0 → digits (0,0,0) → samples (-2,-2,-2) → 2+2+2 = 6.
        assert_eq!(case2_magnitude(0), 6);
        // tmp = 124 → digits (4,4,4) → samples (2,2,2) → 6.
        assert_eq!(case2_magnitude(124), 6);
        // tmp = 63 = 2·25 + 2·5 + 3 → digits (3,2,2) → samples (1,0,0)
        // → 1.
        assert_eq!(case2_magnitude(63), 1);
    }

    #[test]
    fn case2_magnitude_matches_unpacked_triplet_magnitude() {
        // Cross-check against the sample-decode un-bundling for every
        // valid tmp: var[tmp] must equal the summed |sample|.
        use crate::sv8_sample_decode::unpack_grouped3_symbol;
        for tmp in 0..=124_i32 {
            let triplet = unpack_grouped3_symbol(tmp as i8).unwrap();
            let want: u32 = triplet.iter().map(|&s| (s as i32).unsigned_abs()).sum();
            assert_eq!(case2_magnitude(tmp), want, "tmp = {tmp}");
        }
    }

    #[test]
    fn case2_magnitude_out_of_range_is_zero() {
        assert_eq!(case2_magnitude(-1), 0);
        assert_eq!(case2_magnitude(125), 0);
    }

    #[test]
    fn update_group_folds_case2_magnitude() {
        let mut c = Sv8Context::new(2).unwrap(); // idx = 6
        c.update_group(0); // var[0] = 6 → idx = (6 >> 1) + 6 = 9
        assert_eq!(c.idx(), 9);
        assert_eq!(c.table_ctx(), 1); // 9 > 3
        c.update_group(62); // var[62] = 0 → idx = (9 >> 1) + 0 = 4
        assert_eq!(c.idx(), 4);
    }
}
