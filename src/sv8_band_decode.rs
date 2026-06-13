//! SV8 per-band sample-decode case classifier (frame-body inner
//! loop, §3.4 case ladder).
//!
//! Pure structural dispatch over the spec §3.4 `switch (band_type)`
//! block reproduced verbatim below for traceability. The module
//! exposes one enum + one `const fn` classifier and nothing else;
//! the actual SV8 per-band sample decode (sparse-band flag VLC,
//! grouped-codeword sample unpack, first-order context table
//! selection, large-coefficient escape raw-bit count) lives
//! downstream once the SV8 canonical-Huffman entropy layer is wired
//! against `docs/audio/musepack/tables/sv8-canonical-*` +
//! `sv8-symbols-*`.
//!
//! Source-of-record (structural prose only):
//!
//! - `docs/audio/musepack/musepack-sv7-sv8-spec.md` §3.4
//!   (Audio-packet frame body — quantiser), the case-ladder block
//!   reproduced here for traceability:
//!
//!   ```text
//!   for each non-zero band {
//!     switch (band_type) {
//!       case -1:  fill all 36 samples with random values        # noise substitution
//!       case  0:  do nothing                                     # empty band
//!       case  1:  read one VLC carrying flags for 18 samples;    # sparse / 2% case:
//!                 for each set flag, read 1 bit to fill a sample #   mostly-zero band
//!       case  2:  read 12 VLCs to produce the 36 samples         # 3 samples / codeword
//!       case  3..4: read 18 VLCs to produce the 36 samples       # 2 samples / codeword
//!       case  5..8: for each sample, read a VLC whose table is    # context-adaptive:
//!                 chosen by the previously decoded sample          #   table = f(prev sample)
//!       default:  for each sample, read a VLC plus a fixed number  # large-coefficient escape
//!                 of raw bits
//!     }
//!   }
//!   ```
//!
//! The dispatch shape mirrors [`crate::sv7_band_decode::BandDecodeCase`]
//! one-for-one for the SV7-shared cases (`Cns`, `Empty`,
//! `OutOfRange`) and diverges on the SV8-specific cases:
//!
//! - SV7 §2.5 `case 1` (12 VLCs / 3 samples each) maps to SV8 §3.4
//!   `case 2` — the SV8 case ladder shifts the grouped cases up by
//!   one and inserts the `case 1` sparse-band path in front of them.
//! - SV7 §2.5 `case 2` (18 VLCs / 2 samples each) maps to SV8 §3.4
//!   `case 3..4`.
//! - SV7 §2.5 `case 3..=7` (one VLC per sample, table = Q`band_type`)
//!   maps to SV8 §3.4 `case 5..=8`, where the table choice is also a
//!   first-order context of the previously decoded sample
//!   (highlighted in §3.4 as the heart of the "2% smaller files /
//!   faster decoding" Wikipedia note S2).
//! - SV7 §2.5 `case 8..=17` (linear-PCM escape, `band_type - 1` raw
//!   bits per sample with the exact bit width pinned in
//!   `requant-res-bits`) maps to SV8 §3.4 `default` (large-
//!   coefficient escape: one VLC per sample plus a fixed number of
//!   raw bits). The raw-bit count for the SV8 escape path is a
//!   §3.4 GAP — the structural prose names the shape, the literal
//!   count is in the still-DOCS-GAP §3.4 entropy tables.
//!
//! ## Scope strictly limited to structural classification
//!
//! Every consumer of this module reads bytes downstream through a
//! separate path. The classifier exists so a caller can pattern-match
//! the §3.4 `switch` arm a band falls into in pure code — useful for
//! follow-up rounds that wire the per-case sample decoders one at a
//! time as the SV8 canonical-Huffman layer comes online.
//!
//! Cross-references:
//!
//! - SV7 sibling: [`crate::sv7_band_decode::band_type_case`].
//! - SV8 entropy tables: `docs/audio/musepack/tables/sv8-canonical-*`
//!   (length tables) + `sv8-symbols-*` (symbol maps), staged but
//!   currently GAP in the structural prose's "tables GAP" §4 caveat.

/// SV8 §3.4 per-band sample-decode case classifier.
///
/// One variant per `switch (band_type)` arm in the §3.4 case ladder
/// reproduced in the module-level docs. The `default` branch (any
/// `band_type` `>= 9`) is represented as
/// [`Sv8BandDecodeCase::LargeCoeffEscape`]; values below `-1` (the
/// `band_type == -1` CNS lower bound) fall into
/// [`Sv8BandDecodeCase::OutOfRange`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Sv8BandDecodeCase {
    /// `band_type == -1`: CNS / noise substitution.
    /// 36 samples filled with random values per spec §3.4 case `-1`.
    /// Shape-identical to the SV7 §2.5 `case -1` path.
    Cns,

    /// `band_type == 0`: empty band ("do nothing" per spec §3.4
    /// case `0`). Shape-identical to the SV7 §2.5 `case 0` path.
    Empty,

    /// `band_type == 1`: sparse band — one VLC carries flags for 18
    /// samples, then for each set flag a 1-bit sample is read per
    /// spec §3.4 case `1`. SV8-specific path with no SV7 sibling
    /// (the SV7 case ladder routes a similar near-silent band
    /// through `case 1` grouped-3 instead).
    SparseBand,

    /// `band_type == 2`: grouped, 3 samples per Huffman codeword
    /// (12 VLCs → 36 samples) per spec §3.4 case `2`. Maps to the
    /// SV7 §2.5 `case 1` (`Grouped3`) shape, shifted by one because
    /// the SV8 ladder inserts the sparse-band case in front.
    Grouped3,

    /// `band_type == 3` or `band_type == 4`: grouped, 2 samples per
    /// Huffman codeword (18 VLCs → 36 samples) per spec §3.4 case
    /// `3..4`. Maps to the SV7 §2.5 `case 2` (`Grouped2`) shape,
    /// here split across two `band_type` values so the §3.4 entropy
    /// layer can pick a per-`band_type` table.
    Grouped2,

    /// `band_type` in `5..=8`: one VLC per sample, table chosen by
    /// the *previously decoded* sample's magnitude — first-order
    /// context model per spec §3.4 case `5..8`. This is the SV8-
    /// specific entropy refinement highlighted in S2 ("highly
    /// optimized canonical huffman tables that yields 2% smaller
    /// files and faster decoding"); the SV7 equivalent
    /// (`HuffmanPerSample`, cases 3..=7) is context-paired in a
    /// `[2][N]` table layout rather than driven by the previous
    /// sample.
    ContextHuffmanPerSample,

    /// `band_type >= 9`: large-coefficient escape — one VLC per
    /// sample plus a fixed number of raw bits per spec §3.4
    /// `default` arm. The SV7 sibling is the linear-PCM escape
    /// ladder (`PcmEscape`, cases 8..=17, `band_type - 1` raw bits
    /// per sample); the SV8 escape mixes a per-sample VLC with the
    /// raw bits and the exact bit count is a §3.4 GAP (one of the
    /// still-staged `sv8-canonical-*` / `sv8-symbols-*` table-driven
    /// constants).
    LargeCoeffEscape,

    /// `band_type` below the §3.4 enumerated lower bound (`-1`).
    /// The §3.4 prose covers `-1`, `0`, `1`, `2`, `3..4`, `5..8`, and
    /// the `default` (`>= 9`) arm; this variant carries every value
    /// below `-1` so the classifier surface can stay total without
    /// silently routing a malformed band into one of the wired
    /// arms.
    OutOfRange,
}

/// Classify a `band_type` per the §3.4 case ladder. Pure structural
/// dispatch over the `band_type` integer alone; the function runs
/// without consulting bit-stream state and is infallible.
///
/// The function is `const`, so the result is available at the
/// call-site as a compile-time constant for static-shape work
/// (matching SV7 sibling [`crate::sv7_band_decode::band_type_case`]).
pub const fn sv8_band_type_case(band_type: i8) -> Sv8BandDecodeCase {
    match band_type {
        -1 => Sv8BandDecodeCase::Cns,
        0 => Sv8BandDecodeCase::Empty,
        1 => Sv8BandDecodeCase::SparseBand,
        2 => Sv8BandDecodeCase::Grouped3,
        3 | 4 => Sv8BandDecodeCase::Grouped2,
        5..=8 => Sv8BandDecodeCase::ContextHuffmanPerSample,
        v if v >= 9 => Sv8BandDecodeCase::LargeCoeffEscape,
        _ => Sv8BandDecodeCase::OutOfRange,
    }
}

/// True if the case represents a band that emits any samples — i.e.
/// the §3.4 outer loop's "for each non-zero band" predicate fires.
/// The §3.4 prose explicitly excludes `case 0` ("do nothing") from
/// the inner decode; every other arm contributes 36 samples to the
/// band.
///
/// The [`Sv8BandDecodeCase::OutOfRange`] variant returns `false` —
/// an unrepresented `band_type` is treated as the safe "skip" path
/// at the classifier layer; a dispatcher built on top can promote
/// it to a hard error if it prefers fail-loud over fail-quiet.
pub const fn case_emits_samples(case: Sv8BandDecodeCase) -> bool {
    match case {
        Sv8BandDecodeCase::Empty | Sv8BandDecodeCase::OutOfRange => false,
        Sv8BandDecodeCase::Cns
        | Sv8BandDecodeCase::SparseBand
        | Sv8BandDecodeCase::Grouped3
        | Sv8BandDecodeCase::Grouped2
        | Sv8BandDecodeCase::ContextHuffmanPerSample
        | Sv8BandDecodeCase::LargeCoeffEscape => true,
    }
}

/// True if the case is the SV8-only first-order context-modelled
/// per-sample Huffman path (`band_type` in `5..=8`). Useful for
/// follow-up rounds that wire the table-selection helper separately
/// from the per-sample read loop.
pub const fn case_uses_first_order_context(case: Sv8BandDecodeCase) -> bool {
    matches!(case, Sv8BandDecodeCase::ContextHuffmanPerSample)
}

use crate::cns::CnsPrng;
use crate::huffman::Sv7BitReader;
use crate::sv7_band_decode::{fill_cns_band, fill_zero_band, SAMPLES_PER_BAND};
use crate::sv8_sample_decode::{
    decode_sv8_context_band, decode_sv8_escape_band, decode_sv8_grouped2_band,
    decode_sv8_grouped3_band,
};
use crate::{Error, Result};

/// Decode one SV8 band's 36 samples by routing `band_type` through
/// [`sv8_band_type_case`] to the matching §3.4 arm, writing the
/// resulting centred levels into `out` (always `[i32; 36]`).
///
/// This is the single classifier-driven entry point over the
/// per-arm decoders that landed in earlier rounds
/// ([`crate::sv8_sample_decode`]) and the SV7-shared CNS / empty
/// fills ([`fill_cns_band`] / [`fill_zero_band`]). It exists so a
/// frame-body walker can decode a band from its `band_type` alone
/// without re-deriving the §3.4 `switch` arm at every call site.
///
/// # Output width
///
/// The earlier per-arm decoders split their output type by case: the
/// grouped / context arms emit `[i8; 36]` centred levels, while the
/// large-coefficient escape arm emits `[i32; 36]` (its levels exceed
/// `i8` for every escape `band_type`). This dispatcher unifies on
/// `[i32; 36]` — the widest of the two — and widens the `i8` arms'
/// output with a loss-free `as i32`. CNS / empty already produce
/// `[i32; 36]` natively. The widening is exact: every `i8` level
/// round-trips through `i32` unchanged.
///
/// # Context knobs
///
/// The §3.4 grouped (`case 2`, `case 3..4`) and context (`case 5..8`)
/// arms take context parameters whose selection predicate the staged
/// material does not pin (see [`crate::sv8_sample_decode`]'s
/// "Conventions" note). The dispatcher threads them through verbatim:
///
/// - `grouped_ctx` — the half-of-pair pick for the `case 2` table
///   ([`decode_sv8_grouped3_band`]); `case 3..4` selects its table
///   by `band_type`, not `ctx`, so this knob does not reach it.
/// - `initial_ctx` + `ctx_for_prev` — the first-sample context and
///   the previous-sample → context predicate for the `case 5..8`
///   first-order context arm ([`decode_sv8_context_band`]).
///
/// These are caller knobs by construction: the dispatcher makes no
/// choice the staged tables do not already determine.
///
/// # Fail-loud arms
///
/// - **`band_type == 1` (sparse band)** — DOCS-GAP. The §3.4 prose
///   names the shape ("one VLC carrying flags for 18 samples; for
///   each set flag, read 1 bit to fill a sample") but the staged
///   `sv8-symbols-q1` map is a 19-symbol alphabet (`0..=18`) that
///   cannot literally carry an 18-flag bitmap (that needs `2^18`
///   symbols), and the prose covers 18 samples while the band has
///   36. The symbol → flag-pattern mapping and per-band codeword
///   count stay underdetermined by the staged material, so this arm
///   returns [`Error::UnsupportedBandType`] (fail-loud, never
///   silently-wrong) — see [`crate::sv8_sample_decode`]'s
///   "Case NOT implemented" note.
/// - **[`Sv8BandDecodeCase::OutOfRange`]** (`band_type < -1`) — not a
///   §3.4-enumerated arm; returns [`Error::UnsupportedBandType`]
///   (promoting the classifier's fail-quiet `case_emits_samples ==
///   false` to a hard error, the fail-loud option that function's
///   docs call out).
pub fn decode_sv8_band<F>(
    reader: &mut Sv7BitReader<'_>,
    band_type: i8,
    cns: &mut CnsPrng,
    grouped_ctx: u8,
    initial_ctx: u8,
    ctx_for_prev: F,
    out: &mut [i32; SAMPLES_PER_BAND],
) -> Result<()>
where
    F: FnMut(i8) -> u8,
{
    match sv8_band_type_case(band_type) {
        Sv8BandDecodeCase::Cns => {
            fill_cns_band(cns, out);
            Ok(())
        }
        Sv8BandDecodeCase::Empty => {
            fill_zero_band(out);
            Ok(())
        }
        Sv8BandDecodeCase::Grouped3 => {
            let mut tmp = [0_i8; SAMPLES_PER_BAND];
            decode_sv8_grouped3_band(reader, grouped_ctx, &mut tmp)?;
            widen_into(&tmp, out);
            Ok(())
        }
        Sv8BandDecodeCase::Grouped2 => {
            let mut tmp = [0_i8; SAMPLES_PER_BAND];
            decode_sv8_grouped2_band(reader, band_type, &mut tmp)?;
            widen_into(&tmp, out);
            Ok(())
        }
        Sv8BandDecodeCase::ContextHuffmanPerSample => {
            let mut tmp = [0_i8; SAMPLES_PER_BAND];
            decode_sv8_context_band(reader, band_type, initial_ctx, ctx_for_prev, &mut tmp)?;
            widen_into(&tmp, out);
            Ok(())
        }
        Sv8BandDecodeCase::LargeCoeffEscape => decode_sv8_escape_band(reader, band_type, out),
        // Sparse band (case 1) is a DOCS-GAP; OutOfRange is not a
        // §3.4-enumerated arm. Both fail loud rather than emit a
        // silently-wrong band.
        Sv8BandDecodeCase::SparseBand | Sv8BandDecodeCase::OutOfRange => {
            Err(Error::UnsupportedBandType(band_type))
        }
    }
}

/// Loss-free widen of an `[i8; 36]` per-arm result into the
/// dispatcher's unified `[i32; 36]` buffer.
#[inline]
fn widen_into(src: &[i8; SAMPLES_PER_BAND], dst: &mut [i32; SAMPLES_PER_BAND]) {
    for (d, &s) in dst.iter_mut().zip(src.iter()) {
        *d = s as i32;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn classifier_routes_negative_one_to_cns() {
        // §3.4 case -1 — noise substitution.
        assert_eq!(sv8_band_type_case(-1), Sv8BandDecodeCase::Cns);
    }

    #[test]
    fn classifier_routes_zero_to_empty() {
        // §3.4 case 0 — "do nothing".
        assert_eq!(sv8_band_type_case(0), Sv8BandDecodeCase::Empty);
    }

    #[test]
    fn classifier_routes_one_to_sparse_band() {
        // §3.4 case 1 — sparse / 2% case.
        assert_eq!(sv8_band_type_case(1), Sv8BandDecodeCase::SparseBand);
    }

    #[test]
    fn classifier_routes_two_to_grouped_three() {
        // §3.4 case 2 — 3 samples / codeword.
        assert_eq!(sv8_band_type_case(2), Sv8BandDecodeCase::Grouped3);
    }

    #[test]
    fn classifier_routes_three_and_four_to_grouped_two() {
        // §3.4 case 3..4 — 2 samples / codeword.
        assert_eq!(sv8_band_type_case(3), Sv8BandDecodeCase::Grouped2);
        assert_eq!(sv8_band_type_case(4), Sv8BandDecodeCase::Grouped2);
    }

    #[test]
    fn classifier_routes_five_through_eight_to_context_huffman() {
        // §3.4 case 5..8 — first-order context per-sample VLC.
        for bt in 5..=8 {
            assert_eq!(
                sv8_band_type_case(bt),
                Sv8BandDecodeCase::ContextHuffmanPerSample,
                "band_type {bt} should route to ContextHuffmanPerSample",
            );
        }
    }

    #[test]
    fn classifier_routes_nine_and_above_to_large_coeff_escape() {
        // §3.4 `default` arm — large-coefficient escape.
        for bt in 9..=64 {
            assert_eq!(
                sv8_band_type_case(bt),
                Sv8BandDecodeCase::LargeCoeffEscape,
                "band_type {bt} should route to LargeCoeffEscape",
            );
        }
        // i8 upper extreme exercises the saturation edge of the
        // `default` arm.
        assert_eq!(
            sv8_band_type_case(i8::MAX),
            Sv8BandDecodeCase::LargeCoeffEscape,
        );
    }

    #[test]
    fn classifier_routes_below_negative_one_to_out_of_range() {
        // §3.4 enumerates `-1` as the lower bound; anything below
        // falls into the catch-all so a malformed band can be
        // distinguished from a legitimately empty (case 0) band.
        for bt in [-2i8, -10, -100, i8::MIN] {
            assert_eq!(
                sv8_band_type_case(bt),
                Sv8BandDecodeCase::OutOfRange,
                "band_type {bt} should route to OutOfRange",
            );
        }
    }

    #[test]
    fn classifier_total_coverage_over_full_i8_range() {
        // Every `i8` value resolves to exactly one variant — the
        // classifier is total.
        for bt in i8::MIN..=i8::MAX {
            let case = sv8_band_type_case(bt);
            // Result must be one of the enum variants; the `match`
            // above proves the classifier is total.
            let _ = case;
        }
    }

    #[test]
    fn case_emits_samples_distinguishes_empty_and_out_of_range() {
        // §3.4 outer loop says "for each non-zero band"; `Empty`
        // and `OutOfRange` are the only cases that contribute zero
        // samples to the inner decode.
        assert!(!case_emits_samples(Sv8BandDecodeCase::Empty));
        assert!(!case_emits_samples(Sv8BandDecodeCase::OutOfRange));
        for case in [
            Sv8BandDecodeCase::Cns,
            Sv8BandDecodeCase::SparseBand,
            Sv8BandDecodeCase::Grouped3,
            Sv8BandDecodeCase::Grouped2,
            Sv8BandDecodeCase::ContextHuffmanPerSample,
            Sv8BandDecodeCase::LargeCoeffEscape,
        ] {
            assert!(
                case_emits_samples(case),
                "case {case:?} should emit samples",
            );
        }
    }

    #[test]
    fn case_uses_first_order_context_isolates_sv8_specific_path() {
        // The first-order context-model arm is `band_type` in
        // `5..=8` only; every other arm uses a different table-
        // selection rule.
        assert!(case_uses_first_order_context(
            Sv8BandDecodeCase::ContextHuffmanPerSample
        ));
        for case in [
            Sv8BandDecodeCase::Cns,
            Sv8BandDecodeCase::Empty,
            Sv8BandDecodeCase::SparseBand,
            Sv8BandDecodeCase::Grouped3,
            Sv8BandDecodeCase::Grouped2,
            Sv8BandDecodeCase::LargeCoeffEscape,
            Sv8BandDecodeCase::OutOfRange,
        ] {
            assert!(
                !case_uses_first_order_context(case),
                "case {case:?} should report first-order-context = false",
            );
        }
    }

    #[test]
    fn case_uses_first_order_context_matches_band_type_range_five_to_eight() {
        // Cross-check the helper drives off the `band_type` ladder
        // boundary at 5..=8 inclusive.
        for bt in 5..=8 {
            assert!(
                case_uses_first_order_context(sv8_band_type_case(bt)),
                "band_type {bt} should route through the first-order context arm",
            );
        }
        for bt in [-1i8, 0, 1, 2, 3, 4, 9, 10, 16, 17, 64, i8::MAX, i8::MIN] {
            assert!(
                !case_uses_first_order_context(sv8_band_type_case(bt)),
                "band_type {bt} should not route through the first-order context arm",
            );
        }
    }

    #[test]
    fn classifier_is_const_callable() {
        // const-evaluation sanity: the classifier produces a
        // compile-time constant.
        const CASE_NEG_ONE: Sv8BandDecodeCase = sv8_band_type_case(-1);
        const CASE_ZERO: Sv8BandDecodeCase = sv8_band_type_case(0);
        const CASE_ONE: Sv8BandDecodeCase = sv8_band_type_case(1);
        const CASE_FIVE: Sv8BandDecodeCase = sv8_band_type_case(5);
        const CASE_NINE: Sv8BandDecodeCase = sv8_band_type_case(9);
        assert_eq!(CASE_NEG_ONE, Sv8BandDecodeCase::Cns);
        assert_eq!(CASE_ZERO, Sv8BandDecodeCase::Empty);
        assert_eq!(CASE_ONE, Sv8BandDecodeCase::SparseBand);
        assert_eq!(CASE_FIVE, Sv8BandDecodeCase::ContextHuffmanPerSample);
        assert_eq!(CASE_NINE, Sv8BandDecodeCase::LargeCoeffEscape);
    }

    #[test]
    fn classifier_disagrees_with_sv7_sibling_on_grouped_case_indices() {
        // The §3.4 ladder shifts grouped cases up by one relative to
        // the SV7 §2.5 ladder: SV7 case 1 (Grouped3) sits at SV8
        // case 2 (`Grouped3`), SV7 case 2 (Grouped2) sits at SV8
        // case 3..4 (`Grouped2`). The classifiers' outputs must
        // honour the shift.
        use crate::sv7_band_decode::{band_type_case, BandDecodeCase};
        // SV7 case 1 is Grouped3; SV8 case 1 is SparseBand.
        assert_eq!(band_type_case(1), BandDecodeCase::Grouped3);
        assert_eq!(sv8_band_type_case(1), Sv8BandDecodeCase::SparseBand);
        // SV7 case 2 is Grouped2; SV8 case 2 is Grouped3.
        assert_eq!(band_type_case(2), BandDecodeCase::Grouped2);
        assert_eq!(sv8_band_type_case(2), Sv8BandDecodeCase::Grouped3);
        // SV7 case 3..=7 is HuffmanPerSample; SV8 case 3..4 is
        // Grouped2 and 5..8 is ContextHuffmanPerSample.
        assert_eq!(band_type_case(3), BandDecodeCase::HuffmanPerSample);
        assert_eq!(sv8_band_type_case(3), Sv8BandDecodeCase::Grouped2);
        assert_eq!(band_type_case(5), BandDecodeCase::HuffmanPerSample);
        assert_eq!(
            sv8_band_type_case(5),
            Sv8BandDecodeCase::ContextHuffmanPerSample,
        );
    }

    #[test]
    fn classifier_agrees_with_sv7_sibling_on_shared_cns_and_empty_arms() {
        // SV7 and SV8 share the `case -1` (Cns) and `case 0` (Empty)
        // arms verbatim per spec §2.5 + §3.4.
        use crate::sv7_band_decode::{band_type_case, BandDecodeCase};
        assert_eq!(band_type_case(-1), BandDecodeCase::Cns);
        assert_eq!(sv8_band_type_case(-1), Sv8BandDecodeCase::Cns);
        assert_eq!(band_type_case(0), BandDecodeCase::Empty);
        assert_eq!(sv8_band_type_case(0), Sv8BandDecodeCase::Empty);
    }

    #[test]
    fn enum_supports_copy_eq_and_debug() {
        // The classifier surface relies on `Copy` for `const fn`
        // callability and `Eq` / `Debug` for test ergonomics.
        let a = Sv8BandDecodeCase::Cns;
        let b = a; // Copy
        assert_eq!(a, b); // Eq
        let _printed = format!("{a:?}"); // Debug
    }

    // ─── decode_sv8_band dispatcher ─────────────────────────

    use crate::sv8_huffman::{Sv8CanonicalTable, SV8_Q2_1_TABLE, SV8_Q9UP_TABLE};
    use crate::sv8_sample_decode::GROUPED3_CODEWORDS_PER_BAND;

    /// MSB-first left-justified bit packer (mirrors the one in
    /// `sv8_sample_decode`'s tests); push a `length`-bit codeword
    /// from the top of `pattern`, finish flushes + two zero bytes so
    /// `peek16` never starves mid-decode.
    struct BitPacker {
        bytes: Vec<u8>,
        acc: u32,
        nbits: u8,
    }

    impl BitPacker {
        fn new() -> Self {
            BitPacker {
                bytes: Vec::new(),
                acc: 0,
                nbits: 0,
            }
        }

        fn push(&mut self, pattern: u16, length: u8) {
            for i in 0..length {
                let bit = (pattern >> (15 - i)) & 1;
                self.acc = (self.acc << 1) | bit as u32;
                self.nbits += 1;
                if self.nbits == 8 {
                    self.bytes.push(self.acc as u8);
                    self.acc = 0;
                    self.nbits = 0;
                }
            }
        }

        fn push_raw(&mut self, value: u32, length: u8) {
            for i in (0..length).rev() {
                let bit = (value >> i) & 1;
                self.acc = (self.acc << 1) | bit;
                self.nbits += 1;
                if self.nbits == 8 {
                    self.bytes.push(self.acc as u8);
                    self.acc = 0;
                    self.nbits = 0;
                }
            }
        }

        fn finish(mut self) -> Vec<u8> {
            if self.nbits > 0 {
                self.bytes.push((self.acc << (8 - self.nbits)) as u8);
            }
            self.bytes.push(0);
            self.bytes.push(0);
            self.bytes
        }
    }

    /// First length-table row's exact code pattern (low peek bits
    /// zero) — the shortest, most-probable codeword of `table`.
    fn first_row(table: &Sv8CanonicalTable) -> (u16, u8) {
        let e = table.lengths[0];
        (e.code, e.length)
    }

    /// Decode `table`'s shortest codeword once to learn the symbol it
    /// maps to, so the dispatcher tests can assert the routed arm
    /// emitted exactly that symbol per slot.
    fn symbol_of_first_row(table: &Sv8CanonicalTable) -> i8 {
        let (code, len) = first_row(table);
        let mut p = BitPacker::new();
        p.push(code, len);
        let bytes = p.finish();
        let mut reader = Sv7BitReader::new(&bytes);
        table.decode(&mut reader).expect("single-codeword decode")
    }

    /// A no-op `ctx_for_prev` for arms that do not use it.
    fn no_ctx(_prev: i8) -> u8 {
        0
    }

    #[test]
    fn dispatch_cns_matches_direct_prng_walk_and_advances_prng() {
        // case -1 routes to the SV7-shared CNS fill; the dispatcher
        // must produce the same 36 samples as a direct PRNG walk and
        // advance the PRNG identically.
        let mut via_dispatch = [0_i32; SAMPLES_PER_BAND];
        let mut prng_a = CnsPrng::new();
        let mut reader = Sv7BitReader::new(&[0u8; 4]);
        decode_sv8_band(
            &mut reader,
            -1,
            &mut prng_a,
            0,
            0,
            no_ctx,
            &mut via_dispatch,
        )
        .expect("cns dispatch");

        let mut via_direct = [0_i32; SAMPLES_PER_BAND];
        let mut prng_b = CnsPrng::new();
        prng_b.fill_samples(&mut via_direct);
        assert_eq!(via_dispatch, via_direct);
        assert_eq!(prng_a.state(), prng_b.state());
    }

    #[test]
    fn dispatch_empty_zeroes_band_and_reads_no_bits() {
        // case 0 routes to the SV7-shared zero fill; the dispatcher
        // must clear the buffer and consume nothing from the reader.
        let mut out = [42_i32; SAMPLES_PER_BAND];
        let mut prng = CnsPrng::new();
        let bytes = [0xFFu8; 4];
        let mut reader = Sv7BitReader::new(&bytes);
        let before = reader.bits_remaining();
        decode_sv8_band(&mut reader, 0, &mut prng, 0, 0, no_ctx, &mut out).expect("empty dispatch");
        assert!(out.iter().all(|&s| s == 0));
        assert_eq!(reader.bits_remaining(), before, "empty band reads no bits");
    }

    #[test]
    fn dispatch_grouped3_routes_to_case_two_and_widens_to_i32() {
        // case 2 routes to decode_sv8_grouped3_band: 12 codewords ×
        // 3 samples. Feed 12 copies of the q2 ctx-0 shortest codeword
        // (the all-zero base-5 triplet 62 → [0,0,0]) and confirm the
        // dispatcher widens the i8 arm's output into the i32 buffer
        // identically to the direct arm.
        let (code, len) = first_row(&SV8_Q2_1_TABLE);
        let mut p = BitPacker::new();
        for _ in 0..GROUPED3_CODEWORDS_PER_BAND {
            p.push(code, len);
        }
        let bytes = p.finish();

        // Direct arm (i8 output) as the oracle.
        let mut direct = [7_i8; SAMPLES_PER_BAND];
        {
            let mut reader = Sv7BitReader::new(&bytes);
            crate::sv8_sample_decode::decode_sv8_grouped3_band(&mut reader, 0, &mut direct)
                .expect("direct grouped3");
        }

        let mut out = [0_i32; SAMPLES_PER_BAND];
        let mut prng = CnsPrng::new();
        let mut reader = Sv7BitReader::new(&bytes);
        decode_sv8_band(&mut reader, 2, &mut prng, 0, 0, no_ctx, &mut out)
            .expect("grouped3 dispatch");
        for (i, (&d, &o)) in direct.iter().zip(out.iter()).enumerate() {
            assert_eq!(o, d as i32, "sample {i} must match widened direct arm");
        }
    }

    #[test]
    fn dispatch_context_routes_to_case_five_through_eight() {
        // case 5..8 routes to decode_sv8_context_band. With a constant
        // ctx_for_prev (always 0) every sample uses ctx-0's Q5 table;
        // feed 36 copies of that table's shortest codeword and confirm
        // the dispatcher matches the direct arm sample-for-sample.
        use crate::sv8_huffman::{table_for_role, Sv8TableRole};
        let table = table_for_role(Sv8TableRole::Q5, 0).expect("q5 ctx0");
        let (code, len) = first_row(table);
        let mut p = BitPacker::new();
        for _ in 0..SAMPLES_PER_BAND {
            p.push(code, len);
        }
        let bytes = p.finish();

        let mut direct = [0_i8; SAMPLES_PER_BAND];
        {
            let mut reader = Sv7BitReader::new(&bytes);
            crate::sv8_sample_decode::decode_sv8_context_band(
                &mut reader,
                5,
                0,
                |_prev| 0,
                &mut direct,
            )
            .expect("direct context");
        }

        let mut out = [0_i32; SAMPLES_PER_BAND];
        let mut prng = CnsPrng::new();
        let mut reader = Sv7BitReader::new(&bytes);
        decode_sv8_band(&mut reader, 5, &mut prng, 0, 0, |_prev| 0, &mut out)
            .expect("context dispatch");
        for (i, (&d, &o)) in direct.iter().zip(out.iter()).enumerate() {
            assert_eq!(o, d as i32, "sample {i} must match widened direct arm");
        }
    }

    #[test]
    fn dispatch_escape_routes_to_default_arm_native_i32() {
        // band_type 9 routes to decode_sv8_escape_band (0 raw bits, so
        // each sample is exactly the q9up VLC symbol). Feed 36 copies
        // of the q9up shortest codeword and confirm the dispatcher's
        // i32 buffer equals that symbol per slot — proving the escape
        // arm passes its native i32 output through unwidened.
        let sym = symbol_of_first_row(&SV8_Q9UP_TABLE) as i32;
        let (code, len) = first_row(&SV8_Q9UP_TABLE);
        let mut p = BitPacker::new();
        for _ in 0..SAMPLES_PER_BAND {
            p.push(code, len);
        }
        let bytes = p.finish();

        let mut out = [0_i32; SAMPLES_PER_BAND];
        let mut prng = CnsPrng::new();
        let mut reader = Sv7BitReader::new(&bytes);
        decode_sv8_band(&mut reader, 9, &mut prng, 0, 0, no_ctx, &mut out)
            .expect("escape dispatch");
        for (i, &s) in out.iter().enumerate() {
            assert_eq!(s, sym, "escape sample {i} (band_type 9, 0 raw bits)");
        }
    }

    #[test]
    fn dispatch_escape_band_type_ten_composes_vlc_and_one_raw_bit() {
        // band_type 10 → escape_raw_bits == 1: each sample is
        // (symbol << 1) | raw. Feed (q9up shortest codeword, raw=1)
        // 36 times and confirm the dispatcher composes them.
        let sym = symbol_of_first_row(&SV8_Q9UP_TABLE) as i32;
        let (code, len) = first_row(&SV8_Q9UP_TABLE);
        let mut p = BitPacker::new();
        for _ in 0..SAMPLES_PER_BAND {
            p.push(code, len);
            p.push_raw(1, 1);
        }
        let bytes = p.finish();

        let mut out = [0_i32; SAMPLES_PER_BAND];
        let mut prng = CnsPrng::new();
        let mut reader = Sv7BitReader::new(&bytes);
        decode_sv8_band(&mut reader, 10, &mut prng, 0, 0, no_ctx, &mut out)
            .expect("escape dispatch bt10");
        for (i, &s) in out.iter().enumerate() {
            assert_eq!(s, (sym << 1) | 1, "escape sample {i} (band_type 10)");
        }
    }

    #[test]
    fn dispatch_sparse_band_fails_loud_docs_gap() {
        // case 1 (sparse band) is a DOCS-GAP: the staged sv8-symbols-q1
        // 19-symbol alphabet cannot literally carry an 18-flag bitmap.
        // The dispatcher must fail loud, never emit a silently-wrong
        // band.
        let mut out = [0_i32; SAMPLES_PER_BAND];
        let mut prng = CnsPrng::new();
        let bytes = [0u8; 8];
        let mut reader = Sv7BitReader::new(&bytes);
        let err = decode_sv8_band(&mut reader, 1, &mut prng, 0, 0, no_ctx, &mut out).unwrap_err();
        assert!(matches!(err, Error::UnsupportedBandType(1)));
    }

    #[test]
    fn dispatch_out_of_range_fails_loud() {
        // band_type < -1 is not a §3.4-enumerated arm; promote the
        // classifier's fail-quiet OutOfRange to a hard error.
        let mut out = [0_i32; SAMPLES_PER_BAND];
        let mut prng = CnsPrng::new();
        let bytes = [0u8; 4];
        for bt in [-2i8, -10, i8::MIN] {
            let mut reader = Sv7BitReader::new(&bytes);
            let err =
                decode_sv8_band(&mut reader, bt, &mut prng, 0, 0, no_ctx, &mut out).unwrap_err();
            assert!(
                matches!(err, Error::UnsupportedBandType(b) if b == bt),
                "band_type {bt} must fail loud",
            );
        }
    }

    #[test]
    fn dispatch_grouped_ctx_knob_selects_case_two_table_half() {
        // The grouped_ctx knob must reach the case-2 grouped3 arm's
        // table-pair pick (and only it). Decoding the same bytes with
        // ctx 0 vs ctx 1 must agree with the respective direct arm.
        let mut prng = CnsPrng::new();
        for ctx in [0u8, 1u8] {
            let table =
                crate::sv8_huffman::table_for_role(crate::sv8_huffman::Sv8TableRole::Q2, ctx)
                    .expect("q2 table half");
            let (code, len) = first_row(table);
            let mut p = BitPacker::new();
            for _ in 0..GROUPED3_CODEWORDS_PER_BAND {
                p.push(code, len);
            }
            let bytes = p.finish();

            let mut direct = [0_i8; SAMPLES_PER_BAND];
            {
                let mut reader = Sv7BitReader::new(&bytes);
                crate::sv8_sample_decode::decode_sv8_grouped3_band(&mut reader, ctx, &mut direct)
                    .expect("direct grouped3");
            }

            let mut out = [0_i32; SAMPLES_PER_BAND];
            let mut reader = Sv7BitReader::new(&bytes);
            decode_sv8_band(&mut reader, 2, &mut prng, ctx, 0, no_ctx, &mut out)
                .expect("grouped3 dispatch with ctx knob");
            for (&d, &o) in direct.iter().zip(out.iter()) {
                assert_eq!(o, d as i32);
            }
        }
    }
}
