//! SV8 per-band sample decode (§3.4 case ladder) — the grounded
//! subset of the eight-variant [`crate::sv8_band_decode`] ladder.
//!
//! Composes the round-278 canonical-Huffman decoder walk
//! ([`crate::sv8_huffman::Sv8CanonicalTable::decode`]) with the
//! per-case sample fan-out facts pinned by the staged
//! `docs/audio/musepack/tables/sv8-symbols-*.{csv,meta}` material,
//! per the §3.4 audio-packet frame-body case ladder:
//!
//! ```text
//! switch (band_type) {
//!   case -1:  fill all 36 samples with random values   # noise substitution
//!   case  0:  do nothing                                # empty band
//!   case  1:  one VLC carrying flags for 18 samples;    # sparse band
//!             for each set flag, 1 raw bit per sample
//!   case  2:  read 12 VLCs to produce the 36 samples    # 3 samples / codeword
//!   case  3..4: read 18 VLCs to produce the 36 samples  # 2 samples / codeword
//!   case  5..8: per-sample VLC whose table is chosen    # first-order context
//!             by the previously decoded sample
//!   default:  per-sample VLC plus a fixed number of     # large-coefficient
//!             raw bits                                  #   escape
//! }
//! ```
//!
//! # What the staged numeric facts pin (verified by tests here)
//!
//! - **Case 2 (`Grouped3`, tables `sv8-canonical-q2-{1,2}`)** — the
//!   `.meta` `spec_role` says "5x5x5 grouped"; both 125-entry symbol
//!   maps are exact permutations of `0..=124`, and the most-probable
//!   (shortest-code, first-map-position) symbol is the all-zero
//!   triplet `62 = 2·25 + 2·5 + 2`. So each decoded symbol is a
//!   base-5-packed triplet of already-centred samples in `-2..=2`
//!   (digit value = sample + 2; 5 levels = `2D+1` with `D = 2`,
//!   matching the §2.6 requant relation).
//! - **Cases 3 / 4 (`Grouped2`, tables `sv8-canonical-q3` /
//!   `sv8-canonical-q4`)** — `spec_role` says "7x7 grouped" /
//!   "9x9 grouped, padded"; the 49-entry q3 map is an exact
//!   bijection onto `(-3..=3)²` and the first 81 q4 entries onto
//!   `(-4..=4)²` when each `int8` is split into **two signed 4-bit
//!   (two's-complement) nibbles** — e.g. `17 = 0x11 → (1, 1)`,
//!   `63 = 0x3F → (3, -1)`, `-16 = 0xF0 → (-1, 0)`. The 10 q4
//!   padding entries (map slots `81..=90`) are zero and unreachable
//!   per the round-278 exhaustive tiling proof.
//! - **Cases 5..=8 (`ContextHuffmanPerSample`, tables
//!   `sv8-canonical-q{5..8}-{1,2}`)** — every symbol map is an
//!   exact permutation of `-D..=D` for `D = 7 / 15 / 31 / 63`, so
//!   the decoded symbol IS the centred sample level directly (one
//!   VLC per sample, 36 reads).
//!
//! # Conventions the staged material does NOT pin
//!
//! - **Within-group emission order.** §1 / §2.5 ground that a
//!   grouped codeword covers *consecutive* samples, but neither the
//!   structural prose nor the numeric tables can distinguish which
//!   radix digit / nibble maps to the *first* of those samples
//!   (both assignments are bijections). This module emits the
//!   **least-significant digit (low nibble) first**; the choice is
//!   isolated inside [`unpack_grouped3_symbol`] /
//!   [`unpack_grouped2_symbol`] so a future observer trace pinning
//!   the opposite order is a one-line reverse.
//! - **The q2 context-pair selection rule.** Case 2 is outside the
//!   §3.4 `5..=8` first-order-context range, yet the staged tables
//!   ship a `{ctx0, ctx1}` pair for it. The selection rule is GAP;
//!   [`decode_sv8_grouped3_band`] takes `ctx` as a caller knob
//!   (the [`crate::packet_stream::PacketSizeConvention`] precedent
//!   for parameterising a documented GAP).
//! - **The `5..=8` context-update predicate.** §3.4 pins that the
//!   table is "chosen by the previously decoded sample" but not the
//!   predicate mapping that sample to the `{ctx0, ctx1}` pick;
//!   [`decode_sv8_context_band`] takes the rule as a caller-supplied
//!   closure.
//!
//! # Cases NOT implemented (DOCS-GAP, fail-loud)
//!
//! - **Case 1 (`SparseBand`)** — the staged `sv8-symbols-q1` map is
//!   a 19-symbol alphabet (`0..=18`), which cannot literally carry
//!   the "flags for 18 samples" the §3.4 prose describes (an 18-flag
//!   bitmap needs 2¹⁸ symbols); the symbol → flag-pattern semantics
//!   are underdetermined by the staged material.
//! - **Default arm (`LargeCoeffEscape`, `sv8-canonical-q9up`)** —
//!   the §3.4 prose says "a VLC plus a *fixed number* of raw bits"
//!   but does not pin that number (nor its `band_type` dependence).
//!
//! Both are reachable through [`crate::sv8_band_decode`]'s
//! classifier; asking this module to decode them is answered with
//! [`Error::UnsupportedBandType`] — fail-loud, not silently-wrong.
//! Cases `-1` (CNS) and `0` (empty) are shared arms with SV7 (the
//! round-245 classifier tests pin the agreement) and reuse
//! [`crate::sv7_band_decode::fill_cns_band`] /
//! [`crate::sv7_band_decode::fill_zero_band`] unchanged.

use crate::huffman::Sv7BitReader;
use crate::sv7_band_decode::SAMPLES_PER_BAND;
use crate::sv8_huffman::{table_for_role, Sv8TableRole};
use crate::{Error, Result};

/// Number of grouped codewords per band for §3.4 case 2
/// ("read 12 VLCs to produce the 36 samples").
pub const GROUPED3_CODEWORDS_PER_BAND: usize = 12;

/// Number of grouped codewords per band for §3.4 cases 3..=4
/// ("read 18 VLCs to produce the 36 samples").
pub const GROUPED2_CODEWORDS_PER_BAND: usize = 18;

/// Sign-extend the low 4 bits of `n` as a two's-complement nibble.
const fn sign_extend_nibble(n: u8) -> i8 {
    (((n & 0x0F) << 4) as i8) >> 4
}

/// Unpack one §3.4 case-2 grouped codeword symbol into its three
/// consecutive samples.
///
/// The staged `sv8-symbols-q2-{1,2}` maps are permutations of
/// `0..=124` ("5x5x5 grouped" per the `.meta` `spec_role`): each
/// symbol is a base-5-packed triplet with digit value = sample + 2,
/// i.e. samples in `-2..=2` (5 levels = `2D+1`, `D = 2`). The
/// shortest-code symbol is `62`, the all-zero triplet — confirming
/// the centring.
///
/// Emission order: least-significant digit first (see the
/// module-level "Conventions" note — the within-group order is the
/// one convention the staged material does not pin).
///
/// Symbols outside `0..=124` yield
/// [`Error::GroupedSymbolOutOfRange`] (unreachable when the symbol
/// comes from the staged maps, which the tests prove are confined
/// to the alphabet; kept as a defensive bound).
pub fn unpack_grouped3_symbol(symbol: i8) -> Result<[i8; 3]> {
    if !(0..=124).contains(&symbol) {
        return Err(Error::GroupedSymbolOutOfRange(symbol));
    }
    let s = symbol as i32;
    Ok([(s % 5 - 2) as i8, (s / 5 % 5 - 2) as i8, (s / 25 - 2) as i8])
}

/// Unpack one §3.4 case-3/4 grouped codeword symbol into its two
/// consecutive samples.
///
/// The staged `sv8-symbols-q3` map ("7x7 grouped") is an exact
/// bijection onto `(-3..=3)²` and the first 81 entries of
/// `sv8-symbols-q4` ("9x9 grouped, padded") onto `(-4..=4)²` when
/// the `int8` symbol is split into two signed two's-complement
/// nibbles. `band_type` (3 or 4) doubles as the per-nibble magnitude
/// bound `D` (7 resp. 9 levels = `2D+1` with `D = band_type`).
///
/// Emission order: low nibble first (module-level "Conventions"
/// note). A `band_type` outside `3..=4` yields
/// [`Error::UnsupportedBandType`]; a nibble outside `-D..=D` yields
/// [`Error::GroupedSymbolOutOfRange`] (unreachable for symbols drawn
/// from the staged maps; defensive bound).
pub fn unpack_grouped2_symbol(symbol: i8, band_type: i8) -> Result<[i8; 2]> {
    if !(3..=4).contains(&band_type) {
        return Err(Error::UnsupportedBandType(band_type));
    }
    let lo = sign_extend_nibble(symbol as u8);
    let hi = sign_extend_nibble((symbol as u8) >> 4);
    if lo.abs() > band_type || hi.abs() > band_type {
        return Err(Error::GroupedSymbolOutOfRange(symbol));
    }
    Ok([lo, hi])
}

/// Decode 36 samples for an SV8 band with `band_type == 2`
/// (§3.4 case 2, [`crate::sv8_band_decode::Sv8BandDecodeCase::Grouped3`]):
/// 12 canonical-Huffman codewords from the `ctx`-selected half of
/// the `sv8-canonical-q2-{1,2}` pair, each fanned out into 3
/// consecutive samples via [`unpack_grouped3_symbol`].
///
/// `ctx` must be 0 or 1 — the pair-selection rule is GAP (see the
/// module-level "Conventions" note), so the pick is a caller knob;
/// out-of-range `ctx` yields [`Error::UnsupportedBandType`].
pub fn decode_sv8_grouped3_band(
    reader: &mut Sv7BitReader<'_>,
    ctx: u8,
    out: &mut [i8; SAMPLES_PER_BAND],
) -> Result<()> {
    let table = table_for_role(Sv8TableRole::Q2, ctx).ok_or(Error::UnsupportedBandType(2))?;
    for group in out.chunks_exact_mut(3) {
        let symbol = table.decode(reader)?;
        group.copy_from_slice(&unpack_grouped3_symbol(symbol)?);
    }
    Ok(())
}

/// Decode 36 samples for an SV8 band with `band_type` in `3..=4`
/// (§3.4 cases 3..4, [`crate::sv8_band_decode::Sv8BandDecodeCase::Grouped2`]):
/// 18 canonical-Huffman codewords from `sv8-canonical-q3` (band_type
/// 3) or `sv8-canonical-q4` (band_type 4), each fanned out into 2
/// consecutive samples via [`unpack_grouped2_symbol`].
///
/// A `band_type` outside `3..=4` yields
/// [`Error::UnsupportedBandType`].
pub fn decode_sv8_grouped2_band(
    reader: &mut Sv7BitReader<'_>,
    band_type: i8,
    out: &mut [i8; SAMPLES_PER_BAND],
) -> Result<()> {
    let role = match band_type {
        3 => Sv8TableRole::Q3,
        4 => Sv8TableRole::Q4,
        _ => return Err(Error::UnsupportedBandType(band_type)),
    };
    let table = table_for_role(role, 0).ok_or(Error::UnsupportedBandType(band_type))?;
    for group in out.chunks_exact_mut(2) {
        let symbol = table.decode(reader)?;
        group.copy_from_slice(&unpack_grouped2_symbol(symbol, band_type)?);
    }
    Ok(())
}

/// Decode 36 samples for an SV8 band with `band_type` in `5..=8`
/// (§3.4 cases 5..8,
/// [`crate::sv8_band_decode::Sv8BandDecodeCase::ContextHuffmanPerSample`]):
/// one canonical-Huffman codeword per sample from the
/// `sv8-canonical-q{band_type}-{1,2}` context pair. The decoded
/// symbol IS the centred sample level (every staged q5..q8 map is a
/// permutation of `-D..=D`).
///
/// §3.4 pins that each sample's table is "chosen by the previously
/// decoded sample" but not the choice predicate; `ctx_for_prev`
/// supplies it as a caller knob (module-level "Conventions" note):
/// the first sample uses `initial_ctx`, every subsequent sample uses
/// `ctx_for_prev(previous_sample)`. A context value outside `0..=1`
/// (from either source) and a `band_type` outside `5..=8` yield
/// [`Error::UnsupportedBandType`].
pub fn decode_sv8_context_band<F>(
    reader: &mut Sv7BitReader<'_>,
    band_type: i8,
    initial_ctx: u8,
    mut ctx_for_prev: F,
    out: &mut [i8; SAMPLES_PER_BAND],
) -> Result<()>
where
    F: FnMut(i8) -> u8,
{
    let role = match band_type {
        5 => Sv8TableRole::Q5,
        6 => Sv8TableRole::Q6,
        7 => Sv8TableRole::Q7,
        8 => Sv8TableRole::Q8,
        _ => return Err(Error::UnsupportedBandType(band_type)),
    };
    let mut ctx = initial_ctx;
    for slot in out.iter_mut() {
        let table = table_for_role(role, ctx).ok_or(Error::UnsupportedBandType(band_type))?;
        *slot = table.decode(reader)?;
        ctx = ctx_for_prev(*slot);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sv8_huffman::{
        Sv8CanonicalTable, SV8_Q2_1_TABLE, SV8_Q2_2_TABLE, SV8_Q3_TABLE, SV8_Q4_TABLE,
        SV8_Q5_1_TABLE, SV8_Q5_2_TABLE, SV8_Q6_1_TABLE, SV8_Q6_2_TABLE, SV8_Q7_1_TABLE,
        SV8_Q7_2_TABLE, SV8_Q8_1_TABLE, SV8_Q8_2_TABLE,
    };
    use std::collections::BTreeSet;

    /// MSB-first bit packer for hand-building codeword streams
    /// (left-justified `pattern` of `length` bits per push).
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

        /// Flush + append two zero bytes so `peek16` never starves.
        fn finish(mut self) -> Vec<u8> {
            if self.nbits > 0 {
                self.bytes.push((self.acc << (8 - self.nbits)) as u8);
            }
            self.bytes.push(0);
            self.bytes.push(0);
            self.bytes
        }
    }

    /// Decode the symbol selected by `row`'s exact code pattern
    /// (low peek bits zero), via the round-278 walk.
    fn symbol_for_row(table: &Sv8CanonicalTable, row: usize) -> i8 {
        let entry = table.lengths[row];
        symbol_for_pattern(table, entry.code, entry.length)
    }

    /// Decode the symbol selected by an exact left-justified
    /// codeword `pattern` of `length` bits, via the round-278 walk.
    fn symbol_for_pattern(table: &Sv8CanonicalTable, pattern: u16, length: u8) -> i8 {
        let mut p = BitPacker::new();
        p.push(pattern, length);
        let bytes = p.finish();
        let mut reader = Sv7BitReader::new(&bytes);
        let before = reader.bits_remaining();
        let symbol = table.decode(&mut reader).expect("single-codeword decode");
        assert_eq!(
            before - reader.bits_remaining(),
            length as u64,
            "{}: pattern {pattern:#06x} must consume exactly {length} bits",
            table.name,
        );
        symbol
    }

    /// Find a `(pattern, length, symbol)` codeword of `table` whose
    /// decoded symbol satisfies `pred`, walking every codeword of
    /// every data row (a row of length `L` at code `c` spans the
    /// peek range `[c, previous_row_code)` in steps of
    /// `1 << (16 - L)`).
    fn find_codeword(
        table: &Sv8CanonicalTable,
        pred: impl Fn(i8) -> bool,
    ) -> Option<(u16, u8, i8)> {
        let mut upper: u32 = 0x1_0000;
        for entry in table.lengths.iter() {
            if entry.length == 0 {
                continue; // staged q4 padding sentinel
            }
            let step = 1_u32 << (16 - entry.length as u32);
            let mut pat = entry.code as u32;
            while pat < upper {
                let symbol = symbol_for_pattern(table, pat as u16, entry.length);
                if pred(symbol) {
                    return Some((pat as u16, entry.length, symbol));
                }
                pat += step;
            }
            upper = entry.code as u32;
        }
        None
    }

    // ─── Staged-fact pins: symbol-map structure ────────────

    #[test]
    fn q2_symbol_maps_are_base5_permutations_centred_at_62() {
        for table in [&SV8_Q2_1_TABLE, &SV8_Q2_2_TABLE] {
            let mut values: Vec<i8> = table.symbols.to_vec();
            values.sort_unstable();
            let expected: Vec<i8> = (0..125).collect();
            assert_eq!(values, expected, "{}", table.name);
            // Most-probable (first-map-position) symbol is the
            // all-zero triplet 62 = 2*25 + 2*5 + 2 — the centring
            // pin for the base-5 packing.
            assert_eq!(table.symbols[0], 62, "{}", table.name);
            assert_eq!(unpack_grouped3_symbol(table.symbols[0]).unwrap(), [0; 3]);
        }
    }

    #[test]
    fn q3_symbol_map_is_nibble_pair_bijection_over_pm3() {
        assert_eq!(SV8_Q3_TABLE.symbols.len(), 49);
        let mut seen = BTreeSet::new();
        for &sym in SV8_Q3_TABLE.symbols {
            let [lo, hi] = unpack_grouped2_symbol(sym, 3).expect("q3 symbol in (-3..=3)^2");
            seen.insert((lo, hi));
        }
        // Exactly the 49 pairs over (-3..=3)^2, each once.
        assert_eq!(seen.len(), 49);
        for a in -3..=3_i8 {
            for b in -3..=3_i8 {
                assert!(seen.contains(&(a, b)), "missing pair ({a}, {b})");
            }
        }
    }

    #[test]
    fn q4_symbol_map_is_nibble_pair_bijection_over_pm4_plus_zero_padding() {
        assert_eq!(SV8_Q4_TABLE.symbols.len(), 91);
        let mut seen = BTreeSet::new();
        for &sym in &SV8_Q4_TABLE.symbols[..81] {
            let [lo, hi] = unpack_grouped2_symbol(sym, 4).expect("q4 symbol in (-4..=4)^2");
            seen.insert((lo, hi));
        }
        assert_eq!(seen.len(), 81);
        for a in -4..=4_i8 {
            for b in -4..=4_i8 {
                assert!(seen.contains(&(a, b)), "missing pair ({a}, {b})");
            }
        }
        // Slots 81..=90 are zero padding (unreachable per the
        // round-278 tiling proof).
        assert!(SV8_Q4_TABLE.symbols[81..].iter().all(|&s| s == 0));
    }

    #[test]
    fn q5_to_q8_symbol_maps_are_signed_level_permutations() {
        for (table, d) in [
            (&SV8_Q5_1_TABLE, 7_i8),
            (&SV8_Q5_2_TABLE, 7),
            (&SV8_Q6_1_TABLE, 15),
            (&SV8_Q6_2_TABLE, 15),
            (&SV8_Q7_1_TABLE, 31),
            (&SV8_Q7_2_TABLE, 31),
            (&SV8_Q8_1_TABLE, 63),
            (&SV8_Q8_2_TABLE, 63),
        ] {
            let mut values: Vec<i8> = table.symbols.to_vec();
            values.sort_unstable();
            let expected: Vec<i8> = (-d..=d).collect();
            assert_eq!(values, expected, "{}", table.name);
        }
    }

    // ─── unpack_grouped3_symbol ────────────────────────────

    #[test]
    fn unpack_grouped3_hand_vectors() {
        // Centre and the six unit-step neighbours (the staged maps'
        // own structure: 62 ± 1 / ± 5 / ± 25).
        assert_eq!(unpack_grouped3_symbol(62).unwrap(), [0, 0, 0]);
        assert_eq!(unpack_grouped3_symbol(63).unwrap(), [1, 0, 0]);
        assert_eq!(unpack_grouped3_symbol(61).unwrap(), [-1, 0, 0]);
        assert_eq!(unpack_grouped3_symbol(67).unwrap(), [0, 1, 0]);
        assert_eq!(unpack_grouped3_symbol(57).unwrap(), [0, -1, 0]);
        assert_eq!(unpack_grouped3_symbol(87).unwrap(), [0, 0, 1]);
        assert_eq!(unpack_grouped3_symbol(37).unwrap(), [0, 0, -1]);
        // Alphabet corners.
        assert_eq!(unpack_grouped3_symbol(0).unwrap(), [-2, -2, -2]);
        assert_eq!(unpack_grouped3_symbol(124).unwrap(), [2, 2, 2]);
    }

    #[test]
    fn unpack_grouped3_covers_alphabet_bijectively() {
        let mut seen = BTreeSet::new();
        for s in 0..=124_i8 {
            let t = unpack_grouped3_symbol(s).unwrap();
            assert!(t.iter().all(|&v| (-2..=2).contains(&v)), "symbol {s}");
            seen.insert(t);
        }
        assert_eq!(seen.len(), 125);
    }

    #[test]
    fn unpack_grouped3_rejects_out_of_alphabet() {
        for s in [-1_i8, 125, 127, i8::MIN] {
            assert!(matches!(
                unpack_grouped3_symbol(s),
                Err(Error::GroupedSymbolOutOfRange(v)) if v == s,
            ));
        }
    }

    // ─── unpack_grouped2_symbol ────────────────────────────

    #[test]
    fn unpack_grouped2_hand_vectors() {
        // Nibble-pair arithmetic, low nibble first.
        assert_eq!(unpack_grouped2_symbol(0, 3).unwrap(), [0, 0]);
        assert_eq!(unpack_grouped2_symbol(0x11, 3).unwrap(), [1, 1]);
        assert_eq!(unpack_grouped2_symbol(0x10, 3).unwrap(), [0, 1]);
        assert_eq!(unpack_grouped2_symbol(0x01, 3).unwrap(), [1, 0]);
        assert_eq!(unpack_grouped2_symbol(0x1F, 3).unwrap(), [-1, 1]);
        assert_eq!(unpack_grouped2_symbol(-16, 3).unwrap(), [0, -1]); // 0xF0
        assert_eq!(unpack_grouped2_symbol(-1, 3).unwrap(), [-1, -1]); // 0xFF
        assert_eq!(unpack_grouped2_symbol(0x3F, 3).unwrap(), [-1, 3]);
        assert_eq!(unpack_grouped2_symbol(0x33, 3).unwrap(), [3, 3]);
        // band_type 4 widens the bound to ±4.
        assert_eq!(unpack_grouped2_symbol(0x40, 4).unwrap(), [0, 4]);
        assert_eq!(unpack_grouped2_symbol(0x4C, 4).unwrap(), [-4, 4]);
        assert_eq!(unpack_grouped2_symbol(-52, 4).unwrap(), [-4, -4]); // 0xCC
    }

    #[test]
    fn unpack_grouped2_rejects_out_of_range_nibbles_and_band_type() {
        // 0x04: low nibble 4 exceeds the q3 (band_type 3) bound but
        // is fine for q4 (band_type 4).
        assert!(matches!(
            unpack_grouped2_symbol(0x04, 3),
            Err(Error::GroupedSymbolOutOfRange(0x04)),
        ));
        assert_eq!(unpack_grouped2_symbol(0x04, 4).unwrap(), [4, 0]);
        // 0x50: high nibble 5 exceeds both bounds.
        for bt in [3_i8, 4] {
            assert!(matches!(
                unpack_grouped2_symbol(0x50, bt),
                Err(Error::GroupedSymbolOutOfRange(0x50)),
            ));
        }
        // band_type outside 3..=4 is rejected before any unpack.
        for bt in [-1_i8, 0, 1, 2, 5, 8, i8::MAX] {
            assert!(matches!(
                unpack_grouped2_symbol(0, bt),
                Err(Error::UnsupportedBandType(v)) if v == bt,
            ));
        }
    }

    // ─── decode_sv8_grouped3_band (case 2) ─────────────────

    #[test]
    fn grouped3_band_decodes_twelve_codewords_both_contexts() {
        for (ctx, table) in [(0_u8, &SV8_Q2_1_TABLE), (1, &SV8_Q2_2_TABLE)] {
            let entry = table.lengths[0];
            let expected_symbol = symbol_for_row(table, 0);
            let expected_triplet = unpack_grouped3_symbol(expected_symbol).unwrap();

            let mut p = BitPacker::new();
            for _ in 0..GROUPED3_CODEWORDS_PER_BAND {
                p.push(entry.code, entry.length);
            }
            let bytes = p.finish();
            let mut reader = Sv7BitReader::new(&bytes);
            let before = reader.bits_remaining();
            let mut out = [99_i8; SAMPLES_PER_BAND];
            decode_sv8_grouped3_band(&mut reader, ctx, &mut out).expect("decode");
            // 12 codewords, each fanned into 3 consecutive samples.
            for (g, group) in out.chunks_exact(3).enumerate() {
                assert_eq!(group, expected_triplet, "ctx {ctx} group {g}");
            }
            // Exactly 12 codewords' worth of bits consumed.
            assert_eq!(
                before - reader.bits_remaining(),
                12 * entry.length as u64,
                "ctx {ctx}"
            );
        }
    }

    #[test]
    fn grouped3_band_orders_groups_by_codeword_sequence() {
        // Two distinct codewords alternated: groups must alternate
        // their unpacked triplets in stream order.
        let table = &SV8_Q2_1_TABLE;
        let row_a = 0;
        let row_b = table.lengths.len() - 1; // last data row (code 0x0000)
        let tri_a = unpack_grouped3_symbol(symbol_for_row(table, row_a)).unwrap();
        let tri_b = unpack_grouped3_symbol(symbol_for_row(table, row_b)).unwrap();
        assert_ne!(tri_a, tri_b, "rows must decode to distinct triplets");

        let mut p = BitPacker::new();
        for g in 0..GROUPED3_CODEWORDS_PER_BAND {
            let row = if g % 2 == 0 { row_a } else { row_b };
            p.push(table.lengths[row].code, table.lengths[row].length);
        }
        let bytes = p.finish();
        let mut reader = Sv7BitReader::new(&bytes);
        let mut out = [0_i8; SAMPLES_PER_BAND];
        decode_sv8_grouped3_band(&mut reader, 0, &mut out).expect("decode");
        for (g, group) in out.chunks_exact(3).enumerate() {
            let expected = if g % 2 == 0 { tri_a } else { tri_b };
            assert_eq!(group, expected, "group {g}");
        }
    }

    #[test]
    fn grouped3_band_rejects_out_of_range_ctx() {
        let mut out = [0_i8; SAMPLES_PER_BAND];
        for ctx in [2_u8, 3, u8::MAX] {
            let mut reader = Sv7BitReader::new(&[0xFF; 64]);
            assert!(matches!(
                decode_sv8_grouped3_band(&mut reader, ctx, &mut out),
                Err(Error::UnsupportedBandType(2)),
            ));
        }
    }

    #[test]
    fn grouped3_band_propagates_eof() {
        // Far too short for 12 codewords.
        let mut reader = Sv7BitReader::new(&[0xFF]);
        let mut out = [0_i8; SAMPLES_PER_BAND];
        assert!(matches!(
            decode_sv8_grouped3_band(&mut reader, 0, &mut out),
            Err(Error::UnexpectedEof),
        ));
    }

    // ─── decode_sv8_grouped2_band (cases 3..=4) ────────────

    #[test]
    fn grouped2_band_decodes_eighteen_codewords_for_q3_and_q4() {
        for (band_type, table) in [(3_i8, &SV8_Q3_TABLE), (4, &SV8_Q4_TABLE)] {
            let entry = table.lengths[0];
            let expected_pair =
                unpack_grouped2_symbol(symbol_for_row(table, 0), band_type).unwrap();

            let mut p = BitPacker::new();
            for _ in 0..GROUPED2_CODEWORDS_PER_BAND {
                p.push(entry.code, entry.length);
            }
            let bytes = p.finish();
            let mut reader = Sv7BitReader::new(&bytes);
            let before = reader.bits_remaining();
            let mut out = [99_i8; SAMPLES_PER_BAND];
            decode_sv8_grouped2_band(&mut reader, band_type, &mut out).expect("decode");
            for (g, group) in out.chunks_exact(2).enumerate() {
                assert_eq!(group, expected_pair, "band_type {band_type} group {g}");
            }
            assert_eq!(
                before - reader.bits_remaining(),
                18 * entry.length as u64,
                "band_type {band_type}"
            );
        }
    }

    #[test]
    fn grouped2_band_orders_groups_by_codeword_sequence() {
        let table = &SV8_Q3_TABLE;
        let row_a = 0;
        let row_b = table.lengths.len() - 1;
        let pair_a = unpack_grouped2_symbol(symbol_for_row(table, row_a), 3).unwrap();
        let pair_b = unpack_grouped2_symbol(symbol_for_row(table, row_b), 3).unwrap();
        assert_ne!(pair_a, pair_b, "rows must decode to distinct pairs");

        let mut p = BitPacker::new();
        for g in 0..GROUPED2_CODEWORDS_PER_BAND {
            let row = if g % 2 == 0 { row_a } else { row_b };
            p.push(table.lengths[row].code, table.lengths[row].length);
        }
        let bytes = p.finish();
        let mut reader = Sv7BitReader::new(&bytes);
        let mut out = [0_i8; SAMPLES_PER_BAND];
        decode_sv8_grouped2_band(&mut reader, 3, &mut out).expect("decode");
        for (g, group) in out.chunks_exact(2).enumerate() {
            let expected = if g % 2 == 0 { pair_a } else { pair_b };
            assert_eq!(group, expected, "group {g}");
        }
    }

    #[test]
    fn grouped2_band_rejects_band_type_outside_3_4() {
        let mut out = [0_i8; SAMPLES_PER_BAND];
        for bt in [-1_i8, 0, 1, 2, 5, 8, 9, i8::MAX] {
            let mut reader = Sv7BitReader::new(&[0xFF; 64]);
            assert!(matches!(
                decode_sv8_grouped2_band(&mut reader, bt, &mut out),
                Err(Error::UnsupportedBandType(v)) if v == bt,
            ));
        }
    }

    #[test]
    fn grouped2_band_propagates_eof() {
        let mut reader = Sv7BitReader::new(&[0xFF]);
        let mut out = [0_i8; SAMPLES_PER_BAND];
        assert!(matches!(
            decode_sv8_grouped2_band(&mut reader, 3, &mut out),
            Err(Error::UnexpectedEof),
        ));
    }

    // ─── decode_sv8_context_band (cases 5..=8) ─────────────

    #[test]
    fn context_band_constant_rule_decodes_per_sample_all_band_types() {
        for (band_type, table) in [
            (5_i8, &SV8_Q5_1_TABLE),
            (6, &SV8_Q6_1_TABLE),
            (7, &SV8_Q7_1_TABLE),
            (8, &SV8_Q8_1_TABLE),
        ] {
            let entry = table.lengths[0];
            let expected = symbol_for_row(table, 0);
            let mut p = BitPacker::new();
            for _ in 0..SAMPLES_PER_BAND {
                p.push(entry.code, entry.length);
            }
            let bytes = p.finish();
            let mut reader = Sv7BitReader::new(&bytes);
            let before = reader.bits_remaining();
            let mut out = [99_i8; SAMPLES_PER_BAND];
            decode_sv8_context_band(&mut reader, band_type, 0, |_| 0, &mut out).expect("decode");
            assert!(out.iter().all(|&s| s == expected), "band_type {band_type}");
            assert_eq!(
                before - reader.bits_remaining(),
                36 * entry.length as u64,
                "band_type {band_type}"
            );
        }
    }

    #[test]
    fn context_band_switches_tables_on_previous_sample() {
        // Find a ctx-0 row decoding to a negative symbol and a
        // ctx-1 row decoding to a non-negative one, then alternate
        // their codewords under the rule "prev < 0 → ctx 1".
        let t0 = &SV8_Q5_1_TABLE;
        let t1 = &SV8_Q5_2_TABLE;
        let (neg_pat, neg_len, neg_sym) =
            find_codeword(t0, |s| s < 0).expect("q5-1 has a negative-symbol codeword");
        let (nonneg_pat, nonneg_len, nonneg_sym) =
            find_codeword(t1, |s| s >= 0).expect("q5-2 has a non-negative-symbol codeword");

        // Sample i even → decoded via ctx 0 (q5-1, negative symbol),
        // sample i odd → ctx 1 (q5-2, non-negative symbol), and the
        // rule keeps the alternation going for all 36 samples.
        let mut p = BitPacker::new();
        for i in 0..SAMPLES_PER_BAND {
            if i % 2 == 0 {
                p.push(neg_pat, neg_len);
            } else {
                p.push(nonneg_pat, nonneg_len);
            }
        }
        let bytes = p.finish();
        let mut reader = Sv7BitReader::new(&bytes);
        let mut out = [0_i8; SAMPLES_PER_BAND];
        decode_sv8_context_band(
            &mut reader,
            5,
            0,
            |prev| if prev < 0 { 1 } else { 0 },
            &mut out,
        )
        .expect("decode");
        for (i, &s) in out.iter().enumerate() {
            let expected = if i % 2 == 0 { neg_sym } else { nonneg_sym };
            assert_eq!(s, expected, "sample {i}");
        }
    }

    #[test]
    fn context_band_rejects_band_type_outside_5_8() {
        let mut out = [0_i8; SAMPLES_PER_BAND];
        for bt in [-1_i8, 0, 1, 2, 3, 4, 9, 17, i8::MAX] {
            let mut reader = Sv7BitReader::new(&[0xFF; 128]);
            assert!(matches!(
                decode_sv8_context_band(&mut reader, bt, 0, |_| 0, &mut out),
                Err(Error::UnsupportedBandType(v)) if v == bt,
            ));
        }
    }

    #[test]
    fn context_band_rejects_out_of_range_context_values() {
        let mut out = [0_i8; SAMPLES_PER_BAND];
        // Bad initial_ctx fails before any bit is read.
        let mut reader = Sv7BitReader::new(&[0xFF; 128]);
        let before = reader.bits_remaining();
        assert!(matches!(
            decode_sv8_context_band(&mut reader, 5, 2, |_| 0, &mut out),
            Err(Error::UnsupportedBandType(5)),
        ));
        assert_eq!(reader.bits_remaining(), before);
        // A rule emitting ctx >= 2 fails on the second sample.
        let mut reader = Sv7BitReader::new(&[0xFF; 128]);
        assert!(matches!(
            decode_sv8_context_band(&mut reader, 5, 0, |_| 2, &mut out),
            Err(Error::UnsupportedBandType(5)),
        ));
    }

    #[test]
    fn context_band_propagates_eof() {
        let mut reader = Sv7BitReader::new(&[0xFF]);
        let mut out = [0_i8; SAMPLES_PER_BAND];
        assert!(matches!(
            decode_sv8_context_band(&mut reader, 5, 0, |_| 0, &mut out),
            Err(Error::UnexpectedEof),
        ));
    }

    // ─── Classifier composition ────────────────────────────

    #[test]
    fn implemented_cases_match_classifier_arms() {
        use crate::sv8_band_decode::{sv8_band_type_case, Sv8BandDecodeCase};
        // The band_type domains this module's decoders accept are
        // exactly the classifier arms they implement.
        assert_eq!(sv8_band_type_case(2), Sv8BandDecodeCase::Grouped3);
        for bt in 3..=4 {
            assert_eq!(sv8_band_type_case(bt), Sv8BandDecodeCase::Grouped2);
        }
        for bt in 5..=8 {
            assert_eq!(
                sv8_band_type_case(bt),
                Sv8BandDecodeCase::ContextHuffmanPerSample
            );
        }
        // The two DOCS-GAP arms stay unimplemented and fail loudly.
        assert_eq!(sv8_band_type_case(1), Sv8BandDecodeCase::SparseBand);
        assert_eq!(sv8_band_type_case(9), Sv8BandDecodeCase::LargeCoeffEscape);
        let mut out = [0_i8; SAMPLES_PER_BAND];
        for bt in [1_i8, 9] {
            let mut reader = Sv7BitReader::new(&[0xFF; 64]);
            assert!(decode_sv8_grouped2_band(&mut reader, bt, &mut out).is_err());
            let mut reader = Sv7BitReader::new(&[0xFF; 64]);
            assert!(decode_sv8_context_band(&mut reader, bt, 0, |_| 0, &mut out).is_err());
        }
    }
}
