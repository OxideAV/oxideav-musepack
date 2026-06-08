//! SV8 canonical Huffman length-tables and paired symbol maps.
//!
//! Wires the 21 staged `sv8-canonical-*.csv` length-tables and
//! their 21 paired `sv8-symbols-*.csv` symbol maps from
//! `docs/audio/musepack/tables/` into typed Rust statics, exposed
//! as named [`Sv8CanonicalTable`] views the SV8 §3.4 / §3.5 decoder
//! pipeline can `match` against.
//!
//! ## Inventory
//!
//! The 21 pairs map to §3.4 / §3.5 spec roles per
//! `docs/audio/musepack/musepack-sv7-sv8-spec.md` §4 and
//! `docs/audio/musepack/provenance/01-musepack-table-extraction.md`
//! §6:
//!
//! - **§3.4 used-subbands selector** — `bands`.
//! - **§3.4 band-resolution (band_type) selector**, context pair —
//!   `res-1`, `res-2`.
//! - **§3.5 SCFI selector**, context pair — `scfi-1`, `scfi-2`.
//! - **§3.5 delta-scalefactor (DSCF) VLC**, context pair —
//!   `dscf-1`, `dscf-2`.
//! - **§3.4 case-1 (sparse band, 18-flag) VLC** — `q1`.
//! - **§3.4 case-2 (3-samples-per-codeword) VLC**, context pair —
//!   `q2-1`, `q2-2`.
//! - **§3.4 case-3 (2-samples-per-codeword) VLC** — `q3`.
//! - **§3.4 case-4 (2-samples-per-codeword) VLC** — `q4`.
//! - **§3.4 case-5..=8 first-order context-adaptive per-sample
//!   VLCs**, each a context pair — `q5-1`..`q5-2`, `q6-1`..`q6-2`,
//!   `q7-1`..`q7-2`, `q8-1`..`q8-2`.
//! - **§3.4 case-9+ large-coefficient escape VLC**, signed
//!   `−128..=127` symbol map — `q9up`.
//!
//! Per-band_type → table mapping is structurally fixed by the §3.4
//! case ladder reproduced in [`crate::sv8_band_decode`]: `case 1`
//! → `Q1`, `case 2` → `Q2`, `case 3` / `case 4` → `Q3` / `Q4`,
//! `case 5..=8` → `Q5..Q8` (each picks `-1` or `-2` from the
//! first-order context), `default` (`band_type ≥ 9`) → `Q9up`.
//!
//! ## Decoder convention — DOCS-GAP
//!
//! The structural spec
//! (`docs/audio/musepack/musepack-sv7-sv8-spec.md` §3.4) names
//! the canonical Huffman layer ("SV8 canonical Huffman codebooks
//! ... 21 length-tables + 21 symbol maps") and pins the layout of
//! each row as
//! `mpc_huffman = {Code: uint16 left-adjusted, Length: uint8,
//! Value: cumulative index}`, with the staged `.meta` sidecars
//! adding: *"Decode pairs this length-table with its `*-sym`
//! symbol map; the Value column is the running symbol index, not
//! the final symbol."*
//!
//! What the structural prose **does not** pin is the exact
//! arithmetic that maps a peeked 16-bit code window to a symbol
//! index. Two reasonable interpretations of the cumulative-index
//! column give incompatible per-row sub-index assignments
//! (forward-ascending vs descending-from-cum), and the choice is
//! not derivable from the table values alone (a Kraft-McMillan
//! count check rules out the naive "one row covers
//! `2^(16 − Length)` peek bins" formulation: the SV8 length-tables
//! routinely skip intermediate lengths, so the per-row code count
//! does **not** equal the peek-bin count).
//!
//! Per the project's "ask for docs, don't fish" rule, this module
//! deliberately stops at the typed-table surface: the constants,
//! the `Sv8CanonicalTable` newtype, and shape-only sanity tests.
//! The decoder walk is left for the round that follows the §3.4
//! docs patch resolving the cumulative-index convention.
//!
//! ## Source-of-record
//!
//! - Structural prose: `docs/audio/musepack/musepack-sv7-sv8-spec.md`
//!   (§3.4 case ladder, §3.5 SCF, §4 table inventory).
//! - Numeric initialisers: `sv8-canonical-*.csv` + `sv8-symbols-*.csv`
//!   under `docs/audio/musepack/tables/` (mirrored at `<crate>/tables/`).
//! - Provenance: `docs/audio/musepack/provenance/01-musepack-table-extraction.md`
//!   §6 ("SV8 canonical length-tables + symbol maps").
//!
//! No third-party Musepack implementation source was read in
//! authoring this module; the only project material crossed is
//! the staged `docs/` content above and the existing SV7 sibling
//! modules under `crates/oxideav-musepack/src/`.

/// One row of an SV8 canonical Huffman **length table**.
///
/// Fields per the staged `.meta` sidecars'
/// `value_encoding: canonical-huffman length table
/// mpc_huffman={Code:uint16,Length:uint8,Value:cum index}` line:
///
/// - `code` — canonical code word **left-justified into 16 bits**
///   (the high `length` bits carry the literal pattern; the low
///   `16 − length` bits are zero). Rows within a single table are
///   stored sorted by `code` descending (= length non-decreasing),
///   matching the staged CSV row order.
/// - `length` — code length in bits.
/// - `cum_index` — running symbol-index counter into the paired
///   `Sv8CanonicalTable::symbols` map. The exact arithmetic that
///   converts `(peek, code, length, cum_index)` into a symbol
///   index is **DOCS-GAP**; see the module-level docs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Sv8CanonicalEntry {
    /// Code word, left-justified into 16 bits.
    pub code: u16,
    /// Number of bits in the code word.
    pub length: u8,
    /// Running cumulative symbol-index into the paired symbol map.
    /// Stored as `i16` to accommodate the `q9up` large-coefficient
    /// escape map, whose late rows carry small negative cumulative
    /// indices per the staged CSV (`sv8-canonical-q9up.csv` last
    /// rows: `-45, -7, -2, -1`).
    pub cum_index: i16,
}

/// Paired (length-table, symbol-map) view of one of the 21 SV8
/// canonical Huffman tables.
///
/// The `name` field carries the staged CSV stem
/// (e.g. `"sv8-canonical-bands"`) for diagnostics and trace
/// logging; it is **not** consumed by any decoder logic.
#[derive(Debug, Clone, Copy)]
pub struct Sv8CanonicalTable {
    /// Length-table rows, sorted by `code` descending.
    pub lengths: &'static [Sv8CanonicalEntry],
    /// Paired symbol map; the cumulative-index walk against
    /// [`Sv8CanonicalTable::lengths`] indexes into this slice.
    pub symbols: &'static [i8],
    /// Staged CSV stem (e.g. `"sv8-canonical-bands"`).
    pub name: &'static str,
}

impl Sv8CanonicalTable {
    /// Number of rows in the length table.
    pub const fn len_table_rows(&self) -> usize {
        self.lengths.len()
    }

    /// Number of entries in the paired symbol map.
    pub const fn sym_table_rows(&self) -> usize {
        self.symbols.len()
    }

    /// Smallest code length present in this table.
    /// Returns `0` for an empty length table (none of the staged
    /// tables are empty; the helper is total for safety).
    pub fn min_length(&self) -> u8 {
        self.lengths.iter().map(|e| e.length).min().unwrap_or(0)
    }

    /// Largest code length present in this table.
    pub fn max_length(&self) -> u8 {
        self.lengths.iter().map(|e| e.length).max().unwrap_or(0)
    }
}

include!(concat!(env!("OUT_DIR"), "/sv8_canonical_tables.rs"));

/// Catalogue of every SV8 canonical table the build script wires
/// in. The order mirrors `build.rs::SV8_CANONICAL_INPUTS`.
pub static SV8_CANONICAL_CATALOGUE: [&Sv8CanonicalTable; 21] = [
    &SV8_BANDS_TABLE,
    &SV8_RES_1_TABLE,
    &SV8_RES_2_TABLE,
    &SV8_SCFI_1_TABLE,
    &SV8_SCFI_2_TABLE,
    &SV8_DSCF_1_TABLE,
    &SV8_DSCF_2_TABLE,
    &SV8_Q1_TABLE,
    &SV8_Q2_1_TABLE,
    &SV8_Q2_2_TABLE,
    &SV8_Q3_TABLE,
    &SV8_Q4_TABLE,
    &SV8_Q5_1_TABLE,
    &SV8_Q5_2_TABLE,
    &SV8_Q6_1_TABLE,
    &SV8_Q6_2_TABLE,
    &SV8_Q7_1_TABLE,
    &SV8_Q7_2_TABLE,
    &SV8_Q8_1_TABLE,
    &SV8_Q8_2_TABLE,
    &SV8_Q9UP_TABLE,
];

/// Resolve a §3.4 / §3.5 spec role plus first-order context bit
/// into the matching [`Sv8CanonicalTable`].
///
/// `ctx` is `0` or `1`; for tables with no context split (`Bands`,
/// `Q1`, `Q3`, `Q4`, `Q9up`) the `ctx` value is ignored. Returns
/// `None` for an out-of-range `ctx` against a context-pair table.
///
/// The role enum mirrors the §3.4 case ladder reproduced in
/// [`crate::sv8_band_decode`] plus the §3.5 SCF / DSCF additions.
pub fn table_for_role(role: Sv8TableRole, ctx: u8) -> Option<&'static Sv8CanonicalTable> {
    use Sv8TableRole::*;
    Some(match (role, ctx) {
        (Bands, _) => &SV8_BANDS_TABLE,
        (Q1, _) => &SV8_Q1_TABLE,
        (Q3, _) => &SV8_Q3_TABLE,
        (Q4, _) => &SV8_Q4_TABLE,
        (Q9up, _) => &SV8_Q9UP_TABLE,
        (Res, 0) => &SV8_RES_1_TABLE,
        (Res, 1) => &SV8_RES_2_TABLE,
        (Scfi, 0) => &SV8_SCFI_1_TABLE,
        (Scfi, 1) => &SV8_SCFI_2_TABLE,
        (Dscf, 0) => &SV8_DSCF_1_TABLE,
        (Dscf, 1) => &SV8_DSCF_2_TABLE,
        (Q2, 0) => &SV8_Q2_1_TABLE,
        (Q2, 1) => &SV8_Q2_2_TABLE,
        (Q5, 0) => &SV8_Q5_1_TABLE,
        (Q5, 1) => &SV8_Q5_2_TABLE,
        (Q6, 0) => &SV8_Q6_1_TABLE,
        (Q6, 1) => &SV8_Q6_2_TABLE,
        (Q7, 0) => &SV8_Q7_1_TABLE,
        (Q7, 1) => &SV8_Q7_2_TABLE,
        (Q8, 0) => &SV8_Q8_1_TABLE,
        (Q8, 1) => &SV8_Q8_2_TABLE,
        _ => return None,
    })
}

/// SV8 §3.4 / §3.5 canonical Huffman table role tag.
///
/// One variant per *logical* table the case ladder names. Tables
/// with a first-order context split (`Res`, `Scfi`, `Dscf`,
/// `Q2`, `Q5..=Q8`) share a single role tag; the `ctx` argument
/// to [`table_for_role`] picks which of the two context-keyed
/// physical tables to return.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Sv8TableRole {
    /// `sv8-canonical-bands` — §3.4 used-subbands selector.
    Bands,
    /// `sv8-canonical-res-{1,2}` — §3.4 band-resolution / band_type
    /// selector, context pair.
    Res,
    /// `sv8-canonical-scfi-{1,2}` — §3.5 SCFI selector, context
    /// pair.
    Scfi,
    /// `sv8-canonical-dscf-{1,2}` — §3.5 delta-scalefactor VLC,
    /// context pair.
    Dscf,
    /// `sv8-canonical-q1` — §3.4 case-1 sparse-band 18-flag VLC.
    Q1,
    /// `sv8-canonical-q2-{1,2}` — §3.4 case-2 grouped-3 VLC,
    /// context pair.
    Q2,
    /// `sv8-canonical-q3` — §3.4 case-3 grouped-2 VLC.
    Q3,
    /// `sv8-canonical-q4` — §3.4 case-4 grouped-2 VLC.
    Q4,
    /// `sv8-canonical-q5-{1,2}` — §3.4 case-5 context-adaptive
    /// per-sample VLC, context pair.
    Q5,
    /// `sv8-canonical-q6-{1,2}` — §3.4 case-6 context-adaptive
    /// per-sample VLC, context pair.
    Q6,
    /// `sv8-canonical-q7-{1,2}` — §3.4 case-7 context-adaptive
    /// per-sample VLC, context pair.
    Q7,
    /// `sv8-canonical-q8-{1,2}` — §3.4 case-8 context-adaptive
    /// per-sample VLC, context pair.
    Q8,
    /// `sv8-canonical-q9up` — §3.4 default-arm large-coefficient
    /// escape VLC.
    Q9up,
}

#[cfg(test)]
mod tests {
    use super::*;

    // ─── Catalogue shape ───────────────────────────────────

    #[test]
    fn catalogue_has_21_entries() {
        // Per spec §4: 21 length-tables + 21 symbol maps.
        assert_eq!(SV8_CANONICAL_CATALOGUE.len(), 21);
    }

    #[test]
    fn catalogue_names_are_unique_and_sv8_prefixed() {
        let mut names: Vec<&str> = SV8_CANONICAL_CATALOGUE.iter().map(|t| t.name).collect();
        names.sort_unstable();
        names.dedup();
        assert_eq!(
            names.len(),
            21,
            "21 distinct canonical-table names expected"
        );
        for t in SV8_CANONICAL_CATALOGUE.iter() {
            assert!(
                t.name.starts_with("sv8-canonical-"),
                "catalogue name {:?} should start with sv8-canonical-",
                t.name
            );
        }
    }

    // ─── Per-table row counts (sanity vs staged .meta) ─────

    #[test]
    fn bands_table_shape() {
        // sv8-canonical-bands.meta: resolved_dims: [12]
        // sv8-symbols-bands.meta:   resolved_dims: [33]
        assert_eq!(SV8_BANDS_LEN_TABLE.len(), 12);
        assert_eq!(SV8_BANDS_SYM_TABLE.len(), 33);
        // First and last rows per the staged CSV.
        assert_eq!(
            SV8_BANDS_LEN_TABLE[0],
            Sv8CanonicalEntry {
                code: 0x8000,
                length: 1,
                cum_index: 1
            },
        );
        assert_eq!(
            *SV8_BANDS_LEN_TABLE.last().unwrap(),
            Sv8CanonicalEntry {
                code: 0x0000,
                length: 13,
                cum_index: 32
            },
        );
        // Symbol map endpoints.
        assert_eq!(SV8_BANDS_SYM_TABLE[0], 0);
        assert_eq!(*SV8_BANDS_SYM_TABLE.last().unwrap(), 13);
    }

    #[test]
    fn res_context_pair_shapes() {
        // sv8-canonical-res-1.meta: [16]; sv8-canonical-res-2.meta: [12]
        // sv8-symbols-res-{1,2}.meta: [17] each.
        assert_eq!(SV8_RES_1_LEN_TABLE.len(), 16);
        assert_eq!(SV8_RES_2_LEN_TABLE.len(), 12);
        assert_eq!(SV8_RES_1_SYM_TABLE.len(), 17);
        assert_eq!(SV8_RES_2_SYM_TABLE.len(), 17);
    }

    #[test]
    fn scfi_context_pair_shapes() {
        // sv8-canonical-scfi-1.meta: [3]; sv8-canonical-scfi-2.meta: [5]
        // sv8-symbols-scfi-1.meta: [4]; sv8-symbols-scfi-2.meta: [16]
        assert_eq!(SV8_SCFI_1_LEN_TABLE.len(), 3);
        assert_eq!(SV8_SCFI_2_LEN_TABLE.len(), 5);
        assert_eq!(SV8_SCFI_1_SYM_TABLE.len(), 4);
        assert_eq!(SV8_SCFI_2_SYM_TABLE.len(), 16);
    }

    #[test]
    fn dscf_context_pair_shapes() {
        // sv8-canonical-dscf-1.meta: [12]; sv8-canonical-dscf-2.meta: [13]
        // sv8-symbols-dscf-1.meta: [64];   sv8-symbols-dscf-2.meta: [65]
        assert_eq!(SV8_DSCF_1_LEN_TABLE.len(), 12);
        assert_eq!(SV8_DSCF_2_LEN_TABLE.len(), 13);
        assert_eq!(SV8_DSCF_1_SYM_TABLE.len(), 64);
        assert_eq!(SV8_DSCF_2_SYM_TABLE.len(), 65);
    }

    #[test]
    fn q1_q3_q4_simple_table_shapes() {
        // sv8-canonical-q1: [10] / sv8-symbols-q1: [19]
        assert_eq!(SV8_Q1_LEN_TABLE.len(), 10);
        assert_eq!(SV8_Q1_SYM_TABLE.len(), 19);
        // sv8-canonical-q3: [7] / sv8-symbols-q3: [49]
        assert_eq!(SV8_Q3_LEN_TABLE.len(), 7);
        assert_eq!(SV8_Q3_SYM_TABLE.len(), 49);
        // sv8-canonical-q4: [8] / sv8-symbols-q4: [91]
        assert_eq!(SV8_Q4_LEN_TABLE.len(), 8);
        assert_eq!(SV8_Q4_SYM_TABLE.len(), 91);
    }

    #[test]
    fn q2_through_q8_context_pair_shapes() {
        // Per the staged `.meta` resolved_dims.
        assert_eq!(SV8_Q2_1_LEN_TABLE.len(), 10);
        assert_eq!(SV8_Q2_2_LEN_TABLE.len(), 9);
        assert_eq!(SV8_Q2_1_SYM_TABLE.len(), 125);
        assert_eq!(SV8_Q2_2_SYM_TABLE.len(), 125);
        assert_eq!(SV8_Q5_1_LEN_TABLE.len(), 6);
        assert_eq!(SV8_Q5_2_LEN_TABLE.len(), 4);
        assert_eq!(SV8_Q5_1_SYM_TABLE.len(), 15);
        assert_eq!(SV8_Q5_2_SYM_TABLE.len(), 15);
        assert_eq!(SV8_Q6_1_LEN_TABLE.len(), 8);
        assert_eq!(SV8_Q6_2_LEN_TABLE.len(), 5);
        assert_eq!(SV8_Q6_1_SYM_TABLE.len(), 31);
        assert_eq!(SV8_Q6_2_SYM_TABLE.len(), 31);
        assert_eq!(SV8_Q7_1_LEN_TABLE.len(), 9);
        assert_eq!(SV8_Q7_2_LEN_TABLE.len(), 5);
        assert_eq!(SV8_Q7_1_SYM_TABLE.len(), 63);
        assert_eq!(SV8_Q7_2_SYM_TABLE.len(), 63);
        assert_eq!(SV8_Q8_1_LEN_TABLE.len(), 11);
        assert_eq!(SV8_Q8_2_LEN_TABLE.len(), 4);
        assert_eq!(SV8_Q8_1_SYM_TABLE.len(), 127);
        assert_eq!(SV8_Q8_2_SYM_TABLE.len(), 127);
    }

    #[test]
    fn q9up_escape_table_shape() {
        // sv8-canonical-q9up.meta: [6] (six length-class rows)
        // sv8-symbols-q9up.meta:   [256] (signed −128..=127)
        assert_eq!(SV8_Q9UP_LEN_TABLE.len(), 6);
        assert_eq!(SV8_Q9UP_SYM_TABLE.len(), 256);
        // Per provenance.md §6: "signed −128..127".
        let min = *SV8_Q9UP_SYM_TABLE.iter().min().unwrap();
        let max = *SV8_Q9UP_SYM_TABLE.iter().max().unwrap();
        assert!(
            (-128..=127).contains(&min) && (-128..=127).contains(&max),
            "q9up symbol range stays in int8 ({min}..={max})",
        );
    }

    // ─── Per-row invariants ────────────────────────────────

    /// Iterate the rows of `t` that are not the staged
    /// `length == 0` sentinel — the one anomalous case is
    /// `sv8-canonical-q4`, whose staged CSV carries a final
    /// `(0x0000, 0, 90)` sentinel row after the real length-10
    /// terminator. The sentinel is preserved verbatim in the
    /// static array (the CSV is the source of truth) but skipped
    /// by the row-invariant tests, which only describe the
    /// canonical Huffman rows themselves.
    fn data_rows(t: &Sv8CanonicalTable) -> impl Iterator<Item = &Sv8CanonicalEntry> {
        t.lengths.iter().filter(|e| e.length != 0)
    }

    #[test]
    fn data_rows_are_sorted_code_descending() {
        for t in SV8_CANONICAL_CATALOGUE.iter() {
            let rows: Vec<&Sv8CanonicalEntry> = data_rows(t).collect();
            for w in rows.windows(2) {
                assert!(
                    w[0].code > w[1].code,
                    "{}: data rows must be sorted by code descending; got {:#06x} → {:#06x}",
                    t.name,
                    w[0].code,
                    w[1].code,
                );
            }
        }
    }

    #[test]
    fn data_row_lengths_are_non_decreasing() {
        // Equivalent to "code descending" for canonical tables but
        // worth pinning explicitly so a future row insertion can't
        // sneak a length-class reversal past CI.
        for t in SV8_CANONICAL_CATALOGUE.iter() {
            let rows: Vec<&Sv8CanonicalEntry> = data_rows(t).collect();
            for w in rows.windows(2) {
                assert!(
                    w[0].length <= w[1].length,
                    "{}: data-row length must be non-decreasing as code descends; got {} → {}",
                    t.name,
                    w[0].length,
                    w[1].length,
                );
            }
        }
    }

    #[test]
    fn data_row_lengths_within_16_bits() {
        // The mpc_huffman peek window is 16 bits. Data rows (i.e.
        // ignoring the q4 length-0 sentinel) must each declare a
        // length in `1..=16`.
        for t in SV8_CANONICAL_CATALOGUE.iter() {
            for e in data_rows(t) {
                assert!(
                    (1..=16).contains(&e.length),
                    "{}: data-row length {} outside 1..=16 (16-bit mpc_huffman peek)",
                    t.name,
                    e.length,
                );
            }
        }
    }

    #[test]
    fn last_data_row_terminates_at_zero_code() {
        // The canonical-Huffman descending row order ends at the
        // smallest possible code (`0x0000`), so a peek of all
        // zeros always matches the final row. This is required for
        // a total decoder (no "no match" edge for any 16-bit peek).
        // Tested against the last *data* row to tolerate q4's
        // length-0 sentinel.
        for t in SV8_CANONICAL_CATALOGUE.iter() {
            let last_data = data_rows(t).last().unwrap();
            assert_eq!(
                last_data.code, 0x0000,
                "{}: last data row code must be 0x0000 so a peek of all zeros matches",
                t.name,
            );
        }
    }

    #[test]
    fn code_left_justified_into_length_bits() {
        // The low `16 - length` bits of `code` must be zero per the
        // staged `.meta` `value_encoding` line ("Code: uint16
        // left-adjusted code prefix"). Only meaningful for data
        // rows; the q4 length-0 sentinel has all 16 bits of `code`
        // notionally "tail" but is 0x0000 so the check is trivially
        // satisfied — we run it on all rows for safety.
        for t in SV8_CANONICAL_CATALOGUE.iter() {
            for e in t.lengths.iter() {
                let tail_bits = 16u32 - e.length as u32;
                let tail_mask: u32 = if tail_bits >= 16 {
                    0xFFFF
                } else {
                    (1u32 << tail_bits) - 1
                };
                assert_eq!(
                    e.code as u32 & tail_mask,
                    0,
                    "{}: code {:#06x} length {} has non-zero low bits ({:#x} masked)",
                    t.name,
                    e.code,
                    e.length,
                    e.code as u32 & tail_mask,
                );
            }
        }
    }

    #[test]
    fn data_row_cum_index_progresses_per_table() {
        // For the unsigned-cumulative tables (every table except
        // q9up), the cumulative index strictly increases as we
        // walk the data rows in row order. q9up's cumulative index
        // wraps the signed-int8 space (..., 63, 125, -45, -7, -2,
        // -1) per the staged CSV, so the strict-increase check
        // only applies to int8 *unsigned* reinterpretation, which
        // is equivalent to "the high bit may flip exactly once".
        for t in SV8_CANONICAL_CATALOGUE.iter() {
            let rows: Vec<&Sv8CanonicalEntry> = data_rows(t).collect();
            if t.name == "sv8-canonical-q9up" {
                // q9up: progress under int8-as-unsigned wrap.
                for w in rows.windows(2) {
                    let a = (w[0].cum_index as i8) as u8;
                    let b = (w[1].cum_index as i8) as u8;
                    assert!(
                        a < b,
                        "{}: cum_index must progress mod 256; got {} → {}",
                        t.name,
                        w[0].cum_index,
                        w[1].cum_index,
                    );
                }
            } else {
                for w in rows.windows(2) {
                    assert!(
                        w[0].cum_index < w[1].cum_index,
                        "{}: cum_index must strictly increase; got {} → {}",
                        t.name,
                        w[0].cum_index,
                        w[1].cum_index,
                    );
                }
            }
        }
    }

    #[test]
    fn q4_sentinel_row_is_documented() {
        // Pin the q4 length-0 sentinel for future-reader clarity.
        // The staged CSV's last row is (0x0000, 0, 90); the real
        // length-10 terminator is the row before it. The sentinel
        // carries `cum_index = 90`, matching the 91-entry q4
        // symbol map's 0-based maximum index. Whether the §3.4
        // decoder needs this sentinel value (e.g. as a sanity
        // bound on the cumulative walk) is DOCS-GAP.
        let last = SV8_Q4_LEN_TABLE.last().unwrap();
        assert_eq!(
            *last,
            Sv8CanonicalEntry {
                code: 0x0000,
                length: 0,
                cum_index: 90
            }
        );
        // Penultimate row is the real length-10 terminator.
        let penult = SV8_Q4_LEN_TABLE[SV8_Q4_LEN_TABLE.len() - 2];
        assert_eq!(
            penult,
            Sv8CanonicalEntry {
                code: 0x0000,
                length: 10,
                cum_index: 80
            }
        );
        // q4 is the only catalogue entry with a length-0 sentinel.
        let count = SV8_CANONICAL_CATALOGUE
            .iter()
            .filter(|t| t.lengths.iter().any(|e| e.length == 0))
            .count();
        assert_eq!(
            count, 1,
            "exactly one staged table carries a length-0 sentinel (q4)",
        );
    }

    // ─── Helper accessors ──────────────────────────────────

    #[test]
    fn min_max_length_helpers() {
        assert_eq!(SV8_BANDS_TABLE.min_length(), 1);
        assert_eq!(SV8_BANDS_TABLE.max_length(), 13);
        // q1 first row is length 3 per CSV (no length-1/2 rows).
        assert_eq!(SV8_Q1_TABLE.min_length(), 3);
        assert_eq!(SV8_Q1_TABLE.max_length(), 12);
    }

    #[test]
    fn table_for_role_simple_tables_ignore_ctx() {
        // Tables without a context split return the same physical
        // table for any ctx value.
        for ctx in 0..=3u8 {
            assert!(std::ptr::eq(
                table_for_role(Sv8TableRole::Bands, ctx).unwrap(),
                &SV8_BANDS_TABLE,
            ));
            assert!(std::ptr::eq(
                table_for_role(Sv8TableRole::Q1, ctx).unwrap(),
                &SV8_Q1_TABLE,
            ));
            assert!(std::ptr::eq(
                table_for_role(Sv8TableRole::Q3, ctx).unwrap(),
                &SV8_Q3_TABLE,
            ));
            assert!(std::ptr::eq(
                table_for_role(Sv8TableRole::Q4, ctx).unwrap(),
                &SV8_Q4_TABLE,
            ));
            assert!(std::ptr::eq(
                table_for_role(Sv8TableRole::Q9up, ctx).unwrap(),
                &SV8_Q9UP_TABLE,
            ));
        }
    }

    #[test]
    fn table_for_role_context_pair_dispatches() {
        // Each context-pair role returns its `-1` table for ctx=0
        // and its `-2` table for ctx=1.
        for &(role, t0, t1) in &[
            (Sv8TableRole::Res, &SV8_RES_1_TABLE, &SV8_RES_2_TABLE),
            (Sv8TableRole::Scfi, &SV8_SCFI_1_TABLE, &SV8_SCFI_2_TABLE),
            (Sv8TableRole::Dscf, &SV8_DSCF_1_TABLE, &SV8_DSCF_2_TABLE),
            (Sv8TableRole::Q2, &SV8_Q2_1_TABLE, &SV8_Q2_2_TABLE),
            (Sv8TableRole::Q5, &SV8_Q5_1_TABLE, &SV8_Q5_2_TABLE),
            (Sv8TableRole::Q6, &SV8_Q6_1_TABLE, &SV8_Q6_2_TABLE),
            (Sv8TableRole::Q7, &SV8_Q7_1_TABLE, &SV8_Q7_2_TABLE),
            (Sv8TableRole::Q8, &SV8_Q8_1_TABLE, &SV8_Q8_2_TABLE),
        ] {
            assert!(std::ptr::eq(table_for_role(role, 0).unwrap(), t0,));
            assert!(std::ptr::eq(table_for_role(role, 1).unwrap(), t1,));
        }
    }

    #[test]
    fn table_for_role_rejects_oor_context() {
        // Context-pair roles return None for ctx >= 2.
        for role in [
            Sv8TableRole::Res,
            Sv8TableRole::Scfi,
            Sv8TableRole::Dscf,
            Sv8TableRole::Q2,
            Sv8TableRole::Q5,
            Sv8TableRole::Q6,
            Sv8TableRole::Q7,
            Sv8TableRole::Q8,
        ] {
            assert!(table_for_role(role, 2).is_none());
            assert!(table_for_role(role, 255).is_none());
        }
    }

    #[test]
    fn sv8_canonical_table_metadata_carries_csv_stem() {
        // The `name` field is the staged CSV stem; downstream
        // diagnostic / trace logs can use it as-is.
        assert_eq!(SV8_BANDS_TABLE.name, "sv8-canonical-bands");
        assert_eq!(SV8_Q9UP_TABLE.name, "sv8-canonical-q9up");
        assert_eq!(SV8_RES_1_TABLE.name, "sv8-canonical-res-1");
        assert_eq!(SV8_RES_2_TABLE.name, "sv8-canonical-res-2");
    }

    #[test]
    fn sv8_canonical_table_row_helpers() {
        // The const helpers report the same lengths the static
        // arrays do.
        assert_eq!(SV8_BANDS_TABLE.len_table_rows(), SV8_BANDS_LEN_TABLE.len());
        assert_eq!(SV8_BANDS_TABLE.sym_table_rows(), SV8_BANDS_SYM_TABLE.len());
        assert_eq!(SV8_Q9UP_TABLE.sym_table_rows(), 256);
    }

    // ─── Symbol-map content spot-checks ────────────────────

    #[test]
    fn bands_symbol_map_spans_zero_to_thirty_two() {
        // sv8-symbols-bands is described as "0..32" in the
        // provenance §6 row for the used-subbands map.
        let mut seen = [false; 33];
        for &s in SV8_BANDS_SYM_TABLE.iter() {
            assert!(s >= 0, "bands symbol map must be non-negative ({s})");
            seen[s as usize] = true;
        }
        for (i, hit) in seen.iter().enumerate() {
            assert!(*hit, "bands symbol map should cover {i}");
        }
    }

    #[test]
    fn q9up_symbol_map_endpoints_match_csv() {
        // sv8-symbols-q9up first and last entries per the staged
        // CSV (256 entries spanning the int8 range, walking the
        // sign-magnitude ladder the SV8 escape path uses).
        assert_eq!(SV8_Q9UP_SYM_TABLE[0], -128);
        assert_eq!(*SV8_Q9UP_SYM_TABLE.last().unwrap(), -2);
    }
}
