//! SV8 frame-body DSCF delta-loop walk (§3.5 scalefactor delta coding).
//!
//! The §3.5 scalefactor (SCF) layer opens — for each non-zero coded band
//! — with a per-band **SCFI selector** read ([`crate::sv8_scf_header`]),
//! and then transmits **one to three delta-scalefactor (DSCF) VLCs per
//! band**: per §3.5, "a base index plus VLC-coded deltas across the
//! band's granules, reset at band boundaries", a structure inherited
//! from the SV7 SCF scheme (§2.4). This module wires that per-band DSCF
//! VLC loop on top of the staged SV8 canonical-Huffman DSCF context pair:
//!
//! - `sv8-canonical-dscf-{1,2}` (+ symbol maps `sv8-symbols-dscf-{1,2}`)
//!   — the **DSCF delta** codebook, a first-order **context pair**. Each
//!   `.meta` `spec_role` is `"SV8 §3.5 delta-scalefactor (DSCF)
//!   canonical length-table"`; the paired symbol maps span `0..=63`
//!   (variant 1, 64 entries) and `0..=64` (variant 2, 65 entries).
//!
//! ## What the §3.5 structural prose pins (and drives this module)
//!
//! 1. **The DSCF deltas follow the SCFI selector, per band, in
//!    ascending band order.** §3.5 ("SV8 keeps per-band scalefactor
//!    indices but delta-codes them with their own SV8 SCF VLC tables ...
//!    a base index plus VLC-coded deltas across the band's granules,
//!    reset at band boundaries — the *structure* is inherited from the
//!    SV7 SCF scheme §2.4") plus the `dscf` table's `spec_role`
//!    ("delta-scalefactor (DSCF)") ground the per-band DSCF read.
//! 2. **One to three deltas per band.** §2.4 (the inherited structure)
//!    pins "1 to 3 VLCs per non-zero band depending on the coding
//!    method" — the Layer-II three-granule scalefactor structure (§1).
//!    [`decode_dscf_deltas`] reads exactly the caller-supplied count for
//!    each band.
//!
//! ## What the §3.5 prose does NOT pin (caller knobs / GAP)
//!
//! - **The per-band delta count.** §2.4 ties the count (1..=3) to the
//!   SCFI coding-method value, but the SV8 SCFI value →
//!   (count, granule-mapping) table is itself GAP: the SV8 `scfi-2`
//!   symbol map spans `0..=15`, which does **not** match the four-value
//!   SV7 §2.4 SCFI schedule cell-for-cell (see
//!   [`crate::sv8_scf_header`]). Until that schedule is pinned the
//!   per-band delta count cannot be derived from the decoded SCFI value,
//!   so [`decode_dscf_deltas`] takes the count per band as a
//!   caller-supplied slice rather than guessing it.
//! - **The `dscf-1` vs `dscf-2` context-selection rule.** §3.5 ships a
//!   `{ctx0, ctx1}` pair for the DSCF codebook but the structural prose
//!   does not state the predicate choosing which half a given delta
//!   reads from. [`decode_dscf_deltas`] takes the pick as a
//!   caller-supplied closure `ctx_for_prev_dscf(previous_raw_dscf) ->
//!   ctx`, the same GAP-knob precedent
//!   [`crate::sv8_scf_header::decode_scfi_selectors`] uses for the SCFI
//!   context pair; the first delta uses a caller-supplied `initial_ctx`.
//! - **The DSCF symbol → signed-delta centring offset.** The
//!   `sv8-symbols-dscf-{1,2}` maps span `0..=63` / `0..=64` (unsigned),
//!   unlike SV7's directly-signed DSCF symbols (`-7..=8` in
//!   `sv7-huffman-dscf`). The centring offset that turns an SV8 DSCF
//!   symbol into a signed delta — and hence the base-plus-delta SCF
//!   index reconstruction (§2.4's `scf[g] = scf[g-1] + dscf`) — is GAP,
//!   so this module returns the raw VLC value wrapped in [`RawDscfVlc`]
//!   and applies no arithmetic. This is the exact analogue of the
//!   [`crate::sv8_scf_header::RawScfiVlc`] and
//!   [`crate::sv8_band_header::RawResVlc`] raw wrappers.
//!
//! ## Source-of-record
//!
//! - Structural prose: `docs/audio/musepack/musepack-sv7-sv8-spec.md`
//!   §3.5 (SV8 SCF coding) + §2.4 / §1 (the inherited SV7 SCF structure
//!   and Layer-II three-granule scalefactor layout).
//! - Table roles: the `.meta` `spec_role` lines of
//!   `sv8-canonical-dscf-{1,2}` / `sv8-symbols-dscf-{1,2}` under
//!   `docs/audio/musepack/tables/`, and provenance §6.
//! - Canonical-Huffman decode walk:
//!   [`crate::sv8_huffman::Sv8CanonicalTable::decode`].
//!
//! The only project material crossed is the staged `docs/` content
//! above and the sibling modules under
//! `crates/oxideav-musepack/src/`.

use crate::huffman::Sv7BitReader;
use crate::sv8_huffman::{table_for_role, Sv8TableRole};
use crate::{Error, Result};

/// Typed wrapper around the raw `i8` value produced by a single
/// invocation of the `sv8-canonical-dscf-{1,2}` DSCF delta VLC.
///
/// The wrapper keeps the `dscf → signed-delta` mapping honest: the
/// staged SV8 DSCF symbol maps span `0..=63` (variant 1) and `0..=64`
/// (variant 2) — **unsigned** — unlike SV7's directly-signed DSCF
/// symbols (`-7..=8`). The SV8 symbol → signed-delta centring offset is
/// DOCS-GAP, so a caller must apply an explicit, not-yet-pinned centring
/// step before adding the delta to the running SCF index; the distinct
/// type prevents accidental use of the raw unsigned value as a signed
/// delta.
///
/// This is the SV8 DSCF sibling of
/// [`crate::sv8_scf_header::RawScfiVlc`] and
/// [`crate::sv8_band_header::RawResVlc`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RawDscfVlc(i8);

impl RawDscfVlc {
    /// Wrap a raw `i8` VLC symbol. No validity check beyond the `i8`
    /// range — the staged `dscf` maps guarantee values in `0..=64`, but
    /// nothing in §3.5 elevates that to a hard invariant.
    pub const fn from_raw(value: i8) -> Self {
        Self(value)
    }

    /// Expose the underlying `i8` value: the raw input to the
    /// upstream-pending SV8 DSCF symbol → signed-delta centring step and
    /// to the caller-supplied context-selection predicate.
    pub const fn as_i8(self) -> i8 {
        self.0
    }
}

/// Sentinel band_type payload reported when the caller's
/// context-selection rule yields an out-of-range `ctx`. `i8::MIN`
/// (`-128`) is outside the §3.4 enumerated `band_type` domain
/// (`-1..=17`) and every staged DSCF alphabet (`0..=64`), so it cannot
/// collide with a genuine band_type rejection. Mirrors the sentinel
/// [`crate::sv8_scf_header::CONTEXT_FAULT_SENTINEL`].
pub const CONTEXT_FAULT_SENTINEL: i8 = i8::MIN;

/// Walk the §3.5 per-band DSCF delta loop: for each of `nbands` coded
/// bands, read `deltas_per_band[band]` `sv8-canonical-dscf-{1,2}`
/// codewords in ascending band order, returning per band the raw VLC
/// value of each delta wrapped in [`RawDscfVlc`].
///
/// The returned outer `Vec` has exactly `nbands` entries, one per band;
/// the inner `Vec` for band `b` has `deltas_per_band[b]` entries (the
/// caller-supplied 1..=3 count, which the GAP SV8 SCFI schedule would
/// otherwise determine — see the module docs).
///
/// Context-pair selection (the GAP knob): the first DSCF read of the
/// whole walk uses the `initial_ctx` half of the DSCF pair; each
/// subsequent DSCF read uses `ctx_for_prev_dscf(previous_raw_dscf)`. The
/// context carries across band boundaries — §3.5 resets the *base index*
/// at band boundaries, not the entropy context, which the structural
/// prose does not scope per-band — so the predicate sees the previous
/// delta regardless of whether it sat in the same band. A context value
/// outside `0..=1` (from either source) yields
/// [`Error::UnsupportedBandType`] carrying [`CONTEXT_FAULT_SENTINEL`].
///
/// Errors:
///
/// - [`Error::UnexpectedEof`] mid-walk if the reader starves.
/// - [`Error::HuffmanNoMatch`] if a DSCF peek matches no row
///   (unreachable for the staged dscf tables).
/// - [`Error::UnsupportedBandType`] (payload [`CONTEXT_FAULT_SENTINEL`])
///   if the context-selection rule produces a `ctx` outside `0..=1`.
pub fn decode_dscf_deltas<F>(
    reader: &mut Sv7BitReader<'_>,
    deltas_per_band: &[u8],
    initial_ctx: u8,
    mut ctx_for_prev_dscf: F,
) -> Result<Vec<Vec<RawDscfVlc>>>
where
    F: FnMut(RawDscfVlc) -> u8,
{
    let mut out: Vec<Vec<RawDscfVlc>> = Vec::with_capacity(deltas_per_band.len());
    let mut ctx = initial_ctx;
    for &count in deltas_per_band {
        let mut band: Vec<RawDscfVlc> = Vec::with_capacity(count as usize);
        for _ in 0..count {
            let table = table_for_role(Sv8TableRole::Dscf, ctx)
                .ok_or(Error::UnsupportedBandType(CONTEXT_FAULT_SENTINEL))?;
            let raw = RawDscfVlc::from_raw(table.decode(reader)?);
            band.push(raw);
            ctx = ctx_for_prev_dscf(raw);
        }
        out.push(band);
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sv8_huffman::{Sv8CanonicalTable, SV8_DSCF_1_TABLE, SV8_DSCF_2_TABLE};

    /// MSB-first left-justified bit packer (mirrors the SV8 SCFI-header
    /// tests): push a `length`-bit codeword from the top of `pattern`;
    /// `finish` flushes + appends two zero bytes so `peek16` never
    /// starves mid-decode.
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

        fn finish(mut self) -> Vec<u8> {
            if self.nbits > 0 {
                self.bytes.push((self.acc << (8 - self.nbits)) as u8);
            }
            self.bytes.push(0);
            self.bytes.push(0);
            self.bytes
        }
    }

    /// Decode `table`'s row-`r` codeword once, returning the symbol.
    fn symbol_for_row(table: &Sv8CanonicalTable, r: usize) -> i8 {
        let e = table.lengths[r];
        let mut p = BitPacker::new();
        p.push(e.code, e.length);
        let bytes = p.finish();
        let mut reader = Sv7BitReader::new(&bytes);
        table.decode(&mut reader).expect("single-codeword decode")
    }

    // ─── RawDscfVlc newtype ─────────────────────────────────

    #[test]
    fn raw_dscf_vlc_roundtrips() {
        for v in [-1_i8, 0, 1, 31, 63, 64, i8::MIN, i8::MAX] {
            let w = RawDscfVlc::from_raw(v);
            assert_eq!(w.as_i8(), v);
        }
    }

    #[test]
    fn raw_dscf_vlc_is_copy_and_eq() {
        let a = RawDscfVlc::from_raw(7);
        let b = a;
        assert_eq!(a, b);
        assert_ne!(a, RawDscfVlc::from_raw(8));
    }

    // ─── Single-band, single-delta against each context half ──

    #[test]
    fn one_band_one_delta_uses_initial_ctx_zero_variant1() {
        // ctx 0 → dscf-1 (variant 1). Decode its first row.
        let want = symbol_for_row(&SV8_DSCF_1_TABLE, 0);
        let e = SV8_DSCF_1_TABLE.lengths[0];
        let mut p = BitPacker::new();
        p.push(e.code, e.length);
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let got = decode_dscf_deltas(&mut r, &[1], 0, |_| 0).unwrap();
        assert_eq!(got.len(), 1);
        assert_eq!(got[0], vec![RawDscfVlc::from_raw(want)]);
    }

    #[test]
    fn one_band_one_delta_uses_initial_ctx_one_variant2() {
        // ctx 1 → dscf-2 (variant 2). Decode its first row.
        let want = symbol_for_row(&SV8_DSCF_2_TABLE, 0);
        let e = SV8_DSCF_2_TABLE.lengths[0];
        let mut p = BitPacker::new();
        p.push(e.code, e.length);
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let got = decode_dscf_deltas(&mut r, &[1], 1, |_| 1).unwrap();
        assert_eq!(got[0], vec![RawDscfVlc::from_raw(want)]);
    }

    // ─── Three deltas in one band (1..=3 per-band count) ─────

    #[test]
    fn one_band_three_deltas_reads_all_three_in_order() {
        // A band with the worst-case three granule deltas (SCFI count
        // 3, §2.4 inherited). All ctx 0 → dscf-1, three distinct rows.
        let rows = [0usize, 2, 4];
        let mut p = BitPacker::new();
        let mut wants = Vec::new();
        for &row in &rows {
            let e = SV8_DSCF_1_TABLE.lengths[row];
            p.push(e.code, e.length);
            wants.push(RawDscfVlc::from_raw(symbol_for_row(&SV8_DSCF_1_TABLE, row)));
        }
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let got = decode_dscf_deltas(&mut r, &[3], 0, |_| 0).unwrap();
        assert_eq!(got.len(), 1);
        assert_eq!(got[0], wants);
    }

    // ─── Multi-band walk with varying per-band counts ────────

    #[test]
    fn two_bands_varying_counts_partition_correctly() {
        // Band 0: 2 deltas; band 1: 1 delta. All ctx 0 → dscf-1.
        let rows = [1usize, 3, 5];
        let mut p = BitPacker::new();
        let mut sym = Vec::new();
        for &row in &rows {
            let e = SV8_DSCF_1_TABLE.lengths[row];
            p.push(e.code, e.length);
            sym.push(RawDscfVlc::from_raw(symbol_for_row(&SV8_DSCF_1_TABLE, row)));
        }
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let got = decode_dscf_deltas(&mut r, &[2, 1], 0, |_| 0).unwrap();
        assert_eq!(got.len(), 2);
        assert_eq!(got[0], vec![sym[0], sym[1]]);
        assert_eq!(got[1], vec![sym[2]]);
    }

    // ─── Context switching across deltas (and band boundary) ─

    #[test]
    fn context_switches_from_first_delta_to_second() {
        // Delta 0 reads dscf-1 (initial ctx 0); the closure flips to
        // ctx 1 so delta 1 reads dscf-2. Single band, two deltas.
        let e0 = SV8_DSCF_1_TABLE.lengths[1];
        let e1 = SV8_DSCF_2_TABLE.lengths[1];
        let want0 = symbol_for_row(&SV8_DSCF_1_TABLE, 1);
        let want1 = symbol_for_row(&SV8_DSCF_2_TABLE, 1);
        let mut p = BitPacker::new();
        p.push(e0.code, e0.length);
        p.push(e1.code, e1.length);
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let mut seen_prev: Vec<RawDscfVlc> = Vec::new();
        let got = decode_dscf_deltas(&mut r, &[2], 0, |prev| {
            seen_prev.push(prev);
            1
        })
        .unwrap();
        assert_eq!(
            got[0],
            vec![RawDscfVlc::from_raw(want0), RawDscfVlc::from_raw(want1)]
        );
        // The closure runs once per delta read.
        assert_eq!(
            seen_prev,
            vec![RawDscfVlc::from_raw(want0), RawDscfVlc::from_raw(want1)]
        );
    }

    #[test]
    fn context_carries_across_band_boundary() {
        // Band 0 has one delta (ctx 0 → dscf-1); the closure flips to
        // ctx 1, and band 1's first delta must read dscf-2 — proving the
        // entropy context is not reset at the band boundary (only the
        // base SCF index is, per §3.5, which this raw walk does not
        // touch).
        let e0 = SV8_DSCF_1_TABLE.lengths[0];
        let e1 = SV8_DSCF_2_TABLE.lengths[0];
        let want0 = symbol_for_row(&SV8_DSCF_1_TABLE, 0);
        let want1 = symbol_for_row(&SV8_DSCF_2_TABLE, 0);
        let mut p = BitPacker::new();
        p.push(e0.code, e0.length);
        p.push(e1.code, e1.length);
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let got = decode_dscf_deltas(&mut r, &[1, 1], 0, |_| 1).unwrap();
        assert_eq!(got[0], vec![RawDscfVlc::from_raw(want0)]);
        assert_eq!(got[1], vec![RawDscfVlc::from_raw(want1)]);
    }

    // ─── Zero-work paths ─────────────────────────────────────

    #[test]
    fn empty_band_list_reads_nothing() {
        let bytes = [0u8, 0, 0, 0];
        let mut r = Sv7BitReader::new(&bytes);
        let got = decode_dscf_deltas(&mut r, &[], 0, |_| {
            panic!("closure must not run for an empty band list")
        })
        .unwrap();
        assert!(got.is_empty());
    }

    #[test]
    fn zero_count_band_yields_empty_inner_vec() {
        // A band with a zero delta count contributes an empty inner vec
        // and reads no bits; band 1 then decodes normally.
        let e = SV8_DSCF_1_TABLE.lengths[0];
        let want = symbol_for_row(&SV8_DSCF_1_TABLE, 0);
        let mut p = BitPacker::new();
        p.push(e.code, e.length);
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let got = decode_dscf_deltas(&mut r, &[0, 1], 0, |_| 0).unwrap();
        assert_eq!(got.len(), 2);
        assert!(got[0].is_empty());
        assert_eq!(got[1], vec![RawDscfVlc::from_raw(want)]);
    }

    // ─── Context-fault + EOF paths ──────────────────────────

    #[test]
    fn out_of_range_initial_ctx_is_context_fault() {
        let bytes = [0xFFu8, 0xFF, 0, 0];
        let mut r = Sv7BitReader::new(&bytes);
        assert_eq!(
            decode_dscf_deltas(&mut r, &[1], 2, |_| 0),
            Err(Error::UnsupportedBandType(CONTEXT_FAULT_SENTINEL))
        );
    }

    #[test]
    fn out_of_range_ctx_from_closure_is_context_fault_on_second_delta() {
        // Delta 0 decodes fine (ctx 0); the closure returns an invalid
        // ctx 5 so delta 1's table lookup faults.
        let e0 = SV8_DSCF_1_TABLE.lengths[0];
        let mut p = BitPacker::new();
        p.push(e0.code, e0.length);
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        assert_eq!(
            decode_dscf_deltas(&mut r, &[2], 0, |_| 5),
            Err(Error::UnsupportedBandType(CONTEXT_FAULT_SENTINEL))
        );
    }

    #[test]
    fn propagates_unexpected_eof_mid_walk() {
        let bytes: [u8; 0] = [];
        let mut r = Sv7BitReader::new(&bytes);
        assert_eq!(
            decode_dscf_deltas(&mut r, &[1], 0, |_| 0),
            Err(Error::UnexpectedEof)
        );
    }

    #[test]
    fn context_fault_sentinel_is_outside_band_type_and_dscf_domains() {
        // Defensive pin: the sentinel must not collide with any genuine
        // band_type (-1..=17) nor any staged dscf symbol (0..=64).
        assert_eq!(CONTEXT_FAULT_SENTINEL, i8::MIN);
        for v in -1i8..=17 {
            assert_ne!(v, CONTEXT_FAULT_SENTINEL);
        }
        for v in 0i8..=64 {
            assert_ne!(v, CONTEXT_FAULT_SENTINEL);
        }
    }
}
