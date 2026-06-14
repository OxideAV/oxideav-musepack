//! SV8 frame-body band-resolution header walk (§3.4 outer loop).
//!
//! The §3.4 audio-packet frame body opens by deciding **how many
//! subbands are coded** and, per coded band, **what band resolution
//! (`band_type`) that band uses** — before the per-band sample
//! `switch (band_type)` ladder reproduced in
//! [`crate::sv8_band_decode`] runs. This module wires that outer
//! header walk on top of two of the staged SV8 canonical-Huffman
//! tables:
//!
//! - `sv8-canonical-bands` (+ its symbol map `sv8-symbols-bands`) —
//!   the **number-of-used-subbands** selector. Its `.meta`
//!   `spec_role` is `"SV8 spec §3.4 number-of-used-subbands
//!   canonical length-table"` and the paired symbol map's role is
//!   `"SV8 §3.4 used-subbands symbol map (0..32)"`: one VLC decodes
//!   to a count in `0..=32`, the inclusive Layer-II subband bound
//!   (§1 — 32 polyphase subbands per channel).
//! - `sv8-canonical-res-{1,2}` (+ symbol maps `sv8-symbols-res-{1,2}`)
//!   — the **band-resolution** selector, a first-order **context
//!   pair**. Each `.meta` `spec_role` is `"SV8 §3.4 band-resolution
//!   (band_type) canonical length-table"`; the paired symbol maps
//!   span `0..=16`.
//!
//! ## What the §3.4 structural prose pins (and drives this module)
//!
//! 1. **A used-subbands count precedes the per-band loop.** The
//!    `bands` table's `spec_role` names it the
//!    "number-of-used-subbands" selector, and §1 grounds the count's
//!    `0..=32` range (32 subbands, the band loop never exceeds them).
//!    [`decode_used_subbands`] reads that single VLC and validates
//!    the decoded count against the [`SV8_MAX_USED_SUBBANDS`]
//!    inclusive bound.
//! 2. **Each coded band carries a band-resolution VLC.** The `res`
//!    table's `spec_role` ("band-resolution (`band_type`)") plus the
//!    §3.4 `switch (band_type)` ladder ground that the per-band
//!    decode is keyed by a per-band resolution read from a VLC. The
//!    walk reads exactly one `res` VLC per coded band, in ascending
//!    band order.
//!
//! ## What the §3.4 prose does NOT pin (caller knobs / GAP)
//!
//! - **The `res-1` vs `res-2` context-selection rule.** §3.4 ships a
//!   `{ctx0, ctx1}` pair for the band-resolution selector but the
//!   structural prose does not state the predicate choosing which
//!   half a given band reads from (the SV8 first-order context model
//!   is named in §3.4 for the *sample* tables `5..=8`, not spelled
//!   out for the band-resolution selector). [`decode_band_resolutions`]
//!   takes the pick as a caller-supplied closure
//!   `ctx_for_prev_res(previous_raw_res) -> ctx`, the same GAP-knob
//!   precedent the per-sample context arm uses
//!   ([`crate::sv8_sample_decode::decode_sv8_context_band`]); the
//!   first band uses a caller-supplied `initial_ctx`.
//! - **The raw-`res`-symbol → §3.4 `band_type` remap.** The `res`
//!   symbol maps span `0..=16` while the §3.4 sample `switch` ladder
//!   ranges over `band_type` in `-1..=17`. The two domains do not
//!   line up cell-for-cell, so an upstream remap is implied — but
//!   its shape (an offset, a CNS-flag escape, a delta-from-previous)
//!   is unspecified in the structural prose. This module returns the
//!   raw VLC value wrapped in [`RawResVlc`] so a caller cannot
//!   accidentally feed it straight into a
//!   [`crate::sv8_band_decode::sv8_band_type_case`] dispatcher
//!   without an explicit remap step that does not yet exist (the
//!   exact analogue of SV7's [`crate::sv7_band_header::RawBandTypeVlc`]).
//! - **Per-channel ordering / interleaving.** Unlike SV7 §2.3 (which
//!   reproduces an explicit `for ch` inner loop, "left band first,
//!   right band next"), the §3.4 prose reproduces only the per-band
//!   sample `switch` — it does not spell out a channel loop for the
//!   SV8 band-resolution header. This module therefore walks a
//!   **single band-resolution sequence** and leaves the multi-channel
//!   composition (and any per-channel `bands` count) to a future
//!   round once the SH-packet channel-count field map (§3.2, GAP) and
//!   the channel-loop shape are pinned.
//! - **The SH-packet source of the used-subbands count.** Whether the
//!   `bands` VLC count is the loop bound directly, or is itself
//!   clamped by a separately-transmitted `max_band` SH-packet field,
//!   is GAP (the §3.2 SH field map is GAP). [`decode_used_subbands`]
//!   surfaces the decoded count as-is; the caller decides how to
//!   bound the loop.
//!
//! ## Source-of-record
//!
//! - Structural prose: `docs/audio/musepack/musepack-sv7-sv8-spec.md`
//!   §3.4 (audio-packet frame body) + §1 (Layer-II 32-subband
//!   heritage).
//! - Table roles: the `.meta` `spec_role` lines of
//!   `sv8-canonical-bands` / `sv8-symbols-bands` and
//!   `sv8-canonical-res-{1,2}` / `sv8-symbols-res-{1,2}` under
//!   `docs/audio/musepack/tables/`.
//! - Canonical-Huffman decode walk:
//!   [`crate::sv8_huffman::Sv8CanonicalTable::decode`].
//!
//! The only project material crossed is the staged `docs/` content
//! above, `oxideav-core`'s public surface (none needed here), and the
//! sibling modules under `crates/oxideav-musepack/src/`.

use crate::huffman::Sv7BitReader;
use crate::sv8_huffman::{table_for_role, Sv8TableRole, SV8_BANDS_TABLE};
use crate::{Error, Result};

/// Inclusive upper bound for the §3.4 used-subbands count.
///
/// The Layer-II-inherited polyphase filterbank produces 32 subbands
/// per channel (§1), indexed `0..32`, so a frame can code at most all
/// 32 of them. The `sv8-symbols-bands` map's documented range is
/// `0..=32` (the count itself, not a band index — `0` = no coded
/// bands, `32` = all coded), matching this bound.
pub const SV8_MAX_USED_SUBBANDS: u8 = 32;

/// Typed wrapper around the raw `i8` value produced by a single
/// invocation of the `sv8-canonical-res-{1,2}` band-resolution VLC.
///
/// The wrapper keeps the `res → §3.4 band_type` mapping honest: the
/// staged `res` symbol maps span `0..=16` and do **not** cover the
/// §3.4 sample `switch` ladder's `-1..=17` `band_type` domain
/// cell-for-cell. An upstream remap is implied by the structural
/// prose (the §3.4 `switch (band_type)` reads a `band_type` that
/// includes the `-1` CNS case and ranges up to 17) but its shape is
/// DOCS-GAP. Callers must apply the remap explicitly before feeding
/// into [`crate::sv8_band_decode`]; the distinct type prevents
/// accidental composition with the §3.4 dispatcher.
///
/// This is the SV8 sibling of [`crate::sv7_band_header::RawBandTypeVlc`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RawResVlc(i8);

impl RawResVlc {
    /// Wrap a raw `i8` VLC symbol. No validity check beyond the `i8`
    /// range — the staged `res` maps guarantee values in `0..=16`,
    /// but nothing in §3.4 elevates that to a hard invariant.
    pub const fn from_raw(value: i8) -> Self {
        Self(value)
    }

    /// Expose the underlying `i8` value: the raw input to the
    /// upstream-pending `res → band_type` remap and to the
    /// caller-supplied context-selection predicate.
    pub const fn as_i8(self) -> i8 {
        self.0
    }

    /// True iff the value is structurally non-zero. The §3.4 outer
    /// loop is described as walking "each non-zero band"; a raw `res`
    /// of `0` is the natural candidate for the empty-band case (the
    /// §3.4 `case 0` "do nothing" arm), though the exact remap is
    /// GAP. This predicate reports the raw-value zero-ness only; it
    /// makes no claim about the post-remap `band_type`.
    pub const fn is_nonzero(self) -> bool {
        self.0 != 0
    }
}

/// Read the §3.4 used-subbands count: one `sv8-canonical-bands`
/// canonical-Huffman codeword, decoded to a count in
/// `0..=`[`SV8_MAX_USED_SUBBANDS`].
///
/// The `sv8-symbols-bands` map is documented `0..=32` (the
/// used-subbands count, not a band index). A decoded value above
/// [`SV8_MAX_USED_SUBBANDS`] is rejected with
/// [`Error::MaxBandOutOfRange`] — unreachable for the staged table
/// (whose symbol map maxes at 32), kept as a defensive bound for a
/// table built from another source.
///
/// Errors:
///
/// - [`Error::UnexpectedEof`] if fewer than 16 bits remain (the
///   canonical-Huffman peek window is always 16 bits).
/// - [`Error::HuffmanNoMatch`] if the peek matches no row
///   (unreachable for the staged table; see
///   [`crate::sv8_huffman::Sv8CanonicalTable::decode_symbol_index`]).
/// - [`Error::MaxBandOutOfRange`] if the decoded count exceeds 32.
pub fn decode_used_subbands(reader: &mut Sv7BitReader<'_>) -> Result<u8> {
    let symbol = SV8_BANDS_TABLE.decode(reader)?;
    // The bands symbol map is documented non-negative (0..=32); a
    // negative or >32 symbol would be a wrong-source table.
    if !(0..=SV8_MAX_USED_SUBBANDS as i8).contains(&symbol) {
        return Err(Error::MaxBandOutOfRange(symbol.unsigned_abs()));
    }
    Ok(symbol as u8)
}

/// Walk the §3.4 per-band band-resolution header: read `nbands`
/// `sv8-canonical-res-{1,2}` codewords in ascending band order,
/// returning the raw VLC value of each wrapped in [`RawResVlc`].
///
/// Context-pair selection (the GAP knob): the first band reads from
/// the `initial_ctx` half of the res pair; each subsequent band reads
/// from `ctx_for_prev_res(previous_raw_res)`. A context value outside
/// `0..=1` (from either source) yields [`Error::UnsupportedBandType`]
/// with the offending band's index folded into the existing error's
/// `i8` payload is not meaningful here, so the band-resolution error
/// reuses [`Error::UnsupportedBandType`] carrying the res VLC value
/// the caller's rule mishandled — but since the failure is on the
/// *context*, not a band_type, it reports `band_type = -128` (the
/// reserved out-of-range sentinel) so the caller can distinguish a
/// context-rule fault from a genuine band_type rejection.
///
/// `nbands` is the caller-supplied loop bound (typically the
/// [`decode_used_subbands`] count); the function reads exactly that
/// many res VLCs.
///
/// Errors:
///
/// - [`Error::UnexpectedEof`] mid-walk if the reader starves.
/// - [`Error::HuffmanNoMatch`] if a res peek matches no row
///   (unreachable for the staged res tables).
/// - [`Error::UnsupportedBandType`] (payload `-128`) if the
///   context-selection rule produces a `ctx` outside `0..=1`.
pub fn decode_band_resolutions<F>(
    reader: &mut Sv7BitReader<'_>,
    nbands: u8,
    initial_ctx: u8,
    mut ctx_for_prev_res: F,
) -> Result<Vec<RawResVlc>>
where
    F: FnMut(RawResVlc) -> u8,
{
    /// Sentinel band_type payload reported when the caller's
    /// context-selection rule yields an out-of-range `ctx`. `-128`
    /// is outside the §3.4 enumerated `band_type` domain (`-1..=17`)
    /// and the staged res alphabet (`0..=16`), so it cannot collide
    /// with a genuine band_type rejection.
    const CONTEXT_FAULT_SENTINEL: i8 = i8::MIN;

    let mut out = Vec::with_capacity(nbands as usize);
    let mut ctx = initial_ctx;
    for _ in 0..nbands {
        let role = Sv8TableRole::Res;
        let table =
            table_for_role(role, ctx).ok_or(Error::UnsupportedBandType(CONTEXT_FAULT_SENTINEL))?;
        let raw = RawResVlc::from_raw(table.decode(reader)?);
        out.push(raw);
        ctx = ctx_for_prev_res(raw);
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sv8_huffman::{Sv8CanonicalTable, SV8_RES_1_TABLE, SV8_RES_2_TABLE};

    /// MSB-first left-justified bit packer (mirrors the one used in
    /// the SV8 sample-decode tests): push a `length`-bit codeword
    /// from the top of `pattern`; `finish` flushes + appends two zero
    /// bytes so `peek16` never starves mid-decode.
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

    // ─── RawResVlc newtype ─────────────────────────────────

    #[test]
    fn raw_res_vlc_roundtrips_and_reports_zero_ness() {
        for v in [-1_i8, 0, 1, 16, 17, i8::MIN, i8::MAX] {
            let w = RawResVlc::from_raw(v);
            assert_eq!(w.as_i8(), v);
            assert_eq!(w.is_nonzero(), v != 0);
        }
    }

    #[test]
    fn raw_res_vlc_is_distinct_from_plain_i8() {
        // Compile-time: the newtype must not be confusable with the
        // §3.4 dispatcher's i8 band_type. (Type identity is enforced
        // by the signature; this test pins the Eq/Copy surface.)
        let a = RawResVlc::from_raw(3);
        let b = a; // Copy
        assert_eq!(a, b); // Eq
        let _printed = format!("{a:?}"); // Debug
    }

    // ─── decode_used_subbands ──────────────────────────────

    #[test]
    fn used_subbands_bound_constant_matches_layer_two_heritage() {
        assert_eq!(SV8_MAX_USED_SUBBANDS, 32);
    }

    #[test]
    fn used_subbands_decodes_bands_table_codewords() {
        // Each row of the bands table decodes to a used-subbands count
        // in 0..=32; the walker must surface exactly that count.
        for r in 0..SV8_BANDS_TABLE.lengths.len() {
            let want = symbol_for_row(&SV8_BANDS_TABLE, r);
            assert!(
                (0..=32).contains(&want),
                "bands row {r} symbol {want} must be a 0..=32 count",
            );
            let e = SV8_BANDS_TABLE.lengths[r];
            let mut p = BitPacker::new();
            p.push(e.code, e.length);
            let bytes = p.finish();
            let mut reader = Sv7BitReader::new(&bytes);
            assert_eq!(decode_used_subbands(&mut reader).unwrap(), want as u8);
        }
    }

    #[test]
    fn used_subbands_shortest_codeword_is_zero_count() {
        // The bands symbol map's first (most-probable, shortest-code)
        // entry is 0 — "no coded bands", the natural high-frequency
        // case for a silent/low band.
        assert_eq!(SV8_BANDS_TABLE.symbols[0], 0);
        let e = SV8_BANDS_TABLE.lengths[0];
        let mut p = BitPacker::new();
        p.push(e.code, e.length);
        let bytes = p.finish();
        let mut reader = Sv7BitReader::new(&bytes);
        assert_eq!(decode_used_subbands(&mut reader).unwrap(), 0);
    }

    #[test]
    fn used_subbands_propagates_eof() {
        let mut reader = Sv7BitReader::new(&[0xFF]);
        assert!(matches!(
            decode_used_subbands(&mut reader),
            Err(Error::UnexpectedEof),
        ));
    }

    // ─── decode_band_resolutions ───────────────────────────

    #[test]
    fn band_resolutions_reads_n_codewords_constant_ctx() {
        // With a constant ctx_for_prev_res(_) == 0, every band reads
        // from res-1; feed N copies of res-1's shortest codeword and
        // confirm the walk returns N raw values, all that symbol.
        let table = &SV8_RES_1_TABLE;
        let e = table.lengths[0];
        let want = symbol_for_row(table, 0);
        for n in [0_u8, 1, 5, 32] {
            let mut p = BitPacker::new();
            for _ in 0..n {
                p.push(e.code, e.length);
            }
            let bytes = p.finish();
            let mut reader = Sv7BitReader::new(&bytes);
            let before = reader.bits_remaining();
            let res = decode_band_resolutions(&mut reader, n, 0, |_| 0).unwrap();
            assert_eq!(res.len(), n as usize, "n = {n}");
            assert!(res.iter().all(|r| r.as_i8() == want), "n = {n}");
            assert_eq!(
                before - reader.bits_remaining(),
                n as u64 * e.length as u64,
                "n = {n}: exactly n codewords consumed",
            );
        }
    }

    #[test]
    fn band_resolutions_zero_bands_reads_nothing() {
        let mut reader = Sv7BitReader::new(&[0xFF; 4]);
        let before = reader.bits_remaining();
        let res = decode_band_resolutions(&mut reader, 0, 0, |_| 0).unwrap();
        assert!(res.is_empty());
        assert_eq!(reader.bits_remaining(), before, "0 bands reads no bits");
    }

    #[test]
    fn band_resolutions_switches_context_on_previous_res() {
        // Find a res-1 codeword decoding to symbol A and a res-2
        // codeword decoding to symbol B (B != A), then alternate them
        // under the rule "prev even -> ctx 1, prev odd -> ctx 0",
        // starting from ctx 0.
        let t0 = &SV8_RES_1_TABLE; // ctx 0
        let t1 = &SV8_RES_2_TABLE; // ctx 1
        let e0 = t0.lengths[0];
        let sym0 = symbol_for_row(t0, 0);
        let e1 = t1.lengths[0];
        let sym1 = symbol_for_row(t1, 0);

        // Build the rule: after a value with sym0's parity, pick the
        // other table. Choose a rule that yields a clean alternation
        // given sym0/sym1's actual parities.
        let rule = |prev: RawResVlc| -> u8 {
            if prev.as_i8() == sym0 {
                1
            } else {
                0
            }
        };

        // Stream: band0 (ctx0 -> sym0) then band1 (ctx1 -> sym1) then
        // band2 (rule(sym1) -> ctx0 -> sym0) ...
        let mut p = BitPacker::new();
        let n = 6_u8;
        // Pre-compute the expected ctx/symbol chain via the rule.
        let mut ctx = 0_u8;
        let mut expected = Vec::new();
        for _ in 0..n {
            let (e, sym) = if ctx == 0 { (e0, sym0) } else { (e1, sym1) };
            p.push(e.code, e.length);
            expected.push(sym);
            ctx = rule(RawResVlc::from_raw(sym));
        }
        let bytes = p.finish();
        let mut reader = Sv7BitReader::new(&bytes);
        let res = decode_band_resolutions(&mut reader, n, 0, rule).unwrap();
        let got: Vec<i8> = res.iter().map(|r| r.as_i8()).collect();
        assert_eq!(got, expected);
    }

    #[test]
    fn band_resolutions_initial_ctx_one_reads_res_2() {
        // initial_ctx == 1 must read the first band from res-2.
        let table = &SV8_RES_2_TABLE;
        let e = table.lengths[0];
        let want = symbol_for_row(table, 0);
        let mut p = BitPacker::new();
        p.push(e.code, e.length);
        let bytes = p.finish();
        let mut reader = Sv7BitReader::new(&bytes);
        let res = decode_band_resolutions(&mut reader, 1, 1, |_| 1).unwrap();
        assert_eq!(res.len(), 1);
        assert_eq!(res[0].as_i8(), want);
    }

    #[test]
    fn band_resolutions_rejects_out_of_range_initial_ctx() {
        // Bad initial_ctx fails before any bit is read.
        let mut reader = Sv7BitReader::new(&[0xFF; 8]);
        let before = reader.bits_remaining();
        let err = decode_band_resolutions(&mut reader, 1, 2, |_| 0).unwrap_err();
        assert!(matches!(err, Error::UnsupportedBandType(i8::MIN)));
        assert_eq!(reader.bits_remaining(), before, "no bits read on ctx fault");
    }

    #[test]
    fn band_resolutions_rejects_out_of_range_rule_context() {
        // A rule emitting ctx >= 2 fails on the second band.
        let table = &SV8_RES_1_TABLE;
        let e = table.lengths[0];
        let mut p = BitPacker::new();
        p.push(e.code, e.length); // band 0 decodes fine via initial ctx 0
        let bytes = p.finish();
        let mut reader = Sv7BitReader::new(&bytes);
        let err = decode_band_resolutions(&mut reader, 2, 0, |_| 2).unwrap_err();
        assert!(matches!(err, Error::UnsupportedBandType(i8::MIN)));
    }

    #[test]
    fn band_resolutions_propagates_eof_mid_walk() {
        // Two bands requested but only ~one codeword's worth of bits.
        let table = &SV8_RES_1_TABLE;
        let e = table.lengths[0];
        let mut p = BitPacker::new();
        p.push(e.code, e.length);
        // Do NOT call finish (which pads two zero bytes); instead pad
        // just enough to decode band 0 but starve band 1's peek.
        let mut bytes = p.bytes.clone();
        if p.nbits > 0 {
            bytes.push((p.acc << (8 - p.nbits)) as u8);
        }
        // No trailing zero bytes: after band 0, fewer than 16 bits.
        let mut reader = Sv7BitReader::new(&bytes);
        let res = decode_band_resolutions(&mut reader, 2, 0, |_| 0);
        assert!(matches!(res, Err(Error::UnexpectedEof)));
    }

    #[test]
    fn band_resolutions_raw_values_stay_in_staged_alphabet() {
        // Every res codeword (both context halves) decodes to a raw
        // value in 0..=16 — the wrapper keeps the GAP remap honest by
        // never pre-converting to a band_type.
        for table in [&SV8_RES_1_TABLE, &SV8_RES_2_TABLE] {
            for r in 0..table.lengths.len() {
                let sym = symbol_for_row(table, r);
                assert!(
                    (0..=16).contains(&sym),
                    "{} row {r} symbol {sym} outside the staged 0..=16 res alphabet",
                    table.name,
                );
            }
        }
    }
}
