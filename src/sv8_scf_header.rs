//! SV8 frame-body SCFI-selector header walk (§3.5 SCF-coding selector).
//!
//! After the §3.4 band-resolution outer loop ([`crate::sv8_band_header`])
//! decides how many subbands are coded and each band's resolution, the
//! §3.5 scalefactor (SCF) layer opens — for each non-zero band — with a
//! **SCF-coding-method selector VLC** (the SV8 analogue of SV7's §2.4
//! SCFI selector, [`crate::scf::ScfCodingMethod`]). This module wires
//! that per-band selector read on top of the staged SV8 canonical-Huffman
//! SCFI context pair:
//!
//! - `sv8-canonical-scfi-{1,2}` (+ symbol maps `sv8-symbols-scfi-{1,2}`)
//!   — the **SCFI selector**, a first-order **context pair**. Each
//!   `.meta` `spec_role` is `"SV8 spec §3.5 SCFI canonical
//!   length-table"` (variant 1 annotated `"mono / single-SCF stream"`);
//!   the paired symbol maps span `0..=3` (variant 1) and `0..=15`
//!   (variant 2).
//!
//! ## What the §3.5 structural prose pins (and drives this module)
//!
//! 1. **An SCFI selector precedes the per-band SCF deltas.** §3.5
//!    ("SV8 keeps per-band scalefactor indices but delta-codes them
//!    with their own SV8 SCF VLC tables ... a base index plus VLC-coded
//!    deltas across the band's granules, reset at band boundaries — the
//!    *structure* is inherited from the SV7 SCF scheme §2.4") plus the
//!    `scfi` table's `spec_role` ("SCFI ... selector") ground that each
//!    coded band reads exactly one SCFI VLC before its DSCF deltas, in
//!    ascending band order. [`decode_scfi_selectors`] reads exactly one
//!    SCFI VLC per coded band.
//!
//! ## §6.3 now GROUNDS the context rule + the L/R SCFI split
//!
//! The staged `spec/musepack-headers-and-coding.md` **§6.3** closes the
//! `scfi-1` vs `scfi-2` context-selection GAP and pins the packed-value
//! split that the GAP-knob walk ([`decode_scfi_selectors`]) left to the
//! caller — both are now grounded by [`decode_sv8_scfi`]:
//!
//! - **Context.** §6.3: the SCFI context is chosen by "how many channels
//!   are non-zero (0 or 1 → `scfi-1`; both → `scfi-2`)". No longer a
//!   caller closure.
//! - **The packed L/R split.** §6.3: "one canonical decode yields a
//!   packed value split into the L SCFI (`value >> (2·cnt)`) and R SCFI
//!   (`value & 3`)" — so a both-non-zero stereo band's single `scfi-2`
//!   codeword carries *both* channels' SCFI selectors, and a
//!   single-channel band's `scfi-1` codeword is the lone selector.
//!   [`decode_sv8_scfi`] returns the recovered [`Sv8BandScfi`] `{left,
//!   right}` pair ready to feed the SV7-shape granule schedule
//!   ([`crate::scf::ScfCodingMethod`]).
//!
//! ## What the §3.5 prose does NOT pin (caller knobs / GAP)
//!
//! - **The SCFI value → granule-schedule semantics.** SV7's §2.4 SCFI
//!   value is `0..=3` and maps onto the Layer-II SCFSI four-way
//!   granule schedule ([`crate::scf::ScfCodingMethod::schedule`]). The
//!   SV8 `scfi-2` symbol map, however, spans `0..=15` — it does **not**
//!   line up with the four-value SV7 schedule cell-for-cell. §3.5 says
//!   the *structure* (base + VLC deltas across granules) is inherited
//!   but does **not** spell out the SV8 SCFI value → (count,
//!   granule-mapping) table. This module therefore returns the raw VLC
//!   value wrapped in [`RawScfiVlc`] so a caller cannot feed it into the
//!   SV7 [`crate::scf::ScfCodingMethod::from_raw`] schedule (which
//!   would reject every value `>3`) without an explicit, not-yet-pinned
//!   SV8 schedule step. This is the exact analogue of SV8's
//!   [`crate::sv8_band_header::RawResVlc`] band-resolution wrapper.
//! - **The `scfi-1` vs `scfi-2` context-selection rule.** §3.5 ships a
//!   `{ctx0, ctx1}` pair for the SCFI selector (variant 1 `.meta` is
//!   annotated "mono / single-SCF stream") but the structural prose
//!   does not state the predicate choosing which half a given band reads
//!   from. [`decode_scfi_selectors`] takes the pick as a caller-supplied
//!   closure `ctx_for_prev_scfi(previous_raw_scfi) -> ctx`, the same
//!   GAP-knob precedent [`crate::sv8_band_header::decode_band_resolutions`]
//!   uses for the band-resolution context pair; the first band uses a
//!   caller-supplied `initial_ctx`.
//! - **The DSCF delta walk and its symbol → signed-delta offset.** The
//!   `sv8-symbols-dscf-{1,2}` maps span `0..=63` (unsigned), unlike
//!   SV7's directly-signed DSCF symbols (`-7..=8`). The centring offset
//!   that turns an SV8 DSCF symbol into a signed delta is GAP, so this
//!   module reads only the SCFI selector and leaves the DSCF delta walk
//!   to a future round once that offset is pinned.
//!
//! ## Source-of-record
//!
//! - Structural prose: `docs/audio/musepack/musepack-sv7-sv8-spec.md`
//!   §3.5 (SV8 SCF coding) + §2.4 (the inherited SV7 SCF structure).
//! - Table roles: the `.meta` `spec_role` lines of
//!   `sv8-canonical-scfi-{1,2}` / `sv8-symbols-scfi-{1,2}` under
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
/// invocation of the `sv8-canonical-scfi-{1,2}` SCFI selector VLC.
///
/// The wrapper keeps the `scfi → granule-schedule` mapping honest: the
/// staged SV8 SCFI symbol maps span `0..=3` (variant 1) and `0..=15`
/// (variant 2), and the wider variant-2 range does **not** match the
/// four-value SV7 §2.4 SCFI schedule
/// ([`crate::scf::ScfCodingMethod`]) cell-for-cell. The SV8 value →
/// (count, granule-mapping) table is DOCS-GAP, so callers must apply an
/// explicit SV8 schedule step before deriving a granule layout; the
/// distinct type prevents accidental composition with the SV7 schedule.
///
/// This is the SV8 SCFI sibling of
/// [`crate::sv8_band_header::RawResVlc`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RawScfiVlc(i8);

impl RawScfiVlc {
    /// Wrap a raw `i8` VLC symbol. No validity check beyond the `i8`
    /// range — the staged `scfi` maps guarantee values in `0..=15`,
    /// but nothing in §3.5 elevates that to a hard invariant.
    pub const fn from_raw(value: i8) -> Self {
        Self(value)
    }

    /// Expose the underlying `i8` value: the raw input to the
    /// upstream-pending SV8 `scfi → granule-schedule` step and to the
    /// caller-supplied context-selection predicate.
    pub const fn as_i8(self) -> i8 {
        self.0
    }
}

/// Walk the §3.5 per-band SCFI-selector header: read `nbands`
/// `sv8-canonical-scfi-{1,2}` codewords in ascending band order,
/// returning the raw VLC value of each wrapped in [`RawScfiVlc`].
///
/// Context-pair selection (the GAP knob): the first band reads from the
/// `initial_ctx` half of the SCFI pair; each subsequent band reads from
/// `ctx_for_prev_scfi(previous_raw_scfi)`. A context value outside
/// `0..=1` (from either source) yields [`Error::UnsupportedBandType`]
/// carrying the [`CONTEXT_FAULT_SENTINEL`] payload (`i8::MIN`), the same
/// reserved out-of-range marker
/// [`crate::sv8_band_header::decode_band_resolutions`] uses for a
/// context-rule fault so callers can distinguish it from a genuine
/// band_type rejection.
///
/// `nbands` is the caller-supplied loop bound (typically the number of
/// non-zero coded bands from the §3.4 band-resolution walk); the
/// function reads exactly that many SCFI VLCs.
///
/// Errors:
///
/// - [`Error::UnexpectedEof`] mid-walk if the reader starves.
/// - [`Error::HuffmanNoMatch`] if an SCFI peek matches no row
///   (unreachable for the staged scfi tables).
/// - [`Error::UnsupportedBandType`] (payload [`CONTEXT_FAULT_SENTINEL`])
///   if the context-selection rule produces a `ctx` outside `0..=1`.
pub fn decode_scfi_selectors<F>(
    reader: &mut Sv7BitReader<'_>,
    nbands: u8,
    initial_ctx: u8,
    mut ctx_for_prev_scfi: F,
) -> Result<Vec<RawScfiVlc>>
where
    F: FnMut(RawScfiVlc) -> u8,
{
    let mut out = Vec::with_capacity(nbands as usize);
    let mut ctx = initial_ctx;
    for _ in 0..nbands {
        let table = table_for_role(Sv8TableRole::Scfi, ctx)
            .ok_or(Error::UnsupportedBandType(CONTEXT_FAULT_SENTINEL))?;
        let raw = RawScfiVlc::from_raw(table.decode(reader)?);
        out.push(raw);
        ctx = ctx_for_prev_scfi(raw);
    }
    Ok(out)
}

/// Sentinel band_type payload reported when the caller's
/// context-selection rule yields an out-of-range `ctx`. `i8::MIN`
/// (`-128`) is outside the §3.4 enumerated `band_type` domain
/// (`-1..=17`) and every staged SCFI alphabet (`0..=15`), so it cannot
/// collide with a genuine band_type rejection. Mirrors the sentinel
/// [`crate::sv8_band_header::decode_band_resolutions`] uses.
pub const CONTEXT_FAULT_SENTINEL: i8 = i8::MIN;

/// The two per-channel SCFI coding-method values a single §6.3 SCFI
/// decode produces for one band.
///
/// Each value is the SV7-shape `0..=3` SCFI selector (the Layer-II
/// SCFSI four-way granule schedule, [`crate::scf::ScfCodingMethod`]):
/// it says, for that channel's three SCF granules, how many distinct
/// scalefactor indices are coded and how the three granules share them.
/// `right` is meaningful only when the band has both channels non-zero;
/// for a single-non-zero-channel band only `left` is used (the §6.3
/// packed value degenerates to one field — see [`decode_sv8_scfi`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Sv8BandScfi {
    /// Left- (or sole-) channel SCFI selector (`0..=3`).
    pub left: u8,
    /// Right-channel SCFI selector (`0..=3`); meaningful only when
    /// both channels of the band are non-zero.
    pub right: u8,
}

/// Decode the §6.3 SV8 per-band SCFI selector(s), **grounded** by
/// `spec/musepack-headers-and-coding.md` §6.3 — no caller context knob.
///
/// §6.3: "for each used band, a context is chosen by how many channels
/// are non-zero (0 or 1 → `sv8-canonical-scfi-1`; both → `-2`); one
/// canonical decode yields a packed value split into the L SCFI
/// (`value >> (2·cnt)`) and R SCFI (`value & 3`)."
///
/// `nonzero_channels` is the count of channels with a non-zero band
/// resolution for this band (`0`, `1`, or `2`):
///
/// - **`0` or `1`** ⇒ context 0 (`sv8-canonical-scfi-1`, the "mono /
///   single-SCF" half). The decoded value is the single channel's SCFI
///   directly (`cnt = 0`, so `value >> 0 = value`); `right` is set equal
///   to `left` for a total surface but is not a coded field.
/// - **`2`** ⇒ context 1 (`sv8-canonical-scfi-2`). The decoded value
///   packs both: `left = value >> 2`, `right = value & 3` (`cnt = 1`,
///   so `2·cnt = 2`). Here `cnt` is the number of *additional* non-zero
///   channels beyond the first — `1` for a stereo both-non-zero band —
///   which is the only multiplier making the staged `scfi-2` `0..=15`
///   alphabet split into two `0..=3` SCFI fields.
///
/// The split masks each field to the `0..=3` SCFI range
/// ([`crate::scf::ScfCodingMethod`]'s domain); the staged `scfi-1` map
/// is `0..=3` and `scfi-2` is `0..=15`, so both fields land in range for
/// a well-formed stream.
///
/// `nonzero_channels == 0` still reads one `scfi-1` codeword (the §6.3
/// "0 or 1" context-0 branch covers it); a band with no non-zero channel
/// would not normally reach the SCFI layer, but the function stays total
/// over the `0..=2` domain. A `nonzero_channels` above 2 yields
/// [`Error::ChannelCountInvalid`].
///
/// Errors:
///
/// - [`Error::UnexpectedEof`] if the reader starves on the SCFI peek.
/// - [`Error::HuffmanNoMatch`] if the peek matches no row (unreachable
///   for the staged scfi tables).
/// - [`Error::ChannelCountInvalid`] if `nonzero_channels > 2`.
pub fn decode_sv8_scfi(reader: &mut Sv7BitReader<'_>, nonzero_channels: u8) -> Result<Sv8BandScfi> {
    if nonzero_channels > 2 {
        return Err(Error::ChannelCountInvalid(nonzero_channels));
    }
    // §6.3 context: both channels non-zero ⇒ scfi-2 (ctx 1), else scfi-1.
    let ctx: u8 = if nonzero_channels == 2 { 1 } else { 0 };
    let table = table_for_role(Sv8TableRole::Scfi, ctx)
        .ok_or(Error::UnsupportedBandType(CONTEXT_FAULT_SENTINEL))?;
    let value = table.decode(reader)? as u16;
    // `cnt` = additional non-zero channels beyond the first (1 for a
    // stereo both-non-zero band, else 0); the §6.3 shift is `2·cnt`.
    let cnt: u32 = if nonzero_channels == 2 { 1 } else { 0 };
    let left = ((value >> (2 * cnt)) & 0x3) as u8;
    let right = (value & 0x3) as u8;
    Ok(Sv8BandScfi { left, right })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sv8_huffman::{Sv8CanonicalTable, SV8_SCFI_1_TABLE, SV8_SCFI_2_TABLE};

    /// MSB-first left-justified bit packer (mirrors the one used in the
    /// SV8 band-resolution tests): push a `length`-bit codeword from the
    /// top of `pattern`; `finish` flushes + appends two zero bytes so
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

    // ─── RawScfiVlc newtype ─────────────────────────────────

    #[test]
    fn raw_scfi_vlc_roundtrips() {
        for v in [-1_i8, 0, 1, 3, 15, 16, i8::MIN, i8::MAX] {
            let w = RawScfiVlc::from_raw(v);
            assert_eq!(w.as_i8(), v);
        }
    }

    #[test]
    fn raw_scfi_vlc_is_copy_and_eq() {
        let a = RawScfiVlc::from_raw(7);
        let b = a;
        assert_eq!(a, b);
        assert_ne!(a, RawScfiVlc::from_raw(8));
    }

    // ─── Single-band SCFI decode against each context half ───

    #[test]
    fn decode_one_band_uses_initial_ctx_zero_variant1() {
        // ctx 0 → scfi-1 (variant 1). Decode its first row.
        let want = symbol_for_row(&SV8_SCFI_1_TABLE, 0);
        let e = SV8_SCFI_1_TABLE.lengths[0];
        let mut p = BitPacker::new();
        p.push(e.code, e.length);
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let got = decode_scfi_selectors(&mut r, 1, 0, |_| 0).unwrap();
        assert_eq!(got.len(), 1);
        assert_eq!(got[0], RawScfiVlc::from_raw(want));
    }

    #[test]
    fn decode_one_band_uses_initial_ctx_one_variant2() {
        // ctx 1 → scfi-2 (variant 2). Decode its first row.
        let want = symbol_for_row(&SV8_SCFI_2_TABLE, 0);
        let e = SV8_SCFI_2_TABLE.lengths[0];
        let mut p = BitPacker::new();
        p.push(e.code, e.length);
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let got = decode_scfi_selectors(&mut r, 1, 1, |_| 1).unwrap();
        assert_eq!(got[0], RawScfiVlc::from_raw(want));
    }

    // ─── Multi-band walk + context switching ────────────────

    #[test]
    fn decode_two_bands_switches_context_from_first_to_second() {
        // Band 0 reads scfi-1 (initial ctx 0); the closure flips to
        // ctx 1 so band 1 reads scfi-2. Pack: scfi-1 row 1 followed by
        // scfi-2 row 1.
        let e0 = SV8_SCFI_1_TABLE.lengths[1];
        let e1 = SV8_SCFI_2_TABLE.lengths[1];
        let want0 = symbol_for_row(&SV8_SCFI_1_TABLE, 1);
        let want1 = symbol_for_row(&SV8_SCFI_2_TABLE, 1);
        let mut p = BitPacker::new();
        p.push(e0.code, e0.length);
        p.push(e1.code, e1.length);
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        // Track how many times the closure is invoked + with what.
        let mut seen_prev: Vec<RawScfiVlc> = Vec::new();
        let got = decode_scfi_selectors(&mut r, 2, 0, |prev| {
            seen_prev.push(prev);
            1
        })
        .unwrap();
        assert_eq!(
            got,
            vec![RawScfiVlc::from_raw(want0), RawScfiVlc::from_raw(want1)]
        );
        // The closure is called once per band (after each decode),
        // first with band 0's value.
        assert_eq!(
            seen_prev,
            vec![RawScfiVlc::from_raw(want0), RawScfiVlc::from_raw(want1)]
        );
    }

    #[test]
    fn zero_bands_reads_nothing_and_returns_empty() {
        let bytes = [0u8, 0, 0, 0];
        let mut r = Sv7BitReader::new(&bytes);
        let got = decode_scfi_selectors(&mut r, 0, 0, |_| {
            panic!("closure must not run for zero bands")
        })
        .unwrap();
        assert!(got.is_empty());
    }

    // ─── Context-fault + EOF paths ──────────────────────────

    #[test]
    fn out_of_range_initial_ctx_is_context_fault() {
        let bytes = [0xFFu8, 0xFF, 0, 0];
        let mut r = Sv7BitReader::new(&bytes);
        // initial_ctx 2 has no scfi table half.
        assert_eq!(
            decode_scfi_selectors(&mut r, 1, 2, |_| 0),
            Err(Error::UnsupportedBandType(CONTEXT_FAULT_SENTINEL))
        );
    }

    #[test]
    fn out_of_range_ctx_from_closure_is_context_fault_on_second_band() {
        // Band 0 decodes fine (ctx 0); the closure returns an invalid
        // ctx 5 so band 1's table lookup faults.
        let e0 = SV8_SCFI_1_TABLE.lengths[0];
        let mut p = BitPacker::new();
        p.push(e0.code, e0.length);
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        assert_eq!(
            decode_scfi_selectors(&mut r, 2, 0, |_| 5),
            Err(Error::UnsupportedBandType(CONTEXT_FAULT_SENTINEL))
        );
    }

    #[test]
    fn propagates_unexpected_eof_mid_walk() {
        // Empty buffer: the first peek16 starves.
        let bytes: [u8; 0] = [];
        let mut r = Sv7BitReader::new(&bytes);
        assert_eq!(
            decode_scfi_selectors(&mut r, 1, 0, |_| 0),
            Err(Error::UnexpectedEof)
        );
    }

    #[test]
    fn context_fault_sentinel_is_outside_band_type_and_scfi_domains() {
        // Defensive pin: the sentinel must not collide with any genuine
        // band_type (-1..=17) nor any staged scfi symbol (0..=15).
        assert_eq!(CONTEXT_FAULT_SENTINEL, i8::MIN);
        // No genuine band_type (-1..=17) nor staged scfi symbol (0..=15)
        // can equal the sentinel.
        for v in -1i8..=17 {
            assert_ne!(v, CONTEXT_FAULT_SENTINEL);
        }
    }

    // ─── §6.3 grounded SCFI decode (decode_sv8_scfi) ─────────

    /// Find a `(pattern, length)` codeword of `table` whose decoded
    /// symbol equals `target`, walking every codeword of every data row.
    fn codeword_for_symbol(table: &Sv8CanonicalTable, target: i8) -> Option<(u16, u8)> {
        let mut upper: u32 = 0x1_0000;
        for e in table.lengths.iter() {
            if e.length == 0 {
                continue;
            }
            let step = 1u32 << (16 - e.length as u32);
            let mut pat = e.code as u32;
            while pat < upper {
                let mut p = BitPacker::new();
                p.push(pat as u16, e.length);
                let bytes = p.finish();
                let mut r = Sv7BitReader::new(&bytes);
                if table.decode(&mut r).unwrap() == target {
                    return Some((pat as u16, e.length));
                }
                pat += step;
            }
            upper = e.code as u32;
        }
        None
    }

    #[test]
    fn scfi_single_channel_reads_scfi1_value_directly() {
        // §6.3: nonzero_channels 0 or 1 ⇒ ctx 0 (scfi-1); the decoded
        // value is the sole channel's SCFI (cnt = 0, value >> 0).
        for &target in &[0i8, 1, 2, 3] {
            let Some((code, len)) = codeword_for_symbol(&SV8_SCFI_1_TABLE, target) else {
                continue; // scfi-1 spans 0..=3; all four present
            };
            for nch in [0u8, 1] {
                let mut p = BitPacker::new();
                p.push(code, len);
                let bytes = p.finish();
                let mut r = Sv7BitReader::new(&bytes);
                let scfi = decode_sv8_scfi(&mut r, nch).unwrap();
                assert_eq!(scfi.left, target as u8, "nch {nch} target {target}");
                // right mirrors left for the single-channel surface.
                assert_eq!(scfi.right, target as u8);
            }
        }
    }

    #[test]
    fn scfi_both_channels_splits_packed_value_into_left_and_right() {
        // §6.3: nonzero_channels == 2 ⇒ ctx 1 (scfi-2); the value packs
        // L = value >> 2, R = value & 3. Exercise several packed values
        // spanning the 0..=15 scfi-2 alphabet.
        for value in 0i8..=15 {
            let Some((code, len)) = codeword_for_symbol(&SV8_SCFI_2_TABLE, value) else {
                continue;
            };
            let mut p = BitPacker::new();
            p.push(code, len);
            let bytes = p.finish();
            let mut r = Sv7BitReader::new(&bytes);
            let scfi = decode_sv8_scfi(&mut r, 2).unwrap();
            assert_eq!(scfi.left, (value as u8) >> 2, "value {value} left");
            assert_eq!(scfi.right, (value as u8) & 3, "value {value} right");
            // Both fields are valid 0..=3 SCFI selectors.
            assert!(scfi.left <= 3 && scfi.right <= 3);
        }
    }

    #[test]
    fn scfi_both_channels_recovers_each_of_the_four_left_right_combos() {
        // The four canonical (L, R) combos a stereo band can carry:
        // packed value 4·L + R must split back exactly.
        for l in 0u8..=3 {
            for rgt in 0u8..=3 {
                let packed = (l * 4 + rgt) as i8; // 0..=15
                let Some((code, len)) = codeword_for_symbol(&SV8_SCFI_2_TABLE, packed) else {
                    continue;
                };
                let mut p = BitPacker::new();
                p.push(code, len);
                let bytes = p.finish();
                let mut r = Sv7BitReader::new(&bytes);
                let scfi = decode_sv8_scfi(&mut r, 2).unwrap();
                assert_eq!((scfi.left, scfi.right), (l, rgt), "packed {packed}");
            }
        }
    }

    #[test]
    fn scfi_rejects_channel_count_above_two() {
        for nch in [3u8, 4, u8::MAX] {
            let mut r = Sv7BitReader::new(&[0xFF; 4]);
            assert_eq!(
                decode_sv8_scfi(&mut r, nch),
                Err(Error::ChannelCountInvalid(nch))
            );
        }
    }

    #[test]
    fn scfi_grounded_propagates_eof() {
        let mut r = Sv7BitReader::new(&[]);
        assert_eq!(decode_sv8_scfi(&mut r, 2), Err(Error::UnexpectedEof));
    }
}
