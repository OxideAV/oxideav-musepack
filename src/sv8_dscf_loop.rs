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
//! ## §6.3 now GROUNDS the DSCF → SCF-index arithmetic
//!
//! The staged `spec/musepack-headers-and-coding.md` **§6.3** closes the
//! DSCF symbol → signed-delta centring GAP *and* the base-plus-delta SCF
//! index reconstruction the GAP-knob walk ([`decode_dscf_deltas`]) left
//! unscored. [`decode_sv8_band_scf`] is the grounded path:
//!
//! - **`SCF[0]`** — if the per-band "new-block" flag is set (key frame or
//!   first use) read a raw 7-bit absolute index minus 6; otherwise decode
//!   a delta via `sv8-canonical-dscf-2` (value 64 escape ⇒ `+ raw 6
//!   bits`) and fold `SCF[0] = ((SCF_prev2 − 25 + delta) & 127) − 6`.
//! - **`SCF[1]` / `SCF[2]`** — copied from the previous granule when the
//!   SCFI selector marks them shared, else decoded via
//!   `sv8-canonical-dscf-1` (value 31 escape ⇒ `64 + raw 6 bits`) and
//!   folded the same `((prev − 25 + delta) & 127) − 6` way.
//!
//! The DSCF context is no longer a caller knob: §6.3 fixes `SCF[0]` to
//! `dscf-2` and the later granules to `dscf-1`. The SCFI → coded/shared
//! granule schedule is [`scfi_coded_granules`] (the §5.3 SCFI-case
//! table). The legacy GAP-knob [`decode_dscf_deltas`] (raw wrapper +
//! caller context/count closures) is retained for callers that want the
//! pre-arithmetic raw values.
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

// ─── §6.3 grounded DSCF → SCF-index reconstruction ──────────

/// Number of per-band SCF granules (Layer-II three-granule layout,
/// §1). One channel's band carries three SCF indices `SCF[0..=2]`.
pub const SCF_GRANULES_PER_BAND: usize = 3;

/// The §6.3 DSCF folding recentre constant: each decoded SCF index is
/// `((prev − 25 + delta) & 127) − 6` — a 7-bit ring fold (`& 127`) with
/// the `−6` recentring and a `−25` bias applied to the running
/// reference before the delta is added.
const DSCF_FOLD_PREV_BIAS: i32 = 25;
/// The §6.3 DSCF post-fold recentre offset (`… ) − 6`).
const DSCF_FOLD_RECENTRE: i32 = 6;
/// The §6.3 7-bit DSCF index ring mask (`& 127`).
const DSCF_RING_MASK: i32 = 127;

/// The §6.3 `dscf-2` escape symbol (value 64): "value 64 is an escape
/// that adds a further raw 6 bits". Used for the `SCF[0]` delta path.
const DSCF2_ESCAPE_SYMBOL: i8 = 64;
/// The §6.3 `dscf-1` escape symbol (value 31): "value 31 is an escape
/// that switches to `64 + raw 6 bits`". Used for the `SCF[1]`/`SCF[2]`
/// delta path.
const DSCF1_ESCAPE_SYMBOL: i8 = 31;
/// Width of every §6.3 DSCF escape's extra raw field (6 bits).
const DSCF_ESCAPE_RAW_BITS: u8 = 6;
/// Width of the §6.3 `SCF[0]` new-block absolute index (`raw 7-bit
/// index minus 6`).
const DSCF_NEWBLOCK_ABS_BITS: u8 = 7;

/// Fold one §6.3 DSCF `delta` onto a running reference `prev`:
/// `SCF = ((prev − 25 + delta) & 127) − 6`. Shared by both the
/// `SCF[0]` and the `SCF[1]`/`SCF[2]` delta paths (§6.3 applies the
/// identical fold to all three).
#[inline]
const fn dscf_fold(prev: i32, delta: i32) -> i32 {
    ((prev - DSCF_FOLD_PREV_BIAS + delta) & DSCF_RING_MASK) - DSCF_FOLD_RECENTRE
}

/// Read the §6.3 `dscf-2` delta for `SCF[0]`: one `sv8-canonical-dscf-2`
/// codeword, plus — when the symbol is the escape value 64 — a further
/// 6 raw bits added on. Returns the signed delta to fold.
///
/// §6.3: "decode a delta via `sv8-canonical-dscf-2` — value 64 is an
/// escape that adds a further raw 6 bits".
fn read_dscf0_delta(reader: &mut Sv7BitReader<'_>) -> Result<i32> {
    let table = table_for_role(Sv8TableRole::Dscf, 1).ok_or(Error::UnsupportedBandType(i8::MIN))?;
    let symbol = table.decode(reader)?;
    let mut delta = symbol as i32;
    if symbol == DSCF2_ESCAPE_SYMBOL {
        delta += reader.read_bits(DSCF_ESCAPE_RAW_BITS)? as i32;
    }
    Ok(delta)
}

/// Read the §6.3 `dscf-1` delta for `SCF[1]` / `SCF[2]`: one
/// `sv8-canonical-dscf-1` codeword, plus — when the symbol is the
/// escape value 31 — a switch to `64 + raw 6 bits`. Returns the signed
/// delta to fold.
///
/// §6.3: "decoded via `sv8-canonical-dscf-1` (value 31 is an escape that
/// switches to `64 + raw 6 bits`)".
fn read_dscf_later_delta(reader: &mut Sv7BitReader<'_>) -> Result<i32> {
    let table = table_for_role(Sv8TableRole::Dscf, 0).ok_or(Error::UnsupportedBandType(i8::MIN))?;
    let symbol = table.decode(reader)?;
    if symbol == DSCF1_ESCAPE_SYMBOL {
        Ok(64 + reader.read_bits(DSCF_ESCAPE_RAW_BITS)? as i32)
    } else {
        Ok(symbol as i32)
    }
}

/// Whether each of the three SCF granules is independently *coded*
/// (`true`) or *copied* from the previous granule (`false`) for a
/// given SCFI selector value, per the §5.3 SCFI-case table (which §6.3
/// inherits: `SCF[0]` is always coded; `SCF[1]` / `SCF[2]` are coded
/// or shared by the SCFI value):
///
/// | SCFI | SCF\[0] | SCF\[1]      | SCF\[2]      |
/// |-----:|---------|--------------|--------------|
/// | 0    | coded   | coded        | coded        |
/// | 1    | coded   | coded        | = SCF\[1]    |
/// | 2    | coded   | = SCF\[0]    | coded        |
/// | 3    | coded   | = SCF\[0]    | = SCF\[1]    |
///
/// `SCF[0]` is always coded; the returned `[bool; 3]` slot 0 is always
/// `true`. Returns [`Error::InvalidScfCodingMethod`] for `scfi > 3`.
pub fn scfi_coded_granules(scfi: u8) -> Result<[bool; SCF_GRANULES_PER_BAND]> {
    let (c1, c2) = match scfi {
        0 => (true, true),
        1 => (true, false),
        2 => (false, true),
        3 => (false, false),
        other => return Err(Error::InvalidScfCodingMethod(other as i8)),
    };
    Ok([true, c1, c2])
}

/// Reconstruct one channel's three §6.3 SCF indices for one band,
/// **grounded** by `spec/musepack-headers-and-coding.md` §6.3.
///
/// Inputs:
///
/// - `reader` — the band-body bit reader.
/// - `scfi` — this channel's SCFI selector (`0..=3`, from the §6.3
///   [`crate::sv8_scf_header::decode_sv8_scfi`] split) deciding which of
///   `SCF[1]` / `SCF[2]` are coded vs copied ([`scfi_coded_granules`]).
/// - `new_block` — the per-band "new-block" flag. §6.3: "if the per-band
///   'new-block' flag is set (key frame or first use), read a raw 7-bit
///   absolute index minus 6; otherwise decode a delta via
///   `sv8-canonical-dscf-2`". §6.2 forces this set on every key frame.
/// - `prev_scf2` — the previous band's `SCF[2]` for this channel (the
///   `SCF[0]` delta reference); ignored when `new_block` is set.
///
/// §6.3 decode:
///
/// - **`SCF[0]`** — if `new_block`: `raw7 − 6`. Otherwise:
///   `SCF[0] = fold(prev_scf2, dscf2_delta)` where `dscf2_delta` is one
///   `dscf-2` codeword (escape value 64 ⇒ `+ raw6`) and
///   `fold(p, d) = ((p − 25 + d) & 127) − 6`.
/// - **`SCF[1]`** — coded (per `scfi`) ⇒ `fold(SCF[0], dscf1_delta)`;
///   shared ⇒ `= SCF[0]`.
/// - **`SCF[2]`** — coded ⇒ `fold(SCF[1], dscf1_delta)`; shared ⇒
///   `= SCF[1]`.
///
/// where each `dscf1_delta` is one `dscf-1` codeword (escape value 31 ⇒
/// `64 + raw6`).
///
/// Returns the three absolute SCF indices `[SCF[0], SCF[1], SCF[2]]`.
///
/// Errors:
///
/// - [`Error::UnexpectedEof`] if the reader starves on any VLC / raw
///   field.
/// - [`Error::InvalidScfCodingMethod`] if `scfi > 3`.
pub fn decode_sv8_band_scf(
    reader: &mut Sv7BitReader<'_>,
    scfi: u8,
    new_block: bool,
    prev_scf2: i32,
) -> Result<[i32; SCF_GRANULES_PER_BAND]> {
    let coded = scfi_coded_granules(scfi)?;

    // SCF[0]: new-block absolute or dscf-2 delta folded off prev_scf2.
    let scf0 = if new_block {
        reader.read_bits(DSCF_NEWBLOCK_ABS_BITS)? as i32 - DSCF_FOLD_RECENTRE
    } else {
        let delta = read_dscf0_delta(reader)?;
        dscf_fold(prev_scf2, delta)
    };

    // SCF[1]: coded (dscf-1 delta folded off SCF[0]) or copied from SCF[0].
    let scf1 = if coded[1] {
        let delta = read_dscf_later_delta(reader)?;
        dscf_fold(scf0, delta)
    } else {
        scf0
    };

    // SCF[2]: coded (dscf-1 delta folded off SCF[1]) or copied from SCF[1].
    let scf2 = if coded[2] {
        let delta = read_dscf_later_delta(reader)?;
        dscf_fold(scf1, delta)
    } else {
        scf1
    };

    Ok([scf0, scf1, scf2])
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

        /// Push the low `length` bits of `value` MSB-first (a
        /// right-justified raw field, the convention §6.3's escape and
        /// new-block raw reads use).
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

    // ─── §6.3 grounded DSCF → SCF-index reconstruction ──────

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

    /// Spec-replica fold so expectations are computed from §6.3, not the
    /// impl: `SCF = ((prev − 25 + delta) & 127) − 6`.
    fn fold_ref(prev: i32, delta: i32) -> i32 {
        ((prev - 25 + delta) & 127) - 6
    }

    #[test]
    fn scfi_coded_granules_matches_section_5_3_case_table() {
        // §5.3 SCFI case table (SCF[0] always coded):
        assert_eq!(scfi_coded_granules(0).unwrap(), [true, true, true]);
        assert_eq!(scfi_coded_granules(1).unwrap(), [true, true, false]);
        assert_eq!(scfi_coded_granules(2).unwrap(), [true, false, true]);
        assert_eq!(scfi_coded_granules(3).unwrap(), [true, false, false]);
        for bad in [4u8, 5, 255] {
            assert_eq!(
                scfi_coded_granules(bad),
                Err(Error::InvalidScfCodingMethod(bad as i8))
            );
        }
    }

    #[test]
    fn band_scf_new_block_reads_raw7_minus_6_for_scf0() {
        // new_block ⇒ SCF[0] = raw7 − 6. With scfi 2 (SCF[1] shared,
        // SCF[2] coded) the dscf-1 delta for SCF[2] follows.
        // Pick an absolute index, then a dscf-1 codeword for SCF[2].
        let abs: u32 = 70; // arbitrary 7-bit value
        let (code2, len2) = codeword_for_symbol(&SV8_DSCF_1_TABLE, 3).expect("dscf-1 has symbol 3");
        let mut p = BitPacker::new();
        p.push_raw(abs, 7);
        p.push(code2, len2); // SCF[2] delta (SCF[1] is shared = SCF[0])
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let scf = decode_sv8_band_scf(&mut r, 2, true, 999).unwrap();
        let scf0 = abs as i32 - 6;
        let scf2 = fold_ref(scf0, 3);
        assert_eq!(scf, [scf0, scf0, scf2]); // SCF[1] shared = SCF[0]
    }

    #[test]
    fn band_scf_non_new_block_folds_dscf2_delta_off_prev_scf2() {
        // !new_block ⇒ SCF[0] = fold(prev_scf2, dscf2_delta). scfi 3 ⇒
        // SCF[1] and SCF[2] both shared, so only the SCF[0] dscf-2 read.
        let (code0, len0) = codeword_for_symbol(&SV8_DSCF_2_TABLE, 5).expect("dscf-2 has symbol 5");
        let mut p = BitPacker::new();
        p.push(code0, len0);
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let prev = 40;
        let scf = decode_sv8_band_scf(&mut r, 3, false, prev).unwrap();
        let scf0 = fold_ref(prev, 5);
        assert_eq!(scf, [scf0, scf0, scf0]); // both later granules shared
    }

    #[test]
    fn band_scf_scfi0_codes_all_three_granules_with_dscf1_deltas() {
        // scfi 0 ⇒ SCF[1] and SCF[2] both coded. !new_block ⇒ SCF[0]
        // dscf-2 delta, then two dscf-1 deltas folding forward.
        let (c0, l0) = codeword_for_symbol(&SV8_DSCF_2_TABLE, 2).unwrap();
        let (c1, l1) = codeword_for_symbol(&SV8_DSCF_1_TABLE, 4).unwrap();
        let (c2, l2) = codeword_for_symbol(&SV8_DSCF_1_TABLE, 1).unwrap();
        let mut p = BitPacker::new();
        p.push(c0, l0);
        p.push(c1, l1);
        p.push(c2, l2);
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let prev = 30;
        let scf = decode_sv8_band_scf(&mut r, 0, false, prev).unwrap();
        let scf0 = fold_ref(prev, 2);
        let scf1 = fold_ref(scf0, 4);
        let scf2 = fold_ref(scf1, 1);
        assert_eq!(scf, [scf0, scf1, scf2]);
    }

    #[test]
    fn band_scf_dscf2_escape_64_adds_raw6() {
        // SCF[0] non-new-block via dscf-2 escape symbol 64: delta =
        // 64 + raw6. scfi 3 ⇒ no later reads.
        let (c0, l0) = codeword_for_symbol(&SV8_DSCF_2_TABLE, 64).expect("dscf-2 escape symbol 64");
        let raw6: u32 = 41;
        let mut p = BitPacker::new();
        p.push(c0, l0);
        p.push_raw(raw6, 6);
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let prev = 12;
        let scf = decode_sv8_band_scf(&mut r, 3, false, prev).unwrap();
        let scf0 = fold_ref(prev, 64 + raw6 as i32);
        assert_eq!(scf, [scf0, scf0, scf0]);
    }

    #[test]
    fn band_scf_dscf1_escape_31_switches_to_64_plus_raw6() {
        // SCF[1] coded via dscf-1 escape symbol 31: delta = 64 + raw6.
        // scfi 1 ⇒ SCF[1] coded, SCF[2] shared = SCF[1]. SCF[0] new-block.
        let (c1, l1) = codeword_for_symbol(&SV8_DSCF_1_TABLE, 31).expect("dscf-1 escape symbol 31");
        let abs: u32 = 50;
        let raw6: u32 = 7;
        let mut p = BitPacker::new();
        p.push_raw(abs, 7); // SCF[0] absolute
        p.push(c1, l1); // SCF[1] escape codeword
        p.push_raw(raw6, 6); // SCF[1] escape raw
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let scf = decode_sv8_band_scf(&mut r, 1, true, 0).unwrap();
        let scf0 = abs as i32 - 6;
        let scf1 = fold_ref(scf0, 64 + raw6 as i32);
        assert_eq!(scf, [scf0, scf1, scf1]); // SCF[2] shared = SCF[1]
    }

    #[test]
    fn band_scf_rejects_scfi_above_three() {
        let mut r = Sv7BitReader::new(&[0xFF; 8]);
        assert_eq!(
            decode_sv8_band_scf(&mut r, 4, true, 0),
            Err(Error::InvalidScfCodingMethod(4))
        );
    }

    #[test]
    fn band_scf_propagates_eof() {
        // new_block needs 7 bits for SCF[0]; an empty reader starves.
        let mut r = Sv7BitReader::new(&[]);
        assert_eq!(
            decode_sv8_band_scf(&mut r, 3, true, 0),
            Err(Error::UnexpectedEof)
        );
    }
}
