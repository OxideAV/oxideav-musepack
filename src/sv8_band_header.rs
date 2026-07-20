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
//! ## §6.2 now GROUNDS the context rule + the band_type remap
//!
//! The staged `spec/musepack-headers-and-coding.md` **§6.2** closes the
//! two GAPs the original GAP-knob walk ([`decode_band_resolutions`])
//! carried — both are now pinned by [`decode_band_resolutions_grounded`]:
//!
//! - **The `res-1` vs `res-2` context-selection rule.** §6.2: the
//!   Res-table context is selected by "whether the band-above `Res`
//!   exceeds 2" (`> 2` ⇒ `res-2` / ctx 1, else `res-1` / ctx 0). The
//!   **top** used band has no band above and reads from ctx 0. This is
//!   no longer a caller closure — see [`res_ctx_for_above`].
//! - **The raw-`res`-symbol → §3.4 `band_type` remap.** §6.2: bands are
//!   decoded **top-down**; the top band's raw value is the band_type
//!   after "values > 15 wrap by −17 (signed range)", and each lower
//!   band folds `Res[n] = canon(Res, ctx) + Res[n+1]`, re-wrapped the
//!   same way. So the staged `0..=16` raw alphabet maps onto the signed
//!   `-1..=15` band_type ring directly via [`wrap_res`] + the delta
//!   fold — no separate offset/escape table is needed. The grounded
//!   walk therefore returns signed `i8` band_types ready for
//!   [`crate::sv8_band_decode::sv8_band_type_case`].
//!
//! ## What §3.4 / §6.2 still leave to the caller (GAP)
//!
//! - The legacy GAP-knob [`decode_band_resolutions`] (closure + raw
//!   [`RawResVlc`] output) is retained for callers that want to drive
//!   the context rule themselves or inspect the pre-wrap raw alphabet,
//!   but new band-walk wiring should prefer the grounded function.
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

/// Decode one §6.5 bounded **"log" code**: a phased-/truncated-binary
/// codeword naming a value in `0..max`.
///
/// §6.5: the code "reads `floor(log2(max−1))` bits, and one extra bit
/// only when the value falls in the code space's 'lost' tail." This is
/// the same phased-binary prefix step the §6.5 enumerative coder uses
/// (cf. `crate::sv8_sample_decode`'s `enum_decode_subset`): for a code
/// space of `max` distinct values, `bitlen = ceil(log2(max))` is the
/// long-codeword width and `lost = 2^bitlen − max` short codewords use
/// `bitlen − 1` bits. (Identity: for `max ≥ 2`,
/// `bitlen − 1 == floor(log2(max−1))`, matching the §6.5 phrasing.)
///
/// Decode: read `bitlen − 1` bits as `code`; if `code ≥ lost`, read one
/// more bit and rebase `code = (code << 1) − lost + bit`. The `lost`
/// short codewords (`code < lost`) carry the low-ranked values.
///
/// `max ≤ 1` ⇒ the only value is 0 and no bits are read.
///
/// Used by §6.2 for the key-frame `Max_used_Band` (over
/// `0..max_band+1`) and by §6.2's M/S `cnt` selection.
///
/// Errors: [`Error::UnexpectedEof`] if the reader starves mid-code.
pub fn decode_log_code(reader: &mut Sv7BitReader<'_>, max: u32) -> Result<u32> {
    if max <= 1 {
        return Ok(0);
    }
    // bitlen = ceil(log2(max))
    let mut bitlen: u8 = 0;
    while (1u32 << bitlen) < max {
        bitlen += 1;
    }
    let lost = (1u32 << bitlen) - max;
    // Short codewords are `bitlen - 1` bits.
    let mut code = reader.read_bits(bitlen - 1)? as u32;
    if code >= lost {
        code = (code << 1) - lost + reader.read_bits(1)? as u32;
    }
    Ok(code)
}

/// Decode the §6.2 **key-frame** `Max_used_Band`: a §6.5 bounded log
/// code over the **count** range `0..=max_band+1`, where `max_band` is
/// the SH-packet's highest-coded-subband field (`SH` field 6, already
/// `+1`-debiased).
///
/// §6.2 phrases the code space as "the range `0..max_band+1`"; the
/// decoded value is the **count of coded bands**, whose inclusive
/// maximum is `max_band + 1` (all of bands `0..=max_band` coded), so
/// the code space holds `max_band + 2` values. Fixture-pinned (r419):
/// with the smaller `max_band + 1` space every keyframe of the SV8
/// corpus decodes a count one short of the SV7 ground truth; with
/// `max_band + 2` all 92 corpus frames align bit-exactly.
///
/// The result is range-checked against `max_band + 1` (the count cannot
/// exceed the SH-declared band range); a malformed `lost`-tail read is
/// rejected via [`Error::MaxBandOutOfRange`].
pub fn decode_keyframe_max_used_band(reader: &mut Sv7BitReader<'_>, max_band: u8) -> Result<u8> {
    let value = decode_log_code(reader, max_band as u32 + 2)?;
    if value > max_band as u32 + 1 {
        return Err(Error::MaxBandOutOfRange(value.min(u8::MAX as u32) as u8));
    }
    Ok(value as u8)
}

/// Decode the §6.2 **non-key-frame** `Max_used_Band`:
/// `Max_used_Band = last_max_band + canon(Bands)`, where `canon(Bands)`
/// is a signed delta from the `sv8-canonical-bands` table and "results
/// > 32 wrap by subtracting 33".
///
/// §6.2: "Non-key-frame: `Max_used_Band = last_max_band +
/// canon(Bands)`, where `canon(Bands)` decodes a signed delta via the
/// `sv8-canonical-bands` table; results > 32 wrap by subtracting 33."
///
/// `last_max_band` is the previous packet's `Max_used_Band`. The fold
/// keeps the count in the `0..=32` ring: a sum exceeding 32 wraps by
/// −33 (the inclusive 33-value ring `0..=32`).
///
/// Errors:
///
/// - [`Error::UnexpectedEof`] if fewer than 16 bits remain for the
///   `bands` canonical peek.
/// - [`Error::HuffmanNoMatch`] if the peek matches no `bands` row.
/// - [`Error::MaxBandOutOfRange`] if the wrapped result still exceeds
///   [`SV8_MAX_USED_SUBBANDS`] (unreachable for a well-formed delta;
///   defensive).
pub fn decode_nonkey_max_used_band(reader: &mut Sv7BitReader<'_>, last_max_band: u8) -> Result<u8> {
    let delta = SV8_BANDS_TABLE.decode(reader)? as i32;
    // last_max_band + canon(Bands); wrap a result above 32 by −33.
    let mut value = last_max_band as i32 + delta;
    if value > SV8_MAX_USED_SUBBANDS as i32 {
        value -= SV8_MAX_USED_SUBBANDS as i32 + 1;
    }
    if !(0..=SV8_MAX_USED_SUBBANDS as i32).contains(&value) {
        return Err(Error::MaxBandOutOfRange(value.unsigned_abs() as u8));
    }
    Ok(value as u8)
}

/// Decode the §6.2 SV8 **mid/side band-selection** bitmap: which of the
/// `tot` non-zero-channel bands carry a per-band M/S flag.
///
/// §6.2: "rather than one bit per band, SV8 counts the bands with a
/// non-zero channel (`tot`), reads a 'log' code `cnt` = how many of
/// them are mid/side, then — if `0 < cnt < tot` — reads an enumerative
/// (combinatorial) code selecting *which* `cnt` of the `tot` bands are
/// flagged. The bitmap is applied to the non-zero bands from the top
/// down."
///
/// `tot` is the count of bands with at least one non-zero channel
/// (computed by the caller from the §6.2 band-resolution walk). The
/// returned `Vec<bool>` has length `tot`; index `0` is the **lowest**
/// non-zero band and index `tot − 1` the topmost — fixture-pinned
/// (r419): applying the mask MSB-to-lowest-band reproduces the SV7
/// ground-truth per-band M/S flags across the transcoded corpus, while
/// the opposite orientation diverges on the first frame. (The §6.2
/// "applied … from the top down" phrasing describes the decode walk,
/// not the returned order.) `true` ⇒ that band is mid/side.
///
/// Decode:
///
/// 1. `cnt` = [`decode_log_code`] over `0..tot+1` (how many bands are
///    M/S). `cnt == 0` ⇒ no flags set; `cnt == tot` ⇒ all set; both
///    read no enumerative bits.
/// 2. Otherwise a §6.5 enumerative codeword
///    ([`crate::sv8_sample_decode::enum_decode_subset`]) selects
///    `min(cnt, tot − cnt)` positions of `tot`; when `cnt > tot / 2`
///    the coder named the *complement* (the smaller subset is always
///    coded), so the mask is bit-inverted within the `tot`-bit field.
/// 3. The mask is applied MSB-first onto ascending bands: bit
///    `tot − 1` is the lowest non-zero band (`out[0]`), down to bit
///    `0` for the topmost (`out[tot − 1]`) — the same
///    MSB-to-first-element orientation as the §6.4.1 sparse bitmap.
///
/// `tot == 0` returns an empty vector (reads nothing).
///
/// Errors: [`Error::UnexpectedEof`] if the reader starves;
/// [`Error::MaxBandOutOfRange`] if the decoded `cnt` exceeds `tot`
/// (a malformed log-code tail; defensive — the `0..tot+1` bound makes
/// it unreachable for a well-formed stream).
pub fn decode_sv8_ms_flags(reader: &mut Sv7BitReader<'_>, tot: u8) -> Result<Vec<bool>> {
    use crate::sv8_sample_decode::enum_decode_subset;

    let n = tot as u32;
    if n == 0 {
        return Ok(Vec::new());
    }

    let cnt = decode_log_code(reader, n + 1)?;
    if cnt > n {
        return Err(Error::MaxBandOutOfRange(cnt.min(u8::MAX as u32) as u8));
    }

    let mask: u32 = if cnt == 0 {
        0
    } else if cnt == n {
        if n >= 32 {
            u32::MAX
        } else {
            (1u32 << n) - 1
        }
    } else {
        let k = cnt.min(n - cnt);
        let coded = enum_decode_subset(reader, k, n)?;
        let full = if n >= 32 { u32::MAX } else { (1u32 << n) - 1 };
        if cnt > n / 2 {
            (!coded) & full
        } else {
            coded
        }
    };

    // MSB-first onto ascending bands: bit (tot-1) is the lowest
    // non-zero band, out[0] (fixture-pinned orientation, r419).
    let mut out = Vec::with_capacity(tot as usize);
    for p in (0..n).rev() {
        out.push(mask & (1 << p) != 0);
    }
    Ok(out)
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

/// The §6.2 signed-`band_type` wrap: the staged `res` symbol maps emit
/// a raw value in `0..=16`, but the §3.4 sample `switch` ladder ranges
/// over a *signed* `band_type` in `-1..=15` here. Spec §6.2: "values
/// above 15 wrap by −17 (signed range)". So a raw `16` becomes `-1`
/// (the CNS case), and `0..=15` pass through unchanged.
/// The same wrap is reapplied after the delta fold (`canon + above`),
/// keeping every intermediate and final `band_type` inside the signed
/// ring.
#[inline]
const fn wrap_res(v: i32) -> i8 {
    if v > 15 {
        (v - 17) as i8
    } else {
        v as i8
    }
}

/// The §6.2 per-band Res-table **context** predicate: the context-pair
/// half (`sv8-canonical-res-1` = ctx 0 vs `-2` = ctx 1) is selected by
/// "whether the band-above `Res` exceeds 2". So a band whose neighbour
/// above decoded to a `band_type > 2` reads its own res VLC from
/// context 1; otherwise from context 0. The top used band has no band
/// above and reads from context 0 (`res-1`) directly.
#[inline]
const fn res_ctx_for_above(above: i8) -> u8 {
    if above > 2 {
        1
    } else {
        0
    }
}

/// Walk the §6.2 SV8 per-band band-resolution header **fully grounded**
/// — no caller knobs. This is the §6.2 replacement for the GAP-knob
/// [`decode_band_resolutions`]: §6.2 now pins both the context-selection
/// predicate and the top-down delta fold that that function carried as
/// caller-supplied closures.
///
/// §6.2 decode order and arithmetic:
///
/// - **Top band first, downward.** Bands are decoded from the highest
///   coded index down to band 0. The **top** used band reads its res
///   VLC from **context 0** (`sv8-canonical-res-1`); its raw value is
///   wrapped to the signed `band_type` ring by [`wrap_res`] (raw `16`
///   ⇒ `-1`).
/// - **Lower bands delta off the band above.** For each band below the
///   top, the res-table context is [`res_ctx_for_above`] of the
///   *already-decoded band above* (`above > 2` ⇒ ctx 1, else ctx 0);
///   the decoded raw value is added to the band-above `band_type`
///   (`canon(Res, ctx) + Res[n+1]`) and the sum re-wrapped by
///   [`wrap_res`].
///
/// The returned `Vec` is in **ascending band order** (`out[0]` = band
/// 0, the lowest), holding the signed `band_type` ready to feed
/// [`crate::sv8_band_decode::sv8_band_type_case`] directly — this is the
/// §6.2 closure of the `RawResVlc → band_type` remap GAP that
/// [`decode_band_resolutions`] left open.
///
/// `nbands` is the §6.2 used-band count (typically the
/// [`decode_used_subbands`] result). `nbands == 0` reads nothing and
/// returns an empty vector.
///
/// Errors:
///
/// - [`Error::UnexpectedEof`] mid-walk if the reader starves.
/// - [`Error::HuffmanNoMatch`] if a res peek matches no row
///   (unreachable for the staged res tables).
/// - [`Error::UnsupportedBandType`] if [`table_for_role`] cannot supply
///   the selected context's res table (unreachable for `ctx ∈ {0,1}`,
///   kept as a defensive bound).
pub fn decode_band_resolutions_grounded(
    reader: &mut Sv7BitReader<'_>,
    nbands: u8,
) -> Result<Vec<i8>> {
    let n = nbands as usize;
    if n == 0 {
        return Ok(Vec::new());
    }

    // Decoded top-down into a descending buffer, then reversed to
    // ascending band order for the caller. `tmp[0]` = top band.
    let mut tmp: Vec<i8> = Vec::with_capacity(n);

    // Top band: context 0 (`res-1`), no band above to delta against.
    let top_table =
        table_for_role(Sv8TableRole::Res, 0).ok_or(Error::UnsupportedBandType(i8::MIN))?;
    let top = wrap_res(top_table.decode(reader)? as i32);
    tmp.push(top);

    // Remaining bands, top-down: context from the band above, delta
    // folded onto the band above, re-wrapped.
    let mut above = top;
    for _ in 1..n {
        let ctx = res_ctx_for_above(above);
        let table =
            table_for_role(Sv8TableRole::Res, ctx).ok_or(Error::UnsupportedBandType(i8::MIN))?;
        let raw = table.decode(reader)? as i32;
        let res = wrap_res(raw + above as i32);
        tmp.push(res);
        above = res;
    }

    tmp.reverse();
    Ok(tmp)
}

/// Walk the §6.2 band-resolution header for a **two-channel** frame
/// body, fixture-pinned (r419): bands are decoded **top-down** with the
/// two channels **interleaved per band** (left `Res` then right `Res`
/// at each band before moving down), and each channel folds its own
/// delta chain — the channel's res-table context comes from *that
/// channel's* band-above `Res` ([`res_ctx_for_above`]), its delta adds
/// onto *that channel's* band-above value, and every intermediate is
/// re-wrapped by the §6.2 signed ring ("values > 15 wrap by −17").
/// The top band reads both channels from context 0.
///
/// This is the stereo composition of
/// [`decode_band_resolutions_grounded`], pinned by the r419 SV8 corpus:
/// on the losslessly transcoded SV7 fixtures this walk reproduces the
/// SV7 §5.1 ground-truth `Res` pairs for every band of every frame,
/// while the channel-major alternative (all left bands then all right)
/// desynchronises on the first frame.
///
/// The returned `Vec` is in **ascending band order** (`out[0]` = band
/// 0); each element is `[left, right]` signed band_types ready for
/// [`crate::sv8_band_decode::sv8_band_type_case`]. `nbands == 0` reads
/// nothing.
///
/// # Errors
///
/// - [`Error::UnexpectedEof`] mid-walk if the reader starves.
/// - [`Error::HuffmanNoMatch`] if a res peek matches no row
///   (unreachable for the staged res tables).
/// - [`Error::UnsupportedBandType`] if [`table_for_role`] cannot supply
///   a context's res table (unreachable for `ctx ∈ {0,1}`; defensive).
pub fn decode_band_resolutions_stereo_grounded(
    reader: &mut Sv7BitReader<'_>,
    nbands: u8,
) -> Result<Vec<[i8; 2]>> {
    let n = nbands as usize;
    if n == 0 {
        return Ok(Vec::new());
    }

    // Decoded top-down; tmp[0] = top band. Per-channel `above` state.
    let mut tmp: Vec<[i8; 2]> = Vec::with_capacity(n);
    let mut above: [Option<i8>; 2] = [None, None];
    for _ in 0..n {
        let mut pair = [0_i8; 2];
        for (ch, slot) in pair.iter_mut().enumerate() {
            let ctx = match above[ch] {
                None => 0,
                Some(a) => res_ctx_for_above(a),
            };
            let table = table_for_role(Sv8TableRole::Res, ctx)
                .ok_or(Error::UnsupportedBandType(i8::MIN))?;
            let raw = table.decode(reader)? as i32;
            let res = match above[ch] {
                None => wrap_res(raw),
                Some(a) => wrap_res(raw + a as i32),
            };
            *slot = res;
            above[ch] = Some(res);
        }
        tmp.push(pair);
    }

    tmp.reverse();
    Ok(tmp)
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

    // ─── §6.2 grounded helpers ─────────────────────────────

    #[test]
    fn wrap_res_pins_the_signed_band_type_ring() {
        // §6.2: "values > 15 wrap by −17". 0..=15 pass through; 16 ⇒ −1
        // (the CNS case); the wrap is idempotent on already-signed input.
        for v in 0..=15 {
            assert_eq!(wrap_res(v), v as i8, "raw {v} must pass through");
        }
        assert_eq!(wrap_res(16), -1, "raw 16 wraps to the CNS band_type −1");
        // Post-delta sums above 15 also wrap (e.g. 15 + 15 = 30 ⇒ 13).
        assert_eq!(wrap_res(30), 13);
        assert_eq!(wrap_res(31), 14);
        // A sum that stays in range is untouched.
        assert_eq!(wrap_res(7), 7);
        assert_eq!(wrap_res(0), 0);
    }

    #[test]
    fn res_ctx_for_above_uses_the_band_above_exceeds_two_predicate() {
        // §6.2: context-1 (`res-2`) iff the band-above Res exceeds 2.
        for above in -1..=2 {
            assert_eq!(res_ctx_for_above(above), 0, "above {above} ⇒ ctx 0");
        }
        for above in 3..=15 {
            assert_eq!(res_ctx_for_above(above), 1, "above {above} ⇒ ctx 1");
        }
    }

    // ─── decode_band_resolutions_grounded (§6.2) ───────────

    /// Hand-replicate the §6.2 top-down walk independent of the impl:
    /// top band from ctx 0 (wrap), each lower band's ctx from the band
    /// above (`>2 ⇒ 1`), delta = raw + above, re-wrap. Returns
    /// ascending band order. `row_for_ctx` chooses which length-table
    /// row each successive decode reads (so callers drive the stream).
    fn replicate_grounded(rows: &[usize]) -> (Vec<u8>, Vec<i8>) {
        // Build the stream + the expected ascending-order band_type vec.
        let mut p = BitPacker::new();
        let mut top_down: Vec<i8> = Vec::new();
        let mut above: Option<i8> = None;
        for &row in rows {
            let ctx = match above {
                None => 0,
                Some(a) => res_ctx_for_above_ref(a),
            };
            let table = if ctx == 0 {
                &SV8_RES_1_TABLE
            } else {
                &SV8_RES_2_TABLE
            };
            let e = table.lengths[row];
            p.push(e.code, e.length);
            let raw = symbol_for_row(table, row) as i32;
            let res = match above {
                None => wrap_res_ref(raw),
                Some(a) => wrap_res_ref(raw + a as i32),
            };
            top_down.push(res);
            above = Some(res);
        }
        let mut ascending = top_down.clone();
        ascending.reverse();
        (p.finish(), ascending)
    }

    // Test-local mirrors of the (private const) impl helpers, so the
    // expected vector is computed from the spec rule, not the impl.
    fn wrap_res_ref(v: i32) -> i8 {
        if v > 15 {
            (v - 17) as i8
        } else {
            v as i8
        }
    }
    fn res_ctx_for_above_ref(above: i8) -> u8 {
        if above > 2 {
            1
        } else {
            0
        }
    }

    #[test]
    fn grounded_zero_bands_reads_nothing() {
        let mut reader = Sv7BitReader::new(&[0xFF; 4]);
        let before = reader.bits_remaining();
        let res = decode_band_resolutions_grounded(&mut reader, 0).unwrap();
        assert!(res.is_empty());
        assert_eq!(reader.bits_remaining(), before, "0 bands reads no bits");
    }

    #[test]
    fn grounded_single_band_reads_ctx0_and_wraps() {
        // One band: ctx 0 (`res-1`), value wrapped to the signed ring.
        // Use every res-1 row so the wrap is exercised across the full
        // 0..=16 raw alphabet (the row whose symbol is 16 must yield −1).
        for row in 0..SV8_RES_1_TABLE.lengths.len() {
            let e = SV8_RES_1_TABLE.lengths[row];
            let mut p = BitPacker::new();
            p.push(e.code, e.length);
            let bytes = p.finish();
            let mut reader = Sv7BitReader::new(&bytes);
            let res = decode_band_resolutions_grounded(&mut reader, 1).unwrap();
            assert_eq!(res.len(), 1);
            let raw = symbol_for_row(&SV8_RES_1_TABLE, row) as i32;
            assert_eq!(res[0], wrap_res_ref(raw), "res-1 row {row}");
        }
    }

    #[test]
    fn grounded_matches_replicated_spec_walk_for_varied_chains() {
        // Drive several multi-band chains by choosing length-table rows;
        // the impl's ascending output must equal the spec replica's.
        let chains: &[&[usize]] = &[
            &[0, 0, 0],
            &[0, 1, 2, 3],
            &[5, 0, 7, 1, 0],
            &[10, 10, 10, 10, 10, 10],
            &[0, 11, 0, 11],
        ];
        for chain in chains {
            // Skip any row index out of range for the table it would be
            // read from would be a test bug; both res tables have ≥12
            // rows so 0..=11 is always safe.
            let (bytes, expected) = replicate_grounded(chain);
            let mut reader = Sv7BitReader::new(&bytes);
            let got = decode_band_resolutions_grounded(&mut reader, chain.len() as u8).unwrap();
            assert_eq!(got, expected, "chain {chain:?}");
        }
    }

    #[test]
    fn grounded_output_is_ascending_band_order() {
        // Construct a chain whose top-down decode is strictly recognis-
        // able, then assert the returned vec is reversed (band 0 first).
        // Top band reads row 0 of res-1; with a distinct second row the
        // two bands differ, so order is observable.
        let top_e = SV8_RES_1_TABLE.lengths[0];
        let top_sym = symbol_for_row(&SV8_RES_1_TABLE, 0) as i32;
        let top_res = wrap_res_ref(top_sym);
        let ctx_below = res_ctx_for_above_ref(top_res);
        let below_table = if ctx_below == 0 {
            &SV8_RES_1_TABLE
        } else {
            &SV8_RES_2_TABLE
        };
        // pick a below-row whose folded result differs from top_res
        let mut chosen: Option<(usize, i8)> = None;
        for r in 0..below_table.lengths.len() {
            let raw = symbol_for_row(below_table, r) as i32;
            let res = wrap_res_ref(raw + top_res as i32);
            if res != top_res {
                chosen = Some((r, res));
                break;
            }
        }
        let (below_row, below_res) = chosen.expect("a distinct below band exists");
        let mut p = BitPacker::new();
        p.push(top_e.code, top_e.length);
        let be = below_table.lengths[below_row];
        p.push(be.code, be.length);
        let bytes = p.finish();
        let mut reader = Sv7BitReader::new(&bytes);
        let got = decode_band_resolutions_grounded(&mut reader, 2).unwrap();
        // ascending: band 0 (decoded last / bottom) then band 1 (top).
        assert_eq!(got, vec![below_res, top_res]);
    }

    // ─── decode_band_resolutions_stereo_grounded (§6.2, r419) ──

    /// Hand-replicate the stereo walk: per band top-down, L then R,
    /// each channel folding its own chain (ctx from that channel's
    /// band-above, delta onto it, §6.2 wrap). `rows[i] = [lrow, rrow]`
    /// selects the length-table row each channel reads at step `i`
    /// (step 0 = top band).
    fn replicate_stereo(rows: &[[usize; 2]]) -> (Vec<u8>, Vec<[i8; 2]>) {
        let mut p = BitPacker::new();
        let mut top_down: Vec<[i8; 2]> = Vec::new();
        let mut above: [Option<i8>; 2] = [None, None];
        for pair in rows {
            let mut out = [0i8; 2];
            for ch in 0..2 {
                let ctx = match above[ch] {
                    None => 0,
                    Some(a) => res_ctx_for_above_ref(a),
                };
                let table = if ctx == 0 {
                    &SV8_RES_1_TABLE
                } else {
                    &SV8_RES_2_TABLE
                };
                let e = table.lengths[pair[ch]];
                p.push(e.code, e.length);
                let raw = symbol_for_row(table, pair[ch]) as i32;
                let res = match above[ch] {
                    None => wrap_res_ref(raw),
                    Some(a) => wrap_res_ref(raw + a as i32),
                };
                out[ch] = res;
                above[ch] = Some(res);
            }
            top_down.push(out);
        }
        let mut ascending = top_down.clone();
        ascending.reverse();
        (p.finish(), ascending)
    }

    #[test]
    fn stereo_grounded_zero_bands_reads_nothing() {
        let mut reader = Sv7BitReader::new(&[0xFF; 4]);
        let before = reader.bits_remaining();
        let res = decode_band_resolutions_stereo_grounded(&mut reader, 0).unwrap();
        assert!(res.is_empty());
        assert_eq!(reader.bits_remaining(), before);
    }

    #[test]
    fn stereo_grounded_matches_replicated_spec_walk() {
        let chains: &[&[[usize; 2]]] = &[
            &[[0, 0]],
            &[[0, 1], [2, 3], [1, 0]],
            &[[5, 0], [7, 1], [0, 5], [3, 3]],
            &[[10, 11], [10, 11], [0, 0], [11, 10], [1, 2]],
        ];
        for chain in chains {
            let (bytes, expected) = replicate_stereo(chain);
            let mut reader = Sv7BitReader::new(&bytes);
            let got =
                decode_band_resolutions_stereo_grounded(&mut reader, chain.len() as u8).unwrap();
            assert_eq!(got, expected, "chain {chain:?}");
        }
    }

    #[test]
    fn stereo_grounded_channels_fold_independent_chains() {
        // A left chain that ends >2 must NOT flip the right channel's
        // context: give the right channel a small chain and confirm it
        // keeps reading ctx 0 while the left reads ctx 1 below the top.
        // Top band: L raw 5 (ctx0), R raw 0 (ctx0). Next band down:
        // L ctx = above 5 > 2 ⇒ res-2; R ctx = above 0 ⇒ res-1.
        let (bytes, expected) = replicate_stereo(&[[5, 0], [0, 0]]);
        let mut reader = Sv7BitReader::new(&bytes);
        let got = decode_band_resolutions_stereo_grounded(&mut reader, 2).unwrap();
        assert_eq!(got, expected);
        // Sanity: the two channels differ, proving both were decoded.
        assert_ne!(got[1][0], got[1][1]);
    }

    #[test]
    fn stereo_grounded_propagates_eof_mid_walk() {
        // One full pair then starvation on band 2's left read.
        let e = SV8_RES_1_TABLE.lengths[0];
        let mut p = BitPacker::new();
        p.push(e.code, e.length);
        p.push(e.code, e.length);
        let mut bytes = p.bytes.clone();
        if p.nbits > 0 {
            bytes.push((p.acc << (8 - p.nbits)) as u8);
        }
        let mut reader = Sv7BitReader::new(&bytes);
        let res = decode_band_resolutions_stereo_grounded(&mut reader, 2);
        assert!(matches!(res, Err(Error::UnexpectedEof)));
    }

    #[test]
    fn grounded_propagates_eof_mid_walk() {
        let table = &SV8_RES_1_TABLE;
        let e = table.lengths[0];
        let mut p = BitPacker::new();
        p.push(e.code, e.length);
        let mut bytes = p.bytes.clone();
        if p.nbits > 0 {
            bytes.push((p.acc << (8 - p.nbits)) as u8);
        }
        // No trailing zero bytes: band 0 decodes, band 1's peek starves.
        let mut reader = Sv7BitReader::new(&bytes);
        let res = decode_band_resolutions_grounded(&mut reader, 2);
        assert!(matches!(res, Err(Error::UnexpectedEof)));
    }

    // ─── §6.5 bounded "log" code ───────────────────────────

    /// Reference phased-binary encoder for value `v` in `0..max`:
    /// mirrors the §6.5 decode so a roundtrip pins the convention.
    /// Returns the codeword bits MSB-first as (pattern, length).
    fn log_encode(v: u32, max: u32) -> (u16, u8) {
        if max <= 1 {
            return (0, 0);
        }
        let mut bitlen: u8 = 0;
        while (1u32 << bitlen) < max {
            bitlen += 1;
        }
        let lost = (1u32 << bitlen) - max;
        if v < lost {
            // short codeword: v in (bitlen - 1) bits
            let len = bitlen - 1;
            ((v as u16) << (16 - len), len)
        } else {
            // long codeword: (v + lost) in bitlen bits
            let code = v + lost;
            ((code as u16) << (16 - bitlen), bitlen)
        }
    }

    #[test]
    fn log_code_roundtrips_every_value_for_varied_max() {
        for max in 1..=40u32 {
            for v in 0..max {
                let (pat, len) = log_encode(v, max);
                let mut p = BitPacker::new();
                if len > 0 {
                    p.push(pat, len);
                }
                let bytes = p.finish();
                let mut reader = Sv7BitReader::new(&bytes);
                let before = reader.bits_remaining();
                let got = decode_log_code(&mut reader, max).unwrap();
                assert_eq!(got, v, "max {max} value {v}");
                // Codeword width is bitlen-1 (short) or bitlen (long).
                let consumed = before - reader.bits_remaining();
                assert!(consumed <= 8, "max {max} v {v}: {consumed} bits");
            }
        }
    }

    #[test]
    fn log_code_max_le_one_reads_nothing() {
        for max in [0u32, 1] {
            let mut reader = Sv7BitReader::new(&[0xFF; 2]);
            let before = reader.bits_remaining();
            assert_eq!(decode_log_code(&mut reader, max).unwrap(), 0);
            assert_eq!(reader.bits_remaining(), before, "max {max}");
        }
    }

    #[test]
    fn log_code_power_of_two_max_is_plain_fixed_width() {
        // max = 8: lost = 0, every value uses 3 bits, no short tail.
        for v in 0..8u32 {
            let mut p = BitPacker::new();
            p.push((v as u16) << 13, 3);
            let bytes = p.finish();
            let mut reader = Sv7BitReader::new(&bytes);
            assert_eq!(decode_log_code(&mut reader, 8).unwrap(), v);
        }
    }

    #[test]
    fn log_code_propagates_eof() {
        // max = 32 needs up to 5 bits; an empty reader starves.
        let mut reader = Sv7BitReader::new(&[]);
        assert!(matches!(
            decode_log_code(&mut reader, 32),
            Err(Error::UnexpectedEof),
        ));
    }

    // ─── §6.2 Max_used_Band (key + non-key) ────────────────

    #[test]
    fn keyframe_max_used_band_is_log_code_over_the_count_range() {
        // The code space covers the count 0..=max_band+1 (max_band+2
        // values) — fixture-pinned r419 (see the fn docs).
        for max_band in [1u8, 5, 16, 31] {
            for v in 0..=max_band as u32 + 1 {
                let (pat, len) = log_encode(v, max_band as u32 + 2);
                let mut p = BitPacker::new();
                if len > 0 {
                    p.push(pat, len);
                }
                let bytes = p.finish();
                let mut reader = Sv7BitReader::new(&bytes);
                let got = decode_keyframe_max_used_band(&mut reader, max_band).unwrap();
                assert_eq!(got as u32, v, "max_band {max_band} value {v}");
            }
        }
    }

    #[test]
    fn nonkey_max_used_band_folds_delta_and_wraps_above_32() {
        // canon(Bands) row 0 is the shortest codeword; compute its delta
        // and verify last_max_band + delta (wrapped) equals the result.
        for row in 0..SV8_BANDS_TABLE.lengths.len() {
            let delta = symbol_for_row(&SV8_BANDS_TABLE, row) as i32;
            for last in [0u8, 10, 32] {
                let mut expected = last as i32 + delta;
                if expected > 32 {
                    expected -= 33;
                }
                if !(0..=32).contains(&expected) {
                    continue; // a defensive-reject combo; skip here
                }
                let e = SV8_BANDS_TABLE.lengths[row];
                let mut p = BitPacker::new();
                p.push(e.code, e.length);
                let bytes = p.finish();
                let mut reader = Sv7BitReader::new(&bytes);
                let got = decode_nonkey_max_used_band(&mut reader, last).unwrap();
                assert_eq!(got as i32, expected, "row {row} last {last} delta {delta}");
            }
        }
    }

    #[test]
    fn nonkey_max_used_band_wrap_pins_above_32_to_minus_33() {
        // A synthetic delta is not directly injectable (the table is
        // fixed), so pin the wrap arithmetic directly: any sum in
        // 33..=64 maps to sum-33 (i.e. 0..=31).
        for sum in 33..=64i32 {
            let wrapped = sum - 33;
            assert!((0..=31).contains(&wrapped), "sum {sum}");
        }
        // And confirm the impl applies it: the bands table's largest
        // symbol added to last=32 — if it pushes over 32, it wraps.
        let max_sym = (0..SV8_BANDS_TABLE.lengths.len())
            .map(|r| symbol_for_row(&SV8_BANDS_TABLE, r) as i32)
            .max()
            .unwrap();
        let row = (0..SV8_BANDS_TABLE.lengths.len())
            .find(|&r| symbol_for_row(&SV8_BANDS_TABLE, r) as i32 == max_sym)
            .unwrap();
        let sum = 32 + max_sym;
        if sum > 32 && (0..=32).contains(&(sum - 33)) {
            let e = SV8_BANDS_TABLE.lengths[row];
            let mut p = BitPacker::new();
            p.push(e.code, e.length);
            let bytes = p.finish();
            let mut reader = Sv7BitReader::new(&bytes);
            let got = decode_nonkey_max_used_band(&mut reader, 32).unwrap();
            assert_eq!(got as i32, sum - 33);
        }
    }

    #[test]
    fn keyframe_max_used_band_propagates_eof() {
        let mut reader = Sv7BitReader::new(&[]);
        assert!(matches!(
            decode_keyframe_max_used_band(&mut reader, 31),
            Err(Error::UnexpectedEof),
        ));
    }

    // ─── §6.2 SV8 M/S band-selection ───────────────────────

    fn comb(n: u32, k: u32) -> u32 {
        if k > n {
            return 0;
        }
        let k = if k > n - k { n - k } else { k };
        let mut num: u64 = 1;
        for i in 0..k {
            num = num * (n - i) as u64 / (i + 1) as u64;
        }
        num as u32
    }

    /// Reference §6.5 enumerative encoder: given an `n`-bit `mask` with
    /// exactly `k` set bits, emit the phased-binary codeword (combinadic
    /// rank + phased-binary index) MSB-first into `p`.
    fn pack_enum_subset(p: &mut BitPacker, mask: u32, k: u32, n: u32) {
        // combinadic rank
        let mut rank: u32 = 0;
        let mut kk = k;
        for m in (0..n).rev() {
            if kk == 0 {
                break;
            }
            if mask & (1 << m) != 0 {
                rank += comb(m, kk);
                kk -= 1;
            }
        }
        let total = comb(n, k);
        if total <= 1 {
            return; // single codeword, no bits
        }
        let mut bitlen: u8 = 0;
        while (1u32 << bitlen) < total {
            bitlen += 1;
        }
        let lost = (1u32 << bitlen) - total;
        if rank < lost {
            p.push((rank as u16) << (16 - (bitlen - 1)), bitlen - 1);
        } else {
            let code = rank + lost;
            p.push((code as u16) << (16 - bitlen), bitlen);
        }
    }

    /// Encode a §6.2 M/S flag schedule (lowest non-zero band first —
    /// the fixture-pinned r419 orientation) into a stream that
    /// [`decode_sv8_ms_flags`] must reproduce.
    fn pack_ms_flags(flags: &[bool]) -> Vec<u8> {
        let n = flags.len() as u32;
        let mut p = BitPacker::new();
        // mask MSB-first: out[0] (lowest band) is bit n-1.
        let mut mask: u32 = 0;
        for (i, &f) in flags.iter().enumerate() {
            if f {
                mask |= 1 << (n - 1 - i as u32);
            }
        }
        let cnt = mask.count_ones();
        // log code for cnt over 0..n+1
        let (pat, len) = log_encode(cnt, n + 1);
        if len > 0 {
            p.push(pat, len);
        }
        if cnt != 0 && cnt != n {
            // Encode the smaller subset; invert the mask first if the
            // complement is smaller.
            let k = cnt.min(n - cnt);
            let coded_mask = if cnt > n / 2 {
                (!mask) & ((1u32 << n) - 1)
            } else {
                mask
            };
            pack_enum_subset(&mut p, coded_mask, k, n);
        }
        p.finish()
    }

    #[test]
    fn ms_flags_zero_tot_reads_nothing() {
        let mut reader = Sv7BitReader::new(&[0xFF; 4]);
        let before = reader.bits_remaining();
        assert!(decode_sv8_ms_flags(&mut reader, 0).unwrap().is_empty());
        assert_eq!(reader.bits_remaining(), before);
    }

    #[test]
    fn ms_flags_roundtrip_exhaustive_for_small_tot() {
        // Every flag pattern for tot 1..=12 must roundtrip, exercising
        // cnt==0, cnt==tot, and the enumerative middle (with the
        // complement-inversion branch when cnt > tot/2).
        for tot in 1u8..=12 {
            for bits in 0u32..(1u32 << tot) {
                let flags: Vec<bool> = (0..tot).map(|i| bits & (1 << (tot - 1 - i)) != 0).collect();
                let bytes = pack_ms_flags(&flags);
                let mut reader = Sv7BitReader::new(&bytes);
                let got = decode_sv8_ms_flags(&mut reader, tot).unwrap();
                assert_eq!(got, flags, "tot {tot} pattern {bits:#b}");
            }
        }
    }

    #[test]
    fn ms_flags_all_set_and_none_set_read_only_the_count() {
        // cnt==0 and cnt==tot read no enumerative bits — confirm via the
        // exact-bit roundtrip (already covered above) plus an ordering
        // spot check: only the lowest band flagged.
        let mut flags = vec![false; 6];
        flags[0] = true; // lowest non-zero band M/S
        let bytes = pack_ms_flags(&flags);
        let mut reader = Sv7BitReader::new(&bytes);
        let got = decode_sv8_ms_flags(&mut reader, 6).unwrap();
        assert_eq!(got, flags);
        assert!(got[0] && got[1..].iter().all(|&b| !b));
    }

    #[test]
    fn ms_flags_ascending_ordering_is_observable() {
        // A flag only on the topmost band must land at out[tot-1].
        let mut flags = vec![false; 5];
        flags[4] = true; // topmost band
        let bytes = pack_ms_flags(&flags);
        let mut reader = Sv7BitReader::new(&bytes);
        let got = decode_sv8_ms_flags(&mut reader, 5).unwrap();
        assert_eq!(got, flags);
        assert!(got[4] && got[..4].iter().all(|&b| !b));
    }

    #[test]
    fn ms_flags_propagates_eof() {
        let mut reader = Sv7BitReader::new(&[]);
        assert!(matches!(
            decode_sv8_ms_flags(&mut reader, 8),
            Err(Error::UnexpectedEof),
        ));
    }
}
