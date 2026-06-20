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
//! - **Default arm (`LargeCoeffEscape`, table `sv8-canonical-q9up`,
//!   `band_type` 9..=17)** — "for each sample, read a VLC plus a
//!   fixed number of raw bits" (§3.4). Three staged facts pin the
//!   composition completely:
//!   1. the `sv8-symbols-q9up` map is an exact permutation of
//!      `-128..=127` — the full **signed-byte** alphabet, one table
//!      for the whole 9-and-up range (the `.meta` `spec_role` says
//!      "quantiser case-9-and-up");
//!   2. `requant-res-bits.meta`'s `spec_role` scopes the
//!      bits-per-sample ladder to **"SV7 §2.5 / SV8 §3.4"**, so the
//!      total coded sample width for `band_type` ≥ 9 is
//!      `RES_BITS[band_type] = band_type - 1` bits in SV8 too;
//!   3. `requant-quantizer-offset-Dc` pins the level range `-D..=D`
//!      with `D = 2^(band_type - 2) - 1`.
//!
//!   The only composition consistent with all three is: the VLC
//!   symbol carries the **sign-bearing top 8 bits** and the
//!   `n = RES_BITS[band_type] - 8 = band_type - 9` raw bits carry
//!   the low bits, i.e. `sample = symbol · 2ⁿ + raw` — as `symbol`
//!   ranges over the signed-byte alphabet and `raw` over `0..2ⁿ`,
//!   the composed values tile exactly the `(band_type - 1)`-bit
//!   two's-complement range `[-(D + 1), D]`, covering `-D..=D` (the
//!   single extremum `-(D + 1)` is the usual two's-complement
//!   asymmetry; like the SV7 §2.5 escape's one out-of-range level
//!   it is passed through, encoder never emits it). Were the raw
//!   bits the *high* part instead, the sign would live in the raw
//!   field and the symbol alphabet would have to be unsigned digit
//!   values — contradicting the staged signed map. `band_type` 9
//!   degenerates to `n = 0` (the VLC alone spans `-128..=127` ⊇
//!   `-127..=127 = -D..=D`).
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
//! - **The q2 context-pair selection rule** — *now grounded.* Case 2
//!   sits outside the §3.4 `5..=8` first-order-context prose, yet the
//!   staged tables ship a `{ctx0, ctx1}` pair for it, and
//!   `spec/musepack-headers-and-coding.md` §6.4.2 pins the selection
//!   rule: the same `idx > thres[2]` accumulator (`thres[2] = 3`,
//!   init `idx = 6`) folding `var[tmp]` per group. The grounded path
//!   [`decode_sv8_grouped3_band_grounded`] drives it from
//!   [`crate::sv8_context::Sv8Context`]; [`decode_sv8_grouped3_band`]
//!   keeps the fixed-`ctx` knob form for callers that need it.
//! - **The `5..=8` context-update predicate** — *now grounded.* §3.4
//!   pinned only that the table is "chosen by the previously decoded
//!   sample"; §6.4.2 pins the full predicate (init `idx = 2·thres`,
//!   select context-1 when `idx > thres`, fold
//!   `idx = (idx >> 1) + |q|`). [`decode_sv8_context_band_grounded`]
//!   drives it from [`crate::sv8_context::Sv8Context`];
//!   [`decode_sv8_context_band`] keeps the caller-supplied-closure
//!   form for callers that need to override it.
//! - **Raw-bit field read order (escape arm).** The §3.4 prose
//!   pins the per-sample read order ("a VLC plus a fixed number of
//!   raw bits" — VLC first), but not the bit-significance order
//!   *within* the raw field. [`decode_sv8_escape_band`] reads the
//!   field MSB-first as one `n`-bit unsigned integer via
//!   [`Sv7BitReader::read_bits`] — the same primitive and
//!   convention the SV7 §2.5 escape ladder
//!   ([`crate::sv7_band_decode::decode_linear_pcm_band`]) uses,
//!   backed by §3.6's lossless SV7↔SV8 relationship (identical
//!   quantised-coefficient payload, only the entropy layer
//!   differs).
//!
//! # Case 1 (`SparseBand`) — now grounded by §6.4.1 + §6.5
//!
//! The earlier "19-symbol q1 alphabet cannot carry an 18-flag bitmap"
//! blocker is resolved by the staged
//! `spec/musepack-headers-and-coding.md` §6.4.1: the q1 symbol map's
//! `0..=18` alphabet is **not** a flag bitmap but the per-group
//! **non-zero count** `cnt`. A band's 36 samples are decoded as two
//! halves of 18 ([`SPARSE_GROUP_SIZE`]); for each half a q1 codeword
//! gives `cnt`, then a §6.5 enumerative (combinatorial) codeword names
//! *which* `min(cnt, 18 − cnt)` of the 18 positions are non-zero (the
//! mask is bit-inverted when `cnt > 9` because the smaller complement
//! is always coded), and finally one raw sign bit per present position
//! sets it to `±1` (`requant-quantizer-offset-Dc` pins `D = 1` for
//! `band_type` 1, so the only non-zero levels are `{−1, +1}`).
//! [`decode_sv8_sparse_band`] wires this end to end; the enumerative
//! coder ([`enum_decode_subset`]) and its phased-binary index read are
//! both pinned by §6.5 (binomial code space, `bitlen − 1` bits with a
//! conditional extra "lost-codes" bit, then a combinadic peel). All
//! arithmetic is computed (binomial recurrence) — no new tables.
//!
//! Cases `-1` (CNS) and `0` (empty) are shared arms with SV7 (the
//! round-245 classifier tests pin the agreement) and reuse
//! [`crate::sv7_band_decode::fill_cns_band`] /
//! [`crate::sv7_band_decode::fill_zero_band`] unchanged.

use crate::huffman::Sv7BitReader;
use crate::requant::RES_BITS;
use crate::sv7_band_decode::SAMPLES_PER_BAND;
use crate::sv8_huffman::{table_for_role, Sv8TableRole};
use crate::{Error, Result};

/// Number of grouped codewords per band for §3.4 case 2
/// ("read 12 VLCs to produce the 36 samples").
pub const GROUPED3_CODEWORDS_PER_BAND: usize = 12;

/// Number of grouped codewords per band for §3.4 cases 3..=4
/// ("read 18 VLCs to produce the 36 samples").
pub const GROUPED2_CODEWORDS_PER_BAND: usize = 18;

/// Number of sample bits the §3.4 default-arm escape VLC itself
/// carries: the staged `sv8-symbols-q9up` map is an exact
/// permutation of `-128..=127` — the full signed-byte alphabet —
/// so each codeword fixes the sign-bearing top **8** bits of the
/// sample (see the module-level escape grounding note).
pub const ESCAPE_VLC_SYMBOL_BITS: u8 = 8;

/// Fixed raw-bit count per sample for the §3.4 default-arm
/// large-coefficient escape, for `band_type` in `9..=17`.
///
/// `requant-res-bits.meta` scopes its bits-per-sample ladder to
/// "SV7 §2.5 / SV8 §3.4", so the total coded width of an escape
/// sample is [`RES_BITS`]`[band_type] = band_type - 1` bits; the
/// q9up VLC covers the top [`ESCAPE_VLC_SYMBOL_BITS`] of them,
/// leaving `band_type - 9` raw bits (0 for `band_type` 9, up to 8
/// for `band_type` 17).
///
/// Returns `None` outside `9..=17`: the §3.4 ladder routes every
/// `band_type >= 9` to the escape arm, but the staged requant
/// tables (`requant-res-bits`, `requant-quantizer-offset-Dc`)
/// define quantisers only through `band_type` 17, so larger values
/// have no defined sample width.
pub const fn escape_raw_bits(band_type: i8) -> Option<u8> {
    match band_type {
        9..=17 => Some(RES_BITS[band_type as usize] - ESCAPE_VLC_SYMBOL_BITS),
        _ => None,
    }
}

/// Sign-extend the low 4 bits of `n` as a two's-complement nibble.
const fn sign_extend_nibble(n: u8) -> i8 {
    (((n & 0x0F) << 4) as i8) >> 4
}

/// Number of positions in each of the two sparse-band groups
/// (§6.4.1: "the 36 samples are decoded in two halves of 18").
pub const SPARSE_GROUP_SIZE: usize = 18;

/// Binomial coefficient `C(n, k)` for the small `n ≤ 18` the §6.5
/// enumerative coder operates over.
///
/// Computed with the multiplicative recurrence so no precomputed
/// table is needed; `C(18, 9) = 48620` is the largest value reached
/// in the sparse-band path and fits comfortably in `u32`.
const fn binomial(n: u32, k: u32) -> u32 {
    if k > n {
        return 0;
    }
    let k = if k > n - k { n - k } else { k };
    let mut num: u64 = 1;
    let mut i: u32 = 0;
    while i < k {
        num = num * (n - i) as u64 / (i + 1) as u64;
        i += 1;
    }
    num as u32
}

/// `(bitlen, lost)` for a §6.5 enumerative code space of `total`
/// distinct codewords.
///
/// `bitlen = ceil(log2(total))` is the width of a full (long)
/// codeword; `lost = 2^bitlen − total` is the count of "lost codes"
/// the non-power-of-two code space leaves over — those many codewords
/// are stored one bit shorter (the truncated-/phased-binary code the
/// §6.5 prose describes: "Decode reads `bitlen − 1` bits; if the
/// value reaches into the 'lost-codes' region it reads one more bit
/// and rebases"). `total ≤ 1` ⇒ a single codeword carrying zero bits.
const fn enum_bitlen_lost(total: u32) -> (u8, u32) {
    if total <= 1 {
        return (0, 0);
    }
    // bitlen = ceil(log2(total))
    let mut bitlen: u8 = 0;
    while (1u32 << bitlen) < total {
        bitlen += 1;
    }
    let lost = (1u32 << bitlen) - total;
    (bitlen, lost)
}

/// Decode one §6.5 enumerative (combinatorial) codeword naming a
/// specific `k`-subset of `n` positions, returning the selected
/// positions as a low-`n`-bit mask (bit `p` set ⇒ position `p`
/// selected).
///
/// Two stages, both pinned by §6.5:
///
/// 1. **Phased-binary index read.** The code space has
///    `total = C(n, k)` codewords; with
///    `(bitlen, lost) = `[`enum_bitlen_lost`]`(total)`, read
///    `bitlen − 1` bits as `code`; if `code ≥ lost` the codeword is a
///    full one, so read one more bit and rebase
///    `code = (code << 1) − lost + bit`. The `lost` short codewords
///    (`code < lost`) carry the low-ranked subsets.
/// 2. **Combinadic peel.** Walk positions `m` from `n − 1` down to
///    `0`; at each, if the running `code ≥ C(m, k)` mark position `m`,
///    subtract `C(m, k)`, and decrement `k`. The result is exactly an
///    `n`-bit mask with `k` set bits.
///
/// `k == 0` selects the empty subset (no bits read, mask 0); `k == n`
/// selects all positions (a single codeword, no bits read).
///
/// `pub(crate)` so the §6.2 SV8 M/S band-selection
/// ([`crate::sv8_band_header::decode_sv8_ms_flags`]) can reuse the same
/// §6.5 enumerative coder the sparse arm uses.
pub(crate) fn enum_decode_subset(reader: &mut Sv7BitReader<'_>, k: u32, n: u32) -> Result<u32> {
    let total = binomial(n, k);
    let (bitlen, lost) = enum_bitlen_lost(total);
    let mut code: u32 = if bitlen == 0 {
        0
    } else {
        let mut c = reader.read_bits(bitlen - 1)? as u32;
        if c >= lost {
            c = (c << 1) - lost + reader.read_bits(1)? as u32;
        }
        c
    };
    let mut mask: u32 = 0;
    let mut kk = k;
    let mut m = n;
    while m > 0 && kk > 0 {
        m -= 1;
        let c = binomial(m, kk);
        if code >= c {
            mask |= 1 << m;
            code -= c;
            kk -= 1;
        }
    }
    Ok(mask)
}

/// Decode one §6.4.1 sparse-band group of [`SPARSE_GROUP_SIZE`]
/// (= 18) samples into `out`, given the per-group non-zero count
/// `cnt` (already decoded from the `sv8-canonical-q1` table by the
/// caller).
///
/// Per §6.4.1:
///
/// 1. `cnt == 0` ⇒ all 18 samples are zero (no bits read);
///    `cnt == 18` ⇒ all 18 positions are present (no enumerative bits
///    read — the full mask is implied).
/// 2. Otherwise read a §6.5 enumerative codeword
///    ([`enum_decode_subset`]) selecting `min(cnt, 18 − cnt)`
///    positions; when `cnt > 9` the coder named the *complement* (the
///    smaller of the two selections is always coded), so the mask is
///    bit-inverted within the 18-bit field to recover the present
///    positions.
/// 3. Walk the 18 positions MSB-first (position 17 down to 0); each
///    present position reads **one raw sign bit** and is set to
///    `(bit << 1) − 1`, i.e. `+1` for a 1 bit and `−1` for a 0 bit;
///    absent positions stay 0.
///
/// `cnt > 18` yields [`Error::GroupedSymbolOutOfRange`] (a malformed
/// q1 symbol — the staged map spans `0..=18` exactly, so this is
/// unreachable for a well-formed stream and kept as a defensive
/// bound).
fn decode_sparse_group(
    reader: &mut Sv7BitReader<'_>,
    cnt: u8,
    out: &mut [i8; SPARSE_GROUP_SIZE],
) -> Result<()> {
    *out = [0; SPARSE_GROUP_SIZE];
    let n = SPARSE_GROUP_SIZE as u32;
    let cnt_u = cnt as u32;
    if cnt_u > n {
        return Err(Error::GroupedSymbolOutOfRange(cnt as i8));
    }
    let present_mask: u32 = if cnt_u == 0 {
        0
    } else if cnt_u == n {
        (1 << n) - 1
    } else {
        let k = cnt_u.min(n - cnt_u);
        let coded = enum_decode_subset(reader, k, n)?;
        if cnt_u > n / 2 {
            (!coded) & ((1 << n) - 1)
        } else {
            coded
        }
    };
    // Walk positions MSB-first; each present position reads one sign
    // bit and becomes ±1.
    for p in (0..SPARSE_GROUP_SIZE).rev() {
        if present_mask & (1 << p) != 0 {
            let bit = reader.read_bits(1)? as i8;
            out[p] = (bit << 1) - 1;
        }
    }
    Ok(())
}

/// Decode 36 samples for an SV8 band with `band_type == 1`
/// (§3.4 / §6.4.1 sparse case,
/// [`crate::sv8_band_decode::Sv8BandDecodeCase::SparseBand`]).
///
/// The band is decoded as **two halves of 18**
/// ([`SPARSE_GROUP_SIZE`]); for each half:
///
/// 1. one `sv8-canonical-q1` canonical-Huffman codeword decodes the
///    half's non-zero count `cnt` (the q1 symbol map is the 19-symbol
///    alphabet `0..=18`, exactly one count per group of 18 — the fact
///    earlier rounds could not reconcile with the older "18-flag
///    bitmap" reading and that §6.4.1 now resolves);
/// 2. [`decode_sparse_group`] reads the §6.5 enumerative
///    position-selection codeword plus one sign bit per present
///    position, filling that half with values in `{−1, 0, +1}`.
///
/// Output samples are already-centred levels in `{−1, 0, +1}`
/// (`requant-quantizer-offset-Dc` pins `D = 1` for `band_type` 1), so
/// the `[i8; 36]` shape matches the other grouped arms and the
/// [`crate::sv8_band_decode::decode_sv8_band`] dispatcher widens it to
/// `[i32; 36]` loss-free.
///
/// A malformed q1 count (> 18) yields
/// [`Error::GroupedSymbolOutOfRange`]; EOF in any phase propagates as
/// [`Error::UnexpectedEof`].
pub fn decode_sv8_sparse_band(
    reader: &mut Sv7BitReader<'_>,
    out: &mut [i8; SAMPLES_PER_BAND],
) -> Result<()> {
    let table = table_for_role(Sv8TableRole::Q1, 0).ok_or(Error::UnsupportedBandType(1))?;
    for half in out.chunks_exact_mut(SPARSE_GROUP_SIZE) {
        let cnt = table.decode(reader)?;
        if !(0..=SPARSE_GROUP_SIZE as i8).contains(&cnt) {
            return Err(Error::GroupedSymbolOutOfRange(cnt));
        }
        let mut group = [0_i8; SPARSE_GROUP_SIZE];
        decode_sparse_group(reader, cnt as u8, &mut group)?;
        half.copy_from_slice(&group);
    }
    Ok(())
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

/// Decode 36 samples for an SV8 band with `band_type == 2`
/// (§3.4 case 2) using the **grounded** §6.4.2 first-order context
/// model rather than a fixed caller-supplied `ctx`.
///
/// The canonical-decode-path sibling of [`decode_sv8_grouped3_band`]:
/// the `sv8-canonical-q2-{1,2}` half-of-pair pick for each of the 12
/// groups is driven by the [`crate::sv8_context::Sv8Context`]
/// accumulator that `spec/musepack-headers-and-coding.md` §6.4.2 pins.
/// The accumulator starts at `2 × thres[2] = 6` (so the first group
/// reads from context-1, since `6 > thres[2] = 3`); each group's table
/// is context-1 when `idx > 3` else context-0; and after each decoded
/// group's product index `tmp` the accumulator folds in
/// `idx = (idx >> 1) + var[tmp]`, with `var[tmp]` the summed magnitude
/// of the three samples `tmp` encodes
/// ([`crate::sv8_context::case2_magnitude`]). The product index is the
/// raw symbol the q2 table decodes *before* un-bundling, so the
/// magnitude fold uses the symbol directly.
pub fn decode_sv8_grouped3_band_grounded(
    reader: &mut Sv7BitReader<'_>,
    out: &mut [i8; SAMPLES_PER_BAND],
) -> Result<()> {
    let mut ctx = crate::sv8_context::Sv8Context::new(2).ok_or(Error::UnsupportedBandType(2))?;
    for group in out.chunks_exact_mut(3) {
        let table = table_for_role(Sv8TableRole::Q2, ctx.table_ctx())
            .ok_or(Error::UnsupportedBandType(2))?;
        let symbol = table.decode(reader)?;
        group.copy_from_slice(&unpack_grouped3_symbol(symbol)?);
        ctx.update_group(symbol as i32);
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

/// Decode 36 samples for an SV8 band with `band_type` in `5..=8`
/// (§3.4 cases 5..8) using the **grounded** §6.4.2 first-order context
/// model rather than a caller-supplied predicate.
///
/// This is the canonical-decode-path sibling of
/// [`decode_sv8_context_band`]: the table pick for each sample is
/// driven by the [`crate::sv8_context::Sv8Context`] accumulator that
/// `spec/musepack-headers-and-coding.md` §6.4.2 pins — the accumulator
/// starts at `2 × thres[band_type]` (so the first sample reads from
/// context-1), each sample's table is context-1 when `idx > thres`
/// else context-0, and after each decoded sample `q` the accumulator
/// folds in `idx = (idx >> 1) + |q|`. No knob: the predicate comes
/// entirely from the staged §6.4.2 facts.
///
/// A `band_type` outside `5..=8` yields [`Error::UnsupportedBandType`].
pub fn decode_sv8_context_band_grounded(
    reader: &mut Sv7BitReader<'_>,
    band_type: i8,
    out: &mut [i8; SAMPLES_PER_BAND],
) -> Result<()> {
    let role = match band_type {
        5 => Sv8TableRole::Q5,
        6 => Sv8TableRole::Q6,
        7 => Sv8TableRole::Q7,
        8 => Sv8TableRole::Q8,
        _ => return Err(Error::UnsupportedBandType(band_type)),
    };
    let mut ctx = crate::sv8_context::Sv8Context::new(band_type)
        .ok_or(Error::UnsupportedBandType(band_type))?;
    for slot in out.iter_mut() {
        let table =
            table_for_role(role, ctx.table_ctx()).ok_or(Error::UnsupportedBandType(band_type))?;
        *slot = table.decode(reader)?;
        ctx.update_sample(*slot);
    }
    Ok(())
}

/// Decode 36 samples for an SV8 band with `band_type` in `9..=17`
/// (§3.4 `default` arm,
/// [`crate::sv8_band_decode::Sv8BandDecodeCase::LargeCoeffEscape`]):
/// per sample, one canonical-Huffman codeword from
/// `sv8-canonical-q9up` followed by [`escape_raw_bits`]`(band_type)`
/// raw bits, composed as
///
/// ```text
/// sample = (symbol << n) | raw      # n = band_type - 9
/// ```
///
/// i.e. the signed-byte VLC symbol carries the sign-bearing top 8
/// bits and the raw field the low `n` bits of a
/// `(band_type - 1)`-bit two's-complement level — the composition
/// the staged q9up alphabet + `requant-res-bits` +
/// `requant-quantizer-offset-Dc` facts pin (module-level escape
/// grounding note). The raw field is read MSB-first via
/// [`Sv7BitReader::read_bits`], mirroring the SV7 §2.5 escape
/// ladder per the §3.6 lossless SV7↔SV8 relationship (module-level
/// "Conventions" note).
///
/// Output samples are already-centred levels in
/// `[-(D + 1), D]` with `D = 2^(band_type - 2) - 1` (the lone
/// `-(D + 1)` extremum is the two's-complement asymmetry, passed
/// through like the SV7 escape's one out-of-range level), so `out`
/// is `i32` — the levels exceed `i8` for every escape `band_type`.
/// Unlike [`crate::sv7_band_decode::decode_linear_pcm_band`] (which
/// emits raw *uncentred* levels for the caller to centre by `D`),
/// the staged q9up map is signed, so no caller-side centring
/// applies here.
///
/// A `band_type` outside `9..=17` yields
/// [`Error::UnsupportedBandType`] (the staged requant tables define
/// quantisers only through `band_type` 17; see
/// [`escape_raw_bits`]).
pub fn decode_sv8_escape_band(
    reader: &mut Sv7BitReader<'_>,
    band_type: i8,
    out: &mut [i32; SAMPLES_PER_BAND],
) -> Result<()> {
    let raw_bits = escape_raw_bits(band_type).ok_or(Error::UnsupportedBandType(band_type))?;
    let table =
        table_for_role(Sv8TableRole::Q9up, 0).ok_or(Error::UnsupportedBandType(band_type))?;
    for slot in out.iter_mut() {
        let symbol = table.decode(reader)? as i32;
        // read_bits(0) is a defined no-op returning 0 (band_type 9).
        let raw = reader.read_bits(raw_bits)? as i32;
        *slot = (symbol << raw_bits) | raw;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::requant::QUANTIZER_OFFSET_D;
    use crate::sv8_huffman::{
        Sv8CanonicalTable, SV8_Q2_1_TABLE, SV8_Q2_2_TABLE, SV8_Q3_TABLE, SV8_Q4_TABLE,
        SV8_Q5_1_TABLE, SV8_Q5_2_TABLE, SV8_Q6_1_TABLE, SV8_Q6_2_TABLE, SV8_Q7_1_TABLE,
        SV8_Q7_2_TABLE, SV8_Q8_1_TABLE, SV8_Q8_2_TABLE, SV8_Q9UP_TABLE,
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

        /// Push the low `length` bits of `value` MSB-first (a
        /// right-justified raw field, the convention the enumerative
        /// index and sign bits use).
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

    // ─── decode_sv8_context_band_grounded (§6.4.2 cases 5..=8) ─────

    #[test]
    fn grounded_context_band_starts_in_context_one() {
        // §6.4.2: idx init = 2·thres > thres, so the FIRST sample must
        // read from the context-1 table (q{bt}-2). Build a stream whose
        // leading codeword is the shortest ctx-1 codeword; the grounded
        // decoder must produce that ctx-1 symbol for sample 0.
        for (band_type, t1) in [
            (5_i8, &SV8_Q5_2_TABLE),
            (6, &SV8_Q6_2_TABLE),
            (7, &SV8_Q7_2_TABLE),
            (8, &SV8_Q8_2_TABLE),
        ] {
            let entry1 = t1.lengths[0];
            let sym_via_ctx1 = symbol_for_row(t1, 0);
            let mut p = BitPacker::new();
            for _ in 0..SAMPLES_PER_BAND {
                p.push(entry1.code, entry1.length);
            }
            let bytes = p.finish();
            let mut reader = Sv7BitReader::new(&bytes);
            let mut out = [0_i8; SAMPLES_PER_BAND];
            decode_sv8_context_band_grounded(&mut reader, band_type, &mut out).expect("decode");
            assert_eq!(
                out[0], sym_via_ctx1,
                "band_type {band_type}: first sample must use ctx-1 (idx init = 2·thres)",
            );
        }
    }

    #[test]
    fn grounded_context_band_matches_replicated_accumulator() {
        // Feed the closure variant a predicate that replicates the
        // §6.4.2 accumulator and assert byte-for-byte agreement with
        // the grounded path, over an arbitrary mixed codeword stream.
        for band_type in [5_i8, 6, 7, 8] {
            let t0 = match band_type {
                5 => &SV8_Q5_1_TABLE,
                6 => &SV8_Q6_1_TABLE,
                7 => &SV8_Q7_1_TABLE,
                _ => &SV8_Q8_1_TABLE,
            };
            // A long run of the shortest ctx-0 codeword. The grounded
            // path may read some samples through ctx-1 (whose codewords
            // can be longer), so over-provision the buffer well past 36
            // codewords so neither path runs out of bits.
            let entry = t0.lengths[0];
            let mut p = BitPacker::new();
            for _ in 0..(SAMPLES_PER_BAND * 4) {
                p.push(entry.code, entry.length);
            }
            let bytes = p.finish();

            // Grounded path.
            let mut r_g = Sv7BitReader::new(&bytes);
            let mut out_g = [0_i8; SAMPLES_PER_BAND];
            decode_sv8_context_band_grounded(&mut r_g, band_type, &mut out_g).expect("grounded");

            // Closure path with a hand-rolled §6.4.2 accumulator. The
            // grounded decoder uses table_ctx() BEFORE update_sample();
            // the closure form starts from initial_ctx (the init
            // table_ctx) and updates after each sample, so we mirror the
            // accumulator's idx exactly.
            let thres = crate::sv8_context::context_threshold(band_type).unwrap();
            let mut idx = 2 * thres;
            let initial_ctx = (idx > thres) as u8;
            let mut r_c = Sv7BitReader::new(&bytes);
            let mut out_c = [0_i8; SAMPLES_PER_BAND];
            decode_sv8_context_band(
                &mut r_c,
                band_type,
                initial_ctx,
                |prev| {
                    idx = (idx >> 1) + (prev as i32).unsigned_abs();
                    (idx > thres) as u8
                },
                &mut out_c,
            )
            .expect("closure");

            assert_eq!(
                out_g, out_c,
                "band_type {band_type}: grounded vs replicated"
            );
        }
    }

    #[test]
    fn grounded_context_band_rejects_band_type_outside_5_8() {
        let mut out = [0_i8; SAMPLES_PER_BAND];
        for bt in [-1_i8, 0, 1, 2, 3, 4, 9, 17, i8::MAX] {
            let mut reader = Sv7BitReader::new(&[0xFF; 128]);
            assert!(matches!(
                decode_sv8_context_band_grounded(&mut reader, bt, &mut out),
                Err(Error::UnsupportedBandType(v)) if v == bt,
            ));
        }
    }

    #[test]
    fn grounded_context_band_propagates_eof() {
        let mut reader = Sv7BitReader::new(&[0xFF]);
        let mut out = [0_i8; SAMPLES_PER_BAND];
        assert!(matches!(
            decode_sv8_context_band_grounded(&mut reader, 5, &mut out),
            Err(Error::UnexpectedEof),
        ));
    }

    // ─── decode_sv8_grouped3_band_grounded (§6.4.2 case 2) ─────────

    #[test]
    fn grounded_grouped3_band_starts_in_context_one() {
        // §6.4.2: idx init = 2·thres[2] = 6 > thres[2] = 3, so the first
        // group reads from the q2-2 (context-1) table. A stream of
        // shortest-q2-2 codewords must therefore decode sample 0's
        // triplet to the q2-2 row-0 symbol.
        let entry1 = SV8_Q2_2_TABLE.lengths[0];
        let sym_via_ctx1 = symbol_for_row(&SV8_Q2_2_TABLE, 0);
        let mut p = BitPacker::new();
        for _ in 0..GROUPED3_CODEWORDS_PER_BAND {
            p.push(entry1.code, entry1.length);
        }
        let bytes = p.finish();
        let mut reader = Sv7BitReader::new(&bytes);
        let mut out = [0_i8; SAMPLES_PER_BAND];
        decode_sv8_grouped3_band_grounded(&mut reader, &mut out).expect("decode");
        let first_triplet = unpack_grouped3_symbol(sym_via_ctx1).unwrap();
        assert_eq!(
            &out[0..3],
            &first_triplet[..],
            "first group must use q2-2 (ctx-1) per idx init = 2·thres[2]",
        );
    }

    #[test]
    fn grounded_grouped3_band_matches_replicated_accumulator() {
        // A stream of shortest-q2-1 codewords; verify the grounded path
        // reproduces a hand-rolled §6.4.2 accumulator driving the q2
        // table pick per group with the case2_magnitude fold.
        let entry = SV8_Q2_1_TABLE.lengths[0];
        let mut p = BitPacker::new();
        for _ in 0..GROUPED3_CODEWORDS_PER_BAND {
            p.push(entry.code, entry.length);
        }
        let bytes = p.finish();

        let mut r_g = Sv7BitReader::new(&bytes);
        let mut out_g = [0_i8; SAMPLES_PER_BAND];
        decode_sv8_grouped3_band_grounded(&mut r_g, &mut out_g).expect("grounded");

        // Replicate the §6.4.2 per-group accumulator by hand (the knob
        // variant decodes all 12 groups under one fixed ctx, so it
        // cannot drive a per-group switch — replicate at table level).
        let thres = crate::sv8_context::context_threshold(2).unwrap();
        let mut idx = 2 * thres;
        let mut r_m = Sv7BitReader::new(&bytes);
        let mut out_m = [0_i8; SAMPLES_PER_BAND];
        for group in out_m.chunks_exact_mut(3) {
            let ctx = (idx > thres) as u8;
            let table = table_for_role(Sv8TableRole::Q2, ctx).unwrap();
            let symbol = table.decode(&mut r_m).unwrap();
            group.copy_from_slice(&unpack_grouped3_symbol(symbol).unwrap());
            idx = (idx >> 1) + crate::sv8_context::case2_magnitude(symbol as i32);
        }
        assert_eq!(out_g, out_m, "grounded grouped3 vs replicated accumulator");
    }

    #[test]
    fn grounded_grouped3_band_propagates_eof() {
        let mut reader = Sv7BitReader::new(&[0x00]);
        let mut out = [0_i8; SAMPLES_PER_BAND];
        assert!(matches!(
            decode_sv8_grouped3_band_grounded(&mut reader, &mut out),
            Err(Error::UnexpectedEof),
        ));
    }

    // ─── decode_sv8_escape_band (default arm, 9..=17) ──────

    #[test]
    fn q9up_symbol_map_is_full_signed_byte_permutation() {
        // The escape composition's keystone fact: the alphabet is
        // exactly the 256 signed-byte values, i.e. the VLC carries
        // the sign-bearing top 8 bits of the sample.
        assert_eq!(SV8_Q9UP_TABLE.symbols.len(), 256);
        let mut values: Vec<i8> = SV8_Q9UP_TABLE.symbols.to_vec();
        values.sort_unstable();
        let expected: Vec<i8> = (i8::MIN..=i8::MAX).collect();
        assert_eq!(values, expected);
    }

    #[test]
    fn escape_raw_bits_ladder_matches_res_bits_minus_vlc_width() {
        // raw = RES_BITS[band_type] - 8 = band_type - 9 across the
        // whole escape range; None everywhere else.
        for bt in 9..=17_i8 {
            assert_eq!(escape_raw_bits(bt), Some(bt as u8 - 9), "band_type {bt}");
            assert_eq!(
                escape_raw_bits(bt),
                Some(RES_BITS[bt as usize] - ESCAPE_VLC_SYMBOL_BITS),
                "band_type {bt}"
            );
        }
        for bt in [i8::MIN, -1, 0, 1, 5, 8, 18, 19, 64, i8::MAX] {
            assert_eq!(escape_raw_bits(bt), None, "band_type {bt}");
        }
    }

    #[test]
    fn escape_composition_tiles_the_dc_pinned_level_range() {
        // For every escape band_type, the composed value range
        // [-(D+1), D] must cover the Dc-pinned -D..=D, with the
        // maximum landing exactly on D = 2^(band_type-2) - 1.
        for bt in 9..=17_i8 {
            let n = escape_raw_bits(bt).unwrap() as u32;
            let d = QUANTIZER_OFFSET_D[(bt + 1) as usize] as i32;
            let max = (127_i32 << n) | ((1 << n) - 1);
            let min = (-128_i32) << n;
            assert_eq!(max, d, "band_type {bt}: composed max must equal D");
            assert_eq!(
                min,
                -(d + 1),
                "band_type {bt}: composed min is the two's-complement extremum"
            );
        }
    }

    #[test]
    fn escape_band_type_nine_is_pure_vlc_per_sample() {
        // n = 0: the codeword alone is the sample; no raw bits are
        // consumed.
        let table = &SV8_Q9UP_TABLE;
        let entry = table.lengths[0];
        let expected = symbol_for_row(table, 0) as i32;

        let mut p = BitPacker::new();
        for _ in 0..SAMPLES_PER_BAND {
            p.push(entry.code, entry.length);
        }
        let bytes = p.finish();
        let mut reader = Sv7BitReader::new(&bytes);
        let before = reader.bits_remaining();
        let mut out = [99_i32; SAMPLES_PER_BAND];
        decode_sv8_escape_band(&mut reader, 9, &mut out).expect("decode");
        assert!(out.iter().all(|&s| s == expected));
        assert_eq!(before - reader.bits_remaining(), 36 * entry.length as u64);
    }

    #[test]
    fn escape_band_composes_vlc_high_bits_with_raw_low_bits() {
        // band_type 13 → n = 4 raw bits. Alternate a negative- and
        // a positive-symbol codeword, each followed by a distinct
        // 4-bit raw pattern; every sample must equal
        // symbol * 16 + raw.
        let table = &SV8_Q9UP_TABLE;
        let (neg_pat, neg_len, neg_sym) =
            find_codeword(table, |s| s < 0).expect("q9up has a negative-symbol codeword");
        let (pos_pat, pos_len, pos_sym) =
            find_codeword(table, |s| s > 0).expect("q9up has a positive-symbol codeword");

        // Raw patterns chosen so an LSB-first misread would produce
        // a different value (0b1010 reversed is 0b0101).
        let raw_for = |i: usize| [0b1010_u16, 0b0001, 0b1111, 0b0000][i % 4];
        let mut p = BitPacker::new();
        for i in 0..SAMPLES_PER_BAND {
            if i % 2 == 0 {
                p.push(neg_pat, neg_len);
            } else {
                p.push(pos_pat, pos_len);
            }
            p.push(raw_for(i) << 12, 4); // left-justify the 4-bit raw field
        }
        let bytes = p.finish();
        let mut reader = Sv7BitReader::new(&bytes);
        let before = reader.bits_remaining();
        let mut out = [0_i32; SAMPLES_PER_BAND];
        decode_sv8_escape_band(&mut reader, 13, &mut out).expect("decode");
        for (i, &s) in out.iter().enumerate() {
            let sym = if i % 2 == 0 { neg_sym } else { pos_sym } as i32;
            let expected = (sym << 4) | raw_for(i) as i32;
            assert_eq!(s, expected, "sample {i}");
        }
        let codeword_bits = 18 * (neg_len as u64 + pos_len as u64);
        assert_eq!(
            before - reader.bits_remaining(),
            codeword_bits + 36 * 4,
            "every sample must consume its codeword plus exactly 4 raw bits"
        );
    }

    #[test]
    fn escape_band_widest_raw_field_reads_msb_first() {
        // band_type 17 → n = 8: a full raw byte per sample. Use one
        // fixed codeword and a per-sample raw byte equal to the
        // sample index; MSB-first composition means
        // sample = (symbol << 8) | i exactly.
        let table = &SV8_Q9UP_TABLE;
        let entry = table.lengths[0];
        let symbol = symbol_for_row(table, 0) as i32;

        let mut p = BitPacker::new();
        for i in 0..SAMPLES_PER_BAND {
            p.push(entry.code, entry.length);
            p.push((i as u16) << 8, 8);
        }
        let bytes = p.finish();
        let mut reader = Sv7BitReader::new(&bytes);
        let mut out = [0_i32; SAMPLES_PER_BAND];
        decode_sv8_escape_band(&mut reader, 17, &mut out).expect("decode");
        for (i, &s) in out.iter().enumerate() {
            assert_eq!(s, (symbol << 8) | i as i32, "sample {i}");
        }
    }

    #[test]
    fn escape_band_rejects_band_type_outside_9_17() {
        let mut out = [0_i32; SAMPLES_PER_BAND];
        for bt in [i8::MIN, -1, 0, 1, 2, 5, 8, 18, 19, 64, i8::MAX] {
            let mut reader = Sv7BitReader::new(&[0xFF; 128]);
            assert!(matches!(
                decode_sv8_escape_band(&mut reader, bt, &mut out),
                Err(Error::UnsupportedBandType(v)) if v == bt,
            ));
        }
    }

    #[test]
    fn escape_band_propagates_eof_at_codeword_and_inside_raw_field() {
        // Far too short for any codeword: EOF at the VLC peek.
        let mut reader = Sv7BitReader::new(&[0xFF]);
        let mut out = [0_i32; SAMPLES_PER_BAND];
        assert!(matches!(
            decode_sv8_escape_band(&mut reader, 17, &mut out),
            Err(Error::UnexpectedEof),
        ));
        // Exactly 16 bits of zeros: the all-zero peek matches the
        // last q9up row (code 0x0000, length 11), leaving 5 bits —
        // fewer than the 8-bit raw field → EOF inside read_bits.
        let mut reader = Sv7BitReader::new(&[0x00, 0x00]);
        let before = reader.bits_remaining();
        assert_eq!(before, 16);
        assert!(matches!(
            decode_sv8_escape_band(&mut reader, 17, &mut out),
            Err(Error::UnexpectedEof),
        ));
    }

    // ─── Sparse band (case 1, §6.4.1 + §6.5) ──────────────

    /// Reference binomial for the test-side encoder (independent of
    /// the impl's `const fn binomial`).
    fn comb(n: u32, k: u32) -> u32 {
        if k > n {
            return 0;
        }
        let k = k.min(n - k);
        let mut r: u64 = 1;
        for i in 0..k {
            r = r * (n - i) as u64 / (i + 1) as u64;
        }
        r as u32
    }

    /// Encode one §6.5 enumerative codeword for `coded` (a `k`-subset
    /// of `n` positions, as a low-`n`-bit mask) into `p`, mirroring
    /// the phased-binary + combinadic convention the decoder inverts.
    fn pack_enum_subset(p: &mut BitPacker, coded: u32, k: u32, n: u32) {
        let total = comb(n, k);
        if total <= 1 {
            return; // bitlen 0 → no bits
        }
        let mut bitlen: u8 = 0;
        while (1u32 << bitlen) < total {
            bitlen += 1;
        }
        let lost = (1u32 << bitlen) - total;
        // combinadic rank (same greedy high→low walk as the decoder)
        let mut rank: u32 = 0;
        let mut kk = k;
        let mut m = n;
        while m > 0 && kk > 0 {
            m -= 1;
            if coded & (1 << m) != 0 {
                rank += comb(m, kk);
                kk -= 1;
            }
        }
        if rank < lost {
            // short codeword: bitlen-1 bits
            p.push_raw(rank, bitlen - 1);
        } else {
            // long codeword: (rank + lost) in bitlen bits
            p.push_raw(rank + lost, bitlen);
        }
    }

    /// Encode one sparse group of 18 samples (`{-1,0,+1}`) into `p`,
    /// returning the `cnt` the caller must emit via the q1 table:
    /// the enumerative position codeword (omitted for cnt 0 / 18) plus
    /// one MSB-first sign bit per present position.
    fn pack_sparse_group(p: &mut BitPacker, samples: &[i8; 18]) -> u8 {
        let n = 18u32;
        let mut present: u32 = 0;
        for (i, &s) in samples.iter().enumerate() {
            if s != 0 {
                present |= 1 << i;
            }
        }
        let cnt = present.count_ones();
        if cnt != 0 && cnt != n {
            let (coded, k) = if cnt > n / 2 {
                ((!present) & ((1 << n) - 1), n - cnt)
            } else {
                (present, cnt)
            };
            pack_enum_subset(p, coded, k, n);
        }
        // sign bits MSB-first over present positions
        for pos in (0..18).rev() {
            if present & (1 << pos) != 0 {
                let s = samples[pos];
                p.push_raw(if s > 0 { 1 } else { 0 }, 1);
            }
        }
        cnt as u8
    }

    /// Emit a q1 codeword carrying the count `cnt` (the shortest code
    /// that maps to symbol == cnt), so a sparse-band test stream is
    /// self-contained.
    fn push_q1_count(p: &mut BitPacker, cnt: u8) {
        use crate::sv8_huffman::SV8_Q1_TABLE;
        let (pat, len, _sym) = find_codeword(&SV8_Q1_TABLE, |s| s == cnt as i8)
            .unwrap_or_else(|| panic!("q1 has a codeword for cnt {cnt}"));
        p.push(pat, len);
    }

    /// Count of non-zero samples in an 18-sample group.
    fn nonzero_count(samples: &[i8; 18]) -> u8 {
        samples.iter().filter(|&&s| s != 0).count() as u8
    }

    /// Build a complete `band_type == 1` stream from two 18-sample
    /// halves (q1 count + group payload, in stream order per half) and
    /// decode it back, asserting an exact round trip.
    fn sparse_roundtrip(first: [i8; 18], second: [i8; 18]) {
        let mut p = BitPacker::new();
        push_q1_count(&mut p, nonzero_count(&first));
        pack_sparse_group(&mut p, &first);
        push_q1_count(&mut p, nonzero_count(&second));
        pack_sparse_group(&mut p, &second);
        let bytes = p.finish();

        let mut reader = Sv7BitReader::new(&bytes);
        let mut out = [99_i8; SAMPLES_PER_BAND];
        decode_sv8_sparse_band(&mut reader, &mut out).expect("sparse decode");
        let mut expected = [0_i8; SAMPLES_PER_BAND];
        expected[..18].copy_from_slice(&first);
        expected[18..].copy_from_slice(&second);
        assert_eq!(out, expected);
    }

    #[test]
    fn sparse_all_zero_band_reads_only_two_q1_counts() {
        // Both halves cnt 0: no enumerative or sign bits, just two q1
        // count codewords; every sample is 0.
        use crate::sv8_huffman::SV8_Q1_TABLE;
        let (pat, len, _) = find_codeword(&SV8_Q1_TABLE, |s| s == 0).unwrap();
        let mut p = BitPacker::new();
        p.push(pat, len);
        p.push(pat, len);
        let bytes = p.finish();
        let mut reader = Sv7BitReader::new(&bytes);
        let before = reader.bits_remaining();
        let mut out = [7_i8; SAMPLES_PER_BAND];
        decode_sv8_sparse_band(&mut reader, &mut out).expect("decode");
        assert_eq!(out, [0; SAMPLES_PER_BAND]);
        assert_eq!(before - reader.bits_remaining(), 2 * len as u64);
    }

    #[test]
    fn sparse_single_nonzero_per_half_roundtrips() {
        // cnt 1 in each half: enumerative code names one of 18
        // positions, one sign bit each.
        let mut a = [0_i8; 18];
        a[5] = 1;
        let mut b = [0_i8; 18];
        b[12] = -1;
        sparse_roundtrip(a, b);
    }

    #[test]
    fn sparse_complement_inversion_above_half_roundtrips() {
        // cnt 14 (> 9): the coder names the 4-position complement and
        // the decoder bit-inverts. Mixed signs.
        let mut a = [0_i8; 18];
        for (i, s) in a.iter_mut().enumerate() {
            *s = if matches!(i, 2 | 7 | 11 | 15) {
                0
            } else if i % 2 == 0 {
                1
            } else {
                -1
            };
        }
        let b = [1_i8; 18]; // cnt 18: all present, no enumerative bits
        sparse_roundtrip(a, b);
    }

    #[test]
    fn sparse_every_count_roundtrips_with_distinct_positions() {
        // Sweep cnt 0..=18 in the first half (lowest `cnt` positions
        // set, signs alternating) paired with an all-zero second half.
        for cnt in 0..=18usize {
            let mut a = [0_i8; 18];
            for (i, s) in a.iter_mut().enumerate().take(cnt) {
                *s = if i % 2 == 0 { 1 } else { -1 };
            }
            sparse_roundtrip(a, [0; 18]);
        }
    }

    #[test]
    fn sparse_full_band_all_present_both_halves() {
        // cnt 18 in both halves: no enumerative bits, 18 sign bits per
        // half; the decoder must read exactly 2 q1 + 36 sign bits.
        let a = [1_i8; 18];
        let mut b = [0_i8; 18];
        for (i, s) in b.iter_mut().enumerate() {
            *s = if i % 3 == 0 { -1 } else { 1 };
        }
        sparse_roundtrip(a, b);
    }

    #[test]
    fn sparse_rejects_malformed_count_above_18() {
        // decode_sparse_group is internal; exercise its bound directly
        // through a hand value via decode_sv8_sparse_band is not
        // possible (q1 caps at 18), so test the group helper's guard.
        let mut reader = Sv7BitReader::new(&[0xFF; 8]);
        let mut group = [0_i8; SPARSE_GROUP_SIZE];
        assert!(matches!(
            decode_sparse_group(&mut reader, 19, &mut group),
            Err(Error::GroupedSymbolOutOfRange(19)),
        ));
    }

    #[test]
    fn sparse_propagates_eof() {
        // A single q1 count for a non-empty half, then the stream ends
        // before the sign bit can be read.
        use crate::sv8_huffman::SV8_Q1_TABLE;
        let (pat, len, _) = find_codeword(&SV8_Q1_TABLE, |s| s == 1).unwrap();
        let mut p = BitPacker::new();
        p.push(pat, len);
        // cnt 1 needs an enumerative codeword (5 bits) + 1 sign bit;
        // truncate so the decode starves.
        let mut bytes = p.finish();
        bytes.truncate(1);
        let mut reader = Sv7BitReader::new(&bytes);
        let mut out = [0_i8; SAMPLES_PER_BAND];
        assert!(matches!(
            decode_sv8_sparse_band(&mut reader, &mut out),
            Err(Error::UnexpectedEof),
        ));
    }

    #[test]
    fn enum_bitlen_lost_matches_binomial_code_space() {
        // (bitlen, lost) must satisfy 2^bitlen - lost == C(18,k).
        for k in 0..=18u32 {
            let total = binomial(18, k);
            let (bitlen, lost) = enum_bitlen_lost(total);
            if total <= 1 {
                assert_eq!((bitlen, lost), (0, 0), "k {k}");
            } else {
                assert_eq!((1u32 << bitlen) - lost, total, "k {k}");
                // bitlen is the ceil(log2): 2^(bitlen-1) < total.
                assert!((1u32 << (bitlen - 1)) < total, "k {k}");
            }
        }
    }

    #[test]
    fn binomial_recurrence_matches_reference() {
        for n in 0..=18u32 {
            for k in 0..=n {
                assert_eq!(binomial(n, k), comb(n, k), "C({n},{k})");
            }
        }
        assert_eq!(binomial(18, 9), 48620);
        assert_eq!(binomial(5, 3), 10);
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
        // The escape arm is implemented for the requant-defined
        // 9..=17 range of the classifier's `>= 9` default arm.
        for bt in 9..=17 {
            assert_eq!(sv8_band_type_case(bt), Sv8BandDecodeCase::LargeCoeffEscape);
            assert!(escape_raw_bits(bt).is_some());
        }
        // Case 1 (sparse band) has its own dedicated decoder
        // (decode_sv8_sparse_band); the grouped/context/escape
        // decoders still reject band_type 1 (it is not their arm).
        assert_eq!(sv8_band_type_case(1), Sv8BandDecodeCase::SparseBand);
        let mut out = [0_i8; SAMPLES_PER_BAND];
        let mut out_i32 = [0_i32; SAMPLES_PER_BAND];
        let mut reader = Sv7BitReader::new(&[0xFF; 64]);
        assert!(decode_sv8_grouped2_band(&mut reader, 1, &mut out).is_err());
        let mut reader = Sv7BitReader::new(&[0xFF; 64]);
        assert!(decode_sv8_context_band(&mut reader, 1, 0, |_| 0, &mut out).is_err());
        let mut reader = Sv7BitReader::new(&[0xFF; 64]);
        assert!(decode_sv8_escape_band(&mut reader, 1, &mut out_i32).is_err());
    }
}
