//! SV7 frame-body band-type header loop (§2.3).
//!
//! Wires the structurally-specified per-band header loop documented in
//! `docs/audio/musepack/musepack-sv7-sv8-spec.md` §2.3 ("Frame body —
//! band types") on top of the round-197 `SV7_BANDTYPE_HEADER_TABLE`
//! Huffman table and the `Sv7BitReader` MSB-first bit-stream reader.
//!
//! Source-of-record:
//!
//! - **Structural prose**: `docs/audio/musepack/musepack-sv7-sv8-spec.md`
//!   §2.3, the per-band iteration block reproduced here for traceability:
//!
//!   ```text
//!   # max_band comes from the stream header (§2.1)
//!   for (i = 0; i <= max_band; i++) {
//!       for (ch = 0; ch < nch; ch++)
//!           band_type[i][ch] = get_vlc();        # VLC table = sv7-huffman-bandtype-header
//!       if (band_type[i][0] != 0 || band_type[i][1] != 0)
//!           msflag[i] = get_bit();               # M/S vs L/R for this band
//!   }
//!   ```
//!
//! Three structural facts pinned by the §2.3 prose drive this module:
//!
//! 1. **Iteration order**. The outer index `i` runs `0..=max_band`
//!    (inclusive). The inner index `ch` runs `0..nch`.
//! 2. **Channel interleaving**. Within band `i`, the left channel's
//!    `band_type` VLC is read before the right channel's. This is the
//!    "interleaved by channel: left band first, right band next"
//!    sentence of §2.3 line 134-135.
//! 3. **Conditional M/S flag**. A single raw bit follows the channel-
//!    pair VLCs **iff** at least one of the two `band_type` values for
//!    that band is non-zero. If both channels report band_type == 0
//!    the bit is **absent** — the band carries no samples in either
//!    channel, so no stereo-mode decision is necessary. The mono
//!    (`nch == 1`) case is treated as "both channels are the single
//!    channel"; the M/S flag is then conceptually meaningless and is
//!    omitted whenever the single channel's band_type is 0, present
//!    otherwise. The structural prose covers stereo explicitly; the
//!    mono extension is the only sensible reading of "if either
//!    channel is non-zero" when there is one channel.
//!
//! ## §5.1 closes the `RawBandTypeVlc → band_type` remap GAP
//!
//! The structural §2.3 walker ([`decode_band_header`] /
//! [`decode_header_loop`]) reads the header VLC as an **opaque symbol**
//! and wraps it in [`RawBandTypeVlc`] because §2.3 alone did not pin how
//! that symbol maps onto the §2.5 `band_type` switch. The staged
//! `spec/musepack-headers-and-coding.md` **§5.1** now closes that GAP:
//! the header VLC codes a **per-channel delta chain** that produces the
//! §5.4 `Res` (= band_type) directly — band 0 is a raw 4-bit absolute,
//! later bands delta off the same channel's previous `Res` with a
//! `idx == 4` raw-4-bit escape, and the per-band M/S bit is read only
//! when the stream-wide M/S flag is set. [`decode_res_header_grounded`]
//! implements this and returns the [`Sv7ResBand`] sequence whose `res`
//! values feed the §5.4 sample switch with no further remap. The
//! opaque [`RawBandTypeVlc`] path is retained for callers that want the
//! raw symbol; the grounded path is the §5.1 closure.
//!
//! What this module does **not** do (out-of-scope this round):
//!
//! - It does not consume the per-frame 20-bit length prefix or the
//!   "read in 32-LSB units" word packing of §2.2 — those belong to
//!   the frame-driver round and are still GAP.
//! - It does not source `max_band` or `nch`; both are caller-supplied
//!   parameters. The header field map that would source them is GAP
//!   per §2.1.

use crate::huffman::{decode as huffman_decode, Sv7BitReader, SV7_BANDTYPE_HEADER_TABLE};
use crate::{Error, Result};

/// Layer-II-inherited subband count: a Musepack frame's polyphase
/// filterbank produces 32 subbands per channel (§1 line 53-71). The
/// `max_band` field gates how many of those subbands the §2.3 loop
/// actually walks; the value itself is bounded by this constant.
pub const SV7_SUBBAND_COUNT: usize = 32;

/// Inclusive upper bound for the §2.3 loop's `max_band` parameter.
/// With `SV7_SUBBAND_COUNT == 32` subbands indexed `0..32`, the
/// largest valid `max_band` is `31`. A caller-supplied `max_band`
/// above this is rejected with [`Error::MaxBandOutOfRange`].
pub const SV7_MAX_BAND_INCLUSIVE: u8 = (SV7_SUBBAND_COUNT - 1) as u8;

/// Structurally-valid channel counts: 1 (mono) or 2 (stereo). The
/// Layer-II / Musepack frame geometry does not allow other values
/// at the §2.3 layer (the spec's "level 3 = 8 channels" upgrade is
/// SV8-specific and lives in the SH packet field map, which is GAP).
const fn channels_valid(nch: u8) -> bool {
    nch == 1 || nch == 2
}

/// Typed wrapper around the raw `i8` value produced by a single
/// invocation of the `sv7-huffman-bandtype-header` VLC.
///
/// The wrapper exists to keep the `band_type → §2.5 case` mapping
/// honest: the staged bandtype-header VLC's symbol alphabet is
/// `-5..=4` and does **not** cover the full §2.5 dispatcher domain
/// (`-1..=17`). An upstream remap is implied by the structural prose
/// (the §2.5 `switch (band_type)` reads a `band_type` that ranges up
/// to 17) but the shape of that remap is DOCS-GAP. Callers must apply
/// the remap explicitly before feeding into [`crate::sv7_band_decode`];
/// `RawBandTypeVlc::as_i8` exposes the underlying value but the
/// distinct type prevents accidental composition with the §2.5
/// dispatchers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RawBandTypeVlc(i8);

impl RawBandTypeVlc {
    /// Wrap a raw `i8` VLC symbol. No validity check beyond the
    /// `i8` range — the table guarantees values in `-5..=4`, but
    /// nothing in §2.3 elevates that to a hard invariant.
    pub const fn from_raw(value: i8) -> Self {
        Self(value)
    }

    /// Expose the underlying `i8` value. This is the value §2.3 uses
    /// in its `band_type[i][ch] != 0` predicate and the raw input to
    /// the upstream-pending `band_type` remap.
    pub const fn as_i8(self) -> i8 {
        self.0
    }

    /// True iff the value is structurally non-zero, i.e. the band is
    /// not the silent / not-coded case 0 in this channel.
    pub const fn is_nonzero(self) -> bool {
        self.0 != 0
    }
}

/// One decoded entry of the §2.3 per-band header loop.
///
/// Per the spec: a per-channel `band_type` VLC pair, plus an optional
/// `msflag` raw bit that is present iff at least one channel's
/// `band_type` is non-zero. The mono case is represented with the
/// same channel slot used twice and the same conditional applied to
/// the single value (see module-level docs).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BandHeader {
    /// Per-channel raw bandtype-header VLC value. `band_type[0]` is
    /// the left channel; `band_type[1]` is the right channel. In the
    /// mono case both slots carry the same value (the one channel's
    /// VLC). The §2.5 dispatcher domain (and the §2.3 `!= 0`
    /// predicate) reads this value directly via
    /// [`RawBandTypeVlc::as_i8`] / [`RawBandTypeVlc::is_nonzero`].
    pub band_type: [RawBandTypeVlc; 2],
    /// `Some(true)` for "mid-side coded this band", `Some(false)`
    /// for "left-right coded this band", `None` if §2.3's conditional
    /// suppressed the flag (both channels report band_type == 0, so
    /// no stereo-mode decision is encoded).
    pub ms_flag: Option<bool>,
}

impl BandHeader {
    /// True iff at least one channel of this band carries samples
    /// (i.e. the §2.3 conditional that gates the `msflag` bit
    /// triggered). The §2.5 per-band sample decode runs only over
    /// bands whose `has_samples()` is true; bands with all-zero
    /// `band_type` are implicit zero-fills.
    pub const fn has_samples(&self) -> bool {
        self.band_type[0].is_nonzero() || self.band_type[1].is_nonzero()
    }
}

/// Decode one band's §2.3 header entry given an open `Sv7BitReader`.
///
/// Reads `nch` `band_type` VLCs from `SV7_BANDTYPE_HEADER_TABLE` in
/// channel order (left first, then right for stereo), then — iff at
/// least one of the read values is non-zero — reads one raw bit for
/// the M/S flag (`true` = mid-side, `false` = left-right). The mono
/// case (`nch == 1`) treats the single channel's VLC as occupying
/// both `band_type[0]` and `band_type[1]` slots of the returned
/// [`BandHeader`], so [`BandHeader::has_samples`] and downstream
/// per-band-decode loops compose the same way whether the input is
/// mono or stereo.
///
/// Errors:
/// - [`Error::ChannelCountInvalid`] if `nch` is neither 1 nor 2.
/// - [`Error::UnexpectedEof`] if the underlying reader runs out
///   during either the VLC walk or the `msflag` single bit.
/// - [`Error::HuffmanNoMatch`] if any of the `band_type` VLCs fails
///   to match a row of `SV7_BANDTYPE_HEADER_TABLE`.
pub fn decode_band_header(reader: &mut Sv7BitReader<'_>, nch: u8) -> Result<BandHeader> {
    if !channels_valid(nch) {
        return Err(Error::ChannelCountInvalid(nch));
    }
    let left = RawBandTypeVlc::from_raw(huffman_decode(reader, &SV7_BANDTYPE_HEADER_TABLE)?);
    let right = if nch == 2 {
        RawBandTypeVlc::from_raw(huffman_decode(reader, &SV7_BANDTYPE_HEADER_TABLE)?)
    } else {
        left
    };
    let any_nonzero = left.is_nonzero() || right.is_nonzero();
    let ms_flag = if any_nonzero {
        let bit = reader.read_bits(1)?;
        Some(bit & 1 == 1)
    } else {
        None
    };
    Ok(BandHeader {
        band_type: [left, right],
        ms_flag,
    })
}

/// Decode the full §2.3 band-type header loop, walking the outer
/// index `i = 0..=max_band` and accumulating one [`BandHeader`] per
/// band.
///
/// `max_band` is inclusive per the structural prose (`for (i = 0;
/// i <= max_band; i++)`) and must satisfy
/// `max_band <= SV7_MAX_BAND_INCLUSIVE`. The returned vector has
/// length `max_band as usize + 1`.
///
/// Errors:
/// - [`Error::ChannelCountInvalid`] if `nch` is neither 1 nor 2.
/// - [`Error::MaxBandOutOfRange`] if `max_band > 31`.
/// - [`Error::UnexpectedEof`] / [`Error::HuffmanNoMatch`] propagated
///   from [`decode_band_header`] mid-loop.
pub fn decode_header_loop(
    reader: &mut Sv7BitReader<'_>,
    max_band: u8,
    nch: u8,
) -> Result<Vec<BandHeader>> {
    if !channels_valid(nch) {
        return Err(Error::ChannelCountInvalid(nch));
    }
    if max_band > SV7_MAX_BAND_INCLUSIVE {
        return Err(Error::MaxBandOutOfRange(max_band));
    }
    let band_count = max_band as usize + 1;
    let mut out = Vec::with_capacity(band_count);
    for _ in 0..band_count {
        out.push(decode_band_header(reader, nch)?);
    }
    Ok(out)
}

// ─── §5.1 grounded Res (band-type) header decode ──────────────────────
//
// The staged `spec/musepack-headers-and-coding.md` §5.1 closes the
// `RawBandTypeVlc → band_type` remap GAP the structural §2.3 walker
// above leaves open. §5.1 pins the decode as a **delta chain per
// channel** that produces the §2.5/§5.4 `band_type` (`Res`) directly:
//
// - **Band 0:** each channel's `Res` is a raw **4-bit** value.
// - **Bands 1..max_band:** each channel's `Res` deltas off the *same
//   channel's previous band* `Res` via the header VLC
//   (`sv7-huffman-bandtype-header`): `Res[n] = Res[n-1] + idx`, except
//   when the VLC symbol `idx == 4`, an **escape** meaning "read a raw
//   4-bit absolute `Res` instead."
// - **Per-band M/S:** for each band where either channel's `Res != 0`,
//   and only if the stream-wide M/S flag is set, a 1-bit per-band M/S
//   flag follows.
//
// So §5.1's `Res` *is* the band_type the §5.4 sample switch consumes
// directly — no further remap. This is distinct from the structural
// `decode_band_header` / `decode_header_loop` above, which read the VLC
// as an opaque symbol with no band-0 raw, no escape, and an
// unconditional (stream-flag-independent) msflag.

/// The §5.1 band-type-header VLC **escape** symbol: a decoded value of
/// `4` means "read a raw 4-bit absolute `Res` instead of a delta." The
/// staged `sv7-huffman-bandtype-header` table carries the literal `4`.
pub const RES_HEADER_ESCAPE_SYMBOL: i8 = 4;

/// Width, in bits, of a raw absolute `Res` — both the band-0 read and
/// the [`RES_HEADER_ESCAPE_SYMBOL`] escape read (§5.1: "a raw 4-bit
/// value" / "a raw 4-bit absolute `Res`").
pub const RES_RAW_BITS: u8 = 4;

/// One band's grounded §5.1 per-channel `Res` (band_type) plus the
/// optional per-band M/S flag.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Sv7ResBand {
    /// Per-channel `Res` (band_type) directly usable by the §5.4 sample
    /// switch. `res[0]` is the left channel, `res[1]` the right; in mono
    /// both slots carry the single channel's value.
    pub res: [i8; 2],
    /// `Some(true)` mid-side, `Some(false)` left-right, `None` when the
    /// per-band M/S flag was suppressed (stream M/S off, or both
    /// channels' `Res == 0`).
    pub ms_flag: Option<bool>,
}

impl Sv7ResBand {
    /// True iff at least one channel carries samples (`Res != 0`).
    pub const fn has_samples(&self) -> bool {
        self.res[0] != 0 || self.res[1] != 0
    }
}

/// Read one channel's §5.1 `Res`: for band 0, a raw [`RES_RAW_BITS`]-bit
/// absolute; for later bands, a header-VLC delta off `prev` with the
/// [`RES_HEADER_ESCAPE_SYMBOL`] raw-absolute escape.
fn read_res_channel(reader: &mut Sv7BitReader<'_>, band_index: usize, prev: i8) -> Result<i8> {
    if band_index == 0 {
        return Ok(reader.read_bits(RES_RAW_BITS)? as i8);
    }
    let sym = huffman_decode(reader, &SV7_BANDTYPE_HEADER_TABLE)?;
    if sym == RES_HEADER_ESCAPE_SYMBOL {
        Ok(reader.read_bits(RES_RAW_BITS)? as i8)
    } else {
        Ok(prev.wrapping_add(sym))
    }
}

/// Decode the full §5.1 grounded `Res` (band-type) header over
/// `0..=max_band`, producing each band's per-channel `Res` and optional
/// per-band M/S flag.
///
/// `nch` is `1` (mono) or `2` (stereo). `stream_ms` is the stream-wide
/// M/S flag (from the SH packet / SV7 fixed header): the per-band M/S bit
/// is read only when `stream_ms` is set *and* the band has a non-zero
/// channel. In mono (`nch == 1`) both `res` slots carry the single
/// channel's value and no per-band M/S bit is read (the flag is
/// meaningless with one channel — `stream_ms` would not be set for a mono
/// stream).
///
/// The returned `Vec` has length `max_band as usize + 1`, in ascending
/// band order. Each band's `Res` is the §5.4 sample-switch `band_type`
/// directly — no further remap.
///
/// # Errors
///
/// - [`Error::ChannelCountInvalid`] if `nch` is neither 1 nor 2.
/// - [`Error::MaxBandOutOfRange`] if `max_band > SV7_MAX_BAND_INCLUSIVE`.
/// - [`Error::UnexpectedEof`] / [`Error::HuffmanNoMatch`] from a raw
///   read or VLC mid-loop.
pub fn decode_res_header_grounded(
    reader: &mut Sv7BitReader<'_>,
    max_band: u8,
    nch: u8,
    stream_ms: bool,
) -> Result<Vec<Sv7ResBand>> {
    if !channels_valid(nch) {
        return Err(Error::ChannelCountInvalid(nch));
    }
    if max_band > SV7_MAX_BAND_INCLUSIVE {
        return Err(Error::MaxBandOutOfRange(max_band));
    }
    let band_count = max_band as usize + 1;
    let mut out = Vec::with_capacity(band_count);
    // Per-channel running previous Res (band 0 ignores it).
    let mut prev = [0_i8; 2];
    for i in 0..band_count {
        let left = read_res_channel(reader, i, prev[0])?;
        let right = if nch == 2 {
            read_res_channel(reader, i, prev[1])?
        } else {
            left
        };
        prev = [left, right];
        let band = Sv7ResBand {
            res: [left, right],
            ms_flag: None,
        };
        // §5.1: per-band M/S bit only when stream M/S is set, the band
        // has a non-zero channel, and the stream is stereo.
        let ms_flag = if stream_ms && nch == 2 && band.has_samples() {
            Some(reader.read_bits(1)? & 1 == 1)
        } else {
            None
        };
        out.push(Sv7ResBand { ms_flag, ..band });
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Hand-traced bandtype-header VLC bit strings derived directly
    /// from the staged CSV's left-justified `Code` column (high bits
    /// of `code`, taking the first `length` bits MSB-first):
    ///
    /// - value `0`:  `0x8000`, length 1  → bits `1`
    /// - value `1`:  `0x6000`, length 3  → bits `011`
    /// - value `-1`: `0x0000`, length 2  → bits `00`
    /// - value `-2`: `0x4000`, length 4  → bits `0100`
    /// - value `-3`: `0x5000`, length 5  → bits `01010`
    /// - value `2`:  `0x5800`, length 6  → bits `010110`
    ///
    /// These six are enough to exercise the iteration / conditional
    /// logic without re-hand-coding every row of the table.
    fn bits_value_zero() -> &'static [bool] {
        &[true]
    }
    fn bits_value_one() -> &'static [bool] {
        &[false, true, true]
    }
    fn bits_value_neg_one() -> &'static [bool] {
        &[false, false]
    }
    fn bits_value_neg_two() -> &'static [bool] {
        &[false, true, false, false]
    }

    /// Pack a slice of bits MSB-first into a byte vector, padded with
    /// trailing zero bits to the next byte boundary plus an extra
    /// `peek16` worth of trailing zero bytes so the reader can satisfy
    /// its always-16-bit peek on the *last* VLC of a test input
    /// without running off the end. Real decoders see this padding
    /// for free because the frame body is followed by either another
    /// frame or the stream trailer; tests have to add it explicitly.
    fn pack(bits: &[bool]) -> Vec<u8> {
        let mut bytes = vec![0u8; bits.len().div_ceil(8)];
        for (i, &b) in bits.iter().enumerate() {
            if b {
                bytes[i / 8] |= 1 << (7 - (i % 8));
            }
        }
        // peek16 demands 16 bits of look-ahead; pad two extra zero
        // bytes so the inner peek can always succeed even right at
        // the end of the test's logical bit sequence.
        bytes.push(0);
        bytes.push(0);
        bytes
    }

    fn concat(parts: &[&[bool]]) -> Vec<bool> {
        let mut out = Vec::new();
        for p in parts {
            out.extend_from_slice(p);
        }
        out
    }

    #[test]
    fn subband_geometry_matches_layer_ii_heritage() {
        // §1 lines 53-71: 32 polyphase subbands. The inclusive
        // max_band is one less.
        assert_eq!(SV7_SUBBAND_COUNT, 32);
        assert_eq!(SV7_MAX_BAND_INCLUSIVE, 31);
    }

    #[test]
    fn raw_band_type_vlc_round_trips_through_from_raw() {
        for raw in -5_i8..=4 {
            let w = RawBandTypeVlc::from_raw(raw);
            assert_eq!(w.as_i8(), raw);
            assert_eq!(w.is_nonzero(), raw != 0);
        }
    }

    #[test]
    fn raw_band_type_vlc_is_nonzero_only_for_nonzero_values() {
        assert!(!RawBandTypeVlc::from_raw(0).is_nonzero());
        assert!(RawBandTypeVlc::from_raw(1).is_nonzero());
        assert!(RawBandTypeVlc::from_raw(-1).is_nonzero());
        assert!(RawBandTypeVlc::from_raw(-5).is_nonzero());
        assert!(RawBandTypeVlc::from_raw(4).is_nonzero());
    }

    #[test]
    fn band_header_has_samples_matches_or_of_channels() {
        let zero = RawBandTypeVlc::from_raw(0);
        let one = RawBandTypeVlc::from_raw(1);
        let neg_two = RawBandTypeVlc::from_raw(-2);

        let bh = BandHeader {
            band_type: [zero, zero],
            ms_flag: None,
        };
        assert!(!bh.has_samples());

        let bh = BandHeader {
            band_type: [one, zero],
            ms_flag: Some(false),
        };
        assert!(bh.has_samples());

        let bh = BandHeader {
            band_type: [zero, neg_two],
            ms_flag: Some(true),
        };
        assert!(bh.has_samples());

        let bh = BandHeader {
            band_type: [one, neg_two],
            ms_flag: Some(true),
        };
        assert!(bh.has_samples());
    }

    #[test]
    fn decode_band_header_stereo_both_zero_omits_msflag() {
        // Two VLCs of value 0 (each one '1' bit) → no msflag bit.
        let bits = concat(&[bits_value_zero(), bits_value_zero()]);
        let bytes = pack(&bits);
        let mut r = Sv7BitReader::new(&bytes);
        let bh = decode_band_header(&mut r, 2).unwrap();
        assert_eq!(bh.band_type[0].as_i8(), 0);
        assert_eq!(bh.band_type[1].as_i8(), 0);
        assert_eq!(bh.ms_flag, None);
        assert!(!bh.has_samples());
    }

    #[test]
    fn decode_band_header_stereo_left_nonzero_reads_msflag_one() {
        // Left VLC value 1 (`011`), right VLC value 0 (`1`), then
        // one msflag bit `1` (M/S).
        let bits = concat(&[bits_value_one(), bits_value_zero(), &[true]]);
        let bytes = pack(&bits);
        let mut r = Sv7BitReader::new(&bytes);
        let bh = decode_band_header(&mut r, 2).unwrap();
        assert_eq!(bh.band_type[0].as_i8(), 1);
        assert_eq!(bh.band_type[1].as_i8(), 0);
        assert_eq!(bh.ms_flag, Some(true));
        assert!(bh.has_samples());
    }

    #[test]
    fn decode_band_header_stereo_right_nonzero_reads_msflag_zero() {
        // Left VLC value 0 (`1`), right VLC value -1 (`00`), then
        // one msflag bit `0` (L/R).
        let bits = concat(&[bits_value_zero(), bits_value_neg_one(), &[false]]);
        let bytes = pack(&bits);
        let mut r = Sv7BitReader::new(&bytes);
        let bh = decode_band_header(&mut r, 2).unwrap();
        assert_eq!(bh.band_type[0].as_i8(), 0);
        assert_eq!(bh.band_type[1].as_i8(), -1);
        assert_eq!(bh.ms_flag, Some(false));
    }

    #[test]
    fn decode_band_header_mono_zero_omits_msflag() {
        // Single VLC value 0 → no msflag bit.
        let bits = concat(&[bits_value_zero()]);
        let bytes = pack(&bits);
        let mut r = Sv7BitReader::new(&bytes);
        let bh = decode_band_header(&mut r, 1).unwrap();
        assert_eq!(bh.band_type[0].as_i8(), 0);
        assert_eq!(bh.band_type[1].as_i8(), 0);
        assert_eq!(bh.ms_flag, None);
    }

    #[test]
    fn decode_band_header_mono_nonzero_reads_msflag_present() {
        // Single VLC value -2 → both slots = -2 → has_samples → 1
        // msflag bit is consumed (here `1`). The flag has no
        // physical meaning in mono but the §2.3 conditional fires
        // identically, so the bit is structurally present.
        let bits = concat(&[bits_value_neg_two(), &[true]]);
        let bytes = pack(&bits);
        let mut r = Sv7BitReader::new(&bytes);
        let bh = decode_band_header(&mut r, 1).unwrap();
        assert_eq!(bh.band_type[0].as_i8(), -2);
        assert_eq!(bh.band_type[1].as_i8(), -2);
        assert_eq!(bh.ms_flag, Some(true));
    }

    #[test]
    fn decode_band_header_rejects_invalid_nch() {
        let bytes = [0xFF, 0xFF, 0xFF, 0xFF];
        for nch in [0u8, 3, 8, 255] {
            let mut r = Sv7BitReader::new(&bytes);
            assert_eq!(
                decode_band_header(&mut r, nch),
                Err(Error::ChannelCountInvalid(nch))
            );
        }
    }

    #[test]
    fn decode_band_header_propagates_unexpected_eof_in_left_vlc() {
        // peek16 needs 16 bits; supply only one.
        let bytes = [0x80];
        let mut r = Sv7BitReader::new(&bytes);
        assert_eq!(decode_band_header(&mut r, 2), Err(Error::UnexpectedEof));
    }

    /// Pack bits exactly (no peek16 padding) — used by the EOF
    /// tests below to keep the input lean enough that peek16 / the
    /// msflag bit actually run off the end.
    fn pack_exact(bits: &[bool]) -> Vec<u8> {
        let mut bytes = vec![0u8; bits.len().div_ceil(8)];
        for (i, &b) in bits.iter().enumerate() {
            if b {
                bytes[i / 8] |= 1 << (7 - (i % 8));
            }
        }
        bytes
    }

    #[test]
    fn decode_band_header_propagates_unexpected_eof_in_msflag() {
        // Left value 1 (3 bits), right value 0 (1 bit), needs one
        // msflag bit but the buffer only ever holds the 16 bits
        // peek16 read for the VLC walks. We craft a tighter buffer:
        // two bytes that decode the VLCs cleanly, then no further
        // bits — but peek16's pre-fetch makes "no further bits"
        // tricky. Use a 3-byte buffer that has just enough for
        // peek16 to succeed twice but runs out before the msflag
        // single bit is reachable. The reader buffers 8 bytes lazily
        // so we use the minimum 2-byte input.
        //
        // Strategy: 16-bit input that resolves to (value 1, value 0)
        // and leaves 12 bits consumed total. peek16 was satisfied
        // because 16 bits were present. After consuming 4 bits, 12
        // remain. read_bits(1) succeeds. That doesn't trigger EOF —
        // we need a tighter input.
        //
        // Use a 2-byte input where after the two VLCs only 0 bits
        // remain. value 1 = 3 bits, value -1 = 2 bits → 5 bits used,
        // 11 left. Still doesn't EOF.
        //
        // Force a 2-byte input that exactly fits the two VLCs by
        // using the longest-code symbols: value 3 = 9 bits and value
        // 4 = 9 bits would need 18 bits = a 3-byte input, but then
        // there's spare. The cleanest way is to wrap the reader so
        // it has *exactly* 16 bits available, decode two short VLCs
        // that consume some of them, then the msflag has bits. So
        // EOF on msflag isn't reachable from a 2-byte input.
        //
        // The §2.3 EOF surface is therefore primarily in the VLC
        // phase; the msflag-EOF case is reachable only at the very
        // end of a frame that's been padded down to the exact byte
        // boundary. We test it by feeding a 2-byte input that
        // produces two VLCs that together consume all 16 bits
        // *plus* one trailing bit (impossible with one msflag, so
        // we cap with a longer-VLC value pair).
        //
        // The longest pair available is value 3 (9 bits) + value 4
        // (9 bits) = 18 bits. With only 2 bytes (16 bits), the
        // second VLC's peek16 already EOFs before we reach the
        // msflag — same as the left-VLC EOF case above. Pad to 3
        // bytes to satisfy the inner peek16s, then the msflag's
        // read_bits(1) is the first thing to EOF.
        let value_3 = vec![false, true, false, true, true, true, false, true, true]; // 0x5d80 / 9
        let value_4 = vec![false, true, false, true, true, true, false, true, false]; // 0x5d00 / 9
        let bits = concat(&[&value_3, &value_4]);
        let bytes = pack_exact(&bits);
        // Exactly 18 bits → 3 bytes (with 6 bits of zero padding
        // inside the last byte). The reader fetches lazily from
        // the slice and cannot read past its end.
        let mut r = Sv7BitReader::new(&bytes);
        // The first peek16 has 24 bits → OK.
        // The second peek16 needs 16 bits but only 15 remain → EOF.
        // So this still EOFs in the right VLC phase, not msflag.
        // Accept either EOF flavour; the contract is that out-of-
        // bits propagates.
        let err = decode_band_header(&mut r, 2).unwrap_err();
        assert_eq!(err, Error::UnexpectedEof);
    }

    #[test]
    fn decode_header_loop_rejects_max_band_above_thirtyone() {
        let bytes = [0u8; 16];
        let mut r = Sv7BitReader::new(&bytes);
        assert_eq!(
            decode_header_loop(&mut r, 32, 2),
            Err(Error::MaxBandOutOfRange(32))
        );
        assert_eq!(
            decode_header_loop(&mut r, 200, 2),
            Err(Error::MaxBandOutOfRange(200))
        );
    }

    #[test]
    fn decode_header_loop_rejects_invalid_nch() {
        let bytes = [0u8; 16];
        let mut r = Sv7BitReader::new(&bytes);
        assert_eq!(
            decode_header_loop(&mut r, 0, 0),
            Err(Error::ChannelCountInvalid(0))
        );
        assert_eq!(
            decode_header_loop(&mut r, 5, 3),
            Err(Error::ChannelCountInvalid(3))
        );
    }

    #[test]
    fn decode_header_loop_max_band_zero_returns_one_band() {
        // Just one band, both channels value 0 → no msflag, 2 VLC
        // bits total consumed.
        let bits = concat(&[bits_value_zero(), bits_value_zero()]);
        let bytes = pack(&bits);
        let mut r = Sv7BitReader::new(&bytes);
        let out = decode_header_loop(&mut r, 0, 2).unwrap();
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].band_type[0].as_i8(), 0);
        assert_eq!(out[0].band_type[1].as_i8(), 0);
        assert_eq!(out[0].ms_flag, None);
    }

    #[test]
    fn decode_header_loop_three_bands_stereo_mixed() {
        // Band 0: left 0, right 0 → no msflag.
        // Band 1: left 1, right -1 → msflag = 1 (M/S).
        // Band 2: left 0, right -2 → msflag = 0 (L/R).
        let bits = concat(&[
            bits_value_zero(),
            bits_value_zero(),
            //
            bits_value_one(),
            bits_value_neg_one(),
            &[true],
            //
            bits_value_zero(),
            bits_value_neg_two(),
            &[false],
        ]);
        let bytes = pack(&bits);
        let mut r = Sv7BitReader::new(&bytes);
        let out = decode_header_loop(&mut r, 2, 2).unwrap();
        assert_eq!(out.len(), 3);
        // Band 0
        assert_eq!(out[0].band_type[0].as_i8(), 0);
        assert_eq!(out[0].band_type[1].as_i8(), 0);
        assert_eq!(out[0].ms_flag, None);
        assert!(!out[0].has_samples());
        // Band 1
        assert_eq!(out[1].band_type[0].as_i8(), 1);
        assert_eq!(out[1].band_type[1].as_i8(), -1);
        assert_eq!(out[1].ms_flag, Some(true));
        assert!(out[1].has_samples());
        // Band 2
        assert_eq!(out[2].band_type[0].as_i8(), 0);
        assert_eq!(out[2].band_type[1].as_i8(), -2);
        assert_eq!(out[2].ms_flag, Some(false));
        assert!(out[2].has_samples());
    }

    #[test]
    fn decode_header_loop_max_band_thirtyone_walks_thirtytwo_bands() {
        // Maximally-wide stereo frame: every band has both channels
        // value 0 → 32 bands × 2 bits = 64 bits total, no msflags.
        let mut bits = Vec::new();
        for _ in 0..32 {
            bits.extend_from_slice(bits_value_zero());
            bits.extend_from_slice(bits_value_zero());
        }
        assert_eq!(bits.len(), 64);
        let bytes = pack(&bits);
        let mut r = Sv7BitReader::new(&bytes);
        let out = decode_header_loop(&mut r, SV7_MAX_BAND_INCLUSIVE, 2).unwrap();
        assert_eq!(out.len(), 32);
        for band in &out {
            assert_eq!(band.band_type[0].as_i8(), 0);
            assert_eq!(band.band_type[1].as_i8(), 0);
            assert_eq!(band.ms_flag, None);
        }
    }

    #[test]
    fn decode_header_loop_propagates_eof_mid_loop() {
        // Two bands. Band 0 reads cleanly (both value 0, 2 bits);
        // Band 1 needs another peek16 worth of bits but the buffer
        // doesn't have them. Use pack_exact (no peek16 padding) so
        // the reader runs out at the second band's first peek16.
        let bits = concat(&[bits_value_zero(), bits_value_zero()]);
        let bytes = pack_exact(&bits);
        let mut r = Sv7BitReader::new(&bytes);
        // max_band = 1 → 2 bands required; the second band's
        // peek16 EOFs because only 1 byte (8 bits) was packed and
        // the first 2 bits have already been consumed.
        let err = decode_header_loop(&mut r, 1, 2).unwrap_err();
        assert_eq!(err, Error::UnexpectedEof);
    }

    #[test]
    fn raw_band_type_vlc_is_distinct_from_signed_i8() {
        // Compile-time check: the wrapper isn't a transparent newtype
        // exposing operations on i8 — callers must go through
        // as_i8 to read the raw value.
        fn assert_no_into_i8<T: Copy + PartialEq>(_x: T) {}
        let w = RawBandTypeVlc::from_raw(3);
        assert_no_into_i8(w);
        // The only ways to extract are as_i8 and is_nonzero.
        assert_eq!(w.as_i8(), 3);
        assert!(w.is_nonzero());
    }

    // ─── §5.1 grounded Res header decode ──────────────────────────────

    /// MSB-first packer for the grounded §5.1 tests: `push` a VLC
    /// codeword (left-justified), `push_raw` a raw `Res` / msflag field,
    /// `finish` flushes + two zero bytes so peek16 never starves.
    struct ResPacker {
        bytes: Vec<u8>,
        acc: u32,
        nbits: u8,
    }

    impl ResPacker {
        fn new() -> Self {
            ResPacker {
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

    /// Codeword `(pattern, length)` for a band-type-header VLC symbol,
    /// hand-read from the staged `sv7-huffman-bandtype-header` `Code`
    /// column.
    fn hdr_code(value: i8) -> (u16, u8) {
        match value {
            0 => (0x8000, 1),
            1 => (0x6000, 3),
            -4 => (0x5e00, 7),
            3 => (0x5d80, 9),
            4 => (0x5d00, 9),
            -5 => (0x5c00, 8),
            2 => (0x5800, 6),
            -3 => (0x5000, 5),
            -2 => (0x4000, 4),
            -1 => (0x0000, 2),
            _ => unreachable!("unmapped hdr symbol {value}"),
        }
    }

    #[test]
    fn res_header_band0_reads_raw_four_bit_per_channel() {
        // Mono, single band 0: raw 4-bit Res = 5.
        let mut p = ResPacker::new();
        p.push_raw(5, RES_RAW_BITS);
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let bands = decode_res_header_grounded(&mut r, 0, 1, false).unwrap();
        assert_eq!(bands.len(), 1);
        assert_eq!(bands[0].res, [5, 5]);
        assert_eq!(bands[0].ms_flag, None);
        assert!(bands[0].has_samples());
    }

    #[test]
    fn res_header_later_band_deltas_off_same_channel_previous() {
        // Mono, two bands: band0 raw=7, band1 delta=+2 ⇒ Res=9.
        let mut p = ResPacker::new();
        p.push_raw(7, RES_RAW_BITS);
        let (c, l) = hdr_code(2);
        p.push(c, l);
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let bands = decode_res_header_grounded(&mut r, 1, 1, false).unwrap();
        assert_eq!(bands[0].res, [7, 7]);
        assert_eq!(bands[1].res, [9, 9]);
    }

    #[test]
    fn res_header_escape_reads_raw_absolute_for_later_band() {
        // band0 raw=3, band1 VLC symbol 4 (escape) ⇒ raw absolute = 12,
        // ignoring the delta chain.
        let mut p = ResPacker::new();
        p.push_raw(3, RES_RAW_BITS);
        let (ec, el) = hdr_code(RES_HEADER_ESCAPE_SYMBOL);
        p.push(ec, el);
        p.push_raw(12, RES_RAW_BITS);
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let bands = decode_res_header_grounded(&mut r, 1, 1, false).unwrap();
        assert_eq!(bands[0].res, [3, 3]);
        assert_eq!(bands[1].res, [12, 12]);
    }

    #[test]
    fn res_header_stereo_msflag_present_only_when_stream_ms_and_nonzero() {
        // Stereo, stream_ms = true, single band 0: left raw=2, right
        // raw=0 ⇒ non-zero band ⇒ one msflag bit (set to 1 = M/S).
        let mut p = ResPacker::new();
        p.push_raw(2, RES_RAW_BITS); // left
        p.push_raw(0, RES_RAW_BITS); // right
        p.push_raw(1, 1); // msflag = M/S
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let bands = decode_res_header_grounded(&mut r, 0, 2, true).unwrap();
        assert_eq!(bands[0].res, [2, 0]);
        assert_eq!(bands[0].ms_flag, Some(true));
    }

    #[test]
    fn res_header_stereo_msflag_absent_when_band_all_zero() {
        // Stereo, stream_ms = true, single band 0 with both channels 0
        // ⇒ no msflag bit. The next read must therefore land on band-1
        // data, not consume a phantom flag bit. Use two bands to prove
        // alignment: band0 = (0,0) no flag; band1 left delta +1 off 0
        // ⇒ 1, right delta +1 ⇒ 1, then a flag bit.
        let mut p = ResPacker::new();
        p.push_raw(0, RES_RAW_BITS); // band0 left = 0
        p.push_raw(0, RES_RAW_BITS); // band0 right = 0  (no msflag)
        let (c1, l1) = hdr_code(1);
        p.push(c1, l1); // band1 left delta +1 ⇒ 1
        p.push(c1, l1); // band1 right delta +1 ⇒ 1
        p.push_raw(0, 1); // band1 msflag = L/R
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let bands = decode_res_header_grounded(&mut r, 1, 2, true).unwrap();
        assert_eq!(bands[0].res, [0, 0]);
        assert_eq!(bands[0].ms_flag, None);
        assert_eq!(bands[1].res, [1, 1]);
        assert_eq!(bands[1].ms_flag, Some(false));
    }

    #[test]
    fn res_header_stream_ms_off_never_reads_a_flag() {
        // Stereo, stream_ms = false: even a non-zero band reads no flag.
        let mut p = ResPacker::new();
        p.push_raw(3, RES_RAW_BITS); // left
        p.push_raw(4, RES_RAW_BITS); // right
                                     // No msflag bit; immediately the buffer ends (finish pads).
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let bands = decode_res_header_grounded(&mut r, 0, 2, false).unwrap();
        assert_eq!(bands[0].res, [3, 4]);
        assert_eq!(bands[0].ms_flag, None);
    }

    #[test]
    fn res_header_rejects_bad_channel_count_and_max_band() {
        let bytes = [0xFFu8; 8];
        let mut r = Sv7BitReader::new(&bytes);
        assert_eq!(
            decode_res_header_grounded(&mut r, 0, 3, false),
            Err(Error::ChannelCountInvalid(3))
        );
        let mut r = Sv7BitReader::new(&bytes);
        assert_eq!(
            decode_res_header_grounded(&mut r, 32, 2, false),
            Err(Error::MaxBandOutOfRange(32))
        );
    }

    #[test]
    fn res_header_constants_match_spec() {
        assert_eq!(RES_HEADER_ESCAPE_SYMBOL, 4);
        assert_eq!(RES_RAW_BITS, 4);
    }
}
