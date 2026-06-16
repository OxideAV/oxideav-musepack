//! SV7 per-band sample decode (frame-body inner loop, §2.5 case
//! ladder).
//!
//! Wraps the SV7 entropy + linear-PCM-escape + zero-fill +
//! noise-substitution per-band sample paths into a single coherent
//! module, on top of the already-wired:
//!
//! - [`crate::huffman`] — `SV7_Q3_TABLE` .. `SV7_Q7_TABLE` (each a
//!   `[2][N]` context-pair) and the `Sv7BitReader` MSB-first
//!   bit-stream reader.
//! - [`crate::cns`] — `CnsPrng` two-LFSR noise generator
//!   (`.fill_samples(&mut [i32])`).
//! - [`crate::requant`] — `RES_BITS[18]` (bits per sample for the
//!   linear-PCM escape ladder) and [`crate::requant::band_type_to_res_bits`].
//!
//! Source-of-record:
//!
//! - **Structural prose**: `docs/audio/musepack/musepack-sv7-sv8-spec.md`
//!   §2.5 (frame body — quantised subband samples), the case-ladder
//!   block reproduced here for traceability:
//!
//!   ```text
//!   switch (band_type) {
//!     case -1: fill all 36 samples with random/noise values    # CNS path
//!     case  0: zero all 36 samples                              # default branch
//!     case  1: read 12 VLCs to produce the 36 samples           # 3 samples / VLC (grouped)
//!     case  2: read 18 VLCs to produce the 36 samples           # 2 samples / VLC (grouped)
//!     case  3..7:  read 36 VLCs, one per sample, using Q{band_type}
//!     case  8..17: read (band_type - 1) raw bits per sample     # linear-PCM escape
//!   }
//!   ```
//!
//! Every decoding shape comes from the structural §2.5 prose above
//! plus the existing wired tables.
//!
//! # Cases implemented this round
//!
//! - **Empty** (band_type == 0): trivially fill 36 zeros.
//! - **CNS** (band_type == -1): 36 samples drawn from the existing
//!   `CnsPrng` (range -510..=510 per `cns-prng-params.meta` notes).
//! - **Grouped3** (band_type == 1, 12 VLCs → 36 samples, 3 per VLC):
//!   each `sv7-huffman-q1` codeword `value` is a base-3-packed
//!   triplet (see [`unpack_grouped3_value`]).
//! - **Grouped2** (band_type == 2, 18 VLCs → 36 samples, 2 per VLC):
//!   each `sv7-huffman-q2` codeword `value` is a base-5-packed pair
//!   (see [`unpack_grouped2_value`]).
//! - **HuffmanPerSample** (band_type 3..=7): one Q{band_type}
//!   Huffman codeword per sample, context-selected. The Q3..Q7
//!   tables already produce signed `i8` levels covering the spec's
//!   `±D` step range.
//! - **PcmEscape** (band_type 8..=17): `band_type - 1` unsigned bits
//!   per sample (7..=16 bits), read MSB-first via
//!   `Sv7BitReader::read_bits`. The raw unsigned level is returned
//!   in `i32` (so the §2.6 reconstruction can centre it by
//!   subtracting `D = QUANTIZER_OFFSET_D[band_type + 1]`).
//!
//! # Grouped-codeword fan-out — grounded from the staged facts
//!
//! The §2.5 structural prose names the grouped cases ("read 12 VLCs
//! to produce the 36 samples" / "read 18 VLCs … 2 samples per
//! codeword") but the prose alone does not pin how one codeword
//! `value` expands into 3 (resp. 2) signed sample levels. That
//! arithmetic is nevertheless uniquely determined by the staged
//! Feist facts, by the same tiling argument the SV8 grouped round
//! used (`sv8_sample_decode`):
//!
//! - `sv7-huffman-q1` carries exactly 27 distinct `value`s spanning
//!   `0..=26` in each context half; `requant-quantizer-offset-Dc`
//!   pins band_type 1 to `D = 1` (steps `= 2D+1 = 3`). The only
//!   composition consistent with "3 samples per codeword" and 3
//!   levels per sample is a **base-3-packed triplet**: digit value =
//!   sample + D = sample + 1, samples in `-1..=1`. The all-zero
//!   triplet maps to value `13 = 1·9 + 1·3 + 1`, which is the
//!   shortest-code (most-probable) entry in the q1 ctx-0 table —
//!   confirming the centring.
//! - `sv7-huffman-q2` carries exactly 25 distinct `value`s spanning
//!   `0..=24`; band_type 2 has `D = 2` (steps `= 5`). The unique
//!   composition is a **base-5-packed pair**: digit value =
//!   sample + 2, samples in `-2..=2`. The all-zero pair maps to
//!   value `12 = 2·5 + 2`, the shortest-code q2 ctx-0 entry.
//!
//! This mirrors the SV8 case-2 base-5 grouped3 unpack exactly (only
//! the radix differs: SV7 q1 is base-3, q2 base-5), and is further
//! backed by the §3.6 lossless SV7↔SV8 relationship — the two
//! versions carry numerically-identical quantised coefficients and
//! differ only in framing + entropy coding.
//!
//! The one convention the staged values cannot pin (both digit
//! orderings are bijections onto the same value range) is the
//! **within-group emission order** — which radix digit is the first
//! of the consecutive samples. This module emits
//! **least-significant digit first**, the same choice
//! `sv8_sample_decode` made; it is isolated inside the two
//! [`unpack_grouped3_value`] / [`unpack_grouped2_value`] helpers so a
//! future observer trace pinning the opposite order is a one-line
//! flip.

use crate::cns::CnsPrng;
use crate::huffman::{
    decode as huffman_decode, sv7_q1_ctx, sv7_q2_ctx, sv7_q3_ctx, sv7_q4_ctx, sv7_q5_ctx,
    sv7_q6_ctx, sv7_q7_ctx, Sv7BitReader,
};
use crate::requant::band_type_to_res_bits;
use crate::{Error, Result};

/// SV7 §2.5 per-band sample-decode case classifier.
///
/// Mirrors the structural `switch (band_type)` block from §2.5 one
/// variant per case; the `default` branch of the spec (band_type
/// outside the enumerated cases) is represented as
/// [`BandDecodeCase::OutOfRange`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BandDecodeCase {
    /// `band_type == -1`: CNS / noise substitution. Decoder fills 36
    /// samples from the `CnsPrng`.
    Cns,
    /// `band_type == 0`: empty band. Decoder fills 36 zeros.
    Empty,
    /// `band_type == 1`: grouped, 3 samples per Huffman codeword
    /// (`sv7-huffman-q1` value = base-3-packed triplet, see
    /// [`decode_grouped3_band`]).
    Grouped3,
    /// `band_type == 2`: grouped, 2 samples per Huffman codeword
    /// (`sv7-huffman-q2` value = base-5-packed pair, see
    /// [`decode_grouped2_band`]).
    Grouped2,
    /// `band_type == 3..=7`: one Huffman codeword per sample, table
    /// `Q{band_type}` (each a `[2][N]` context-pair).
    HuffmanPerSample,
    /// `band_type == 8..=17`: linear-PCM escape, `band_type - 1`
    /// raw bits per sample (no Huffman).
    PcmEscape,
    /// `band_type` outside `-1..=17`. The spec's `default` branch
    /// (zero the band) is the safe choice for a real decoder, but
    /// the classifier returns this variant so the dispatcher can
    /// distinguish a malformed value from a legitimately-empty
    /// (case 0) band.
    OutOfRange,
}

/// Number of subband samples per band per frame (Layer-II heritage,
/// §1: each subband carries 36 samples per frame, internally 3
/// granules of 12 samples).
pub const SAMPLES_PER_BAND: usize = 36;

/// Classify a `band_type` per the §2.5 case ladder. Pure structural
/// dispatch — no bit-stream access.
pub const fn band_type_case(band_type: i8) -> BandDecodeCase {
    match band_type {
        -1 => BandDecodeCase::Cns,
        0 => BandDecodeCase::Empty,
        1 => BandDecodeCase::Grouped3,
        2 => BandDecodeCase::Grouped2,
        3..=7 => BandDecodeCase::HuffmanPerSample,
        8..=17 => BandDecodeCase::PcmEscape,
        _ => BandDecodeCase::OutOfRange,
    }
}

/// Fill 36 zero samples (case 0, "empty band").
#[inline]
pub fn fill_zero_band(out: &mut [i32; SAMPLES_PER_BAND]) {
    out.fill(0);
}

/// Fill 36 noise-substitution samples (case -1) from the supplied
/// `CnsPrng`. Each sample is in `-510..=510` per the
/// `cns-prng-params.meta` notes.
#[inline]
pub fn fill_cns_band(prng: &mut CnsPrng, out: &mut [i32; SAMPLES_PER_BAND]) {
    prng.fill_samples(out);
}

/// Number of grouped codewords a case-1 band reads (12 codewords ×
/// 3 samples each = 36 samples).
pub const GROUPED3_CODEWORDS_PER_BAND: usize = 12;

/// Number of grouped codewords a case-2 band reads (18 codewords ×
/// 2 samples each = 36 samples).
pub const GROUPED2_CODEWORDS_PER_BAND: usize = 18;

/// Unpack one §2.5 case-1 grouped codeword `value` into its three
/// consecutive samples.
///
/// The `sv7-huffman-q1` table carries exactly 27 distinct values
/// spanning `0..=26` (band_type 1, `D = 1`, 3 levels per sample), so
/// a value is a **base-3-packed triplet** with digit value =
/// sample + 1, i.e. samples in `-1..=1`. The all-zero triplet maps
/// to value `13`. See the module-level "Grouped-codeword fan-out"
/// note for the grounding and the emission-order convention
/// (least-significant digit first).
///
/// Values outside `0..=26` yield [`Error::GroupedSymbolOutOfRange`]
/// (unreachable when the value comes from `decode`-ing the staged q1
/// table, whose value alphabet the tests confine to `0..=26`; kept
/// as a defensive bound).
pub fn unpack_grouped3_value(value: i8) -> Result<[i8; 3]> {
    if !(0..=26).contains(&value) {
        return Err(Error::GroupedSymbolOutOfRange(value));
    }
    let v = value as i32;
    Ok([(v % 3 - 1) as i8, (v / 3 % 3 - 1) as i8, (v / 9 - 1) as i8])
}

/// Unpack one §2.5 case-2 grouped codeword `value` into its two
/// consecutive samples.
///
/// The `sv7-huffman-q2` table carries exactly 25 distinct values
/// spanning `0..=24` (band_type 2, `D = 2`, 5 levels per sample), so
/// a value is a **base-5-packed pair** with digit value = sample + 2,
/// i.e. samples in `-2..=2`. The all-zero pair maps to value `12`.
/// Emission order: least-significant digit first.
///
/// Values outside `0..=24` yield [`Error::GroupedSymbolOutOfRange`]
/// (defensive bound; unreachable for values drawn from the staged q2
/// table).
pub fn unpack_grouped2_value(value: i8) -> Result<[i8; 2]> {
    if !(0..=24).contains(&value) {
        return Err(Error::GroupedSymbolOutOfRange(value));
    }
    let v = value as i32;
    Ok([(v % 5 - 2) as i8, (v / 5 - 2) as i8])
}

/// Decode 36 samples for a band whose `band_type == 1` (case
/// "Grouped3"): 12 `sv7-huffman-q1` codewords from the `ctx`-selected
/// half of the staged `[2][27]` table, each fanned out into 3
/// consecutive samples via [`unpack_grouped3_value`].
///
/// `ctx` must be `0` or `1`; any other value yields
/// [`Error::UnsupportedBandType`] (`band_type` `1`), the same
/// fail-loud channel `decode_huffman_band` uses.
pub fn decode_grouped3_band(
    reader: &mut Sv7BitReader<'_>,
    ctx: usize,
    out: &mut [i8; SAMPLES_PER_BAND],
) -> Result<()> {
    if ctx > 1 {
        return Err(Error::UnsupportedBandType(1));
    }
    let table = sv7_q1_ctx(ctx);
    for group in out.chunks_exact_mut(3) {
        let value = huffman_decode(reader, table)?;
        group.copy_from_slice(&unpack_grouped3_value(value)?);
    }
    Ok(())
}

/// Decode 36 samples for a band whose `band_type == 2` (case
/// "Grouped2"): 18 `sv7-huffman-q2` codewords from the `ctx`-selected
/// half of the staged `[2][25]` table, each fanned out into 2
/// consecutive samples via [`unpack_grouped2_value`].
///
/// `ctx` must be `0` or `1`; any other value yields
/// [`Error::UnsupportedBandType`] (`band_type` `2`).
pub fn decode_grouped2_band(
    reader: &mut Sv7BitReader<'_>,
    ctx: usize,
    out: &mut [i8; SAMPLES_PER_BAND],
) -> Result<()> {
    if ctx > 1 {
        return Err(Error::UnsupportedBandType(2));
    }
    let table = sv7_q2_ctx(ctx);
    for group in out.chunks_exact_mut(2) {
        let value = huffman_decode(reader, table)?;
        group.copy_from_slice(&unpack_grouped2_value(value)?);
    }
    Ok(())
}

/// Decode 36 samples for a band whose `band_type` is in `3..=7`
/// (case "HuffmanPerSample"). Each sample is one Q`band_type`
/// Huffman codeword from the `ctx`-selected half of the staged
/// `[2][N]` table.
///
/// `ctx` must be `0` or `1`; any other value yields
/// [`Error::UnsupportedBandType`] (the same fail-loud channel the
/// dispatcher uses for the unsupported grouped cases). A `band_type`
/// outside `3..=7` likewise returns [`Error::UnsupportedBandType`].
pub fn decode_huffman_band(
    reader: &mut Sv7BitReader<'_>,
    band_type: i8,
    ctx: usize,
    out: &mut [i8; SAMPLES_PER_BAND],
) -> Result<()> {
    if ctx > 1 {
        return Err(Error::UnsupportedBandType(band_type));
    }
    let table: &'static [crate::huffman::Sv7Entry] = match band_type {
        3 => sv7_q3_ctx(ctx),
        4 => sv7_q4_ctx(ctx),
        5 => sv7_q5_ctx(ctx),
        6 => sv7_q6_ctx(ctx),
        7 => sv7_q7_ctx(ctx),
        _ => return Err(Error::UnsupportedBandType(band_type)),
    };
    for slot in out.iter_mut() {
        *slot = huffman_decode(reader, table)?;
    }
    Ok(())
}

/// Decode 36 samples for a band whose `band_type` is in `8..=17`
/// (case "PcmEscape" / linear-PCM escape ladder).
///
/// Reads `band_type - 1` unsigned bits per sample MSB-first from
/// `reader` and stores each raw level into `out`. The §2.6
/// reconstruction step centres the raw level by subtracting
/// `D = QUANTIZER_OFFSET_D[band_type + 1]`; this function emits the
/// raw pre-centring level, leaving the dequant arithmetic to the
/// caller.
///
/// `band_type` outside `8..=17` yields
/// [`Error::UnsupportedBandType`].
pub fn decode_linear_pcm_band(
    reader: &mut Sv7BitReader<'_>,
    band_type: i8,
    out: &mut [i32; SAMPLES_PER_BAND],
) -> Result<()> {
    if !(8..=17).contains(&band_type) {
        return Err(Error::UnsupportedBandType(band_type));
    }
    // band_type is in 8..=17 → `as u8` is well-defined and lookups via
    // band_type_to_res_bits are guaranteed Some(_).
    let bits =
        band_type_to_res_bits(band_type as u8).ok_or(Error::UnsupportedBandType(band_type))?;
    for slot in out.iter_mut() {
        // read_bits handles n in 1..=16; band_type - 1 yields 7..=16,
        // which lies in that range.
        *slot = reader.read_bits(bits)? as i32;
    }
    Ok(())
}

/// Loss-free widen of an `[i8; 36]` per-arm result into the
/// dispatcher's unified `[i32; 36]` buffer. Every `i8` level
/// round-trips through `i32` unchanged.
#[inline]
fn widen_into(src: &[i8; SAMPLES_PER_BAND], dst: &mut [i32; SAMPLES_PER_BAND]) {
    for (d, &s) in dst.iter_mut().zip(src.iter()) {
        *d = s as i32;
    }
}

/// Decode the 36 subband samples of one SV7 band from its `band_type`
/// alone, routing through [`band_type_case`] to the matching per-arm
/// decoder and unifying every arm on an `[i32; 36]` output.
///
/// This is the SV7 sibling of [`crate::sv8_band_decode::decode_sv8_band`]:
/// it walks the §2.5 `switch (band_type)` ladder end to end. The arms
/// that already produce `[i32; 36]` natively (CNS, empty, linear-PCM
/// escape) write `out` directly; the arms that produce signed `i8`
/// levels (the two grouped cases and the per-sample Huffman cases) are
/// decoded into a scratch `[i8; 36]` and widened into `out` via
/// [`widen_into`].
///
/// # Context knob
///
/// The grouped (`band_type` 1 / 2) and per-sample Huffman (`band_type`
/// 3..=7) arms read from the `ctx`-selected half of their staged
/// `[2][N]` context-pair tables (see [`decode_grouped3_band`],
/// [`decode_grouped2_band`], [`decode_huffman_band`]). The dispatcher
/// threads the caller-supplied `ctx` through verbatim; it makes no
/// context choice the staged tables do not already determine. The
/// CNS / empty / PCM-escape arms take no context and ignore `ctx`. A
/// `ctx` outside `0..=1` reaches the per-arm decoders' own fail-loud
/// [`Error::UnsupportedBandType`] channel.
///
/// # Fail-loud arms
///
/// - **[`BandDecodeCase::OutOfRange`]** (`band_type` outside `-1..=17`)
///   returns [`Error::UnsupportedBandType`] rather than silently
///   zeroing the band, so a malformed `band_type` is distinguishable
///   from a legitimately-empty (case 0) band — the same fail-loud
///   posture [`crate::sv8_band_decode::decode_sv8_band`] takes for its
///   non-enumerated arms.
pub fn decode_sv7_band(
    reader: &mut Sv7BitReader<'_>,
    band_type: i8,
    cns: &mut CnsPrng,
    ctx: usize,
    out: &mut [i32; SAMPLES_PER_BAND],
) -> Result<()> {
    match band_type_case(band_type) {
        BandDecodeCase::Cns => {
            fill_cns_band(cns, out);
            Ok(())
        }
        BandDecodeCase::Empty => {
            fill_zero_band(out);
            Ok(())
        }
        BandDecodeCase::Grouped3 => {
            let mut tmp = [0_i8; SAMPLES_PER_BAND];
            decode_grouped3_band(reader, ctx, &mut tmp)?;
            widen_into(&tmp, out);
            Ok(())
        }
        BandDecodeCase::Grouped2 => {
            let mut tmp = [0_i8; SAMPLES_PER_BAND];
            decode_grouped2_band(reader, ctx, &mut tmp)?;
            widen_into(&tmp, out);
            Ok(())
        }
        BandDecodeCase::HuffmanPerSample => {
            let mut tmp = [0_i8; SAMPLES_PER_BAND];
            decode_huffman_band(reader, band_type, ctx, &mut tmp)?;
            widen_into(&tmp, out);
            Ok(())
        }
        BandDecodeCase::PcmEscape => decode_linear_pcm_band(reader, band_type, out),
        BandDecodeCase::OutOfRange => Err(Error::UnsupportedBandType(band_type)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::huffman::SV7_Q3_TABLE;

    // ─── band_type_case classifier ─────────────────────────

    #[test]
    fn classifier_covers_every_documented_case() {
        assert_eq!(band_type_case(-1), BandDecodeCase::Cns);
        assert_eq!(band_type_case(0), BandDecodeCase::Empty);
        assert_eq!(band_type_case(1), BandDecodeCase::Grouped3);
        assert_eq!(band_type_case(2), BandDecodeCase::Grouped2);
        for bt in 3..=7 {
            assert_eq!(band_type_case(bt), BandDecodeCase::HuffmanPerSample);
        }
        for bt in 8..=17 {
            assert_eq!(band_type_case(bt), BandDecodeCase::PcmEscape);
        }
    }

    #[test]
    fn classifier_flags_out_of_range() {
        assert_eq!(band_type_case(-2), BandDecodeCase::OutOfRange);
        assert_eq!(band_type_case(18), BandDecodeCase::OutOfRange);
        assert_eq!(band_type_case(i8::MIN), BandDecodeCase::OutOfRange);
        assert_eq!(band_type_case(i8::MAX), BandDecodeCase::OutOfRange);
    }

    // ─── Empty band (case 0) ───────────────────────────────

    #[test]
    fn fill_zero_band_clears_buffer() {
        let mut out = [7_i32; SAMPLES_PER_BAND];
        fill_zero_band(&mut out);
        assert!(out.iter().all(|&s| s == 0));
    }

    // ─── CNS band (case -1) ────────────────────────────────

    #[test]
    fn fill_cns_band_matches_direct_prng_walk() {
        let mut via_helper = [0_i32; SAMPLES_PER_BAND];
        let mut prng_a = CnsPrng::new();
        fill_cns_band(&mut prng_a, &mut via_helper);

        // Reference: directly call fill_samples on a fresh PRNG; the
        // helper must be a pass-through so the two walks coincide
        // sample-for-sample.
        let mut via_direct = [0_i32; SAMPLES_PER_BAND];
        let mut prng_b = CnsPrng::new();
        prng_b.fill_samples(&mut via_direct);
        assert_eq!(via_helper, via_direct);

        // Range invariant: per cns-prng-params.meta, every sample is
        // in -510..=510.
        for &s in via_helper.iter() {
            assert!((-510..=510).contains(&s));
        }

        // Both walks must have advanced the PRNG by the same amount.
        assert_eq!(prng_a.state(), prng_b.state());
    }

    // ─── Grouped value unpack (cases 1, 2) ─────────────────

    #[test]
    fn unpack_grouped3_value_covers_base3_triplet() {
        // value 13 = 1*9 + 1*3 + 1 -> digits [1,1,1] -> [0,0,0].
        assert_eq!(unpack_grouped3_value(13).unwrap(), [0, 0, 0]);
        // Least-significant digit first: value 14 bumps the first
        // sample only (digit0 1->2 -> sample -1->0... wait centred):
        // 14 = 1*9 + 1*3 + 2 -> [2-1, 1-1, 1-1] = [1, 0, 0].
        assert_eq!(unpack_grouped3_value(14).unwrap(), [1, 0, 0]);
        // 12 = 1*9 + 1*3 + 0 -> [-1, 0, 0].
        assert_eq!(unpack_grouped3_value(12).unwrap(), [-1, 0, 0]);
        // 16 = 1*9 + 2*3 + 1 -> [0, 1, 0].
        assert_eq!(unpack_grouped3_value(16).unwrap(), [0, 1, 0]);
        // 10 = 1*9 + 0*3 + 1 -> [0, -1, 0].
        assert_eq!(unpack_grouped3_value(10).unwrap(), [0, -1, 0]);
        // 22 = 2*9 + 1*3 + 1 -> [0, 0, 1].
        assert_eq!(unpack_grouped3_value(22).unwrap(), [0, 0, 1]);
        // 4 = 0*9 + 1*3 + 1 -> [0, 0, -1].
        assert_eq!(unpack_grouped3_value(4).unwrap(), [0, 0, -1]);
        // Corners.
        assert_eq!(unpack_grouped3_value(0).unwrap(), [-1, -1, -1]);
        assert_eq!(unpack_grouped3_value(26).unwrap(), [1, 1, 1]);
    }

    #[test]
    fn unpack_grouped3_value_is_a_bijection_onto_minus1_to_1_cubed() {
        use std::collections::HashSet;
        let mut seen = HashSet::new();
        for v in 0_i8..=26 {
            let t = unpack_grouped3_value(v).unwrap();
            for &s in &t {
                assert!((-1..=1).contains(&s));
            }
            assert!(seen.insert(t), "duplicate triplet for value {v}");
        }
        assert_eq!(seen.len(), 27);
    }

    #[test]
    fn unpack_grouped3_value_rejects_out_of_range() {
        for v in [-1_i8, 27, 100, i8::MIN, i8::MAX] {
            assert!(matches!(
                unpack_grouped3_value(v),
                Err(Error::GroupedSymbolOutOfRange(_)),
            ));
        }
    }

    #[test]
    fn unpack_grouped2_value_covers_base5_pair() {
        // value 12 = 2*5 + 2 -> [0, 0].
        assert_eq!(unpack_grouped2_value(12).unwrap(), [0, 0]);
        // 13 = 2*5 + 3 -> [1, 0]; 11 = 2*5 + 1 -> [-1, 0].
        assert_eq!(unpack_grouped2_value(13).unwrap(), [1, 0]);
        assert_eq!(unpack_grouped2_value(11).unwrap(), [-1, 0]);
        // 17 = 3*5 + 2 -> [0, 1]; 7 = 1*5 + 2 -> [0, -1].
        assert_eq!(unpack_grouped2_value(17).unwrap(), [0, 1]);
        assert_eq!(unpack_grouped2_value(7).unwrap(), [0, -1]);
        // Corners.
        assert_eq!(unpack_grouped2_value(0).unwrap(), [-2, -2]);
        assert_eq!(unpack_grouped2_value(24).unwrap(), [2, 2]);
    }

    #[test]
    fn unpack_grouped2_value_is_a_bijection_onto_minus2_to_2_squared() {
        use std::collections::HashSet;
        let mut seen = HashSet::new();
        for v in 0_i8..=24 {
            let p = unpack_grouped2_value(v).unwrap();
            for &s in &p {
                assert!((-2..=2).contains(&s));
            }
            assert!(seen.insert(p), "duplicate pair for value {v}");
        }
        assert_eq!(seen.len(), 25);
    }

    #[test]
    fn unpack_grouped2_value_rejects_out_of_range() {
        for v in [-1_i8, 25, 100, i8::MIN, i8::MAX] {
            assert!(matches!(
                unpack_grouped2_value(v),
                Err(Error::GroupedSymbolOutOfRange(_)),
            ));
        }
    }

    /// Pack `count` repetitions of one left-justified `(code, length)`
    /// Huffman codeword MSB-first, then 2 zero tail bytes so `peek16`
    /// never runs dry mid-decode.
    fn pack_codeword(code: u16, length: u8, count: usize) -> Vec<u8> {
        let mut bits: Vec<u8> = Vec::new();
        let mut acc: u16 = 0;
        let mut nbits: u8 = 0;
        for _ in 0..count {
            for i in 0..length {
                let bit = (code >> (15 - i)) & 1;
                acc = (acc << 1) | bit;
                nbits += 1;
                if nbits == 8 {
                    bits.push(acc as u8);
                    acc = 0;
                    nbits = 0;
                }
            }
        }
        if nbits > 0 {
            bits.push((acc << (8 - nbits)) as u8);
        }
        bits.push(0);
        bits.push(0);
        bits
    }

    // ─── Grouped3 band (case 1) ────────────────────────────

    #[test]
    fn decode_grouped3_band_all_zero_triplets() {
        // The all-zero triplet is value 13; its q1 ctx-0 codeword is
        // the shortest (length-3) entry. Drive 12 of them -> 36 zeros.
        let entry = sv7_q1_ctx(0)
            .iter()
            .find(|e| e.value == 13)
            .copied()
            .expect("value 13 present");
        let bits = pack_codeword(entry.code, entry.length, GROUPED3_CODEWORDS_PER_BAND);
        let mut reader = Sv7BitReader::new(&bits);
        let mut out = [9_i8; SAMPLES_PER_BAND];
        decode_grouped3_band(&mut reader, 0, &mut out).expect("decode");
        assert!(out.iter().all(|&s| s == 0));
    }

    #[test]
    fn decode_grouped3_band_max_triplets_both_contexts() {
        // value 26 -> [1,1,1]; verify on both context halves.
        for ctx in 0..=1 {
            let entry = sv7_q1_ctx(ctx)
                .iter()
                .find(|e| e.value == 26)
                .copied()
                .expect("value 26 present");
            let bits = pack_codeword(entry.code, entry.length, GROUPED3_CODEWORDS_PER_BAND);
            let mut reader = Sv7BitReader::new(&bits);
            let mut out = [0_i8; SAMPLES_PER_BAND];
            decode_grouped3_band(&mut reader, ctx, &mut out).expect("decode");
            assert!(out.iter().all(|&s| s == 1), "ctx {ctx}");
        }
    }

    #[test]
    fn decode_grouped3_band_rejects_bad_ctx() {
        let mut reader = Sv7BitReader::new(&[0u8; 8]);
        let mut out = [0_i8; SAMPLES_PER_BAND];
        assert!(matches!(
            decode_grouped3_band(&mut reader, 2, &mut out),
            Err(Error::UnsupportedBandType(1)),
        ));
    }

    #[test]
    fn decode_grouped3_band_propagates_eof() {
        // value 26 codeword once, then EOF before 12 codewords read.
        let entry = sv7_q1_ctx(0)
            .iter()
            .find(|e| e.value == 26)
            .copied()
            .unwrap();
        // Pack only 1 codeword, no tail padding -> peek16 underruns.
        let mut bits: Vec<u8> = Vec::new();
        let mut acc: u16 = 0;
        let mut nbits: u8 = 0;
        for i in 0..entry.length {
            let bit = (entry.code >> (15 - i)) & 1;
            acc = (acc << 1) | bit;
            nbits += 1;
            if nbits == 8 {
                bits.push(acc as u8);
                acc = 0;
                nbits = 0;
            }
        }
        if nbits > 0 {
            bits.push((acc << (8 - nbits)) as u8);
        }
        let mut reader = Sv7BitReader::new(&bits);
        let mut out = [0_i8; SAMPLES_PER_BAND];
        assert!(matches!(
            decode_grouped3_band(&mut reader, 0, &mut out),
            Err(Error::UnexpectedEof),
        ));
    }

    // ─── Grouped2 band (case 2) ────────────────────────────

    #[test]
    fn decode_grouped2_band_all_zero_pairs() {
        // value 12 -> [0,0]; q2 ctx-0 shortest entry. 18 of them ->
        // 36 zeros.
        let entry = sv7_q2_ctx(0)
            .iter()
            .find(|e| e.value == 12)
            .copied()
            .expect("value 12 present");
        let bits = pack_codeword(entry.code, entry.length, GROUPED2_CODEWORDS_PER_BAND);
        let mut reader = Sv7BitReader::new(&bits);
        let mut out = [9_i8; SAMPLES_PER_BAND];
        decode_grouped2_band(&mut reader, 0, &mut out).expect("decode");
        assert!(out.iter().all(|&s| s == 0));
    }

    #[test]
    fn decode_grouped2_band_corner_pairs_both_contexts() {
        // value 24 -> [2,2]; both context halves.
        for ctx in 0..=1 {
            let entry = sv7_q2_ctx(ctx)
                .iter()
                .find(|e| e.value == 24)
                .copied()
                .expect("value 24 present");
            let bits = pack_codeword(entry.code, entry.length, GROUPED2_CODEWORDS_PER_BAND);
            let mut reader = Sv7BitReader::new(&bits);
            let mut out = [0_i8; SAMPLES_PER_BAND];
            decode_grouped2_band(&mut reader, ctx, &mut out).expect("decode");
            assert!(out.iter().all(|&s| s == 2), "ctx {ctx}");
        }
    }

    #[test]
    fn decode_grouped2_band_rejects_bad_ctx() {
        let mut reader = Sv7BitReader::new(&[0u8; 8]);
        let mut out = [0_i8; SAMPLES_PER_BAND];
        assert!(matches!(
            decode_grouped2_band(&mut reader, 5, &mut out),
            Err(Error::UnsupportedBandType(2)),
        ));
    }

    #[test]
    fn grouped_codeword_counts_tile_the_band() {
        assert_eq!(GROUPED3_CODEWORDS_PER_BAND * 3, SAMPLES_PER_BAND);
        assert_eq!(GROUPED2_CODEWORDS_PER_BAND * 2, SAMPLES_PER_BAND);
    }

    // ─── HuffmanPerSample (cases 3..=7) ────────────────────

    #[test]
    fn decode_huffman_band_3_ctx0_thirtysix_shortest_codes_round_trips() {
        // First entry of SV7_Q3 ctx 0 is (code=0xe000, length=3, value=1):
        // bits "111" repeated 36 times = 108 bits = 13 full bytes +
        // 4 trailing bits. The 4 trailing bits "1111" pad with one
        // final 1-bit code that the decoder ought to consume on the
        // 36th sample. Build that explicitly.
        assert_eq!(SV7_Q3_TABLE[0].code, 0xe000);
        assert_eq!(SV7_Q3_TABLE[0].length, 3);
        assert_eq!(SV7_Q3_TABLE[0].value, 1);
        let mut bits: Vec<u8> = Vec::new();
        // Pack 36 copies of "111" MSB-first.
        let mut acc: u8 = 0;
        let mut nbits: u8 = 0;
        for _ in 0..36 {
            for _ in 0..3 {
                acc = (acc << 1) | 1;
                nbits += 1;
                if nbits == 8 {
                    bits.push(acc);
                    acc = 0;
                    nbits = 0;
                }
            }
        }
        if nbits > 0 {
            bits.push(acc << (8 - nbits));
        }
        // Tail: 2 zero bytes so peek16 never runs out mid-decode.
        bits.push(0);
        bits.push(0);

        let mut reader = Sv7BitReader::new(&bits);
        let mut out = [0_i8; SAMPLES_PER_BAND];
        decode_huffman_band(&mut reader, 3, 0, &mut out).expect("decode");
        for (i, &s) in out.iter().enumerate() {
            assert_eq!(s, 1, "sample {i}");
        }
    }

    #[test]
    fn decode_huffman_band_7_both_contexts_produce_signed_levels() {
        // For band_type 7 (D=31 per QUANTIZER_OFFSET_D[8]), the
        // Huffman value lies in -31..=31. Walk one of the
        // shortest-code entries from each context and confirm both
        // accept the same bit-pattern.
        // SV7_Q7 first row of ctx 0:
        let ctx0_first = sv7_q7_ctx(0)[0];
        let ctx1_first = sv7_q7_ctx(1)[0];
        // Construct a stream of the ctx 0 first row's bit pattern
        // repeated 36 times.
        let mut bits: Vec<u8> = Vec::new();
        let pat = ctx0_first.code; // left-justified u16
        let len = ctx0_first.length;
        let mut acc: u16 = 0;
        let mut nbits: u8 = 0;
        for _ in 0..36 {
            // pull `len` MSB-bits out of `pat` and pack into `acc`
            for i in 0..len {
                let bit = (pat >> (15 - i)) & 1;
                acc = (acc << 1) | bit;
                nbits += 1;
                if nbits == 8 {
                    bits.push(acc as u8);
                    acc = 0;
                    nbits = 0;
                }
            }
        }
        if nbits > 0 {
            bits.push((acc << (8 - nbits)) as u8);
        }
        bits.push(0);
        bits.push(0);
        let mut reader = Sv7BitReader::new(&bits);
        let mut out = [0_i8; SAMPLES_PER_BAND];
        decode_huffman_band(&mut reader, 7, 0, &mut out).expect("ctx 0 decode");
        for &s in out.iter() {
            assert_eq!(s, ctx0_first.value);
            assert!((-31..=31).contains(&s));
        }

        // Sanity-check that ctx 1's first row decodes too (separate
        // reader so the two contexts don't interleave).
        let pat = ctx1_first.code;
        let len = ctx1_first.length;
        let mut bits: Vec<u8> = Vec::new();
        let mut acc: u16 = 0;
        let mut nbits: u8 = 0;
        for _ in 0..36 {
            for i in 0..len {
                let bit = (pat >> (15 - i)) & 1;
                acc = (acc << 1) | bit;
                nbits += 1;
                if nbits == 8 {
                    bits.push(acc as u8);
                    acc = 0;
                    nbits = 0;
                }
            }
        }
        if nbits > 0 {
            bits.push((acc << (8 - nbits)) as u8);
        }
        bits.push(0);
        bits.push(0);
        let mut reader = Sv7BitReader::new(&bits);
        let mut out = [0_i8; SAMPLES_PER_BAND];
        decode_huffman_band(&mut reader, 7, 1, &mut out).expect("ctx 1 decode");
        for &s in out.iter() {
            assert_eq!(s, ctx1_first.value);
            assert!((-31..=31).contains(&s));
        }
    }

    #[test]
    fn decode_huffman_band_rejects_out_of_range_band_type() {
        let reader = Sv7BitReader::new(&[0, 0, 0, 0]);
        let mut out = [0_i8; SAMPLES_PER_BAND];
        // band_type 0, 1, 2, 8 are out-of-range for this function.
        for bt in [-1_i8, 0, 1, 2, 8, 17, -2] {
            let mut r = reader.clone();
            assert!(matches!(
                decode_huffman_band(&mut r, bt, 0, &mut out),
                Err(Error::UnsupportedBandType(_)),
            ));
        }
        // ctx out of range.
        let mut r = reader.clone();
        assert!(matches!(
            decode_huffman_band(&mut r, 3, 2, &mut out),
            Err(Error::UnsupportedBandType(_)),
        ));
    }

    // ─── PcmEscape (cases 8..=17) ──────────────────────────

    #[test]
    fn decode_linear_pcm_band_8_reads_seven_bits_per_sample() {
        // band_type 8 -> 7 bits per sample, 36 samples = 252 bits =
        // 31 full bytes + 4 trailing bits.
        // Construct an input where sample i = i & 0x7F (an
        // increasing ramp), packed MSB-first.
        let mut bits = Vec::new();
        let mut acc: u32 = 0;
        let mut nbits: u32 = 0;
        for i in 0..SAMPLES_PER_BAND as u32 {
            let v = i & 0x7F; // 7 bits
            acc = (acc << 7) | v;
            nbits += 7;
            while nbits >= 8 {
                let shift = nbits - 8;
                bits.push((acc >> shift) as u8);
                acc &= (1 << shift) - 1;
                nbits -= 8;
            }
        }
        if nbits > 0 {
            bits.push((acc << (8 - nbits)) as u8);
        }
        let mut reader = Sv7BitReader::new(&bits);
        let mut out = [0_i32; SAMPLES_PER_BAND];
        decode_linear_pcm_band(&mut reader, 8, &mut out).expect("decode");
        for (i, &s) in out.iter().enumerate() {
            assert_eq!(s, (i as i32) & 0x7F, "sample {i}");
        }
    }

    #[test]
    fn decode_linear_pcm_band_17_reads_sixteen_bits_per_sample() {
        // band_type 17 -> 16 bits per sample. Pick 36 distinct
        // u16 patterns (sample i -> 0x0100 + i) and verify they
        // round-trip.
        let mut bits = Vec::new();
        for i in 0..SAMPLES_PER_BAND as u32 {
            let v = 0x0100u32 + i;
            bits.push((v >> 8) as u8);
            bits.push((v & 0xFF) as u8);
        }
        let mut reader = Sv7BitReader::new(&bits);
        let mut out = [0_i32; SAMPLES_PER_BAND];
        decode_linear_pcm_band(&mut reader, 17, &mut out).expect("decode");
        for (i, &s) in out.iter().enumerate() {
            assert_eq!(s, 0x0100 + i as i32, "sample {i}");
        }
    }

    #[test]
    fn decode_linear_pcm_band_rejects_band_types_outside_8_17() {
        let reader = Sv7BitReader::new(&[0xFF; 256]);
        let mut out = [0_i32; SAMPLES_PER_BAND];
        for bt in [-1_i8, 0, 1, 2, 3, 7, 18, -2] {
            let mut r = reader.clone();
            assert!(matches!(
                decode_linear_pcm_band(&mut r, bt, &mut out),
                Err(Error::UnsupportedBandType(_)),
            ));
        }
    }

    #[test]
    fn decode_linear_pcm_band_propagates_eof() {
        // Only 1 byte of input — band_type 8 needs 7 * 36 = 252 bits.
        let mut reader = Sv7BitReader::new(&[0xFF]);
        let mut out = [0_i32; SAMPLES_PER_BAND];
        assert!(matches!(
            decode_linear_pcm_band(&mut reader, 8, &mut out),
            Err(Error::UnexpectedEof),
        ));
    }

    // ─── Unified dispatcher (decode_sv7_band) ──────────────

    #[test]
    fn dispatch_empty_band_zeroes_and_touches_no_bits() {
        // case 0 -> Empty: no bit-stream read, 36 zeros.
        let mut reader = Sv7BitReader::new(&[0xFF; 8]);
        let mut cns = CnsPrng::new();
        let mut out = [9_i32; SAMPLES_PER_BAND];
        decode_sv7_band(&mut reader, 0, &mut cns, 0, &mut out).expect("dispatch");
        assert!(out.iter().all(|&s| s == 0));
        // Empty arm consumed nothing: a fresh reader on the same bytes
        // still peeks the same first 16 bits.
        let mut fresh = Sv7BitReader::new(&[0xFF; 8]);
        assert_eq!(reader.peek16().unwrap(), fresh.peek16().unwrap());
    }

    #[test]
    fn dispatch_cns_band_matches_direct_fill_and_advances_prng() {
        // case -1 -> Cns: dispatcher fill must equal a direct
        // fill_cns_band walk from an identically-seeded PRNG.
        let mut reader = Sv7BitReader::new(&[0u8; 8]);
        let mut cns_a = CnsPrng::new();
        let mut out_a = [0_i32; SAMPLES_PER_BAND];
        decode_sv7_band(&mut reader, -1, &mut cns_a, 0, &mut out_a).expect("dispatch");

        let mut cns_b = CnsPrng::new();
        let mut out_b = [0_i32; SAMPLES_PER_BAND];
        fill_cns_band(&mut cns_b, &mut out_b);

        assert_eq!(out_a, out_b);
        assert_eq!(cns_a.state(), cns_b.state());
    }

    #[test]
    fn dispatch_grouped3_matches_direct_decode_widened() {
        // case 1 -> Grouped3: 12 all-zero-triplet codewords. The
        // dispatcher output (i32) must equal the direct i8 decode
        // widened to i32 (here all zero).
        let entry = sv7_q1_ctx(0)
            .iter()
            .find(|e| e.value == 13)
            .copied()
            .expect("value 13 present");
        let bits = pack_codeword(entry.code, entry.length, GROUPED3_CODEWORDS_PER_BAND);

        let mut reader_a = Sv7BitReader::new(&bits);
        let mut cns = CnsPrng::new();
        let mut out_a = [7_i32; SAMPLES_PER_BAND];
        decode_sv7_band(&mut reader_a, 1, &mut cns, 0, &mut out_a).expect("dispatch");

        let mut reader_b = Sv7BitReader::new(&bits);
        let mut tmp = [0_i8; SAMPLES_PER_BAND];
        decode_grouped3_band(&mut reader_b, 0, &mut tmp).expect("direct");
        let expected: [i32; SAMPLES_PER_BAND] = core::array::from_fn(|i| tmp[i] as i32);

        assert_eq!(out_a, expected);
        assert!(out_a.iter().all(|&s| s == 0));
    }

    #[test]
    fn dispatch_grouped2_threads_context_through() {
        // case 2 -> Grouped2: value 24 -> [2,2] on context half 1.
        for ctx in 0..=1 {
            let entry = sv7_q2_ctx(ctx)
                .iter()
                .find(|e| e.value == 24)
                .copied()
                .expect("value 24 present");
            let bits = pack_codeword(entry.code, entry.length, GROUPED2_CODEWORDS_PER_BAND);
            let mut reader = Sv7BitReader::new(&bits);
            let mut cns = CnsPrng::new();
            let mut out = [0_i32; SAMPLES_PER_BAND];
            decode_sv7_band(&mut reader, 2, &mut cns, ctx, &mut out).expect("dispatch");
            assert!(out.iter().all(|&s| s == 2), "ctx {ctx}");
        }
    }

    #[test]
    fn dispatch_huffman_per_sample_matches_direct_decode() {
        // case 3 -> HuffmanPerSample: 36 shortest-code (value 1)
        // codewords. Dispatcher must equal the direct i8 decode widened.
        let entry = SV7_Q3_TABLE[0];
        let bits = pack_codeword(entry.code, entry.length, SAMPLES_PER_BAND);

        let mut reader_a = Sv7BitReader::new(&bits);
        let mut cns = CnsPrng::new();
        let mut out_a = [0_i32; SAMPLES_PER_BAND];
        decode_sv7_band(&mut reader_a, 3, &mut cns, 0, &mut out_a).expect("dispatch");

        let mut reader_b = Sv7BitReader::new(&bits);
        let mut tmp = [0_i8; SAMPLES_PER_BAND];
        decode_huffman_band(&mut reader_b, 3, 0, &mut tmp).expect("direct");
        let expected: [i32; SAMPLES_PER_BAND] = core::array::from_fn(|i| tmp[i] as i32);

        assert_eq!(out_a, expected);
        assert!(out_a.iter().all(|&s| s == entry.value as i32));
    }

    #[test]
    fn dispatch_pcm_escape_matches_direct_decode() {
        // case 8 -> PcmEscape: 7 bits/sample, sample i -> i & 0x7F.
        let mut bits = Vec::new();
        let mut acc: u8 = 0;
        let mut nbits: u8 = 0;
        for i in 0..SAMPLES_PER_BAND {
            let v = (i as u8) & 0x7F;
            for b in (0..7).rev() {
                acc = (acc << 1) | ((v >> b) & 1);
                nbits += 1;
                if nbits == 8 {
                    bits.push(acc);
                    acc = 0;
                    nbits = 0;
                }
            }
        }
        if nbits > 0 {
            bits.push(acc << (8 - nbits));
        }

        let mut reader_a = Sv7BitReader::new(&bits);
        let mut cns = CnsPrng::new();
        let mut out_a = [0_i32; SAMPLES_PER_BAND];
        decode_sv7_band(&mut reader_a, 8, &mut cns, 0, &mut out_a).expect("dispatch");

        let mut reader_b = Sv7BitReader::new(&bits);
        let mut out_b = [0_i32; SAMPLES_PER_BAND];
        decode_linear_pcm_band(&mut reader_b, 8, &mut out_b).expect("direct");

        assert_eq!(out_a, out_b);
        for (i, &s) in out_a.iter().enumerate() {
            assert_eq!(s, (i as i32) & 0x7F, "sample {i}");
        }
    }

    #[test]
    fn dispatch_out_of_range_band_type_fails_loud() {
        // band_type outside -1..=17 -> OutOfRange -> hard error, never
        // a silently-zeroed band.
        let mut cns = CnsPrng::new();
        let mut out = [3_i32; SAMPLES_PER_BAND];
        for bt in [-2_i8, 18, i8::MIN, i8::MAX] {
            let mut reader = Sv7BitReader::new(&[0xFF; 8]);
            assert!(
                matches!(
                    decode_sv7_band(&mut reader, bt, &mut cns, 0, &mut out),
                    Err(Error::UnsupportedBandType(b)) if b == bt,
                ),
                "band_type {bt} should fail loud",
            );
        }
    }

    #[test]
    fn dispatch_bad_ctx_reaches_per_arm_fail_loud() {
        // ctx > 1 on a context-using arm reaches the per-arm fail-loud
        // channel (grouped/huffman cases); CNS/empty/escape ignore ctx.
        let mut reader = Sv7BitReader::new(&[0xFF; 8]);
        let mut cns = CnsPrng::new();
        let mut out = [0_i32; SAMPLES_PER_BAND];
        assert!(matches!(
            decode_sv7_band(&mut reader, 1, &mut cns, 2, &mut out),
            Err(Error::UnsupportedBandType(1)),
        ));
    }

    #[test]
    fn dispatch_eof_propagates_from_arm() {
        // case 8 needs 7*36=252 bits; 1 byte -> EOF surfaces through
        // the dispatcher.
        let mut reader = Sv7BitReader::new(&[0xFF]);
        let mut cns = CnsPrng::new();
        let mut out = [0_i32; SAMPLES_PER_BAND];
        assert!(matches!(
            decode_sv7_band(&mut reader, 8, &mut cns, 0, &mut out),
            Err(Error::UnexpectedEof),
        ));
    }
}
