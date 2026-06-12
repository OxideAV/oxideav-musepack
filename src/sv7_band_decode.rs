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
//! # Cases NOT implemented this round (DOCS-GAP)
//!
//! - **Grouped3** (band_type == 1, 12 VLCs → 36 samples, 3 per VLC)
//! - **Grouped2** (band_type == 2, 18 VLCs → 36 samples, 2 per VLC)
//!
//! The §2.5 structural prose names the case but does not specify the
//! per-codeword **sample-unpack** convention — how a single `i8`
//! codeword expands into 3 (or 2) signed sample levels. That fact
//! lives only in the walled Trac `SV7Specification` page and the
//! decoder source, both forbidden. The dispatcher classifier
//! [`band_type_case`] returns the right enum variant for these
//! cases; the dispatcher itself returns
//! [`Error::UnsupportedBandType`] when asked to actually decode
//! one — fail-loud, not silently-wrong.

use crate::cns::CnsPrng;
use crate::huffman::{
    decode as huffman_decode, sv7_q3_ctx, sv7_q4_ctx, sv7_q5_ctx, sv7_q6_ctx, sv7_q7_ctx,
    Sv7BitReader,
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
    /// `band_type == 1`: grouped, 3 samples per Huffman codeword.
    /// **DOCS-GAP**: per-codeword unpack convention unspecified in
    /// the structural §2.5 prose.
    Grouped3,
    /// `band_type == 2`: grouped, 2 samples per Huffman codeword.
    /// **DOCS-GAP**: per-codeword unpack convention unspecified.
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
}
