//! SV7 §2.5 per-band quantised-sample **encode** — inverse of the
//! [`crate::sv7_band_decode`] sample arms.
//!
//! Serialises a band's 36 quantised sample levels into the bit run the
//! §2.5 `switch (band_type)` decoder reads back, one arm per case:
//!
//! - **Empty** (`band_type == 0`) / **CNS** (`band_type == -1`): no bits
//!   — an empty band carries no samples and a noise band is synthesised
//!   from the PRNG on decode, not sample-coded. [`encode_sv7_band`] emits
//!   nothing for these arms (the input levels are ignored, exactly as the
//!   decoder ignores the stream for them).
//! - **Grouped3** (`band_type == 1`): 12 `sv7-huffman-q1` codewords, each
//!   the base-3 pack of a `-1..=1` triplet ([`pack_grouped3`]).
//! - **Grouped2** (`band_type == 2`): 18 `sv7-huffman-q2` codewords, each
//!   the base-5 pack of a `-2..=2` pair ([`pack_grouped2`]).
//! - **HuffmanPerSample** (`band_type 3..=7`): 36 `Q{band_type}`
//!   codewords, one per sample.
//! - **PcmEscape** (`band_type 8..=17`): `band_type - 1` raw bits per
//!   sample (the unsigned pre-centring level the decoder reads).
//!
//! The base-3 / base-5 packing here is the exact algebraic inverse of
//! [`crate::sv7_band_decode::unpack_grouped3_value`] /
//! [`unpack_grouped2_value`](crate::sv7_band_decode::unpack_grouped2_value),
//! including their least-significant-digit-first emission order, so a
//! grouped band round-trips digit-for-digit.
//!
//! Source-of-record: `docs/audio/musepack/musepack-sv7-sv8-spec.md` §2.5
//! and `docs/audio/musepack/spec/musepack-headers-and-coding.md`
//! §5.4/§5.5. No new format facts — pure inversion of the wired §2.5
//! decode.

use crate::huffman::{
    sv7_q1_ctx, sv7_q2_ctx, sv7_q3_ctx, sv7_q4_ctx, sv7_q5_ctx, sv7_q6_ctx, sv7_q7_ctx,
};
use crate::requant::band_type_to_res_bits;
use crate::sv7_band_decode::{band_type_case, BandDecodeCase, SAMPLES_PER_BAND};
use crate::sv7_bitwriter::Sv7BitWriter;
use crate::sv7_huffman_encode::write_symbol;
use crate::{Error, Result};

/// Pack a `-1..=1` triplet into its §2.5 case-1 base-3 codeword value
/// (`0..=26`), the inverse of
/// [`crate::sv7_band_decode::unpack_grouped3_value`]:
/// `value = (s0+1) + (s1+1)·3 + (s2+1)·9`.
///
/// # Errors
///
/// [`Error::SampleOutOfRange`] if any sample is outside `-1..=1`.
pub fn pack_grouped3(samples: [i8; 3]) -> Result<i8> {
    for &s in &samples {
        if !(-1..=1).contains(&s) {
            return Err(Error::SampleOutOfRange(s as i32));
        }
    }
    let v = (samples[0] as i32 + 1) + (samples[1] as i32 + 1) * 3 + (samples[2] as i32 + 1) * 9;
    Ok(v as i8)
}

/// Pack a `-2..=2` pair into its §2.5 case-2 base-5 codeword value
/// (`0..=24`), the inverse of
/// [`crate::sv7_band_decode::unpack_grouped2_value`]:
/// `value = (s0+2) + (s1+2)·5`.
///
/// # Errors
///
/// [`Error::SampleOutOfRange`] if either sample is outside `-2..=2`.
pub fn pack_grouped2(samples: [i8; 2]) -> Result<i8> {
    for &s in &samples {
        if !(-2..=2).contains(&s) {
            return Err(Error::SampleOutOfRange(s as i32));
        }
    }
    let v = (samples[0] as i32 + 2) + (samples[1] as i32 + 2) * 5;
    Ok(v as i8)
}

/// Encode 36 case-1 samples (12 base-3 q1 codewords) from the `ctx`
/// context half.
///
/// # Errors
///
/// - [`Error::UnsupportedBandType`] (`1`) if `ctx > 1`.
/// - [`Error::SampleOutOfRange`] if any sample is outside `-1..=1`.
pub fn encode_grouped3_band(
    writer: &mut Sv7BitWriter,
    ctx: usize,
    samples: &[i8; SAMPLES_PER_BAND],
) -> Result<()> {
    if ctx > 1 {
        return Err(Error::UnsupportedBandType(1));
    }
    let table = sv7_q1_ctx(ctx);
    for group in samples.chunks_exact(3) {
        let value = pack_grouped3([group[0], group[1], group[2]])?;
        write_symbol(writer, table, value)?;
    }
    Ok(())
}

/// Encode 36 case-2 samples (18 base-5 q2 codewords) from the `ctx`
/// context half.
///
/// # Errors
///
/// - [`Error::UnsupportedBandType`] (`2`) if `ctx > 1`.
/// - [`Error::SampleOutOfRange`] if any sample is outside `-2..=2`.
pub fn encode_grouped2_band(
    writer: &mut Sv7BitWriter,
    ctx: usize,
    samples: &[i8; SAMPLES_PER_BAND],
) -> Result<()> {
    if ctx > 1 {
        return Err(Error::UnsupportedBandType(2));
    }
    let table = sv7_q2_ctx(ctx);
    for group in samples.chunks_exact(2) {
        let value = pack_grouped2([group[0], group[1]])?;
        write_symbol(writer, table, value)?;
    }
    Ok(())
}

/// Encode 36 samples for a `band_type` in `3..=7` (one `Q{band_type}`
/// codeword per sample) from the `ctx` context half.
///
/// # Errors
///
/// - [`Error::UnsupportedBandType`] if `ctx > 1` or `band_type` is not in
///   `3..=7`.
/// - [`Error::SymbolNotEncodable`] if a sample is outside the table's
///   alphabet.
pub fn encode_huffman_band(
    writer: &mut Sv7BitWriter,
    band_type: i8,
    ctx: usize,
    samples: &[i8; SAMPLES_PER_BAND],
) -> Result<()> {
    if ctx > 1 {
        return Err(Error::UnsupportedBandType(band_type));
    }
    let table = match band_type {
        3 => sv7_q3_ctx(ctx),
        4 => sv7_q4_ctx(ctx),
        5 => sv7_q5_ctx(ctx),
        6 => sv7_q6_ctx(ctx),
        7 => sv7_q7_ctx(ctx),
        _ => return Err(Error::UnsupportedBandType(band_type)),
    };
    for &s in samples.iter() {
        write_symbol(writer, table, s)?;
    }
    Ok(())
}

/// Encode 36 samples for a `band_type` in `8..=17` (linear-PCM escape):
/// `band_type - 1` raw bits per sample, MSB-first.
///
/// Each sample is the unsigned pre-centring level the decoder reads back
/// via `Sv7BitReader::read_bits`; it must fit the arm's bit width.
///
/// # Errors
///
/// - [`Error::UnsupportedBandType`] if `band_type` is not in `8..=17`.
/// - [`Error::SampleOutOfRange`] if a level does not fit `band_type - 1`
///   unsigned bits.
pub fn encode_linear_pcm_band(
    writer: &mut Sv7BitWriter,
    band_type: i8,
    samples: &[i32; SAMPLES_PER_BAND],
) -> Result<()> {
    if !(8..=17).contains(&band_type) {
        return Err(Error::UnsupportedBandType(band_type));
    }
    let bits =
        band_type_to_res_bits(band_type as u8).ok_or(Error::UnsupportedBandType(band_type))?;
    let max = if bits >= 32 {
        u32::MAX
    } else {
        (1u32 << bits) - 1
    };
    for &s in samples.iter() {
        if s < 0 || s as u32 > max {
            return Err(Error::SampleOutOfRange(s));
        }
        writer.write_bits(s as u32, bits);
    }
    Ok(())
}

/// Encode the 36 subband samples of one SV7 band from its `band_type`,
/// the exact inverse of [`crate::sv7_band_decode::decode_sv7_band`].
///
/// Routes through [`band_type_case`] to the matching per-arm encoder. The
/// CNS (`-1`) and empty (`0`) arms write **no bits** (a noise band is
/// PRNG-synthesised on decode and an empty band carries nothing) — the
/// input `samples` are ignored for those arms, exactly as the decoder
/// ignores the stream. The grouped and per-sample-Huffman arms require
/// `samples` to be i8-range and valid for the arm; the PCM-escape arm
/// takes the unsigned raw levels directly.
///
/// # Errors
///
/// - [`Error::UnsupportedBandType`] for a `band_type` outside `-1..=17`
///   or an out-of-range `ctx`.
/// - [`Error::SampleOutOfRange`] / [`Error::SymbolNotEncodable`] from an
///   arm whose input level is not representable.
pub fn encode_sv7_band(
    writer: &mut Sv7BitWriter,
    band_type: i8,
    ctx: usize,
    samples: &[i32; SAMPLES_PER_BAND],
) -> Result<()> {
    match band_type_case(band_type) {
        // No bits: noise band (PRNG on decode) / empty band.
        BandDecodeCase::Cns | BandDecodeCase::Empty => Ok(()),
        BandDecodeCase::Grouped3 => {
            let narrowed = narrow_to_i8(samples)?;
            encode_grouped3_band(writer, ctx, &narrowed)
        }
        BandDecodeCase::Grouped2 => {
            let narrowed = narrow_to_i8(samples)?;
            encode_grouped2_band(writer, ctx, &narrowed)
        }
        BandDecodeCase::HuffmanPerSample => {
            let narrowed = narrow_to_i8(samples)?;
            encode_huffman_band(writer, band_type, ctx, &narrowed)
        }
        BandDecodeCase::PcmEscape => encode_linear_pcm_band(writer, band_type, samples),
        BandDecodeCase::OutOfRange => Err(Error::UnsupportedBandType(band_type)),
    }
}

/// Narrow an `[i32; 36]` sample buffer to `[i8; 36]`, failing loud on any
/// value outside the `i8` range (the grouped / Huffman arms only ever
/// carry small signed levels).
fn narrow_to_i8(samples: &[i32; SAMPLES_PER_BAND]) -> Result<[i8; SAMPLES_PER_BAND]> {
    let mut out = [0_i8; SAMPLES_PER_BAND];
    for (o, &s) in out.iter_mut().zip(samples.iter()) {
        *o = i8::try_from(s).map_err(|_| Error::SampleOutOfRange(s))?;
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cns::CnsPrng;
    use crate::huffman::Sv7BitReader;
    use crate::sv7_band_decode::{decode_sv7_band, unpack_grouped2_value, unpack_grouped3_value};

    /// Encode a band and decode it back through `decode_sv7_band`,
    /// returning the reconstructed `[i32; 36]`.
    fn round_trip_band(band_type: i8, ctx: usize, samples: &[i32; SAMPLES_PER_BAND]) -> [i32; 36] {
        let mut w = Sv7BitWriter::new();
        encode_sv7_band(&mut w, band_type, ctx, samples).expect("encode");
        let mut bytes = w.finish();
        bytes.push(0);
        bytes.push(0);
        bytes.push(0);
        bytes.push(0);
        let mut r = Sv7BitReader::new(&bytes);
        let mut cns = CnsPrng::new();
        let mut out = [0_i32; SAMPLES_PER_BAND];
        decode_sv7_band(&mut r, band_type, &mut cns, ctx, &mut out).expect("decode");
        out
    }

    #[test]
    fn pack_grouped3_inverts_unpack() {
        for v in 0_i8..=26 {
            let t = unpack_grouped3_value(v).unwrap();
            assert_eq!(pack_grouped3(t).unwrap(), v, "triplet {t:?}");
        }
    }

    #[test]
    fn pack_grouped2_inverts_unpack() {
        for v in 0_i8..=24 {
            let p = unpack_grouped2_value(v).unwrap();
            assert_eq!(pack_grouped2(p).unwrap(), v, "pair {p:?}");
        }
    }

    #[test]
    fn pack_grouped_rejects_out_of_range() {
        assert_eq!(pack_grouped3([2, 0, 0]), Err(Error::SampleOutOfRange(2)));
        assert_eq!(pack_grouped2([3, 0]), Err(Error::SampleOutOfRange(3)));
    }

    #[test]
    fn grouped3_band_round_trips_both_contexts() {
        // A varied -1..=1 pattern over 36 samples.
        let mut s = [0_i32; SAMPLES_PER_BAND];
        for (i, v) in s.iter_mut().enumerate() {
            *v = (i as i32 % 3) - 1;
        }
        for ctx in 0..=1 {
            assert_eq!(round_trip_band(1, ctx, &s), s, "ctx {ctx}");
        }
    }

    #[test]
    fn grouped2_band_round_trips_both_contexts() {
        let mut s = [0_i32; SAMPLES_PER_BAND];
        for (i, v) in s.iter_mut().enumerate() {
            *v = (i as i32 % 5) - 2;
        }
        for ctx in 0..=1 {
            assert_eq!(round_trip_band(2, ctx, &s), s, "ctx {ctx}");
        }
    }

    #[test]
    fn huffman_bands_round_trip_over_table_alphabet() {
        // For each band_type 3..=7, drive the 36 samples with values
        // drawn from the ctx-0 table's own alphabet so every symbol is
        // encodable, then confirm the decode reproduces them.
        for bt in 3..=7i8 {
            let table = match bt {
                3 => sv7_q3_ctx(0),
                4 => sv7_q4_ctx(0),
                5 => sv7_q5_ctx(0),
                6 => sv7_q6_ctx(0),
                7 => sv7_q7_ctx(0),
                _ => unreachable!(),
            };
            let alphabet: Vec<i32> = {
                let mut v: Vec<i32> = table.iter().map(|e| e.value as i32).collect();
                v.dedup();
                v
            };
            let mut s = [0_i32; SAMPLES_PER_BAND];
            for (i, slot) in s.iter_mut().enumerate() {
                *slot = alphabet[i % alphabet.len()];
            }
            assert_eq!(round_trip_band(bt, 0, &s), s, "band_type {bt}");
        }
    }

    #[test]
    fn pcm_escape_bands_round_trip() {
        // band_type 8 -> 7 bits, 17 -> 16 bits. Ramp within each width.
        for bt in [8_i8, 12, 17] {
            let bits = band_type_to_res_bits(bt as u8).unwrap();
            let max = (1u32 << bits) - 1;
            let mut s = [0_i32; SAMPLES_PER_BAND];
            for (i, slot) in s.iter_mut().enumerate() {
                *slot = (i as u32 % (max + 1)) as i32;
            }
            assert_eq!(round_trip_band(bt, 0, &s), s, "band_type {bt}");
        }
    }

    #[test]
    fn cns_and_empty_write_no_bits() {
        let s = [3_i32; SAMPLES_PER_BAND];
        for bt in [-1_i8, 0] {
            let mut w = Sv7BitWriter::new();
            encode_sv7_band(&mut w, bt, 0, &s).unwrap();
            assert!(w.is_empty(), "band_type {bt} should write no bits");
        }
    }

    #[test]
    fn pcm_escape_rejects_level_too_wide() {
        // band_type 8 -> 7 bits; 128 does not fit.
        let mut s = [0_i32; SAMPLES_PER_BAND];
        s[0] = 128;
        let mut w = Sv7BitWriter::new();
        assert_eq!(
            encode_linear_pcm_band(&mut w, 8, &s),
            Err(Error::SampleOutOfRange(128)),
        );
    }

    #[test]
    fn out_of_range_band_type_fails_loud() {
        let s = [0_i32; SAMPLES_PER_BAND];
        let mut w = Sv7BitWriter::new();
        for bt in [-2_i8, 18] {
            assert!(matches!(
                encode_sv7_band(&mut w, bt, 0, &s),
                Err(Error::UnsupportedBandType(_)),
            ));
        }
    }

    #[test]
    fn grouped_sample_outside_i8_fails_loud() {
        // A grouped arm handed a value outside i8 range narrows-and-fails.
        let mut s = [0_i32; SAMPLES_PER_BAND];
        s[0] = 100_000;
        let mut w = Sv7BitWriter::new();
        assert_eq!(
            encode_sv7_band(&mut w, 1, 0, &s),
            Err(Error::SampleOutOfRange(100_000)),
        );
    }
}
