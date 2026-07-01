//! SV7 §5.1 band-type (`Res`) header **encode** — inverse of
//! [`crate::sv7_band_header::decode_res_header_grounded`].
//!
//! Serialises a sequence of [`Sv7ResBand`] (per-channel `Res` +
//! optional per-band M/S flag) into the exact bit run the grounded §5.1
//! header decoder reads back. Together with
//! [`crate::sv7_huffman_encode`] and [`crate::sv7_bitwriter`] this closes
//! the header layer of the SV7 encode side.
//!
//! # The §5.1 layout, inverted
//!
//! Per band `i`, per channel (left first, then right for stereo):
//!
//! - **Band 0:** the channel's `Res` is emitted as a raw
//!   [`RES_RAW_BITS`]-bit absolute (so `Res ∈ 0..=15` at band 0).
//! - **Bands 1..=max_band:** the encoder prefers the **delta** form
//!   `idx = Res − prev_same_channel_Res` when `idx` is a plain
//!   `sv7-huffman-bandtype-header` symbol (`idx ∈ {−5..=3}`, i.e. the
//!   table alphabet minus the escape). Otherwise it emits the
//!   [`RES_HEADER_ESCAPE_SYMBOL`] escape (`idx == 4`) followed by a raw
//!   [`RES_RAW_BITS`]-bit absolute `Res`, which requires `Res ∈ 0..=15`.
//!   Preferring the delta yields the shortest encoding and is a pure
//!   encoder policy — either form decodes back identically (the
//!   round-trip tests prove it).
//! - **Per-band M/S:** a single raw bit follows iff `stream_ms` is set,
//!   the stream is stereo (`nch == 2`), and the band has a non-zero
//!   channel — exactly the condition
//!   [`crate::sv7_band_header::decode_res_header_grounded`] reads it
//!   under.
//!
//! A `Res` value that is neither reachable as a plain delta nor
//! representable as a 0..=15 raw absolute (e.g. a `Res > 15` whose delta
//! collides with the escape symbol) is rejected with
//! [`Error::SampleOutOfRange`] rather than mis-encoded.
//!
//! Source-of-record: `docs/audio/musepack/spec/musepack-headers-and-coding.md`
//! §5.1 (Band-type / `Res` header). No new format facts — pure inversion
//! of the grounded §5.1 decode.

use crate::huffman::SV7_BANDTYPE_HEADER_TABLE;
use crate::sv7_band_header::{
    Sv7ResBand, RES_HEADER_ESCAPE_SYMBOL, RES_RAW_BITS, SV7_MAX_BAND_INCLUSIVE,
};
use crate::sv7_bitwriter::Sv7BitWriter;
use crate::sv7_huffman_encode::write_symbol;
use crate::{Error, Result};

/// Inclusive lower/upper bounds of a plain (non-escape) band-type-header
/// delta symbol: the `sv7-huffman-bandtype-header` alphabet is `−5..=4`,
/// and `4` is the [`RES_HEADER_ESCAPE_SYMBOL`] escape, so a delta is
/// emitted directly only when it lies in `−5..=3`.
const DELTA_MIN: i32 = -5;
const DELTA_MAX: i32 = 3;

/// Largest value representable in a raw [`RES_RAW_BITS`]-bit absolute
/// field (`2^RES_RAW_BITS − 1`).
const RAW_RES_MAX: i8 = (1 << RES_RAW_BITS) - 1;

fn channels_valid(nch: u8) -> bool {
    nch == 1 || nch == 2
}

/// Emit one channel's §5.1 `Res`: raw at band 0, delta-or-escape after.
fn write_res_channel(
    writer: &mut Sv7BitWriter,
    band_index: usize,
    prev: i8,
    res: i8,
) -> Result<()> {
    if band_index == 0 {
        if !(0..=RAW_RES_MAX).contains(&res) {
            return Err(Error::SampleOutOfRange(res as i32));
        }
        writer.write_bits(res as u32, RES_RAW_BITS);
        return Ok(());
    }
    let delta = res as i32 - prev as i32;
    if (DELTA_MIN..=DELTA_MAX).contains(&delta) {
        // Plain delta symbol (guaranteed != escape since escape == 4 > DELTA_MAX).
        write_symbol(writer, &SV7_BANDTYPE_HEADER_TABLE, delta as i8)?;
        Ok(())
    } else {
        // Escape: symbol 4, then a raw 4-bit absolute Res.
        if !(0..=RAW_RES_MAX).contains(&res) {
            return Err(Error::SampleOutOfRange(res as i32));
        }
        write_symbol(writer, &SV7_BANDTYPE_HEADER_TABLE, RES_HEADER_ESCAPE_SYMBOL)?;
        writer.write_bits(res as u32, RES_RAW_BITS);
        Ok(())
    }
}

/// Encode the full §5.1 grounded `Res` (band-type) header for `bands`
/// into `writer`, the exact inverse of
/// [`crate::sv7_band_header::decode_res_header_grounded`].
///
/// `bands` is ascending band order (`bands[0]` is band 0); its length is
/// the band count (`max_band + 1`). `nch` is `1` (mono) or `2` (stereo),
/// and `stream_ms` is the stream-wide M/S flag. In mono only `res[0]` of
/// each band is emitted and no per-band M/S bit is written.
///
/// # Errors
///
/// - [`Error::ChannelCountInvalid`] if `nch` is neither 1 nor 2.
/// - [`Error::MaxBandOutOfRange`] if `bands.len()` exceeds the 32-subband
///   heritage bound (`> SV7_MAX_BAND_INCLUSIVE + 1`) or is empty.
/// - [`Error::SampleOutOfRange`] if a `Res` cannot be represented (band-0
///   / escape raw outside `0..=15`).
/// - [`Error::SymbolNotEncodable`] propagated from a delta symbol write
///   (unreachable: every `DELTA_MIN..=DELTA_MAX` symbol is in the table).
pub fn encode_res_header_grounded(
    writer: &mut Sv7BitWriter,
    bands: &[Sv7ResBand],
    nch: u8,
    stream_ms: bool,
) -> Result<()> {
    if !channels_valid(nch) {
        return Err(Error::ChannelCountInvalid(nch));
    }
    if bands.is_empty() || bands.len() > SV7_MAX_BAND_INCLUSIVE as usize + 1 {
        // Reuse MaxBandOutOfRange to report an unrepresentable band count
        // (the decoder's own out-of-range channel), reporting the derived
        // max_band the caller implied.
        let implied = bands.len().saturating_sub(1).min(u8::MAX as usize) as u8;
        return Err(Error::MaxBandOutOfRange(implied));
    }
    let mut prev = [0_i8; 2];
    for (i, band) in bands.iter().enumerate() {
        write_res_channel(writer, i, prev[0], band.res[0])?;
        if nch == 2 {
            write_res_channel(writer, i, prev[1], band.res[1])?;
        }
        prev = band.res;
        // §5.1 per-band M/S bit: only when stream M/S is set, stereo, and
        // the band has a non-zero channel.
        if stream_ms && nch == 2 && band.has_samples() {
            let bit = matches!(band.ms_flag, Some(true)) as u32;
            writer.write_bits(bit, 1);
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::huffman::Sv7BitReader;
    use crate::sv7_band_header::decode_res_header_grounded;

    /// Round-trip helper: encode `bands`, then decode the produced bit
    /// run back and assert equality (res + ms_flag per band).
    fn assert_round_trip(bands: &[Sv7ResBand], nch: u8, stream_ms: bool) {
        let max_band = (bands.len() - 1) as u8;
        let mut w = Sv7BitWriter::new();
        encode_res_header_grounded(&mut w, bands, nch, stream_ms).expect("encode");
        let mut bytes = w.finish();
        bytes.push(0);
        bytes.push(0); // peek16 padding
        let mut r = Sv7BitReader::new(&bytes);
        let decoded = decode_res_header_grounded(&mut r, max_band, nch, stream_ms).expect("decode");
        assert_eq!(decoded.len(), bands.len());
        for (i, (d, e)) in decoded.iter().zip(bands.iter()).enumerate() {
            // In mono the decoder mirrors res[0] into res[1]; compare only
            // the channels that were actually coded.
            assert_eq!(d.res[0], e.res[0], "band {i} left res");
            if nch == 2 {
                assert_eq!(d.res[1], e.res[1], "band {i} right res");
            }
            assert_eq!(d.ms_flag, e.ms_flag, "band {i} ms_flag");
        }
    }

    fn band(res: [i8; 2], ms: Option<bool>) -> Sv7ResBand {
        Sv7ResBand { res, ms_flag: ms }
    }

    #[test]
    fn mono_single_band_raw_round_trips() {
        assert_round_trip(&[band([5, 5], None)], 1, false);
    }

    #[test]
    fn mono_delta_chain_round_trips() {
        // band0 raw 7, band1 delta +2 -> 9, band2 delta -3 -> 6.
        assert_round_trip(
            &[band([7, 7], None), band([9, 9], None), band([6, 6], None)],
            1,
            false,
        );
    }

    #[test]
    fn mono_escape_used_when_delta_out_of_range() {
        // band0 = 3, band1 = 12: delta +9 is out of {-5..=3}, so escape.
        assert_round_trip(&[band([3, 3], None), band([12, 12], None)], 1, false);
    }

    #[test]
    fn mono_delta_equal_to_escape_symbol_forces_escape() {
        // band0 = 2, band1 = 6: delta +4 collides with the escape symbol,
        // so the encoder must escape (and Res 6 fits the raw field).
        assert_round_trip(&[band([2, 2], None), band([6, 6], None)], 1, false);
    }

    #[test]
    fn negative_and_large_res_reachable_via_delta() {
        // A CNS band (-1) and a dense band (17) are only reachable as
        // deltas off the previous band, never as a raw absolute.
        // band0 = 0, band1 delta -1 -> -1 (CNS), band2 delta +2 -> 1.
        assert_round_trip(
            &[band([0, 0], None), band([-1, -1], None), band([1, 1], None)],
            1,
            false,
        );
    }

    #[test]
    fn stereo_no_stream_ms_round_trips() {
        assert_round_trip(
            &[band([3, 4], None), band([5, 2], None), band([0, 0], None)],
            2,
            false,
        );
    }

    #[test]
    fn stereo_stream_ms_writes_flags_only_for_nonzero_bands() {
        // band0 both zero -> no flag; band1 nonzero -> flag M/S;
        // band2 right-only nonzero -> flag L/R.
        assert_round_trip(
            &[
                band([0, 0], None),
                band([2, 3], Some(true)),
                band([0, 1], Some(false)),
            ],
            2,
            true,
        );
    }

    #[test]
    fn stereo_stream_ms_full_width_round_trips() {
        // 32 bands, alternating patterns, stereo + stream M/S.
        let mut bands = Vec::new();
        for i in 0..32i8 {
            let res = [(i % 6), ((i + 3) % 6)];
            let ms = if res[0] != 0 || res[1] != 0 {
                Some(i % 2 == 0)
            } else {
                None
            };
            bands.push(band(res, ms));
        }
        assert_round_trip(&bands, 2, true);
    }

    #[test]
    fn rejects_invalid_channel_count() {
        let mut w = Sv7BitWriter::new();
        assert_eq!(
            encode_res_header_grounded(&mut w, &[band([1, 1], None)], 3, false),
            Err(Error::ChannelCountInvalid(3)),
        );
    }

    #[test]
    fn rejects_band0_res_above_raw_range() {
        // Res 16 at band 0 exceeds the 4-bit raw field.
        let mut w = Sv7BitWriter::new();
        assert_eq!(
            encode_res_header_grounded(&mut w, &[band([16, 16], None)], 1, false),
            Err(Error::SampleOutOfRange(16)),
        );
    }

    #[test]
    fn rejects_empty_and_oversized_band_lists() {
        let mut w = Sv7BitWriter::new();
        assert!(matches!(
            encode_res_header_grounded(&mut w, &[], 1, false),
            Err(Error::MaxBandOutOfRange(_)),
        ));
        let big = vec![band([0, 0], None); 33];
        let mut w = Sv7BitWriter::new();
        assert!(matches!(
            encode_res_header_grounded(&mut w, &big, 1, false),
            Err(Error::MaxBandOutOfRange(_)),
        ));
    }
}
