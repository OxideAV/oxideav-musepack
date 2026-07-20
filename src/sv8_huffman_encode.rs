//! SV8 entropy-layer **encoders**: the exact inverses of the crate's
//! canonical-Huffman, bounded-log, and enumerative decode primitives.
//!
//! The SV8 counterpart of [`crate::sv7_huffman_encode`] (round 419).
//! Each staged `sv8-canonical-*` table is a canonical prefix code, so
//! symbol → codeword inversion is exact and unambiguous; the §6.5
//! bounded "log" and enumerative (combinatorial) codes are likewise
//! bijections. Every function here is round-tripped bit-for-bit against
//! its decode counterpart — no new format facts are introduced.
//!
//! Writer: the MSB-first [`crate::sv7_bitwriter::Sv7BitWriter`] is
//! shared — SV8 is byte-natural (§4), so the writer's output feeds the
//! byte-natural [`crate::huffman::Sv7BitReader`] directly (no §4
//! word-swap on the SV8 side).
//!
//! Source-of-record: `docs/audio/musepack/spec/musepack-headers-and-coding.md`
//! §6.1 (canonical cum_index arithmetic), §6.5 (log + enumerative
//! codes); the staged `sv8-canonical-*` / `sv8-symbols-*` tables under
//! `docs/audio/musepack/tables/`.

use crate::sv7_bitwriter::Sv7BitWriter;
use crate::sv8_huffman::Sv8CanonicalTable;
use crate::sv8_sample_decode::{binomial, enum_bitlen_lost};
use crate::{Error, Result};

/// Find the canonical codeword for symbol-map index `index` of `table`:
/// the inverse of [`Sv8CanonicalTable::decode_symbol_index`].
///
/// The decode arithmetic per length-class row is
/// `index = (cum_index − bin) mod 256` with `bin` the `length`-bit code
/// value; each row covers a contiguous descending `bin` span (from the
/// row's own left-justified `code` up to the previous row's). Inverting:
/// per row compute `bin = (cum_index − index) mod 256` and accept the
/// row whose span contains it.
///
/// Returns the left-justified 16-bit code plus its bit length.
///
/// # Errors
///
/// [`Error::SymbolNotEncodable`] if `index` is outside the table's
/// symbol map or no length class covers it (unreachable for the staged
/// tables, whose classes tile the full code space — the exhaustive
/// round-trip tests prove it).
pub fn encode_symbol_index(table: &Sv8CanonicalTable, index: usize) -> Result<(u16, u8)> {
    if index >= table.symbols.len() {
        return Err(Error::SymbolNotEncodable(index as i32));
    }
    // Rows are sorted by `code` descending; each data row's bin span is
    // [code >> (16 − len), upper >> (16 − len) − 1] against the previous
    // data row's left-justified code (0x10000 for the first).
    let mut upper: u32 = 0x1_0000;
    for entry in table.lengths.iter() {
        if entry.length == 0 {
            // Staged q4 padding sentinel — not a codeword.
            continue;
        }
        let shift = 16 - u32::from(entry.length);
        let bin_lo = u32::from(entry.code) >> shift;
        let bin_hi = (upper >> shift).saturating_sub(1);
        let bin = (i32::from(entry.cum_index) - index as i32).rem_euclid(256) as u32;
        if bin >= bin_lo && bin <= bin_hi {
            return Ok(((bin << shift) as u16, entry.length));
        }
        upper = u32::from(entry.code);
    }
    Err(Error::SymbolNotEncodable(index as i32))
}

/// Find the canonical codeword for `symbol` in `table`: the inverse of
/// [`Sv8CanonicalTable::decode`].
///
/// Within each table's decode-reachable region every symbol is unique
/// (test-proven), so the first match is the codeword the decoder maps
/// back to that symbol. (The staged `q4` map carries an unreachable
/// padding tail whose entries repeat symbols; it sits after the real
/// alphabet and is never matched first.)
///
/// # Errors
///
/// [`Error::SymbolNotEncodable`] if `symbol` is not in the table's
/// alphabet.
pub fn encode_symbol(table: &Sv8CanonicalTable, symbol: i8) -> Result<(u16, u8)> {
    let index = table
        .symbols
        .iter()
        .position(|&s| s == symbol)
        .ok_or(Error::SymbolNotEncodable(i32::from(symbol)))?;
    encode_symbol_index(table, index)
}

/// Encode `symbol` through `table` into `writer`: the write-side twin
/// of [`Sv8CanonicalTable::decode`].
///
/// # Errors
///
/// [`Error::SymbolNotEncodable`] as [`encode_symbol`].
pub fn write_symbol(
    writer: &mut Sv7BitWriter,
    table: &Sv8CanonicalTable,
    symbol: i8,
) -> Result<()> {
    let (code, length) = encode_symbol(table, symbol)?;
    writer.write_left_justified(code, length);
    Ok(())
}

/// Encode one §6.5 bounded **"log" code** for `value` in `0..max`: the
/// inverse of [`crate::sv8_band_header::decode_log_code`].
///
/// With `bitlen = ceil(log2(max))` and `lost = 2^bitlen − max`: values
/// below `lost` are short codewords of `bitlen − 1` bits carrying the
/// value directly; the rest are full `bitlen`-bit codewords carrying
/// `value + lost`. `max ≤ 1` writes nothing.
///
/// # Errors
///
/// [`Error::SampleOutOfRange`] if `value ≥ max` (no codeword exists).
pub fn write_log_code(writer: &mut Sv7BitWriter, value: u32, max: u32) -> Result<()> {
    if max <= 1 {
        return if value == 0 {
            Ok(())
        } else {
            Err(Error::SampleOutOfRange(value as i32))
        };
    }
    if value >= max {
        return Err(Error::SampleOutOfRange(value as i32));
    }
    let (bitlen, lost) = enum_bitlen_lost(max);
    if value < lost {
        writer.write_bits(value, bitlen - 1);
    } else {
        writer.write_bits(value + lost, bitlen);
    }
    Ok(())
}

/// Encode one §6.5 **enumerative (combinatorial) codeword** naming the
/// `k`-subset `mask` of `n` positions: the inverse of the crate's
/// enumerative decode (`enum_decode_subset`).
///
/// Two stages, mirroring the decode:
///
/// 1. **Combinadic rank.** Walk positions `m` from `n − 1` down to `0`;
///    each set bit at position `m` adds `C(m, remaining_k)` to the rank
///    and decrements `remaining_k`.
/// 2. **Phased-binary index write.** With
///    `(bitlen, lost) = enum_bitlen_lost(C(n, k))`: ranks below `lost`
///    are `bitlen − 1`-bit codewords; the rest are `bitlen`-bit
///    codewords carrying `rank + lost`. `C(n, k) ≤ 1` writes nothing.
///
/// # Errors
///
/// [`Error::SampleOutOfRange`] if `mask` does not have exactly `k` set
/// bits within the low `n` positions.
pub fn write_enum_subset(writer: &mut Sv7BitWriter, mask: u32, k: u32, n: u32) -> Result<()> {
    let field = if n >= 32 { u32::MAX } else { (1u32 << n) - 1 };
    if mask & !field != 0 || mask.count_ones() != k {
        return Err(Error::SampleOutOfRange(mask as i32));
    }
    // Combinadic rank of the subset.
    let mut rank: u32 = 0;
    let mut kk = k;
    let mut m = n;
    while m > 0 && kk > 0 {
        m -= 1;
        if mask & (1 << m) != 0 {
            rank += binomial(m, kk);
            kk -= 1;
        }
    }
    let total = binomial(n, k);
    let (bitlen, lost) = enum_bitlen_lost(total);
    if bitlen == 0 {
        return Ok(()); // single-codeword space carries no bits
    }
    if rank < lost {
        writer.write_bits(rank, bitlen - 1);
    } else {
        writer.write_bits(rank + lost, bitlen);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::huffman::Sv7BitReader;
    use crate::sv8_band_header::decode_log_code;
    use crate::sv8_huffman::{
        table_for_role, Sv8TableRole, SV8_BANDS_TABLE, SV8_DSCF_1_TABLE, SV8_DSCF_2_TABLE,
        SV8_Q1_TABLE, SV8_Q9UP_TABLE, SV8_RES_1_TABLE, SV8_RES_2_TABLE, SV8_SCFI_1_TABLE,
        SV8_SCFI_2_TABLE,
    };
    use crate::sv8_sample_decode::enum_decode_subset;

    /// Every staged table, by role/context, for exhaustive round-trips.
    fn all_tables() -> Vec<&'static Sv8CanonicalTable> {
        let mut out: Vec<&'static Sv8CanonicalTable> = vec![
            &SV8_BANDS_TABLE,
            &SV8_RES_1_TABLE,
            &SV8_RES_2_TABLE,
            &SV8_SCFI_1_TABLE,
            &SV8_SCFI_2_TABLE,
            &SV8_DSCF_1_TABLE,
            &SV8_DSCF_2_TABLE,
            &SV8_Q1_TABLE,
            &SV8_Q9UP_TABLE,
        ];
        for role in [
            Sv8TableRole::Q2,
            Sv8TableRole::Q3,
            Sv8TableRole::Q4,
            Sv8TableRole::Q5,
            Sv8TableRole::Q6,
            Sv8TableRole::Q7,
            Sv8TableRole::Q8,
        ] {
            for ctx in 0..2 {
                if let Some(t) = table_for_role(role, ctx) {
                    if !out.iter().any(|e| std::ptr::eq(*e, t)) {
                        out.push(t);
                    }
                }
            }
        }
        out
    }

    /// The set of symbol-map indices reachable by the decode walk: the
    /// union of every data row's contiguous `bin` span mapped through
    /// the `index = (cum − bin) mod 256` arithmetic. (The staged `q4`
    /// table carries tail padding entries past its 81-symbol alphabet
    /// that no codeword selects.)
    fn covered_indices(table: &Sv8CanonicalTable) -> Vec<usize> {
        let mut out = Vec::new();
        let mut upper: u32 = 0x1_0000;
        for e in table.lengths.iter() {
            if e.length == 0 {
                continue;
            }
            let shift = 16 - u32::from(e.length);
            let lo = u32::from(e.code) >> shift;
            let hi = (upper >> shift) - 1;
            for bin in lo..=hi {
                out.push((i32::from(e.cum_index) - bin as i32).rem_euclid(256) as usize);
            }
            upper = u32::from(e.code);
        }
        out
    }

    #[test]
    fn every_reachable_codeword_round_trips_exactly() {
        // For every decode-reachable index of every staged table:
        // encode, decode back, and confirm the exact codeword width.
        for table in all_tables() {
            let covered = covered_indices(table);
            assert!(!covered.is_empty(), "{}", table.name);
            for &index in &covered {
                let (code, length) = encode_symbol_index(table, index)
                    .unwrap_or_else(|e| panic!("{}[{index}]: {e:?}", table.name));
                let mut w = Sv7BitWriter::new();
                w.write_left_justified(code, length);
                w.write_bits(0, 16); // peek slack
                let bytes = w.finish();
                let mut r = Sv7BitReader::new(&bytes);
                let before = r.bits_remaining();
                let got = table.decode_symbol_index(&mut r).unwrap();
                assert_eq!(got, index, "{} index {index}", table.name);
                assert_eq!(
                    before - r.bits_remaining(),
                    u64::from(length),
                    "{} index {index}: exact codeword width",
                    table.name
                );
            }
        }
    }

    #[test]
    fn reachable_symbols_are_unique_and_symbol_encode_round_trips() {
        // Within the decode-reachable region every symbol is unique
        // (the padding tail may repeat symbols but is never selected),
        // so the symbol-level encoder is well-defined: decode(encode(s))
        // must return s for every reachable symbol.
        for table in all_tables() {
            let covered = covered_indices(table);
            let mut seen = std::collections::BTreeSet::new();
            for &i in &covered {
                assert!(
                    seen.insert(table.symbols[i]),
                    "{}: duplicate reachable symbol {}",
                    table.name,
                    table.symbols[i]
                );
            }
            for &i in &covered {
                let symbol = table.symbols[i];
                let mut w = Sv7BitWriter::new();
                write_symbol(&mut w, table, symbol).unwrap();
                w.write_bits(0, 16);
                let bytes = w.finish();
                let mut r = Sv7BitReader::new(&bytes);
                assert_eq!(
                    table.decode(&mut r).unwrap(),
                    symbol,
                    "{} symbol {symbol}",
                    table.name
                );
            }
        }
    }

    #[test]
    fn unknown_symbol_is_rejected() {
        // scfi-1's alphabet is 0..=3; 99 is not encodable.
        assert!(matches!(
            encode_symbol(&SV8_SCFI_1_TABLE, 99),
            Err(Error::SymbolNotEncodable(99))
        ));
        let out_of_map = SV8_SCFI_1_TABLE.symbols.len();
        assert!(matches!(
            encode_symbol_index(&SV8_SCFI_1_TABLE, out_of_map),
            Err(Error::SymbolNotEncodable(_))
        ));
    }

    #[test]
    fn write_symbol_streams_compose() {
        // A run of symbols through one table decodes back in order.
        let table = &SV8_RES_1_TABLE;
        let symbols: Vec<i8> = table.symbols.to_vec();
        let mut w = Sv7BitWriter::new();
        for &s in &symbols {
            write_symbol(&mut w, table, s).unwrap();
        }
        w.write_bits(0, 16);
        let bytes = w.finish();
        let mut r = Sv7BitReader::new(&bytes);
        for &s in &symbols {
            assert_eq!(table.decode(&mut r).unwrap(), s);
        }
    }

    #[test]
    fn log_code_round_trips_every_value() {
        for max in 1..=40u32 {
            for v in 0..max {
                let mut w = Sv7BitWriter::new();
                write_log_code(&mut w, v, max).unwrap();
                w.write_bits(0, 16);
                let bytes = w.finish();
                let mut r = Sv7BitReader::new(&bytes);
                assert_eq!(decode_log_code(&mut r, max).unwrap(), v, "max {max} v {v}");
            }
        }
    }

    #[test]
    fn log_code_rejects_out_of_range_value() {
        let mut w = Sv7BitWriter::new();
        assert!(matches!(
            write_log_code(&mut w, 5, 5),
            Err(Error::SampleOutOfRange(5))
        ));
        assert!(matches!(
            write_log_code(&mut w, 1, 1),
            Err(Error::SampleOutOfRange(1))
        ));
        // max 1, value 0: writes nothing, succeeds.
        write_log_code(&mut w, 0, 1).unwrap();
        assert!(w.is_empty());
    }

    #[test]
    fn enum_subset_round_trips_exhaustively_for_small_n() {
        for n in 1..=12u32 {
            for mask in 0..(1u32 << n) {
                let k = mask.count_ones();
                let mut w = Sv7BitWriter::new();
                write_enum_subset(&mut w, mask, k, n).unwrap();
                w.write_bits(0, 16);
                let bytes = w.finish();
                let mut r = Sv7BitReader::new(&bytes);
                let got = enum_decode_subset(&mut r, k, n).unwrap();
                assert_eq!(got, mask, "n {n} mask {mask:#b}");
            }
        }
    }

    #[test]
    fn enum_subset_rejects_wrong_popcount_or_stray_bits() {
        let mut w = Sv7BitWriter::new();
        assert!(matches!(
            write_enum_subset(&mut w, 0b101, 1, 3),
            Err(Error::SampleOutOfRange(_))
        ));
        assert!(matches!(
            write_enum_subset(&mut w, 1 << 5, 1, 3),
            Err(Error::SampleOutOfRange(_))
        ));
        assert!(w.is_empty(), "rejections write no bits");
    }
}
