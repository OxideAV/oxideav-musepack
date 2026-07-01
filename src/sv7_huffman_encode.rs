//! SV7 entropy **encode**: symbol → `mpc_huffman` codeword.
//!
//! The exact inverse of [`crate::huffman::decode`]. Given a target
//! symbol and one of the staged `sv7-huffman-*` decoder tables, this
//! module finds the codeword that [`crate::huffman::decode`] maps back
//! to that symbol and emits it through [`crate::sv7_bitwriter::Sv7BitWriter`].
//!
//! # Why the lookup is exact
//!
//! Each `mpc_huffman` table ([`crate::huffman::Sv7Entry`]) is a valid
//! canonical prefix code: rows are sorted by the left-justified 16-bit
//! `code` descending, and [`crate::huffman::decode`] returns the first
//! row whose `code <= peek16`. A row `E` with `length` `L` "owns" every
//! 16-bit peek in `[E.code, E.code + 2^(16-L) - 1]` — a contiguous block
//! that (prefix-code property) lies strictly below the next-larger row's
//! `code`. So emitting `E`'s high `L` bits, whatever bits follow, always
//! decodes back to `E.value`. [`encode_symbol`] therefore just returns
//! the first table row carrying the requested `value`; the
//! [`crate::sv7_bitwriter`] round-trip tests confirm the inversion for
//! every value of every staged table.
//!
//! When a table lists more than one row for the same symbol value (it
//! does not, for the staged SV7 tables — each is a bijection over its
//! alphabet — but the lookup does not assume so), the first match in
//! table order is chosen; either row decodes back correctly, so the
//! choice is immaterial to correctness.
//!
//! Source-of-record: `docs/audio/musepack/spec/musepack-headers-and-coding.md`
//! §5 (the SV7 band-type / SCFI / DSCF / sample VLC tables) and the
//! staged `docs/audio/musepack/tables/sv7-huffman-*` facts wired through
//! [`crate::huffman`]. No new format facts — pure inversion of an
//! existing decode table.

use crate::huffman::Sv7Entry;
use crate::sv7_bitwriter::Sv7BitWriter;
use crate::{Error, Result};

/// Find the `mpc_huffman` codeword that decodes to `value` in `table`.
///
/// Returns the first [`Sv7Entry`] whose `value` equals `value`. This is
/// the exact codeword [`crate::huffman::decode`] would map back to that
/// symbol (see the module docs for why "first match" is correct).
///
/// # Errors
///
/// [`Error::SymbolNotEncodable`] if no row of `table` carries `value`
/// (the symbol is outside the table's alphabet).
pub fn encode_symbol(table: &[Sv7Entry], value: i8) -> Result<Sv7Entry> {
    table
        .iter()
        .find(|e| e.value == value)
        .copied()
        .ok_or(Error::SymbolNotEncodable(value as i32))
}

/// Emit the `mpc_huffman` codeword for `value` from `table` into
/// `writer` (its high `length` bits, MSB-first).
///
/// # Errors
///
/// [`Error::SymbolNotEncodable`] if `value` is outside `table`'s
/// alphabet.
pub fn write_symbol(writer: &mut Sv7BitWriter, table: &[Sv7Entry], value: i8) -> Result<()> {
    let entry = encode_symbol(table, value)?;
    writer.write_left_justified(entry.code, entry.length);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::huffman::{
        decode, sv7_q1_ctx, sv7_q2_ctx, sv7_q3_ctx, sv7_q4_ctx, sv7_q5_ctx, sv7_q6_ctx, sv7_q7_ctx,
        Sv7BitReader, SV7_BANDTYPE_HEADER_TABLE, SV7_DSCF_TABLE, SV7_SCFI_TABLE,
    };

    /// Encode every distinct value of `table`, then decode it back and
    /// confirm the round-trip is the identity. This is the core
    /// correctness guarantee for the whole encode side: if every symbol
    /// of every table round-trips, the entropy encoder is bit-exact
    /// against the decoder that already exists.
    fn assert_table_round_trips(table: &[Sv7Entry], label: &str) {
        // Collect the distinct values in table order.
        let mut values: Vec<i8> = Vec::new();
        for e in table {
            if !values.contains(&e.value) {
                values.push(e.value);
            }
        }
        assert!(!values.is_empty(), "{label}: table has no symbols");
        for &v in &values {
            let mut w = Sv7BitWriter::new();
            write_symbol(&mut w, table, v).unwrap_or_else(|_| panic!("{label}: encode {v}"));
            let mut bytes = w.finish();
            bytes.push(0);
            bytes.push(0); // peek16 look-ahead padding
            let mut r = Sv7BitReader::new(&bytes);
            let got = decode(&mut r, table).unwrap_or_else(|_| panic!("{label}: decode {v}"));
            assert_eq!(got, v, "{label}: round-trip value {v}");
        }
    }

    #[test]
    fn bandtype_header_table_round_trips() {
        assert_table_round_trips(&SV7_BANDTYPE_HEADER_TABLE, "bandtype-header");
    }

    #[test]
    fn scfi_table_round_trips() {
        assert_table_round_trips(&SV7_SCFI_TABLE, "scfi");
    }

    #[test]
    fn dscf_table_round_trips() {
        assert_table_round_trips(&SV7_DSCF_TABLE, "dscf");
    }

    #[test]
    fn quantiser_pair_tables_round_trip_both_contexts() {
        for ctx in 0..=1 {
            assert_table_round_trips(sv7_q1_ctx(ctx), "q1");
            assert_table_round_trips(sv7_q2_ctx(ctx), "q2");
            assert_table_round_trips(sv7_q3_ctx(ctx), "q3");
            assert_table_round_trips(sv7_q4_ctx(ctx), "q4");
            assert_table_round_trips(sv7_q5_ctx(ctx), "q5");
            assert_table_round_trips(sv7_q6_ctx(ctx), "q6");
            assert_table_round_trips(sv7_q7_ctx(ctx), "q7");
        }
    }

    #[test]
    fn back_to_back_symbols_decode_in_order() {
        // Emit a run of several bandtype-header symbols and decode the
        // whole run — the bit alignment across codewords must hold.
        let seq = [0_i8, 1, -1, 2, -3, 0, 4];
        let mut w = Sv7BitWriter::new();
        for &v in &seq {
            write_symbol(&mut w, &SV7_BANDTYPE_HEADER_TABLE, v).unwrap();
        }
        let mut bytes = w.finish();
        bytes.push(0);
        bytes.push(0);
        let mut r = Sv7BitReader::new(&bytes);
        for &v in &seq {
            assert_eq!(decode(&mut r, &SV7_BANDTYPE_HEADER_TABLE).unwrap(), v);
        }
    }

    #[test]
    fn encode_symbol_reports_unencodable() {
        // The scfi table's alphabet is exactly {0,1,2,3}; 9 is absent.
        assert_eq!(
            encode_symbol(&SV7_SCFI_TABLE, 9),
            Err(Error::SymbolNotEncodable(9)),
        );
    }

    #[test]
    fn encode_symbol_returns_a_row_whose_value_matches() {
        let e = encode_symbol(&SV7_DSCF_TABLE, 8).unwrap();
        assert_eq!(e.value, 8);
        // The escape symbol (8) is present in the dscf table.
        assert!(e.length >= 1);
    }
}
