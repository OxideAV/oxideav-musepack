//! Canonical-Huffman VLC reader for Musepack tables.
//!
//! SV7 tables are stored inline as `(symbol, length)` lists; SV8 tables
//! are stored as a length-count histogram (`L[1..=16]`) plus a separate
//! symbol-order array, JPEG-style. Either representation can be turned
//! into a flat `(code, length, symbol)` table by assigning canonical
//! codes (group-by-length, then by listed-order, MSB-first).
//!
//! Decoding is bit-by-bit with early termination once the running
//! prefix matches a stored code at its corresponding length. Tables are
//! small (≤ 256 entries × ≤ 16 bits) so a serial walk is fast enough
//! and avoids the complexity of a multi-level lookup.

use oxideav_core::bits::BitReader;
use oxideav_core::{Error, Result};

/// One Huffman entry: a `length`-bit `code` (MSB-aligned within a
/// `u32`) decodes to `symbol`. Codes are stored sorted by length then
/// by canonical order.
#[derive(Clone, Copy, Debug)]
pub struct HuffEntry {
    /// Code bits, MSB-aligned in the low `length` bits. Equivalent to
    /// the canonical-Huffman code's bit pattern read MSB-first.
    pub code: u32,
    /// Code length in bits, in `[1..=16]`.
    pub length: u8,
    /// Decoded symbol value.
    pub symbol: i32,
}

/// A canonical-Huffman table built from either the SV7 inline form or
/// the SV8 length-count + symbol-order form. Stored as a flat list of
/// `(code, length, symbol)` triples sorted by `(length, code)`.
#[derive(Clone, Debug)]
pub struct VlcTable {
    pub entries: Vec<HuffEntry>,
    pub max_length: u8,
}

impl VlcTable {
    /// Build a VLC from the SV7 inline `[(symbol, length)]` form.
    /// Codes are assigned canonically — for each length in increasing
    /// order, symbols listed at that length get the next sequential
    /// code value, with the cursor `(prev_code + 1) << 1` between
    /// lengths.
    pub fn from_sv7(symbol_lengths: &[(i32, u8)]) -> Self {
        let mut entries: Vec<HuffEntry> = Vec::with_capacity(symbol_lengths.len());
        let mut max_length = 0u8;
        // Group symbols by length, preserving inline order within each
        // length.
        let mut by_length: Vec<Vec<i32>> = vec![Vec::new(); 17];
        for &(sym, len) in symbol_lengths {
            assert!(
                len >= 1 && len <= 16,
                "VLC length out of range 1..=16: {len}"
            );
            by_length[len as usize].push(sym);
            if len > max_length {
                max_length = len;
            }
        }
        let mut code: u32 = 0;
        for len in 1..=16 {
            for &sym in &by_length[len] {
                entries.push(HuffEntry {
                    code,
                    length: len as u8,
                    symbol: sym,
                });
                code += 1;
            }
            code <<= 1;
        }
        VlcTable {
            entries,
            max_length,
        }
    }

    /// Build a VLC from the SV8 JPEG-style form: `length_counts[i]` is
    /// the number of codes of length `i + 1`; `symbols` lists the
    /// decoded symbols in canonical order (length-first, then listed
    /// order within each length).
    pub fn from_sv8(length_counts: &[u8; 16], symbols: &[i32]) -> Self {
        let mut entries: Vec<HuffEntry> = Vec::with_capacity(symbols.len());
        let mut max_length = 0u8;
        let mut code: u32 = 0;
        let mut sym_cursor = 0usize;
        for (idx, &count) in length_counts.iter().enumerate() {
            let len = (idx + 1) as u8;
            for _ in 0..count {
                let sym = symbols[sym_cursor];
                sym_cursor += 1;
                entries.push(HuffEntry {
                    code,
                    length: len,
                    symbol: sym,
                });
                code += 1;
                if count > 0 {
                    max_length = len;
                }
            }
            code <<= 1;
        }
        debug_assert_eq!(
            sym_cursor,
            symbols.len(),
            "symbol count mismatch with length_counts"
        );
        VlcTable {
            entries,
            max_length,
        }
    }

    /// Decode one symbol from `br`. Reads up to `max_length` bits.
    /// Returns `Error::Invalid` if the running prefix never matches an
    /// entry (corrupt stream).
    pub fn read(&self, br: &mut BitReader<'_>) -> Result<i32> {
        let mut code: u32 = 0;
        let mut len: u8 = 0;
        // Bit-by-bit walk. For tables we use here (≤ 256 entries × ≤
        // 16 bits) this is plenty fast and avoids any indexed-table
        // build-up cost.
        while len < self.max_length {
            code = (code << 1) | br.read_u32(1)?;
            len += 1;
            // Linear scan through entries of this length — they are
            // contiguous in the entries vector, so we can early-out on
            // the first length > len.
            for entry in &self.entries {
                if entry.length == len && entry.code == code {
                    return Ok(entry.symbol);
                }
                if entry.length > len {
                    break;
                }
            }
        }
        Err(Error::invalid("musepack VLC: unmatched code prefix"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sv7_scfi_table() {
        // SV7 SCFI: symbol 1 → 1 bit, symbol 3 → 2 bits, symbols 0 and
        // 2 → 3 bits (vlc-tables.md §1.1). Canonical assignment:
        //   sym 1 = '0' (1 bit)
        //   sym 3 = '10' (2 bits)
        //   sym 0 = '110' (3 bits)
        //   sym 2 = '111' (3 bits)
        let table = VlcTable::from_sv7(&[(1, 1), (3, 2), (0, 3), (2, 3)]);
        assert_eq!(table.entries.len(), 4);
        assert_eq!(table.max_length, 3);

        // Decode each canonical code.
        let cases: [(u8, i32); 4] = [
            (0b0_0000000, 1), // '0'
            (0b10_000000, 3), // '10'
            (0b110_00000, 0), // '110'
            (0b111_00000, 2), // '111'
        ];
        for (byte, expected) in cases {
            let data = [byte];
            let mut br = BitReader::new(&data);
            let got = table.read(&mut br).unwrap();
            assert_eq!(got, expected, "byte = {byte:#010b}");
        }
    }

    #[test]
    #[allow(clippy::unusual_byte_groupings)]
    fn sv8_jpeg_form() {
        // 4-symbol example: length_counts {1, 1, 2} (= 1 one-bit, 1
        // two-bit, 2 three-bit codes). Symbols listed in canonical
        // order = {0, 1, 3, 2}.
        let counts = [1u8, 1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        let symbols = [0, 1, 3, 2];
        let table = VlcTable::from_sv8(&counts, &symbols);
        assert_eq!(table.entries.len(), 4);
        // Same canonical layout as the SCFI test above (symbols
        // re-mapped). Code '0' → 0, '10' → 1, '110' → 3, '111' → 2.
        let data = [0b0_10_110_11u8, 0b1_0000000];
        let mut br = BitReader::new(&data);
        assert_eq!(table.read(&mut br).unwrap(), 0);
        assert_eq!(table.read(&mut br).unwrap(), 1);
        assert_eq!(table.read(&mut br).unwrap(), 3);
        assert_eq!(table.read(&mut br).unwrap(), 2);
    }
}
