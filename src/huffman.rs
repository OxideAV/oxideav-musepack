//! SV7 entropy-coding Huffman tables.
//!
//! Wires the staged "`mpc_huffman`-shape" decoder tables under
//! `docs/audio/musepack/tables/sv7-huffman-*.csv` into typed Rust
//! constants. The `build.rs` parses each CSV and emits a
//! `Sv7Entry`-typed array per table; this module exposes them, the
//! shared `Sv7Entry` struct, and the bit-stream decoder shape that
//! the staged sidecars describe.
//!
//! Source-of-record:
//!
//! - **Structural prose**: `docs/audio/musepack/musepack-sv7-sv8-spec.md`
//!   * §2.3 band-type loop (table `sv7-huffman-bandtype-header`).
//!   * §2.4 SCF coding-method selector + delta-coded SCF (tables
//!     `sv7-huffman-scfi` and `sv7-huffman-dscf`).
//!   * §2.5 per-quantiser sample VLC switch (tables
//!     `sv7-huffman-q{1..=7}`, each a context-pair).
//! - **Numeric values + decoder convention**: the staged `.meta`
//!   sidecars:
//!   * `value_encoding: mpc_huffman = {Code:uint16 left-adjusted,
//!     Length:uint8, Value:int8}; decoder table sorted by Code descending`.
//!   * `notes: For [2][N] tables the CSV concatenates context 0 then
//!     context 1 (each N rows); split at element_count/2.`
//!
//! The table contents are the staged CSV
//! facts and the bit-stream decoder shape follows directly from the
//! `mpc_huffman` value-encoding sentence above.
//!
//! # Bit-stream convention
//!
//! Each row is `(code, length, value)` where `code` is the canonical
//! code word **left-justified into 16 bits** (i.e. the high `length`
//! bits of `code` carry the literal pattern, the low `16 - length`
//! bits are zero). To decode, the caller peeks 16 bits from the bit
//! stream (MSB-first), then walks the table — sorted by `code`
//! descending — and returns the first entry whose `code <= peek`.
//! The matched entry's `length` is the number of bits to actually
//! consume.
//!
//! This implementation drives that walk through [`Sv7BitReader`], a
//! tiny MSB-first bit reader over an in-memory `&[u8]` slice that
//! supports `peek16` / `consume_bits`. The reader is intentionally
//! standalone here; the larger SV7 bitstream framing (the per-frame
//! 20-bit length prefix and the "read in 32-LSB units" word packing
//! per spec §2.2) is out of scope for this entropy-table wiring and
//! left for a later round.

use crate::{Error, Result};

/// One row of an SV7 `mpc_huffman` decoder table.
///
/// The `code` field is the canonical code word **left-justified into
/// 16 bits**: the high `length` bits carry the literal code, and the
/// low `16 - length` bits are zero. Decoder tables are sorted by
/// `code` descending, so a linear walk hits the first matching entry
/// — see [`decode`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Sv7Entry {
    /// Code word, left-justified into 16 bits (high bits carry the
    /// literal pattern).
    pub code: u16,
    /// Number of bits in the code word (high bits of `code`).
    pub length: u8,
    /// The decoded symbol value, signed 8-bit.
    pub value: i8,
}

include!(concat!(env!("OUT_DIR"), "/sv7_huffman_tables.rs"));

/// Decode one `mpc_huffman` symbol from `reader` using `table`.
///
/// The table must be sorted by `code` descending per the staged
/// `.meta` convention (the generated arrays from `build.rs`
/// preserve the CSV row order, which is already in that shape).
///
/// Returns the decoded `value` and consumes exactly `length` bits
/// from `reader`. Errors:
///
/// - [`Error::UnexpectedEof`] if `reader` has fewer than 16 bits
///   remaining (peeking is always 16 bits per the table's
///   left-justified-to-u16 convention).
/// - [`Error::HuffmanNoMatch`] if no table row matches the peeked
///   16-bit window (a malformed bitstream or wrong table for the
///   current context).
pub fn decode(reader: &mut Sv7BitReader<'_>, table: &[Sv7Entry]) -> Result<i8> {
    let peek = reader.peek16()?;
    for entry in table {
        if entry.code <= peek {
            reader.consume_bits(entry.length)?;
            return Ok(entry.value);
        }
    }
    Err(Error::HuffmanNoMatch)
}

/// MSB-first bit reader over an in-memory byte slice, sized to the
/// SV7 mpc_huffman decoder's needs (16-bit peek window).
///
/// Bits are consumed strictly in the order they appear in the byte
/// stream: the high (`0x80`) bit of `bytes[0]` is the first bit
/// produced. The reader holds an internal 32-bit window so a 16-bit
/// peek is always one shift-and-mask away.
#[derive(Debug, Clone)]
pub struct Sv7BitReader<'a> {
    bytes: &'a [u8],
    /// Number of bytes from `bytes` that have already been latched
    /// into `window`.
    next_byte: usize,
    /// Bits buffered in the high end of a 64-bit register.
    /// `window_bits` says how many high bits of `window` are valid.
    window: u64,
    /// Number of valid bits in `window` (0..=64).
    window_bits: u32,
}

impl<'a> Sv7BitReader<'a> {
    /// Build a reader over `bytes`, with the cursor at bit 0
    /// (the high bit of `bytes[0]`).
    pub fn new(bytes: &'a [u8]) -> Self {
        Self {
            bytes,
            next_byte: 0,
            window: 0,
            window_bits: 0,
        }
    }

    /// Number of bits remaining to be consumed (buffered + still in
    /// the underlying slice).
    pub fn bits_remaining(&self) -> u64 {
        self.window_bits as u64 + ((self.bytes.len() - self.next_byte) as u64) * 8
    }

    /// True once every bit has been consumed.
    pub fn is_empty(&self) -> bool {
        self.bits_remaining() == 0
    }

    /// Peek the next 16 bits MSB-first without consuming them.
    /// Returns [`Error::UnexpectedEof`] if fewer than 16 bits
    /// remain.
    pub fn peek16(&mut self) -> Result<u16> {
        self.fill_to(16)?;
        Ok((self.window >> (64 - 16)) as u16)
    }

    /// Consume `n` bits (1..=32) from the high end of the window.
    /// Returns [`Error::UnexpectedEof`] if fewer than `n` bits
    /// remain. `n == 0` is a no-op.
    pub fn consume_bits(&mut self, n: u8) -> Result<()> {
        if n == 0 {
            return Ok(());
        }
        let n = n as u32;
        if n > 32 {
            // Defensive: the mpc_huffman code lengths are at most
            // 16 bits in the staged tables; anything larger is a
            // caller bug, not a stream condition.
            return Err(Error::HuffmanNoMatch);
        }
        self.fill_to(n)?;
        self.window <<= n;
        self.window_bits -= n;
        Ok(())
    }

    /// Read the next `n` bits (1..=16) MSB-first as a `u16`. Useful
    /// for the spec §2.5 case 8..=17 linear-PCM escape ladder
    /// (`band_type - 1` bits per sample).
    pub fn read_bits(&mut self, n: u8) -> Result<u16> {
        if n == 0 {
            return Ok(0);
        }
        if n > 16 {
            return Err(Error::HuffmanNoMatch);
        }
        let n_u32 = n as u32;
        self.fill_to(n_u32)?;
        let val = (self.window >> (64 - n_u32)) as u16;
        self.window <<= n_u32;
        self.window_bits -= n_u32;
        Ok(val)
    }

    /// Pull bytes from `bytes` into the high end of `window` until
    /// at least `n` bits are buffered. Returns
    /// [`Error::UnexpectedEof`] if the underlying slice runs out.
    fn fill_to(&mut self, n: u32) -> Result<()> {
        while self.window_bits < n {
            if self.next_byte >= self.bytes.len() {
                return Err(Error::UnexpectedEof);
            }
            let byte = self.bytes[self.next_byte];
            self.next_byte += 1;
            // Place `byte` into the highest still-empty 8 bits of
            // `window`: those are the bits at positions
            // `[64 - window_bits - 8, 64 - window_bits)`.
            let shift = 64 - self.window_bits - 8;
            self.window |= (byte as u64) << shift;
            self.window_bits += 8;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ─── Bit reader ─────────────────────────────────────────

    #[test]
    fn bit_reader_msb_first_byte_boundary() {
        // Byte 0xA5 = 1010_0101. MSB-first: 1,0,1,0,0,1,0,1.
        let mut r = Sv7BitReader::new(&[0xA5]);
        assert_eq!(r.read_bits(1).unwrap(), 1);
        assert_eq!(r.read_bits(1).unwrap(), 0);
        assert_eq!(r.read_bits(1).unwrap(), 1);
        assert_eq!(r.read_bits(1).unwrap(), 0);
        assert_eq!(r.read_bits(4).unwrap(), 0b0101);
        assert!(r.is_empty());
    }

    #[test]
    fn bit_reader_peek_does_not_consume() {
        let mut r = Sv7BitReader::new(&[0xFF, 0x00]);
        let p1 = r.peek16().unwrap();
        let p2 = r.peek16().unwrap();
        assert_eq!(p1, 0xFF00);
        assert_eq!(p2, 0xFF00);
        assert_eq!(r.bits_remaining(), 16);
    }

    #[test]
    fn bit_reader_consume_walks_across_bytes() {
        let mut r = Sv7BitReader::new(&[0b1100_0011, 0b1111_0000]);
        // Consume 4 high bits of byte 0, then peek16 — the window
        // should now start at the low nibble of byte 0.
        r.consume_bits(4).unwrap();
        // Remaining bits: 0011_1111_0000 (high to low) followed by
        // EOF-padding zeros once we ask for more. peek16 needs 16
        // bits, but we only have 12; should EOF.
        assert!(matches!(r.peek16(), Err(Error::UnexpectedEof)));
        assert_eq!(r.read_bits(8).unwrap(), 0b0011_1111);
        assert_eq!(r.read_bits(4).unwrap(), 0);
    }

    #[test]
    fn bit_reader_eof_on_short_input() {
        let mut r = Sv7BitReader::new(&[0xAA]);
        // peek16 needs 16 bits, only 8 are available.
        assert!(matches!(r.peek16(), Err(Error::UnexpectedEof)));
    }

    // ─── Table shape (entry count + last entry) ─────────────

    #[test]
    fn sv7_bandtype_header_table_shape() {
        // .meta resolved_dims: [10]
        assert_eq!(SV7_BANDTYPE_HEADER_TABLE.len(), 10);
        // Last entry per the staged CSV.
        assert_eq!(
            *SV7_BANDTYPE_HEADER_TABLE.last().unwrap(),
            Sv7Entry {
                code: 0x0000,
                length: 2,
                value: -1
            },
        );
    }

    #[test]
    fn sv7_scfi_table_shape() {
        // .meta resolved_dims: [4]
        assert_eq!(SV7_SCFI_TABLE.len(), 4);
        assert_eq!(
            *SV7_SCFI_TABLE.last().unwrap(),
            Sv7Entry {
                code: 0x0000,
                length: 2,
                value: 3
            },
        );
    }

    #[test]
    fn sv7_dscf_table_shape() {
        // .meta resolved_dims: [16]
        assert_eq!(SV7_DSCF_TABLE.len(), 16);
        assert_eq!(
            *SV7_DSCF_TABLE.last().unwrap(),
            Sv7Entry {
                code: 0x0000,
                length: 3,
                value: -2
            },
        );
    }

    #[test]
    fn sv7_quantiser_pair_tables_shape() {
        // .meta resolved_dims values for Q1..=Q7.
        assert_eq!(SV7_Q1_TABLE.len(), 54);
        assert_eq!(SV7_Q2_TABLE.len(), 50);
        assert_eq!(SV7_Q3_TABLE.len(), 14);
        assert_eq!(SV7_Q4_TABLE.len(), 18);
        assert_eq!(SV7_Q5_TABLE.len(), 30);
        assert_eq!(SV7_Q6_TABLE.len(), 62);
        assert_eq!(SV7_Q7_TABLE.len(), 126);

        // Last entries per CSV.
        assert_eq!(
            *SV7_Q1_TABLE.last().unwrap(),
            Sv7Entry {
                code: 0x0000,
                length: 4,
                value: 10
            },
        );
        assert_eq!(
            *SV7_Q7_TABLE.last().unwrap(),
            Sv7Entry {
                code: 0x0000,
                length: 4,
                value: 1
            },
        );
    }

    #[test]
    fn sv7_context_pair_splits_at_half() {
        // Q1 is [2][27] -> 54 rows, ctx 0 = first 27, ctx 1 = last 27.
        assert_eq!(sv7_q1_ctx(0).len(), 27);
        assert_eq!(sv7_q1_ctx(1).len(), 27);
        // First row of ctx 0 should match the first CSV row of Q1.
        assert_eq!(
            sv7_q1_ctx(0)[0],
            Sv7Entry {
                code: 0xE000,
                length: 3,
                value: 13
            },
        );
        // First row of ctx 1 should be the 28th CSV row of Q1
        // (`0x8000,1,13`).
        assert_eq!(
            sv7_q1_ctx(1)[0],
            Sv7Entry {
                code: 0x8000,
                length: 1,
                value: 13
            },
        );
    }

    // ─── Decoder ────────────────────────────────────────────

    #[test]
    fn decode_bandtype_header_value_zero() {
        // bandtype-header table row 0: code 0x8000, length 1, value 0.
        // A single MSB '1' bit decodes to value 0.
        // 0b1000_0000 = 0x80.
        let mut r = Sv7BitReader::new(&[0x80, 0x00]);
        let v = decode(&mut r, &SV7_BANDTYPE_HEADER_TABLE).unwrap();
        assert_eq!(v, 0);
        // Length 1 consumed -> 15 bits left.
        assert_eq!(r.bits_remaining(), 15);
    }

    #[test]
    fn decode_bandtype_header_value_minus_one() {
        // Last row: code 0x0000, length 2, value -1.
        // Bits 00 followed by any tail decodes to -1.
        let mut r = Sv7BitReader::new(&[0x00, 0x00]);
        let v = decode(&mut r, &SV7_BANDTYPE_HEADER_TABLE).unwrap();
        assert_eq!(v, -1);
        assert_eq!(r.bits_remaining(), 14);
    }

    #[test]
    fn decode_bandtype_header_value_two() {
        // Row: code 0x5800, length 6, value 2. Bits 010110...
        // 0b0101_1000 = 0x58.
        let mut r = Sv7BitReader::new(&[0x58, 0x00]);
        let v = decode(&mut r, &SV7_BANDTYPE_HEADER_TABLE).unwrap();
        assert_eq!(v, 2);
        assert_eq!(r.bits_remaining(), 10);
    }

    #[test]
    fn decode_scfi_three() {
        // sv7-huffman-scfi last row: code 0x0000, length 2, value 3.
        let mut r = Sv7BitReader::new(&[0x00, 0xFF]);
        let v = decode(&mut r, &SV7_SCFI_TABLE).unwrap();
        assert_eq!(v, 3);
    }

    #[test]
    fn decode_dscf_back_to_back_two_symbols() {
        // First sym: code 0xa000, length 3, value 1 -> bits 101.
        // Second sym: code 0x4000, length 3, value 2 -> bits 010.
        // Packed: 101_010_00 = 0b1010_1000 = 0xA8. Pad with two
        // trailing zero bytes so the second decode's 16-bit peek
        // always has bits available (the table tail row 0x0000,3,-2
        // matches any all-zero peek tail, which is fine for the
        // round-trip check).
        let mut r = Sv7BitReader::new(&[0xA8, 0x00, 0x00]);
        let v1 = decode(&mut r, &SV7_DSCF_TABLE).unwrap();
        let v2 = decode(&mut r, &SV7_DSCF_TABLE).unwrap();
        assert_eq!((v1, v2), (1, 2));
    }

    #[test]
    fn decode_eof_on_empty_stream() {
        let mut r = Sv7BitReader::new(&[]);
        assert!(matches!(
            decode(&mut r, &SV7_BANDTYPE_HEADER_TABLE),
            Err(Error::UnexpectedEof),
        ));
    }
}
