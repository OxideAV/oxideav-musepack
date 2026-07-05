//! SV7 MSB-first bit writer — the exact inverse of
//! [`crate::huffman::Sv7BitReader`].
//!
//! The SV7 frame body is a continuous, non-byte-aligned bit run consumed
//! **most-significant bit first** (`docs/audio/musepack/spec/musepack-headers-and-coding.md`
//! §4; the structural spec §2.2). [`Sv7BitReader`](crate::huffman::Sv7BitReader)
//! reads that run; this module *produces* it. Together they let the crate
//! grow an encode side that round-trips every decode path bit-for-bit
//! against the reader that already exists.
//!
//! # Bit order (matches the reader exactly)
//!
//! - [`Sv7BitWriter::write_bits`] appends the low `n` bits of a value
//!   **MSB-first** (bit `n-1` first, bit `0` last). This is the exact
//!   inverse of [`Sv7BitReader::read_bits`](crate::huffman::Sv7BitReader::read_bits),
//!   which returns the next `n` bits right-justified. Writing `value`
//!   with width `n` then reading `n` bits back yields `value` unchanged.
//! - Huffman codewords are stored **left-justified in 16 bits** in the
//!   `mpc_huffman` tables ([`crate::huffman::Sv7Entry`]): the high
//!   `length` bits carry the literal pattern. To emit such a codeword,
//!   pass the high `length` bits as a right-justified value, i.e.
//!   `write_bits(code >> (16 - length), length)`. The
//!   [`crate::sv7_huffman_encode`] layer wraps that shift.
//!
//! # No word-swap here
//!
//! This writer emits the **logical** post-word-swap bit run — the byte
//! order the reader walks. The §4 32-bit-word body byte-swap (SV7 stores
//! the run in little-endian 32-bit word units) is a separate, involutive
//! transform already wired as [`crate::sv7_word_swap::word_swap_sv7_body`];
//! apply it to this writer's output to obtain the raw on-disk body order.
//! SV8 needs no swap (§4).

/// An MSB-first bit writer that accumulates a non-byte-aligned bit run
/// into a `Vec<u8>`, the exact inverse of
/// [`crate::huffman::Sv7BitReader`].
///
/// Bits are buffered in the low end of a 64-bit accumulator and flushed
/// to `bytes` a full byte at a time (high bits first). A trailing
/// partial byte is emitted only at [`Sv7BitWriter::finish`], zero-padded
/// in its low bits — the same padding a real SV7 body carries before the
/// next frame or the stream trailer.
#[derive(Debug, Clone, Default)]
pub struct Sv7BitWriter {
    bytes: Vec<u8>,
    /// The `nbits` least-significant bits of `acc` hold the not-yet-
    /// flushed tail of the run, in write order (MSB-first).
    acc: u64,
    /// Number of valid buffered bits in `acc` (`0..8` between calls).
    nbits: u32,
}

impl Sv7BitWriter {
    /// A fresh writer with an empty run.
    pub fn new() -> Self {
        Self::default()
    }

    /// Append the low `n` bits of `value` **MSB-first** (`n` in
    /// `0..=32`; `n == 0` is a no-op). Bit `n-1` of `value` is written
    /// first, bit `0` last — the exact inverse of
    /// [`Sv7BitReader::read_bits`](crate::huffman::Sv7BitReader::read_bits).
    ///
    /// Bits of `value` above bit `n-1` are ignored (masked off), so a
    /// caller may pass a wider value and rely on `n` to select the low
    /// field.
    ///
    /// # Panics
    ///
    /// Panics if `n > 32` — the SV7 bitstream never writes a single
    /// field wider than the 32-bit header quantities, and those are
    /// assembled from narrower writes, so a wider request is a caller
    /// bug rather than a stream condition.
    pub fn write_bits(&mut self, value: u32, n: u8) {
        assert!(n <= 32, "Sv7BitWriter::write_bits width {n} exceeds 32");
        let n = n as u32;
        if n == 0 {
            return;
        }
        // Mask `value` to its low `n` bits, then shift it into the low
        // end of the accumulator.
        let masked = if n == 32 {
            value as u64
        } else {
            (value as u64) & ((1u64 << n) - 1)
        };
        self.acc = (self.acc << n) | masked;
        self.nbits += n;
        // Flush whole bytes from the high end of the buffered bits.
        while self.nbits >= 8 {
            self.nbits -= 8;
            let byte = (self.acc >> self.nbits) as u8;
            self.bytes.push(byte);
        }
        // Retain only the still-buffered low `nbits`.
        self.acc &= if self.nbits == 0 {
            0
        } else {
            (1u64 << self.nbits) - 1
        };
    }

    /// Append a Huffman codeword stored **left-justified in 16 bits**:
    /// the high `length` bits of `code` carry the literal pattern (the
    /// [`crate::huffman::Sv7Entry`] convention). Equivalent to
    /// `write_bits(code >> (16 - length), length)`.
    ///
    /// # Panics
    ///
    /// Panics if `length == 0` or `length > 16` — a valid `mpc_huffman`
    /// entry always carries a `1..=16`-bit code.
    pub fn write_left_justified(&mut self, code: u16, length: u8) {
        assert!(
            (1..=16).contains(&length),
            "Sv7BitWriter::write_left_justified length {length} out of 1..=16",
        );
        let value = (code as u32) >> (16 - length as u32);
        self.write_bits(value, length);
    }

    /// Number of bits written so far (flushed bytes × 8 plus the
    /// buffered tail). Handy for frame-length bookkeeping.
    pub fn bit_len(&self) -> u64 {
        self.bytes.len() as u64 * 8 + self.nbits as u64
    }

    /// True until the first bit is written.
    pub fn is_empty(&self) -> bool {
        self.bit_len() == 0
    }

    /// Append another writer's entire bit run to this one, preserving
    /// its exact (possibly non-byte-aligned) length. Used by the
    /// whole-file composer to emit a frame body *after* its 20-bit
    /// bit-length prefix (the body must be assembled first so its
    /// length is known).
    pub fn append(&mut self, other: &Sv7BitWriter) {
        for &byte in &other.bytes {
            self.write_bits(u32::from(byte), 8);
        }
        if other.nbits > 0 {
            self.write_bits(other.acc as u32, other.nbits as u8);
        }
    }

    /// Finish the run and return its bytes. A trailing partial byte is
    /// zero-padded in its low bits (the padding a real SV7 body carries
    /// before the next frame / trailer). Consumes the writer.
    pub fn finish(mut self) -> Vec<u8> {
        if self.nbits > 0 {
            let byte = (self.acc << (8 - self.nbits)) as u8;
            self.bytes.push(byte);
            self.acc = 0;
            self.nbits = 0;
        }
        self.bytes
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::huffman::Sv7BitReader;

    #[test]
    fn single_bits_round_trip_msb_first() {
        // Write 1,0,1,0,0,1,0,1 -> byte 0xA5.
        let mut w = Sv7BitWriter::new();
        for &b in &[1u32, 0, 1, 0, 0, 1, 0, 1] {
            w.write_bits(b, 1);
        }
        let bytes = w.finish();
        assert_eq!(bytes, vec![0xA5]);
    }

    #[test]
    fn write_bits_is_inverse_of_read_bits() {
        // A mixed sequence of widths and values; read them back with the
        // reader and confirm each field survives.
        let fields: &[(u32, u8)] = &[
            (0b101, 3),
            (0, 1),
            (0xABCD, 16),
            (0b1, 1),
            (0x7F, 7),
            (0x0, 4),
            (0xFFFF, 16),
            (0b1001, 4),
        ];
        let mut w = Sv7BitWriter::new();
        for &(v, n) in fields {
            w.write_bits(v, n);
        }
        let bytes = w.finish();
        let mut r = Sv7BitReader::new(&bytes);
        for &(v, n) in fields {
            let masked = if n == 32 { v } else { v & ((1u32 << n) - 1) };
            assert_eq!(r.read_bits(n).unwrap() as u32, masked, "field ({v:#x},{n})");
        }
    }

    #[test]
    fn thirtytwo_bit_field_reads_back_as_two_halves() {
        // The header frame-count / CRC quantities are 32 bits, assembled
        // from two 16-bit reads on the decode side.
        let mut w = Sv7BitWriter::new();
        w.write_bits(0x1234_5678, 32);
        let bytes = w.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let hi = r.read_bits(16).unwrap();
        let lo = r.read_bits(16).unwrap();
        assert_eq!(((hi as u32) << 16) | lo as u32, 0x1234_5678);
    }

    #[test]
    fn write_bits_masks_high_bits() {
        // Passing a value wider than `n` writes only the low `n` bits.
        let mut w = Sv7BitWriter::new();
        w.write_bits(0xFF, 4); // only 0xF written
        w.write_bits(0xF0, 4); // only 0x0 written
        let bytes = w.finish();
        assert_eq!(bytes, vec![0xF0]);
    }

    #[test]
    fn write_bits_zero_width_is_noop() {
        let mut w = Sv7BitWriter::new();
        w.write_bits(0xFFFF_FFFF, 0);
        assert!(w.is_empty());
        assert_eq!(w.finish(), Vec::<u8>::new());
    }

    #[test]
    fn left_justified_codeword_round_trips_through_huffman_decode() {
        use crate::huffman::{decode, SV7_BANDTYPE_HEADER_TABLE};
        // Emit the bandtype-header codeword for value 2 (code 0x5800,
        // length 6) then decode it back.
        let entry = SV7_BANDTYPE_HEADER_TABLE
            .iter()
            .find(|e| e.value == 2)
            .copied()
            .unwrap();
        let mut w = Sv7BitWriter::new();
        w.write_left_justified(entry.code, entry.length);
        let mut bytes = w.finish();
        bytes.push(0);
        bytes.push(0); // peek16 padding
        let mut r = Sv7BitReader::new(&bytes);
        assert_eq!(decode(&mut r, &SV7_BANDTYPE_HEADER_TABLE).unwrap(), 2);
    }

    #[test]
    fn bit_len_tracks_written_bits() {
        let mut w = Sv7BitWriter::new();
        assert_eq!(w.bit_len(), 0);
        w.write_bits(0, 3);
        assert_eq!(w.bit_len(), 3);
        w.write_bits(0, 7);
        assert_eq!(w.bit_len(), 10);
        w.write_bits(0, 6);
        assert_eq!(w.bit_len(), 16);
    }

    #[test]
    fn partial_final_byte_is_zero_padded_low() {
        // Write 3 bits '101' -> final byte 0b1010_0000 = 0xA0.
        let mut w = Sv7BitWriter::new();
        w.write_bits(0b101, 3);
        assert_eq!(w.finish(), vec![0xA0]);
    }

    #[test]
    #[should_panic]
    fn write_bits_rejects_width_above_32() {
        let mut w = Sv7BitWriter::new();
        w.write_bits(0, 33);
    }

    #[test]
    fn append_preserves_non_aligned_runs() {
        // 3 bits + 13 bits appended onto 5 bits: identical to writing
        // the whole sequence into one writer.
        let mut inner = Sv7BitWriter::new();
        inner.write_bits(0b101, 3);
        inner.write_bits(0x1ABC & 0x1FFF, 13);

        let mut outer = Sv7BitWriter::new();
        outer.write_bits(0b10011, 5);
        outer.append(&inner);
        assert_eq!(outer.bit_len(), 5 + 16);

        let mut direct = Sv7BitWriter::new();
        direct.write_bits(0b10011, 5);
        direct.write_bits(0b101, 3);
        direct.write_bits(0x1ABC & 0x1FFF, 13);
        assert_eq!(outer.finish(), direct.finish());
    }
}
