//! SV7 32-bit-word byte-swap for the body bit reader
//! (`spec/musepack-headers-and-coding.md` §4).
//!
//! The SV7 audio body is a continuous, non-byte-aligned MSB-first bit
//! run, but the bytes are **not** laid out in the natural order the
//! [`crate::huffman::Sv7BitReader`] walks. §4 records the historic
//! "read in 32-LSB units" packing: bytes are loaded in **32-bit
//! little-endian word units with an in-place byte-swap** — each aligned
//! 4-byte group is reversed before the bit reader sees it. (SV8 dropped
//! this: its bytes are loaded in natural order — see §4 second bullet —
//! so SV8 needs no analogue of this module.)
//!
//! This module is the one transform that turns a raw SV7 body byte
//! buffer into the word-swapped buffer the bit reader consumes, so a
//! stream driver can take *raw* SV7 bytes rather than a caller that has
//! already done the swap by hand. It is pure byte rearrangement — no
//! bitstream interpretation, no new format facts beyond §4.
//!
//! ## The transform
//!
//! For an input split into aligned 4-byte groups
//! `[b0 b1 b2 b3] [b4 b5 b6 b7] …`, each group is emitted reversed:
//! `[b3 b2 b1 b0] [b7 b6 b5 b4] …`. This is exactly the byte order of
//! the same 4 bytes interpreted as a little-endian 32-bit word and then
//! re-serialised big-endian (so that the bit reader's MSB-first walk of
//! the swapped buffer visits the word's bits from bit 31 down to bit 0).
//!
//! ## Trailing partial word
//!
//! A raw body whose length is not a multiple of 4 has a final group of
//! 1, 2, or 3 bytes. §4 describes the swap over *aligned* 4-byte groups;
//! the historic word-oriented buffer is filled a full 32-bit word at a
//! time, so a partial trailing group is zero-extended up to four bytes
//! **before** the reversal, then the reversed four bytes are emitted.
//! Concretely a trailing `[b0]` becomes `[00 00 00 b0]`, `[b0 b1]`
//! becomes `[00 00 b1 b0]`, `[b0 b1 b2]` becomes `[00 b2 b1 b0]`. This
//! keeps every real body byte at its word-swapped position and pads the
//! word's high (later-consumed) end with the zero bits a word-aligned
//! reader would have seen.
//!
//! The padding only adds zero bits *after* every coded bit of the
//! partial word, so it never disturbs a code that ends inside the last
//! real byte; a decoder that stops at its declared sample budget never
//! reaches the padding.

/// Word-swap a raw SV7 body buffer into the byte order the
/// [`crate::huffman::Sv7BitReader`] expects, per §4.
///
/// Each aligned 4-byte group of `raw` is reversed. A trailing partial
/// group (1..=3 bytes) is zero-extended to four bytes before reversal,
/// so the output length is always a multiple of 4 and every real input
/// byte lands at its word-swapped position (see the module docs for the
/// trailing-word convention).
///
/// `raw.len() == 0` returns an empty buffer.
pub fn word_swap_sv7_body(raw: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(raw.len().div_ceil(4) * 4);
    for chunk in raw.chunks(4) {
        // Build the four word bytes, zero-extending a short trailing
        // group, then emit them reversed (little-endian word ->
        // big-endian byte order for the MSB-first reader).
        let mut word = [0u8; 4];
        word[..chunk.len()].copy_from_slice(chunk);
        word.reverse();
        out.extend_from_slice(&word);
    }
    out
}

/// In-place variant of [`word_swap_sv7_body`] for a buffer that is
/// already a whole number of 32-bit words (`buf.len() % 4 == 0`).
///
/// Reverses each aligned 4-byte group of `buf`. Returns `false` without
/// modifying `buf` when `buf.len()` is not a multiple of 4 (a partial
/// trailing word cannot be swapped in place without growing the buffer —
/// use [`word_swap_sv7_body`] for that case).
pub fn word_swap_sv7_body_in_place(buf: &mut [u8]) -> bool {
    if buf.len() % 4 != 0 {
        return false;
    }
    for word in buf.chunks_exact_mut(4) {
        word.reverse();
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_input_is_empty_output() {
        assert!(word_swap_sv7_body(&[]).is_empty());
    }

    #[test]
    fn single_aligned_word_reverses() {
        // One 32-bit word: [00 01 02 03] -> [03 02 01 00].
        assert_eq!(word_swap_sv7_body(&[0, 1, 2, 3]), vec![3, 2, 1, 0]);
    }

    #[test]
    fn two_aligned_words_reverse_independently() {
        let raw = [0, 1, 2, 3, 4, 5, 6, 7];
        assert_eq!(word_swap_sv7_body(&raw), vec![3, 2, 1, 0, 7, 6, 5, 4],);
    }

    #[test]
    fn trailing_one_byte_zero_extends_high() {
        // [b0] -> word [b0 00 00 00] reversed -> [00 00 00 b0].
        assert_eq!(word_swap_sv7_body(&[0xAB]), vec![0x00, 0x00, 0x00, 0xAB]);
    }

    #[test]
    fn trailing_two_bytes_zero_extend_high() {
        // [b0 b1] -> [b0 b1 00 00] reversed -> [00 00 b1 b0].
        assert_eq!(
            word_swap_sv7_body(&[0xAB, 0xCD]),
            vec![0x00, 0x00, 0xCD, 0xAB],
        );
    }

    #[test]
    fn trailing_three_bytes_zero_extend_high() {
        // [b0 b1 b2] -> [b0 b1 b2 00] reversed -> [00 b2 b1 b0].
        assert_eq!(
            word_swap_sv7_body(&[0xAB, 0xCD, 0xEF]),
            vec![0x00, 0xEF, 0xCD, 0xAB],
        );
    }

    #[test]
    fn aligned_plus_partial() {
        // One full word then a 2-byte tail.
        let raw = [1, 2, 3, 4, 5, 6];
        assert_eq!(word_swap_sv7_body(&raw), vec![4, 3, 2, 1, 0, 0, 6, 5],);
    }

    #[test]
    fn output_length_is_word_padded() {
        for len in 0..=16usize {
            let raw: Vec<u8> = (0..len as u8).collect();
            let out = word_swap_sv7_body(&raw);
            assert_eq!(out.len(), len.div_ceil(4) * 4, "len {len}");
        }
    }

    #[test]
    fn in_place_matches_allocating_for_aligned() {
        let raw = [10u8, 20, 30, 40, 50, 60, 70, 80];
        let mut buf = raw;
        assert!(word_swap_sv7_body_in_place(&mut buf));
        assert_eq!(buf.to_vec(), word_swap_sv7_body(&raw));
    }

    #[test]
    fn in_place_rejects_unaligned() {
        let mut buf = [1u8, 2, 3];
        assert!(!word_swap_sv7_body_in_place(&mut buf));
        // Buffer left untouched.
        assert_eq!(buf, [1, 2, 3]);
    }

    #[test]
    fn double_swap_is_identity_for_aligned() {
        let raw = [9u8, 8, 7, 6, 5, 4, 3, 2];
        let once = word_swap_sv7_body(&raw);
        let twice = word_swap_sv7_body(&once);
        assert_eq!(twice, raw.to_vec());
    }

    #[test]
    fn swapped_word_is_big_endian_of_le_word() {
        // The reversal is exactly: read 4 bytes as LE u32, write BE.
        let raw = [0x78u8, 0x56, 0x34, 0x12];
        let le = u32::from_le_bytes(raw);
        assert_eq!(word_swap_sv7_body(&raw), le.to_be_bytes().to_vec());
    }
}
