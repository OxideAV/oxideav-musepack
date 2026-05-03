//! Musepack SV8 variable-length integer encodings.
//!
//! SV8 uses **two distinct** base-128-big-endian variable-length integer
//! forms for the same conceptual integer (this is one of the subtler
//! pitfalls of the format — see the trace report §3.2.3 / §5.11):
//!
//! * **Byte-form** — used for chunk sizes and for the `total_samples` /
//!   `silence_samples` fields of the `SH` chunk. Each byte contributes
//!   7 data bits (top bit `0x80` is a continuation flag); decoded
//!   big-endian. The trace report's text mentions a
//!   "reserved-byte-count subtraction", but verification against an
//!   actual `sv8-notags.mpc` confirms the on-disk values decode
//!   **without** any post-subtraction — chunk-size byte `0x0F`
//!   decodes to 15 (= chunk's total bytes including tag), and the
//!   `total_samples` byte stream `81 89 87 44` decodes to 2,245,572
//!   exactly. The "byte-count" mention in the writeup applies on the
//!   encoder side (the encoder picks the shortest of N representations
//!   that fits a given value), not on decode.
//!
//! * **Bit-form** — used inside the `ST` seek-table bit-stream and
//!   inside the audio bitstream. Identical to the byte form except
//!   that bits are read directly from a [`BitReader`] rather than
//!   via byte boundaries.
//!
//! Both forms are unsigned. The byte-form parser returns the consumed
//! byte count alongside the value so callers can advance the slice.
//!
//! Reference: `docs/audio/musepack/musepack-trace-reverse-engineering.md`
//! §3.2.1 (chunk-size encoding) and §3.2.3 (`total_samples`).

use oxideav_core::bits::BitReader;
use oxideav_core::{Error, Result};

/// Decode the byte-form var-len integer at `data[0..]`. Returns
/// `(value, bytes_consumed)` on success. The decoder reads up to nine
/// bytes (sufficient for a 63-bit value); on EOF or overflow the call
/// returns `Error::Invalid`.
pub fn read_byte_varlen(data: &[u8]) -> Result<(u64, usize)> {
    let mut value: u64 = 0;
    let mut consumed: usize = 0;
    for &b in data.iter().take(9) {
        consumed += 1;
        // Pre-shift then OR so the **first** byte sits in the **highest**
        // 7-bit window (big-endian byte order).
        value = value
            .checked_shl(7)
            .ok_or_else(|| Error::invalid("musepack varlen: shift overflow"))?
            | u64::from(b & 0x7F);
        if b & 0x80 == 0 {
            return Ok((value, consumed));
        }
    }
    Err(Error::invalid(
        "musepack varlen: missing terminator byte (or value too large)",
    ))
}

/// Decode the bit-form var-len integer from a [`BitReader`]. **No**
/// byte-count subtraction is applied (this is the on-the-wire difference
/// from [`read_byte_varlen`]).
pub fn read_bit_varlen(br: &mut BitReader<'_>) -> Result<u64> {
    let mut value: u64 = 0;
    for _ in 0..9 {
        let b = br.read_u32(8)? as u64;
        value = value
            .checked_shl(7)
            .ok_or_else(|| Error::invalid("musepack varlen: shift overflow"))?
            | (b & 0x7F);
        if b & 0x80 == 0 {
            return Ok(value);
        }
    }
    Err(Error::invalid(
        "musepack varlen: missing terminator byte (or value too large)",
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn byte_varlen_one_byte() {
        // 0x00: high bit clear → final byte, value = 0.
        assert_eq!(read_byte_varlen(&[0x00]).unwrap(), (0, 1));
        // 0x0F: chunk-size of an SH chunk in a real file (offset 6 of
        // sv8-notags.mpc) — decodes to 15 = the chunk's total bytes.
        assert_eq!(read_byte_varlen(&[0x0F]).unwrap(), (15, 1));
    }

    #[test]
    fn byte_varlen_total_samples_real_file() {
        // SH.total_samples in sv8-notags.mpc: bytes `81 89 87 44` →
        // 2,245,572 (verified against the file's offset-7 byte stream
        // and ffprobe's reported duration of ~50.92 s @ 44.1 kHz).
        let bytes = [0x81u8, 0x89, 0x87, 0x44];
        let (got, consumed) = read_byte_varlen(&bytes).unwrap();
        assert_eq!(consumed, 4);
        assert_eq!(got, 2_245_572);
    }

    #[test]
    fn bit_varlen_round_trip() {
        // 0x07 (single byte, no continuation) decoded via bit-reader.
        let data = [0x07u8];
        let mut br = BitReader::new(&data);
        assert_eq!(read_bit_varlen(&mut br).unwrap(), 7);
    }
}
