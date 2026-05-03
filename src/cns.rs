//! Combinatorial-number-system (CNS) helpers — SV8 only.
//!
//! SV8 codes the **MS-stereo bitmask** and the **`res = 1` half-band
//! binary spike map** as integers in `[0, C(n, k))`, using a bijection
//! between `k`-subsets of an `n`-element set and the natural numbers.
//! The decoder reads two pieces:
//!
//! 1. A **modified-Golomb** integer `t` in `[0..n]` giving the
//!    population count (subset size). At `n = 33` the modified-Golomb
//!    is the same near-uniform code as `mpc8_dec_base(1, 33)`.
//!
//! 2. The **subset index** itself, decoded by walking down `(k, n)` of
//!    the Pascal grid one step at a time. After reading the bit at each
//!    step we know whether the next item is "in" the subset or not.
//!
//! When `2 * t > n` the decoder reads `n - t` instead and complements
//! the bitmask — so the bit cost is always `≤ ceil(log2(C(n, ⌈n/2⌉)))`.
//!
//! Reference: `docs/audio/musepack/musepack-trace-reverse-engineering.md`
//! §4.2.2 and §5.14; sidecar §3.

use oxideav_core::bits::BitReader;
use oxideav_core::{Error, Result};

use crate::tables::{CNK, CNK_LEN, CNK_LOST};

/// Read a base code with `mpc8_dec_base(k, n)` — base `len` bits, plus
/// one extra bit if the value exceeds the lost-code threshold.
///
/// Equivalent to "ceiling-log encoding with one-bit-fewer for the
/// shorter side": a reader for an integer in `[0..C(n, k))` using
/// `ceil(log2(C(n, k)))` bits on average.
pub fn dec_base(br: &mut BitReader<'_>, k: usize, n: usize) -> Result<u32> {
    if k == 0 || k > 16 || n == 0 || n > 32 {
        return Err(Error::invalid(format!(
            "musepack CNS dec_base: out-of-range (k={k}, n={n})"
        )));
    }
    let len_field = CNK_LEN[k - 1][n - 1];
    if len_field == 0 {
        // 0-bit code: the value is 0 always.
        return Ok(0);
    }
    let len = len_field as u32 - 1;
    let lost = CNK_LOST[k - 1][n - 1];
    let mut code = if len == 0 { 0 } else { br.read_u32(len)? };
    if code >= lost {
        code = ((code << 1) | br.read_u32(1)?).wrapping_sub(lost);
    }
    Ok(code)
}

/// Modified-Golomb decode — read an integer in `[0..=m]`. At `m = 0`
/// the value is always 0; otherwise the call delegates to
/// `dec_base(1, m + 1)`.
pub fn mod_golomb(br: &mut BitReader<'_>, m: usize) -> Result<u32> {
    if m == 0 {
        return Ok(0);
    }
    if m + 1 > 32 {
        return Err(Error::invalid(format!(
            "musepack CNS mod_golomb: m={m} exceeds limit"
        )));
    }
    dec_base(br, 1, m + 1)
}

/// Decode the CNS subset index — a `k`-subset of `[0..n)` represented
/// as a bitmask. Returns the bitmask with bit `i` set iff item `i` is
/// in the subset. At `k == 0` returns 0; at `k == n` returns
/// `(1 << n) - 1`.
///
/// Algorithm: at each position `i`, the bitmask divides into "include"
/// (`C(n - 1, k - 1)` outcomes, indices `[0..C(n - 1, k - 1))`) and
/// "exclude" (`C(n - 1, k)` outcomes, indices
/// `[C(n - 1, k - 1)..C(n, k))`). Read `dec_base(k, n)`; if the index
/// falls in the include range, set the bit and decrement `k`; otherwise
/// shift the index by `C(n - 1, k - 1)` and leave `k`. Continue until
/// `n == 0`.
pub fn dec_enum(br: &mut BitReader<'_>, k: usize, n: usize) -> Result<u64> {
    if k == 0 {
        return Ok(0);
    }
    if k > n {
        return Err(Error::invalid(format!(
            "musepack CNS dec_enum: k > n (k={k}, n={n})"
        )));
    }
    if k > 16 || n > 32 {
        return Err(Error::invalid(format!(
            "musepack CNS dec_enum: out-of-range (k={k}, n={n})"
        )));
    }
    let mut code = u64::from(dec_base(br, k, n)?);
    let mut mask: u64 = 0;
    let mut kk = k;
    let mut nn = n;
    while nn > 0 && kk > 0 {
        // Number of `kk`-subsets of [0..nn) that **include** the
        // highest position (nn - 1) is `C(nn - 1, kk - 1)`. Our
        // `CNK[k - 1][n - 1] = C(n - 1, k)`, so to obtain
        // `C(nn - 1, kk - 1)` we look up `CNK[kk - 2][nn - 1]`. The
        // `kk == 1` corner case is `C(nn - 1, 0) = 1`.
        let include: u64 = if kk == 1 { 1 } else { CNK[kk - 2][nn - 1] };
        if code < include {
            // Item nn-1 is in the subset.
            mask |= 1u64 << (nn - 1);
            kk -= 1;
            // code stays
        } else {
            code -= include;
            // Item nn-1 is not in the subset; kk unchanged.
        }
        nn -= 1;
    }
    Ok(mask)
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxideav_core::bits::BitWriter;

    #[test]
    fn cnk_lookup_consistent() {
        // C(5, 2) = 10
        assert_eq!(CNK[1][5], 10);
        // C(8, 3) = 56
        assert_eq!(CNK[2][8], 56);
    }

    #[test]
    #[allow(clippy::unusual_byte_groupings)]
    fn dec_base_known_value() {
        // CNK_LEN[0][2] = 2, CNK_LOST[0][2] = 1 (k=1, n=3).
        // dec_base reads `len = 2 - 1 = 1` bit; if the value ≥ 1 (the
        // lost threshold), one extra bit is read and `lost` is
        // subtracted from the doubled value.
        // Bit stream "0 1 1 ...": first read 1 bit = 0 → 0 < 1, so
        // value = 0. Second read 1 bit = 1 → 1 ≥ 1, read one more
        // bit (1), value = (1 << 1 | 1) - 1 = 2.
        let data = [0b0_1_1_00000u8];
        let mut br = BitReader::new(&data);
        let v0 = dec_base(&mut br, 1, 3).unwrap();
        let v1 = dec_base(&mut br, 1, 3).unwrap();
        assert_eq!(v0, 0);
        assert_eq!(v1, 2);
    }

    #[test]
    fn mod_golomb_zero() {
        let data = [0u8];
        let mut br = BitReader::new(&data);
        assert_eq!(mod_golomb(&mut br, 0).unwrap(), 0);
    }

    #[test]
    fn dec_enum_full_subset() {
        // k = n: only one subset (all items in). Should return full
        // mask without consuming bits.
        let data = [0u8; 4];
        let mut br = BitReader::new(&data);
        assert_eq!(dec_enum(&mut br, 4, 4).unwrap(), 0b1111);
        assert_eq!(br.bit_position(), 0);
    }

    #[test]
    fn dec_enum_singleton() {
        // n = 4, k = 1: there are 4 singletons (C(4,1) = 4). The
        // mpc8_cnk_len[0][3] = 2, lost = 0 → straight 2-bit code.
        // Code 0b00 should select position 3 (the highest bit walked
        // first per the algorithm above — the include test fires when
        // code < C(nn-1, 0) = 1 at position nn=4, so "include 3" iff
        // code < 1 → code == 0 → bit 3).
        let mut bw = BitWriter::new();
        bw.write_u32(0b00, 2);
        let bytes = bw.finish();
        let mut br = BitReader::new(&bytes);
        let m = dec_enum(&mut br, 1, 4).unwrap();
        assert_eq!(m.count_ones(), 1);
    }
}
