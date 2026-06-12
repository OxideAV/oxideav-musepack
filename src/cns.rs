//! SV7 / SV8 CNS (noise-substitution) PRNG.
//!
//! Wires the two-LFSR pseudo-random noise generator that supplies
//! the spec §2.5 / §3.4 `band_type == -1` "fill all 36 samples with
//! random values" path.
//!
//! Source-of-record:
//!
//! - **Structural prose**: `docs/audio/musepack/musepack-sv7-sv8-spec.md`
//!   §2.5 (SV7 case -1) and §3.4 (SV8 case -1).
//! - **Numeric values + generator step**: the staged sidecars:
//!   * `docs/audio/musepack/tables/cns-prng-parity.csv` — the
//!     256-byte parity-of-popcount lookup (consumed in the LFSR
//!     update).
//!   * `docs/audio/musepack/tables/cns-prng-params.csv` — seeds,
//!     tap masks, shift, and the noise-sample byte-sum bias.
//!   * Both sidecars' `notes:` line describes the generator step
//!     verbatim: `r1=(r1>>1)|(Parity[r1&0xF5]<<31);
//!     r2=(r2<<1)|Parity[(r2>>25)&0x63]; word=r1^r2; noise q =
//!     (b0+b1+b2+b3) - 510 over the 4 bytes of word.`
//!
//! The constants are the staged CSV facts and the step is the staged
//! `.meta` notes-line transcription.

// Pulls in `PARITY`, `R1_SEED`, `R2_SEED`, `R1_TAP_MASK`,
// `R2_TAP_MASK`, `R2_SHIFT`, `NOISE_SAMPLE_BYTE_SUM_BIAS` from
// `build.rs`.
include!(concat!(env!("OUT_DIR"), "/cns_prng_tables.rs"));

/// Two-LFSR noise generator state used by CNS / noise substitution.
///
/// On [`CnsPrng::reset`] both LFSRs are loaded with `1` (the staged
/// `cns-prng-params.csv` `r{1,2}_seed` rows). Each call to
/// [`CnsPrng::next_sample`] advances both LFSRs by one step, XORs
/// them into a 32-bit word, sums the four bytes of that word, and
/// returns `byte_sum + NOISE_SAMPLE_BYTE_SUM_BIAS` as an `i32` —
/// the result lies in `-510..=510`.
#[derive(Debug, Clone, Copy)]
pub struct CnsPrng {
    r1: u32,
    r2: u32,
}

impl Default for CnsPrng {
    fn default() -> Self {
        Self::new()
    }
}

impl CnsPrng {
    /// Build a fresh generator in the reset state (both LFSRs == 1).
    #[inline]
    pub const fn new() -> Self {
        Self {
            r1: R1_SEED,
            r2: R2_SEED,
        }
    }

    /// Reset both LFSRs to their seed values. The Musepack decoder
    /// applies this on every stream reset.
    #[inline]
    pub fn reset(&mut self) {
        self.r1 = R1_SEED;
        self.r2 = R2_SEED;
    }

    /// Current `(r1, r2)` LFSR state; primarily for tests.
    #[inline]
    pub fn state(&self) -> (u32, u32) {
        (self.r1, self.r2)
    }

    /// Advance both LFSRs one step and return the XOR of the
    /// updated registers as a fresh 32-bit PRNG word.
    #[inline]
    pub fn next_word(&mut self) -> u32 {
        // Step `r1`: right-rotate, OR the parity of `r1 & 0xF5`
        // into bit 31.
        let p1 = u32::from(PARITY[(self.r1 & R1_TAP_MASK) as usize]);
        self.r1 = (self.r1 >> 1) | (p1 << 31);
        // Step `r2`: left-rotate, OR the parity of `(r2 >> 25) &
        // 0x63` into bit 0.
        let p2 = u32::from(PARITY[((self.r2 >> R2_SHIFT) & R2_TAP_MASK) as usize]);
        self.r2 = (self.r2 << 1) | p2;
        self.r1 ^ self.r2
    }

    /// Generate one noise sample per spec §2.5 case -1: take the
    /// XOR'd LFSR word, sum its four bytes (each as an unsigned
    /// `u8` cast to `i32`), then add the bias `-510`. The result
    /// lies in `-510..=510`.
    #[inline]
    pub fn next_sample(&mut self) -> i32 {
        let word = self.next_word();
        let b0 = (word & 0xFF) as i32;
        let b1 = ((word >> 8) & 0xFF) as i32;
        let b2 = ((word >> 16) & 0xFF) as i32;
        let b3 = ((word >> 24) & 0xFF) as i32;
        (b0 + b1 + b2 + b3) + NOISE_SAMPLE_BYTE_SUM_BIAS
    }

    /// Fill `dest` with consecutive noise samples. The Musepack
    /// case -1 path calls this for the 36 subband samples of one
    /// granule of a CNS band.
    pub fn fill_samples(&mut self, dest: &mut [i32]) {
        for slot in dest.iter_mut() {
            *slot = self.next_sample();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ─── Parity table shape (entry count + last entry) ──────

    /// `.meta` `resolved_dims: [256]`. The last entry corresponds
    /// to byte 0xFF, whose popcount is 8 (even) -> parity bit 0.
    #[test]
    fn parity_table_shape() {
        assert_eq!(PARITY.len(), 256);
        assert_eq!(PARITY[0xFF], 0);
    }

    /// The parity table is the popcount-mod-2 of the byte index.
    /// Spot-check a handful of bytes against `u8::count_ones`.
    #[test]
    fn parity_table_matches_popcount_mod_2() {
        for b in 0u32..=255 {
            let expected = (b.count_ones() & 1) as u8;
            assert_eq!(
                PARITY[b as usize], expected,
                "PARITY[{b:#04x}] expected popcount-mod-2 == {expected}",
            );
        }
    }

    // ─── Scalar constants ───────────────────────────────────

    /// `.meta` `cns-prng-params` rows — seeds, tap masks, shift,
    /// bias.
    #[test]
    fn scalar_constants_match_meta() {
        assert_eq!(R1_SEED, 1);
        assert_eq!(R2_SEED, 1);
        assert_eq!(R1_TAP_MASK, 0xF5);
        assert_eq!(R2_SHIFT, 25);
        assert_eq!(R2_TAP_MASK, 0x63);
        assert_eq!(NOISE_SAMPLE_BYTE_SUM_BIAS, -510);
    }

    // ─── Generator behaviour ────────────────────────────────

    /// Sanity: a freshly reset generator has both LFSRs == seed.
    #[test]
    fn reset_state() {
        let mut g = CnsPrng::new();
        g.reset();
        assert_eq!(g.state(), (R1_SEED, R2_SEED));
    }

    /// One step from the reset state, walked by hand:
    /// r1 = 1, r1 & 0xF5 = 1, popcount = 1 -> parity 1, so
    ///   r1' = (1 >> 1) | (1 << 31) = 0x8000_0000.
    /// r2 = 1, (r2 >> 25) & 0x63 = 0, parity 0, so
    ///   r2' = (1 << 1) | 0 = 2.
    /// word = r1' ^ r2' = 0x8000_0002.
    #[test]
    fn first_step_matches_handcrank() {
        let mut g = CnsPrng::new();
        let word = g.next_word();
        assert_eq!(g.state(), (0x8000_0000, 2));
        assert_eq!(word, 0x8000_0002);
        // Sample = bytes of word summed + bias.
        // Bytes (LE): 0x02, 0x00, 0x00, 0x80 -> sum = 130.
        // Sample = 130 - 510 = -380.
        let mut g2 = CnsPrng::new();
        let s = g2.next_sample();
        assert_eq!(s, -380);
    }

    /// The sample range must be inside `-510..=510` for every step
    /// in a longer run. Also pins generator determinism (a reset
    /// produces the same sequence).
    #[test]
    fn samples_stay_in_range_and_are_deterministic() {
        let mut g = CnsPrng::new();
        let mut buf_a = [0i32; 1024];
        g.fill_samples(&mut buf_a);
        for (i, &s) in buf_a.iter().enumerate() {
            assert!(
                (-510..=510).contains(&s),
                "sample {i} == {s} outside -510..=510",
            );
        }
        g.reset();
        let mut buf_b = [0i32; 1024];
        g.fill_samples(&mut buf_b);
        assert_eq!(buf_a, buf_b);
    }

    /// Default == reset state.
    #[test]
    fn default_is_reset() {
        let a = CnsPrng::default();
        let b = CnsPrng::new();
        assert_eq!(a.state(), b.state());
    }
}
