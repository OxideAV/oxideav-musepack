//! SV7 §2.4 scalefactor (SCF) coding-method decoder.
//!
//! Wires the per-non-zero-band SCF VLC loop documented in
//! `docs/audio/musepack/musepack-sv7-sv8-spec.md` §2.4 ("Frame body —
//! scalefactor (SCF) coding") on top of the already-staged
//! `sv7-huffman-scfi` selector VLC and `sv7-huffman-dscf` delta VLC.
//!
//! # Decoder shape (from §2.4 + §1)
//!
//! For each non-zero band (`band_type != 0`) the SV7 frame body
//! carries:
//!
//! 1. **One SCF coding-method selector** (VLC over
//!    [`crate::huffman::SV7_SCFI_TABLE`]) — a 2-bit value in
//!    `{0, 1, 2, 3}` selecting how many distinct per-granule SCF
//!    indices are sent for the band and which granules each covers.
//! 2. **One to three SCF deltas** (VLC over
//!    [`crate::huffman::SV7_DSCF_TABLE`]), one per *distinct* SCF
//!    transmitted (1, 2, or 3 reads depending on the selector).
//!
//! Layer-II §2.4.2.5/2.4.2.6 (`audio/mp3/ISO_IEC_11172-3-MP3-1993.pdf`,
//! cited as source S3 in §1 of the structural spec) defines the
//! granule-coverage convention by SCFSI value, which §1 lines 79-82
//! of the structural spec restates verbatim:
//!
//! - `scfsi == 0`: three SCFs, one each for granules 0, 1, 2.
//! - `scfsi == 1`: two — first for granules 0 and 1, second for 2.
//! - `scfsi == 2`: one SCF valid for all three granules.
//! - `scfsi == 3`: two — first for granule 0, second for granules 1
//!   and 2.
//!
//! §2.4 then says Musepack "replaces the *coding* of these scalefactor
//! indices (it delta-codes them with its own VLC, see §2.4 / §3.5) but
//! the underlying notion of a per-subband, per-granule scalefactor
//! index is inherited" (§1 lines 83-85). So the SCFI value drives the
//! same granule-mapping table as Layer II SCFSI; only the delta
//! coding is Musepack's.
//!
//! # Delta semantics
//!
//! §2.4 line 164 states the coding-method selector picks whether SCFs
//! "are sent or some are shared / delta-coded against the previous
//! one." The "previous one" is the most recently decoded SCF in the
//! same band: each successive distinct SCF transmitted by the band
//! is a `DSCF` value added to the previous one, with the first being
//! added to a caller-supplied per-band anchor.
//!
//! The anchor itself — the SCF base for the band — is not specified
//! by the structural prose (it belongs to the SV7 fixed-header field
//! map, which §2.1 lists as GAP). This module therefore takes the
//! base as an `i32` argument and leaves anchor sourcing to the
//! frame driver.
//!
//! # API
//!
//! - [`ScfCodingMethod`] — typed wrapper around the `0..=3` SCFI
//!   value with a granule-schedule accessor.
//! - [`GranuleSchedule`] — small struct exposing both the number of
//!   distinct SCFs to read (`deltas_to_read()`) and the
//!   granule → stored-delta-slot mapping (`granule_to_slot()`).
//! - [`decode_band_scf`] — full per-band loop: reads one SCFI VLC,
//!   reads N DSCF VLCs, walks the schedule, and returns the 3
//!   reconstructed per-granule SCF indices given a base anchor.
//!
//! All bit reads are SCF-side only (this module never touches the
//! per-band band-type header VLC nor the per-sample quantiser VLC);
//! the caller drives the per-non-zero-band loop and decides per-band
//! whether to invoke this module based on the band-type ≠ 0 check.

use crate::huffman::{decode, Sv7BitReader, SV7_DSCF_TABLE, SV7_SCFI_TABLE};
use crate::{Error, Result};

/// Number of SCF granules per band per channel (Layer-II /
/// Musepack-inherited frame geometry: 1152 samples / 32 subbands /
/// 12 samples-per-granule = 3 granules per subband per channel; see
/// §1 of the structural spec, lines 65-71 and 75-76).
pub const SCF_GRANULES_PER_BAND: usize = 3;

/// Maximum number of distinct SCFs transmitted by a single
/// non-zero band: the SCFI value `0` ("three SCFs, one each for
/// granules 0/1/2"; §1 line 79) is the worst case.
pub const SCF_MAX_DISTINCT: usize = SCF_GRANULES_PER_BAND;

/// SV7 §2.4 SCF coding-method index.
///
/// The selector VLC ([`SV7_SCFI_TABLE`]) emits one of the four
/// values `0..=3`. Each value chooses a fixed
/// (count, granule-mapping) pair per Layer-II SCFSI convention; see
/// [`GranuleSchedule`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ScfCodingMethod(u8);

impl ScfCodingMethod {
    /// Build from a raw SCFI value. Returns
    /// [`Error::InvalidScfCodingMethod`] for values outside `0..=3`.
    pub fn from_raw(raw: i8) -> Result<Self> {
        if (0..=3).contains(&raw) {
            Ok(Self(raw as u8))
        } else {
            Err(Error::InvalidScfCodingMethod(raw))
        }
    }

    /// The raw `0..=3` value.
    pub fn raw(self) -> u8 {
        self.0
    }

    /// Granule schedule for this coding method.
    pub fn schedule(self) -> GranuleSchedule {
        // The four arms transcribed from §1 lines 79-82 (the
        // Layer-II SCFSI convention restated by the Musepack
        // structural spec). The `slots` triple gives, for each of
        // the 3 granules in band order, which transmitted delta
        // slot (0, 1, or 2) supplies its SCF.
        let (count, slots) = match self.0 {
            // scfsi == 0: three SCFs, one each for granules 0,1,2.
            0 => (3, [0u8, 1, 2]),
            // scfsi == 1: two — first for granules 0 and 1, second
            // for 2.
            1 => (2, [0u8, 0, 1]),
            // scfsi == 2: one SCF valid for all three granules.
            2 => (1, [0u8, 0, 0]),
            // scfsi == 3: two — first for granule 0, second for
            // granules 1 and 2.
            3 => (2, [0u8, 1, 1]),
            // `from_raw` constrains the inner value to 0..=3;
            // unreachable in safe code.
            _ => unreachable!("ScfCodingMethod inner value out of range"),
        };
        GranuleSchedule { count, slots }
    }
}

/// Per-band schedule: number of distinct SCF deltas to read, plus
/// the granule → delta-slot mapping derived from the SCFI value.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GranuleSchedule {
    count: u8,
    slots: [u8; SCF_GRANULES_PER_BAND],
}

impl GranuleSchedule {
    /// Number of distinct DSCF deltas the caller should read off the
    /// bitstream for this band (`1..=3`).
    pub fn deltas_to_read(self) -> usize {
        self.count as usize
    }

    /// Per-granule mapping: `granule_to_slot()[g]` is the index
    /// (in the 0..deltas_to_read() range) of the transmitted delta
    /// that supplies granule `g`'s SCF.
    pub fn granule_to_slot(self) -> [u8; SCF_GRANULES_PER_BAND] {
        self.slots
    }
}

/// Reconstruct the three per-granule SCF indices for a band.
///
/// Inputs:
/// - `reader` — bit reader over the band-body bit stream.
/// - `base` — the per-band anchor SCF index (see module doc; in the
///   SV7 frame the anchor either comes from the fixed-header (GAP)
///   or from the last SCF of the previously decoded band — the
///   exact convention belongs to the frame driver, not this module).
///
/// Returns `(method, scf_indices)` where:
/// - `method` is the decoded [`ScfCodingMethod`] (caller-observable
///   for unit testing / round-trip);
/// - `scf_indices[g]` is the absolute SCF index for granule `g`,
///   computed as `base + Σ DSCFs[0..=slots[g]]` per the
///   "delta-coded against the previous one" rule (§2.4 line 164).
pub fn decode_band_scf(reader: &mut Sv7BitReader<'_>, base: i32) -> Result<BandScf> {
    let scfi_raw = decode(reader, &SV7_SCFI_TABLE)?;
    let method = ScfCodingMethod::from_raw(scfi_raw)?;
    let schedule = method.schedule();
    reconstruct_scf_from_deltas(reader, base, schedule).map(|indices| BandScf { method, indices })
}

/// Read `schedule.deltas_to_read()` DSCF values and turn them into
/// the three per-granule SCF indices.
///
/// Public for testing / for an SV7 frame driver that has already
/// peeled off the SCFI VLC and wants to drive the delta loop
/// separately.
pub fn reconstruct_scf_from_deltas(
    reader: &mut Sv7BitReader<'_>,
    base: i32,
    schedule: GranuleSchedule,
) -> Result<[i32; SCF_GRANULES_PER_BAND]> {
    let mut transmitted = [0i32; SCF_MAX_DISTINCT];
    let mut running = base;
    for transmitted_slot in transmitted.iter_mut().take(schedule.deltas_to_read()) {
        let delta = decode(reader, &SV7_DSCF_TABLE)? as i32;
        running = running.saturating_add(delta);
        *transmitted_slot = running;
    }
    let slots = schedule.granule_to_slot();
    let mut out = [0i32; SCF_GRANULES_PER_BAND];
    for g in 0..SCF_GRANULES_PER_BAND {
        out[g] = transmitted[slots[g] as usize];
    }
    Ok(out)
}

/// Decoded per-band SCF data.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BandScf {
    /// Coding method recovered from the SCFI VLC.
    pub method: ScfCodingMethod,
    /// Absolute SCF index per granule (`indices[0..3]` for granules
    /// 0, 1, 2 respectively).
    pub indices: [i32; SCF_GRANULES_PER_BAND],
}

#[cfg(test)]
mod tests {
    use super::*;

    // ─── Coding-method classifier ────────────────────────────

    #[test]
    fn scfi_values_round_trip_through_from_raw() {
        for v in 0..=3i8 {
            let m = ScfCodingMethod::from_raw(v).unwrap();
            assert_eq!(m.raw() as i8, v);
        }
    }

    #[test]
    fn scfi_negative_or_overflow_rejected() {
        // §2.4 only defines values 0..=3; the DSCF table can emit
        // negative values, but the SCFI table cannot.
        for v in [-1i8, -2, 4, 5, 7, i8::MAX, i8::MIN] {
            assert_eq!(
                ScfCodingMethod::from_raw(v),
                Err(Error::InvalidScfCodingMethod(v))
            );
        }
    }

    // ─── Granule schedule (the Layer-II SCFSI inheritance) ────

    #[test]
    fn schedule_method_0_is_three_distinct_one_per_granule() {
        let s = ScfCodingMethod::from_raw(0).unwrap().schedule();
        assert_eq!(s.deltas_to_read(), 3);
        assert_eq!(s.granule_to_slot(), [0, 1, 2]);
    }

    #[test]
    fn schedule_method_1_is_two_first_pair_then_singleton() {
        // §1 line 80: first for granules 0 and 1, second for 2.
        let s = ScfCodingMethod::from_raw(1).unwrap().schedule();
        assert_eq!(s.deltas_to_read(), 2);
        assert_eq!(s.granule_to_slot(), [0, 0, 1]);
    }

    #[test]
    fn schedule_method_2_is_one_shared_for_all_three() {
        // §1 line 81: one SCF valid for all three granules.
        let s = ScfCodingMethod::from_raw(2).unwrap().schedule();
        assert_eq!(s.deltas_to_read(), 1);
        assert_eq!(s.granule_to_slot(), [0, 0, 0]);
    }

    #[test]
    fn schedule_method_3_is_two_singleton_then_pair() {
        // §1 line 82: first for granule 0, second for granules 1 and 2.
        let s = ScfCodingMethod::from_raw(3).unwrap().schedule();
        assert_eq!(s.deltas_to_read(), 2);
        assert_eq!(s.granule_to_slot(), [0, 1, 1]);
    }

    // ─── Delta reconstruction (deterministic, no bit reader) ──

    #[test]
    fn reconstruct_method_2_replicates_single_delta_into_all_three_granules() {
        // Build a bitstream that produces a single DSCF == +2.
        // From sv7-huffman-dscf.csv: row "0x4000,3,2" — code prefix
        // 010 (top 3 bits) decodes to delta 2. Pack it into a byte
        // with the rest zeroed (peek16 sees 0x4000 << 0).
        // We pad with 0x00 0x00 to satisfy the bit reader's 16-bit
        // peek window even after the consume.
        let bytes = [0b0100_0000u8, 0x00, 0x00, 0x00];
        let mut r = Sv7BitReader::new(&bytes);
        let schedule = ScfCodingMethod::from_raw(2).unwrap().schedule();
        let scfs = reconstruct_scf_from_deltas(&mut r, 10, schedule).unwrap();
        assert_eq!(scfs, [12, 12, 12]);
    }

    #[test]
    fn reconstruct_method_0_accumulates_three_deltas_running_sum() {
        // Use three DSCF values whose codes pack neatly: pick
        //   delta = -1  → row "0x6000,3,-1"  → bits 011
        //   delta = +2  → row "0x4000,3,2"   → bits 010
        //   delta = -2  → row "0x0000,3,-2"  → bits 000
        // Concatenated MSB-first: 011 010 000 = 0b011_010_000 (9 bits).
        // Packed into a byte stream: byte0 = 0b0110_1000 = 0x68,
        // byte1 has the remaining 1 high bit '0' then zeros.
        let bytes = [0x68u8, 0x00, 0x00, 0x00];
        let mut r = Sv7BitReader::new(&bytes);
        let schedule = ScfCodingMethod::from_raw(0).unwrap().schedule();
        // Base 100; running deltas -1, +2, -2 → transmitted slots
        // 99, 101, 99; granule mapping is identity for method 0.
        let scfs = reconstruct_scf_from_deltas(&mut r, 100, schedule).unwrap();
        assert_eq!(scfs, [99, 101, 99]);
    }

    #[test]
    fn reconstruct_method_1_shares_first_delta_across_granules_0_and_1() {
        // Two DSCFs: +2 then -1.
        //   +2 → bits 010 (length 3)
        //   -1 → bits 011 (length 3)
        // Concatenated MSB-first: 010 011 = 0b010_011_00 = 0x4C
        // (after padding to a byte). The remaining 2 low bits are
        // garbage we never consume.
        let bytes = [0x4Cu8, 0x00, 0x00, 0x00];
        let mut r = Sv7BitReader::new(&bytes);
        let schedule = ScfCodingMethod::from_raw(1).unwrap().schedule();
        // Base 50; running deltas +2 then -1 → transmitted [52, 51];
        // granule mapping [0,0,1] → [52, 52, 51].
        let scfs = reconstruct_scf_from_deltas(&mut r, 50, schedule).unwrap();
        assert_eq!(scfs, [52, 52, 51]);
    }

    #[test]
    fn reconstruct_method_3_singleton_then_pair_mapping() {
        // Two DSCFs: -2 then +2. Mapping [0,1,1] → second covers
        // granules 1 and 2.
        //   -2 → bits 000 (length 3)
        //   +2 → bits 010 (length 3)
        // Concatenated: 000 010 = 0b00001000 = 0x08 followed by zeros.
        let bytes = [0x08u8, 0x00, 0x00, 0x00];
        let mut r = Sv7BitReader::new(&bytes);
        let schedule = ScfCodingMethod::from_raw(3).unwrap().schedule();
        let scfs = reconstruct_scf_from_deltas(&mut r, 0, schedule).unwrap();
        // Running: -2 then -2+2=0; transmitted [-2, 0];
        // granule mapping [0,1,1] → [-2, 0, 0].
        assert_eq!(scfs, [-2, 0, 0]);
    }

    // ─── End-to-end: SCFI VLC + DSCF VLCs ─────────────────────

    #[test]
    fn decode_band_scf_end_to_end_method_2() {
        // SCFI value 2: from sv7-huffman-scfi.csv, row "0x4000,3,0" is value 0,
        // row "0x6000,3,2" is value 2, row "0x8000,1,1" is value 1, row
        // "0x0000,2,3" is value 3. Value 2 has code prefix 011 (length 3).
        // Then one DSCF value: pick +2 (code prefix 010, length 3).
        // Concatenated MSB-first: 011 010 = 0b01101000 = 0x68.
        let bytes = [0x68u8, 0x00, 0x00, 0x00];
        let mut r = Sv7BitReader::new(&bytes);
        let band = decode_band_scf(&mut r, 7).unwrap();
        assert_eq!(band.method.raw(), 2);
        // Method 2 replicates a single SCF across all three granules:
        // base 7 + delta 2 = 9.
        assert_eq!(band.indices, [9, 9, 9]);
    }

    #[test]
    fn decode_band_scf_propagates_unexpected_eof_in_scfi_phase() {
        // Empty buffer: peek16 must EOF before any value is decoded.
        let bytes = [];
        let mut r = Sv7BitReader::new(&bytes);
        assert_eq!(decode_band_scf(&mut r, 0), Err(Error::UnexpectedEof));
    }

    #[test]
    fn decode_band_scf_propagates_unexpected_eof_in_dscf_phase() {
        // Provide exactly the bits the SCFI VLC needs (2 bytes = 16
        // bits, since the reader's peek window is 16-wide). The SCFI
        // decode consumes 3 bits for value 2; the subsequent DSCF
        // peek16 must EOF because only 13 buffered bits remain and
        // no further bytes back the underlying slice.
        //
        // Pack: bits 0..=2 = '011' (SCFI value 2); bits 3..=15 = 0.
        // Byte 0 = 0b0110_0000 = 0x60; byte 1 = 0.
        let bytes = [0x60u8, 0x00];
        let mut r = Sv7BitReader::new(&bytes);
        assert_eq!(decode_band_scf(&mut r, 0), Err(Error::UnexpectedEof));
    }

    #[test]
    fn schedule_count_never_exceeds_max_distinct() {
        for v in 0..=3i8 {
            let s = ScfCodingMethod::from_raw(v).unwrap().schedule();
            assert!(s.deltas_to_read() <= SCF_MAX_DISTINCT);
            // Every granule slot must reference a valid transmitted
            // delta (in 0..count).
            for &slot in &s.granule_to_slot() {
                assert!(
                    (slot as usize) < s.deltas_to_read(),
                    "method {v} slot {slot} out of range [0, {})",
                    s.deltas_to_read()
                );
            }
        }
    }

    #[test]
    fn schedule_granule_count_is_three_per_band() {
        // §1 / §2.4 frame geometry invariant: each band has exactly
        // three SCF granules. Surface this as a const sanity test
        // so a future refactor cannot quietly bump the constant.
        assert_eq!(SCF_GRANULES_PER_BAND, 3);
    }

    #[test]
    fn reconstruct_with_zero_base_yields_running_delta_sum() {
        // Sanity check that an empty / zero base reduces the
        // function to "running sum of the read deltas across the
        // chosen granule mapping".
        // Three deltas: -1, +2, -2 (same packing as the method-0 test).
        let bytes = [0x68u8, 0x00, 0x00, 0x00];
        let mut r = Sv7BitReader::new(&bytes);
        let schedule = ScfCodingMethod::from_raw(0).unwrap().schedule();
        let scfs = reconstruct_scf_from_deltas(&mut r, 0, schedule).unwrap();
        assert_eq!(scfs, [-1, 1, -1]);
    }
}
