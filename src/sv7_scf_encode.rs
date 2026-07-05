//! SV7 §5.2/§5.3 scalefactor (SCFI + DSCF) **encode** — inverse of
//! [`crate::sv7_scf_decode::decode_sv7_band_scf`].
//!
//! Serialises a band's SCFI selector and its `1..=3` coded per-granule
//! scalefactor indices into the bit run the grounded §5.3 SCF decoder
//! reads back.
//!
//! # The §5.3 layout, inverted
//!
//! One SCFI VLC (`sv7-huffman-scfi`, value `0..=3`) then, per the §5.3
//! case table, the coded indices in the `SCF[0] → SCF[1] → SCF[2]` chain:
//!
//! | SCFI | `SCF[0]` | `SCF[1]` | `SCF[2]` |
//! |-----:|----------|----------|----------|
//! | 0 | coded (Δ vs prev band `SCF[2]`) | coded (Δ vs `SCF[0]`) | coded (Δ vs `SCF[1]`) |
//! | 1 | coded | coded (Δ vs `SCF[0]`) | = `SCF[1]` |
//! | 2 | coded | = `SCF[0]` | coded (Δ vs `SCF[1]`) |
//! | 3 | coded | = `SCF[0]` | = `SCF[1]` |
//!
//! Each coded index is emitted as a **DSCF delta** off the index before
//! it in the chain (`SCF[0]` off the previous band's `SCF[2]`) when the
//! delta is a plain `sv7-huffman-dscf` symbol (`−7..=7`, the alphabet
//! minus the escape). Otherwise the [`DSCF_ESCAPE_SYMBOL`] escape
//! (symbol `8`) is emitted followed by a raw [`DSCF_ESCAPE_RAW_BITS`]-bit
//! absolute index (so an escaped index must lie in `0..=63`). Preferring
//! the delta is pure encoder policy; either form decodes identically.
//!
//! [`choose_scfi`] picks the SCFI that shares the most indices for a
//! given `[SCF0, SCF1, SCF2]`, and [`encode_sv7_band_scf_auto`] wraps
//! selection + emit so a caller supplies only the three indices.
//!
//! Source-of-record: `docs/audio/musepack/spec/musepack-headers-and-coding.md`
//! §5.2 (SCFI) + §5.3 (DSCF cases / escape / prev-band reference). No new
//! format facts — pure inversion of the grounded §5.3 decode.

use crate::huffman::{SV7_DSCF_TABLE, SV7_SCFI_TABLE};
use crate::scf::SCF_GRANULES_PER_BAND;
use crate::sv7_bitwriter::Sv7BitWriter;
use crate::sv7_huffman_encode::write_symbol;
use crate::sv7_scf_decode::{DSCF_ESCAPE_RAW_BITS, DSCF_ESCAPE_SYMBOL};
use crate::{Error, Result};

/// Plain (non-escape) DSCF delta range: the `sv7-huffman-dscf` alphabet
/// is `−7..=8`, and `8` is the [`DSCF_ESCAPE_SYMBOL`] escape, so a delta
/// is emitted directly only when it lies in `−7..=7`.
const DSCF_DELTA_MIN: i32 = -7;
const DSCF_DELTA_MAX: i32 = 7;

/// Largest value representable in a raw [`DSCF_ESCAPE_RAW_BITS`]-bit
/// absolute index field (`2^6 − 1 = 63`).
const DSCF_RAW_MAX: i32 = (1 << DSCF_ESCAPE_RAW_BITS) - 1;

/// Pick the SCFI selector that shares the most of a band's three
/// reconstructed indices, matching the §5.3 case table's "copied"
/// semantics.
///
/// - all three equal ⇒ SCFI **3** (`SCF[1]=SCF[0]`, `SCF[2]=SCF[1]`);
/// - `SCF[0] == SCF[1]` only ⇒ SCFI **2** (`SCF[1]` copied, `SCF[2]`
///   coded);
/// - `SCF[1] == SCF[2]` only ⇒ SCFI **1** (`SCF[2]` copied);
/// - otherwise ⇒ SCFI **0** (all three coded).
pub fn choose_scfi(indices: [i32; SCF_GRANULES_PER_BAND]) -> u8 {
    let [s0, s1, s2] = indices;
    if s0 == s1 && s1 == s2 {
        3
    } else if s0 == s1 {
        2
    } else if s1 == s2 {
        1
    } else {
        0
    }
}

/// Emit one coded SCF index as a DSCF delta off `reference`, or — when
/// the delta is out of the plain range — the escape + a raw 6-bit
/// absolute index.
fn write_scf_index(writer: &mut Sv7BitWriter, reference: i32, index: i32) -> Result<()> {
    let delta = index - reference;
    if (DSCF_DELTA_MIN..=DSCF_DELTA_MAX).contains(&delta) {
        write_symbol(writer, &SV7_DSCF_TABLE, delta as i8)?;
        Ok(())
    } else {
        if !(0..=DSCF_RAW_MAX).contains(&index) {
            return Err(Error::SampleOutOfRange(index));
        }
        write_symbol(writer, &SV7_DSCF_TABLE, DSCF_ESCAPE_SYMBOL)?;
        writer.write_bits(index as u32, DSCF_ESCAPE_RAW_BITS);
        Ok(())
    }
}

/// Encode one band's SCFI selector and coded DSCF indices into `writer`,
/// the exact inverse of [`crate::sv7_scf_decode::decode_sv7_band_scf`].
///
/// `scfi` (`0..=3`) selects which of the three indices are independently
/// coded (see the module table). `indices` are the three reconstructed
/// per-granule SCF indices; only the coded ones are emitted — the copied
/// ones (per `scfi`) are recomputed on decode and **must** already equal
/// their source in `indices` for the round-trip to hold (use
/// [`encode_sv7_band_scf_auto`] to guarantee that). `prev_band_scf2` is
/// the reference `SCF[0]` deltas off (the previous band's `SCF[2]`;
/// thread the previous band's `indices[2]`).
///
/// # Errors
///
/// - [`Error::InvalidScfCodingMethod`] if `scfi > 3`.
/// - [`Error::SampleOutOfRange`] if an escaped index is outside `0..=63`.
/// - [`Error::SymbolNotEncodable`] propagated from a DSCF delta write
///   (unreachable for `−7..=7`, which the table covers).
pub fn encode_sv7_band_scf(
    writer: &mut Sv7BitWriter,
    scfi: u8,
    indices: [i32; SCF_GRANULES_PER_BAND],
    prev_band_scf2: i32,
) -> Result<()> {
    encode_sv7_scfi(writer, scfi)?;
    encode_sv7_band_dscf(writer, scfi, indices, prev_band_scf2)
}

/// Write one SCFI selector VLC alone — the SCFI-pass half of
/// [`encode_sv7_band_scf`], for the corpus-pinned two-pass frame-body
/// layout (all SCFI selectors first, then all DSCF chains; see
/// [`crate::sv7_stereo_frame`]).
///
/// # Errors
///
/// [`Error::InvalidScfCodingMethod`] if `scfi > 3`.
pub fn encode_sv7_scfi(writer: &mut Sv7BitWriter, scfi: u8) -> Result<()> {
    if scfi > 3 {
        return Err(Error::InvalidScfCodingMethod(scfi as i8));
    }
    write_symbol(writer, &SV7_SCFI_TABLE, scfi as i8)
}

/// Write one band's coded DSCF indices alone (no SCFI selector) — the
/// DSCF-pass half of [`encode_sv7_band_scf`]. `reference` is the
/// band's `SCF[0]` delta reference (the corpus-pinned per-band memory:
/// the same subband's previous-frame `SCF[2]`).
///
/// # Errors
///
/// As [`encode_sv7_band_scf`].
pub fn encode_sv7_band_dscf(
    writer: &mut Sv7BitWriter,
    scfi: u8,
    indices: [i32; SCF_GRANULES_PER_BAND],
    reference: i32,
) -> Result<()> {
    if scfi > 3 {
        return Err(Error::InvalidScfCodingMethod(scfi as i8));
    }
    let prev_band_scf2 = reference;
    let [s0, s1, s2] = indices;
    // SCF[0]: always coded, off the previous band's SCF[2].
    write_scf_index(writer, prev_band_scf2, s0)?;
    // SCF[1]: coded (Δ vs SCF[0]) for SCFI 0/1; copied otherwise.
    if matches!(scfi, 0 | 1) {
        write_scf_index(writer, s0, s1)?;
    }
    // SCF[2]: coded (Δ vs SCF[1]) for SCFI 0/2; copied otherwise.
    if matches!(scfi, 0 | 2) {
        write_scf_index(writer, s1, s2)?;
    }
    Ok(())
}

/// Pick the sharing-maximal SCFI for `indices` via [`choose_scfi`] and
/// encode the band, returning the chosen SCFI. This is the natural
/// encoder entry point: it guarantees the copied-index consistency
/// [`encode_sv7_band_scf`] requires.
///
/// # Errors
///
/// As [`encode_sv7_band_scf`] (an escaped index outside `0..=63`).
pub fn encode_sv7_band_scf_auto(
    writer: &mut Sv7BitWriter,
    indices: [i32; SCF_GRANULES_PER_BAND],
    prev_band_scf2: i32,
) -> Result<u8> {
    let scfi = choose_scfi(indices);
    encode_sv7_band_scf(writer, scfi, indices, prev_band_scf2)?;
    Ok(scfi)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::huffman::Sv7BitReader;
    use crate::sv7_scf_decode::decode_sv7_band_scf;

    fn round_trip(indices: [i32; 3], prev: i32) -> (u8, [i32; 3]) {
        let mut w = Sv7BitWriter::new();
        let scfi = encode_sv7_band_scf_auto(&mut w, indices, prev).expect("encode");
        let mut bytes = w.finish();
        bytes.push(0);
        bytes.push(0);
        let mut r = Sv7BitReader::new(&bytes);
        let dec = decode_sv7_band_scf(&mut r, prev).expect("decode");
        assert_eq!(dec.scfi, scfi, "scfi mismatch");
        (dec.scfi, dec.indices)
    }

    #[test]
    fn choose_scfi_matches_case_table() {
        assert_eq!(choose_scfi([5, 5, 5]), 3);
        assert_eq!(choose_scfi([5, 5, 9]), 2);
        assert_eq!(choose_scfi([5, 9, 9]), 1);
        assert_eq!(choose_scfi([5, 9, 13]), 0);
        // s0 == s2 but s1 differs is still "all coded" (SCFI 0): the
        // case table has no "SCF[2] = SCF[0]" sharing.
        assert_eq!(choose_scfi([5, 9, 5]), 0);
    }

    #[test]
    fn scfi0_all_distinct_round_trips() {
        let (scfi, idx) = round_trip([50, 49, 52], 48);
        assert_eq!(scfi, 0);
        assert_eq!(idx, [50, 49, 52]);
    }

    #[test]
    fn scfi1_shared_scf2_round_trips() {
        let (scfi, idx) = round_trip([14, 16, 16], 10);
        assert_eq!(scfi, 1);
        assert_eq!(idx, [14, 16, 16]);
    }

    #[test]
    fn scfi2_shared_scf1_round_trips() {
        let (scfi, idx) = round_trip([21, 21, 26], 20);
        assert_eq!(scfi, 2);
        assert_eq!(idx, [21, 21, 26]);
    }

    #[test]
    fn scfi3_all_shared_round_trips() {
        let (scfi, idx) = round_trip([28, 28, 28], 30);
        assert_eq!(scfi, 3);
        assert_eq!(idx, [28, 28, 28]);
    }

    #[test]
    fn escape_used_when_delta_out_of_range() {
        // prev = 0, SCF[0] = 45: delta +45 is out of {-7..=7}, so escape
        // to a raw 6-bit absolute (45 fits 0..=63). All three shared.
        let (scfi, idx) = round_trip([45, 45, 45], 0);
        assert_eq!(scfi, 3);
        assert_eq!(idx, [45, 45, 45]);
    }

    #[test]
    fn escape_for_a_later_coded_index() {
        // SCFI 0: SCF[0] small delta, SCF[1] escapes (big jump), SCF[2]
        // small delta off the escaped SCF[1].
        let (scfi, idx) = round_trip([3, 40, 39], 0);
        assert_eq!(scfi, 0);
        assert_eq!(idx, [3, 40, 39]);
    }

    #[test]
    fn explicit_scfi_with_consistent_indices_round_trips() {
        // Drive the lower-level entry point directly with a consistent
        // SCFI-2 index set (SCF[1] == SCF[0]).
        let mut w = Sv7BitWriter::new();
        encode_sv7_band_scf(&mut w, 2, [12, 12, 20], 5).expect("encode");
        let mut bytes = w.finish();
        bytes.push(0);
        bytes.push(0);
        let mut r = Sv7BitReader::new(&bytes);
        let dec = decode_sv7_band_scf(&mut r, 5).unwrap();
        assert_eq!(dec.scfi, 2);
        assert_eq!(dec.indices, [12, 12, 20]);
    }

    #[test]
    fn prev_band_scf2_threads_between_bands() {
        // Two consecutive bands; band 1's SCF[0] deltas off band 0's SCF[2].
        let mut w = Sv7BitWriter::new();
        let s0 = encode_sv7_band_scf_auto(&mut w, [5, 5, 5], 0).unwrap();
        let s1 = encode_sv7_band_scf_auto(&mut w, [7, 7, 7], 5).unwrap();
        let mut bytes = w.finish();
        bytes.push(0);
        bytes.push(0);
        let mut r = Sv7BitReader::new(&bytes);
        let b0 = decode_sv7_band_scf(&mut r, 0).unwrap();
        let b1 = decode_sv7_band_scf(&mut r, b0.last_index()).unwrap();
        assert_eq!((b0.scfi, b0.indices), (s0, [5, 5, 5]));
        assert_eq!((b1.scfi, b1.indices), (s1, [7, 7, 7]));
    }

    #[test]
    fn rejects_scfi_above_three() {
        let mut w = Sv7BitWriter::new();
        assert_eq!(
            encode_sv7_band_scf(&mut w, 4, [0, 0, 0], 0),
            Err(Error::InvalidScfCodingMethod(4)),
        );
    }

    #[test]
    fn rejects_unrepresentable_escape_index() {
        // prev = 0, index = 200: delta 200 out of range and 200 > 63, so
        // neither delta nor raw-6-bit escape can represent it.
        let mut w = Sv7BitWriter::new();
        assert_eq!(
            encode_sv7_band_scf(&mut w, 3, [200, 200, 200], 0),
            Err(Error::SampleOutOfRange(200)),
        );
    }
}
