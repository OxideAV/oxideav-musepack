//! SV7 §5.3 grounded scalefactor (SCF / DSCF) decode.
//!
//! The simpler [`crate::scf`] module decodes a band's three per-granule
//! SCF indices from a **caller-supplied base anchor** using the Layer-II
//! SCFSI granule-coverage convention restated by the structural spec §1.
//! That model leaves three things open that the staged
//! `spec/musepack-headers-and-coding.md` **§5.3** now pins exactly:
//!
//! 1. **The reference for `SCF[0]`.** §5.3: "The reference for `SCF[0]`
//!    is the previous band's `SCF[2]`; within a band each later index
//!    deltas off the previous one." So the per-band base anchor is *not*
//!    a free caller knob — it is the previously-decoded band's third
//!    scalefactor (the first band of a channel deltas off a documented
//!    starting reference; see [`decode_sv7_band_scf`]).
//!
//! 2. **The exact SCFI → coded/shared pattern.** §5.3 spells out the
//!    four SCFI cases cell-for-cell, and this differs from the Layer-II
//!    `granule_to_slot` table [`crate::scf::GranuleSchedule`] carries:
//!
//!    | SCFI | `SCF[0]` | `SCF[1]` | `SCF[2]` |
//!    |-----:|----------|----------|----------|
//!    | 0 | coded (Δ vs prev band `SCF[2]`) | coded (Δ vs `SCF[0]`) | coded (Δ vs `SCF[1]`) |
//!    | 1 | coded (Δ vs prev band `SCF[2]`) | coded (Δ vs `SCF[0]`) | = `SCF[1]` |
//!    | 2 | coded (Δ vs prev band `SCF[2]`) | = `SCF[0]` | coded (Δ vs `SCF[1]`) |
//!    | 3 | coded (Δ vs prev band `SCF[2]`) | = `SCF[0]` | = `SCF[1]` |
//!
//!    In every case `SCF[0]` is independently coded; the SCFI value only
//!    decides whether `SCF[1]` / `SCF[2]` are independently coded or
//!    copied from the index before them. Each *coded* index reads one
//!    DSCF symbol and deltas off the index immediately preceding it in
//!    the `SCF[0] → SCF[1] → SCF[2]` chain — not off a fixed transmitted
//!    slot. (This is why the §5.3 model cannot be expressed by
//!    [`crate::scf`]'s slot table: §5.3 case 2 copies `SCF[1]` from
//!    `SCF[0]` but still codes `SCF[2]` as a delta off that copied
//!    `SCF[1]`, whereas the slot table would have `SCF[2]` read a fresh
//!    transmitted delta off the band base.)
//!
//! 3. **The DSCF escape.** §5.3: a DSCF symbol of `8` is an **escape**
//!    meaning "read a raw 6-bit absolute index instead" (not a delta).
//!    The staged `sv7-huffman-dscf` table carries the literal value `8`
//!    (`0xc000,4,8`), so the escape is reachable from a real bitstream.
//!
//! `SCF[0]`'s delta-vs-prev-band-`SCF[2]` is also subject to the escape:
//! the first coded index of a band uses the same DSCF read, so an `8`
//! there likewise switches to a raw 6-bit absolute.
//!
//! ## Clamp
//!
//! §5.3: "A decoded index exceeding 1024 is clamped to a sentinel
//! (treated as out-of-range/silent)." This module surfaces an
//! out-of-range index via [`Sv7BandScf::clamped`] rather than silently
//! zeroing — the same fail-soft-but-observable posture the rest of the
//! crate takes — leaving the silent-band substitution to the
//! reconstruction layer.
//!
//! ## Source-of-record
//!
//! `docs/audio/musepack/spec/musepack-headers-and-coding.md` §5.2
//! (SCFI VLC) + §5.3 (DSCF cases, escape, prev-band reference, clamp);
//! the `sv7-huffman-scfi` / `sv7-huffman-dscf` table facts under
//! `docs/audio/musepack/tables/`. The only project material crossed is
//! that staged `docs/` content and the sibling modules under
//! `crates/oxideav-musepack/src/`.

use crate::huffman::{decode as huffman_decode, Sv7BitReader, SV7_DSCF_TABLE, SV7_SCFI_TABLE};
use crate::scf::SCF_GRANULES_PER_BAND;
use crate::{Error, Result};

/// The §5.3 DSCF escape symbol: a decoded DSCF value of `8` means "read
/// a raw 6-bit absolute index instead of treating the symbol as a
/// delta." The staged `sv7-huffman-dscf` table carries the literal `8`.
pub const DSCF_ESCAPE_SYMBOL: i8 = 8;

/// Width, in bits, of the raw absolute index read when a DSCF symbol is
/// the [`DSCF_ESCAPE_SYMBOL`] escape (§5.3: "a raw 6-bit absolute
/// index").
pub const DSCF_ESCAPE_RAW_BITS: u8 = 6;

/// §5.3 out-of-range clamp threshold: "A decoded index exceeding 1024
/// is clamped to a sentinel (treated as out-of-range/silent)."
pub const SCF_CLAMP_THRESHOLD: i32 = 1024;

/// One band's three per-granule SV7 scalefactor indices plus the
/// observable bookkeeping the §5.3 walk produced.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Sv7BandScf {
    /// The raw `0..=3` SCFI selector value the band's SCFI VLC decoded.
    pub scfi: u8,
    /// The three reconstructed per-granule SCF indices (granules 0, 1,
    /// 2). Index `0` is always independently coded; `1` / `2` are coded
    /// or copied per the §5.3 SCFI table.
    pub indices: [i32; SCF_GRANULES_PER_BAND],
    /// True iff any reconstructed index exceeded [`SCF_CLAMP_THRESHOLD`]
    /// (§5.3's "exceeding 1024 ⇒ sentinel" condition). The indices are
    /// returned verbatim; the reconstruction layer decides the
    /// silent-band substitution.
    pub clamped: bool,
}

impl Sv7BandScf {
    /// The last granule's SCF index — the §5.3 reference the *next*
    /// band's `SCF[0]` deltas off of.
    pub const fn last_index(&self) -> i32 {
        self.indices[SCF_GRANULES_PER_BAND - 1]
    }
}

/// Read one §5.3 scalefactor index: either a DSCF delta added to
/// `reference`, or — when the DSCF symbol is the [`DSCF_ESCAPE_SYMBOL`]
/// escape — a raw [`DSCF_ESCAPE_RAW_BITS`]-bit **absolute** index.
///
/// Returns the reconstructed absolute index.
///
/// # Errors
///
/// - [`Error::UnexpectedEof`] if the reader starves on the DSCF VLC or
///   the escape's raw bits.
/// - [`Error::HuffmanNoMatch`] if the DSCF peek matches no table row.
fn read_scf_index(reader: &mut Sv7BitReader<'_>, reference: i32) -> Result<i32> {
    let sym = huffman_decode(reader, &SV7_DSCF_TABLE)?;
    if sym == DSCF_ESCAPE_SYMBOL {
        // Escape: a raw 6-bit absolute index replaces the delta.
        Ok(reader.read_bits(DSCF_ESCAPE_RAW_BITS)? as i32)
    } else {
        Ok(reference.saturating_add(sym as i32))
    }
}

/// Decode one band's three SV7 scalefactor indices per §5.2 + §5.3.
///
/// Reads exactly one SCFI selector VLC, then `1..=3` DSCF indices
/// according to the §5.3 SCFI case table:
///
/// - `SCF[0]` is **always** coded — one DSCF index relative to
///   `prev_band_scf2` (the previous band's `SCF[2]`, per §5.3); the
///   first band of a channel passes the channel's documented starting
///   reference.
/// - `SCF[1]`: coded (Δ vs `SCF[0]`) for SCFI `0`/`1`; copied from
///   `SCF[0]` for SCFI `2`/`3`.
/// - `SCF[2]`: coded (Δ vs `SCF[1]`) for SCFI `0`/`2`; copied from
///   `SCF[1]` for SCFI `1`/`3`.
///
/// Each *coded* index uses [`read_scf_index`], so the §5.3 `idx == 8`
/// raw-6-bit absolute escape applies to every coded index including
/// `SCF[0]`.
///
/// `prev_band_scf2` is the reference for this band's `SCF[0]`; thread
/// [`Sv7BandScf::last_index`] of the previously-decoded non-zero band
/// forward across the channel's band loop.
///
/// # Errors
///
/// - [`Error::InvalidScfCodingMethod`] if the SCFI VLC decodes outside
///   `0..=3` (unreachable for the staged `sv7-huffman-scfi` table,
///   whose value alphabet is exactly `{0,1,2,3}`; kept as a defensive
///   bound).
/// - [`Error::UnexpectedEof`] / [`Error::HuffmanNoMatch`] propagated
///   from the SCFI or DSCF reads.
pub fn decode_sv7_band_scf(
    reader: &mut Sv7BitReader<'_>,
    prev_band_scf2: i32,
) -> Result<Sv7BandScf> {
    let scfi_raw = huffman_decode(reader, &SV7_SCFI_TABLE)?;
    if !(0..=3).contains(&scfi_raw) {
        return Err(Error::InvalidScfCodingMethod(scfi_raw));
    }
    let scfi = scfi_raw as u8;

    // SCF[0]: always coded, delta off the previous band's SCF[2].
    let scf0 = read_scf_index(reader, prev_band_scf2)?;

    // SCF[1]: coded (Δ vs SCF[0]) for SCFI 0/1; copied from SCF[0]
    // otherwise (SCFI 2/3).
    let scf1 = match scfi {
        0 | 1 => read_scf_index(reader, scf0)?,
        _ => scf0,
    };

    // SCF[2]: coded (Δ vs SCF[1]) for SCFI 0/2; copied from SCF[1]
    // otherwise (SCFI 1/3).
    let scf2 = match scfi {
        0 | 2 => read_scf_index(reader, scf1)?,
        _ => scf1,
    };

    let indices = [scf0, scf1, scf2];
    let clamped = indices.iter().any(|&i| i > SCF_CLAMP_THRESHOLD);
    Ok(Sv7BandScf {
        scfi,
        indices,
        clamped,
    })
}

/// Number of distinct DSCF index reads SCFI value `scfi` performs
/// (`SCF[0]` always + one for each independently-coded later index).
///
/// `0` → 3, `1` → 2, `2` → 2, `3` → 1. Returns `None` for `scfi > 3`.
/// Pure structural helper for callers that need to size a bit budget
/// without driving the decode.
pub const fn dscf_reads_for_scfi(scfi: u8) -> Option<usize> {
    match scfi {
        0 => Some(3),
        1 | 2 => Some(2),
        3 => Some(1),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// MSB-first bit packer (mirrors the SV7 band-header tests): push a
    /// `length`-bit codeword from the top of `pattern`; `push_raw` a
    /// right-justified raw field; `finish` flushes + appends two zero
    /// bytes so the reader's 16-bit peek never starves mid-decode.
    struct BitPacker {
        bytes: Vec<u8>,
        acc: u32,
        nbits: u8,
    }

    impl BitPacker {
        fn new() -> Self {
            BitPacker {
                bytes: Vec::new(),
                acc: 0,
                nbits: 0,
            }
        }

        fn push(&mut self, pattern: u16, length: u8) {
            for i in 0..length {
                let bit = (pattern >> (15 - i)) & 1;
                self.acc = (self.acc << 1) | bit as u32;
                self.nbits += 1;
                if self.nbits == 8 {
                    self.bytes.push(self.acc as u8);
                    self.acc = 0;
                    self.nbits = 0;
                }
            }
        }

        fn push_raw(&mut self, value: u32, length: u8) {
            for i in (0..length).rev() {
                let bit = (value >> i) & 1;
                self.acc = (self.acc << 1) | bit;
                self.nbits += 1;
                if self.nbits == 8 {
                    self.bytes.push(self.acc as u8);
                    self.acc = 0;
                    self.nbits = 0;
                }
            }
        }

        fn finish(mut self) -> Vec<u8> {
            if self.nbits > 0 {
                self.bytes.push((self.acc << (8 - self.nbits)) as u8);
            }
            self.bytes.push(0);
            self.bytes.push(0);
            self.bytes
        }
    }

    /// Codeword `(pattern, length)` for an SCFI value (from the staged
    /// `sv7-huffman-scfi` table, hand-read from its left-justified
    /// `Code` column): 0→`0x4000`/3, 1→`0x8000`/1, 2→`0x6000`/3,
    /// 3→`0x0000`/2.
    fn scfi_code(value: u8) -> (u16, u8) {
        match value {
            0 => (0x4000, 3),
            1 => (0x8000, 1),
            2 => (0x6000, 3),
            3 => (0x0000, 2),
            _ => unreachable!(),
        }
    }

    /// Codeword `(pattern, length)` for a DSCF symbol, hand-read from
    /// the staged `sv7-huffman-dscf` left-justified `Code` column.
    fn dscf_code(value: i8) -> (u16, u8) {
        match value {
            5 => (0xf800, 5),
            -4 => (0xf000, 5),
            3 => (0xe000, 4),
            -3 => (0xd000, 4),
            8 => (0xc000, 4),
            1 => (0xa000, 3),
            0 => (0x9000, 4),
            -5 => (0x8800, 5),
            7 => (0x8400, 6),
            -7 => (0x8000, 6),
            -1 => (0x6000, 3),
            2 => (0x4000, 3),
            4 => (0x3000, 4),
            6 => (0x2800, 5),
            -6 => (0x2000, 5),
            -2 => (0x0000, 3),
            _ => unreachable!("unmapped dscf symbol {value}"),
        }
    }

    #[test]
    fn scfi_0_codes_all_three_off_a_running_chain() {
        // SCFI 0: SCF[0]=prev+Δ0, SCF[1]=SCF[0]+Δ1, SCF[2]=SCF[1]+Δ2.
        let (sc, sl) = scfi_code(0);
        let (d0c, d0l) = dscf_code(2);
        let (d1c, d1l) = dscf_code(-1);
        let (d2c, d2l) = dscf_code(3);
        let mut p = BitPacker::new();
        p.push(sc, sl);
        p.push(d0c, d0l);
        p.push(d1c, d1l);
        p.push(d2c, d2l);
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let scf = decode_sv7_band_scf(&mut r, 50).unwrap();
        assert_eq!(scf.scfi, 0);
        // 50+2=52, 52-1=51, 51+3=54.
        assert_eq!(scf.indices, [52, 51, 54]);
        assert_eq!(scf.last_index(), 54);
        assert!(!scf.clamped);
    }

    #[test]
    fn scfi_1_shares_scf2_from_scf1() {
        // SCFI 1: SCF[0]=prev+Δ0, SCF[1]=SCF[0]+Δ1, SCF[2]=SCF[1].
        let (sc, sl) = scfi_code(1);
        let (d0c, d0l) = dscf_code(4);
        let (d1c, d1l) = dscf_code(2);
        let mut p = BitPacker::new();
        p.push(sc, sl);
        p.push(d0c, d0l);
        p.push(d1c, d1l);
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let scf = decode_sv7_band_scf(&mut r, 10).unwrap();
        assert_eq!(scf.scfi, 1);
        // 10+4=14, 14+2=16, SCF[2]=SCF[1]=16.
        assert_eq!(scf.indices, [14, 16, 16]);
    }

    #[test]
    fn scfi_2_copies_scf1_then_codes_scf2_off_the_copy() {
        // SCFI 2: SCF[1]=SCF[0] (copied), SCF[2]=SCF[1]+Δ — the case the
        // Layer-II slot table cannot express (SCF[2] deltas off a copy).
        let (sc, sl) = scfi_code(2);
        let (d0c, d0l) = dscf_code(1);
        let (d2c, d2l) = dscf_code(5);
        let mut p = BitPacker::new();
        p.push(sc, sl);
        p.push(d0c, d0l);
        p.push(d2c, d2l);
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let scf = decode_sv7_band_scf(&mut r, 20).unwrap();
        assert_eq!(scf.scfi, 2);
        // 20+1=21, SCF[1]=21, SCF[2]=21+5=26.
        assert_eq!(scf.indices, [21, 21, 26]);
    }

    #[test]
    fn scfi_3_copies_both_later_indices() {
        // SCFI 3: only SCF[0] coded; SCF[1]=SCF[0], SCF[2]=SCF[1].
        let (sc, sl) = scfi_code(3);
        let (d0c, d0l) = dscf_code(-2);
        let mut p = BitPacker::new();
        p.push(sc, sl);
        p.push(d0c, d0l);
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let scf = decode_sv7_band_scf(&mut r, 30).unwrap();
        assert_eq!(scf.scfi, 3);
        // 30-2=28, copied to all three.
        assert_eq!(scf.indices, [28, 28, 28]);
        assert_eq!(scf.last_index(), 28);
    }

    #[test]
    fn dscf_escape_reads_raw_six_bit_absolute_for_scf0() {
        // SCFI 3 (only SCF[0] coded). SCF[0] DSCF symbol = 8 (escape) ⇒
        // SCF[0] is a raw 6-bit absolute, ignoring the prev-band ref.
        let (sc, sl) = scfi_code(3);
        let (esc_c, esc_l) = dscf_code(DSCF_ESCAPE_SYMBOL);
        let mut p = BitPacker::new();
        p.push(sc, sl);
        p.push(esc_c, esc_l);
        p.push_raw(45, DSCF_ESCAPE_RAW_BITS); // raw absolute index 45
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        // Pass a wildly different prev ref to prove the escape ignores it.
        let scf = decode_sv7_band_scf(&mut r, 999).unwrap();
        assert_eq!(scf.indices, [45, 45, 45]);
    }

    #[test]
    fn dscf_escape_applies_to_a_later_coded_index_too() {
        // SCFI 0: SCF[0] delta, SCF[1] = escape→raw absolute, SCF[2]
        // deltas off the escaped SCF[1].
        let (sc, sl) = scfi_code(0);
        let (d0c, d0l) = dscf_code(3);
        let (esc_c, esc_l) = dscf_code(DSCF_ESCAPE_SYMBOL);
        let (d2c, d2l) = dscf_code(-1);
        let mut p = BitPacker::new();
        p.push(sc, sl);
        p.push(d0c, d0l);
        p.push(esc_c, esc_l);
        p.push_raw(40, DSCF_ESCAPE_RAW_BITS);
        p.push(d2c, d2l);
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let scf = decode_sv7_band_scf(&mut r, 0).unwrap();
        // 0+3=3; SCF[1]=raw 40; SCF[2]=40-1=39.
        assert_eq!(scf.indices, [3, 40, 39]);
    }

    #[test]
    fn prev_band_scf2_threads_into_next_band_scf0() {
        // Two bands, both SCFI 3 (single coded SCF). Band 1's SCF[0]
        // must delta off band 0's SCF[2].
        let (sc, sl) = scfi_code(3);
        let (b0c, b0l) = dscf_code(5);
        let (b1c, b1l) = dscf_code(2);
        let mut p = BitPacker::new();
        p.push(sc, sl);
        p.push(b0c, b0l);
        p.push(sc, sl);
        p.push(b1c, b1l);
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let b0 = decode_sv7_band_scf(&mut r, 0).unwrap();
        assert_eq!(b0.indices, [5, 5, 5]);
        let b1 = decode_sv7_band_scf(&mut r, b0.last_index()).unwrap();
        // Band 1 SCF[0] = 5 (prev SCF[2]) + 2 = 7.
        assert_eq!(b1.indices, [7, 7, 7]);
    }

    #[test]
    fn clamp_flag_trips_when_a_running_delta_chain_exceeds_threshold() {
        // A single escape's raw field tops out at 63 (< 1024), so the
        // clamp can only trip through an accumulated delta chain. Thread
        // a large prev-band reference into SCFI 3 (single coded SCF, a
        // +5 delta): index = 1024 + 5 = 1029 > 1024 ⇒ clamped.
        let (sc, sl) = scfi_code(3);
        let (d0c, d0l) = dscf_code(5);
        let mut p = BitPacker::new();
        p.push(sc, sl);
        p.push(d0c, d0l);
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let scf = decode_sv7_band_scf(&mut r, SCF_CLAMP_THRESHOLD).unwrap();
        assert_eq!(scf.indices, [1029, 1029, 1029]);
        assert!(scf.clamped);

        // And a chain that stays at/below 1024 must NOT set the flag.
        let (d0c, d0l) = dscf_code(-1);
        let mut p = BitPacker::new();
        p.push(sc, sl);
        p.push(d0c, d0l);
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let scf = decode_sv7_band_scf(&mut r, SCF_CLAMP_THRESHOLD).unwrap();
        assert_eq!(scf.indices, [1023, 1023, 1023]);
        assert!(!scf.clamped);
    }

    #[test]
    fn dscf_reads_for_scfi_matches_case_table() {
        assert_eq!(dscf_reads_for_scfi(0), Some(3));
        assert_eq!(dscf_reads_for_scfi(1), Some(2));
        assert_eq!(dscf_reads_for_scfi(2), Some(2));
        assert_eq!(dscf_reads_for_scfi(3), Some(1));
        assert_eq!(dscf_reads_for_scfi(4), None);
    }

    #[test]
    fn propagates_eof_on_starved_dscf() {
        // SCFI 0 promises 3 DSCF reads but only one codeword is present
        // with no trailing peek padding.
        let (sc, sl) = scfi_code(0);
        let (d0c, d0l) = dscf_code(1);
        let mut p = BitPacker::new();
        p.push(sc, sl);
        p.push(d0c, d0l);
        // Flush without the two trailing zero bytes ⇒ later peeks starve.
        let mut bytes = p.bytes.clone();
        if p.nbits > 0 {
            bytes.push((p.acc << (8 - p.nbits)) as u8);
        }
        let mut r = Sv7BitReader::new(&bytes);
        assert_eq!(decode_sv7_band_scf(&mut r, 0), Err(Error::UnexpectedEof));
    }
}
