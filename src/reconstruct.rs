//! SV7 §2.6 sample reconstruction primitives.
//!
//! Wires the per-sample dequantisation step that follows the per-band
//! level decode of [`crate::sv7_band_decode`]. The structural spec
//! §2.6 (`docs/audio/musepack/musepack-sv7-sv8-spec.md`) describes the
//! reconstruction as: "requantise each sample by the quantiser implied
//! by its `band_type`, multiply by the band/granule scalefactor [...],
//! undo M/S where `msflag` set, then run the inherited 32-band
//! synthesis subband filter". The constants for the requantise step
//! are already wired in [`crate::requant`]:
//!
//! - `QUANTIZER_OFFSET_D[i]` — integer offset `D` per indexed band
//!   entry (number of quantiser steps = `2 * D + 1`).
//! - `DEQUANT_COEFFICIENT_C[i]` — dequant coefficient
//!   `C = 65536 / (2 * D + 1)`.
//!
//! This module wires the per-sample dequantise step (the product
//! `centred_level * C / 65536`) **and** the per-granule scalefactor
//! multiply that follows it (each band's 36 samples split into 3
//! granules of 12, each granule scaled by its own SCF index relative to
//! a shared anchor — see [`apply_granule_scf_relative`] /
//! [`reconstruct_band_with_granule_scf`]). The remaining §2.6 steps —
//! M/S undo and the synthesis filterbank — are not yet wired; M/S undo
//! and the synthesis-window `D_i` table are documented gaps (the latter
//! lives in the ISO Layer-II PDF referenced by the spec).
//!
//! # Centring convention
//!
//! Two cases come out of the §2.5 per-band sample decode:
//!
//! - **Huffman path** (band_types 3..=7): the staged Q3..Q7 tables
//!   already produce signed `i8` levels in `-D..=D` (e.g. band_type 3
//!   has `D = 3` so values are `-3..=3`; band_type 7 has `D = 31` so
//!   values are `-31..=31`). These levels are *already centred*;
//!   no further centring is needed before the dequant multiply.
//! - **Linear-PCM escape path** (band_types 8..=17): the raw level
//!   read off the bitstream is *unsigned* in `0..=2*D`. The §2.5
//!   prose specifies the "linear quantiser" produces a centred level
//!   in `-D..=D`; the centring step is therefore the subtraction
//!   `centred = raw_unsigned - D`. This module exposes the centring
//!   step as a separate function so the PCM-escape decoder can
//!   convert its `[i32; 36]` raw-level buffer into a centred
//!   `[i32; 36]` buffer before dequantising.
//!
//! The CNS / noise-substitution path (band_type == -1) is handled
//! separately by [`crate::cns`] — those samples come out of the PRNG
//! already in `-510..=510` and use the CNS dequant constant at
//! `DEQUANT_COEFFICIENT_C[0]`.
//!
//! # Where the SCF multiply lives
//!
//! §2.6's structural step is `sample * C * scf_gain`. The full
//! 256-entry scalefactor-index → *absolute* gain table needs an
//! anchor point (the absolute gain at the reference index) that the
//! structural prose does not pin down. What **is** independently
//! specified — by the `scf-step-ratio.meta` line "SCF table built as
//! a geometric sequence around index 1: f *= 0.83298066476582673961
//! per step (... 256 indices)" — is the *geometric* relation between
//! any two SCF indices: the gain at index `n` over the gain at index
//! `m` is exactly [`crate::requant::SCF_STEP_RATIO`]`^(n − m)`. That
//! ratio is **anchor-independent**, so the *relative* SCF gain
//! between two indices (e.g. between two granules of the same band,
//! whose SCF indices the [`crate::scf`] decoder reconstructs as
//! deltas) is fully determined even with the absolute anchor still
//! GAP.
//!
//! This module therefore wires the *relative* SCF gain ladder
//! ([`scf_relative_gain`], [`scf_gain_relative_to_anchor`],
//! [`apply_scf_relative`]) but leaves the *absolute* anchored gain
//! table (which needs the GAP reference-index value) to a later
//! round. A caller that only needs to equalise granules *within* a
//! band — applying the per-granule SCF index difference on top of a
//! shared base — can do so exactly with the relative ladder; the
//! absolute output then differs from a fully-anchored decode by one
//! global constant scale.

use crate::requant::{band_type_index, DEQUANT_COEFFICIENT_C, QUANTIZER_OFFSET_D, SCF_STEP_RATIO};
use crate::sv7_band_decode::SAMPLES_PER_BAND;
use crate::{Error, Result};

/// Number of distinct scalefactor (SCF) indices in the §2.6 geometric
/// gain ladder.
///
/// Pinned by the `scf-step-ratio.meta` notes line: "SCF table built as
/// a geometric sequence around index 1: f *= 0.83298066476582673961
/// per step (handles +1.58..-98.41 dB, **256 indices**)". The valid
/// SCF index range is therefore `0..=255`.
pub const SCF_INDEX_COUNT: usize = 256;

/// Number of 12-sample granules a band's 36 subband samples split into.
///
/// §1 (`musepack-sv7-sv8-spec.md`): each subband carries 36 samples per
/// frame, "internally grouped as 3 granules of 12 samples". The §2.4
/// scalefactor layer transmits up to one SCF index per granule (the
/// Layer-II SCFSI inheritance, see [`crate::scf`]). Kept in lockstep
/// with the scf layer's `SCF_GRANULES_PER_BAND` by a compile-time
/// assertion at the end of this module.
pub const GRANULES_PER_BAND: usize = 3;

/// Number of subband samples in one SCF granule (`36 / 3`).
///
/// The first 12 samples belong to granule 0, the next 12 to granule 1,
/// the last 12 to granule 2 — the contiguous time order in which the
/// §2.5 per-band sample decode emits them.
pub const SAMPLES_PER_GRANULE: usize = SAMPLES_PER_BAND / GRANULES_PER_BAND;

/// Multiplicative SCF gain at index `to` *relative to* index `from`.
///
/// Returns [`crate::requant::SCF_STEP_RATIO`]`^(to − from)`. This is
/// the **anchor-independent** part of the §2.6 SCF gain table: the
/// `scf-step-ratio.meta` line fixes the geometric step (downward:
/// `gain[n] / gain[n − 1] == SCF_STEP_RATIO`; upward: the reciprocal),
/// so the ratio between any two indices is fully determined even
/// though the *absolute* gain at the reference index is GAP.
///
/// `from == to` yields exactly `1.0`. A higher index than the anchor
/// (`to > from`) is "downward" in the stored direction, so it yields a
/// gain **below** `1.0` (`SCF_STEP_RATIO < 1.0` per step); a lower
/// index yields a gain **above** `1.0` (the reciprocal,
/// `≈ 1.2005` per step).
///
/// Total over the whole `u8 × u8` domain: both indices are `u8`, hence
/// structurally within the `0..=255` [`SCF_INDEX_COUNT`] ladder, so no
/// range error is reachable and the function is infallible.
#[inline]
pub fn scf_relative_gain(from: u8, to: u8) -> f64 {
    let exponent = i32::from(to) - i32::from(from);
    SCF_STEP_RATIO.powi(exponent)
}

/// Fill `out` with the SCF gains of indices `0..=255` *relative to*
/// the `anchor` index, i.e. `out[i] == scf_relative_gain(anchor, i)`.
///
/// `out[anchor]` is exactly `1.0`. The ladder is monotonically
/// **decreasing** in `i` (since each upward index step multiplies by
/// `SCF_STEP_RATIO < 1.0` in the stored "downward" direction — higher
/// index = quieter), matching the `scf-step-ratio.meta` "+1.58..−98.41
/// dB" span across the 256 indices.
pub fn scf_gain_relative_to_anchor(anchor: u8, out: &mut [f64; SCF_INDEX_COUNT]) {
    for (i, slot) in out.iter_mut().enumerate() {
        *slot = scf_relative_gain(anchor, i as u8);
    }
}

/// Scale a 36-sample dequantised band in place by the relative SCF
/// gain moving from `from_index` to `to_index`.
///
/// Equivalent to multiplying every sample by
/// [`scf_relative_gain`]`(from_index, to_index)`. This applies a
/// per-granule SCF index *difference* (the [`crate::scf`] decoder
/// reconstructs the three per-granule indices as deltas off a shared
/// base) without needing the still-GAP absolute anchor: the result is
/// correct up to the single global constant that the absolute table
/// would supply.
pub fn apply_scf_relative(from_index: u8, to_index: u8, band: &mut [f64; SAMPLES_PER_BAND]) {
    let gain = scf_relative_gain(from_index, to_index);
    for slot in band.iter_mut() {
        *slot *= gain;
    }
}

/// Scale a 36-sample dequantised band in place by a **per-granule** SCF
/// gain, applying each of the three granule SCF indices to its own
/// contiguous 12-sample slice, *relative to* a shared `anchor` index.
///
/// This is the §2.6 "multiply by the band/granule scalefactor" step at
/// the granularity the §2.4 scalefactor layer actually transmits it:
/// one SCF index per 12-sample granule (the Layer-II SCFSI inheritance,
/// §1). `granule_scf[g]` is the absolute SCF index for granule `g`, as
/// reconstructed by the [`crate::scf`] decoder
/// ([`crate::scf::BandScf::indices`]). Samples `0..12` are scaled by
/// `scf_relative_gain(anchor, granule_scf[0])`, `12..24` by
/// granule 1's gain, and `24..36` by granule 2's.
///
/// # Why `anchor`-relative
///
/// The §2.6 absolute scalefactor gain table needs a reference-index
/// anchor value that the structural prose leaves GAP (see the module
/// docs). What **is** fully specified is the geometric ratio between any
/// two SCF indices, so this function multiplies each granule by its gain
/// *relative to* the caller's `anchor`. Passing the band's own minimum
/// (or any fixed) SCF index as `anchor` makes the three granules carry
/// their exact relative loudness; the whole band then differs from a
/// fully-anchored decode by the single global constant the GAP anchor
/// would supply. The relative loudness *between granules* and *between
/// bands sharing an anchor* is exact.
///
/// In-place. Infallible: `anchor` and every `granule_scf` entry are
/// `u8`, hence structurally inside the `0..=255` SCF ladder.
pub fn apply_granule_scf_relative(
    anchor: u8,
    granule_scf: [u8; GRANULES_PER_BAND],
    band: &mut [f64; SAMPLES_PER_BAND],
) {
    for (g, &scf) in granule_scf.iter().enumerate() {
        let gain = scf_relative_gain(anchor, scf);
        let start = g * SAMPLES_PER_GRANULE;
        for slot in band[start..start + SAMPLES_PER_GRANULE].iter_mut() {
            *slot *= gain;
        }
    }
}

/// Multiplicative SCF gain at signed index `to` *relative to* signed
/// index `from`, for SCF indices that fall **outside** the unsigned
/// `0..=255` [`SCF_INDEX_COUNT`] ladder.
///
/// Identical geometry to [`scf_relative_gain`] —
/// [`crate::requant::SCF_STEP_RATIO`]`^(to − from)` — but accepts the
/// signed `i32` indices the SV8 §6.3 DSCF fold produces. That fold,
/// `SCF = ((prev − 25 + delta) & 127) − 6`, recenters the 7-bit ring by
/// `−6`, so a reconstructed SV8 SCF index lies in the signed range
/// `−6..=121` rather than the SV7 `u8` ladder. Because the SCF gain is
/// purely geometric (the `scf-step-ratio.meta` step is anchor- and
/// sign-independent), the ratio between two indices is well-defined for
/// any integers; this entry point just lifts the `u8` bound so the SV8
/// signed indices can be used directly without an offset hack.
///
/// `from == to` yields exactly `1.0`; a higher `to` (downward in the
/// stored direction) yields a gain below `1.0`, a lower `to` a gain
/// above `1.0` — the same orientation as [`scf_relative_gain`].
#[inline]
pub fn scf_relative_gain_signed(from: i32, to: i32) -> f64 {
    SCF_STEP_RATIO.powi(to - from)
}

/// Scale a 36-sample dequantised band in place by a **per-granule** SCF
/// gain, applying each of the three signed granule SCF indices to its
/// own contiguous 12-sample slice, *relative to* a shared signed
/// `anchor` index.
///
/// The signed-index counterpart of [`apply_granule_scf_relative`] for
/// the SV8 path: the §6.3 DSCF fold reconstructs each granule SCF index
/// in the signed range `−6..=121`, so this variant takes `i32` indices
/// and an `i32` anchor and uses [`scf_relative_gain_signed`]. Samples
/// `0..12` are scaled by granule 0's gain relative to `anchor`, `12..24`
/// by granule 1's, and `24..36` by granule 2's.
///
/// In-place. Infallible: the geometric gain is total for any integer
/// indices (the relative-loudness-only convention of
/// [`apply_granule_scf_relative`] applies — the result is exact up to
/// the single GAP global anchor constant).
pub fn apply_granule_scf_relative_signed(
    anchor: i32,
    granule_scf: [i32; GRANULES_PER_BAND],
    band: &mut [f64; SAMPLES_PER_BAND],
) {
    for (g, &scf) in granule_scf.iter().enumerate() {
        let gain = scf_relative_gain_signed(anchor, scf);
        let start = g * SAMPLES_PER_GRANULE;
        for slot in band[start..start + SAMPLES_PER_GRANULE].iter_mut() {
            *slot *= gain;
        }
    }
}

/// Full §2.6 per-band reconstruction for the entropy-Huffman /
/// PCM-escape range: dequantise 36 already-centred levels by the
/// `band_type` quantiser, then apply the three per-granule SCF gains
/// relative to `anchor` — producing the reconstructed `f64` subband
/// samples ready for M/S undo + the synthesis filterbank.
///
/// `centred` carries the signed, already-centred levels from the §2.5
/// per-band sample decode (the Q3..Q7 Huffman levels are centred as
/// decoded; the PCM-escape levels must first pass through
/// [`centre_pcm_band`]). `band_type` must be in the quantiser-bearing
/// range `0..=17`; the CNS band (`band_type == -1`) uses
/// [`reconstruct_cns_band_with_granule_scf`] instead.
///
/// Returns [`Error::UnsupportedBandType`] for a `band_type` outside
/// `0..=17`.
pub fn reconstruct_band_with_granule_scf(
    band_type: i8,
    centred: &[i32; SAMPLES_PER_BAND],
    anchor: u8,
    granule_scf: [u8; GRANULES_PER_BAND],
    out: &mut [f64; SAMPLES_PER_BAND],
) -> Result<()> {
    dequantise_band(band_type, centred, out)?;
    apply_granule_scf_relative(anchor, granule_scf, out);
    Ok(())
}

/// Full §2.6 per-band reconstruction for the CNS / noise band
/// (`band_type == -1`): dequantise the PRNG samples by the CNS
/// coefficient (see [`dequantise_cns_band`]), then apply the three
/// per-granule SCF gains relative to `anchor`.
///
/// The CNS path has no `band_type` in the quantiser range, so it gets
/// its own entry point. Infallible — the CNS dequant is total and the
/// SCF gain is `u8`-bounded.
pub fn reconstruct_cns_band_with_granule_scf(
    cns_levels: &[i32; SAMPLES_PER_BAND],
    anchor: u8,
    granule_scf: [u8; GRANULES_PER_BAND],
    out: &mut [f64; SAMPLES_PER_BAND],
) {
    dequantise_cns_band(cns_levels, out);
    apply_granule_scf_relative(anchor, granule_scf, out);
}

/// End-to-end §2.6 reconstruction of one SV7 band, from the unified
/// `[i32; 36]` level buffer that [`crate::sv7_band_decode::decode_sv7_band`]
/// emits, straight to reconstructed `f64` subband samples.
///
/// This is the integrating entry point that joins the §2.5 per-band
/// sample decode to the §2.6 dequant + per-granule SCF multiply. It
/// branches on the [`crate::sv7_band_decode::band_type_case`] classifier
/// so each arm's level convention is handled correctly **before** the
/// dequant:
///
/// - **Empty** (`band_type == 0`): the levels are all zero; dequant of
///   zero is zero, and the SCF multiply leaves them zero. The band
///   reconstructs to silence regardless of `anchor` / `granule_scf`.
/// - **CNS** (`band_type == -1`): PRNG levels dequantised by the CNS
///   coefficient (`DEQUANT_COEFFICIENT_C[0]`), via
///   [`reconstruct_cns_band_with_granule_scf`].
/// - **Grouped / per-sample Huffman** (`band_type` 1..=7): the decoded
///   levels are already signed-centred; dequantised directly.
/// - **PCM-escape** (`band_type` 8..=17): the decoded levels are raw
///   *unsigned* (`0..=2D`); centred in place by subtracting `D`
///   ([`centre_pcm_band`]) before the dequant.
///
/// After dequant, the three per-granule SCF gains are applied relative
/// to `anchor` ([`apply_granule_scf_relative`]). `granule_scf[g]` is the
/// absolute SCF index for granule `g`, as reconstructed by the
/// [`crate::scf`] decoder; `anchor` carries the still-GAP absolute
/// reference (see the module docs — relative loudness between granules
/// and between anchor-sharing bands is exact).
///
/// `levels` is taken by value into a local scratch buffer so the
/// in-place PCM centring does not mutate the caller's decode output.
///
/// Returns [`Error::UnsupportedBandType`] for a `band_type` outside the
/// structurally-enumerated `-1..=17` range (the classifier's
/// `OutOfRange` arm).
pub fn reconstruct_sv7_band_from_levels(
    band_type: i8,
    levels: &[i32; SAMPLES_PER_BAND],
    anchor: u8,
    granule_scf: [u8; GRANULES_PER_BAND],
    out: &mut [f64; SAMPLES_PER_BAND],
) -> Result<()> {
    use crate::sv7_band_decode::{band_type_case, BandDecodeCase};

    match band_type_case(band_type) {
        BandDecodeCase::Cns => {
            reconstruct_cns_band_with_granule_scf(levels, anchor, granule_scf, out);
            Ok(())
        }
        BandDecodeCase::PcmEscape => {
            // Raw unsigned levels → centre in a scratch copy, then
            // dequantise + apply the per-granule SCF.
            let mut centred = *levels;
            centre_pcm_band(band_type, &mut centred)?;
            reconstruct_band_with_granule_scf(band_type, &centred, anchor, granule_scf, out)
        }
        BandDecodeCase::Empty
        | BandDecodeCase::Grouped3
        | BandDecodeCase::Grouped2
        | BandDecodeCase::HuffmanPerSample => {
            // Already-centred levels (zero for Empty); dequantise in the
            // quantiser-bearing 0..=17 range, then apply per-granule SCF.
            reconstruct_band_with_granule_scf(band_type, levels, anchor, granule_scf, out)
        }
        BandDecodeCase::OutOfRange => Err(Error::UnsupportedBandType(band_type)),
    }
}

// ───────────────── corpus-pinned absolute reconstruction ─────────────────

/// §5.3 out-of-range sentinel bound re-exported for the absolute path:
/// a granule whose SCF index exceeds this reconstructs to silence.
pub use crate::sv7_scf_decode::SCF_CLAMP_THRESHOLD as SCF_ABSOLUTE_SENTINEL;

/// The **absolute** SV7 per-granule gain, pinned by the fixture corpus:
/// `gain(idx) = SCF_STEP_RATIO^(idx − 1)` in the signed-16-bit output
/// domain.
///
/// Empirically resolved (previously the §2.6 GAP anchor): decoding the
/// four independent mppenc streams under `tests/fixtures/sv7/` with the
/// full pipeline `level × C[band] × gain(scf)` → synthesis reproduces
/// FFmpeg's `mpc7` s16 oracle to within ±1 LSB with ~75% of samples
/// bit-exact, and the fitted global scale equals `65536 / SCF_STEP_RATIO`
/// to five significant figures — i.e. the ladder is anchored at **index
/// 1 = unity gain** (matching the `scf-step-ratio.meta` "geometric
/// sequence around index 1" phrasing) and the `C` coefficients are used
/// *unnormalised* (no `/65536`), placing the reconstruction directly in
/// the s16 sample domain.
///
/// A granule index above [`SCF_ABSOLUTE_SENTINEL`] (the §5.3 "exceeding
/// 1024 ⇒ sentinel" clamp) yields gain `0.0` (silent).
#[inline]
pub fn sv7_absolute_scf_gain(scf_index: i32) -> f64 {
    if scf_index > SCF_ABSOLUTE_SENTINEL {
        return 0.0;
    }
    SCF_STEP_RATIO.powi(scf_index - 1)
}

/// Full corpus-pinned absolute reconstruction of one SV7 band: from the
/// unified `[i32; 36]` level buffer to s16-domain `f64` subband samples,
/// `out[i] = level[i] × C[band_type + 1] × gain(granule_scf[i / 12])`.
///
/// Level conventions per arm match [`reconstruct_sv7_band_from_levels`]:
/// PCM-escape levels (`band_type` 8..=17) are raw-unsigned and centred
/// here; Huffman / grouped levels are already centred; CNS (`-1`) uses
/// the `DEQUANT_COEFFICIENT_C[0]` coefficient (per the structural spec
/// the noise band is "scaled by the band's scalefactor", so the granule
/// gains apply to it exactly like a coded band). The CNS band's *wire
/// participation* in the SCF layer is fixture-proven
/// (`tests/sv7_cns_corpus.rs`); its absolute noise gain cannot be
/// cross-checked against the corpus oracle, whose noise generator
/// demonstrably differs from the staged one (see the fixture suite
/// docs).
///
/// `granule_scf` carries the three signed per-granule SCF indices from
/// the §5.3 decode ([`crate::sv7_scf_decode::Sv7BandScf::indices`]).
///
/// # Errors
///
/// [`Error::UnsupportedBandType`] for a `band_type` outside `-1..=17`.
pub fn reconstruct_sv7_band_absolute(
    band_type: i8,
    levels: &[i32; SAMPLES_PER_BAND],
    granule_scf: [i32; GRANULES_PER_BAND],
    out: &mut [f64; SAMPLES_PER_BAND],
) -> Result<()> {
    let idx = band_type_index(band_type).ok_or(Error::UnsupportedBandType(band_type))?;
    let c = DEQUANT_COEFFICIENT_C[idx];
    // PCM-escape raw levels are unsigned; centre by subtracting D.
    let centre = if (8..=17).contains(&band_type) {
        QUANTIZER_OFFSET_D[idx] as i32
    } else {
        0
    };
    for (g, &scf) in granule_scf.iter().enumerate() {
        let gain = c * sv7_absolute_scf_gain(scf);
        let start = g * SAMPLES_PER_GRANULE;
        for k in start..start + SAMPLES_PER_GRANULE {
            out[k] = f64::from(levels[k] - centre) * gain;
        }
    }
    Ok(())
}

/// Compile-time sanity: the reconstruct-layer granule geometry must
/// match the scf-layer's `SCF_GRANULES_PER_BAND`, and the 3×12 split
/// must tile the full 36-sample band exactly.
const _: () = {
    assert!(GRANULES_PER_BAND == crate::scf::SCF_GRANULES_PER_BAND);
    assert!(GRANULES_PER_BAND * SAMPLES_PER_GRANULE == SAMPLES_PER_BAND);
};

/// Divisor in the §2.6 dequant relation `sample = centred_level * C / 65536`.
///
/// Tied to the requantiser table relation `C = 65536 / (2 * D + 1)`
/// (see [`crate::requant::DEQUANT_COEFFICIENT_C`]). Stored as `f64`
/// because the dequant arithmetic is floating point.
pub const DEQUANT_DIVISOR: f64 = 65536.0;

/// Centre a single PCM-escape raw level by subtracting `D` for the
/// given `band_type`. Returns [`Error::UnsupportedBandType`] if
/// `band_type` is outside the linear-PCM escape range `8..=17`.
///
/// The escape ladder packs `band_type - 1` unsigned bits per sample
/// into the raw level; the centred result is the signed value in the
/// inclusive range `-D..=D` per the §2.5 / §2.6 "linear quantiser"
/// description.
#[inline]
pub fn centre_pcm_level(band_type: i8, raw_unsigned: i32) -> Result<i32> {
    if !(8..=17).contains(&band_type) {
        return Err(Error::UnsupportedBandType(band_type));
    }
    // band_type in 8..=17 -> index in 9..=18.
    let idx = band_type_index(band_type).ok_or(Error::UnsupportedBandType(band_type))?;
    let d = QUANTIZER_OFFSET_D[idx] as i32;
    Ok(raw_unsigned - d)
}

/// Centre an entire 36-sample PCM-escape band in place.
///
/// Subtracts `D = QUANTIZER_OFFSET_D[band_type + 1]` from every
/// sample of `buf`. Returns [`Error::UnsupportedBandType`] for a
/// `band_type` outside `8..=17`.
///
/// The result satisfies `buf[i] ∈ -D..=D` whenever the input was a
/// valid `(band_type - 1)`-bit unsigned raw level (i.e. in
/// `0..=2D`). Inputs outside that range are not rejected here — the
/// PCM-escape reader bounds the raw level structurally, and bounds
/// checking it again would make the function panicky on legitimate
/// CNS-style "wider range" callers.
pub fn centre_pcm_band(band_type: i8, buf: &mut [i32; SAMPLES_PER_BAND]) -> Result<()> {
    if !(8..=17).contains(&band_type) {
        return Err(Error::UnsupportedBandType(band_type));
    }
    let idx = band_type_index(band_type).ok_or(Error::UnsupportedBandType(band_type))?;
    let d = QUANTIZER_OFFSET_D[idx] as i32;
    for slot in buf.iter_mut() {
        *slot -= d;
    }
    Ok(())
}

/// Dequantise a single already-centred sample level for a given
/// `band_type` in the normal entropy/PCM range `0..=17`.
///
/// Returns `centred_level * C / 65536` where
/// `C = DEQUANT_COEFFICIENT_C[band_type + 1]`. The CNS / noise
/// band (signed `band_type == -1`) has its own dequant path keyed off
/// `DEQUANT_COEFFICIENT_C[0]`; pass `band_type == -1` to use it.
///
/// Returns [`Error::UnsupportedBandType`] for `band_type` outside
/// `-1..=17`.
#[inline]
pub fn dequantise_sample(band_type: i8, centred_level: i32) -> Result<f64> {
    let idx = band_type_index(band_type).ok_or(Error::UnsupportedBandType(band_type))?;
    let c = DEQUANT_COEFFICIENT_C[idx];
    Ok(centred_level as f64 * c / DEQUANT_DIVISOR)
}

/// Dequantise a 36-sample band of already-centred levels (Huffman
/// path: `band_type` in `3..=7`; PCM-escape path: caller first runs
/// [`centre_pcm_band`] then this) into `out`.
///
/// Returns [`Error::UnsupportedBandType`] for a `band_type` outside
/// the structurally-documented `0..=17` quantiser-bearing range.
/// Use [`dequantise_cns_band`] for the CNS / noise band
/// (`band_type == -1`).
pub fn dequantise_band(
    band_type: i8,
    centred: &[i32; SAMPLES_PER_BAND],
    out: &mut [f64; SAMPLES_PER_BAND],
) -> Result<()> {
    if !(0..=17).contains(&band_type) {
        return Err(Error::UnsupportedBandType(band_type));
    }
    let idx = band_type_index(band_type).ok_or(Error::UnsupportedBandType(band_type))?;
    let c = DEQUANT_COEFFICIENT_C[idx];
    for (dst, &src) in out.iter_mut().zip(centred.iter()) {
        *dst = src as f64 * c / DEQUANT_DIVISOR;
    }
    Ok(())
}

/// Dequantise a 36-sample band of Huffman-coded levels (`band_type`
/// 3..=7). Convenience wrapper over [`dequantise_band`] that
/// accepts the `[i8; 36]` shape returned by
/// [`crate::sv7_band_decode::decode_huffman_band`] — the Q3..Q7
/// tables produce signed `i8` levels that are already centred.
pub fn dequantise_huffman_band(
    band_type: i8,
    huffman_levels: &[i8; SAMPLES_PER_BAND],
    out: &mut [f64; SAMPLES_PER_BAND],
) -> Result<()> {
    if !(3..=7).contains(&band_type) {
        return Err(Error::UnsupportedBandType(band_type));
    }
    let idx = band_type_index(band_type).ok_or(Error::UnsupportedBandType(band_type))?;
    let c = DEQUANT_COEFFICIENT_C[idx];
    for (dst, &src) in out.iter_mut().zip(huffman_levels.iter()) {
        *dst = src as f64 * c / DEQUANT_DIVISOR;
    }
    Ok(())
}

/// Dequantise a 36-sample CNS / noise band (`band_type == -1`).
///
/// The CNS PRNG (see [`crate::cns::CnsPrng`]) emits samples in
/// `-510..=510`; this multiplies them by the CNS dequant coefficient
/// at `DEQUANT_COEFFICIENT_C[0]` (`= 111.285962475327`, per the
/// `cns-prng-params.meta` notes line, anchored to
/// `32768 / 2 / 255 * sqrt(3)`).
pub fn dequantise_cns_band(
    cns_levels: &[i32; SAMPLES_PER_BAND],
    out: &mut [f64; SAMPLES_PER_BAND],
) {
    let c = DEQUANT_COEFFICIENT_C[0];
    for (dst, &src) in out.iter_mut().zip(cns_levels.iter()) {
        *dst = src as f64 * c / DEQUANT_DIVISOR;
    }
}

/// Helper: return the `D` (= `QUANTIZER_OFFSET_D[band_type + 1]`)
/// associated with a PCM-escape `band_type` in `8..=17`. Returns
/// `None` outside the PCM-escape range.
#[inline]
pub fn pcm_escape_d(band_type: i8) -> Option<i32> {
    if !(8..=17).contains(&band_type) {
        return None;
    }
    let idx = band_type_index(band_type)?;
    Some(QUANTIZER_OFFSET_D[idx] as i32)
}

/// Compile-time sanity: keep the dequant divisor synced with the
/// requantiser-table relation. The CSV-extracted coefficient for
/// band_type 0 (index 1) is exactly `65536.0`; if either value moves,
/// the spec relation no longer holds.
#[inline]
fn _sanity_band0_c_is_divisor() {
    // Not exposed; just a guarded local assertion the test below
    // also verifies.
    assert!((DEQUANT_COEFFICIENT_C[1] - DEQUANT_DIVISOR).abs() < 1e-9);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cns::CnsPrng;
    use crate::requant::QUANTIZER_OFFSET_D;

    // ─── PCM centring ──────────────────────────────────────

    #[test]
    fn centre_pcm_level_subtracts_d_for_band_type_8() {
        // band_type 8: D = QUANTIZER_OFFSET_D[9] = 63. Raw range
        // 0..=126 (7 unsigned bits). Centred range -63..=63.
        let d = QUANTIZER_OFFSET_D[9] as i32;
        assert_eq!(d, 63);
        assert_eq!(centre_pcm_level(8, 0).unwrap(), -d);
        assert_eq!(centre_pcm_level(8, d).unwrap(), 0);
        assert_eq!(centre_pcm_level(8, 2 * d).unwrap(), d);
    }

    #[test]
    fn centre_pcm_level_subtracts_d_for_band_type_17() {
        // band_type 17: D = QUANTIZER_OFFSET_D[18] = 32767.
        let d = QUANTIZER_OFFSET_D[18] as i32;
        assert_eq!(d, 32767);
        assert_eq!(centre_pcm_level(17, 0).unwrap(), -d);
        assert_eq!(centre_pcm_level(17, d).unwrap(), 0);
        assert_eq!(centre_pcm_level(17, 2 * d).unwrap(), d);
    }

    #[test]
    fn centre_pcm_level_rejects_out_of_range_band_types() {
        for bt in [-2_i8, -1, 0, 1, 2, 3, 7, 18, i8::MAX, i8::MIN] {
            assert!(matches!(
                centre_pcm_level(bt, 0),
                Err(Error::UnsupportedBandType(_)),
            ));
        }
    }

    #[test]
    fn centre_pcm_band_in_place_round_trip() {
        // Ramp 0..=126 (band_type 8, 7 unsigned bits): after centring,
        // values should be -63..=63 in order.
        let mut buf = [0_i32; SAMPLES_PER_BAND];
        for (i, slot) in buf.iter_mut().enumerate() {
            *slot = (i as i32) * 2;
        }
        // Highest in [0, 2D] for band_type 8 (2D=126) is buf[35]=70.
        // We only need to verify the subtraction is applied
        // uniformly, not assert clamping.
        centre_pcm_band(8, &mut buf).unwrap();
        let d = QUANTIZER_OFFSET_D[9] as i32;
        assert_eq!(d, 63);
        for (i, &v) in buf.iter().enumerate() {
            assert_eq!(v, (i as i32) * 2 - d);
        }
    }

    #[test]
    fn centre_pcm_band_rejects_out_of_range_band_types() {
        let mut buf = [0_i32; SAMPLES_PER_BAND];
        for bt in [-1_i8, 0, 3, 7, 18] {
            assert!(matches!(
                centre_pcm_band(bt, &mut buf),
                Err(Error::UnsupportedBandType(_)),
            ));
        }
    }

    // ─── Single-sample dequantisation ──────────────────────

    #[test]
    fn dequantise_sample_band_type_0_is_identity_scaled_by_c() {
        // band_type 0: D = 0, C = 65536. centred level can only be 0.
        // For other inputs (e.g. a stress test), C/65536 = 1, so
        // dequant == centred_level.
        let val = dequantise_sample(0, 0).unwrap();
        assert!((val - 0.0).abs() < 1e-12);
        let val = dequantise_sample(0, 5).unwrap();
        assert!((val - 5.0).abs() < 1e-9, "C / 65536 should be 1.0");
    }

    #[test]
    fn dequantise_sample_band_type_3_uses_correct_c() {
        // band_type 3: D = QUANTIZER_OFFSET_D[4] = 3 (one above
        // band_type 2's D=2 in the entropy ladder), 2D+1 = 7,
        // C = 65536/7 ≈ 9362.285714. dequantise_sample(3, 3) =
        // 3 * C / 65536 = 3/7 ≈ 0.428571.
        let d = QUANTIZER_OFFSET_D[band_type_index(3).unwrap()] as i32;
        assert_eq!(d, 3);
        let val = dequantise_sample(3, d).unwrap();
        let expected = d as f64 / (2.0 * d as f64 + 1.0);
        assert!((val - expected).abs() < 1e-9, "got {val}, want {expected}");
        let val = dequantise_sample(3, -d).unwrap();
        assert!((val + expected).abs() < 1e-9, "got {val}");
    }

    #[test]
    fn dequantise_sample_band_type_17_uses_correct_c() {
        // band_type 17: D = 32767, 2D+1 = 65535, C = 65536/65535
        // ≈ 1.00001526. dequant of D should be ~D / 65535 * 65536 / 65536
        // = D / 65535 ≈ 0.499992...
        let val = dequantise_sample(17, 32767).unwrap();
        // Expected: 32767 * (65536/65535) / 65536 = 32767/65535 ≈ 0.49999237
        let expected = 32767.0_f64 / 65535.0;
        assert!((val - expected).abs() < 1e-9, "got {val}, want {expected}");
    }

    #[test]
    fn dequantise_sample_cns_band_uses_c0() {
        // band_type -1 -> index 0 -> C = 111.285962475327.
        // Dequant of 0 = 0; dequant of 510 = 510 * C / 65536.
        let val = dequantise_sample(-1, 0).unwrap();
        assert!(val.abs() < 1e-12);
        let val = dequantise_sample(-1, 510).unwrap();
        let expected = 510.0_f64 * 111.285962475327 / 65536.0;
        assert!((val - expected).abs() < 1e-9, "got {val}, want {expected}");
    }

    #[test]
    fn dequantise_sample_rejects_out_of_range() {
        for bt in [-2_i8, 18, i8::MAX, i8::MIN] {
            assert!(matches!(
                dequantise_sample(bt, 0),
                Err(Error::UnsupportedBandType(_)),
            ));
        }
    }

    // ─── Whole-band dequantisation ─────────────────────────

    #[test]
    fn dequantise_band_matches_single_sample_path() {
        let mut centred = [0_i32; SAMPLES_PER_BAND];
        for (i, slot) in centred.iter_mut().enumerate() {
            // Span -D..=D for band_type 5 (D=4): use signed ramp.
            *slot = (i as i32) - 18;
        }
        let mut out = [0.0_f64; SAMPLES_PER_BAND];
        dequantise_band(5, &centred, &mut out).unwrap();
        for i in 0..SAMPLES_PER_BAND {
            let expected = dequantise_sample(5, centred[i]).unwrap();
            assert!(
                (out[i] - expected).abs() < 1e-12,
                "sample {i}: got {} want {}",
                out[i],
                expected
            );
        }
    }

    #[test]
    fn dequantise_band_rejects_negative_band_type() {
        // Use dequantise_cns_band for CNS; dequantise_band only
        // handles 0..=17.
        let centred = [0_i32; SAMPLES_PER_BAND];
        let mut out = [0.0_f64; SAMPLES_PER_BAND];
        assert!(matches!(
            dequantise_band(-1, &centred, &mut out),
            Err(Error::UnsupportedBandType(_)),
        ));
        assert!(matches!(
            dequantise_band(18, &centred, &mut out),
            Err(Error::UnsupportedBandType(_)),
        ));
    }

    #[test]
    fn dequantise_huffman_band_round_trips_signed_i8() {
        // band_type 3 Q3 values lie in -D..=D = -2..=2. Build a
        // synthetic 36-sample signed pattern and dequantise.
        let mut huffman_levels = [0_i8; SAMPLES_PER_BAND];
        for (i, slot) in huffman_levels.iter_mut().enumerate() {
            *slot = ((i as i32 - 18) % 3) as i8; // values in -2..=2
        }
        let mut out = [0.0_f64; SAMPLES_PER_BAND];
        dequantise_huffman_band(3, &huffman_levels, &mut out).unwrap();
        let c = DEQUANT_COEFFICIENT_C[band_type_index(3).unwrap()];
        for i in 0..SAMPLES_PER_BAND {
            let expected = huffman_levels[i] as f64 * c / DEQUANT_DIVISOR;
            assert!((out[i] - expected).abs() < 1e-12);
        }
    }

    #[test]
    fn dequantise_huffman_band_rejects_outside_3_7() {
        let huffman_levels = [0_i8; SAMPLES_PER_BAND];
        let mut out = [0.0_f64; SAMPLES_PER_BAND];
        for bt in [-1_i8, 0, 1, 2, 8, 17, 18] {
            assert!(matches!(
                dequantise_huffman_band(bt, &huffman_levels, &mut out),
                Err(Error::UnsupportedBandType(_)),
            ));
        }
    }

    #[test]
    fn dequantise_cns_band_uses_c0_constant() {
        // Take a fresh CnsPrng walk, then dequantise.
        let mut prng = CnsPrng::new();
        let mut cns_levels = [0_i32; SAMPLES_PER_BAND];
        prng.fill_samples(&mut cns_levels);
        let mut out = [0.0_f64; SAMPLES_PER_BAND];
        dequantise_cns_band(&cns_levels, &mut out);
        let c = DEQUANT_COEFFICIENT_C[0];
        for i in 0..SAMPLES_PER_BAND {
            let expected = cns_levels[i] as f64 * c / DEQUANT_DIVISOR;
            assert!((out[i] - expected).abs() < 1e-12);
        }
        // CNS samples have a known bound -510..=510 from cns.rs; the
        // dequantised magnitude is therefore bounded by 510 * C / 65536.
        let max_mag = 510.0_f64 * c / DEQUANT_DIVISOR;
        for &v in out.iter() {
            assert!(
                v.abs() <= max_mag + 1e-9,
                "CNS dequant out of expected magnitude bound"
            );
        }
    }

    // ─── pcm_escape_d helper ───────────────────────────────

    #[test]
    fn pcm_escape_d_matches_quantizer_offset_table() {
        for bt in 8_i8..=17 {
            let d = pcm_escape_d(bt).expect("PCM range");
            let idx = band_type_index(bt).unwrap();
            assert_eq!(d, QUANTIZER_OFFSET_D[idx] as i32);
        }
        for bt in [-1_i8, 0, 7, 18] {
            assert!(pcm_escape_d(bt).is_none());
        }
    }

    // ─── Cross-module integration: PCM-escape decode -> centre -> dequant ───

    #[test]
    fn pcm_escape_decode_then_centre_then_dequant_round_trips() {
        use crate::huffman::Sv7BitReader;
        use crate::sv7_band_decode::decode_linear_pcm_band;

        // band_type 8 -> 7 bits per sample. Build a stream where each
        // sample's raw level encodes its position modulo 2D+1.
        let two_d_plus_1 = (2 * QUANTIZER_OFFSET_D[9] + 1) as u32; // 127
        let expected_raw: Vec<u32> = (0..SAMPLES_PER_BAND as u32)
            .map(|i| i % two_d_plus_1)
            .collect();
        let mut bits = Vec::new();
        let mut acc: u32 = 0;
        let mut nbits: u32 = 0;
        for &v in &expected_raw {
            acc = (acc << 7) | v;
            nbits += 7;
            while nbits >= 8 {
                let shift = nbits - 8;
                bits.push((acc >> shift) as u8);
                acc &= (1 << shift) - 1;
                nbits -= 8;
            }
        }
        if nbits > 0 {
            bits.push((acc << (8 - nbits)) as u8);
        }

        // Decode raw.
        let mut reader = Sv7BitReader::new(&bits);
        let mut raw = [0_i32; SAMPLES_PER_BAND];
        decode_linear_pcm_band(&mut reader, 8, &mut raw).expect("decode");
        for i in 0..SAMPLES_PER_BAND {
            assert_eq!(raw[i] as u32, expected_raw[i]);
        }

        // Centre.
        centre_pcm_band(8, &mut raw).expect("centre");
        let d = QUANTIZER_OFFSET_D[9] as i32;
        for i in 0..SAMPLES_PER_BAND {
            let want = expected_raw[i] as i32 - d;
            assert_eq!(raw[i], want, "sample {i} centred");
            assert!((-d..=d).contains(&raw[i]));
        }

        // Dequant.
        let mut out = [0.0_f64; SAMPLES_PER_BAND];
        dequantise_band(8, &raw, &mut out).expect("dequant");
        let c = DEQUANT_COEFFICIENT_C[9];
        for i in 0..SAMPLES_PER_BAND {
            let expected = raw[i] as f64 * c / DEQUANT_DIVISOR;
            assert!((out[i] - expected).abs() < 1e-12);
        }
    }

    // ─── relative SCF gain ladder (§2.6) ───────────────────

    #[test]
    fn scf_index_count_is_256() {
        // scf-step-ratio.meta: "... 256 indices".
        assert_eq!(SCF_INDEX_COUNT, 256);
    }

    #[test]
    fn scf_relative_gain_identity_is_one() {
        for idx in [0_u8, 1, 50, 128, 200, 255] {
            assert_eq!(scf_relative_gain(idx, idx), 1.0, "anchor {idx}");
        }
    }

    #[test]
    fn scf_relative_gain_one_step_up_is_step_ratio() {
        // Index +1 == one "downward" step == multiply by SCF_STEP_RATIO.
        let g = scf_relative_gain(10, 11);
        assert!((g - SCF_STEP_RATIO).abs() < 1e-15, "got {g}");
        assert!(g < 1.0, "higher index is quieter; got {g}");
    }

    #[test]
    fn scf_relative_gain_one_step_down_is_reciprocal() {
        // Index −1 == one upward step == multiply by 1/SCF_STEP_RATIO.
        let g = scf_relative_gain(11, 10);
        let want = 1.0 / SCF_STEP_RATIO;
        assert!((g - want).abs() < 1e-12, "got {g}, want {want}");
        assert!(g > 1.0, "lower index is louder; got {g}");
    }

    #[test]
    fn scf_relative_gain_is_inverse_symmetric() {
        // gain(a,b) * gain(b,a) == 1 for any pair.
        for (a, b) in [(0_u8, 255_u8), (37, 200), (128, 64), (1, 2)] {
            let round_trip = scf_relative_gain(a, b) * scf_relative_gain(b, a);
            assert!(
                (round_trip - 1.0).abs() < 1e-9,
                "pair ({a},{b}) round trip {round_trip}"
            );
        }
    }

    #[test]
    fn scf_relative_gain_composes_additively_in_exponent() {
        // gain(a,c) == gain(a,b) * gain(b,c).
        let (a, b, c) = (20_u8, 40, 90);
        let direct = scf_relative_gain(a, c);
        let composed = scf_relative_gain(a, b) * scf_relative_gain(b, c);
        assert!(
            (direct - composed).abs() < 1e-9,
            "direct {direct} vs composed {composed}"
        );
    }

    #[test]
    fn scf_relative_gain_n_steps_equals_ratio_pow_n() {
        // Moving up by k indices multiplies by SCF_STEP_RATIO^k.
        for k in [2_u8, 5, 13, 40] {
            let g = scf_relative_gain(0, k);
            let want = SCF_STEP_RATIO.powi(i32::from(k));
            assert!((g - want).abs() < 1e-12, "k={k}: {g} vs {want}");
        }
    }

    #[test]
    fn scf_gain_relative_to_anchor_anchor_is_unity() {
        let mut tbl = [0.0_f64; SCF_INDEX_COUNT];
        scf_gain_relative_to_anchor(100, &mut tbl);
        assert_eq!(tbl[100], 1.0);
        // Each entry equals scf_relative_gain(anchor, i).
        for (i, &g) in tbl.iter().enumerate() {
            assert!((g - scf_relative_gain(100, i as u8)).abs() < 1e-15);
        }
    }

    #[test]
    fn scf_gain_relative_to_anchor_is_monotonically_decreasing() {
        // Higher index == quieter, so the ladder strictly decreases.
        let mut tbl = [0.0_f64; SCF_INDEX_COUNT];
        scf_gain_relative_to_anchor(0, &mut tbl);
        for i in 1..SCF_INDEX_COUNT {
            assert!(
                tbl[i] < tbl[i - 1],
                "index {i}: {} not < {}",
                tbl[i],
                tbl[i - 1]
            );
        }
        // Index 0 anchor is exactly unity.
        assert_eq!(tbl[0], 1.0);
    }

    #[test]
    fn apply_scf_relative_scales_every_sample() {
        let mut band = [2.0_f64; SAMPLES_PER_BAND];
        apply_scf_relative(5, 8, &mut band);
        let gain = scf_relative_gain(5, 8);
        for &s in band.iter() {
            assert!((s - 2.0 * gain).abs() < 1e-12);
        }
    }

    #[test]
    fn apply_scf_relative_identity_leaves_band_unchanged() {
        let mut band = [-3.5_f64; SAMPLES_PER_BAND];
        apply_scf_relative(77, 77, &mut band);
        for &s in band.iter() {
            assert_eq!(s, -3.5);
        }
    }

    #[test]
    fn apply_scf_relative_round_trips_through_inverse() {
        // Applying (a→b) then (b→a) returns the original band.
        let original = 4.25_f64;
        let mut band = [original; SAMPLES_PER_BAND];
        apply_scf_relative(30, 95, &mut band);
        apply_scf_relative(95, 30, &mut band);
        for &s in band.iter() {
            assert!((s - original).abs() < 1e-9, "got {s}");
        }
    }

    // ─── Per-granule SCF reconstruction ────────────────────

    #[test]
    fn granule_geometry_tiles_the_band() {
        assert_eq!(GRANULES_PER_BAND, 3);
        assert_eq!(SAMPLES_PER_GRANULE, 12);
        assert_eq!(GRANULES_PER_BAND * SAMPLES_PER_GRANULE, SAMPLES_PER_BAND);
    }

    #[test]
    fn apply_granule_scf_scales_each_twelve_sample_slice_independently() {
        // Three distinct granule SCFs over a flat band: each contiguous
        // 12-sample slice must carry its own relative gain.
        let mut band = [1.0_f64; SAMPLES_PER_BAND];
        let anchor = 50;
        let scf = [50_u8, 52, 60];
        apply_granule_scf_relative(anchor, scf, &mut band);
        for (g, &s) in scf.iter().enumerate() {
            let want = scf_relative_gain(anchor, s);
            for k in 0..SAMPLES_PER_GRANULE {
                let i = g * SAMPLES_PER_GRANULE + k;
                assert!((band[i] - want).abs() < 1e-12, "granule {g} sample {k}");
            }
        }
    }

    #[test]
    fn apply_granule_scf_anchor_equal_indices_is_identity() {
        let mut band = [-2.5_f64; SAMPLES_PER_BAND];
        apply_granule_scf_relative(33, [33, 33, 33], &mut band);
        for &s in band.iter() {
            assert_eq!(s, -2.5);
        }
    }

    #[test]
    fn apply_granule_scf_uniform_indices_matches_apply_scf_relative() {
        // When all three granule SCFs are equal, the per-granule path
        // must agree with the whole-band apply_scf_relative.
        let mut granule_band = [3.0_f64; SAMPLES_PER_BAND];
        let mut whole_band = [3.0_f64; SAMPLES_PER_BAND];
        apply_granule_scf_relative(10, [40, 40, 40], &mut granule_band);
        apply_scf_relative(10, 40, &mut whole_band);
        for i in 0..SAMPLES_PER_BAND {
            assert!((granule_band[i] - whole_band[i]).abs() < 1e-12, "{i}");
        }
    }

    #[test]
    fn reconstruct_band_dequantises_then_applies_granule_scf() {
        // band_type 3: C = DEQUANT_COEFFICIENT_C[4]. Centred level 1 in
        // every slot; check granule 1 carries the (anchor→scf[1]) gain
        // on top of the dequant product.
        let band_type = 3_i8;
        let centred = [1_i32; SAMPLES_PER_BAND];
        let anchor = 20;
        let scf = [20_u8, 25, 30];
        let mut out = [0.0_f64; SAMPLES_PER_BAND];
        reconstruct_band_with_granule_scf(band_type, &centred, anchor, scf, &mut out).unwrap();

        let mut dq = [0.0_f64; SAMPLES_PER_BAND];
        dequantise_band(band_type, &centred, &mut dq).unwrap();
        for (g, &s) in scf.iter().enumerate() {
            let gain = scf_relative_gain(anchor, s);
            for k in 0..SAMPLES_PER_GRANULE {
                let i = g * SAMPLES_PER_GRANULE + k;
                assert!(
                    (out[i] - dq[i] * gain).abs() < 1e-12,
                    "granule {g} sample {k}"
                );
            }
        }
    }

    #[test]
    fn reconstruct_band_rejects_out_of_quantiser_range_band_type() {
        let centred = [0_i32; SAMPLES_PER_BAND];
        let mut out = [0.0_f64; SAMPLES_PER_BAND];
        for bt in [-2_i8, -1, 18, i8::MAX, i8::MIN] {
            assert!(matches!(
                reconstruct_band_with_granule_scf(bt, &centred, 0, [0, 0, 0], &mut out),
                Err(Error::UnsupportedBandType(_)),
            ));
        }
    }

    #[test]
    fn reconstruct_cns_band_dequantises_then_applies_granule_scf() {
        let cns_levels = [255_i32; SAMPLES_PER_BAND];
        let anchor = 64;
        let scf = [64_u8, 66, 70];
        let mut out = [0.0_f64; SAMPLES_PER_BAND];
        reconstruct_cns_band_with_granule_scf(&cns_levels, anchor, scf, &mut out);

        let mut dq = [0.0_f64; SAMPLES_PER_BAND];
        dequantise_cns_band(&cns_levels, &mut dq);
        for (g, &s) in scf.iter().enumerate() {
            let gain = scf_relative_gain(anchor, s);
            for k in 0..SAMPLES_PER_GRANULE {
                let i = g * SAMPLES_PER_GRANULE + k;
                assert!(
                    (out[i] - dq[i] * gain).abs() < 1e-12,
                    "granule {g} sample {k}"
                );
            }
        }
    }

    // ─── End-to-end SV7 band reconstruction from levels ────

    #[test]
    fn reconstruct_from_levels_empty_band_is_silence() {
        // band_type 0: levels all zero; reconstruct to all-zero samples
        // regardless of SCF.
        let levels = [0_i32; SAMPLES_PER_BAND];
        let mut out = [9.9_f64; SAMPLES_PER_BAND];
        reconstruct_sv7_band_from_levels(0, &levels, 30, [30, 35, 40], &mut out).unwrap();
        for &s in out.iter() {
            assert_eq!(s, 0.0);
        }
    }

    #[test]
    fn reconstruct_from_levels_huffman_path_matches_centred_reconstruct() {
        // band_type 4 (Huffman, already-centred levels): the level
        // buffer is fed straight through dequant + per-granule SCF.
        let band_type = 4_i8;
        let mut levels = [0_i32; SAMPLES_PER_BAND];
        for (i, l) in levels.iter_mut().enumerate() {
            *l = (i as i32 % 7) - 3; // small centred values
        }
        let anchor = 40;
        let scf = [40_u8, 44, 48];
        let mut got = [0.0_f64; SAMPLES_PER_BAND];
        reconstruct_sv7_band_from_levels(band_type, &levels, anchor, scf, &mut got).unwrap();

        let mut want = [0.0_f64; SAMPLES_PER_BAND];
        reconstruct_band_with_granule_scf(band_type, &levels, anchor, scf, &mut want).unwrap();
        for i in 0..SAMPLES_PER_BAND {
            assert!((got[i] - want[i]).abs() < 1e-12, "{i}");
        }
    }

    #[test]
    fn reconstruct_from_levels_pcm_escape_centres_before_dequant() {
        // band_type 8: raw unsigned levels 0..=2D, D = 63. A raw level
        // of D must reconstruct to 0 (centred = 0).
        let band_type = 8_i8;
        let d = QUANTIZER_OFFSET_D[9] as i32;
        let levels = [d; SAMPLES_PER_BAND]; // every centred level == 0
        let mut out = [1.0_f64; SAMPLES_PER_BAND];
        reconstruct_sv7_band_from_levels(band_type, &levels, 50, [50, 50, 50], &mut out).unwrap();
        for &s in out.iter() {
            assert_eq!(s, 0.0);
        }

        // A raw level of 2D → centred +D → positive sample scaled by SCF.
        let levels_hi = [2 * d; SAMPLES_PER_BAND];
        let mut out_hi = [0.0_f64; SAMPLES_PER_BAND];
        reconstruct_sv7_band_from_levels(band_type, &levels_hi, 50, [50, 50, 50], &mut out_hi)
            .unwrap();
        // Expected: raw 2D centred to +D, dequantised, gain 1.0
        // (anchor == scf).
        let mut centred = [2 * d; SAMPLES_PER_BAND];
        centre_pcm_band(band_type, &mut centred).unwrap();
        let mut want = [0.0_f64; SAMPLES_PER_BAND];
        dequantise_band(band_type, &centred, &mut want).unwrap();
        for i in 0..SAMPLES_PER_BAND {
            assert!((out_hi[i] - want[i]).abs() < 1e-12, "{i}");
        }
    }

    #[test]
    fn reconstruct_from_levels_does_not_mutate_caller_levels() {
        // PCM-escape centring must operate on a scratch copy.
        let band_type = 9_i8;
        let d = QUANTIZER_OFFSET_D[10] as i32;
        let levels = [d; SAMPLES_PER_BAND];
        let mut out = [0.0_f64; SAMPLES_PER_BAND];
        reconstruct_sv7_band_from_levels(band_type, &levels, 0, [0, 0, 0], &mut out).unwrap();
        // Caller's buffer untouched.
        for &l in levels.iter() {
            assert_eq!(l, d);
        }
    }

    #[test]
    fn reconstruct_from_levels_cns_uses_cns_coefficient() {
        let levels = [200_i32; SAMPLES_PER_BAND];
        let anchor = 64;
        let scf = [64_u8, 64, 64];
        let mut got = [0.0_f64; SAMPLES_PER_BAND];
        reconstruct_sv7_band_from_levels(-1, &levels, anchor, scf, &mut got).unwrap();

        let mut want = [0.0_f64; SAMPLES_PER_BAND];
        reconstruct_cns_band_with_granule_scf(&levels, anchor, scf, &mut want);
        for i in 0..SAMPLES_PER_BAND {
            assert!((got[i] - want[i]).abs() < 1e-12, "{i}");
        }
    }

    #[test]
    fn reconstruct_from_levels_rejects_out_of_range_band_type() {
        let levels = [0_i32; SAMPLES_PER_BAND];
        let mut out = [0.0_f64; SAMPLES_PER_BAND];
        for bt in [-2_i8, 18, i8::MAX, i8::MIN] {
            assert!(matches!(
                reconstruct_sv7_band_from_levels(bt, &levels, 0, [0, 0, 0], &mut out),
                Err(Error::UnsupportedBandType(_)),
            ));
        }
    }

    // ─── _sanity_band0_c_is_divisor doesn't drift ──────────

    #[test]
    fn band_type_0_dequant_coefficient_equals_divisor() {
        // The relation C = 65536 / (2D+1) with D=0 gives C = 65536,
        // which is exactly DEQUANT_DIVISOR. This invariant is what
        // lets dequantise_sample(0, x) == x.
        assert!((DEQUANT_COEFFICIENT_C[1] - DEQUANT_DIVISOR).abs() < 1e-9);
        // And call the internal sanity helper so dead-code analysis
        // can't drop it.
        _sanity_band0_c_is_divisor();
    }

    // ─── signed (SV8) SCF gain ─────────────────────────────

    #[test]
    fn scf_relative_gain_signed_matches_unsigned_in_overlapping_range() {
        // For indices that fit the u8 ladder, the signed variant must
        // produce bit-identical gains to the u8 entry point.
        for from in [0_i32, 1, 17, 121] {
            for to in [0_i32, 1, 25, 121] {
                let s = scf_relative_gain_signed(from, to);
                let u = scf_relative_gain(from as u8, to as u8);
                assert!((s - u).abs() < 1e-15, "from {from} to {to}: {s} vs {u}");
            }
        }
    }

    #[test]
    fn scf_relative_gain_signed_handles_negative_indices() {
        // SV8 fold can yield indices down to -6. Equal indices ⇒ 1.0;
        // a one-step increase ⇒ SCF_STEP_RATIO; symmetry under swap.
        assert_eq!(scf_relative_gain_signed(-6, -6), 1.0);
        assert!((scf_relative_gain_signed(-6, -5) - SCF_STEP_RATIO).abs() < 1e-15);
        // Reciprocal symmetry: gain(a,b) * gain(b,a) == 1.
        let g = scf_relative_gain_signed(-6, 121);
        let inv = scf_relative_gain_signed(121, -6);
        assert!((g * inv - 1.0).abs() < 1e-12);
        // Higher index = quieter (downward stored direction).
        assert!(scf_relative_gain_signed(0, 10) < 1.0);
        assert!(scf_relative_gain_signed(0, -10) > 1.0);
    }

    #[test]
    fn apply_granule_scf_relative_signed_scales_each_granule() {
        let mut band = [1.0_f64; SAMPLES_PER_BAND];
        // Anchor 0; granule gains: 1.0, ratio, ratio^2.
        apply_granule_scf_relative_signed(0, [0, 1, 2], &mut band);
        for s in &band[0..SAMPLES_PER_GRANULE] {
            assert!((s - 1.0).abs() < 1e-15);
        }
        for s in &band[SAMPLES_PER_GRANULE..2 * SAMPLES_PER_GRANULE] {
            assert!((s - SCF_STEP_RATIO).abs() < 1e-15);
        }
        for s in &band[2 * SAMPLES_PER_GRANULE..] {
            assert!((s - SCF_STEP_RATIO * SCF_STEP_RATIO).abs() < 1e-15);
        }
    }

    #[test]
    fn apply_granule_scf_relative_signed_with_negative_anchor() {
        // A negative anchor (legal for SV8) still produces 1.0 when a
        // granule SCF equals the anchor, and the relative loudness
        // ordering is preserved.
        let mut band = [2.0_f64; SAMPLES_PER_BAND];
        apply_granule_scf_relative_signed(-6, [-6, -5, -4], &mut band);
        // Granule 0 == anchor ⇒ unchanged.
        assert!((band[0] - 2.0).abs() < 1e-15);
        // Each later granule is one more downward step ⇒ quieter.
        assert!(band[SAMPLES_PER_GRANULE].abs() < band[0].abs());
        assert!(band[2 * SAMPLES_PER_GRANULE].abs() < band[SAMPLES_PER_GRANULE].abs());
    }
}
