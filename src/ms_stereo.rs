//! SV7/SV8 §2.6 mid/side (M/S) stereo-undo step.
//!
//! §2.6 lists "undo M/S where `msflag` set" as a reconstruction step
//! between the per-band dequant+SCF multiply
//! ([`crate::frame_reconstruct`]) and the synthesis filterbank. When a
//! subband's per-band `msflag` is set, that subband's two channels were
//! coded as a **mid** channel (`L`-row) and a **side** channel
//! (`R`-row) rather than as left/right directly; the decoder must
//! invert that transform before the filterbank.
//!
//! # The arithmetic is a documented GAP — threaded as a caller knob
//!
//! The structural spec (§2.6) names the step and the per-band `msflag`
//! that gates it (decoded by [`crate::sv7_band_header`]), but the
//! **exact channel arithmetic** — whether `L = M + S` / `R = M − S`,
//! and any `0.5` / `√2` normalisation — is **not specified anywhere
//! under `docs/audio/musepack/`**. Rather than guess one of the
//! several plausible conventions (which would be silently wrong if the
//! docs later pin a different one), this module follows the crate's
//! established GAP-knob pattern (cf.
//! [`crate::sv8_band_header::decode_band_resolutions`]'s
//! `ctx_for_prev_res` closure): the per-sample mid/side → left/right
//! transform is a **caller-supplied closure**, isolated here so a
//! single edit wires the real arithmetic once a docs trace lands.
//!
//! This module therefore wires the *structure* of the §2.6 M/S-undo
//! step — the per-subband gating on `msflag`, the row pairing across
//! the two channels, the L/R-row pass-through — without committing to
//! the GAP arithmetic.
//!
//! Source-of-record (facts only):
//!
//! - `docs/audio/musepack/musepack-sv7-sv8-spec.md` §2.3 / §2.6 — the
//!   per-band `msflag` and the "undo M/S where `msflag` set" step.
//! - `docs/audio/musepack/musepack-sv7-sv8-spec.md` §1 — the
//!   32-subband frame geometry the two channels share.

use crate::frame_reconstruct::SubbandMatrix;
use crate::sv7_band_decode::SAMPLES_PER_BAND;
use crate::sv7_band_header::SV7_SUBBAND_COUNT;
use crate::{Error, Result};

/// A stereo pair of per-channel subband matrices for one frame:
/// `[0]` is the left/mid channel, `[1]` is the right/side channel
/// (which role depends on each subband's `msflag`). The two channels
/// share the [`SV7_SUBBAND_COUNT`] × [`SAMPLES_PER_BAND`] geometry.
pub type StereoSubbandMatrix = [SubbandMatrix; 2];

/// Apply the §2.6 M/S-undo step across a stereo frame, in place.
///
/// `ms_flags[b]` gates subband `b`: when `true`, the two channels'
/// row `b` is the (mid, side) pair and is transformed sample-by-sample
/// via `undo` into the (left, right) pair; when `false`, the rows are
/// already left/right and pass through unchanged.
///
/// `undo(m, s) -> (l, r)` is the **GAP** per-sample mid/side →
/// left/right transform (see the module docs). It is invoked once per
/// sample of each M/S subband, with `m` taken from channel 0's row and
/// `s` from channel 1's row; the returned `l` is written back to
/// channel 0 and `r` to channel 1.
///
/// # Errors
///
/// [`Error::MaxBandOutOfRange`] if `ms_flags` is longer than
/// [`SV7_SUBBAND_COUNT`] (a schedule that cannot index the frame's
/// subbands). A shorter slice is allowed: subbands past its end are
/// treated as L/R (no undo) — the §2.3 loop only emits an `msflag` for
/// bands it actually codes.
pub fn undo_ms_stereo<F>(stereo: &mut StereoSubbandMatrix, ms_flags: &[bool], undo: F) -> Result<()>
where
    F: Fn(f64, f64) -> (f64, f64),
{
    if ms_flags.len() > SV7_SUBBAND_COUNT {
        return Err(Error::MaxBandOutOfRange(ms_flags.len() as u8));
    }
    for (b, &is_ms) in ms_flags.iter().enumerate() {
        if !is_ms {
            continue;
        }
        // Split-borrow the two channels so both rows are mutable at once.
        let (left_ch, right_ch) = stereo.split_at_mut(1);
        let mid_row = &mut left_ch[0][b];
        let side_row = &mut right_ch[0][b];
        for i in 0..SAMPLES_PER_BAND {
            let (l, r) = undo(mid_row[i], side_row[i]);
            mid_row[i] = l;
            side_row[i] = r;
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame_reconstruct::zero_subband_matrix;

    /// A representative `L = M + S` / `R = M − S` transform used **only
    /// in tests** to exercise the structural plumbing. This is *not* a
    /// claim about the real (GAP) Musepack arithmetic — it is an
    /// arbitrary invertible stand-in for verifying gating / pairing /
    /// pass-through.
    fn test_undo(m: f64, s: f64) -> (f64, f64) {
        (m + s, m - s)
    }

    fn stereo_with(row: usize, mid: f64, side: f64) -> StereoSubbandMatrix {
        let mut st = [zero_subband_matrix(), zero_subband_matrix()];
        st[0][row].fill(mid);
        st[1][row].fill(side);
        st
    }

    #[test]
    fn ms_subband_is_transformed() {
        let mut st = stereo_with(0, 3.0, 1.0);
        undo_ms_stereo(&mut st, &[true], test_undo).unwrap();
        // L = M + S = 4, R = M - S = 2.
        assert!(st[0][0].iter().all(|&s| s == 4.0));
        assert!(st[1][0].iter().all(|&s| s == 2.0));
    }

    #[test]
    fn lr_subband_passes_through() {
        let mut st = stereo_with(0, 3.0, 1.0);
        undo_ms_stereo(&mut st, &[false], test_undo).unwrap();
        assert!(st[0][0].iter().all(|&s| s == 3.0));
        assert!(st[1][0].iter().all(|&s| s == 1.0));
    }

    #[test]
    fn only_flagged_rows_change() {
        let mut st = [zero_subband_matrix(), zero_subband_matrix()];
        let (ch0, ch1) = st.split_at_mut(1);
        for (mid_row, side_row) in ch0[0].iter_mut().zip(ch1[0].iter_mut()).take(3) {
            mid_row.fill(10.0);
            side_row.fill(4.0);
        }
        // Only subband 1 is M/S.
        undo_ms_stereo(&mut st, &[false, true, false], test_undo).unwrap();
        // Row 0 + row 2 unchanged.
        for &row in &[0usize, 2] {
            assert_eq!(st[0][row][0], 10.0);
            assert_eq!(st[1][row][0], 4.0);
        }
        // Row 1 transformed: L = 14, R = 6.
        assert_eq!(st[0][1][0], 14.0);
        assert_eq!(st[1][1][0], 6.0);
    }

    #[test]
    fn empty_schedule_is_noop() {
        let mut st = stereo_with(5, 7.0, 2.0);
        let before = st;
        undo_ms_stereo(&mut st, &[], test_undo).unwrap();
        assert_eq!(st, before);
    }

    #[test]
    fn short_schedule_leaves_uncovered_subbands_as_lr() {
        // Schedule only covers subband 0; subband 4 (M/S-looking data)
        // is past its end and must pass through untouched.
        let mut st = stereo_with(4, 9.0, 3.0);
        undo_ms_stereo(&mut st, &[true], test_undo).unwrap();
        assert_eq!(st[0][4][0], 9.0);
        assert_eq!(st[1][4][0], 3.0);
    }

    #[test]
    fn full_width_schedule_is_accepted() {
        let mut st = [zero_subband_matrix(), zero_subband_matrix()];
        let flags = [false; SV7_SUBBAND_COUNT];
        assert!(undo_ms_stereo(&mut st, &flags, test_undo).is_ok());
    }

    #[test]
    fn rejects_overlong_schedule() {
        let mut st = [zero_subband_matrix(), zero_subband_matrix()];
        let flags = [false; SV7_SUBBAND_COUNT + 1];
        assert_eq!(
            undo_ms_stereo(&mut st, &flags, test_undo),
            Err(Error::MaxBandOutOfRange((SV7_SUBBAND_COUNT + 1) as u8))
        );
    }

    #[test]
    fn closure_sees_per_sample_pairs() {
        // A distinct mid/side per sample confirms each is paired by index.
        let mut st = [zero_subband_matrix(), zero_subband_matrix()];
        let (ch0, ch1) = st.split_at_mut(1);
        for (i, (m, s)) in ch0[0][0].iter_mut().zip(ch1[0][0].iter_mut()).enumerate() {
            *m = i as f64;
            *s = (i as f64) * 2.0;
        }
        undo_ms_stereo(&mut st, &[true], test_undo).unwrap();
        for (i, (&l, &r)) in st[0][0].iter().zip(st[1][0].iter()).enumerate() {
            let (m, s) = (i as f64, (i as f64) * 2.0);
            assert_eq!(l, m + s);
            assert_eq!(r, m - s);
        }
    }
}
