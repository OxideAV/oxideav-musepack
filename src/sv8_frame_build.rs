//! SV8 **frame builder** — analysed subband data → the structured
//! [`Sv8StereoFrameDecode`] the wire encoder
//! ([`crate::sv8_stereo_frame_encode::encode_sv8_stereo_frame`])
//! consumes.
//!
//! This is the encoder's *decision* layer, sitting between the
//! analysis filterbank ([`crate::analysis`]) and the frame-body wire
//! encoder: per band it elects L/R vs M/S coding, picks each channel's
//! `Res` (band type) with the [`crate::sv8_quantize`] bit-allocation
//! policy, quantises the surviving channels, and derives the SCFI
//! selectors from the granule-SCF equalities.
//!
//! # Forced facts vs. policy
//!
//! Two wire facts constrain the builder; everything else is encoder
//! policy:
//!
//! - **SCFI sharing is forced by the DSCF alphabet.** The §6.3
//!   later-granule delta table (`dscf-1`) has **no codeword for the
//!   identity delta** (the value-31 slot is the escape): equal
//!   consecutive granule SCFs *must* be expressed through the SCFI
//!   share bits, never as a zero delta. The builder therefore sets
//!   SCFI exactly from the equalities
//!   (`scfi = 2·[SCF1 == SCF0] + [SCF2 == SCF1]`), which
//!   simultaneously minimises the DSCF bit spend.
//! - **The SV8 `Res` ring is `−1..=15`.** The §6.2 wrap ("values above
//!   15 wrap by −17") makes band types 16/17 unreachable on the SV8
//!   wire (unlike SV7's 0..=17 ladder), so the allocation policy is
//!   capped at `Res = 15` ([`SV8_MAX_RES`]).
//!
//! **Policy** (any choice yields a valid stream): the flat s16-domain
//! noise-step target driving [`crate::sv8_quantize::band_type_for_peak`],
//! and the per-band M/S election — mid = `(L + R) / 2`, side =
//! `(L − R) / 2` (the exact forward of the corpus-pinned undo
//! `L = M + S`, `R = M − S`), elected when the estimated bit cost of
//! coding (M, S) beats (L, R). A silent side channel (correlated
//! stereo, mono-duplicated input) makes M/S strictly cheaper, which is
//! also how a mono stream gets its cheap two-channel body: feed the
//! same matrix as both channels and every coded band elects M/S with
//! an empty side.
//!
//! Source-of-record: the wire constraints above are decode-side facts
//! already pinned in [`crate::sv8_dscf_loop`] / [`crate::sv8_band_header`]
//! (`spec/musepack-headers-and-coding.md` §6.2/§6.3); the M/S undo is
//! the r390/r419 corpus-pinned [`crate::ms_stereo::ms_to_lr`]. No new
//! format facts.

use crate::frame_reconstruct::SubbandMatrix;
use crate::requant::{band_type_index, QUANTIZER_OFFSET_D};
use crate::scf::SCF_GRANULES_PER_BAND;
use crate::sv7_band_decode::SAMPLES_PER_BAND;
use crate::sv7_band_header::SV7_SUBBAND_COUNT;
use crate::sv8_quantize::{band_type_for_peak, quantize_band};
use crate::sv8_stereo_frame::Sv8StereoFrameDecode;
use crate::{Error, Result};

/// Highest band type reachable on the SV8 wire: the §6.2 fold confines
/// `Res` to the signed ring `−1..=15`.
pub const SV8_MAX_RES: i8 = 15;

/// Frame-builder settings (encoder policy knobs).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sv8FrameBuildSettings {
    /// The flat s16-domain quantisation-step target driving the
    /// per-band `Res` choice
    /// ([`crate::sv8_quantize::band_type_for_peak`]). Smaller =
    /// finer quantisers = higher rate and higher SNR.
    pub step_target: f64,
    /// Whether the stream codes with the `SH` stream-wide M/S flag
    /// set. When `false` no band elects M/S (and the frame encoder
    /// writes no bitmap).
    pub stream_ms: bool,
}

impl Default for Sv8FrameBuildSettings {
    /// Default: `step_target = 2.0` s16 LSBs (a noise floor roughly
    /// 84 dB below full scale per band before SCF granularity),
    /// stream M/S on.
    fn default() -> Self {
        Self {
            step_target: 2.0,
            stream_ms: true,
        }
    }
}

/// Rough per-band bit-cost estimate for electing L/R vs M/S coding:
/// `36 × log2(2·D[bt] + 1)` sample bits plus a flat per-coded-channel
/// SCF/SCFI overhead. Policy only — used for a comparison, never for
/// budgets.
fn band_cost_estimate(band_type: i8) -> f64 {
    if band_type == 0 {
        return 0.0;
    }
    let idx = band_type_index(band_type).expect("builder band types are 0..=15");
    let d = f64::from(QUANTIZER_OFFSET_D[idx]);
    let bits_per_sample = (2.0 * d + 1.0).log2();
    36.0 * bits_per_sample + 24.0
}

/// The per-band peak magnitude.
fn band_peak(band: &[f64; SAMPLES_PER_BAND]) -> f64 {
    band.iter().fold(0.0_f64, |a, &x| a.max(x.abs()))
}

/// Derive the §6.3 SCFI selector from the granule-SCF equalities. The
/// sharing is **forced** (see the module docs): `dscf-1` cannot code
/// an identity delta, so equal neighbours must use the share bits.
fn scfi_for(scf: &[i32; SCF_GRANULES_PER_BAND]) -> u8 {
    let share1 = u8::from(scf[1] == scf[0]);
    let share2 = u8::from(scf[2] == scf[1]);
    (share1 << 1) | share2
}

/// Build one SV8 two-channel frame from a pair of analysed subband
/// matrices (the [`crate::analysis`] output for one 1152-sample frame
/// of each channel).
///
/// For a **mono** stream pass the same matrix as both channels: every
/// coded band then elects M/S with a silent side (see the module
/// docs), producing the cheap mono body shape.
///
/// `max_band` is the `SH` header's highest-coded-subband field
/// (bands `0..=max_band` participate; anything above is left uncoded).
///
/// The output is exactly the structure
/// [`crate::sv8_stereo_frame::decode_sv8_stereo_frame`] would decode
/// from the resulting bits, so build → encode → decode round trips
/// structurally.
///
/// # Errors
///
/// [`Error::MaxBandOutOfRange`] if `max_band` is outside `1..=31`
/// (the `SH` §2 sanity range).
pub fn build_sv8_stereo_frame(
    left: &SubbandMatrix,
    right: &SubbandMatrix,
    max_band: u8,
    settings: &Sv8FrameBuildSettings,
) -> Result<Sv8StereoFrameDecode> {
    if !(1..=31).contains(&max_band) {
        return Err(Error::MaxBandOutOfRange(max_band));
    }
    let nb_considered = (max_band as usize + 1).min(SV7_SUBBAND_COUNT);

    let mut res: Vec<[i8; 2]> = Vec::with_capacity(nb_considered);
    let mut ms_flags: Vec<bool> = Vec::with_capacity(nb_considered);
    let mut scfi: Vec<[u8; 2]> = Vec::with_capacity(nb_considered);
    let mut granule_scf: Vec<[[i32; SCF_GRANULES_PER_BAND]; 2]> = Vec::with_capacity(nb_considered);
    let mut levels: Vec<[[i32; SAMPLES_PER_BAND]; 2]> = Vec::with_capacity(nb_considered);

    for b in 0..nb_considered {
        let l = &left[b];
        let r = &right[b];

        // L/R posture.
        let bt_l = band_type_for_peak(band_peak(l), settings.step_target).min(SV8_MAX_RES);
        let bt_r = band_type_for_peak(band_peak(r), settings.step_target).min(SV8_MAX_RES);

        // M/S posture (only when the stream allows it).
        let mut use_ms = false;
        let mut mid = [0.0_f64; SAMPLES_PER_BAND];
        let mut side = [0.0_f64; SAMPLES_PER_BAND];
        let mut bt_m = 0_i8;
        let mut bt_s = 0_i8;
        if settings.stream_ms {
            for k in 0..SAMPLES_PER_BAND {
                mid[k] = (l[k] + r[k]) / 2.0;
                side[k] = (l[k] - r[k]) / 2.0;
            }
            bt_m = band_type_for_peak(band_peak(&mid), settings.step_target).min(SV8_MAX_RES);
            bt_s = band_type_for_peak(band_peak(&side), settings.step_target).min(SV8_MAX_RES);
            let cost_lr = band_cost_estimate(bt_l) + band_cost_estimate(bt_r);
            let cost_ms = band_cost_estimate(bt_m) + band_cost_estimate(bt_s);
            use_ms = cost_ms < cost_lr;
        }

        let (bt0, bt1, ch0, ch1): (i8, i8, &[f64; SAMPLES_PER_BAND], &[f64; SAMPLES_PER_BAND]) =
            if use_ms {
                (bt_m, bt_s, &mid, &side)
            } else {
                (bt_l, bt_r, l, r)
            };

        let mut band_scfi = [0_u8; 2];
        let mut band_scf = [[0_i32; SCF_GRANULES_PER_BAND]; 2];
        let mut band_levels = [[0_i32; SAMPLES_PER_BAND]; 2];
        for (ch, (bt, data)) in [(bt0, ch0), (bt1, ch1)].into_iter().enumerate() {
            if bt == 0 {
                continue;
            }
            let q = quantize_band(bt, data)?;
            band_scfi[ch] = scfi_for(&q.scf);
            band_scf[ch] = q.scf;
            band_levels[ch] = q.levels;
        }

        res.push([bt0, bt1]);
        ms_flags.push(use_ms && (bt0 != 0 || bt1 != 0));
        scfi.push(band_scfi);
        granule_scf.push(band_scf);
        levels.push(band_levels);
    }

    // Max_used_Band: everything above the highest coded band is
    // dropped from the frame entirely.
    let nbands = res
        .iter()
        .rposition(|&[a, b]| a != 0 || b != 0)
        .map_or(0, |i| i + 1);
    res.truncate(nbands);
    ms_flags.truncate(nbands);
    scfi.truncate(nbands);
    granule_scf.truncate(nbands);
    levels.truncate(nbands);

    Ok(Sv8StereoFrameDecode {
        nbands: nbands as u8,
        res,
        ms_flags,
        scfi,
        granule_scf,
        levels,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cns::CnsPrng;
    use crate::frame_reconstruct::zero_subband_matrix;
    use crate::huffman::Sv7BitReader;
    use crate::ms_stereo::undo_ms_stereo_pinned;
    use crate::sv7_bitwriter::Sv7BitWriter;
    use crate::sv8_stereo_frame::{
        decode_sv8_stereo_frame, reconstruct_sv8_stereo_frame, Sv8FrameState,
    };
    use crate::sv8_stereo_frame_encode::encode_sv8_stereo_frame;

    /// Deterministic pseudo-random f64 in [-1, 1) (xorshift64*).
    struct Rng(u64);
    impl Rng {
        fn next(&mut self) -> f64 {
            let mut x = self.0;
            x ^= x >> 12;
            x ^= x << 25;
            x ^= x >> 27;
            self.0 = x;
            let v = x.wrapping_mul(0x2545_F491_4F6C_DD1D);
            ((v >> 11) as f64) / ((1u64 << 53) as f64) * 2.0 - 1.0
        }
    }

    fn random_matrix(rng: &mut Rng, bands: usize, amp: f64) -> SubbandMatrix {
        let mut m = zero_subband_matrix();
        for band in m.iter_mut().take(bands) {
            for v in band.iter_mut() {
                *v = rng.next() * amp;
            }
        }
        m
    }

    /// Build → wire-encode → wire-decode reproduces the built structure
    /// exactly (key frame and the encoder/decoder states agree).
    #[test]
    fn build_encode_decode_round_trips_structurally() {
        let mut rng = Rng(0xABCD_EF01_2345_6789);
        let left = random_matrix(&mut rng, 20, 9_000.0);
        let right = random_matrix(&mut rng, 18, 7_000.0);
        let settings = Sv8FrameBuildSettings::default();
        let frame = build_sv8_stereo_frame(&left, &right, 28, &settings).unwrap();
        assert!(frame.nbands > 0);

        let mut w = Sv7BitWriter::new();
        let mut estate = Sv8FrameState::new();
        encode_sv8_stereo_frame(&mut w, &frame, 28, true, true, &mut estate).unwrap();
        let mut bytes = w.finish();
        bytes.extend_from_slice(&[0, 0]);

        let mut reader = Sv7BitReader::new(&bytes);
        let mut dstate = Sv8FrameState::new();
        let mut cns = CnsPrng::new();
        let decoded =
            decode_sv8_stereo_frame(&mut reader, 28, true, true, &mut dstate, &mut cns).unwrap();
        assert_eq!(decoded, frame, "wire round trip must be lossless");
        assert_eq!(estate, dstate);
    }

    /// The reconstruction error of a built frame stays within the
    /// half-step bound of each band's quantiser (after M/S undo, in
    /// the L/R domain the bound doubles at worst: L = M + S sums two
    /// independent half-step errors).
    #[test]
    fn build_reconstruct_error_bounded() {
        use crate::sv8_quantize::quant_step;

        let mut rng = Rng(0x1111_2222_3333_4444);
        let left = random_matrix(&mut rng, 24, 5_000.0);
        let right = random_matrix(&mut rng, 24, 5_000.0);
        let settings = Sv8FrameBuildSettings::default();
        let frame = build_sv8_stereo_frame(&left, &right, 28, &settings).unwrap();

        let mut stereo = reconstruct_sv8_stereo_frame(&frame).unwrap();
        undo_ms_stereo_pinned(&mut stereo, &frame.ms_flags).unwrap();

        for b in 0..frame.nbands as usize {
            // Bound: the coarsest step among the band's coded
            // channels, doubled for the M/S sum, plus zeroing loss for
            // silent channels (≤ step_target/2 by the policy).
            let mut worst = settings.step_target; // silent-band zeroing bound
            for ch in 0..2 {
                if frame.res[b][ch] != 0 {
                    for g in 0..SCF_GRANULES_PER_BAND {
                        worst = worst.max(
                            quant_step(frame.res[b][ch], frame.granule_scf[b][ch][g]).unwrap(),
                        );
                    }
                }
            }
            let bound = worst * (1.0 + 1e-9);
            for k in 0..SAMPLES_PER_BAND {
                let el = (stereo[0][b][k] - left[b][k]).abs();
                let er = (stereo[1][b][k] - right[b][k]).abs();
                assert!(
                    el <= bound && er <= bound,
                    "band {b} sample {k}: err ({el}, {er}) > bound {bound}"
                );
            }
        }
        // Bands past nbands reconstruct silent; their zeroing loss is
        // bounded by the policy threshold.
        for band in left.iter().take(29).skip(frame.nbands as usize) {
            for &x in band.iter() {
                assert!(x.abs() < settings.step_target);
            }
        }
    }

    /// Identical channels elect M/S everywhere (silent side) — the
    /// mono body shape — and reconstruct L == R.
    #[test]
    fn identical_channels_elect_ms_with_silent_side() {
        let mut rng = Rng(0x5555_6666_7777_8888);
        let mono = random_matrix(&mut rng, 16, 8_000.0);
        let settings = Sv8FrameBuildSettings::default();
        let frame = build_sv8_stereo_frame(&mono, &mono, 28, &settings).unwrap();
        assert!(frame.nbands > 0);
        for b in 0..frame.nbands as usize {
            if frame.res[b][0] != 0 {
                assert!(frame.ms_flags[b], "band {b}: coded band must elect M/S");
                assert_eq!(frame.res[b][1], 0, "band {b}: side must be silent");
            }
        }
        let mut stereo = reconstruct_sv8_stereo_frame(&frame).unwrap();
        undo_ms_stereo_pinned(&mut stereo, &frame.ms_flags).unwrap();
        for b in 0..frame.nbands as usize {
            let (l_band, r_band) = (&stereo[0][b], &stereo[1][b]);
            for (k, (&lv, &rv)) in l_band.iter().zip(r_band.iter()).enumerate() {
                assert_eq!(lv, rv, "band {b} sample {k}: L must equal R");
            }
        }
    }

    /// stream_ms = false never elects M/S and never sets a flag.
    #[test]
    fn stream_ms_off_never_elects() {
        let mut rng = Rng(0x9999_AAAA_BBBB_CCCC);
        let mono = random_matrix(&mut rng, 10, 3_000.0);
        let settings = Sv8FrameBuildSettings {
            stream_ms: false,
            ..Default::default()
        };
        let frame = build_sv8_stereo_frame(&mono, &mono, 28, &settings).unwrap();
        assert!(frame.ms_flags.iter().all(|&f| !f));
        // Both channels carry the (identical) signal.
        for b in 0..frame.nbands as usize {
            assert_eq!(frame.res[b][0], frame.res[b][1]);
        }
    }

    /// A silent frame builds to nbands = 0.
    #[test]
    fn silent_frame_builds_empty() {
        let z = zero_subband_matrix();
        let frame = build_sv8_stereo_frame(&z, &z, 28, &Sv8FrameBuildSettings::default()).unwrap();
        assert_eq!(frame.nbands, 0);
        assert!(frame.res.is_empty());
    }

    /// The SCFI selectors always mirror the granule-SCF equalities
    /// (the forced-sharing wire constraint), and band types stay on
    /// the SV8 ring.
    #[test]
    fn scfi_matches_equalities_and_res_stays_on_ring() {
        let mut rng = Rng(0xDDDD_EEEE_FFFF_0001);
        // Very loud input to push at the top of the Res ladder.
        let left = random_matrix(&mut rng, 32, 32_000.0);
        let right = random_matrix(&mut rng, 32, 32_000.0);
        let settings = Sv8FrameBuildSettings {
            step_target: 0.5,
            ..Default::default()
        };
        let frame = build_sv8_stereo_frame(&left, &right, 31, &settings).unwrap();
        for b in 0..frame.nbands as usize {
            for ch in 0..2 {
                let bt = frame.res[b][ch];
                assert!((0..=SV8_MAX_RES).contains(&bt), "band {b} ch {ch}: {bt}");
                if bt != 0 {
                    let scf = &frame.granule_scf[b][ch];
                    let expect = (u8::from(scf[1] == scf[0]) << 1) | u8::from(scf[2] == scf[1]);
                    assert_eq!(frame.scfi[b][ch], expect, "band {b} ch {ch}");
                }
            }
        }
    }

    /// max_band bounds: 0 and 32+ are rejected; the band range is
    /// honoured (nothing above max_band is coded).
    #[test]
    fn max_band_bounds_and_truncation() {
        let mut rng = Rng(0x0F0F_0F0F_0F0F_0F0F);
        let m = random_matrix(&mut rng, 32, 10_000.0);
        assert!(matches!(
            build_sv8_stereo_frame(&m, &m, 0, &Sv8FrameBuildSettings::default()),
            Err(Error::MaxBandOutOfRange(0))
        ));
        assert!(matches!(
            build_sv8_stereo_frame(&m, &m, 32, &Sv8FrameBuildSettings::default()),
            Err(Error::MaxBandOutOfRange(32))
        ));
        let frame = build_sv8_stereo_frame(&m, &m, 5, &Sv8FrameBuildSettings::default()).unwrap();
        assert!(frame.nbands <= 6);
    }
}
