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
use crate::requant::{DEQUANT_COEFFICIENT_C, SCF_STEP_RATIO};
use crate::scf::SCF_GRANULES_PER_BAND;
use crate::sv7_band_decode::SAMPLES_PER_BAND;
use crate::sv7_band_header::SV7_SUBBAND_COUNT;
use crate::sv8_quantize::{
    band_type_for_peak, quant_step, quantize_band, QuantizedBand, SAMPLES_PER_GRANULE, SV8_SCF_MAX,
    SV8_SCF_MIN,
};
use crate::sv8_stereo_frame::Sv8StereoFrameDecode;
use crate::sv8_stereo_frame_encode::band_sample_bits;
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
    /// Noise-substitution threshold, in s16-domain subband peak
    /// units; `0.0` disables CNS emission (the default). When
    /// positive, a coded channel whose band peak sits below the
    /// threshold is emitted as a `Res == −1` noise band: **zero**
    /// sample-pass bits, with the scalefactor chosen so the decoder's
    /// PRNG noise ([`crate::cns`]) reproduces the band's power. The
    /// substituted waveform is noise, not the original — a loudness-
    /// preserving trade of waveform fidelity for rate on hiss-like
    /// bands (the spec §5.4/§6.4 `Res = −1` path; wire-compatible
    /// with the corpus `cns-pns` fixtures).
    pub pns_threshold: f64,
}

impl Default for Sv8FrameBuildSettings {
    /// Default: `step_target = 2.0` s16 LSBs (a noise floor roughly
    /// 84 dB below full scale per band before SCF granularity),
    /// stream M/S on, CNS emission off.
    fn default() -> Self {
        Self {
            step_target: 2.0,
            stream_ms: true,
            pns_threshold: 0.0,
        }
    }
}

/// RMS of one decoder CNS noise level: the PRNG sample is the sum of
/// the four bytes of a 32-bit word minus 510 (staged
/// `cns-prng-params` facts), i.e. a sum of four uniform 0..=255
/// byte values recentred — variance `4 × (256² − 1) / 12`, rms
/// ≈ 147.8. Derived from the staged generator facts alone.
const CNS_LEVEL_RMS: f64 = 147.791_573_839_773;

/// Pick the SCF index whose gain makes the decoder's CNS noise rms
/// (`CNS_LEVEL_RMS × C[0] × gain(scf)`) match `target_rms` most
/// closely (nearest in log domain), clamped to the SV8 SCF ring.
fn cns_scf_for_rms(target_rms: f64) -> i32 {
    let base = CNS_LEVEL_RMS * DEQUANT_COEFFICIENT_C[0];
    if target_rms <= 0.0 {
        return SV8_SCF_MAX;
    }
    let scf = 1 + ((target_rms / base).ln() / SCF_STEP_RATIO.ln()).round() as i32;
    scf.clamp(SV8_SCF_MIN, SV8_SCF_MAX)
}

/// Flat per-coded-channel SCFI + DSCF overhead estimate, in bits,
/// for the posture election (the exact DSCF spend depends on the
/// temporal prediction state, which is not known band-locally; a
/// flat mean keeps the comparison honest between one- and
/// two-coded-channel postures). Policy only.
const CODED_CHANNEL_OVERHEAD_BITS: f64 = 22.0;

/// One quantised channel candidate for the posture election.
struct ChannelCandidate {
    bt: i8,
    q: QuantizedBand,
}

impl ChannelCandidate {
    fn build(data: &[f64; SAMPLES_PER_BAND], step_target: f64) -> Result<Self> {
        let bt = band_type_for_peak(band_peak(data), step_target).min(SV8_MAX_RES);
        let q = if bt == 0 {
            QuantizedBand {
                scf: [0; SCF_GRANULES_PER_BAND],
                levels: [0; SAMPLES_PER_BAND],
            }
        } else {
            quantize_band(bt, data)?
        };
        Ok(Self { bt, q })
    }

    /// Exact §6.4 sample-pass wire bits plus the flat SCF overhead.
    fn bits(&self) -> Result<f64> {
        if self.bt == 0 {
            return Ok(0.0);
        }
        Ok(band_sample_bits(self.bt, &self.q.levels)? as f64 + CODED_CHANNEL_OVERHEAD_BITS)
    }

    /// Reconstructed samples (`level × step`), zero for an uncoded
    /// band.
    fn recon(&self) -> Result<[f64; SAMPLES_PER_BAND]> {
        let mut out = [0.0_f64; SAMPLES_PER_BAND];
        if self.bt == 0 {
            return Ok(out);
        }
        for g in 0..SCF_GRANULES_PER_BAND {
            let step = quant_step(self.bt, self.q.scf[g])?;
            let range = g * SAMPLES_PER_GRANULE..(g + 1) * SAMPLES_PER_GRANULE;
            for (o, &lv) in out[range.clone()].iter_mut().zip(&self.q.levels[range]) {
                *o = f64::from(lv) * step;
            }
        }
        Ok(out)
    }
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

    // Rate-distortion weight for the posture election: at a uniform
    // quantiser near the step target, one extra bit of rate buys
    // roughly a 4x noise-power reduction, i.e. Δsse per sample per
    // bit ≈ step²/16 — used as the Lagrangian λ so bit costs and
    // squared errors are commensurable. Policy only.
    let lambda = settings.step_target * settings.step_target / 16.0;

    for b in 0..nb_considered {
        let l = &left[b];
        let r = &right[b];

        // L/R posture candidate.
        let cand_l = ChannelCandidate::build(l, settings.step_target)?;
        let cand_r = ChannelCandidate::build(r, settings.step_target)?;

        // M/S posture candidate (only when the stream allows it),
        // elected by measured rate + distortion: the mid/side
        // transform halves a correlated pair's side energy (cheap)
        // but sums the two channels' quantisation errors back into
        // L/R (r390-pinned undo `L = M + S`, `R = M − S`), which the
        // sse term prices in — unlike a pure alphabet-size estimate.
        let mut mid = [0.0_f64; SAMPLES_PER_BAND];
        let mut side = [0.0_f64; SAMPLES_PER_BAND];
        for k in 0..SAMPLES_PER_BAND {
            mid[k] = (l[k] + r[k]) / 2.0;
            side[k] = (l[k] - r[k]) / 2.0;
        }

        let mut use_ms = false;
        let mut cand_ms: Option<(ChannelCandidate, ChannelCandidate)> = None;
        if settings.stream_ms {
            let cand_m = ChannelCandidate::build(&mid, settings.step_target)?;
            let cand_s = ChannelCandidate::build(&side, settings.step_target)?;

            // Measured L/R-domain squared error of each posture.
            let (rec_l, rec_r) = (cand_l.recon()?, cand_r.recon()?);
            let (rec_m, rec_s) = (cand_m.recon()?, cand_s.recon()?);
            let mut sse_lr = 0.0_f64;
            let mut sse_ms = 0.0_f64;
            for k in 0..SAMPLES_PER_BAND {
                sse_lr += (l[k] - rec_l[k]).powi(2) + (r[k] - rec_r[k]).powi(2);
                sse_ms +=
                    (l[k] - (rec_m[k] + rec_s[k])).powi(2) + (r[k] - (rec_m[k] - rec_s[k])).powi(2);
            }
            let j_lr = sse_lr + lambda * (cand_l.bits()? + cand_r.bits()?);
            let j_ms = sse_ms + lambda * (cand_m.bits()? + cand_s.bits()?);
            if j_ms < j_lr {
                use_ms = true;
                cand_ms = Some((cand_m, cand_s));
            }
        }

        let (mut c0, mut c1) = if use_ms {
            let (m, s) = cand_ms.expect("set with use_ms");
            (m, s)
        } else {
            (cand_l, cand_r)
        };

        // CNS election (opt-in): a coded channel whose peak sits
        // below the threshold becomes a Res = −1 noise band — zero
        // sample-pass bits; a single SCF index across the three
        // granules (so SCFI shares everything) sets the decoder
        // PRNG's noise power to the channel's measured rms.
        if settings.pns_threshold > 0.0 {
            let (d0, d1): (&[f64; SAMPLES_PER_BAND], &[f64; SAMPLES_PER_BAND]) =
                if use_ms { (&mid, &side) } else { (l, r) };
            for (c, data) in [(&mut c0, d0), (&mut c1, d1)] {
                if c.bt > 0 && band_peak(data) < settings.pns_threshold {
                    let rms =
                        (data.iter().map(|x| x * x).sum::<f64>() / SAMPLES_PER_BAND as f64).sqrt();
                    c.bt = -1;
                    c.q = QuantizedBand {
                        scf: [cns_scf_for_rms(rms); SCF_GRANULES_PER_BAND],
                        levels: [0; SAMPLES_PER_BAND],
                    };
                }
            }
        }

        let mut band_scfi = [0_u8; 2];
        let mut band_scf = [[0_i32; SCF_GRANULES_PER_BAND]; 2];
        let mut band_levels = [[0_i32; SAMPLES_PER_BAND]; 2];
        for (ch, c) in [&c0, &c1].into_iter().enumerate() {
            if c.bt == 0 {
                continue;
            }
            band_scfi[ch] = scfi_for(&c.q.scf);
            band_scf[ch] = c.q.scf;
            band_levels[ch] = c.q.levels;
        }

        res.push([c0.bt, c1.bt]);
        ms_flags.push(use_ms && (c0.bt != 0 || c1.bt != 0));
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
        let (left_out, right_out) = stereo.split_at(1);
        for (b, (l_band, r_band)) in left_out[0]
            .iter()
            .zip(right_out[0].iter())
            .take(frame.nbands as usize)
            .enumerate()
        {
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
