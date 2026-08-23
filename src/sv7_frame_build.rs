//! SV7 **frame builder** — analysed subband data → the
//! [`crate::sv7_file_encode::Sv7EncStereoFrame`] structure the SV7
//! frame writer consumes: the SV7 twin of [`crate::sv8_frame_build`].
//!
//! Same decision layer, SV7 shapes:
//!
//! - the §5.4 band-type ladder runs the full `1..=17` (no SV8 ring
//!   cap), with the same flat s16-domain noise-step allocation policy
//!   ([`crate::sv8_quantize::band_type_for_peak`] — the quantiser
//!   algebra is generation-shared);
//! - SCF indices live on the SV7 6-bit absolute grid
//!   ([`SV7_SCF_MIN`]`..=`[`SV7_SCF_MAX`], the §5.3 raw-6-bit escape's
//!   reach), not the SV8 `−6..=121` fold ring;
//! - the §5.4 1-bit **context selector** (band types 1..=7) is chosen
//!   per band by measuring both tables' exact wire bits and keeping
//!   the cheaper;
//! - posture election (L/R vs M/S per band) is the same measured
//!   rate-distortion comparison as SV8: exact sample-pass wire bits
//!   (via a scratch [`crate::sv7_frame_encode::encode_sv7_band_samples`]
//!   run) against the measured L/R-domain squared error, under
//!   `λ = step_target²/16`;
//! - opt-in CNS emission (`pns_threshold`): a coded channel below the
//!   threshold becomes [`crate::sv7_frame_encode::Sv7EncBand::Cns`]
//!   with the SCF sized so the decoder PRNG reproduces the band's rms
//!   (the same [`crate::sv8_frame_build`] policy; the stream's
//!   version byte then carries the `MP+ 0x17` PNS flag).
//!
//! Everything here is encoder policy over decode-side facts already
//! pinned in the crate; no new format facts.

use crate::frame_reconstruct::SubbandMatrix;
use crate::requant::{band_type_index, DEQUANT_COEFFICIENT_C, QUANTIZER_OFFSET_D, SCF_STEP_RATIO};
use crate::scf::SCF_GRANULES_PER_BAND;
use crate::sv7_band_decode::{band_type_uses_context_selector, SAMPLES_PER_BAND};
use crate::sv7_band_header::SV7_MAX_BAND_INCLUSIVE;
use crate::sv7_bitwriter::Sv7BitWriter;
use crate::sv7_file_encode::Sv7EncStereoFrame;
use crate::sv7_frame_encode::{encode_sv7_band_samples, Sv7EncBand};
use crate::sv8_quantize::{
    band_type_for_peak, choose_granule_scf, quant_step, quantize_granule, SAMPLES_PER_GRANULE,
};
use crate::{Error, Result};

/// Lowest SV7 SCF index: the §5.3 raw-6-bit absolute escape bottoms
/// out at 0.
pub const SV7_SCF_MIN: i32 = 0;

/// Highest SV7 SCF index the 6-bit absolute escape can express.
pub const SV7_SCF_MAX: i32 = 63;

/// Frame-builder settings (encoder policy knobs) — the SV7 twin of
/// [`crate::sv8_frame_build::Sv8FrameBuildSettings`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sv7FrameBuildSettings {
    /// Flat s16-domain quantisation-step target (see the SV8 twin).
    pub step_target: f64,
    /// Whether the stream's §1 mid-side flag is set; gates per-band
    /// M/S election.
    pub stream_ms: bool,
    /// Noise-substitution threshold in s16-domain subband peak units;
    /// `0.0` (default) emits no CNS bands. See the SV8 twin.
    pub pns_threshold: f64,
}

impl Default for Sv7FrameBuildSettings {
    fn default() -> Self {
        Self {
            step_target: 2.0,
            stream_ms: true,
            pns_threshold: 0.0,
        }
    }
}

/// §5.1 wire-reachability of a `Res` value at `band` given the same
/// channel's previous band `prev`: band 0 is a raw 4-bit absolute
/// (`0..=15` only); later bands take a delta in `−5..=3` or an escape
/// to a raw 4-bit absolute (`0..=15`). `Res` 16/17 and the CNS `−1`
/// are reachable only through in-range deltas.
fn res_reachable(band: usize, prev: i8, res: i8) -> bool {
    if band == 0 {
        return (0..=15).contains(&res);
    }
    let delta = i32::from(res) - i32::from(prev);
    (-5..=3).contains(&delta) || (0..=15).contains(&res)
}

/// Flat per-coded-channel SCFI + DSCF overhead estimate (bits) for
/// the posture election, as in the SV8 builder.
const CODED_CHANNEL_OVERHEAD_BITS: f64 = 22.0;

/// RMS of one decoder CNS noise level (staged `cns-prng-params`
/// facts: sum of the four bytes of a PRNG word minus 510 — variance
/// `4 × (256² − 1) / 12`).
const CNS_LEVEL_RMS: f64 = 147.791_573_839_773;

fn band_peak(band: &[f64; SAMPLES_PER_BAND]) -> f64 {
    band.iter().fold(0.0_f64, |a, &x| a.max(x.abs()))
}

/// SCF index whose gain best matches `target_rms` of decoder CNS
/// noise, on the SV7 6-bit grid.
fn cns_scf_for_rms(target_rms: f64) -> i32 {
    let base = CNS_LEVEL_RMS * DEQUANT_COEFFICIENT_C[0];
    if target_rms <= 0.0 {
        return SV7_SCF_MAX;
    }
    let scf = 1 + ((target_rms / base).ln() / SCF_STEP_RATIO.ln()).round() as i32;
    scf.clamp(SV7_SCF_MIN, SV7_SCF_MAX)
}

/// One quantised channel candidate.
struct ChannelCandidate {
    band: Sv7EncBand,
    /// Cached reconstruction for the sse term.
    recon: [f64; SAMPLES_PER_BAND],
    /// Cached exact sample-pass bits (context selector included).
    bits: f64,
}

impl ChannelCandidate {
    fn build(data: &[f64; SAMPLES_PER_BAND], step_target: f64) -> Result<Self> {
        Self::build_at(data, band_type_for_peak(band_peak(data), step_target))
    }

    /// Build the candidate at a caller-fixed band type (the §5.1
    /// legality pass re-quantises at a capped `Res` when the desired
    /// one is not delta/escape-reachable).
    fn build_at(data: &[f64; SAMPLES_PER_BAND], bt: i8) -> Result<Self> {
        if bt == 0 {
            return Ok(Self {
                band: Sv7EncBand::Empty,
                recon: [0.0; SAMPLES_PER_BAND],
                bits: 0.0,
            });
        }

        // Per-granule SCF on the SV7 grid, zero-granule neighbour
        // fill as in the SV8 quantiser.
        let mut scf = [SV7_SCF_MAX; SCF_GRANULES_PER_BAND];
        let mut have = [false; SCF_GRANULES_PER_BAND];
        for g in 0..SCF_GRANULES_PER_BAND {
            let peak = data[g * SAMPLES_PER_GRANULE..(g + 1) * SAMPLES_PER_GRANULE]
                .iter()
                .fold(0.0_f64, |a, &x| a.max(x.abs()));
            if peak > 0.0 {
                scf[g] = choose_granule_scf(bt, peak)?.clamp(SV7_SCF_MIN, SV7_SCF_MAX);
                have[g] = true;
            }
        }
        for g in 1..SCF_GRANULES_PER_BAND {
            if !have[g] && have[g - 1] {
                scf[g] = scf[g - 1];
                have[g] = true;
            }
        }
        for g in (0..SCF_GRANULES_PER_BAND - 1).rev() {
            if !have[g] && have[g + 1] {
                scf[g] = scf[g + 1];
                have[g] = true;
            }
        }

        let mut levels = [0_i32; SAMPLES_PER_BAND];
        let mut recon = [0.0_f64; SAMPLES_PER_BAND];
        for (g, &granule_scf) in scf.iter().enumerate() {
            let range = g * SAMPLES_PER_GRANULE..(g + 1) * SAMPLES_PER_GRANULE;
            quantize_granule(bt, granule_scf, &data[range.clone()], &mut levels[range])?;
            let step = quant_step(bt, granule_scf)?;
            let range = g * SAMPLES_PER_GRANULE..(g + 1) * SAMPLES_PER_GRANULE;
            for k in range {
                recon[k] = f64::from(levels[k]) * step;
            }
        }

        // §5.4 PCM-escape level convention: the wire carries the
        // RAW unsigned levels (`signed + Dc[bt]`, `bt − 1` bits per
        // sample); the Huffman arms carry the signed levels
        // directly. The reconstruction above stays in the signed
        // domain either way (decode re-centres before dequant).
        if bt >= 8 {
            let idx = band_type_index(bt).ok_or(Error::UnsupportedBandType(bt))?;
            let d = i32::from(QUANTIZER_OFFSET_D[idx]);
            for lv in levels.iter_mut() {
                *lv += d;
            }
        }

        // §5.4 context selector: measure both tables, keep the
        // cheaper (the arms without a selector ignore `ctx`).
        let mut best_ctx = 0usize;
        let mut best_bits = sample_bits(bt, 0, scf, &levels)?;
        if band_type_uses_context_selector(bt) {
            let alt = sample_bits(bt, 1, scf, &levels)?;
            if alt < best_bits {
                best_ctx = 1;
                best_bits = alt;
            }
        }

        Ok(Self {
            band: Sv7EncBand::Coded {
                band_type: bt,
                ctx: best_ctx,
                scf,
                levels,
            },
            recon,
            bits: best_bits as f64 + CODED_CHANNEL_OVERHEAD_BITS,
        })
    }
}

/// Exact §5.4 sample-pass wire bits for one coded band candidate.
fn sample_bits(
    bt: i8,
    ctx: usize,
    scf: [i32; SCF_GRANULES_PER_BAND],
    levels: &[i32; SAMPLES_PER_BAND],
) -> Result<u64> {
    let mut scratch = Sv7BitWriter::new();
    encode_sv7_band_samples(
        &mut scratch,
        &Sv7EncBand::Coded {
            band_type: bt,
            ctx,
            scf,
            levels: *levels,
        },
    )?;
    Ok(scratch.bit_len())
}

/// Build one SV7 stereo frame from a pair of analysed subband
/// matrices. Bands `0..=max_band` participate; the output vectors
/// cover exactly `max_band + 1` bands (the SV7 frame layout has no
/// per-frame band truncation — uncoded bands are `Empty`).
///
/// For a **mono** stream pass the same matrix twice: every coded band
/// then elects M/S with a silent side.
///
/// # Errors
///
/// [`Error::MaxBandOutOfRange`] if `max_band` is outside `1..=31`;
/// quantiser/encode errors as the SV8 twin.
pub fn build_sv7_stereo_frame(
    left: &SubbandMatrix,
    right: &SubbandMatrix,
    max_band: u8,
    settings: &Sv7FrameBuildSettings,
) -> Result<Sv7EncStereoFrame> {
    if !(1..=SV7_MAX_BAND_INCLUSIVE).contains(&max_band) {
        return Err(Error::MaxBandOutOfRange(max_band));
    }
    let nb = max_band as usize + 1;
    let lambda = settings.step_target * settings.step_target / 16.0;

    let mut out = Sv7EncStereoFrame {
        left: Vec::with_capacity(nb),
        right: Vec::with_capacity(nb),
        ms_flags: Vec::with_capacity(nb),
    };
    let mut prev_res = [0_i8; 2];

    for b in 0..nb {
        let l = &left[b];
        let r = &right[b];
        let cand_l = ChannelCandidate::build(l, settings.step_target)?;
        let cand_r = ChannelCandidate::build(r, settings.step_target)?;

        let mut mid = [0.0_f64; SAMPLES_PER_BAND];
        let mut side = [0.0_f64; SAMPLES_PER_BAND];
        for k in 0..SAMPLES_PER_BAND {
            mid[k] = (l[k] + r[k]) / 2.0;
            side[k] = (l[k] - r[k]) / 2.0;
        }

        let mut use_ms = false;
        let mut elected: (ChannelCandidate, ChannelCandidate);
        if settings.stream_ms {
            let cand_m = ChannelCandidate::build(&mid, settings.step_target)?;
            let cand_s = ChannelCandidate::build(&side, settings.step_target)?;
            let mut sse_lr = 0.0_f64;
            let mut sse_ms = 0.0_f64;
            for k in 0..SAMPLES_PER_BAND {
                sse_lr += (l[k] - cand_l.recon[k]).powi(2) + (r[k] - cand_r.recon[k]).powi(2);
                sse_ms += (l[k] - (cand_m.recon[k] + cand_s.recon[k])).powi(2)
                    + (r[k] - (cand_m.recon[k] - cand_s.recon[k])).powi(2);
            }
            let j_lr = sse_lr + lambda * (cand_l.bits + cand_r.bits);
            let j_ms = sse_ms + lambda * (cand_m.bits + cand_s.bits);
            if j_ms < j_lr {
                use_ms = true;
                elected = (cand_m, cand_s);
            } else {
                elected = (cand_l, cand_r);
            }
        } else {
            elected = (cand_l, cand_r);
        }

        // §5.1 legality + opt-in CNS, per channel in header order:
        // the coded `Res` must be delta/escape-reachable from the
        // same channel's previous band (re-quantise at the capped
        // `Res` when not), and a CNS `−1` is only emitted where its
        // delta is in range.
        let (d0, d1): (&[f64; SAMPLES_PER_BAND], &[f64; SAMPLES_PER_BAND]) =
            if use_ms { (&mid, &side) } else { (l, r) };
        for (ch, (c, data)) in [(&mut elected.0, d0), (&mut elected.1, d1)]
            .into_iter()
            .enumerate()
        {
            let bt = c.band.res();
            if bt > 0 && !res_reachable(b, prev_res[ch], bt) {
                // 16/17 out of delta reach: cap to the escapable 15.
                *c = ChannelCandidate::build_at(data, 15)?;
            }
            if settings.pns_threshold > 0.0
                && matches!(c.band, Sv7EncBand::Coded { .. })
                && band_peak(data) < settings.pns_threshold
            {
                if res_reachable(b, prev_res[ch], -1) {
                    let rms =
                        (data.iter().map(|x| x * x).sum::<f64>() / SAMPLES_PER_BAND as f64).sqrt();
                    c.band = Sv7EncBand::Cns {
                        scf: [cns_scf_for_rms(rms); SCF_GRANULES_PER_BAND],
                    };
                } else {
                    // `−1` is only delta-reachable from `Res ≤ 4`
                    // (§5.1: delta floor −5; the escape is unsigned).
                    // Code this band as a **bridge** at `Res = 4` —
                    // always reachable itself (escape) and quiet by
                    // election — so the next band can enter the CNS
                    // chain. This mirrors the corpus streams, whose
                    // CNS runs are entered from low/empty bands.
                    *c = ChannelCandidate::build_at(data, 4)?;
                }
            }
            prev_res[ch] = c.band.res();
        }

        let any_coded = !matches!(
            (&elected.0.band, &elected.1.band),
            (Sv7EncBand::Empty, Sv7EncBand::Empty)
        );
        out.ms_flags.push(use_ms && any_coded);
        out.left.push(elected.0.band);
        out.right.push(elected.1.band);
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame_reconstruct::zero_subband_matrix;
    use crate::huffman::Sv7BitReader;
    use crate::sv7_band_header::decode_res_header_grounded;
    use crate::sv7_bitwriter::Sv7BitWriter;
    use crate::sv7_stereo_frame::{decode_sv7_stereo_frame, Sv7ScfMemory};
    use crate::sv7_stereo_frame_encode::encode_sv7_stereo_frame;

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

    /// A built frame wire-encodes and structurally decodes back to
    /// the same band shapes (res values, SCF triples, levels).
    #[test]
    fn build_encode_decode_round_trips_structurally() {
        let mut rng = Rng(0x5eed_0007);
        let max_band = 12u8;
        let left = random_matrix(&mut rng, 13, 4000.0);
        let right = random_matrix(&mut rng, 13, 3000.0);
        let settings = Sv7FrameBuildSettings::default();
        let frame = build_sv7_stereo_frame(&left, &right, max_band, &settings).expect("build");
        assert_eq!(frame.left.len(), 13);

        let mut w = Sv7BitWriter::new();
        let mut enc_scf = Sv7ScfMemory::new();
        encode_sv7_stereo_frame(
            &mut w,
            &frame.left,
            &frame.right,
            &frame.ms_flags,
            true,
            &mut enc_scf,
        )
        .expect("encode");
        let mut bytes = w.finish();
        bytes.extend_from_slice(&[0, 0, 0, 0]);

        // Pass-1 structural agreement: the wire res sequence is the
        // built one.
        let mut reader = Sv7BitReader::new(&bytes);
        let header =
            decode_res_header_grounded(&mut reader, max_band, 2, true).expect("res header");
        for (b, hb) in header.iter().enumerate() {
            assert_eq!(hb.res[0], frame.left[b].res(), "band {b} left res");
            assert_eq!(hb.res[1], frame.right[b].res(), "band {b} right res");
            assert_eq!(
                hb.ms_flag.unwrap_or(false),
                frame.ms_flags[b],
                "band {b} ms"
            );
        }

        // Full-frame decode consumes cleanly (all four passes agree
        // with what the builder emitted).
        let mut reader = Sv7BitReader::new(&bytes);
        let mut dec_scf = Sv7ScfMemory::new();
        let mut cns = crate::cns::CnsPrng::new();
        let decoded = decode_sv7_stereo_frame(&mut reader, max_band, true, &mut dec_scf, &mut cns)
            .expect("decode");
        assert_eq!(decoded.ms_flags, frame.ms_flags);
    }

    /// Identical channels elect M/S with a silent side; decorrelated
    /// loud channels stay L/R.
    #[test]
    fn posture_election_shapes() {
        let mut rng = Rng(0xabc);
        let same = random_matrix(&mut rng, 8, 3000.0);
        let f = build_sv7_stereo_frame(&same, &same, 7, &Sv7FrameBuildSettings::default())
            .expect("build");
        for b in 0..8 {
            if f.left[b].res() != 0 {
                assert!(f.ms_flags[b], "band {b}: identical channels elect M/S");
                assert_eq!(f.right[b].res(), 0, "band {b}: silent side");
            }
        }
        let a = random_matrix(&mut rng, 8, 3000.0);
        let bmat = random_matrix(&mut rng, 8, 3000.0);
        let f2 =
            build_sv7_stereo_frame(&a, &bmat, 7, &Sv7FrameBuildSettings::default()).expect("build");
        let lr_bands = (0..8).filter(|&b| !f2.ms_flags[b]).count();
        assert!(lr_bands >= 6, "decorrelated bands mostly stay L/R");
    }

    /// The CNS knob replaces quiet coded bands with noise bands whose
    /// SCF grows with the band's level.
    #[test]
    fn cns_threshold_emits_noise_bands() {
        let mut rng = Rng(0xc45);
        let quiet = random_matrix(&mut rng, 10, 120.0);
        let settings = Sv7FrameBuildSettings {
            pns_threshold: 400.0,
            stream_ms: false,
            ..Default::default()
        };
        let f = build_sv7_stereo_frame(&quiet, &quiet, 9, &settings).expect("build");
        let cns = f.left.iter().filter(|b| b.res() == -1).count();
        assert!(cns >= 8, "quiet bands become CNS (got {cns})");
        // Off by default.
        let f2 = build_sv7_stereo_frame(&quiet, &quiet, 9, &Sv7FrameBuildSettings::default())
            .expect("build");
        assert!(f2.left.iter().all(|b| b.res() != -1));
    }

    /// SCF indices stay on the SV7 6-bit grid.
    #[test]
    fn scf_indices_stay_on_the_sv7_grid() {
        let mut rng = Rng(0x717);
        for amp in [10.0, 500.0, 32000.0] {
            let m = random_matrix(&mut rng, 20, amp);
            let f = build_sv7_stereo_frame(&m, &m, 19, &Sv7FrameBuildSettings::default())
                .expect("build");
            for band in f.left.iter().chain(f.right.iter()) {
                if let Some(scf) = band.scf() {
                    for s in scf {
                        assert!((SV7_SCF_MIN..=SV7_SCF_MAX).contains(&s), "scf {s}");
                    }
                }
            }
        }
    }
}
