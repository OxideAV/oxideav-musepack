//! SV7 single-channel frame-body **encode** — inverse of
//! [`crate::sv7_frame_decode::decode_sv7_frame_channel`].
//!
//! Composes the SV7 encode sub-walks
//! ([`crate::sv7_scf_encode`], [`crate::sv7_sample_encode`]) into a
//! full per-channel frame **body** writer, in the exact §5 phase order
//! the decoder reads: for each band, given its `Res` (band-type),
//!
//! 1. **empty** (`Res == 0`): write nothing (the subband is silent);
//! 2. **CNS** (`Res == -1`): write nothing (a noise band is
//!    PRNG-synthesised on decode, so it carries no body bits);
//! 3. **coded** (`Res` in `1..=17`): write the §5.2/§5.3 SCF layer
//!    ([`crate::sv7_scf_encode::encode_sv7_band_scf_auto`], threading the
//!    previous coded band's `SCF[2]` forward), the §5.4 **1-bit context
//!    selector** when the band-type uses one, then the 36 §5.4 sample
//!    levels ([`crate::sv7_sample_encode::encode_sv7_band`]).
//!
//! This is the body **after** the §5.1 `Res` (band-type) header — which
//! [`crate::sv7_band_header_encode::encode_res_header_grounded`] emits —
//! exactly mirroring [`crate::sv7_frame_decode::decode_sv7_frame_channel`],
//! which takes the already-decoded `Res` sequence as a parameter and
//! reads only the body. The **cross-channel interleaving** and the **M/S
//! undo** remain GAP (the same gaps the decode side documents), so this
//! stays scoped to one channel's body.
//!
//! Source-of-record: `docs/audio/musepack/spec/musepack-headers-and-coding.md`
//! §5.1–§5.4. No new format facts — composition of the grounded encode
//! sub-walks in the documented §5 phase order, round-tripped against the
//! decoder that already exists.

use crate::sv7_band_decode::SAMPLES_PER_BAND;
use crate::sv7_band_header::SV7_SUBBAND_COUNT;
use crate::sv7_bitwriter::Sv7BitWriter;
use crate::sv7_frame_decode::band_type_uses_context_selector;
use crate::sv7_sample_encode::encode_sv7_band;
use crate::sv7_scf_encode::encode_sv7_band_scf_auto;
use crate::{Error, Result};

/// One band of a channel's frame body, in the three §5.4 shapes the
/// body encoder handles. Empty / CNS bands carry no body data (they are
/// reconstructed without reading the stream); a coded band carries its
/// band-type, the 1-bit context selector, its three per-granule SCF
/// indices, and its 36 sample levels.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Sv7EncBand {
    /// `Res == 0`: silent subband. No body bits.
    Empty,
    /// `Res == -1`: noise substitution. No body bits (PRNG on decode).
    Cns,
    /// `Res` in `1..=17`: a coded band.
    Coded {
        /// The band-type (`Res`), driving the §5.4 sample arm.
        band_type: i8,
        /// The §5.4 1-bit context selector (`0` / `1`); ignored by the
        /// no-context arms (PCM-escape). Must be `0` or `1`.
        ctx: usize,
        /// The three per-granule SCF indices; the SCFI selector is
        /// derived from their sharing pattern
        /// ([`crate::sv7_scf_encode::choose_scfi`]).
        scf: [i32; 3],
        /// The 36 quantised sample levels for the band.
        levels: [i32; SAMPLES_PER_BAND],
    },
}

impl Sv7EncBand {
    /// The band's `Res` (band-type) as it appears in the §5.1 header
    /// sequence: `0` for [`Sv7EncBand::Empty`], `-1` for
    /// [`Sv7EncBand::Cns`], and the coded band's `band_type` otherwise.
    pub fn res(&self) -> i8 {
        match self {
            Sv7EncBand::Empty => 0,
            Sv7EncBand::Cns => -1,
            Sv7EncBand::Coded { band_type, .. } => *band_type,
        }
    }
}

/// Derive the `Res` (band-type) sequence a `bands` slice implies — the
/// column [`crate::sv7_frame_decode::decode_sv7_frame_channel`] takes as
/// its `res_per_band` argument.
pub fn res_sequence(bands: &[Sv7EncBand]) -> Vec<i8> {
    bands.iter().map(Sv7EncBand::res).collect()
}

/// Encode one channel's SV7 frame **body** into `writer`, the exact
/// inverse of [`crate::sv7_frame_decode::decode_sv7_frame_channel`].
///
/// `bands` is ascending subband order; `first_scf_ref` is the §5.3
/// `SCF[0]` reference for the first coded band (the channel's SCF anchor,
/// GAP per §2.6 — pass `0` for the relative-anchor convention). Empty and
/// CNS bands emit no bits; each coded band emits SCF → context-selector
/// (when applicable) → 36 samples, threading `SCF[2]` forward.
///
/// # Errors
///
/// - [`Error::MaxBandOutOfRange`] if `bands` is longer than
///   [`SV7_SUBBAND_COUNT`].
/// - [`Error::UnsupportedBandType`] for a coded band whose `band_type` is
///   outside `1..=17`, or a `ctx` outside `0..=1`.
/// - [`Error::SampleOutOfRange`] / [`Error::SymbolNotEncodable`] from an
///   SCF or sample arm whose input is not representable.
pub fn encode_sv7_frame_channel(
    writer: &mut Sv7BitWriter,
    bands: &[Sv7EncBand],
    first_scf_ref: i32,
) -> Result<()> {
    if bands.len() > SV7_SUBBAND_COUNT {
        return Err(Error::MaxBandOutOfRange(bands.len() as u8));
    }
    let mut prev_scf2 = first_scf_ref;
    for band in bands {
        match band {
            Sv7EncBand::Empty | Sv7EncBand::Cns => {
                // No body bits: silent / PRNG-synthesised on decode.
            }
            Sv7EncBand::Coded {
                band_type,
                ctx,
                scf,
                levels,
            } => {
                if *ctx > 1 {
                    return Err(Error::UnsupportedBandType(*band_type));
                }
                // §5.2/§5.3 SCF: SCFI derived from the sharing pattern,
                // SCF[0] off the previous coded band's SCF[2].
                encode_sv7_band_scf_auto(writer, *scf, prev_scf2)?;
                prev_scf2 = scf[2];
                // §5.4 context selector, only for the context-using arms.
                if band_type_uses_context_selector(*band_type) {
                    writer.write_bits(*ctx as u32, 1);
                }
                // §5.4 samples.
                encode_sv7_band(writer, *band_type, *ctx, levels)?;
            }
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cns::CnsPrng;
    use crate::huffman::{
        sv7_q1_ctx, sv7_q2_ctx, sv7_q3_ctx, sv7_q4_ctx, sv7_q5_ctx, sv7_q6_ctx, sv7_q7_ctx,
        Sv7BitReader,
    };
    use crate::sv7_band_header::Sv7ResBand;
    use crate::sv7_band_header_encode::encode_res_header_grounded;
    use crate::sv7_frame_decode::decode_sv7_frame_channel;

    /// Encode `bands`' body, decode it back through
    /// `decode_sv7_frame_channel`, and assert the coded bands round-trip
    /// (band_type + subband index + SCF triple + levels). CNS bands are
    /// checked against a fresh PRNG walk.
    fn assert_body_round_trips(bands: &[Sv7EncBand], first_scf_ref: i32) {
        let mut w = Sv7BitWriter::new();
        encode_sv7_frame_channel(&mut w, bands, first_scf_ref).expect("encode");
        let mut bytes = w.finish();
        bytes.extend_from_slice(&[0, 0, 0, 0]); // peek padding

        let res = res_sequence(bands);
        let mut r = Sv7BitReader::new(&bytes);
        let mut cns = CnsPrng::new();
        let decoded = decode_sv7_frame_channel(&mut r, &res, first_scf_ref, &mut cns).expect("dec");

        // A reference PRNG to reproduce CNS-band expected levels in order.
        let mut ref_cns = CnsPrng::new();

        // Walk expected coded/CNS bands and match against decoded records
        // (which skip empty subbands).
        let mut di = 0usize;
        for (subband, band) in bands.iter().enumerate() {
            match band {
                Sv7EncBand::Empty => {}
                Sv7EncBand::Cns => {
                    let rec = &decoded[di];
                    di += 1;
                    assert_eq!(rec.subband, subband);
                    assert_eq!(rec.band_type, -1);
                    let mut expect = [0_i32; SAMPLES_PER_BAND];
                    ref_cns.fill_samples(&mut expect);
                    assert_eq!(rec.levels, expect, "CNS band {subband}");
                }
                Sv7EncBand::Coded {
                    band_type,
                    scf,
                    levels,
                    ..
                } => {
                    let rec = &decoded[di];
                    di += 1;
                    assert_eq!(rec.subband, subband);
                    assert_eq!(rec.band_type, *band_type);
                    // Non-negative SCF indices round-trip exactly through
                    // the u32 granule triple.
                    let want_scf = [scf[0] as u32, scf[1] as u32, scf[2] as u32];
                    assert_eq!(rec.granule_scf, want_scf, "SCF band {subband}");
                    assert_eq!(rec.levels, *levels, "levels band {subband}");
                }
            }
        }
        assert_eq!(di, decoded.len(), "record count");
    }

    fn table_alphabet(bt: i8) -> Vec<i32> {
        let t = match bt {
            3 => sv7_q3_ctx(0),
            4 => sv7_q4_ctx(0),
            5 => sv7_q5_ctx(0),
            6 => sv7_q6_ctx(0),
            7 => sv7_q7_ctx(0),
            _ => unreachable!(),
        };
        let mut v: Vec<i32> = t.iter().map(|e| e.value as i32).collect();
        v.dedup();
        v
    }

    fn ramp_levels(alphabet: &[i32]) -> [i32; SAMPLES_PER_BAND] {
        let mut s = [0_i32; SAMPLES_PER_BAND];
        for (i, slot) in s.iter_mut().enumerate() {
            *slot = alphabet[i % alphabet.len()];
        }
        s
    }

    #[test]
    fn empty_only_channel_writes_nothing() {
        let bands = vec![Sv7EncBand::Empty; 5];
        let mut w = Sv7BitWriter::new();
        encode_sv7_frame_channel(&mut w, &bands, 0).unwrap();
        assert!(w.is_empty());
        assert_body_round_trips(&bands, 0);
    }

    #[test]
    fn single_cns_band_round_trips_against_prng() {
        assert_body_round_trips(&[Sv7EncBand::Cns], 0);
    }

    #[test]
    fn grouped3_coded_band_round_trips() {
        let mut levels = [0_i32; SAMPLES_PER_BAND];
        for (i, v) in levels.iter_mut().enumerate() {
            *v = (i as i32 % 3) - 1;
        }
        assert_body_round_trips(
            &[Sv7EncBand::Coded {
                band_type: 1,
                ctx: 1,
                scf: [20, 22, 22],
                levels,
            }],
            10,
        );
    }

    #[test]
    fn grouped2_coded_band_round_trips() {
        let mut levels = [0_i32; SAMPLES_PER_BAND];
        for (i, v) in levels.iter_mut().enumerate() {
            *v = (i as i32 % 5) - 2;
        }
        // Touch the q1/q2 accessors for import parity with the huffman set.
        let _ = (sv7_q1_ctx(0).len(), sv7_q2_ctx(0).len());
        assert_body_round_trips(
            &[Sv7EncBand::Coded {
                band_type: 2,
                ctx: 0,
                scf: [40, 40, 45],
                levels,
            }],
            30,
        );
    }

    #[test]
    fn huffman_and_pcm_bands_round_trip() {
        for bt in 3..=7i8 {
            let levels = ramp_levels(&table_alphabet(bt));
            assert_body_round_trips(
                &[Sv7EncBand::Coded {
                    band_type: bt,
                    ctx: 0,
                    scf: [50, 50, 50],
                    levels,
                }],
                48,
            );
        }
        // PCM escape band_type 8 (7 bits/sample).
        let mut levels = [0_i32; SAMPLES_PER_BAND];
        for (i, v) in levels.iter_mut().enumerate() {
            *v = (i as i32) & 0x7F;
        }
        assert_body_round_trips(
            &[Sv7EncBand::Coded {
                band_type: 8,
                ctx: 0,
                scf: [60, 61, 62],
                levels,
            }],
            55,
        );
    }

    #[test]
    fn mixed_channel_body_round_trips_with_scf_threading() {
        // empty, CNS, two coded bands (SCF[2] threads), empty, coded.
        let g3: [i32; SAMPLES_PER_BAND] = core::array::from_fn(|i| (i as i32 % 3) - 1);
        let q3_levels = ramp_levels(&table_alphabet(3));
        let bands = vec![
            Sv7EncBand::Empty,
            Sv7EncBand::Cns,
            Sv7EncBand::Coded {
                band_type: 1,
                ctx: 0,
                scf: [10, 11, 12],
                levels: g3,
            },
            Sv7EncBand::Coded {
                band_type: 3,
                ctx: 1,
                scf: [14, 14, 14],
                levels: q3_levels,
            },
            Sv7EncBand::Empty,
            Sv7EncBand::Cns,
        ];
        assert_body_round_trips(&bands, 5);
    }

    #[test]
    fn header_plus_body_compose_for_one_channel() {
        // A full single-channel frame: §5.1 Res header (mono) + §5 body.
        // Encode both, then decode both, confirming the res sequence and
        // the per-band records line up.
        let q3_levels = ramp_levels(&table_alphabet(3));
        let bands = vec![
            Sv7EncBand::Empty,
            Sv7EncBand::Coded {
                band_type: 3,
                ctx: 0,
                scf: [7, 7, 7],
                levels: q3_levels,
            },
            Sv7EncBand::Cns,
        ];
        let res = res_sequence(&bands);
        let res_bands: Vec<Sv7ResBand> = res
            .iter()
            .map(|&r| Sv7ResBand {
                res: [r, r],
                ms_flag: None,
            })
            .collect();

        let mut w = Sv7BitWriter::new();
        // Mono header (nch=1, stream_ms=false) then the body.
        encode_res_header_grounded(&mut w, &res_bands, 1, false).unwrap();
        encode_sv7_frame_channel(&mut w, &bands, 0).unwrap();
        let mut bytes = w.finish();
        bytes.extend_from_slice(&[0, 0, 0, 0]);

        let mut r = Sv7BitReader::new(&bytes);
        let max_band = (res.len() - 1) as u8;
        let decoded_res =
            crate::sv7_band_header::decode_res_header_grounded(&mut r, max_band, 1, false).unwrap();
        let decoded_res_seq: Vec<i8> = decoded_res.iter().map(|b| b.res[0]).collect();
        assert_eq!(decoded_res_seq, res);

        let mut cns = CnsPrng::new();
        let decoded = decode_sv7_frame_channel(&mut r, &decoded_res_seq, 0, &mut cns).unwrap();
        assert_eq!(decoded.len(), 2); // coded + CNS (empty emits none)
        assert_eq!(decoded[0].subband, 1);
        assert_eq!(decoded[0].band_type, 3);
        assert_eq!(decoded[0].granule_scf, [7, 7, 7]);
        assert_eq!(decoded[1].subband, 2);
        assert_eq!(decoded[1].band_type, -1);
    }

    #[test]
    fn rejects_too_many_bands() {
        let bands = vec![Sv7EncBand::Empty; SV7_SUBBAND_COUNT + 1];
        let mut w = Sv7BitWriter::new();
        assert_eq!(
            encode_sv7_frame_channel(&mut w, &bands, 0),
            Err(Error::MaxBandOutOfRange((SV7_SUBBAND_COUNT + 1) as u8)),
        );
    }

    #[test]
    fn rejects_bad_ctx() {
        let bands = vec![Sv7EncBand::Coded {
            band_type: 3,
            ctx: 2,
            scf: [1, 1, 1],
            levels: [0; SAMPLES_PER_BAND],
        }];
        let mut w = Sv7BitWriter::new();
        assert_eq!(
            encode_sv7_frame_channel(&mut w, &bands, 0),
            Err(Error::UnsupportedBandType(3)),
        );
    }

    #[test]
    fn res_helpers_map_variants() {
        assert_eq!(Sv7EncBand::Empty.res(), 0);
        assert_eq!(Sv7EncBand::Cns.res(), -1);
        assert_eq!(
            Sv7EncBand::Coded {
                band_type: 5,
                ctx: 0,
                scf: [0; 3],
                levels: [0; SAMPLES_PER_BAND],
            }
            .res(),
            5,
        );
        let bands = [Sv7EncBand::Empty, Sv7EncBand::Cns];
        assert_eq!(res_sequence(&bands), vec![0, -1]);
        // q4..q7 accessors touched for import parity.
        let _ = (
            sv7_q4_ctx(0).len(),
            sv7_q5_ctx(0).len(),
            sv7_q6_ctx(0).len(),
            sv7_q7_ctx(0).len(),
        );
    }
}
