//! SV7 stereo (two-channel) frame-body **encode** — inverse of
//! [`crate::sv7_stereo_frame::decode_sv7_stereo_frame`]'s bitstream
//! layout.
//!
//! Composes the §5.1 header encoder
//! ([`crate::sv7_band_header_encode::encode_res_header_grounded`]) and
//! the single-channel body encoder
//! ([`crate::sv7_frame_encode::encode_sv7_frame_channel`]) into a full
//! stereo frame **body** writer, in the exact §5 phase order the stereo
//! decoder reads:
//!
//! 1. **§5.1 band-type header** — both channels interleaved per band
//!    (left `Res` then right `Res`) plus the per-band M/S bit, a single
//!    shared header sweep.
//! 2. **§5.3/§5.4 bodies** — *"Left channel is decoded first, then
//!    right."* So the **whole** SCF-then-samples body for the left
//!    channel, then the **whole** body for the right — a per-channel
//!    sweep, not a per-band interleave.
//!
//! The CNS PRNG threads left-channel-then-right on decode; since CNS
//! bands emit no body bits, the encoder simply omits them and the
//! decoder's shared PRNG reproduces the noise in that order.
//!
//! The two §2.6 GAPs the decoder threads as caller arguments (the
//! absolute SCF anchor and the M/S-undo arithmetic) are unchanged here:
//! the absolute anchor is `first_scf_ref` (pass `0` for the relative
//! convention) and the M/S-undo is not an encode concern — the per-band
//! M/S flags are written verbatim into the §5.1 header.
//!
//! Source-of-record: `docs/audio/musepack/spec/musepack-headers-and-coding.md`
//! §5.1 (shared header, both channels), §5.2/§5.3/§5.4 ("left first, then
//! right"). No new format facts — composition of the grounded encode
//! sub-walks, round-tripped against the decoder that already exists.

use crate::sv7_band_header::{Sv7ResBand, SV7_MAX_BAND_INCLUSIVE};
use crate::sv7_bitwriter::Sv7BitWriter;
use crate::sv7_frame_encode::{encode_sv7_frame_channel, Sv7EncBand};
use crate::{Error, Result};

/// Build the §5.1 [`Sv7ResBand`] header sequence for a stereo frame from
/// the two channels' band specs and the per-band M/S flags.
///
/// The per-band M/S flag is carried as `Some(flag)` only for bands with
/// a non-zero channel (the condition under which the decoder reads it);
/// silent bands get `None`.
fn build_res_bands(
    left: &[Sv7EncBand],
    right: &[Sv7EncBand],
    ms_flags: &[bool],
) -> Result<Vec<Sv7ResBand>> {
    if left.len() != right.len() || left.len() != ms_flags.len() {
        // A length mismatch is a caller bug; report it via the same
        // out-of-range channel the band-count guard uses.
        let implied = left
            .len()
            .max(right.len())
            .saturating_sub(1)
            .min(u8::MAX as usize) as u8;
        return Err(Error::MaxBandOutOfRange(implied));
    }
    let mut out = Vec::with_capacity(left.len());
    for ((l, r), &ms) in left.iter().zip(right.iter()).zip(ms_flags.iter()) {
        let res = [l.res(), r.res()];
        let has_samples = res[0] != 0 || res[1] != 0;
        out.push(Sv7ResBand {
            res,
            ms_flag: if has_samples { Some(ms) } else { None },
        });
    }
    Ok(out)
}

/// Encode one SV7 **stereo** frame body into `writer`, the exact inverse
/// of the bitstream [`crate::sv7_stereo_frame::decode_sv7_stereo_frame`]
/// reads.
///
/// `left` / `right` are the two channels' band specs (ascending subband
/// order, equal length ≤ 32). `ms_flags` is the per-band M/S flag (only
/// emitted for bands with a non-zero channel, when `stream_ms` is set).
/// `first_scf_ref` seeds each channel's first coded band (GAP §2.6
/// anchor; pass `0`).
///
/// Emits the §5.1 shared header (both channels + M/S bits), then the
/// left channel's full body, then the right channel's full body.
///
/// # Errors
///
/// - [`Error::MaxBandOutOfRange`] if the channel/flag lengths disagree or
///   exceed `SV7_MAX_BAND_INCLUSIVE + 1`.
/// - [`Error::SampleOutOfRange`] / [`Error::SymbolNotEncodable`] /
///   [`Error::UnsupportedBandType`] from a header or body sub-walk.
pub fn encode_sv7_stereo_frame(
    writer: &mut Sv7BitWriter,
    left: &[Sv7EncBand],
    right: &[Sv7EncBand],
    ms_flags: &[bool],
    stream_ms: bool,
    first_scf_ref: i32,
) -> Result<()> {
    if left.len() > SV7_MAX_BAND_INCLUSIVE as usize + 1 {
        return Err(Error::MaxBandOutOfRange(left.len() as u8));
    }
    let res_bands = build_res_bands(left, right, ms_flags)?;
    // §5.1 shared band-type header (both channels + per-band M/S).
    crate::sv7_band_header_encode::encode_res_header_grounded(writer, &res_bands, 2, stream_ms)?;
    // §5.3/§5.4: left channel body, then right channel body.
    encode_sv7_frame_channel(writer, left, first_scf_ref)?;
    encode_sv7_frame_channel(writer, right, first_scf_ref)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cns::CnsPrng;
    use crate::huffman::{sv7_q3_ctx, Sv7BitReader};
    use crate::sv7_band_decode::SAMPLES_PER_BAND;
    use crate::sv7_band_header::decode_res_header_grounded;
    use crate::sv7_frame_decode::decode_sv7_frame_channel;
    use crate::sv7_stereo_frame::decode_sv7_stereo_frame;

    fn q3_alphabet() -> Vec<i32> {
        let mut v: Vec<i32> = sv7_q3_ctx(0).iter().map(|e| e.value as i32).collect();
        v.dedup();
        v
    }

    fn q3_levels() -> [i32; SAMPLES_PER_BAND] {
        let a = q3_alphabet();
        core::array::from_fn(|i| a[i % a.len()])
    }

    /// Encode a stereo frame, then decode header + both channel bodies at
    /// the BandLevels layer (the same reader the stereo decoder walks) and
    /// assert res / SCF / levels round-trip for each channel.
    fn assert_stereo_round_trips(
        left: &[Sv7EncBand],
        right: &[Sv7EncBand],
        ms_flags: &[bool],
        stream_ms: bool,
        first_scf_ref: i32,
    ) {
        let mut w = Sv7BitWriter::new();
        encode_sv7_stereo_frame(&mut w, left, right, ms_flags, stream_ms, first_scf_ref)
            .expect("encode");
        let mut bytes = w.finish();
        bytes.extend_from_slice(&[0, 0, 0, 0]);

        let max_band = (left.len() - 1) as u8;
        let mut r = Sv7BitReader::new(&bytes);
        let header = decode_res_header_grounded(&mut r, max_band, 2, stream_ms).expect("hdr");
        assert_eq!(header.len(), left.len());
        for (i, band) in header.iter().enumerate() {
            assert_eq!(band.res[0], left[i].res(), "band {i} left res");
            assert_eq!(band.res[1], right[i].res(), "band {i} right res");
            let has_samples = band.res[0] != 0 || band.res[1] != 0;
            if stream_ms && has_samples {
                assert_eq!(band.ms_flag, Some(ms_flags[i]), "band {i} ms");
            } else {
                assert_eq!(band.ms_flag, None, "band {i} ms absent");
            }
        }
        let left_res: Vec<i8> = header.iter().map(|b| b.res[0]).collect();
        let right_res: Vec<i8> = header.iter().map(|b| b.res[1]).collect();

        // Left body then right body, sharing the PRNG (left-then-right).
        let mut cns = CnsPrng::new();
        let ldec = decode_sv7_frame_channel(&mut r, &left_res, first_scf_ref, &mut cns).unwrap();
        let rdec = decode_sv7_frame_channel(&mut r, &right_res, first_scf_ref, &mut cns).unwrap();

        assert_channel(left, &ldec);
        assert_channel(right, &rdec);
    }

    fn assert_channel(spec: &[Sv7EncBand], decoded: &[crate::frame_reconstruct::BandLevels]) {
        let mut di = 0;
        for (subband, band) in spec.iter().enumerate() {
            match band {
                Sv7EncBand::Empty => {}
                Sv7EncBand::Cns => {
                    assert_eq!(decoded[di].subband, subband);
                    assert_eq!(decoded[di].band_type, -1);
                    di += 1;
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
                    assert_eq!(
                        rec.granule_scf,
                        [scf[0] as u32, scf[1] as u32, scf[2] as u32]
                    );
                    assert_eq!(rec.levels, *levels);
                }
            }
        }
        assert_eq!(di, decoded.len());
    }

    #[test]
    fn all_silent_stereo_frame_round_trips() {
        assert_stereo_round_trips(
            &[Sv7EncBand::Empty],
            &[Sv7EncBand::Empty],
            &[false],
            false,
            0,
        );
    }

    #[test]
    fn coded_both_channels_with_ms_flag_round_trips() {
        let coded = |bt: i8| Sv7EncBand::Coded {
            band_type: bt,
            ctx: 0,
            scf: [7, 7, 7],
            levels: q3_levels(),
        };
        assert_stereo_round_trips(&[coded(3)], &[coded(3)], &[true], true, 5);
    }

    #[test]
    fn cns_threads_left_then_right_and_decodes_via_stereo_frame() {
        // Both channels: band0 empty, band1 CNS. Encode, then run the
        // full stereo decoder — its channels[1] rows must differ between
        // the two channels because the shared PRNG advanced left→right.
        let left = vec![Sv7EncBand::Empty, Sv7EncBand::Cns];
        let right = vec![Sv7EncBand::Empty, Sv7EncBand::Cns];
        let ms = vec![false, false];
        assert_stereo_round_trips(&left, &right, &ms, false, 0);

        let mut w = Sv7BitWriter::new();
        encode_sv7_stereo_frame(&mut w, &left, &right, &ms, false, 0).unwrap();
        let mut bytes = w.finish();
        bytes.extend_from_slice(&[0, 0, 0, 0]);
        let mut r = Sv7BitReader::new(&bytes);
        let mut cns = CnsPrng::new();
        let frame = decode_sv7_stereo_frame(&mut r, 1, false, 0, 0, &mut cns).unwrap();
        assert_eq!(frame.ms_flags, vec![false, false]);
        // Subband-1 (CNS) rows differ between channels: PRNG advanced.
        assert_ne!(frame.channels[0][1], frame.channels[1][1]);
    }

    #[test]
    fn mixed_asymmetric_channels_round_trip() {
        // Left: empty, coded(3), CNS. Right: coded(1), empty, coded(3).
        let g3: [i32; SAMPLES_PER_BAND] = core::array::from_fn(|i| (i as i32 % 3) - 1);
        let left = vec![
            Sv7EncBand::Empty,
            Sv7EncBand::Coded {
                band_type: 3,
                ctx: 1,
                scf: [10, 10, 10],
                levels: q3_levels(),
            },
            Sv7EncBand::Cns,
        ];
        let right = vec![
            Sv7EncBand::Coded {
                band_type: 1,
                ctx: 0,
                scf: [20, 21, 22],
                levels: g3,
            },
            Sv7EncBand::Empty,
            Sv7EncBand::Coded {
                band_type: 3,
                ctx: 0,
                scf: [15, 15, 15],
                levels: q3_levels(),
            },
        ];
        // Band 1: left coded (samples) — ms flag present; band 0: right
        // coded — ms flag present; band 2: both nonzero — present.
        let ms = vec![true, false, true];
        assert_stereo_round_trips(&left, &right, &ms, true, 8);
    }

    #[test]
    fn stream_ms_off_writes_no_flag_bits() {
        // Even with non-zero bands, stream_ms=false emits no M/S bits, so
        // the two channels' bodies pack tighter. Round-trip proves the
        // alignment holds without the flag bits.
        let coded = Sv7EncBand::Coded {
            band_type: 3,
            ctx: 0,
            scf: [9, 9, 9],
            levels: q3_levels(),
        };
        let chan = vec![coded];
        assert_stereo_round_trips(&chan, &chan, &[true], false, 0);
    }

    #[test]
    fn rejects_length_mismatch() {
        let mut w = Sv7BitWriter::new();
        assert!(matches!(
            encode_sv7_stereo_frame(
                &mut w,
                &[Sv7EncBand::Empty, Sv7EncBand::Empty],
                &[Sv7EncBand::Empty],
                &[false, false],
                false,
                0,
            ),
            Err(Error::MaxBandOutOfRange(_)),
        ));
    }

    #[test]
    fn rejects_oversized_frame() {
        let big = vec![Sv7EncBand::Empty; 33];
        let ms = vec![false; 33];
        let mut w = Sv7BitWriter::new();
        assert!(matches!(
            encode_sv7_stereo_frame(&mut w, &big, &big, &ms, false, 0),
            Err(Error::MaxBandOutOfRange(_)),
        ));
    }
}
