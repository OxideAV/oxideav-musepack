//! SV7 stereo frame-body **encode** — inverse of
//! [`crate::sv7_stereo_frame::decode_sv7_stereo_frame`]'s corpus-pinned
//! four-pass bitstream layout:
//!
//! 1. **§5.1 band-type header** — both channels interleaved per band
//!    (left `Res` then right `Res`) plus the per-band M/S bit;
//! 2. **SCFI pass** — for each band, for each channel with `Res ≠ 0`
//!    (coded *and* CNS): the SCFI selector VLC;
//! 3. **DSCF pass** — same order: each band's coded DSCF indices,
//!    `SCF[0]` delta'd off the per-band cross-frame memory
//!    ([`crate::sv7_stereo_frame::Sv7ScfMemory`]);
//! 4. **samples pass** — same order: each coded band's context selector
//!    + 36 levels (CNS and silent bands write nothing).
//!
//! The M/S flags are written verbatim into the §5.1 header — whether a
//! band is coded mid/side is the encoder's (caller's) decision; the
//! pinned `L = M + S` / `R = M − S` undo is a decode-side concern.
//!
//! Source-of-record: `docs/audio/musepack/spec/musepack-headers-and-coding.md`
//! §5.1–§5.5; pass layout + SCF[0] reference fixture-corpus-pinned (see
//! [`crate::sv7_stereo_frame`]). Round-tripped against the decoder in
//! the tests below.

use crate::sv7_band_header::{Sv7ResBand, SV7_MAX_BAND_INCLUSIVE};
use crate::sv7_bitwriter::Sv7BitWriter;
use crate::sv7_frame_encode::{encode_sv7_band_samples, Sv7EncBand};
use crate::sv7_scf_encode::{choose_scfi, encode_sv7_band_dscf, encode_sv7_scfi};
use crate::sv7_stereo_frame::Sv7ScfMemory;
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
/// of [`crate::sv7_stereo_frame::decode_sv7_stereo_frame`].
///
/// `left` / `right` are the two channels' band specs (ascending subband
/// order, equal length ≤ 32). `ms_flags` is the per-band M/S flag (only
/// emitted for bands with a non-zero channel, when `stream_ms` is set).
/// `scf` is the cross-frame per-band SCF memory — one per stream,
/// zero-initialised, advanced by every non-silent band exactly as the
/// decoder's is.
///
/// # Errors
///
/// - [`Error::MaxBandOutOfRange`] if the channel/flag lengths disagree or
///   exceed `SV7_MAX_BAND_INCLUSIVE + 1`.
/// - [`Error::SampleOutOfRange`] / [`Error::SymbolNotEncodable`] /
///   [`Error::UnsupportedBandType`] from a header, SCF, or sample write.
pub fn encode_sv7_stereo_frame(
    writer: &mut Sv7BitWriter,
    left: &[Sv7EncBand],
    right: &[Sv7EncBand],
    ms_flags: &[bool],
    stream_ms: bool,
    scf: &mut Sv7ScfMemory,
) -> Result<()> {
    if left.len() > SV7_MAX_BAND_INCLUSIVE as usize + 1 {
        return Err(Error::MaxBandOutOfRange(left.len() as u8));
    }
    let res_bands = build_res_bands(left, right, ms_flags)?;
    // Pass 1 — §5.1 shared band-type header (both channels + M/S).
    crate::sv7_band_header_encode::encode_res_header_grounded(writer, &res_bands, 2, stream_ms)?;

    let chan = |ch: usize, b: usize| -> &Sv7EncBand {
        if ch == 0 {
            &left[b]
        } else {
            &right[b]
        }
    };

    // Pass 2 — SCFI selectors, band-major / channel-minor.
    for b in 0..left.len() {
        for ch in 0..2 {
            if let Some(indices) = chan(ch, b).scf() {
                encode_sv7_scfi(writer, choose_scfi(indices))?;
            }
        }
    }

    // Pass 3 — DSCF chains off the per-band cross-frame memory.
    for b in 0..left.len() {
        for ch in 0..2 {
            if let Some(indices) = chan(ch, b).scf() {
                let scfi = choose_scfi(indices);
                encode_sv7_band_dscf(writer, scfi, indices, scf.reference(ch, b))?;
                scf.update(ch, b, indices[2]);
            }
        }
    }

    // Pass 4 — samples.
    for b in 0..left.len() {
        for ch in 0..2 {
            encode_sv7_band_samples(writer, chan(ch, b))?;
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cns::CnsPrng;
    use crate::huffman::{sv7_q3_ctx, Sv7BitReader};
    use crate::reconstruct::{reconstruct_sv7_band_absolute, sv7_absolute_scf_gain};
    use crate::requant::DEQUANT_COEFFICIENT_C;
    use crate::sv7_band_decode::SAMPLES_PER_BAND;
    use crate::sv7_stereo_frame::decode_sv7_stereo_frame;

    fn q3_levels() -> [i32; SAMPLES_PER_BAND] {
        let mut a: Vec<i32> = sv7_q3_ctx(0).iter().map(|e| e.value as i32).collect();
        a.dedup();
        core::array::from_fn(|i| a[i % a.len()])
    }

    fn coded(band_type: i8, ctx: usize, scf: [i32; 3]) -> Sv7EncBand {
        Sv7EncBand::Coded {
            band_type,
            ctx,
            scf,
            levels: q3_levels(),
        }
    }

    /// Encode a stereo frame, decode it back through the stereo frame
    /// decoder, and check the M/S flags and each coded band's absolute
    /// reconstruction (which pins band_type + SCF + levels together).
    fn assert_stereo_round_trips(
        left: &[Sv7EncBand],
        right: &[Sv7EncBand],
        ms_flags: &[bool],
        stream_ms: bool,
    ) {
        let mut w = Sv7BitWriter::new();
        let mut enc_scf = Sv7ScfMemory::new();
        encode_sv7_stereo_frame(&mut w, left, right, ms_flags, stream_ms, &mut enc_scf)
            .expect("encode");
        let mut bytes = w.finish();
        bytes.extend_from_slice(&[0, 0, 0, 0]);

        let max_band = (left.len() - 1) as u8;
        let mut r = Sv7BitReader::new(&bytes);
        let mut cns = CnsPrng::new();
        let mut dec_scf = Sv7ScfMemory::new();
        let frame = decode_sv7_stereo_frame(&mut r, max_band, stream_ms, &mut dec_scf, &mut cns)
            .expect("decode");

        // The encoder's and decoder's SCF memories end identical.
        assert_eq!(enc_scf, dec_scf, "SCF memory divergence");

        // Expected M/S flags: present only where a band has samples.
        for (b, &ms) in ms_flags.iter().enumerate() {
            let has = left[b].res() != 0 || right[b].res() != 0;
            let want = stream_ms && has && ms;
            assert_eq!(frame.ms_flags[b], want, "band {b} ms");
        }

        // Per-band reconstruction equality against a direct rebuild.
        let mut ref_cns = CnsPrng::new();
        for b in 0..left.len() {
            for (ch, spec) in [&left[b], &right[b]].into_iter().enumerate() {
                match spec {
                    Sv7EncBand::Empty => {
                        assert!(
                            frame.channels[ch][b].iter().all(|&s| s == 0.0),
                            "band {b} ch {ch} silent"
                        );
                    }
                    Sv7EncBand::Cns { scf } => {
                        let mut noise = [0i32; SAMPLES_PER_BAND];
                        ref_cns.fill_samples(&mut noise);
                        let mut want = [0.0; SAMPLES_PER_BAND];
                        reconstruct_sv7_band_absolute(-1, &noise, *scf, &mut want).unwrap();
                        assert_eq!(frame.channels[ch][b], want, "band {b} ch {ch} cns");
                    }
                    Sv7EncBand::Coded {
                        band_type,
                        scf,
                        levels,
                        ..
                    } => {
                        let mut want = [0.0; SAMPLES_PER_BAND];
                        reconstruct_sv7_band_absolute(*band_type, levels, *scf, &mut want).unwrap();
                        assert_eq!(frame.channels[ch][b], want, "band {b} ch {ch} coded");
                    }
                }
            }
        }
    }

    #[test]
    fn all_silent_stereo_frame_round_trips() {
        assert_stereo_round_trips(&[Sv7EncBand::Empty], &[Sv7EncBand::Empty], &[false], false);
    }

    #[test]
    fn coded_both_channels_with_ms_flag_round_trips() {
        assert_stereo_round_trips(
            &[coded(3, 0, [7, 7, 7])],
            &[coded(3, 1, [9, 10, 9])],
            &[true],
            true,
        );
    }

    #[test]
    fn cns_bands_carry_scf_and_thread_the_prng() {
        // Both channels: band0 empty, band1 CNS with distinct SCF
        // triples — the SCF layer is written/read for CNS and the PRNG
        // advances left-then-right within the band.
        let left = vec![Sv7EncBand::Empty, Sv7EncBand::Cns { scf: [5, 5, 5] }];
        let right = vec![Sv7EncBand::Empty, Sv7EncBand::Cns { scf: [11, 12, 12] }];
        assert_stereo_round_trips(&left, &right, &[false, false], false);
    }

    #[test]
    fn cns_scf_gain_reaches_the_noise() {
        // A CNS band's SCF triple scales the PRNG noise. (CNS lives at
        // band 1 — the §5.1 band-0 raw 4-bit absolute cannot carry the
        // value −1, only the delta chain reaches it.)
        let mut w = Sv7BitWriter::new();
        let mut enc_scf = Sv7ScfMemory::new();
        let left = vec![Sv7EncBand::Empty, Sv7EncBand::Cns { scf: [1, 1, 1] }];
        let right = vec![Sv7EncBand::Empty, Sv7EncBand::Cns { scf: [20, 20, 20] }];
        encode_sv7_stereo_frame(&mut w, &left, &right, &[false, false], false, &mut enc_scf)
            .unwrap();
        let mut bytes = w.finish();
        bytes.extend_from_slice(&[0, 0, 0, 0]);
        let mut r = Sv7BitReader::new(&bytes);
        let mut cns = CnsPrng::new();
        let mut dec_scf = Sv7ScfMemory::new();
        let frame = decode_sv7_stereo_frame(&mut r, 1, false, &mut dec_scf, &mut cns).unwrap();
        // Rebuild the raw noise to compare gains sample-for-sample.
        let mut ref_cns = CnsPrng::new();
        let mut l_noise = [0i32; SAMPLES_PER_BAND];
        ref_cns.fill_samples(&mut l_noise);
        let c0 = DEQUANT_COEFFICIENT_C[0];
        for (k, &n) in l_noise.iter().enumerate() {
            let want = n as f64 * c0 * sv7_absolute_scf_gain(1);
            assert!((frame.channels[0][1][k] - want).abs() < 1e-9, "{k}");
        }
    }

    #[test]
    fn mixed_asymmetric_channels_round_trip() {
        let g3: [i32; SAMPLES_PER_BAND] = core::array::from_fn(|i| (i as i32 % 3) - 1);
        let left = vec![
            Sv7EncBand::Empty,
            coded(3, 1, [10, 10, 10]),
            Sv7EncBand::Cns { scf: [30, 31, 32] },
        ];
        let right = vec![
            Sv7EncBand::Coded {
                band_type: 1,
                ctx: 0,
                scf: [20, 21, 22],
                levels: g3,
            },
            Sv7EncBand::Empty,
            coded(3, 0, [15, 15, 15]),
        ];
        assert_stereo_round_trips(&left, &right, &[true, false, true], true);
    }

    #[test]
    fn stream_ms_off_writes_no_flag_bits() {
        let chan = vec![coded(3, 0, [9, 9, 9])];
        assert_stereo_round_trips(&chan, &chan, &[true], false);
    }

    #[test]
    fn scf_memory_threads_across_encoded_frames() {
        // Two frames of the same coded band: frame 2's SCF[0] delta must
        // ride on frame 1's SCF[2] via the shared memory (a fresh
        // decoder memory reproduces both).
        let mut w = Sv7BitWriter::new();
        let mut enc_scf = Sv7ScfMemory::new();
        let f1 = vec![coded(3, 0, [10, 10, 10])];
        let f2 = vec![coded(3, 0, [12, 12, 12])];
        encode_sv7_stereo_frame(&mut w, &f1, &f1, &[false], false, &mut enc_scf).unwrap();
        encode_sv7_stereo_frame(&mut w, &f2, &f2, &[false], false, &mut enc_scf).unwrap();
        let mut bytes = w.finish();
        bytes.extend_from_slice(&[0, 0, 0, 0]);

        let mut r = Sv7BitReader::new(&bytes);
        let mut cns = CnsPrng::new();
        let mut dec_scf = Sv7ScfMemory::new();
        decode_sv7_stereo_frame(&mut r, 0, false, &mut dec_scf, &mut cns).unwrap();
        assert_eq!(dec_scf.reference(0, 0), 10);
        decode_sv7_stereo_frame(&mut r, 0, false, &mut dec_scf, &mut cns).unwrap();
        assert_eq!(dec_scf.reference(0, 0), 12);
        assert_eq!(enc_scf, dec_scf);
    }

    #[test]
    fn rejects_length_mismatch() {
        let mut w = Sv7BitWriter::new();
        let mut scf = Sv7ScfMemory::new();
        assert!(matches!(
            encode_sv7_stereo_frame(
                &mut w,
                &[Sv7EncBand::Empty, Sv7EncBand::Empty],
                &[Sv7EncBand::Empty],
                &[false, false],
                false,
                &mut scf,
            ),
            Err(Error::MaxBandOutOfRange(_)),
        ));
    }

    #[test]
    fn rejects_oversized_frame() {
        let big = vec![Sv7EncBand::Empty; 33];
        let ms = vec![false; 33];
        let mut w = Sv7BitWriter::new();
        let mut scf = Sv7ScfMemory::new();
        assert!(matches!(
            encode_sv7_stereo_frame(&mut w, &big, &big, &ms, false, &mut scf),
            Err(Error::MaxBandOutOfRange(_)),
        ));
    }
}
