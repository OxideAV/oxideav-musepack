//! SV8 **two-channel frame-body encoder** — the exact inverse of
//! [`crate::sv8_stereo_frame::decode_sv8_stereo_frame`] (round 419).
//!
//! Re-emits a decoded [`Sv8StereoFrameDecode`] in the fixture-pinned
//! real-stream layout: `Max_used_Band` (keyframe log code / non-key
//! `Bands` delta), the §6.2 stereo resolution walk (top-down, L/R
//! interleaved, per-channel delta chains), the §6.2 M/S bitmap, then
//! the three band-major passes (SCFI, DSCF, samples). Every codeword
//! choice is the algebraic inverse of the decode — the only encoder
//! freedom the wire leaves is which of two `Bands` deltas names a
//! non-key `Max_used_Band` (the ring offers `d` and `d + 33`; the
//! in-alphabet one is taken) and the DSCF escape forms, which are
//! forced (escapes cover exactly the deltas the direct symbols cannot).
//!
//! Like the SV7 encode side (round 382) this is clean-room-invertible
//! composition: no new format facts, round-tripped bit-for-bit against
//! the decoder, and — corpus-proven (`tests/sv8_corpus_reencode.rs`) —
//! **wire-symmetric with the reference transcoder/encoder**: re-encoding
//! the decoded structure of every corpus `AP` packet reproduces the
//! original payload bytes exactly.
//!
//! Source-of-record: the same §6.2/§6.3/§6.4/§6.5 facts as the decode
//! modules; see [`crate::sv8_stereo_frame`].

use crate::requant::QUANTIZER_OFFSET_D;
use crate::scf::SCF_GRANULES_PER_BAND;
use crate::sv7_band_decode::SAMPLES_PER_BAND;
use crate::sv7_bitwriter::Sv7BitWriter;
use crate::sv8_band_decode::{sv8_band_type_case, Sv8BandDecodeCase};
use crate::sv8_context::Sv8Context;
use crate::sv8_dscf_loop::scfi_coded_granules;
use crate::sv8_huffman::{table_for_role, Sv8TableRole};
use crate::sv8_huffman_encode::{write_enum_subset, write_log_code, write_symbol};
use crate::sv8_sample_decode::SPARSE_GROUP_SIZE;
use crate::sv8_stereo_frame::{Sv8FrameState, Sv8StereoFrameDecode, SV8_BODY_CHANNELS};
use crate::{Error, Result};

/// §6.3 DSCF fold inverse: the delta `d` (in the 7-bit ring) that folds
/// `prev` onto `target` under `target = ((prev − 25 + d) & 127) − 6`.
fn dscf_delta_for(prev: i32, target: i32) -> i32 {
    (target + 6 - (prev - 25)).rem_euclid(128)
}

/// Encode one §6.3 later-granule delta (`dscf-1`): direct symbols carry
/// deltas `0..=63` except 31; symbol 31 escapes to `64 + raw6`
/// (deltas `64..=127`). A delta of exactly 31 has no codeword — it
/// cannot arise from a decoded stream.
fn write_dscf1_delta(writer: &mut Sv7BitWriter, delta: i32) -> Result<()> {
    let table = table_for_role(Sv8TableRole::Dscf, 0).ok_or(Error::SampleOutOfRange(delta))?;
    match delta {
        0..=63 if delta != 31 => write_symbol(writer, table, delta as i8),
        64..=127 => {
            write_symbol(writer, table, 31)?;
            writer.write_bits((delta - 64) as u32, 6);
            Ok(())
        }
        _ => Err(Error::SampleOutOfRange(delta)),
    }
}

/// Encode one §6.3 `SCF[0]` delta (`dscf-2`): direct symbols carry
/// deltas `0..=63`; symbol 64 escapes to `64 + raw6` (deltas
/// `64..=127`).
fn write_dscf2_delta(writer: &mut Sv7BitWriter, delta: i32) -> Result<()> {
    let table = table_for_role(Sv8TableRole::Dscf, 1).ok_or(Error::SampleOutOfRange(delta))?;
    match delta {
        0..=63 => write_symbol(writer, table, delta as i8),
        64..=127 => {
            write_symbol(writer, table, 64)?;
            writer.write_bits((delta - 64) as u32, 6);
            Ok(())
        }
        _ => Err(Error::SampleOutOfRange(delta)),
    }
}

/// Encode one channel's 36 sample levels for `band_type`, the inverse
/// of [`crate::sv8_band_decode::decode_sv8_band_grounded`]. CNS and
/// empty bands emit no bits.
fn write_band_samples(
    writer: &mut Sv7BitWriter,
    band_type: i8,
    levels: &[i32; SAMPLES_PER_BAND],
) -> Result<()> {
    match sv8_band_type_case(band_type) {
        Sv8BandDecodeCase::Cns | Sv8BandDecodeCase::Empty => Ok(()),
        Sv8BandDecodeCase::SparseBand => {
            let q1 = table_for_role(Sv8TableRole::Q1, 0).ok_or(Error::UnsupportedBandType(1))?;
            for half in levels.chunks_exact(SPARSE_GROUP_SIZE) {
                let n = SPARSE_GROUP_SIZE as u32;
                let mut mask: u32 = 0;
                let mut cnt: u32 = 0;
                for (pos, &s) in half.iter().enumerate() {
                    match s {
                        0 => {}
                        -1 | 1 => {
                            mask |= 1 << (SPARSE_GROUP_SIZE - 1 - pos);
                            cnt += 1;
                        }
                        other => return Err(Error::SampleOutOfRange(other)),
                    }
                }
                write_symbol(writer, q1, cnt as i8)?;
                if cnt > 0 && cnt < n {
                    let (coded, k) = if cnt > n / 2 {
                        ((!mask) & ((1 << n) - 1), n - cnt)
                    } else {
                        (mask, cnt)
                    };
                    write_enum_subset(writer, coded, k, n)?;
                }
                for &s in half.iter() {
                    if s != 0 {
                        writer.write_bits(u32::from(s > 0), 1);
                    }
                }
            }
            Ok(())
        }
        Sv8BandDecodeCase::Grouped3 => {
            let mut ctx = Sv8Context::new(2).ok_or(Error::UnsupportedBandType(2))?;
            for group in levels.chunks_exact(3) {
                let mut tmp: i32 = 0;
                for (digit, &s) in group.iter().enumerate() {
                    if !(-2..=2).contains(&s) {
                        return Err(Error::SampleOutOfRange(s));
                    }
                    tmp += (s + 2) * 5_i32.pow(digit as u32);
                }
                let table = table_for_role(Sv8TableRole::Q2, ctx.table_ctx())
                    .ok_or(Error::UnsupportedBandType(2))?;
                write_symbol(writer, table, tmp as i8)?;
                ctx.update_group(tmp);
            }
            Ok(())
        }
        Sv8BandDecodeCase::Grouped2 => {
            let role = if band_type == 3 {
                Sv8TableRole::Q3
            } else {
                Sv8TableRole::Q4
            };
            let table = table_for_role(role, 0).ok_or(Error::UnsupportedBandType(band_type))?;
            for pair in levels.chunks_exact(2) {
                for &s in pair {
                    if s.abs() > i32::from(band_type) {
                        return Err(Error::SampleOutOfRange(s));
                    }
                }
                // Low nibble = first sample, high nibble = second.
                let symbol = (((pair[1] as u8) & 0xF) << 4) | ((pair[0] as u8) & 0xF);
                write_symbol(writer, table, symbol as i8)?;
            }
            Ok(())
        }
        Sv8BandDecodeCase::ContextHuffmanPerSample => {
            let role = match band_type {
                5 => Sv8TableRole::Q5,
                6 => Sv8TableRole::Q6,
                7 => Sv8TableRole::Q7,
                _ => Sv8TableRole::Q8,
            };
            let mut ctx =
                Sv8Context::new(band_type).ok_or(Error::UnsupportedBandType(band_type))?;
            for &s in levels.iter() {
                let q = i8::try_from(s).map_err(|_| Error::SampleOutOfRange(s))?;
                let table = table_for_role(role, ctx.table_ctx())
                    .ok_or(Error::UnsupportedBandType(band_type))?;
                write_symbol(writer, table, q)?;
                ctx.update_sample(q);
            }
            Ok(())
        }
        Sv8BandDecodeCase::LargeCoeffEscape => {
            let raw_bits = u32::from(band_type as u8 - 9);
            let d = i32::from(QUANTIZER_OFFSET_D[(band_type + 1) as usize]);
            let q9up = table_for_role(Sv8TableRole::Q9up, 0)
                .ok_or(Error::UnsupportedBandType(band_type))?;
            for &s in levels.iter() {
                let v = s + d;
                if !(0..=(255 << raw_bits) | ((1 << raw_bits) - 1)).contains(&v) {
                    return Err(Error::SampleOutOfRange(s));
                }
                let symbol = (v >> raw_bits) as u8 as i8;
                write_symbol(writer, q9up, symbol)?;
                if raw_bits > 0 {
                    writer.write_bits((v as u32) & ((1 << raw_bits) - 1), raw_bits as u8);
                }
            }
            Ok(())
        }
        Sv8BandDecodeCase::OutOfRange => Err(Error::UnsupportedBandType(band_type)),
    }
}

/// Encode one SV8 two-channel frame body: the exact inverse of
/// [`crate::sv8_stereo_frame::decode_sv8_stereo_frame`].
///
/// `state` must carry the same cross-frame posture the decoder's would
/// at this frame (reset it at every `AP` boundary; the key frame is the
/// packet's first). The frame's `scfi` selectors are re-emitted
/// verbatim, so a decode → encode round trip reproduces the original
/// bits exactly.
///
/// # Errors
///
/// - [`Error::MaxBandOutOfRange`] if `frame.nbands` exceeds
///   `max_band + 1` or the structured vectors are shorter than
///   `nbands`.
/// - [`Error::SymbolNotEncodable`] / [`Error::SampleOutOfRange`] for a
///   value outside its arm's coded alphabet.
/// - [`Error::InvalidScfCodingMethod`] for an SCFI selector above 3.
pub fn encode_sv8_stereo_frame(
    writer: &mut Sv7BitWriter,
    frame: &Sv8StereoFrameDecode,
    max_band: u8,
    keyframe: bool,
    stream_ms: bool,
    state: &mut Sv8FrameState,
) -> Result<()> {
    let nb = frame.nbands as usize;
    if frame.nbands > max_band + 1
        || frame.res.len() < nb
        || frame.ms_flags.len() < nb
        || frame.scfi.len() < nb
        || frame.granule_scf.len() < nb
        || frame.levels.len() < nb
    {
        return Err(Error::MaxBandOutOfRange(frame.nbands));
    }

    // 1. §6.2 Max_used_Band.
    if keyframe {
        write_log_code(writer, u32::from(frame.nbands), u32::from(max_band) + 2)?;
    } else {
        let bands =
            table_for_role(Sv8TableRole::Bands, 0).ok_or(Error::MaxBandOutOfRange(frame.nbands))?;
        // The ring offers d and d + 33; take the in-alphabet form.
        let d = i32::from(frame.nbands) - i32::from(state.last_nbands());
        let sym = if bands.symbols.contains(&(d as i8)) {
            d as i8
        } else {
            i8::try_from(d + 33).map_err(|_| Error::SampleOutOfRange(d))?
        };
        write_symbol(writer, bands, sym)?;
    }
    state.note_nbands(frame.nbands);

    // 2. §6.2 stereo resolution walk (top-down, L/R interleaved).
    let mut above: [Option<i8>; 2] = [None, None];
    for b in (0..nb).rev() {
        for (ch, above_ch) in above.iter_mut().enumerate().take(SV8_BODY_CHANNELS) {
            let bt = frame.res[b][ch];
            let (ctx, raw) = match *above_ch {
                None => (
                    0,
                    if bt < 0 {
                        i32::from(bt) + 17
                    } else {
                        i32::from(bt)
                    },
                ),
                Some(a) => {
                    let ctx = u8::from(a > 2);
                    (ctx, (i32::from(bt) - i32::from(a)).rem_euclid(17))
                }
            };
            let table =
                table_for_role(Sv8TableRole::Res, ctx).ok_or(Error::UnsupportedBandType(bt))?;
            write_symbol(writer, table, raw as i8)?;
            *above_ch = Some(bt);
        }
    }

    // 3. §6.2 M/S bitmap over the bands with a non-zero channel.
    if stream_ms {
        let scope: Vec<usize> = (0..nb)
            .filter(|&b| frame.res[b][0] != 0 || frame.res[b][1] != 0)
            .collect();
        let n = scope.len() as u32;
        let mut mask: u32 = 0;
        for (k, &b) in scope.iter().enumerate() {
            if frame.ms_flags[b] {
                mask |= 1 << (n as usize - 1 - k);
            }
        }
        let cnt = mask.count_ones();
        write_log_code(writer, cnt, n + 1)?;
        if cnt > 0 && cnt < n {
            let (coded, k) = if cnt > n / 2 {
                ((!mask) & ((1 << n) - 1), n - cnt)
            } else {
                (mask, cnt)
            };
            write_enum_subset(writer, coded, k, n)?;
        }
    }

    // 4. SCFI pass (ascending; CNS bands participate).
    for b in 0..nb {
        let chs: Vec<usize> = (0..SV8_BODY_CHANNELS)
            .filter(|&ch| frame.res[b][ch] != 0)
            .collect();
        if chs.is_empty() {
            continue;
        }
        for &ch in &chs {
            if frame.scfi[b][ch] > 3 {
                return Err(Error::InvalidScfCodingMethod(frame.scfi[b][ch] as i8));
            }
        }
        if chs.len() == 2 {
            let table =
                table_for_role(Sv8TableRole::Scfi, 1).ok_or(Error::InvalidScfCodingMethod(-1))?;
            let value = (frame.scfi[b][0] << 2) | frame.scfi[b][1];
            write_symbol(writer, table, value as i8)?;
        } else {
            let table =
                table_for_role(Sv8TableRole::Scfi, 0).ok_or(Error::InvalidScfCodingMethod(-1))?;
            write_symbol(writer, table, frame.scfi[b][chs[0]] as i8)?;
        }
    }

    // 5. DSCF pass (ascending, left then right; temporal memory).
    for b in 0..nb {
        for ch in 0..SV8_BODY_CHANNELS {
            if frame.res[b][ch] == 0 {
                continue;
            }
            let scf = frame.granule_scf[b][ch];
            let coded = scfi_coded_granules(frame.scfi[b][ch])?;
            let new_block = keyframe || state.scf_ref(ch, b).is_none();
            if new_block {
                let raw7 = scf[0] + 6;
                if !(0..=127).contains(&raw7) {
                    return Err(Error::SampleOutOfRange(scf[0]));
                }
                writer.write_bits(raw7 as u32, 7);
            } else {
                let prev = state.scf_ref(ch, b).unwrap_or(0);
                write_dscf2_delta(writer, dscf_delta_for(prev, scf[0]))?;
            }
            if coded[1] {
                write_dscf1_delta(writer, dscf_delta_for(scf[0], scf[1]))?;
            } else if scf[1] != scf[0] {
                return Err(Error::SampleOutOfRange(scf[1]));
            }
            if coded[2] {
                write_dscf1_delta(writer, dscf_delta_for(scf[1], scf[2]))?;
            } else if scf[2] != scf[1] {
                return Err(Error::SampleOutOfRange(scf[2]));
            }
            state.note_scf2(ch, b, scf[SCF_GRANULES_PER_BAND - 1]);
        }
    }

    // 6. Sample pass (ascending, left then right).
    for b in 0..nb {
        for ch in 0..SV8_BODY_CHANNELS {
            if frame.res[b][ch] != 0 {
                write_band_samples(writer, frame.res[b][ch], &frame.levels[b][ch])?;
            }
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cns::CnsPrng;
    use crate::huffman::Sv7BitReader;
    use crate::sv8_stereo_frame::decode_sv8_stereo_frame;

    /// Decode a frame from `bytes`, re-encode it, and assert the
    /// emitted bits equal the consumed prefix exactly.
    fn round_trip(bytes: &[u8], max_band: u8, keyframe: bool, stream_ms: bool) {
        let mut padded = bytes.to_vec();
        padded.extend_from_slice(&[0, 0]);
        let mut reader = Sv7BitReader::new(&padded);
        let total = reader.bits_remaining();
        let mut dstate = Sv8FrameState::new();
        let mut cns = CnsPrng::new();
        let frame = decode_sv8_stereo_frame(
            &mut reader,
            max_band,
            keyframe,
            stream_ms,
            &mut dstate,
            &mut cns,
        )
        .expect("decode");
        let consumed = total - reader.bits_remaining();

        let mut estate = Sv8FrameState::new();
        let mut w = Sv7BitWriter::new();
        encode_sv8_stereo_frame(&mut w, &frame, max_band, keyframe, stream_ms, &mut estate)
            .expect("encode");
        assert_eq!(w.bit_len(), consumed, "bit count");
        assert_eq!(estate, dstate, "cross-frame state agrees");
        let emitted = w.finish();
        // Compare the consumed prefix bit-for-bit.
        let full_bytes = (consumed / 8) as usize;
        assert_eq!(&emitted[..full_bytes], &bytes[..full_bytes]);
        let tail_bits = (consumed % 8) as u8;
        if tail_bits > 0 {
            let m = 0xFFu8 << (8 - tail_bits);
            assert_eq!(emitted[full_bytes] & m, bytes[full_bytes] & m);
        }
    }

    #[test]
    fn corpus_keyframes_round_trip_bit_exactly() {
        // The first frame of every fixture AP payload is a key frame;
        // decode + re-encode it and require identical bits. This runs
        // the whole encoder over real reference-encoder output (full
        // multi-frame packets are exercised in
        // tests/sv8_corpus_reencode.rs; unit scope here is one frame).
        use crate::packet_stream::{PacketSizeConvention, PacketStream};
        use crate::sh_header::StreamHeaderFields;

        for name in [
            "stereo-sine-partial-last-frame",
            "cns-pns",
            "mono-sine-standard",
        ] {
            let path = format!(
                "{}/tests/fixtures/sv8/{name}/input.mpc",
                env!("CARGO_MANIFEST_DIR")
            );
            let bytes = std::fs::read(&path).expect("fixture");
            let mut stream = PacketStream::new(&bytes[4..], PacketSizeConvention::Inclusive);
            let mut sh: Option<StreamHeaderFields> = None;
            while let Some(p) = stream.next_packet().unwrap() {
                match &p.key {
                    k if format!("{k:?}") == "StreamHeader" => {
                        sh = Some(StreamHeaderFields::parse(p.payload).unwrap());
                    }
                    k if format!("{k:?}") == "AudioPacket" => {
                        let sh = sh.as_ref().expect("SH before AP");
                        round_trip(p.payload, sh.max_band, true, sh.mid_side);
                        break;
                    }
                    _ => {}
                }
            }
        }
    }

    #[test]
    fn rejects_undersized_structure() {
        let frame = Sv8StereoFrameDecode {
            nbands: 2,
            res: vec![[0, 0]], // shorter than nbands
            ms_flags: vec![false],
            scfi: vec![[0, 0]],
            granule_scf: vec![[[0; 3]; 2]],
            levels: vec![[[0; SAMPLES_PER_BAND]; 2]],
        };
        let mut w = Sv7BitWriter::new();
        let mut st = Sv8FrameState::new();
        assert!(matches!(
            encode_sv8_stereo_frame(&mut w, &frame, 4, true, true, &mut st),
            Err(Error::MaxBandOutOfRange(2))
        ));
    }

    #[test]
    fn dscf_delta_inverse_matches_fold_over_the_ring() {
        // fold(prev, dscf_delta_for(prev, target)) == target over the
        // full signed index range.
        let fold = |p: i32, d: i32| ((p - 25 + d) & 127) - 6;
        for prev in -6..=121 {
            for target in -6..=121 {
                let d = dscf_delta_for(prev, target);
                assert!((0..=127).contains(&d));
                assert_eq!(fold(prev, d), target, "prev {prev} target {target}");
            }
        }
    }
}
