//! End-to-end gates for the from-PCM SV8 encoder (round 429):
//! encode → own-decode SNR against the input, gapless exactness,
//! re-parse wire symmetry, and frame-budget checks.

use oxideav_musepack::cns::CnsPrng;
use oxideav_musepack::huffman::Sv7BitReader;
use oxideav_musepack::packet_stream::{PacketSizeConvention, PacketStream};
use oxideav_musepack::sv7_bitwriter::Sv7BitWriter;
use oxideav_musepack::sv8_decode::decode_sv8_stream;
use oxideav_musepack::sv8_file_encode::{
    encode_sv8_from_pcm_f64, encode_sv8_from_pcm_s16, Sv8EncoderSettings,
};
use oxideav_musepack::sv8_stereo_frame::{decode_sv8_stereo_frame, Sv8FrameState};
use oxideav_musepack::sv8_stereo_frame_encode::encode_sv8_stereo_frame;
use oxideav_musepack::typed_packet::TypedPacket;
use std::f64::consts::PI;

/// Per-channel SNR (dB) of `decoded` against `input` (both interleaved
/// with `nch` channels, equal length).
fn snr_db_per_channel(input: &[f64], decoded: &[f64], nch: usize) -> Vec<f64> {
    assert_eq!(input.len(), decoded.len());
    let mut sig = vec![0.0_f64; nch];
    let mut err = vec![0.0_f64; nch];
    for (i, (&x, &y)) in input.iter().zip(decoded.iter()).enumerate() {
        let ch = i % nch;
        sig[ch] += x * x;
        err[ch] += (y - x) * (y - x);
    }
    sig.iter()
        .zip(err.iter())
        .map(|(&s, &e)| {
            if e == 0.0 {
                f64::INFINITY
            } else {
                10.0 * (s / e).log10()
            }
        })
        .collect()
}

/// A deterministic stereo multi-tone test signal in the s16 domain.
fn stereo_multitone(n: usize) -> Vec<f64> {
    let sr = 44_100.0;
    let mut pcm = Vec::with_capacity(2 * n);
    for i in 0..n {
        let t = i as f64 / sr;
        let l = 18_000.0 * (2.0 * PI * 440.0 * t).sin() + 6_000.0 * (2.0 * PI * 2_970.0 * t).sin();
        let r = 15_000.0 * (2.0 * PI * 660.0 * t).sin() + 7_500.0 * (2.0 * PI * 5_512.5 * t).sin();
        pcm.push(l);
        pcm.push(r);
    }
    pcm
}

#[test]
fn stereo_multitone_encode_decode_snr() {
    let n = 30_000usize; // deliberately not a multiple of 1152
    let pcm = stereo_multitone(n);
    let enc = encode_sv8_from_pcm_f64(&pcm, 2, 0, &Sv8EncoderSettings::default()).unwrap();
    let out = decode_sv8_stream(&enc.bytes).unwrap();
    assert_eq!(out.pcm.len(), pcm.len(), "gapless: exact sample count");

    let snr = snr_db_per_channel(&pcm, &out.pcm, 2);
    let rate_kbps = (enc.bytes.len() as f64 * 8.0) / (n as f64 / 44_100.0) / 1000.0;
    eprintln!(
        "stereo multitone: SNR L {:.1} dB, R {:.1} dB, {rate_kbps:.0} kbps",
        snr[0], snr[1]
    );
    // Measured ~81 / ~85 dB at ~173 kbps with the default settings;
    // gate with headroom.
    assert!(snr[0] > 75.0, "L SNR {:.1} dB", snr[0]);
    assert!(snr[1] > 75.0, "R SNR {:.1} dB", snr[1]);
    assert!(rate_kbps < 400.0, "rate {rate_kbps:.0} kbps");
}

#[test]
fn mono_sine_encode_decode_snr() {
    let n = 22_050usize;
    let sr = 44_100.0;
    let pcm: Vec<f64> = (0..n)
        .map(|i| 20_000.0 * (2.0 * PI * 525.0 * (i as f64) / sr).sin())
        .collect();
    let enc = encode_sv8_from_pcm_f64(&pcm, 1, 0, &Sv8EncoderSettings::default()).unwrap();
    let out = decode_sv8_stream(&enc.bytes).unwrap();
    assert_eq!(out.pcm.len(), n);
    let snr = snr_db_per_channel(&pcm, &out.pcm, 1);
    let rate_kbps = (enc.bytes.len() as f64 * 8.0) / (n as f64 / 44_100.0) / 1000.0;
    eprintln!("mono sine: SNR {:.1} dB, {rate_kbps:.0} kbps", snr[0]);
    // Measured ~82 dB at ~61 kbps; gate with headroom.
    assert!(snr[0] > 75.0, "SNR {:.1} dB", snr[0]);
    assert!(rate_kbps < 150.0, "rate {rate_kbps:.0} kbps");
}

#[test]
fn white_noise_worst_case_snr() {
    // Full-band noise codes every subband — the allocator's worst
    // case.
    let mut state = 0x1357_9BDF_2468_ACE0_u64;
    let mut next = move || {
        state ^= state >> 12;
        state ^= state << 25;
        state ^= state >> 27;
        ((state.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 11) as f64) / ((1u64 << 53) as f64) * 2.0
            - 1.0
    };
    let n = 23_040usize;
    let pcm: Vec<f64> = (0..2 * n).map(|_| next() * 10_000.0).collect();
    let enc = encode_sv8_from_pcm_f64(&pcm, 2, 0, &Sv8EncoderSettings::default()).unwrap();
    let out = decode_sv8_stream(&enc.bytes).unwrap();
    let snr = snr_db_per_channel(&pcm, &out.pcm, 2);
    let rate_kbps = (enc.bytes.len() as f64 * 8.0) / (n as f64 / 44_100.0) / 1000.0;
    eprintln!(
        "white noise: SNR L {:.1} dB, R {:.1} dB, {rate_kbps:.0} kbps",
        snr[0], snr[1]
    );
    // Measured ~65 dB (every subband coded, the aliasing-cancellation
    // ripple of the filterbank pair caps the reachable SNR on
    // full-band noise); gate with headroom.
    assert!(snr[0] > 55.0, "L SNR {:.1} dB", snr[0]);
    assert!(snr[1] > 55.0, "R SNR {:.1} dB", snr[1]);
}

#[test]
fn quality_knob_trades_rate_for_snr() {
    let n = 12_000usize;
    let pcm = stereo_multitone(n);
    let coarse = Sv8EncoderSettings {
        step_target: 16.0,
        ..Default::default()
    };
    let fine = Sv8EncoderSettings {
        step_target: 0.5,
        ..Default::default()
    };
    let enc_coarse = encode_sv8_from_pcm_f64(&pcm, 2, 0, &coarse).unwrap();
    let enc_fine = encode_sv8_from_pcm_f64(&pcm, 2, 0, &fine).unwrap();
    let snr_coarse =
        snr_db_per_channel(&pcm, &decode_sv8_stream(&enc_coarse.bytes).unwrap().pcm, 2);
    let snr_fine = snr_db_per_channel(&pcm, &decode_sv8_stream(&enc_fine.bytes).unwrap().pcm, 2);
    eprintln!(
        "coarse: {} bytes, SNR {:.1}; fine: {} bytes, SNR {:.1}",
        enc_coarse.bytes.len(),
        snr_coarse[0],
        enc_fine.bytes.len(),
        snr_fine[0]
    );
    assert!(enc_fine.bytes.len() > enc_coarse.bytes.len());
    // Measured: coarse ~62 dB, fine ~83 dB.
    assert!(snr_fine[0] > snr_coarse[0] + 10.0);
    assert!(snr_coarse[0] > 50.0);
}

/// Wire symmetry of our own streams: decode the structure of every AP
/// payload and re-encode it — byte-identical, and the consumed bits
/// match the re-emitted bits (frame budget exactness).
#[test]
fn own_stream_reparse_is_byte_exact() {
    let n = 20_000usize;
    let pcm = stereo_multitone(n);
    let enc = encode_sv8_from_pcm_f64(&pcm, 2, 0, &Sv8EncoderSettings::default()).unwrap();

    let mut stream = PacketStream::new(&enc.bytes[4..], PacketSizeConvention::Inclusive);
    let mut sh = None;
    let mut frames_remaining = enc.frames;
    let mut aps = 0u64;
    while let Some(p) = stream.next_packet().unwrap() {
        match TypedPacket::classify(p) {
            TypedPacket::StreamHeader(h) => sh = Some(h.fields().unwrap()),
            TypedPacket::Audio(ap) => {
                let sh = sh.as_ref().expect("SH before AP");
                let frames = frames_remaining.min(sh.frames_per_audio_packet());
                let payload = ap.payload_bytes();

                // Decode the packet's frames structurally.
                let mut padded = payload.to_vec();
                padded.extend_from_slice(&[0, 0]);
                let mut reader = Sv7BitReader::new(&padded);
                let total_bits = reader.bits_remaining();
                let mut dstate = Sv8FrameState::new();
                let mut cns = CnsPrng::new();
                let mut decoded = Vec::new();
                for f in 0..frames {
                    decoded.push(
                        decode_sv8_stereo_frame(
                            &mut reader,
                            sh.max_band,
                            f == 0,
                            sh.mid_side,
                            &mut dstate,
                            &mut cns,
                        )
                        .unwrap(),
                    );
                }
                let consumed = total_bits - reader.bits_remaining();
                assert!(
                    consumed.div_ceil(8) as usize == payload.len(),
                    "AP {aps}: payload must be exactly the consumed bits, byte-padded \
                     (consumed {consumed} bits, payload {} bytes)",
                    payload.len()
                );

                // Re-encode and compare bytes.
                let mut w = Sv7BitWriter::new();
                let mut estate = Sv8FrameState::new();
                for (f, frame) in decoded.iter().enumerate() {
                    encode_sv8_stereo_frame(
                        &mut w,
                        frame,
                        sh.max_band,
                        f == 0,
                        sh.mid_side,
                        &mut estate,
                    )
                    .unwrap();
                }
                assert_eq!(w.bit_len(), consumed, "AP {aps}: bit budget");
                assert_eq!(w.finish(), payload, "AP {aps}: byte-exact re-encode");

                frames_remaining -= frames;
                aps += 1;
            }
            TypedPacket::StreamEnd(_) => break,
            _ => {}
        }
    }
    assert_eq!(aps, enc.audio_packets);
    assert_eq!(frames_remaining, 0);
}

/// The s16 entry point produces a stream whose decode matches the
/// input within the encoder's own noise floor (i16 in, i16-comparable
/// out).
#[test]
fn s16_entry_end_to_end() {
    let n = 6_000usize;
    let pcm_f = stereo_multitone(n);
    let pcm_i: Vec<i16> = pcm_f.iter().map(|&x| x.round() as i16).collect();
    let enc = encode_sv8_from_pcm_s16(&pcm_i, 2, 0, &Sv8EncoderSettings::default()).unwrap();
    let out = decode_sv8_stream(&enc.bytes).unwrap();
    let ours = out.pcm_s16();
    assert_eq!(ours.len(), pcm_i.len());
    let inp: Vec<f64> = pcm_i.iter().map(|&s| f64::from(s)).collect();
    let dec: Vec<f64> = ours.iter().map(|&s| f64::from(s)).collect();
    let snr = snr_db_per_channel(&inp, &dec, 2);
    eprintln!("s16 entry: SNR L {:.1} dB, R {:.1} dB", snr[0], snr[1]);
    // Measured ~80 / ~81 dB; the i16 rounding of input + output sits
    // inside the encoder's own noise floor.
    assert!(snr[0] > 70.0, "L SNR {:.1} dB", snr[0]);
    assert!(snr[1] > 70.0, "R SNR {:.1} dB", snr[1]);
}
