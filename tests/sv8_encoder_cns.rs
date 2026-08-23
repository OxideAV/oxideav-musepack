//! Encoder-side CNS (`Res == −1`) gates — r450: the from-PCM SV8
//! encoder can now *emit* noise-substitution bands (opt-in
//! `pns_threshold`), closing the last decode-ladder case the encoder
//! never exercised.
//!
//! What makes this testable at all is the r450 native-oracle result
//! (`tests/cns_native_oracle.rs`): this crate's CNS generator and
//! consumption order are per-sample identical to the native console
//! decoder's, so an encoder that emits `Res == −1` bands produces
//! streams whose noise decodes *deterministically and identically*
//! here and in the reference tools — the binary-gated parity test
//! below proves that end to end on our own streams.
//!
//! The substitution trades waveform fidelity for rate on hiss-like
//! bands: the gates therefore check (a) structural emission and the
//! `EI` PNS flag, (b) rate strictly saved, (c) tonal content still
//! transparent and noise-band *loudness* preserved, (d) black-box
//! decoder parity.

use oxideav_musepack::cns::CnsPrng;
use oxideav_musepack::huffman::Sv7BitReader;
use oxideav_musepack::packet_stream::{PacketSizeConvention, PacketStream};
use oxideav_musepack::sv8_decode::decode_sv8_stream;
use oxideav_musepack::sv8_file_encode::{encode_sv8_from_pcm_f64, Sv8EncoderSettings};
use oxideav_musepack::sv8_stereo_frame::{decode_sv8_stereo_frame, Sv8FrameState};
use oxideav_musepack::typed_packet::TypedPacket;
use std::f64::consts::PI;

/// Tonal maskers + quiet wideband hiss (deterministic), in the same
/// spirit as the staged `cns-pns` fixture source: the hiss is what a
/// noise-substituting encoder should replace.
fn tone_plus_hiss(n: usize) -> Vec<f64> {
    let mut rng: u64 = 0x00c5_5eed_0000_0001;
    let mut next = move || {
        rng ^= rng >> 12;
        rng ^= rng << 25;
        rng ^= rng >> 27;
        ((rng.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 11) as f64) / ((1u64 << 53) as f64) * 2.0 - 1.0
    };
    let mut pcm = Vec::with_capacity(n * 2);
    for t in 0..n {
        let x = t as f64 / 44100.0;
        let tone_l = 9000.0 * (2.0 * PI * 220.0 * x).sin();
        let tone_r = 9000.0 * (2.0 * PI * 330.0 * x).sin();
        let hiss = 260.0 * next();
        pcm.push(tone_l + hiss);
        pcm.push(tone_r + hiss * 0.8);
    }
    pcm
}

fn settings(pns_threshold: f64) -> Sv8EncoderSettings {
    Sv8EncoderSettings {
        pns_threshold,
        ..Sv8EncoderSettings::default()
    }
}

/// Count `Res == −1` (band, channel) instances across a stream by
/// structurally decoding every frame body.
fn count_cns_instances(bytes: &[u8]) -> (u64, bool) {
    let mut stream = PacketStream::new(&bytes[4..], PacketSizeConvention::Inclusive);
    let mut sh = None;
    let mut ei_pns = false;
    let mut cns_count = 0u64;
    let mut frames_remaining = 0u64;
    while let Some(p) = stream.next_packet().expect("walk") {
        match TypedPacket::classify(p) {
            TypedPacket::StreamHeader(h) => {
                let f = h.fields().expect("sh");
                frames_remaining = f
                    .sample_count
                    .div_ceil(oxideav_musepack::SAMPLES_PER_FRAME_PER_CHANNEL as u64);
                sh = Some(f);
            }
            TypedPacket::EncoderInfo(e) => {
                ei_pns = e.fields().expect("ei").pns;
            }
            TypedPacket::Audio(ap) => {
                let f = sh.as_ref().expect("sh before ap");
                let frames = frames_remaining.min(f.frames_per_audio_packet());
                let mut payload = ap.payload_bytes().to_vec();
                payload.extend_from_slice(&[0, 0]);
                let mut reader = Sv7BitReader::new(&payload);
                let mut state = Sv8FrameState::new();
                let mut cns = CnsPrng::new();
                for pf in 0..frames {
                    let frame = decode_sv8_stereo_frame(
                        &mut reader,
                        f.max_band,
                        pf == 0,
                        f.mid_side,
                        &mut state,
                        &mut cns,
                    )
                    .expect("frame decode");
                    for rr in frame.res.iter() {
                        for &bt in rr.iter() {
                            if bt == -1 {
                                cns_count += 1;
                            }
                        }
                    }
                }
                frames_remaining -= frames;
            }
            TypedPacket::StreamEnd(_) => break,
            _ => {}
        }
    }
    (cns_count, ei_pns)
}

/// Second-difference high-pass (noise-dominated region) over
/// interleaved stereo.
fn hf_rms(x: &[f64]) -> f64 {
    let nch = 2;
    let v: Vec<f64> = x
        .iter()
        .enumerate()
        .skip(2 * nch)
        .map(|(i, &s)| s - x[i - 2 * nch])
        .collect();
    (v.iter().map(|&s| s * s).sum::<f64>() / v.len() as f64).sqrt()
}

/// Complex projection of one channel of interleaved stereo onto a
/// tone at `freq` — the tone's amplitude-and-phase coefficient.
fn tone_coeff(x: &[f64], ch: usize, freq: f64) -> (f64, f64) {
    let (mut re, mut im) = (0.0f64, 0.0f64);
    let mut n = 0usize;
    for (i, &s) in x.iter().enumerate() {
        if i % 2 != ch {
            continue;
        }
        let w = 2.0 * PI * freq * (n as f64) / 44100.0;
        re += s * w.cos();
        im -= s * w.sin();
        n += 1;
    }
    (re / n as f64, im / n as f64)
}

/// Structural emission: off by default; on with a threshold, with the
/// `EI` PNS flag raised and a strictly smaller stream.
#[test]
fn cns_emission_is_opt_in_and_saves_rate() {
    let pcm = tone_plus_hiss(66150);
    let off = encode_sv8_from_pcm_f64(&pcm, 2, 0, &settings(0.0)).expect("encode off");
    let on = encode_sv8_from_pcm_f64(&pcm, 2, 0, &settings(400.0)).expect("encode on");

    let (cns_off, pns_off) = count_cns_instances(&off.bytes);
    let (cns_on, pns_on) = count_cns_instances(&on.bytes);
    println!(
        "off: {} bytes, {} CNS; on: {} bytes, {} CNS",
        off.bytes.len(),
        cns_off,
        on.bytes.len(),
        cns_on
    );
    assert_eq!(cns_off, 0, "default emits no CNS");
    assert!(!pns_off);
    assert!(cns_on > 100, "threshold engages CNS ({cns_on} instances)");
    assert!(pns_on, "EI PNS flag raised");
    assert!(
        (on.bytes.len() as f64) < 0.85 * off.bytes.len() as f64,
        "CNS saves the hiss bands' sample bits: {} vs {}",
        on.bytes.len(),
        off.bytes.len()
    );
}

/// Fidelity split: the tonal content stays transparent (the coded
/// masker bands are untouched by the substitution — their projected
/// amplitude/phase survives to within a fraction of a percent) while
/// the substituted noise keeps the original loudness (rms within a
/// factor bounded by the SCF grid), even though its waveform is the
/// decoder PRNG's, not the input's.
#[test]
fn cns_streams_preserve_tone_fidelity_and_noise_loudness() {
    let pcm = tone_plus_hiss(66150);
    let on = encode_sv8_from_pcm_f64(&pcm, 2, 0, &settings(400.0)).expect("encode");
    let dec = decode_sv8_stream(&on.bytes).expect("decode");
    assert_eq!(dec.pcm.len(), pcm.len());

    for (ch, freq) in [(0usize, 220.0f64), (1, 330.0)] {
        let (or_re, or_im) = tone_coeff(&pcm, ch, freq);
        let (de_re, de_im) = tone_coeff(&dec.pcm, ch, freq);
        let orig_mag = (or_re * or_re + or_im * or_im).sqrt();
        let err = ((de_re - or_re).powi(2) + (de_im - or_im).powi(2)).sqrt();
        let rel = err / orig_mag;
        println!("ch {ch} tone {freq} Hz: relative coefficient error {rel:.5}");
        assert!(rel < 0.01, "tone degraded on channel {ch}: {rel:.5}");
    }
    let hf_ratio = hf_rms(&dec.pcm) / hf_rms(&pcm);
    println!("noise-region rms ratio {hf_ratio:.3}");
    assert!(
        (0.5..=2.0).contains(&hf_ratio),
        "substituted noise loudness off: {hf_ratio:.3}"
    );
}

/// Black-box parity: the reference console decoder decodes our
/// CNS-bearing stream to the same PCM we do, within ±1 LSB — the
/// generator identity proven on the corpus holds for encoder-emitted
/// noise bands too. Skips without the binary (CI).
#[test]
fn reference_decoder_matches_our_decode_of_cns_streams() {
    let Some(mpcdec) = std::env::split_paths(&std::env::var_os("PATH").unwrap_or_default())
        .map(|d| d.join("mpcdec"))
        .find(|c| c.is_file())
    else {
        eprintln!("skip: no reference console decoder on PATH");
        return;
    };
    let pcm = tone_plus_hiss(44100);
    let on = encode_sv8_from_pcm_f64(&pcm, 2, 0, &settings(400.0)).expect("encode");
    let ours = decode_sv8_stream(&on.bytes).expect("decode").pcm_s16();

    let dir = std::env::temp_dir().join("oxideav-musepack-cns-test");
    std::fs::create_dir_all(&dir).expect("mkdir");
    let mpc = dir.join("own-cns.mpc");
    let wav = dir.join("own-cns.wav");
    std::fs::write(&mpc, &on.bytes).expect("write");
    let st = std::process::Command::new(&mpcdec)
        .arg(&mpc)
        .arg(&wav)
        .output()
        .expect("run decoder");
    assert!(st.status.success(), "reference decoder rejected the stream");
    let wav_bytes = std::fs::read(&wav).expect("read wav");
    // 44-byte canonical RIFF header, then s16le samples.
    let data = &wav_bytes[44..];
    let theirs: Vec<i16> = data
        .chunks_exact(2)
        .map(|c| i16::from_le_bytes([c[0], c[1]]))
        .collect();
    assert_eq!(theirs.len(), ours.len(), "sample counts differ");
    let mut max_d = 0i32;
    for (i, (&a, &b)) in ours.iter().zip(theirs.iter()).enumerate() {
        let d = (i32::from(a) - i32::from(b)).abs();
        assert!(d <= 1, "sample {i}: ours {a} vs reference {b}");
        max_d = max_d.max(d);
    }
    println!("reference parity on CNS stream: max |delta| = {max_d}");
}
