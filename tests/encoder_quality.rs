//! Measured rate-ladder gates for the SMR-driven allocation
//! (`smr_alloc`, r454): the `quality` knob must trade rate for
//! fidelity monotonically on both encoder generations, and the
//! perceptual allocation must undercut the flat allocation's rate on
//! masking-friendly (music-like) content while keeping the stream
//! decodable at full declared length.

use oxideav_musepack::mpc_decode::decode_mpc_stream;
use oxideav_musepack::sv7_pcm_encode::{encode_sv7_from_pcm_f64, Sv7EncoderSettings};
use oxideav_musepack::sv8_file_encode::{encode_sv8_from_pcm_f64, Sv8EncoderSettings};
use std::f64::consts::PI;

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

/// Music-like stereo test content in the s16 domain: a handful of
/// tones with harmonics (strong maskers across several subbands) over
/// a low hiss floor — the shape the SMR allocation is built for.
fn music_pcm(seconds: f64) -> Vec<f64> {
    let n = (44100.0 * seconds) as usize;
    let mut rng = Rng(0x5eed_4545);
    let mut pcm = Vec::with_capacity(n * 2);
    for t in 0..n {
        let x = t as f64 / 44100.0;
        let env = 0.6 + 0.4 * (2.0 * PI * 1.5 * x).sin();
        let l = env
            * (9000.0 * (2.0 * PI * 440.0 * x).sin()
                + 3000.0 * (2.0 * PI * 880.0 * x).sin()
                + 1500.0 * (2.0 * PI * 2640.0 * x).sin())
            + 120.0 * rng.next();
        let r = env
            * (8000.0 * (2.0 * PI * 660.0 * x).sin()
                + 2500.0 * (2.0 * PI * 1980.0 * x).sin()
                + 1200.0 * (2.0 * PI * 5280.0 * x).sin())
            + 120.0 * rng.next();
        pcm.push(l);
        pcm.push(r);
    }
    pcm
}

fn snr_db(input: &[f64], decoded: &[f64]) -> f64 {
    assert_eq!(input.len(), decoded.len());
    let (mut sig, mut err) = (0.0_f64, 0.0_f64);
    for (a, b) in input.iter().zip(decoded.iter()) {
        sig += a * a;
        err += (a - b) * (a - b);
    }
    10.0 * (sig / err.max(1e-12)).log10()
}

fn kbps(bytes: usize, samples_per_channel: usize) -> f64 {
    bytes as f64 * 8.0 * 44100.0 / samples_per_channel as f64 / 1000.0
}

/// One ladder measurement: encode at `quality`, decode our own
/// stream, return `(bytes, snr_db)`.
fn measure_sv8(pcm: &[f64], quality: Option<f64>) -> (usize, f64) {
    let enc = encode_sv8_from_pcm_f64(
        pcm,
        2,
        0,
        &Sv8EncoderSettings {
            quality,
            ..Sv8EncoderSettings::default()
        },
    )
    .expect("SV8 encode");
    let dec = decode_mpc_stream(&enc.bytes).expect("decode");
    assert_eq!(dec.pcm().len(), pcm.len(), "gapless length");
    (enc.bytes.len(), snr_db(pcm, dec.pcm()))
}

fn measure_sv7(pcm: &[f64], quality: Option<f64>) -> (usize, f64) {
    let enc = encode_sv7_from_pcm_f64(
        pcm,
        2,
        0,
        &Sv7EncoderSettings {
            quality,
            ..Sv7EncoderSettings::default()
        },
    )
    .expect("SV7 encode");
    let dec = decode_mpc_stream(&enc.bytes).expect("decode");
    assert_eq!(dec.pcm().len(), pcm.len(), "gapless length");
    (enc.bytes.len(), snr_db(pcm, dec.pcm()))
}

#[test]
fn sv8_quality_ladder_is_monotone() {
    let pcm = music_pcm(1.5);
    let spc = pcm.len() / 2;
    let ladder: Vec<(usize, f64)> = [2.0, 5.0, 8.0]
        .iter()
        .map(|&q| measure_sv8(&pcm, Some(q)))
        .collect();
    for (q, (bytes, snr)) in [2.0, 5.0, 8.0].iter().zip(&ladder) {
        println!(
            "SV8 q{q}: {bytes} bytes ({:.1} kbps), SNR {snr:.1} dB",
            kbps(*bytes, spc)
        );
    }
    // Rate strictly grows with quality; fidelity strictly grows too.
    assert!(ladder[0].0 < ladder[1].0 && ladder[1].0 < ladder[2].0);
    assert!(ladder[0].1 < ladder[1].1 && ladder[1].1 < ladder[2].1);
}

#[test]
fn sv7_quality_ladder_is_monotone() {
    let pcm = music_pcm(1.5);
    let spc = pcm.len() / 2;
    let ladder: Vec<(usize, f64)> = [2.0, 5.0, 8.0]
        .iter()
        .map(|&q| measure_sv7(&pcm, Some(q)))
        .collect();
    for (q, (bytes, snr)) in [2.0, 5.0, 8.0].iter().zip(&ladder) {
        println!(
            "SV7 q{q}: {bytes} bytes ({:.1} kbps), SNR {snr:.1} dB",
            kbps(*bytes, spc)
        );
    }
    assert!(ladder[0].0 < ladder[1].0 && ladder[1].0 < ladder[2].0);
    assert!(ladder[0].1 < ladder[1].1 && ladder[1].1 < ladder[2].1);
}

#[test]
fn smr_undercuts_flat_rate_on_music_like_content() {
    let pcm = music_pcm(1.5);
    let spc = pcm.len() / 2;
    let (flat_bytes, flat_snr) = measure_sv8(&pcm, None);
    let (smr_bytes, smr_snr) = measure_sv8(&pcm, Some(7.0));
    println!(
        "flat: {flat_bytes} bytes ({:.1} kbps) SNR {flat_snr:.1} dB; \
         SMR q7: {smr_bytes} bytes ({:.1} kbps) SNR {smr_snr:.1} dB",
        kbps(flat_bytes, spc),
        kbps(smr_bytes, spc)
    );
    assert!(
        smr_bytes < flat_bytes,
        "SMR q7 ({smr_bytes}) must spend less than flat ({flat_bytes})"
    );
    assert!(smr_snr > 30.0, "SMR q7 SNR {smr_snr:.1} dB");
}
