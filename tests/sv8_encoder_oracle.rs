//! Black-box **oracle gates for the from-PCM SV8 encoder** (r429):
//! when the reference decoders are installed, decode our own encoded
//! streams with them and require playable output, input alignment,
//! and parity with our own decode.
//!
//! Two black-box validators (binaries only; no source consulted):
//!
//! - the reference console decoder (`mpcdec`, Musepack tools — the
//!   same family as the corpus producers): must emit **exactly N
//!   input-aligned samples** (it applies the r429 gapless window
//!   itself, so this validates our `SH` totals + silence posture
//!   end-to-end) at max 1-LSB parity with our own decode;
//! - FFmpeg's `mpc8` decoder: emits the untrimmed run; our window
//!   must match it at the `481 + silence` offset at the same parity.
//!
//! On machines without the binaries the tests print a skip note and
//! pass vacuously (CI runners carry no reference tools; the staged
//! corpus gates in `tests/sv8_corpus.rs` remain the always-on oracle
//! coverage).

use oxideav_musepack::sv8_decode::decode_sv8_stream;
use oxideav_musepack::sv8_file_encode::{encode_sv8_from_pcm_f64, Sv8EncoderSettings};
use std::f64::consts::PI;
use std::path::PathBuf;
use std::process::Command;

/// Locate a binary on PATH.
fn find_bin(name: &str) -> Option<PathBuf> {
    let path = std::env::var_os("PATH")?;
    std::env::split_paths(&path)
        .map(|d| d.join(name))
        .find(|c| c.is_file())
}

/// Interleaved s16le bytes → i16s.
fn s16_of(bytes: &[u8]) -> Vec<i16> {
    bytes
        .chunks_exact(2)
        .map(|c| i16::from_le_bytes([c[0], c[1]]))
        .collect()
}

/// Minimal WAV reader: return the s16 samples of the `data` chunk.
fn wav_s16(bytes: &[u8]) -> Vec<i16> {
    assert_eq!(&bytes[..4], b"RIFF", "oracle wav: RIFF header");
    let mut pos = 12;
    while pos + 8 <= bytes.len() {
        let id = &bytes[pos..pos + 4];
        let len = u32::from_le_bytes(bytes[pos + 4..pos + 8].try_into().unwrap()) as usize;
        if id == b"data" {
            return s16_of(&bytes[pos + 8..(pos + 8 + len).min(bytes.len())]);
        }
        pos += 8 + len + (len & 1);
    }
    panic!("oracle wav: no data chunk");
}

/// SNR of `b` against reference `a`, plus the max absolute diff.
fn snr_and_max(a: &[i16], b: &[i16]) -> (f64, i32) {
    assert_eq!(a.len(), b.len());
    let (mut sig, mut err) = (0.0_f64, 0.0_f64);
    let mut max = 0i32;
    for (&x, &y) in a.iter().zip(b.iter()) {
        let d = i32::from(y) - i32::from(x);
        sig += f64::from(x) * f64::from(x);
        err += f64::from(d) * f64::from(d);
        max = max.max(d.abs());
    }
    let snr = if err == 0.0 {
        f64::INFINITY
    } else {
        10.0 * (sig / err).log10()
    };
    (snr, max)
}

struct Case {
    name: &'static str,
    channels: u8,
    pcm: Vec<f64>,
}

fn cases() -> Vec<Case> {
    let sr = 44_100.0;
    let n = 22_050usize;
    let mut stereo = Vec::with_capacity(2 * n);
    for i in 0..n {
        let t = i as f64 / sr;
        stereo.push(
            18_000.0 * (2.0 * PI * 440.0 * t).sin() + 6_000.0 * (2.0 * PI * 2_970.0 * t).sin(),
        );
        stereo.push(
            15_000.0 * (2.0 * PI * 660.0 * t).sin() + 7_500.0 * (2.0 * PI * 5_512.5 * t).sin(),
        );
    }
    let mono: Vec<f64> = (0..n)
        .map(|i| 20_000.0 * (2.0 * PI * 525.0 * (i as f64) / sr).sin())
        .collect();
    vec![
        Case {
            name: "stereo",
            channels: 2,
            pcm: stereo,
        },
        Case {
            name: "mono",
            channels: 1,
            pcm: mono,
        },
    ]
}

/// Reference console decoder: exact length, input alignment, 1-LSB
/// parity with our own decode.
#[test]
fn reference_console_decoder_plays_our_streams_aligned() {
    let Some(bin) = find_bin("mpcdec") else {
        eprintln!("skipped: mpcdec not on PATH (staged-corpus gates still cover decode)");
        return;
    };
    let dir = std::env::temp_dir().join(format!("oxideav-musepack-oracle-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    for case in cases() {
        let enc =
            encode_sv8_from_pcm_f64(&case.pcm, case.channels, 0, &Sv8EncoderSettings::default())
                .unwrap();
        let mpc = dir.join(format!("{}.mpc", case.name));
        let wav = dir.join(format!("{}.wav", case.name));
        std::fs::write(&mpc, &enc.bytes).unwrap();
        let status = Command::new(&bin)
            .arg(&mpc)
            .arg(&wav)
            .output()
            .expect("run oracle");
        assert!(
            status.status.success(),
            "{}: oracle decoder failed: {}",
            case.name,
            String::from_utf8_lossy(&status.stderr)
        );
        let oracle = wav_s16(&std::fs::read(&wav).unwrap());
        assert_eq!(
            oracle.len(),
            case.pcm.len(),
            "{}: oracle must emit exactly the input sample count",
            case.name
        );

        // Input alignment.
        let input: Vec<i16> = case
            .pcm
            .iter()
            .map(|&v| v.round().clamp(-32768.0, 32767.0) as i16)
            .collect();
        let (snr_in, _) = snr_and_max(&input, &oracle);
        assert!(
            snr_in > 70.0,
            "{}: oracle-vs-input SNR {snr_in:.1} dB",
            case.name
        );

        // Parity with our own decode (its float DSP vs our f64).
        let ours = decode_sv8_stream(&enc.bytes).unwrap().pcm_s16();
        let (snr_parity, max_d) = snr_and_max(&ours, &oracle);
        assert!(
            max_d <= 1,
            "{}: oracle-vs-ours max diff {max_d} LSB",
            case.name
        );
        assert!(
            snr_parity > 75.0,
            "{}: oracle-vs-ours SNR {snr_parity:.1} dB",
            case.name
        );
        eprintln!(
            "{}: oracle aligned ({} samples), SNR vs input {snr_in:.1} dB, parity {snr_parity:.1} dB / max {max_d} LSB",
            case.name,
            oracle.len()
        );
    }
    let _ = std::fs::remove_dir_all(&dir);
}

/// FFmpeg oracle: untrimmed output; our gapless window matches it at
/// the `481 + silence` offset at 1-LSB parity.
#[test]
fn ffmpeg_oracle_matches_our_window_at_the_priming_offset() {
    let Some(bin) = find_bin("ffmpeg") else {
        eprintln!("skipped: ffmpeg not on PATH (staged-corpus gates still cover decode)");
        return;
    };
    let dir = std::env::temp_dir().join(format!("oxideav-musepack-ffo-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    for case in cases() {
        let enc =
            encode_sv8_from_pcm_f64(&case.pcm, case.channels, 0, &Sv8EncoderSettings::default())
                .unwrap();
        let mpc = dir.join(format!("{}.mpc", case.name));
        let raw = dir.join(format!("{}.pcm", case.name));
        std::fs::write(&mpc, &enc.bytes).unwrap();
        let status = Command::new(&bin)
            .args(["-y", "-loglevel", "error", "-i"])
            .arg(&mpc)
            .args(["-f", "s16le", "-acodec", "pcm_s16le"])
            .arg(&raw)
            .output()
            .expect("run ffmpeg");
        assert!(
            status.status.success(),
            "{}: ffmpeg failed: {}",
            case.name,
            String::from_utf8_lossy(&status.stderr)
        );
        let oracle = s16_of(&std::fs::read(&raw).unwrap());

        let header = decode_sv8_stream(&enc.bytes).unwrap();
        let silence = header.header.beginning_silence as usize;
        let offset = (481 + silence) * case.channels as usize;
        let ours = header.pcm_s16();
        assert!(
            oracle.len() >= offset + ours.len(),
            "{}: ffmpeg output too short",
            case.name
        );
        let (snr, max_d) = snr_and_max(&ours, &oracle[offset..offset + ours.len()]);
        assert!(max_d <= 1, "{}: max diff {max_d} LSB", case.name);
        assert!(snr > 75.0, "{}: parity SNR {snr:.1} dB", case.name);
        eprintln!(
            "{}: ffmpeg parity at offset {offset}: SNR {snr:.1} dB, max {max_d} LSB",
            case.name
        );
    }
    let _ = std::fs::remove_dir_all(&dir);
}
