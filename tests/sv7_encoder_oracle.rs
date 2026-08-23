//! Black-box oracle gates for the **from-PCM SV7 encoder** (r450):
//! decode our own `MP+` streams with the reference tools and require
//! input alignment and parity with our own decode — the SV7 twin of
//! `tests/sv8_encoder_oracle.rs`. Skips without the binaries (CI).

use oxideav_musepack::sv7_file_decode::decode_sv7_file;
use oxideav_musepack::sv7_pcm_encode::{encode_sv7_from_pcm_f64, Sv7EncoderSettings};
use std::f64::consts::PI;
use std::path::PathBuf;
use std::process::Command;

fn find_bin(name: &str) -> Option<PathBuf> {
    let path = std::env::var_os("PATH")?;
    std::env::split_paths(&path)
        .map(|d| d.join(name))
        .find(|c| c.is_file())
}

fn s16_of(bytes: &[u8]) -> Vec<i16> {
    bytes
        .chunks_exact(2)
        .map(|c| i16::from_le_bytes([c[0], c[1]]))
        .collect()
}

fn test_signal(n: usize, hiss: f64) -> Vec<f64> {
    let mut rng: u64 = 0x517e_0451_beef_cafe;
    let mut next = move || {
        rng ^= rng >> 12;
        rng ^= rng << 25;
        rng ^= rng >> 27;
        ((rng.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 11) as f64) / ((1u64 << 53) as f64) * 2.0 - 1.0
    };
    let mut pcm = Vec::with_capacity(n * 2);
    for t in 0..n {
        let x = t as f64 / 44100.0;
        pcm.push(9000.0 * (2.0 * PI * 440.0 * x).sin() + hiss * next());
        pcm.push(9000.0 * (2.0 * PI * 660.0 * x).sin() + hiss * next());
    }
    pcm
}

fn write_stream(name: &str, bytes: &[u8]) -> PathBuf {
    let dir = std::env::temp_dir().join("oxideav-musepack-sv7enc-test");
    std::fs::create_dir_all(&dir).expect("mkdir");
    let p = dir.join(name);
    std::fs::write(&p, bytes).expect("write stream");
    p
}

fn snr_i16(a: &[i16], b: &[i16]) -> f64 {
    let (mut s, mut e) = (0.0f64, 0.0f64);
    for (x, y) in a.iter().zip(b.iter()) {
        let (x, y) = (f64::from(*x), f64::from(*y));
        s += x * x;
        e += (x - y) * (x - y);
    }
    10.0 * (s / e).log10()
}

/// The reference console decoder plays our SV7 streams: exactly N
/// input-aligned samples per channel (it applies the gapless window
/// itself, flush frame included), at 1-LSB parity with our decode.
#[test]
fn reference_console_decoder_plays_our_sv7_streams_aligned() {
    let Some(mpcdec) = find_bin("mpcdec") else {
        eprintln!("skip: no reference console decoder on PATH");
        return;
    };
    // One length with slack ≥ 481 (no flush) and one exact multiple
    // (flush frame engages).
    for (name, n) in [("plain", 20_000usize), ("flush", 1152 * 12)] {
        let pcm = test_signal(n, 0.0);
        let enc =
            encode_sv7_from_pcm_f64(&pcm, 2, 0, &Sv7EncoderSettings::default()).expect("encode");
        let ours = decode_sv7_file(&enc.bytes).expect("decode").pcm_s16();
        assert_eq!(ours.len(), n * 2);

        let mpc = write_stream(&format!("{name}.mpc"), &enc.bytes);
        let wav = mpc.with_extension("wav");
        let st = Command::new(&mpcdec)
            .arg(&mpc)
            .arg(&wav)
            .output()
            .expect("run decoder");
        assert!(st.status.success(), "{name}: reference decoder rejected");
        let theirs = s16_of(&std::fs::read(&wav).expect("read wav")[44..]);
        assert_eq!(theirs.len(), n * 2, "{name}: sample count");
        let mut max_d = 0i32;
        for (i, (&a, &b)) in ours.iter().zip(theirs.iter()).enumerate() {
            let d = (i32::from(a) - i32::from(b)).abs();
            assert!(d <= 1, "{name} sample {i}: ours {a} vs reference {b}");
            max_d = max_d.max(d);
        }
        let s = snr_i16(
            &pcm.iter().map(|&v| v.round() as i16).collect::<Vec<i16>>(),
            &theirs,
        );
        println!("{name}: reference parity max |delta| = {max_d}, SNR vs input {s:.1} dB");
        assert!(s > 55.0, "{name}: reference decode SNR {s:.1} dB");
    }
}

/// The FFmpeg oracle decodes our SV7 streams too; it performs no
/// gapless skip, so our window matches at the 481-sample offset.
#[test]
fn ffmpeg_oracle_matches_our_sv7_window_at_the_priming_offset() {
    let Some(ffmpeg) = find_bin("ffmpeg") else {
        eprintln!("skip: no ffmpeg binary on PATH");
        return;
    };
    let n = 20_000usize;
    let pcm = test_signal(n, 0.0);
    let enc = encode_sv7_from_pcm_f64(&pcm, 2, 0, &Sv7EncoderSettings::default()).expect("encode");
    let ours = decode_sv7_file(&enc.bytes).expect("decode").pcm_s16();

    let mpc = write_stream("ffmpeg-probe.mpc", &enc.bytes);
    let out = Command::new(&ffmpeg)
        .args(["-v", "error", "-i"])
        .arg(&mpc)
        .args(["-f", "s16le", "-acodec", "pcm_s16le", "-"])
        .output()
        .expect("run ffmpeg");
    assert!(
        out.status.success(),
        "ffmpeg failed: {}",
        String::from_utf8_lossy(&out.stderr)
    );
    let theirs = s16_of(&out.stdout);
    let offset = 481 * 2;
    assert!(
        theirs.len() >= offset + ours.len(),
        "untrimmed run too short"
    );
    let mut max_d = 0i32;
    for (i, &a) in ours.iter().enumerate() {
        let b = theirs[offset + i];
        let d = (i32::from(a) - i32::from(b)).abs();
        assert!(d <= 1, "sample {i}: ours {a} vs oracle {b}");
        max_d = max_d.max(d);
    }
    println!("ffmpeg parity at offset 481: max |delta| = {max_d}");
}

/// CNS emission on SV7: the reference console decoder decodes our
/// `MP+ 0x17` PNS streams at 1-LSB parity with our own decode (the
/// generator identity), and the stream is much smaller.
#[test]
fn reference_decoder_matches_our_sv7_cns_streams() {
    let Some(mpcdec) = find_bin("mpcdec") else {
        eprintln!("skip: no reference console decoder on PATH");
        return;
    };
    let n = 30_000usize;
    let pcm = test_signal(n, 260.0);
    let base = encode_sv7_from_pcm_f64(&pcm, 2, 0, &Sv7EncoderSettings::default()).expect("plain");
    let cns_settings = Sv7EncoderSettings {
        pns_threshold: 400.0,
        ..Sv7EncoderSettings::default()
    };
    let enc = encode_sv7_from_pcm_f64(&pcm, 2, 0, &cns_settings).expect("cns encode");
    assert_eq!(&enc.bytes[..3], b"MP+");
    assert_eq!(enc.bytes[3], 0x17, "PNS version byte");
    assert!(
        (enc.bytes.len() as f64) < 0.8 * base.bytes.len() as f64,
        "CNS saves rate: {} vs {}",
        enc.bytes.len(),
        base.bytes.len()
    );

    let ours = decode_sv7_file(&enc.bytes).expect("decode").pcm_s16();
    let mpc = write_stream("cns.mpc", &enc.bytes);
    let wav = mpc.with_extension("wav");
    let st = Command::new(&mpcdec)
        .arg(&mpc)
        .arg(&wav)
        .output()
        .expect("run decoder");
    assert!(st.status.success(), "reference decoder rejected CNS stream");
    let theirs = s16_of(&std::fs::read(&wav).expect("read wav")[44..]);
    assert_eq!(theirs.len(), ours.len());
    let mut max_d = 0i32;
    for (i, (&a, &b)) in ours.iter().zip(theirs.iter()).enumerate() {
        let d = (i32::from(a) - i32::from(b)).abs();
        assert!(d <= 1, "sample {i}: ours {a} vs reference {b}");
        max_d = max_d.max(d);
    }
    println!(
        "SV7 CNS parity: max |delta| = {max_d}; {} vs {} bytes",
        enc.bytes.len(),
        base.bytes.len()
    );
}
