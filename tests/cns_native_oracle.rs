//! CNS calibration gates against the **native console-decoder
//! oracle** (`expected-mpcdec.pcm`, docs staging `af8e75c` —
//! `fixtures/cns-pns/notes.md` "Second oracle").
//!
//! The staged notes measure that the two black-box oracles disagree
//! on the CNS noise itself: the FFmpeg `mpc7`/`mpc8` decode carries a
//! noise floor ~1.7-1.8× the native decoder's across exactly the
//! substituted bands (mean power ratio 1.675 over bands 8..27,
//! agreement within 0.06 dB outside them) — so no single "the"
//! oracle waveform exists. The staged notes designate the native
//! decoder as the calibration reference for any CNS coefficient, and
//! against *that* oracle the waveform turns out to be fully
//! conformance-checkable after all: this crate's decode of the CNS
//! stream lands within **±1 LSB per sample** of the native PCM
//! (headline gate below), which retro-validates the staged two-LFSR
//! generator facts *and* this crate's generator-consumption order as
//! the native decoder's own. The r405 "waveform not reproducible"
//! finding was a property of the FFmpeg oracle's different generator,
//! not of the staged facts. The remaining gates pin the same story at
//! the aggregate level:
//!
//! - the native gapless window is the same `decoded[481 .. 481 +
//!   total]` law this crate pinned in r429, so the two decodes align
//!   1:1 with no offset;
//! - overall loudness must sit at the native decoder's level, not the
//!   FFmpeg oracle's inflated one;
//! - the high-frequency (noise-dominated) energy must match the
//!   native level — the discriminating band: the FFmpeg oracle is
//!   ~2.8× in power there;
//! - the SV7 stream and its SV8 transcode decode to the **same** PCM
//!   here, mirroring the staged fact that the native decodes of the
//!   pair are byte-identical (the CNS parameters and generator state
//!   survive the transcode).
//!
//! Measured values are printed by each gate; thresholds leave slack
//! so only a structural regression (CNS gain law, PRNG wiring, SCF
//! participation) trips them.

use oxideav_musepack::sv7_file_decode::decode_sv7_file;
use oxideav_musepack::sv8_decode::decode_sv8_stream;

fn fixture(gen: &str, file: &str) -> Vec<u8> {
    let path = format!(
        "{}/tests/fixtures/{gen}/cns-pns/{file}",
        env!("CARGO_MANIFEST_DIR")
    );
    std::fs::read(&path).unwrap_or_else(|e| panic!("read {path}: {e}"))
}

fn s16(bytes: &[u8]) -> Vec<i16> {
    bytes
        .chunks_exact(2)
        .map(|c| i16::from_le_bytes([c[0], c[1]]))
        .collect()
}

fn rms(x: &[i16]) -> f64 {
    (x.iter().map(|&v| f64::from(v) * f64::from(v)).sum::<f64>() / x.len() as f64).sqrt()
}

/// Second-difference high-pass per channel over interleaved stereo:
/// `y[n] = x[n] − x[n−2·nch]` suppresses the tonal maskers (220/330
/// Hz: |H| ≈ 0.06 at 330 Hz) while passing the substituted noise
/// region (bands 8..27, ≥ 5.5 kHz: |H| ≥ 1.4) — a cheap projector
/// onto the noise-dominated spectrum.
fn hf(x: &[i16]) -> Vec<f64> {
    let nch = 2;
    x.iter()
        .enumerate()
        .skip(2 * nch)
        .map(|(i, &v)| f64::from(v) - f64::from(x[i - 2 * nch]))
        .collect()
}

fn rms_f(x: &[f64]) -> f64 {
    (x.iter().map(|&v| v * v).sum::<f64>() / x.len() as f64).sqrt()
}

/// Window + loudness: the native oracle applies the same gapless law
/// this crate does, so the decodes align 1:1 sample-for-sample, and
/// the overall level must sit at the native decoder's — not at the
/// FFmpeg oracle's ~14 % (rms) inflated one.
#[test]
fn sv7_cns_loudness_calibrates_to_the_native_oracle() {
    let native = s16(&fixture("sv7", "expected-mpcdec.pcm"));
    let ours = decode_sv7_file(&fixture("sv7", "input.mpc"))
        .expect("decode")
        .pcm_s16();
    // Both decoders emit the r429 window: 22050 samples/ch.
    assert_eq!(ours.len(), native.len(), "gapless windows disagree");

    let (r_ours, r_native) = (rms(&ours), rms(&native));
    let ratio = r_ours / r_native;
    println!("rms ours={r_ours:.1} native={r_native:.1} ratio={ratio:.4}");
    // Staged notes: native rms 1905.7 vs FFmpeg 2175.9 (ratio 1.14).
    // Gate at ±6 % so the native level passes and the FFmpeg level
    // would not.
    assert!(
        (0.94..=1.06).contains(&ratio),
        "overall loudness off the native calibration: ratio {ratio:.4}"
    );
}

/// Noise-region calibration: in the second-difference high-pass
/// domain (noise-dominated — the substituted bands 8..27 carry the
/// only high-frequency content) our energy must match the native
/// oracle's. The FFmpeg oracle measures ~1.7× rms here, so this gate
/// pins which of the two amplitude conventions the crate follows.
#[test]
fn sv7_cns_noise_energy_matches_the_native_oracle() {
    let native = s16(&fixture("sv7", "expected-mpcdec.pcm"));
    let ours = decode_sv7_file(&fixture("sv7", "input.mpc"))
        .expect("decode")
        .pcm_s16();
    let (h_ours, h_native) = (hf(&ours), hf(&native));
    let (e_ours, e_native) = (rms_f(&h_ours), rms_f(&h_native));
    let ratio = e_ours / e_native;
    let db = 20.0 * ratio.log10();
    println!("hf rms ours={e_ours:.1} native={e_native:.1} ratio={ratio:.4} ({db:+.2} dB)");
    assert!(
        (0.85..=1.18).contains(&ratio),
        "noise-region energy off the native calibration: {ratio:.4} ({db:+.2} dB)"
    );
}

/// The residual against the native oracle sits in the two-independent-
/// noise-sequences regime: comparable to the noise floor itself (the
/// waveforms cannot match — see the module docs) but far below the
/// signal, with the tonal content strongly correlated.
#[test]
fn sv7_cns_residual_stays_in_the_noise_regime() {
    let native = s16(&fixture("sv7", "expected-mpcdec.pcm"));
    let ours = decode_sv7_file(&fixture("sv7", "input.mpc"))
        .expect("decode")
        .pcm_s16();
    let n = ours.len();
    let (mut dot, mut na, mut nb, mut err2) = (0.0f64, 0.0f64, 0.0f64, 0.0f64);
    for i in 0..n {
        let a = f64::from(ours[i]);
        let b = f64::from(native[i]);
        dot += a * b;
        na += a * a;
        nb += b * b;
        err2 += (a - b) * (a - b);
    }
    let corr = dot / (na.sqrt() * nb.sqrt());
    let rms_err = (err2 / n as f64).sqrt();
    let rms_sig = (nb / n as f64).sqrt();
    println!("corr={corr:.4} rms_err={rms_err:.1} rms_signal={rms_sig:.1}");
    assert!(corr > 0.70, "correlation collapsed: {corr:.4}");
    assert!(
        rms_err < 0.85 * rms_sig,
        "residual out of the noise regime: {rms_err:.1} vs {rms_sig:.1}"
    );
}

/// Transcode invariance on the CNS path: the staged notes pin that
/// the native decodes of the SV7 stream and its SV8 transcode are
/// byte-identical; this crate's two decoders must agree on the pair
/// the same way (same CNS parameters, same generator consumption).
#[test]
fn cns_sv7_and_sv8_transcode_decode_identically() {
    let sv7 = decode_sv7_file(&fixture("sv7", "input.mpc"))
        .expect("sv7 decode")
        .pcm_s16();
    let sv8 = decode_sv8_stream(&fixture("sv8", "input.mpc"))
        .expect("sv8 decode")
        .pcm_s16();
    assert_eq!(sv7.len(), sv8.len(), "window lengths differ");
    let diff = sv7.iter().zip(sv8.iter()).filter(|(a, b)| a != b).count();
    println!("sv7-vs-sv8 differing samples: {diff}/{}", sv7.len());
    assert_eq!(diff, 0, "SV7 and SV8 transcode decodes diverge");
}

/// The SV8 twin calibrates to the native oracle the same way the SV7
/// stream does (the sv8 fixture ships the identical native PCM).
#[test]
fn sv8_cns_loudness_calibrates_to_the_native_oracle() {
    let native = s16(&fixture("sv8", "expected-mpcdec.pcm"));
    let ours = decode_sv8_stream(&fixture("sv8", "input.mpc"))
        .expect("decode")
        .pcm_s16();
    assert_eq!(ours.len(), native.len(), "gapless windows disagree");
    let ratio = rms(&ours) / rms(&native);
    println!("sv8 rms ratio={ratio:.4}");
    assert!(
        (0.94..=1.06).contains(&ratio),
        "overall loudness off the native calibration: ratio {ratio:.4}"
    );
    let hf_ratio = rms_f(&hf(&ours)) / rms_f(&hf(&native));
    println!("sv8 hf rms ratio={hf_ratio:.4}");
    assert!(
        (0.85..=1.18).contains(&hf_ratio),
        "noise-region energy off the native calibration: {hf_ratio:.4}"
    );
}

/// The headline gate: the CNS noise waveform is **per-sample exact**
/// against the native oracle — max |delta| ≤ 1 LSB across the whole
/// stream (noise bands included), the same bound as the non-CNS
/// corpus. This proves the staged two-LFSR generator facts *and* this
/// crate's consumption order (whole-stream generator state, advanced
/// in band-decode order) are the native decoder's own; the earlier
/// "waveform not reproducible" finding was a property of the FFmpeg
/// oracle's different generator, not of the staged facts.
#[test]
fn cns_waveform_is_per_sample_exact_against_the_native_oracle() {
    for gen in ["sv7", "sv8"] {
        let native = s16(&fixture(gen, "expected-mpcdec.pcm"));
        let ours = if gen == "sv7" {
            decode_sv7_file(&fixture(gen, "input.mpc"))
                .expect("decode")
                .pcm_s16()
        } else {
            decode_sv8_stream(&fixture(gen, "input.mpc"))
                .expect("decode")
                .pcm_s16()
        };
        assert_eq!(ours.len(), native.len());
        let mut max_delta = 0i32;
        let mut exact = 0usize;
        for (i, (&a, &b)) in ours.iter().zip(native.iter()).enumerate() {
            let d = (i32::from(a) - i32::from(b)).abs();
            assert!(d <= 1, "{gen} sample {i}: ours {a} vs native {b}");
            max_delta = max_delta.max(d);
            if d == 0 {
                exact += 1;
            }
        }
        println!(
            "{gen}: max|delta|={max_delta}, {exact}/{} bit-exact ({:.1}%)",
            ours.len(),
            100.0 * exact as f64 / ours.len() as f64
        );
        // Measured: 49.9 % bit-exact (the ±1 residue is this crate's
        // f64 synthesis vs the native f32 DSP, as across the corpus).
        assert!(
            exact * 5 >= ours.len() * 2,
            "{gen}: bit-exact fraction collapsed"
        );
    }
}
