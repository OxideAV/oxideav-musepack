//! SV8 corpus conformance gates (round 419).
//!
//! Runs the crate's SV8 whole-stream decode against the fixture corpus
//! under `tests/fixtures/sv8/` — five lossless SV7→SV8 transcodes of
//! the staged SV7 corpus plus two fresh reference-encoder streams,
//! with FFmpeg's `mpc8` decoder as a black-box oracle (see the corpus
//! `README.md` for provenance).
//!
//! Three gate families:
//!
//! 1. **Oracle agreement** — every non-CNS stream decodes 100% within
//!    ±1 LSB of the oracle with ≥ 70% bit-exact samples (the residue
//!    is the oracle's f32 DSP vs this crate's f64 synthesis — the same
//!    profile as the SV7 corpus gates).
//! 2. **Lossless-transcode identity (§3.6)** — each transcoded SV8
//!    stream decodes to *bit-identical f64 PCM* with the SV7 decode of
//!    its sibling stream under `tests/fixtures/sv7/`: the two
//!    generations share the signal model exactly, so only the entropy
//!    layer differs. This is the oracle-free gate that pins the whole
//!    SV8 frame-body layout, including the CNS stream (the PRNG runs
//!    in the same decode order in both generations).
//! 3. **Structure** — header fields, packet/frame counts (incl. the
//!    two-packet 77-frame stream: key frame + SCF reset per `AP`),
//!    gapless trim lengths, and the magic dispatch.
//!
//! The `cns-pns` transcode is excluded from the per-sample oracle gate:
//! the oracle's CNS noise *waveform* is not reproducible from the
//! staged generator facts (a filed docs gap — see
//! `tests/sv7_cns_corpus.rs`). It is gated on its CNS-free first frame
//! (±1 LSB) plus a statistical correlation bound, like its SV7 sibling
//! — and, more strongly, by the transcode-identity gate above.

use oxideav_musepack::framing::StreamKind;
use oxideav_musepack::mpc_decode::{decode_mpc_stream, MpcDecodedStream};
use oxideav_musepack::sv7_file_decode::decode_sv7_file;
use oxideav_musepack::sv8_decode::{decode_sv8_stream, Sv8DecodedStream};

/// One fixture's expected `SH` facts plus its gate profile.
struct Expect {
    name: &'static str,
    channels: u8,
    max_band: u8,
    block_power: u8,
    sample_count: u64,
    audio_packets: u64,
    frames: u64,
    /// The SV7 sibling fixture for the §3.6 identity gate (transcodes
    /// only).
    sv7_sibling: Option<&'static str>,
    /// Whether the per-sample oracle gate applies (false for the CNS
    /// stream — see the module docs).
    oracle_per_sample: bool,
}

const CORPUS: [Expect; 7] = [
    Expect {
        name: "stereo-sine-partial-last-frame",
        channels: 2,
        max_band: 28,
        block_power: 3,
        sample_count: 22050,
        audio_packets: 1,
        frames: 20,
        sv7_sibling: Some("stereo-sine-partial-last-frame"),
        oracle_per_sample: true,
    },
    Expect {
        name: "exact-multiple-16-frames",
        channels: 2,
        max_band: 28,
        block_power: 3,
        sample_count: 18432,
        audio_packets: 1,
        frames: 16,
        sv7_sibling: Some("exact-multiple-16-frames"),
        oracle_per_sample: true,
    },
    Expect {
        name: "silence-then-tone-partial",
        channels: 2,
        max_band: 28,
        block_power: 3,
        // 15 full frames + a 360-sample tail (the SV7 gapless total).
        sample_count: 17640,
        audio_packets: 1,
        frames: 16,
        sv7_sibling: Some("silence-then-tone-partial"),
        oracle_per_sample: true,
    },
    Expect {
        name: "stereo-sine-xtreme-quality",
        channels: 2,
        max_band: 31,
        block_power: 3,
        sample_count: 22050,
        audio_packets: 1,
        frames: 20,
        sv7_sibling: Some("stereo-sine-xtreme-quality"),
        oracle_per_sample: true,
    },
    Expect {
        name: "cns-pns",
        channels: 2,
        max_band: 28,
        block_power: 3,
        sample_count: 22050,
        audio_packets: 1,
        frames: 20,
        sv7_sibling: Some("cns-pns"),
        oracle_per_sample: false,
    },
    Expect {
        name: "mono-sine-standard",
        channels: 1,
        max_band: 28,
        block_power: 3,
        sample_count: 22050,
        audio_packets: 1,
        frames: 20,
        sv7_sibling: None,
        oracle_per_sample: true,
    },
    Expect {
        name: "stereo-sine-two-packets",
        channels: 2,
        max_band: 28,
        block_power: 3,
        sample_count: 88200,
        audio_packets: 2,
        frames: 77,
        sv7_sibling: None,
        oracle_per_sample: true,
    },
];

fn fixture_bytes(gen: &str, name: &str, file: &str) -> Vec<u8> {
    let path = format!(
        "{}/tests/fixtures/{gen}/{name}/{file}",
        env!("CARGO_MANIFEST_DIR")
    );
    std::fs::read(&path).unwrap_or_else(|e| panic!("read {path}: {e}"))
}

fn oracle_s16(gen: &str, name: &str) -> Vec<i16> {
    fixture_bytes(gen, name, "expected.pcm")
        .chunks_exact(2)
        .map(|c| i16::from_le_bytes([c[0], c[1]]))
        .collect()
}

fn decode(name: &str) -> Sv8DecodedStream {
    let bytes = fixture_bytes("sv8", name, "input.mpc");
    decode_sv8_stream(&bytes).unwrap_or_else(|e| panic!("{name}: decode failed: {e:?}"))
}

#[test]
fn header_and_stream_structure_match_the_corpus() {
    for e in &CORPUS {
        let out = decode(e.name);
        assert_eq!(out.header.channels, e.channels, "{}: channels", e.name);
        assert_eq!(out.header.max_band, e.max_band, "{}: max_band", e.name);
        assert_eq!(
            out.header.block_power, e.block_power,
            "{}: block_power",
            e.name
        );
        assert_eq!(
            out.header.sample_count, e.sample_count,
            "{}: sample_count",
            e.name
        );
        assert_eq!(out.audio_packets, e.audio_packets, "{}: APs", e.name);
        assert_eq!(out.frames_decoded, e.frames, "{}: frames", e.name);
        assert_eq!(
            out.pcm.len() as u64,
            e.sample_count * u64::from(e.channels),
            "{}: gapless-trimmed PCM length",
            e.name
        );
    }
}

#[test]
fn non_cns_streams_match_the_oracle_within_one_lsb() {
    for e in CORPUS.iter().filter(|e| e.oracle_per_sample) {
        let ours = decode(e.name).pcm_s16();
        let oracle = oracle_s16("sv8", e.name);
        // The oracle emits untrimmed full frames (and possibly a flush
        // frame); our gapless-trimmed output is a prefix of it.
        assert!(
            oracle.len() >= ours.len(),
            "{}: oracle shorter than decode",
            e.name
        );
        let mut exact = 0usize;
        for (i, (&a, &b)) in ours.iter().zip(oracle.iter()).enumerate() {
            let d = (i32::from(a) - i32::from(b)).abs();
            assert!(d <= 1, "{}: sample {i}: {a} vs oracle {b}", e.name);
            if d == 0 {
                exact += 1;
            }
        }
        let ratio = exact as f64 / ours.len() as f64;
        assert!(
            ratio >= 0.70,
            "{}: only {:.1}% bit-exact",
            e.name,
            100.0 * ratio
        );
    }
}

#[test]
fn transcoded_streams_decode_identical_to_their_sv7_siblings() {
    // §3.6 lossless SV7↔SV8: identical quantised payload ⇒ identical
    // reconstruction. Both pipelines share the reconstruction and
    // synthesis code paths, so the f64 PCM must match bit-for-bit —
    // including the CNS stream (same PRNG draw order).
    for e in CORPUS.iter().filter(|e| e.sv7_sibling.is_some()) {
        let sv7_name = e.sv7_sibling.unwrap();
        let sv7 = decode_sv7_file(&fixture_bytes("sv7", sv7_name, "input.mpc"))
            .unwrap_or_else(|err| panic!("{sv7_name}: SV7 decode failed: {err:?}"));
        let sv8 = decode(e.name);
        assert_eq!(
            sv7.pcm.len(),
            sv8.pcm.len(),
            "{}: trimmed lengths differ",
            e.name
        );
        for (i, (&a, &b)) in sv7.pcm.iter().zip(sv8.pcm.iter()).enumerate() {
            assert!(
                a == b,
                "{}: f64 sample {i} differs: SV7 {a} vs SV8 {b}",
                e.name
            );
        }
    }
}

#[test]
fn cns_stream_matches_oracle_on_the_noise_free_frame_and_statistically() {
    // Frame 0 of the CNS stream carries no noise bands: it must meet
    // the same ±1 LSB gate as the rest of the corpus. The noise-bearing
    // remainder is gated statistically (the oracle's noise waveform is
    // not reproducible from the staged generator facts — filed docs
    // gap; see tests/sv7_cns_corpus.rs for the SV7-side analysis).
    let ours = decode("cns-pns").pcm_s16();
    let oracle = oracle_s16("sv8", "cns-pns");
    let frame0 = 2 * 1152;
    for i in 0..frame0 {
        let d = (i32::from(ours[i]) - i32::from(oracle[i])).abs();
        assert!(d <= 1, "frame-0 sample {i}: {} vs {}", ours[i], oracle[i]);
    }
    // Global correlation over the whole trimmed run: the decoded noise
    // has the right per-band per-granule energy (SCF layer) even though
    // the waveform differs, so correlation stays high.
    let n = ours.len().min(oracle.len());
    let (mut sxy, mut sxx, mut syy) = (0f64, 0f64, 0f64);
    for i in 0..n {
        let a = f64::from(ours[i]);
        let b = f64::from(oracle[i]);
        sxy += a * b;
        sxx += a * a;
        syy += b * b;
    }
    let corr = sxy / (sxx.sqrt() * syy.sqrt());
    assert!(corr > 0.7, "global corr {corr:.4} too low");
}

#[test]
fn magic_dispatch_routes_sv8_to_the_packet_decoder() {
    let bytes = fixture_bytes("sv8", "stereo-sine-partial-last-frame", "input.mpc");
    let out = decode_mpc_stream(&bytes).expect("dispatch decode");
    assert_eq!(out.kind(), StreamKind::Sv8);
    assert_eq!(out.channels(), 2);
    assert_eq!(out.sample_rate_hz(), Some(44100));
    match out {
        MpcDecodedStream::Sv8(s) => {
            assert_eq!(s.frames_decoded, 20);
            assert_eq!(s.pcm.len(), 2 * 22050);
        }
        MpcDecodedStream::Sv7(_) => panic!("expected SV8"),
    }
}

#[test]
fn mono_stream_output_is_single_channel() {
    let out = decode("mono-sine-standard");
    assert_eq!(out.header.channels, 1);
    assert_eq!(out.pcm.len(), 22050, "one value per sample");
    // The mono content is a real sine — assert it is not silence.
    assert!(out.pcm.iter().any(|&s| s.abs() > 100.0));
}

#[test]
fn two_packet_stream_chains_key_frames_at_packet_boundaries() {
    // 88200 samples = 77 frames across APs of 64 (block_power 3): the
    // second packet must restart with a key frame + SCF reset; a wrong
    // boundary treatment desynchronises its entropy decode within one
    // band, so mere success plus the oracle gate pins the behaviour.
    let out = decode("stereo-sine-two-packets");
    assert_eq!(out.header.frames_per_audio_packet(), 64);
    assert_eq!(out.audio_packets, 2);
    assert_eq!(out.frames_decoded, 77);
    assert_eq!(out.pcm.len(), 2 * 88200);
}
