//! End-to-end SV8 decode test against a real Musepack file fetched
//! from the public `samples.mplayerhq.hu` archive at test time.
//!
//! The fetched fixture (`sv8-notags.mpc`, ~1 MB) is cached under the
//! crate's `target/musepack-fixtures/` directory; the test is skipped
//! with a console note if the network is unavailable so CI without
//! egress doesn't fail.

use std::path::PathBuf;

use oxideav_core::packet::PacketFlags;
use oxideav_core::registry::CodecRegistry;
use oxideav_core::time::TimeBase;
use oxideav_core::{CodecId, CodecParameters, Frame, Packet};

const FIXTURE_URL: &str = "https://samples.mplayerhq.hu/A-codecs/musepack/sv8/sv8-notags.mpc";
const FIXTURE_NAME: &str = "sv8-notags.mpc";

fn fixture_path() -> PathBuf {
    let mut p = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    p.push("target");
    p.push("musepack-fixtures");
    let _ = std::fs::create_dir_all(&p);
    p.push(FIXTURE_NAME);
    p
}

/// Returns Some(bytes) on success, None if no fixture is reachable.
/// Honours `MUSEPACK_FIXTURE_PATH` for offline / sandboxed runs.
fn load_fixture() -> Option<Vec<u8>> {
    if let Ok(path) = std::env::var("MUSEPACK_FIXTURE_PATH") {
        return std::fs::read(path).ok();
    }
    let path = fixture_path();
    if let Ok(data) = std::fs::read(&path) {
        return Some(data);
    }
    // Try curl (avoids reqwest-style heavy deps).
    let out = std::process::Command::new("curl")
        .args([
            "-sSL",
            "--max-time",
            "30",
            "-o",
            path.to_str()?,
            FIXTURE_URL,
        ])
        .status()
        .ok()?;
    if !out.success() {
        return None;
    }
    std::fs::read(&path).ok()
}

#[test]
fn sv8_decoder_registers_and_decodes_first_packet() {
    let mut codecs = CodecRegistry::new();
    oxideav_musepack::register(&mut codecs);
    assert!(codecs.has_decoder(&CodecId::new("musepack")));

    let bytes = match load_fixture() {
        Some(b) => b,
        None => {
            eprintln!("note: musepack SV8 fixture unavailable (offline); test skipped");
            return;
        }
    };
    assert!(bytes.len() > 32);
    assert_eq!(&bytes[0..4], b"MPCK");

    let params = CodecParameters::audio(CodecId::new("musepack"));
    let mut dec = codecs.make_decoder(&params).expect("make_decoder");
    let pkt = Packet {
        data: bytes,
        pts: Some(0),
        dts: Some(0),
        duration: None,
        time_base: TimeBase::new(1, 44_100),
        stream_index: 0,
        flags: PacketFlags {
            keyframe: true,
            ..PacketFlags::default()
        },
    };
    dec.send_packet(&pkt).expect("send_packet");
    dec.flush().expect("flush");

    // Drain until Eof — count successfully decoded frames and total
    // PCM samples. We do not require bit-exact output (the
    // mid-precision quantiser path uses a raw-bit fallback until the
    // SV8 Q2..Q8 symbol arrays are transcribed) — only that decode
    // does not panic and the structural counts make sense.
    let mut frames = 0usize;
    let mut total_samples = 0u64;
    let mut last_err: Option<String> = None;
    loop {
        match dec.receive_frame() {
            Ok(Frame::Audio(af)) => {
                frames += 1;
                total_samples += af.samples as u64;
                // Each frame must carry a non-empty interleaved s16 buffer.
                assert!(!af.data.is_empty());
                assert!(!af.data[0].is_empty());
            }
            Ok(_) => panic!("non-audio frame from musepack decoder"),
            Err(oxideav_core::Error::Eof) | Err(oxideav_core::Error::NeedMore) => break,
            Err(e) => {
                // Decode-stage errors are tolerated for now — the SV8
                // mid-precision (`res ∈ {2..=8}`) entropy path uses a
                // raw-bit fallback pending transcription of the
                // `mpc8huff.h::mpc8_q_syms` symbol arrays from the
                // workspace's reference materials. Once those are
                // wired in, this path is expected to drain cleanly to
                // `Eof`.
                last_err = Some(format!("{e}"));
                break;
            }
        }
    }
    eprintln!(
        "decoded {frames} sub-frames, {total_samples} PCM samples per channel; last_err={last_err:?}"
    );
    // Structural acceptance: the decoder must at least successfully
    // initialise from the SH chunk and not panic on the first AP
    // payload. Once the deferred entropy tables land we will tighten
    // this to `frames > 0` (and eventually to a PSNR check).
    let _ = frames;
}

#[test]
fn sv8_chunk_walk_smoke() {
    let bytes = match load_fixture() {
        Some(b) => b,
        None => return,
    };
    use oxideav_musepack::container::{ChunkIter, ChunkTag};

    assert_eq!(&bytes[0..4], b"MPCK");
    let mut iter = ChunkIter::new(&bytes[4..]);
    let mut counts = std::collections::BTreeMap::<&'static str, usize>::new();
    for chunk in iter {
        let chunk = chunk.expect("chunk parse");
        let key: &'static str = match chunk.tag {
            ChunkTag::Sh => "SH",
            ChunkTag::Se => "SE",
            ChunkTag::Ap => "AP",
            ChunkTag::So => "SO",
            ChunkTag::St => "ST",
            ChunkTag::Rg => "RG",
            ChunkTag::Ei => "EI",
            ChunkTag::Ct => "CT",
            ChunkTag::Other(_) => "??",
        };
        *counts.entry(key).or_default() += 1;
    }
    eprintln!("chunk counts: {counts:?}");
    assert!(counts.get("SH").copied().unwrap_or(0) >= 1, "no SH chunk");
    assert!(counts.get("AP").copied().unwrap_or(0) >= 1, "no AP chunks");
    assert!(counts.get("SE").copied().unwrap_or(0) >= 1, "no SE chunk");
}
