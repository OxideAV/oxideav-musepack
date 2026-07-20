//! SV7 corpus conformance gates.
//!
//! Runs the crate's SV7 file layer against the four independently
//! encoded fixture streams under `tests/fixtures/sv7/` (mppenc 1.16
//! as a black-box producer, FFmpeg's `mpc7` decoder as a black-box
//! oracle — see `tests/fixtures/sv7/README.md` for provenance).
//!
//! These are the external-validation gates the r385 file layer lacked:
//! every fact asserted here is pinned by the corpus itself, not by the
//! crate's own writer.

use oxideav_musepack::mpc_decode::{decode_mpc_stream, MpcDecodedStream};
use oxideav_musepack::sv7_file_decode::decode_sv7_file;
use oxideav_musepack::sv7_header::Sv7HeaderFields;

/// One fixture's expected §1 header facts, from the corpus notes.
struct Expect {
    name: &'static str,
    frames: u32,
    max_band: u8,
    profile: u8,
    last_frame_samples: u16,
}

const CORPUS: [Expect; 4] = [
    Expect {
        name: "stereo-sine-partial-last-frame",
        frames: 20,
        max_band: 28,
        profile: 10,
        last_frame_samples: 162,
    },
    Expect {
        name: "exact-multiple-16-frames",
        frames: 16,
        max_band: 28,
        profile: 10,
        last_frame_samples: 1152,
    },
    Expect {
        name: "silence-then-tone-partial",
        frames: 16,
        max_band: 28,
        profile: 10,
        last_frame_samples: 360,
    },
    Expect {
        name: "stereo-sine-xtreme-quality",
        frames: 20,
        max_band: 31,
        profile: 11,
        last_frame_samples: 162,
    },
];

fn fixture_bytes(name: &str, file: &str) -> Vec<u8> {
    let path = format!(
        "{}/tests/fixtures/sv7/{name}/{file}",
        env!("CARGO_MANIFEST_DIR")
    );
    std::fs::read(&path).unwrap_or_else(|e| panic!("read {path}: {e}"))
}

/// §1 fixed-header conformance: every independently-encoded stream's
/// header parses to exactly the fields the corpus notes pin — including
/// the two previously-unpinned facts, the bit-200 body boundary (the
/// parse succeeding at all pins the 168-bit field span after the 4-byte
/// prefix) and the 11-bit `last_frame_samples` field at bits 161–171
/// (its full-width value 1152 in `exact-multiple-16-frames` would
/// corrupt neighbouring fields under any off-by-one placement).
#[test]
fn corpus_headers_parse_to_pinned_fields() {
    for e in &CORPUS {
        let bytes = fixture_bytes(e.name, "input.mpc");
        let h = Sv7HeaderFields::parse(&bytes)
            .unwrap_or_else(|err| panic!("{}: header parse failed: {err}", e.name));
        assert_eq!(h.frame_count, e.frames, "{}: frame_count", e.name);
        assert_eq!(h.max_band, e.max_band, "{}: max_band", e.name);
        assert_eq!(h.profile, e.profile, "{}: profile", e.name);
        assert_eq!(
            h.last_frame_samples, e.last_frame_samples,
            "{}: last_frame_samples",
            e.name
        );
        // Shared facts across the whole corpus.
        assert!(h.mid_side, "{}: stream M/S", e.name);
        assert!(h.true_gapless, "{}: true-gapless", e.name);
        assert!(h.fast_seek, "{}: fast-seek", e.name);
        assert!(!h.intensity_stereo, "{}: intensity off", e.name);
        assert_eq!(h.encoder_version, 116, "{}: encoder version", e.name);
        assert_eq!(h.sample_rate_hz(), Some(44100), "{}: rate", e.name);
        assert_eq!(h.channels(), 2, "{}: channels", e.name);
    }
}

/// Whole-file decode conformance: every fixture decodes end-to-end —
/// which by construction verifies each frame body against its 20-bit
/// bit-length prefix (any syntax divergence fails loudly) — and the
/// in-stream 11-bit trailer equals header field 14.
#[test]
fn corpus_streams_decode_with_exact_frame_budgets() {
    for e in &CORPUS {
        let bytes = fixture_bytes(e.name, "input.mpc");
        let out = decode_sv7_file(&bytes)
            .unwrap_or_else(|err| panic!("{}: whole-file decode failed: {err}", e.name));
        assert_eq!(out.frames_decoded, u64::from(e.frames), "{}", e.name);
        assert_eq!(
            out.stream_last_frame_samples,
            Some(e.last_frame_samples),
            "{}: trailer",
            e.name
        );
        // Gapless-trimmed output length.
        let want = 2 * (u64::from(e.frames - 1) * 1152 + u64::from(e.last_frame_samples));
        assert_eq!(out.pcm.len() as u64, want, "{}: trimmed pcm len", e.name);
    }
}

/// PCM conformance gate against the FFmpeg `mpc7` oracle: every decoded
/// sample within ±1 LSB, and at least 70% bit-exact. (The oracle runs
/// f32 DSP; this crate's synthesis is f64 — a systematic error in any
/// pinned fact (SCF law, M/S undo, dequant, synthesis window) blows far
/// past 1 LSB, so this is a tight gate on the arithmetic.)
#[test]
fn corpus_pcm_matches_oracle_within_one_lsb() {
    for e in &CORPUS {
        let bytes = fixture_bytes(e.name, "input.mpc");
        let oracle: Vec<i16> = fixture_bytes(e.name, "expected.pcm")
            .chunks_exact(2)
            .map(|c| i16::from_le_bytes([c[0], c[1]]))
            .collect();
        let out = decode_sv7_file(&bytes).expect("decode");
        let ours = out.pcm_s16();
        // The oracle is untrimmed; compare over the trimmed length.
        assert!(ours.len() <= oracle.len(), "{}", e.name);
        let mut exact = 0usize;
        for (i, (&a, &b)) in ours.iter().zip(oracle.iter()).enumerate() {
            let err = (i32::from(a) - i32::from(b)).abs();
            assert!(err <= 1, "{}: sample {i}: {a} vs oracle {b}", e.name);
            if err == 0 {
                exact += 1;
            }
        }
        assert!(
            exact * 10 >= ours.len() * 7,
            "{}: only {exact}/{} samples bit-exact",
            e.name,
            ours.len()
        );
    }
}

/// The magic-dispatched unified entry point routes the corpus streams
/// to the same SV7 decode.
#[test]
fn corpus_streams_decode_through_unified_entry() {
    let e = &CORPUS[0];
    let bytes = fixture_bytes(e.name, "input.mpc");
    let out = decode_mpc_stream(&bytes).expect("decode");
    assert_eq!(out.channels(), 2);
    assert_eq!(out.sample_rate_hz(), Some(44100));
    match out {
        MpcDecodedStream::Sv7(f) => {
            assert_eq!(f, decode_sv7_file(&bytes).unwrap());
        }
        MpcDecodedStream::Sv8(_) => panic!("expected SV7"),
    }
}

/// The `exact-multiple-16-frames` fixture carries mppenc's undeclared
/// **flush frame** after the 11-bit trailer: one more
/// `[20-bit length][body]` unit that decoders must ignore (the oracle
/// emits exactly 16 frames). Prove (a) the whole-file decoder ignores
/// it, and (b) it is nonetheless a syntactically valid SV7 frame that
/// consumes exactly its declared bit budget — a 73rd independent
/// frame-syntax check.
#[test]
fn corpus_flush_frame_after_trailer_is_valid_and_ignored() {
    use oxideav_musepack::huffman::Sv7BitReader;
    use oxideav_musepack::sv7_stream::Sv7StreamDecoder;
    use oxideav_musepack::sv7_word_swap::word_swap_sv7_body;

    let bytes = fixture_bytes("exact-multiple-16-frames", "input.mpc");
    let header = Sv7HeaderFields::parse(&bytes).unwrap();
    assert_eq!(decode_sv7_file(&bytes).unwrap().frames_decoded, 16);

    // Re-walk the framing manually to reach the tail.
    let mut swapped = word_swap_sv7_body(&bytes);
    swapped.extend_from_slice(&[0u8; 4]);
    let mut r = Sv7BitReader::new(&swapped);
    let total = r.bits_remaining();
    let skip = |r: &mut Sv7BitReader<'_>, mut n: u64| {
        while n > 0 {
            let step = n.min(16) as u8;
            r.read_bits(step).unwrap();
            n -= u64::from(step);
        }
    };
    skip(&mut r, 200);
    let read20 = |r: &mut Sv7BitReader<'_>| -> u64 {
        let hi = u64::from(r.read_bits(16).unwrap());
        let lo = u64::from(r.read_bits(4).unwrap());
        (hi << 4) | lo
    };
    // A stream decoder consuming the declared frames keeps the state
    // (SCF memory, PRNG) the flush frame was encoded against.
    let mut dec = Sv7StreamDecoder::from_header(&header).unwrap();
    for _ in 0..16 {
        let len = read20(&mut r);
        let start = total - r.bits_remaining();
        dec.decode_frame(&mut r).unwrap();
        assert_eq!(total - r.bits_remaining() - start, len);
    }
    let trailer = r.read_bits(11).unwrap();
    assert_eq!(trailer, 1152);

    // The tail: one more prefixed, budget-exact, decodable frame.
    let flush_len = read20(&mut r);
    assert!(flush_len > 0);
    let start = total - r.bits_remaining();
    dec.decode_frame(&mut r).expect("flush frame decodes");
    assert_eq!(
        total - r.bits_remaining() - start,
        flush_len,
        "flush frame consumes its declared budget"
    );
    // Nothing but word padding remains.
    let file_bits = (bytes.len() * 8) as u64;
    let pos = total - r.bits_remaining();
    assert!(file_bits - pos < 32, "only padding after the flush frame");
}

/// The oracle PCM length is exactly `frames × 1152` samples per channel
/// (the FFmpeg oracle does not gapless-trim), and the header's derived
/// totals agree with it.
#[test]
fn corpus_oracle_lengths_match_header_totals() {
    for e in &CORPUS {
        let mpc = fixture_bytes(e.name, "input.mpc");
        let pcm = fixture_bytes(e.name, "expected.pcm");
        let h = Sv7HeaderFields::parse(&mpc).unwrap();
        let untrimmed = h.total_samples();
        // s16le, 2 channels interleaved: 4 bytes per per-channel sample.
        assert_eq!(pcm.len() as u64, untrimmed * 4, "{}: oracle len", e.name);
        // The gapless-trimmed total is what a conformant player emits.
        let trimmed = h.effective_total_samples();
        assert_eq!(
            trimmed,
            untrimmed - 1152 + u64::from(e.last_frame_samples),
            "{}: trimmed total",
            e.name
        );
    }
}
