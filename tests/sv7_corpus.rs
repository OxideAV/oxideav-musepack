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
