//! Deterministic hostile-input robustness gates over the SV7/SV8
//! frame readers — the always-on (CI) companion to the `fuzz/`
//! libFuzzer harness (`sv7_file_decode` / `sv8_stream_decode` /
//! `sv8_seek_index` targets, run out-of-band on nightly).
//!
//! Every gate feeds adversarial bytes to a whole-input entry point
//! and requires a *return* — `Ok` or a structured `Error` — promptly
//! and without panic. Inputs are derived from the staged corpus by a
//! fixed-seed LCG (bit flips, byte stomps), by truncation, and by
//! construction (a CRC-valid `SH` promising a 2^40-sample timeline —
//! the r450 drain-bound regression).

use oxideav_musepack::sv7_file_decode::decode_sv7_file;
use oxideav_musepack::sv8_decode::decode_sv8_stream;
use oxideav_musepack::sv8_file_encode::{sh_payload, write_packet};
use oxideav_musepack::sv8_seek::{decode_sv8_from_entry, SeekTableFields, Sv8SeekIndex};
use oxideav_musepack::Error;

const SV7_FIXTURES: &[&str] = &[
    "cns-pns",
    "exact-multiple-16-frames",
    "silence-then-tone-partial",
    "stereo-sine-partial-last-frame",
    "stereo-sine-xtreme-quality",
];
const SV8_FIXTURES: &[&str] = &[
    "cns-pns",
    "exact-multiple-16-frames",
    "mono-sine-standard",
    "silence-then-tone-partial",
    "stereo-sine-partial-last-frame",
    "stereo-sine-two-packets",
    "stereo-sine-xtreme-quality",
];

fn fixture(gen: &str, name: &str) -> Vec<u8> {
    let path = format!(
        "{}/tests/fixtures/{gen}/{name}/input.mpc",
        env!("CARGO_MANIFEST_DIR")
    );
    std::fs::read(&path).unwrap_or_else(|e| panic!("read {path}: {e}"))
}

/// Minimal fixed-seed LCG (Lehmer-style 64-bit) so the mutation set
/// is identical on every run and platform.
struct Lcg(u64);
impl Lcg {
    fn next(&mut self) -> u64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        self.0 >> 16
    }
}

fn sv8_all_entries(bytes: &[u8]) {
    if let Ok(Some(index)) = Sv8SeekIndex::from_seek_packets(bytes) {
        for entry in 0..index.positions.len().min(4) {
            let _ = decode_sv8_from_entry(bytes, &index, entry);
        }
    }
    let _ = Sv8SeekIndex::from_packet_walk(bytes);
}

/// Point mutations of every corpus stream: 32 single-byte stomps per
/// fixture (the libFuzzer harness goes deep; this gate stays cheap
/// enough for a debug-profile CI run), all decode paths must return
/// without panic.
#[test]
fn corpus_point_mutations_return_structured_results() {
    let mut rng = Lcg(0x05ee_d450);
    for name in SV7_FIXTURES {
        let base = fixture("sv7", name);
        for _ in 0..32 {
            let mut m = base.clone();
            let at = (rng.next() as usize) % m.len();
            m[at] = rng.next() as u8;
            let _ = decode_sv7_file(&m);
        }
    }
    for name in SV8_FIXTURES {
        let base = fixture("sv8", name);
        for i in 0..32 {
            let mut m = base.clone();
            let at = (rng.next() as usize) % m.len();
            m[at] = rng.next() as u8;
            let _ = decode_sv8_stream(&m);
            if i % 3 == 0 {
                sv8_all_entries(&m);
            }
        }
    }
}

/// Truncations at 12 pseudo-random lengths per fixture (plus the
/// first 16 byte lengths exactly) — mid-header, mid-packet, mid-frame.
#[test]
fn corpus_truncations_return_structured_results() {
    let mut rng = Lcg(0x7A_u64 ^ 0x450);
    for name in SV7_FIXTURES {
        let base = fixture("sv7", name);
        for cut in (0..16).chain((0..12).map(|_| (rng.next() as usize) % base.len())) {
            let _ = decode_sv7_file(&base[..cut]);
        }
    }
    for name in SV8_FIXTURES {
        let base = fixture("sv8", name);
        for cut in (0..16).chain((0..12).map(|_| (rng.next() as usize) % base.len())) {
            let _ = decode_sv8_stream(&base[..cut]);
            sv8_all_entries(&base[..cut]);
        }
    }
}

/// Pure-noise buffers wearing the right magics.
#[test]
fn random_buffers_with_valid_magic_return_structured_results() {
    let mut rng = Lcg(0x0bad_c0de);
    for _ in 0..64 {
        let len = 8 + (rng.next() as usize) % 4096;
        let mut buf: Vec<u8> = (0..len).map(|_| rng.next() as u8).collect();
        buf[..4].copy_from_slice(b"MPCK");
        let _ = decode_sv8_stream(&buf);
        sv8_all_entries(&buf);
        buf[..3].copy_from_slice(b"MP+");
        buf[3] = 0x07;
        let _ = decode_sv7_file(&buf);
    }
}

/// r450 drain-bound regression: a tiny, CRC-valid stream declaring a
/// 2^40-sample timeline with no audio behind it must fail loud
/// (truncated), not zero-drain a terabyte-scale window.
#[test]
fn sv8_giant_sample_count_with_no_audio_fails_loud() {
    let mut buf = b"MPCK".to_vec();
    let sh = sh_payload(1 << 40, 0, 0, 31, 2, true, 3).expect("sh");
    write_packet(&mut buf, *b"SH", &sh);
    write_packet(&mut buf, *b"SE", &[]);
    assert_eq!(decode_sv8_stream(&buf), Err(Error::UnexpectedEof));
}

/// Hostile `ST` payloads at the raw-parser level: promised entry
/// counts and Golomb runs beyond what the payload carries are
/// rejected without allocation or hang.
#[test]
fn hostile_seek_tables_are_rejected() {
    let mut rng = Lcg(0x5eec);
    for _ in 0..2000 {
        let len = 1 + (rng.next() as usize) % 64;
        let buf: Vec<u8> = (0..len).map(|_| rng.next() as u8).collect();
        let _ = SeekTableFields::parse(&buf);
    }
    // All-continuation varint: overlong.
    assert_eq!(
        SeekTableFields::parse(&[0xFF; 16]),
        Err(Error::VarintTooLong)
    );
}
