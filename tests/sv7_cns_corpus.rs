//! CNS / PNS fixture conformance gates (`tests/fixtures/sv7/cns-pns/`).
//!
//! The staged fixture (docs commit `0f1b6a2`,
//! `docs/audio/musepack/fixtures/cns-pns/`) is the first corpus stream
//! that **uses Clear Noise Substitution**: mppenc 1.16 `--pns 1.0` on a
//! deterministic tonal-masker + quiet high-passed-noise source. It
//! exists to pin **CNS-SCF participation** — noise bands (`Res == -1`)
//! carry no spectral samples but still take part in the SCFI + DSCF
//! scalefactor passes (spec §5.2/§5.3 + erratum E1) — and the
//! stream-level PNS flag in the version byte (`MP+ 0x17`).
//!
//! What these gates pin, and what they deliberately do not:
//!
//! - **Wire syntax with CNS bands** is pinned *exactly*: all 20 frames
//!   decode budget-exact against their 20-bit prefixes, which is only
//!   possible if CNS bands are visited by the SCFI and DSCF passes and
//!   read **zero** sample-pass bits (215 CNS band-instances across 18
//!   frames — any mis-walk desynchronises every later read).
//! - **PCM around the noise** is pinned where the noise cannot reach:
//!   frame 0 carries no CNS band and no synthesis history, and matches
//!   the FFmpeg oracle within ±1 LSB like the rest of the corpus.
//! - **The noise waveform itself is NOT sample-comparable.** The
//!   fixture's `expected.pcm` comes from the FFmpeg `mpc7` black-box
//!   oracle, and its noise-band waveform is *not reproducible from the
//!   staged CNS generator facts*: a matched-filter search (synthesising
//!   a single CNS band's contribution from the staged two-LFSR PRNG at
//!   every stream offset in `0..30000`, strides 1..=40, 1/2/4-word
//!   groupings, plus whole-stream / per-frame / per-band reset
//!   hypotheses) finds no correlation above the noise floor (~0.07)
//!   anywhere, while the same search against this crate's own decode
//!   spikes at the true offset (0.28). The oracle's noise residual is
//!   also ~2× the staged generator's `C[0]`-scaled amplitude. The
//!   oracle evidently synthesises CNS noise with a different generator;
//!   per the clean-room wall its source cannot be consulted, so this
//!   suite gates the noise **statistically** (documented measured
//!   values) instead of per-sample and the exact-match corpus gate
//!   excludes this fixture.

use oxideav_musepack::cns::CnsPrng;
use oxideav_musepack::framing::{SV7Header, SV7_VERSION_PNS_FLAG};
use oxideav_musepack::huffman::Sv7BitReader;
use oxideav_musepack::sv7_band_decode::{
    band_type_uses_context_selector, decode_sv7_band, SAMPLES_PER_BAND,
};
use oxideav_musepack::sv7_band_header::decode_res_header_grounded;
use oxideav_musepack::sv7_file_decode::decode_sv7_file;
use oxideav_musepack::sv7_header::Sv7HeaderFields;
use oxideav_musepack::sv7_scf_decode::{decode_sv7_band_dscf, decode_sv7_scfi};
use oxideav_musepack::sv7_stereo_frame::Sv7ScfMemory;
use oxideav_musepack::sv7_word_swap::word_swap_sv7_body;

fn fixture_bytes(name: &str, file: &str) -> Vec<u8> {
    let path = format!(
        "{}/tests/fixtures/sv7/{name}/{file}",
        env!("CARGO_MANIFEST_DIR")
    );
    std::fs::read(&path).unwrap_or_else(|e| panic!("read {path}: {e}"))
}

fn oracle_s16(name: &str) -> Vec<i16> {
    fixture_bytes(name, "expected.pcm")
        .chunks_exact(2)
        .map(|c| i16::from_le_bytes([c[0], c[1]]))
        .collect()
}

/// §1 header conformance: the fixture-notes-pinned fields, including
/// the PNS version byte (`MP+ 0x17`) that the rest of the corpus lacks.
#[test]
fn cns_header_parses_to_pinned_fields_with_pns_flag() {
    let bytes = fixture_bytes("cns-pns", "input.mpc");
    assert_eq!(&bytes[..3], b"MP+");
    assert_eq!(bytes[3], 0x17, "PNS version byte");

    let magic = SV7Header::parse_magic(&bytes).unwrap();
    assert!(magic.pns());
    assert_eq!(magic.stream_version(), 7);
    assert_eq!(magic.version_byte & SV7_VERSION_PNS_FLAG, 0x10);

    let h = Sv7HeaderFields::parse(&bytes).unwrap();
    assert!(h.pns, "header surfaces the PNS flag");
    assert_eq!(h.version_byte(), 0x17);
    assert_eq!(h.frame_count, 20);
    assert_eq!(h.max_band, 28);
    assert_eq!(h.profile, 5, "mppenc records a lower profile under PNS");
    assert_eq!(h.last_frame_samples, 162);
    assert!(h.mid_side);
    assert!(h.true_gapless);
    assert!(h.fast_seek);
    assert!(!h.intensity_stereo);
    assert_eq!(h.encoder_version, 116);
    assert_eq!(h.sample_rate_hz(), Some(44100));
    assert_eq!(h.channels(), 2);
    assert_eq!(h.effective_total_samples(), 19 * 1152 + 162);
}

/// The four non-PNS corpus streams all carry `MP+ 0x07` — the flag is
/// specific to the PNS encode.
#[test]
fn non_pns_corpus_streams_do_not_carry_the_flag() {
    for name in [
        "stereo-sine-partial-last-frame",
        "exact-multiple-16-frames",
        "silence-then-tone-partial",
        "stereo-sine-xtreme-quality",
    ] {
        let bytes = fixture_bytes(name, "input.mpc");
        assert_eq!(bytes[3], 0x07, "{name}: version byte");
        assert!(!Sv7HeaderFields::parse(&bytes).unwrap().pns, "{name}");
    }
}

/// Whole-file decode with CNS bands: every one of the 20 frames must
/// consume exactly its 20-bit-prefix bit budget (verified internally by
/// `decode_sv7_file`, which fails loudly on any mismatch) — the wire
/// proof that CNS bands participate in SCFI + DSCF and read no sample
/// bits — and the 11-bit trailer and gapless trim must match.
#[test]
fn cns_stream_decodes_with_exact_frame_budgets() {
    let bytes = fixture_bytes("cns-pns", "input.mpc");
    let out = decode_sv7_file(&bytes).expect("whole-file decode");
    assert_eq!(out.frames_decoded, 20);
    assert_eq!(out.stream_last_frame_samples, Some(162));
    assert_eq!(out.pcm.len(), 2 * (19 * 1152 + 162));
}

/// Walk the frame bodies and census the CNS bands: the fixture-notes
/// facts (18 of 20 frames use CNS; 215 (band, channel) instances; band
/// indices within 8..27) plus the structural facts every instance
/// exhibits on this stream — the paired channel is empty and the band
/// is M/S-flagged.
#[test]
fn cns_band_census_matches_fixture_notes() {
    let bytes = fixture_bytes("cns-pns", "input.mpc");
    let header = Sv7HeaderFields::parse(&bytes).unwrap();
    let mut swapped = word_swap_sv7_body(&bytes);
    swapped.extend_from_slice(&[0u8; 4]);
    let mut r = Sv7BitReader::new(&swapped);
    // Skip the 200-bit fixed header.
    for _ in 0..12 {
        r.read_bits(16).unwrap();
    }
    r.read_bits(8).unwrap();

    let mut scf = Sv7ScfMemory::new();
    let mut cns = CnsPrng::new();
    let mut cns_frames = 0u32;
    let mut cns_instances = 0u32;
    let mut min_band = usize::MAX;
    let mut max_band_seen = 0usize;
    let mut frames_with_cns = Vec::new();

    for f in 0..header.frame_count {
        // 20-bit prefix.
        r.read_bits(16).unwrap();
        r.read_bits(4).unwrap();
        let hdr = decode_res_header_grounded(&mut r, header.max_band, 2, header.mid_side).unwrap();
        let res: Vec<[i8; 2]> = hdr.iter().map(|b| b.res).collect();
        let ms: Vec<bool> = hdr.iter().map(|b| b.ms_flag.unwrap_or(false)).collect();

        let mut frame_cns = 0u32;
        for (b, rr) in res.iter().enumerate() {
            for ch in 0..2 {
                if rr[ch] == -1 {
                    frame_cns += 1;
                    min_band = min_band.min(b);
                    max_band_seen = max_band_seen.max(b);
                    assert_eq!(
                        rr[1 - ch],
                        0,
                        "frame {f} band {b}: CNS pairs with an empty channel"
                    );
                    assert!(ms[b], "frame {f} band {b}: CNS band is M/S-flagged");
                }
            }
        }
        if frame_cns > 0 {
            cns_frames += 1;
            frames_with_cns.push(f);
        }
        cns_instances += frame_cns;

        // Walk the remaining passes to stay bit-synchronised.
        let mut scfi = vec![[0u8; 2]; res.len()];
        for (b, rr) in res.iter().enumerate() {
            for (ch, slot) in scfi[b].iter_mut().enumerate() {
                if rr[ch] != 0 {
                    *slot = decode_sv7_scfi(&mut r).unwrap();
                }
            }
        }
        for (b, rr) in res.iter().enumerate() {
            for ch in 0..2 {
                if rr[ch] != 0 {
                    let bs =
                        decode_sv7_band_dscf(&mut r, scfi[b][ch], scf.reference(ch, b)).unwrap();
                    scf.update(ch, b, bs.indices[2]);
                }
            }
        }
        let mut lv = [0i32; SAMPLES_PER_BAND];
        for rr in res.iter() {
            for &bt in rr.iter() {
                if bt == 0 {
                    continue;
                }
                let ctx = if band_type_uses_context_selector(bt) {
                    (r.read_bits(1).unwrap() & 1) as usize
                } else {
                    0
                };
                decode_sv7_band(&mut r, bt, &mut cns, ctx, &mut lv).unwrap();
            }
        }
    }

    assert_eq!(cns_frames, 18, "18 of 20 frames use CNS");
    assert_eq!(cns_instances, 215, "215 (band, channel) CNS instances");
    assert_eq!(
        frames_with_cns,
        (1..=18).collect::<Vec<_>>(),
        "only the cold-start frame 0 and trailing frame 19 are CNS-free"
    );
    assert!(
        (8..=27).contains(&min_band) && (8..=27).contains(&max_band_seen),
        "CNS bands live in the upper subbands 8..27 (got {min_band}..{max_band_seen})"
    );
}

/// PCM conformance where the noise cannot reach: frame 0 has no CNS
/// band and no synthesis history behind it, so it must match the
/// oracle within ±1 LSB exactly like the non-PNS corpus.
#[test]
fn cns_frame0_pcm_matches_oracle_within_one_lsb() {
    let bytes = fixture_bytes("cns-pns", "input.mpc");
    let oracle = oracle_s16("cns-pns");
    let ours = decode_sv7_file(&bytes).expect("decode").pcm_s16();
    let frame0 = 2 * 1152;
    let mut exact = 0usize;
    for i in 0..frame0 {
        let err = (i32::from(ours[i]) - i32::from(oracle[i])).abs();
        assert!(err <= 1, "sample {i}: {} vs oracle {}", ours[i], oracle[i]);
        if err == 0 {
            exact += 1;
        }
    }
    assert!(
        exact * 10 >= frame0 * 7,
        "only {exact}/{frame0} frame-0 samples bit-exact"
    );
}

/// Statistical gate over the noise-bearing frames — the fixture's
/// oracle noise waveform is not reproducible from the staged generator
/// facts (see the module docs), so the decode is gated on aggregate
/// agreement instead of per-sample: the decoded PCM must track the
/// oracle's tonal content closely (high global correlation) and the
/// residual must stay in the quiet-noise regime.
#[test]
fn cns_stream_pcm_tracks_oracle_statistically() {
    let bytes = fixture_bytes("cns-pns", "input.mpc");
    let oracle = oracle_s16("cns-pns");
    let ours = decode_sv7_file(&bytes).expect("decode").pcm_s16();
    let n = ours.len().min(oracle.len());

    let mut dot = 0.0f64;
    let mut na = 0.0f64;
    let mut nb = 0.0f64;
    let mut err2 = 0.0f64;
    for i in 0..n {
        let a = f64::from(ours[i]);
        let b = f64::from(oracle[i]);
        dot += a * b;
        na += a * a;
        nb += b * b;
        let e = a - b;
        err2 += e * e;
    }
    let corr = dot / (na.sqrt() * nb.sqrt());
    let rms_err = (err2 / n as f64).sqrt();
    let rms_sig = (nb / n as f64).sqrt();
    println!("corr={corr:.4} rms_err={rms_err:.1} rms_signal={rms_sig:.1}");

    // Measured r405: corr ≈ 0.776, rms_err ≈ 1377 against rms_signal
    // ≈ 2158 (the residual is dominated by the oracle's ~2×-louder,
    // differently-sequenced noise floor). Gate with slack so only a
    // structural regression (broken SCF/CNS wire walk, gain law, M/S)
    // trips it.
    assert!(corr > 0.70, "global correlation collapsed: {corr:.4}");
    assert!(
        rms_err < 0.85 * rms_sig,
        "residual no longer in the noise regime: rms_err={rms_err:.1} rms_signal={rms_sig:.1}"
    );
}
