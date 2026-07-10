//! SV7 encoder wire-symmetry gate: re-encoding the parsed structure of
//! every corpus frame reproduces mppenc 1.16's bytes **exactly**.
//!
//! Each fixture is parsed structurally (Res header → SCFI → DSCF →
//! samples, using only the crate's public primitives) into
//! `Sv7EncStereoFrame` values, then re-encoded with the crate's
//! whole-file writer. Byte-for-byte equality proves the encoder's
//! wire-level decisions — the SCFI selector choice, the DSCF
//! delta-vs-escape rule, the §5.1 header delta/escape rule, the 20-bit
//! prefixes and 11-bit trailer, the word swap — all coincide with the
//! independent reference encoder's on real streams, not just on the
//! crate's own round-trips.
//!
//! Three fixtures reproduce in full; `exact-multiple-16-frames`
//! reproduces through the trailer (the original then carries mppenc's
//! undeclared flush frame, which this writer intentionally does not
//! emit — see `sv7_file_encode`), so it is compared up to the last
//! whole 32-bit word before the divergent trailer/flush word; the
//! CNS-bearing `cns-pns` stream reproduces byte-exact through its full
//! content (the original carries one dead all-zero tail word).

use oxideav_musepack::huffman::{
    decode as vlc, sv7_q1_ctx, sv7_q2_ctx, sv7_q3_ctx, sv7_q4_ctx, sv7_q5_ctx, sv7_q6_ctx,
    sv7_q7_ctx, Sv7BitReader,
};
use oxideav_musepack::sv7_band_decode::{
    band_type_uses_context_selector, unpack_grouped2_value, unpack_grouped3_value,
};
use oxideav_musepack::sv7_band_header::decode_res_header_grounded;
use oxideav_musepack::sv7_file_encode::{encode_sv7_file_with_version, Sv7EncStereoFrame};
use oxideav_musepack::sv7_frame_encode::Sv7EncBand;
use oxideav_musepack::sv7_header::Sv7HeaderFields;
use oxideav_musepack::sv7_scf_decode::{decode_sv7_band_dscf, decode_sv7_scfi};
use oxideav_musepack::sv7_word_swap::word_swap_sv7_body;

fn fixture_bytes(name: &str) -> Vec<u8> {
    let path = format!(
        "{}/tests/fixtures/sv7/{name}/input.mpc",
        env!("CARGO_MANIFEST_DIR")
    );
    std::fs::read(&path).unwrap_or_else(|e| panic!("read {path}: {e}"))
}

fn skip_bits(reader: &mut Sv7BitReader<'_>, mut n: u64) {
    while n > 0 {
        let step = n.min(16) as u8;
        reader.read_bits(step).unwrap();
        n -= u64::from(step);
    }
}

/// Structural parse of one frame body into encoder input, mirroring the
/// corpus-pinned four-pass layout with the crate's public primitives.
fn parse_frame(
    r: &mut Sv7BitReader<'_>,
    h: &Sv7HeaderFields,
    scf_mem: &mut [[i32; 32]; 2],
) -> Sv7EncStereoFrame {
    let bands = decode_res_header_grounded(r, h.max_band, 2, h.mid_side).unwrap();
    let n = bands.len();
    let res: Vec<[i8; 2]> = bands.iter().map(|b| b.res).collect();
    let ms_flags: Vec<bool> = bands.iter().map(|b| b.ms_flag.unwrap_or(false)).collect();

    let mut scfi = vec![[0u8; 2]; n];
    for (b, re) in res.iter().enumerate() {
        for ch in 0..2 {
            if re[ch] != 0 {
                scfi[b][ch] = decode_sv7_scfi(r).unwrap();
            }
        }
    }
    let mut scf = vec![[[0i32; 3]; 2]; n];
    for (b, re) in res.iter().enumerate() {
        for ch in 0..2 {
            if re[ch] != 0 {
                let s = decode_sv7_band_dscf(r, scfi[b][ch], scf_mem[ch][b]).unwrap();
                scf[b][ch] = s.indices;
                scf_mem[ch][b] = s.indices[2];
            }
        }
    }
    let mut chans: [Vec<Sv7EncBand>; 2] = [Vec::new(), Vec::new()];
    for c in &mut chans {
        c.resize(n, Sv7EncBand::Empty);
    }
    for b in 0..n {
        for ch in 0..2 {
            let bt = res[b][ch];
            chans[ch][b] = match bt {
                0 => Sv7EncBand::Empty,
                -1 => Sv7EncBand::Cns { scf: scf[b][ch] },
                _ => {
                    let ctx = if band_type_uses_context_selector(bt) {
                        (r.read_bits(1).unwrap() & 1) as usize
                    } else {
                        0
                    };
                    let mut levels = [0i32; 36];
                    match bt {
                        1 => {
                            for g in 0..12 {
                                let idx = vlc(r, sv7_q1_ctx(ctx)).unwrap();
                                let tri = unpack_grouped3_value(idx).unwrap();
                                for k in 0..3 {
                                    levels[g * 3 + k] = i32::from(tri[k]);
                                }
                            }
                        }
                        2 => {
                            for g in 0..18 {
                                let idx = vlc(r, sv7_q2_ctx(ctx)).unwrap();
                                let duo = unpack_grouped2_value(idx).unwrap();
                                for k in 0..2 {
                                    levels[g * 2 + k] = i32::from(duo[k]);
                                }
                            }
                        }
                        3..=7 => {
                            let t = match bt {
                                3 => sv7_q3_ctx(ctx),
                                4 => sv7_q4_ctx(ctx),
                                5 => sv7_q5_ctx(ctx),
                                6 => sv7_q6_ctx(ctx),
                                _ => sv7_q7_ctx(ctx),
                            };
                            for l in levels.iter_mut() {
                                *l = i32::from(vlc(r, t).unwrap());
                            }
                        }
                        8..=17 => {
                            for l in levels.iter_mut() {
                                *l = i32::from(r.read_bits((bt - 1) as u8).unwrap());
                            }
                        }
                        _ => unreachable!("res header bounds band types"),
                    }
                    Sv7EncBand::Coded {
                        band_type: bt,
                        ctx,
                        scf: scf[b][ch],
                        levels,
                    }
                }
            };
        }
    }
    Sv7EncStereoFrame {
        left: chans[0].clone(),
        right: chans[1].clone(),
        ms_flags,
    }
}

/// Parse `name`'s frames and re-encode with the crate writer, returning
/// (re-encoded bytes, original bytes).
fn reencode(name: &str) -> (Vec<u8>, Vec<u8>) {
    let bytes = fixture_bytes(name);
    let h = Sv7HeaderFields::parse(&bytes).unwrap();
    let mut swapped = word_swap_sv7_body(&bytes);
    swapped.extend_from_slice(&[0u8; 4]);
    let mut r = Sv7BitReader::new(&swapped);
    skip_bits(&mut r, 200);
    let mut scf_mem = [[0i32; 32]; 2];
    let mut frames = Vec::new();
    for _ in 0..h.frame_count {
        // Consume (and ignore) the frame's 20-bit length prefix; the
        // writer recomputes it.
        let _ = r.read_bits(16).unwrap();
        let _ = r.read_bits(4).unwrap();
        frames.push(parse_frame(&mut r, &h, &mut scf_mem));
    }
    let version_byte = bytes[3];
    let re = encode_sv7_file_with_version(&h, &frames, version_byte).unwrap();
    (re, bytes)
}

#[test]
fn full_files_reencode_byte_exact() {
    for name in [
        "stereo-sine-partial-last-frame",
        "silence-then-tone-partial",
        "stereo-sine-xtreme-quality",
    ] {
        let (re, orig) = reencode(name);
        assert_eq!(re.len(), orig.len(), "{name}: length");
        assert_eq!(re, orig, "{name}: bytes");
    }
}

#[test]
fn cns_pns_file_reencodes_byte_exact_content() {
    // The PNS stream: 215 CNS band-instances (Res == -1, no sample
    // bits, SCFI + DSCF carried) and the 0x17 PNS version byte. A
    // byte-exact re-encode proves the writer's CNS arm — the empty
    // sample pass plus full scalefactor participation — and the PNS
    // flag plumbing coincide with the reference encoder on a real
    // noise-substitution stream. mppenc appends one extra all-zero
    // word after the trailer on this stream (dead tail, ignored by
    // decoders); the writer intentionally does not, so the gate is:
    // identical through the re-encode's full length, all-zero surplus.
    let (re, orig) = reencode("cns-pns");
    assert_eq!(re[..3], *b"MP+");
    assert_eq!(re[3], 0x17, "re-encode preserves the PNS version byte");
    assert_eq!(orig.len() - re.len(), 4, "one surplus tail word");
    assert_eq!(re[..], orig[..re.len()], "cns-pns: content bytes");
    assert!(
        orig[re.len()..].iter().all(|&b| b == 0),
        "surplus tail must be zero padding"
    );
}

#[test]
fn flush_frame_file_reencodes_byte_exact_up_to_the_trailer_word() {
    // The original ends [.. 16 frames][11-bit trailer][flush frame]
    // [padding]; the re-encode ends [.. 16 frames][11-bit trailer]
    // [padding]. Everything up to the word containing the trailer's
    // final bits must match byte-for-byte.
    let (re, orig) = reencode("exact-multiple-16-frames");
    assert!(re.len() < orig.len(), "original carries the flush frame");
    // The re-encode's final word mixes trailer bits with zero padding
    // where the original has flush-frame bits; compare up to it.
    let shared_words = (re.len() / 4) - 1;
    assert_eq!(
        re[..shared_words * 4],
        orig[..shared_words * 4],
        "bytes through the last full pre-trailer word"
    );
}
