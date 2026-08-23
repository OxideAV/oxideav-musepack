//! SV8 seek-layer conformance gates (headers-and-coding §9) over the
//! staged fixture corpus (`tests/fixtures/sv8/`, docs `af8e75c` +
//! `dd39285`).
//!
//! Every corpus stream carries the §9.0 skeleton
//! `MPCK SH RG EI SO AP… ST SE`; these gates pin, per fixture:
//!
//! - **§9.1 `SO`** — fixed 8-byte packet (5-byte payload), whose
//!   varint equals the *measured* `ST_position − SO_position`
//!   byte distance, with zero back-patch padding;
//! - **§9.2 `ST`** — the parsed entry list equals the measured `AP`
//!   packet offsets thinned to every `2^seek_pwr_delta`-th (the
//!   corpus posture is `seek_pwr_delta == 1`, one entry per two
//!   `AP`s, `n_entries == ceil(n_AP / 2)`), and re-composing the
//!   parsed table reproduces the reference encoder's `ST` payload
//!   **byte-for-byte** (wire symmetry of the Golomb residual coder);
//! - **§9.3 `SE`** — 3-byte packet, empty payload;
//! - **random access** — entering the stream at each index entry via
//!   [`oxideav_musepack::sv8_seek::decode_sv8_from_entry`]
//!   reproduces the linear whole-stream decode exactly (±1 LSB in
//!   the s16 domain) past the cold-filterbank priming transient.
//!
//! Seek-accuracy depth (many entries, mid-stream entry points) is
//! exercised on this crate's own encoder output in
//! `tests/sv8_encoder_seek.rs` — the staged fixtures are short (1-2
//! `AP` packets each).

use oxideav_musepack::framing::parse_sv8_magic;
use oxideav_musepack::packet_stream::{PacketSizeConvention, PacketStream};
use oxideav_musepack::sv8_decode::decode_sv8_stream;
use oxideav_musepack::sv8_seek::{
    decode_sv8_from_entry, SeekTableFields, Sv8SeekIndex, SO_PAYLOAD_LEN,
};
use oxideav_musepack::synthesis::SYNTHESIS_PRIME_SAMPLES;
use oxideav_musepack::SAMPLES_PER_FRAME_PER_CHANNEL;

const FIXTURES: &[&str] = &[
    "cns-pns",
    "exact-multiple-16-frames",
    "mono-sine-standard",
    "silence-then-tone-partial",
    "stereo-sine-partial-last-frame",
    "stereo-sine-two-packets",
    "stereo-sine-xtreme-quality",
];

fn fixture_bytes(name: &str) -> Vec<u8> {
    let path = format!(
        "{}/tests/fixtures/sv8/{name}/input.mpc",
        env!("CARGO_MANIFEST_DIR")
    );
    std::fs::read(&path).unwrap_or_else(|e| panic!("read {path}: {e}"))
}

/// Byte-level packet walk: (key, packet start offset from `MPCK`,
/// header_len, payload copy) for every packet.
#[allow(clippy::type_complexity)]
fn walk(input: &[u8]) -> Vec<([u8; 2], u64, usize, Vec<u8>)> {
    let after_magic = parse_sv8_magic(input).expect("magic");
    let total = input.len() - after_magic;
    let mut stream = PacketStream::new(&input[after_magic..], PacketSizeConvention::Inclusive);
    let mut out = Vec::new();
    while let Some(p) = stream.next_packet().expect("walk") {
        let next = (total - stream.remaining_bytes().len()) as u64;
        let extent = (p.header.header_len + p.payload.len()) as u64;
        out.push((
            p.key.as_bytes(),
            next - extent + after_magic as u64,
            p.header.header_len,
            p.payload.to_vec(),
        ));
    }
    out
}

/// §9.0 + §9.1: skeleton order, the fixed `SO` shape, and the
/// measured `SO → ST` distance.
#[test]
fn corpus_so_packet_is_fixed_width_and_points_at_st() {
    for name in FIXTURES {
        let input = fixture_bytes(name);
        let packets = walk(&input);
        let keys: Vec<[u8; 2]> = packets.iter().map(|p| p.0).collect();
        // §9.0 skeleton: SH RG EI SO AP… ST SE.
        assert_eq!(&keys[..4], &[*b"SH", *b"RG", *b"EI", *b"SO"], "{name}");
        assert_eq!(keys[keys.len() - 2..], [*b"ST", *b"SE"], "{name}");
        assert!(
            keys[4..keys.len() - 2].iter().all(|k| k == b"AP"),
            "{name}: non-AP packet inside the audio run"
        );

        let (_, so_pos, so_hdr, so_payload) = &packets[3];
        assert_eq!(*so_hdr + so_payload.len(), 8, "{name}: SO packet size");
        assert_eq!(so_payload.len(), SO_PAYLOAD_LEN, "{name}");
        let (_, st_pos, ..) = packets[packets.len() - 2];
        let so = oxideav_musepack::sv8_seek::SeekOffsetFields::parse(so_payload).expect("SO");
        assert_eq!(so.st_offset, st_pos - so_pos, "{name}: SO → ST distance");
        // Back-patch padding after the varint is zero (§9.1).
        let mut varint = Vec::new();
        oxideav_musepack::sv8_file_encode::write_varint(&mut varint, so.st_offset);
        assert!(
            so_payload[varint.len()..].iter().all(|&b| b == 0),
            "{name}: nonzero SO padding"
        );

        // §9.3: SE is 3 bytes, empty payload.
        let (_, _, se_hdr, se_payload) = &packets[packets.len() - 1];
        assert_eq!(*se_hdr, 3, "{name}: SE header length");
        assert!(se_payload.is_empty(), "{name}: SE payload");
    }
}

/// §9.2: the parsed table equals the measured `AP` offsets under the
/// corpus's `seek_pwr_delta == 1` posture, and the index resolver
/// agrees with the ground-truth packet walk.
#[test]
fn corpus_st_entries_equal_measured_ap_offsets() {
    for name in FIXTURES {
        let input = fixture_bytes(name);
        let packets = walk(&input);
        let ap_offsets: Vec<u64> = packets
            .iter()
            .filter(|p| p.0 == *b"AP")
            .map(|p| p.1)
            .collect();
        let st_payload = &packets[packets.len() - 2].3;
        let table = SeekTableFields::parse(st_payload).expect("ST parse");
        assert_eq!(table.seek_pwr_delta, 1, "{name}: corpus seek_pwr_delta");
        let expected: Vec<u64> = ap_offsets.iter().copied().step_by(2).collect();
        assert_eq!(table.entries, expected, "{name}: ST entries");
        assert_eq!(
            table.entries.len() as u64,
            (ap_offsets.len() as u64).div_ceil(2),
            "{name}: n_entries rule"
        );

        // The resolved index (SO → ST fast path) matches the walk.
        let index = Sv8SeekIndex::from_seek_packets(&input)
            .expect("index")
            .expect("corpus stream has a seek layer");
        assert_eq!(index.positions, expected, "{name}: resolved index");
        assert_eq!(index.packets_per_entry, 2, "{name}");
        let ground = Sv8SeekIndex::from_packet_walk(&input).expect("walk index");
        assert_eq!(ground.positions, ap_offsets, "{name}");
        assert_eq!(index.frames_per_packet, ground.frames_per_packet, "{name}");
    }
}

/// §9.2 wire symmetry: re-composing the parsed table reproduces the
/// reference encoder's `ST` payload byte-for-byte on all 7 streams.
#[test]
fn corpus_st_payload_recompose_is_byte_exact() {
    for name in FIXTURES {
        let input = fixture_bytes(name);
        let packets = walk(&input);
        let st_payload = &packets[packets.len() - 2].3;
        let table = SeekTableFields::parse(st_payload).expect("ST parse");
        let recomposed = table.payload().expect("compose");
        assert_eq!(&recomposed, st_payload, "{name}: ST payload bytes");
    }
}

/// Random access: entering at every index entry reproduces the linear
/// decode exactly past the synthesis priming transient. Entry 0 has
/// no transient at all (the linear decode starts equally cold).
#[test]
fn corpus_entry_decode_matches_linear_decode() {
    for name in FIXTURES {
        let input = fixture_bytes(name);
        let linear = decode_sv8_stream(&input).expect("linear decode");
        let nch = usize::from(linear.header.channels);
        let index = Sv8SeekIndex::from_seek_packets(&input)
            .expect("index")
            .expect("seek layer");
        for entry in 0..index.positions.len() {
            let seek = decode_sv8_from_entry(&input, &index, entry).expect("entry decode");
            assert_eq!(
                seek.first_frame,
                entry as u64 * index.frames_per_entry(),
                "{name}"
            );
            // Map the seek decode into the linear (trimmed) output
            // timeline: seek sample t (per channel) sits at decoded
            // position first_frame·1152 + t = output index + 481 +
            // silence.
            let window =
                (SYNTHESIS_PRIME_SAMPLES as u64 + linear.header.beginning_silence) * nch as u64;
            let seek_start = seek.first_frame * SAMPLES_PER_FRAME_PER_CHANNEL as u64 * nch as u64;
            // Skip the cold-start transient (one priming window) for
            // mid-stream entries.
            let transient = if entry == 0 {
                0
            } else {
                (SYNTHESIS_PRIME_SAMPLES + 1) * nch
            };
            let mut compared = 0usize;
            for (t, &s) in seek.pcm.iter().enumerate().skip(transient) {
                let decoded_pos = seek_start + t as u64;
                let Some(out_idx) = decoded_pos.checked_sub(window) else {
                    continue;
                };
                let Some(&lin) = linear.pcm.get(out_idx as usize) else {
                    break;
                };
                let delta = (s.round() - lin.round()).abs();
                assert!(
                    delta <= 1.0,
                    "{name}: entry {entry} sample {t}: seek {s} vs linear {lin}"
                );
                compared += 1;
            }
            assert!(
                compared > 0,
                "{name}: entry {entry} compared no samples against the linear decode"
            );
        }
    }
}
