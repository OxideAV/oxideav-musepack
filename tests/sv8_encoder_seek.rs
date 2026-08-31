//! Seek-layer gates over this crate's **own** SV8 encoder output
//! (headers-and-coding §9, r450): the from-PCM encoder now writes the
//! full §9.0 skeleton `MPCK SH RG EI SO AP… ST SE`, and — unlike the
//! staged fixtures, which are 1-2 `AP` packets long — an encoder
//! stream with a small `block_power` gives the seek path *many*
//! mid-stream entries, so these tests exercise what the corpus gates
//! cannot: real random access far from the stream head.

use oxideav_musepack::sv8_decode::decode_sv8_stream;
use oxideav_musepack::sv8_file_encode::{
    encode_sv8_from_pcm_f64, seek_pwr_delta_for, sh_payload, write_packet, write_varint,
    Sv8EncoderSettings, SEEK_PWR_DELTA,
};
use oxideav_musepack::sv8_seek::{
    decode_sv8_from_entry, SeekOffsetFields, SeekTableFields, Sv8SeekIndex, SEEK_TABLE_CAP,
    SO_PAYLOAD_LEN,
};
use oxideav_musepack::synthesis::SYNTHESIS_PRIME_SAMPLES;
use oxideav_musepack::SAMPLES_PER_FRAME_PER_CHANNEL;
use std::f64::consts::PI;

/// 3 s of a stereo two-tone test signal in the s16 domain.
fn test_pcm(seconds: f64) -> Vec<f64> {
    let n = (44100.0 * seconds) as usize;
    let mut pcm = Vec::with_capacity(n * 2);
    for t in 0..n {
        let x = t as f64 / 44100.0;
        let l = 9000.0 * (2.0 * PI * 440.0 * x).sin() + 4000.0 * (2.0 * PI * 1320.0 * x).sin();
        let r = 9000.0 * (2.0 * PI * 660.0 * x).sin() + 4000.0 * (2.0 * PI * 880.0 * x).sin();
        pcm.push(l);
        pcm.push(r);
    }
    pcm
}

fn settings_with_small_packets() -> Sv8EncoderSettings {
    Sv8EncoderSettings {
        // 4 frames per AP: a 3 s stream yields ~29 APs / ~15 entries.
        block_power: 1,
        ..Sv8EncoderSettings::default()
    }
}

/// The encoder's seek layer parses back to exactly its own `AP`
/// offsets under the `SEEK_PWR_DELTA` thinning, and the `SO` forward
/// reference lands on the `ST` packet.
#[test]
fn encoder_seek_table_indexes_its_own_ap_packets() {
    let pcm = test_pcm(3.0);
    let enc = encode_sv8_from_pcm_f64(&pcm, 2, 0, &settings_with_small_packets()).expect("encode");
    assert!(
        enc.audio_packets >= 16,
        "want a multi-entry stream, got {} APs",
        enc.audio_packets
    );

    let index = Sv8SeekIndex::from_seek_packets(&enc.bytes)
        .expect("seek packets")
        .expect("encoder writes a seek layer");
    let ground = Sv8SeekIndex::from_packet_walk(&enc.bytes).expect("walk");
    assert_eq!(ground.positions.len() as u64, enc.audio_packets);
    let expected: Vec<u64> = ground
        .positions
        .iter()
        .copied()
        .step_by(1 << SEEK_PWR_DELTA)
        .collect();
    assert_eq!(index.positions, expected);
    assert_eq!(index.packets_per_entry, 1 << SEEK_PWR_DELTA);
    assert_eq!(index.frames_per_packet, ground.frames_per_packet);
    // §9.2 n_entries rule at delta = 1: ceil(n_AP / 2).
    assert_eq!(index.positions.len() as u64, enc.audio_packets.div_ceil(2));
}

/// The default posture (64-frame packets) also carries the layer, and
/// a stream whose `SO → ST` distance needs a multi-byte varint still
/// back-patches within the fixed 5-byte slot.
#[test]
fn encoder_so_slot_holds_multibyte_distances() {
    let pcm = test_pcm(3.0);
    let enc = encode_sv8_from_pcm_f64(&pcm, 2, 0, &Sv8EncoderSettings::default()).expect("encode");
    let index = Sv8SeekIndex::from_seek_packets(&enc.bytes)
        .expect("seek packets")
        .expect("seek layer");
    assert_eq!(
        index.positions.len() as u64,
        enc.audio_packets.div_ceil(1 << SEEK_PWR_DELTA)
    );
    // The stream is > 16 KiB, so the distance needs ≥ 2 varint bytes;
    // find the SO packet and check its slot layout directly.
    let so_at = enc
        .bytes
        .windows(3)
        .position(|w| w == [b'S', b'O', 0x08])
        .expect("SO packet");
    let payload = &enc.bytes[so_at + 3..so_at + 8];
    let parsed = oxideav_musepack::sv8_seek::SeekOffsetFields::parse(payload).expect("SO");
    let mut varint = Vec::new();
    write_varint(&mut varint, parsed.st_offset);
    assert!(varint.len() >= 2, "expected a multi-byte distance");
    assert!(payload[varint.len()..].iter().all(|&b| b == 0));
}

/// Random access into the encoder's own stream: every entry rejoins
/// the linear decode within ±1 LSB once the synthesis priming
/// transient has passed — mid-stream entries included.
#[test]
fn encoder_stream_random_access_rejoins_linear_decode() {
    let pcm = test_pcm(3.0);
    let enc = encode_sv8_from_pcm_f64(&pcm, 2, 0, &settings_with_small_packets()).expect("encode");
    let linear = decode_sv8_stream(&enc.bytes).expect("linear decode");
    let nch = usize::from(linear.header.channels);
    let index = Sv8SeekIndex::from_seek_packets(&enc.bytes)
        .expect("seek packets")
        .expect("seek layer");
    assert!(index.positions.len() >= 8);

    let window = (SYNTHESIS_PRIME_SAMPLES as u64 + linear.header.beginning_silence) * nch as u64;
    // First, middle, and last entries.
    let picks = [0, index.positions.len() / 2, index.positions.len() - 1];
    for &entry in &picks {
        let seek = decode_sv8_from_entry(&enc.bytes, &index, entry).expect("entry decode");
        assert_eq!(seek.first_frame, entry as u64 * index.frames_per_entry());
        let seek_start = seek.first_frame * SAMPLES_PER_FRAME_PER_CHANNEL as u64 * nch as u64;
        let transient = if entry == 0 {
            0
        } else {
            (SYNTHESIS_PRIME_SAMPLES + 1) * nch
        };
        let mut compared = 0usize;
        let mut max_delta = 0.0f64;
        for (t, &s) in seek.pcm.iter().enumerate().skip(transient) {
            let decoded_pos = seek_start + t as u64;
            let Some(out_idx) = decoded_pos.checked_sub(window) else {
                continue;
            };
            let Some(&lin) = linear.pcm.get(out_idx as usize) else {
                break;
            };
            let delta = (s.round() - lin.round()).abs();
            max_delta = max_delta.max(delta);
            compared += 1;
        }
        assert!(
            compared > SAMPLES_PER_FRAME_PER_CHANNEL,
            "entry {entry}: compared only {compared} samples"
        );
        assert!(
            max_delta <= 1.0,
            "entry {entry}: max |seek − linear| = {max_delta}"
        );
    }
}

/// The seek layer must not disturb the audio: an encode with the
/// seek packets present decodes to the same PCM as before (the linear
/// decoder skips `SO` / `ST`), and the input round-trips at the
/// encoder's usual fidelity.
#[test]
fn seek_layer_is_transparent_to_linear_decode() {
    let pcm = test_pcm(1.0);
    let enc = encode_sv8_from_pcm_f64(&pcm, 2, 0, &Sv8EncoderSettings::default()).expect("encode");
    let dec = decode_sv8_stream(&enc.bytes).expect("decode");
    assert_eq!(dec.pcm.len(), pcm.len());
    // SNR of the round trip in the s16 domain.
    let (mut sig, mut err) = (0.0f64, 0.0f64);
    for (a, b) in pcm.iter().zip(dec.pcm.iter()) {
        sig += a * a;
        err += (a - b) * (a - b);
    }
    let snr = 10.0 * (sig / err).log10();
    assert!(snr > 60.0, "round-trip SNR {snr:.1} dB");
}

/// §9.2 decoder-side table thinning on a long-for-its-table stream:
/// a small capacity ceiling keeps every `2^diff_pwr`-th entry, the
/// frame bookkeeping stays consistent, and random access through the
/// thinned index still rejoins the linear decode within ±1 LSB past
/// the priming transient.
#[test]
fn thinned_index_random_access_rejoins_linear_decode() {
    let pcm = test_pcm(3.0);
    let enc = encode_sv8_from_pcm_f64(
        &pcm,
        2,
        0,
        &Sv8EncoderSettings {
            // 1 frame per AP → ~116 APs / ~58 stored entries.
            block_power: 0,
            ..Sv8EncoderSettings::default()
        },
    )
    .expect("encode");

    let full = Sv8SeekIndex::from_seek_packets(&enc.bytes)
        .expect("seek packets")
        .expect("seek layer");
    assert!(full.positions.len() >= 32, "want a many-entry table");
    let thin = Sv8SeekIndex::from_seek_packets_with_ceiling(&enc.bytes, 8)
        .expect("seek packets")
        .expect("seek layer");
    assert!(
        thin.positions.len() <= 8,
        "ceiling 8 kept {} entries",
        thin.positions.len()
    );

    // The thinned index is every 2^diff_pwr-th full entry, with the
    // packet stride widened to match.
    let stride = (thin.packets_per_entry / full.packets_per_entry) as usize;
    assert!(stride >= 2 && stride.is_power_of_two(), "stride {stride}");
    let expected: Vec<u64> = full.positions.iter().copied().step_by(stride).collect();
    assert_eq!(thin.positions, expected);
    assert_eq!(
        thin.frames_per_entry(),
        full.frames_per_entry() * stride as u64
    );

    // Random access from a mid-stream thinned entry.
    let linear = decode_sv8_stream(&enc.bytes).expect("linear decode");
    let nch = usize::from(linear.header.channels);
    let window = (SYNTHESIS_PRIME_SAMPLES as u64 + linear.header.beginning_silence) * nch as u64;
    let entry = thin.positions.len() / 2;
    let seek = decode_sv8_from_entry(&enc.bytes, &thin, entry).expect("entry decode");
    assert_eq!(seek.first_frame, entry as u64 * thin.frames_per_entry());
    let seek_start = seek.first_frame * SAMPLES_PER_FRAME_PER_CHANNEL as u64 * nch as u64;
    let transient = (SYNTHESIS_PRIME_SAMPLES + 1) * nch;
    let mut compared = 0usize;
    let mut max_delta = 0.0f64;
    for (t, &s) in seek.pcm.iter().enumerate().skip(transient) {
        let decoded_pos = seek_start + t as u64;
        let Some(out_idx) = decoded_pos.checked_sub(window) else {
            continue;
        };
        let Some(&lin) = linear.pcm.get(out_idx as usize) else {
            break;
        };
        max_delta = max_delta.max((s.round() - lin.round()).abs());
        compared += 1;
    }
    assert!(compared > SAMPLES_PER_FRAME_PER_CHANNEL);
    assert!(max_delta <= 1.0, "max |seek − linear| = {max_delta}");
}

/// §9.2 clamp: an `ST` that stores more entries than the `SH` sample
/// count can justify keeps only `cap << diff_pwr` of them.
#[test]
fn overpromising_seek_table_is_clamped_to_the_sample_count() {
    // Hand-composed §9.0 skeleton: 4 coded frames' worth of samples,
    // block_power 0, but an ST storing 40 entries.
    let mut out = b"MPCK".to_vec();
    let sh = sh_payload(
        4 * SAMPLES_PER_FRAME_PER_CHANNEL as u64,
        0,
        0,
        31,
        2,
        true,
        0,
    )
    .expect("sh payload");
    write_packet(&mut out, *b"SH", &sh);
    let so = SeekOffsetFields {
        // §9.1: byte distance from the SO key byte to the ST key byte
        // — here the SO packet's own extent.
        st_offset: (2 + 1 + SO_PAYLOAD_LEN) as u64,
    }
    .payload()
    .expect("so payload");
    write_packet(&mut out, *b"SO", &so);
    let table = SeekTableFields {
        seek_pwr_delta: 0,
        entries: (0..40u64).map(|i| 44 + 9 * i).collect(),
    };
    write_packet(&mut out, *b"ST", &table.payload().expect("st payload"));
    write_packet(&mut out, *b"SE", &[]);

    let idx = Sv8SeekIndex::from_seek_packets(&out)
        .expect("seek packets")
        .expect("seek layer");
    // cap = 2 + 4·1152 / (1152 << 0) = 6; diff_pwr = 0 at the default
    // ceiling — 6 of the 40 stored entries survive.
    assert_eq!(idx.positions.len(), 6);
}

/// The encoder's §9.2 granularity policy: the reference default
/// stride until the stored count would overrun the reference
/// decoder's table capacity, then the smallest power-of-two widening
/// that fits, capped at the 4-bit field maximum.
#[test]
fn seek_pwr_delta_widens_only_past_the_table_cap() {
    assert_eq!(seek_pwr_delta_for(0), SEEK_PWR_DELTA);
    assert_eq!(seek_pwr_delta_for(116), SEEK_PWR_DELTA);
    assert_eq!(seek_pwr_delta_for(2 * SEEK_TABLE_CAP), SEEK_PWR_DELTA);
    assert_eq!(seek_pwr_delta_for(2 * SEEK_TABLE_CAP + 1), 2);
    assert_eq!(seek_pwr_delta_for(4 * SEEK_TABLE_CAP + 1), 3);
    assert_eq!(seek_pwr_delta_for(u64::MAX), 0xF);
}

/// Black-box: a third-party demuxer honours our generated seek table.
/// `ffmpeg -ss` on an `.mpc` input seeks via the container's seek
/// layer; on our encoder's output it must land on an **entry-aligned
/// frame boundary** and, past the demuxer's own cold-start transient,
/// reproduce its own linear decode of the same stream exactly.
/// (Measured here: `-ss 1.5` on a 116-frame stream with 8-frame
/// entries lands on frame 56 = entry 7, byte-identical thereafter.)
/// Skips when the binary is absent (CI runners).
#[test]
fn ffmpeg_seek_lands_on_our_seek_table() {
    let Some(ffmpeg) = std::env::split_paths(&std::env::var_os("PATH").unwrap_or_default())
        .map(|d| d.join("ffmpeg"))
        .find(|c| c.is_file())
    else {
        eprintln!("skip: no ffmpeg binary on PATH");
        return;
    };
    let pcm = test_pcm(3.0);
    let enc = encode_sv8_from_pcm_f64(&pcm, 2, 0, &settings_with_small_packets()).expect("encode");
    let dir = std::env::temp_dir().join("oxideav-musepack-seek-test");
    std::fs::create_dir_all(&dir).expect("mkdir");
    let mpc = dir.join("own-seek.mpc");
    std::fs::write(&mpc, &enc.bytes).expect("write stream");

    let decode = |ss: Option<&str>| -> Vec<i16> {
        let mut cmd = std::process::Command::new(&ffmpeg);
        cmd.args(["-v", "error"]);
        if let Some(ss) = ss {
            cmd.args(["-ss", ss]);
        }
        cmd.arg("-i")
            .arg(&mpc)
            .args(["-f", "s16le", "-acodec", "pcm_s16le", "-"]);
        let out = cmd.output().expect("run ffmpeg");
        assert!(
            out.status.success(),
            "ffmpeg failed: {}",
            String::from_utf8_lossy(&out.stderr)
        );
        out.stdout
            .chunks_exact(2)
            .map(|c| i16::from_le_bytes([c[0], c[1]]))
            .collect()
    };
    let full = decode(None);
    let tail = decode(Some("1.5"));
    let nch = 2usize;
    let frame = SAMPLES_PER_FRAME_PER_CHANNEL * nch;

    // The seek must land on a frame boundary at an indexed entry.
    assert!(tail.len() < full.len(), "no seek happened");
    assert_eq!((full.len() - tail.len()) % frame, 0, "not frame-aligned");
    let start_frame = (full.len() - tail.len()) / frame;
    let index = Sv8SeekIndex::from_seek_packets(&enc.bytes)
        .expect("seek packets")
        .expect("seek layer");
    let fpe = index.frames_per_entry() as usize;
    assert_eq!(
        start_frame % fpe,
        0,
        "seek landed at frame {start_frame}, not a {fpe}-frame entry boundary"
    );
    // Near the requested 1.5 s (within one entry of granularity).
    let want = (1.5 * 44100.0 / 1152.0) as usize;
    assert!(
        start_frame.abs_diff(want) <= fpe,
        "seek landed at frame {start_frame}, wanted about {want}"
    );

    // Past the demuxer's cold-start transient the tail equals the
    // linear decode exactly.
    let skip = 2 * frame;
    let offset = full.len() - tail.len();
    assert!(
        tail[skip..]
            .iter()
            .zip(full[offset + skip..].iter())
            .all(|(a, b)| a == b),
        "seek tail diverges from the linear decode"
    );
}
