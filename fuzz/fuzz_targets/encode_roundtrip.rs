//! Structure-aware fuzz of the from-PCM encode → decode round trip
//! for both generations: the first input bytes pick the generation
//! and the policy knobs (quality/flat allocation, M/S, `max_band`,
//! `block_power`, CNS threshold, sample rate), the rest becomes s16
//! PCM. The encoders must accept every such input, the emitted
//! stream must decode, and the decode must return exactly the
//! gapless sample count — any panic, error, or count mismatch is a
//! finding.
#![no_main]

use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    if data.len() < 10 {
        return;
    }
    let (hdr, rest) = data.split_at(8);
    let sv7 = hdr[0] & 1 == 0;
    let channels = 1 + (hdr[0] >> 1 & 1);
    let quality = match hdr[1] {
        0xFF => None,
        q => Some(f64::from(q) * 10.0 / 254.0),
    };
    let stream_ms = hdr[2] & 1 != 0;
    let max_band = 1 + (hdr[3] & 0x1F).min(30);
    let block_power = hdr[4] & 7;
    let pns_threshold = f64::from(hdr[5]) * 4.0;
    let sample_freq_index = hdr[6] & 3;
    let step_target = f64::from(hdr[7]).max(1.0) / 8.0 + 0.25;

    // Bounded PCM from the remaining bytes.
    let nch = channels as usize;
    let n = rest.len().min(16384) / 2 / nch * nch;
    if n == 0 {
        return;
    }
    let mut pcm = Vec::with_capacity(n);
    for pair in rest[..n * 2].chunks_exact(2) {
        pcm.push(i16::from_le_bytes([pair[0], pair[1]]));
    }

    if sv7 {
        let settings = oxideav_musepack::sv7_pcm_encode::Sv7EncoderSettings {
            step_target,
            stream_ms,
            max_band,
            profile: 10,
            pns_threshold,
            quality,
        };
        let enc = oxideav_musepack::sv7_pcm_encode::encode_sv7_from_pcm_s16(
            &pcm,
            channels,
            sample_freq_index,
            &settings,
        )
        .expect("SV7 encode must accept in-range inputs");
        let dec = oxideav_musepack::sv7_file_decode::decode_sv7_file(&enc.bytes)
            .expect("own SV7 stream must decode");
        assert_eq!(dec.pcm.len() / 2, pcm.len() / nch, "SV7 gapless count");
    } else {
        let settings = oxideav_musepack::sv8_file_encode::Sv8EncoderSettings {
            step_target,
            stream_ms,
            max_band,
            block_power,
            profile: 80,
            pns_threshold,
            quality,
        };
        let enc = oxideav_musepack::sv8_file_encode::encode_sv8_from_pcm_s16(
            &pcm,
            channels,
            sample_freq_index,
            &settings,
        )
        .expect("SV8 encode must accept in-range inputs");
        let dec = oxideav_musepack::sv8_decode::decode_sv8_stream(&enc.bytes)
            .expect("own SV8 stream must decode");
        assert_eq!(dec.pcm.len(), pcm.len(), "SV8 gapless count");
    }
});
