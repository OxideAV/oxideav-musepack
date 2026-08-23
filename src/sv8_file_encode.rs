//! SV8 **whole-stream encoder** — PCM in, a complete `MPCK` byte
//! stream out.
//!
//! This closes the encoder pipeline on top of the layers below it:
//! [`crate::analysis`] (PCM → subband matrices),
//! [`crate::sv8_frame_build`] (matrices → structured frames),
//! [`crate::sv8_stereo_frame_encode`] (frames → `AP` payload bits),
//! plus the packet layer this module adds (varint / packet writers,
//! the `SH` / `RG` / `EI` payload composers with the empirically
//! pinned `SH` CRC — [`crate::sv8_crc`]).
//!
//! # Stream shape
//!
//! `MPCK`, then `SH` → `RG` → `EI` → `SO` → `AP`×N → `ST` → `SE` —
//! the full §9.0 packet skeleton including the seek layer
//! (headers-and-coding §9, wired r450): `SO` is written before the
//! audio as the fixed 5-byte back-patchable forward reference and
//! patched once the `ST` position is known; `ST` carries one entry
//! per `2^seek_pwr_delta` `AP` packets (the corpus posture
//! `seek_pwr_delta = 1`) under the §9.2 Golomb `k = 12`
//! second-difference entry code ([`crate::sv8_seek`]).
//!
//! Every `AP` packet carries up to `2^(block_power × 2)` frames
//! (headers-and-coding §2, field 9) and opens with a **key frame**
//! (absolute scalefactors, fresh `Max_used_Band` log code — spec
//! §3.3); the cross-frame SCF memory resets at each packet boundary,
//! mirroring [`crate::sv8_stream::Sv8StreamDecoder`].
//!
//! # Gapless fields
//!
//! A Musepack decoder outputs
//! `decoded[481 + silence .. 481 + sample_count]` (the
//! reference-decoder-pinned window — see
//! [`crate::synthesis::SYNTHESIS_PRIME_SAMPLES`] and
//! [`crate::sv8_decode::decode_sv8_stream`]), decoding
//! `⌈sample_count / 1152⌉` frames and draining the filterbank for any
//! remainder. The analysis front end already delays the signal by the
//! matching [`ANALYSIS_SYNTHESIS_DELAY`] (481) samples, so a stream
//! with `silence = 0`, `sample_count = N` decodes to exactly the
//! input, time-aligned. The encoder additionally picks the smallest
//! front pad `silence ∈ 0..=481` with
//! `(N + silence) mod 1152 ∈ 1..=671`, which keeps the coded frames'
//! slack past the nominal timeline at ≥ 481 samples — the drain then
//! only ever covers padding, never real audio (unlike the reference
//! posture on exact-multiple streams, whose last 481 samples are
//! flush-approximated). Decoders skip `481 + silence` and return
//! exactly `sample_count − silence = N` samples.
//!
//! # Mono
//!
//! A mono stream declares `channels = 1` in the `SH` but still codes
//! two channels per frame body (the fixture-pinned r419 body shape).
//! The encoder feeds the mono analysis matrix as both builder inputs:
//! every coded band then elects M/S with a silent side (mid = the
//! signal, side = 0), the cheap mono body, and the decoder's
//! channel-0 output (`L = M + S = M`) is the signal.
//!
//! Source-of-record: `spec/musepack-headers-and-coding.md` §2 (`SH` /
//! `RG` / `EI` field maps, block power), §3 (varint, inclusive packet
//! size); `musepack-sv7-sv8-spec.md` §3.1-§3.3 (packet stream, packet
//! vocabulary, keyframes). The `SH` CRC parameters and the packet
//! order are fixture-pinned ([`crate::sv8_crc`], corpus README). No
//! new frame-body facts — the body layer is the r419 wire-symmetric
//! encoder.

use crate::analysis::{analyze_frame_channel, AnalysisFilter, ANALYSIS_SYNTHESIS_DELAY};
use crate::framing::SV8_MAGIC;
use crate::sh_header::{SV8_SAMPLE_RATES, SV8_STREAM_VERSION};
use crate::sv7_bitwriter::Sv7BitWriter;
use crate::sv8_crc::sv8_crc32;
use crate::sv8_frame_build::{build_sv8_stereo_frame, Sv8FrameBuildSettings};
use crate::sv8_seek::{SeekOffsetFields, SeekTableFields};
use crate::sv8_stereo_frame::Sv8FrameState;
use crate::sv8_stereo_frame_encode::encode_sv8_stereo_frame;
use crate::{Error, Result, SAMPLES_PER_FRAME_PER_CHANNEL};

/// Whole-stream encoder settings.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sv8EncoderSettings {
    /// The flat s16-domain quantisation-step target (see
    /// [`Sv8FrameBuildSettings::step_target`]).
    pub step_target: f64,
    /// Stream-wide M/S posture (`SH` field 8 + per-band election).
    pub stream_ms: bool,
    /// `SH` highest coded subband (`1..=31`). Bands above it are never
    /// coded.
    pub max_band: u8,
    /// `SH` block power (`0..=7`): each `AP` packet carries up to
    /// `2^(block_power × 2)` frames. The fixture corpus uses 3
    /// (64 frames per packet).
    pub block_power: u8,
    /// The `EI` profile byte's 7-bit profile field (the quality
    /// preset × 8; informational).
    pub profile: u8,
    /// Noise-substitution threshold in s16-domain subband peak units
    /// (`0.0` = CNS emission off, the default) — see
    /// [`Sv8FrameBuildSettings::pns_threshold`]. When enabled the
    /// `EI` packet's PNS flag is set.
    pub pns_threshold: f64,
}

impl Default for Sv8EncoderSettings {
    /// Defaults: `step_target = 2.0`, M/S on, all 32 subbands,
    /// `block_power = 3` (the reference posture), profile 80
    /// (= 10.0, the "extreme"-tier tag).
    fn default() -> Self {
        Self {
            step_target: 2.0,
            stream_ms: true,
            max_band: 31,
            block_power: 3,
            profile: 80,
            pns_threshold: 0.0,
        }
    }
}

/// §9.2 seek granularity the encoder writes: one `ST` entry per
/// `2^SEEK_PWR_DELTA` `AP` packets (the reference default posture,
/// `seek_pwr_delta = 1`).
pub const SEEK_PWR_DELTA: u8 = 1;

/// Append a §3 varint: big-endian 7-bit groups, continuation high bit
/// on all but the last byte.
pub fn write_varint(out: &mut Vec<u8>, value: u64) {
    let mut groups = [0u8; 10];
    let mut n = 0;
    let mut v = value;
    loop {
        groups[n] = (v & 0x7F) as u8;
        n += 1;
        v >>= 7;
        if v == 0 {
            break;
        }
    }
    for i in (0..n).rev() {
        let cont = if i == 0 { 0 } else { 0x80 };
        out.push(groups[i] | cont);
    }
}

/// The number of bytes [`write_varint`] emits for `value`.
#[must_use]
pub fn varint_len(value: u64) -> usize {
    let mut n = 1;
    let mut v = value >> 7;
    while v != 0 {
        n += 1;
        v >>= 7;
    }
    n
}

/// Append one packet: 2-byte key, the **inclusive** §3 size varint
/// (key + size bytes + payload), then the payload. The size field's
/// own length feeds back into the size, so the writer converges the
/// fixed point (`size = 2 + varint_len(size) + payload`).
pub fn write_packet(out: &mut Vec<u8>, key: [u8; 2], payload: &[u8]) {
    // Fixed point: try each size-field length until self-consistent
    // (monotone, so the first match is the unique one).
    let mut size_len = 1;
    loop {
        let total = (2 + size_len + payload.len()) as u64;
        let need = varint_len(total);
        if need == size_len {
            break;
        }
        size_len = need;
    }
    let total = (2 + size_len + payload.len()) as u64;
    out.extend_from_slice(&key);
    write_varint(out, total);
    out.extend_from_slice(payload);
}

/// Compose the `SH` payload (§2): CRC-32 over the rest, stream
/// version 8, sample-count and beginning-silence varints, then the
/// packed 16-bit tail (`freq:3, max_band−1:5, channels−1:4, ms:1,
/// block_power:3`).
///
/// # Errors
///
/// [`Error::HeaderFieldOutOfRange`] for a field outside its §2 width
/// or bias range.
pub fn sh_payload(
    sample_count: u64,
    beginning_silence: u64,
    sample_freq_index: u8,
    max_band: u8,
    channels: u8,
    mid_side: bool,
    block_power: u8,
) -> Result<Vec<u8>> {
    if sample_freq_index as usize >= SV8_SAMPLE_RATES.len() {
        return Err(Error::HeaderFieldOutOfRange("sample_freq_index"));
    }
    if !(1..=31).contains(&max_band) {
        return Err(Error::MaxBandOutOfRange(max_band));
    }
    if !(1..=16).contains(&channels) {
        return Err(Error::HeaderFieldOutOfRange("channels"));
    }
    if block_power > 7 {
        return Err(Error::HeaderFieldOutOfRange("block_power"));
    }
    let mut body = Vec::with_capacity(8);
    body.push(SV8_STREAM_VERSION);
    write_varint(&mut body, sample_count);
    write_varint(&mut body, beginning_silence);
    let packed: u16 = (u16::from(sample_freq_index) << 13)
        | (u16::from(max_band - 1) << 8)
        | (u16::from(channels - 1) << 4)
        | (u16::from(mid_side) << 3)
        | u16::from(block_power);
    body.extend_from_slice(&packed.to_be_bytes());

    let mut payload = Vec::with_capacity(4 + body.len());
    payload.extend_from_slice(&sv8_crc32(&body).to_be_bytes());
    payload.extend_from_slice(&body);
    Ok(payload)
}

/// Compose the `RG` payload (§2): version 1, then the four 16-bit
/// gain/peak fields (zero = "not computed"; bitstream-level
/// ReplayGain analysis is out of the codec's scope).
#[must_use]
pub fn rg_payload(title_gain: u16, title_peak: u16, album_gain: u16, album_peak: u16) -> Vec<u8> {
    let mut out = Vec::with_capacity(9);
    out.push(1);
    out.extend_from_slice(&title_gain.to_be_bytes());
    out.extend_from_slice(&title_peak.to_be_bytes());
    out.extend_from_slice(&album_gain.to_be_bytes());
    out.extend_from_slice(&album_peak.to_be_bytes());
    out
}

/// Compose the `EI` payload (§2): the packed `profile×8`(7)+PNS(1)
/// byte, then the three encoder-version bytes (this crate does not
/// emit noise-substituted bands, so PNS is always 0).
///
/// # Errors
///
/// [`Error::HeaderFieldOutOfRange`] if `profile` exceeds its 7-bit
/// field.
pub fn ei_payload(profile: u8, pns: bool, major: u8, minor: u8, build: u8) -> Result<Vec<u8>> {
    if profile > 0x7F {
        return Err(Error::HeaderFieldOutOfRange("profile"));
    }
    Ok(vec![(profile << 1) | u8::from(pns), major, minor, build])
}

/// The result of a whole-stream encode: the `MPCK` bytes plus the
/// realised stream geometry.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Sv8EncodedStream {
    /// The complete `MPCK` byte stream.
    pub bytes: Vec<u8>,
    /// Frames coded.
    pub frames: u64,
    /// `AP` packets written.
    pub audio_packets: u64,
}

/// Encode interleaved f64 PCM (s16 domain: full scale ≈ ±32767) into
/// a complete SV8 `MPCK` stream.
///
/// `pcm` is interleaved `L, R, …` for `channels == 2`, plain mono for
/// `channels == 1`; `sample_freq_index` indexes
/// [`SV8_SAMPLE_RATES`]. The `SH` totals make the stream gapless:
/// decoding returns exactly `pcm.len() / channels` samples per
/// channel, time-aligned with the input (see the module docs).
///
/// # Errors
///
/// - [`Error::ChannelCountInvalid`] for a channel count other than 1
///   or 2.
/// - [`Error::HeaderFieldOutOfRange`] / [`Error::MaxBandOutOfRange`]
///   for out-of-range settings.
pub fn encode_sv8_from_pcm_f64(
    pcm: &[f64],
    channels: u8,
    sample_freq_index: u8,
    settings: &Sv8EncoderSettings,
) -> Result<Sv8EncodedStream> {
    if !(1..=2).contains(&channels) || pcm.len() % channels as usize != 0 {
        return Err(Error::ChannelCountInvalid(channels));
    }
    let nch = channels as usize;
    let n = (pcm.len() / nch) as u64;

    // Gapless geometry (module docs). The decoder outputs
    // `decoded[481 + silence .. 481 + sample_count]`, decoding
    // `⌈sample_count / 1152⌉` frames and draining the rest — so the
    // encoder picks the smallest `beginning_silence` pad that keeps
    // the real tail inside the coded frames:
    // `(n + silence) mod 1152 ∈ 1..=671` ⇒ the frame slack past the
    // nominal timeline is ≥ 481 (the priming skip) and the drain
    // never has to approximate real samples.
    let frame_len = SAMPLES_PER_FRAME_PER_CHANNEL as u64;
    let delay = ANALYSIS_SYNTHESIS_DELAY as u64;
    let r = n % frame_len;
    let silence = if (1..=(frame_len - delay)).contains(&r) {
        0
    } else if r == 0 {
        1
    } else {
        frame_len - r + 1
    };
    let sample_count = n + silence;
    let frames = sample_count.div_ceil(frame_len);
    let frames_per_packet = 1u64 << (u32::from(settings.block_power) * 2);

    let build = Sv8FrameBuildSettings {
        step_target: settings.step_target,
        stream_ms: settings.stream_ms,
        pns_threshold: settings.pns_threshold,
    };

    // Stream prefix: magic + SH + RG + EI.
    let mut out = Vec::new();
    out.extend_from_slice(&SV8_MAGIC);
    let sh = sh_payload(
        sample_count,
        silence,
        sample_freq_index,
        settings.max_band,
        channels,
        settings.stream_ms,
        settings.block_power,
    )?;
    write_packet(&mut out, *b"SH", &sh);
    write_packet(&mut out, *b"RG", &rg_payload(0, 0, 0, 0));
    write_packet(
        &mut out,
        *b"EI",
        &ei_payload(settings.profile, settings.pns_threshold > 0.0, 0, 1, 0)?,
    );

    // §9.0/§9.1: the SO packet precedes the audio; its payload is a
    // fixed 5-byte slot back-patched with the ST distance below.
    let so_pos = out.len();
    write_packet(&mut out, *b"SO", &[0u8; crate::sv8_seek::SO_PAYLOAD_LEN]);

    // Audio: per-channel analysis filters persist across the whole
    // stream; the frame-state resets per packet.
    let mut filters = vec![AnalysisFilter::new(); nch];
    let mut frame_pcm = [0.0_f64; SAMPLES_PER_FRAME_PER_CHANNEL];
    let mut audio_packets = 0u64;
    let mut ap_offsets: Vec<u64> = Vec::new();
    let mut frame_index = 0u64;
    while frame_index < frames {
        let packet_frames = frames_per_packet.min(frames - frame_index);
        ap_offsets.push(out.len() as u64);
        let mut writer = Sv7BitWriter::new();
        let mut state = Sv8FrameState::new();
        for pf in 0..packet_frames {
            let f = frame_index + pf;
            let base = f * SAMPLES_PER_FRAME_PER_CHANNEL as u64;
            // De-interleave (and zero-pad) each channel's 1152 samples,
            // run the analysis, build, and wire-encode the frame.
            let mut matrices = Vec::with_capacity(nch);
            for (ch, filter) in filters.iter_mut().enumerate() {
                for (k, slot) in frame_pcm.iter_mut().enumerate() {
                    // The fed timeline is [silence pad][input][tail pad].
                    let t = base + k as u64;
                    *slot = if t < silence {
                        0.0
                    } else {
                        let idx = (t - silence) * nch as u64 + ch as u64;
                        pcm.get(idx as usize).copied().unwrap_or(0.0)
                    };
                }
                matrices.push(analyze_frame_channel(filter, &frame_pcm));
            }
            let (left, right) = if nch == 2 {
                (&matrices[0], &matrices[1])
            } else {
                (&matrices[0], &matrices[0])
            };
            let frame = build_sv8_stereo_frame(left, right, settings.max_band, &build)?;
            encode_sv8_stereo_frame(
                &mut writer,
                &frame,
                settings.max_band,
                pf == 0,
                settings.stream_ms,
                &mut state,
            )?;
        }
        write_packet(&mut out, *b"AP", &writer.finish());
        audio_packets += 1;
        frame_index += packet_frames;
    }

    // §9.2: the seek table lands after the audio — one entry per
    // 2^SEEK_PWR_DELTA AP packets (entries are byte offsets relative
    // to header_position, which is 0 in this buffer) — then the SO
    // forward reference is back-patched with the measured distance
    // (§9.1: ST_position − SO_position).
    let st_pos = out.len();
    let table = SeekTableFields {
        seek_pwr_delta: SEEK_PWR_DELTA,
        entries: ap_offsets
            .iter()
            .copied()
            .step_by(1 << SEEK_PWR_DELTA)
            .collect(),
    };
    write_packet(&mut out, *b"ST", &table.payload()?);
    let so_payload = SeekOffsetFields {
        st_offset: (st_pos - so_pos) as u64,
    }
    .payload()?;
    out[so_pos + 3..so_pos + 8].copy_from_slice(&so_payload);

    write_packet(&mut out, *b"SE", &[]);
    Ok(Sv8EncodedStream {
        bytes: out,
        frames,
        audio_packets,
    })
}

/// [`encode_sv8_from_pcm_f64`] over interleaved `i16` PCM.
///
/// # Errors
///
/// As [`encode_sv8_from_pcm_f64`].
pub fn encode_sv8_from_pcm_s16(
    pcm: &[i16],
    channels: u8,
    sample_freq_index: u8,
    settings: &Sv8EncoderSettings,
) -> Result<Sv8EncodedStream> {
    let f: Vec<f64> = pcm.iter().map(|&s| f64::from(s)).collect();
    encode_sv8_from_pcm_f64(&f, channels, sample_freq_index, settings)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::framing::parse_varint;
    use crate::packet_stream::{PacketSizeConvention, PacketStream};
    use crate::sh_header::StreamHeaderFields;
    use crate::sv8_decode::decode_sv8_stream;
    use crate::typed_packet::TypedPacket;

    /// write_varint ↔ parse_varint round trip across widths.
    #[test]
    fn varint_round_trips() {
        // parse_varint caps at 9 bytes = 63 payload bits, so the
        // round-trippable domain is 0..=2^63 − 1 (sample counts and
        // packet sizes sit far below it).
        for v in [
            0u64,
            1,
            0x7F,
            0x80,
            0x3FFF,
            0x4000,
            22_050,
            u64::from(u32::MAX),
            (1u64 << 63) - 1,
        ] {
            let mut buf = Vec::new();
            write_varint(&mut buf, v);
            assert_eq!(buf.len(), varint_len(v), "value {v}");
            let (parsed, used) = parse_varint(&buf).unwrap();
            assert_eq!((parsed, used), (v, buf.len()), "value {v}");
        }
    }

    /// The packet writer's inclusive size self-references correctly:
    /// the stream walker returns exactly the payload for sizes around
    /// the 1-byte/2-byte varint boundary.
    #[test]
    fn packet_size_fixed_point_around_varint_boundary() {
        for payload_len in [0usize, 1, 122, 123, 124, 125, 200, 16_381, 16_382, 16_383] {
            let payload: Vec<u8> = (0..payload_len).map(|i| i as u8).collect();
            let mut buf = Vec::new();
            write_packet(&mut buf, *b"AP", &payload);
            let mut stream = PacketStream::new(&buf, PacketSizeConvention::Inclusive);
            let p = stream.next_packet().unwrap().expect("one packet");
            assert_eq!(p.payload, &payload[..], "payload_len {payload_len}");
            assert!(stream.next_packet().unwrap().is_none());
        }
    }

    /// The SH composer is the exact inverse of the SH parser, and its
    /// CRC validates.
    #[test]
    fn sh_payload_parses_back_with_valid_crc() {
        let payload = sh_payload(22_050, 481, 0, 28, 2, true, 3).unwrap();
        let fields = StreamHeaderFields::parse(&payload).unwrap();
        assert_eq!(fields.sample_count, 22_050);
        assert_eq!(fields.beginning_silence, 481);
        assert_eq!(fields.sample_freq_index, 0);
        assert_eq!(fields.max_band, 28);
        assert_eq!(fields.channels, 2);
        assert!(fields.mid_side);
        assert_eq!(fields.block_power, 3);
        assert_eq!(fields.crc, sv8_crc32(&payload[4..]));
    }

    #[test]
    fn sh_payload_field_bounds() {
        assert!(matches!(
            sh_payload(0, 0, 4, 28, 2, true, 3),
            Err(Error::HeaderFieldOutOfRange("sample_freq_index"))
        ));
        assert!(matches!(
            sh_payload(0, 0, 0, 0, 2, true, 3),
            Err(Error::MaxBandOutOfRange(0))
        ));
        assert!(matches!(
            sh_payload(0, 0, 0, 28, 0, true, 3),
            Err(Error::HeaderFieldOutOfRange("channels"))
        ));
        assert!(matches!(
            sh_payload(0, 0, 0, 28, 2, true, 8),
            Err(Error::HeaderFieldOutOfRange("block_power"))
        ));
    }

    /// The RG / EI composers parse back through their field-map
    /// decoders.
    #[test]
    fn rg_ei_payloads_parse_back() {
        use crate::ei_header::EncoderInfoFields;
        use crate::rg_header::ReplayGainFields;

        let rg = ReplayGainFields::parse(&rg_payload(1, 2, 3, 4)).unwrap();
        assert_eq!(
            (rg.title_gain, rg.title_peak, rg.album_gain, rg.album_peak),
            (1, 2, 3, 4)
        );

        let ei = EncoderInfoFields::parse(&ei_payload(80, false, 0, 1, 0).unwrap()).unwrap();
        assert_eq!(ei.profile_int(), 10);
        assert!(!ei.pns);
        assert!(matches!(
            ei_payload(0x80, false, 0, 0, 0),
            Err(Error::HeaderFieldOutOfRange("profile"))
        ));
    }

    /// A silent stereo encode produces a decodable stream of the
    /// declared shape: SH → RG → EI → AP×N → SE, exact totals, silent
    /// output.
    #[test]
    fn silent_stereo_stream_decodes_to_exact_silence() {
        let n = 3000usize;
        let pcm = vec![0.0_f64; 2 * n];
        let enc = encode_sv8_from_pcm_f64(&pcm, 2, 0, &Sv8EncoderSettings::default()).unwrap();
        // 3000 + 481 = 3481 → 4 frames, one AP (block_power 3 = 64).
        assert_eq!(enc.frames, 4);
        assert_eq!(enc.audio_packets, 1);

        // Packet order.
        let mut stream = PacketStream::new(&enc.bytes[4..], PacketSizeConvention::Inclusive);
        let mut kinds = Vec::new();
        while let Some(p) = stream.next_packet().unwrap() {
            kinds.push(format!("{:?}", p.key));
            if kinds.last().map(String::as_str) == Some("StreamEnd") {
                break;
            }
        }
        assert_eq!(
            kinds,
            [
                "StreamHeader",
                "ReplayGain",
                "EncoderInfo",
                "SeekTableOffset",
                "AudioPacket",
                "SeekTable",
                "StreamEnd"
            ]
        );

        let out = decode_sv8_stream(&enc.bytes).unwrap();
        assert_eq!(out.pcm.len(), 2 * n);
        assert!(out.pcm.iter().all(|&s| s == 0.0));
    }

    /// Mono declares one channel in the SH and decodes to n samples.
    #[test]
    fn mono_stream_shape() {
        let n = 2000usize;
        let pcm = vec![0.0_f64; n];
        let enc = encode_sv8_from_pcm_f64(&pcm, 1, 1, &Sv8EncoderSettings::default()).unwrap();
        let out = decode_sv8_stream(&enc.bytes).unwrap();
        assert_eq!(out.header.channels, 1);
        assert_eq!(out.header.sample_rate_hz(), Some(48_000));
        assert_eq!(out.pcm.len(), n);
    }

    /// s16 and f64 entries produce identical bytes.
    #[test]
    fn s16_entry_matches_f64() {
        let pcm_i: Vec<i16> = (0..2048).map(|i| ((i * 37) % 1024) as i16 - 512).collect();
        let pcm_f: Vec<f64> = pcm_i.iter().map(|&s| f64::from(s)).collect();
        let a = encode_sv8_from_pcm_s16(&pcm_i, 2, 0, &Sv8EncoderSettings::default()).unwrap();
        let b = encode_sv8_from_pcm_f64(&pcm_f, 2, 0, &Sv8EncoderSettings::default()).unwrap();
        assert_eq!(a, b);
    }

    /// Channel-count and interleave validation.
    #[test]
    fn rejects_bad_channel_shapes() {
        assert!(matches!(
            encode_sv8_from_pcm_f64(&[0.0; 10], 3, 0, &Sv8EncoderSettings::default()),
            Err(Error::ChannelCountInvalid(3))
        ));
        assert!(matches!(
            encode_sv8_from_pcm_f64(&[0.0; 11], 2, 0, &Sv8EncoderSettings::default()),
            Err(Error::ChannelCountInvalid(2))
        ));
    }

    /// block_power drives the AP packet split: 0 ⇒ one frame per
    /// packet.
    #[test]
    fn block_power_zero_puts_one_frame_per_packet() {
        let n = 2400usize; // + 481 ⇒ 3 frames
        let pcm = vec![0.0_f64; 2 * n];
        let settings = Sv8EncoderSettings {
            block_power: 0,
            ..Default::default()
        };
        let enc = encode_sv8_from_pcm_f64(&pcm, 2, 0, &settings).unwrap();
        assert_eq!(enc.frames, 3);
        assert_eq!(enc.audio_packets, 3);
        let out = decode_sv8_stream(&enc.bytes).unwrap();
        assert_eq!(out.header.frames_per_audio_packet(), 1);
        assert_eq!(out.audio_packets, 3);
        assert_eq!(out.pcm.len(), 2 * n);
    }

    /// The SH the encoder writes round-trips through the typed packet
    /// layer with a self-consistent CRC.
    #[test]
    fn encoded_sh_crc_is_self_consistent() {
        let enc =
            encode_sv8_from_pcm_f64(&[0.0; 512], 1, 0, &Sv8EncoderSettings::default()).unwrap();
        let mut stream = PacketStream::new(&enc.bytes[4..], PacketSizeConvention::Inclusive);
        while let Some(p) = stream.next_packet().unwrap() {
            if let TypedPacket::StreamHeader(sh) = TypedPacket::classify(p) {
                let payload = sh.payload_bytes();
                let fields = StreamHeaderFields::parse(payload).unwrap();
                assert_eq!(fields.crc, sv8_crc32(&payload[4..]));
                // n = 512 ⇒ 512 mod 1152 = 512 ∈ 1..=671 ⇒ no pad.
                assert_eq!(fields.beginning_silence, 0);
                assert_eq!(fields.sample_count, 512);
                return;
            }
        }
        panic!("no SH packet");
    }
}
