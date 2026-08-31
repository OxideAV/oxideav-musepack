//! oxideav-core integration: the registry entry point and the direct
//! `make_decoder` / `make_encoder` factories (the crate's dual-API
//! convention — both the `oxideav_core::register!` path and
//! directly-callable factories).
//!
//! The [`Decoder`] implementation is a whole-stream decoder: Musepack
//! `.mpc` files are single continuous streams (SV7 has no packet
//! framing a demuxer could split on without decoding), so packets are
//! accumulated and the stream is decoded either when the accumulated
//! bytes already form a complete file or at [`Decoder::flush`]. Decoded
//! PCM is emitted as interleaved [`oxideav_core::SampleFormat::S16`]
//! frames of up to 1152 samples per channel.
//!
//! Both stream generations decode through the magic dispatch
//! ([`crate::mpc_decode::decode_mpc_stream`]) at absolute loudness,
//! each corpus-validated to ±1 LSB against black-box reference
//! decodes (SV7: [`crate::sv7_file_decode::decode_sv7_file`], r390;
//! SV8: [`crate::sv8_decode::decode_sv8_stream`], r419/r429).
//!
//! The [`oxideav_core::Encoder`] implementation (round 429; typed
//! options + SV7 output round 454) is a whole-stream from-PCM
//! encoder for **both generations**: S16 interleaved frames in, one
//! complete `MPCK` (default) or `MP+` (`sv=7`) stream out at flush
//! (the stream totals are only known once the input ends). The
//! [`MusepackEncoderOptions`] schema exposes the generation switch,
//! the quality / step allocation knobs, M/S posture, `max_band`,
//! `block_power`, CNS threshold, and profile tag through
//! `CodecParameters::options`.

use std::collections::VecDeque;

use oxideav_core::{
    parse_options, AudioFrame, CodecCapabilities, CodecId, CodecInfo, CodecOptionsStruct,
    CodecParameters, Decoder, Frame, OptionField, OptionKind, OptionValue, Packet, RuntimeContext,
};

use crate::mpc_decode::decode_mpc_stream_tagged;
use crate::SAMPLES_PER_FRAME_PER_CHANNEL;

/// The registry codec id for Musepack (both stream generations).
pub const MUSEPACK_CODEC_ID: &str = "musepack";

/// Whole-stream Musepack decoder (see the module docs).
struct MpcStreamDecoder {
    codec_id: CodecId,
    /// Accumulated compressed bytes (a whole `.mpc` stream, possibly
    /// split across packets).
    buffer: Vec<u8>,
    /// Cap on `buffer` growth, from the caller's `DecoderLimits`.
    max_input_bytes: u64,
    /// Decoded-but-not-yet-emitted frames.
    pending: VecDeque<AudioFrame>,
    /// Set once the accumulated stream has been decoded (no more input
    /// is accepted for this stream).
    decoded: bool,
    /// Set by `flush`.
    flushed: bool,
}

impl MpcStreamDecoder {
    fn new(params: &CodecParameters) -> Self {
        Self {
            codec_id: params.codec_id.clone(),
            buffer: Vec::new(),
            max_input_bytes: params.limits.max_alloc_bytes_per_frame,
            pending: VecDeque::new(),
            decoded: false,
            flushed: false,
        }
    }

    /// Decode the accumulated stream and queue its PCM as S16
    /// interleaved frames of up to 1152 samples per channel.
    fn decode_buffer(&mut self) -> oxideav_core::Result<()> {
        let out = decode_mpc_stream_tagged(&self.buffer)
            .map_err(|e| oxideav_core::Error::invalid(e.to_string()))?;
        let channels = usize::from(out.channels().max(1));
        let pcm = out.pcm();
        let per_frame = SAMPLES_PER_FRAME_PER_CHANNEL * channels;
        for chunk in pcm.chunks(per_frame) {
            let mut data = Vec::with_capacity(chunk.len() * 2);
            for &v in chunk {
                let s = v.round().clamp(f64::from(i16::MIN), f64::from(i16::MAX)) as i16;
                data.extend_from_slice(&s.to_le_bytes());
            }
            self.pending.push_back(AudioFrame {
                samples: (chunk.len() / channels) as u32,
                pts: None,
                data: vec![data],
            });
        }
        self.decoded = true;
        Ok(())
    }
}

impl Decoder for MpcStreamDecoder {
    fn codec_id(&self) -> &CodecId {
        &self.codec_id
    }

    fn send_packet(&mut self, packet: &Packet) -> oxideav_core::Result<()> {
        if self.decoded {
            return Err(oxideav_core::Error::invalid(
                "musepack: stream already decoded; reset before feeding a new stream",
            ));
        }
        if (self.buffer.len() + packet.data.len()) as u64 > self.max_input_bytes {
            return Err(oxideav_core::Error::resource_exhausted(format!(
                "musepack: accumulated input exceeds the {}-byte limit",
                self.max_input_bytes
            )));
        }
        self.buffer.extend_from_slice(&packet.data);
        Ok(())
    }

    fn receive_frame(&mut self) -> oxideav_core::Result<Frame> {
        if let Some(frame) = self.pending.pop_front() {
            return Ok(Frame::Audio(frame));
        }
        if self.decoded || self.flushed && self.buffer.is_empty() {
            return Err(oxideav_core::Error::Eof);
        }
        if !self.buffer.is_empty() {
            // Try an eager whole-stream decode: succeeds when the caller
            // delivered a complete file in the packets so far.
            match self.decode_buffer() {
                Ok(()) => {}
                // A truncated stream just needs more packets — unless
                // the caller already flushed, in which case it is a
                // genuine error. A buffer still inside a leading ID3v2
                // block (tag pass-through) reports a magic failure
                // until the wrapped stream arrives — same treatment.
                Err(oxideav_core::Error::InvalidData(msg))
                    if !self.flushed
                        && (msg.contains("unexpected end of input")
                            || (self.buffer.starts_with(b"ID3")
                                && msg.contains("does not start with the SV7"))) =>
                {
                    return Err(oxideav_core::Error::NeedMore);
                }
                Err(e) => return Err(e),
            }
            if let Some(frame) = self.pending.pop_front() {
                return Ok(Frame::Audio(frame));
            }
            return Err(oxideav_core::Error::Eof);
        }
        Err(oxideav_core::Error::NeedMore)
    }

    fn flush(&mut self) -> oxideav_core::Result<()> {
        self.flushed = true;
        Ok(())
    }

    fn reset(&mut self) -> oxideav_core::Result<()> {
        self.buffer.clear();
        self.pending.clear();
        self.decoded = false;
        self.flushed = false;
        Ok(())
    }
}

/// Typed encoder options (the crate's [`CodecOptionsStruct`] schema),
/// parsed once at [`make_encoder`] from `CodecParameters::options`.
/// Every knob mirrors a field of
/// [`crate::sv8_file_encode::Sv8EncoderSettings`] /
/// [`crate::sv7_pcm_encode::Sv7EncoderSettings`]; consumers that know
/// the typed structs at compile time can call the whole-stream
/// encoders directly instead.
#[derive(Debug, Clone, PartialEq)]
pub struct MusepackEncoderOptions {
    /// Stream generation to emit: `7` (`MP+`) or `8` (`MPCK`, the
    /// default).
    pub sv: u8,
    /// Perceptual quality `0..=10` switching to the SMR-driven
    /// allocation ([`crate::smr_alloc`]); `None` = flat allocation.
    pub quality: Option<f64>,
    /// Flat s16-domain step target (used when `quality` is unset).
    pub step_target: f64,
    /// Stream-wide M/S posture.
    pub stream_ms: bool,
    /// Highest coded subband (`1..=31`).
    pub max_band: u8,
    /// SV8 `SH` block power (`0..=7`); ignored for SV7 output.
    pub block_power: u8,
    /// Noise-substitution threshold (`0.0` = CNS emission off).
    pub pns_threshold: f64,
    /// Informational profile tag; `None` = the generation's default
    /// (80 for SV8, 10 for SV7).
    pub profile: Option<u8>,
}

impl Default for MusepackEncoderOptions {
    fn default() -> Self {
        Self {
            sv: 8,
            quality: None,
            step_target: 2.0,
            stream_ms: true,
            max_band: 31,
            block_power: 3,
            pns_threshold: 0.0,
            profile: None,
        }
    }
}

impl CodecOptionsStruct for MusepackEncoderOptions {
    const SCHEMA: &'static [OptionField] = &[
        OptionField {
            name: "sv",
            kind: OptionKind::U32,
            default: OptionValue::U32(8),
            help: "stream generation to emit: 7 (MP+) or 8 (MPCK)",
        },
        OptionField {
            name: "quality",
            kind: OptionKind::F32,
            default: OptionValue::F32(-1.0),
            help: "perceptual quality 0-10 (SMR allocation); negative = flat step allocation",
        },
        OptionField {
            name: "step",
            kind: OptionKind::F32,
            default: OptionValue::F32(2.0),
            help: "flat quantisation-step target in s16 LSBs (when quality is unset)",
        },
        OptionField {
            name: "ms",
            kind: OptionKind::Bool,
            default: OptionValue::Bool(true),
            help: "stream-wide mid/side posture",
        },
        OptionField {
            name: "max_band",
            kind: OptionKind::U32,
            default: OptionValue::U32(31),
            help: "highest coded subband (1-31)",
        },
        OptionField {
            name: "block_power",
            kind: OptionKind::U32,
            default: OptionValue::U32(3),
            help: "SV8 audio-block size exponent (0-7); ignored for sv=7",
        },
        OptionField {
            name: "pns",
            kind: OptionKind::F32,
            default: OptionValue::F32(0.0),
            help: "noise-substitution threshold in s16 subband-peak units (0 = off)",
        },
        OptionField {
            name: "profile",
            kind: OptionKind::U32,
            default: OptionValue::U32(0),
            help: "informational profile tag (omit for the generation default)",
        },
    ];

    fn apply(&mut self, key: &str, value: &OptionValue) -> oxideav_core::Result<()> {
        match key {
            "sv" => {
                let v = value.as_u32()?;
                if v != 7 && v != 8 {
                    return Err(oxideav_core::Error::invalid(format!(
                        "musepack: option 'sv' must be 7 or 8, got {v}"
                    )));
                }
                self.sv = v as u8;
            }
            "quality" => {
                let q = value.as_f32()?;
                self.quality = if q < 0.0 {
                    None
                } else {
                    Some(f64::from(q).min(10.0))
                };
            }
            "step" => {
                let s = value.as_f32()?;
                if !s.is_finite() || s <= 0.0 {
                    return Err(oxideav_core::Error::invalid(
                        "musepack: option 'step' must be positive",
                    ));
                }
                self.step_target = f64::from(s);
            }
            "ms" => self.stream_ms = value.as_bool()?,
            "max_band" => {
                let v = value.as_u32()?;
                if !(1..=31).contains(&v) {
                    return Err(oxideav_core::Error::invalid(format!(
                        "musepack: option 'max_band' must be 1..=31, got {v}"
                    )));
                }
                self.max_band = v as u8;
            }
            "block_power" => {
                let v = value.as_u32()?;
                if v > 7 {
                    return Err(oxideav_core::Error::invalid(format!(
                        "musepack: option 'block_power' must be 0..=7, got {v}"
                    )));
                }
                self.block_power = v as u8;
            }
            "pns" => {
                let v = value.as_f32()?;
                if v < 0.0 {
                    return Err(oxideav_core::Error::invalid(
                        "musepack: option 'pns' must be non-negative",
                    ));
                }
                self.pns_threshold = f64::from(v);
            }
            "profile" => {
                let v = value.as_u32()?;
                if v > 127 {
                    return Err(oxideav_core::Error::invalid(format!(
                        "musepack: option 'profile' must fit 7 bits, got {v}"
                    )));
                }
                self.profile = if v == 0 { None } else { Some(v as u8) };
            }
            _ => unreachable!("guarded by SCHEMA"),
        }
        Ok(())
    }
}

/// Whole-stream SV8 **encoder** (round 429): accumulates interleaved
/// S16 PCM frames and, at [`Encoder::flush`], runs the from-PCM SV8
/// pipeline ([`crate::sv8_file_encode::encode_sv8_from_pcm_s16`]) and
/// emits the complete `MPCK` stream as a single packet. Whole-stream
/// because a Musepack stream's `SH` header carries the total sample
/// count up front — the totals are only known once the input ends.
struct MpcStreamEncoder {
    codec_id: CodecId,
    output_params: CodecParameters,
    sample_freq_index: u8,
    channels: u8,
    /// Parsed typed options (stream generation, allocation, CNS…).
    opts: MusepackEncoderOptions,
    /// Accumulated interleaved input samples.
    pcm: Vec<i16>,
    /// The encoded stream, produced at flush and drained by
    /// `receive_packet`.
    encoded: Option<Vec<u8>>,
    flushed: bool,
}

impl MpcStreamEncoder {
    fn new(params: &CodecParameters) -> oxideav_core::Result<Self> {
        let sample_rate = params
            .sample_rate
            .ok_or_else(|| oxideav_core::Error::invalid("musepack: sample_rate is required"))?;
        let sample_freq_index = crate::sh_header::SV8_SAMPLE_RATES
            .iter()
            .position(|&r| r == sample_rate)
            .ok_or_else(|| {
                oxideav_core::Error::unsupported(format!(
                    "musepack: sample rate {sample_rate} Hz is not one of {:?}",
                    crate::sh_header::SV8_SAMPLE_RATES
                ))
            })? as u8;
        let channels = params.channels.unwrap_or(2);
        if !(1..=2).contains(&channels) {
            return Err(oxideav_core::Error::unsupported(format!(
                "musepack: {channels} channels (only mono and stereo are wired)"
            )));
        }
        if let Some(fmt) = params.sample_format {
            if fmt != oxideav_core::SampleFormat::S16 {
                return Err(oxideav_core::Error::unsupported(format!(
                    "musepack: encoder input must be S16 interleaved, got {fmt:?}"
                )));
            }
        }
        let opts: MusepackEncoderOptions = parse_options(&params.options)?;
        if opts.sv == 7 && channels != 2 {
            return Err(oxideav_core::Error::unsupported(
                "musepack: SV7 output is stereo-only (the MP+ frame body always codes two \
                 channels and decodes as stereo); feed 2 channels or use sv=8",
            ));
        }
        let mut output_params = CodecParameters::audio(params.codec_id.clone());
        output_params.sample_rate = Some(sample_rate);
        output_params.channels = Some(channels);
        output_params.sample_format = Some(oxideav_core::SampleFormat::S16);
        Ok(Self {
            codec_id: params.codec_id.clone(),
            output_params,
            sample_freq_index,
            channels: channels as u8,
            opts,
            pcm: Vec::new(),
            encoded: None,
            flushed: false,
        })
    }
}

impl oxideav_core::Encoder for MpcStreamEncoder {
    fn codec_id(&self) -> &CodecId {
        &self.codec_id
    }

    fn output_params(&self) -> &CodecParameters {
        &self.output_params
    }

    fn send_frame(&mut self, frame: &Frame) -> oxideav_core::Result<()> {
        if self.flushed {
            return Err(oxideav_core::Error::invalid(
                "musepack: encoder already flushed",
            ));
        }
        let Frame::Audio(af) = frame else {
            return Err(oxideav_core::Error::invalid("musepack: audio frames only"));
        };
        let data = af.data.first().ok_or_else(|| {
            oxideav_core::Error::invalid("musepack: audio frame carries no plane")
        })?;
        let want = af.samples as usize * usize::from(self.channels) * 2;
        if data.len() < want {
            return Err(oxideav_core::Error::invalid(format!(
                "musepack: frame declares {} samples but carries {} bytes",
                af.samples,
                data.len()
            )));
        }
        for pair in data[..want].chunks_exact(2) {
            self.pcm.push(i16::from_le_bytes([pair[0], pair[1]]));
        }
        Ok(())
    }

    fn receive_packet(&mut self) -> oxideav_core::Result<Packet> {
        match self.encoded.take() {
            Some(bytes) => {
                let rate = self.output_params.sample_rate.unwrap_or(44_100);
                Ok(Packet::new(
                    0,
                    oxideav_core::TimeBase::from_rate(rate),
                    bytes,
                ))
            }
            None if self.flushed => Err(oxideav_core::Error::Eof),
            None => Err(oxideav_core::Error::NeedMore),
        }
    }

    fn flush(&mut self) -> oxideav_core::Result<()> {
        if self.flushed {
            return Ok(());
        }
        self.flushed = true;
        let o = &self.opts;
        let bytes = if o.sv == 7 {
            let settings = crate::sv7_pcm_encode::Sv7EncoderSettings {
                step_target: o.step_target,
                stream_ms: o.stream_ms,
                max_band: o.max_band,
                profile: o.profile.unwrap_or(10),
                pns_threshold: o.pns_threshold,
                quality: o.quality,
            };
            crate::sv7_pcm_encode::encode_sv7_from_pcm_s16(
                &self.pcm,
                self.channels,
                self.sample_freq_index,
                &settings,
            )
            .map_err(|e| oxideav_core::Error::invalid(e.to_string()))?
            .bytes
        } else {
            let settings = crate::sv8_file_encode::Sv8EncoderSettings {
                step_target: o.step_target,
                stream_ms: o.stream_ms,
                max_band: o.max_band,
                block_power: o.block_power,
                profile: o.profile.unwrap_or(80),
                pns_threshold: o.pns_threshold,
                quality: o.quality,
            };
            crate::sv8_file_encode::encode_sv8_from_pcm_s16(
                &self.pcm,
                self.channels,
                self.sample_freq_index,
                &settings,
            )
            .map_err(|e| oxideav_core::Error::invalid(e.to_string()))?
            .bytes
        };
        self.encoded = Some(bytes);
        Ok(())
    }
}

/// Direct encoder factory — the crate's dual-API encoder endpoint,
/// also installed as the registry's encoder factory. Produces SV8
/// (`MPCK`) streams from S16 interleaved PCM.
///
/// # Errors
///
/// [`oxideav_core::Error::InvalidData`] / `Unsupported` for a missing
/// or non-SV8 sample rate, an unsupported channel count, or a
/// non-S16 sample format.
pub fn make_encoder(
    params: &CodecParameters,
) -> oxideav_core::Result<Box<dyn oxideav_core::Encoder>> {
    Ok(Box::new(MpcStreamEncoder::new(params)?))
}

/// Direct decoder factory — the crate's historical-signature endpoint,
/// also installed as the registry's decoder factory.
///
/// # Errors
///
/// Infallible today (construction defers all validation to the decode
/// calls); kept fallible per the [`oxideav_core::DecoderFactory`]
/// contract.
pub fn make_decoder(params: &CodecParameters) -> oxideav_core::Result<Box<dyn Decoder>> {
    Ok(Box::new(MpcStreamDecoder::new(params)))
}

/// Install the Musepack codec into the runtime registry.
pub fn register(ctx: &mut RuntimeContext) {
    let mut caps = CodecCapabilities::audio("musepack_sw");
    caps.decode = true;
    caps.encode = true;
    caps.lossy = true;
    ctx.codecs.register(
        CodecInfo::new(CodecId::new(MUSEPACK_CODEC_ID))
            .capabilities(caps)
            .decoder(make_decoder)
            .encoder(make_encoder),
    );
}

oxideav_core::register!("musepack", register);

#[cfg(test)]
mod tests {
    use super::*;

    fn params() -> CodecParameters {
        CodecParameters::audio(CodecId::new(MUSEPACK_CODEC_ID))
    }

    fn packet(data: Vec<u8>) -> Packet {
        Packet::new(0, oxideav_core::TimeBase::from_rate(44_100), data)
    }

    /// A small complete SV7 stream via the crate's own writer.
    fn sv7_stream() -> Vec<u8> {
        use crate::sv7_file_encode::{encode_sv7_file, Sv7EncStereoFrame};
        use crate::sv7_header::Sv7HeaderFields;
        let hdr = Sv7HeaderFields {
            frame_count: 2,
            max_band: 3,
            profile: 10,
            sample_freq_index: 0,
            ..Default::default()
        };
        encode_sv7_file(&hdr, &vec![Sv7EncStereoFrame::silent(4); 2]).unwrap()
    }

    #[test]
    fn factory_builds_a_decoder_with_the_requested_id() {
        let d = make_decoder(&params()).unwrap();
        assert_eq!(d.codec_id().as_str(), MUSEPACK_CODEC_ID);
    }

    #[test]
    fn whole_file_packet_decodes_to_frames() {
        let mut d = make_decoder(&params()).unwrap();
        d.send_packet(&packet(sv7_stream())).unwrap();
        let mut frames = 0;
        loop {
            match d.receive_frame() {
                Ok(Frame::Audio(a)) => {
                    assert_eq!(a.samples, 1152);
                    assert_eq!(a.data.len(), 1);
                    assert_eq!(a.data[0].len(), 1152 * 2 * 2);
                    frames += 1;
                }
                Ok(_) => panic!("expected audio frames"),
                Err(oxideav_core::Error::Eof) => break,
                Err(e) => panic!("{e}"),
            }
        }
        assert_eq!(frames, 2);
    }

    #[test]
    fn split_packets_need_flush_or_completion() {
        let raw = sv7_stream();
        let (a, b) = raw.split_at(raw.len() / 2);
        let mut d = make_decoder(&params()).unwrap();
        d.send_packet(&packet(a.to_vec())).unwrap();
        assert!(matches!(
            d.receive_frame(),
            Err(oxideav_core::Error::NeedMore)
        ));
        d.send_packet(&packet(b.to_vec())).unwrap();
        d.flush().unwrap();
        assert!(matches!(d.receive_frame(), Ok(Frame::Audio(_))));
    }

    #[test]
    fn id3v2_prefixed_stream_decodes_through_the_registry() {
        // Tag pass-through (headers-and-coding §9.2/§9.3): a stream
        // wrapped in a leading ID3v2 block and a trailing APEv2-shaped
        // tail decodes as if untagged.
        let mut tagged = b"ID3\x04\x00\x00opaque-tag-bytes".to_vec();
        tagged.extend_from_slice(&sv7_stream());
        tagged.extend_from_slice(b"APETAGEX\xd0\x07\x00\x00tail");
        let mut d = make_decoder(&params()).unwrap();
        d.send_packet(&packet(tagged)).unwrap();
        d.flush().unwrap();
        let mut frames = 0;
        while let Ok(Frame::Audio(_)) = d.receive_frame() {
            frames += 1;
        }
        assert_eq!(frames, 2);
    }

    #[test]
    fn garbage_input_is_invalid_data() {
        let mut d = make_decoder(&params()).unwrap();
        d.send_packet(&packet(b"not a musepack stream".to_vec()))
            .unwrap();
        d.flush().unwrap();
        assert!(matches!(
            d.receive_frame(),
            Err(oxideav_core::Error::InvalidData(_))
        ));
    }

    #[test]
    fn reset_accepts_a_new_stream() {
        let mut d = make_decoder(&params()).unwrap();
        d.send_packet(&packet(sv7_stream())).unwrap();
        assert!(matches!(d.receive_frame(), Ok(Frame::Audio(_))));
        Decoder::reset(&mut *d).unwrap();
        d.send_packet(&packet(sv7_stream())).unwrap();
        assert!(matches!(d.receive_frame(), Ok(Frame::Audio(_))));
    }

    #[test]
    fn registry_lookup_finds_the_decoder() {
        let mut ctx = RuntimeContext::default();
        register(&mut ctx);
        assert!(ctx.codecs.has_decoder(&CodecId::new(MUSEPACK_CODEC_ID)));
        let d = ctx.codecs.first_decoder(&params()).expect("registered");
        assert_eq!(d.codec_id().as_str(), MUSEPACK_CODEC_ID);
    }

    #[test]
    fn entry_point_symbol_registers() {
        let mut ctx = RuntimeContext::default();
        crate::__oxideav_entry(&mut ctx);
        assert!(ctx.codecs.first_decoder(&params()).is_ok());
    }

    fn encoder_params(rate: u32, channels: u16) -> CodecParameters {
        let mut p = CodecParameters::audio(CodecId::new(MUSEPACK_CODEC_ID));
        p.sample_rate = Some(rate);
        p.channels = Some(channels);
        p.sample_format = Some(oxideav_core::SampleFormat::S16);
        p
    }

    /// Encoder → decoder round trip through the registry surfaces: an
    /// S16 sine goes in as audio frames, one `MPCK` packet comes out
    /// at flush, and the packet decodes back through `make_decoder` to
    /// the same sample count with real (non-silent) content.
    #[test]
    fn encoder_round_trips_through_the_registry_decoder() {
        let mut e = make_encoder(&encoder_params(44_100, 1)).unwrap();
        assert_eq!(e.codec_id().as_str(), MUSEPACK_CODEC_ID);
        assert_eq!(e.output_params().sample_rate, Some(44_100));

        let n = 3_000usize;
        let pcm: Vec<i16> = (0..n)
            .map(|i| (10_000.0 * (0.05 * i as f64).sin()) as i16)
            .collect();
        let mut data = Vec::with_capacity(2 * n);
        for &s in &pcm {
            data.extend_from_slice(&s.to_le_bytes());
        }
        e.send_frame(&Frame::Audio(AudioFrame {
            samples: n as u32,
            pts: None,
            data: vec![data],
        }))
        .unwrap();
        assert!(matches!(
            e.receive_packet(),
            Err(oxideav_core::Error::NeedMore)
        ));
        e.flush().unwrap();
        let pkt = e.receive_packet().unwrap();
        assert_eq!(&pkt.data[..4], b"MPCK");
        assert!(matches!(e.receive_packet(), Err(oxideav_core::Error::Eof)));

        let mut d = make_decoder(&params()).unwrap();
        d.send_packet(&pkt).unwrap();
        let mut decoded = 0usize;
        let mut nonzero = false;
        loop {
            match d.receive_frame() {
                Ok(Frame::Audio(af)) => {
                    decoded += af.samples as usize;
                    nonzero |= af.data[0].iter().any(|&b| b != 0);
                }
                Ok(_) => panic!("audio frames only"),
                Err(oxideav_core::Error::Eof) => break,
                Err(e) => panic!("decode: {e}"),
            }
        }
        assert_eq!(decoded, n, "gapless sample count through the registry");
        assert!(nonzero, "decoded audio must not be silence");
    }

    fn opt_params(rate: u32, channels: u16, opts: &[(&str, &str)]) -> CodecParameters {
        let mut p = encoder_params(rate, channels);
        for (k, v) in opts {
            p.options.insert(*k, *v);
        }
        p
    }

    fn sine_frame(n: usize, channels: usize) -> Frame {
        let mut data = Vec::with_capacity(2 * n * channels);
        for i in 0..n {
            let s = (9000.0 * (0.07 * i as f64).sin()) as i16;
            for _ in 0..channels {
                data.extend_from_slice(&s.to_le_bytes());
            }
        }
        Frame::Audio(AudioFrame {
            samples: n as u32,
            pts: None,
            data: vec![data],
        })
    }

    fn encode_all(params: &CodecParameters, frame: &Frame) -> Packet {
        let mut e = make_encoder(params).unwrap();
        e.send_frame(frame).unwrap();
        e.flush().unwrap();
        e.receive_packet().unwrap()
    }

    /// The `sv=7` option emits an `MP+` stream that round-trips
    /// through the registry decoder at the exact gapless length.
    #[test]
    fn sv7_option_emits_mp_plus_and_round_trips() {
        let n = 3_000usize;
        let frame = sine_frame(n, 2);
        let pkt = encode_all(&opt_params(44_100, 2, &[("sv", "7")]), &frame);
        assert_eq!(&pkt.data[..3], b"MP+");

        let mut d = make_decoder(&params()).unwrap();
        d.send_packet(&pkt).unwrap();
        let mut decoded = 0usize;
        loop {
            match d.receive_frame() {
                Ok(Frame::Audio(af)) => decoded += af.samples as usize,
                Ok(_) => panic!("audio frames only"),
                Err(oxideav_core::Error::Eof) => break,
                Err(e) => panic!("decode: {e}"),
            }
        }
        assert_eq!(decoded, n, "gapless sample count");
    }

    /// The quality knob reaches the registry: a coarse quality
    /// shrinks the packet against the flat default, for both
    /// generations.
    #[test]
    fn quality_option_scales_the_rate() {
        let frame = sine_frame(6_000, 2);
        for sv in ["7", "8"] {
            let flat = encode_all(&opt_params(44_100, 2, &[("sv", sv)]), &frame);
            let coarse = encode_all(
                &opt_params(44_100, 2, &[("sv", sv), ("quality", "3")]),
                &frame,
            );
            assert!(
                coarse.data.len() < flat.data.len(),
                "sv{sv}: quality 3 ({}) must undercut flat ({})",
                coarse.data.len(),
                flat.data.len()
            );
        }
    }

    #[test]
    fn invalid_options_are_rejected_at_construction() {
        for bad in [
            ("sv", "9"),
            ("max_band", "0"),
            ("max_band", "32"),
            ("block_power", "9"),
            ("step", "0"),
            ("pns", "-1"),
            ("profile", "128"),
            ("nonsense", "1"),
        ] {
            assert!(
                make_encoder(&opt_params(44_100, 2, &[bad])).is_err(),
                "expected rejection for {bad:?}"
            );
        }
        // SV7 output is stereo-only through the registry.
        assert!(make_encoder(&opt_params(44_100, 1, &[("sv", "7")])).is_err());
        // Mono SV8 with options stays fine.
        assert!(make_encoder(&opt_params(44_100, 1, &[("sv", "8"), ("quality", "5")])).is_ok());
    }

    #[test]
    fn encoder_rejects_unsupported_shapes() {
        assert!(make_encoder(&params()).is_err(), "missing sample rate");
        assert!(
            make_encoder(&encoder_params(11_025, 2)).is_err(),
            "non-SV8 rate"
        );
        assert!(
            make_encoder(&encoder_params(44_100, 3)).is_err(),
            "channel count"
        );
        let mut p = encoder_params(44_100, 2);
        p.sample_format = Some(oxideav_core::SampleFormat::F32);
        assert!(make_encoder(&p).is_err(), "sample format");
    }

    #[test]
    fn registry_lookup_finds_the_encoder() {
        let mut ctx = RuntimeContext::default();
        register(&mut ctx);
        let e = ctx
            .codecs
            .first_encoder(&encoder_params(44_100, 2))
            .expect("registered encoder");
        assert_eq!(e.codec_id().as_str(), MUSEPACK_CODEC_ID);
    }
}
