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
//! The [`oxideav_core::Encoder`] implementation (round 429) is the
//! whole-stream from-PCM **SV8 encoder**
//! ([`crate::sv8_file_encode`]): S16 interleaved frames in, one
//! complete `MPCK` stream out at flush (the `SH` totals are only
//! known once the input ends).

use std::collections::VecDeque;

use oxideav_core::{
    AudioFrame, CodecCapabilities, CodecId, CodecInfo, CodecParameters, Decoder, Frame, Packet,
    RuntimeContext,
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
        let mut output_params = CodecParameters::audio(params.codec_id.clone());
        output_params.sample_rate = Some(sample_rate);
        output_params.channels = Some(channels);
        output_params.sample_format = Some(oxideav_core::SampleFormat::S16);
        Ok(Self {
            codec_id: params.codec_id.clone(),
            output_params,
            sample_freq_index,
            channels: channels as u8,
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
        let enc = crate::sv8_file_encode::encode_sv8_from_pcm_s16(
            &self.pcm,
            self.channels,
            self.sample_freq_index,
            &crate::sv8_file_encode::Sv8EncoderSettings::default(),
        )
        .map_err(|e| oxideav_core::Error::invalid(e.to_string()))?;
        self.encoded = Some(enc.bytes);
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
