//! oxideav-core integration: the registry entry point and the direct
//! `make_decoder` factory (the crate's dual-API convention — both the
//! `oxideav_core::register!` path and a directly-callable factory).
//!
//! The [`Decoder`] implementation is a whole-stream decoder: Musepack
//! `.mpc` files are single continuous streams (SV7 has no packet
//! framing a demuxer could split on without decoding), so packets are
//! accumulated and the stream is decoded either when the accumulated
//! bytes already form a complete file or at [`Decoder::flush`]. Decoded
//! PCM is emitted as interleaved [`oxideav_core::SampleFormat::S16`]
//! frames of up to 1152 samples per channel.
//!
//! SV7 decode is the corpus-validated path
//! ([`crate::sv7_file_decode::decode_sv7_file`], ±1 LSB vs the FFmpeg
//! oracle); SV8 input routes to the grounded mono subset
//! ([`crate::sv8_decode`], relative loudness — its absolute anchor is
//! still a docs gap) through the same magic dispatch
//! ([`crate::mpc_decode::decode_mpc_stream`]).

use std::collections::VecDeque;

use oxideav_core::{
    AudioFrame, CodecCapabilities, CodecId, CodecInfo, CodecParameters, Decoder, Frame, Packet,
    RuntimeContext,
};

use crate::mpc_decode::decode_mpc_stream;
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
        let out = decode_mpc_stream(&self.buffer, 0)
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
                // genuine error.
                Err(oxideav_core::Error::InvalidData(msg))
                    if !self.flushed && msg.contains("unexpected end of input") =>
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
    caps.lossy = true;
    ctx.codecs.register(
        CodecInfo::new(CodecId::new(MUSEPACK_CODEC_ID))
            .capabilities(caps)
            .decoder(make_decoder),
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
}
