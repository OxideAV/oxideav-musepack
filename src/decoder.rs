//! `oxideav_core::Decoder` shim — accepts whole-file Musepack streams
//! (or pre-demuxed `AP` packets) and emits `AudioFrame`s.
//!
//! Two modes:
//!
//! * **Full-file** — the first `send_packet` carries the entire file
//!   starting at the `MPCK` magic. We walk chunks, parse `SH`,
//!   construct an [`Sv8Decoder`], then decode each `AP` chunk on
//!   subsequent `receive_frame` calls.
//!
//! * **Pre-demuxed** — the caller has parsed the container externally
//!   and feeds raw `AP` payloads with the codec parameters carrying
//!   the `SH` extradata. (Not yet wired in — the full-file path is
//!   the default.)

use oxideav_core::{
    AudioFrame, CodecId, CodecParameters, Decoder, Error, Frame, Packet, Result,
};

use crate::container::{parse_sh, ChunkIter, ChunkTag, MAGIC_SV7, MAGIC_SV8};
use crate::sv8::Sv8Decoder;

/// Build a Musepack decoder. The codec parameters are consulted for
/// the canonical `codec_id` only — every stream property (sample
/// rate, channel count, block_power, …) is derived from the in-band
/// `SH` chunk.
pub fn make_decoder(params: &CodecParameters) -> Result<Box<dyn Decoder>> {
    Ok(Box::new(MusepackDecoder {
        codec_id: params.codec_id.clone(),
        sv8: None,
        pending_packets: Vec::new(),
        cursor: 0,
        sample_rate: 0,
        channels: 0,
        eof: false,
    }))
}

struct MusepackDecoder {
    codec_id: CodecId,
    sv8: Option<Sv8Decoder>,
    /// Queue of `AP` payloads pending decode (each is a Vec<u8> copy
    /// of the chunk's payload bytes — the input packet's lifetime
    /// ends after `send_packet` returns).
    pending_packets: Vec<Vec<u8>>,
    /// Next packet index to decode.
    cursor: usize,
    sample_rate: u32,
    channels: u8,
    eof: bool,
}

impl Decoder for MusepackDecoder {
    fn codec_id(&self) -> &CodecId {
        &self.codec_id
    }

    fn send_packet(&mut self, packet: &Packet) -> Result<()> {
        // First packet must contain the full file (or at least the
        // header up through SH). Subsequent packets are appended as
        // raw AP payloads (the pre-demuxed form, not yet supported).
        if self.sv8.is_some() {
            return Err(Error::Unsupported(
                "musepack: post-init packet feeding not implemented".into(),
            ));
        }
        let data = &packet.data;
        if data.len() >= 4 && data[0..4] == MAGIC_SV8 {
            self.init_sv8(&data[4..])?;
        } else if data.len() >= 3 && data[0..3] == MAGIC_SV7 {
            return Err(Error::Unsupported(
                "musepack: SV7 (MP+) decode not yet implemented".into(),
            ));
        } else {
            return Err(Error::invalid(
                "musepack: input does not start with MPCK or MP+ magic",
            ));
        }
        Ok(())
    }

    fn receive_frame(&mut self) -> Result<Frame> {
        let dec = match self.sv8.as_mut() {
            Some(d) => d,
            None => {
                return if self.eof {
                    Err(Error::Eof)
                } else {
                    Err(Error::NeedMore)
                }
            }
        };
        if self.cursor >= self.pending_packets.len() {
            return if self.eof {
                Err(Error::Eof)
            } else {
                Err(Error::NeedMore)
            };
        }
        let payload = &self.pending_packets[self.cursor];
        self.cursor += 1;

        let mut pcm: Vec<Vec<f32>> = (0..self.channels).map(|_| Vec::new()).collect();
        dec.decode_packet(payload, &mut pcm)?;

        // Interleave to s16.
        let total_samples = pcm.first().map(|p| p.len()).unwrap_or(0);
        let mut bytes = Vec::with_capacity(total_samples * self.channels as usize * 2);
        for i in 0..total_samples {
            for ch in 0..(self.channels as usize) {
                let f = pcm[ch][i].clamp(-1.0, 1.0);
                let s = (f * 32767.0) as i16;
                bytes.extend_from_slice(&s.to_le_bytes());
            }
        }
        Ok(Frame::Audio(AudioFrame {
            samples: total_samples as u32,
            pts: None,
            data: vec![bytes],
        }))
    }

    fn flush(&mut self) -> Result<()> {
        self.eof = true;
        Ok(())
    }

    fn reset(&mut self) -> Result<()> {
        self.sv8 = None;
        self.pending_packets.clear();
        self.cursor = 0;
        self.eof = false;
        Ok(())
    }
}

impl MusepackDecoder {
    /// Walk an SV8 file (after the 4-byte `MPCK` magic), build the
    /// `Sv8Decoder`, queue the `AP` payloads.
    fn init_sv8(&mut self, after_magic: &[u8]) -> Result<()> {
        let mut header = None;
        let iter = ChunkIter::new(after_magic);
        for chunk_res in iter {
            let chunk = chunk_res?;
            match chunk.tag {
                ChunkTag::Sh => {
                    let sh = parse_sh(chunk.payload)?;
                    self.sample_rate = sh.sample_rate;
                    self.channels = sh.channels;
                    header = Some(sh);
                }
                ChunkTag::Ap => {
                    if header.is_none() {
                        return Err(Error::invalid(
                            "musepack SV8: AP chunk before SH",
                        ));
                    }
                    self.pending_packets.push(chunk.payload.to_vec());
                }
                ChunkTag::Se => {
                    self.eof = true;
                    break;
                }
                // RG / EI / SO / ST / CT / Other are pure side-data
                // and don't influence audio decode (see trace report
                // §6.3). Skip silently.
                _ => {}
            }
        }
        let header = header.ok_or_else(|| {
            Error::invalid("musepack SV8: no SH chunk found before EOF")
        })?;
        self.sv8 = Some(Sv8Decoder::new(header));
        Ok(())
    }
}
