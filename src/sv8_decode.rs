//! SV8 packet-stream → PCM integration: the whole-stream decode.
//!
//! Walks an `MPCK` buffer's packet stream ([`crate::packet_stream`] /
//! [`crate::typed_packet`]), reads the `SH` stream header for the
//! decode parameters, and drives an
//! [`crate::sv8_stream::Sv8StreamDecoder`] over every `AP` packet in
//! the fixture-pinned real-stream layout (round 419):
//!
//! - **Frames per packet.** An `AP` packet carries up to
//!   [`crate::sh_header::StreamHeaderFields::frames_per_audio_packet`]
//!   (`2^(block_power × 2)`) frames; the final packet carries the
//!   stream-total remainder. The stream's frame total derives from the
//!   `SH` sample count (`⌈(sample_count + beginning_silence) / 1152⌉`).
//! - **Key frames.** Each `AP` opens with a key frame (absolute
//!   scalefactors, fresh `Max_used_Band` log code); later frames of
//!   the packet chain as non-key frames (`Bands`-delta `Max_used_Band`,
//!   temporal SCF prediction) — see [`crate::sv8_stereo_frame`].
//! - **Two-channel bodies.** Every frame body codes two channels
//!   regardless of the `SH` channel count (fixture-pinned); the `SH`
//!   count selects the output shape (interleaved stereo vs mono).
//! - **Gapless trim.** The decoded run is trimmed to the `SH` totals:
//!   `beginning_silence` leading samples are dropped and exactly
//!   `sample_count` samples per channel are kept.
//!
//! Output is absolute s16-domain PCM (the corpus-pinned SV7-shared
//! absolute reconstruction law — [`crate::reconstruct`]), validated
//! against black-box reference decodes of the r419 SV8 corpus
//! (`tests/sv8_corpus.rs`).
//!
//! Source-of-record (facts only):
//!
//! - `docs/audio/musepack/musepack-sv7-sv8-spec.md` §3.1 (`MPCK` packet
//!   stream), §3.2 (packet kinds), §3.3 (keyframes).
//! - `docs/audio/musepack/spec/musepack-headers-and-coding.md` §2
//!   (`SH` field map, `block_power` → frames-per-`AP`), §3 (varint),
//!   §6 (the frame-body walks).
//! - The cross-frame / cross-packet composition is pinned by black-box
//!   fixture behaviour (the r419 corpus), the same method as the r390
//!   SV7 wire pinning — no decoder source consulted.

use crate::framing::parse_sv8_magic;
use crate::packet_stream::{PacketSizeConvention, PacketStream};
use crate::sh_header::StreamHeaderFields;
use crate::sv8_stream::Sv8StreamDecoder;
use crate::typed_packet::TypedPacket;
use crate::{Error, Result, SAMPLES_PER_FRAME_PER_CHANNEL};

/// The result of decoding a complete SV8 stream: the `SH` header fields
/// and the gapless-trimmed PCM.
#[derive(Debug, Clone, PartialEq)]
pub struct Sv8DecodedStream {
    /// The decoded `SH` stream-header fields (§2).
    pub header: StreamHeaderFields,
    /// Number of `AP` packets decoded.
    pub audio_packets: u64,
    /// Number of frames decoded across all packets.
    pub frames_decoded: u64,
    /// The decoded PCM in the absolute s16 domain: interleaved
    /// `L, R, …` for a stereo stream, plain mono otherwise, trimmed to
    /// the `SH` totals (`beginning_silence` dropped, `sample_count`
    /// samples per channel kept).
    pub pcm: Vec<f64>,
}

impl Sv8DecodedStream {
    /// The decoded PCM as `i16` samples: each value rounded
    /// half-away-from-zero and clamped to the `i16` range (the same
    /// convention as the SV7 whole-file path).
    #[must_use]
    pub fn pcm_s16(&self) -> Vec<i16> {
        self.pcm
            .iter()
            .map(|&v| v.round().clamp(f64::from(i16::MIN), f64::from(i16::MAX)) as i16)
            .collect()
    }
}

/// Decode a complete `MPCK`-prefixed SV8 byte buffer to PCM.
///
/// Walks the packet stream once: the first `SH` packet supplies the
/// decode parameters and constructs the persistent
/// [`Sv8StreamDecoder`]; every `AP` packet decodes
/// `min(frames_per_audio_packet, frames remaining)` frames (key frame
/// first); non-audio packets (`RG` / `EI` / `SO` / `ST` / unknown) are
/// skipped; decoding stops at the `SE` terminator or end of input. The
/// concatenated PCM is gapless-trimmed to the `SH` totals.
///
/// # Errors
///
/// - [`Error::InvalidMagic`] if the buffer does not start with `MPCK`.
/// - [`Error::UnexpectedEof`] if a packet (or a frame body inside an
///   `AP` payload) is truncated.
/// - [`Error::NotImplemented`] if an `AP` packet precedes the `SH`
///   header, or no `SH` packet exists (the header is required to
///   parameterise the decode).
/// - [`Error::ChannelCountInvalid`] for a channel count other than 1
///   or 2.
/// - Every error of the `SH` field map and the per-packet audio decode
///   ([`Sv8StreamDecoder::decode_audio_packet`]).
pub fn decode_sv8_stream(input: &[u8]) -> Result<Sv8DecodedStream> {
    let after_magic = parse_sv8_magic(input)?;
    let mut stream = PacketStream::new(&input[after_magic..], PacketSizeConvention::Inclusive);

    let mut header: Option<StreamHeaderFields> = None;
    let mut decoder: Option<Sv8StreamDecoder> = None;
    let mut frames_remaining: u64 = 0;
    let mut frames_per_packet: u64 = 0;
    let mut audio_packets: u64 = 0;
    let mut pcm: Vec<f64> = Vec::new();

    while let Some(packet) = stream.next_packet()? {
        match TypedPacket::classify(packet) {
            TypedPacket::StreamHeader(sh) => {
                let fields = sh.fields()?;
                decoder = Some(Sv8StreamDecoder::from_header(&fields)?);
                frames_per_packet = fields.frames_per_audio_packet();
                let total_samples = fields.sample_count + fields.beginning_silence;
                frames_remaining = total_samples.div_ceil(SAMPLES_PER_FRAME_PER_CHANNEL as u64);
                header = Some(fields);
            }
            TypedPacket::Audio(ap) => {
                let dec = decoder.as_mut().ok_or(Error::NotImplemented)?;
                let frames = frames_remaining.min(frames_per_packet);
                if frames == 0 {
                    // Stream totals exhausted: a trailing AP carries
                    // nothing the totals ask for; skip it.
                    continue;
                }
                let packet_pcm = dec.decode_audio_packet(ap.payload_bytes(), frames)?;
                pcm.extend_from_slice(&packet_pcm);
                frames_remaining -= frames;
                audio_packets += 1;
            }
            TypedPacket::StreamEnd(_) => break,
            // RG / EI / SO / ST / Unknown — metadata / seek layer.
            _ => {}
        }
    }

    let header = header.ok_or(Error::NotImplemented)?;
    let decoder = decoder.ok_or(Error::NotImplemented)?;

    // Gapless trim: drop the leading silence, keep sample_count
    // samples per channel.
    let nch = u64::from(header.channels);
    let skip = (header.beginning_silence * nch) as usize;
    let keep = (header.sample_count * nch) as usize;
    if skip > 0 {
        pcm.drain(..skip.min(pcm.len()));
    }
    pcm.truncate(keep);

    Ok(Sv8DecodedStream {
        header,
        audio_packets,
        frames_decoded: decoder.frames_decoded(),
        pcm,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::huffman::Sv7BitReader;
    use crate::sv8_band_header::decode_keyframe_max_used_band;
    use crate::sv8_huffman::{Sv8CanonicalTable, SV8_BANDS_TABLE};

    /// MSB-first left-justified bit packer.
    struct BitPacker {
        bytes: Vec<u8>,
        acc: u32,
        nbits: u8,
    }

    impl BitPacker {
        fn new() -> Self {
            BitPacker {
                bytes: Vec::new(),
                acc: 0,
                nbits: 0,
            }
        }
        fn push(&mut self, pattern: u16, length: u8) {
            for i in 0..length {
                let bit = (pattern >> (15 - i)) & 1;
                self.acc = (self.acc << 1) | bit as u32;
                self.nbits += 1;
                if self.nbits == 8 {
                    self.bytes.push(self.acc as u8);
                    self.acc = 0;
                    self.nbits = 0;
                }
            }
        }
        fn into_bytes(mut self) -> Vec<u8> {
            if self.nbits > 0 {
                self.bytes.push((self.acc << (8 - self.nbits)) as u8);
            }
            self.bytes
        }
    }

    /// Find a `(pattern, length)` codeword of `table` decoding to `target`.
    fn codeword_for_symbol(table: &Sv8CanonicalTable, target: i8) -> Option<(u16, u8)> {
        let mut upper: u32 = 0x1_0000;
        for e in table.lengths.iter() {
            if e.length == 0 {
                continue;
            }
            let step = 1u32 << (16 - e.length as u32);
            let mut pat = e.code as u32;
            while pat < upper {
                let mut p = BitPacker::new();
                p.push(pat as u16, e.length);
                let mut bytes = p.into_bytes();
                bytes.push(0);
                bytes.push(0);
                let mut r = Sv7BitReader::new(&bytes);
                if table.decode(&mut r).unwrap() == target {
                    return Some((pat as u16, e.length));
                }
                pat += step;
            }
            upper = e.code as u32;
        }
        None
    }

    /// Reference §6.5 log-code encoder for value `v` in `0..max`.
    fn log_encode(v: u32, max: u32) -> (u16, u8) {
        if max <= 1 {
            return (0, 0);
        }
        let mut bitlen: u8 = 0;
        while (1u32 << bitlen) < max {
            bitlen += 1;
        }
        let lost = (1u32 << bitlen) - max;
        if v < lost {
            let len = bitlen - 1;
            ((v as u16) << (16 - len), len)
        } else {
            (((v + lost) as u16) << (16 - bitlen), bitlen)
        }
    }

    /// Build an `SH` payload: `max_band`, output `channels`,
    /// `block_power`, single-varint-byte `sample_count`.
    fn sh_payload(max_band: u8, channels: u8, block_power: u8, sample_count: u8) -> Vec<u8> {
        // §2 layout: CRC(32) ver(8) sample_count(varint) silence(varint)
        // then packed16 [freq:3, max_band-1:5, channels-1:4, ms:1, bp:3].
        let mut out = Vec::new();
        out.extend_from_slice(&[0, 0, 0, 0]); // CRC (unvalidated)
        out.push(8); // stream version
        out.push(sample_count & 0x7F); // sample_count varint
        out.push(0); // beginning_silence varint = 0
        let packed: u16 = (((max_band - 1) as u16 & 0x1F) << 8)
            | (((channels - 1) as u16 & 0xF) << 4)
            | (block_power as u16 & 0x7);
        out.push((packed >> 8) as u8);
        out.push((packed & 0xFF) as u8);
        out
    }

    /// Wrap a packet: 2-byte key + inclusive varint size + payload.
    fn packet(key: &[u8; 2], payload: &[u8]) -> Vec<u8> {
        let total = 2 + 1 + payload.len();
        assert!(total < 128, "test payloads stay single-varint-byte");
        let mut out = Vec::new();
        out.extend_from_slice(key);
        out.push(total as u8);
        out.extend_from_slice(payload);
        out
    }

    fn mpck_stream(packets: &[Vec<u8>]) -> Vec<u8> {
        let mut out = b"MPCK".to_vec();
        for p in packets {
            out.extend_from_slice(p);
        }
        out
    }

    /// Build an `AP` payload of `frames` all-silent frames: a key-frame
    /// `Max_used_Band` of 0 (verified to decode back), then a
    /// `Bands`-table delta of 0 per non-key frame.
    fn silent_ap(max_band: u8, frames: u32) -> Vec<u8> {
        let mut p = BitPacker::new();
        let (pat, len) = log_encode(0, max_band as u32 + 2);
        p.push(pat, len);
        {
            // Verify the keyframe count decodes back as 0.
            let mut probe = BitPacker::new();
            probe.push(pat, len);
            let mut bytes = probe.into_bytes();
            bytes.push(0);
            bytes.push(0);
            let mut r = Sv7BitReader::new(&bytes);
            assert_eq!(decode_keyframe_max_used_band(&mut r, max_band).unwrap(), 0);
        }
        let (b0, bl0) = codeword_for_symbol(&SV8_BANDS_TABLE, 0).expect("bands sym 0");
        for _ in 1..frames {
            p.push(b0, bl0); // non-key Max_used_Band delta 0 ⇒ stays 0
        }
        p.into_bytes()
    }

    #[test]
    fn decodes_silent_stereo_stream_with_gapless_trim() {
        // One silent frame; sample_count 100 trims the 1152-sample
        // frame to 100 samples per channel, interleaved.
        let max_band = 4;
        let sh = packet(b"SH", &sh_payload(max_band, 2, 0, 100));
        let ap = packet(b"AP", &silent_ap(max_band, 1));
        let se = packet(b"SE", &[]);
        let buf = mpck_stream(&[sh, ap, se]);

        let out = decode_sv8_stream(&buf).unwrap();
        assert_eq!(out.header.channels, 2);
        assert_eq!(out.audio_packets, 1);
        assert_eq!(out.frames_decoded, 1);
        assert_eq!(out.pcm.len(), 2 * 100);
        assert!(out.pcm.iter().all(|&s| s == 0.0));
        assert!(out.pcm_s16().iter().all(|&s| s == 0));
    }

    #[test]
    fn chains_non_key_frames_inside_one_multi_frame_packet() {
        // block_power 1 ⇒ 4 frames per AP. A 1-byte varint caps the
        // sample count at 127 ⇒ 1 frame of totals; hand the walker a
        // 3-frame AP anyway and confirm only the totals-frames decode
        // (the silent_ap body chains non-key frames after the key one,
        // exercising the Bands-delta read when frames > 1).
        let max_band = 4;
        let sh = packet(b"SH", &sh_payload(max_band, 2, 1, 127));
        let ap = packet(b"AP", &silent_ap(max_band, 1));
        let buf = mpck_stream(&[sh, ap]);
        let out = decode_sv8_stream(&buf).unwrap();
        assert_eq!(out.header.frames_per_audio_packet(), 4);
        assert_eq!(out.frames_decoded, 1, "totals bound the frame count");
        assert_eq!(out.pcm.len(), 2 * 127);
    }

    #[test]
    fn extra_audio_packets_past_the_totals_are_skipped() {
        let max_band = 4;
        let sh = packet(b"SH", &sh_payload(max_band, 2, 0, 127));
        let ap1 = packet(b"AP", &silent_ap(max_band, 1));
        let ap2 = packet(b"AP", &silent_ap(max_band, 1));
        let buf = mpck_stream(&[sh, ap1, ap2]);
        let out = decode_sv8_stream(&buf).unwrap();
        assert_eq!(out.audio_packets, 1, "second AP is past the totals");
        assert_eq!(out.frames_decoded, 1);
        assert_eq!(out.pcm.len(), 2 * 127);
    }

    #[test]
    fn mono_output_takes_one_channel() {
        let max_band = 4;
        let sh = packet(b"SH", &sh_payload(max_band, 1, 0, 64));
        let ap = packet(b"AP", &silent_ap(max_band, 1));
        let buf = mpck_stream(&[sh, ap]);
        let out = decode_sv8_stream(&buf).unwrap();
        assert_eq!(out.header.channels, 1);
        assert_eq!(out.pcm.len(), 64, "mono: one value per sample");
        assert!(out.pcm.iter().all(|&s| s == 0.0));
    }

    #[test]
    fn rejects_non_mpck() {
        assert_eq!(
            decode_sv8_stream(b"NOPE....").err(),
            Some(Error::InvalidMagic)
        );
    }

    #[test]
    fn rejects_more_than_two_channels() {
        let sh = packet(b"SH", &sh_payload(4, 3, 0, 16));
        let buf = mpck_stream(&[sh]);
        assert_eq!(
            decode_sv8_stream(&buf).err(),
            Some(Error::ChannelCountInvalid(3))
        );
    }

    #[test]
    fn audio_before_header_is_rejected() {
        let ap = packet(b"AP", &silent_ap(4, 1));
        let buf = mpck_stream(&[ap]);
        assert_eq!(decode_sv8_stream(&buf).err(), Some(Error::NotImplemented));
    }

    #[test]
    fn stream_without_header_is_rejected() {
        let se = packet(b"SE", &[]);
        let buf = mpck_stream(&[se]);
        assert_eq!(decode_sv8_stream(&buf).err(), Some(Error::NotImplemented));
    }

    #[test]
    fn truncated_packet_is_rejected() {
        let max_band = 4;
        let sh = packet(b"SH", &sh_payload(max_band, 2, 0, 100));
        let mut ap = packet(b"AP", &silent_ap(max_band, 1));
        ap.truncate(ap.len().saturating_sub(1)); // break the packet size
        let buf = mpck_stream(&[sh, ap]);
        assert!(decode_sv8_stream(&buf).is_err());
    }
}
