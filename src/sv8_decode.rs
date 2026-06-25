//! SV8 packet-stream → audio-frame integration (mono keyframe path).
//!
//! This is the first wiring of the SV8 **packet layer**
//! ([`crate::packet_stream`] / [`crate::typed_packet`]) to the SV8
//! **audio decode** ([`crate::sv8_stream`]): it walks an `MPCK` stream,
//! reads the `SH` stream header for the decode parameters, and drives an
//! [`crate::sv8_stream::Sv8MonoStreamDecoder`] over each `AP` packet,
//! producing PCM for the supported subset.
//!
//! # Supported subset (and why)
//!
//! The fully-grounded SV8 audio decode is the **mono, single-frame-per-`AP`,
//! key-frame** path:
//!
//! - **Mono** — SV8's per-channel band interleaving is a DOCS-GAP (see
//!   [`crate::sv8_stream`] / [`crate::sv8_frame_decode`]), so a stereo
//!   stream cannot be decoded sample-exact yet.
//! - **`block_power == 0`** — one frame per `AP` packet
//!   ([`crate::sh_header::StreamHeaderFields::frames_per_audio_packet`]
//!   `== 1`). With multiple frames per packet the §6.2 per-frame
//!   `Max_used_Band` read position and the key→non-key transition inside
//!   a packet are not pinned cell-for-cell.
//! - **Key-frame `AP`** — each `AP` frame is treated as a key frame: it
//!   reads its own `Max_used_Band` via the grounded §6.2 keyframe log
//!   code ([`crate::sv8_band_header::decode_keyframe_max_used_band`]) and
//!   codes scalefactors absolutely (`new_block = true`). SV8's keyframe
//!   design (§3.3) is built precisely so a decoder can start at any
//!   `AP` boundary, so decoding every `AP` as an independent key frame
//!   is the conservative grounded behaviour.
//!
//! A stream outside this subset is rejected with a precise error
//! ([`Error::ChannelCountInvalid`] for non-mono,
//! [`Error::UnsupportedBlockPower`] for `block_power != 0`) rather than
//! decoded wrong. The stereo / multi-frame-packet paths wait on the
//! channel-loop + per-frame-`Max_used_Band` DOCS-GAPs.
//!
//! # What is grounded here
//!
//! - Packet walking (§3.1 framing, the inclusive varint size §3) — the
//!   already-grounded [`crate::packet_stream`].
//! - `SH` field map (§2) — [`crate::sh_header`].
//! - Per-`AP` key-frame `Max_used_Band` (§6.2 log code) +
//!   per-frame body decode + reconstruction + synthesis —
//!   [`crate::sv8_stream`] over the grounded sub-walks.
//!
//! Source-of-record (facts only):
//!
//! - `docs/audio/musepack/musepack-sv7-sv8-spec.md` §3.1 (`MPCK` packet
//!   stream), §3.2 (`SH` / `AP` packet kinds), §3.3 (keyframes).
//! - `docs/audio/musepack/spec/musepack-headers-and-coding.md` §2
//!   (`SH` field map, `block_power` → frames-per-`AP`), §3 (varint),
//!   §6.2 (keyframe `Max_used_Band`).

use crate::framing::parse_sv8_magic;
use crate::huffman::Sv7BitReader;
use crate::packet_stream::{PacketSizeConvention, PacketStream};
use crate::sh_header::StreamHeaderFields;
use crate::sv8_band_header::decode_keyframe_max_used_band;
use crate::sv8_stream::{Sv8FrameParams, Sv8MonoStreamDecoder};
use crate::typed_packet::TypedPacket;
use crate::{Error, Result, SAMPLES_PER_FRAME_PER_CHANNEL};

/// The result of decoding the supported subset of an SV8 stream: the
/// `SH` header fields and the concatenated mono PCM of every decoded
/// `AP` packet.
#[derive(Debug, Clone, PartialEq)]
pub struct Sv8DecodedStream {
    /// The decoded `SH` stream-header fields (§2).
    pub header: StreamHeaderFields,
    /// Number of `AP` packets decoded.
    pub audio_packets: u64,
    /// The concatenated mono PCM (one value per decoded sample;
    /// `audio_packets × `[`SAMPLES_PER_FRAME_PER_CHANNEL`] long for the
    /// one-frame-per-`AP` subset).
    pub pcm: Vec<f64>,
}

/// Decode the supported subset (mono, `block_power == 0`, key-frame `AP`)
/// of a complete `MPCK`-prefixed SV8 byte buffer into mono PCM.
///
/// Walks the packet stream once: the first `SH` packet supplies
/// `max_band` / `channels` / `block_power`; every `AP` packet is decoded
/// as one key frame through a persistent
/// [`Sv8MonoStreamDecoder`] (so the synthesis overlap and CNS PRNG thread
/// across packets); non-audio packets (`RG` / `EI` / `SO` / `ST` /
/// `SE` / unknown) are skipped. Decoding stops at the `SE` terminator or
/// end of input.
///
/// `anchor` is the §2.6 absolute SCF anchor (GAP; pass `0` for the
/// relative-loudness convention).
///
/// # Errors
///
/// - [`Error::InvalidMagic`] if the buffer does not start with `MPCK`.
/// - [`Error::UnexpectedEof`] if a packet is truncated.
/// - [`Error::NotImplemented`] if no `SH` packet is found before the
///   first `AP` (the header is required to parameterise the decode).
/// - [`Error::ChannelCountInvalid`] if the stream is not mono.
/// - [`Error::UnsupportedBlockPower`] if `block_power != 0`.
/// - Every error of the `SH` field map and the per-`AP` audio decode.
pub fn decode_sv8_mono_stream(input: &[u8], anchor: i32) -> Result<Sv8DecodedStream> {
    let after_magic = parse_sv8_magic(input)?;
    let mut stream = PacketStream::new(&input[after_magic..], PacketSizeConvention::Inclusive);

    let mut header: Option<StreamHeaderFields> = None;
    let mut decoder: Option<Sv8MonoStreamDecoder> = None;
    let mut audio_packets: u64 = 0;
    let mut pcm: Vec<f64> = Vec::new();

    while let Some(packet) = stream.next_packet()? {
        match TypedPacket::classify(packet) {
            TypedPacket::StreamHeader(sh) => {
                let fields = sh.fields()?;
                if fields.channels != 1 {
                    return Err(Error::ChannelCountInvalid(fields.channels));
                }
                if fields.block_power != 0 {
                    return Err(Error::UnsupportedBlockPower(fields.block_power));
                }
                decoder = Some(Sv8MonoStreamDecoder::new(anchor));
                header = Some(fields);
            }
            TypedPacket::Audio(ap) => {
                let fields = header.as_ref().ok_or(Error::NotImplemented)?;
                let dec = decoder.as_mut().ok_or(Error::NotImplemented)?;
                // §6.2 key-frame: read this frame's Max_used_Band via the
                // bounded log code, then decode the body as a key frame.
                //
                // The MSB-first bit reader always peeks a full 16-bit
                // look-ahead window, so the *last* VLC of an exactly-sized
                // AP payload needs trailing bytes to peek into. In a live
                // stream those are the following packet's bytes; here the
                // payload is its own slice, so pad it with the two
                // zero bytes the reader would otherwise read past the end
                // (the bits are consumed only if the codeword needs them —
                // a valid prefix code never over-consumes).
                let mut framed = ap.payload_bytes().to_vec();
                framed.extend_from_slice(&[0, 0]);
                let mut reader = Sv7BitReader::new(&framed);
                let nbands = decode_keyframe_max_used_band(&mut reader, fields.max_band)?;
                let frame = dec.decode_frame(
                    &mut reader,
                    Sv8FrameParams {
                        nbands,
                        new_block: true,
                    },
                )?;
                pcm.extend_from_slice(&frame);
                audio_packets += 1;
            }
            TypedPacket::StreamEnd(_) => break,
            // RG / EI / SO / ST / Unknown — skipped (metadata / GAP).
            _ => {}
        }
    }

    let header = header.ok_or(Error::NotImplemented)?;
    debug_assert_eq!(
        pcm.len(),
        audio_packets as usize * SAMPLES_PER_FRAME_PER_CHANNEL
    );
    Ok(Sv8DecodedStream {
        header,
        audio_packets,
        pcm,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sv8_huffman::{Sv8CanonicalTable, SV8_RES_1_TABLE};

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

    /// Build an `SH` payload for a mono, block_power-0 stream with the
    /// given `max_band`.
    fn sh_payload(max_band: u8) -> Vec<u8> {
        // §2 layout: CRC(32) ver(8) sample_count(varint) silence(varint)
        // then packed16 [freq:3, max_band-1:5, channels-1:4, ms:1, bp:3].
        let mut out = Vec::new();
        out.extend_from_slice(&[0, 0, 0, 0]); // CRC (unvalidated)
        out.push(8); // stream version
        out.push(0); // sample_count varint = 0
        out.push(0); // beginning_silence varint = 0
                     // packed16 fields: freq idx 0, channels-1 = 0 (mono), ms = 0,
                     // block_power = 0 — all but max_band-1 are zero, so only the
                     // max_band-1 field (bits 8..=12) is set.
        let packed: u16 = ((max_band - 1) as u16 & 0x1F) << 8;
        out.push((packed >> 8) as u8);
        out.push((packed & 0xFF) as u8);
        out
    }

    /// Wrap a packet: 2-byte key + inclusive varint size + payload.
    fn packet(key: &[u8; 2], payload: &[u8]) -> Vec<u8> {
        // Inclusive size = 2 (key) + size_bytes + payload. For payloads
        // small enough the size fits one varint byte (size < 128), so
        // size_bytes = 1 and total = 3 + payload.len().
        let total = 2 + 1 + payload.len();
        assert!(total < 128, "test payloads stay single-varint-byte");
        let mut out = Vec::new();
        out.extend_from_slice(key);
        out.push(total as u8); // single-byte varint (high bit clear)
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

    /// Build a keyframe `AP` payload that decodes to `nbands` empty bands.
    /// Uses the grounded keyframe log-code by searching for the bit prefix
    /// that reads back as `nbands`, then appends `nbands` res-1 sym-0
    /// codewords (empty bands).
    fn keyframe_ap(max_band: u8, nbands: u8) -> Vec<u8> {
        // Search a short bit prefix whose decode_keyframe_max_used_band
        // over 0..max_band+1 yields `nbands`. The log code reads at most
        // ceil(log2(max_band+1))+1 bits, so an 8-bit brute force suffices
        // for the small max_band values used in tests.
        let (res0_code, res0_len) = codeword_for_symbol(&SV8_RES_1_TABLE, 0).expect("res-1 sym 0");
        for nbits in 1u8..=8 {
            for value in 0u16..(1 << nbits) {
                let mut p = BitPacker::new();
                p.push(value << (16 - nbits), nbits);
                for _ in 0..nbands {
                    p.push(res0_code, res0_len);
                }
                let mut bytes = p.into_bytes();
                bytes.push(0);
                bytes.push(0);
                let mut r = Sv7BitReader::new(&bytes);
                if let Ok(decoded) = decode_keyframe_max_used_band(&mut r, max_band) {
                    if decoded == nbands {
                        // Re-pack without the extra peek padding bytes; the
                        // packet walker provides following-packet bytes as
                        // padding in a real stream, and a trailing SE keeps
                        // the reader fed in tests.
                        let mut p2 = BitPacker::new();
                        p2.push(value << (16 - nbits), nbits);
                        for _ in 0..nbands {
                            p2.push(res0_code, res0_len);
                        }
                        return p2.into_bytes();
                    }
                }
            }
        }
        panic!("no keyframe log-code prefix decodes to nbands={nbands}");
    }

    #[test]
    fn decodes_mono_keyframe_stream_to_silent_pcm() {
        let max_band = 4;
        let sh = packet(b"SH", &sh_payload(max_band));
        // Two AP packets, each one silent keyframe (some peek padding is
        // provided by the following packet bytes in the contiguous stream).
        let ap_body = keyframe_ap(max_band, 3);
        let ap1 = packet(b"AP", &ap_body);
        let ap2 = packet(b"AP", &ap_body);
        let se = packet(b"SE", &[]);
        let buf = mpck_stream(&[sh, ap1, ap2, se]);

        let out = decode_sv8_mono_stream(&buf, 0).unwrap();
        assert_eq!(out.header.channels, 1);
        assert_eq!(out.header.max_band, max_band);
        assert_eq!(out.header.block_power, 0);
        assert_eq!(out.audio_packets, 2);
        assert_eq!(out.pcm.len(), 2 * SAMPLES_PER_FRAME_PER_CHANNEL);
        assert!(out.pcm.iter().all(|&s| s == 0.0));
    }

    #[test]
    fn rejects_non_mpck() {
        let err = decode_sv8_mono_stream(b"NOPE....", 0).err();
        assert_eq!(err, Some(Error::InvalidMagic));
    }

    #[test]
    fn rejects_stereo_stream() {
        // Hand-build an SH with channels-1 = 1 (stereo).
        let mut payload = sh_payload(4);
        // The packed16 tail is the last two bytes; set channels-1 = 1.
        let len = payload.len();
        let packed = ((payload[len - 2] as u16) << 8) | payload[len - 1] as u16;
        let packed = (packed & !(0xF << 4)) | (1u16 << 4); // channels-1 = 1
        payload[len - 2] = (packed >> 8) as u8;
        payload[len - 1] = (packed & 0xFF) as u8;
        let sh = packet(b"SH", &payload);
        let buf = mpck_stream(&[sh]);
        assert_eq!(
            decode_sv8_mono_stream(&buf, 0).err(),
            Some(Error::ChannelCountInvalid(2))
        );
    }

    #[test]
    fn rejects_nonzero_block_power() {
        let mut payload = sh_payload(4);
        let len = payload.len();
        let packed = ((payload[len - 2] as u16) << 8) | payload[len - 1] as u16;
        let packed = (packed & !0x7) | 1u16; // block_power = 1
        payload[len - 2] = (packed >> 8) as u8;
        payload[len - 1] = (packed & 0xFF) as u8;
        let sh = packet(b"SH", &payload);
        let buf = mpck_stream(&[sh]);
        assert_eq!(
            decode_sv8_mono_stream(&buf, 0).err(),
            Some(Error::UnsupportedBlockPower(1))
        );
    }

    #[test]
    fn audio_before_header_is_rejected() {
        // An AP packet with no preceding SH.
        let ap = packet(b"AP", &keyframe_ap(4, 1));
        let buf = mpck_stream(&[ap]);
        assert_eq!(
            decode_sv8_mono_stream(&buf, 0).err(),
            Some(Error::NotImplemented)
        );
    }
}
