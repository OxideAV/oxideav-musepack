//! Unified `.mpc` whole-stream decode entry — magic-dispatched over the
//! two stream generations.
//!
//! A Musepack file starts with either the SV7 `MP+` magic or the SV8
//! `MPCK` magic ([`crate::framing::identify_stream`]). This module is
//! the one call that routes a raw buffer to the matching whole-stream
//! decoder:
//!
//! - **SV7** → [`crate::sv7_file_decode::decode_sv7_file`] (stereo; the
//!   §1 header, the §1.1 continuous audio run, gapless trim).
//! - **SV8** → [`crate::sv8_decode::decode_sv8_mono_stream`] (the
//!   grounded SV8 subset: mono, `block_power == 0`, key-frame `AP`
//!   packets; out-of-subset streams are rejected with precise errors).
//!
//! The two §2.6 DOCS-GAP knobs thread through unchanged: `anchor` (the
//! absolute SCF anchor) reaches both paths, and the M/S-undo closure
//! reaches the SV7 path (the SV8 subset is mono, so it has no M/S
//! step). Output is [`MpcDecodedStream`], which surfaces the common
//! queries (PCM run, channel count, sample rate) without erasing the
//! per-generation detail.
//!
//! Source-of-record: `docs/audio/musepack/musepack-sv7-sv8-spec.md` §1
//! (the two magics / stream generations). No new format facts — pure
//! dispatch over the two already-grounded whole-stream decoders.

use crate::framing::{identify_stream, StreamKind};
use crate::sv7_file_decode::{decode_sv7_file, Sv7DecodedFile};
use crate::sv8_decode::{decode_sv8_mono_stream, Sv8DecodedStream};
use crate::Result;

/// A decoded Musepack stream of either generation.
#[derive(Debug, Clone, PartialEq)]
pub enum MpcDecodedStream {
    /// An SV7 (`MP+`) stereo stream.
    Sv7(Sv7DecodedFile),
    /// An SV8 (`MPCK`) stream (grounded subset: mono).
    Sv8(Sv8DecodedStream),
}

impl MpcDecodedStream {
    /// Which stream generation was decoded.
    #[must_use]
    pub fn kind(&self) -> StreamKind {
        match self {
            MpcDecodedStream::Sv7(_) => StreamKind::Sv7,
            MpcDecodedStream::Sv8(_) => StreamKind::Sv8,
        }
    }

    /// The decoded PCM run: interleaved `L, R, …` for SV7, mono for
    /// SV8. Relative loudness (the absolute SCF anchor is GAP).
    #[must_use]
    pub fn pcm(&self) -> &[f64] {
        match self {
            MpcDecodedStream::Sv7(f) => &f.pcm,
            MpcDecodedStream::Sv8(s) => &s.pcm,
        }
    }

    /// Channel count: always 2 for SV7 (§1 derived fact); the `SH`
    /// header's channel field for SV8 (1 within the grounded subset).
    #[must_use]
    pub fn channels(&self) -> u8 {
        match self {
            MpcDecodedStream::Sv7(f) => f.header.channels(),
            MpcDecodedStream::Sv8(s) => s.header.channels,
        }
    }

    /// The stream's sample rate in Hz, or `None` for an index outside
    /// the four defined rates.
    #[must_use]
    pub fn sample_rate_hz(&self) -> Option<u32> {
        match self {
            MpcDecodedStream::Sv7(f) => f.header.sample_rate_hz(),
            MpcDecodedStream::Sv8(s) => s.header.sample_rate_hz(),
        }
    }
}

/// Decode a complete `.mpc` buffer of either stream generation.
///
/// `anchor` is the §2.6 absolute-SCF-anchor GAP knob (pass 0 for the
/// relative convention); `undo` is the §2.6 M/S-undo arithmetic GAP
/// closure (applied on the SV7 path only — the grounded SV8 subset is
/// mono).
///
/// # Errors
///
/// - [`crate::Error::InvalidMagic`] if `bytes` starts with neither
///   magic.
/// - Every error of the routed whole-stream decoder
///   ([`decode_sv7_file`] / [`decode_sv8_mono_stream`]).
pub fn decode_mpc_stream<U>(bytes: &[u8], anchor: u8, undo: U) -> Result<MpcDecodedStream>
where
    U: Fn(f64, f64) -> (f64, f64),
{
    match identify_stream(bytes)? {
        StreamKind::Sv7 => Ok(MpcDecodedStream::Sv7(decode_sv7_file(bytes, anchor, undo)?)),
        StreamKind::Sv8 => Ok(MpcDecodedStream::Sv8(decode_sv8_mono_stream(
            bytes,
            i32::from(anchor),
        )?)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sv7_file_encode::{encode_sv7_file, Sv7EncStereoFrame};
    use crate::sv7_header::Sv7HeaderFields;
    use crate::Error;

    fn test_undo(m: f64, s: f64) -> (f64, f64) {
        (m + s, m - s)
    }

    fn sv7_file() -> (Sv7HeaderFields, Vec<u8>) {
        let hdr = Sv7HeaderFields {
            frame_count: 2,
            max_band: 3,
            profile: 10,
            sample_freq_index: 1,
            ..Default::default()
        };
        let frames = vec![Sv7EncStereoFrame::silent(4); 2];
        let raw = encode_sv7_file(&hdr, &frames, 0).unwrap();
        (hdr, raw)
    }

    /// A minimal SV8 stream: `MPCK` + one `SH` packet (mono,
    /// `block_power == 0`), no audio packets.
    fn sv8_stream() -> Vec<u8> {
        // §2 SH payload: CRC(32) ver(8) sample_count(varint)
        // silence(varint) packed16 [freq:3, max_band-1:5, ch-1:4, ms:1,
        // bp:3] — everything zero except max_band-1 (max_band 5 → 4).
        let mut payload = vec![0, 0, 0, 0, 8, 0, 0];
        let packed: u16 = (4u16 & 0x1F) << 8;
        payload.push((packed >> 8) as u8);
        payload.push((packed & 0xFF) as u8);
        // §3 packet: key + inclusive one-byte varint size + payload.
        let mut out = b"MPCK".to_vec();
        out.extend_from_slice(b"SH");
        out.push((2 + 1 + payload.len()) as u8);
        out.extend_from_slice(&payload);
        out
    }

    #[test]
    fn sv7_magic_routes_to_the_file_decoder() {
        let (hdr, raw) = sv7_file();
        let out = decode_mpc_stream(&raw, 0, test_undo).unwrap();
        assert_eq!(out.kind(), StreamKind::Sv7);
        assert_eq!(out.channels(), 2);
        assert_eq!(out.sample_rate_hz(), Some(48000));
        assert_eq!(out.pcm().len(), 2 * 2 * 1152);
        match out {
            MpcDecodedStream::Sv7(f) => {
                assert_eq!(f.header, hdr);
                let direct = crate::sv7_file_decode::decode_sv7_file(&raw, 0, test_undo).unwrap();
                assert_eq!(f, direct);
            }
            MpcDecodedStream::Sv8(_) => panic!("expected SV7"),
        }
    }

    #[test]
    fn sv8_magic_routes_to_the_packet_decoder() {
        let raw = sv8_stream();
        let out = decode_mpc_stream(&raw, 0, test_undo).unwrap();
        assert_eq!(out.kind(), StreamKind::Sv8);
        assert_eq!(out.channels(), 1);
        assert_eq!(out.sample_rate_hz(), Some(44100));
        assert!(out.pcm().is_empty(), "no AP packets, no PCM");
    }

    #[test]
    fn unknown_magic_is_rejected() {
        assert_eq!(
            decode_mpc_stream(b"RIFFxxxx", 0, test_undo),
            Err(Error::InvalidMagic),
        );
        // Too short for either magic: the framing layer reports
        // starvation rather than a magic mismatch.
        assert_eq!(
            decode_mpc_stream(b"", 0, test_undo),
            Err(Error::UnexpectedEof),
        );
    }

    #[test]
    fn sv7_decode_errors_propagate_through_the_dispatch() {
        let (_, mut raw) = sv7_file();
        raw.truncate(10); // valid magic, truncated header
        assert_eq!(
            decode_mpc_stream(&raw, 0, test_undo),
            Err(Error::UnexpectedEof),
        );
    }
}
