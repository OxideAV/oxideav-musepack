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
//! - **SV8** → [`crate::sv8_decode::decode_sv8_stream`] (mono + stereo,
//!   multi-frame `AP` packets, key-frame chaining, gapless trim — the
//!   r419 fixture-pinned real-stream path).
//!
//! Both paths are knob-free and corpus-pinned: the absolute SCF law,
//! the M/S arithmetic, and the per-generation framing are all
//! fixture-validated (see [`crate::sv7_file_decode`] /
//! [`crate::sv8_decode`]). Output is
//! [`MpcDecodedStream`], which surfaces the common queries (PCM run,
//! channel count, sample rate) without erasing the per-generation
//! detail.
//!
//! Source-of-record: `docs/audio/musepack/musepack-sv7-sv8-spec.md` §1
//! (the two magics / stream generations). No new format facts — pure
//! dispatch over the two already-grounded whole-stream decoders.

use crate::framing::{identify_stream, StreamKind};
use crate::sv7_file_decode::{decode_sv7_file, Sv7DecodedFile};
use crate::sv8_decode::{decode_sv8_stream, Sv8DecodedStream};
use crate::Result;

/// A decoded Musepack stream of either generation.
#[derive(Debug, Clone, PartialEq)]
pub enum MpcDecodedStream {
    /// An SV7 (`MP+`) stereo stream.
    Sv7(Sv7DecodedFile),
    /// An SV8 (`MPCK`) stream (mono or stereo).
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

    /// The decoded PCM run in the corpus-pinned absolute s16 domain:
    /// interleaved `L, R, …` for stereo (always, for SV7), plain mono
    /// for a one-channel SV8 stream. SV8 output is gapless-trimmed to
    /// the `SH` totals.
    #[must_use]
    pub fn pcm(&self) -> &[f64] {
        match self {
            MpcDecodedStream::Sv7(f) => &f.pcm,
            MpcDecodedStream::Sv8(s) => &s.pcm,
        }
    }

    /// Channel count: always 2 for SV7 (§1 derived fact); the `SH`
    /// header's channel field for SV8 (1 or 2).
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
/// Both paths are knob-free and corpus-pinned (the former SV8
/// `sv8_anchor` GAP knob is gone: the r419 corpus pins the SV8
/// absolute law as the SV7-shared one).
///
/// # Errors
///
/// - [`crate::Error::InvalidMagic`] if `bytes` starts with neither
///   magic.
/// - Every error of the routed whole-stream decoder
///   ([`decode_sv7_file`] / [`decode_sv8_stream`]).
pub fn decode_mpc_stream(bytes: &[u8]) -> Result<MpcDecodedStream> {
    match identify_stream(bytes)? {
        StreamKind::Sv7 => Ok(MpcDecodedStream::Sv7(decode_sv7_file(bytes)?)),
        StreamKind::Sv8 => Ok(MpcDecodedStream::Sv8(decode_sv8_stream(bytes)?)),
    }
}

/// The 3-byte marker opening an ID3v2 tag block.
const ID3V2_MARKER: [u8; 3] = *b"ID3";

/// Cap on how many resync candidates [`decode_mpc_stream_tagged`]
/// will attempt a whole-stream decode at (each failed attempt is
/// fail-fast, but a hostile tag stuffed with magic look-alikes must
/// not turn into unbounded re-decodes).
const MAX_RESYNC_ATTEMPTS: usize = 16;

/// Positions in `bytes` (strictly after the ID3v2 marker) that look
/// like a Musepack stream start: the SV8 `MPCK` magic, or the SV7
/// `MP+` magic followed by a version byte with low nibble 7 (the §1
/// stream-version convention — `0x07` / `0x17`).
fn resync_candidates(bytes: &[u8]) -> impl Iterator<Item = usize> + '_ {
    (ID3V2_MARKER.len()..bytes.len().saturating_sub(3)).filter(|&i| {
        bytes[i..].starts_with(b"MPCK")
            || (bytes[i..].starts_with(b"MP+") && bytes[i + 3] & 0xF == 7)
    })
}

/// [`decode_mpc_stream`] with **tag pass-through**: a buffer whose
/// Musepack stream is wrapped in metadata tags decodes as if the tags
/// were absent.
///
/// - **Leading ID3v2 block** — headers-and-coding §9.2 defines the
///   stream's `header_position` as "the start of the stream after any
///   leading ID3v2 block". When `bytes` opens with the `ID3` marker
///   instead of a Musepack magic, this entry resyncs by scanning for
///   the first plausible stream start (`MPCK`, or `MP+` + version
///   nibble 7) that decodes successfully (bounded attempts). The tag
///   block itself is passed over, not parsed — tag *contents* belong
///   to the metadata sibling crates.
/// - **Trailing tags** — nothing to do here: both whole-stream
///   decoders already stop at their in-stream terminator (SV7: the §1.1
///   11-bit trailer + flush frame, with any tail ignored; SV8: the
///   `SE` packet — §9.3 places APEv2/ID3 tags after it, outside the
///   packet stream).
///
/// A buffer that already starts with a Musepack magic decodes
/// identically to [`decode_mpc_stream`].
///
/// # Errors
///
/// - [`crate::Error::InvalidMagic`] if `bytes` starts with neither a
///   Musepack magic nor an ID3v2 marker, or no resync candidate
///   decodes.
/// - Every error of the routed whole-stream decoder.
pub fn decode_mpc_stream_tagged(bytes: &[u8]) -> Result<MpcDecodedStream> {
    match decode_mpc_stream(bytes) {
        Ok(out) => Ok(out),
        Err(e) => {
            if !bytes.starts_with(&ID3V2_MARKER) {
                return Err(e);
            }
            let mut last = crate::Error::InvalidMagic;
            for (attempt, pos) in resync_candidates(bytes).enumerate() {
                if attempt >= MAX_RESYNC_ATTEMPTS {
                    break;
                }
                match decode_mpc_stream(&bytes[pos..]) {
                    Ok(out) => return Ok(out),
                    Err(e) => last = e,
                }
            }
            Err(last)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sv7_file_encode::{encode_sv7_file, Sv7EncStereoFrame};
    use crate::sv7_header::Sv7HeaderFields;
    use crate::Error;

    fn sv7_file() -> (Sv7HeaderFields, Vec<u8>) {
        let hdr = Sv7HeaderFields {
            frame_count: 2,
            max_band: 3,
            profile: 10,
            sample_freq_index: 1,
            ..Default::default()
        };
        let frames = vec![Sv7EncStereoFrame::silent(4); 2];
        let raw = encode_sv7_file(&hdr, &frames).unwrap();
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
        let out = decode_mpc_stream(&raw).unwrap();
        assert_eq!(out.kind(), StreamKind::Sv7);
        assert_eq!(out.channels(), 2);
        assert_eq!(out.sample_rate_hz(), Some(48000));
        assert_eq!(out.pcm().len(), 2 * 2 * 1152);
        match out {
            MpcDecodedStream::Sv7(f) => {
                assert_eq!(f.header, hdr);
                let direct = crate::sv7_file_decode::decode_sv7_file(&raw).unwrap();
                assert_eq!(f, direct);
            }
            MpcDecodedStream::Sv8(_) => panic!("expected SV7"),
        }
    }

    #[test]
    fn sv8_magic_routes_to_the_packet_decoder() {
        let raw = sv8_stream();
        let out = decode_mpc_stream(&raw).unwrap();
        assert_eq!(out.kind(), StreamKind::Sv8);
        assert_eq!(out.channels(), 1);
        assert_eq!(out.sample_rate_hz(), Some(44100));
        assert!(out.pcm().is_empty(), "no AP packets, no PCM");
    }

    #[test]
    fn unknown_magic_is_rejected() {
        assert_eq!(decode_mpc_stream(b"RIFFxxxx"), Err(Error::InvalidMagic));
        // Too short for either magic: the framing layer reports
        // starvation rather than a magic mismatch.
        assert_eq!(decode_mpc_stream(b""), Err(Error::UnexpectedEof));
    }

    #[test]
    fn sv7_decode_errors_propagate_through_the_dispatch() {
        let (_, mut raw) = sv7_file();
        raw.truncate(10); // valid magic, truncated header
        assert_eq!(decode_mpc_stream(&raw), Err(Error::UnexpectedEof));
    }

    // ─── tag pass-through ───────────────────────────────────

    /// A fake ID3v2-shaped leading block: the `ID3` marker plus
    /// opaque bytes (the tagged entry never parses the block, so the
    /// content past the marker is arbitrary).
    fn id3_prefix(len: usize) -> Vec<u8> {
        let mut out = b"ID3\x04\x00\x00".to_vec();
        out.extend((0..len).map(|i| (i * 37 + 5) as u8));
        out
    }

    #[test]
    fn tagged_entry_is_transparent_for_untagged_streams() {
        let (_, raw) = sv7_file();
        assert_eq!(
            decode_mpc_stream_tagged(&raw).unwrap(),
            decode_mpc_stream(&raw).unwrap()
        );
    }

    #[test]
    fn leading_id3v2_block_is_skipped_sv7() {
        let (_, raw) = sv7_file();
        let mut tagged = id3_prefix(301);
        tagged.extend_from_slice(&raw);
        let out = decode_mpc_stream_tagged(&tagged).unwrap();
        assert_eq!(out, decode_mpc_stream(&raw).unwrap());
    }

    #[test]
    fn leading_id3v2_block_is_skipped_sv8() {
        let raw = sv8_stream();
        let mut tagged = id3_prefix(64);
        tagged.extend_from_slice(&raw);
        let out = decode_mpc_stream_tagged(&tagged).unwrap();
        assert_eq!(out.kind(), StreamKind::Sv8);
        assert_eq!(out.channels(), 1);
    }

    #[test]
    fn resync_skips_magic_lookalikes_inside_the_tag() {
        // A tag block containing a decoy `MPCK` (backed by garbage)
        // and a decoy `MP+\x07` before the real SV7 stream: the
        // bounded resync walks past both.
        let (_, raw) = sv7_file();
        let mut tagged = id3_prefix(16);
        tagged.extend_from_slice(b"MPCKgarbage");
        tagged.extend_from_slice(b"MP+\x07nope");
        tagged.extend_from_slice(&raw);
        let out = decode_mpc_stream_tagged(&tagged).unwrap();
        assert_eq!(out, decode_mpc_stream(&raw).unwrap());
    }

    #[test]
    fn tagged_entry_rejects_non_tag_garbage() {
        assert_eq!(
            decode_mpc_stream_tagged(b"RIFFxxxxxxxx"),
            Err(Error::InvalidMagic)
        );
        // ID3 marker but no stream behind it.
        assert_eq!(
            decode_mpc_stream_tagged(&id3_prefix(64)),
            Err(Error::InvalidMagic)
        );
    }

    #[test]
    fn trailing_tag_bytes_after_the_stream_are_ignored() {
        // §9.3: anything after the SV7 trailer / SV8 `SE` packet is
        // outside the stream — an APEv2-shaped tail must not disturb
        // the decode.
        let (_, raw) = sv7_file();
        let mut with_tail = raw.clone();
        with_tail.extend_from_slice(b"APETAGEX\xd0\x07\x00\x00trailing-tag-bytes");
        assert_eq!(
            decode_mpc_stream_tagged(&with_tail).unwrap(),
            decode_mpc_stream(&raw).unwrap()
        );

        let raw8 = sv8_stream();
        let mut with_tail8 = raw8.clone();
        // sv8_stream() has no SE packet, so close it first.
        with_tail8.extend_from_slice(b"SE\x03");
        with_tail8.extend_from_slice(b"APETAGEX\xd0\x07\x00\x00tail");
        let out = decode_mpc_stream_tagged(&with_tail8).unwrap();
        assert_eq!(out.kind(), StreamKind::Sv8);
    }
}
