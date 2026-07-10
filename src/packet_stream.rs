//! SV8 packet-stream walker.
//!
//! Wraps the per-packet [`crate::framing::parse_packet_header`] +
//! offset-advance pattern (already proven inside `framing::tests`)
//! into a public iterator-like API that walks an SV8 byte stream
//! one packet at a time and stops at the `SE` terminator.
//!
//! Source-of-record (structural prose only):
//!
//! - `docs/audio/musepack/musepack-sv7-sv8-spec.md` §3.1 — packet
//!   outer-frame layout: `[2-byte key][varint size][payload]`.
//! - `docs/audio/musepack/musepack-sv7-sv8-spec.md` §3.2 — packet-key
//!   vocabulary, with `SE` documented as the stream-end terminator.
//!
//! Per spec §3.1 the varint **convention** (inclusive vs exclusive of
//! the key + size header bytes) is GAP. The walker takes that pick as
//! an explicit [`PacketSizeConvention`] knob on construction — the
//! caller chooses once when standing the walker up, and every packet
//! advances by the chosen interpretation. The pending observer-trace
//! round (workspace task #1263) is expected to fix one convention as
//! the only correct reading; until then both options are exposed.
//!
//! What this module does **not** do:
//!
//! - It does not parse the `MPCK` magic. The caller runs
//!   [`crate::framing::parse_sv8_magic`] first and hands the
//!   post-magic slice to [`PacketStream::new`]. Keeping the two
//!   separate matches the spec's prose split (§3.1 first paragraph
//!   = magic; §3.1 second paragraph onward = packet sequence).
//! - It does not interpret any packet payload. Each iteration yields
//!   a [`PacketRef`] whose `.payload` is an opaque slice; payload
//!   field maps for `SH` / `RG` / `EI` / `SO` / `ST` / `AP` are GAP
//!   per §3.2 and live downstream of this module.
//! - It does not validate that `SE` is the **last** packet emitted
//!   nor that `SH` is the **first**. The structural prose lists
//!   roles for each key but does not pin a strict ordering grammar
//!   (the README §3.3 keyframe note flags ordering as GAP). The
//!   walker reports each packet it sees and treats `SE` as a stop
//!   signal regardless of where it appears.

use crate::framing::{parse_packet_header, PacketHeader, PacketKey};
use crate::{Error, Result};

/// Which interpretation of the SV8 packet varint to use when
/// advancing the stream cursor.
///
/// Per spec §3.1 the varint convention is GAP. The `Inclusive`
/// variant reads the literal §3.1 sentence "total packet length
/// (key + size field + payload)" — the varint counts the header
/// bytes the walker just read. The `Exclusive` variant treats the
/// varint as the payload byte count alone, i.e. the bytes
/// immediately following the size varint.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PacketSizeConvention {
    /// `raw_size` counts the 2-byte key + the size varint + payload.
    /// To find the payload, the walker subtracts the header length
    /// from `raw_size`; a `raw_size` smaller than the header length
    /// is a malformed packet (rejected with
    /// [`Error::VarintTooLong`]).
    Inclusive,
    /// `raw_size` counts only the payload bytes following the size
    /// varint. The walker advances by `header_len + raw_size`.
    Exclusive,
}

/// One decoded SV8 packet, surfaced by [`PacketStream`] iteration.
///
/// `payload` is a borrow into the underlying stream slice and
/// remains valid for the lifetime of that slice. Its bytes are
/// **not** interpreted by this module — `SH` / `RG` / `EI` / `SO` /
/// `ST` payload field maps are GAP per spec §3.2, and `AP` payload
/// entropy decode lives downstream of [`crate::framing`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PacketRef<'a> {
    /// Classified 2-byte packet key (§3.2).
    pub key: PacketKey,
    /// Raw `[2-byte key][varint size]` header.
    pub header: PacketHeader,
    /// Opaque payload bytes between the size varint and the next
    /// packet's start, sized per the [`PacketSizeConvention`] the
    /// stream was created with.
    pub payload: &'a [u8],
}

/// Walker over an SV8 byte stream, post-`MPCK` magic.
///
/// Built from the slice that starts at the first packet (immediately
/// after the `MPCK` magic; see [`crate::framing::parse_sv8_magic`])
/// and a [`PacketSizeConvention`] pick for the still-GAP varint
/// interpretation. Iterates one packet at a time via
/// [`PacketStream::next_packet`], returning `Ok(Some(_))` for each
/// decoded packet, `Ok(None)` after the `SE` terminator has been
/// observed, and `Err(_)` on a malformed or truncated input.
///
/// The walker stores the *remaining* slice (the bytes from the next
/// unread packet onward); each successful read advances that cursor
/// past the packet's full extent.
#[derive(Debug, Clone)]
pub struct PacketStream<'a> {
    remaining: &'a [u8],
    convention: PacketSizeConvention,
    stopped: bool,
}

impl<'a> PacketStream<'a> {
    /// Build a walker over `bytes` with the supplied size
    /// interpretation. `bytes` must start at the first packet's
    /// 2-byte key — i.e. the slice **after** the `MPCK` magic.
    pub fn new(bytes: &'a [u8], convention: PacketSizeConvention) -> Self {
        Self {
            remaining: bytes,
            convention,
            stopped: false,
        }
    }

    /// Bytes still unread in the underlying slice. Decreases by the
    /// packet's full extent on each successful [`Self::next_packet`].
    pub fn remaining_bytes(&self) -> &'a [u8] {
        self.remaining
    }

    /// True once the stream has reached its terminator (after a
    /// successful `SE` read) or after a hard error short-circuited
    /// further reads.
    pub fn is_stopped(&self) -> bool {
        self.stopped
    }

    /// The size-interpretation pick used to advance the cursor.
    pub fn convention(&self) -> PacketSizeConvention {
        self.convention
    }

    /// Decode one packet from the head of the remaining slice and
    /// advance the cursor past it.
    ///
    /// Returns:
    ///
    /// - `Ok(Some(PacketRef))` for a successfully-parsed packet.
    ///   `payload` is the opaque slice between the size varint and
    ///   the next packet's start.
    /// - `Ok(None)` when the walker is in the stopped state (the
    ///   most recent `SE` was already returned, or the underlying
    ///   slice was empty on entry — the spec §3.2 `SE` terminator
    ///   convention).
    /// - `Err(Error::UnexpectedEof)` if the underlying slice ran
    ///   out partway through the header or payload.
    /// - `Err(Error::VarintTooLong)` if the inclusive-convention
    ///   `raw_size` is smaller than the header length (a malformed
    ///   packet that claims a sub-header size in inclusive mode).
    /// - `Err(_)` from [`parse_packet_header`] propagation
    ///   (`VarintTooLong` on an overlong varint).
    ///
    /// After an `Err(_)` the walker is left in the stopped state;
    /// subsequent calls return `Ok(None)`.
    pub fn next_packet(&mut self) -> Result<Option<PacketRef<'a>>> {
        if self.stopped {
            return Ok(None);
        }
        if self.remaining.is_empty() {
            self.stopped = true;
            return Ok(None);
        }
        let header = match parse_packet_header(self.remaining) {
            Ok(h) => h,
            Err(e) => {
                self.stopped = true;
                return Err(e);
            }
        };
        let payload_len = match self.convention {
            PacketSizeConvention::Inclusive => match header.payload_len_inclusive() {
                Some(len) => len,
                None => {
                    self.stopped = true;
                    return Err(Error::VarintTooLong);
                }
            },
            PacketSizeConvention::Exclusive => header.payload_len_exclusive(),
        };
        let payload_len_usize: usize = match usize::try_from(payload_len) {
            Ok(v) => v,
            Err(_) => {
                self.stopped = true;
                return Err(Error::VarintTooLong);
            }
        };
        let total = match header.header_len.checked_add(payload_len_usize) {
            Some(v) => v,
            None => {
                self.stopped = true;
                return Err(Error::VarintTooLong);
            }
        };
        if total > self.remaining.len() {
            self.stopped = true;
            return Err(Error::UnexpectedEof);
        }
        let payload = &self.remaining[header.header_len..total];
        let pkt = PacketRef {
            key: header.key,
            header,
            payload,
        };
        self.remaining = &self.remaining[total..];
        if pkt.key == PacketKey::StreamEnd {
            self.stopped = true;
        }
        Ok(Some(pkt))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::framing::SV8_MAGIC;

    /// Build a synthetic SV8 byte stream: `MPCK` + each packet as
    /// `[key][1-byte size varint][payload]`. `size` is the literal
    /// varint value written — the test caller decides which
    /// convention they're targeting (and matches it on
    /// construction).
    fn build_stream(packets: &[(&[u8; 2], u8, &[u8])]) -> Vec<u8> {
        let mut buf = Vec::new();
        buf.extend_from_slice(&SV8_MAGIC);
        for (key, size, payload) in packets {
            buf.extend_from_slice(&**key);
            buf.push(*size);
            buf.extend_from_slice(payload);
        }
        buf
    }

    /// Strip the `MPCK` magic prefix and return the post-magic slice
    /// to hand to [`PacketStream::new`].
    fn after_magic(buf: &[u8]) -> &[u8] {
        &buf[SV8_MAGIC.len()..]
    }

    #[test]
    fn empty_input_yields_none_and_stops() {
        let mut s = PacketStream::new(&[], PacketSizeConvention::Exclusive);
        assert!(!s.is_stopped());
        assert_eq!(s.next_packet().unwrap(), None);
        assert!(s.is_stopped());
        // Second call still returns None, doesn't error.
        assert_eq!(s.next_packet().unwrap(), None);
    }

    #[test]
    fn convention_accessor_round_trips() {
        let s = PacketStream::new(&[], PacketSizeConvention::Inclusive);
        assert_eq!(s.convention(), PacketSizeConvention::Inclusive);
        let s = PacketStream::new(&[], PacketSizeConvention::Exclusive);
        assert_eq!(s.convention(), PacketSizeConvention::Exclusive);
    }

    #[test]
    fn exclusive_single_se_terminator() {
        // SE with size=0 under the exclusive convention: no payload.
        let buf = build_stream(&[(b"SE", 0, &[])]);
        let mut s = PacketStream::new(after_magic(&buf), PacketSizeConvention::Exclusive);
        let pkt = s.next_packet().unwrap().expect("packet");
        assert_eq!(pkt.key, PacketKey::StreamEnd);
        assert_eq!(pkt.header.header_len, 3);
        assert!(pkt.payload.is_empty());
        assert!(s.is_stopped());
        // Past the terminator -> None, stays stopped.
        assert_eq!(s.next_packet().unwrap(), None);
        assert!(s.is_stopped());
    }

    #[test]
    fn exclusive_three_packets_walked_in_order() {
        // SH (4 opaque payload bytes) + AP (8 opaque payload bytes) +
        // SE (empty).
        let buf = build_stream(&[
            (b"SH", 4, &[0xDE, 0xAD, 0xBE, 0xEF]),
            (b"AP", 8, &[0; 8]),
            (b"SE", 0, &[]),
        ]);
        let mut s = PacketStream::new(after_magic(&buf), PacketSizeConvention::Exclusive);
        let mut seen = Vec::new();
        while let Some(pkt) = s.next_packet().unwrap() {
            seen.push((pkt.key, pkt.payload.to_vec()));
        }
        assert_eq!(
            seen,
            vec![
                (PacketKey::StreamHeader, vec![0xDE, 0xAD, 0xBE, 0xEF]),
                (PacketKey::AudioPacket, vec![0; 8]),
                (PacketKey::StreamEnd, vec![]),
            ],
        );
        assert!(s.is_stopped());
    }

    #[test]
    fn exclusive_walk_stops_at_se_even_with_trailing_bytes() {
        // Spec §3.2 says SE terminates the stream. Bytes after an SE
        // are out of scope for the walker — it stops without
        // erroring.
        let mut buf = build_stream(&[(b"AP", 2, &[0xAA, 0xBB]), (b"SE", 0, &[])]);
        // Pretend the file has stray trailing bytes after SE.
        buf.extend_from_slice(b"junk-after-se");
        let post_magic_len = buf.len() - SV8_MAGIC.len();
        let mut s = PacketStream::new(after_magic(&buf), PacketSizeConvention::Exclusive);
        // Walk two packets.
        let p1 = s.next_packet().unwrap().expect("AP");
        assert_eq!(p1.key, PacketKey::AudioPacket);
        let p2 = s.next_packet().unwrap().expect("SE");
        assert_eq!(p2.key, PacketKey::StreamEnd);
        assert!(s.is_stopped());
        // After SE: no more packets, even though bytes remain.
        assert_eq!(s.next_packet().unwrap(), None);
        // Sanity: the slice we built was indeed longer than what we
        // walked, i.e. the trailing junk was structurally present.
        assert!(s.remaining_bytes().len() < post_magic_len);
    }

    #[test]
    fn inclusive_single_packet_with_inclusive_size() {
        // Inclusive convention: raw_size counts key+size_varint+payload.
        // SH with 4-byte payload: header_len = 3, payload_len = 4 ->
        // raw_size = 7 inclusive.
        let buf = build_stream(&[(b"SH", 7, &[0x01, 0x02, 0x03, 0x04])]);
        let mut s = PacketStream::new(after_magic(&buf), PacketSizeConvention::Inclusive);
        let pkt = s.next_packet().unwrap().expect("SH");
        assert_eq!(pkt.key, PacketKey::StreamHeader);
        assert_eq!(pkt.payload, &[0x01, 0x02, 0x03, 0x04]);
        // Stream isn't terminated (no SE) but is fully consumed.
        // The next call sees an empty slice and returns None.
        assert_eq!(s.next_packet().unwrap(), None);
        assert!(s.is_stopped());
    }

    #[test]
    fn inclusive_size_smaller_than_header_rejects() {
        // raw_size = 2 in inclusive mode but header_len = 3 -> error.
        let buf = build_stream(&[(b"SH", 2, &[])]);
        let mut s = PacketStream::new(after_magic(&buf), PacketSizeConvention::Inclusive);
        let err = s.next_packet().unwrap_err();
        assert_eq!(err, Error::VarintTooLong);
        assert!(s.is_stopped());
        // Subsequent calls report None, not the error again.
        assert_eq!(s.next_packet().unwrap(), None);
    }

    #[test]
    fn truncated_payload_propagates_eof() {
        // Declare 8 payload bytes but only ship 3.
        let mut buf = Vec::new();
        buf.extend_from_slice(&SV8_MAGIC);
        buf.extend_from_slice(b"AP");
        buf.push(0x08); // raw_size = 8 (exclusive convention)
        buf.extend_from_slice(&[0xAA, 0xBB, 0xCC]); // only 3, not 8.
        let mut s = PacketStream::new(after_magic(&buf), PacketSizeConvention::Exclusive);
        let err = s.next_packet().unwrap_err();
        assert_eq!(err, Error::UnexpectedEof);
        assert!(s.is_stopped());
        assert_eq!(s.next_packet().unwrap(), None);
    }

    #[test]
    fn malformed_header_propagates_and_stops() {
        // 1-byte input: too short to be a key.
        let buf = *b"S";
        let mut s = PacketStream::new(&buf, PacketSizeConvention::Exclusive);
        let err = s.next_packet().unwrap_err();
        assert_eq!(err, Error::UnexpectedEof);
        assert!(s.is_stopped());
    }

    #[test]
    fn remaining_bytes_shrinks_after_each_read() {
        let buf = build_stream(&[
            (b"AP", 2, &[0xAA, 0xBB]),
            (b"AP", 1, &[0xCC]),
            (b"SE", 0, &[]),
        ]);
        let post = after_magic(&buf);
        let mut s = PacketStream::new(post, PacketSizeConvention::Exclusive);
        let len_initial = s.remaining_bytes().len();
        assert_eq!(len_initial, post.len());
        let _ = s.next_packet().unwrap().expect("AP 1");
        let after_one = s.remaining_bytes().len();
        assert!(after_one < len_initial);
        let _ = s.next_packet().unwrap().expect("AP 2");
        let after_two = s.remaining_bytes().len();
        assert!(after_two < after_one);
        let _ = s.next_packet().unwrap().expect("SE");
        // SE doesn't necessarily drain remaining_bytes (it stops the
        // walker), but it must not leave more bytes than after_two.
        assert!(s.remaining_bytes().len() <= after_two);
        assert!(s.is_stopped());
    }

    #[test]
    fn unknown_key_is_passed_through_not_rejected() {
        // The walker is structure-only: an unknown key still has a
        // varint size and payload, so the walker reports it as
        // PacketKey::Unknown without erroring (the observer-trace
        // round may surface new keys).
        let buf = build_stream(&[(b"XY", 3, &[1, 2, 3]), (b"SE", 0, &[])]);
        let mut s = PacketStream::new(after_magic(&buf), PacketSizeConvention::Exclusive);
        let p1 = s.next_packet().unwrap().expect("XY");
        assert_eq!(p1.key, PacketKey::Unknown(*b"XY"));
        assert_eq!(p1.payload, &[1, 2, 3]);
        let p2 = s.next_packet().unwrap().expect("SE");
        assert_eq!(p2.key, PacketKey::StreamEnd);
        assert!(s.is_stopped());
    }

    #[test]
    fn collect_full_walk_returns_expected_count() {
        // Five packets ending in SE — verify we get exactly five.
        let buf = build_stream(&[
            (b"SH", 1, &[0x80]),
            (b"RG", 1, &[0x00]),
            (b"EI", 1, &[0x00]),
            (b"AP", 2, &[0xAA, 0xBB]),
            (b"SE", 0, &[]),
        ]);
        let mut s = PacketStream::new(after_magic(&buf), PacketSizeConvention::Exclusive);
        let mut count = 0;
        while s.next_packet().unwrap().is_some() {
            count += 1;
        }
        assert_eq!(count, 5);
    }

    #[test]
    fn error_path_leaves_stopped_so_no_repeated_errors() {
        // Same as truncated_payload_propagates_eof but check the
        // post-error state across multiple calls.
        let mut buf = Vec::new();
        buf.extend_from_slice(b"AP");
        buf.push(0x08);
        buf.extend_from_slice(&[0xAA]); // 1 byte instead of 8.
        let mut s = PacketStream::new(&buf, PacketSizeConvention::Exclusive);
        assert_eq!(s.next_packet().unwrap_err(), Error::UnexpectedEof);
        assert!(s.is_stopped());
        // Calls 2 and 3 are quiet.
        for _ in 0..2 {
            assert_eq!(s.next_packet().unwrap(), None);
        }
    }

    #[test]
    fn packet_ref_payload_borrows_from_input() {
        // Lifetime check: the payload slice in PacketRef must be
        // tied to the underlying input, not copied. We verify by
        // computing the byte offset of the payload's first byte
        // within the post-magic slice.
        let buf = build_stream(&[(b"AP", 3, &[0xCA, 0xFE, 0xBA]), (b"SE", 0, &[])]);
        let post = after_magic(&buf);
        let mut s = PacketStream::new(post, PacketSizeConvention::Exclusive);
        let pkt = s.next_packet().unwrap().expect("AP");
        // Byte offset of the payload start within `post`. The AP
        // packet's header is 3 bytes (2 key + 1 varint size), so the
        // payload's first byte sits at offset 3.
        let payload_offset = post.len() - s.remaining_bytes().len() - pkt.payload.len();
        assert_eq!(payload_offset, 3);
        // Sanity: the payload bytes match the slice we extracted at
        // offset 3..6 of `post`.
        assert_eq!(pkt.payload, &post[3..6]);
    }
}
