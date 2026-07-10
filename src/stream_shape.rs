//! SV8 stream-shape observer.
//!
//! Walks a complete SV8 byte stream — including the leading `MPCK`
//! magic — and surfaces a structural summary: how many packets of
//! each §3.2 kind were seen, how many opaque payload bytes they
//! carried, and which kind appeared first / last. Pure observer:
//! the module does **not** validate packet ordering, does not
//! interpret any payload bytes, and does not return decoded data.
//!
//! Source-of-record (structural prose only):
//!
//! - `docs/audio/musepack/musepack-sv7-sv8-spec.md` §3.1 — `MPCK`
//!   magic + `[2-byte key][varint size][payload]` packet outer frame.
//! - `docs/audio/musepack/musepack-sv7-sv8-spec.md` §3.2 — the
//!   `{SH, RG, EI, SO, ST, AP, SE}` packet-key vocabulary.
//!
//! This module is the natural top-level entry point for a caller
//! that wants "tell me what's in this `.mpc` file" without descending
//! into payload field decoding (whose maps remain GAP per §3.2).
//! It composes:
//!
//! - [`crate::framing::parse_sv8_magic`] for the `MPCK` magic check;
//! - [`crate::packet_stream::PacketStream`] for the outer-frame walk;
//! - [`crate::typed_packet::TypedPacket::classify`] for the typed
//!   per-kind view.
//!
//! What this module **does not** do:
//!
//! - It does not enforce that `SH` is the first emitted packet nor
//!   that `SE` is the last. The structural spec lists roles for each
//!   key but does not pin a strict ordering grammar (see the
//!   [`crate::packet_stream`] module-level note). The shape simply
//!   records the kinds it observed.
//! - It does not decode any payload field. `SH` / `RG` / `EI` / `SO`
//!   / `ST` payload field maps remain GAP per §3.2; the `AP`
//!   entropy-coded body lives downstream of [`crate::packet_stream`].
//!   Each packet contributes only its opaque payload byte count to
//!   the shape's `total_payload_bytes` tally.
//! - It does not implement an SV7 (`MP+`) shape. SV7's stream layout
//!   is a fixed header plus non-byte-aligned frames (spec §2.1 /
//!   §2.2), not a packet sequence; the shape observer here is SV8
//!   only.

use crate::framing::{parse_sv8_magic, PacketKey};
use crate::packet_stream::{PacketSizeConvention, PacketStream};
use crate::typed_packet::TypedPacket;
use crate::Result;

/// Structural summary of one SV8 byte stream.
///
/// Built by [`scan_sv8_stream`] over a complete `MPCK`-prefixed
/// byte buffer. Every counter is incremented exactly once per
/// observed packet; the §3.2 vocabulary fields cover the known keys,
/// `unknown_count` aggregates every 2-byte key outside the
/// vocabulary, and `total_payload_bytes` is the cumulative opaque
/// payload byte tally across every emitted packet.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct StreamShape {
    /// Count of `SH` Stream-Header packets observed.
    pub sh_count: u32,
    /// Count of `RG` ReplayGain packets observed.
    pub rg_count: u32,
    /// Count of `EI` Encoder-Info packets observed.
    pub ei_count: u32,
    /// Count of `SO` Seek-table-Offset packets observed.
    pub so_count: u32,
    /// Count of `ST` Seek-Table packets observed.
    pub st_count: u32,
    /// Count of `AP` Audio packets observed.
    pub ap_count: u32,
    /// Count of `SE` Stream-End packets observed.
    pub se_count: u32,
    /// Count of packets whose 2-byte key was outside the §3.2
    /// vocabulary (preserved as [`PacketKey::Unknown`] by the
    /// upstream walker).
    pub unknown_count: u32,
    /// Cumulative opaque payload byte count across every emitted
    /// packet, computed at the [`PacketSizeConvention`] the scan was
    /// driven with. Header bytes (key + size varint) are NOT
    /// included.
    pub total_payload_bytes: u64,
    /// First emitted packet's classified key, or `None` if the
    /// stream had no packets (an `MPCK` magic followed by zero
    /// bytes — the walker's `Ok(None)` exit on an empty post-magic
    /// slice).
    pub first_kind: Option<PacketKey>,
    /// Last emitted packet's classified key, or `None` if the
    /// stream had no packets. After a normal stream this is
    /// [`PacketKey::StreamEnd`]; after an early-terminated stream
    /// (no `SE` packet emitted, walker exhausted on an empty
    /// buffer) it reflects whatever was last seen.
    pub last_kind: Option<PacketKey>,
}

impl StreamShape {
    /// Total number of packets observed across every §3.2 kind plus
    /// the unknown-key catch-all.
    pub fn total_packets(&self) -> u64 {
        u64::from(self.sh_count)
            + u64::from(self.rg_count)
            + u64::from(self.ei_count)
            + u64::from(self.so_count)
            + u64::from(self.st_count)
            + u64::from(self.ap_count)
            + u64::from(self.se_count)
            + u64::from(self.unknown_count)
    }

    /// True if every counter is zero and the stream had no packets.
    pub fn is_empty(&self) -> bool {
        self.total_packets() == 0
    }

    /// True if at least one `SE` Stream-End packet was observed.
    /// Useful for callers that want to confirm a clean walker
    /// termination without inspecting the last-kind field.
    pub fn saw_stream_end(&self) -> bool {
        self.se_count > 0
    }

    /// Per-kind count for any [`PacketKey`] — including the
    /// `Unknown` variant, whose raw key bytes are aggregated into
    /// `unknown_count` regardless of the specific 2-byte value.
    pub fn count_for(&self, key: PacketKey) -> u32 {
        match key {
            PacketKey::StreamHeader => self.sh_count,
            PacketKey::ReplayGain => self.rg_count,
            PacketKey::EncoderInfo => self.ei_count,
            PacketKey::SeekTableOffset => self.so_count,
            PacketKey::SeekTable => self.st_count,
            PacketKey::AudioPacket => self.ap_count,
            PacketKey::StreamEnd => self.se_count,
            PacketKey::Unknown(_) => self.unknown_count,
        }
    }

    /// Increment the per-kind counter that matches `key`.
    fn bump(&mut self, key: PacketKey) {
        match key {
            PacketKey::StreamHeader => self.sh_count += 1,
            PacketKey::ReplayGain => self.rg_count += 1,
            PacketKey::EncoderInfo => self.ei_count += 1,
            PacketKey::SeekTableOffset => self.so_count += 1,
            PacketKey::SeekTable => self.st_count += 1,
            PacketKey::AudioPacket => self.ap_count += 1,
            PacketKey::StreamEnd => self.se_count += 1,
            PacketKey::Unknown(_) => self.unknown_count += 1,
        }
    }
}

/// Walk a complete `MPCK`-prefixed SV8 byte stream and surface its
/// structural [`StreamShape`].
///
/// Steps:
///
/// 1. Validate the leading `MPCK` magic via
///    [`crate::framing::parse_sv8_magic`]. A mismatched magic
///    propagates [`crate::Error::InvalidMagic`]; a short input
///    propagates [`crate::Error::UnexpectedEof`].
/// 2. Build a [`PacketStream`] over the post-magic slice with the
///    caller-supplied [`PacketSizeConvention`] (the still-GAP
///    §3.1 varint convention; the caller picks one per the
///    [`crate::packet_stream`] module note).
/// 3. Walk one [`TypedPacket`] at a time, classifying each emitted
///    [`crate::packet_stream::PacketRef`] via
///    [`TypedPacket::classify`], and update the in-progress
///    [`StreamShape`]: bump the matching per-kind counter, add the
///    payload byte length to `total_payload_bytes`, record
///    `first_kind` on the first packet, and update `last_kind` on
///    every packet.
/// 4. Return the [`StreamShape`] once the walker reports `Ok(None)`
///    (post-`SE` terminator or empty post-magic slice).
///
/// Walker errors propagate unchanged from
/// [`PacketStream::next_packet`] — a truncated or malformed packet
/// surfaces the same error it would when walked directly, and the
/// already-accumulated counters are dropped (the function returns
/// before the partial shape is observable).
pub fn scan_sv8_stream(input: &[u8], convention: PacketSizeConvention) -> Result<StreamShape> {
    let magic_len = parse_sv8_magic(input)?;
    let post_magic = &input[magic_len..];
    let mut stream = PacketStream::new(post_magic, convention);
    let mut shape = StreamShape::default();
    while let Some(pkt) = stream.next_packet()? {
        let typed = TypedPacket::classify(pkt);
        let key = typed.key();
        shape.bump(key);
        // `payload_bytes()` returns the opaque slice the walker
        // surfaced at the chosen size convention; its length is the
        // payload byte count for this packet.
        let payload_len = typed.payload_bytes().len() as u64;
        shape.total_payload_bytes = shape.total_payload_bytes.saturating_add(payload_len);
        if shape.first_kind.is_none() {
            shape.first_kind = Some(key);
        }
        shape.last_kind = Some(key);
    }
    Ok(shape)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::framing::SV8_MAGIC;
    use crate::Error;

    /// Build a synthetic SV8 byte stream: `MPCK` magic + each packet
    /// as `[2-byte key][1-byte size varint][payload]`. The size is
    /// the literal varint value written; tests pass it under
    /// `PacketSizeConvention::Exclusive` to match the payload byte
    /// counts directly.
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

    #[test]
    fn rejects_input_missing_mpck_magic() {
        // `MP+`-style SV7 magic is not the SV8 outer frame.
        let buf = [b'M', b'P', b'+', 0x07];
        assert_eq!(
            scan_sv8_stream(&buf, PacketSizeConvention::Exclusive),
            Err(Error::InvalidMagic),
        );
        // Truncated below the four magic bytes -> EOF.
        let buf2 = *b"MPC";
        assert_eq!(
            scan_sv8_stream(&buf2, PacketSizeConvention::Exclusive),
            Err(Error::UnexpectedEof),
        );
    }

    #[test]
    fn empty_post_magic_yields_empty_shape() {
        // Just the `MPCK` magic, no packets following.
        let shape =
            scan_sv8_stream(&SV8_MAGIC, PacketSizeConvention::Exclusive).expect("empty scan");
        assert!(shape.is_empty(), "no packets => empty shape");
        assert_eq!(shape.total_packets(), 0);
        assert_eq!(shape.total_payload_bytes, 0);
        assert_eq!(shape.first_kind, None);
        assert_eq!(shape.last_kind, None);
        assert!(!shape.saw_stream_end());
    }

    #[test]
    fn single_se_terminator_records_first_and_last_as_stream_end() {
        let buf = build_stream(&[(b"SE", 0, &[])]);
        let shape = scan_sv8_stream(&buf, PacketSizeConvention::Exclusive).expect("SE-only scan");
        assert_eq!(shape.se_count, 1);
        assert_eq!(shape.total_packets(), 1);
        assert_eq!(shape.total_payload_bytes, 0);
        assert_eq!(shape.first_kind, Some(PacketKey::StreamEnd));
        assert_eq!(shape.last_kind, Some(PacketKey::StreamEnd));
        assert!(shape.saw_stream_end());
        assert!(!shape.is_empty());
    }

    #[test]
    fn walks_full_vocabulary_in_order_with_correct_first_and_last() {
        // One of every §3.2 kind, in the order the structural spec
        // table lists them (SH, RG, EI, SO, ST, AP, SE).
        let buf = build_stream(&[
            (b"SH", 3, &[0x10, 0x20, 0x30]),
            (b"RG", 2, &[0xAA, 0xBB]),
            (b"EI", 1, &[0x55]),
            (b"SO", 1, &[0x77]),
            (b"ST", 4, &[1, 2, 3, 4]),
            (b"AP", 5, &[9, 8, 7, 6, 5]),
            (b"SE", 0, &[]),
        ]);
        let shape =
            scan_sv8_stream(&buf, PacketSizeConvention::Exclusive).expect("full-vocabulary scan");
        assert_eq!(shape.sh_count, 1);
        assert_eq!(shape.rg_count, 1);
        assert_eq!(shape.ei_count, 1);
        assert_eq!(shape.so_count, 1);
        assert_eq!(shape.st_count, 1);
        assert_eq!(shape.ap_count, 1);
        assert_eq!(shape.se_count, 1);
        assert_eq!(shape.unknown_count, 0);
        assert_eq!(shape.total_packets(), 7);
        // Sum of payload sizes 3 + 2 + 1 + 1 + 4 + 5 + 0 = 16.
        assert_eq!(shape.total_payload_bytes, 16);
        assert_eq!(shape.first_kind, Some(PacketKey::StreamHeader));
        assert_eq!(shape.last_kind, Some(PacketKey::StreamEnd));
        assert!(shape.saw_stream_end());
    }

    #[test]
    fn aggregates_repeat_ap_packets_into_a_single_counter() {
        // Many AP packets, bracketed by SH first and SE last —
        // exercises the per-kind counter increment loop.
        let buf = build_stream(&[
            (b"SH", 2, &[0xAA, 0xBB]),
            (b"AP", 4, &[1, 2, 3, 4]),
            (b"AP", 4, &[5, 6, 7, 8]),
            (b"AP", 4, &[9, 10, 11, 12]),
            (b"AP", 4, &[13, 14, 15, 16]),
            (b"SE", 0, &[]),
        ]);
        let shape = scan_sv8_stream(&buf, PacketSizeConvention::Exclusive).expect("ap-burst scan");
        assert_eq!(shape.ap_count, 4);
        assert_eq!(shape.sh_count, 1);
        assert_eq!(shape.se_count, 1);
        assert_eq!(shape.total_packets(), 6);
        assert_eq!(shape.total_payload_bytes, 2 + 4 * 4);
        assert_eq!(shape.first_kind, Some(PacketKey::StreamHeader));
        assert_eq!(shape.last_kind, Some(PacketKey::StreamEnd));
    }

    #[test]
    fn aggregates_unknown_keys_into_a_single_counter_preserving_first_last() {
        // Two unrelated unknown keys plus an SE terminator. Both
        // unknowns must be counted, but `first_kind` and `last_kind`
        // surface the raw bytes of the FIRST and LAST packet seen,
        // not the §3.2 names.
        let buf = build_stream(&[
            (b"XY", 2, &[0xC0, 0xDE]),
            (b"ZZ", 1, &[0x42]),
            (b"SE", 0, &[]),
        ]);
        let shape =
            scan_sv8_stream(&buf, PacketSizeConvention::Exclusive).expect("unknown-keys scan");
        assert_eq!(shape.unknown_count, 2);
        assert_eq!(shape.se_count, 1);
        assert_eq!(shape.total_packets(), 3);
        // Payload bytes: 2 + 1 + 0 (SE).
        assert_eq!(shape.total_payload_bytes, 3);
        assert_eq!(shape.first_kind, Some(PacketKey::Unknown(*b"XY")));
        assert_eq!(shape.last_kind, Some(PacketKey::StreamEnd));
    }

    #[test]
    fn count_for_routes_every_known_kind_to_the_matching_counter() {
        // Verify the `count_for` accessor against a hand-built shape.
        let shape = StreamShape {
            sh_count: 11,
            rg_count: 22,
            ei_count: 33,
            so_count: 44,
            st_count: 55,
            ap_count: 66,
            se_count: 77,
            unknown_count: 88,
            total_payload_bytes: 0,
            first_kind: None,
            last_kind: None,
        };
        assert_eq!(shape.count_for(PacketKey::StreamHeader), 11);
        assert_eq!(shape.count_for(PacketKey::ReplayGain), 22);
        assert_eq!(shape.count_for(PacketKey::EncoderInfo), 33);
        assert_eq!(shape.count_for(PacketKey::SeekTableOffset), 44);
        assert_eq!(shape.count_for(PacketKey::SeekTable), 55);
        assert_eq!(shape.count_for(PacketKey::AudioPacket), 66);
        assert_eq!(shape.count_for(PacketKey::StreamEnd), 77);
        // The Unknown variant — regardless of its raw bytes — maps
        // to the single unknown_count aggregator.
        assert_eq!(shape.count_for(PacketKey::Unknown(*b"XY")), 88);
        assert_eq!(shape.count_for(PacketKey::Unknown(*b"ZZ")), 88);
    }

    #[test]
    fn truncated_payload_propagates_unexpected_eof() {
        // SH declares a 5-byte payload but only 2 bytes follow before
        // the buffer ends. The walker reports UnexpectedEof; the scan
        // surfaces the same error without returning a partial shape.
        let mut buf: Vec<u8> = Vec::new();
        buf.extend_from_slice(&SV8_MAGIC);
        buf.extend_from_slice(b"SH");
        buf.push(0x05);
        buf.extend_from_slice(&[0x10, 0x20]);
        assert_eq!(
            scan_sv8_stream(&buf, PacketSizeConvention::Exclusive),
            Err(Error::UnexpectedEof),
        );
    }

    #[test]
    fn ignores_trailing_bytes_after_se_terminator() {
        // After SE the walker stops and reports Ok(None); any
        // trailing garbage in the buffer is left untouched and does
        // NOT contribute to the shape.
        let mut buf = build_stream(&[
            (b"SH", 1, &[0xAA]),
            (b"AP", 2, &[0xBB, 0xCC]),
            (b"SE", 0, &[]),
        ]);
        buf.extend_from_slice(&[0xFF; 8]); // garbage past SE.
        let shape =
            scan_sv8_stream(&buf, PacketSizeConvention::Exclusive).expect("scan stops at SE");
        assert_eq!(shape.total_packets(), 3);
        assert_eq!(shape.sh_count, 1);
        assert_eq!(shape.ap_count, 1);
        assert_eq!(shape.se_count, 1);
        // Payload byte tally is the legitimate SH + AP payload only.
        assert_eq!(shape.total_payload_bytes, 1 + 2);
        assert_eq!(shape.last_kind, Some(PacketKey::StreamEnd));
    }

    #[test]
    fn shape_with_no_se_terminator_still_reports_last_kind_observed() {
        // Build an SH + AP run that ends *without* an SE packet.
        // The walker's empty-input exit terminates the scan; the
        // shape records what was actually seen.
        let buf = build_stream(&[(b"SH", 3, &[0x10, 0x20, 0x30]), (b"AP", 2, &[0xAB, 0xCD])]);
        let shape = scan_sv8_stream(&buf, PacketSizeConvention::Exclusive).expect("se-less scan");
        assert_eq!(shape.sh_count, 1);
        assert_eq!(shape.ap_count, 1);
        assert_eq!(shape.se_count, 0);
        assert!(!shape.saw_stream_end());
        assert_eq!(shape.first_kind, Some(PacketKey::StreamHeader));
        assert_eq!(shape.last_kind, Some(PacketKey::AudioPacket));
    }

    #[test]
    fn total_payload_bytes_is_payload_only_not_header_bytes() {
        // Each packet here has a 3-byte header (2 key + 1 size
        // varint). The tally MUST be payload only — not header.
        let buf = build_stream(&[
            (b"SH", 4, &[1, 2, 3, 4]),
            (b"AP", 8, &[10, 11, 12, 13, 14, 15, 16, 17]),
            (b"SE", 0, &[]),
        ]);
        let shape =
            scan_sv8_stream(&buf, PacketSizeConvention::Exclusive).expect("payload tally scan");
        // 4 + 8 + 0 = 12. Header bytes excluded.
        assert_eq!(shape.total_payload_bytes, 12);
        assert!(shape.total_payload_bytes < buf.len() as u64);
    }

    #[test]
    fn scan_under_inclusive_convention_succeeds_when_sizes_match() {
        // Inclusive convention: each `raw_size` counts the 3-byte
        // header itself + payload. SH(payload=2) -> raw_size=5;
        // AP(payload=4) -> raw_size=7; SE(payload=0) -> raw_size=3.
        let buf = build_stream(&[
            (b"SH", 5, &[0x10, 0x20]),
            (b"AP", 7, &[1, 2, 3, 4]),
            (b"SE", 3, &[]),
        ]);
        let shape = scan_sv8_stream(&buf, PacketSizeConvention::Inclusive).expect("inclusive scan");
        assert_eq!(shape.total_packets(), 3);
        // Under the inclusive convention the payload byte counts are
        // (raw_size - header_len): 2, 4, 0 — sum 6.
        assert_eq!(shape.total_payload_bytes, 6);
        assert_eq!(shape.first_kind, Some(PacketKey::StreamHeader));
        assert_eq!(shape.last_kind, Some(PacketKey::StreamEnd));
    }

    #[test]
    fn default_shape_is_all_zero_and_empty() {
        let shape = StreamShape::default();
        assert_eq!(shape.sh_count, 0);
        assert_eq!(shape.rg_count, 0);
        assert_eq!(shape.ei_count, 0);
        assert_eq!(shape.so_count, 0);
        assert_eq!(shape.st_count, 0);
        assert_eq!(shape.ap_count, 0);
        assert_eq!(shape.se_count, 0);
        assert_eq!(shape.unknown_count, 0);
        assert_eq!(shape.total_payload_bytes, 0);
        assert_eq!(shape.first_kind, None);
        assert_eq!(shape.last_kind, None);
        assert!(shape.is_empty());
        assert!(!shape.saw_stream_end());
    }

    #[test]
    fn shape_is_copy_and_eq() {
        // `StreamShape` is a pure-data summary; callers should be
        // able to keep a copy around without lifetime gymnastics.
        let a = StreamShape {
            sh_count: 1,
            ap_count: 4,
            se_count: 1,
            total_payload_bytes: 42,
            first_kind: Some(PacketKey::StreamHeader),
            last_kind: Some(PacketKey::StreamEnd),
            ..Default::default()
        };
        let b = a;
        assert_eq!(a, b);
        assert_eq!(a.total_packets(), 6);
    }

    #[test]
    fn first_kind_locked_in_on_the_first_packet_and_does_not_update() {
        // Confirm the first-packet-wins semantics of `first_kind`:
        // a later RG packet must not overwrite the SH first-seen.
        let buf = build_stream(&[
            (b"SH", 1, &[0xAA]),
            (b"RG", 2, &[0xBB, 0xCC]),
            (b"AP", 1, &[0xDD]),
            (b"SE", 0, &[]),
        ]);
        let shape =
            scan_sv8_stream(&buf, PacketSizeConvention::Exclusive).expect("first-kind scan");
        assert_eq!(shape.first_kind, Some(PacketKey::StreamHeader));
        // Sanity: every packet was counted.
        assert_eq!(shape.total_packets(), 4);
    }
}
