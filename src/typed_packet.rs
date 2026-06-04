//! Typed SV8 packet surface.
//!
//! Wraps a [`crate::packet_stream::PacketRef`] in a typed sum that
//! discriminates the spec §3.2 packet vocabulary at the API surface,
//! routing each known 2-byte key to a per-kind borrowed newtype that
//! carries the opaque payload slice. The payload **bytes** are still
//! not interpreted — field maps for `SH` / `RG` / `EI` / `SO` / `ST`
//! remain GAP per spec §3.2 and live downstream of this module — but
//! a caller that wants to write `if let TypedPacket::StreamHeader(sh)
//! = ...` instead of matching on a `PacketKey` enum + re-validating
//! the raw byte slice gets one.
//!
//! Source-of-record (structural prose only):
//!
//! - `docs/audio/musepack/musepack-sv7-sv8-spec.md` §3.1 — packet
//!   outer-frame layout (key + varint size + payload), supplied by
//!   the upstream [`crate::packet_stream::PacketStream`].
//! - `docs/audio/musepack/musepack-sv7-sv8-spec.md` §3.2 — packet-key
//!   vocabulary: `SH` Stream Header, `RG` ReplayGain, `EI` Encoder
//!   Info, `SO` Seek-table Offset, `ST` Seek Table, `AP` Audio
//!   Packet, `SE` Stream End.
//!
//! What this module does **not** do:
//!
//! - It does not parse any payload field. Each typed wrapper holds
//!   the same opaque `&[u8]` slice the upstream `PacketRef` exposed;
//!   the SH CRC / sample-rate index / `max_band` etc. cited at the
//!   §3.2 prose level are GAP per the table's "Field layout" column
//!   and live downstream of this module.
//! - It does not validate ordering (SH-first / SE-last). The
//!   structural spec lists per-key roles but does not pin a strict
//!   grammar — see the `PacketStream` module-level note. The typed
//!   surface simply mirrors what the walker emits.
//! - It does not introduce a parser for unknown keys. A 2-byte key
//!   outside the §3.2 vocabulary is preserved as
//!   [`TypedPacket::Unknown`] carrying the raw bytes so a caller
//!   can log them.

use crate::framing::PacketKey;
use crate::packet_stream::PacketRef;

/// Stream-header (`SH`) packet — first payload packet per spec §3.2.
///
/// Holds the opaque payload slice. The per-field map (CRC, stream
/// version 8, sample-rate index, max used bands, channel count,
/// mid-side flag, total sample count + beginning/gapless silence)
/// is GAP per spec §3.2 and is not decoded here.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StreamHeaderPacket<'a> {
    payload: &'a [u8],
}

impl<'a> StreamHeaderPacket<'a> {
    /// Opaque payload bytes between the size varint and the next
    /// packet's start.
    pub fn payload_bytes(&self) -> &'a [u8] {
        self.payload
    }
}

/// ReplayGain (`RG`) packet — bitstream-level loudness metadata per
/// spec §3.2. Field sizes (version + title gain/peak + album
/// gain/peak) are GAP.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ReplayGainPacket<'a> {
    payload: &'a [u8],
}

impl<'a> ReplayGainPacket<'a> {
    /// Opaque payload bytes.
    pub fn payload_bytes(&self) -> &'a [u8] {
        self.payload
    }
}

/// Encoder-info (`EI`) packet — encoder identification per spec
/// §3.2 (profile / quality, PNS flag, encoder version). Field map
/// is GAP.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EncoderInfoPacket<'a> {
    payload: &'a [u8],
}

impl<'a> EncoderInfoPacket<'a> {
    /// Opaque payload bytes.
    pub fn payload_bytes(&self) -> &'a [u8] {
        self.payload
    }
}

/// Seek-table-offset (`SO`) packet — a single offset pointing at the
/// `ST` seek table per spec §3.2. Field map is GAP.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SeekTableOffsetPacket<'a> {
    payload: &'a [u8],
}

impl<'a> SeekTableOffsetPacket<'a> {
    /// Opaque payload bytes.
    pub fn payload_bytes(&self) -> &'a [u8] {
        self.payload
    }
}

/// Seek-table (`ST`) packet — entry count + delta-coded offsets per
/// spec §3.2. Field map is GAP.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SeekTablePacket<'a> {
    payload: &'a [u8],
}

impl<'a> SeekTablePacket<'a> {
    /// Opaque payload bytes.
    pub fn payload_bytes(&self) -> &'a [u8] {
        self.payload
    }
}

/// Audio (`AP`) packet — one or more entropy-coded audio frames per
/// spec §3.2 / §3.4. The inner SV8 entropy layer (canonical Huffman
/// tables `sv8-canonical-*` + `sv8-symbols-*`) is GAP at this layer
/// and decoded downstream.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AudioPacket<'a> {
    payload: &'a [u8],
}

impl<'a> AudioPacket<'a> {
    /// Opaque payload bytes (the SV8 entropy-coded frame body).
    pub fn payload_bytes(&self) -> &'a [u8] {
        self.payload
    }
}

/// Stream-end (`SE`) packet — terminator per spec §3.2. Carries an
/// opaque payload slice (typically empty, though the structural
/// spec does not forbid `SE` payload bytes; the walker treats
/// whatever it finds as opaque).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StreamEndPacket<'a> {
    payload: &'a [u8],
}

impl<'a> StreamEndPacket<'a> {
    /// Opaque payload bytes (typically empty).
    pub fn payload_bytes(&self) -> &'a [u8] {
        self.payload
    }
}

/// Typed view of one SV8 packet.
///
/// Built from a [`PacketRef`] via [`TypedPacket::classify`]: maps
/// each spec §3.2 packet key to its per-kind newtype wrapper, and
/// preserves any out-of-vocabulary 2-byte key in
/// [`TypedPacket::Unknown`] for forward compatibility (the
/// pending observer-trace round may surface new keys).
///
/// Every variant carries the same opaque payload slice the upstream
/// walker emitted — this module does NOT decode any payload field.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TypedPacket<'a> {
    /// `SH` Stream Header (spec §3.2).
    StreamHeader(StreamHeaderPacket<'a>),
    /// `RG` ReplayGain (spec §3.2).
    ReplayGain(ReplayGainPacket<'a>),
    /// `EI` Encoder Info (spec §3.2).
    EncoderInfo(EncoderInfoPacket<'a>),
    /// `SO` Seek-table Offset (spec §3.2).
    SeekTableOffset(SeekTableOffsetPacket<'a>),
    /// `ST` Seek Table (spec §3.2).
    SeekTable(SeekTablePacket<'a>),
    /// `AP` Audio Packet (spec §3.2 / §3.4).
    Audio(AudioPacket<'a>),
    /// `SE` Stream End terminator (spec §3.2).
    StreamEnd(StreamEndPacket<'a>),
    /// A 2-byte ASCII key not in the §3.2 vocabulary. The raw bytes
    /// are preserved alongside the opaque payload slice.
    Unknown {
        /// Raw 2-byte ASCII key as it appeared in the stream.
        key: [u8; 2],
        /// Opaque payload bytes.
        payload: &'a [u8],
    },
}

impl<'a> TypedPacket<'a> {
    /// Build a typed packet view from a [`PacketRef`] surfaced by
    /// [`crate::packet_stream::PacketStream::next_packet`].
    ///
    /// Pure classification: the call cannot fail, since the upstream
    /// walker has already validated the outer frame.
    pub fn classify(pkt: PacketRef<'a>) -> Self {
        match pkt.key {
            PacketKey::StreamHeader => Self::StreamHeader(StreamHeaderPacket {
                payload: pkt.payload,
            }),
            PacketKey::ReplayGain => Self::ReplayGain(ReplayGainPacket {
                payload: pkt.payload,
            }),
            PacketKey::EncoderInfo => Self::EncoderInfo(EncoderInfoPacket {
                payload: pkt.payload,
            }),
            PacketKey::SeekTableOffset => Self::SeekTableOffset(SeekTableOffsetPacket {
                payload: pkt.payload,
            }),
            PacketKey::SeekTable => Self::SeekTable(SeekTablePacket {
                payload: pkt.payload,
            }),
            PacketKey::AudioPacket => Self::Audio(AudioPacket {
                payload: pkt.payload,
            }),
            PacketKey::StreamEnd => Self::StreamEnd(StreamEndPacket {
                payload: pkt.payload,
            }),
            PacketKey::Unknown(raw) => Self::Unknown {
                key: raw,
                payload: pkt.payload,
            },
        }
    }

    /// Recover the underlying [`PacketKey`] discriminator, useful
    /// for callers that want to log / aggregate by key without
    /// matching every variant.
    pub fn key(&self) -> PacketKey {
        match self {
            Self::StreamHeader(_) => PacketKey::StreamHeader,
            Self::ReplayGain(_) => PacketKey::ReplayGain,
            Self::EncoderInfo(_) => PacketKey::EncoderInfo,
            Self::SeekTableOffset(_) => PacketKey::SeekTableOffset,
            Self::SeekTable(_) => PacketKey::SeekTable,
            Self::Audio(_) => PacketKey::AudioPacket,
            Self::StreamEnd(_) => PacketKey::StreamEnd,
            Self::Unknown { key, .. } => PacketKey::Unknown(*key),
        }
    }

    /// Opaque payload bytes, identical across every variant — handy
    /// for length-only logging or pass-through plumbing without
    /// re-matching the variant.
    pub fn payload_bytes(&self) -> &'a [u8] {
        match self {
            Self::StreamHeader(p) => p.payload_bytes(),
            Self::ReplayGain(p) => p.payload_bytes(),
            Self::EncoderInfo(p) => p.payload_bytes(),
            Self::SeekTableOffset(p) => p.payload_bytes(),
            Self::SeekTable(p) => p.payload_bytes(),
            Self::Audio(p) => p.payload_bytes(),
            Self::StreamEnd(p) => p.payload_bytes(),
            Self::Unknown { payload, .. } => payload,
        }
    }

    /// True if the variant is `SE` Stream End — handy for a typed
    /// stop-condition in caller loops.
    pub fn is_stream_end(&self) -> bool {
        matches!(self, Self::StreamEnd(_))
    }

    /// True if the variant is the §3.2 audio payload carrier (`AP`).
    pub fn is_audio(&self) -> bool {
        matches!(self, Self::Audio(_))
    }

    /// True if the variant is one of the §3.2 metadata / header
    /// packets (SH / RG / EI / SO / ST) — i.e. anything that is
    /// neither an audio frame nor the stream terminator nor an
    /// unrecognised key. Useful for filtering a stream walk down to
    /// the non-payload preamble.
    pub fn is_metadata(&self) -> bool {
        matches!(
            self,
            Self::StreamHeader(_)
                | Self::ReplayGain(_)
                | Self::EncoderInfo(_)
                | Self::SeekTableOffset(_)
                | Self::SeekTable(_)
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::framing::{PacketHeader, PacketKey};
    use crate::packet_stream::{PacketSizeConvention, PacketStream};

    /// Build a synthetic SV8 byte stream (without the `MPCK`
    /// magic) for the typed surface tests. Each tuple is
    /// `(2-byte key, 1-byte raw_size varint, payload)`.
    fn build_post_magic(packets: &[(&[u8; 2], u8, &[u8])]) -> Vec<u8> {
        let mut buf = Vec::new();
        for (key, size, payload) in packets {
            buf.extend_from_slice(&**key);
            buf.push(*size);
            buf.extend_from_slice(payload);
        }
        buf
    }

    fn synthetic_ref<'a>(key: PacketKey, payload: &'a [u8]) -> PacketRef<'a> {
        PacketRef {
            key,
            header: PacketHeader {
                key,
                raw_size: payload.len() as u64,
                header_len: 3,
            },
            payload,
        }
    }

    #[test]
    fn classify_routes_every_known_key_to_typed_variant() {
        let payload: &[u8] = &[0x11, 0x22, 0x33];
        let cases = [
            (PacketKey::StreamHeader, true, false, false),
            (PacketKey::ReplayGain, true, false, false),
            (PacketKey::EncoderInfo, true, false, false),
            (PacketKey::SeekTableOffset, true, false, false),
            (PacketKey::SeekTable, true, false, false),
            (PacketKey::AudioPacket, false, true, false),
            (PacketKey::StreamEnd, false, false, true),
        ];
        for (key, is_meta, is_audio, is_se) in cases {
            let p = synthetic_ref(key, payload);
            let tp = TypedPacket::classify(p);
            assert_eq!(tp.key(), key, "key round-trip failed for {key:?}");
            assert_eq!(tp.payload_bytes(), payload);
            assert_eq!(
                tp.is_metadata(),
                is_meta,
                "is_metadata mismatch for {key:?}"
            );
            assert_eq!(tp.is_audio(), is_audio, "is_audio mismatch for {key:?}");
            assert_eq!(
                tp.is_stream_end(),
                is_se,
                "is_stream_end mismatch for {key:?}"
            );
        }
    }

    #[test]
    fn classify_preserves_unknown_key_and_payload() {
        let payload: &[u8] = &[0xDE, 0xAD, 0xBE, 0xEF];
        let p = synthetic_ref(PacketKey::Unknown(*b"XY"), payload);
        let tp = TypedPacket::classify(p);
        assert_eq!(tp.key(), PacketKey::Unknown(*b"XY"));
        assert_eq!(tp.payload_bytes(), payload);
        assert!(!tp.is_metadata());
        assert!(!tp.is_audio());
        assert!(!tp.is_stream_end());
        // The variant itself preserves the raw bytes for log paths.
        match tp {
            TypedPacket::Unknown { key, payload: p2 } => {
                assert_eq!(key, *b"XY");
                assert_eq!(p2, payload);
            }
            other => panic!("expected TypedPacket::Unknown, got {other:?}"),
        }
    }

    #[test]
    fn typed_walk_over_packet_stream_emits_expected_sequence() {
        // SH (3 bytes) + RG (2 bytes) + EI (1 byte) + SO (1 byte) +
        // ST (4 bytes) + AP (5 bytes) + SE (empty) — all sizes under
        // the exclusive varint convention.
        let raw = build_post_magic(&[
            (b"SH", 3, &[0x10, 0x20, 0x30]),
            (b"RG", 2, &[0xAA, 0xBB]),
            (b"EI", 1, &[0x55]),
            (b"SO", 1, &[0x77]),
            (b"ST", 4, &[1, 2, 3, 4]),
            (b"AP", 5, &[9, 8, 7, 6, 5]),
            (b"SE", 0, &[]),
        ]);
        let mut s = PacketStream::new(&raw, PacketSizeConvention::Exclusive);
        let mut seq: Vec<TypedPacket<'_>> = Vec::new();
        while let Some(pkt) = s.next_packet().unwrap() {
            seq.push(TypedPacket::classify(pkt));
        }
        assert_eq!(seq.len(), 7);
        assert!(matches!(seq[0], TypedPacket::StreamHeader(_)));
        assert!(matches!(seq[1], TypedPacket::ReplayGain(_)));
        assert!(matches!(seq[2], TypedPacket::EncoderInfo(_)));
        assert!(matches!(seq[3], TypedPacket::SeekTableOffset(_)));
        assert!(matches!(seq[4], TypedPacket::SeekTable(_)));
        assert!(matches!(seq[5], TypedPacket::Audio(_)));
        assert!(matches!(seq[6], TypedPacket::StreamEnd(_)));
        // Spot-check a payload accessor mid-stream.
        if let TypedPacket::Audio(ap) = seq[5] {
            assert_eq!(ap.payload_bytes(), &[9, 8, 7, 6, 5]);
        } else {
            panic!("expected AP at index 5");
        }
    }

    #[test]
    fn metadata_filter_keeps_only_header_family() {
        let raw = build_post_magic(&[
            (b"SH", 1, &[0xAA]),
            (b"AP", 2, &[0xBB, 0xCC]),
            (b"RG", 1, &[0xDD]),
            (b"AP", 1, &[0xEE]),
            (b"SE", 0, &[]),
        ]);
        let mut s = PacketStream::new(&raw, PacketSizeConvention::Exclusive);
        let mut meta_count = 0usize;
        let mut audio_count = 0usize;
        let mut end_count = 0usize;
        while let Some(pkt) = s.next_packet().unwrap() {
            let tp = TypedPacket::classify(pkt);
            if tp.is_metadata() {
                meta_count += 1;
            }
            if tp.is_audio() {
                audio_count += 1;
            }
            if tp.is_stream_end() {
                end_count += 1;
            }
        }
        assert_eq!(meta_count, 2, "SH + RG");
        assert_eq!(audio_count, 2, "two AP packets");
        assert_eq!(end_count, 1, "single SE");
    }

    #[test]
    fn payload_bytes_consistent_across_variant_and_match() {
        // Whatever the variant, the top-level payload_bytes accessor
        // must equal the inner newtype's payload_bytes() (or the
        // Unknown variant's payload field).
        for (key, name) in [
            (PacketKey::StreamHeader, "SH"),
            (PacketKey::ReplayGain, "RG"),
            (PacketKey::EncoderInfo, "EI"),
            (PacketKey::SeekTableOffset, "SO"),
            (PacketKey::SeekTable, "ST"),
            (PacketKey::AudioPacket, "AP"),
            (PacketKey::StreamEnd, "SE"),
        ] {
            let payload = [0x42u8; 4];
            let tp = TypedPacket::classify(synthetic_ref(key, &payload));
            let top = tp.payload_bytes();
            let inner = match tp {
                TypedPacket::StreamHeader(p) => p.payload_bytes(),
                TypedPacket::ReplayGain(p) => p.payload_bytes(),
                TypedPacket::EncoderInfo(p) => p.payload_bytes(),
                TypedPacket::SeekTableOffset(p) => p.payload_bytes(),
                TypedPacket::SeekTable(p) => p.payload_bytes(),
                TypedPacket::Audio(p) => p.payload_bytes(),
                TypedPacket::StreamEnd(p) => p.payload_bytes(),
                TypedPacket::Unknown { payload: p, .. } => p,
            };
            assert_eq!(top, inner, "payload mismatch for {name}");
            assert_eq!(top, &payload[..], "payload contents for {name}");
        }
    }

    #[test]
    fn empty_payload_round_trips_through_classify() {
        // SE typically ships with empty payload, but every other
        // kind must also tolerate it (the structural spec doesn't
        // forbid a zero-length body for any §3.2 packet).
        for key in [
            PacketKey::StreamHeader,
            PacketKey::ReplayGain,
            PacketKey::EncoderInfo,
            PacketKey::SeekTableOffset,
            PacketKey::SeekTable,
            PacketKey::AudioPacket,
            PacketKey::StreamEnd,
        ] {
            let tp = TypedPacket::classify(synthetic_ref(key, &[]));
            assert!(tp.payload_bytes().is_empty(), "empty payload for {key:?}");
            assert_eq!(tp.key(), key);
        }
    }

    #[test]
    fn unknown_key_passes_through_walker_into_typed_unknown() {
        // The walker reports unknown 2-byte keys via
        // PacketKey::Unknown; classify() must preserve them as
        // TypedPacket::Unknown without erroring.
        let raw = build_post_magic(&[(b"ZQ", 2, &[0xC0, 0xDE]), (b"SE", 0, &[])]);
        let mut s = PacketStream::new(&raw, PacketSizeConvention::Exclusive);
        let first = s.next_packet().unwrap().expect("ZQ");
        let tp = TypedPacket::classify(first);
        match tp {
            TypedPacket::Unknown { key, payload } => {
                assert_eq!(key, *b"ZQ");
                assert_eq!(payload, &[0xC0, 0xDE]);
            }
            other => panic!("expected TypedPacket::Unknown, got {other:?}"),
        }
        let second = s.next_packet().unwrap().expect("SE");
        let tp_se = TypedPacket::classify(second);
        assert!(tp_se.is_stream_end());
    }

    #[test]
    fn typed_packet_is_copy_and_eq() {
        // The variants are Copy + PartialEq + Eq so callers can keep
        // a typed packet around without lifetime gymnastics.
        let payload: &[u8] = &[0xAB, 0xCD];
        let tp = TypedPacket::classify(synthetic_ref(PacketKey::AudioPacket, payload));
        let copy = tp;
        assert_eq!(tp, copy);
        // And the inner newtype is Copy too.
        if let TypedPacket::Audio(ap) = tp {
            let ap2 = ap;
            assert_eq!(ap, ap2);
        } else {
            unreachable!("expected Audio variant");
        }
    }

    #[test]
    fn classify_is_independent_of_packet_header_size_fields() {
        // Two PacketRefs that classify to the same TypedPacket
        // variant even when their PacketHeader::raw_size /
        // header_len differ — classify() only consults the key and
        // the payload slice.
        let payload: &[u8] = &[1, 2, 3];
        let ref_a = PacketRef {
            key: PacketKey::ReplayGain,
            header: PacketHeader {
                key: PacketKey::ReplayGain,
                raw_size: 3,
                header_len: 3,
            },
            payload,
        };
        let ref_b = PacketRef {
            key: PacketKey::ReplayGain,
            header: PacketHeader {
                key: PacketKey::ReplayGain,
                raw_size: 12345,
                header_len: 9,
            },
            payload,
        };
        assert_eq!(TypedPacket::classify(ref_a), TypedPacket::classify(ref_b));
    }

    #[test]
    fn predicates_are_mutually_exclusive() {
        // is_metadata / is_audio / is_stream_end are disjoint —
        // exactly one (or zero, for Unknown) of them is true per
        // typed packet.
        let payload: &[u8] = &[];
        for key in [
            PacketKey::StreamHeader,
            PacketKey::ReplayGain,
            PacketKey::EncoderInfo,
            PacketKey::SeekTableOffset,
            PacketKey::SeekTable,
            PacketKey::AudioPacket,
            PacketKey::StreamEnd,
            PacketKey::Unknown(*b"XY"),
        ] {
            let tp = TypedPacket::classify(synthetic_ref(key, payload));
            let truths = [tp.is_metadata(), tp.is_audio(), tp.is_stream_end()];
            let set = truths.iter().filter(|b| **b).count();
            assert!(set <= 1, "predicates not disjoint for {key:?}: {truths:?}");
        }
    }
}
