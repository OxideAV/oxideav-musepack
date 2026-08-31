//! SV8 seek layer — `SO` / `ST` payload field maps and the seek
//! index (headers-and-coding §9).
//!
//! Closes the last SV8 packet-payload gap: the seek-table offset
//! (`SO`), the seek table itself (`ST`), and the empty stream-end
//! (`SE`) payloads, per
//! `docs/audio/musepack/spec/musepack-headers-and-coding.md` §9:
//!
//! - **§9.0 packet skeleton** — `MPCK SH RG EI SO AP… ST SE`: `SO` is
//!   a fixed-width forward reference written before the audio (so an
//!   encoder can back-patch it), and the table itself lands after the
//!   last `AP`.
//! - **§9.1 `SO`** — a §3 varint giving the byte distance from the
//!   first byte of the `SO` packet to the first byte of the `ST`
//!   packet, zero-padded to a fixed 5-byte payload (8-byte packet).
//! - **§9.2 `ST`** — a bit-packed payload, MSB-first, not byte
//!   aligned after the first field: entry-count varint, a 4-bit
//!   `seek_pwr_delta`, two varint absolute entries, then per-entry
//!   **second-difference residuals under a `k = 12` Golomb code**
//!   with the sign in the code word's least-significant bit. Entries
//!   are byte offsets of `AP` packet starts **relative to
//!   `header_position`** (the `MPCK` magic), one entry every
//!   `2^seek_pwr` frames where `seek_pwr = block_pwr +
//!   seek_pwr_delta`.
//! - **§9.3 `SE`** — empty payload.
//!
//! On top of the wire maps this module builds [`Sv8SeekIndex`]: the
//! decoded table resolved to absolute `AP` byte offsets plus the
//! frame-granularity bookkeeping needed to enter the packet stream at
//! an arbitrary frame. Because every `AP` opens with a key frame and
//! the cross-frame entropy state resets at each packet boundary
//! (spec §3.3 + the r419 fixture pinning), entropy decode from any
//! `AP` start is exact; only the synthesis filterbank enters cold, so
//! the first [`crate::synthesis::SYNTHESIS_PRIME_SAMPLES`] output
//! samples per channel of a mid-stream entry are the priming
//! transient (`tests/sv8_seek_corpus.rs` measures the post-transient
//! agreement against the linear whole-stream decode).
//!
//! Source-of-record (facts only): headers-and-coding §9 (field maps,
//! Golomb parameter, reference point, thinning policy), §3 (varint
//! packing), §2 (`SH` `block_power`); `musepack-sv7-sv8-spec.md` §3.1
//! / §3.2 (packet stream + vocabulary). The §9.1/§9.3 rows are
//! black-box stream measurements and §9.2 is a facts-only extraction
//! — both staged in `docs/`; no external source consulted here.

use crate::framing::{parse_sv8_magic, parse_varint, SV8_MAGIC};
use crate::huffman::Sv7BitReader;
use crate::packet_stream::{PacketSizeConvention, PacketStream};
use crate::sv7_bitwriter::Sv7BitWriter;
use crate::typed_packet::TypedPacket;
use crate::{Error, Result};

/// §9.1: the `SO` payload is zero-padded to this fixed length, so the
/// packet can be written before the offset is known and back-patched.
pub const SO_PAYLOAD_LEN: usize = 5;

/// §9.2: the Golomb parameter for the entry-residual code — `k` raw
/// bits after the leading-zeros run, minimum code length `k + 1`.
pub const SEEK_GOLOMB_K: u8 = 12;

/// §9.2: the reference decoder's seek-table capacity ceiling used by
/// the table-thinning policy (entries kept in memory, not a wire
/// bound).
pub const SEEK_TABLE_CAP: u64 = 65536;

/// Sanity bound on a decoded entry position (bytes from
/// `header_position`): 2^40 — far beyond the 35-bit `SO` reach and
/// any real stream, tight enough that the §9.2 second-order
/// extrapolation arithmetic can never overflow on hostile input.
pub const SEEK_ENTRY_BOUND: u64 = 1 << 40;

/// Decoded `SO` (seek-table offset) payload — headers-and-coding §9.1.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SeekOffsetFields {
    /// Byte distance from the first byte of the `SO` packet (its key
    /// byte) to the first byte of the `ST` packet:
    /// `ST_position = SO_position + st_offset`.
    pub st_offset: u64,
}

impl SeekOffsetFields {
    /// Parse an `SO` payload: one §3 varint, then reserved padding
    /// (zero in every corpus stream; ignored here — §9.1 marks the
    /// tail as back-patch slack, not data).
    ///
    /// # Errors
    ///
    /// - [`Error::UnexpectedEof`] on an empty payload.
    /// - [`Error::VarintTooLong`] on an overlong varint.
    pub fn parse(payload: &[u8]) -> Result<Self> {
        let (st_offset, _len) = parse_varint(payload)?;
        Ok(Self { st_offset })
    }

    /// Compose the fixed 5-byte §9.1 payload: the offset varint,
    /// zero-padded to [`SO_PAYLOAD_LEN`].
    ///
    /// # Errors
    ///
    /// [`Error::HeaderFieldOutOfRange`] if the offset needs more than
    /// the 5 varint bytes (35 bits) the fixed payload can hold.
    pub fn payload(&self) -> Result<[u8; SO_PAYLOAD_LEN]> {
        if self.st_offset >= 1u64 << 35 {
            return Err(Error::HeaderFieldOutOfRange("SO st_offset"));
        }
        let mut varint = Vec::with_capacity(SO_PAYLOAD_LEN);
        crate::sv8_file_encode::write_varint(&mut varint, self.st_offset);
        let mut out = [0u8; SO_PAYLOAD_LEN];
        out[..varint.len()].copy_from_slice(&varint);
        Ok(out)
    }
}

/// Decoded `ST` (seek table) payload — headers-and-coding §9.2.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SeekTableFields {
    /// The 4-bit seek-granularity exponent delta:
    /// `seek_pwr = block_pwr + seek_pwr_delta` (frames per entry =
    /// `2^seek_pwr`; equivalently one entry every `2^seek_pwr_delta`
    /// `AP` packets).
    pub seek_pwr_delta: u8,
    /// The decoded entries, in stream order: byte offsets of `AP`
    /// packet starts **relative to `header_position`** (the byte
    /// position of the `MPCK` magic). `entries.len()` is the wire
    /// `n_entries`.
    pub entries: Vec<u64>,
}

/// Read a §3 varint through the bit reader, 8 bits at a time, from
/// wherever the bit cursor currently sits (§9.2: the table's varints
/// are not byte aligned after the `seek_pwr` nibble).
fn read_bit_varint(reader: &mut Sv7BitReader<'_>) -> Result<u64> {
    let mut value: u64 = 0;
    for _ in 0..10 {
        let byte = reader.read_bits(8)?;
        value = (value << 7) | u64::from(byte & 0x7F);
        if byte & 0x80 == 0 {
            return Ok(value);
        }
    }
    Err(Error::VarintTooLong)
}

impl SeekTableFields {
    /// Parse an `ST` payload per §9.2: `n_entries` varint, 4-bit
    /// `seek_pwr_delta`, two varint absolute entries, then Golomb
    /// `k = 12` second-difference residuals. Trailing bits are
    /// padding.
    ///
    /// The parse works in the **byte domain** (offsets relative to
    /// `header_position`); the §9.2 residual mapping's `<< 2`
    /// bit-domain rescale is divided out, i.e. residuals are applied
    /// as `pos[i] = d2 + 2·pos[i−1] − pos[i−2]` with
    /// `d2 = ±(code >> 1)` bytes.
    ///
    /// # Errors
    ///
    /// - [`Error::UnexpectedEof`] if the payload runs out mid-field
    ///   (including a hostile `n_entries` that promises more entries
    ///   than the payload's bits can carry).
    /// - [`Error::VarintTooLong`] on an overlong varint.
    /// - [`Error::SeekTableCorrupt`] if a reconstructed entry goes
    ///   negative or a Golomb zero-run exceeds the 35-bit offset
    ///   bound (a malformed / hostile residual stream).
    pub fn parse(payload: &[u8]) -> Result<Self> {
        let mut reader = Sv7BitReader::new(payload);
        let n_entries = read_bit_varint(&mut reader)?;
        // Defensive ceiling before any allocation: every entry costs
        // ≥ 8 bits (a 1-byte varint) for the first two and ≥ 13 bits
        // (the Golomb floor) after — a count the payload cannot carry
        // is corrupt however the reads would fall.
        let max_entries = 2 + (payload.len() as u64) * 8 / u64::from(SEEK_GOLOMB_K + 1);
        if n_entries > max_entries {
            return Err(Error::SeekTableCorrupt("n_entries exceeds payload bits"));
        }
        let seek_pwr_delta = reader.read_bits(4)? as u8;
        let n = n_entries as usize;
        let mut entries: Vec<u64> = Vec::with_capacity(n);
        for _ in 0..n.min(2) {
            let e = read_bit_varint(&mut reader)?;
            if e >= SEEK_ENTRY_BOUND {
                return Err(Error::SeekTableCorrupt("entry position out of bounds"));
            }
            entries.push(e);
        }
        for i in 2..n {
            // Golomb k=12: `l` leading zeros, a terminating 1, then
            // 12 raw bits; code = (l << 12) | raw.
            let mut l: u64 = 0;
            while reader.read_bits(1)? == 0 {
                l += 1;
                if l > 35 {
                    return Err(Error::SeekTableCorrupt("Golomb zero-run overflow"));
                }
            }
            let raw = u64::from(reader.read_bits(SEEK_GOLOMB_K)?);
            let code = (l << SEEK_GOLOMB_K) | raw;
            // Sign in the LSB: odd ⇒ negative second difference.
            let magnitude = (code >> 1) as i64;
            let d2 = if code & 1 != 0 { -magnitude } else { magnitude };
            // Both predecessors are < 2^40 and |d2| < 2^47, so this
            // cannot overflow.
            let pos = d2 + 2 * entries[i - 1] as i64 - entries[i - 2] as i64;
            if pos < 0 {
                return Err(Error::SeekTableCorrupt("entry position went negative"));
            }
            if pos as u64 >= SEEK_ENTRY_BOUND {
                return Err(Error::SeekTableCorrupt("entry position out of bounds"));
            }
            entries.push(pos as u64);
        }
        Ok(Self {
            seek_pwr_delta,
            entries,
        })
    }

    /// Compose the §9.2 payload for this table (the exact inverse of
    /// [`SeekTableFields::parse`]): `n_entries` varint, the 4-bit
    /// `seek_pwr_delta`, two varint entries, Golomb `k = 12`
    /// second-difference residuals, zero-padded to a byte.
    ///
    /// # Errors
    ///
    /// [`Error::SeekTableCorrupt`] if `seek_pwr_delta` exceeds its
    /// 4-bit field or an entry sequence needs a residual whose Golomb
    /// zero-run would exceed the offset bound (never the case for
    /// monotone `AP` offsets within the 35-bit `SO` reach).
    pub fn payload(&self) -> Result<Vec<u8>> {
        if self.seek_pwr_delta > 0xF {
            return Err(Error::SeekTableCorrupt("seek_pwr_delta exceeds 4 bits"));
        }
        if self.entries.iter().any(|&e| e >= SEEK_ENTRY_BOUND) {
            return Err(Error::SeekTableCorrupt("entry position out of bounds"));
        }
        let mut w = Sv7BitWriter::new();
        write_bit_varint(&mut w, self.entries.len() as u64);
        w.write_bits(u32::from(self.seek_pwr_delta), 4);
        for (i, &e) in self.entries.iter().enumerate() {
            if i < 2 {
                write_bit_varint(&mut w, e);
                continue;
            }
            let d2 = e as i64 - 2 * self.entries[i - 1] as i64 + self.entries[i - 2] as i64;
            // §9.2 encoder mapping: shift the magnitude left by one to
            // free the sign bit; set the low bit for a negative value
            // (both 0 and 1 denote zero — an encoder emits 0).
            let code = (d2.unsigned_abs() << 1) | u64::from(d2 < 0);
            let l = code >> SEEK_GOLOMB_K;
            if l > 35 {
                return Err(Error::SeekTableCorrupt("residual exceeds Golomb bound"));
            }
            for _ in 0..l {
                w.write_bits(0, 1);
            }
            w.write_bits(1, 1);
            w.write_bits((code & ((1 << SEEK_GOLOMB_K) - 1)) as u32, SEEK_GOLOMB_K);
        }
        Ok(w.finish())
    }

    /// The effective seek granularity exponent for a stream with the
    /// given `SH` `block_power`: one entry every `2^seek_pwr` frames
    /// (§9.2 `seek_pwr = block_pwr + seek_pwr_delta`, with the `SH`
    /// field's `× 2` block-exponent scaling applied by the caller via
    /// [`crate::sh_header::StreamHeaderFields::frames_per_audio_packet`]
    /// — this helper takes the *effective* block exponent).
    #[must_use]
    pub fn seek_pwr(&self, effective_block_pwr: u8) -> u8 {
        effective_block_pwr + self.seek_pwr_delta
    }
}

/// Append a §3 varint through the bit writer, 8 bits at a time (the
/// `ST` payload's varints are not byte aligned).
fn write_bit_varint(w: &mut Sv7BitWriter, value: u64) {
    let mut groups = [0u8; 10];
    let mut n = 0;
    let mut v = value;
    loop {
        groups[n] = (v & 0x7F) as u8;
        n += 1;
        v >>= 7;
        if v == 0 {
            break;
        }
    }
    for i in (0..n).rev() {
        let cont = if i > 0 { 0x80 } else { 0 };
        w.write_bits(u32::from(groups[i] | cont), 8);
    }
}

/// A resolved seek index over one in-memory SV8 stream: the `ST`
/// entries as **absolute byte offsets** into the buffer, plus the
/// frame bookkeeping to enter the stream at an arbitrary frame.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Sv8SeekIndex {
    /// Absolute byte offsets (into the buffer the index was built
    /// from) of the indexed `AP` packets' key bytes.
    pub positions: Vec<u64>,
    /// `AP` packets between consecutive entries (`2^seek_pwr_delta`;
    /// 1 when the index was built by walking every packet).
    pub packets_per_entry: u64,
    /// Frames carried per `AP` packet (`SH` §2 field 9).
    pub frames_per_packet: u64,
}

impl Sv8SeekIndex {
    /// Frames between consecutive index entries.
    #[must_use]
    pub fn frames_per_entry(&self) -> u64 {
        self.packets_per_entry * self.frames_per_packet
    }

    /// The index entry covering `frame` (0-based, counted from the
    /// stream's first coded frame), and the frame number at which
    /// that entry's decode begins. Returns `None` on an empty index.
    #[must_use]
    pub fn entry_for_frame(&self, frame: u64) -> Option<(usize, u64)> {
        if self.positions.is_empty() {
            return None;
        }
        let fpe = self.frames_per_entry().max(1);
        let idx = ((frame / fpe) as usize).min(self.positions.len() - 1);
        Some((idx, idx as u64 * fpe))
    }

    /// Build the index from a stream's own `SO` → `ST` seek packets
    /// (§9.0/§9.1/§9.2). Walks the packet stream only until the `SO`
    /// packet, then jumps straight to the table.
    ///
    /// Returns `Ok(None)` when the stream carries no `SO` packet
    /// before its audio (a legal stream — the seek layer is
    /// optional); use [`Sv8SeekIndex::from_packet_walk`] to build an
    /// index for such a stream by scanning.
    ///
    /// # Errors
    ///
    /// - [`Error::InvalidMagic`] if the buffer does not start with
    ///   `MPCK`.
    /// - [`Error::SeekTableCorrupt`] if the `SO` offset lands outside
    ///   the buffer or on a non-`ST` packet, or the table fails its
    ///   §9.2 parse.
    /// - Any packet-walk error of the prefix scan.
    pub fn from_seek_packets(input: &[u8]) -> Result<Option<Self>> {
        Self::from_seek_packets_with_ceiling(input, SEEK_TABLE_CAP)
    }

    /// [`Sv8SeekIndex::from_seek_packets`] with an explicit capacity
    /// ceiling — the §9.2 **decoder-side table-thinning** policy. When
    /// the stream's own capacity estimate
    /// `cap = 2 + total_samples / (1152 << seek_pwr)` exceeds
    /// `ceiling`, both `seek_pwr` and a counter `diff_pwr` are
    /// incremented until it fits; only every `2^diff_pwr`-th decoded
    /// entry is then retained, and a stored entry count beyond
    /// `cap << diff_pwr` (more entries than the `SH` sample count can
    /// justify) is clamped. Every retained-or-not entry is still
    /// *decoded* in sequence — each §9.2 residual depends on its two
    /// predecessors — so this is a memory policy, not a wire-format
    /// change; `diff_pwr` is 0 for any ordinary file at the default
    /// ceiling ([`SEEK_TABLE_CAP`], the reference posture).
    ///
    /// # Errors
    ///
    /// As [`Sv8SeekIndex::from_seek_packets`].
    pub fn from_seek_packets_with_ceiling(input: &[u8], ceiling: u64) -> Result<Option<Self>> {
        let after_magic = parse_sv8_magic(input)?;
        let total = input.len() - after_magic;
        let mut stream = PacketStream::new(&input[after_magic..], PacketSizeConvention::Inclusive);

        let mut frames_per_packet = 0u64;
        let mut sample_count: Option<u64> = None;
        let mut so: Option<(u64, SeekOffsetFields)> = None;
        while let Some(packet) = stream.next_packet()? {
            // Offset (relative to header_position) of the *next*
            // packet is total − remaining; recover this packet's own
            // start from its extent.
            let next_off = (total - stream.remaining_bytes().len()) as u64;
            match TypedPacket::classify(packet) {
                TypedPacket::StreamHeader(sh) => {
                    let fields = sh.fields()?;
                    frames_per_packet = fields.frames_per_audio_packet();
                    sample_count = Some(fields.sample_count);
                }
                TypedPacket::SeekTableOffset(pkt) => {
                    let extent = packet.header.header_len as u64 + packet.payload.len() as u64;
                    so = Some((
                        next_off - extent,
                        SeekOffsetFields::parse(pkt.payload_bytes())?,
                    ));
                    break;
                }
                // The §9.0 skeleton puts SO before the audio; a
                // stream that reaches its AP run without one carries
                // no seek layer.
                TypedPacket::Audio(_) | TypedPacket::StreamEnd(_) => break,
                _ => {}
            }
        }
        let Some((so_pos, so_fields)) = so else {
            return Ok(None);
        };

        // §9.1: ST_position = SO_position + st_offset, both relative
        // to header_position.
        let st_pos = so_pos + so_fields.st_offset;
        let st_index = usize::try_from(st_pos)
            .ok()
            .and_then(|p| p.checked_add(after_magic))
            .filter(|&p| p < input.len())
            .ok_or(Error::SeekTableCorrupt("SO offset outside the buffer"))?;
        let mut st_stream = PacketStream::new(&input[st_index..], PacketSizeConvention::Inclusive);
        let st_packet = st_stream
            .next_packet()?
            .ok_or(Error::SeekTableCorrupt("SO offset points at no packet"))?;
        let TypedPacket::SeekTable(st) = TypedPacket::classify(st_packet) else {
            return Err(Error::SeekTableCorrupt(
                "SO offset points at a non-ST packet",
            ));
        };
        let mut table = SeekTableFields::parse(st.payload_bytes())?;

        // §9.2 decoder-side thinning (see the method docs). Only
        // meaningful with a parsed `SH` (the §9.0 skeleton places one
        // before `SO`; without it there is no sample count to bound
        // by).
        let mut diff_pwr = 0u32;
        if let Some(samples) = sample_count {
            let frame_len = crate::SAMPLES_PER_FRAME_PER_CHANNEL as u64;
            let eff_block_pwr = frames_per_packet.max(1).trailing_zeros();
            let ceiling = ceiling.max(2);
            let cap_for =
                |sp: u32| 2 + samples / frame_len.checked_shl(sp).unwrap_or(u64::MAX).max(1);
            let mut seek_pwr = eff_block_pwr + u32::from(table.seek_pwr_delta);
            let mut cap = cap_for(seek_pwr);
            while cap > ceiling {
                seek_pwr += 1;
                diff_pwr += 1;
                cap = cap_for(seek_pwr);
            }
            let justified = cap.checked_shl(diff_pwr).unwrap_or(u64::MAX);
            if table.entries.len() as u64 > justified {
                table.entries.truncate(justified as usize);
            }
        }

        // Resolve to absolute buffer offsets. Entries are relative to
        // header_position (the MPCK magic) — §9.2 "Reference point".
        let base = (after_magic - SV8_MAGIC.len()) as u64;
        let positions = table
            .entries
            .iter()
            .step_by(1usize << diff_pwr.min(63))
            .map(|&e| base + e)
            .collect();
        Ok(Some(Self {
            positions,
            packets_per_entry: (1u64 << table.seek_pwr_delta) << diff_pwr,
            frames_per_packet,
        }))
    }

    /// Build a full-granularity index (every `AP` packet) by walking
    /// the whole packet stream — the fallback for streams without a
    /// seek layer, and the ground truth the corpus gates compare the
    /// `ST` table against.
    ///
    /// # Errors
    ///
    /// [`Error::InvalidMagic`] / packet-walk errors.
    pub fn from_packet_walk(input: &[u8]) -> Result<Self> {
        let after_magic = parse_sv8_magic(input)?;
        let total = input.len() - after_magic;
        let mut stream = PacketStream::new(&input[after_magic..], PacketSizeConvention::Inclusive);
        let mut frames_per_packet = 0u64;
        let mut positions = Vec::new();
        while let Some(packet) = stream.next_packet()? {
            let next_off = total - stream.remaining_bytes().len();
            match TypedPacket::classify(packet) {
                TypedPacket::StreamHeader(sh) => {
                    frames_per_packet = sh.fields()?.frames_per_audio_packet();
                }
                TypedPacket::Audio(_) => {
                    let extent = packet.header.header_len + packet.payload.len();
                    positions.push((after_magic + next_off - extent) as u64);
                }
                TypedPacket::StreamEnd(_) => break,
                _ => {}
            }
        }
        Ok(Self {
            positions,
            packets_per_entry: 1,
            frames_per_packet,
        })
    }
}

/// The result of a random-access decode entered mid-stream via an
/// [`Sv8SeekIndex`] entry.
#[derive(Debug, Clone, PartialEq)]
pub struct Sv8SeekDecode {
    /// The stream frame number (0-based, from the first coded frame)
    /// at which `pcm` begins.
    pub first_frame: u64,
    /// Decoded PCM from `first_frame` to the end of the coded run, in
    /// the **untrimmed decoded timeline** (absolute s16 domain,
    /// interleaved for stereo): sample `t` per channel corresponds to
    /// decoded-timeline position `first_frame × 1152 + t`. No gapless
    /// window is applied — a caller seeking within the output
    /// timeline offsets by `481 + beginning_silence` itself. The
    /// first [`crate::synthesis::SYNTHESIS_PRIME_SAMPLES`] samples
    /// per channel are the cold-filterbank priming transient (for
    /// `first_frame > 0` they approximate the linear decode; beyond
    /// them the decode is exact — the corpus gates measure this).
    pub pcm: Vec<f64>,
}

/// Decode an SV8 stream from seek-index entry `entry` to the end of
/// its coded run — random access per headers-and-coding §9.
///
/// Entropy state is exact from any `AP` start (each `AP` opens with a
/// key frame and the cross-frame state resets per packet — spec §3.3);
/// the synthesis filterbank starts cold, so the leading
/// [`crate::synthesis::SYNTHESIS_PRIME_SAMPLES`] output samples are a
/// transient (see [`Sv8SeekDecode::pcm`]).
///
/// # Errors
///
/// - [`Error::SeekTableCorrupt`] if `entry` is out of range for the
///   index or an indexed position does not parse as an `AP` packet.
/// - [`Error::InvalidMagic`] / header / packet / frame-decode errors
///   of the underlying walkers.
pub fn decode_sv8_from_entry(
    input: &[u8],
    index: &Sv8SeekIndex,
    entry: usize,
) -> Result<Sv8SeekDecode> {
    let after_magic = parse_sv8_magic(input)?;
    let Some(&entry_pos) = index.positions.get(entry) else {
        return Err(Error::SeekTableCorrupt("seek entry out of range"));
    };
    let first_frame = entry as u64 * index.frames_per_entry();

    // Re-read the SH header from the stream prefix for the decode
    // parameters and stream totals.
    let mut prefix = PacketStream::new(&input[after_magic..], PacketSizeConvention::Inclusive);
    let mut header = None;
    while let Some(packet) = prefix.next_packet()? {
        match TypedPacket::classify(packet) {
            TypedPacket::StreamHeader(sh) => {
                header = Some(sh.fields()?);
                break;
            }
            TypedPacket::Audio(_) | TypedPacket::StreamEnd(_) => break,
            _ => {}
        }
    }
    let header = header.ok_or(Error::NotImplemented)?;
    let mut decoder = crate::sv8_stream::Sv8StreamDecoder::from_header(&header)?;
    let total_frames = header
        .sample_count
        .div_ceil(crate::SAMPLES_PER_FRAME_PER_CHANNEL as u64);
    let mut frames_remaining = total_frames.saturating_sub(first_frame);
    let frames_per_packet = header.frames_per_audio_packet();

    let start = usize::try_from(entry_pos)
        .ok()
        .filter(|&p| p < input.len())
        .ok_or(Error::SeekTableCorrupt("seek entry outside the buffer"))?;
    let mut stream = PacketStream::new(&input[start..], PacketSizeConvention::Inclusive);
    let mut first = true;
    let mut pcm = Vec::new();
    while let Some(packet) = stream.next_packet()? {
        match TypedPacket::classify(packet) {
            TypedPacket::Audio(ap) => {
                first = false;
                let frames = frames_remaining.min(frames_per_packet);
                if frames == 0 {
                    continue;
                }
                pcm.extend_from_slice(&decoder.decode_audio_packet(ap.payload_bytes(), frames)?);
                frames_remaining -= frames;
            }
            TypedPacket::StreamEnd(_) => break,
            _ if first => {
                // §9.2 entries must point at AP packet starts; any
                // other packet kind here means the index is stale or
                // the table lied.
                return Err(Error::SeekTableCorrupt("seek entry is not an AP packet"));
            }
            _ => {}
        }
    }
    Ok(Sv8SeekDecode { first_frame, pcm })
}

#[cfg(test)]
mod tests {
    use super::*;

    // ─── SO payload (§9.1) ─────────────────────────────────

    #[test]
    fn so_payload_roundtrip_and_padding() {
        for off in [0u64, 44, 2143, 881_827, (1 << 35) - 1] {
            let so = SeekOffsetFields { st_offset: off };
            let payload = so.payload().expect("compose");
            assert_eq!(payload.len(), SO_PAYLOAD_LEN);
            let parsed = SeekOffsetFields::parse(&payload).expect("parse");
            assert_eq!(parsed.st_offset, off);
        }
        // §9.1 bound: 35 bits is the most the fixed payload can hold.
        assert_eq!(
            SeekOffsetFields { st_offset: 1 << 35 }.payload(),
            Err(Error::HeaderFieldOutOfRange("SO st_offset"))
        );
    }

    #[test]
    fn so_parse_empty_is_eof() {
        assert_eq!(SeekOffsetFields::parse(&[]), Err(Error::UnexpectedEof));
    }

    /// §9.1 corpus shape: a 1-byte varint (44) padded with zeros
    /// parses to 44 regardless of the padding bytes' presence.
    #[test]
    fn so_parse_ignores_backpatch_padding() {
        assert_eq!(
            SeekOffsetFields::parse(&[44, 0, 0, 0, 0])
                .unwrap()
                .st_offset,
            44
        );
        assert_eq!(SeekOffsetFields::parse(&[44]).unwrap().st_offset, 44);
    }

    // ─── ST payload (§9.2) ─────────────────────────────────

    #[test]
    fn st_roundtrip_zero_one_two_entries() {
        for entries in [vec![], vec![44], vec![44, 2100]] {
            let t = SeekTableFields {
                seek_pwr_delta: 1,
                entries: entries.clone(),
            };
            let payload = t.payload().expect("compose");
            let parsed = SeekTableFields::parse(&payload).expect("parse");
            assert_eq!(parsed.seek_pwr_delta, 1);
            assert_eq!(parsed.entries, entries);
        }
    }

    /// Constant-stride positions: every second difference is zero, so
    /// each residual costs exactly the 13-bit Golomb floor (§9.2 "why
    /// the entries are so cheap").
    #[test]
    fn st_roundtrip_constant_stride_costs_the_golomb_floor() {
        let entries: Vec<u64> = (0..54).map(|i| 44 + i * 2048).collect();
        let t = SeekTableFields {
            seek_pwr_delta: 1,
            entries: entries.clone(),
        };
        let payload = t.payload().expect("compose");
        // n_entries varint (8) + nibble (4) + 2 varints (8 + 16) +
        // 52 × 13 bits, byte-padded.
        let bits: usize = 8 + 4 + 8 + 16 + 52 * 13;
        assert_eq!(payload.len(), bits.div_ceil(8));
        assert_eq!(SeekTableFields::parse(&payload).unwrap().entries, entries);
    }

    /// Jittered strides exercise both residual signs and multi-run
    /// Golomb codes.
    #[test]
    fn st_roundtrip_jittered_and_large_residuals() {
        let mut entries: Vec<u64> = vec![44, 5000];
        let deltas: [i64; 9] = [4956, 6000, 3000, 3001, 2999, 20000, 100, 65000, 4096];
        for d in deltas {
            let last = *entries.last().unwrap() as i64;
            entries.push((last + d) as u64);
        }
        let t = SeekTableFields {
            seek_pwr_delta: 3,
            entries: entries.clone(),
        };
        let parsed = SeekTableFields::parse(&t.payload().unwrap()).unwrap();
        assert_eq!(parsed.entries, entries);
        assert_eq!(parsed.seek_pwr_delta, 3);
    }

    /// §9.2: `code = 0` and `code = 1` both denote a zero residual
    /// (the mapping is not a bijection; an encoder emits 0). Build
    /// the code-1 variant by hand and check it decodes like code 0.
    #[test]
    fn st_negative_zero_code_decodes_to_zero_residual() {
        let mut w = Sv7BitWriter::new();
        write_bit_varint(&mut w, 3); // n_entries
        w.write_bits(1, 4); // seek_pwr_delta
        write_bit_varint(&mut w, 44); // entry 0
        write_bit_varint(&mut w, 100); // entry 1
        w.write_bits(1, 1); // Golomb: l = 0 terminator
        w.write_bits(1, SEEK_GOLOMB_K); // raw = 1 ⇒ code 1 ⇒ d2 = −0
        let parsed = SeekTableFields::parse(&w.finish()).unwrap();
        assert_eq!(parsed.entries, vec![44, 100, 156]); // 2·100 − 44
    }

    #[test]
    fn st_hostile_n_entries_rejected_without_allocation() {
        // A varint promising u64::MAX-ish entries in a 6-byte payload.
        let payload = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F];
        assert_eq!(
            SeekTableFields::parse(&payload),
            Err(Error::SeekTableCorrupt("n_entries exceeds payload bits"))
        );
    }

    #[test]
    fn st_truncated_payload_is_eof() {
        let t = SeekTableFields {
            seek_pwr_delta: 1,
            entries: (0..20).map(|i| 44 + i * 2048).collect(),
        };
        let payload = t.payload().unwrap();
        // A cut that keeps the bit budget plausible fails with a
        // clean EOF mid-entry…
        assert_eq!(
            SeekTableFields::parse(&payload[..payload.len() - 1]),
            Err(Error::UnexpectedEof)
        );
        // …while a cut so deep the count can no longer fit the
        // payload's bits trips the pre-allocation ceiling instead.
        for cut in [1, payload.len() / 2] {
            assert_eq!(
                SeekTableFields::parse(&payload[..cut]),
                Err(Error::SeekTableCorrupt("n_entries exceeds payload bits"))
            );
        }
    }

    #[test]
    fn st_negative_position_rejected() {
        // Entries 100, 50: extrapolation 2·50 − 100 = 0; a negative
        // residual then drives the position below zero.
        let mut w = Sv7BitWriter::new();
        write_bit_varint(&mut w, 3);
        w.write_bits(1, 4);
        write_bit_varint(&mut w, 100);
        write_bit_varint(&mut w, 50);
        w.write_bits(1, 1); // l = 0
        w.write_bits(3, SEEK_GOLOMB_K); // code 3 ⇒ d2 = −1 ⇒ pos = −1
        assert_eq!(
            SeekTableFields::parse(&w.finish()),
            Err(Error::SeekTableCorrupt("entry position went negative"))
        );
    }

    // ─── bit-varint helper ─────────────────────────────────

    #[test]
    fn bit_varint_roundtrip_unaligned() {
        for v in [0u64, 1, 127, 128, 16383, 16384, u64::from(u32::MAX)] {
            let mut w = Sv7BitWriter::new();
            w.write_bits(0b101, 3); // misalign
            write_bit_varint(&mut w, v);
            let bytes = w.finish();
            let mut r = Sv7BitReader::new(&bytes);
            assert_eq!(r.read_bits(3).unwrap(), 0b101);
            assert_eq!(read_bit_varint(&mut r).unwrap(), v);
        }
    }

    // ─── Sv8SeekIndex frame bookkeeping ────────────────────

    #[test]
    fn entry_for_frame_maps_and_clamps() {
        let idx = Sv8SeekIndex {
            positions: vec![10, 20, 30],
            packets_per_entry: 2,
            frames_per_packet: 64,
        };
        assert_eq!(idx.frames_per_entry(), 128);
        assert_eq!(idx.entry_for_frame(0), Some((0, 0)));
        assert_eq!(idx.entry_for_frame(127), Some((0, 0)));
        assert_eq!(idx.entry_for_frame(128), Some((1, 128)));
        assert_eq!(idx.entry_for_frame(100_000), Some((2, 256)));
        let empty = Sv8SeekIndex {
            positions: vec![],
            packets_per_entry: 1,
            frames_per_packet: 64,
        };
        assert_eq!(empty.entry_for_frame(0), None);
    }
}
