//! Musepack container parsers — SV8 chunked (`MPCK`) and SV7
//! fixed-prefix (`MP+`).
//!
//! Both formats wrap the **same** 32-band subband audio coding (the
//! difference is in entropy details and packet framing), but the on-disk
//! byte layouts are completely different:
//!
//! * **SV8** — `MPCK` magic, then a sequence of self-delimiting
//!   `<2-byte ASCII tag><var-len size>` chunks until `SE` (Stream End).
//!   Audio lives in `AP` (Audio Packet) chunks, one per "frame block"
//!   of `1 << (2 * block_power)` 1152-sample sub-frames.
//!
//! * **SV7** — `MP+` magic followed by a 32-bit total-frame-count, a
//!   16-byte fixed header, then a **byte-unaligned** stream of audio
//!   frames each preceded by a 20-bit length prefix. The bit cursor
//!   crosses frame boundaries unconditionally.
//!
//! Reference: `docs/audio/musepack/musepack-trace-reverse-engineering.md`
//! §3 — the chunk taxonomy of §3.2.2 and the `SH` / `SE` field map of
//! §3.2.3 / §5.11.

use oxideav_core::{Error, Result};

use crate::tables::SAMPLE_RATES;
use crate::varlen::read_byte_varlen;

// ---------- SV8 ----------

/// SV8 file magic — `'MPCK'`.
pub const MAGIC_SV8: [u8; 4] = *b"MPCK";

/// SV7 file magic — `'MP+'` plus a stream-version byte (low nibble
/// must be `0x7`; valid bytes are `0x07` and `0x17` only).
pub const MAGIC_SV7: [u8; 3] = *b"MP+";

/// SV8 chunk tags. Two bytes of all-uppercase ASCII; unknown tags are
/// silently skipped by spec.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ChunkTag {
    Sh, // Stream Header
    Se, // Stream End
    Ap, // Audio Packet
    So, // Seek-table Offset
    St, // Seek Table
    Rg, // Replay-Gain
    Ei, // Encoder Information
    Ct, // Chapter Tag
    Other([u8; 2]),
}

impl ChunkTag {
    pub fn from_bytes(b: [u8; 2]) -> Self {
        match &b {
            b"SH" => ChunkTag::Sh,
            b"SE" => ChunkTag::Se,
            b"AP" => ChunkTag::Ap,
            b"SO" => ChunkTag::So,
            b"ST" => ChunkTag::St,
            b"RG" => ChunkTag::Rg,
            b"EI" => ChunkTag::Ei,
            b"CT" => ChunkTag::Ct,
            _ => ChunkTag::Other(b),
        }
    }

    pub fn as_str(&self) -> &str {
        match self {
            ChunkTag::Sh => "SH",
            ChunkTag::Se => "SE",
            ChunkTag::Ap => "AP",
            ChunkTag::So => "SO",
            ChunkTag::St => "ST",
            ChunkTag::Rg => "RG",
            ChunkTag::Ei => "EI",
            ChunkTag::Ct => "CT",
            ChunkTag::Other(_) => "??",
        }
    }
}

/// One SV8 chunk: tag + payload slice. The `total_size` includes the
/// tag and the var-len size field.
#[derive(Debug)]
pub struct Chunk<'a> {
    pub tag: ChunkTag,
    pub total_size: u64,
    pub header_bytes: usize,
    pub payload: &'a [u8],
}

/// Iterate SV8 chunks from a byte slice positioned **after** the
/// `MPCK` magic. Yields chunks one at a time until `SE` is reached or
/// the slice is exhausted. The iterator is forward-only and
/// position-tracking; the `SE` chunk is yielded as the last item.
pub struct ChunkIter<'a> {
    data: &'a [u8],
    cursor: usize,
    /// Set true once we have yielded `SE`; the next call returns
    /// `None`.
    finished: bool,
}

impl<'a> ChunkIter<'a> {
    pub fn new(data: &'a [u8]) -> Self {
        Self {
            data,
            cursor: 0,
            finished: false,
        }
    }

    pub fn position(&self) -> usize {
        self.cursor
    }
}

impl<'a> Iterator for ChunkIter<'a> {
    type Item = Result<Chunk<'a>>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.finished {
            return None;
        }
        if self.cursor + 2 > self.data.len() {
            return None;
        }
        let tag_bytes = [self.data[self.cursor], self.data[self.cursor + 1]];
        let tag = ChunkTag::from_bytes(tag_bytes);
        let after_tag = self.cursor + 2;
        let (size, size_bytes) = match read_byte_varlen(&self.data[after_tag..]) {
            Ok(v) => v,
            Err(e) => return Some(Err(e)),
        };
        // The chunk's `size` includes the tag (2) + size field bytes.
        let header_bytes = 2 + size_bytes;
        if size < header_bytes as u64 {
            return Some(Err(Error::invalid(format!(
                "musepack SV8 chunk '{}' size {size} < header bytes {header_bytes}",
                tag.as_str()
            ))));
        }
        let payload_len = (size - header_bytes as u64) as usize;
        let payload_start = after_tag + size_bytes;
        if payload_start + payload_len > self.data.len() {
            return Some(Err(Error::invalid(format!(
                "musepack SV8 chunk '{}' truncated (need {} bytes, have {})",
                tag.as_str(),
                payload_len,
                self.data.len().saturating_sub(payload_start)
            ))));
        }
        let payload = &self.data[payload_start..payload_start + payload_len];
        self.cursor = payload_start + payload_len;
        if matches!(tag, ChunkTag::Se) {
            self.finished = true;
        }
        Some(Ok(Chunk {
            tag,
            total_size: size,
            header_bytes,
            payload,
        }))
    }
}

/// SV8 Stream Header (`SH` chunk) decoded from a 12-byte payload.
#[derive(Clone, Debug)]
pub struct StreamHeaderSv8 {
    /// CRC-32 of the rest of the chunk payload (we do not verify it).
    pub crc32: u32,
    /// Stream version, must be `8`.
    pub version: u8,
    /// Total PCM samples (duration of the audio).
    pub total_samples: u64,
    /// Pre-skip silence samples at stream start (gapless padding).
    pub silence_samples: u64,
    /// Sample rate in Hz, mapped from a 3-bit index `[0..3]`.
    pub sample_rate: u32,
    /// Highest active subband (`maxbands_minus_one + 1`, `1..=32`).
    pub maxbands: u8,
    /// Channel count, `1..=2` supported.
    pub channels: u8,
    /// Mid-side stereo global enable.
    pub mid_side: bool,
    /// `block_power` — sub-frames per `AP` packet =
    /// `1 << (2 * block_power)`.
    pub block_power: u8,
}

/// Parse an SV8 `SH` chunk payload.
pub fn parse_sh(payload: &[u8]) -> Result<StreamHeaderSv8> {
    if payload.len() < 7 {
        return Err(Error::invalid(format!(
            "musepack SH: payload too short ({} bytes)",
            payload.len()
        )));
    }
    let crc32 = u32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]);
    let version = payload[4];
    if version != 8 {
        return Err(Error::invalid(format!(
            "musepack SH: unexpected stream version {version} (expected 8)"
        )));
    }
    let mut cursor = 5usize;
    let (total_samples, n) = read_byte_varlen(&payload[cursor..])?;
    cursor += n;
    let (silence_samples, n) = read_byte_varlen(&payload[cursor..])?;
    cursor += n;
    if cursor + 2 > payload.len() {
        return Err(Error::invalid("musepack SH: truncated extradata"));
    }
    let extradata = u16::from_be_bytes([payload[cursor], payload[cursor + 1]]);
    let sample_rate_idx = ((extradata >> 13) & 0x07) as usize;
    if sample_rate_idx >= SAMPLE_RATES.len() {
        return Err(Error::invalid(format!(
            "musepack SH: reserved sample-rate index {sample_rate_idx}"
        )));
    }
    let sample_rate = SAMPLE_RATES[sample_rate_idx];
    let maxbands_minus_one = ((extradata >> 8) & 0x1F) as u8;
    let maxbands = maxbands_minus_one + 1;
    if maxbands > 32 {
        return Err(Error::invalid(format!(
            "musepack SH: maxbands {maxbands} exceeds 32"
        )));
    }
    let channels_minus_one = ((extradata >> 4) & 0x0F) as u8;
    let channels = channels_minus_one + 1;
    if channels > 2 {
        return Err(Error::Unsupported(format!(
            "musepack SH: {channels}-channel streams not supported"
        )));
    }
    let mid_side = ((extradata >> 3) & 0x01) != 0;
    let block_power = (extradata & 0x07) as u8;
    Ok(StreamHeaderSv8 {
        crc32,
        version,
        total_samples,
        silence_samples,
        sample_rate,
        maxbands,
        channels,
        mid_side,
        block_power,
    })
}

// ---------- SV7 ----------

/// SV7 16-byte fixed header (after the 4-byte `'MP+' + sv-byte` magic
/// and 4-byte little-endian frame count). Bytes 8..23 are read as a
/// 128-bit packed bit-stream after a per-32-bit-word byte swap.
#[derive(Clone, Debug)]
pub struct StreamHeaderSv7 {
    /// Total frame count (header bytes 4..8 — 32-bit little-endian).
    pub total_frames: u32,
    /// Intensity-stereo enable.
    pub intensity_stereo: bool,
    /// Mid-side-stereo global enable.
    pub mid_side: bool,
    /// Highest active subband (`< 32`).
    pub maxbands: u8,
    /// Sample rate in Hz, derived from the SV7 sample-rate field.
    /// Note: SV7's sample-rate index lives elsewhere in the header
    /// (we conservatively default to 44 100 Hz when not yet wired in).
    pub sample_rate: u32,
    /// Channel count — SV7 is always stereo (2). The codec rejects
    /// mono files; mono encoder emits one band of L only.
    pub channels: u8,
    /// Gapless flag.
    pub gapless: bool,
    /// `last_frame_len` — sample count of the final frame (`0..2047`).
    pub last_frame_len: u16,
}

/// Parse the SV7 prefix `'MP+' + sv_byte + frame_count + 16 byte header`.
/// `data` must start at the file's first byte.
pub fn parse_sv7_header(data: &[u8]) -> Result<(StreamHeaderSv7, usize)> {
    if data.len() < 24 {
        return Err(Error::invalid("musepack SV7: file too short for header"));
    }
    if data[0..3] != MAGIC_SV7 {
        return Err(Error::invalid("musepack SV7: bad magic (expected 'MP+')"));
    }
    let sv_byte = data[3];
    if (sv_byte & 0x0F) != 0x07 {
        return Err(Error::invalid(format!(
            "musepack SV7: unsupported stream version nibble {:#x}",
            sv_byte & 0x0F
        )));
    }
    let total_frames = u32::from_le_bytes([data[4], data[5], data[6], data[7]]);
    // 16-byte header bytes 8..23 — read after per-32-bit-word swap.
    let mut hdr = [0u8; 16];
    for word in 0..4 {
        for b in 0..4 {
            hdr[word * 4 + b] = data[8 + word * 4 + (3 - b)];
        }
    }
    // Read MSB-first from `hdr`.
    let intensity_stereo = (hdr[0] & 0x80) != 0;
    let mid_side = (hdr[0] & 0x40) != 0;
    let maxbands = hdr[0] & 0x3F;
    if maxbands >= 32 {
        return Err(Error::invalid(format!(
            "musepack SV7: maxbands {maxbands} >= 32"
        )));
    }
    // Bits 96..107 = byte 12 .. byte 13 (12 bits). Bit 96 = MSB of
    // byte 12 = `gapless`; bits 97..107 = `last_frame_len` (11 bits).
    let gapless = (hdr[12] & 0x80) != 0;
    let last_frame_len = ((u16::from(hdr[12] & 0x7F) << 4) | u16::from(hdr[13] >> 4)) & 0x07FF;

    Ok((
        StreamHeaderSv7 {
            total_frames,
            intensity_stereo,
            mid_side,
            maxbands,
            sample_rate: 44_100, // TODO: SV7 sample-rate field is in the skipped 88 bits of the header
            channels: 2,
            gapless,
            last_frame_len,
        },
        24,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a synthetic minimal SV8 stream with no audio packets and
    /// verify the chunk iterator walks SH → SE cleanly.
    #[test]
    fn sv8_chunk_iter_minimal() {
        // 12-byte SH payload: zero CRC, version 8, total_samples = 0
        // (single byte 0x01 → raw 1 - 1 byte = 0), silence = 0,
        // extradata = 0x0000.
        let sh_payload: [u8; 12] = [
            0x00, 0x00, 0x00, 0x00, // CRC
            0x08, // version
            0x00, // total_samples var-len = 0
            0x00, // silence_samples var-len = 0
            0x00, 0x00, // extradata
            0x00, 0x00, 0x00, // padding
        ];
        // SH chunk: tag 'SH' + var-len size 15 (single byte 0x0F).
        // 15 = total_size = tag (2) + size_field (1) + payload (12).
        let mut sh_chunk = vec![b'S', b'H', 0x0F];
        sh_chunk.extend_from_slice(&sh_payload);
        // SE chunk: tag 'SE' + var-len size 3 (single byte 0x03).
        let se_chunk: Vec<u8> = vec![b'S', b'E', 0x03];

        let mut bytes = Vec::new();
        bytes.extend_from_slice(&sh_chunk);
        bytes.extend_from_slice(&se_chunk);

        let mut iter = ChunkIter::new(&bytes);
        let sh = iter.next().unwrap().unwrap();
        assert_eq!(sh.tag, ChunkTag::Sh);
        assert_eq!(sh.total_size, 15);
        assert_eq!(sh.payload.len(), 12);
        let se = iter.next().unwrap().unwrap();
        assert_eq!(se.tag, ChunkTag::Se);
        assert_eq!(se.total_size, 3);
        assert_eq!(se.payload.len(), 0);
        assert!(iter.next().is_none());
    }

    #[test]
    fn sv8_sh_extradata_parse() {
        // Build an SH payload with sample_rate_idx=0 (44.1 kHz),
        // maxbands_minus_one=27 → maxbands=28, channels_minus_one=1
        // → channels=2, mid_side=1, block_power=3.
        let extradata: u16 = ((27u16 << 8)) | (1u16 << 4) | (1u16 << 3) | 3u16;
        let mut payload = vec![0u8; 4]; // CRC
        payload.push(0x08); // version
        payload.push(0x00); // total_samples = 0
        payload.push(0x00); // silence = 0
        payload.extend_from_slice(&extradata.to_be_bytes());
        let sh = parse_sh(&payload).unwrap();
        assert_eq!(sh.sample_rate, 44_100);
        assert_eq!(sh.maxbands, 28);
        assert_eq!(sh.channels, 2);
        assert!(sh.mid_side);
        assert_eq!(sh.block_power, 3);
    }
}
