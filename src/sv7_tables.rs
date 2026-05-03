//! SV7 Huffman tables — `(symbol, length)` inline form.
//!
//! Transcribed from `docs/audio/musepack/data/musepack-vlc-tables.md`
//! §1. Build the runtime [`VlcTable`]s lazily from these via
//! `VlcTable::from_sv7`.
//!
//! SV7 is currently **sketch-stage** in this crate — the demuxer reads
//! the `MP+` header but the per-frame entropy path is not yet wired in.
//! The tables below are exposed so the wiring can land in a follow-up
//! round.
//!
//! [`VlcTable`]: crate::vlc::VlcTable

/// SCFI VLC — 4-entry, max 3 bits. Decodes the 2-bit
/// "scalefactor coding mode" (see trace report §4.1.3).
pub const SCFI_VLC: [(i32, u8); 4] = [
    (1, 1), // all three scalefactors equal
    (3, 2), // third == second; first independent
    (0, 3), // all three independent
    (2, 3), // second == first; third independent
];

/// DSCF VLC — 16-entry, max 6 bits, post-bias `-7`. Symbol `15` (after
/// bias = `+8`) is the **escape** that triggers a raw 6-bit absolute
/// scalefactor re-read. Other symbols are signed deltas in `[-7..+7]`.
pub const DSCF_VLC: [(i32, u8); 16] = [
    (9, 3),  // +2
    (6, 3),  // -1
    (8, 3),  // +1
    (11, 4), // +4
    (7, 4),  // 0
    (15, 4), // +8 (escape)
    (4, 4),  // -3
    (10, 4), // +3
    (1, 5),  // -6
    (13, 5), // +6
    (2, 5),  // -5
    (3, 5),  // -4
    (12, 5), // +5
    (5, 3),  // -2
    (0, 6),  // -7
    (14, 6), // +7
];

/// HDR VLC — 10-entry, max 9 bits, post-bias `-5`. Symbol value `9`
/// (post-bias `+4`) is the **escape** that triggers a raw 4-bit
/// re-read.
pub const HDR_VLC: [(i32, u8); 10] = [
    (5, 1), // 0
    (6, 3), // +1
    (4, 2), // -1
    (3, 4), // -2
    (2, 5), // -3
    (7, 6), // +2
    (1, 7), // -4
    (0, 8), // -5
    (9, 9), // +4 (escape)
    (8, 9), // +3
];

// Quantizer VLCs (sets 0..=6, two parallel sub-tables each — 14 VLCs
// total) are large transcriptions; deferred to the SV7 wire-up round.
// The data lives in §1.4 of the sidecar (each set is sketched in this
// module's doc-comment for now).
