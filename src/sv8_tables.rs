//! SV8 Huffman tables — length counts + symbol arrays.
//!
//! All entries transcribed from
//! `docs/audio/musepack/data/musepack-vlc-tables.md` §2. The tables
//! are stored in JPEG-style canonical-Huffman form: a 16-element
//! `length_counts` histogram and a separate symbol-order array.
//!
//! This module exposes the raw arrays; build the runtime [`VlcTable`]s
//! lazily from them in `sv8::Decoder` once at codec init.
//!
//! [`VlcTable`]: crate::vlc::VlcTable

/// Bands VLC: signed delta against the previous sub-frame's `maxband`,
/// modulo-33 wraparound. 33 entries.
pub const BAND_VLC_LENGTHS: [u8; 16] = [1, 1, 1, 0, 2, 2, 1, 3, 2, 3, 4, 11, 2, 0, 0, 0];
pub const BAND_VLC_SYMBOLS: [i32; 33] = [
    13, 19, 10, 11, 12, 14, 15, 16, 17, 18, 20, 21, 22, 9, 23, 24, 25, 8, 26, 27, 7, 28, 5, 6, 29,
    4, 3, 30, 2, 31, 1, 32, 0,
];

/// SCFI VLCs (two parallel sub-tables — `[mono, stereo]`).
pub const SCFI0_VLC_LENGTHS: [u8; 16] = [1, 1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
pub const SCFI0_VLC_SYMBOLS: [i32; 4] = [0, 1, 3, 2];

pub const SCFI1_VLC_LENGTHS: [u8; 16] = [0, 2, 2, 0, 5, 5, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0];
pub const SCFI1_VLC_SYMBOLS: [i32; 16] = [1, 4, 0, 2, 3, 8, 12, 5, 6, 7, 9, 13, 11, 14, 10, 15];

/// DSCF VLCs (two parallel sub-tables). `[0]` is for the second/third
/// scalefactors of a band-channel (escape symbol = 31, raw 6-bit
/// extension); `[1]` is for the first scalefactor of a band-channel
/// (escape symbol = 64, raw 6-bit extension).
pub const DSCF0_VLC_LENGTHS: [u8; 16] = [0, 0, 3, 6, 3, 4, 5, 7, 7, 9, 6, 5, 3, 6, 0, 0];
pub const DSCF0_VLC_SYMBOLS: [i32; 64] = [
    58, 59, 60, 61, 62, 63, 55, 56, 57, 0, 1, 2, 53, 54, 3, 4, 5, 50, 51, 52, 6, 7, 8, 9, 10, 31,
    47, 48, 49, 11, 12, 13, 14, 44, 45, 46, 15, 16, 17, 18, 41, 42, 43, 19, 20, 21, 22, 40, 23, 24,
    38, 39, 25, 28, 37, 26, 27, 29, 30, 32, 36, 33, 34, 35,
];

pub const DSCF1_VLC_LENGTHS: [u8; 16] = [0, 0, 5, 3, 3, 2, 3, 4, 5, 7, 7, 9, 6, 5, 6, 0];
pub const DSCF1_VLC_SYMBOLS: [i32; 65] = [
    0, 59, 60, 61, 62, 63, 1, 2, 56, 57, 58, 3, 4, 5, 53, 54, 55, 6, 7, 8, 9, 49, 50, 51, 52, 64,
    10, 11, 12, 13, 46, 47, 48, 14, 15, 16, 17, 43, 44, 45, 18, 19, 20, 41, 42, 21, 22, 39, 40, 23,
    24, 38, 25, 37, 26, 35, 36, 27, 28, 34, 29, 30, 31, 32, 33,
];

/// RES VLCs (two parallel sub-tables). 17 entries each. After decode
/// the running absolute `res` may be > 15, in which case the decoder
/// subtracts 17 to wrap into `[-1..14]`.
pub const RES0_VLC_LENGTHS: [u8; 16] = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2];
pub const RES0_VLC_SYMBOLS: [i32; 17] = [13, 14, 12, 11, 10, 9, 8, 7, 6, 15, 5, 4, 3, 2, 16, 1, 0];

pub const RES1_VLC_LENGTHS: [u8; 16] = [0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 4, 0, 0];
pub const RES1_VLC_SYMBOLS: [i32; 17] = [8, 9, 10, 11, 7, 12, 6, 13, 5, 4, 14, 3, 15, 2, 0, 1, 16];

/// Q1 VLC — used at `res = 1` only. Reads a popcount over 18 sample
/// positions (half-band binary spike map). 19 entries.
pub const Q1_VLC_LENGTHS: [u8; 16] = [0, 0, 5, 5, 1, 1, 1, 1, 1, 1, 1, 2, 0, 0, 0, 0];
pub const Q1_VLC_SYMBOLS: [i32; 19] = [
    17, 18, 16, 15, 14, 13, 12, 0, 11, 1, 2, 8, 9, 10, 3, 4, 5, 6, 7,
];

/// Q9UP VLC — used at `res ∈ {9..17}`. The 9-bit code provides sign
/// + 8 base bits; for `res > 9` the decoder appends `res - 9` raw
/// bits, then biases by `(1 << (res - 2)) - 1`. 256 entries.
pub const Q9UP_VLC_LENGTHS: [u8; 16] = [0, 0, 0, 0, 0, 2, 38, 134, 71, 9, 2, 0, 0, 0, 0, 0];
pub const Q9UP_VLC_SYMBOLS: [i32; 256] = [
    254, 255, 0, 1, 2, 3, 4, 250, 251, 252, 253, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18,
    21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 41, 213, 214, 215,
    216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234,
    235, 236, 237, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 19, 20, 40, 42, 43,
    44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67,
    68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91,
    92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 147, 149, 150, 151,
    152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170,
    171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188, 189,
    190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208,
    209, 210, 211, 212, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122,
    123, 124, 125, 126, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143,
    144, 145, 146, 148, 127, 128,
];

/// Q2 VLCs (two sub-tables for `res = 2`). The 125-entry symbol array
/// is shared between sub-tables; only the length-count histograms
/// differ.
pub const Q2_VLC0_LENGTHS: [u8; 16] = [0, 0, 1, 6, 0, 17, 9, 24, 24, 9, 27, 4, 4, 0, 0, 0];
pub const Q2_VLC1_LENGTHS: [u8; 16] = [0, 0, 0, 1, 16, 10, 6, 48, 9, 27, 4, 4, 0, 0, 0, 0];

/// Length-count histograms for Q3 (`res = 3`, 49 entries) and Q4
/// (`res = 4`, 81 entries) — the shared sub-table approach folded
/// into a single VLC each (offsets `-48` for Q3, `-64` for Q4).
pub const Q3_VLC_LENGTHS: [u8; 16] = [0, 0, 1, 6, 6, 11, 13, 8, 4, 0, 0, 0, 0, 0, 0, 0];
pub const Q4_VLC_LENGTHS: [u8; 16] = [0, 0, 0, 1, 12, 23, 14, 19, 8, 4, 0, 0, 0, 0, 0, 0];

// Note: full 125-entry, 49-entry, 81-entry symbol arrays for Q2/Q3/Q4
// and 15/31/63/127-entry arrays for Q5..Q8 are large transcriptions
// from `mpc8huff.h::mpc8_q_syms`. The sidecar
// `data/musepack-vlc-tables.md` §2.7..§2.9 only sketches lengths +
// offsets; the full symbol arrays are deferred to a follow-up round
// (the decoder paths that consume them — `res = 2..=8` — error out
// with `Error::Unsupported` until the symbol arrays land). The
// `q1_vlc`, `q9up_vlc`, `band_vlc`, `res_vlc`, `dscf_vlc`, and
// `scfi_vlc` paths above are fully populated and decode correctly.

/// Identity-permutation symbol array for the ranges where the SV8
/// canonical encoding lists symbols `0, 1, ..., n-1` in order — used
/// as a sentinel by the `res = 2..=8` VLC builders that haven't been
/// transcribed yet, so the decoder can still build a syntactically
/// valid `VlcTable` for length-count assertions while the real
/// symbols are absent.
pub fn identity_symbols(n: usize) -> Vec<i32> {
    (0..n as i32).collect()
}
