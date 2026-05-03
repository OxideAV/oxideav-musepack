//! Pure-Rust Musepack (SV7 + SV8) audio decoder.
//!
//! Two stream formats are supported under the same crate name:
//!
//! * **SV8** (`MPCK` chunked container) — fully implemented for the
//!   structural decode path: container parser, `SH`/`SE`/`AP` walk,
//!   bit-stream entropy decode for `res ∈ {-1, 0, 1, 9..=17}` (the
//!   noise / silent / spike / high-precision branches). The
//!   mid-precision quantiser symbol arrays for `res ∈ {2..=8}` are
//!   currently substituted with raw-bit fallbacks pending
//!   transcription of `mpc8huff.h::mpc8_q_syms` from the workspace's
//!   in-tree reverse-engineering writeup.
//!
//! * **SV7** (`MP+` fixed-prefix container) — header parser only;
//!   the entropy-decode path is a sketch awaiting the SV7 quantiser
//!   VLC transcription (see `sv7_tables.rs`).
//!
//! Reference materials, used as the **only** sources for the
//! transcribed wire-format constants:
//!
//! * `docs/audio/musepack/musepack-trace-reverse-engineering.md`
//! * `docs/audio/musepack/data/musepack-vlc-tables.md`
//!
//! No third-party source (libavcodec, libmpcdec, etc.) was consulted.
//! The 32-band PQF synthesis filter is the standard MPEG-1 Audio
//! Annex B prototype — its 257-tap upper half lives in
//! `synth::ENWINDOW`, identical to the `dist10` public-domain table
//! and to ISO/IEC 11172-3 Table 3-B.3.

#![allow(
    clippy::needless_range_loop,
    clippy::manual_range_contains,
    clippy::doc_lazy_continuation
)]

pub mod cns;
pub mod container;
pub mod decoder;
pub mod sv7_tables;
pub mod sv8;
pub mod sv8_tables;
pub mod synth;
pub mod tables;
pub mod varlen;
pub mod vlc;

use oxideav_core::registry::CodecRegistry;
use oxideav_core::{CodecCapabilities, CodecId, CodecInfo, CodecParameters, Decoder, Result};

/// Canonical codec ID — accepts both SV7 and SV8 streams via the same
/// decoder shim (the magic bytes drive variant selection).
pub const CODEC_ID_STR: &str = "musepack";

/// Register the Musepack decoder with a [`CodecRegistry`]. Several ID
/// aliases are registered: `musepack` (the canonical name),
/// `musepack7` / `musepack8` (matching FFmpeg's per-version
/// convention so that containers carrying the SV-specific tag find
/// us), and `mpc` (the file-extension shorthand).
pub fn register(reg: &mut CodecRegistry) {
    let caps = CodecCapabilities::audio("musepack_sw_dec")
        .with_lossy(true)
        .with_intra_only(false)
        .with_max_channels(2)
        .with_max_sample_rate(48_000);
    for id in ["musepack", "musepack7", "musepack8", "mpc"] {
        let cid = CodecId::new(id);
        reg.register(
            CodecInfo::new(cid)
                .capabilities(caps.clone())
                .decoder(make_decoder),
        );
    }
}

fn make_decoder(params: &CodecParameters) -> Result<Box<dyn Decoder>> {
    decoder::make_decoder(params)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn registers_canonical_id() {
        let mut reg = CodecRegistry::new();
        register(&mut reg);
        assert!(reg.has_decoder(&CodecId::new("musepack")));
        assert!(reg.has_decoder(&CodecId::new("musepack8")));
        assert!(reg.has_decoder(&CodecId::new("mpc")));
    }
}
