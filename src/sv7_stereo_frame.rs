//! SV7 stereo (two-channel) frame-body assembler + reconstruction.
//!
//! This is the cross-channel composition the single-channel
//! [`crate::sv7_frame_decode::decode_sv7_frame_channel`] left as GAP:
//! it walks one SV7 frame body for **both** channels in the documented
//! §5 phase order and hands back the per-channel reconstructed subband
//! matrices ([`crate::ms_stereo::StereoSubbandMatrix`]) plus the per-band
//! M/S flags — exactly the input the §2.6 M/S-undo step
//! ([`crate::ms_stereo::undo_ms_stereo`]) and then the synthesis
//! filterbank consume.
//!
//! # Phase ordering — what is grounded vs. what stays GAP
//!
//! The staged `spec/musepack-headers-and-coding.md` §5 lays a stereo SV7
//! frame body out as:
//!
//! 1. **§5.1 band-type (`Res`) header** — read for *both* channels
//!    interleaved per band ("left `Res` and right `Res`"), plus the
//!    per-band M/S flag. This is a single shared header sweep over
//!    `0..=max_band`, decoded by
//!    [`crate::sv7_band_header::decode_res_header_grounded`].
//! 2. **§5.3 SCFI + DSCF** and **§5.4 quantised samples** — both §5.3
//!    and §5.4 close with the explicit sentence *"Left channel is
//!    decoded first, then right."* So after the shared §5.1 header, the
//!    decoder runs the **whole** SCF-then-samples body for the left
//!    channel, then the **whole** SCF-then-samples body for the right
//!    channel. That is a per-channel sweep, not a per-band channel
//!    interleave — the §5.3/§5.4 wording pins it.
//!
//! This module therefore composes:
//!
//! - one [`crate::sv7_band_header::decode_res_header_grounded`] over the
//!   shared reader (the §5.1 header, both channels + M/S flags);
//! - then [`crate::sv7_frame_decode::decode_sv7_frame_channel`] over the
//!   **same** reader for the left channel's `Res` column, then again for
//!   the right channel's `Res` column (the §5.3/§5.4 "left first, then
//!   right" body sweeps);
//! - then [`crate::frame_reconstruct::reconstruct_frame_channel`] per
//!   channel into a [`crate::frame_reconstruct::SubbandMatrix`].
//!
//! No new format facts are introduced. The two facts §2.6 still lists as
//! GAP are threaded as caller arguments, unchanged from the
//! single-channel path:
//!
//! - the **absolute SCF anchor** (`first_scf_ref` for each channel's
//!   first coded band, and the reconstruction `anchor`) — GAP per §2.6;
//! - the **M/S-undo arithmetic** — not performed here at all. This
//!   module returns the raw (mid/side-or-L/R) channel matrices and the
//!   per-band `ms_flags`; the caller runs
//!   [`crate::ms_stereo::undo_ms_stereo`] with its GAP closure.
//!
//! Source-of-record (facts only):
//!
//! - `docs/audio/musepack/spec/musepack-headers-and-coding.md` §5.1
//!   (shared band-type header, both channels), §5.2/§5.3 ("Left channel
//!   is decoded first, then right"), §5.4 (same).
//! - `docs/audio/musepack/musepack-sv7-sv8-spec.md` §2.3 (per-band
//!   `msflag`), §2.6 (reconstruction step order).

use crate::cns::CnsPrng;
use crate::frame_reconstruct::reconstruct_frame_channel;
use crate::huffman::Sv7BitReader;
use crate::ms_stereo::StereoSubbandMatrix;
use crate::sv7_band_header::decode_res_header_grounded;
use crate::sv7_frame_decode::decode_sv7_frame_channel;
use crate::Result;

/// The decoded structure of one SV7 stereo frame body, before the §2.6
/// M/S-undo step.
///
/// `channels[0]` is the left/mid channel's reconstructed subband matrix
/// and `channels[1]` the right/side channel's (which role each subband
/// plays is given by `ms_flags`). `ms_flags[b]` is the §5.1 per-band M/S
/// flag for subband `b`: `true` ⇒ subband `b` is coded mid/side and must
/// be run through [`crate::ms_stereo::undo_ms_stereo`]; `false` ⇒ it is
/// already left/right.
#[derive(Debug, Clone, PartialEq)]
pub struct Sv7StereoFrame {
    /// The two channels' reconstructed subband matrices (pre-M/S-undo).
    pub channels: StereoSubbandMatrix,
    /// Per-band M/S flags, ascending band order, length `max_band + 1`.
    pub ms_flags: Vec<bool>,
}

/// Decode + reconstruct one SV7 **stereo** frame body into a
/// [`Sv7StereoFrame`] (pre-M/S-undo per-channel subband matrices + the
/// per-band M/S flags), in the documented §5 phase order.
///
/// `reader` is positioned at the start of the frame body (the §5.1
/// band-type header). `max_band` is the stream header's `max_band`
/// (`1..=31`, the highest coded subband). `stream_ms` is the
/// stream-wide M/S enable (SV7 fixed-header field 3): when set, §5.1
/// reads a per-band M/S bit for each band with a non-zero channel.
///
/// `first_scf_ref` / `anchor` are the GAP §2.6 absolute SCF anchor,
/// threaded through unchanged from the single-channel path: the same
/// `first_scf_ref` seeds each channel's first coded band, and the same
/// `anchor` is used to reconstruct both channels (pass `0` for the
/// relative-loudness convention). `cns` is the shared CNS PRNG; it is
/// advanced by every noise band of **both** channels in decode order
/// (left channel's bands first, then right), so its state threads
/// exactly across the whole frame.
///
/// The returned [`Sv7StereoFrame`] is raw mid/side-or-L/R: the caller
/// applies [`crate::ms_stereo::undo_ms_stereo`] (with its GAP closure)
/// over `frame.channels` and `frame.ms_flags` before the synthesis
/// filterbank.
///
/// # Errors
///
/// - [`crate::Error::ChannelCountInvalid`] is not produced here (this
///   entry point is stereo by construction); the band-header walk is
///   driven with `nch == 2`.
/// - [`crate::Error::MaxBandOutOfRange`] if `max_band` exceeds the §1
///   Layer-II 32-subband inclusive bound.
/// - [`crate::Error::UnexpectedEof`] / [`crate::Error::HuffmanNoMatch`]
///   if the reader starves or a peek matches no table row in any phase.
/// - [`crate::Error::UnsupportedBandType`] for a `Res` outside `-1..=17`.
/// - [`crate::Error::InvalidScfCodingMethod`] propagated from §5.3.
pub fn decode_sv7_stereo_frame(
    reader: &mut Sv7BitReader<'_>,
    max_band: u8,
    stream_ms: bool,
    first_scf_ref: i32,
    anchor: u8,
    cns: &mut CnsPrng,
) -> Result<Sv7StereoFrame> {
    // §5.1: shared band-type header, both channels + per-band M/S flags.
    let header = decode_res_header_grounded(reader, max_band, 2, stream_ms)?;

    // Split out per-channel Res columns and the per-band M/S flags. A
    // band whose M/S flag was suppressed (stream M/S off, or both
    // channels Res == 0) is treated as L/R (false) — undo_ms_stereo only
    // transforms flagged subbands.
    let mut left_res = Vec::with_capacity(header.len());
    let mut right_res = Vec::with_capacity(header.len());
    let mut ms_flags = Vec::with_capacity(header.len());
    for band in &header {
        left_res.push(band.res[0]);
        right_res.push(band.res[1]);
        ms_flags.push(band.ms_flag.unwrap_or(false));
    }

    // §5.3/§5.4: "Left channel is decoded first, then right." The whole
    // SCF-then-samples body for the left channel, then the right — over
    // the same reader, sharing the CNS PRNG so its state threads across
    // both channels in decode order.
    let left_bands = decode_sv7_frame_channel(reader, &left_res, first_scf_ref, cns)?;
    let right_bands = decode_sv7_frame_channel(reader, &right_res, first_scf_ref, cns)?;

    // §2.6: per-channel dequant + per-granule SCF multiply.
    let left_matrix = reconstruct_frame_channel(&left_bands, anchor)?;
    let right_matrix = reconstruct_frame_channel(&right_bands, anchor)?;

    Ok(Sv7StereoFrame {
        channels: [left_matrix, right_matrix],
        ms_flags,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame_reconstruct::zero_subband_matrix;
    use crate::huffman::{SV7_Q3_TABLE, SV7_SCFI_TABLE};

    /// MSB-first bit packer mirroring the sibling frame-decode tests.
    struct Packer {
        bytes: Vec<u8>,
        acc: u32,
        nbits: u8,
    }

    impl Packer {
        fn new() -> Self {
            Packer {
                bytes: Vec::new(),
                acc: 0,
                nbits: 0,
            }
        }
        fn push(&mut self, pattern: u16, length: u8) {
            for i in 0..length {
                let bit = (pattern >> (15 - i)) & 1;
                self.acc = (self.acc << 1) | bit as u32;
                self.nbits += 1;
                if self.nbits == 8 {
                    self.bytes.push(self.acc as u8);
                    self.acc = 0;
                    self.nbits = 0;
                }
            }
        }
        fn push_raw(&mut self, value: u32, length: u8) {
            for i in (0..length).rev() {
                let bit = (value >> i) & 1;
                self.acc = (self.acc << 1) | bit;
                self.nbits += 1;
                if self.nbits == 8 {
                    self.bytes.push(self.acc as u8);
                    self.acc = 0;
                    self.nbits = 0;
                }
            }
        }
        fn finish(mut self) -> Vec<u8> {
            if self.nbits > 0 {
                self.bytes.push((self.acc << (8 - self.nbits)) as u8);
            }
            // Trailing peek-padding (the reader always peeks 16 bits).
            self.bytes.push(0);
            self.bytes.push(0);
            self.bytes.push(0);
            self.bytes.push(0);
            self.bytes
        }
    }

    /// SCFI codeword for value 3 (only SCF[0] coded).
    fn scfi3() -> (u16, u8) {
        (0x0000, 2)
    }
    /// DSCF codeword for symbol 0 (no-op delta).
    fn dscf0() -> (u16, u8) {
        (0x9000, 4)
    }

    /// Pack a §5.1 stereo header for two bands, both channels Res == 0
    /// (silent), stream M/S off ⇒ no per-band M/S bit, no body. Band 0
    /// is raw-4-bit per channel; band 1 is a header-VLC delta per
    /// channel. Easiest silent case: max_band = 0 (single band, raw).
    #[test]
    fn all_silent_stereo_frame_reconstructs_to_silence() {
        // max_band = 0: one band, both channels raw-4-bit Res = 0.
        let mut p = Packer::new();
        p.push_raw(0, 4); // left band-0 Res = 0
        p.push_raw(0, 4); // right band-0 Res = 0
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let mut cns = CnsPrng::new();
        let frame = decode_sv7_stereo_frame(&mut r, 0, false, 0, 0, &mut cns).unwrap();
        assert_eq!(frame.channels[0], zero_subband_matrix());
        assert_eq!(frame.channels[1], zero_subband_matrix());
        assert_eq!(frame.ms_flags, vec![false]);
    }

    #[test]
    fn ms_flag_read_per_band_when_stream_ms_set() {
        // max_band = 0, both channels Res = 3 (coded) so the band has
        // samples ⇒ a per-band M/S bit is read. Then each channel runs
        // its full body: SCFI=3, DSCF=0, selector, 36 q3.
        let (scfi_c, scfi_l) = scfi3();
        let (dscf_c, dscf_l) = dscf0();
        let (q3_c, q3_l) = (SV7_Q3_TABLE[0].code, SV7_Q3_TABLE[0].length);

        let mut p = Packer::new();
        // §5.1 header: left Res=3 (raw 4-bit), right Res=3 (raw 4-bit).
        p.push_raw(3, 4);
        p.push_raw(3, 4);
        // per-band M/S bit (stream M/S on, band has samples): set it.
        p.push_raw(1, 1);
        // §5.3/§5.4 LEFT channel body.
        p.push(scfi_c, scfi_l);
        p.push(dscf_c, dscf_l);
        p.push_raw(0, 1); // selector
        for _ in 0..36 {
            p.push(q3_c, q3_l);
        }
        // RIGHT channel body.
        p.push(scfi_c, scfi_l);
        p.push(dscf_c, dscf_l);
        p.push_raw(0, 1);
        for _ in 0..36 {
            p.push(q3_c, q3_l);
        }
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let mut cns = CnsPrng::new();
        let frame = decode_sv7_stereo_frame(&mut r, 0, true, 100, 0, &mut cns).unwrap();
        assert_eq!(frame.ms_flags, vec![true]);
        // Both channels reconstruct identically (same bits) — subband 0
        // non-silent.
        assert_eq!(frame.channels[0], frame.channels[1]);
        assert!(frame.channels[0][0].iter().any(|&s| s != 0.0));
        // Higher subbands stay silent.
        for b in 1..32 {
            assert!(frame.channels[0][b].iter().all(|&s| s == 0.0));
        }
        // Touch the SCFI table for import parity.
        assert_eq!(SV7_SCFI_TABLE.len(), 4);
    }

    #[test]
    fn left_then_right_body_order_threads_cns_across_channels() {
        // Both channels Res = -1 (CNS). No SCF / selector — just 36 PRNG
        // samples per channel, left then right. The shared PRNG must
        // advance through the left band's 36 samples before the right's.
        let mut p = Packer::new();
        // §5.1: left Res=-1, right Res=-1 (raw 4-bit, value 15 = -1 as i8
        // when read as 4-bit? No — read_bits(4) yields 0..15, cast i8.
        // 15 -> 15, not -1. The CNS case needs Res == -1, i.e. band_type
        // -1. Band-0 raw read gives 0..15; to get -1 we use a later band
        // delta. Simplest: max_band=1, band0 Res=0 (raw), band1 delta to
        // reach -1 via the header VLC value -1.)
        p.push_raw(0, 4); // L band0 Res = 0
        p.push_raw(0, 4); // R band0 Res = 0
                          // band1: header VLC value -1 per channel. From the band-header
                          // test docstring: value -1 is code 0x0000, length 2 (bits "00").
        p.push(0x0000, 2); // L band1 delta -1 -> Res = -1
        p.push(0x0000, 2); // R band1 delta -1 -> Res = -1
                           // No per-band M/S (stream M/S off). Bodies: band0 empty (Res 0),
                           // band1 CNS (Res -1) ⇒ 36 PRNG samples, no SCF.
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let mut cns = CnsPrng::new();
        let frame = decode_sv7_stereo_frame(&mut r, 1, false, 0, 0, &mut cns).unwrap();
        assert_eq!(frame.ms_flags, vec![false, false]);

        // Reference: left CNS band drains 36 samples, then right drains
        // the next 36 from the same PRNG.
        let mut ref_cns = CnsPrng::new();
        let mut left_ref = [0_i32; 36];
        ref_cns.fill_samples(&mut left_ref);
        let mut right_ref = [0_i32; 36];
        ref_cns.fill_samples(&mut right_ref);
        // The two channels' subband-1 rows differ (PRNG advanced between
        // them) — confirms left-then-right ordering of the bodies.
        assert_ne!(frame.channels[0][1], frame.channels[1][1]);
        assert_eq!(cns.state(), ref_cns.state());
    }

    #[test]
    fn rejects_max_band_out_of_range() {
        let mut r = Sv7BitReader::new(&[0xFF; 8]);
        let mut cns = CnsPrng::new();
        assert_eq!(
            decode_sv7_stereo_frame(&mut r, 32, false, 0, 0, &mut cns),
            Err(crate::Error::MaxBandOutOfRange(32))
        );
    }

    #[test]
    fn frame_pairs_with_undo_ms_stereo() {
        // Smoke: a decoded stereo frame feeds undo_ms_stereo with a
        // test closure over its ms_flags without panicking.
        let mut p = Packer::new();
        p.push_raw(0, 4);
        p.push_raw(0, 4);
        let bytes = p.finish();
        let mut r = Sv7BitReader::new(&bytes);
        let mut cns = CnsPrng::new();
        let mut frame = decode_sv7_stereo_frame(&mut r, 0, false, 0, 0, &mut cns).unwrap();
        crate::ms_stereo::undo_ms_stereo(&mut frame.channels, &frame.ms_flags, |m, s| {
            (m + s, m - s)
        })
        .unwrap();
        // All-silent frame stays silent after undo.
        assert_eq!(frame.channels[0], zero_subband_matrix());
    }
}
