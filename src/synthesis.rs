//! §2.6 final step — the 32-band polyphase **synthesis subband
//! filter**, the last stage that turns a frame's reconstructed
//! [`crate::frame_reconstruct::SubbandMatrix`] into PCM samples.
//!
//! Musepack (SV7 and SV8 alike) inherits this filterbank unchanged from
//! MPEG-1 Audio Layer I/II
//! (`docs/audio/musepack/musepack-sv7-sv8-spec.md` §1 lines 55-66:
//! "Musepack reuses this filterbank, so a Musepack decoder needs the
//! *same* `D_i` window and `N_ik` matrix"). The spec places the exact
//! algorithm and the window coefficients in the in-repo ISO/IEC
//! 11172-3:1993 PDF staged under `docs/audio/mp3/` — a standards-body
//! document whose transcription is policy-clean (spec source S3).
//!
//! # The algorithm (ISO 11172-3 Figure 3-A.2, verbatim)
//!
//! Each call to [`SynthesisFilter::synthesize`] consumes one set of 32
//! subband samples `S[0..31]` (one for each of the 32 subbands at one
//! time slot) and produces 32 consecutive PCM samples, running the five
//! steps the ISO Figure 3-A.2 "Synthesis subband filter flow chart"
//! lays out (transcribed from `docs/audio/mp3/ISO_IEC_11172-3-MP3-1993.pdf`,
//! Annex A, Figure A.2):
//!
//! 1. **Shifting** — `for i = 1023 down to 64: V[i] = V[i-64]`.
//!    (`V` is a 1024-entry FIFO, zero-initialised at startup per the
//!    figure's footnote 1.)
//! 2. **Matrixing** — `for i = 0 to 63: V[i] = sum_{k=0}^{31} N_ik · S_k`,
//!    where `N_ik = cos[(16+i)·(2k+1)·π / 64]`.
//! 3. **Build a 512-value vector U** — `for i = 0 to 7, for j = 0 to 31:
//!    U[i·64 + j] = V[i·128 + j]; U[i·64 + 32 + j] = V[i·128 + 96 + j]`.
//! 4. **Window** — `for i = 0 to 511: W[i] = U[i] · D[i]`, with `D`
//!    the 512-tap synthesis window of ISO Table 3-B.3 (transcribed in
//!    [`SYNTHESIS_WINDOW`]).
//! 5. **Calculate 32 samples** — `for j = 0 to 31: out_j =
//!    sum_{i=0}^{15} W[j + 32·i]`.
//!
//! # Frame driver
//!
//! A [`crate::frame_reconstruct::SubbandMatrix`] is laid out as
//! `[[f64; 36]; 32]`: row `b` is subband `b`'s 36 time-ordered samples.
//! The filterbank instead consumes the matrix **column by column**: at
//! each of the 36 time slots it takes one sample from every one of the
//! 32 subbands (the `S[0..31]` vector for that slot) and emits 32 PCM
//! samples, for `36 × 32 = 1152` PCM samples per channel
//! ([`crate::SAMPLES_PER_FRAME_PER_CHANNEL`]).
//! [`synthesize_frame_channel`] drives exactly that, returning the
//! `1152` PCM samples in output order.
//!
//! # Source-of-record (facts only)
//!
//! - `docs/audio/musepack/musepack-sv7-sv8-spec.md` §1 — the inherited
//!   Layer-II filterbank; the 32×36 = 1152 frame geometry.
//! - `docs/audio/mp3/ISO_IEC_11172-3-MP3-1993.pdf` Annex A Figure A.2
//!   ("Synthesis subband filter flow chart") — the five-step algorithm
//!   and the `N_ik = cos[(16+i)(2k+1)π/64]` matrixing formula.
//! - `docs/audio/mp3/ISO_IEC_11172-3-MP3-1993.pdf` Annex B Table 3-B.3
//!   ("Coefficients D_i of the synthesis window") — the 512 window
//!   coefficients (visual transcription from the 400-DPI page renders;
//!   cross-referenced against
//!   `docs/audio/mp3/mp1-annex-b-iso-extracts.md`'s table description).

use crate::frame_reconstruct::SubbandMatrix;
use crate::sv7_band_decode::SAMPLES_PER_BAND;
use crate::sv7_band_header::SV7_SUBBAND_COUNT;
use crate::SAMPLES_PER_FRAME_PER_CHANNEL;
use core::f64::consts::PI;

/// Number of subbands fed to the filter at each time slot (the 32
/// polyphase subbands of the inherited Layer-II filterbank).
pub const SUBBANDS: usize = 32;

/// Length of the persistent `V` FIFO (16 shift slots × 64).
pub const V_LEN: usize = 1024;

/// Length of the windowed intermediate vector `U` / `W` (the 512-tap
/// synthesis window of ISO Table 3-B.3).
pub const WINDOW_LEN: usize = 512;

/// ISO/IEC 11172-3 Table 3-B.3 — coefficients `D_i` of the synthesis
/// window, all 512 values in index order `D[0]..=D[511]`.
///
/// Transcribed from the in-repo ISO PDF page renders (Annex B, pp.
/// 50-52 of the standard / PDF pages 56-58). The table is the artefact
/// of the original numerical-optimisation run and is **not** derivable
/// in closed form, so it is loaded verbatim. It is approximately
/// symmetric about `D[256]` (the windowed-sinc origin) and the values
/// are signed decimal fractions (the PDF prints them in European
/// decimal-comma notation, parsed here as `.`).
pub static SYNTHESIS_WINDOW: [f64; WINDOW_LEN] = [
    // D[0..=15]
    0.000000000,
    -0.000015259,
    -0.000015259,
    -0.000015259,
    -0.000015259,
    -0.000015259,
    -0.000015259,
    -0.000030518,
    -0.000030518,
    -0.000030518,
    -0.000030518,
    -0.000045776,
    -0.000045776,
    -0.000061035,
    -0.000061035,
    -0.000076294,
    // D[16..=31]
    -0.000076294,
    -0.000091553,
    -0.000106812,
    -0.000106812,
    -0.000122070,
    -0.000137329,
    -0.000152588,
    -0.000167847,
    -0.000198364,
    -0.000213623,
    -0.000244141,
    -0.000259399,
    -0.000289917,
    -0.000320435,
    -0.000366211,
    -0.000396729,
    // D[32..=47]
    -0.000442505,
    -0.000473022,
    -0.000534058,
    -0.000579834,
    -0.000625610,
    -0.000686646,
    -0.000747681,
    -0.000808716,
    -0.000885010,
    -0.000961304,
    -0.001037598,
    -0.001113892,
    -0.001205444,
    -0.001296997,
    -0.001388550,
    -0.001480103,
    // D[48..=63]
    -0.001586914,
    -0.001693726,
    -0.001785278,
    -0.001907349,
    -0.002014160,
    -0.002120972,
    -0.002243042,
    -0.002349854,
    -0.002456665,
    -0.002578735,
    -0.002685547,
    -0.002792358,
    -0.002899170,
    -0.002990723,
    -0.003082275,
    -0.003173828,
    // D[64..=79]
    0.003250122,
    0.003326416,
    0.003387451,
    0.003433228,
    0.003479004,
    0.003479004,
    0.003479004,
    0.003463745,
    0.003417969,
    0.003372192,
    0.003280640,
    0.003173828,
    0.003051758,
    0.002883911,
    0.002700806,
    0.002487183,
    // D[80..=95]
    0.002227783,
    0.001937866,
    0.001617432,
    0.001266479,
    0.000869751,
    0.000442505,
    -0.000030518,
    -0.000549316,
    -0.001098633,
    -0.001693726,
    -0.002334595,
    -0.003005981,
    -0.003723145,
    -0.004486084,
    -0.005294800,
    -0.006118774,
    // D[96..=111]
    -0.007003784,
    -0.007919312,
    -0.008865356,
    -0.009841919,
    -0.010848999,
    -0.011886597,
    -0.012939453,
    -0.014022827,
    -0.015121460,
    -0.016235352,
    -0.017349243,
    -0.018463135,
    -0.019577026,
    -0.020690918,
    -0.021789551,
    -0.022857666,
    // D[112..=127]
    -0.023910522,
    -0.024932861,
    -0.025909424,
    -0.026840210,
    -0.027725220,
    -0.028533936,
    -0.029281616,
    -0.029937744,
    -0.030532837,
    -0.031005859,
    -0.031387329,
    -0.031661987,
    -0.031814575,
    -0.031845093,
    -0.031738281,
    -0.031478882,
    // D[128..=143]
    0.031082153,
    0.030517578,
    0.029785156,
    0.028884888,
    0.027801514,
    0.026535034,
    0.025085449,
    0.023422241,
    0.021575928,
    0.019531250,
    0.017257690,
    0.014801025,
    0.012115479,
    0.009231567,
    0.006134033,
    0.002822876,
    // D[144..=159]
    -0.000686646,
    -0.004394531,
    -0.008316040,
    -0.012420654,
    -0.016708374,
    -0.021179199,
    -0.025817871,
    -0.030609131,
    -0.035552979,
    -0.040634155,
    -0.045837402,
    -0.051132202,
    -0.056533813,
    -0.061996460,
    -0.067520142,
    -0.073059082,
    // D[160..=175]
    -0.078628540,
    -0.084182739,
    -0.089706421,
    -0.095169067,
    -0.100540161,
    -0.105819702,
    -0.110946655,
    -0.115921021,
    -0.120697021,
    -0.125259399,
    -0.129562378,
    -0.133590698,
    -0.137298584,
    -0.140670776,
    -0.143676758,
    -0.146255493,
    // D[176..=191]
    -0.148422241,
    -0.150115967,
    -0.151306152,
    -0.151962280,
    -0.152069092,
    -0.151596069,
    -0.150497437,
    -0.148773193,
    -0.146362305,
    -0.143264771,
    -0.139450073,
    -0.134887695,
    -0.129577637,
    -0.123474121,
    -0.116577148,
    -0.108856201,
    // D[192..=207]
    0.100311279,
    0.090927124,
    0.080688477,
    0.069595337,
    0.057617187,
    0.044784546,
    0.031082153,
    0.016510010,
    0.001068115,
    -0.015228271,
    -0.032379150,
    -0.050354004,
    -0.069168091,
    -0.088775635,
    -0.109161377,
    -0.130310059,
    // D[208..=223]
    -0.152206421,
    -0.174789429,
    -0.198059082,
    -0.221984863,
    -0.246505737,
    -0.271591187,
    -0.297210693,
    -0.323318481,
    -0.349868774,
    -0.376800537,
    -0.404083252,
    -0.431655884,
    -0.459472656,
    -0.487472534,
    -0.515609741,
    -0.543823242,
    // D[224..=239]
    -0.572036743,
    -0.600219727,
    -0.628295898,
    -0.656219482,
    -0.683914185,
    -0.711318970,
    -0.738372803,
    -0.765029907,
    -0.791213989,
    -0.816864014,
    -0.841949463,
    -0.866363525,
    -0.890090942,
    -0.913055420,
    -0.935195923,
    -0.956481934,
    // D[240..=255]
    -0.976852417,
    -0.996246338,
    -1.014617920,
    -1.031936646,
    -1.048156738,
    -1.063217163,
    -1.077117920,
    -1.089782715,
    -1.101211548,
    -1.111373901,
    -1.120223999,
    -1.127746582,
    -1.133926392,
    -1.138763428,
    -1.142211914,
    -1.144287109,
    // D[256..=271]
    1.144989014,
    1.144287109,
    1.142211914,
    1.138763428,
    1.133926392,
    1.127746582,
    1.120223999,
    1.111373901,
    1.101211548,
    1.089782715,
    1.077117920,
    1.063217163,
    1.048156738,
    1.031936646,
    1.014617920,
    0.996246338,
    // D[272..=287]
    0.976852417,
    0.956481934,
    0.935195923,
    0.913055420,
    0.890090942,
    0.866363525,
    0.841949463,
    0.816864014,
    0.791213989,
    0.765029907,
    0.738372803,
    0.711318970,
    0.683914185,
    0.656219482,
    0.628295898,
    0.600219727,
    // D[288..=303]
    0.572036743,
    0.543823242,
    0.515609741,
    0.487472534,
    0.459472656,
    0.431655884,
    0.404083252,
    0.376800537,
    0.349868774,
    0.323318481,
    0.297210693,
    0.271591187,
    0.246505737,
    0.221984863,
    0.198059082,
    0.174789429,
    // D[304..=319]
    0.152206421,
    0.130310059,
    0.109161377,
    0.088775635,
    0.069168091,
    0.050354004,
    0.032379150,
    0.015228271,
    -0.001068115,
    -0.016510010,
    -0.031082153,
    -0.044784546,
    -0.057617187,
    -0.069595337,
    -0.080688477,
    -0.090927124,
    // D[320..=335]
    0.100311279,
    0.108856201,
    0.116577148,
    0.123474121,
    0.129577637,
    0.134887695,
    0.139450073,
    0.143264771,
    0.146362305,
    0.148773193,
    0.150497437,
    0.151596069,
    0.152069092,
    0.151962280,
    0.151306152,
    0.150115967,
    // D[336..=351]
    0.148422241,
    0.146255493,
    0.143676758,
    0.140670776,
    0.137298584,
    0.133590698,
    0.129562378,
    0.125259399,
    0.120697021,
    0.115921021,
    0.110946655,
    0.105819702,
    0.100540161,
    0.095169067,
    0.089706421,
    0.084182739,
    // D[352..=367]
    0.078628540,
    0.073059082,
    0.067520142,
    0.061996460,
    0.056533813,
    0.051132202,
    0.045837402,
    0.040634155,
    0.035552979,
    0.030609131,
    0.025817871,
    0.021179199,
    0.016708374,
    0.012420654,
    0.008316040,
    0.004394531,
    // D[368..=383]
    0.000686646,
    -0.002822876,
    -0.006134033,
    -0.009231567,
    -0.012115479,
    -0.014801025,
    -0.017257690,
    -0.019531250,
    -0.021575928,
    -0.023422241,
    -0.025085449,
    -0.026535034,
    -0.027801514,
    -0.028884888,
    -0.029785156,
    -0.030517578,
    // D[384..=399]
    0.031082153,
    0.031478882,
    0.031738281,
    0.031845093,
    0.031814575,
    0.031661987,
    0.031387329,
    0.031005859,
    0.030532837,
    0.029937744,
    0.029281616,
    0.028533936,
    0.027725220,
    0.026840210,
    0.025909424,
    0.024932861,
    // D[400..=415]
    0.023910522,
    0.022857666,
    0.021789551,
    0.020690918,
    0.019577026,
    0.018463135,
    0.017349243,
    0.016235352,
    0.015121460,
    0.014022827,
    0.012939453,
    0.011886597,
    0.010848999,
    0.009841919,
    0.008865356,
    0.007919312,
    // D[416..=431]
    0.007003784,
    0.006118774,
    0.005294800,
    0.004486084,
    0.003723145,
    0.003005981,
    0.002334595,
    0.001693726,
    0.001098633,
    0.000549316,
    0.000030518,
    -0.000442505,
    -0.000869751,
    -0.001266479,
    -0.001617432,
    -0.001937866,
    // D[432..=447]
    -0.002227783,
    -0.002487183,
    -0.002700806,
    -0.002883911,
    -0.003051758,
    -0.003173828,
    -0.003280640,
    -0.003372192,
    -0.003417969,
    -0.003463745,
    -0.003479004,
    -0.003479004,
    -0.003479004,
    -0.003433228,
    -0.003387451,
    -0.003326416,
    // D[448..=463]
    0.003250122,
    0.003173828,
    0.003082275,
    0.002990723,
    0.002899170,
    0.002792358,
    0.002685547,
    0.002578735,
    0.002456665,
    0.002349854,
    0.002243042,
    0.002120972,
    0.002014160,
    0.001907349,
    0.001785278,
    0.001693726,
    // D[464..=479]
    0.001586914,
    0.001480103,
    0.001388550,
    0.001296997,
    0.001205444,
    0.001113892,
    0.001037598,
    0.000961304,
    0.000885010,
    0.000808716,
    0.000747681,
    0.000686646,
    0.000625610,
    0.000579834,
    0.000534058,
    0.000473022,
    // D[480..=495]
    0.000442505,
    0.000396729,
    0.000366211,
    0.000320435,
    0.000289917,
    0.000259399,
    0.000244141,
    0.000213623,
    0.000198364,
    0.000167847,
    0.000152588,
    0.000137329,
    0.000122070,
    0.000106812,
    0.000106812,
    0.000091553,
    // D[496..=511]
    0.000076294,
    0.000076294,
    0.000061035,
    0.000061035,
    0.000045776,
    0.000045776,
    0.000030518,
    0.000030518,
    0.000030518,
    0.000030518,
    0.000015259,
    0.000015259,
    0.000015259,
    0.000015259,
    0.000015259,
    0.000015259,
];

/// The ISO Figure A.2 matrixing coefficient
/// `N_ik = cos[(16 + i)·(2k + 1)·π / 64]` for `i` in `0..64` (the V
/// output index) and `k` in `0..32` (the subband index).
///
/// Computed from the closed-form formula the ISO Annex A figure gives;
/// no table is transcribed for `N_ik` (the spec prints the formula, not
/// a table).
#[inline]
#[must_use]
pub fn matrix_coefficient(i: usize, k: usize) -> f64 {
    debug_assert!(i < 64 && k < SUBBANDS);
    (((16 + i) * (2 * k + 1)) as f64 * PI / 64.0).cos()
}

/// The persistent 32-band polyphase synthesis subband filter.
///
/// Holds the 1024-entry `V` FIFO, zero-initialised at startup
/// (ISO Figure A.2 footnote 1). One [`SynthesisFilter`] instance is
/// driven per audio channel across a stream; its `V` state carries the
/// inter-call overlap that the windowed sum needs.
#[derive(Clone)]
pub struct SynthesisFilter {
    /// The `V` FIFO. `v[0..64]` is the most recent matrixing output;
    /// older blocks shift up toward `v[1023]`.
    v: [f64; V_LEN],
}

impl Default for SynthesisFilter {
    fn default() -> Self {
        Self::new()
    }
}

impl core::fmt::Debug for SynthesisFilter {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        // The 1024-entry FIFO is not worth printing; summarise instead.
        f.debug_struct("SynthesisFilter")
            .field("v_len", &self.v.len())
            .finish()
    }
}

impl SynthesisFilter {
    /// A fresh filter with the `V` FIFO zero-initialised
    /// (ISO Figure A.2 footnote 1: "`V` to be initialized with zeroes
    /// during startup").
    #[must_use]
    pub fn new() -> Self {
        Self { v: [0.0; V_LEN] }
    }

    /// Run one time slot of the synthesis filter: consume the 32 subband
    /// samples `samples[k]` (one per subband `k`) and produce 32
    /// consecutive PCM output samples, advancing the internal `V` FIFO.
    ///
    /// Implements ISO Figure A.2 steps 1-5 (shift, matrix, build U,
    /// window, sum) exactly.
    pub fn synthesize(&mut self, samples: &[f64; SUBBANDS]) -> [f64; SUBBANDS] {
        // Step 1 — shift the V FIFO up by 64 (oldest 64 entries drop off
        // the top). `for i = 1023 down to 64: V[i] = V[i-64]`.
        self.v.copy_within(0..V_LEN - 64, 64);

        // Step 2 — matrixing into the freshly-vacated V[0..64].
        // `for i = 0 to 63: V[i] = sum_{k} N_ik · S_k`.
        for (i, slot) in self.v[0..64].iter_mut().enumerate() {
            let mut acc = 0.0_f64;
            for (k, &s) in samples.iter().enumerate() {
                acc += matrix_coefficient(i, k) * s;
            }
            *slot = acc;
        }

        // Step 3 — build the 512-value vector U from the V FIFO.
        // `for i=0..8, j=0..32: U[i*64+j]=V[i*128+j];
        //  U[i*64+32+j]=V[i*128+96+j]`.
        let mut u = [0.0_f64; WINDOW_LEN];
        for i in 0..8 {
            for j in 0..32 {
                u[i * 64 + j] = self.v[i * 128 + j];
                u[i * 64 + 32 + j] = self.v[i * 128 + 96 + j];
            }
        }

        // Steps 4 & 5 — window by D, then sum 16 windowed taps per
        // output sample. `W[i] = U[i] · D[i]`;
        // `out_j = sum_{i=0}^{15} W[j + 32*i]`.
        let mut out = [0.0_f64; SUBBANDS];
        for (j, o) in out.iter_mut().enumerate() {
            let mut acc = 0.0_f64;
            for i in 0..16 {
                let idx = j + 32 * i;
                acc += u[idx] * SYNTHESIS_WINDOW[idx];
            }
            *o = acc;
        }
        out
    }

    /// Reset the `V` FIFO to all-zero (the startup state), e.g. at a
    /// stream seek or before decoding a fresh independent channel.
    pub fn reset(&mut self) {
        self.v = [0.0; V_LEN];
    }
}

/// Run the synthesis filterbank over one channel's reconstructed
/// [`SubbandMatrix`] for a single frame, producing the
/// [`SAMPLES_PER_FRAME_PER_CHANNEL`] (`1152`) PCM samples in output
/// order.
///
/// The matrix row `b` is subband `b`'s 36 time-ordered samples; the
/// filterbank consumes the matrix **column by column** — at each of the
/// 36 time slots it takes one sample from every subband as the
/// `S[0..31]` vector and emits 32 PCM samples
/// (`36 × 32 = 1152`). `filter` carries the inter-frame `V` overlap, so
/// the same instance must be reused across consecutive frames of one
/// channel.
///
/// (The [`SubbandMatrix`] has [`SV7_SUBBAND_COUNT`] = 32 rows and
/// [`SAMPLES_PER_BAND`] = 36 columns; both match the filterbank's
/// [`SUBBANDS`] and the 36-slot frame geometry.)
pub fn synthesize_frame_channel(
    filter: &mut SynthesisFilter,
    matrix: &SubbandMatrix,
) -> [f64; SAMPLES_PER_FRAME_PER_CHANNEL] {
    debug_assert_eq!(SV7_SUBBAND_COUNT, SUBBANDS);
    debug_assert_eq!(SAMPLES_PER_BAND, 36);

    let mut pcm = [0.0_f64; SAMPLES_PER_FRAME_PER_CHANNEL];
    for slot in 0..SAMPLES_PER_BAND {
        // Gather the per-subband sample at this time slot.
        let mut s = [0.0_f64; SUBBANDS];
        for (k, sk) in s.iter_mut().enumerate() {
            *sk = matrix[k][slot];
        }
        let out = filter.synthesize(&s);
        pcm[slot * SUBBANDS..slot * SUBBANDS + SUBBANDS].copy_from_slice(&out);
    }
    pcm
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn window_has_512_entries() {
        assert_eq!(SYNTHESIS_WINDOW.len(), WINDOW_LEN);
        assert_eq!(WINDOW_LEN, 512);
    }

    #[test]
    fn window_endpoints_match_table_b3() {
        // D[0] = 0; D[1..=6] = -0.000015259; D[7] = -0.000030518.
        assert_eq!(SYNTHESIS_WINDOW[0], 0.0);
        for &v in &SYNTHESIS_WINDOW[1..=6] {
            assert_eq!(v, -0.000015259);
        }
        assert_eq!(SYNTHESIS_WINDOW[7], -0.000030518);
        // The peak at D[256].
        assert_eq!(SYNTHESIS_WINDOW[256], 1.144989014);
        // D[511] tail value.
        assert_eq!(SYNTHESIS_WINDOW[511], 0.000015259);
    }

    #[test]
    fn window_peak_is_at_256() {
        // D[256] is the global maximum of the window (windowed-sinc peak).
        let peak = SYNTHESIS_WINDOW[256];
        for (idx, &v) in SYNTHESIS_WINDOW.iter().enumerate() {
            assert!(
                v <= peak,
                "D[{idx}] = {v} exceeds the documented peak D[256] = {peak}",
            );
        }
    }

    #[test]
    fn window_magnitude_symmetric_about_256() {
        // ISO Table 3-B.3 is magnitude-symmetric: |D[512 - i]| == |D[i]|
        // for i in 1..=255 (the renders confirm e.g. D[64]/D[448],
        // D[224]/D[288], D[240]/D[272]). This catches the bulk of
        // single-digit transcription typos: any value that breaks the
        // mirror is suspect. (D[0] = 0 has no mirror partner < 512.)
        for i in 1..256 {
            let lhs = SYNTHESIS_WINDOW[i].abs();
            let rhs = SYNTHESIS_WINDOW[512 - i].abs();
            assert!(
                (lhs - rhs).abs() < 1e-12,
                "|D[{i}]| = {lhs} vs |D[{}]| = {rhs} must match (window magnitude symmetry)",
                512 - i,
            );
        }
    }

    #[test]
    fn window_sign_mirror_about_256() {
        // Beyond magnitude, the documented sign relationship is
        // D[512 - i] == -D[i] for the lower octant pairs and == +D[i]
        // for the others; the precise rule is the windowed-sinc lobe
        // structure. We pin the two boundary samples whose sign was
        // transcription-sensitive (D[128]/D[384], D[320]/D[192]).
        assert!(SYNTHESIS_WINDOW[128] > 0.0); // +0.031082153
        assert!(SYNTHESIS_WINDOW[384] > 0.0); // +0.031082153
        assert!(SYNTHESIS_WINDOW[192] > 0.0); // +0.100311279
        assert!(SYNTHESIS_WINDOW[320] > 0.0); // +0.100311279
        assert!(SYNTHESIS_WINDOW[377] < 0.0); // -0.023422241
    }

    #[test]
    fn matrix_coefficient_formula_endpoints() {
        // N_ik = cos[(16+i)(2k+1)π/64].
        // i=0, k=0: cos(16π/64) = cos(π/4) = √2/2.
        assert!((matrix_coefficient(0, 0) - (2.0_f64).sqrt() / 2.0).abs() < 1e-12);
        // i=16, k=0: cos(32π/64) = cos(π/2) = 0.
        assert!(matrix_coefficient(16, 0).abs() < 1e-12);
        // i=48, k=0: cos(64π/64) = cos(π) = -1.
        assert!((matrix_coefficient(48, 0) - (-1.0)).abs() < 1e-12);
    }

    #[test]
    fn fresh_filter_v_is_zero() {
        let f = SynthesisFilter::new();
        assert!(f.v.iter().all(|&x| x == 0.0));
    }

    #[test]
    fn synthesize_zero_input_gives_zero_output() {
        let mut f = SynthesisFilter::new();
        let out = f.synthesize(&[0.0; SUBBANDS]);
        assert!(out.iter().all(|&x| x == 0.0));
    }

    #[test]
    fn first_slot_uses_only_top_64_of_v() {
        // After one synthesize() on a zeroed filter, V[64..] is still
        // zero (the shift moved zeros up), so U's contributions all come
        // from the freshly-matrixed V[0..64]. A non-zero subband input
        // must therefore produce a non-zero output.
        let mut f = SynthesisFilter::new();
        let mut s = [0.0; SUBBANDS];
        s[0] = 1.0;
        let out = f.synthesize(&s);
        assert!(
            out.iter().any(|&x| x.abs() > 1e-12),
            "non-zero subband 0 input should yield non-zero PCM output",
        );
    }

    #[test]
    fn synthesize_is_linear() {
        // The whole filter is linear: synth(a·x) == a·synth(x) and
        // synth(x+y) == synth(x) + synth(y) on a fresh filter.
        let mut s = [0.0; SUBBANDS];
        for (k, v) in s.iter_mut().enumerate() {
            *v = (k as f64 * 0.013).sin();
        }

        let mut f1 = SynthesisFilter::new();
        let a = f1.synthesize(&s);

        let mut scaled = s;
        for v in scaled.iter_mut() {
            *v *= 3.0;
        }
        let mut f2 = SynthesisFilter::new();
        let b = f2.synthesize(&scaled);

        for (x, y) in a.iter().zip(b.iter()) {
            assert!((x * 3.0 - y).abs() < 1e-9, "scaling: {} vs {}", x * 3.0, y);
        }
    }

    #[test]
    fn shift_carries_state_across_calls() {
        // A DC subband-0 impulse on call 1 then silence on call 2 must
        // still produce output on call 2, because the matrixed V block
        // shifted down into the U window's later taps.
        let mut f = SynthesisFilter::new();
        let mut s = [0.0; SUBBANDS];
        s[0] = 1.0;
        let _ = f.synthesize(&s);
        let out2 = f.synthesize(&[0.0; SUBBANDS]);
        assert!(
            out2.iter().any(|&x| x.abs() > 1e-12),
            "the shifted V state should still contribute on the next call",
        );
    }

    #[test]
    fn reset_zeroes_state() {
        let mut f = SynthesisFilter::new();
        let mut s = [0.0; SUBBANDS];
        s[5] = 0.7;
        let _ = f.synthesize(&s);
        f.reset();
        assert!(f.v.iter().all(|&x| x == 0.0));
        // After reset, a zero input gives a pure-zero output again.
        let out = f.synthesize(&[0.0; SUBBANDS]);
        assert!(out.iter().all(|&x| x == 0.0));
    }

    #[test]
    fn frame_channel_zero_matrix_is_silent() {
        let mut f = SynthesisFilter::new();
        let m = crate::frame_reconstruct::zero_subband_matrix();
        let pcm = synthesize_frame_channel(&mut f, &m);
        assert_eq!(pcm.len(), SAMPLES_PER_FRAME_PER_CHANNEL);
        assert!(pcm.iter().all(|&x| x == 0.0));
    }

    #[test]
    fn frame_channel_matches_slot_by_slot_driver() {
        // synthesize_frame_channel must equal driving synthesize()
        // column by column on the same filter state.
        let mut m = crate::frame_reconstruct::zero_subband_matrix();
        for (b, row) in m.iter_mut().enumerate() {
            for (t, x) in row.iter_mut().enumerate() {
                *x = ((b * 7 + t) as f64 * 0.001).sin();
            }
        }

        let mut f_frame = SynthesisFilter::new();
        let pcm = synthesize_frame_channel(&mut f_frame, &m);

        let mut f_manual = SynthesisFilter::new();
        let mut expected = [0.0_f64; SAMPLES_PER_FRAME_PER_CHANNEL];
        for slot in 0..SAMPLES_PER_BAND {
            let mut s = [0.0_f64; SUBBANDS];
            for (k, sk) in s.iter_mut().enumerate() {
                *sk = m[k][slot];
            }
            let out = f_manual.synthesize(&s);
            expected[slot * SUBBANDS..slot * SUBBANDS + SUBBANDS].copy_from_slice(&out);
        }

        for (a, b) in pcm.iter().zip(expected.iter()) {
            assert_eq!(a, b);
        }
    }

    #[test]
    fn frame_channel_produces_full_1152() {
        let mut f = SynthesisFilter::new();
        let mut m = crate::frame_reconstruct::zero_subband_matrix();
        m[0][0] = 1.0;
        let pcm = synthesize_frame_channel(&mut f, &m);
        assert_eq!(pcm.len(), 1152);
        assert!(pcm.iter().any(|&x| x.abs() > 1e-12));
    }
}
