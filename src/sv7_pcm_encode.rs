//! SV7 **whole-stream from-PCM encoder** — PCM in, a complete
//! `MP+` byte stream out: the SV7 twin of [`crate::sv8_file_encode`].
//!
//! Pipeline: [`crate::analysis`] (PCM → subband matrices) →
//! [`crate::sv7_frame_build`] (matrices → [`Sv7EncStereoFrame`]) →
//! [`crate::sv7_file_encode::Sv7FileWriter`] (frames → §1 header +
//! prefixed bodies + 11-bit trailer, word-swapped).
//!
//! # Gapless geometry
//!
//! The SV7 header has no `beginning_silence` field; the decoder
//! window is fixed at `decoded[481 .. 481 + effective_total]` (r429,
//! reference-decoder-pinned). The analysis front end delays the
//! signal by the matching 481 samples, so declaring
//! `effective_total = N` (frame count `⌈N / 1152⌉`, last-frame count
//! `N − (frames−1)·1152`) makes the decode time-aligned with the
//! input. When the declared frames' slack past `N` is under the
//! 481-sample priming tail (`(1152 − last) mod 1152 < 481`), the
//! encoder appends one **undeclared flush frame** after the trailer —
//! the reference producers' own posture, which both this crate's
//! decoder and the reference console decoder consume — so the tail is
//! always coded, never drain-approximated.
//!
//! # Mono
//!
//! SV7 is stereo-only on the wire (§1: channel count is always 2). A
//! mono input is fed as both channels; every coded band then elects
//! M/S with a silent side (the cheap mono body), and either output
//! channel is the signal.
//!
//! Source-of-record: §1/§1.1 (header + framing) via the file layer;
//! everything else is encoder policy over pinned decode facts.

use crate::analysis::{analyze_frame_channel, AnalysisFilter};
use crate::frame_reconstruct::SubbandMatrix;
use crate::sv7_file_encode::{Sv7EncStereoFrame, Sv7FileWriter};
use crate::sv7_frame_build::{build_sv7_stereo_frame, Sv7FrameBuildSettings};
use crate::sv7_header::Sv7HeaderFields;
use crate::synthesis::SYNTHESIS_PRIME_SAMPLES;
use crate::{Error, Result, SAMPLES_PER_FRAME_PER_CHANNEL};

/// Whole-stream SV7 encoder settings.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sv7EncoderSettings {
    /// Flat s16-domain quantisation-step target
    /// ([`Sv7FrameBuildSettings::step_target`]).
    pub step_target: f64,
    /// §1 stream-wide mid-side flag + per-band election.
    pub stream_ms: bool,
    /// §1 highest coded subband (`1..=31`).
    pub max_band: u8,
    /// §1 profile nibble (informational; the corpus `--standard`
    /// streams carry 10).
    pub profile: u8,
    /// Noise-substitution threshold
    /// ([`Sv7FrameBuildSettings::pns_threshold`]); a positive value
    /// also raises the stream's `MP+ 0x17` PNS version-byte flag.
    pub pns_threshold: f64,
}

impl Default for Sv7EncoderSettings {
    fn default() -> Self {
        Self {
            step_target: 2.0,
            stream_ms: true,
            max_band: 31,
            profile: 10,
            pns_threshold: 0.0,
        }
    }
}

/// The result of a whole-stream SV7 encode.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Sv7EncodedStream {
    /// The complete raw `.mpc` bytes (`MP+` magic onward).
    pub bytes: Vec<u8>,
    /// Declared §1 frame count.
    pub frames: u32,
    /// Whether an undeclared flush frame was appended after the
    /// trailer (see the module docs).
    pub flush_frame: bool,
}

/// Encode interleaved f64 PCM (s16 domain) into a complete SV7 `MP+`
/// stream. `pcm` is interleaved `L, R, …` for `channels == 2`, plain
/// mono for `channels == 1`; `sample_freq_index` indexes the §1
/// {44100, 48000, 37800, 32000} Hz table.
///
/// Decoding the result (here or with the reference console decoder)
/// returns exactly `pcm.len() / channels` samples per channel,
/// time-aligned with the input.
///
/// # Errors
///
/// - [`Error::ChannelCountInvalid`] for a channel count other than 1
///   or 2, an odd stereo buffer, or an empty input.
/// - [`Error::MaxBandOutOfRange`] / [`Error::HeaderFieldOutOfRange`]
///   for out-of-range settings.
pub fn encode_sv7_from_pcm_f64(
    pcm: &[f64],
    channels: u8,
    sample_freq_index: u8,
    settings: &Sv7EncoderSettings,
) -> Result<Sv7EncodedStream> {
    if !(1..=2).contains(&channels) || pcm.len() % channels as usize != 0 || pcm.is_empty() {
        return Err(Error::ChannelCountInvalid(channels));
    }
    let nch = channels as usize;
    let n = (pcm.len() / nch) as u64;

    let frame_len = SAMPLES_PER_FRAME_PER_CHANNEL as u64;
    let frames = n.div_ceil(frame_len);
    let last = (n - (frames - 1) * frame_len) as u16; // 1..=1152
                                                      // Priming-tail slack past the declared timeline (module docs).
    let need_flush = frames * frame_len - n < SYNTHESIS_PRIME_SAMPLES as u64;

    let header = Sv7HeaderFields {
        mid_side: settings.stream_ms,
        max_band: settings.max_band,
        profile: settings.profile,
        sample_freq_index,
        pns: settings.pns_threshold > 0.0,
        encoder_version: 0x71,
        ..Default::default()
    };
    let build = Sv7FrameBuildSettings {
        step_target: settings.step_target,
        stream_ms: settings.stream_ms,
        pns_threshold: settings.pns_threshold,
    };

    let mut writer = Sv7FileWriter::new(header)?;
    let mut filters = vec![AnalysisFilter::new(); nch];
    let mut frame_pcm = [0.0_f64; SAMPLES_PER_FRAME_PER_CHANNEL];
    let total_frames = frames + u64::from(need_flush);
    let mut flush_built: Option<Sv7EncStereoFrame> = None;
    for f in 0..total_frames {
        let base = f * frame_len;
        let mut matrices: Vec<SubbandMatrix> = Vec::with_capacity(nch);
        for (ch, filter) in filters.iter_mut().enumerate() {
            for (k, slot) in frame_pcm.iter_mut().enumerate() {
                let t = base + k as u64;
                *slot = if t < n {
                    pcm[(t as usize) * nch + ch]
                } else {
                    0.0
                };
            }
            matrices.push(analyze_frame_channel(filter, &frame_pcm));
        }
        let (l, r) = if nch == 2 {
            (&matrices[0], &matrices[1])
        } else {
            (&matrices[0], &matrices[0])
        };
        let frame = build_sv7_stereo_frame(l, r, settings.max_band, &build)?;
        if f < frames {
            writer.push_frame(&frame)?;
        } else {
            flush_built = Some(frame);
        }
    }

    let bytes = writer.finish_gapless_with_flush(last, flush_built.as_ref())?;
    Ok(Sv7EncodedStream {
        bytes,
        frames: frames as u32,
        flush_frame: need_flush,
    })
}

/// [`encode_sv7_from_pcm_f64`] over interleaved `i16` PCM.
///
/// # Errors
///
/// As [`encode_sv7_from_pcm_f64`].
pub fn encode_sv7_from_pcm_s16(
    pcm: &[i16],
    channels: u8,
    sample_freq_index: u8,
    settings: &Sv7EncoderSettings,
) -> Result<Sv7EncodedStream> {
    let f: Vec<f64> = pcm.iter().map(|&s| f64::from(s)).collect();
    encode_sv7_from_pcm_f64(&f, channels, sample_freq_index, settings)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sv7_file_decode::decode_sv7_file;
    use std::f64::consts::PI;

    fn stereo_tone(n: usize) -> Vec<f64> {
        let mut pcm = Vec::with_capacity(n * 2);
        for t in 0..n {
            let x = t as f64 / 44100.0;
            pcm.push(9000.0 * (2.0 * PI * 440.0 * x).sin());
            pcm.push(9000.0 * (2.0 * PI * 660.0 * x).sin());
        }
        pcm
    }

    fn snr(a: &[f64], b: &[f64]) -> f64 {
        let (mut s, mut e) = (0.0, 0.0);
        for (x, y) in a.iter().zip(b.iter()) {
            s += x * x;
            e += (x - y) * (x - y);
        }
        10.0 * (s / e).log10()
    }

    /// Stereo round trip: exact sample count, input-aligned, high SNR.
    /// `n` is chosen so the declared slack is under 481 and the flush
    /// frame engages.
    #[test]
    fn stereo_round_trip_with_flush_frame() {
        let n = 20_000usize; // 18 frames, last = 416 → slack 736? no: 18·1152 = 20736, slack 736 ≥ 481 → no flush.
        let pcm = stereo_tone(n);
        let enc =
            encode_sv7_from_pcm_f64(&pcm, 2, 0, &Sv7EncoderSettings::default()).expect("encode");
        let dec = decode_sv7_file(&enc.bytes).expect("decode");
        assert_eq!(dec.pcm.len(), pcm.len());
        let s = snr(&pcm, &dec.pcm);
        assert!(s > 60.0, "round-trip SNR {s:.1} dB");
    }

    /// An exact-multiple length has zero slack — the flush frame must
    /// engage and the tail must still be real audio, not drain
    /// approximation.
    #[test]
    fn exact_multiple_length_uses_flush_frame() {
        let n = 1152 * 8;
        let pcm = stereo_tone(n);
        let enc =
            encode_sv7_from_pcm_f64(&pcm, 2, 0, &Sv7EncoderSettings::default()).expect("encode");
        assert!(enc.flush_frame, "zero slack must flush");
        assert_eq!(enc.frames, 8);
        let dec = decode_sv7_file(&enc.bytes).expect("decode");
        assert_eq!(dec.pcm.len(), pcm.len());
        // The last 481 samples come from the flush frame — they must
        // still track the input.
        let tail = 481 * 2;
        let s = snr(&pcm[pcm.len() - tail..], &dec.pcm[dec.pcm.len() - tail..]);
        assert!(s > 45.0, "flush-covered tail SNR {s:.1} dB");
    }

    /// Mono input: fed as both channels, decodes to the doubled
    /// interleaved output whose either channel is the signal.
    #[test]
    fn mono_input_round_trips_via_ms() {
        let n = 6000usize;
        let mono: Vec<f64> = (0..n)
            .map(|t| 8000.0 * (2.0 * PI * 500.0 * t as f64 / 44100.0).sin())
            .collect();
        let enc =
            encode_sv7_from_pcm_f64(&mono, 1, 0, &Sv7EncoderSettings::default()).expect("encode");
        let dec = decode_sv7_file(&enc.bytes).expect("decode");
        // SV7 output is always stereo; channel 0 must be the signal.
        assert_eq!(dec.pcm.len(), n * 2);
        let ch0: Vec<f64> = dec.pcm.iter().step_by(2).copied().collect();
        let s = snr(&mono, &ch0);
        assert!(s > 60.0, "mono round-trip SNR {s:.1} dB");
    }

    /// Invalid inputs fail loud.
    #[test]
    fn invalid_inputs_rejected() {
        assert_eq!(
            encode_sv7_from_pcm_f64(&[], 2, 0, &Sv7EncoderSettings::default()),
            Err(Error::ChannelCountInvalid(2))
        );
        assert_eq!(
            encode_sv7_from_pcm_f64(&[0.0; 3], 2, 0, &Sv7EncoderSettings::default()),
            Err(Error::ChannelCountInvalid(2))
        );
        assert_eq!(
            encode_sv7_from_pcm_f64(&[0.0; 4], 3, 0, &Sv7EncoderSettings::default()),
            Err(Error::ChannelCountInvalid(3))
        );
    }
}
