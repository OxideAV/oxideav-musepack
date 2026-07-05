//! SV7 frame-body encode input model + per-band sample writer.
//!
//! [`Sv7EncBand`] describes one subband of one channel in the three §5.4
//! shapes the frame writer handles; [`encode_sv7_band_samples`] writes
//! one band's samples-pass bits (the 1-bit context selector when the
//! band-type uses one, then the 36 levels). The whole-frame pass
//! ordering — §5.1 header, then the SCFI pass, then the DSCF pass, then
//! the samples pass, each band-major / channel-minor — lives in
//! [`crate::sv7_stereo_frame_encode`], mirroring the corpus-pinned
//! decode layout ([`crate::sv7_stereo_frame`]).
//!
//! CNS bands (`Res == -1`) carry a scalefactor triple like coded bands
//! (the §5.2 "for each channel whose `Res ≠ 0`" gate; the structural
//! spec's noise band is "scaled by the band's scalefactor") but no
//! sample bits — the 36 noise levels come from the shared PRNG on
//! decode.
//!
//! Source-of-record: `docs/audio/musepack/spec/musepack-headers-and-coding.md`
//! §5.1–§5.5; the pass layout is fixture-corpus-pinned (see
//! [`crate::sv7_stereo_frame`]).

use crate::sv7_band_decode::{band_type_uses_context_selector, SAMPLES_PER_BAND};
use crate::sv7_bitwriter::Sv7BitWriter;
use crate::sv7_sample_encode::encode_sv7_band;
use crate::{Error, Result};

/// One band of a channel's frame body, in the three §5.4 shapes the
/// frame writer handles.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Sv7EncBand {
    /// `Res == 0`: silent subband. No SCF layer, no sample bits.
    Empty,
    /// `Res == -1`: noise substitution. Carries the SCF triple (the
    /// noise gain) but no sample bits (PRNG on decode).
    Cns {
        /// The three per-granule SCF indices scaling the noise.
        scf: [i32; 3],
    },
    /// `Res` in `1..=17`: a coded band.
    Coded {
        /// The band-type (`Res`), driving the §5.4 sample arm.
        band_type: i8,
        /// The §5.4 1-bit context selector (`0` / `1`); ignored by the
        /// no-context arms (PCM-escape). Must be `0` or `1`.
        ctx: usize,
        /// The three per-granule SCF indices; the SCFI selector is
        /// derived from their sharing pattern
        /// ([`crate::sv7_scf_encode::choose_scfi`]).
        scf: [i32; 3],
        /// The 36 quantised sample levels for the band.
        levels: [i32; SAMPLES_PER_BAND],
    },
}

impl Sv7EncBand {
    /// The band's `Res` (band-type) as it appears in the §5.1 header
    /// sequence: `0` for [`Sv7EncBand::Empty`], `-1` for
    /// [`Sv7EncBand::Cns`], and the coded band's `band_type` otherwise.
    pub fn res(&self) -> i8 {
        match self {
            Sv7EncBand::Empty => 0,
            Sv7EncBand::Cns { .. } => -1,
            Sv7EncBand::Coded { band_type, .. } => *band_type,
        }
    }

    /// The band's SCF triple, `None` for the silent band (which carries
    /// no SCF layer).
    pub fn scf(&self) -> Option<[i32; 3]> {
        match self {
            Sv7EncBand::Empty => None,
            Sv7EncBand::Cns { scf } | Sv7EncBand::Coded { scf, .. } => Some(*scf),
        }
    }
}

/// Derive the `Res` (band-type) sequence a `bands` slice implies — the
/// per-channel column of the §5.1 header.
pub fn res_sequence(bands: &[Sv7EncBand]) -> Vec<i8> {
    bands.iter().map(Sv7EncBand::res).collect()
}

/// Write one band's **samples-pass** bits: nothing for the silent and
/// CNS bands; the 1-bit context selector (when the band-type uses one)
/// plus the 36 §5.4 sample levels for a coded band.
///
/// # Errors
///
/// - [`Error::UnsupportedBandType`] for a coded `band_type` outside
///   `1..=17`, or a `ctx` outside `0..=1`.
/// - [`Error::SampleOutOfRange`] / [`Error::SymbolNotEncodable`] from a
///   sample arm whose input is not representable.
pub fn encode_sv7_band_samples(writer: &mut Sv7BitWriter, band: &Sv7EncBand) -> Result<()> {
    match band {
        Sv7EncBand::Empty | Sv7EncBand::Cns { .. } => Ok(()),
        Sv7EncBand::Coded {
            band_type,
            ctx,
            levels,
            ..
        } => {
            if *ctx > 1 {
                return Err(Error::UnsupportedBandType(*band_type));
            }
            if band_type_uses_context_selector(*band_type) {
                writer.write_bits(*ctx as u32, 1);
            }
            encode_sv7_band(writer, *band_type, *ctx, levels)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::huffman::{sv7_q3_ctx, Sv7BitReader};
    use crate::sv7_band_decode::decode_sv7_band;

    fn q3_levels() -> [i32; SAMPLES_PER_BAND] {
        let mut a: Vec<i32> = sv7_q3_ctx(0).iter().map(|e| e.value as i32).collect();
        a.dedup();
        core::array::from_fn(|i| a[i % a.len()])
    }

    #[test]
    fn res_helpers_map_variants() {
        assert_eq!(Sv7EncBand::Empty.res(), 0);
        assert_eq!(Sv7EncBand::Cns { scf: [4, 4, 4] }.res(), -1);
        let coded = Sv7EncBand::Coded {
            band_type: 5,
            ctx: 0,
            scf: [0; 3],
            levels: [0; SAMPLES_PER_BAND],
        };
        assert_eq!(coded.res(), 5);
        assert_eq!(coded.scf(), Some([0, 0, 0]));
        assert_eq!(Sv7EncBand::Empty.scf(), None);
        assert_eq!(Sv7EncBand::Cns { scf: [1, 2, 3] }.scf(), Some([1, 2, 3]));
        let bands = [Sv7EncBand::Empty, Sv7EncBand::Cns { scf: [0; 3] }];
        assert_eq!(res_sequence(&bands), vec![0, -1]);
    }

    #[test]
    fn silent_and_cns_bands_write_no_sample_bits() {
        let mut w = Sv7BitWriter::new();
        encode_sv7_band_samples(&mut w, &Sv7EncBand::Empty).unwrap();
        encode_sv7_band_samples(&mut w, &Sv7EncBand::Cns { scf: [9, 9, 9] }).unwrap();
        assert!(w.is_empty());
    }

    #[test]
    fn coded_band_samples_round_trip_with_selector() {
        let levels = q3_levels();
        let band = Sv7EncBand::Coded {
            band_type: 3,
            ctx: 1,
            scf: [7, 7, 7],
            levels,
        };
        let mut w = Sv7BitWriter::new();
        encode_sv7_band_samples(&mut w, &band).unwrap();
        let mut bytes = w.finish();
        bytes.extend_from_slice(&[0, 0, 0, 0]);
        let mut r = Sv7BitReader::new(&bytes);
        // Selector first.
        assert_eq!(r.read_bits(1).unwrap(), 1);
        let mut cns = crate::cns::CnsPrng::new();
        let mut got = [0i32; SAMPLES_PER_BAND];
        decode_sv7_band(&mut r, 3, &mut cns, 1, &mut got).unwrap();
        assert_eq!(got, levels);
    }

    #[test]
    fn pcm_escape_band_writes_no_selector() {
        let mut levels = [0i32; SAMPLES_PER_BAND];
        for (i, v) in levels.iter_mut().enumerate() {
            *v = (i as i32) & 0x7F;
        }
        let band = Sv7EncBand::Coded {
            band_type: 8,
            ctx: 0,
            scf: [0; 3],
            levels,
        };
        let mut w = Sv7BitWriter::new();
        encode_sv7_band_samples(&mut w, &band).unwrap();
        // 36 × 7 raw bits exactly, no selector.
        assert_eq!(w.bit_len(), 36 * 7);
    }

    #[test]
    fn rejects_bad_ctx() {
        let band = Sv7EncBand::Coded {
            band_type: 3,
            ctx: 2,
            scf: [1, 1, 1],
            levels: [0; SAMPLES_PER_BAND],
        };
        let mut w = Sv7BitWriter::new();
        assert_eq!(
            encode_sv7_band_samples(&mut w, &band),
            Err(Error::UnsupportedBandType(3)),
        );
    }
}
