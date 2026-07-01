//! Pure-Rust Musepack audio codec.
//!
//! **Clean-room rebuild in progress** (orphan `master` post the
//! 2026-05-06 docs audit). The crate is being grown back up against
//! the staged structural spec at
//! `docs/audio/musepack/musepack-sv7-sv8-spec.md` plus the numeric
//! tables under `docs/audio/musepack/tables/` (CSV + `.meta`
//! sidecars, extracted under the *Feist v. Rural* (1991) facts-only
//! exception by a walled extraction round — see
//! `docs/audio/musepack/provenance/01-musepack-table-extraction.md`).
//!
//! ## Format outline (overview-level)
//!
//! Musepack ships in two incompatible stream-format generations:
//!
//! - **SV7** (Stream Version 7, aka *MPEGplus / MP+*, c. 1997-2005):
//!   subband filter inherited from MPEG-1 Layer 2 (32-band polyphase)
//!   plus replaced bit-allocation, quantisation, and Huffman coding.
//!   Filename `.mpc` or legacy `.mp+`.
//! - **SV8** (c. 2008-): different bitstream packaging (KEY / SIZE /
//!   PAYLOAD packets, magic `MPCK`) and updated entropy coding.
//!   Same subband filter and psychoacoustic model as SV7; the upgrade
//!   is mainly in container framing, gapless-playback metadata, and
//!   chapter support.
//!
//! Both targets are ReplayGain-tagged by default. Stream-format level
//! 3 (8 channels) is supported in principle though almost never used.
//!
//! ## Module surface so far
//!
//! - [`requant`] — SV7 §2.5 / §2.6 requantiser constants:
//!   `RES_BITS[18]`, `QUANTIZER_OFFSET_D[19]`,
//!   `DEQUANT_COEFFICIENT_C[19]`, and `SCF_STEP_RATIO`.
//! - [`framing`] — SV7 / SV8 stream-magic identification and the
//!   SV8 packet outer-frame walker (key + varint size).
//! - [`huffman`] — SV7 `mpc_huffman`-shape entropy tables
//!   (`sv7-huffman-bandtype-header` / `sv7-huffman-scfi` /
//!   `sv7-huffman-dscf` / `sv7-huffman-q{1..=7}`) plus a
//!   left-justified-code linear decoder and an MSB-first bit
//!   reader. The `[2][N]` quantiser tables are exposed both as the
//!   full concatenated array and as per-context slices.
//! - [`cns`] — CNS / noise-substitution two-LFSR PRNG and the
//!   256-byte parity-of-popcount lookup that drives it
//!   (`cns-prng-parity` + `cns-prng-params`).
//! - [`sv7_band_decode`] — SV7 §2.5 per-band sample-decode. A
//!   classifier enum ([`sv7_band_decode::BandDecodeCase`]) covers
//!   every §2.5 case; per-arm decoders cover CNS (`-1`), empty
//!   (`0`), grouped (`1` / `2`), Huffman-per-sample (`3..=7`), and
//!   linear-PCM escape (`8..=17`); and the unified entry point
//!   [`sv7_band_decode::decode_sv7_band`] walks the §2.5
//!   `switch (band_type)` ladder end to end from `band_type` alone,
//!   routing each arm to its decoder and unifying them on an
//!   `[i32; 36]` output (the SV7 sibling of
//!   [`sv8_band_decode::decode_sv8_band`]). `band_type` outside
//!   `-1..=17` fails loud rather than silently zeroing the band.
//! - [`reconstruct`] — SV7 §2.6 per-sample reconstruction
//!   primitives (centring of PCM-escape raw levels by subtracting
//!   `D`; per-band dequant multiply by `C / 65536`; CNS dequant
//!   path keyed off `DEQUANT_COEFFICIENT_C[0]`) plus the §2.6
//!   *relative* scalefactor gain ladder
//!   ([`reconstruct::scf_relative_gain`] /
//!   [`reconstruct::scf_gain_relative_to_anchor`] /
//!   [`reconstruct::apply_scf_relative`]) — the anchor-independent
//!   geometric `SCF_STEP_RATIO^(Δindex)` part of the SCF multiply
//!   over the 256-index ladder. The *absolute* anchored gain table
//!   (its reference-index gain is DOCS-GAP), the M/S undo, and the
//!   synthesis filterbank are out of scope here.
//! - [`scf`] — SV7 §2.4 SCF coding-method decoder: reads the
//!   per-non-zero-band SCFI selector VLC, classifies it into a
//!   granule-coverage schedule (mirroring Layer-II SCFSI per §1
//!   lines 79-82), then reads N DSCF deltas and reconstructs the
//!   three per-granule SCF indices given a per-band base anchor.
//! - [`sv7_band_header`] — SV7 §2.3 per-band header loop walker:
//!   reads the `band_type` Huffman VLC per channel (stereo: left
//!   first, then right) and the conditional 1-bit `msflag` that
//!   follows iff at least one channel's `band_type` is non-zero,
//!   over `0..=max_band`. Returns a `BandHeader { band_type:
//!   [RawBandTypeVlc; 2], ms_flag: Option<bool> }` sequence. The
//!   raw VLC value is wrapped in [`sv7_band_header::RawBandTypeVlc`]
//!   to keep the §2.3-VLC-symbol → §2.5-dispatcher-case remap
//!   honest. The staged §5.1 closes that remap GAP:
//!   [`sv7_band_header::decode_res_header_grounded`] reads the
//!   per-channel `Res` (= band_type) **delta chain** directly — band 0 a
//!   raw 4-bit absolute, later bands a header-VLC delta off the same
//!   channel's previous `Res` with a `idx == 4` raw-4-bit escape, and a
//!   per-band M/S bit gated on the stream-wide M/S flag — returning
//!   [`sv7_band_header::Sv7ResBand`] values ready for the §5.4 sample
//!   switch with no further remap.
//! - [`sv7_scf_decode`] — SV7 §5.3 **grounded** scalefactor decode: the
//!   precise SCFI-case model the staged `headers-and-coding` §5.3 pins,
//!   distinct from the simpler [`scf`] Layer-II-schedule path.
//!   [`sv7_scf_decode::decode_sv7_band_scf`] reads the SCFI selector then
//!   `1..=3` DSCF indices where `SCF[0]` is always coded (Δ vs the
//!   *previous band's* `SCF[2]`, threaded via
//!   [`sv7_scf_decode::Sv7BandScf::last_index`]) and `SCF[1]`/`SCF[2]`
//!   are each coded-off-the-preceding-index or copied per the §5.3 table.
//!   The §5.3 `idx == 8` raw-6-bit absolute escape
//!   ([`sv7_scf_decode::DSCF_ESCAPE_SYMBOL`]) applies to every coded
//!   index, and the §5.3 "index > 1024 ⇒ sentinel" clamp is surfaced as
//!   [`sv7_scf_decode::Sv7BandScf::clamped`].
//! - [`sv8_band_decode`] — SV8 §3.4 per-band sample-decode case
//!   classifier mirroring [`sv7_band_decode::BandDecodeCase`] for
//!   the SV8 ladder shape (`Cns` / `Empty` / `SparseBand` /
//!   `Grouped3` / `Grouped2` / `ContextHuffmanPerSample` /
//!   `LargeCoeffEscape` / `OutOfRange`). Pure structural dispatch:
//!   one `const fn` plus two predicate helpers
//!   ([`sv8_band_decode::case_emits_samples`],
//!   [`sv8_band_decode::case_uses_first_order_context`]) routing
//!   `band_type` to its §3.4 `switch` arm, plus the
//!   classifier-driven entry point
//!   [`sv8_band_decode::decode_sv8_band`] that walks a band from its
//!   `band_type` alone to the matching per-arm decoder, unifying the
//!   grounded arms (CNS / empty / grouped3 / grouped2 / context /
//!   escape) on an `[i32; 36]` output and failing loud on the
//!   DOCS-GAP sparse band (case 1) and the out-of-range catch-all.
//!   The per-case sample decoders live in [`sv8_sample_decode`]
//!   downstream of the SV8 canonical-Huffman entropy layer
//!   (`sv8-canonical-*` + `sv8-symbols-*` tables, staged under
//!   `docs/audio/musepack/tables/`).
//! - [`sv8_sample_decode`] — SV8 §3.4 per-case sample decoders for
//!   the grounded subset of the ladder:
//!   [`sv8_sample_decode::decode_sv8_grouped3_band`] (case 2 — 12
//!   codewords, base-5-packed triplets over `-2..=2`),
//!   [`sv8_sample_decode::decode_sv8_grouped2_band`] (cases 3..=4 —
//!   18 codewords, signed-nibble pairs over `±band_type`),
//!   [`sv8_sample_decode::decode_sv8_context_band`] (cases 5..=8 —
//!   one VLC per sample, table chosen per previous sample through a
//!   caller-supplied context rule, the §3.4 GAP knob), and
//!   [`sv8_sample_decode::decode_sv8_escape_band`] (default arm,
//!   `band_type` 9..=17 — one VLC plus `band_type - 9` raw bits), and
//!   [`sv8_sample_decode::decode_sv8_sparse_band`] (case 1 — two
//!   halves of 18, a `sv8-canonical-q1` non-zero count per half, a
//!   §6.5 enumerative position-selection codeword, and one sign bit
//!   per present `±1` sample). Every SV8 §3.4 sample-decode arm is now
//!   wired.
//! - [`packet_stream`] — SV8 §3.1/§3.2 packet-stream walker on top
//!   of [`framing::parse_packet_header`]. `PacketStream::new` takes
//!   the post-`MPCK` slice plus a [`packet_stream::PacketSizeConvention`]
//!   pick (the GAP varint convention) and yields one
//!   [`packet_stream::PacketRef`] per call until the `SE`
//!   terminator. Payload bytes are surfaced as opaque borrows
//!   over the input slice — the per-payload field maps (`SH` /
//!   `RG` / `EI` / `SO` / `ST`) remain GAP per §3.2.
//! - [`typed_packet`] — typed §3.2 packet surface: each known
//!   2-byte key maps to a per-kind borrowed newtype
//!   ([`typed_packet::StreamHeaderPacket`] / `ReplayGainPacket` /
//!   `EncoderInfoPacket` / `SeekTableOffsetPacket` /
//!   `SeekTablePacket` / `AudioPacket` / `StreamEndPacket`), all
//!   wrapped in a [`typed_packet::TypedPacket`] sum that callers can
//!   `match` instead of re-validating raw `PacketKey` strings.
//!   Payload bytes remain opaque borrows over the input — field
//!   maps continue to be GAP per §3.2.
//! - [`stream_shape`] — SV8 stream-shape observer: walks a complete
//!   `MPCK`-prefixed byte buffer via [`framing::parse_sv8_magic`] +
//!   [`packet_stream::PacketStream`] + [`typed_packet::TypedPacket`]
//!   and surfaces a [`stream_shape::StreamShape`] summary of
//!   per-§3.2-kind counts, cumulative opaque payload bytes, and
//!   first/last seen packet kinds. Pure observer — no payload
//!   interpretation, no ordering enforcement.
//! - [`sv8_huffman`] — SV8 §3.4 / §3.5 canonical Huffman
//!   length-tables and paired int8 symbol maps wired as typed
//!   statics. Exposes 21 [`sv8_huffman::Sv8CanonicalTable`] views
//!   (`Bands`, `Res-{1,2}`, `Scfi-{1,2}`, `Dscf-{1,2}`, `Q1`,
//!   `Q2-{1,2}`, `Q3`, `Q4`, `Q5-{1,2}`..`Q8-{1,2}`, `Q9up`) plus
//!   a [`sv8_huffman::Sv8TableRole`] enum + first-order context
//!   dispatcher [`sv8_huffman::table_for_role`], plus the
//!   cumulative-index → symbol-index canonical-Huffman decode walk
//!   ([`sv8_huffman::Sv8CanonicalTable::decode`]) derived from the
//!   staged numeric facts (the per-row sub-index arithmetic
//!   `index = (cum_index − (peek16 >> (16 − length))) mod 256`,
//!   proven to tile every staged symbol map bijectively over all
//!   2^16 peeks) — see the module-level docs for the derivation.
//! - [`sv8_band_header`] — SV8 §3.4 frame-body band-resolution
//!   header walk: [`sv8_band_header::decode_used_subbands`] reads the
//!   `sv8-canonical-bands` VLC into a used-subbands count
//!   (`0..=`[`sv8_band_header::SV8_MAX_USED_SUBBANDS`]`, the §1
//!   Layer-II 32-subband bound), and
//!   [`sv8_band_header::decode_band_resolutions`] walks that many
//!   bands reading one `sv8-canonical-res-{1,2}` VLC each (the
//!   context-pair pick is the §3.4 GAP, threaded as a caller-supplied
//!   `ctx_for_prev_res` closure mirroring
//!   [`sv8_sample_decode::decode_sv8_context_band`]). Each raw res
//!   value is wrapped in [`sv8_band_header::RawResVlc`] to keep the
//!   GAP `res`-symbol (`0..=16`) → §3.4 `band_type` (`-1..=17`) remap
//!   honest — the SV8 sibling of
//!   [`sv7_band_header::RawBandTypeVlc`].
//! - [`sv8_scf_header`] / [`sv8_dscf_loop`] — SV8 §3.5 scalefactor
//!   layer: [`sv8_scf_header::decode_scfi_selectors`] reads the
//!   per-band SCFI selector VLC, then
//!   [`sv8_dscf_loop::decode_dscf_deltas`] reads the per-band DSCF
//!   deltas (1..=3 per band, count caller-supplied since the SV8 SCFI
//!   → granule schedule is GAP). Each raw value is wrapped in
//!   [`sv8_scf_header::RawScfiVlc`] / [`sv8_dscf_loop::RawDscfVlc`] to
//!   keep the GAP SCFI-value → schedule and DSCF-symbol → signed-delta
//!   centring mappings honest.
//!
//! - [`sv8_frame_decode`] — SV8 single-channel audio-packet frame-body
//!   assembler. [`sv8_frame_decode::decode_sv8_frame_channel`] joins the
//!   grounded SV8 sub-walks in the documented frame-body phase order: a
//!   §6.2 resolution sweep ([`sv8_band_header::decode_band_resolutions_grounded`]),
//!   then per non-zero band a §6.3 SCFI decode
//!   ([`sv8_scf_header::decode_sv8_scfi`]), the §6.3 per-granule SCF-index
//!   reconstruction ([`sv8_dscf_loop::decode_sv8_band_scf`], threading the
//!   previous band's `SCF[2]` forward), and the §3.4 sample decode
//!   ([`sv8_band_decode::decode_sv8_band_grounded`]). Empty (`band_type
//!   0`) bands emit a silent record; CNS (`band_type -1`) bands fill from
//!   the shared PRNG with no SCF layer. The output is a per-coded-subband
//!   [`sv8_frame_decode::Sv8BandDecode`] sequence — the structured input
//!   the §2.6 / §3.6 reconstruction (dequant + per-granule SCF multiply +
//!   synthesis filterbank) consumes. Multi-channel interleaving, the M/S
//!   undo, and the cross-phase SCF/sample ordering remain GAP.
//!
//! - [`sv7_frame_decode`] — SV7 single-channel frame-body assembler, the
//!   SV7 counterpart of [`sv8_frame_decode`].
//!   [`sv7_frame_decode::decode_sv7_frame_channel`] takes one channel's
//!   §5.1 `Res` (band_type) sequence and walks each band in the §5
//!   phase order: empty (`0`) ⇒ silent (no record); CNS (`-1`) ⇒ 36 PRNG
//!   samples, no SCF; coded (`1..=17`) ⇒ the §5.3 SCF decode
//!   ([`sv7_scf_decode::decode_sv7_band_scf`], threading the previous
//!   band's `SCF[2]`), the §5.4 **1-bit context selector** (read only for
//!   the grouped / per-sample-Huffman cases, gated by
//!   [`sv7_frame_decode::band_type_uses_context_selector`]), then the 36
//!   sample levels ([`sv7_band_decode::decode_sv7_band`]). Output is a
//!   [`frame_reconstruct::BandLevels`] sequence ready for
//!   [`frame_reconstruct::reconstruct_frame_channel`]. Cross-channel
//!   interleaving, the M/S undo, and the absolute SCF anchor remain GAP.
//!
//! - [`synthesis`] — the §2.6 final step: the 32-band polyphase
//!   **synthesis subband filter** inherited from MPEG-1 Layer I/II
//!   (spec §1 lines 55-66). [`synthesis::SynthesisFilter`] holds the
//!   persistent 1024-entry `V` FIFO and runs ISO 11172-3 Figure 3-A.2's
//!   five-step reconstruction (shift / matrix / build-U / window / sum)
//!   per time slot, turning 32 subband samples into 32 PCM samples;
//!   [`synthesis::synthesize_frame_channel`] drives it column-by-column
//!   over a [`frame_reconstruct::SubbandMatrix`] for the 1152 PCM
//!   samples of one channel-frame. The window coefficients
//!   ([`synthesis::SYNTHESIS_WINDOW`], ISO Table 3-B.3) are transcribed
//!   from the in-repo ISO PDF page renders under `docs/audio/mp3/`; the
//!   matrixing coefficient ([`synthesis::matrix_coefficient`]) is the
//!   closed-form `cos[(16+i)(2k+1)π/64]` the figure gives.
//!
//! Per-field header decoding (including the per-band SCF anchor
//! the [`scf`] module currently takes as an argument), the SV7
//! per-frame 20-bit length prefix + "read in 32-LSB units"
//! packing, the GAP `res`/`band_type` remap and res-context
//! selection rule the [`sv8_band_header`] walker threads as caller
//! knobs, the SCF index → gain anchor for §2.6, and the synthesis
//! filterbank are still pending. See `CHANGELOG.md` `[Unreleased]`
//! for the gap list.

#![forbid(unsafe_code)]

pub mod cns;
pub mod ei_header;
pub mod frame_reconstruct;
pub mod framing;
pub mod huffman;
pub mod ms_stereo;
pub mod packet_stream;
pub mod reconstruct;
pub mod requant;
pub mod rg_header;
pub mod scf;
pub mod sh_header;
pub mod stream_shape;
pub mod sv7_band_decode;
pub mod sv7_band_header;
pub mod sv7_bitwriter;
pub mod sv7_frame_decode;
pub mod sv7_header;
pub mod sv7_huffman_encode;
pub mod sv7_scf_decode;
pub mod sv7_stereo_frame;
pub mod sv7_stream;
pub mod sv7_word_swap;
pub mod sv8_band_decode;
pub mod sv8_band_header;
pub mod sv8_context;
pub mod sv8_decode;
pub mod sv8_dscf_loop;
pub mod sv8_frame_decode;
pub mod sv8_huffman;
pub mod sv8_reconstruct;
pub mod sv8_sample_decode;
pub mod sv8_scf_header;
pub mod sv8_stream;
pub mod synthesis;
pub mod typed_packet;

/// Total subband samples per frame per channel, inherited from
/// MPEG-1 Layer II (32 polyphase subbands × 36 samples per band).
///
/// Per `docs/audio/musepack/musepack-sv7-sv8-spec.md` §1 lines 65-71
/// ("One frame contains 36 × 32 = 1152 subband samples") this value
/// is identical for SV7 and SV8 — only the entropy / framing layer
/// differs between the two stream versions; the underlying sample
/// geometry is shared.
pub const SAMPLES_PER_FRAME_PER_CHANNEL: usize = 1152;

/// Crate-local error type. Concrete variants land as the Implementer
/// rounds populate the codec pipeline.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Error {
    /// Reserved placeholder. Replaced by real variants in round 1.
    NotImplemented,
    /// The input did not start with the expected stream magic
    /// (`MP+` for SV7 or `MPCK` for SV8).
    InvalidMagic,
    /// The input ran out before the requested item could be parsed.
    UnexpectedEof,
    /// The SV7 stream version byte's low nibble was not
    /// [`framing::SV7_VERSION_NIBBLE`]. The full version byte is
    /// included so a caller can log which version was rejected.
    UnsupportedVersion(u8),
    /// A varint kept its continuation bit set past the maximum
    /// supported byte length.
    VarintTooLong,
    /// The peeked 16-bit code window did not match any row of the
    /// supplied SV7 Huffman table — a malformed bitstream or a
    /// wrong-context table for the current sample.
    HuffmanNoMatch,
    /// A per-band sample-decode dispatcher was called with a
    /// `band_type` value that is either outside the structurally-
    /// documented range or in a case that is not yet wired
    /// (currently SV7 §2.5 cases 1 / 2 — grouped codewords — whose
    /// per-codeword sample-unpack convention is DOCS-GAP, plus an
    /// invalid `ctx` value for the cases that take one). The
    /// out-of-range value is reported so callers can log which
    /// `band_type` was rejected.
    UnsupportedBandType(i8),
    /// An SV8 §3.4 grouped-codeword unpack was handed a symbol
    /// outside the staged grouped alphabet (`0..=124` for the
    /// case-2 base-5 triplets; nibbles within `±band_type` for the
    /// case-3/4 signed-nibble pairs). Unreachable when the symbol
    /// comes from the staged `sv8-symbols-*` maps (whose confinement
    /// to the alphabet is test-proven); kept as a defensive bound
    /// for symbols sourced elsewhere. The offending symbol is
    /// reported for diagnostic logging.
    GroupedSymbolOutOfRange(i8),
    /// The SV7 §2.4 SCFI VLC decoded a value outside the
    /// structurally-documented `0..=3` range. The offending raw
    /// value is included for diagnostic logging.
    InvalidScfCodingMethod(i8),
    /// The §2.3 band-type header loop was driven with a `max_band`
    /// parameter above the Layer-II 32-subband heritage's inclusive
    /// upper bound (`SV7_MAX_BAND_INCLUSIVE == 31`). The offending
    /// value is included for diagnostic logging.
    MaxBandOutOfRange(u8),
    /// A per-band decoder was driven with a `nch` (channel count)
    /// other than 1 or 2. The offending value is included for
    /// diagnostic logging. Multi-channel streams (the SH-packet
    /// "level 3 = 8 channels" SV8 upgrade) need a separate decode
    /// path that is not wired this round.
    ChannelCountInvalid(u8),
    /// The SV8 `SH` stream-header packet declared a stream-version
    /// byte other than the required value 8
    /// (`spec/musepack-headers-and-coding.md` §2, field 2). The
    /// offending value is included for diagnostic logging.
    InvalidStreamVersion(u8),
    /// The SV8 `RG` (ReplayGain) packet declared a version byte other
    /// than the required value 1
    /// (`spec/musepack-headers-and-coding.md` §2). The offending value
    /// is included for diagnostic logging.
    InvalidReplayGainVersion(u8),
    /// The SV8 stream-level decode
    /// ([`sv8_decode::decode_sv8_mono_stream`]) encountered a `block_power`
    /// other than `0` (more than one frame per `AP` packet). The
    /// multi-frame-per-packet path is a DOCS-GAP (the per-frame
    /// `Max_used_Band` read position is not pinned cell-for-cell); the
    /// offending value is included for diagnostic logging.
    UnsupportedBlockPower(u8),
    /// An SV7 entropy **encoder** was asked to emit a symbol that has no
    /// codeword in the target `mpc_huffman` table (its value is outside
    /// the table's symbol alphabet). The offending symbol is reported.
    /// Distinct from [`Error::HuffmanNoMatch`] (a *decode*-side "no code
    /// matched the peeked bits"): this is the encode-side "no code
    /// exists for this symbol".
    SymbolNotEncodable(i32),
    /// An SV7 sample **encoder** was handed a level outside the range its
    /// band-type arm can represent (a grouped digit outside `-1..=1`
    /// (case 1) / `-2..=2` (case 2), or a linear-PCM-escape level that
    /// does not fit the arm's `band_type - 1` raw bits). The offending
    /// level is reported.
    SampleOutOfRange(i32),
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Error::NotImplemented => f.write_str(
                "oxideav-musepack: clean-room rebuild in progress — see crates/oxideav-musepack/README.md",
            ),
            Error::InvalidMagic => f.write_str(
                "oxideav-musepack: input does not start with the SV7 (MP+) or SV8 (MPCK) magic",
            ),
            Error::UnexpectedEof => {
                f.write_str("oxideav-musepack: unexpected end of input while parsing")
            }
            Error::UnsupportedVersion(byte) => write!(
                f,
                "oxideav-musepack: unsupported SV7 stream version (version byte {byte:#04x})",
            ),
            Error::VarintTooLong => f.write_str(
                "oxideav-musepack: varint exceeded the supported maximum byte length",
            ),
            Error::HuffmanNoMatch => f.write_str(
                "oxideav-musepack: no SV7 Huffman table entry matched the peeked code window",
            ),
            Error::UnsupportedBandType(bt) => write!(
                f,
                "oxideav-musepack: unsupported or out-of-range band_type {bt} for the sample-decode dispatcher",
            ),
            Error::GroupedSymbolOutOfRange(symbol) => write!(
                f,
                "oxideav-musepack: grouped-codeword symbol {symbol} lies outside the staged SV8 §3.4 grouped alphabet",
            ),
            Error::InvalidScfCodingMethod(raw) => write!(
                f,
                "oxideav-musepack: SCFI VLC produced value {raw} outside the spec §2.4 0..=3 range",
            ),
            Error::MaxBandOutOfRange(value) => write!(
                f,
                "oxideav-musepack: max_band {value} exceeds the spec §1 Layer-II 32-subband inclusive bound 31",
            ),
            Error::ChannelCountInvalid(nch) => write!(
                f,
                "oxideav-musepack: channel count {nch} is not 1 (mono) or 2 (stereo) at the §2.3 band-header layer",
            ),
            Error::InvalidStreamVersion(v) => write!(
                f,
                "oxideav-musepack: SV8 SH stream-version byte {v} is not the required value 8",
            ),
            Error::InvalidReplayGainVersion(v) => write!(
                f,
                "oxideav-musepack: SV8 RG packet version byte {v} is not the required value 1",
            ),
            Error::UnsupportedBlockPower(bp) => write!(
                f,
                "oxideav-musepack: SV8 block_power {bp} (>0, multi-frame AP) is not yet wired in the stream-level decode",
            ),
            Error::SymbolNotEncodable(sym) => write!(
                f,
                "oxideav-musepack: symbol {sym} has no codeword in the target SV7 mpc_huffman table",
            ),
            Error::SampleOutOfRange(level) => write!(
                f,
                "oxideav-musepack: sample level {level} is outside the range its SV7 band-type arm can encode",
            ),
        }
    }
}

impl std::error::Error for Error {}

/// Crate-local `Result` alias.
pub type Result<T> = core::result::Result<T, Error>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn error_display_points_at_readme() {
        let s = format!("{}", Error::NotImplemented);
        assert!(
            s.contains("clean-room rebuild"),
            "Error::NotImplemented Display should mention the clean-room rebuild status; got: {s}"
        );
        assert!(
            s.contains("README.md"),
            "Error::NotImplemented Display should point at the crate README; got: {s}"
        );
    }

    #[test]
    fn error_is_std_error() {
        // Compile-time check: Error implements std::error::Error.
        fn assert_error<E: std::error::Error>() {}
        assert_error::<Error>();
    }

    #[test]
    fn error_is_clone_and_eq() {
        let a = Error::NotImplemented;
        let b = a.clone();
        assert_eq!(a, b);
    }

    #[test]
    fn result_alias_resolves() {
        let ok: Result<u32> = Ok(7);
        let err: Result<u32> = Err(Error::NotImplemented);
        assert_eq!(ok, Ok(7));
        assert_eq!(err, Err(Error::NotImplemented));
    }

    #[test]
    fn samples_per_frame_per_channel_matches_layer_two_heritage() {
        // §1 lines 65-71: 32 subbands × 36 samples = 1152.
        assert_eq!(SAMPLES_PER_FRAME_PER_CHANNEL, 1152);
        assert_eq!(
            SAMPLES_PER_FRAME_PER_CHANNEL,
            sv7_band_header::SV7_SUBBAND_COUNT * sv7_band_decode::SAMPLES_PER_BAND,
        );
    }
}
