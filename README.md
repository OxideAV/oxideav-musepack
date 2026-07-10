# oxideav-musepack

[![CI](https://github.com/OxideAV/oxideav-musepack/actions/workflows/ci.yml/badge.svg)](https://github.com/OxideAV/oxideav-musepack/actions/workflows/ci.yml) [![crates.io](https://img.shields.io/crates/v/oxideav-musepack.svg)](https://crates.io/crates/oxideav-musepack) [![docs.rs](https://docs.rs/oxideav-musepack/badge.svg)](https://docs.rs/oxideav-musepack) [![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

Pure-Rust Musepack (SV7 + SV8) audio codec for the
[oxideav](https://github.com/OxideAV/oxideav-workspace) framework.

## Status

**Clean-room rebuild in progress.** This `master` branch is a fresh
orphan; the prior implementation was retired alongside the 2026-05-06
docs audit, which found that the source-of-record trace document did
not satisfy clean-room separation. The crate is being grown back up
against the staged structural spec at
`docs/audio/musepack/musepack-sv7-sv8-spec.md` plus the numeric tables
under `docs/audio/musepack/tables/` (CSV + `.meta` sidecars, extracted
facts-only per the *Feist v. Rural* exception).

**Round 390: the SV7 decoder is externally validated and registered.**
The SV7 fixture corpus staged at `docs/audio/musepack/fixtures/`
(four independent mppenc 1.16 streams + FFmpeg `mpc7` PCM oracles,
imported under `tests/fixtures/sv7/`) pinned every fact the file layer
previously carried as a GAP knob, and several it had wrong:

- **wire framing** — every frame body is preceded by a **20-bit
  bit-length prefix**, and an **11-bit last-frame-samples trailer**
  (equal to header field 14) follows the final body; the decoder
  verifies every frame against its bit budget and fails loud;
- **frame-body syntax** — four sequential band-major/channel-minor
  passes (`Res` header → SCFI → DSCF → samples), not the whole-channel
  sweep previously implemented;
- **SCF[0] reference** — per-band per-channel memory (the same
  subband's previous-frame `SCF[2]`), not the within-frame chain;
- **absolute reconstruction** — `level × C[Res+1] ×
  SCF_STEP_RATIO^(scf−1)` directly in the s16 domain (anchor = unity
  at index 1);
- **M/S undo** — `L = M + S`, `R = M − S`, no normalisation.

Result: all four corpus streams decode end-to-end with every one of
their 72 frames bit-budget-exact and PCM within **±1 LSB of the FFmpeg
oracle** (~75–88 % bit-exact; the residue is the oracle's f32 DSP vs
this crate's f64 synthesis) — pinned as CI conformance gates
(`tests/sv7_corpus.rs`). The **encode side is wire-symmetric with
mppenc itself**: re-encoding the parsed structure of the corpus frames
reproduces the reference encoder's bytes exactly
(`tests/sv7_corpus_reencode.rs`). The decoder is wired into the
**`oxideav-core` registry** (`registry::register` /
`oxideav_core::register!`) with a directly-callable
`registry::make_decoder` factory. The SV8 grounded subset (mono,
keyframe `AP`) still decodes at relative loudness — no SV8 corpus
exists yet. ~670 lib tests + the corpus integration gates; remaining
gaps tracked in `CHANGELOG.md` `[Unreleased]`.

**Round 405: CNS / PNS validated on the wire.** The freshly staged
`cns-pns` fixture (mppenc 1.16 `--pns 1.0`; 215 noise-band instances
across 18 of 20 frames) exercises Clear Noise Substitution for the
first time. All 20 frames decode **bit-budget-exact**, proving the
r390 convention that `Res == -1` bands take part in the SCFI + DSCF
scalefactor passes (spec §5.2/§5.3 + erratum E1) while reading zero
sample-pass bits; frame 0 (CNS-free) matches the FFmpeg oracle within
±1 LSB, and the stream-level **PNS flag in the version byte**
(`MP+ 0x17`) is parsed/encoded end-to-end
(`Sv7HeaderFields::pns`). The noise-bearing frames are gated
**statistically** (global corr 0.776, `tests/sv7_cns_corpus.rs`): the
oracle's noise *waveform* is not reproducible from the staged
generator facts — matched-filter searches over the staged two-LFSR
stream (30 000 offsets × strides × groupings × reset hypotheses) find
nothing above the noise floor while self-validation spikes at the true
offset, and the oracle's noise residual is ~2× the staged
`C[0]`-scaled amplitude — so a per-sample noise comparison needs a
staged oracle whose CNS generator matches the spec's (reported as a
docs gap). The staged docs round (`0f1b6a2`) also folded the r390
empirical wire facts into the spec itself: §1.1 now documents the
20-bit prefix / four band-major passes / 11-bit trailer, and erratum
E1 records the temporal `SCF[0]` predictor — module docs now cite
those sections as source-of-record.

The crate now also grows an **SV7 bitstream encode side** (round 382):
a clean-room-invertible encoder for the SV7 frame body that round-trips
every decode path bit-for-bit against the readers/decoders already in
the crate. Encoding is the exact algebraic inverse of the documented §5
decode — no new format facts. See the `sv7_*_encode` modules below.

Round 385 built the **SV7 `.mpc` file layer** on both sides (round 390
corrected its wire framing against the corpus, above): a §1
fixed-header encoder, a whole-stream composer (`encode_sv7_file` + the
incremental `Sv7FileWriter`), a whole-file decoder (`decode_sv7_file`),
and a unified magic-dispatched entry (`mpc_decode::decode_mpc_stream`)
that routes `MP+` / `MPCK` buffers to the matching whole-stream
decoder. Write → decode round-trips are proven across every §5.4
band-type arm.

## Format outline

Musepack ships in two incompatible stream-format generations:

- **SV7** (a.k.a. MPEGplus / MP+, c. 1997–2005): a 32-band polyphase
  subband filter inherited from MPEG-1 Layer II, with replaced
  bit-allocation, quantisation, and Huffman coding. Files end `.mpc`
  (or legacy `.mp+`).
- **SV8** (c. 2008–): different bitstream packaging (KEY / SIZE /
  PAYLOAD packets, magic `MPCK`) and updated entropy coding, with the
  same subband filter and psychoacoustic model as SV7.

## Module surface

- `framing` — SV7 / SV8 stream-magic identification and the SV8 packet
  outer-frame walker (key + varint size).
- `sh_header` — SV8 `SH` (Stream Header) packet payload field-map
  decoder: CRC, stream version, total / beginning-silence sample
  counts (varint), sample-freq index → Hz, the −1-biased `max_band` and
  `channels` fields, stream M/S, and the block-power → frames-per-`AP`
  derivation (headers-and-coding §2). Surfaced as
  `StreamHeaderPacket::fields()`.
- `sv7_header` — SV7 (`MP+`) fixed-header field-map decoder (the SV7
  analogue of `sh_header`): all 17 fields (frame count, intensity / M/S
  flags, `max_band`, profile, link, sample-freq index, max-level, the
  ReplayGain title/album gain+peak quad, true-gapless + 11-bit
  last-frame samples, fast-seek, reserved, encoder version), the SV7
  32-bit-word byte-swap framing that the field reader runs over
  (headers-and-coding §1 / §4), the `1 ≤ max_band ≤ 31` sanity gate,
  and the stereo-only `channels` + `frame_count × 1152` total-sample
  derivations.
- `packet_stream` / `typed_packet` / `stream_shape` — SV8 packet-stream
  walker, per-kind typed packet views (`SH` / `RG` / `EI` / `SO` /
  `ST` / `AP` / `SE`), and a structural stream observer.
- `rg_header` / `ei_header` — SV8 `RG` (ReplayGain) and `EI` (Encoder
  Info) packet payload field-map decoders
  (`spec/musepack-headers-and-coding.md` §2). `RG` carries the version
  byte plus the title/album gain+peak quad (raw 16-bit, verbatim); `EI`
  carries the packed `profile×8` + PNS flag byte plus the three-byte
  encoder version (major / minor / build), with `profile()` /
  `profile_int()` / `version_word()` helpers. Surfaced as
  `ReplayGainPacket::fields()` and `EncoderInfoPacket::fields()`.
- `huffman` — SV7 entropy tables plus a left-justified-code linear
  decoder and an MSB-first bit reader.
- `sv8_huffman` — the 21 SV8 canonical-Huffman length tables + paired
  symbol maps, with the cumulative-index decoder walk.
- `requant` / `reconstruct` — SV7 requantiser constants and the §2.6
  reconstruction path: PCM-escape centring, the per-`band_type` dequant
  multiply, the relative scalefactor gain ladder, and the **per-granule
  SCF multiply** (each band's 36 samples are 3 granules of 12, each
  granule scaled by its own SCF index — the Layer-II SCFSI inheritance).
  `reconstruct_sv7_band_from_levels` is the integrating entry point that
  joins the §2.5 per-band sample decode to §2.6: it takes the unified
  `[i32; 36]` level buffer from `decode_sv7_band` and, branching on the
  band-type case so each arm's level convention (raw-unsigned PCM-escape
  vs already-centred Huffman vs CNS-PRNG) is centred/dequantised
  correctly, produces the reconstructed `f64` subband samples — relative
  to a caller-supplied SCF anchor (the absolute anchor is GAP), so
  granule-to-granule and anchor-sharing-band loudness is exact.
- `frame_reconstruct` — SV7 §2.6 frame-level reconstruction assembler:
  `reconstruct_frame_channel(bands, anchor)` composes the per-band
  `reconstruct::reconstruct_sv7_band_from_levels` over the Layer-II
  32-subband frame geometry (§1: 32 subbands × 36 samples = 1152 per
  channel), producing the per-channel `SubbandMatrix`
  (`[[f64; 36]; 32]`) — the structure the remaining §2.6 steps (M/S
  undo, then the synthesis filterbank) consume. Uncoded subbands
  reconstruct to silence (the §2.3 / §2.5 "data stored only for
  non-zero bands" convention). Pure composition — no new format facts;
  fail-loud on out-of-range subband / SCF-ladder index / band_type.
- `synthesis` — the §2.6 final step: the **32-band polyphase synthesis
  subband filter** that turns a per-channel reconstructed
  `frame_reconstruct::SubbandMatrix` into PCM. Inherited unchanged from
  MPEG-1 Layer I/II (§1); the algorithm + window table live in the
  in-repo ISO/IEC 11172-3 PDF under `docs/audio/mp3/`, a standards-body
  source the spec (§1, S3) authorises transcribing. `SynthesisFilter`
  carries the persistent 1024-entry `V` FIFO and runs ISO Figure 3-A.2's
  five steps (shift / matrix `N_ik = cos[(16+i)(2k+1)π/64]` / build-`U` /
  window by the 512-tap `D_i` of Table 3-B.3 / sum) per time slot;
  `synthesize_frame_channel` drives it column-by-column over the matrix
  for the 1152 PCM samples of one channel-frame. The 512 `D_i`
  coefficients (`SYNTHESIS_WINDOW`) are transcribed verbatim from the ISO
  Annex B page renders, guarded by a full magnitude-symmetry test over
  all 255 mirror pairs. `MultiChannelSynthesis` owns one persistent
  filter per channel (the filterbank's overlap spans the previous 15
  frames, so each channel's filter must be reused across all frames) and
  `synthesize_stereo_frame_interleaved` produces interleaved `2 × 1152`
  `L, R, …` PCM from the post-M/S-undo L/R subband matrices.
- `ms_stereo` — SV7/SV8 §2.6 mid/side stereo-undo *structure*:
  `undo_ms_stereo(stereo, ms_flags, undo)` walks a stereo pair of
  `SubbandMatrix` rows, transforming each `msflag`-set subband's
  (mid, side) rows via a caller-supplied `undo(m, s) -> (l, r)` closure
  and passing L/R subbands through. The per-sample mid/side →
  left/right arithmetic is a documented GAP (unspecified under
  `docs/audio/musepack/`) threaded as the closure knob, isolated for a
  one-edit pin once a trace lands.
- `scf` — SV7 SCF coding-method decoder (SCFI selector + DSCF deltas).
- `cns` — CNS / noise-substitution two-LFSR PRNG.
- `sv7_band_decode` / `sv7_band_header` — SV7 per-band header loop and
  sample-decode covering every §2.5 case: CNS, empty, grouped (base-3
  q1 triplets / base-5 q2 pairs), per-sample Huffman (Q3..Q7), and the
  linear-PCM escape ladder, all reachable through the unified
  `decode_sv7_band` dispatcher that walks the §2.5 `switch (band_type)`
  ladder end to end (the SV7 sibling of SV8's `decode_sv8_band`). The
  §5.1-grounded `decode_res_header_grounded` reads the per-channel `Res`
  delta chain directly (band-0 raw-4-bit, later-band header-VLC delta
  with the `idx == 4` raw-4-bit escape, stream-gated per-band M/S),
  closing the `RawBandTypeVlc → band_type` remap gap.
- `sv7_scf_decode` — SV7 §5.3 grounded scalefactor decode:
  `decode_sv7_band_scf` reads the SCFI selector then the per-band DSCF
  indices where `SCF[0]` is always coded (delta vs the previous band's
  `SCF[2]`) and `SCF[1]`/`SCF[2]` are coded-off-the-preceding-index or
  copied per the cell-for-cell §5.3 SCFI table, with the `idx == 8`
  raw-6-bit absolute escape and the `index > 1024` clamp flag.
- `sv7_frame_decode` — SV7 single-channel frame-body assembler (the SV7
  counterpart of `sv8_frame_decode`). `decode_sv7_frame_channel` takes a
  channel's §5.1 `Res` sequence and walks each band in the §5 phase
  order — silent empties, CNS PRNG bands (no SCF), and coded bands (§5.3
  SCF threaded off the previous band's `SCF[2]`, the §5.4 1-bit context
  selector for the grouped / per-sample-Huffman cases, then 36 sample
  levels) — emitting a `frame_reconstruct::BandLevels` sequence ready for
  `reconstruct_frame_channel`. Cross-channel interleaving, the M/S undo,
  and the absolute SCF anchor remain GAP.
- `sv8_frame_decode` — SV8 single-channel audio-packet frame-body
  assembler. `decode_sv8_frame_channel` joins the grounded SV8 sub-walks
  in the documented frame-body phase order (§2.3–§2.6): a §6.2
  resolution sweep, then per non-zero band a §6.3 SCFI decode, the §6.3
  per-granule SCF-index reconstruction (threading the previous band's
  `SCF[2]` forward), and the §3.4 sample decode. Empty bands emit a
  silent record; CNS bands fill from the shared PRNG with no SCF layer.
  Output is a per-coded-subband `Sv8BandDecode` sequence (subband index,
  `band_type`, three SCF indices, 36 sample levels) — the structured
  input the §2.6/§3.6 reconstruction (dequant + per-granule SCF multiply
  + synthesis filterbank) consumes. Multi-channel interleaving, the M/S
  undo, and the cross-phase SCF/sample ordering remain GAP.
- `sv8_reconstruct` — SV8 frame-decode → reconstructed subband-sample
  bridge (the SV8 counterpart of `frame_reconstruct`).
  `reconstruct_sv8_frame_channel` turns the `Vec<Sv8BandDecode>` of
  `decode_sv8_frame_channel` into a per-channel `SubbandMatrix` of `f64`
  samples (dequant by the SV7-shared quantiser + the three signed
  per-granule SCF gains relative to a caller anchor);
  `decode_and_reconstruct_sv8_channel` runs decode + reconstruct in one
  call straight from frame-body bits. A dedicated SV8 path because SV8
  levels are already-signed/centred for every arm (no SV7 PCM-escape
  centring) and SV8 SCF indices are signed (`−6..=121`, the §6.3 fold).
- `sv7_stereo_frame` — SV7 §5 **two-channel** frame decode +
  reconstruction. `decode_sv7_stereo_frame` composes the §5.1 shared
  band-type header (both channels + per-band M/S flags), the §5.3/§5.4
  "Left channel is decoded first, then right" per-channel body sweeps,
  and per-channel `reconstruct_frame_channel` into an `Sv7StereoFrame
  { channels: StereoSubbandMatrix, ms_flags }` — exactly the input
  `ms_stereo::undo_ms_stereo` + the synthesis filterbank consume. The
  shared CNS PRNG threads across both channels in decode order; the two
  §2.6 GAPs (absolute SCF anchor, M/S-undo arithmetic) stay caller knobs.
- `sv7_stream` — SV7 **stereo stream driver**. `Sv7StreamDecoder` owns
  the cross-frame state (one persistent `MultiChannelSynthesis` — the
  filterbank overlap spans 15 frames — a shared CNS PRNG, and the §2.6
  M/S-undo closure); `decode_frame` runs the full §2.6 per-frame pipeline
  (stereo decode + reconstruct → M/S undo → synthesis, interleaved L,R,…)
  over a caller-positioned `Sv7BitReader`; `decode_frames` loops it across
  the non-byte-aligned (§2.2) continuous bit run; `decode_body_bytes`
  takes a **raw** (non-swapped) body buffer and applies the §4
  `sv7_word_swap` internally so the caller need not hand-swap; and
  `from_header` constructs the driver straight from a parsed
  `sv7_header::Sv7HeaderFields` (`max_band` + M/S flag). The driver
  itself stays positioning-agnostic (body bytes or a positioned
  reader); the whole-file positioning (§1.1: the body run begins at
  bit 200 of the word-swapped stream) is owned by `sv7_file_decode`
  (round 385).
- `sv7_word_swap` — the §4 SV7 **32-bit-word body byte-swap**.
  `word_swap_sv7_body(raw)` turns a raw SV7 body buffer (the continuous
  bit run after the §1 fixed header) into the byte order the
  `huffman::Sv7BitReader` walks: each aligned 4-byte group is reversed
  (a little-endian 32-bit word re-serialised big-endian, so the
  MSB-first reader visits the word's bits high-to-low), the historic
  "read in 32-LSB units" packing. A partial trailing group zero-extends
  to a full word before reversal so every real body byte lands at its
  word-swapped position; `word_swap_sv7_body_in_place` is the
  allocation-free variant for already-word-aligned buffers. SV8 needs no
  analogue (§4: SV8 loads bytes in natural order). This is the transform
  that lets `sv7_stream::Sv7StreamDecoder::decode_body_bytes` take a raw
  body rather than a pre-swapped, hand-positioned reader. The SV7 header
  parser (`sv7_header`) shares this one definition (its private swap is
  now a test-only alias), so header and body word-swap are no longer
  duplicated.
- `sv7_bitwriter` / `sv7_huffman_encode` / `sv7_band_header_encode` /
  `sv7_scf_encode` / `sv7_sample_encode` / `sv7_frame_encode` /
  `sv7_stereo_frame_encode` — the **SV7 encode side** (round 382): the
  exact inverse of the SV7 decode path, each module round-tripped
  bit-for-bit against its decode counterpart. `Sv7BitWriter` is the
  MSB-first inverse of `huffman::Sv7BitReader`; `write_symbol` inverts
  `huffman::decode` (symbol → `mpc_huffman` codeword — valid because each
  staged table is a canonical prefix code); `encode_res_header_grounded`
  inverts the §5.1 `Res` header (band-0 raw + delta-or-escape later
  bands + gated M/S bit); `encode_sv7_band_scf` / `choose_scfi` invert
  the §5.2/§5.3 SCFI+DSCF layer (with sharing-maximal SCFI selection);
  `encode_sv7_band` inverts the §2.5 sample switch (base-3/base-5 grouped
  packs, per-sample Huffman, linear-PCM escape; CNS/empty emit no bits);
  `encode_sv7_frame_channel` composes the single-channel §5 body
  (SCF → context-selector → samples per coded band); and
  `encode_sv7_stereo_frame` composes the two-channel body (shared §5.1
  header + left-then-right bodies, CNS PRNG threading intact). The
  escape-vs-delta and SCFI *choices* are encoder policy; no new format
  facts. The full `.mpc` writer above these bodies landed in round 385
  (`sv7_header_encode` / `sv7_file_encode`).
- `sv7_header_encode` / `sv7_file_encode` / `sv7_file_decode` /
  `mpc_decode` — the **SV7 whole-file layer** (round 385).
  `sv7_header_encode` is the exact inverse of `sv7_header::parse`
  (the logical 200-bit header run, or the standalone 28-byte on-disk
  header; fail-loud per-field width validation via
  `Error::HeaderFieldOutOfRange`). `encode_sv7_file` composes header
  + the §1.1 continuous audio bit run (frame bodies back-to-back from
  bit 200, no per-frame length prefix) + the single §4 word-swap;
  `Sv7FileWriter` is the incremental push-frame builder
  (byte-identical output, auto `frame_count`, `finish_gapless` for §1
  fields 13/14). `decode_sv7_file` walks the whole file back to
  interleaved PCM (exactly `frame_count` frames, fail-loud on
  truncation, gapless trim via
  `Sv7HeaderFields::effective_total_samples`); and
  `decode_mpc_stream` dispatches a raw buffer by magic to the SV7 or
  SV8 whole-stream decoder. Self-decodable and spec-grounded
  (§1/§1.1/§4); byte-for-byte interop with externally-encoded files
  awaits a fixture corpus (none under `docs/audio/musepack/`).
- `sv8_stream` — SV8 **mono stream driver**. `Sv8MonoStreamDecoder` is
  the SV8 counterpart of `sv7_stream` for one channel: a persistent
  single-channel `SynthesisFilter` + shared CNS PRNG threaded across the
  `block_power`-derived frames of an `AP` packet; `decode_frame` runs §6
  decode + §2.6/§3.6 reconstruct + synthesis per frame, with the per-frame
  `Sv8FrameParams { nbands, new_block }` caller-supplied.
- `sv8_decode` — SV8 **packet-stream → audio integration** (first wiring
  of the packet layer to the audio decode). `decode_sv8_mono_stream`
  walks an `MPCK` buffer, reads the `SH` header, and drives an
  `Sv8MonoStreamDecoder` over every `AP` packet as one §6.2 key frame
  (reading its own `Max_used_Band` log code), emitting `Sv8DecodedStream
  { header, audio_packets, pcm }`. Supported subset is mono +
  `block_power == 0` + key-frame `AP`; out-of-subset streams are rejected
  with precise errors (`ChannelCountInvalid` / `UnsupportedBlockPower`).
  SV8 stereo + multi-frame-packet wait on the per-channel-interleaving +
  per-frame-`Max_used_Band` DOCS-GAPs. The SV7/SV8 multi-channel
  composition is now wired (SV7 stereo via `sv7_stream`); only the §2.6
  M/S-undo *arithmetic* and the absolute SCF anchor remain GAP.
- `sv8_band_decode` / `sv8_band_header` / `sv8_sample_decode` /
  `sv8_context` / `sv8_scf_header` / `sv8_dscf_loop` — SV8
  band-resolution walk, per-band sample-decode dispatcher (CNS / empty
  / **sparse** / grouped / context-Huffman / large-coefficient escape
  arms), and scalefactor layer. The sparse arm (§6.4.1) decodes each
  band as two halves of 18: a `sv8-canonical-q1` non-zero count per
  half, a §6.5 enumerative (combinatorial) position-selection codeword
  (binomial-coded, computed — no new tables), and one sign bit per
  present `±1` sample. Every SV8 §3.4 sample-decode arm is now wired.
  The §6.3 scalefactor layer is now **grounded** too:
  `sv8_scf_header::decode_sv8_scfi` picks the SCFI context by non-zero
  channel count and splits the packed value into L/R selectors
  (`left = value >> (2·cnt)`, `right = value & 3`), and
  `sv8_dscf_loop::decode_sv8_band_scf` reconstructs the three per-granule
  SCF indices — `SCF[0]` new-block raw7−6 or `dscf-2` delta (escape 64),
  `SCF[1]`/`SCF[2]` shared-or-`dscf-1`-delta (escape 31) — each folded
  `((prev−25+delta) & 127) − 6`. The legacy GAP-knob raw walks are
  retained.
- `sv8_band_header` — the §6.2 band-resolution outer walk is now
  **grounded** (was GAP-knobs). `decode_band_resolutions_grounded`
  decodes bands top-down: the top band reads res-1 (context 0), each
  lower band picks its context from "band-above `Res` > 2" (res-2 vs
  res-1) and folds `Res[n] = canon(Res, ctx) + Res[n+1]`, with the
  §6.2 signed wrap "values above 15 wrap by −17" (raw `16 → −1` CNS)
  applied to both the top value and every delta sum — closing the
  `RawResVlc → band_type` remap GAP and emitting signed `i8`
  band_types (ascending order) ready for `sv8_band_type_case`. The
  §6.2 `Max_used_Band` count is wired for both packet kinds:
  `decode_keyframe_max_used_band` (a §6.5 bounded "log" /
  truncated-binary code over `0..max_band+1`) and
  `decode_nonkey_max_used_band` (`last_max_band + canon(Bands)`, with
  the ">32 wraps by −33" fold). `decode_log_code` is the reusable §6.5
  bounded-log primitive (also serving M/S `cnt`). The §6.2 SV8 mid/side
  band-selection bitmap is wired too: `decode_sv8_ms_flags(reader, tot)`
  reads `cnt` (M/S band count) via the log code, then a §6.5
  enumerative codeword naming `min(cnt, tot−cnt)` of the `tot`
  non-zero-channel bands (complement-inverted when `cnt > tot/2`),
  returning a top-down `Vec<bool>` for the `ms_stereo` undo step.
- `sv8_context` — the SV8 §6.4.2 first-order context model, now
  grounded. `Sv8Context` is the running accumulator the context-adaptive
  sample arms use to pick their canonical-Huffman table half: per `Res`
  it inits `idx = 2 × thres[Res]` (thresholds `Res 2→3, 5→1, 6→3, 7→4,
  8→8`), selects context-1 when `idx > thres` else context-0, and folds
  `idx = (idx >> 1) + |q|` per sample (cases 5..=8) or
  `idx = (idx >> 1) + var[tmp]` per group (case 2), where `var[tmp]` is
  the summed magnitude of the three base-5 samples the product index
  `tmp` encodes (computed from §6.4.2/§5.5, no new table). This closes
  the two context-selection GAP-knobs `sv8_sample_decode` previously
  carried; `decode_sv8_context_band_grounded` /
  `decode_sv8_grouped3_band_grounded` / `decode_sv8_band_grounded` are
  the knob-free canonical decode paths (the closure-knob variants are
  retained for callers that need to override the predicate).

## Not yet wired (DOCS-GAP / downstream)

- **SV7 absolute SCF anchor — CLOSED (round 390, corpus-pinned):**
  `reconstruct::sv7_absolute_scf_gain` (unity at index 1, s16 domain).
  The **SV8** absolute anchor remains open (no SV8 corpus).
- **SV7 M/S undo arithmetic — CLOSED (round 390, corpus-pinned):**
  `ms_stereo::ms_to_lr` (`L = M + S`, `R = M − S`). The generic
  closure entry point remains for the SV8 path.
- **CNS scalefactor participation** — CNS bands (`Res == −1`) now read
  SCFI + DSCF like coded bands (the §5.2 "`Res ≠ 0`" gate + the
  structural spec's "noise scaled by the band's scalefactor"), but the
  corpus contains **no CNS bands**, so this is grounded-but-unvalidated;
  a wrong convention now fails the per-frame bit budget loudly. A
  CNS-bearing fixture (mppenc `--pns`?) would pin it.
- The `SO` / `ST` packet payload field maps (the `SH` / `RG` / `EI`
  field maps are now wired — see `sh_header` / `rg_header` /
  `ei_header`; the `SO` seek-table-offset and `ST` seek-table layouts
  remain GAP in `spec/musepack-headers-and-coding.md`).
- **32-band polyphase synthesis filterbank** — **WIRED** (round 366,
  `synthesis`). The reconstruction path now runs end-to-end to PCM for
  both generations: per-band decode → dequant + per-granule-SCF multiply
  → per-channel `frame_reconstruct::SubbandMatrix` (SV7 via
  `reconstruct_frame_channel`, SV8 via
  `sv8_reconstruct::reconstruct_sv8_frame_channel`) →
  `synthesis::synthesize_frame_channel` → 1152 PCM samples. The
  synthesis window `D_i` (ISO Table 3-B.3) and the `N_ik` matrixing
  formula were transcribed from the in-repo ISO 11172-3 PDF under
  `docs/audio/mp3/` — the spec (§1, source S3) authorises transcribing
  that repo-resident standards document, so this was never a docs-gap.
  Output is still **relative** loudness (the absolute SCF anchor below).
- **Stream-level decode loop** — **WIRED** (round 371). The full pipeline
  (header → per-frame decode/recon → M/S undo → filterbank → interleaved
  / mono output) now runs end-to-end for the grounded subset: SV7 stereo
  via `sv7_stream::Sv7StreamDecoder`, and SV8 mono keyframe streams via
  `sv8_decode::decode_sv8_mono_stream` → `sv8_stream::Sv8MonoStreamDecoder`,
  with the filterbank overlap + CNS PRNG threaded across frames. Out of
  scope still: the SV8 **stereo** path (per-channel band interleaving is
  GAP) and the SV8 **multi-frame `AP`** path (`block_power > 0` — the
  per-frame `Max_used_Band` position is GAP). The SV7 **whole-file
  path is closed and externally validated** (rounds 385 + 390): a raw
  `.mpc` buffer decodes end-to-end to s16-domain PCM over the
  corpus-pinned framing (20-bit per-frame prefixes, 11-bit trailer),
  `sv7_file_encode` writes the same layout (round-trip proven), and
  the four-fixture corpus gates hold every decoded sample within
  ±1 LSB of the FFmpeg oracle.

The SV8 sparse band (case 1) is now wired (see `sv8_sample_decode`),
and the SV8 packet-size varint convention is resolved as inclusive
(`spec/musepack-headers-and-coding.md` §3).

## Codec category

This crate owns the **Musepack bitstream** only — SV7 frame layout and
SV8 packet structure (SV8's packet framing is intrinsic to the format).
Container-level concerns beyond the codec's intrinsic framing (e.g.
APE-tag parsing for ReplayGain metadata) route through the relevant
sibling crate.

## License

MIT — see `LICENSE`.
