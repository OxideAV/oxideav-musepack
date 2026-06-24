# oxideav-musepack

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

The codec is **not yet wired into the `oxideav-core` registry** and
cannot decode a full stream end-to-end. The crate today is a set of
verified building-block modules with extensive unit-test coverage
(~517 lib tests), now including the §2.6 synthesis filterbank that
produces PCM from the reconstructed subband matrix. Remaining gaps are
tracked in `CHANGELOG.md` `[Unreleased]`.

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
  all 255 mirror pairs.
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
  The absolute SCF anchor, multi-channel composition, M/S undo, and the
  synthesis filterbank remain GAP.
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

- Absolute SCF anchor gain (the relative ladder + per-granule multiply
  are wired; the reference-index gain value is unspecified in the
  structural prose).
- The `SO` / `ST` packet payload field maps (the `SH` / `RG` / `EI`
  field maps are now wired — see `sh_header` / `rg_header` /
  `ei_header`; the `SO` seek-table-offset and `ST` seek-table layouts
  remain GAP in `spec/musepack-headers-and-coding.md` and are the next
  pick).
- **M/S undo arithmetic** — the §2.6 M/S-undo *structure* is now wired
  in `ms_stereo` (`undo_ms_stereo` gates each subband on its `msflag`,
  pairs the two channels' rows, passes L/R rows through unchanged), but
  the exact per-sample channel arithmetic (whether `L = M + S` /
  `R = M − S`, and any 0.5 / √2 normalisation) is **not specified
  anywhere under `docs/audio/musepack/`** and is threaded as a
  caller-supplied closure (the crate's GAP-knob pattern). The closure
  is the one edit that pins it once a docs trace lands. DOCS-GAP.
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
  Output is still **relative** loudness (the absolute SCF anchor below),
  and the stream-level decode loop (header → per-frame decode/recon →
  M/S undo → this filterbank → interleaved output) is the next
  integration step.

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
