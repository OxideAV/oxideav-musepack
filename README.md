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
(~300 lib tests). Remaining gaps are tracked in `CHANGELOG.md`
`[Unreleased]`.

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
- `scf` — SV7 SCF coding-method decoder (SCFI selector + DSCF deltas).
- `cns` — CNS / noise-substitution two-LFSR PRNG.
- `sv7_band_decode` / `sv7_band_header` — SV7 per-band header loop and
  sample-decode covering every §2.5 case: CNS, empty, grouped (base-3
  q1 triplets / base-5 q2 pairs), per-sample Huffman (Q3..Q7), and the
  linear-PCM escape ladder, all reachable through the unified
  `decode_sv7_band` dispatcher that walks the §2.5 `switch (band_type)`
  ladder end to end (the SV7 sibling of SV8's `decode_sv8_band`).
- `sv8_band_decode` / `sv8_band_header` / `sv8_sample_decode` /
  `sv8_scf_header` / `sv8_dscf_loop` — SV8 band-resolution walk,
  per-band sample-decode dispatcher (CNS / empty / **sparse** /
  grouped / context-Huffman / large-coefficient escape arms), and
  scalefactor layer. The sparse arm (§6.4.1) decodes each band as two
  halves of 18: a `sv8-canonical-q1` non-zero count per half, a §6.5
  enumerative (combinatorial) position-selection codeword
  (binomial-coded, computed — no new tables), and one sign bit per
  present `±1` sample. Every SV8 §3.4 sample-decode arm is now wired.

## Not yet wired (DOCS-GAP / downstream)

- Absolute SCF anchor gain (the relative ladder + per-granule multiply
  are wired; the reference-index gain value is unspecified in the
  structural prose).
- The `SO` / `ST` packet payload field maps (the `SH` / `RG` / `EI`
  field maps are now wired — see `sh_header` / `rg_header` /
  `ei_header`; the `SO` seek-table-offset and `ST` seek-table layouts
  remain GAP in `spec/musepack-headers-and-coding.md` and are the next
  pick).
- **M/S undo** — §2.6 says "undo M/S where `msflag` set" but the exact
  channel arithmetic (whether `L = M + S` / `R = M − S`, and any 0.5 /
  √2 normalisation) is not specified anywhere under
  `docs/audio/musepack/`. DOCS-GAP.
- **32-band polyphase synthesis filterbank** — the reconstruction path
  now reaches the full per-channel dequantised, per-granule-SCF-scaled
  `f64` subband-sample matrix (`frame_reconstruct::SubbandMatrix`, via
  `reconstruct_frame_channel` composing the per-band
  `reconstruct_sv7_band_from_levels`). The final windowing step needs
  the Layer-II synthesis window `D_i` (Table 3-B.3) and the `N_ik`
  matrix, which §1 of the spec states live in the in-repo ISO
  11172-3 PDF under `docs/audio/mp3/` — outside this crate's
  `docs/audio/musepack/` source-of-truth scope. Staging those two
  tables (or their facts) under `docs/audio/musepack/tables/` would
  unblock the PCM step.

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
