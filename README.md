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
- `packet_stream` / `typed_packet` / `stream_shape` — SV8 packet-stream
  walker, per-kind typed packet views (`SH` / `RG` / `EI` / `SO` /
  `ST` / `AP` / `SE`), and a structural stream observer.
- `huffman` — SV7 entropy tables plus a left-justified-code linear
  decoder and an MSB-first bit reader.
- `sv8_huffman` — the 21 SV8 canonical-Huffman length tables + paired
  symbol maps, with the cumulative-index decoder walk.
- `requant` / `reconstruct` — SV7 requantiser constants and the
  per-sample reconstruction primitives (PCM-escape centring, dequant
  multiply, and the relative scalefactor gain ladder).
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
  per-band sample-decode dispatcher (CNS / empty / grouped /
  context-Huffman / large-coefficient escape arms), and scalefactor
  layer.

## Not yet wired (DOCS-GAP / downstream)

- Absolute SCF anchor gain (the relative ladder is wired; the
  reference-index gain value is unspecified in the structural prose).
- SV7 fixed-header field map, SV7 32-LSB word packing.
- SV8 sparse band (case 1), packet payload field maps, and the varint
  inclusive/exclusive convention.
- M/S undo + the 32-band polyphase synthesis filterbank.

## Codec category

This crate owns the **Musepack bitstream** only — SV7 frame layout and
SV8 packet structure (SV8's packet framing is intrinsic to the format).
Container-level concerns beyond the codec's intrinsic framing (e.g.
APE-tag parsing for ReplayGain metadata) route through the relevant
sibling crate.

## License

MIT — see `LICENSE`.
