# oxideav-musepack

Pure-Rust Musepack audio codec for the
[oxideav](https://github.com/OxideAV/oxideav-workspace) framework.

## Status

**Clean-room rebuild in progress.** This `master` branch is a fresh
orphan; the previous implementation was retired alongside the docs
audit dated 2026-05-06
([`AUDIT-2026-05-06.md`](https://github.com/OxideAV/docs/blob/master/AUDIT-2026-05-06.md)),
which found that the source-of-record trace document for this codec
was authored with a methodology that did not satisfy clean-room
separation. The prior history is preserved on the `old` branch for
archival but is forbidden input for the rebuild.

The `oxideav_core::CodecResolver` registration this crate will
expose through a future `register(ctx)` function is not wired yet;
the public API today surfaces the crate-local
`Error::NotImplemented` placeholder plus the new `requant` module
(see below).

## Docs status (round 191 — NEWLY UNBLOCKED)

A docs round closed `#927`, staging under `docs/audio/musepack/`:

- `musepack-sv7-sv8-spec.md` — a clean-room **structural / framing**
  spec authored in-repo from independent sources only (the
  multimedia.cx wiki snapshot, the Wikipedia article, the in-repo
  ISO/IEC 11172-3 Layer-II PDF, and general DSP knowledge). It
  establishes the Layer-II heritage, the SV7 frame container,
  band-type loop, SCF coding-method, per-band-type quantiser
  switch, SV8 KEY / SIZE / PAYLOAD packet vocabulary, and the SV8
  first-order-context quantiser switch.
- `tables/` — the SV7 / SV8 **entropy-coding, quantiser, CNS, and
  scalefactor numeric tables** extracted as Feist-v.-Rural-style
  facts (numeric initialisers only) by a walled extraction round
  documented in `provenance/01-musepack-table-extraction.md`. CSV
  + `.meta` sidecar per table, mirroring the
  `docs/audio/g729/tables/` convention.

## Round 191 — requantiser constants wired

Round 191 lands the `requant` module against the structural spec
§2.5 (frame body — quantised subband samples) and §2.6
(reconstruction), reading only `docs/audio/musepack/`:

- `RES_BITS: [u8; 18]` — bits per quantised sample per `band_type`
  in `0..=17`. Entropy-coded band types (`0..=7`) carry their
  sample width inside the Huffman codebook, so this array reports
  `0` for them; the linear-PCM escape ladder (`8..=17`) carries
  `band_type − 1` raw bits per sample.
- `QUANTIZER_OFFSET_D: [i16; 19]` — quantiser offset `D` per
  indexed band entry (number of distinct levels = `2 * D + 1`).
- `DEQUANT_COEFFICIENT_C: [f64; 19]` — dequant coefficient. The
  spec's relation `C = 65536 / (2 * D + 1)` is enforced as a unit
  test (max round-trip error `< 1e-6`) for all 18 non-CNS entries.
- `SCF_STEP_RATIO: f64` — geometric ratio between adjacent
  scalefactor-index gains (downward; upward is the reciprocal).
- `band_type_index(signed)` + `band_type_to_res_bits(unsigned)`
  helper functions.

12 unit tests cover length, spot-value, and the §2.6 product
relation; `cargo test` is green.

### Still gapped

- **SV7 header field map** (the 20-bit per-frame length prefix
  encoding details + fixed-header layout). The structural spec
  defers this to the project's Trac wiki page, which is link-only
  per the clean-room policy.
- **SV8 packet field map** for SH / RG / EI / SO / ST payload
  bodies (the packet KEY / SIZE varint frame is documented).
- **Huffman codebooks** — staged in `tables/` (SV7
  `sv7-huffman-*`, SV8 `sv8-canonical-*` + `sv8-symbols-*`) but
  not yet wired here.
- **CNS / noise-substitution generator** — staged in
  `tables/cns-prng-*` but not yet wired.
- **Frame driver + synthesis subband filter** — the ISO Layer-II
  filterbank tables live in `docs/audio/mp3/` and are reusable.

See `CHANGELOG.md` `[Unreleased]` for the per-round gap tracker.

## Codec category

Per the workspace's codec/container discipline, this crate owns the
**Musepack bitstream** only — SV7 frame layout and SV8 packet
structure (since SV8's packet framing is intrinsic to the format,
not a separate generic container the bitstream might-or-might-not
be carried in, akin to FLAC / MP3 / TTA / Shorten in the codecs-
with-dedicated-native-containers list). Container-level concerns
beyond the codec's intrinsic framing (e.g. APE-tag parsing for
ReplayGain metadata) route through the relevant sibling crate, not
here.

## Licence

MIT — see `LICENSE`.
