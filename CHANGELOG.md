# Changelog

All notable changes to this crate are documented in this file. The format
follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and
versioning follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- **Round 223** — `sv7_band_header` module wiring the SV7 §2.3
  per-band header loop on top of the round-197
  `SV7_BANDTYPE_HEADER_TABLE` Huffman table and the `Sv7BitReader`,
  reading only `docs/audio/musepack/`:
  - `SV7_SUBBAND_COUNT = 32` + `SV7_MAX_BAND_INCLUSIVE = 31`
    constants pinning the Layer-II 32-subband geometry inherited
    per spec §1 lines 53-71.
  - `RawBandTypeVlc(i8)` typed wrapper around the raw `i8` value
    produced by one invocation of the `sv7-huffman-bandtype-header`
    VLC. `from_raw(i8)` / `as_i8()` / `is_nonzero()` are the only
    accessors; the type intentionally does not expose arithmetic
    so the still-DOCS-GAP §2.3-VLC-symbol → §2.5-dispatcher-case
    remap cannot be implicitly composed with the
    `sv7_band_decode` dispatchers.
  - `BandHeader { band_type: [RawBandTypeVlc; 2], ms_flag:
    Option<bool> }` — one entry per band. `ms_flag` is `None`
    when §2.3's conditional suppressed the flag (both channels'
    `band_type == 0`) and `Some(true|false)` otherwise (true =
    M/S, false = L/R). `has_samples()` short-cuts the §2.5 inner
    loop's "for non-zero bands" predicate.
  - `decode_band_header(reader, nch) -> Result<BandHeader>` —
    one band's read: `nch` (1 or 2) bandtype-header VLCs followed
    by the conditional 1-bit `msflag`. Mono is treated as the
    single channel's VLC occupying both slots so the predicate
    fires the same way.
  - `decode_header_loop(reader, max_band, nch) ->
    Result<Vec<BandHeader>>` — the full §2.3 outer loop walking
    `i = 0..=max_band`, returning `max_band as usize + 1`
    entries.
  - New crate-level `Error::MaxBandOutOfRange(u8)` variant
    rejecting `max_band > 31`, and
    `Error::ChannelCountInvalid(u8)` variant rejecting `nch`
    outside `{1, 2}`.
- **Round 223** — 19 new unit tests across `sv7_band_header::tests`
  covering: the Layer-II 32-subband geometry constants;
  `RawBandTypeVlc` round-trip through `from_raw` + `is_nonzero`
  across `-5..=4`; `BandHeader::has_samples` across the four
  channel-zero-pattern combinations; `decode_band_header` for
  stereo both-zero (no msflag), stereo left-non-zero (msflag=1
  read), stereo right-non-zero (msflag=0 read), mono both-zero
  (no msflag), and mono non-zero (msflag consumed); rejection of
  `nch` outside `{1, 2}` (`0`, `3`, `8`, `255`);
  `UnexpectedEof` propagation in the left-VLC phase and the
  msflag phase; `decode_header_loop` rejection of `max_band > 31`
  (`32`, `200`); rejection of `nch` outside `{1, 2}` in the loop
  entry point; `max_band == 0` returning a single-band vector;
  a three-band stereo walk covering all three msflag outcomes;
  the maximally-wide stereo frame (`max_band == 31` → 32 bands,
  64 bits all-zero); and mid-loop `UnexpectedEof` propagation.
  Total crate test count `101 → 120`.

- **Round 214** — `scf` module wiring the SV7 §2.4 SCF
  coding-method decoder on top of the round-197 SCFI / DSCF
  Huffman tables, reading only `docs/audio/musepack/`:
  - `SCF_GRANULES_PER_BAND = 3` and `SCF_MAX_DISTINCT = 3`
    constants pinning the Layer-II-inherited band geometry
    (1152 samples / 32 bands / 12-sample granules; spec §1
    lines 65-71).
  - `ScfCodingMethod` typed wrapper around the `0..=3` SCFI
    selector value, with `from_raw(i8)` validating that the
    decoded SCFI VLC value is in range and
    `Error::InvalidScfCodingMethod(i8)` carrying the offending
    value on rejection.
  - `GranuleSchedule` exposing `deltas_to_read()` (number of
    distinct DSCFs the band transmits, `1..=3`) and
    `granule_to_slot()` (the granule → stored-delta-index
    mapping). The four schedules are transcribed verbatim from
    §1 lines 79-82 (the Layer-II SCFSI convention restated in
    the Musepack structural spec):
    - method 0: 3 SCFs, mapping `[0, 1, 2]`.
    - method 1: 2 SCFs, mapping `[0, 0, 1]`.
    - method 2: 1 SCF, mapping `[0, 0, 0]`.
    - method 3: 2 SCFs, mapping `[0, 1, 1]`.
  - `reconstruct_scf_from_deltas(reader, base, schedule) ->
    Result<[i32; 3]>` — reads `schedule.deltas_to_read()` DSCF
    VLCs, runs the §2.4 "delta-coded against the previous one"
    accumulation against the caller-supplied `base` anchor,
    and projects the running transmitted values through the
    granule mapping into the three per-granule SCF indices.
  - `decode_band_scf(reader, base) -> Result<BandScf>` — full
    per-band entry point: reads the SCFI VLC, classifies it,
    then drives the DSCF loop. Returns the recovered
    `ScfCodingMethod` alongside the three SCF indices.
  - New crate-level `Error::InvalidScfCodingMethod(i8)`
    variant. The base anchor is sourced upstream (SV7
    fixed-header field map, currently GAP per §2.1) and is not
    decided by this module.
- **Round 214** — 16 new unit tests across `scf::tests`
  covering: SCFI `0..=3` round-trip through `from_raw`;
  rejection of `-1`, `-2`, `4`, `5`, `7`, `i8::MAX`, `i8::MIN`;
  the four granule-schedule shapes against their §1 lines
  79-82 source rows; delta reconstruction for methods 0, 1,
  2, 3 driven through the real `Sv7BitReader` against
  hand-packed DSCF bit streams (covering single-shared,
  three-distinct running-sum, two-pair-mappings); end-to-end
  `decode_band_scf` for method 2 against a 16-bit packed
  stream; `UnexpectedEof` propagation in both the SCFI and
  the DSCF phase; the geometric invariant that every
  schedule's mapping references only valid transmitted slots;
  the constant `SCF_GRANULES_PER_BAND == 3`; and a zero-base
  reconstruction reducing the function to a pure running-sum
  walker. Total crate test count `85 → 101`.
- **Round 206** — `reconstruct` module wiring the SV7 §2.6
  per-sample reconstruction primitives on top of the round-191
  requantiser constants and the round-201 per-band level decode:
  - `DEQUANT_DIVISOR: f64 = 65536.0` constant tied to the
    requantiser relation `C = 65536 / (2D + 1)`.
  - `centre_pcm_level(band_type, raw_unsigned) -> Result<i32>` and
    `centre_pcm_band(band_type, &mut [i32; 36]) -> Result<()>` —
    PCM-escape centring (subtract `D = QUANTIZER_OFFSET_D[band_type
    + 1]`) for band_types 8..=17.
  - `dequantise_sample(band_type, centred_level) -> Result<f64>` —
    single-sample dequant via `centred_level * C / 65536`, covering
    both the CNS / noise band (`-1`) and the normal `0..=17`
    range.
  - `dequantise_band(band_type, &centred, &mut out) -> Result<()>` —
    whole-band variant for `0..=17`.
  - `dequantise_huffman_band(band_type, &huffman_i8, &mut out) ->
    Result<()>` — convenience wrapper accepting the `[i8; 36]`
    shape returned by `decode_huffman_band` for band_types 3..=7.
  - `dequantise_cns_band(&cns_levels, &mut out)` — CNS-specific
    wrapper keyed off `DEQUANT_COEFFICIENT_C[0]` (the
    `32768/2/255*sqrt(3)` anchor per the
    `cns-prng-params.meta` notes line).
  - `pcm_escape_d(band_type) -> Option<i32>` helper.
- **Round 206** — 18 new unit tests across `reconstruct::tests`
  covering: single-sample PCM-centring at band_types 8 and 17
  (boundary inputs `0`, `D`, `2D`), full-band in-place centring,
  out-of-range rejection for both centring functions;
  single-sample dequant for band_type 0 (identity scaling),
  band_type 3 (`d / (2d + 1)`), band_type 17 (`d / (2d + 1)`,
  large-`D` floating-point sanity), and the CNS band (`-1`); whole-
  band dequant against the single-sample path; Huffman-band
  dequant for band_type 3 with a signed i8 ramp; CNS dequant
  magnitude bound check against the PRNG's `-510..=510` range; the
  `pcm_escape_d` helper across the PCM-escape range; out-of-range
  rejection for whole-band paths; and a cross-module integration
  test that wires the PCM-escape Sv7BitReader, the round-201
  PCM-escape decoder, the round-206 centring step, and the
  round-206 dequant multiply end-to-end against a known synthetic
  input. Total crate test count `67 → 85`.

- **Round 201** — `sv7_band_decode` module wiring the SV7 §2.5
  per-band sample-decode `switch (band_type)` dispatch on top of the
  already-staged Huffman / CNS / requant tables:
  - `BandDecodeCase` enum classifies every spec case (`Cns`,
    `Empty`, `Grouped3`, `Grouped2`, `HuffmanPerSample`, `PcmEscape`,
    `OutOfRange`); `band_type_case(i8) -> BandDecodeCase` is a pure
    `const fn` dispatch.
  - `fill_zero_band(out)` — case 0, fills 36 zero samples.
  - `fill_cns_band(prng, out)` — case -1, passes through to the
    already-wired `CnsPrng::fill_samples`.
  - `decode_huffman_band(reader, band_type, ctx, out)` — cases
    3..=7, one Q`band_type` Huffman codeword per sample,
    context-selected via `sv7_q{3..=7}_ctx(ctx)`.
  - `decode_linear_pcm_band(reader, band_type, out)` — cases
    8..=17, `band_type - 1` unsigned bits per sample read MSB-first
    into an `[i32; 36]` raw-level buffer.
  - `SAMPLES_PER_BAND = 36` shared by all four decoders (Layer-II
    heritage per spec §1).
- **Round 201** — `Error::UnsupportedBandType(i8)` variant for the
  per-band-decode dispatcher's fail-loud channel: triggered by the
  structurally-documented-but-unimplemented grouped cases (1, 2)
  and by any out-of-range `band_type` or `ctx`.
- **Round 201** — 11 new unit tests across `sv7_band_decode::tests`
  covering: the classifier across `-2..=18` plus `i8` extremes; the
  zero / CNS fill paths (CNS round-trip vs a directly-driven PRNG
  walk with matching state); the Huffman path on band_type 3 (ctx
  0, shortest-code 36×) and band_type 7 (both contexts, signed-level
  range `-31..=31` invariant); the PCM-escape path on band_type 8
  (7 bits/sample, ramp round-trip) and band_type 17 (16 bits/sample,
  distinct-pattern round-trip); explicit `UnsupportedBandType`
  rejection edges for every dispatcher; EOF propagation through the
  PCM-escape reader. Total crate test count `56 → 67`.

- **Round 197** — `huffman` module wiring the SV7
  `mpc_huffman`-shape entropy tables staged under
  `docs/audio/musepack/tables/sv7-huffman-*.csv` through a
  `build.rs`-driven CSV-to-Rust generator. The script reads only
  the `.csv` numeric initialisers (the Feist facts of the format)
  and emits typed `Sv7Entry` arrays into `OUT_DIR`. Generated /
  exposed tables, all keyed by the staged `.meta`
  `resolved_dims`:
  - `SV7_BANDTYPE_HEADER_TABLE` (10 entries) — band-type / header
    VLC per spec §2.3.
  - `SV7_SCFI_TABLE` (4 entries) — SCF coding-method selector per
    spec §2.4.
  - `SV7_DSCF_TABLE` (16 entries) — delta-scalefactor VLC per spec
    §2.4.
  - `SV7_Q1_TABLE` .. `SV7_Q7_TABLE` (54 / 50 / 14 / 18 / 30 / 62
    / 126 entries) — per-quantiser sample VLCs per spec §2.5,
    each a `[2][N]` context-pair with the two contexts
    concatenated. A `sv7_q{1..=7}_ctx(ctx)` accessor returns the
    requested half-slice per the `.meta` notes line.
  - A `Sv7Entry { code, length, value }` struct mirroring the
    sidecar's `mpc_huffman = {Code:uint16 left-adjusted,
    Length:uint8, Value:int8}` shape, with the canonical-code
    `code <= peek16()` decode loop driving `huffman::decode`.
  - A standalone `Sv7BitReader<'a>` over an in-memory `&[u8]`
    slice, MSB-first, with `peek16`, `consume_bits(1..=32)`, and
    `read_bits(1..=16)` for the spec §2.5 case 8..=17 linear-PCM
    escape ladder. The SV7 per-frame 20-bit length prefix and the
    "read in 32-LSB units" word packing per spec §2.2 are *not*
    handled here — they belong to a later frame-driver round.
- **Round 197** — `cns` module wiring the noise-substitution
  generator from `docs/audio/musepack/tables/cns-prng-{parity,
  params}.csv` via the same `build.rs`. The 256-byte `PARITY`
  table is generated from `cns-prng-parity.csv`; the six scalar
  parameters (`R1_SEED` / `R2_SEED` / `R1_TAP_MASK` /
  `R2_TAP_MASK` / `R2_SHIFT` / `NOISE_SAMPLE_BYTE_SUM_BIAS`) are
  generated from `cns-prng-params.csv`. The `CnsPrng` two-LFSR
  state machine implements the generator step transcribed from
  the staged `.meta` `notes:` line: `r1 = (r1 >> 1) | (Parity[r1
  & 0xF5] << 31); r2 = (r2 << 1) | Parity[(r2 >> 25) & 0x63];
  word = r1 ^ r2; q = byte_sum(word) - 510`. The first step from
  the reset state is verified against a hand-cranked walk
  (`r1' = 0x8000_0000`, `r2' = 2`, `word = 0x8000_0002`, first
  sample = `-380`).
- **Round 197** — `Error::HuffmanNoMatch` variant covering the
  case where no row of the supplied SV7 Huffman table matches the
  peeked 16-bit code window.
- **Round 197** — 22 new unit tests across `huffman::tests` (10)
  and `cns::tests` (6) plus shape assertions on each of the 10
  Huffman tables and the parity table (each test pins the entry
  count from the `.meta` `resolved_dims` line plus the last
  entry of the CSV). Total crate test count `34 → 56`; `cargo
  build`, `cargo test`, `cargo clippy -- -D warnings` and `cargo
  fmt --check` all clean.

### Notes

- The SV8 canonical-Huffman entropy tables
  (`sv8-canonical-*` length tables + `sv8-symbols-*` symbol
  maps) and the SV7/SV8 quantiser/CNS-related constants beyond
  the four wired in round 191 (`requant-*`) remain staged but
  unwired this round — they need a length-table-to-code-table
  builder (canonical-Huffman) that's a different decoder shape
  from the SV7 `mpc_huffman` linear walk and is queued for a
  follow-up round.
- The `build.rs` accepts an `OXIDEAV_MUSEPACK_DOCS_DIR` env-var
  override pointing at the `docs/` root, so the crate can be
  built outside the umbrella checkout. For standalone / CI /
  crates.io builds the script falls back to a vendored snapshot
  of the consumed CSV+`.meta` subset shipped at
  `<crate>/tables/`; the snapshot must stay byte-equal with the
  umbrella's `docs/audio/musepack/tables/` and is refreshed
  manually whenever the docs collaborator restages.

- **Round 194** — `framing` module covering the parts of the
  Musepack container that are *structurally* specified by
  independent sources in `docs/audio/musepack/musepack-sv7-sv8-spec.md`:
  - `SV7_MAGIC = b"MP+"` + `SV7_VERSION_NIBBLE = 7` constants and
    a `SV7Header::parse_magic` recogniser (§2.1) that returns the
    version byte and a slice over the still-GAP rest of the fixed
    header without decoding it.
  - `SV8_MAGIC = b"MPCK"` constant + `parse_sv8_magic` recogniser
    (§3.1).
  - `PacketKey` enum covering the full §3.2 vocabulary
    (`SH` / `RG` / `EI` / `SO` / `ST` / `AP` / `SE`) plus an
    `Unknown([u8; 2])` catch-all, with `from_bytes` /
    `as_bytes` / `is_known` helpers.
  - `PacketHeader` + `parse_packet_header` walking the
    `[2-byte key][varint size][payload]` outer frame per §3.1,
    plus a `parse_varint` decoder for the continuation-bit
    big-endian shape described there. Because §3.1 flags the
    "varint inclusive of header bytes?" convention as GAP, the
    decoded header exposes both `payload_len_inclusive()` (returns
    `None` if the size is too small to cover the header) and
    `payload_len_exclusive()` so the caller can pick the right one
    once the observer trace lands.
  - `StreamKind` + `identify_stream` for magic-bytes-only
    dispatch between SV7 and SV8.
  - `Error` enum extended with `InvalidMagic`, `UnexpectedEof`,
    `UnsupportedVersion(u8)`, and `VarintTooLong` variants.
- **Round 194** — 22 new unit tests in `framing::tests` covering
  magic acceptance and rejection (both versions), the seven known
  packet keys + unknown round-trip, varint single / two / three-byte
  values + truncated and overlong rejection, packet header parsing
  for `SH` / `AP` / `SE`, and a synthetic SV8 stream walk
  reconstructing the packet sequence end-to-end. Total crate test
  count `12 → 34`, all green; clippy clean with
  `-D warnings`; `cargo fmt --check` clean.

### Notes

- The bit-precise field layouts **inside** the SV7 fixed header
  (sample count, intensity / MS flags, `max_band`, encoder
  profile / quality, gapless trailing-sample count, ReplayGain
  title / album gain + peak) and **inside** each SV8 SH / RG / EI
  / SO / ST payload are GAP per the structural spec — they live
  only in the project's walled Trac `SV7Specification` /
  `SV8Specification` pages. They are intentionally **not**
  implemented this round and are the target of the pending
  Musepack observer-trace round (workspace task #1263).

## [0.0.2](https://github.com/OxideAV/oxideav-musepack/releases/tag/v0.0.2) - 2026-05-30

### Other

- Round 191 — wire SV7/SV8 requantiser constants
- Round 186 — refresh docs-blocker assessment against 2026-05-25 staging
- Round 84 — file docs gap blocking SV7/SV8 header parse
- Round 0 — clean-room rebuild scaffold (orphan master)

### Added

- **Round 191** — `requant` module exposing the SV7 / SV8
  requantiser constants against
  `docs/audio/musepack/musepack-sv7-sv8-spec.md` §2.5 / §2.6:
  - `RES_BITS: [u8; 18]` — bits per quantised sample per
    `band_type` 0..=17 (0 for the entropy-coded ladder, 7..=16 for
    the linear-PCM escape ladder), transcribed from
    `docs/audio/musepack/tables/requant-res-bits.csv`.
  - `QUANTIZER_OFFSET_D: [i16; 19]` — offset `D` per indexed band
    entry (number of quantiser steps = `2 * D + 1`; index 0 = CNS /
    noise band entry), transcribed from
    `docs/audio/musepack/tables/requant-quantizer-offset-Dc.csv`.
  - `DEQUANT_COEFFICIENT_C: [f64; 19]` — dequant coefficient
    (`C = 65536 / (2 * D + 1)` for normal entries; index 0 carries
    the CNS / noise dequant constant), transcribed from
    `docs/audio/musepack/tables/requant-coefficient-Cc.csv`.
  - `SCF_STEP_RATIO: f64` — geometric ratio between adjacent
    scalefactor-index gains (downward direction), transcribed from
    `docs/audio/musepack/tables/scf-step-ratio.csv`.
  - `band_type_index` / `band_type_to_res_bits` helpers.
  - 8 unit tests exercising lengths, the §2.6 product relation
    `C * (2D + 1) == 65536` (max round-trip error `< 1e-6` across
    all 18 non-CNS entries), boundary values, and the helper
    mapping; total crate test count 12 lib tests, all green.

### Changed

- **Round 191** — README "Status" + "Docs status" sections rewritten
  against the docs-staging round that closed `#927`. The crate's
  module-level docs now point at the staged structural spec at
  `docs/audio/musepack/musepack-sv7-sv8-spec.md` and the
  Feist-extracted tables at `docs/audio/musepack/tables/` as the
  source-of-record for further work, with the project-shipped
  reference material remaining link-only / off-limits to
  Implementer rounds.

### Pending (next-round candidates)

- **SV7 header field map** — the 20-bit per-frame length-prefix
  encoding and the fixed-header layout (sample-frequency /
  max-band / max-level / title / VBR fields). The structural spec
  at `docs/audio/musepack/musepack-sv7-sv8-spec.md` §2.1 defers
  this to the project's Trac wiki page, which is link-only per the
  clean-room policy; a clean-room observer-trace round is the
  recommended unblock.
- **SV8 packet payload field maps** — the KEY / SIZE varint
  packet framing is documented in the structural spec §3, but
  the per-packet payload bodies (SH stream header, RG replaygain,
  EI encoder info, SO seek offset, ST seek table) need a
  clean-room observer-trace round to map.
- **Huffman codebooks** — fully staged under
  `docs/audio/musepack/tables/` (SV7 `sv7-huffman-*`, SV8
  `sv8-canonical-*` + `sv8-symbols-*`) but not yet wired into
  Rust modules.
- **CNS / noise-substitution generator** — staged under
  `docs/audio/musepack/tables/cns-prng-*` (LFSR taps / seeds +
  256-byte `Parity` table) but not yet wired.
- **Frame driver + 32-band polyphase synthesis filter** — the
  ISO/IEC 11172-3 Annex B Table 3-B.3 synthesis window and
  matrix `N_ik` live under `docs/audio/mp3/` and are reusable.
- **Encoder.** Out of scope for the early Implementer rounds.

### Historical

- **Round 0** — clean-room rebuild from a fresh orphan `master`;
  the previous implementation was retired by the OxideAV docs
  audit dated 2026-05-06.
- **Round 84** — targeted a foundational SV8 stream-header parser
  and confirmed `docs/audio/musepack/` then contained only the
  multimedia.cx wiki overview; deferred to a future docs round.
- **Round 186** — README refresh restating the docs-blocker against
  the 2026-05-25 staging (workspace docs commit `78e2487`).
- **Round 191** — this round. See "Added" / "Changed" above.
