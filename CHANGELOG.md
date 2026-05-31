# Changelog

All notable changes to this crate are documented in this file. The format
follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and
versioning follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

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
