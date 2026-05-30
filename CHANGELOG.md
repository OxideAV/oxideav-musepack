# Changelog

All notable changes to this crate are documented in this file. The format
follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and
versioning follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

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
